# =============================================================================
# UCCNC 4TH AXIS GENERATOR
# Version 2.16 - MkII development; TAPER uses indexed no-trench ring roughing plus spiral finish
#
# Four-stage architecture:
#   TAB 1 - ROUND  : square stock -> known round cylinder
#   TAB 2 - WAVE   : inherited cylinder -> sinusoidal/cosine profile
#   TAB 3 - TAPER  : inherited cylinder -> linear taper (WAVE must be zero)
#   TAB 4 - GROOVE : inherited ROUND/WAVE/TAPER profile -> helical groove(s)
#
# MACHINE COORDINATE CONVENTION
#   * X0/Y0 = bottom-left of spoilboard viewed from above.
#   * +Y moves north/up the spoilboard.
#   * Z0 is the upper Z travel limit; downward motion is increasingly negative.
#   * All generated XYZ motion is explicitly G53 machine-coordinate motion.
#   * Rotary stock axis is physically parallel to Y but remains named A.
#   * UCCNC A-axis "Roll over on 360" must be OFF.
#   * Program end uses G92 A0.000, never a physical A unwind.
# =============================================================================

import math
import re
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

APP_TITLE = "UCCNC 4th Axis Generator v2.23"
DEFAULT_OUTPUT_DIR = r"C:\UCCNC"

# =============================================================================
# MkII MACHINE CONFIGURATION / INI-STYLE PARAMETERS
# Change these when the machine hardware or permanent rotary setup changes.
# =============================================================================
STOCK_SQUARE_MAX = 80.0          # mm; safe maximum square stock side for the new 120 mm rotating envelope
STOCK_LENGTH_MAX = 700.0         # mm; absolute maximum physical blank length that can be fitted
WASTE_CHUCK = 20.0               # mm; physical chuck-end sacrificial allowance (does not define machine Y minimum)
WASTE_TAIL = 20.0                # mm; required physical tail-end sacrificial stock beyond operator Cut Y Max

A_CENTER_X = 778.70              # machine X of permanently fixed A-axis centreline; calibrated from opposite-side probing of 12 mm test rod
CHUCK_FACE_MACHINE_Y = -70.0      # machine mm; fixed chuck face position, 70 mm before machine Y0

Y_MACHINE_MIN = 1.0              # mm; HARD minimum machine-coordinate Y permitted for rotary cutting/motion
Y_MACHINE_MAX = 600.0            # mm; HARD maximum machine-coordinate Y permitted for rotary cutting/motion
STOCK_Y_DIRECTION = 1            # MkII stock extends toward +Y from the chuck

PROBE1_BLOCK_HEIGHT = 40.000         # mm; physical single-wire Probe 1 block thickness
SPOILBOARD_TO_40120_TOP = 4.862      # mm; 40120 top is this far ABOVE the spoilboard
LATHE_CARRIAGE_TOP_TO_A_CENTER = 65.000  # mm; A centre is this far ABOVE the 40120 rotary carriage
PROBE1_RAW_TO_A_CENTER = (-PROBE1_BLOCK_HEIGHT + SPOILBOARD_TO_40120_TOP + LATHE_CARRIAGE_TOP_TO_A_CENTER)  # = +29.862 mm
RPM_MAX = 40.0                   # chuck RPM hard ceiling for every generated operation
Z_MIN_DIAMETER = 20.0
MIN_TAPER_DIAMETER = 20.0            # mm; smallest permitted finished/root diameter anywhere
A_SOFT_LIMIT_ABS = 3600000.0     # deg; UCCNC A-axis software limit, verified saved/rebooted on machine

# ---- Machining policy ----
Z_APPROACH_CLEARANCE = 2.0
SAFE_Z_EXTRA = 3.0
ROUND_SPIRAL_MAX_CLEANUP = 0.50  # radial mm; if 32-stage leaves more, add 64-stage
PROFILE_SAMPLE_MM = 1.0
WAVE_LOCAL_CLEARANCE = 2.0
TAPER_ROUGH_STEP_FACTOR = 0.80   # station spacing as fraction of flat-cutter diameter
GROOVE_EDGE_MARGIN_FACTOR = 0.60 # cutter-centre setback from each inherited profile end, as fraction of groove cutter diameter


def sanitise_job_name(value: str) -> str:
    value = value.strip()
    value = re.sub(r"\s+", "_", value)
    value = re.sub(r"[^A-Za-z0-9_-]", "", value)
    return value or "JOB"


def fmt_angle(angle: float) -> str:
    if abs(angle - round(angle)) < 1e-9:
        return f"{int(round(angle))}"
    return f"{angle:.3f}".rstrip("0").rstrip(".")


def square_model_angle_to_machine_orientation(model_angle: float) -> float:
    """Map polygon-model angle to physical A orientation.

    The polygon model is axis-aligned, so its first square corners lie at 45 deg.
    MkII operator setup defines physical A0 as CORNER-UP / diagonal-up. Therefore
    model 45 deg == machine A0, i.e. a fixed -45 deg phase shift.
    Returned orientation is normalized to [0, 360).
    """
    return (model_angle - 45.0) % 360.0


def next_forward_a(current_a: float, orientation_deg: float) -> float:
    """Return the next equivalent absolute A target at/after current_a.

    Rollover is OFF, so keep indexed ROUND motion monotonic and avoid accidental
    long reverse unwinds between equivalent 0..360 orientations.
    """
    base = orientation_deg % 360.0
    turns = math.floor(current_a / 360.0)
    target = base + turns * 360.0
    if target < current_a - 1e-9:
        target += 360.0
    return target


def next_phase_aligned_a(current_a: float, phase_deg: float, hand_sign: float) -> float:
    """Return the nearest phase-equivalent absolute A target in cutting direction.

    GROOVE depth passes must re-enter exactly the same physical helix phase.
    With rollover OFF we avoid physical multi-revolution unwinds by choosing the
    next equivalent phase in the same direction as the requested helix.
    """
    base = phase_deg % 360.0
    if hand_sign >= 0:
        turns = math.floor(current_a / 360.0)
        target = base + turns * 360.0
        if target < current_a - 1e-9:
            target += 360.0
    else:
        turns = math.ceil(current_a / 360.0)
        target = base + turns * 360.0
        if target > current_a + 1e-9:
            target -= 360.0
    return target


# -----------------------------------------------------------------------------
# Convex-polygon geometry used ONLY by Tab 1 stock preparation.
# -----------------------------------------------------------------------------

def clip_polygon_halfplane(poly, angle_deg, limit):
    """Keep points satisfying n dot p <= limit."""
    th = math.radians(angle_deg)
    nx, ny = math.cos(th), math.sin(th)
    out = []
    if not poly:
        return out

    for i, p1 in enumerate(poly):
        p2 = poly[(i + 1) % len(poly)]
        d1 = nx * p1[0] + ny * p1[1] - limit
        d2 = nx * p2[0] + ny * p2[1] - limit
        in1 = d1 <= 1e-10
        in2 = d2 <= 1e-10

        if in1:
            out.append(p1)

        if in1 != in2:
            den = d1 - d2
            if abs(den) > 1e-12:
                t = d1 / den
                out.append((
                    p1[0] + t * (p2[0] - p1[0]),
                    p1[1] + t * (p2[1] - p1[1]),
                ))
    return out


def polygon_support(poly, angle_deg):
    th = math.radians(angle_deg)
    nx, ny = math.cos(th), math.sin(th)
    return max(nx * x + ny * y for x, y in poly)


def polygon_line_width(poly, angle_deg, limit):
    """Length of intersection between polygon and line n dot p = limit."""
    th = math.radians(angle_deg)
    nx, ny = math.cos(th), math.sin(th)
    tx, ty = -ny, nx
    hits = []

    for i, p1 in enumerate(poly):
        p2 = poly[(i + 1) % len(poly)]
        v1 = nx * p1[0] + ny * p1[1] - limit
        v2 = nx * p2[0] + ny * p2[1] - limit

        if abs(v1) <= 1e-9:
            hits.append(p1[0] * tx + p1[1] * ty)

        if v1 * v2 < -1e-12:
            t = v1 / (v1 - v2)
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            hits.append(x * tx + y * ty)

    if len(hits) < 2:
        return 0.0
    return max(hits) - min(hits)


def deepest_no_trench_radius(poly, angle_deg, target_radius, cutter_dia):
    """
    Deepest support-plane radius at this A angle such that the entire exposed
    flat is no wider than cutter_dia. Never goes below target_radius.
    """
    support = polygon_support(poly, angle_deg)

    if polygon_line_width(poly, angle_deg, target_radius) <= cutter_dia + 1e-9:
        return target_radius

    lo = target_radius
    hi = support
    for _ in range(80):
        mid = (lo + hi) / 2.0
        if polygon_line_width(poly, angle_deg, mid) > cutter_dia:
            lo = mid
        else:
            hi = mid
    return hi


def max_polygon_radius(poly):
    return max(math.hypot(x, y) for x, y in poly)


class CNCApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.configure(bg="#d7dfe6")
        self.minsize(1280, 720)
        try:
            self.state("zoomed")
        except tk.TclError:
            self.geometry(f"{self.winfo_screenwidth()}x{self.winfo_screenheight()}+0+0")

        self.palette = {
            "app_bg": "#000000",
            "panel": "#101010",
            "header": "#0b4f8a",
            "header_text": "#ffffff",
            "text": "#ffffff",
            "border": "#4a4a4a",
            "editable_ok": "#161616",
            "editable_bad": "#5a1111",
            "readonly_bg": "#202020",
            "safe": "#2f9e44",
            "warn": "#d69e2e",
            "unsafe": "#d9534f",
            "status_bg": "#111111",
            "button_blue": "#0b63b6",
            "button_blue_active": "#0f76d1",
        }

        self._build_styles()

        # Common / Tab 1 inputs.
        self.v = {
            "job_name": tk.StringVar(value=""),
            "stock_shape": tk.StringVar(value="SQUARE"),
            "stock_square": tk.StringVar(value="40.000"),
            "round_carriage_probe_z": tk.StringVar(value=""),
            "wave_carriage_probe_z": tk.StringVar(value=""),
            "groove_carriage_probe_z": tk.StringVar(value=""),
            "round_dia": tk.StringVar(value="40.000"),
            "stock_length": tk.StringVar(value="250.000"),
            # Explicit operator-selected machine-coordinate end of the rotary cut.
            # Deliberately blank on launch: never infer this from physical stock length.
            "cut_y_max": tk.StringVar(value=""),
            "round_tool_dia": tk.StringVar(value="12.700"),
            "round_flutes": tk.StringVar(value="2"),
            "round_feed": tk.StringVar(value="2500"),
            "round_plunge": tk.StringVar(value="200"),
            "round_doc": tk.StringVar(value="1.000"),
            "round_spiral_rpm": tk.StringVar(value="30.000"),
            "round_spiral_pitch": tk.StringVar(value="1.500"),

            # Tab 2 only.
            "wave_dmin": tk.StringVar(value="30.000"),
            "waves": tk.StringVar(value="+3"),
            # WAVE cutter defaults inherit ROUND, but remain independently editable.
            "wave_tool_dia": tk.StringVar(value="12.700"),
            "wave_flutes": tk.StringVar(value="2"),
            "wave_feed": tk.StringVar(value="1800"),
            "wave_plunge": tk.StringVar(value="100"),
            "wave_doc": tk.StringVar(value="1.000"),
            "wave_finish_allow": tk.StringVar(value="0.250"),
            "wave_spiral_rpm": tk.StringVar(value="30.000"),
            "wave_spiral_pitch": tk.StringVar(value="0.500"),

            # Tab 3 - TAPER. WAVE must be zero for TAPER generation.
            "taper_carriage_probe_z": tk.StringVar(value=""),
            "taper_start_dia": tk.StringVar(value="40.000"),
            "taper_end_dia": tk.StringVar(value="30.000"),
            "taper_y_start": tk.StringVar(value="20.000"),
            "taper_y_end": tk.StringVar(value=""),
            "taper_tool_dia": tk.StringVar(value="12.700"),
            "taper_flutes": tk.StringVar(value="2"),
            "taper_feed": tk.StringVar(value="1800"),
            "taper_plunge": tk.StringVar(value="100"),
            "taper_doc": tk.StringVar(value="1.000"),
            "taper_rpm": tk.StringVar(value="30.000"),
            "taper_pitch": tk.StringVar(value="0.500"),

            # Tab 4 only. GROOVE source is deliberately blank on launch so
            # the operator must explicitly declare which physical base profile
            # is actually in the chuck: ROUND, WAVE or TAPER.
            "groove_source": tk.StringVar(value=""),
            "groove_enable": tk.BooleanVar(value=True),
            "groove_tool_dia": tk.StringVar(value="6.350"),
            "groove_depth": tk.StringVar(value="3.000"),
            "groove_doc": tk.StringVar(value="1.000"),
            "groove_pitch": tk.StringVar(value="25.000"),
            "groove_starts": tk.StringVar(value="2"),
            "groove_hand": tk.StringVar(value="Right-hand"),
            "groove_start_angle": tk.StringVar(value="0.000"),
            "groove_feed": tk.StringVar(value="1200"),
            "groove_plunge": tk.StringVar(value="150"),
            "groove_rpm": tk.StringVar(value="25.000"),
        }

        self.calc = {
            "cut_y_start": tk.StringVar(),
            "cut_y_end": tk.StringVar(),
            "z_zero_square": tk.StringVar(),
            "corner_radius": tk.StringVar(),
            "safe_z": tk.StringVar(),
            "round_a_center_z": tk.StringVar(),
            "round_z_cut_min": tk.StringVar(),
            "wave_a_center_z": tk.StringVar(),
            "wave_z_cut_min": tk.StringVar(),
            "taper_a_center_z": tk.StringVar(),
            "taper_z_cut_min": tk.StringVar(),
            "groove_a_center_z": tk.StringVar(),
            "groove_z_cut_min": tk.StringVar(),
            "round_stages": tk.StringVar(),
            "round_cleanup": tk.StringVar(),
            "round_revs": tk.StringVar(),
            "round_actual_pitch": tk.StringVar(),
            "round_y_feed": tk.StringVar(),
            "round_file": tk.StringVar(),
            "wave_file": tk.StringVar(),
            "taper_file": tk.StringVar(),
            "groove_file": tk.StringVar(),
            "wave_start_radius": tk.StringVar(),
            "wave_amplitude": tk.StringVar(),
            "wave_shells": tk.StringVar(),
            "wave_tool_radius": tk.StringVar(),
            "wave_wavelength": tk.StringVar(),
            "wave_min_curve_radius": tk.StringVar(),
            "wave_comp_x": tk.StringVar(),
            "wave_comp_z": tk.StringVar(),
            "wave_revs": tk.StringVar(),
            "wave_actual_pitch": tk.StringVar(),
            "wave_y_feed": tk.StringVar(),
            "wave_station_step": tk.StringVar(),
            "wave_station_count": tk.StringVar(),
            "wave_ring_count": tk.StringVar(),
            "wave_sweep_count": tk.StringVar(),
            "wave_rough_time": tk.StringVar(),
            "wave_finish_time": tk.StringVar(),
            "wave_total_time": tk.StringVar(),
            "wave_total_a": tk.StringVar(),
            "wave_final_phase": tk.StringVar(),
            "taper_length": tk.StringVar(),
            "taper_angle": tk.StringVar(),
            "taper_diameter_change": tk.StringVar(),
            "taper_radius_change": tk.StringVar(),
            "taper_min_dia": tk.StringVar(),
            "taper_max_dia": tk.StringVar(),
            "taper_revs": tk.StringVar(),
            "taper_actual_pitch": tk.StringVar(),
            "taper_y_feed": tk.StringVar(),
            "taper_rough_step": tk.StringVar(),
            "taper_rough_stations": tk.StringVar(),
            "taper_rough_rings": tk.StringVar(),
            "taper_rough_a_time": tk.StringVar(),
            "taper_finish_time": tk.StringVar(),
            "taper_total_a_time": tk.StringVar(),
            "taper_status": tk.StringVar(),
            "groove_root": tk.StringVar(),
            "groove_lead": tk.StringVar(),
            "groove_separation": tk.StringVar(),
            "groove_passes": tk.StringVar(),
            "groove_source_display": tk.StringVar(value="SELECT ROUND / WAVE / TAPER"),
            "groove_parent_y_start": tk.StringVar(),
            "groove_parent_y_end": tk.StringVar(),
            "groove_cut_y_start": tk.StringVar(),
            "groove_cut_y_end": tk.StringVar(),
            "round_status": tk.StringVar(),
            "wave_status": tk.StringVar(),
            "groove_status": tk.StringVar(),
        }

        # Safety confirmations are separate because each NC file is standalone.
        self.round_square_diagonal = tk.BooleanVar(value=False)
        self.round_rollover = tk.BooleanVar(value=False)
        self.wave_tool_changed = tk.BooleanVar(value=False)
        self.wave_rollover = tk.BooleanVar(value=False)
        self.taper_tool_changed = tk.BooleanVar(value=False)
        self.taper_rollover = tk.BooleanVar(value=False)
        self.groove_tool_changed = tk.BooleanVar(value=False)
        self.groove_rollover = tk.BooleanVar(value=False)

        self.entries = {}
        self._round_model = None
        self._recalc_busy = False

        # WAVE cutter defaults follow ROUND until the operator edits either
        # WAVE cutter field.  After that, WAVE remains independent.
        self._wave_tool_sync_busy = False
        self._wave_tool_overridden = False
        # TAPER cutter also follows ROUND until explicitly edited.
        self._taper_tool_sync_busy = False
        self._taper_tool_overridden = False

        # Waves=0 state management.
        # Remember the operator's last non-cylinder Dmin, force Dmin=Dmax at zero,
        # then restore the remembered value when leaving zero.
        self._wave_zero_busy = False
        self._last_wave_value = self._safe_int_value(self.v["waves"].get(), 3)
        self._remembered_wave_dmin = self.v["wave_dmin"].get()

        self._build_menu()
        self._build_ui()

        self.v["waves"].trace_add("write", self._on_waves_changed)
        self.v["wave_dmin"].trace_add("write", self._remember_wave_dmin)
        self.v["round_dia"].trace_add("write", self._sync_zero_wave_dmin_to_round)
        self.v["round_tool_dia"].trace_add("write", self._sync_wave_tool_from_round)
        self.v["round_flutes"].trace_add("write", self._sync_wave_tool_from_round)
        self.v["wave_tool_dia"].trace_add("write", self._mark_wave_tool_override)
        self.v["wave_flutes"].trace_add("write", self._mark_wave_tool_override)
        self.v["round_tool_dia"].trace_add("write", self._sync_taper_tool_from_round)
        self.v["round_flutes"].trace_add("write", self._sync_taper_tool_from_round)
        self.v["taper_tool_dia"].trace_add("write", self._mark_taper_tool_override)
        self.v["taper_flutes"].trace_add("write", self._mark_taper_tool_override)

        for key, var in self.v.items():
            if isinstance(var, tk.BooleanVar):
                continue
            var.trace_add("write", lambda *_: self.recalculate())
        self.v["groove_enable"].trace_add("write", lambda *_: self.recalculate())

        for b in (
            self.round_square_diagonal, self.round_rollover,
            self.wave_tool_changed, self.wave_rollover,
            self.taper_tool_changed, self.taper_rollover,
            self.groove_tool_changed, self.groove_rollover,
        ):
            b.trace_add("write", lambda *_: self.recalculate())

        self.recalculate()

    @staticmethod
    def _safe_int_value(value, default=0):
        try:
            return int(str(value).strip().replace("+", ""))
        except Exception:
            return default

    def _sync_wave_tool_from_round(self, *_):
        if self._wave_tool_overridden or self._wave_tool_sync_busy:
            return
        self._wave_tool_sync_busy = True
        try:
            self.v["wave_tool_dia"].set(self.v["round_tool_dia"].get())
            self.v["wave_flutes"].set(self.v["round_flutes"].get())
        finally:
            self._wave_tool_sync_busy = False

    def _mark_wave_tool_override(self, *_):
        if not self._wave_tool_sync_busy:
            self._wave_tool_overridden = True

    def _reset_wave_tool_to_round(self):
        self._wave_tool_overridden = False
        self._sync_wave_tool_from_round()
        self.recalculate()

    def _sync_taper_tool_from_round(self, *_):
        if self._taper_tool_overridden or self._taper_tool_sync_busy:
            return
        self._taper_tool_sync_busy = True
        try:
            self.v["taper_tool_dia"].set(self.v["round_tool_dia"].get())
            self.v["taper_flutes"].set(self.v["round_flutes"].get())
        finally:
            self._taper_tool_sync_busy = False

    def _mark_taper_tool_override(self, *_):
        if not self._taper_tool_sync_busy:
            self._taper_tool_overridden = True

    def _reset_taper_tool_to_round(self):
        self._taper_tool_overridden = False
        self._sync_taper_tool_from_round()
        self.recalculate()

    def _set_wave_dmin_state(self, locked):
        e = self.entries.get("wave_dmin")
        if e is None:
            return
        try:
            if locked:
                e.configure(
                    state="disabled",
                    disabledbackground=self.palette["readonly_bg"],
                    disabledforeground="#ffffff",
                )
            else:
                e.configure(state="normal", bg=self.palette["editable_ok"])
        except tk.TclError:
            pass

    def _remember_wave_dmin(self, *_):
        if self._wave_zero_busy:
            return
        waves = self._safe_int_value(self.v["waves"].get(), self._last_wave_value)
        if waves != 0:
            value = self.v["wave_dmin"].get().strip()
            if value:
                self._remembered_wave_dmin = value

    def _sync_zero_wave_dmin_to_round(self, *_):
        if self._wave_zero_busy:
            return
        waves = self._safe_int_value(self.v["waves"].get(), self._last_wave_value)
        if waves == 0:
            self._wave_zero_busy = True
            try:
                self.v["wave_dmin"].set(self.v["round_dia"].get())
            finally:
                self._wave_zero_busy = False

    def _on_waves_changed(self, *_):
        if self._wave_zero_busy:
            return

        waves = self._safe_int_value(self.v["waves"].get(), self._last_wave_value)
        previous = self._last_wave_value

        self._wave_zero_busy = True
        try:
            if waves == 0:
                # Capture the last useful non-cylinder value exactly once when
                # crossing into zero.
                if previous != 0:
                    candidate = self.v["wave_dmin"].get().strip()
                    if candidate:
                        self._remembered_wave_dmin = candidate
                self.v["wave_dmin"].set(self.v["round_dia"].get())
                self._set_wave_dmin_state(True)
            else:
                # Crossing back out of zero restores the operator's prior Dmin.
                if previous == 0:
                    self.v["wave_dmin"].set(self._remembered_wave_dmin)
                self._set_wave_dmin_state(False)

            self._last_wave_value = waves
        finally:
            self._wave_zero_busy = False

        self.recalculate()

    def build_wave_station_model(self, common, tool_dia, doc, allowance):
        """Precompute the deterministic indexed-ring roughing plan."""
        length = common["length"]
        start_radius = common["round_dia"] / 2.0
        cutter_radius = tool_dia / 2.0
        if cutter_radius <= 0:
            raise ValueError("Ball nose cutter radius must be positive.")
        if doc <= 0:
            raise ValueError("WAVE rough DOC must be positive.")
        if allowance < 0:
            raise ValueError("WAVE finish allowance cannot be negative.")

        intervals = max(1, math.ceil(length / cutter_radius))
        step = length / intervals
        stations = []
        total_rings = 0
        max_sweeps = 0

        for i in range(intervals + 1):
            x = length * i / intervals
            final_r = self.wave_radius(x, common)
            rough_target = min(start_radius, final_r + allowance)
            removal = max(0.0, start_radius - rough_target)
            rings = math.ceil(removal / doc - 1e-12) if removal > 1e-12 else 0
            total_rings += rings
            max_sweeps = max(max_sweeps, rings)
            stations.append({
                "index": i,
                "x": x,
                "final_radius": final_r,
                "rough_target": rough_target,
                "current_radius": start_radius,
                "rings": rings,
            })

        sweeps = []
        current_a = 0.0
        for sweep_index in range(max_sweeps):
            order = list(range(len(stations)))
            if sweep_index % 2:
                order.reverse()

            cuts = []
            for idx in order:
                st = stations[idx]
                current = st["current_radius"]
                target = st["rough_target"]
                if current <= target + 1e-10:
                    continue

                next_radius = max(target, current - doc)
                actual_doc = current - next_radius
                current_a += 360.0
                cuts.append({
                    "station_index": idx,
                    "x": st["x"],
                    "from_radius": current,
                    "to_radius": next_radius,
                    "actual_doc": actual_doc,
                    "a_target": current_a,
                })
                st["current_radius"] = next_radius

            if cuts:
                sweeps.append(cuts)

        return {
            "stations": stations,
            "sweeps": sweeps,
            "intervals": intervals,
            "station_count": len(stations),
            "step": step,
            "ring_count": total_rings,
            "sweep_count": len(sweeps),
            "rough_a_degrees": total_rings * 360.0,
        }

    def wave_runtime_model(self, common, station_model, rpm, finish_pitch, plunge):
        length = common["length"]
        start_r = common["round_dia"] / 2.0
        finish_revs = max(1, math.ceil(length / finish_pitch))
        actual_pitch = length / finish_revs
        finish_minutes = finish_revs / rpm
        ring_minutes = station_model["ring_count"] / rpm

        x_distance = 0.0
        for cuts in station_model["sweeps"]:
            for a, b in zip(cuts, cuts[1:]):
                x_distance += abs(b["x"] - a["x"])
        x_feed = max(self._f("wave_feed"), 1.0)
        x_minutes = x_distance / x_feed

        z_down = 0.0
        for cuts in station_model["sweeps"]:
            for cut in cuts:
                z_down += max(0.0, start_r + WAVE_LOCAL_CLEARANCE - cut["to_radius"])
        z_minutes = z_down / max(plunge, 1.0)

        rough_minutes = ring_minutes + x_minutes + z_minutes
        return {
            "finish_revs": finish_revs,
            "actual_pitch": actual_pitch,
            "finish_x_feed": actual_pitch * rpm,
            "rough_minutes": rough_minutes,
            "finish_minutes": finish_minutes,
            "total_minutes": rough_minutes + finish_minutes,
        }

    # ------------------------------------------------------------------
    # GUI construction
    # ------------------------------------------------------------------
    def _build_styles(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=self.palette["app_bg"])
        style.configure("TLabel", background=self.palette["app_bg"], foreground=self.palette["text"], font=("Segoe UI", 9))
        style.configure("TNotebook", background=self.palette["app_bg"])
        style.configure("TNotebook.Tab", background="#161616", foreground="#ffffff", font=("Segoe UI", 10, "bold"), padding=(12, 6))
        style.map("TNotebook.Tab", background=[("selected", self.palette["header"])], foreground=[("selected", "#ffffff")])
        style.configure("TLabelframe", background=self.palette["panel"], bordercolor=self.palette["border"], borderwidth=1, relief="solid")
        style.configure("TLabelframe.Label", background=self.palette["header"], foreground=self.palette["header_text"], font=("Segoe UI", 10, "bold"), padding=(7, 3))
        style.configure("Readonly.TEntry", fieldbackground=self.palette["readonly_bg"], foreground="#ffffff", padding=1)
        style.configure("Primary.TButton", background=self.palette["button_blue"], foreground="#ffffff", font=("Segoe UI", 10, "bold"), padding=(12, 6))
        style.map("Primary.TButton", background=[("active", self.palette["button_blue_active"]), ("pressed", self.palette["button_blue_active"])], foreground=[("disabled", "#8a8a8a"), ("!disabled", "#ffffff")])

    def _build_menu(self):
        m = tk.Menu(self)
        f = tk.Menu(m, tearoff=0)
        f.add_command(label="Make ROUND G Code", command=self.generate_round)
        f.add_command(label="Make WAVE G Code", command=self.generate_wave)
        f.add_command(label="Make GROOVE G Code", command=self.generate_groove)
        f.add_separator()
        f.add_command(label="Exit", command=self.destroy)
        m.add_cascade(label="File", menu=f)
        h = tk.Menu(m, tearoff=0)
        h.add_command(label="About", command=lambda: messagebox.showinfo("About", f"{APP_TITLE}\n\nSquare -> Round -> Wave -> Groove"))
        m.add_cascade(label="Help", menu=h)
        self.config(menu=m)

    def _build_ui(self):
        outer = ttk.Frame(self, padding=8)
        outer.pack(fill="both", expand=True)

        ttk.Label(outer, text=APP_TITLE, font=("Segoe UI", 14, "bold")).pack(anchor="w", pady=(0, 6))

        self.nb = ttk.Notebook(outer)
        self.nb.pack(fill="both", expand=True)

        self.tab_round = ttk.Frame(self.nb, padding=7)
        self.tab_wave = ttk.Frame(self.nb, padding=7)
        self.tab_taper = ttk.Frame(self.nb, padding=7)
        self.tab_groove = ttk.Frame(self.nb, padding=7)
        self.nb.add(self.tab_round, text="  Make Round  ")
        self.nb.add(self.tab_wave, text="  Wave Profile  ")
        self.nb.add(self.tab_taper, text="  Taper  ")
        self.nb.add(self.tab_groove, text="  Grooves  ")

        self._build_round_tab()
        self._build_wave_tab()
        self._build_taper_tab()
        self._build_groove_tab()

        footer = ttk.Frame(outer)
        footer.pack(fill="x", pady=(3, 0))
        ttk.Label(footer, text="Units: mm   |   XYZ output is G53 machine coordinates").pack(side="left")
        ttk.Label(footer, text=datetime.now().strftime("%d/%m/%Y %I:%M %p")).pack(side="right")

    def _three_columns(self, parent, left_width=285, mid_width=340, right_min=500):
        holder = ttk.Frame(parent)
        holder.pack(fill="both", expand=True)

        # Phase Two UI: keep the two data columns deliberately narrow and give
        # the preview/cross-section panel all remaining horizontal space.
        # grid_propagate(False) is important here: without it, long labels in
        # the child frames force Tk to widen the columns again regardless of
        # the column weights.
        left = ttk.Frame(holder, width=left_width)
        mid = ttk.Frame(holder, width=mid_width)
        right = ttk.Frame(holder)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 2))
        mid.grid(row=0, column=1, sticky="nsew", padx=2)
        right.grid(row=0, column=2, sticky="nsew", padx=(2, 0))
        left.grid_propagate(False)
        mid.grid_propagate(False)

        holder.columnconfigure(0, weight=0, minsize=left_width)
        holder.columnconfigure(1, weight=0, minsize=mid_width)
        holder.columnconfigure(2, weight=1, minsize=right_min)
        holder.rowconfigure(0, weight=1)
        return left, mid, right

    def _edit_row(self, parent, row, label, key, unit=""):
        # Compact operator row: fixed label/unit columns prevent labels from
        # silently re-expanding the left-hand panel.
        ttk.Label(parent, text=label, width=17, anchor="w").grid(row=row, column=0, sticky="w", pady=1)
        e = tk.Entry(parent, textvariable=self.v[key], width=9, font=("Segoe UI", 9), bg=self.palette["editable_ok"], fg=self.palette["text"], insertbackground="#ffffff", relief="solid", bd=1)
        e.grid(row=row, column=1, sticky="ew", padx=(3, 3), pady=1)
        ttk.Label(parent, text=unit, width=8, anchor="w").grid(row=row, column=2, sticky="w")
        parent.columnconfigure(0, weight=0)
        parent.columnconfigure(1, weight=1)
        parent.columnconfigure(2, weight=0)
        self.entries[key] = e
        return e

    def _read_row(self, parent, row, label, var, unit=""):
        ttk.Label(parent, text=label, width=17, anchor="w").grid(row=row, column=0, sticky="w", pady=1)
        e = ttk.Entry(parent, textvariable=var, width=9, state="readonly", style="Readonly.TEntry")
        e.grid(row=row, column=1, sticky="ew", padx=(3, 3), pady=1)
        ttk.Label(parent, text=unit, width=8, anchor="w").grid(row=row, column=2, sticky="w")
        parent.columnconfigure(0, weight=0)
        parent.columnconfigure(1, weight=1)
        parent.columnconfigure(2, weight=0)
        return e

    def _fixed_row(self, parent, row, label, value, unit=""):
        # Keep an owning reference to fixed StringVars.  Without this, Tkinter
        # may garbage-collect the Python variable and readonly entries appear blank.
        if not hasattr(self, "_fixed_vars"):
            self._fixed_vars = []
        var = tk.StringVar(value="TBD" if value is None else f"{value:.3f}")
        self._fixed_vars.append(var)
        self._read_row(parent, row, label, var, unit)

    def _status_panel(self, parent, var):
        box = tk.Label(parent, textvariable=var, justify="left", anchor="w", bg=self.palette["status_bg"], fg="#ffffff", relief="solid", bd=1, padx=8, pady=4, font=("Segoe UI", 9, "bold"), wraplength=520)
        box.grid(row=0, column=0, columnspan=3, sticky="ew")
        parent.columnconfigure(1, weight=1)
        return box

    def _build_round_tab(self):
        left, mid, right = self._three_columns(self.tab_round)

        p = ttk.LabelFrame(left, text="Job / Stock", padding=4); p.pack(fill="x", pady=(0, 4))
        self._edit_row(p, 0, "Job Name", "job_name")
        ttk.Label(p, text="Raw Stock Shape", width=17, anchor="w").grid(row=1, column=0, sticky="w", pady=1)
        shape = ttk.Combobox(p, textvariable=self.v["stock_shape"], values=("SQUARE", "ROUND"), state="readonly", width=9)
        shape.grid(row=1, column=1, sticky="ew", padx=(3, 3), pady=1)
        self._edit_row(p, 2, "Raw Stock Size", "stock_square", "mm")
        self._edit_row(p, 3, "Finished Diameter", "round_dia", "mm")
        self._edit_row(p, 4, "Raw Stock Length", "stock_length", "mm")
        self._edit_row(p, 5, "Cut Y Max", "cut_y_max", "machine mm")
        ttk.Label(
            p,
            text=(f"Mandatory MACHINE Y. Must be <= {Y_MACHINE_MAX:.0f} and remain below the fitted-stock "
                  f"tail limit: Chuck Face Y + Stock Length - {WASTE_TAIL:.0f} mm."),
            foreground="#8c3434", wraplength=265, justify="left"
        ).grid(row=6, column=0, columnspan=3, sticky="w", pady=(2,0))
        self._edit_row(p, 7, "Probe 1 RAW Z", "round_carriage_probe_z", "machine mm")
        ttk.Label(p, text=("Place the 40 mm Probe 1 block ON THE SPOILBOARD and probe with the fitted cutter. "
                                "Enter the RAW negative MACHINE Z shown by UCCNC. DO NOT subtract 40 mm yourself."),
                  foreground="#8c3434", wraplength=265, justify="left").grid(row=8, column=0, columnspan=3, sticky="w", pady=(2,0))

        p = ttk.LabelFrame(left, text="Flat End Mill / Cutting", padding=4); p.pack(fill="x", pady=(0, 4))
        self._edit_row(p, 0, "Cutter Diameter", "round_tool_dia", "mm")
        self._edit_row(p, 1, "Flutes", "round_flutes")
        self._edit_row(p, 2, "Cut Feed", "round_feed", "mm/min")
        self._edit_row(p, 3, "Plunge Feed", "round_plunge", "mm/min")
        self._edit_row(p, 4, "Max Radial DOC", "round_doc", "mm")

        p = ttk.LabelFrame(left, text="Round Finish", padding=4); p.pack(fill="x")
        self._edit_row(p, 0, "A-axis Speed", "round_spiral_rpm", "RPM")
        self._edit_row(p, 1, "Spiral Pitch", "round_spiral_pitch", "mm/rev")
        ttk.Label(p, text="Final spiral is constant-radius cleanup only.\nIndexed stages never deliberately finish in a trench.", justify="left").grid(row=2, column=0, columnspan=3, sticky="w", pady=(6, 0))

        p = ttk.LabelFrame(mid, text="MkII Machine Configuration", padding=6); p.pack(fill="x", pady=(0, 4))
        self._fixed_row(p, 0, "A Centre X", A_CENTER_X, "machine mm")
        self._fixed_row(p, 1, "Chuck Face Machine Y", CHUCK_FACE_MACHINE_Y, "machine mm")
        self._fixed_row(p, 2, "Y Machine Minimum", Y_MACHINE_MIN, "machine mm")
        self._fixed_row(p, 3, "Y Machine Maximum", Y_MACHINE_MAX, "machine mm")
        self._fixed_row(p, 4, "Probe 1 Block Height", PROBE1_BLOCK_HEIGHT, "mm")
        self._fixed_row(p, 5, "Spoilboard -> 40120", SPOILBOARD_TO_40120_TOP, "mm")
        self._fixed_row(p, 6, "40120 -> A Centre", LATHE_CARRIAGE_TOP_TO_A_CENTER, "mm")
        self._fixed_row(p, 7, "Raw Probe -> A Centre", PROBE1_RAW_TO_A_CENTER, "mm")
        self._fixed_row(p, 8, "Max Square Stock", STOCK_SQUARE_MAX, "mm")
        self._fixed_row(p, 9, "Max Stock Length", STOCK_LENGTH_MAX, "mm")
        self._fixed_row(p, 10, "Chuck Waste", WASTE_CHUCK, "mm")
        self._fixed_row(p, 11, "Tail Waste", WASTE_TAIL, "mm")
        self._fixed_row(p, 12, "RPM Maximum", RPM_MAX, "RPM")
        self._fixed_row(p, 13, "Minimum Diameter", Z_MIN_DIAMETER, "mm")

        p = ttk.LabelFrame(mid, text="Calculated Geometry", padding=6); p.pack(fill="x", pady=(0, 4))
        self._read_row(p, 0, "Cut Y Start", self.calc["cut_y_start"], "mm")
        self._read_row(p, 1, "Cut Y End", self.calc["cut_y_end"], "mm")
        self._read_row(p, 2, "Current A Centre Z", self.calc["round_a_center_z"], "machine mm")
        self._read_row(p, 3, "Deepest Legal Cut Z", self.calc["round_z_cut_min"], "machine mm")
        self._read_row(p, 4, "Rotating Stock Radius", self.calc["corner_radius"], "mm")
        self._read_row(p, 5, "Safe Z", self.calc["safe_z"], "machine mm")

        p = ttk.LabelFrame(mid, text="Rounding Strategy", padding=6); p.pack(fill="x")
        self._read_row(p, 0, "Indexed Stages", self.calc["round_stages"])
        self._read_row(p, 1, "Worst Cleanup Spiral Cut", self.calc["round_cleanup"], "mm radial")
        self._read_row(p, 2, "Spiral Revolutions", self.calc["round_revs"], "rev")
        self._read_row(p, 3, "Actual Spiral Pitch", self.calc["round_actual_pitch"], "mm/rev")
        self._read_row(p, 4, "Calculated Y Feed", self.calc["round_y_feed"], "mm/min")
        ttk.Label(p, text=f"32 facets are accepted when final radial cleanup <= {ROUND_SPIRAL_MAX_CLEANUP:.2f} mm; otherwise 64 is added.", justify="left", wraplength=390).grid(row=5, column=0, columnspan=3, sticky="w", pady=(6, 0))

        p = ttk.LabelFrame(right, text="Cross-Section Evolution", padding=4); p.pack(fill="both", expand=True, pady=(0, 4))
        self.round_canvas = tk.Canvas(p, bg="#050505", highlightthickness=1, highlightbackground=self.palette["border"], height=300)
        self.round_canvas.pack(fill="both", expand=True)

        p = ttk.LabelFrame(right, text="Pre-flight / Operator Safety", padding=4); p.pack(fill="x", pady=(0, 4))
        ttk.Checkbutton(
            p,
            text="I confirm square stock is at A0 in the required DIAGONAL / CORNER-UP orientation",
            variable=self.round_square_diagonal,
        ).pack(anchor="w")
        ttk.Checkbutton(p, text='I confirm UCCNC A-axis "Roll over on 360°" is OFF', variable=self.round_rollover).pack(anchor="w")
        ttk.Label(
            p,
            text=("The ROUND model assumes the square starts in the defined diagonal orientation. "
                  "If it does not, early air-cuts can invalidate later DOC assumptions."),
            justify="left", wraplength=820,
        ).pack(anchor="w", pady=(3, 0))
        ttk.Label(p, text="ROUND uses G53 machine coordinates; X is fixed at the configured A centreline.", justify="left").pack(anchor="w", pady=(3, 0))

        p = ttk.LabelFrame(right, text="Status / Output", padding=4); p.pack(fill="x")
        self.round_status_box = self._status_panel(p, self.calc["round_status"])
        self._read_row(p, 1, "Output File", self.calc["round_file"])
        self.round_btn = ttk.Button(p, text="Make G Code", style="Primary.TButton", command=self.generate_round)
        self.round_btn.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))

    def _build_wave_tab(self):
        left, mid, right = self._three_columns(self.tab_wave)

        p = ttk.LabelFrame(left, text="Inherited From Tab 1", padding=4); p.pack(fill="x", pady=(0, 4))
        self._read_row(p, 0, "Job Name", self.v["job_name"])
        self._read_row(p, 1, "Finished Diameter", self.v["round_dia"], "mm")
        self._read_row(p, 2, "Stock Length", self.v["stock_length"], "mm")
        self._read_row(p, 3, "Cut Y Max", self.v["cut_y_max"], "machine mm")
        self._fixed_row(p, 4, "Chuck Waste", WASTE_CHUCK, "mm")
        self._fixed_row(p, 5, "Tail Waste", WASTE_TAIL, "mm")
        self._read_row(p, 6, "Cut Y Start", self.calc["cut_y_start"], "mm")
        self._read_row(p, 7, "Cut Y End", self.calc["cut_y_end"], "mm")
        self._edit_row(p, 8, "Probe 1 RAW Z", "wave_carriage_probe_z", "machine mm")
        ttk.Label(
            p,
            text=("WAVE: fit the ball nose cutter, place the 40 mm Probe 1 block ON THE SPOILBOARD, probe it, "
                  "and enter the RAW negative MACHINE Z. DO NOT subtract 40 mm; ignore work Z."),
            foreground="#8c3434", wraplength=265, justify="left"
        ).grid(row=9, column=0, columnspan=3, sticky="w", pady=(2,0))
        self._read_row(p, 10, "Current A Centre Z", self.calc["wave_a_center_z"], "machine mm")
        self._read_row(p, 11, "Deepest Legal Cut Z", self.calc["wave_z_cut_min"], "machine mm")

        p = ttk.LabelFrame(left, text="Wave Profile", padding=6); p.pack(fill="x", pady=(0, 4))
        self._read_row(p, 0, "Maximum Diameter", self.v["round_dia"], "mm")
        self._edit_row(p, 1, "Minimum Diameter", "wave_dmin", "mm")
        ttk.Label(p, text="Waves (-5 .. +5)").grid(row=2, column=0, sticky="w", pady=2)
        waves_spin = tk.Spinbox(
            p,
            from_=-5,
            to=5,
            increment=1,
            textvariable=self.v["waves"],
            width=12,
            font=("Segoe UI", 10),
            bg=self.palette["editable_ok"],
            fg=self.palette["text"],
            buttonbackground="#303030",
            relief="solid",
            bd=1,
            wrap=False,
        )
        waves_spin.grid(row=2, column=1, sticky="ew", padx=6, pady=2)
        ttk.Label(p, text="").grid(row=2, column=2, sticky="w")
        p.columnconfigure(1, weight=1)
        self.entries["waves"] = waves_spin

        self._edit_row(p, 3, "Rough Radial DOC", "wave_doc", "mm")
        self._edit_row(p, 4, "Finish Allowance", "wave_finish_allow", "mm radial")

        p = ttk.LabelFrame(left, text="Ball Nose / Cutting", padding=6); p.pack(fill="x")
        self._edit_row(p, 0, "Ball Nose Dia", "wave_tool_dia", "mm")
        self._edit_row(p, 1, "Flutes", "wave_flutes")
        ttk.Label(p, text="Defaults follow ROUND until edited. Change these when fitting a different ball nose.", foreground="#526878", wraplength=255, justify="left").grid(row=2, column=0, columnspan=3, sticky="w", pady=(2,2))
        ttk.Button(p, text="Reset cutter to ROUND values", command=self._reset_wave_tool_to_round).grid(row=3, column=0, columnspan=3, sticky="ew", pady=(0,4))
        self._edit_row(p, 4, "Cut Feed (nominal)", "wave_feed", "mm/min")
        self._edit_row(p, 5, "Plunge Feed", "wave_plunge", "mm/min")
        self._edit_row(p, 6, "A-axis Speed", "wave_spiral_rpm", "RPM")
        ttk.Label(p, text=f"Direct A feed; limited to {RPM_MAX:.1f} RPM", foreground="#526878").grid(row=6, column=3, sticky="w", padx=(4, 0))
        self._edit_row(p, 7, "Helical Pitch", "wave_spiral_pitch", "mm/rev")

        p = ttk.LabelFrame(mid, text="Profile Strategy", padding=6); p.pack(fill="x", pady=(0, 4))
        self._read_row(p, 0, "Starting Radius", self.calc["wave_start_radius"], "mm")
        self._read_row(p, 1, "Wave Amplitude", self.calc["wave_amplitude"], "mm radial")
        self._read_row(p, 2, "Y Station Step", self.calc["wave_station_step"], "mm")
        self._read_row(p, 3, "Y Stations", self.calc["wave_station_count"], "stations")
        self._read_row(p, 4, "Roughing Sweeps", self.calc["wave_sweep_count"], "passes")
        self._read_row(p, 5, "Total Rough Rings", self.calc["wave_ring_count"], "rings")
        self._read_row(p, 6, "Ball Nose Radius", self.calc["wave_tool_radius"], "mm")
        self._read_row(p, 7, "Wave Length", self.calc["wave_wavelength"], "mm/wave")
        self._read_row(p, 8, "Min Curve Radius", self.calc["wave_min_curve_radius"], "mm")
        self._read_row(p, 9, "Finish Revolutions", self.calc["wave_revs"], "rev")
        self._read_row(p,10, "Actual Finish Pitch", self.calc["wave_actual_pitch"], "mm/rev")
        self._read_row(p,11, "Finish Y Feed", self.calc["wave_y_feed"], "mm/min")
        self._read_row(p,12, "Estimated Rough Time", self.calc["wave_rough_time"], "min")
        self._read_row(p,13, "Estimated Finish Time", self.calc["wave_finish_time"], "min")
        self._read_row(p,14, "Estimated Total Time", self.calc["wave_total_time"], "min")
        self._read_row(p,15, "True A Travel", self.calc["wave_total_a"], "deg")
        self._read_row(p,16, "Final Physical A Phase", self.calc["wave_final_phase"], "original A0")
        ttk.Label(
            p,
            text=("Roughing uses complete 360° rings at fixed X/Z. Finished stations "
                  "drop out of later sweeps. Final pass is a compensated cosine spiral."),
            justify="left", wraplength=390,
        ).grid(row=14, column=0, columnspan=3, sticky="w", pady=(6, 0))

        p = ttk.LabelFrame(mid, text="Manual Tool Change", padding=6); p.pack(fill="x")
        ttk.Label(p, text=("Tab 2 assumes Tab 1 has completed and stopped.\nFit the ball nose cutter, put the 40 mm Probe 1 block on the spoilboard, probe it with that cutter, and enter the RAW MACHINE Z above. Do not subtract the block height yourself."), justify="left", wraplength=390).pack(anchor="w")

        p = ttk.LabelFrame(right, text="Wave Preview", padding=4); p.pack(fill="both", expand=True, pady=(0, 4))
        self.wave_canvas = tk.Canvas(p, bg="#050505", highlightthickness=1, highlightbackground=self.palette["border"], height=300)
        self.wave_canvas.pack(fill="both", expand=True)

        p = ttk.LabelFrame(right, text="Operator Safety", padding=6); p.pack(fill="x", pady=(0, 4))
        ttk.Checkbutton(p, text="I have changed to the ball nose cutter", variable=self.wave_tool_changed).pack(anchor="w")
        ttk.Checkbutton(p, text='UCCNC A-axis "Roll over on 360°" is OFF', variable=self.wave_rollover).pack(anchor="w")
        ttk.Label(
            p,
            text=(f"WAVE drives A directly from the requested RPM (A feed = RPM x 360 deg/min). "
                  f"Requested speed must be <= {RPM_MAX:.1f} RPM."),
            foreground="#526878", wraplength=560, justify="left",
        ).pack(anchor="w", pady=(4, 0))

        p = ttk.LabelFrame(right, text="Status / Output", padding=4); p.pack(fill="x")
        self.wave_status_box = self._status_panel(p, self.calc["wave_status"])
        self._read_row(p, 1, "Output File", self.calc["wave_file"])
        self.wave_btn = ttk.Button(p, text="Make G Code", style="Primary.TButton", command=self.generate_wave)
        self.wave_btn.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))

    def _build_taper_tab(self):
        left, mid, right = self._three_columns(self.tab_taper)

        p = ttk.LabelFrame(left, text="Inherited / Datum", padding=4); p.pack(fill="x", pady=(0, 4))
        self._read_row(p, 0, "Job Name", self.v["job_name"])
        self._read_row(p, 1, "Round Blank Dia", self.v["round_dia"], "mm")
        self._read_row(p, 2, "Stock Length", self.v["stock_length"], "mm")
        self._read_row(p, 3, "Machine Cut Y Max", self.v["cut_y_max"], "mm")
        self._edit_row(p, 4, "Probe 1 RAW Z", "taper_carriage_probe_z", "machine mm")
        ttk.Label(p, text=("Fit the TAPER cutter. Put the 40 mm Probe 1 block ON THE SPOILBOARD, probe it, and enter the RAW negative MACHINE Z. DO NOT subtract 40 mm yourself."), foreground="#8c3434", wraplength=265, justify="left").grid(row=5, column=0, columnspan=3, sticky="w", pady=(2,0))
        self._read_row(p, 6, "Current A Centre Z", self.calc["taper_a_center_z"], "machine mm")
        self._read_row(p, 7, "Deepest Legal Cut Z", self.calc["taper_z_cut_min"], "machine mm")

        p = ttk.LabelFrame(left, text="Taper Geometry", padding=4); p.pack(fill="x", pady=(0,4))
        self._edit_row(p, 0, "Start Diameter", "taper_start_dia", "mm")
        self._edit_row(p, 1, "End Diameter", "taper_end_dia", "mm")
        self._edit_row(p, 2, "Taper Y Start", "taper_y_start", "machine mm")
        self._edit_row(p, 3, "Taper Y End", "taper_y_end", "machine mm")
        ttk.Label(p, text=f"Both finished diameters must be >= {MIN_TAPER_DIAMETER:.1f} mm. Direction is defined by Start/End diameter.", foreground="#526878", wraplength=260, justify="left").grid(row=4, column=0, columnspan=3, sticky="w", pady=(3,0))

        p = ttk.LabelFrame(left, text="Cutter / Cutting", padding=4); p.pack(fill="x")
        self._edit_row(p, 0, "Cutter Diameter", "taper_tool_dia", "mm")
        self._edit_row(p, 1, "Flutes", "taper_flutes")
        ttk.Label(p, text="Defaults follow ROUND until edited.", foreground="#526878", wraplength=255, justify="left").grid(row=2, column=0, columnspan=3, sticky="w", pady=(2,2))
        ttk.Button(p, text="Reset cutter to ROUND values", command=self._reset_taper_tool_to_round).grid(row=3, column=0, columnspan=3, sticky="ew", pady=(0,4))
        self._edit_row(p, 4, "Cut Feed", "taper_feed", "mm/min")
        self._edit_row(p, 5, "Plunge Feed", "taper_plunge", "mm/min")
        self._edit_row(p, 6, "Max Radial DOC", "taper_doc", "mm")
        self._edit_row(p, 7, "A-axis Speed", "taper_rpm", "RPM")
        self._edit_row(p, 8, "Helical Pitch", "taper_pitch", "mm/rev")

        p = ttk.LabelFrame(mid, text="Calculated Taper", padding=4); p.pack(fill="x", pady=(0,4))
        self._read_row(p, 0, "Taper Length", self.calc["taper_length"], "mm")
        self._read_row(p, 1, "Diameter Change", self.calc["taper_diameter_change"], "mm")
        self._read_row(p, 2, "Radius Change", self.calc["taper_radius_change"], "mm")
        self._read_row(p, 3, "Taper Angle", self.calc["taper_angle"], "deg")
        self._read_row(p, 4, "Minimum Diameter", self.calc["taper_min_dia"], "mm")
        self._read_row(p, 5, "Maximum Diameter", self.calc["taper_max_dia"], "mm")
        self._read_row(p, 6, "Rough Y Step", self.calc["taper_rough_step"], "mm")
        self._read_row(p, 7, "Rough Stations", self.calc["taper_rough_stations"], "stations")
        self._read_row(p, 8, "Rough Rings", self.calc["taper_rough_rings"], "rings")
        self._read_row(p, 9, "Rough A Time", self.calc["taper_rough_a_time"], "min")
        self._read_row(p, 10, "Finish Revolutions", self.calc["taper_revs"], "rev")
        self._read_row(p, 11, "Actual Pitch", self.calc["taper_actual_pitch"], "mm/rev")
        self._read_row(p, 12, "Finish Y Feed", self.calc["taper_y_feed"], "mm/min")
        self._read_row(p, 13, "Finish A Time", self.calc["taper_finish_time"], "min")
        self._read_row(p, 14, "Total A Time", self.calc["taper_total_a_time"], "min")
        ttk.Label(p, text="Roughing starts at the thin end, uses 80% cutter-width indexed rings with no-trench footprint protection, then one spiral finish.", foreground="#526878", wraplength=320, justify="left").grid(row=15, column=0, columnspan=3, sticky="w", pady=(6,0))

        p = ttk.LabelFrame(right, text="Taper Preview", padding=4); p.pack(fill="both", expand=True, pady=(0,4))
        self.taper_canvas = tk.Canvas(p, bg="#050505", highlightthickness=1, highlightbackground=self.palette["border"], height=300)
        self.taper_canvas.pack(fill="both", expand=True)

        p = ttk.LabelFrame(right, text="Operator Safety", padding=4); p.pack(fill="x", pady=(0,4))
        ttk.Checkbutton(p, text="I have fitted the intended TAPER cutter and re-probed the 40120 carriage", variable=self.taper_tool_changed).pack(anchor="w")
        ttk.Checkbutton(p, text='UCCNC A-axis "Roll over on 360°" is OFF', variable=self.taper_rollover).pack(anchor="w")
        ttk.Label(p, text="TAPER is locked out whenever WAVE count is non-zero. Y start/end are checked against the same stock, tail-waste and Y600 machine limits as ROUND/WAVE.", wraplength=700, justify="left").pack(anchor="w", pady=(3,0))

        p = ttk.LabelFrame(right, text="Status / Output", padding=4); p.pack(fill="x")
        self.taper_status_box = self._status_panel(p, self.calc["taper_status"])
        self._read_row(p, 1, "Output File", self.calc["taper_file"])
        self.taper_btn = ttk.Button(p, text="Make G Code", style="Primary.TButton", command=self.generate_taper)
        self.taper_btn.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8,0))

    def _build_groove_tab(self):
        left, mid, right = self._three_columns(self.tab_groove, left_width=255, mid_width=315, right_min=560)

        p = ttk.LabelFrame(left, text="GROOVE Base Profile - OPERATOR SELECT", padding=4); p.pack(fill="x", pady=(0, 4))
        ttk.Label(
            p, textvariable=self.calc["groove_source_display"],
            font=("Segoe UI", 10, "bold"), anchor="center"
        ).pack(fill="x", pady=(0, 4))
        for label in ("ROUND", "WAVE", "TAPER"):
            ttk.Radiobutton(p, text=label, variable=self.v["groove_source"], value=label).pack(anchor="w")
        ttk.Label(
            p, text="Select the profile that actually exists on the stock. No auto-detection.",
            foreground="#8c3434", wraplength=235, justify="left"
        ).pack(anchor="w", pady=(3, 0))

        p = ttk.LabelFrame(left, text="Inherited Geometry", padding=4); p.pack(fill="x", pady=(0, 4))
        self._read_row(p, 0, "Job Name", self.v["job_name"])
        self._read_row(p, 1, "Finished Diameter", self.v["round_dia"], "mm")
        self._read_row(p, 2, "Min Wave Diameter", self.v["wave_dmin"], "mm")
        self._read_row(p, 3, "Waves", self.v["waves"])
        self._read_row(p, 4, "Stock Length", self.v["stock_length"], "mm")
        self._read_row(p, 5, "Cut Y Max", self.v["cut_y_max"], "machine mm")
        self._read_row(p, 6, "Parent Y Start", self.calc["groove_parent_y_start"], "machine mm")
        self._read_row(p, 7, "Parent Y End", self.calc["groove_parent_y_end"], "machine mm")
        self._read_row(p, 8, "Groove Y Start", self.calc["groove_cut_y_start"], "machine mm")
        self._read_row(p, 9, "Groove Y End", self.calc["groove_cut_y_end"], "machine mm")
        self._edit_row(p, 10, "Probe 1 RAW Z", "groove_carriage_probe_z", "machine mm")
        ttk.Label(
            p,
            text=("GROOVE: fit the groove cutter, put the 40 mm Probe 1 block ON THE SPOILBOARD, probe it, "
                  "and enter the RAW negative MACHINE Z. DO NOT subtract 40 mm; ignore work Z."),
            foreground="#8c3434", wraplength=235, justify="left"
        ).grid(row=11, column=0, columnspan=3, sticky="w", pady=(2,0))
        self._read_row(p, 12, "Current A Centre Z", self.calc["groove_a_center_z"], "machine mm")
        self._read_row(p, 13, "Deepest Legal Cut Z", self.calc["groove_z_cut_min"], "machine mm")

        p = ttk.LabelFrame(mid, text="Calculated Groove Geometry", padding=6); p.pack(fill="x", pady=(0, 4))
        self._read_row(p, 0, "Root Diameter at Dmin", self.calc["groove_root"], "mm")
        self._read_row(p, 1, "Depth Passes", self.calc["groove_passes"], "passes")
        self._read_row(p, 2, "Groove Lead", self.calc["groove_lead"], "mm/start/rev")
        self._read_row(p, 3, "Start Separation", self.calc["groove_separation"], "deg")

        p = ttk.LabelFrame(mid, text="Tool / Cutting", padding=6); p.pack(fill="x", pady=(0, 4))
        self._edit_row(p, 0, "Cut Feed (nominal)", "groove_feed", "mm/min")
        self._edit_row(p, 1, "Plunge Feed", "groove_plunge", "mm/min")
        self._edit_row(p, 2, "A-axis Speed", "groove_rpm", "RPM")
        ttk.Label(p, text="Groove path follows the explicitly selected ROUND / WAVE / TAPER base profile.\nEach start is a separate helix.", justify="left", wraplength=290).grid(row=3, column=0, columnspan=3, sticky="w", pady=(6, 0))

        p = ttk.LabelFrame(mid, text="Groove Settings", padding=4); p.pack(fill="x")
        ttk.Checkbutton(p, text="Enable Grooves", variable=self.v["groove_enable"]).grid(row=0, column=0, columnspan=3, sticky="w", pady=2)
        self._edit_row(p, 1, "Cutter Diameter", "groove_tool_dia", "mm")
        self._edit_row(p, 2, "Groove Depth", "groove_depth", "mm radial")
        self._edit_row(p, 3, "Max DOC / Pass", "groove_doc", "mm radial")
        self._edit_row(p, 4, "Pitch", "groove_pitch", "mm/rev")
        self._edit_row(p, 5, "Starts", "groove_starts")
        ttk.Label(p, text="Handedness", width=17, anchor="w").grid(row=6, column=0, sticky="w", pady=1)
        hand = ttk.Combobox(p, textvariable=self.v["groove_hand"], values=("Right-hand", "Left-hand"), state="readonly", width=9)
        hand.grid(row=6, column=1, sticky="ew", padx=(3,3), pady=1)
        ttk.Label(p, text="", width=8).grid(row=6, column=2, sticky="w")
        self._edit_row(p, 7, "Start Angle", "groove_start_angle", "deg")

        p = ttk.LabelFrame(right, text="Groove Preview", padding=4); p.pack(fill="both", expand=True, pady=(0, 4))
        self.groove_canvas = tk.Canvas(p, bg="#050505", highlightthickness=1, highlightbackground=self.palette["border"], height=300)
        self.groove_canvas.pack(fill="both", expand=True)

        p = ttk.LabelFrame(right, text="Operator Safety", padding=6); p.pack(fill="x", pady=(0, 4))
        ttk.Checkbutton(p, text="I have changed to the groove cutter", variable=self.groove_tool_changed).pack(anchor="w")
        ttk.Checkbutton(p, text='UCCNC A-axis "Roll over on 360°" is OFF', variable=self.groove_rollover).pack(anchor="w")

        p = ttk.LabelFrame(right, text="Status / Output", padding=4); p.pack(fill="x")
        self.groove_status_box = self._status_panel(p, self.calc["groove_status"])
        self._read_row(p, 1, "Output File", self.calc["groove_file"])
        self.groove_btn = ttk.Button(p, text="Make G Code", style="Primary.TButton", command=self.generate_groove)
        self.groove_btn.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))

    # ------------------------------------------------------------------
    # Parsing / validation
    # ------------------------------------------------------------------
    def _f(self, key):
        return float(self.v[key].get().strip())

    def _i(self, key):
        return int(self.v[key].get().strip())

    def common_values(self):
        stock_length = self._f("stock_length")
        # Rotary cutting may begin at the hard machine minimum Y0. Physical
        # stock length is measured from the chuck face; the separate physical
        # chuck-end waste allowance does not redefine the machine cut envelope.
        cut_y_start = Y_MACHINE_MIN

        # SAFETY: Cut Y Max remains an explicit operator decision in MACHINE Y.
        # It is never inferred automatically, but it is cross-checked against
        # both the hard machine Y limit and the actual fitted stock/tail reserve.
        # A blank value remains a mandatory-input condition.
        raw_y_max = self.v["cut_y_max"].get().strip()
        cut_y_max_entered = bool(raw_y_max)
        cut_y_end = float(raw_y_max) if raw_y_max else cut_y_start
        machining_length = cut_y_end - cut_y_start
        return {
            "job": sanitise_job_name(self.v["job_name"].get()),
            "stock_shape": self.v["stock_shape"].get().strip().upper(),
            "stock": self._f("stock_square"),
            "round_dia": self._f("round_dia"),
            "stock_length": stock_length,
            "cut_y_max": cut_y_end,
            "cut_y_max_entered": cut_y_max_entered,
            "length": machining_length,
        }

    @staticmethod
    def machine_config_errors():
        e = []
        if STOCK_Y_DIRECTION != 1:
            e.append("MkII requires STOCK_Y_DIRECTION = +1.")
        if Y_MACHINE_MIN < 0:
            e.append("Y_MACHINE_MIN cannot be negative for this build.")
        if Y_MACHINE_MAX <= Y_MACHINE_MIN:
            e.append("Y_MACHINE_MAX must be greater than Y_MACHINE_MIN.")
        if CHUCK_FACE_MACHINE_Y >= 0:
            e.append("CHUCK_FACE_MACHINE_Y must be negative because the fixed chuck face is before machine Y0.")
        if WASTE_CHUCK < 0 or WASTE_TAIL < 0:
            e.append("Configured waste lengths cannot be negative.")
        if PROBE1_BLOCK_HEIGHT <= 0:
            e.append("PROBE1_BLOCK_HEIGHT must be positive.")
        if SPOILBOARD_TO_40120_TOP <= 0:
            e.append("SPOILBOARD_TO_40120_TOP must be positive.")
        if LATHE_CARRIAGE_TOP_TO_A_CENTER <= 0:
            e.append("LATHE_CARRIAGE_TOP_TO_A_CENTER must be positive.")
        return e

    def probe_center_z(self, key):
        # Operator enters the RAW G53 Z reported when the fitted cutter touches
        # the top of the 40 mm Probe 1 block sitting directly on the spoilboard.
        # Do not pre-correct the reading in UCCNC or by hand.
        #
        # spoilboard Z = raw_probe_z - PROBE1_BLOCK_HEIGHT
        # 40120 top Z  = spoilboard Z + SPOILBOARD_TO_40120_TOP
        # A centre Z    = 40120 top Z + LATHE_CARRIAGE_TOP_TO_A_CENTER
        raw = self.v[key].get().strip()
        if not raw:
            raise ValueError("Probe 1 RAW machine Z is mandatory. Probe the 40 mm block on the spoilboard and enter the uncorrected G53 Z.")
        try:
            probe_z = float(raw)
        except ValueError:
            raise ValueError("Probe 1 RAW Z must be a valid machine-coordinate number.")
        if probe_z >= 0:
            raise ValueError("Probe 1 RAW Z should be a negative machine coordinate.")
        return probe_z + PROBE1_RAW_TO_A_CENTER

    @staticmethod
    def deepest_allowed_z(center_z):
        return center_z + Z_MIN_DIAMETER / 2.0

    @staticmethod
    def radius_to_machine_z(radius, center_z):
        return center_z + radius

    def common_geometry(self, c, center_z):
        # Chuck/headstock hardware is permanently outside the cut envelope.
        # Rotary cutting may start at hard machine Y minimum (Y0 in this build),
        # with the operator-entered machine-coordinate Cut Y Max as the end.
        cut_y_start = Y_MACHINE_MIN
        cut_y_end = c["cut_y_max"]

        if c["stock_shape"] == "SQUARE":
            rotating_radius = c["stock"] * math.sqrt(2.0) / 2.0
            stock_top_radius = c["stock"] / 2.0
        elif c["stock_shape"] == "ROUND":
            rotating_radius = c["stock"] / 2.0
            stock_top_radius = rotating_radius
        else:
            raise ValueError("Raw Stock Shape must be SQUARE or ROUND.")

        safe_z = center_z + rotating_radius + SAFE_Z_EXTRA
        stock_top_z = center_z + stock_top_radius
        return cut_y_start, cut_y_end, rotating_radius, safe_z, stock_top_z

    @staticmethod
    def validate_rpm(rpm, label):
        if rpm <= 0:
            raise ValueError(f"{label} RPM must be positive.")
        if rpm > RPM_MAX + 1e-9:
            raise ValueError(f"{label} RPM {rpm:.3f} exceeds configured RPM_MAX {RPM_MAX:.3f}.")

    def common_errors(self, c):
        e = list(self.machine_config_errors())
        if c["stock_shape"] not in ("SQUARE", "ROUND"):
            e.append("Raw Stock Shape must be SQUARE or ROUND.")
        if c["stock"] <= 0 or c["stock"] > STOCK_SQUARE_MAX + 1e-9:
            e.append(f"Stock size must be >0 and <= {STOCK_SQUARE_MAX:.0f} mm.")
        if c["round_dia"] <= 0 or c["round_dia"] > c["stock"] + 1e-9:
            e.append("Finished round diameter must be positive and cannot exceed raw stock size.")
        if c["round_dia"] < Z_MIN_DIAMETER - 1e-9:
            e.append(f"Finished round diameter cannot be below Z_MIN_DIAMETER {Z_MIN_DIAMETER:.3f} mm.")
        if c["stock_length"] <= 0 or c["stock_length"] > STOCK_LENGTH_MAX + 1e-9:
            e.append(f"Raw stock length must be >0 and <= {STOCK_LENGTH_MAX:.0f} mm.")
        # Convert physical stock length to the machine-Y coordinate of its tail end.
        # The fixed chuck face is at CHUCK_FACE_MACHINE_Y (-70 mm in this build),
        # so a 250 mm blank ends at machine Y180: -70 + 250 = 180.
        stock_tail_y = CHUCK_FACE_MACHINE_Y + c["stock_length"]
        tail_safe_limit_y = stock_tail_y - WASTE_TAIL

        if not c["cut_y_max_entered"]:
            e.append("Cut Y Max is mandatory; enter the actual last machine Y you want cut.")
        elif c["cut_y_max"] > Y_MACHINE_MAX + 1e-9:
            e.append(f"Cut Y Max {c['cut_y_max']:.3f} exceeds hard Y_MACHINE_MAX {Y_MACHINE_MAX:.3f} mm.")
        elif c["cut_y_max"] <= Y_MACHINE_MIN + 1e-9:
            e.append(f"Cut Y Max must be greater than machine Y minimum Y{Y_MACHINE_MIN:.3f}.")
        elif c["cut_y_max"] >= tail_safe_limit_y - 1e-9:
            e.append(
                f"Cut Y Max {c['cut_y_max']:.3f} is too close to the tailstock. "
                f"With {c['stock_length']:.3f} mm stock and chuck face at machine Y{CHUCK_FACE_MACHINE_Y:.3f}, "
                f"the stock ends at machine Y{stock_tail_y:.3f}; Cut Y Max must be below "
                f"Y{tail_safe_limit_y:.3f} to preserve more than {WASTE_TAIL:.3f} mm tail waste."
            )
        if tail_safe_limit_y <= Y_MACHINE_MIN + 1e-9:
            e.append(
                f"Raw stock is too short for the configured geometry: its tail-safe limit is "
                f"Y{tail_safe_limit_y:.3f}, not beyond machine Y minimum Y{Y_MACHINE_MIN:.3f}."
            )
        if c["length"] <= 0 and c["cut_y_max_entered"]:
            e.append("Machining length must be positive from the machine Y minimum.")
        return e

    @staticmethod
    def validate_y(y, cutting=False):
        # The chuck is permanently outside the cut envelope. The hard lower
        # rotary machine limit is Y_MACHINE_MIN (Y0 in this build).
        if y < Y_MACHINE_MIN - 1e-9:
            raise ValueError(f"Y{y:.3f} is below hard Y_MACHINE_MIN {Y_MACHINE_MIN:.3f}.")
        if y > Y_MACHINE_MAX + 1e-9:
            raise ValueError(f"Y{y:.3f} exceeds hard Y_MACHINE_MAX {Y_MACHINE_MAX:.3f}.")

    # ------------------------------------------------------------------
    # Tab 1 model: no-trench progressive indexed rounding
    # ------------------------------------------------------------------
    def build_round_model(self, c, cutter_dia):
        a = c["stock"] / 2.0
        target_r = c["round_dia"] / 2.0
        poly = [(-a, -a), (a, -a), (a, a), (-a, a)]
        snapshots = [("Raw square", list(poly))]
        stages = []

        definitions = [
            ("45° corners", [45.0 + 90.0 * k for k in range(4)]),
            ("22.5° interleave", [22.5 + 45.0 * k for k in range(8)]),
            ("11.25° interleave", [11.25 + 22.5 * k for k in range(16)]),
        ]

        for label, angles in definitions:
            pre = list(poly)
            # Symmetric geometry means these should be identical, but calculate
            # each independently and retain the most conservative (largest radius).
            targets = [deepest_no_trench_radius(pre, ang, target_r, cutter_dia) for ang in angles]
            stage_target = max(targets)
            start_support = max(polygon_support(pre, ang) for ang in angles)
            facet_width = max(polygon_line_width(pre, ang, stage_target) for ang in angles)

            for ang in angles:
                poly = clip_polygon_halfplane(poly, ang, stage_target)

            stages.append({
                "label": label,
                "angles": angles,
                "start_support": start_support,
                "target_radius": stage_target,
                "facet_width": facet_width,
            })
            snapshots.append((label, list(poly)))

        cleanup = max(0.0, max_polygon_radius(poly) - target_r)

        # If the 32-ish state still leaves too much radial cleanup for the
        # finishing spiral, add the 5.625-degree interleave.
        if cleanup > ROUND_SPIRAL_MAX_CLEANUP + 1e-9:
            label = "5.625° interleave"
            angles = [5.625 + 11.25 * k for k in range(32)]
            pre = list(poly)
            targets = [deepest_no_trench_radius(pre, ang, target_r, cutter_dia) for ang in angles]
            stage_target = max(targets)
            start_support = max(polygon_support(pre, ang) for ang in angles)
            facet_width = max(polygon_line_width(pre, ang, stage_target) for ang in angles)
            for ang in angles:
                poly = clip_polygon_halfplane(poly, ang, stage_target)
            stages.append({
                "label": label,
                "angles": angles,
                "start_support": start_support,
                "target_radius": stage_target,
                "facet_width": facet_width,
            })
            snapshots.append((label, list(poly)))
            cleanup = max(0.0, max_polygon_radius(poly) - target_r)

        return {
            "stages": stages,
            "snapshots": snapshots,
            "polygon": poly,
            "cleanup": cleanup,
            "target_radius": target_r,
        }

    # ------------------------------------------------------------------
    # Wave geometry
    # ------------------------------------------------------------------
    def wave_radius(self, x_local, c=None):
        if c is None:
            c = self.common_values()
        dmax = c["round_dia"]
        dmin = self._f("wave_dmin")
        waves = self._i("waves")
        length = c["length"]
        if waves == 0:
            return dmax / 2.0
        rmid = (dmax + dmin) / 4.0
        ramp = (dmax - dmin) / 4.0
        sign = 1.0 if waves > 0 else -1.0
        return rmid + sign * ramp * math.cos(2.0 * math.pi * abs(waves) * x_local / length)

    def sample_wave(self, c, shell_floor=None, allowance=0.0, step=PROFILE_SAMPLE_MM):
        count = max(2, math.ceil(c["length"] / step))
        pts = []
        for i in range(count + 1):
            x = c["length"] * i / count
            target = self.wave_radius(x, c) + allowance
            z = target if shell_floor is None else max(target, shell_floor)
            pts.append((x, z))
        return pts

    def compensate_ball_nose_tip_path(self, surface_points, tool_diameter):
        """
        Convert a desired X/Z surface curve into the programmed ball-nose TIP path.

        Z is touched off at the lowest point of the ball nose.  For a desired
        surface z=f(x), the ball centre follows the outward normal by tool radius R:

            centre = surface + R * (-f', 1) / sqrt(1 + f'^2)

        The programmed tool-tip coordinate is one radius below the centre:

            X_tip = x - R*f'/sqrt(1+f'^2)
            Z_tip = z + R/sqrt(1+f'^2) - R

        On a horizontal surface f'=0, X_tip=x and Z_tip=z, which preserves the
        established Z touch-off convention.  On a slope, cutter diameter changes
        both the X lead/lag and the small Z correction.
        """
        if len(surface_points) < 2:
            return list(surface_points)

        radius = tool_diameter / 2.0
        compensated = []

        for i, (x, z) in enumerate(surface_points):
            if i == 0:
                x0, z0 = surface_points[i]
                x1, z1 = surface_points[i + 1]
            elif i == len(surface_points) - 1:
                x0, z0 = surface_points[i - 1]
                x1, z1 = surface_points[i]
            else:
                x0, z0 = surface_points[i - 1]
                x1, z1 = surface_points[i + 1]

            dx = x1 - x0
            slope = (z1 - z0) / dx if abs(dx) > 1e-12 else 0.0
            denom = math.sqrt(1.0 + slope * slope)

            x_tip = x - radius * slope / denom
            z_tip = z + radius / denom - radius

            # Integer-cycle cosine profiles have zero slope at each end.
            # Clamp tiny numerical excursions to the inherited machining range.
            x_tip = min(max(x_tip, 0.0), surface_points[-1][0])
            compensated.append((x_tip, z_tip))

        # Numerical compensation should remain monotonic for the allowed geometry.
        # Sort defensively and de-duplicate identical X values.
        compensated.sort(key=lambda p: p[0])
        clean = []
        for p in compensated:
            if clean and abs(p[0] - clean[-1][0]) < 1e-9:
                if p[1] < clean[-1][1]:
                    clean[-1] = p
            else:
                clean.append(p)
        return clean

    @staticmethod
    def compensation_stats(surface_points, toolpath_points):
        if not surface_points or not toolpath_points:
            return 0.0, 0.0

        # Compare each original point against the closest compensated point by
        # sequence position; both are densely and monotonically sampled.
        n = min(len(surface_points), len(toolpath_points))
        max_dx = 0.0
        max_dz = 0.0
        for i in range(n):
            max_dx = max(max_dx, abs(toolpath_points[i][0] - surface_points[i][0]))
            max_dz = max(max_dz, abs(toolpath_points[i][1] - surface_points[i][1]))
        return max_dx, max_dz

    # ------------------------------------------------------------------
    # Recalculate all tabs
    # ------------------------------------------------------------------
    def recalculate(self):
        if self._recalc_busy:
            return
        self._recalc_busy = True
        try:
            # Reset colours.
            for e in self.entries.values():
                try:
                    e.configure(bg=self.palette["editable_ok"])
                except tk.TclError:
                    pass

            c = self.common_values()
            errors = self.common_errors(c)
            job_entry = self.entries.get("job_name")
            if job_entry is not None:
                job_entry.configure(bg=self.palette["editable_ok"] if self.v["job_name"].get().strip() else self.palette["editable_bad"])
            y_max_entry = self.entries.get("cut_y_max")
            if y_max_entry is not None:
                tail_safe_limit_y = CHUCK_FACE_MACHINE_Y + c["stock_length"] - WASTE_TAIL
                y_ok = (
                    c["cut_y_max_entered"]
                    and Y_MACHINE_MIN < c["cut_y_max"] <= Y_MACHINE_MAX
                    and c["cut_y_max"] < tail_safe_limit_y
                )
                y_max_entry.configure(bg=self.palette["editable_ok"] if y_ok else self.palette["editable_bad"])
            # Y geometry does not depend on cutter length.
            dummy_center = 0.0
            ys, ye, cr, _, _ = self.common_geometry(c, dummy_center)
            self.calc["cut_y_start"].set(f"{ys:.3f}")
            self.calc["cut_y_end"].set(f"{ye:.3f}" if c["cut_y_max_entered"] else "REQUIRED")
            self.calc["corner_radius"].set(f"{cr:.3f}")
            self.calc["z_zero_square"].set("")

            # Each stage has its own cutter and therefore its own mandatory Probe 1 value.
            centers = {}
            for stage, key in (("round", "round_carriage_probe_z"), ("wave", "wave_carriage_probe_z"), ("taper", "taper_carriage_probe_z"), ("groove", "groove_carriage_probe_z")):
                try:
                    cz = self.probe_center_z(key)
                    centers[stage] = cz
                    self.calc[f"{stage}_a_center_z"].set(f"{cz:.3f}")
                    self.calc[f"{stage}_z_cut_min"].set(f"{self.deepest_allowed_z(cz):.3f}")
                    self.entries[key].configure(bg=self.palette["editable_ok"])
                except Exception:
                    centers[stage] = None
                    self.calc[f"{stage}_a_center_z"].set("REQUIRED")
                    self.calc[f"{stage}_z_cut_min"].set("REQUIRED")
                    if key in self.entries:
                        self.entries[key].configure(bg=self.palette["editable_bad"])
            if centers["round"] is not None:
                _, _, _, sz, stock_top_z = self.common_geometry(c, centers["round"])
                self.calc["safe_z"].set(f"{sz:.3f}")
                self.calc["z_zero_square"].set(f"{stock_top_z:.3f}")
            else:
                self.calc["safe_z"].set("REQUIRED")

            stamp = datetime.now().strftime("%Y%m%d_%H%M")
            self.calc["round_file"].set(f"{c['job']}_ROUND_{stamp}.nc")
            self.calc["wave_file"].set(f"{c['job']}_WAVE_{stamp}.nc")
            self.calc["taper_file"].set(f"{c['job']}_TAPER_{stamp}.nc")
            self.calc["groove_file"].set(f"{c['job']}_GROOVE_{stamp}.nc")

            # ROUND
            try:
                tool = self._f("round_tool_dia")
                doc = self._f("round_doc")
                rpm = self._f("round_spiral_rpm")
                pitch = self._f("round_spiral_pitch")
                if tool <= 0 or doc <= 0 or rpm <= 0 or pitch <= 0:
                    raise ValueError("ROUND cutter, DOC, RPM and pitch must be positive.")
                if rpm > RPM_MAX + 1e-9:
                    raise ValueError(f"ROUND RPM exceeds RPM_MAX {RPM_MAX:.3f}.")
                if self._f("round_feed") <= 0 or self._f("round_plunge") <= 0:
                    raise ValueError("ROUND feeds must be positive.")
                if centers["round"] is None:
                    raise ValueError("ROUND requires the RAW Probe 1 G53 Z from the 40 mm block on the spoilboard.")
                if c["stock_shape"] != "SQUARE":
                    raise ValueError("ROUND indexed roughing is for SQUARE raw stock; ROUND stock can proceed directly to WAVE when already at the required blank diameter.")
                model = self.build_round_model(c, tool)
                self._round_model = model
                stage_names = " -> ".join(s["label"].split("°")[0].strip() + "°" for s in model["stages"])
                self.calc["round_stages"].set(stage_names)
                self.calc["round_cleanup"].set(f"{model['cleanup']:.3f}")
                revs = max(1, math.ceil(c["length"] / pitch))
                actual = c["length"] / revs
                xfeed = actual * rpm
                self.calc["round_revs"].set(str(revs))
                self.calc["round_actual_pitch"].set(f"{actual:.3f}")
                self.calc["round_y_feed"].set(f"{xfeed:.3f}")
                round_errors = list(errors)
                if model["cleanup"] > ROUND_SPIRAL_MAX_CLEANUP + 1e-9:
                    round_errors.append("Even the 64-stage leaves too much cleanup for the final spiral.")
            except Exception as exc:
                model = None
                self._round_model = None
                round_errors = list(errors) + [str(exc)]

            round_checks = centers["round"] is not None and self.round_square_diagonal.get() and self.round_rollover.get()
            self._set_status(self.round_status_box, self.calc["round_status"], self.round_btn, round_errors, round_checks,
                             "ROUND geometry ready. Enter Probe 1 Z, confirm A0 diagonal/corner-up stock orientation and rollover OFF.",
                             "Ready to generate ROUND program.")

            # WAVE
            wave_errors = list(errors)
            try:
                if centers["wave"] is None:
                    wave_errors.append("WAVE requires the RAW Probe 1 G53 Z from the 40 mm block on the spoilboard.")
                dmin = self._f("wave_dmin")
                waves = self._i("waves")
                wdoc = self._f("wave_doc")
                allow = self._f("wave_finish_allow")
                if waves not in (-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5):
                    wave_errors.append("Waves must be -5 to +5.")
                if dmin < Z_MIN_DIAMETER - 1e-9 or dmin > c["round_dia"] + 1e-9:
                    wave_errors.append(f"WAVE minimum diameter must be >= {Z_MIN_DIAMETER:.3f} and <= inherited round diameter.")
                if wdoc <= 0:
                    wave_errors.append("WAVE rough DOC must be positive.")
                if allow < 0:
                    wave_errors.append("WAVE finish allowance cannot be negative.")
                if self._f("wave_tool_dia") <= 0 or self._i("wave_flutes") <= 0 or self._f("wave_feed") <= 0 or self._f("wave_plunge") <= 0 or self._f("wave_spiral_rpm") <= 0 or self._f("wave_spiral_pitch") <= 0:
                    wave_errors.append("WAVE cutter diameter, flute count, feeds, RPM and pitch must be positive.")
                if self._f("wave_spiral_rpm") > RPM_MAX + 1e-9:
                    wave_errors.append(f"WAVE RPM exceeds RPM_MAX {RPM_MAX:.3f}.")
                # Waves=0 is a cylinder: lock Dmin to inherited Dmax while
                # retaining the previous non-zero Dmin for restoration.
                if waves == 0:
                    self._set_wave_dmin_state(True)
                else:
                    self._set_wave_dmin_state(False)

                amp = (c["round_dia"] - dmin) / 4.0
                self.calc["wave_start_radius"].set(f"{c['round_dia']/2.0:.3f}")
                self.calc["wave_amplitude"].set(f"{amp:.3f}")

                tool_dia = self._f("wave_tool_dia")
                tool_r = tool_dia / 2.0
                self.calc["wave_tool_radius"].set(f"{tool_r:.3f}")

                # Cutter/profile sanity.  For r(x)=r_mid+A*cos(kx), the tightest
                # local radius of curvature occurs at the wave crest/valley and
                # is 1/(A*k^2).  A ball nose larger than the concave profile
                # radius cannot physically reproduce the requested wave without
                # gouging/bridging the valley.
                if waves == 0 or abs(amp) <= 1e-12:
                    wavelength = float("inf")
                    min_curve_radius = float("inf")
                    self.calc["wave_wavelength"].set("Cylinder")
                    self.calc["wave_min_curve_radius"].set("N/A")
                else:
                    wavelength = c["length"] / abs(waves)
                    k = 2.0 * math.pi * abs(waves) / c["length"]
                    min_curve_radius = 1.0 / (abs(amp) * k * k)
                    self.calc["wave_wavelength"].set(f"{wavelength:.3f}")
                    self.calc["wave_min_curve_radius"].set(f"{min_curve_radius:.3f}")
                    if tool_r >= min_curve_radius - 1e-9:
                        wave_errors.append(
                            f"WAVE density is too high for the selected Ø{tool_dia:.3f} mm ball nose: "
                            f"{abs(waves)} waves over {c['length']:.3f} mm gives {wavelength:.3f} mm/wave, "
                            f"but the profile tightens to R{min_curve_radius:.3f} mm while the cutter is R{tool_r:.3f} mm. "
                            f"Use fewer waves, a smaller ball nose, a longer Y traverse, or a shallower wave."
                        )

                surface = self.sample_wave(
                    c, shell_floor=None, allowance=0.0,
                    step=max(0.25, c["length"] / 1000.0)
                )
                compensated = self.compensate_ball_nose_tip_path(surface, tool_dia)
                max_dx, max_dz = self.compensation_stats(surface, compensated)
                self.calc["wave_comp_x"].set(f"{max_dx:.3f}")
                self.calc["wave_comp_z"].set(f"{max_dz:.3f}")

                if allow + 1e-9 < max_dz:
                    wave_errors.append(
                        f"Finish allowance {allow:.3f} mm is smaller than the "
                        f"ball-nose slope protection requirement {max_dz:.3f} mm."
                    )

                station_model = self.build_wave_station_model(c, tool_dia, wdoc, allow)
                self.calc["wave_station_step"].set(f"{station_model['step']:.3f}")
                self.calc["wave_station_count"].set(str(station_model["station_count"]))
                self.calc["wave_ring_count"].set(str(station_model["ring_count"]))
                self.calc["wave_sweep_count"].set(str(station_model["sweep_count"]))
                self.calc["wave_shells"].set(str(station_model["sweep_count"]))

                wpitch = self._f("wave_spiral_pitch")
                wrpm = self._f("wave_spiral_rpm")
                timing = self.wave_runtime_model(
                    c, station_model, wrpm, wpitch, self._f("wave_plunge")
                )
                self.calc["wave_revs"].set(str(timing["finish_revs"]))
                self.calc["wave_actual_pitch"].set(f"{timing['actual_pitch']:.3f}")
                self.calc["wave_y_feed"].set(f"{timing['finish_x_feed']:.3f}")
                self.calc["wave_rough_time"].set(f"{timing['rough_minutes']:.1f}")
                self.calc["wave_finish_time"].set(f"{timing['finish_minutes']:.1f}")
                self.calc["wave_total_time"].set(f"{timing['total_minutes']:.1f}")

                total_a = station_model["rough_a_degrees"] + timing["finish_revs"] * 360.0
                self.calc["wave_total_a"].set(f"{total_a:.0f}")

                # WAVE now uses one literal accumulated A coordinate throughout
                # the program.  No internal G92 rebasing is used.  Every roughing
                # ring and the integer-revolution finish end at the ORIGINAL A0
                # physical phase, so the stage can hand a deterministic phase to
                # downstream operations.
                if total_a > A_SOFT_LIMIT_ABS + 1e-9:
                    wave_errors.append(
                        f"WAVE accumulated A travel {total_a:.0f} deg exceeds the configured "
                        f"UCCNC A soft limit {A_SOFT_LIMIT_ABS:.0f} deg."
                    )
                phase_error = abs(total_a % 360.0)
                if min(phase_error, abs(360.0-phase_error)) > 1e-7:
                    wave_errors.append("WAVE final A phase is not the original A0; this is unsafe for downstream GROOVE/TAPER handoff.")
                    self.calc["wave_final_phase"].set("ERROR")
                else:
                    self.calc["wave_final_phase"].set("A0 (0 deg)")
            except Exception as exc:
                wave_errors.append(str(exc))

            wave_checks = (
                centers["wave"] is not None
                and self.wave_tool_changed.get()
                and self.wave_rollover.get()
            )
            self._set_status(self.wave_status_box, self.calc["wave_status"], self.wave_btn, wave_errors, wave_checks,
                             "WAVE geometry ready. Enter Probe 1 Z and complete tool-change checks.",
                             "Ready to generate WAVE program.")

            # TAPER
            taper_errors = list(errors)
            try:
                if centers["taper"] is None:
                    taper_errors.append("TAPER requires the RAW Probe 1 G53 Z from the 40 mm block on the spoilboard.")
                if self._i("waves") != 0:
                    taper_errors.append("TAPER is locked out while WAVE count is non-zero. Set WAVE to 0 first.")
                d0 = self._f("taper_start_dia")
                d1 = self._f("taper_end_dia")
                ty0 = self._f("taper_y_start")
                raw_ty1 = self.v["taper_y_end"].get().strip()
                if not raw_ty1:
                    raise ValueError("TAPER Y End is mandatory.")
                ty1 = float(raw_ty1)
                tool = self._f("taper_tool_dia")
                flutes = self._i("taper_flutes")
                feed = self._f("taper_feed")
                plunge = self._f("taper_plunge")
                doc = self._f("taper_doc")
                rpm = self._f("taper_rpm")
                pitch = self._f("taper_pitch")
                if min(d0, d1) < MIN_TAPER_DIAMETER - 1e-9:
                    taper_errors.append(f"TAPER finished diameters must be >= {MIN_TAPER_DIAMETER:.3f} mm.")
                if max(d0, d1) > c["round_dia"] + 1e-9:
                    taper_errors.append("TAPER diameter cannot exceed the inherited ROUND blank diameter.")
                if abs(d0-d1) < 1e-9:
                    taper_errors.append("Start and end diameters are equal; this is a cylinder, not a taper.")
                tail_safe = CHUCK_FACE_MACHINE_Y + c["stock_length"] - WASTE_TAIL
                if ty0 < Y_MACHINE_MIN - 1e-9:
                    taper_errors.append(f"TAPER Y Start must be >= Y{Y_MACHINE_MIN:.3f}.")
                if ty1 <= ty0 + 1e-9:
                    taper_errors.append("TAPER Y End must be greater than TAPER Y Start.")
                if ty1 > Y_MACHINE_MAX + 1e-9:
                    taper_errors.append(f"TAPER Y End exceeds hard Y_MACHINE_MAX {Y_MACHINE_MAX:.3f}.")
                if ty1 >= tail_safe - 1e-9:
                    taper_errors.append(f"TAPER Y End must be below Y{tail_safe:.3f} to preserve tail waste.")
                if c["cut_y_max_entered"] and ty1 > c["cut_y_max"] + 1e-9:
                    taper_errors.append("TAPER Y End cannot exceed the operator-selected Cut Y Max from ROUND.")
                if min(tool, feed, plunge, doc, rpm, pitch) <= 0 or flutes <= 0:
                    taper_errors.append("TAPER cutter, flutes, feeds, DOC, RPM and pitch must be positive.")
                if rpm > RPM_MAX + 1e-9:
                    taper_errors.append(f"TAPER RPM exceeds RPM_MAX {RPM_MAX:.3f}.")
                tlen = ty1 - ty0
                dr = (d1-d0)/2.0
                angle = math.degrees(math.atan2(abs(dr), tlen)) if tlen > 0 else 0.0
                revs = max(1, math.ceil(tlen/pitch)) if pitch > 0 and tlen > 0 else 0
                actual_pitch = tlen/revs if revs else 0.0
                yfeed = actual_pitch*rpm if revs else 0.0
                self.calc["taper_length"].set(f"{tlen:.3f}")
                self.calc["taper_diameter_change"].set(f"{d1-d0:+.3f}")
                self.calc["taper_radius_change"].set(f"{dr:+.3f}")
                self.calc["taper_angle"].set(f"{angle:.3f}")
                self.calc["taper_min_dia"].set(f"{min(d0,d1):.3f}")
                self.calc["taper_max_dia"].set(f"{max(d0,d1):.3f}")
                self.calc["taper_revs"].set(str(revs) if revs else "")
                self.calc["taper_actual_pitch"].set(f"{actual_pitch:.3f}" if revs else "")
                self.calc["taper_y_feed"].set(f"{yfeed:.3f}" if revs else "")
                if tool > 0 and doc > 0 and ty1 > ty0:
                    rough_model = self._taper_rough_model(d0, d1, ty0, ty1, tool, doc, c["round_dia"])
                    rough_a_time = rough_model["ring_count"] / rpm if rpm > 0 else 0.0
                    finish_time = revs / rpm if rpm > 0 and revs else 0.0
                    self.calc["taper_rough_step"].set(f"{rough_model['step']:.3f}")
                    self.calc["taper_rough_stations"].set(str(rough_model["station_count"]))
                    self.calc["taper_rough_rings"].set(str(rough_model["ring_count"]))
                    self.calc["taper_rough_a_time"].set(f"{rough_a_time:.1f}")
                    self.calc["taper_finish_time"].set(f"{finish_time:.1f}")
                    self.calc["taper_total_a_time"].set(f"{rough_a_time+finish_time:.1f}")
            except Exception as exc:
                taper_errors.append(str(exc))

            taper_checks = centers["taper"] is not None and self.taper_tool_changed.get() and self.taper_rollover.get()
            self._set_status(self.taper_status_box, self.calc["taper_status"], self.taper_btn, taper_errors, taper_checks,
                             "TAPER geometry ready. Enter Probe 1 Z and complete tool-change checks.",
                             "Ready to generate TAPER program.")

            # GROOVE
            groove_errors = list(errors)
            try:
                if centers["groove"] is None:
                    groove_errors.append("GROOVE requires the RAW Probe 1 G53 Z from the 40 mm block on the spoilboard.")
                depth = self._f("groove_depth")
                gdoc = self._f("groove_doc")
                gpitch = self._f("groove_pitch")
                starts = self._i("groove_starts")
                source = self.groove_source()
                groove_errors.extend(self.groove_source_errors(c))
                self.calc["groove_source_display"].set(
                    f"ACTIVE GROOVE SOURCE: {source}" if source in ("ROUND", "WAVE", "TAPER")
                    else "SELECT ROUND / WAVE / TAPER"
                )
                gtool = self._f("groove_tool_dia")
                if source in ("ROUND", "WAVE", "TAPER"):
                    py0, py1, gy0, gy1, gmargin = self.groove_cut_y_bounds(c, gtool, source)
                    self.calc["groove_parent_y_start"].set(f"{py0:.3f}")
                    self.calc["groove_parent_y_end"].set(f"{py1:.3f}")
                    self.calc["groove_cut_y_start"].set(f"{gy0:.3f}")
                    self.calc["groove_cut_y_end"].set(f"{gy1:.3f}")
                    base_min_dia = self.groove_base_min_diameter(c, source, gtool)
                else:
                    self.calc["groove_parent_y_start"].set("")
                    self.calc["groove_parent_y_end"].set("")
                    self.calc["groove_cut_y_start"].set("")
                    self.calc["groove_cut_y_end"].set("")
                    base_min_dia = c["round_dia"]
                root = base_min_dia - 2.0 * depth
                lead = gpitch * starts
                sep = 360.0 / starts if starts > 0 else 0.0
                passes = math.ceil(depth / gdoc - 1e-12) if depth > 0 and gdoc > 0 else 0
                self.calc["groove_root"].set(f"{root:.3f}")
                self.calc["groove_passes"].set(str(passes) if passes else "")
                self.calc["groove_lead"].set(f"{lead:.3f}")
                self.calc["groove_separation"].set(f"{sep:.3f}")
                if depth <= 0:
                    groove_errors.append("Groove depth must be positive.")
                if gdoc <= 0:
                    groove_errors.append("Groove max radial DOC per pass must be positive.")
                if gpitch <= 0:
                    groove_errors.append("Groove pitch must be positive.")
                if starts < 1:
                    groove_errors.append("Groove starts must be at least 1.")
                if root < Z_MIN_DIAMETER - 1e-9:
                    groove_errors.append(f"Groove root diameter {root:.3f} is below Z_MIN_DIAMETER {Z_MIN_DIAMETER:.3f} mm.")
                if self._f("groove_tool_dia") <= 0 or self._f("groove_feed") <= 0 or self._f("groove_plunge") <= 0 or self._f("groove_rpm") <= 0:
                    groove_errors.append("GROOVE cutter, feeds and RPM must be positive.")
                if self._f("groove_rpm") > RPM_MAX + 1e-9:
                    groove_errors.append(f"GROOVE RPM exceeds RPM_MAX {RPM_MAX:.3f}.")
            except Exception as exc:
                groove_errors.append(str(exc))

            if not self.v["groove_enable"].get():
                self.calc["groove_status"].set("Grooves disabled.")
                self.groove_status_box.configure(bg=self.palette["status_bg"], fg="#ffffff")
                self.groove_btn.state(["disabled"])
            else:
                groove_checks = centers["groove"] is not None and self.groove_tool_changed.get() and self.groove_rollover.get()
                self._set_status(self.groove_status_box, self.calc["groove_status"], self.groove_btn, groove_errors, groove_checks,
                                 "GROOVE geometry ready. Enter Probe 1 Z and complete tool-change checks.",
                                 "Ready to generate GROOVE program.")

            self.update_round_preview(model)
            self.update_wave_preview(c)
            self.update_taper_preview(c)
            self.update_groove_preview(c)

        except Exception as exc:
            msg = f"Invalid input: {exc}"
            for var in (self.calc["round_status"], self.calc["wave_status"], self.calc["taper_status"], self.calc["groove_status"]):
                var.set(msg)
            for box in (self.round_status_box, self.wave_status_box, self.taper_status_box, self.groove_status_box):
                box.configure(bg="#fbe4e4", fg=self.palette["unsafe"])
            for btn in (self.round_btn, self.wave_btn, self.taper_btn, self.groove_btn):
                btn.state(["disabled"])
        finally:
            self._recalc_busy = False

    def _set_status(self, box, var, btn, errors, checks_ok, pending, ready):
        # De-duplicate cascading inherited errors.
        dedup = []
        for e in errors:
            if e and e not in dedup:
                dedup.append(e)
        if dedup:
            var.set("NOT SAFE: " + dedup[0])
            box.configure(bg="#fbe4e4", fg=self.palette["unsafe"])
            btn.state(["disabled"])
        elif not checks_ok:
            var.set(pending)
            box.configure(bg=self.palette["status_bg"], fg="#ffffff")
            btn.state(["disabled"])
        else:
            var.set(ready)
            box.configure(bg="#e9f5eb", fg=self.palette["safe"])
            btn.state(["!disabled"])

    # ------------------------------------------------------------------
    # Previews
    # ------------------------------------------------------------------
    def update_round_preview(self, model):
        c = self.round_canvas
        c.delete("all")
        if not model:
            return

        w = max(c.winfo_width(), 600)
        h = max(c.winfo_height(), 300)
        snaps = model["snapshots"]
        stages = model["stages"]
        n = len(snaps)
        cell = w / n
        target_r = model["target_radius"]
        all_r = max(max_polygon_radius(poly) for _, poly in snaps)

        # Leave deliberate room for:
        #   title + subtitle at top,
        #   geometry labels and dimensions at bottom.
        top_margin = 66
        bottom_margin = 105
        drawable_h = max(100, h - top_margin - bottom_margin)
        scale = min(cell * 0.33, drawable_h * 0.46) / max(all_r, 1e-6)
        cy = top_margin + drawable_h * 0.48

        # Two-line heading so it cannot run off the right edge.
        c.create_text(
            w / 2, 16,
            text="Square → progressive no-trench interleaves → roundish",
            font=("Segoe UI", 10, "bold"),
            fill="#ffffff",
            anchor="n",
        )
        c.create_text(
            w / 2, 36,
            text="Final operation: constant-radius spiral cleanup",
            font=("Segoe UI", 9),
            fill="#9fd3ff",
            anchor="n",
        )

        for idx, (label, poly) in enumerate(snaps):
            cx = cell * (idx + 0.5)

            # Target finished circle.
            rr = target_r * scale
            c.create_oval(
                cx - rr, cy - rr, cx + rr, cy + rr,
                outline="#2e8fe6", dash=(4, 3)
            )

            pts = []
            for x, y in poly:
                pts.extend((cx + x * scale, cy - y * scale))
            c.create_polygon(
                *pts,
                fill="#202020",
                outline="#d0d0d0",
                width=2
            )

            # Phase name.
            c.create_text(
                cx, h - 79,
                text=label,
                width=max(90, cell - 12),
                font=("Segoe UI", 9, "bold"),
                fill="#ffffff",
                justify="center",
            )

            # Dimensional callouts for each phase.
            max_d = 2.0 * max_polygon_radius(poly)
            if idx == 0:
                stock = max(abs(x) for x, _ in poly) * 2.0
                dim_text = (
                    f"{stock:.3f} mm across flats\n"
                    f"{max_d:.3f} mm corner diagonal"
                )
            else:
                stage = stages[idx - 1]
                dim_text = (
                    f"Cut radius {stage['target_radius']:.3f} mm\n"
                    f"Facet width {stage['facet_width']:.3f} mm\n"
                    f"Remaining max Ø {max_d:.3f} mm"
                )

            c.create_text(
                cx, h - 47,
                text=dim_text,
                width=max(100, cell - 10),
                font=("Segoe UI", 8),
                fill="#ffffff",
                justify="center",
                anchor="n",
            )

            if idx < n - 1:
                c.create_text(
                    cell * (idx + 1), cy,
                    text="→",
                    font=("Segoe UI", 18, "bold"),
                    fill="#9fd3ff",
                )

    def update_wave_preview(self, common):
        c = self.wave_canvas
        c.delete("all")
        try:
            w = max(c.winfo_width(), 600)
            h = max(c.winfo_height(), 300)
            mx = 45
            top_margin = 62
            bottom_margin = 38
            cy = (top_margin + (h - bottom_margin)) / 2.0
            length = max(common["length"], 1.0)
            rmax = common["round_dia"] / 2.0
            scale = ((h - bottom_margin - top_margin) / 2.0 - 8) / max(rmax, 1.0)

            def sx(x):
                return mx + (w - 2 * mx) * x / length

            surface = self.sample_wave(
                common, shell_floor=None, allowance=0.0,
                step=max(0.25, length / 1000.0)
            )
            allow = self._f("wave_finish_allow")
            rough_surface = [(x, min(rmax, z + allow)) for x, z in surface]

            top = [(sx(x), cy - r * scale) for x, r in surface]
            bot = [(sx(x), cy + r * scale) for x, r in reversed(surface)]
            flat = [q for p in top + bot for q in p]
            c.create_polygon(*flat, fill="#202020", outline="#d0d0d0", width=2)

            rr = rmax * scale
            c.create_rectangle(mx, cy - rr, w - mx, cy + rr,
                               outline="#2e8fe6", dash=(4, 3))
            c.create_line(mx, cy, w - mx, cy, fill="#707070", dash=(3, 3))

            rough_line = []
            for x, z in rough_surface:
                rough_line.extend((sx(x), cy - z * scale))
            c.create_line(*rough_line, fill="#4ac26b", dash=(5, 3), width=2)

            model = self.build_wave_station_model(
                common, self._f("wave_tool_dia"), self._f("wave_doc"), allow
            )
            for st in model["stations"]:
                xx = sx(st["x"])
                c.create_line(xx, cy - rr, xx, cy + rr,
                              fill="#404040", dash=(2, 5))

            tool_dia = self._f("wave_tool_dia")
            toolpts = self.compensate_ball_nose_tip_path(surface, tool_dia)
            tool_line = []
            for x, z in toolpts:
                tool_line.extend((sx(x), cy - z * scale))
            if len(tool_line) >= 4:
                c.create_line(*tool_line, fill="#f0a34a", dash=(7, 3), width=2)

            c.create_text(
                w / 2, 16,
                text=(f"Cosine WAVE roughing: {model['station_count']} Y stations, "
                      f"{model['ring_count']} complete rings"),
                font=("Segoe UI", 10, "bold"), fill="#ffffff",
            )
            c.create_text(
                w / 2, 37,
                text=(f"Solid = finished   Green dashed = +{allow:.3f} rough target   "
                      f"Brown dashed = compensated finish path"),
                font=("Segoe UI", 8), fill="#9fd3ff",
            )
        except Exception:
            pass

    def update_taper_preview(self, common):
        c=self.taper_canvas
        c.delete("all")
        w=max(c.winfo_width(),600); h=max(c.winfo_height(),300)
        try:
            y0=self._f("taper_y_start"); y1=self._f("taper_y_end")
            d0=self._f("taper_start_dia"); d1=self._f("taper_end_dia")
            if y1<=y0:
                return
            margin=55; mid=h/2.0
            sx=(w-2*margin)/(y1-y0)
            maxr=max(d0,d1,common["round_dia"])/2.0
            sz=(h-90)/(2*maxr) if maxr>0 else 1.0
            x0=margin; x1=w-margin
            r0=d0/2.0*sz; r1=d1/2.0*sz
            c.create_text(w/2,20,text="TAPER profile (side view)",fill="#ffffff",font=("Segoe UI",11,"bold"))
            c.create_polygon(x0,mid-r0,x1,mid-r1,x1,mid+r1,x0,mid+r0,outline="#66a3d2",fill="",width=2)
            c.create_line(x0,mid,x1,mid,fill="#666666",dash=(4,4))
            c.create_text(x0,mid-r0-18,text=f"D{d0:.1f}",fill="#dddddd",anchor="w")
            c.create_text(x1,mid-r1-18,text=f"D{d1:.1f}",fill="#dddddd",anchor="e")
            c.create_text(w/2,h-22,text=f"Y{y0:.1f} → Y{y1:.1f}",fill="#aaaaaa")
        except Exception:
            return

    def update_groove_preview(self, common):
        c=self.groove_canvas; c.delete("all")
        try:
            source = self.groove_source()
            w=max(c.winfo_width(),600); h=max(c.winfo_height(),300)
            if source not in ("ROUND", "WAVE", "TAPER"):
                c.create_text(w/2,h/2,text="SELECT GROOVE BASE PROFILE: ROUND / WAVE / TAPER",font=("Segoe UI",12,"bold"),fill="#ffffff")
                return
            tool = self._f("groove_tool_dia")
            parent_y0, parent_y1, gy0, gy1, margin = self.groove_cut_y_bounds(common, tool, source)
            parent_len=max(parent_y1-parent_y0,1.0)
            mx=40; cy=h/2
            rmax=common["round_dia"]/2.0
            scale=(h/2-42)/max(rmax,1.0)
            def sx(y): return mx+(w-2*mx)*(y-parent_y0)/parent_len
            count=max(2, math.ceil(parent_len/max(0.5,parent_len/500)))
            pts=[(parent_y0+parent_len*i/count, self.base_profile_radius_machine_y(parent_y0+parent_len*i/count, common, source)) for i in range(count+1)]
            top=[(sx(y),cy-r*scale) for y,r in pts]
            bot=[(sx(y),cy+r*scale) for y,r in reversed(pts)]
            c.create_polygon(*[q for p in top+bot for q in p], fill="#202020", outline="#d0d0d0", width=2)
            starts=max(1,self._i("groove_starts")); pitch=self._f("groove_pitch")
            groove_len=gy1-gy0
            for s in range(starts):
                phase=2*math.pi*s/starts
                line=[]
                for i in range(401):
                    y=gy0+groove_len*i/400
                    r=self.base_profile_radius_machine_y(y,common,source)
                    ang=2*math.pi*(y-gy0)/max(pitch,1e-6)+phase
                    yy=cy-r*scale*0.68*math.cos(ang)
                    line.extend((sx(y),yy))
                c.create_line(*line,fill="#2e8fe6",width=2)
            c.create_line(sx(gy0), 38, sx(gy0), h-28, fill="#777777", dash=(3,3))
            c.create_line(sx(gy1), 38, sx(gy1), h-28, fill="#777777", dash=(3,3))
            c.create_text(w/2,18,text=f"{source} base profile + {starts}-start {self.v['groove_hand'].get()} groove",font=("Segoe UI",11,"bold"),fill="#ffffff")
            c.create_text(w/2,h-15,text=f"GROOVE centreline Y{gy0:.1f} → Y{gy1:.1f}  ({margin:.2f} mm edge setback)",font=("Segoe UI",8),fill="#aaaaaa")
        except Exception:
            pass

    # ------------------------------------------------------------------
    # G-code helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _header(stage, tool_dia, safe_z, center_z, probe_z):
        return [
            f"( {APP_TITLE} )",
            f"( STAGE: {stage} )",
            "( ALL XYZ coordinates are G53 machine coordinates )",
            f"( A centreline machine X = {A_CENTER_X:.3f} mm )",
            f"( Probe 1 RAW Z on 40 mm block over spoilboard = {probe_z:.3f} )",
            f"( Probe correction: -{PROBE1_BLOCK_HEIGHT:.3f} + {SPOILBOARD_TO_40120_TOP:.3f} + {LATHE_CARRIAGE_TOP_TO_A_CENTER:.3f} = {PROBE1_RAW_TO_A_CENTER:+.3f} mm )",
            f"( Derived A centre Z = {center_z:.3f} )",
            f"( Deepest legal cutting Z = {center_z + Z_MIN_DIAMETER/2.0:.3f} for minimum diameter {Z_MIN_DIAMETER:.3f} mm )",
            '( UCCNC A-axis "Roll over on 360" must be OFF )',
            "( Program end uses G92 A0.000; no physical A unwind )",
            "",
            "G21 G90 G64 G40",
            f"G53 G0 Z{safe_z:.3f}",
            f"G53 G0 X{A_CENTER_X:.3f}",
            f"( T0 : {tool_dia:.3f} )",
            "T0 M6",
            "",
        ]

    @staticmethod
    def _footer(safe_z):
        return [
            "",
            f"G53 G0 Z{safe_z:.3f}",
            "G92 A0.000",
            "M5",
            "M30",
        ]

    @staticmethod
    def _segment_feed(dy, dz, da_deg, rpm, fallback_feed):
        if rpm <= 0 or abs(da_deg) < 1e-12:
            return fallback_feed
        minutes = (abs(da_deg) / 360.0) / rpm
        dist = math.hypot(dy, dz)
        if dist < 1e-9 or minutes <= 1e-12:
            return fallback_feed
        return dist / minutes

    def _helical_profile_pass(self, g, common, z_points, safe_z, center_z, rpm, pitch, plunge, nominal_feed, start_a, hand_sign=1.0, comment="", y_start=None, y_end=None):
        """Generate one phase-locked helical pass and return its ending A coordinate."""
        default_ys, default_ye, _, _, _ = self.common_geometry(common, center_z)
        ys = default_ys if y_start is None else y_start
        ye = default_ye if y_end is None else y_end
        self.validate_y(ys, cutting=True); self.validate_y(ye, cutting=True)
        length = ye - ys
        revs = max(1, math.ceil(length / pitch))
        actual_pitch = length / revs
        total_a = revs * 360.0 * hand_sign
        end_a = start_a + total_a

        if abs(start_a) > A_SOFT_LIMIT_ABS + 1e-9 or abs(end_a) > A_SOFT_LIMIT_ABS + 1e-9:
            raise ValueError(f"Helical pass A coordinate exceeds A_SOFT_LIMIT_ABS {A_SOFT_LIMIT_ABS:.0f}.")
        self.validate_rpm(rpm, "Helical")

        if comment:
            g.append(f"( {comment} )")
        g.append(f"( Phase-locked start A {start_a:.3f}; {revs} rev; actual pitch {actual_pitch:.4f} mm/rev )")
        g.append(f"G53 G0 Z{safe_z:.3f}")
        g.append(f"G53 G0 X{A_CENTER_X:.3f} Y{ys:.3f}")
        # Explicitly re-index A at safe Z before every DOC pass so deeper passes
        # enter the exact same physical groove phase.
        g.append(f"G0 A{start_a:.3f}")
        first_z = self.radius_to_machine_z(z_points[0][1], center_z)
        if first_z < self.deepest_allowed_z(center_z) - 1e-9:
            raise ValueError("Helical path violates Z_MIN_DIAMETER.")
        g.append(f"G53 G0 Z{min(safe_z, first_z + Z_APPROACH_CLEARANCE):.3f}")
        g.append(f"G53 G1 Z{first_z:.3f} F{plunge:.1f}")

        prev_y = ys
        prev_z = first_z
        prev_a = start_a
        for local, radius in z_points[1:]:
            y = ys + STOCK_Y_DIRECTION * local
            self.validate_y(y, cutting=True)
            z = self.radius_to_machine_z(radius, center_z)
            if z < self.deepest_allowed_z(center_z) - 1e-9:
                raise ValueError("Helical path violates Z_MIN_DIAMETER.")
            frac = local / length if length > 0 else 0.0
            a = start_a + total_a * frac
            feed = self._segment_feed(y-prev_y, z-prev_z, a-prev_a, rpm, nominal_feed)
            feed = min(max(feed, 1.0), max(nominal_feed * 2.0, nominal_feed))
            g.append(f"G53 G1 Y{y:.3f} Z{z:.3f} A{a:.3f} F{feed:.1f}")
            prev_y, prev_z, prev_a = y, z, a
        g.append(f"G53 G0 Z{safe_z:.3f}")
        g.append("")
        return end_a

    # ------------------------------------------------------------------
    # ROUND G-code
    # ------------------------------------------------------------------
    def build_round_gcode(self):
        c = self.common_values(); errs = self.common_errors(c)
        if errs: raise ValueError(errs[0])
        if c["stock_shape"] != "SQUARE":
            raise ValueError("ROUND indexed roughing requires SQUARE raw stock. For ROUND stock already at blank diameter, proceed to WAVE.")
        center_z = self.probe_center_z("round_carriage_probe_z")
        probe_z = float(self.v["round_carriage_probe_z"].get().strip())
        if not self.round_square_diagonal.get():
            raise ValueError(
                "Pre-flight incomplete: confirm square stock is at A0 in the required "
                "DIAGONAL / CORNER-UP orientation."
            )
        if not self.round_rollover.get():
            raise ValueError("Confirm A-axis rollover is OFF.")

        tool = self._f("round_tool_dia"); doc = self._f("round_doc")
        feed = self._f("round_feed"); plunge = self._f("round_plunge")
        rpm = self._f("round_spiral_rpm"); pitch = self._f("round_spiral_pitch")
        self.validate_rpm(rpm, "ROUND")
        model = self.build_round_model(c, tool)
        ys, ye, cr, safe_z, _ = self.common_geometry(c, center_z)
        self.validate_y(ys, cutting=True); self.validate_y(ye, cutting=True)
        g = self._header("ROUND - square stock to cylinder", tool, safe_z, center_z, probe_z)
        g += [
            f"( Job: {c['job']} )",
            f"( Raw stock: {c['stock']:.3f} mm square x {c['stock_length']:.3f} mm physical length )",
            f"( Operator Cut Y Max = {c['cut_y_max']:.3f} machine mm; hard machine Y maximum = {Y_MACHINE_MAX:.3f} )",
            f"( Chuck face machine Y = {CHUCK_FACE_MACHINE_Y:.3f}; tail-safe cut limit = Y{CHUCK_FACE_MACHINE_Y + c['stock_length'] - WASTE_TAIL:.3f} )",
            f"( Cut range Y{ys:.3f} to Y{ye:.3f}; hard Y minimum {Y_MACHINE_MIN:.3f}; required physical tail waste {WASTE_TAIL:.3f} mm )",
            "( IMPORTANT: Cut Y Max is operator-selected and is not inferred from stock length or tail waste. )",
            f"( Target round D {c['round_dia']:.3f} mm; tool {tool:.3f} mm flat end mill )",
            "( PRE-FLIGHT CONFIRMED: square stock at A0 in DIAGONAL / CORNER-UP orientation )",
            "( NO-TRENCH RULE: each indexed stage stops where exposed facet width <= cutter diameter )",
            f"( Final spiral cleanup predicted max radial cut {model['cleanup']:.3f} mm )",
            "",
        ]

        current_a = 0.0
        for si, stage in enumerate(model["stages"], start=1):
            start_r = stage["start_support"]; target = stage["target_radius"]
            total = max(0.0, start_r-target)
            passes = max(1, math.ceil(total/doc)) if total > 1e-9 else 1
            levels = [max(target, start_r-i*doc) for i in range(1, passes+1)]
            if not levels or abs(levels[-1]-target) > 1e-9:
                levels.append(target)

            g.append(f"( ===== ROUND INDEX STAGE {si}: {stage['label']} ===== )")
            g.append(f"( Start support R {start_r:.3f}; target R {target:.3f}; final exposed facet width {stage['facet_width']:.3f} mm )")
            machine_orientations = [square_model_angle_to_machine_orientation(a) for a in stage["angles"]]
            g.append("( Model angles: " + ", ".join(fmt_angle(a) for a in stage["angles"]) + " )")
            g.append("( Machine A orientations from CORNER-UP A0: " +
                     ", ".join(fmt_angle(a) for a in machine_orientations) + " )")

            prev_level = start_r
            for li, radius in enumerate(levels, start=1):
                actual = max(0.0, prev_level-radius)
                z = self.radius_to_machine_z(radius, center_z)
                if z < self.deepest_allowed_z(center_z) - 1e-9:
                    raise ValueError("ROUND path violates Z_MIN_DIAMETER.")
                g.append(f"( Layer {li}: radius {radius:.3f}; machine Z {z:.3f}; radial removal <= {actual:.3f} mm )")
                for model_ang in stage["angles"]:
                    orientation = square_model_angle_to_machine_orientation(model_ang)
                    a_target = next_forward_a(current_a, orientation)
                    g.append(f"G53 G0 Z{safe_z:.3f}")
                    g.append(f"G53 G0 X{A_CENTER_X:.3f} Y{ys:.3f}")
                    g.append(f"G0 A{a_target:.3f}")
                    current_a = a_target
                    g.append(f"G53 G0 Z{min(safe_z,z+Z_APPROACH_CLEARANCE):.3f}")
                    g.append(f"G53 G1 Z{z:.3f} F{plunge:.1f}")
                    g.append(f"G53 G1 Y{ye:.3f} Z{z:.3f} F{feed:.1f}")
                    g.append(f"G53 G0 Z{safe_z:.3f}")
                prev_level = radius
                g.append("")

        target = c["round_dia"]/2.0
        target_z = self.radius_to_machine_z(target, center_z)
        revs = max(1, math.ceil(c["length"]/pitch))
        actual = c["length"]/revs
        yfeed = actual*rpm
        total_a = revs*360.0
        if current_a + total_a > A_SOFT_LIMIT_ABS + 1e-9:
            raise ValueError("ROUND accumulated A coordinate exceeds A_SOFT_LIMIT_ABS.")
        g.append("( ===== FINAL CONSTANT-RADIUS ROUND CLEANUP SPIRAL ===== )")
        g.append(f"( Max predicted residual high spot {model['cleanup']:.3f} mm radial )")
        g.append(f"( {revs} rev; actual pitch {actual:.4f} mm/rev; target A speed {rpm:.3f} RPM )")
        g.append(f"G53 G0 Z{safe_z:.3f}")
        g.append(f"G53 G0 X{A_CENTER_X:.3f} Y{ys:.3f}")
        g.append(f"G53 G0 Z{min(safe_z,target_z+Z_APPROACH_CLEARANCE):.3f}")
        g.append(f"G53 G1 Z{target_z:.3f} F{plunge:.1f}")
        g.append(f"G53 G1 Y{ye:.3f} A{current_a + total_a:.3f} F{yfeed:.3f}")
        g.append(f"G53 G0 Z{safe_z:.3f}")
        g.append("( ROUND stage complete. STOP for manual tool change before WAVE. )")
        g += self._footer(safe_z)
        return "\n".join(g)+"\n"

    # ------------------------------------------------------------------
    # WAVE G-code
    # ------------------------------------------------------------------
    def build_wave_gcode(self):
        c = self.common_values(); errs = self.common_errors(c)
        if errs: raise ValueError(errs[0])
        if not (self.wave_tool_changed.get() and self.wave_rollover.get()):
            raise ValueError("Complete WAVE tool-change and rollover confirmations.")

        center_z = self.probe_center_z("wave_carriage_probe_z")
        probe_z = float(self.v["wave_carriage_probe_z"].get().strip())

        dmin = self._f("wave_dmin"); waves = self._i("waves")
        tool = self._f("wave_tool_dia"); feed = self._f("wave_feed")
        plunge = self._f("wave_plunge"); doc = self._f("wave_doc")
        allow = self._f("wave_finish_allow"); rpm = self._f("wave_spiral_rpm")
        pitch = self._f("wave_spiral_pitch")
        self.validate_rpm(rpm, "WAVE")

        if waves not in (-5,-4,-3,-2,-1,0,1,2,3,4,5): raise ValueError("Waves must be -5 to +5.")
        if dmin < Z_MIN_DIAMETER - 1e-9 or dmin > c["round_dia"] + 1e-9:
            raise ValueError(f"Minimum wave diameter must be >= {Z_MIN_DIAMETER:.3f} and <= inherited round diameter.")
        if min(tool,feed,plunge,doc,pitch) <= 0 or self._i("wave_flutes") <= 0 or allow < 0:
            raise ValueError("Invalid WAVE cutting values.")

        # Hard cutter/profile feasibility check repeated here so G-code cannot
        # be generated by bypassing the GUI status logic.
        amp = (c["round_dia"] - dmin) / 4.0
        if waves != 0 and abs(amp) > 1e-12:
            wavelength = c["length"] / abs(waves)
            k = 2.0 * math.pi * abs(waves) / c["length"]
            min_curve_radius = 1.0 / (abs(amp) * k * k)
            if tool / 2.0 >= min_curve_radius - 1e-9:
                raise ValueError(
                    f"WAVE density is too high for selected Ø{tool:.3f} mm ball nose: "
                    f"{abs(waves)} waves over {c['length']:.3f} mm gives {wavelength:.3f} mm/wave; "
                    f"profile Rmin is {min_curve_radius:.3f} mm versus cutter R{tool/2.0:.3f} mm."
                )

        ys, ye, _, safe_z, _ = self.common_geometry(c, center_z)
        self.validate_y(ys, cutting=True); self.validate_y(ye, cutting=True)
        start_r = c["round_dia"]/2.0
        local_safe = min(safe_z, self.radius_to_machine_z(start_r + WAVE_LOCAL_CLEARANCE, center_z))

        final_surface = self.sample_wave(c, shell_floor=None, allowance=0.0, step=max(0.25,c["length"]/1000.0))
        final_comp = self.compensate_ball_nose_tip_path(final_surface, tool)
        _, slope_z_protection = self.compensation_stats(final_surface, final_comp)
        if allow + 1e-9 < slope_z_protection:
            raise ValueError(f"Finish allowance {allow:.3f} mm is smaller than ball-nose slope protection requirement {slope_z_protection:.3f} mm.")
        if min(r for _,r in final_comp) * 2.0 < Z_MIN_DIAMETER - 1e-9:
            raise ValueError("Compensated WAVE path violates Z_MIN_DIAMETER.")

        model = self.build_wave_station_model(c, tool, doc, allow)
        timing = self.wave_runtime_model(c, model, rpm, pitch, plunge)
        finish_revs = timing["finish_revs"]; finish_pitch = timing["actual_pitch"]
        total_true_a = model["rough_a_degrees"] + finish_revs*360.0

        # v2.14 WAVE policy: use one literal accumulated A coordinate from the
        # beginning of roughing through the end of finishing.  The machine's
        # verified UCCNC A soft limit is now +/-3,600,000 degrees, so internal
        # G92 rebasing is unnecessary and would only complicate preservation of
        # the original physical A0 reference.
        if total_true_a > A_SOFT_LIMIT_ABS + 1e-9:
            raise ValueError(
                f"WAVE accumulated A travel {total_true_a:.0f} deg exceeds A_SOFT_LIMIT_ABS {A_SOFT_LIMIT_ABS:.0f}."
            )
        phase_error = abs(total_true_a % 360.0)
        if min(phase_error, abs(360.0-phase_error)) > 1e-7:
            raise ValueError("WAVE must finish at the original physical A0 phase before downstream operations.")

        g = self._header("WAVE - indexed circumferential roughing then cosine spiral finish", tool, safe_z, center_z, probe_z)
        g += [
            f"( Job: {c['job']} )",
            f"( Inherited ROUND blank D {c['round_dia']:.3f} mm )",
            f"( Wave: {waves:+d}; Dmax {c['round_dia']:.3f}; Dmin {dmin:.3f} )",
            f"( Ball nose D {tool:.3f}, {self._i('wave_flutes')} flute; Y station step {model['step']:.3f} <= cutter radius {tool/2.0:.3f} )",
            f"( Roughing: {model['station_count']} stations; {model['sweep_count']} sweeps; {model['ring_count']} rings )",
            f"( A target speed {rpm:.3f} RPM = {rpm*360.0:.1f} deg/min; configured maximum {RPM_MAX:.3f} RPM )",
            f"( Accumulated A travel for this WAVE job = {total_true_a:.0f} deg; UCCNC soft limit = {A_SOFT_LIMIT_ABS:.0f} deg. )",
            "( No internal G92 A rebasing is used. Original physical A0 is preserved literally. )",
            "( A speed is commanded directly by G-code feed; no speed-control macros are required. )",
            "",
        ]
        current_a = 0.0
        a_deg_per_min = rpm*360.0
        for sweep_no, cuts in enumerate(model["sweeps"], start=1):
            direction = "Ymin -> Ymax" if sweep_no % 2 else "Ymax -> Ymin"
            g.append(f"( ===== WAVE ROUGH SWEEP {sweep_no}/{model['sweep_count']}: {direction} ===== )")
            for cut in cuts:
                y_machine = ys + STOCK_Y_DIRECTION * cut["x"]
                self.validate_y(y_machine, cutting=True)
                current_a = cut["a_target"]
                if current_a > A_SOFT_LIMIT_ABS + 1e-9:
                    raise ValueError("WAVE roughing exceeds the configured A soft limit.")
                z = self.radius_to_machine_z(cut["to_radius"], center_z)
                if z < self.deepest_allowed_z(center_z) - 1e-9:
                    raise ValueError("WAVE roughing violates Z_MIN_DIAMETER.")
                g.append(f"( Y local {cut['x']:.3f}: R {cut['from_radius']:.3f} -> {cut['to_radius']:.3f}; machine Z {z:.3f} )")
                g.append(f"G53 G0 Z{local_safe:.3f}")
                g.append(f"G53 G0 X{A_CENTER_X:.3f} Y{y_machine:.3f}")
                g.append(f"G53 G1 Z{z:.3f} F{plunge:.1f}")
                g.append(f"G1 A{current_a:.3f} F{a_deg_per_min:.1f}")
                g.append(f"G53 G0 Z{local_safe:.3f}")
            g.append("")

        finish_surface = self.sample_wave(c, shell_floor=None, allowance=0.0, step=PROFILE_SAMPLE_MM)
        finish_tip = self.compensate_ball_nose_tip_path(finish_surface, tool)

        # Roughing ends after complete 360-degree rings, so current_a is already
        # at the original physical A0 phase.  Continue accumulating A directly.
        g.append("( ===== FINAL COMPENSATED COSINE SPIRAL FINISH ===== )")
        g.append(f"( {finish_revs} rev; actual pitch {finish_pitch:.4f} mm/rev; target A speed {rpm:.3f} RPM )")
        g.append(f"G53 G0 Z{safe_z:.3f}")
        g.append(f"G53 G0 X{A_CENTER_X:.3f} Y{ys:.3f}")
        first_z = self.radius_to_machine_z(finish_tip[0][1], center_z)
        g.append(f"G53 G0 Z{min(safe_z,first_z+Z_APPROACH_CLEARANCE):.3f}")
        g.append(f"G53 G1 Z{first_z:.3f} F{plunge:.1f}")
        finish_total_a = finish_revs*360.0
        finish_start_a = current_a

        prev_y = ys
        prev_z = first_z
        prev_a = finish_start_a
        for local, radius in finish_tip[1:]:
            y_machine = ys + STOCK_Y_DIRECTION * local
            self.validate_y(y_machine, cutting=True)
            z = self.radius_to_machine_z(radius, center_z)
            if z < self.deepest_allowed_z(center_z) - 1e-9:
                raise ValueError("WAVE finish violates Z_MIN_DIAMETER.")
            frac = local/c["length"] if c["length"] > 0 else 0.0
            a = finish_start_a + finish_total_a*frac
            if a > A_SOFT_LIMIT_ABS + 1e-9:
                raise ValueError("WAVE finish exceeds the configured A soft limit.")
            seg_feed = self._segment_feed(y_machine-prev_y, z-prev_z, a-prev_a, rpm, feed)
            seg_feed = min(max(seg_feed,1.0), max(feed*2.0,feed))
            g.append(f"G53 G1 Y{y_machine:.3f} Z{z:.3f} A{a:.3f} F{seg_feed:.1f}")
            prev_y, prev_z, prev_a = y_machine, z, a

        current_a = total_true_a
        g.append(f"G53 G0 Z{safe_z:.3f}")
        g.append(f"( WAVE accumulated A travel {current_a:.0f} deg. Final physical phase = ORIGINAL A0. )")
        g.append("( WAVE stage complete at original A0 phase. STOP for manual tool change before next stage. )")
        g += self._footer(safe_z)
        return "\n".join(g)+"\n"

    # ------------------------------------------------------------------
    # TAPER geometry / G-code
    # ------------------------------------------------------------------
    def taper_is_active(self):
        try:
            return (self._i("waves") == 0 and
                    bool(self.v["taper_y_end"].get().strip()) and
                    abs(self._f("taper_start_dia") - self._f("taper_end_dia")) > 1e-9)
        except Exception:
            return False

    def taper_radius_at_machine_y(self, y):
        y0 = self._f("taper_y_start")
        y1 = self._f("taper_y_end")
        if y1 <= y0:
            raise ValueError("Invalid TAPER Y range.")
        frac = min(1.0, max(0.0, (y-y0)/(y1-y0)))
        r0 = self._f("taper_start_dia")/2.0
        r1 = self._f("taper_end_dia")/2.0
        return r0 + (r1-r0)*frac

    def groove_source(self):
        return self.v["groove_source"].get().strip().upper()

    def groove_source_errors(self, common):
        """Validate the operator-selected physical base profile for GROOVE.

        This is intentionally explicit.  GROOVE never guesses which upstream
        operation actually produced the stick currently mounted in the machine.
        """
        source = self.groove_source()
        errs = []
        if source not in ("ROUND", "WAVE", "TAPER"):
            return ["Select GROOVE Base Profile: ROUND, WAVE or TAPER."]

        if source == "WAVE":
            waves = self._i("waves")
            dmin = self._f("wave_dmin")
            if waves == 0:
                errs.append("GROOVE source WAVE selected, but WAVE count is zero.")
            if waves not in (-5, -4, -3, -2, -1, 1, 2, 3, 4, 5):
                errs.append("Selected WAVE source must have 1 to 5 waves in either direction.")
            if dmin < Z_MIN_DIAMETER - 1e-9 or dmin > common["round_dia"] + 1e-9:
                errs.append("Selected WAVE source has an invalid minimum diameter.")

        elif source == "TAPER":
            if self._i("waves") != 0:
                errs.append("GROOVE source TAPER requires WAVE = 0.")
            if not self.taper_is_active():
                errs.append("GROOVE source TAPER selected, but no valid active taper is defined.")
            else:
                y0 = self._f("taper_y_start")
                y1 = self._f("taper_y_end")
                d0 = self._f("taper_start_dia")
                d1 = self._f("taper_end_dia")
                if y1 <= y0:
                    errs.append("Selected TAPER source has an invalid Y range.")
                if y0 < Y_MACHINE_MIN - 1e-9 or y1 > common["cut_y_max"] + 1e-9:
                    errs.append("Selected TAPER source lies outside the current rotary cut Y range.")
                if min(d0, d1) < MIN_TAPER_DIAMETER - 1e-9:
                    errs.append(f"Selected TAPER source goes below {MIN_TAPER_DIAMETER:.3f} mm diameter.")
                if max(d0, d1) > common["round_dia"] + 1e-9:
                    errs.append("Selected TAPER source exceeds the inherited ROUND diameter.")

        # ROUND is deliberately allowed even if stale WAVE/TAPER values remain
        # in their tabs. The operator has explicitly declared that the physical
        # part in the chuck is the ROUND result.
        return errs

    def groove_parent_y_bounds(self, common, source=None):
        """Return machine-Y bounds of the explicitly selected parent profile."""
        source = (source or self.groove_source()).upper()
        if source in ("ROUND", "WAVE"):
            return Y_MACHINE_MIN, common["cut_y_max"]
        if source == "TAPER":
            return self._f("taper_y_start"), self._f("taper_y_end")
        raise ValueError("GROOVE base profile has not been selected.")

    def groove_cut_y_bounds(self, common, tool_dia, source=None):
        """Cutter-centre Y envelope kept safely inside the machined parent profile."""
        parent_start, parent_end = self.groove_parent_y_bounds(common, source)
        margin = GROOVE_EDGE_MARGIN_FACTOR * tool_dia
        cut_start = parent_start + margin
        cut_end = parent_end - margin
        if cut_end <= cut_start + 1e-9:
            raise ValueError(
                f"GROOVE cutter D{tool_dia:.3f} with {GROOVE_EDGE_MARGIN_FACTOR*100:.0f}% edge setback "
                f"does not fit inside parent Y range {parent_start:.3f}..{parent_end:.3f}."
            )
        return parent_start, parent_end, cut_start, cut_end, margin

    def base_profile_radius_machine_y(self, y, common, source=None):
        """Return selected parent-profile radius at an absolute machine-Y coordinate."""
        source = (source or self.groove_source()).upper()
        if source == "WAVE":
            local = y - Y_MACHINE_MIN
            if local < -1e-9 or local > common["length"] + 1e-9:
                raise ValueError("Requested GROOVE Y lies outside the WAVE parent profile.")
            return self.wave_radius(min(max(local, 0.0), common["length"]), common)
        if source == "TAPER":
            y0 = self._f("taper_y_start")
            y1 = self._f("taper_y_end")
            if y < y0 - 1e-9 or y > y1 + 1e-9:
                raise ValueError("Requested GROOVE Y lies outside the TAPER parent profile.")
            return self.taper_radius_at_machine_y(y)
        if source == "ROUND":
            return common["round_dia"] / 2.0
        raise ValueError("GROOVE base profile has not been selected.")

    def base_profile_radius(self, local, common, source=None):
        """Compatibility helper: local distance measured from the selected parent Y start."""
        parent_start, _ = self.groove_parent_y_bounds(common, source)
        return self.base_profile_radius_machine_y(parent_start + local, common, source)

    def groove_base_min_diameter(self, common, source=None, tool_dia=None):
        """Minimum diameter along the actual cutter-centre GROOVE envelope."""
        source = (source or self.groove_source()).upper()
        if tool_dia is None:
            tool_dia = self._f("groove_tool_dia")
        _, _, y0, y1, _ = self.groove_cut_y_bounds(common, tool_dia, source)
        length = y1 - y0
        count = max(2, math.ceil(length / PROFILE_SAMPLE_MM))
        min_r = min(
            self.base_profile_radius_machine_y(y0 + length * i / count, common, source)
            for i in range(count + 1)
        )
        return 2.0 * min_r

    def _taper_rough_model(self, d0, d1, y0, y1, tool, doc, round_dia):
        """Build thin-end-first, no-trench indexed ring roughing model.

        Each station is spaced at 80% of the flat cutter diameter.  The safe
        rough radius is the LARGEST required final radius under the full cutter
        footprint, so the flat cutter cannot undercut/trench the finished taper.
        Stations progress from the minimum-diameter end toward the thick end so
        the work remains supported by progressively thicker stock.
        """
        if y1 <= y0:
            raise ValueError("Invalid TAPER Y range.")
        if tool <= 0 or doc <= 0:
            raise ValueError("TAPER cutter and DOC must be positive.")
        r0, r1 = d0 / 2.0, d1 / 2.0
        round_r = round_dia / 2.0
        step = tool * TAPER_ROUGH_STEP_FACTOR
        half_tool = tool / 2.0

        def radius_at(y):
            frac = min(1.0, max(0.0, (y - y0) / (y1 - y0)))
            return r0 + (r1 - r0) * frac

        # Thin end first, regardless of taper direction.
        if r1 < r0:
            direction = -1.0
            y = y1
            stop = y0
        else:
            direction = 1.0
            y = y0
            stop = y1

        ys = []
        while True:
            ys.append(y)
            next_y = y + direction * step
            if (direction < 0 and next_y <= stop + 1e-9) or (direction > 0 and next_y >= stop - 1e-9):
                if abs(ys[-1] - stop) > 1e-9:
                    ys.append(stop)
                break
            y = next_y

        stations = []
        total_rings = 0
        for y in ys:
            left = max(y0, y - half_tool)
            right = min(y1, y + half_tool)
            # Linear taper is monotonic; max of the two footprint ends is the
            # conservative no-trench radius for the whole flat-cutter footprint.
            safe_r = max(radius_at(left), radius_at(right))
            removal = max(0.0, round_r - safe_r)
            rings = int(math.ceil(removal / doc - 1e-12)) if removal > 1e-12 else 0
            levels = []
            for i in range(1, rings + 1):
                levels.append(max(safe_r, round_r - i * doc))
            if levels:
                levels[-1] = safe_r
            total_rings += len(levels)
            stations.append({
                "y": y, "safe_radius": safe_r, "removal": removal,
                "levels": levels, "footprint_left": left, "footprint_right": right,
            })

        return {
            "step": step,
            "stations": stations,
            "station_count": len(stations),
            "ring_count": total_rings,
            "thin_end_y": y1 if r1 < r0 else y0,
            "direction": direction,
        }

    def _taper_helical_pass(self, g, y0, y1, radii, safe_z, center_z, rpm, pitch, plunge, nominal_feed, start_a, comment=""):
        length = y1-y0
        revs = max(1, math.ceil(length/pitch))
        actual_pitch = length/revs
        total_a = revs*360.0
        end_a = start_a + total_a
        if end_a > A_SOFT_LIMIT_ABS + 1e-9:
            raise ValueError(f"TAPER A coordinate exceeds A_SOFT_LIMIT_ABS {A_SOFT_LIMIT_ABS:.0f}.")
        if comment:
            g.append(f"( {comment} )")
        g.append(f"( {revs} rev; actual pitch {actual_pitch:.4f} mm/rev )")
        g.append(f"G53 G0 Z{safe_z:.3f}")
        g.append(f"G53 G0 X{A_CENTER_X:.3f} Y{y0:.3f}")
        g.append(f"G0 A{start_a:.3f}")
        first_z=self.radius_to_machine_z(radii[0][1], center_z)
        g.append(f"G53 G0 Z{min(safe_z, first_z+Z_APPROACH_CLEARANCE):.3f}")
        g.append(f"G53 G1 Z{first_z:.3f} F{plunge:.1f}")
        py,pz,pa=y0,first_z,start_a
        for frac,radius in radii[1:]:
            y=y0+length*frac
            self.validate_y(y, cutting=True)
            z=self.radius_to_machine_z(radius, center_z)
            if z < self.deepest_allowed_z(center_z)-1e-9:
                raise ValueError("TAPER path violates minimum finished diameter.")
            a=start_a+total_a*frac
            f=self._segment_feed(y-py,z-pz,a-pa,rpm,nominal_feed)
            f=min(max(f,1.0),max(nominal_feed*2.0,nominal_feed))
            g.append(f"G53 G1 Y{y:.3f} Z{z:.3f} A{a:.3f} F{f:.1f}")
            py,pz,pa=y,z,a
        g.append(f"G53 G0 Z{safe_z:.3f}")
        g.append("")
        return end_a

    def build_taper_gcode(self):
        c=self.common_values(); errs=self.common_errors(c)
        if errs:
            raise ValueError(errs[0])
        if self._i("waves") != 0:
            raise ValueError("TAPER cannot run while WAVE count is non-zero. Set WAVE to 0 first.")
        if not (self.taper_tool_changed.get() and self.taper_rollover.get()):
            raise ValueError("Complete TAPER tool-change and rollover confirmations.")
        center_z=self.probe_center_z("taper_carriage_probe_z")
        probe_z=float(self.v["taper_carriage_probe_z"].get().strip())
        d0=self._f("taper_start_dia"); d1=self._f("taper_end_dia")
        y0=self._f("taper_y_start"); y1=self._f("taper_y_end")
        tool=self._f("taper_tool_dia"); doc=self._f("taper_doc")
        feed=self._f("taper_feed"); plunge=self._f("taper_plunge")
        rpm=self._f("taper_rpm"); pitch=self._f("taper_pitch")
        if min(d0,d1) < MIN_TAPER_DIAMETER-1e-9:
            raise ValueError(f"TAPER diameter cannot be below {MIN_TAPER_DIAMETER:.3f} mm.")
        if max(d0,d1) > c["round_dia"]+1e-9:
            raise ValueError("TAPER diameter cannot exceed inherited ROUND diameter.")
        if y1 <= y0:
            raise ValueError("TAPER Y End must be greater than Y Start.")
        tail_safe=CHUCK_FACE_MACHINE_Y+c["stock_length"]-WASTE_TAIL
        if y0 < Y_MACHINE_MIN-1e-9 or y1 > Y_MACHINE_MAX+1e-9 or y1 >= tail_safe-1e-9 or y1 > c["cut_y_max"]+1e-9:
            raise ValueError("TAPER Y range violates the configured machine/stock/tail safety envelope.")
        if min(tool,doc,feed,plunge,rpm,pitch) <= 0:
            raise ValueError("TAPER cutter, DOC, feeds, RPM and pitch must be positive.")
        self.validate_rpm(rpm,"TAPER")
        _,_,_,safe_z,_=self.common_geometry(c,center_z)

        rough=self._taper_rough_model(d0,d1,y0,y1,tool,doc,c["round_dia"])
        finish_n=max(2,math.ceil((y1-y0)/PROFILE_SAMPLE_MM))
        final=[]
        for i in range(finish_n+1):
            frac=i/finish_n
            r=(d0/2.0)+(d1-d0)/2.0*frac
            final.append((frac,r))

        finish_revs=max(1,math.ceil((y1-y0)/pitch))
        finish_a=finish_revs*360.0
        total_a=(rough["ring_count"]*360.0)+finish_a
        if total_a > A_SOFT_LIMIT_ABS + 1e-9:
            raise ValueError(f"TAPER accumulated A travel {total_a:.0f} deg exceeds A_SOFT_LIMIT_ABS {A_SOFT_LIMIT_ABS:.0f}.")

        angle=math.degrees(math.atan2(abs((d1-d0)/2.0),y1-y0))
        g=self._header("TAPER - indexed no-trench ring roughing then spiral finish",tool,safe_z,center_z,probe_z)
        g += [
            f"( Job: {c['job']} )",
            f"( TAPER Y{y0:.3f} to Y{y1:.3f}; D{d0:.3f} to D{d1:.3f}; angle {angle:.3f} deg )",
            f"( Roughing: thin-end first; station step {rough['step']:.3f} mm = {TAPER_ROUGH_STEP_FACTOR*100:.0f}% of cutter D{tool:.3f} )",
            f"( No-trench rule: each ring radius is the largest required taper radius under the full cutter footprint. )",
            f"( Rough stations {rough['station_count']}; rough rings {rough['ring_count']}; max radial DOC {doc:.3f} mm )",
            f"( Final spiral: {finish_revs} rev; requested pitch {pitch:.3f} mm/rev )",
            "( WAVE lock confirmed: Waves = 0 )",
            "",
        ]

        current_a=0.0
        a_feed=rpm*360.0
        for si,station in enumerate(rough["stations"],start=1):
            levels=station["levels"]
            if not levels:
                continue
            y=station["y"]
            self.validate_y(y,cutting=True)
            g.append(
                f"( ===== TAPER ROUGH STATION {si}/{rough['station_count']} Y{y:.3f}: "
                f"safe R{station['safe_radius']:.3f}; footprint Y{station['footprint_left']:.3f}..Y{station['footprint_right']:.3f}; "
                f"{len(levels)} ring(s) ===== )"
            )
            g.append(f"G53 G0 Z{safe_z:.3f}")
            g.append(f"G53 G0 X{A_CENTER_X:.3f} Y{y:.3f}")
            first_z=self.radius_to_machine_z(levels[0],center_z)
            g.append(f"G53 G0 Z{min(safe_z,first_z+Z_APPROACH_CLEARANCE):.3f}")
            for li,radius in enumerate(levels,start=1):
                z=self.radius_to_machine_z(radius,center_z)
                if z < self.deepest_allowed_z(center_z)-1e-9:
                    raise ValueError("TAPER roughing violates minimum finished diameter.")
                g.append(f"G53 G1 Z{z:.3f} F{plunge:.1f}")
                current_a += 360.0
                if current_a > A_SOFT_LIMIT_ABS + 1e-9:
                    raise ValueError("TAPER roughing exceeds the configured A soft limit.")
                g.append(f"G1 A{current_a:.3f} F{a_feed:.1f} ( ring {li}/{len(levels)} )")
            g.append(f"G53 G0 Z{safe_z:.3f}")
            g.append("")

        # All rough cuts are complete 360-degree rings, so current_a is still
        # exactly at the original physical A0 phase.  Finish with one full
        # Y/Z/A spiral across the requested taper.
        current_a=self._taper_helical_pass(
            g,y0,y1,final,safe_z,center_z,rpm,pitch,plunge,feed,current_a,
            comment="FINAL TAPER SPIRAL FINISH"
        )
        phase_error=abs(current_a % 360.0)
        if min(phase_error,abs(360.0-phase_error)) > 1e-7:
            raise ValueError("TAPER did not finish at the original physical A0 phase.")
        g.append(f"( TAPER accumulated A travel {current_a:.0f} deg. Final physical phase = ORIGINAL A0. )")
        g += self._footer(safe_z)
        return "\n".join(g)+"\n"

    # ------------------------------------------------------------------
    # GROOVE G-code
    # ------------------------------------------------------------------
    def build_groove_gcode(self):
        if not self.v["groove_enable"].get():
            raise ValueError("Grooves are disabled.")
        c = self.common_values(); errs = self.common_errors(c)
        if errs:
            raise ValueError(errs[0])
        if not (self.groove_tool_changed.get() and self.groove_rollover.get()):
            raise ValueError("Complete GROOVE tool-change and rollover confirmations.")

        center_z = self.probe_center_z("groove_carriage_probe_z")
        probe_z = float(self.v["groove_carriage_probe_z"].get().strip())

        tool = self._f("groove_tool_dia")
        depth = self._f("groove_depth")
        max_doc = self._f("groove_doc")
        pitch = self._f("groove_pitch")
        starts = self._i("groove_starts")
        start_angle = self._f("groove_start_angle")
        feed = self._f("groove_feed")
        plunge = self._f("groove_plunge")
        rpm = self._f("groove_rpm")
        if min(tool, depth, max_doc, pitch, feed, plunge, rpm) <= 0 or starts < 1:
            raise ValueError("Invalid GROOVE values; cutter, depth, DOC, pitch, feeds and RPM must be positive and starts >= 1.")
        self.validate_rpm(rpm, "GROOVE")

        source = self.groove_source()
        source_errors = self.groove_source_errors(c)
        if source_errors:
            raise ValueError(source_errors[0])
        parent_y_start, parent_y_end, groove_y_start, groove_y_end, edge_margin = self.groove_cut_y_bounds(c, tool, source)
        base_min_dia = self.groove_base_min_diameter(c, source, tool)
        root = base_min_dia - 2.0 * depth
        if root < Z_MIN_DIAMETER - 1e-9:
            raise ValueError(f"Groove root diameter {root:.3f} is below Z_MIN_DIAMETER {Z_MIN_DIAMETER:.3f} mm.")

        _, _, _, safe_z, _ = self.common_geometry(c, center_z)
        hand_sign = 1.0 if self.v["groove_hand"].get() == "Right-hand" else -1.0

        pass_count = max(1, math.ceil(depth / max_doc - 1e-12))
        depth_levels = [min(depth, (i + 1) * max_doc) for i in range(pass_count)]
        depth_levels[-1] = depth
        separation = 360.0 / starts
        base_phases = [(start_angle + s * separation) % 360.0 for s in range(starts)]

        g = self._header(f"GROOVE - explicit {source} base profile to multi-pass helical grooves", tool, safe_z, center_z, probe_z)
        g += [
            f"( Job: {c['job']} )",
            f"( OPERATOR-SELECTED base profile: {source} )",
            f"( Parent profile Y range: {parent_y_start:.3f} to {parent_y_end:.3f} machine mm )",
            f"( Groove cutter-centre Y range: {groove_y_start:.3f} to {groove_y_end:.3f} machine mm )",
            f"( Edge protection: centreline setback {edge_margin:.3f} mm = {GROOVE_EDGE_MARGIN_FACTOR*100:.0f}% of cutter D{tool:.3f} at BOTH ends )",
            f"( Groove: final depth {depth:.3f} radial; max DOC/pass {max_doc:.3f}; {pass_count} depth pass(es) )",
            f"( Pitch {pitch:.3f}; starts {starts}; separation {separation:.3f} deg; {self.v['groove_hand'].get()} )",
            f"( Root diameter at Dmin = {root:.3f} mm )",
            f"( Groove cutter D {tool:.3f}; exact groove cutter contact compensation NOT applied in v2.01 )",
            "( DEPTH-FIRST ORDER: all starts are cut at each DOC level before the next deeper level. )",
            "( PHASE LOCK: every deeper pass is explicitly re-indexed to the same physical A start phase. )",
            "",
        ]

        # Keep A monotonic in helix direction with rollover OFF. The stored base
        # phase for each start never changes; only the equivalent accumulated A
        # coordinate changes to avoid physical multi-revolution unwinds.
        current_a = 0.0
        groove_length = groove_y_end - groove_y_start
        count = max(2, math.ceil(groove_length / PROFILE_SAMPLE_MM))

        for pass_index, pass_depth in enumerate(depth_levels, start=1):
            g.append(f"( ===== GROOVE DEPTH PASS {pass_index}/{pass_count}: cumulative radial depth {pass_depth:.3f} mm ===== )")

            pts = []
            for i in range(count + 1):
                local = groove_length * i / count
                y = groove_y_start + local
                pts.append((local, self.base_profile_radius_machine_y(y, c, source) - pass_depth))

            for s, base_phase in enumerate(base_phases, start=1):
                start_a = next_phase_aligned_a(current_a, base_phase, hand_sign)
                if abs(start_a) > A_SOFT_LIMIT_ABS + 1e-9:
                    raise ValueError(f"GROOVE start A{start_a:.0f} exceeds A_SOFT_LIMIT_ABS {A_SOFT_LIMIT_ABS:.0f}.")
                comment = (
                    f"GROOVE DEPTH {pass_index}/{pass_count}, START {s}/{starts}: "
                    f"depth {pass_depth:.3f} mm; stored phase {base_phase:.3f} deg; "
                    f"absolute start A {start_a:.3f}"
                )
                current_a = self._helical_profile_pass(
                    g, c, pts, safe_z, center_z, rpm, pitch, plunge, feed,
                    start_a=start_a, hand_sign=hand_sign, comment=comment,
                    y_start=groove_y_start, y_end=groove_y_end
                )

        g += self._footer(safe_z)
        return "\n".join(g) + "\n"

    # ------------------------------------------------------------------
    # File output
    # ------------------------------------------------------------------
    def _write(self, filename, content):
        outdir=Path(DEFAULT_OUTPUT_DIR)
        try:
            outdir.mkdir(parents=True,exist_ok=True)
            path=outdir/filename
            path.write_text(content,encoding="utf-8")
        except Exception:
            # Development / non-Windows fallback.
            outdir=Path.cwd()
            path=outdir/filename
            path.write_text(content,encoding="utf-8")
        return path

    def generate_round(self):
        try:
            code=self.build_round_gcode(); path=self._write(self.calc["round_file"].get(),code)
            messagebox.showinfo("ROUND G Code", f"Created:\n{path}")
        except Exception as exc:
            messagebox.showerror("ROUND not generated", str(exc))

    def generate_wave(self):
        try:
            code=self.build_wave_gcode(); path=self._write(self.calc["wave_file"].get(),code)
            messagebox.showinfo("WAVE G Code", f"Created:\n{path}")
        except Exception as exc:
            messagebox.showerror("WAVE not generated", str(exc))

    def generate_taper(self):
        try:
            code=self.build_taper_gcode(); path=self._write(self.calc["taper_file"].get(),code)
            messagebox.showinfo("TAPER G Code", f"Created:\n{path}")
        except Exception as exc:
            messagebox.showerror("TAPER not generated", str(exc))

    def generate_groove(self):
        try:
            code=self.build_groove_gcode(); path=self._write(self.calc["groove_file"].get(),code)
            messagebox.showinfo("GROOVE G Code", f"Created:\n{path}")
        except Exception as exc:
            messagebox.showerror("GROOVE not generated", str(exc))


if __name__ == "__main__":
    app=CNCApp()
    app.mainloop()
