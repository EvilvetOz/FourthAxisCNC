# FourthAxisCNC Design Notes

**Design rationale for my DIY CC Router**

Current documented build: **v2.23**

# 1. Design intent

This document records the reasoning behind the current machining
strategies. It is intended to explain why the generator behaves as it
does, particularly where the code contains conservative geometry or
seemingly unusual sequencing.

The generator is not intended to reproduce a commercial multi-axis CAM
system. It converts a small set of useful rotary woodworking operations
into machine-specific, inspectable G-code.

# 2. High-level architecture

| **Block**         | **Responsibility**                                              | **Reason for separation**                                                 |
|-------------------|-----------------------------------------------------------------|---------------------------------------------------------------------------|
| Machine constants | Physical machine limits, datum offsets and hard safety values   | Keeps fixture geometry distinct from job geometry.                        |
| Common geometry   | Probe conversion, Y validation, safe Z and shared constraints   | Ensures all operations use the same physical machine model.               |
| ROUND model       | Square/round stock to finished cylinder                         | Provides the base stock geometry for all later operations.                |
| WAVE model        | Cosine radial surface and ball-nose compensation                | Encapsulates profile sampling and curvature limits.                       |
| TAPER model       | Linear radius profile plus indexed-ring roughing                | Provides a simple variable-radius parent surface.                         |
| GROOVE model      | Helical depth passes on an explicit parent profile              | Lets grooves follow ROUND, WAVE or TAPER without silent source inference. |
| GUI / validation  | Collects inputs and exposes calculated values before generation | Makes operator intent and invalid states visible before G-code exists.    |
| G-code builders   | Translate validated models to standalone UCCNC programs         | Keeps UI state separate from emitted motion commands.                     |

# 3. Machine-coordinate model

XYZ positions are emitted in machine coordinates using G53. The central
machine model currently uses:

A_CENTER_X = 778.70  
CHUCK_FACE_MACHINE_Y = -70.0  
Y_MACHINE_MIN = 1.0  
Y_MACHINE_MAX = 600.0  
STOCK_LENGTH_MAX = 700.0  
WASTE_CHUCK = 20.0  
WASTE_TAIL = 20.0  
PROBE1_BLOCK_HEIGHT = 40.000  
SPOILBOARD_TO_40120_TOP = 4.862  
LATHE_CARRIAGE_TOP_TO_A_CENTER = 65.000

The physical stock tail coordinate is derived from the chuck-face
machine coordinate and the entered physical stock length:

stock_tail_machine_y = CHUCK_FACE_MACHINE_Y + stock_length

The tail-side machining limit must remain inside that physical tail
position by the configured tail waste, and it must also remain inside
the hard machine-Y range Y1.0 to Y600.0. Y0 is intentionally not
emitted: although it is the nominal end of travel, UCCNC rejects a
commanded move exactly on the configured zero soft-limit boundary.

# 4. Cutter-dependent A-centre Z and Probe 1 conversion

A fixed machine Z for the rotary centreline is not stored because the
machine Z coordinate of a cutter tip changes with cutter length and
stick-out. Directly probing the top of the 40120 rotary bed became
awkward once mounted stock obscured the surface, so v2.23 uses the 40 mm
single-wire Probe 1 block on the spoilboard as the operational
reference.

spoilboard_z = raw_probe_z - 40.000  
carriage_top_z = spoilboard_z + 4.862  
A_CENTER_Z = carriage_top_z + 65.000  
  
Therefore:  
A_CENTER_Z = raw_probe_z + 29.862

The operator enters the raw negative G53 Z value shown after probing and
must not manually subtract the 40 mm block thickness. The conversion
explicitly removes the block height, adds the measured
spoilboard-to-40120 step, then adds the known 65 mm rotary centre
height. Because the actual cutter performs the touch, cutter stick-out
is automatically included. All later Safe Z, ROUND, WAVE, TAPER and
GROOVE calculations continue to use the resulting A_CENTER_Z exactly as
before.

The current A-centre X value of 778.70 mm was established by probing
opposite sides of a 12 mm steel test rod at A0 and A180 and averaging
the inferred centres. The measured chuck/rod eccentricity was consistent
with the chuck manufacturer's stated centering precision, so 778.70 mm
is treated as the practical calibrated centreline rather than a nominal
mounting dimension.

# 5. A-axis phase convention

The original physical stock orientation at A0 is treated as a persistent
phase reference. For square stock, A0 is the original required
diagonal/corner-up orientation. Equivalent-looking orientations at 90,
180 or 270 degrees are not treated as the original phase when later
operations may depend on start angle.

The generator uses literal accumulated A motion. The UCCNC A soft limit
was enlarged to +/-3,600,000 degrees, removing the need for periodic
G92-based rebasing during normal jobs. WAVE is designed to finish at an
exact whole-revolution phase so the original A0 orientation remains
meaningful for later operations.

# 6. ROUND strategy

## 6.1 Problem being solved

A square blank cannot safely be treated as a cylinder at its corner
radius. A wide cutter can create deep trenches if indexed cuts are
spaced too far apart. ROUND therefore models the square cross-section
and accepts an indexed stage only when the exposed facet width can be
covered by the cutter footprint.

## 6.2 Phase mapping

The polygon mathematics naturally places corners at model angles 45,
135, 225 and 315 degrees. Physical A0, however, is defined as corner-up.
The code therefore maps model angle to machine orientation with a
45-degree phase correction.

## 6.3 No-trench indexed stages

The current sequence progressively increases the number of indexed
facets. The final constant-radius spiral is allowed only when the
residual radial cleanup is small enough to avoid turning the finish pass
into an interrupted heavy cut.

ROUND_SPIRAL_MAX_CLEANUP = 0.50 \# radial mm

This is a safety/finish constraint, not a machine envelope limit.

# 7. WAVE strategy

## 7.1 Surface model

WAVE uses a cosine-style radius profile along machine Y. The operator
supplies the finished maximum diameter, minimum diameter and signed wave
count. The current range is -5 to +5. A zero wave count represents no
WAVE operation and is also used as a lock condition for TAPER.

## 7.2 Roughing

Roughing is performed as complete 360-degree rings at discrete Y
stations. Deeper finished stations drop out of later sweeps as their
target radius is reached. This keeps each roughing move easy to reason
about and returns to the same physical A phase after every ring.

## 7.3 Ball-nose finishing

The finishing pass is a continuous helical A+Y+Z motion. Because a ball
nose contacts the profile away from its centre on sloped surfaces, the
code samples the requested profile and compensates the tool-tip path. A
minimum local curve radius is calculated and compared with ball radius;
geometry that is too tight for the selected cutter is rejected.

## 7.4 Cutter inheritance

WAVE cutter diameter and flute count initially follow ROUND values for
convenience, but become independent once edited. This supports a common
workflow where ROUND uses a flat cutter and WAVE switches to a smaller
ball nose.

# 8. TAPER strategy

## 8.1 Linear profile model

A taper is defined by start diameter, end diameter, taper Y start and
taper Y end. Direction is implicit in the entered diameters; there is
intentionally no reverse-taper switch. Both finished diameters must
remain at or above MIN_TAPER_DIAMETER = 20 mm.

## 8.2 Why the first roughing strategy was replaced

The initial implementation used repeated full-length helical passes at
progressively deeper radial shells. It was safe but created excessive
full-length Y travel and poor cycle time.

## 8.3 Current thin-end-first indexed-ring roughing

The current strategy starts at the minimum-diameter end while the work
is still supported by thicker uncut material. The cutter then moves
toward the thicker end in stations separated by 80 percent of cutter
diameter. Each station may use several complete rings if the required
radial removal exceeds Max Radial DOC.

rough_y_step = cutter_diameter \* 0.80

A station is not cut blindly to the nominal taper diameter at its
centre. The full flat-cutter footprint is considered and the
conservative radius across that footprint is used. This leaves a safe
staircase outside the final taper rather than gouging a trench below it.
One continuous helical finishing pass then removes the staircase.

This method substantially reduces cycle time compared with repeated
full-length roughing helices while retaining a simple no-trench rule.

# 9. WAVE / TAPER exclusivity

TAPER is refused whenever WAVE count is non-zero. This prevents two
independent variable-radius parent models from being silently stacked. A
future general profile-composition system could relax this, but the
current code deliberately keeps the workflow explicit.

# 10. GROOVE strategy

## 10.1 Explicit parent selection

GROOVE presents a mandatory operator choice of ROUND, WAVE or TAPER. The
software does not auto-detect which surface exists on the workpiece. The
selected source is validated before generation.

## 10.2 Parent surface abstraction

The groove path is calculated against the selected local surface radius
rather than assuming a single cylinder radius. Conceptually the
downstream calculation is:

groove_tip_radius(y) = parent_surface_radius(y) - groove_depth

For multi-pass grooves, the same helix phase is reused at each deeper
DOC pass so a later pass deepens the existing groove rather than cutting
a new one.

## 10.3 Multi-start phase handling

Each groove start has a fixed angular phase derived from start angle and
start separation. Depth passes are ordered depth-first: all starts at
pass 1, then all starts at pass 2, and so on. A may continue
accumulating, but each start is aligned to its required phase relative
to the original A0 reference.

## 10.4 Cutter-edge protection

GROOVE does not send the cutter centre all the way to the parent profile
boundary. To prevent the outer portion of a cutter from striking
untouched stock, the centreline is set back from both inherited Y
boundaries by 60 percent of cutter diameter.

GROOVE_EDGE_MARGIN_FACTOR = 0.60  
  
groove_y_start = parent_y_start + cutter_diameter \* 0.60  
groove_y_end = parent_y_end - cutter_diameter \* 0.60

This means at least about 40 percent of the cutter diameter remains over
already-machined material at the boundary, with additional margin beyond
the bare cutter-radius requirement.

# 11. Minimum-diameter safety

The global minimum finished/root diameter is 20 mm. This applies not
only to nominal ROUND or TAPER diameters but also to local groove roots.
For variable parent profiles, the minimum is evaluated over the
applicable groove envelope rather than from a single nominal diameter.

# 12. Validation hierarchy

| **Stage** | **Must be true before generation**                                                                                                                           |
|-----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ROUND     | Machine configuration valid; stock/finished geometry legal; Y limits legal; raw Probe 1 Z supplied; A0 orientation and rollover confirmations complete.      |
| WAVE      | ROUND/base geometry valid; WAVE cutter and curvature legal; raw Probe 1 Z supplied; rotary limits legal.                                                     |
| TAPER     | ROUND/base geometry valid; WAVE count = 0; start/end diameters and Y limits legal; raw Probe 1 Z supplied.                                                   |
| GROOVE    | Operator-selected source explicitly valid; local root diameter legal; cutter-offset Y envelope non-empty; raw Probe 1 Z and operator confirmations supplied. |

# 13. Runtime estimation

Displayed runtime estimates are planning aids. A-axis cutting time can
be calculated directly from revolutions and RPM, but short Z and Y moves
may never reach configured maximum velocity because acceleration
dominates. The estimates should therefore be treated as comparative
rather than metrology-grade cycle prediction.

# 14. Calibration revisions through v2.23

v2.20 moved the calibrated A centreline X from the earlier nominal value
to 778.50 mm and changed the intended rotary Y start to Y0. Testing then
showed UCCNC rejects a commanded move exactly at the zero soft-limit
boundary, so v2.21 established Y_MACHINE_MIN = 1.0 mm. Further
opposite-side probing of a 12 mm test rod refined A_CENTER_X to 778.70
mm, and v2.22 adopted the 65.000 mm 40120-to-centre height. v2.23
retained those calibrations but changed the operational tool-height
workflow to the spoilboard-based 40 mm Probe 1 conversion, eliminating
the need to reach under mounted stock.

# 15. Current boundaries of the design

- The spindle does not tilt. TAPER is a practical fixed-spindle result,
  especially when using large flat cutters.

- There is no general-purpose arbitrary profile/spline engine.

- There is no automatic proof that GUI values still match a physical
  part machined days earlier.

- There is no current saved job recipe or stage-completion history.

- The code is intentionally machine-specific and should not be presented
  as safe for another machine without re-measuring its geometry.

# 16. Backlog ideas

- Saved job recipe / provenance record for disjointed multi-day
  workflows.

- Practical minimum cutter-diameter floor.

- Spherical/convex profile segment ("ball on a stick").

- Further WAVE roughing cycle-time optimisation after geometry remains
  stable.

- Publishing cleanup: configuration separation, README examples,
  changelog and source-level rationale comments.

- Potential future emboss/pattern operation for repeated motifs around a
  cylinder.

# 17. Source-commenting policy for publication

When the source is prepared for public release, comments should explain
non-obvious intent and safety reasoning rather than restating Python
syntax. Good candidates for detailed inline comments are the no-trench
calculations, A0 phase mapping, raw-Probe-1-to-A-centre conversion,
taper cutter-footprint protection, groove source selection and edge
setback.
