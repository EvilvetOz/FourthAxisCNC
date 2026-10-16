# Changelog

This file records major development milestones. Minor UI-only revisions are omitted.

| Version | Major change |
|---|---|
| v1.01 | First MkII generator foundation for ROUND, WAVE and GROOVE. |
| v1.04 | Introduced progressive indexed square-to-round roughing with no-trench intent. |
| v1.05 | Corrected the 45-degree model-to-machine A-phase mapping so physical A0 is the original corner-up orientation. |
| v1.06 | Removed unused RPM macros and used direct A-axis feed control; complete ROUND -> WAVE -> GROOVE workflow tested. |
| v1.07 | Added multi-pass GROOVE radial DOC with phase-aligned multi-start passes. Locked Phase One baseline. |
| v2.01 | Began Phase Two UCCNC-style GUI redesign and improved job/output handling. |
| v2.09 | Updated permanent rotary-bed geometry and stock/Y safety model. |
| v2.10 | Made the permanent 40120 rotary bed the A-centre Z reference. Locked ROUND baseline. |
| v2.12 | Expanded WAVE to +/-5 waves; WAVE cutter geometry defaults from ROUND but remains editable. |
| v2.14 | Removed experimental G92 rebasing; raised verified UCCNC A soft limit to +/-3,600,000 degrees. Locked WAVE baseline. |
| v2.15 | Added TAPER tab between WAVE and GROOVE. |
| v2.16 | Replaced slow full-length taper roughing with thin-end-first indexed rings and cutter-footprint no-trench protection. |
| v2.17 | Added explicit GROOVE base-profile selection: ROUND, WAVE or TAPER. |
| v2.18 | Added source-aware GROOVE Y inheritance and 60% cutter-diameter edge setback. |
| v2.20 | Updated calibrated A centre X and machine Y minimum. |
| v2.21 | Set generated Y minimum to 1.0 mm to avoid UCCNC soft-limit rejection at exact Y0. |
| v2.22 | Calibrated A centre X to 778.70 mm and 40120-to-centre height to 65.00 mm. |
| v2.23 | Reworked Probe 1: 40 mm block on spoilboard, raw G53 Z entered directly, A-centre Z derived with a +29.862 mm net correction. |
