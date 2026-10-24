# Print settings (PLA, 0.4 mm nozzle, 0.45 mm lines, 220 × 220 × 250 bed)

Print-oriented STLs are in `cad/stl/print/` (`analysis/print_check.py` rotates the SolidWorks exports in `cad/stl/`). **No supports on any part.** Every part was sliced with supports off in PrusaSlicer 2.9.6. Its support-spot check result and the bridge / overhang check are in `analysis/out/print.md`. Times and grams below are the slicer's.

| part | orientation on bed | layer | perimeters | infill | top/bottom | time | g | notes |
|---|---|---|---|---|---|---|---|---|
| thigh_outer | outboard face (+z) down | 0.2 | 4 | 40% gyroid | 4 | 4h 35m 47s | 48.59 | channel side up |
| thigh_cover | inboard face (−z) down | 0.2 | 4 | 40% gyroid | 4 | 2h 37m 29s | 23.9 | outer (inboard) face down |
| knee_pulley_60T | outboard face (+z) down | 0.12 | 3 | 100% rectilinear | 5 | 1h 30m 24s | 7.8 | teeth vertical, flat face down, 0.12 mm layers |
| shank | inboard face (−z) down | 0.2 | 4 | 40% gyroid | 4 | 1h 43m 35s | 18.92 | web pocket and hex pocket open upward |
| snowshoe | −y face down | 0.2 | 3 | 100% rectilinear | 4 | 3h 23m 58s | 30.78 | sole down (closed skin on the bed), Voronoi ribs and cheeks rise from it; lips bridge 10.4 mm |
| hip_shaft_adapter | inboard face (−z) down | 0.16 | 4 | 100% rectilinear | 4 | 43m 4s | 6.86 | motor face down |
| module_plate_inboard | inboard face (−z) down | 0.2 | 4 | 40% gyroid | 4 | 1h 58m 27s | 18.09 | insert zones up |
| module_flange_back | back face (−x) down | 0.2 | 4 | 40% gyroid | 4 | 1h 5m 8s | 10.21 | flat: the Ø15 locating hole and the bolt holes print round |
| module_flange_front | front face (+x) down | 0.2 | 4 | 40% gyroid | 4 | 1h 6m 7s | 10.45 | flat: stub-axle hex pocket up |
| module_frame_outboard | +y face down | 0.2 | 4 | 40% gyroid | 4 | 5h 1m 46s | 54.14 | lying on the column's top face; both plates stand; teardrop cable holes |
| body_bracket_back | back face (−x) down | 0.2 | 4 | 40% gyroid | 4 | 1h 50m 45s | 18.25 | wall face down: bores vertical, foot rises |
| body_bracket_front | front face (+x) down | 0.2 | 4 | 40% gyroid | 4 | 1h 42m 56s | 18.27 | wall face down: bores vertical, foot rises |
| body_spine | inboard face (−z) down | 0.2 | 4 | 40% gyroid | 4 | 3h 23m 59s | 31.51 | back block and inserts up; rig inserts from the bed face |
| rig_plate | inboard face (−z) down | 0.2 | 4 | 40% gyroid | 4 | 2h 47m 7s | 25.41 | counterbores on the bed face |
| coupon_fit_test | inboard face (−z) down | 0.2 | 4 | 100% rectilinear | 4 | 3h 58m 24s | 50.08 | print first |

**Total excluding the coupon: 33.5 h, 323 g.**

General: add a 0.4 mm elephant's-foot compensation (or 0.4 mm chamfer) on the bed face. Never fillet a hole edge. Bearing seats are Ø16.10 (625 + 0.10); hole sizes come from the coupon, so regenerate if the coupon says otherwise. PLA softens around 55 °C: keep printed parts off the motor cans (0.5 mm air gap everywhere except the bolted rotor faces).