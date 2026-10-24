# Print check — support-free + PrusaSlicer (analysis/print_check.py)

Each STL is rotated so the listed face is on the bed (body-frame outward normal) and written to cad/stl/print/. Supports OFF. Bridges = flat downward regions off the bed, span = smaller side (≤ 10 mm prints unsupported). Steep overhang = downward facets steeper than 46° (small round holes printed sideways). PrusaSlicer 2.9.6 CLI, 0.4 nozzle, 0.45 lines, PLA 1.24 g/cm³; 'warnings' = PrusaSlicer's own support-spot check.

| part | on bed | fits 220×220×250 | max bridge mm | steep overhang mm² | layer / perim / infill | time | g | slicer warnings |
|---|---|---|---|---|---|---|---|---|
| thigh_outer | +z | yes | 0.0 | 0.0 | 0.2 / 4 / 40% gyroid | 4h 35m 47s | 48.59 | none |
| thigh_cover | -z | yes | 3.0 | 0.0 | 0.2 / 4 / 40% gyroid | 2h 37m 29s | 23.9 | none |
| knee_pulley_60T | +z | yes | 2.9 | 0.0 | 0.12 / 3 / 100% rectilinear | 1h 30m 24s | 7.8 | none |
| shank | -z | yes | 0.0 | 0.0 | 0.2 / 4 / 40% gyroid | 1h 43m 35s | 18.92 | none |
| snowshoe | -y | yes | 3.9 | 232.9 | 0.2 / 3 / 100% rectilinear | 3h 23m 58s | 30.78 | none |
| hip_shaft_adapter | -z | yes | 6.4 | 0.0 | 0.16 / 4 / 100% rectilinear | 43m 4s | 6.86 | none |
| module_plate_inboard | -z | yes | 0.0 | 0.0 | 0.2 / 4 / 40% gyroid | 1h 58m 27s | 18.09 | none |
| module_flange_back | -x | yes | 0.6 | 28.4 | 0.2 / 4 / 40% gyroid | 1h 5m 8s | 10.21 | none |
| module_flange_front | +x | yes | 0.6 | 28.4 | 0.2 / 4 / 40% gyroid | 1h 6m 7s | 10.45 | none |
| module_frame_outboard | +y | yes | 0.5 | 486.0 | 0.2 / 4 / 40% gyroid | 5h 1m 46s | 54.14 | none |
| body_bracket_back | -x | yes | 0.6 | 56.9 | 0.2 / 4 / 40% gyroid | 1h 50m 45s | 18.25 | none |
| body_bracket_front | +x | yes | 0.6 | 56.9 | 0.2 / 4 / 40% gyroid | 1h 42m 56s | 18.27 | none |
| body_spine | -z | yes | 4.0 | 0.0 | 0.2 / 4 / 40% gyroid | 3h 23m 59s | 31.51 | none |
| rig_plate | -z | yes | 3.2 | 0.0 | 0.2 / 4 / 40% gyroid | 2h 47m 7s | 25.41 | none |
| coupon_fit_test | -z | yes | 0.0 | 0.0 | 0.2 / 4 / 100% rectilinear | 3h 58m 24s | 50.08 | none |

**Printed parts excluding the coupon: 33.5 h, 323 g.**