# Print check — support-free + PrusaSlicer (analysis/print_check.py)

Orientation = the listed face on the bed (body-frame outward normal). Supports OFF. Bridges = flat downward regions off the bed (hole tops, pocket roofs), span = smaller side; ≤ 10 mm is printable unsupported. Steep overhang = downward facets steeper than 45° (round holes printed sideways show up here). PrusaSlicer 2.9.6 CLI, 0.4 nozzle, 0.45 lines, PLA 1.24 g/cm³.

| part | on bed | size mm | fits | max bridge span mm | bridges > 10 mm | steep overhang mm² | layer / perim / infill | time | g |
|---|---|---|---|---|---|---|---|---|---|
| snow_thigh_outer | +z | [np.float64(52.2), np.float64(180.0), np.float64(14.0)] | yes | 0.0 | 0 | 0.0 | 0.2 / 4 / 40% gyroid | 4h 53m 30s | 51.09 |
| snow_thigh_cover | -z | [np.float64(52.2), np.float64(180.0), np.float64(5.6)] | yes | 4.1 | 0 | 0.0 | 0.2 / 4 / 40% gyroid | 2h 56m 18s | 26.54 |
| snow_knee_pulley_60T | +z | [np.float64(40.7), np.float64(40.6), np.float64(7.5)] | yes | 2.9 | 0 | 0.0 | 0.12 / 3 / 100% rectilinear | 1h 30m 24s | 7.8 |
| snow_shank | -z | [np.float64(48.8), np.float64(164.2), np.float64(10.0)] | yes | 4.1 | 0 | 0.0 | 0.2 / 3 / 15% gyroid | 1h 50m 41s | 18.98 |
| snow_foot_snowshoe | +y | [np.float64(117.0), np.float64(80.0), np.float64(7.0)] | yes | 4.1 | 0 | 0.0 | 0.2 / 2 / 10% gyroid | 2h 13m 28s | 19.67 |
| snow_ankle_bracket_in | +z | [np.float64(46.0), np.float64(28.2), np.float64(16.0)] | yes | 0.9 | 0 | 11.9 | 0.16 / 3 / 30% gyroid | 35m 55s | 4.64 |
| snow_ankle_bracket_out | -z | [np.float64(46.0), np.float64(28.2), np.float64(16.0)] | yes | 0.9 | 0 | 11.9 | 0.16 / 3 / 30% gyroid | 36m 1s | 4.59 |
| snow_hip_shaft_adapter | -z | [np.float64(36.0), np.float64(36.0), np.float64(6.5)] | yes | 6.4 | 0 | 0.0 | 0.16 / 4 / 100% rectilinear | 43m 4s | 6.86 |
| snow_module_plate_inboard | -z | [np.float64(72.0), np.float64(58.3), np.float64(7.0)] | yes | 0.0 | 0 | 0.0 | 0.2 / 4 / 40% gyroid | 1h 58m 28s | 18.09 |
| snow_module_flange_back | -x | [np.float64(39.5), np.float64(52.0), np.float64(11.8)] | yes | 0.6 | 0 | 28.4 | 0.2 / 4 / 40% gyroid | 1h 5m 8s | 10.21 |
| snow_module_flange_front | +x | [np.float64(39.5), np.float64(52.0), np.float64(11.8)] | yes | 0.6 | 0 | 28.4 | 0.2 / 4 / 40% gyroid | 1h 6m 7s | 10.45 |
| snow_module_frame_outboard | +y | [np.float64(60.0), np.float64(113.5), np.float64(80.0)] | yes | 0.5 | 0 | 486.0 | 0.2 / 4 / 40% gyroid | 5h 9m 5s | 55.26 |
| snow_hip_shroud_out | +z | [np.float64(73.0), np.float64(80.5), np.float64(42.5)] | yes | 0.0 | 0 | 0.0 | 0.2 / 3 / 15% gyroid | 3h 59m 40s | 50.13 |
| snow_body_bracket_back | -x | [np.float64(67.0), np.float64(54.0), np.float64(28.0)] | yes | 0.9 | 0 | 46.0 | 0.2 / 4 / 40% gyroid | 1h 51m 35s | 18.04 |
| snow_body_bracket_front | +x | [np.float64(68.0), np.float64(48.0), np.float64(25.5)] | yes | 0.6 | 0 | 56.9 | 0.2 / 4 / 40% gyroid | 1h 42m 56s | 18.27 |
| snow_body_spine | -z | [np.float64(109.1), np.float64(60.0), np.float64(14.0)] | yes | 6.5 | 0 | 0.0 | 0.2 / 4 / 40% gyroid | 4h 46m 22s | 44.78 |
| snow_rig_plate | -z | [np.float64(109.1), np.float64(60.0), np.float64(8.0)] | yes | 4.5 | 0 | 0.0 | 0.2 / 4 / 40% gyroid | 3h 21m 55s | 30.63 |
| snow_coupon_fit_test | -z | [np.float64(175.0), np.float64(80.0), np.float64(15.1)] | yes | 0.0 | 0 | 0.0 | 0.2 / 4 / 100% rectilinear | 7h 3m 47s | 73.29 |

**Snow leg printed parts (excl. coupon): 40.3 h, 396 g.**

## Original Leg Day parts, same method (for COMPARISON.md)

| part | time | g |
|---|---|---|
| thigh_outer | 4h 50m 55s | 50.49 |
| thigh_cover | 2h 48m 43s | 25.39 |
| knee_pulley_60T | 1h 30m 23s | 7.93 |
| shank | 1h 30m 21s | 16.71 |
| foot | 40m 43s | 7.17 |
| hip_shaft_adapter | 43m 4s | 6.86 |
| module_frame_inboard | 3h 45m 23s | 40.57 |
| module_plate_outboard | 3h 7m 23s | 28.18 |
| body_bracket_back | 1h 34m 0s | 15.09 |
| body_bracket_front | 1h 15m 25s | 12.87 |
| body_spine | 2h 25m 12s | 23.55 |
| rig_plate | 2h 13m 28s | 20.14 |
| coupon_fit_test | 3h 58m 24s | 50.08 |

**Original printed parts (excl. coupon): 26.4 h, 255 g.**
