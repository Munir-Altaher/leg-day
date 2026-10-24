# Sim mass report (sim/export_urdf.py)

| link | parts | mass g | COM (link frame) mm |
|---|---|---|---|
| base | body_bracket_back, body_bracket_front, body_spine, rig_plate, ref_motor_abad (+ 625 bearing (front wall) + 16 M3 screws/washers + 4 inserts) | 303.8 | (-43.0, 0.0, -13.3) |
| abad | module_plate_inboard, module_flange_back, module_flange_front, module_frame_outboard, ref_motor_hip, ref_motor_knee, hip_shaft_adapter, ref_pulley_20T_kit (+ M5 hip shaft + 13 M2.5 screws + 4 inserts + M2.5 x 12 x 5 + 350 g counterweight at z 95) | 846.1 | (0.0, 1.2, 57.8) |
| thigh | thigh_outer, thigh_cover (+ 3 x 625 bearings + GT2 belt + idler + 7 M3 screws) | 100.3 | (-0.0, -63.3, 38.4) |
| shank | shank, knee_pulley_60T (+ M5 x 25 knee axle + nylock + washers) | 33.2 | (3.2, -35.4, 48.4) |
| shoe | snowshoe (+ M3 x 25 ankle bolt + nylock + wave washer) | 33.3 | (-7.2, -9.2, 53.1) |

**Total 1317 g** (moving above the base: 1013 g)

## Joints
| joint | axis | limits deg | effort N m | velocity rad/s |
|---|---|---|---|---|
| abad | (1, 0, 0) | -45.0 … 45.0 | 4.10 | 38.7 |
| hip | (0, 0, 1) | -100.0 … 100.0 | 4.10 | 38.7 |
| knee | (0, 0, 1) | 15.0 … 145.0 | 2.29 | 12.9 |
| ankle | (0, 0, 1) | -52.0 … -11.0 | 0.00 | 50.0 |

Knee effort = min(3 x 4.1 N m motor, belt 120 N x r 19.1 mm) = 2.29 N m (continuous: 1.15 N m).

Ankle pivot is 15 mm above the sole; knee -> ankle pivot 120.2 mm.

Sideways balance: whole-leg CoM at z = 39.6 mm; snowshoe spans z 18 … 88 mm -> CoM 21.6 mm inside the inboard edge (counterweight 350 g).
