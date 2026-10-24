# Heat-set insert joints (analysis/insert_check.py)

Robot 4.5 kg; fall = 3 × weight = 132 N on one foot + 40 N lateral; capacity M3 800 N (brief 6.3 design value), M5 2000 N (ASSUMED: M3 value × insert surface-area ratio — confirm on the coupon). Required SF ≥ 3.

| joint | size | n | worst case | pull-out N | shear N | capacity N | SF | ok | note |
|---|---|---|---|---|---|---|---|---|---|
| thigh cover → thigh outer | M3 | 4 | fall + belt peak | 3 | 107 | 800 | 7.5 | yes | cover carries the hip-shaft bearing (belt pull) and half the knee bearing load; axial knee loads go to the outer wall |
| module flange back → inboard plate | M3 | 2 | fall (3 W + 30 % lateral), foot x -60, lateral - | 197 | 91 | 800 | 4.1 | yes | ab/ad output (back) / stub-axle bearing (front) reaction; ab/ad torque capped at the 4.1 Nm motor peak |
| module flange front → inboard plate | M3 | 2 | fall (3 W + 30 % lateral), foot x +60, lateral - | 98 | 85 | 800 | 8.2 | yes | ab/ad output (back) / stub-axle bearing (front) reaction; ab/ad torque capped at the 4.1 Nm motor peak |
| body back wall foot → spine | M5 | 2 | fall (3 W + 30 % lateral), foot x -60, lateral - | 558 | 115 | 2000 | 3.6 | yes | ab/ad motor stator reaction (torque + back support force), 40 mm lever to the spine; as M3 this joint would have SF 1.4 -> M5 chosen |
| body front wall foot → spine | M3 | 4 | fall (3 W + 30 % lateral), foot x +60, lateral + | 165 | 62 | 800 | 4.8 | yes | stub-axle bearing reaction, 46 mm lever to the spine |
| rig plate → spine (body mount) | M5 | 4 | fall (3 W + 30 % lateral), foot x +60, lateral + | 474 | 33 | 2000 | 4.2 | yes | whole leg reaction; 4 x M5 at x -48/-34, y ±19 (block widened to ±30); as M3 the same group would have SF 1.7 |
| ankle bracket flanges → foot deck | M3 | 4 | fall onto the heel tip, toe-down stop engaged | 200 | 10 | 800 | 4.0 | yes | moment 9.7 Nm about the pivot through the stop; screw couple across x ±15 mm |
| ankle stop cross-bolt → outer cheek | M3 | 1 | same heel fall | 0 | 243 | 800 | 3.3 | yes | stop force 487 N shared by both cheeks (bolt in double shear) |
| ice spikes (set screws) → foot | M3 | 4 | lateral fall on ice, 2 spikes engaged | 0 | 20 | 800 | 40.3 | yes | set screws pushed inward by the ground; shear only |
| spring posts (deck ×2, shank ×1) | M3 | 3 | spring at the stop | 4 | 0 | 800 | 200.0 | yes | 0.1 N/mm springs, 20 mm stretch |
| knee stop post (upper, 139°) → shank | M5 | 1 | fall onto a crouched leg at the stop | 0 | 333 | 2000 | 6.0 | yes | post 48.5 mm from the knee; foot under the hip |
| knee stop post (lower, 13°) → shank | M5 | 1 | fall onto a straight leg at the stop | 0 | 57 | 2000 | 35.0 | yes | post on the horn, 34 mm from the knee |
| knee stop posts at the belt limit | M5 | 1 | knee driven into a stop at 2.29 Nm | 0 | 67 | 2000 | 29.8 | yes | control-failure case |
| ball foot lap → shank (FOOT = 'ball') | M3 | 2 | fall | 0 | 69 | 800 | 11.6 | yes | lap joint in shear |

## Geometry rules (brief 6.4)

- hole Ø as modelled: M3 4.1 mm, M5 6.5 mm (datasheet + 0.1 mm print compensation; coupon holes at ±0.1 to calibrate)
- depth ≥ insert length + 2 pitches: M3 6.7 mm, M5 11.1 mm (the two M5 knee-stop inserts sit in through-holes of the 10 mm shank)
- boss radius = hole radius + one insert OD: M3 r 5.05 (wall 3.00 mm ≥ 3), M5 r 7.00 (wall 3.75 mm)
- every insert hole axis is parallel to its part's print-bed normal (vertical when printed): thigh outer (z, bed +z), inboard plate (z, bed −z), spine (z, bed −z), foot (y, bed +y), outer cheek (z, bed −z), shank (z, bed −z)

**All joints SF ≥ 3: YES**

Not inserts (brief 6.1 exceptions): M2.5 screws into the motors' own threads (hip, knee, ab/ad flanges and back faces; the C-frame tab is clamped by the 5 knee-motor back screws), the torque-carrying hex pockets (knee M5 head and nylock, hip-shaft M5 head), the idler bolt (M3 + nut: it slides in a slot).