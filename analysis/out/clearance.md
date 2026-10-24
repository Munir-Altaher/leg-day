# Swept clearance check (analysis/clearance_check.py)

Required clearance 0.5 mm. Moving parts = STL point clouds (1.5 mm edge sampling); static parts = exact boxes/cylinders. The snowshoe is tested at both ankle stops (11 and 52 deg).

- Knee, q1 = 0: collision-free q2 in [-30, 152] deg (design limits (15.0, 145.0)) -> PASS
- Hip, all knee angles in limits (5 deg steps): collision-free q1 in [-107, 107] deg (design limits (-100.0, 100.0)) -> PASS

| q2 | hip free from | to |
|---|---|---|
| 15 | -107 | 107 |
| 20 | -107 | 107 |
| 25 | -107 | 107 |
| 30 | -107 | 107 |
| 35 | -107 | 107 |
| 40 | -107 | 107 |
| 45 | -107 | 107 |
| 50 | -107 | 107 |
| 55 | -107 | 107 |
| 60 | -107 | 107 |
| 65 | -107 | 107 |
| 70 | -107 | 107 |
| 75 | -107 | 107 |
| 80 | -107 | 107 |
| 85 | -107 | 107 |
| 90 | -107 | 107 |
| 95 | -107 | 107 |
| 100 | -107 | 107 |
| 105 | -107 | 107 |
| 110 | -107 | 107 |
| 115 | -107 | 107 |
| 120 | -107 | 107 |
| 125 | -107 | 107 |
| 130 | -107 | 107 |
| 135 | -107 | 107 |
| 140 | -107 | 107 |
| 145 | -107 | 107 |

## Ab/ad sweep (module + leg about X), hip and knee at several poses

- q1 =    0, q2 =  15: ab/ad collision-free [-46, 93] deg
- q1 =    0, q2 =  90: ab/ad collision-free [-46, 93] deg
- q1 =  -60, q2 = 120: ab/ad collision-free [-46, 97] deg
- q1 =   60, q2 =  30: ab/ad collision-free [-46, 94] deg
- q1 = -100, q2 = 150: ab/ad collision-free [-46, 87] deg
- q1 =  100, q2 =  15: ab/ad collision-free [-46, 101] deg
- q1 = -100, q2 =  15: ab/ad collision-free [-46, 113] deg
- q1 =  100, q2 = 150: ab/ad collision-free [-46, 101] deg

Ab/ad collision-free over all tested poses: [-46, 87] deg (design limits (-45.0, 45.0)) -> PASS

## Belt
- belt back to channel wall: knee 1.87 mm, hip 10.01 mm (>= 1 mm required); knee-pulley flange to wall 1.75 mm
- belt width 6.0 mm in a 9.0 mm channel: 1.5 mm per side