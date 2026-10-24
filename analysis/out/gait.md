# Gait check: can a quadruped of these legs walk? (analysis/gait_check.py)

Robot assumed = 4 legs × 0.87 kg + 0.8 kg body = **4.28 kg**. Stride 120 mm, swing clearance 40 mm, hip 224 mm above the ground (ankle pivot 209 mm below the hip, snowshoe foot), ab/ad held at 0. Stance loads = weight shared over the feet on the ground + 10 % lateral; swing = link gravity + inertia.

## walk: period 1.0 s, duty 0.75, forward speed 0.16 m/s

| check | result | value |
|---|---|---|
| joint angles inside limits (collision-free box) | PASS | q1 -55…-12°, q2 58…94° |
| motor speeds ≤ rated 38.7 rad/s | PASS | hip 6.3, knee motor 19.9 rad/s |
| motor torques ≤ rated 1.3 Nm | **FAIL** | ab/ad 0.45, hip 1.87, knee 0.41 Nm |
| knee motor ≤ 0.76 Nm firmware limit | PASS | 0.41 Nm |
| belt ≤ 60 N continuous | **FAIL** | 64 N |
| belt ≤ 120 N peak | PASS | 64 N |

Max robot mass for this gait: **4.0 kg** with the belt at its 60 N continuous rating, 8.0 kg at the 120 N peak.

## trot: period 0.5 s, duty 0.5, forward speed 0.48 m/s

| check | result | value |
|---|---|---|
| joint angles inside limits (collision-free box) | PASS | q1 -55…-12°, q2 58…94° |
| motor speeds ≤ rated 38.7 rad/s | PASS | hip 6.3, knee motor 20.2 rad/s |
| motor torques ≤ rated 1.3 Nm | **FAIL** | ab/ad 0.67, hip 3.19, knee 0.61 Nm |
| knee motor ≤ 0.76 Nm firmware limit | PASS | 0.61 Nm |
| belt ≤ 60 N continuous | **FAIL** | 96 N |
| belt ≤ 120 N peak | PASS | 96 N |

Max robot mass for this gait: **2.7 kg** with the belt at its 60 N continuous rating, 5.3 kg at the 120 N peak.
