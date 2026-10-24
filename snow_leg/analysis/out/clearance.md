# Swept clearance check — snow leg (analysis/clearance_check.py)

Required clearance 0.5 mm. Moving parts = STL point clouds (2 mm edge sampling, ≤ 40k points per part); module parts = exact envelopes; thigh body = stadium outline × its z range. The knee-stop sleeves are meant to touch the thigh at 13° / 152° (outside the 15–150° firmware range).

Ankle angles tested: -10.6°, 18.5°, 47.5° (toe-up stop, neutral, toe-down stop).

- Knee sweep, q1 = 0, α = -10.6°: clear 15–137° -> PASS
- Knee sweep, q1 = 0, α = 18.5°: clear 15–137° -> PASS
- Knee sweep, q1 = 0, α = 47.5°: clear 15–137° -> PASS

- Hip × knee grid (3075 poses: q1 (-100.0, 100.0), q2 (15.0, 137.0), 5° steps × 3 ankle angles): 0 colliding -> PASS

## Ab/ad physical stops (blocks on the spine, x −20…6)

- plus: block y (-8.0, 4.0), top z -42.75 → module lands on it at +47.5° (other direction first touches it at 51.0°)
- minus: block y (-4.0, 8.0), top z -43.00 → module lands on it at −47.5° (other direction first touches it at 51.0°)

## Ab/ad sweep (module + leg about X) at several hip / knee / ankle poses

- q1 =  -10, q2 =  20, α = -10.6: ab/ad free [-46, 45]°
- q1 =  -10, q2 =  20, α =  47.5: ab/ad free [-46, 45]°
- q1 =    0, q2 =  90, α = -10.6: ab/ad free [-46, 45]°
- q1 =    0, q2 =  90, α =  47.5: ab/ad free [-46, 45]°
- q1 =  -60, q2 = 120, α = -10.6: ab/ad free [-46, 45]°
- q1 =  -60, q2 = 120, α =  47.5: ab/ad free [-46, 45]°
- q1 =   60, q2 =  30, α = -10.6: ab/ad free [-46, 45]°
- q1 =   60, q2 =  30, α =  47.5: ab/ad free [-46, 45]°
- q1 = -100, q2 = 150, α = -10.6: ab/ad free [-46, 45]°
- q1 = -100, q2 = 150, α =  47.5: ab/ad free [-46, 45]°
- q1 =  100, q2 =  15, α = -10.6: ab/ad free [-46, 45]°
- q1 =  100, q2 =  15, α =  47.5: ab/ad free [-46, 45]°
- q1 = -100, q2 =  15, α = -10.6: ab/ad free [-46, 45]°
- q1 = -100, q2 =  15, α =  47.5: ab/ad free [-46, 45]°
- q1 =  100, q2 = 150, α = -10.6: ab/ad free [-46, 45]°
- q1 =  100, q2 = 150, α =  47.5: ab/ad free [-46, 45]°

Ab/ad collision-free over all tested poses: [-46, 45]° (firmware (-45.0, 45.0); stops at ±48°) -> PASS

## Belt channel
- belt back to channel wall: knee 1.87 mm, hip 10.01 mm (≥ 1 mm); knee-pulley flange to wall 1.75 mm
- seam lip on the cover z 33.5…34.1 vs knee-pulley face z 34.0 (hub only) / flange z 34.5: axial gap to the flange 0.4 mm; lip inner radius at the knee 20.70 vs flange radius 20.34 mm