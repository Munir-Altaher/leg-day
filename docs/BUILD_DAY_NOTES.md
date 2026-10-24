# Build-day notes (Thu 29 Oct)

## CAN IDs and modes
| Joint | Motor | CAN ID | Mode |
|---|---|---|---|
| ab/ad (q0) | AK40-10 behind the hip | 1 | MIT mode |
| hip pitch (q1) | AK40-10 outboard (bolted to the thigh) | 2 | MIT mode |
| knee (q2) | AK40-10 inboard (drives the belt) | 3 | MIT mode |

## Joint ↔ motor mapping (belt coupling, verified by a virtual-work check in `analysis/leg_model.py`)
N = 60 / 20 = **3.0** (printed 60T knee pulley, 20T kit hip pulley, open belt)

```
θ_abad  = q0
θ_hip   = q1
θ_knee  = q1 + N·q2            ← the knee motor must follow the hip
τ_knee_motor = τ_q2 / N
τ_hip_motor  = τ_q1 − τ_q2 / N
belt effective tension T = |τ_knee_motor| / r_hip = |τ_q2| / r_knee     (r_hip 6.37 mm, r_knee 19.10 mm)
```
Swinging the hip with the knee motor held still **bends the knee** (by −q1/N). Always command θ_knee = q1 + N·q2.

Check the signs before closing the loop. Power only the knee motor, turn it +10°, and confirm the knee moves +3.3° (foot swings forward, counter-clockwise seen from +Z / outboard). If not, flip the knee direction in software.

## Conventions
Body frame: origin = hip axis ∩ ab/ad axis, X forward, Y up, Z outboard. q1 = 0 means thigh straight down, + = foot forward. q2 = 0 means straight, + = knee bends backward (knee behind, foot forward). q0 is right-hand about +X.

## Limits
| | Firmware limit | Mechanical / reason |
|---|---|---|
| q0 | −45° … +45° | free to ±98°; cable service loop sized for ±45° |
| q1 | −100° … +100° | module bridge stop at ±107° |
| q2 | +15° … +150° | singular at 0°; free −30° … 163° |
| knee motor torque | **0.76 Nm** | belt peak 120 N × r_hip 6.37 mm |
| knee motor continuous | 0.38 Nm | belt 60 N |
| hip motor torque | 3.0 Nm | PLA bearing at the 3 × M2.5 flange is 8.1 MPa at 4.1 Nm (allowable 12.5), keeps margin |
| ab/ad motor torque | 3.0 Nm | same flange check (7.4 MPa at 4.1 Nm) |
| current (all) | 2.7 A continuous | PLA softens at 55–60 °C; keep motors near rated |

## IK (frame B → joints), l = 53 mm (`L_ABAD`), L1 = L2 = 130 mm
```
D  = sqrt(y² + z² − l²)
q0 = atan2(z, y) − atan2(l, −D)
q2 = +acos((x² + D² − L1² − L2²) / (2·L1·L2))
q1 = atan2(x, D) − atan2(L2·sin q2, L1 + L2·cos q2)
```
`analysis/leg_model.py` has `fk()`, `ik()` and `jac()` ready to copy. The FK→IK round trip passes to 1e-13°.

Nominal stance: foot 213 mm below the hip axis (q1 = −34.9°, q2 = 69.8°). That stance carries 15.4 N continuous within rated torque and a 60 N belt, or 30.8 N peak.

## Zeroing
1. Hang the leg straight (q1 = q2 = 0) against a printed 90° square, with ab/ad level (thigh plane vertical).
2. Set zero on all three motors in that pose. At zero the knee motor reading equals q1 + 3·q2 = 0.

## First-power checklist
- [ ] Belt tensioned: press the span midway, ≈ 2–3 mm deflection under a firm finger (≥ 60 N preload), idler locked
- [ ] All grub screws threadlocked on the filed flat; knee M5 nylock threadlocked
- [ ] Torque limits loaded **before** enabling: knee 0.76, hip 3.0, ab/ad 3.0 Nm
- [ ] Position limits loaded (table above)
- [ ] Low gains first (kp 2, kd 0.1); jog each joint ±10°, check the direction and the knee coupling
- [ ] Cables: service loop at both ab/ad limits, nothing touches the thigh at q1 = ±100°
- [ ] Re-tension the belt after 10 minutes of running (PLA creep)
