# LEG DAY: design rationale

**One 3-DOF quadruped leg (ab/ad + hip + knee), all three AK40-10s at the hip, belt-driven knee, PLA.**
The topology is MIT Mini Cheetah's (Katz, Di Carlo & Kim, ICRA 2019), scaled to Ø53 mm actuators. Every number below comes from `analysis/out/report.md`, `clearance.md`, `skeleton.md`, the generated CAD (`cad/parts/parts_report.json`) or the motor sheet.

## Headline numbers
| | Value | Source |
|---|---|---|
| Links | L1 = L2 = 130 mm (equal links: no unreachable hole near the hip) | sweep 110–160 mm |
| Belt drive | GT2 6 mm, 20T kit pulley at the hip → **printed 60T at the knee, N = 3.0** | §Load |
| Ab/ad offset l | **53 mm** (motor radius 26.5 + 2 mm clearance + thigh/shank stack) | params |
| Continuous foot force | **15.4 N** vertical with ±10 % lateral, all motors ≤ rated, belt ≤ 60 N | leg_model |
| Peak foot force | **30.8 N** (motors ≤ peak, belt ≤ 120 N) | leg_model |
| Ranges (collision-free) | ab/ad −99° … +98° (used ±45°), hip ±107° (used ±100°), knee −30° … 152° with the snowshoe (used 15° … 145°) | clearance_check |
| Swinging leg mass | ≈ 131 g (thigh, shank, foot, knee pulley, bearings, belt) | CAD volumes |
| Total leg incl. three motors | ≈ 0.87 kg | CAD + 3 × 185 g |
| Foot deflection | 0.87 mm at 15 N (target < 1 mm) | skeleton_check |
| Interferences | 0 at home pose (SolidWorks); swept check passes all ranges | build_assembly, clearance_check |

## 1. Mechanical design (50 %)

### Motor placement: actuator positioning that reduces moving mass
- All three motors sit at the hip. The hip and knee motors are coaxial on the hip axis, one each side of the thigh (the "sandwich"), and the ab/ad motor sits behind them. The swinging leg carries **no motor**, only ≈ 131 g of PLA, bearings and belt.
- **Rejected: knee motor at the knee.** It puts +185 g at 130 mm. Swinging mass goes 131 → 316 g (2.4×). Link inertia about the hip goes from ≈ 8.6 kg·cm² to ≈ 40 kg·cm² (4.6×). The analysis model gives 22 → 53 kg·cm² (2.4×) for the extended leg with foot and hardware. Low leg inertia is what makes a leg impact-tolerant (Wensing et al., T-RO 2017) and energy-efficient (Seok et al., T-Mech 2015).
- **Ab/ad motor behind and inboard** (−X, 53 mm inboard of the leg plane), so the thigh swings back past it with 2 mm clearance at every ab/ad angle. The thigh is ≥ 28.5 mm from the ab/ad axis and the motor radius is 26.5 mm, so the clearance is the same at every ab/ad angle.
  - Rejected: ab/ad motor in the leg plane (l = 0). It gives the lowest ab/ad torque, but the thigh couldn't swing back past it.
  - Rejected: ab/ad motor outboard of the hip stack (l ≈ 80 mm). Ab/ad torque would be 0.93 Nm instead of 0.51 Nm at 15 N with 10 % lateral.
- Rejected: coaxial five-bar (Minitaur, Doggo). No belt, but a smaller workspace with singularities inside it (Kenneally et al. 2016; Kau et al. 2019).

### Linkage: soundness and effectiveness
- **Belt-driven knee, 1 : 3.** The organizers' kit uses 20T both ends (1 : 1), which allows only ≈ 8 N at the foot before the belt reaches its 120 N peak. Belt tension for a given knee torque depends only on the knee pulley radius (T = τ_q2 / r_knee). The printed 60T knee pulley therefore triples knee torque per newton of belt: 2.29 Nm at the 120 N limit.
- **Sweep:** with a 40T / 48T / 60T knee pulley the continuous capability is 10.3 / 12.3 / **15.4 N**. The 15 N target needed the 60T pulley plus a straighter nominal stance (0.82 × (L1 + L2) = 213 mm). A printed 24–30T hip pulley would put more teeth in mesh but needs a 72–90T knee pulley for the same N, which doesn't fit the thigh.
- **Belt coupling handled in software:** θ_knee = q1 + N·q2. This was checked with a virtual-work identity in the model and is written out for the firmware in `BUILD_DAY_NOTES.md`.
- **The belt is the weakest link on purpose.** Firmware caps the knee motor at 0.76 Nm (120 N × 6.37 mm). That is ≈ 4.5× below the belt's breaking load, and below every printed part's limit.
- **Steel at every joint:** Ø5 steel (M5 shanks) on 625ZZ bearings at the hip shaft, knee and ab/ad stub axle. One axle size across the leg, matching the kit pulley bore. Torque never passes through a PLA shaft: the knee torque path is a hex head to a hex nut, both captured in 100 %-infill pockets.
- **Two-end supports** (for Stability):
  - The thigh is bolted to the hip motor on one side and runs on a bearing on the knee-motor shaft on the other.
  - The hip module is bolted to the ab/ad output at the back and carried on a steel stub axle in a 625 in the front bracket wall.

### Kinematics: range and quality of motion
- Hip ±100° (bridge stop at ±107°), knee 15°–145° (130° of travel, lowered from 150° so the snowshoe clears the hip module; the shank sits outboard of the thigh, so it folds without hitting it), ab/ad ±45° (mechanically ±98°). All three meet or beat the targets (≥ ±30°, ≥ ±100°, ≥ 130°).
- 15° of minimum knee bend keeps the leg away from the straight-knee singularity.
- FK/IK in closed form, with an FK→IK round trip over the joint ranges (worst error 3e-13°).
- Joint speeds: hip 38.7 rad/s, knee 12.9 rad/s at rated motor speed. A ±60 mm step in 0.2 s needs ≈ 0.26 Nm at the hip.

### Adaptability: one parameter edit + regenerate (`params.py`)
| To change | Edit | Then |
|---|---|---|
| Knee ratio | `KNEE_TEETH = 48` (pulley, channel, belt length and thigh width follow) | `build_parts.py`, `leg_model.py` |
| Link lengths | `L1`, `L2` | same |
| Ab/ad offset | computed from the stack (`ABAD_CLEAR`, `THIGH_WALL`, `SHANK_T`…) | same |
| Foot | `FOOT_D` | `build_parts.py foot` |
| Rig interface | `RIG_SLOT_GRID`, or swap `rig_plate` (4 screws) | `build_parts.py rig_plate` |
| Skeleton | `LEG_SKELETON=0` env var (or `SKELETON` in params), `SK_*` pattern params | `build_parts.py` |
| Fits | `FIT_BEARING`, `FIT_LOCATE`, `CLR`, `TAP` (from the coupon) | `build_parts.py` |

The 2-DOF fallback is the same `hip_module.SLDASM` bolted to a plate through the same back-flange pattern (3 × M2.5 on Ø27 + Ø15 pocket).

### Can it walk? (`analysis/gait_check.py`; animation `docs/renders/walk_cycle.gif`, drawn from the real CAD outlines)
I put four copies of this leg on an assumed 4.3 kg quadruped (4 × 0.87 kg legs + 0.8 kg body). Each foot follows a 120 mm stride with 40 mm swing clearance at the 213 mm stance height, and the leg model is checked through every pose.

- **Kinematics:** walking is fully feasible. The gait uses q1 −55…−12° and q2 58…94° (IK point = the snowshoe's ankle pivot), well inside the limits and inside the collision-free box. The fastest motor is the knee motor at 18 rad/s, against 38.7 rad/s rated.
- **Loads, walk** (lateral-sequence, 3 feet down, 0.16 m/s): the belt reaches 64 N and the hip motor 1.87 Nm. Both are above the continuous ratings (60 N, 1.3 Nm) and below the peaks (120 N, 4.1 Nm). The belt allows **4.0 kg** for continuous walking (the snowshoe's ankle pivot shortens the effective shank to 120 mm, which lowers the knee torque). The hip motor sets a similar or slightly lower limit; that number wasn't computed.
- **Loads, trot** (diagonal pairs, 0.48 m/s): belt 105 N, hip 3.1 Nm. Only short bursts are possible, and continuous trotting needs a robot of **≤ 2.4 kg**.
- **What it means:** the leg moves like a dog leg. A four-legged robot built from it can walk if the robot is kept to about 3.5 kg or the body is light, but continuous trotting needs a lighter robot or a stronger knee drive (wider belt, or a bigger knee pulley). On the competition's single-leg rig the walking motion is well within every limit.

## 2. Build quality (25 %)
- **Tolerances:** every fit is a parameter; a fit-test coupon (`coupon_fit_test`) calibrates them before anything else is printed.

  | Fit | Nominal | Designed |
  |---|---|---|
  | 625 OD in PLA | 16.00 | 16.10 |
  | Ø15 motor boss | 15.00 ± 0.05 | 15.20 pocket |
  | Ø5 axle | 5.00 | 5.20 |
  | M2.5 / M3 / M5 clearance | — | 2.9 / 3.4 / 5.5 |
  | M3 self-tap | — | 2.6 |
  | M5 head/nut hex | — | 8.2 AF |
  | Moving gaps | — | 0.5 mm per side |

- **Thread engagement** at the motors is designed so a standard screw gives 2.0–3.0 mm in the 3 mm and 5 mm motor holes and never bottoms out (fastener table in `BOM.md`).
- **Stability:** two-end supports at the thigh and at the ab/ad stage (above). Motor stators are located on the motor features and plates. The belt is preloaded ≥ 60 N with a lockable idler (9 mm slot travel ≥ ±3 mm cutting error + stretch). Gearbox backlash alone gives ≈ 1.4 mm at the foot; the structure adds 0.87 mm elastic deflection at 15 N.
- **Load capacity:** with Kt = 2 at the holes, the worst printed-part stress is ≈ 9.0 MPa (shank, 30 % lateral case, including the bend's axial-load moment), against 12.5 MPa allowable (25 MPa / SF 2). The shank at the 2.3 Nm belt-limited knee peak sees 6.0 MPa.
  - Hip-motor flange screws at the 4.1 Nm peak: 8.1 MPa bearing in PLA.
  - Ø5 steel shaft: 167 MPa at 4.1 Nm (fine for steel).
  - The firmware torque limits (knee 0.76 Nm, hip and ab/ad 3.0 Nm) mean the belt or the firmware gives before any part does.
- **The shank is 10 mm thick.** The first 8 mm version failed the lateral check (13.2 MPa) and the 1 mm deflection target (1.53 mm), so `SHANK_T` was raised to 10. Now: lateral 9.0 MPa and deflection 0.87 mm (with the bent shank). Both checks use a conservative flange-only section with no credit for the struts.
- **Not yet known:** the organizers' test load. Above ≈ 31 N vertical, the belt-limited firmware cap stops the knee before anything breaks.

## 3. Pitch (15 %): the story in three lines
1. **Mass where it costs nothing.** Three motors at the hip, a 131 g swinging leg, and a 4.6× lower link inertia than putting the knee motor at the knee.
2. **The belt decides the strength**, so we sized the knee pulley around it. 60T → 3 : 1 → 15 N continuous, 31 N peak, with a firmware limit that makes the belt the fuse.
3. **Everything is a parameter.** One `params.py` drives the analysis, the SolidWorks parts (built through the API) and the clearance checks. Change the knee pulley or the link length and the whole leg regenerates in about 5 minutes.

## 4. Aesthetics / compactness (10 %)
- Leg envelope at q = 0: 72 mm fore-aft (module) × 304 mm tall × 114 mm wide (module plates). Bracket and rig plate add 122 mm fore-aft.
- Thigh 66 mm deep including the screw ears, 19 mm thick. The shank is bent like Spot's lower leg (one gentle bow, convex to the front like Spot's, 9 mm off the knee-foot line) and tapers 26 → 22 mm into a 110 × 70 mm snowshoe on a friction-hinge ankle.
- Three large rounded-rectangle cutouts down the middle of each link, thick walls between them: 5 mm in the thigh walls, 7 mm in the bent shank's web (support-free). Chosen from the preview options in `docs/renders/cutout_options*.png` (`analysis/cutout_options.py`).

## Skeleton (brief §5.1): result
The skeleton is parametric, support-free and switchable (`LEG_SKELETON=0`).
- **Thigh:** 3 rounded-rectangle through-cuts between the belt spans, filleted in the sketch, plus walls thinned from 5 to 3 mm outside the bearing and motor bosses (in both variants).
- **Shank:** a channel section. Full-height 4.5 mm edge flanges stand on a 3 mm web on the bed face, so the pocket opens upward and needs no supports.

| | Solid | Skeleton | Saving |
|---|---|---|---|
| Shank (bent, 3 rectangle cutouts in the web) | 27.5 g | 18.2 g | **34 %** |
| Thigh (2 halves, 3 rectangle cutouts each) | 71.6 g | 65.9 g | 8 % |
| Thigh + shank | 99.2 g | 84.2 g | 15 % |
| Link inertia about the hip | 12.8 kg·cm² | 9.3 kg·cm² | **28 %** |

The shank was lightened hardest because a gram there is worth ≈ 8× a gram of thigh (m·r²). The combined 30 % mass target is **not met**: the thigh is a belt-carrying box whose walls are mostly keep-solid (belt guard bands, bearing rings, screw ears). All strength cases still pass with Kt = 2 (worst 9.0 MPa vs 12.5 allowable), and foot deflection is 0.87 mm. Next step: a 2D topology optimisation of the thigh walls (brief stretch goal; Andreassen et al. 2011).

## References
Katz, Di Carlo & Kim, ICRA 2019 (Mini Cheetah) · Grimminger et al., RA-L 2020 (ODRI Solo) · Wensing et al., T-RO 2017 · Seok et al., T-Mech 2015 · Kenneally, De & Koditschek, RA-L 2016 (Minitaur) · Kau et al., ICRA 2019 (Stanford Doggo) · Andreassen et al., SMO 2011 · Raibert 1986.
