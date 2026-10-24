> Snow variant (Arctic search-and-rescue): see `snow_leg/` and `snow_leg/CLAUDE.md`.

# LEG DAY: what has been built (state at hand-in, Thu 22 Oct 2026)

This file is a factual handover. It describes exactly what exists in SolidWorks and in this repo, how it is generated, what was verified, and what is weak, so that another agent can write an improvement brief. The original design brief is `C:\Users\<user>\Downloads\LEG_DAY_PROMPT.md`. All numbers below come from the generated files; if they disagree with this text, the files win.

## 1. What the thing is
- One leg of a quadruped for the IEEE IES × Robotics Club "Leg Day" competition, made for **PLA printing**.
- **3 DOF: ab/ad (hip roll) + hip pitch + knee pitch**, driven by three **CubeMars AK40-10** actuators (Ø53 × 37 mm, 1.3 Nm rated / 4.1 Nm peak, 185 g, no shaft: the output is the rotor face with 3 × M2.5 on Ø27 and a Ø15 boss).
- Mini Cheetah topology: all three motors at the hip. The knee is driven by the organizers' GT2 belt kit, running inside the thigh.
- **Built two ways, in this order:** `hip_module.SLDASM` (2-DOF, can be handed in alone on a rig plate) nested in `leg_3dof.SLDASM` (adds the ab/ad motor and the body bracket).

## 2. How it is generated (Route A: SolidWorks API from Python)
- Environment: Windows 11, **SolidWorks 2025 Student (rev 33.4)**, Python 3.14 + `pywin32`, `numpy`, `matplotlib`, `Pillow`. No SolidWorks Simulation.
- **Nothing in SolidWorks was modelled by hand.** Every part and assembly is produced by scripts. Edit `params.py`, then run, in order:
  1. `python params.py` (rewrites `params.json`)
  2. `python analysis/leg_model.py`
  3. `python cad/build_parts.py` (optionally with part names; `LEG_SKELETON=0` builds solid variants for mass comparison only, never saved over the parts)
  4. `python cad/build_assembly.py`
  5. `python cad/verify_mates.py`
  6. `python analysis/clearance_check.py`
  7. `python analysis/skeleton_check.py`
  8. `python analysis/gait_check.py`
  9. (on request only) `python cad/walk_animation.py`: the SolidWorks walking render.
- `cad/swlib.py` is a tiny COM helper. A part is a stack of **2.5D "slabs"**: closed loops (circles, lines and arcs, stadiums, rounded polygons) sketched on a plane parallel to Front (XY), Right (YZ) or Top (XZ) and extruded between two coordinates along the plane normal. Inner loops become holes. Extra features: `cut()` (blind cut between two coordinates), `revolve_capsule()` (the retired domed foot), `rotate_z()` (Move/Copy Body; not used by the snowshoe, see its row), `("spline", pts)` path segments (the bent shank), `axis()` (named reference axes for mates), `teardrop()` (holes printed sideways).
- Consequence: **every part is a prismatic/extruded body** . There are no fillets on 3D edges, no chamfers, no lofts or sweeps, and no real GT2 tooth profile: the knee pulley grooves are 60 round cuts R0.555 × 0.75 deep.

### SolidWorks API gotchas found (keep these)
- Creating a reference plane at a **negative** offset through `InsertRefPlane` hung SolidWorks. Workaround: every part is modelled in the body frame **shifted by +300 mm on X, Y and Z** (`swlib.SHIFT`), so all plane offsets are positive. **Each part's own origin is at (−300, −300, −300) mm** relative to the leg. The part planes are still parallel to the body frame, and all parts share one frame.
- `FeatureCut4` cuts toward −normal by default; the helper passes `Dir=True`. `FeatureExtrusion3` goes +normal.
- `AddComponent5` places the component's **bounding-box centre** at the given point, not its origin. `build_assembly.place()` passes each part's body-frame box centre, so parts land in their home pose.
- Late-bound COM: several methods are properties (`GetTitle`, `GetMathUtility`, `CreateMassProperty`, `EditRebuild3`). `OpenDoc6` / `AddMate5` need `VARIANT(VT_BYREF|VT_I4)` for their by-ref arguments. The first component inserted into an assembly is auto-fixed.
- Angle mates between planes are **unsigned (0–180°)**.
- Mate features can't be renamed through `FeatureByPositionReverse(0)`, so the names stay `Angle1`, `LimitAngle1`, …

## 3. Coordinate frame and joint conventions
- **Body frame (the assembly origin):** origin = hip-pitch axis ∩ ab/ad axis. X forward (= ab/ad axis), Y up, Z lateral, positive outboard.
- Home pose: q0 = q1 = q2 = 0, leg hanging straight down.
- **Leg plane (shank/foot mid-plane) is z = l = 53 mm** (`L_ABAD`).
- q1 is the thigh angle from straight down (+ = foot forward). q2 is the knee angle relative to the thigh (+ = knee bends backward, Mini Cheetah style). q0 is right-handed about +X.
- **Belt coupling:** N = 60/20 = 3.0. θ_knee_motor = q1 + N·q2; τ_knee_motor = τ_q2 / N; τ_hip_motor = τ_q1 − τ_q2 / N.

### Lateral stack (body-frame z, mm)

| Item | z range |
|---|---|
| Module inboard plate | −20 … −15 |
| Knee motor (output at high z) | −15 … 22 |
| Hip-shaft adapter | 22 … 28 |
| Thigh cover (inboard wall) | 28.5 … 33.5 |
| Belt channel (9 mm; 6 mm belt) | 33.5 … 42.5 |
| Thigh outboard wall | 42.5 … 47.5 |
| Shank (10 mm plate, leg plane at 53) | 48 … 58 |
| Snowshoe pad (110 × 70) | centred at z 53 (18 … 88) |
| Hip motor (output at low z, bolted to the thigh outer wall) | 47.5 … 84.5 |
| Module outboard plate | 84.5 … 89.5 |

- **Ab/ad motor** sits behind the module: x −73 … −36, coaxial with the X axis. It clears the thigh by 2 mm at every ab/ad angle, because the thigh is ≥ 28.5 mm from the axis and the motor radius is 26.5 mm.
- **Body bracket:**
  - back wall: x −79 … −73
  - front wall: x 36.5 … 42.5
  - spine: z −52 … −46 (+ end blocks)
  - rig plate: z −58 … −52

## 4. Parts (all in `cad/parts/` as SLDPRT + STEP, and `cad/stl/` as STL in the assembly pose)

| Part | What it is / key features |
|---|---|
| `thigh_outer` | Outboard half of a box-section thigh. A stadium outline (R23 at the hip, R26.1 at the knee, L1 = 130) carries a 9 mm-tall rim around a belt channel (inner radii 17.5 / 22.1). The wall is 3 mm, with 5 mm bosses at the hip (R20) and knee (R12). Hip: Ø15.2 boss pocket + 3 × Ø2.9 on Ø27 bolting to the hip-motor rotor. Knee: 625ZZ seat Ø16.1 with a 0.5 mm lip. 3 M3 cover bosses at the hip, 4 screw ears (2 at y −80, 2 at the knee), an idler slot (M3, 9 mm travel at y −40), and 3 large rounded-rectangle cutouts down the middle between the belt spans (`rect_cutouts()`, y −45 to −116, 5 mm walls between them, following the taper). Named axes HIP_AXIS, KNEE_AXIS. |
| `thigh_cover` | Inboard wall (3 mm, 5 mm bosses). 625 seats at the hip (rides on the knee-motor shaft, giving two-sided thigh support) and at the knee. Counterbored M3 holes, idler slot, the same 3 rectangle cutouts. |
| `knee_pulley_60T` | Printed, PD 38.20 / OD 37.69, 6.4 mm tooth face + two flanges, Ø9 hubs bearing on the 625 inner races, M5 nylock hex pocket, 6 lightening holes. Sits in the channel. |
| `shank` | 10 mm plate **offset outboard of the thigh** (not in-line), **bent like Spot's lower leg**: the outline follows a cubic-Bezier centre line (splines in the sketch) that leaves the knee 10° forward, bows 9 mm off the knee-foot line (convex to the front, the side the knee folds toward, as on Spot) and meets the foot 10° back. Knee folds to 152° with the snowshoe at either stop (limit now 145°). Tapers R13 → R11. M5 head hex pocket at the knee (torque path: hex head → M5 bolt → nylock in pulley). Channel-section skeleton: a pocket following the bend leaves 4.5 mm flanges on a 3 mm web, with 3 rounded-rectangle cutouts along the bend (7 mm walls between them, 2.5 mm inside the pocket edge, kept smaller than the thigh's for strength). A 12 × 10 end plug on the tilted foot axis. The plug's M3 cross hole is the ankle pivot. Named KNEE_AXIS, ANKLE_AXIS. |
| `snowshoe` | **Snowshoe foot (replaces the ball, design review v2).** 110 × 70 mm teardrop pad, 59 cm² of sole, on the shank's existing M3 cross-bolt, which is now the ankle (pitch) pivot, 15 mm above the sole. A closed 0.8 mm skin on the snow side carries a generative Voronoi lattice of 1.2 mm ribs, 3 mm tall: cells grow from 5.5 mm at the pivot to 11 mm at the edge (`cad/snowshoe_geo.py`). Three tapered keels (2.4 mm, 10 → 6 mm tall) carry the toe/heel bending into a solid hub; 2.4 × 6 mm rim; toe bevelled up 5 mm. Two cheeks either side of the plug and two lips that catch the plug's flat faces at shank angle 11° and 52° in the pad frame (walking stance needs 16–47°). Friction hinge: M3 wave washer under the nylock (~90 mN·m) so the pad holds its lift-off angle in swing and always lands toe-up; the ground flattens it. Modelled flat (part planes = pad frame); the assembly's ankle mate sets 32°. Named ANKLE_AXIS. Prints sole-down, no supports. |
| `hip_shaft_adapter` | Bolts to the knee-motor rotor (3 × M2.5 counterbored, Ø15.2 pocket). Captures an M5 bolt head that acts as the Ø5 hip shaft carrying the kit 20T pulley. |
| `module_plate_inboard` | Inboard plate (knee motor back face, 5 × M2.5 on Ø45), thickened to 7 mm under the flange feet with 4 × M3 heat-set inserts. Named HIP_AXIS, ABAD_AXIS. |
| `module_flange_back` / `module_flange_front` | L-brackets printed flat: back = ab/ad output interface (3 × M2.5 on Ø27 + Ø15.2 through-hole), front = M5 stub-axle hex pocket. Each foot screws down into the plate (2 × M3 × 12). |
| `module_frame_outboard` | C-frame: outboard plate (hip motor back face) + bridge column over the top (x ±11, y 28.5–40; hip hard stop ±107°) + an inboard tab plate with 45° chamfers, clamped to the inboard plate by the 5 knee-motor back screws (M2.5 × 12). Printed lying on the column's top face. Teardrop cable holes. |
| `body_bracket_back` | Wall holding the ab/ad motor's back face (5 × M2.5 on Ø45, Ø20 cable hole). |
| `body_bracket_front` | Wall with the 625 for the ab/ad stub axle (two-end support of the module). |
| `body_spine` | Joins the walls, inboard of the leg; 2 zip-tie slots; insert holes. |
| `rig_plate` | Swappable test-rig plate, 4 × M5 slots on a 20 mm grid. |
| `coupon_fit_test` | Fit coupon: Ø15 pockets +0.10…0.25, 625 seats +0.05…0.20, Ø5 holes, clearance/tap/insert holes, hex pockets, a skeleton strut sample. **No embossed labels.** |
| `ref_motor_hip/knee/abad` | Simplified motor bodies (Ø53 cylinder + Ø37 × 1 step). **Not the official CubeMars STEP.** |
| `ref_pulley_20T_kit` | Plain Ø12.22 cylinder standing in for the kit pulley. |
| `ref_ground` | Ground plate for the walking animation only. |

Not modelled at all: the belt, idler, belt clamp, bearings, screws, cables, hard-stop lugs for the knee and ab/ad.

## 5. Assemblies and mates
- **`hip_module.SLDASM`:**
  - Fixed: frame, outboard plate, hip and knee motors.
  - Hip revolute: HIP_AXIS coincident + Front planes coincident + limit angle (thigh Right vs frame Right) 0–100°.
  - Cover rigid to the thigh (3 plane coincidences).
  - Adapter + kit pulley spin freely on HIP_AXIS.
  - Knee revolute: KNEE_AXIS + Front planes + limit angle 0–145°.
  - Pulley rigid to the shank. **Ankle:** `ANKLE_axis` + `ANKLE_plane` + `ANKLE_limit` (snowshoe Top Plane ↔ shank Right Plane, 38–79° = 90° − shank angle 52…11°; this pair is unambiguous, the mirrored solution reads 101–142°). Drag the snowshoe to see the lips stop it.
- **`leg_3dof.SLDASM`:**
  - Bracket, spine, rig plate and ab/ad motor fixed.
  - `hip_module` is a **flexible, unfixed** sub-assembly.
  - Ab/ad revolute: module ABAD_AXIS ↔ spine ABAD_AXIS + Right planes + limit angle (Top vs Top) 0–45°.
- **`walk_anim.SLDASM`:** the full leg + ground plate. The hip and knee use **driving** angle mates (hip value = 90° − q1, knee value = q2) instead of limits; used for the animation.
- **Verified** (`cad/parts/mate_check.txt`): no mate errors; each joint moves when driven inside its range; each limit stops the joint when driven past it. 0 interferences at the home pose in both assemblies (`interference_home_pose.txt`).
- Known mate limitations:
  - The angle limits are unsigned, so the knee CAD limit is 0–145° (firmware: 15–145°; was 150°, lowered for the snowshoe).
  - No gear mate couples the hip pulley to the knee pulley.
  - No physical knee or ab/ad stops.

## 6. Analysis and verification results (`analysis/out/*.md`)

**Kinematics** (`report.md`)
- L1 = L2 = 130 mm, nominal stance height 213 mm (0.82 × (L1 + L2)); at that stance q1 = −34.9°, q2 = 69.8°.
- FK→IK round trip passes (3e-13°). The belt-coupling virtual-work check passes.
- The brief's sanity anchors are reproduced (8.3 / 16.6 / 19.9 / 24.9 N for 20/40/48/60T; ab/ad 0.80 Nm).

**Force capability**
- 15.4 N continuous vertical with ±10 % lateral (motors ≤ rated, belt ≤ 60 N); 30.8 N peak (belt ≤ 120 N). The **belt governs**.
- Firmware knee-motor limit = 0.76 Nm.
- Belt cut length 372 mm (342 mm loop + clamp overlap).

**Clearance** (`clearance.md`, from STL point clouds vs. primitives)

| Joint | Collision-free range | Limit used |
|---|---|---|
| Hip | ±107° at every knee angle | ±100° |
| Knee | −30° … 152° (snowshoe at either stop) | 15–145° |
| Ab/ad | −99° … +98° | ±45° |

Belt-to-wall clearance ≥ 1.75 mm.

**Strength and stiffness** (`skeleton.md`; hand beam model, Kt = 2, allowable 12.5 MPa = 25 MPa / SF 2)
- All cases pass. Worst: 9.0 MPa (shank, 30 % lateral; includes axial load × the 9 mm bend offset).
- Foot deflection 0.87 mm at 15 N (target < 1 mm). Gearbox backlash alone gives ~1.4 mm of play at the foot.
- **No FEA was run.**

**Mass**
- Swinging leg ≈ 131 g.
- Whole leg with 3 motors ≈ 0.87 kg.
- Printed parts are estimated at 0.75 × solid volume (not weighed).

**Skeleton**

| | Saving vs solid |
|---|---|
| Shank | 34 % |
| Thigh | 6–11 % |
| Thigh + shank | **15 % (target was 30 %: not met)** |
| Link inertia about the hip | 28 % |

**Gait** (`gait.md`)

The test is four copies of the leg on an assumed 4.3 kg robot, with a 120 mm stride and 40 mm clearance. Kinematically fine: q1 −55…−12°, q2 58…94°, motor speeds ≤ 20 rad/s. The IK point is the snowshoe's ankle pivot (L2 = 120.2 mm to it), with the hip 224 mm above the ground as before.

| Gait | Belt | Hip motor | Continuous mass limit (belt) |
|---|---|---|---|
| Walk (0.16 m/s) | 64 N | 1.87 Nm | 4.0 kg |
| Trot (0.48 m/s) | 96 N | 3.19 Nm | 2.7 kg |

Both gaits exceed the continuous ratings (60 N, 1.3 Nm) and stay within the peaks.

**Animations**
- `docs/renders/walk_cycle.gif` (**the walking animation; regenerate this one**): matplotlib side view of a quadruped of four legs + a single-leg panel with the foot path + live load bars. The legs are drawn from the real CAD outlines in `cad/build_parts.py` (bent shank, cutouts, snowshoe: flat in stance, holding its lift-off angle in swing), so a geometry change shows up after `python analysis/gait_check.py`.
- `docs/renders/walk_solidworks.gif`: SolidWorks-rendered walk (`cad/walk_animation.py`, made on request), 36 frames of `walk_anim.SLDASM` with driving HIP / KNEE / ANKLE angle mates; the snowshoe lies flat in stance and holds its lift-off angle in swing. The ankle plane pair also reads the same at 180° − α, so each frame checks the shoe's real angle and flips the mate if needed. Image frames, **not a native Motion Study**.
- Static renders: `docs/renders/leg_3dof_*.png`.

## 7. Known weaknesses: candidates for the improvement brief
1. **The belt is the bottleneck.** Continuous capability is 15.4 N; a 4.3 kg quadruped can't walk within continuous ratings. Options:
   - a wider belt (9 mm GT2) or a GT3/HTD profile, if the rules allow
   - a two-stage reduction
   - a larger knee pulley (the thigh would get wider)
   - a stance closer to straight
   - a parallelogram push-rod knee instead of the belt
2. **The hip motor exceeds rated torque in walking** (1.87 Nm).
3. **The thigh is heavy and wide** (66 mm deep including ears, ~70 g). The skeleton barely helps (4 %), because the box section's walls are mostly keep-solid (belt guard bands, bearing rings, ears). The 30 % target was missed. Ideas:
   - 2D topology optimisation of the walls (the brief's stretch goal)
   - a narrower channel with an outside idler
   - drop the ears for internal bosses
4. **No physical hard stops** at the knee or ab/ad (firmware only); only the hip has the bridge stop.
5. **Simplified geometry:**
   - The motors are placeholder cylinders; swap in the official CubeMars STEP and re-check fits and cable exits.
   - The kit pulley is a plain cylinder.
   - The belt, idler, clamp, bearings and fasteners are not modelled.
   - The knee pulley grooves are approximate, not a true GT2 profile.
6. **Print issues:**
   - **Fixed after the hand-in:** the flanges are separate L-brackets printed flat (round holes, no reaming); the outboard plate became a support-free C-frame; the walls stand on L-feet screwed down into spine inserts (no drilled holes); the foot is now a snowshoe printed sole-down; the thigh's motor-boss pocket is a through-hole and the knee pulley has a flat face. See `analysis/out/print.md` for the support-free slicer check.
7. **Unconfirmed assumptions** (`ASSUMPTIONS.md`):
   - which motor face rotates
   - boss height (1.0 mm assumed)
   - cable exit (Ø20 centre holes assumed)
   - belt length supplied, idler size (Ø10 assumed), clamp type
   - rotor inertia
   - the organizers' test load (unknown)
   - the robot body mass for the gait study (0.8 kg)
8. **Compactness:**
   - Lateral width is 114 mm (module plates z −24 … 89.5).
   - The ab/ad offset l = 53 mm (the brief suggested ~40). It's larger because the shank is offset outboard of the thigh to get knee range.
   - An in-line shank would cut l to ~40 mm but limits knee range (the analysis gave ~90–110°).
9. **Analysis fidelity:** stress, deflection and swing dynamics are hand estimates. No SolidWorks Simulation, and no measured masses.
10. **Process:** the SHIFT = 300 mm origin workaround makes the part files awkward to edit by hand (feature dimensions are offset by 300).

## 8. File map
```
leg_day/
  CLAUDE.md                 this handover
  params.py / params.json   single source of truth (all dims, fits, limits, skeleton switches)
  ASSUMPTIONS.md            defaults used + open questions + things that fought back
  analysis/leg_model.py     FK/IK, Jacobian, belt coupling, sweeps, force maps → out/report.md + png
  analysis/clearance_check.py  swept collision check → out/clearance.md
  analysis/skeleton_check.py   link stress/deflection/mass → out/skeleton.md
  analysis/gait_check.py    walk/trot feasibility + matplotlib GIF → out/gait.md
  cad/swlib.py              SolidWorks COM helper (slabs, cuts, sphere, axes, save SLDPRT/STEP/STL)
  cad/build_parts.py        all part builders (+ AXES for mates)
  cad/build_assembly.py     hip_module + leg_3dof, mates, interference detection
  cad/verify_mates.py       drives each joint / limit to prove the mates
  cad/walk_animation.py     walk_anim.SLDASM + SolidWorks-rendered walking GIF (on request)
  cad/parts/                SLDPRT, STEP, SLDASM, parts_report*.json, mate_check.txt, interference_home_pose.txt
  cad/stl/                  STLs (assembly pose: re-orient per docs/PRINT_SETTINGS.md)
  docs/DESIGN_RATIONALE.md  pitch document with numbers per judging criterion
  docs/ASSEMBLY.md          mates, build order, cable routing, hard stops
  docs/BOM.md               printed parts, kit parts, fastener table with thread engagement
  docs/PRINT_SETTINGS.md    orientation, layers, infill, time, filament per part
  docs/BUILD_DAY_NOTES.md   CAN IDs, joint↔motor mapping, limits, IK, zeroing, first-power checklist
  docs/renders/             PNG renders + walk GIFs
  reference/                photos of the organizer sheets
  handin.zip                everything above, as handed in
```

## 9. Rules for an agent improving this
- Change dimensions in `params.py`, never in the SolidWorks files. Regenerate with the scripts in §2. Re-run all checks and quote their output.
- Keep `hip_module.SLDASM` valid on its own (it's the 2-DOF fallback).
- Keep the thigh and shank skeleton support-free: through-cuts or pockets opening away from the bed only.
- Keep the belt or the firmware as the weakest link, with every printed part at SF ≥ 2 at actuator peak.
- Close SolidWorks documents before re-saving parts (`CloseAllDocuments`), otherwise saves silently fail. Never kill SolidWorks without asking the user.
