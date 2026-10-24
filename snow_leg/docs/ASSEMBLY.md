# Assembly — snow leg

## Files and regeneration
- `cad/parts/snow_hip_module.SLDASM` (2-DOF hip module + shank + snowshoe foot; valid on its own) nested in `cad/parts/snow_leg_3dof.SLDASM` (+ ab/ad motor, body bracket, rig plate).
- `cad/parts/snow_walk_anim.SLDASM` holds the same parts with driving mates, for the walking animation.
- `compare_legs.SLDASM` (in `snow_leg/`) holds the original and the snow leg side by side.
- Every file is prefixed `snow_` and references only files inside `snow_leg/` (checked through the API; see `cad/parts/interference_home_pose.txt`). The original leg and the snow leg can be open at the same time.
- Regenerate everything: `python regenerate.py` (SolidWorks), or `python regenerate.py --analysis` (no SolidWorks). Change values only in `params.py`.
- Each part's own origin sits at (−300, −300, −300) mm, the same API workaround as Leg Day. The assemblies place every part in the body frame. The display pose is q1 = −10°, q2 = 20°, with the sole level.

## Mates (generated; drag the thigh, shank or foot)

| Joint | Mates | Limit |
|---|---|---|
| hip | `HIP_AXIS` coincident + Front planes + limit angle (thigh Right vs frame Right) | 0–100° (unsigned → ±100°) |
| knee | `KNEE_AXIS` + Front planes + limit angle (shank Right vs thigh Right, aligned) | 15–137° |
| ankle | `ANKLE_AXIS` (pin) + Front planes + limit angle (foot Right vs shank Top = 90° + α, aligned) | α −10.6…+47.5° |
| ab/ad (in `snow_leg_3dof`) | `ABAD_AXIS` + Right planes + limit angle (frame Top vs spine Top), hip module flexible | 0–45° (unsigned → ±45°) |

- **Fixed:** the module (inboard plate, two flanges, C-frame, shroud, motors) in the hip module; the body bracket, spine, rig plate and ab/ad motor in the 3-DOF leg.
- **Rigid groups (3 base-plane coincidences):**
  - cover → thigh
  - kit pulley → adapter
  - pulley and post sleeves → shank
  - foot, inner cheek, pin, stop sleeves and springs → outer cheek
- **Checked by `cad/verify_mates.py`** (`cad/parts/mate_check.txt`): every joint moves inside its range, every limit holds past it (the ankle in both directions), and there are no mate errors.

## Build order (every insert reachable with the iron, every screw head reachable)

1. **Print the coupon first**, set the insert hole sizes and bearing fits (`docs/INSERTS.md`, `params.py`), and regenerate. Do the freezer test on it (`SNOW_TEST_PLAN.md` §3).
2. **Install every insert** before any assembly. The list and faces are in `docs/INSERTS.md`.
3. **Body bracket.**
   - Bolt the ab/ad motor's back face to `snow_body_bracket_back` (5 × M2.5 × 8).
   - Press a 625-2RS into `snow_body_bracket_front`.
   - Screw the wall feet to the spine: back 2 × M5 × 16, front 4 × M3 × 12. Washers on PLA.
   - Bolt `snow_rig_plate` under the spine (4 × M5 × 12, heads in the counterbores).
4. **Module.**
   - Screw `snow_module_flange_back` and `_front` to `snow_module_plate_inboard` (feet, 2 × M3 × 12 each, from the motor side), **before the knee motor goes in**.
   - Bolt the back flange to the ab/ad output (3 × M2.5 × 8 from inside; the Ø15 boss enters the through-hole).
   - Push the M5 stub axle (head in the front flange's hex pocket) out through the front flange.
5. **Knee motor + C-frame.** Put the C-frame's inboard tab on the inboard plate's inboard face. Fix the knee motor through both with 5 × M2.5 × 12 into its back face.
6. **Hip shaft.**
   - M5 × **16** with its head in the adapter's hex pocket. Leg Day's M5 × 25 would have reached into the hip motor face.
   - Bolt the adapter to the knee-motor output (3 × M2.5 × 6).
7. **Thigh.**
   - Press 625-2RS bearings: cover hip and knee, outer-wall knee (from the channel side).
   - Bolt `snow_thigh_outer` to the hip-motor output (3 × M2.5 × 8 + washer, heads in the channel). The motor's Ø15 boss enters the through-hole and closes it.
8. **Knee.**
   - Fit the M5 × 25 head in the shank's hex pocket.
   - Stack the M5 washer, the outer-wall bearing, the 0.5 mm shim, then `snow_knee_pulley_60T`.
   - Thread the nylock into the pulley pocket with medium threadlocker.
   - Screw in the two knee-stop posts (M5 × 16 + Ø12 sleeves) from the shank's inboard face.
9. **Belt.**
   - Cut 372 mm. Loop it over the 60T, slide the kit 20T onto the hip shaft inside the loop, and clamp the ends. The clamp must stay on a span over the full range.
   - Put the idler (M3 × 16, two **M3 × 12 mm fender washers** outside both walls) in its slot, loose.
10. **Close the thigh.** Offer up `snow_thigh_cover`: its lip drops inside the rim, and the knee bolt end enters the cover bearing. Fit 4 × M3 × 8 button heads + washers into the inserts.
11. **Hang the hip/knee stack in the module.**
    - Bolt the hip motor's back face through the shroud and the C-frame outboard plate (5 × M2.5 × 12).
    - Put the foam liner inside the shroud first.
    - Push the stub axle into the front-wall bearing.
12. **Tension.** Slide the idler until the belt is tight (2–3 mm deflection under a firm finger, ≥ 60 N), then lock it. The fender washers cover the slot at any position.
13. **Ankle and foot.**
    - Screw the two cheeks to the deck (4 × M3 × 8 + washers into the deck inserts).
    - Put the shank tip between the cheeks, push the M5 × 25 pin through (head outboard), and fit the nylock just snug: the shank must turn freely on the bushing.
    - Fit the two stop bolts M3 × 25 through the inner cheek boss and a Ø5 × 11 sleeve into the outer-cheek insert. They set the cheek spacing.
    - Hook the two springs from the shank spring post (M3 × 16 + spacer) to the stop-bolt heads.
    - Fit 4 × M3 × 8 set-screw spikes from the top face, sunk 2 mm (tips 3 mm past the lugs). Remove them for indoor use.

## Sealing and snow inspection (do this before every outing)
- **Belt channel.**
  - No through-holes anywhere in the thigh walls: the motor-boss hole is closed by the motor, the knee and hip bores by 2RS bearings, and the idler slot by fender washers.
  - The cover seam is a 0.6 mm labyrinth lip.
  - Shine a light into the knee gap and look for snow or ice on the belt. Any ice in the belt means skipped teeth: dry it warm, never chip it.
- **Ankle:** knock snow out from between the cheeks, check the foot returns to neutral, and check the spikes are tight.
- **Hip pod:** check the foam liner is dry and the glands are tight.

## Cable routing
- The hip and knee leads leave through the Ø20 cable holes (C-frame plates). The hip motor's leads go through the shroud gland.
- Run along the bridge column, zip-tie at the back flange, leave a 60 mm service loop around the ab/ad axis, and anchor in the spine slots.
- Check at ab/ad ±45° that nothing stretches or pinches.

## Hard stops (all physical)
| Joint | Stop | Angle |
|---|---|---|
| hip | the C-frame column | ±107° |
| knee | two steel M5 posts with Ø12 sleeves | 13° and 139° (firmware 15–137°) |
| ankle | two cross-bolts with Ø5 sleeves | α −10.6° / +47.5° |
| ab/ad | the C-frame column and the bottom lug land on the spine blocks | ±47.5° (firmware ±45°) |
