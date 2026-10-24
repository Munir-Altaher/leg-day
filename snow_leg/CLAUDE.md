# SNOW LEG — handover (Arctic search-and-rescue variant of the Leg Day leg)

This is the factual handover for `snow_leg/`, in the same style as the root `../CLAUDE.md`. The brief is `C:\Users\<user>\Downloads\SNOW_LEG_PROMPT.md`. Numbers come from the generated outputs in `analysis/out/` and `cad/parts/`; **if they disagree with this text, the files win.** The original Leg Day leg in `../` is untouched: SHA-256 before and after are in `docs/verification/`.

## 1. What it is
- The Leg Day 3-DOF leg (ab/ad + hip + knee, three AK40-10 at the hip, GT2 belt to the knee) adapted for snow:
  1. **Snowshoe foot:** 120 × 80 mm teardrop, 76.5 cm², on a **passive pitch ankle** with stops and centring springs.
  2. **Snow-animal rules** applied to the foot, links, stance and gait (`docs/SNOW_RATIONALE.md`).
  3. **Heat-set insert assembly**, M3 or M5 chosen by calculation.
  4. **Snow and cold measures:** sealed belt channel, closed shank, hip-pod shroud, physical hard stops on every joint.
- **Built the same way as Leg Day:** Python drives SolidWorks 2025 through COM (`cad/swlib.py`, 2.5D slabs + blind cuts + one revolve), and every file is prefixed `snow_`. Nothing in SolidWorks was modelled by hand.

## 2. Regenerate
- `python regenerate.py` runs the whole chain in dependency order (`analysis/out/regenerate_log.txt`):
  params → leg_model → snow_model → build_parts → clearance_check → build_parts body_spine → build_assembly → verify_mates → print_check → insert_check → skeleton_check → gait_check → walk_animation.
  - clearance_check re-solves the ab/ad stop blocks, which is why the spine is rebuilt after it.
  - print_check writes the print-oriented STLs and runs PrusaSlicer.
- `python regenerate.py --analysis` runs without SolidWorks.
- Extras:
  - `python cad/renders.py` makes the renders and `compare_legs.SLDASM`.
  - `python analysis/compare.py` writes `COMPARISON.md`.
  - `python analysis/print_settings_doc.py` writes `docs/PRINT_SETTINGS.md`.
- **Switches** (`params.py`):
  - `FOOT` (env `SNOW_FOOT`): "snowshoe" | "ball". Ball mode builds `snow_shank_ball`, `snow_foot_ball` and `snow_*_ball.SLDASM`.
  - `SNOW_MODE` (env `SNOW_MODE`).
  - `THIGH_HOLES = False`, `SHANK_CLOSED = True`, `FOOT_WINGS = False` (stretch goal, not built).
- **API workarounds kept:** +300 mm `SHIFT`; `AddComponent5` places the bounding-box centre. One more was added: sub-assemblies are re-placed by measuring a child part, because the foot's sketches enlarge the box.
  - Limit-angle mates need the current angle passed in (`cur=`), or SolidWorks snaps the joint to 0.
  - Plane-angle mates are unsigned, so use pairs that are unambiguous over the range (90° ± q) or check the branch from the component transform.
  - PrusaSlicer booleans are written `--no-support-material`.

## 3. Geometry (body frame as in Leg Day: origin = hip axis ∩ ab/ad axis, X forward, Y up, Z outboard)
- L1 = L2 = 130 mm, with L2 measured knee → **ankle pivot**. The pivot is 17 mm above the sole (`ANKLE_H`); the ground force passes through it.
- **Foot (foot frame, origin at the pivot):**
  - heel at −73.5 mm, toe at +46.5 mm; pivot 12 mm (10 % of the length) ahead of the planform centroid
  - deck 2 mm, 5 transverse lugs + rim 5 mm tall, 1.6 wide
  - centre pad flush with the lugs (`PAD_PROUD = 0`; 6 and 12 mm were evaluated)
  - 224 papillae Ø1 × 1.2; 45° toe bevel; 4 M3 set-screw ice spikes
- **Ankle range:** α = −10.6…+47.5°. That's the stance shank angle 4.4…32.5° in the snow stance, ±15°. α = shank angle in the foot frame, so sole level ⇔ α = q1 + q2. Stops are M3 cross-bolts with Ø5 sleeves at r 20 mm; the centring springs' neutral is 18.5°.
- **Snow stance:** pivot 239 mm below the hip (0.92·(L1+L2)), knee about 46° bent at mid-stance.
- **Knee firmware 15–137°.** Past about 140° the 80 mm-wide foot reaches the hip pod. Steel stop posts (M5 + Ø12 sleeve) on the shank hit the thigh rim at 13° and 139°.
- **Module:** inboard plate + two printed-flat flange L-brackets + a C-frame (outboard plate + bridge column + inboard tab plate + stop lug).
  - The C-frame tab is clamped by the 5 knee-motor back screws, replacing Leg Day's 2-screw bridge joint, which couldn't carry lateral moments.
  - Hip shroud with foam liner.
- **Body:** back and front walls with L-feet on the spine, which has ab/ad stop blocks (±47.5°) and a back block widened to ±30 mm for the rig screws.

## 4. Results (see `analysis/out/*.md`)
<!-- RESULTS -->

## 5. Weaknesses / next steps
<!-- WEAKNESSES -->

## 6. File map
```
snow_leg/
  CLAUDE.md                this handover
  params.py / params.json  single source of truth (+ SNOW, INSERTS, SNOW_MODE blocks + switches)
  snowgeo.py               shared foot / ankle / knee-stop geometry (used by CAD and analysis)
  regenerate.py            the whole chain
  COMPARISON.md            original vs snow (generated)
  compare_legs.SLDASM      both legs side by side (original referenced read-only from ../cad/parts)
  analysis/                leg_model, snow_model, insert_check, gait_check (SNOW_MODE), clearance_check,
                           skeleton_check (strength / stiffness / mass), print_check (STL orient + overhangs +
                           PrusaSlicer), mass_model, compare, print_settings_doc;  out/ = reports + plots
  cad/                     swlib, build_parts, build_assembly, verify_mates, walk_animation, renders
  cad/parts/               snow_*.SLDPRT / .STEP / .SLDASM + reports
  cad/stl/                 print-oriented snow_*.stl;  cad/stl/assembly_pose/ = SolidWorks exports
  docs/                    SNOW_RATIONALE, INSERTS, SNOW_TEST_PLAN, ASSEMBLY, BOM, PRINT_SETTINGS,
                           BUILD_DAY_NOTES, renders/, verification/ (SHA-256 lists, sources checked, G-code)
  snow_leg.zip             everything above
```
