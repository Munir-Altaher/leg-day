# Bill of materials — snow leg

Printed masses are PrusaSlicer filament grams (`analysis/out/print.md`, profiles in `PRINT_SETTINGS.md`). Hardware masses (used in `analysis/mass_model.py`) are typical catalogue values, **ASSUMED** until weighed.

## Printed (PLA prototype; field material in `SNOW_RATIONALE.md` § cold)

| Part | File | Qty | Notes |
|---|---|---|---|
| Thigh, outboard half (sealed: no truss holes, no ears) | `snow_thigh_outer` | 1 | 4 × M3 inserts; motor-boss hole through (closed by the rotor) |
| Thigh cover + seam lip | `snow_thigh_cover` | 1 | 0.6 mm labyrinth lip into the rim |
| Knee pulley 60T | `snow_knee_pulley_60T` | 1 | flat bed face; 0.5 mm shim washer replaces the second hub spacer |
| Shank, closed (option a) | `snow_shank` | 1 | 3 perimeters + 15 % gyroid; 2 × M5 + 1 × M3 inserts; ankle bushing Ø5.3 |
| Snowshoe foot 120 × 80 | `snow_foot_snowshoe` | 1 | lugs, rim, pad, 224 papillae, toe bevel; 8 × M3 inserts |
| Ankle bracket, inner / outer cheek | `snow_ankle_bracket_in` / `_out` | 1 + 1 | outer cheek: 2 × M3 inserts for the stop bolts |
| Knee-motor pulley adapter | `snow_hip_shaft_adapter` | 1 | 100 % infill |
| Module inboard plate | `snow_module_plate_inboard` | 1 | 4 × M3 inserts for the flange feet |
| Module flange, back (ab/ad output) / front (stub axle) | `snow_module_flange_back` / `_front` | 1 + 1 | printed flat: locating pocket and bores vertical |
| Module C-frame (outboard plate + bridge column + inboard tab + stop lug) | `snow_module_frame_outboard` | 1 | clamped by the 5 knee-motor screws |
| Hip-pod shroud | `snow_hip_shroud_out` | 1 | + 5 mm closed-cell foam liner, cable gland |
| Body bracket, back wall / front wall | `snow_body_bracket_back` / `_front` | 1 + 1 | L-feet onto the spine |
| Body spine (+ ab/ad stop blocks) | `snow_body_spine` | 1 | 2 × M5 + 4 × M5 + 4 × M3 inserts |
| Rig / body plate | `snow_rig_plate` | 1 | 4 × M5 to the spine |
| Fit + insert coupon | `snow_coupon_fit_test` | 1 | print first |
| Ø5 × 11 stop sleeves | (`snow_ref_stop_sleeves`) | 2 | print at 100 % or use an aluminium spacer |
| Ø12 × 8 knee-stop sleeves | (`snow_ref_knee_post_sleeves`) | 2 | print at 100 % (or a rubber/PU bumper sleeve) |

## Motors, kit and bearings

| Item | Qty | Where |
|---|---|---|
| CubeMars AK40-10 KV170 | 3 | ab/ad, hip, knee (all at the hip) |
| GT2 6 mm belt, 372 mm cut (342 mm loop) | 1 | thigh channel; cold-rated alternative in `SNOW_RATIONALE.md` |
| GT2 20T pulley, 5 mm bore (kit) | 1 | hip shaft |
| Idler (kit) + M3 × 16 + nut + 2 × M3 × 12 mm fender washers | 1 | thigh slot (the fender washers close the slot) |
| **625-2RS** (sealed) 5 × 16 × 5 | 4 | thigh cover hip + knee, thigh outer knee, front wall (stub axle) |
| Extension spring, steel, ≈ 0.1 N/mm, 20–35 mm | 2 | ankle centring pair |
| Closed-cell foam sheet 5 mm | ≈ 150 cm² | hip shroud liner |
| Cable glands PG7 (or printed + silicone) | 2 | hip shroud, body |

## Fasteners (inserts: `docs/INSERTS.md`)

| Item | Qty | Where |
|---|---|---|
| M3 heat-set insert (Ruthex-type 5.7 mm) | 4 + 4 + 8 + 2 + 1 + 4 = 23 | thigh, inboard plate, foot, outer cheek, shank, spine front |
| M5 heat-set insert (9.5 mm) | 2 + 4 + 2 = 8 | spine (back wall, rig), shank (knee-stop posts) |
| M2.5 × 8 | 3 + 3 + 5 + 5 | thigh → hip rotor (+ washer), ab/ad flange → rotor, back wall → ab/ad stator, hip stator → C-frame |
| M2.5 × 12 | 5 + 5 | knee stator through the C-frame tab + inboard plate; hip stator through the shroud + C-frame |
| M2.5 × 6 | 3 | adapter → knee rotor |
| M5 × 16 | 1 + 2 | hip shaft (head in the adapter; **was M5 × 25 in Leg Day, which would hit the hip motor face**), knee-stop posts |
| M5 × 25 | 1 + 1 + 1 | knee axle (head in the shank), ankle pin, ab/ad stub axle |
| M5 nylock | 2 | knee pulley pocket (threadlocker), ankle pin |
| M5 washer 1 mm + 0.5 mm shim | 1 + 1 | shank ↔ knee bearing; knee pulley ↔ outer-wall bearing |
| M5 × 12 | 4 | rig plate → spine |
| M5 × 16 | 2 | back wall foot → spine |
| M3 × 8 button head + washer | 4 | thigh cover |
| M3 × 12 | 4 + 4 | flange feet → plate; front wall foot → spine |
| M3 × 8 + washer | 4 | ankle bracket → foot |
| M3 × 25 | 2 | ankle stop bolts (with Ø5 × 11 sleeves); the spring loops hook under their heads |
| M3 × 8 set screw (ice spike) | 4 | foot |
| M3 × 16 + Ø6 × 6 spacer | 1 | spring post on the shank |
