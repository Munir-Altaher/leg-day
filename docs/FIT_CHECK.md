# Fit check before submission (2026-10-23)

Every row comes from the generated CAD and the scripts in `cad/` and `analysis/`, not from hand measurement. Nothing has been printed or test-fitted yet. **Print `coupon_fit_test` first**: it tunes all of the hole fits below for your printer.

## Whole-assembly checks (SolidWorks)
| Check | Result | Source |
|---|---|---|
| Interference, hip module at home pose (after mates) | **0** | `cad/parts/interference_home_pose.txt` |
| Interference, full leg + body + ab/ad motor | **0** | same |
| Mates rebuild without error; hip, knee, ab/ad, ankle move and stop at their limits | **pass** (4 limits hold) | `cad/parts/mate_check.txt` |
| Swept clearance, 0.5 mm gap required (snowshoe at both ankle stops) | hip ±107°, knee −30…152°, ab/ad −46…+87° → **pass** (limits ±100°, 15–145°, ±45°) | `analysis/out/clearance.md` |
| Belt to channel wall | ≥ 1.75 mm (≥ 1 mm needed) → **pass** | same |

## Motors (AK40-10: Ø53 × 37)
| Check | Value | Verdict |
|---|---|---|
| Knee motor: inboard plate face → adapter | z −68.0 → −31.0 = 37.0 mm = motor length | fits exactly; rotor bolts to adapter, stator to plate |
| Hip motor: thigh outer wall → outboard plate | z −5.5 → 31.5 = 37.0 mm | fits exactly |
| Adapter (rotates) → thigh cover | 0.5 mm gap | OK |
| Ø15 rotor locating boss → printed pocket | Ø15.2 (+0.2) | OK; through-hole in the thigh |
| Motor body inside the C-frame and walls | 0 interferences at home pose; swept check passes | OK |
| Rotor screws M2.5, Ø27 pattern, 3 mm-deep holes | 2.5 mm engagement (M2.5 × 8 through 5 mm wall + 0.5 washer; × 6 in the adapter counterbores; × 8 through the 5.5 mm flange) | OK: never bottoms out |
| Stator screws M2.5, Ø45 pattern, 5 mm-deep holes | 2.0–3.0 mm engagement (× 8 through 5–6 mm; × 12 through the 4 mm tab + 5 mm plate) | OK |

## Bearings, axles, holes
| Feature | Size | Verdict |
|---|---|---|
| 625ZZ seats (Ø16 OD) | Ø16.10, 0.5 mm lip | OK (+0.10 slip fit; the coupon offers +0.05 to +0.20) |
| M5 axles (hip shaft, knee, ab/ad stub) | Ø5.2 printed bores; steel M5 shanks in 625 inner races | OK |
| Clearance holes | M2.5 Ø2.9, M3 Ø3.4, M5 Ø5.5 | OK (ISO medium) |
| Self-tapping holes (thigh cover screws) | M3 Ø2.6 | OK for PLA |
| Hex pockets | M5 nut / head 8.2 across flats, M3 5.7 | OK |

## Heat-set inserts (16 × M3, 5.7 mm long)
| Check | Value | Verdict |
|---|---|---|
| Hole | Ø4.0 × 6.7 deep, 3 mm boss wall | OK: standard for M3 × 5.7 inserts, 1 mm spare for melt |
| Spine (8 mm) | 1.3 mm floor under the insert | OK |
| Module plate pads (7 mm) | **0.3 mm floor** | Works, but press slowly. The floor may bulge or open on the far face; that's cosmetic only, nothing mounts there |
| All inserts go in from an open face, vertical in print | yes | OK |
| M3 × 12 screws into inserts | 6 mm feet + washer → 5.5 mm thread in the insert, shorter than the 6.7 mm hole | OK: never bottoms out |

## Snowshoe ankle
| Check | Value | Verdict |
|---|---|---|
| Shank plug (10 mm thick) between the cheeks | 10.4 mm gap, 0.2 mm per side | OK |
| Ankle bolt M3 × 25 | cheek 4 + gap 10.4 + cheek 4 = 18.4, + wave washer 0.5 + nylock 4.0 = 22.9 → 2.1 mm protrudes | OK: full nylock engagement |
| Lips vs the shank's tip arc | lips sit 9–15 mm above the pivot; the tip arc never reaches that zone | OK (0 interference, swept at both stops) |
| Plug end vs the honeycomb / keels | plug end sweeps r 7.8 mm; ribs top out 11.2 mm below the pivot, centre keel stops 9 mm from it | OK |

## Printing
All 15 parts slice in PrusaSlicer 2.9.6 with supports off and no warnings, and each fits a 220 × 220 × 250 bed (`analysis/out/print.md`). Total 33.5 h, 323 g.

## Things only a test print can confirm
- Hole sizes on your printer: use the coupon and change `FIT_*` / `CLR` in `params.py` if needed, then regenerate.
- The motors' real rotor boss height and screw depths. These are from the datasheet and are marked ASSUMED in `params.py`.
- The ankle friction setting (see `docs/ASSEMBLY.md` step 13).
