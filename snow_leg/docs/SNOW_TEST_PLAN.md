# Snow test plan: calibrate the model, then test the foot

The sinkage and traction numbers in `analysis/out/snow.md` come from **plate models for homogeneous snow** with published parameters (Lines, Elliott & Ray 2021, compiling Wong 2001). Real Arctic snow is layered, wind-packed and crusted. Run these tests before trusting any absolute number, then put the fitted parameters into `analysis/snow_model.py` and regenerate.

## 1. Plate-sinkage tests (bevameter method)

The aim is to fit Bekker (n, k_c, k_φ) and Preston-Thomas (k, z_m) for the local snow.

**Equipment**
- Two printed or plywood plates of different width b, same shape as the foot: 60 × 40 mm and 120 × 80 mm (the real foot outline), each with a central post.
- A stack of known weights: 0.5 kg steps up to 6 kg, which covers 0.6–9 kPa on the large plate and 2–25 kPa on the small one.
- A depth gauge on a bridge that rests on undisturbed snow ≥ 0.5 m away (a ruler on a straight bar is enough). A thermometer, plus a snow-density cutter or 250 ml cup and a kitchen scale.

**Procedure** (per snow site: fresh powder, wind-packed, trail, crust)
1. Record air and snow temperature, snow depth (probe), and density (cut a known volume and weigh it).
2. Place the plate, zero the gauge, and add weights one step at a time. Read the sinkage after 10 s at each step.
3. Repeat 3× per plate per site, each time on fresh, undisturbed snow 0.5 m apart.
4. Fit with least squares (≥ 8 load points per curve):
   - **Bekker:** p = (k_c / b + k_φ) zⁿ, using both plate widths together to separate k_c from k_φ (Yan et al. 2025 describe the inversion).
   - **Preston-Thomas:** p = k[−ln(1 − z/z_m)].

   Add each site as a new set in `BEKKER` / `PRESTON` in `snow_model.py`.
5. Also note the crust: the load that breaks through and the crust thickness, for both plates. The punch-through load should scale with the perimeter (325 mm for the foot vs 63 mm for the ball).

## 2. Prototype foot tests (printed `snow_foot_snowshoe` + ankle bracket + shank stub)

| Test | Set-up | Measure | Pass / compare |
|---|---|---|---|
| **Drop test** | Foot + ankle + 1.1 kg (one quarter of the robot) dropped from 50 mm and 100 mm onto snow and onto ice | Sinkage, whether the sole stays level, anything cracking or inserts pulling | Sinkage within ±30 % of `snow.md` for the fitted parameters; no damage at 100 mm |
| **Extraction (pull-out)** | Load the foot to 14.7 N (walk) for 5 s, then lift vertically with a spring scale. Repeat with a lift-off toe-up tilt (the ankle free) | Peak pull force, compaction work (area under the force × displacement curve) | Pull with the free ankle < pull with the ankle locked level (the snowshoe-binding effect) |
| **Traction** | Foot loaded to 22 N (trot), dragged horizontally with a spring scale on: poorly bonded snow, well-bonded snow, glare ice (spikes in and out) | Peak horizontal force | ≥ 30 % of vertical (6.6 N); compare with `snow.md` (8.1 N poorly bonded, ≈ 55 N well bonded); on ice, spikes must hold ≥ 6.6 N |
| **Snow shedding** | Lift the foot out of wet snow 20 times | Mass of snow stuck to the foot (weigh) | < 10 g (the deck is flat on top: see P7 in `SNOW_RATIONALE.md`) |
| **Ankle** | Hand-cycle the ankle through its range at −20 °C | Stops hit cleanly, springs return the foot to neutral, no binding | — |

## 3. Freezer tests (coupon + parts)

1. **Insert coupon** (`snow_coupon_fit_test`).
   - Install inserts at the three hole diameters (M3 4.0 / 4.1 / 4.2; M5 6.4 / 6.5 / 6.6) in bosses with 2 / 3 / 4 mm walls.
   - Night 1 at room temperature: for each, measure the torque to spin out (torque screwdriver) and the pull-out (spring scale or hanging weights, up to 60 kg).
   - Night 2: same tests straight out of a −18 °C freezer, within 60 s.
   - Look for cracks from the PLA/brass contraction mismatch (≈ 2.9 × 10⁻³ strain from 20 to −40 °C).
   - Update `INSERTS[...]["hole"]` and `["cap"]` in `params.py`.
2. **Printed-part impact** after a night at −18 °C: drop a spare shank and a foot from 1 m onto concrete. PLA's cold impact behaviour wasn't measured by Karimi et al. 2023, so this is the test that decides whether PLA is acceptable for the field or only for the prototype.
3. **Belt and grease**: the GT2 belt and bearings after 2 h at −18 °C.
   - Bend the belt around the 20T pulley by hand and check for surface cracks.
   - Spin the bearings and compare the drag with room temperature.
4. **Motors**: power each AK40-10 at −18 °C. Log the NTC temperature and current at zero load for 5 min, then run the preheat routine in `BUILD_DAY_NOTES.md` and record the time to reach the rated −20 °C ambient margin.

## 4. Reporting

Put the raw data in `docs/verification/snow_tests/` (CSV per test), the fitted parameters in `snow_model.py`, and regenerate (`python regenerate.py --analysis`). Then compare predicted vs measured sinkage in a table in `SNOW_RATIONALE.md`.
