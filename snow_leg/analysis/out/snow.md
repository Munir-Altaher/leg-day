# Snow model (analysis/snow_model.py)

Robot 4.5 kg (ASSUMED). Loads per foot: static (4 down) 11.0 N, walk (3 down) 14.7 N, trot (2 down) 22.1 N, trot peak 33.1 N.

Parameters: Lines, Elliott & Ray (2021) compiling Wong (2001); plate models for homogeneous snow — calibrate on real snow (docs/SNOW_TEST_PLAN.md) before trusting absolute numbers.

## Sanity anchors (brief 4.3)

| anchor | model | brief | |
|---|---|---|---|
| ball R10 trot pressure | 70.26 kPa | 69–71 | PASS |
| ball R10 trot foot load | 716.20 g/cm² | 710–722 | PASS |
| ball trot sinkage Bekker min | 14.61 cm | 14.5–15.5 | PASS |
| ball trot sinkage Bekker max | 29.52 cm | 29.5–30.5 | PASS |
| ball trot sinkage Preston min (P2-P5) | 19.16 cm | 18.5–19.5 | PASS |
| ball trot sinkage Preston max | 52.61 cm | 52.5–53.5 | PASS |
| snowshoe trot pressure | 2.93 kPa | 2.85–2.95 | PASS |
| snowshoe trot foot load | 29.84 g/cm² | 29.5–30.1 | PASS |
| snowshoe trot Bekker min | 5.36 cm | 5.35–5.45 | PASS |
| snowshoe trot Bekker max | 6.19 cm | 6.15–6.25 | PASS |
| snowshoe trot Preston min | 1.28 cm | 1.25–1.35 | PASS |
| snowshoe trot Preston max | 7.64 cm | 7.55–7.75 | PASS |
| snowshoe static Bekker min | 3.31 cm | 3.25–3.35 | PASS |
| snowshoe static Bekker max | 4.01 cm | 3.95–4.05 | PASS |
| snowshoe static Preston min | 0.65 cm | 0.55–0.65 | PASS |
| snowshoe static Preston max | 3.96 cm | 3.95–4.05 | PASS |
| traction snowshoe, trot, poorly bonded | 8.02 N | 7.9–8.1 | PASS |
| traction ball, trot, poorly bonded | 3.68 N | 3.6–3.8 | PASS |
| traction snowshoe, trot, well bonded | 54.61 N | 54–56 | PASS |
| compaction work per trot step, P5 | 0.82 J | 0.75–0.85 | PASS |

**Anchors: ALL PASS** (20/20)

## Foot sweep (teardrop planform, walk = snow gait load, trot for reference)

| foot L×W | area cm² | perim mm | trot g/cm² | walk sink B1–B3 cm | walk sink P4 / P5 cm | trot sink P5 cm | compaction J (walk, P5) | est. foot g | added hip inertia ×1e-3 kg m² |
|---|---|---|---|---|---|---|---|---|---|
| 100×70 | 54.5 | 274 | 41.3 | 4.8–5.8 | 4.7 / 7.1 | 10.3 | 0.51 | 28 | 1.89 |
| 110×75 | 65.1 | 300 | 34.6 | 4.3–5.2 | 4.0 / 6.0 | 8.7 | 0.43 | 30 | 2.04 |
| 120×80 | 76.5 | 325 | 29.4 | 4.0–4.8 | 3.4 / 5.2 | 7.5 | 0.37 | 32 | 2.19 |
| 130×90 | 93.4 | 357 | 24.1 | 3.7–4.3 | 2.8 / 4.3 | 6.3 | 0.31 | 36 | 2.43 |

**Chosen: 120 × 80 mm, tail radius 34** → area 76.5 cm², trot foot load 29.4 g/cm² (lynx 25.9–31.6), within the 130 × 90 limit. 100 × 70 and 110 × 75 exceed lynx-level loading in a trot; 130 × 90 sinks ~1 cm less but adds mass, inertia and clash risk between front and hind feet.

## Stepped 'hoof' pad (P2): PAD_PROUD 0 / 6 / 12 mm, walk load

| PAD_PROUD | B1 pad/deck cm | B3 pad/deck cm | P4 Lebel pad/deck cm | P5 Lebel pad/deck cm | ride height lost on rock |
|---|---|---|---|---|---|
| 0 mm | 4.8 / 4.8 | 4.0 / 4.0 | 3.4 / 3.4 | 5.2 / 5.2 | +0 mm rocker under a free ankle |
| 6 mm | 5.1 / 4.5 | 4.1 / 3.5 | 3.9 / 3.3 | 5.7 / 5.1 | +6 mm rocker under a free ankle |
| 12 mm | 5.6 / 4.4 | 4.5 / 3.3 | 4.4 / 3.2 | 6.2 / 5.0 | +12 mm rocker under a free ankle |

The pad adds sinkage at the pad and gains nothing in soft snow (the deck sinks about the same). On rock or ice a proud pad makes the foot a rocker under a free-pitch ankle and lifts the spikes off the surface (spikes protrude only 3 mm). **PAD_PROUD = 0 chosen**: the pad is flush with the lug tips and acts as a solid 'hoof' zone (insert bosses + bracket load path). Revisit with real-snow plate data.

## Belly clearance vs sinkage, L2 sweep (snow stance, walk load)

Ankle pivot 17 mm above the sole. Stance: pivot at 0.92·(L1+L2) below the hip. Belly = 40 mm below the hip axis (ASSUMED). Target: sinkage ≤ 25 % of belly clearance.

| L2 mm | hip height mm | belly clearance mm | 25 % mm | walk sink P4 / P5 mm | pass | knee q2° | belt N (walk) | hip / knee motor Nm |
|---|---|---|---|---|---|---|---|---|
| 130 | 256 | 216 | 54 | 34 / 52 | PASS | 46 | 54 | 0.93 / 0.34 |
| 145 | 270 | 230 | 58 | 34 / 52 | PASS | 46 | 58 | 0.94 / 0.37 |
| 160 | 284 | 244 | 61 | 34 / 52 | PASS | 46 | 62 | 0.95 / 0.39 |

L2 = 130 mm kept: it already passes the clearance target in P4/P5 with the taller stance, keeps L1 = L2 (no unreachable hole), and a longer shank raises distal mass and knee torque in the fold. The L2 lever is available (one parameter) if real snow is softer than P5.

## Traction (A c + W tan φ) vs need (30 % of vertical)

| foot | case | poorly bonded N | well bonded N | needed N | poorly-bonded verdict |
|---|---|---|---|---|---|
| snowshoe (chosen) | walk (3 down) | 6.9 | 52.1 | 4.4 | PASS |
| snowshoe (chosen) | trot (2 down) | 8.1 | 55.2 | 6.6 | PASS |
| ball R10 | walk (3 down) | 2.5 | 8.1 | 4.4 | SHORT |
| ball R10 | trot (2 down) | 3.7 | 11.3 | 6.6 | SHORT |

Lugs make the shear surface snow-on-snow below the sole, so A is the full planform. On lake/sea ice traction comes from the 4 steel spikes (3 mm proud), not from this model.

## Crust and snow load

- Punch-through resistance scales with perimeter × crust thickness: snowshoe perimeter 325 mm vs ball contact ≈ 63 mm → ≈ 5× the crust load before breaking through (qualitative; test it).
- Snow on the deck: 10 mm of 300 kg/m³ snow on 76 cm² = 23 g (P7) — the top face is flat (it is the print bed face); shake-off relies on the swing and the toe-up tilt.
