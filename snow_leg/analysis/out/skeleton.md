# Strength, stiffness and mass — snow leg (analysis/skeleton_check.py)

Allowable in-plane stress 12.5 MPa (25 MPa / SF 2); E = 2300 MPa; robot 4.5 kg; fall = 3 W = 132 N on one foot + 30 % lateral.

## Closed shank: option (a) one closed part vs (b) clamshell

| option | mass g | I in-plane mm⁴ | I out-of-plane mm⁴ | seams into the shank | chosen |
|---|---|---|---|---|---|
| (a) one part, 3 perimeters + 15 % gyroid | 19.0 (slicer) | 4831 | 953 | none | **yes** |
| (b) clamshell, 1.2 mm skins + ribs, 4 × M3 inserts | ≈ 21.1 (estimate) | 4669 | 1195 | one full-length seam | no |

(a) is about the same mass, stiffer out of plane (its skins sit at the full 10 mm thickness), has no seam for snow to wick into and needs no inserts. Section 30 mm below the knee, infill ignored (conservative).

## Shank stresses (snow stance: shank 23° from vertical)

| case | section | in-plane MPa | out-of-plane MPa | Kt | combined MPa | allow | ok |
|---|---|---|---|---|---|---|---|
| walk + 10 % lat | 30 mm below knee | 1.44 | 0.77 | 1 | 2.21 | 12.5 | yes |
| walk + 10 % lat | upper stop post (48 mm) | 1.33 | 0.66 | 2 | 3.97 | 12.5 | yes |
| trot peak + 30 % lat | 30 mm below knee | 3.24 | 5.21 | 1 | 8.45 | 12.5 | yes |
| trot peak + 30 % lat | upper stop post (48 mm) | 3.00 | 4.43 | 2 | 14.84 | 12.5 | **NO** |
| fall + 30 % lat | 30 mm below knee | 12.98 | 20.84 | 1 | 33.82 | 25.0 | **NO** |
| fall + 30 % lat | upper stop post (48 mm) | 11.99 | 17.70 | 2 | 59.38 | 25.0 | **NO** |
| knee peak (belt 120 N → 2.29 Nm) | 30 mm below knee | 5.73 | – | 1 | 5.73 | 12.5 | yes |

Fall cases are checked against the 25 MPa strength (SF 1 for an accident case); service cases against 12.5 MPa.

## Thigh (sealed box, 3 mm walls, no holes): hip-motor peak 4.1 Nm at the root → 1.94 MPa (Kt 2) vs 12.5 -> PASS

## Snowshoe deck under 2.89 kPa (trot), E = 1955 MPa

- heel cantilever (51 mm from the pad, deck + 2 rims): tip 0.23 mm, σ 5.00 MPa
- toe cantilever (25 mm): tip 0.01 mm, σ 0.96 MPa
- side strip (pad edge → rim, 23 mm, deck + lug T-section per 15 mm): 0.007 mm
- worst rim deflection 0.23 mm (target < 2 mm) -> PASS

## Ankle (fall onto the heel tip, toe-down stop engaged)

- moment about the pivot 9.7 Nm → stop force 487 N at r 20 mm
- M5 pin, double shear on the Ø4.0 core: 6 MPa (steel 8.8: ~380 MPa shear) -> PASS
- cheek bearing at the pin (Ø5 × 4.0 mm, per cheek): 3.5 MPa vs 25 -> PASS
- cheek bearing at the stop bolt (Ø3 × 6.7 mm boss, per cheek): 12.1 MPa vs 25 -> PASS
- Ø5 sleeve on the shank edge (10 mm): 9.7 MPa vs 25 -> PASS
- centring springs (2 × 0.1 N/mm, preload 2.0 N): ≈ 0.89 N·mm/° about the pivot; the foot's own weight torque is ≈ 4.7 N·mm, so the springs hold the swing pitch within ≈ 5° of neutral (18.5°)

## Knee stop posts: fall onto a crouched leg at the upper stop → 333 N on the post; M5 (core Ø4.0) bending at 4 mm: 212 MPa vs 640 MPa yield (8.8) → SF 3.0

## Deflection at the ankle pivot, 14.7 N walk load + 10 % lateral (snow stance)
- thigh 0.009 mm, shank in-plane 0.380 mm, shank out-of-plane 0.492 mm → 0.88 mm (target < 1 mm) -> PASS; gearbox backlash adds ≈ 1.4 mm of free play

## Mass (slicer filament + hardware list, analysis/mass_model.py)

| group | original g | snow g |
|---|---|---|
| foot | 8 | 46 |
| shank | 31 | 46 |
| thigh | 104 | 108 |
| swinging | 143 | 200 |
| hip_module | 606 | 743 |
| whole | 882 | 1081 |

Leg inertia about the hip, leg straight: original 21.8 kg·cm², snow 53.5 kg·cm² (+3.16×10⁻³ kg·m²; the foot group alone adds 3.14×10⁻³).

**Foot group 46.5 g (printed 28.9 g + hardware 17.6 g) vs the 35 g budget -> OVER BUDGET**

**Strength / stiffness checks: SOME FAIL**