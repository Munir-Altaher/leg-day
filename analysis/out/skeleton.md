# Skeleton links: strength, stiffness, mass (analysis/skeleton_check.py)

## Shank (critical section 25 mm below knee, width 24.9 mm, flanges 4.5 mm; the bend bows 9.3 mm off the knee-foot line, axial load x bow is included)

| case | F_inplane N | F_out N | sigma solid MPa | sigma skeleton (Kt=2) MPa | allow MPa | ok |
|---|---|---|---|---|---|---|
| stance | 8.6 | 1.5 | 1.36 | 4.75 | 12.5 | yes |
| push-off | 4.9 | 0.0 | 0.63 | 1.70 | 12.5 | yes |
| lateral | 8.6 | 4.5 | 2.12 | 8.95 | 12.5 | yes |
| knee peak (belt 120 N -> 2.29 Nm) | - | - | 2.21 | 5.98 | 12.5 | yes |

## Thigh (box section at the widest cutout, 116 mm below the hip: depth 51.5 mm, walls 5.0 mm with a 24.1 mm cutout, rim 5.0 mm)

- stance: sigma (Kt=2) 0.38 MPa; hip-motor peak 4.1 Nm stalled against the ground: 1.40 MPa; allow 12.5 MPa -> PASS
- hip-motor flange, 3 x M2.5 on Ø27 at 4.1 Nm: 101 N per screw, PLA bearing 8.1 MPa (allow 12.5; PASS)
- knee adapter at the 0.76 Nm firmware limit: 19 N per M2.5, bearing 2.1 MPa
- ab/ad flange (5.5 mm) at 4.1 Nm: bearing 7.4 MPa (-> ab/ad firmware limit 4.1 Nm)
- Ø5 steel hip shaft at 0.76 Nm: tau = 31 MPa
- Ø5 steel hip shaft at 4.1 Nm: tau = 167 MPa

## Foot deflection at 15 N stance
- thigh 0.018 mm, shank in-plane 0.216 mm, shank out-of-plane (10 % lateral) 0.637 mm -> total 0.87 mm (target < 1 mm) -> PASS
- for reference, gearbox backlash alone (18 arcmin) gives ~1.4 mm of free play at the foot (not elastic deflection)

## Snowshoe (worst case: trot peak 32 N all at the toe, Kt = 2)
- toe cantilever 32 mm at the hub edge (section 68 mm wide: 0.8 skin + Voronoi ribs + rim + 3 keels 9.7 mm tall, I = 1199 mm^4): sigma 5.3 MPa, allow 12.5 -> PASS
- ankle bolt bearing on the two 4 mm cheeks: 1.3 MPa -> PASS
- lip stop (walking load at the toe, held by a lip 12 mm above the pivot): 52 N on a 6 x 10.4 mm face = 0.8 MPa -> PASS

## Mass (printed estimate = solid volume x 1.24 g/cm^3 x 0.75)

| part | solid g | skeleton g | saving |
|---|---|---|---|
| thigh_outer | 46.7 | 43.9 | 6 % |
| thigh_cover | 24.9 | 22.0 | 11 % |
| shank | 27.5 | 18.2 | 34 % |
| **thigh + shank** | 99.2 | 84.2 | **15 %** |

Link inertia about the hip (leg straight, links only): solid 12.81 kg cm^2, skeleton 9.26 kg cm^2 (28 % less). A gram off the shank is worth 8x a gram off the thigh.