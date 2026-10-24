## Installing the heat-set inserts (brief 6.6)

1. **Print the coupon first** (`snow_coupon_fit_test`, flat, 100 % infill).
   - Install inserts in the three hole sizes:

     | Size | Hole sizes on the coupon (mm) |
     |---|---|
     | M3 | 4.0 / 4.1 / 4.2 |
     | M5 | 6.4 / 6.5 / 6.6 |

   - Pick the hole where the insert goes in with light pressure, ends flush to 0.2 mm below the surface, and doesn't push a melt ring above it.
   - Put that diameter into `params.INSERTS[...]["hole"]` (the models already include +0.1 mm print compensation) and regenerate.
2. **Iron:** a heat-set tip matching the insert, at print temperature + 10 °C, which is 220 °C for this PLA (CNC Kitchen used 210 °C).
3. Press slowly with minimal force and let the tip do the work. Stop flush to 0.2 mm below the surface. Remove the iron and hold the insert square with a flat tool for 5 s while it sets.
4. **Orientation:** every insert hole is vertical in its print orientation (so it's round), and every insert is installed from a face you can reach with the iron after printing:

   | Part | Inserts | Installed from |
   |---|---|---|
   | `snow_thigh_outer` | 4 × M3 cover screws | channel side (top when printed) |
   | `snow_module_plate_inboard` | 4 × M3 flange feet | motor-side face, on the thickened zones |
   | `snow_body_spine` | 2 × M5 back wall (top face); 4 × M5 rig plate (bottom face); 4 × M3 front wall (top face) | as listed |
   | `snow_foot_snowshoe` | 4 × M3 ankle bracket + 4 × M3 ice spikes | top face (the print bed face) |
   | `snow_ankle_bracket_out` | 2 × M3 stop bolts | inner face |
   | `snow_shank` | 2 × M5 knee-stop posts (through) + 1 × M3 spring post | inboard face (the print bed face) |
5. **Torque.**
   - Find the spin-out (or crack) torque on the coupon, both warm and after a night in the freezer.
   - Assemble at **≤ 50 %** of the lower value. As a starting point (CNC Kitchen: the M3 screw head sheared at 3–4 Nm before any good insert spun), use 0.6 Nm for M3 and 1.5 Nm for M5 until the coupon gives the real number.
   - Always put a washer under screw heads that bear on PLA.
6. **Cold** (brief 6.5): PLA contracts about 3 × more than brass. From 20 to −40 °C the mismatch strain is ≈ 2.9 × 10⁻³, up to ≈ 10 MPa of hoop stress if fully constrained.
   - Every boss has ≥ 3 mm of wall and its insert axis ≥ one insert OD from any free edge.
   - No sharp internal corners next to an insert.
   - Check for cracks after the freezer test, and re-torque after the first cold cycle.

## Screw lengths

Engagement in the insert is 1.5–2 × d, and the screw never reaches the bottom of the hole (hole depth = insert length + 2 pitches).

| Joint | Screw | Engagement in the insert |
|---|---|---|
| thigh cover → thigh outer | M3 × 8 button head + washer (2.7 mm under the head in the counterbore) | 5.3 mm (1.8 d) |
| flange feet → inboard plate | M3 × 10 (6 mm foot) | 4.0 mm (1.3 d): use M3 × 12 → 5.7 mm (1.9 d) |
| front wall foot → spine | M3 × 12 (6 mm foot) | 5.7 mm (1.9 d) |
| back wall foot → spine | M5 × 16 (6 mm foot) | 9.5 mm (1.9 d) |
| rig plate → spine | M5 × 12 (2.8 mm under the head in the 5.2 mm counterbore) | 9.2 mm (1.8 d) |
| ankle bracket flange → deck | M3 × 8 + washer (2.5 mm flange) | 5.0 mm (1.7 d) |
| ankle stop bolt (through the inner cheek, its boss, the Ø5 × 11 sleeve) → outer cheek | M3 × 25 | 5.7 mm (full insert) |
| ice spike | M3 × 8 set screw, sunk 2 mm below the top face | 5.7 mm, tip 3 mm past the lugs |
| knee-stop post | M5 × 16, Ø12 × 8 sleeve | 8 mm (1.6 d) |
| spring post (shank) | M3 × 16 + Ø6 × 6 spacer | 5.7 mm |
