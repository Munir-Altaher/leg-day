# Sources checked on the web (snow leg)

The brief asks to check its sources where web access exists and never to attribute invented values. This is what was actually checked, on 23 Oct 2026. Everything else is used exactly as the brief quotes it, and `SNOW_RATIONALE.md` marks it "(as quoted in the brief)".

| Source | What was checked | Result |
|---|---|---|
| Lines, Elliott & Ray, "Rigid Wheel Design and Evaluation for Lightweight Snow Rover", *Proc. ISTVS 20th Int. Conf. and 9th Americas Conf.*, 27–29 Sept 2021 (full text via par.nsf.gov) | Table 1, Bekker (n, k_c, k_φ) | **Matches exactly:** 1.6/4.37/196.72; 1.6/2.49/245.90; 1.44/10.55/66.08 (Wong 2001) |
| same | Table 2, Preston-Thomas (k, z_m) | **Matches exactly:** 59/0.06 (Summit Station, Lever 2007); 65/0.29; 56.4/0.35; 27.3/0.50; 19.2/0.54 (Lebel-sur-Quévillon). The paper gives k in kPa/m, while the log term is dimensionless, so the model treats k as kPa, as the brief says to |
| same | Snow shear | **Matches:** cohesion ≈ 0.6–6 kPa; friction angle 9° (Haehnel & Shoop 2004) to 23° (Wong 2001) |
| same | Field references | **Matches:** snowmobile track ≈ 3.5 kPa; boot ≈ 60 kPa sank about 3–4 cm |
| Murray & Boutin 1991, *Oecologia* 88:463–469 (abstract) | Foot load ratio | **Matches:** coyote foot load ≈ 4.1–8.8 × lynx; coyotes sank deeper, but by less than the foot-load ratio. The absolute g/cm² and the 9.5 / 7.7 cm sinkage figures were **not** re-checked (full text not read) |
| CNC Kitchen, "Threaded Inserts for 3D Prints – Cheap vs Expensive" | M3 insert pull-out and torque | **Matches:** Ruthex 181 kg, cheap insert 39 kg (≈ ¼), screw head shears at 3–4 Nm before any insert spins out |
| CubeMars AK40-10 KV170 (OpenELAB and RobotShop listings) | Operating ambient | **Matches:** −20 to 50 °C. The V3.0 listing gives 190 g and Ø53 × 40.2 mm (this design uses the 37 mm sheet version); NTC sensor on board |

Not checked: Renard et al. 2008, Kelsall 1969, Telfer & Kelsall 1984, Nieminen 1990, Crête & Larivière 2003, Godon et al. 2024, Orndorf et al. 2022, Jones et al. 1984, Legg & Mahanty 1986, Karimi et al. 2023, Yan et al. 2025, AMC, JLC3DP, insertguide.com, Prusa. Their numbers are used as quoted in the brief.
