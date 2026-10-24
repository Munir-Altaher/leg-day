"""Writes docs/PRINT_SETTINGS.md and fills the gram column of docs/BOM.md from analysis/out/print.json."""
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")

BED_TXT = {"+z": "outboard face (+z) down", "-z": "inboard face (−z) down", "+y": "+y face down",
           "-y": "−y face down", "+x": "front face (+x) down", "-x": "back face (−x) down"}
NOTES = {
    "thigh_outer": "channel side up",
    "thigh_cover": "outer (inboard) face down",
    "knee_pulley_60T": "teeth vertical, flat face down, 0.12 mm layers",
    "shank": "web pocket and hex pocket open upward",
    "snowshoe": "sole down (closed skin on the bed), Voronoi ribs and cheeks rise from it; lips bridge 10.4 mm",
    "hip_shaft_adapter": "motor face down",
    "module_plate_inboard": "insert zones up",
    "module_flange_back": "flat: the Ø15 locating hole and the bolt holes print round",
    "module_flange_front": "flat: stub-axle hex pocket up",
    "module_frame_outboard": "lying on the column's top face; both plates stand; teardrop cable holes",
    "body_bracket_back": "wall face down: bores vertical, foot rises",
    "body_bracket_front": "wall face down: bores vertical, foot rises",
    "body_spine": "back block and inserts up; rig inserts from the bed face",
    "rig_plate": "counterbores on the bed face",
    "coupon_fit_test": "print first",
}


def main():
    d = json.load(open(os.path.join(HERE, "out", "print.json")))
    L = ["# Print settings (PLA, 0.4 mm nozzle, 0.45 mm lines, 220 × 220 × 250 bed)\n",
         "Print-oriented STLs are in `cad/stl/print/` (`analysis/print_check.py` rotates the SolidWorks exports in `cad/stl/`). "
         "**No supports on any part.** Every part was sliced with supports off in PrusaSlicer 2.9.6. Its support-spot check "
         "result and the bridge / overhang check are in `analysis/out/print.md`. Times and grams below are the slicer's.\n",
         "| part | orientation on bed | layer | perimeters | infill | top/bottom | time | g | notes |",
         "|---|---|---|---|---|---|---|---|---|"]
    tm = tg = 0.0
    for n, r in d.items():
        lay, per, fill, pat, tb = r["profile"]
        L.append(f"| {n} | {BED_TXT[r['bed']]} | {lay} | {per} | {fill}% {pat} | {tb} | {r['time']} | {r['grams']} | {NOTES.get(n, '')} |")
        if n != "coupon_fit_test":
            tm += r["minutes"]
            tg += r["grams"] or 0
    L.append(f"\n**Total excluding the coupon: {tm/60:.1f} h, {tg:.0f} g.**\n")
    L.append("General: add a 0.4 mm elephant's-foot compensation (or 0.4 mm chamfer) on the bed face. Never fillet a hole edge. "
             "Bearing seats are Ø16.10 (625 + 0.10); hole sizes come from the coupon, so regenerate if the coupon says otherwise. "
             "PLA softens around 55 °C: keep printed parts off the motor cans (0.5 mm air gap everywhere except the bolted rotor faces).")
    open(os.path.join(ROOT, "docs", "PRINT_SETTINGS.md"), "w", encoding="utf-8").write("\n".join(L))
    bom = os.path.join(ROOT, "docs", "BOM.md")
    s = open(bom, encoding="utf-8").read()
    for n, r in d.items():
        s = s.replace("{" + n + "}", f"{r['grams']:.1f}" if r["grams"] else "–")
    open(bom, "w", encoding="utf-8").write(s)
    left = re.findall(r"\{[a-z_0-9]+\}", s)
    print("PRINT_SETTINGS.md written; BOM placeholders left:", left)


if __name__ == "__main__":
    main()
