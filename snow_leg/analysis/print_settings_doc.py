"""Writes docs/PRINT_SETTINGS.md from analysis/out/print.json (analysis/print_check.py)."""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")

BED_TXT = {"+z": "outboard face (+z) down", "-z": "inboard face (−z) down", "+y": "top face (+y) down",
           "-y": "bottom face down", "+x": "front face (+x) down", "-x": "back face (−x) down"}
NOTES = {
    "thigh_outer": "channel side up; inserts from the channel side",
    "thigh_cover": "lip up",
    "knee_pulley_60T": "teeth vertical; flat face down; 0.12 mm layers",
    "shank": "option (a): closed part, infill is the skeleton",
    "foot_snowshoe": "printed INVERTED: smooth top face on the bed, every sole feature rises",
    "ankle_bracket_in": "flange rises from the cheek; pin / stop holes vertical",
    "ankle_bracket_out": "inserts from the bed face",
    "module_frame_outboard": "lying on the column's top face; both plates stand; teardrop cable holes",
    "module_flange_back": "locating hole and bolt holes vertical (fixes Leg Day's reaming)",
    "module_flange_front": "stub-axle hex pocket up",
    "hip_shroud_out": "cap on the bed, wall rises; foam liner glued in after printing",
    "body_bracket_back": "bores vertical; foot rises",
    "body_bracket_front": "bores vertical; foot rises (wall as wide as the foot)",
    "body_spine": "stop blocks + back block rise; rig inserts from the bed face",
    "coupon_fit_test": "print first",
}


def main():
    d = json.load(open(os.path.join(HERE, "out", "print.json")))["snow"]
    L = ["# Print settings — snow leg (PLA prototype)\n",
         "Print-oriented STLs are in `cad/stl/` (`analysis/print_check.py` rotates the SolidWorks exports, kept in "
         "`cad/stl/assembly_pose/`). **No supports anywhere.** Every part was sliced with supports off in PrusaSlicer "
         "2.9.6 (0.4 nozzle, 0.45 mm lines, 220 × 220 × 250 bed, PLA 210/60 °C); its support-spot check raised no stability "
         "warning on any snow part, and the geometric check finds no bridge over 10 mm (details in `analysis/out/print.md`). "
         "Times and grams below are the slicer's.\n",
         "| part | orientation | layer | perimeters | infill | top/bottom | time | g | notes |",
         "|---|---|---|---|---|---|---|---|---|"]
    tm = tg = 0.0
    for n, r in d.items():
        lay, per, fill, pat, tb = r["profile"]
        L.append(f"| snow_{n} | {BED_TXT[r['bed']]} | {lay} | {per} | {fill}% {pat} | {tb} | {r['time']} | {r['grams']} | {NOTES.get(n, '')} |")
        if n != "coupon_fit_test":
            tm += r["minutes"]
            tg += r["grams"] or 0
    L.append(f"\n**Total (excluding the coupon): {tm/60:.1f} h, {tg:.0f} g.** Small references (Ø5 × 11 stop sleeves, Ø12 × 8 "
             "knee-stop sleeves) print in minutes at 100 %.\n")
    L.append("General: 0.4 mm elephant's-foot compensation on bed faces; never fillet a hole edge; washers under screw heads on "
             "PLA; for a deployable leg reprint in PETG / ASA (or PA-CF) with a TPU sole (see `SNOW_RATIONALE.md`).")
    open(os.path.join(ROOT, "docs", "PRINT_SETTINGS.md"), "w", encoding="utf-8").write("\n".join(L))


if __name__ == "__main__":
    main()
