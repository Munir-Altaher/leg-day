"""Mass model for the snow leg and the original Leg Day leg.

Printed parts: PrusaSlicer filament grams from analysis/out/print.json (analysis/print_check.py).
Hardware: typical catalogue masses (ASSUMED until weighed on build day).
Groups: foot (everything below the ankle pivot that pitches with the foot), swinging leg (thigh +
shank + foot group + knee pulley + belt + bearings in them), hip module (incl. hip/knee motors),
whole leg (incl. ab/ad motor, body bracket, rig plate).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import params as P  # noqa: E402

HW = {  # g each, ASSUMED (catalogue / typical)
    "AK40-10": P.MOTOR_MASS * 1000, "625": 4.5, "M5x25": 4.6, "M5x16": 3.3, "M5x12": 2.8, "M5 nylock": 1.3,
    "M5 washer": 0.5, "M3x8": 0.6, "M3x10": 0.7, "M3x16": 1.0, "M3x20": 1.2, "M3x8 set": 0.35, "M3 button x8": 0.6, "M3 nut": 0.25,
    "M3 washer": 0.15, "M2.5x8": 0.4, "M2.5x12": 0.55, "M3 insert": 0.33, "M5 insert": 1.2, "spring": 0.5,
    "belt": 3.0, "pulley 20T": 5.0, "idler": 3.0, "sleeve 5": 0.3, "sleeve 12": 1.0, "foam": 2.0,
}


def printed(which="snow"):
    path = os.path.join(HERE, "out", "print.json")
    d = json.load(open(path))["snow" if which == "snow" else "original"]
    return {k: (v["grams"] or 0.0) for k, v in d.items()}


def hw(items):
    return sum(HW[k] * n for k, n in items)


def snow_groups():
    pr = printed("snow")
    foot_hw = [("M5x25", 1), ("M5 nylock", 1), ("M3x8", 4), ("M3 washer", 4), ("M3 insert", 4 + P.SPIKE_N + 2),
               ("M3x8 set", P.SPIKE_N), ("M3x20", 2), ("sleeve 5", 2), ("spring", 2)]
    foot = pr["foot_snowshoe"] + pr["ankle_bracket_in"] + pr["ankle_bracket_out"] + hw(foot_hw)
    shank_hw = [("M5 insert", 2), ("M5x16", 2), ("sleeve 12", 2), ("M3 insert", 1), ("M3x16", 1), ("M5x25", 1),
                ("M5 nylock", 1), ("M5 washer", 2)]
    shank = pr["shank"] + pr["knee_pulley_60T"] + hw(shank_hw)
    thigh_hw = [("625", 4), ("M3 insert", 4), ("M3 button x8", 4), ("M3 washer", 4), ("belt", 1), ("idler", 1),
                ("M3x16", 1), ("M3 nut", 1), ("M3 washer", 2)]
    thigh = pr["thigh_outer"] + pr["thigh_cover"] + hw(thigh_hw)
    module_hw = [("AK40-10", 2), ("M2.5x8", 3 + 3), ("M2.5x12", 10), ("M3 insert", 4), ("M3x10", 4), ("pulley 20T", 1),
                 ("M5x16", 1), ("foam", 1)]
    module = (pr["hip_shaft_adapter"] + pr["module_plate_inboard"] + pr["module_flange_back"] + pr["module_flange_front"] +
              pr["module_frame_outboard"] + pr["hip_shroud_out"] + hw(module_hw))
    body_hw = [("AK40-10", 1), ("M2.5x8", 5), ("625", 1), ("M5x25", 1), ("M5 insert", 2 + 4), ("M5x12", 2 + 4),
               ("M3 insert", 4 + 2), ("M3x10", 4 + 2)]
    body = pr["body_bracket_back"] + pr["body_bracket_front"] + pr["body_spine"] + pr["rig_plate"] + hw(body_hw)
    swing = thigh + shank + foot
    return dict(foot=foot, shank=shank, thigh=thigh, swinging=swing, hip_module=module + swing, whole=module + swing + body,
                foot_hw=hw(foot_hw), foot_printed=pr["foot_snowshoe"] + pr["ankle_bracket_in"] + pr["ankle_bracket_out"])


def original_groups():
    pr = printed("original")
    foot = pr["foot"] + hw([("M3x8", 2)])
    shank = pr["shank"] + pr["knee_pulley_60T"] + hw([("M5x25", 1), ("M5 nylock", 1), ("M5 washer", 1)])
    thigh = pr["thigh_outer"] + pr["thigh_cover"] + hw([("625", 3), ("M3x16", 3), ("M3x16", 4), ("M2.5x8", 3), ("belt", 1),
                                                        ("idler", 1), ("M3 nut", 1)])
    module = (pr["hip_shaft_adapter"] + pr["module_frame_inboard"] + pr["module_plate_outboard"] +
              hw([("AK40-10", 2), ("M2.5x8", 3 + 5 + 5 + 3), ("M3x10", 2), ("pulley 20T", 1), ("M5x25", 1)]))
    body = (pr["body_bracket_back"] + pr["body_bracket_front"] + pr["body_spine"] + pr["rig_plate"] +
            hw([("AK40-10", 1), ("M2.5x8", 5), ("625", 1), ("M5x25", 1), ("M3x16", 4), ("M3x10", 4), ("M3 insert", 4)]))
    swing = thigh + shank + foot
    return dict(foot=foot, shank=shank, thigh=thigh, swinging=swing, hip_module=module + swing, whole=module + swing + body)


if __name__ == "__main__":
    for name, g in (("snow", snow_groups()), ("original", original_groups())):
        print(name, {k: round(v, 1) for k, v in g.items()})
