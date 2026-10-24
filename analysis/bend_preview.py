"""Preview of shank bend options (no SolidWorks): the current bend vs. the bend flipped to the
fold side and made gentler. Each is drawn hanging straight (q = 0) and in a walking stance pose.
Writes docs/renders/bend_preview.png.  Run:  python analysis/bend_preview.py
"""
import importlib
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))
import params as P  # noqa: E402
import build_parts as B  # noqa: E402
import gait_check as GC  # noqa: E402

POSE = (math.radians(-32.7), math.radians(69.7))      # walking stance pose from the animation
OPTIONS = [("Before: bowed 20 mm to the back", 22.0, 22.0), ("Built: flipped, bows ~9 mm\n(to the front / fold side)", -10.0, -10.0)]


def use(bend, tilt):
    P.SHANK_BEND_DEG, P.SHANK_FOOT_TILT_DEG = bend, tilt
    importlib.reload(B)
    GC._SHAPES = None
    return max(abs(B.shank_frame(B.SH_LEN * i / 100)[0][0]) for i in range(101))


def main():
    orig = (P.SHANK_BEND_DEG, P.SHANK_FOOT_TILT_DEG)
    fig, axs = plt.subplots(1, 2 * len(OPTIONS), figsize=(13, 7))
    ink, near = "#1f2328", "#2f6fdf"
    for j, (name, bend, tilt) in enumerate(OPTIONS):
        bow = use(bend, tilt)
        for k, (q1, q2, sub) in enumerate(((0.0, 0.0, "hanging straight"), (*POSE, "walking stance"))):
            ax = axs[2 * j + k]
            L = GC.draw_leg(ax, (0, 0), q1, q2, near, ink, "#57606a")
            ax.plot([0, L["knee"][0], L["ft"][0]], [0, L["knee"][1], L["ft"][1]], color="#d1242f", lw=0.8, ls="--")
            ax.set_xlim(-110, 150); ax.set_ylim(-290, 40); ax.set_aspect("equal"); ax.axis("off")
            ax.set_title(f"{name}\n{sub}" if k == 0 else f"{sub}\n(bow {bow:.0f} mm)", fontsize=9.5)
    use(*orig)
    fig.suptitle("Shank bend: robot front is to the right (+x). Red dashes = straight hip-knee-foot line; "
                 "foot position and kinematics are the same in both.", fontsize=10)
    out = os.path.join(ROOT, "docs", "renders", "bend_preview.png")
    fig.savefig(out, dpi=100, bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    main()
