"""Snowshoe foot preview (no SolidWorks): top view, side view, mechanics and the numbers behind them.

One printed part (deck + rim + ribs + clevis cheeks) replaces the Ø22 ball foot. It hangs on the
shank's existing foot plug through the existing M3 x 25 cross bolt, which becomes a free pitch pivot
(ankle) limited by two stop blocks. Writes docs/renders/snowshoe_preview.png and prints the numbers.
Snow sinkage: Bekker p = (k_c/b + k_phi) z^n with the three snow sets compiled by Lines, Elliott &
Ray (2021) from Wong (2001) (as used in snow_leg/analysis/snow_model.py).
Run:  python analysis/snowshoe_preview.py
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.path import Path  # noqa: E402
from matplotlib.patches import Circle, Polygon, FancyArrowPatch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))
import params as P  # noqa: E402
import build_parts as B  # noqa: E402
import gait_check as GC  # noqa: E402

# ------------------------------------------------------------------ proposed snowshoe (mm)
SHOE_L, R_TOE, R_TAIL = 110.0, 35.0, 27.0      # teardrop planform: 70 mm wide at the toe, 54 at the heel
DECK_T, RIM_H, RIM_W = 2.4, 6.0, 2.0           # 6 perimeters deck... rim and ribs stiffen it
RIB_X = (-28.0, 26.0)                          # transverse ribs (pad frame, x forward from the pivot)
CHEEK_T, CHEEK_GAP = 4.0, P.SHANK_T + 0.4      # clevis cheeks either side of the 10 mm shank plug
PIVOT_UP = 13.0                                # pivot axis above the sole
PIVOT_AHEAD = 0.08                             # pivot ahead of the planform centroid (fraction of length)
TOE_LIFT = 8.0                                 # upturned toe: last 18 mm of the sole bevelled up 8 mm
MARGIN = 10.0                                  # deg of pivot travel past the walking stance range
HOOK_X = (-36.0, 40.0)                         # band hooks on the deck (pad frame)
BAND_WRAP = 32.0                               # band crosses the shank this far up from the pivot

BEKKER = {"B1": (1.6, 4.37, 196.72), "B2": (1.6, 2.49, 245.90), "B3": (1.44, 10.55, 66.08)}   # n, k_c, k_phi


def teardrop(n=200):
    """Planform in the pad frame (x forward, z sideways), tail centre at x = 0."""
    d = SHOE_L - R_TOE - R_TAIL
    a = math.acos((R_TOE - R_TAIL) / d)
    pts = [(d + R_TOE * math.cos(t), R_TOE * math.sin(t)) for t in np.linspace(-a, a, n)]
    pts += [(R_TAIL * math.cos(t), R_TAIL * math.sin(t)) for t in np.linspace(a, 2 * math.pi - a, n)]
    return np.array(pts)


def area_centroid(p):
    x, y = p[:, 0], p[:, 1]
    cr = x * np.roll(y, -1) - np.roll(x, -1) * y
    A = cr.sum() / 2
    return abs(A), (x + np.roll(x, -1)) @ cr / (6 * A)


def sinkage_mm(F, area_mm2, b_mm):
    """Bekker sinkage per snow set (mm) under force F (N) on area (mm^2), b = smaller half-dimension."""
    p = F / area_mm2 * 1000.0                          # kPa
    return {k: 1000 * (p / (kc / (b_mm / 1000) + kphi)) ** (1 / n) for k, (n, kc, kphi) in BEKKER.items()}


def main():
    plan = teardrop()
    A, xc = area_centroid(plan)
    xp = xc + PIVOT_AHEAD * SHOE_L                     # pivot x in the tail-centre frame
    plan_p = plan - (xp, 0.0)                          # pad frame: origin under the pivot
    x_tail, x_toe = -R_TAIL - xp, SHOE_L - R_TAIL - xp

    # stance shank angles (walk) -> pivot range needed to keep the sole flat
    rows, _ = GC.analyse("walk", GC.GAITS["walk"])
    phi = [math.degrees(r["q"][1] + r["q"][2]) for r in rows if r["stance"]]
    a_lo, a_hi = min(phi) - MARGIN, max(phi) + MARGIN
    i_td = next(i for i in range(len(rows)) if rows[i]["stance"] and not rows[i - 1]["stance"])     # touchdown
    i_lo = next(i for i in range(len(rows)) if not rows[i]["stance"] and rows[i - 1]["stance"])     # lift-off
    ang = lambda i: math.degrees(rows[i]["q"][1] + rows[i]["q"][2])                                 # noqa: E731
    phi_td, phi_lo = ang(i_td), ang(i_lo - 1)
    phi_sw = float(np.mean([math.degrees(r["q"][1] + r["q"][2]) for r in rows if not r["stance"]]))

    # loads
    m = GC.ROBOT_MASS + 4 * 0.015                      # + four snowshoes (~25 g each instead of 9 g balls)
    F_walk = m * 9.81 / 3
    ball_A = math.pi * (P.FOOT_D / 2) ** 2             # ball pushed in to its equator
    s_ball = sinkage_mm(F_walk, ball_A, P.FOOT_D / 2)
    s_shoe = sinkage_mm(F_walk, A, R_TAIL)
    gcm2 = lambda F, a: F / 9.81 * 1000 / (a / 100)   # noqa: E731  g per cm^2 (snow-animal "foot load")

    # pivot = the existing M3 cross hole on the tilted foot axis
    piv = B.ax(B.FOOT_SCREW_A)
    L2p = math.dist(piv, B.KNEE)
    off = math.degrees(math.atan2(piv[0] - B.KNEE[0], B.KNEE[1] - piv[1]))

    # ---- knee fold check in side view (conservative: assumes the pad overlaps the thigh / hip in z)
    thigh = Path(GC._ccw(GC._loop_pts(B.thigh_outline())))
    hip_r = P.MOTOR_D / 2 + 2.0
    side = lambda a: np.array([[x, -PIVOT_UP] for x in np.linspace(x_tail, x_toe, 60)] +          # noqa: E731
                              [[x, -PIVOT_UP + DECK_T + RIM_H] for x in np.linspace(x_tail, x_toe, 60)])
    fold = 0
    for q2 in range(0, 171):
        hit = False
        for a in (a_lo, a_hi):
            pts = GC._rot(side(0) , -math.radians(a)) + piv      # pad flat-in-world at shank angle a
            pts = GC._rot(pts, math.radians(q2), B.KNEE)
            if thigh.contains_points(pts).any() or (np.hypot(pts[:, 0], pts[:, 1]) < hip_r).any():
                hit = True
        if hit:
            break
        fold = q2

    # ---------------------------------------------------------------- figure
    ink, blue, pad_c, grid = "#1f2328", "#2f6fdf", "#e07b39", "#d0d7de"
    fig = plt.figure(figsize=(15, 9.2))
    fig.suptitle("Snowshoe foot: one printed part on the existing M3 cross-bolt (free pitch ankle with stops). "
                 "Robot front = right (+x).", fontsize=12, weight="bold")

    # top view
    ax = fig.add_axes([0.02, 0.36, 0.30, 0.52])
    ax.add_patch(Polygon(plan_p, closed=True, fc=pad_c, ec=ink, lw=1.2, alpha=0.9))
    inner = teardrop()
    inner = (inner - (inner.mean(0))) * np.array([(SHOE_L - 2 * RIM_W) / SHOE_L, (2 * R_TOE - 2 * RIM_W) / (2 * R_TOE)]) + inner.mean(0) - (xp, 0)
    ax.add_patch(Polygon(inner, closed=True, fc="#f3a46d", ec=ink, lw=0.6))
    zw = lambda x: np.interp(x, *zip(*sorted((p[0], abs(p[1])) for p in plan_p if p[1] >= 0)))  # noqa: E731
    for rx in RIB_X:
        ax.add_patch(Polygon([(rx - 1, -zw(rx) + 2), (rx + 1, -zw(rx) + 2), (rx + 1, zw(rx) - 2), (rx - 1, zw(rx) - 2)], fc=ink, alpha=0.55))
    ax.add_patch(Polygon([(x_tail + 6, -1), (x_toe - 6, -1), (x_toe - 6, 1), (x_tail + 6, 1)], fc=ink, alpha=0.55))
    for hx in HOOK_X:
        ax.add_patch(Circle((hx, 0), 2.5, fc="#2da44e", ec=ink, lw=0.6))
    ax.plot(HOOK_X, [0, 0], color="#2da44e", lw=2, alpha=0.6)
    for s in (-1, 1):                                                     # clevis cheeks
        z0 = s * CHEEK_GAP / 2
        ax.add_patch(Polygon([(-11, z0), (11, z0), (11, z0 + s * CHEEK_T), (-11, z0 + s * CHEEK_T)], fc="#8c4a1d", ec=ink, lw=0.6))
    ax.add_patch(Polygon([(-6, -P.SHANK_T / 2), (6, -P.SHANK_T / 2), (6, P.SHANK_T / 2), (-6, P.SHANK_T / 2)], fc=blue, ec=ink, lw=0.6))
    ax.plot([0, 0], [-CHEEK_GAP / 2 - CHEEK_T - 3, CHEEK_GAP / 2 + CHEEK_T + 3], color="#d1242f", lw=1.6)
    ax.plot(xc - xp, 0, "k+", ms=10)
    ax.annotate("planform centroid", (xc - xp, 0), (-55, -48), fontsize=8, arrowprops=dict(arrowstyle="-", color=ink, lw=0.6))
    ax.annotate("pivot: existing M3 × 25 bolt\nthrough cheeks + shank plug", (0, CHEEK_GAP / 2 + CHEEK_T + 3), (5, 46), fontsize=8,
                arrowprops=dict(arrowstyle="-", color=ink, lw=0.6))
    ax.text(x_toe - 4, -44, "toe", fontsize=8, ha="right"); ax.text(x_tail + 2, -44, "heel", fontsize=8)
    ax.annotate("", (x_tail, -40), (x_toe, -40), arrowprops=dict(arrowstyle="<->", lw=0.7))
    ax.text((x_tail + x_toe) / 2, -38, f"{SHOE_L:.0f} mm", ha="center", fontsize=8)
    ax.text(x_toe + 4, 0, f"{2 * R_TOE:.0f}\nwide", fontsize=8, va="center")
    ax.text(x_tail - 4, 0, f"{2 * R_TAIL:.0f}\nwide", fontsize=8, va="center", ha="right")
    ax.set_xlim(x_tail - 22, x_toe + 22); ax.set_ylim(-52, 62); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(f"TOP VIEW — sole area {A / 100:.0f} cm² (ball foot: {ball_A / 100:.1f} cm²)", fontsize=10)

    # side view (pad frame, at stance) with the shank tip
    ax = fig.add_axes([0.34, 0.36, 0.33, 0.52])
    sole = [(x_tail + 4, -PIVOT_UP), (x_toe - 18, -PIVOT_UP), (x_toe, -PIVOT_UP + TOE_LIFT), (x_toe, -PIVOT_UP + DECK_T + RIM_H),
            (x_tail, -PIVOT_UP + DECK_T + RIM_H), (x_tail, -PIVOT_UP + 4)]
    ax.add_patch(Polygon(sole, closed=True, fc=pad_c, ec=ink, lw=1.2))
    ax.plot([x_tail + 3, x_toe - 3], [-PIVOT_UP + DECK_T, -PIVOT_UP + DECK_T], color=ink, lw=0.6, ls=":")
    ax.add_patch(Polygon([(-11, -PIVOT_UP + DECK_T + RIM_H), (11, -PIVOT_UP + DECK_T + RIM_H), (8, 7), (-8, 7)], fc="#8c4a1d", ec=ink, lw=0.8))
    # shank tip at mid-stance angle, in the pad frame (pad flat): rotate the CAD shank about the pivot
    mid = (min(phi) + max(phi)) / 2
    for loops, fc in ((GC.cad_shapes()["shank"], blue),):
        for k, lp in enumerate(loops[:1]):
            pts = GC._rot(lp - piv, math.radians(mid))
            ax.add_patch(Polygon(pts, closed=True, fc=fc, ec=ink, lw=0.8, alpha=0.95))
    plug = GC._rot(GC.cad_shapes()["plug"] - piv, math.radians(mid))
    ax.add_patch(Polygon(plug, closed=True, fc=blue, ec=ink, lw=0.8))
    ax.add_patch(Circle((0, 0), 1.7, fc="white", ec="#d1242f", lw=1.5))
    wrap = GC._rot(np.array([[B.FOOT_UP[0] * BAND_WRAP, B.FOOT_UP[1] * BAND_WRAP]]), math.radians(mid))[0]   # on the plug axis
    for hx in HOOK_X:
        ax.add_patch(Polygon([(hx - 2, -PIVOT_UP + DECK_T), (hx + 2, -PIVOT_UP + DECK_T), (hx + 2, -PIVOT_UP + DECK_T + RIM_H + 5),
                              (hx - 2, -PIVOT_UP + DECK_T + RIM_H + 5)], fc="#8c4a1d", ec=ink, lw=0.6))
        ax.plot([hx, wrap[0]], [-PIVOT_UP + DECK_T + RIM_H + 4, wrap[1]], color="#2da44e", lw=2)
    ax.annotate("rubber band: heel hook → round the shank → toe hook\n(centres the pad in swing; the ground flattens it in stance)",
                (HOOK_X[1], -PIVOT_UP + DECK_T + RIM_H + 5), (HOOK_X[1] - 30, 52), fontsize=7.5, color="#2da44e",
                arrowprops=dict(arrowstyle="-", color="#2da44e", lw=0.6))
    for a, lab in ((a_lo, "heel-down stop"), (a_hi, "toe-down stop")):
        d = GC._rot(np.array([[0.0, 34.0]]), math.radians(a))[0]
        ax.add_patch(FancyArrowPatch((0, 0), tuple(d), arrowstyle="-", ls="--", color="#d1242f", lw=0.9))
        ax.text(d[0] * 1.08, d[1] * 1.08 + 2, lab, fontsize=7.5, color="#d1242f", ha="center")
    ax.plot([x_tail - 15, x_toe + 15], [-PIVOT_UP, -PIVOT_UP], color=ink, lw=1.2)
    for gx in np.arange(x_tail - 15, x_toe + 15, 8):
        ax.plot([gx, gx - 4], [-PIVOT_UP, -PIVOT_UP - 4], color=grid, lw=0.8)
    ax.annotate(f"upturned toe ({TOE_LIFT:.0f} mm)", (x_toe - 6, -PIVOT_UP + 4), (x_toe - 40, -PIVOT_UP - 12), fontsize=8,
                arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.annotate(f"pivot {PIVOT_UP:.0f} mm above sole", (0, 0), (-60, 30), fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.text(x_tail, -PIVOT_UP + DECK_T + RIM_H + 2, f"rim + 2 ribs {RIM_H:.0f} mm tall\ndeck {DECK_T} mm", fontsize=7.5)
    ax.set_xlim(x_tail - 18, x_toe + 18); ax.set_ylim(-PIVOT_UP - 18, 62); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(f"SIDE VIEW — at mid-stance (shank {mid:.0f}° from vertical)", fontsize=10)

    # mechanics: three little poses (heel strike, mid-stance, toe-off) + swing
    ax = fig.add_axes([0.69, 0.36, 0.30, 0.52])
    ax.set_title("HOW IT MOVES (one step, left → right)", fontsize=10)
    poses = [("end of swing: band\nholds the pad a few\ndegrees toe-up", phi_td, 22), ("touch-down:\nsole flat", phi_td, 0),
             ("mid-stance:\nsole flat", mid, 0), ("lift-off:\nsole flat", phi_lo, 0)]
    for i, (lab, a, lift) in enumerate(poses):
        ox = i * 75
        tilt = (a - mid) if i == 0 else 0.0                # swing: band holds the shank at `mid` in the pad frame
        padp = GC._rot(np.array(sole, float), math.radians(tilt)) * 0.55 + (ox, lift)
        ax.add_patch(Polygon(padp, closed=True, fc=pad_c, ec=ink, lw=0.8))
        d = GC._rot(np.array([[0.0, 60.0]]), math.radians(a))[0] * 0.55
        ax.plot([ox, ox + d[0]], [lift, lift + d[1]], color=blue, lw=5, solid_capstyle="round")
        ax.add_patch(Circle((ox, lift), 1.8, fc="white", ec="#d1242f", lw=1.2))
        ax.text(ox, -22, lab, ha="center", va="top", fontsize=7.5)
    ax.plot([-40, 270], [-PIVOT_UP * 0.55, -PIVOT_UP * 0.55], color=ink, lw=1)
    ax.set_xlim(-45, 275); ax.set_ylim(-60, 75); ax.set_aspect("equal"); ax.axis("off")

    # numbers
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.30]); ax.axis("off")
    rows_txt = [
        f"Load per foot, walking (3 feet down, {m:.1f} kg robot): {F_walk:.1f} N",
        f"Foot pressure: ball {gcm2(F_walk, ball_A):.0f} g/cm²  →  snowshoe {gcm2(F_walk, A):.0f} g/cm²  "
        f"({A / ball_A:.0f}× lower; lynx 26–32, red fox 89, coyote 106–170, moose 618–922 g/cm²)",
        "Bekker sinkage in soft snow (sets B1 / B2 / B3): ball " + " / ".join(f"{v:.0f}" for v in s_ball.values()) +
        " mm  →  snowshoe " + " / ".join(f"{v:.0f}" for v in s_shoe.values()) + " mm",
        f"Ankle: pitch pivot on the existing M3 × 25 bolt (snug nylock, not clamped). In stance the shank goes from {phi_td:.0f}° (touch-down) "
        f"to {phi_lo:.0f}° (lift-off) from vertical; the pivot lets the sole stay flat the whole time. Stops at {a_lo:.0f}° and {a_hi:.0f}° (±{MARGIN:.0f}°).",
        f"One rubber band (heel hook → round the shank → toe hook) centres the pad at the mid-stance angle ({mid:.0f}°) in swing, so it lands "
        f"{phi_td - mid:.0f}° toe-up (heel first) and never toe-down; it is weak, so body weight flattens the pad. Pivot {PIVOT_AHEAD * 100:.0f} % "
        "ahead of the centroid and an upturned toe help it ride up out of the snow.",
        f"Kinematics: the IK foot point moves from the ball centre to the pivot, {L2p:.1f} mm from the knee ({off:+.1f}° off the old line), "
        f"so L2 becomes {L2p:.0f} mm in the gait / IK model; the sole sits {PIVOT_UP:.0f} mm below it (ball: {P.FOOT_D / 2:.0f} mm), "
        f"so at the same joint angles the body stands ~{(P.L2 - L2p) + (PIVOT_UP - P.FOOT_D / 2):.0f} mm lower.",
        f"Knee fold with the shoe (side-view check, pad on either stop): clear to about {fold}° (ball foot: 162°; firmware limit 150°). "
        "Walking uses 63–96°; if the SolidWorks sweep confirms it, the knee limit drops to ~145° with the snowshoe.",
        f"Parts: 1 printed snowshoe (deck + rim + ribs + cheeks + stops + 2 hooks, ~25 g PLA, prints flat sole-down, no supports) "
        "+ 1 rubber band + the existing M3 × 25 bolt and nylock. The ball foot is retired; no other part of the leg changes.",
    ]
    for i, t in enumerate(rows_txt):
        ax.text(0.0, 0.95 - i * 0.125, "•  " + t, fontsize=9.2, va="top", transform=ax.transAxes)
    out = os.path.join(ROOT, "docs", "renders", "snowshoe_preview.png")
    fig.savefig(out, dpi=100, bbox_inches="tight")
    print("\n".join(rows_txt))
    print("wrote", out)


if __name__ == "__main__":
    main()
