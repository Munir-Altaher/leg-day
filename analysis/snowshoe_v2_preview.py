"""Snowshoe foot v2 preview (no SolidWorks): generative honeycomb (graded Voronoi) sole and
lip stops instead of the rubber band. Two sole options are drawn:
  A  closed 0.8 mm skin on the snow side + Voronoi ribs on top (pockets): full bearing area
  B  open Voronoi lattice through the sole: lightest, but snow can fall through the cells
Stops-only viability: the swing-phase torques on the pad (gravity, inertia) are compared with the
hinge friction, from the walking gait in gait_check.py.
Writes docs/renders/snowshoe_v2_preview.png.  Run:  python analysis/snowshoe_v2_preview.py
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))
import params as P  # noqa: E402
import build_parts as B  # noqa: E402
import gait_check as GC  # noqa: E402
import snowshoe_preview as S1  # noqa: E402

PIVOT_UP = 15.0           # pivot above the sole (raised 2 mm so the plug end clears the lattice ribs)
RIM_W, RIM_H = 2.4, 6.0
SKIN_A = 0.8              # option A: closed skin (4 layers at 0.2 mm)
RIB_W = 1.2               # Voronoi rib width (3 lines at 0.4 mm)
RIB_H_A = 3.0             # option A: rib height above the skin
DECK_B = 2.4              # option B: open lattice thickness
CELL_MIN, CELL_MAX = 5.5, 11.0   # Voronoi seed spacing near / far from the pivot (generative grading)
SOLID_PIVOT = (-12.0, 12.0, -9.5, 9.5)    # solid zone under the cheeks (x0, x1, z0, z1)
STOP_MARGIN = 5.0         # deg outside the walking stance range
LIP_H = 15.0              # lip contact height above the pivot
PAD_DENS = 1.24e-3        # g/mm^3 PLA (thin walls print solid)
MU, PRELOAD = 0.3, 60.0   # friction hinge: M3 nylock + wave washer preload (N) -> friction torque


# ------------------------------------------------------------------ geometry helpers
def clip(poly, a, b):
    """Keep the part of convex polygon `poly` with (p - a) . b <= 0 (Sutherland-Hodgman, one half-plane)."""
    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        fp, fq = np.dot(p - a, b), np.dot(q - a, b)
        if fp <= 0:
            out.append(p)
        if fp * fq < 0:
            out.append(p + (q - p) * fp / (fp - fq))
    return np.array(out) if out else np.zeros((0, 2))


def offset_convex(poly, d):
    """Convex polygon offset inward by d (edges moved in, consecutive lines intersected)."""
    n = len(poly)
    ccw = np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1]) > 0
    lines = []
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        t = q - p
        L = np.linalg.norm(t)
        if L < 1e-6:
            continue
        nrm = np.array([-t[1], t[0]]) / L * (1 if ccw else -1)        # inward normal
        lines.append((p + nrm * d, t / L))
    out = []
    for i in range(len(lines)):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-9:
            continue
        s = ((p2 - p1)[0] * d2[1] - (p2 - p1)[1] * d2[0]) / den
        out.append(p1 + d1 * s)
    out = np.array(out)
    if len(out) < 3:
        return None
    # collapsed if orientation flipped or area tiny
    a = np.sum(out[:, 0] * np.roll(out[:, 1], -1) - np.roll(out[:, 0], -1) * out[:, 1]) / 2
    return out if (a > 0) == ccw and abs(a) > 6.0 else None


def area(p):
    return abs(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])) / 2


def inside_convex(poly, pts):
    ok = np.ones(len(pts), bool)
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        ok &= (q[0] - p[0]) * (pts[:, 1] - p[1]) - (q[1] - p[1]) * (pts[:, 0] - p[0]) >= -1e-9
    return ok


def voronoi_cells(outline, seed=7):
    """Graded Voronoi lattice: seeds by dart throwing with spacing CELL_MIN at the pivot growing to CELL_MAX
    at the far edge (the plate's bending moment is largest at the pivot, so ribs are densest there)."""
    rng = np.random.default_rng(seed)
    rmax = np.max(np.hypot(outline[:, 0], outline[:, 1]))
    spacing = lambda p: CELL_MIN + (CELL_MAX - CELL_MIN) * min(1.0, math.hypot(*p) / rmax)   # noqa: E731
    x0, x1, z0, z1 = outline[:, 0].min(), outline[:, 0].max(), outline[:, 1].min(), outline[:, 1].max()
    seeds = []
    for _ in range(40000):
        p = np.array([rng.uniform(x0, x1), rng.uniform(z0, z1)])
        if not inside_convex(outline, p[None])[0]:
            continue
        if all(np.hypot(*(p - q)) >= 0.5 * (spacing(p) + spacing(q)) for q in seeds):
            seeds.append(p)
    seeds = np.array(seeds)
    cells = []
    for i, p in enumerate(seeds):
        c = outline.copy()
        near = np.argsort(np.hypot(*(seeds - p).T))[1:16]
        for j in near:
            q = seeds[j]
            c = clip(c, (p + q) / 2, q - p)
            if len(c) < 3:
                break
        if len(c) >= 3:
            cells.append(c)
    return seeds, cells


def lattice(outline):
    """Cell openings: Voronoi cells shrunk by RIB_W/2, minus the solid pivot zone."""
    _, cells = voronoi_cells(outline)
    holes = []
    for c in cells:
        h = offset_convex(c, RIB_W / 2)
        if h is None:
            continue
        x0, x1, z0, z1 = SOLID_PIVOT
        if (h[:, 0].max() > x0 and h[:, 0].min() < x1 and h[:, 1].max() > z0 and h[:, 1].min() < z1):
            continue                                              # keep the cheek footing solid
        holes.append(h)
    return holes


# ------------------------------------------------------------------ main
def main():
    plan = S1.teardrop()
    A_out, xc = S1.area_centroid(plan)
    xp = xc + S1.PIVOT_AHEAD * S1.SHOE_L
    plan = plan - (xp, 0.0)
    x_tail, x_toe = plan[:, 0].min(), plan[:, 0].max()
    c0 = plan.mean(0)
    inner = (plan - c0) * np.array([(S1.SHOE_L - 2 * RIM_W) / S1.SHOE_L, (2 * S1.R_TOE - 2 * RIM_W) / (2 * S1.R_TOE)]) + c0
    holes = lattice(inner)
    A_in = area(inner)
    A_holes = sum(area(h) for h in holes)
    A_ribs = A_in - A_holes
    open_frac = A_holes / A_out

    # masses (deck only + rim; cheeks/lips ~4 g common to all)
    rim_v = (A_out - A_in) * RIM_H
    m_solid = (A_out * 2.4 + rim_v + 2 * 60 * 2 * 6 + 80 * 2 * 4) * PAD_DENS          # v1: 2.4 deck + 2 ribs + keel
    m_A = (A_out * SKIN_A + A_ribs * RIB_H_A + rim_v) * PAD_DENS
    m_B = (A_ribs * DECK_B + rim_v) * PAD_DENS
    cheeks = 4.0

    # gait: shank angle in stance / swing
    rows, _ = GC.analyse("walk", GC.GAITS["walk"])
    phi = np.array([math.degrees(r["q"][1] + r["q"][2]) for r in rows])
    st = np.array([r["stance"] for r in rows])
    p_lo, p_hi = phi[st].min(), phi[st].max()
    i_td = next(i for i in range(len(rows)) if st[i] and not st[i - 1])
    phi_td = phi[i_td]
    a_lo, a_hi = p_lo - STOP_MARGIN, p_hi + STOP_MARGIN

    # ---- stops-only viability: swing torques on the pad about the pivot
    m_pad = (m_A + cheeks) / 1000                         # kg
    e = S1.PIVOT_AHEAD * S1.SHOE_L / 1000                 # centroid offset behind the pivot (m)
    I_pad = m_pad * ((S1.SHOE_L / 1000) ** 2 / 12 + e ** 2)
    T = GC.GAITS["walk"]["T"]
    dt = T / len(rows)
    w = np.gradient(np.radians(phi), dt)
    al = np.gradient(w, dt)
    # smooth the trajectory corners (the planned path has sharp stance/swing corners a real controller rounds)
    k = 9
    al_s = np.convolve(np.concatenate([al[-k:], al, al[:k]]), np.ones(2 * k + 1) / (2 * k + 1), "same")[k:-k]
    tq_grav = m_pad * 9.81 * e * 1000                     # mN m
    tq_inert = I_pad * np.abs(al_s[~st]).max() * 1000     # mN m (pad held still relative to the shank)
    tq_fric = MU * PRELOAD * 0.0025 * 1000 * 2            # two faces, mean radius 2.5 mm, mN m
    F_walk = (GC.ROBOT_MASS + 0.1) * 9.81 / 3
    tq_ground = F_walk * (x_toe / 1000) * 1000            # ground force at the toe, mN m
    land_up = phi_td - p_lo                               # pad keeps its lift-off angle -> toe-up at touch-down

    # ---------------------------------------------------------------- figure
    ink, pad_c, pad_d, blue, red, grid = "#1f2328", "#e07b39", "#9c4f1b", "#2f6fdf", "#d1242f", "#d0d7de"
    fig = plt.figure(figsize=(16, 11))
    fig.suptitle("Snowshoe foot v2 — generative honeycomb sole + lip stops (no rubber band). Robot front = right (+x).",
                 fontsize=13, weight="bold")

    def top(ax, title, open_cells):
        ax.add_patch(Polygon(plan, closed=True, fc=pad_d, ec=ink, lw=1.2))
        ax.add_patch(Polygon(inner, closed=True, fc=pad_c, ec=ink, lw=0.4))
        for h in holes:
            ax.add_patch(Polygon(h, closed=True, fc="white" if open_cells else "#f6c9a6", ec=ink, lw=0.35))
        for s in (-1, 1):
            z0 = s * (P.SHANK_T + 0.4) / 2
            ax.add_patch(Polygon([(-11, z0), (11, z0), (11, z0 + s * 4), (-11, z0 + s * 4)], fc="#6b3410", ec=ink, lw=0.6))
        ax.add_patch(Polygon([(-6, -5), (6, -5), (6, 5), (-6, 5)], fc=blue, ec=ink, lw=0.6))
        ax.plot([0, 0], [-13, 13], color=red, lw=1.6)
        ax.set_xlim(x_tail - 6, x_toe + 6); ax.set_ylim(-40, 40); ax.set_aspect("equal"); ax.axis("off")
        ax.set_title(title, fontsize=9.5)

    top(fig.add_axes([0.02, 0.55, 0.30, 0.36]),
        f"A (recommended): closed {SKIN_A} mm skin underneath + ribs on top\n"
        f"{len(holes)} cells graded {CELL_MIN:.0f}→{CELL_MAX:.0f} mm · deck {m_A:.1f} g (solid: {m_solid:.1f} g)\n"
        f"all {A_out / 100:.0f} cm² bears on the snow", False)
    top(fig.add_axes([0.34, 0.55, 0.30, 0.36]),
        f"B: open lattice through the sole ({DECK_B} mm)\n"
        f"deck {m_B:.1f} g · {open_frac * 100:.0f} % open\n"
        f"snow can fall through (worst case only {A_out * (1 - open_frac) / 100:.0f} cm² bears)", True)

    # section sketch through a few cells
    ax = fig.add_axes([0.67, 0.62, 0.31, 0.25])
    ax.set_title("Sections (not to scale across)", fontsize=9.5)
    for row, (lab, skin, rib) in enumerate((("A", SKIN_A, RIB_H_A), ("B", 0.0, DECK_B))):
        y0 = -row * 14
        if skin:
            ax.add_patch(Polygon([(0, y0), (60, y0), (60, y0 + skin), (0, y0 + skin)], fc=pad_d, ec=ink, lw=0.6))
        for xr in np.arange(0, 61, 10):
            ax.add_patch(Polygon([(xr - RIB_W / 2, y0 + skin), (xr + RIB_W / 2, y0 + skin), (xr + RIB_W / 2, y0 + skin + rib),
                                  (xr - RIB_W / 2, y0 + skin + rib)], fc=pad_d, ec=ink, lw=0.6))
        ax.plot([-3, 63], [y0 - 0.3, y0 - 0.3], color="#8fb3d9", lw=3, alpha=0.6)
        ax.text(-6, y0 + 2, lab, fontsize=10, weight="bold", ha="right")
    ax.text(64, 2, f"skin {SKIN_A} + ribs {RIB_W} × {RIB_H_A}", fontsize=8, va="center")
    ax.text(64, -12, f"ribs {RIB_W} × {DECK_B}, no skin", fontsize=8, va="center")
    ax.text(30, -20, "blue = snow surface", fontsize=7.5, color="#4a7fb5", ha="center")
    ax.set_xlim(-10, 100); ax.set_ylim(-24, 8); ax.set_aspect("equal"); ax.axis("off")

    # side views at both stops: lips catch the flat faces of the shank plug
    def side(ax, a, title):
        sole = np.array([(x_tail + 4, -PIVOT_UP), (x_toe - 18, -PIVOT_UP), (x_toe, -PIVOT_UP + S1.TOE_LIFT), (x_toe, -PIVOT_UP + RIM_H),
                         (x_tail, -PIVOT_UP + RIM_H), (x_tail, -PIVOT_UP + 4)])
        ax.add_patch(Polygon(sole, closed=True, fc=pad_c, ec=ink, lw=1.0))
        u0, n0 = np.array(B.FOOT_UP), np.array(B.FOOT_ACROSS)
        R = lambda v, ang: GC._rot(np.atleast_2d(v), math.radians(ang))   # noqa: E731
        lips = []
        for ang, sgn in ((a_lo, 1), (a_hi, -1)):          # low stop: +x face of the plug; high stop: -x face
            pc = R(LIP_H * u0 + sgn * 6.0 * n0, ang)[0]
            uu, nn = R(u0, ang)[0], R(sgn * n0, ang)[0]
            lips.append(np.array([pc - uu * 4, pc + uu * 4, pc + uu * 4 + nn * 5, pc - uu * 4 + nn * 5]))
        cheek = np.array([(-16, -PIVOT_UP + RIM_H), (16, -PIVOT_UP + RIM_H)] +
                         [tuple(lips[0][2]), tuple(lips[0][1]), tuple(lips[1][1]), tuple(lips[1][2])])
        ax.add_patch(Polygon(cheek, closed=True, fc="#c4703a", ec=ink, lw=0.8, alpha=0.9))
        for L in lips:
            ax.add_patch(Polygon(L, closed=True, fc="#6b3410", ec=ink, lw=0.8))
        sh = GC._rot(GC.cad_shapes()["shank"][0] - B.ax(B.FOOT_SCREW_A), math.radians(a))
        ax.add_patch(Polygon(sh, closed=True, fc=blue, ec=ink, lw=0.8, alpha=0.95))
        plug = GC._rot(GC.cad_shapes()["plug"] - B.ax(B.FOOT_SCREW_A), math.radians(a))
        ax.add_patch(Polygon(plug, closed=True, fc=blue, ec=ink, lw=0.8))
        ax.add_patch(Circle((0, 0), 1.7, fc="white", ec=red, lw=1.5))
        ax.plot([x_tail - 10, x_toe + 10], [-PIVOT_UP, -PIVOT_UP], color=ink, lw=1.1)
        for gx in np.arange(x_tail - 10, x_toe + 10, 8):
            ax.plot([gx, gx - 4], [-PIVOT_UP, -PIVOT_UP - 4], color=grid, lw=0.8)
        ax.set_xlim(x_tail - 10, x_toe + 10); ax.set_ylim(-PIVOT_UP - 8, 58); ax.set_aspect("equal"); ax.axis("off")
        ax.set_title(title, fontsize=9.5)

    side(fig.add_axes([0.02, 0.25, 0.30, 0.27]), a_lo,
         f"Side: on the FRONT lip — shank at {a_lo:.0f}° in the pad frame\n(lift-off is at {p_lo:.0f}°; {STOP_MARGIN:.0f}° spare)")
    side(fig.add_axes([0.34, 0.25, 0.30, 0.27]), a_hi,
         f"Side: on the REAR lip — shank at {a_hi:.0f}°\n(touch-down is at {p_hi:.0f}°; stops toe-down beyond this)")

    # step sequence with a friction hinge
    ax = fig.add_axes([0.67, 0.27, 0.31, 0.28])
    ax.set_title("One step with lip stops + friction hinge", fontsize=9.5)
    seq = [("lift-off:\nflat", p_lo, 0, 0.0), ("mid-swing:\nkeeps its angle\n→ toe up", 40.0, 22, 40.0 - p_lo),
           (f"touch-down:\n{land_up:.0f}° toe-up,\nheel first", phi_td, 6, land_up), ("ground rolls\nit flat", phi_td, 0, 0.0)]
    for i, (lab, a, lift, tilt) in enumerate(seq):
        ox = i * 72
        sole = np.array([(-30, -8), (25, -8), (32, -4), (32, -3), (-30, -3)], float)
        pp = GC._rot(sole, math.radians(tilt)) + (ox, lift)
        ax.add_patch(Polygon(pp, closed=True, fc=pad_c, ec=ink, lw=0.8))
        d = GC._rot(np.array([[0.0, 40.0]]), math.radians(a))[0]
        ax.plot([ox, ox + d[0]], [lift, lift + d[1]], color=blue, lw=5, solid_capstyle="round")
        ax.add_patch(Circle((ox, lift), 1.8, fc="white", ec=red, lw=1.2))
        ax.text(ox, -16, lab, ha="center", va="top", fontsize=7.5)
    ax.plot([-40, 260], [-8, -8], color=ink, lw=1)
    ax.set_xlim(-45, 262); ax.set_ylim(-48, 62); ax.set_aspect("equal"); ax.axis("off")

    # verdict text
    ax = fig.add_axes([0.02, 0.01, 0.96, 0.22]); ax.axis("off")
    lines = [
        f"STOPS-ONLY VERDICT: viable, if the hinge has a little friction. Two printed lips between the cheeks catch the flat faces of the shank plug at "
        f"{a_lo:.0f}° and {a_hi:.0f}° (stance needs {p_lo:.0f}–{p_hi:.0f}°). Stop load ≤ {tq_ground / LIP_H:.0f} N on a 8 × 10 mm lip face (~{tq_ground / LIP_H / 80:.1f} MPa) — fine for PLA.",
        f"Why friction: in swing the pad's own weight gives only {tq_grav:.1f} mN·m about the pivot, while swinging the leg gives up to ~{tq_inert:.0f} mN·m (smoothed gait). "
        f"A free hinge would flap between the lips. An M3 nylock on a wave washer (~{PRELOAD:.0f} N) gives ~{tq_fric:.0f} mN·m: enough to hold the pad still in swing, "
        f"tiny next to the ~{tq_ground:.0f} mN·m the ground applies at the toe, so the ground still flattens it.",
        f"Behaviour: the pad keeps its lift-off angle through swing, so it is always toe-UP (never toe-down) and lands {land_up:.0f}° heel-first, then rolls flat. "
        f"The band version landed 16° toe-up; stops-only lands steeper but has no rubber to harden or snap in the cold (natural rubber stiffens below about −20 °C).",
        f"Honeycomb: option A keeps a closed {SKIN_A} mm skin under the Voronoi ribs, so the whole {A_out / 100:.0f} cm² still floats on the snow (24 g/cm²), and the deck is "
        f"{m_A:.1f} g vs {m_solid:.1f} g solid, while stiffer ({SKIN_A + RIB_H_A:.1f} mm deep section). Option B saves more ({m_B:.1f} g) but {open_frac * 100:.0f} % of the sole is open: "
        "loose snow falls through, which is what the shoe is meant to stop.",
        f"The lattice is generative: cells are {CELL_MIN:.0f} mm near the pivot, where the plate's bending is largest, growing to {CELL_MAX:.0f} mm at the edges "
        f"(seeds by dart throwing, Voronoi cells, ribs {RIB_W} mm = 3 printed lines). Prints sole-down with no supports in both options.",
        f"Parts: 1 printed snowshoe (sole + lattice + rim + cheeks + 2 lips) + existing M3 × 25 bolt + nylock + 1 M3 wave washer. Pivot now {PIVOT_UP:.0f} mm above the sole.",
    ]
    for i, t in enumerate(lines):
        ax.text(0.0, 0.98 - i * 0.165, "•  " + t, fontsize=8.8, va="top", transform=ax.transAxes, wrap=True)
    out = os.path.join(ROOT, "docs", "renders", "snowshoe_v2_preview.png")
    fig.savefig(out, dpi=100, bbox_inches="tight")
    print("\n".join(lines))
    print(f"cells {len(holes)}, open {open_frac:.2f}, mA {m_A:.1f} mB {m_B:.1f} msolid {m_solid:.1f}, grav {tq_grav:.2f} inert {tq_inert:.1f} fric {tq_fric:.0f}")
    print("wrote", out)


if __name__ == "__main__":
    main()
