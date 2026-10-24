"""Preview of large-cutout options for the thigh walls and the bent shank (no SolidWorks).
Each option puts 2-4 big pockets down the middle of each link, inside the same keep-outs the
CAD uses (thigh: 4 mm belt guard band + knee/idler rings; shank: 4.5 mm edge flanges), with
WEB mm walls between neighbouring cutouts. Writes docs/renders/cutout_options.png and prints
the material removed per option.  Run:  python analysis/cutout_options.py [triangles]
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "cad"))
import params as P  # noqa: E402
import build_parts as B  # noqa: E402

WEB = 5.0                     # wall between neighbouring cutouts, mm
EDGE = 1.0                    # cutout edge to the keep-out line, mm
G_PER_MM3 = 1.24e-3 * 0.75    # PLA, printed-infill factor as in skeleton_check

# --------------------------------------------------------------- the two links in local (s, n) coordinates
TH = dict(s0=44.0, s1=119.0, frame=lambda s: ((0.0, -s), (1.0, 0.0)),
          hw=lambda s: B.span_x(-s) - 0.7 - 4.0 - EDGE, t=2 * P.THIGH_WALL_THIN)    # both thigh walls
SH = dict(s0=17.0, s1=B.SH_LEN - 3.0, frame=B.shank_frame,
          hw=lambda s: B.shank_pocket_hw(s) - EDGE, t=P.SK_WEB,                       # cuts the 3 mm web
          web=P.SK_STRUT)            # 4 mm struts in the shank web (the edge flanges carry the bending)


def to_xy(link, s, n):
    c, nrm = link["frame"](s)
    return (c[0] + n * nrm[0], c[1] + n * nrm[1])


def shape(link, sa, sb, kind, flip=False, m=60):
    """One cutout between sa and sb along the link, as a closed xy polygon."""
    pts = []
    for i in range(m):
        th = 2 * math.pi * i / m
        cu, cv = math.cos(th), math.sin(th)
        if kind in ("oval", "rrect", "slot", "diamond"):
            p = {"oval": 2.0, "rrect": 5.0, "slot": 8.0, "diamond": 1.3}[kind]
            u = math.copysign(abs(cu) ** (2 / p), cu)
            v = math.copysign(abs(cv) ** (2 / p), cv)
            scale = 1.0
        elif kind == "tear":                       # wide end low (where the link is wider) unless flipped
            u, v = cu, cv
            k = (1 + u) / 2 if not flip else (1 - u) / 2   # u=-1 is the top (sa): narrow there
            scale = 0.3 + 0.7 * k ** 0.7
        s = (sa + sb) / 2 + u * (sb - sa) / 2
        w = min(link["hw"](s), link["hw"]((sa + sb) / 2 + (sb - sa) / 2 * u))
        pts.append(to_xy(link, s, v * w * scale))
    return pts


def _truss_nodes(link, ss):
    return [to_xy(link, sv, link.get("phase", 1) * (-1) ** i * (link["hw"](sv) + link.get("web", WEB) / 2)) for i, sv in enumerate(ss)]


def _hole_area(tri, d):
    """Size measure of the triangle offset inward by d: its inscribed radius squared (0 if it vanishes)."""
    a = math.dist(tri[1], tri[2]); b = math.dist(tri[0], tri[2]); c = math.dist(tri[0], tri[1])
    A = area(tri)
    r = 2 * A / (a + b + c)
    return 0.0 if r <= d else (r - d) ** 2          # inscribed circle of the hole: how "fat" it looks


def _size_cost(ar):
    """Holes within ~15 % of each other in inscribed size (they read as one family), then as large as possible."""
    lo, hi = min(ar), max(ar)
    return -lo + 5.0 * max(0.0, hi - 1.3 * lo)       # ar are radii squared: 1.3 ~ 14 % in radius


def truss_spacing(link, n, equal=True):
    """Node positions along the link. equal=False: evenly spaced. equal=True: the interior nodes are
    moved (coordinate search) so the holes come out within ~15 % of each other in size and as large as possible, so narrow-end holes get more length."""
    ss = [link["s0"] + (link["s1"] - link["s0"]) * i / (n + 1) for i in range(n + 2)]
    if not equal:
        return ss

    def cost(v):
        if any(v[i + 1] - v[i] < 6.0 for i in range(len(v) - 1)):
            return 1e9
        nd = _truss_nodes(link, v)
        ar = [_hole_area(nd[i:i + 3], link.get("web", WEB) / 2) for i in range(n)]
        return _size_cost(ar)
    # n = 3: try every interior-node triple on a 1 mm grid (the curved shank traps a local search)
    import functools
    import itertools
    nd = functools.lru_cache(maxsize=None)(lambda sv, k: to_xy(link, sv, k * (link["hw"](sv) + link.get("web", WEB) / 2)))
    grid = [link["s0"] + float(i) for i in range(1, int(link["s1"] - link["s0"]))]
    best = (cost(ss), ss)
    for s1, s2, s3 in itertools.combinations(grid, 3):
        v = [link["s0"], s1, s2, s3, link["s1"]]
        if min(v[i + 1] - v[i] for i in range(4)) < 6.0:
            continue
        pts = [nd(sv, link.get("phase", 1) * (-1) ** i) for i, sv in enumerate(v)]
        c = _size_cost([_hole_area(pts[i:i + 3], link.get("web", WEB) / 2) for i in range(3)])
        if c < best[0]:
            best = (c, v)
    ss = best[1]
    return ss


def tri_truss(link, n, equal=False):
    """n triangles pointing alternately to each side (Warren truss): bases on the side walls, uniform
    WEB-wide diagonal walls between neighbours (outer triangles shrunk by WEB/2 about their incentre)."""
    nodes = _truss_nodes(link, truss_spacing(link, n, equal))
    out = []
    for i in range(n):
        h = B._shrink_tri(nodes[i:i + 3], link.get("web", WEB) / 2, 2.5, 0.0)
        if h:
            out.append(sample(h))
    return out


def tri_stack(link, n):
    """n triangles stacked down the link, alternately base-up and apex-up, WEB mm apart. The first one
    points into the narrow end of the link (thigh: apex up at the hip; shank: base up at the knee)."""
    L = (link["s1"] - link["s0"] - (n - 1) * WEB) / n
    first_base_up = link["hw"](link["s0"]) > link["hw"](link["s1"])
    out = []
    for i in range(n):
        sa = link["s0"] + i * (L + WEB)
        sb = sa + L
        if (i % 2 == 0) == first_base_up:      # base up, apex down
            pts = [to_xy(link, sa, -link["hw"](sa)), to_xy(link, sb, 0.0), to_xy(link, sa, link["hw"](sa))]
        else:               # apex up, base down
            pts = [to_xy(link, sa, 0.0), to_xy(link, sb, -link["hw"](sb)), to_xy(link, sb, link["hw"](sb))]
        out.append(sample(B.rounded_poly(pts, 2.5)))
    return out


def layout(link, n, kind, alt_flip=False):
    if kind == "tri_truss":
        return tri_truss(link, n)
    if kind == "tri_truss_eq":
        return tri_truss(link, n, equal=True)
    if kind == "tri_stack":
        return tri_stack(link, n)
    L = (link["s1"] - link["s0"] - (n - 1) * WEB) / n
    return [shape(link, link["s0"] + i * (L + WEB), link["s0"] + i * (L + WEB) + L, kind,
                  flip=(alt_flip and i % 2 == 1)) for i in range(n)]


def area(poly):
    return abs(sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                   for i in range(len(poly)))) / 2


def sample(loop):
    """Sample a swlib loop (current CAD holes) as a polygon."""
    out = []
    for sg in loop[1]:
        if sg[0] == "line":
            out += [sg[1]]
        elif sg[0] == "spline":
            out += list(sg[1])
        else:
            c, a, b, d = sg[1], sg[2], sg[3], sg[4]
            t0 = math.atan2(a[1] - c[1], a[0] - c[0]); t1 = math.atan2(b[1] - c[1], b[0] - c[0]); r = math.dist(c, a)
            if d > 0 and t1 < t0: t1 += 2 * math.pi
            if d < 0 and t1 > t0: t1 -= 2 * math.pi
            out += [(c[0] + r * math.cos(t0 + (t1 - t0) * k / 12), c[1] + r * math.sin(t0 + (t1 - t0) * k / 12)) for k in range(13)]
    return out


def current():
    th = [sample(h) for h in B.thigh_wall_holes()]
    sh = [sample(h) for h in B.shank_holes()]
    return th, sh


OPTIONS = {
    "Now (CAD)": None,
    "A: 2 long slots": dict(th=(2, "slot"), sh=(2, "slot")),
    "B: 3 rounded rectangles": dict(th=(3, "rrect"), sh=(3, "rrect")),
    "C: 4 ovals": dict(th=(4, "oval"), sh=(3, "oval")),
    "D: 3 diamonds": dict(th=(3, "diamond"), sh=(3, "diamond")),
    "E: 2 teardrops": dict(th=(2, "tear"), sh=(2, "tear")),
}

FINAL_OPTIONS = {
    "B: 3 rounded rectangles (preview)": dict(th=(3, "rrect"), sh=(3, "rrect")),
    "Chosen, from the CAD code:\nshank cutouts smaller": None,
}

TRI_OPTIONS = {
    "B: 3 rounded rectangles (ref.)": dict(th=(3, "rrect"), sh=(3, "rrect")),
    "B-T1: 3 triangles, alternating\nsides (uniform diagonal walls)": dict(th=(3, "tri_truss"), sh=(3, "tri_truss")),
    "B-T1 equalised: spacing tuned\nso each link's 3 holes look the same size": dict(th=(3, "tri_truss_eq"), sh=(3, "tri_truss_eq")),
}


def outline_poly(loop):
    return sample(loop)


def main():
    tri = "triangles" in sys.argv[1:]
    final = "final" in sys.argv[1:]
    options = FINAL_OPTIONS if final else TRI_OPTIONS if tri else OPTIONS
    fig, axs = plt.subplots(1, len(options), figsize=(3.6 * len(options), 9.5))
    thigh = outline_poly(B.thigh_outline())
    shank = outline_poly(B._band(0.0, B.SH_LEN, B.shank_hw))
    pocket = outline_poly(B._band(P.SK_KNEE_SOLID_R + B.shank_pocket_hw(P.SK_KNEE_SOLID_R + 8.0),
                                  B.SH_LEN - B.shank_pocket_hw(B.SH_LEN), B.shank_pocket_hw))
    plug = outline_poly(B.ax_rect(-B.PLUG_W / 2, B.FOOT_TOP_A - B.PLUG_DEPTH, B.PLUG_W / 2, B.SH_END_A - 3.0))
    rows = []
    for ax_, (name, opt) in zip(axs, options.items()):
        if opt is None:
            th, sh = current()
        else:
            th = layout(TH, *opt["th"])
            sh = layout(SH, *opt["sh"])
        g = sum(area(p) for p in th) * TH["t"] * G_PER_MM3 + sum(area(p) for p in sh) * SH["t"] * G_PER_MM3
        rows.append((name, len(th), len(sh), g))
        for poly, fc in ((thigh, "#c9ccd3"), (shank, "#b5b9c2"), (plug, "#b5b9c2")):
            ax_.add_patch(Polygon(poly, closed=True, fc=fc, ec="#33363d", lw=1.0))
        ax_.add_patch(Polygon(pocket, closed=True, fc="none", ec="#6b7080", lw=0.7, ls="--"))
        for c in (B.KNEE, (0.0, 0.0)):
            ax_.add_patch(plt.Circle(c, 6.0 if c == B.KNEE else 7.6, fc="white", ec="#33363d", lw=0.8))
        ax_.add_patch(plt.Circle(B.FOOT, B.FOOT_R, fc="#9aa0aa", ec="#33363d", lw=1.0))
        for poly in th + sh:
            ax_.add_patch(Polygon(poly, closed=True, fc="#20232a", ec="#20232a"))
        ax_.set_title(f"{name}\n−{g:.1f} g", fontsize=10)
        ax_.set_xlim(-38, 38); ax_.set_ylim(-275, 30); ax_.set_aspect("equal"); ax_.axis("off")
    fig.suptitle("Large-cutout options (side view, hip at top). Dark = through-cut; dashed = shank web pocket. " +
                 (f"Chosen: walls between cutouts thigh {P.SK_CUT_WALL_THIGH:.0f} mm, shank {P.SK_CUT_WALL_SHANK:.0f} mm; shank cutouts "
                  f"{P.SK_CUT_INSET_SHANK} mm inside the web pocket; thigh keeps its 4 mm belt guard bands." if final else
                  f"Walls between cutouts: thigh {WEB:.0f} mm, shank web {P.SK_STRUT:.0f} mm (inside its 4.5 mm edge flanges); "
                  "thigh keeps its 4 mm belt guard bands."), fontsize=10)
    out = os.path.join(ROOT, "docs", "renders", "cutout_final.png" if final else "cutout_options_triangles.png" if tri else "cutout_options.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    for r in rows:
        print(f"{r[0].replace(chr(10), ' '):60s} thigh {r[1]}  shank {r[2]}  material removed ≈ {r[3]:.1f} g")
    print("wrote", out)


if __name__ == "__main__":
    main()
