"""Snowshoe foot geometry shared by the CAD builder and the analysis/animation scripts (no SolidWorks).

Pad frame: origin on the ankle pivot, u forward (toe), v up, w across (outboard +). The pad is flat in
this frame; build_parts places it at the ankle and turns it to ANKLE_BUILD (mid-stance).
The sole lattice is generative: Voronoi cells from seeds dart-thrown with a spacing that grows from
SHOE_CELL[0] at the pivot (where the plate's bending moment is largest) to SHOE_CELL[1] at the far edge.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import params as P  # noqa: E402

SOLID_HUB = (-12.0, 12.0, -9.5, 9.5)          # u0, u1, w0, w1: solid rib zone under the cheeks


def _tear(r_toe, r_tail, n=64):
    """Teardrop planform polygon (u, w), CCW, in the tail-centre frame."""
    d = P.SHOE_L - P.SHOE_R_TOE - P.SHOE_R_TAIL
    a = math.acos((P.SHOE_R_TOE - P.SHOE_R_TAIL) / d)
    pts = [(d + r_toe * math.cos(t), r_toe * math.sin(t)) for t in np.linspace(-a, a, n)]
    pts += [(r_tail * math.cos(t), r_tail * math.sin(t)) for t in np.linspace(a, 2 * math.pi - a, n)]
    return np.array(pts)


def _area_centroid(p):
    x, y = p[:, 0], p[:, 1]
    cr = x * np.roll(y, -1) - np.roll(x, -1) * y
    A = cr.sum() / 2
    return abs(A), (x + np.roll(x, -1)) @ cr / (6 * A)


_A, _XC = _area_centroid(_tear(P.SHOE_R_TOE, P.SHOE_R_TAIL, 400))
U_TAILC = -(_XC + P.SHOE_PIVOT_AHEAD * P.SHOE_L)          # heel-circle centre u (pivot at u = 0)
U_TOEC = U_TAILC + P.SHOE_L - P.SHOE_R_TOE - P.SHOE_R_TAIL  # toe-circle centre u
U_HEEL, U_TOE = U_TAILC - P.SHOE_R_TAIL, U_TOEC + P.SHOE_R_TOE
AREA = _A                                                   # sole area, mm^2
V_SOLE = -P.SHOE_PIVOT_UP


def outline(inset=0.0, n=64):
    return _tear(P.SHOE_R_TOE - inset, P.SHOE_R_TAIL - inset, n) + (U_TAILC, 0.0)


# ------------------------------------------------------------------ Voronoi lattice
def _clip(poly, a, b):
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


def _signed_area(p):
    return np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]) / 2


def _offset(poly, d):
    """Convex polygon offset inward by d; None if it collapses."""
    ccw = _signed_area(poly) > 0
    lines = []
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        t = q - p
        L = np.linalg.norm(t)
        if L < 1e-6:
            continue
        nrm = np.array([-t[1], t[0]]) / L * (1 if ccw else -1)
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
    a = _signed_area(out)
    return out if (a > 0) == ccw and abs(a) > 6.0 else None


def _simplify(p, min_edge=1.2, min_turn=12.0):
    """Drop near-collinear and very close vertices so the sketch gets clean, filletable corners."""
    pts = [q for q in p]
    changed = True
    while changed and len(pts) > 3:
        changed = False
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            v1, v2 = b - a, c - b
            turn = abs(math.degrees(math.atan2(v1[0] * v2[1] - v1[1] * v2[0], v1 @ v2)))
            if np.linalg.norm(v2) < min_edge or turn < min_turn:
                del pts[i if turn < min_turn else (i + 1) % len(pts)]
                changed = True
                break
    return np.array(pts)


def lattice_cells(seed=7):
    """Cell openings (pockets above the skin) in the pad frame (u, w), CCW polygons."""
    inner = outline(P.SHOE_RIM_W, 48)
    rng = np.random.default_rng(seed)
    rmax = np.max(np.hypot(inner[:, 0], inner[:, 1]))
    c0, c1 = P.SHOE_CELL
    sp = lambda q: c0 + (c1 - c0) * min(1.0, math.hypot(*q) / rmax)     # noqa: E731
    u0, u1, w0, w1 = inner[:, 0].min(), inner[:, 0].max(), inner[:, 1].min(), inner[:, 1].max()

    def inside(q):
        n = len(inner)
        return all((inner[(i + 1) % n][0] - inner[i][0]) * (q[1] - inner[i][1]) -
                   (inner[(i + 1) % n][1] - inner[i][1]) * (q[0] - inner[i][0]) >= 0 for i in range(n))
    seeds = []
    for _ in range(40000):
        q = np.array([rng.uniform(u0, u1), rng.uniform(w0, w1)])
        if inside(q) and all(np.hypot(*(q - s)) >= 0.5 * (sp(q) + sp(s)) for s in seeds):
            seeds.append(q)
    seeds = np.array(seeds)
    cells = []
    for i, q in enumerate(seeds):
        c = inner.copy()
        for j in np.argsort(np.hypot(*(seeds - q).T))[1:16]:
            c = _clip(c, (q + seeds[j]) / 2, seeds[j] - q)
            if len(c) < 3:
                break
        if len(c) < 3:
            continue
        h = _offset(c, P.SHOE_RIB_W / 2)
        if h is None:
            continue
        h = _simplify(h)
        if len(h) < 3 or abs(_signed_area(h)) < 6.0:
            continue
        if (h[:, 0].max() > SOLID_HUB[0] and h[:, 0].min() < SOLID_HUB[1] and
                h[:, 1].max() > SOLID_HUB[2] and h[:, 1].min() < SOLID_HUB[3]):
            continue
        cells.append(h if _signed_area(h) > 0 else h[::-1])
    return cells


def safe_fillet(poly, r):
    """Largest fillet <= r that fits every corner of the polygon."""
    best = r
    n = len(poly)
    for i in range(n):
        a, b, c = poly[i - 1], poly[i], poly[(i + 1) % n]
        e1, e2 = np.linalg.norm(a - b), np.linalg.norm(c - b)
        cosang = np.clip(((a - b) @ (c - b)) / (e1 * e2), -1, 1)
        th = math.acos(cosang)
        best = min(best, 0.45 * min(e1, e2) * math.tan(th / 2))
    return best


# ------------------------------------------------------------------ cheeks and lips (pad frame, side view u-v)
def rot(p, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    p = np.atleast_2d(p)
    return np.column_stack([p[:, 0] * c - p[:, 1] * s, p[:, 0] * s + p[:, 1] * c])


def plug_dirs(foot_up, foot_across, alpha):
    """Plug axis and across-direction in the pad frame when the shank is at alpha (deg) in the pad frame."""
    return rot(foot_up, alpha)[0], rot(foot_across, alpha)[0]


def lips(foot_up, foot_across, plug_w):
    """Two lip blocks (u, v quads): front lip catches the plug's +x face at ANKLE_LIM[0], rear lip its
    -x face at ANKLE_LIM[1]; contact centred ANKLE_LIP_H above the pivot."""
    out = []
    for ang, sgn in ((P.ANKLE_LIM[0], 1), (P.ANKLE_LIM[1], -1)):
        uu, nn = plug_dirs(np.array(foot_up), sgn * np.array(foot_across), ang)
        pc = P.ANKLE_LIP_H * uu + plug_w / 2 * nn
        out.append(np.array([pc - uu * 3.0, pc + nn * 5.0 - uu * 3.0, pc + nn * 5.0 + uu * 3.0, pc + uu * 3.0]))
    return out


def cheek_profile(lip_quads):
    """Convex hull of the cheek footing on the skin, a ring round the pivot and the lip blocks."""
    pts = [(-14.0, V_SOLE + P.SHOE_SKIN), (14.0, V_SOLE + P.SHOE_SKIN)]
    pts += [(6.0 * math.cos(t), 6.0 * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 24, endpoint=False)]
    for q in lip_quads:
        pts += [tuple(v) for v in q]
    pts = sorted(set((round(a, 4), round(b, 4)) for a, b in pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def keels():
    """Keel side profiles: list of (w, [(u, v), ...]) polygons in the pad frame. Height tapers linearly from
    SHOE_KEEL_H at the hub to the rim height at the ends (the toe/heel bending moment falls to zero there)."""
    inner = outline(P.SHOE_RIM_W, 200)
    v0 = V_SOLE + P.SHOE_SKIN
    hi, lo = V_SOLE + P.SHOE_KEEL_H, V_SOLE + P.SHOE_RIM_H
    out = []
    for w in (-P.SHOE_KEEL_Z, 0.0, P.SHOE_KEEL_Z):
        us = [inner[i][0] + (inner[i + 1][0] - inner[i][0]) * (w - inner[i][1]) / (inner[i + 1][1] - inner[i][1])
              for i in range(len(inner) - 1)
              if (inner[i][1] - w) * (inner[i + 1][1] - w) <= 0 and inner[i + 1][1] != inner[i][1]]
        u_heel, u_toe = min(us) + 0.5, max(us) - 0.5
        if w == 0.0:
            segs = [(u_heel, -P.SHOE_KEEL_HUB), (P.SHOE_KEEL_HUB, u_toe)]
        else:
            segs = [(u_heel, u_toe)]
        for a, b in segs:
            ends = (a, b)
            hub = min(max(0.0, a), b)                      # highest at the point nearest the pivot
            top = [(a, lo if abs(a) > abs(hub) + 1 else hi), (hub, hi), (b, lo if abs(b) > abs(hub) + 1 else hi)]
            top = [t for i, t in enumerate(top) if i == 0 or abs(t[0] - top[i - 1][0]) > 0.5]
            out.append((w, [(ends[0], v0)] + top + [(ends[1], v0)]))
    return out


def side_profile():
    """Pad side outline (u, v) for drawings: sole, toe bevel, rim top."""
    v0, top = V_SOLE, V_SOLE + P.SHOE_RIM_H
    return np.array([(U_HEEL + 3, v0), (U_TOE - P.SHOE_TOE_RUN, v0), (U_TOE, v0 + P.SHOE_TOE_LIFT), (U_TOE, top),
                     (U_HEEL, top), (U_HEEL, v0 + 3)])
