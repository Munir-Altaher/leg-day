"""Shared snow-leg geometry used by both the CAD (cad/build_parts.py) and the analyses.

Body frame as in params.py: origin = hip axis ∩ ab/ad axis, X forward, Y up, Z outboard.
Home pose q0 = q1 = q2 = 0 (shank vertical) with ankle angle 0 = sole perpendicular to the shank.

Ankle convention: alpha = angle of the shank axis (pivot -> knee) measured in the foot frame,
CCW from the sole normal (+Y of the foot), seen from +Z. Sole level in the world <=> alpha = phi,
where phi = q1 + q2 is the shank's CCW angle from vertical. Foot pitch (toe up +) = phi - alpha.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "analysis"))
import params as P  # noqa: E402

L1, L2 = P.L1, P.L2
KNEE = (0.0, -L1)
PIVOT = (0.0, -L1 - L2)                      # ankle pivot at q = 0 (L2 = knee -> ankle pivot)
ZC = P.L_ABAD                                # leg plane (shank / foot mid-plane), body z


# ------------------------------------------------------------------ planform (foot frame: pivot at x = 0)
def teardrop_props(Lf=P.FOOT_L, Wf=P.FOOT_W, rt=P.FOOT_R_TAIL):
    r0, r1 = Wf / 2, rt
    d = Lf - r0 - r1
    a = math.acos((r0 - r1) / d)
    pts = [(d + r0 * math.cos(t), r0 * math.sin(t)) for t in np.linspace(-a, a, 400)]
    pts += [(r1 * math.cos(t), r1 * math.sin(t)) for t in np.linspace(a, 2 * math.pi - a, 400)]
    pts = np.array(pts)
    x, y = pts[:, 0], pts[:, 1]
    xs, ys = np.roll(x, -1), np.roll(y, -1)
    cr = x * ys - xs * y
    A = cr.sum() / 2
    cx = ((x + xs) * cr).sum() / (6 * A)
    return abs(A), np.hypot(xs - x, ys - y).sum(), cx + r1      # area, perimeter, centroid from the tail tip


AREA, PERIM, CENTROID_FROM_TAIL = teardrop_props()
PIVOT_FROM_TAIL = CENTROID_FROM_TAIL + P.PIVOT_AHEAD_FRAC * P.FOOT_L
X_HEEL = -PIVOT_FROM_TAIL                      # heel tip, relative to the pivot
X_TOE = P.FOOT_L - PIVOT_FROM_TAIL             # toe tip
X_TAIL_C = X_HEEL + P.FOOT_R_TAIL              # tail-circle centre
X_TOE_C = X_TOE - P.FOOT_R_TOE                 # toe-circle centre
# vertical levels relative to the pivot (foot frame)
Y_DECK_TOP = -P.PIVOT_ABOVE_DECK
Y_DECK_BOT = Y_DECK_TOP - P.DECK_T
Y_LUG_TIP = Y_DECK_BOT - P.LUG_H
Y_PAD_TIP = Y_LUG_TIP - P.PAD_PROUD
ANKLE_H = -Y_PAD_TIP                           # pivot above the ground contact


def half_width(x):
    """Planform half-width (z) at foot-frame x (pivot = 0); 0 outside."""
    r0, r1 = P.FOOT_R_TOE, P.FOOT_R_TAIL
    d = X_TOE_C - X_TAIL_C
    a = math.acos((r0 - r1) / d)
    # tangent points
    xt0, xt1 = X_TAIL_C + r1 * math.cos(a), X_TOE_C + r0 * math.cos(a)
    if x <= X_HEEL or x >= X_TOE:
        return 0.0
    if x < xt0:
        return math.sqrt(max(r1 ** 2 - (x - X_TAIL_C) ** 2, 0))
    if x > xt1:
        return math.sqrt(max(r0 ** 2 - (x - X_TOE_C) ** 2, 0))
    y0, y1 = r1 * math.sin(a), r0 * math.sin(a)
    return y0 + (y1 - y0) * (x - xt0) / (xt1 - xt0)


def foot_outline_pts(n=200):
    """Sole outline points (foot frame x, lateral offset) for drawing / clearance."""
    xs = np.linspace(X_HEEL + 1e-3, X_TOE - 1e-3, n)
    top = [(x, half_width(x)) for x in xs]
    return np.array(top + [(x, -h) for x, h in reversed(top)])


# ------------------------------------------------------------------ ankle range from the snow stance
def stance_phis(frac=None, step=None):
    from leg_model import ik
    frac = P.SNOW_STANCE_FRAC if frac is None else frac
    step = P.SNOW_STEP if step is None else step
    H = frac * (L1 + L2) / 1000
    l = P.L_ABAD / 1000
    out = []
    for x in np.linspace(-step / 2, step / 2, 25):
        q = ik(np.array([x, -H, l]), L1 / 1000, L2 / 1000, l)
        out.append(math.degrees(q[1] + q[2]))
    return min(out), max(out)


PHI_ST = stance_phis()
ALPHA_MIN = PHI_ST[0] - P.ANKLE_MARGIN          # toe-up stop
ALPHA_MAX = PHI_ST[1] + P.ANKLE_MARGIN          # toe-down stop
ALPHA_NEUTRAL = (PHI_ST[0] + PHI_ST[1]) / 2     # centring springs' neutral (sole level at mid-stance)


# ------------------------------------------------------------------ ankle stops (cross bolts with Ø5 sleeves)
def shank_halfwidth_at(r_from_pivot):
    """Shank half-width at a distance r up from the pivot (stadium knee R13 -> tip R9)."""
    rk, rt = P.SHANK_W_KNEE / 2, P.SHANK_TIP_R
    return rt + (rk - rt) * r_from_pivot / L2


def stop_points():
    R = P.STOP_R
    w = shank_halfwidth_at(R)
    delta = math.degrees(math.asin((w + P.STOP_SLEEVE_D / 2 + 0.3) / R))
    pts = {}
    for name, ang in (("toe_up", ALPHA_MIN - delta), ("toe_down", ALPHA_MAX + delta)):
        a = math.radians(ang)
        pts[name] = (-R * math.sin(a), R * math.cos(a))       # CCW from +Y
    return pts, delta


STOPS, STOP_DELTA = stop_points()


def foot_low_point(psi_deg):
    """Lowest point of the sole below the pivot for a foot pitch psi (toe up +)."""
    s, c = math.sin(math.radians(psi_deg)), math.cos(math.radians(psi_deg))
    ys = [x * s + Y_PAD_TIP * c for x in (X_HEEL, X_TOE, 0.0)]
    return min(ys)


# ------------------------------------------------------------------ knee stops (fins on the shank hitting the thigh rim)
def thigh_outline_poly(n=120):
    """Thigh outline (thigh frame = body frame at q1 = 0), stadium hip R -> knee R."""
    from math import acos, atan2, cos, sin
    r0, r1 = P.THIGH_R_HIP, P.THIGH_R_KNEE
    c0, c1 = (0.0, 0.0), KNEE
    d = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
    ang = atan2(c1[1] - c0[1], c1[0] - c0[0])
    a = acos((r0 - r1) / d)
    pts = [(c1[0] + r1 * cos(t), c1[1] + r1 * sin(t)) for t in np.linspace(ang - a, ang + a, n)]
    pts += [(c0[0] + r0 * cos(t), c0[1] + r0 * sin(t)) for t in np.linspace(ang + a, ang - a + 2 * math.pi, n)]
    return np.array(pts)


def _dist_to_poly(p, poly):
    from matplotlib.path import Path
    inside = Path(poly).contains_point(p)
    a, b = poly, np.roll(poly, -1, 0)
    ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / (ab * ab).sum(1), 0, 1)
    d = np.min(np.hypot(*(a + ab * t[:, None] - p).T))
    return -d if inside else d


def knee_fin_contact(p_shank, r_fin, q2s):
    """First q2 (in the given sweep order) where a fin at shank-frame point p_shank touches the thigh outline."""
    poly = thigh_outline_poly()
    for q2 in q2s:
        c, s = math.cos(math.radians(q2)), math.sin(math.radians(q2))
        x = KNEE[0] + p_shank[0] * c - p_shank[1] * s
        y = KNEE[1] + p_shank[0] * s + p_shank[1] * c
        if _dist_to_poly(np.array([x, y]), poly) < r_fin + 0.3:
            return q2
    return None


KNEE_STOP_MAX = P.Q2_LIM[1] + 2.0     # physical stops 2 deg outside the firmware limits
KNEE_STOP_MIN = P.Q2_LIM[0] - 2.0
FIN_HALF = 6.0                         # knee stop posts: M5 screw in an insert with a Ø12 sleeve (r = 6)


def design_knee_fins():
    """Upper fin on the shank axis at distance d below the knee; lower fin on a horn at radius
    R_h, angle delta CW from the shank axis. Solved so contact happens at the stop angles."""
    up = None
    for d in np.arange(40.0, 120.0, 0.5):
        q = knee_fin_contact((0.0, -d), FIN_HALF, np.arange(P.Q2_LIM[1], 179.0, 0.25))
        if q is not None and q >= KNEE_STOP_MAX:
            up = (d, q)
            break
    lo = None
    Rh = P.THIGH_R_KNEE + FIN_HALF + 2.0
    for dl in np.arange(60.0, 175.0, 0.25):
        # point dl degrees CW from the shank axis (shank axis = -Y in the shank frame)
        p = (-Rh * math.sin(math.radians(dl)), -Rh * math.cos(math.radians(dl)))
        q = knee_fin_contact(p, FIN_HALF, np.arange(P.Q2_LIM[0], -60.0, -0.25))
        if q is None or q > KNEE_STOP_MIN:
            continue
        if knee_fin_contact(p, FIN_HALF, np.arange(P.Q2_LIM[0], P.Q2_LIM[1] + 0.1, 1.0)) is not None:
            continue
        if lo is None or q > lo[3]:
            lo = (Rh, dl, p, q)
    return up, lo


def summary():
    d = dict(area_cm2=AREA / 100, perim_mm=PERIM, pivot_from_tail=PIVOT_FROM_TAIL, x_heel=X_HEEL, x_toe=X_TOE,
             ankle_h=ANKLE_H, phi_stance=PHI_ST, alpha_min=ALPHA_MIN, alpha_max=ALPHA_MAX, alpha_neutral=ALPHA_NEUTRAL,
             stops=STOPS, stop_delta=STOP_DELTA)
    return d


if __name__ == "__main__":
    for k, v in summary().items():
        print(k, v)
    up, lo = design_knee_fins()
    print("knee fin up (d below knee, contact q2):", up)
    print("knee fin low (R, deg CW, point, contact q2):", lo)
