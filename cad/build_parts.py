"""Route A generator: builds every part in SolidWorks from params.py.
Run:  python cad/build_parts.py [part ...]
All parts are modelled in the body frame at q = 0 (leg hanging straight down),
so components inserted at the same placement assemble in their home pose.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import params as P  # noqa: E402
from swlib import Part, circle, stadium, rounded_poly, rect, slot, bolt_circle, teardrop  # noqa: E402

PARTS = os.path.join(HERE, "parts")
STL = os.path.join(HERE, "stl")
zc = P.zc
L1, L2 = P.L1, P.L2
KNEE = (0.0, -L1)
FOOT = (0.0, -L1 - L2)
M25, M3, M5 = P.CLR["M2.5"], P.CLR["M3"], P.CLR["M5"]


def hexagon(c, af, rot=0.0):
    r = af / math.sqrt(3)
    pts = [(c[0] + r * math.cos(math.radians(rot + 60 * i)), c[1] + r * math.sin(math.radians(rot + 60 * i))) for i in range(6)]
    return ("path", [("line", pts[i], pts[(i + 1) % 6]) for i in range(6)])


def _shrink_tri(tri, d, fillet, min_hole):
    """Triangle offset inward by d (= scaled about its incentre), corners rounded; None if the hole's
    inscribed circle would be under min_hole."""
    a = math.dist(tri[1], tri[2]); b = math.dist(tri[0], tri[2]); c = math.dist(tri[0], tri[1])
    s = (a + b + c) / 2
    area = abs((tri[1][0] - tri[0][0]) * (tri[2][1] - tri[0][1]) - (tri[2][0] - tri[0][0]) * (tri[1][1] - tri[0][1])) / 2
    r = area / s
    if 2 * (r - d) < min_hole:
        return None
    ic = ((a * tri[0][0] + b * tri[1][0] + c * tri[2][0]) / (2 * s), (a * tri[0][1] + b * tri[1][1] + c * tri[2][1]) / (2 * s))
    k = (r - d) / r
    shr = [(ic[0] + (p[0] - ic[0]) * k, ic[1] + (p[1] - ic[1]) * k) for p in tri]
    return rounded_poly(shr, min(fillet, (r - d) * 0.6))


def stacked_holes(s0, s1, n, frame, hw_fn, strut, fillet, min_hole, side=1):
    """n copies of one rounded triangle patterned down a link (the design review: every hole the same
    shape as the top one). In the link's local frame, frame(s) -> (centre-line point, unit normal) at
    distance s along it, each triangle has its long edge at +side*hw and its apex at -side*hw.
    Neighbours share long-edge corners and are shrunk by strut/2, so every strut is >= strut wide.
    Each corner sits at the narrowest half-width that corner meets in any copy, so the copies are identical
    (in the local frame) and as large as the link allows."""
    H = (s1 - s0) / n
    corners = [(0.0, side), (H, side), (H / 2, -side)]          # (offset down the copy, which flange)
    w = [min(hw_fn(s0 + i * H + o) for i in range(n)) + strut / 2 for o, _ in corners]
    holes = []
    for i in range(n):
        tri = []
        for (o, k), wk in zip(corners, w):
            c, nrm = frame(s0 + i * H + o)
            tri.append((c[0] + k * wk * nrm[0], c[1] + k * wk * nrm[1]))
        h = _shrink_tri(tri, strut / 2, fillet, min_hole)
        if h:
            holes.append(h)
    return holes


def rect_cutouts(s0, s1, n, frame, hw_fn, wall, r):
    """n rounded quadrilaterals stacked down a link between s0 and s1, `wall` mm apart. In the link's
    local frame, frame(s) -> (centre-line point, unit normal), each one spans +-hw_fn(s) at its two
    ends, so it follows the link's taper (and its bend: corners sit on the curved centre line)."""
    L = (s1 - s0 - (n - 1) * wall) / n
    out = []
    for i in range(n):
        sa, sb = s0 + i * (L + wall), s0 + i * (L + wall) + L
        q = []
        for sv, k in ((sa, -1), (sb, -1), (sb, 1), (sa, 1)):
            c, nrm = frame(sv)
            q.append((c[0] + k * hw_fn(sv) * nrm[0], c[1] + k * hw_fn(sv) * nrm[1]))
        out.append(rounded_poly(q, r))
    return out


def thigh_outline():
    return stadium((0, 0), P.THIGH_R_HIP, KNEE, P.THIGH_R_KNEE)


def channel_outline():
    return stadium((0, 0), P.THIGH_CH_R_HIP, KNEE, P.THIGH_CH_R_KNEE)


def span_x(y):
    return P.PD_HIP / 2 + (P.PD_KNEE - P.PD_HIP) / 2 * (-y) / L1


def outer_hw(y):
    return P.THIGH_R_HIP + (P.THIGH_R_KNEE - P.THIGH_R_HIP) * (-y) / L1


HIP_BOSSES = [(14.0 * math.cos(math.radians(a)), 14.0 * math.sin(math.radians(a))) for a in (30, 150, 270)]
EARS = [(s * (outer_hw(-80) + 1.5), -80.0) for s in (-1, 1)] + [(s * (P.THIGH_R_KNEE + 2.5), -L1) for s in (-1, 1)]
IDLER_SLOT = slot((5.0, P.IDLER_Y), (5.0 + P.IDLER_SLOT_TRAVEL, P.IDLER_Y), P.CLR[P.IDLER_BOLT] / 2)


THIGH_CUT_S = (45.0, 116.0)          # below the idler slot (y -40); stops 2 mm short of the knee ring (r 12)


def thigh_cut_hw(s):
    """Half-width open to the thigh cutouts at distance s below the hip: belt span - 0.7 - 4 mm guard band - 1."""
    return span_x(-s) - 0.7 - 4.0 - 1.0


def thigh_wall_holes():
    if not P.SKELETON:
        return []
    down = lambda s: ((0.0, -s), (1.0, 0.0))                     # noqa: E731  straight link, normal +x
    return rect_cutouts(*THIGH_CUT_S, P.SK_CUTS, down, thigh_cut_hw, P.SK_CUT_WALL_THIGH, P.SK_CUT_R)


def ears(p, z0, z1, hole_d):
    for e in EARS:
        p.slab("XY", z0, z1, [circle(e, 4.5), circle(e, hole_d / 2)])


# ------------------------------------------------------------------ thigh (outboard half: wall + rim)
def thigh_outer():
    p = Part("thigh_outer")
    z0, z1 = zc(P.Z_THIGH_OUT[0]), zc(P.Z_THIGH_OUT[1])
    zt = z1 - P.THIGH_WALL_THIN                     # wall thinned from the channel side
    c0 = zc(P.Z_CHANNEL[0])
    loops = [thigh_outline(), circle(KNEE, 6.0), IDLER_SLOT] + bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, M25) + thigh_wall_holes()
    p.slab("XY", zt, z1, loops)
    # full-thickness bosses: hip (motor flange) and knee (625 seat)
    p.slab("XY", z0, zt, [circle((0, 0), P.THIGH_BOSS_R_HIP)] + bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, M25))
    p.slab("XY", z0, zt, [circle(KNEE, P.THIGH_BOSS_R_KNEE), circle(KNEE, 6.0)])
    # rim + cover bosses (tapped for M3 self-tappers)
    p.slab("XY", c0, zt, [thigh_outline(), channel_outline()])
    for b in HIP_BOSSES:
        p.slab("XY", c0, z0, [circle(b, 4.0), circle(b, P.TAP["M3"] / 2)])
    ears(p, c0, z1, P.TAP["M3"])
    # Ø15 boss pocket on motor face, knee 625 seat from channel side (0.5 mm lip)
    # motor-boss locating hole: through (no 15 mm bridge on the bed face); the rotor face closes it
    p.cut("XY", z0, z1, [circle((0, 0), (P.MOTOR_BOSS_D + P.FIT_LOCATE) / 2)])
    p.cut("XY", z0, z1 - 0.5, [circle(KNEE, (P.B625["D"] + P.FIT_BEARING) / 2)])
    return p


def thigh_cover():
    p = Part("thigh_cover")
    z0, z1 = zc(P.Z_THIGH_IN[0]), zc(P.Z_THIGH_IN[1])
    zt = z0 + P.THIGH_WALL_THIN
    loops = [thigh_outline(), circle((0, 0), 6.0), circle(KNEE, 6.0), IDLER_SLOT] +             [circle(b, M3 / 2) for b in HIP_BOSSES] + thigh_wall_holes()
    p.slab("XY", z0, zt, loops)
    p.slab("XY", zt, z1, [circle((0, 0), P.THIGH_BOSS_R_HIP), circle((0, 0), 6.0)] + [circle(b, M3 / 2) for b in HIP_BOSSES])
    p.slab("XY", zt, z1, [circle(KNEE, P.THIGH_BOSS_R_KNEE), circle(KNEE, 6.0)])
    ears(p, z0, z1, M3)
    seat = (P.B625["D"] + P.FIT_BEARING) / 2
    p.cut("XY", z0 + 0.5, z1, [circle((0, 0), seat), circle(KNEE, seat)])
    p.cut("XY", z0, z0 + 3.2, [circle(b, 3.2) for b in HIP_BOSSES])          # flush M3 heads
    return p


# ------------------------------------------------------------------ knee pulley (printed GT2, PD-based)
def knee_pulley():
    name = f"knee_pulley_{P.KNEE_TEETH}T"
    p = Part(name)
    a0, a1 = zc(P.Z_CHANNEL[0]) + 0.5, zc(P.Z_CHANNEL[1]) - 0.5        # 0.5 gap to walls
    hub = 4.5                                                           # Ø9 hub bears on inner races
    ro = P.OD_KNEE / 2
    # hub spacer on the -z side only: the +z face is flat (print bed face); a 0.5 mm M5 shim washer
    # replaces it against the outer-wall bearing's inner race
    p.slab("XY", a0, a1 - 0.5, [circle(KNEE, hub), circle(KNEE, (P.AXLE_D + 0.2) / 2)])
    p.slab("XY", a0 + 0.5, a0 + 1.3, [circle(KNEE, ro + 1.5), circle(KNEE, hub)])                 # flanges
    p.slab("XY", a1 - 1.3, a1 - 0.5, [circle(KNEE, ro + 1.5), circle(KNEE, hub)])
    p.slab("XY", a0 + 1.3, a1 - 1.3, [circle(KNEE, ro), circle(KNEE, hub)])
    # GT2 grooves: 0.75 deep, R0.555 nominal, + tooth tweak
    rg = 0.555 + P.GT2_TOOTH_TWEAK / 2
    grooves = []
    for i in range(P.KNEE_TEETH):
        t = 2 * math.pi * i / P.KNEE_TEETH
        rc = ro - 0.75 + rg
        grooves.append(circle((KNEE[0] + rc * math.cos(t), KNEE[1] + rc * math.sin(t)), rg))
    p.cut("XY", a0 + 1.3, a1 - 1.3, grooves)
    # M5 nylock pocket (inboard face) + belt-clamp pocket (option) left solid
    p.cut("XY", a0 , a0 + 4.0, [hexagon(KNEE, P.NUT_AF["M5"])])
    # lightening ring of round holes (pulley stays a solid disc otherwise)
    p.cut("XY", a0, a1, [circle((KNEE[0] + 11.5 * math.cos(math.radians(a)), KNEE[1] + 11.5 * math.sin(math.radians(a))), 3.5)
                        for a in range(0, 360, 60)])
    return p


# ------------------------------------------------------------------ shank + foot
SH_R_KNEE = P.SHANK_W_KNEE / 2
SH_R_END = P.SHANK_W_FOOT / 2
FOOT_R = P.FOOT_D / 2
# foot axis: tilted SHANK_FOOT_TILT_DEG so the plug follows the end of the bent shank. ax(a, x) is the point
# a mm up the axis from the foot centre and x mm across it (+x side)
_TILT = math.radians(P.SHANK_FOOT_TILT_DEG)
FOOT_UP = (-math.sin(_TILT), math.cos(_TILT))
FOOT_ACROSS = (math.cos(_TILT), math.sin(_TILT))


def ax(a, x=0.0):
    return (FOOT[0] + a * FOOT_UP[0] + x * FOOT_ACROSS[0], FOOT[1] + a * FOOT_UP[1] + x * FOOT_ACROSS[1])


def ax_rect(x0, a0, x1, a1):
    q = [ax(a0, x0), ax(a0, x1), ax(a1, x1), ax(a1, x0)]
    return ("path", [("line", q[i], q[(i + 1) % 4]) for i in range(4)])


SH_END_A = 35.0                      # tip-arc centre, up the foot axis
FOOT_TOP_A = 15.0                    # flat top of the domed foot (print bed face)
PLUG_W = 12.0                        # shank end plugs into a socket in the foot
PLUG_DEPTH = 10.0
FOOT_SCREW_A = FOOT_TOP_A - PLUG_DEPTH / 2


def _shank_bezier(t):
    """Bent (Spot-style) shank centre line, knee -> tip-arc centre: leaves the knee SHANK_BEND_DEG off
    vertical (negative = toward +x, the fold side, as on Spot) and arrives along the tilted foot axis. Returns point, derivative."""
    a = math.radians(P.SHANK_BEND_DEG)
    p = [KNEE, (KNEE[0] - P.SHANK_BEND_REACH * math.sin(a), KNEE[1] - P.SHANK_BEND_REACH * math.cos(a)),
         ax(SH_END_A + P.SHANK_BEND_FOOT), ax(SH_END_A)]
    u = 1 - t
    pt = tuple(u ** 3 * p[0][i] + 3 * u * u * t * p[1][i] + 3 * u * t * t * p[2][i] + t ** 3 * p[3][i] for i in range(2))
    d = tuple(3 * u * u * (p[1][i] - p[0][i]) + 6 * u * t * (p[2][i] - p[1][i]) + 3 * t * t * (p[3][i] - p[2][i]) for i in range(2))
    return pt, d


_SH_T = [i / 400 for i in range(401)]
_SH_S = [0.0]
for _i in range(1, 401):
    _SH_S.append(_SH_S[-1] + math.dist(_shank_bezier(_SH_T[_i])[0], _shank_bezier(_SH_T[_i - 1])[0]))
SH_LEN = _SH_S[-1]                   # centre-line length knee -> tip arc


def shank_frame(s):
    """(point, unit normal) at arc length s from the knee; the normal points to the +x side."""
    s = min(max(s, 0.0), SH_LEN)
    j = min(range(1, 401), key=lambda k: abs(_SH_S[k] - s) if _SH_S[k] >= s else 1e9)
    f = (s - _SH_S[j - 1]) / (_SH_S[j] - _SH_S[j - 1])
    pt, d = _shank_bezier(_SH_T[j - 1] + f * (_SH_T[j] - _SH_T[j - 1]))
    L = math.hypot(*d)
    return pt, (-d[1] / L, d[0] / L)


def shank_hw(s):
    """Outline half-width at arc length s (linear taper knee -> tip)."""
    return SH_R_KNEE + (SH_R_END - SH_R_KNEE) * min(max(s / SH_LEN, 0.0), 1.0)


def _band(s0, s1, hw_fn, n=13):
    """Closed curved band between s0 and s1 of the centre line, half-width hw_fn(s), round ends
    (arcs centred on the centre line) -> CCW path."""
    ss = [s0 + (s1 - s0) * i / (n - 1) for i in range(n)]
    off = lambda s, k: (shank_frame(s)[0][0] + k * hw_fn(s) * shank_frame(s)[1][0],          # noqa: E731
                        shank_frame(s)[0][1] + k * hw_fn(s) * shank_frame(s)[1][1])
    L = [off(s, -1) for s in ss]
    R = [off(s, 1) for s in ss]
    return ("path", [("spline", L), ("arc", shank_frame(s1)[0], L[-1], R[-1], 1),
                     ("spline", R[::-1]), ("arc", shank_frame(s0)[0], R[0], L[0], 1)])


def shank_pocket_hw(s):
    return shank_hw(s) - P.SK_FLANGE


def shank_holes():
    """3 rounded rectangles following the bend, inside the web pocket (they cut the 3 mm web only)."""
    hw = lambda s: shank_pocket_hw(s) - P.SK_CUT_INSET_SHANK        # noqa: E731
    return rect_cutouts(17.0, SH_LEN - 3.0, P.SK_CUTS, shank_frame, hw, P.SK_CUT_WALL_SHANK, P.SK_CUT_R - 0.5)


def shank():
    p = Part("shank")
    z0, z1 = zc(P.Z_SHANK[0]), zc(P.Z_SHANK[1])
    holes = []
    if P.SKELETON:
        holes = shank_holes()
    p.slab("XY", z0, z1, [_band(0.0, SH_LEN, shank_hw), circle(KNEE, (P.AXLE_D + 0.2) / 2)] + holes)
    # plug into the foot's socket, with the cross screw hole (vertical when the shank is printed flat)
    p.slab("XY", z0, z1, [ax_rect(-PLUG_W / 2, FOOT_TOP_A - PLUG_DEPTH, PLUG_W / 2, SH_END_A - 3.0), circle(ax(FOOT_SCREW_A), M3 / 2)])
    # M5 head hex pocket on the outboard face (an M5 washer spaces the shank off the 625 inner race,
    # so the inboard face stays flat for the bed)
    p.cut("XY", z1 - 3.6, z1 + 0.01, [hexagon(KNEE, P.NUT_AF["M5"])])
    if P.SKELETON:
        # web pocket from the outboard (top-when-printed) face, following the bend: edge flanges stay
        # full height, web SK_WEB thick on the bed side -> channel section, support-free
        sa = P.SK_KNEE_SOLID_R + shank_pocket_hw(P.SK_KNEE_SOLID_R + 8.0)
        sb = SH_LEN - shank_pocket_hw(SH_LEN)
        p.cut("XY", z0 + P.SK_WEB, z1 + 0.01, [_band(sa, sb, shank_pocket_hw)])
    return p


def _poly(pts, r=0.0):
    """Closed polygon loop; corners filleted at r when r > 0.3 mm."""
    pts = [tuple(map(float, q)) for q in pts]
    if r > 0.3:
        return rounded_poly(pts, r)
    return ("path", [("line", pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))])


ANKLE = ax(P.ANKLE_UP)               # ankle pivot = the plug's M3 cross hole


def snowshoe():
    """Snowshoe foot (replaces the ball): teardrop pad with a closed skin under generative Voronoi ribs,
    rim, upturned toe, two cheeks round the shank plug and two lips that stop the plug at ANKLE_LIM.
    Modelled flat at the ankle (part planes = pad frame, shank angle 0); the assembly's ankle mate turns it to
    ANKLE_BUILD. (Move Body would leave the part planes behind and confuse the mate.)"""
    import snowshoe_geo as G
    p = Part("snowshoe")
    px, py = ANKLE
    zl = zc(0.0)
    X = lambda u: px + u                                            # noqa: E731
    Y = lambda v: py + v                                            # noqa: E731
    ys = Y(G.V_SOLE)
    outer = stadium((X(G.U_TAILC), zl), P.SHOE_R_TAIL, (X(G.U_TOEC), zl), P.SHOE_R_TOE)
    inner = stadium((X(G.U_TAILC), zl), P.SHOE_R_TAIL - P.SHOE_RIM_W, (X(G.U_TOEC), zl), P.SHOE_R_TOE - P.SHOE_RIM_W)
    # skin, rim, Voronoi ribs (cells are pockets down to the skin)
    p.slab("XZ", ys, ys + P.SHOE_SKIN, [outer])
    p.slab("XZ", ys, ys + P.SHOE_RIM_H, [outer, inner])
    cells = [_poly([(X(u), zl + w) for u, w in c], G.safe_fillet(c, 0.8)) for c in G.lattice_cells()]
    p.slab("XZ", ys + P.SHOE_SKIN, ys + P.SHOE_SKIN + P.SHOE_RIB_H, [inner] + cells)
    # three tapered keels carry the toe / heel bending into the hub
    for w, prof in G.keels():
        p.slab("XY", zl + w - P.SHOE_KEEL_W / 2, zl + w + P.SHOE_KEEL_W / 2, [_poly([(X(u), Y(v)) for u, v in prof])])
    # toe edge chamfered up 45 deg so it rides up out of the snow (and prints without supports)
    xt, run = X(G.U_TOE), P.SHOE_TOE_RUN
    p.cut("XY", zl - 40.0, zl + 40.0, [_poly([(xt - run, ys), (xt - run, ys - 1.0), (xt + 1.0, ys - 1.0),
                                              (xt + 1.0, ys + P.SHOE_TOE_LIFT * (run + 1.0) / run)])])
    # cheeks either side of the 10 mm shank plug, and the two lips bridging them
    lip = G.lips(FOOT_UP, FOOT_ACROSS, PLUG_W)
    cheek = _poly([(X(u), Y(v)) for u, v in G.cheek_profile(lip)])
    g = P.SHANK_T / 2 + 0.2
    for s in (-1, 1):
        p.slab("XY", zl + s * g, zl + s * (g + P.SHOE_CHEEK_T), [cheek, circle(ANKLE, M3 / 2)])
    p.slab("XY", zl - g, zl + g, [_poly([(X(u), Y(v)) for u, v in q]) for q in lip])
    return p


def foot():
    """RETIRED (replaced by snowshoe()). Domed foot: Ø22 cylinder + hemisphere on the tilted foot axis, printed standing on its flat top
    (dome up, no supports). The shank end plugs into a 12.4 x 10.4 socket; one M3 x 25 + nylock across holds it."""
    p = Part("foot")
    p.revolve_capsule(FOOT, FOOT_R, FOOT_TOP_A, zc(0.0), P.SHANK_FOOT_TILT_DEG)
    zl = zc(0.0)
    p.cut("XY", zl - (P.SHANK_T / 2 + 0.2), zl + (P.SHANK_T / 2 + 0.2),
          [ax_rect(-PLUG_W / 2 - 0.2, FOOT_TOP_A - PLUG_DEPTH, PLUG_W / 2 + 0.2, FOOT_TOP_A + 1.0)])
    # flats for the screw head and the nut, then the cross hole (horizontal in print, Ø3.4 clearance)
    flat = ax_rect(-5.0, FOOT_SCREW_A - 5.0, 5.0, FOOT_TOP_A + 1.0)
    p.cut("XY", zl + FOOT_R - 1.5, zl + FOOT_R + 1.0, [flat])
    p.cut("XY", zl - FOOT_R - 1.0, zl - FOOT_R + 1.5, [flat])
    p.cut("XY", zl - FOOT_R - 1.0, zl + FOOT_R + 1.0, [circle(ax(FOOT_SCREW_A), M3 / 2)])
    return p


# ------------------------------------------------------------------ knee-motor adapter (hip-pulley shaft)
def hip_adapter():
    p = Part("hip_shaft_adapter")
    z0, z1 = zc(P.Z_ADAPTER[0]), zc(P.Z_ADAPTER[1])
    p.slab("XY", z0, z1, [circle((0, 0), 18.0), circle((0, 0), (P.AXLE_D + 0.2) / 2)] + bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, M25))
    p.cut("XY", z1 - 2.5, z1 + 0.01, bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, 5.0))                 # cbores
    p.cut("XY", z0 , z0 + P.MOTOR_BOSS_H + 0.2, [circle((0, 0), (P.MOTOR_BOSS_D + P.FIT_LOCATE) / 2)])
    p.cut("XY", z0 + P.MOTOR_BOSS_H + 0.2, z0 + P.MOTOR_BOSS_H + 3.8, [hexagon((0, 0), P.NUT_AF["M5"])])  # M5 head
    p.slab("XY", z1, z1 + 0.5, [circle((0, 0), 4.5), circle((0, 0), (P.AXLE_D + 0.2) / 2)])          # inner-race spacer
    return p


# ------------------------------------------------------------------ hip module frame
FX0 = P.FLANGE_X
FX1 = P.FLANGE_X + P.FLANGE_T
ZP_IN0, ZP_IN1 = zc(P.Z_PLATE_IN[0]), zc(P.Z_PLATE_IN[1])
ZP_PAD = ZP_IN1 + 2.0                         # plate thickened to 7 mm under the flange feet (insert depth)
FOOT_Z = (ZP_PAD, ZP_PAD + 6.0)
FL_Z1 = zc(P.Z_THIGH_IN[0]) - P.ABAD_CLEAR    # flange edge, 2 mm inboard of the thigh's swept slab
FL_Y = 26.0
BRIDGE = (-11.0, 11.0, P.BRIDGE_Y[0], 40.0)  # C-frame column x0, x1, y0, y1
PLATE_TOP = P.BRIDGE_Y[0] - 0.2               # inboard plate stops under the column
TAB_Z0 = ZP_IN0 - 4.0                         # C-frame tab plate on the inboard plate's inboard face
FOOT_SCREWS = {s: [(s * 29.3, -17.0), (s * 29.3, 17.0)] for s in (-1, 1)}
INS = P.INSERT_M3
INS_DEPTH = 6.7                               # M3 x 5.7 insert + 2 pitches
INS_BOSS = 5.05                               # insert hole radius + 3 mm wall
BACK_PHASE = 126.0
ABAD_PHASE = 0.0                              # ab/ad output pattern turned so no hole falls on the flange foot


def insert_holes(p, plane, face, into, pts):
    a, b = (face, face + into * INS_DEPTH) if into > 0 else (face - INS_DEPTH, face)
    p.cut(plane, min(a, b), max(a, b), [circle(c, INS / 2) for c in pts])


def module_plate_inboard():
    """Knee-motor plate. The two ab/ad flanges are separate L-brackets (printed flat, so their locating
    hole and bolt holes come out round), screwed down into heat-set inserts in thickened zones."""
    p = Part("module_plate_inboard")
    p.slab("XY", ZP_IN0, ZP_IN1, [rect(-FX1, -30.0, FX1, PLATE_TOP), circle((0, 0), 10.0)] +
           bolt_circle((0, 0), P.MOTOR_BACK_PCD, P.MOTOR_BACK_N, M25, BACK_PHASE))
    for s_ in (-1, 1):
        x0, x1 = sorted((s_ * 27.5, s_ * FX1))
        p.slab("XY", ZP_IN0, ZP_PAD, [rect(x0, -FL_Y, x1, FL_Y)])
        for c in FOOT_SCREWS[s_]:
            p.slab("XY", ZP_IN0, ZP_PAD, [circle(c, INS_BOSS)])
        insert_holes(p, "XY", ZP_PAD, -1, FOOT_SCREWS[s_])
    return p


def module_flange(side):
    back = side == "back"
    p = Part(f"module_flange_{side}")
    s_ = -1 if back else 1
    x0, x1 = sorted((s_ * FX0, s_ * FX1))
    holes = bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, M25, ABAD_PHASE) if back else [circle((0, 0), (P.STUB_D + 0.2) / 2)]
    p.slab("YZ", x0, x1, [rect(FOOT_Z[0], -FL_Y, FL_Z1, FL_Y)] + holes)
    if back:
        p.cut("YZ", x0, x1, [circle((0, 0), (P.MOTOR_BOSS_D + P.FIT_LOCATE) / 2)])     # through: no bed-face bridge
    else:
        p.cut("YZ", x0, x0 + 3.6, [hexagon((0, 0), P.NUT_AF["M5"], 30)])             # M5 head, stub axle
    fx0, fx1 = sorted((s_ * 27.5, s_ * FX1))
    p.slab("XY", FOOT_Z[0], FOOT_Z[1], [rect(fx0, -FL_Y, fx1, FL_Y)])
    for c in FOOT_SCREWS[s_]:
        p.slab("XY", FOOT_Z[0], FOOT_Z[1], [circle(c, INS_BOSS), circle(c, M3 / 2)])
    p.cut("XY", FOOT_Z[0], FOOT_Z[1], [circle(c, M3 / 2) for c in FOOT_SCREWS[s_]])
    return p


def module_frame_outboard():
    """C-frame: outboard plate (hip-motor back face) + bridge column over the top (hip hard stop, +/-107 deg)
    + inboard tab plate clamped to the inboard plate by the 5 knee-motor back screws (M2.5 x 12).
    Replaces the outboard plate whose 8 mm bridge tab needed support and hung on 2 M3 self-tappers.
    Printed lying on the column's top face: both plates stand up, no supports."""
    p = Part("module_frame_outboard")
    zo0, zo1 = zc(P.Z_PLATE_OUT[0]), zc(P.Z_PLATE_OUT[1])
    motor_holes = bolt_circle((0, 0), P.MOTOR_BACK_PCD, P.MOTOR_BACK_N, M25, BACK_PHASE)
    cable = teardrop((0, 0), 10.0, (0.0, -1.0))          # -y is 'up' when printed
    p.slab("XY", zo0, zo1, [circle((0, 0), 30.0)])
    p.slab("XY", zo0, zo1, [rect(-30.0, 0.0, 30.0, BRIDGE[3])])
    p.cut("XY", zo0, zo1, [cable] + motor_holes)
    # inboard tab: 45-deg chamfers (self-supporting when printed; corners inside r 38.4 for the ab/ad sweep)
    pts = [(BRIDGE[0], BRIDGE[3]), (BRIDGE[1], BRIDGE[3]), (BRIDGE[1], 34.0), (30.0, 15.0), (30.0, -30.0),
           (-30.0, -30.0), (-30.0, 15.0), (BRIDGE[0], 34.0)]
    outline = ("path", [("line", pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))])
    p.slab("XY", TAB_Z0, ZP_IN0, [outline, cable] + motor_holes)
    p.slab("XY", TAB_Z0, zo0, [rect(BRIDGE[0], BRIDGE[2], BRIDGE[1], BRIDGE[3])])
    return p


# ------------------------------------------------------------------ body bracket (ab/ad stage)
ABAD_X1 = -FX1                       # ab/ad motor output face (bolted to the back flange)
ABAD_X0 = ABAD_X1 - P.MOTOR_L
BB_BACK = (ABAD_X0 - P.BRACKET_T, ABAD_X0)
BB_FRONT = (FX1 + P.GAP_MOVING, FX1 + P.GAP_MOVING + P.BRACKET_T)
SP0, SP1 = P.SPINE_Z
BLK = (BB_BACK[1] + 0.2, -48.0, SP1 + 6.0)      # back block x0, x1, top z: the back wall's foot sits on it
WALL_BACK_SCREWS = [(x, y) for x in (-67.0, -55.0) for y in (-12.0, 12.0)]
WALL_FRONT_SCREWS = [(x, y) for x in (20.0, 29.5) for y in (-17.3, 17.3)]
SPINE_FRONT_W = 24.0                             # spine widened under the front wall foot
FRONT_FOOT_X0 = 17.0
RIG_SCREWS = [(-62.0, -6.0), (-62.0, 6.0), (26.0, -8.0), (26.0, 8.0)]
RIG_Z = (SP0 - 8.0, SP0)


def body_bracket_back():
    p = Part("body_bracket_back")
    p.slab("YZ", *BB_BACK, [circle((0, 0), P.BB_BACK_R)])
    p.slab("YZ", *BB_BACK, [rect(BLK[2], -20.0, -15.0, 20.0)])
    p.cut("YZ", BB_BACK[0], BB_BACK[1] + 0.01, [circle((0, 0), 10.0)] + bolt_circle((0, 0), P.MOTOR_BACK_PCD, P.MOTOR_BACK_N, M25, 90))
    # L-foot on the spine's back block: screws go DOWN into heat-set inserts (no horizontal holes to drill)
    p.slab("XY", BLK[2], BLK[2] + 6.0, [rect(BB_BACK[1] - 0.01, -20.0, -51.0, 20.0)] + [circle(c, M3 / 2) for c in WALL_BACK_SCREWS])
    return p


def body_bracket_front():
    p = Part("body_bracket_front")
    p.slab("YZ", *BB_FRONT, [circle((0, 0), P.BB_FRONT_R)])
    p.slab("YZ", *BB_FRONT, [rect(SP1, -SPINE_FRONT_W, -12.0, SPINE_FRONT_W)])
    seat = (P.B625["D"] + P.FIT_BEARING) / 2
    p.cut("YZ", BB_FRONT[0], BB_FRONT[1] + 0.01, [circle((0, 0), 6.0)])
    p.cut("YZ", BB_FRONT[0], BB_FRONT[1] - 0.5, [circle((0, 0), seat)])
    p.slab("XY", SP1, SP1 + 6.0, [rect(FRONT_FOOT_X0, -SPINE_FRONT_W, BB_FRONT[0] + 0.01, SPINE_FRONT_W)] +
           [circle(c, M3 / 2) for c in WALL_FRONT_SCREWS])
    return p


def body_spine():
    p = Part("body_spine")
    x0, x1 = BB_BACK[1] + 0.2, BB_FRONT[0] - 0.2
    p.slab("XY", SP0, SP1, [rect(x0, -20.0, x1, 20.0), slot((-20.0, -4.0), (-20.0, 4.0), 2.0), slot((0.0, -4.0), (0.0, 4.0), 2.0)])
    p.slab("XY", SP0, SP1, [rect(14.0, -SPINE_FRONT_W, x1, SPINE_FRONT_W)])
    p.slab("XY", SP1, BLK[2], [rect(BLK[0], -20.0, BLK[1], 20.0)])
    insert_holes(p, "XY", BLK[2], -1, WALL_BACK_SCREWS)
    insert_holes(p, "XY", SP1, -1, WALL_FRONT_SCREWS)
    insert_holes(p, "XY", SP0, +1, RIG_SCREWS)
    return p


def rig_plate():
    p = Part("rig_plate")
    x0, x1 = BB_BACK[1] + 0.2, BB_FRONT[0] - 0.2
    g = P.RIG_SLOT_GRID / 2
    loops = [rect(x0, -20.0, x1, 20.0)]
    loops += [slot((x - 3, y), (x + 3, y), P.CLR["M5"] / 2) for x in (-32.0, -12.0) for y in (-g, g)]
    loops += [circle(c, M3 / 2) for c in RIG_SCREWS]
    p.slab("XY", RIG_Z[0], RIG_Z[1], loops)
    p.cut("XY", RIG_Z[0], RIG_Z[0] + 3.5, [circle(c, 3.3) for c in RIG_SCREWS])        # M3 heads flush with the rig face
    return p


# ------------------------------------------------------------------ reference bodies (not printed)
def motor(name, axis, n0, n1, out_at_high):
    p = Part(name)
    body = [circle((0, 0), P.MOTOR_D / 2)]
    step = [circle((0, 0), 37.0 / 2)]
    if out_at_high:
        p.slab(axis, n0, n1 - 1.0, body); p.slab(axis, n1 - 1.0, n1, step)
    else:
        p.slab(axis, n0 + 1.0, n1, body); p.slab(axis, n0, n0 + 1.0, step)
    return p


def ref_hip_motor():
    return motor("ref_motor_hip", "XY", zc(P.Z_HIP_MOTOR[0]), zc(P.Z_HIP_MOTOR[1]), out_at_high=False)


def ref_knee_motor():
    return motor("ref_motor_knee", "XY", zc(P.Z_KNEE_MOTOR[0]), zc(P.Z_KNEE_MOTOR[1]), out_at_high=True)


def ref_abad_motor():
    return motor("ref_motor_abad", "YZ", ABAD_X0, ABAD_X1, out_at_high=True)


def ref_hip_pulley():
    p = Part("ref_pulley_20T_kit")
    a0, a1 = zc(P.Z_CHANNEL[0]) + 0.5, zc(P.Z_CHANNEL[1]) - 0.5
    p.slab("XY", a0, a1, [circle((0, 0), P.OD_HIP / 2), circle((0, 0), P.AXLE_D / 2)])
    return p


# ------------------------------------------------------------------ fit-test coupon
def coupon():
    p = Part("coupon_fit_test")
    W, H = 165.0, 60.0
    loops = [rect(0, 0, W, H)]
    x = 12.0
    for d in (0.10, 0.15, 0.20, 0.25):                 # Ø15 boss pocket
        loops.append(circle((x, 15.0), (15.0 + d) / 2)); x += 19.0
    for d in (0.05, 0.10, 0.15, 0.20):                 # 625 seats
        loops.append(circle((x, 15.0), (16.0 + d) / 2)); x += 19.5
    x = 10.0
    for d in (0.2, 0.3, 0.4):                          # Ø5 axle
        loops.append(circle((x, 38.0), (5.0 + d) / 2)); x += 9.0
    for dd in (M25, M3, P.TAP["M2.5"], P.TAP["M3"], P.INSERT_M3):
        loops.append(circle((x, 38.0), dd / 2)); x += 9.0
    loops.append(hexagon((x + 2, 38.0), P.NUT_AF["M5"])); x += 14.0
    loops.append(hexagon((x, 38.0), P.NUT_AF["M3"])); x += 10.0
    # skeleton sample: one rounded triangle hole leaving a min-width strut
    loops.append(rounded_poly([(x, 30.0), (x + 14.0, 30.0), (x + 7.0, 46.0)], P.SK_FILLET))
    loops.append(rounded_poly([(x + 14.0 + P.SK_STRUT * 1.2, 30.0), (x + 21.0 + P.SK_STRUT * 1.2, 46.0), (x + 7.0 + P.SK_STRUT * 1.2, 46.0)], P.SK_FILLET))
    p.slab("XY", 0.0, 5.0, loops)
    return p


# named reference axes used by the assembly mates (body frame)
HIP_AX = ("HIP_AXIS", "Z", 0.0, 0.0)
KNEE_AX = ("KNEE_AXIS", "Z", KNEE[0], KNEE[1])
ABAD_AX = ("ABAD_AXIS", "X", 0.0, 0.0)
ANKLE_AX = ("ANKLE_AXIS", "Z", ANKLE[0], ANKLE[1])
AXES = {"module_plate_inboard": [HIP_AX, ABAD_AX], "thigh_outer": [HIP_AX, KNEE_AX], "shank": [KNEE_AX, ANKLE_AX],
        "snowshoe": [ANKLE_AX],
        "hip_shaft_adapter": [HIP_AX], "body_spine": [ABAD_AX]}

BUILDERS = {
    "thigh_outer": thigh_outer, "thigh_cover": thigh_cover, "knee_pulley": knee_pulley, "shank": shank,
    "snowshoe": snowshoe, "hip_shaft_adapter": hip_adapter, "module_plate_inboard": module_plate_inboard,
    "module_flange_back": lambda: module_flange("back"), "module_flange_front": lambda: module_flange("front"),
    "module_frame_outboard": module_frame_outboard, "body_bracket_back": body_bracket_back,
    "body_bracket_front": body_bracket_front, "body_spine": body_spine, "rig_plate": rig_plate,
    "ref_motor_hip": ref_hip_motor, "ref_motor_knee": ref_knee_motor, "ref_motor_abad": ref_abad_motor,
    "ref_pulley_20T_kit": ref_hip_pulley, "coupon_fit_test": coupon,
}


def main(names):
    from swlib import sw
    sw().CloseAllDocuments(True)
    report = {}
    path = os.path.join(PARTS, "parts_report.json" if P.SKELETON else "parts_report_solid.json")
    if os.path.exists(path):
        report = json.load(open(path))
    for n in names or BUILDERS:
        print("building", n, flush=True)
        p = BUILDERS[n]()
        for ax in AXES.get(n, []):
            p.axis(*ax)
        vol = p.volume_mm3()
        report[p.name] = {"bbox_mm": p.bbox(), "volume_mm3": round(vol), "solid_pla_g": round(vol * P.PLA_RHO * 1000, 1),
                          "skeleton": P.SKELETON}
        if P.SKELETON:
            p.save(PARTS, STL)        # solid variants are only measured, never saved over the shipped parts
        p.close()
        print("  ", report[p.name], flush=True)
    json.dump(report, open(path, "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1:])
