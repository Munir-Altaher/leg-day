"""Route A generator for the SNOW LEG: builds every part in SolidWorks from snow_leg/params.py.
Run:  python cad/build_parts.py [part ...]        (part keys without the snow_ prefix)
All parts are modelled in the body frame at q = 0 (leg straight down, ankle angle 0 =
sole perpendicular to the shank), so components placed at their box centre assemble in
the home pose. Every file is saved as snow_<name> (SolidWorks needs unique names to have
the original leg open at the same time). Paths are relative to this file, so nothing is
written outside snow_leg/.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
import params as P  # noqa: E402
import snowgeo as S  # noqa: E402
from swlib import Part, circle, stadium, rounded_poly, rect, slot, bolt_circle, teardrop  # noqa: E402

PARTS = os.path.join(HERE, "parts")
STL = os.path.join(HERE, "stl", "assembly_pose")     # SolidWorks exports (assembly pose); print-oriented copies in cad/stl/
N = lambda s: P.FILE_PREFIX + s          # noqa: E731
zc = P.zc
L1, L2 = P.L1, P.L2
KNEE = (0.0, -L1)
PIV = S.PIVOT                             # ankle pivot (snowshoe) / ball centre (ball foot)
ZL = P.L_ABAD                             # leg plane
M25, M3, M5 = P.CLR["M2.5"], P.CLR["M3"], P.CLR["M5"]
INS = P.INSERTS


def boss_r(size):
    """Standalone insert boss radius (brief 6.4): insert axis at least one insert OD from any free edge,
    and at least INSERT_WALL of material around the hole -> max(OD, hole/2 + wall)."""
    return max(INS[size]["od"], INS[size]["hole"] / 2 + P.INSERT_WALL)


def insert_hole(p, plane, face, into, pts, size, through=False):
    """Cut heat-set insert holes (modelled Ø incl. +0.1 compensation) from `face` going `into` (+1 / -1)."""
    d = 50.0 if through else P.insert_depth(size)
    a, b = (face, face + into * d) if into > 0 else (face - d, face)
    p.cut(plane, min(a, b), max(a, b), [circle(c, INS[size]["hole"] / 2) for c in pts])


def hexagon(c, af, rot=0.0):
    r = af / math.sqrt(3)
    pts = [(c[0] + r * math.cos(math.radians(rot + 60 * i)), c[1] + r * math.sin(math.radians(rot + 60 * i))) for i in range(6)]
    return ("path", [("line", pts[i], pts[(i + 1) % 6]) for i in range(6)])


def poly(pts):
    return ("path", [("line", pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))])


def bar(p0, p1, w):
    """Thin rectangle (path) of width w along the segment p0 -> p1 (for reference springs)."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    nx, ny = -dy / L * w / 2, dx / L * w / 2
    return poly([(p0[0] + nx, p0[1] + ny), (p1[0] + nx, p1[1] + ny), (p1[0] - nx, p1[1] - ny), (p0[0] - nx, p0[1] - ny)])


# ================================================================== thigh (sealed belt channel)
def thigh_outline():
    return stadium((0, 0), P.THIGH_R_HIP, KNEE, P.THIGH_R_KNEE)


def channel_outline(inset=0.0):
    return stadium((0, 0), P.THIGH_CH_R_HIP - inset, KNEE, P.THIGH_CH_R_KNEE - inset)


# cover screws: 2 hip bosses (30 / 150 deg; the 270 deg one sat between the belt spans) + 2 internal bosses
COVER_SCREWS = [(16.5 * math.cos(math.radians(a)), 16.5 * math.sin(math.radians(a))) for a in (30, 150)] + \
               [(0.0, -60.0), (0.0, -100.0)]
IDLER_SLOT = slot((5.0, P.IDLER_Y), (5.0 + P.IDLER_SLOT_TRAVEL, P.IDLER_Y), P.CLR[P.IDLER_BOLT] / 2)
LIP_H = 0.6                    # labyrinth lip on the cover, into the rim
LIP_W = 1.2
LIP_GAP = 0.2


def thigh_outer():
    p = Part(N("thigh_outer"))
    z0, z1 = zc(P.Z_THIGH_OUT[0]), zc(P.Z_THIGH_OUT[1])
    zt = z1 - P.THIGH_WALL_THIN
    c0 = zc(P.Z_CHANNEL[0])
    p.slab("XY", zt, z1, [thigh_outline(), circle(KNEE, 6.0), IDLER_SLOT] + bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, M25))
    p.slab("XY", z0, zt, [circle((0, 0), P.THIGH_BOSS_R_HIP)] + bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, M25))
    p.slab("XY", z0, zt, [circle(KNEE, P.THIGH_BOSS_R_KNEE), circle(KNEE, 6.0)])
    p.slab("XY", c0, zt, [thigh_outline(), channel_outline()])          # closed rim: no ears, no holes
    for b in COVER_SCREWS:
        p.slab("XY", c0, zt, [circle(b, boss_r("M3"))])                 # down to the (thinned) wall
    insert_hole(p, "XY", c0, +1, COVER_SCREWS, "M3")                     # inserts from the channel side (top when printed)
    # motor-boss locating hole: through (no bridge on the bed face); the motor's rotor face closes it
    p.cut("XY", z0, z1, [circle((0, 0), (P.MOTOR_BOSS_D + P.FIT_LOCATE) / 2)])
    p.cut("XY", z0, z1 - 0.5, [circle(KNEE, (P.B625["D"] + P.FIT_BEARING) / 2)])
    return p


def thigh_cover():
    p = Part(N("thigh_cover"))
    z0, z1 = zc(P.Z_THIGH_IN[0]), zc(P.Z_THIGH_IN[1])
    zt = z0 + P.THIGH_WALL_THIN
    p.slab("XY", z0, zt, [thigh_outline(), circle((0, 0), 6.0), circle(KNEE, 6.0), IDLER_SLOT] +
           [circle(b, M3 / 2) for b in COVER_SCREWS])
    p.slab("XY", zt, z1, [circle((0, 0), P.THIGH_BOSS_R_HIP), circle((0, 0), 6.0)] + [circle(b, M3 / 2) for b in COVER_SCREWS[:2]])
    p.slab("XY", zt, z1, [circle(KNEE, P.THIGH_BOSS_R_KNEE), circle(KNEE, 6.0)])
    for b in COVER_SCREWS[2:]:                                            # local 5 mm pads at the internal bosses
        p.slab("XY", zt, z1, [circle(b, 6.5), circle(b, M3 / 2)])
    # labyrinth lip just inside the rim (seam seal)
    p.slab("XY", z1, z1 + LIP_H, [channel_outline(LIP_GAP), channel_outline(LIP_GAP + LIP_W)])
    p.cut("XY", z1, z1 + LIP_H, [circle(b, boss_r("M3") + 0.4) for b in COVER_SCREWS[:2]])
    seat = (P.B625["D"] + P.FIT_BEARING) / 2
    p.cut("XY", z0 + 0.5, z1, [circle((0, 0), seat), circle(KNEE, seat)])
    p.cut("XY", z0, z0 + 2.3, [circle(b, 3.75) for b in COVER_SCREWS])  # flush M3 button heads + washer
    return p


# ================================================================== knee pulley (unchanged design)
def knee_pulley():
    p = Part(N(f"knee_pulley_{P.KNEE_TEETH}T"))
    a0, a1 = zc(P.Z_CHANNEL[0]) + 0.5, zc(P.Z_CHANNEL[1]) - 0.5
    hub = 4.5
    ro = P.OD_KNEE / 2
    # hub spacer only on the -z side; the +z face is flat (print bed face), a 0.5 mm M5 shim washer
    # takes its place against the outer-wall bearing's inner race
    p.slab("XY", a0, a1 - 0.5, [circle(KNEE, hub), circle(KNEE, (P.AXLE_D + 0.2) / 2)])
    p.slab("XY", a0 + 0.5, a0 + 1.3, [circle(KNEE, ro + 1.5), circle(KNEE, hub)])
    p.slab("XY", a1 - 1.3, a1 - 0.5, [circle(KNEE, ro + 1.5), circle(KNEE, hub)])
    p.slab("XY", a0 + 1.3, a1 - 1.3, [circle(KNEE, ro), circle(KNEE, hub)])
    rg = 0.555 + P.GT2_TOOTH_TWEAK / 2
    grooves = [circle((KNEE[0] + (ro - 0.75 + rg) * math.cos(2 * math.pi * i / P.KNEE_TEETH),
                       KNEE[1] + (ro - 0.75 + rg) * math.sin(2 * math.pi * i / P.KNEE_TEETH)), rg) for i in range(P.KNEE_TEETH)]
    p.cut("XY", a0 + 1.3, a1 - 1.3, grooves)
    p.cut("XY", a0, a0 + 4.0, [hexagon(KNEE, P.NUT_AF["M5"])])
    p.cut("XY", a0, a1, [circle((KNEE[0] + 11.5 * math.cos(math.radians(a)), KNEE[1] + 11.5 * math.sin(math.radians(a))), 3.5)
                        for a in range(0, 360, 60)])
    return p


# ================================================================== shank (closed) + knee stop posts + ankle tip
KNEE_UP, KNEE_LO = S.design_knee_fins()
POST_UP = (KNEE[0], KNEE[1] - KNEE_UP[0])                                          # on the shank axis
POST_LO = (KNEE[0] + KNEE_LO[2][0], KNEE[1] + KNEE_LO[2][1])                       # on a horn behind the knee
SPRING_POST_SHANK = (PIV[0], PIV[1] + 30.0)                                        # on the shank axis, 30 above the pivot
POST_SLEEVE_R = S.FIN_HALF
POST_Z = (zc(P.Z_SHANK[0]) - 8.0, zc(P.Z_SHANK[0]))                                # sleeves stand 8 mm proud, inboard


def shank():
    if P.FOOT == "ball":
        return shank_ball()
    p = Part(N("shank"))
    z0, z1 = zc(P.Z_SHANK[0]), zc(P.Z_SHANK[1])
    p.slab("XY", z0, z1, [stadium(KNEE, P.SHANK_W_KNEE / 2, PIV, P.SHANK_TIP_R), circle(KNEE, (P.AXLE_D + 0.2) / 2),
                          circle(PIV, (P.ANKLE_PIN_D + P.ANKLE_BUSH_FIT) / 2)])
    # horn for the lower knee-stop post (M5 insert, edge distance one insert OD)
    p.slab("XY", z0, z1, [stadium(KNEE, P.SHANK_W_KNEE / 2, POST_LO, boss_r("M5"))])
    p.cut("XY", z1 - 3.6, z1, [hexagon(KNEE, P.NUT_AF["M5"])])                      # M5 head (outboard face, top when printed)
    insert_hole(p, "XY", z0, +1, [POST_UP, POST_LO], "M5", through=True)            # 9.5 mm inserts in the 10 mm shank
    insert_hole(p, "XY", z0, +1, [SPRING_POST_SHANK], "M3")
    return p


def shank_ball():
    """FOOT = 'ball': the Leg Day shank end (lap joint, sphere centre at L2) with the snow-leg knee stops."""
    p = Part(N("shank_ball"))
    z0, z1 = zc(P.Z_SHANK[0]), zc(P.Z_SHANK[1])
    foot = PIV
    end_y = foot[1] + 35.0
    lap = (foot[1] + 11.0, foot[1] + 32.0)
    screws = [(0.0, foot[1] + 14.0), (0.0, foot[1] + 23.0)]
    p.slab("XY", z0, z1, [stadium(KNEE, P.SHANK_W_KNEE / 2, (0, end_y), P.SHANK_W_FOOT / 2), circle(KNEE, (P.AXLE_D + 0.2) / 2)])
    p.slab("XY", z0, z0 + 4.0, [rect(-P.SHANK_W_FOOT / 2, lap[0], P.SHANK_W_FOOT / 2, end_y)] + [circle(s, M3 / 2) for s in screws])
    p.slab("XY", z0, z1, [stadium(KNEE, P.SHANK_W_KNEE / 2, POST_LO, boss_r("M5"))])
    p.cut("XY", z1 - 3.6, z1, [hexagon(KNEE, P.NUT_AF["M5"])])
    insert_hole(p, "XY", z0, +1, [POST_UP, POST_LO], "M5", through=True)
    return p


def foot_ball():
    p = Part(N("foot_ball"))
    z0, z1 = zc(P.Z_SHANK[0]), zc(P.Z_SHANK[1])
    foot = PIV
    end_y = foot[1] + 35.0
    lap = (foot[1] + 11.0, foot[1] + 32.0)
    screws = [(0.0, foot[1] + 14.0), (0.0, foot[1] + 23.0)]
    p.revolve_sphere(foot, P.FOOT_D / 2, zc(0.0))
    p.slab("XY", z0 + 4.0, z1, [rect(-P.SHANK_W_FOOT / 2, lap[0] - 3.0, P.SHANK_W_FOOT / 2, end_y - P.SHANK_W_FOOT / 2 - 0.3)])
    insert_hole(p, "XY", z1, -1, screws, "M3")          # inserts from the outboard face (top when printed)
    return p


# ================================================================== snowshoe foot (built in the body frame, foot frame origin = pivot)
FX = lambda x: PIV[0] + x                      # noqa: E731  foot-frame x -> body x
FY = lambda y: PIV[1] + y                      # noqa: E731  foot-frame y -> body y
Y_DT, Y_DB, Y_LT = FY(S.Y_DECK_TOP), FY(S.Y_DECK_BOT), FY(S.Y_LUG_TIP)
BRACKET_SCREWS = [(FX(x), z) for x in (-15.0, 15.0) for z in (ZL - 16.5, ZL + 16.5)]   # (x, z) on the deck; x +/-15 for the heel-fall moment
SPIKES = [(FX(28.0), ZL - 18.0), (FX(28.0), ZL + 18.0), (FX(-55.0), ZL - 18.0), (FX(-55.0), ZL + 18.0)]
LUG_X = [-62.0, -47.0, -32.0, 27.0, 39.0]      # foot-frame x of the transverse lugs (pad covers -22..22)


def planform(inset=0.0):
    """Teardrop outline in the XZ plane (u = x, v = z)."""
    return stadium((FX(S.X_TAIL_C), ZL), P.FOOT_R_TAIL - inset, (FX(S.X_TOE_C), ZL), P.FOOT_R_TOE - inset)


def pad_outline(inset=0.0):
    a = (P.PAD_L - P.PAD_W) / 2
    return stadium((FX(-a), ZL), P.PAD_W / 2 - inset, (FX(a), ZL), P.PAD_W / 2 - inset)


def papillae_points():
    import numpy as np
    pts = []
    keep = [(b[0], b[1], boss_r("M3") + 1.5) for b in BRACKET_SCREWS + SPIKES]
    for x in np.arange(S.X_HEEL + 6, S.X_TOE - 6, P.PAP_PITCH):
        hw = S.half_width(x) - P.RIM_W - 2.5
        if hw <= 0:
            continue
        if any(abs(x - lx) < P.LUG_W / 2 + 1.5 for lx in LUG_X):
            continue
        if abs(x) < P.PAD_L / 2 + 1.5:
            continue
        for z in np.arange(-hw, hw + 1e-6, P.PAP_PITCH):
            X, Z = FX(x), ZL + z
            if any(math.hypot(X - bx, Z - bz) < r for bx, bz, r in keep):
                continue
            pts.append((X, Z))
    if len(pts) > P.PAP_MAX:                       # keep the forefoot and heel bays, drop evenly
        step = len(pts) / P.PAP_MAX
        pts = [pts[int(i * step)] for i in range(P.PAP_MAX)]
    return pts


def foot_snowshoe():
    p = Part(N("foot_snowshoe"))
    # deck (top face = print bed face)
    p.slab("XZ", Y_DB, Y_DT, [planform()])
    # perimeter rim + transverse lugs + pad ring + insert bosses: everything on the sole side
    p.slab("XZ", Y_LT, Y_DB, [planform(), planform(P.RIM_W)])
    for lx in LUG_X:
        hw = S.half_width(lx) - P.RIM_W + 0.5
        p.slab("XZ", Y_LT, Y_DB, [rect(FX(lx) - P.LUG_W / 2, ZL - hw, FX(lx) + P.LUG_W / 2, ZL + hw)])
    p.slab("XZ", Y_LT - P.PAD_PROUD, Y_DB, [pad_outline(), pad_outline(P.LUG_W)])
    for b in BRACKET_SCREWS + SPIKES:
        p.slab("XZ", Y_LT - (P.PAD_PROUD if b in BRACKET_SCREWS else 0.0), Y_DB, [circle(b, boss_r("M3"))])
    # papillae field (Ø1 x 1.2 pins between the lugs)
    pap = papillae_points()
    p.slab("XZ", Y_DB - P.PAP_H, Y_DB, [circle(c, P.PAP_D / 2) for c in pap])
    # inserts from the top face (bed face; installed after printing, holes vertical)
    insert_hole(p, "XZ", Y_DT, -1, BRACKET_SCREWS, "M3")
    insert_hole(p, "XZ", Y_DT, -1, SPIKES, "M3")
    p.cut("XZ", Y_LT - 1.0, Y_DT, [circle(c, 1.6) for c in SPIKES])        # set-screw clearance below the insert
    # toe bevel: 45 deg, sole side, through the full width (Front-plane triangle)
    xt = FX(S.X_TOE)
    run = P.TOE_BEVEL_RUN
    p.cut("XY", ZL - P.FOOT_W, ZL + P.FOOT_W, [poly([(xt - run, Y_LT - 0.01), (xt + 2.0, Y_LT - 0.01), (xt + 2.0, Y_LT + run + 2.0)])])
    p._papillae = len(pap)
    return p


# ================================================================== ankle bracket (two mirror cheeks) + references
STOP_TOE = (FX(S.STOPS["toe_up"][0]), FY(S.STOPS["toe_up"][1]))
STOP_HEEL = (FX(S.STOPS["toe_down"][0]), FY(S.STOPS["toe_down"][1]))
CHEEK_IN_Z = (zc(P.Z_SHANK[0]) - P.GAP_MOVING - P.CHEEK_T, zc(P.Z_SHANK[0]) - P.GAP_MOVING)
CHEEK_OUT_Z = (zc(P.Z_SHANK[1]) + P.GAP_MOVING, zc(P.Z_SHANK[1]) + P.GAP_MOVING + P.CHEEK_T)
FLANGE_W = 12.0
FLANGE_T = 2.5                                # cheek flange on the deck
SPRING_Z = CHEEK_IN_Z[0] - (P.insert_depth("M3") - P.CHEEK_T) - 1.0   # springs hook under the stop-bolt heads


def cheek_web(p, z0, z1, holes):
    rb = boss_r("M3")
    p.slab("XY", z0, z1, [rect(FX(-22.0), Y_DT, FX(19.0), FY(2.0))] + holes)
    p.slab("XY", z0, z1, [circle(PIV, 9.5), circle(PIV, (P.ANKLE_PIN_D + 0.2) / 2)])
    for s in (STOP_TOE, STOP_HEEL):
        p.slab("XY", z0, z1, [circle(s, rb)])
    p.slab("XY", z0, z1, [rect(STOP_TOE[0] - rb, Y_DT, STOP_TOE[0] + rb, STOP_TOE[1])])


def ankle_bracket(side):
    inner = side == "in"
    p = Part(N(f"ankle_bracket_{side}"))
    z0, z1 = CHEEK_IN_Z if inner else CHEEK_OUT_Z
    cheek_web(p, z0, z1, [])
    # flange onto the deck (outward), screws down into the deck inserts
    fz = (z0 - FLANGE_W, z0) if inner else (z1, z1 + FLANGE_W)
    p.slab("XZ", Y_DT, Y_DT + FLANGE_T, [rect(FX(-21.0), fz[0], FX(21.0), fz[1])] +
           [circle(b, M3 / 2) for b in BRACKET_SCREWS if fz[0] < b[1] < fz[1]])
    p.cut("XY", z0, z1, [circle(PIV, (P.ANKLE_PIN_D + 0.2) / 2)])
    boss_h = P.insert_depth("M3") - P.CHEEK_T                                            # stop-bolt bosses: bearing length
    if inner:
        p.slab("XY", z0 - boss_h, z0, [circle(s, boss_r("M3")) for s in (STOP_TOE, STOP_HEEL)])
        p.cut("XY", z0 - boss_h, z1, [circle(s, M3 / 2) for s in (STOP_TOE, STOP_HEEL)])   # stop-bolt heads on this side
    else:
        p.slab("XY", z1, z1 + (P.insert_depth("M3") - P.CHEEK_T), [circle(s, boss_r("M3")) for s in (STOP_TOE, STOP_HEEL)])
        insert_hole(p, "XY", z0, +1, [STOP_TOE, STOP_HEEL], "M3")                            # inserts from the inner (bed) face
    return p


def ref_ankle_pin():
    p = Part(N("ref_ankle_pin"))
    p.slab("XY", CHEEK_IN_Z[0] - 5.0, CHEEK_OUT_Z[1], [circle(PIV, P.ANKLE_PIN_D / 2)])           # M5 x 25 + nylock
    p.slab("XY", CHEEK_OUT_Z[1], CHEEK_OUT_Z[1] + 3.5, [circle(PIV, 4.25)])
    return p


def ref_stop_sleeves():
    p = Part(N("ref_stop_sleeves"))
    for s in (STOP_TOE, STOP_HEEL):
        p.slab("XY", CHEEK_IN_Z[1], CHEEK_OUT_Z[0], [circle(s, P.STOP_SLEEVE_D / 2), circle(s, M3 / 2)])
    return p


def ref_springs():
    """Two small steel extension springs (centring pair): shank post -> toe-stop bolt head and
    shank post -> heel-stop bolt head (no extra posts on the deck)."""
    p = Part(N("ref_springs"))
    for d in (STOP_TOE, STOP_HEEL):
        p.slab("XY", SPRING_Z - 1.0, SPRING_Z + 1.0, [bar(SPRING_POST_SHANK, d, 2.5)])
    return p


def ref_knee_post_sleeves():
    p = Part(N("ref_knee_post_sleeves"))
    for c in (POST_UP, POST_LO):
        p.slab("XY", POST_Z[0], POST_Z[1], [circle(c, POST_SLEEVE_R), circle(c, M5 / 2)])
    return p


# ================================================================== knee-motor adapter (unchanged)
def hip_adapter():
    p = Part(N("hip_shaft_adapter"))
    z0, z1 = zc(P.Z_ADAPTER[0]), zc(P.Z_ADAPTER[1])
    p.slab("XY", z0, z1, [circle((0, 0), 18.0), circle((0, 0), (P.AXLE_D + 0.2) / 2)] + bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, M25))
    p.cut("XY", z1 - 2.5, z1, bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, 5.0))
    p.cut("XY", z0, z0 + P.MOTOR_BOSS_H + 0.2, [circle((0, 0), (P.MOTOR_BOSS_D + P.FIT_LOCATE) / 2)])
    p.cut("XY", z0 + P.MOTOR_BOSS_H + 0.2, z0 + P.MOTOR_BOSS_H + 3.8, [hexagon((0, 0), P.NUT_AF["M5"])])
    p.slab("XY", z1, z1 + 0.5, [circle((0, 0), 4.5), circle((0, 0), (P.AXLE_D + 0.2) / 2)])
    return p


# ================================================================== hip module: plate (+ bridge column) and two flange L-brackets
FX0 = P.FLANGE_X
FX1 = P.FLANGE_X + P.FLANGE_T
ZP_IN0, ZP_IN1 = zc(P.Z_PLATE_IN[0]), zc(P.Z_PLATE_IN[1])
ZP_PAD = ZP_IN1 + 2.0                         # plate thickened to 7 mm under the flange feet (insert depth)
FOOT_Z = (ZP_PAD, ZP_PAD + 6.0)
FL_Z1 = zc(P.Z_THIGH_IN[0]) - P.ABAD_CLEAR
FL_Y = 26.0
BRIDGE = (-11.0, 11.0, P.BRIDGE_Y[0], 40.0)  # x0, x1, y0, y1 (column from the plate to the outboard plate)
PLATE_TOP = P.BRIDGE_Y[0] - 0.2               # inboard plate stops under the C-frame column
TAB_Z0 = ZP_IN0 - 4.0                          # C-frame tab plate on the inboard plate's inboard face
FOOT_SCREWS = {s: [(s * 29.3, -17.0), (s * 29.3, 17.0)] for s in (-1, 1)}   # bosses stay inside |x| <= 36
BACK_PHASE = 126.0
ABAD_PHASE = 0.0                              # ab/ad output pattern rotated so no hole falls below the flange foot


def module_plate_inboard():
    p = Part(N("module_plate_inboard"))
    p.slab("XY", ZP_IN0, ZP_IN1, [rect(-FX1, -30.0, FX1, PLATE_TOP), circle((0, 0), 10.0)] +
           bolt_circle((0, 0), P.MOTOR_BACK_PCD, P.MOTOR_BACK_N, M25, BACK_PHASE))
    for s in (-1, 1):
        x0, x1 = sorted((s * 27.5, s * FX1))
        p.slab("XY", ZP_IN0, ZP_PAD, [rect(x0, -FL_Y, x1, FL_Y)])
        for c in FOOT_SCREWS[s]:
            p.slab("XY", ZP_IN0, ZP_PAD, [circle(c, boss_r("M3"))])
        insert_hole(p, "XY", ZP_PAD, -1, FOOT_SCREWS[s], "M3")
    return p


def module_frame_outboard():
    """C-frame: outboard plate (hip-motor back face) + bridge column over the top (hip hard stop,
    +/-107 deg) + inboard tab plate clamped to the inboard plate by the 5 knee-motor back screws
    (M2.5 x 12). One part, printed lying on the column's top face, both plates standing up."""
    p = Part(N("module_frame_outboard"))
    zo0, zo1 = zc(P.Z_PLATE_OUT[0]), zc(P.Z_PLATE_OUT[1])
    motor_holes = bolt_circle((0, 0), P.MOTOR_BACK_PCD, P.MOTOR_BACK_N, M25, BACK_PHASE)
    # outboard plate: disc + rectangle up to the column (stands on the column face when printed)
    cable = teardrop((0, 0), 10.0, (0.0, -1.0))          # printed on the +y face: -y is 'up'
    p.slab("XY", zo0, zo1, [circle((0, 0), 30.0)])
    p.slab("XY", zo0, zo1, [rect(-30.0, 0.0, 30.0, BRIDGE[3])])
    p.cut("XY", zo0, zo1, [cable] + motor_holes)
    # inboard tab: 45-deg chamfered outline (self-supporting when printed on the column face; its
    # corners stay inside r 38.4 so the ab/ad sweep clears the back block and the front wall foot)
    p.slab("XY", TAB_Z0, ZP_IN0, [poly([(BRIDGE[0], BRIDGE[3]), (BRIDGE[1], BRIDGE[3]), (BRIDGE[1], 34.0), (30.0, 15.0),
                                        (30.0, -30.0), (BRIDGE[1], -30.0), (BRIDGE[1], -BRIDGE[3]), (BRIDGE[0], -BRIDGE[3]),
                                        (BRIDGE[0], -30.0), (-30.0, -30.0), (-30.0, 15.0), (BRIDGE[0], 34.0)]),
                                  teardrop((0, 0), 10.0, (0.0, -1.0))] + motor_holes)   # + lug at the bottom: mirror of the column (ab/ad stop)
    p.slab("XY", TAB_Z0, zo0, [rect(BRIDGE[0], BRIDGE[2], BRIDGE[1], BRIDGE[3])])
    return p


def module_flange(side):
    back = side == "back"
    p = Part(N(f"module_flange_{side}"))
    s = -1 if back else 1
    x0, x1 = sorted((s * FX0, s * FX1))
    holes = bolt_circle((0, 0), P.MOTOR_OUT_PCD, 3, M25, ABAD_PHASE) if back else [circle((0, 0), (P.STUB_D + 0.2) / 2)]
    p.slab("YZ", x0, x1, [rect(FOOT_Z[0], -FL_Y, FL_Z1, FL_Y)] + holes)
    if back:
        p.cut("YZ", x0, x1, [circle((0, 0), (P.MOTOR_BOSS_D + P.FIT_LOCATE) / 2)])      # through: no bed-face bridge
    else:
        p.cut("YZ", x0, x0 + 3.6, [hexagon((0, 0), P.NUT_AF["M5"], 30)])
    # foot on the plate's thickened zone: clearance holes, screws down into the plate inserts
    fx0, fx1 = sorted((s * 27.5, s * FX1))
    p.slab("XY", FOOT_Z[0], FOOT_Z[1], [rect(fx0, -FL_Y, fx1, FL_Y)])
    for c in FOOT_SCREWS[s]:
        p.slab("XY", FOOT_Z[0], FOOT_Z[1], [circle(c, boss_r("M3")), circle(c, M3 / 2)])
    p.cut("XY", FOOT_Z[0], FOOT_Z[1], [circle(c, M3 / 2) for c in FOOT_SCREWS[s]])
    return p


def hip_shroud_out():
    """Printed shroud over the hip motor + outboard plate: 5 mm closed-cell foam liner inside, cable
    gland Ø10 over the plate's Ø20 cable hole. Clamped by the 5 hip-motor back screws (M2.5 x 12)."""
    p = Part(N("hip_shroud_out"))
    z1 = zc(P.Z_PLATE_OUT[1])
    zb = zc(P.Z_HIP_MOTOR[0]) + 2.5
    r_in, r_out = P.MOTOR_D / 2 + 5.0, P.MOTOR_D / 2 + 10.0
    top_in, top_out = BRIDGE[3] + 1.5, BRIDGE[3] + 4.0
    for loops in ([circle((0, 0), r_out)], [rect(-r_out, 0.0, r_out, top_out)]):
        p.slab("XY", zb, z1 + 3.0, loops)
    p.cut("XY", zb, z1, [circle((0, 0), r_in)])
    p.cut("XY", zb, z1, [rect(-r_in, 0.0, r_in, top_in)])
    p.cut("XY", z1, z1 + 3.0, [circle((0.0, 0.0), 5.0)] + bolt_circle((0, 0), P.MOTOR_BACK_PCD, P.MOTOR_BACK_N, M25, BACK_PHASE))
    return p


# ================================================================== body bracket (ab/ad stage)
ABAD_X1 = -FX1
ABAD_X0 = ABAD_X1 - P.MOTOR_L
BB_BACK = (ABAD_X0 - P.BRACKET_T, ABAD_X0)
BB_FRONT = (FX1 + P.GAP_MOVING, FX1 + P.GAP_MOVING + P.BRACKET_T)
SP0, SP1 = -54.0, -46.0                         # spine 8 mm (M3 insert depth), top unchanged (ab/ad sweep)
BLK = (BB_BACK[1] + 0.2, -23.5, SP1 + 6.0)      # back block x0, x1, top z (M5 depth)
WALL_BACK_SCREWS = [(-62.0, -9.5), (-62.0, 9.5)]
RIG_SCREWS = [(-48.0, -19.0), (-48.0, 19.0), (-34.0, -19.0), (-34.0, 19.0)]   # y +/-19 for the roll moment (insert_check)
BACK_W = 30.0                            # spine back block + rig plate widened to +/-30 there
WALL_FRONT_SCREWS = [(x, y) for x in (20.0, 29.5) for y in (-17.3, 17.3)]
SPINE_FRONT_W = 24.0                    # spine widened under the front wall foot (screw spacing)
FRONT_FOOT_X0 = 17.0                    # front wall foot inner edge (C-frame tab chamfer passes x 14 at -32 deg ab/ad)
RIG_Z = (SP0 - 8.0, SP0)
ABAD_STOPS = {"plus": None, "minus": None}      # filled by tune_abad_stops() (needs the module STLs)


def body_bracket_back():
    p = Part(N("body_bracket_back"))
    p.slab("YZ", *BB_BACK, [circle((0, 0), P.BB_BACK_R)])
    p.slab("YZ", *BB_BACK, [rect(BLK[2], -20.0, -15.0, 20.0)])
    p.cut("YZ", BB_BACK[0], BB_BACK[1], [circle((0, 0), 10.0)] + bolt_circle((0, 0), P.MOTOR_BACK_PCD, P.MOTOR_BACK_N, M25, 90))
    p.slab("XY", BLK[2], BLK[2] + 6.0, [rect(BB_BACK[1] - 0.01, -20.0, -51.0, 20.0)] + [circle(c, M5 / 2) for c in WALL_BACK_SCREWS])
    return p


def body_bracket_front():
    p = Part(N("body_bracket_front"))
    p.slab("YZ", *BB_FRONT, [circle((0, 0), P.BB_FRONT_R)])
    p.slab("YZ", *BB_FRONT, [rect(SP1, -SPINE_FRONT_W, -12.0, SPINE_FRONT_W)])      # as wide as its foot (print support)
    seat = (P.B625["D"] + P.FIT_BEARING) / 2
    p.cut("YZ", BB_FRONT[0], BB_FRONT[1], [circle((0, 0), 6.0)])
    p.cut("YZ", BB_FRONT[0], BB_FRONT[1] - 0.5, [circle((0, 0), seat)])
    p.slab("XY", SP1, SP1 + 6.0, [rect(FRONT_FOOT_X0, -SPINE_FRONT_W, BB_FRONT[0] + 0.01, SPINE_FRONT_W)] + [circle(c, M3 / 2) for c in WALL_FRONT_SCREWS])
    return p


def body_spine():
    p = Part(N("body_spine"))
    x0, x1 = BB_BACK[1] + 0.2, BB_FRONT[0] - 0.2
    p.slab("XY", SP0, SP1, [rect(x0, -20.0, x1, 20.0), slot((-12.0, -4.0), (-12.0, 4.0), 2.0), slot((2.0, -4.0), (2.0, 4.0), 2.0)])
    p.slab("XY", SP0, SP1, [rect(14.0, -SPINE_FRONT_W, x1, SPINE_FRONT_W)])
    p.slab("XY", SP0, SP1, [rect(BLK[0], -BACK_W, BLK[1], BACK_W)])
    p.slab("XY", SP1, BLK[2], [rect(BLK[0], -BACK_W, BLK[1], BACK_W)])
    insert_hole(p, "XY", BLK[2], -1, WALL_BACK_SCREWS, "M5")
    insert_hole(p, "XY", SP0, +1, RIG_SCREWS, "M5")
    insert_hole(p, "XY", SP1, -1, WALL_FRONT_SCREWS, "M3")
    for k, v in ABAD_STOPS.items():
        if v:
            (ya, yb), top = v[0], v[1]
            p.slab("XY", SP1, top, [rect(-20.0, ya, 6.0, yb)])
    return p


def rig_plate():
    p = Part(N("rig_plate"))
    x0, x1 = BB_BACK[1] + 0.2, BB_FRONT[0] - 0.2
    g = P.RIG_SLOT_GRID / 2
    loops = [rect(x0, -20.0, x1, 20.0)] + [slot((x - 3, y), (x + 3, y), P.CLR["M5"] / 2) for x in (-12.0, -12.0 + 2 * g) for y in (-g, g)]
    p.slab("XY", RIG_Z[0], RIG_Z[1], loops)
    p.slab("XY", RIG_Z[0], RIG_Z[1], [rect(BLK[0], -BACK_W, BLK[1], -20.0)])
    p.slab("XY", RIG_Z[0], RIG_Z[1], [rect(BLK[0], 20.0, BLK[1], BACK_W)])
    p.cut("XY", RIG_Z[0], RIG_Z[1], [circle(c, M5 / 2) for c in RIG_SCREWS])
    p.cut("XY", RIG_Z[0], RIG_Z[0] + 5.2, [circle(c, 5.0) for c in RIG_SCREWS])        # M5 heads flush with the rig face
    return p


# ================================================================== reference bodies
def motor(name, axis, n0, n1, out_at_high):
    p = Part(N(name))
    body = [circle((0, 0), P.MOTOR_D / 2)]
    step = [circle((0, 0), 37.0 / 2)]
    if out_at_high:
        p.slab(axis, n0, n1 - 1.0, body); p.slab(axis, n1 - 1.0, n1, step)
    else:
        p.slab(axis, n0 + 1.0, n1, body); p.slab(axis, n0, n0 + 1.0, step)
    return p


def ref_hip_pulley():
    p = Part(N("ref_pulley_20T_kit"))
    a0, a1 = zc(P.Z_CHANNEL[0]) + 0.5, zc(P.Z_CHANNEL[1]) - 0.5
    p.slab("XY", a0, a1, [circle((0, 0), P.OD_HIP / 2), circle((0, 0), P.AXLE_D / 2)])
    return p


def ref_ground():
    p = Part(N("ref_ground"))
    y = FY(S.Y_PAD_TIP) if P.FOOT == "snowshoe" else PIV[1] - P.FOOT_D / 2
    p.slab("XZ", y - 4.0, y, [rect(-200.0, -40.0, 200.0, 130.0)])
    return p


# ================================================================== fit + insert coupon
def coupon():
    p = Part(N("coupon_fit_test"))
    W, H = 175.0, 80.0
    loops = [rect(0, 0, W, H)]
    x = 12.0
    for d in (0.10, 0.15, 0.20, 0.25):                 # Ø15 boss pocket
        loops.append(circle((x, 15.0), (15.0 + d) / 2)); x += 19.0
    for d in (0.05, 0.10, 0.15, 0.20):                 # 625(-2RS) seats
        loops.append(circle((x, 15.0), (16.0 + d) / 2)); x += 19.5
    x = 10.0
    for d in (0.2, 0.3, 0.4):                          # Ø5 axle / ankle bushing
        loops.append(circle((x, 38.0), (5.0 + d) / 2)); x += 9.0
    for dd in (M25, M3, M5):
        loops.append(circle((x, 38.0), dd / 2)); x += 9.0
    loops.append(hexagon((x + 2, 38.0), P.NUT_AF["M5"])); x += 14.0
    p.slab("XY", 0.0, 4.0, loops)
    # insert bosses: M3 holes at 4.0 / 4.1 / 4.2 in walls 2 / 3 / 4; M5 at 6.4 / 6.5 / 6.6
    bx = 100.0
    for i, d in enumerate((4.0, 4.1, 4.2)):
        for j, wall in enumerate((2.0, 3.0, 4.0)):
            c = (bx + i * 14.0, 30.0 + j * 15.0)
            p.slab("XY", 4.0, 4.0 + 7.0, [circle(c, d / 2 + wall), circle(c, d / 2)])
    for i, d in enumerate((6.4, 6.5, 6.6)):
        c = (20.0 + i * 22.0, 62.0)
        p.slab("XY", 4.0, 4.0 + 11.1, [circle(c, d / 2 + 4.0), circle(c, d / 2)])
    # lug + papillae sample
    p.slab("XY", 4.0, 4.0 + P.LUG_H, [rect(88.0, 52.0, 88.0 + P.LUG_W, 76.0)])
    p.slab("XY", 4.0, 4.0 + P.PAP_H, [circle((150.0 + (k % 6) * P.PAP_PITCH, 30.0 + (k // 6) * P.PAP_PITCH), P.PAP_D / 2) for k in range(48)])
    # ice-spike sample: insert hole + set-screw clearance through
    p.cut("XY", 0.0, 4.0, [circle((160.0, 66.0), 1.6)])
    p.slab("XY", 4.0, 4.0 + P.LUG_H, [circle((160.0, 66.0), boss_r("M3")), circle((160.0, 66.0), 1.6)])
    return p


# named reference axes for mates (body frame)
HIP_AX = ("HIP_AXIS", "Z", 0.0, 0.0)
KNEE_AX = ("KNEE_AXIS", "Z", KNEE[0], KNEE[1])
ANKLE_AX = ("ANKLE_AXIS", "Z", PIV[0], PIV[1])
ABAD_AX = ("ABAD_AXIS", "X", 0.0, 0.0)
AXES = {"module_plate_inboard": [HIP_AX, ABAD_AX], "thigh_outer": [HIP_AX, KNEE_AX], "shank": [KNEE_AX, ANKLE_AX],
        "shank_ball": [KNEE_AX], "hip_shaft_adapter": [HIP_AX], "body_spine": [ABAD_AX],
        "ankle_bracket_out": [ANKLE_AX]}

BUILDERS = {
    "thigh_outer": thigh_outer, "thigh_cover": thigh_cover, "knee_pulley": knee_pulley, "shank": shank,
    "foot_snowshoe": foot_snowshoe, "ankle_bracket_in": lambda: ankle_bracket("in"),
    "ankle_bracket_out": lambda: ankle_bracket("out"), "ref_ankle_pin": ref_ankle_pin,
    "ref_stop_sleeves": ref_stop_sleeves, "ref_springs": ref_springs, "ref_knee_post_sleeves": ref_knee_post_sleeves,
    "hip_shaft_adapter": hip_adapter, "module_plate_inboard": module_plate_inboard,
    "module_flange_back": lambda: module_flange("back"), "module_flange_front": lambda: module_flange("front"),
    "module_frame_outboard": module_frame_outboard, "hip_shroud_out": hip_shroud_out,
    "body_bracket_back": body_bracket_back, "body_bracket_front": body_bracket_front, "body_spine": body_spine,
    "rig_plate": rig_plate,
    "ref_motor_hip": lambda: motor("ref_motor_hip", "XY", zc(P.Z_HIP_MOTOR[0]), zc(P.Z_HIP_MOTOR[1]), False),
    "ref_motor_knee": lambda: motor("ref_motor_knee", "XY", zc(P.Z_KNEE_MOTOR[0]), zc(P.Z_KNEE_MOTOR[1]), True),
    "ref_motor_abad": lambda: motor("ref_motor_abad", "YZ", ABAD_X0, ABAD_X1, True),
    "ref_pulley_20T_kit": ref_hip_pulley, "ref_ground": ref_ground, "coupon_fit_test": coupon,
    "foot_ball": foot_ball, "shank_ball": shank_ball,
}
SNOWSHOE_ONLY = {"foot_snowshoe", "ankle_bracket_in", "ankle_bracket_out", "ref_ankle_pin", "ref_stop_sleeves", "ref_springs"}
BALL_ONLY = {"foot_ball", "shank_ball"}


def default_names():
    names = [n for n in BUILDERS if n not in (BALL_ONLY if P.FOOT == "snowshoe" else SNOWSHOE_ONLY)]
    if P.FOOT == "ball":
        names = [n for n in names if n != "shank"]
    return names


def load_abad_stops():
    path = os.path.join(PARTS, "abad_stops.json")
    if os.path.exists(path):
        d = json.load(open(path))
        for k in ABAD_STOPS:
            ABAD_STOPS[k] = d.get(k)


def main(names):
    from swlib import sw
    sw().CloseAllDocuments(True)
    load_abad_stops()
    report = {}
    path = os.path.join(PARTS, "parts_report.json" if P.SKELETON else "parts_report_solid.json")
    if os.path.exists(path):
        report = json.load(open(path))
    for n in names or default_names():
        print("building", N(n), flush=True)
        p = BUILDERS[n]()
        for ax in AXES.get(n, []):
            p.axis(*ax)
        vol = p.volume_mm3()
        report[p.name] = {"bbox_mm": p.bbox(), "volume_mm3": round(vol), "solid_pla_g": round(vol * P.PLA_RHO * 1000, 1),
                          "skeleton": P.SKELETON}
        if hasattr(p, "_papillae"):
            report[p.name]["papillae"] = p._papillae
        if P.SKELETON:
            p.save(PARTS, STL)
        p.close()
        print("  ", report[p.name], flush=True)
    json.dump(report, open(path, "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1:])
