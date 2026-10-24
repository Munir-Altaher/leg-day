"""Swept clearance check for the SNOW LEG, from the exported STLs.

Moving parts are sampled as point clouds (STL vertices + points every 2 mm along every
triangle edge, ≤ 40k points per part) and moved through the joint ranges; the hip-module
parts are modelled as exact envelopes (boxes / cylinders, from params and build_parts),
the thigh body as its stadium outline × its z range (for the wide foot at deep knee fold).
  - knee sweep at q1 = 0 and a hip × knee grid (5° steps) with the ankle at both stops + neutral
  - ab/ad sweep of the whole module + leg against the body bracket, spine, rig plate, ab/ad motor
  - solves the two ab/ad stop blocks on the spine (module lands on them at ±(limit + 3°))
Writes analysis/out/clearance.md and cad/parts/abad_stops.json.
Run after cad/build_parts.py:  python analysis/clearance_check.py
"""
import json
import math
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "cad"))
import params as P  # noqa: E402
import snowgeo as SG  # noqa: E402
import build_parts as B  # noqa: E402

STL = os.path.join(ROOT, "cad", "stl", "assembly_pose")
OUT = os.path.join(HERE, "out")
SHIFT = 300.0
GAP = 0.5                       # required clearance, mm


def load(name, step=2.0, keep=40000):
    with open(os.path.join(STL, P.FILE_PREFIX + name + ".STL"), "rb") as f:
        data = f.read()
    if data[:5] == b"solid":                     # SolidWorks writes ASCII STL by default
        v = [ln.split()[1:4] for ln in data.decode("ascii", "ignore").splitlines() if ln.strip().startswith("vertex")]
        tri = np.array(v, float).reshape(-1, 3, 3)
    else:
        n = struct.unpack("<I", data[80:84])[0]
        tri = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"].reshape(-1, 3, 3).astype(float)
    if np.abs(tri).max() < 5:                    # exported in metres
        tri *= 1000.0
    tri -= SHIFT
    pts = [tri.reshape(-1, 3)]
    for i, j in ((0, 1), (1, 2), (2, 0)):
        a, b = tri[:, i], tri[:, j]
        L = np.linalg.norm(b - a, axis=1)
        k = np.maximum(1, (L / step).astype(int))
        for kk in range(1, k.max()):
            sel = k > kk
            t = kk / k[sel]
            pts.append(a[sel] + (b[sel] - a[sel]) * t[:, None])
    pts = np.unique(np.round(np.vstack(pts), 2), axis=0)
    if len(pts) > keep:
        pts = pts[np.random.default_rng(0).choice(len(pts), keep, replace=False)]
    return pts


def rz(p, ang, c=(0, 0)):
    c0, s0 = math.cos(ang), math.sin(ang)
    x, y = p[:, 0] - c[0], p[:, 1] - c[1]
    return np.column_stack([c[0] + x * c0 - y * s0, c[1] + x * s0 + y * c0, p[:, 2]])


def rx(p, ang):
    c0, s0 = math.cos(ang), math.sin(ang)
    return np.column_stack([p[:, 0], p[:, 1] * c0 - p[:, 2] * s0, p[:, 1] * s0 + p[:, 2] * c0])


def in_box(p, lo, hi, g=GAP):
    lo, hi = np.array(lo) - g, np.array(hi) + g
    return np.any(np.all((p > lo) & (p < hi), axis=1))


def in_cyl_z(p, r, z0, z1, g=GAP):
    return np.any((np.hypot(p[:, 0], p[:, 1]) < r + g) & (p[:, 2] > z0 - g) & (p[:, 2] < z1 + g))


def in_cyl_x(p, r, x0, x1, g=GAP):
    return np.any((np.hypot(p[:, 1], p[:, 2]) < r + g) & (p[:, 0] > x0 - g) & (p[:, 0] < x1 + g))


THIGH_PATH = None


def in_thigh_body(p, g=GAP):
    """Points inside the thigh's stadium outline (thigh frame) within the thigh's z range."""
    global THIGH_PATH
    from matplotlib.path import Path
    if THIGH_PATH is None:
        THIGH_PATH = Path(SG.thigh_outline_poly())
    z0, z1 = P.zc(P.Z_THIGH_IN[0]), P.zc(P.Z_THIGH_OUT[1])
    sel = (p[:, 2] > z0 - g) & (p[:, 2] < z1 + g)
    if not sel.any():
        return False
    q = p[sel][:, :2]
    return bool(THIGH_PATH.contains_points(q, radius=g).any() or THIGH_PATH.contains_points(q, radius=-g).any())


def main():
    zc = P.zc
    thigh = np.vstack([load("thigh_outer"), load("thigh_cover")])
    shank = load("shank" if P.FOOT == "snowshoe" else "shank_ball")
    sleeves = load("ref_knee_post_sleeves")
    foot_names = (("foot_snowshoe", "ankle_bracket_in", "ankle_bracket_out", "ref_ankle_pin", "ref_stop_sleeves", "ref_springs")
                  if P.FOOT == "snowshoe" else ("foot_ball",))
    foot = np.vstack([load(n) for n in foot_names])
    PIV = SG.PIVOT
    boxes_mod = {
        "C-frame column": ((B.BRIDGE[0], B.BRIDGE[2], B.TAB_Z0), (B.BRIDGE[1], B.BRIDGE[3], zc(P.Z_PLATE_OUT[1]))),
        "C-frame outboard plate": ((-30.0, -30.0, zc(P.Z_PLATE_OUT[0])), (30.0, B.BRIDGE[3], zc(P.Z_PLATE_OUT[1]))),
        "hip shroud (envelope)": ((-36.5, 0.0, zc(P.Z_HIP_MOTOR[0]) + 2.5), (36.5, B.BRIDGE[3] + 4.0, zc(P.Z_PLATE_OUT[1]) + 3.0)),
        "inboard plate + tab": ((-B.FX1, -30.0, B.TAB_Z0), (B.FX1, P.BRIDGE_Y[0], B.ZP_PAD)),
        "flange back": ((-B.FX1, -B.FL_Y, B.FOOT_Z[0]), (-B.FX0, B.FL_Y, B.FL_Z1)),
        "flange front": ((B.FX0, -B.FL_Y, B.FOOT_Z[0]), (B.FX1, B.FL_Y, B.FL_Z1)),
    }
    cyl_mod = {"hip motor": (P.MOTOR_D / 2, zc(P.Z_HIP_MOTOR[0]), zc(P.Z_HIP_MOTOR[1])),
               "knee motor": (P.MOTOR_D / 2, zc(P.Z_KNEE_MOTOR[0]), zc(P.Z_KNEE_MOTOR[1])),
               "hip shroud": (P.MOTOR_D / 2 + 10.0, zc(P.Z_HIP_MOTOR[0]) + 2.5, zc(P.Z_PLATE_OUT[1]) + 3.0)}

    def lower(q2, alpha):
        f = rz(foot, math.radians(-alpha), PIV) if P.FOOT == "snowshoe" else foot
        return rz(np.vstack([shank, sleeves, f]), math.radians(q2), B.KNEE), rz(f, math.radians(q2), B.KNEE)

    def hits(q1, q2, alpha):
        low, f_only = lower(q2, alpha)
        out = []
        if in_thigh_body(f_only):
            out.append("thigh body")
        W = rz(np.vstack([low, thigh]), math.radians(q1))
        for k, (lo, hi) in boxes_mod.items():
            if in_box(W, lo, hi):
                out.append(k)
        L = rz(low, math.radians(q1))
        for k, (r, a_, b_) in cyl_mod.items():
            if in_cyl_z(L, r, a_, b_):
                out.append(k)
        return out

    alphas = [SG.ALPHA_MIN, SG.ALPHA_NEUTRAL, SG.ALPHA_MAX] if P.FOOT == "snowshoe" else [0.0]
    lines = ["# Swept clearance check — snow leg (analysis/clearance_check.py)\n",
             f"Required clearance {GAP} mm. Moving parts = STL point clouds (2 mm edge sampling, ≤ 40k points per part); module "
             "parts = exact envelopes; thigh body = stadium outline × its z range. The knee-stop sleeves are meant to touch the "
             "thigh at 13° / 152° (outside the 15–150° firmware range).\n",
             f"Ankle angles tested: {', '.join(f'{a:.1f}°' for a in alphas)} (toe-up stop, neutral, toe-down stop).\n"]
    knee_ok = True
    for al in alphas:
        blocked = [(q, hits(0.0, q, al)) for q in np.arange(P.Q2_LIM[0], P.Q2_LIM[1] + 0.1, 1.0)]
        blocked = [b for b in blocked if b[1]]
        knee_ok &= not blocked
        lines.append(f"- Knee sweep, q1 = 0, α = {al:.1f}°: " +
                     (f"clear {P.Q2_LIM[0]:.0f}–{P.Q2_LIM[1]:.0f}° -> PASS" if not blocked else
                      f"blocked from {blocked[0][0]:.0f}° by {blocked[0][1]} -> FAIL"))
    grid_fail = []
    for al in alphas:
        for q2 in np.arange(P.Q2_LIM[0], P.Q2_LIM[1] + 0.1, 5.0):
            for q1 in np.arange(P.Q1_LIM[0], P.Q1_LIM[1] + 0.1, 5.0):
                h = hits(q1, q2, al)
                if h:
                    grid_fail.append((round(al, 1), float(q1), float(q2), h))
    n_poses = len(alphas) * len(np.arange(P.Q2_LIM[0], P.Q2_LIM[1] + 0.1, 5.0)) * len(np.arange(P.Q1_LIM[0], P.Q1_LIM[1] + 0.1, 5.0))
    lines.append(f"\n- Hip × knee grid ({n_poses} poses: q1 {P.Q1_LIM}, q2 {P.Q2_LIM}, 5° steps × 3 ankle angles): "
                 f"{len(grid_fail)} colliding -> {'PASS' if not grid_fail else 'FAIL'}")
    if grid_fail:
        from collections import Counter
        cnt = Counter(h for g in grid_fail for h in g[3])
        lines.append("  obstacles: " + "; ".join(f"{k} ×{v}" for k, v in cnt.most_common()))
        lines.append(f"  smallest colliding knee angle: {min(g[2] for g in grid_fail):.0f}°; example {grid_fail[0]}")

    # ---------------- ab/ad: module + leg about X vs body bracket; solve the stop blocks
    module = np.vstack([load(n) for n in ("module_plate_inboard", "module_flange_back", "module_flange_front",
                                          "module_frame_outboard", "hip_shroud_out", "ref_motor_hip",
                                          "ref_motor_knee", "hip_shaft_adapter")])
    static_boxes = [
        ((B.BB_BACK[0], -20.0, B.BLK[2]), (B.BB_BACK[1], 20.0, -15.0)),
        ((B.BB_BACK[1], -20.0, B.BLK[2]), (-51.0, 20.0, B.BLK[2] + 6.0)),
        ((B.BB_FRONT[0], -B.SPINE_FRONT_W, B.SP1), (B.BB_FRONT[1], B.SPINE_FRONT_W, -12.0)),
        ((B.FRONT_FOOT_X0, -B.SPINE_FRONT_W, B.SP1), (B.BB_FRONT[0], B.SPINE_FRONT_W, B.SP1 + 6.0)),
        ((B.BB_BACK[1], -20.0, B.RIG_Z[0]), (B.BB_FRONT[0], 20.0, B.SP1)),
        ((14.0, -B.SPINE_FRONT_W, B.RIG_Z[0]), (B.BB_FRONT[0], B.SPINE_FRONT_W, B.SP1)),
        ((B.BLK[0], -B.BACK_W, B.SP1), (B.BLK[1], B.BACK_W, B.BLK[2])),
    ]
    static_cyl = [(P.BB_BACK_R, *B.BB_BACK), (P.BB_FRONT_R, *B.BB_FRONT), (P.MOTOR_D / 2, B.ABAD_X0, B.ABAD_X1 - 1.0)]
    q_stop = max(abs(v) for v in P.Q0_LIM) + 3.0

    def first_contact(top, ya, yb, sgn):
        """First |q0| (deg) at which the module touches a block x -20..6, y ya..yb, z up to `top`."""
        lo_, hi_ = np.array([-20.0, ya, B.SP1 - 1.0]), np.array([6.0, yb, top])
        for q0 in np.arange(0.0, 90.0, 0.5):
            if in_box(rx(module, math.radians(sgn * q0)), lo_, hi_, 0.0):
                return q0
        return 90.0

    stops = {}
    for name, sgn in (("plus", 1), ("minus", -1)):
        best = None
        for ya, yb in ((-20.0, -8.0), (-14.0, -2.0), (-8.0, 4.0), (-4.0, 8.0), (2.0, 14.0), (8.0, 20.0)):
            # tallest block that is first touched at q_stop in its own direction and not before q_stop the other way
            for top in np.arange(B.SP1 + 25.0, B.SP1 + 1.0, -0.25):
                own, other = first_contact(top, ya, yb, sgn), first_contact(top, ya, yb, -sgn)
                if own >= q_stop - 0.5 and other >= q_stop:
                    if own <= q_stop + 1.0 and (best is None or top > best[1]):
                        best = ((ya, yb), float(top), float(own), float(other))
                    break
        stops[name] = best
    with open(os.path.join(ROOT, "cad", "parts", "abad_stops.json"), "w") as f:
        json.dump(stops, f, indent=1)
    lines.append("\n## Ab/ad physical stops (blocks on the spine, x −20…6)\n")
    for k, v in stops.items():
        lines.append(f"- {k}: " + (f"block y {v[0]}, top z {v[1]:.2f} → module lands on it at {'+' if k == 'plus' else '−'}{v[2]:.1f}° "
                                   f"(other direction first touches it at {v[3]:.1f}°)" if v else "no block position found -> firmware limit only"))
        if v:
            static_boxes.append(((-20.0, v[0][0], B.SP1), (6.0, v[0][1], v[1])))
    lines.append("\n## Ab/ad sweep (module + leg about X) at several hip / knee / ankle poses\n")
    res = []
    for q1d, q2d in ((-10, 20), (0, 90), (-60, 120), (60, 30), (-100, 150), (100, 15), (-100, 15), (100, 150)):
        for al in (alphas[0], alphas[-1]):
            low, _ = lower(q2d, al)
            mov = np.vstack([module, rz(np.vstack([thigh, low]), math.radians(q1d))])
            lim = [None, None]
            for sgn, idx in ((1, 1), (-1, 0)):
                last = 0.0
                for q0 in np.arange(0.0, 121.0, 1.0):
                    pts = rx(mov, math.radians(sgn * q0))
                    if any(in_box(pts, lo, hi, 0.2) for lo, hi in static_boxes) or \
                            any(in_cyl_x(pts, r, a, b, 0.2) for r, a, b in static_cyl):
                        break
                    last = sgn * q0
                lim[idx] = last
            res.append(lim)
            lines.append(f"- q1 = {q1d:4d}, q2 = {q2d:3d}, α = {al:5.1f}: ab/ad free [{lim[0]:.0f}, {lim[1]:.0f}]°")
    lo = max(r[0] for r in res)
    hi = min(r[1] for r in res)
    ok_ab = lo <= P.Q0_LIM[0] and hi >= P.Q0_LIM[1]
    lines.append(f"\nAb/ad collision-free over all tested poses: [{lo:.0f}, {hi:.0f}]° (firmware {P.Q0_LIM}; stops at ±{q_stop:.0f}°) -> "
                 f"{'PASS' if ok_ab else 'FAIL'}")
    belt_t = 1.38
    c_knee = P.THIGH_CH_R_KNEE - (P.PD_KNEE / 2 + belt_t - 0.254)
    c_hip = P.THIGH_CH_R_HIP - (P.PD_HIP / 2 + belt_t - 0.254)
    fl = P.THIGH_CH_R_KNEE - (P.OD_KNEE / 2 + 1.5)
    lines.append(f"\n## Belt channel\n- belt back to channel wall: knee {c_knee:.2f} mm, hip {c_hip:.2f} mm (≥ 1 mm); "
                 f"knee-pulley flange to wall {fl:.2f} mm")
    lines.append(f"- seam lip on the cover z {zc(P.Z_THIGH_IN[1]):.1f}…{zc(P.Z_THIGH_IN[1]) + B.LIP_H:.1f} vs knee-pulley face "
                 f"z {zc(P.Z_CHANNEL[0]) + 0.5:.1f} (hub only) / flange z {zc(P.Z_CHANNEL[0]) + 1.0:.1f}: axial gap to the flange "
                 f"{zc(P.Z_CHANNEL[0]) + 1.0 - zc(P.Z_THIGH_IN[1]) - B.LIP_H:.1f} mm; lip inner radius at the knee "
                 f"{P.THIGH_CH_R_KNEE - B.LIP_GAP - B.LIP_W:.2f} vs flange radius {P.OD_KNEE / 2 + 1.5:.2f} mm")
    txt = "\n".join(lines)
    with open(os.path.join(OUT, "clearance.md"), "w", encoding="utf-8") as f:
        f.write(txt)
    print(txt)
    return knee_ok and not grid_fail and ok_ab


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
