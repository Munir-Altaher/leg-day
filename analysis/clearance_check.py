"""Swept clearance check over the joint ranges, from the exported STLs.

Moving parts are sampled as dense point clouds (STL vertices + points every 1.5 mm
along every triangle edge), moved through q0 x q1 x q2 and tested against the
static parts modelled as exact primitives (boxes / cylinders, from params and
build_parts constants). Reports the collision-free hip range per knee angle,
the knee range, and the ab/ad range. Writes analysis/out/clearance.md.
Run after cad/build_parts.py:  python analysis/clearance_check.py
"""
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
import build_parts as B  # noqa: E402

STL = os.path.join(ROOT, "cad", "stl")
OUT = os.path.join(HERE, "out")
SHIFT = 300.0
GAP = 0.5                       # required clearance, mm


def load(name, step=1.5):
    with open(os.path.join(STL, name + ".STL"), "rb") as f:
        data = f.read()
    if data[:5] == b"solid":                     # SolidWorks writes ASCII STL by default
        v = [ln.split()[1:4] for ln in data.decode("ascii", "ignore").splitlines() if ln.strip().startswith("vertex")]
        tri = np.array(v, float).reshape(-1, 3, 3)
    else:
        n = struct.unpack("<I", data[80:84])[0]
        tri = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"].reshape(-1, 3, 3).astype(float)
    if np.abs(tri).max() < 5:                 # exported in metres
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
    return np.unique(np.round(np.vstack(pts), 2), axis=0)


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


def main():
    thigh = np.vstack([load("thigh_outer"), load("thigh_cover")])
    # snowshoe at both lip stops (the part is modelled flat at shank angle 0): turn it about the ankle pivot
    shoe = load("snowshoe")
    shoes = [rz(shoe, math.radians(-a), B.ANKLE) for a in P.ANKLE_LIM]
    base = np.vstack([load("shank"), load(f"knee_pulley_{P.KNEE_TEETH}T")])
    lowers = [np.vstack([base, sh]) for sh in shoes]
    lower = np.vstack([base] + shoes)                              # both stop poses together (conservative)
    lowers_out = [lw[lw[:, 2] > B.zc(P.Z_THIGH_OUT[1])] for lw in lowers]   # shank/shoe outboard of the thigh
    zc = P.zc
    bridge = ((B.BRIDGE[0], B.BRIDGE[2], B.TAB_Z0), (B.BRIDGE[1], B.BRIDGE[3], zc(P.Z_PLATE_OUT[1])))      # C-frame column
    hip_motor = (P.MOTOR_D / 2, zc(P.Z_HIP_MOTOR[0]), zc(P.Z_HIP_MOTOR[1]))
    plate_out = (30.0, zc(P.Z_PLATE_OUT[0]), zc(P.Z_PLATE_OUT[1]))
    plate_out_box = ((-30.0, 0.0, zc(P.Z_PLATE_OUT[0])), (30.0, B.BRIDGE[3], zc(P.Z_PLATE_OUT[1])))

    def leg_hits(q1, q2):
        T = rz(thigh, q1)
        if in_box(T, *bridge):
            return True
        for lower_out in lowers_out:
            L = rz(rz(lower_out, q2, B.KNEE), q1)
            if (in_box(L, *bridge) or in_cyl_z(L, *hip_motor, g=GAP) or in_cyl_z(L, *plate_out)
                    or in_box(L, *plate_out_box)):
                return True
        return False

    lines = ["# Swept clearance check (analysis/clearance_check.py)\n",
             f"Required clearance {GAP} mm. Moving parts = STL point clouds (1.5 mm edge sampling); static parts = exact boxes/cylinders. "
             f"The snowshoe is tested at both ankle stops ({P.ANKLE_LIM[0]:.0f} and {P.ANKLE_LIM[1]:.0f} deg).\n"]
    # knee range at q1 = 0
    q2s = np.arange(-30, 181, 1.0)
    free = [q for q in q2s if not leg_hits(0.0, math.radians(q))]
    kmin, kmax = min(free), max(free)
    lines.append(f"- Knee, q1 = 0: collision-free q2 in [{kmin:.0f}, {kmax:.0f}] deg (design limits {P.Q2_LIM}) -> {'PASS' if kmin <= P.Q2_LIM[0] and kmax >= P.Q2_LIM[1] else 'FAIL'}")
    # hip range per knee angle (5 deg steps)
    worst = (999, -999)
    rows = []
    for q2 in np.arange(P.Q2_LIM[0], P.Q2_LIM[1] + 0.1, 5.0):
        ok_pos = 0.0
        for q1 in np.arange(0, 181, 1.0):
            if leg_hits(math.radians(q1), math.radians(q2)):
                break
            ok_pos = q1
        ok_neg = 0.0
        for q1 in np.arange(0, -181, -1.0):
            if leg_hits(math.radians(q1), math.radians(q2)):
                break
            ok_neg = q1
        rows.append((q2, ok_neg, ok_pos))
        worst = (max(worst[0], ok_neg), min(worst[1], ok_pos)) if worst[0] != 999 else (ok_neg, ok_pos)
    lines.append(f"- Hip, all knee angles in limits (5 deg steps): collision-free q1 in [{worst[0]:.0f}, {worst[1]:.0f}] deg "
                 f"(design limits {P.Q1_LIM}) -> {'PASS' if worst[0] <= P.Q1_LIM[0] and worst[1] >= P.Q1_LIM[1] else 'FAIL'}")
    lines.append("\n| q2 | hip free from | to |\n|---|---|---|")
    lines += [f"| {r[0]:.0f} | {r[1]:.0f} | {r[2]:.0f} |" for r in rows]

    # ab/ad: whole hip module + leg rotates about X; static body bracket + ab/ad motor
    module = np.vstack([load(n) for n in ("module_plate_inboard", "module_flange_back", "module_flange_front", "module_frame_outboard", "ref_motor_hip",
                                          "ref_motor_knee", "hip_shaft_adapter")])
    static_boxes = [
        ((B.BB_BACK[0], -20.0, B.BLK[2]), (B.BB_BACK[1], 20.0, -15.0)),              # back wall leg
        ((B.BB_BACK[1], -20.0, B.BLK[2]), (-51.0, 20.0, B.BLK[2] + 6.0)),            # back wall foot
        ((B.BB_FRONT[0], -B.SPINE_FRONT_W, B.SP1), (B.BB_FRONT[1], B.SPINE_FRONT_W, -12.0)),   # front wall leg
        ((B.FRONT_FOOT_X0, -B.SPINE_FRONT_W, B.SP1), (B.BB_FRONT[0], B.SPINE_FRONT_W, B.SP1 + 6.0)),  # front wall foot
        ((B.BB_BACK[1], -20.0, B.RIG_Z[0]), (B.BB_FRONT[0], 20.0, B.SP1)),          # spine + rig plate
        ((14.0, -B.SPINE_FRONT_W, B.RIG_Z[0]), (B.BB_FRONT[0], B.SPINE_FRONT_W, B.SP1)),       # widened spine front
        ((B.BLK[0], -20.0, B.SP1), (B.BLK[1], 20.0, B.BLK[2])),                     # back block
    ]
    # the ab/ad motor's output face is bolted to the module flange -> test its body only up to 1 mm short of that face
    static_cyl = [(P.BB_BACK_R, *B.BB_BACK), (P.BB_FRONT_R, *B.BB_FRONT), (P.MOTOR_D / 2, B.ABAD_X0, B.ABAD_X1 - 1.0)]
    lines.append("\n## Ab/ad sweep (module + leg about X), hip and knee at several poses\n")
    res = []
    for q1d, q2d in ((0, 15), (0, 90), (-60, 120), (60, 30), (-100, 150), (100, 15), (-100, 15), (100, 150)):
        mov = np.vstack([module, rz(thigh, math.radians(q1d)), rz(rz(lower, math.radians(q2d), B.KNEE), math.radians(q1d))])
        lim = [None, None]
        for sgn, idx in ((1, 1), (-1, 0)):
            last = 0.0
            for q0 in np.arange(0, 121, 1.0):
                pts = rx(mov, math.radians(sgn * q0))
                hit = any(in_box(pts, lo, hi) for lo, hi in static_boxes) or \
                    any(in_cyl_x(pts, r, a, b) for r, a, b in static_cyl)
                if hit:
                    break
                last = sgn * q0
            lim[idx] = last
        res.append((q1d, q2d, lim))
        lines.append(f"- q1 = {q1d:4d}, q2 = {q2d:3d}: ab/ad collision-free [{lim[0]:.0f}, {lim[1]:.0f}] deg")
    lo = max(r[2][0] for r in res)
    hi = min(r[2][1] for r in res)
    lines.append(f"\nAb/ad collision-free over all tested poses: [{lo:.0f}, {hi:.0f}] deg (design limits {P.Q0_LIM}) -> "
                 f"{'PASS' if lo <= P.Q0_LIM[0] and hi >= P.Q0_LIM[1] else 'FAIL'}")
    # belt / channel radial clearance
    belt_t = 1.38
    c_knee = P.THIGH_CH_R_KNEE - (P.PD_KNEE / 2 + belt_t - 0.254)
    c_hip = P.THIGH_CH_R_HIP - (P.PD_HIP / 2 + belt_t - 0.254)
    fl = P.THIGH_CH_R_KNEE - (P.OD_KNEE / 2 + 1.5)
    lines.append(f"\n## Belt\n- belt back to channel wall: knee {c_knee:.2f} mm, hip {c_hip:.2f} mm (>= 1 mm required); knee-pulley flange to wall {fl:.2f} mm")
    lines.append(f"- belt width {P.BELT_W} mm in a {P.THIGH_CHANNEL} mm channel: {(P.THIGH_CHANNEL - P.BELT_W)/2:.1f} mm per side")
    txt = "\n".join(lines)
    with open(os.path.join(OUT, "clearance.md"), "w", encoding="utf-8") as f:
        f.write(txt)
    print(txt)


if __name__ == "__main__":
    main()
