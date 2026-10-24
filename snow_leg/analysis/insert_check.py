"""Heat-set insert joints of the snow leg: worst pull-out and shear per joint vs design capacity.

Method: each joint is a planar screw group with axis n. The load at the joint (force F and
moment M about the group centroid) is found from the load cases below; per-screw pull-out =
F_n/k + (M about the in-plane axes) by the elastic screw-group rule (sum of r^2), shear = in-plane
force / k + torsion. Where all screws lie on one line, the moment about that line is taken by
prying about the nearest foot edge (conservative). Capacity: params.INSERTS (M3 800 N = brief 6.3
design value; M5 2000 N ASSUMED). Required SF >= 3.
Load cases: robot 4.5 kg; foot loads walk / trot / trot peak (x1.5); fall = 3 x robot weight on one
foot + 30 % lateral; actuator peaks (hip 4.1 Nm, ab/ad 4.1 Nm, knee joint 2.29 Nm = belt 120 N limit).
Writes analysis/out/inserts.md (also the joint table for docs/INSERTS.md).
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "cad"))
sys.path.insert(0, HERE)
import params as P  # noqa: E402
import snowgeo as SG  # noqa: E402
import build_parts as B  # noqa: E402
from leg_model import ik, joint_torques  # noqa: E402

G = P.G_ACC
W = P.ROBOT_MASS_SNOW * G
F_FALL = P.FALL_G * W
F_LAT = 0.3 * F_FALL
TAU_HIP, TAU_ABAD = P.TAU_PEAK, P.TAU_PEAK
TAU_KNEE_JOINT = P.BELT_T_PEAK * P.PD_KNEE / 2000        # Nm at the knee joint (belt-limited)
L1, L2, l = P.L1 / 1000, P.L2 / 1000, P.L_ABAD / 1000
H_SNOW = P.SNOW_STANCE_FRAC * (L1 + L2)


def group(screws, F, M, axis_plane, edge_lever=None):
    """screws: list of (u, v) in the joint plane (mm, about any origin). F = (Fu, Fv, Fn) N,
    M = (Mu, Mv, Mn) Nm about the group centroid (Mu about the u axis etc.). Returns max pull, max shear (N)."""
    pts = np.array(screws, float)
    c = pts.mean(0)
    d = (pts - c) / 1000.0                                  # m
    k = len(pts)
    Fu, Fv, Fn = F
    Mu, Mv, Mn = M
    su, sv = (d[:, 0] ** 2).sum(), (d[:, 1] ** 2).sum()
    pull = np.full(k, Fn / k)
    # moment about u-axis (tilts about u) -> varies with v; about v-axis -> varies with u
    if sv > 1e-12:
        pull += Mu * d[:, 1] / sv
    elif abs(Mu) > 0:
        pull += abs(Mu) / (edge_lever / 1000) / k
    if su > 1e-12:
        pull -= Mv * d[:, 0] / su
    elif abs(Mv) > 0:
        pull += abs(Mv) / (edge_lever / 1000) / k
    r2 = (d ** 2).sum(1).sum()
    sh = np.hypot(Fu / k - Mn * d[:, 1] / max(r2, 1e-12), Fv / k + Mn * d[:, 0] / max(r2, 1e-12))
    return float(np.max(pull)), float(np.max(sh))


def module_reactions(F, p_foot):
    """Leg force F (N, body frame, on the leg from the ground) at p_foot (m). Module supported by the ab/ad
    output at the back flange (x_b) and the stub-axle bearing at the front wall (x_f), both on the X axis.
    Returns (R_back(3), R_front(3), T_x) — forces the supports exert on the module, ab/ad torque."""
    xb, xf = -(B.FX0 + B.FX1) / 2000, (B.BB_FRONT[0] + B.BB_FRONT[1]) / 2000
    M0 = np.cross(p_foot, F)                                     # about the origin (on the X axis)
    T_x = M0[0]
    # moment about Y and Z resolved into forces at xb, xf; x-force taken at the back
    # sum Fy: Rb_y + Rf_y = -F_y ; moment about z at origin: xb*Rb_y + xf*Rf_y = -M0[2]
    A = np.array([[1, 1], [xb, xf]])
    Rb_y, Rf_y = np.linalg.solve(A, [-F[1], -M0[2]])
    # sum Fz: Rb_z + Rf_z = -F_z ; moment about y: -(xb*Rb_z + xf*Rf_z) = -M0[1]
    Rb_z, Rf_z = np.linalg.solve(A, [-F[2], M0[1]])
    return np.array([-F[0], Rb_y, Rb_z]), np.array([0.0, Rf_y, Rf_z]), T_x


def cases():
    """(name, F on the leg from the ground (N), foot point (m), ab/ad torque cap)"""
    out = []
    for name, Fv, lat in (("trot peak", 1.5 * W / 2, 0.3), ("fall (3 W + 30 % lateral)", F_FALL, 0.3)):
        for x in (-0.06, 0.06):
            for s in (-1, 1):
                out.append((f"{name}, foot x {x*1000:+.0f}, lateral {'+' if s > 0 else '-'}",
                            np.array([0.0, Fv, s * lat * Fv]), np.array([x, -H_SNOW - SG.ANKLE_H / 1000, l])))
    return out


def worst(fn):
    best = None
    for name, F, p in cases():
        r = fn(F, p)
        if best is None or max(r[0] / r[2], r[1] / r[2]) > max(best[1][0] / best[1][2], best[1][1] / best[1][2]):
            best = (name, r)
    return best


def main():
    rows = []

    def add(joint, size, n, case, pull, shear, note):
        cap = P.INSERTS[size]["cap"]
        sf = cap / max(pull, shear, 1e-9)
        rows.append((joint, size, n, case, pull, shear, cap, sf, note))

    # 1 thigh cover -> outer (4 x M3, axis z, in the thigh plane)
    belt = 2 * 60.0 + P.BELT_T_PEAK                         # hip shaft: both spans at peak, preload 60 N
    knee_share = 0.5 * math.hypot(F_FALL, F_LAT) + 0.5 * belt
    shear = (belt + knee_share) / 4
    pull = (0.25 * F_LAT + 4 * 1.0) / 4                     # lateral share + gasket/lip preload
    add("thigh cover → thigh outer", "M3", 4, "fall + belt peak", pull, shear,
        "cover carries the hip-shaft bearing (belt pull) and half the knee bearing load; axial knee loads go to the outer wall")

    # 2 flange feet -> inboard plate (2 x M3 per flange, axis z; screws at x = 29.3, y = +/-17; joint plane z = -13)
    def flange(back):
        def f(F, p):
            Rb, Rf, Tx = module_reactions(F, p)
            Tx = float(np.clip(Tx, -TAU_ABAD, TAU_ABAD)) if back else 0.0
            R = Rb if back else Rf
            r = np.array([(-B.FX0 if back else B.FX0) / 1000 - (-29.3 if back else 29.3) / 1000, 0.0, 0.0 - B.ZP_PAD / 1000])
            M = np.cross(r, R) + np.array([Tx, 0, 0])
            pull, sh = group([(29.3, -17.0), (29.3, 17.0)], (R[0], R[1], R[2]), (M[0], M[1], M[2]), "xy", edge_lever=4.25)
            return pull, sh, P.INSERTS["M3"]["cap"]
        return f
    for back in (True, False):
        name, r = worst(flange(back))
        add(f"module flange {'back' if back else 'front'} → inboard plate", "M3", 2, name, r[0], r[1],
            "ab/ad output (back) / stub-axle bearing (front) reaction; ab/ad torque capped at the 4.1 Nm motor peak")

    # 3 back wall foot -> spine (2 x M5 at x -62, y +/-9.5; joint plane z = -40), carries the ab/ad stator reaction
    def wall_back(F, p):
        Rb, Rf, Tx = module_reactions(F, p)
        Tx = float(np.clip(Tx, -TAU_ABAD, TAU_ABAD))
        R = -Rb                                                # module pushes on the motor -> wall
        r = np.array([(B.ABAD_X0 + B.ABAD_X1) / 2000 - (-0.062), 0.0, 0.0 - B.BLK[2] / 1000])
        M = np.cross(r, R) + np.array([-Tx, 0, 0])
        pull, sh = group([(-62.0, -9.5), (-62.0, 9.5)], (R[0], R[1], R[2]), (M[0], M[1], M[2]), "xy", edge_lever=5.5)
        return pull, sh, P.INSERTS["M5"]["cap"]
    name, r = worst(wall_back)
    add("body back wall foot → spine", "M5", 2, name, r[0], r[1], "ab/ad motor stator reaction (torque + back support force), 40 mm lever to the spine")
    cap3 = P.INSERTS["M3"]["cap"] / max(r[0], r[1])
    rows[-1] = rows[-1][:-1] + (rows[-1][-1] + f"; as M3 this joint would have SF {cap3:.1f} -> M5 chosen",)

    # 4 front wall foot -> spine (4 x M3 at x 20 / 29.5, y +/-17.3; joint plane z = -46)
    def wall_front(F, p):
        Rb, Rf, Tx = module_reactions(F, p)
        R = -Rf
        r = np.array([(B.BB_FRONT[0] + B.BB_FRONT[1]) / 2000 - 0.02475, 0.0, 0.0 - B.SP1 / 1000])
        M = np.cross(r, R)
        pull, sh = group([(20.0, -17.3), (20.0, 17.3), (29.5, -17.3), (29.5, 17.3)], (R[0], R[1], R[2]), (M[0], M[1], M[2]), "xy")
        return pull, sh, P.INSERTS["M3"]["cap"]
    name, r = worst(wall_front)
    add("body front wall foot → spine", "M3", 4, name, r[0], r[1], "stub-axle bearing reaction, 46 mm lever to the spine")

    # 5 rig plate -> spine: 4 x M5 in the widened back block; whole leg reaction about the group centroid
    def rig(F, p):
        pts = B.RIG_SCREWS
        c = np.mean(np.array(pts), 0) / 1000
        M = np.cross(p - np.array([c[0], c[1], B.SP0 / 1000]), F)
        d = (np.array(pts) / 1000 - c)
        su, sv = (d[:, 0] ** 2).sum(), (d[:, 1] ** 2).sum()
        pull = np.abs(F[2] / len(pts) + M[0] * d[:, 1] / sv - M[1] * d[:, 0] / su)
        return float(pull.max()), float(np.hypot(F[0], F[1]) / len(pts)), P.INSERTS["M5"]["cap"]
    name, r = worst(rig)
    add("rig plate → spine (body mount)", "M5", 4, name, r[0], r[1],
        "whole leg reaction; 4 x M5 at x -48/-34, y ±19 (block widened to ±30); as M3 the same group would have SF "
        f"{P.INSERTS['M3']['cap'] / r[0]:.1f}")

    # 6 foot <-> ankle bracket (4 x M3, axis y, at x +/-10, z +/-16.5 about the pivot): heel/toe fall with a stop engaged
    lever_heel, lever_toe = -SG.X_HEEL / 1000, SG.X_TOE / 1000
    Mz = F_FALL * max(lever_heel, lever_toe)
    pull, sh = group([(-15.0, -16.5), (-15.0, 16.5), (15.0, -16.5), (15.0, 16.5)], (0.0, F_LAT, F_FALL),
                     (F_LAT * 0.007, Mz, 0.0), "xz")
    add("ankle bracket flanges → foot deck", "M3", 4, "fall onto the heel tip, toe-down stop engaged", pull, sh,
        f"moment {Mz:.1f} Nm about the pivot through the stop; screw couple across x ±15 mm")

    # 7 ankle stop cross-bolts: insert in the outer cheek takes half the stop force in shear
    F_stop = Mz / (P.STOP_R / 1000)
    add("ankle stop cross-bolt → outer cheek", "M3", 1, "same heel fall", 0.0, F_stop / 2,
        f"stop force {F_stop:.0f} N shared by both cheeks (bolt in double shear)")

    # 8 ice spikes, 9 spring posts
    add("ice spikes (set screws) → foot", "M3", P.SPIKE_N, "lateral fall on ice, 2 spikes engaged", 0.0, F_LAT / 2, "set screws pushed inward by the ground; shear only")
    add("spring posts (deck ×2, shank ×1)", "M3", 3, "spring at the stop", P.SPRING_PRELOAD + P.SPRING_K * 20.0, 0.0, "0.1 N/mm springs, 20 mm stretch")

    # 10 knee stop posts (M5 inserts through the shank): fall onto a leg resting on a stop
    def knee_post(q2_deg, r_mm):
        q2 = math.radians(q2_deg)
        # foot under the hip with the knee at the stop
        h = math.sqrt(max(L1 ** 2 + L2 ** 2 + 2 * L1 * L2 * math.cos(q2), 1e-6))
        q = ik(np.array([0.0, -h, l]), L1, L2, l)
        tq = joint_torques(q, np.array([0.0, -F_FALL, 0.0]), L1, L2, l)
        return abs(tq[2]) / (r_mm / 1000)
    add("knee stop post (upper, 139°) → shank", "M5", 1, "fall onto a crouched leg at the stop",
        0.0, knee_post(P.Q2_LIM[1] + 2.0, B.KNEE_UP[0]), "post 48.5 mm from the knee; foot under the hip")
    add("knee stop post (lower, 13°) → shank", "M5", 1, "fall onto a straight leg at the stop",
        0.0, knee_post(P.Q2_LIM[0] - 2.0, B.KNEE_LO[0]), "post on the horn, 34 mm from the knee")
    add("knee stop posts at the belt limit", "M5", 1, "knee driven into a stop at 2.29 Nm", 0.0,
        TAU_KNEE_JOINT / (B.KNEE_LO[0] / 1000), "control-failure case")

    # 11 foot_ball (FOOT = 'ball' only)
    add("ball foot lap → shank (FOOT = 'ball')", "M3", 2, "fall", 0.0, math.hypot(F_FALL, F_LAT) / 2, "lap joint in shear")

    L = ["# Heat-set insert joints (analysis/insert_check.py)\n",
         f"Robot {P.ROBOT_MASS_SNOW} kg; fall = {P.FALL_G:.0f} × weight = {F_FALL:.0f} N on one foot + {F_LAT:.0f} N lateral; "
         f"capacity M3 {P.INSERTS['M3']['cap']:.0f} N (brief 6.3 design value), M5 {P.INSERTS['M5']['cap']:.0f} N (ASSUMED: M3 value × "
         "insert surface-area ratio — confirm on the coupon). Required SF ≥ 3.\n",
         "| joint | size | n | worst case | pull-out N | shear N | capacity N | SF | ok | note |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    all_ok = True
    for j, sz, n, case, pull, sh, cap, sf, note in rows:
        ok = sf >= P.INSERT_SF
        all_ok &= ok
        L.append(f"| {j} | {sz} | {n} | {case} | {pull:.0f} | {sh:.0f} | {cap:.0f} | {sf:.1f} | {'yes' if ok else '**NO**'} | {note} |")
    # geometry rules
    L.append("\n## Geometry rules (brief 6.4)\n")
    L.append(f"- hole Ø as modelled: M3 {P.INSERTS['M3']['hole']} mm, M5 {P.INSERTS['M5']['hole']} mm (datasheet + 0.1 mm print compensation; coupon "
             "holes at ±0.1 to calibrate)")
    L.append(f"- depth ≥ insert length + 2 pitches: M3 {P.insert_depth('M3'):.1f} mm, M5 {P.insert_depth('M5'):.1f} mm "
             "(the two M5 knee-stop inserts sit in through-holes of the 10 mm shank)")
    L.append(f"- boss radius = hole radius + one insert OD: M3 r {B.boss_r('M3'):.2f} (wall {B.boss_r('M3') - P.INSERTS['M3']['hole']/2:.2f} mm ≥ 3), "
             f"M5 r {B.boss_r('M5'):.2f} (wall {B.boss_r('M5') - P.INSERTS['M5']['hole']/2:.2f} mm)")
    L.append("- every insert hole axis is parallel to its part's print-bed normal (vertical when printed): thigh outer (z, bed +z), "
             "inboard plate (z, bed −z), spine (z, bed −z), foot (y, bed +y), outer cheek (z, bed −z), shank (z, bed −z)")
    L.append(f"\n**All joints SF ≥ {P.INSERT_SF:.0f}: {'YES' if all_ok else 'NO'}**")
    L.append("\nNot inserts (brief 6.1 exceptions): M2.5 screws into the motors' own threads (hip, knee, ab/ad flanges and back "
             "faces; the C-frame tab is clamped by the 5 knee-motor back screws), the torque-carrying hex pockets (knee M5 head "
             "and nylock, hip-shaft M5 head), the idler bolt (M3 + nut: it slides in a slot).")
    txt = "\n".join(L)
    open(os.path.join(HERE, "out", "inserts.md"), "w", encoding="utf-8").write(txt)
    print(txt)
    return all_ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
