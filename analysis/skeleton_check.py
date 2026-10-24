"""Link strength / stiffness check, solid vs skeleton (brief section 5.1).

Beam model: each link is a cantilever loaded at its end; the critical section is
the most-holed one. Thigh = box section (two 5 mm walls + rim, channel inside);
shank = single plate (8 mm) with flanges either side of a truss web.
Load cases: stance (continuous 15 N + 10 % lateral), push-off (30 % fore-aft),
lateral (30 % sideways, out-of-plane), and actuator peak torque (control failure).
Writes analysis/out/skeleton.md. Masses come from cad/parts/parts_report*.json.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))
import params as P  # noqa: E402
import build_parts as B  # noqa: E402
from leg_model import ik, joint_torques, motor_torques  # noqa: E402

OUT = os.path.join(HERE, "out")
INFILL_FACTOR = 0.75        # 4 perimeters + 40 % gyroid on thin parts ~ 75 % of solid mass (ESTIMATE)
KT = 2.0


def rect_I(b, h):
    return b * h ** 3 / 12


def main():
    L1, L2, l = P.L1, P.L2, P.L_ABAD
    H = P.STANCE_FRAC * (L1 + L2)
    q = ik(np.array([0, -H, l]) / 1000, L1 / 1000, L2 / 1000, l / 1000)
    lines = ["# Skeleton links: strength, stiffness, mass (analysis/skeleton_check.py)\n"]

    # ---------------- loads at the foot (N)
    Fv = 15.0
    cases = {"stance": np.array([0, -Fv, 0.1 * Fv]), "push-off": np.array([0.3 * Fv, -Fv, 0]),
             "lateral": np.array([0, -Fv, 0.3 * Fv])}

    # ---------------- shank section (plate 8 thick), in-plane bending about z, out-of-plane about x
    t = P.SHANK_T
    # most holed section: ~25 mm below the knee, width ~ 2*hw(y)
    y_sec = 25.0
    w_full = P.SHANK_W_KNEE - (P.SHANK_W_KNEE - P.SHANK_W_FOOT) * y_sec / (L2 - 35)
    fl = P.SK_FLANGE
    I_in_solid = rect_I(t, w_full)
    # skeleton: two flanges + one strut crossing at ~45 deg (ignored -> conservative)
    I_in_sk = 2 * (rect_I(t, fl) + t * fl * ((w_full - fl) / 2) ** 2)
    I_out_solid = rect_I(w_full, t)
    I_out_sk = 2 * rect_I(fl, t)
    arm = L2 - y_sec              # foot to section
    # bent shank: the centre line bows off the knee-foot line, so the axial foot force adds a moment F_ax * e
    e_bow = max(abs(B.shank_frame(B.SH_LEN * i / 100)[0][0] - B.KNEE[0]) for i in range(101))
    S = P.PLA_STRENGTH / P.SF
    lines.append(f"## Shank (critical section {y_sec:.0f} mm below knee, width {w_full:.1f} mm, flanges {fl} mm; "
                 f"the bend bows {e_bow:.1f} mm off the knee-foot line, axial load x bow is included)\n")
    lines.append("| case | F_inplane N | F_out N | sigma solid MPa | sigma skeleton (Kt=2) MPa | allow MPa | ok |\n|---|---|---|---|---|---|---|")
    # shank angle at stance: q1+q2 from vertical
    phi = q[1] + q[2]
    for name, F in cases.items():
        Fx, Fy, Fz = F
        f_in = abs(Fx * math.cos(phi) + Fy * math.sin(phi))     # perpendicular to shank, in the leg plane
        f_out = abs(Fz)
        f_ax = abs(Fx * math.sin(phi) - Fy * math.cos(phi))     # along the knee-foot line
        M_in, M_out = f_in * arm + f_ax * e_bow, f_out * arm
        s_solid = M_in * (w_full / 2) / I_in_solid + M_out * (t / 2) / I_out_solid
        s_sk = KT * (M_in * (w_full / 2) / I_in_sk + M_out * (t / 2) / I_out_sk)
        lines.append(f"| {name} | {f_in:.1f} | {f_out:.1f} | {s_solid:.2f} | {s_sk:.2f} | {S:.1f} | {'yes' if s_sk < S else 'NO'} |")
    # peak knee torque (belt-limited 120 N x r_knee) as a pure moment at the knee section
    Mk = P.BELT_T_PEAK * P.PD_KNEE / 2
    s_pk = KT * Mk * (w_full / 2) / I_in_sk
    lines.append(f"| knee peak (belt 120 N -> {Mk/1000:.2f} Nm) | - | - | {Mk*(w_full/2)/I_in_solid:.2f} | {s_pk:.2f} | {S:.1f} | {'yes' if s_pk < S else 'NO'} |")

    # ---------------- thigh: box section (walls 5 + 5, depth ~ outline width, rim 4-5.5)
    # critical section: the bottom of the lowest wall cutout, where the cutout is widest
    s_cut = B.THIGH_CUT_S[1]
    wd = 2 * B.outer_hw(-s_cut)
    tw = P.THIGH_WALL
    b_tot = 2 * tw + P.THIGH_CHANNEL
    rim = 5.0
    I_solid = rect_I(b_tot, wd) - rect_I(P.THIGH_CHANNEL, wd - 2 * rim)
    hw_holes = 2 * B.thigh_cut_hw(s_cut)       # cutout width through both walls at that section
    I_sk = I_solid - 2 * rect_I(tw, hw_holes)
    lines.append(f"\n## Thigh (box section at the widest cutout, {s_cut:.0f} mm below the hip: depth {wd:.1f} mm, "
                 f"walls {tw} mm with a {hw_holes:.1f} mm cutout, rim {rim} mm)\n")
    peak_hip = P.TAU_PEAK * 1000            # Nmm, hip motor peak into the thigh root
    s_th_peak = KT * peak_hip * (wd / 2) / I_sk
    tq = joint_torques(q, cases["stance"], L1 / 1000, L2 / 1000, l / 1000)
    s_th = KT * abs(tq[1] - 0) * 1000 * (wd / 2) / I_sk + KT * abs(tq[2]) * 1000 * (wd / 2) / I_sk
    lines.append(f"- stance: sigma (Kt=2) {s_th:.2f} MPa; hip-motor peak 4.1 Nm stalled against the ground: {s_th_peak:.2f} MPa; allow {S:.1f} MPa "
                 f"-> {'PASS' if max(s_th, s_th_peak) < S else 'FAIL'}")
    # hip bolt joint: 3 x M2.5 on PCD27 at 4.1 Nm -> shear + PLA bearing
    Fb = P.TAU_PEAK / (3 * 0.0135)
    bear = Fb / (2.5 * P.THIGH_WALL)
    lines.append(f"- hip-motor flange, 3 x M2.5 on Ø27 at 4.1 Nm: {Fb:.0f} N per screw, PLA bearing {bear:.1f} MPa (allow {S:.1f}; "
                 f"{'PASS' if bear < S else 'FAIL -> firmware hip limit ' + str(round(P.TAU_PEAK * S / bear, 1)) + ' Nm'})")
    # same check for the knee adapter (belt-limited 0.76 Nm) and ab/ad flange (4.1 Nm)
    Fa = 0.76 / (3 * 0.0135)
    lines.append(f"- knee adapter at the 0.76 Nm firmware limit: {Fa:.0f} N per M2.5, bearing {Fa/(2.5*3.5):.1f} MPa")
    Fab = P.TAU_PEAK / (3 * 0.0135)
    lines.append(f"- ab/ad flange (5.5 mm) at 4.1 Nm: bearing {Fab/(2.5*P.FLANGE_T):.1f} MPa "
                 f"(-> ab/ad firmware limit {min(P.TAU_PEAK, P.TAU_PEAK * S / (Fab/(2.5*P.FLANGE_T))):.1f} Nm)")
    # Ø5 steel shaft torsion
    for tau in (0.76, 4.1):
        lines.append(f"- Ø5 steel hip shaft at {tau} Nm: tau = {16*tau*1000/(math.pi*5**3):.0f} MPa")

    # ---------------- deflection at the foot (continuous load), stance pose
    E = P.PLA_E
    F = Fv
    d_thigh = F * (L1 * abs(math.sin(q[1]))) * L1 ** 2 / (3 * E * I_sk)   # crude: perpendicular component
    d_shank = F * abs(math.sin(phi)) * L2 ** 3 / (3 * E * I_in_sk) * 0.6     # tapered + holes, 0.6 ~ taper gain (ESTIMATE)
    d_out = 0.1 * F * L2 ** 3 / (3 * E * I_out_sk)
    d_shank += F * abs(math.cos(phi)) * e_bow * L2 ** 2 / (2 * E * I_in_sk)    # bend: axial load x bow, as an end moment
    back = math.radians(P.BACKLASH_ARCMIN / 60) * (L1 + L2) * 1.0
    lines.append(f"\n## Foot deflection at {Fv:.0f} N stance\n- thigh {d_thigh:.3f} mm, shank in-plane {d_shank:.3f} mm, "
                 f"shank out-of-plane (10 % lateral) {d_out:.3f} mm -> total {d_thigh + d_shank + d_out:.2f} mm (target < 1 mm) "
                 f"-> {'PASS' if d_thigh + d_shank + d_out < 1 else 'FAIL'}")
    lines.append(f"- for reference, gearbox backlash alone (18 arcmin) gives ~{back:.1f} mm of free play at the foot (not elastic deflection)")

    # ---------------- snowshoe: toe cantilever at the hub edge, ankle bolt bearing, lip contact
    import snowshoe_geo as SG
    F_pk = 1.5 * 4.4 * 9.81 / 2                         # trot peak per foot (2 feet down, x1.5), whole load at the toe
    u_sec = SG.SOLID_HUB[1]                             # section at the edge of the solid hub under the cheeks
    arm = SG.U_TOE - u_sec
    o = SG.outline(0.0, 200)
    half_w = max(abs(o[i][1] + (o[i + 1][1] - o[i][1]) * (u_sec - o[i][0]) / (o[i + 1][0] - o[i][0]))
                 for i in range(len(o) - 1) if (o[i][0] - u_sec) * (o[i + 1][0] - u_sec) <= 0 and o[i + 1][0] != o[i][0])
    W = 2 * half_w
    rib_frac = P.SHOE_RIB_W / (0.5 * sum(P.SHOE_CELL))  # rib share of the width across the lattice
    parts = [(W, P.SHOE_SKIN, P.SHOE_SKIN / 2),                                                     # skin
             (rib_frac * (W - 2 * P.SHOE_RIM_W), P.SHOE_RIB_H, P.SHOE_SKIN + P.SHOE_RIB_H / 2),      # Voronoi ribs
             (2 * P.SHOE_RIM_W, P.SHOE_RIM_H - P.SHOE_SKIN, P.SHOE_SKIN + (P.SHOE_RIM_H - P.SHOE_SKIN) / 2)]   # rim sides
    kh = P.SHOE_KEEL_H - (P.SHOE_KEEL_H - P.SHOE_RIM_H) * (u_sec - P.SHOE_KEEL_HUB) / (SG.U_TOE - P.SHOE_KEEL_HUB)
    parts.append((3 * P.SHOE_KEEL_W, kh - P.SHOE_SKIN, P.SHOE_SKIN + (kh - P.SHOE_SKIN) / 2))      # 3 keels, tapered height here
    A_s = sum(b * h for b, h, _ in parts)
    yc = sum(b * h * y for b, h, y in parts) / A_s
    I_s = sum(b * h ** 3 / 12 + b * h * (y - yc) ** 2 for b, h, y in parts)
    c_max = max(yc, P.SHOE_RIM_H - yc)
    s_toe = KT * F_pk * arm * c_max / I_s
    bear = F_pk / (2 * P.SHOE_CHEEK_T * 3.0)             # M3 bolt on the two cheeks
    tq_lip = 14.2 * SG.U_TOE                             # walking foot load at the toe, N mm, held by a lip
    f_lip = tq_lip / P.ANKLE_LIP_H
    lines.append(f"\n## Snowshoe (worst case: trot peak {F_pk:.0f} N all at the toe, Kt = 2)\n"
                 f"- toe cantilever {arm:.0f} mm at the hub edge (section {W:.0f} mm wide: {P.SHOE_SKIN} skin + Voronoi ribs + rim + 3 keels {kh:.1f} mm tall, "
                 f"I = {I_s:.0f} mm^4): sigma {s_toe:.1f} MPa, allow {S:.1f} -> {'PASS' if s_toe < S else 'FAIL'}\n"
                 f"- ankle bolt bearing on the two {P.SHOE_CHEEK_T:.0f} mm cheeks: {bear:.1f} MPa -> {'PASS' if bear < S else 'FAIL'}\n"
                 f"- lip stop (walking load at the toe, held by a lip {P.ANKLE_LIP_H:.0f} mm above the pivot): {f_lip:.0f} N "
                 f"on a 6 x 10.4 mm face = {f_lip / 62:.1f} MPa -> {'PASS' if f_lip / 62 < S else 'FAIL'}")

    # ---------------- masses
    rep = json.load(open(os.path.join(ROOT, "cad", "parts", "parts_report.json")))
    try:
        sol = json.load(open(os.path.join(ROOT, "cad", "parts", "parts_report_solid.json")))
    except FileNotFoundError:
        sol = {}
    g = lambda r, n: r.get(n, {}).get("solid_pla_g", float("nan")) * INFILL_FACTOR
    lines.append(f"\n## Mass (printed estimate = solid volume x 1.24 g/cm^3 x {INFILL_FACTOR})\n")
    lines.append("| part | solid g | skeleton g | saving |\n|---|---|---|---|")
    tot_s = tot_k = 0
    for n in ("thigh_outer", "thigh_cover", "shank"):
        a, b = g(sol, n), g(rep, n)
        tot_s += a; tot_k += b
        lines.append(f"| {n} | {a:.1f} | {b:.1f} | {100*(1-b/a):.0f} % |")
    lines.append(f"| **thigh + shank** | {tot_s:.1f} | {tot_k:.1f} | **{100*(1-tot_k/tot_s):.0f} %** |")
    # inertia about hip (point-mass approximation, centroids at L1/2 and L1 + 0.45 L2)
    def inertia(m_th, m_sh):
        return (m_th * (L1 / 2) ** 2 + m_sh * (L1 + 0.45 * L2) ** 2) * 1e-6 / 1000   # kg m^2
    rs = (L1 + 0.45 * L2) ** 2 / (L1 / 2) ** 2
    Is = inertia(g(sol, "thigh_outer") + g(sol, "thigh_cover"), g(sol, "shank"))
    Ik = inertia(g(rep, "thigh_outer") + g(rep, "thigh_cover"), g(rep, "shank"))
    lines.append(f"\nLink inertia about the hip (leg straight, links only): solid {Is*1e4:.2f} kg cm^2, skeleton {Ik*1e4:.2f} kg cm^2 "
                 f"({100*(1-Ik/Is):.0f} % less). A gram off the shank is worth {rs:.0f}x a gram off the thigh.")
    txt = "\n".join(lines)
    with open(os.path.join(OUT, "skeleton.md"), "w", encoding="utf-8") as f:
        f.write(txt)
    print(txt)


if __name__ == "__main__":
    main()
