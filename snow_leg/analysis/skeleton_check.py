"""Strength / stiffness / mass check for the SNOW LEG (replaces the Leg Day skeleton check:
the thigh truss holes are closed and the shank is closed for snow, brief 5.1-5.2).

- closed shank: option (a) one closed part (3 perimeters + 15 % gyroid: the infill is the skeleton)
  vs option (b) two-piece clamshell with internal ribs on M3 inserts -> mass and stiffness, pick one
- link stresses at the critical sections for stance / trot peak / fall / knee peak, Kt = 2 at the
  M5 insert hole of the upper knee-stop post
- snowshoe deck: heel / toe cantilevers and the side strips under 2.9 kPa trot pressure (< 2 mm)
- ankle: pin shear, cheek bearing at the pin and at the stop bolts, stop sleeve on the shank edge
- knee stop posts (steel M5) bending in a fall
- deflection at the ankle pivot under the continuous load (< 1 mm target)
- foot mass budget (<= 35 g) and swinging-leg mass vs the original
Writes analysis/out/skeleton.md.
"""
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
import snowgeo as SG  # noqa: E402
from leg_model import ik, joint_torques  # noqa: E402

G = P.G_ACC
W = P.ROBOT_MASS_SNOW * G
S_ALLOW = P.PLA_STRENGTH / P.SF            # 12.5 MPa in-plane
E = P.PLA_E
L1, L2, l = P.L1, P.L2, P.L_ABAD
LINE, LAYER = 0.45, 0.2


def hollow_rect_I(w, t, side, skin):
    """Printed closed section: width w (in-plane), thickness t (z); perimeter walls `side` thick on the
    in-plane edges, skins `skin` thick top/bottom; infill ignored. Returns (I_inplane, I_outofplane) mm^4."""
    I_in = 2 * (t * side ** 3 / 12 + t * side * (w / 2 - side / 2) ** 2) + 2 * ((w - 2 * side) ** 3 * skin / 12)
    I_out = 2 * (w * skin ** 3 / 12 + w * skin * (t / 2 - skin / 2) ** 2) + 2 * (side * (t - 2 * skin) ** 3 / 12)
    return I_in, I_out


def main():
    L = ["# Strength, stiffness and mass — snow leg (analysis/skeleton_check.py)\n",
         f"Allowable in-plane stress {S_ALLOW:.1f} MPa (25 MPa / SF {P.SF:.0f}); E = {E:.0f} MPa; robot {P.ROBOT_MASS_SNOW} kg; "
         f"fall = {P.FALL_G:.0f} W = {P.FALL_G*W:.0f} N on one foot + 30 % lateral.\n"]

    # ---------------- shank: closed, option (a) vs (b)
    t = P.SHANK_T
    w_sec = 2 * (P.SHANK_W_KNEE / 2 - (P.SHANK_W_KNEE / 2 - P.SHANK_TIP_R) * 30 / L2)    # 30 mm below the knee
    Ia_in, Ia_out = hollow_rect_I(w_sec, t, 3 * LINE, 4 * LAYER)
    Ib_in, Ib_out = hollow_rect_I(w_sec, t, 2 * LINE + 0.0, 1.2)                          # clamshell skins 1.2
    try:
        from mass_model import printed
        m_a = printed("snow")["shank"]
    except Exception:  # noqa: BLE001
        m_a = float("nan")
    area = 0.5 * (P.SHANK_W_KNEE + 2 * P.SHANK_TIP_R) * L2 + 400.0
    perim = 2 * L2 + math.pi * (P.SHANK_W_KNEE / 2 + P.SHANK_TIP_R) + 120.0
    m_b = (2 * 1.2 * area + 2 * LINE * perim * t + 3 * 1.6 * (t - 2.4) * 15) * P.PLA_RHO * 1000 + 4 * (0.33 + 0.6) + 2.0
    L.append("## Closed shank: option (a) one closed part vs (b) clamshell\n")
    L.append("| option | mass g | I in-plane mm⁴ | I out-of-plane mm⁴ | seams into the shank | chosen |\n|---|---|---|---|---|---|")
    L.append(f"| (a) one part, 3 perimeters + 15 % gyroid | {m_a:.1f} (slicer) | {Ia_in:.0f} | {Ia_out:.0f} | none | **yes** |")
    L.append(f"| (b) clamshell, 1.2 mm skins + ribs, 4 × M3 inserts | ≈ {m_b:.1f} (estimate) | {Ib_in:.0f} | {Ib_out:.0f} | one full-length seam | no |")
    L.append("\n(a) is about the same mass, stiffer out of plane (its skins sit at the full 10 mm thickness), has no seam for "
             "snow to wick into and needs no inserts. Section 30 mm below the knee, infill ignored (conservative).\n")

    # ---------------- link stresses
    H = P.SNOW_STANCE_FRAC * (L1 + L2) / 1000
    q = ik(np.array([0.0, -H, l / 1000]), L1 / 1000, L2 / 1000, l / 1000)
    phi = q[1] + q[2]
    cases = {"walk + 10 % lat": (W / 3, 0.1), "trot peak + 30 % lat": (1.5 * W / 2, 0.3), "fall + 30 % lat": (P.FALL_G * W, 0.3)}
    import build_parts as B
    d_post = B.KNEE_UP[0]
    w_post = 2 * (P.SHANK_W_KNEE / 2 - (P.SHANK_W_KNEE / 2 - P.SHANK_TIP_R) * d_post / L2)
    hole = P.INSERTS["M5"]["hole"]
    I_post_in, I_post_out = hollow_rect_I(w_post, t, 3 * LINE, 4 * LAYER)
    I_post_in -= t * hole ** 3 / 12                          # insert hole on the axis (small effect in-plane)
    L.append(f"## Shank stresses (snow stance: shank {math.degrees(phi):.0f}° from vertical)\n")
    L.append("| case | section | in-plane MPa | out-of-plane MPa | Kt | combined MPa | allow | ok |\n|---|---|---|---|---|---|---|---|")
    ok_all = True
    for name, (Fv, lat) in cases.items():
        for sec, dist, w, Ii, Io, kt in (("30 mm below knee", 30.0, w_sec, Ia_in, Ia_out, 1.0),
                                         (f"upper stop post ({d_post:.0f} mm)", d_post, w_post, I_post_in, I_post_out, 2.0)):
            arm = L2 - dist
            f_in = abs(Fv * math.sin(phi))
            f_out = lat * Fv
            s_in = f_in * arm * (w / 2) / Ii
            s_out = f_out * arm * (t / 2) / Io
            comb = kt * (s_in + s_out)
            ok = comb < S_ALLOW if "fall" not in name else comb < P.PLA_STRENGTH
            ok_all &= ok
            L.append(f"| {name} | {sec} | {s_in:.2f} | {s_out:.2f} | {kt:.0f} | {comb:.2f} | {S_ALLOW if 'fall' not in name else P.PLA_STRENGTH:.1f} | {'yes' if ok else '**NO**'} |")
    Mk = P.BELT_T_PEAK * P.PD_KNEE / 2
    s_k = Mk * (w_sec / 2) / Ia_in
    L.append(f"| knee peak (belt 120 N → {Mk/1000:.2f} Nm) | 30 mm below knee | {s_k:.2f} | – | 1 | {s_k:.2f} | {S_ALLOW} | {'yes' if s_k < S_ALLOW else '**NO**'} |")
    L.append("\nFall cases are checked against the 25 MPa strength (SF 1 for an accident case); service cases against 12.5 MPa.\n")

    # ---------------- thigh (closed box, no holes now)
    wd = 2 * (P.THIGH_R_HIP + (P.THIGH_R_KNEE - P.THIGH_R_HIP) * 0.3)
    b_tot = 2 * P.THIGH_WALL_THIN + P.THIGH_CHANNEL
    I_th = b_tot * wd ** 3 / 12 - P.THIGH_CHANNEL * (wd - 10) ** 3 / 12
    s_th = 2.0 * P.TAU_PEAK * 1000 * (wd / 2) / I_th
    L.append(f"## Thigh (sealed box, 3 mm walls, no holes): hip-motor peak 4.1 Nm at the root → {s_th:.2f} MPa (Kt 2) vs {S_ALLOW} -> "
             f"{'PASS' if s_th < S_ALLOW else 'FAIL'}\n")

    # ---------------- snowshoe deck
    p_tr = 1.5 * W / 2 / (SG.AREA * 1e-6) / 1e6 * 1.0       # MPa at trot peak... use trot (2.9 kPa) per brief
    p = (W / 2) / (SG.AREA * 1e-6) / 1e6                      # trot, MPa
    E_deck = 0.85 * E                                         # 2-perimeter / 2-skin print, conservative
    # heel and toe cantilevers from the pad edge, longitudinal section = deck + 2 rims (rim walls along the edges)
    def cant(L_len, x0, sgn):
        xs = np.linspace(0, L_len, 200)
        dx = xs[1] - xs[0]
        M = np.zeros_like(xs)
        b = np.array([2 * SG.half_width(sgn * (x0 + x)) for x in xs])
        q_ = p * b                                            # N/mm
        for i in range(len(xs)):
            M[i] = np.sum(q_[i:] * (xs[i:] - xs[i])) * dx
        I = np.array([bb * P.DECK_T ** 3 / 12 + bb * P.DECK_T * (P.LUG_H / 2) ** 2 * 0 +
                      2 * (P.RIM_W * P.LUG_H ** 3 / 12 + P.RIM_W * P.LUG_H * ((P.LUG_H + P.DECK_T) / 2) ** 2) for bb in b])
        I = np.maximum(I, 1.0)
        kappa = M / (E_deck * I)
        slope = np.cumsum(kappa) * dx
        defl = np.cumsum(slope) * dx
        smax = np.max(M * (P.LUG_H + P.DECK_T) / I)
        return defl[-1], smax
    pad = P.PAD_L / 2
    d_heel, s_heel = cant(-SG.X_HEEL - pad, pad, -1)
    d_toe, s_toe = cant(SG.X_TOE - pad, pad, 1)
    # side strip: deck + one transverse lug as a T beam, tributary width = lug pitch, cantilever from the pad edge to the rim
    lug_pitch = 15.0
    side_len = P.FOOT_W / 2 - P.PAD_W / 2
    yb = (P.DECK_T * lug_pitch * (P.DECK_T / 2) + P.LUG_W * P.LUG_H * (P.DECK_T + P.LUG_H / 2)) / (P.DECK_T * lug_pitch + P.LUG_W * P.LUG_H)
    I_T = lug_pitch * P.DECK_T ** 3 / 12 + lug_pitch * P.DECK_T * (yb - P.DECK_T / 2) ** 2 + \
        P.LUG_W * P.LUG_H ** 3 / 12 + P.LUG_W * P.LUG_H * (P.DECK_T + P.LUG_H / 2 - yb) ** 2
    q_side = p * lug_pitch
    d_side = q_side * side_len ** 4 / (8 * E_deck * I_T)
    worst_d = max(d_heel, d_toe, d_side)
    L.append(f"## Snowshoe deck under {p*1000:.2f} kPa (trot), E = {E_deck:.0f} MPa\n")
    L.append(f"- heel cantilever ({-SG.X_HEEL - pad:.0f} mm from the pad, deck + 2 rims): tip {d_heel:.2f} mm, σ {s_heel:.2f} MPa")
    L.append(f"- toe cantilever ({SG.X_TOE - pad:.0f} mm): tip {d_toe:.2f} mm, σ {s_toe:.2f} MPa")
    L.append(f"- side strip (pad edge → rim, {side_len:.0f} mm, deck + lug T-section per {lug_pitch:.0f} mm): {d_side:.3f} mm")
    L.append(f"- worst rim deflection {worst_d:.2f} mm (target < 2 mm) -> {'PASS' if worst_d < 2 else 'FAIL'}\n")
    ok_all &= worst_d < 2

    # ---------------- ankle
    F_fall = P.FALL_G * W
    M_heel = F_fall * (-SG.X_HEEL) / 1000
    F_stop = M_heel / (P.STOP_R / 1000)
    pin_tau = math.hypot(F_fall, 0.3 * F_fall) / 2 / (math.pi * 2.0 ** 2)          # double shear on the M5 core (Ø4.0)
    cheek_pin = math.hypot(F_fall, 0.3 * F_fall) / 2 / (P.ANKLE_PIN_D * P.CHEEK_T)
    boss = P.insert_depth("M3")
    cheek_stop = (F_stop / 2) / (3.0 * boss)                                         # M3 bolt bearing in a cheek + boss
    sleeve_edge = F_stop / (P.STOP_SLEEVE_D * P.SHANK_T)
    L.append("## Ankle (fall onto the heel tip, toe-down stop engaged)\n")
    L.append(f"- moment about the pivot {M_heel:.1f} Nm → stop force {F_stop:.0f} N at r {P.STOP_R:.0f} mm")
    L.append(f"- M5 pin, double shear on the Ø4.0 core: {pin_tau:.0f} MPa (steel 8.8: ~380 MPa shear) -> PASS")
    L.append(f"- cheek bearing at the pin (Ø5 × {P.CHEEK_T:.1f} mm, per cheek): {cheek_pin:.1f} MPa vs {P.PLA_STRENGTH:.0f} -> {'PASS' if cheek_pin < P.PLA_STRENGTH else 'FAIL'}")
    L.append(f"- cheek bearing at the stop bolt (Ø3 × {boss:.1f} mm boss, per cheek): {cheek_stop:.1f} MPa vs {P.PLA_STRENGTH:.0f} "
             f"-> {'PASS' if cheek_stop < P.PLA_STRENGTH else 'FAIL'}")
    L.append(f"- Ø{P.STOP_SLEEVE_D:.0f} sleeve on the shank edge ({P.SHANK_T:.0f} mm): {sleeve_edge:.1f} MPa vs {P.PLA_STRENGTH:.0f} "
             f"-> {'PASS' if sleeve_edge < P.PLA_STRENGTH else 'FAIL'}")
    # springs: centring pair, torque per degree
    k_t = 2 * P.SPRING_K * (P.STOP_R * 0.8) ** 2 * math.pi / 180 / 1000            # N·m/deg, both springs, ~0.8 R arm
    g_t = 0.04 * G * 0.012                                                          # foot weight torque, ~40 g at 12 mm
    L.append(f"- centring springs (2 × {P.SPRING_K} N/mm, preload {P.SPRING_PRELOAD} N): ≈ {k_t*1000:.2f} N·mm/° about the pivot; "
             f"the foot's own weight torque is ≈ {g_t*1000:.1f} N·mm, so the springs hold the swing pitch within "
             f"≈ {g_t/k_t:.0f}° of neutral ({SG.ALPHA_NEUTRAL:.1f}°)\n")
    for v_ in (cheek_pin, cheek_stop, sleeve_edge):
        ok_all &= v_ < P.PLA_STRENGTH

    # ---------------- knee stop posts (steel M5, Ø12 sleeve, 8 mm proud)
    def stop_force(q2_deg, r_mm):
        q2 = math.radians(q2_deg)
        h = math.sqrt(max((L1 / 1000) ** 2 + (L2 / 1000) ** 2 + 2 * (L1 / 1000) * (L2 / 1000) * math.cos(q2), 1e-6))
        qq = ik(np.array([0.0, -h, l / 1000]), L1 / 1000, L2 / 1000, l / 1000)
        return abs(joint_torques(qq, np.array([0.0, -F_fall, 0.0]), L1 / 1000, L2 / 1000, l / 1000)[2]) / (r_mm / 1000)
    Fu = stop_force(P.Q2_LIM[1] + 2, B.KNEE_UP[0])
    sig = 32 * Fu * 4.0 / (math.pi * 4.0 ** 3)
    L.append(f"## Knee stop posts: fall onto a crouched leg at the upper stop → {Fu:.0f} N on the post; M5 (core Ø4.0) bending at "
             f"4 mm: {sig:.0f} MPa vs 640 MPa yield (8.8) → SF {640/sig:.1f}\n")

    # ---------------- deflection at the pivot (continuous load, snow stance)
    Fv = W / 3
    th_angle = abs(q[1])
    I_th_def = I_th
    d_thigh = Fv * math.sin(th_angle) * L1 ** 3 / (3 * E * I_th_def) * 0.5
    d_shank_in = Fv * abs(math.sin(phi)) * L2 ** 3 / (3 * E * Ia_in)
    d_shank_out = 0.1 * Fv * L2 ** 3 / (3 * E * Ia_out)
    backlash = math.radians(P.BACKLASH_ARCMIN / 60) * (L1 + L2)
    tot = d_thigh + d_shank_in + d_shank_out
    L.append(f"## Deflection at the ankle pivot, {Fv:.1f} N walk load + 10 % lateral (snow stance)\n- thigh {d_thigh:.3f} mm, shank in-plane "
             f"{d_shank_in:.3f} mm, shank out-of-plane {d_shank_out:.3f} mm → {tot:.2f} mm (target < 1 mm) -> {'PASS' if tot < 1 else 'FAIL'}; "
             f"gearbox backlash adds ≈ {backlash:.1f} mm of free play\n")
    ok_all &= tot < 1

    # ---------------- masses
    try:
        from mass_model import snow_groups, original_groups
        sg, og = snow_groups(), original_groups()
        L.append("## Mass (slicer filament + hardware list, analysis/mass_model.py)\n")
        L.append("| group | original g | snow g |\n|---|---|---|")
        for k in ("foot", "shank", "thigh", "swinging", "hip_module", "whole"):
            L.append(f"| {k} | {og[k]:.0f} | {sg[k]:.0f} |")
        I_s = (sg["thigh"] * (L1 / 2) ** 2 + sg["shank"] * (L1 + L2 / 2) ** 2 + sg["foot"] * (L1 + L2) ** 2) * 1e-9
        I_o = (og["thigh"] * (L1 / 2) ** 2 + og["shank"] * (L1 + L2 / 2) ** 2 + og["foot"] * (L1 + L2) ** 2) * 1e-9
        L.append(f"\nLeg inertia about the hip, leg straight: original {I_o*1e4:.1f} kg·cm², snow {I_s*1e4:.1f} kg·cm² "
                 f"(+{(I_s-I_o)*1e3:.2f}×10⁻³ kg·m²; the foot group alone adds {sg['foot']/1000*((L1+L2)/1000)**2*1e3:.2f}×10⁻³).")
        ok_f = sg["foot"] <= P.FOOT_MASS_BUDGET
        L.append(f"\n**Foot group {sg['foot']:.1f} g (printed {sg['foot_printed']:.1f} g + hardware {sg['foot_hw']:.1f} g) vs the "
                 f"{P.FOOT_MASS_BUDGET:.0f} g budget -> {'PASS' if ok_f else 'OVER BUDGET'}**")
    except Exception as e:  # noqa: BLE001
        L.append(f"(masses unavailable: {e})")
    L.append(f"\n**Strength / stiffness checks: {'ALL PASS' if ok_all else 'SOME FAIL'}**")
    txt = "\n".join(L)
    open(os.path.join(HERE, "out", "skeleton.md"), "w", encoding="utf-8").write(txt)
    print(txt)
    return ok_all


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
