"""Snow model for the snow leg (brief SNOW_LEG_PROMPT.md section 4).

- Bekker  p = (k_c/b + k_phi) z^n      (p kPa, z m, b = smaller plan dimension / radius)
- Preston-Thomas  p = k [-ln(1 - z/z_m)]  ->  z = z_m (1 - exp(-p/k))   (k treated as kPa)
- compaction work W = A * integral_0^z p dz
- traction F_t = A c + W_n tan(phi)
- stepped sole: central pad PAD_PROUD below the deck; the deck engages after that much sinkage
Parameters: Lines, Elliott & Ray (2021), compiling Wong (2001) + Canadian field data (brief table 4.2).
Writes analysis/out/snow.md and the plots snow_*.png.
Run:  python analysis/snow_model.py
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
import params as P  # noqa: E402
from leg_model import ik, joint_torques, motor_torques  # noqa: E402

OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)
G = P.G_ACC

BEKKER = {"B1": (1.6, 4.37, 196.72), "B2": (1.6, 2.49, 245.90), "B3": (1.44, 10.55, 66.08)}
PRESTON = {"P1 Summit (GL)": (59.0, 0.06), "P2 Lebel (QC)": (65.0, 0.29), "P3 Lebel": (56.4, 0.35),
           "P4 Lebel": (27.3, 0.50), "P5 Lebel": (19.2, 0.54)}
SHEAR = {"poorly bonded": (0.6, 9.0), "well bonded": (6.0, 23.0)}       # c kPa, phi deg

# foot loads of snow animals, g/cm^2 (brief table section 2, with sources)
ANIMALS = [("marten", 9.4, 9.4, "Renard 2008"), ("wolverine", 22, 22, "Renard 2008"),
           ("lynx", 25.9, 31.6, "Murray & Boutin 1991"), ("fisher", 25, 36, "Renard 2008"),
           ("reindeer, hooves + dewclaws", 85, 102, "Nieminen 1990"), ("red fox", 88.5, 88.5, "Renard 2008"),
           ("coyote", 106, 170, "Murray & Boutin 1991; Renard 2008"), ("reindeer, hooves only", 235, 268, "Nieminen 1990"),
           ("moose", 618, 922, "Kelsall 1969")]

CASES = {"static (4 down)": 4, "walk (3 down)": 3, "trot (2 down)": 2}


def per_foot_N(case, mass=P.ROBOT_MASS_SNOW):
    if case == "trot peak":
        return 1.5 * mass * G / 2
    return mass * G / CASES[case]


# ------------------------------------------------------------------ geometry
def teardrop(L, W, r_tail):
    """Area (mm^2), perimeter (mm), centroid x from the tail tip (mm) of the planform."""
    r0, r1 = W / 2, r_tail
    d = L - r0 - r1
    a = math.acos((r0 - r1) / d)                   # tangent half-angle (same construction as swlib.stadium)
    # polygon approximation of the exact outline (fine sampling)
    pts = []
    for t in np.linspace(-a, a, 400):                              # toe arc (centre at x = d)
        pts.append((d + r0 * math.cos(t), r0 * math.sin(t)))
    for t in np.linspace(a, 2 * math.pi - a, 400):                 # tail arc (centre at 0)
        pts.append((r1 * math.cos(t), r1 * math.sin(t)))
    pts = np.array(pts)
    x, y = pts[:, 0], pts[:, 1]
    xs, ys = np.roll(x, -1), np.roll(y, -1)
    cr = x * ys - xs * y
    A = cr.sum() / 2
    cx = ((x + xs) * cr).sum() / (6 * A)
    per = np.hypot(xs - x, ys - y).sum()
    return abs(A), per, cx + r1


def ellipse_area(L, W):
    return math.pi * L * W / 4


# ------------------------------------------------------------------ sinkage models
def z_bekker(p_kpa, b_m, s):
    n, kc, kphi = BEKKER[s]
    return (p_kpa / (kc / b_m + kphi)) ** (1 / n)


def p_bekker(z, b_m, s):
    n, kc, kphi = BEKKER[s]
    return (kc / b_m + kphi) * max(z, 0.0) ** n


def z_preston(p_kpa, s):
    k, zm = PRESTON[s]
    return zm * (1 - math.exp(-p_kpa / k))


def p_preston(z, s):
    k, zm = PRESTON[s]
    z = min(max(z, 0.0), zm * 0.999999)
    return k * -math.log(1 - z / zm)


def p_of(z, s, b):
    return p_bekker(z, b, s) if s in BEKKER else p_preston(z, s)


def sink(F_N, A_m2, b_m, s):
    p = F_N / A_m2 / 1000
    return z_bekker(p, b_m, s) if s in BEKKER else z_preston(p, s)


def sink_stepped(F_N, A_deck, b_deck, A_pad, b_pad, proud_m, s):
    """Pad carries load alone until it has sunk proud_m; then pad + rest of the deck share.
    Returns sinkage of the pad bottom (m) and of the deck/lug tips (m)."""
    if proud_m <= 0:
        z = sink(F_N, A_deck, b_deck, s)
        return z, z
    zp = sink(F_N, A_pad, b_pad, s)
    if zp <= proud_m:
        return zp, 0.0
    A_rest = A_deck - A_pad
    lo, hi = proud_m, 2.0
    for _ in range(80):
        z = (lo + hi) / 2
        f = 1000 * (A_pad * p_of(z, s, b_pad) + A_rest * p_of(z - proud_m, s, b_deck))
        lo, hi = (z, hi) if f < F_N else (lo, z)
    return z, z - proud_m


def compaction_work(F_N, A, b, s):
    z = sink(F_N, A, b, s)
    zz = np.linspace(0, z, 400)
    return A * 1000 * np.trapezoid([p_of(v, s, b) for v in zz], zz), z


def traction(F_N, A, c_kpa, phi_deg):
    return A * c_kpa * 1000 + F_N * math.tan(math.radians(phi_deg))


# ------------------------------------------------------------------ foot mass estimate (printed)
def foot_mass_g(L, W, r_tail, proud=P.PAD_PROUD):
    A, per, _ = teardrop(L, W, r_tail)
    deck = A * P.DECK_T * (1.2 + (P.DECK_T - 1.2) * 0.15) / P.DECK_T        # 0.6 mm top + bottom skins, 15 % infill core
    rim = per * P.RIM_W * P.LUG_H
    lugs = P.N_LUGS * (0.8 * W) * P.LUG_W * P.LUG_H
    pad = (P.PAD_L * P.PAD_W * 0.785 - (P.PAD_L - 2 * P.LUG_W) * (P.PAD_W - 2 * P.LUG_W) * 0.785) * (P.LUG_H + proud)
    paps = min(P.PAP_MAX, A / P.PAP_PITCH ** 2 * 0.5) * math.pi * P.PAP_D ** 2 / 4 * P.PAP_H
    return (deck + rim + lugs + pad + paps) * P.PLA_RHO * 1000


# ------------------------------------------------------------------ main
def main():
    L = ["# Snow model (analysis/snow_model.py)\n",
         f"Robot {P.ROBOT_MASS_SNOW} kg (ASSUMED). Loads per foot: "
         + ", ".join(f"{c} {per_foot_N(c):.1f} N" for c in list(CASES) + ["trot peak"]) + ".\n",
         "Parameters: Lines, Elliott & Ray (2021) compiling Wong (2001); plate models for homogeneous snow — "
         "calibrate on real snow (docs/SNOW_TEST_PLAN.md) before trusting absolute numbers.\n"]

    # ---------------- anchors (brief 4.3)
    L.append("## Sanity anchors (brief 4.3)\n")
    F_tr, F_st = per_foot_N("trot (2 down)"), per_foot_N("static (4 down)")
    A_ball, b_ball = math.pi * 0.01 ** 2, 0.010
    A_ss, b_ss = ellipse_area(120, 80) * 1e-6, 0.080            # the brief's 75 cm^2 reference shoe
    checks = []

    def anchor(name, val, lo, hi, unit):
        ok = lo - 1e-9 <= val <= hi + 1e-9
        checks.append(ok)
        L.append(f"| {name} | {val:.2f} {unit} | {lo}–{hi} | {'PASS' if ok else 'FAIL'} |")

    L.append("| anchor | model | brief | |\n|---|---|---|---|")
    anchor("ball R10 trot pressure", F_tr / A_ball / 1000, 69, 71, "kPa")
    anchor("ball R10 trot foot load", F_tr / G * 1000 / (A_ball * 1e4), 710, 722, "g/cm²")
    zb = [sink(F_tr, A_ball, b_ball, s) * 100 for s in BEKKER]
    zp = [sink(F_tr, A_ball, b_ball, s) * 100 for s in list(PRESTON)[1:]]
    anchor("ball trot sinkage Bekker min", min(zb), 14.5, 15.5, "cm"); anchor("ball trot sinkage Bekker max", max(zb), 29.5, 30.5, "cm")
    anchor("ball trot sinkage Preston min (P2-P5)", min(zp), 18.5, 19.5, "cm"); anchor("ball trot sinkage Preston max", max(zp), 52.5, 53.5, "cm")
    anchor("snowshoe trot pressure", F_tr / A_ss / 1000, 2.85, 2.95, "kPa")
    anchor("snowshoe trot foot load", F_tr / G * 1000 / (A_ss * 1e4), 29.5, 30.1, "g/cm²")
    zb = [sink(F_tr, A_ss, b_ss, s) * 100 for s in BEKKER]
    zp = [sink(F_tr, A_ss, b_ss, s) * 100 for s in list(PRESTON)[1:]]
    anchor("snowshoe trot Bekker min", min(zb), 5.35, 5.45, "cm"); anchor("snowshoe trot Bekker max", max(zb), 6.15, 6.25, "cm")
    anchor("snowshoe trot Preston min", min(zp), 1.25, 1.35, "cm"); anchor("snowshoe trot Preston max", max(zp), 7.55, 7.75, "cm")
    zb = [sink(F_st, A_ss, b_ss, s) * 100 for s in BEKKER]
    zp = [sink(F_st, A_ss, b_ss, s) * 100 for s in list(PRESTON)[1:]]
    anchor("snowshoe static Bekker min", min(zb), 3.25, 3.35, "cm"); anchor("snowshoe static Bekker max", max(zb), 3.95, 4.05, "cm")
    anchor("snowshoe static Preston min", min(zp), 0.55, 0.65, "cm"); anchor("snowshoe static Preston max", max(zp), 3.95, 4.05, "cm")
    anchor("traction snowshoe, trot, poorly bonded", traction(F_tr, A_ss, *SHEAR["poorly bonded"]), 7.9, 8.1, "N")
    anchor("traction ball, trot, poorly bonded", traction(F_tr, A_ball, *SHEAR["poorly bonded"]), 3.6, 3.8, "N")
    anchor("traction snowshoe, trot, well bonded", traction(F_tr, A_ss, *SHEAR["well bonded"]), 54, 56, "N")
    W, _ = compaction_work(F_tr, A_ss, b_ss, "P5 Lebel")
    anchor("compaction work per trot step, P5", W, 0.75, 0.85, "J")
    L.append(f"\n**Anchors: {'ALL PASS' if all(checks) else 'SOME FAIL'}** ({sum(checks)}/{len(checks)})\n")

    # ---------------- foot sweep
    L.append("## Foot sweep (teardrop planform, walk = snow gait load, trot for reference)\n")
    L.append("| foot L×W | area cm² | perim mm | trot g/cm² | walk sink B1–B3 cm | walk sink P4 / P5 cm | trot sink P5 cm | "
             "compaction J (walk, P5) | est. foot g | added hip inertia ×1e-3 kg m² |\n|---|---|---|---|---|---|---|---|---|---|")
    F_w = per_foot_N("walk (3 down)")
    sweep = []
    for (Lf, Wf) in ((100, 70), (110, 75), (120, 80), (130, 90)):
        rt = Wf / 2 - 6
        A, per, _ = teardrop(Lf, Wf, rt)
        Am, b = A * 1e-6, Wf / 1000
        fl = F_tr / G * 1000 / (A / 100)
        zb = [sink(F_w, Am, b, s) * 100 for s in BEKKER]
        z4, z5 = sink(F_w, Am, b, "P4 Lebel") * 100, sink(F_w, Am, b, "P5 Lebel") * 100
        zt5 = sink(F_tr, Am, b, "P5 Lebel") * 100
        Wc, _ = compaction_work(F_w, Am, b, "P5 Lebel")
        m = foot_mass_g(Lf, Wf, rt) + 12.0            # + cheeks, pin, spring, spikes, inserts (estimate)
        I = m / 1000 * ((P.L1 + P.L2) / 1000) ** 2
        sweep.append((Lf, Wf, A, fl, z5, m))
        L.append(f"| {Lf}×{Wf} | {A/100:.1f} | {per:.0f} | {fl:.1f} | {min(zb):.1f}–{max(zb):.1f} | {z4:.1f} / {z5:.1f} | {zt5:.1f} | "
                 f"{Wc:.2f} | {m:.0f} | {I*1e3:.2f} |")
    A_sel, per_sel, cx_tail = teardrop(P.FOOT_L, P.FOOT_W, P.FOOT_R_TAIL)
    L.append(f"\n**Chosen: {P.FOOT_L:.0f} × {P.FOOT_W:.0f} mm, tail radius {P.FOOT_R_TAIL:.0f}** → area {A_sel/100:.1f} cm², "
             f"trot foot load {F_tr/G*1000/(A_sel/100):.1f} g/cm² (lynx 25.9–31.6), within the 130 × 90 limit. "
             "100 × 70 and 110 × 75 exceed lynx-level loading in a trot; 130 × 90 sinks ~1 cm less but adds mass, inertia and "
             "clash risk between front and hind feet.\n")

    # ---------------- stepped pad
    L.append("## Stepped 'hoof' pad (P2): PAD_PROUD 0 / 6 / 12 mm, walk load\n")
    A_pad = math.pi * P.PAD_L * P.PAD_W / 4 * 1e-6
    b_pad = P.PAD_W / 1000
    L.append("| PAD_PROUD | " + " | ".join(f"{s} pad/deck cm" for s in ("B1", "B3", "P4 Lebel", "P5 Lebel")) + " | ride height lost on rock |\n|---|---|---|---|---|---|")
    for pr in (0, 6, 12):
        cells = []
        for s in ("B1", "B3", "P4 Lebel", "P5 Lebel"):
            zp_, zd_ = sink_stepped(F_w, A_sel * 1e-6, P.FOOT_W / 1000, A_pad, b_pad, pr / 1000, s)
            cells.append(f"{zp_*100:.1f} / {zd_*100:.1f}")
        L.append(f"| {pr} mm | " + " | ".join(cells) + f" | +{pr} mm rocker under a free ankle |")
    L.append("\nThe pad adds sinkage at the pad and gains nothing in soft snow (the deck sinks about the same). "
             "On rock or ice a proud pad makes the foot a rocker under a free-pitch ankle and lifts the spikes off the "
             "surface (spikes protrude only 3 mm). **PAD_PROUD = 0 chosen**: the pad is flush with the lug tips and acts "
             "as a solid 'hoof' zone (insert bosses + bracket load path). Revisit with real-snow plate data.\n")

    # ---------------- belly clearance, L2 sweep
    L.append("## Belly clearance vs sinkage, L2 sweep (snow stance, walk load)\n")
    ankle_h = P.PIVOT_ABOVE_DECK + P.DECK_T + P.LUG_H + P.PAD_PROUD
    L.append(f"Ankle pivot {ankle_h:.0f} mm above the sole. Stance: pivot at {P.SNOW_STANCE_FRAC:.2f}·(L1+L2) below the hip. "
             f"Belly = {P.BODY_BELOW_HIP:.0f} mm below the hip axis (ASSUMED). Target: sinkage ≤ 25 % of belly clearance.\n")
    L.append("| L2 mm | hip height mm | belly clearance mm | 25 % mm | walk sink P4 / P5 mm | pass | knee q2° | belt N (walk) | hip / knee motor Nm |\n|---|---|---|---|---|---|---|---|---|")
    l = P.L_ABAD / 1000
    rk, rh = P.PD_KNEE / 2000, P.PD_HIP / 2000
    for L2 in (130, 145, 160):
        Hp = P.SNOW_STANCE_FRAC * (P.L1 + L2)
        Hh = Hp + ankle_h
        clr = Hh - P.BODY_BELOW_HIP
        z4 = sink(F_w, A_sel * 1e-6, P.FOOT_W / 1000, "P4 Lebel") * 1000
        z5 = sink(F_w, A_sel * 1e-6, P.FOOT_W / 1000, "P5 Lebel") * 1000
        q = ik(np.array([0.0, -Hp / 1000, l]), P.L1 / 1000, L2 / 1000, l)
        worst_belt, worst = 0, (0, 0)
        for x in np.linspace(-P.SNOW_STEP / 2, P.SNOW_STEP / 2, 13):
            qq = ik(np.array([x, -Hp / 1000, l]), P.L1 / 1000, L2 / 1000, l)
            tq = joint_torques(qq, np.array([0, -F_w, 0.1 * F_w]), P.L1 / 1000, L2 / 1000, l)
            tm = motor_torques(tq, P.N_RATIO)
            worst_belt = max(worst_belt, abs(tq[2]) / rk)
            worst = (max(worst[0], abs(tm[1])), max(worst[1], abs(tm[2])))
        ok = max(z4, z5) <= 0.25 * clr
        L.append(f"| {L2} | {Hh:.0f} | {clr:.0f} | {0.25*clr:.0f} | {z4:.0f} / {z5:.0f} | {'PASS' if ok else 'FAIL'} | "
                 f"{math.degrees(q[2]):.0f} | {worst_belt:.0f} | {worst[0]:.2f} / {worst[1]:.2f} |")
    L.append(f"\nL2 = {P.L2:.0f} mm kept: it already passes the clearance target in P4/P5 with the taller stance, keeps L1 = L2 "
             "(no unreachable hole), and a longer shank raises distal mass and knee torque in the fold. The L2 lever is "
             "available (one parameter) if real snow is softer than P5.\n")

    # ---------------- traction
    L.append("## Traction (A c + W tan φ) vs need (30 % of vertical)\n")
    L.append("| foot | case | poorly bonded N | well bonded N | needed N | poorly-bonded verdict |\n|---|---|---|---|---|---|")
    for nm, A, in (("snowshoe (chosen)", A_sel * 1e-6), ("ball R10", A_ball)):
        for case in ("walk (3 down)", "trot (2 down)"):
            F = per_foot_N(case)
            tp = traction(F, A, *SHEAR["poorly bonded"])
            tw = traction(F, A, *SHEAR["well bonded"])
            need = 0.3 * F
            L.append(f"| {nm} | {case} | {tp:.1f} | {tw:.1f} | {need:.1f} | {'PASS' if tp >= need else 'SHORT'} |")
    L.append("\nLugs make the shear surface snow-on-snow below the sole, so A is the full planform. On lake/sea ice "
             "traction comes from the 4 steel spikes (3 mm proud), not from this model.\n")

    # ---------------- crust, snow load
    per_ball = 2 * math.pi * 10
    L.append("## Crust and snow load\n")
    L.append(f"- Punch-through resistance scales with perimeter × crust thickness: snowshoe perimeter {per_sel:.0f} mm vs ball "
             f"contact ≈ {per_ball:.0f} mm → ≈ {per_sel/per_ball:.0f}× the crust load before breaking through (qualitative; test it).")
    L.append(f"- Snow on the deck: 10 mm of 300 kg/m³ snow on {A_sel/100:.0f} cm² = {300*0.010*A_sel*1e-6*1000:.0f} g "
             "(P7) — the top face is flat (it is the print bed face); shake-off relies on the swing and the toe-up tilt.\n")

    txt = "\n".join(L)
    with open(os.path.join(OUT, "snow.md"), "w", encoding="utf-8") as f:
        f.write(txt)
    print(txt)
    plots(A_sel, sweep)
    return all(checks)


def plots(A_sel, sweep):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ink, acc, mute = "#1f2328", "#2f6fdf", "#8b949e"
    F_st, F_tr = per_foot_N("static (4 down)"), per_foot_N("trot (2 down)")
    rob = [("robot, ball foot (static)", F_st / G * 1000 / math.pi, "robot"),
           ("robot, ball foot (trot)", F_tr / G * 1000 / math.pi, "robot"),
           ("robot, snowshoe (static)", F_st / G * 1000 / (A_sel / 100), "shoe"),
           ("robot, snowshoe (trot)", F_tr / G * 1000 / (A_sel / 100), "shoe")]
    rows = [(n, lo, hi, "animal") for n, lo, hi, _ in ANIMALS] + [(n, v, v, k) for n, v, k in rob]
    rows.sort(key=lambda r: r[2])
    fig, ax = plt.subplots(figsize=(8, 5.2), dpi=110)
    for i, (n, lo, hi, kind) in enumerate(rows):
        col = {"animal": mute, "robot": "#d1242f", "shoe": acc}[kind]
        ax.barh(i, hi, color=col, alpha=0.35 if kind == "animal" else 0.9, height=0.65)
        if hi > lo:
            ax.plot([lo, hi], [i, i], color=col, lw=3)
        ax.text(hi * 1.05, i, f"{hi:.0f}" if lo == hi else f"{lo:.0f}–{hi:.0f}", va="center", fontsize=7.5, color=ink)
    ax.set_yticks(range(len(rows)), [r[0] for r in rows], fontsize=8)
    ax.set_xscale("log")
    ax.set_xlabel("foot load, g/cm² (log)")
    ax.axvline(30, color=acc, ls="--", lw=1)
    ax.text(31, len(rows) - 0.6, "lynx level ≈ 30", color=acc, fontsize=8)
    ax.set_title("Foot load: robot vs snow animals (brief §2 sources)", fontsize=10)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "snow_foot_load.png"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4.5), dpi=110)
    areas = np.linspace(20, 120, 60)
    F_w = per_foot_N("walk (3 down)")
    for s in list(BEKKER) + list(PRESTON):
        ax.plot(areas, [sink(F_w, a * 1e-4, math.sqrt(a * 1e-4 / 1.5), s) * 100 for a in areas], label=s, lw=1.4)
    ax.axvline(A_sel / 100, color=ink, ls="--", lw=1)
    ax.text(A_sel / 100 + 1, 1, "chosen foot", fontsize=8)
    ax.set_xlabel("foot area, cm² (L ≈ 1.5 W)")
    ax.set_ylabel("sinkage at walk load, cm")
    ax.set_title(f"Sinkage vs foot area, {F_w:.1f} N per foot (walk)", fontsize=10)
    ax.legend(fontsize=7, ncol=2)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "snow_sinkage_vs_area.png"))
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
