"""Snow-leg gait check (SNOW_MODE) + walking animation.

Four copies of this leg on a P.ROBOT_MASS_SNOW robot. The IK target is the ankle pivot; with a
free-pitch ankle the ground force passes through the pivot, so tau = J^T F at the pivot is exact.
SNOW_MODE gait (brief 4.4-2):
  - taller 'stilts' stance: pivot at SNOW_STANCE_FRAC * (L1 + L2) below the hip (>= 15-20 deg knee bend)
  - stance: foot still on the snow, sole level -> ankle alpha = phi (= q1 + q2)
  - swing: vertical lift out of the hole, forward at a height solved so the lowest point of the
    (centring-spring-held, toe-up) foot clears the predicted P5 snow surface by SNOW_SWING_MARGIN,
    vertical plunge into the new print
  - lateral-sequence walk, duty 0.75, slow; stride / duty = hip spacing / 2 -> direct register
Loads: stance = weight over the feet down + 10 % lateral; swing = link gravity + inertia with the
foot mass at the pivot. Reports continuous vs peak like the Leg Day gait.md.
Writes analysis/out/gait.md, analysis/out/gait.json, docs/renders/snow_walk_cycle.gif.
Run:  python analysis/gait_check.py [--no-anim]
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
import params as P  # noqa: E402
import snowgeo as SG  # noqa: E402
from leg_model import ik, joint_torques, motor_torques  # noqa: E402
from snow_model import sink  # noqa: E402

OUT = os.path.join(HERE, "out")
REND = os.path.join(ROOT, "docs", "renders")
G = P.G_ACC
L1, L2, l = P.L1 / 1000, P.L2 / 1000, P.L_ABAD / 1000
N = P.N_RATIO
R_HIP, R_KNEE = P.PD_HIP / 2000, P.PD_KNEE / 2000
OMEGA_MAX = P.SPEED_RATED * 2 * math.pi / 60
ROBOT_MASS = P.ROBOT_MASS_SNOW
STEP = P.SNOW_STEP
HIP_SPACING = P.HIP_SPACING_SNOW
SNOW_SET = "P5 Lebel"                    # softest set in the brief's table drives the swing height


def masses():
    try:
        from mass_model import snow_groups
        g = snow_groups()
        return g["thigh"] / 1000, g["shank"] / 1000, g["foot"] / 1000
    except Exception:  # noqa: BLE001
        return 0.075, 0.035, 0.035


M_THIGH, M_SHANK, M_FOOT = masses()


def gaits(snow=True):
    if snow:
        return {"walk": dict(T=P.SNOW_PERIOD, duty=P.SNOW_DUTY, phase={"LH": 0.0, "LF": 0.25, "RH": 0.5, "RF": 0.75}),
                "trot": dict(T=0.8, duty=0.5, phase={"LF": 0.0, "RH": 0.0, "RF": 0.5, "LH": 0.5})}
    return {"walk": dict(T=1.0, duty=0.75, phase={"LH": 0.0, "LF": 0.25, "RH": 0.5, "RF": 0.75}),
            "trot": dict(T=0.5, duty=0.5, phase={"LF": 0.0, "RH": 0.0, "RF": 0.5, "LH": 0.5})}


def stance_height(snow=True):
    return (P.SNOW_STANCE_FRAC if snow else P.STANCE_FRAC) * (L1 + L2)


def predicted_sink(case_N):
    return sink(case_N, SG.AREA * 1e-6, P.FOOT_W / 1000, SNOW_SET)


def swing_xy(u, h, f_l=P.SNOW_LIFT_FRAC):
    """Swing phase u in [0,1): vertical lift, forward cycloid at height h, vertical plunge. Returns (x, dy)."""
    if u < f_l:
        return -STEP / 2, h * (u / f_l)
    if u > 1 - f_l:
        return STEP / 2, h * ((1 - u) / f_l)
    v = (u - f_l) / (1 - 2 * f_l)
    return -STEP / 2 + STEP * (v - math.sin(2 * math.pi * v) / (2 * math.pi)), h


def foot_pose(s, duty, H, h, snow=True):
    """Pivot position relative to the hip (m) and stance flag."""
    if s < duty:
        u = s / duty
        return STEP / 2 - STEP * u, -H, True
    u = (s - duty) / (1 - duty)
    if snow:
        x, dy = swing_xy(u, h)
        return x, -H + dy, False
    x = -STEP / 2 + STEP * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
    return x, -H + h * math.sin(math.pi * u), False


def swing_alpha():
    return min(max(SG.ALPHA_NEUTRAL, SG.ALPHA_MIN), SG.ALPHA_MAX)


def swing_clearance(h, H, z_sink, n=60):
    """Min clearance (mm) of the lowest foot point above the snow surface during the forward part of swing."""
    worst = 1e9
    f_l = P.SNOW_LIFT_FRAC
    for u in np.linspace(f_l, 1 - f_l, n):
        x, dy = swing_xy(u, h)
        q = ik(np.array([x, -H + dy, l]), L1, L2, l)
        phi = math.degrees(q[1] + q[2])
        psi = phi - swing_alpha()
        low = SG.foot_low_point(psi) / 1000                    # m, below the pivot (negative)
        surface = -H - SG.ANKLE_H / 1000 + z_sink              # stance hole bottom + sinkage
        worst = min(worst, ((-H + dy) + low - surface) * 1000)
    return worst


def solve_lift(H, z_sink):
    for h_mm in np.arange(40.0, 220.0, 1.0):
        if swing_clearance(h_mm / 1000, H, z_sink) >= P.SNOW_SWING_MARGIN:
            return h_mm / 1000
    return 0.22


def analyse(g, snow=True, n=240):
    T, duty = g["T"], g["duty"]
    H = stance_height(snow)
    F_walk = ROBOT_MASS * G / 3
    z_sink = predicted_sink(F_walk) if snow else 0.0
    h = solve_lift(H, z_sink) if snow else 0.040
    dt = T / n
    rows = []
    for k in range(n):
        s = k / n
        x, y, st = foot_pose(s, duty, H, h, snow)
        q = ik(np.array([x, y, l]), L1, L2, l)
        n_down = sum(1 for ph in g["phase"].values() if ((s + ph) % 1.0) < duty)
        phi = math.degrees(q[1] + q[2])
        if st:
            F = np.array([0.0, -ROBOT_MASS * G / n_down, 0.0])
            F[2] = 0.1 * abs(F[1])
            tq = joint_torques(q, F, L1, L2, l)
            alpha = phi                                         # sole level
        else:
            th = q[1]
            kx = L1 * math.sin(th)
            px = kx + L2 * math.sin(th + q[2])
            tq = np.array([0.0,
                           -(M_THIGH * G * L1 / 2 * math.sin(th) + M_SHANK * G * (kx + L2 / 2 * math.sin(th + q[2])) + M_FOOT * G * px),
                           -(M_SHANK * G * L2 / 2 * math.sin(th + q[2]) + M_FOOT * G * L2 * math.sin(th + q[2]))])
            alpha = swing_alpha()
        rows.append(dict(s=s, x=x, y=y, stance=st, q=q, tq=tq, n_down=n_down, phi=phi, alpha=alpha))
    qs = np.array([r["q"] for r in rows])
    qd = (np.roll(qs, -1, 0) - np.roll(qs, 1, 0)) / (2 * dt)
    qdd = (np.roll(qs, -1, 0) - 2 * qs + np.roll(qs, 1, 0)) / dt ** 2
    I1 = M_THIGH * (L1 / 2) ** 2 + M_SHANK * (L1 + L2 / 2) ** 2 + M_FOOT * (L1 + L2) ** 2
    I2 = M_SHANK * (L2 / 2) ** 2 + M_FOOT * L2 ** 2
    w = dict(q1=[1e9, -1e9], q2=[1e9, -1e9], alpha_st=[1e9, -1e9], w_hip=0, w_knee=0, t_hip=0, t_knee=0, t_abad=0, belt=0)
    for i, r in enumerate(rows):
        tq = r["tq"].copy()
        if not r["stance"]:
            tq[1] += I1 * qdd[i, 1]
            tq[2] += I2 * qdd[i, 2]
        tm = motor_torques(tq, N)
        r["tm"], r["belt"] = tm, abs(tq[2]) / R_KNEE
        q1, q2 = math.degrees(r["q"][1]), math.degrees(r["q"][2])
        w["q1"] = [min(w["q1"][0], q1), max(w["q1"][1], q1)]
        w["q2"] = [min(w["q2"][0], q2), max(w["q2"][1], q2)]
        if r["stance"]:
            w["alpha_st"] = [min(w["alpha_st"][0], r["alpha"]), max(w["alpha_st"][1], r["alpha"])]
        w["w_hip"] = max(w["w_hip"], abs(qd[i, 1]))
        w["w_knee"] = max(w["w_knee"], abs(qd[i, 1] + N * qd[i, 2]))
        w["t_abad"] = max(w["t_abad"], abs(tm[0]))
        w["t_hip"] = max(w["t_hip"], abs(tm[1]))
        w["t_knee"] = max(w["t_knee"], abs(tm[2]))
        w["belt"] = max(w["belt"], r["belt"])
    belt_per_kg = max(r["belt"] for r in rows if r["stance"]) / ROBOT_MASS
    w.update(mass_belt_cont=P.BELT_T_CONT / belt_per_kg, mass_belt_peak=P.BELT_T_PEAK / belt_per_kg,
             speed=STEP / (duty * T), lift=h, sink=z_sink, H=H, clearance=swing_clearance(h, H, z_sink) if snow else None)
    return rows, w


def register(g, w):
    """Direct register: hind foot (LH) touchdown vs the front foot (LF) print on the same side."""
    T, duty, ph = g["T"], g["duty"], g["phase"]
    v = w["speed"]
    prints = []
    for leg, hip_x in (("LF", HIP_SPACING / 2), ("LH", -HIP_SPACING / 2)):
        for c in range(-1, 6):
            t_td = ((-ph[leg]) % 1.0 + c) * T                  # stance starts at s = 0
            t_lo = t_td + duty * T
            X = v * t_td + hip_x + STEP / 2
            prints.append((leg, X, t_td, t_lo))
    front = [p for p in prints if p[0] == "LF"]
    best = None
    for leg, X, t_td, t_lo in prints:
        if leg != "LH" or t_td < 2 * T:
            continue
        f = min(front, key=lambda p: abs(p[1] - X))
        err = (X - f[1]) * 1000
        gap = t_td - f[3]
        if best is None or abs(err) < abs(best[0]):
            best = (err, gap)
    # fore-aft gap between the front and hind feet on the same side, over the cycle
    min_gap = 1e9
    for k in range(200):
        t = k / 200 * T
        xs = {}
        for leg, hip_x in (("LF", HIP_SPACING / 2), ("LH", -HIP_SPACING / 2)):
            s = (t / T + ph[leg]) % 1.0
            x, _, _ = foot_pose(s, duty, w["H"], w["lift"], True)
            xs[leg] = hip_x + x
        min_gap = min(min_gap, (xs["LF"] + SG.X_HEEL / 1000 - (xs["LH"] + SG.X_TOE / 1000)) * 1000)
    return best, min_gap


def verdicts(w):
    lim_ok = (w["q1"][0] >= P.Q1_LIM[0] and w["q1"][1] <= P.Q1_LIM[1] and w["q2"][0] >= P.Q2_LIM[0] and w["q2"][1] <= P.Q2_LIM[1])
    out = [("joint angles inside the limits (collision-free box)", lim_ok,
            f"q1 {w['q1'][0]:.0f}…{w['q1'][1]:.0f}°, q2 {w['q2'][0]:.0f}…{w['q2'][1]:.0f}° (limits {P.Q1_LIM}, {P.Q2_LIM})")]
    if w["alpha_st"][0] < 1e8:
        out.append(("stance ankle angle inside the stops", SG.ALPHA_MIN <= w["alpha_st"][0] and w["alpha_st"][1] <= SG.ALPHA_MAX,
                    f"α {w['alpha_st'][0]:.1f}…{w['alpha_st'][1]:.1f}° (stops {SG.ALPHA_MIN:.1f} / {SG.ALPHA_MAX:.1f}°)"))
    out += [("motor speeds ≤ rated 38.7 rad/s", max(w["w_hip"], w["w_knee"]) <= OMEGA_MAX, f"hip {w['w_hip']:.1f}, knee motor {w['w_knee']:.1f} rad/s"),
            ("motor torques ≤ rated 1.3 Nm", max(w["t_hip"], w["t_knee"], w["t_abad"]) <= P.TAU_RATED,
             f"ab/ad {w['t_abad']:.2f}, hip {w['t_hip']:.2f}, knee {w['t_knee']:.2f} Nm"),
            ("motor torques ≤ peak 4.1 Nm", max(w["t_hip"], w["t_knee"], w["t_abad"]) <= P.TAU_PEAK, ""),
            ("knee motor ≤ 0.76 Nm firmware limit", w["t_knee"] <= P.BELT_T_PEAK * R_HIP, f"{w['t_knee']:.2f} Nm"),
            ("belt ≤ 60 N continuous", w["belt"] <= P.BELT_T_CONT, f"{w['belt']:.0f} N"),
            ("belt ≤ 120 N peak", w["belt"] <= P.BELT_T_PEAK, f"{w['belt']:.0f} N")]
    return out


def main(anim=True):
    res = {}
    L = ["# Gait check — snow leg (analysis/gait_check.py)\n",
         f"Robot {ROBOT_MASS} kg (ASSUMED). Leg masses from the slicer + hardware list (analysis/mass_model.py): thigh "
         f"{M_THIGH*1000:.0f} g, shank + knee pulley {M_SHANK*1000:.0f} g, foot group {M_FOOT*1000:.0f} g. Stride {STEP*1000:.0f} mm. "
         "Stance loads = weight over the feet on the ground + 10 % lateral; swing = link gravity + inertia (foot mass at the pivot).\n"]
    for snow in (True, False):
        gs = gaits(snow)
        title = (f"SNOW MODE — pivot {stance_height(True)*1000:.0f} mm below the hip ({P.SNOW_STANCE_FRAC:.2f}·(L1+L2)), "
                 f"swing over the {SNOW_SET} surface" if snow else
                 f"Competition stance for reference — pivot {stance_height(False)*1000:.0f} mm below the hip, 40 mm swing, same robot")
        L.append(f"## {title}\n")
        for name, g in gs.items():
            rows, w = analyse(g, snow)
            res[("snow_" if snow else "normal_") + name] = (rows, w)
            hdr = f"### {name}: period {g['T']} s, duty {g['duty']}, speed {w['speed']:.2f} m/s"
            if snow:
                hdr += (f"; predicted sinkage {w['sink']*1000:.0f} mm ({SNOW_SET}, walk load), lift {w['lift']*1000:.0f} mm, "
                        f"lowest foot point {w['clearance']:.1f} mm above the snow surface (target ≥ {P.SNOW_SWING_MARGIN:.0f})")
            L.append(hdr + "\n")
            L.append("| check | result | value |\n|---|---|---|")
            for txt, ok, val in verdicts(w):
                L.append(f"| {txt} | {'PASS' if ok else '**FAIL**'} | {val} |")
            L.append(f"\nMax robot mass with the belt at its 60 N continuous rating: **{w['mass_belt_cont']:.1f} kg** "
                     f"({w['mass_belt_peak']:.1f} kg at the 120 N peak).\n")
            if snow and name == "walk":
                (err, gap), mg = register(g, w)
                L.append(f"Direct register: hind foot lands {err:+.1f} mm from the front foot's print, {gap:.2f} s after the front "
                         f"foot left it; closest front-heel to hind-toe gap on one side over the cycle {mg:.0f} mm "
                         f"({'no clash' if mg > 0 else 'CLASH'}). (Stride / duty = {STEP/g['duty']*1000:.0f} mm = hip spacing "
                         f"{HIP_SPACING*1000:.0f} mm / 2.)\n")
                res["register"] = dict(err_mm=err, gap_s=gap, min_gap_mm=mg)
    with open(os.path.join(OUT, "gait.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    summary = {k: v[1] for k, v in res.items() if isinstance(v, tuple)}
    summary["register"] = res.get("register")
    json.dump(summary, open(os.path.join(OUT, "gait.json"), "w"), indent=1, default=float)
    print("\n".join(L))
    if anim:
        animate(res["snow_walk"][0], res["snow_walk"][1], os.path.join(REND, "snow_walk_cycle.gif"))
    plot_loads(res)
    return res


def plot_loads(res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.8), dpi=110)
    for ax, key, title in ((axs[0], "snow_walk", "Snow walk"), (axs[1], "snow_trot", "Snow-stance trot (reference)")):
        rows, w = res[key]
        s = [r["s"] for r in rows]
        ax.plot(s, [abs(r["tm"][1]) for r in rows], label="hip motor Nm", color="#2f6fdf")
        ax.plot(s, [abs(r["tm"][2]) for r in rows], label="knee motor Nm", color="#d1242f")
        ax.plot(s, [r["belt"] / 100 for r in rows], label="belt N / 100", color="#1a7f37")
        ax.axhline(P.TAU_RATED, color="#2f6fdf", ls=":", lw=1)
        ax.axhline(P.BELT_T_CONT / 100, color="#1a7f37", ls=":", lw=1)
        ax.set_title(f"{title}: {w['speed']:.2f} m/s", fontsize=10)
        ax.set_xlabel("cycle phase (stance first)")
        ax.legend(fontsize=7)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "snow_gait_loads.png"))
    plt.close(fig)


# ------------------------------------------------------------------ animation (2D side view with the snowshoe and snow)
def stadium_pts(c0, r0, c1, r1, n=24):
    (x0, y0), (x1, y1) = c0, c1
    d = math.hypot(x1 - x0, y1 - y0)
    ang = math.atan2(y1 - y0, x1 - x0)
    a = math.acos((r0 - r1) / d)
    pts = [(x1 + r1 * math.cos(t), y1 + r1 * math.sin(t)) for t in np.linspace(ang - a, ang + a, n)]
    pts += [(x0 + r0 * math.cos(t), y0 + r0 * math.sin(t)) for t in np.linspace(ang + a, ang - a + 2 * math.pi, n)]
    return np.array(pts)


def foot_side(pivot, psi_deg):
    """Side outline of the snowshoe (deck + lugs) at a pivot with pitch psi (toe up +)."""
    c, s = math.cos(math.radians(psi_deg)), math.sin(math.radians(psi_deg))
    pts = [(SG.X_HEEL, SG.Y_DECK_TOP), (SG.X_TOE, SG.Y_DECK_TOP), (SG.X_TOE, SG.Y_LUG_TIP + P.TOE_BEVEL_RUN * 0.7),
           (SG.X_TOE - P.TOE_BEVEL_RUN, SG.Y_LUG_TIP), (SG.X_HEEL, SG.Y_LUG_TIP)]
    return np.array([(pivot[0] + x * c - y * s, pivot[1] + x * s + y * c) for x, y in pts])


def animate(rows, w, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle, Polygon, FancyBboxPatch, Rectangle

    g = gaits(True)["walk"]
    T, duty, phase = g["T"], g["duty"], g["phase"]
    n = len(rows)
    v = w["speed"] * 1000
    H = w["H"] * 1000
    sole = -H - SG.ANKLE_H
    surface = sole + w["sink"] * 1000
    fpc, cycles = 40, 2
    hips = {"LF": HIP_SPACING * 500, "RF": HIP_SPACING * 500, "LH": -HIP_SPACING * 500, "RH": -HIP_SPACING * 500}
    ink, mute, snowc, near, far = "#1f2328", "#8b949e", "#eef3fa", "#2f6fdf", "#9db8ea"
    fig = plt.figure(figsize=(11, 5.4), dpi=90)
    ax = fig.add_axes([0.02, 0.05, 0.96, 0.9])

    def draw(fi):
        t = (fi % (fpc * cycles)) / fpc * T
        bx = v * t
        ax.clear()
        ax.set_xlim(bx - 380, bx + 380)
        ax.set_ylim(sole - 30, 140)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.add_patch(Rectangle((bx - 400, sole - 30), 800, surface - sole + 30, fc=snowc, ec="none"))
        ax.plot([bx - 400, bx + 400], [surface, surface], color="#9aa9bd", lw=1)
        for leg in ("RF", "RH", "LF", "LH"):
            s = ((t / T) + phase[leg]) % 1.0
            r = rows[int(s * n) % n]
            q1, q2 = r["q"][1], r["q"][2]
            hip = (bx + hips[leg], 0.0)
            kx, ky = hip[0] + P.L1 * math.sin(q1), hip[1] - P.L1 * math.cos(q1)
            px, py = kx + P.L2 * math.sin(q1 + q2), ky - P.L2 * math.cos(q1 + q2)
            col = far if leg[0] == "R" else near
            psi = r["phi"] - r["alpha"]
            ax.add_patch(Polygon(foot_side((px, py), psi), closed=True, fc=col, ec=ink, lw=0.6))
            ax.add_patch(Polygon(stadium_pts((kx, ky), P.SHANK_W_KNEE / 2, (px, py), P.SHANK_TIP_R), closed=True, fc=col, ec=ink, lw=0.6))
            ax.add_patch(Polygon(stadium_pts(hip, P.THIGH_R_HIP, (kx, ky), P.THIGH_R_KNEE), closed=True, fc=col, ec=ink, lw=0.6))
            ax.add_patch(Circle(hip, P.MOTOR_D / 2 + 10, fc="#57606a" if leg[0] == "L" else "#adb5bd", ec=ink, lw=0.6))
        ax.add_patch(FancyBboxPatch((bx - HIP_SPACING * 500 - 50, 30), HIP_SPACING * 1000 + 100, 70,
                                    boxstyle="round,pad=2,rounding_size=14", fc="#e6ecf5", ec=ink, lw=0.8))
        ax.text(bx - 360, 125, f"Snow leg — 4.5 kg robot, snow walk {v/1000:.2f} m/s, snowshoe feet, lift {w['lift']*1000:.0f} mm",
                fontsize=10, color=ink, weight="bold")
        ax.text(bx - 360, 110, f"t = {t:4.2f} s   shaded = predicted sinkage {w['sink']*1000:.0f} mm ({SNOW_SET}); sole stays level in stance",
                fontsize=8, color=mute)

    anim = FuncAnimation(fig, draw, frames=fpc * cycles, interval=1000 * T / fpc)
    anim.save(path, writer=PillowWriter(fps=fpc / T))
    draw(int(fpc * 0.3))
    fig.savefig(path.replace(".gif", "_frame.png"))
    plt.close(fig)


if __name__ == "__main__":
    main(anim="--no-anim" not in sys.argv)
