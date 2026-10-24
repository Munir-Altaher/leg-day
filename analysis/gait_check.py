"""Can it walk like a dog? Gait feasibility + walking animation.

Drives the real leg model (L1, L2, N, limits from params.py) through a foot
trajectory (straight stance stroke on the ground, raised swing arc), solves IK
every 1/120 of a cycle and checks: joint limits (the clearance check showed the
whole limit box is collision-free), motor speeds, motor torques, belt tension.
Two gaits for a quadruped built from four copies of this leg:
  walk : four-beat lateral sequence (LH, LF, RH, RF), duty 0.75 -> 3 feet down
  trot : diagonal pairs, duty 0.5 -> 2 feet down
Writes analysis/out/gait.md and docs/renders/walk_cycle.gif.
Run:  python analysis/gait_check.py
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
REND = os.path.join(ROOT, "docs", "renders")
G = 9.81
L1, l = P.L1 / 1000, P.L_ABAD / 1000
L2 = P.L2_ANKLE / 1000                  # snowshoe foot: the IK point is the ankle pivot (ground force acts through it)
OFF = math.radians(P.ANKLE_OFF_DEG)     # knee->pivot line vs the shank's knee->FOOT line
PHI_LIFT = 16.0                         # shank angle at lift-off (set from the gait in animate())
N = P.N_RATIO
R_HIP, R_KNEE = P.PD_HIP / 2000, P.PD_KNEE / 2000
GROUND = P.STANCE_FRAC * (P.L1 + P.L2) / 1000 + P.FOOT_D / 2000    # hip height above the ground, 224 mm (as with the ball)
H = GROUND - P.SHOE_PIVOT_UP / 1000                                 # ankle pivot height below the hip
STEP = 0.120                            # stride length (brief step box +/- 60 mm)
CLEAR = 0.040                           # swing clearance
LEG_MASS = 0.87                         # one complete leg incl. 3 motors + bracket (CAD estimate)
BODY_MASS = 0.80                        # ASSUMED body: battery, CAN board, frame
ROBOT_MASS = 4 * LEG_MASS + BODY_MASS
HIP_SPACING = 0.32                      # fore-aft hip-to-hip distance for the drawing (ASSUMED)
M_THIGH, M_LOWER = 0.075, 0.045         # swinging link masses (thigh; shank + foot + pulley), kg
OMEGA_MAX = P.SPEED_RATED * 2 * math.pi / 60

GAITS = {
    "walk": dict(T=1.0, duty=0.75, phase={"LH": 0.0, "LF": 0.25, "RH": 0.5, "RF": 0.75}),
    "trot": dict(T=0.5, duty=0.5, phase={"LF": 0.0, "RH": 0.0, "RF": 0.5, "LH": 0.5}),
}


def foot(s, duty):
    """Foot position relative to the hip (x fwd, y up) at cycle phase s in [0, 1)."""
    if s < duty:                                    # stance: foot fixed on ground, body moves forward
        u = s / duty
        return STEP / 2 - STEP * u, -H, True
    u = (s - duty) / (1 - duty)                     # swing: cycloid forward, sine lift
    x = -STEP / 2 + STEP * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
    return x, -H + CLEAR * math.sin(math.pi * u), False


def analyse(name, g, n=240):
    T, duty = g["T"], g["duty"]
    dt = T / n
    rows = []
    for k in range(n):
        s = k / n
        x, y, st = foot(s, duty)
        q = ik(np.array([x, y, l]), L1, L2, l)              # angles of the straight hip-knee-pivot chain
        # feet on the ground at this instant (for load sharing)
        n_down = sum(1 for ph in g["phase"].values() if ((s + ph) % 1.0) < duty)
        if st:
            F = np.array([0.0, -ROBOT_MASS * G / n_down, 0.0])
            F[2] = 0.1 * abs(F[1])                  # +10 % lateral margin, as in the brief
            tq = joint_torques(q, F, L1, L2, l)
        else:
            # swing: gravity of the links only (inertia added below from accelerations)
            th = q[1]
            kx = L1 * math.sin(th)
            tq = np.array([0.0,
                           -(M_THIGH * G * L1 / 2 * math.sin(th) + M_LOWER * G * (kx + L2 / 2 * math.sin(th + q[2]))),
                           -(M_LOWER * G * L2 / 2 * math.sin(th + q[2]))])
        qj = q.copy()
        qj[2] -= OFF                                        # real knee angle (the pivot sits OFF off the shank line)
        rows.append(dict(s=s, x=x, y=y, stance=st, q=qj, tq=tq, n_down=n_down))
    qs = np.array([r["q"] for r in rows])
    qd = (np.roll(qs, -1, 0) - np.roll(qs, 1, 0)) / (2 * dt)
    qdd = (np.roll(qs, -1, 0) - 2 * qs + np.roll(qs, 1, 0)) / dt ** 2
    I1 = M_THIGH * (L1 / 2) ** 2 + M_LOWER * (L1 + L2 / 2) ** 2      # crude, extended
    I2 = M_LOWER * (L2 / 2) ** 2
    worst = dict(q1=(1e9, -1e9), q2=(1e9, -1e9), w_hip=0, w_knee=0, t_hip=0, t_knee=0, t_abad=0, belt=0)
    for i, r in enumerate(rows):
        tq = r["tq"].copy()
        if not r["stance"]:
            tq[1] += I1 * qdd[i, 1]
            tq[2] += I2 * qdd[i, 2]
        tm = motor_torques(tq, N)
        r["tm"] = tm
        r["belt"] = abs(tq[2]) / R_KNEE
        q1, q2 = math.degrees(r["q"][1]), math.degrees(r["q"][2])
        worst["q1"] = (min(worst["q1"][0], q1), max(worst["q1"][1], q1))
        worst["q2"] = (min(worst["q2"][0], q2), max(worst["q2"][1], q2))
        worst["w_hip"] = max(worst["w_hip"], abs(qd[i, 1]))
        worst["w_knee"] = max(worst["w_knee"], abs(qd[i, 1] + N * qd[i, 2]))
        worst["t_abad"] = max(worst["t_abad"], abs(tm[0]))
        worst["t_hip"] = max(worst["t_hip"], abs(tm[1]))
        worst["t_knee"] = max(worst["t_knee"], abs(tm[2]))
        worst["belt"] = max(worst["belt"], r["belt"])
    # largest robot mass that keeps the belt within its continuous rating in this gait
    belt_per_kg = max(r["belt"] for r in rows if r["stance"]) / ROBOT_MASS
    worst["mass_belt_cont"] = P.BELT_T_CONT / belt_per_kg
    worst["mass_belt_peak"] = P.BELT_T_PEAK / belt_per_kg
    worst["speed"] = STEP / (duty * T)
    return rows, worst


def verdicts(w):
    lim_ok = (w["q1"][0] >= P.Q1_LIM[0] and w["q1"][1] <= P.Q1_LIM[1] and
              w["q2"][0] >= P.Q2_LIM[0] and w["q2"][1] <= P.Q2_LIM[1])
    return [
        ("joint angles inside limits (collision-free box)", lim_ok,
         f"q1 {w['q1'][0]:.0f}…{w['q1'][1]:.0f}°, q2 {w['q2'][0]:.0f}…{w['q2'][1]:.0f}°"),
        ("motor speeds ≤ rated 38.7 rad/s", max(w["w_hip"], w["w_knee"]) <= OMEGA_MAX,
         f"hip {w['w_hip']:.1f}, knee motor {w['w_knee']:.1f} rad/s"),
        ("motor torques ≤ rated 1.3 Nm", max(w["t_hip"], w["t_knee"], w["t_abad"]) <= P.TAU_RATED,
         f"ab/ad {w['t_abad']:.2f}, hip {w['t_hip']:.2f}, knee {w['t_knee']:.2f} Nm"),
        ("knee motor ≤ 0.76 Nm firmware limit", w["t_knee"] <= P.BELT_T_PEAK * R_HIP, f"{w['t_knee']:.2f} Nm"),
        ("belt ≤ 60 N continuous", w["belt"] <= P.BELT_T_CONT, f"{w['belt']:.0f} N"),
        ("belt ≤ 120 N peak", w["belt"] <= P.BELT_T_PEAK, f"{w['belt']:.0f} N"),
    ]


def report(results):
    L = ["# Gait check: can a quadruped of these legs walk? (analysis/gait_check.py)\n",
         f"Robot assumed = 4 legs × {LEG_MASS} kg + {BODY_MASS} kg body = **{ROBOT_MASS:.2f} kg**. "
         f"Stride {STEP*1000:.0f} mm, swing clearance {CLEAR*1000:.0f} mm, hip {GROUND*1000:.0f} mm above the ground (ankle pivot {H*1000:.0f} mm below the hip, snowshoe foot), ab/ad held at 0. "
         "Stance loads = weight shared over the feet on the ground + 10 % lateral; swing = link gravity + inertia.\n"]
    for name, (rows, w) in results.items():
        g = GAITS[name]
        L.append(f"## {name}: period {g['T']} s, duty {g['duty']}, forward speed {w['speed']:.2f} m/s\n")
        L.append("| check | result | value |\n|---|---|---|")
        for txt, ok, val in verdicts(w):
            L.append(f"| {txt} | {'PASS' if ok else '**FAIL**'} | {val} |")
        L.append(f"\nMax robot mass for this gait: **{w['mass_belt_cont']:.1f} kg** with the belt at its 60 N continuous rating, "
                 f"{w['mass_belt_peak']:.1f} kg at the 120 N peak.\n")
    with open(os.path.join(OUT, "gait.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    print("\n".join(L))


# ------------------------------------------------------------------ animation
def _loop_pts(loop):
    """Sample a swlib sketch loop (circle / lines, arcs, splines) as an (n, 2) polygon."""
    if loop[0] == "circle":
        t = np.linspace(0, 2 * math.pi, 48, endpoint=False)
        return np.column_stack([loop[1] + loop[3] * np.cos(t), loop[2] + loop[3] * np.sin(t)])
    out = []
    for sg in loop[1]:
        if sg[0] == "line":
            out.append(sg[1])
        elif sg[0] == "spline":
            out += list(sg[1])[:-1]
        else:
            c, p0, p1, d = sg[1], sg[2], sg[3], sg[4]
            t0, t1 = math.atan2(p0[1] - c[1], p0[0] - c[0]), math.atan2(p1[1] - c[1], p1[0] - c[0])
            if d > 0 and t1 < t0:
                t1 += 2 * math.pi
            if d < 0 and t1 > t0:
                t1 -= 2 * math.pi
            r = math.dist(c, p0)
            out += [(c[0] + r * math.cos(t), c[1] + r * math.sin(t)) for t in np.linspace(t0, t1, 10)[:-1]]
    return np.array(out)


def _ccw(p, ccw=True):
    a = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])
    return p if (a > 0) == ccw else p[::-1]


_SHAPES = None


def cad_shapes():
    """Side-view outlines of the real parts at q = 0 (body frame, mm), from cad/build_parts.py:
    thigh with its cutouts, bent shank with its cutouts, foot plug, tilted domed foot."""
    global _SHAPES
    if _SHAPES is None:
        sys.path.insert(0, os.path.join(ROOT, "cad"))
        import build_parts as B
        thigh = [_ccw(_loop_pts(B.thigh_outline()))] + [_ccw(_loop_pts(h), False) for h in B.thigh_wall_holes()]
        shank = [_ccw(_loop_pts(B._band(0.0, B.SH_LEN, B.shank_hw)))] + [_ccw(_loop_pts(h), False) for h in B.shank_holes()]
        plug = _loop_pts(B.ax_rect(-B.PLUG_W / 2, B.FOOT_TOP_A - B.PLUG_DEPTH, B.PLUG_W / 2, B.SH_END_A - 3.0))
        import snowshoe_geo as SG
        cheek = SG.cheek_profile(SG.lips(B.FOOT_UP, B.FOOT_ACROSS, B.PLUG_W))
        _SHAPES = dict(thigh=thigh, shank=shank, plug=plug, shoe=SG.side_profile(), cheek=cheek,
                       ankle=np.array(B.ANKLE), knee=B.KNEE)
    return _SHAPES


def _rot(p, ang, c=(0.0, 0.0)):
    c0, s0 = math.cos(ang), math.sin(ang)
    x, y = p[:, 0] - c[0], p[:, 1] - c[1]
    return np.column_stack([c[0] + x * c0 - y * s0, c[1] + x * s0 + y * c0])


def shoe_alpha(q1, q2, stance):
    """Shank angle in the snowshoe's frame (deg): flat on the ground in stance; in swing the friction hinge
    keeps the lift-off angle. Clamped to the lips."""
    a = math.degrees(q1 + q2) if stance else PHI_LIFT
    return min(max(a, P.ANKLE_LIM[0]), P.ANKLE_LIM[1])


def leg_shapes(hip, q1, q2, alpha=None):
    """Side-view outlines (mm) of the CAD thigh, shank (with cutouts), plug and snowshoe at a hip position:
    thigh rotated q1 about the hip, shank + shoe rotated q2 about the knee then q1; the shoe sits at
    `alpha` (shank angle in the pad frame; default: flat on the ground)."""
    S = cad_shapes()
    if alpha is None:
        alpha = shoe_alpha(q1, q2, True)
    hx, hy = hip
    mv = lambda p, lower: _rot(_rot(p, q2, S["knee"]) if lower else p, q1) + (hx, hy)   # noqa: E731
    kx, ky = hx + P.L1 * math.sin(q1), hy - P.L1 * math.cos(q1)
    pad = lambda p: _rot(p, -math.radians(alpha)) + S["ankle"]                          # noqa: E731
    ft = mv(S["ankle"][None], True)[0]
    return dict(thigh=[mv(p, False) for p in S["thigh"]], shank=[mv(p, True) for p in S["shank"]],
                plug=mv(S["plug"], True), shoe=mv(pad(S["shoe"]), True), cheek=mv(pad(S["cheek"]), True),
                knee=(kx, ky), ft=tuple(ft))


def _with_holes(loops, **kw):
    """PathPatch of an outline with see-through holes (holes wound the other way)."""
    from matplotlib.path import Path
    from matplotlib.patches import PathPatch
    verts, codes = [], []
    for p in loops:
        verts += list(map(tuple, p)) + [tuple(p[0])]
        codes += [Path.MOVETO] + [Path.LINETO] * (len(p) - 1) + [Path.CLOSEPOLY]
    return PathPatch(Path(verts, codes), **kw)


def draw_leg(ax, hip, q1, q2, col, ink, hub, alpha=None):
    from matplotlib.patches import Circle, Polygon
    L = leg_shapes(hip, q1, q2, alpha)
    ax.add_patch(Polygon(L["shoe"], closed=True, fc="#e07b39", ec=ink, lw=0.6))
    ax.add_patch(Polygon(L["plug"], closed=True, fc=col, ec=ink, lw=0.6))
    ax.add_patch(_with_holes(L["shank"], fc=col, ec=ink, lw=0.6))
    ax.add_patch(Polygon(L["cheek"], closed=True, fc="#c4703a", ec=ink, lw=0.6, alpha=0.85))
    ax.add_patch(Circle(L["ft"], 1.6, fc="white", ec=ink, lw=0.5))
    ax.add_patch(_with_holes(L["thigh"], fc=col, ec=ink, lw=0.6))
    ax.add_patch(Circle(L["knee"], 4.0, fc="white", ec=ink, lw=0.6))
    ax.add_patch(Circle(hip, P.MOTOR_D / 2, fc=hub, ec=ink, lw=0.6))
    return L


def animate(rows_walk, w_walk, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle, FancyBboxPatch

    g = GAITS["walk"]
    T, duty, phase = g["T"], g["duty"], g["phase"]
    n = len(rows_walk)
    v = w_walk["speed"] * 1000                       # mm/s
    frames_per_cycle = 40
    cycles = 2
    hips = {"LF": HIP_SPACING * 500, "RF": HIP_SPACING * 500, "LH": -HIP_SPACING * 500, "RH": -HIP_SPACING * 500}
    Hmm = GROUND * 1000                               # ground line (sole level)
    global PHI_LIFT
    i_lo = next(i for i in range(n) if not rows_walk[i]["stance"] and rows_walk[i - 1]["stance"])
    PHI_LIFT = math.degrees(rows_walk[i_lo - 1]["q"][1] + rows_walk[i_lo - 1]["q"][2])

    ink, mute, grid = "#1f2328", "#8b949e", "#d0d7de"
    near, far = "#2f6fdf", "#9db8ea"
    fig = plt.figure(figsize=(11, 5.2), dpi=90)
    axq = fig.add_axes([0.03, 0.08, 0.60, 0.84])
    axl = fig.add_axes([0.66, 0.30, 0.32, 0.62])
    axb = fig.add_axes([0.70, 0.07, 0.27, 0.17])

    def draw(fi):
        t = (fi % (frames_per_cycle * cycles)) / frames_per_cycle * T
        bx = v * t
        for ax in (axq, axl, axb):
            ax.clear()
        # ---------- quadruped side view
        axq.set_xlim(bx - 360, bx + 360)
        axq.set_ylim(-Hmm - 25, 175)
        axq.set_aspect("equal")
        axq.axis("off")
        axq.plot([bx - 400, bx + 400], [-Hmm, -Hmm], color=ink, lw=1.2)
        for gx in np.arange(math.floor((bx - 400) / 50) * 50, bx + 400, 50):
            axq.plot([gx, gx - 12], [-Hmm, -Hmm - 12], color=grid, lw=1)
        order = ["RF", "RH", "LF", "LH"]                 # far side first
        for leg in order:
            s = ((t / T) + phase[leg]) % 1.0
            r = rows_walk[int(s * n) % n]
            q1, q2 = r["q"][1], r["q"][2]
            hip = (bx + hips[leg], 0.0)
            draw_leg(axq, hip, q1, q2, far if leg[0] == "R" else near, ink, "#57606a" if leg[0] == "L" else "#adb5bd",
                     shoe_alpha(q1, q2, r["stance"]))
        axq.add_patch(FancyBboxPatch((bx - HIP_SPACING * 500 - 45, 20), HIP_SPACING * 1000 + 90, 70,
                                     boxstyle="round,pad=2,rounding_size=14", fc="#e6ecf5", ec=ink, lw=0.8))
        axq.text(bx - 340, 158, f"Quadruped of 4 × this leg — walk (lateral sequence), {v/1000:.2f} m/s",
                 fontsize=10, color=ink, weight="bold")
        axq.text(bx - 340, 136, f"t = {t:4.2f} s   robot {ROBOT_MASS:.1f} kg (assumed)   dark = left legs, light = right legs",
                 fontsize=8, color=mute)
        # ---------- single leg detail (left front) with foot path
        s = ((t / T) + phase["LF"]) % 1.0
        k = int(s * n) % n
        r = rows_walk[k]
        q1, q2 = r["q"][1], r["q"][2]
        path_xy = np.array([[rr["x"] * 1000, rr["y"] * 1000] for rr in rows_walk])    # ankle-pivot path
        axl.plot(path_xy[:, 0], path_xy[:, 1], color=grid, lw=1.5)
        Ld = draw_leg(axl, (0, 0), q1, q2, near, ink, "#57606a", shoe_alpha(q1, q2, r["stance"]))
        axl.add_patch(Circle(Ld["ft"], 2.0, fc="#d1242f"))           # foot centre (the IK target)
        axl.set_xlim(-150, 150)
        axl.set_ylim(-Hmm - 30, 45)
        axl.set_aspect("equal")
        axl.axis("off")
        axl.set_title("Left-front leg (hip frame) and foot path", fontsize=9, color=ink)
        axl.text(-148, -Hmm - 28, f"q1 {math.degrees(q1):5.1f}°  q2 {math.degrees(q2):5.1f}°  "
                 f"{'STANCE' if r['stance'] else 'SWING'}", fontsize=8.5, color=ink, family="monospace")
        # ---------- live load bars
        vals = [abs(r["tm"][1]) / P.TAU_RATED, abs(r["tm"][2]) / (P.BELT_T_PEAK * R_HIP), r["belt"] / P.BELT_T_CONT]
        labels = ["hip motor / rated", "knee motor / 0.76 Nm", "belt / 60 N"]
        colors = ["#d1242f" if x > 1 else near for x in vals]
        axb.barh(range(3), [min(x, 1.6) for x in vals], color=colors, height=0.6)
        axb.axvline(1.0, color=ink, lw=1, ls="--")
        axb.set_yticks(range(3), labels, fontsize=7.5)
        axb.set_xlim(0, 1.6)
        axb.tick_params(axis="x", labelsize=7)
        axb.invert_yaxis()
        for sp in ("top", "right"):
            axb.spines[sp].set_visible(False)

    anim = FuncAnimation(fig, draw, frames=frames_per_cycle * cycles, interval=1000 * T / frames_per_cycle)
    anim.save(path, writer=PillowWriter(fps=frames_per_cycle / T))
    draw(int(frames_per_cycle * 0.3))
    fig.savefig(path.replace(".gif", "_frame.png"))
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(REND, exist_ok=True)
    res = {name: analyse(name, g) for name, g in GAITS.items()}
    report(res)
    animate(res["walk"][0], res["walk"][1], os.path.join(REND, "walk_cycle.gif"))
    print("wrote", os.path.join(REND, "walk_cycle.gif"))
