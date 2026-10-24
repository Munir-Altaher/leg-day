"""Export the leg (free-standing on the snowshoe) to URDF for Isaac Sim / Isaac Lab.

Five rigid bodies, built from the SolidWorks STL exports in cad/stl/ (body frame, +300 mm SHIFT):
  base   body brackets + spine + rig plate + ab/ad motor            (floating root)
  abad   hip module: plates, flanges, C-frame, hip + knee motors, hip shaft adapter, 20T pulley
  thigh  thigh_outer + thigh_cover (+ 3 x 625 bearings, belt)
  shank  shank + 60T knee pulley
  shoe   snowshoe                                                    (passive friction hinge, lip stops)
Joints: abad (X), hip (Z), knee (Z), ankle (Z, passive). Angles follow params.py: q1 / q2 positive CCW
about +Z (body frame X fwd, Y up, Z outboard). Body frame is Y-up: rotate the root +90 deg about X in the
simulator (see leg_snowshoe_cfg.py).

Masses: printed parts = PrusaSlicer grams (analysis/out/print.json, with infill), spread uniformly over the
part's mesh; motors 185 g each (solid cylinders); a few hardware masses added (listed in HARDWARE).
Inertias: exact polyhedral integrals of each mesh (signed tetrahedra), scaled to the part mass.
Limits: hip / ab-ad = AK40-10 peak 4.1 Nm; knee = belt peak 120 N x knee pulley radius (the belt, not the
motor, limits it); speeds = motor rated speed (knee / 3 through the belt). The knee motor is at the hip:
motor angle = q1 + 3 q2, so its torque also loads the hip joint (see README.md).
Counterweight (competition-day fix, no new printed parts): the leg's centre of mass sits only 2 mm inside the
snowshoe's inboard edge, so a weight is bolted to the outboard face of the hip module's C-frame (abad link,
z = COUNTERWEIGHT_Z). Default COUNTERWEIGHT_G; override with  --counterweight GRAMS  (0 = none).
Run:  python sim/export_urdf.py [--counterweight 150]   -> sim/leg_snowshoe.urdf, sim/meshes/*.stl, sim/mass_report.md
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
import build_parts as B  # noqa: E402

STL = os.path.join(ROOT, "cad", "stl")
MESH = os.path.join(HERE, "meshes")
SHIFT = 300.0
COUNTERWEIGHT_G = 350.0            # g, on the C-frame's outboard face -> CoM ~22 mm inside the shoe's inboard edge (MuJoCo: stands 10/10; 150 g only 2/10)
COUNTERWEIGHT_Z = 95.0             # mm, body frame (C-frame outer face at 89.5 + half a 10 mm thick weight)

LINKS = {
    "base": ["body_bracket_back", "body_bracket_front", "body_spine", "rig_plate", "ref_motor_abad"],
    "abad": ["module_plate_inboard", "module_flange_back", "module_flange_front", "module_frame_outboard",
             "ref_motor_hip", "ref_motor_knee", "hip_shaft_adapter", "ref_pulley_20T_kit"],
    "thigh": ["thigh_outer", "thigh_cover"],
    "shank": ["shank", f"knee_pulley_{P.KNEE_TEETH}T"],
    "shoe": ["snowshoe"],
}
ORIGIN = {"base": (0.0, 0.0), "abad": (0.0, 0.0), "thigh": (0.0, 0.0), "shank": B.KNEE, "shoe": B.ANKLE}   # joint points (x, y) mm
MOTOR_G = P.MOTOR_MASS * 1000
FIXED_G = {"ref_motor_abad": MOTOR_G, "ref_motor_hip": MOTOR_G, "ref_motor_knee": MOTOR_G, "ref_pulley_20T_kit": 8.0}
HARDWARE = {   # grams added per link, spread like the link's printed mass (ESTIMATES)
    "base": ("625 bearing (front wall) + 16 M3 screws/washers + 4 inserts", 4.6 + 16 * 1.2 + 4 * 0.4),
    "abad": ("M5 hip shaft + 13 M2.5 screws + 4 inserts + M2.5 x 12 x 5", 5.0 + 13 * 0.6 + 4 * 0.4 + 5 * 0.8),
    "thigh": ("3 x 625 bearings + GT2 belt + idler + 7 M3 screws", 3 * 4.6 + 3.0 + 4.0 + 7 * 1.0),
    "shank": ("M5 x 25 knee axle + nylock + washers", 6.5),
    "shoe": ("M3 x 25 ankle bolt + nylock + wave washer", 2.5),
}


def read_stl(name):
    with open(os.path.join(STL, name + ".STL"), "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [ln.split()[1:4] for ln in data.decode("ascii", "ignore").splitlines() if ln.strip().startswith("vertex")]
        tri = np.array(v, float).reshape(-1, 3, 3)
    else:
        n = struct.unpack("<I", data[80:84])[0]
        tri = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"].reshape(-1, 3, 3).astype(float)
    if np.abs(tri).max() < 5:
        tri = tri * 1000.0
    return tri - SHIFT                                    # body frame, mm


def write_stl(path, tri_m):
    with open(path, "wb") as f:
        f.write(b"leg_day sim mesh (m)".ljust(80, b" "))
        f.write(struct.pack("<I", len(tri_m)))
        nrm = np.cross(tri_m[:, 1] - tri_m[:, 0], tri_m[:, 2] - tri_m[:, 0])
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1)[:, None], 1e-15)
        rec = np.zeros(len(tri_m), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        rec["n"] = nrm
        rec["v"] = tri_m.reshape(-1, 9)
        f.write(rec.tobytes())


def mass_props(tri):
    """Volume (mm^3), centroid (mm) and inertia tensor about the centroid per unit density (mm^5) of a closed mesh."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))         # 6 x signed tetra volume (apex at origin)
    V = v6.sum() / 6
    com = ((a + b + c) * v6[:, None]).sum(0) / 24 / V
    # second moments  int x_i x_j dV  over tetrahedra (origin, a, b, c)
    S = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            S[i, j] = (v6 / 120 * (2 * (a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j]) +
                                   a[:, i] * b[:, j] + b[:, i] * a[:, j] + a[:, i] * c[:, j] + c[:, i] * a[:, j] +
                                   b[:, i] * c[:, j] + c[:, i] * b[:, j])).sum()
    S -= V * np.outer(com, com)                          # about the centroid
    I = np.trace(S) * np.eye(3) - S
    return V, com, I


CW_G = COUNTERWEIGHT_G


def main():
    global CW_G
    if "--counterweight" in sys.argv:
        CW_G = float(sys.argv[sys.argv.index("--counterweight") + 1])
    os.makedirs(MESH, exist_ok=True)
    grams = {k: v["grams"] for k, v in json.load(open(os.path.join(ROOT, "analysis", "out", "print.json"))).items()}
    links = {}
    report = ["# Sim mass report (sim/export_urdf.py)\n", "| link | parts | mass g | COM (link frame) mm |", "|---|---|---|---|"]
    for ln, parts in LINKS.items():
        ox, oy = ORIGIN[ln]
        m_tot, mc, I_tot, tris = 0.0, np.zeros(3), np.zeros((3, 3)), []
        bodies = []
        for pn in parts:
            tri = read_stl(pn) - (ox, oy, 0.0)
            tris.append(tri)
            V, com, Iu = mass_props(tri)
            m = FIXED_G.get(pn, grams.get(pn))
            if m is None:
                raise KeyError(pn)
            bodies.append((m, com, Iu / V * m))           # g, mm, g mm^2
        hw_name, hw_g = HARDWARE[ln]
        m_print = sum(m for m, _, _ in bodies)
        bodies = [(m * (1 + hw_g / m_print), c, I * (1 + hw_g / m_print)) for m, c, I in bodies]
        if ln == "abad" and CW_G > 0:
            # steel block ~ 30 x 30 x 10 mm (z thin) centred on the hip axis, outboard of the C-frame
            bx, by, bz = 30.0, 30.0, 10.0
            Icw = CW_G / 12 * np.diag([by ** 2 + bz ** 2, bx ** 2 + bz ** 2, bx ** 2 + by ** 2])
            bodies.append((CW_G, np.array([0.0, 0.0, COUNTERWEIGHT_Z]), Icw))
            hw_name += f" + {CW_G:.0f} g counterweight at z {COUNTERWEIGHT_Z:.0f}"
        m_tot = sum(m for m, _, _ in bodies)
        mc = sum(m * c for m, c, _ in bodies) / m_tot
        for m, c, I in bodies:
            d = c - mc
            I_tot += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        allt = np.vstack(tris) / 1000.0
        write_stl(os.path.join(MESH, f"{ln}.stl"), allt)
        links[ln] = dict(m=m_tot / 1000, com=mc / 1000, I=I_tot * 1e-9)     # kg, m, kg m^2
        report.append(f"| {ln} | {', '.join(parts)} (+ {hw_name}) | {m_tot:.1f} | ({mc[0]:.1f}, {mc[1]:.1f}, {mc[2]:.1f}) |")
    tot = sum(v["m"] for v in links.values())
    report.append(f"\n**Total {tot * 1000:.0f} g** (moving above the base: {(tot - links['base']['m']) * 1000:.0f} g)")

    # ---------------- limits
    w_motor = P.SPEED_RATED * 2 * math.pi / 60                 # rad/s at the output
    r_knee = P.PD_KNEE / 2000
    knee_eff = min(P.N_RATIO * P.TAU_PEAK, P.BELT_T_PEAK * r_knee)
    J = {
        "abad": dict(parent="base", child="abad", xyz=(0, 0, 0), axis=(1, 0, 0), lim=P.Q0_LIM, eff=P.TAU_PEAK, vel=w_motor),
        "hip": dict(parent="abad", child="thigh", xyz=(0, 0, 0), axis=(0, 0, 1), lim=P.Q1_LIM, eff=P.TAU_PEAK, vel=w_motor),
        "knee": dict(parent="thigh", child="shank", xyz=(B.KNEE[0] / 1000, B.KNEE[1] / 1000, 0), axis=(0, 0, 1), lim=P.Q2_LIM,
                     eff=knee_eff, vel=w_motor / P.N_RATIO),
        # ankle angle = rotation of the shoe relative to the shank = -alpha (alpha = shank angle in the pad frame)
        "ankle": dict(parent="shank", child="shoe", xyz=((B.ANKLE[0] - B.KNEE[0]) / 1000, (B.ANKLE[1] - B.KNEE[1]) / 1000, 0),
                      axis=(0, 0, 1), lim=(-P.ANKLE_LIM[1], -P.ANKLE_LIM[0]), eff=0.0, vel=50.0),
    }
    f3 = lambda v: " ".join(f"{x:.6g}" for x in v)          # noqa: E731
    X = ['<?xml version="1.0"?>', '<!-- generated by sim/export_urdf.py from the leg_day CAD; body frame X fwd, Y up, Z outboard -->',
         '<robot name="leg_day_snowshoe">']
    for ln, L in links.items():
        I = L["I"]
        X += [f'  <link name="{ln}">',
              '    <inertial>', f'      <origin xyz="{f3(L["com"])}" rpy="0 0 0"/>', f'      <mass value="{L["m"]:.5f}"/>',
              f'      <inertia ixx="{I[0,0]:.4e}" ixy="{I[0,1]:.4e}" ixz="{I[0,2]:.4e}" iyy="{I[1,1]:.4e}" iyz="{I[1,2]:.4e}" izz="{I[2,2]:.4e}"/>',
              '    </inertial>',
              f'    <visual><geometry><mesh filename="meshes/{ln}.stl"/></geometry></visual>',
              f'    <collision><geometry><mesh filename="meshes/{ln}.stl"/></geometry></collision>',
              '  </link>']
    for jn, j in J.items():
        lo, hi = (math.radians(v) for v in j["lim"])
        X += [f'  <joint name="{jn}" type="revolute">', f'    <parent link="{j["parent"]}"/>', f'    <child link="{j["child"]}"/>',
              f'    <origin xyz="{f3(j["xyz"])}" rpy="0 0 0"/>', f'    <axis xyz="{f3(j["axis"])}"/>',
              f'    <limit lower="{lo:.5f}" upper="{hi:.5f}" effort="{j["eff"]:.3f}" velocity="{j["vel"]:.2f}"/>']
        if jn == "ankle":
            X += ['    <dynamics damping="0.002" friction="0.09"/>   <!-- friction hinge: M3 nylock on a wave washer, ~90 mN m -->']
        X += ['  </joint>']
    X += ['</robot>']
    open(os.path.join(HERE, "leg_snowshoe.urdf"), "w", encoding="utf-8").write("\n".join(X) + "\n")

    report += ["\n## Joints", "| joint | axis | limits deg | effort N m | velocity rad/s |", "|---|---|---|---|---|"]
    for jn, j in J.items():
        report.append(f"| {jn} | {j['axis']} | {j['lim'][0]:.1f} … {j['lim'][1]:.1f} | {j['eff']:.2f} | {j['vel']:.1f} |")
    report.append(f"\nKnee effort = min(3 x {P.TAU_PEAK} N m motor, belt {P.BELT_T_PEAK:.0f} N x r {r_knee * 1000:.1f} mm) = {knee_eff:.2f} N m "
                  f"(continuous: {P.BELT_T_CONT * r_knee:.2f} N m).")
    report.append(f"\nAnkle pivot is {P.SHOE_PIVOT_UP:.0f} mm above the sole; knee -> ankle pivot {P.L2_ANKLE:.1f} mm.")
    # sideways balance in the standing pose (q1 -24, q2 64, shoe flat): only the base/abad/thigh/shank/shoe z matter
    z_com = sum(v["m"] * v["com"][2] for v in links.values()) / tot * 1000
    inner = B.zc(0.0) - P.SHOE_R_TOE
    report.append(f"\nSideways balance: whole-leg CoM at z = {z_com:.1f} mm; snowshoe spans z {inner:.0f} … {B.zc(0.0) + P.SHOE_R_TOE:.0f} mm "
                  f"-> CoM {z_com - inner:.1f} mm inside the inboard edge (counterweight {CW_G:.0f} g).")
    open(os.path.join(HERE, "mass_report.md"), "w", encoding="utf-8").write("\n".join(report) + "\n")
    print("\n".join(report))
    print("wrote", os.path.join(HERE, "leg_snowshoe.urdf"))


if __name__ == "__main__":
    main()
