"""Build a MuJoCo scene from sim/leg_snowshoe.urdf: floating leg on its snowshoe on a floor.

Runs on any PC (no NVIDIA GPU needed). Writes sim/leg_snowshoe_scene.xml (open it in the MuJoCo viewer) and,
with --check, holds the standing pose for 3 s with joint PD control and reports whether the leg stays up.
Run (from leg_day/):  sim/.venv/Scripts/python sim/build_mujoco.py --check
View:                 sim/.venv/Scripts/python -m mujoco.viewer --mjcf=sim/leg_snowshoe_scene.xml
"""
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "cad"))
import params as P  # noqa: E402

import snowshoe_geo as SG  # noqa: E402

URDF = os.path.join(HERE, "leg_snowshoe.urdf")
OUT = os.path.join(HERE, "leg_snowshoe_scene.xml")
# Standing pose. The ankle is a free hinge (0.09 N m friction), so the leg can only stand still leaning on a lip:
# shank at the rear stop (52 deg in the pad frame, gravity holds it there), hip placed over the middle of the shoe.
ANKLE_STOP = math.radians(P.ANKLE_LIM[1])
Q1 = math.radians(-40.0)
Q2 = ANKLE_STOP - Q1                                         # shank angle q1 + q2 = the stop -> shoe flat
STAND = {"abad": 0.0, "hip": Q1, "knee": Q2, "ankle": -ANKLE_STOP}
HIP_H = 0.20                                                 # first guess; build() drops the sole onto the floor
EFFORT = {"abad": P.TAU_PEAK, "hip": P.TAU_PEAK, "knee": min(P.N_RATIO * P.TAU_PEAK, P.BELT_T_PEAK * P.PD_KNEE / 2000)}


def lowest(m, d, g):
    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
        return d.geom_xpos[g][2] - m.geom_size[g][0]
    mid = m.geom_dataid[g]
    V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    return ((d.geom_xmat[g].reshape(3, 3) @ V.T).T + d.geom_xpos[g])[:, 2].min()


def build():
    spec = mujoco.MjSpec.from_file(URDF)
    spec.option.timestep = 0.0005
    spec.compiler.degree = False
    spec.compiler.discardvisual = False          # keep look-only geoms (the snowshoe mesh; its contact is the sole spheres)
    wb = spec.worldbody
    base = spec.body("base")
    base.add_freejoint(name="root")
    base.pos = [0.0, 0.0, HIP_H + 0.002]
    base.quat = [math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0]     # CAD Y-up -> world Z-up
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.05], rgba=[0.85, 0.88, 0.92, 1],
                contype=1, conaffinity=2, friction=[0.6, 0.005, 0.0001], solref=[0.002, 1.0])
    # stiff contact = a hard floor / table. With MuJoCo's default soft contact (0.02 s) the shoe's heavily loaded
    # inboard edge sinks and the leg rolls over: with 150 g counterweight the CoM is only ~12 mm inside that edge,
    # so on soft ground (snow) it would tip. Demo on a hard surface.
    wb.add_light(pos=[0.3, -0.6, 1.2], dir=[-0.2, 0.5, -1.0], diffuse=[0.8, 0.8, 0.8])
    colours = {"base": [0.35, 0.37, 0.42, 1], "abad": [0.35, 0.37, 0.42, 1], "thigh": [0.18, 0.44, 0.87, 1],
               "shank": [0.18, 0.44, 0.87, 1], "shoe": [0.88, 0.48, 0.22, 1]}
    for g in spec.geoms:
        if g.name == "floor":
            continue
        b = g.parent.name if hasattr(g.parent, "name") else ""
        g.contype, g.conaffinity = 2, 1                      # robot touches the floor only (no self-collision)
        g.solref = [0.002, 1.0]
        if b in colours:
            g.rgba = colours[b]
    # sole contact: MuJoCo collides a mesh with the floor through a few contact points that need not sit at the
    # shoe's edges, which made the 70 mm wide shoe behave like a narrow one. Contact is instead carried by small
    # spheres along the real sole outline (the mesh stays for looks only).
    shoe = spec.body("shoe")
    for g in spec.geoms:
        if g.parent.name == "shoe" and g.type == mujoco.mjtGeom.mjGEOM_MESH:
            g.contype, g.conaffinity = 0, 0
    rs = 0.0015
    zl = P.zc(0.0)
    o = SG.outline(rs * 1000, 16)
    ring = [o[int(i * len(o) / 16)] for i in range(16)] + [(SG.U_HEEL + 20.0, 0.0), (0.0, 0.0), (SG.U_TOE - 20.0, 0.0)]
    for k, (u, w) in enumerate(ring):
        shoe.add_geom(name=f"sole{k}", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[rs, 0, 0],
                      pos=[u / 1000, (SG.V_SOLE / 1000) + rs, (zl + w) / 1000], contype=2, conaffinity=1,
                      solref=[0.002, 1.0], rgba=[0.88, 0.48, 0.22, 0.0], mass=0.0, density=0.0)
    for jn, e in EFFORT.items():
        a = spec.add_actuator(name=jn, target=jn, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.ctrllimited = True
        a.ctrlrange = [-e, e]
        a.gear = [1, 0, 0, 0, 0, 0]
    ank = spec.joint("ankle")
    ank.frictionloss = 0.09                                   # friction hinge
    ank.damping = [0.002, 0.0, 0.0]
    # the lips are rigid printed stops: stiff limit (MuJoCo's default soft limit let gravity push 24 deg past them)
    ank.solref_limit = [0.002, 1.0]
    ank.solimp_limit = [0.98, 0.995, 0.0005, 0.5, 2.0]
    # same for the driven joints: their limits are hard stops / firmware limits, not springs
    # (soft limits let the policy drive the hip to -111 deg and the knee to 158 deg)
    for jn in ("abad", "hip", "knee"):
        j = spec.joint(jn)
        j.solref_limit = [0.002, 1.0]
        j.solimp_limit = [0.98, 0.995, 0.0005, 0.5, 2.0]
    key = spec.add_key(name="stand")
    m = spec.compile()
    qpos = np.zeros(m.nq)
    qpos[0:3] = [0.0, 0.0, HIP_H + 0.002]
    qpos[3:7] = [math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0]
    for jn, v in STAND.items():
        qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)]] = v
    # drop the sole onto the floor: forward kinematics, then lift / lower the root so the lowest point is 1 mm up
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    mujoco.mj_forward(m, d)
    zmin = min(lowest(m, d, g) for g in range(m.ngeom)
               if m.geom_dataid[g] >= 0 or m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE)
    qpos[2] -= zmin - 0.001
    key.qpos = qpos
    m = spec.compile()
    xml = spec.to_xml()
    open(OUT, "w", encoding="utf-8").write(xml)
    return m


def check(m):
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    jid = {jn: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn) for jn in EFFORT}
    act = {jn: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, jn) for jn in EFFORT}
    kp, kd = 15.0, 0.4
    up0 = None
    for i in range(int(5.0 / m.opt.timestep)):
        for jn in EFFORT:
            qa, va = m.jnt_qposadr[jid[jn]], m.jnt_dofadr[jid[jn]]
            d.ctrl[act[jn]] = kp * (STAND[jn] - d.qpos[qa]) - kd * d.qvel[va]
        mujoco.mj_step(m, d)
        # base 'up' (CAD +Y) in world
        R = d.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")].reshape(3, 3)
        up = R[:, 1]
        if up0 is None:
            up0 = up.copy()
    tilt = math.degrees(math.acos(max(-1, min(1, up[2]))))
    h = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")][2]
    print(f"after 5 s holding the standing pose: base tilt {tilt:.1f} deg, hip height {h * 1000:.0f} mm "
          f"-> {'STANDS' if tilt < 15 and h > 0.15 else 'FALLS OVER'}")
    return tilt, h


if __name__ == "__main__":
    m = build()
    print(f"wrote {OUT}: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, total mass {sum(m.body_mass):.3f} kg")
    if "--check" in sys.argv:
        check(m)
