"""SolidWorks walking animation of the real CAD.

Builds cad/parts/walk_anim.SLDASM (full 3-DOF leg + snowshoe + a ground plate). The hip,
knee and ankle are held by *driving* angle mates instead of limit mates. Each frame sets
hip and knee to the IK angles of the walking gait in analysis/gait_check.py (left-front
leg, lateral-sequence walk, 120 mm stride, 40 mm swing clearance) and the ankle to the
snowshoe's angle (flat on the ground in stance, holding its lift-off angle in swing),
rebuilds, and saves a SolidWorks view. The frames are stitched into
docs/renders/walk_solidworks.gif. To edit the pose in SolidWorks, change the
HIP / KNEE angle mates.
Run:  python cad/walk_animation.py
"""
import math
import os
import sys

import pythoncom
import win32com.client as w32

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "analysis"))
import params as P  # noqa: E402
from swlib import sw, MM, NOTHING, Part, rect  # noqa: E402
import build_assembly as BA  # noqa: E402
from build_assembly import comp_of, feat, mate, fix, rigid, COINCIDENT, ANGLE, ALIGNED, CLOSEST  # noqa: E402
import gait_check as GC  # noqa: E402
from leg_model import ik  # noqa: E402
import numpy as np  # noqa: E402

PARTS = BA.PARTS
FRAMES = os.path.join(ROOT, "docs", "renders", "walk_frames")
GROUND_Y = -GC.GROUND * 1000                 # sole level


def ground_part():
    p = Part("ref_ground")
    p.slab("XZ", GROUND_Y - 4.0, GROUND_Y, [rect(-170.0, -20.0, 170.0, 110.0)])
    p.save(PARTS, os.path.join(ROOT, "cad", "stl"))
    p.close()


def angle_mates(asm):
    """Driving angle mate features in creation order."""
    out = []
    f = asm.FirstFeature
    while f is not None:
        if f.GetTypeName2 == "MateGroup":
            m = f.GetFirstSubFeature
            while m is not None:
                if m.Name.startswith("Angle"):
                    out.append(m)
                m = m.GetNextSubFeature
        f = f.GetNextFeature
    return out


def set_angle(mfeat, deg):
    d = mfeat.Parameter("D1")
    d.SetSystemValue3(math.radians(deg), 1, NOTHING)      # swSetValue_InThisConfiguration


def foot_xy(asm):
    b = comp_of(asm, "snowshoe").GetBox(False, False)
    return (b[0] + b[3]) / 2 / MM, (b[1] + b[4]) / 2 / MM


def rot_deg(comp):
    """Component rotation about Z, degrees (CCW)."""
    a = comp.Transform2.ArrayData
    return math.degrees(math.atan2(a[1], a[0]))


def build():
    sw().CloseAllDocuments(True)
    ground_part()
    names = BA.HIP_MODULE + BA.ABAD + ["ref_ground"]
    asm, path = BA.build("walk_anim", names)
    g = lambda n: comp_of(asm, n)
    for n in ("module_plate_inboard", "module_flange_back", "module_flange_front", "module_frame_outboard", "ref_motor_hip", "ref_motor_knee", "ref_ground") + tuple(BA.ABAD):
        fix(asm, g(n))
    frame = g("module_plate_inboard")
    # hip: axis + plane + driving angle (thigh Right vs frame Top: angle = 90 - q1, unambiguous for |q1| < 90)
    mate(asm, COINCIDENT, feat(g("thigh_outer"), "HIP_AXIS"), feat(frame, "HIP_AXIS"), label="HIP_axis")
    mate(asm, COINCIDENT, feat(g("thigh_outer"), "Front Plane"), feat(frame, "Front Plane"), ALIGNED, label="HIP_plane")
    mate(asm, ANGLE, feat(g("thigh_outer"), "Right Plane"), feat(frame, "Top Plane"), CLOSEST, 90.0, 90.0, label="HIP_drive")
    rigid(asm, g("thigh_outer"), g("thigh_cover"))
    mate(asm, COINCIDENT, feat(g("hip_shaft_adapter"), "HIP_AXIS"), feat(frame, "HIP_AXIS"), label="KM_axis")
    mate(asm, COINCIDENT, feat(g("hip_shaft_adapter"), "Front Plane"), feat(frame, "Front Plane"), ALIGNED, label="KM_plane")
    rigid(asm, g("hip_shaft_adapter"), g("ref_pulley_20T_kit"))
    # knee: axis + plane + driving angle (shank Right vs thigh Right = |q2|)
    mate(asm, COINCIDENT, feat(g("shank"), "KNEE_AXIS"), feat(g("thigh_outer"), "KNEE_AXIS"), label="KNEE_axis")
    mate(asm, COINCIDENT, feat(g("shank"), "Front Plane"), feat(g("thigh_outer"), "Front Plane"), ALIGNED, label="KNEE_plane")
    mate(asm, ANGLE, feat(g("shank"), "Right Plane"), feat(g("thigh_outer"), "Right Plane"), CLOSEST, 0.0, 0.0, label="KNEE_drive")
    rigid(asm, g("shank"), g(f"knee_pulley_{P.KNEE_TEETH}T"))
    # ankle: axis + plane + driving angle (snowshoe Top vs shank Right = 90 - alpha, unambiguous)
    mate(asm, COINCIDENT, feat(g("snowshoe"), "ANKLE_AXIS"), feat(g("shank"), "ANKLE_AXIS"), label="ANKLE_axis")
    mate(asm, COINCIDENT, feat(g("snowshoe"), "Front Plane"), feat(g("shank"), "Front Plane"), ALIGNED, label="ANKLE_plane")
    mate(asm, ANGLE, feat(g("snowshoe"), "Top Plane"), feat(g("shank"), "Right Plane"), CLOSEST,
         90.0 - P.ANKLE_BUILD, 90.0 - P.ANKLE_BUILD, label="ANKLE_drive", cur=90.0 - P.ANKLE_BUILD)
    asm.ForceRebuild3(False)
    asm.SaveAs3(path, 0, 2)
    return asm, path


def pose(asm, hip_m, knee_m, ankle_m, q1, q2, alpha):
    """Set the leg to (q1, q2) and the snowshoe to alpha, in degrees; fix the unsigned knee-angle branch by
    checking the snowshoe against FK (its box centre sits within ~20 mm of the ankle pivot)."""
    a = math.radians(q1 + q2 + P.ANKLE_OFF_DEG)
    want = (P.L1 * math.sin(math.radians(q1)) + P.L2_ANKLE * math.sin(a),
            -P.L1 * math.cos(math.radians(q1)) - P.L2_ANKLE * math.cos(a))
    set_angle(hip_m, 90.0 - q1)
    set_angle(knee_m, abs(q2))
    set_angle(ankle_m, 90.0 - alpha)
    asm.ForceRebuild3(False)
    fx, fy = foot_xy(asm)
    err = math.hypot(fx - want[0], fy - want[1])
    if err > 40.0:      # mirrored branch is ~230 mm off
        km = knee_m.GetSpecificFeature2
        km.Flipped = not km.Flipped
        asm.ForceRebuild3(False)
        fx, fy = foot_xy(asm)
        err = math.hypot(fx - want[0], fy - want[1])
    # the ankle plane pair also reads the same at 180 - alpha: check the shoe's real angle against the shank's
    def shoe_off():
        rel = rot_deg(comp_of(asm, "snowshoe")) - rot_deg(comp_of(asm, "shank"))
        return abs((rel + alpha + 180.0) % 360.0 - 180.0)          # wanted: rel = -alpha
    if shoe_off() > 5.0:
        am = ankle_m.GetSpecificFeature2
        am.Flipped = not am.Flipped
        asm.ForceRebuild3(False)
    if shoe_off() > 5.0:
        print(f"    WARNING: snowshoe {shoe_off():.1f} deg off its target", flush=True)
    return err


def main():
    asm, path = build()
    ms = angle_mates(asm)
    hip_m, knee_m, ankle_m = ms[0], ms[1], ms[2]
    os.makedirs(FRAMES, exist_ok=True)
    for old in os.listdir(FRAMES):
        os.remove(os.path.join(FRAMES, old))
    rows, _ = GC.analyse("walk", GC.GAITS["walk"], n=240)
    i_lo = next(i for i in range(len(rows)) if not rows[i]["stance"] and rows[i - 1]["stance"])
    GC.PHI_LIFT = math.degrees(rows[i_lo - 1]["q"][1] + rows[i_lo - 1]["q"][2])
    n_frames = 36
    asm.ShowNamedView2("*Trimetric", -1)
    first = True
    errs = []
    files = []
    ph = GC.GAITS["walk"]["phase"]["LF"]
    for k in range(n_frames):
        s = (k / n_frames + ph) % 1.0
        r = rows[int(s * len(rows)) % len(rows)]
        q1, q2 = math.degrees(r["q"][1]), math.degrees(r["q"][2])
        al = GC.shoe_alpha(r["q"][1], r["q"][2], r["stance"])
        e = pose(asm, hip_m, knee_m, ankle_m, q1, q2, al)
        errs.append(e)
        if first:
            asm.ViewZoomtofit2()
            first = False
        f = os.path.join(FRAMES, f"f{k:03d}.png")
        asm.SaveAs3(f, 0, 2)
        files.append(f)
        print(f"frame {k:02d} q1 {q1:6.1f} q2 {q2:6.1f} ankle {al:5.1f} {'stance' if r['stance'] else 'swing '} "
              f"shoe box vs pivot {e:.1f} mm", flush=True)
    # leave the saved assembly in the mid-stance pose
    s = ph % 1.0
    r = rows[int(s * len(rows)) % len(rows)]
    pose(asm, hip_m, knee_m, ankle_m, math.degrees(r["q"][1]), math.degrees(r["q"][2]),
         GC.shoe_alpha(r["q"][1], r["q"][2], r["stance"]))
    asm.SaveAs3(path, 0, 2)
    # stitch
    from PIL import Image
    imgs = [Image.open(f).convert("RGB") for f in files]
    # crop to the union bounding box of non-background pixels
    import numpy as np
    boxes = []
    for im in imgs:
        a = np.asarray(im).astype(int)
        mask = (np.abs(a - a[5, 5]).sum(axis=2) > 60)
        ys, xs = np.where(mask)
        if len(xs):
            boxes.append((xs.min(), ys.min(), xs.max(), ys.max()))
    x0 = max(0, min(b[0] for b in boxes) - 30); y0 = max(0, min(b[1] for b in boxes) - 30)
    x1 = max(b[2] for b in boxes) + 30; y1 = max(b[3] for b in boxes) + 30
    imgs = [im.crop((x0, y0, x1, y1)) for im in imgs]
    w = 640
    imgs = [im.resize((w, int(im.height * w / im.width))) for im in imgs]
    out = os.path.join(ROOT, "docs", "renders", "walk_solidworks.gif")
    imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=int(1000 * GC.GAITS["walk"]["T"] / n_frames), loop=0)
    print("max foot error vs FK:", round(max(errs), 2), "mm")
    print("wrote", out)


if __name__ == "__main__":
    main()
