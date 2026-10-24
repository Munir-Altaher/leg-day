"""SolidWorks walking animation of the SNOW LEG CAD (snow-mode walk from analysis/gait_check.py).

Builds cad/parts/snow_walk_anim.SLDASM (full 3-DOF snow leg + a ground plate at the stance sole
level) with DRIVING angle mates instead of limit mates:
  hip    thigh Right vs frame Right  = |q1|      (branch checked against the thigh's real rotation)
  knee   shank Right vs thigh Right  = q2        (branch checked)
  ankle  foot Right vs shank Top     = 90 + alpha (unambiguous); alpha = phi in stance (sole level),
         the springs' neutral in swing
Each frame sets the three mates, rebuilds, verifies the pose (thigh / shank / foot rotations vs the
model, < 0.5 deg) and saves a SolidWorks image; the frames are stitched into
docs/renders/snow_walk_solidworks.gif. Nothing outside snow_leg/ is opened or written.
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
import snowgeo as SG  # noqa: E402
from swlib import sw, MM, NOTHING  # noqa: E402
import build_assembly as BA  # noqa: E402
from build_assembly import comp_of, feat, mate, fix, rigid, comp_angle, COINCIDENT, ANGLE, ALIGNED, CLOSEST, N  # noqa: E402
import build_parts as BP  # noqa: E402
import gait_check as GC  # noqa: E402

PARTS = BA.PARTS
FRAMES = os.path.join(ROOT, "docs", "renders", "walk_frames")


def angle_mate_feats(asm):
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


def set_val(mf, deg):
    mf.Parameter("D1").SetSystemValue3(math.radians(deg), 1, NOTHING)


def build():
    sw().CloseAllDocuments(True)
    p = BP.Part(N("ref_ground"))
    y = -(P.SNOW_STANCE_FRAC * (P.L1 + P.L2) + SG.ANKLE_H)
    p.slab("XZ", y - 4.0, y, [BP.rect(-260.0, -40.0, 260.0, 140.0)])
    p.save(PARTS, BP.STL)
    p.close()
    names = BA.MODULE + BA.LEG + BA.FOOT_GROUP + BA.ABAD + ["ref_ground"]
    out = os.path.join(PARTS, N("walk_anim") + ".SLDASM")
    if os.path.exists(out):
        os.remove(out)
    asm, path = BA.build("walk_anim", names)
    g = lambda n: comp_of(asm, n)        # noqa: E731
    for n in BA.MODULE + BA.ABAD + ["ref_ground"]:
        fix(asm, g(n))
    frame, thigh, shank = g("module_plate_inboard"), g("thigh_outer"), g(BA.SHANK)
    mate(asm, COINCIDENT, feat(thigh, "HIP_AXIS"), feat(frame, "HIP_AXIS"), label="HIP_axis")
    mate(asm, COINCIDENT, feat(thigh, "Front Plane"), feat(frame, "Front Plane"), ALIGNED, label="HIP_plane")
    mate(asm, ANGLE, feat(thigh, "Right Plane"), feat(frame, "Right Plane"), CLOSEST, 0.0, 0.0, label="HIP_drive")
    rigid(asm, thigh, g("thigh_cover"))
    mate(asm, COINCIDENT, feat(g("hip_shaft_adapter"), "HIP_AXIS"), feat(frame, "HIP_AXIS"))
    mate(asm, COINCIDENT, feat(g("hip_shaft_adapter"), "Front Plane"), feat(frame, "Front Plane"), ALIGNED)
    rigid(asm, g("hip_shaft_adapter"), g("ref_pulley_20T_kit"))
    mate(asm, COINCIDENT, feat(shank, "KNEE_AXIS"), feat(thigh, "KNEE_AXIS"), label="KNEE_axis")
    mate(asm, COINCIDENT, feat(shank, "Front Plane"), feat(thigh, "Front Plane"), ALIGNED, label="KNEE_plane")
    mate(asm, ANGLE, feat(shank, "Right Plane"), feat(thigh, "Right Plane"), CLOSEST, 0.0, 0.0, label="KNEE_drive")
    rigid(asm, shank, g(f"knee_pulley_{P.KNEE_TEETH}T"))
    rigid(asm, shank, g("ref_knee_post_sleeves"))
    foot = g("ankle_bracket_out")
    for n in ("foot_snowshoe", "ankle_bracket_in", "ref_ankle_pin", "ref_stop_sleeves", "ref_springs"):
        rigid(asm, foot, g(n))
    mate(asm, COINCIDENT, feat(foot, "ANKLE_AXIS"), feat(shank, "ANKLE_AXIS"), label="ANKLE_axis")
    mate(asm, COINCIDENT, feat(foot, "Front Plane"), feat(shank, "Front Plane"), ALIGNED, label="ANKLE_plane")
    mate(asm, ANGLE, feat(foot, "Right Plane"), feat(shank, "Top Plane"), CLOSEST, 90.0, 90.0, label="ANKLE_drive", cur=90.0)
    asm.ForceRebuild3(False)
    asm.SaveAs3(path, 0, 2)
    return asm, path


def pose(asm, mates, q1, q2, alpha):
    """Set the driving mates and verify the absolute rotations; flip a mate's branch if needed."""
    g = lambda n: comp_of(asm, n)        # noqa: E731
    hip_m, knee_m, ank_m = mates
    want = {"thigh_outer": q1, BA.SHANK: q1 + q2, "ankle_bracket_out": q1 + q2 - alpha}
    set_val(hip_m, abs(q1))
    set_val(knee_m, abs(q2))
    set_val(ank_m, 90.0 + alpha)
    asm.ForceRebuild3(False)
    for n, mf in (("thigh_outer", hip_m), (BA.SHANK, knee_m)):
        err = (comp_angle(g(n)) - want[n] + 180) % 360 - 180
        if abs(err) > 0.5:
            spec = mf.GetSpecificFeature2
            spec.Flipped = not spec.Flipped
            asm.ForceRebuild3(False)
    errs = {n: abs((comp_angle(g(n)) - w + 180) % 360 - 180) for n, w in want.items()}
    return max(errs.values())


def main():
    asm, path = build()
    mates = angle_mate_feats(asm)                         # creation order: HIP, KNEE, ANKLE
    rows, w = GC.analyse(GC.gaits(True)["walk"], True, n=240)
    os.makedirs(FRAMES, exist_ok=True)
    for f in os.listdir(FRAMES):
        os.remove(os.path.join(FRAMES, f))
    nf = 36
    ph = GC.gaits(True)["walk"]["phase"]["LF"]
    asm.ShowNamedView2("*Trimetric", -1)
    files, errs = [], []
    for k in range(nf):
        s = (k / nf + ph) % 1.0
        r = rows[int(s * len(rows)) % len(rows)]
        q1, q2 = math.degrees(r["q"][1]), math.degrees(r["q"][2])
        e = pose(asm, mates, q1, q2, r["alpha"])
        errs.append(e)
        if k == 0:
            asm.ViewZoomtofit2()
        f = os.path.join(FRAMES, f"f{k:03d}.png")
        asm.SaveAs3(f, 0, 2)
        files.append(f)
        print(f"frame {k:02d} q1 {q1:7.1f} q2 {q2:6.1f} alpha {r['alpha']:5.1f} {'stance' if r['stance'] else 'swing '} "
              f"pose error {e:.2f} deg", flush=True)
    r0 = rows[int(ph * len(rows)) % len(rows)]
    pose(asm, mates, math.degrees(r0["q"][1]), math.degrees(r0["q"][2]), r0["alpha"])
    asm.SaveAs3(path, 0, 2)
    sw().CloseAllDocuments(True)
    from PIL import Image
    import numpy as np
    imgs = [Image.open(f).convert("RGB") for f in files]
    boxes = []
    for im in imgs:
        a = np.asarray(im).astype(int)
        ys, xs = np.where(np.abs(a - a[5, 5]).sum(axis=2) > 60)
        if len(xs):
            boxes.append((xs.min(), ys.min(), xs.max(), ys.max()))
    x0 = max(0, min(b[0] for b in boxes) - 30); y0 = max(0, min(b[1] for b in boxes) - 30)
    x1 = max(b[2] for b in boxes) + 30; y1 = max(b[3] for b in boxes) + 30
    imgs = [im.crop((x0, y0, x1, y1)) for im in imgs]
    imgs = [im.resize((720, int(im.height * 720 / im.width))) for im in imgs]
    out = os.path.join(ROOT, "docs", "renders", "snow_walk_solidworks.gif")
    imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=int(1000 * GC.gaits(True)["walk"]["T"] / nf), loop=0)
    print(f"max pose error {max(errs):.2f} deg; wrote {out}")
    return max(errs) < 1.0


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
