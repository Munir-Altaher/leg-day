"""Builds hip_module.SLDASM (2-DOF, stand-alone) and leg_3dof.SLDASM (hip module +
ab/ad motor + body bracket) from the generated parts, all in their home pose.
Then runs SolidWorks interference detection at the home pose.
Run after build_parts.py:  python cad/build_assembly.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import params as P  # noqa: E402
from swlib import sw, SHIFT, MM, NOTHING  # noqa: E402

PARTS = os.path.join(HERE, "parts")
HIP_MODULE = ["module_plate_inboard", "module_flange_back", "module_flange_front", "module_frame_outboard", "ref_motor_hip", "ref_motor_knee",
              "hip_shaft_adapter", "ref_pulley_20T_kit", "thigh_outer", "thigh_cover",
              f"knee_pulley_{P.KNEE_TEETH}T", "shank", "snowshoe"]
ABAD = ["ref_motor_abad", "body_bracket_back", "body_bracket_front", "body_spine", "rig_plate"]


def new_assembly():
    tpl = sw().GetUserPreferenceStringValue(9)      # swDefaultTemplateAssembly
    return sw().NewDocument(tpl, 0, 0, 0)


def place(asm, path):
    import pythoncom
    import win32com.client as w32
    e = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    wn = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    typ = 2 if path.upper().endswith(".SLDASM") else 1
    doc = sw().OpenDoc6(path, typ, 1, "", e, wn)
    if typ == 1:
        b = doc.GetPartBox(True)
        c = [(b[i] + b[i + 3]) / 2 - SHIFT * MM for i in range(3)]
    else:
        c = [v * MM for v in SUBASM_CENTER]
    sw().ActivateDoc3(asm.GetTitle, False, 0, e)
    # AddComponent5 puts the component's bounding-box centre at the given point,
    # so pass the body-frame centre of the part -> part lands in its home pose.
    comp = asm.AddComponent5(path, 0, "", False, "", c[0], c[1], c[2])
    return comp


SUBASM_CENTER = [0.0, 0.0, 0.0]
SUBASM_REF = {}            # child part name -> its box (m) inside the sub-assembly, to re-place the sub-assembly exactly


def replace_subasm(asm, comp, path):
    """AddComponent5 centres the sub-assembly's own box (enlarged by reference geometry such as the snowshoe's
    planes) on the given point. Compare a known child's box with where it sat in the sub-assembly file;
    if off, delete and re-place with the corrected centre."""
    name, want = next(iter(SUBASM_REF.items()))
    got = comp_of(asm, name, comp).GetBox(False, False)
    d = [((want[i] + want[i + 3]) - (got[i] + got[i + 3])) / 2 for i in range(3)]
    if max(abs(v) for v in d) > 1e-5:
        asm.ClearSelection2(True)
        comp.Select4(False, NOTHING, False)
        asm.EditDelete()
        SUBASM_CENTER[:] = [SUBASM_CENTER[i] + d[i] / MM for i in range(3)]
        comp = place(asm, path)
        got = comp_of(asm, name, comp).GetBox(False, False)
        d = [((want[i] + want[i + 3]) - (got[i] + got[i + 3])) / 2 for i in range(3)]
    print(f"  sub-assembly placement residual (mm): {[round(v / MM, 4) for v in d]}")
    return comp


def build(name, parts, subasm=None):
    asm = new_assembly()
    title = asm.GetTitle
    comps = []
    if subasm:
        comps.append(replace_subasm(asm, place(asm, subasm), subasm))
    for n in parts:
        comps.append(place(asm, os.path.join(PARTS, n + ".SLDPRT")))
    for n in parts:
        try:
            sw().CloseDoc(n + ".SLDPRT")
        except Exception:
            pass
    asm.ForceRebuild3(False)
    out = os.path.join(PARTS, name + ".SLDASM")
    asm.SaveAs3(out, 0, 2)
    # sanity: component placement (first real part)
    for c in comps:
        if c is None:
            continue
        b = c.GetBox(False, False)
        print(f"  {c.Name2:28s} box(mm) x[{b[0]/MM:7.1f},{b[3]/MM:7.1f}] y[{b[1]/MM:7.1f},{b[4]/MM:7.1f}] z[{b[2]/MM:7.1f},{b[5]/MM:7.1f}]")
    return asm, out


# ------------------------------------------------------------------ mates
COINCIDENT, CONCENTRIC, ANGLE = 0, 1, 6        # swMateType_e
ALIGNED, CLOSEST = 0, 2                         # swMateAlign_e
PLANES3 = ("Front Plane", "Top Plane", "Right Plane")


def comp_of(asm, name, parent=None):
    comps = parent.GetChildren if parent is not None else asm.GetComponents(True)
    for c in comps:
        if c.Name2.split("/")[-1].rsplit("-", 1)[0] == name:
            return c
    raise KeyError(name)


def feat(comp, fname):
    f = comp.FeatureByName(fname)
    if f is None:
        raise KeyError(f"{fname} in {comp.Name2}")
    return f


def mate(asm, kind, fa, fb, align=CLOSEST, lo=0.0, hi=0.0, label="", cur=0.0):
    import pythoncom
    import win32com.client as w32
    asm.ClearSelection2(True)
    fa.Select2(False, 1)
    fb.Select2(True, 1)
    err = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    m = asm.AddMate5(kind, align, False, 0.0, 0.0, 0.0, 1.0, 1.0, math.radians(cur), math.radians(hi), math.radians(lo),
                     False, False, 0, err)
    asm.ClearSelection2(True)
    status = "ok" if m is not None and err.value in (0, 1) else f"FAILED (err {err.value})"
    if m is not None and label:
        try:
            asm.FeatureByPositionReverse(0).Name = label
        except Exception:  # noqa: BLE001
            pass
    print(f"    mate {label or kind}: {status}")
    return m is not None


def fix(asm, comp):
    asm.ClearSelection2(True)
    comp.Select4(False, NOTHING, False)
    asm.FixComponent()
    asm.ClearSelection2(True)


def rigid(asm, a, b):
    """b rides with a: coincide all three base planes (all parts share one frame)."""
    return all(mate(asm, COINCIDENT, feat(b, pl), feat(a, pl), ALIGNED) for pl in PLANES3)


def revolute(asm, moving, base, axis, plane, lim_plane, lo, hi, label, cur=0.0):
    ok = mate(asm, COINCIDENT, feat(moving, axis), feat(base, axis), CLOSEST, label=f"{label}_axis")
    ok &= mate(asm, COINCIDENT, feat(moving, plane), feat(base, plane), ALIGNED, label=f"{label}_plane")
    ok &= mate(asm, ANGLE, feat(moving, lim_plane), feat(base, lim_plane), CLOSEST, lo, hi, label=f"{label}_limit", cur=cur)
    return ok


def mate_hip_module(asm):
    g = lambda n: comp_of(asm, n)
    frame = g("module_plate_inboard")
    for n in ("module_plate_inboard", "module_flange_back", "module_flange_front", "module_frame_outboard", "ref_motor_hip", "ref_motor_knee"):
        fix(asm, g(n))
    ok = revolute(asm, g("thigh_outer"), frame, "HIP_AXIS", "Front Plane", "Right Plane", *P_LIM["hip"], "HIP")
    ok &= rigid(asm, g("thigh_outer"), g("thigh_cover"))
    ok &= mate(asm, COINCIDENT, feat(g("hip_shaft_adapter"), "HIP_AXIS"), feat(frame, "HIP_AXIS"), label="KNEE_MOTOR_axis")
    ok &= mate(asm, COINCIDENT, feat(g("hip_shaft_adapter"), "Front Plane"), feat(frame, "Front Plane"), ALIGNED, label="KNEE_MOTOR_plane")
    ok &= rigid(asm, g("hip_shaft_adapter"), g("ref_pulley_20T_kit"))
    ok &= revolute(asm, g("shank"), g("thigh_outer"), "KNEE_AXIS", "Front Plane", "Right Plane", *P_LIM["knee"], "KNEE")
    ok &= rigid(asm, g("shank"), g(f"knee_pulley_{P.KNEE_TEETH}T"))
    # snowshoe on the ankle: friction hinge about the plug's M3 bolt, lips at ANKLE_LIM (the pad is modelled at
    # ANKLE_BUILD, inside the range, so the unsigned plane angle never crosses 0)
    # (snowshoe Top Plane vs shank Right Plane reads 90 - alpha, so the mirrored solution, 90 + alpha, is outside the limit)
    ok &= mate(asm, COINCIDENT, feat(g("snowshoe"), "ANKLE_AXIS"), feat(g("shank"), "ANKLE_AXIS"), CLOSEST, label="ANKLE_axis")
    ok &= mate(asm, COINCIDENT, feat(g("snowshoe"), "Front Plane"), feat(g("shank"), "Front Plane"), ALIGNED, label="ANKLE_plane")
    ok &= mate(asm, ANGLE, feat(g("snowshoe"), "Top Plane"), feat(g("shank"), "Right Plane"), CLOSEST,
               90.0 - P.ANKLE_LIM[1], 90.0 - P.ANKLE_LIM[0], label="ANKLE_limit", cur=90.0 - P.ANKLE_BUILD)
    return ok


def mate_leg(asm):
    g = lambda n: comp_of(asm, n)
    for n in ("body_spine", "body_bracket_back", "body_bracket_front", "rig_plate", "ref_motor_abad"):
        fix(asm, g(n))
    hip = g("hip_module")
    asm.ClearSelection2(True)                # inserted first -> SolidWorks fixed it; the ab/ad mates locate it
    hip.Select4(False, NOTHING, False)
    asm.UnfixComponent()
    asm.ClearSelection2(True)
    try:
        hip.Solving = 1                     # swComponentFlexibleSolving: drag hip + knee inside the leg
    except Exception as e:  # noqa: BLE001
        print("    could not set hip_module flexible:", e)
    frame = comp_of(asm, "module_plate_inboard", hip)
    spine = g("body_spine")
    return revolute(asm, frame, spine, "ABAD_AXIS", "Right Plane", "Top Plane", *P_LIM["abad"], "ABAD")


# unsigned angle limits (SolidWorks angle mates between planes measure 0..180)
P_LIM = {"hip": (0.0, max(abs(v) for v in P.Q1_LIM)), "knee": (0.0, P.Q2_LIM[1]),
         "abad": (0.0, max(abs(v) for v in P.Q0_LIM))}


def interference(asm):
    try:
        mgr = asm.InterferenceDetectionManager
        mgr.TreatCoincidenceAsInterference = False
        mgr.IncludeMultibodyPartInterferences = False
        n = mgr.GetInterferenceCount
        res = mgr.GetInterferences
        lines = []
        for it in (res or []):
            comps = [c.Name2 for c in it.Components]
            lines.append(f"{' x '.join(comps)}: {it.Volume / MM**3:.1f} mm^3")
        mgr.Done()
        return n, lines
    except Exception as e:  # noqa: BLE001
        return None, [f"interference API failed: {e}"]


if __name__ == "__main__":
    sw().CloseAllDocuments(True)
    for f in ("hip_module.SLDASM", "leg_3dof.SLDASM"):
        if os.path.exists(os.path.join(PARTS, f)):
            os.remove(os.path.join(PARTS, f))
    asm, hip = build("hip_module", HIP_MODULE)
    boxes = [c.GetBox(False, False) for c in asm.GetComponents(True)]
    SUBASM_CENTER[:] = [(min(b[i] for b in boxes) + max(b[i + 3] for b in boxes)) / 2 / MM for i in range(3)]
    SUBASM_REF["module_plate_inboard"] = list(comp_of(asm, "module_plate_inboard").GetBox(False, False))
    print("hip_module mates:")
    ok1 = mate_hip_module(asm)
    asm.ForceRebuild3(False)
    n, lines = interference(asm)                 # at the mated home pose (the ankle mate turns the snowshoe)
    print("hip_module interferences:", n, *lines, sep="\n  ")
    asm.SaveAs3(hip, 0, 2)
    sw().CloseDoc(asm.GetTitle)
    asm2, leg = build("leg_3dof", ABAD, subasm=hip)
    print("leg_3dof mates:")
    ok2 = mate_leg(asm2)
    asm2.ForceRebuild3(False)
    n2, lines2 = interference(asm2)
    asm2.SaveAs3(leg, 0, 2)
    print("MATES", "ALL OK" if ok1 and ok2 else "SOME FAILED")
    print("leg_3dof interferences:", n2, *lines2, sep="\n  ")
    with open(os.path.join(PARTS, "interference_home_pose.txt"), "w") as f:
        f.write(f"hip_module: {n}\n" + "\n".join(lines) + f"\nleg_3dof: {n2}\n" + "\n".join(lines2) + "\n")
