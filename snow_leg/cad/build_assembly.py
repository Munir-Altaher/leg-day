"""Builds the SNOW LEG assemblies from the generated parts, all in their home pose:
  snow_hip_module.SLDASM   2-DOF hip module + shank + foot (valid on its own: the fallback)
  snow_leg_3dof.SLDASM     snow_hip_module + ab/ad motor + body bracket + rig plate
(FOOT = "ball" builds snow_hip_module_ball / snow_leg_3dof_ball instead.)
Mates: fixed frame, rigid groups by three base-plane coincidences, revolute joints by named
axes + plane + limit-angle mates (hip, knee, ankle, ab/ad). Then SolidWorks interference
detection at the home pose. Nothing outside snow_leg/ is opened or written.
Run after build_parts.py:  python cad/build_assembly.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import params as P  # noqa: E402
import snowgeo as S  # noqa: E402
from swlib import sw, SHIFT, MM, NOTHING  # noqa: E402

PARTS = os.path.join(HERE, "parts")
N = lambda s: P.FILE_PREFIX + s          # noqa: E731
SUFFIX = "" if P.FOOT == "snowshoe" else "_ball"
MODULE = ["module_plate_inboard", "module_flange_back", "module_flange_front", "module_frame_outboard", "hip_shroud_out",
          "ref_motor_hip", "ref_motor_knee"]
LEG = ["hip_shaft_adapter", "ref_pulley_20T_kit", "thigh_outer", "thigh_cover", f"knee_pulley_{P.KNEE_TEETH}T",
       "ref_knee_post_sleeves"]
FOOT_GROUP = (["shank", "foot_snowshoe", "ankle_bracket_in", "ankle_bracket_out", "ref_ankle_pin", "ref_stop_sleeves", "ref_springs"]
              if P.FOOT == "snowshoe" else ["shank_ball", "foot_ball"])
HIP_MODULE = MODULE + LEG + FOOT_GROUP
ABAD = ["ref_motor_abad", "body_bracket_back", "body_bracket_front", "body_spine", "rig_plate"]
SHANK = FOOT_GROUP[0]


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
REF_CHILD = "module_plate_inboard"


def place_subasm(asm, path):
    """AddComponent5 centres the sub-assembly's own box (which can include sketch geometry) on the
    given point. Place it, compare a known child's box with that part's body-frame box, re-place."""
    import pythoncom
    import win32com.client as w32
    comp = place(asm, path)
    child = comp_of(asm, REF_CHILD, comp)
    e = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    wn = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    doc = sw().OpenDoc6(os.path.join(PARTS, N(REF_CHILD) + ".SLDPRT"), 1, 1, "", e, wn)
    want = [v - SHIFT * MM for v in doc.GetPartBox(True)]
    sw().ActivateDoc3(asm.GetTitle, False, 0, e)
    got = child.GetBox(False, False)
    d = [((want[i] + want[i + 3]) - (got[i] + got[i + 3])) / 2 for i in range(3)]
    if max(abs(v) for v in d) > 1e-5:
        asm.ClearSelection2(True)
        comp.Select4(False, NOTHING, False)
        asm.EditDelete()
        SUBASM_CENTER[:] = [SUBASM_CENTER[i] + d[i] / MM for i in range(3)]
        comp = place(asm, path)
        got = comp_of(asm, REF_CHILD, comp).GetBox(False, False)
        d = [((want[i] + want[i + 3]) - (got[i] + got[i + 3])) / 2 / MM for i in range(3)]
    print(f"  sub-assembly placement residual (mm): {[round(v, 4) for v in d]}")
    return comp


def build(name, parts, subasm=None):
    asm = new_assembly()
    title = asm.GetTitle
    comps = []
    if subasm:
        comps.append(place_subasm(asm, subasm))
    for n in parts:
        comps.append(place(asm, os.path.join(PARTS, N(n) + ".SLDPRT")))
    for n in parts:
        try:
            sw().CloseDoc(N(n) + ".SLDPRT")
        except Exception:
            pass
    asm.ForceRebuild3(False)
    out = os.path.join(PARTS, N(name) + ".SLDASM")
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
        if c.Name2.split("/")[-1].rsplit("-", 1)[0] in (name, N(name)):
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


def revolute(asm, moving, base, axis, plane, lim_plane, lo, hi, label):
    ok = mate(asm, COINCIDENT, feat(moving, axis), feat(base, axis), CLOSEST, label=f"{label}_axis")
    ok &= mate(asm, COINCIDENT, feat(moving, plane), feat(base, plane), ALIGNED, label=f"{label}_plane")
    ok &= mate(asm, ANGLE, feat(moving, lim_plane), feat(base, lim_plane), CLOSEST, lo, hi, label=f"{label}_limit")
    return ok


def mate_hip_module(asm):
    g = lambda n: comp_of(asm, n)        # noqa: E731
    frame = g("module_plate_inboard")
    for n in MODULE:
        fix(asm, g(n))
    ok = revolute(asm, g("thigh_outer"), frame, "HIP_AXIS", "Front Plane", "Right Plane", *P_LIM["hip"], "HIP")
    ok &= rigid(asm, g("thigh_outer"), g("thigh_cover"))
    ok &= mate(asm, COINCIDENT, feat(g("hip_shaft_adapter"), "HIP_AXIS"), feat(frame, "HIP_AXIS"), label="KNEE_MOTOR_axis")
    ok &= mate(asm, COINCIDENT, feat(g("hip_shaft_adapter"), "Front Plane"), feat(frame, "Front Plane"), ALIGNED, label="KNEE_MOTOR_plane")
    ok &= rigid(asm, g("hip_shaft_adapter"), g("ref_pulley_20T_kit"))
    ok &= mate(asm, COINCIDENT, feat(g(SHANK), "KNEE_AXIS"), feat(g("thigh_outer"), "KNEE_AXIS"), CLOSEST, label="KNEE_axis")
    ok &= mate(asm, COINCIDENT, feat(g(SHANK), "Front Plane"), feat(g("thigh_outer"), "Front Plane"), ALIGNED, label="KNEE_plane")
    ok &= rigid(asm, g(SHANK), g(f"knee_pulley_{P.KNEE_TEETH}T"))
    ok &= rigid(asm, g(SHANK), g("ref_knee_post_sleeves"))
    if P.FOOT == "snowshoe":
        foot = g("ankle_bracket_out")
        for n in ("foot_snowshoe", "ankle_bracket_in", "ref_ankle_pin", "ref_stop_sleeves", "ref_springs"):
            ok &= rigid(asm, foot, g(n))
        # ankle: foot Right plane vs shank Top plane = 90 + alpha (unambiguous for |alpha| < 90)
        ok &= mate(asm, COINCIDENT, feat(foot, "ANKLE_AXIS"), feat(g(SHANK), "ANKLE_AXIS"), CLOSEST, label="ANKLE_axis")
        ok &= mate(asm, COINCIDENT, feat(foot, "Front Plane"), feat(g(SHANK), "Front Plane"), ALIGNED, label="ANKLE_plane")
    else:
        ok &= rigid(asm, g(SHANK), g("foot_ball"))
    # display pose (the lower knee-stop post forbids q2 < 13 deg, so q = 0 is not a reachable pose)
    set_pose(asm, DISPLAY_POSE)
    ok &= mate(asm, ANGLE, feat(g(SHANK), "Right Plane"), feat(g("thigh_outer"), "Right Plane"), ALIGNED, *P_LIM["knee"], label="KNEE_limit",
               cur=DISPLAY_POSE[1])
    if P.FOOT == "snowshoe":
        ok &= mate(asm, ANGLE, feat(g("ankle_bracket_out"), "Right Plane"), feat(g(SHANK), "Top Plane"), ALIGNED, *P_LIM["ankle"], label="ANKLE_limit",
                   cur=90.0 + sum(DISPLAY_POSE))
    return ok


DISPLAY_POSE = (-10.0, 20.0)       # q1, q2 (deg); ankle set so the sole is level


def _drive(asm, fa, fb, deg):
    """Temporary driving angle mate; returns its feature so it can be deleted."""
    import pythoncom
    import win32com.client as w32
    asm.ClearSelection2(True)
    fa.Select2(False, 1)
    fb.Select2(True, 1)
    err = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    m = asm.AddMate5(ANGLE, CLOSEST, False, 0.0, 0.0, 0.0, 1.0, 1.0, math.radians(deg), math.radians(deg), math.radians(deg),
                     False, False, 0, err)
    asm.ClearSelection2(True)
    asm.ForceRebuild3(False)
    return m


def _delete_last_mate(asm):
    f = asm.FirstFeature
    last = None
    while f is not None:
        if f.GetTypeName2 == "MateGroup":
            m = f.GetFirstSubFeature
            while m is not None:
                last = m
                m = m.GetNextSubFeature
        f = f.GetNextFeature
    if last is not None:
        asm.ClearSelection2(True)
        last.Select2(False, 0)
        asm.Extension.DeleteSelection2(0)
        asm.ForceRebuild3(False)


def _box_c(comp):
    b = comp.GetBox(False, False)
    return [(b[i] + b[i + 3]) / 2 / MM for i in range(3)]


def comp_angle(comp):
    """Rotation of a component about Z (deg, CCW seen from +Z) from its transform."""
    m = comp.Transform2.ArrayData
    return math.degrees(math.atan2(m[1], m[0]))


def _drive_checked(asm, moving, fa, fb, value, expected_abs):
    """Drive an angle mate to `value`; if the solver picked the mirror branch, use 180 - value."""
    for v in (value, 180.0 - value):
        _drive(asm, fa, fb, v)
        got = comp_angle(moving)
        _delete_last_mate(asm)
        err = (got - expected_abs + 180.0) % 360.0 - 180.0
        if abs(err) < 1.0:
            return got
    return got


def set_pose(asm, pose):
    """Drive hip, knee (and ankle) with temporary angle mates (checked against the expected absolute
    rotation), then delete them; components keep the solved positions."""
    g = lambda n: comp_of(asm, n)        # noqa: E731
    q1, q2 = pose
    frame, thigh, shank = g("module_plate_inboard"), g("thigh_outer"), g(SHANK)
    a1 = _drive_checked(asm, thigh, feat(thigh, "Right Plane"), feat(frame, "Top Plane"), 90.0 - q1, q1)
    a2 = _drive_checked(asm, shank, feat(shank, "Right Plane"), feat(thigh, "Top Plane"), 90.0 - q2, q1 + q2)
    a3 = None
    if P.FOOT == "snowshoe":
        foot = g("ankle_bracket_out")
        a3 = _drive_checked(asm, foot, feat(foot, "Right Plane"), feat(shank, "Top Plane"), 90.0 + q1 + q2, 0.0)
    print(f"    display pose set: thigh {a1:.2f} (want {q1}), shank {a2:.2f} (want {q1 + q2}), foot {a3} (want 0 = sole level)")


def mate_leg(asm):
    g = lambda n: comp_of(asm, n)        # noqa: E731
    for n in ABAD:
        fix(asm, g(n))
    hip = g("hip_module" + SUFFIX)
    asm.ClearSelection2(True)                # inserted first -> SolidWorks fixed it; the ab/ad mates locate it
    hip.Select4(False, NOTHING, False)
    asm.UnfixComponent()
    asm.ClearSelection2(True)
    try:
        hip.Solving = 1                     # swComponentFlexibleSolving: drag hip, knee, ankle inside the leg
    except Exception as e:  # noqa: BLE001
        print("    could not set hip_module flexible:", e)
    frame = comp_of(asm, "module_plate_inboard", hip)
    return revolute(asm, frame, g("body_spine"), "ABAD_AXIS", "Right Plane", "Top Plane", *P_LIM["abad"], "ABAD")


# unsigned angle limits (SolidWorks angle mates between planes measure 0..180)
P_LIM = {"hip": (0.0, max(abs(v) for v in P.Q1_LIM)), "knee": (P.Q2_LIM[0], P.Q2_LIM[1]),
         "abad": (0.0, max(abs(v) for v in P.Q0_LIM)),
         "ankle": (90.0 + S.ALPHA_MIN, 90.0 + S.ALPHA_MAX)}


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


def dependencies(path):
    """Every file the saved assembly references (must all live in snow_leg/)."""
    deps = sw().GetDocumentDependencies2(path, True, True, False)
    return list(deps[1::2]) if deps else []


if __name__ == "__main__":
    sw().CloseAllDocuments(True)
    for f in ("hip_module", "leg_3dof"):
        f = os.path.join(PARTS, N(f + SUFFIX) + ".SLDASM")
        if os.path.exists(f):
            os.remove(f)
    asm, hip = build("hip_module" + SUFFIX, HIP_MODULE)
    NL = chr(10)
    print("hip_module mates:")
    ok1 = mate_hip_module(asm)
    asm.ForceRebuild3(False)
    n, lines = interference(asm)
    print("hip_module interferences (display pose):", n, *lines, sep=NL + "  ")
    boxes = [c.GetBox(False, False) for c in asm.GetComponents(True)]
    SUBASM_CENTER[:] = [(min(b[i] for b in boxes) + max(b[i + 3] for b in boxes)) / 2 / MM for i in range(3)]
    asm.SaveAs3(hip, 0, 2)
    sw().CloseDoc(asm.GetTitle)
    asm2, leg = build("leg_3dof" + SUFFIX, ABAD, subasm=hip)
    n2, lines2 = interference(asm2)
    print("leg_3dof mates:")
    ok2 = mate_leg(asm2)
    asm2.ForceRebuild3(False)
    asm2.SaveAs3(leg, 0, 2)
    print("MATES", "ALL OK" if ok1 and ok2 else "SOME FAILED")
    print("leg_3dof interferences:", n2, *lines2, sep=NL + "  ")
    sw().CloseAllDocuments(True)
    root = os.path.abspath(os.path.join(HERE, "..")).lower()
    dep_lines = []
    for a_ in (hip, leg):
        deps = dependencies(a_)
        outside = [d for d in deps if not os.path.abspath(d).lower().startswith(root)]
        dep_lines.append(f"{os.path.basename(a_)}: {len(deps)} references, outside snow_leg: {outside or 'none'}")
        print(dep_lines[-1])
    with open(os.path.join(PARTS, f"interference_home_pose{SUFFIX}.txt"), "w") as f:
        f.write(NL.join([f"{N('hip_module' + SUFFIX)}: {n}"] + lines + [f"{N('leg_3dof' + SUFFIX)}: {n2}"] + lines2 + dep_lines) + NL)
