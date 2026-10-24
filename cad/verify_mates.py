"""Checks the generated mates: every mate rebuilds without error, each joint
really rotates (a temporary angle mate drives it inside its limit and the moving
part's box changes), and each limit mate really blocks (driving past the limit
gives a mate error). Nothing is saved. Writes cad/parts/mate_check.txt.
Run after build_assembly.py:  python cad/verify_mates.py
"""
import math
import os
import sys

import pythoncom
import win32com.client as w32

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from swlib import sw, MM  # noqa: E402
from build_assembly import comp_of, feat, ANGLE, CLOSEST, PARTS  # noqa: E402

out = []


def log(s):
    print(s)
    out.append(s)


def open_asm(name):
    e = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    wn = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    return sw().OpenDoc6(os.path.join(PARTS, name), 2, 1, "", e, wn)


def mate_errors(asm):
    bad = []
    f = asm.FirstFeature
    while f is not None:
        if f.GetTypeName2 == "MateGroup":
            m = f.GetFirstSubFeature
            while m is not None:
                code = m.GetErrorCode2(w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_BOOL, False))
                if code not in (0, None):
                    bad.append((m.Name, code))
                m = m.GetNextSubFeature
        f = f.GetNextFeature
    return bad


def try_angle(asm, moving, base, plane, deg):
    """Add a temporary driving angle mate, rebuild, report new box + errors, then delete it."""
    b0 = moving.GetBox(False, False)
    asm.ClearSelection2(True)
    feat(moving, plane).Select2(False, 1)
    feat(base, plane).Select2(True, 1)
    err = w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    m = asm.AddMate5(ANGLE, CLOSEST, False, 0.0, 0.0, 0.0, 1.0, 1.0, math.radians(deg), math.radians(deg), math.radians(deg),
                     False, False, 0, err)
    asm.ClearSelection2(True)
    asm.ForceRebuild3(False)
    b1 = moving.GetBox(False, False)
    errs = mate_errors(asm)
    moved = max(abs(a - b) for a, b in zip(b0, b1)) / MM
    # remove the temporary mate again
    asm.ClearSelection2(True)
    if m is not None:
        asm.FeatureByPositionReverse(0).Select2(False, 0)
        asm.Extension.DeleteSelection2(0)
    asm.ForceRebuild3(False)
    return m is not None, moved, errs


def check(asm, label, moving, base, plane, inside, outside):
    ok_in, moved, errs = try_angle(asm, moving, base, plane, inside)
    log(f"- {label}: driven to {inside} deg -> moved {moved:.1f} mm, mate errors {errs or 'none'} "
        f"-> {'PASS' if ok_in and moved > 1 and not errs else 'FAIL'}")
    ok_out, moved2, errs2 = try_angle(asm, moving, base, plane, outside)
    blocked = bool(errs2) or not ok_out or moved2 < moved * 0.5
    log(f"- {label}: driven past limit to {outside} deg -> errors {errs2 or 'none'}, moved {moved2:.1f} mm "
        f"-> limit {'HOLDS' if blocked else 'DOES NOT HOLD'}")


def fresh(name):
    sw().CloseAllDocuments(True)       # discard any temporary mates
    return open_asm(name)


if __name__ == "__main__":
    hip = fresh("hip_module.SLDASM")
    log("## hip_module.SLDASM")
    log(f"- mate errors after rebuild: {mate_errors(hip) or 'none'}")
    for label, mv, bs, ins, outs in (("hip +", "thigh_outer", "module_plate_inboard", 60, 125),
                                     ("knee", "shank", "thigh_outer", 90, 170),
                                     ("ankle (snowshoe)", "snowshoe", "shank", 45, 70)):
        hip = fresh("hip_module.SLDASM")
        g = lambda n: comp_of(hip, n)
        ok_in, moved, errs = try_angle(hip, g(mv), g(bs), "Right Plane", ins)
        log(f"- {label}: driven to {ins} deg -> moved {moved:.1f} mm, errors {errs or 'none'} -> {'PASS' if ok_in and moved > 1 and not errs else 'FAIL'}")
        hip = fresh("hip_module.SLDASM")
        g = lambda n: comp_of(hip, n)
        ok_out, moved2, errs2 = try_angle(hip, g(mv), g(bs), "Right Plane", outs)
        log(f"- {label}: driven past limit to {outs} deg -> errors {errs2 or 'none'}, moved {moved2:.1f} mm -> limit {'HOLDS' if errs2 or moved2 < 1 else 'DOES NOT HOLD'}")
    for ins, outs in ((30, 70),):
        leg = fresh("leg_3dof.SLDASM")
        if ins == 30:
            log("## leg_3dof.SLDASM")
            log(f"- mate errors after rebuild: {mate_errors(leg) or 'none'}")
        hipc = comp_of(leg, "hip_module")
        log(f"- hip_module solving = {hipc.Solving} (1 = flexible), fixed = {hipc.IsFixed}")
        ok_in, moved, errs = try_angle(leg, comp_of(leg, "module_plate_inboard", hipc), comp_of(leg, "body_spine"), "Top Plane", ins)
        log(f"- ab/ad: driven to {ins} deg -> moved {moved:.1f} mm, errors {errs or 'none'} -> {'PASS' if ok_in and moved > 1 and not errs else 'FAIL'}")
        leg = fresh("leg_3dof.SLDASM")
        hipc = comp_of(leg, "hip_module")
        ok_out, moved2, errs2 = try_angle(leg, comp_of(leg, "module_plate_inboard", hipc), comp_of(leg, "body_spine"), "Top Plane", outs)
        log(f"- ab/ad: driven past limit to {outs} deg -> errors {errs2 or 'none'}, moved {moved2:.1f} mm -> limit {'HOLDS' if errs2 or moved2 < 1 else 'DOES NOT HOLD'}")
    sw().CloseAllDocuments(True)
    with open(os.path.join(PARTS, "mate_check.txt"), "w") as f:
        f.write("\n".join(out) + "\n")
