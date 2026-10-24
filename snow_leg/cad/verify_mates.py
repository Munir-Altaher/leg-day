"""SNOW LEG: checks the generated mates (snow_hip_module / snow_leg_3dof): every mate rebuilds without error, each joint
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
from build_assembly import comp_of, feat, ANGLE, CLOSEST, PARTS, N, SUFFIX, SHANK  # noqa: E402
import params as P  # noqa: E402
import snowgeo as SG  # noqa: E402

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


def try_angle(asm, moving, base, plane, deg, base_plane=None):
    """Add a temporary driving angle mate, rebuild, report new box + errors, then delete it."""
    b0 = moving.GetBox(False, False)
    asm.ClearSelection2(True)
    feat(moving, plane).Select2(False, 1)
    feat(base, base_plane or plane).Select2(True, 1)
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


def joint(asmname, label, mv, bs, plane, base_plane, inside, outsides, nested=False):
    for kind, val in [("in", inside)] + [("out", o) for o in outsides]:
        a = fresh(asmname)
        if nested:
            hipc = comp_of(a, "hip_module" + SUFFIX)
            m_, b_ = comp_of(a, mv, hipc), comp_of(a, bs)
        else:
            m_, b_ = comp_of(a, mv), comp_of(a, bs)
        ok, moved, errs = try_angle(a, m_, b_, plane, val, base_plane)
        if kind == "in":
            log(f"- {label}: driven to {val:.1f}° -> moved {moved:.1f} mm, errors {errs or 'none'} -> "
                f"{'PASS' if ok and moved > 1 and not errs else 'FAIL'}")
        else:
            log(f"- {label}: driven past the limit to {val:.1f}° -> errors {errs or 'none'}, moved {moved:.1f} mm -> "
                f"limit {'HOLDS' if errs or moved < 1 else 'DOES NOT HOLD'}")


if __name__ == "__main__":
    hip, leg = N("hip_module" + SUFFIX) + ".SLDASM", N("leg_3dof" + SUFFIX) + ".SLDASM"
    a = fresh(hip)
    log(f"## {hip}")
    log(f"- mate errors after rebuild: {mate_errors(a) or 'none'}")
    joint(hip, "hip (thigh Right vs frame Right)", "thigh_outer", "module_plate_inboard", "Right Plane", None, 60.0, [125.0])
    joint(hip, "knee (shank Right vs thigh Right)", SHANK, "thigh_outer", "Right Plane", None, 90.0, [160.0])
    if P.FOOT == "snowshoe":
        joint(hip, "ankle (foot Right vs shank Top = 90 + α)", "ankle_bracket_out", SHANK, "Right Plane", "Top Plane",
              90.0 + 30.0, [90.0 + SG.ALPHA_MAX + 15.0, 90.0 + SG.ALPHA_MIN - 15.0])
    a = fresh(leg)
    log(f"## {leg}")
    log(f"- mate errors after rebuild: {mate_errors(a) or 'none'}")
    hc = comp_of(a, "hip_module" + SUFFIX)
    log(f"- hip_module solving = {hc.Solving} (1 = flexible), fixed = {hc.IsFixed}")
    joint(leg, "ab/ad (frame Top vs spine Top)", "module_plate_inboard", "body_spine", "Top Plane", None, 30.0, [70.0], nested=True)
    sw().CloseAllDocuments(True)
    with open(os.path.join(PARTS, f"mate_check{SUFFIX}.txt"), "w", encoding="utf-8") as f:
        f.write(chr(10).join(out) + chr(10))
