"""Renders + compare_legs.SLDASM for the SNOW LEG.

- snow renders: snow_leg_3dof (trimetric / side / front / bottom), the snowshoe foot (sole view,
  close-up), the leg in the snow stance (snow_walk_anim saved pose)
- comparison renders: the ORIGINAL leg_3dof.SLDASM (opened read-only from ../cad/parts) and the
  snow leg from the same camera views, stitched side by side
- compare_legs.SLDASM: both legs ~250 mm apart, both fixed; only this document is saved
While the original files are open they are made read-only on disk (file attribute only, content and
SHA-256 unchanged) so SolidWorks cannot save over them; attributes are restored at the end.
Run:  python cad/renders.py
"""
import os
import stat
import sys

import pythoncom
import win32com.client as w32

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
from swlib import sw, MM, NOTHING  # noqa: E402
import build_assembly as BA  # noqa: E402
from build_assembly import comp_of, fix, N  # noqa: E402

PARTS = BA.PARTS
REND = os.path.join(ROOT, "docs", "renders")
ORIG_PARTS = os.path.abspath(os.path.join(ROOT, "..", "cad", "parts"))
ORIG_LEG = os.path.join(ORIG_PARTS, "leg_3dof.SLDASM")
VIEWS = (("*Trimetric", "trimetric"), ("*Front", "side"), ("*Right", "front"), ("*Bottom", "bottom"))


def var():
    return w32.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)


def open_doc(path, typ, readonly=False):
    return sw().OpenDoc6(path, typ, 1 | (2 if readonly else 0), "", var(), var())     # silent (+ read-only)


def shoot(doc, name, views=VIEWS):
    out = []
    for v, tag in views:
        doc.ShowNamedView2(v, -1)
        doc.ViewZoomtofit2()
        f = os.path.join(REND, f"{name}_{tag}.png")
        doc.SaveAs3(f, 0, 2)                    # swSaveAsOptions_Copy: exports an image, the document is untouched
        out.append(f)
    return out


def side_by_side(a, b, out, labels=("Leg Day (original)", "Snow leg")):
    from PIL import Image, ImageDraw
    ia, ib = Image.open(a).convert("RGB"), Image.open(b).convert("RGB")
    h = max(ia.height, ib.height)
    im = Image.new("RGB", (ia.width + ib.width, h + 30), "white")
    im.paste(ia, (0, 30)); im.paste(ib, (ia.width, 30))
    d = ImageDraw.Draw(im)
    d.text((10, 8), labels[0], fill=(30, 30, 30)); d.text((ia.width + 10, 8), labels[1], fill=(30, 30, 30))
    im.save(out)


def set_readonly(paths, on):
    for p in paths:
        mode = os.stat(p).st_mode
        os.chmod(p, (mode & ~stat.S_IWRITE) if on else (mode | stat.S_IWRITE))


def main():
    os.makedirs(REND, exist_ok=True)
    sw().CloseAllDocuments(True)
    snow_leg = open_doc(os.path.join(PARTS, N("leg_3dof") + ".SLDASM"), 2)
    snow_imgs = shoot(snow_leg, "snow_leg_3dof")
    sw().CloseAllDocuments(True)
    foot = open_doc(os.path.join(PARTS, N("foot_snowshoe") + ".SLDPRT"), 1)
    shoot(foot, "snow_foot", (("*Bottom", "sole"), ("*Isometric", "closeup"), ("*Front", "side")))
    sw().CloseAllDocuments(True)
    walk = os.path.join(PARTS, N("walk_anim") + ".SLDASM")
    if os.path.exists(walk):
        d = open_doc(walk, 2)
        shoot(d, "snow_stance", (("*Trimetric", "trimetric"), ("*Front", "side")))
        sw().CloseAllDocuments(True)
    # ---------------- original, read-only
    orig_files = [os.path.join(ORIG_PARTS, f) for f in os.listdir(ORIG_PARTS) if f.upper().endswith((".SLDPRT", ".SLDASM"))]
    set_readonly(orig_files, True)
    try:
        o = open_doc(ORIG_LEG, 2, readonly=True)
        orig_imgs = shoot(o, "original_leg_3dof")
        sw().CloseAllDocuments(True)
        for (v, tag), a, b in zip(VIEWS, orig_imgs, snow_imgs):
            side_by_side(a, b, os.path.join(REND, f"compare_{tag}.png"))
        # ---------------- compare_legs.SLDASM: both legs side by side, both fixed; save only this document
        tpl = sw().GetUserPreferenceStringValue(9)
        asm = sw().NewDocument(tpl, 0, 0, 0)
        title = asm.GetTitle
        comps = []
        for path, x in ((ORIG_LEG, -125.0), (os.path.join(PARTS, N("leg_3dof") + ".SLDASM"), 125.0)):
            open_doc(path, 2, readonly=(path == ORIG_LEG))
            sw().ActivateDoc3(title, False, 0, var())
            comps.append(asm.AddComponent5(path, 0, "", False, "", x * MM, 0.0, 0.0))
        for c in comps:
            fix(asm, c)
        asm.ForceRebuild3(False)
        out = os.path.join(ROOT, "compare_legs.SLDASM")
        if os.path.exists(out):
            os.remove(out)
        ok = asm.SaveAs3(out, 0, 1)             # silent; the referenced originals are read-only on disk
        asm.ShowNamedView2("*Trimetric", -1)
        asm.ViewZoomtofit2()
        asm.SaveAs3(os.path.join(REND, "compare_legs_trimetric.png"), 0, 2)
        asm.ShowNamedView2("*Front", -1)
        asm.ViewZoomtofit2()
        asm.SaveAs3(os.path.join(REND, "compare_legs_side.png"), 0, 2)
        deps = sw().GetDocumentDependencies2(out, True, True, False)
        print("compare_legs saved:", ok, "references:", len(deps) // 2 if deps else 0)
        sw().CloseAllDocuments(True)
    finally:
        set_readonly(orig_files, False)
    print("renders written to", REND)


if __name__ == "__main__":
    main()
