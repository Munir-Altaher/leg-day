"""Print preparation + support-free check + PrusaSlicer numbers for the SNOW LEG.

For every printed part:
  1. load the SolidWorks STL (assembly pose, cad/stl/assembly_pose/) and rotate it into its stated
     print orientation (the face listed in ORIENT goes on the bed), centre it on a 220 x 220 bed and
     write a binary STL to cad/stl/<name>.stl (print-oriented, as the brief asks);
  2. overhang check with supports off: facets facing downward (n_z < -cos 45°) that are not on the
     bed. Flat downward regions are bridges — reported with their span (smallest box side); sloped
     downward facets steeper than 45° are reported as overhang area;
  3. slice with PrusaSlicer 2.9 CLI (supports off) -> estimated print time and filament grams.
The original Leg Day STLs are read (never written) from ../cad/stl/ the same way for COMPARISON.md;
their oriented copies go to docs/verification/original_print/.
Writes analysis/out/print.md and analysis/out/print.json.
Run:  python analysis/print_check.py [--no-slice]
"""
import json
import math
import os
import re
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
import params as P  # noqa: E402

SNOW_STL = os.path.join(ROOT, "cad", "stl", "assembly_pose")
SNOW_OUT = os.path.join(ROOT, "cad", "stl")
ORIG_STL = os.path.abspath(os.path.join(ROOT, "..", "cad", "stl"))          # read-only
ORIG_OUT = os.path.join(ROOT, "docs", "verification", "original_print")
OUT = os.path.join(HERE, "out")
GCODE = os.path.join(ROOT, "docs", "verification", "gcode")
SLICER = r"C:\Program Files\Prusa3D\PrusaSlicer\prusa-slicer-console.exe"
SHIFT = 300.0

# bed-face outward normal (body frame) and slicer profile per part
# profile: (layer, perimeters, infill %, pattern, top/bottom layers)
STD = (0.2, 4, 40, "gyroid", 4)
SOLID = (0.16, 4, 100, "rectilinear", 4)
SNOW_PARTS = {
    "thigh_outer": ("+z", STD), "thigh_cover": ("-z", STD), "knee_pulley_60T": ("+z", (0.12, 3, 100, "rectilinear", 5)),
    "shank": ("-z", (0.2, 3, 15, "gyroid", 4)),                      # option (a): closed part, infill is the skeleton
    "foot_snowshoe": ("+y", (0.2, 2, 10, "gyroid", 2)),          # thin deck + lugs: 2 perimeters / skins, 10 % core
    "ankle_bracket_in": ("+z", (0.16, 3, 30, "gyroid", 3)), "ankle_bracket_out": ("-z", (0.16, 3, 30, "gyroid", 3)),
    "hip_shaft_adapter": ("-z", SOLID), "module_plate_inboard": ("-z", STD), "module_flange_back": ("-x", STD),
    "module_flange_front": ("+x", STD), "module_frame_outboard": ("+y", STD), "hip_shroud_out": ("+z", (0.2, 3, 15, "gyroid", 4)),
    "body_bracket_back": ("-x", STD), "body_bracket_front": ("+x", STD), "body_spine": ("-z", STD), "rig_plate": ("-z", STD),
    "coupon_fit_test": ("-z", (0.2, 4, 100, "rectilinear", 4)),
}
ORIG_PARTS = {
    "thigh_outer": ("+z", STD), "thigh_cover": ("-z", STD), "knee_pulley_60T": ("+z", (0.12, 3, 100, "rectilinear", 5)),
    "shank": ("-z", STD), "foot": ("-z", SOLID), "hip_shaft_adapter": ("-z", SOLID), "module_frame_inboard": ("-z", STD),
    "module_plate_outboard": ("-z", STD), "body_bracket_back": ("-x", STD), "body_bracket_front": ("+x", STD),
    "body_spine": ("-z", STD), "rig_plate": ("-z", STD), "coupon_fit_test": ("-z", (0.2, 4, 100, "rectilinear", 4)),
}


def read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [ln.split()[1:4] for ln in data.decode("ascii", "ignore").splitlines() if ln.strip().startswith("vertex")]
        tri = np.array(v, float).reshape(-1, 3, 3)
    else:
        n = struct.unpack("<I", data[80:84])[0]
        tri = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))["v"].reshape(-1, 3, 3).astype(float)
    if np.abs(tri).max() < 5:
        tri = tri * 1000.0
    return tri - SHIFT


def write_stl(path, tri):
    with open(path, "wb") as f:
        f.write(b"snow-leg print-oriented".ljust(80, b" "))
        f.write(struct.pack("<I", len(tri)))
        nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nrm /= np.maximum(np.linalg.norm(nrm, axis=1)[:, None], 1e-12)
        rec = np.zeros(len(tri), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        rec["n"] = nrm
        rec["v"] = tri.reshape(-1, 9)
        f.write(rec.tobytes())


def orient(tri, bed):
    """Rotate so the outward normal `bed` points to -Z; centre on the bed; min z = 0."""
    R = {"+z": np.diag([1.0, -1.0, -1.0]), "-z": np.eye(3),
         "+y": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float), "-y": np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float),
         "+x": np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float), "-x": np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float)}[bed]
    t = tri.reshape(-1, 3) @ R.T
    t -= [(t[:, 0].min() + t[:, 0].max()) / 2 - 110, (t[:, 1].min() + t[:, 1].max()) / 2 - 110, t[:, 2].min()]
    return t.reshape(-1, 3, 3)


def overhangs(tri):
    """Downward-facing facets off the bed: (bridge regions with spans, steep-overhang area mm²)."""
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    n = np.cross(e1, e2)
    area = np.linalg.norm(n, axis=1) / 2
    nz = n[:, 2] / np.maximum(2 * area, 1e-12)
    zc = tri[:, :, 2].mean(axis=1)
    off_bed = zc > 0.25
    flat = off_bed & (nz < -0.985)
    steep = off_bed & (nz < -math.cos(math.radians(46))) & ~flat      # 45-deg teardrop flanks are printable
    # bridges: group flat-down facets by z level and connectivity (shared rounded vertices)
    regions = []
    idx = np.where(flat)[0]
    if len(idx):
        keyz = np.round(zc[idx], 2)
        for z in np.unique(keyz):
            sel = idx[keyz == z]
            # union-find on shared vertices
            parent = {i: i for i in sel}

            def find(a):
                while parent[a] != a:
                    parent[a] = parent[parent[a]]
                    a = parent[a]
                return a
            vmap = {}
            for i in sel:
                for v in np.round(tri[i], 3):
                    k = tuple(v)
                    if k in vmap:
                        parent[find(i)] = find(vmap[k])
                    else:
                        vmap[k] = i
            groups = {}
            for i in sel:
                groups.setdefault(find(i), []).append(i)
            for g in groups.values():
                pts = tri[g].reshape(-1, 3)
                ext = pts.max(0) - pts.min(0)
                # boundary length: edges used by exactly one facet of the region
                ecount = {}
                for i in g:
                    v = [tuple(np.round(p, 3)) for p in tri[i]]
                    for a_, b_ in ((0, 1), (1, 2), (2, 0)):
                        k = tuple(sorted((v[a_], v[b_])))
                        ecount[k] = ecount.get(k, 0) + 1
                per = sum(math.dist(a_, b_) for (a_, b_), c in ecount.items() if c == 1)
                A = float(area[g].sum())
                eff = min(float(min(ext[0], ext[1])), 4 * A / per if per > 0 else 0.0)   # disc -> D, annulus -> 2 x width
                regions.append(dict(z=float(z), span=eff, box=[round(float(v), 1) for v in ext[:2]], area=A,
                                    centre=[round(float(v), 1) for v in pts.mean(0)]))
    return regions, float(area[steep].sum()), float(area.sum())


def slice_part(stl, prof, tag):
    os.makedirs(GCODE, exist_ok=True)
    out = os.path.join(GCODE, tag + ".gcode")
    layer, per, fill, pat, tb = prof
    cmd = [SLICER, "--export-gcode", "--output", out, "--layer-height", str(layer), "--first-layer-height", "0.2",
           "--perimeters", str(per), "--fill-density", f"{fill}%", "--fill-pattern", pat,
           "--top-solid-layers", str(tb), "--bottom-solid-layers", str(tb), "--no-support-material",
           "--nozzle-diameter", "0.4", "--extrusion-width", "0.45", "--filament-diameter", "1.75",
           "--filament-density", "1.24", "--bed-shape", "0x0,220x0,220x220,0x220", "--max-print-height", "250",
           "--temperature", "210", "--bed-temperature", "60", stl]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    log = (r.stdout or "") + (r.stderr or "")
    t = g = None
    if os.path.exists(out):
        txt = open(out, encoding="utf-8", errors="ignore").read()[-20000:]
        m = re.search(r"estimated printing time \(normal mode\) = ([^\n]+)", txt)
        t = m.group(1).strip() if m else None
        m = re.search(r"filament used \[g\] = ([0-9.]+)", txt)
        g = float(m.group(1)) if m else None
    warn = [ln.strip() for ln in log.splitlines()
            if re.search(r"warning|detected|stability|loose|collaps|floating|bed adhesion|empty layer", ln, re.I)]
    return t, g, warn


def minutes(t):
    if not t:
        return 0
    d = dict((u, int(v)) for v, u in re.findall(r"(\d+)([dhms])", t))
    return d.get("d", 0) * 1440 + d.get("h", 0) * 60 + d.get("m", 0) + d.get("s", 0) / 60


def run(parts, src, dst, prefix, do_slice, label):
    os.makedirs(dst, exist_ok=True)
    rows = {}
    for name, (bed, prof) in parts.items():
        path = os.path.join(src, prefix + name + ".STL")
        if not os.path.exists(path):
            continue
        tri = orient(read_stl(path), bed)
        out_stl = os.path.join(dst, prefix + name + ".stl")
        write_stl(out_stl, tri)
        ext = tri.reshape(-1, 3).max(0) - tri.reshape(-1, 3).min(0)
        regions, steep, total = overhangs(tri)
        max_span = max([r["span"] for r in regions], default=0.0)
        long_bridges = [r for r in regions if r["span"] > 10.0]
        for r_ in long_bridges:
            print("   long bridge", name, r_, flush=True)
        t = g = None
        warn = []
        if do_slice:
            t, g, warn = slice_part(out_stl, prof, f"{label}_{name}")
        rows[name] = dict(bed=bed, profile=prof, size=[round(v, 1) for v in ext], fits=bool(ext[0] <= 220 and ext[1] <= 220 and ext[2] <= 250),
                          bridges=len(regions), max_bridge_span=round(max_span, 1), long_bridges=len(long_bridges),
                          steep_overhang_mm2=round(steep, 1), time=t, minutes=round(minutes(t), 1), grams=g, slicer_warnings=warn[:5])
        print(label, name, rows[name], flush=True)
    return rows


def main():
    do_slice = "--no-slice" not in sys.argv
    snow = run(SNOW_PARTS, SNOW_STL, SNOW_OUT, P.FILE_PREFIX, do_slice, "snow")
    orig = run(ORIG_PARTS, ORIG_STL, ORIG_OUT, "", do_slice, "orig")
    json.dump(dict(snow=snow, original=orig), open(os.path.join(OUT, "print.json"), "w"), indent=1)
    L = ["# Print check — support-free + PrusaSlicer (analysis/print_check.py)\n",
         "Orientation = the listed face on the bed (body-frame outward normal). Supports OFF. Bridges = flat downward "
         "regions off the bed (hole tops, pocket roofs), span = smaller side; ≤ 10 mm is printable unsupported. Steep "
         "overhang = downward facets steeper than 45° (round holes printed sideways show up here). PrusaSlicer 2.9.6 CLI, "
         "0.4 nozzle, 0.45 lines, PLA 1.24 g/cm³.\n",
         "| part | on bed | size mm | fits | max bridge span mm | bridges > 10 mm | steep overhang mm² | layer / perim / infill | time | g |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    tot_m = tot_g = 0
    for n, r in snow.items():
        pr = r["profile"]
        L.append(f"| snow_{n} | {r['bed']} | {r['size']} | {'yes' if r['fits'] else 'NO'} | {r['max_bridge_span']} | {r['long_bridges']} | "
                 f"{r['steep_overhang_mm2']} | {pr[0]} / {pr[1]} / {pr[2]}% {pr[3]} | {r['time']} | {r['grams']} |")
        if n != "coupon_fit_test":
            tot_m += r["minutes"]
            tot_g += r["grams"] or 0
    L.append(f"\n**Snow leg printed parts (excl. coupon): {tot_m/60:.1f} h, {tot_g:.0f} g.**\n")
    L.append("## Original Leg Day parts, same method (for COMPARISON.md)\n")
    L.append("| part | time | g |\n|---|---|---|")
    om = og = 0
    for n, r in orig.items():
        L.append(f"| {n} | {r['time']} | {r['grams']} |")
        if n != "coupon_fit_test":
            om += r["minutes"]
            og += r["grams"] or 0
    L.append(f"\n**Original printed parts (excl. coupon): {om/60:.1f} h, {og:.0f} g.**\n")
    txt = "\n".join(L)
    open(os.path.join(OUT, "print.md"), "w", encoding="utf-8").write(txt)
    print(txt)


if __name__ == "__main__":
    main()
