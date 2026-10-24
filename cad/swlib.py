"""Tiny SolidWorks COM helper (Route A). Lengths passed in mm, converted to m.

A part is built from 2.5D "slabs": closed loops sketched on Front (XY),
Right (YZ) or Top (XZ) plane, extruded along the plane normal from n0 to n1
(model coordinates). Inner loops in the same sketch become through-holes.
Loop items:
  ("circle", cx, cy, r)
  ("path", [seg, ...])  seg = ("line", (u0,v0), (u1,v1)) | ("arc", (cu,cv), (u0,v0), (u1,v1), dir)  dir +1 CCW / -1 CW
                        | ("spline", [(u,v), ...])  (through-points spline)
(u, v) are plane coordinates: XY -> (x, y); YZ -> (z, y); XZ -> (x, z).
"""
import math
import os

import pythoncom
import win32com.client as w32

MM = 1e-3
NOTHING = w32.VARIANT(pythoncom.VT_DISPATCH, None)
SHIFT = 300.0     # mm, see Part._sketch_on
PLANES = {"XY": "Front Plane", "YZ": "Right Plane", "XZ": "Top Plane"}

_sw = None


def sw():
    global _sw
    if _sw is None:
        _sw = w32.Dispatch("SldWorks.Application")
        _sw.Visible = True
    return _sw


class Part:
    def __init__(self, name):
        self.name = name
        tpl = sw().GetUserPreferenceStringValue(8)          # swDefaultTemplatePart
        self.m = sw().NewDocument(tpl, 0, 0, 0)
        self.sm = self.m.SketchManager
        self.fm = self.m.FeatureManager
        self._planes = {}

    # -- SolidWorks will not reliably make planes at negative offsets through the API, so
    # every part is modelled in the body frame shifted by +SHIFT mm on all three axes
    # (all slab offsets then positive). Assemblies place each component at -SHIFT.
    def _sketch_on(self, plane, off):
        key = (plane, round(off, 4))
        self.m.ClearSelection2(True)
        if off < 1e-6:
            name = PLANES[plane]
        elif key in self._planes:
            name = self._planes[key]
        else:
            self.m.Extension.SelectByID2(PLANES[plane], "PLANE", 0, 0, 0, False, 0, NOTHING, 0)
            rp = self.fm.InsertRefPlane(8, off * MM, 0, 0, 0, 0)      # swRefPlaneReferenceConstraint_Distance
            if rp is None:
                raise RuntimeError(f"{self.name}: ref plane {plane}@{off} failed")
            name = rp.Name
            self._planes[key] = name
            self.m.ClearSelection2(True)
        ok = self.m.Extension.SelectByID2(name, "PLANE", 0, 0, 0, False, 0, NOTHING, 0)
        assert ok, f"cannot select {name}"
        self.sm.InsertSketch(True)
        self.sm.AddToDB = True
        # sketch (x, y) components of model u and v axes (standard SW plane orientation,
        # checked against part bounding boxes in the self-test)
        return {"XY": ((1, 0), (0, 1)), "YZ": ((-1, 0), (0, 1)), "XZ": ((1, 0), (0, -1))}[plane]

    def _draw(self, plane, off, loops):
        (ux, uy), (vx, vy) = self._sketch_on(plane, off)

        def P(u, v):
            u, v = u + SHIFT, v + SHIFT
            return (u * ux + v * vx) * MM, (u * uy + v * vy) * MM

        det = ux * vy - uy * vx            # -1 if the plane mapping mirrors -> arc direction flips
        for lp in loops:
            if lp[0] == "circle":
                x, y = P(lp[1], lp[2])
                self.sm.CreateCircleByRadius(x, y, 0, lp[3] * MM)
            else:
                for s in lp[1]:
                    if s[0] == "line":
                        x0, y0 = P(*s[1]); x1, y1 = P(*s[2])
                        self.sm.CreateLine(x0, y0, 0, x1, y1, 0)
                    elif s[0] == "spline":
                        xyz = []
                        for q in s[1]:
                            xyz += [*P(*q), 0.0]
                        if self.sm.CreateSpline2(w32.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, xyz), False) is None:
                            raise RuntimeError(f"{self.name}: spline failed")
                    else:
                        xc, yc = P(*s[1]); x0, y0 = P(*s[2]); x1, y1 = P(*s[3])
                        self.sm.CreateArc(xc, yc, 0, x0, y0, 0, x1, y1, 0, int(s[4] * (1 if det > 0 else -1)))
        self.sm.AddToDB = False

    def slab(self, plane, n0, n1, loops, cut=False):
        """Extrude (or cut) loops on `plane` between normal coordinates n0..n1 (body frame, mm).
        Plane normals: XY -> +Z, YZ -> +X, XZ -> +Y."""
        a, b = sorted((n0, n1))
        a += SHIFT
        b += SHIFT
        self._draw(plane, a, loops)
        depth = (b - a) * MM
        if not cut:
            f = self.fm.FeatureExtrusion3(True, False, False, 0, 0, depth, 0.0, False, False, False, False,
                                          0.0, 0.0, False, False, False, False, True, True, True,
                                          0, 0.0, False)
        else:
            f = self.fm.FeatureCut4(True, False, True, 0, 0, depth, 0.0, False, False, False, False,
                                    0.0, 0.0, False, False, False, False, False, True, True, True, True, False,
                                    0, 0.0, False, False)
        self.m.ClearSelection2(True)
        if f is None:
            raise RuntimeError(f"{self.name}: {'cut' if cut else 'extrude'} failed on {plane} {n0}..{n1}")
        return f

    def revolve_sphere(self, c, r, z):
        """Solid sphere centre (c[0], c[1], z): half-disc on an XY plane revolved about x = c[0]."""
        self._sketch_on("XY", z + SHIFT)
        cx, cy = (c[0] + SHIFT) * MM, (c[1] + SHIFT) * MM
        r *= MM
        self.sm.CreateArc(cx, cy, 0, cx, cy + r, 0, cx, cy - r, 0, -1)
        self.sm.CreateLine(cx, cy - r, 0, cx, cy + r, 0)
        self.sm.CreateCenterLine(cx, cy - 1.5 * r, 0, cx, cy + 1.5 * r, 0)
        self.sm.AddToDB = False
        f = self.fm.FeatureRevolve2(True, True, False, False, False, False, 0, 0, 2 * math.pi, 0.0,
                                    False, False, 0.0, 0.0, 0, 0.0, 0.0, True, True, True)
        self.m.ClearSelection2(True)
        if f is None:
            raise RuntimeError(f"{self.name}: revolve failed")
        return f

    def _ref_plane(self, plane, off):
        """Select (append) a plane parallel to `plane` at body-frame offset `off` (creates it once)."""
        a = off + SHIFT
        key = (plane, round(a, 4))
        if key not in self._planes:
            self.m.ClearSelection2(True)
            self.m.Extension.SelectByID2(PLANES[plane], "PLANE", 0, 0, 0, False, 0, NOTHING, 0)
            rp = self.fm.InsertRefPlane(8, a * MM, 0, 0, 0, 0)
            if rp is None:
                raise RuntimeError(f"{self.name}: ref plane {plane}@{off} failed")
            self._planes[key] = rp.Name
            self.m.ClearSelection2(True)
        return self._planes[key]

    def axis(self, name, along, a, b):
        """Named reference axis for mates. along='Z': line x=a, y=b; along='X': line y=a, z=b."""
        if along == "Z":
            p1, p2 = self._ref_plane("YZ", a), self._ref_plane("XZ", b)
        else:
            p1, p2 = self._ref_plane("XZ", a), self._ref_plane("XY", b)
        self.m.ClearSelection2(True)
        self.m.Extension.SelectByID2(p1, "PLANE", 0, 0, 0, False, 0, NOTHING, 0)
        self.m.Extension.SelectByID2(p2, "PLANE", 0, 0, 0, True, 0, NOTHING, 0)
        if not self.m.InsertAxis2(True):
            raise RuntimeError(f"{self.name}: axis {name} failed")
        f = self.m.FeatureByPositionReverse(0)
        f.Name = name
        self.m.ClearSelection2(True)

    def revolve_capsule(self, c, r, top, z, tilt=0.0):
        """Solid of revolution about the line through c: a cylinder of radius r from `top` mm above c down
        to c, closed by a hemisphere centred on c (the ball-foot dome), in the XY plane at z. tilt (deg)
        turns the axis CCW from +y (the axis then points (-sin, cos))."""
        self._sketch_on("XY", z + SHIFT)
        ct, st = math.cos(math.radians(tilt)), math.sin(math.radians(tilt))
        P = lambda u, v: ((c[0] + u * ct - v * st + SHIFT) * MM, (c[1] + u * st + v * ct + SHIFT) * MM)   # noqa: E731
        a, b, d, e = P(0, top), P(r, top), P(r, 0), P(0, -r)
        o = P(0, 0)
        self.sm.CreateLine(*a, 0, *b, 0)
        self.sm.CreateLine(*b, 0, *d, 0)
        self.sm.CreateArc(*o, 0, *d, 0, *e, 0, -1)
        self.sm.CreateLine(*e, 0, *a, 0)
        self.sm.CreateCenterLine(*P(0, -1.5 * r), 0, *P(0, top + 0.5 * r), 0)
        self.sm.AddToDB = False
        f = self.fm.FeatureRevolve2(True, True, False, False, False, False, 0, 0, 2 * math.pi, 0.0,
                                    False, False, 0.0, 0.0, 0, 0.0, 0.0, True, True, True)
        self.m.ClearSelection2(True)
        if f is None:
            raise RuntimeError(f"{self.name}: revolve failed")
        return f

    def rotate_z(self, c, deg):
        """Rotate the whole (single) body CCW by deg about the line parallel to Z through c = (x, y)
        (Move/Copy Body; the first rotation slot of InsertMoveCopyBody2 is the Z rotation)."""
        body = self.m.GetBodies2(0, False)[0]
        self.m.ClearSelection2(True)
        if not self.m.Extension.SelectByID2(body.Name, "SOLIDBODY", 0, 0, 0, False, 1, NOTHING, 0):
            raise RuntimeError(f"{self.name}: body select failed")
        f = self.fm.InsertMoveCopyBody2(0, 0, 0, 0, (c[0] + SHIFT) * MM, (c[1] + SHIFT) * MM, SHIFT * MM,
                                        math.radians(deg), 0, 0, False, 1)
        self.m.ClearSelection2(True)
        if f is None:
            raise RuntimeError(f"{self.name}: rotate failed")
        return f

    def cut(self, plane, n0, n1, loops):
        return self.slab(plane, n0, n1, loops, cut=True)

    def bbox(self):
        b = self.m.GetPartBox(True)
        return [round(v / MM - SHIFT, 2) for v in b]

    def volume_mm3(self):
        mp = self.m.Extension.CreateMassProperty
        return mp.Volume / MM ** 3

    def save(self, folder_parts, folder_stl):
        os.makedirs(folder_parts, exist_ok=True)
        os.makedirs(folder_stl, exist_ok=True)
        out = {}
        for ext, folder in ((".SLDPRT", folder_parts), (".STEP", folder_parts), (".STL", folder_stl)):
            p = os.path.join(folder, self.name + ext)
            ok = self.m.SaveAs3(p, 0, 2)
            out[ext] = (ok, p)
        return out

    def close(self):
        sw().CloseDoc(self.m.GetTitle)


# ---------------------------------------------------------------- 2D geometry helpers (mm)
def circle(c, r):
    return ("circle", c[0], c[1], r)


def stadium(c0, r0, c1, r1):
    """Closed outline of two circles joined by external tangents (CCW)."""
    (x0, y0), (x1, y1) = c0, c1
    d = math.hypot(x1 - x0, y1 - y0)
    ang = math.atan2(y1 - y0, x1 - x0)
    a = math.acos((r0 - r1) / d)
    p = lambda c, r, t: (c[0] + r * math.cos(t), c[1] + r * math.sin(t))
    t0a, t0b = ang + a, ang - a
    s = [("line", p(c0, r0, t0b), p(c1, r1, t0b)),
         ("arc", c1, p(c1, r1, t0b), p(c1, r1, t0a), 1),
         ("line", p(c1, r1, t0a), p(c0, r0, t0a)),
         ("arc", c0, p(c0, r0, t0a), p(c0, r0, t0b), 1)]
    return ("path", s)


def rounded_poly(pts, r):
    """Polygon (CCW or CW) with every corner filleted at radius r (in-plane)."""
    n = len(pts)
    area = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1] for i in range(n))
    ccw = area > 0
    corners = []
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        a = (p0[0] - p1[0], p0[1] - p1[1]); b = (p2[0] - p1[0], p2[1] - p1[1])
        la, lb = math.hypot(*a), math.hypot(*b)
        a = (a[0] / la, a[1] / la); b = (b[0] / lb, b[1] / lb)
        th = math.acos(max(-1, min(1, a[0] * b[0] + a[1] * b[1])))
        t = r / math.tan(th / 2)
        bis = (a[0] + b[0], a[1] + b[1]); lbis = math.hypot(*bis)
        h = r / math.sin(th / 2)
        c = (p1[0] + bis[0] / lbis * h, p1[1] + bis[1] / lbis * h)
        corners.append(((p1[0] + a[0] * t, p1[1] + a[1] * t), (p1[0] + b[0] * t, p1[1] + b[1] * t), c))
    segs = []
    for i in range(n):
        s_in, s_out, c = corners[i]
        segs.append(("arc", c, s_in, s_out, 1 if ccw else -1))
        segs.append(("line", s_out, corners[(i + 1) % n][0]))
    return ("path", segs)


def rect(x0, y0, x1, y1):
    return ("path", [("line", (x0, y0), (x1, y0)), ("line", (x1, y0), (x1, y1)),
                     ("line", (x1, y1), (x0, y1)), ("line", (x0, y1), (x0, y0))])


def slot(c0, c1, r):
    return stadium(c0, r, c1, r)


def bolt_circle(c, pcd, n, d, phase=90.0):
    return [circle((c[0] + pcd / 2 * math.cos(math.radians(phase + 360 * i / n)),
                    c[1] + pcd / 2 * math.sin(math.radians(phase + 360 * i / n))), d / 2) for i in range(n)]


def teardrop(c, r, d=(0.0, 1.0)):
    """Round hole with a 45-deg pointed top in direction d (print 'up'): no unsupported roof."""
    dx, dy = d
    L = math.hypot(dx, dy)
    dx, dy = dx / L, dy / L
    ang = math.atan2(dy, dx)
    a0, a1 = ang + math.radians(45), ang - math.radians(45)
    A = (c[0] + r * math.cos(a0), c[1] + r * math.sin(a0))
    B = (c[0] + r * math.cos(a1), c[1] + r * math.sin(a1))
    apex = (c[0] + r * math.sqrt(2) * dx, c[1] + r * math.sqrt(2) * dy)
    return ("path", [("arc", c, A, B, 1), ("line", B, apex), ("line", apex, A)])
