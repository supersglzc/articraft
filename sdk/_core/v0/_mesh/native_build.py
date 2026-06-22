"""Mesh-native construction helpers (trimesh/manifold3d-free at this layer).

Small, composable helpers used by the geometry generators in place of CadQuery. They
return :class:`MeshGeometry` and build on the existing native primitives + boolean ops,
so generators read as plain mesh construction with no CAD kernel.

What lives here (the CadQuery vocabulary the generators actually used):
- ``annulus`` / ``annulus_x``      tube (cq ``circle().circle().extrude()``)
- ``round_polygon_2d``             corner rounding/chamfer of a convex profile
- ``rounded_prism`` / ``rounded_box``   cq ``box.edges("|Z").fillet/chamfer``

Primitives (box/cylinder/extrude/revolve), booleans, and transforms already exist on
:class:`MeshGeometry`; import them directly rather than re-wrapping.
"""

from __future__ import annotations

from math import acos, pi, sin, tan
from typing import List, Sequence, Tuple

import numpy as np

from .booleans import boolean_difference
from .primitives import CylinderGeometry, ExtrudeGeometry, MeshGeometry

Vec2 = Tuple[float, float]

_EPS = 1.0e-6


def annulus(r_inner: float, r_outer: float, height: float, *, segments: int = 128) -> MeshGeometry:
    """Hollow tube along +Z centered at the origin (cq ``circle(r_o).circle(r_i).extrude``)."""
    if not (0.0 < r_inner < r_outer):
        raise ValueError("annulus requires 0 < r_inner < r_outer")
    outer = CylinderGeometry(r_outer, height, radial_segments=segments)
    # slightly taller inner cutter avoids coplanar caps that confuse the boolean kernel
    inner = CylinderGeometry(r_inner, height + 2.0 * _EPS, radial_segments=segments)
    return boolean_difference(outer, inner)


def annulus_x(
    r_inner: float, r_outer: float, width: float, x_center: float = 0.0, *, segments: int = 128
) -> MeshGeometry:
    """Tube aligned to local X (cq ``Workplane("YZ").circle().circle().extrude``)."""
    geom = annulus(r_inner, r_outer, width, segments=segments).rotate_y(pi / 2.0)
    return geom.translate(x_center, 0.0, 0.0) if x_center else geom


def prism_yz(profile: Sequence[Vec2], length: float, x_center: float = 0.0) -> MeshGeometry:
    """Extrude a 2D ``(y, z)`` profile along local X (cq ``Workplane("YZ").polyline(...).extrude``).

    ExtrudeGeometry builds the profile in XY extruded along Z. A 120 deg rotation about
    (1,1,1) cyclically maps old (x,y,z) -> (z,x,y), i.e. profile (u,v) -> (y=u, z=v) with
    the extrude axis on X — matching CadQuery's YZ-plane convention exactly.
    """
    geom = ExtrudeGeometry(profile, length).rotate((1.0, 1.0, 1.0), 2.0 * pi / 3.0)
    return geom.translate(x_center, 0.0, 0.0) if x_center else geom


def round_polygon_2d(
    points: Sequence[Vec2], radius: float, *, kind: str = "fillet", arc_segments: int = 16
) -> List[Vec2]:
    """Round (fillet) or bevel (chamfer) every corner of a convex CCW polygon.

    Each corner is replaced by an arc tangent to both edges — exactly what CadQuery's
    ``.edges("|Z").fillet(r)`` produces on a prism.
    """
    pts = [np.asarray(p, float) for p in points]
    n = len(pts)
    out: List[Vec2] = []
    for i in range(n):
        p, v, q = pts[(i - 1) % n], pts[i], pts[(i + 1) % n]
        u_in = (p - v) / np.linalg.norm(p - v)
        u_out = (q - v) / np.linalg.norm(q - v)
        half = acos(float(np.clip(np.dot(u_in, u_out), -1.0, 1.0))) / 2.0
        setback = radius / tan(half)
        t_in, t_out = v + u_in * setback, v + u_out * setback
        if kind == "chamfer":
            out.extend([tuple(t_in), tuple(t_out)])
            continue
        bis = u_in + u_out
        bis /= np.linalg.norm(bis)
        centre = v + bis * (radius / sin(half))
        a0 = np.arctan2(t_in[1] - centre[1], t_in[0] - centre[0])
        a1 = np.arctan2(t_out[1] - centre[1], t_out[0] - centre[0])
        if a1 - a0 > pi:
            a1 -= 2 * pi
        elif a0 - a1 > pi:
            a1 += 2 * pi
        out.extend(
            tuple(centre + radius * np.array([np.cos(a), np.sin(a)]))
            for a in np.linspace(a0, a1, arc_segments)
        )
    return out


def rounded_prism(
    profile: Sequence[Vec2], thickness: float, radius: float, *, kind: str = "fillet"
) -> MeshGeometry:
    """Extrude a convex profile with rounded/chamfered vertical edges, centered on Z."""
    rounded = round_polygon_2d(profile, radius, kind=kind) if radius > _EPS else list(profile)
    return ExtrudeGeometry(rounded, thickness)


def rounded_box(
    width: float, height: float, thickness: float, radius: float, *, kind: str = "fillet"
) -> MeshGeometry:
    """Box with vertical edges rounded/chamfered (cq ``box.edges("|Z").fillet(r)``)."""
    w, h = width * 0.5, height * 0.5
    rect = [(-w, -h), (w, -h), (w, h), (-w, h)]
    return rounded_prism(rect, thickness, radius, kind=kind)
