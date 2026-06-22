"""Mesh-native fillet/chamfer for the dominant case: rounding the vertical edges of a
prism (CadQuery's `.edges("|Z").fillet(r)` / `.chamfer(d)`).

Approach: round/chamfer the 2D cross-section corners analytically, then extrude. This is
an exact twin of CadQuery's vertical-edge fillet — each rounded corner is a circular arc
tangent to both adjacent edges, which is precisely what a quarter-cylinder fillet produces.
No shapely required; caps are triangulated by a convex centroid fan.
"""

from __future__ import annotations

import numpy as np
import trimesh


def _corner_arc(prev_pt, vtx, next_pt, radius: float, kind: str, n: int = 16):
    """Return points replacing a single convex corner with a fillet arc or chamfer."""
    p, v, q = (np.asarray(x, float) for x in (prev_pt, vtx, next_pt))
    u_in = (p - v) / np.linalg.norm(p - v)  # toward previous vertex
    u_out = (q - v) / np.linalg.norm(q - v)  # toward next vertex
    cos_theta = float(np.clip(np.dot(u_in, u_out), -1.0, 1.0))
    half = np.arccos(cos_theta) / 2.0
    setback = radius / np.tan(half)  # distance from corner to tangent points
    t_in = v + u_in * setback
    t_out = v + u_out * setback
    if kind == "chamfer":
        return [t_in, t_out]
    # fillet: arc centre on the angle bisector
    bis = u_in + u_out
    bis /= np.linalg.norm(bis)
    centre = v + bis * (radius / np.sin(half))
    a0 = np.arctan2(t_in[1] - centre[1], t_in[0] - centre[0])
    a1 = np.arctan2(t_out[1] - centre[1], t_out[0] - centre[0])
    # take the short way around
    if a1 - a0 > np.pi:
        a1 -= 2 * np.pi
    elif a0 - a1 > np.pi:
        a1 += 2 * np.pi
    return [centre + radius * np.array([np.cos(a), np.sin(a)]) for a in np.linspace(a0, a1, n)]


def round_polygon_2d(points, radius: float, kind: str = "fillet"):
    """Round (or chamfer) every corner of a convex 2D polygon (CCW), radius in world units."""
    pts = [np.asarray(p, float) for p in points]
    n = len(pts)
    out = []
    for i in range(n):
        out.extend(_corner_arc(pts[(i - 1) % n], pts[i], pts[(i + 1) % n], radius, kind))
    return np.asarray(out)


def extrude_profile(profile_2d, thickness: float) -> trimesh.Trimesh:
    """Extrude a convex 2D profile to a centered prism (caps via centroid fan)."""
    poly = np.asarray(profile_2d, float)
    n = len(poly)
    h = thickness * 0.5
    centroid = poly.mean(axis=0)
    bottom = np.column_stack([poly, np.full(n, -h)])
    top = np.column_stack([poly, np.full(n, h)])
    cb = np.array([[centroid[0], centroid[1], -h]])
    ct = np.array([[centroid[0], centroid[1], h]])
    verts = np.vstack([bottom, top, cb, ct])
    ci_b, ci_t = 2 * n, 2 * n + 1
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.append([ci_b, j, i])  # bottom cap (faces down)
        faces.append([ci_t, n + i, n + j])  # top cap (faces up)
        faces.append([i, j, n + j])  # side
        faces.append([i, n + j, n + i])
    return trimesh.Trimesh(vertices=verts, faces=np.asarray(faces), process=True)


def rounded_box(
    width: float, height: float, thickness: float, radius: float, kind: str = "fillet"
) -> trimesh.Trimesh:
    """Box with vertical edges rounded/chamfered — twin of cq box.edges('|Z').fillet(r)."""
    w, h = width * 0.5, height * 0.5
    rect = [(-w, -h), (w, -h), (w, h), (-w, h)]  # CCW
    profile = round_polygon_2d(rect, radius, kind=kind)
    return extrude_profile(profile, thickness)


if __name__ == "__main__":
    import json
    from pathlib import Path

    # CadQuery baseline: a box with vertical edges filleted
    from sdk._dependencies import require_cadquery
    from tools.mesh_twin.compare import compare, render_side_by_side

    cq = require_cadquery(feature="fillet-baseline")
    W, H, T, R = 0.20, 0.12, 0.03, 0.02
    cq_shape = cq.Workplane("XY").box(W, H, T).edges("|Z").fillet(R)
    v, f = cq_shape.val().tessellate(0.0005, 0.1)
    baseline = trimesh.Trimesh(
        vertices=[(p.x, p.y, p.z) for p in v], faces=np.asarray(f), process=True
    )

    candidate = rounded_box(W, H, T, R, kind="fillet")
    result = compare(baseline, candidate)
    out = Path("artifacts/mesh_twin")
    out.mkdir(parents=True, exist_ok=True)
    render_side_by_side(
        baseline,
        candidate,
        f"Box.edges('|Z').fillet({R}) — CadQuery vs mesh-native",
        str(out / "fillet_twin.png"),
    )
    print(json.dumps(result, indent=2))
    print(f"\nTWIN VERDICT: {'PASS' if result['is_twin'] else 'FAIL'}")
