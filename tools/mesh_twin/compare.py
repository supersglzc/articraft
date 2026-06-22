"""Mesh twin-verification harness.

Compares a baseline mesh (CadQuery-backed) against a candidate mesh (mesh-native)
and reports geometric-similarity metrics plus a side-by-side render so a human can
visually confirm the rewrite produces a faithful twin.

Usage (as a library):
    from tools.mesh_twin.compare import extract_mesh, compare, render_side_by_side
"""

from __future__ import annotations

import numpy as np
import trimesh


def extract_mesh(obj) -> trimesh.Trimesh:
    """Coerce an SDK geometry object / MeshGeometry / Mesh into a trimesh.Trimesh."""
    geom = getattr(obj, "geometry", obj)  # Mesh wraps a MeshGeometry under .geometry sometimes
    verts = getattr(geom, "vertices", None)
    faces = getattr(geom, "faces", None)
    if verts is None or faces is None:
        raise TypeError(f"Cannot extract vertices/faces from {type(obj).__name__}")
    return trimesh.Trimesh(
        vertices=np.asarray(verts, dtype=float),
        faces=np.asarray(faces, dtype=np.int64),
        process=False,
    )


def _robust_volume(mesh: trimesh.Trimesh) -> float:
    """Signed volume via the divergence theorem over triangles.

    Works for closed surfaces even when vertices aren't shared across faces — which is
    how the SDK's manifold->mesh boolean output is laid out, so trimesh's own
    ``mesh.volume`` (which needs strict watertightness) reports None on it.
    """
    tris = mesh.vertices[mesh.faces]
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    return float(abs(np.einsum("ij,ij->i", v0, np.cross(v1, v2)).sum()) / 6.0)


def metrics(mesh: trimesh.Trimesh) -> dict:
    return {
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "volume": _robust_volume(mesh),
        "area": float(mesh.area),
        "bbox": [round(float(d), 6) for d in mesh.extents],
        "centroid": [round(float(c), 6) for c in mesh.centroid],
    }


def _chamfer(a: trimesh.Trimesh, b: trimesh.Trimesh, n: int = 30000) -> dict:
    """Symmetric *point-to-surface* distance.

    Sample points on each mesh, measure their true distance to the OTHER mesh's
    surface (not to sampled points), so the result reflects real geometric
    deviation rather than sample spacing.
    """
    pa = a.sample(n)
    pb = b.sample(n)
    _, da, _ = b.nearest.on_surface(pa)
    _, db, _ = a.nearest.on_surface(pb)
    diag = float(np.linalg.norm(a.extents)) or 1.0
    return {
        "mean_dist": float((da.mean() + db.mean()) / 2),
        "max_dist": float(max(da.max(), db.max())),
        "mean_dist_pct_bbox": float((da.mean() + db.mean()) / 2 / diag * 100),
        "max_dist_pct_bbox": float(max(da.max(), db.max()) / diag * 100),
    }


def compare(baseline: trimesh.Trimesh, candidate: trimesh.Trimesh) -> dict:
    mb, mc = metrics(baseline), metrics(candidate)
    out = {"baseline": mb, "candidate": mc, "distance": _chamfer(baseline, candidate)}
    vol_b, vol_c = mb["volume"], mc["volume"]
    out["volume_err_pct"] = (
        abs(vol_b - vol_c) / abs(vol_b) * 100
        if vol_b not in (None, 0) and vol_c is not None
        else None
    )
    # twin verdict: surfaces agree to <1% of bbox diagonal and volume to <2%
    out["is_twin"] = out["distance"]["max_dist_pct_bbox"] < 1.0 and (
        out["volume_err_pct"] is None or out["volume_err_pct"] < 2.0
    )
    return out


def render_side_by_side(
    baseline: trimesh.Trimesh, candidate: trimesh.Trimesh, title: str, out_png: str
):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(15, 5))
    panels = [
        ("CadQuery (baseline)", baseline, "#4C72B0"),
        ("trimesh+manifold3d (new)", candidate, "#C44E52"),
        ("overlay", None, None),
    ]
    for i, (label, mesh, color) in enumerate(panels):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        if label == "overlay":
            for m, c, a in ((baseline, "#4C72B0", 0.45), (candidate, "#C44E52", 0.45)):
                ax.add_collection3d(
                    Poly3DCollection(m.vertices[m.faces], alpha=a, facecolor=c, edgecolor="none")
                )
            ref = baseline
        else:
            ax.add_collection3d(
                Poly3DCollection(
                    mesh.vertices[mesh.faces],
                    alpha=0.9,
                    facecolor=color,
                    edgecolor="k",
                    linewidths=0.05,
                )
            )
            ref = mesh
        b = ref.bounds
        ax.set_xlim(b[0][0], b[1][0])
        ax.set_ylim(b[0][1], b[1][1])
        ax.set_zlim(b[0][2], b[1][2])
        ax.set_box_aspect(tuple(float(e) for e in ref.extents))
        ax.set_title(label, fontsize=10)
        ax.set_axis_off()
    fig.suptitle(title, fontsize=12)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return out_png
