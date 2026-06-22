"""Mesh-native (trimesh + manifold3d) reimplementation of the default WheelGeometry path.

Proof-of-concept twin: reproduces WheelGeometry(radius, width) with all-default specs
using only trimesh primitives + manifold3d booleans (no CadQuery). Mirrors the branch
logic in sdk/_core/v0/_mesh/wheels.py for the default (flat hub, disc spokes, round bore).
"""

from __future__ import annotations

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

_ROT_Z_TO_X = rotation_matrix(np.pi / 2.0, [0.0, 1.0, 0.0])  # map local +Z axis onto +X


def _annulus_x(
    r_inner: float, r_outer: float, width: float, x_center: float = 0.0
) -> trimesh.Trimesh:
    m = trimesh.creation.annulus(r_min=r_inner, r_max=r_outer, height=width, sections=128)
    m.apply_transform(_ROT_Z_TO_X)
    m.apply_translation([x_center, 0.0, 0.0])
    return m


def _cylinder_x(radius: float, length: float, x_center: float = 0.0) -> trimesh.Trimesh:
    m = trimesh.creation.cylinder(radius=radius, height=length, sections=128)
    m.apply_transform(_ROT_Z_TO_X)
    m.apply_translation([x_center, 0.0, 0.0])
    return m


def build_default_wheel(radius: float, width: float) -> trimesh.Trimesh:
    """Mirror WheelGeometry(radius, width) with default specs, mesh-native."""
    radius, width = float(radius), float(width)
    # --- rim (WheelRim defaults: no flange, no bead seat) ---
    rim_outer = radius
    rim_inner = radius * 0.68
    rim = _annulus_x(rim_inner, rim_outer, width)

    # --- hub (WheelHub: radius=r*0.18, width=w*0.55, cap_style="flat") ---
    hub_radius = radius * 0.18
    hub_width = width * 0.55
    hub = _cylinder_x(hub_radius, hub_width)

    # --- face discs (WheelFace defaults all 0; WheelSpokes style="disc") ---
    disc_thickness = max(0.0, width * 0.08, 0.002)
    face_outer = min(
        rim_outer - max(0.0, radius * 0.02), rim_inner + (rim_outer - rim_inner) * 0.28
    )
    face_outer = max(face_outer, hub_radius * 1.6)
    disc_inner = max(hub_radius * 0.78, hub_radius - max(disc_thickness * 0.35, 0.0012))
    front_x = width * 0.5 - disc_thickness * 0.5
    front_disc = _annulus_x(disc_inner, face_outer, disc_thickness, front_x)
    rear_disc = _annulus_x(disc_inner, face_outer, disc_thickness, -front_x)

    solid = trimesh.boolean.union([rim, hub, front_disc, rear_disc], engine="manifold")

    # --- bore (WheelBore round, diameter = max(r*0.18, 0.004)), cut through ---
    bore_d = max(radius * 0.18, 0.004)
    bore = _cylinder_x(bore_d * 0.5, width + 0.04)
    solid = trimesh.boolean.difference([solid, bore], engine="manifold")
    return solid


if __name__ == "__main__":
    import json
    from pathlib import Path

    from tools.mesh_twin.compare import compare, render_side_by_side

    base = np.load("artifacts/mesh_twin/baseline/wheel.npz")
    baseline = trimesh.Trimesh(vertices=base["vertices"], faces=base["faces"], process=False)
    candidate = build_default_wheel(0.30, 0.12)

    result = compare(baseline, candidate)
    out = Path("artifacts/mesh_twin")
    render_side_by_side(
        baseline,
        candidate,
        "WheelGeometry(0.30, 0.12) — CadQuery vs mesh-native",
        str(out / "wheel_twin.png"),
    )
    print(json.dumps(result, indent=2))
    print(f"\nTWIN VERDICT: {'PASS' if result['is_twin'] else 'FAIL'}")
