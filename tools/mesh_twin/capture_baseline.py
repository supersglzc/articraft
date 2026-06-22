"""Capture CadQuery-backed baseline meshes for twin verification.

Run BEFORE rewriting any generator. Saves each unit's mesh (npz) + metrics (json)
so the mesh-native rewrite can be diffed against a frozen reference.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, ".")
import sdk  # noqa: E402
from tools.mesh_twin.compare import extract_mesh, metrics  # noqa: E402

OUT = Path("artifacts/mesh_twin/baseline")
OUT.mkdir(parents=True, exist_ok=True)


def _units():
    """(name, callable) for representative geometry units, best-effort defaults."""
    yield "prim_box", lambda: sdk.mesh_from_geometry(sdk.BoxGeometry((0.2, 0.3, 0.1)), "box")
    yield (
        "prim_cylinder",
        lambda: sdk.mesh_from_geometry(sdk.CylinderGeometry(radius=0.1, height=0.3), "cyl"),
    )
    yield "prim_sphere", lambda: sdk.mesh_from_geometry(sdk.SphereGeometry(radius=0.15), "sph")
    yield "wheel", lambda: sdk.WheelGeometry(radius=0.30, width=0.12)
    yield "fan_rotor", lambda: sdk.FanRotorGeometry(radius=0.18, hub_radius=0.05, blade_count=7)
    yield "knob", lambda: sdk.KnobGeometry(radius=0.04, height=0.05)
    yield "barrel_hinge", lambda: sdk.BarrelHingeGeometry(length=0.12, radius=0.012)
    yield "spur_gear", lambda: sdk.SpurGear(module=0.004, teeth_number=18, width=0.02)


def main():
    summary = {}
    for name, build in _units():
        try:
            mesh = extract_mesh(build())
            np.savez_compressed(OUT / f"{name}.npz", vertices=mesh.vertices, faces=mesh.faces)
            summary[name] = {"status": "ok", **metrics(mesh)}
            print(
                f"  OK   {name}: {summary[name]['vertices']} verts, watertight={summary[name]['watertight']}"
            )
        except Exception as exc:  # noqa: BLE001
            summary[name] = {"status": "error", "error": f"{type(exc).__name__}: {exc}"}
            print(f"  FAIL {name}: {summary[name]['error']}")
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))
    print(
        f"\nBaseline saved to {OUT}/ ({sum(1 for v in summary.values() if v.get('status') == 'ok')} ok)"
    )


if __name__ == "__main__":
    main()
