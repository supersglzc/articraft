---
title: 'BGA Package'
description: 'A generic ball grid array package authored natively, with a spherical index mark cut into one top corner using a boolean difference.'
tags:
  - sdk
  - base sdk
  - bga
  - package
  - smd
  - electronics
  - boolean difference
  - mesh geometry
---
# BGA Package

This base-SDK example reproduces a generic ball grid array (BGA) package: a thin
rectangular body with a small spherical index mark cut into one top corner so the
package orientation is identifiable. The body is a `BoxGeometry` and the index
mark is a `SphereGeometry` removed with `boolean_difference`. Dimensions follow
the original 20 mm x 20 mm x 1 mm part, expressed in meters.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    Inertial,
    MeshGeometry,
    SphereGeometry,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

LENGTH = 0.020
WIDTH = 0.020
HEIGHT = 0.001

INDEX_MARK_RADIUS = 0.002
# Place the index mark near one corner, 1 mm in from each edge, matching the
# original part's corner-indexing convention.
INDEX_MARK_X = -((LENGTH / 2.0) - 0.001)
INDEX_MARK_Y = -((WIDTH / 2.0) - 0.001)
# Sit the sphere just above the top face so only a shallow spherical dimple is
# carved into the surface (the original uses a 0.93 * radius elevation).
INDEX_MARK_Z = (HEIGHT / 2.0) + (INDEX_MARK_RADIUS * 0.93)


def _weld(geometry: MeshGeometry, *, tol: float = 1e-7) -> MeshGeometry:
    """Merge coincident vertices so the mesh is a watertight manifold.

    Revolved primitives such as ``SphereGeometry`` emit duplicated pole and
    seam vertices, which leave the surface non-manifold for boolean operations.
    Welding by quantized position closes those seams.
    """

    welded = MeshGeometry()
    index: dict[tuple[int, int, int], int] = {}
    remap: list[int] = []
    for x, y, z in geometry.vertices:
        key = (round(x / tol), round(y / tol), round(z / tol))
        if key not in index:
            index[key] = welded.add_vertex(x, y, z)
        remap.append(index[key])
    for a, b, c in geometry.faces:
        a, b, c = remap[a], remap[b], remap[c]
        if a == b or b == c or a == c:
            continue
        welded.add_face(a, b, c)
    return welded


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bga_package")
    body_finish = model.material("bga_body_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = BoxGeometry((LENGTH, WIDTH, HEIGHT))

    index_mark = _weld(
        SphereGeometry(INDEX_MARK_RADIUS, width_segments=32, height_segments=24)
    )
    index_mark.translate(INDEX_MARK_X, INDEX_MARK_Y, INDEX_MARK_Z)

    package = boolean_difference(body, index_mark)

    part = model.part("package")
    part.visual(
        mesh_from_geometry(package, "bga_package_body"),
        material=body_finish,
        name="package_body",
    )
    part.inertial = Inertial.from_geometry(
        Box((LENGTH, WIDTH, HEIGHT)),
        mass=0.0015,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    part = object_model.get_part("package")
    ctx.check("package_part_present", part is not None, "Expected a package part.")
    if part is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(part)
    ctx.check("package_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("package_length", 0.018 <= size[0] <= 0.022, f"size={size!r}")
    ctx.check("package_width", 0.018 <= size[1] <= 0.022, f"size={size!r}")
    ctx.check("package_thin", size[2] <= 0.0015, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
