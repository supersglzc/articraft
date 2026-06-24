---
title: 'Simple Rectangular Plate'
description: 'Just about the simplest possible base-SDK example: a single rectangular plate built from BoxGeometry and emitted as a managed mesh.'
tags:
  - sdk
  - base sdk
  - examples
  - simple
  - rectangular
  - plate
  - box
  - mesh geometry
---
# Simple Rectangular Plate

This is the minimal native-SDK shape example: a single static rectangular plate.
It mirrors the classic "just about the simplest possible example, a rectangular
box" by building one `BoxGeometry` solid and exporting it with
`mesh_from_geometry`. There is no articulation; it is a single root part. Use it
as a reference for `BoxGeometry`, `mesh_from_geometry`, and the required script
contract.

The plate keeps the original 4 : 4 : 1 proportions at a realistic real-world
scale in meters (a 0.20 x 0.20 m plate, 0.05 m thick).

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

PLATE_LENGTH = 0.20
PLATE_WIDTH = 0.20
PLATE_THICKNESS = 0.05


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_rectangular_plate")
    finish = model.material("plate_steel", rgba=(0.62, 0.64, 0.66, 1.0))

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(
            BoxGeometry((PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)),
            "plate_body",
        ),
        # BoxGeometry is centered at the origin; lift it so the plate rests on z=0.
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS / 2.0)),
        material=finish,
        name="plate_body",
    )
    plate.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)),
        mass=1.5,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("plate")
    ctx.check("plate_part_present", plate is not None, "Expected a plate part.")
    if plate is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(plate)
    ctx.check("plate_aabb_present", aabb is not None, "Expected a world AABB for the plate.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("plate_length", abs(size[0] - PLATE_LENGTH) <= 1e-3, f"size={size!r}")
    ctx.check("plate_width", abs(size[1] - PLATE_WIDTH) <= 1e-3, f"size={size!r}")
    ctx.check("plate_thickness", abs(size[2] - PLATE_THICKNESS) <= 1e-3, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
