---
title: 'Rounding Corners with Fillet'
description: 'Base SDK example showing how to round the vertical corners of a plate, reproducing a filleted box by extruding a rounded-rectangle profile.'
tags:
  - sdk
  - base sdk
  - rounding
  - corners
  - fillet
  - rounded rect
  - plate
  - mesh geometry
---
# Rounding Corners with Fillet

This base-SDK example reproduces the classic "round all the vertical edges of a
plate" teaching shape. In a B-rep CAD kernel you would select the four vertical
edges of a box and apply a fillet. With native mesh geometry there is no edge
selector, so the equivalent move is to extrude a rounded-rectangle profile: the
corner radius of the profile becomes the vertical-edge fillet of the resulting
plate.

The plate here is a `0.3 x 0.3 x 0.05` m slab with all four vertical corners
rounded at a `0.0125` m radius (a meter-scale version of a `3 x 3 x 0.5` box
filleted at `0.125`). It is useful for queries such as `fillet corners`,
`rounded plate`, `rounded_rect_profile`, and `round vertical edges`.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    ExtrudeGeometry,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

PLATE_WIDTH = 0.30
PLATE_DEPTH = 0.30
PLATE_THICKNESS = 0.05
FILLET_RADIUS = 0.0125


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="filleted_plate")
    finish = model.material("plate_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    plate = model.part("plate")

    # Extruding a rounded-rectangle profile along Z gives a slab whose four
    # vertical edges are rounded. The profile corner radius is the fillet.
    profile = rounded_rect_profile(
        PLATE_WIDTH,
        PLATE_DEPTH,
        FILLET_RADIUS,
        corner_segments=8,
    )
    plate_geom = ExtrudeGeometry.centered(profile, PLATE_THICKNESS, cap=True)

    plate.visual(
        mesh_from_geometry(plate_geom, "filleted_plate"),
        material=finish,
        name="plate_body",
    )
    plate.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, PLATE_DEPTH, PLATE_THICKNESS)),
        mass=1.0,
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

    # The rounded profile preserves the full nominal footprint along its axes.
    ctx.check("plate_width", abs(size[0] - PLATE_WIDTH) <= 1e-3, f"size={size!r}")
    ctx.check("plate_depth", abs(size[1] - PLATE_DEPTH) <= 1e-3, f"size={size!r}")
    ctx.check("plate_thickness", abs(size[2] - PLATE_THICKNESS) <= 1e-3, f"size={size!r}")

    # The rounded corners pull material in: the extreme corner of a sharp box
    # would sit at (W/2, D/2). With a fillet the actual corner is recessed, so
    # the half-diagonal of the bounding box must exceed the rounded outline.
    corner_x = PLATE_WIDTH / 2.0
    corner_y = PLATE_DEPTH / 2.0
    rounded_corner_x = corner_x - FILLET_RADIUS
    ctx.check(
        "corner_rounded_inward",
        rounded_corner_x < corner_x,
        f"corner_x={corner_x!r} rounded={rounded_corner_x!r}",
    )

    return ctx.report()


object_model = build_object_model()
```
