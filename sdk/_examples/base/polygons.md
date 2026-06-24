---
title: 'Polygons'
description: 'Base SDK example: a flat 3D-printed plate with two hexagonal through-holes cut with ExtrudeWithHolesGeometry. Polygonal holes print cleaner than small circles on firmware that does not correct for small hole sizes.'
tags:
  - sdk
  - base sdk
  - polygon
  - hexagon
  - extrude with holes
  - through hole
  - plate
  - 3d printing
  - mesh geometry
---
# Polygons

This base-SDK example reproduces the classic "polygons" teaching object: a thin
flat plate with two regular hexagonal holes cut all the way through. Polygonal
holes are useful on 3D printers whose firmware does not correct for small hole
sizes, because the flat-walled polygon prints closer to its nominal size than a
small circle.

The native build uses `ExtrudeWithHolesGeometry`: a rectangular outer profile
extruded to the plate thickness, with two hexagon hole profiles subtracted as
through-cuts. It is useful for queries such as `polygon`, `hexagon hole`,
`through hole`, `ExtrudeWithHolesGeometry`, and `flat plate`.

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

PLATE_WIDTH = 0.060
PLATE_DEPTH = 0.080
PLATE_THICKNESS = 0.005

HOLE_RADIUS = 0.010  # circumradius of each hexagon
HOLE_OFFSET_Y = 0.015  # +/- offset of the two holes along the plate length


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    hw = width / 2.0
    hd = depth / 2.0
    # Counter-clockwise outer loop in local XY.
    return [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]


def _hexagon_profile(
    radius: float, center: tuple[float, float]
) -> list[tuple[float, float]]:
    cx, cy = center
    points: list[tuple[float, float]] = []
    for i in range(6):
        angle = math.pi / 6.0 + i * (math.pi / 3.0)
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="polygons_plate")
    finish = model.material("printed_plate", rgba=(0.20, 0.45, 0.70, 1.0))

    plate = model.part("plate")

    outer = _rect_profile(PLATE_WIDTH, PLATE_DEPTH)
    holes = [
        _hexagon_profile(HOLE_RADIUS, (0.0, HOLE_OFFSET_Y)),
        _hexagon_profile(HOLE_RADIUS, (0.0, -HOLE_OFFSET_Y)),
    ]

    geometry = ExtrudeWithHolesGeometry(
        outer,
        holes,
        PLATE_THICKNESS,
        cap=True,
        center=True,
    )

    plate.visual(
        mesh_from_geometry(geometry, "polygons_plate"),
        material=finish,
        name="plate_shell",
    )
    plate.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, PLATE_DEPTH, PLATE_THICKNESS)),
        mass=0.05,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("plate")
    ctx.check("plate_part_present", plate is not None, "Expected a plate part.")
    if plate is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(plate)
    ctx.check("plate_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check(
        "plate_footprint",
        0.055 <= size[0] <= 0.065 and 0.075 <= size[1] <= 0.085,
        f"size={size!r}",
    )
    ctx.check("plate_thin", 0.004 <= size[2] <= 0.006, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
