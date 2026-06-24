---
title: 'An Extruded Prismatic Solid'
description: 'Base SDK example showing how to build a prismatic solid by extruding a centered rectangular 2D profile along Z with ExtrudeGeometry, the native equivalent of drawing a rectangle on a workplane and extruding it.'
tags:
  - sdk
  - base sdk
  - extrude
  - extrusion
  - prismatic solid
  - rectangular profile
  - mesh geometry
---
# An Extruded Prismatic Solid

This base-SDK example reproduces the classic "extruded prismatic solid" tutorial:
a rectangular 2D profile is extruded along the Z axis to form a single prismatic
block. In the native SDK this is done directly with `ExtrudeGeometry`, which
takes a closed 2D profile in local XY and extrudes it a given height along Z.

The original tutorial drew a rectangle centered on a previous working point and
extruded it; the circle in that snippet only served as the centering reference,
so the resulting solid is purely the extruded rectangle. Here the profile is a
`rounded_rect_profile` centered at the origin, extruded with `center=True` so the
solid spans `z in [-height/2, +height/2]`. Useful for queries such as
`ExtrudeGeometry`, `extrude rectangle`, and `prismatic solid`.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)

# Keep the source proportions (0.5 x 0.75 base, 0.5 tall) at a realistic
# tabletop scale in meters.
WIDTH = 0.05
DEPTH = 0.075
HEIGHT = 0.05
CORNER_RADIUS = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extruded_prismatic_solid")
    finish = model.material("prism_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    # A closed, centered rectangular profile in local XY...
    profile = rounded_rect_profile(WIDTH, DEPTH, CORNER_RADIUS)

    # ...extruded along Z into a prismatic solid centered on the profile plane.
    solid = ExtrudeGeometry.centered(profile, HEIGHT, cap=True)

    block = model.part("prism")
    block.visual(
        mesh_from_geometry(solid, "prism_block"),
        material=finish,
        name="prism_solid",
    )
    block.inertial = Inertial.from_geometry(
        Box((WIDTH, DEPTH, HEIGHT)),
        mass=0.5,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    block = object_model.get_part("prism")
    ctx.check("prism_part_present", block is not None, "Expected a prism part.")
    if block is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(block)
    ctx.check("prism_aabb_present", aabb is not None, "Expected a world AABB for the prism.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # The extruded prismatic solid should match the rectangular footprint and
    # the extrusion height.
    ctx.check("prism_width", abs(size[0] - WIDTH) <= 0.002, f"size={size!r}")
    ctx.check("prism_depth", abs(size[1] - DEPTH) <= 0.002, f"size={size!r}")
    ctx.check("prism_height", abs(size[2] - HEIGHT) <= 0.002, f"size={size!r}")
    # Confirm the centered extrusion spans symmetrically about z=0.
    ctx.check(
        "prism_centered_z",
        abs(float(mins[2]) + float(maxs[2])) <= 0.002,
        f"z_span=({float(mins[2])!r}, {float(maxs[2])!r})",
    )
    return ctx.report()


object_model = build_object_model()
```
