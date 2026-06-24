---
title: 'Making Lofts'
description: 'Base SDK example that lofts a solid transition from a circular bottom section to a small rectangular top section, mounted on a flat base plate, using section_loft and mesh geometry.'
tags:
  - sdk
  - base sdk
  - loft
  - section loft
  - circle to rectangle
  - transition
  - mesh geometry
---
# Making Lofts

A loft is a solid swept through a set of ordered cross-sections. This base-SDK
example reproduces the classic "circle to rectangle" loft: a flat base plate
carries a lofted transition body whose bottom section is a circle and whose top
section is a small rectangle offset above the plate. It is useful for queries
such as `loft`, `section_loft`, `circle to rectangle transition`, and
`lofted solid`.

The modeling patterns worth copying are:

- `section_loft(...)` plus `repair_loft(...)` for a clean solid through two
  ordered cross-sections.
- sampling both the circular and rectangular sections to the same point count so
  the loft corresponds cleanly section to section.
- `boolean_union(...)` to fuse the loft body to the base plate into one
  watertight visual.

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    BoxGeometry,
    Inertial,
    Box,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

# Real-world scale (meters). The original teaching loft is unitless; here it is a
# small benchtop part: a 100 mm square plate with a circle-to-rectangle loft
# rising 75 mm above the plate top.
PLATE_SIZE = 0.100
PLATE_THICK = 0.006

CIRCLE_RADIUS = 0.0375
TOP_WIDTH = 0.019
TOP_DEPTH = 0.0125
LOFT_HEIGHT = 0.075

SECTION_POINTS = 64


def _circle_loop(radius: float, z: float, count: int) -> list[tuple[float, float, float]]:
    loop: list[tuple[float, float, float]] = []
    for i in range(count):
        theta = (i / count) * math.tau
        loop.append((radius * math.cos(theta), radius * math.sin(theta), z))
    return loop


def _rect_loop(
    half_w: float, half_d: float, z: float, count: int
) -> list[tuple[float, float, float]]:
    # Sample the rectangle perimeter at the same parameter values as the circle so
    # the two sections correspond cleanly and the loft does not twist.
    loop: list[tuple[float, float, float]] = []
    for i in range(count):
        theta = (i / count) * math.tau
        c = math.cos(theta)
        s = math.sin(theta)
        # Project the unit-circle direction onto the rectangle boundary.
        scale = 1.0 / max(abs(c) / half_w, abs(s) / half_d)
        loop.append((c * scale, s * scale, z))
    return loop


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="circle_to_rect_loft")
    body_material = model.material("loft_body", rgba=(0.62, 0.64, 0.68, 1.0))

    part = model.part("loft_body")

    # Flat base plate, centered on the origin, top face at z = PLATE_THICK.
    plate = BoxGeometry((PLATE_SIZE, PLATE_SIZE, PLATE_THICK)).translate(
        0.0, 0.0, PLATE_THICK / 2.0
    )

    # Loft from the circular bottom section (on the plate top) up to the small
    # rectangular top section, LOFT_HEIGHT above the plate top.
    bottom = _circle_loop(CIRCLE_RADIUS, PLATE_THICK, SECTION_POINTS)
    top = _rect_loop(
        TOP_WIDTH / 2.0, TOP_DEPTH / 2.0, PLATE_THICK + LOFT_HEIGHT, SECTION_POINTS
    )
    loft = repair_loft(section_loft([bottom, top]))

    # Fuse the loft body to the plate so the part is one watertight solid.
    fused = boolean_union(plate, loft)

    part.visual(
        mesh_from_geometry(fused, "loft_body"),
        material=body_material,
        name="loft_body_shell",
    )
    part.inertial = Inertial.from_geometry(
        Box((PLATE_SIZE, PLATE_SIZE, PLATE_THICK + LOFT_HEIGHT)),
        mass=0.4,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    part = object_model.get_part("loft_body")
    ctx.check("loft_part_present", part is not None, "Expected a loft_body part.")
    if part is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(part)
    ctx.check("loft_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))

    # Footprint is set by the base plate (circle diameter is smaller than plate).
    ctx.check(
        "plate_footprint",
        0.095 <= size[0] <= 0.105 and 0.095 <= size[1] <= 0.105,
        f"size={size!r}",
    )
    # Total height is plate thickness plus the loft rise.
    expected_height = PLATE_THICK + LOFT_HEIGHT
    ctx.check(
        "total_height",
        abs(size[2] - expected_height) <= 0.004,
        f"size={size!r} expected_height={expected_height!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
