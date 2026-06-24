---
title: 'Using Point Lists'
description: 'Base SDK example showing how to place several identical features by iterating over a list of (x, y) points, building a round base plate with four mounting posts.'
tags:
  - sdk
  - base sdk
  - point list
  - repeated features
  - pattern
  - base plate
  - mounting posts
  - boolean union
  - mesh geometry
---
# Using Point Lists

Sometimes you need to create a number of identical features at various
locations. Instead of hand-placing each one, keep the locations in a list of
`(x, y)` points and loop over it. This is the native-SDK equivalent of pushing a
point list onto a stack and operating on all of the points at once.

This example builds a round base plate and then stamps one cylindrical mounting
post at each point in `POST_POINTS`. Each post is built from the same
`CylinderGeometry`, translated to its point, and fused into the single base part
with `boolean_union`. Scale is a small fixture, roughly 0.20 m across.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Cylinder,
    CylinderGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# Base plate (the "make base" circle, extruded into a thin disk).
PLATE_RADIUS = 0.10
PLATE_THICKNESS = 0.0125

# Repeated post feature.
POST_RADIUS = 0.0125
POST_HEIGHT = 0.0125

# The point list: four locations where an identical post is placed.
RING = 0.075
POST_POINTS = [
    (RING, 0.0),
    (0.0, RING),
    (-RING, 0.0),
    (0.0, -RING),
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="point_list_post_plate")
    steel = model.material("plate_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    # Base disk centered on z=0, spanning [-t/2, +t/2].
    geometry = CylinderGeometry(PLATE_RADIUS, PLATE_THICKNESS, radial_segments=48)

    # Operate on every point in the list, stamping one post at each.
    post_top_z = PLATE_THICKNESS / 2.0 + POST_HEIGHT / 2.0
    for px, py in POST_POINTS:
        post = CylinderGeometry(POST_RADIUS, POST_HEIGHT, radial_segments=24)
        post.translate(px, py, post_top_z)
        geometry = boolean_union(geometry, post)

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(geometry, "point_list_post_plate"),
        material=steel,
        name="plate_shell",
    )
    plate.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATE_RADIUS, length=PLATE_THICKNESS),
        mass=0.45,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("plate")
    ctx.check("plate_present", plate is not None, "Expected a plate part.")
    if plate is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(plate)
    ctx.check("plate_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))

    # The plate diameter should govern the XY footprint.
    expected_diameter = 2.0 * PLATE_RADIUS
    ctx.check(
        "plate_footprint",
        abs(size[0] - expected_diameter) <= 0.003
        and abs(size[1] - expected_diameter) <= 0.003,
        f"size={size!r}",
    )

    # Total height = plate thickness + the posts standing proud on top.
    expected_height = PLATE_THICKNESS + POST_HEIGHT
    ctx.check(
        "plate_with_posts_height",
        abs(size[2] - expected_height) <= 0.002,
        f"size={size!r}",
    )

    # The point list defines four posts.
    ctx.check("four_points", len(POST_POINTS) == 4, f"points={POST_POINTS!r}")
    return ctx.report()


object_model = build_object_model()
```
