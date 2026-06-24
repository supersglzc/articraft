---
title: 'Defining an Edge with a Spline'
description: 'Base SDK example that builds a flat extruded plate whose top edge follows a spline curve through a set of control points, while the other edges stay straight.'
tags:
  - sdk
  - base sdk
  - spline
  - spline edge
  - extrude
  - profile
  - extruded plate
  - mesh geometry
---
# Defining an Edge with a Spline

This base-SDK example reproduces the classic "edge defined by a spline" idea: a
flat plate is extruded from a closed 2D profile whose bottom and right edges are
straight line segments, but whose top edge sweeps through a collection of points
as a smooth spline. This is the native equivalent of building a profile with two
`lineTo` calls plus a `spline(...)` through interior points and then extruding
it.

The profile is sampled with `sample_catmull_rom_spline_2d(...)` so the top edge
passes through every control point, then closed into a single CCW loop and given
thickness with `ExtrudeGeometry`. Useful for queries such as `spline edge`,
`spline profile`, `extrude spline`, and `complex edge profile`.

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
    sample_catmull_rom_spline_2d,
)

# Scale the original unit-less CadQuery sketch into realistic meters.
# The source profile spans roughly 3 x 1.75 units; treat one unit as 0.1 m so
# the plate is a ~0.3 m wide bracket-style blank.
SCALE = 0.1
THICKNESS = 0.05  # 0.5 units * SCALE


def _spline_edge_profile() -> list[tuple[float, float]]:
    """Closed CCW loop: straight bottom and right edges, spline-defined top edge.

    Mirrors the original sketch:
        start (0, 0) -> lineTo (3, 0) -> lineTo (3, 1)
        -> spline through interior points -> close back to (0, 0)
    """
    # Control points the top spline edge must pass through, from the right
    # (3, 1) corner back toward the left side, ending near (0, 1).
    spline_points = [
        (3.0, 1.0),
        (2.75, 1.5),
        (2.5, 1.75),
        (2.0, 1.5),
        (1.5, 1.0),
        (1.0, 1.25),
        (0.5, 1.0),
        (0.0, 1.0),
    ]
    spline_edge = sample_catmull_rom_spline_2d(
        spline_points,
        samples_per_segment=14,
        closed=False,
    )

    # Straight portion of the loop: bottom edge then right edge.
    loop: list[tuple[float, float]] = [
        (0.0, 0.0),  # start
        (3.0, 0.0),  # lineTo bottom-right
        (3.0, 1.0),  # lineTo up the right edge (spline starts here)
    ]
    # Append the spline samples, skipping the first point because it duplicates
    # the (3, 1) corner already in the loop.
    loop.extend(spline_edge[1:])
    # The spline already ends at (0, 1); closing back to (0, 0) is implicit.

    # Apply the meter scale to every point.
    return [(x * SCALE, y * SCALE) for (x, y) in loop]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spline_edge_plate")
    finish = model.material("plate_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    profile = _spline_edge_profile()
    plate_geom = ExtrudeGeometry.from_z0(profile, THICKNESS, cap=True)

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(plate_geom, "spline_edge_plate"),
        material=finish,
        name="plate_body",
    )
    plate.inertial = Inertial.from_geometry(
        Box((3.0 * SCALE, 1.75 * SCALE, THICKNESS)),
        mass=0.8,
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

    # Width follows the straight bottom edge: 3 units -> 0.30 m.
    ctx.check("plate_width", 0.28 <= size[0] <= 0.32, f"size={size!r}")
    # Height is driven by the spline crest near y=1.75 units -> ~0.175 m.
    ctx.check("plate_height", 0.16 <= size[1] <= 0.19, f"size={size!r}")
    # Extruded thickness.
    ctx.check("plate_thickness", 0.045 <= size[2] <= 0.055, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
