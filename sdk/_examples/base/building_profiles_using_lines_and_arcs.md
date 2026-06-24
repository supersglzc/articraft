---
title: 'Building Profiles Using Lines and Arcs'
description: 'Base SDK example that builds a prismatic solid by composing a closed 2D profile from straight lines and a three-point arc, then extruding it with ExtrudeGeometry.'
tags:
  - sdk
  - base sdk
  - profiles
  - lines
  - arcs
  - extrude
  - prismatic
  - mesh geometry
---
# Building Profiles Using Lines and Arcs

Sometimes you need to build a complex outline from straight runs and curved
sections, then extrude it into a prismatic solid. In the native SDK you build a
closed 2D profile as an ordered list of `(x, y)` points and hand it to
`ExtrudeGeometry`. Straight segments are just consecutive points; an arc is a
short run of sampled points along the curve.

This example reproduces the classic "lines and arcs" outline: from the origin a
line runs out along `+x`, a second line runs up along `+y`, then a three-point
arc curves back over the top to the upper-left corner, and the profile closes
along the left edge. The closed loop is extruded into a thin prismatic plate.
The shape is scaled to a small real-world plate (roughly 0.20 x 0.15 m,
0.025 m thick).

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ExtrudeGeometry,
    Inertial,
    Box,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# Scale the unit-ish CadQuery outline (about 2.0 x 1.5) down to meters.
SCALE = 0.10
THICKNESS = 0.025


def _three_point_arc(
    start: tuple[float, float],
    via: tuple[float, float],
    end: tuple[float, float],
    *,
    segments: int = 24,
) -> list[tuple[float, float]]:
    """Sample points along the circular arc through three 2D points.

    Returns interior + end points (the start point is assumed already present
    in the profile). Falls back to a straight chord if the points are collinear.
    """
    (x1, y1), (x2, y2), (x3, y3) = start, via, end
    # Circumcenter of the three points.
    d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-12:
        return [end]
    ux = (
        (x1 * x1 + y1 * y1) * (y2 - y3)
        + (x2 * x2 + y2 * y2) * (y3 - y1)
        + (x3 * x3 + y3 * y3) * (y1 - y2)
    ) / d
    uy = (
        (x1 * x1 + y1 * y1) * (x3 - x2)
        + (x2 * x2 + y2 * y2) * (x1 - x3)
        + (x3 * x3 + y3 * y3) * (x2 - x1)
    ) / d
    cx, cy = ux, uy
    r = math.hypot(x1 - cx, y1 - cy)

    a_start = math.atan2(y1 - cy, x1 - cx)
    a_via = math.atan2(y2 - cy, x2 - cx)
    a_end = math.atan2(y3 - cy, x3 - cx)

    # Choose sweep direction so the arc passes through the via point.
    def _norm(a: float) -> float:
        while a < 0:
            a += 2.0 * math.pi
        while a >= 2.0 * math.pi:
            a -= 2.0 * math.pi
        return a

    sweep_ccw = _norm(a_end - a_start)
    via_ccw = _norm(a_via - a_start)
    if via_ccw <= sweep_ccw:
        total = sweep_ccw  # counter-clockwise
        sign = 1.0
    else:
        total = 2.0 * math.pi - sweep_ccw  # clockwise
        sign = -1.0

    pts: list[tuple[float, float]] = []
    for i in range(1, segments + 1):
        t = total * (i / segments)
        a = a_start + sign * t
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _plate_profile() -> list[tuple[float, float]]:
    # CadQuery: lineTo(2,0).lineTo(2,1).threePointArc((1,1.5),(0,1)).close()
    profile: list[tuple[float, float]] = [(0.0, 0.0)]  # current point at origin
    profile.append((2.0, 0.0))  # lineTo(2, 0)
    profile.append((2.0, 1.0))  # lineTo(2, 1)
    # threePointArc: from (2,1) through (1,1.5) to (0,1)
    profile.extend(_three_point_arc((2.0, 1.0), (1.0, 1.5), (0.0, 1.0)))
    # close() returns to the start point along the left edge.
    # Center the loop on the origin and scale to meters.
    cx = sum(p[0] for p in profile) / len(profile)
    cy = sum(p[1] for p in profile) / len(profile)
    return [((p[0] - cx) * SCALE, (p[1] - cy) * SCALE) for p in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lines_and_arcs_plate")
    finish = model.material("plate_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(_plate_profile(), THICKNESS, cap=True),
            "lines_and_arcs_plate",
        ),
        material=finish,
        name="plate_body",
    )
    plate.inertial = Inertial.from_geometry(
        Box((2.0 * SCALE, 1.5 * SCALE, THICKNESS)),
        mass=0.8,
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
    # Outline spans 2.0 wide and ~1.5 tall before scaling; thickness is fixed.
    ctx.check("plate_width", 0.18 <= size[0] <= 0.22, f"size={size!r}")
    ctx.check("plate_height", 0.13 <= size[1] <= 0.17, f"size={size!r}")
    ctx.check("plate_thickness", 0.020 <= size[2] <= 0.030, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
