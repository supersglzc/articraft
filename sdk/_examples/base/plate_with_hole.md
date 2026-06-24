---
title: 'Plate with Hole'
description: 'Base SDK example reproducing the classic plate-with-hole teaching part: a rectangular box with a single centered through hole, cut with native boolean geometry.'
tags:
  - sdk
  - base sdk
  - plate
  - through hole
  - mounting plate
  - boolean difference
  - mesh geometry
---
# Plate with Hole

This is the canonical "rectangular box, but with a hole added" teaching part. In
the original CadQuery example a `box` is created, the top face (`>Z`) is selected,
a workplane is laid on it, and `hole(...)` cuts a through hole at the projected
origin, which lands at the center of the face. There is no workplane stack in the
native SDK, so the same teaching intent is expressed directly: cut a native
cylinder tool through the center of a `BoxGeometry` plate with
`boolean_difference(...)`.

The hole is centered on the plate (matching the projected-origin behaviour of the
original) and goes all the way through the thickness (the default through-hole
depth). Parameters at the top of the script make every dimension adjustable,
mirroring the `length / height / thickness / center_hole_dia` variables in the
original.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

# Parameters (meters). The original CadQuery values were unitless
# (box 80 x 60 x 10; center hole diameter 22). They are interpreted here as a
# realistic small steel plate and scaled to meters.
LENGTH = 0.080  # X
WIDTH = 0.060  # Y (original "height" of the box footprint)
THICKNESS = 0.010  # Z
CENTER_HOLE_DIAM = 0.022  # centered through hole

_SEGMENTS = 48
_EPS = 1.0e-4  # small overshoot so the cutter fully clears both faces


def _plate_geometry() -> "BoxGeometry":
    # Plate centered at the origin, thickness along Z.
    solid = BoxGeometry((LENGTH, WIDTH, THICKNESS))

    # A single clearance cylinder centered on the plate, drilled all the way
    # through the thickness with a tiny overshoot on each face.
    hole = CylinderGeometry(
        CENTER_HOLE_DIAM / 2.0,
        THICKNESS + 2.0 * _EPS,
        radial_segments=_SEGMENTS,
    )
    solid = boolean_difference(solid, hole)
    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plate_with_hole")
    steel = model.material("plate_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(_plate_geometry(), "plate"),
        material=steel,
        name="plate_body",
    )
    plate.inertial = Inertial.from_geometry(
        Box((LENGTH, WIDTH, THICKNESS)),
        mass=0.30,
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
        "plate_length",
        abs(size[0] - LENGTH) <= 0.001,
        f"expected length={LENGTH}, size={size!r}",
    )
    ctx.check(
        "plate_width",
        abs(size[1] - WIDTH) <= 0.001,
        f"expected width={WIDTH}, size={size!r}",
    )
    ctx.check(
        "plate_thickness",
        abs(size[2] - THICKNESS) <= 0.001,
        f"expected thickness={THICKNESS}, size={size!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
