---
title: 'Making Counter-bored and Counter-sunk Holes'
description: 'Base SDK example reproducing the classic counterbore/countersink teaching plate: a rectangular box with four corner counterbored holes and four corner countersunk holes, cut with native boolean geometry.'
tags:
  - sdk
  - base sdk
  - counterbore
  - countersink
  - mounting plate
  - holes
  - boolean difference
  - mesh geometry
---
# Making Counter-bored and Counter-sunk Holes

Counterbored and countersunk holes are so common that CAD systems usually
provide one-step macros for them. In the original CadQuery example a `box` is
created, the top face is selected, a construction rectangle is placed, and a
counterbored hole is cut at each rectangle vertex with `cboreHole(...)`. There
is no workplane stack in the native SDK, so the same teaching intent is
expressed directly: cut native cylinder and cone tools out of a `BoxGeometry`
plate with `boolean_difference(...)`.

This example keeps the original construction logic. It starts from a
`length x width x thickness` plate and places fastener holes at the four
vertices of an inset construction rectangle. To show both classic features in
one part, the two short-side corners receive **counterbored** holes (a narrow
through clearance hole plus a wider, flat-bottomed recess from the top face),
and the two opposite corners receive **countersunk** holes (the same clearance
hole plus a conical chamfer that opens out toward the top face). Parameters at
the top of the script make every dimension adjustable, mirroring the
`(diameter, cboreDiameter, cboreDepth)` arguments of the original macro.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    ConeGeometry,
    CylinderGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

# Parameters (meters). The original CadQuery values were unitless
# (box 4 x 2 x 0.5; cboreHole 0.125 / 0.25 / 0.125). They are interpreted here
# as a realistic small steel plate and scaled to meters.
LENGTH = 0.200  # X
WIDTH = 0.100  # Y
THICKNESS = 0.025  # Z

# Construction rectangle the fastener holes sit on (original: 3.5 x 1.5).
RECT_LENGTH = 0.175  # X
RECT_WIDTH = 0.075  # Y

# Fastener hole stack (original cboreHole 0.125 / 0.25 / 0.125).
HOLE_DIAM = 0.0125  # through clearance hole
CBORE_DIAM = 0.025  # wider counterbore recess
CBORE_DEPTH = 0.0125  # counterbore / countersink depth from the top face

_SEGMENTS = 48
_EPS = 1.0e-4  # small overshoot so cutters fully clear the faces


def _counterbore_tools(cx: float, cy: float) -> list["CylinderGeometry"]:
    """Through clearance hole plus a flat-bottomed counterbore recess."""
    top_z = THICKNESS / 2.0
    clearance = CylinderGeometry(
        HOLE_DIAM / 2.0,
        THICKNESS + 2.0 * _EPS,
        radial_segments=_SEGMENTS,
    ).translate(cx, cy, 0.0)
    recess = CylinderGeometry(
        CBORE_DIAM / 2.0,
        CBORE_DEPTH + _EPS,
        radial_segments=_SEGMENTS,
    ).translate(cx, cy, top_z - CBORE_DEPTH / 2.0 + _EPS / 2.0)
    return [clearance, recess]


def _countersink_tools(cx: float, cy: float):
    """Through clearance hole plus a conical chamfer opening to the top face."""
    top_z = THICKNESS / 2.0
    clearance = CylinderGeometry(
        HOLE_DIAM / 2.0,
        THICKNESS + 2.0 * _EPS,
        radial_segments=_SEGMENTS,
    ).translate(cx, cy, 0.0)
    # ConeGeometry is centered on Z with its wide base toward -Z and its apex
    # toward +Z. Flip it 180 degrees about X so the wide mouth opens upward to
    # the top face (where a flat-head screw seats) and the narrow end blends
    # into the clearance hole, giving a classic countersink chamfer.
    sink = (
        ConeGeometry(
            CBORE_DIAM / 2.0,
            CBORE_DEPTH + _EPS,
            radial_segments=_SEGMENTS,
        )
        .rotate_x(3.141592653589793)
        .translate(cx, cy, top_z - CBORE_DEPTH / 2.0 + _EPS / 2.0)
    )
    return [clearance, sink]


def _plate_geometry() -> "BoxGeometry":
    # Plate centered at the origin, thickness along Z.
    solid = BoxGeometry((LENGTH, WIDTH, THICKNESS))

    half_x = RECT_LENGTH / 2.0
    half_y = RECT_WIDTH / 2.0

    # The two -X corners get counterbores; the two +X corners get countersinks.
    for sy in (-1.0, 1.0):
        for tool in _counterbore_tools(-half_x, sy * half_y):
            solid = boolean_difference(solid, tool)
        for tool in _countersink_tools(half_x, sy * half_y):
            solid = boolean_difference(solid, tool)

    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbore_countersink_plate")
    steel = model.material("plate_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(_plate_geometry(), "plate"),
        material=steel,
        name="plate_body",
    )
    plate.inertial = Inertial.from_geometry(
        Box((LENGTH, WIDTH, THICKNESS)),
        mass=0.40,
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
