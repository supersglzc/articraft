---
title: 'A Parametric Bearing Pillow Block'
description: 'Base SDK example building a parametric bearing pillow block: a rectangular plate with a central bearing bore and four corner counterbored mounting holes, cut with native boolean geometry.'
tags:
  - sdk
  - base sdk
  - parametric
  - bearing
  - pillow block
  - counterbore
  - mounting plate
  - boolean difference
  - mesh geometry
---
# A Parametric Bearing Pillow Block

This base-SDK example reproduces the classic parametric bearing pillow block:
a rectangular plate carrying a central through-bore for a bearing and four
counterbored mounting holes at the inset corners. In the native SDK there is no
CadQuery workplane stack, so the same intent is expressed by cutting native
cylinder tools out of a `BoxGeometry` plate with `boolean_difference(...)`.

The construction logic mirrors the original: start with a `length x height x
thickness` plate, drill the bearing bore straight through the thickness, then
place four counterbored holes on the corners of a `(length - padding) x
(height - padding)` rectangle. Each counterbore is a narrow through clearance
hole plus a wider, shallow recess cut from the top face. Parameters at the top
of the script make every dimension adjustable.

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

# Parameters (meters). Mirrors the original (length, height, bearing_diam,
# thickness, padding) = (30, 40, 22, 10, 8) mm, plus the cbore hole/cbore/depth
# (2.4, 4.4, 2.1) mm, scaled to meters.
LENGTH = 0.030
HEIGHT = 0.040
THICKNESS = 0.010
BEARING_DIAM = 0.022
PADDING = 0.008

CBORE_HOLE_DIAM = 0.0024
CBORE_DIAM = 0.0044
CBORE_DEPTH = 0.0021

_SEGMENTS = 48
_EPS = 1.0e-4  # small overshoot so cutters fully clear the faces


def _pillow_block_geometry() -> "BoxGeometry":
    # Plate centered at the origin, thickness along Z.
    plate = BoxGeometry((LENGTH, HEIGHT, THICKNESS))

    # Central bearing bore: a full through-cylinder along Z.
    bore = CylinderGeometry(
        BEARING_DIAM / 2.0,
        THICKNESS + 2.0 * _EPS,
        radial_segments=_SEGMENTS,
    )
    solid = boolean_difference(plate, bore)

    # Four corner counterbores on a (length - padding) x (height - padding) rect.
    half_x = (LENGTH - PADDING) / 2.0
    half_y = (HEIGHT - PADDING) / 2.0
    top_z = THICKNESS / 2.0

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            cx = sx * half_x
            cy = sy * half_y

            # Clearance hole: thin through-cylinder.
            clearance = CylinderGeometry(
                CBORE_HOLE_DIAM / 2.0,
                THICKNESS + 2.0 * _EPS,
                radial_segments=_SEGMENTS,
            ).translate(cx, cy, 0.0)
            solid = boolean_difference(solid, clearance)

            # Counterbore recess: wider, shallow pocket from the top face.
            recess = CylinderGeometry(
                CBORE_DIAM / 2.0,
                CBORE_DEPTH + _EPS,
                radial_segments=_SEGMENTS,
            ).translate(cx, cy, top_z - CBORE_DEPTH / 2.0 + _EPS / 2.0)
            solid = boolean_difference(solid, recess)

    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bearing_pillow_block")
    steel = model.material("pillow_block_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    block = model.part("pillow_block")
    block.visual(
        mesh_from_geometry(_pillow_block_geometry(), "pillow_block"),
        material=steel,
        name="pillow_block_body",
    )
    block.inertial = Inertial.from_geometry(
        Box((LENGTH, HEIGHT, THICKNESS)),
        mass=0.12,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    block = object_model.get_part("pillow_block")
    ctx.check("block_part_present", block is not None, "Expected a pillow block part.")
    if block is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(block)
    ctx.check("block_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check(
        "block_length",
        abs(size[0] - LENGTH) <= 0.001,
        f"expected length={LENGTH}, size={size!r}",
    )
    ctx.check(
        "block_height",
        abs(size[1] - HEIGHT) <= 0.001,
        f"expected height={HEIGHT}, size={size!r}",
    )
    ctx.check(
        "block_thickness",
        abs(size[2] - THICKNESS) <= 0.001,
        f"expected thickness={THICKNESS}, size={size!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
