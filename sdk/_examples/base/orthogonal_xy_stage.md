---
title: 'Orthogonal XY Stage'
description: 'Base SDK precision XY stage with a grounded base carrying X rails, an X carriage with its own Y rails, and a Y plate on top, driven by two stacked orthogonal prismatic joints.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - xy stage
  - precision stage
  - linear stage
  - cartesian stage
  - prismatic
  - prismatic articulation
  - linear rails
  - carriage
  - stacked stage
  - orthogonal motion
  - motion limits
  - box geometry
  - cylinder geometry
---
# Orthogonal XY Stage

This base-SDK example is a faithful reference for a precision XY positioning
stage built from two stacked, orthogonal linear axes. It is useful for queries
such as `xy stage`, `linear stage`, `precision stage`, `cartesian stage`,
`prismatic articulation`, `linear rails`, `carriage`, and `stacked stage`.

The mechanism is intentionally split into three parts that mirror a real stage:

- a grounded `base` plate that also carries the lower (X-axis) rail pair,
- an `x_stage` carriage that rides the X rails and carries its own upper
  (Y-axis) rail pair, and
- a `y_stage` plate that rides the Y rails on top.

Motion is two `PRISMATIC` joints. `base_to_x` slides the X carriage along world
`+X`, and `x_to_y` slides the Y plate along the carriage-local `+Y`. The rails
are native `CylinderGeometry` runs and the plates/carriage bodies are
`BoxGeometry`, all exported as mesh-backed visuals. The stacked carriage and
plate ride above their rails with a small intentional embedment around the rail
diameter, declared with `allow_overlap(...)`.

```python
from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# Base plate footprint and rail layout (meters).
BASE_L = 0.220
BASE_W = 0.180
BASE_T = 0.020

# Lower (X-axis) rails run along X, spaced apart in Y, sitting on top of the base.
X_RAIL_LEN = 0.200
X_RAIL_R = 0.008
X_RAIL_SPACING = 0.120
RAIL_TOP_GAP = 0.004  # rail diameter standoff above the plate top

# X carriage that rides the lower rails.
X_CAR_L = 0.150
X_CAR_W = 0.160
X_CAR_T = 0.018

# Upper (Y-axis) rails run along Y, carried by the X carriage.
Y_RAIL_LEN = 0.130
Y_RAIL_R = 0.007
Y_RAIL_SPACING = 0.100

# Top Y plate that rides the upper rails.
Y_PLATE_L = 0.120
Y_PLATE_W = 0.110
Y_PLATE_T = 0.014

# Derived heights so each stage seats just above its rails.
BASE_TOP_Z = BASE_T  # base bottom sits on z=0
X_RAIL_CENTER_Z = BASE_TOP_Z + X_RAIL_R
X_JOINT_Z = X_RAIL_CENTER_Z + X_RAIL_R  # X carriage frame at the top of lower rails

Y_RAIL_CENTER_Z = X_CAR_T + Y_RAIL_R  # measured in x_stage local frame
Y_JOINT_Z = Y_RAIL_CENTER_Z + Y_RAIL_R  # Y plate frame at the top of upper rails


def _save(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rail_pair(length: float, radius: float, spacing: float, axis: str, center_z: float):
    """Return a single watertight mesh of two parallel rails plus end blocks."""
    half = spacing * 0.5
    block_t = radius * 2.4
    block_h = radius * 2.0
    parts = []
    if axis == "x":
        # rails run along X (local mesh frame is the parent part frame)
        for sign in (-1.0, 1.0):
            rail = CylinderGeometry(radius, length, radial_segments=24).rotate_y(pi / 2.0)
            rail.translate(0.0, sign * half, center_z)
            parts.append(rail)
        for end in (-1.0, 1.0):
            block = BoxGeometry((block_t, spacing + radius * 2.0, block_h))
            block.translate(end * length * 0.5, 0.0, center_z)
            parts.append(block)
    else:
        for sign in (-1.0, 1.0):
            rail = CylinderGeometry(radius, length, radial_segments=24).rotate_x(pi / 2.0)
            rail.translate(sign * half, 0.0, center_z)
            parts.append(rail)
        for end in (-1.0, 1.0):
            block = BoxGeometry((spacing + radius * 2.0, block_t, block_h))
            block.translate(0.0, end * length * 0.5, center_z)
            parts.append(block)
    merged = parts[0]
    for extra in parts[1:]:
        merged = boolean_union(merged, extra)
    return merged


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xy_stage")

    anodized_black = model.material("anodized_black", rgba=(0.16, 0.17, 0.19, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    machined_gray = model.material("machined_gray", rgba=(0.60, 0.62, 0.66, 1.0))
    plate_blue = model.material("plate_blue", rgba=(0.41, 0.47, 0.60, 1.0))

    # --- Base: grounded plate + lower X rails ---
    base = model.part("base")
    base_body = BoxGeometry((BASE_L, BASE_W, BASE_T))
    base_body.translate(0.0, 0.0, BASE_T * 0.5)
    base.visual(_save("xy_base_body", base_body), material=anodized_black, name="base_body")
    base.visual(
        _save("xy_x_rails", _rail_pair(X_RAIL_LEN, X_RAIL_R, X_RAIL_SPACING, "x", X_RAIL_CENTER_Z)),
        material=ground_steel,
        name="x_rails",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_T * 0.5)),
    )

    # --- X stage: carriage riding the lower rails + upper Y rails ---
    # The x_stage part frame sits at X_JOINT_Z (top of the lower rails); the
    # carriage body hangs just below its frame so it straddles the rails.
    x_stage = model.part("x_stage")
    x_car = BoxGeometry((X_CAR_L, X_CAR_W, X_CAR_T))
    x_car.translate(0.0, 0.0, X_CAR_T * 0.5)
    x_stage.visual(_save("xy_x_carriage", x_car), material=machined_gray, name="x_carriage")
    x_stage.visual(
        _save("xy_y_rails", _rail_pair(Y_RAIL_LEN, Y_RAIL_R, Y_RAIL_SPACING, "y", Y_RAIL_CENTER_Z)),
        material=ground_steel,
        name="y_rails",
    )
    x_stage.inertial = Inertial.from_geometry(
        Box((X_CAR_L, X_CAR_W, X_CAR_T)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, X_CAR_T * 0.5)),
    )

    # --- Y stage: top plate riding the upper rails ---
    y_stage = model.part("y_stage")
    y_plate = BoxGeometry((Y_PLATE_L, Y_PLATE_W, Y_PLATE_T))
    y_plate.translate(0.0, 0.0, Y_PLATE_T * 0.5)
    y_stage.visual(_save("xy_y_plate", y_plate), material=plate_blue, name="y_plate")
    y_stage.inertial = Inertial.from_geometry(
        Box((Y_PLATE_L, Y_PLATE_W, Y_PLATE_T)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, Y_PLATE_T * 0.5)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.060, upper=0.060, effort=120.0, velocity=0.25),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.035, upper=0.035, effort=90.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    base_to_x = object_model.get_articulation("base_to_x")
    x_to_y = object_model.get_articulation("x_to_y")

    ctx.check("has_base", base is not None, "Expected a base part.")
    ctx.check("has_x_stage", x_stage is not None, "Expected an x_stage part.")
    ctx.check("has_y_stage", y_stage is not None, "Expected a y_stage part.")
    ctx.check(
        "base_to_x_prismatic",
        base_to_x is not None and base_to_x.type == ArticulationType.PRISMATIC,
        "base_to_x must be PRISMATIC.",
    )
    ctx.check(
        "x_to_y_prismatic",
        x_to_y is not None and x_to_y.type == ArticulationType.PRISMATIC,
        "x_to_y must be PRISMATIC.",
    )

    # The carriage and plate ride above their rails with a thin embedment.
    ctx.allow_overlap(base, x_stage, reason="X carriage straddles the lower rails")
    ctx.allow_overlap(x_stage, y_stage, reason="Y plate straddles the upper rails")

    # Each stage stacks above the previous one at the neutral pose.
    with ctx.pose({base_to_x: 0.0, x_to_y: 0.0}):
        base_aabb = ctx.part_world_aabb(base)
        x_aabb = ctx.part_world_aabb(x_stage)
        y_aabb = ctx.part_world_aabb(y_stage)
        ctx.check(
            "x_above_base",
            base_aabb is not None and x_aabb is not None and x_aabb[1][2] > base_aabb[1][2],
            "X carriage should sit above the base.",
        )
        ctx.check(
            "y_above_x",
            x_aabb is not None and y_aabb is not None and y_aabb[1][2] > x_aabb[1][2],
            "Y plate should sit above the X carriage.",
        )

    # base_to_x drives the carriage along world +X.
    with ctx.pose({base_to_x: 0.0, x_to_y: 0.0}):
        x0 = ctx.part_world_position(x_stage)
    with ctx.pose({base_to_x: 0.060, x_to_y: 0.0}):
        x1 = ctx.part_world_position(x_stage)
    ctx.check(
        "x_moves_along_x",
        x0 is not None and x1 is not None and (x1[0] - x0[0]) > 0.05,
        f"x0={x0!r} x1={x1!r}",
    )

    # x_to_y drives the top plate along world +Y (and the carriage carries it in X).
    with ctx.pose({base_to_x: 0.0, x_to_y: 0.0}):
        y0 = ctx.part_world_position(y_stage)
    with ctx.pose({base_to_x: 0.0, x_to_y: 0.035}):
        y1 = ctx.part_world_position(y_stage)
    ctx.check(
        "y_moves_along_y",
        y0 is not None and y1 is not None and (y1[1] - y0[1]) > 0.03,
        f"y0={y0!r} y1={y1!r}",
    )

    return ctx.report()


object_model = build_object_model()
```
