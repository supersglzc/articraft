---
title: 'XYZ Cartesian Stage'
description: 'Base SDK three-axis linear motion stage with stacked X, Y, and Z prismatic slides built from native box geometry and a unioned base plate with dovetail rails.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - xyz stage
  - cartesian stage
  - linear stage
  - three axis stage
  - prismatic
  - prismatic articulation
  - linear slide
  - gantry
  - saddle
  - carriage
  - rail
  - motion limits
  - box geometry
  - boolean union
---
# XYZ Cartesian Stage

This base-SDK example is a compact reference for a three-axis Cartesian motion
stage: a fixed base with linear rails carrying an X gantry, the gantry carrying
a Y saddle, and the saddle carrying a vertical Z carriage with a tool plate.
It is useful for queries such as `xyz stage`, `cartesian stage`, `linear stage`,
`three axis stage`, `prismatic articulation`, `linear slide`, `gantry`, and
`MotionLimits` on prismatic joints.

The modeling patterns worth copying are:

- a fixed root part whose plate and rails are unioned into one watertight mesh
  so the part has no disconnected geometry islands.
- a stack of three `PRISMATIC` articulations, one per axis (X, Y, Z), each with
  its own travel limits expressed through `MotionLimits`.
- pose-driven tests that prove each slide translates along the intended world
  axis and that travel limits stay within the authored envelope.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

BASE_X_TRAVEL = 0.080
Y_STAGE_TRAVEL = 0.045
Z_STAGE_TRAVEL = 0.030


def _box(size, center):
    return BoxGeometry(size).translate(*center)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage")

    anodized_black = model.material("anodized_black", rgba=(0.16, 0.17, 0.20, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.56, 0.58, 0.62, 1.0))
    plate_blue = model.material("plate_blue", rgba=(0.22, 0.35, 0.66, 1.0))

    # Base: a thick anodized plate with two X rails on top. Union the plate and
    # rails into one mesh so the fixed root reads as a single connected body.
    base = model.part("base")
    base_geom = boolean_union(
        boolean_union(
            _box((0.36, 0.25, 0.03), (0.0, 0.0, 0.015)),
            _box((0.28, 0.024, 0.014), (0.0, -0.078, 0.034)),
        ),
        _box((0.28, 0.024, 0.014), (0.0, 0.078, 0.034)),
    )
    base.visual(mesh_from_geometry(base_geom, "base_plate"), material=anodized_black)
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.25, 0.05)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    # X gantry: rides the base rails along X, carries two Y rails on its top.
    x_gantry = model.part("x_gantry")
    x_gantry_geom = boolean_union(
        boolean_union(
            _box((0.19, 0.20, 0.028), (0.0, 0.0, 0.014)),
            _box((0.020, 0.180, 0.014), (-0.032, 0.0, 0.035)),
        ),
        _box((0.020, 0.180, 0.014), (0.032, 0.0, 0.035)),
    )
    x_gantry.visual(mesh_from_geometry(x_gantry_geom, "x_gantry_body"), material=machined_aluminum)
    x_gantry.inertial = Inertial.from_geometry(
        Box((0.19, 0.20, 0.042)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    # Y saddle: rides the gantry Y rails, carries a vertical Z column.
    y_saddle = model.part("y_saddle")
    y_saddle_geom = boolean_union(
        boolean_union(
            _box((0.112, 0.120, 0.024), (-0.016, 0.0, 0.012)),
            _box((0.028, 0.024, 0.130), (0.021, -0.030, 0.075)),
        ),
        _box((0.028, 0.024, 0.130), (0.021, 0.030, 0.075)),
    )
    # Web that ties the two vertical column rails into one connected body.
    y_saddle_geom = boolean_union(
        y_saddle_geom,
        _box((0.018, 0.084, 0.130), (0.021, 0.0, 0.075)),
    )
    y_saddle.visual(mesh_from_geometry(y_saddle_geom, "y_saddle_body"), material=machined_aluminum)
    y_saddle.inertial = Inertial.from_geometry(
        Box((0.112, 0.120, 0.150)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
    )

    # Z carriage: rides the saddle column vertically, holds the tool plate.
    z_carriage = model.part("z_carriage")
    z_carriage_geom = boolean_union(
        _box((0.032, 0.076, 0.090), (0.0, 0.0, 0.0)),
        _box((0.012, 0.078, 0.078), (0.022, 0.0, 0.0)),
    )
    z_carriage.visual(mesh_from_geometry(z_carriage_geom, "z_carriage_body"), material=machined_aluminum)
    z_carriage.visual(
        Box((0.008, 0.082, 0.082)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=plate_blue,
        name="tool_plate",
    )
    z_carriage.inertial = Inertial.from_geometry(
        Box((0.046, 0.082, 0.090)),
        mass=0.7,
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_gantry,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-BASE_X_TRAVEL, upper=BASE_X_TRAVEL, effort=120.0, velocity=0.25),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_gantry,
        child=y_saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-Y_STAGE_TRAVEL, upper=Y_STAGE_TRAVEL, effort=90.0, velocity=0.20),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_saddle,
        child=z_carriage,
        origin=Origin(xyz=(0.021, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-Z_STAGE_TRAVEL, upper=Z_STAGE_TRAVEL, effort=70.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    x_gantry = object_model.get_part("x_gantry")
    y_saddle = object_model.get_part("y_saddle")
    z_carriage = object_model.get_part("z_carriage")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("x_gantry_present", x_gantry is not None, "Expected an x_gantry part.")
    ctx.check("y_saddle_present", y_saddle is not None, "Expected a y_saddle part.")
    ctx.check("z_carriage_present", z_carriage is not None, "Expected a z_carriage part.")

    base_to_x = object_model.get_articulation("base_to_x")
    x_to_y = object_model.get_articulation("x_to_y")
    y_to_z = object_model.get_articulation("y_to_z")

    ctx.check(
        "base_to_x_prismatic",
        base_to_x is not None and base_to_x.type == ArticulationType.PRISMATIC,
        "base_to_x must be a prismatic joint.",
    )
    ctx.check(
        "x_to_y_prismatic",
        x_to_y is not None and x_to_y.type == ArticulationType.PRISMATIC,
        "x_to_y must be a prismatic joint.",
    )
    ctx.check(
        "y_to_z_prismatic",
        y_to_z is not None and y_to_z.type == ArticulationType.PRISMATIC,
        "y_to_z must be a prismatic joint.",
    )

    # Each slide should translate the carried stack along the intended world axis.
    ctx.expect_joint_motion_axis(
        base_to_x, x_gantry, world_axis="x", direction="positive", min_delta=BASE_X_TRAVEL * 0.5
    )
    ctx.expect_joint_motion_axis(
        x_to_y, y_saddle, world_axis="y", direction="positive", min_delta=Y_STAGE_TRAVEL * 0.5
    )
    ctx.expect_joint_motion_axis(
        y_to_z, z_carriage, world_axis="z", direction="positive", min_delta=Z_STAGE_TRAVEL * 0.5
    )

    return ctx.report()


object_model = build_object_model()
```
