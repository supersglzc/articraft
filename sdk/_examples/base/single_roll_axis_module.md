---
title: 'Single Roll Axis Module'
description: 'Base SDK motorized roll-stage module: framed base plate with two pillow-block supports and a motor can, plus a coaxial sensor tube with an amber signal flag that rolls about the X axis.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - roll stage
  - roll axis
  - motorized stage
  - sensor tube
  - signal flag
  - pillow block
  - motor
  - revolute articulation
  - roll
  - boolean union
  - cylinder geometry
  - box geometry
---
# Single Roll Axis Module

This base-SDK example is a faithful native reproduction of a 5-star motorized
roll-stage module. The fixed `base` part carries a framed base plate, two
pillow-block bearing supports straddling the roll axis, and a cylindrical motor
can mounted at one end. The moving `sensor_tube` part is a coaxial tube body
with an amber signal flag that rolls about the `X` axis between the supports.

It is useful for queries such as `roll axis`, `roll stage`, `motorized stage`,
`sensor tube`, `signal flag`, `pillow block bearing`, `revolute roll
articulation`, `boolean_union`, and procedural `CylinderGeometry` / `BoxGeometry`
assembly.

The modeling patterns worth copying are:

- assembling a single watertight base-frame visual from a plate, two supports,
  and a motor can with `boolean_union(...)`.
- building the roll member as a fused tube body plus a radial signal flag, again
  unioned into one watertight mesh.
- a single `REVOLUTE` articulation whose `X` axis is coaxial with the tube and
  the bearing bores, with realistic `MotionLimits`.

```python
from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# Real-world scale, meters.
BASE_LEN = 0.30
BASE_WIDTH = 0.14
PLATE_THICK = 0.012

SUPPORT_SPAN = 0.20  # center-to-center distance between the two bearing supports
SUPPORT_WIDTH = 0.030
SUPPORT_HEIGHT = 0.070
SUPPORT_TOP_Z = SUPPORT_HEIGHT  # support top above the table reference plane

AXIS_HEIGHT = 0.055  # height of the roll axis above z=0

MOTOR_RADIUS = 0.030
MOTOR_LENGTH = 0.060

TUBE_RADIUS = 0.018
TUBE_LENGTH = 0.150
FLAG_LEN = 0.060
FLAG_HEIGHT = 0.038
FLAG_THICK = 0.006


def _save(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_base_frame_geometry():
    # Flat base plate sitting on z=0.
    plate = BoxGeometry((BASE_LEN, BASE_WIDTH, PLATE_THICK)).translate(
        0.0, 0.0, PLATE_THICK / 2.0
    )

    # Two pillow-block supports straddling the roll axis along X.
    support = BoxGeometry((SUPPORT_WIDTH, BASE_WIDTH * 0.7, SUPPORT_HEIGHT))
    left = support.copy().translate(-SUPPORT_SPAN / 2.0, 0.0, SUPPORT_HEIGHT / 2.0)
    right = support.copy().translate(SUPPORT_SPAN / 2.0, 0.0, SUPPORT_HEIGHT / 2.0)

    frame = boolean_union(plate, left)
    frame = boolean_union(frame, right)
    return frame


def _build_motor_geometry():
    # Motor can mounted on the base at the +X end, coaxial with the roll axis.
    can = CylinderGeometry(radius=MOTOR_RADIUS, height=MOTOR_LENGTH)
    can = can.rotate_y(pi / 2.0)  # lay the can along X
    can = can.translate(
        SUPPORT_SPAN / 2.0 + SUPPORT_WIDTH / 2.0 + MOTOR_LENGTH / 2.0,
        0.0,
        AXIS_HEIGHT,
    )
    # Mounting flange where the can meets the support.
    flange = BoxGeometry((0.010, BASE_WIDTH * 0.6, SUPPORT_HEIGHT * 0.9)).translate(
        SUPPORT_SPAN / 2.0 + SUPPORT_WIDTH / 2.0 + 0.005,
        0.0,
        SUPPORT_HEIGHT * 0.45,
    )
    return boolean_union(can, flange)


def _build_sensor_tube_geometry():
    # Tube body laid along X, centered on the local part origin (roll axis).
    body = CylinderGeometry(radius=TUBE_RADIUS, height=TUBE_LENGTH).rotate_y(pi / 2.0)
    # End caps as slightly larger collars so the tube reads as an assembly.
    collar = CylinderGeometry(radius=TUBE_RADIUS * 1.25, height=0.010).rotate_y(pi / 2.0)
    left_collar = collar.copy().translate(-TUBE_LENGTH / 2.0 + 0.005, 0.0, 0.0)
    right_collar = collar.copy().translate(TUBE_LENGTH / 2.0 - 0.005, 0.0, 0.0)
    tube = boolean_union(body, left_collar)
    tube = boolean_union(tube, right_collar)
    return tube


def _build_sensor_flag_geometry():
    # Radial signal flag standing off the tube along +Z, rooted into the tube.
    flag = BoxGeometry((FLAG_LEN, FLAG_THICK, FLAG_HEIGHT)).translate(
        0.0, 0.0, TUBE_RADIUS + FLAG_HEIGHT / 2.0 - 0.004
    )
    # Stub that overlaps the tube body so the flag is connected, not floating.
    stub = BoxGeometry((FLAG_LEN * 0.4, FLAG_THICK, TUBE_RADIUS)).translate(
        0.0, 0.0, TUBE_RADIUS / 2.0
    )
    return boolean_union(flag, stub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motorized_roll_stage")

    machined_aluminum = model.material(
        "machined_aluminum", rgba=(0.76, 0.78, 0.81, 1.0)
    )
    motor_black = model.material("motor_black", rgba=(0.18, 0.19, 0.21, 1.0))
    tube_black = model.material("tube_black", rgba=(0.12, 0.13, 0.15, 1.0))
    signal_amber = model.material("signal_amber", rgba=(0.87, 0.52, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        _save("roll_stage_base_frame.obj", _build_base_frame_geometry()),
        material=machined_aluminum,
        name="base_frame",
    )
    base.visual(
        _save("roll_stage_motor.obj", _build_motor_geometry()),
        material=motor_black,
        name="motor_can",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_WIDTH, SUPPORT_TOP_Z)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_TOP_Z / 2.0)),
    )

    sensor_tube = model.part("sensor_tube")
    sensor_tube.visual(
        _save("sensor_tube_body.obj", _build_sensor_tube_geometry()),
        material=tube_black,
        name="tube_body",
    )
    sensor_tube.visual(
        _save("sensor_tube_flag.obj", _build_sensor_flag_geometry()),
        material=signal_amber,
        name="signal_flag",
    )
    sensor_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_RADIUS, length=TUBE_LENGTH),
        mass=0.42,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    # Coaxial roll: the part origin sits on the roll axis, midway between the two
    # bearing supports and at AXIS_HEIGHT above the base plate.
    model.articulation(
        "tube_roll",
        ArticulationType.REVOLUTE,
        parent=base,
        child=sensor_tube,
        origin=Origin(xyz=(0.0, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=6.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tube = object_model.get_part("sensor_tube")
    roll = object_model.get_articulation("tube_roll")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("tube_present", tube is not None, "Expected a sensor_tube part.")
    ctx.check("roll_present", roll is not None, "Expected a tube_roll articulation.")
    if base is None or tube is None or roll is None:
        return ctx.report()

    ctx.check(
        "roll_is_revolute",
        roll.articulation_type == ArticulationType.REVOLUTE,
        "Roll joint should be revolute.",
    )

    # At neutral the amber flag points up; rolling +90deg should swing it toward +Y.
    with ctx.pose({roll: 0.0}):
        neutral = ctx.part_world_aabb(tube)
    with ctx.pose({roll: pi / 2.0}):
        rolled = ctx.part_world_aabb(tube)

    ctx.check("tube_aabb_neutral", neutral is not None, "Expected a tube AABB at q=0.")
    ctx.check("tube_aabb_rolled", rolled is not None, "Expected a tube AABB at q=pi/2.")
    if neutral is not None and rolled is not None:
        top_drop = float(neutral[1][2] - rolled[1][2])
        ctx.check(
            "flag_rolls_down_in_z",
            top_drop > 0.02,
            f"Flag top should drop in Z when rolled; top_drop={top_drop:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
