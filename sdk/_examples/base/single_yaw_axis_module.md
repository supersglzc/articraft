---
title: 'Single Yaw Axis Module'
description: 'Base SDK pan turntable: a fixed base ring, a revolute yaw turntable plate, and a fixed payload pedestal that stages a sensor above the rotation axis.'
tags:
  - sdk
  - base sdk
  - articulation
  - revolute
  - yaw
  - pan
  - turntable
  - rotary stage
  - pan tilt
  - sensor mount
  - payload pedestal
  - single axis
  - mesh geometry
  - cylinder geometry
  - lathe geometry
  - motion limits
---
# Single Yaw Axis Module

This base-SDK example is a compact reference for a single yaw-axis pan module:
a fixed base, a revolute turntable plate spinning about `+Z`, and a fixed
payload pedestal that lifts a sensor block above the rotation axis. It is useful
for queries such as `pan turntable`, `yaw axis`, `rotary stage`, `sensor mount`,
`single axis`, and `revolute MotionLimits`.

The modeling patterns worth copying are:

- a native lathed `LatheGeometry` base puck with a recessed bearing seat.
- a revolute yaw joint about `+Z` between the base and the turntable plate, with
  realistic angular `MotionLimits`.
- a fixed payload pedestal mounted off-center on the spinning plate so it sweeps
  through a circle as the turntable pans.

```python
from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)


def _build_base_geometry() -> "object":
    # Lathed base puck: wide foot ring tapering up to a raised bearing collar,
    # with a recessed seat on top for the turntable to sit in.
    profile = [
        (0.0, 0.0),
        (0.105, 0.0),
        (0.105, 0.012),
        (0.082, 0.020),
        (0.082, 0.030),
        (0.072, 0.030),
        (0.072, 0.022),
        (0.0, 0.022),
    ]
    base = LatheGeometry(profile, segments=64)
    # Three mounting bosses spaced around the foot ring.
    for i in range(3):
        angle = 2.0 * pi * i / 3.0
        boss = CylinderGeometry(radius=0.013, height=0.014, radial_segments=20)
        boss.translate(0.090 * cos(angle), 0.090 * sin(angle), 0.007)
        base = boolean_union(base, boss)
    return base


def _build_turntable_geometry() -> "object":
    # Disc plate with a downward bearing hub that drops into the base seat.
    plate = CylinderGeometry(radius=0.090, height=0.011, radial_segments=64)
    plate.translate(0.0, 0.0, 0.0055)
    hub = CylinderGeometry(radius=0.068, height=0.012, radial_segments=48)
    hub.translate(0.0, 0.0, -0.006)
    body = boolean_union(plate, hub)
    # Lighten the plate with a shallow central pocket on top.
    pocket = CylinderGeometry(radius=0.052, height=0.008, radial_segments=48)
    pocket.translate(0.0, 0.0, 0.012)
    return boolean_difference(body, pocket)


def _build_pedestal_geometry() -> "object":
    # Riser column plus a top mounting plate that carries the sensor block.
    column = BoxGeometry((0.038, 0.038, 0.072))
    column.translate(0.0, 0.0, 0.036)
    top_plate = BoxGeometry((0.075, 0.055, 0.012))
    top_plate.translate(0.0, 0.0, 0.078)
    sensor = BoxGeometry((0.050, 0.040, 0.034))
    sensor.translate(0.0, 0.0, 0.101)
    body = boolean_union(column, top_plate)
    return boolean_union(body, sensor)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_turntable_sensor_mount")
    powder_black = model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    pedestal_gray = model.material("pedestal_gray", rgba=(0.58, 0.60, 0.64, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_build_base_geometry(), "turntable_base"),
        material=powder_black,
        name="turntable_base",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.105, length=0.030),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    turntable = model.part("turntable_plate")
    turntable.visual(
        mesh_from_geometry(_build_turntable_geometry(), "turntable_plate"),
        material=machined_aluminum,
        name="turntable_plate",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.011),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
    )

    pedestal = model.part("payload_pedestal")
    pedestal.visual(
        mesh_from_geometry(_build_pedestal_geometry(), "payload_pedestal"),
        material=pedestal_gray,
        name="payload_pedestal",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.075, 0.055, 0.089)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, 0.0445)),
    )

    # Yaw axis: turntable pans about +Z relative to the fixed base.
    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.967, upper=2.967, effort=12.0, velocity=2.5),
    )
    # The pedestal is rigidly bolted to the plate, off-center from the axis.
    model.articulation(
        "turntable_to_pedestal",
        ArticulationType.FIXED,
        parent=turntable,
        child=pedestal,
        origin=Origin(xyz=(0.035, 0.0, 0.011)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable_plate")
    pedestal = object_model.get_part("payload_pedestal")
    yaw = object_model.get_articulation("base_to_turntable")
    mount = object_model.get_articulation("turntable_to_pedestal")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("turntable_present", turntable is not None, "Expected a turntable plate part.")
    ctx.check("pedestal_present", pedestal is not None, "Expected a payload pedestal part.")
    ctx.check(
        "yaw_is_revolute",
        yaw is not None and yaw.articulation_type == ArticulationType.REVOLUTE,
        "Expected base_to_turntable to be REVOLUTE.",
    )
    ctx.check(
        "yaw_axis_is_z",
        yaw is not None and tuple(round(v, 3) for v in yaw.axis) == (0.0, 0.0, 1.0),
        "Expected the yaw axis to point along +Z.",
    )
    ctx.check(
        "pedestal_is_fixed",
        mount is not None and mount.articulation_type == ArticulationType.FIXED,
        "Expected turntable_to_pedestal to be FIXED.",
    )

    if base is None or turntable is None or pedestal is None:
        return ctx.report()

    # Panning the turntable sweeps the off-center pedestal around the axis.
    centered = ctx.part_world_aabb(pedestal)
    with ctx.pose({yaw: pi / 2.0}):
        rotated = ctx.part_world_aabb(pedestal)
    ctx.check(
        "pedestal_swings_with_yaw",
        centered is not None
        and rotated is not None
        and abs(centered[0][0] - rotated[0][0]) > 0.01,
        "Expected the off-center pedestal to move when the turntable pans.",
    )

    return ctx.report()


object_model = build_object_model()
```
