---
title: 'Yaw-pitch-roll Wrist'
description: 'Base SDK robotic tool wrist with nested yaw, pitch, and roll revolute axes built from native mesh geometry (base shell, yaw collar, pitch yoke, roll spindle).'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - robot wrist
  - tool wrist
  - wrist
  - yaw
  - pitch
  - roll
  - nested axes
  - revolute articulation
  - yoke
  - collar
  - spindle
  - cylinder geometry
  - boolean union
  - motion limits
---
# Yaw-pitch-roll Wrist

This base-SDK example reproduces the nested-axis layout of a 3-DOF robotic
tool wrist: a fixed base shell, a yaw collar that rotates about the vertical
axis, a pitch yoke carried by the collar, and a roll spindle that spins inside
the yoke. It is useful for queries such as `tool wrist`, `robot wrist`,
`yaw pitch roll`, `nested revolute axes`, `pitch yoke`, `roll spindle`, and
`CylinderGeometry` boolean assemblies.

The patterns worth copying are:

- nesting three `REVOLUTE` articulations (yaw about `+Z`, pitch about `-Y`,
  roll about `+X`) so each child frame sits on its own joint axis.
- building each link from a few `CylinderGeometry` / `BoxGeometry` primitives
  fused with `boolean_union(...)` into a single connected solid before export.
- placing each part frame at its proximal joint and extending the geometry
  toward the next joint so the chain stays mechanically plausible.

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


def _make_base_shape() -> "BoxGeometry":
    # Fixed mounting base: a flange plate with a short pedestal that carries the
    # yaw bearing. The part frame is at the mounting plane (z = 0); the yaw
    # joint sits at z = 0.040.
    plate = BoxGeometry((0.110, 0.110, 0.018)).translate(0.0, 0.0, 0.009)
    pedestal = CylinderGeometry(0.044, 0.030, radial_segments=40).translate(0.0, 0.0, 0.033)
    bearing = CylinderGeometry(0.034, 0.016, radial_segments=40).translate(0.0, 0.0, 0.040)
    shape = boolean_union(plate, pedestal)
    shape = boolean_union(shape, bearing)
    return shape


def _make_yaw_collar_shape() -> "CylinderGeometry":
    # Yaw collar: a vertical hub sitting on the base bearing, with an arm that
    # reaches out along +X to the pitch axis at (0.068, 0, 0.022). The part
    # frame is at the yaw axis.
    hub = CylinderGeometry(0.033, 0.034, radial_segments=40).translate(0.0, 0.0, 0.012)
    arm = BoxGeometry((0.080, 0.046, 0.038)).translate(0.040, 0.0, 0.022)
    cheek = CylinderGeometry(0.024, 0.040, radial_segments=32)
    cheek = cheek.rotate_x(pi / 2.0).translate(0.068, 0.0, 0.022)
    shape = boolean_union(hub, arm)
    shape = boolean_union(shape, cheek)
    return shape


def _make_pitch_yoke_shape() -> "BoxGeometry":
    # Pitch yoke: a U-shaped fork that pivots about -Y at its proximal end and
    # carries the roll bearing at +X = 0.060. The part frame is at the pitch
    # axis. Two cheeks plus a back web keep it a single connected solid.
    back = BoxGeometry((0.022, 0.072, 0.040)).translate(0.006, 0.0, 0.0)
    upper = BoxGeometry((0.068, 0.018, 0.036)).translate(0.034, 0.026, 0.0)
    lower = BoxGeometry((0.068, 0.018, 0.036)).translate(0.034, -0.026, 0.0)
    bearing = CylinderGeometry(0.020, 0.060, radial_segments=36)
    bearing = bearing.rotate_y(pi / 2.0).translate(0.060, 0.0, 0.0)
    shape = boolean_union(back, upper)
    shape = boolean_union(shape, lower)
    shape = boolean_union(shape, bearing)
    return shape


def _make_roll_spindle_shape() -> "CylinderGeometry":
    # Roll spindle: spins about +X inside the yoke bearing and presents a tool
    # flange at its distal end. The part frame is at the roll axis origin.
    shaft = CylinderGeometry(0.017, 0.064, radial_segments=36)
    shaft = shaft.rotate_y(pi / 2.0).translate(0.014, 0.0, 0.0)
    flange = CylinderGeometry(0.030, 0.012, radial_segments=40)
    flange = flange.rotate_y(pi / 2.0).translate(0.046, 0.0, 0.0)
    boss = CylinderGeometry(0.014, 0.014, radial_segments=28)
    boss = boss.rotate_y(pi / 2.0).translate(0.059, 0.0, 0.0)
    shape = boolean_union(shaft, flange)
    shape = boolean_union(shape, boss)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tool_wrist")

    base_paint = model.material("base_paint", rgba=(0.22, 0.22, 0.24, 1.0))
    collar_paint = model.material("collar_paint", rgba=(0.16, 0.17, 0.19, 1.0))
    yoke_finish = model.material("yoke_finish", rgba=(0.68, 0.71, 0.74, 1.0))
    spindle_finish = model.material("spindle_finish", rgba=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_make_base_shape(), "tool_wrist_base"),
        material=base_paint,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 0.058)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    yaw_collar = model.part("yaw_collar")
    yaw_collar.visual(
        mesh_from_geometry(_make_yaw_collar_shape(), "tool_wrist_yaw_collar"),
        material=collar_paint,
        name="yaw_collar_shell",
    )
    yaw_collar.inertial = Inertial.from_geometry(
        Box((0.100, 0.046, 0.040)),
        mass=0.9,
        origin=Origin(xyz=(0.034, 0.0, 0.014)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_geometry(_make_pitch_yoke_shape(), "tool_wrist_pitch_yoke"),
        material=yoke_finish,
        name="pitch_yoke_shell",
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.080, 0.072, 0.040)),
        mass=0.6,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        mesh_from_geometry(_make_roll_spindle_shape(), "tool_wrist_roll_spindle"),
        material=spindle_finish,
        name="roll_spindle_shell",
    )
    roll_spindle.inertial = Inertial.from_geometry(
        Box((0.066, 0.060, 0.060)),
        mass=0.35,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_collar,
        child=pitch_yoke,
        origin=Origin(xyz=(0.068, 0.0, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.3, lower=-1.1, upper=1.1),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_spindle,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=4.0, lower=-3.0, upper=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    yaw = object_model.get_part("yaw_collar")
    pitch = object_model.get_part("pitch_yoke")
    roll = object_model.get_part("roll_spindle")
    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("yaw_present", yaw is not None, "Expected a yaw_collar part.")
    ctx.check("pitch_present", pitch is not None, "Expected a pitch_yoke part.")
    ctx.check("roll_present", roll is not None, "Expected a roll_spindle part.")

    yaw_joint = object_model.get_articulation("base_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")
    ctx.check(
        "three_revolute_axes",
        all(
            j is not None and j.articulation_type is ArticulationType.REVOLUTE
            for j in (yaw_joint, pitch_joint, roll_joint)
        ),
        "Expected three revolute wrist axes.",
    )
    ctx.check(
        "yaw_axis_vertical",
        yaw_joint is not None and yaw_joint.axis == (0.0, 0.0, 1.0),
        "Yaw axis should be vertical (+Z).",
    )
    ctx.check(
        "roll_axis_along_x",
        roll_joint is not None and roll_joint.axis == (1.0, 0.0, 0.0),
        "Roll axis should be along +X.",
    )

    if None not in (yaw_joint, pitch_joint, roll_joint):
        with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, roll_joint: 0.0}):
            ctx.expect_overlap(yaw, base, axes="xy", min_overlap=0.01)

    return ctx.report()


object_model = build_object_model()
```
