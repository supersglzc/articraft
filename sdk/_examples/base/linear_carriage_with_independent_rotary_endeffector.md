---
title: 'Linear Carriage with Independent Rotary End-effector'
description: 'Base SDK example of a prismatic linear carriage riding a profiled rail and carrying its own independent revolute rotary spindle end-effector, built from native mesh geometry.'
tags:
  - sdk
  - base sdk
  - linear carriage
  - linear slide
  - rail
  - prismatic
  - prismatic articulation
  - revolute articulation
  - spindle
  - rotary end-effector
  - end effector
  - motion stage
  - cartesian stage
  - motion limits
  - mesh geometry
  - extrude geometry
  - boolean union
---
# Linear Carriage with Independent Rotary End-effector

This base-SDK example reproduces a rail-mounted spindle stage: a prismatic
carriage slides along a profiled extruded rail, and the spindle end-effector
gets its own independent revolute rotary articulation on the carriage. It is a
useful reference for queries such as `linear carriage`, `linear slide`,
`prismatic rail`, `motion stage`, `rotary end-effector`, `spindle`, combining a
`PRISMATIC` slide with an independent `REVOLUTE` tool axis.

The modeling patterns worth copying are:

- a profiled rail built by extruding a rounded-rect cross section along its run.
- a carriage shell unioned with riser blocks so the spindle mount is a single
  connected body.
- a spindle stack (mounting flange, body, side control module) assembled with
  `boolean_union(...)` into one watertight tool head.
- one `PRISMATIC` slide articulation plus one independent `REVOLUTE` rotary
  articulation, each with realistic `MotionLimits`.

```python
from __future__ import annotations

# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

RAIL_LENGTH = 0.46
RAIL_WIDTH = 0.060
RAIL_HEIGHT = 0.040

CARRIAGE_LENGTH = 0.110
CARRIAGE_WIDTH = 0.090
CARRIAGE_HEIGHT = 0.034
CARRIAGE_CLEAR_Z = RAIL_HEIGHT / 2.0 + CARRIAGE_HEIGHT / 2.0 + 0.001

RISER_HEIGHT = 0.046
RISER_TOP_Z = CARRIAGE_CLEAR_Z + CARRIAGE_HEIGHT / 2.0 + RISER_HEIGHT

SPINDLE_FLANGE_RADIUS = 0.030
SPINDLE_FLANGE_LENGTH = 0.014
SPINDLE_BODY_RADIUS = 0.022
SPINDLE_BODY_LENGTH = 0.072
SPINDLE_MODULE = (0.022, 0.030, 0.030)

PRISMATIC_TRAVEL = 0.16


def _save(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rail_geometry():
    # Profiled rail cross section in XZ, extruded along the +Y run direction.
    profile = rounded_rect_profile(RAIL_WIDTH, RAIL_HEIGHT, 0.008, corner_segments=6)
    rail = ExtrudeGeometry.centered(profile, RAIL_LENGTH, cap=True)
    # Profile lies in XY, extruded along Z. Re-orient so width->X, run->Y, height->Z.
    rail.rotate_x(pi / 2.0)
    # Two raised guide ribs along the top edges keep the carriage captured.
    rib = BoxGeometry((0.010, RAIL_LENGTH, 0.012))
    left = rib.copy().translate(RAIL_WIDTH / 2.0 - 0.006, 0.0, RAIL_HEIGHT / 2.0 + 0.004)
    right = rib.copy().translate(-(RAIL_WIDTH / 2.0 - 0.006), 0.0, RAIL_HEIGHT / 2.0 + 0.004)
    return boolean_union(boolean_union(rail, left), right)


def _carriage_geometry():
    body = BoxGeometry((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT))
    body.translate(0.0, 0.0, CARRIAGE_CLEAR_Z)
    # Two risers rise from the carriage top to the spindle mounting plate.
    riser_z = CARRIAGE_CLEAR_Z + CARRIAGE_HEIGHT / 2.0 + RISER_HEIGHT / 2.0
    riser = BoxGeometry((0.024, 0.070, RISER_HEIGHT))
    left = riser.copy().translate(0.034, 0.0, riser_z)
    right = riser.copy().translate(-0.034, 0.0, riser_z)
    # Mounting plate caps the risers and carries the spindle joint.
    plate = BoxGeometry((CARRIAGE_LENGTH, 0.078, 0.010))
    plate.translate(0.0, 0.0, RISER_TOP_Z + 0.005)
    out = boolean_union(body, left)
    out = boolean_union(out, right)
    out = boolean_union(out, plate)
    return out


def _spindle_geometry():
    # Spindle stack along its local +Z axis: mounting flange, body, side module.
    flange = CylinderGeometry(SPINDLE_FLANGE_RADIUS, SPINDLE_FLANGE_LENGTH, radial_segments=40)
    flange.translate(0.0, 0.0, SPINDLE_FLANGE_LENGTH / 2.0)
    body = CylinderGeometry(SPINDLE_BODY_RADIUS, SPINDLE_BODY_LENGTH, radial_segments=40)
    body.translate(0.0, 0.0, SPINDLE_FLANGE_LENGTH + SPINDLE_BODY_LENGTH / 2.0)
    module = BoxGeometry(SPINDLE_MODULE)
    module.translate(
        0.0,
        SPINDLE_BODY_RADIUS + SPINDLE_MODULE[1] / 2.0 - 0.006,
        SPINDLE_FLANGE_LENGTH + 0.024,
    )
    out = boolean_union(flange, body)
    out = boolean_union(out, module)
    return out


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_carriage_spindle")

    aluminum = model.material("anodized_aluminum", rgba=(0.74, 0.76, 0.80, 1.0))
    polymer = model.material("dark_polymer", rgba=(0.16, 0.17, 0.20, 1.0))
    steel = model.material("tool_steel", rgba=(0.56, 0.57, 0.60, 1.0))

    rail_base = model.part("rail_base")
    rail_base.visual(_save("rail_base", _rail_geometry()), material=aluminum)
    rail_base.inertial = Inertial.from_geometry(
        Box((RAIL_WIDTH, RAIL_LENGTH, RAIL_HEIGHT)),
        mass=6.0,
    )

    carriage = model.part("carriage")
    carriage.visual(_save("carriage", _carriage_geometry()), material=polymer)
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_CLEAR_Z)),
    )

    # Spindle mesh is authored along +Z; the joint frame mounts it on the plate
    # and the revolute axis is +Z so it spins about its own long axis.
    spindle = model.part("spindle")
    spindle.visual(_save("spindle", _spindle_geometry()), material=steel)
    spindle.inertial = Inertial.from_geometry(
        Box((2.0 * SPINDLE_FLANGE_RADIUS, 2.0 * SPINDLE_FLANGE_RADIUS, SPINDLE_BODY_LENGTH)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_BODY_LENGTH / 2.0)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-PRISMATIC_TRAVEL,
            upper=PRISMATIC_TRAVEL,
            effort=150.0,
            velocity=0.5,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, RISER_TOP_Z + 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=12.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()

    rail = object_model.get_part("rail_base")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("rail_to_carriage")
    rotary = object_model.get_articulation("carriage_to_spindle")

    ctx.check("has_rail", rail is not None, "Expected a rail_base part.")
    ctx.check("has_carriage", carriage is not None, "Expected a carriage part.")
    ctx.check("has_spindle", spindle is not None, "Expected a spindle part.")
    ctx.check(
        "slide_is_prismatic",
        slide is not None and slide.articulation_type == ArticulationType.PRISMATIC,
        "Expected a prismatic slide articulation.",
    )
    ctx.check(
        "rotary_is_revolute",
        rotary is not None and rotary.articulation_type == ArticulationType.REVOLUTE,
        "Expected a revolute spindle articulation.",
    )

    # The slide carries the spindle: at the far travel pose the spindle moves
    # the same way as the carriage along the rail (+Y) and the rotary axis is
    # independent of the slide.
    if carriage is not None and spindle is not None:
        with ctx.pose({slide: PRISMATIC_TRAVEL, rotary: 0.0}):
            ctx.expect_overlap(spindle, carriage, axes="xy", min_overlap=0.01)
        with ctx.pose({slide: 0.0, rotary: pi / 2.0}):
            ctx.expect_overlap(spindle, carriage, axes="xy", min_overlap=0.01)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
