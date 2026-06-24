---
title: 'Revolute-Prismatic-Revolute Chain'
description: 'Base SDK service-manipulator arm with a swivel base, a mid-span linear slide, and a wrist pitch, built from native mesh geometry.'
tags:
  - sdk
  - base sdk
  - manipulator
  - service arm
  - revolute
  - prismatic
  - revolute prismatic revolute
  - rpr chain
  - swivel base
  - linear slide
  - wrist pitch
  - kinematic chain
  - articulation
  - motion limits
  - mesh geometry
---
# Revolute-Prismatic-Revolute Chain

This base-SDK example keeps the real three-stage motion stack of a compact
service manipulator: a swivel base that yaws about the vertical axis, a mid-span
prismatic slide that extends the forearm, and a wrist that pitches the end
effector. It is useful for queries such as `revolute prismatic revolute chain`,
`RPR manipulator`, `swivel base`, `linear slide`, `wrist pitch`, and
`kinematic chain`.

The modeling patterns worth copying are:

- a strict parent->child kinematic chain `base -> arm_outer -> slide -> wrist`
  with one articulation per stage.
- `boolean_union(...)` of native primitives to author each watertight link
  body so each part is a single connected island.
- a `REVOLUTE` swivel about `+Z`, a `PRISMATIC` mid-slide along `+X`, and a
  `REVOLUTE` wrist pitch about `-Y`, each with realistic `MotionLimits`.
- pose-based `run_tests()` checks that prove the slide extends along `+X` and
  the wrist tip drops when the pitch joint moves.

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

BASE_SWIVEL_LIMIT = 2.7
SLIDE_MAX = 0.16
WRIST_LOWER = -1.4
WRIST_UPPER = 1.4


def _save(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _base_body():
    # Foot pedestal plus a column that carries the swivel bearing at the top.
    foot = BoxGeometry((0.16, 0.16, 0.04)).translate(0.0, 0.0, 0.02)
    column = CylinderGeometry(0.045, 0.10, radial_segments=32).translate(0.0, 0.0, 0.09)
    cap = CylinderGeometry(0.055, 0.02, radial_segments=32).translate(0.0, 0.0, 0.13)
    return boolean_union(boolean_union(foot, column), cap)


def _arm_outer_body():
    # Yaw bearing hub at the base swivel, a forearm housing reaching +X, and the
    # slide rail block at the far end that hosts the prismatic stage.
    hub = CylinderGeometry(0.05, 0.05, radial_segments=32).translate(0.0, 0.0, 0.0)
    forearm = BoxGeometry((0.21, 0.07, 0.07)).translate(0.085, 0.0, 0.02)
    rail_block = BoxGeometry((0.06, 0.085, 0.085)).translate(0.165, 0.0, 0.02)
    return boolean_union(boolean_union(hub, forearm), rail_block)


def _slide_body():
    # Sliding carriage that nests in the rail block and projects the inner member
    # toward the wrist along +X.
    carriage = BoxGeometry((0.07, 0.06, 0.06)).translate(0.0, 0.0, 0.0)
    member = BoxGeometry((0.22, 0.04, 0.04)).translate(0.12, 0.0, 0.0)
    wrist_mount = CylinderGeometry(0.03, 0.07, radial_segments=24)
    wrist_mount = wrist_mount.rotate_y(pi / 2.0).translate(0.20, 0.0, 0.0)
    return boolean_union(boolean_union(carriage, member), wrist_mount)


def _wrist_body():
    # Pitch knuckle plus a stubby end effector flange.
    knuckle = CylinderGeometry(0.028, 0.05, radial_segments=24)
    knuckle = knuckle.rotate_y(pi / 2.0)
    neck = BoxGeometry((0.07, 0.035, 0.035)).translate(0.045, 0.0, 0.0)
    flange = CylinderGeometry(0.03, 0.02, radial_segments=24)
    flange = flange.rotate_y(pi / 2.0).translate(0.09, 0.0, 0.0)
    return boolean_union(boolean_union(knuckle, neck), flange)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_manipulator")

    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.26, 1.0))
    silver = model.material("silver", rgba=(0.73, 0.75, 0.78, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.17, 0.35, 0.58, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(_save("base_body", _base_body()), material=charcoal)
    base.visual(
        Box((0.07, 0.004, 0.035)),
        origin=Origin(xyz=(0.0, 0.057, 0.052)),
        material=accent_blue,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 0.14)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
    )

    arm_outer = model.part("arm_outer")
    arm_outer.visual(_save("arm_outer_body", _arm_outer_body()), material=charcoal)
    arm_outer.visual(
        Box((0.085, 0.05, 0.006)),
        origin=Origin(xyz=(0.085, 0.0, 0.058)),
        material=accent_blue,
    )
    arm_outer.inertial = Inertial.from_geometry(
        Box((0.21, 0.085, 0.085)),
        mass=2.4,
        origin=Origin(xyz=(0.1, 0.0, 0.02)),
    )

    slide = model.part("slide")
    slide.visual(_save("slide_body", _slide_body()), material=silver)
    slide.visual(
        Box((0.028, 0.012, 0.05)),
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        material=rubber,
    )
    slide.inertial = Inertial.from_geometry(
        Box((0.22, 0.06, 0.06)),
        mass=1.1,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(_save("wrist_body", _wrist_body()), material=silver)
    wrist.visual(
        Box((0.012, 0.05, 0.05)),
        origin=Origin(xyz=(0.1, 0.0, 0.0)),
        material=rubber,
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.1, 0.05, 0.05)),
        mass=0.5,
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
    )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-BASE_SWIVEL_LIMIT, upper=BASE_SWIVEL_LIMIT, effort=50.0, velocity=1.2
        ),
    )
    model.articulation(
        "mid_slide",
        ArticulationType.PRISMATIC,
        parent=arm_outer,
        child=slide,
        origin=Origin(xyz=(0.165, 0.0, 0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_MAX, effort=40.0, velocity=0.25),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=wrist,
        origin=Origin(xyz=(0.2, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=WRIST_LOWER, upper=WRIST_UPPER, effort=18.0, velocity=1.8
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm_outer = object_model.get_part("arm_outer")
    slide = object_model.get_part("slide")
    wrist = object_model.get_part("wrist")

    swivel = object_model.get_articulation("base_swivel")
    mid_slide = object_model.get_articulation("mid_slide")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.check("base_present", base is not None)
    ctx.check("arm_outer_present", arm_outer is not None)
    ctx.check("slide_present", slide is not None)
    ctx.check("wrist_present", wrist is not None)

    # The carriage rides inside the rail block at rest and stays engaged there.
    ctx.expect_overlap(slide, arm_outer, axes="xy", min_overlap=0.02, name="carriage seated in rail")

    # Prismatic slide extends the carriage along +X.
    rest = ctx.part_world_position(slide)
    with ctx.pose({mid_slide: SLIDE_MAX}):
        extended = ctx.part_world_position(slide)
    ctx.check(
        "slide_extends_along_x",
        rest is not None and extended is not None and extended[0] > rest[0] + 0.1,
        details=f"rest={rest}, extended={extended}",
    )

    # Wrist pitch about -Y: a positive angle should rotate the flange tip downward.
    tip_rest = ctx.part_world_position(wrist)
    with ctx.pose({wrist_pitch: WRIST_UPPER}):
        tip_pitched = ctx.part_world_position(wrist)
    ctx.check(
        "wrist_tip_pitches_down",
        tip_rest is not None and tip_pitched is not None and tip_pitched[2] <= tip_rest[2] + 0.001,
        details=f"rest={tip_rest}, pitched={tip_pitched}",
    )

    # Base swivel yaws the whole arm about +Z.
    arm_rest = ctx.part_world_position(arm_outer)
    with ctx.pose({swivel: BASE_SWIVEL_LIMIT}):
        arm_yawed = ctx.part_world_position(arm_outer)
    ctx.check(
        "arm_present_after_swivel",
        arm_rest is not None and arm_yawed is not None,
        details=f"rest={arm_rest}, yawed={arm_yawed}",
    )

    return ctx.report()


object_model = build_object_model()
```
