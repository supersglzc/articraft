---
title: 'Prismatic-revolute-revolute Chain'
description: 'Base SDK mixed-topology module: a linear slide feeds a moving carriage that pivots a shoulder link and then a forearm link, built from native mesh geometry.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - articulation
  - prismatic
  - revolute
  - prismatic joint
  - revolute joint
  - linear slide
  - carriage
  - shoulder
  - forearm
  - serial chain
  - mixed topology
  - motion limits
  - boolean union
---
# Prismatic-revolute-revolute Chain

This base-SDK example reproduces the mixed serial topology of a compact
linear-dual-rotary module: translate first along a rail, then pivot twice off
the moving carriage. The kinematic chain is

`base --prismatic--> carriage --revolute--> shoulder_link --revolute--> forearm_link`.

It is useful for queries such as `prismatic revolute revolute`, `linear slide
plus arm`, `carriage`, `shoulder forearm chain`, `mixed topology`, and
`PRR manipulator`. The geometry is authored natively with `BoxGeometry`,
`CylinderGeometry`, and `boolean_union(...)`, and each part is emitted with
`mesh_from_geometry(...)`.

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


def _make_base_geometry() -> "BoxGeometry":
    # A fixed steel base plate carrying a raised linear rail along +X.
    plate = BoxGeometry((0.34, 0.12, 0.04)).translate(0.0, 0.0, 0.02)
    rail = BoxGeometry((0.30, 0.05, 0.03)).translate(0.0, 0.0, 0.055)
    left_foot = BoxGeometry((0.04, 0.14, 0.02)).translate(0.14, 0.0, 0.01)
    right_foot = BoxGeometry((0.04, 0.14, 0.02)).translate(-0.14, 0.0, 0.01)
    geom = boolean_union(plate, rail)
    geom = boolean_union(geom, left_foot)
    geom = boolean_union(geom, right_foot)
    return geom


def _make_carriage_geometry() -> "BoxGeometry":
    # The carriage straddles the rail and carries a vertical pivot tower.
    body = BoxGeometry((0.10, 0.11, 0.05)).translate(0.0, 0.0, 0.07)
    # Side rails that wrap down around the base rail.
    left_skirt = BoxGeometry((0.10, 0.02, 0.05)).translate(0.0, 0.052, 0.05)
    right_skirt = BoxGeometry((0.10, 0.02, 0.05)).translate(0.0, -0.052, 0.05)
    # Vertical boss where the shoulder revolute pivots (along +Z).
    tower = CylinderGeometry(0.026, 0.05, radial_segments=28).translate(0.052, 0.0, 0.07)
    geom = boolean_union(body, left_skirt)
    geom = boolean_union(geom, right_skirt)
    geom = boolean_union(geom, tower)
    return geom


def _make_shoulder_geometry() -> "BoxGeometry":
    # Shoulder link: a hub at the carriage pivot and an arm reaching +X to the
    # forearm pivot, which sits along -Y.
    hub = CylinderGeometry(0.024, 0.052, radial_segments=28)
    arm = BoxGeometry((0.112, 0.05, 0.034)).translate(0.056, 0.0, 0.0)
    # Cross boss carrying the forearm revolute (axis along Y).
    elbow = CylinderGeometry(0.022, 0.072, radial_segments=28)
    elbow = elbow.rotate_x(pi / 2.0).translate(0.112, 0.0, 0.0)
    geom = boolean_union(hub, arm)
    geom = boolean_union(geom, elbow)
    return geom


def _make_forearm_geometry() -> "BoxGeometry":
    # Forearm link: yoke at the elbow plus a tapered arm reaching out to a tool
    # flange at the tip.
    yoke = CylinderGeometry(0.020, 0.064, radial_segments=28).rotate_x(pi / 2.0)
    arm = BoxGeometry((0.150, 0.040, 0.028)).translate(0.075, 0.0, 0.0)
    flange = CylinderGeometry(0.026, 0.018, radial_segments=28)
    flange = flange.rotate_y(pi / 2.0).translate(0.150, 0.0, 0.0)
    geom = boolean_union(yoke, arm)
    geom = boolean_union(geom, flange)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_dual_rotary_module")

    base_steel = model.material("base_steel", rgba=(0.53, 0.56, 0.60, 1.0))
    carriage_dark = model.material("carriage_dark", rgba=(0.20, 0.22, 0.26, 1.0))
    joint_blue = model.material("joint_blue", rgba=(0.24, 0.39, 0.65, 1.0))
    arm_silver = model.material("arm_silver", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_make_base_geometry(), "module_base"),
        material=base_steel,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.12, 0.04)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_geometry(_make_carriage_geometry(), "module_carriage"),
        material=carriage_dark,
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.10, 0.11, 0.10)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        mesh_from_geometry(_make_shoulder_geometry(), "module_shoulder"),
        material=joint_blue,
        name="shoulder_shell",
    )
    shoulder.inertial = Inertial.from_geometry(
        Box((0.16, 0.05, 0.05)),
        mass=0.9,
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
    )

    forearm = model.part("forearm_link")
    forearm.visual(
        mesh_from_geometry(_make_forearm_geometry(), "module_forearm"),
        material=arm_silver,
        name="forearm_shell",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.18, 0.05, 0.04)),
        mass=0.6,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    # 1) Prismatic slide of the carriage along the rail (+X).
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.08, 0.0, 0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.14, effort=250.0, velocity=0.25),
    )
    # 2) Shoulder pivots about the carriage tower (+Z).
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.052, 0.0, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=40.0, velocity=1.2),
    )
    # 3) Forearm pivots about the shoulder elbow (-Y), lifting in pitch.
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forearm,
        origin=Origin(xyz=(0.112, 0.0, 0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.1, effort=25.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    shoulder = object_model.get_part("shoulder_link")
    forearm = object_model.get_part("forearm_link")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("carriage_present", carriage is not None, "Expected a carriage part.")
    ctx.check("shoulder_present", shoulder is not None, "Expected a shoulder link.")
    ctx.check("forearm_present", forearm is not None, "Expected a forearm link.")

    slide = object_model.get_articulation("base_to_carriage")
    shoulder_joint = object_model.get_articulation("carriage_to_shoulder")
    forearm_joint = object_model.get_articulation("shoulder_to_forearm")

    ctx.check(
        "slide_is_prismatic",
        slide is not None and slide.type == ArticulationType.PRISMATIC,
        "base_to_carriage should be prismatic.",
    )
    ctx.check(
        "shoulder_is_revolute",
        shoulder_joint is not None and shoulder_joint.type == ArticulationType.REVOLUTE,
        "carriage_to_shoulder should be revolute.",
    )
    ctx.check(
        "forearm_is_revolute",
        forearm_joint is not None and forearm_joint.type == ArticulationType.REVOLUTE,
        "shoulder_to_forearm should be revolute.",
    )

    # The prismatic slide should translate the carriage along +X by ~the travel.
    if carriage is not None and slide is not None:
        retracted = ctx.part_world_aabb(carriage)
        with ctx.pose({slide: 0.14}):
            extended = ctx.part_world_aabb(carriage)
        if retracted is not None and extended is not None:
            dx = extended[0][0] - retracted[0][0]
            ctx.check(
                "slide_translates_x",
                0.12 <= dx <= 0.16,
                f"carriage should advance ~0.14 m along +X, got {dx:.3f}.",
            )

    return ctx.report()


object_model = build_object_model()
```
