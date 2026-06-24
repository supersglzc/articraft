---
title: 'Two-joint Revolute Chain'
description: 'Base SDK reading-lamp example with a weighted base, lower arm, and upper arm-plus-shade assembly forming a realistic 2R serial revolute chain.'
tags:
  - sdk
  - base sdk
  - articulation
  - revolute
  - serial chain
  - 2r chain
  - reading lamp
  - desk lamp
  - lamp
  - arm
  - shade
  - mesh geometry
  - lathe geometry
  - cone geometry
  - capsule geometry
  - motion limits
---
# Two-joint Revolute Chain

This base-SDK example reproduces a realistic two-joint (2R) serial revolute
chain in the form of a reading lamp: a weighted base, a lower arm, and an upper
assembly that carries the lamp shade. It is useful for queries such as
`two joint revolute chain`, `2R chain`, `serial revolute arm`, `reading lamp`,
`desk lamp`, and `revolute articulation`.

Both arm links extend along their local `+X` from their pivots, so the pitch
joints use `axis=(0, -1, 0)`: positive joint values raise the arms upward. The
chain is `base -> lower_arm -> upper_assembly`, with the shoulder joint at the
top of the base column and the elbow joint at the far end of the lower arm.

```python
from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

BASE_RADIUS = 0.090
BASE_THICKNESS = 0.026
COLUMN_RADIUS = 0.016
COLUMN_HEIGHT = 0.060
SHOULDER_Z = BASE_THICKNESS + COLUMN_HEIGHT

LOWER_ARM_LENGTH = 0.260
LOWER_ARM_RADIUS = 0.013

UPPER_ARM_LENGTH = 0.230
UPPER_ARM_RADIUS = 0.012
SHADE_DROP = 0.020

SHOULDER_LIMITS = (-0.35, 1.40)
ELBOW_LIMITS = (-2.20, 0.20)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_base_shape():
    # A weighted, slightly tapered disc plus a short mounting column, revolved
    # as one watertight lathe so the base reads as a single cast foot.
    profile = [
        (0.0, 0.0),
        (BASE_RADIUS, 0.0),
        (BASE_RADIUS, BASE_THICKNESS * 0.55),
        (BASE_RADIUS * 0.74, BASE_THICKNESS),
        (COLUMN_RADIUS * 1.6, BASE_THICKNESS),
        (COLUMN_RADIUS, BASE_THICKNESS + 0.004),
        (COLUMN_RADIUS, BASE_THICKNESS + COLUMN_HEIGHT),
        (0.0, BASE_THICKNESS + COLUMN_HEIGHT),
    ]
    return LatheGeometry(profile, segments=48)


def _build_lower_arm_shape():
    # A cylindrical strut running along +X from the shoulder pivot, fused with a
    # small pivot boss at the elbow end so the chain reads mechanically. Both
    # operands are manifold cylinders, so the union stays watertight.
    strut = CylinderGeometry(LOWER_ARM_RADIUS, LOWER_ARM_LENGTH, radial_segments=20)
    strut.rotate_y(pi / 2.0).translate(LOWER_ARM_LENGTH / 2.0, 0.0, 0.0)
    shoulder_boss = CylinderGeometry(LOWER_ARM_RADIUS * 1.7, LOWER_ARM_RADIUS * 3.4)
    shoulder_boss.rotate_x(pi / 2.0)
    elbow_boss = CylinderGeometry(LOWER_ARM_RADIUS * 1.7, LOWER_ARM_RADIUS * 3.4)
    elbow_boss.rotate_x(pi / 2.0).translate(LOWER_ARM_LENGTH, 0.0, 0.0)
    arm = boolean_union(strut, shoulder_boss)
    return boolean_union(arm, elbow_boss)


def _build_upper_arm_frame_shape():
    # Upper cylindrical strut along +X from the elbow pivot, with a pivot boss at
    # the elbow root.
    strut = CylinderGeometry(UPPER_ARM_RADIUS, UPPER_ARM_LENGTH, radial_segments=20)
    strut.rotate_y(pi / 2.0).translate(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)
    elbow_boss = CylinderGeometry(UPPER_ARM_RADIUS * 1.7, UPPER_ARM_RADIUS * 3.4)
    elbow_boss.rotate_x(pi / 2.0)
    return boolean_union(strut, elbow_boss)


def _build_shade_shape():
    # A conical lamp shade at the far end of the upper arm, opening downward.
    cone = ConeGeometry(0.060, 0.075, radial_segments=40)
    # ConeGeometry tapers toward +Z; flip so the wide rim faces down, then drop
    # the shade slightly below the arm line at the upper end.
    cone.rotate_x(pi).translate(UPPER_ARM_LENGTH, 0.0, -SHADE_DROP)
    neck = CylinderGeometry(0.010, 0.030)
    neck.translate(UPPER_ARM_LENGTH, 0.0, -SHADE_DROP * 0.2)
    return boolean_union(cone, neck)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reading_lamp")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.14, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.66, 0.56, 0.29, 1.0))
    shade_cream = model.material("shade_cream", rgba=(0.93, 0.91, 0.84, 1.0))

    base = model.part("base")
    base.visual(_save_mesh("lamp_base.obj", _build_base_shape()), material=powder_black)
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(_save_mesh("lower_arm.obj", _build_lower_arm_shape()), material=warm_brass)
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH, 0.024, 0.024)),
        mass=0.42,
        origin=Origin(xyz=(LOWER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    upper_assembly = model.part("upper_assembly")
    upper_assembly.visual(
        _save_mesh("upper_arm_frame.obj", _build_upper_arm_frame_shape()),
        material=warm_brass,
    )
    upper_assembly.visual(_save_mesh("lamp_shade.obj", _build_shade_shape()), material=shade_cream)
    upper_assembly.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH + 0.075, 0.072, 0.090)),
        mass=0.48,
        origin=Origin(xyz=(UPPER_ARM_LENGTH * 0.55, 0.0, -SHADE_DROP)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lower_arm",
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        # Closed arm geometry extends along +X from the shoulder.
        # -Y makes positive q pitch the arm upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
            effort=18.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_assembly",
        ArticulationType.REVOLUTE,
        parent="lower_arm",
        child="upper_assembly",
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        # Same sign convention: positive q raises the upper assembly.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
            effort=12.0,
            velocity=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_assembly = object_model.get_part("upper_assembly")
    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_assembly")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("lower_arm_present", lower_arm is not None, "Expected a lower_arm part.")
    ctx.check(
        "upper_assembly_present",
        upper_assembly is not None,
        "Expected an upper_assembly part.",
    )
    ctx.check("shoulder_revolute", shoulder.type is ArticulationType.REVOLUTE, "shoulder type")
    ctx.check("elbow_revolute", elbow.type is ArticulationType.REVOLUTE, "elbow type")

    # Positive shoulder rotation should raise the elbow end of the lower arm.
    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        flat = ctx.part_world_aabb(lower_arm)
    with ctx.pose({shoulder: SHOULDER_LIMITS[1], elbow: 0.0}):
        raised = ctx.part_world_aabb(upper_assembly)
        flat_upper = None
    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        flat_upper = ctx.part_world_aabb(upper_assembly)

    if flat is not None and raised is not None and flat_upper is not None:
        ctx.check(
            "shoulder_raises_chain",
            raised[1][2] > flat_upper[1][2],
            "Positive shoulder pose should lift the upper assembly higher.",
        )

    return ctx.report()


object_model = build_object_model()
```
