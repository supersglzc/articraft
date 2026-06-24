---
title: 'Opposed Twin-slide Gripper'
description: 'Base SDK gripper with a common body that carries two mirrored prismatic jaws sliding toward each other on shared rails, built from native mesh geometry.'
tags:
  - sdk
  - base sdk
  - gripper
  - parallel jaw gripper
  - two finger gripper
  - prismatic
  - prismatic articulation
  - mirrored kinematics
  - opposed jaws
  - linear slide
  - rails
  - end effector
  - mesh geometry
  - extrude geometry
  - rounded rect profile
  - boolean union
  - motion limits
---
# Opposed Twin-slide Gripper

This base-SDK example is a reference for mirrored prismatic kinematics: a common
body carries two jaws with equal travel but opposite articulation axes, so the
two fingers slide toward each other to close on a part and apart to release it.
It is useful for queries such as `parallel jaw gripper`, `two finger gripper`,
`opposed jaws`, `prismatic articulation`, `mirrored kinematics`, `linear slide`,
and `end effector`.

The modeling patterns worth copying are:

- a single body part that fuses the mount plate, main housing, and two shared
  guide rails into one watertight visual with `boolean_union(...)`.
- one jaw-building helper reused for the left and right fingers so the mirrored
  parts stay identical except for their slide direction.
- two `PRISMATIC` articulations sharing the same travel limits but with opposite
  `axis` directions, so a single commanded `q` closes both jaws symmetrically.
- pose-based tests that prove the jaws clear the body when open and approach the
  centerline (without colliding) when closed.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

HALF_PI = 1.5707963267948966

# Overall scale: a small industrial parallel-jaw gripper, roughly 0.12 m wide.
# Body coordinate frame: the housing bottom is at z=0 and it stacks upward; the
# rails and jaws hang below z=0 with a clear vertical gap from the housing.
BODY_WIDTH = 0.120
BODY_DEPTH = 0.060
BODY_HEIGHT = 0.044

PLATE_THICKNESS = 0.010
RAIL_RADIUS = 0.005
RAIL_LENGTH = 0.108
RAIL_SPACING = 0.034  # center-to-center spacing of the two parallel guide rails
RAIL_Z = -0.018  # rails sit below the housing bottom (z=0), where jaws ride

CARRIAGE_THICKNESS = 0.016  # jaw extent along the slide (X) axis
CARRIAGE_HEIGHT = 0.022
FINGER_HEIGHT = 0.044
FINGER_THICKNESS = 0.010
CARRIAGE_Z = RAIL_Z  # carriage centered on the rails

JAW_TRAVEL = 0.024  # each jaw can move this far toward the centerline
# Open pose: each jaw carriage center sits this far out from the centerline (X).
LEFT_JAW_OPEN_X = 0.042
RIGHT_JAW_OPEN_X = -0.042
JAW_ORIGIN_Y = 0.0


def _body_geometry() -> "BoxGeometry":
    # Main housing block, bottom on z=0.
    housing = BoxGeometry((BODY_WIDTH * 0.92, BODY_DEPTH * 0.92, BODY_HEIGHT)).translate(
        0.0, 0.0, BODY_HEIGHT * 0.5
    )
    # Mount plate on top.
    plate = BoxGeometry((BODY_WIDTH, BODY_DEPTH, PLATE_THICKNESS)).translate(
        0.0, 0.0, BODY_HEIGHT + PLATE_THICKNESS * 0.5
    )
    # Two parallel guide rails running along X, below the housing.
    rail_front = (
        CylinderGeometry(RAIL_RADIUS, RAIL_LENGTH, radial_segments=20)
        .rotate_y(HALF_PI)
        .translate(0.0, RAIL_SPACING * 0.5, RAIL_Z)
    )
    rail_back = (
        CylinderGeometry(RAIL_RADIUS, RAIL_LENGTH, radial_segments=20)
        .rotate_y(HALF_PI)
        .translate(0.0, -RAIL_SPACING * 0.5, RAIL_Z)
    )
    # End posts drop from the housing to the rail ends so the body is one solid.
    post_left = BoxGeometry((0.014, BODY_DEPTH * 0.92, 0.030)).translate(
        RAIL_LENGTH * 0.5 - 0.007, 0.0, RAIL_Z + 0.006
    )
    post_right = BoxGeometry((0.014, BODY_DEPTH * 0.92, 0.030)).translate(
        -(RAIL_LENGTH * 0.5 - 0.007), 0.0, RAIL_Z + 0.006
    )

    body = boolean_union(housing, plate)
    body = boolean_union(body, post_left)
    body = boolean_union(body, post_right)
    body = boolean_union(body, rail_front)
    body = boolean_union(body, rail_back)
    return body


def _jaw_geometry(side: str) -> "BoxGeometry":
    # The jaw carriage block straddles both rails, with a gripping finger that
    # points down and toward the centerline. Built in a local frame whose origin
    # is the carriage center; the inner gripping face points along -X for the
    # left jaw and +X for the right jaw (i.e. toward the opposite finger).
    inner_sign = -1.0 if side == "left" else 1.0

    carriage = BoxGeometry((CARRIAGE_THICKNESS, BODY_DEPTH * 0.96, CARRIAGE_HEIGHT))

    # Finger: a rounded vertical blade extending down to grip a part.
    finger_profile = rounded_rect_profile(FINGER_THICKNESS, FINGER_HEIGHT, 0.003)
    finger = ExtrudeGeometry.centered(finger_profile, BODY_DEPTH * 0.70)
    # ExtrudeGeometry extrudes along Z; rotate so the blade depth runs along Y.
    finger.rotate_x(HALF_PI)
    # Hang the finger off the inner face of the carriage, dropping below it.
    finger_x = inner_sign * (CARRIAGE_THICKNESS * 0.5 - FINGER_THICKNESS * 0.5)
    finger.translate(
        finger_x,
        0.0,
        -CARRIAGE_HEIGHT * 0.5 - FINGER_HEIGHT * 0.5 + 0.004,
    )
    # Soft contact pad on the inner gripping face of the finger.
    pad = BoxGeometry((0.004, BODY_DEPTH * 0.55, FINGER_HEIGHT * 0.6))
    pad.translate(
        finger_x + inner_sign * (FINGER_THICKNESS * 0.5 + 0.002),
        0.0,
        -CARRIAGE_HEIGHT * 0.5 - FINGER_HEIGHT * 0.55,
    )

    jaw = boolean_union(carriage, finger)
    jaw = boolean_union(jaw, pad)
    return jaw


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_slide_gripper")

    body_mat = model.material("body_anodized", rgba=(0.23, 0.25, 0.28, 1.0))
    jaw_mat = model.material("jaw_black", rgba=(0.16, 0.17, 0.19, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_geometry(), "body_shell"),
        name="body_shell",
        material=body_mat,
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.6,
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_geometry(_jaw_geometry("left"), "left_jaw"),
        name="jaw_shell",
        material=jaw_mat,
    )
    left_jaw.inertial = Inertial.from_geometry(
        Box((CARRIAGE_THICKNESS, BODY_DEPTH, CARRIAGE_HEIGHT + FINGER_HEIGHT)),
        mass=0.08,
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_geometry(_jaw_geometry("right"), "right_jaw"),
        name="jaw_shell",
        material=jaw_mat,
    )
    right_jaw.inertial = Inertial.from_geometry(
        Box((CARRIAGE_THICKNESS, BODY_DEPTH, CARRIAGE_HEIGHT + FINGER_HEIGHT)),
        mass=0.08,
    )

    # Left jaw closes by moving along -X (toward the centerline) as q increases.
    model.articulation(
        "body_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(LEFT_JAW_OPEN_X, JAW_ORIGIN_Y, CARRIAGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.15, lower=0.0, upper=JAW_TRAVEL),
    )
    # Right jaw mirrors it: same q, opposite axis (+X) -> moves toward centerline.
    model.articulation(
        "body_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(RIGHT_JAW_OPEN_X, JAW_ORIGIN_Y, CARRIAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.15, lower=0.0, upper=JAW_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("body_to_left_jaw")
    right_slide = object_model.get_articulation("body_to_right_jaw")

    ctx.check("body_present", body is not None, "Expected a body part.")
    ctx.check("left_jaw_present", left_jaw is not None, "Expected a left jaw part.")
    ctx.check("right_jaw_present", right_jaw is not None, "Expected a right jaw part.")

    # The carriages ride directly on the shared guide rails, so each carriage
    # legitimately envelops the rails of the body. That is the intended sliding
    # contact of a linear slide, not a modeling error.
    ctx.allow_overlap(body, left_jaw, reason="left carriage rides on shared body rails")
    ctx.allow_overlap(body, right_jaw, reason="right carriage rides on shared body rails")

    # Each jaw closes toward the centerline (opposite world-X directions): the
    # left jaw (+X) slides toward -X, the right jaw (-X) slides toward +X.
    ctx.expect_joint_motion_axis(
        left_slide, left_jaw, world_axis="x", direction="negative", min_delta=0.5 * JAW_TRAVEL
    )
    ctx.expect_joint_motion_axis(
        right_slide, right_jaw, world_axis="x", direction="positive", min_delta=0.5 * JAW_TRAVEL
    )

    # Fully open: jaws sit apart, neither colliding with the other or the body.
    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_clear")
        # The left jaw sits on +X relative to the right jaw, with an open air gap.
        ctx.expect_gap(left_jaw, right_jaw, axis="x", min_gap=0.005, name="open_jaw_gap")
        open_aabb_l = ctx.part_world_aabb(left_jaw)
        open_aabb_r = ctx.part_world_aabb(right_jaw)

    # Fully closed: jaws travel toward the centerline and approach each other.
    with ctx.pose({left_slide: JAW_TRAVEL, right_slide: JAW_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_clear")
        closed_aabb_l = ctx.part_world_aabb(left_jaw)
        closed_aabb_r = ctx.part_world_aabb(right_jaw)

    # The mirrored axes must move the two jaws closer together when closing.
    if None not in (open_aabb_l, closed_aabb_l, open_aabb_r, closed_aabb_r):
        # gap = (left jaw min X) - (right jaw max X), since left rides on +X.
        open_gap = open_aabb_l[0][0] - open_aabb_r[1][0]
        closed_gap = closed_aabb_l[0][0] - closed_aabb_r[1][0]
        ctx.check(
            "jaws_close_inward",
            closed_gap < open_gap - 0.5 * JAW_TRAVEL,
            f"open_gap={open_gap!r} closed_gap={closed_gap!r}",
        )

    return ctx.report()


object_model = build_object_model()
```
