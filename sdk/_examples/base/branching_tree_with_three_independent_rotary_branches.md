---
title: 'Branching Tree with Three Independent Rotary Branches'
description: 'Base SDK example showing a central hub carrying three independent revolute arms generated through one loop instead of hand-writing each articulation.'
tags:
  - sdk
  - base sdk
  - articulation
  - branching
  - revolute
  - hub
  - rotary arm
  - repeated articulation
  - mesh geometry
---
# Branching Tree with Three Independent Rotary Branches

This base-SDK example shows how to scale a branching topology with a small loop
instead of writing each articulation by hand. A single cylindrical hub carries
three identical rotary arms, each connected by its own `REVOLUTE` articulation
placed around the hub at evenly spaced angles. It is useful for queries such as
`branching tree`, `rotary arms around a hub`, `repeated revolute articulation`,
`independent branches`, and `loop-generated joints`.

The modeling patterns worth copying are:

- one mesh-building helper reused for every arm so the branch geometry is
  authored once.
- a loop over `(name, angle)` pairs that builds each branch part and adds its
  `REVOLUTE` articulation with a frame placed at `JOINT_RADIUS` around the hub.
- per-branch articulation origins that both translate to the rim of the hub and
  rotate the child frame so every arm points radially outward.

```python
from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

HUB_RADIUS = 0.060
HUB_HEIGHT = 0.050
JOINT_RADIUS = 0.058

ARM_LENGTH = 0.180
ARM_RADIUS = 0.016
ARM_PIVOT_RADIUS = 0.022

BRANCH_NAMES = ("branch_a", "branch_b", "branch_c")
BRANCH_ANGLES = (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)
ARM_LIMITS = MotionLimits(effort=4.0, velocity=3.0, lower=-1.4, upper=1.4)


def _build_hub_geometry():
    barrel = CylinderGeometry(HUB_RADIUS, HUB_HEIGHT, radial_segments=48)
    collar = CylinderGeometry(HUB_RADIUS * 0.62, HUB_HEIGHT * 1.28, radial_segments=40)
    return boolean_union(barrel, collar)


def _build_arm_geometry():
    # Pivot boss at the joint (axis along Y), a straight arm shaft along +X,
    # and a flat end cap at the tip.
    boss = CylinderGeometry(ARM_PIVOT_RADIUS, 0.034, radial_segments=32).rotate_x(pi / 2.0)

    shaft = CylinderGeometry(ARM_RADIUS, ARM_LENGTH, radial_segments=28)
    shaft.rotate_y(pi / 2.0)
    shaft.translate(ARM_PIVOT_RADIUS + ARM_LENGTH * 0.5, 0.0, 0.0)

    tip = CylinderGeometry(ARM_RADIUS * 1.35, 0.018, radial_segments=28)
    tip.rotate_y(pi / 2.0)
    tip.translate(ARM_PIVOT_RADIUS + ARM_LENGTH, 0.0, 0.0)

    geom = boolean_union(boss, shaft)
    geom = boolean_union(geom, tip)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_tree_mechanism")

    hub_gray = model.material("hub_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    arm_blue = model.material("arm_blue", rgba=(0.23, 0.45, 0.72, 1.0))

    hub_mesh = mesh_from_geometry(_build_hub_geometry(), "rotary_tree_hub")

    hub = model.part("hub")
    hub.visual(hub_mesh, material=hub_gray)
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        mass=1.15,
    )

    arm_geometry = _build_arm_geometry()

    for name, angle in zip(BRANCH_NAMES, BRANCH_ANGLES):
        branch = model.part(name)
        # Each branch reuses the same authored arm geometry, exported under a
        # unique logical mesh name. The arm is authored along local +X, so the
        # joint origin rotates the child frame by `angle` to fan the arms out.
        branch.visual(
            mesh_from_geometry(arm_geometry.clone(), f"rotary_tree_arm_{name}"),
            material=arm_blue,
        )
        branch.inertial = Inertial.from_geometry(
            Box((ARM_LENGTH, ARM_RADIUS * 2.0, ARM_RADIUS * 2.0)),
            mass=0.25,
            origin=Origin(xyz=(ARM_PIVOT_RADIUS + ARM_LENGTH * 0.5, 0.0, 0.0)),
        )

        model.articulation(
            f"hub_to_{name}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=branch,
            origin=Origin(
                xyz=(JOINT_RADIUS * cos(angle), JOINT_RADIUS * sin(angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=ARM_LIMITS,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hub = object_model.get_part("hub")
    ctx.check("hub_present", hub is not None, "Expected a hub part.")

    # Exactly three independent rotary branches, each its own revolute joint.
    branch_count = 0
    for name in BRANCH_NAMES:
        branch = object_model.get_part(name)
        ctx.check(f"{name}_present", branch is not None, f"Expected branch part {name}.")
        joint = object_model.get_articulation(f"hub_to_{name}")
        ctx.check(
            f"hub_to_{name}_revolute",
            joint is not None and joint.articulation_type == ArticulationType.REVOLUTE,
            f"Expected revolute articulation hub_to_{name}.",
        )
        if branch is not None and joint is not None:
            branch_count += 1
    ctx.check("three_branches", branch_count == 3, f"branch_count={branch_count}")

    # The three arms should fan out radially: at the rest pose they cluster near
    # the hub center, and rotating one branch should not disturb the others.
    joint_a = object_model.get_articulation("hub_to_branch_a")
    joint_b = object_model.get_articulation("hub_to_branch_b")
    branch_b = object_model.get_part("branch_b")
    if joint_a is not None and joint_b is not None and branch_b is not None:
        with ctx.pose({joint_a: 0.0, joint_b: 0.0}):
            rest_aabb = ctx.part_world_aabb(branch_b)
        with ctx.pose({joint_a: 1.2, joint_b: 0.0}):
            moved_aabb = ctx.part_world_aabb(branch_b)
        ctx.check(
            "branch_b_independent",
            rest_aabb is not None and moved_aabb is not None and rest_aabb == moved_aabb,
            "Rotating branch_a must not move branch_b.",
        )

    return ctx.report()


object_model = build_object_model()
```
