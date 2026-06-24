---
title: 'Branching Tree with Two Independent Rotary Branches'
description: 'Base SDK Y-tree with one trunk and two independent revolute branches, showing a non-serial articulation graph with procedural mesh geometry.'
tags:
  - sdk
  - base sdk
  - tree
  - branching
  - y tree
  - revolute articulation
  - independent branches
  - non serial graph
  - trunk
  - foliage
  - mesh geometry
---
# Branching Tree with Two Independent Rotary Branches

This base-SDK example reproduces the classic Y-tree teaching object: one fixed
trunk carries two branches, each driven by its own independent revolute joint.
It is the simplest clean example of a non-serial articulation graph (one parent,
two sibling children that move independently). It is useful for queries such as
`tree`, `branching`, `Y-tree`, `independent branches`, `non-serial articulation
graph`, and `revolute branch`.

The patterns worth copying are:

- one root trunk part with two sibling child parts hanging off it.
- two independent `REVOLUTE` articulations sharing one parent but no motion link.
- procedural mesh trunk/branch members built from tapered `LatheGeometry` and
  capped with a `SphereGeometry` foliage crown via `boolean_union`.

```python
from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# Hub offsets where each branch leaves the trunk (meters).
HUB_X = 0.10
HUB_Z = 0.95
BRANCH_LEN = 0.55


def _tapered_member(bottom_radius: float, top_radius: float, height: float, name: str):
    """A trunk/branch segment: a tapered revolved column standing on z=0."""
    profile = [
        (bottom_radius, 0.0),
        (bottom_radius * 0.92, height * 0.35),
        (top_radius * 1.05, height * 0.75),
        (top_radius, height),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=24), name)


def _branch_limb(name: str):
    """A branch limb: a tapered revolved column running along local +Z."""
    limb = LatheGeometry(
        [
            (0.040, 0.0),
            (0.034, BRANCH_LEN * 0.45),
            (0.028, BRANCH_LEN * 0.80),
            (0.022, BRANCH_LEN),
        ],
        segments=20,
    )
    return mesh_from_geometry(limb, name)


def _branch_crown(name: str):
    """A spherical foliage crown seated on the free end of the limb."""
    crown = SphereGeometry(0.16, width_segments=24, height_segments=16).translate(
        0.0, 0.0, BRANCH_LEN + 0.02
    )
    return mesh_from_geometry(crown, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_y_tree")

    bark = model.material("frame_dark", rgba=(0.34, 0.24, 0.17, 1.0))
    foliage = model.material("branch_green", rgba=(0.31, 0.54, 0.36, 1.0))

    # Root trunk: a tapered column plus a small hub block where branches mount.
    trunk = model.part("trunk")
    trunk.visual(_tapered_member(0.10, 0.06, HUB_Z, "trunk_column"), material=bark)
    trunk.visual(
        Box((0.26, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        material=bark,
    )
    trunk.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, HUB_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, HUB_Z * 0.5)),
    )

    # Two independent branches. Each branch frame sits at its hub; the limb runs
    # along local +Z out of the hub and the foliage crown caps the free end.
    left_branch = model.part("left_branch")
    # Limb leans outward (-X) from the hub before the foliage crown.
    left_branch.visual(
        _branch_limb("left_branch_limb"),
        origin=Origin(rpy=(0.0, -0.5, 0.0)),
        material=bark,
    )
    left_branch.visual(
        _branch_crown("left_branch_crown"),
        origin=Origin(rpy=(0.0, -0.5, 0.0)),
        material=foliage,
    )
    left_branch.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, BRANCH_LEN)),
        mass=3.0,
        origin=Origin(xyz=(-0.18, 0.0, 0.32)),
    )

    right_branch = model.part("right_branch")
    # Limb leans outward (+X) from the hub before the foliage crown.
    right_branch.visual(
        _branch_limb("right_branch_limb"),
        origin=Origin(rpy=(0.0, 0.5, 0.0)),
        material=bark,
    )
    right_branch.visual(
        _branch_crown("right_branch_crown"),
        origin=Origin(rpy=(0.0, 0.5, 0.0)),
        material=foliage,
    )
    right_branch.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, BRANCH_LEN)),
        mass=3.0,
        origin=Origin(xyz=(0.18, 0.0, 0.32)),
    )

    # Sibling revolute joints share the trunk parent but are otherwise unlinked.
    model.articulation(
        "trunk_to_left_branch",
        ArticulationType.REVOLUTE,
        parent=trunk,
        child=left_branch,
        origin=Origin(xyz=(-HUB_X, 0.0, HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.24, upper=0.24, effort=4.0, velocity=1.5),
    )
    model.articulation(
        "trunk_to_right_branch",
        ArticulationType.REVOLUTE,
        parent=trunk,
        child=right_branch,
        origin=Origin(xyz=(HUB_X, 0.0, HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.24, upper=0.24, effort=4.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    trunk = object_model.get_part("trunk")
    left = object_model.get_part("left_branch")
    right = object_model.get_part("right_branch")
    left_joint = object_model.get_articulation("trunk_to_left_branch")
    right_joint = object_model.get_articulation("trunk_to_right_branch")

    ctx.check("trunk_present", trunk is not None, "Expected a trunk part.")
    ctx.check("left_branch_present", left is not None, "Expected a left branch part.")
    ctx.check("right_branch_present", right is not None, "Expected a right branch part.")

    # Non-serial graph: both branches must hang directly off the trunk.
    ctx.check(
        "left_parent_is_trunk",
        left_joint is not None and left_joint.parent == "trunk",
        "Left branch must be a direct child of the trunk.",
    )
    ctx.check(
        "right_parent_is_trunk",
        right_joint is not None and right_joint.parent == "trunk",
        "Right branch must be a direct child of the trunk.",
    )

    # The branches move independently: posing one must not move the other.
    if left_joint is not None and right_joint is not None:
        rest = ctx.part_world_aabb(right)
        with ctx.pose({left_joint: 0.24}):
            moved = ctx.part_world_aabb(left)
            still = ctx.part_world_aabb(right)
        ctx.check(
            "left_branch_moves",
            rest is not None and moved is not None,
            "Expected world AABBs for the left branch.",
        )
        if rest is not None and still is not None:
            same = all(
                abs(rest[0][i] - still[0][i]) < 1e-6 and abs(rest[1][i] - still[1][i]) < 1e-6
                for i in range(3)
            )
            ctx.check(
                "right_branch_independent",
                same,
                "Posing the left branch must not move the right branch.",
            )

    return ctx.report()


object_model = build_object_model()
```
