---
title: 'Dual Independent Finger Chains'
description: 'Base SDK two-finger gripper with one palm carrying two uncoupled proximal/distal phalanx chains, each driven by independent revolute knuckle and tip joints.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - gripper
  - two finger gripper
  - finger
  - phalanx
  - proximal
  - distal
  - knuckle joint
  - articulation
  - revolute
  - kinematic chain
  - branching kinematics
  - rounded rect profile
  - extrude geometry
  - motion limits
---
# Dual Independent Finger Chains

This base-SDK example reproduces the dual-finger gripper decomposition: one
palm body that carries two completely independent finger chains. Each finger is
a proximal phalanx hinged to the palm at a knuckle and a distal phalanx hinged
to the proximal at the tip joint. The two chains share no coupling, so the four
revolute joints move freely; the only symmetry is in the geometry and limits.

It is a good reference for branching but symmetric kinematics: queries such as
`two finger gripper`, `finger chain`, `proximal distal phalanx`, `knuckle
joint`, `independent fingers`, and `revolute kinematic chain`.

The patterns worth copying are:

- a single shared link-shape helper (`_make_link_geometry`) that builds one
  rounded finger phalanx mesh and is reused for all four phalanges.
- a palm shell built from an extruded rounded-rect plate plus two raised
  knuckle bosses, kept connected as one part.
- four `REVOLUTE` joints forming two parallel `palm -> proximal -> distal`
  chains, with mirrored axes so positive motion curls both fingers inward.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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

# Palm plate footprint (X = width across fingers, Y = depth, Z = thickness).
PALM_SIZE = (0.090, 0.060, 0.020)
# Phalanx link footprint. Y is the length of the segment along the finger.
PROXIMAL_SIZE = (0.022, 0.050, 0.018)
DISTAL_SIZE = (0.020, 0.040, 0.016)

# Lateral offset of each knuckle from the palm center, and forward mount line.
ROOT_X_OFFSET = 0.026
ROOT_Y_OFFSET = 0.030
ROOT_Z = 0.010


def _make_palm_geometry(name: str):
    """Rounded palm plate plus two raised knuckle bosses, fused into one body."""
    width, depth, thickness = PALM_SIZE
    plate = ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, depth, 0.010, corner_segments=6),
        thickness,
        cap=True,
    )
    geom = plate
    for sign in (-1.0, 1.0):
        boss = CylinderGeometry(0.012, thickness + 0.008, radial_segments=20)
        boss.translate(sign * ROOT_X_OFFSET, ROOT_Y_OFFSET, (thickness + 0.008) * 0.5)
        geom = boolean_union(geom, boss)
    return mesh_from_geometry(geom, name)


def _make_link_geometry(size: tuple[float, float, float], name: str):
    """One finger phalanx: a rounded extruded bar capped with a rounded tip.

    The part frame sits at the proximal hinge; the segment extends along +Y so
    chained joints can stack child frames at ``y = length``.
    """
    width, length, thickness = size
    bar = ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, length, min(width, 0.008) * 0.5, corner_segments=6),
        thickness,
        cap=True,
    )
    # rounded_rect_profile is centered, so the bar spans y in [-length/2, +length/2].
    # Shift it so the segment runs from the hinge (y=0) toward the tip (+Y).
    bar.translate(0.0, length * 0.5, 0.0)
    tip = CylinderGeometry(width * 0.5, thickness, radial_segments=20)
    tip.translate(0.0, length, thickness * 0.5)
    geom = boolean_union(bar, tip)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_finger_gripper")

    palm_gray = model.material("palm_gray", rgba=(0.24, 0.26, 0.30, 1.0))
    finger_aluminum = model.material("finger_aluminum", rgba=(0.72, 0.75, 0.79, 1.0))

    palm = model.part("palm")
    palm.visual(_make_palm_geometry("palm"), name="palm_shell", material=palm_gray)
    palm.inertial = Inertial.from_geometry(
        Box(PALM_SIZE),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, PALM_SIZE[2] * 0.5)),
    )

    def _add_phalanx(part_name: str, size: tuple[float, float, float], mass: float):
        part = model.part(part_name)
        part.visual(
            _make_link_geometry(size, part_name),
            name=f"{part_name}_shell",
            material=finger_aluminum,
        )
        part.inertial = Inertial.from_geometry(
            Box((size[0], size[1], size[2])),
            mass=mass,
            origin=Origin(xyz=(0.0, size[1] * 0.5, size[2] * 0.5)),
        )
        return part

    left_proximal = _add_phalanx("left_proximal", PROXIMAL_SIZE, 0.08)
    left_distal = _add_phalanx("left_distal", DISTAL_SIZE, 0.05)
    right_proximal = _add_phalanx("right_proximal", PROXIMAL_SIZE, 0.08)
    right_distal = _add_phalanx("right_distal", DISTAL_SIZE, 0.05)

    # Left chain: palm -> proximal -> distal. axis -Z curls the finger inward.
    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(-ROOT_X_OFFSET, ROOT_Y_OFFSET, ROOT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.42, effort=3.0, velocity=3.0),
    )
    model.articulation(
        "left_proximal_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_distal,
        origin=Origin(xyz=(0.0, PROXIMAL_SIZE[1], 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.34, effort=2.0, velocity=3.0),
    )

    # Right chain mirrors the left; axis +Z curls inward from the other side.
    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(ROOT_X_OFFSET, ROOT_Y_OFFSET, ROOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.42, effort=3.0, velocity=3.0),
    )
    model.articulation(
        "right_proximal_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_distal,
        origin=Origin(xyz=(0.0, PROXIMAL_SIZE[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.34, effort=2.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_distal = object_model.get_part("right_distal")

    for name in (
        "palm",
        "left_proximal",
        "left_distal",
        "right_proximal",
        "right_distal",
    ):
        ctx.check(f"{name}_present", object_model.get_part(name) is not None, name)

    left_knuckle = object_model.get_articulation("palm_to_left_proximal")
    left_tip = object_model.get_articulation("left_proximal_to_left_distal")
    right_knuckle = object_model.get_articulation("palm_to_right_proximal")
    right_tip = object_model.get_articulation("right_proximal_to_right_distal")

    # Two independent chains: four revolute joints, none coupled.
    for joint in (left_knuckle, left_tip, right_knuckle, right_tip):
        ctx.check(
            f"{joint.name}_is_revolute",
            joint.type is ArticulationType.REVOLUTE,
            f"{joint.name} type={joint.type}",
        )

    # Open pose: every phalanx should be seated at/above its parent hinge.
    with ctx.pose(
        {
            left_knuckle: 0.0,
            left_tip: 0.0,
            right_knuckle: 0.0,
            right_tip: 0.0,
        }
    ):
        ctx.expect_aabb_overlap(left_proximal, palm, axes="z", min_overlap=0.0)
        ctx.expect_aabb_overlap(right_proximal, palm, axes="z", min_overlap=0.0)

    # Curl only the left chain. The left distal must move while the right chain,
    # being independent, stays put: prove decoupling by checking the right tip
    # frame is unaffected when only left joints are driven.
    with ctx.pose({left_knuckle: 0.0, left_tip: 0.0}):
        left_distal_open = ctx.part_world_position(left_distal)
    with ctx.pose({left_knuckle: 0.40, left_tip: 0.30}):
        left_distal_curled = ctx.part_world_position(left_distal)

    moved = any(
        abs(left_distal_curled[i] - left_distal_open[i]) > 0.005 for i in range(3)
    )
    ctx.check(
        "left_chain_curls",
        moved,
        f"open={left_distal_open!r} curled={left_distal_curled!r}",
    )

    with ctx.pose({right_knuckle: 0.0}):
        right_distal_a = ctx.part_world_position(right_distal)
    with ctx.pose({left_knuckle: 0.40, left_tip: 0.30, right_knuckle: 0.0}):
        right_distal_b = ctx.part_world_position(right_distal)
    independent = all(
        abs(right_distal_a[i] - right_distal_b[i]) < 1e-6 for i in range(3)
    )
    ctx.check(
        "right_chain_independent_of_left",
        independent,
        f"a={right_distal_a!r} b={right_distal_b!r}",
    )

    return ctx.report()


object_model = build_object_model()
```
