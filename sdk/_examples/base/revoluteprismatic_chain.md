---
title: 'Revolute-prismatic Chain'
description: 'Base SDK inspection arm with a rotary base hinge feeding a linear extension stage, built from native mesh geometry and boolean cuts.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - articulation
  - revolute
  - prismatic
  - mixed topology
  - inspection arm
  - kinematic chain
  - base hinge
  - linear stage
  - boolean difference
  - motion limits
---
# Revolute-prismatic Chain

This base-SDK example reproduces a compact inspection arm whose kinematic chain
mixes joint topologies: a rotary base hinge (REVOLUTE) lifts a pivot arm, and a
linear extension stage (PRISMATIC) telescopes out of that arm to carry an
inspection head. It is a strong reference for queries such as
`revolute prismatic chain`, `mixed topology arm`, `inspection arm`,
`base hinge plus linear stage`, and `boolean_difference mounting bores`.

The modeling patterns worth copying are:

- a fixed `base_mount` plate built from a `BoxGeometry` with `boolean_difference`
  bolt-hole counterbores, topped by a cylindrical pivot hub.
- a `pivot_arm` revolved/boxed body whose part frame sits on the hinge line so a
  positive REVOLUTE angle lifts the arm upward.
- an `extension_stage` rail that slides along the arm via a PRISMATIC joint and
  carries an inspection head with a glass lens and an amber status light.
- realistic `MotionLimits` for both the rotary and the linear stage.

```python
from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)


def _base_mount_geom() -> "MeshGeometry":
    # Steel mounting plate with four counterbored bolt holes and a pivot hub.
    plate = BoxGeometry((0.140, 0.090, 0.018)).translate(0.0, 0.0, 0.009)
    for x_pos in (-0.055, 0.055):
        for y_pos in (-0.032, 0.032):
            bore = CylinderGeometry(radius=0.006, height=0.030).translate(
                x_pos, y_pos, 0.009
            )
            plate = boolean_difference(plate, bore)
    # Raised pillar that lifts the hinge axis off the plate.
    pillar = BoxGeometry((0.060, 0.070, 0.090)).translate(0.0, 0.0, 0.063)
    body = boolean_union(plate, pillar)
    # Pivot hub aligned with the hinge axis (axis runs along Y at z=0.095).
    hub = CylinderGeometry(radius=0.024, height=0.080).translate(0.0, 0.0, 0.095)
    hub = hub.rotate_x(pi / 2.0)
    return boolean_union(body, hub)


def _pivot_arm_geom() -> "MeshGeometry":
    # Aluminum arm: a hinge boss at the base end and a square rail housing that
    # runs out along +X to receive the sliding extension stage.
    boss = CylinderGeometry(radius=0.022, height=0.070).rotate_x(pi / 2.0)
    arm = BoxGeometry((0.150, 0.044, 0.040)).translate(0.073, 0.0, 0.0)
    body = boolean_union(boss, arm)
    # Hollow the rail housing so the extension stage can nest into it.
    channel = BoxGeometry((0.140, 0.030, 0.026)).translate(0.090, 0.0, 0.0)
    return boolean_difference(body, channel)


def _extension_rail_geom() -> "MeshGeometry":
    # Anodized square rail that telescopes out of the pivot arm.
    return BoxGeometry((0.170, 0.026, 0.022)).translate(0.085, 0.0, 0.0)


def _inspection_head_geom() -> "MeshGeometry":
    # Matte-black housing at the free end of the rail.
    housing = BoxGeometry((0.046, 0.040, 0.036)).translate(0.214, 0.0, 0.0)
    snout = CylinderGeometry(radius=0.016, height=0.024).rotate_y(pi / 2.0)
    snout = snout.translate(0.245, 0.0, 0.0)
    return boolean_union(housing, snout)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_arm")

    powder_steel = model.material("powder_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    zinc = model.material("zinc", rgba=(0.70, 0.72, 0.76, 1.0))
    aluminum = model.material("aluminum", rgba=(0.67, 0.70, 0.74, 1.0))
    anodized_dark = model.material("anodized_dark", rgba=(0.30, 0.33, 0.36, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.52, 0.72, 0.84, 0.55))
    amber = model.material("amber", rgba=(0.93, 0.64, 0.17, 0.95))

    base_mount = model.part("base_mount")
    base_mount.visual(
        mesh_from_geometry(_base_mount_geom(), "base_mount"),
        material=powder_steel,
    )
    for x_pos in (-0.055, 0.055):
        for y_pos in (-0.032, 0.032):
            base_mount.visual(
                Cylinder(radius=0.0058, length=0.004),
                origin=Origin(xyz=(x_pos, y_pos, 0.016)),
                material=zinc,
            )
    base_mount.inertial = Inertial.from_geometry(
        Box((0.140, 0.090, 0.110)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        mesh_from_geometry(_pivot_arm_geom(), "pivot_arm"),
        material=aluminum,
    )
    pivot_arm.inertial = Inertial.from_geometry(
        Box((0.150, 0.044, 0.040)),
        mass=0.45,
        origin=Origin(xyz=(0.073, 0.0, 0.0)),
    )

    extension_stage = model.part("extension_stage")
    extension_stage.visual(
        mesh_from_geometry(_extension_rail_geom(), "extension_rail"),
        material=anodized_dark,
    )
    extension_stage.visual(
        mesh_from_geometry(_inspection_head_geom(), "inspection_head"),
        material=matte_black,
    )
    extension_stage.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.249, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
    )
    extension_stage.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.214, 0.0, -0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=amber,
    )
    extension_stage.inertial = Inertial.from_geometry(
        Box((0.170, 0.046, 0.040)),
        mass=0.30,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
    )

    # Rotary base hinge: axis along -Y at the top of the pivot hub so a positive
    # angle lifts the pivot arm upward.
    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=pivot_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.1, effort=18.0, velocity=1.5),
    )
    # Linear extension stage: slides out along the arm's +X axis.
    model.articulation(
        "stage_extension",
        ArticulationType.PRISMATIC,
        parent=pivot_arm,
        child=extension_stage,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.14, effort=12.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_mount = object_model.get_part("base_mount")
    pivot_arm = object_model.get_part("pivot_arm")
    extension_stage = object_model.get_part("extension_stage")
    hinge = object_model.get_articulation("base_hinge")
    stage = object_model.get_articulation("stage_extension")

    ctx.check("has_base_mount", base_mount is not None)
    ctx.check("has_pivot_arm", pivot_arm is not None)
    ctx.check("has_extension_stage", extension_stage is not None)
    ctx.check(
        "base_hinge_is_revolute",
        hinge is not None and hinge.type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "stage_extension_is_prismatic",
        stage is not None and stage.type == ArticulationType.PRISMATIC,
    )

    # The extension head should travel further out along +X when the stage
    # slides to its upper limit.
    if extension_stage is not None and stage is not None:
        with ctx.pose({stage: 0.0}):
            retracted = ctx.part_world_aabb(extension_stage)
        with ctx.pose({stage: 0.14}):
            extended = ctx.part_world_aabb(extension_stage)
        if retracted is not None and extended is not None:
            ctx.check(
                "stage_travels_outward",
                extended[1][0] > retracted[1][0] + 0.10,
                f"retracted_max_x={retracted[1][0]!r} extended_max_x={extended[1][0]!r}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
