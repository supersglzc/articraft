---
title: 'Three-joint Revolute Chain'
description: 'Base SDK desk-mounted microphone boom built as a 3R chain: base yaw, elbow pitch, and wrist pitch carrying a mic capsule, with all visuals authored as native mesh geometry.'
tags:
  - sdk
  - base sdk
  - articulation
  - revolute
  - three joint chain
  - 3r chain
  - serial chain
  - microphone boom
  - boom arm
  - desk mount
  - clamp
  - lower arm
  - upper arm
  - wrist
  - microphone
  - mesh geometry
  - box geometry
  - cylinder geometry
  - sphere geometry
  - motion limits
  - revolute articulation
---
# Three-joint Revolute Chain

This base-SDK example is a compact reference for a realistic serial 3R revolute
chain. It models a desk-mounted microphone boom where each stage has a clear
role: a clamped base that yaws, a lower arm that pitches at an elbow, an upper
arm that pitches at a wrist, and a microphone capsule at the end. It is useful
for queries such as `three joint chain`, `3R chain`, `serial revolute chain`,
`microphone boom`, `boom arm`, `base yaw`, `elbow pitch`, `wrist pitch`, and
`MotionLimits`.

The modeling patterns worth copying are:

- a strict parent/child revolute chain (`base -> lower_arm -> upper_arm ->
  microphone`) where each child part frame sits on its own joint axis and its
  geometry extends along local `+X` toward the next joint.
- each articulation origin placed at the far end of the parent link
  (`LOWER_ARM_LENGTH`, `UPPER_ARM_LENGTH`) so links connect tip-to-base.
- all visuals authored as native `MeshGeometry` primitives combined with
  `boolean_union`, then exported with `mesh_from_geometry(...)`.
- pose-driven `expect_*` checks that prove the chain reaches and the joints move
  in the intended directions.

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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

LOWER_ARM_LENGTH = 0.260
UPPER_ARM_LENGTH = 0.230


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _make_base_shape() -> "object":
    # A desk clamp: a flat foot, a vertical riser, and a yaw post on top.
    foot = BoxGeometry((0.090, 0.100, 0.022)).translate(0.0, 0.0, -0.085)
    jaw = BoxGeometry((0.060, 0.100, 0.034)).translate(0.0, 0.0, -0.060)
    riser = BoxGeometry((0.050, 0.060, 0.060)).translate(0.0, 0.0, -0.018)
    post = CylinderGeometry(radius=0.020, height=0.030).translate(0.0, 0.0, 0.015)
    shape = boolean_union(foot, jaw)
    shape = boolean_union(shape, riser)
    shape = boolean_union(shape, post)
    return shape


def _make_arm_shape(length: float, name: str) -> "object":
    # A link whose base hub sits on the joint axis at local origin and whose
    # tube extends along +X to the next joint, ending in a small knuckle hub.
    base_hub = CylinderGeometry(radius=0.018, height=0.044).rotate_x(pi / 2.0)
    tube = (
        CylinderGeometry(radius=0.012, height=length)
        .rotate_y(pi / 2.0)
        .translate(length * 0.5, 0.0, 0.0)
    )
    tip_hub = (
        CylinderGeometry(radius=0.018, height=0.040)
        .rotate_x(pi / 2.0)
        .translate(length, 0.0, 0.0)
    )
    shape = boolean_union(base_hub, tube)
    shape = boolean_union(shape, tip_hub)
    return shape


def _make_microphone_mount_shape() -> "object":
    # A small yoke/mount block that bolts the capsule onto the wrist hub.
    hub = CylinderGeometry(radius=0.016, height=0.036).rotate_x(pi / 2.0)
    neck = BoxGeometry((0.040, 0.030, 0.030)).translate(0.024, 0.0, 0.0)
    shape = boolean_union(hub, neck)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_mounted_microphone_boom")

    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.16, 0.16, 0.18, 1.0))
    mic_body = model.material("mic_body", rgba=(0.33, 0.35, 0.38, 1.0))
    charcoal_grille = model.material("charcoal_grille", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(_save_mesh("base_clamp.obj", _make_base_shape()), material=matte_black)
    base.inertial = Inertial.from_geometry(
        Box((0.090, 0.100, 0.110)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        _save_mesh("lower_arm.obj", _make_arm_shape(LOWER_ARM_LENGTH, "lower_arm")),
        material=satin_silver,
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH, 0.044, 0.044)),
        mass=0.45,
        origin=Origin(xyz=(LOWER_ARM_LENGTH * 0.5, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        _save_mesh("upper_arm.obj", _make_arm_shape(UPPER_ARM_LENGTH, "upper_arm")),
        material=satin_silver,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.040, 0.040)),
        mass=0.38,
        origin=Origin(xyz=(UPPER_ARM_LENGTH * 0.5, 0.0, 0.0)),
    )

    microphone = model.part("microphone")
    microphone.visual(
        _save_mesh("microphone_mount.obj", _make_microphone_mount_shape()),
        material=dark_polymer,
    )
    mic_capsule = (
        CylinderGeometry(radius=0.021, height=0.082)
        .rotate_y(pi / 2.0)
        .translate(0.084, 0.0, 0.0)
    )
    microphone.visual(_save_mesh("microphone_body.obj", mic_capsule), material=mic_body)
    grille_band = (
        CylinderGeometry(radius=0.0185, height=0.028)
        .rotate_y(pi / 2.0)
        .translate(0.139, 0.0, 0.0)
    )
    microphone.visual(
        _save_mesh("microphone_grille.obj", grille_band), material=charcoal_grille
    )
    # The rounded grille cap is emitted as a primitive Sphere visual because the
    # native sphere tessellation is not a watertight boolean operand.
    microphone.visual(
        Sphere(radius=0.0185),
        origin=Origin(xyz=(0.153, 0.0, 0.0)),
        material=charcoal_grille,
    )
    microphone.inertial = Inertial.from_geometry(
        Box((0.160, 0.042, 0.042)),
        mass=0.22,
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.75, upper=1.75, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=1.15, effort=8.0, velocity=2.5),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=microphone,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.00, upper=0.75, effort=4.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    microphone = object_model.get_part("microphone")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("lower_arm_present", lower_arm is not None, "Expected a lower_arm part.")
    ctx.check("upper_arm_present", upper_arm is not None, "Expected an upper_arm part.")
    ctx.check("microphone_present", microphone is not None, "Expected a microphone part.")

    base_yaw = object_model.get_articulation("base_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.check(
        "chain_is_revolute",
        all(
            j.articulation_type == ArticulationType.REVOLUTE
            for j in (base_yaw, elbow_pitch, wrist_pitch)
        ),
        "Expected all three joints to be revolute.",
    )

    # Folded vs reached: with the elbow lifted and wrist down the mic tip should
    # sit higher than in the fully extended straight pose.
    with ctx.pose({base_yaw: 0.0, elbow_pitch: 0.0, wrist_pitch: 0.0}):
        straight = ctx.part_world_aabb(microphone)
    with ctx.pose({base_yaw: 0.0, elbow_pitch: 1.0, wrist_pitch: 0.0}):
        lifted = ctx.part_world_aabb(microphone)

    ctx.check(
        "elbow_lifts_microphone",
        straight is not None and lifted is not None and lifted[1][2] > straight[1][2] + 0.05,
        "Positive elbow_pitch should raise the microphone.",
    )

    # Base yaw should swing the microphone in Y away from the X axis.
    with ctx.pose({base_yaw: 1.2, elbow_pitch: 0.0, wrist_pitch: 0.0}):
        ctx.expect_overlap(microphone, base, axes="z", min_overlap=0.0)
        yawed = ctx.part_world_aabb(microphone)
    ctx.check(
        "base_yaw_swings_microphone",
        yawed is not None and max(abs(yawed[0][1]), abs(yawed[1][1])) > 0.20,
        "Base yaw should move the microphone off the +X line in Y.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
