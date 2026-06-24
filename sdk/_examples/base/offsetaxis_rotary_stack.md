---
title: 'Offset-axis Rotary Stack'
description: 'Base SDK serial rotary stack where each revolute stage spins on a laterally displaced (offset) axis, built from native mesh carriers.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - articulation
  - revolute
  - revolute articulation
  - serial chain
  - offset axis
  - rotary stack
  - stage carrier
  - motion limits
  - cylinder geometry
  - box geometry
---
# Offset-axis Rotary Stack

This base-SDK example reproduces an offset-axis rotary stack: a fixed base frame
carries three serial revolute stages, but unlike a coaxial stack each stage is
displaced laterally in space so the rotation axes do not share a single line. It
is useful for queries such as `offset axis`, `rotary stack`, `serial revolute
chain`, `stage carrier`, and `MotionLimits`.

The patterns worth copying are:

- a serial `base -> stage1 -> stage2 -> stage3` revolute chain where each child
  is parented to the previous stage rather than the base.
- articulation origins with nonzero lateral offset so successive spin axes are
  displaced from one another, while every stage keeps the same `+Y` axis.
- native mesh carriers (`CylinderGeometry`/`BoxGeometry` + `boolean_union`) that
  bridge from each joint frame to the next, so the stack reads as one connected
  arm rather than floating blocks.

```python
from __future__ import annotations

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

# Lateral offsets between successive spin axes (meters).
STAGE1_OFFSET = (0.135, 0.0, 0.045)
STAGE2_OFFSET = (0.110, 0.0, -0.038)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rotary_module(hub_radius: float, hub_height: float) -> "BoxGeometry":
    """A turret-style rotary module: a vertical hub barrel on a short pedestal.

    The hub barrel is centered on the local origin and aligned with the +Y spin
    axis (CylinderGeometry extends along local Z, so rotate onto Y).
    """
    barrel = CylinderGeometry(hub_radius, hub_height, radial_segments=32).rotate_x(
        1.5707963267948966
    )
    collar = CylinderGeometry(hub_radius * 1.18, hub_height * 0.30, radial_segments=32).rotate_x(
        1.5707963267948966
    )
    collar.translate(0.0, hub_height * 0.40, 0.0)
    return boolean_union(barrel, collar)


def _carrier_arm(
    name: str,
    *,
    hub_radius: float,
    hub_height: float,
    reach: tuple[float, float, float],
) -> "BoxGeometry":
    """Rotary module at the joint frame plus an arm reaching to the next stage.

    `reach` is the local vector from this stage's spin axis to the next stage's
    spin axis, so the arm physically bridges the lateral offset and the stack
    stays a single connected solid.
    """
    module = _rotary_module(hub_radius, hub_height)

    rx, ry, rz = reach
    length = (rx * rx + rz * rz) ** 0.5
    if length < 1.0e-6:
        return module

    arm = BoxGeometry((length, hub_height * 0.55, hub_radius * 0.9))
    # Lay the arm flat along the reach direction in the XZ plane.
    angle = _atan2(rz, rx)
    arm.rotate_y(-angle)
    arm.translate(rx * 0.5, ry, rz * 0.5)
    return boolean_union(module, arm)


def _atan2(y: float, x: float) -> float:
    from math import atan2

    return atan2(y, x)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_axis_rotary_stack")

    frame_dark = model.material("frame_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    module_gray = model.material("module_gray", rgba=(0.44, 0.46, 0.49, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.18, 0.38, 0.74, 1.0))
    tool_light = model.material("tool_light", rgba=(0.73, 0.76, 0.80, 1.0))

    # Fixed base frame: a low pedestal carrying the first spin axis at its top.
    base = model.part("base_frame")
    base_plate = BoxGeometry((0.30, 0.10, 0.22))
    base_plate.translate(0.0, -0.07, 0.0)
    base_post = CylinderGeometry(0.055, 0.08, radial_segments=32).rotate_x(1.5707963267948966)
    base_geom = boolean_union(base_plate, base_post)
    base.visual(_save_mesh("base_frame.obj", base_geom), material=frame_dark)
    base.inertial = Inertial.from_geometry(
        Box((0.30, 0.18, 0.22)), mass=8.0, origin=Origin(xyz=(0.0, -0.05, 0.0))
    )

    # Stage 1: rotary module at the base axis, arm reaching to stage-2 axis.
    stage1 = model.part("stage1_carrier")
    stage1.visual(
        _save_mesh(
            "stage1_carrier.obj",
            _carrier_arm(
                "stage1",
                hub_radius=0.050,
                hub_height=0.070,
                reach=STAGE1_OFFSET,
            ),
        ),
        material=module_gray,
    )
    stage1.inertial = Inertial.from_geometry(
        Box((0.18, 0.07, 0.10)), mass=2.4, origin=Origin(xyz=(0.067, 0.0, 0.022))
    )

    # Stage 2: rotary module at stage-2 axis, arm reaching to stage-3 axis.
    stage2 = model.part("stage2_carrier")
    stage2.visual(
        _save_mesh(
            "stage2_carrier.obj",
            _carrier_arm(
                "stage2",
                hub_radius=0.044,
                hub_height=0.062,
                reach=STAGE2_OFFSET,
            ),
        ),
        material=accent_blue,
    )
    stage2.inertial = Inertial.from_geometry(
        Box((0.15, 0.06, 0.09)), mass=1.8, origin=Origin(xyz=(0.055, 0.0, -0.019))
    )

    # Stage 3: terminal platform that the chain orients.
    stage3 = model.part("stage3_platform")
    plat_hub = CylinderGeometry(0.038, 0.050, radial_segments=32).rotate_x(1.5707963267948966)
    plat_top = BoxGeometry((0.090, 0.018, 0.090))
    plat_top.translate(0.0, 0.034, 0.0)
    stage3_geom = boolean_union(plat_hub, plat_top)
    stage3.visual(_save_mesh("stage3_platform.obj", stage3_geom), material=tool_light)
    stage3.inertial = Inertial.from_geometry(
        Box((0.09, 0.05, 0.09)), mass=0.9, origin=Origin(xyz=(0.0, 0.01, 0.0))
    )

    model.articulation(
        "base_to_stage1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.20, effort=18.0, velocity=2.4),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.REVOLUTE,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=STAGE1_OFFSET),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.05, effort=15.0, velocity=2.8),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.REVOLUTE,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=STAGE2_OFFSET),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.00, upper=1.35, effort=10.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    stage1 = object_model.get_part("stage1_carrier")
    stage2 = object_model.get_part("stage2_carrier")
    stage3 = object_model.get_part("stage3_platform")
    ctx.check("base_present", base is not None, "Expected base_frame part.")
    ctx.check("stage1_present", stage1 is not None, "Expected stage1_carrier part.")
    ctx.check("stage2_present", stage2 is not None, "Expected stage2_carrier part.")
    ctx.check("stage3_present", stage3 is not None, "Expected stage3_platform part.")

    j1 = object_model.get_articulation("base_to_stage1")
    j2 = object_model.get_articulation("stage1_to_stage2")
    j3 = object_model.get_articulation("stage2_to_stage3")
    for name, art in (("base_to_stage1", j1), ("stage1_to_stage2", j2), ("stage2_to_stage3", j3)):
        ctx.check(
            f"{name}_revolute",
            art is not None and art.type == ArticulationType.REVOLUTE,
            f"Expected {name} to be REVOLUTE.",
        )

    # The defining feature: stage-to-stage joint frames are laterally offset, so
    # successive spin axes are displaced rather than coaxial.
    ctx.check(
        "stage2_axis_offset",
        abs(STAGE1_OFFSET[0]) > 0.05,
        "Expected stage1->stage2 axis to be laterally offset.",
    )
    ctx.check(
        "stage3_axis_offset",
        abs(STAGE2_OFFSET[0]) > 0.05,
        "Expected stage2->stage3 axis to be laterally offset.",
    )

    return ctx.report()


object_model = build_object_model()
```
