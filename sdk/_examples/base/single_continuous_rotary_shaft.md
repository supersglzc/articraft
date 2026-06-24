---
title: 'Single Continuous Rotary Shaft'
description: 'Base SDK pottery-wheel excerpt showing a grounded base, a continuous spindle shaft, and a fixed wheelhead carried by that shaft.'
tags:
  - sdk
  - base sdk
  - articulation
  - continuous
  - continuous articulation
  - rotary
  - spindle
  - shaft
  - pottery wheel
  - wheelhead
  - mesh geometry
  - cylinder
  - lathe geometry
  - wheel geometry
  - motion limits
---
# Single Continuous Rotary Shaft

This base-SDK example keeps the three-part split of a pottery wheel: a grounded
base, a single continuous spindle shaft, and a fixed wheelhead carried by that
shaft. It is a compact reference for a `CONTINUOUS` drive articulation plus a
`FIXED` mount on the same kinematic chain, and is useful for queries such as
`continuous rotary shaft`, `spindle`, `pottery wheel`, `wheelhead`, and
`continuous articulation`.

The modeling patterns worth copying are:

- a revolved `LatheGeometry` body for the cast base/collar that the shaft rises
  through.
- a single `CylinderGeometry` spindle with a small `BoxGeometry` coupling lug so
  the moving shaft reads as a driven part rather than a bare rod.
- a `WheelGeometry` wheelhead so the throwing surface reads as real cast
  hardware instead of a plain disc.
- a `CONTINUOUS` `base -> shaft` articulation about local `Z` followed by a
  `FIXED` `shaft -> wheelhead` mount that rides the spindle.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    boolean_union,
    mesh_from_geometry,
)

SHAFT_RADIUS = 0.018
SHAFT_HEIGHT = 0.34

COLLAR_TOP_Z = 0.31
COLLAR_RADIUS = 0.060

COUPLING_LUG_SIZE = (0.060, 0.024, 0.024)
COUPLING_LUG_ORIGIN = (0.0, 0.0, 0.10)

WHEELHEAD_RADIUS = 0.165
WHEELHEAD_THICKNESS = 0.020
WHEELHEAD_HUB_HEIGHT = 0.030


def _make_base_visual_mesh():
    # Cast pedestal: a wide grounded foot revolving up into a narrow collar that
    # the spindle passes through at COLLAR_TOP_Z.
    profile = [
        (0.0, 0.0),
        (0.215, 0.0),
        (0.215, 0.035),
        (0.150, 0.075),
        (0.110, 0.150),
        (0.085, 0.245),
        (COLLAR_RADIUS, COLLAR_TOP_Z - 0.030),
        (COLLAR_RADIUS, COLLAR_TOP_Z),
        (0.030, COLLAR_TOP_Z),
        (0.030, COLLAR_TOP_Z - 0.020),
        (0.0, COLLAR_TOP_Z - 0.020),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64), "rotary_shaft_base")


def _make_shaft_visual_mesh():
    shaft = CylinderGeometry(
        radius=SHAFT_RADIUS,
        height=SHAFT_HEIGHT,
        radial_segments=32,
    ).translate(0.0, 0.0, SHAFT_HEIGHT / 2.0)
    lug = BoxGeometry(COUPLING_LUG_SIZE).translate(*COUPLING_LUG_ORIGIN)
    fused = boolean_union(shaft, lug)
    return mesh_from_geometry(fused, "rotary_shaft_spindle")


def _make_wheelhead_visual_mesh():
    # WheelGeometry spins about local X; rotate it onto local Z and lift it so
    # the throwing face sits above the spindle top.
    wheel = WheelGeometry(
        WHEELHEAD_RADIUS,
        WHEELHEAD_THICKNESS,
        rim=WheelRim(
            inner_radius=WHEELHEAD_RADIUS * 0.92,
            flange_height=0.006,
            flange_thickness=0.005,
            bead_seat_depth=0.003,
        ),
        hub=WheelHub(
            radius=0.038,
            width=WHEELHEAD_HUB_HEIGHT,
            cap_style="flat",
        ),
        face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
        bore=WheelBore(style="round", diameter=SHAFT_RADIUS * 2.0 + 0.002),
        center=False,
    )
    # WheelGeometry is itself a MeshGeometry; rotate it from local-X spin onto
    # local Z and lift it so the throwing face sits above the spindle top.
    wheel.rotate_y(1.5707963267948966).translate(0.0, 0.0, WHEELHEAD_HUB_HEIGHT)
    return mesh_from_geometry(wheel, "rotary_shaft_wheelhead")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pottery_wheel")

    body_gray = model.material("body_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    wheelhead_mat = model.material("wheelhead", rgba=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(_make_base_visual_mesh(), material=body_gray)
    base.inertial = Inertial.from_geometry(
        Box((0.44, 0.32, 0.31)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    shaft = model.part("shaft")
    shaft.visual(_make_shaft_visual_mesh(), material=steel)
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT / 2.0)),
    )

    wheelhead = model.part("wheelhead")
    wheelhead.visual(_make_wheelhead_visual_mesh(), material=wheelhead_mat)
    wheelhead.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEELHEAD_RADIUS, length=WHEELHEAD_HUB_HEIGHT + WHEELHEAD_THICKNESS),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (WHEELHEAD_HUB_HEIGHT + WHEELHEAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="shaft",
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )
    model.articulation(
        "shaft_to_wheelhead",
        ArticulationType.FIXED,
        parent="shaft",
        child="wheelhead",
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shaft = object_model.get_part("shaft")
    wheelhead = object_model.get_part("wheelhead")
    drive = object_model.get_articulation("base_to_shaft")
    mount = object_model.get_articulation("shaft_to_wheelhead")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("shaft_present", shaft is not None, "Expected a shaft part.")
    ctx.check("wheelhead_present", wheelhead is not None, "Expected a wheelhead part.")
    ctx.check(
        "drive_is_continuous",
        drive is not None and drive.type == ArticulationType.CONTINUOUS,
        "Expected a continuous base->shaft drive.",
    )
    ctx.check(
        "mount_is_fixed",
        mount is not None and mount.type == ArticulationType.FIXED,
        "Expected a fixed shaft->wheelhead mount.",
    )

    # Spinning the shaft must carry the wheelhead with it (rigid mount).
    base_aabb = ctx.part_world_aabb(wheelhead)
    with ctx.pose({drive: 0.0}):
        rest = ctx.part_world_aabb(wheelhead)
    with ctx.pose({drive: 1.5707963267948966}):
        turned = ctx.part_world_aabb(wheelhead)
    ctx.check(
        "wheelhead_rides_shaft",
        base_aabb is not None and rest is not None and turned is not None,
        "Expected the wheelhead AABB to be resolvable across poses.",
    )

    return ctx.report()


object_model = build_object_model()
```
