---
title: 'Coaxial Rotary Stack'
description: 'Base SDK example of a nested industrial turntable with a grounded pedestal, slew carrier, and top platter sharing one vertical centerline through two stacked revolute stages.'
tags:
  - sdk
  - base sdk
  - articulation
  - revolute
  - coaxial
  - turntable
  - slew bearing
  - nested rotation
  - pedestal
  - platter
  - mesh geometry
  - lathe geometry
  - cylinder geometry
---
# Coaxial Rotary Stack

This base-SDK example reproduces the core layout of a nested industrial
turntable: a grounded pedestal, a slew carrier, and a top platter that all
rotate about the same vertical axis. It is useful for queries such as
`coaxial rotation`, `nested turntable`, `slew bearing`, `stacked revolute
stages`, and `shared centerline articulation`.

The patterns worth copying are:

- two `REVOLUTE` stages chained on a common `+Z` axis so each stage spins
  relative to the one below it.
- `LatheGeometry` revolved profiles for the bearing-stepped pedestal and
  carrier hubs, plus `CylinderGeometry` for plates and collars.
- `Origin` offsets that stack the joints up the centerline while keeping the
  motion axes coaxial.

```python
from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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

PRIMARY_AXIS_LIMIT = pi
SECONDARY_AXIS_LIMIT = pi

PEDESTAL_TOP_Z = 0.060
PLATTER_STAGE_Z = 0.060


def _build_pedestal_shape():
    # Revolved pedestal: wide ground foot, tapered column, bearing seat on top.
    column = LatheGeometry(
        [
            (0.130, 0.000),
            (0.130, 0.018),
            (0.090, 0.022),
            (0.078, 0.040),
            (0.078, 0.052),
            (0.092, 0.054),
            (0.092, 0.060),
            (0.000, 0.060),
        ],
        segments=64,
    )
    # Inner bearing-race ring that the slew carrier seats into.
    race = CylinderGeometry(0.064, 0.012, radial_segments=48).translate(0.0, 0.0, 0.066)
    return boolean_union(column, race)


def _build_slew_shape():
    # Revolved slew carrier: hub that seats on the pedestal bearing and a flange
    # that the platter stage rides on, all on the centerline.
    hub = LatheGeometry(
        [
            (0.060, 0.000),
            (0.060, 0.026),
            (0.072, 0.030),
            (0.072, 0.044),
            (0.040, 0.048),
            (0.040, 0.060),
            (0.000, 0.060),
        ],
        segments=64,
    )
    # Upper bearing seat for the platter stage.
    seat = CylinderGeometry(0.034, 0.010, radial_segments=48).translate(0.0, 0.0, 0.063)
    return boolean_union(hub, seat)


def _build_platter_shape():
    # Flat top platter with a short centering collar underneath.
    plate = CylinderGeometry(0.150, 0.018, radial_segments=72).translate(0.0, 0.0, 0.009)
    collar = CylinderGeometry(0.032, 0.014, radial_segments=48).translate(0.0, 0.0, -0.007)
    return boolean_union(plate, collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_industrial_turntable")

    machine_base = model.material("machine_base", rgba=(0.20, 0.22, 0.24, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    tooling_orange = model.material("tooling_orange", rgba=(0.83, 0.39, 0.10, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        mesh_from_geometry(_build_pedestal_shape(), "pedestal_base"),
        material=machine_base,
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 0.078)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )

    slew_carrier = model.part("slew_carrier")
    slew_carrier.visual(
        mesh_from_geometry(_build_slew_shape(), "slew_carrier"),
        material=bearing_steel,
    )
    slew_carrier.inertial = Inertial.from_geometry(
        Box((0.144, 0.144, 0.073)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
    )

    top_platter = model.part("top_platter")
    top_platter.visual(
        mesh_from_geometry(_build_platter_shape(), "top_platter"),
        material=tooling_orange,
    )
    top_platter.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.032)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    # Stage 1: pedestal carries the slew carrier; carrier hub seats on the
    # pedestal bearing race at the top of the column.
    model.articulation(
        "base_to_slew",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=slew_carrier,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-PRIMARY_AXIS_LIMIT,
            upper=PRIMARY_AXIS_LIMIT,
            effort=80.0,
            velocity=1.2,
        ),
    )
    # Stage 2: slew carrier carries the top platter on the same centerline,
    # one stage higher, so both joints share the vertical axis.
    model.articulation(
        "slew_to_platter",
        ArticulationType.REVOLUTE,
        parent=slew_carrier,
        child=top_platter,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-SECONDARY_AXIS_LIMIT,
            upper=SECONDARY_AXIS_LIMIT,
            effort=45.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal_base")
    carrier = object_model.get_part("slew_carrier")
    platter = object_model.get_part("top_platter")
    stage1 = object_model.get_articulation("base_to_slew")
    stage2 = object_model.get_articulation("slew_to_platter")

    ctx.check("pedestal_present", pedestal is not None, "Expected a pedestal base part.")
    ctx.check("carrier_present", carrier is not None, "Expected a slew carrier part.")
    ctx.check("platter_present", platter is not None, "Expected a top platter part.")

    ctx.check(
        "stage1_revolute",
        stage1 is not None and stage1.type == ArticulationType.REVOLUTE,
        "base_to_slew should be a revolute stage.",
    )
    ctx.check(
        "stage2_revolute",
        stage2 is not None and stage2.type == ArticulationType.REVOLUTE,
        "slew_to_platter should be a revolute stage.",
    )

    # Both stages must rotate about the same vertical centerline.
    ctx.check(
        "coaxial_axes",
        stage1 is not None
        and stage2 is not None
        and stage1.axis == (0.0, 0.0, 1.0)
        and stage2.axis == (0.0, 0.0, 1.0),
        "Both rotary stages must share the +Z axis to be coaxial.",
    )

    # At a neutral pose the stack should read top-down: pedestal, carrier, platter.
    with ctx.pose({stage1: 0.0, stage2: 0.0}):
        ctx.expect_above(carrier, pedestal, axis="z")
        ctx.expect_above(platter, carrier, axis="z")

    # Both stages turning together must keep the platter centered on the axis.
    with ctx.pose({stage1: 0.7, stage2: -0.5}):
        aabb = ctx.part_world_aabb(platter)
        ctx.check("platter_aabb_present", aabb is not None, "Expected a platter AABB.")
        if aabb is not None:
            mins, maxs = aabb
            cx = float(mins[0] + maxs[0]) * 0.5
            cy = float(mins[1] + maxs[1]) * 0.5
            ctx.check(
                "platter_stays_on_axis",
                abs(cx) <= 0.005 and abs(cy) <= 0.005,
                f"platter center=({cx:.4f}, {cy:.4f}) should stay near the centerline.",
            )

    return ctx.report()


object_model = build_object_model()
```
