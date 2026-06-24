---
title: 'Single Prismatic Slider'
description: 'Base SDK machine-tool linear stage: a fixed guide rail and a carriage that slides along it on a single prismatic joint, with end wipers and explicit travel limits.'
tags:
  - sdk
  - base sdk
  - prismatic
  - prismatic joint
  - linear stage
  - linear slider
  - slider
  - carriage
  - guide rail
  - machine tool
  - travel limits
  - motion limits
  - wipers
  - mesh geometry
  - extrude geometry
  - boolean difference
  - rounded rect profile
---
# Single Prismatic Slider

This base-SDK example reproduces the core of a machine-tool linear stage: a
fixed steel guide rail and a carriage block that slides along the rail on a
single `PRISMATIC` articulation. The carriage straddles a raised guide ridge on
the rail, carries black end wipers, and is constrained by symmetric travel
limits. It is a focused reference for queries such as `prismatic joint`,
`linear stage`, `linear slider`, `carriage`, `guide rail`, `travel limits`, and
`MotionLimits`.

The patterns worth copying are:

- a fixed rail root with a raised central guide ridge built from extruded
  profiles, so the carriage has something real to ride on.
- a carriage built as a block with a `boolean_difference` channel cut into its
  underside so it wraps the rail ridge instead of floating above it.
- a single `PRISMATIC` articulation along the rail's long axis with symmetric
  `MotionLimits` that encode the usable stroke.
- pose-driven tests that prove the carriage actually translates along the rail
  axis and stays seated on the ridge through its full travel.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

RAIL_LENGTH = 0.60
RAIL_WIDTH = 0.10
RAIL_BASE_HEIGHT = 0.020
RIDGE_WIDTH = 0.040
RIDGE_HEIGHT = 0.022

CARRIAGE_LENGTH = 0.14
CARRIAGE_WIDTH = 0.12
CARRIAGE_HEIGHT = 0.050
CHANNEL_CLEARANCE = 0.0015

WIPER_THICKNESS = 0.006
WIPER_HEIGHT = 0.016

# Usable half-stroke: the carriage can travel +/- this far from rail center
# without the carriage body overhanging the rail ends.
TRAVEL_HALF_RANGE = (RAIL_LENGTH - CARRIAGE_LENGTH) / 2.0


def _build_rail_mesh():
    """Flat steel base plate with a raised central guide ridge along +X."""
    base = BoxGeometry((RAIL_LENGTH, RAIL_WIDTH, RAIL_BASE_HEIGHT))
    base.translate(0.0, 0.0, RAIL_BASE_HEIGHT / 2.0)

    ridge = BoxGeometry((RAIL_LENGTH, RIDGE_WIDTH, RIDGE_HEIGHT))
    ridge.translate(0.0, 0.0, RAIL_BASE_HEIGHT + RIDGE_HEIGHT / 2.0)

    return boolean_union(base, ridge)


def _build_carriage_mesh():
    """Carriage block with an inverted-channel cut so it wraps the rail ridge."""
    block = ExtrudeGeometry.from_z0(
        rounded_rect_profile(CARRIAGE_LENGTH, CARRIAGE_WIDTH, 0.012),
        CARRIAGE_HEIGHT,
        cap=True,
    )

    # Cut a channel into the underside that matches the ridge plus clearance.
    channel_width = RIDGE_WIDTH + 2.0 * CHANNEL_CLEARANCE
    channel_height = RIDGE_HEIGHT + CHANNEL_CLEARANCE
    channel = BoxGeometry((CARRIAGE_LENGTH * 1.2, channel_width, channel_height))
    channel.translate(0.0, 0.0, channel_height / 2.0)

    return boolean_difference(block, channel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tool_linear_stage")
    model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("carriage_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("wiper_black", rgba=(0.08, 0.08, 0.09, 1.0))

    rail = model.part("rail")
    rail.visual(
        mesh_from_geometry(_build_rail_mesh(), "rail_guide"),
        material="rail_steel",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_BASE_HEIGHT + RIDGE_HEIGHT)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, (RAIL_BASE_HEIGHT + RIDGE_HEIGHT) / 2.0)),
    )

    # The carriage part frame sits at the top of the rail ridge. The carriage
    # body extends upward from z=0 in part-local coordinates.
    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_geometry(_build_carriage_mesh(), "carriage_body"),
        material="carriage_steel",
    )
    carriage.visual(
        Box((WIPER_THICKNESS, CARRIAGE_WIDTH * 0.90, WIPER_HEIGHT)),
        origin=Origin(
            xyz=(CARRIAGE_LENGTH / 2.0 - WIPER_THICKNESS / 2.0, 0.0, WIPER_HEIGHT / 2.0)
        ),
        material="wiper_black",
    )
    carriage.visual(
        Box((WIPER_THICKNESS, CARRIAGE_WIDTH * 0.90, WIPER_HEIGHT)),
        origin=Origin(
            xyz=(-CARRIAGE_LENGTH / 2.0 + WIPER_THICKNESS / 2.0, 0.0, WIPER_HEIGHT / 2.0)
        ),
        material="wiper_black",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    # Place the carriage frame on top of the rail ridge. Positive joint travel
    # slides the carriage toward +X along the rail.
    ridge_top_z = RAIL_BASE_HEIGHT + RIDGE_HEIGHT
    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent="rail",
        child="carriage",
        origin=Origin(xyz=(0.0, 0.0, ridge_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
            effort=1500.0,
            velocity=0.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    joint = object_model.get_articulation("rail_to_carriage")

    ctx.check("rail_present", rail is not None, "Expected a rail part.")
    ctx.check("carriage_present", carriage is not None, "Expected a carriage part.")
    ctx.check(
        "joint_is_prismatic",
        joint is not None and joint.type == ArticulationType.PRISMATIC,
        "Expected a prismatic rail_to_carriage joint.",
    )
    if rail is None or carriage is None or joint is None:
        return ctx.report()

    # Carriage center at mid-travel.
    with ctx.pose({joint: 0.0}):
        center = ctx.part_world_position(carriage)
    # Carriage center at full forward travel.
    with ctx.pose({joint: TRAVEL_HALF_RANGE}):
        forward = ctx.part_world_position(carriage)
    # Carriage center at full backward travel.
    with ctx.pose({joint: -TRAVEL_HALF_RANGE}):
        backward = ctx.part_world_position(carriage)

    ctx.check(
        "carriage_translates_x",
        center is not None
        and forward is not None
        and backward is not None
        and forward[0] > center[0] > backward[0],
        f"center={center!r} forward={forward!r} backward={backward!r}",
    )
    if center is not None and forward is not None:
        ctx.check(
            "x_travel_matches_limit",
            abs((forward[0] - center[0]) - TRAVEL_HALF_RANGE) < 1e-6,
            f"delta={forward[0] - center[0] if center else None!r}",
        )
    # The slider only moves along X; Y and Z of the carriage stay fixed.
    if center is not None and forward is not None and backward is not None:
        ctx.check(
            "no_lateral_drift",
            abs(forward[1] - center[1]) < 1e-9
            and abs(forward[2] - center[2]) < 1e-9
            and abs(backward[1] - center[1]) < 1e-9
            and abs(backward[2] - center[2]) < 1e-9,
            f"center={center!r} forward={forward!r} backward={backward!r}",
        )

    # Carriage stays seated on the rail (vertical overlap) through full travel.
    with ctx.pose({joint: TRAVEL_HALF_RANGE}):
        ctx.expect_overlap(carriage, rail, axes="xy", min_overlap=0.02)
    with ctx.pose({joint: -TRAVEL_HALF_RANGE}):
        ctx.expect_overlap(carriage, rail, axes="xy", min_overlap=0.02)

    return ctx.report()


object_model = build_object_model()
```
