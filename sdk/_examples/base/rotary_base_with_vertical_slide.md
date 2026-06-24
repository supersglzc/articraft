---
title: 'Rotary Base with Vertical Slide'
description: 'Base SDK two-axis machine module: a yaw rotary base carrying a vertical prismatic slide carriage, built from native mesh geometry.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - rotary base
  - turntable
  - yaw
  - vertical slide
  - linear slide
  - lift
  - prismatic
  - revolute articulation
  - prismatic articulation
  - machine module
  - two axis stage
  - column
  - carriage
  - motion limits
---
# Rotary Base with Vertical Slide

This base-SDK example reproduces the two-axis machine-module layout: a fixed
base frame, a revolute rotary head that yaws about vertical `Z`, and a vertical
column on the head that carries a prismatic slide carriage which travels up and
down. It is useful for queries such as `rotary base`, `turntable`, `vertical
slide`, `linear lift stage`, `prismatic carriage`, `two-axis module`,
`revolute + prismatic`, and `MotionLimits`.

The modeling patterns worth copying are:

- a flat boxed base frame with chamfered foot pads, authored as native
  `BoxGeometry` solids merged into one mesh per part.
- a revolute `base_yaw` joint about `+Z` that spins the rotary head on the base.
- a vertical guide column carried by the rotary head plus a prismatic
  `column_lift` joint whose carriage slides along the column rails.
- `MotionLimits` chosen so the carriage travels within the column height and the
  rotary head stays within a bounded yaw sweep.

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

# --- Key dimensions (meters) -------------------------------------------------
BASE_W = 0.30
BASE_D = 0.24
BASE_H = 0.07

ROTARY_JOINT_Z = BASE_H  # rotary head sits on top of the base frame
HEAD_PLATE_H = 0.030
HEAD_PLATE_R = 0.105

COLUMN_W = 0.060
COLUMN_D = 0.050
COLUMN_H = 0.260
COLUMN_Y = -0.060  # column stands behind the head center

CARRIAGE_W = 0.110
CARRIAGE_D = 0.040
CARRIAGE_H = 0.070

LIFT_TRAVEL = 0.170
# Lift joint frame: front face of the column, low on the column.
LIFT_JOINT_ORIGIN = (0.0, COLUMN_Y + COLUMN_D / 2.0, HEAD_PLATE_H + 0.020)


def _box_mesh(size, *, origin=(0.0, 0.0, 0.0)) -> BoxGeometry:
    geom = BoxGeometry(size)
    geom.translate(*origin)
    return geom


def _make_base_geometry() -> BoxGeometry:
    # Main slab plus four chamfered-ish foot pads at the corners.
    parts = [_box_mesh((BASE_W, BASE_D, BASE_H), origin=(0.0, 0.0, BASE_H / 2.0))]
    pad = 0.04
    px = BASE_W / 2.0 - pad / 2.0
    py = BASE_D / 2.0 - pad / 2.0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            parts.append(
                _box_mesh((pad, pad, 0.018), origin=(sx * px, sy * py, -0.009))
            )
    geom = parts[0]
    for extra in parts[1:]:
        geom = boolean_union(geom, extra)
    return geom


def _make_rotary_head_geometry() -> BoxGeometry:
    # Round turntable plate plus a tall guide column rising from it.
    plate = CylinderGeometry(HEAD_PLATE_R, HEAD_PLATE_H, radial_segments=48)
    plate.translate(0.0, 0.0, HEAD_PLATE_H / 2.0)

    column = _box_mesh(
        (COLUMN_W, COLUMN_D, COLUMN_H),
        origin=(0.0, COLUMN_Y, HEAD_PLATE_H + COLUMN_H / 2.0),
    )
    geom = boolean_union(plate, column)

    # Two guide rails on the front face of the column.
    rail_h = COLUMN_H - 0.020
    rail_y = COLUMN_Y + COLUMN_D / 2.0 - 0.004
    for sx in (-1.0, 1.0):
        rail = _box_mesh(
            (0.012, 0.016, rail_h),
            origin=(sx * 0.030, rail_y, HEAD_PLATE_H + rail_h / 2.0 + 0.010),
        )
        geom = boolean_union(geom, rail)
    return geom


def _make_carriage_geometry() -> BoxGeometry:
    # Slide block that rides the column rails, with a small front mounting boss.
    block = _box_mesh((CARRIAGE_W, CARRIAGE_D, CARRIAGE_H), origin=(0.0, 0.0, 0.0))
    boss = _box_mesh(
        (0.060, 0.024, 0.044),
        origin=(0.0, CARRIAGE_D / 2.0 + 0.010, 0.0),
    )
    return boolean_union(block, boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_lift_module")

    base_paint = model.material("base_paint", rgba=(0.26, 0.29, 0.31, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.92, 0.46, 0.12, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_geometry(_make_base_geometry(), "base_frame"),
        material=base_paint,
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_H)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
    )

    rotary = model.part("rotary_head")
    rotary.visual(
        mesh_from_geometry(_make_rotary_head_geometry(), "rotary_head"),
        material=machined_steel,
    )
    rotary.inertial = Inertial.from_geometry(
        Box((COLUMN_W, COLUMN_D, COLUMN_H)),
        mass=4.0,
        origin=Origin(xyz=(0.0, COLUMN_Y, HEAD_PLATE_H + COLUMN_H / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_geometry(_make_carriage_geometry(), "carriage"),
        material=safety_orange,
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_W, CARRIAGE_D, CARRIAGE_H)),
        mass=2.5,
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary,
        origin=Origin(xyz=(0.0, 0.0, ROTARY_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.5, upper=2.5, effort=25.0, velocity=1.5),
    )
    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=rotary,
        child=carriage,
        origin=Origin(xyz=LIFT_JOINT_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=LIFT_TRAVEL, effort=120.0, velocity=0.25
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    rotary = object_model.get_part("rotary_head")
    carriage = object_model.get_part("carriage")
    yaw = object_model.get_articulation("base_yaw")
    lift = object_model.get_articulation("column_lift")

    ctx.check("base_present", base is not None, "Expected a base_frame part.")
    ctx.check("rotary_present", rotary is not None, "Expected a rotary_head part.")
    ctx.check("carriage_present", carriage is not None, "Expected a carriage part.")

    # The rotary head should yaw about vertical Z on top of the base.
    with ctx.pose({yaw: 0.0, lift: 0.0}):
        low = ctx.part_world_aabb(carriage)
    with ctx.pose({yaw: 0.0, lift: LIFT_TRAVEL}):
        high = ctx.part_world_aabb(carriage)

    if low is not None and high is not None:
        rise = float(high[0][2] - low[0][2])
        ctx.check(
            "carriage_lifts",
            rise > LIFT_TRAVEL * 0.9,
            f"carriage should rise by about {LIFT_TRAVEL} m, got {rise:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
```
