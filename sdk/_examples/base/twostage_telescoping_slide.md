---
title: 'Two-stage Telescoping Slide'
description: 'Base SDK two-stage telescoping linear slide with a fixed base plate and nested outer, middle, and inner C-channel rails driven by two prismatic stages.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - telescoping slide
  - telescoping
  - linear slide
  - drawer slide
  - nested rails
  - c channel
  - prismatic articulation
  - prismatic
  - motion limits
  - retained insertion
  - extrude geometry
---
# Two-stage Telescoping Slide

This base-SDK example is a compact reference for a two-stage telescoping linear
slide built entirely from native mesh geometry. It is useful for queries such as
`telescoping slide`, `drawer slide`, `nested rails`, `linear stage`, `prismatic
articulation`, and `retained insertion`.

The teaching intent is the nested-stage pattern: a fixed base plate carries an
`outer_rail`, the `middle_rail` slides inside the outer rail, and the
`inner_rail` slides inside the middle rail. Each rail is an open-topped C-channel
(`ExtrudeGeometry` of a C-shaped profile along the travel axis) so the smaller
stage seats inside the larger one. For any nested linear stage, the moving member
should still remain inserted at full travel: each moving rail is sized for the
extended pose and the prismatic travel limit stops the stage before it fully
exits its sleeve.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# Travel axis is +X. Each rail is a C-channel that opens toward +Z so the next
# (smaller) stage nests inside it. Profiles live in the YZ plane and are
# extruded along X to the rail length.

PLATE_LENGTH = 0.40
PLATE_WIDTH = 0.10
PLATE_THICKNESS = 0.010

WALL = 0.004  # C-channel wall/floor thickness

# Rail lengths (along X) and channel cross-sections (Y width, Z height).
OUTER_LENGTH = 0.40
OUTER_WIDTH = 0.072
OUTER_HEIGHT = 0.044

MIDDLE_LENGTH = 0.36
MIDDLE_WIDTH = 0.052
MIDDLE_HEIGHT = 0.034

INNER_LENGTH = 0.32
INNER_WIDTH = 0.034
INNER_HEIGHT = 0.026

# Prismatic travel and the Z height of each nested stage's channel floor.
OUTER_TRAVEL = 0.22
INNER_TRAVEL = 0.20

# Where each child rail starts (X) when collapsed, measured in the parent frame,
# and the vertical seat of the child channel floor inside the parent channel.
OUTER_INSERT = 0.0
INNER_INSERT = 0.0
OUTER_STAGE_Z = WALL
INNER_STAGE_Z = WALL


def _c_channel_profile(width: float, height: float, wall: float) -> list[tuple[float, float]]:
    """Closed CCW C-channel cross-section in the YZ plane.

    The channel floor sits on z=0, walls rise to +z, and the top is open so a
    smaller rail can nest inside. Returned points are (y, z).
    """
    hy = width / 2.0
    return [
        (-hy, 0.0),
        (hy, 0.0),
        (hy, height),
        (hy - wall, height),
        (hy - wall, wall),
        (-hy + wall, wall),
        (-hy + wall, height),
        (-hy, height),
    ]


def _c_rail_mesh(name: str, length: float, width: float, height: float):
    """Extrude a C-channel profile along +X into a managed rail mesh.

    ExtrudeGeometry extrudes a local-XY profile along Z, so we build the C in the
    profile plane, extrude to the rail length, then reorient so the extrusion
    runs along world X and the channel opens toward +Z.
    """
    profile = _c_channel_profile(width, height, WALL)
    geom = ExtrudeGeometry.from_z0(profile, length, cap=True)
    # Profile plane (x'=channel-y, y'=channel-height) extruded along z'=length.
    # Map (x',y',z') -> world so the extrusion runs along +X, the channel height
    # rises along +Z, and the floor sits near z=0.
    geom.rotate_x(1.5707963267948966)  # bring profile y' (channel height) to +Z
    geom.rotate_z(1.5707963267948966)  # bring extrusion z' to +X
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide")

    zinc = model.material("zinc", rgba=(0.80, 0.82, 0.86, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.39, 0.41, 0.45, 1.0))

    # Fixed base plate, top face at z=0 so the outer rail floor seats on it.
    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(PLATE_LENGTH / 2.0, 0.0, -PLATE_THICKNESS / 2.0)),
        name="base_plate_body",
        material=zinc,
    )
    base_plate.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)),
        mass=0.75,
        origin=Origin(xyz=(PLATE_LENGTH / 2.0, 0.0, -PLATE_THICKNESS / 2.0)),
    )

    # Outer rail: rigidly mounted to the base, channel floor on z=0.
    outer_rail = model.part("outer_rail")
    outer_rail.visual(
        _c_rail_mesh("outer_rail", OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT),
        name="outer_rail_body",
        material=rail_steel,
    )
    outer_rail.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=0.42,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    # Middle rail: nests inside the outer channel and slides along +X.
    middle_rail = model.part("middle_rail")
    middle_rail.visual(
        _c_rail_mesh("middle_rail", MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT),
        name="middle_rail_body",
        material=dark_steel,
    )
    middle_rail.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=0.28,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0)),
    )

    # Inner rail: nests inside the middle channel and slides along +X.
    inner_rail = model.part("inner_rail")
    inner_rail.visual(
        _c_rail_mesh("inner_rail", INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT),
        name="inner_rail_body",
        material=rail_steel,
    )
    inner_rail.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=0.18,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base_plate,
        child=outer_rail,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_rail,
        child=middle_rail,
        origin=Origin(xyz=(OUTER_INSERT, 0.0, OUTER_STAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=OUTER_TRAVEL, effort=120.0, velocity=0.40),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_rail,
        child=inner_rail,
        origin=Origin(xyz=(INNER_INSERT, 0.0, INNER_STAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=INNER_TRAVEL, effort=90.0, velocity=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_rail = object_model.get_part("outer_rail")
    middle_rail = object_model.get_part("middle_rail")
    inner_rail = object_model.get_part("inner_rail")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    # Collapsed pose: each moving rail centered in and inserted within its sleeve.
    ctx.expect_within(
        middle_rail,
        outer_rail,
        axes="yz",
        name="middle rail nests inside outer rail (collapsed)",
    )
    ctx.expect_overlap(
        middle_rail,
        outer_rail,
        axes="x",
        min_overlap=0.12,
        name="collapsed middle rail remains inserted in outer rail",
    )
    ctx.expect_within(
        inner_rail,
        middle_rail,
        axes="yz",
        name="inner rail nests inside middle rail (collapsed)",
    )

    rest_mid = ctx.part_world_position(middle_rail)
    rest_inner = ctx.part_world_position(inner_rail)

    # Extend both stages and confirm motion plus retained insertion.
    with ctx.pose({outer_to_middle: OUTER_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_within(
            middle_rail,
            outer_rail,
            axes="yz",
            name="extended middle rail stays centered in outer rail",
        )
        ctx.expect_overlap(
            middle_rail,
            outer_rail,
            axes="x",
            min_overlap=0.03,
            name="extended middle rail still retains insertion in outer rail",
        )
        ctx.expect_overlap(
            inner_rail,
            middle_rail,
            axes="x",
            min_overlap=0.03,
            name="extended inner rail still retains insertion in middle rail",
        )
        ext_mid = ctx.part_world_position(middle_rail)
        ext_inner = ctx.part_world_position(inner_rail)

    ctx.check(
        "middle rail extends along +X",
        rest_mid is not None and ext_mid is not None and ext_mid[0] > rest_mid[0] + 0.02,
        details=f"rest={rest_mid}, extended={ext_mid}",
    )
    ctx.check(
        "inner rail extends further than middle rail along +X",
        rest_inner is not None
        and ext_inner is not None
        and ext_inner[0] > rest_inner[0] + 0.02,
        details=f"rest={rest_inner}, extended={ext_inner}",
    )

    return ctx.report()


object_model = build_object_model()
```
