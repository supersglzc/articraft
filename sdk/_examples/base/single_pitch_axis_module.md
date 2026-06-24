---
title: 'Single Pitch Axis Module'
description: 'Base SDK motorized tilt cradle: a fixed stand and a tray cradle that pitches about a single centered revolute axis, built from native mesh geometry.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - articulation
  - revolute
  - pitch
  - tilt
  - cradle
  - stand
  - pivot
  - trunnion
  - motorized tilt
  - single axis
  - boolean difference
  - rounded rect profile
  - motion limits
---
# Single Pitch Axis Module

This base-SDK example reproduces a motorized tilt cradle reduced to its core
single-pitch mechanism: a fixed **stand** that carries a **cradle** tray on one
centered revolute (pitch) axis. It is a compact reference for `single pitch
axis`, `tilt cradle`, `trunnion pivot`, `motorized tilt`, and a centered
`REVOLUTE` joint built entirely from native `MeshGeometry`.

The patterns worth copying are:

- A rigid stand assembled from a base plate plus two fork uprights and a pivot
  boss, fused with `boolean_union` so the part is one connected island.
- A cradle tray modeled with `ExtrudeGeometry` over a `rounded_rect_profile`,
  hollowed by `boolean_difference` to read as an open tray, and joined to a
  hub collar that rides on the pivot axis.
- A single centered `REVOLUTE` articulation whose origin sits on the pivot line
  and whose `axis=(0, 1, 0)` tilts the tray fore/aft within `MotionLimits`.
- Rest-pose and posed `expect_*` checks that prove the cradle is carried at the
  pivot and that positive motion actually pitches the tray downward in front.

```python
from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
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

# Real-world scale: a desktop-class motorized tilt cradle, dimensions in meters.
PIVOT_Z = 0.120
TILT_LOWER = -0.6
TILT_UPPER = 0.6

STAND_BASE = (0.180, 0.140, 0.020)
UPRIGHT = (0.026, 0.022, 0.110)
UPRIGHT_Y = 0.052

CRADLE_W = 0.150
CRADLE_D = 0.090
CRADLE_WALL = 0.010
CRADLE_RIM_Z = 0.046


def _make_stand_geometry():
    """Base plate, two fork uprights, and a centered pivot boss, fused into one
    connected solid."""
    base = BoxGeometry(STAND_BASE).translate(0.0, 0.0, STAND_BASE[2] * 0.5)

    upright_z = STAND_BASE[2] + UPRIGHT[2] * 0.5
    left = BoxGeometry(UPRIGHT).translate(0.0, UPRIGHT_Y, upright_z)
    right = BoxGeometry(UPRIGHT).translate(0.0, -UPRIGHT_Y, upright_z)

    # Pivot boss spans between the two uprights along Y at the pivot height.
    boss = CylinderGeometry(0.018, UPRIGHT_Y * 2.0 + UPRIGHT[1], radial_segments=32)
    boss = boss.rotate_x(pi / 2.0).translate(0.0, 0.0, PIVOT_Z)

    solid = boolean_union(base, left)
    solid = boolean_union(solid, right)
    solid = boolean_union(solid, boss)
    return solid


def _make_cradle_geometry():
    """An open tray whose part frame sits on the pivot line: a hub collar on the
    axis plus a hollow tray held above it."""
    # Hub collar centered on the pivot (local origin), bored to read as a sleeve
    # riding the stand boss.
    collar = CylinderGeometry(0.013, UPRIGHT_Y * 2.0 - UPRIGHT[1], radial_segments=32)
    collar = collar.rotate_x(pi / 2.0)
    bore = CylinderGeometry(0.0085, UPRIGHT_Y * 2.0, radial_segments=24)
    bore = bore.rotate_x(pi / 2.0)
    collar = boolean_difference(collar, bore)

    # Tray block above the pivot, then hollowed from the top to form the cradle.
    tray_h = CRADLE_RIM_Z
    tray_cz = 0.030 + tray_h * 0.5
    outer = ExtrudeGeometry.centered(
        rounded_rect_profile(CRADLE_W, CRADLE_D, 0.012),
        tray_h,
    ).translate(0.0, 0.0, tray_cz)
    cavity = ExtrudeGeometry.centered(
        rounded_rect_profile(
            CRADLE_W - 2.0 * CRADLE_WALL,
            CRADLE_D - 2.0 * CRADLE_WALL,
            0.008,
        ),
        tray_h,
    ).translate(0.0, 0.0, tray_cz + CRADLE_WALL)
    tray = boolean_difference(outer, cavity)

    # A neck post connects the tray floor down to the hub collar so the part is
    # one connected island.
    neck = BoxGeometry((0.030, 0.030, 0.030)).translate(0.0, 0.0, 0.015)

    solid = boolean_union(collar, neck)
    solid = boolean_union(solid, tray)
    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motorized_tilt_cradle")
    tower_dark = model.material("tower_dark", rgba=(0.16, 0.18, 0.21, 1.0))
    plate_light = model.material("plate_light", rgba=(0.74, 0.77, 0.80, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_geometry(_make_stand_geometry(), "stand_shell"),
        material=tower_dark,
        name="stand_shell",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.180, 0.140, 0.120)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_geometry(_make_cradle_geometry(), "cradle_tray"),
        material=plate_light,
        name="cradle_tray",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.150, 0.090, 0.050)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    # Single centered pitch axis: positive q rotates by the right-hand rule about
    # +Y, which pitches the front of the tray (+X) downward.
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=TILT_LOWER,
            upper=TILT_UPPER,
            effort=18.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    cradle = object_model.get_part("cradle")
    tilt = object_model.get_articulation("tilt_joint")

    ctx.check("stand_present", stand is not None, "Expected a stand part.")
    ctx.check("cradle_present", cradle is not None, "Expected a cradle part.")
    if stand is None or cradle is None:
        return ctx.report()

    # The hub collar rides the stand pivot boss: they should be in contact, and
    # the cradle tray should sit above the pivot at rest.
    with ctx.pose({tilt: 0.0}):
        ctx.expect_contact(
            cradle,
            stand,
            elem_a="cradle_tray",
            elem_b="stand_shell",
            contact_tol=0.002,
            name="cradle rides the stand pivot",
        )
        rest = ctx.part_element_world_aabb(cradle, elem="cradle_tray")
        ctx.check(
            "tray_above_pivot",
            rest is not None and rest[1][2] > PIVOT_Z,
            f"rest_aabb={rest}",
        )
        rest_front_x = rest[1][0] if rest is not None else 0.0

    # Positive tilt about +Y should pitch the front edge (+X) forward and down:
    # the tray reaches farther in +X and its lowest corner drops.
    with ctx.pose({tilt: TILT_UPPER}):
        tilted = ctx.part_element_world_aabb(cradle, elem="cradle_tray")
        front_x_up = tilted[1][0] if tilted is not None else 0.0
        low_z_up = tilted[0][2] if tilted is not None else 0.0

    with ctx.pose({tilt: TILT_LOWER}):
        tilted_neg = ctx.part_element_world_aabb(cradle, elem="cradle_tray")
        front_x_dn = tilted_neg[1][0] if tilted_neg is not None else 0.0

    ctx.check(
        "positive_tilt_pitches_front_forward",
        front_x_up > rest_front_x + 0.005,
        f"rest_front_x={rest_front_x}, front_x_up={front_x_up}",
    )
    ctx.check(
        "positive_tilt_drops_front_corner",
        low_z_up < PIVOT_Z,
        f"low_z_up={low_z_up}, pivot={PIVOT_Z}",
    )
    ctx.check(
        "negative_tilt_pitches_other_way",
        front_x_dn < rest_front_x + 0.005,
        f"rest_front_x={rest_front_x}, front_x_dn={front_x_dn}",
    )

    return ctx.report()


object_model = build_object_model()
```
