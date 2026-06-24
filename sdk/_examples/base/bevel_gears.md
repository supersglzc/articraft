---
title: 'Bevel Gears'
description: 'Base SDK example showing a standalone bevel gear and a meshing right-angle bevel pair built from the native gear classes and mounted on a demo plate.'
tags:
  - sdk
  - base sdk
  - gear
  - bevel gear
  - bevel gear pair
  - right angle drive
  - mesh geometry
---
# Bevel Gears

This base-SDK example reproduces the classic bevel-gear teaching layout: a single
bevel gear next to a meshing right-angle bevel pair, both authored directly in
meters. The native gear classes (`BevelGear`, `BevelGearPair`) build watertight
`MeshGeometry` solids, so each one drops straight into `mesh_from_geometry(...)`.

The bevel pair already unions its gear and pinion into one connected solid that
meshes at the pitch cone, so it reads as a single drive unit. A thin demo plate
ties both showpieces into one connected object, the standalone gear spins on a
vertical revolute axis, and the pair spins on its gear axis to demonstrate the
right-angle drive.

Note: the native gear classes use an approximate, polygon-lofted involute/bevel
tooth profile rather than an exact spherical-involute surface, so tooth flanks
are a faithful approximation rather than analytically exact bevel geometry.

```python
from __future__ import annotations

import numpy as np

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BevelGear,
    BevelGearPair,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

MODULE = 0.001
FACE_WIDTH = 0.004

SINGLE_X = -0.018
PAIR_X = 0.018
PLATE_TOP_Z = 0.002


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bevel_gears")
    plate_finish = model.material("demo_plate_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    steel = model.material("gear_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    brass = model.material("gear_brass", rgba=(0.72, 0.58, 0.24, 1.0))

    # Demo base plate that carries both showpieces.
    base = model.part("base_plate")
    base.visual(
        Box((0.060, 0.030, PLATE_TOP_Z)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z / 2.0)),
        material=plate_finish,
        name="base_plate_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, PLATE_TOP_Z)), mass=0.20
    )

    # Standalone bevel gear, axis vertical, seated on the plate.
    single_geom = BevelGear(
        module=MODULE,
        teeth_number=18,
        cone_angle=45.0,
        face_width=FACE_WIDTH,
    )
    single = model.part("single_gear")
    single.visual(
        mesh_from_geometry(single_geom, "single_gear"),
        material=steel,
        name="single_gear_body",
    )
    single.inertial = Inertial.from_geometry(
        Box((0.010, 0.010, 0.006)), mass=0.02
    )

    # Meshing right-angle bevel pair: gear + pinion unioned into one drive unit.
    pair_geom = BevelGearPair(
        module=MODULE,
        gear_teeth=24,
        pinion_teeth=16,
        face_width=FACE_WIDTH,
        axis_angle=90.0,
    )
    pair = model.part("bevel_pair")
    pair.visual(
        mesh_from_geometry(pair_geom, "bevel_pair"),
        material=brass,
        name="bevel_pair_body",
    )
    pair.inertial = Inertial.from_geometry(
        Box((0.016, 0.016, 0.016)), mass=0.04
    )

    # Standalone gear spins about its vertical axis on the plate.
    model.articulation(
        "plate_to_single",
        ArticulationType.REVOLUTE,
        parent=base,
        child=single,
        origin=Origin(xyz=(SINGLE_X, 0.0, PLATE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0, lower=-6.28, upper=6.28),
    )

    # Bevel pair spins about the gear (vertical) axis, driving the pinion.
    model.articulation(
        "plate_to_pair",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pair,
        origin=Origin(xyz=(PAIR_X, 0.0, PLATE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0, lower=-6.28, upper=6.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    single = object_model.get_part("single_gear")
    pair = object_model.get_part("bevel_pair")

    ctx.check("base_present", base is not None, "Expected a base plate part.")
    ctx.check("single_present", single is not None, "Expected a standalone bevel gear.")
    ctx.check("pair_present", pair is not None, "Expected a bevel gear pair.")
    if base is None or single is None or pair is None:
        return ctx.report()

    single_axis = object_model.get_articulation("plate_to_single")
    pair_axis = object_model.get_articulation("plate_to_pair")

    # The standalone gear is a compact toothed disc.
    s_aabb = ctx.part_world_aabb(single)
    ctx.check("single_aabb", s_aabb is not None, "Expected an AABB for the single gear.")
    if s_aabb is not None:
        smin, smax = s_aabb
        s_size = tuple(float(smax[i] - smin[i]) for i in range(3))
        ctx.check(
            "single_diameter",
            0.010 <= max(s_size[0], s_size[1]) <= 0.030,
            f"single size={s_size!r}",
        )

    # The bevel pair is a right-angle unit, so it has meaningful extent on
    # both the gear axis (Z) and the pinion axis (X).
    p_aabb = ctx.part_world_aabb(pair)
    ctx.check("pair_aabb", p_aabb is not None, "Expected an AABB for the bevel pair.")
    if p_aabb is not None:
        pmin, pmax = p_aabb
        p_size = tuple(float(pmax[i] - pmin[i]) for i in range(3))
        ctx.check(
            "pair_right_angle_extent",
            p_size[0] >= 0.008 and p_size[2] >= 0.008,
            f"pair size={p_size!r}",
        )

    # Both showpieces spin without colliding with each other at rest or rotated.
    with ctx.pose({single_axis: 0.0, pair_axis: 0.0}):
        ctx.expect_gap(single, pair, axis="x", max_gap=1.0, max_penetration=0.0)
    with ctx.pose({single_axis: np.pi / 2.0, pair_axis: np.pi / 2.0}):
        ctx.expect_gap(single, pair, axis="x", max_gap=1.0, max_penetration=0.0)

    return ctx.report()


object_model = build_object_model()
```
