---
title: 'Involute Gear'
description: 'Base SDK example reproducing the twisted involute gear: a single helical (twist-extruded) involute-tooth gear with a central shaft bore, authored from the native gear classes and spun on a demo shaft.'
tags:
  - sdk
  - base sdk
  - gear
  - involute
  - helical gear
  - twisted gear
  - shaft bore
  - mesh geometry
---
# Involute Gear

This base-SDK example reproduces the teaching intent of the classic involute-gear
demo: a single toothed gear built from an involute tooth profile that is then
twist-extruded into a gently helical (twisted) gear body. The original parametric
build assembled one involute tooth flank/tip/root profile, patterned it into a
full `module=1`, `z=20` gear, and `twistExtrude`d it over the gear height. Here
the same form is produced directly with the native `SpurGear` class, which lofts
involute-style tooth flanks and uses a `helix_angle` to twist the tooth band
along the axis, plus a `bore_d` for the central through hole.

The native gear classes build watertight `MeshGeometry` solids, so the gear drops
straight into `mesh_from_geometry(...)`. A short demo shaft passes through the
bore and ties the assembly into one connected object, and the gear spins on its
central axis via a revolute joint.

Note: the native gear classes use an approximate, polygon-lofted involute tooth
profile (twist-extruded for the helix) rather than an exact analytic involute
tooth surface, so the teeth are a faithful approximation of the original involute
profile.

```python
from __future__ import annotations

import numpy as np

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# module=1, z=20 in the original demo, scaled to meters (module = 1 mm).
MODULE = 0.001
TEETH = 20
GEAR_WIDTH = 0.020
# Gentle helix reproduces the original twist-extruded tooth band.
HELIX_ANGLE = 15.0
BORE_D = 0.006

# Demo shaft that passes through the bore so the gear is a connected assembly.
SHAFT_D = 0.0058
SHAFT_LEN = 0.034


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="involute_gear")
    steel = model.material("gear_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    shaft_finish = model.material("shaft_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    # Fixed central shaft is the root; the gear rides on it and spins.
    shaft = model.part("shaft")
    shaft.visual(
        mesh_from_geometry(
            CylinderGeometry(SHAFT_D / 2.0, SHAFT_LEN),
            "shaft_body",
        ),
        material=shaft_finish,
        name="shaft_body",
    )
    shaft.inertial = Inertial.from_geometry(
        Box((SHAFT_D, SHAFT_D, SHAFT_LEN)), mass=0.02
    )

    # Single helical (twisted) involute gear with a central bore.
    gear_geom = SpurGear(
        MODULE,
        TEETH,
        GEAR_WIDTH,
        helix_angle=HELIX_ANGLE,
        bore_d=BORE_D,
    )
    gear = model.part("gear")
    gear.visual(
        mesh_from_geometry(gear_geom, "gear_body"),
        material=steel,
        # SpurGear is built centered on z; place its mid-plane at the shaft center.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="gear_body",
    )
    gear.inertial = Inertial.from_geometry(
        Box((MODULE * TEETH, MODULE * TEETH, GEAR_WIDTH)), mass=0.05
    )

    # The gear spins about the shaft (Z) axis.
    model.articulation(
        "shaft_to_gear",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=gear,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0, lower=-6.28, upper=6.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    gear = object_model.get_part("gear")

    ctx.check("shaft_present", shaft is not None, "Expected a shaft part.")
    ctx.check("gear_present", gear is not None, "Expected a gear part.")
    if shaft is None or gear is None:
        return ctx.report()

    spin = object_model.get_articulation("shaft_to_gear")

    # The gear is a toothed disc whose pitch diameter is module * teeth.
    g_aabb = ctx.part_world_aabb(gear)
    ctx.check("gear_aabb", g_aabb is not None, "Expected an AABB for the gear.")
    if g_aabb is not None:
        gmin, gmax = g_aabb
        g_size = tuple(float(gmax[i] - gmin[i]) for i in range(3))
        pitch_d = MODULE * TEETH
        ctx.check(
            "gear_diameter",
            pitch_d * 0.9 <= max(g_size[0], g_size[1]) <= pitch_d * 1.4,
            f"gear size={g_size!r}",
        )
        ctx.check(
            "gear_width",
            GEAR_WIDTH * 0.8 <= g_size[2] <= GEAR_WIDTH * 1.2,
            f"gear size={g_size!r}",
        )

    # The shaft passes through the gear bore: it must reach beyond the gear width.
    s_aabb = ctx.part_world_aabb(shaft)
    if s_aabb is not None:
        smin, smax = s_aabb
        s_height = float(smax[2] - smin[2])
        ctx.check(
            "shaft_protrudes",
            s_height > GEAR_WIDTH,
            f"shaft height={s_height!r}",
        )

    # The gear spins freely about the shaft axis without changing its footprint.
    with ctx.pose({spin: 0.0}):
        ctx.expect_overlap(gear, shaft, axes="xy", min_overlap=BORE_D * 0.5)
    with ctx.pose({spin: np.pi / 2.0}):
        ctx.expect_overlap(gear, shaft, axes="xy", min_overlap=BORE_D * 0.5)

    return ctx.report()


object_model = build_object_model()
```
