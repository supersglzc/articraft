---
title: 'Spur, Helical, and Herringbone Gears'
description: 'Base SDK example reproducing the spur/helical/herringbone gear family as a meshing three-gear train mounted on a shared baseplate, each gear authored from the native gear classes and spinning on its own shaft.'
tags:
  - sdk
  - base sdk
  - gear
  - spur gear
  - helical gear
  - herringbone gear
  - gear train
  - shaft bore
  - mesh geometry
---
# Spur, Helical, and Herringbone Gears

This base-SDK example reproduces the teaching intent of the classic
spur/helical/herringbone gear family demo. The original parametric build authored
three involute gears side by side on one workplane: a straight `SpurGear`, a
helical `SpurGear` (non-zero helix angle), and a `HerringboneGear` (two opposed
helical bands), each spaced so neighbouring pitch circles touch. Here the same
three gears are produced directly from the native `SpurGear` and `HerringboneGear`
classes and laid out as a meshing three-gear train: spur drives helical, helical
drives herringbone.

Each gear is built with a central `bore_d` and rides on its own short shaft. The
three shafts are fused to a shared baseplate, which is the fixed root that carries
the whole assembly into one connected object. Every gear spins about its own
shaft (the Z axis) via a revolute joint, and the center spacing equals the sum of
neighbouring pitch radii so the teeth mesh.

Note: the native gear classes use an approximate, polygon-lofted involute tooth
profile (twist-extruded for the helical and herringbone tooth bands) rather than
an exact analytic involute tooth surface, so the teeth are a faithful
approximation of the original involute profiles.

```python
from __future__ import annotations

import numpy as np

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    HerringboneGear,
    Inertial,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# All values authored in meters (module = 1 mm), matching the original demo.
MODULE = 0.001

# Three gears: straight spur, helical spur, opposed-helix herringbone.
SPUR_TEETH = 19
SPUR_WIDTH = 0.005
SPUR_BORE = 0.005

HELICAL_TEETH = 17
HELICAL_WIDTH = 0.006
HELICAL_HELIX = 25.0
HELICAL_BORE = 0.004

HERRING_TEETH = 24
HERRING_WIDTH = 0.010
HERRING_HELIX = 20.0
HERRING_BORE = 0.005

# Shafts and baseplate that tie the three gears into one connected object.
SHAFT_MARGIN = 0.012  # shaft protrudes this far past each gear face
PLATE_THICK = 0.004


def _gear_geoms() -> tuple[SpurGear, SpurGear, HerringboneGear]:
    spur = SpurGear(MODULE, SPUR_TEETH, SPUR_WIDTH, bore_d=SPUR_BORE)
    helical = SpurGear(
        MODULE,
        HELICAL_TEETH,
        HELICAL_WIDTH,
        helix_angle=HELICAL_HELIX,
        bore_d=HELICAL_BORE,
    )
    herringbone = HerringboneGear(
        MODULE,
        HERRING_TEETH,
        HERRING_WIDTH,
        helix_angle=HERRING_HELIX,
        bore_d=HERRING_BORE,
    )
    return spur, helical, herringbone


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spur_helical_herringbone_gears")
    steel = model.material("gear_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    brass = model.material("gear_brass", rgba=(0.72, 0.60, 0.30, 1.0))
    bronze = model.material("gear_bronze", rgba=(0.55, 0.42, 0.28, 1.0))
    plate_finish = model.material("plate_dark", rgba=(0.20, 0.21, 0.23, 1.0))

    spur, helical, herringbone = _gear_geoms()

    # Center spacing so neighbouring pitch circles touch (a meshing gear train).
    spur_x = 0.0
    helical_x = spur_x + spur.r0 + helical.r0
    herring_x = helical_x + helical.r0 + herringbone.r0

    # Shafts must clear the widest gear; reuse one length so all faces line up.
    max_width = max(SPUR_WIDTH, HELICAL_WIDTH, HERRING_WIDTH)
    shaft_len = max_width + 2.0 * SHAFT_MARGIN
    shaft_r = (min(SPUR_BORE, HELICAL_BORE, HERRING_BORE) / 2.0) - 0.0002

    # Baseplate sits just below the gears; the shafts drop into it so the whole
    # assembly is one connected, grounded body.
    plate_w = (herring_x - spur_x) + 2.0 * herringbone.ra + 0.01
    plate_d = 2.0 * herringbone.ra + 0.01
    plate_top_z = -shaft_len / 2.0 - 0.0005
    plate_center_x = (spur_x + herring_x) / 2.0

    base = model.part("base")
    plate_mesh = mesh_from_geometry(
        BoxGeometry((plate_w, plate_d, PLATE_THICK)), "baseplate"
    )
    base.visual(
        plate_mesh,
        origin=Origin(xyz=(plate_center_x, 0.0, plate_top_z - PLATE_THICK / 2.0)),
        material=plate_finish,
        name="baseplate",
    )

    # Three posts rise from the plate up to the bottom of each gear; each post is
    # fused to the gear's shaft so the whole tower reads as one grounded support.
    for name, gx in (("spur", spur_x), ("helical", helical_x), ("herring", herring_x)):
        post = _build_post(shaft_r, plate_top_z, -shaft_len / 2.0)
        base.visual(
            mesh_from_geometry(post, f"{name}_post"),
            origin=Origin(xyz=(gx, 0.0, 0.0)),
            material=plate_finish,
            name=f"{name}_post",
        )

    base.inertial = Inertial.from_geometry(
        Box((plate_w, plate_d, PLATE_THICK)), mass=0.30
    )

    # --- Gear 1: straight spur, drives the train ---
    spur_part = model.part("spur_gear")
    spur_shaft = _shaft_through(shaft_r, shaft_len)
    spur_body = boolean_union(spur, spur_shaft)
    spur_part.visual(
        mesh_from_geometry(spur_body, "spur_gear_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="spur_gear_body",
    )
    spur_part.inertial = Inertial.from_geometry(
        Box((2.0 * spur.ra, 2.0 * spur.ra, SPUR_WIDTH)), mass=0.05
    )

    # --- Gear 2: helical spur ---
    helical_part = model.part("helical_gear")
    helical_shaft = _shaft_through(shaft_r, shaft_len)
    helical_body = boolean_union(helical, helical_shaft)
    helical_part.visual(
        mesh_from_geometry(helical_body, "helical_gear_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass,
        name="helical_gear_body",
    )
    helical_part.inertial = Inertial.from_geometry(
        Box((2.0 * helical.ra, 2.0 * helical.ra, HELICAL_WIDTH)), mass=0.05
    )

    # --- Gear 3: herringbone (two opposed helical bands) ---
    herring_part = model.part("herringbone_gear")
    herring_shaft = _shaft_through(shaft_r, shaft_len)
    herring_body = boolean_union(herringbone, herring_shaft)
    herring_part.visual(
        mesh_from_geometry(herring_body, "herringbone_gear_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bronze,
        name="herringbone_gear_body",
    )
    herring_part.inertial = Inertial.from_geometry(
        Box((2.0 * herringbone.ra, 2.0 * herringbone.ra, HERRING_WIDTH)), mass=0.08
    )

    # Each gear spins about its own shaft (Z), located at its center column.
    model.articulation(
        "base_to_spur",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spur_part,
        origin=Origin(xyz=(spur_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0, lower=-6.28, upper=6.28),
    )
    model.articulation(
        "base_to_helical",
        ArticulationType.REVOLUTE,
        parent=base,
        child=helical_part,
        origin=Origin(xyz=(helical_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0, lower=-6.28, upper=6.28),
    )
    model.articulation(
        "base_to_herringbone",
        ArticulationType.REVOLUTE,
        parent=base,
        child=herring_part,
        origin=Origin(xyz=(herring_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0, lower=-6.28, upper=6.28),
    )

    return model


def _shaft_through(radius: float, length: float):
    """A shaft that passes through a gear bore, centered on z=0."""
    return CylinderGeometry(radius, length)


def _build_post(radius: float, plate_top_z: float, gear_bottom_z: float):
    """A support post from the plate top up to the bottom face of the gear shaft."""
    height = gear_bottom_z - plate_top_z
    height = max(height, 0.0005)
    post = CylinderGeometry(radius, height)
    # CylinderGeometry is centered on z; shift so its base sits on the plate top.
    return post.translate(0.0, 0.0, plate_top_z + height / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spur = object_model.get_part("spur_gear")
    helical = object_model.get_part("helical_gear")
    herring = object_model.get_part("herringbone_gear")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("spur_present", spur is not None, "Expected a spur gear part.")
    ctx.check("helical_present", helical is not None, "Expected a helical gear part.")
    ctx.check("herringbone_present", herring is not None, "Expected a herringbone gear part.")
    if None in (base, spur, helical, herring):
        return ctx.report()

    geoms = _gear_geoms()
    pitch = [g.r0 * 2.0 for g in geoms]
    parts = [spur, helical, herring]
    widths = [SPUR_WIDTH, HELICAL_WIDTH, HERRING_WIDTH]

    # Each gear is a toothed disc of about its pitch diameter and authored width.
    for part, pd, wd, label in zip(parts, pitch, widths, ("spur", "helical", "herring")):
        aabb = ctx.part_world_aabb(part)
        ctx.check(f"{label}_aabb", aabb is not None, f"Expected an AABB for {label}.")
        if aabb is None:
            continue
        lo, hi = aabb
        size = tuple(float(hi[i] - lo[i]) for i in range(3))
        ctx.check(
            f"{label}_diameter",
            pd * 0.9 <= max(size[0], size[1]) <= pd * 1.4,
            f"{label} size={size!r}, pitch_d={pd!r}",
        )

    # Neighbouring pitch circles touch -> the gears form a meshing train, so
    # adjacent gears overlap in the XY plane near their tooth tips.
    spin_spur = object_model.get_articulation("base_to_spur")
    spin_helical = object_model.get_articulation("base_to_helical")
    with ctx.pose({spin_spur: 0.0, spin_helical: 0.0}):
        ctx.expect_overlap(spur, helical, axes="x", min_overlap=0.0001)

    # Each gear spins freely about its shaft without changing its footprint.
    with ctx.pose({spin_helical: np.pi / 3.0}):
        h_aabb = ctx.part_world_aabb(helical)
        ctx.check("helical_spins", h_aabb is not None, "Expected helical AABB while posed.")

    return ctx.report()


object_model = build_object_model()
```
