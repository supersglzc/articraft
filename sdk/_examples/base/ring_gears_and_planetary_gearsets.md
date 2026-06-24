---
title: 'Ring Gears and Planetary Gearsets'
description: 'Base SDK example reproducing the ring-gear and planetary-gearset demo: a standalone internal ring gear next to a working sun/planet/ring planetary mechanism, authored from the native gear classes.'
tags:
  - sdk
  - base sdk
  - gear
  - ring gear
  - internal gear
  - planetary gearset
  - epicyclic
  - mesh geometry
---
# Ring Gears and Planetary Gearsets

This base-SDK example reproduces the teaching intent of the original
ring-gear/planetary demo: a standalone internal (ring) gear shown next to a full
planetary gearset built from a meshing sun, three planets, and an enclosing ring.
The native gear classes (`RingGear`, `PlanetaryGearset`) build watertight
`MeshGeometry` solids, so each component drops straight into
`mesh_from_geometry(...)`.

Where the original composed two static assemblies side by side, this version
turns the planetary half into a real articulated mechanism. The ring gear is the
fixed housing/root, a back-plate carrier (fixed to the ring) carries three pins,
the sun gear spins on the central axis, and each planet spins on its own pin so
the sun/planet mesh can be exercised. A thin bridge bar fixes the standalone ring
showpiece to the same housing so the whole demo reads as one connected object.

The sun and planet pitch radii are sourced from the `PlanetaryGearset` definition
(`orbit_r`, `sun`, `planet`, `ring`) so the authored gears mesh at the same
centers the gearset would assemble them.

Note: the native gear classes use an approximate, polygon-lofted involute tooth
profile rather than an exact analytic involute surface, so the ring and planetary
teeth are a faithful approximation rather than analytically exact gear geometry.

```python
from __future__ import annotations

import numpy as np

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    PlanetaryGearset,
    RingGear,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# module = 1 mm in the original demo, authored in meters here.
MODULE = 0.001
SUN_TEETH = 12
PLANET_TEETH = 9
N_PLANETS = 3
WIDTH = 0.005
RIM_WIDTH = 0.003

# Source consistent, meshing geometry from the planetary gearset definition.
_SET = PlanetaryGearset(
    module=MODULE,
    sun_teeth_number=SUN_TEETH,
    planet_teeth_number=PLANET_TEETH,
    width=WIDTH,
    rim_width=RIM_WIDTH,
    n_planets=N_PLANETS,
)
ORBIT_R = _SET.orbit_r
RING_RIM_R = _SET.ring.rim_r
PLANET_RA = _SET.planet.ra
SUN_RA = _SET.sun.ra

# The standalone internal ring-gear showpiece sits to one side.
SHOWPIECE_X = -0.045

# Carrier back plate + pins that hold the planets so nothing floats.
PLATE_T = 0.0015
PIN_LEN = WIDTH + PLATE_T
PIN_R = 0.0018


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_and_planetary_gears")
    steel = model.material("gear_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    brass = model.material("gear_brass", rgba=(0.72, 0.58, 0.24, 1.0))
    plate_finish = model.material("carrier_gray", rgba=(0.30, 0.31, 0.33, 1.0))

    # --- The planetary mechanism: ring is the fixed housing / root. ---
    ring_geom = RingGear(
        module=MODULE,
        teeth_number=SUN_TEETH + PLANET_TEETH * 2,
        width=WIDTH,
        rim_width=RIM_WIDTH,
    )
    ring = model.part("ring")
    ring.visual(
        mesh_from_geometry(ring_geom, "ring_gear"),
        material=steel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="ring_gear_body",
    )
    ring.inertial = Inertial.from_geometry(
        Box((RING_RIM_R * 2, RING_RIM_R * 2, WIDTH)), mass=0.12
    )

    # --- Standalone internal ring gear showpiece, fixed to the housing. ---
    showpiece_geom = RingGear(
        module=MODULE,
        teeth_number=42,
        width=WIDTH,
        rim_width=RIM_WIDTH,
    )
    # A bridge bar spans from the mechanism to the showpiece so the whole demo
    # reads as one connected object (the showpiece is fixed to the ring below).
    bridge_len = abs(SHOWPIECE_X)
    bridge_geom = boolean_union(
        showpiece_geom.translate(SHOWPIECE_X, 0.0, 0.0),
        BoxGeometry((bridge_len, 0.004, WIDTH)).translate(SHOWPIECE_X / 2.0, 0.0, 0.0),
    )
    ring_show = model.part("ring_showpiece")
    ring_show.visual(
        mesh_from_geometry(bridge_geom, "ring_showpiece"),
        material=steel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="ring_showpiece_body",
    )
    ring_show.inertial = Inertial.from_geometry(
        Box((0.045, 0.045, WIDTH)), mass=0.10
    )

    # Carrier: a back plate behind the gears plus three planet pins. It is fixed
    # to the ring so the planets have a real mount (no floating parts).
    plate_z = -WIDTH / 2.0 - PLATE_T / 2.0
    carrier_geom = CylinderGeometry(RING_RIM_R, PLATE_T).translate(0.0, 0.0, plate_z)
    for i in range(N_PLANETS):
        a = i * 2.0 * np.pi / N_PLANETS
        px = np.cos(a) * ORBIT_R
        py = np.sin(a) * ORBIT_R
        pin = CylinderGeometry(PIN_R, PIN_LEN).translate(px, py, plate_z + PIN_LEN / 2.0)
        carrier_geom = boolean_union(carrier_geom, pin)
    carrier = model.part("carrier")
    carrier.visual(
        mesh_from_geometry(carrier_geom, "carrier_body"),
        material=plate_finish,
        name="carrier_shell",
    )
    carrier.inertial = Inertial.from_geometry(
        Box((RING_RIM_R * 2, RING_RIM_R * 2, PLATE_T)), mass=0.06
    )

    # Sun gear at the center, with a bore for the central pin, spinning on the
    # central axis.
    sun_geom = _SET.sun.build(bore_d=PIN_R * 1.6)
    sun = model.part("sun")
    sun.visual(
        mesh_from_geometry(sun_geom, "sun_gear"),
        material=brass,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="sun_gear_body",
    )
    sun.inertial = Inertial.from_geometry(
        Box((SUN_RA * 2, SUN_RA * 2, WIDTH)), mass=0.02
    )

    # Three planet gears, each bored for and spinning on its own pin.
    for i in range(N_PLANETS):
        a = i * 2.0 * np.pi / N_PLANETS
        px = float(np.cos(a) * ORBIT_R)
        py = float(np.sin(a) * ORBIT_R)
        planet_geom = _SET.planet.build(bore_d=PIN_R * 1.6)
        planet = model.part(f"planet_{i + 1}")
        planet.visual(
            mesh_from_geometry(planet_geom, f"planet_{i + 1}_gear"),
            material=steel,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            name=f"planet_{i + 1}_body",
        )
        planet.inertial = Inertial.from_geometry(
            Box((PLANET_RA * 2, PLANET_RA * 2, WIDTH)), mass=0.01
        )
        model.articulation(
            f"pin_to_planet_{i + 1}",
            ArticulationType.REVOLUTE,
            parent=carrier,
            child=planet,
            origin=Origin(xyz=(px, py, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=10.0, lower=-6.28, upper=6.28),
        )

    # Carrier is rigidly fixed to the ring housing.
    model.articulation(
        "ring_to_carrier",
        ArticulationType.FIXED,
        parent=ring,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # The standalone ring showpiece is rigidly fixed to the mechanism housing.
    model.articulation(
        "ring_to_showpiece",
        ArticulationType.FIXED,
        parent=ring,
        child=ring_show,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Sun gear spins about the central axis.
    model.articulation(
        "carrier_to_sun",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=sun,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0, lower=-6.28, upper=6.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ring = object_model.get_part("ring")
    sun = object_model.get_part("sun")
    carrier = object_model.get_part("carrier")
    ring_show = object_model.get_part("ring_showpiece")
    ctx.check("ring_present", ring is not None, "Expected a ring gear part.")
    ctx.check("sun_present", sun is not None, "Expected a sun gear part.")
    ctx.check("carrier_present", carrier is not None, "Expected a carrier part.")
    ctx.check("ring_showpiece_present", ring_show is not None, "Expected a ring showpiece.")
    if ring is None or sun is None or carrier is None:
        return ctx.report()

    planets = [object_model.get_part(f"planet_{i + 1}") for i in range(N_PLANETS)]
    ctx.check("planet_count", all(p is not None for p in planets), "Expected three planets.")

    # The ring housing is a hollow internal gear sized around its rim radius.
    r_aabb = ctx.part_world_aabb(ring)
    if r_aabb is not None:
        rmin, rmax = r_aabb
        r_size = tuple(float(rmax[i] - rmin[i]) for i in range(3))
        ctx.check(
            "ring_outer_diameter",
            RING_RIM_R * 2 * 0.85 <= max(r_size[0], r_size[1]) <= RING_RIM_R * 2 * 1.15,
            f"ring size={r_size!r}",
        )

    sun_spin = object_model.get_articulation("carrier_to_sun")
    p1_spin = object_model.get_articulation("pin_to_planet_1")

    # The sun meshes with the planets: sun and planet_1 overlap near the pitch line.
    with ctx.pose({sun_spin: 0.0, p1_spin: 0.0}):
        ctx.expect_overlap(sun, planets[0], axes="xy", min_overlap=0.0001)
    # The gears keep meshing as the sun and a planet rotate.
    with ctx.pose({sun_spin: np.pi / 2.0, p1_spin: np.pi / 3.0}):
        ctx.expect_overlap(sun, planets[0], axes="xy", min_overlap=0.0001)

    return ctx.report()


object_model = build_object_model()
```
