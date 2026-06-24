---
title: 'Rack and Pinion'
description: 'Base SDK example reproducing a rack-and-pinion drive: a straight rack gear that slides linearly while a round spur pinion meshes with it and spins, authored from the native gear classes on a fixed mounting base.'
tags:
  - sdk
  - base sdk
  - gear
  - rack
  - pinion
  - spur gear
  - linear drive
  - prismatic
  - revolute
  - mesh geometry
---
# Rack and Pinion

This base-SDK example reproduces the teaching intent of the classic rack-and-pinion
demo: a straight toothed `RackGear` that converts rotary motion into linear travel
through a meshing round `SpurGear` pinion. The original parametric build placed a
`module=1`, `z=18` spur pinion next to a straight rack and offset the pinion by its
pitch radius so the teeth engaged. Here the same pairing is produced directly with
the native `SpurGear` and `RackGear` classes, which build watertight `MeshGeometry`
solids that drop straight into `mesh_from_geometry(...)`.

The assembly is anchored on a fixed mounting base with a small bracket and a stub
shaft for the pinion. The rack slides along its length on a prismatic joint, and the
pinion spins on the stub shaft via a revolute joint. The pinion is positioned one
pitch radius above the rack pitch line so the two gears mesh; in a real drive,
turning the pinion would drive the rack linearly, but rack-linear and pinion-rotary
motion live in different motion domains, so they are authored as two independent
joints rather than coupled with a single `Mimic` ratio.

Note: the native gear classes use an approximate, polygon-lofted tooth profile rather
than an exact analytic involute tooth surface, so both the rack teeth and the pinion
teeth are a faithful approximation of the original involute profiles.

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
    RackGear,
    SpurGear,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# module = 1 mm in the original demo, scaled to meters.
MODULE = 0.001
PINION_TEETH = 18
GEAR_WIDTH = 0.006  # common face width (Z) shared by rack and pinion.
BORE_D = 0.004  # central bore of the pinion, rides on the stub shaft.

RACK_LENGTH = 0.035
RACK_HEIGHT = 0.004  # solid body below the rack teeth.

# Derived gear dimensions (see geometry probe in run_tests()).
PITCH_RADIUS = MODULE * PINION_TEETH / 2.0  # 0.009 m
# The rack pitch line sits at y = 0; the pinion center is one pitch radius above it
# so the tooth circles engage.
PINION_CENTER_Y = PITCH_RADIUS

# Rack center along its length.
RACK_CENTER_X = RACK_LENGTH / 2.0

# Mounting base / bracket geometry.
BASE_THICK = 0.004
BASE_TOP_Y = -0.0055  # the rack body bottom sits at this height.
BRACKET_THICK = 0.003
BRACKET_Z = -0.004  # bracket wall sits on the -Z side of the gear band.
STUB_LEN = 0.009  # stub shaft length along +Z, through the pinion bore.


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_and_pinion")
    steel = model.material("gear_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    base_finish = model.material("base_dark", rgba=(0.20, 0.21, 0.23, 1.0))

    # --- Fixed mounting base (root): a flat plate, a bracket wall on the -Z side,
    # and a stub shaft that the pinion spins on. Built as one connected solid. ---
    plate = BoxGeometry(
        (RACK_LENGTH + 0.010, 0.006, GEAR_WIDTH + 0.012)
    ).translate(
        RACK_CENTER_X,
        BASE_TOP_Y - 0.003,
        GEAR_WIDTH / 2.0,
    )
    bracket = BoxGeometry(
        (0.014, PINION_CENTER_Y - (BASE_TOP_Y - 0.006), BRACKET_THICK)
    ).translate(
        RACK_CENTER_X,
        (PINION_CENTER_Y + (BASE_TOP_Y - 0.006)) / 2.0,
        BRACKET_Z,
    )
    # Stub shaft runs along the pinion axis (Z): from the bracket wall up through
    # the pinion bore. CylinderGeometry is built along Z and centered on the origin.
    stub = CylinderGeometry(BORE_D / 2.0 - 0.0002, STUB_LEN).translate(
        RACK_CENTER_X,
        PINION_CENTER_Y,
        BRACKET_Z + STUB_LEN / 2.0 - 0.001,
    )
    base_solid = boolean_union(boolean_union(plate, bracket), stub)

    base = model.part("base")
    base.visual(
        mesh_from_geometry(base_solid, "base_body"),
        material=base_finish,
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((RACK_LENGTH + 0.010, 0.020, GEAR_WIDTH + 0.012)), mass=0.20
    )

    # --- Rack: a straight toothed bar. Built with x along its length (0..length),
    # y as the tooth height (teeth point +Y), extruded along +Z for the face width. ---
    rack_geom = RackGear(MODULE, RACK_LENGTH, GEAR_WIDTH, RACK_HEIGHT)
    rack = model.part("rack")
    rack.visual(
        mesh_from_geometry(rack_geom, "rack_body"),
        material=steel,
        name="rack_body",
    )
    rack.inertial = Inertial.from_geometry(
        Box((RACK_LENGTH, RACK_HEIGHT + 0.003, GEAR_WIDTH)), mass=0.04
    )

    # --- Pinion: a round spur gear with a central bore, centered on the origin in
    # XY and extruded along +Z. Placed one pitch radius above the rack pitch line. ---
    pinion_geom = SpurGear(MODULE, PINION_TEETH, GEAR_WIDTH, bore_d=BORE_D)
    pinion = model.part("pinion")
    pinion.visual(
        mesh_from_geometry(pinion_geom, "pinion_body"),
        material=steel,
        # SpurGear is centered on XY and spans z in [0, GEAR_WIDTH].
        origin=Origin(xyz=(RACK_CENTER_X, PINION_CENTER_Y, 0.0)),
        name="pinion_body",
    )
    pinion.inertial = Inertial.from_geometry(
        Box((MODULE * PINION_TEETH, MODULE * PINION_TEETH, GEAR_WIDTH)), mass=0.03
    )

    # The rack slides along its length (the world X axis).
    model.articulation(
        "base_to_rack",
        ArticulationType.PRISMATIC,
        parent=base,
        child=rack,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.5, lower=-0.010, upper=0.010),
    )

    # The pinion spins on the stub shaft (the world Z axis).
    model.articulation(
        "base_to_pinion",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pinion,
        origin=Origin(xyz=(RACK_CENTER_X, PINION_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0, lower=-6.28, upper=6.28),
    )

    return model


def run_tests() -> TestReport:
    import numpy as np

    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rack = object_model.get_part("rack")
    pinion = object_model.get_part("pinion")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("rack_present", rack is not None, "Expected a rack part.")
    ctx.check("pinion_present", pinion is not None, "Expected a pinion part.")
    if base is None or rack is None or pinion is None:
        return ctx.report()

    slide = object_model.get_articulation("base_to_rack")
    spin = object_model.get_articulation("base_to_pinion")

    # The rack is a long, thin toothed bar: much longer along X than tall in Y.
    r_aabb = ctx.part_world_aabb(rack)
    ctx.check("rack_aabb", r_aabb is not None, "Expected an AABB for the rack.")
    if r_aabb is not None:
        rmin, rmax = r_aabb
        r_size = tuple(float(rmax[i] - rmin[i]) for i in range(3))
        ctx.check(
            "rack_length",
            RACK_LENGTH * 0.9 <= r_size[0] <= RACK_LENGTH * 1.1,
            f"rack size={r_size!r}",
        )
        ctx.check(
            "rack_is_long_bar",
            r_size[0] > r_size[1] * 3.0,
            f"rack size={r_size!r}",
        )

    # The pinion is a round toothed disc whose pitch diameter is module * teeth.
    p_aabb = ctx.part_world_aabb(pinion)
    ctx.check("pinion_aabb", p_aabb is not None, "Expected an AABB for the pinion.")
    if p_aabb is not None:
        pmin, pmax = p_aabb
        p_size = tuple(float(pmax[i] - pmin[i]) for i in range(3))
        pitch_d = MODULE * PINION_TEETH
        ctx.check(
            "pinion_diameter",
            pitch_d * 0.9 <= max(p_size[0], p_size[1]) <= pitch_d * 1.4,
            f"pinion size={p_size!r}",
        )

    # The rack and pinion mesh: at the neutral pose their teeth bands overlap in the
    # mesh (Y) direction near the contact line.
    with ctx.pose({slide: 0.0, spin: 0.0}):
        ctx.expect_overlap(pinion, rack, axes="x", min_overlap=0.001)

    # The pinion spins freely about its shaft without leaving the contact zone.
    with ctx.pose({slide: 0.0, spin: np.pi / 4.0}):
        ctx.expect_overlap(pinion, rack, axes="x", min_overlap=0.001)

    # The rack slides linearly along X relative to the base.
    base_aabb = ctx.part_world_aabb(base)
    if base_aabb is not None and r_aabb is not None:
        with ctx.pose({slide: 0.0}):
            a0 = ctx.part_world_aabb(rack)
        with ctx.pose({slide: 0.008}):
            a1 = ctx.part_world_aabb(rack)
        if a0 is not None and a1 is not None:
            dx = float(a1[0][0] - a0[0][0])
            ctx.check(
                "rack_translates_along_x",
                dx > 0.006,
                f"rack x shift={dx!r}",
            )

    return ctx.report()


object_model = build_object_model()
```
