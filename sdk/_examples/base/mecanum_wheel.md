---
title: 'Mecanum Wheel'
description: 'Base SDK mecanum wheel with a flanged drum hub, repeated mounting lugs and spokes, and eleven tilted barrel rollers that each spin freely around their own 45-degree axis.'
tags:
  - sdk
  - base sdk
  - mecanum
  - mecanum wheel
  - omni wheel
  - wheel
  - roller
  - tilted rollers
  - hub
  - flange
  - lofted roller
  - continuous articulation
  - wheel spin
  - mesh geometry
---
# Mecanum Wheel

This base-SDK example builds a mecanum wheel: a flanged drum hub carrying a ring
of spokes and mounting lugs, plus eleven barrel-shaped rollers tilted at 45
degrees around the rim. Each roller is a free-spinning part with its own
continuous articulation, which is the defining mechanism of a mecanum wheel
(the hub turns about its main axis while the passive rollers roll about their
tilted axes). It is useful for queries such as `mecanum wheel`, `omni wheel`,
`tilted rollers`, `lofted roller`, `flanged hub`, and `continuous wheel spin`.

The modeling patterns worth copying are:

- a revolved drum hub plus flange annuli built with `LatheGeometry`, fused with
  `boolean_union` and bored out with `boolean_difference`.
- a repeated spoke-and-lug ring placed around the hub at each roller station.
- a barrel roller tread lofted from end radius to a crowned mid radius with
  `LoftGeometry`, composed with a shaft, bushings, and end caps.
- one continuous articulation per roller, each on its own tilted axis, so the
  passive rollers spin independently of the main hub spin.

```python
from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

NUM_ROLLERS = 11
NUM_BOLTS = 6

HUB_THICKNESS = 0.056
HUB_CORE_RADIUS = 0.028
HUB_DRUM_RADIUS = 0.042
HUB_DRUM_THICKNESS = 0.038
HUB_FLANGE_INNER_RADIUS = 0.037
HUB_FLANGE_OUTER_RADIUS = 0.062
HUB_FLANGE_THICKNESS = 0.007
HUB_FLANGE_OFFSET = 0.0185
HUB_BORE_RADIUS = 0.0135
HUB_SPOKE_LENGTH = 0.038
HUB_SPOKE_WIDTH = 0.010
HUB_SPOKE_HEIGHT = 0.036
HUB_SPOKE_CENTER_RADIUS = 0.039
BOLT_CIRCLE_RADIUS = 0.025
BOLT_HEAD_RADIUS = 0.0032
BOLT_HEAD_LENGTH = 0.004
BOLT_HEAD_Z = 0.024

ROLLER_TILT = pi / 4.0
ROLLER_CENTER_RADIUS = 0.079
ROLLER_LENGTH = 0.056
ROLLER_GROOVE_WIDTH = 0.010
ROLLER_END_RADIUS = 0.0105
ROLLER_CROWN_RADIUS = 0.018
ROLLER_SHAFT_RADIUS = 0.0036
ROLLER_BUSH_RADIUS = 0.0068
ROLLER_BUSH_LENGTH = 0.008
ROLLER_ENDCAP_RADIUS = 0.0095
ROLLER_ENDCAP_LENGTH = 0.005
ROLLER_ENDCAP_OFFSET = 0.0255

MOUNT_LUG_RADIUS = 0.015
MOUNT_LUG_LENGTH = 0.010
MOUNT_ARM_LENGTH = 0.026
MOUNT_ARM_WIDTH = 0.016
MOUNT_HOLE_RADIUS = 0.0044


def _disc(radius: float, thickness: float, z_center: float):
    return CylinderGeometry(radius, thickness, radial_segments=48).translate(0.0, 0.0, z_center)


def _annulus(outer_radius: float, inner_radius: float, thickness: float, z_center: float):
    half = thickness / 2.0
    profile = [
        (inner_radius, -half),
        (outer_radius, -half),
        (outer_radius, half),
        (inner_radius, half),
    ]
    return LatheGeometry(profile, segments=48, closed=True).translate(0.0, 0.0, z_center)


def _ring_circle(radius: float) -> list[tuple[float, float, float]]:
    return [
        (radius * cos(2.0 * pi * i / 48.0), radius * sin(2.0 * pi * i / 48.0), 0.0)
        for i in range(48)
    ]


def _build_hub_geometry():
    hub = _disc(HUB_DRUM_RADIUS, HUB_DRUM_THICKNESS, 0.0)
    hub = boolean_union(hub, _disc(HUB_CORE_RADIUS, HUB_THICKNESS, 0.0))
    hub = boolean_union(
        hub,
        _annulus(
            HUB_FLANGE_OUTER_RADIUS,
            HUB_FLANGE_INNER_RADIUS,
            HUB_FLANGE_THICKNESS,
            HUB_FLANGE_OFFSET,
        ),
    )
    hub = boolean_union(
        hub,
        _annulus(
            HUB_FLANGE_OUTER_RADIUS,
            HUB_FLANGE_INNER_RADIUS,
            HUB_FLANGE_THICKNESS,
            -HUB_FLANGE_OFFSET,
        ),
    )

    for index in range(NUM_ROLLERS):
        angle = 2.0 * pi * index / NUM_ROLLERS
        spoke = BoxGeometry((HUB_SPOKE_LENGTH, HUB_SPOKE_WIDTH, HUB_SPOKE_HEIGHT))
        spoke.translate(HUB_SPOKE_CENTER_RADIUS, 0.0, 0.0)
        spoke.rotate_z(angle)
        hub = boolean_union(hub, spoke)

    bore = CylinderGeometry(HUB_BORE_RADIUS, HUB_THICKNESS + 0.01, radial_segments=48)
    return boolean_difference(hub, bore)


def _build_mount_lug_geometry():
    # A short cylindrical pocket boss with a stiffening arm reaching back toward
    # the hub, holed through for the roller axle.
    lug = CylinderGeometry(MOUNT_LUG_RADIUS, MOUNT_LUG_LENGTH, radial_segments=32)
    arm = BoxGeometry((MOUNT_ARM_LENGTH, MOUNT_ARM_WIDTH, MOUNT_LUG_LENGTH))
    arm.translate(-MOUNT_ARM_LENGTH / 2.0, 0.0, 0.0)
    body = boolean_union(lug, arm)
    hole = CylinderGeometry(MOUNT_HOLE_RADIUS, MOUNT_LUG_LENGTH + 0.006, radial_segments=24)
    return boolean_difference(body, hole)


def _build_roller_tread_geometry():
    # Barrel tread: end radius -> crowned mid radius -> waist groove -> mid -> end.
    half_span = (ROLLER_LENGTH - ROLLER_GROOVE_WIDTH) / 2.0
    z0 = -ROLLER_LENGTH / 2.0
    profiles = [
        [(ROLLER_END_RADIUS * cos(t), ROLLER_END_RADIUS * sin(t), z0) for t in _angles()],
        [
            (ROLLER_CROWN_RADIUS * cos(t), ROLLER_CROWN_RADIUS * sin(t), z0 + half_span)
            for t in _angles()
        ],
        [
            (ROLLER_CROWN_RADIUS * cos(t), ROLLER_CROWN_RADIUS * sin(t), z0 + half_span + ROLLER_GROOVE_WIDTH)
            for t in _angles()
        ],
        [
            (ROLLER_END_RADIUS * cos(t), ROLLER_END_RADIUS * sin(t), z0 + ROLLER_LENGTH)
            for t in _angles()
        ],
    ]
    return LoftGeometry(profiles, cap=True, closed=True)


def _angles() -> list[float]:
    return [2.0 * pi * i / 40.0 for i in range(40)]


def _roller_local_geometry():
    # Whole roller authored about local Z (its own spin axis), centered at origin.
    geom = _build_roller_tread_geometry()
    shaft = CylinderGeometry(ROLLER_SHAFT_RADIUS, ROLLER_LENGTH + 0.004, radial_segments=24)
    geom = boolean_union(geom, shaft)
    for side in (-1.0, 1.0):
        bush_z = side * (ROLLER_LENGTH - ROLLER_BUSH_LENGTH) / 2.0
        bush = CylinderGeometry(ROLLER_BUSH_RADIUS, ROLLER_BUSH_LENGTH, radial_segments=24)
        bush.translate(0.0, 0.0, bush_z)
        geom = boolean_union(geom, bush)
        cap = CylinderGeometry(ROLLER_ENDCAP_RADIUS, ROLLER_ENDCAP_LENGTH, radial_segments=24)
        cap.translate(0.0, 0.0, side * ROLLER_ENDCAP_OFFSET)
        geom = boolean_union(geom, cap)
    return geom


def _roller_center(index: int) -> tuple[float, float, float]:
    theta = 2.0 * pi * index / NUM_ROLLERS
    return (ROLLER_CENTER_RADIUS * cos(theta), ROLLER_CENTER_RADIUS * sin(theta), 0.0)


def _normalize(vector):
    x, y, z = vector
    length = sqrt(x * x + y * y + z * z)
    return (x / length, y / length, z / length)


def _roller_axis(index: int) -> tuple[float, float, float]:
    theta = 2.0 * pi * index / NUM_ROLLERS
    tangent = (-sin(theta), cos(theta), 0.0)
    return _normalize(
        (
            tangent[0] * sin(ROLLER_TILT),
            tangent[1] * sin(ROLLER_TILT),
            cos(ROLLER_TILT),
        )
    )


def _axis_rpy(axis) -> tuple[float, float, float]:
    ax, ay, az = _normalize(axis)
    yaw = atan2(ay, ax)
    pitch = atan2(sqrt(ax * ax + ay * ay), az)
    return (0.0, pitch, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mecanum_wheel")

    gunmetal = model.material("gunmetal", rgba=(0.28, 0.31, 0.35, 1.0))
    accent = model.material("accent", rgba=(0.82, 0.46, 0.16, 1.0))
    rubber = model.material("roller_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.70, 1.0))

    hub = model.part("hub")
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=HUB_FLANGE_OUTER_RADIUS, length=HUB_DRUM_THICKNESS),
        mass=0.45,
    )
    hub.visual(mesh_from_geometry(_build_hub_geometry(), "mecanum_hub"), material=gunmetal)

    # Mount lugs ride on the hub at each roller station, tilted to carry the
    # tilted roller axle.
    lug_local = _build_mount_lug_geometry()
    for index in range(NUM_ROLLERS):
        angle = 2.0 * pi * index / NUM_ROLLERS
        lug = lug_local.clone()
        lug.rotate_x(-ROLLER_TILT)
        lug.translate(ROLLER_CENTER_RADIUS, 0.0, 0.0)
        lug.rotate_z(angle)
        hub.visual(mesh_from_geometry(lug, f"mecanum_lug_{index}"), material=gunmetal)

    # Accent discs and bolt heads on both faces.
    for side in (-1.0, 1.0):
        face = "front" if side > 0 else "back"
        hub.visual(
            mesh_from_geometry(_disc(0.024, 0.004, side * 0.026), f"mecanum_accent_{face}"),
            material=accent,
        )
    for index in range(NUM_BOLTS):
        angle = 2.0 * pi * index / NUM_BOLTS
        bx = BOLT_CIRCLE_RADIUS * cos(angle)
        by = BOLT_CIRCLE_RADIUS * sin(angle)
        for side in (-1.0, 1.0):
            face = "front" if side > 0 else "back"
            bolt = CylinderGeometry(BOLT_HEAD_RADIUS, BOLT_HEAD_LENGTH, radial_segments=16)
            bolt.translate(bx, by, side * BOLT_HEAD_Z)
            hub.visual(
                mesh_from_geometry(bolt, f"mecanum_bolt_{index}_{face}"),
                material=accent,
            )

    # Rollers: one free-spinning part each, authored about local Z then mounted
    # on its own tilted axis through a continuous articulation.
    roller_local = _roller_local_geometry()
    for index in range(NUM_ROLLERS):
        roller = model.part(f"roller_{index}")
        roller.inertial = Inertial.from_geometry(
            Cylinder(radius=ROLLER_CROWN_RADIUS, length=ROLLER_LENGTH),
            mass=0.02,
        )
        tread = roller_local.clone()
        roller.visual(mesh_from_geometry(tread, f"mecanum_roller_tread_{index}"), material=rubber)
        shaft = CylinderGeometry(ROLLER_SHAFT_RADIUS, ROLLER_LENGTH + 0.004, radial_segments=24)
        roller.visual(
            mesh_from_geometry(shaft, f"mecanum_roller_shaft_{index}"),
            material=steel,
        )

    for index in range(NUM_ROLLERS):
        center = _roller_center(index)
        axis = _roller_axis(index)
        rpy = _axis_rpy(axis)
        model.articulation(
            f"roller_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent="hub",
            child=f"roller_{index}",
            origin=Origin(xyz=center, rpy=rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hub = object_model.get_part("hub")
    ctx.check("hub_present", hub is not None, "Expected a hub part.")

    rollers = [object_model.get_part(f"roller_{i}") for i in range(NUM_ROLLERS)]
    ctx.check(
        "all_rollers_present",
        all(r is not None for r in rollers),
        "Expected all roller parts.",
    )

    spins = [object_model.get_articulation(f"roller_spin_{i}") for i in range(NUM_ROLLERS)]
    ctx.check(
        "all_roller_spins_present",
        all(s is not None and s.type == ArticulationType.CONTINUOUS for s in spins),
        "Expected continuous spin articulations for every roller.",
    )

    if hub is not None:
        aabb = ctx.part_world_aabb(hub)
        if aabb is not None:
            mins, maxs = aabb
            diameter = max(maxs[0] - mins[0], maxs[1] - mins[1])
            ctx.check(
                "hub_diameter",
                0.110 <= diameter <= 0.135,
                f"hub diameter={diameter!r}",
            )

    if all(r is not None for r in rollers):
        outer = 0.0
        for roller in rollers:
            rb = ctx.part_world_aabb(roller)
            if rb is None:
                continue
            mins, maxs = rb
            for i in range(2):
                outer = max(outer, abs(mins[i]), abs(maxs[i]))
        ctx.check(
            "rollers_form_outer_ring",
            outer >= ROLLER_CENTER_RADIUS,
            f"max roller extent={outer!r}",
        )

    return ctx.report()


object_model = build_object_model()
```
