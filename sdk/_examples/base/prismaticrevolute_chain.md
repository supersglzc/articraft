---
title: 'Prismatic-revolute Chain'
description: 'Base SDK inspection fixture: a sliding carriage rides a fixed rail (prismatic) and carries a hinged inspection flap (revolute), built from native mesh geometry.'
tags:
  - sdk
  - base sdk
  - inspection fixture
  - fixture
  - prismatic
  - prismatic articulation
  - revolute
  - revolute articulation
  - sliding carriage
  - linear rail
  - hinged flap
  - kinematic chain
  - motion limits
  - mesh geometry
  - extrude geometry
  - boolean union
---
# Prismatic-revolute Chain

This base-SDK example reproduces a two-degree-of-freedom inspection fixture that
cleanly separates a guided base motion from a distal flap motion. A fixed base
plate carries a profiled rail; a carriage rides that rail with a `PRISMATIC`
slide along `X`; and an inspection flap is hinged to the carriage with a
`REVOLUTE` joint about `Y`. It is a useful reference for queries such as
`prismatic-revolute chain`, `inspection fixture`, `sliding carriage`, `hinged
flap`, and combining a `PRISMATIC` slide with a downstream `REVOLUTE` hinge.

The modeling patterns worth copying are:

- a base plate with a profiled extruded guide rail unioned into one body.
- a carriage shell with two captured guide shoes that wrap the rail, plus a
  raised hinge ear that carries the flap joint, all unioned into one watertight
  body.
- a flap authored from the hinge line outward so the joint frame sits on the
  hinge axis.
- one `PRISMATIC` slide articulation feeding one downstream `REVOLUTE` hinge,
  each with realistic `MotionLimits`.

```python
from __future__ import annotations

# >>> USER_CODE_START
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
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

BASE_L = 0.40
BASE_W = 0.18
BASE_T = 0.030

RAIL_LENGTH = 0.34
RAIL_WIDTH = 0.044
RAIL_HEIGHT = 0.026
RAIL_TOP_Z = BASE_T / 2.0 + RAIL_HEIGHT / 2.0

CARRIAGE_L = 0.110
CARRIAGE_W = 0.130
CARRIAGE_T = 0.028
# The carriage body sits just above the rail top so its shoes can wrap the rail.
CARRIAGE_Z = BASE_T / 2.0 + RAIL_HEIGHT + 0.001 + CARRIAGE_T / 2.0

EAR_X = 0.052
EAR_Z = CARRIAGE_Z + CARRIAGE_T / 2.0 + 0.026

FLAP_L = 0.120
FLAP_W = 0.110
FLAP_T = 0.012

PRISMATIC_LOWER = -0.11
PRISMATIC_UPPER = 0.11
FLAP_UPPER = 1.9


def _save(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _base_geometry():
    # Base plate centered on the origin, top face at +BASE_T/2.
    plate = BoxGeometry((BASE_L, BASE_W, BASE_T))
    # Profiled guide rail: rounded-rect cross section in XY, extruded along its
    # run, then re-oriented so the run is along +X and the height is +Z.
    profile = rounded_rect_profile(RAIL_WIDTH, RAIL_HEIGHT, 0.006, corner_segments=6)
    rail = ExtrudeGeometry.centered(profile, RAIL_LENGTH, cap=True)
    rail.rotate_y(pi / 2.0)
    rail.translate(0.0, 0.0, RAIL_TOP_Z)
    return boolean_union(plate, rail)


def _carriage_geometry():
    # Carriage body straddles the rail just above its top face.
    body = BoxGeometry((CARRIAGE_L, CARRIAGE_W, CARRIAGE_T))
    body.translate(0.0, 0.0, CARRIAGE_Z)
    # Two guide shoes drop down on either side of the rail to capture it.
    shoe = BoxGeometry((CARRIAGE_L, 0.018, RAIL_HEIGHT + 0.004))
    shoe_z = BASE_T / 2.0 + (RAIL_HEIGHT + 0.004) / 2.0
    shoe_y = RAIL_WIDTH / 2.0 + 0.009
    left = shoe.copy().translate(0.0, shoe_y, shoe_z)
    right = shoe.copy().translate(0.0, -shoe_y, shoe_z)
    # Hinge ear rises on the +X end and carries the flap joint.
    ear = BoxGeometry((0.020, CARRIAGE_W * 0.55, EAR_Z - CARRIAGE_Z + CARRIAGE_T / 2.0))
    ear_z = (EAR_Z + (CARRIAGE_Z - CARRIAGE_T / 2.0)) / 2.0
    ear.translate(EAR_X, 0.0, ear_z)
    # Barrel along the hinge (Y) axis at the ear top.
    barrel = CylinderGeometry(0.012, CARRIAGE_W * 0.60, radial_segments=28)
    barrel.rotate_x(pi / 2.0)
    barrel.translate(EAR_X, 0.0, EAR_Z)
    out = boolean_union(body, left)
    out = boolean_union(out, right)
    out = boolean_union(out, ear)
    out = boolean_union(out, barrel)
    return out


def _flap_geometry():
    # Flap authored from the hinge line (local origin) outward along -X so a
    # positive hinge angle lifts the free edge up and away.
    panel = BoxGeometry((FLAP_L, FLAP_W, FLAP_T))
    panel.translate(-FLAP_L / 2.0, 0.0, 0.0)
    # A small knuckle at the hinge line gives the flap a real connection to the
    # carriage barrel rather than a floating panel.
    knuckle = CylinderGeometry(0.012, FLAP_W * 0.40, radial_segments=24)
    knuckle.rotate_x(pi / 2.0)
    return boolean_union(panel, knuckle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_fixture")

    base_steel = model.material("fixture_base", rgba=(0.42, 0.44, 0.48, 1.0))
    carriage_orange = model.material("fixture_carriage", rgba=(0.78, 0.46, 0.14, 1.0))
    flap_yellow = model.material("fixture_flap", rgba=(0.92, 0.77, 0.16, 1.0))

    base = model.part("base")
    base.visual(_save("fixture_base", _base_geometry()), material=base_steel)
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(_save("carriage", _carriage_geometry()), material=carriage_orange)
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_L, CARRIAGE_W, CARRIAGE_T)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_Z)),
    )

    flap = model.part("flap")
    flap.visual(_save("inspection_flap", _flap_geometry()), material=flap_yellow)
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_L, FLAP_W, FLAP_T)),
        mass=0.45,
        origin=Origin(xyz=(-FLAP_L / 2.0, 0.0, 0.0)),
    )

    # The carriage slides along the rail (+X). Its child frame coincides with the
    # base frame at zero, so the carriage geometry already sits at rest pose.
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=PRISMATIC_LOWER,
            upper=PRISMATIC_UPPER,
            effort=120.0,
            velocity=0.25,
        ),
    )
    # The flap hinges about the carriage barrel (Y axis). The flap panel extends
    # along local -X from the hinge, so axis +Y lifts the free edge upward.
    model.articulation(
        "carriage_to_flap",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=flap,
        origin=Origin(xyz=(EAR_X, 0.0, EAR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FLAP_UPPER,
            effort=10.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    flap = object_model.get_part("flap")
    slide = object_model.get_articulation("base_to_carriage")
    hinge = object_model.get_articulation("carriage_to_flap")

    ctx.check("has_base", base is not None, "Expected a base part.")
    ctx.check("has_carriage", carriage is not None, "Expected a carriage part.")
    ctx.check("has_flap", flap is not None, "Expected a flap part.")
    ctx.check(
        "slide_is_prismatic",
        slide is not None and slide.articulation_type == ArticulationType.PRISMATIC,
        "Expected a prismatic base-to-carriage articulation.",
    )
    ctx.check(
        "hinge_is_revolute",
        hinge is not None and hinge.articulation_type == ArticulationType.REVOLUTE,
        "Expected a revolute carriage-to-flap articulation.",
    )

    # The carriage actually translates along +X across its travel, and the flap
    # rides with it (the prismatic-revolute chain stays connected).
    if carriage is not None and flap is not None:
        rest = ctx.part_world_position(carriage)
        with ctx.pose({slide: PRISMATIC_UPPER, hinge: 0.0}):
            extended = ctx.part_world_position(carriage)
            ctx.expect_overlap(flap, carriage, axes="y", min_overlap=0.02)
        ctx.check(
            "carriage_slides_along_x",
            rest is not None and extended is not None and extended[0] > rest[0] + 0.05,
            details=f"rest={rest}, extended={extended}",
        )

        # The flap lifts upward when the hinge opens.
        flap_rest = ctx.part_world_aabb(flap)
        with ctx.pose({slide: 0.0, hinge: FLAP_UPPER}):
            flap_open = ctx.part_world_aabb(flap)
        ctx.check(
            "flap_lifts_when_opened",
            flap_rest is not None
            and flap_open is not None
            and flap_open[1][2] > flap_rest[1][2] + 0.02,
            details=f"rest_aabb={flap_rest}, open_aabb={flap_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
