---
title: 'Vertical Slide with Wrist Hinge'
description: 'Base SDK Z-axis wrist module with a fixed column, a prismatic lifting carriage, and a revolute wrist nose bracket built from native mesh geometry.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - vertical slide
  - linear slide
  - z axis
  - lift carriage
  - prismatic articulation
  - revolute articulation
  - wrist
  - wrist pitch
  - hinge
  - toolhead
  - gantry
  - linear rail
  - motion limits
  - clevis bracket
---
# Vertical Slide with Wrist Hinge

This base-SDK example reproduces a Z-axis wrist module: a fixed column with a
linear rail, a carriage that rides up and down on a prismatic joint, and a
hinged nose bracket (wrist) that pitches on a revolute joint at the carriage
face. It is a good pattern for lift-and-tilt toolheads and is useful for
queries such as `vertical slide`, `linear slide`, `z axis lift`, `prismatic
carriage`, `wrist pitch hinge`, `toolhead`, and `linear rail`.

The modeling patterns worth copying are:

- a fixed column built from a base plate, a tall extruded body, and a raised
  ground rail so the carriage has something visible to ride on.
- a carriage shell with an open-front pocket cut with `boolean_difference`,
  back-side rail shoes that wrap the rail, and integral hinge ears.
- a `ClevisBracketGeometry` wrist that pins to the carriage ears so the
  revolute pitch axis is co-located with real hinge hardware.
- a PRISMATIC `z_slide` joint (column -> carriage) along `+Z` and a REVOLUTE
  `wrist_pitch` joint (carriage -> wrist) about `+X`, both with `MotionLimits`.

```python
from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ClevisBracketGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# Column geometry (meters).
COLUMN_W = 0.16
COLUMN_D = 0.10
COLUMN_H = 0.46
BASE_W = 0.26
BASE_D = 0.22
BASE_H = 0.030
RAIL_W = 0.060
RAIL_PROUD = 0.022  # how far the rail stands proud of the column front face

# Carriage geometry.
CARRIAGE_W = 0.13
CARRIAGE_H = 0.12
CARRIAGE_D = 0.066
CARRIAGE_FRONT_Y = COLUMN_D / 2.0 + RAIL_PROUD  # front face of column rail

# Z travel.
Z_TRAVEL = 0.22
Z_HOME = 0.16  # carriage frame height at q=0 relative to column origin


def _column_geometry() -> BoxGeometry:
    """Base plate + upright body + proud ground rail, fused into one solid."""
    base = BoxGeometry((BASE_W, BASE_D, BASE_H)).translate(0.0, 0.0, BASE_H / 2.0)
    body = BoxGeometry((COLUMN_W, COLUMN_D, COLUMN_H)).translate(
        0.0, 0.0, BASE_H + COLUMN_H / 2.0
    )
    rail = BoxGeometry((RAIL_W, RAIL_PROUD + 0.004, COLUMN_H * 0.92)).translate(
        0.0,
        COLUMN_D / 2.0 + RAIL_PROUD / 2.0 - 0.002,
        BASE_H + COLUMN_H / 2.0,
    )
    solid = boolean_union(base, body)
    solid = boolean_union(solid, rail)
    return solid


def _carriage_geometry() -> BoxGeometry:
    """Carriage shell built in its own part frame.

    The carriage frame origin sits on the wrist pivot line at the front face of
    the carriage; the body extends back toward the column (-Y) and the hinge
    ears extend forward (+Y).
    """
    # Main block sits behind the pivot line, hugging the column rail.
    block = BoxGeometry((CARRIAGE_W, CARRIAGE_D, CARRIAGE_H)).translate(
        0.0, -CARRIAGE_D / 2.0, 0.0
    )
    # Open the rear face into a pocket so the carriage reads as a slider shell.
    pocket = BoxGeometry(
        (RAIL_W + 0.012, CARRIAGE_D * 0.7, CARRIAGE_H * 0.72)
    ).translate(0.0, -CARRIAGE_D - 0.004, 0.0)
    shell = boolean_difference(block, pocket)

    # Rail shoes: two short flanges either side of the rear pocket that wrap the
    # proud column rail.
    shoe_w = (CARRIAGE_W - RAIL_W) / 2.0 - 0.006
    shoe_off = RAIL_W / 2.0 + 0.006 + shoe_w / 2.0
    for sign in (-1.0, 1.0):
        shoe = BoxGeometry((shoe_w, 0.018, CARRIAGE_H * 0.84)).translate(
            sign * shoe_off, -CARRIAGE_D - 0.009, 0.0
        )
        shell = boolean_union(shell, shoe)

    # Central hinge tongue extending forward of the pivot; it fits inside the
    # wrist clevis U-gap (0.026 wide) so the pin passes through both.
    tongue = BoxGeometry((0.022, 0.034, 0.052)).translate(0.0, 0.012, 0.0)
    shell = boolean_union(shell, tongue)
    return shell


def _wrist_geometry() -> BoxGeometry:
    """Hinged nose bracket: a clevis knuckle pinned at the carriage ears plus a
    forward tool plate, all in the wrist part frame (origin on the pivot).

    ``ClevisBracketGeometry`` is centered, with its base at local -Z, cheeks
    rising toward +Z, the U-gap along X, and a transverse pin bore along X. We
    build it tall enough that the bore sits well above the base, then rotate it
    so the cheeks point forward (+Y) and translate so the bore lands on the
    part origin (the pivot line).
    """
    clevis_h = 0.060
    bore_z = 0.044  # bore center measured up from the clevis base
    clevis = ClevisBracketGeometry(
        (0.052, 0.040, clevis_h),
        gap_width=0.026,
        bore_diameter=0.012,
        bore_center_z=bore_z,
        base_thickness=0.014,
        corner_radius=0.004,
    )
    # Rotate so the original +Z (cheek) direction points forward (+Y): a -90 deg
    # rotation about X maps +Z -> +Y and keeps the bore axis along X.
    clevis.rotate_x(-pi / 2.0)
    # After rotation the bore (originally at local z = -clevis_h/2 + bore_z) sits
    # at local y = clevis_h/2 - bore_z. Shift it back so the bore is at y=0.
    clevis.translate(0.0, -(clevis_h / 2.0 - bore_z), 0.0)
    knuckle = clevis

    # Cross pin through the knuckle, along X, on the pivot line.
    pin = CylinderGeometry(radius=0.0058, height=0.072).rotate_y(pi / 2.0)
    body = boolean_union(knuckle, pin)

    # Forward tool plate the wrist carries.
    plate = BoxGeometry((0.060, 0.046, 0.012)).translate(0.0, 0.060, 0.0)
    body = boolean_union(body, plate)
    # A small spindle stub on the plate so the wrist reads as a toolhead.
    stub = (
        CylinderGeometry(radius=0.010, height=0.040)
        .rotate_x(-pi / 2.0)
        .translate(0.0, 0.086, 0.0)
    )
    body = boolean_union(body, stub)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="z_axis_wrist_module")

    machine_gray = model.material("machine_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    ground_rail = model.material("ground_rail", rgba=(0.74, 0.76, 0.79, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.88, 0.50, 0.12, 1.0))

    column = model.part("column")
    column.visual(
        mesh_from_geometry(_column_geometry(), "column"),
        material=machine_gray,
        name="column_body",
    )
    column.inertial = Inertial.from_geometry(
        Box((COLUMN_W, COLUMN_D, COLUMN_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + COLUMN_H / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_geometry(_carriage_geometry(), "carriage"),
        material=painted_steel,
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_W, CARRIAGE_D, CARRIAGE_H)),
        mass=3.5,
        origin=Origin(xyz=(0.0, -CARRIAGE_D / 2.0, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        mesh_from_geometry(_wrist_geometry(), "wrist"),
        material=safety_orange,
        name="wrist_body",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.060, 0.090, 0.046)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
    )

    # Cosmetic rail cap so the proud rail reads as a separate ground rail.
    column.visual(
        Box((RAIL_W * 0.5, 0.004, COLUMN_H * 0.92)),
        origin=Origin(
            xyz=(0.0, COLUMN_D / 2.0 + RAIL_PROUD, BASE_H + COLUMN_H / 2.0)
        ),
        material=ground_rail,
    )

    # PRISMATIC lift: carriage rides up the column front along +Z.
    # The carriage frame sits at the rail front face, at home height Z_HOME.
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_FRONT_Y, Z_HOME)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=Z_TRAVEL, effort=900.0, velocity=0.35),
    )

    # REVOLUTE wrist pitch about +X at the carriage hinge ears (carriage frame
    # origin is on the pivot line, so the joint origin is the carriage origin).
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.85, upper=0.90, effort=45.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist")
    z_slide = object_model.get_articulation("z_slide")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.check("z_slide_prismatic", z_slide.type == ArticulationType.PRISMATIC, "z_slide must be prismatic")
    ctx.check("wrist_revolute", wrist_pitch.type == ArticulationType.REVOLUTE, "wrist_pitch must be revolute")

    # The carriage should lift clear of its home position when the slide extends.
    with ctx.pose({z_slide: 0.0}):
        low = ctx.part_world_aabb(carriage)
    with ctx.pose({z_slide: Z_TRAVEL}):
        high = ctx.part_world_aabb(carriage)
    if low is not None and high is not None:
        rise = float(high[0][2] - low[0][2])
        ctx.check("carriage_lifts", rise > 0.18, f"carriage rise={rise:.3f}")

    # The wrist should swing forward/up when the pitch joint opens.
    with ctx.pose({z_slide: 0.0, wrist_pitch: 0.0}):
        flat = ctx.part_world_aabb(wrist)
    with ctx.pose({z_slide: 0.0, wrist_pitch: 0.85}):
        tilted = ctx.part_world_aabb(wrist)
    if flat is not None and tilted is not None:
        top_change = float(tilted[1][2] - flat[1][2])
        ctx.check("wrist_pitches_up", top_change > 0.01, f"wrist top change={top_change:.3f}")

    ctx.check("has_column", column is not None, "column part present")
    return ctx.report()


object_model = build_object_model()
```
