---
title: 'Portal Gantry with Vertical Slide'
description: 'Base SDK three-axis-style portal gantry: a rigid portal frame, a bridge carriage that translates horizontally along the beam rails, and a hanging Z slide that translates vertically. Uses native mesh geometry.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - gantry
  - portal
  - portal gantry
  - linear axis
  - prismatic
  - prismatic articulation
  - carriage
  - vertical slide
  - z axis
  - rails
  - motion limits
  - box geometry
  - cylinder geometry
  - extrude with holes
---
# Portal Gantry with Vertical Slide

This base-SDK example keeps the real three-part split from a portal-gantry motion
stage: a rigid portal frame that carries the bridge beam, a bridge carriage that
rides the beam rails on a horizontal prismatic axis, and a separate hanging slide
that drops on a vertical prismatic axis. It is useful for queries such as
`portal gantry`, `gantry`, `linear axis`, `prismatic carriage`, `vertical slide`,
`Z axis`, `rails`, `MotionLimits`, and `prismatic articulation`.

The modeling patterns worth copying are:

- a rigid frame root built from mesh box primitives (legs, base, cross beam, rails).
- two stacked prismatic articulations: horizontal carriage travel along the beam,
  then vertical slide travel hanging from the carriage.
- `ExtrudeWithHolesGeometry` for a lightening-window face plate on the carriage.
- decisive `expect_*` pose checks proving the carriage and slide actually move.

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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

# Beam runs along X. Carriage rides the beam in X. Slide drops in -Z.
BEAM_Z = 0.66
BEAM_LEN = 0.62
CARRIAGE_TRAVEL = 0.13
SLIDE_TRAVEL = 0.18


def _box(geom_xyz, center):
    g = BoxGeometry(geom_xyz)
    g.translate(center[0], center[1], center[2])
    return g


def _union(*parts):
    acc = parts[0]
    for part in parts[1:]:
        acc = boolean_union(acc, part)
    return acc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_axis")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    carriage_body = model.material("carriage_body", rgba=(0.18, 0.20, 0.23, 1.0))
    slide_body = model.material("slide_body", rgba=(0.86, 0.87, 0.88, 1.0))
    tool_steel = model.material("tool_steel", rgba=(0.42, 0.44, 0.48, 1.0))

    # ---- Frame (root): two legs, a base bar, the bridge beam, and beam rails. ----
    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((BEAM_LEN, 0.30, BEAM_Z + 0.08)),
        mass=40.0,
        origin=Origin(xyz=(0.0, 0.0, BEAM_Z * 0.5)),
    )

    frame_struct = _union(
        # base bar on the ground
        _box((BEAM_LEN, 0.16, 0.04), (0.0, 0.0, 0.02)),
        # left leg
        _box((0.06, 0.10, BEAM_Z - 0.02), (-0.27, 0.0, (BEAM_Z - 0.02) * 0.5 + 0.04)),
        # right leg
        _box((0.06, 0.10, BEAM_Z - 0.02), (0.27, 0.0, (BEAM_Z - 0.02) * 0.5 + 0.04)),
        # bridge beam across the top
        _box((BEAM_LEN, 0.12, 0.08), (0.0, 0.0, BEAM_Z)),
    )
    frame.visual(mesh_from_geometry(frame_struct, "gantry_frame"), material=frame_aluminum)
    # Two steel guide rails on the front of the beam (the carriage rides these).
    frame.visual(
        mesh_from_geometry(_box((0.56, 0.012, 0.018), (0.0, 0.054, BEAM_Z + 0.022)), "gantry_rail_top"),
        material=rail_steel,
    )
    frame.visual(
        mesh_from_geometry(_box((0.56, 0.012, 0.018), (0.0, 0.054, BEAM_Z - 0.022)), "gantry_rail_bottom"),
        material=rail_steel,
    )

    # ---- Carriage: bridge block + a lightened face plate + hanging guide channel. ----
    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((0.17, 0.10, 0.50)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
    )

    carriage_block = _box((0.17, 0.086, 0.14), (0.0, 0.0, 0.0))
    # Face plate (a flat plate that hangs down) with lightening windows.
    window = rounded_rect_profile(0.040, 0.090, 0.010, corner_segments=6)
    face_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.16, 0.42, 0.012),
        [
            window,
        ],
        height=0.020,
        center=True,
    )
    # The extrude lies in XY by default; lay it into the XZ plane and drop it below the block.
    face_plate.rotate_x(pi / 2.0)
    face_plate.translate(0.0, 0.026, -0.20)
    carriage_struct = boolean_union(carriage_block, face_plate)
    carriage.visual(mesh_from_geometry(carriage_struct, "gantry_carriage"), material=carriage_body)

    # Vertical guide rails that the slide rides on.
    carriage.visual(
        mesh_from_geometry(_box((0.022, 0.014, 0.40), (-0.05, 0.040, -0.27)), "carriage_rail_left"),
        material=rail_steel,
    )
    carriage.visual(
        mesh_from_geometry(_box((0.022, 0.014, 0.40), (0.05, 0.040, -0.27)), "carriage_rail_right"),
        material=rail_steel,
    )

    # ---- Slide: vertical plate + carriage block + a spindle/tool stub. ----
    slide = model.part("slide")
    slide.inertial = Inertial.from_geometry(
        Box((0.13, 0.08, 0.26)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
    )

    slide_struct = _union(
        # vertical slide plate
        _box((0.12, 0.030, 0.26), (0.0, 0.052, -0.12)),
        # carriage block that grips the rails
        _box((0.082, 0.050, 0.11), (0.0, 0.040, -0.04)),
    )
    slide.visual(mesh_from_geometry(slide_struct, "gantry_slide"), material=slide_body)
    # Tool spindle stub pointing forward (+Y) at the bottom of the slide.
    spindle = CylinderGeometry(radius=0.018, height=0.045, radial_segments=24)
    spindle.rotate_x(pi / 2.0)
    spindle.translate(0.0, 0.072, -0.24)
    slide.visual(mesh_from_geometry(spindle, "gantry_spindle"), material=tool_steel)

    # ---- Articulations. ----
    # Horizontal carriage travel along the beam (X). Frame is the parent.
    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.054, BEAM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-CARRIAGE_TRAVEL, upper=CARRIAGE_TRAVEL, effort=450.0, velocity=0.8),
    )
    # Vertical slide travel hanging from the carriage (down, -Z). Positive q lowers it.
    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slide,
        origin=Origin(xyz=(0.0, 0.012, -0.04)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=220.0, velocity=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_part("slide")
    carriage_axis = object_model.get_articulation("frame_to_carriage")
    slide_axis = object_model.get_articulation("carriage_to_slide")

    ctx.check("frame_present", frame is not None, "Expected a frame part.")
    ctx.check("carriage_present", carriage is not None, "Expected a carriage part.")
    ctx.check("slide_present", slide is not None, "Expected a slide part.")

    # Carriage translates +X when its prismatic axis is driven positive.
    with ctx.pose({carriage_axis: 0.0}):
        zero = ctx.part_world_aabb(carriage)
    with ctx.pose({carriage_axis: CARRIAGE_TRAVEL}):
        moved = ctx.part_world_aabb(carriage)
    if zero is not None and moved is not None:
        dx = float(((moved[0][0] + moved[1][0]) - (zero[0][0] + zero[1][0])) * 0.5)
        ctx.check(
            "carriage_moves_x",
            dx >= CARRIAGE_TRAVEL * 0.9,
            f"carriage X shift={dx!r}",
        )

    # Slide drops in -Z when its prismatic axis is driven positive.
    with ctx.pose({slide_axis: 0.0}):
        zero_s = ctx.part_world_aabb(slide)
    with ctx.pose({slide_axis: SLIDE_TRAVEL}):
        moved_s = ctx.part_world_aabb(slide)
    if zero_s is not None and moved_s is not None:
        dz = float(((moved_s[0][2] + moved_s[1][2]) - (zero_s[0][2] + zero_s[1][2])) * 0.5)
        ctx.check(
            "slide_drops_z",
            dz <= -SLIDE_TRAVEL * 0.9,
            f"slide Z shift={dz!r}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
