---
title: 'Vane Array with Independent Pivots'
description: 'Base SDK example of a rigid louver frame carrying a stacked array of independently pivoting vanes, each on its own revolute joint.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - vane
  - vanes
  - louver
  - louvers
  - shutter
  - damper
  - register
  - blade array
  - vane array
  - independent pivots
  - revolute articulation
  - repeated articulation
  - array of joints
  - frame
  - motion limits
  - extrude geometry
  - rounded rect profile
---
# Vane Array with Independent Pivots

This base-SDK example shows how to author a stack of repeated, independently
articulated parts against a single grounded frame. The object is a louver /
register-style vane array: a rigid rectangular frame holds a column of thin
vanes (blades), and each vane pivots on its own revolute joint about a shared
horizontal axis. It is useful for queries such as `vane array`, `louver`,
`shutter`, `damper`, `register`, `independent pivots`, `repeated revolute
articulations`, and `array of joints`.

The modeling patterns worth copying are:

- one grounded frame part built from four extruded rails, with the inner
  rectangular opening left open so the vanes are visible.
- a helper that builds one vane mesh and reuses it for every blade.
- a loop that creates `n` independent revolute articulations, all sharing the
  same axis but at different heights, so each vane can tilt on its own.
- `MotionLimits` that let each vane swing symmetrically about the horizontal.

```python
from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# Overall louver frame, in meters.
FRAME_WIDTH = 0.300
FRAME_HEIGHT = 0.360
FRAME_DEPTH = 0.040
RAIL_THICKNESS = 0.024  # width of each frame rail (the solid border)

# Inner opening that the vanes cover.
OPENING_WIDTH = FRAME_WIDTH - 2.0 * RAIL_THICKNESS
OPENING_HEIGHT = FRAME_HEIGHT - 2.0 * RAIL_THICKNESS

# Vane geometry.
VANE_COUNT = 6
VANE_CHORD = 0.052  # front-to-back-ish height of one vane when flat (along Z)
VANE_THICKNESS = 0.006
VANE_LENGTH = OPENING_WIDTH - 0.006  # slight side clearance inside the opening
VANE_LIMIT = 1.40  # max tilt each way, radians (~80 deg)

# Vertical pivot spacing so the closed vanes evenly fill the opening.
VANE_PITCH = OPENING_HEIGHT / VANE_COUNT
FIRST_PIVOT_Z = (-OPENING_HEIGHT / 2.0) + (VANE_PITCH / 2.0)


def _make_frame_mesh():
    """Four rails forming a rigid rectangular border with an open center."""
    half_w = FRAME_WIDTH / 2.0
    half_h = FRAME_HEIGHT / 2.0
    inner_half_w = OPENING_WIDTH / 2.0
    inner_half_h = OPENING_HEIGHT / 2.0

    # Each rail is a centered box extruded through the frame depth (along Y).
    rail_xy = [
        # (size_x, size_z, center_x, center_z)
        (FRAME_WIDTH, RAIL_THICKNESS, 0.0, half_h - RAIL_THICKNESS / 2.0),  # top
        (FRAME_WIDTH, RAIL_THICKNESS, 0.0, -half_h + RAIL_THICKNESS / 2.0),  # bottom
        (RAIL_THICKNESS, OPENING_HEIGHT, -inner_half_w - RAIL_THICKNESS / 2.0, 0.0),  # left
        (RAIL_THICKNESS, OPENING_HEIGHT, inner_half_w + RAIL_THICKNESS / 2.0, 0.0),  # right
    ]

    frame = None
    for size_x, size_z, cx, cz in rail_xy:
        # Profile in XZ; extrude along its local Z, then orient so depth runs +Y.
        profile = rounded_rect_profile(size_x, size_z, min(size_x, size_z) * 0.18)
        rail = ExtrudeGeometry.centered(profile, FRAME_DEPTH)
        rail.rotate_x(-1.5707963267948966)  # extrude axis Z -> Y
        rail.translate(cx, 0.0, cz)
        frame = rail if frame is None else frame.merge(rail)

    assert inner_half_h > 0.0
    return frame


def _make_vane_mesh():
    """A single thin vane plate spanning the opening width, centered on origin."""
    profile = rounded_rect_profile(VANE_LENGTH, VANE_CHORD, VANE_CHORD * 0.12)
    vane = ExtrudeGeometry.centered(profile, VANE_THICKNESS)
    # Profile lies in XY (length along X, chord along Y) extruded along Z.
    # Rotate so chord lies along Z and thickness along Y (plate faces +/-Y).
    vane.rotate_x(1.5707963267948966)
    return vane


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vane_array")

    frame_finish = model.material("frame_finish", rgba=(0.19, 0.21, 0.24, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.78, 0.80, 0.83, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_make_frame_mesh(), "vane_frame"),
        material=frame_finish,
        name="frame_border",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=2.4,
    )

    vane_mesh = _make_vane_mesh()
    for index in range(VANE_COUNT):
        vane = model.part(f"vane_{index + 1}")
        vane.visual(
            mesh_from_geometry(vane_mesh.clone(), f"vane_blade_{index + 1}"),
            material=blade_finish,
            name="vane_blade",
        )
        vane.inertial = Inertial.from_geometry(
            Box((VANE_LENGTH, VANE_THICKNESS, VANE_CHORD)),
            mass=0.14,
        )

        pivot_z = FIRST_PIVOT_Z + index * VANE_PITCH
        model.articulation(
            f"frame_to_vane_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-VANE_LIMIT,
                upper=VANE_LIMIT,
                effort=1.0,
                velocity=2.5,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    ctx.check("frame_present", frame is not None, "Expected a grounded frame part.")

    # Every vane should exist, be independently articulated, and tilt about X.
    for index in range(VANE_COUNT):
        name = f"vane_{index + 1}"
        joint_name = f"frame_to_vane_{index + 1}"
        vane = object_model.get_part(name)
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{name}_present", vane is not None, f"Missing {name}.")
        ctx.check(f"{joint_name}_present", joint is not None, f"Missing {joint_name}.")

    # Decisive independence check: pose one vane open while leaving its neighbor
    # closed, and confirm only the posed vane tilts out of plane.
    j2 = object_model.get_articulation("frame_to_vane_2")
    vane2 = object_model.get_part("vane_2")
    vane3 = object_model.get_part("vane_3")

    closed_aabb2 = ctx.part_world_aabb(vane2)
    closed_aabb3 = ctx.part_world_aabb(vane3)

    with ctx.pose({j2: VANE_LIMIT}):
        open_aabb2 = ctx.part_world_aabb(vane2)
        open_aabb3 = ctx.part_world_aabb(vane3)

    def _depth(aabb):
        return None if aabb is None else float(aabb[1][1] - aabb[0][1])

    # The posed vane should swing so its Y-extent (out-of-plane depth) grows.
    ctx.check(
        "posed_vane_tilts",
        _depth(open_aabb2) is not None
        and _depth(closed_aabb2) is not None
        and _depth(open_aabb2) > _depth(closed_aabb2) + 0.01,
        details=f"closed={_depth(closed_aabb2)}, open={_depth(open_aabb2)}",
    )
    # The neighbor must not move: independent pivots.
    ctx.check(
        "neighbor_vane_unmoved",
        _depth(open_aabb3) is not None
        and _depth(closed_aabb3) is not None
        and abs(_depth(open_aabb3) - _depth(closed_aabb3)) < 1e-6,
        details=f"closed={_depth(closed_aabb3)}, open={_depth(open_aabb3)}",
    )

    return ctx.report()


object_model = build_object_model()

# >>> USER_CODE_END
```
