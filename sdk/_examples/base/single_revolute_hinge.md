---
title: 'Single Revolute Hinge'
description: 'Base SDK wall-mounted cabinet with one revolute door hinge, showing the canonical body/door part split, a visible barrel hinge, and the sign choice that swings the free door edge outward instead of into the carcass.'
tags:
  - sdk
  - base sdk
  - articulation
  - revolute
  - hinge
  - barrel hinge
  - cabinet
  - door
  - wall mounted cabinet
  - mesh geometry
  - motion limits
  - revolute articulation
---
# Single Revolute Hinge

This base-SDK example reproduces the canonical single-hinge cabinet: a hollow
wall-mounted carcass (open front), a swinging door with a handle, and one
`REVOLUTE` articulation. It is a compact reference for queries such as
`single revolute hinge`, `cabinet door`, `wall mounted cabinet`, `barrel hinge`,
and `revolute articulation`.

The important teaching point is the sign choice. The closed door geometry
extends along local `+X` from the hinge line, so `axis=(0, 0, 1)` makes positive
joint values swing the free edge outward toward the front `+Y` instead of
rotating into the carcass. The door part frame sits on the hinge line so that at
`q=0` the child frame coincides with the articulation frame.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    BoxGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

# Cabinet carcass (meters). X = width, Y = depth, Z = height.
CABINET_WIDTH = 0.50
CABINET_DEPTH = 0.30
CABINET_HEIGHT = 0.60
WALL_THICKNESS = 0.018

# Door panel: hinged on its left edge, extends along +X from the hinge line.
DOOR_WIDTH = 0.48
DOOR_THICKNESS = 0.018
DOOR_HEIGHT = 0.58

# Hinge placement.
HINGE_SIDE_OFFSET = 0.004
FRONT_GAP = 0.004
HINGE_LENGTH = 0.10


def _body_mesh() -> object:
    """Hollow carcass with an open front toward +Y."""
    outer = BoxGeometry((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT))
    # Inner cavity, opened through the front (+Y) face by extending the cutter
    # past the front wall so the front reads as open.
    cavity = BoxGeometry(
        (
            CABINET_WIDTH - 2.0 * WALL_THICKNESS,
            CABINET_DEPTH,  # extend through the front face to leave it open
            CABINET_HEIGHT - 2.0 * WALL_THICKNESS,
        )
    )
    cavity.translate(0.0, WALL_THICKNESS, 0.0)
    return boolean_difference(outer, cavity)


def _door_panel_mesh() -> object:
    """Door panel built in the door part frame: hinge line at local X=0,
    panel body centered at +DOOR_WIDTH/2."""
    panel = BoxGeometry((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT))
    panel.translate(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, 0.0)
    return panel


def _door_handle_mesh() -> object:
    """Vertical bar handle near the free (far) edge of the door, standing proud
    of the front face."""
    bar = BoxGeometry((0.020, 0.030, 0.16))
    bar.translate(
        DOOR_WIDTH - 0.04,
        DOOR_THICKNESS + 0.015,
        0.0,
    )
    return bar


def _hinge_mesh() -> object:
    """Visible barrel hinge. The helper builds it around local Z (the door swing
    axis), with leaves spreading in X."""
    return BarrelHingeGeometry(
        HINGE_LENGTH,
        leaf_width_a=0.020,
        leaf_width_b=0.020,
        leaf_thickness=0.0024,
        pin_diameter=0.004,
        knuckle_count=5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_cabinet")

    carcass_finish = model.material("carcass_finish", rgba=(0.93, 0.94, 0.95, 1.0))
    door_finish = model.material("door_finish", rgba=(0.90, 0.91, 0.92, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    hinge_finish = model.material("hinge_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    body = model.part("cabinet_body")
    body.visual(mesh_from_geometry(_body_mesh(), "cabinet_body_shell"), material=carcass_finish)
    # Visible hinge knuckle mounted on the body at the hinge line / left front edge.
    body.visual(
        mesh_from_geometry(_hinge_mesh(), "cabinet_hinge"),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 - HINGE_SIDE_OFFSET,
                CABINET_DEPTH / 2.0 + FRONT_GAP,
                0.0,
            )
        ),
        material=hinge_finish,
    )
    body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=8.5,
    )

    door = model.part("door")
    door.visual(mesh_from_geometry(_door_panel_mesh(), "door_panel"), material=door_finish)
    door.visual(mesh_from_geometry(_door_handle_mesh(), "door_handle"), material=handle_finish)
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        mass=2.6,
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent="cabinet_body",
        child="door",
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 - HINGE_SIDE_OFFSET,
                CABINET_DEPTH / 2.0 + FRONT_GAP,
                0.0,
            )
        ),
        # Closed door geometry extends along +X from the hinge line.
        # Positive q around +Z swings the free edge outward toward +Y.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=10.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    ctx.check("body_present", body is not None, "Expected a cabinet_body part.")
    ctx.check("door_present", door is not None, "Expected a door part.")
    ctx.check("hinge_present", hinge is not None, "Expected the body_to_door hinge.")
    if body is None or door is None or hinge is None:
        return ctx.report()

    # Closed pose: door seats across the cabinet front, free edge near the
    # right side, not swung into the carcass.
    with ctx.pose({hinge: 0.0}):
        closed = ctx.part_world_aabb(door)
        ctx.check("door_closed_aabb", closed is not None, "Expected closed door AABB.")
        if closed is not None:
            mins, maxs = closed
            # Free edge reaches toward +X (right side of the cabinet).
            ctx.check(
                "door_closed_reaches_right",
                maxs[0] > 0.20,
                f"closed door max x={maxs[0]:.3f}",
            )

    # Open pose: positive angle should swing the free edge forward (+Y).
    with ctx.pose({hinge: 1.5}):
        opened = ctx.part_world_aabb(door)
        ctx.check("door_open_aabb", opened is not None, "Expected open door AABB.")
        if opened is not None:
            mins, maxs = opened
            ctx.check(
                "door_opens_forward",
                maxs[1] > CABINET_DEPTH / 2.0 + 0.10,
                f"open door max y={maxs[1]:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
```
