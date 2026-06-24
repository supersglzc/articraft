---
title: 'Wall Vent with Louvered Grille'
description: 'Base SDK example showing a framed wall vent with a real louvered slat grille, a recessed rear duct sleeve, and corner mounting screws built from VentGrilleGeometry.'
tags:
  - sdk
  - base sdk
  - vent
  - grille
  - louver
  - register
  - wall vent
  - mesh geometry
---
# Wall Vent with Louvered Grille

This base-SDK example reproduces a wall vent that should read as a clean
manufactured shell: a framed face, a slatted (louvered) grille, a recessed rear
duct sleeve, and four corner mounting screws. It is the native counterpart to a
hand-assembled slat stack and is a compact reference for `VentGrilleGeometry`,
`VentGrilleSlats`, `VentGrilleFrame`, `VentGrilleMounts`, and
`VentGrilleSleeve`. It is useful for queries such as `wall vent`,
`louvered grille`, `air register`, and `VentGrilleGeometry`.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    CylinderGeometry,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)

WIDTH = 0.18
HEIGHT = 0.10
FACE_THICKNESS = 0.004
FRAME = 0.012
DUCT_DEPTH = 0.026
DUCT_WALL = 0.003
SLAT_PITCH = 0.018
SLAT_WIDTH = 0.009
SLAT_ANGLE_DEG = 35.0
MOUNT_INSET = 0.008
MOUNT_HOLE_DIAMETER = 0.0032

SCREW_RADIUS = 0.0024
SCREW_LENGTH = 0.0024
SCREW_OFFSET_X = WIDTH / 2.0 - MOUNT_INSET
SCREW_OFFSET_Y = HEIGHT / 2.0 - MOUNT_INSET


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_vent")
    model.material("plastic_white", rgba=(0.90, 0.90, 0.90, 1.0))
    model.material("metal_silver", rgba=(0.70, 0.70, 0.70, 1.0))

    vent_body = model.part("vent_body")

    # The whole vent shell - framed face, louvered slats, and rear duct sleeve -
    # is one continuous manufactured piece. The grille lies in local XY with the
    # face toward +Z and the recessed sleeve extending toward -Z.
    grille = VentGrilleGeometry(
        (WIDTH, HEIGHT),
        frame=FRAME,
        face_thickness=FACE_THICKNESS,
        duct_depth=DUCT_DEPTH,
        duct_wall=DUCT_WALL,
        slat_pitch=SLAT_PITCH,
        slat_width=SLAT_WIDTH,
        slat_angle_deg=SLAT_ANGLE_DEG,
        corner_radius=0.006,
        slats=VentGrilleSlats(profile="flat", direction="down"),
        frame_profile=VentGrilleFrame(style="flush"),
        mounts=VentGrilleMounts(
            style="holes",
            inset=MOUNT_INSET,
            hole_diameter=MOUNT_HOLE_DIAMETER,
        ),
        sleeve=VentGrilleSleeve(style="full"),
    )
    vent_body.visual(
        mesh_from_geometry(grille, "vent_shell"),
        material="plastic_white",
        name="vent_shell",
    )

    # The grille face sits at the +Z extreme of the shell. Seat a flush-fit screw
    # head into each corner mounting hole so the screws read as mounted features.
    face_z = FACE_THICKNESS / 2.0
    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        vent_body.visual(
            CylinderGeometry(radius=SCREW_RADIUS, height=SCREW_LENGTH),
            origin=Origin(
                xyz=(
                    sx * SCREW_OFFSET_X,
                    sy * SCREW_OFFSET_Y,
                    face_z + SCREW_LENGTH / 2.0 - 0.0006,
                )
            ),
            material="metal_silver",
            name=f"screw_{i}",
        )

    vent_body.inertial = Inertial.from_geometry(
        Box((WIDTH, HEIGHT, FACE_THICKNESS + DUCT_DEPTH)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -(DUCT_DEPTH) / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    vent = object_model.get_part("vent_body")
    ctx.check("vent_part_present", vent is not None, "Expected a vent_body part.")
    if vent is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(vent)
    ctx.check("vent_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("vent_width", 0.17 <= size[0] <= 0.20, f"size={size!r}")
    ctx.check("vent_height", 0.095 <= size[1] <= 0.115, f"size={size!r}")
    # Face plate plus rear sleeve plus the proud screw heads give real depth.
    ctx.check("vent_depth", 0.020 <= size[2] <= 0.045, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
