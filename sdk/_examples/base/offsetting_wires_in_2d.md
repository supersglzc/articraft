---
title: 'Offsetting Wires in 2D'
description: 'Native base-SDK demonstration of 2D profile offsetting: a pentagon outline offset outward and inward to drive stacked extruded plates, plus a flange whose bolt-hole ring is offset inward from its outline.'
tags:
  - sdk
  - base sdk
  - offsetting
  - profile offset
  - polygon
  - extrude
  - bolt holes
  - flange
  - mesh geometry
---
# Offsetting Wires in 2D

In CadQuery this concept is taught with `Workplane.offset2D()`, which grows or
shrinks a closed 2D wire (with `"arc"` or `"intersection"` corner styles) and is
commonly used to inset bolt holes from an object outline. The native base SDK has
no `offset2D` operation, so this example reproduces the *teaching intent* with
plain 2D math: we author a closed pentagon profile, then compute outward- and
inward-offset variants of that same profile and extrude each into a stacked
plate. The lowest plate keeps the original outline, the middle plate is offset
outward, and the top plate is offset inward, so the three offsets read clearly
when stacked along `+Z`.

We then add a separate flange plate that demonstrates the practical use of an
inward offset: a ring of bolt holes inset from the part outline, cut through the
plate with `ExtrudeWithHolesGeometry`. This is the native equivalent of offsetting
the outline inward "for construction" and dropping holes on the result.

The corner-style choice (`"arc"` vs `"intersection"`) is approximated: an outward
polygon offset here uses the simple radial-scale form, which matches the
`"intersection"` (sharp, extended-corner) behavior. Rounded `"arc"` corners are
not reproduced, since they do not change the part identity of these plates.

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

PLATE_THICKNESS = 0.010
BASE_RADIUS = 0.060  # circumradius of the original pentagon, in meters
OFFSET = 0.012  # outward/inward offset distance, in meters


def _regular_polygon(sides: int, radius: float) -> list[tuple[float, float]]:
    """Counter-clockwise regular polygon centered on the origin in local XY."""
    pts: list[tuple[float, float]] = []
    for i in range(sides):
        theta = (2.0 * math.pi * i) / sides + math.pi / 2.0
        pts.append((radius * math.cos(theta), radius * math.sin(theta)))
    return pts


def _offset_polygon(profile: list[tuple[float, float]], distance: float) -> list[tuple[float, float]]:
    """Offset a centered convex polygon by ``distance`` along each vertex radial.

    Positive distance grows the outline outward; negative shrinks it inward.
    For a regular polygon centered on the origin this radial offset reproduces
    the sharp-cornered ("intersection"-style) behavior of a 2D wire offset.
    """
    offset: list[tuple[float, float]] = []
    for x, y in profile:
        length = math.hypot(x, y)
        if length <= 1e-9:
            offset.append((x, y))
            continue
        ux, uy = x / length, y / length
        offset.append((x + ux * distance, y + uy * distance))
    return offset


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_pentagon_plates")
    plate_mat = model.material("plate_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    flange_mat = model.material("flange_brass", rgba=(0.72, 0.60, 0.30, 1.0))

    original_profile = _regular_polygon(5, BASE_RADIUS)
    outward_profile = _offset_polygon(original_profile, OFFSET)
    inward_profile = _offset_polygon(original_profile, -OFFSET)

    stack = model.part("offset_stack")

    # Bottom plate: original pentagon outline.
    stack.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(original_profile, PLATE_THICKNESS),
            "plate_original",
        ),
        material=plate_mat,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="plate_original",
    )

    # Middle plate: outline offset outward.
    stack.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(outward_profile, PLATE_THICKNESS),
            "plate_offset_out",
        ),
        material=plate_mat,
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS)),
        name="plate_offset_out",
    )

    # Top plate: outline offset inward.
    stack.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(inward_profile, PLATE_THICKNESS),
            "plate_offset_in",
        ),
        material=plate_mat,
        origin=Origin(xyz=(0.0, 0.0, 2.0 * PLATE_THICKNESS)),
        name="plate_offset_in",
    )

    stack.inertial = Inertial.from_geometry(
        Box((2.0 * (BASE_RADIUS + OFFSET), 2.0 * (BASE_RADIUS + OFFSET), 3.0 * PLATE_THICKNESS)),
        mass=0.8,
    )

    # Flange plate: square outline with a bolt-hole ring offset inward from it.
    flange = model.part("bolt_flange")
    flange_half = 0.050
    flange_outline = [
        (-flange_half, -flange_half),
        (flange_half, -flange_half),
        (flange_half, flange_half),
        (-flange_half, flange_half),
    ]
    # Bolt circle radius derived by offsetting the outline inward, then placing
    # holes on the inset outline near each corner.
    bolt_inset = flange_half - 0.012
    hole_radius = 0.0035
    hole_profiles: list[list[tuple[float, float]]] = []
    for cx, cy in [
        (-bolt_inset, -bolt_inset),
        (bolt_inset, -bolt_inset),
        (bolt_inset, bolt_inset),
        (-bolt_inset, bolt_inset),
    ]:
        ring = [
            (cx + hole_radius * math.cos(2.0 * math.pi * k / 24),
             cy + hole_radius * math.sin(2.0 * math.pi * k / 24))
            for k in range(24)
        ]
        hole_profiles.append(ring)

    flange.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                flange_outline,
                hole_profiles,
                PLATE_THICKNESS,
                center=False,
            ),
            "bolt_flange",
        ),
        material=flange_mat,
        # Sit beside the stack so both teaching pieces are visible together.
        origin=Origin(xyz=(0.0, BASE_RADIUS + OFFSET + flange_half + 0.010, 0.0)),
        name="bolt_flange",
    )
    flange.inertial = Inertial.from_geometry(
        Box((2.0 * flange_half, 2.0 * flange_half, PLATE_THICKNESS)),
        mass=0.3,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stack = object_model.get_part("offset_stack")
    flange = object_model.get_part("bolt_flange")
    ctx.check("stack_present", stack is not None, "Expected the offset_stack part.")
    ctx.check("flange_present", flange is not None, "Expected the bolt_flange part.")
    if stack is None or flange is None:
        return ctx.report()

    stack_aabb = ctx.part_world_aabb(stack)
    ctx.check("stack_aabb_present", stack_aabb is not None, "Expected a stack AABB.")
    if stack_aabb is not None:
        mins, maxs = stack_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        # Outward offset plate sets the widest extent (~2*(R+OFFSET)).
        expected_w = 2.0 * (BASE_RADIUS + OFFSET)
        ctx.check(
            "stack_width",
            0.8 * expected_w <= max(size[0], size[1]) <= 1.05 * expected_w,
            f"size={size!r}",
        )
        # Three stacked plates of equal thickness.
        ctx.check(
            "stack_height",
            abs(size[2] - 3.0 * PLATE_THICKNESS) <= 0.002,
            f"size={size!r}",
        )

    flange_aabb = ctx.part_world_aabb(flange)
    ctx.check("flange_aabb_present", flange_aabb is not None, "Expected a flange AABB.")
    if flange_aabb is not None:
        mins, maxs = flange_aabb
        fsize = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("flange_thickness", abs(fsize[2] - PLATE_THICKNESS) <= 0.001, f"fsize={fsize!r}")

    return ctx.report()


object_model = build_object_model()
```
