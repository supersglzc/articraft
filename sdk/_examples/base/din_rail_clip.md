---
title: 'DIN Rail Clip'
description: 'Base SDK example of a molded plastic DIN-rail mounting clip: a chamfered body block with a top-hat rail aperture pocket and three countersunk mounting holes, built from native mesh primitives and booleans.'
tags:
  - sdk
  - base sdk
  - din rail
  - rail clip
  - mounting clip
  - countersunk hole
  - chamfer
  - boolean difference
  - mesh geometry
---
# DIN Rail Clip

This base-SDK example reproduces the classic `cq-electronics` DIN rail clip as a
single molded plastic part. The teaching intent is faithful to the original:

- a rectangular body block,
- a recessed **top-hat rail aperture** pocket on the underside,
- three **countersunk** clearance holes (two outer mount holes plus one inside
  the rail aperture face), and
- **chamfered vertical corners** on the short ends.

The native build uses mesh primitives (`BoxGeometry`, `CylinderGeometry`,
`ConeGeometry`) plus `boolean_union` / `boolean_difference`. Countersunk holes
are modeled as a clearance-diameter cylinder topped by a conical sink, matching
the `cskHole` semantics. The chamfered corners are produced by unioning the body
block from a slightly inset core plus narrowed slabs, which clips the four
vertical edges.

All dimensions are converted from the original millimeter model to meters.

```python
from __future__ import annotations

from math import radians, tan

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    ConeGeometry,
    CylinderGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# --- Source dimensions (converted mm -> m) ---
LENGTH = 0.076
WIDTH = 0.020
HEIGHT = 0.008

TOP_HAT_WIDTH = 0.035
RAIL_APERTURE_DEPTH = 0.004
BETWEEN_MOUNT_HOLES = 0.063
CORNER_CHAMFER = 0.003

# Countersunk M4 clearance hole geometry.
CSK_ANGLE_DEG = 90.0
CSK_CLEARANCE_DIAMETER = 0.0045
CSK_HEAD_DIAMETER = 0.0094

HALF_LENGTH = LENGTH / 2.0
# Rail aperture is offset toward -X by (half_length - 30 mm).
RAIL_APERTURE_OFFSET = HALF_LENGTH - 0.030
RAIL_APERTURE_CENTER_X = -RAIL_APERTURE_OFFSET
HALF_BETWEEN = BETWEEN_MOUNT_HOLES / 2.0

# Body is centered at the origin; top face at +Z, underside (rail face) at -Z.
TOP_Z = HEIGHT / 2.0
BOTTOM_Z = -HEIGHT / 2.0
# Floor of the rail aperture pocket, measured up from the underside.
APERTURE_FLOOR_Z = BOTTOM_Z + RAIL_APERTURE_DEPTH


def _chamfered_body() -> "BoxGeometry":
    """Body block with the four vertical (|Z) corners chamfered.

    Built by unioning a Y-narrowed slab and an X-narrowed slab so the union
    footprint is an octagon-cornered rectangle, clipping the vertical edges by
    CORNER_CHAMFER.
    """
    slab_x = BoxGeometry((LENGTH, WIDTH - 2.0 * CORNER_CHAMFER, HEIGHT))
    slab_y = BoxGeometry((LENGTH - 2.0 * CORNER_CHAMFER, WIDTH, HEIGHT))
    return boolean_union(slab_x, slab_y)


def _countersunk_cutter(center_x: float, center_y: float) -> "CylinderGeometry":
    """Through clearance bore plus a conical sink opening at the top face.

    Models cq's cskHole: clearance shank through the part with a 90-degree
    countersink cone breaking out at the top (+Z) face.
    """
    clr_r = CSK_CLEARANCE_DIAMETER / 2.0
    head_r = CSK_HEAD_DIAMETER / 2.0

    # Through shank, made over-tall so both faces are cleanly pierced.
    shank = CylinderGeometry(clr_r, HEIGHT * 2.0, radial_segments=32)
    shank.translate(center_x, center_y, 0.0)

    # Countersink cone: full angle CSK_ANGLE_DEG. Depth so the cone spans from
    # the clearance radius up to the head radius at the top face.
    half_angle = radians(CSK_ANGLE_DEG) / 2.0
    cone_depth = (head_r - clr_r) / tan(half_angle)
    # ConeGeometry tapers from base radius at -Z to an apex at +Z. Build it with
    # the wide head opening at the top face and let it run below into the shank.
    cone = ConeGeometry(head_r, cone_depth * 2.0, radial_segments=32)
    # Apex of the doubled cone sits cone_depth below the head plane; place the
    # head opening (base) flush with the top face.
    cone.translate(center_x, center_y, TOP_Z - cone_depth)

    return boolean_union(shank, cone)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="din_rail_clip")
    plastic = model.material("din_clip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = _chamfered_body()

    # Rail aperture pocket: a top-hat-width slot cut into the underside.
    pocket = BoxGeometry((TOP_HAT_WIDTH, WIDTH * 2.0, RAIL_APERTURE_DEPTH * 2.0))
    # Center the pocket so its top is at the aperture floor and it opens out the
    # underside.
    pocket.translate(RAIL_APERTURE_CENTER_X, 0.0, BOTTOM_Z)
    body = boolean_difference(body, pocket)

    # Two outer countersunk mount holes through the full body.
    body = boolean_difference(body, _countersunk_cutter(HALF_BETWEEN, 0.0))
    body = boolean_difference(body, _countersunk_cutter(-HALF_BETWEEN, 0.0))

    # One countersunk hole centered on the rail aperture, sunk from the top face
    # and breaking through into the pocket.
    body = boolean_difference(body, _countersunk_cutter(RAIL_APERTURE_CENTER_X, 0.0))

    clip = model.part("clip")
    clip.visual(
        mesh_from_geometry(body, "din_rail_clip"),
        material=plastic,
        name="clip_body",
    )
    clip.inertial = Inertial.from_geometry(
        Box((LENGTH, WIDTH, HEIGHT)),
        mass=0.02,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clip = object_model.get_part("clip")
    ctx.check("clip_part_present", clip is not None, "Expected a clip part.")
    if clip is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(clip)
    ctx.check("clip_aabb_present", aabb is not None, "Expected a world AABB for the clip.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("clip_length", 0.073 <= size[0] <= 0.077, f"size={size!r}")
    ctx.check("clip_width", 0.018 <= size[1] <= 0.021, f"size={size!r}")
    ctx.check("clip_height", 0.0075 <= size[2] <= 0.0085, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
