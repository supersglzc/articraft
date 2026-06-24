---
title: 'A Parametric Enclosure'
description: 'Base SDK parametric two-part electronics enclosure: a rounded hollow box body with internal screw posts and a hinged lid with a snug inner lip and counterbored screw holes, built from native mesh booleans.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - enclosure
  - parametric enclosure
  - electronics case
  - project box
  - housing
  - lid
  - hinged lid
  - screw posts
  - counterbore
  - boolean difference
  - boolean union
  - extrude geometry
  - rounded rect profile
  - revolute articulation
  - motion limits
---
# A Parametric Enclosure

This base-SDK example is a native reproduction of the classic parametric
enclosure: a rounded, hollow box body with four internal screw posts and a
removable lid that has a snug interior lip plus counterbored screw holes that
line up with the posts. It is useful for queries such as `parametric enclosure`,
`project box`, `electronics case`, `housing with screw posts`, `counterbore`,
`snap-fit lid`, `boolean_difference`, and `ExtrudeGeometry`.

The teaching intent is parametric, boolean-driven housing construction. All
key dimensions are top-level parameters, the body and lid are derived from the
same outer footprint, and the lid lip is sized to nest inside the body wall for
a snug fit. The functional mechanism is preserved as a working two-part
enclosure: the body is the root and the lid is a separate part on a rear hinge
so it can open and re-seat.

The modeling patterns worth copying are:

- `rounded_rect_profile(...)` + `ExtrudeGeometry.from_z0(...)` to get a rounded
  rectangular prism, then `boolean_difference(...)` to hollow it into a wall.
- screw posts built as solid cylinders unioned into the body wall, with a
  through bore cut by `boolean_difference(...)`.
- a lid whose interior lip is a thin rounded ring sized to the inner wall, so
  it nests into the open body for a snug fit.
- counterbore screw holes in the lid via stacked cylinder cuts (a wide shallow
  bore over a narrow through hole), matching the `cboreHole` intent.
- a rear revolute hinge with `MotionLimits` so positive motion lifts the lid.

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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

# --- Parameters (meters) ---------------------------------------------------
# A realistic project-box footprint, scaled down from the original 100x150x50 mm.
OUTER_WIDTH = 0.100  # X
OUTER_LENGTH = 0.150  # Y
OUTER_HEIGHT = 0.050  # Z (body interior depth + walls)

THICKNESS = 0.003  # wall / floor / lid thickness
SIDE_RADIUS = 0.010  # rounding radius for the vertical side edges
LIP_HEIGHT = 0.004  # height of the lid lip that nests inside the body
LIP_CLEARANCE = 0.0004  # gap so the lip fits inside the body wall

SCREWPOST_INSET = 0.012  # how far in from the outer edges the posts sit
SCREWPOST_OD = 0.010  # outer diameter of the posts
SCREWPOST_ID = 0.004  # inner bore diameter (screw shaft)

BORE_DIAMETER = 0.008  # counterbore diameter in the lid (screw head clearance)
BORE_DEPTH = 0.001  # counterbore depth from the lid top

CORNER_SEGMENTS = 8
RADIAL_SEGMENTS = 32

# Derived post X/Y centers (mirrored four-up rectangle inside the footprint).
POST_X = OUTER_WIDTH / 2.0 - SCREWPOST_INSET
POST_Y = OUTER_LENGTH / 2.0 - SCREWPOST_INSET
POST_CENTERS = [
    (POST_X, POST_Y),
    (-POST_X, POST_Y),
    (POST_X, -POST_Y),
    (-POST_X, -POST_Y),
]


def _rounded_prism(width: float, length: float, height: float, radius: float):
    """Solid rounded-rectangular prism spanning z in [0, height]."""
    profile = rounded_rect_profile(width, length, radius, corner_segments=CORNER_SEGMENTS)
    return ExtrudeGeometry.from_z0(profile, height, cap=True)


def _build_body():
    """Hollow rounded box body, open at the top, with bored screw posts."""
    # Outer solid spans the full body height; the floor stays solid.
    outer = _rounded_prism(OUTER_WIDTH, OUTER_LENGTH, OUTER_HEIGHT, SIDE_RADIUS)

    # Inner cavity: inset by the wall thickness, starting above the floor and
    # left open at the top so the body reads as a real shell.
    inner = _rounded_prism(
        OUTER_WIDTH - 2.0 * THICKNESS,
        OUTER_LENGTH - 2.0 * THICKNESS,
        OUTER_HEIGHT,  # over-tall so it punches through the open top
        max(SIDE_RADIUS - THICKNESS, 0.001),
    ).translate(0.0, 0.0, THICKNESS + 0.001)

    body = boolean_difference(outer, inner)

    # Screw posts: solid cylinders standing on the floor, bored through.
    post_height = OUTER_HEIGHT - THICKNESS
    for cx, cy in POST_CENTERS:
        post = CylinderGeometry(
            SCREWPOST_OD / 2.0,
            post_height,
            radial_segments=RADIAL_SEGMENTS,
        ).translate(cx, cy, THICKNESS + post_height / 2.0)
        body = boolean_union(body, post)

        bore = CylinderGeometry(
            SCREWPOST_ID / 2.0,
            post_height + 4.0 * THICKNESS,
            radial_segments=RADIAL_SEGMENTS,
        ).translate(cx, cy, THICKNESS + post_height / 2.0)
        body = boolean_difference(body, bore)

    return body


def _build_lid():
    """Lid plate with an interior nesting lip and counterbored screw holes.

    Built in a local frame whose top face sits at z = 0 and whose lip extends
    downward (-Z). This matches the rear-hinge child frame used below.
    """
    # Top plate spans z in [-THICKNESS, 0].
    plate = _rounded_prism(OUTER_WIDTH, OUTER_LENGTH, THICKNESS, SIDE_RADIUS).translate(
        0.0, 0.0, -THICKNESS
    )

    # Interior lip ring that drops into the body cavity for a snug fit.
    lip_outer_w = OUTER_WIDTH - 2.0 * THICKNESS - 2.0 * LIP_CLEARANCE
    lip_outer_l = OUTER_LENGTH - 2.0 * THICKNESS - 2.0 * LIP_CLEARANCE
    lip_solid = _rounded_prism(
        lip_outer_w,
        lip_outer_l,
        LIP_HEIGHT,
        max(SIDE_RADIUS - THICKNESS, 0.001),
    ).translate(0.0, 0.0, -THICKNESS - LIP_HEIGHT)
    lip_cavity = _rounded_prism(
        lip_outer_w - 2.0 * THICKNESS,
        lip_outer_l - 2.0 * THICKNESS,
        LIP_HEIGHT + 0.002,
        max(SIDE_RADIUS - 2.0 * THICKNESS, 0.0008),
    ).translate(0.0, 0.0, -THICKNESS - LIP_HEIGHT - 0.001)
    lip = boolean_difference(lip_solid, lip_cavity)
    lid = boolean_union(plate, lip)

    # Counterbored screw holes aligned with the body posts.
    for cx, cy in POST_CENTERS:
        through = CylinderGeometry(
            SCREWPOST_ID / 2.0,
            THICKNESS + LIP_HEIGHT + 0.004,
            radial_segments=RADIAL_SEGMENTS,
        ).translate(cx, cy, -(THICKNESS + LIP_HEIGHT) / 2.0)
        lid = boolean_difference(lid, through)

        cbore = CylinderGeometry(
            BORE_DIAMETER / 2.0,
            BORE_DEPTH + 0.001,
            radial_segments=RADIAL_SEGMENTS,
        ).translate(cx, cy, -BORE_DEPTH / 2.0 + 0.0005)
        lid = boolean_difference(lid, cbore)

    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parametric_enclosure")

    case_gray = model.material("case_gray", rgba=(0.32, 0.34, 0.37, 1.0))
    lid_gray = model.material("lid_gray", rgba=(0.22, 0.23, 0.25, 1.0))

    body = model.part("body")
    body.visual(mesh_from_geometry(_build_body(), "enclosure_body"), material=case_gray)
    body.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, OUTER_LENGTH, OUTER_HEIGHT)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(mesh_from_geometry(_build_lid(), "enclosure_lid"), material=lid_gray)
    lid.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, OUTER_LENGTH, THICKNESS + LIP_HEIGHT)),
        mass=0.06,
        origin=Origin(xyz=(0.0, -OUTER_LENGTH / 2.0, -(THICKNESS + LIP_HEIGHT) / 2.0)),
    )

    # Rear hinge: the hinge line runs along X at the +Y rear edge, on top of the
    # body wall. The lid child frame is placed so its top face seats flush with
    # the body opening and its lip drops into the cavity at q = 0. Positive q
    # rotates about -X so the front edge of the lid lifts upward.
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, OUTER_LENGTH / 2.0, OUTER_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check("body_present", body is not None, "Expected a body part.")
    ctx.check("lid_present", lid is not None, "Expected a lid part.")
    ctx.check("hinge_present", hinge is not None, "Expected a body_to_lid hinge.")
    if body is None or lid is None or hinge is None:
        return ctx.report()

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "body_footprint",
            0.090 <= size[0] <= 0.112 and 0.140 <= size[1] <= 0.162,
            f"body size={size!r}",
        )
        ctx.check("body_height", 0.045 <= size[2] <= 0.055, f"body size={size!r}")

    # Closed: the lid seats on the body opening. Open: the lid swings clear.
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    with ctx.pose({hinge: 1.6}):
        ctx.expect_gap(lid, body, axis="z", max_penetration=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
