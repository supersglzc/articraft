---
title: 'Lego Brick'
description: 'Base SDK parametric reproduction of a regular rectangular LEGO-style brick: a hollow box shell with a grid of top studs and underside tubes, built from native mesh booleans.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - lego
  - lego brick
  - brick
  - studs
  - parametric
  - boolean union
  - boolean difference
  - box geometry
  - cylinder geometry
---
# Lego Brick

This base-SDK example is a native reproduction of the classic parametric LEGO
brick: a hollow rectangular shell with a grid of round studs ("bumps") on top
and a matching grid of hollow tubes underneath that grip the studs of the brick
below. It is useful for queries such as `lego brick`, `brick`, `studs`,
`parametric brick`, `hollow box shell`, `boolean_union`, `boolean_difference`,
and `CylinderGeometry`.

The teaching intent is the same as the original: a single parametric part whose
real subtlety lives in the underside logic. The brick is `lbumps` studs long by
`wbumps` studs wide, all key dimensions follow the standard LEGO constants, and
the underside support is chosen by geometry:

- For a multi-stud-by-multi-stud brick, the underside has hollow tubes centered
  between the top studs.
- For a 1-wide brick (one dimension is a single stud), the underside has solid
  thin posts instead of tubes.
- A 1x1 brick has no underside support at all (just the shell).

The brick is a single rigid part, so there is no articulation. The modeling
patterns worth copying are:

- a shell built by subtracting an inner cavity from a solid box
  (`boolean_difference(...)`), open on the underside.
- a grid of studs unioned onto the top face (`boolean_union(...)`).
- underside tubes built as an outer cylinder unioned in, then bored hollow with
  `boolean_difference(...)`, matching the LEGO tube logic.

```python
from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# --- Inputs ----------------------------------------------------------------
LBUMPS = 6  # number of studs long (X)
WBUMPS = 2  # number of studs wide (Y)
THIN = True  # True for a plate-height brick, False for a tall brick

# --- LEGO constants (millimeters, converted to meters) ---------------------
# These are the canonical dimensions that make a LEGO a LEGO.
MM = 0.001
PITCH = 8.0 * MM  # center-to-center stud spacing
CLEARANCE = 0.1 * MM  # gap so adjacent bricks do not bind
STUD_DIAM = 4.8 * MM  # top stud ("bump") diameter
STUD_HEIGHT = 1.8 * MM  # top stud height
HEIGHT = (3.2 if THIN else 9.6) * MM  # overall shell height

# Wall thickness derived exactly as in the original script.
WALL = (PITCH - 2.0 * CLEARANCE - STUD_DIAM) / 2.0
TUBE_OUTER_DIAM = PITCH - WALL  # works out to 6.5 mm for standard bricks

TOTAL_LENGTH = LBUMPS * PITCH - 2.0 * CLEARANCE  # X
TOTAL_WIDTH = WBUMPS * PITCH - 2.0 * CLEARANCE  # Y

RADIAL_SEGMENTS = 32

# Small overlaps so boolean operands intersect cleanly instead of sharing a
# coincident face.
EPS = 0.05 * MM


def _stud_centers(nx: int, ny: int):
    """Centers of an nx-by-ny grid spaced by PITCH, centered on the origin."""
    centers = []
    for i in range(nx):
        cx = (i - (nx - 1) / 2.0) * PITCH
        for j in range(ny):
            cy = (j - (ny - 1) / 2.0) * PITCH
            centers.append((cx, cy))
    return centers


def _build_brick():
    """Hollow brick shell with top studs and underside support."""
    # Outer solid spans z in [0, HEIGHT].
    outer = BoxGeometry((TOTAL_LENGTH, TOTAL_WIDTH, HEIGHT)).translate(
        0.0, 0.0, HEIGHT / 2.0
    )

    # Inner cavity: inset by the wall thickness on the sides and ceiling, open at
    # the bottom so the brick reads as a real shell. Over-tall so it punches
    # cleanly through the open underside.
    inner = BoxGeometry(
        (
            TOTAL_LENGTH - 2.0 * WALL,
            TOTAL_WIDTH - 2.0 * WALL,
            HEIGHT,
        )
    ).translate(0.0, 0.0, HEIGHT / 2.0 - WALL)
    brick = boolean_difference(outer, inner)

    # Top studs: a full LBUMPS x WBUMPS grid sitting on the top face.
    for cx, cy in _stud_centers(LBUMPS, WBUMPS):
        stud = CylinderGeometry(
            STUD_DIAM / 2.0,
            STUD_HEIGHT + EPS,
            radial_segments=RADIAL_SEGMENTS,
        ).translate(cx, cy, HEIGHT + STUD_HEIGHT / 2.0 - EPS / 2.0)
        brick = boolean_union(brick, stud)

    # Underside support. Tubes/posts hang from the inner ceiling down to (but not
    # through) the open bottom, leaving WALL of clearance, exactly as in the
    # original "height - t" extrusion.
    support_height = HEIGHT - WALL
    support_z = support_height / 2.0  # spans z in [0, support_height]

    if LBUMPS > 1 and WBUMPS > 1:
        # Hollow tubes centered between the top studs (an (L-1) x (W-1) grid).
        for cx, cy in _stud_centers(LBUMPS - 1, WBUMPS - 1):
            tube = CylinderGeometry(
                TUBE_OUTER_DIAM / 2.0,
                support_height,
                radial_segments=RADIAL_SEGMENTS,
            ).translate(cx, cy, support_z)
            brick = boolean_union(brick, tube)

            bore = CylinderGeometry(
                STUD_DIAM / 2.0,
                support_height + 2.0 * EPS,
                radial_segments=RADIAL_SEGMENTS,
            ).translate(cx, cy, support_z)
            brick = boolean_difference(brick, bore)
    elif LBUMPS > 1:
        # Single row of solid thin posts along X (radius = WALL).
        for i in range(LBUMPS - 1):
            cx = (i - (LBUMPS - 2) / 2.0) * PITCH
            post = CylinderGeometry(
                WALL,
                support_height,
                radial_segments=RADIAL_SEGMENTS,
            ).translate(cx, 0.0, support_z)
            brick = boolean_union(brick, post)
    elif WBUMPS > 1:
        # Single row of solid thin posts along Y (radius = WALL).
        for j in range(WBUMPS - 1):
            cy = (j - (WBUMPS - 2) / 2.0) * PITCH
            post = CylinderGeometry(
                WALL,
                support_height,
                radial_segments=RADIAL_SEGMENTS,
            ).translate(0.0, cy, support_z)
            brick = boolean_union(brick, post)
    # else: 1x1 brick has no underside support; the bare shell is correct.

    return brick


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lego_brick")

    brick_red = model.material("lego_red", rgba=(0.72, 0.10, 0.10, 1.0))

    brick = model.part("brick")
    brick.visual(mesh_from_geometry(_build_brick(), "lego_brick"), material=brick_red)
    brick.inertial = Inertial.from_geometry(
        Box((TOTAL_LENGTH, TOTAL_WIDTH, HEIGHT + STUD_HEIGHT)),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, (HEIGHT + STUD_HEIGHT) / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    brick = object_model.get_part("brick")
    ctx.check("brick_present", brick is not None, "Expected a brick part.")
    if brick is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(brick)
    ctx.check("brick_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))

    # Footprint follows directly from the LEGO pitch and stud counts.
    ctx.check(
        "brick_length",
        abs(size[0] - TOTAL_LENGTH) <= 0.5 * MM,
        f"size={size!r}, expected length={TOTAL_LENGTH!r}",
    )
    ctx.check(
        "brick_width",
        abs(size[1] - TOTAL_WIDTH) <= 0.5 * MM,
        f"size={size!r}, expected width={TOTAL_WIDTH!r}",
    )
    # Overall height includes the shell plus the protruding top studs.
    ctx.check(
        "brick_height",
        abs(size[2] - (HEIGHT + STUD_HEIGHT)) <= 0.5 * MM,
        f"size={size!r}, expected height={HEIGHT + STUD_HEIGHT!r}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
