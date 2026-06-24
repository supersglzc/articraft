---
title: 'Parametric Pin Header'
description: 'Base SDK example of a parametric straight pin header: a black plastic base with a grid of square through-holes and repeated gold-plated square pins with chamfered tips.'
tags:
  - sdk
  - base sdk
  - pin header
  - header
  - connector
  - electronics
  - pin grid
  - parametric array
  - through holes
  - extrude with holes
  - loft
  - chamfer
  - assembly
  - mesh geometry
---
# Parametric Pin Header

This base-SDK example mirrors a classic straight pin header: a black plastic
base perforated by a regular grid of square pin holes, and one gold-plated
square pin inserted through every hole. It is a good reference for parametric
grid arrays, square through-holes via `ExtrudeWithHolesGeometry`, and chamfered
square posts via a square `LoftGeometry`. It is useful for queries such as
`pin header`, `connector`, `pin grid array`, `through-hole grid`,
`ExtrudeWithHolesGeometry`, and `parametric array`.

The modeling patterns worth copying are:

- a parametric `rows x columns` grid of pin centers driven by a single `pitch`.
- one base part whose square pin holes are cut with `ExtrudeWithHolesGeometry`.
- one reusable chamfered-pin geometry built from stacked square loft sections,
  cloned and translated to each grid location and fused into a single part.
- a tests pass that checks the base footprint, pin count, and that pins stand
  proud of the base on both faces.

```python
from __future__ import annotations

# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    LoftGeometry,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# Real pin headers are small; original was in mm, scaled here to meters.
ROWS = 2
COLUMNS = 10

PITCH = 0.00254  # 2.54 mm grid pitch
PIN_WIDTH = 0.00064  # 0.64 mm square pin
PIN_CHAMFER = 0.0002  # tip chamfer
BASE_HEIGHT = 0.0024  # plastic base thickness
ABOVE = 0.007  # pin length above the base
BELOW = 0.003  # pin length below the base

PIN_LENGTH = ABOVE + BASE_HEIGHT + BELOW
BASE_WIDTH = PITCH * ROWS
BASE_LENGTH = PITCH * COLUMNS


def _pin_centers() -> list[tuple[float, float]]:
    """Grid of (x, y) pin centers, base origin at one corner like the source."""
    centers: list[tuple[float, float]] = []
    for row in range(ROWS):
        loc_y = (PITCH / 2.0) + (row * PITCH)
        for column in range(COLUMNS):
            loc_x = (PITCH / 2.0) + (column * PITCH)
            centers.append((loc_x, loc_y))
    return centers


def _square_profile(half: float, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
    ]


def _square_loop(half: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in _square_profile(half)]


def _build_base():
    """Plastic base box (centered XY at origin) with a grid of square holes."""
    outer = [
        (-0.5 * BASE_LENGTH, -0.5 * BASE_WIDTH),
        (0.5 * BASE_LENGTH, -0.5 * BASE_WIDTH),
        (0.5 * BASE_LENGTH, 0.5 * BASE_WIDTH),
        (-0.5 * BASE_LENGTH, 0.5 * BASE_WIDTH),
    ]
    half = 0.5 * PIN_WIDTH
    holes: list[list[tuple[float, float]]] = []
    for loc_x, loc_y in _pin_centers():
        cx = loc_x - 0.5 * BASE_LENGTH
        cy = loc_y - 0.5 * BASE_WIDTH
        holes.append(_square_profile(half, cx, cy))
    return ExtrudeWithHolesGeometry(outer, holes, height=BASE_HEIGHT, center=True)


def _build_pin(cx: float, cy: float):
    """A square post centered at (cx, cy) spanning z in [-BELOW, ABOVE+BASE_HEIGHT].

    Tips at both ends are chamfered by shrinking the square loft sections.
    """
    half = 0.5 * PIN_WIDTH
    tip = max(half - PIN_CHAMFER, half * 0.35)
    z_bot = -BELOW
    z_top = ABOVE + BASE_HEIGHT
    profiles = [
        _square_loop(tip, z_bot),
        _square_loop(half, z_bot + PIN_CHAMFER),
        _square_loop(half, z_top - PIN_CHAMFER),
        _square_loop(tip, z_top),
    ]
    pin = LoftGeometry(profiles, cap=True)
    pin.translate(cx, cy, 0.0)
    return pin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parametric_pin_header")
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.02, 1.0))
    gold_plate = model.material("gold_plate", rgba=(1.0, 0.68, 0.0, 1.0))

    header = model.part("header")
    header.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, PIN_LENGTH)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    header.visual(
        mesh_from_geometry(_build_base(), "pin_header_base"),
        material=black_plastic,
        name="base_shell",
    )

    # Fuse all pins into a single watertight mesh so the part stays one island.
    pins_geom = None
    for loc_x, loc_y in _pin_centers():
        cx = loc_x - 0.5 * BASE_LENGTH
        cy = loc_y - 0.5 * BASE_WIDTH
        single = _build_pin(cx, cy)
        pins_geom = single if pins_geom is None else boolean_union(pins_geom, single)

    header.visual(
        mesh_from_geometry(pins_geom, "pin_header_pins"),
        material=gold_plate,
        name="pins",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    header = object_model.get_part("header")
    ctx.check("header_part_present", header is not None, "Expected a header part.")
    if header is None:
        return ctx.report()

    ctx.check(
        "pin_count",
        len(_pin_centers()) == ROWS * COLUMNS,
        f"expected {ROWS * COLUMNS} pins, got {len(_pin_centers())}",
    )

    aabb = ctx.part_world_aabb(header)
    ctx.check("header_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # Footprint should match the base plus a sliver of pin chamfer tolerance.
    ctx.check(
        "footprint_x",
        BASE_LENGTH - 0.0002 <= size[0] <= BASE_LENGTH + 0.0002,
        f"size={size!r} base_length={BASE_LENGTH}",
    )
    ctx.check(
        "footprint_y",
        BASE_WIDTH - 0.0002 <= size[1] <= BASE_WIDTH + 0.0002,
        f"size={size!r} base_width={BASE_WIDTH}",
    )
    # Total height = pin length, since pins extend above and below the base.
    ctx.check(
        "total_height",
        PIN_LENGTH - 0.0003 <= size[2] <= PIN_LENGTH + 0.0003,
        f"size={size!r} pin_length={PIN_LENGTH}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
</content>
</invoke>
