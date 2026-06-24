---
title: 'Embossed Braille Plate'
description: 'Base SDK example that embosses a Grade-1 braille word as spherical-cap dots on a flat reading plate by unioning shallow sphere caps onto a base slab.'
tags:
  - sdk
  - base sdk
  - braille
  - embossed
  - tactile
  - spherical cap
  - plate
  - boolean union
  - mesh geometry
---
# Embossed Braille Plate

This base-SDK example reproduces the classic CadQuery braille demo as native
mesh geometry: a flat reading plate with raised braille dots shaped as shallow
spherical caps. The teaching intent is the embossing method, not a CadQuery
fillet trick. Each dot is a sphere whose buried portion is clipped by the plate
via `boolean_union`, so only a spherical cap of the requested dot height stays
proud of the reading surface. It is useful for queries such as `braille`,
`embossed dots`, `tactile plate`, and `spherical cap union`.

All braille geometry is one rigid part. Dots are placed on a standard 2x3 cell
grid (with extension dots 7/8) using the Unicode braille bit pattern, so the
same routine renders any braille string.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    Inertial,
    MeshGeometry,
    SphereGeometry,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# --- Braille cell geometry (meters). Source demo uses millimeters. ---
HORIZONTAL_INTERDOT = 0.0025  # dot spacing across a cell
VERTICAL_INTERDOT = 0.0025  # dot spacing down a cell
INTERCELL = 0.0060  # cell-to-cell pitch
INTERLINE = 0.0100  # line-to-line pitch
DOT_HEIGHT = 0.0005  # proud cap height
DOT_DIAMETER = 0.0013  # cap base diameter
BASE_THICKNESS = 0.0030  # reading-plate thickness

# Word to emboss (Grade-1 braille for "code"): c o d e
TEXT = "⠉⠕⠙⠑"

# Unicode braille dot order -> (col, row) cell positions.
# Dots 1..6 fill the 2x3 cell; 7/8 are the extension row below.
_CELL_POSITIONS = (
    (0.0, 2.0),  # dot 1
    (0.0, 1.0),  # dot 2
    (0.0, 0.0),  # dot 3
    (1.0, 2.0),  # dot 4
    (1.0, 1.0),  # dot 5
    (1.0, 0.0),  # dot 6
    (0.0, -1.0),  # dot 7
    (1.0, -1.0),  # dot 8
)
_BLANK = ord("⠀")


def _weld(geometry: MeshGeometry, *, tol: float = 1e-7) -> MeshGeometry:
    """Merge coincident vertices so the mesh is a watertight manifold.

    Revolved primitives such as ``SphereGeometry`` emit duplicated pole and seam
    vertices, which leave the surface non-manifold for boolean operations.
    Welding by quantized position closes those seams before the union.
    """
    welded = MeshGeometry()
    index: dict[tuple[int, int, int], int] = {}
    remap: list[int] = []
    for x, y, z in geometry.vertices:
        key = (round(x / tol), round(y / tol), round(z / tol))
        if key not in index:
            index[key] = welded.add_vertex(x, y, z)
        remap.append(index[key])
    for a, b, c in geometry.faces:
        a, b, c = remap[a], remap[b], remap[c]
        if a == b or b == c or a == c:
            continue
        welded.add_face(a, b, c)
    return welded


def _cap_sphere_radius() -> float:
    """Sphere radius whose cap of DOT_HEIGHT has base diameter DOT_DIAMETER."""
    r = DOT_DIAMETER / 2.0
    return (r * r + DOT_HEIGHT * DOT_HEIGHT) / (2.0 * DOT_HEIGHT)


def _dot_xy_positions() -> list[tuple[float, float]]:
    """World XY centers (meters) of every raised dot for TEXT, single line."""
    positions: list[tuple[float, float]] = []
    margin_x = 2.0 * HORIZONTAL_INTERDOT
    margin_y = 2.0 * VERTICAL_INTERDOT
    for cell_index, char in enumerate(TEXT):
        bits = ord(char) - _BLANK
        cell_x = margin_x + cell_index * INTERCELL
        for dot_index, (col, row) in enumerate(_CELL_POSITIONS):
            if bits & (1 << dot_index):
                x = cell_x + col * HORIZONTAL_INTERDOT
                y = margin_y + row * VERTICAL_INTERDOT
                positions.append((x, y))
    return positions


def _plate_size(dot_xy: list[tuple[float, float]]) -> tuple[float, float]:
    """Plate footprint sized to frame all dots with an even margin."""
    margin_x = 2.0 * HORIZONTAL_INTERDOT
    margin_y = 2.0 * VERTICAL_INTERDOT
    max_x = max(x for x, _ in dot_xy)
    max_y = max(y for _, y in dot_xy)
    min_y = min(y for _, y in dot_xy)
    width = max_x + 2.0 * margin_x
    height = (max_y - min_y) + 2.0 * margin_y
    return width, height


def _build_plate_geometry():
    dot_xy = _dot_xy_positions()
    width, height = _plate_size(dot_xy)
    cx, cy = width / 2.0, height / 2.0

    # Reading plate: a box whose top face sits at z = BASE_THICKNESS.
    geom = BoxGeometry((width, height, BASE_THICKNESS))
    geom.translate(cx, cy, BASE_THICKNESS / 2.0)

    sphere_r = _cap_sphere_radius()
    # Sphere center so that exactly DOT_HEIGHT protrudes above the plate top.
    center_z = BASE_THICKNESS + DOT_HEIGHT - sphere_r
    for x, y in dot_xy:
        dot = _weld(SphereGeometry(sphere_r, width_segments=24, height_segments=16))
        dot.translate(x, y, center_z)
        # The plate slab clips the buried portion, leaving a clean cap.
        geom = boolean_union(geom, dot)
    return geom, width, height


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="embossed_braille_plate")
    finish = model.material("braille_plate", rgba=(0.86, 0.85, 0.82, 1.0))

    geom, width, height = _build_plate_geometry()

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(geom, "braille_plate"),
        material=finish,
        name="plate_shell",
    )
    plate.inertial = Inertial.from_geometry(
        Box((width, height, BASE_THICKNESS)),
        mass=0.05,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("plate")
    ctx.check("plate_present", plate is not None, "Expected a plate part.")
    if plate is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(plate)
    ctx.check("plate_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))

    # Total height = plate thickness + the proud cap height, within tolerance.
    expected_z = BASE_THICKNESS + DOT_HEIGHT
    ctx.check(
        "emboss_height",
        abs(size[2] - expected_z) <= 0.0002,
        f"size={size!r} expected_z={expected_z!r}",
    )
    # Dots must rise above the reading surface, not merely sit flush.
    ctx.check(
        "dots_are_proud",
        size[2] > BASE_THICKNESS + DOT_HEIGHT * 0.5,
        f"size={size!r}",
    )
    # Footprint should read as a small handheld reading plate.
    ctx.check(
        "plate_footprint",
        0.01 <= size[0] <= 0.08 and 0.005 <= size[1] <= 0.05,
        f"size={size!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
