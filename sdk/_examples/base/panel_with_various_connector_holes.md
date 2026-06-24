---
title: 'Panel With Various Connector Holes'
description: 'Base SDK example: a flat I/O bracket panel with rows of D-subminiature connector cutouts (DB9/DB25/DB37-style D-shaped openings plus screw holes) cut as through-holes with ExtrudeWithHolesGeometry.'
tags:
  - sdk
  - base sdk
  - panel
  - connector
  - d-sub
  - cutout
  - through holes
  - extrude with holes
  - mesh geometry
---
# Panel With Various Connector Holes

This base-SDK example reproduces a flat metal I/O panel populated with several
columns of D-subminiature connector cutouts, the same teaching intent as the
CadQuery "panel with various connector holes" sample. Each connector opening is
the classic trapezoidal "D" outline with rounded corners, flanked by two
mounting-screw clearance holes. The panel is a single static part: a flat plate
extruded with many through-hole profiles via `ExtrudeWithHolesGeometry`.

It is useful for queries such as `connector panel`, `D-sub cutout`,
`DB9 / DB25 / DB37 holes`, `I/O bracket`, and `ExtrudeWithHolesGeometry`.

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# Real-world I/O panel: 0.40 m x 0.50 m face, 2 mm thick sheet.
# (Original CadQuery sample was in millimeters; converted to meters here.)
PANEL_W = 0.400
PANEL_H = 0.500
THICKNESS = 0.002

# Connector half-spans (meters). A "D-sub" outline is a rounded trapezoid:
#   half_top    = half width of the long (top) edge
#   half_bottom = half width of the short (bottom) edge
#   half_height = half height of the opening
#   corner_r    = corner round-over radius
#   screw_dx    = +/- x offset of the two mounting-screw holes
#   screw_r     = mounting-screw clearance radius
# Three connector sizes mirror DB37 (large), DB25 (medium), DB9 (small).
DB_LARGE = dict(half_top=0.02125, half_bottom=0.0204, half_height=0.0057,
                corner_r=0.0025, screw_dx=0.0235, screw_r=0.0016)
DB_MEDIUM = dict(half_top=0.0144, half_bottom=0.01359, half_height=0.0057,
                 corner_r=0.0025, screw_dx=0.01665, screw_r=0.0016)
DB_SMALL = dict(half_top=0.01025, half_bottom=0.009439, half_height=0.0057,
                corner_r=0.0025, screw_dx=0.0125, screw_r=0.0016)


def _circle_profile(cx, cy, r, *, segments=20):
    """Closed circle loop (counter-clockwise) for a screw clearance hole."""
    pts = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _dsub_outline(cx, cy, spec, *, arc_segments=4):
    """Rounded trapezoidal D-sub connector opening, centered at (cx, cy).

    Returns a closed counter-clockwise loop. The top edge is wider than the
    bottom edge, giving the characteristic 'D' keying shape.
    """
    ht = spec["half_top"]
    hb = spec["half_bottom"]
    hh = spec["half_height"]
    r = spec["corner_r"]

    # Corner centers, inset by the round-over radius.
    # Order corners counter-clockwise starting bottom-right.
    corners = [
        ((hb - r), (-hh + r), -math.pi / 2, 0.0),          # bottom-right
        ((ht - r), (hh - r), 0.0, math.pi / 2),            # top-right
        ((-ht + r), (hh - r), math.pi / 2, math.pi),       # top-left
        ((-hb + r), (-hh + r), math.pi, 3 * math.pi / 2),  # bottom-left
    ]

    pts = []
    for ccx, ccy, a0, a1 in corners:
        for i in range(arc_segments + 1):
            a = a0 + (a1 - a0) * i / arc_segments
            pts.append((cx + ccx + r * math.cos(a), cy + ccy + r * math.sin(a)))
    return pts


def _connector_holes(cx, cy, spec):
    """One connector: the D opening plus its two mounting-screw holes."""
    holes = [_dsub_outline(cx, cy, spec)]
    holes.append(_circle_profile(cx - spec["screw_dx"], cy, spec["screw_r"]))
    holes.append(_circle_profile(cx + spec["screw_dx"], cy, spec["screw_r"]))
    return holes


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="connector_panel")
    metal = model.material("panel_aluminum", rgba=(0.62, 0.64, 0.67, 1.0))

    # Outer panel profile (centered rectangle, counter-clockwise).
    hw, hh = PANEL_W / 2.0, PANEL_H / 2.0
    outer = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]

    # Column x-centers and per-column connector specs / row layouts.
    # Spacing chosen so every cutout stays well inside the panel face.
    col_sep = 0.060
    h_sep = 0.060      # large/medium row pitch
    h_sep_small = 0.030  # small (DB9-like) row pitch

    holes: list[list[tuple[float, float]]] = []

    def add_column(cx, spec, y_start, count, pitch):
        for idx in range(count):
            cy = y_start - idx * pitch
            holes.extend(_connector_holes(cx, cy, spec))

    # Column 1: large connectors, upper + lower groups.
    add_column(2 * col_sep, DB_LARGE, 0.210, 4, h_sep)
    add_column(2 * col_sep, DB_MEDIUM, -0.030, 4, h_sep)
    # Column 2: tall stack of small connectors.
    add_column(1 * col_sep, DB_SMALL, 0.225, 8, h_sep_small)
    # Column 3: large + medium groups again.
    add_column(0.0, DB_LARGE, 0.210, 4, h_sep)
    add_column(0.0, DB_MEDIUM, -0.030, 4, h_sep)
    # Column 4: another small stack.
    add_column(-1 * col_sep, DB_SMALL, 0.225, 8, h_sep_small)
    # Column 5: large group + a row of round screw bosses.
    add_column(-2 * col_sep, DB_LARGE, 0.210, 4, h_sep)
    for idx in range(4):
        cy = -0.030 - idx * h_sep
        holes.append(_circle_profile(-2 * col_sep, cy, 0.005, segments=24))
    # Column 6: final small stack + small D openings at the bottom.
    add_column(-3 * col_sep, DB_SMALL, 0.225, 8, h_sep_small)

    panel_geom = ExtrudeWithHolesGeometry(
        outer,
        holes,
        THICKNESS,
        cap=True,
        center=True,
    )

    panel = model.part("panel")
    panel.visual(
        mesh_from_geometry(panel_geom, "connector_panel_face"),
        material=metal,
        name="panel_face",
    )
    panel.inertial = Inertial.from_geometry(
        Box((PANEL_W, PANEL_H, THICKNESS)),
        mass=1.5,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel = object_model.get_part("panel")
    ctx.check("panel_part_present", panel is not None, "Expected a panel part.")
    if panel is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(panel)
    ctx.check("panel_aabb_present", aabb is not None, "Expected a world AABB for the panel.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("panel_width", 0.38 <= size[0] <= 0.42, f"size={size!r}")
    ctx.check("panel_height", 0.48 <= size[2 if size[2] > size[1] else 1] <= 0.52, f"size={size!r}")
    ctx.check(
        "panel_is_thin",
        min(size) <= 0.004,
        f"Expected a thin sheet panel; size={size!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
