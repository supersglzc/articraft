---
title: 'Polylines'
description: 'Base SDK example building an I-beam (H-section) by extruding a closed polyline profile, mirrored across the Y axis, with ExtrudeGeometry.'
tags:
  - sdk
  - base sdk
  - polyline
  - profile
  - extrude
  - i beam
  - h section
  - structural
  - mesh geometry
---
# Polylines

A polyline is just a chain of 2D points connected by straight segments. In the
native SDK you build that chain as an explicit list of `(x, y)` points and hand
the closed loop to `ExtrudeGeometry`, which extrudes the profile along `Z` into a
solid prism.

This example reproduces the classic polyline teaching case: one half of an
I-beam (H-section) cross-section is described as a chained polyline, mirrored
across the Y axis to form the full symmetric profile, and then extruded along
the beam length. The result is a single static structural part.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
    ExtrudeGeometry,
)

# I-beam parameters in meters (length, height, flange width, web/flange thickness).
L = 1.0
H = 0.20
W = 0.20
T = 0.012


def _half_ibeam_points() -> list[tuple[float, float]]:
    """The right half of the I-beam cross-section as a chained polyline.

    Points run from the top center, out along the top flange, down the flange
    edge, in along the underside, down the web, then back out and down the
    bottom flange, ending at the bottom center.
    """
    return [
        (0.0, H / 2.0),
        (W / 2.0, H / 2.0),
        (W / 2.0, H / 2.0 - T),
        (T / 2.0, H / 2.0 - T),
        (T / 2.0, T - H / 2.0),
        (W / 2.0, T - H / 2.0),
        (W / 2.0, -H / 2.0),
        (0.0, -H / 2.0),
    ]


def _ibeam_profile() -> list[tuple[float, float]]:
    """Build the full closed I-beam profile by mirroring the half across Y.

    The right half goes top-center -> down to bottom-center. Mirroring it across
    the Y axis (negate X) and walking it in reverse closes the loop back up the
    left side, yielding a single watertight counter-clockwise-friendly polygon.
    """
    right = _half_ibeam_points()
    # Mirror across the Y axis: negate X. Reverse so the loop stays continuous.
    left_mirrored = [(-x, y) for (x, y) in reversed(right)]
    # Drop the shared top-center and bottom-center duplicates at the seams.
    loop = right + left_mirrored[1:-1]
    return loop


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ibeam_polyline")
    steel = model.material("structural_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    beam = model.part("beam")
    # The closed I-beam profile lives in local XY; extrude it along Z to length L.
    geometry = ExtrudeGeometry.centered(_ibeam_profile(), L, cap=True)
    beam.visual(
        mesh_from_geometry(geometry, "ibeam"),
        material=steel,
        name="ibeam_shell",
    )
    beam.inertial = Inertial.from_geometry(
        Box((W, H, L)),
        mass=12.0,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    ctx.check("beam_part_present", beam is not None, "Expected a beam part.")
    if beam is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(beam)
    ctx.check("beam_aabb_present", aabb is not None, "Expected a world AABB for the beam.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # Profile spans full flange width in X and full section height in Y.
    ctx.check("beam_width", abs(size[0] - W) <= 0.01, f"size={size!r}")
    ctx.check("beam_height", abs(size[1] - H) <= 0.01, f"size={size!r}")
    # Extrusion runs the full length along Z.
    ctx.check("beam_length", abs(size[2] - L) <= 0.01, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
