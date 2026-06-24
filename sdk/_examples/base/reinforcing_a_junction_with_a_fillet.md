---
title: 'Reinforcing a Junction with a Fillet'
description: 'Base SDK example reproducing a pipe-connector: a bolted square base plate, a hollow riser tube, and a filleted collar that reinforces the plate-to-tube junction using native mesh geometry and booleans.'
tags:
  - sdk
  - base sdk
  - pipe connector
  - junction
  - fillet
  - reinforcement
  - boolean
  - mesh geometry
---
# Reinforcing a Junction with a Fillet

This base-SDK example reproduces the classic pipe-connector teaching object: a
square base plate with four countersunk bolt holes, a hollow riser tube standing
on it, and a reinforcing fillet at the junction where the tube meets the plate.

In the native mesh SDK there is no edge-selection `fillet(...)` operator, so the
reinforcement is modeled directly as geometry: a concave-to-the-eye conical
collar (a ramped gusset ring) is unioned around the base of the riser tube so the
plate-to-tube junction reads as smoothly reinforced rather than a thin, weak
right-angle joint. This is the same functional intent as filleting the circular
junction edge in the original CAD recipe.

```python
from __future__ import annotations

import math

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

# All dimensions in meters. The source used millimeters; we keep the same
# proportions at a realistic small-fitting scale.
PLATE_SIDE = 0.060  # 60 mm square base plate
PLATE_THICK = 0.008  # 8 mm thick plate
BOLT_CIRCLE = 0.040  # bolt holes on a 40 mm square pattern
BOLT_HOLE_R = 0.0035  # clearance hole radius
CSK_TOP_R = 0.0070  # countersink top radius
CSK_DEPTH = 0.0030  # countersink depth

TUBE_OUTER_R = 0.016  # riser outer radius
TUBE_BORE_R = 0.011  # riser through-bore radius
TUBE_HEIGHT = 0.040  # riser height above the plate

FILLET_R = 0.006  # reinforcing collar reach / rise at the junction


def _countersunk_hole(cx: float, cy: float) -> "object":
    """A through clearance hole plus a countersink cone as one cutter mesh."""
    # Straight clearance shaft spanning the full plate (with margin).
    shaft = CylinderGeometry(BOLT_HOLE_R, PLATE_THICK + 0.004, radial_segments=24)
    shaft = shaft.translate(cx, cy, PLATE_THICK / 2.0)

    # Countersink cone opening at the top face (wide at top, narrow at bottom).
    csk = ConeGeometry(CSK_TOP_R, CSK_DEPTH, radial_segments=24)
    # ConeGeometry is centered on Z; its apex points to +Z. Flip so the wide
    # mouth sits at the top face (z = PLATE_THICK) and tapers downward.
    csk = csk.rotate_x(math.pi)
    csk = csk.translate(cx, cy, PLATE_THICK - CSK_DEPTH / 2.0)
    return boolean_union(shaft, csk)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="filleted_pipe_connector")
    steel = model.material("connector_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    # --- Base plate -------------------------------------------------------
    plate = BoxGeometry((PLATE_SIDE, PLATE_SIDE, PLATE_THICK))
    plate = plate.translate(0.0, 0.0, PLATE_THICK / 2.0)

    # --- Riser tube (hollow) over the plate -------------------------------
    riser_outer = CylinderGeometry(TUBE_OUTER_R, TUBE_HEIGHT, radial_segments=48)
    riser_outer = riser_outer.translate(0.0, 0.0, PLATE_THICK + TUBE_HEIGHT / 2.0)

    # --- Reinforcing fillet collar at the junction ------------------------
    # A short cone, wide at the plate and narrowing as it rises, unioned around
    # the tube base. This is the native stand-in for filleting the junction edge.
    collar = ConeGeometry(TUBE_OUTER_R + FILLET_R, FILLET_R, radial_segments=48)
    collar = collar.translate(0.0, 0.0, PLATE_THICK + FILLET_R / 2.0)

    body = boolean_union(plate, riser_outer)
    body = boolean_union(body, collar)

    # --- Through-bore down the riser and into the plate -------------------
    bore = CylinderGeometry(TUBE_BORE_R, PLATE_THICK + TUBE_HEIGHT + 0.004, radial_segments=48)
    bore = bore.translate(0.0, 0.0, (PLATE_THICK + TUBE_HEIGHT) / 2.0)
    body = boolean_difference(body, bore)

    # --- Four countersunk bolt holes near the plate corners ---------------
    half = BOLT_CIRCLE / 2.0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body = boolean_difference(body, _countersunk_hole(sx * half, sy * half))

    connector = model.part("connector")
    connector.visual(
        mesh_from_geometry(body, "filleted_pipe_connector"),
        material=steel,
        name="connector_body",
    )
    connector.inertial = Inertial.from_geometry(
        Box((PLATE_SIDE, PLATE_SIDE, PLATE_THICK + TUBE_HEIGHT)),
        mass=0.35,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    connector = object_model.get_part("connector")
    ctx.check("connector_present", connector is not None, "Expected a connector part.")
    if connector is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(connector)
    ctx.check("connector_aabb", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # Footprint should match the plate; the collar must not exceed the plate.
    ctx.check("plate_footprint", 0.058 <= max(size[0], size[1]) <= 0.062, f"size={size!r}")
    # Overall height is plate + riser.
    expected_h = PLATE_THICK + TUBE_HEIGHT
    ctx.check("overall_height", abs(size[2] - expected_h) <= 0.002, f"size={size!r}")
    # Collar reach must stay inside the plate so the junction is reinforced,
    # not overhanging.
    ctx.check(
        "collar_within_plate",
        (TUBE_OUTER_R + FILLET_R) < PLATE_SIDE / 2.0,
        "Reinforcing collar should not overhang the plate edge.",
    )
    return ctx.report()


object_model = build_object_model()
```
