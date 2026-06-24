---
title: 'Resin Mold'
description: 'Base SDK example of a single-piece resin casting mold: a rectangular block with a revolved wire-channel pocket, four corner mounting holes, and three transverse fill holes, all formed by boolean_difference of negative volumes from the block.'
tags:
  - sdk
  - base sdk
  - resin
  - mold
  - casting mold
  - wire pocket
  - mounting holes
  - fill holes
  - revolved channel
  - boolean difference
  - cylinder geometry
  - lathe geometry
  - mesh geometry
---
# Resin Mold

This base-SDK example reproduces a simple resin casting mold. A solid
rectangular block has a wire-channel pocket carved into its top, four
mounting holes at the corners, and three transverse fill holes on a long
side. It is useful for queries such as `resin mold`, `casting mold`,
`wire pocket`, `fill holes`, `mounting holes`, and `boolean_difference mold`.

The original CadQuery model built the central pocket as a revolved 2D wire
profile (a thin channel that bulges to a wider resin pool in the middle),
then cut it from the block; the corner holes and fill holes were drilled with
face workplanes and `.hole(...)`. None of those B-rep operations exist in the
native SDK, so the mold is built the closest faithful way: every removed
feature is authored as its own watertight negative solid and subtracted from
the block with `boolean_difference`. The bulging wire channel is approximated
as a revolved `LatheGeometry` channel (a body of revolution about the long
axis) joined with a flat wire pass-through groove at each end.

Approximation note: the SDK has no B-rep revolve-and-cut, face-workplane
drilling, or edge fillet. The pocket cross-section is approximated by a
revolved lathe profile instead of an exact `revolve(...)` of a polyline, and
the soft pocket fillet of the CadQuery source is not reproduced. The result is
geometrically and functionally equivalent (same block, same channel pool, same
corner holes, same three fill holes) but the carved-feature edges are crisp
rather than filleted.

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# Parameters (meters; original CadQuery source was in millimeters).
MOUNT_HOLES = True

ML = 0.120  # mold length (X)
MW = 0.040  # mold width (Y)
MH = 0.013  # mold height (Z)

WD = 0.006  # wire diameter (thin channel)
RT = 0.007  # resin pool extra thickness above the wire channel
RL = 0.050  # resin pool length
RWPL = 0.010  # ramp length from wire channel up to the resin pool

MHD = 0.007  # mount hole diameter
MHT = 0.003  # mount hole inset from each edge
FHD = 0.006  # fill hole diameter


def _wire_channel() -> "object":
    """Revolved wire/resin channel cut, as a body of revolution about +X.

    Profile (radius vs. axial position) mirrors the CadQuery wire: a thin
    wire-radius channel along the full length that ramps up to a wider
    resin-pool radius across the central RL span. LatheGeometry revolves a
    (radius, z) profile about local Z; we build it about Z then rotate so the
    revolution axis becomes the block's long axis (+X), sitting at the top
    face so the channel opens upward.
    """
    wire_r = WD / 2.0
    pool_r = WD / 2.0 + RT
    half_len = ML / 2.0
    pool_half = RL / 2.0
    ramp_start = pool_half + RWPL

    # (radius, z) profile along the revolution axis (z here = axial position).
    profile = [
        (0.0, -half_len),
        (wire_r, -half_len),
        (wire_r, -ramp_start),
        (pool_r, -pool_half),
        (pool_r, pool_half),
        (wire_r, ramp_start),
        (wire_r, half_len),
        (0.0, half_len),
    ]
    channel = LatheGeometry(profile, segments=48, closed=True)
    # Revolve axis Z -> X, then lift to the top face of the block.
    channel.rotate_y(math.pi / 2.0)
    channel.translate(0.0, 0.0, MH)
    return channel


def _mount_hole(px: float, py: float) -> "object":
    """Vertical through-hole cylinder at (px, py)."""
    bore = CylinderGeometry(MHD / 2.0, MH * 2.0, radial_segments=32, closed=True)
    bore.translate(px, py, MH / 2.0)
    return bore


def _fill_hole(cx: float) -> "object":
    """Transverse (along Y) fill hole at axial position cx, centered in Z."""
    bore = CylinderGeometry(FHD / 2.0, MW * 2.0, radial_segments=32, closed=True)
    bore.rotate_x(math.pi / 2.0)
    bore.translate(cx, 0.0, MH / 2.0)
    return bore


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="resin_mold")
    silicone = model.material("mold_silicone", rgba=(0.20, 0.45, 0.55, 1.0))

    # Block, centered in XY and resting on z=0.
    block = BoxGeometry((ML, MW, MH))
    block.translate(0.0, 0.0, MH / 2.0)

    # Collect every negative volume and union them for one clean subtraction.
    cutters = [_wire_channel()]

    if MOUNT_HOLES:
        px = ML / 2.0 - MHT - MHD / 2.0
        py = MW / 2.0 - MHT - MHD / 2.0
        cutters.extend(
            [
                _mount_hole(px, py),
                _mount_hole(-px, py),
                _mount_hole(-px, -py),
                _mount_hole(px, -py),
            ]
        )

    for cx in (-RL / 2.0, 0.0, RL / 2.0):
        cutters.append(_fill_hole(cx))

    negative = cutters[0]
    for cutter in cutters[1:]:
        negative = boolean_union(negative, cutter)

    mold = boolean_difference(block, negative)

    part = model.part("mold")
    part.visual(
        mesh_from_geometry(mold, "resin_mold"),
        material=silicone,
        name="mold_body",
    )
    part.inertial = Inertial.from_geometry(
        Box((ML, MW, MH)),
        mass=0.18,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mold = object_model.get_part("mold")
    ctx.check("mold_part_present", mold is not None, "Expected a mold part.")
    if mold is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(mold)
    ctx.check("mold_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("mold_length", abs(size[0] - ML) <= 0.002, f"size={size!r}")
    ctx.check("mold_width", abs(size[1] - MW) <= 0.002, f"size={size!r}")
    ctx.check("mold_height", abs(size[2] - MH) <= 0.002, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
