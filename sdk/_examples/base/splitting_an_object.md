---
title: 'Splitting an Object'
description: 'Base SDK example reproducing the CadQuery split teaching part: a cube with a centered through hole, cut in half by a workplane and keeping one half, approximated natively with a half-space intersection.'
tags:
  - sdk
  - base sdk
  - split
  - half section
  - section cut
  - boolean intersection
  - through hole
  - mesh geometry
---
# Splitting an Object

The original CadQuery example builds a unit cube, drills a through hole down its
top face with `cutThruAll()`, then calls `split(keepTop=True)` to slice the solid
with a workplane and discard everything on one side, retaining only the top half.

`split(...)` is a CadQuery-only B-rep operation: it intersects the solid against
an infinite half-space defined by a workplane and keeps the requested side (and
can optionally keep both halves). There is no exact native equivalent in the
base SDK. The faithful native approximation is to intersect the part against a
large half-space box positioned so it overlaps only the side we want to keep,
which is exactly what `split(keepTop=True)` does geometrically. The teaching
intent (a cube, with a through hole, cleanly sectioned so only one half remains)
is preserved. See the note at the end for the limitation.

The construction order matches the original: build the cube, subtract a vertical
clearance cylinder through it (the `cutThruAll` hole), then `boolean_intersection`
with the keep-side half-space block to perform the split.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_intersection,
    mesh_from_geometry,
)

# Parameters (meters). The original CadQuery cube was a unitless 1 x 1 x 1 with a
# 0.25-radius through hole; interpreted here as a 0.10 m machined block.
CUBE = 0.10  # cube edge along X, Y, Z
HOLE_RADIUS = 0.025  # 0.25 of the edge, matching the original ratio

_SEGMENTS = 48
_EPS = 1.0e-4  # small overshoot so cutters fully clear the faces
_BIG = 10.0 * CUBE  # half-space block large enough to act as a workplane cut


def _split_geometry() -> "BoxGeometry":
    # Unit cube centered at the origin.
    solid = BoxGeometry((CUBE, CUBE, CUBE))

    # cutThruAll: a vertical clearance hole drilled straight down through the
    # top (+Z) face, centered on the cube. The cylinder axis is local Z.
    hole = CylinderGeometry(
        HOLE_RADIUS,
        CUBE + 2.0 * _EPS,
        radial_segments=_SEGMENTS,
    )
    solid = boolean_difference(solid, hole)

    # split(keepTop=True): the workplane sits at the section plane (here the
    # horizontal mid-plane z=0) and only the +Z side is retained. We approximate
    # the half-space with a large block that spans well past the cube on the keep
    # side and starts exactly at the section plane, then intersect.
    keep_half_space = BoxGeometry((_BIG, _BIG, _BIG))
    # Shift the block up by half its size so its lower face lands on z=0; only the
    # top half of the cube survives the intersection.
    keep_half_space = keep_half_space.translate(0.0, 0.0, _BIG / 2.0)
    solid = boolean_intersection(solid, keep_half_space)
    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="splitting_an_object")
    metal = model.material("split_block_metal", rgba=(0.58, 0.60, 0.64, 1.0))

    block = model.part("block")
    block.visual(
        mesh_from_geometry(_split_geometry(), "split_block"),
        material=metal,
        name="block_body",
    )
    block.inertial = Inertial.from_geometry(
        Box((CUBE, CUBE, CUBE / 2.0)),
        mass=0.40,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    block = object_model.get_part("block")
    ctx.check("block_part_present", block is not None, "Expected a block part.")
    if block is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(block)
    ctx.check("block_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # Footprint stays full cube in X/Y.
    ctx.check(
        "block_footprint_x",
        abs(size[0] - CUBE) <= 0.002,
        f"expected x={CUBE}, size={size!r}",
    )
    ctx.check(
        "block_footprint_y",
        abs(size[1] - CUBE) <= 0.002,
        f"expected y={CUBE}, size={size!r}",
    )
    # Split removed the bottom half: only ~half the height remains.
    ctx.check(
        "block_split_height",
        abs(size[2] - CUBE / 2.0) <= 0.003,
        f"expected z~={CUBE / 2.0}, size={size!r}",
    )
    # The retained half sits entirely on the +Z side of the section plane.
    ctx.check(
        "block_kept_top_half",
        mins[2] >= -0.002,
        f"expected bottom at section plane z=0, mins={mins!r}",
    )
    return ctx.report()


object_model = build_object_model()
```

## Native approximation note

`split(...)` is a CadQuery-only B-rep workplane section operation with no exact
native equivalent. Here it is approximated by `boolean_intersection` against a
large half-space block whose face is placed on the section plane, which keeps the
chosen side just as `keepTop=True` does. The retained half is a real watertight
solid, but this approximation keeps a single side; CadQuery's option to retain
both halves as separate solids would require running the intersection (and its
complement) separately.
