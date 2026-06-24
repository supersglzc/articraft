---
title: 'Shelling to Create Thin Features'
description: 'Base SDK example reproducing the classic shelling teaching part: a solid box hollowed into a uniform-thickness open-top shell, approximated natively by subtracting an inset cavity with a boolean difference.'
tags:
  - sdk
  - base sdk
  - shell
  - hollow box
  - thin wall
  - open container
  - boolean difference
  - mesh geometry
---
# Shelling to Create Thin Features

The original CadQuery example demonstrates `Workplane.shell(...)`: it converts a
solid box into a thin shell of uniform wall thickness, optionally removing one or
more selected faces so the result reads as an open container rather than a sealed
hollow box. The canonical case is `box(2, 2, 2).faces("+Z").shell(...)`, which
hollows the cube and leaves the top (`+Z`) face open.

There is no B-rep `shell` operator in the native mesh SDK. The faithful native
approximation is to build the outer solid, build an inset cavity solid (the same
box scaled inward by the wall thickness on the closed faces, and run past the
open face so it removes that wall too), and subtract the cavity with
`boolean_difference(...)`. The remaining material is a uniform-thickness shell
with the `+Z` face removed, which is the same object and the same teaching intent
as the CadQuery `faces("+Z").shell(...)` result.

**Approximation note:** this reproduces the *result* of shelling (a uniform thin
wall with an open face) via an inset-cavity boolean cut, not the CadQuery
`shell()` B-rep offset operation. There is no native uniform-thickness offset or
face selector; the wall thickness here is enforced by construction (cavity inset)
rather than by a true surface offset, and outside edges are square rather than
filleted.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

# Parameters (meters). The original CadQuery cube was a unitless 2 x 2 x 2 box
# shelled with a 0.1 wall. Interpreted here as a small open container and scaled
# to meters: a 0.20 m cube with a 10 mm wall.
SIZE = 0.20  # outer edge length (X, Y, Z)
WALL = 0.010  # uniform wall thickness on the closed faces
_EPS = 1.0e-4  # overshoot so the cavity fully clears the open (+Z) face


def _shell_geometry() -> "BoxGeometry":
    # Outer solid cube, centered at the origin.
    outer = BoxGeometry((SIZE, SIZE, SIZE))

    # Inset cavity. It is shrunk by one wall thickness on each of the four sides
    # and on the bottom, but extended past the top so the +Z face is fully
    # removed -> an open-top container with uniform walls everywhere else.
    inner_xy = SIZE - 2.0 * WALL
    inner_z = SIZE - WALL + _EPS  # bottom wall kept, top wall removed
    cavity = BoxGeometry((inner_xy, inner_xy, inner_z))
    # Shift the cavity up by half a wall so its floor sits at +WALL/2 above the
    # box floor (leaving a WALL-thick bottom) while its roof pokes through +Z.
    cavity = cavity.translate(0.0, 0.0, WALL / 2.0)

    return boolean_difference(outer, cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shelled_open_container")
    finish = model.material("shell_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_geometry(_shell_geometry(), "shell"),
        material=finish,
        name="shell_body",
    )
    shell.inertial = Inertial.from_geometry(
        Box((SIZE, SIZE, SIZE)),
        mass=0.40,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    ctx.check("shell_part_present", shell is not None, "Expected a shell part.")
    if shell is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(shell)
    ctx.check("shell_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # The shell's outer bounding box should still equal the original cube: the
    # walls reach the outer faces, only material on the inside was removed.
    for axis, label in enumerate(("x", "y", "z")):
        ctx.check(
            f"shell_outer_{label}",
            abs(size[axis] - SIZE) <= 0.002,
            f"expected outer {label}={SIZE}, size={size!r}",
        )
    return ctx.report()


object_model = build_object_model()
```
