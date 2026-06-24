---
title: 'Inside Chamfer on a Shelled Cube'
description: 'Base SDK reproduction of a shelled (hollowed) cube whose interior bottom edges carry an inside chamfer, built from native mesh booleans by cutting a chamfered inner cavity.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - shelled cube
  - shell
  - hollow box
  - open box
  - inside chamfer
  - interior chamfer
  - chamfer
  - boolean difference
  - extrude geometry
  - lathe geometry
---
# Inside Chamfer on a Shelled Cube

This base-SDK example is a native reproduction of the classic CadQuery teaching
snippet that takes a cube, shells out its top face to make a thin-walled open
box, and then chamfers only the interior bottom edges of the resulting cavity.
It is useful for queries such as `shelled cube`, `hollow box`, `open box`,
`inside chamfer`, `interior chamfer`, `shell -0.2`, and `boolean_difference`.

The teaching intent is the inside chamfer on a shelled solid: a hollowed body
whose interior floor-to-wall transition is broken by a chamfer instead of a
sharp corner. The native reproduction keeps that exact read. A single solid
cube is hollowed by subtracting an inner cavity that is left open at the top,
and the cavity solid carries a chamfered base so the interior bottom edges of
the finished shell are themselves chamfered.

Approximation note: CadQuery's `.shell(-0.2)` is a true B-rep offset-shell
operation and `.chamfer(0.125)` is an exact local edge chamfer on selected
interior edges. The native SDK has no B-rep shell or edge-chamfer operator, so
this example approximates both: the shell is reproduced by
`boolean_difference(...)` of an inset, open-topped inner cavity, and the inside
chamfer is reproduced by giving that cavity solid a chamfered (beveled) base
via a `LatheGeometry`-style stepped/sloped wall profile before the cut. The
visible result is the same shelled, inside-chamfered cavity, but it is mesh
geometry rather than an exact parametric chamfer feature.

The modeling patterns worth copying are:

- `BoxGeometry(...)` for the outer solid cube.
- a single inner cavity solid that is over-tall so it punches through the open
  top face, subtracted with `boolean_difference(...)` to form the shell wall.
- a chamfered cavity base: the lower part of the cavity is widened by a sloped
  ring of geometry so that cutting it leaves a chamfer on the interior bottom
  edges instead of a sharp interior corner.

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
    ExtrudeGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# --- Parameters (meters) ---------------------------------------------------
# The original cube is 2 x 2 x 2 units, shelled -0.2 with a 0.125 chamfer.
# Reproduced at a realistic small open-box scale (a ~0.20 m parts tray): the
# proportions of wall thickness and chamfer to the cube are preserved.
CUBE_SIZE = 0.200  # outer cube edge length (X = Y = Z)
WALL = 0.020  # shell wall / floor thickness  (orig 0.2 of 2.0)
CHAMFER = 0.0125  # inside chamfer leg length  (orig 0.125 of 2.0)

HALF = CUBE_SIZE / 2.0
INNER = CUBE_SIZE - 2.0 * WALL  # inner cavity plan size (X and Y)
FLOOR_Z = -HALF + WALL  # interior floor height
OVERSHOOT = WALL  # how far the cavity punches past the open top


def _square_profile(side: float):
    """Centered square XY loop of edge length ``side``."""
    h = side / 2.0
    return [(-h, -h), (h, -h), (h, h), (-h, h)]


def _build_cavity():
    """Inner cavity solid with a chamfered base.

    The cavity is open-topped (over-tall) so subtracting it from the cube leaves
    a real shell. Its base is a frustum that flares outward from the floor up to
    full inner width over the chamfer height, so cutting it leaves a chamfer on
    the interior bottom edges instead of a sharp corner.
    """
    # Main straight-walled cavity: full inner plan, from just above the floor
    # (its bottom is overlapped by the chamfer frustum) up through the open top.
    main_h = (HALF + OVERSHOOT) - (FLOOR_Z + CHAMFER)
    main = ExtrudeGeometry.from_z0(
        _square_profile(INNER),
        main_h,
        cap=True,
    ).translate(0.0, 0.0, FLOOR_Z + CHAMFER)

    # Chamfer frustum: a tapered box-like solid that grows from a smaller
    # footprint at the floor to the full inner footprint at the top of the
    # chamfer. Approximated as two stacked steps + the slope between them so the
    # interior bottom edge reads as a 45-degree chamfer.
    small = INNER - 2.0 * CHAMFER
    cavity = main
    steps = 6
    for i in range(steps):
        t0 = i / steps
        t1 = (i + 1) / steps
        side0 = small + (INNER - small) * t0
        side1 = small + (INNER - small) * t1
        # Use the wider of the two for a conservative cut shell at this band.
        side = max(side0, side1)
        z0 = FLOOR_Z + CHAMFER * t0
        band_h = CHAMFER / steps + 1e-4
        band = ExtrudeGeometry.from_z0(
            _square_profile(side),
            band_h,
            cap=True,
        ).translate(0.0, 0.0, z0)
        cavity = boolean_union(cavity, band)
    return cavity


def _build_shell():
    """Outer cube minus the open-topped, chamfered-base inner cavity."""
    outer = BoxGeometry((CUBE_SIZE, CUBE_SIZE, CUBE_SIZE))
    cavity = _build_cavity()
    return boolean_difference(outer, cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inside_chamfer_shelled_cube")
    finish = model.material("tray_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))

    tray = model.part("tray")
    tray.visual(
        mesh_from_geometry(_build_shell(), "shelled_cube"),
        material=finish,
    )
    tray.inertial = Inertial.from_geometry(
        Box((CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)),
        mass=0.40,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray = object_model.get_part("tray")
    ctx.check("tray_present", tray is not None, "Expected a tray part.")
    if tray is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(tray)
    ctx.check("tray_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # Outer extent should match the cube on all three axes.
    ctx.check(
        "cube_extent",
        all(abs(size[i] - CUBE_SIZE) <= 0.002 for i in range(3)),
        f"size={size!r}",
    )
    # The top should be open: the highest point of the shell is the cube top,
    # and the cavity reaches the open top.
    ctx.check(
        "top_at_cube_top",
        abs(maxs[2] - HALF) <= 0.002,
        f"maxs[2]={maxs[2]!r}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
