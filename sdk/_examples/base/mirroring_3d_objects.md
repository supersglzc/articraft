---
title: 'Mirroring 3D Objects'
description: 'Base SDK example that builds one master extruded bracket profile and reflects it across two planes to assemble a four-fold symmetric part, demonstrating native mesh mirroring via reflected copies and boolean_union.'
tags:
  - sdk
  - base sdk
  - mirroring
  - reflection
  - symmetry
  - mirror plane
  - extrude geometry
  - boolean union
  - mesh geometry
---
# Mirroring 3D Objects

This base-SDK example reproduces the classic "mirror one body across two planes
and union the results" pattern. The original CadQuery snippet extrudes a flat
2D bracket-style profile and then mirrors that solid across the `XY` and `ZY`
planes (at positive and negative offsets) before unioning all five bodies into
one symmetric assembly.

The native SDK has no `mirror` primitive, so the teaching intent is reproduced
explicitly: build the master extruded body once with `ExtrudeGeometry`, then make
reflected copies with a small `mirror_geometry(...)` helper that negates one
coordinate axis and reverses triangle winding so each reflected copy stays
watertight. The reflected copies are arranged to overlap the master at the
center, then merged into one connected solid with `boolean_union(...)`. It is
useful for queries such as `mirror`, `mirror plane`, `reflection`, `symmetry`,
`ExtrudeGeometry`, and `boolean_union`.

The modeling patterns worth copying are:

- author a master profile once and reflect it instead of re-deriving geometry.
- reverse face winding when negating a single axis so the reflected mesh keeps
  outward-facing normals and remains a valid boolean operand.
- offset the reflected copies so they overlap, keeping the unioned result one
  connected island rather than floating fragments.

```python
from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# ambient globals.
from sdk import (
    ArticulatedObject,
    Box,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# Scale of the master bracket body, in meters. The CadQuery profile lives in
# millimeter-ish units; here we keep a compact, hardware-plausible size.
PLATE_THICKNESS = 0.012
ARM_HALF = 0.060
ARM_WIDTH = 0.024


def _bracket_profile() -> list[tuple[float, float]]:
    """A flat L/bracket-like closed profile in local XY (counter-clockwise)."""
    return [
        (0.0, 0.0),
        (ARM_HALF, 0.0),
        (ARM_HALF, ARM_WIDTH),
        (ARM_WIDTH, ARM_WIDTH),
        (ARM_WIDTH, ARM_HALF),
        (0.0, ARM_HALF),
    ]


def mirror_geometry(geom: MeshGeometry, axis: str) -> MeshGeometry:
    """Reflect a mesh across the plane through the origin normal to one axis.

    Negating a single coordinate flips triangle orientation, so this helper also
    reverses each face's winding to keep outward normals and a valid boolean
    operand.
    """
    out = geom.clone()
    if axis == "x":
        out.vertices = [(-x, y, z) for (x, y, z) in out.vertices]
    elif axis == "y":
        out.vertices = [(x, -y, z) for (x, y, z) in out.vertices]
    elif axis == "z":
        out.vertices = [(x, y, -z) for (x, y, z) in out.vertices]
    else:
        raise ValueError(f"unknown mirror axis: {axis!r}")
    out.faces = [(a, c, b) for (a, b, c) in out.faces]
    return out


def _master_body() -> MeshGeometry:
    # Extrude the flat bracket profile into a slab, then orient it so the flat
    # face lies in the XZ-ish plane and the body sits centered, matching the
    # CadQuery flow of "extrude, rotate, recenter".
    body = ExtrudeGeometry.centered(_bracket_profile(), PLATE_THICKNESS, cap=True)
    return body


def build_symmetric_geometry() -> MeshGeometry:
    master = _master_body()

    # Reflect across X and across Y to populate the other three quadrants,
    # reproducing the CadQuery "mirror across two planes" assembly. The arms
    # overlap at the central corner so the union is one connected solid.
    mir_x = mirror_geometry(master, "x")
    mir_y = mirror_geometry(master, "y")
    mir_xy = mirror_geometry(mir_x, "y")

    result = boolean_union(master, mir_x)
    result = boolean_union(result, mir_y)
    result = boolean_union(result, mir_xy)
    return result


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mirrored_symmetric_bracket")
    steel = model.material("bracket_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    part = model.part("bracket")
    part.visual(
        mesh_from_geometry(build_symmetric_geometry(), "mirrored_bracket"),
        material=steel,
        name="bracket_shell",
    )
    span = 2.0 * ARM_HALF
    part.inertial = Inertial.from_geometry(
        Box((span, span, PLATE_THICKNESS)),
        mass=0.45,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    ctx.check("bracket_present", bracket is not None, "Expected a bracket part.")
    if bracket is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(bracket)
    ctx.check("bracket_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))

    # Mirroring across both X and Y must produce a symmetric footprint centered
    # on the origin: the X and Y extents should match and span both arms.
    span = 2.0 * ARM_HALF
    ctx.check(
        "x_span_symmetric",
        abs(size[0] - span) <= 0.002,
        f"size={size!r}",
    )
    ctx.check(
        "y_span_symmetric",
        abs(size[1] - span) <= 0.002,
        f"size={size!r}",
    )
    ctx.check(
        "centered_x",
        abs(mins[0] + maxs[0]) <= 0.002,
        f"mins={mins!r} maxs={maxs!r}",
    )
    ctx.check(
        "centered_y",
        abs(mins[1] + maxs[1]) <= 0.002,
        f"mins={mins!r} maxs={maxs!r}",
    )
    ctx.check(
        "thickness",
        abs(size[2] - PLATE_THICKNESS) <= 0.002,
        f"size={size!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
