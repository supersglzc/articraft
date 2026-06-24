---
title: 'Mirroring From Faces'
description: 'Base SDK reproduction of the classic mirror-about-a-selected-face teaching snippet: a half-profile prism is reflected across its own +X face plane and immediately unioned into one symmetric solid.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - mirroring
  - mirror about face
  - reflection
  - symmetry
  - boolean union
  - extrude geometry
---
# Mirroring From Faces

This base-SDK example is a native reproduction of the classic CadQuery teaching
snippet that mirrors a solid about a *selected face* and unions the reflection
back onto the original in a single step. It is useful for queries such as
`mirror about a face`, `mirror and union`, `reflection`, `symmetry`, and
`boolean_union`.

The original snippet draws an open half-profile on the XY workplane
(`(0,0) -> (0,1) -> (1,1) -> (1,0.5)` closed back to the origin), extrudes it
into a prism, then calls `result.mirror(result.faces(">X"), union=True)`. The
`>X` face is the prism's right-hand face at its maximum X, so mirroring about
that plane reflects the half-profile to the far side and the `union=True` flag
fuses the reflection onto the source to yield one symmetric solid spanning twice
the original X width.

The teaching intent is therefore: take a half-shape, mirror it across one of its
own bounding faces, and immediately union the two halves into a single symmetric
body. The native reproduction keeps that exact read.

Approximation note: CadQuery's `mirror(faces(">X"), union=True)` selects a real
B-rep face, derives that face's plane, mirrors the solid across it, and performs
an exact B-rep fuse. The native SDK has no face-selection or B-rep mirror
operator, so this example reproduces the same result with native mesh tools:

- the half-profile is built with `ExtrudeGeometry` from the same polygon,
- the `>X` "selected face" is its maximum-X bounding plane, computed directly,
- the mirror is a native mesh reflection across that plane (`scale(-1, ...)` in
  X plus a translate so the mirror plane is fixed), with triangle winding
  corrected so the reflected solid stays outward-facing,
- the `union=True` fuse is reproduced with `boolean_union(...)`.

The visible result is the same symmetric solid, but it is mesh geometry rather
than an exact parametric mirror feature.

```python
from __future__ import annotations

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

# --- Parameters (meters) ---------------------------------------------------
# The original half-profile is drawn in unit-ish coordinates:
#   (0, 0) -> (0, 1) -> (1, 1) -> (1, 0.5) -> close
# extruded 1 unit thick. Reproduced here at a realistic small-bracket scale
# while preserving the proportions exactly.
SCALE = 0.06  # 1 source unit -> 0.06 m
HALF_WIDTH = 1.0 * SCALE  # source X extent of the half-profile
PROFILE_TALL = 1.0 * SCALE  # tallest profile edge (the +Y, -X corner)
PROFILE_SHORT = 0.5 * SCALE  # shortest profile edge (the +X corner)
THICKNESS = 1.0 * SCALE  # extrusion depth along Z


def _half_profile():
    """The source half-profile loop in local XY, returned counter-clockwise.

    Source path: (0,0) -> (0,1) -> (1,1) -> (1,0.5) -> close. That ordering is
    counter-clockwise already, which is the orientation the extruder expects.
    """
    return [
        (0.0, 0.0),
        (0.0, PROFILE_TALL),
        (HALF_WIDTH, PROFILE_TALL),
        (HALF_WIDTH, PROFILE_SHORT),
    ]


def _mirror_across_x_plane(geom: MeshGeometry, plane_x: float) -> MeshGeometry:
    """Reflect a mesh across the vertical plane X = ``plane_x``.

    This is the native stand-in for selecting the ">X" face and mirroring about
    it. Reflecting one axis flips triangle orientation, so winding is reversed to
    keep the reflected solid outward-facing for a clean boolean union.
    """
    mirrored = geom.clone()
    # x -> 2*plane_x - x  ==  scale x by -1 then translate by 2*plane_x.
    mirrored.scale(-1.0, 1.0, 1.0)
    mirrored.translate(2.0 * plane_x, 0.0, 0.0)
    # Reverse winding on every face so normals point back outward.
    mirrored.faces = [(c, b, a) for (a, b, c) in mirrored.faces]
    return mirrored


def _build_mirrored_solid() -> MeshGeometry:
    """Half-prism mirrored about its own +X face and unioned into one solid."""
    half = ExtrudeGeometry.from_z0(_half_profile(), THICKNESS, cap=True)

    # The ">X" selected face is the prism's maximum-X bounding plane.
    plane_x = max(v[0] for v in half.vertices)

    mirror = _mirror_across_x_plane(half, plane_x)

    # `union=True` in the source: fuse the reflection onto the original.
    return boolean_union(half, mirror)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mirrored_from_face_solid")
    finish = model.material("mirror_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    solid = _build_mirrored_solid()

    part = model.part("solid")
    part.visual(
        mesh_from_geometry(solid, "mirrored_solid"),
        material=finish,
        name="mirrored_solid",
    )
    # Full symmetric span is 2 * HALF_WIDTH in X.
    part.inertial = Inertial.from_geometry(
        Box((2.0 * HALF_WIDTH, PROFILE_TALL, THICKNESS)),
        mass=0.30,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    part = object_model.get_part("solid")
    ctx.check("solid_present", part is not None, "Expected the mirrored solid part.")
    if part is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(part)
    ctx.check("solid_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))

    # The mirror-and-union doubles the X extent to the full symmetric width.
    ctx.check(
        "full_width",
        abs(size[0] - 2.0 * HALF_WIDTH) <= 0.003,
        f"size={size!r}",
    )
    # Height (Y) and thickness (Z) are unchanged by the mirror.
    ctx.check("height", abs(size[1] - PROFILE_TALL) <= 0.003, f"size={size!r}")
    ctx.check("thickness", abs(size[2] - THICKNESS) <= 0.003, f"size={size!r}")

    # The result must be symmetric about the mirror plane: its X center should
    # sit on the selected +X face of the original half (x = HALF_WIDTH).
    x_center = 0.5 * (float(mins[0]) + float(maxs[0]))
    ctx.check(
        "symmetric_about_face",
        abs(x_center - HALF_WIDTH) <= 0.003,
        f"x_center={x_center!r}, expected={HALF_WIDTH!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
