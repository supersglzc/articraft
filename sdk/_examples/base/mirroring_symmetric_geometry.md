---
title: 'Mirroring Symmetric Geometry'
description: 'Build a left-right symmetric extruded plate by authoring only a half profile and mirroring its points across the centerline, then extruding the closed loop into a solid.'
tags:
  - sdk
  - base sdk
  - mirroring
  - symmetric
  - extrude
  - profile
  - mesh geometry
---
# Mirroring Symmetric Geometry

When a shape is left-right symmetric you only need to author one half of its 2D
profile. Build the half profile from the centerline outward, then mirror those
points across the symmetry axis to close the loop. Extruding the mirrored loop
gives a perfectly symmetric solid with half the hand-authored coordinates.

This is the native-SDK analogue of drawing a half outline and calling a 2D
`mirror` before `extrude`. Here we build a small symmetric notched keystone
plate: the right half is authored explicitly and the left half is the mirror.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    ExtrudeGeometry,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# Right-half outline of the plate, authored from the bottom centerline up.
# Points run along +X (away from the symmetry axis at x=0) and +Y.
# This is the only geometry we hand-author; the left half is the mirror.
HALF_PROFILE = [
    (0.000, 0.000),  # bottom centerline
    (0.060, 0.000),  # bottom-right corner
    (0.060, 0.040),  # right wall up to the shoulder
    (0.040, 0.080),  # angled shoulder inward
    (0.020, 0.080),  # top flat, right of the central notch
    (0.020, 0.060),  # down into the central notch
    (0.000, 0.060),  # notch floor on the centerline
]


def mirror_y(half_profile):
    """Mirror a half profile across the Y axis (x -> -x) into a closed loop.

    The right half is traversed bottom-to-top. We then append the mirrored
    points top-to-bottom (excluding the two on-axis endpoints, which are shared)
    so the combined loop is a single non-self-intersecting closed contour.
    """
    interior = half_profile[1:-1]
    mirrored = [(-x, y) for (x, y) in reversed(interior)]
    return list(half_profile) + mirrored


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="symmetric_keystone_plate")
    finish = model.material("plate_steel", rgba=(0.62, 0.64, 0.68, 1.0))

    profile = mirror_y(HALF_PROFILE)

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(profile, 0.012, cap=True),
            "keystone_plate",
        ),
        material=finish,
        name="plate_body",
    )
    plate.inertial = Inertial.from_geometry(
        Box((0.120, 0.080, 0.012)),
        mass=0.25,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()

    plate = object_model.get_part("plate")
    ctx.check("plate_present", plate is not None, "Expected a plate part.")
    if plate is None:
        return ctx.report()

    # The mirrored loop must be symmetric across x=0.
    loop = mirror_y(HALF_PROFILE)
    xs = [x for (x, _y) in loop]
    width = max(xs) - min(xs)
    ctx.check(
        "centered_on_axis",
        abs(max(xs) + min(xs)) < 1e-9,
        f"loop not centered on x=0: min={min(xs)} max={max(xs)}",
    )

    aabb = ctx.part_world_aabb(plate)
    ctx.check("plate_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()
    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check(
        "plate_width_matches_mirror",
        abs(size[0] - width) < 1e-3,
        f"width {size[0]} != mirrored profile width {width}",
    )
    ctx.check("plate_thickness", 0.010 <= size[2] <= 0.014, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
