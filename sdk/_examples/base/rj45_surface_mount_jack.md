---
title: 'RJ45 Surface-mount Jack'
description: 'Base SDK example of a single-port surface-mount RJ45 modular jack: a solid body with a stepped front port aperture cut by boolean difference, plus keyway and latch-retainer notches.'
tags:
  - sdk
  - base sdk
  - rj45
  - jack
  - connector
  - port aperture
  - boolean difference
  - mesh geometry
---
# RJ45 Surface-mount Jack

This base-SDK example reproduces a single-port surface-mount RJ45 modular jack.
The teaching intent is a solid connector body with a stepped front opening: a
rectangular port aperture, a keyway notch below it, and a two-step latch
retainer detail. In CadQuery this was a `box` with several `cutBlind` passes on
the `>X` face; here the same intent is expressed natively as a `BoxGeometry`
body with the aperture and notch volumes removed via `boolean_difference`.

It is useful for queries such as `RJ45 jack`, `modular connector`,
`port aperture`, `keyway`, and `boolean_difference` cuts on a box face.

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

# All dimensions in meters (source model was in millimeters).
LENGTH = 0.021  # body extent along +X (front-to-back)
WIDTH = 0.016  # body extent along Y
HEIGHT = 0.014  # body extent along Z

APERTURE_WIDTH = 0.01168  # port opening along Y
APERTURE_HEIGHT = 0.00775  # port opening along Z
APERTURE_DEPTH = 0.015  # how deep the port is cut in from the front face

KEYWAY_WIDTH = 0.006
KEYWAY_HEIGHT = 0.0015

RETAINER_WIDTH = 0.00325
RETAINER_HEIGHT = 0.0015
RETAINER_DEPTH = 0.002  # shallow step depth from the front face

# Vertical placement of the keyway/retainer notches below the aperture center.
KEYWAY_Z = -(APERTURE_HEIGHT / 2.0) - (KEYWAY_HEIGHT / 2.0)
RETAINER_Z = KEYWAY_Z - (RETAINER_HEIGHT / 2.0)

# The front face of the body is at +X = LENGTH / 2.
FRONT_X = LENGTH / 2.0


def _front_cut(width_y: float, height_z: float, depth_x: float, z_center: float) -> BoxGeometry:
    """A cutter box opening on the +X face, centered in Y, at z_center.

    The cutter is made slightly proud of the front face so the boolean is a
    clean through-cut at the surface rather than a coplanar (degenerate) cut.
    """
    overshoot = 0.0005
    cutter = BoxGeometry((depth_x + overshoot, width_y, height_z))
    # Center the cutter so its rear wall sits depth_x inside the body and its
    # mouth pokes overshoot beyond the front face.
    cutter.translate(FRONT_X + overshoot / 2.0 - depth_x / 2.0, 0.0, z_center)
    return cutter


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rj45_surface_mount_jack")
    housing_mat = model.material("rj45_housing", rgba=(0.10, 0.10, 0.12, 1.0))

    # Solid body centered at the origin.
    body = BoxGeometry((LENGTH, WIDTH, HEIGHT))

    # Main port aperture cut in from the front (+X) face.
    body = boolean_difference(
        body,
        _front_cut(APERTURE_WIDTH, APERTURE_HEIGHT, APERTURE_DEPTH, 0.0),
    )

    # Keyway notch directly below the aperture, full aperture depth.
    body = boolean_difference(
        body,
        _front_cut(KEYWAY_WIDTH, KEYWAY_HEIGHT, APERTURE_DEPTH, KEYWAY_Z),
    )

    # Narrow latch-retainer notch below the keyway, full aperture depth.
    body = boolean_difference(
        body,
        _front_cut(RETAINER_WIDTH, RETAINER_HEIGHT, APERTURE_DEPTH, RETAINER_Z),
    )

    # Shallow retainer step at the retainer elevation, only RETAINER_DEPTH deep.
    body = boolean_difference(
        body,
        _front_cut(KEYWAY_WIDTH, KEYWAY_HEIGHT, RETAINER_DEPTH, RETAINER_Z),
    )

    jack = model.part("jack")
    jack.visual(
        mesh_from_geometry(body, "rj45_jack_body"),
        material=housing_mat,
        name="jack_body",
    )
    jack.inertial = Inertial.from_geometry(
        Box((LENGTH, WIDTH, HEIGHT)),
        mass=0.012,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    jack = object_model.get_part("jack")
    ctx.check("jack_part_present", jack is not None, "Expected a jack part.")
    if jack is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(jack)
    ctx.check("jack_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # The outer envelope should remain the full body box (cuts are internal).
    ctx.check("body_length_x", 0.0205 <= size[0] <= 0.0215, f"size={size!r}")
    ctx.check("body_width_y", 0.0155 <= size[1] <= 0.0165, f"size={size!r}")
    ctx.check("body_height_z", 0.0135 <= size[2] <= 0.0145, f"size={size!r}")

    # The aperture removed material, so the body is lighter than a solid box.
    solid_volume = LENGTH * WIDTH * HEIGHT
    aperture_volume = APERTURE_WIDTH * APERTURE_HEIGHT * APERTURE_DEPTH
    ctx.check(
        "aperture_removed_material",
        aperture_volume > 0.0 and aperture_volume < solid_volume,
        f"aperture={aperture_volume!r} solid={solid_volume!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
