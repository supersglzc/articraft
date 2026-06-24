---
title: 'The Classic OCC Bottle'
description: 'Base SDK reproduction of the famous OpenCascade bottle: a lens-shaped extruded body with a round neck, hollowed into a thin-walled open bottle by subtracting an inset cavity instead of a B-rep shell.'
tags:
  - sdk
  - base sdk
  - bottle
  - occ bottle
  - opencascade
  - shell
  - hollow body
  - extrude geometry
  - boolean difference
  - mesh geometry
---
# The Classic OCC Bottle

This base-SDK example reproduces the famous OpenCascade / CadQuery "bottle":
a flat lens-shaped body whose long sides bulge outward on a circular arc, an
extruded vertical wall, a short cylindrical neck on top, and finally a
hollowing operation that turns the solid into a thin-walled open bottle. It is
useful for queries such as `OCC bottle`, `OpenCascade bottle`, `bottle shell`,
`hollow body`, and `extrude + shell`.

The original CadQuery model drew half the bottle profile (a vertical line, a
three-point arc bulge, a mirrored half), extruded it, added the neck by
selecting the top face and extruding a circle, then called `.shell(0.3)` on the
top face to hollow the solid. The native SDK has no B-rep face-selection,
three-point-arc sketch, or `shell(...)` operation, so the bottle is built the
closest faithful way: the cross-section is sampled as an explicit 2D loop (the
two flat ends plus the two bulged arc sides), extruded with `ExtrudeGeometry`,
unioned with a `CylinderGeometry` neck, and then hollowed by subtracting an
inset inner solid (the same body and neck scaled inward by the wall thickness)
that is raised so it breaks through the top of the neck. This leaves a thin
wall, a closed bottom, and an open mouth, exactly like the shelled OCC result.

Approximation note: the SDK has no B-rep `shell(...)`, so the constant-thickness
hollowing is approximated by `boolean_difference` of an inset cavity solid. The
inner cavity is an inward-offset copy of the body, which gives a wall that is
very close to but not exactly the constant geometric offset OCC produces near
the bulged corners; the soft sketch fillets of the kernel example are also not
reproduced. The result is the same object: a lens-bodied, round-necked,
thin-walled bottle that is hollow inside and open at the top.

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    Box,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# Parameters (meters). The original OCC/CadQuery source was in millimeters
# (L, w, t) = (20, 6, 3) with a 30 mm body and a 3 mm-radius / 2 mm neck.
# Here the proportions are preserved at a realistic small-bottle scale.
L = 0.080  # overall body length (X), flat-end to flat-end
W = 0.024  # body width (Y) across the straight section
T = 0.012  # arc bulge: how far each long side swells beyond W/2
BODY_H = 0.120  # extruded body height (Z)
NECK_R = 0.012  # neck radius
NECK_H = 0.010  # neck height above the body
WALL = 0.0030  # shell wall thickness


def _body_profile(length: float, width: float, bulge: float, *, arc_segments: int = 24):
    """2D cross-section of the bottle in local XY.

    Two short flat ends at x = +-length/2 plus two long sides that bulge
    outward on a circular arc by ``bulge`` (the native stand-in for the OCC
    three-point arc). Returned as a closed counter-clockwise loop.
    """
    half_l = length / 2.0
    half_w = width / 2.0

    # Circle through the two end points (+-half_l, +-half_w) and the apex
    # (0, +-(half_w + bulge)) for the bulged side. Solve for the radius/center.
    # Sagitta relation: bulge = r - sqrt(r^2 - half_l^2).
    r = (half_l * half_l + bulge * bulge) / (2.0 * bulge)
    yc = (half_w + bulge) - r  # center y for the +Y arc (below the apex)
    theta = math.asin(half_l / r)  # half sweep angle

    # +Y arc: sweep from the +X end across the apex to the -X end.
    top = []
    for i in range(arc_segments + 1):
        a = theta - 2.0 * theta * (i / arc_segments)
        top.append((r * math.sin(a), yc + r * math.cos(a)))

    # -Y arc: mirror of the +Y arc, swept back the other way.
    bottom = []
    for i in range(arc_segments + 1):
        a = -theta + 2.0 * theta * (i / arc_segments)
        bottom.append((r * math.sin(a), -(yc + r * math.cos(a))))

    # CCW loop: +Y arc (end +X -> -X), then -Y arc (-X -> +X). Drop the
    # duplicated shared end points where the loops meet.
    loop = top + bottom[1:-1]
    return loop


def _solid_bottle(length, width, bulge, body_h, neck_r, neck_h):
    """Solid (un-hollowed) bottle: extruded lens body + cylindrical neck."""
    body = ExtrudeGeometry.from_z0(_body_profile(length, width, bulge), body_h, cap=True)
    neck = CylinderGeometry(neck_r, neck_h, radial_segments=48, closed=True)
    # Cylinder is centered on Z; lift it so it overlaps the top of the body.
    neck.translate(0.0, 0.0, body_h + neck_h / 2.0 - WALL)
    return boolean_union(body, neck)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="occ_bottle")
    glass = model.material("bottle_glass", rgba=(0.40, 0.62, 0.58, 1.0))

    outer = _solid_bottle(L, W, T, BODY_H, NECK_R, NECK_H)

    # Inner cavity: the same body/neck offset inward by WALL, raised so it
    # breaks through the top of the neck (open mouth) but stops short of the
    # bottom (closed base). This is the native stand-in for shell(top, WALL).
    inner_len = L - 2.0 * WALL
    inner_w = W - 2.0 * WALL
    inner_bulge = max(T - WALL, WALL)
    inner_body_h = BODY_H + NECK_H  # tall enough to punch out through the neck
    inner = ExtrudeGeometry.from_z0(
        _body_profile(inner_len, inner_w, inner_bulge),
        inner_body_h,
        cap=True,
    )
    inner.translate(0.0, 0.0, WALL)  # leave a WALL-thick closed bottom

    inner_neck = CylinderGeometry(NECK_R - WALL, NECK_H * 2.0, radial_segments=48, closed=True)
    inner_neck.translate(0.0, 0.0, BODY_H + NECK_H / 2.0)
    cavity = boolean_union(inner, inner_neck)

    bottle = boolean_difference(outer, cavity)

    part = model.part("bottle")
    part.visual(
        mesh_from_geometry(bottle, "occ_bottle"),
        material=glass,
        name="bottle_shell",
    )
    part.inertial = Inertial.from_geometry(
        Box((L, W + 2.0 * T, BODY_H + NECK_H)),
        mass=0.20,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    ctx.check("bottle_part_present", bottle is not None, "Expected a bottle part.")
    if bottle is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(bottle)
    ctx.check("bottle_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # Length and full bulged width.
    ctx.check("bottle_length", abs(size[0] - L) <= 0.003, f"size={size!r}")
    ctx.check("bottle_width", abs(size[1] - (W + 2.0 * T)) <= 0.004, f"size={size!r}")
    # Total height = body + neck.
    ctx.check(
        "bottle_height",
        abs(size[2] - (BODY_H + NECK_H)) <= 0.004,
        f"size={size!r}",
    )
    return ctx.report()


object_model = build_object_model()
```
