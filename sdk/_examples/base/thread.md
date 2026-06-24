---
title: 'Thread'
description: 'Native-SDK threaded sleeve: a hollow cylindrical core with a helical external thread approximated by a swept ridge.'
tags:
  - sdk
  - base sdk
  - thread
  - helix
  - screw thread
  - threaded sleeve
  - sweep
  - mesh geometry
---
# Thread

The original CadQuery example constructs a screw thread analytically from a pair
of parametric helices joined by ruled (B-rep) surfaces, then unions that thread
form onto a hollow cylindrical core. There is no native B-rep ruled-surface or
analytic-helix-face operation in this SDK, so this example reproduces the *same
object* (a hollow threaded sleeve) with the closest native construction:

- The core is a hollow cylinder built by `boolean_difference` of an outer
  `CylinderGeometry` and a slightly taller inner bore cylinder.
- The external thread is a single helical ridge swept with
  `tube_from_spline_points(...)`: a circular profile follows a helix
  centerline that wraps the core for several turns, giving a continuous raised
  thread.
- The ridge and the core are joined with `boolean_union` so the result is one
  watertight solid, exactly as the source unions the thread onto the core.

Approximation note: the swept circular thread profile is a smooth bead rather
than the source's sharp triangular ruled-surface thread cross-section, and the
helix is sampled as a polyline spline rather than an exact analytic curve. The
visible read (a multi-turn external thread on a hollow tube) is faithful.

```python
from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    Box,
    CylinderGeometry,
    Inertial,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    tube_from_spline_points,
)

# Real-world scale: a coarse-pitch threaded sleeve, all dimensions in meters.
CORE_OUTER_R = 0.040
CORE_INNER_R = 0.030
HEIGHT = 0.044
PITCH = 0.011  # axial rise per full turn
TURNS = HEIGHT / PITCH
THREAD_R = 0.0035  # swept thread bead radius
HELIX_R = CORE_OUTER_R  # thread centerline sits on the core surface


def _helix_points(turns: float, helix_r: float, pitch: float, height: float):
    # Sample a helix centerline as a polyline for the swept thread ridge.
    samples = max(48, int(turns * 32))
    pts = []
    for i in range(samples + 1):
        t = i / samples
        ang = 2.0 * pi * turns * t
        z = height * t
        pts.append((helix_r * cos(ang), helix_r * sin(ang), z))
    return pts


def _build_thread_geometry():
    # Hollow cylindrical core: outer cylinder minus a through bore.
    outer = CylinderGeometry(CORE_OUTER_R, HEIGHT, radial_segments=48)
    outer.translate(0.0, 0.0, HEIGHT / 2.0)
    bore = CylinderGeometry(CORE_INNER_R, HEIGHT + 0.004, radial_segments=48)
    bore.translate(0.0, 0.0, HEIGHT / 2.0)
    core = boolean_difference(outer, bore)

    # External helical thread ridge swept along a multi-turn helix.
    helix = _helix_points(TURNS, HELIX_R, PITCH, HEIGHT)
    ridge = tube_from_spline_points(
        helix,
        radius=THREAD_R,
        radial_segments=12,
        samples_per_segment=1,
        cap_ends=True,
    )

    return boolean_union(core, ridge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="thread")
    steel = model.material("thread_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_geometry(_build_thread_geometry(), "threaded_sleeve"),
        material=steel,
        name="threaded_sleeve",
    )
    sleeve.inertial = Inertial.from_geometry(
        Box((2.0 * CORE_OUTER_R, 2.0 * CORE_OUTER_R, HEIGHT)),
        mass=0.25,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    ctx.check("sleeve_present", sleeve is not None, "Expected a sleeve part.")
    if sleeve is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(sleeve)
    ctx.check("sleeve_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    # Outer span includes the thread bead proud of the core surface.
    outer_span = 2.0 * (CORE_OUTER_R + THREAD_R)
    ctx.check(
        "sleeve_diameter",
        outer_span - 0.006 <= max(size[0], size[1]) <= outer_span + 0.006,
        f"size={size!r}",
    )
    ctx.check("sleeve_height", HEIGHT - 0.003 <= size[2] <= HEIGHT + 0.006, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
```
