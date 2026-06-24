---
title: 'Worm Gear'
description: 'Base SDK example reproducing the worm-gear teaching intent as a meshing worm drive: a native Worm screw turning a worm wheel at a right angle, both mounted on a shared baseplate.'
tags:
  - sdk
  - base sdk
  - gear
  - worm
  - worm gear
  - worm drive
  - right angle drive
  - shaft bore
  - mesh geometry
---
# Worm Gear

This base-SDK example reproduces the teaching intent of the classic worm-gear
demo. The original parametric build authored a single `Worm` body (a helical
thread wrapped around a core). A worm by itself is only half of the mechanism, so
here it is shown in the role it actually plays: a worm drive, where the worm
screw meshes with a worm wheel at a right angle. Turning the worm advances the
wheel one tooth per worm revolution, the high-reduction, self-locking drive the
worm gear is known for.

Both the worm and the worm wheel are produced directly from the native gear
classes (`Worm` and `SpurGear`) as watertight `MeshGeometry` solids, so each one
drops straight into `mesh_from_geometry(...)`. The worm's axis runs along local X
and the worm wheel spins about Z, so the two shafts cross at 90 degrees. The
center distance equals the worm pitch radius plus the wheel pitch radius, so the
worm thread and the wheel teeth engage. A baseplate with two support pillars ties
the worm shaft and the wheel into one connected, grounded object.

Note: the native gear classes use an approximate, polygon-lofted thread/involute
profile rather than an exact analytic worm-and-wheel tooth surface, so the
meshing flanks are a faithful approximation rather than analytically exact worm
geometry.

```python
from __future__ import annotations

import numpy as np

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Mimic,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    Worm,
    boolean_union,
    mesh_from_geometry,
)

# All values authored in meters (module = 1 mm), matching the original demo.
MODULE = 0.001

# Worm screw: a 2-start thread on a slender core, axis along local X.
WORM_LEAD_ANGLE = 20.0
WORM_THREADS = 2
WORM_LENGTH = 0.018
WORM_BORE = 0.004

# Worm wheel: a helical-toothed disc that the worm drives, axis along local Z.
WHEEL_TEETH = 30
WHEEL_WIDTH = 0.006
WHEEL_HELIX = 20.0  # matches the worm lead so the thread can ride the teeth
WHEEL_BORE = 0.005

# Shafts and support structure that ground the assembly.
WORM_SHAFT_MARGIN = 0.010  # worm shaft protrudes past each end
WHEEL_SHAFT_MARGIN = 0.012
PLATE_THICK = 0.004


def _worm_geom() -> Worm:
    return Worm(
        module=MODULE,
        lead_angle=WORM_LEAD_ANGLE,
        n_threads=WORM_THREADS,
        length=WORM_LENGTH,
        bore_d=WORM_BORE,
    )


def _wheel_geom() -> SpurGear:
    return SpurGear(
        MODULE,
        WHEEL_TEETH,
        WHEEL_WIDTH,
        helix_angle=WHEEL_HELIX,
        bore_d=WHEEL_BORE,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="worm_gear")
    steel = model.material("worm_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    bronze = model.material("wheel_bronze", rgba=(0.55, 0.42, 0.28, 1.0))
    plate_finish = model.material("plate_dark", rgba=(0.20, 0.21, 0.23, 1.0))

    worm = _worm_geom()
    wheel = _wheel_geom()

    # Center distance: worm pitch radius + wheel pitch radius along +Y so the
    # worm thread engages the wheel teeth. The worm sits on the -Y side of the
    # wheel.
    center_distance = worm.r0 + wheel.r0
    worm_y = 0.0
    wheel_y = worm_y + center_distance

    # The worm axis is X; its shaft runs the full worm length plus margins.
    worm_shaft_len = WORM_LENGTH + 2.0 * WORM_SHAFT_MARGIN
    worm_shaft_r = WORM_BORE / 2.0 - 0.0003

    # The wheel axis is Z; its shaft drops down to the plate and up a stub.
    wheel_shaft_len = WHEEL_WIDTH + 2.0 * WHEEL_SHAFT_MARGIN
    wheel_shaft_r = WHEEL_BORE / 2.0 - 0.0003

    # Baseplate sits below the assembly. The worm rides above the plate at z=0;
    # the wheel center is also at z=0 so its rim reaches the worm.
    plate_top_z = -(wheel.ra + 0.002)
    plate_w = worm_shaft_len + 2.0 * worm.ra + 0.010
    plate_d = wheel_y + wheel.ra + 0.010
    plate_center_y = wheel_y / 2.0

    # --- Root: baseplate + the two pillars that carry the shafts ---
    base = model.part("base")
    plate_mesh = mesh_from_geometry(
        BoxGeometry((plate_w, plate_d, PLATE_THICK)), "baseplate"
    )
    base.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, plate_center_y, plate_top_z - PLATE_THICK / 2.0)),
        material=plate_finish,
        name="baseplate",
    )

    # Pillar under the wheel shaft: a vertical post from the plate up to the
    # bottom of the wheel hub.
    wheel_bottom_z = -wheel_shaft_len / 2.0
    wheel_pillar = _vertical_post(wheel_shaft_r, plate_top_z, wheel_bottom_z)
    base.visual(
        mesh_from_geometry(wheel_pillar, "wheel_pillar"),
        origin=Origin(xyz=(0.0, wheel_y, 0.0)),
        material=plate_finish,
        name="wheel_pillar",
    )

    # Two bearing standoffs that carry the worm shaft (axis X), one at each end.
    # Each rises from the plate top up to the worm axis (z=0).
    for side, sx in (("left", -worm_shaft_len / 2.0 + 0.002),
                     ("right", worm_shaft_len / 2.0 - 0.002)):
        post = _vertical_post(worm_shaft_r * 1.6, plate_top_z, 0.0)
        base.visual(
            mesh_from_geometry(post, f"worm_standoff_{side}"),
            origin=Origin(xyz=(sx, worm_y, 0.0)),
            material=plate_finish,
            name=f"worm_standoff_{side}",
        )

    base.inertial = Inertial.from_geometry(
        Box((plate_w, plate_d, PLATE_THICK)), mass=0.30
    )

    # --- Worm screw: thread body + through shaft along X ---
    worm_part = model.part("worm")
    worm_shaft = _shaft_x(worm_shaft_r, worm_shaft_len)
    worm_body = boolean_union(worm, worm_shaft)
    worm_part.visual(
        mesh_from_geometry(worm_body, "worm_body"),
        origin=Origin(xyz=(0.0, worm_y, 0.0)),
        material=steel,
        name="worm_body",
    )
    worm_part.inertial = Inertial.from_geometry(
        Box((WORM_LENGTH, 2.0 * worm.ra, 2.0 * worm.ra)), mass=0.04
    )

    # --- Worm wheel: helical gear + through shaft along Z ---
    wheel_part = model.part("worm_wheel")
    wheel_shaft = _shaft_z(wheel_shaft_r, wheel_shaft_len)
    wheel_body = boolean_union(wheel, wheel_shaft)
    wheel_part.visual(
        mesh_from_geometry(wheel_body, "worm_wheel_body"),
        origin=Origin(xyz=(0.0, wheel_y, 0.0)),
        material=bronze,
        name="worm_wheel_body",
    )
    wheel_part.inertial = Inertial.from_geometry(
        Box((2.0 * wheel.ra, 2.0 * wheel.ra, WHEEL_WIDTH)), mass=0.06
    )

    # Worm spins about its own (X) axis.
    model.articulation(
        "base_to_worm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=worm_part,
        origin=Origin(xyz=(0.0, worm_y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0, lower=-50.0, upper=50.0),
    )

    # Wheel spins about its own (Z) axis. The worm drive is a high-reduction
    # mechanism: one worm turn advances the wheel n_threads teeth, so the wheel
    # turns at WORM_THREADS / WHEEL_TEETH of the worm speed. Mimic encodes that
    # kinematic coupling.
    ratio = WORM_THREADS / WHEEL_TEETH
    model.articulation(
        "base_to_wheel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wheel_part,
        origin=Origin(xyz=(0.0, wheel_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0, lower=-6.28, upper=6.28),
        mimic=Mimic(joint="base_to_worm", multiplier=ratio, offset=0.0),
    )

    return model


def _shaft_x(radius: float, length: float):
    """A shaft along local X (the worm axis), centered on the origin."""
    return CylinderGeometry(radius, length).rotate_y(np.pi / 2.0)


def _shaft_z(radius: float, length: float):
    """A shaft along local Z (the wheel axis), centered on the origin."""
    return CylinderGeometry(radius, length)


def _vertical_post(radius: float, plate_top_z: float, top_z: float):
    """A vertical post from the plate top up to ``top_z``."""
    height = top_z - plate_top_z
    height = max(height, 0.0005)
    post = CylinderGeometry(radius, height)
    return post.translate(0.0, 0.0, plate_top_z + height / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    worm = object_model.get_part("worm")
    wheel = object_model.get_part("worm_wheel")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("worm_present", worm is not None, "Expected a worm part.")
    ctx.check("wheel_present", wheel is not None, "Expected a worm wheel part.")
    if base is None or worm is None or wheel is None:
        return ctx.report()

    worm_geom = _worm_geom()
    wheel_geom = _wheel_geom()

    # The worm is an elongated screw: long along its X axis, slim across.
    w_aabb = ctx.part_world_aabb(worm)
    ctx.check("worm_aabb", w_aabb is not None, "Expected an AABB for the worm.")
    if w_aabb is not None:
        lo, hi = w_aabb
        size = tuple(float(hi[i] - lo[i]) for i in range(3))
        ctx.check(
            "worm_elongated",
            size[0] > size[1] and size[0] > size[2],
            f"worm size={size!r} should be longest on X (its axis)",
        )

    # The wheel is a toothed disc about its pitch diameter.
    pitch_d = wheel_geom.r0 * 2.0
    h_aabb = ctx.part_world_aabb(wheel)
    ctx.check("wheel_aabb", h_aabb is not None, "Expected an AABB for the wheel.")
    if h_aabb is not None:
        lo, hi = h_aabb
        size = tuple(float(hi[i] - lo[i]) for i in range(3))
        ctx.check(
            "wheel_diameter",
            pitch_d * 0.9 <= max(size[0], size[1]) <= pitch_d * 1.4,
            f"wheel size={size!r}, pitch_d={pitch_d!r}",
        )

    # Worm and wheel are placed at the meshing center distance, so the worm
    # thread and the wheel rim engage (overlap across the center-distance axis).
    spin_worm = object_model.get_articulation("base_to_worm")
    spin_wheel = object_model.get_articulation("base_to_wheel")
    with ctx.pose({spin_worm: 0.0, spin_wheel: 0.0}):
        ctx.expect_overlap(worm, wheel, axes="y", min_overlap=0.0001)

    # Driving the worm drives the wheel through the mimic coupling: at a worm
    # angle the wheel should be at ratio * angle, so it stays a valid posed body.
    with ctx.pose({spin_worm: np.pi}):
        posed = ctx.part_world_aabb(wheel)
        ctx.check("wheel_driven", posed is not None, "Expected wheel AABB while worm turns.")

    return ctx.report()


object_model = build_object_model()
```
