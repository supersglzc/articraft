---
title: 'Crossed and Hyperbolic Gears'
description: 'Base SDK example comparing two skew-axis gear families in one scene: a crossed-helical gear pair and a hyperbolic gear pair, each with continuously spinning driver and follower gears on a shared mounting plate.'
tags:
  - sdk
  - base sdk
  - gear
  - gears
  - crossed helical gear
  - hyperbolic gear
  - skew axis gears
  - gear pair
  - gear train
  - mesh geometry
  - continuous articulation
  - mimic
  - CrossedGearPair
  - HyperbolicGearPair
---
# Crossed and Hyperbolic Gears

This base-SDK example compares two skew-axis gear families side by side: a
crossed-helical gear pair and a hyperbolic gear pair. It is useful for queries
such as `crossed helical gear`, `hyperbolic gear`, `skew axis gears`,
`gear pair`, `CrossedGearPair`, and `HyperbolicGearPair`.

The modeling patterns worth copying are:

- `CrossedGearPair` and `HyperbolicGearPair` produce each gear of a meshing pair
  already positioned relative to its mate; build the driver and follower as
  separate watertight meshes with `pair._build(build_gear2=False)` and
  `pair._build(build_gear1=False)` so each gear can become its own articulated
  part.
- a single fixed mounting plate carries both pairs as the root body.
- continuous rotation articulations spin each driver on its own shaft, and a
  `Mimic` couples each follower to its driver so the pair stays meshed.

The native gear classes produce an approximate involute / crossed-helical tooth
profile, not an exact mathematically-derived involute, helical, or hyperboloid
surface; they are intended as visually faithful gear stand-ins.

```python
from __future__ import annotations

from math import cos, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CrossedGearPair,
    HyperbolicGearPair,
    Inertial,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# Scale factor from the gear modules (mm-like units) into meters.
SCALE = 0.005

# Lateral offsets that place the two pairs on either side of the plate center.
CROSSED_X = -0.09
HYPERBOLIC_X = 0.09

# Pair definitions (kept close to the canonical skew-axis demo values).
CROSSED_SHAFT_ANGLE_DEG = 90.0
HYPERBOLIC_SHAFT_ANGLE_DEG = 60.0


def _rot_x(vec: tuple[float, float, float], angle_rad: float) -> tuple[float, float, float]:
    c, s = cos(angle_rad), sin(angle_rad)
    x, y, z = vec
    return (x, c * y - s * z, s * y + c * z)


def _scaled_mesh(geometry, name: str):
    geometry = geometry.copy()
    geometry.scale(SCALE)
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="crossed_and_hyperbolic_gears")
    plate_finish = model.material("plate_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    driver_finish = model.material("gear_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    follower_finish = model.material("gear_brass", rgba=(0.72, 0.58, 0.26, 1.0))

    # --- Root: fixed mounting plate -------------------------------------------------
    plate = model.part("base_plate")
    plate_size = (0.26, 0.20, 0.012)
    plate.visual(
        Box(plate_size),
        origin=Origin(xyz=(0.0, 0.0, -plate_size[2] / 2.0)),
        material=plate_finish,
        name="base_plate_shell",
    )
    plate.inertial = Inertial.from_geometry(Box(plate_size), mass=2.0)

    # --- Crossed-helical pair -------------------------------------------------------
    crossed = CrossedGearPair(
        module=1.0,
        gear1_teeth_number=20,
        gear2_teeth_number=20,
        gear1_width=4.0,
        gear2_width=4.0,
        shaft_angle=CROSSED_SHAFT_ANGLE_DEG,
        gear1_helix_angle=30.0,
    )
    crossed_driver_geom = crossed._build(build_gear2=False)
    crossed_follower_geom = crossed._build(build_gear1=False)

    crossed_origin = (CROSSED_X, 0.0, 0.0)
    crossed_w1 = crossed.gear1.width * SCALE
    # Driver shaft axis is +Z; pivot at the gear center.
    crossed_driver_pivot = (
        CROSSED_X,
        0.0,
        crossed_w1 / 2.0,
    )
    # Follower is rotated about X by the shaft angle and offset along +X.
    crossed_shaft = radians(CROSSED_SHAFT_ANGLE_DEG)
    crossed_follower_axis = _rot_x((0.0, 0.0, 1.0), crossed_shaft)
    crossed_center_dist = (crossed.gear1.r0 + crossed.gear2.r0) * SCALE
    crossed_follower_pivot = (
        CROSSED_X + crossed_center_dist,
        0.0,
        crossed_w1 / 2.0,
    )

    crossed_driver = model.part("crossed_driver")
    crossed_driver.visual(
        _scaled_mesh(crossed_driver_geom, "crossed_driver"),
        origin=Origin(xyz=crossed_origin),
        material=driver_finish,
        name="crossed_driver_gear",
    )
    crossed_follower = model.part("crossed_follower")
    crossed_follower.visual(
        _scaled_mesh(crossed_follower_geom, "crossed_follower"),
        origin=Origin(xyz=crossed_origin),
        material=follower_finish,
        name="crossed_follower_gear",
    )

    # --- Hyperbolic pair ------------------------------------------------------------
    hyperbolic = HyperbolicGearPair(
        module=1.0,
        gear1_teeth_number=20,
        width=4.0,
        shaft_angle=HYPERBOLIC_SHAFT_ANGLE_DEG,
    )
    hyper_driver_geom = hyperbolic._build(build_gear2=False)
    hyper_follower_geom = hyperbolic._build(build_gear1=False)

    hyper_origin = (HYPERBOLIC_X, 0.0, 0.0)
    hyper_w1 = hyperbolic.gear1.width * SCALE
    hyper_driver_pivot = (
        HYPERBOLIC_X,
        0.0,
        hyper_w1 / 2.0,
    )
    hyper_shaft = radians(HYPERBOLIC_SHAFT_ANGLE_DEG)
    hyper_follower_axis = _rot_x((0.0, 0.0, 1.0), hyper_shaft)
    hyper_center_dist = (hyperbolic.gear1.throat_r + hyperbolic.gear2.throat_r) * SCALE
    hyper_follower_pivot = (
        HYPERBOLIC_X + hyper_center_dist,
        0.0,
        hyper_w1 / 2.0,
    )

    hyper_driver = model.part("hyperbolic_driver")
    hyper_driver.visual(
        _scaled_mesh(hyper_driver_geom, "hyperbolic_driver"),
        origin=Origin(xyz=hyper_origin),
        material=driver_finish,
        name="hyperbolic_driver_gear",
    )
    hyper_follower = model.part("hyperbolic_follower")
    hyper_follower.visual(
        _scaled_mesh(hyper_follower_geom, "hyperbolic_follower"),
        origin=Origin(xyz=hyper_origin),
        material=follower_finish,
        name="hyperbolic_follower_gear",
    )

    # --- Articulations --------------------------------------------------------------
    spin_limits = MotionLimits(effort=2.0, velocity=8.0, lower=0.0, upper=0.0)

    model.articulation(
        "crossed_driver_spin",
        ArticulationType.CONTINUOUS,
        parent=plate,
        child=crossed_driver,
        origin=Origin(xyz=crossed_driver_pivot),
        axis=(0.0, 0.0, 1.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "crossed_follower_spin",
        ArticulationType.CONTINUOUS,
        parent=plate,
        child=crossed_follower,
        origin=Origin(xyz=crossed_follower_pivot),
        axis=crossed_follower_axis,
        motion_limits=spin_limits,
        mimic=Mimic(
            joint="crossed_driver_spin",
            multiplier=-crossed.gear1.z / crossed.gear2.z,
        ),
    )

    model.articulation(
        "hyperbolic_driver_spin",
        ArticulationType.CONTINUOUS,
        parent=plate,
        child=hyper_driver,
        origin=Origin(xyz=hyper_driver_pivot),
        axis=(0.0, 0.0, 1.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "hyperbolic_follower_spin",
        ArticulationType.CONTINUOUS,
        parent=plate,
        child=hyper_follower,
        origin=Origin(xyz=hyper_follower_pivot),
        axis=hyper_follower_axis,
        motion_limits=spin_limits,
        mimic=Mimic(
            joint="hyperbolic_driver_spin",
            multiplier=-hyperbolic.gear1.z / hyperbolic.gear2.z,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plate = object_model.get_part("base_plate")
    crossed_driver = object_model.get_part("crossed_driver")
    crossed_follower = object_model.get_part("crossed_follower")
    hyper_driver = object_model.get_part("hyperbolic_driver")
    hyper_follower = object_model.get_part("hyperbolic_follower")

    for name, part in [
        ("base_plate", plate),
        ("crossed_driver", crossed_driver),
        ("crossed_follower", crossed_follower),
        ("hyperbolic_driver", hyper_driver),
        ("hyperbolic_follower", hyper_follower),
    ]:
        ctx.check(f"{name}_present", part is not None, f"Expected part {name!r}.")

    if None in (plate, crossed_driver, crossed_follower, hyper_driver, hyper_follower):
        return ctx.report()

    # The crossed pair should sit on the -X side and the hyperbolic pair on +X.
    crossed_aabb = ctx.part_world_aabb(crossed_driver)
    hyper_aabb = ctx.part_world_aabb(hyper_driver)
    if crossed_aabb is not None and hyper_aabb is not None:
        crossed_cx = 0.5 * (crossed_aabb[0][0] + crossed_aabb[1][0])
        hyper_cx = 0.5 * (hyper_aabb[0][0] + hyper_aabb[1][0])
        ctx.check(
            "pairs_separated",
            crossed_cx < 0.0 < hyper_cx,
            f"crossed_cx={crossed_cx!r}, hyper_cx={hyper_cx!r}",
        )

    # The follower of each pair sits to +X of its driver (skew-axis mesh point).
    cf_aabb = ctx.part_world_aabb(crossed_follower)
    cd_aabb = ctx.part_world_aabb(crossed_driver)
    if cf_aabb is not None and cd_aabb is not None:
        cf_cx = 0.5 * (cf_aabb[0][0] + cf_aabb[1][0])
        cd_cx = 0.5 * (cd_aabb[0][0] + cd_aabb[1][0])
        ctx.check("crossed_follower_offset", cf_cx > cd_cx, f"cf={cf_cx!r}, cd={cd_cx!r}")

    crossed_spin = object_model.get_articulation("crossed_driver_spin")
    hyper_spin = object_model.get_articulation("hyperbolic_driver_spin")
    ctx.check("crossed_spin_present", crossed_spin is not None, "Expected crossed driver spin.")
    ctx.check("hyperbolic_spin_present", hyper_spin is not None, "Expected hyperbolic driver spin.")

    # Spinning a driver should move its gear (continuous rotation about its shaft).
    if crossed_spin is not None:
        with ctx.pose({crossed_spin: 0.0}):
            base_aabb = ctx.part_world_aabb(crossed_driver)
        with ctx.pose({crossed_spin: 0.6}):
            turned_aabb = ctx.part_world_aabb(crossed_driver)
        if base_aabb is not None and turned_aabb is not None:
            moved = any(
                abs(base_aabb[0][i] - turned_aabb[0][i]) > 1e-5
                or abs(base_aabb[1][i] - turned_aabb[1][i]) > 1e-5
                for i in range(3)
            )
            ctx.check("crossed_driver_spins", moved, "Driver gear did not move when spun.")

    return ctx.report()


object_model = build_object_model()
```
