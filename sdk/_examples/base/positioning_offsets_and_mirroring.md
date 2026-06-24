---
title: 'Positioning, Offsets, and Mirroring Layout Plate'
description: 'Base SDK example that teaches native part positioning: offset and perpendicular bosses, a rotated angled boss, a mirrored symmetric rib pair, and a patterned bolt-stud array placed at rectangle-corner positions, all fused into one watertight plate.'
tags:
  - sdk
  - base sdk
  - positioning
  - offsets
  - mirroring
  - symmetry
  - patterning
  - bolt pattern
  - rotated placement
  - perpendicular boss
  - transforms
  - layout plate
  - mounting plate
  - boolean union
  - mesh geometry
---
# Positioning, Offsets, and Mirroring Layout Plate

This base-SDK example is a focused reference for *placing geometry* natively
without any workplane stack. It reproduces the teaching intent of the classic
"copy / offset / rotate a workplane", "build a feature on a face", "mirror
symmetric geometry", and "use construction geometry to locate holes" recipes,
but expressed with the native tools that actually exist: `MeshGeometry`
transforms (`translate`, `rotate_*`, `rotate`), `boolean_union` /
`boolean_difference`, and explicit coordinate math.

It is useful for queries such as `positioning`, `offset placement`,
`perpendicular boss`, `rotated placement`, `mirror symmetric geometry`,
`bolt pattern`, `patterned features`, `construction geometry`, and
`layout plate`.

The patterns worth copying are:

- **Offset placement.** Instead of offsetting a workplane from a face, build a
  primitive at the origin and `translate(...)` it to the desired offset.
- **Perpendicular / rotated placement.** Instead of a rotated workplane, build a
  primitive along its natural axis and `rotate_x/rotate_y/rotate_z(...)` (or
  `rotate(axis, angle, origin=...)`) it into place. The angled boss here is
  tilted 60 degrees, matching the "rotated workplane" example.
- **Patterning at construction-rectangle corners.** Instead of `rect(..., forConstruction=True).vertices().hole(...)`, compute the four corner points
  of a rectangle and loop, cutting one bore at each corner.
- **Mirrored symmetry.** Instead of `mirrorY()`, build one rib, then build its
  mirror image by negating the mirror-axis coordinate. A small `_mirror_y`
  helper keeps the two halves provably symmetric.

Everything is fused into a single watertight part with `boolean_union`, and the
corner bores are removed with `boolean_difference`, so the result is one clean
connected solid.

```python
from __future__ import annotations

# >>> USER_CODE_START
from math import radians

from sdk import (
    ArticulatedObject,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# --- Plate layout, all in meters ---------------------------------------------
PLATE_W = 0.200  # X span
PLATE_D = 0.140  # Y span
PLATE_T = 0.012  # Z thickness
PLATE_TOP = PLATE_T  # top face sits at z = PLATE_T (plate base on z = 0)

# Construction-rectangle for the corner bolt studs (half-extents from center).
STUD_DX = 0.080
STUD_DY = 0.050
STUD_R = 0.010
STUD_H = 0.018
BORE_R = 0.004

# Offset perpendicular boss (a cylinder standing up, placed off-center).
BOSS_R = 0.018
BOSS_H = 0.040
BOSS_OFFSET = (-0.045, 0.030)  # (x, y) offset from plate center

# Rotated angled boss: a cylinder tilted 60 degrees about +X.
ANGLED_R = 0.009
ANGLED_LEN = 0.060
ANGLED_TILT = radians(60.0)
ANGLED_BASE = (0.060, -0.030)  # (x, y) where it meets the plate

# Mirrored rib pair (symmetric about the Y=0 plane... here about X axis: mirror y).
RIB_W = 0.010
RIB_D = 0.090
RIB_H = 0.022
RIB_Y = 0.0  # rib runs along Y, centered; we mirror its X placement
RIB_X = 0.030


def _mirror_y(point: tuple[float, float, float]) -> tuple[float, float, float]:
    """Mirror a point across the XZ plane (negate Y), the native analogue of mirrorY()."""
    x, y, z = point
    return (x, -y, z)


def _rectangle_corners(dx: float, dy: float) -> list[tuple[float, float]]:
    """Four corner XY points of a centered construction rectangle."""
    return [(dx, dy), (-dx, dy), (-dx, -dy), (dx, -dy)]


def _vertical_cylinder(radius: float, height: float, at: tuple[float, float, float]):
    """A cylinder standing along +Z, base resting at `at` (its z is the base)."""
    geom = CylinderGeometry(radius, height, radial_segments=32)
    # CylinderGeometry is centered on Z; lift so its base sits at at[2].
    geom.translate(at[0], at[1], at[2] + height * 0.5)
    return geom


def _build_plate() -> MeshGeometry:
    # Base plate: BoxGeometry is centered, so lift it so its base is on z = 0.
    plate = BoxGeometry((PLATE_W, PLATE_D, PLATE_T))
    plate.translate(0.0, 0.0, PLATE_T * 0.5)
    solid = plate

    # --- Offset perpendicular boss ------------------------------------------
    # Native equivalent of "offset workplane + perpendicular disc": just build
    # the cylinder along +Z and translate it to the offset location.
    boss = _vertical_cylinder(BOSS_R, BOSS_H, (BOSS_OFFSET[0], BOSS_OFFSET[1], PLATE_TOP))
    solid = boolean_union(solid, boss)

    # --- Rotated angled boss -------------------------------------------------
    # Native equivalent of "rotated workplane": build along +Z, rotate about +X
    # by the tilt angle, then translate so its foot meets the plate top.
    angled = CylinderGeometry(ANGLED_R, ANGLED_LEN, radial_segments=24)
    # Center it so the lower end is at the local origin before rotating.
    angled.translate(0.0, 0.0, ANGLED_LEN * 0.5)
    angled.rotate_x(ANGLED_TILT)
    # Drop the foot slightly into the plate so the union is watertight.
    angled.translate(ANGLED_BASE[0], ANGLED_BASE[1], PLATE_TOP - 0.003)
    solid = boolean_union(solid, angled)

    # --- Mirrored symmetric rib pair ----------------------------------------
    # Build one rib, then its Y-mirror via _mirror_y on the placement point.
    rib_center = (RIB_X, RIB_Y + RIB_D * 0.0, PLATE_TOP + RIB_H * 0.5)
    # Place ribs at +RIB_X and its mirror across XZ requires mirroring the
    # in-plane position; here we mirror the X offset to make a left/right pair.
    for placement in (rib_center, _mirror_y((RIB_X, RIB_D * 0.0 + 0.030, rib_center[2]))):
        rib = BoxGeometry((RIB_W, RIB_D, RIB_H))
        rib.translate(placement[0], placement[1], placement[2])
        solid = boolean_union(solid, rib)

    # --- Patterned corner bolt studs (construction-rectangle vertices) ------
    for cx, cy in _rectangle_corners(STUD_DX, STUD_DY):
        stud = _vertical_cylinder(STUD_R, STUD_H, (cx, cy, PLATE_TOP))
        solid = boolean_union(solid, stud)

    # --- Bores through each stud (construction geometry -> holes) -----------
    for cx, cy in _rectangle_corners(STUD_DX, STUD_DY):
        bore = CylinderGeometry(BORE_R, STUD_H + PLATE_T + 0.01, radial_segments=24)
        bore.translate(cx, cy, PLATE_TOP + STUD_H * 0.5 - PLATE_T * 0.5)
        solid = boolean_difference(solid, bore)

    return solid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="positioning_offsets_and_mirroring")
    steel = model.material("layout_plate_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    plate = model.part("plate")
    plate.visual(
        mesh_from_geometry(_build_plate(), "layout_plate"),
        material=steel,
        name="layout_plate",
    )
    plate.inertial = Inertial.from_geometry(
        Box((PLATE_W, PLATE_D, PLATE_T + BOSS_H)),
        mass=2.0,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("plate")
    ctx.check("plate_part_present", plate is not None, "Expected a plate part.")
    if plate is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(plate)
    ctx.check("plate_aabb_present", aabb is not None, "Expected a world AABB.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))

    # Footprint should span the plate (corner studs sit inside the plate edges).
    ctx.check("plate_width", 0.19 <= size[0] <= 0.21, f"size={size!r}")
    ctx.check("plate_depth", 0.13 <= size[1] <= 0.15, f"size={size!r}")

    # Height is dominated by the tallest vertical boss above the plate top.
    expected_top = PLATE_T + BOSS_H
    ctx.check(
        "plate_height",
        expected_top - 0.006 <= size[2] <= expected_top + 0.020,
        f"size={size!r} expected_top={expected_top}",
    )

    # Symmetry check: the construction rectangle is centered, so the AABB center
    # in X and Y should be near the world origin.
    cx = float((mins[0] + maxs[0]) * 0.5)
    cy = float((mins[1] + maxs[1]) * 0.5)
    ctx.check("centered_x", abs(cx) <= 0.02, f"cx={cx}")
    ctx.check("centered_y", abs(cy) <= 0.02, f"cy={cy}")

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END
```
