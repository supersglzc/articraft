---
title: 'Remote Enclosure'
description: 'Base SDK native reproduction of a compact remote enclosure: a shelled trapezoidal top with five countersunk button holes in a plus pattern and a mating cover that seats into the open bottom.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - enclosure
  - remote
  - remote enclosure
  - handheld case
  - housing
  - shelled top
  - countersunk holes
  - countersink
  - button holes
  - mating cover
  - trapezoid profile
  - boolean difference
  - boolean union
  - extrude geometry
  - cone geometry
  - cylinder geometry
  - prismatic articulation
  - motion limits
---
# Remote Enclosure

This base-SDK example is a native reproduction of the classic CadQuery remote
enclosure: a hollow, shelled top whose cross-section is a slightly flared
trapezoid, with five countersunk button holes arranged in a plus pattern on the
top face, plus a mating cover that nests into the open bottom of the shell. It is
useful for queries such as `remote enclosure`, `handheld remote case`,
`shelled housing`, `countersunk button holes`, `countersink`, `mating cover`,
`trapezoid enclosure`, `boolean_difference`, `ConeGeometry`, and
`ExtrudeGeometry`.

The teaching intent is parametric, boolean-driven enclosure construction with a
shelled body and countersunk through-holes. All key dimensions are top-level
parameters, the top and the cover are derived from the same trapezoidal
footprint, and the cover is sized so it nests up into the open bottom of the
shelled top. The functional mechanism is preserved as a working two-part
assembly: the shelled top is the root and the cover is a separate part on a
prismatic joint so it can drop out and re-seat into the shell.

The modeling patterns worth copying are:

- a trapezoidal `ExtrudeGeometry.from_z0(...)` solid, hollowed into a shell by a
  `boolean_difference(...)` with an inset trapezoid that punches through the
  open bottom.
- countersunk holes built as a `ConeGeometry` chamfer stacked over a through
  `CylinderGeometry`, both cut with `boolean_difference(...)`, in a five-point
  plus pattern.
- a mating cover derived from the same outer footprint with a thin nesting lip
  sized to the inner wall for a snug fit.
- a prismatic joint with `MotionLimits` so positive motion lowers the cover out
  of the shell.

```python
from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# --- Parameters (meters) ---------------------------------------------------
# A realistic handheld-remote footprint, scaled down from the original
# 2.2 x 1.5 x 0.5 unit block. The cross-section in plan (XY) is a trapezoid:
# the top edge (+Y) is the narrow end, the bottom edge (-Y) flares wider.
WIDTH = 0.066  # narrow (top) edge width, X
WIDE = 0.099  # wide (bottom) edge width, X = WIDTH * 1.5
LENGTH = 0.150  # Y span of the footprint
HEIGHT = 0.030  # Z height of the shelled top

WALL_THICKNESS = 0.0035  # shell wall / roof thickness
COVER_THICKNESS = 0.012  # cover plate thickness
LIP_HEIGHT = 0.006  # height of the cover lip that nests into the shell
LIP_CLEARANCE = 0.0005  # gap so the lip fits inside the shell wall

HOLE_RADIUS = 0.0090  # button through-hole radius
CSK_RADIUS = 0.0135  # countersink outer radius (HOLE_RADIUS * 1.5)
CSK_DEPTH = 0.0030  # depth of the conical countersink from the top face
HOLE_OFFSET = 0.030  # plus-pattern spacing from center

RADIAL_SEGMENTS = 32

# Five button centers on the top face: center plus four cardinal positions.
HOLE_CENTERS = [
    (0.0, 0.0),
    (-HOLE_OFFSET, 0.0),
    (HOLE_OFFSET, 0.0),
    (0.0, -HOLE_OFFSET),
    (0.0, HOLE_OFFSET),
]


def _trapezoid_profile(narrow: float, wide: float, length: float):
    """Centered trapezoid loop in XY: narrow edge at +Y, wide edge at -Y."""
    xn = narrow / 2.0
    xw = wide / 2.0
    y = length / 2.0
    # Counter-clockwise loop.
    return [
        (-xw, -y),
        (xw, -y),
        (xn, y),
        (-xn, y),
    ]


def _shelled_top():
    """Shell with a solid roof: cavity rises to HEIGHT - WALL_THICKNESS."""
    outer = ExtrudeGeometry.from_z0(
        _trapezoid_profile(WIDTH, WIDE, LENGTH), HEIGHT, cap=True
    )
    cavity_height = HEIGHT - WALL_THICKNESS
    inner = ExtrudeGeometry.from_z0(
        _trapezoid_profile(
            WIDTH - 2.0 * WALL_THICKNESS,
            WIDE - 2.0 * WALL_THICKNESS,
            LENGTH - 2.0 * WALL_THICKNESS,
        ),
        cavity_height + 0.002,  # over-tall through the open bottom
    ).translate(0.0, 0.0, -0.001)
    shell = boolean_difference(outer, inner)

    # Countersunk button holes through the roof, in the plus pattern.
    for cx, cy in HOLE_CENTERS:
        through = CylinderGeometry(
            HOLE_RADIUS,
            HEIGHT + 0.004,
            radial_segments=RADIAL_SEGMENTS,
        ).translate(cx, cy, HEIGHT / 2.0)
        shell = boolean_difference(shell, through)

        # Conical chamfer: wide at the top face, tapering down into the bore.
        csk = ConeGeometry(
            CSK_RADIUS,
            CSK_DEPTH,
            radial_segments=RADIAL_SEGMENTS,
        )
        # ConeGeometry tapers from base radius at -Z to apex at +Z; flip so the
        # wide mouth opens at the top face and place it at the roof.
        csk = csk.rotate_x(3.141592653589793)
        csk = csk.translate(cx, cy, HEIGHT - CSK_DEPTH / 2.0)
        shell = boolean_difference(shell, csk)

    return shell


def _build_cover():
    """Cover plate with a nesting lip, built so its top face sits at z = 0.

    The cover is authored in a local frame with its top at z = 0 and the plate
    body extending downward (-Z). The lip rises up (+Z) to nest into the open
    bottom of the shell.
    """
    plate = ExtrudeGeometry.from_z0(
        _trapezoid_profile(WIDTH, WIDE, LENGTH),
        COVER_THICKNESS,
    ).translate(0.0, 0.0, -COVER_THICKNESS)

    # Nesting lip sized to the inner wall, with a small clearance, rising up.
    lip_outer = ExtrudeGeometry.from_z0(
        _trapezoid_profile(
            WIDTH - 2.0 * WALL_THICKNESS - 2.0 * LIP_CLEARANCE,
            WIDE - 2.0 * WALL_THICKNESS - 2.0 * LIP_CLEARANCE,
            LENGTH - 2.0 * WALL_THICKNESS - 2.0 * LIP_CLEARANCE,
        ),
        LIP_HEIGHT,
    )
    lip_cavity = ExtrudeGeometry.from_z0(
        _trapezoid_profile(
            WIDTH - 4.0 * WALL_THICKNESS - 2.0 * LIP_CLEARANCE,
            WIDE - 4.0 * WALL_THICKNESS - 2.0 * LIP_CLEARANCE,
            LENGTH - 4.0 * WALL_THICKNESS - 2.0 * LIP_CLEARANCE,
        ),
        LIP_HEIGHT + 0.002,
    ).translate(0.0, 0.0, -0.001)
    lip = boolean_difference(lip_outer, lip_cavity)
    cover = boolean_union(plate, lip)
    return cover


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_enclosure")

    shell_gray = model.material("shell_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.20, 0.21, 0.23, 1.0))

    top = model.part("top")
    top.visual(mesh_from_geometry(_shelled_top(), "remote_shell"), material=shell_gray)
    top.inertial = Inertial.from_geometry(
        Box((WIDE, LENGTH, HEIGHT)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0)),
    )

    cover = model.part("cover")
    cover.visual(mesh_from_geometry(_build_cover(), "remote_cover"), material=cover_gray)
    cover.inertial = Inertial.from_geometry(
        Box((WIDE, LENGTH, COVER_THICKNESS + LIP_HEIGHT)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, -(COVER_THICKNESS) / 2.0)),
    )

    # The cover seats into the open bottom of the shell. Its child frame is placed
    # at the bottom opening of the shell (z = 0 plane of the shell), where the
    # cover's top face seats flush and its lip rises into the cavity at q = 0.
    # The prismatic joint runs along -Z so positive motion drops the cover out.
    model.articulation(
        "top_to_cover",
        ArticulationType.PRISMATIC,
        parent=top,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=0.0, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top = object_model.get_part("top")
    cover = object_model.get_part("cover")
    joint = object_model.get_articulation("top_to_cover")

    ctx.check("top_present", top is not None, "Expected a top part.")
    ctx.check("cover_present", cover is not None, "Expected a cover part.")
    ctx.check("joint_present", joint is not None, "Expected a top_to_cover joint.")
    if top is None or cover is None or joint is None:
        return ctx.report()

    top_aabb = ctx.part_world_aabb(top)
    if top_aabb is not None:
        mins, maxs = top_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "top_footprint",
            0.090 <= size[0] <= 0.105 and 0.140 <= size[1] <= 0.160,
            f"top size={size!r}",
        )
        ctx.check("top_height", 0.027 <= size[2] <= 0.033, f"top size={size!r}")

    # Seated: the cover nests into the shell bottom and overlaps in plan.
    # Removed: the cover drops clear of the shell along -Z.
    with ctx.pose({joint: 0.0}):
        ctx.expect_overlap(cover, top, axes="xy", min_overlap=0.05)
    with ctx.pose({joint: 0.045}):
        ctx.expect_gap(cover, top, axis="z", max_penetration=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
```
