---
title: 'PiTray Clip'
description: 'Base SDK example of a sheet-metal style PiTray bracket assembly: a right-angle stainless bracket with PCB and clip mounting bosses, bolted to a black plastic DIN-rail clip, built from native mesh primitives, booleans, and a fixed joint.'
tags:
  - sdk
  - base sdk
  - pitray
  - bracket
  - angle bracket
  - din rail
  - rail clip
  - mounting boss
  - countersunk hole
  - fixed joint
  - assembly
  - mesh geometry
---
# PiTray Clip

This base-SDK example reproduces the `cq-electronics` **PiTray clip** assembly.
The original is two bolted members:

- a **right-angle (L-section) stainless bracket** with a horizontal PCB-mount
  leg and a vertical leg, raised **PCB-mount bosses** with tapped holes, raised
  **clip-mount bosses** with tapped holes, and **filleted outer corners**, and
- a **black plastic DIN-rail clip** bolted underneath, with a recessed
  **top-hat rail aperture** pocket and **countersunk** mounting holes.

The two members are rigidly bolted together, so the assembly is modeled as a
`bracket` root part plus a `din_clip` child joined by a `FIXED` articulation,
matching the original `cq.Assembly` with the clip located below the bracket.

The native build uses mesh primitives (`BoxGeometry`, `CylinderGeometry`,
`ConeGeometry`) plus `boolean_union` / `boolean_difference`. The L-section is the
union of a horizontal slab and a vertical slab. Mounting bosses are short
cylinders unioned to the legs; tapped/clearance holes and the rail pocket are
cut with `boolean_difference`. Countersunk holes follow the `cskHole`
semantics: a clearance shank topped by a conical sink. Filleted corners are
approximated by clipping the four long vertical edges of the vertical leg.

All dimensions are converted from the original millimeter model to meters.

```python
from __future__ import annotations

from math import radians, tan

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    CylinderGeometry,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_intersection,
    boolean_union,
    mesh_from_geometry,
)

# --- Bracket source dimensions (mm -> m) ---
LENGTH = 0.076  # X span of the bracket
WIDTH = 0.020  # Y depth of the bracket
HEIGHT = 0.015  # Z height of the vertical leg
THICKNESS = 0.0014  # sheet thickness of each leg
CORNER_FILLET = 0.003  # outer vertical-corner fillet (approximated as a clip)

# RPi mounting-hole spacing along the long axis.
RPI_HOLE_CENTERS_LONG = 0.058
# DIN-clip mount-hole spacing along the long axis.
BETWEEN_MOUNT_HOLES = 0.063

# PCB-mount bosses on the vertical leg (radius, stand-off length, tap hole).
PCB_BOSS_RADIUS = 0.0041 / 2.0
PCB_BOSS_LENGTH = 0.0055
PCB_BOSS_FROM_EDGE = 0.0037  # boss center down from the top edge of the leg
PCB_TAP_DIAMETER = 0.00215

# Clip-mount bosses on the horizontal leg (radius, stand-off length, tap hole).
CLIP_BOSS_RADIUS = 0.00795 / 2.0
CLIP_BOSS_LENGTH = 0.004
CLIP_TAP_DIAMETER = 0.0032

# --- DIN clip source dimensions (mm -> m) ---
DIN_LENGTH = 0.076
DIN_WIDTH = 0.020
DIN_HEIGHT = 0.008
TOP_HAT_WIDTH = 0.035
RAIL_APERTURE_DEPTH = 0.004
DIN_CORNER_CHAMFER = 0.003

# Countersunk M4 clearance hole geometry (cskHole semantics).
CSK_ANGLE_DEG = 90.0
CSK_CLEARANCE_DIAMETER = 0.0045
CSK_HEAD_DIAMETER = 0.0094

_SEGMENTS = 32
_EPS = 1.0e-4

# Bracket frame: the L sits with its corner near the origin. The vertical leg
# wall is at +Y; the horizontal leg floor is at the bottom (-Z). The bracket is
# centered on X so x in [-LENGTH/2, +LENGTH/2].
HALF_LENGTH = LENGTH / 2.0
TOP_Z = HEIGHT / 2.0
BOTTOM_Z = -HEIGHT / 2.0
BACK_Y = WIDTH / 2.0  # outer face of the vertical leg
FRONT_Y = -WIDTH / 2.0
# Inner faces of the two legs.
VERT_INNER_Y = BACK_Y - THICKNESS
HORIZ_TOP_Z = BOTTOM_Z + THICKNESS

HALF_RPI = RPI_HOLE_CENTERS_LONG / 2.0
HALF_BETWEEN = BETWEEN_MOUNT_HOLES / 2.0

# DIN clip frame helpers (its own centered box).
DIN_TOP_Z = DIN_HEIGHT / 2.0
DIN_BOTTOM_Z = -DIN_HEIGHT / 2.0
DIN_HALF_LENGTH = DIN_LENGTH / 2.0
RAIL_APERTURE_CENTER_X = -(DIN_HALF_LENGTH - 0.030)


def _l_section() -> BoxGeometry:
    """Right-angle L-section: a horizontal floor leg plus a vertical back leg."""
    horizontal = BoxGeometry((LENGTH, WIDTH, THICKNESS))
    horizontal.translate(0.0, 0.0, BOTTOM_Z + THICKNESS / 2.0)
    vertical = BoxGeometry((LENGTH, THICKNESS, HEIGHT))
    vertical.translate(0.0, BACK_Y - THICKNESS / 2.0, 0.0)
    return boolean_union(horizontal, vertical)


def _clip_vertical_corners(body: BoxGeometry) -> BoxGeometry:
    """Approximate filleting the long vertical edges of the back leg.

    Intersect the vertical-leg region with an X-narrowed slab so the two
    end corners of the back leg are clipped, mirroring the bracket's filleted
    outer corners on the tall leg.
    """
    clip_slab = BoxGeometry(
        (LENGTH - 2.0 * CORNER_FILLET, WIDTH * 2.0, HEIGHT * 2.0)
    )
    # Only narrow the upper (vertical-leg) band so the horizontal floor keeps
    # its full length; keep everything below the leg untouched by unioning the
    # floor band back in.
    floor_band = BoxGeometry((LENGTH, WIDTH * 2.0, THICKNESS))
    floor_band.translate(0.0, 0.0, BOTTOM_Z + THICKNESS / 2.0)
    clipped = boolean_intersection(body, clip_slab)
    return boolean_union(clipped, boolean_intersection(body, floor_band))


def _countersunk_cutter(cx: float, cy: float, top_z: float, height: float):
    """Through clearance bore plus a conical sink opening at the top face."""
    clr_r = CSK_CLEARANCE_DIAMETER / 2.0
    head_r = CSK_HEAD_DIAMETER / 2.0
    shank = CylinderGeometry(clr_r, height * 2.0, radial_segments=_SEGMENTS)
    shank.translate(cx, cy, 0.0)
    half_angle = radians(CSK_ANGLE_DEG) / 2.0
    cone_depth = (head_r - clr_r) / tan(half_angle)
    cone = ConeGeometry(head_r, cone_depth * 2.0, radial_segments=_SEGMENTS)
    cone.translate(cx, cy, top_z - cone_depth)
    return boolean_union(shank, cone)


def _build_bracket() -> BoxGeometry:
    body = _l_section()
    body = _clip_vertical_corners(body)

    # PCB-mount bosses: short cylinders standing proud of the vertical leg's
    # inner face, pointing toward -Y, with tapped through holes.
    pcb_boss_z = TOP_Z - PCB_BOSS_FROM_EDGE
    for sx in (-1.0, 1.0):
        cx = sx * HALF_RPI
        boss = CylinderGeometry(
            PCB_BOSS_RADIUS, PCB_BOSS_LENGTH, radial_segments=_SEGMENTS
        )
        # Cylinder axis is +Z; rotate it onto +Y, then push it proud of the
        # inner face of the vertical leg.
        boss.rotate_x(radians(90.0))
        boss.translate(cx, VERT_INNER_Y - PCB_BOSS_LENGTH / 2.0, pcb_boss_z)
        body = boolean_union(body, boss)
        # Tapped hole through the boss and the leg.
        bore = CylinderGeometry(
            PCB_TAP_DIAMETER / 2.0,
            (PCB_BOSS_LENGTH + THICKNESS) * 2.0,
            radial_segments=_SEGMENTS,
        )
        bore.rotate_x(radians(90.0))
        bore.translate(cx, BACK_Y, pcb_boss_z)
        body = boolean_difference(body, bore)

    # Clip-mount bosses: short cylinders standing proud below the horizontal
    # floor leg (toward -Z), with tapped through holes for the DIN-clip bolts.
    for sx in (-1.0, 1.0):
        cx = sx * HALF_BETWEEN
        boss = CylinderGeometry(
            CLIP_BOSS_RADIUS, CLIP_BOSS_LENGTH, radial_segments=_SEGMENTS
        )
        boss.translate(cx, 0.0, BOTTOM_Z - CLIP_BOSS_LENGTH / 2.0)
        body = boolean_union(body, boss)
        bore = CylinderGeometry(
            CLIP_TAP_DIAMETER / 2.0,
            (CLIP_BOSS_LENGTH + THICKNESS) * 2.0,
            radial_segments=_SEGMENTS,
        )
        bore.translate(cx, 0.0, BOTTOM_Z)
        body = boolean_difference(body, bore)

    return body


def _din_chamfered_body() -> BoxGeometry:
    slab_x = BoxGeometry((DIN_LENGTH, DIN_WIDTH - 2.0 * DIN_CORNER_CHAMFER, DIN_HEIGHT))
    slab_y = BoxGeometry((DIN_LENGTH - 2.0 * DIN_CORNER_CHAMFER, DIN_WIDTH, DIN_HEIGHT))
    return boolean_union(slab_x, slab_y)


def _build_din_clip() -> BoxGeometry:
    body = _din_chamfered_body()

    # Rail aperture pocket cut into the underside.
    pocket = BoxGeometry((TOP_HAT_WIDTH, DIN_WIDTH * 2.0, RAIL_APERTURE_DEPTH * 2.0))
    pocket.translate(RAIL_APERTURE_CENTER_X, 0.0, DIN_BOTTOM_Z)
    body = boolean_difference(body, pocket)

    # Two outer countersunk mount holes plus one centered on the rail aperture.
    body = boolean_difference(
        body, _countersunk_cutter(HALF_BETWEEN, 0.0, DIN_TOP_Z, DIN_HEIGHT)
    )
    body = boolean_difference(
        body, _countersunk_cutter(-HALF_BETWEEN, 0.0, DIN_TOP_Z, DIN_HEIGHT)
    )
    body = boolean_difference(
        body, _countersunk_cutter(RAIL_APERTURE_CENTER_X, 0.0, DIN_TOP_Z, DIN_HEIGHT)
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitray_clip")
    steel = model.material("bracket_stainless", rgba=(0.82, 0.83, 0.85, 1.0))
    plastic = model.material("din_clip_black", rgba=(0.05, 0.05, 0.05, 1.0))

    bracket = model.part("bracket")
    bracket.visual(
        mesh_from_geometry(_build_bracket(), "pitray_bracket"),
        material=steel,
        name="bracket_body",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((LENGTH, WIDTH, HEIGHT)),
        mass=0.06,
    )

    din_clip = model.part("din_clip")
    din_clip.visual(
        mesh_from_geometry(_build_din_clip(), "pitray_din_clip"),
        material=plastic,
        name="din_clip_body",
    )
    din_clip.inertial = Inertial.from_geometry(
        Box((DIN_LENGTH, DIN_WIDTH, DIN_HEIGHT)),
        mass=0.02,
    )

    # The clip is bolted flush under the bracket's horizontal floor leg.
    clip_elevation = BOTTOM_Z - CLIP_BOSS_LENGTH - DIN_TOP_Z
    model.articulation(
        "bracket_to_din_clip",
        ArticulationType.FIXED,
        parent=bracket,
        child=din_clip,
        origin=Origin(xyz=(0.0, 0.0, clip_elevation)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    din_clip = object_model.get_part("din_clip")
    ctx.check("bracket_present", bracket is not None, "Expected a bracket part.")
    ctx.check("din_clip_present", din_clip is not None, "Expected a din_clip part.")
    if bracket is None or din_clip is None:
        return ctx.report()

    b_aabb = ctx.part_world_aabb(bracket)
    c_aabb = ctx.part_world_aabb(din_clip)
    ctx.check("bracket_aabb", b_aabb is not None, "Expected a bracket AABB.")
    ctx.check("din_clip_aabb", c_aabb is not None, "Expected a din_clip AABB.")
    if b_aabb is None or c_aabb is None:
        return ctx.report()

    b_min, b_max = b_aabb
    b_size = tuple(float(b_max[i] - b_min[i]) for i in range(3))
    ctx.check("bracket_length", 0.073 <= b_size[0] <= 0.077, f"size={b_size!r}")
    ctx.check("bracket_width", 0.018 <= b_size[1] <= 0.022, f"size={b_size!r}")
    ctx.check("bracket_height", 0.013 <= b_size[2] <= 0.017, f"size={b_size!r}")

    c_min, c_max = c_aabb
    c_size = tuple(float(c_max[i] - c_min[i]) for i in range(3))
    ctx.check("din_clip_length", 0.073 <= c_size[0] <= 0.077, f"size={c_size!r}")
    ctx.check("din_clip_height", 0.0075 <= c_size[2] <= 0.0085, f"size={c_size!r}")

    # The clip must sit entirely below the bracket's floor leg.
    ctx.check(
        "clip_below_bracket",
        c_max[2] <= b_min[2] + 1.0e-4,
        f"clip_top={c_max[2]!r} bracket_bottom={b_min[2]!r}",
    )

    return ctx.report()


object_model = build_object_model()
```
</content>
</invoke>
