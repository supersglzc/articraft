---
title: 'Three-stage Telescoping Slide'
description: 'Base SDK three-stage drawer slide with nested outer, middle, and inner C-channel rails, end brackets, and stop tabs, driven by two stacked prismatic articulations along the slide axis.'
tags:
  - sdk
  - base sdk
  - telescoping slide
  - drawer slide
  - ball bearing slide
  - three stage
  - nested rails
  - c channel
  - rail
  - prismatic
  - prismatic articulation
  - linear slide
  - stop feature
  - end bracket
  - mesh geometry
  - extrude with holes
  - motion limits
---
# Three-stage Telescoping Slide

This base-SDK example is a reference for a three-stage telescoping drawer slide:
an outer rail bolted to a cabinet, a middle rail that slides out of it, and an
inner rail that slides out of the middle. Each rail is a C-shaped channel that
nests inside the next larger one, so the assembly extends along a single slide
axis. It is useful for queries such as `telescoping slide`, `drawer slide`,
`three stage slide`, `ball bearing slide`, `nested rails`, `C channel`,
`prismatic articulation`, `stop feature`, and `ExtrudeWithHolesGeometry`.

The modeling patterns worth copying are:

- a helper that builds a C-channel rail cross-section once and extrudes it along
  the slide axis, so the three rails share one construction path at different
  scales.
- nesting smaller channels inside larger channels so the visible slide reads as
  layered steel rather than one solid bar.
- two stacked `PRISMATIC` articulations (`outer_to_middle`, `middle_to_inner`)
  along `+X`, each with realistic `MotionLimits`, so positive motion extends the
  slide outward.
- explicit stop tabs and end brackets that read as the real travel limiters.
- telescoping tests: centering on the non-motion axes, retained insertion via
  `expect_overlap(...)` at rest and at full extension, and a pose check that the
  inner rail actually translates along the slide axis.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
)

# Slide axis is +X. Rails are C-channels: a back web plus top and bottom flanges
# that wrap toward +Y, leaving the channel open on +Y so the next rail nests in.
RAIL_LENGTH = 0.46
OUTER_TO_MIDDLE_MAX = 0.40
MIDDLE_TO_INNER_HOME = 0.02
MIDDLE_TO_INNER_MAX = 0.40

# Cross-section heights (Z) of each nested rail, shrinking inward.
OUTER_HEIGHT = 0.072
MIDDLE_HEIGHT = 0.052
INNER_HEIGHT = 0.034

WALL = 0.004
WEB_DEPTH = 0.018  # how far the back web sits along -Y


def _c_channel(length: float, height: float, depth: float, wall: float) -> BoxGeometry:
    """Build a C-channel rail centered at the local origin.

    The channel runs along X. Its back web sits at -Y and the top/bottom flanges
    wrap toward +Y, leaving the +Y face open so a smaller rail can nest inside.
    """
    half_h = height * 0.5
    # Back web spanning the full height at the -Y wall.
    web = BoxGeometry((length, wall, height)).translate(0.0, -depth + wall * 0.5, 0.0)
    # Top and bottom flanges spanning toward +Y.
    flange = BoxGeometry((length, depth, wall))
    top = flange.clone().translate(0.0, -depth * 0.5 + wall * 0.5, half_h - wall * 0.5)
    bottom = flange.clone().translate(0.0, -depth * 0.5 + wall * 0.5, -half_h + wall * 0.5)
    channel = boolean_union(boolean_union(web, top), bottom)
    return channel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    outer_steel = model.material("outer_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    middle_steel = model.material("middle_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    inner_steel = model.material("inner_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    machined_bracket = model.material("machined_bracket", rgba=(0.82, 0.84, 0.86, 1.0))
    stop_feature = model.material("stop_feature", rgba=(0.16, 0.18, 0.20, 1.0))

    # --- Outer rail: fixed to the cabinet, carries the assembly. ---
    outer = model.part("outer_rail")
    outer.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, WEB_DEPTH * 2.0, OUTER_HEIGHT)),
        mass=1.2,
    )
    outer.visual(
        mesh_from_geometry(
            _c_channel(RAIL_LENGTH, OUTER_HEIGHT, WEB_DEPTH, WALL),
            "outer_rail_body",
        ),
        material=outer_steel,
        name="outer_sleeve",
    )
    # End bracket at the back of the outer rail (cabinet mount face).
    outer.visual(
        Box((0.012, WEB_DEPTH * 1.4, OUTER_HEIGHT * 1.2)),
        origin=Origin(xyz=(-RAIL_LENGTH * 0.5 - 0.006, -WEB_DEPTH * 0.4, 0.0)),
        material=machined_bracket,
        name="outer_end_bracket",
    )

    # --- Middle rail: nests inside the outer channel, slides out along +X. ---
    middle = model.part("middle_rail")
    middle.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, WEB_DEPTH * 2.0, MIDDLE_HEIGHT)),
        mass=0.85,
    )
    middle.visual(
        mesh_from_geometry(
            _c_channel(RAIL_LENGTH, MIDDLE_HEIGHT, WEB_DEPTH * 0.78, WALL),
            "middle_rail_body",
        ),
        material=middle_steel,
        name="middle_member",
    )
    # Stop tab at the trailing (-X) end so the middle rail cannot pull free.
    middle.visual(
        Box((0.010, WEB_DEPTH * 0.9, MIDDLE_HEIGHT * 0.85)),
        origin=Origin(xyz=(-RAIL_LENGTH * 0.5 + 0.005, -WEB_DEPTH * 0.3, 0.0)),
        material=stop_feature,
        name="middle_stop_tab",
    )

    # --- Inner rail: nests inside the middle channel, carries the drawer. ---
    inner = model.part("inner_rail")
    inner.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, WEB_DEPTH * 1.6, INNER_HEIGHT)),
        mass=0.55,
    )
    inner.visual(
        mesh_from_geometry(
            _c_channel(RAIL_LENGTH, INNER_HEIGHT, WEB_DEPTH * 0.56, WALL),
            "inner_rail_body",
        ),
        material=inner_steel,
        name="inner_member",
    )
    # End bracket at the leading (+X) end where the drawer bolts on.
    inner.visual(
        Box((0.012, WEB_DEPTH * 1.0, INNER_HEIGHT * 1.3)),
        origin=Origin(xyz=(RAIL_LENGTH * 0.5 + 0.006, -WEB_DEPTH * 0.25, 0.0)),
        material=machined_bracket,
        name="inner_end_bracket",
    )
    # Stop tab at the trailing (-X) end.
    inner.visual(
        Box((0.010, WEB_DEPTH * 0.6, INNER_HEIGHT * 0.85)),
        origin=Origin(xyz=(-RAIL_LENGTH * 0.5 + 0.005, -WEB_DEPTH * 0.2, 0.0)),
        material=stop_feature,
        name="inner_stop_tab",
    )

    # Two stacked prismatic stages, both extending along +X.
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=OUTER_TO_MIDDLE_MAX, effort=80.0, velocity=0.60),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=MIDDLE_TO_INNER_MAX, effort=65.0, velocity=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_rail")
    middle = object_model.get_part("middle_rail")
    inner = object_model.get_part("inner_rail")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.check("outer_present", outer is not None, "Expected an outer rail.")
    ctx.check("middle_present", middle is not None, "Expected a middle rail.")
    ctx.check("inner_present", inner is not None, "Expected an inner rail.")

    # Nested rails intentionally share volume as one slides inside the next.
    ctx.allow_overlap(
        outer,
        middle,
        elem_a="outer_sleeve",
        elem_b="middle_member",
        reason="The middle rail is intentionally nested inside the outer channel.",
    )
    ctx.allow_overlap(
        middle,
        inner,
        elem_a="middle_member",
        elem_b="inner_member",
        reason="The inner rail is intentionally nested inside the middle channel.",
    )

    # Collapsed pose: each stage stays centered in its host and remains inserted.
    ctx.expect_within(
        middle,
        outer,
        axes="z",
        inner_elem="middle_member",
        outer_elem="outer_sleeve",
        margin=0.001,
        name="middle stays centered in the outer channel",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="middle_member",
        elem_b="outer_sleeve",
        min_overlap=0.10,
        name="collapsed middle remains inserted in the outer rail",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_member",
        elem_b="middle_member",
        min_overlap=0.10,
        name="collapsed inner remains inserted in the middle rail",
    )

    rest_pos = ctx.part_world_position(inner)

    # Fully extended pose: drive both prismatic stages to their upper limits.
    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_MAX, middle_to_inner: MIDDLE_TO_INNER_MAX}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_member",
            elem_b="outer_sleeve",
            min_overlap=0.02,
            name="extended middle retains insertion in the outer rail",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_member",
            elem_b="middle_member",
            min_overlap=0.02,
            name="extended inner retains insertion in the middle rail",
        )
        extended_pos = ctx.part_world_position(inner)

    ctx.check(
        "inner_rail_extends_along_x",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
```
