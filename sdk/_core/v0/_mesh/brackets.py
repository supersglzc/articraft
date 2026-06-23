from __future__ import annotations

from typing import Sequence, Tuple

from .booleans import boolean_difference, boolean_intersection, boolean_union
from .native_build import cylinder_x, round_polygon_2d, rounded_box
from .primitives import (
    BoxGeometry,
    ExtrudeGeometry,
    MeshGeometry,
    _adopt_mesh_geometry,
    _mesh_geometry_shifted_to_z0,
)


def _rounded_rect_profile(
    width: float, depth: float, fillet: float, x_lo: float, x_hi: float
) -> list:
    """A 2D ``(x, z=depth)`` rectangle spanning x in [x_lo, x_hi], y in [-depth/2, depth/2],
    with its vertical (Z) corners filleted by ``fillet`` (matches ``edges("|Z").fillet``).
    """
    d = depth * 0.5
    rect = [(x_lo, -d), (x_hi, -d), (x_hi, d), (x_lo, d)]
    return round_polygon_2d(rect, fillet, kind="fillet") if fillet > 1e-6 else rect


def _box(size: Tuple[float, float, float]) -> MeshGeometry:
    """A centered box with outward-facing winding (positive signed volume).

    ``BoxGeometry`` is wound inward, which manifold3d ingests as an inverted solid and
    breaks boolean differences. Reversing the triangle order yields the same outward
    convention as the curved primitives so boolean ops behave correctly.
    """
    box = BoxGeometry(size)
    box.faces = [(a, c, b) for (a, b, c) in box.faces]
    return box


class ClevisBracketGeometry(MeshGeometry):
    """
    Build a U-shaped clevis bracket with a bottom base and a transverse pin bore.
    """

    def __init__(
        self,
        overall_size: Sequence[float],
        *,
        gap_width: float,
        bore_diameter: float,
        bore_center_z: float,
        base_thickness: float,
        corner_radius: float = 0.0,
        center: bool = True,
    ):
        super().__init__()
        width = float(overall_size[0])
        depth = float(overall_size[1])
        height = float(overall_size[2])
        gap_width = float(gap_width)
        bore_diameter = float(bore_diameter)
        bore_center_z = float(bore_center_z)
        base_thickness = float(base_thickness)
        corner_radius = max(0.0, float(corner_radius))

        if width <= 0.0 or depth <= 0.0 or height <= 0.0:
            raise ValueError("overall_size values must be positive")
        if gap_width <= 0.0 or gap_width >= width:
            raise ValueError("gap_width must be positive and less than overall_size[0]")
        cheek_thickness = 0.5 * (width - gap_width)
        if cheek_thickness <= 1e-6:
            raise ValueError("gap_width leaves no side wall material")
        if bore_diameter <= 0.0 or bore_diameter >= min(cheek_thickness * 2.0, depth, height):
            raise ValueError("bore_diameter is too large for the clevis envelope")
        if base_thickness <= 0.0 or base_thickness >= height:
            raise ValueError("base_thickness must be positive and less than overall_size[2]")
        bore_radius = bore_diameter * 0.5
        if bore_center_z - bore_radius <= base_thickness or bore_center_z + bore_radius >= height:
            raise ValueError(
                "bore_center_z must leave material above the base and below the top edge"
            )

        # construction: box -> cut full-depth slot -> fillet "|Z" -> cut bore. The slot spans the
        # full Y depth, so the Z-vertical cross-section is a solid base plus two cheeks. Build
        # each region as a profile extrusion with rounded vertical corners; this reproduces the
        # post-slot fillet on both the outer corners and the inner slot walls exactly.
        fillet = (
            min(corner_radius, cheek_thickness * 0.6, depth * 0.25, height * 0.25)
            if corner_radius > 0.0
            else 0.0
        )
        half_w = width * 0.5
        half_gap = gap_width * 0.5
        base_geom = ExtrudeGeometry(
            _rounded_rect_profile(width, depth, fillet, -half_w, half_w), base_thickness
        ).translate(0.0, 0.0, -height * 0.5 + base_thickness * 0.5)
        cheek_height = height - base_thickness
        cheek_z = -height * 0.5 + base_thickness + cheek_height * 0.5
        left_cheek = ExtrudeGeometry(
            _rounded_rect_profile(width, depth, fillet, -half_w, -half_gap), cheek_height
        ).translate(0.0, 0.0, cheek_z)
        right_cheek = ExtrudeGeometry(
            _rounded_rect_profile(width, depth, fillet, half_gap, half_w), cheek_height
        ).translate(0.0, 0.0, cheek_z)
        shape = boolean_union(boolean_union(base_geom, left_cheek), right_cheek)

        bore_z = -height * 0.5 + bore_center_z
        bore = cylinder_x(bore_radius, (width + 0.01) * 2.0).translate(0.0, 0.0, bore_z)
        shape = boolean_difference(shape, bore)

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)


class PivotForkGeometry(MeshGeometry):
    """
    Build an open-front pivot fork with a rear bridge and a transverse pin bore.
    """

    def __init__(
        self,
        overall_size: Sequence[float],
        *,
        gap_width: float,
        bore_diameter: float,
        bore_center_z: float,
        bridge_thickness: float,
        corner_radius: float = 0.0,
        center: bool = True,
    ):
        super().__init__()
        width = float(overall_size[0])
        depth = float(overall_size[1])
        height = float(overall_size[2])
        gap_width = float(gap_width)
        bore_diameter = float(bore_diameter)
        bore_center_z = float(bore_center_z)
        bridge_thickness = float(bridge_thickness)
        corner_radius = max(0.0, float(corner_radius))

        if width <= 0.0 or depth <= 0.0 or height <= 0.0:
            raise ValueError("overall_size values must be positive")
        if gap_width <= 0.0 or gap_width >= width:
            raise ValueError("gap_width must be positive and less than overall_size[0]")
        cheek_thickness = 0.5 * (width - gap_width)
        if cheek_thickness <= 1e-6:
            raise ValueError("gap_width leaves no side wall material")
        if bridge_thickness <= 0.0 or bridge_thickness >= depth:
            raise ValueError("bridge_thickness must be positive and less than overall_size[1]")
        if bore_diameter <= 0.0 or bore_diameter >= min(cheek_thickness * 2.0, depth, height):
            raise ValueError("bore_diameter is too large for the pivot fork envelope")
        bore_radius = bore_diameter * 0.5
        if bore_center_z - bore_radius <= 0.0 or bore_center_z + bore_radius >= height:
            raise ValueError("bore_center_z must keep the bore inside the fork cheeks")

        tine_depth = depth
        left_tine = _box((cheek_thickness, tine_depth, height)).translate(
            -(gap_width * 0.5 + cheek_thickness * 0.5), 0.0, 0.0
        )
        right_tine = _box((cheek_thickness, tine_depth, height)).translate(
            (gap_width * 0.5 + cheek_thickness * 0.5), 0.0, 0.0
        )
        rear_bridge = _box((width, bridge_thickness, height)).translate(
            0.0, -depth * 0.5 + bridge_thickness * 0.5, 0.0
        )
        shape = boolean_union(boolean_union(left_tine, right_tine), rear_bridge)
        if corner_radius > 0.0:
            # Vertical-edge fillet of the U-shape outline. Build the rounded outer envelope
            # and intersect to round only the exterior vertical edges, leaving the inner
            # gap untouched (matches box-union fillet behavior closely on the corners).
            fillet = min(
                corner_radius, cheek_thickness * 0.6, bridge_thickness * 0.6, height * 0.25
            )
            envelope = rounded_box(width, depth, height, fillet, kind="fillet")
            shape = boolean_intersection(shape, envelope)

        bore_z = -height * 0.5 + bore_center_z
        bore = cylinder_x(bore_radius, (width + 0.01) * 2.0).translate(0.0, 0.0, bore_z)
        shape = boolean_difference(shape, bore)

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)


class TrunnionYokeGeometry(MeshGeometry):
    """
    Build a trunnion support yoke with a bottom base and cheek-mounted trunnion bores.
    """

    def __init__(
        self,
        overall_size: Sequence[float],
        *,
        span_width: float,
        trunnion_diameter: float,
        trunnion_center_z: float,
        base_thickness: float,
        corner_radius: float = 0.0,
        center: bool = True,
    ):
        super().__init__()
        width = float(overall_size[0])
        depth = float(overall_size[1])
        height = float(overall_size[2])
        span_width = float(span_width)
        trunnion_diameter = float(trunnion_diameter)
        trunnion_center_z = float(trunnion_center_z)
        base_thickness = float(base_thickness)
        corner_radius = max(0.0, float(corner_radius))

        if width <= 0.0 or depth <= 0.0 or height <= 0.0:
            raise ValueError("overall_size values must be positive")
        if span_width <= 0.0 or span_width >= width:
            raise ValueError("span_width must be positive and less than overall_size[0]")
        cheek_thickness = 0.5 * (width - span_width)
        if cheek_thickness <= 1e-6:
            raise ValueError("span_width leaves no side wall material")
        if base_thickness <= 0.0 or base_thickness >= height:
            raise ValueError("base_thickness must be positive and less than overall_size[2]")
        if trunnion_diameter <= 0.0 or trunnion_diameter >= min(
            cheek_thickness * 2.0, depth, height
        ):
            raise ValueError("trunnion_diameter is too large for the yoke envelope")
        trunnion_radius = trunnion_diameter * 0.5
        if (
            trunnion_center_z - trunnion_radius <= base_thickness
            or trunnion_center_z + trunnion_radius >= height
        ):
            raise ValueError(
                "trunnion_center_z must leave material above the base and below the top edge"
            )

        base = _box((width, depth, base_thickness)).translate(
            0.0, 0.0, -height * 0.5 + base_thickness * 0.5
        )
        cheek_height = height - base_thickness
        cheek_z = -height * 0.5 + base_thickness + cheek_height * 0.5
        left_cheek = _box((cheek_thickness, depth, cheek_height)).translate(
            -(span_width * 0.5 + cheek_thickness * 0.5), 0.0, cheek_z
        )
        right_cheek = _box((cheek_thickness, depth, cheek_height)).translate(
            (span_width * 0.5 + cheek_thickness * 0.5), 0.0, cheek_z
        )
        boss_radius = max(trunnion_radius * 1.4, cheek_thickness * 0.55)
        boss_length = min(cheek_thickness * 0.75, depth * 0.35)
        boss_z = -height * 0.5 + trunnion_center_z
        # left_boss: extrude(boss_length) -> x in [0, boss_length], then translate to outer
        # cheek face at -(span/2 + cheek); spans inward toward the gap.
        left_boss = cylinder_x(
            boss_radius, boss_length, -(span_width * 0.5 + cheek_thickness) + boss_length * 0.5
        ).translate(0.0, 0.0, boss_z)
        # right_boss: extrude(-boss_length) -> x in [-boss_length, 0], then translate to outer
        # cheek face at +(span/2 + cheek).
        right_boss = cylinder_x(
            boss_radius, boss_length, (span_width * 0.5 + cheek_thickness) - boss_length * 0.5
        ).translate(0.0, 0.0, boss_z)
        shape = boolean_union(base, left_cheek)
        shape = boolean_union(shape, right_cheek)
        shape = boolean_union(shape, left_boss)
        shape = boolean_union(shape, right_boss)

        trunnion_bore = cylinder_x(
            trunnion_radius, (width + boss_length * 2.0 + 0.01) * 2.0
        ).translate(0.0, 0.0, boss_z)
        shape = boolean_difference(shape, trunnion_bore)

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)
