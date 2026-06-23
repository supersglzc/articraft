from __future__ import annotations

from math import cos, pi, radians, sin
from typing import List, Optional, Sequence, Union

from .booleans import boolean_difference, boolean_union
from .common import _copy_manifold_provenance
from .native_build import cylinder_z, prism_yz, rounded_box
from .primitives import (
    BoxGeometry,
    ExtrudeGeometry,
    MeshGeometry,
    _adopt_mesh_geometry,
    _centered_axis_positions,
    _mesh_geometry_shifted_to_z0,
    _normalize_pitch_2d,
)
from .shape_helpers import _rounded_slot_profile
from .specs import (
    VentGrilleFrame,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
)


def _box_cut_z(width: float, height: float, depth: float) -> MeshGeometry:
    """A centered rectangular prism extruded along Z, for boolean cut/union tools.

    Uses ``ExtrudeGeometry`` rather than ``BoxGeometry``: the manifold boolean leaves
    phantom interior bodies / degenerate slivers when an axis-aligned ``BoxGeometry``
    (coplanar faces) is used as a cut or as a near-coincident union piece, whereas an
    extruded-rect prism resolves cleanly.
    """
    w, h = width * 0.5, height * 0.5
    rect = [(-w, -h), (w, -h), (w, h), (-w, h)]
    return ExtrudeGeometry(rect, depth)


class LouverPanelGeometry(MeshGeometry):
    """
    Build a rectangular panel with slot cutouts and fused louver fins.

    The panel lies in XY and is extruded along Z.
    """

    def __init__(
        self,
        panel_size: Sequence[float],
        thickness: float,
        *,
        frame: float = 0.008,
        slat_pitch: float = 0.024,
        slat_width: float = 0.010,
        slat_angle_deg: float = 32.0,
        corner_radius: float = 0.004,
        center: bool = True,
        fin_thickness: Optional[float] = None,
    ):
        super().__init__()
        panel_w = float(panel_size[0])
        panel_h = float(panel_size[1])
        t = float(thickness)
        frame = float(frame)
        pitch = float(slat_pitch)
        slat_w = float(slat_width)
        corner_radius = max(0.0, float(corner_radius))

        if panel_w <= 0 or panel_h <= 0 or t <= 0:
            raise ValueError("panel_size and thickness must be positive")
        if frame <= 0 or frame >= min(panel_w, panel_h) * 0.5:
            raise ValueError("frame must be > 0 and less than half of min(panel_size)")
        if pitch <= 0 or slat_w <= 0:
            raise ValueError("slat_pitch and slat_width must be positive")
        gap = pitch - slat_w
        if gap <= 1e-6:
            raise ValueError("slat_pitch must be greater than slat_width")

        inner_w = panel_w - 2.0 * frame
        inner_h = panel_h - 2.0 * frame
        if inner_w <= 0 or inner_h <= 0:
            raise ValueError("frame leaves no interior area")
        slot_h = min(gap * 0.9, inner_h * 0.8)
        slot_w = inner_w * 0.95
        if slot_h <= 0.0 or slot_w <= 0.0:
            raise ValueError("frame/slat settings leave no slot area")

        slat_rows: List[float] = []
        y = -inner_h * 0.5 + pitch * 0.5
        limit = inner_h * 0.5 - pitch * 0.5
        while y <= limit + 1e-9:
            slat_rows.append(y)
            y += pitch
        if not slat_rows:
            raise ValueError("No louver rows fit panel; increase panel height or reduce slat_pitch")

        fin_t = float(fin_thickness) if fin_thickness is not None else t * 0.35
        fin_t = max(1e-4, fin_t)
        if corner_radius > 0.0:
            shape: MeshGeometry = rounded_box(
                panel_w,
                panel_h,
                t,
                min(corner_radius, panel_w * 0.5, panel_h * 0.5),
                kind="fillet",
            )
        else:
            shape = BoxGeometry((panel_w, panel_h, t))

        slot_depth = t + max(0.002, t * 0.5)
        slat_span = min(inner_w + frame * 0.35, panel_w - 2.0e-4)
        slat_z = -t * 0.05

        for row_y in slat_rows:
            slot = _box_cut_z(slot_w, slot_h, slot_depth).translate(0.0, row_y, 0.0)
            shape = boolean_difference(shape, slot)

            slat = BoxGeometry((slat_span, slat_w, fin_t))
            slat = slat.rotate((1.0, 0.0, 0.0), radians(-float(slat_angle_deg)))
            slat = slat.translate(0.0, row_y + slot_h * 0.20, slat_z)
            shape = boolean_union(shape, slat)

        geom = shape
        if not center:
            geom.translate(0.0, 0.0, t * 0.5)
        self.vertices = [tuple(vertex) for vertex in geom.vertices]
        self.faces = [tuple(face) for face in geom.faces]
        _copy_manifold_provenance(geom, self)


class VentGrilleGeometry(MeshGeometry):
    """
    Build a rectangular vent grille with real openings and a shallow rear sleeve.

    The front flange is centered on local z=0 and the sleeve extends toward -Z.
    """

    def __init__(
        self,
        panel_size: Sequence[float],
        *,
        frame: float = 0.012,
        face_thickness: float = 0.004,
        duct_depth: float = 0.026,
        duct_wall: float = 0.003,
        slat_pitch: float = 0.018,
        slat_width: float = 0.009,
        slat_angle_deg: float = 35.0,
        slat_thickness: Optional[float] = None,
        corner_radius: float = 0.0,
        slats: Optional[VentGrilleSlats] = None,
        frame_profile: Optional[VentGrilleFrame] = None,
        mounts: Optional[VentGrilleMounts] = None,
        sleeve: Optional[VentGrilleSleeve] = None,
        center: bool = True,
    ):
        super().__init__()
        panel_w = float(panel_size[0])
        panel_h = float(panel_size[1])
        frame = float(frame)
        face_t = float(face_thickness)
        duct_depth = float(duct_depth)
        duct_wall = float(duct_wall)
        slat_pitch = float(slat_pitch)
        slat_w = float(slat_width)
        slat_t = float(slat_thickness) if slat_thickness is not None else max(0.001, face_t * 0.35)
        corner_radius = max(0.0, float(corner_radius))
        slats = slats or VentGrilleSlats()
        frame_profile = frame_profile or VentGrilleFrame()
        mounts = mounts or VentGrilleMounts()
        sleeve = sleeve or VentGrilleSleeve()

        if panel_w <= 0 or panel_h <= 0:
            raise ValueError("panel_size must be positive")
        if face_t <= 0 or duct_depth <= 0 or duct_wall <= 0:
            raise ValueError("face_thickness, duct_depth, and duct_wall must be positive")
        if frame <= 0 or frame >= min(panel_w, panel_h) * 0.5:
            raise ValueError("frame must be > 0 and less than half of min(panel_size)")
        if slat_pitch <= 0 or slat_w <= 0 or slat_t <= 0:
            raise ValueError("slat_pitch, slat_width, and slat_thickness must be positive")
        if slat_pitch <= slat_w:
            raise ValueError("slat_pitch must be greater than slat_width")

        slat_profile = str(slats.profile)
        if slat_profile not in {"flat", "airfoil", "boxed"}:
            raise ValueError("slats.profile must be one of flat, airfoil, or boxed")
        slat_direction = str(slats.direction)
        if slat_direction not in {"down", "up"}:
            raise ValueError("slats.direction must be one of down or up")
        slat_inset = float(slats.inset)
        divider_count = int(slats.divider_count)
        divider_width = float(slats.divider_width)
        if slat_inset < 0.0:
            raise ValueError("slats.inset must be non-negative")
        if divider_count < 0:
            raise ValueError("slats.divider_count must be non-negative")
        if divider_count > 0 and divider_width <= 0.0:
            raise ValueError("slats.divider_width must be positive when dividers are requested")

        frame_style = str(frame_profile.style)
        if frame_style not in {"flush", "beveled", "radiused"}:
            raise ValueError("frame_profile.style must be one of flush, beveled, or radiused")
        frame_depth = float(frame_profile.depth)
        if frame_depth < 0.0:
            raise ValueError("frame_profile.depth must be non-negative")

        mount_style = str(mounts.style)
        if mount_style not in {"none", "holes"}:
            raise ValueError("mounts.style must be one of none or holes")
        mount_inset = float(mounts.inset)
        if mount_inset < 0.0:
            raise ValueError("mounts.inset must be non-negative")
        mount_hole_diameter = (
            float(mounts.hole_diameter)
            if mounts.hole_diameter is not None
            else max(frame * 0.36, 0.0032)
        )
        if mount_style != "none" and mount_hole_diameter <= 0.0:
            raise ValueError("mounts.hole_diameter must be positive when mounts are enabled")

        sleeve_style = str(sleeve.style)
        if sleeve_style not in {"none", "short", "full"}:
            raise ValueError("sleeve.style must be one of none, short, or full")
        sleeve_depth = float(sleeve.depth) if sleeve.depth is not None else duct_depth
        sleeve_wall = float(sleeve.wall) if sleeve.wall is not None else duct_wall
        if sleeve_style == "short" and sleeve.depth is None:
            sleeve_depth = max(face_t * 1.8, duct_depth * 0.45)
        if sleeve_style == "none":
            sleeve_depth = 0.0
        if sleeve_style != "none" and sleeve_depth <= 0.0:
            raise ValueError("sleeve.depth must be positive when sleeve.style is not none")
        if sleeve_style != "none" and sleeve_wall <= 0.0:
            raise ValueError("sleeve.wall must be positive when sleeve.style is not none")

        opening_w = panel_w - 2.0 * frame
        opening_h = panel_h - 2.0 * frame
        if sleeve_style != "none" and (
            opening_w <= 2.0 * sleeve_wall or opening_h <= 2.0 * sleeve_wall
        ):
            raise ValueError("frame/duct_wall leave no open sleeve area")
        y = -opening_h * 0.5 + slat_pitch * 0.5
        limit = opening_h * 0.5 - slat_pitch * 0.5
        slat_rows: List[float] = []
        while y <= limit + 1e-9:
            slat_rows.append(y)
            y += slat_pitch
        if not slat_rows:
            raise ValueError("No slat rows fit panel; increase panel height or reduce slat_pitch")

        if divider_count > 0:
            usable_divider_span = opening_w - divider_width
            if usable_divider_span <= 0.0:
                raise ValueError("slats.divider_width leaves no grille opening")
            divider_pitch = usable_divider_span / float(divider_count + 1)
            if divider_pitch <= divider_width:
                raise ValueError("slats.divider_count/divider_width leave no usable openings")

        mount_positions = [
            (panel_w * 0.5 - mount_inset, panel_h * 0.5 - mount_inset),
            (-panel_w * 0.5 + mount_inset, panel_h * 0.5 - mount_inset),
            (panel_w * 0.5 - mount_inset, -panel_h * 0.5 + mount_inset),
            (-panel_w * 0.5 + mount_inset, -panel_h * 0.5 + mount_inset),
        ]
        if mount_style != "none":
            hole_radius = mount_hole_diameter * 0.5
            if hole_radius >= frame * 0.80:
                raise ValueError("mounts.hole_diameter is too large for the frame width")
            for hole_x, hole_y in mount_positions:
                if (
                    abs(hole_x) + hole_radius > panel_w * 0.5
                    or abs(hole_y) + hole_radius > panel_h * 0.5
                ):
                    raise ValueError("mounts.inset places holes outside the face")

        if corner_radius > 0.0:
            shape: MeshGeometry = rounded_box(
                panel_w,
                panel_h,
                face_t,
                min(corner_radius, panel_w * 0.5 - frame, panel_h * 0.5 - frame),
                kind="fillet",
            )
        else:
            shape = BoxGeometry((panel_w, panel_h, face_t))
        # cq `.edges(">Z").chamfer/fillet` are top-face (3D) edge rounds we don't support;
        # they are cosmetic frame_profile bevels, so skip them (beveled/radiused -> flush).

        shape = boolean_difference(
            shape,
            _box_cut_z(
                opening_w,
                opening_h,
                face_t + max(0.002, face_t * 0.5),
            ),
        )

        if sleeve_style != "none":
            duct_outer = _box_cut_z(opening_w, opening_h, sleeve_depth)
            duct_inner = _box_cut_z(
                opening_w - 2.0 * sleeve_wall,
                opening_h - 2.0 * sleeve_wall,
                sleeve_depth + face_t + 0.004,
            )
            duct_shell = boolean_difference(duct_outer, duct_inner).translate(
                0.0, 0.0, -face_t * 0.5 - sleeve_depth * 0.5 + min(face_t * 0.25, 0.001)
            )
            shape = boolean_union(shape, duct_shell)

        slat_embed = min(frame * 0.5, 0.002)
        slat_clear_w = opening_w - max(0.0, divider_count * divider_width)
        slat_chord = max(1.0e-4, slat_clear_w + 2.0 * slat_embed)
        slat_z = -face_t * 0.25 - slat_inset
        slat_angle = abs(float(slat_angle_deg)) * (-1.0 if slat_direction == "down" else 1.0)

        def _make_slat() -> MeshGeometry:
            # Rectangular slats are built as (y, z) prisms extruded along X (not
            # BoxGeometry) so they union with the frame without manifold erosion.
            if slat_profile == "flat":
                hw, ht = slat_w * 0.5, slat_t * 0.5
                rect = [(-hw, -ht), (hw, -ht), (hw, ht), (-hw, ht)]
                return prism_yz(rect, slat_chord)
            if slat_profile == "boxed":
                box_t = max(slat_t * 1.35, slat_w * 0.42)
                hw, ht = slat_w * 0.5, box_t * 0.5
                rect = [(-hw, -ht), (hw, -ht), (hw, ht), (-hw, ht)]
                return prism_yz(rect, slat_chord)

            # Airfoil section profiled in (y, z) and extruded along X (cq YZ workplane).
            section_points = [
                (-0.50 * slat_w, 0.00),
                (-0.22 * slat_w, 0.56 * slat_t),
                (0.08 * slat_w, 0.64 * slat_t),
                (0.44 * slat_w, 0.10 * slat_t),
                (0.34 * slat_w, -0.22 * slat_t),
                (-0.10 * slat_w, -0.38 * slat_t),
                (-0.44 * slat_w, -0.14 * slat_t),
            ]
            return prism_yz(section_points, slat_chord)

        for row_y in slat_rows:
            slat = _make_slat()
            slat = slat.rotate((1.0, 0.0, 0.0), radians(slat_angle))
            slat = slat.translate(0.0, row_y, slat_z)
            shape = boolean_union(shape, slat)

        if divider_count > 0:
            divider_positions = _centered_axis_positions(
                opening_w * 0.5 - divider_width * 0.5,
                (opening_w - divider_width) / float(divider_count + 1),
            )
            divider_positions = (
                divider_positions[1:-1]
                if len(divider_positions) > divider_count
                else divider_positions
            )
            divider_depth = max(face_t * 0.90, slat_t * 1.10)
            divider_span_y = opening_h + min(frame * 0.35, 0.003)
            for x_pos in divider_positions[:divider_count]:
                # Extruded-rect prism (not BoxGeometry) so the divider tips that poke
                # into the frame union cleanly instead of leaving degenerate slivers.
                divider = _box_cut_z(divider_width, divider_span_y, divider_depth)
                divider = divider.translate(x_pos, 0.0, -face_t * 0.18)
                shape = boolean_union(shape, divider)

        if mount_style != "none":
            hole_depth = face_t + max(sleeve_depth, 0.0) + 0.01
            for hole_x, hole_y in mount_positions:
                hole = cylinder_z(mount_hole_diameter * 0.5, hole_depth).translate(
                    hole_x, hole_y, -max(sleeve_depth, 0.0) * 0.5
                )
                shape = boolean_difference(shape, hole)

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        self.vertices = [tuple(vertex) for vertex in geom.vertices]
        self.faces = [tuple(face) for face in geom.faces]
        _copy_manifold_provenance(geom, self)


class PerforatedPanelGeometry(MeshGeometry):
    """
    Build a rectangular plate with a grid of round through-holes.

    The panel lies in XY and is extruded along Z.
    """

    def __init__(
        self,
        panel_size: Sequence[float],
        thickness: float,
        *,
        hole_diameter: float,
        pitch: Union[float, Sequence[float]],
        frame: float = 0.008,
        corner_radius: float = 0.0,
        stagger: bool = False,
        center: bool = True,
    ):
        super().__init__()
        panel_w = float(panel_size[0])
        panel_h = float(panel_size[1])
        thickness = float(thickness)
        hole_diameter = float(hole_diameter)
        frame = float(frame)
        corner_radius = max(0.0, float(corner_radius))
        pitch_x, pitch_y = _normalize_pitch_2d(pitch)

        if panel_w <= 0.0 or panel_h <= 0.0 or thickness <= 0.0:
            raise ValueError("panel_size and thickness must be positive")
        if hole_diameter <= 0.0:
            raise ValueError("hole_diameter must be positive")
        if frame < 0.0 or frame >= min(panel_w, panel_h) * 0.5:
            raise ValueError("frame must be >= 0 and less than half of min(panel_size)")
        if pitch_x <= hole_diameter or pitch_y <= hole_diameter:
            raise ValueError("pitch must be greater than hole_diameter on both axes")

        hole_radius = hole_diameter * 0.5
        x_limit = panel_w * 0.5 - frame - hole_radius
        y_limit = panel_h * 0.5 - frame - hole_radius
        if x_limit < -1e-9 or y_limit < -1e-9:
            raise ValueError("frame/hole_diameter leave no usable perforation area")

        y_positions = _centered_axis_positions(y_limit, pitch_y)
        if not y_positions:
            raise ValueError("No perforation rows fit panel; increase panel size or reduce pitch")

        point_rows: list[list[tuple[float, float]]] = []
        for row_index, y in enumerate(y_positions):
            x_offset = pitch_x * 0.5 if stagger and row_index % 2 == 1 else 0.0
            row_limit = x_limit - abs(x_offset)
            if row_limit < -1e-9:
                continue
            x_positions = _centered_axis_positions(row_limit, pitch_x)
            row_points = [(x + x_offset, y) for x in x_positions]
            if row_points:
                point_rows.append(row_points)
        if not point_rows:
            raise ValueError(
                "No perforation columns fit panel; increase panel size or reduce pitch"
            )

        if corner_radius > 0.0:
            shape: MeshGeometry = rounded_box(
                panel_w,
                panel_h,
                thickness,
                min(corner_radius, panel_w * 0.5 - 1e-4, panel_h * 0.5 - 1e-4),
                kind="fillet",
            )
        else:
            shape = BoxGeometry((panel_w, panel_h, thickness))

        cut_depth = thickness + max(0.002, thickness * 0.5)
        cut_shape: Optional[MeshGeometry] = None
        for row_points in point_rows:
            for x_pos, y_pos in row_points:
                hole = cylinder_z(hole_radius, 2.0 * cut_depth).translate(x_pos, y_pos, 0.0)
                cut_shape = hole if cut_shape is None else boolean_union(cut_shape, hole)
        if cut_shape is not None:
            shape = boolean_difference(shape, cut_shape)

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)


class SlotPatternPanelGeometry(MeshGeometry):
    """
    Build a rectangular plate with a grid of rounded through-slots.

    The panel lies in XY and is extruded along Z.
    """

    def __init__(
        self,
        panel_size: Sequence[float],
        thickness: float,
        *,
        slot_size: Sequence[float],
        pitch: Union[float, Sequence[float]],
        frame: float = 0.008,
        corner_radius: float = 0.0,
        slot_angle_deg: float = 0.0,
        stagger: bool = False,
        center: bool = True,
    ):
        super().__init__()
        panel_w = float(panel_size[0])
        panel_h = float(panel_size[1])
        thickness = float(thickness)
        slot_length = float(slot_size[0])
        slot_width = float(slot_size[1])
        frame = float(frame)
        corner_radius = max(0.0, float(corner_radius))
        slot_angle_deg = float(slot_angle_deg)
        pitch_x, pitch_y = _normalize_pitch_2d(pitch)

        if panel_w <= 0.0 or panel_h <= 0.0 or thickness <= 0.0:
            raise ValueError("panel_size and thickness must be positive")
        if slot_length <= 0.0 or slot_width <= 0.0:
            raise ValueError("slot_size values must be positive")
        if slot_length < slot_width:
            raise ValueError("slot_size[0] must be greater than or equal to slot_size[1]")
        if frame < 0.0 or frame >= min(panel_w, panel_h) * 0.5:
            raise ValueError("frame must be >= 0 and less than half of min(panel_size)")
        if abs(slot_angle_deg) >= 90.0:
            raise ValueError("abs(slot_angle_deg) must be < 90")

        slot_angle_rad = slot_angle_deg * pi / 180.0
        slot_half_x = 0.5 * (
            abs(slot_length * cos(slot_angle_rad)) + abs(slot_width * sin(slot_angle_rad))
        )
        slot_half_y = 0.5 * (
            abs(slot_length * sin(slot_angle_rad)) + abs(slot_width * cos(slot_angle_rad))
        )
        if pitch_x <= 2.0 * slot_half_x or pitch_y <= 2.0 * slot_half_y:
            raise ValueError("pitch must be greater than the rotated slot envelope on both axes")

        x_limit = panel_w * 0.5 - frame - slot_half_x
        y_limit = panel_h * 0.5 - frame - slot_half_y
        if x_limit < -1e-9 or y_limit < -1e-9:
            raise ValueError("frame/slot_size leave no usable slot area")

        y_positions = _centered_axis_positions(y_limit, pitch_y)
        if not y_positions:
            raise ValueError("No slot rows fit panel; increase panel size or reduce pitch")

        point_rows: list[list[tuple[float, float]]] = []
        for row_index, y in enumerate(y_positions):
            x_offset = pitch_x * 0.5 if stagger and row_index % 2 == 1 else 0.0
            row_limit = x_limit - abs(x_offset)
            if row_limit < -1e-9:
                continue
            x_positions = _centered_axis_positions(row_limit, pitch_x)
            row_points = [(x + x_offset, y) for x in x_positions]
            if row_points:
                point_rows.append(row_points)
        if not point_rows:
            raise ValueError("No slot columns fit panel; increase panel size or reduce pitch")

        cut_depth = thickness + max(0.002, thickness * 0.5)
        # Build the rounded slot (core + end caps) as a single extruded profile. A
        # box-core + cylinder-cap union as a cut tool leaves manifold phantom bodies;
        # one rounded-rect prism subtracts cleanly to a real through-slot.
        slot_profile = _rounded_slot_profile(slot_length, slot_width)

        def build_slot_cut(center_xy: tuple[float, float]) -> MeshGeometry:
            slot_cut = ExtrudeGeometry(slot_profile, 2.0 * cut_depth)
            slot_cut = slot_cut.rotate((0.0, 0.0, 1.0), radians(slot_angle_deg))
            return slot_cut.translate(center_xy[0], center_xy[1], 0.0)

        if corner_radius > 0.0:
            shape: MeshGeometry = rounded_box(
                panel_w,
                panel_h,
                thickness,
                min(corner_radius, panel_w * 0.5 - 1e-4, panel_h * 0.5 - 1e-4),
                kind="fillet",
            )
        else:
            shape = BoxGeometry((panel_w, panel_h, thickness))

        cut_shape: Optional[MeshGeometry] = None
        for row_points in point_rows:
            for point in row_points:
                slot_cut = build_slot_cut(point)
                cut_shape = slot_cut if cut_shape is None else boolean_union(cut_shape, slot_cut)
        if cut_shape is not None:
            shape = boolean_difference(shape, cut_shape)

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)
