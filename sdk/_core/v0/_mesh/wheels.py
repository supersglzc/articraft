from __future__ import annotations

from math import cos, pi, radians, sin
from typing import List, Optional, Sequence, Tuple

from .booleans import boolean_difference, boolean_union
from .common import _clamp
from .native_build import annulus_x, cylinder_x, prism_yz
from .primitives import (
    BoxGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    _adopt_mesh_geometry,
    _mesh_geometry_shifted_to_axis0,
)
from .shape_helpers import _centered_pattern_positions, _rounded_slot_profile
from .specs import (
    BoltPattern,
    TireCarcass,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelFlange,
    WheelHub,
    WheelRim,
    WheelSpokes,
)


def _box_solid(sx: float, sy: float, sz: float) -> MeshGeometry:
    """Outward-wound rectangular prism (BoxGeometry is inward-wound; bad as a boolean tool)."""
    hx, hy = sx * 0.5, sy * 0.5
    return ExtrudeGeometry([(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)], sz)


def _revolve_x(profile_axial_radial: List[Tuple[float, float]], segments: int = 96) -> MeshGeometry:
    """Revolve a ``(axial, radial)`` profile 360 deg about local X (cq revolve about (1,0,0)).

    LatheGeometry revolves a ``(radius, height)`` profile about Z, so we feed
    ``(radial, axial)`` and rotate the result onto X.
    """
    lathe = LatheGeometry([(r, a) for (a, r) in profile_axial_radial], segments=segments)
    return lathe.rotate_y(pi / 2.0)


def _circle_pts_x(x: float, radius: float, segments: int = 48) -> list[tuple[float, float, float]]:
    """A circle of given radius in the plane X=x (for LoftGeometry sections along X)."""
    return [
        (x, radius * cos(2.0 * pi * k / segments), radius * sin(2.0 * pi * k / segments))
        for k in range(segments)
    ]


class WheelGeometry(MeshGeometry):
    """
    Build a wheel/rim/hub visual aligned to local X.
    """

    def __init__(
        self,
        radius: float,
        width: float,
        *,
        rim: Optional[WheelRim] = None,
        hub: Optional[WheelHub] = None,
        face: Optional[WheelFace] = None,
        spokes: Optional[WheelSpokes] = None,
        bore: Optional[WheelBore] = None,
        flange: Optional[WheelFlange] = None,
        center: bool = True,
    ):
        super().__init__()
        radius = float(radius)
        width = float(width)
        if radius <= 0.0 or width <= 0.0:
            raise ValueError("radius and width must be positive")

        rim = rim or WheelRim()
        hub = hub or WheelHub(radius=radius * 0.18, width=width * 0.55)
        face = face or WheelFace()
        spokes = spokes or WheelSpokes(style="disc")
        bore = bore or WheelBore(style="round", diameter=max(radius * 0.18, 0.004))

        rim_outer = float(rim.outer_radius) if rim.outer_radius is not None else radius
        rim_inner = float(rim.inner_radius) if rim.inner_radius is not None else radius * 0.68
        if rim_inner <= 0.0 or rim_inner >= rim_outer:
            raise ValueError("WheelRim.inner_radius must be positive and less than outer_radius")
        if hub.radius <= 0.0 or hub.width <= 0.0 or hub.radius >= rim_inner:
            raise ValueError("WheelHub dimensions must be positive and fit inside the rim")
        if bore.diameter <= 0.0 or bore.diameter >= hub.radius * 2.0:
            raise ValueError("WheelBore.diameter must fit inside the hub")

        shape: MeshGeometry = annulus_x(rim_inner, rim_outer, width)
        if rim.flange_height > 1e-6 and rim.flange_thickness > 1e-6:
            flange_inner = max(rim_outer - rim.flange_height, rim_inner + 1.0e-4)
            lip_dx = width * 0.5 - rim.flange_thickness * 0.5
            for sign in (1.0, -1.0):
                lip = annulus_x(flange_inner, rim_outer, rim.flange_thickness, sign * lip_dx)
                shape = boolean_union(shape, lip)
        if rim.bead_seat_depth > 1e-6:
            seat = annulus_x(
                max(rim_outer - rim.bead_seat_depth, rim_inner + 1.0e-4),
                rim_outer,
                max(width * 0.22, 0.004),
            )
            shape = boolean_difference(shape, seat)

        shape = boolean_union(shape, cylinder_x(hub.radius, hub.width))
        if hub.cap_style == "domed":
            levels = [
                (hub.radius, 0.0),
                (hub.radius * 0.82, hub.width * 0.12),
                (hub.radius * 0.30, hub.width * 0.28),
            ]
            base_x = hub.width * 0.5
            profiles = [_circle_pts_x(base_x + z, r) for r, z in levels]
            shape = boolean_union(shape, LoftGeometry(profiles))
        elif hub.cap_style == "protruding":
            cap_len = max(width * 0.12, 0.003)
            shape = boolean_union(
                shape, cylinder_x(hub.radius * 0.82, cap_len, hub.width * 0.5 + cap_len * 0.5)
            )
        elif hub.cap_style == "recessed":
            poc_len = max(width * 0.10, 0.002)
            shape = boolean_difference(
                shape, cylinder_x(hub.radius * 0.55, poc_len, hub.width * 0.5 - poc_len * 0.5)
            )

        disc_thickness = max(spokes.thickness, width * 0.08, 0.002)
        face_outer_radius = min(
            rim_outer - max(rim.flange_height * 0.25, radius * 0.02),
            rim_inner + (rim_outer - rim_inner) * 0.28 + face.window_depth,
        )
        face_outer_radius = max(face_outer_radius, hub.radius * 1.6)
        front_inset = _clamp(
            face.front_inset + max(face.dish_depth, 0.0), 0.0, width * 0.5 - disc_thickness * 0.5
        )
        rear_inset = _clamp(face.rear_inset, 0.0, width * 0.5 - disc_thickness * 0.5)
        disc_inner_radius = max(hub.radius * 0.78, hub.radius - max(disc_thickness * 0.35, 0.0012))
        front_disc = annulus_x(
            disc_inner_radius,
            face_outer_radius,
            disc_thickness,
            width * 0.5 - front_inset - disc_thickness * 0.5,
        )
        rear_disc = annulus_x(
            disc_inner_radius,
            face_outer_radius,
            disc_thickness,
            -(width * 0.5 - rear_inset - disc_thickness * 0.5),
        )
        shape = boolean_union(boolean_union(shape, front_disc), rear_disc)

        spoke_style = spokes.style
        if spoke_style not in {"none", "disc"}:
            spoke_count = spokes.count or (6 if spoke_style in {"straight", "solid_slots"} else 5)
            if spoke_count < 2:
                raise ValueError("WheelSpokes.count must be at least 2")
            window_width = max(
                spokes.window_radius
                if spokes.window_radius > 0.0
                else (rim_outer - hub.radius) * 0.09,
                0.003,
            )
            slot_length = max((face_outer_radius - hub.radius) * 0.75, 0.010)
            cut_width = width + 0.02
            cutters: list[object] = []

            def radial_slot(
                angle_deg: float, center_radius: float, length: float, width_local: float
            ):
                profile = _rounded_slot_profile(length, width_local)
                pts = [(point[1], point[0] + center_radius) for point in profile]
                return prism_yz(pts, cut_width).rotate((1.0, 0.0, 0.0), radians(angle_deg))

            if spoke_style in {"straight", "solid_slots"}:
                for index in range(spoke_count):
                    cutters.append(
                        radial_slot(
                            360.0 * index / float(spoke_count),
                            (hub.radius + face_outer_radius) * 0.5,
                            slot_length,
                            window_width * (1.5 if spoke_style == "solid_slots" else 1.0),
                        )
                    )
            elif spoke_style == "split_y":
                for index in range(spoke_count):
                    base = 360.0 * index / float(spoke_count)
                    cutters.append(
                        radial_slot(
                            base - 10.0,
                            hub.radius + (face_outer_radius - hub.radius) * 0.44,
                            slot_length * 0.55,
                            window_width * 0.85,
                        )
                    )
                    cutters.append(
                        radial_slot(
                            base + 10.0,
                            hub.radius + (face_outer_radius - hub.radius) * 0.64,
                            slot_length * 0.55,
                            window_width * 0.85,
                        )
                    )
            else:
                rows = (
                    hub.radius + (face_outer_radius - hub.radius) * 0.36,
                    hub.radius + (face_outer_radius - hub.radius) * 0.68,
                )
                for row_index, center_radius in enumerate(rows):
                    count = spoke_count * (2 if row_index == 1 else 1)
                    for index in range(count):
                        cutters.append(
                            radial_slot(
                                360.0 * index / float(count)
                                + (180.0 / float(count) if row_index == 1 else 0.0),
                                center_radius,
                                slot_length * (0.30 if row_index == 0 else 0.24),
                                window_width * 0.72,
                            )
                        )
            for cutter in cutters:
                shape = boolean_difference(shape, cutter)

        if hub.bolt_pattern is not None:
            pattern = hub.bolt_pattern
            if pattern.count < 2 or pattern.circle_diameter <= 0.0 or pattern.hole_diameter <= 0.0:
                raise ValueError(
                    "BoltPattern count, circle_diameter, and hole_diameter must be positive"
                )
            bolt_r = pattern.circle_diameter * 0.5
            if bolt_r + pattern.hole_diameter * 0.5 >= hub.radius:
                raise ValueError("BoltPattern holes must fit inside the hub")
            for index in range(pattern.count):
                angle = 2.0 * pi * index / float(pattern.count)
                hole = cylinder_x(pattern.hole_diameter * 0.5, width + 0.04).translate(
                    0.0, bolt_r * cos(angle), bolt_r * sin(angle)
                )
                shape = boolean_difference(shape, hole)

        if bore.style == "round":
            bore_cut = cylinder_x(bore.diameter * 0.5, width + 0.04)
        elif bore.style == "hex":
            br = bore.diameter * 0.5
            hex_pts = [(br * cos(radians(60 * k)), br * sin(radians(60 * k))) for k in range(6)]
            bore_cut = prism_yz(hex_pts, width + 0.04)
        else:
            key_width = bore.key_width or bore.diameter * 0.32
            bore_cut = cylinder_x(bore.diameter * 0.5, width + 0.04)
            key = BoxGeometry((bore.diameter * 0.5, width + 0.04, key_width)).translate(
                0.0, bore.diameter * 0.25, 0.0
            )
            bore_cut = boolean_union(bore_cut, key)
        shape = boolean_difference(shape, bore_cut)

        if flange is not None:
            if flange.radius <= 0.0 or flange.thickness <= 0.0:
                raise ValueError("WheelFlange radius and thickness must be positive")
            ring_inner = max(flange.radius - flange.thickness, hub.radius * 1.05)
            ring = annulus_x(ring_inner, flange.radius, flange.thickness * 0.6)
            bridge_len = max(flange.offset + flange.thickness * 0.6, flange.thickness * 0.8)
            bridge_radial = max(flange.radius - rim_outer + flange.thickness * 0.35, 0.006)
            bridge_thickness = max(flange.thickness * 0.55, 0.0015)
            x_offsets: list[float] = []
            if flange.side in {"front", "both"}:
                x_offsets.append(width * 0.5 + flange.offset)
            if flange.side in {"rear", "both"}:
                x_offsets.append(-(width * 0.5 + flange.offset))
            for x_offset in x_offsets:
                shape = boolean_union(shape, ring.translate(x_offset, 0.0, 0.0))
                axial_mid = (
                    (width * 0.5 + x_offset) * 0.5
                    if x_offset >= 0.0
                    else (-(width * 0.5) + x_offset) * 0.5
                )
                radial_mid = (rim_outer + ring_inner) * 0.5
                for angle_deg in (0.0, 90.0, 180.0, 270.0):
                    strut = BoxGeometry((bridge_len, bridge_radial, bridge_thickness)).translate(
                        axial_mid, radial_mid, 0.0
                    )
                    shape = boolean_union(shape, strut.rotate((1.0, 0.0, 0.0), radians(angle_deg)))

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_axis0(geom, 0)
        _adopt_mesh_geometry(self, geom)


class TireGeometry(MeshGeometry):
    """
    Build a tire aligned to local X.
    """

    def __init__(
        self,
        outer_radius: float,
        width: float,
        *,
        inner_radius: Optional[float] = None,
        carcass: Optional[TireCarcass] = None,
        tread: Optional[TireTread] = None,
        grooves: Sequence[TireGroove] = (),
        sidewall: Optional[TireSidewall] = None,
        shoulder: Optional[TireShoulder] = None,
        center: bool = True,
    ):
        super().__init__()
        outer_radius = float(outer_radius)
        width = float(width)
        inner_radius = float(inner_radius) if inner_radius is not None else outer_radius * 0.58
        carcass = carcass or TireCarcass()
        tread = tread or TireTread()
        sidewall = sidewall or TireSidewall()
        shoulder = shoulder or TireShoulder()

        if outer_radius <= 0.0 or width <= 0.0 or inner_radius <= 0.0:
            raise ValueError("outer_radius, width, and inner_radius must be positive")
        if inner_radius >= outer_radius:
            raise ValueError("inner_radius must be less than outer_radius")
        if tread.depth < 0.0:
            raise ValueError("TireTread.depth must be non-negative")
        for groove in grooves:
            if groove.width < 0.0 or groove.depth < 0.0:
                raise ValueError("TireGroove width/depth must be non-negative")

        half_w = width * 0.5
        belt_half = half_w * _clamp(carcass.belt_width_ratio, 0.12, 0.96)
        shoulder_w = shoulder.width if shoulder.width > 1e-6 else width * 0.11
        side_bulge = max(sidewall.bulge, carcass.sidewall_bulge, 0.0)
        inner_crown = inner_radius * 0.82
        if sidewall.style == "square":
            outer_side = outer_radius
            shoulder_radius = max(
                outer_radius - max(shoulder.radius, tread.depth * 0.5, outer_radius * 0.01),
                inner_radius + 1.0e-4,
            )
        elif sidewall.style == "flat":
            outer_side = max(outer_radius - width * 0.08, inner_radius + 1.0e-4)
            shoulder_radius = outer_radius
        elif sidewall.style == "bulged":
            outer_side = min(outer_radius + side_bulge * width, outer_radius + width * 0.12)
            shoulder_radius = outer_radius
        else:
            outer_side = max(outer_radius - width * 0.04, inner_radius + 1.0e-4)
            shoulder_radius = outer_radius

        profile = [
            (-half_w, inner_radius),
            (-half_w * 0.92, inner_radius),
            (
                -half_w * 0.78,
                max(inner_radius + (outer_side - inner_radius) * 0.28, inner_radius + 1.0e-4),
            ),
            (-(belt_half + shoulder_w), outer_side),
            (-belt_half, shoulder_radius),
            (0.0, outer_radius),
            (belt_half, shoulder_radius),
            (belt_half + shoulder_w, outer_side),
            (
                half_w * 0.78,
                max(inner_radius + (outer_side - inner_radius) * 0.28, inner_radius + 1.0e-4),
            ),
            (half_w * 0.92, inner_radius),
            (half_w, inner_radius),
            (half_w * 0.55, inner_crown),
            (0.0, inner_crown),
            (-half_w * 0.55, inner_crown),
            (-half_w, inner_radius),
        ]
        shape = _revolve_x(profile)

        for groove in grooves:
            if groove.width <= 1e-6 or groove.depth <= 1e-6:
                continue
            cutter = _revolve_x(
                [
                    (groove.center_offset - groove.width * 0.5, outer_radius - groove.depth),
                    (groove.center_offset + groove.width * 0.5, outer_radius - groove.depth),
                    (groove.center_offset + groove.width * 0.5, outer_radius + groove.depth * 1.6),
                    (groove.center_offset - groove.width * 0.5, outer_radius + groove.depth * 1.6),
                ]
            )
            shape = boolean_difference(shape, cutter)

        tread_style = tread.style
        tread_depth = tread.depth
        if tread_style == "circumferential" and tread_depth > 1e-6:
            groove_count = tread.count or 3
            band_positions = _centered_pattern_positions(
                groove_count, max(tread.pitch or (width * 0.20), width * 0.14)
            )
            for pos in band_positions:
                cutter = _revolve_x(
                    [
                        (pos - width * 0.03, outer_radius - tread_depth),
                        (pos + width * 0.03, outer_radius - tread_depth),
                        (pos + width * 0.03, outer_radius + tread_depth * 1.4),
                        (pos - width * 0.03, outer_radius + tread_depth * 1.4),
                    ]
                )
                shape = boolean_difference(shape, cutter)
        elif tread_style == "rib" and tread_depth > 1e-6:
            rib_count = tread.count or 3
            rib_positions = _centered_pattern_positions(
                rib_count, max(tread.pitch or (width * 0.20), width * 0.14)
            )
            for pos in rib_positions:
                rib = _revolve_x(
                    [
                        (pos - width * 0.04, outer_radius),
                        (pos + width * 0.04, outer_radius),
                        (pos + width * 0.04, outer_radius + tread_depth),
                        (pos - width * 0.04, outer_radius + tread_depth),
                    ]
                )
                shape = boolean_union(shape, rib)
        elif tread_style in {"block", "lug", "chevron"} and tread_depth > 1e-6:
            circ_count = tread.count or (
                12
                if tread_style == "lug"
                else max(
                    14,
                    int(
                        (2.0 * pi * outer_radius) / max(tread.pitch or (width * 0.45), width * 0.24)
                    ),
                )
            )
            axial_rows = (
                [-width * 0.22, width * 0.22]
                if tread_style == "lug"
                else (
                    [-width * 0.22, 0.0, width * 0.22]
                    if tread_style == "block"
                    else [-width * 0.18, width * 0.18]
                )
            )
            tangential = (2.0 * pi * outer_radius / float(circ_count)) * _clamp(
                tread.land_ratio, 0.18, 0.85
            )
            axial_width = width * (0.14 if tread_style == "lug" else 0.12)
            for row_index, row_x in enumerate(axial_rows):
                for index in range(circ_count):
                    angle_deg = 360.0 * index / float(circ_count) + (
                        180.0 / float(circ_count) if row_index % 2 == 1 else 0.0
                    )
                    if tread_style == "chevron":
                        for sign in (-1.0, 1.0):
                            tilt = (
                                tread.angle_deg * sign
                                if abs(tread.angle_deg) > 1e-6
                                else 24.0 * sign
                            )
                            lug = (
                                _box_solid(axial_width, tread_depth * 1.35, tangential * 0.48)
                                .translate(
                                    row_x + sign * axial_width * 0.38,
                                    outer_radius + tread_depth * 0.16,
                                    0.0,
                                )
                                .rotate((0.0, 1.0, 0.0), radians(tilt))
                                .rotate((1.0, 0.0, 0.0), radians(angle_deg))
                            )
                            shape = boolean_union(shape, lug)
                    else:
                        lug = (
                            _box_solid(
                                axial_width,
                                tread_depth * 1.35,
                                tangential * (0.55 if tread_style == "lug" else 0.42),
                            )
                            .translate(row_x, outer_radius + tread_depth * 0.16, 0.0)
                            .rotate((1.0, 0.0, 0.0), radians(angle_deg))
                        )
                        shape = boolean_union(shape, lug)

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_axis0(geom, 0)
        _adopt_mesh_geometry(self, geom)
