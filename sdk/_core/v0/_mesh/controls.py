from __future__ import annotations

from math import cos, pi, radians, sin
from typing import Literal, Optional, Sequence, Union

import numpy as np

from .booleans import boolean_difference, boolean_union
from .native_build import cylinder_z
from .primitives import (
    ExtrudeGeometry,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    _adopt_mesh_geometry,
    _mesh_geometry_shifted_to_z0,
)
from .shape_helpers import (
    _centered_pattern_positions,
    _shape_profile_points_2d,
    _shape_size_from_wall,
)
from .specs import (
    BezelCutout,
    BezelEdgeFeature,
    BezelFace,
    BezelFlange,
    BezelMounts,
    BezelRecess,
    BezelVisor,
    KnobBore,
    KnobGrip,
    KnobIndicator,
    KnobRelief,
    KnobSkirt,
    KnobTopFeature,
)


def _box(sx: float, sy: float, sz: float) -> MeshGeometry:
    """Outward-wound rectangular prism for use as a boolean tool (avoids inward BoxGeometry)."""
    half_x = sx * 0.5
    half_y = sy * 0.5
    rect = [
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    ]
    return ExtrudeGeometry(rect, sz)


def _polygon_extrude(points: Sequence[tuple[float, float]], height: float) -> MeshGeometry:
    """Extrude a closed 2D profile one-sided along +Z (``polyline(pts).close().extrude(h)``)."""
    return ExtrudeGeometry(points, height).translate(0.0, 0.0, height * 0.5)


def _natural_cubic_samples(
    zs: Sequence[float], rs: Sequence[float], *, factor: int = 4
) -> list[tuple[float, float]]:
    """Densify (z, r) control points with a natural cubic spline.

    cq lofted its circle sections with ``ruled=False`` (a smooth BSpline through the
    sections). A straight-segment lathe cuts the spline's corners, so resample the
    silhouette through a natural cubic spline to recover the bulged profile.
    """
    z = np.asarray(zs, dtype=float)
    r = np.asarray([max(1.0e-4, float(value)) for value in rs], dtype=float)
    n = z.size
    if n < 3:
        return [(float(zi), float(ri)) for zi, ri in zip(z, r)]
    h = np.diff(z)
    matrix = np.zeros((n, n))
    rhs = np.zeros(n)
    matrix[0, 0] = 1.0
    matrix[-1, -1] = 1.0
    for i in range(1, n - 1):
        matrix[i, i - 1] = h[i - 1]
        matrix[i, i] = 2.0 * (h[i - 1] + h[i])
        matrix[i, i + 1] = h[i]
        rhs[i] = 3.0 * ((r[i + 1] - r[i]) / h[i] - (r[i] - r[i - 1]) / h[i - 1])
    c = np.linalg.solve(matrix, rhs)

    def evaluate(value: float) -> float:
        i = int(min(max(np.searchsorted(z, value) - 1, 0), n - 2))
        dz = value - z[i]
        b = (r[i + 1] - r[i]) / h[i] - h[i] * (2.0 * c[i] + c[i + 1]) / 3.0
        d = (c[i + 1] - c[i]) / (3.0 * h[i])
        return float(r[i] + b * dz + c[i] * dz * dz + d * dz**3)

    dense = np.linspace(z[0], z[-1], factor * (n - 1) + 1)
    return [(float(value), max(1.0e-4, evaluate(float(value)))) for value in dense]


def _frustum_z(
    radii_and_offsets: Sequence[tuple[float, float]], *, segments: int = 128
) -> MeshGeometry:
    """Solid revolved between circle sections along Z (replaces ``_loft_between_radii_z``).

    For three or more sections the silhouette is resampled through a natural cubic
    spline to mirror cq's ``loft(ruled=False)``; two-section frustums stay linear.
    """
    zs = [offset for _radius, offset in radii_and_offsets]
    rs = [radius for radius, _offset in radii_and_offsets]
    samples = _natural_cubic_samples(zs, rs)
    profile: list[tuple[float, float]] = [(0.0, samples[0][0])]
    for z_value, radius in samples:
        profile.append((max(1.0e-4, radius), z_value))
    profile.append((0.0, samples[-1][0]))
    return LatheGeometry(profile, segments=segments)


def _ring_solid(
    outer_points: Sequence[tuple[float, float]],
    inner_points: Sequence[tuple[float, float]],
    depth: float,
    *,
    center: bool = True,
) -> MeshGeometry:
    """Extrude an outer profile and subtract an inner profile (replaces ``_cq_ring_solid``)."""
    outer = ExtrudeGeometry(outer_points, depth, center=center)
    inner_height = depth + max(0.002, depth * 0.5)
    inner = ExtrudeGeometry(inner_points, inner_height, center=center)
    return boolean_difference(outer, inner)


def _circle_section_z(
    z: float, radius: float, segments: int = 64
) -> list[tuple[float, float, float]]:
    """A circle of given radius at height z (for LoftGeometry sections along Z)."""
    return [
        (radius * cos(2.0 * pi * k / segments), radius * sin(2.0 * pi * k / segments), z)
        for k in range(segments)
    ]


class KnobGeometry(MeshGeometry):
    """
    Build a flexible rotary knob aligned to local Z with multiple silhouette families.
    """

    def __init__(
        self,
        diameter: float,
        height: float,
        *,
        body_style: Literal[
            "cylindrical",
            "tapered",
            "domed",
            "mushroom",
            "skirted",
            "hourglass",
            "faceted",
            "lobed",
        ] = "cylindrical",
        top_diameter: Optional[float] = None,
        base_diameter: Optional[float] = None,
        crown_radius: float = 0.0,
        edge_radius: float = 0.0,
        side_draft_deg: float = 0.0,
        skirt: Optional[KnobSkirt] = None,
        grip: Optional[KnobGrip] = None,
        indicator: Optional[KnobIndicator] = None,
        top_feature: Optional[KnobTopFeature] = None,
        bore: Optional[KnobBore] = None,
        body_reliefs: Sequence[KnobRelief] = (),
        center: bool = True,
    ):
        super().__init__()
        diameter = float(diameter)
        height = float(height)
        crown_radius = max(0.0, float(crown_radius))
        edge_radius = max(0.0, float(edge_radius))
        side_draft_deg = float(side_draft_deg)
        if diameter <= 0.0 or height <= 0.0:
            raise ValueError("diameter and height must be positive")
        if abs(side_draft_deg) >= 45.0:
            raise ValueError("abs(side_draft_deg) must be < 45")

        base_diameter = float(base_diameter) if base_diameter is not None else diameter
        if base_diameter <= 0.0:
            raise ValueError("base_diameter must be positive")
        top_diameter = float(top_diameter) if top_diameter is not None else diameter
        if top_diameter <= 0.0:
            raise ValueError("top_diameter must be positive")

        skirt = skirt
        grip = grip or KnobGrip()
        indicator = indicator or KnobIndicator()
        top_feature = top_feature or KnobTopFeature()
        bore = bore or KnobBore(style="none")
        if skirt is not None and (skirt.diameter <= 0.0 or skirt.height <= 0.0):
            raise ValueError("KnobSkirt diameter and height must be positive")
        if grip.depth < 0.0:
            raise ValueError("KnobGrip.depth must be non-negative")
        if indicator.depth < 0.0:
            raise ValueError("KnobIndicator.depth must be non-negative")
        if top_feature.depth < 0.0 or top_feature.height < 0.0:
            raise ValueError("KnobTopFeature depth/height must be non-negative")
        if bore.diameter is not None and bore.diameter <= 0.0:
            raise ValueError("KnobBore.diameter must be positive when provided")
        for relief in body_reliefs:
            if relief.depth < 0.0:
                raise ValueError("KnobRelief.depth must be non-negative")

        body_height = height
        body_bottom = -height * 0.5
        max_radius = max(base_diameter, top_diameter) * 0.5
        if skirt is not None:
            max_radius = max(max_radius, skirt.diameter * 0.5)

        def body_radius_at(t: float) -> float:
            if body_style == "cylindrical":
                return max(base_diameter, top_diameter) * 0.5
            if body_style == "tapered":
                return (base_diameter * (1.0 - t) + top_diameter * t) * 0.5
            if body_style == "domed":
                if t < 0.72:
                    return (base_diameter * 0.5) * (
                        1.0 - min(max(side_draft_deg / 50.0, -0.2), 0.2) * t
                    )
                u = (t - 0.72) / 0.28
                return max(
                    0.001,
                    top_diameter * 0.5 + (base_diameter * 0.5 - top_diameter * 0.5) * (1.0 - u * u),
                )
            if body_style == "mushroom":
                stem_radius = min(base_diameter, top_diameter, diameter) * 0.28
                cap_radius = max(base_diameter, top_diameter, diameter) * 0.5
                if t < 0.42:
                    return stem_radius
                if t < 0.62:
                    u = (t - 0.42) / 0.20
                    return stem_radius + (cap_radius - stem_radius) * u
                if t < 0.85:
                    return cap_radius
                u = (t - 0.85) / 0.15
                return max(cap_radius * (1.0 - 0.20 * u * u), stem_radius)
            if body_style == "skirted":
                skirt_radius = max(base_diameter * 0.55, diameter * 0.52)
                crown_radius_local = top_diameter * 0.5
                if t < 0.40:
                    return skirt_radius
                if t < 0.56:
                    u = (t - 0.40) / 0.16
                    return skirt_radius + (crown_radius_local - skirt_radius) * u
                return crown_radius_local
            if body_style == "hourglass":
                waist_radius = min(base_diameter, top_diameter, diameter) * 0.32
                if t < 0.5:
                    u = t / 0.5
                    return base_diameter * 0.5 + (waist_radius - base_diameter * 0.5) * u
                u = (t - 0.5) / 0.5
                return waist_radius + (top_diameter * 0.5 - waist_radius) * u
            if body_style == "faceted":
                return (base_diameter * (1.0 - t) + top_diameter * t) * 0.5
            if body_style == "lobed":
                lower_radius = base_diameter * 0.5
                upper_radius = max(top_diameter, diameter * 1.02) * 0.5
                if t < 0.24:
                    u = t / 0.24
                    return lower_radius + (upper_radius * 0.92 - lower_radius) * u
                if t < 0.80:
                    u = (t - 0.24) / 0.56
                    return upper_radius * (0.92 + 0.08 * sin(u * pi))
                u = (t - 0.80) / 0.20
                return upper_radius + (top_diameter * 0.5 - upper_radius) * u
            raise ValueError(f"Unsupported body_style {body_style!r}")

        def section_outline(radius: float, t: float) -> list[tuple[float, float]] | None:
            if body_style == "faceted":
                facet_count = 6
                phase = pi / float(facet_count)
                return [
                    (
                        radius * cos(phase + 2.0 * pi * index / float(facet_count)),
                        radius * sin(phase + 2.0 * pi * index / float(facet_count)),
                    )
                    for index in range(facet_count)
                ]
            if body_style == "lobed":
                lobe_count = 5
                point_count = 72
                blend = min(max((t - 0.10) / 0.30, 0.0), 1.0)
                amplitude = radius * (0.04 + 0.14 * blend)
                valley_floor = radius * 0.62
                points: list[tuple[float, float]] = []
                for index in range(point_count):
                    theta = 2.0 * pi * index / float(point_count)
                    local_radius = radius - amplitude * (0.5 - 0.5 * cos(lobe_count * theta))
                    local_radius = max(local_radius, valley_floor)
                    points.append((local_radius * cos(theta), local_radius * sin(theta)))
                return points
            return None

        section_offsets = [
            0.0,
            body_height * 0.18,
            body_height * 0.42,
            body_height * 0.72,
            body_height,
        ]
        radii_and_offsets = [
            (max(0.001, body_radius_at(offset / body_height)), body_bottom + offset)
            for offset in section_offsets
        ]
        # cq lofted the circle/polygon sections; build the equivalent native loft of
        # constant-z section loops. Circular sections become LatheGeometry (smoother and
        # always manifold); non-circular families loft their explicit point loops.
        if body_style in {"faceted", "lobed"}:
            loft_sections: list[list[tuple[float, float, float]]] = []
            for radius, offset in radii_and_offsets:
                t = (offset - body_bottom) / body_height if body_height > 1e-9 else 0.0
                profile_points = section_outline(radius, t)
                if profile_points is None:
                    profile_points = [
                        (radius * cos(2.0 * pi * k / 64), radius * sin(2.0 * pi * k / 64))
                        for k in range(64)
                    ]
                loft_sections.append([(px, py, offset) for px, py in profile_points])
            shape: MeshGeometry = LoftGeometry(loft_sections)
        else:
            shape = _frustum_z(radii_and_offsets)

        if skirt is not None:
            skirt_radius = skirt.diameter * 0.5
            skirt_bottom = body_bottom - skirt.height
            skirt_shape = _frustum_z(
                [
                    (max(0.001, skirt_radius * (1.0 + skirt.flare)), skirt_bottom),
                    (max(0.001, skirt_radius), body_bottom),
                ]
            )
            shape = boolean_union(shape, skirt_shape)
            # cq ``faces("<Z").edges().chamfer(...)`` on the skirt bottom is a cosmetic 3D
            # edge round and is unsupported in the native layer -> SKIP.

        # cq ``edges("|Z").fillet`` and ``faces(">Z").edges().fillet`` are cosmetic 3D edge
        # rounds with no native equivalent -> SKIP (edge_radius / crown_radius).

        if grip.style != "none" and grip.depth > 1e-6:
            grip_count = grip.count or (28 if grip.style in {"knurled", "diamond_knurl"} else 18)
            if grip_count < 2:
                raise ValueError("KnobGrip.count must be at least 2")
            grip_width = (
                float(grip.width)
                if grip.width is not None
                else (
                    max_radius * 0.18
                    if grip.style in {"fluted", "scalloped"}
                    else max_radius * 0.10
                )
            )
            if grip_width <= 0.0:
                raise ValueError("KnobGrip.width must be positive when provided")

            if grip.style in {"fluted", "scalloped", "ribbed"}:
                cutter_radius = grip_width * (0.55 if grip.style != "ribbed" else 0.38)
                radial_center = max_radius + cutter_radius - grip.depth
                cut_len = height + (skirt.height if skirt is not None else 0.0) + 0.01
                for index in range(grip_count):
                    cutter = (
                        cylinder_z(cutter_radius, 2.0 * cut_len)
                        .translate(radial_center, 0.0, body_bottom + height * 0.5)
                        .rotate((0.0, 0.0, 1.0), radians(360.0 * index / float(grip_count)))
                    )
                    shape = boolean_difference(shape, cutter)
            else:
                tangential = max(grip_width, max_radius * 0.06)
                radial = max(grip.depth * 1.8, max_radius * 0.06)
                box_height = height * 1.25
                helix_angle = grip.helix_angle_deg if abs(grip.helix_angle_deg) > 1e-6 else 24.0
                for index in range(grip_count):
                    base_angle = 360.0 * index / float(grip_count)
                    for tilt_sign in (-1.0, 1.0) if grip.style == "diamond_knurl" else (1.0,):
                        cutter = (
                            _box(radial, tangential, box_height)
                            .translate(
                                max_radius - grip.depth * 0.5, 0.0, body_bottom + height * 0.5
                            )
                            .rotate((0.0, 1.0, 0.0), radians(helix_angle * float(tilt_sign)))
                            .rotate((0.0, 0.0, 1.0), radians(base_angle))
                        )
                        shape = boolean_difference(shape, cutter)

        if indicator.style != "none":
            indicator_length = indicator.length or max(diameter * 0.34, 0.003)
            indicator_width = indicator.width or max(diameter * 0.06, 0.0015)
            indicator_depth = max(indicator.depth, max(height * 0.03, 0.0008))
            top_z = body_bottom + height
            if indicator.style in {"line", "notch"}:
                feature = _box(indicator_length, indicator_width, indicator_depth).translate(
                    indicator_length * 0.18, 0.0, top_z + indicator_depth * 0.5
                )
            elif indicator.style == "wedge":
                profile = [
                    (-indicator_width * 0.5, 0.0),
                    (indicator_width * 0.5, 0.0),
                    (0.0, indicator_length),
                ]
                feature = _polygon_extrude(profile, indicator_depth).translate(0.0, 0.0, top_z)
            else:
                dot_radius = indicator_width * 0.5
                feature = cylinder_z(dot_radius, indicator_depth).translate(
                    indicator_length * 0.22, 0.0, top_z + indicator_depth * 0.5
                )
            feature = feature.rotate((0.0, 0.0, 1.0), radians(indicator.angle_deg))
            if indicator.mode == "raised" and indicator.style != "notch":
                shape = boolean_union(shape, feature)
            else:
                shape = boolean_difference(
                    shape, feature.translate(0.0, 0.0, -indicator_depth * 0.5)
                )

        if top_feature.style != "none":
            feature_diameter = top_feature.diameter or diameter * 0.55
            feature_radius = feature_diameter * 0.5
            top_z = body_bottom + height
            if feature_radius <= 0.0:
                raise ValueError("KnobTopFeature.diameter must be positive when provided")
            if top_feature.style == "flush_disk":
                feat_h = max(top_feature.height, height * 0.06)
                feature = cylinder_z(feature_radius, feat_h).translate(
                    0.0, 0.0, top_z + feat_h * 0.5
                )
                shape = boolean_union(shape, feature)
            elif top_feature.style == "top_insert":
                feat_h = max(top_feature.height, height * 0.04)
                feature = cylinder_z(feature_radius, feat_h).translate(
                    0.0, 0.0, top_z + height * 0.01 + feat_h * 0.5
                )
                shape = boolean_union(shape, feature)
            else:
                feat_h = max(top_feature.depth, height * 0.08)
                feature = cylinder_z(feature_radius, feat_h).translate(
                    0.0, 0.0, top_z - feat_h * 0.5
                )
                shape = boolean_difference(shape, feature)

        if bore.style != "none":
            bore_diameter = bore.diameter or diameter * 0.34
            if bore_diameter <= 0.0 or bore_diameter >= max_radius * 2.0:
                raise ValueError("KnobBore diameter must fit inside the knob body")
            bore_depth = (
                height + (skirt.height if skirt is not None else 0.0) + 0.01
                if bore.through
                else max(height * 0.7, diameter * 0.22)
            )
            if bore.style == "round":
                bore_cut = cylinder_z(bore_diameter * 0.5, bore_depth).translate(
                    0.0, 0.0, bore_depth * 0.5
                )
            elif bore.style == "hex":
                # cq ``polygon(6, d)`` inscribes verts on a circle of diameter d (radius d/2).
                hex_pts = [
                    (
                        0.5 * bore_diameter * cos(2.0 * pi * k / 6),
                        0.5 * bore_diameter * sin(2.0 * pi * k / 6),
                    )
                    for k in range(6)
                ]
                bore_cut = _polygon_extrude(hex_pts, bore_depth)
            elif bore.style in {"d_shaft", "double_d"}:
                flat_depth = (
                    float(bore.flat_depth) if bore.flat_depth is not None else bore_diameter * 0.16
                )
                cut_r = bore_diameter * 0.5
                flat_x = cut_r - flat_depth
                circle_points = [
                    (cut_r * cos(theta), cut_r * sin(theta))
                    for theta in np.linspace(0.0, 2.0 * pi, 48, endpoint=False)
                ]
                if bore.style == "d_shaft":
                    profile_points = [(min(point[0], flat_x), point[1]) for point in circle_points]
                else:
                    profile_points = [
                        (max(min(point[0], flat_x), -flat_x), point[1]) for point in circle_points
                    ]
                bore_cut = _polygon_extrude(profile_points, bore_depth)
            else:
                spline_count = bore.spline_count or 8
                if spline_count < 3:
                    raise ValueError("KnobBore.spline_count must be at least 3")
                spline_depth = max(bore.spline_depth, bore_diameter * 0.06)
                outer_r = bore_diameter * 0.5
                inner_r = max(outer_r - spline_depth, outer_r * 0.72)
                points = []
                for index in range(spline_count * 2):
                    theta = pi * index / float(spline_count)
                    radius = outer_r if index % 2 == 0 else inner_r
                    points.append((radius * cos(theta), radius * sin(theta)))
                bore_cut = _polygon_extrude(points, bore_depth)
            bore_cut = bore_cut.translate(
                0.0, 0.0, body_bottom - (skirt.height if skirt is not None else 0.0)
            )
            shape = boolean_difference(shape, bore_cut)

        for relief in body_reliefs:
            relief_depth = max(relief.depth, diameter * 0.04)
            relief_width = relief.width or diameter * 0.22
            relief_height = relief.height or height * 0.18
            if relief.style == "side_window":
                cutter = (
                    _box(relief_depth * 2.2, relief_width, relief_height)
                    .translate(max_radius - relief_depth * 0.55, 0.0, body_bottom + height * 0.55)
                    .rotate((0.0, 0.0, 1.0), radians(relief.angle_deg))
                )
                shape = boolean_difference(shape, cutter)
            elif relief.style == "top_recess":
                cutter = cylinder_z(relief_width * 0.5, relief_depth).translate(
                    0.0, 0.0, body_bottom + height - relief_depth * 0.5
                )
                shape = boolean_difference(shape, cutter)
            else:
                cutter = (
                    _box(relief_width, relief_width * 0.24, relief_depth)
                    .translate(0.0, 0.0, body_bottom + height - relief_depth * 0.5)
                    .rotate((0.0, 0.0, 1.0), radians(relief.angle_deg))
                )
                shape = boolean_difference(shape, cutter)

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)


class BezelGeometry(MeshGeometry):
    """
    Build a framed opening with optional recess, visor, flange, and rear mounts.
    """

    def __init__(
        self,
        opening_size: Sequence[float],
        outer_size: Sequence[float],
        depth: float,
        *,
        opening_shape: Literal[
            "rect", "rounded_rect", "circle", "ellipse", "superellipse"
        ] = "rounded_rect",
        outer_shape: Literal[
            "rect", "rounded_rect", "circle", "ellipse", "superellipse"
        ] = "rounded_rect",
        opening_corner_radius: float = 0.0,
        outer_corner_radius: float = 0.0,
        wall: Union[float, tuple[float, float, float, float], None] = None,
        face: Optional[BezelFace] = None,
        recess: Optional[BezelRecess] = None,
        visor: Optional[BezelVisor] = None,
        flange: Optional[BezelFlange] = None,
        mounts: Optional[BezelMounts] = None,
        cutouts: Sequence[BezelCutout] = (),
        edge_features: Sequence[BezelEdgeFeature] = (),
        center: bool = True,
    ):
        super().__init__()
        opening_w = float(opening_size[0])
        opening_h = float(opening_size[1])
        outer_w = float(outer_size[0])
        outer_h = float(outer_size[1])
        depth = float(depth)
        opening_corner_radius = max(0.0, float(opening_corner_radius))
        outer_corner_radius = max(0.0, float(outer_corner_radius))
        face = face or BezelFace()
        visor = visor or BezelVisor()
        flange = flange or BezelFlange()
        mounts = mounts or BezelMounts()

        if min(opening_w, opening_h, outer_w, outer_h, depth) <= 0.0:
            raise ValueError("opening_size, outer_size, and depth must be positive")
        if opening_w >= outer_w or opening_h >= outer_h:
            raise ValueError("opening_size must be smaller than outer_size on both axes")
        if recess is not None and (recess.depth <= 0.0 or recess.inset < 0.0):
            raise ValueError("BezelRecess depth must be positive and inset must be non-negative")

        outer_points = _shape_profile_points_2d(
            outer_shape,
            (outer_w, outer_h),
            corner_radius=outer_corner_radius,
        )
        opening_points = _shape_profile_points_2d(
            opening_shape,
            (opening_w, opening_h),
            corner_radius=opening_corner_radius,
        )
        shape: MeshGeometry = _ring_solid(outer_points, opening_points, depth, center=True)

        # cq ``edges("|Z").fillet/chamfer`` on the frame face is a cosmetic 3D edge round
        # with no native equivalent -> SKIP (face.style rounded/chamfered/radiused_step).

        derived_wall = (
            (outer_w - opening_w) * 0.5,
            (outer_w - opening_w) * 0.5,
            (outer_h - opening_h) * 0.5,
            (outer_h - opening_h) * 0.5,
        )
        if face.front_lip > 1e-6 or face.style == "radiused_step":
            lip_thickness = max(face.front_lip, depth * 0.08, 0.0015)
            if wall is not None:
                lip_size = _shape_size_from_wall(opening_size, wall)
            else:
                lip_size = (
                    min(
                        opening_w + max(face.front_lip * 2.0, derived_wall[0] * 1.05),
                        outer_w - 0.001,
                    ),
                    min(
                        opening_h + max(face.front_lip * 2.0, derived_wall[2] * 1.05),
                        outer_h - 0.001,
                    ),
                )
            if lip_size[0] < outer_w and lip_size[1] < outer_h:
                lip_outer = _shape_profile_points_2d(
                    opening_shape if outer_shape != "circle" else outer_shape,
                    lip_size,
                    corner_radius=min(
                        opening_corner_radius + face.front_lip,
                        lip_size[0] * 0.25,
                        lip_size[1] * 0.25,
                    ),
                )
                lip = _ring_solid(lip_outer, opening_points, lip_thickness, center=True).translate(
                    0.0, 0.0, depth * 0.5
                )
                shape = boolean_union(shape, lip)

        if recess is not None:
            requested_recess_size = (opening_w + recess.inset * 2.0, opening_h + recess.inset * 2.0)
            if wall is None and (
                requested_recess_size[0] >= outer_w or requested_recess_size[1] >= outer_h
            ):
                raise ValueError("recess wall leaves no outer frame material")
            recess_size = (
                _shape_size_from_wall(opening_size, wall)
                if wall is not None
                else (
                    min(requested_recess_size[0], outer_w - 0.001),
                    min(requested_recess_size[1], outer_h - 0.001),
                )
            )
            if recess_size[0] >= outer_w or recess_size[1] >= outer_h:
                raise ValueError("recess wall leaves no outer frame material")
            recess_points = _shape_profile_points_2d(
                opening_shape if outer_shape != "circle" else outer_shape,
                recess_size,
                corner_radius=min(
                    opening_corner_radius + recess.inset,
                    recess_size[0] * 0.25,
                    recess_size[1] * 0.25,
                ),
            )
            recess_cut = _ring_solid(
                recess_points, opening_points, recess.depth, center=True
            ).translate(0.0, 0.0, depth * 0.5 - recess.depth * 0.5)
            shape = boolean_difference(shape, recess_cut)

        if visor.thickness > 1e-6 and (visor.top_extension > 1e-6 or visor.side_extension > 1e-6):
            top_visor = _box(
                outer_w + visor.side_extension * 2.0,
                max(visor.top_extension, visor.thickness),
                visor.thickness,
            ).translate(0.0, outer_h * 0.5 + visor.top_extension * 0.5, depth * 0.5)
            shape = boolean_union(shape, top_visor)
            if visor.side_extension > 1e-6:
                cheek_y = max(visor.top_extension, outer_h * 0.5) * 0.5
                cheek = _box(
                    visor.side_extension,
                    max(visor.top_extension, outer_h * 0.45),
                    visor.thickness,
                )
                shape = boolean_union(
                    shape,
                    cheek.copy().translate(
                        outer_w * 0.5 + visor.side_extension * 0.5, cheek_y, depth * 0.5
                    ),
                )
                shape = boolean_union(
                    shape,
                    cheek.copy().translate(
                        -(outer_w * 0.5 + visor.side_extension * 0.5), cheek_y, depth * 0.5
                    ),
                )

        if flange.width > 1e-6 and flange.thickness > 1e-6:
            flange_outer = _shape_profile_points_2d(
                outer_shape,
                (outer_w + flange.width * 2.0, outer_h + flange.width * 2.0),
                corner_radius=outer_corner_radius + flange.width,
            )
            flange_shape = _ring_solid(
                flange_outer, outer_points, flange.thickness, center=True
            ).translate(0.0, 0.0, -depth * 0.5 - flange.offset)
            shape = boolean_union(shape, flange_shape)

        if mounts.style == "bosses" and mounts.hole_count > 0:
            boss_radius = (
                max(float(mounts.boss_diameter) * 0.5, 0.0015)
                if mounts.boss_diameter is not None
                else max(min(outer_w, outer_h) * 0.05, 0.003)
            )
            hole_radius = (
                max(float(mounts.hole_diameter) * 0.5, 0.0008)
                if mounts.hole_diameter is not None
                else boss_radius * 0.38
            )
            boss_thickness = max(depth * 0.18, boss_radius * 0.8)
            margin_x = outer_w * 0.5 - boss_radius - max(mounts.setback, 0.001)
            margin_y = outer_h * 0.5 - boss_radius - max(mounts.setback, 0.001)
            boss_points = [
                (-margin_x, -margin_y),
                (margin_x, -margin_y),
                (margin_x, margin_y),
                (-margin_x, margin_y),
            ][: mounts.hole_count]
            for bx, by in boss_points:
                boss = cylinder_z(boss_radius, boss_thickness).translate(
                    bx, by, -depth * 0.5 - boss_thickness * 0.5
                )
                hole_len = boss_thickness + depth + 0.01
                hole = cylinder_z(hole_radius, hole_len).translate(
                    bx, by, -depth * 0.5 - boss_thickness + hole_len * 0.5
                )
                shape = boolean_difference(boolean_union(shape, boss), hole)
        elif mounts.style == "tabs" and mounts.hole_count > 0:
            tab_width = max(outer_w * 0.16, 0.008)
            tab_depth = max(depth * 0.14, 0.002)
            hole_radius = (
                max(float(mounts.hole_diameter) * 0.5, 0.0008)
                if mounts.hole_diameter is not None
                else tab_width * 0.14
            )
            tab_positions = _centered_pattern_positions(
                mounts.hole_count, outer_w / max(mounts.hole_count, 1)
            )
            for px in tab_positions:
                tab = _box(tab_width, tab_width * 0.6, tab_depth).translate(
                    px, -(outer_h * 0.5 + tab_width * 0.3), -depth * 0.5 - tab_depth * 0.5
                )
                hole_len = tab_depth + depth + 0.01
                hole = cylinder_z(hole_radius, hole_len).translate(
                    px,
                    -(outer_h * 0.5 + tab_width * 0.3),
                    -depth * 0.5 - tab_depth + hole_len * 0.5,
                )
                shape = boolean_difference(boolean_union(shape, tab), hole)
        elif (
            mounts.style == "rear_flange"
            and mounts.hole_count > 0
            and mounts.hole_diameter is not None
        ):
            flange_width = max(max(mounts.setback, 0.003), min(outer_w, outer_h) * 0.06)
            rear_flange_outer = _shape_profile_points_2d(
                outer_shape,
                (outer_w + flange_width * 2.0, outer_h + flange_width * 2.0),
                corner_radius=outer_corner_radius + flange_width,
            )
            rear_thickness = max(depth * 0.12, 0.002)
            rear_flange = _ring_solid(
                rear_flange_outer, outer_points, rear_thickness, center=True
            ).translate(0.0, 0.0, -depth * 0.5 - rear_thickness)
            shape = boolean_union(shape, rear_flange)

        for cutout in cutouts:
            if cutout.width <= 0.0 or cutout.depth <= 0.0:
                raise ValueError("BezelCutout width and depth must be positive")
            cut_height = cutout.width
            cut_depth = cutout.depth
            if cutout.edge in {"top", "bottom"}:
                cutter = _box(cutout.width, cut_depth, depth + visor.thickness + 0.02)
                y = (
                    outer_h * 0.5 - cut_depth * 0.5
                    if cutout.edge == "top"
                    else -outer_h * 0.5 + cut_depth * 0.5
                )
                cutter = cutter.translate(cutout.offset, y, 0.0)
            else:
                cutter = _box(cut_depth, cut_height, depth + visor.thickness + 0.02)
                x = (
                    outer_w * 0.5 - cut_depth * 0.5
                    if cutout.edge == "right"
                    else -outer_w * 0.5 + cut_depth * 0.5
                )
                cutter = cutter.translate(x, cutout.offset, 0.0)
            shape = boolean_difference(shape, cutter)

        for feature in edge_features:
            if feature.size <= 0.0:
                continue
            extent = (
                feature.extent
                if feature.extent > 0.0
                else (outer_w if feature.edge in {"top", "bottom"} else outer_h)
            )
            if feature.style == "notch":
                if feature.edge in {"top", "bottom"}:
                    cutter = _box(extent, feature.size, depth + visor.thickness + 0.02)
                    y = (
                        outer_h * 0.5 - feature.size * 0.5
                        if feature.edge == "top"
                        else -outer_h * 0.5 + feature.size * 0.5
                    )
                    cutter = cutter.translate(feature.offset, y, 0.0)
                else:
                    cutter = _box(feature.size, extent, depth + visor.thickness + 0.02)
                    x = (
                        outer_w * 0.5 - feature.size * 0.5
                        if feature.edge == "right"
                        else -outer_w * 0.5 + feature.size * 0.5
                    )
                    cutter = cutter.translate(x, feature.offset, 0.0)
                shape = boolean_difference(shape, cutter)
                continue
            if feature.edge in {"top", "bottom"}:
                solid = _box(extent, feature.size, max(depth * 0.10, 0.0015))
                y = (
                    outer_h * 0.5 + feature.size * 0.5
                    if feature.edge == "top"
                    else -(outer_h * 0.5 + feature.size * 0.5)
                )
                solid = solid.translate(feature.offset, y, depth * 0.5)
            else:
                solid = _box(feature.size, extent, max(depth * 0.10, 0.0015))
                x = (
                    outer_w * 0.5 + feature.size * 0.5
                    if feature.edge == "right"
                    else -(outer_w * 0.5 + feature.size * 0.5)
                )
                solid = solid.translate(x, feature.offset, depth * 0.5)
            if feature.style == "bead":
                shape = boolean_union(shape, solid)
            else:
                shape = boolean_difference(
                    shape, solid.translate(0.0, 0.0, -max(depth * 0.04, 0.0008))
                )

        geom = shape
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)
