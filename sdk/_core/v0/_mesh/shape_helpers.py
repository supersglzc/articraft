"""Pure 2D profile / pattern helpers shared by the geometry generators (no CAD kernel)."""

from __future__ import annotations

from math import cos, pi, sin
from typing import Sequence, Union

from .common import Vec2, rounded_rect_profile, superellipse_profile


def _sample_ellipse_profile(width: float, height: float, *, segments: int = 64) -> list[Vec2]:
    rx = float(width) * 0.5
    ry = float(height) * 0.5
    return [
        (rx * cos(2.0 * pi * index / float(segments)), ry * sin(2.0 * pi * index / float(segments)))
        for index in range(segments)
    ]


def _sample_rect_profile(width: float, height: float) -> list[Vec2]:
    hw = float(width) * 0.5
    hh = float(height) * 0.5
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def _shape_profile_points_2d(
    shape: str,
    size: Sequence[float],
    *,
    corner_radius: float = 0.0,
    segments: int = 64,
) -> list[Vec2]:
    width = float(size[0])
    height = float(size[1])
    if width <= 0.0 or height <= 0.0:
        raise ValueError("shape size values must be positive")
    if shape == "rect":
        return _sample_rect_profile(width, height)
    if shape == "rounded_rect":
        return rounded_rect_profile(
            width, height, max(0.0, corner_radius), corner_segments=max(4, segments // 16)
        )
    if shape == "circle":
        diameter = min(width, height)
        return _sample_ellipse_profile(diameter, diameter, segments=segments)
    if shape == "ellipse":
        return _sample_ellipse_profile(width, height, segments=segments)
    if shape == "superellipse":
        return superellipse_profile(width, height, exponent=2.8, segments=segments)
    raise ValueError(f"Unsupported shape {shape!r}")


def _shape_size_from_wall(
    inner_size: Sequence[float],
    wall: Union[float, Sequence[float]],
) -> tuple[float, float]:
    inner_w = float(inner_size[0])
    inner_h = float(inner_size[1])
    if isinstance(wall, (int, float)):
        wall_left = wall_right = wall_bottom = wall_top = float(wall)
    else:
        if len(wall) != 4:
            raise ValueError("wall must be a float or a 4-sequence")
        wall_left, wall_right, wall_bottom, wall_top = (float(value) for value in wall)
    if min(wall_left, wall_right, wall_bottom, wall_top) < 0.0:
        raise ValueError("wall values must be non-negative")
    return (inner_w + wall_left + wall_right, inner_h + wall_bottom + wall_top)


def _centered_pattern_positions(count: int, spacing: float) -> list[float]:
    if count <= 0:
        return []
    if count == 1:
        return [0.0]
    origin = -0.5 * float(spacing) * float(count - 1)
    return [origin + float(index) * float(spacing) for index in range(count)]


def _rounded_slot_profile(length: float, width: float, *, segments: int = 10) -> list[Vec2]:
    radius = width * 0.5
    straight = max(length - width, 0.0)
    half = straight * 0.5
    points: list[Vec2] = []
    for index in range(segments + 1):
        theta = -pi * 0.5 + pi * (index / float(segments))
        points.append((half + radius * cos(theta), radius * sin(theta)))
    for index in range(segments + 1):
        theta = pi * 0.5 + pi * (index / float(segments))
        points.append((-half + radius * cos(theta), radius * sin(theta)))
    return points
