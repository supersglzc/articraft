from __future__ import annotations

from contextlib import suppress
from dataclasses import dataclass
from typing import Literal, Mapping, Optional, Sequence

import numpy as np

from ._mesh.booleans import boolean_union
from ._mesh.common import (
    sample_catmull_rom_spline_3d,
    sample_cubic_bezier_spline_3d,
)
from ._mesh.primitives import LoftGeometry
from .errors import ValidationError
from .mesh import MeshGeometry, _geometry_from_manifold, _manifold_from_geometry

Vec3 = tuple[float, float, float]

Continuity = Literal["C1", "C2", "C3"]
Parametrization = Literal["uniform", "chordal", "centripetal"]
RepairMode = Literal["off", "mesh", "kernel", "auto"]
SymmetryMode = Literal["mirror_yz"]

# Number of intermediate cross-sections inserted between two user-supplied
# sections when building a smooth (non-ruled) loft. The native loft is ruled
# (straight) between consecutive sections, so we densify with sampled sections
# to make a smooth loft look smooth and stay watertight.
_SMOOTH_SECTION_SUBDIV = 8
# Sampling density for spline/polyline sweep paths.
_PATH_SAMPLES_PER_SEGMENT = 12

__all__ = [
    "LoftSection",
    "LoftTessellation",
    "SectionLoftSpec",
    "section_loft",
    "repair_loft",
]


def _as_vec3(values: Sequence[float], *, name: str) -> Vec3:
    if len(values) != 3:
        raise ValidationError(f"{name} must have 3 elements")
    return (float(values[0]), float(values[1]), float(values[2]))


@dataclass(frozen=True)
class LoftTessellation:
    tolerance: float = 0.001
    angular_tolerance: float = 0.1

    def __post_init__(self) -> None:
        if float(self.tolerance) <= 0.0:
            raise ValidationError("tessellation.tolerance must be > 0")
        if float(self.angular_tolerance) <= 0.0:
            raise ValidationError("tessellation.angular_tolerance must be > 0")
        object.__setattr__(self, "tolerance", float(self.tolerance))
        object.__setattr__(self, "angular_tolerance", float(self.angular_tolerance))


@dataclass(frozen=True)
class LoftSection:
    points: tuple[Vec3, ...]

    def __post_init__(self) -> None:
        coerced = tuple(_normalize_section_points(self.points, name="section.points"))
        object.__setattr__(self, "points", coerced)


@dataclass(frozen=True)
class SectionLoftSpec:
    sections: tuple[LoftSection, ...]
    path: Optional[tuple[Vec3, ...]] = None
    guide_curves: Optional[Mapping[str, tuple[Vec3, ...]]] = None
    cap: bool = True
    solid: bool = True
    symmetry: Optional[SymmetryMode] = None
    ruled: bool = False
    continuity: Continuity = "C2"
    parametrization: Parametrization = "uniform"
    degree: int = 3
    compat: bool = True
    smoothing: bool = False
    weights: tuple[float, float, float] = (1.0, 1.0, 1.0)
    repair: RepairMode = "auto"
    tessellation: LoftTessellation = LoftTessellation()

    def __post_init__(self) -> None:
        sections = tuple(_coerce_loft_section(section) for section in self.sections)
        if len(sections) < 2:
            raise ValidationError("SectionLoftSpec.sections must contain at least two sections")
        object.__setattr__(self, "sections", sections)

        if self.path is not None:
            object.__setattr__(
                self,
                "path",
                tuple(_as_vec3(point, name="path[]") for point in self.path),
            )
        if self.guide_curves is not None:
            guide_curves = {
                str(name): tuple(_as_vec3(point, name=f"guide_curves[{name!r}][]") for point in pts)
                for name, pts in self.guide_curves.items()
            }
            object.__setattr__(self, "guide_curves", guide_curves)

        if self.symmetry is not None and self.symmetry != "mirror_yz":
            raise ValidationError("symmetry must be 'mirror_yz' or None")
        if self.degree < 1:
            raise ValidationError("degree must be >= 1")
        if len(self.weights) != 3:
            raise ValidationError("weights must contain exactly 3 values")
        object.__setattr__(self, "degree", int(self.degree))
        object.__setattr__(
            self,
            "weights",
            (float(self.weights[0]), float(self.weights[1]), float(self.weights[2])),
        )


def _normalize_section_points(points: Sequence[Sequence[float] | Vec3], *, name: str) -> list[Vec3]:
    raw = [_as_vec3(point, name=name) for point in points]
    if len(raw) < 3:
        raise ValidationError(f"{name} must contain at least 3 points")

    out: list[Vec3] = []
    for point in raw:
        if not out or point != out[-1]:
            out.append(point)

    if len(out) >= 2 and out[0] == out[-1]:
        out.pop()

    if len(out) < 3:
        raise ValidationError(f"{name} must contain at least 3 distinct points")
    return out


def _coerce_loft_section(value: LoftSection | Sequence[Sequence[float] | Vec3]) -> LoftSection:
    if isinstance(value, LoftSection):
        return value
    return LoftSection(points=tuple(value))


def _coerce_loft_spec(
    spec: SectionLoftSpec | Sequence[LoftSection | Sequence[Sequence[float] | Vec3]],
) -> SectionLoftSpec:
    if isinstance(spec, SectionLoftSpec):
        return spec
    return SectionLoftSpec(sections=tuple(_coerce_loft_section(section) for section in spec))


def _require_trimesh():
    try:
        import trimesh
    except Exception as exc:  # pragma: no cover - optional import path
        raise RuntimeError(
            "repair_loft requires the `trimesh` package, but it is not installed in the current "
            "environment. Run `uv sync --group dev` from the repository root, then retry."
        ) from exc
    return trimesh


# ---------------------------------------------------------------------------
# Native section-loop construction
# ---------------------------------------------------------------------------


def _resample_loop(loop: Sequence[Vec3], count: int) -> list[Vec3]:
    """Resample a closed 3D loop to ``count`` points by arc-length.

    The loop is treated as closed (the segment from the last point back to the
    first is included). Output points preserve the loop ordering and start at the
    same point as the input loop.
    """
    pts = [(float(x), float(y), float(z)) for (x, y, z) in loop]
    n = len(pts)
    if n < 2:
        raise ValidationError("section loop must contain at least 2 points")
    if count < 3:
        raise ValidationError("resample count must be >= 3")

    # Cumulative arc length around the closed loop.
    seg_lengths: list[float] = []
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        seg_lengths.append(float(np.linalg.norm(np.asarray(b) - np.asarray(a))))
    total = float(sum(seg_lengths))
    if total <= 1e-12:
        raise ValidationError("section loop has zero length")

    cumulative = [0.0]
    for length in seg_lengths:
        cumulative.append(cumulative[-1] + length)

    out: list[Vec3] = []
    for k in range(count):
        target = total * (float(k) / float(count))
        # Locate the segment containing ``target``.
        seg = 0
        while seg < n and cumulative[seg + 1] < target:
            seg += 1
        seg = min(seg, n - 1)
        seg_len = seg_lengths[seg]
        if seg_len <= 1e-12:
            out.append(pts[seg])
            continue
        local = (target - cumulative[seg]) / seg_len
        a = np.asarray(pts[seg])
        b = np.asarray(pts[(seg + 1) % n])
        p = a + (b - a) * local
        out.append((float(p[0]), float(p[1]), float(p[2])))
    return out


def _loop_centroid(loop: Sequence[Vec3]) -> Vec3:
    arr = np.asarray(loop, dtype=np.float64)
    c = arr.mean(axis=0)
    return (float(c[0]), float(c[1]), float(c[2]))


def _loop_normal(loop: Sequence[Vec3]) -> Vec3:
    """Best-fit plane normal of a loop via Newell's method."""
    pts = list(loop)
    n = len(pts)
    nx = ny = nz = 0.0
    for i in range(n):
        x0, y0, z0 = pts[i]
        x1, y1, z1 = pts[(i + 1) % n]
        nx += (y0 - y1) * (z0 + z1)
        ny += (z0 - z1) * (x0 + x1)
        nz += (x0 - x1) * (y0 + y1)
    norm = (nx * nx + ny * ny + nz * nz) ** 0.5
    if norm <= 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx / norm, ny / norm, nz / norm)


def _section_loops(spec: SectionLoftSpec) -> list[list[Vec3]]:
    """Return one closed 3D loop per user section."""
    loops: list[list[Vec3]] = []
    for section in spec.sections:
        loops.append([tuple(p) for p in section.points])
    return loops


def _smooth_loops(loops: Sequence[Sequence[Vec3]], *, ruled: bool) -> list[list[Vec3]]:
    """Insert intermediate cross-sections so a ruled loft looks smooth.

    All loops are first resampled to a common ring count. When ``ruled`` is
    False, intermediate sections are produced by sampling, per-ring, a
    Catmull-Rom spline through the corresponding vertices across sections.
    """
    ring_count = max(len(loop) for loop in loops)
    resampled = [_resample_loop(loop, ring_count) for loop in loops]

    if ruled or len(resampled) < 2 or _SMOOTH_SECTION_SUBDIV <= 1:
        return resampled

    # For each ring index, fit a spline through that vertex across all sections,
    # producing a denser stack of cross-sections.
    per_ring_paths: list[list[Vec3]] = []
    for j in range(ring_count):
        column = [resampled[i][j] for i in range(len(resampled))]
        if len(column) >= 2:
            sampled = sample_catmull_rom_spline_3d(
                column,
                samples_per_segment=int(_SMOOTH_SECTION_SUBDIV),
                closed=False,
                alpha=0.5,
            )
        else:
            sampled = column
        per_ring_paths.append(sampled)

    section_count = min(len(path) for path in per_ring_paths)
    dense: list[list[Vec3]] = []
    for s in range(section_count):
        dense.append([per_ring_paths[j][s] for j in range(ring_count)])
    return dense


def _fan_cap_faces(
    geometry: MeshGeometry,
    loop_indices: Sequence[int],
    loop_points: Sequence[Vec3],
    *,
    flip: bool,
) -> None:
    """Add a centroid-fan triangulation closing a loop of existing vertices."""
    centroid = _loop_centroid(loop_points)
    center_idx = geometry.add_vertex(*centroid)
    n = len(loop_indices)
    for i in range(n):
        a = loop_indices[i]
        b = loop_indices[(i + 1) % n]
        if flip:
            geometry.add_face(center_idx, b, a)
        else:
            geometry.add_face(center_idx, a, b)


def _skin_loops(
    loops: Sequence[Sequence[Vec3]],
    *,
    cap: bool,
) -> MeshGeometry:
    """Skin a stack of equal-length closed 3D loops into a solid mesh.

    This is a general (orientation-agnostic) replacement for the OCC loft. It
    builds quad side walls between consecutive loops and optional centroid-fan
    end caps. The result is run through mesh repair downstream to weld/clean.
    """
    if len(loops) < 2:
        raise ValidationError("loft requires at least two sections")
    ring_count = len(loops[0])
    if ring_count < 3:
        raise ValidationError("loft sections must contain at least 3 points")
    for loop in loops:
        if len(loop) != ring_count:
            raise ValidationError("loft sections must share a common ring count")

    geometry = MeshGeometry()
    ring_offsets: list[int] = []
    for loop in loops:
        ring_offsets.append(len(geometry.vertices))
        for x, y, z in loop:
            geometry.add_vertex(float(x), float(y), float(z))

    for i in range(len(loops) - 1):
        o0 = ring_offsets[i]
        o1 = ring_offsets[i + 1]
        for j in range(ring_count):
            j2 = (j + 1) % ring_count
            a = o0 + j
            b = o0 + j2
            c = o1 + j2
            d = o1 + j
            geometry.add_face(a, b, c)
            geometry.add_face(a, c, d)

    if cap:
        first_indices = [ring_offsets[0] + j for j in range(ring_count)]
        last_indices = [ring_offsets[-1] + j for j in range(ring_count)]
        _fan_cap_faces(geometry, first_indices, list(loops[0]), flip=True)
        _fan_cap_faces(geometry, last_indices, list(loops[-1]), flip=False)

    return geometry


def _build_planar_loft(loops: Sequence[Sequence[Vec3]], *, cap: bool) -> MeshGeometry:
    """Build a loft using the native ``LoftGeometry`` when sections are planar
    constant-z stacks; otherwise fall back to the general skinner."""
    planar = True
    for loop in loops:
        zs = [p[2] for p in loop]
        if max(zs) - min(zs) > 1e-6:
            planar = False
            break
    if planar:
        try:
            loft = LoftGeometry(loops, cap=bool(cap), closed=True)
            return MeshGeometry(
                vertices=[tuple(v) for v in loft.vertices],
                faces=[tuple(f) for f in loft.faces],
            )
        except Exception:
            pass
    return _skin_loops(loops, cap=cap)


# ---------------------------------------------------------------------------
# Path / sweep handling
# ---------------------------------------------------------------------------


def _sample_path(points: Sequence[Vec3], *, name: str) -> list[Vec3]:
    pts = [(float(x), float(y), float(z)) for (x, y, z) in points]
    if len(pts) < 2:
        raise ValidationError(f"{name} must contain at least 2 points")
    if len(pts) == 2:
        return pts
    # Try a smooth Catmull-Rom fit; fall back to the raw polyline.
    with suppress(Exception):
        sampled = sample_catmull_rom_spline_3d(
            pts,
            samples_per_segment=int(_PATH_SAMPLES_PER_SEGMENT),
            closed=False,
            alpha=0.5,
        )
        if len(sampled) >= 2:
            return sampled
    with suppress(Exception):
        sampled = sample_cubic_bezier_spline_3d(
            pts, samples_per_segment=int(_PATH_SAMPLES_PER_SEGMENT)
        )
        if len(sampled) >= 2:
            return sampled
    return pts


def _frames_along_path(
    path: Sequence[Vec3],
    *,
    aux_path: Optional[Sequence[Vec3]],
) -> list[tuple[Vec3, Vec3, Vec3]]:
    """Return (tangent, normal, binormal) frames at each path station.

    Uses a parallel-transport style frame. When ``aux_path`` is provided, the
    binormal direction is biased toward (aux_station - station) to emulate the
    OCC ``mode`` auxiliary spine.
    """
    pts = [np.asarray(p, dtype=np.float64) for p in path]
    n = len(pts)
    tangents: list[np.ndarray] = []
    for i in range(n):
        if i == 0:
            t = pts[1] - pts[0]
        elif i == n - 1:
            t = pts[-1] - pts[-2]
        else:
            t = pts[i + 1] - pts[i - 1]
        norm = np.linalg.norm(t)
        if norm <= 1e-12:
            t = np.asarray([0.0, 0.0, 1.0])
            norm = 1.0
        tangents.append(t / norm)

    frames: list[tuple[Vec3, Vec3, Vec3]] = []
    # Seed reference up vector.
    up = np.asarray([0.0, 1.0, 0.0])
    if abs(float(np.dot(up, tangents[0]))) > 0.9:
        up = np.asarray([1.0, 0.0, 0.0])

    prev_normal: Optional[np.ndarray] = None
    for i in range(n):
        t = tangents[i]
        if aux_path is not None and i < len(aux_path):
            aim = np.asarray(aux_path[i], dtype=np.float64) - pts[i]
            aim = aim - t * float(np.dot(aim, t))
            if np.linalg.norm(aim) > 1e-9:
                binormal = aim / np.linalg.norm(aim)
                normal = np.cross(binormal, t)
                nn = np.linalg.norm(normal)
                if nn > 1e-9:
                    normal = normal / nn
                    binormal = np.cross(t, normal)
                    frames.append(
                        (
                            (float(t[0]), float(t[1]), float(t[2])),
                            (float(normal[0]), float(normal[1]), float(normal[2])),
                            (float(binormal[0]), float(binormal[1]), float(binormal[2])),
                        )
                    )
                    prev_normal = normal
                    continue

        if prev_normal is None:
            normal = up - t * float(np.dot(up, t))
            nn = np.linalg.norm(normal)
            normal = normal / nn if nn > 1e-9 else np.asarray([1.0, 0.0, 0.0])
        else:
            # Parallel transport the previous normal.
            normal = prev_normal - t * float(np.dot(prev_normal, t))
            nn = np.linalg.norm(normal)
            normal = normal / nn if nn > 1e-9 else prev_normal
        binormal = np.cross(t, normal)
        bn = np.linalg.norm(binormal)
        binormal = binormal / bn if bn > 1e-9 else binormal
        frames.append(
            (
                (float(t[0]), float(t[1]), float(t[2])),
                (float(normal[0]), float(normal[1]), float(normal[2])),
                (float(binormal[0]), float(binormal[1]), float(binormal[2])),
            )
        )
        prev_normal = normal
    return frames


def _section_local_profile(section: Sequence[Vec3]) -> tuple[Vec3, list[tuple[float, float]]]:
    """Express a section loop in its own plane as 2D (normal, binormal) coords.

    Returns the section centroid and a list of (u, v) coordinates where ``u`` is
    along the section's in-plane normal axis and ``v`` along its binormal axis.
    The section's plane normal is treated as the local tangent direction.
    """
    centroid = np.asarray(_loop_centroid(section), dtype=np.float64)
    plane_normal = np.asarray(_loop_normal(section), dtype=np.float64)
    # Build in-plane basis (u, v) spanning the section plane.
    ref = np.asarray([1.0, 0.0, 0.0])
    if abs(float(np.dot(ref, plane_normal))) > 0.9:
        ref = np.asarray([0.0, 1.0, 0.0])
    u_axis = ref - plane_normal * float(np.dot(ref, plane_normal))
    nu = np.linalg.norm(u_axis)
    u_axis = u_axis / nu if nu > 1e-9 else np.asarray([1.0, 0.0, 0.0])
    v_axis = np.cross(plane_normal, u_axis)

    coords: list[tuple[float, float]] = []
    for p in section:
        rel = np.asarray(p, dtype=np.float64) - centroid
        coords.append((float(np.dot(rel, u_axis)), float(np.dot(rel, v_axis))))
    return (
        (float(centroid[0]), float(centroid[1]), float(centroid[2])),
        coords,
    )


def _place_section_on_frame(
    coords: Sequence[tuple[float, float]],
    station: Vec3,
    frame: tuple[Vec3, Vec3, Vec3],
) -> list[Vec3]:
    """Place a 2D (u, v) section onto a path frame's (normal, binormal) plane."""
    _tangent, normal, binormal = frame
    origin = np.asarray(station, dtype=np.float64)
    n_axis = np.asarray(normal, dtype=np.float64)
    b_axis = np.asarray(binormal, dtype=np.float64)
    out: list[Vec3] = []
    for u, v in coords:
        p = origin + n_axis * float(u) + b_axis * float(v)
        out.append((float(p[0]), float(p[1]), float(p[2])))
    return out


def _build_swept_loft(
    spec: SectionLoftSpec,
    path_points: Sequence[Vec3],
    aux_points: Optional[Sequence[Vec3]],
) -> MeshGeometry:
    """Emulate ``Solid.sweep_multi``: distribute the section profiles along the
    sampled path and skin the resulting oriented cross-sections."""
    path = _sample_path(path_points, name="path")
    aux = _sample_path(aux_points, name="guide_curves['aux_spine']") if aux_points else None
    if aux is not None and len(aux) != len(path):
        # Resample aux to match the path station count.
        aux = _resample_polyline_to_count(aux, len(path))

    frames = _frames_along_path(path, aux_path=aux)

    # Normalize every user section to a shared ring count and express each in
    # local 2D coordinates so it can be re-placed on path frames.
    ring_count = max(len(s.points) for s in spec.sections)
    local_sections: list[list[tuple[float, float]]] = []
    for section in spec.sections:
        loop = _resample_loop([tuple(p) for p in section.points], ring_count)
        _centroid, coords = _section_local_profile(loop)
        local_sections.append(coords)

    n_stations = len(path)
    n_sections = len(local_sections)
    loops: list[list[Vec3]] = []
    for k in range(n_stations):
        # Map station index -> section index (interpolating section shape).
        t = (float(k) / float(n_stations - 1)) if n_stations > 1 else 0.0
        fpos = t * (n_sections - 1)
        i0 = int(np.floor(fpos))
        i0 = max(0, min(i0, n_sections - 1))
        i1 = min(i0 + 1, n_sections - 1)
        frac = fpos - i0
        coords = [
            (
                local_sections[i0][j][0] * (1.0 - frac) + local_sections[i1][j][0] * frac,
                local_sections[i0][j][1] * (1.0 - frac) + local_sections[i1][j][1] * frac,
            )
            for j in range(ring_count)
        ]
        loops.append(_place_section_on_frame(coords, path[k], frames[k]))

    return _skin_loops(loops, cap=bool(spec.cap and spec.solid))


def _resample_polyline_to_count(points: Sequence[Vec3], count: int) -> list[Vec3]:
    pts = [np.asarray(p, dtype=np.float64) for p in points]
    if len(pts) <= 1:
        return [tuple(float(c) for c in p) for p in pts]
    seg = [float(np.linalg.norm(pts[i + 1] - pts[i])) for i in range(len(pts) - 1)]
    cumulative = [0.0]
    for length in seg:
        cumulative.append(cumulative[-1] + length)
    total = cumulative[-1]
    if total <= 1e-12:
        return [tuple(float(c) for c in pts[0]) for _ in range(count)]
    out: list[Vec3] = []
    for k in range(count):
        target = total * (float(k) / float(max(count - 1, 1)))
        s = 0
        while s < len(seg) and cumulative[s + 1] < target:
            s += 1
        s = min(s, len(seg) - 1)
        seg_len = seg[s]
        local = (target - cumulative[s]) / seg_len if seg_len > 1e-12 else 0.0
        p = pts[s] + (pts[s + 1] - pts[s]) * local
        out.append((float(p[0]), float(p[1]), float(p[2])))
    return out


# ---------------------------------------------------------------------------
# Resolution / symmetry / repair
# ---------------------------------------------------------------------------


def _resolve_path_points(spec: SectionLoftSpec) -> Optional[tuple[Vec3, ...]]:
    if spec.path is not None:
        return spec.path
    if spec.guide_curves is None:
        return None
    spine = spec.guide_curves.get("spine")
    return spine


def _resolve_aux_spine_points(spec: SectionLoftSpec) -> Optional[tuple[Vec3, ...]]:
    if spec.guide_curves is None:
        return None
    aux = spec.guide_curves.get("aux_spine")
    if aux is not None:
        return aux
    return spec.guide_curves.get("binormal")


def _validate_guide_curve_names(spec: SectionLoftSpec) -> None:
    if spec.guide_curves is None:
        return
    supported = {"spine", "aux_spine", "binormal"}
    unsupported = sorted(set(spec.guide_curves) - supported)
    if unsupported:
        raise ValidationError(
            "Unsupported guide_curves keys: "
            + ", ".join(repr(name) for name in unsupported)
            + ". Supported keys are 'spine', 'aux_spine', and 'binormal'."
        )


def _mirror_geometry_yz(geometry: MeshGeometry) -> MeshGeometry:
    mirrored = MeshGeometry(
        vertices=[(-float(x), float(y), float(z)) for (x, y, z) in geometry.vertices],
        faces=[(int(a), int(c), int(b)) for (a, b, c) in geometry.faces],
    )
    return mirrored


def _apply_symmetry_geometry(
    geometry: MeshGeometry, *, symmetry: Optional[SymmetryMode]
) -> MeshGeometry:
    if symmetry is None:
        return geometry
    if symmetry != "mirror_yz":
        raise ValidationError(f"Unsupported symmetry mode: {symmetry!r}")

    mirrored = _mirror_geometry_yz(geometry)
    # Try a true solid union first; fall back to a naive merge if the boolean
    # backend cannot fuse the two halves.
    with suppress(Exception):
        return boolean_union(geometry, mirrored)
    fused = geometry.copy()
    fused.merge(mirrored)
    return fused


def _repair_mesh_geometry(geometry: MeshGeometry) -> MeshGeometry:
    trimesh = _require_trimesh()
    if not geometry.vertices or not geometry.faces:
        return geometry.copy()

    mesh = trimesh.Trimesh(
        vertices=np.asarray(geometry.vertices, dtype=np.float64),
        faces=np.asarray(geometry.faces, dtype=np.int64),
        process=False,
        validate=False,
    )
    mesh.process(validate=False)
    mesh.update_faces(mesh.unique_faces() & mesh.nondegenerate_faces())
    mesh.fill_holes()
    mesh.fix_normals(multibody=False)
    mesh.remove_unreferenced_vertices()

    repaired = MeshGeometry(
        vertices=[(float(v[0]), float(v[1]), float(v[2])) for v in mesh.vertices],
        faces=[(int(f[0]), int(f[1]), int(f[2])) for f in mesh.faces],
    )

    with suppress(Exception):
        repaired = _geometry_from_manifold(_manifold_from_geometry(repaired, name="geometry"))

    return repaired


def section_loft(
    spec: SectionLoftSpec | Sequence[LoftSection | Sequence[Sequence[float] | Vec3]],
    /,
    **overrides,
) -> MeshGeometry:
    if overrides:
        spec = SectionLoftSpec(**({**_coerce_loft_spec(spec).__dict__, **overrides}))
    else:
        spec = _coerce_loft_spec(spec)

    _validate_guide_curve_names(spec)
    path_points = _resolve_path_points(spec)
    aux_spine_points = _resolve_aux_spine_points(spec)

    if path_points is None:
        loops = _smooth_loops(_section_loops(spec), ruled=bool(spec.ruled))
        geometry = _build_planar_loft(loops, cap=bool(spec.cap and spec.solid))
    else:
        geometry = _build_swept_loft(spec, path_points, aux_spine_points)

    geometry = _apply_symmetry_geometry(geometry, symmetry=spec.symmetry)

    if spec.repair in {"auto", "mesh", "kernel"}:
        geometry = _repair_mesh_geometry(geometry)

    return geometry


def repair_loft(
    geometry_or_spec: MeshGeometry
    | SectionLoftSpec
    | Sequence[LoftSection | Sequence[Sequence[float] | Vec3]],
    /,
    *,
    repair: RepairMode = "auto",
) -> MeshGeometry:
    if isinstance(geometry_or_spec, MeshGeometry):
        if repair == "off":
            return geometry_or_spec.copy()
        return _repair_mesh_geometry(geometry_or_spec)

    spec = _coerce_loft_spec(geometry_or_spec)
    if repair != "auto":
        spec = SectionLoftSpec(**{**spec.__dict__, "repair": repair})
    return section_loft(spec)
