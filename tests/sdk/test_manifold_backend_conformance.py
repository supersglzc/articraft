"""Conformance test for the active Manifold backend.

The SDK depends on a small, fixed slice of the ``manifold3d`` Python API. This
test exercises exactly that surface against whatever backend ``_manifold_backend``
resolves to (the real ``manifold3d`` wheel by default, or a C-library adapter when
``ARTICRAFT_MANIFOLD_BACKEND=c`` / the wheel is absent).

Run it after wiring a C adapter to confirm the binding satisfies the SDK's needs:

    ARTICRAFT_MANIFOLD_BACKEND=c uv run --group dev pytest \
        tests/sdk/test_manifold_backend_conformance.py -q

The OPTIONAL surface (CrossSection, triangulate) is intentionally NOT required
here: ``common.py`` ear-clips when ``triangulate`` returns None, and the planar
reconstruction path is only reached without libigl present.
"""

from __future__ import annotations

import numpy as np

from sdk._core.v0._mesh._manifold_backend import m3d


def _unit_cube_mesh(size: float = 1.0):
    """A closed axis-aligned cube as a (verts, faces) mesh for Manifold(mesh)."""
    s = size / 2.0
    verts = np.array(
        [
            (-s, -s, -s),
            (s, -s, -s),
            (s, s, -s),
            (-s, s, -s),
            (-s, -s, s),
            (s, -s, s),
            (s, s, s),
            (-s, s, s),
        ],
        dtype=float,
    )
    faces = np.array(
        [
            (0, 3, 2),
            (0, 2, 1),
            (4, 5, 6),
            (4, 6, 7),
            (0, 1, 5),
            (0, 5, 4),
            (2, 3, 7),
            (2, 7, 6),
            (1, 2, 6),
            (1, 6, 5),
            (0, 4, 7),
            (0, 7, 3),
        ],
        dtype=np.int64,
    )
    return verts, faces


def test_backend_has_required_surface() -> None:
    for name in ("Manifold", "Mesh", "Error"):
        assert hasattr(m3d, name), f"backend missing {name}"
    assert hasattr(m3d.Manifold, "cube"), "Manifold.cube missing"
    assert hasattr(m3d.Error, "NoError"), "Error.NoError missing"


def test_construct_status_and_export() -> None:
    verts, faces = _unit_cube_mesh()
    mesh = m3d.Mesh(verts.astype(np.float32), faces.astype(np.uint32))
    man = m3d.Manifold(mesh)

    assert man.status() == m3d.Error.NoError
    assert not man.is_empty()
    assert float(man.get_tolerance()) >= 0.0

    out = man.to_mesh()
    vp = np.asarray(out.vert_properties)
    tv = np.asarray(out.tri_verts)
    assert vp.ndim == 2 and vp.shape[1] >= 3, "vert_properties must be (N, >=3)"
    assert tv.ndim == 2 and tv.shape[1] == 3, "tri_verts must be (M, 3)"
    assert len(tv) >= 12, "a cube should triangulate to at least 12 tris"


def test_empty_manifold() -> None:
    empty = m3d.Manifold()
    assert empty.is_empty()


def test_cube_factory_and_translate() -> None:
    cube = m3d.Manifold.cube((1.0, 1.0, 1.0), center=True)
    assert not cube.is_empty()
    moved = cube.translate((10.0, 0.0, 0.0))
    bb = list(moved.bounding_box())
    assert len(bb) == 6
    # the translated cube's x-extent should straddle ~10
    assert bb[0] > 9.0 and bb[3] < 11.0


def test_boolean_operators() -> None:
    a = m3d.Manifold.cube((2.0, 2.0, 2.0), center=True)
    b = m3d.Manifold.cube((2.0, 2.0, 2.0), center=True).translate((1.0, 0.0, 0.0))

    union = a + b
    diff = a - b
    inter = a ^ b
    for result in (union, diff, inter):
        assert result.status() == m3d.Error.NoError
        assert not result.is_empty()

    def volume(man) -> float:
        out = man.to_mesh()
        v = np.asarray(out.vert_properties)[:, :3]
        f = np.asarray(out.tri_verts)
        tris = v[f]
        return float(
            abs(np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum()) / 6.0
        )

    # union >= each operand >= intersection (overlapping equal cubes)
    assert volume(union) > volume(inter)
    assert volume(diff) > 0.0
