"""Adapter that presents the ``manifold3d`` Python surface used by the SDK on top
of the Manifold **C** library (``manifoldc.h``), for environments that ship the C
binding instead of the ``manifold3d`` PyPI wheel.

This is a SKELETON. It encodes the exact API surface the SDK depends on (see
``test_manifold_backend_conformance.py``), the memory-management pattern the C
API requires, and the MeshGL <-> numpy round-trip. Spots that must be checked
against your specific build are marked ``# VERIFY``. Once the core methods are
wired to your library, the SDK runs unchanged via ``_manifold_backend``.

Recommended instead: install the ``manifold3d`` PyPI wheel (exact API match,
zero adapter). Use this only when the wheel is unavailable.

Surface required by the SDK (used only in ``booleans.py`` and ``common.py``):

  Manifold()                       -> empty solid
  Manifold(mesh)                   -> from a Mesh
  Manifold.cube((x,y,z), center=)  -> axis-aligned box
  a + b / a - b / a ^ b            -> union / difference / intersection
  m.translate((x, y, z))           -> translated copy        (no rotate needed)
  m.status() == Error.NoError      -> validity check
  m.is_empty()                     -> bool
  m.get_tolerance()                -> float
  m.bounding_box()                 -> (minx,miny,minz,maxx,maxy,maxz)
  m.to_mesh()                      -> object with .vert_properties (N x >=3 float),
                                                  .tri_verts (M x 3 uint32)
  Mesh(verts, faces)               -> input mesh
  CrossSection(polys, FillRule.Positive) / .simplify / .decompose / .to_polygons
                                   -> OPTIONAL (planar fallback only)
  triangulate(polys, epsilon=)     -> OPTIONAL (common.py ear-clips if this is None)
  Error.NoError                    -> status sentinel
"""

from __future__ import annotations

import ctypes
import os
from ctypes import POINTER, c_double, c_float, c_int, c_size_t, c_uint32, c_void_p
from enum import IntEnum
from typing import Optional, Sequence

import numpy as np

# --------------------------------------------------------------------------- #
# Library loading (lazy; import stays cheap so the real wheel can be preferred)
# --------------------------------------------------------------------------- #
_LIB: Optional[ctypes.CDLL] = None


def _lib() -> ctypes.CDLL:
    """Load and memoize the Manifold C shared library.

    Set ``MANIFOLD_C_LIB`` to the absolute path of your built ``libmanifoldc``
    (``.so`` / ``.dylib`` / ``.dll``). # VERIFY: name/path for your build.
    """
    global _LIB
    if _LIB is not None:
        return _LIB
    path = os.environ.get("MANIFOLD_C_LIB")
    if not path:
        raise RuntimeError(
            "Manifold C library not found. Set MANIFOLD_C_LIB to the path of your "
            "libmanifoldc shared object, or install the manifold3d Python wheel."
        )
    lib = ctypes.CDLL(path)
    _bind_signatures(lib)
    _LIB = lib
    return lib


def _bind_signatures(lib: ctypes.CDLL) -> None:
    """Declare argtypes/restypes for the C entry points the adapter uses.

    Symbol names follow upstream ``manifoldc.h``. # VERIFY each against your build.
    """
    # sizes for caller-allocated buffers (manifoldc uses caller-managed memory)
    for fn in ("manifold_manifold_size", "manifold_meshgl_size", "manifold_box_size"):
        getattr(lib, fn).restype = c_size_t
        getattr(lib, fn).argtypes = []

    lib.manifold_empty.restype = c_void_p
    lib.manifold_empty.argtypes = [c_void_p]

    lib.manifold_cube.restype = c_void_p
    lib.manifold_cube.argtypes = [c_void_p, c_double, c_double, c_double, c_int]

    for op in ("manifold_union", "manifold_difference", "manifold_intersection"):
        getattr(lib, op).restype = c_void_p
        getattr(lib, op).argtypes = [c_void_p, c_void_p, c_void_p]

    lib.manifold_translate.restype = c_void_p
    lib.manifold_translate.argtypes = [c_void_p, c_void_p, c_double, c_double, c_double]

    lib.manifold_is_empty.restype = c_int
    lib.manifold_is_empty.argtypes = [c_void_p]

    lib.manifold_status.restype = c_int
    lib.manifold_status.argtypes = [c_void_p]

    lib.manifold_get_tolerance.restype = c_double  # VERIFY: may be manifold_precision
    lib.manifold_get_tolerance.argtypes = [c_void_p]

    # MeshGL construction + accessors
    lib.manifold_meshgl.restype = c_void_p
    lib.manifold_meshgl.argtypes = [
        c_void_p,
        POINTER(c_float),
        c_size_t,
        c_size_t,
        POINTER(c_uint32),
        c_size_t,
    ]
    lib.manifold_of_meshgl.restype = c_void_p
    lib.manifold_of_meshgl.argtypes = [c_void_p, c_void_p]
    lib.manifold_get_meshgl.restype = c_void_p
    lib.manifold_get_meshgl.argtypes = [c_void_p, c_void_p]

    for fn in ("manifold_meshgl_num_vert", "manifold_meshgl_num_tri", "manifold_meshgl_num_prop"):
        getattr(lib, fn).restype = c_size_t
        getattr(lib, fn).argtypes = [c_void_p]
    lib.manifold_meshgl_vert_properties.restype = POINTER(c_float)
    lib.manifold_meshgl_vert_properties.argtypes = [c_void_p, c_void_p]
    lib.manifold_meshgl_tri_verts.restype = POINTER(c_uint32)
    lib.manifold_meshgl_tri_verts.argtypes = [c_void_p, c_void_p]

    # bounding box
    lib.manifold_bounding_box.restype = c_void_p
    lib.manifold_bounding_box.argtypes = [c_void_p, c_void_p]
    lib.manifold_box_min.restype = None  # VERIFY: returns ManifoldVec3 by value in some builds
    lib.manifold_box_max.restype = None

    # deleters
    for fn in ("manifold_delete_manifold", "manifold_delete_meshgl", "manifold_delete_box"):
        getattr(lib, fn).restype = None
        getattr(lib, fn).argtypes = [c_void_p]


def _alloc(size_fn: str) -> c_void_p:
    """Allocate a caller-managed buffer of the size the C API requires."""
    size = getattr(_lib(), size_fn)()
    buf = ctypes.create_string_buffer(size)
    return ctypes.cast(buf, c_void_p), buf  # keep buf alive with the handle


# --------------------------------------------------------------------------- #
# Status / FillRule enums (match manifold3d names the SDK references)
# --------------------------------------------------------------------------- #
class Error(IntEnum):
    NoError = 0  # VERIFY: ManifoldError.NoError ordinal in your header
    # Other statuses are not compared by name in the SDK; only NoError is used.


class FillRule(IntEnum):
    EvenOdd = 0
    NonZero = 1
    Positive = 2  # VERIFY ordinal
    Negative = 3


# --------------------------------------------------------------------------- #
# MeshGL view returned by Manifold.to_mesh()
# --------------------------------------------------------------------------- #
class _MeshGLView:
    """Holds the numpy arrays the SDK reads as ``.vert_properties`` / ``.tri_verts``."""

    __slots__ = ("vert_properties", "tri_verts")

    def __init__(self, vert_properties: np.ndarray, tri_verts: np.ndarray) -> None:
        self.vert_properties = vert_properties  # (N, num_prop) float32, cols 0:3 = xyz
        self.tri_verts = tri_verts  # (M, 3) uint32


# --------------------------------------------------------------------------- #
# Mesh: input geometry handed to Manifold(mesh)
# --------------------------------------------------------------------------- #
class Mesh:
    """Input mesh wrapper (verts (N,3) float, faces (M,3) int)."""

    def __init__(self, verts, faces) -> None:
        self._verts = np.ascontiguousarray(verts, dtype=np.float32)
        self._faces = np.ascontiguousarray(faces, dtype=np.uint32)
        if self._verts.ndim != 2 or self._verts.shape[1] != 3:
            raise ValueError("verts must be (N, 3)")
        if self._faces.ndim != 2 or self._faces.shape[1] != 3:
            raise ValueError("faces must be (M, 3)")

    def _to_c_meshgl(self) -> c_void_p:
        lib = _lib()
        mem, _buf = _alloc("manifold_meshgl_size")
        vp = self._verts.ravel()
        tv = self._faces.ravel()
        handle = lib.manifold_meshgl(
            mem,
            vp.ctypes.data_as(POINTER(c_float)),
            c_size_t(self._verts.shape[0]),
            c_size_t(3),
            tv.ctypes.data_as(POINTER(c_uint32)),
            c_size_t(self._faces.shape[0]),
        )
        return handle, _buf, vp, tv  # keep buffers alive until of_meshgl consumes them


# --------------------------------------------------------------------------- #
# Manifold: the solid
# --------------------------------------------------------------------------- #
class Manifold:
    """Solid wrapper over a ``ManifoldManifold*`` handle with lifetime management."""

    def __init__(self, mesh: Optional[Mesh] = None, _handle: Optional[c_void_p] = None) -> None:
        self._keep = []  # buffers that must outlive the handle
        if _handle is not None:
            self._h = _handle
            return
        lib = _lib()
        if mesh is None:
            mem, buf = _alloc("manifold_manifold_size")
            self._keep.append(buf)
            self._h = lib.manifold_empty(mem)
        else:
            mgl, mbuf, vp, tv = mesh._to_c_meshgl()
            mem, buf = _alloc("manifold_manifold_size")
            self._keep += [mbuf, vp, tv, buf]
            self._h = lib.manifold_of_meshgl(mem, mgl)
            lib.manifold_delete_meshgl(mgl)

    # ---- factories -------------------------------------------------------- #
    @staticmethod
    def cube(size: Sequence[float], center: bool = False) -> "Manifold":
        lib = _lib()
        mem, buf = _alloc("manifold_manifold_size")
        h = lib.manifold_cube(
            mem, c_double(size[0]), c_double(size[1]), c_double(size[2]), c_int(1 if center else 0)
        )
        m = Manifold(_handle=h)
        m._keep.append(buf)
        return m

    # ---- boolean operators ------------------------------------------------ #
    def _binop(self, other: "Manifold", c_fn: str) -> "Manifold":
        lib = _lib()
        mem, buf = _alloc("manifold_manifold_size")
        h = getattr(lib, c_fn)(mem, self._h, other._h)
        out = Manifold(_handle=h)
        out._keep.append(buf)
        return out

    def __add__(self, other: "Manifold") -> "Manifold":
        return self._binop(other, "manifold_union")

    def __sub__(self, other: "Manifold") -> "Manifold":
        return self._binop(other, "manifold_difference")

    def __xor__(self, other: "Manifold") -> "Manifold":
        return self._binop(other, "manifold_intersection")

    # ---- transforms ------------------------------------------------------- #
    def translate(self, vec: Sequence[float]) -> "Manifold":
        lib = _lib()
        mem, buf = _alloc("manifold_manifold_size")
        h = lib.manifold_translate(
            mem, self._h, c_double(vec[0]), c_double(vec[1]), c_double(vec[2])
        )
        out = Manifold(_handle=h)
        out._keep.append(buf)
        return out

    # ---- queries ---------------------------------------------------------- #
    def is_empty(self) -> bool:
        return bool(_lib().manifold_is_empty(self._h))

    def status(self) -> int:
        return int(_lib().manifold_status(self._h))

    def get_tolerance(self) -> float:
        return float(_lib().manifold_get_tolerance(self._h))

    def bounding_box(self):
        # Pattern: mem, _ = _alloc("manifold_box_size");
        #          box = _lib().manifold_bounding_box(mem, self._h)
        # then read manifold_box_min(box) / manifold_box_max(box).
        # VERIFY box min/max extraction for your build; return a 6-sequence
        # (minx, miny, minz, maxx, maxy, maxz) — the SDK indexes [0..5].
        raise NotImplementedError(
            "bounding_box: wire manifold_box_min/max for your build (return 6-tuple)"
        )

    # ---- export ----------------------------------------------------------- #
    def to_mesh(self) -> _MeshGLView:
        lib = _lib()
        mem, buf = _alloc("manifold_meshgl_size")
        mgl = lib.manifold_get_meshgl(mem, self._h)
        n_vert = lib.manifold_meshgl_num_vert(mgl)
        n_prop = lib.manifold_meshgl_num_prop(mgl)
        n_tri = lib.manifold_meshgl_num_tri(mgl)
        vp_ptr = lib.manifold_meshgl_vert_properties(
            c_void_p(), mgl
        )  # VERIFY out-buffer convention
        tv_ptr = lib.manifold_meshgl_tri_verts(c_void_p(), mgl)
        vp = np.ctypeslib.as_array(vp_ptr, shape=(n_vert, n_prop)).copy()
        tv = np.ctypeslib.as_array(tv_ptr, shape=(n_tri, 3)).astype(np.int64)
        lib.manifold_delete_meshgl(mgl)
        return _MeshGLView(vp, tv)

    def __del__(self) -> None:
        try:
            if getattr(self, "_h", None):
                _lib().manifold_delete_manifold(self._h)
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# OPTIONAL surface (planar-fallback path only). With libigl present the SDK's
# exact-CGAL fallback handles dirty meshes, so these are rarely exercised.
# Leave as NotImplementedError until/unless the planar path is needed.
# --------------------------------------------------------------------------- #
class CrossSection:
    def __init__(self, polygons, fill_rule: int = FillRule.Positive) -> None:
        raise NotImplementedError(
            "CrossSection is only used by the planar reconstruction fallback. "
            "Wire manifold_cross_section_of_polygons / _simplify / _decompose / "
            "_to_polygons if you need it; otherwise the libigl/CGAL path covers "
            "dirty-mesh booleans."
        )


def triangulate(polygons, epsilon: float = 0.0):
    """Optional: ``common.py`` falls back to an ear-clip triangulator when this
    returns ``None``, so a stub is acceptable."""
    return None
