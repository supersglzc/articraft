"""Single indirection point for the Manifold binding.

The SDK uses only a small slice of the ``manifold3d`` Python API (see
``_manifold_c_adapter`` for the exact surface). Prefer the real ``manifold3d``
PyPI wheel; fall back to the C-library adapter when the wheel is unavailable.

All SDK mesh modules import the backend from here:

    from ._manifold_backend import m3d as _m3d

so swapping bindings is a one-line concern instead of a per-module edit.
"""

from __future__ import annotations

import os

# ARTICRAFT_MANIFOLD_BACKEND=c forces the C adapter even if the wheel is present
# (useful for exercising the adapter in CI).
_force = os.environ.get("ARTICRAFT_MANIFOLD_BACKEND", "").strip().lower()

if _force == "c":
    from . import _manifold_c_adapter as m3d  # noqa: F401
else:
    try:
        import manifold3d as m3d  # noqa: F401
    except ImportError:  # no wheel: use the C-library adapter
        from . import _manifold_c_adapter as m3d  # noqa: F401

__all__ = ["m3d"]
