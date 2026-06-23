from __future__ import annotations

import importlib
import sys

import pytest


def test_sdk_gears_work_without_cadquery(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Gears are native mesh geometry now — they must import and build with CadQuery absent."""
    real_import_module = importlib.import_module

    def fake_import_module(name: str, package: str | None = None):
        if name == "cadquery" or name.startswith("cadquery."):
            raise ModuleNotFoundError("No module named 'cadquery'")
        return real_import_module(name, package)

    monkeypatch.setattr(importlib, "import_module", fake_import_module)

    original_sdk_modules = {
        name: module
        for name, module in sys.modules.items()
        if name == "sdk" or name.startswith("sdk.")
    }
    try:
        for name in list(original_sdk_modules):
            sys.modules.pop(name, None)

        module = importlib.import_module("sdk")
        gear = module.SpurGear(module=0.5, teeth_number=12, width=3.0, bore_d=2.0)
        assert isinstance(gear, module.MeshGeometry)
        assert len(gear.vertices) > 0 and len(gear.faces) > 0
    finally:
        for name in list(sys.modules):
            if name == "sdk" or name.startswith("sdk."):
                sys.modules.pop(name, None)
        sys.modules.update(original_sdk_modules)
