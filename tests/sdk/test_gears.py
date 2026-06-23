from __future__ import annotations

import importlib

import pytest

import sdk


@pytest.mark.parametrize(
    ("gear_ctor", "kwargs"),
    [
        ("SpurGear", {"module": 0.5, "teeth_number": 12, "width": 3.0, "bore_d": 2.0}),
        ("RingGear", {"module": 0.5, "teeth_number": 24, "width": 3.0, "rim_width": 1.5}),
        (
            "HerringbonePlanetaryGearset",
            {
                "module": 0.5,
                "sun_teeth_number": 8,
                "planet_teeth_number": 6,
                "width": 3.0,
                "rim_width": 1.0,
                "n_planets": 2,
                "helix_angle": 15.0,
            },
        ),
        (
            "BevelGearPair",
            {
                "module": 0.5,
                "gear_teeth": 12,
                "pinion_teeth": 8,
                "face_width": 2.0,
                "axis_angle": 90.0,
            },
        ),
        ("RackGear", {"module": 0.5, "length": 10.0, "width": 3.0, "height": 2.0}),
        ("Worm", {"module": 0.5, "lead_angle": 20.0, "n_threads": 1, "length": 8.0}),
        (
            "CrossedGearPair",
            {
                "module": 0.5,
                "gear1_teeth_number": 12,
                "gear2_teeth_number": 12,
                "gear1_width": 2.0,
                "gear2_width": 2.0,
                "shaft_angle": 90.0,
                "gear1_helix_angle": 30.0,
            },
        ),
        (
            "HyperbolicGearPair",
            {"module": 0.5, "gear1_teeth_number": 12, "width": 2.0, "shaft_angle": 40.0},
        ),
    ],
)
def test_representative_vendored_gears_build(
    gear_ctor: str,
    kwargs: dict[str, float | int],
) -> None:
    """Each gear is a native MeshGeometry and builds a non-empty body (no CadQuery)."""
    gear_cls = getattr(sdk, gear_ctor)
    gear = gear_cls(**kwargs)

    assert isinstance(gear, sdk.MeshGeometry)
    body = gear.build()
    assert isinstance(body, sdk.MeshGeometry)
    assert len(body.vertices) > 0 and len(body.faces) > 0


def test_top_level_gear_helper_merges_onto_target() -> None:
    """``sdk.gear`` merges a gear onto a MeshGeometry target (native, no CadQuery plugin)."""
    spur = sdk.SpurGear(module=0.5, teeth_number=12, width=3.0, bore_d=2.0)
    merged = sdk.gear(sdk.BoxGeometry((10.0, 10.0, 1.0)), spur)
    assert isinstance(merged, sdk.MeshGeometry)
    assert len(merged.vertices) > 0


def test_sdk_gears_module_reexports_classes() -> None:
    from sdk.v0.gears import RingGear, SpurGear

    current_sdk = importlib.import_module("sdk")

    assert SpurGear.__name__ == current_sdk.SpurGear.__name__
    assert RingGear.__name__ == current_sdk.RingGear.__name__
