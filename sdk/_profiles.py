"""Generator profiles.

A profile bundles everything that makes one *generator* (one authoring domain):
its SDK package name, the scaffold template, the docs that get mounted into the
agent's virtual workspace, and the designer system prompt. The runtime
(harness/compiler/storage) is domain-agnostic and reads only this profile, so a
second generator (e.g. rigid-body) is added by registering another profile here.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

# Single Gemini designer prompt (this project is Gemini-only).
DESIGNER_PROMPT_NAME = "designer_system_prompt_gemini.txt"


@dataclass(slots=True, frozen=True)
class SdkProfile:
    package_name: str
    scaffold_path: Path
    docs_full: tuple[Path, ...]
    docs_core: tuple[Path, ...]
    designer_prompt_name: str

    def docs_for_mode(self, docs_mode: str) -> tuple[Path, ...]:
        if docs_mode == "full":
            return self.docs_full
        if docs_mode == "core":
            return self.docs_core
        if docs_mode == "none":
            return ()
        raise ValueError(f"Unsupported SDK docs mode: {docs_mode!r}")

    def prompt_name_for_provider(self, provider: str | None = None) -> str:
        # Gemini is the only provider; the argument is accepted for call-site stability.
        return self.designer_prompt_name


_COMMON_DOCS = (
    Path("sdk/_docs/common/00_quickstart.md"),
    Path("sdk/_docs/common/10_errors.md"),
    Path("sdk/_docs/common/20_core_types.md"),
    Path("sdk/_docs/common/30_articulated_object.md"),
    Path("sdk/_docs/common/40_assets.md"),
    Path("sdk/_docs/common/50_placement.md"),
    Path("sdk/_docs/common/70_probe_tooling.md"),
    Path("sdk/_docs/common/80_testing.md"),
)

_BASE_DOCS = (
    Path("sdk/_docs/base/40_mesh_geometry.md"),
    Path("sdk/_docs/base/41_panels_and_grilles.md"),
    Path("sdk/_docs/base/42_brackets_and_mounts.md"),
    Path("sdk/_docs/base/43_fans_and_rotors.md"),
    Path("sdk/_docs/base/44_knobs_and_controls.md"),
    Path("sdk/_docs/base/45_wires.md"),
    Path("sdk/_docs/base/46_section_lofts.md"),
    Path("sdk/_docs/base/47_bezels_and_frames.md"),
    Path("sdk/_docs/base/48_wheels_and_tires.md"),
    Path("sdk/_docs/base/49_hinges.md"),
    Path("sdk/_docs/base/50_gears.md"),
    Path("sdk/_docs/base/51_positioning_and_transforms.md"),
)

_SDK_PACKAGE_ALIASES = {
    "": "sdk",
    "base": "sdk",
    "sdk": "sdk",
}


SDK_PROFILES: dict[str, SdkProfile] = {
    "sdk": SdkProfile(
        package_name="sdk",
        scaffold_path=Path("sdk/scaffold.py"),
        docs_full=_COMMON_DOCS[:4] + _BASE_DOCS + _COMMON_DOCS[4:],
        docs_core=(
            Path("sdk/_docs/common/00_quickstart.md"),
            Path("sdk/_docs/common/70_probe_tooling.md"),
            Path("sdk/_docs/common/80_testing.md"),
        ),
        designer_prompt_name=DESIGNER_PROMPT_NAME,
    ),
}


SUPPORTED_SDK_PACKAGES = frozenset(SDK_PROFILES)


def get_sdk_profile(package_name: str) -> SdkProfile:
    normalized = _SDK_PACKAGE_ALIASES.get(str(package_name or "").strip().lower())
    if normalized is None:
        raise ValueError(
            f"Unsupported SDK package: {package_name!r}. "
            f"Expected one of {sorted(SUPPORTED_SDK_PACKAGES)!r}."
        )
    try:
        return SDK_PROFILES[normalized]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported SDK package: {package_name!r}. "
            f"Expected one of {sorted(SUPPORTED_SDK_PACKAGES)!r}."
        ) from exc
