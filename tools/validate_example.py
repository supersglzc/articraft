"""Validate a base-SDK example .md by executing its python fence and running the
same SDK checks as tests/sdk/test_base_example_derivations.py.

Usage:
    uv run python tools/validate_example.py sdk/_examples/base/<name>.md

Exit 0 + "OK" on success; exit 1 + the error on failure.
"""

from __future__ import annotations

import re
import sys
import tempfile
from pathlib import Path

import sdk

_PYTHON_FENCE_RE = re.compile(r"```python\n(.*?)```", re.DOTALL)


def main(rel_or_abs: str) -> int:
    path = Path(rel_or_abs).resolve()
    if not path.exists():
        print(f"FAIL: file not found: {path}")
        return 1
    text = path.read_text(encoding="utf-8")
    match = _PYTHON_FENCE_RE.search(text)
    if match is None:
        print(f"FAIL: no ```python fence in {path}")
        return 1
    code = match.group(1)
    with tempfile.TemporaryDirectory() as td:
        namespace = {"__file__": str(Path(td) / f"{path.stem}.py")}
        try:
            exec(code, namespace)
            if "object_model" not in namespace:
                print("FAIL: script did not define `object_model`")
                return 1
            object_model = namespace["object_model"]
            ctx = sdk.TestContext(object_model)
            ctx.check_model_valid()
            ctx.check_mesh_assets_ready()
        except Exception as exc:  # noqa: BLE001
            import traceback

            print(f"FAIL: {type(exc).__name__}: {exc}")
            traceback.print_exc()
            return 1
    print(f"OK: {path.name}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: validate_example.py <path-to-example.md>")
        raise SystemExit(2)
    raise SystemExit(main(sys.argv[1]))
