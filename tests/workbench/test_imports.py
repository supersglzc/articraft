from __future__ import annotations

from pathlib import Path

from engine.cli.workbench import main as workbench_main
from engine.storage.repo import StorageRepo


def test_workbench_imports() -> None:
    assert callable(workbench_main)
    repo = StorageRepo(Path.cwd())
    assert repo.layout.runs_root.name == "runs"
