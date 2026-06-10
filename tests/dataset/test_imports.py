from __future__ import annotations

from pathlib import Path

from engine.cli.dataset import main as dataset_main
from engine.storage.repo import StorageRepo


def test_dataset_imports() -> None:
    assert callable(dataset_main)
    repo = StorageRepo(Path.cwd())
    assert repo.layout.records_root.name == "records"
