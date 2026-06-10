from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from engine.storage.records_index import load_records_index
from engine.storage.repo import StorageRepo


@dataclass(slots=True)
class StorageQueries:
    repo: StorageRepo

    def list_record_ids(self) -> list[str]:
        record_ids = {
            str(row.get("record_id"))
            for row in load_records_index(self.repo)
            if isinstance(row.get("record_id"), str)
        }
        records_root = self.repo.layout.records_root
        if not records_root.exists():
            return sorted(record_ids)
        record_ids.update(path.name for path in records_root.iterdir() if path.is_dir())
        return sorted(record_ids)

    def list_category_slugs(self) -> list[str]:
        categories_root = self.repo.layout.categories_root
        if not categories_root.exists():
            return []
        return sorted(path.name for path in categories_root.iterdir() if path.is_dir())

    def list_record_ids_for_category(self, category_slug: str) -> list[str]:
        record_ids = {
            str(row.get("record_id"))
            for row in load_records_index(self.repo)
            if row.get("category_slug") == category_slug and isinstance(row.get("record_id"), str)
        }
        for record_id in self.list_record_ids():
            if record_id in record_ids:
                continue
            record = self.repo.read_json(self.repo.layout.record_metadata_path(record_id))
            if not isinstance(record, dict):
                continue
            if str(record.get("category_slug") or "") == category_slug:
                record_ids.add(record_id)
        return sorted(record_ids)

    def list_run_ids_for_category(self, category_slug: str) -> list[str]:
        runs_root = self.repo.layout.runs_root
        if not runs_root.exists():
            return []

        run_ids: list[str] = []
        for run_dir in sorted(path for path in runs_root.iterdir() if path.is_dir()):
            run = self.repo.read_json(self.repo.layout.run_metadata_path(run_dir.name))
            if not isinstance(run, dict):
                continue
            category_slug_value = str(run.get("category_slug") or "")
            category_slugs = run.get("category_slugs")
            if category_slug_value == category_slug:
                run_ids.append(run_dir.name)
                continue
            if isinstance(category_slugs, list) and category_slug in {
                str(value) for value in category_slugs if str(value).strip()
            }:
                run_ids.append(run_dir.name)
        return run_ids

    def record_dir(self, record_id: str) -> Path:
        return self.repo.layout.record_dir(record_id)
