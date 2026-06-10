from __future__ import annotations

import json
from pathlib import Path

import pytest

from engine.agent import runner


def test_runner_help_text(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit, match="0"):
        runner.main(["--help"])

    help_text = capsys.readouterr().out
    assert "--prompt" in help_text
    assert "--image" in help_text
    assert "--provider {gemini}" in help_text
    assert "--openai-transport {http,websocket}" in help_text
    assert "--collection {workbench,dataset}" in help_text
    assert "--dataset-id DATASET_ID" in help_text
    assert "--max-cost-usd MAX_COST_USD" in help_text
    assert "--flash" not in help_text
    assert "--hybrid-sdk" not in help_text
    assert "--openai-reasoning-summary" not in help_text
    assert "Currently supported only with --provider openai." not in help_text


def test_runner_rejects_removed_flash_flag(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit, match="2"):
        runner.main(["--prompt", "test", "--flash"])

    assert "unrecognized arguments: --flash" in capsys.readouterr().err


def test_runner_requires_dataset_id_for_dataset_collection(
    capsys: pytest.CaptureFixture[str],
) -> None:
    with pytest.raises(SystemExit, match="2"):
        runner.main(["--prompt", "test", "--collection", "dataset"])

    assert "--dataset-id is required when --collection dataset." in capsys.readouterr().err


def test_runner_dump_provider_payload_supports_sdk(
    capsys: pytest.CaptureFixture[str],
) -> None:
    exit_code = runner.main(
        [
            "--prompt",
            "test prompt",
            "--dump-provider-payload",
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    docs_message = payload["contents"][0]["parts"][0]["text"]
    assert "## docs/sdk/references/quickstart.md" in docs_message
    assert "Import from `sdk` in `model.py`." in docs_message


def test_runner_infers_provider_from_env_default_model(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    (repo_root / ".env").write_text(
        "ARTICRAFT_MODEL=gemini-3-flash-preview\nARTICRAFT_THINKING_LEVEL=high\n",
        encoding="utf-8",
    )
    monkeypatch.delenv("ARTICRAFT_MODEL", raising=False)
    monkeypatch.delenv("ARTICRAFT_THINKING_LEVEL", raising=False)

    exit_code = runner.main(
        [
            "--prompt",
            "test prompt",
            "--repo-root",
            str(repo_root),
            "--dump-provider-payload",
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["model"] == "gemini-3-flash-preview"
    assert "contents" in payload
    assert "reasoning" not in payload


def test_runner_rejects_invalid_env_thinking_level(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    (repo_root / ".env").write_text("ARTICRAFT_THINKING_LEVEL=not-a-level\n", encoding="utf-8")
    monkeypatch.delenv("ARTICRAFT_THINKING_LEVEL", raising=False)

    with pytest.raises(SystemExit, match="2"):
        runner.main(
            [
                "--prompt",
                "test prompt",
                "--repo-root",
                str(repo_root),
                "--dump-provider-payload",
            ]
        )

    assert "ARTICRAFT_THINKING_LEVEL must be one of" in capsys.readouterr().err


def test_runner_loads_env_defaults_from_repo_root(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    (repo_root / ".env").write_text(
        "ARTICRAFT_MODEL=gpt-5.5-direct-runner\nARTICRAFT_THINKING_LEVEL=xhigh\n",
        encoding="utf-8",
    )
    other_cwd = tmp_path / "cwd"
    other_cwd.mkdir()
    monkeypatch.chdir(other_cwd)
    monkeypatch.delenv("ARTICRAFT_MODEL", raising=False)
    monkeypatch.delenv("ARTICRAFT_THINKING_LEVEL", raising=False)

    exit_code = runner.main(
        [
            "--prompt",
            "test prompt",
            "--provider",
            "gemini",
            "--repo-root",
            str(repo_root),
            "--dump-provider-payload",
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["model"] == "gpt-5.5-direct-runner"
