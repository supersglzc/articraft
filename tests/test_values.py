from __future__ import annotations

import pytest

from engine.articraft.values import (
    ProviderName,
    ThinkingLevel,
    infer_provider_from_model_id,
    normalize_provider_name,
    normalize_thinking_level,
    provider_reasoning_level,
    reasoning_level_alias,
)


@pytest.mark.parametrize(
    ("model_id", "provider"),
    [
        ("gemini-3-flash-preview", ProviderName.GEMINI),
        ("gemini-3.5-flash", ProviderName.GEMINI),
        ("", ProviderName.GEMINI),
    ],
)
def test_infer_provider_from_model_id_gemini(model_id: str, provider: ProviderName) -> None:
    assert infer_provider_from_model_id(model_id) is provider


@pytest.mark.parametrize("model_id", ["gpt-5.5", "claude-sonnet-4-5", "deepseek-v4-pro"])
def test_infer_provider_from_model_id_non_gemini_returns_none(model_id: str) -> None:
    assert infer_provider_from_model_id(model_id) is None


def test_normalize_provider_name_always_gemini() -> None:
    assert normalize_provider_name("anything") is ProviderName.GEMINI
    assert normalize_provider_name() is ProviderName.GEMINI


def test_thinking_level_helpers_keep_public_med_spelling() -> None:
    assert normalize_thinking_level("medium") is ThinkingLevel.MED
    assert provider_reasoning_level("med") == "medium"
    assert reasoning_level_alias("med") == "medium"


def test_thinking_level_helpers_accept_xhigh() -> None:
    assert normalize_thinking_level("xhigh") is ThinkingLevel.XHIGH
    assert provider_reasoning_level("xhigh") == "xhigh"
    assert reasoning_level_alias("xhigh") == "xhigh"
