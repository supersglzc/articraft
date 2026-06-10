"""Gemini-only provider wiring.

This project targets Gemini as the single LLM backend. The factory keeps a small,
stable surface (``ProviderConfig``, ``create_provider_client``, ``default_model_id``,
``infer_provider_from_model_id``, ``normalize_provider_name``,
``validate_provider_credentials``) so the rest of the runtime does not need to know
that only one provider exists. To add another provider later, reintroduce a routing
branch here.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from engine.agent.providers.base import ProviderClient
from engine.agent.providers.gemini import (
    DEFAULT_GEMINI_MODEL,
    GeminiLLM,
    gemini_client_config_from_env,
)
from engine.articraft.config import default_model_from_env, default_thinking_level_from_env
from engine.articraft.values import ProviderName

PROVIDER = ProviderName.GEMINI.value


@dataclass(slots=True, frozen=True)
class ProviderConfig:
    provider: str = PROVIDER
    model_id: str | None = None
    thinking_level: str = field(default_factory=default_thinking_level_from_env)

    @property
    def normalized_provider(self) -> str:
        return PROVIDER


def normalize_provider_name(provider: str | None = None) -> str:
    """Every supported provider is Gemini."""
    return PROVIDER


def infer_provider_from_model_id(model_id: str | None) -> str | None:
    """Model ids are Gemini ids; unknown ids return None so callers can error clearly."""
    model_norm = (model_id or "").strip().lower()
    if not model_norm or model_norm.startswith("gemini-"):
        return PROVIDER
    return None


def default_model_id(config: ProviderConfig) -> str:
    if config.model_id:
        return config.model_id
    env_model = default_model_from_env()
    return env_model or DEFAULT_GEMINI_MODEL


def create_provider_client(
    config: ProviderConfig,
    *,
    dry_run: bool = False,
) -> ProviderClient:
    return GeminiLLM(
        model_id=default_model_id(config),
        thinking_level=config.thinking_level,
        dry_run=dry_run,
    )


def validate_provider_credentials(provider: str | None = None) -> None:
    # Raises if Gemini credentials are not configured in the environment.
    gemini_client_config_from_env()
