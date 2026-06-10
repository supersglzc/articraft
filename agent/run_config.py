from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(slots=True, frozen=True)
class SingleRunSettings:
    provider: str
    model_id: str
    thinking_level: str
    max_turns: int
    system_prompt_path: str
    sdk_package: str
    # Vestigial: retained so existing call sites/records stay valid. Unused by Gemini.
    openai_transport: str = "http"
    openai_reasoning_summary: str | None = "auto"
    max_cost_usd: float | None = None

    def to_summary(self) -> dict[str, Any]:
        return {
            "provider": self.provider,
            "model_id": self.model_id,
            "thinking_level": self.thinking_level,
            "max_turns": self.max_turns,
            "max_cost_usd": self.max_cost_usd,
            "system_prompt_path": self.system_prompt_path,
            "sdk_package": self.sdk_package,
        }
