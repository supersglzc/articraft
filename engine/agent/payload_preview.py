"""
Provider request preview construction for dry-run CLI/debug flows.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Optional

from engine.agent.prompts import load_sdk_docs_reference, load_system_prompt_text
from engine.agent.providers.factory import ProviderConfig, create_provider_client
from engine.agent.tools import (
    build_first_turn_messages as _build_first_turn_messages,
)
from engine.agent.tools import build_tool_registry


def build_provider_payload_preview(
    user_content: Any,
    *,
    provider: str,
    model_id: str,
    openai_transport: str = "http",
    thinking_level: str,
    system_prompt_path: str,
    sdk_package: str = "sdk",
    openai_reasoning_summary: Optional[str] = "auto",
    tool_registry_builder: Callable[..., Any] = build_tool_registry,
) -> dict:
    repo_root = Path(__file__).resolve().parents[2]

    _, system_prompt = load_system_prompt_text(
        system_prompt_path,
        provider=provider,
        sdk_package=sdk_package,
        repo_root=repo_root,
    )

    docs = load_sdk_docs_reference(
        repo_root,
        sdk_package=sdk_package,
    )
    conversation = _build_first_turn_messages(
        user_content,
        sdk_docs_context=docs,
        provider=provider,
    )
    tools = tool_registry_builder(provider, sdk_package=sdk_package).get_tool_schemas()

    llm = create_provider_client(
        ProviderConfig(
            provider=provider,
            model_id=model_id,
            thinking_level=thinking_level,
        ),
        dry_run=True,
    )
    return llm.build_request_preview(
        system_prompt=system_prompt,
        messages=conversation,
        tools=tools,
    )
