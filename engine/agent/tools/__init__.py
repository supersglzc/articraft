from __future__ import annotations

import mimetypes
from pathlib import Path
from typing import Any

from engine.agent.prompts import normalize_sdk_package
from engine.agent.runtime_limits import BatchRuntimeLimits
from engine.agent.tools.base import (
    BaseDeclarativeTool,
    BaseToolInvocation,
    ToolResult,
    ToolSchema,
    make_tool_schema,
)
from engine.agent.tools.compile_model import CompileModelTool
from engine.agent.tools.edit_code import ReplaceTool
from engine.agent.tools.find_examples import FindExamplesTool
from engine.agent.tools.probe_model import ProbeModelTool
from engine.agent.tools.read_file import ReadFileTool
from engine.agent.tools.registry import ToolRegistry
from engine.agent.tools.write_code import WriteFileTool
from engine.articraft.values import ProviderName

SUPPORTED_IMAGE_MIME_TYPES_BY_PROVIDER: dict[str, set[str]] = {
    ProviderName.GEMINI.value: {
        "image/png",
        "image/jpeg",
        "image/webp",
        "image/heic",
        "image/heif",
    },
}

_FIRST_TURN_RUNTIME_GUIDANCE_SHARED = (
    "<runtime_task_guidance>\n"
    "- Read the current `model.py` before editing.\n"
    "- Start with a realism-first structure plan. Use one coherent scaffold when the real object needs layered bodies, hollow forms, mechanisms, or repeated features; otherwise make small focused edits.\n"
    "- Treat visual realism as part of the deliverable: make the object read clearly as the requested thing, with believable proportions, silhouette, colors/materials, and major visible surface treatment.\n"
    "- Run `compile_model` to check your latest revision.\n"
    "- If compile is clean and the model already satisfies the realism/mechanism brief, conclude.\n"
    "</runtime_task_guidance>"
)


def build_tool_registry(
    provider: str = ProviderName.GEMINI.value,
    *,
    sdk_package: str = "sdk",
    runtime_limits: BatchRuntimeLimits | None = None,
) -> ToolRegistry:
    # Gemini tool set: edit via replace/write_file, plus compile/probe/find_examples.
    package = normalize_sdk_package(sdk_package)
    tools: list[BaseDeclarativeTool] = [
        ReadFileTool(editable_model_only=True),
        ReplaceTool(),
        WriteFileTool(),
        CompileModelTool(),
        ProbeModelTool(sdk_package=package, runtime_limits=runtime_limits),
        FindExamplesTool(sdk_package=package, include_paths=False),
    ]
    return ToolRegistry(tools)


def build_first_turn_runtime_guidance(_provider: str) -> str:
    return _FIRST_TURN_RUNTIME_GUIDANCE_SHARED


def prepend_runtime_guidance(
    user_content: Any,
    *,
    runtime_guidance_text: str | None = None,
) -> Any:
    guidance = (runtime_guidance_text or "").strip()
    if not guidance:
        return user_content

    if isinstance(user_content, str):
        if not user_content.strip():
            return guidance
        return f"{guidance}\n\n{user_content}"

    if not isinstance(user_content, list):
        return user_content

    return [{"type": "input_text", "text": guidance}, *user_content]


def build_first_turn_messages(
    user_content: Any,
    *,
    sdk_docs_context: str,
    provider: str,
) -> list[dict[str, Any]]:
    messages: list[dict[str, Any]] = []
    if sdk_docs_context:
        messages.append({"role": "user", "content": sdk_docs_context})
    messages.append(
        {
            "role": "user",
            "content": prepend_runtime_guidance(
                user_content,
                runtime_guidance_text=build_first_turn_runtime_guidance(provider),
            ),
        }
    )
    return messages


def build_initial_user_content(
    text_prompt: str,
    *,
    image_path: Path | None = None,
    image_detail: str = "high",
    runtime_guidance_text: str | None = None,
) -> Any:
    content: Any
    if image_path is None:
        content = text_prompt
    else:
        content = [
            {"type": "input_text", "text": text_prompt},
            {
                "type": "input_image",
                "image_path": str(image_path),
                "detail": image_detail,
            },
        ]

    return prepend_runtime_guidance(content, runtime_guidance_text=runtime_guidance_text)


def resolve_image_path(
    image_arg: str | None,
    *,
    provider: str | None = None,
) -> Path | None:
    if not image_arg:
        return None

    path = Path(image_arg).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Image file not found: {path}")
    if not path.is_file():
        raise ValueError(f"Image path is not a file: {path}")

    mime_type, _ = mimetypes.guess_type(path.name)
    supported_mime_types = SUPPORTED_IMAGE_MIME_TYPES_BY_PROVIDER[ProviderName.GEMINI.value]
    if mime_type not in supported_mime_types:
        raise ValueError(
            f"Unsupported image type for Gemini: {path.name} ({mime_type or 'unknown'})"
        )

    size_bytes = path.stat().st_size
    if size_bytes >= 20 * 1024 * 1024:
        raise ValueError(
            f"Image file exceeds Gemini inline request limit: {path} "
            "(must stay under 20 MB including prompt text)"
        )

    return path


__all__ = [
    "BaseDeclarativeTool",
    "BaseToolInvocation",
    "ToolResult",
    "ToolSchema",
    "make_tool_schema",
    "CompileModelTool",
    "FindExamplesTool",
    "ProbeModelTool",
    "ReadFileTool",
    "ReplaceTool",
    "WriteFileTool",
    "ToolRegistry",
    "SUPPORTED_IMAGE_MIME_TYPES_BY_PROVIDER",
    "build_tool_registry",
    "build_first_turn_runtime_guidance",
    "prepend_runtime_guidance",
    "build_first_turn_messages",
    "build_initial_user_content",
    "resolve_image_path",
]
