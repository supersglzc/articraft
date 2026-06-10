# Gemini-only refactor (branch `gemini`)

This branch trims Articraft to a single LLM backend (Gemini) and reorganizes the
code so a second generator (e.g. rigid-body) can be added cleanly.

## Phase 1 — collapse providers to Gemini

Removed:
- Provider modules: `agent/providers/{openai,anthropic,openrouter,deepseek,dashscope,codex_cli,openai_codec}.py`.
- Multi-provider routing in `agent/providers/factory.py` (now Gemini-only helpers).
- Multi-provider tool sets in `agent/tools/__init__.py` (now one Gemini tool set);
  deleted `agent/tools/apply_patch.py` (was OpenAI/codex only).
- Non-Gemini designer prompts under `agent/prompts/generated/`.
- `ProviderName` enum reduced to `GEMINI` only.
- Provider-specific tests (`test_openai_provider.py`, `test_anthropic_provider.py`,
  `test_provider_conformance.py`, `test_provider_tool_registry.py`, …).

Changed:
- `agent/harness.py` constructs `GeminiLLM` directly; `self.provider` is always `"gemini"`.
- `articraft/values.py`: `normalize_provider_name` always returns Gemini;
  `infer_provider_from_model_id` returns Gemini for `gemini-*`/empty, else `None`.
- `.env.example`: only `GEMINI_API_KEYS` / Vertex / `ARTICRAFT_*`.

### Known vestigial bits (safe to delete once the suite is green)
These are inert under Gemini and were left in place to avoid editing ~90 call sites
blind (no runnable test env on this machine):
- `openai_transport` / `openai_reasoning_summary` params still thread through
  `SingleRunSettings`, `single_run`, `record_persistence`, CLI, batch — always unused.
- `build_openai_prompt_cache_settings(...)` in `agent/harness.py` is dead.
- Unused non-Gemini pricing constants/detectors in `agent/cost.py`.
- `--provider` CLI flag still exists but only accepts `gemini`.

## Phase 2 — engine / generators split (DONE)

The runtime packages were moved under a single `engine/` package:

    agent storage cli viewer articraft  ->  engine/{agent,storage,cli,viewer,articraft}

`sdk/` stays a top-level package (generated records do `import sdk`); `scaffold.py`
moved into it (`sdk/scaffold.py`). All imports were rewritten to `engine.*`, and the
`Path(__file__).parents[N]` repo-root computations were bumped one level. Packaging,
console script (`engine.cli.main:main`), viewer launch strings, justfile, and
pre-commit paths were updated. See `GENERATORS.md` for the structure and how to add a
second generator (e.g. rigid-body).

The engine is domain-agnostic and reads only a *generator profile*
(`sdk/_profiles.py: SdkProfile`): SDK package + scaffold + docs + designer prompt.

## Verifying

Done in-env (installed `uv`, `uv sync --group dev`):

```bash
uv run --group dev pytest -q        # 732 passed, 1 skipped
uv run ruff check engine sdk tests  # clean
uv run articraft status             # CLI works; dataset intact (10,787 records)
```
