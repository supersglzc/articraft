# Generators & the engine

This repo is split into a domain-agnostic **engine** and one or more **generators**.

```
engine/                # shared runtime — knows nothing about a specific domain
  agent/               #   Gemini harness loop, tools, compiler, batch runner
  storage/             #   records + dataset (canonical on-disk layer)
  cli/                 #   the `articraft` command
  viewer/              #   FastAPI + React/Three.js inspector
  articraft/           #   config + values (env, defaults, provider/thinking enums)

sdk/                   # the "articulated" generator (the first/only one today)
  scaffold.py          #   starting model.py written into a fresh run
  _profiles.py         #   SdkProfile + SDK_PROFILES registry (the engine reads this)
  _docs/  _examples/   #   agent-facing docs + examples
  v0/  _core/          #   the authoring API imported by generated model.py (`import sdk`)
```

`sdk/` stays a **top-level package** on purpose: every generated record's `model.py`
does `import sdk`, and the compiler executes those scripts with the repo root on
`sys.path`. Moving it under `engine/` or `generators/` would break ~10K existing
records, so each generator is its own importable top-level package.

## The seam: `sdk_package` + `SdkProfile`

The engine never hardcodes anything articulated. A run carries an `sdk_package`
string (default `"sdk"`) that the engine resolves to a profile:

```python
from sdk._profiles import get_sdk_profile   # engine/agent/harness.py, workspace_docs.py, prompts/loader.py
profile = get_sdk_profile(sdk_package)
profile.scaffold_path          # starting model.py
profile.docs_for_mode("full")  # docs mounted into the agent's virtual workspace
profile.prompt_name_for_provider(...)  # designer system prompt
```

A `SdkProfile` bundles everything domain-specific: the SDK package name, the
scaffold template, the docs to mount, and the designer prompt. That is the entire
contract between the engine and a generator.

## Adding a second generator (e.g. rigid-body)

1. **Create a new top-level package** mirroring `sdk/`, e.g. `sdk_rigid/`:
   - `sdk_rigid/v0/` — the public authoring API generated `model.py` imports.
   - `sdk_rigid/_core/` — geometry/export internals + the compile/QC checks for the
     rigid-body domain (the engine's compiler calls into this via the same
     `compile_object_to_urdf_xml`-style entry point sdk exposes).
   - `sdk_rigid/scaffold.py` — the starting `model.py` (build/run_tests contract).
   - `sdk_rigid/_docs/`, `sdk_rigid/_examples/` — agent-facing docs + examples.

2. **Register a profile.** Add an entry to `SDK_PROFILES` (in `sdk/_profiles.py`, or
   lift that registry into a neutral `generators/` module if you prefer the engine
   not to import from `sdk`):
   ```python
   SDK_PROFILES["sdk_rigid"] = SdkProfile(
       package_name="sdk_rigid",
       scaffold_path=Path("sdk_rigid/scaffold.py"),
       docs_full=(...),
       docs_core=(...),
       designer_prompt_name="designer_system_prompt_gemini.txt",
   )
   ```

3. **Add the designer prompt** (reuse the section-based prompt build under
   `engine/agent/prompts/` — add a `provider_*`/`sdk_*` section for the rigid domain).

4. **Add it to packaging:** append `"sdk_rigid"` to
   `[tool.hatch.build.targets.wheel] packages` in `pyproject.toml`.

5. **Generate against it:** pass `--sdk-package sdk_rigid` (the value threads through
   the CLI → run context → harness → compiler unchanged).

Nothing in `engine/` changes. The harness loop, Gemini provider, tools, storage,
batch runner, and viewer are all reused as-is — they only ever see `sdk_package` and
the `SdkProfile` it resolves to.

## LLM backend

Gemini is the only provider. The whole LLM layer lives in
`engine/agent/providers/gemini.py`; `engine/agent/providers/factory.py` is a thin
Gemini-only seam (see `REFACTOR_NOTES.md`). A second generator does **not** touch the
LLM layer.
