# Migrating CadQuery code to the native Articraft SDK

**Audience:** an automated agent (or engineer) porting old CadQuery-based
Articraft model code to the current **native mesh** SDK. Read this file together
with the live repo — the docs and examples it points to are the source of truth.

This branch has **no CadQuery**. `import cadquery` raises `ModuleNotFoundError`.
Geometry is authored as native `MeshGeometry` and combined with mesh booleans.
Your job: reproduce the *object and its articulation/teaching intent*, not the
CadQuery mechanism, and prove the result compiles.

---

## 0. Orient yourself in the repo first

Read these before porting anything:

- `sdk/_docs/common/00_quickstart.md` — script contract, imports, workflow.
- `sdk/_docs/base/40_mesh_geometry.md` — primitives, profiles, **Boolean
  Composition**, `mesh_from_geometry`.
- `sdk/_docs/base/50_gears.md`, `51_positioning_and_transforms.md`, and the
  family pages `41`–`49` (panels, brackets, fans, knobs, wires, lofts, bezels,
  wheels, hinges).
- `sdk/_examples/base/*.md` — 80+ **working** native examples. Find the closest
  analogue to what you're porting and mirror its structure. Articulation
  references: `*_chain.md`, `single_*_module.md`, `*_gripper.md`. Geometry
  references: `lego_brick.md`, `a_parametric_enclosure.md`, `plate_with_hole.md`.
- The public API surface: `grep` names in `sdk/v0/__init__.py`. Import only
  those. Never guess a helper name — if it isn't exported, it isn't available.

The native `sdk` import surface is small and flat: ~30 `*Geometry` classes,
`boolean_union/difference/intersection`, profile/spline helpers, the semantic
generators (incl. gears), `mesh_from_geometry`, plus the articulation API
(`ArticulatedObject`, `ArticulationType`, `Origin`, `MotionLimits`, `Box`, …).

---

## 1. The script contract (unchanged)

Every model file must define:

```python
def build_object_model() -> ArticulatedObject: ...
def run_tests() -> TestReport: ...
object_model = build_object_model()
```

The articulation/parts/test API (`model.part(...)`, `part.visual(...)`,
`model.articulation(...)`, `TestContext`, `expect_*`, placement helpers) is
**unchanged** from the CadQuery era. Most porting effort is therefore confined to
how geometry is *built*, not how the object is assembled.

---

## 2. API mapping: CadQuery → native

| CadQuery | Native SDK |
| --- | --- |
| `import cadquery as cq` | *(delete)* — `from sdk import (...)` only |
| `cq.Workplane("XY").box(x,y,z)` | `BoxGeometry((x, y, z))` |
| `.circle(r).extrude(h)` | `CylinderGeometry(radius=r, height=h)` |
| `.cylinder(h, r)` | `CylinderGeometry(radius=r, height=h)` |
| `Workplane().sphere(r)` | `SphereGeometry(radius=r)` |
| `.polygon(n, d)` / 2D sketch → `.extrude(h)` | build a profile (list of `(x,y)`, or `rounded_rect_profile` / `superellipse_profile`) → `ExtrudeGeometry(profile, h, center=...)` |
| `.revolve(...)` | `LatheGeometry(...)` (revolve a profile about an axis) |
| `.loft([...])` | `LoftGeometry([profile0, profile1, ...])` (planar rings at constant z) |
| `.sweep(path)` | `SweepGeometry(...)` / the spline helpers in `45_wires.md` |
| `.cut(tool)` | `boolean_difference(solid, tool)` |
| `.union(other)` | `boolean_union(a, b)` |
| `.intersect(other)` | `boolean_intersection(a, b)` |
| `.translate((x,y,z))` | `geom.translate(x, y, z)` |
| `.rotate(axisStart, axisEnd, deg)` | `geom.rotate((ax,ay,az), radians, origin=(...))` |
| `.mirror(...)` | place sign-flipped copies (`51_positioning_and_transforms.md`); reflection via `scale(-1, 1, 1)` only for chiral shapes |
| `.faces(sel).workplane().hole(d)` | `boolean_difference(solid, CylinderGeometry(...).translate(...))` or `ExtrudeWithHolesGeometry` / `PerforatedPanelGeometry` |
| `.edges(sel).fillet(r)` / `.chamfer(d)` | `rounded_box(...)` / `round_polygon_2d(...)` for vertical-edge rounding; **no general arbitrary-edge fillet** (see §4) |
| `.shell(t)` | `boolean_difference(outer, inner_cavity)` |
| workplane copy/offset/rotate, tags, construction geo | explicit `translate`/`rotate` + `Origin`; see `51_positioning_and_transforms.md` |
| `mesh_from_cadquery(solid, "name")` | `mesh_from_geometry(geometry, "name")` |
| `cq` gear plugin (`SpurGear`, etc.) | `from sdk import SpurGear, RingGear, BevelGearPair, Worm, ...` (see `50_gears.md`) |

---

## 3. Hard rules for native geometry

1. **Boolean operands must be closed, watertight solids.** Primitives and
   generators already are. Never boolean an open shell, a single face, or
   zero-thickness geometry.
2. **Make cutters overshoot.** A through-bore must be longer than the wall it
   pierces so entry/exit faces aren't left coplanar.
3. **Transform to final pose before the boolean.** Transforms mutate in place
   and return the geometry; chain them on freshly constructed operands, don't
   reuse one mutated base (`copy()`/`clone()` if you must reuse).
4. **Angles are radians** unless a parameter ends in `_deg`.
5. **Prefer parts over CSG for assembly.** Distinct/moving pieces → separate
   `model.part(...)` + `part.visual(...)` with placement helpers. Use raw
   booleans only to shape a single solid (bores, pockets, fused features).
6. **Use dedicated helpers where they exist.** Repeated holes →
   `ExtrudeWithHolesGeometry` / `PerforatedPanelGeometry`; semantic families →
   their `*Geometry` generator; openings → `cut_opening_on_face`.

---

## 4. Operations with no exact native equivalent (approximate + note it)

These CadQuery B-rep ops have no native primitive. Build the closest faithful
approximation and add a one-line prose note in the example/docstring stating the
approximation. Set the example's frontmatter/notes accordingly.

- **Arbitrary-edge fillet/chamfer** → only vertical-edge rounding via
  `rounded_box` / `round_polygon_2d`. For other edges, approximate or omit.
- **Shell** → `boolean_difference(outer_solid, inset_cavity_solid)`.
- **Split** → `boolean_intersection` / `boolean_difference` with a half-space box.
- **Thread** → a smooth cylinder, or a helical `SweepGeometry` approximation.
- **Extrude/cut "until a face"** → compute the distance explicitly and cut with a
  sized tool.
- **Exact gears** → native gears are approximate involutes (`50_gears.md`); never
  claim metrology-grade teeth.
- **Exact curved-surface precision** → everything is tessellated mesh; expect
  sub-percent deviation, not analytic surfaces.

---

## 5. Validate every ported file (required)

A port is not done until it compiles and passes the SDK checks. Run:

```bash
uv run python tools/validate_example.py path/to/model_or_example.md
```

It extracts the ```python fence, executes it, and runs
`TestContext.check_model_valid()` + `check_mesh_assets_ready()`. It must print
`OK`. For a standalone `model.py` (not a `.md`), run it directly and construct a
`TestContext(object_model)` the same way.

Iterate until clean. Common failures and fixes:

| Error | Cause / fix |
| --- | --- |
| `ModuleNotFoundError: cadquery` | leftover import — remove it |
| `is not a valid manifold solid` | boolean operand isn't watertight — use a primitive/closed solid |
| `Loft profile area must be non-zero in XY projection` | loft rings must be closed XY loops at constant z |
| disconnected part / island | give the feature a real rib/stem/wall, or split into a part |
| current-pose overlap QC failure | fix the offset, or declare a scoped `allow_overlap(...)` with a proof check |
| missing mesh assets | every visible mesh must go through `mesh_from_geometry(...)` |

---

## 6. Per-file porting checklist

1. Read the original CadQuery source and identify: object identity, parts,
   articulations, and the prompt-critical visible features.
2. Find the nearest native example under `sdk/_examples/base/` and mirror its
   structure.
3. Replace all geometry construction using the §2 table; delete every CadQuery
   import and `mesh_from_cadquery` call.
4. Keep the articulation graph, tests, and placement logic intact.
5. For §4 operations, approximate and add a limitation note.
6. Run `tools/validate_example.py` until it prints `OK`.
7. Update frontmatter tags: drop `cadquery`; add `sdk`, `base sdk`, `mesh
   geometry`, and topical tags.

---

## 7. Worked example (before → after)

**CadQuery (old):**

```python
import cadquery as cq
from sdk import mesh_from_cadquery

plate = cq.Workplane("XY").box(0.12, 0.08, 0.01)
plate = plate.faces(">Z").workplane().pushPoints([(0.03, 0.0), (-0.03, 0.0)]).hole(0.008)
visual = mesh_from_cadquery(plate, "plate")
```

**Native (new):**

```python
from sdk import BoxGeometry, CylinderGeometry, boolean_difference, mesh_from_geometry

plate = BoxGeometry((0.12, 0.08, 0.01))
for x in (0.03, -0.03):
    hole = CylinderGeometry(radius=0.004, height=0.03).translate(x, 0.0, 0.0)
    plate = boolean_difference(plate, hole)
visual = mesh_from_geometry(plate, "plate")
```

Note the cutter (`height=0.03`) overshoots the plate (`0.01`) so the bore is
clean, and the two holes are placed by a loop instead of a workplane point list.
See `sdk/_examples/base/plate_with_hole.md` and
`making_counter_bored_and_counter_sunk_holes.md` for fuller references.
