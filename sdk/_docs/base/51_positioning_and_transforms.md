# Positioning, Transforms, and Symmetry

## Purpose

The native SDK has no workplane stack. Instead of selecting a workplane,
offsetting it, or copying it, you place geometry directly with explicit
transforms and place parts with `Origin`. Use this page for the results that
CadQuery's workplane mechanics used to produce: offset and rotated placement,
mirrored and symmetric layouts, and repeated feature patterns.

## Transforming geometry

Every `MeshGeometry` supports chainable transforms. They mutate in place and
return the geometry, so apply them to freshly constructed operands.

```python
from sdk import BoxGeometry
from math import pi

# offset placement (replaces "offset workplane" / "move the working point")
shifted = BoxGeometry((0.04, 0.04, 0.01)).translate(0.05, 0.0, 0.02)

# rotated placement (replaces "rotated workplane")
tilted = BoxGeometry((0.04, 0.04, 0.01)).rotate((0.0, 1.0, 0.0), pi / 6)

# rotate about a point other than the origin
hinged = BoxGeometry((0.04, 0.01, 0.01)).rotate((0.0, 0.0, 1.0), pi / 4, origin=(0.02, 0.0, 0.0))
```

- `translate(dx, dy, dz)` — meters.
- `rotate(axis, angle, *, origin=(0, 0, 0))` — `angle` in radians, right-hand rule.
- `scale(sx, sy=None, sz=None)` — uniform when only `sx` is given.

## Symmetric and mirrored layouts

Build symmetry by placing copies at sign-flipped offsets rather than reflecting
geometry. This keeps every copy a valid, correctly-wound solid.

```python
from sdk import BoxGeometry, boolean_union

def lug(x_sign):
    return BoxGeometry((0.01, 0.02, 0.03)).translate(x_sign * 0.06, 0.0, 0.0)

# left/right symmetric pair (replaces "mirror across a face")
body = BoxGeometry((0.10, 0.04, 0.03))
part = boolean_union(boolean_union(body, lug(1.0)), lug(-1.0))
```

For a true geometric reflection use `scale(-1.0, 1.0, 1.0)` (mirror across the
YZ plane), but prefer sign-flipped placement for layout symmetry; reflection is
mainly useful for genuinely chiral single shapes.

## Repeated feature patterns

Replace point-list / construction-geometry patterning with a loop that places
each instance, then union (for bosses) or difference (for holes).

```python
from sdk import BoxGeometry, CylinderGeometry, boolean_difference

plate = BoxGeometry((0.12, 0.08, 0.01))

# 2 x 3 grid of through-holes (replaces "rarray of points + holes")
for ix in range(2):
    for iy in range(3):
        x = (ix - 0.5) * 0.08
        y = (iy - 1.0) * 0.03
        hole = CylinderGeometry(radius=0.004, height=0.03).translate(x, y, 0.0)
        plate = boolean_difference(plate, hole)
```

For a radial pattern, drive the angle from the loop index:

```python
from math import cos, sin, pi
from sdk import CylinderGeometry, boolean_union

hub = CylinderGeometry(radius=0.02, height=0.01)
count = 6
for i in range(count):
    a = 2.0 * pi * i / count
    spoke = CylinderGeometry(radius=0.003, height=0.04).translate(0.03 * cos(a), 0.03 * sin(a), 0.0)
    hub = boolean_union(hub, spoke)
```

For dense or regular hole fields in panels, prefer the dedicated
`PerforatedPanelGeometry` / `ExtrudeWithHolesGeometry` helpers
(see `41_panels_and_grilles.md`) over hand-rolled boolean loops.

## Placing parts (not just geometry)

To position a whole part within the assembly, set its frame with `Origin`
on the visual or articulation, or use a placement helper. Reach for explicit
geometry transforms (above) to shape a single mesh, and `Origin` / placement
helpers to locate parts relative to each other.

```python
from sdk import ArticulatedObject, Box, Origin

model = ArticulatedObject(name="offset_demo")
base = model.part("base")
base.visual(Box((0.10, 0.06, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.01)), name="base_shell")
# a boss placed at an offset + rotation, the native equivalent of an offset/rotated workplane
base.visual(Box((0.02, 0.02, 0.03)), origin=Origin(xyz=(0.03, 0.0, 0.035), rpy=(0.0, 0.0, 0.4)), name="boss")
```

## Cross-references

- `40_mesh_geometry.md` for primitives, booleans, and export.
- `50_placement.md` (common) for `place_on_surface`, `place_on_face`, wrapping,
  and alignment helpers used to mount parts onto other parts.
