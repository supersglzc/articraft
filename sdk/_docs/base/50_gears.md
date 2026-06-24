# Gears

## Purpose

Use these helpers when a prompt needs generated gear geometry (spur, helical,
herringbone, ring, planetary, bevel, rack, worm, crossed-helical, hyperbolic)
rather than hand-authored tooth profiles. Each gear is a native
`MeshGeometry`: construct it and use it directly, or call `.build()` to get the
mesh, then export with `mesh_from_geometry(...)`.

## Approximation note

Native gears are **approximate**, not exact analytic gears. Tooth flanks are
polygon-lofted samples of the involute (controlled by `curve_points`), helical
and herringbone teeth are built from a finite stack of twist-extruded slices
(`twist_slices`), and bevel teeth use a simplified cone-projected profile rather
than exact spherical involutes. They are visually and dimensionally faithful for
assembly and articulation use, but should not be treated as metrology-grade
involute surfaces. State this when a prompt asks for "exact" gears.

## Import

```python
from sdk import (
    SpurGear,
    HerringboneGear,
    RingGear,
    PlanetaryGearset,
    BevelGear,
    BevelGearPair,
    RackGear,
    Worm,
    CrossedHelicalGear,
    HyperbolicGear,
    gear,
    mesh_from_geometry,
)
```

## Recommended APIs

| Shape intent | Helper | Required positional args |
| --- | --- | --- |
| Spur / helical gear | `SpurGear` | `module, teeth_number, width` (`helix_angle=` for helical) |
| Herringbone (double-helical) gear | `HerringboneGear` | `module, teeth_number, width` |
| Internal ring (annular) gear | `RingGear` | `module, teeth_number, width, rim_width` |
| Sun + planets + ring set | `PlanetaryGearset` | `module, sun_teeth_number, planet_teeth_number, width, rim_width, n_planets` |
| Single bevel gear | `BevelGear` | `module, teeth_number, width, cone_angle` |
| Meshing right-angle bevel pair | `BevelGearPair` | `module, gear_teeth, pinion_teeth, face_width` (`axis_angle=` deg) |
| Linear rack | `RackGear` | `module, length, width, height` |
| Worm (drives a worm wheel) | `Worm` | `module, lead_angle, n_threads, length` |
| Crossed-axis helical gear | `CrossedHelicalGear` | `module, teeth_number, width` |
| Hyperbolic / hypoid-style gear | `HyperbolicGear` | `module, teeth_number, width` |

Common keyword arguments shared by the involute gears: `pressure_angle=20.0`
(degrees), `helix_angle=0.0` (degrees), `clearance=0.0`, `backlash=0.0`.

## Units and sizing

- `module` is in meters. Pitch radius is `module * teeth_number / 2`. For a
  ~36 mm pitch-radius gear with 18 teeth, use `module=0.004`.
- `width` (face width) and all linear dimensions are meters.

## Usage

A gear is already a `MeshGeometry`, so export it directly:

```python
from sdk import SpurGear, mesh_from_geometry

spur = SpurGear(module=0.004, teeth_number=18, width=0.020)
mesh = mesh_from_geometry(spur, "drive_gear")
```

Add a shaft bore (or any feature) with a boolean before exporting:

```python
from sdk import SpurGear, CylinderGeometry, boolean_difference, mesh_from_geometry

spur = SpurGear(module=0.004, teeth_number=18, width=0.020, helix_angle=15.0)
bore = CylinderGeometry(radius=0.005, height=0.030)
mesh = mesh_from_geometry(boolean_difference(spur, bore), "drive_gear")
```

Merge a gear onto an existing target body with `gear(target, gear_, ...)`
(it builds the gear and unions it onto the target):

```python
from sdk import SpurGear, CylinderGeometry, gear, mesh_from_geometry

hub = CylinderGeometry(radius=0.012, height=0.020)
combined = gear(hub, SpurGear(module=0.003, teeth_number=24, width=0.020))
mesh = mesh_from_geometry(combined, "geared_hub")
```

## Modeling motion

Gears are visual geometry; mesh motion comes from the articulation graph, not
the tooth contact. Mount each gear on its own `REVOLUTE` joint about its shaft
axis, and use `MotionLimits` / mimic coupling to express the gear ratio between
a driver and a follower. Do not rely on the teeth to transmit motion in
simulation.

## Cross-references

- `40_mesh_geometry.md` for primitives, booleans, and `mesh_from_geometry`.
- `30_articulated_object.md` (common) for parts, joints, and mimic coupling.
