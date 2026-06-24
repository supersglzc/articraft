---
title: 'Spherical Ball-and-Socket Joint (Cut Until a Curved Face)'
description: 'Base SDK example translating the CadQuery "extrude until a given face" spherical-joint demo into a native mesh ball-and-socket: a base block whose socket cavity is cut to a spherical surface, plus a ball-and-stem shaft that seats in it on a multi-DOF floating articulation.'
tags:
  - sdk
  - base sdk
  - mesh geometry
  - boolean difference
  - boolean intersection
  - spherical joint
  - ball and socket
  - ball joint
  - socket cavity
  - curved surface cut
  - extrude until face
  - floating articulation
  - sphere geometry
  - cylinder geometry
---
# Spherical Ball-and-Socket Joint (Cut Until a Curved Face)

This base-SDK example reproduces the object built by the CadQuery
"extruding until a given face" demo: a spherical ball-and-socket joint. In the
original, a slot is cut and a bore is extruded *until a curved spherical face*
so the cut follows the sphere without overlap guesswork. That B-rep "until a
given face" operation has no exact native equivalent.

The faithful native approximation builds the same mechanism with explicit mesh
geometry and boolean operations:

- the socket cavity is formed by `boolean_difference` of a base block with a
  sphere, so the inner cavity wall *is* the curved spherical face the original
  cut had to chase.
- the seated ball is a `SphereGeometry` sized to the cavity, fused to a
  cylindrical stem with `boolean_union`.
- the ball and socket move on a multi-DOF `FLOATING` articulation, which is the
  closest native stand-in for a true spherical (ball) joint.

It is useful for queries such as `ball and socket`, `spherical joint`,
`ball joint`, `socket cavity`, `cut until a curved face`,
`boolean_difference`, and `floating articulation`.

**Approximation note:** the original CadQuery `extrude("next")` /
`cutBlind(face)` operations terminate a feature exactly on a named B-rep face
(here a sphere face). Native geometry has no "extrude/cut until a given face"
primitive, so the curved termination is approximated by differencing the part
against the actual sphere solid that defines that face. The result reads and
seats the same, but the cut surface is the tessellated sphere rather than an
analytic B-rep face. A real ball joint also has three rotational DOFs; the
native `FLOATING` joint is used as the closest multi-DOF stand-in.

```python
from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    Origin,
    SphereGeometry,
    Sphere,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
)

# Real-world scale: a compact mechanical ball joint, roughly 0.12 m across.
BLOCK = (0.12, 0.12, 0.10)
BALL_RADIUS = 0.050
SOCKET_RADIUS = 0.052  # slightly larger than the ball so it seats with clearance
SOCKET_CENTER_Z = 0.07  # socket sphere center, near the top of the block
STEM_RADIUS = 0.018
STEM_LENGTH = 0.20


def _weld(geometry: MeshGeometry, *, tol: float = 1e-7) -> MeshGeometry:
    """Merge coincident vertices so revolved primitives are watertight manifolds.

    ``SphereGeometry`` and ``CylinderGeometry`` emit duplicated pole and seam
    vertices, which leave the surface non-manifold for boolean operations.
    Welding by quantized position closes those seams.
    """

    welded = MeshGeometry()
    index: dict[tuple[int, int, int], int] = {}
    remap: list[int] = []
    for x, y, z in geometry.vertices:
        key = (round(x / tol), round(y / tol), round(z / tol))
        if key not in index:
            index[key] = welded.add_vertex(x, y, z)
        remap.append(index[key])
    for a, b, c in geometry.faces:
        a, b, c = remap[a], remap[b], remap[c]
        if a == b or b == c or a == c:
            continue
        welded.add_face(a, b, c)
    return welded


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spherical_ball_and_socket_joint")

    steel = model.material("joint_steel", rgba=(0.62, 0.64, 0.68, 1.0))
    dark_steel = model.material("joint_dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))

    # --- Base block with a spherical socket cavity --------------------------
    # The cavity is cut with a sphere, so the inner cavity wall IS the curved
    # spherical face that the original "cut until a given face" chased.
    block = BoxGeometry(BLOCK).translate(0.0, 0.0, BLOCK[2] / 2.0)
    socket_cutter = _weld(
        SphereGeometry(SOCKET_RADIUS, width_segments=48, height_segments=32)
    )
    socket_cutter.translate(0.0, 0.0, SOCKET_CENTER_Z)
    # Open the mouth of the socket: a smaller bore above the sphere so the ball
    # stem can pass, approximating the original bore "extruded until" the face.
    mouth = _weld(CylinderGeometry(STEM_RADIUS * 1.6, BLOCK[2], radial_segments=32))
    mouth.translate(0.0, 0.0, BLOCK[2] * 0.5 + SOCKET_CENTER_Z)
    socket_solid = boolean_difference(block, socket_cutter)
    socket_solid = boolean_difference(socket_solid, mouth)

    base = model.part("base")
    base.visual(
        mesh_from_geometry(socket_solid, "socket_block"),
        material=steel,
        name="socket_block",
    )
    # Mounting post below the block (the original base extrudes a circle down).
    post = CylinderGeometry(0.020, 0.08, radial_segments=28).translate(0.0, 0.0, -0.04)
    base.visual(
        mesh_from_geometry(post, "base_post"),
        material=dark_steel,
        name="base_post",
    )
    base.inertial = Inertial.from_geometry(
        Box(BLOCK),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, BLOCK[2] / 2.0)),
    )

    # --- Ball-and-stem shaft that seats in the socket ----------------------
    ball = _weld(SphereGeometry(BALL_RADIUS, width_segments=48, height_segments=32))
    ball.translate(0.0, 0.0, SOCKET_CENTER_Z)
    stem = _weld(CylinderGeometry(STEM_RADIUS, STEM_LENGTH, radial_segments=32))
    stem.translate(0.0, 0.0, SOCKET_CENTER_Z + STEM_LENGTH / 2.0)
    ball_shaft_solid = boolean_union(ball, stem)

    ball_shaft = model.part("ball_shaft")
    ball_shaft.visual(
        mesh_from_geometry(ball_shaft_solid, "ball_shaft"),
        material=dark_steel,
        name="ball_shaft",
    )
    ball_shaft.inertial = Inertial.from_geometry(
        Sphere(BALL_RADIUS),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, SOCKET_CENTER_Z)),
    )

    # A real ball joint has three rotational DOFs; FLOATING is the closest
    # native multi-DOF stand-in. The joint frame sits at the socket center.
    model.articulation(
        "ball_in_socket",
        ArticulationType.FLOATING,
        parent=base,
        child=ball_shaft,
        origin=Origin(xyz=(0.0, 0.0, SOCKET_CENTER_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    ball_shaft = object_model.get_part("ball_shaft")

    ctx.check("base_present", base is not None, "Expected a base part.")
    ctx.check("ball_present", ball_shaft is not None, "Expected a ball_shaft part.")
    if base is None or ball_shaft is None:
        return ctx.report()

    # The ball must sit inside the socket: it overlaps the base block in all
    # three axes at the socket center, proving the cavity actually receives it.
    ctx.expect_overlap(ball_shaft, base, axes="xyz", min_overlap=0.02)

    base_aabb = ctx.part_world_aabb(base)
    ctx.check("base_aabb_present", base_aabb is not None, "Expected a base AABB.")
    if base_aabb is not None:
        mins, maxs = base_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "base_footprint",
            0.10 <= max(size[0], size[1]) <= 0.14,
            f"size={size!r}",
        )

    ball_aabb = ctx.part_world_aabb(ball_shaft)
    ctx.check("ball_aabb_present", ball_aabb is not None, "Expected a ball AABB.")
    if ball_aabb is not None:
        mins, maxs = ball_aabb
        # The stem makes the shaft taller than it is wide.
        ctx.check(
            "shaft_tall",
            float(maxs[2] - mins[2]) > float(maxs[0] - mins[0]),
            f"mins={mins!r} maxs={maxs!r}",
        )

    return ctx.report()


object_model = build_object_model()
```
