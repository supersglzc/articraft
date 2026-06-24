---
title: 'Raspberry Pi 3 Model B Assembly'
description: 'Base SDK PCB assembly with a rounded green board, top and bottom solder mask layers, a 2x20 GPIO pin header, an RJ45 ethernet jack with a cut aperture, and three black BGA chip packages mounted to the board.'
tags:
  - sdk
  - base sdk
  - raspberry pi
  - single board computer
  - pcb
  - circuit board
  - electronics assembly
  - gpio header
  - pin header
  - rj45 jack
  - ethernet port
  - bga package
  - chip package
  - solder mask
  - mounting holes
  - mesh geometry
  - extrude with holes
  - rounded rect profile
  - boolean difference
  - fixed articulation
---
# Raspberry Pi 3 Model B Assembly

This base-SDK example reproduces the `cq-electronics` Raspberry Pi 3 Model B
assembly as a native mesh model: a rounded green PCB with four mounting holes,
top and bottom solder mask layers, a 2x20 0.1 inch GPIO pin header, an RJ45
ethernet jack with a cut connector aperture, and three black BGA packages
(the BCM2837 SoC, the USB/Ethernet controller, and the RAM mounted on the
underside). It is useful for queries such as `raspberry pi`, `single board
computer`, `pcb assembly`, `gpio header`, `pin header`, `rj45 jack`,
`ethernet port`, `bga package`, `solder mask`, `mounting holes`,
`ExtrudeWithHolesGeometry`, and `boolean_difference`.

The original CadQuery model is a constraint-solved rigid `Assembly`: there is no
real mechanism, every component is rigidly seated on the board. This native
version keeps that intent by making the PCB substrate the single root part and
attaching every component with `FIXED` articulations. Real geometry is modeled
where the original cuts it: the board and solder masks carry true through holes
(`ExtrudeWithHolesGeometry`), the RJ45 jack has an aperture cut into its front
face, and each through-hole pin actually passes through the header base.

```python
from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

# Real Raspberry Pi 3 Model B board dimensions, in meters.
BOARD_LEN = 0.085  # long edge, along +X
BOARD_WID = 0.056  # short edge, along +Y
BOARD_THK = 0.0015
MASK_THK = 0.0002
CORNER_RADIUS = 0.003

HOLE_DIAMETER = 0.0027
MASK_HOLE_DIAMETER = 0.006
HOLE_INSET = 0.0035  # mounting-hole inset from each edge
HOLE_PITCH_X = 0.058  # long-axis spacing between the two hole columns

TOP_Z = BOARD_THK / 2.0  # board top surface


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    from math import cos, sin

    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _hole_centers() -> list[tuple[float, float]]:
    half_x = HOLE_PITCH_X / 2.0
    cy = BOARD_WID / 2.0 - HOLE_INSET
    return [(half_x, cy), (half_x, -cy), (-half_x, cy), (-half_x, -cy)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="raspberry_pi_3_model_b")

    pcb_green = model.material("pcb_substrate", rgba=(0.85, 0.81, 0.52, 1.0))
    mask_green = model.material("solder_mask_green", rgba=(0.0, 0.55, 0.18, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.06, 0.06, 0.06, 1.0))
    gold_plate = model.material("gold_plate", rgba=(0.92, 0.68, 0.05, 1.0))
    tin_plate = model.material("tin_plate", rgba=(0.55, 0.60, 0.62, 1.0))
    package_black = model.material("package_black", rgba=(0.09, 0.09, 0.09, 1.0))

    hole_profiles = [
        _circle_profile(HOLE_DIAMETER / 2.0) for _ in _hole_centers()
    ]
    hole_profiles = [
        [(x + cx, y + cy) for (x, y) in prof]
        for prof, (cx, cy) in zip(hole_profiles, _hole_centers())
    ]

    # --- PCB substrate (root) -------------------------------------------------
    substrate = model.part("pcb_substrate")
    board_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(BOARD_LEN, BOARD_WID, CORNER_RADIUS, corner_segments=8),
        hole_profiles,
        height=BOARD_THK,
        center=True,
    )
    substrate.visual(
        mesh_from_geometry(board_geom, "pi_pcb_substrate"),
        material=pcb_green,
    )
    substrate.inertial = Inertial.from_geometry(
        Box((BOARD_LEN, BOARD_WID, BOARD_THK)),
        mass=0.045,
    )

    # --- Solder mask layers (top + bottom) ------------------------------------
    mask_hole_profiles = [
        [(x + cx, y + cy) for (x, y) in _circle_profile(MASK_HOLE_DIAMETER / 2.0)]
        for (cx, cy) in _hole_centers()
    ]
    mask_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(BOARD_LEN, BOARD_WID, CORNER_RADIUS, corner_segments=8),
        mask_hole_profiles,
        height=MASK_THK,
        center=True,
    )

    mask_top = model.part("solder_mask_top")
    mask_top.visual(
        mesh_from_geometry(mask_geom.clone(), "pi_solder_mask_top"),
        material=mask_green,
    )
    mask_top.inertial = Inertial.from_geometry(
        Box((BOARD_LEN, BOARD_WID, MASK_THK)), mass=0.002
    )

    mask_bottom = model.part("solder_mask_bottom")
    mask_bottom.visual(
        mesh_from_geometry(mask_geom.clone(), "pi_solder_mask_bottom"),
        material=mask_green,
    )
    mask_bottom.inertial = Inertial.from_geometry(
        Box((BOARD_LEN, BOARD_WID, MASK_THK)), mass=0.002
    )

    # --- GPIO 2x20 0.1in pin header -------------------------------------------
    pitch = 0.00254
    pin_width = 0.00064
    base_height = 0.0024
    pin_above = 0.007
    pin_below = 0.003
    pin_length = pin_above + base_height + pin_below
    rows, cols = 2, 20
    base_len = pitch * cols
    base_wid = pitch * rows

    gpio = model.part("gpio_header")
    base_geom = BoxGeometry((base_len, base_wid, base_height))
    pin_local = []
    for r in range(rows):
        ly = -base_wid / 2.0 + pitch / 2.0 + r * pitch
        for c in range(cols):
            lx = -base_len / 2.0 + pitch / 2.0 + c * pitch
            pin_local.append((lx, ly))
    # Cut pin clearance holes through the plastic base.
    for lx, ly in pin_local:
        slot = BoxGeometry((pin_width, pin_width, base_height + 0.0005)).translate(lx, ly, 0.0)
        base_geom = boolean_difference(base_geom, slot)
    gpio.visual(
        mesh_from_geometry(base_geom, "pi_gpio_base"),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=black_plastic,
    )
    # Gold pins as one fused mesh so the part is a single connected island.
    pins_geom = None
    pin_center_z = pin_length / 2.0 - pin_below
    for lx, ly in pin_local:
        pin = BoxGeometry((pin_width, pin_width, pin_length)).translate(
            lx, ly, pin_center_z
        )
        pins_geom = pin if pins_geom is None else boolean_union(pins_geom, pin)
    gpio.visual(
        mesh_from_geometry(pins_geom, "pi_gpio_pins"),
        material=gold_plate,
    )
    gpio.inertial = Inertial.from_geometry(
        Box((base_len, base_wid, pin_length)), mass=0.005
    )

    # --- RJ45 ethernet jack with cut aperture ---------------------------------
    jack_len = 0.021  # along +X (depth into board)
    jack_wid = 0.016  # along +Y
    jack_hgt = 0.014
    aperture_w = 0.01168
    aperture_h = 0.00775

    ethernet = model.part("ethernet_jack")
    jack_geom = BoxGeometry((jack_len, jack_wid, jack_hgt))
    # Cut the connector aperture as a blind pocket into the +X (front) face.
    aperture_depth = 0.013
    cutter = BoxGeometry((aperture_depth, aperture_w, aperture_h)).translate(
        jack_len / 2.0 - aperture_depth / 2.0 + 0.0005, 0.0, 0.0
    )
    jack_geom = boolean_difference(jack_geom, cutter)
    ethernet.visual(
        mesh_from_geometry(jack_geom, "pi_ethernet_jack"),
        material=tin_plate,
    )
    ethernet.inertial = Inertial.from_geometry(
        Box((jack_len, jack_wid, jack_hgt)), mass=0.012
    )

    # --- BGA chip packages ----------------------------------------------------
    def _bga(part_name: str, mesh_name: str, size: float, height: float) -> "Part":
        part = model.part(part_name)
        geom = BoxGeometry((size, size, height))
        # Pin-1 index mark: small notched divot on the top corner.
        notch = BoxGeometry((0.002, 0.002, height * 0.6)).translate(
            -(size / 2.0 - 0.0012),
            -(size / 2.0 - 0.0012),
            height / 2.0,
        )
        geom = boolean_difference(geom, notch)
        part.visual(mesh_from_geometry(geom, mesh_name), material=package_black)
        part.inertial = Inertial.from_geometry(
            Box((size, size, height)), mass=0.001
        )
        return part

    _bga("bcm2837", "pi_bcm2837", 0.014, 0.0011)
    _bga("usb_controller", "pi_usb_controller", 0.009, 0.0011)
    _bga("ram", "pi_ram", 0.009, 0.0011)

    # --- Rigid mounts (the original used constraint-solved fixed seating) ------
    model.articulation(
        "mask_top_mount",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="solder_mask_top",
        origin=Origin(xyz=(0.0, 0.0, TOP_Z + MASK_THK / 2.0)),
    )
    model.articulation(
        "mask_bottom_mount",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="solder_mask_bottom",
        origin=Origin(xyz=(0.0, 0.0, -TOP_Z - MASK_THK / 2.0)),
    )
    mask_surface = TOP_Z + MASK_THK
    model.articulation(
        "gpio_mount",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="gpio_header",
        origin=Origin(xyz=(-0.027, 0.0155, mask_surface)),
    )
    model.articulation(
        "ethernet_mount",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="ethernet_jack",
        # Rotated 90deg about Z so the aperture faces the long board edge.
        origin=Origin(xyz=(0.030, 0.0205, mask_surface + jack_hgt / 2.0), rpy=(0.0, 0.0, pi / 2.0)),
    )
    model.articulation(
        "bcm2837_mount",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="bcm2837",
        origin=Origin(xyz=(0.0, -0.006, mask_surface + 0.0011 / 2.0)),
    )
    model.articulation(
        "usb_controller_mount",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="usb_controller",
        origin=Origin(xyz=(0.030, -0.012, mask_surface + 0.0011 / 2.0)),
    )
    model.articulation(
        "ram_mount",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="ram",
        # RAM is on the underside of the board.
        origin=Origin(xyz=(0.0, -0.006, -mask_surface - 0.0011 / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Solder masks seat flush on the board faces; allow that contact embedding.
    ctx.allow_overlap("pcb_substrate", "solder_mask_top")
    ctx.allow_overlap("pcb_substrate", "solder_mask_bottom")

    substrate = object_model.get_part("pcb_substrate")
    aabb = ctx.part_world_aabb(substrate)
    ctx.check("board_aabb_present", aabb is not None, "Expected a board AABB.")
    if aabb is not None:
        mins, maxs = aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("board_long_edge", 0.082 <= size[0] <= 0.088, f"size={size!r}")
        ctx.check("board_short_edge", 0.053 <= size[1] <= 0.059, f"size={size!r}")

    for name in ("gpio_header", "ethernet_jack", "bcm2837", "usb_controller", "ram"):
        ctx.check(f"{name}_present", object_model.get_part(name) is not None, name)

    # GPIO header should sit above the board top surface.
    gpio_aabb = ctx.part_world_aabb(object_model.get_part("gpio_header"))
    if gpio_aabb is not None:
        ctx.check("gpio_above_board", gpio_aabb[0][2] >= -0.001, f"{gpio_aabb!r}")

    # RAM should sit below the board (underside mount).
    ram_aabb = ctx.part_world_aabb(object_model.get_part("ram"))
    if ram_aabb is not None:
        ctx.check("ram_below_board", ram_aabb[1][2] <= 0.001, f"{ram_aabb!r}")

    return ctx.report()


object_model = build_object_model()
```
