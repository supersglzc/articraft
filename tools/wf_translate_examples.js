export const meta = {
  name: 'translate-cadquery-examples',
  description: 'Translate the deleted CadQuery example corpus into working native-SDK examples (functional parity), each validated to compile.',
  phases: [
    { title: 'Translate' },
    { title: 'Repair-existing' },
  ],
}

// ---- work list -------------------------------------------------------------
// Pure workplane-stack concepts with no native analogue: consolidated, not 1:1.
const WORKPLANE_EXCLUDE = [
  'copying_workplanes', 'creating_workplanes_on_faces', 'locating_a_workplane_on_a_vertex',
  'moving_the_current_working_point', 'offset_workplanes', 'rotated_workplanes',
  'using_construction_geometry', 'tagging_objects',
]
const GEARS = [
  'bevel_gears', 'crossed_and_hyperbolic_gears', 'cycloidal_gear', 'involute_gear',
  'rack_and_pinion', 'ring_gears_and_planetary_gearsets', 'spur_helical_herringbone_gears', 'worm_gear',
]
const HARD = [
  'extruding_until_a_given_face', 'inside_chamfer_on_a_shelled_cube', 'resin_mold',
  'shelling_to_create_thin_features', 'splitting_an_object', 'the_classic_occ_bottle', 'thread',
]
const ALL = [
  'a_parametric_bearing_pillow_block','a_parametric_enclosure','an_extruded_prismatic_solid','bevel_gears',
  'bga_package','braille_example','branching_tree_with_three_independent_rotary_branches',
  'branching_tree_with_two_independent_rotary_branches','building_profiles_using_lines_and_arcs',
  'coaxial_rotary_stack','copying_workplanes','creating_workplanes_on_faces','crossed_and_hyperbolic_gears',
  'cycloidal_gear','defining_an_edge_with_a_spline','din_rail_clip','dual_independent_finger_chains',
  'extruding_until_a_given_face','inside_chamfer_on_a_shelled_cube','involute_gear','lego_brick',
  'linear_carriage_with_independent_rotary_endeffector','locating_a_workplane_on_a_vertex',
  'making_counter_bored_and_counter_sunk_holes','making_lofts','mecanum_wheel','mirroring_3d_objects',
  'mirroring_from_faces','mirroring_symmetric_geometry','moving_the_current_working_point','offset_workplanes',
  'offsetaxis_rotary_stack','offsetting_wires_in_2d','opposed_twinslide_gripper','orthogonal_xy_stage',
  'panel_with_various_connector_holes','parametric_pin_header','pitray_clip','plate_with_hole','polygons',
  'polylines','portal_gantry_with_vertical_slide','prismaticrevolute_chain','prismaticrevoluterevolute_chain',
  'rack_and_pinion','raspberry_pi_3_model_b_assembly','reinforcing_a_junction_with_a_fillet','remote_enclosure',
  'resin_mold','revoluteprismatic_chain','revoluteprismaticrevolute_chain','ring_gears_and_planetary_gearsets',
  'rj45_surface_mount_jack','rotary_base_with_vertical_slide','rotated_workplanes','rounding_corners_with_fillet',
  'shelling_to_create_thin_features','simple_rectangular_plate','single_continuous_rotary_shaft',
  'single_pitch_axis_module','single_prismatic_slider','single_revolute_hinge','single_roll_axis_module',
  'single_yaw_axis_module','splitting_an_object','spur_helical_herringbone_gears','tagging_objects',
  'the_classic_occ_bottle','thread','threejoint_revolute_chain','threestage_telescoping_slide',
  'twojoint_revolute_chain','twostage_telescoping_slide','using_construction_geometry','using_point_lists',
  'vane_array_with_independent_pivots','vertical_slide_with_wrist_hinge','wall_vent_with_louvered_grille',
  'worm_gear','xyz_cartesian_stage','yawpitchroll_wrist',
]

const TRANSLATE = ALL.filter((n) => !WORKPLANE_EXCLUDE.includes(n)).map((n) => ({
  name: n,
  hint: GEARS.includes(n) ? 'gear' : HARD.includes(n) ? 'hard' : 'standard',
}))
// one consolidated example that teaches the workplane-mechanic *results* natively
const CONSOLIDATED = [{ name: 'positioning_offsets_and_mirroring', hint: 'positioning' }]
const REPAIRS = ['scooter_wheel_with_road_tire', 'orchestra_style_music_stand_with_tripod_base_and_telescoping_pole']

const SCHEMA = {
  type: 'object',
  additionalProperties: false,
  properties: {
    name: { type: 'string' },
    compiled: { type: 'boolean', description: 'true only if validate_example.py printed OK' },
    approximated: { type: 'boolean', description: 'true if native output is an approximation of a CadQuery-only feature' },
    summary: { type: 'string', description: 'one line: what was built + any approximation/limitation' },
    last_error: { type: 'string', description: 'final validator error if compiled is false, else empty' },
  },
  required: ['name', 'compiled', 'approximated', 'summary', 'last_error'],
}

const API_BRIEF = `
NATIVE SDK AUTHORING CONTRACT (no CadQuery exists anymore):
- Import ONLY public names from \`sdk\` (grep sdk/v0/__init__.py for the full public list). NEVER import cadquery / cq / mesh_from_cadquery.
- Geometry is native MeshGeometry. Build shapes with primitives (BoxGeometry, CylinderGeometry, ConeGeometry, SphereGeometry, DomeGeometry, CapsuleGeometry, TorusGeometry), LatheGeometry / ExtrudeGeometry / ExtrudeWithHolesGeometry / SweepGeometry / LoftGeometry, profile helpers (rounded_rect_profile, superellipse_profile), and the semantic generators (WheelGeometry, KnobGeometry, BarrelHingeGeometry, gears, etc.).
- Combine/cut solids with boolean_union / boolean_difference / boolean_intersection. Cut openings with cut_opening_on_face or ExtrudeWithHolesGeometry. Emit a part visual mesh with mesh_from_geometry(geometry, "logical_name").
- Required script shape (exactly): build_object_model() -> ArticulatedObject, run_tests() -> TestReport, and a module-level \`object_model = build_object_model()\`.
- Read these before writing: sdk/_docs/common/00_quickstart.md, sdk/_docs/base/40_mesh_geometry.md, and the most relevant sdk/_docs/base/4x_*.md family page. Read 1-2 existing native examples for exact format: sdk/_examples/base/atv_quad_bike_with_front_steering_and_suspension.md (articulation) and sdk/_examples/base/fluted_range_knob_with_d_shaft_bore.md (geometry).
- Frontmatter: keep a 'title' and 'description'; tags must include 'sdk' and 'base sdk' plus relevant topical tags and 'mesh geometry'. Do NOT include the tag 'cadquery'.
`

function translatePrompt(item) {
  const src = `/tmp/cq_src/examples/${item.name}.md`
  const target = `sdk/_examples/base/${item.name}.md`
  let hintText = ''
  if (item.hint === 'gear') {
    hintText = `This is a GEAR example. Use the native gear classes (sdk.SpurGear, sdk.RingGear, sdk.BevelGearPair, sdk.Worm, sdk.gear, etc. — check gears.py / sdk exports). Native gears are APPROXIMATIONS of true involute/bevel/worm geometry; set approximated=true and add a one-sentence prose note in the example stating it is an approximate gear profile.`
  } else if (item.hint === 'hard') {
    hintText = `This demonstrates a CadQuery-only B-rep operation (thread / shell / split / extrude-until-face / inside-chamfer / mold) that has NO exact native equivalent. Build the closest faithful native approximation: shells via boolean_difference of an inset cavity; splits via boolean_intersection/difference with a half-space box; threads via a smooth cylinder or a helical SweepGeometry approximation; molds via boolean_difference of the part from a block. Set approximated=true and add a short prose note naming the limitation.`
  } else if (item.hint === 'positioning') {
    hintText = `SYNTHESIS TASK (not a 1:1 translation). CadQuery's workplane-stack concepts (copying/offsetting/rotating workplanes, locating on faces/vertices, construction geometry, tagging) have no native analogue. Write ONE cohesive native example that teaches the same RESULTS natively: placing multiple parts/visuals at offset and rotated positions via Origin/transforms, building a small mirrored/symmetric layout, and patterning repeated features. Read several of these CadQuery sources for intent: /tmp/cq_src/examples/{copying_workplanes,offset_workplanes,rotated_workplanes,creating_workplanes_on_faces,mirroring_symmetric_geometry,using_construction_geometry}.md`
  }
  return `You are translating one CadQuery example into a WORKING native-SDK example in the articraft repo (branch mesh-native-geometry). Functional parity: reproduce the SAME object/mechanism/teaching intent, not the CadQuery mechanism.

${item.hint === 'positioning' ? '' : `Original CadQuery source to translate: ${src} (read it first).`}
Write the result to: ${target}
${API_BRIEF}
${hintText}

VALIDATION (mandatory): after writing ${target}, run:
  uv run python tools/validate_example.py ${target}
It MUST print "OK". If it prints FAIL, read the error and fix the example (common causes: importing a non-existent name, non-watertight boolean operands, zero-area loft profiles, disconnected part islands, unmet expect_* checks, missing mesh assets). Re-run until it prints OK, up to ~5 serious attempts. Keep geometry realistic in scale (meters) and keep the part/articulation structure faithful to the source.

Do not edit any file other than ${target}. Return the structured result. Set compiled=true ONLY if the validator printed OK on your final run; otherwise compiled=false with the final error in last_error.`
}

function repairPrompt(name) {
  const target = `sdk/_examples/base/${name}.md`
  return `An existing native-SDK example in the articraft repo currently FAILS validation. Fix it.

File: ${target}
Run \`uv run python tools/validate_example.py ${target}\` to see the error, then edit ONLY ${target} to make it pass (print "OK"). Preserve the object's intent and structure; make the minimal geometry/API fixes needed (e.g., fix a zero-area loft profile, a non-watertight boolean, a disconnected part, or a stale import). Re-run until OK, up to ~5 attempts.
${API_BRIEF}
Return the structured result (approximated=false unless you had to approximate something).`
}

// ---- run -------------------------------------------------------------------
phase('Translate')
const items = [...TRANSLATE, ...CONSOLIDATED]
log(`Translating ${items.length} examples (+${REPAIRS.length} repairs) to native SDK...`)

const translated = await parallel(
  items.map((it) => () =>
    agent(translatePrompt(it), { label: `tx:${it.name}`, phase: 'Translate', schema: SCHEMA })
  )
)

const repaired = await parallel(
  REPAIRS.map((n) => () =>
    agent(repairPrompt(n), { label: `fix:${n}`, phase: 'Repair-existing', schema: SCHEMA })
  )
)

const all = [...translated, ...repaired].filter(Boolean)
const failed = all.filter((r) => !r.compiled)
const approx = all.filter((r) => r.approximated)
log(`Done. compiled=${all.filter((r) => r.compiled).length}/${all.length}  failed=${failed.length}  approximated=${approx.length}`)

return {
  total: all.length,
  compiled: all.filter((r) => r.compiled).length,
  failed: failed.map((r) => ({ name: r.name, error: r.last_error })),
  approximated: approx.map((r) => r.name),
  results: all.map((r) => ({ name: r.name, compiled: r.compiled, approximated: r.approximated, summary: r.summary })),
}
