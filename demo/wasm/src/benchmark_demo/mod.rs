//! Benchmark samples — the full `sample_benchmark.cpp` set, ported 1:1 from the
//! pinned C reference (scene builders in `shared/benchmarks.c`).
//!
//! `mod.rs` owns the shared [`BenchScene`] state, the scene enum, the shared
//! `new_world` / `empty_scene` / `install` helpers, the step dispatch, and every
//! scene-agnostic wasm export (poses, styles, mesh wireframe, counters, the mouse
//! grab / spawn / delete surface via [`crate::demo_shell!`], and the world
//! toggles). Each family of scenes lives in a submodule that owns its
//! `bench_reset_*` builder plus any per-scene control setters:
//!
//! - [`pyramids`]  — Large Pyramid, Wide Pyramid, Many Pyramids
//! - [`piles`]     — Falling Boxes, Candy Cups, Washer, Destruction
//! - [`rain`]      — Rain (library humans + `StepRain`)
//! - [`structs`]   — Joint Grid, Chains, Hull
//! - [`fields`]    — Explosion, Height Field, Large World
//! - [`legacy`]    — Junkyard, Falling Trees (the previously verified builders)
//!
//! # Counts policy
//!
//! Every count is one of Erin's two (DEBUG or release) values, never a third
//! invented number. The C release value is used when serial wasm holds it
//! interactively; otherwise the C DEBUG value is used and the info text discloses
//! "C release uses N". See each builder's doc comment for the choice + rationale.

mod fields;
mod legacy;
mod piles;
mod pyramids;
mod rain;
mod structs;

use crate::interact::{self, MouseGrab};
use crate::vis::{push_poses, VisBody};
use box3d_rust::height_field::HeightFieldData;
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{Pos, Vec3};
use box3d_rust::types::{default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<BenchScene>> = const { RefCell::new(None) };
}

/// Which benchmark scene is live. Drives the step dispatch and a couple of
/// per-scene render/telemetry exports.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum BenchKind {
    LargePyramid,
    WidePyramid,
    ManyPyramids,
    Rain,
    JointGrid,
    FallingBoxes,
    CandyCups,
    Explosion,
    HeightField,
    Trees,
    Washer,
    LargeWorld,
    Hull,
    Chains,
    Destruction,
    Junkyard,
}

/// Junkyard kinematic pusher animation (`StepJunkyard`, `benchmarks.c` :872).
pub(crate) struct JunkyardAnim {
    pub pusher_id: BodyId,
    pub degrees: f32,
    pub radius: f32,
}

/// Washer kinematic drum handle plus its baked hull geometry (the 36 wall + 4 rib
/// child hulls, flattened to drum-local space) so the browser renders the real drum
/// rather than a cylinder outline.
pub(crate) struct WasherState {
    pub drum_id: BodyId,
    pub drum_radius: f32,
    pub drum_half_len: f32,
    /// Solid faces of every drum child hull, `[x,y,z]` vertex triples (9 per tri).
    pub drum_tris: Vec<f32>,
    /// Wireframe edges of every drum child hull, endpoint triples (6 per edge).
    pub drum_edges: Vec<f32>,
}

/// Shared state for the single live benchmark scene.
pub(crate) struct BenchScene {
    pub world: World,
    pub bodies: Vec<VisBody>,
    pub grab: MouseGrab,
    pub kind: BenchKind,
    /// Static ground/terrain triangle edges in world space (`[x0,y0,z0,x1,y1,z1]*N`).
    /// Empty for the box-ground scenes; populated for the mesh-ground scenes
    /// (Rain, Chains, Trees, Explosion, Height Field, Destruction).
    pub ground_edges: Vec<f32>,
    pub step_count: u32,

    // --- Per-scene extra state (only the active scene's is populated) ---
    pub junkyard: Option<JunkyardAnim>,
    pub rain: Option<rain::RainState>,
    pub chains: Option<structs::ChainsState>,
    pub destruction: Option<piles::DestructionState>,
    pub washer: Option<WasherState>,
    pub large_world: Option<fields::LargeWorldState>,

    /// Candy Cups frustum-hull solid faces in cup-local space (`[x,y,z]` triples, 9
    /// per triangle), shared by every cup. Empty unless the live scene is Candy Cups.
    pub candy_hull: Vec<f32>,

    /// Explosion impulse-per-area (`m_impulse`, live Magnitude slider).
    pub explosion_impulse: f32,

    /// Height Field cast radius (`m_radius`, live Radius slider) + grid extent.
    pub hf_radius: f32,
    pub hf_columns: i32,
    pub hf_rows: i32,
    /// Kept only so `destroy_height_field` has something to consume on reset; the
    /// casts run against the world, not this handle.
    pub hf: Option<HeightFieldData>,

    /// Hull sample readout: trial count + surface areas (`BenchmarkHull::Step`).
    pub hull_trials: i32,
    pub hull_area: f32,
    pub hull_clone_area: f32,
    /// Hull sample: transformed (yellow) hull wireframe edges. The original
    /// (green) hull rides in `ground_edges`.
    pub hull_edges_b: Vec<f32>,
}

pub(crate) fn with_state<R>(f: impl FnOnce(&mut BenchScene) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("benchmark not initialized — call bench_reset_* first"))
    })
}

/// Fresh world with C gravity (`{0,-10,0}`). Restores the base launch/draw scales
/// on every scene reset (each reset funnels through here).
pub(crate) fn new_world() -> World {
    crate::interact::reset_scene_scales();
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

/// A `BenchScene` with all per-scene extras cleared; builders fill `world`/`bodies`
/// and then set whatever animation state their scene needs.
pub(crate) fn empty_scene(world: World, bodies: Vec<VisBody>, kind: BenchKind) -> BenchScene {
    BenchScene {
        world,
        bodies,
        grab: MouseGrab::default(),
        kind,
        ground_edges: Vec::new(),
        step_count: 0,
        junkyard: None,
        rain: None,
        chains: None,
        destruction: None,
        washer: None,
        large_world: None,
        candy_hull: Vec::new(),
        explosion_impulse: 1000.0,
        hf_radius: 0.1,
        hf_columns: 50,
        hf_rows: 50,
        hf: None,
        hull_trials: 0,
        hull_area: 0.0,
        hull_clone_area: 0.0,
        hull_edges_b: Vec::new(),
    }
}

pub(crate) fn install(scene: BenchScene) -> u32 {
    let count = scene.bodies.len() as u32;
    STATE.with(|cell| *cell.borrow_mut() = Some(scene));
    count
}

// ---------------------------------------------------------------------------
// Scene reset exports (thin wrappers over the submodule builders)
// ---------------------------------------------------------------------------

#[wasm_bindgen]
pub fn bench_reset_large_pyramid() -> u32 {
    install(pyramids::build_large_pyramid())
}

#[wasm_bindgen]
pub fn bench_reset_wide_pyramid() -> u32 {
    install(pyramids::build_wide_pyramid())
}

#[wasm_bindgen]
pub fn bench_reset_many_pyramids() -> u32 {
    install(pyramids::build_many_pyramids())
}

#[wasm_bindgen]
pub fn bench_reset_rain() -> u32 {
    install(rain::build_rain())
}

#[wasm_bindgen]
pub fn bench_reset_joint_grid() -> u32 {
    install(structs::build_joint_grid())
}

#[wasm_bindgen]
pub fn bench_reset_falling_boxes() -> u32 {
    install(piles::build_falling_boxes())
}

#[wasm_bindgen]
pub fn bench_reset_candy_cups() -> u32 {
    install(piles::build_candy_cups())
}

#[wasm_bindgen]
pub fn bench_reset_explosion() -> u32 {
    install(fields::build_explosion())
}

#[wasm_bindgen]
pub fn bench_reset_height_field() -> u32 {
    install(fields::build_height_field())
}

#[wasm_bindgen]
pub fn bench_reset_trees(grid_size: u32) -> u32 {
    install(legacy::build_trees(grid_size))
}

#[wasm_bindgen]
pub fn bench_reset_washer() -> u32 {
    install(piles::build_washer())
}

#[wasm_bindgen]
pub fn bench_reset_large_world() -> u32 {
    install(fields::build_large_world())
}

#[wasm_bindgen]
pub fn bench_reset_hull() -> u32 {
    install(structs::build_hull())
}

#[wasm_bindgen]
pub fn bench_reset_chains() -> u32 {
    install(structs::build_chains())
}

#[wasm_bindgen]
pub fn bench_reset_destruction() -> u32 {
    install(piles::build_destruction())
}

#[wasm_bindgen]
pub fn bench_reset_junkyard() -> u32 {
    install(legacy::build_junkyard())
}

// ---------------------------------------------------------------------------
// Step + render exports
// ---------------------------------------------------------------------------

#[wasm_bindgen]
pub fn bench_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        match state.kind {
            BenchKind::Junkyard => legacy::step_junkyard(state, dt),
            BenchKind::Rain => rain::step_rain(state),
            BenchKind::Chains => structs::step_chains(state),
            BenchKind::LargeWorld => fields::step_large_world(state),
            BenchKind::Destruction => piles::step_destruction(state),
            _ => {}
        }
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.wrapping_add(1);
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn bench_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

/// Packed engine-driven style words parallel to [`bench_poses`]. Drives the
/// per-instance sleep/wake recoloring on the instanced piles.
#[wasm_bindgen]
pub fn bench_styles() -> Vec<u32> {
    with_state(|state| crate::draw_data::shape_styles(&mut state.world, &state.bodies))
}

/// Static ground/terrain triangle edges (mesh-ground scenes only).
#[wasm_bindgen]
pub fn bench_mesh_wireframe() -> Vec<f32> {
    with_state(|state| state.ground_edges.clone())
}

// ---------------------------------------------------------------------------
// Per-scene controls + telemetry (C DrawControls / DrawTextLine)
// ---------------------------------------------------------------------------

/// Explosion Magnitude slider (`m_impulse`, `sample_benchmark.cpp` :468).
#[wasm_bindgen]
pub fn bench_set_explosion_magnitude(impulse: f32) {
    with_state(|state| state.explosion_impulse = impulse);
}

/// Explosion "Explode" button (`BenchmarkExplosion::Explode` :456).
#[wasm_bindgen]
pub fn bench_explode() {
    with_state(fields::explode);
}

/// Height Field Radius slider (`m_radius`, `sample_benchmark.cpp` :545).
#[wasm_bindgen]
pub fn bench_set_height_field_radius(radius: f32) {
    with_state(|state| state.hf_radius = radius);
}

/// Run the Height Field ray/shape cast grid once and return `[castCount, hitCount]`
/// (`BenchmarkHeightField::Render` :557-632). Timing is measured on the JS side.
#[wasm_bindgen]
pub fn bench_height_field_cast() -> Vec<f32> {
    with_state(fields::height_field_cast)
}

/// Hull sample readout: `[trials, area, cloneArea]` (`BenchmarkHull::Step`).
#[wasm_bindgen]
pub fn bench_hull_info() -> Vec<f32> {
    with_state(|state| {
        vec![
            state.hull_trials as f32,
            state.hull_area,
            state.hull_clone_area,
        ]
    })
}

/// Hull sample: transformed (yellow) hull wireframe edges (the original green hull
/// rides in [`bench_mesh_wireframe`]).
#[wasm_bindgen]
pub fn bench_hull_wireframe_b() -> Vec<f32> {
    with_state(|state| state.hull_edges_b.clone())
}

/// Washer drum pose + dims `[px,py,pz, qx,qy,qz,qw, radius, halfLen]`, or empty when
/// the live scene is not the Washer. The browser positions the drum geometry
/// ([`bench_washer_drum_geometry`]) with the pose fields.
#[wasm_bindgen]
pub fn bench_washer_drum() -> Vec<f32> {
    with_state(|state| {
        let Some(w) = &state.washer else {
            return Vec::new();
        };
        let xf = box3d_rust::body::get_body_transform(&state.world, w.drum_id.index1 - 1);
        vec![
            xf.p.x as f32,
            xf.p.y as f32,
            xf.p.z as f32,
            xf.q.v.x,
            xf.q.v.y,
            xf.q.v.z,
            xf.q.s,
            w.drum_radius,
            w.drum_half_len,
        ]
    })
}

/// Washer drum geometry in drum-local space: `[triCount, tris…, edgeCount, edges…]`
/// (the real 36 wall + 4 rib child hulls, `vis::hull_triangles`/`hull_edges`). Empty
/// unless the live scene is the Washer. Rendered at the [`bench_washer_drum`] pose.
#[wasm_bindgen]
pub fn bench_washer_drum_geometry() -> Vec<f32> {
    with_state(|state| {
        let Some(w) = &state.washer else {
            return Vec::new();
        };
        let mut out = Vec::with_capacity(2 + w.drum_tris.len() + w.drum_edges.len());
        out.push(w.drum_tris.len() as f32);
        out.extend_from_slice(&w.drum_tris);
        out.push(w.drum_edges.len() as f32);
        out.extend_from_slice(&w.drum_edges);
        out
    })
}

/// Candy Cups frustum-hull solid faces in cup-local space (flat `[x,y,z]` vertex
/// triples, 9 per triangle), shared by every cup. Empty unless the live scene is
/// Candy Cups. The browser uses this as the per-instance geometry for the cups.
#[wasm_bindgen]
pub fn bench_candy_hull() -> Vec<f32> {
    with_state(|state| state.candy_hull.clone())
}

// ---------------------------------------------------------------------------
// Shared grab / spawn / delete / counters / debug surface + world toggles
// ---------------------------------------------------------------------------

/// Push a freshly spawned bullet sphere onto the render list (shift-click).
fn on_spawn(state: &mut BenchScene, spawned: Option<interact::SpawnedBody>) -> Vec<f32> {
    match spawned {
        Some(sp) => {
            state
                .bodies
                .push(VisBody::sphere_body(sp.body_index, sp.half_extents[0]));
            vec![1.0, sp.body_index as f32]
        }
        None => vec![0.0, 0.0],
    }
}

crate::demo_shell! {
    with_state: with_state,
    state: BenchScene,
    world: world,
    bodies: bodies,
    grab: grab,
    base: |_s| crate::shell::ZERO_POS,
    mouse_down: bench_mouse_down,
    mouse_move: bench_mouse_move,
    mouse_up: bench_mouse_up,
    mouse_active: bench_mouse_active,
    spawn_random: bench_spawn_random = |state, spawned| on_spawn(state, spawned),
    delete_at_ray: bench_delete_at_ray = |_state, _index| {},
    counters: bench_counters,
    debug_draw: bench_debug_draw,
    debug_text: bench_debug_text,
}

crate::demo_world_toggles! {
    with_state: with_state,
    world: world,
    set_enable_sleep: bench_set_enable_sleep,
    set_enable_warm_starting: bench_set_enable_warm_starting,
    set_enable_continuous: bench_set_enable_continuous,
    set_recycle_distance: bench_set_recycle_distance,
}

/// `Sample::AddGroundBox( extent )` — ground body at `(0,-1,0)`, `extent×1×extent`
/// box hull, pushed as `VisBody` index 0. Shared by several benchmark scenes.
pub(crate) fn add_ground_box(scene: &mut BenchScene, extent: f32) -> BodyId {
    use box3d_rust::body::create_body;
    use box3d_rust::hull::make_box_hull;
    use box3d_rust::shape::create_hull_shape;
    use box3d_rust::types::{default_body_def, default_shape_def};

    let mut def = default_body_def();
    def.position = Pos {
        x: 0.0,
        y: -1.0,
        z: 0.0,
    };
    def.type_ = BodyType::Static;
    let ground = create_body(&mut scene.world, &def);
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape(&mut scene.world, ground, &default_shape_def(), &hull.base);
    scene
        .bodies
        .push(VisBody::box_body(ground.index1 - 1, extent, 1.0, extent));
    ground
}
