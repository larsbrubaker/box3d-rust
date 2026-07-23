//! Determinism soak scenes (`sample_determinism.cpp`, driving `shared/determinism.c`).
//!
//! Four scenes share one wasm module and the `determinism_` interaction prefix
//! (mouse grab, spawn, counters, debug draw, world toggles). Each runs until every
//! body sleeps, then the settled transforms are hashed and the HUD reports the
//! sleep step and world hash, exactly like the C sample's `printf`:
//!
//! - **Falling Ragdolls** — a 2×2 tile field catches groups of humans (unchanged
//!   from the original single-scene page).
//! - **Wave Pile** — 100 mixed convex bodies dropped on a wave height field.
//! - **Query Spawn** — zero-gravity, query-driven spawning; a per-cycle ray /
//!   overlap AABB / swept sphere / spawn marker is exposed for the TS overlay.
//! - **Mesh Drop** — a 20×20 grid of thin fast boxes dropped on a wave mesh
//!   (moved from Continuous upstream at c52908c).
//!
//! The scene builders, sleep/hash tracking, and teardown are the ported library
//! functions in `box3d_rust::determinism` — this module only wraps them for the
//! browser (pose packing, ground wireframe, the interaction shell).

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::draw_data::shape_styles;
use crate::interact::{self, MouseGrab};
use crate::shell::ZERO_POS;
use crate::vis::{
    capsule_from_body, hf_triangle_edges, mesh_triangle_edges, mesh_triangle_edges_offset,
    push_poses, VisBody,
};
use box3d_rust::core::NULL_INDEX;
use box3d_rust::determinism::{
    create_falling_ragdolls, create_mesh_drop, create_query_spawn, create_wave_pile,
    destroy_falling_ragdolls, destroy_mesh_drop, destroy_query_spawn, destroy_wave_pile,
    update_falling_ragdolls, update_mesh_drop, update_query_spawn, update_wave_pile,
    FallingRagdollData, MeshDropData, QuerySpawnData, WavePileData, MESH_DROP_BODY_COUNT,
    QUERY_SPAWN_CAST_RADIUS, QUERY_SPAWN_COUNT, RAGDOLL_GRID_COUNT, WAVE_PILE_BODY_COUNT,
};
use box3d_rust::hull::get_hull_points;
use box3d_rust::human::BONE_COUNT;
use box3d_rust::math_functions::{Vec3, VEC3_ONE};
use box3d_rust::shape::ShapeGeometry;
use box3d_rust::types::default_world_def;
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// Ground tile spacing (`GRID_SIZE`, `shared/determinism.c` :13). Private in the
/// library, so mirrored here to recompute the tile centers for the wireframe.
const GRID_SIZE: f32 = 15.0;

thread_local! {
    static STATE: RefCell<Option<DetState>> = const { RefCell::new(None) };
}

/// The active scenario's ported library data. Held in one enum so the shared
/// interaction shell (`world` / `bodies` / `grab`) and the pose/hash exports work
/// for every scene without a per-scene state type.
enum Scene {
    FallingRagdolls(FallingRagdollData),
    WavePile(WavePileData),
    QuerySpawn(QuerySpawnData),
    MeshDrop(MeshDropData),
}

impl Scene {
    /// Sleep step recorded once the scene settled (`data.sleepStep`).
    fn sleep_step(&self) -> i32 {
        match self {
            Scene::FallingRagdolls(d) => d.sleep_step,
            Scene::WavePile(d) => d.sleep_step,
            Scene::QuerySpawn(d) => d.sleep_step,
            Scene::MeshDrop(d) => d.sleep_step,
        }
    }

    /// Settled world hash (`data.hash`, `B3_HASH`); 0 until the scene sleeps.
    fn hash(&self) -> u32 {
        match self {
            Scene::FallingRagdolls(d) => d.hash,
            Scene::WavePile(d) => d.hash,
            Scene::QuerySpawn(d) => d.hash,
            Scene::MeshDrop(d) => d.hash,
        }
    }
}

struct DetState {
    world: World,
    scene: Scene,
    /// Render bodies (capsule bones / spheres / boxes / rock stand-ins).
    bodies: Vec<VisBody>,
    /// Static ground tile edges (`[x0,y0,z0, x1,y1,z1, ...]`); empty for Query Spawn.
    ground_edges: Vec<f32>,
    grab: MouseGrab,
    step_count: u32,
    done: bool,
    /// Query Spawn: the C sample advances the scenario every 10th step so each
    /// query lingers on screen (`sample_determinism.cpp` QuerySpawn::Step).
    query_frame_count: i32,
    /// Query Spawn: render bodies already appended (data.bodies grows one per cycle).
    query_vis_count: i32,
}

fn with_state<R>(f: impl FnOnce(&mut DetState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("determinism not initialized — call a determinism_reset* first"))
    })
}

/// Drop any prior scene's owned meshes/height fields before rebuilding.
fn destroy_prev(prev: &mut DetState) {
    match &mut prev.scene {
        Scene::FallingRagdolls(d) => destroy_falling_ragdolls(d),
        Scene::WavePile(d) => destroy_wave_pile(d),
        Scene::QuerySpawn(d) => destroy_query_spawn(d),
        Scene::MeshDrop(d) => destroy_mesh_drop(d),
    }
}

fn new_world() -> World {
    interact::reset_scene_scales();
    // The C sample uses the Sample base world (b3DefaultWorldDef), gravity -10.
    World::new(&default_world_def())
}

/// Build a render body for a settled convex body by inspecting its first shape:
/// sphere / capsule render exactly; an 8-point axis-aligned box hull renders as a
/// box; any other convex hull (the wave-pile rocks) uses an icosahedron stand-in
/// sized to the hull's bounding radius (matching the `KIND_ICOSAHEDRON` rock path).
fn vis_body_from_body(world: &World, body_index: i32) -> Option<VisBody> {
    let mut sid = world.bodies[body_index as usize].head_shape_id;
    while sid != NULL_INDEX {
        let shape = &world.shapes[sid as usize];
        match &shape.geometry {
            ShapeGeometry::Sphere(s) => {
                return Some(VisBody::sphere_body(body_index, s.radius));
            }
            ShapeGeometry::Capsule(c) => {
                return Some(VisBody::capsule_body(body_index, c));
            }
            ShapeGeometry::Hull(h) => {
                let points = get_hull_points(h);
                if points.len() == 8 {
                    // make_box_hull → 8-point box centered at the origin.
                    let mut hx = 0.0f32;
                    let mut hy = 0.0f32;
                    let mut hz = 0.0f32;
                    for p in points {
                        hx = hx.max(p.x.abs());
                        hy = hy.max(p.y.abs());
                        hz = hz.max(p.z.abs());
                    }
                    return Some(VisBody::box_body(body_index, hx, hy, hz));
                }
                let mut r = 0.0f32;
                for p in points {
                    r = r.max((p.x * p.x + p.y * p.y + p.z * p.z).sqrt());
                }
                return Some(VisBody::icosahedron_colored(body_index, r, 0));
            }
            _ => {}
        }
        sid = shape.next_shape_id;
    }
    None
}

// -------------------------------------------------------------------------------------------------
// Falling Ragdolls (original single-scene page)
// -------------------------------------------------------------------------------------------------

/// Build the ragdoll ground wireframe: a grid mesh and a torus mesh at every tile
/// center, recomputing the centers with the same stepping `CreateFallingRagdolls`
/// uses.
fn build_ragdoll_ground(data: &FallingRagdollData) -> Vec<f32> {
    let mut out = Vec::new();
    let (grid, torus) = match (data.grid_mesh.as_ref(), data.torus_mesh.as_ref()) {
        (Some(g), Some(t)) => (g, t),
        _ => return out,
    };
    let span = GRID_SIZE * RAGDOLL_GRID_COUNT as f32;
    let start = -0.5 * span + 0.5 * GRID_SIZE;
    for i in 0..RAGDOLL_GRID_COUNT {
        for j in 0..RAGDOLL_GRID_COUNT {
            let offset = Vec3 {
                x: start + i as f32 * GRID_SIZE,
                y: 0.0,
                z: start + j as f32 * GRID_SIZE,
            };
            out.extend_from_slice(&mesh_triangle_edges_offset(grid, VEC3_ONE, offset));
            out.extend_from_slice(&mesh_triangle_edges_offset(torus, VEC3_ONE, offset));
        }
    }
    out
}

#[wasm_bindgen]
pub fn determinism_reset() -> u32 {
    STATE.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            destroy_prev(prev);
        }
        let mut world = new_world();
        let data = create_falling_ragdolls(&mut world);

        let mut bodies = Vec::new();
        for group in &data.groups {
            for human in &group.humans {
                for b in 0..BONE_COUNT {
                    let body_id = human.bones[b].body_id;
                    if body_id.is_null() {
                        continue;
                    }
                    let body_index = body_id.index1 - 1;
                    if let Some(cap) = capsule_from_body(&world, body_index) {
                        bodies.push(VisBody::capsule_body(body_index, &cap));
                    }
                }
            }
        }

        let ground_edges = build_ragdoll_ground(&data);
        let n = bodies.len() as u32;
        *cell.borrow_mut() = Some(DetState {
            world,
            scene: Scene::FallingRagdolls(data),
            bodies,
            ground_edges,
            grab: MouseGrab::default(),
            step_count: 0,
            done: false,
            query_frame_count: 0,
            query_vis_count: 0,
        });
        n
    })
}

// -------------------------------------------------------------------------------------------------
// Wave Pile
// -------------------------------------------------------------------------------------------------

#[wasm_bindgen]
pub fn determinism_reset_wave_pile() -> u32 {
    STATE.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            destroy_prev(prev);
        }
        let mut world = new_world();
        let data = create_wave_pile(&mut world);

        let mut bodies = Vec::new();
        for i in 0..WAVE_PILE_BODY_COUNT {
            let body_id = data.bodies[i];
            if body_id.is_null() {
                continue;
            }
            let body_index = body_id.index1 - 1;
            if let Some(vb) = vis_body_from_body(&world, body_index) {
                bodies.push(vb);
            }
        }

        // Height-field ground grows from a corner; CreateWavePile offsets the body
        // to center the patch: extent = scale.x * (field_count - 1) = 1.0 * 20.
        let extent = 1.0f32 * (21 - 1) as f32;
        let origin = Vec3 {
            x: -0.5 * extent,
            y: 0.0,
            z: -0.5 * extent,
        };
        let ground_edges = data
            .height_field
            .as_ref()
            .map(|hf| hf_triangle_edges(hf, origin))
            .unwrap_or_default();

        let n = bodies.len() as u32;
        *cell.borrow_mut() = Some(DetState {
            world,
            scene: Scene::WavePile(data),
            bodies,
            ground_edges,
            grab: MouseGrab::default(),
            step_count: 0,
            done: false,
            query_frame_count: 0,
            query_vis_count: 0,
        });
        n
    })
}

// -------------------------------------------------------------------------------------------------
// Query Spawn
// -------------------------------------------------------------------------------------------------

#[wasm_bindgen]
pub fn determinism_reset_query_spawn() -> u32 {
    STATE.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            destroy_prev(prev);
        }
        let mut world = new_world();
        let data = create_query_spawn(&mut world);

        // Zero-gravity empty space: bodies appear one per query cycle (see
        // determinism_step), so the render list starts empty.
        *cell.borrow_mut() = Some(DetState {
            world,
            scene: Scene::QuerySpawn(data),
            bodies: Vec::new(),
            ground_edges: Vec::new(),
            grab: MouseGrab::default(),
            step_count: 0,
            done: false,
            query_frame_count: 0,
            query_vis_count: 0,
        });
        0
    })
}

/// Last query cycle recorded for the TS overlay. Layout (all base-frame f32):
/// `[spawn_count, query_hit_count, spawn_total,`
/// ` ox,oy,oz, px,py,pz, did_hit, nx,ny,nz,`
/// ` lx,ly,lz, ux,uy,uz, cast_fraction, tx,ty,tz, sx,sy,sz, cast_radius]`.
#[wasm_bindgen]
pub fn determinism_query_viz() -> Vec<f32> {
    with_state(|state| {
        let d = match &state.scene {
            Scene::QuerySpawn(d) => d,
            _ => return Vec::new(),
        };
        vec![
            d.spawn_count as f32,
            d.query_hit_count as f32,
            QUERY_SPAWN_COUNT as f32,
            d.ray_origin.x as f32,
            d.ray_origin.y as f32,
            d.ray_origin.z as f32,
            d.ray_point.x as f32,
            d.ray_point.y as f32,
            d.ray_point.z as f32,
            if d.ray_did_hit { 1.0 } else { 0.0 },
            d.ray_normal.x,
            d.ray_normal.y,
            d.ray_normal.z,
            d.overlap_bounds.lower_bound.x,
            d.overlap_bounds.lower_bound.y,
            d.overlap_bounds.lower_bound.z,
            d.overlap_bounds.upper_bound.x,
            d.overlap_bounds.upper_bound.y,
            d.overlap_bounds.upper_bound.z,
            d.cast_fraction,
            d.ray_translation.x,
            d.ray_translation.y,
            d.ray_translation.z,
            d.last_spawn_position.x as f32,
            d.last_spawn_position.y as f32,
            d.last_spawn_position.z as f32,
            QUERY_SPAWN_CAST_RADIUS,
        ]
    })
}

// -------------------------------------------------------------------------------------------------
// Mesh Drop
// -------------------------------------------------------------------------------------------------

#[wasm_bindgen]
pub fn determinism_reset_mesh_drop() -> u32 {
    STATE.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            destroy_prev(prev);
        }
        let mut world = new_world();
        // C MeshDropDeterminism: CreateMeshDrop( worldId, b3Pos_zero ).
        let data = create_mesh_drop(&mut world, ZERO_POS);

        let mut bodies = Vec::new();
        for i in 0..MESH_DROP_BODY_COUNT {
            let body_id = data.bodies[i];
            if body_id.is_null() {
                continue;
            }
            let body_index = body_id.index1 - 1;
            if let Some(vb) = vis_body_from_body(&world, body_index) {
                bodies.push(vb);
            }
        }

        let ground_edges = data
            .mesh
            .as_ref()
            .map(|mesh| mesh_triangle_edges(mesh, VEC3_ONE))
            .unwrap_or_default();

        let n = bodies.len() as u32;
        *cell.borrow_mut() = Some(DetState {
            world,
            scene: Scene::MeshDrop(data),
            bodies,
            ground_edges,
            grab: MouseGrab::default(),
            step_count: 0,
            done: false,
            query_frame_count: 0,
            query_vis_count: 0,
        });
        n
    })
}

// -------------------------------------------------------------------------------------------------
// Shared per-step advance + read-outs
// -------------------------------------------------------------------------------------------------

#[wasm_bindgen]
pub fn determinism_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.saturating_add(1);

        if !state.done {
            match &mut state.scene {
                Scene::FallingRagdolls(d) => {
                    state.done = update_falling_ragdolls(&state.world, d);
                }
                Scene::WavePile(d) => {
                    state.done = update_wave_pile(&state.world, d);
                }
                Scene::MeshDrop(d) => {
                    state.done = update_mesh_drop(&state.world, d);
                }
                Scene::QuerySpawn(d) => {
                    // Advance the scenario every 10th step so each query lingers
                    // on screen (C QuerySpawn::Step, m_frameCount % 10 == 1).
                    state.query_frame_count += 1;
                    if state.query_frame_count % 10 == 1 {
                        state.done = update_query_spawn(&mut state.world, d);
                    }
                    // Append render bodies for any newly spawned shapes.
                    while state.query_vis_count < d.spawn_count {
                        let body_id = d.bodies[state.query_vis_count as usize];
                        if !body_id.is_null() {
                            let body_index = body_id.index1 - 1;
                            if let Some(vb) = vis_body_from_body(&state.world, body_index) {
                                state.bodies.push(vb);
                            }
                        }
                        state.query_vis_count += 1;
                    }
                }
            }
        }
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn determinism_step_count() -> u32 {
    with_state(|s| s.step_count)
}

#[wasm_bindgen]
pub fn determinism_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn determinism_styles() -> Vec<u32> {
    with_state(|state| shape_styles(&mut state.world, &state.bodies))
}

#[wasm_bindgen]
pub fn determinism_ground_wireframe() -> Vec<f32> {
    with_state(|s| s.ground_edges.clone())
}

/// Whether the scene has fully settled and the hash has been captured.
#[wasm_bindgen]
pub fn determinism_done() -> bool {
    with_state(|s| s.done)
}

/// Sleep step recorded when the scene settled (`data.sleepStep`).
#[wasm_bindgen]
pub fn determinism_sleep_step() -> i32 {
    with_state(|s| s.scene.sleep_step())
}

/// Settled world hash (`data.hash`, `B3_HASH`); 0 until the scene sleeps.
#[wasm_bindgen]
pub fn determinism_hash() -> u32 {
    with_state(|s| s.scene.hash())
}

/// Query Spawn's accumulated query hash (`QuerySpawnData.queryHash`); 0 for other
/// scenes. Reported on the settled HUD's second line (C QuerySpawn::Step).
#[wasm_bindgen]
pub fn determinism_query_hash() -> u32 {
    with_state(|s| match &s.scene {
        Scene::QuerySpawn(d) => d.query_hash,
        _ => 0,
    })
}

crate::demo_shell! {
    with_state: with_state,
    state: DetState,
    world: world,
    bodies: bodies,
    grab: grab,
    base: |_s| ZERO_POS,
    mouse_down: determinism_mouse_down,
    mouse_move: determinism_mouse_move,
    mouse_up: determinism_mouse_up,
    mouse_active: determinism_mouse_active,
    spawn_random: determinism_spawn_random = |state, spawned| {
        crate::interact::append_spawned_vis(&state.world, &mut state.bodies, spawned);
        match spawned.first() {
            Some(sp) => vec![1.0, sp.body_index as f32],
            None => vec![0.0, 0.0],
        }
    },
    delete_at_ray: determinism_delete_at_ray = |_state, _index| {},
    counters: determinism_counters,
    debug_draw: determinism_debug_draw,
    debug_text: determinism_debug_text,
}

crate::demo_world_toggles! {
    with_state: with_state,
    world: world,
    set_enable_sleep: determinism_set_enable_sleep,
    set_enable_warm_starting: determinism_set_enable_warm_starting,
    set_enable_continuous: determinism_set_enable_continuous,
    set_recycle_distance: determinism_set_recycle_distance,
}
