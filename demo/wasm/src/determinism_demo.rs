//! Falling Ragdolls determinism soak (`sample_determinism.cpp` FallingRagdolls,
//! driving `shared/determinism.c` `CreateFallingRagdolls`). A grid of static
//! grid-mesh + torus-mesh tiles catches groups of humans dropped from y = 15;
//! once every body sleeps the settled transforms are hashed and the HUD reports
//! the sleep step and world hash, exactly like the C sample's `printf`.
//!
//! The scene builder, sleep/hash tracking, and teardown are the ported library
//! functions in `box3d_rust::determinism` — this module only wraps them for the
//! browser (pose packing, ground wireframe, the interaction shell). Grid counts
//! are the upstream `RAGDOLL_GRID_COUNT = 2` / `RAGDOLL_GROUP_SIZE = 2` (no
//! debug/release split upstream): a 2×2 tile field with 2 humans per tile.

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::draw_data::shape_styles;
use crate::interact::{self, MouseGrab};
use crate::shell::ZERO_POS;
use crate::vis::{capsule_from_body, mesh_triangle_edges_offset, push_poses, VisBody};
use box3d_rust::determinism::{
    create_falling_ragdolls, destroy_falling_ragdolls, update_falling_ragdolls, FallingRagdollData,
    RAGDOLL_GRID_COUNT,
};
use box3d_rust::human::BONE_COUNT;
use box3d_rust::math_functions::{Vec3, VEC3_ONE};
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

struct DetState {
    world: World,
    data: FallingRagdollData,
    /// Capsule bones of every human (rendered from the 16-float `vis` stride).
    bodies: Vec<VisBody>,
    /// Grid + torus tile edges (`[x0,y0,z0, x1,y1,z1, ...]`).
    ground_edges: Vec<f32>,
    grab: MouseGrab,
    step_count: u32,
    done: bool,
}

fn with_state<R>(f: impl FnOnce(&mut DetState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("determinism not initialized — call determinism_reset first"))
    })
}

/// Build the ground wireframe: a grid mesh and a torus mesh at every tile center,
/// recomputing the centers with the same stepping `CreateFallingRagdolls` uses.
fn build_ground_edges(data: &FallingRagdollData) -> Vec<f32> {
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

fn new_world() -> World {
    interact::reset_scene_scales();
    // The C sample uses the Sample base world (b3DefaultWorldDef), gravity -10.
    World::new(&default_world_def())
}

#[wasm_bindgen]
pub fn determinism_reset() -> u32 {
    STATE.with(|cell| {
        // Drop any prior scene's owned meshes before rebuilding.
        if let Some(prev) = cell.borrow_mut().as_mut() {
            destroy_falling_ragdolls(&mut prev.data);
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

        let ground_edges = build_ground_edges(&data);
        let n = bodies.len() as u32;
        *cell.borrow_mut() = Some(DetState {
            world,
            data,
            bodies,
            ground_edges,
            grab: MouseGrab::default(),
            step_count: 0,
            done: false,
        });
        n
    })
}

#[wasm_bindgen]
pub fn determinism_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.saturating_add(1);
        if !state.done {
            state.done = update_falling_ragdolls(&state.world, &mut state.data);
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

/// Whether the pile has fully settled and the hash has been captured.
#[wasm_bindgen]
pub fn determinism_done() -> bool {
    with_state(|s| s.done)
}

/// Sleep step recorded when the pile settled (`data.sleepStep`).
#[wasm_bindgen]
pub fn determinism_sleep_step() -> i32 {
    with_state(|s| s.data.sleep_step)
}

/// Settled world hash (`data.hash`, `B3_HASH`); 0 until the pile sleeps.
#[wasm_bindgen]
pub fn determinism_hash() -> u32 {
    with_state(|s| s.data.hash)
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
