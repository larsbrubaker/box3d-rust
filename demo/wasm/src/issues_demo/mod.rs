//! Issues samples — faithful ports of `box3d-cpp-reference/samples/sample_issues.cpp`.
//!
//! Each C `RegisterSample( "Issues", … )` maps to one [`IssuesScene`] built by
//! [`scenes`]. The scenes share one stepping [`World`] plus the standard
//! [`demo_shell!`] export surface (mouse grab, shift-click spawn, ray delete,
//! counters, debug overlays) and the four world toggles. Static collision geometry
//! (Crash's grid mesh, s&box mover's height field + platform, Capsule Mesh's
//! building) is emitted as a wireframe; the two arbitrary hulls of Convex Jitter ride
//! a dedicated per-body geometry channel; Hull Crash is a static hull/points render
//! with no simulated bodies.
//!
//! # Deviations from C (disclosed)
//!
//! - **Dump Loader** reproduces the recorded body/shape defs inline: the C "dump" is
//!   emitted C++ source (`b3World_Dump` output `#include`d into the constructor), not a
//!   runtime-loadable format, and box3d-rust ports no dump *loader* API. Values are
//!   bit-exact; the only divergence is hand-porting the recorded calls.
//! - **Multiple Prismatic** sets `m_mouseForceScale = 1e6` in C (a stronger picker
//!   pull); the shared [`crate::interact::MouseGrab`] has no such scale, so the grab
//!   uses its default strength. Cosmetic only.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::excessive_precision)]

mod scenes;

use crate::interact::{self, MouseGrab};
use crate::shell::ZERO_POS;
use crate::vis::{push_poses, VisBody};
use box3d_rust::body::get_body_transform;
use box3d_rust::id::{BodyId, NULL_BODY_ID};
use box3d_rust::math_functions::Vec3;
use box3d_rust::types::{default_weld_joint_def, default_world_def};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// One arbitrary convex-hull render body (Convex Jitter). `tris`/`edges` are the
/// hull-local fan triangles and wire edges; the live transform comes from
/// `body_index` each frame.
pub(super) struct HullBody {
    pub body_index: i32,
    pub tris: Vec<f32>,
    pub edges: Vec<f32>,
}

pub(super) struct IssuesState {
    pub world: World,
    pub bodies: Vec<VisBody>,
    pub grab: MouseGrab,
    /// Combined static collision-mesh / height-field wireframe (world space).
    pub static_wire: Vec<f32>,
    /// Arbitrary-hull render bodies (Convex Jitter).
    pub hull_bodies: Vec<HullBody>,
    /// Crash "Add Joint" targets.
    pub crash_body1: BodyId,
    pub crash_body2: BodyId,
    /// Hull Crash static render (no simulated bodies).
    pub hull_crash_ok: bool,
    pub hull_crash_tris: Vec<f32>,
    pub hull_crash_edges: Vec<f32>,
    pub hull_crash_points: Vec<f32>,
}

impl IssuesState {
    pub(super) fn new() -> Self {
        IssuesState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
            static_wire: Vec::new(),
            hull_bodies: Vec::new(),
            crash_body1: NULL_BODY_ID,
            crash_body2: NULL_BODY_ID,
            hull_crash_ok: false,
            hull_crash_tris: Vec::new(),
            hull_crash_edges: Vec::new(),
            hull_crash_points: Vec::new(),
        }
    }
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

thread_local! {
    static STATE: RefCell<Option<IssuesState>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut IssuesState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("issues demo not initialized — call issues_reset_* first"))
    })
}

fn install(state: IssuesState) -> u32 {
    interact::reset_scene_scales();
    let count = state.bodies.len() as u32;
    STATE.with(|cell| *cell.borrow_mut() = Some(state));
    count
}

// --- Scene resets ----------------------------------------------------------

#[wasm_bindgen]
pub fn issues_reset_dump_loader() -> u32 {
    install(scenes::build_dump_loader())
}

#[wasm_bindgen]
pub fn issues_reset_crash() -> u32 {
    install(scenes::build_crash())
}

#[wasm_bindgen]
pub fn issues_reset_multiple_prismatic() -> u32 {
    install(scenes::build_multiple_prismatic())
}

#[wasm_bindgen]
pub fn issues_reset_hull_crash() -> u32 {
    install(scenes::build_hull_crash())
}

#[wasm_bindgen]
pub fn issues_reset_convex_jitter() -> u32 {
    install(scenes::build_convex_jitter())
}

#[wasm_bindgen]
pub fn issues_reset_sbox_mover() -> u32 {
    install(scenes::build_sbox_mover())
}

#[wasm_bindgen]
pub fn issues_reset_capsule_mesh() -> u32 {
    install(scenes::build_capsule_mesh())
}

// --- Stepping + render surface --------------------------------------------

#[wasm_bindgen]
pub fn issues_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        // Hull Crash is a static render (no simulated bodies); still step the empty
        // world so the shared surface behaves uniformly.
        state.world.step(dt, sub_steps);
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn issues_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn issues_styles() -> Vec<u32> {
    with_state(|state| crate::draw_data::shape_styles(&mut state.world, &state.bodies))
}

#[wasm_bindgen]
pub fn issues_body_count() -> u32 {
    with_state(|state| state.bodies.len() as u32)
}

/// Static collision-mesh / height-field wireframe (world space), if any.
#[wasm_bindgen]
pub fn issues_static_wireframe() -> Vec<f32> {
    with_state(|state| state.static_wire.clone())
}

/// Convex Jitter hull geometry (built once). Per hull body:
/// `[triFloatCount, tris…, edgeFloatCount, edges…]` in hull-local space.
#[wasm_bindgen]
pub fn issues_hull_geometry() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        for hb in &state.hull_bodies {
            out.push(hb.tris.len() as f32);
            out.extend_from_slice(&hb.tris);
            out.push(hb.edges.len() as f32);
            out.extend_from_slice(&hb.edges);
        }
        out
    })
}

/// Live transform per Convex Jitter hull body, index-aligned to
/// [`issues_hull_geometry`]: `[px,py,pz, qx,qy,qz,qw]` × N.
#[wasm_bindgen]
pub fn issues_hull_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::with_capacity(state.hull_bodies.len() * 7);
        for hb in &state.hull_bodies {
            let xf = get_body_transform(&state.world, hb.body_index);
            out.extend_from_slice(&[
                xf.p.x as f32,
                xf.p.y as f32,
                xf.p.z as f32,
                xf.q.v.x,
                xf.q.v.y,
                xf.q.v.z,
                xf.q.s,
            ]);
        }
        out
    })
}

/// Hull Crash static render: `[ok, triCount, tris…, edgeCount, edges…, ptCount, pts…]`.
/// When `ok == 1` the point block is empty; when `ok == 0` the tri/edge blocks are.
#[wasm_bindgen]
pub fn issues_hull_crash() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        out.push(if state.hull_crash_ok { 1.0 } else { 0.0 });
        out.push(state.hull_crash_tris.len() as f32);
        out.extend_from_slice(&state.hull_crash_tris);
        out.push(state.hull_crash_edges.len() as f32);
        out.extend_from_slice(&state.hull_crash_edges);
        out.push(state.hull_crash_points.len() as f32);
        out.extend_from_slice(&state.hull_crash_points);
        out
    })
}

/// Crash "Add Joint" button — weld the two dynamic boxes (C `Crash::DrawControls`).
#[wasm_bindgen]
pub fn issues_add_joint() {
    with_state(|state| {
        if !state.crash_body1.is_non_null() || !state.crash_body2.is_non_null() {
            return;
        }
        let mut joint_def = default_weld_joint_def();
        joint_def.base.body_id_a = state.crash_body1;
        joint_def.base.body_id_b = state.crash_body2;
        box3d_rust::joint::create_weld_joint(&mut state.world, &joint_def);
    });
}

crate::demo_shell! {
    with_state: with_state,
    state: IssuesState,
    world: world,
    bodies: bodies,
    grab: grab,
    base: |_s| ZERO_POS,
    mouse_down: issues_mouse_down,
    mouse_move: issues_mouse_move,
    mouse_up: issues_mouse_up,
    mouse_active: issues_mouse_active,
    spawn_random: issues_spawn_random = |state, spawned| match spawned {
        Some(sp) => {
            let idx = sp.body_index;
            let hx = sp.half_extents[0];
            state.bodies.push(VisBody::sphere_body(idx, hx));
            vec![1.0, idx as f32, hx, hx, hx, sp.kind as f32]
        }
        None => vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    delete_at_ray: issues_delete_at_ray = |state, index| {
        // Prune a deleted Convex Jitter hull body from its render channel.
        state.hull_bodies.retain(|hb| hb.body_index != index);
        if state.crash_body1.is_non_null() && state.crash_body1.index1 - 1 == index {
            state.crash_body1 = NULL_BODY_ID;
        }
        if state.crash_body2.is_non_null() && state.crash_body2.index1 - 1 == index {
            state.crash_body2 = NULL_BODY_ID;
        }
    },
    counters: issues_counters,
    debug_draw: issues_debug_draw,
    debug_text: issues_debug_text,
}

crate::demo_world_toggles! {
    with_state: with_state,
    world: world,
    set_enable_sleep: issues_set_enable_sleep,
    set_enable_warm_starting: issues_set_enable_warm_starting,
    set_enable_continuous: issues_set_enable_continuous,
    set_recycle_distance: issues_set_recycle_distance,
}
