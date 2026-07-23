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
//! - **Multiple Prismatic** sets `m_mouseForceScale = 1e6` in C (a stronger picker
//!   pull); [`issues_reset_multiple_prismatic`] applies the same override via
//!   [`crate::interact::set_grab_force_scale`], so the mouse grab matches C exactly.
//! - **GMod Wheel Stack** renders each wheel through the arbitrary-hull channel
//!   (30 instances of the wrapping hull). The C `Step` HUD prints per-step profiling
//!   (`b3World_GetProfile` step/collide/solve milliseconds and the effective contact
//!   hz derived from the app's step hz × sub-steps); those are C-app profiling lines
//!   and are not reproduced — the scene is otherwise bit-identical.
//! - **s&box Ghost Collisions** ports the procedural floor mesh, the velocity-driven
//!   fixed-rotation character, the ghost-launch detection, and the two DrawControls
//!   walk-speed sliders + Reset Counters button exactly (see [`ghost_mesh`]).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::excessive_precision)]

mod ghost_mesh;
mod scenes;
mod wheel_data;

use crate::interact::{self, MouseGrab};
use crate::shell::ZERO_POS;
use crate::vis::{push_poses, VisBody};
use box3d_rust::body::{
    body_get_linear_velocity, body_get_position, body_set_linear_velocity, get_body_transform,
};
use box3d_rust::id::{BodyId, NULL_BODY_ID};
use box3d_rust::math_functions::{Pos, Vec3};
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
    /// Restitution Overshoot bounce tracking (only when that scene is live).
    pub resti: Option<RestitutionTrack>,
    /// s&box Ghost Collisions velocity control + launch tracking (that scene only).
    pub ghost: Option<GhostTrack>,
}

/// s&box Ghost Collisions per-step state (`SBoxGhostCollisions`, sample_issues.cpp:508).
/// The character is driven by pure velocity control each step; any upward velocity
/// spike while grounded is counted as a ghost launch and its position recorded.
pub(super) struct GhostTrack {
    pub character: BodyId,
    /// Half height of the box hull (C `m_bodyHalfHeight`), for the grounded test.
    pub body_half_height: f32,
    pub walk_direction_x: f32,
    pub walk_direction_z: f32,
    pub walk_speed_x: f32,
    pub walk_speed_z: f32,
    pub launch_count: i32,
    pub max_launch_speed: f32,
    pub was_launched: bool,
    /// Recorded launch positions (C `m_launchMarkers`, capacity 64).
    pub launch_markers: Vec<Pos>,
    /// Latest vertical velocity, for the HUD (`DrawTextLine` in C `Step`).
    pub vertical_velocity: f32,
}

// C `SBoxGhostCollisions` static constexpr thresholds (sample_issues.cpp:907-910).
const GHOST_WALK_RANGE_X: f32 = 3.5; // turn around beyond +/- this x (meters)
const GHOST_WALK_RANGE_Z: f32 = 0.5; // turn around beyond +/- this z (meters)
const GHOST_LAUNCH_THRESHOLD: f32 = 0.5; // upward m/s counted as a ghost launch
const GHOST_MARKER_CAPACITY: usize = 64;

/// Restitution Overshoot per-step bounce tracking (`RestitutionOvershoot::Step`,
/// sample_issues.cpp:1190). Mirrors the C member fields so the HUD and the
/// PASS/FAIL verdict match exactly.
pub(super) struct RestitutionTrack {
    pub box_body: BodyId,
    pub drop_height: f32,
    pub box_half: f32,
    pub tolerance: f32,
    pub current_y: f32,
    pub max_bounce_y: f32,
    pub bounced: bool,
    pub failed: bool,
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
            resti: None,
            ghost: None,
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
pub fn issues_reset_crash() -> u32 {
    install(scenes::build_crash())
}

#[wasm_bindgen]
pub fn issues_reset_multiple_prismatic() -> u32 {
    let count = install(scenes::build_multiple_prismatic());
    // C `MultiplePrismatic` sets `m_mouseForceScale = 1e6` (sample_issues.cpp:163)
    // for a much stronger picker pull; re-apply after `install` restores the default.
    interact::set_grab_force_scale(1_000_000.0);
    count
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

#[wasm_bindgen]
pub fn issues_reset_restitution_overshoot() -> u32 {
    install(scenes::build_restitution_overshoot())
}

#[wasm_bindgen]
pub fn issues_reset_slide_twist_off_center() -> u32 {
    install(scenes::build_slide_twist_off_center())
}

#[wasm_bindgen]
pub fn issues_reset_wheel_stack() -> u32 {
    install(scenes::build_wheel_stack())
}

#[wasm_bindgen]
pub fn issues_reset_sbox_ghost() -> u32 {
    install(scenes::build_sbox_ghost())
}

// --- Stepping + render surface --------------------------------------------

#[wasm_bindgen]
pub fn issues_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        // s&box Ghost Collisions drives the character with pure velocity control set
        // just before the step (C `SBoxGhostCollisions::Step` before `Sample::Step`).
        ghost_pre_step(state);
        // Hull Crash is a static render (no simulated bodies); still step the empty
        // world so the shared surface behaves uniformly.
        state.world.step(dt, sub_steps);
        update_restitution(state);
        // Ghost-launch detection reads the post-step velocity (C, after `Sample::Step`).
        ghost_post_step(state);
        state.bodies.len() as u32
    })
}

/// s&box Ghost Collisions velocity control (`SBoxGhostCollisions::Step`, the part
/// before `Sample::Step`): flip the walk direction at the range limits, then overwrite
/// the horizontal velocity while keeping the solver's vertical velocity.
fn ghost_pre_step(state: &mut IssuesState) {
    let Some(g) = state.ghost.as_ref() else {
        return;
    };
    let character = g.character;
    let speed_x = g.walk_speed_x;
    let speed_z = g.walk_speed_z;
    let mut dir_x = g.walk_direction_x;
    let mut dir_z = g.walk_direction_z;

    let position = body_get_position(&state.world, character);
    if position.x as f32 > GHOST_WALK_RANGE_X {
        dir_x = -1.0;
    } else if (position.x as f32) < -GHOST_WALK_RANGE_X {
        dir_x = 1.0;
    }
    if position.z as f32 > GHOST_WALK_RANGE_Z {
        dir_z = -1.0;
    } else if (position.z as f32) < -GHOST_WALK_RANGE_Z {
        dir_z = 1.0;
    }

    let mut velocity = body_get_linear_velocity(&state.world, character);
    velocity.x = dir_x * speed_x;
    velocity.z = dir_z * speed_z;
    body_set_linear_velocity(&mut state.world, character, velocity);

    if let Some(g) = state.ghost.as_mut() {
        g.walk_direction_x = dir_x;
        g.walk_direction_z = dir_z;
    }
}

/// s&box Ghost Collisions launch detection (`SBoxGhostCollisions::Step`, the part after
/// `Sample::Step`): the walkable plane is exactly y = 0, so any upward velocity spike
/// while grounded is a ghost collision. Latch each rising edge, track the worst speed,
/// and record the launch position (up to the marker capacity).
fn ghost_post_step(state: &mut IssuesState) {
    let Some(g) = state.ghost.as_ref() else {
        return;
    };
    let character = g.character;
    let body_half_height = g.body_half_height;
    let was_launched = g.was_launched;

    let position = body_get_position(&state.world, character);
    let velocity = body_get_linear_velocity(&state.world, character);

    let grounded = (position.y as f32) < body_half_height + 0.01 + 4.0 * ghost_mesh::SRC;
    let launched = velocity.y > GHOST_LAUNCH_THRESHOLD;

    let mut new_count = g.launch_count;
    let mut new_max = g.max_launch_speed;
    let mut new_marker: Option<Pos> = None;
    if grounded && launched && !was_launched {
        new_count += 1;
        new_max = new_max.max(velocity.y);
        if g.launch_markers.len() < GHOST_MARKER_CAPACITY {
            new_marker = Some(position);
        }
    }

    if let Some(g) = state.ghost.as_mut() {
        g.launch_count = new_count;
        g.max_launch_speed = new_max;
        if let Some(m) = new_marker {
            g.launch_markers.push(m);
        }
        g.was_launched = launched;
        g.vertical_velocity = velocity.y;
    }
}

/// s&box Ghost Collisions HUD readout, or empty when that scene is not live:
/// `[launchCount, maxLaunchSpeed (m/s), verticalVelocity (m/s)]`. JS derives the
/// inch/s figures (`/ SRC`) for the C `DrawTextLine` text.
#[wasm_bindgen]
pub fn issues_ghost_hud() -> Vec<f32> {
    with_state(|state| match &state.ghost {
        Some(g) => vec![g.launch_count as f32, g.max_launch_speed, g.vertical_velocity],
        None => Vec::new(),
    })
}

/// s&box Ghost Collisions launch markers (C red `DrawPoint`s): flat `[x,y,z]` × N.
#[wasm_bindgen]
pub fn issues_ghost_markers() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        if let Some(g) = &state.ghost {
            for m in &g.launch_markers {
                out.extend_from_slice(&[m.x as f32, m.y as f32, m.z as f32]);
            }
        }
        out
    })
}

/// Set the s&box walk speed along x, in s&box inch/s (C `DrawControls` slider,
/// range 100..=400). Converted to m/s via `SRC`.
#[wasm_bindgen]
pub fn issues_ghost_set_speed_x(inch_per_s: f32) {
    with_state(|state| {
        if let Some(g) = state.ghost.as_mut() {
            g.walk_speed_x = inch_per_s * ghost_mesh::SRC;
        }
    });
}

/// Set the s&box walk speed along z, in s&box inch/s (C `DrawControls` slider,
/// range 10..=100). Converted to m/s via `SRC`.
#[wasm_bindgen]
pub fn issues_ghost_set_speed_z(inch_per_s: f32) {
    with_state(|state| {
        if let Some(g) = state.ghost.as_mut() {
            g.walk_speed_z = inch_per_s * ghost_mesh::SRC;
        }
    });
}

/// Reset the ghost-launch counters (C `DrawControls` "Reset Counters" button).
#[wasm_bindgen]
pub fn issues_ghost_reset_counters() {
    with_state(|state| {
        if let Some(g) = state.ghost.as_mut() {
            g.launch_count = 0;
            g.max_launch_speed = 0.0;
            g.launch_markers.clear();
        }
    });
}

/// `RestitutionOvershoot::Step` (sample_issues.cpp:1190) bounce tracking: record the
/// current height, latch the first upward velocity as the bounce start, track the max
/// bounce height, and fail the moment the box exceeds the drop height + tolerance.
fn update_restitution(state: &mut IssuesState) {
    let Some(track) = state.resti.as_mut() else {
        return;
    };
    if !track.box_body.is_non_null() {
        return;
    }
    let position = box3d_rust::body::body_get_position(&state.world, track.box_body);
    track.current_y = position.y as f32;

    let velocity = box3d_rust::body::body_get_linear_velocity(&state.world, track.box_body);
    if !track.bounced && velocity.y > 0.0 {
        track.bounced = true;
    }

    if track.bounced {
        let py = position.y as f32;
        if py > track.max_bounce_y {
            track.max_bounce_y = py;
        }
        if py > track.drop_height + track.tolerance {
            track.failed = true;
        }
    }
}

/// Restitution Overshoot HUD readout, or empty when that scene is not live:
/// `[dropHeight, currentY, maxBounceY, markerY, bounced, failed]` (the last two are
/// 0/1 flags). `markerY = dropHeight + boxHalf` is the yellow marker-plane height.
#[wasm_bindgen]
pub fn issues_restitution_hud() -> Vec<f32> {
    with_state(|state| match &state.resti {
        Some(t) => vec![
            t.drop_height,
            t.current_y,
            t.max_bounce_y,
            t.drop_height + t.box_half,
            if t.bounced { 1.0 } else { 0.0 },
            if t.failed { 1.0 } else { 0.0 },
        ],
        None => Vec::new(),
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
    spawn_random: issues_spawn_random = |state, spawned| {
        crate::interact::append_spawned_vis(&state.world, &mut state.bodies, spawned);
        crate::interact::spawn_ok_payload(spawned)
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
