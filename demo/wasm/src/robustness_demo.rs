//! Robustness demos — the four `sample_robustness.cpp` samples:
//!
//! - HighMassRatio1 (:16)      — three box pyramids, each capped by a heavy box
//!   (top density `(j+1)·100`); stresses the solver's mass-ratio handling.
//! - Tiny Pyramid (:75)        — a pyramid of 5 cm boxes (extent 0.025, base 30);
//!   tiny shapes relative to the AABB margin / linear slop.
//! - Overlap Recovery (:131)   — an intentionally overlapping box pyramid rebuilt
//!   live from sliders (`b3World_SetContactTuning` + push-out speed).
//! - Overflow Color Pile (:244)— `CreateOverflowColorPile` (`shared/overflow_color.c`):
//!   a heavy hub ringed by boxes so the constraint-graph color set overflows.
//!
//! Follows the ragdoll page pattern: a single thread-local `RobustnessState`, one
//! `robustness_reset_scene(scene)` entry, and the shared `demo_shell!` grab / spawn
//! / delete plumbing. Ground and every box render as `VisBody` boxes.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::draw_data::shape_styles;
use crate::interact::MouseGrab;
use crate::shell::ZERO_POS;
use crate::vis::{pos, push_poses, VisBody};
use box3d_rust::body::{create_body, destroy_body};
use box3d_rust::hull::make_box_hull;
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{Pos, Vec3};
use box3d_rust::shape::create_hull_shape;
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::{world_get_counters, world_set_contact_tuning, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// Scene identifiers (mirror the four `RegisterSample` rows, C sort order).
const SCENE_HIGH_MASS_RATIO: u32 = 0;
const SCENE_TINY_PYRAMID: u32 = 1;
const SCENE_OVERLAP_RECOVERY: u32 = 2;
const SCENE_OVERFLOW_COLOR_PILE: u32 = 3;

/// C `TinyPyramid::m_extent` (5 cm boxes).
const TINY_EXTENT: f32 = 0.025;
/// C `TinyPyramid` base count.
const TINY_BASE_COUNT: i32 = 30;

/// `CreateOverflowColorPile` ring layout (`shared/overflow_color.c`).
const OVERFLOW_RING_COUNT: i32 = 5;
const OVERFLOW_PER_RING: i32 = 5;

thread_local! {
    static STATE: RefCell<Option<RobustnessState>> = const { RefCell::new(None) };
}

/// Live tunables for Overlap Recovery (C `OverlapRecovery` members).
#[derive(Clone, Copy)]
struct RecoveryParams {
    base_count: i32,
    overlap: f32,
    extent: f32,
    speed: f32,
    hertz: f32,
    damping_ratio: f32,
}

impl Default for RecoveryParams {
    fn default() -> Self {
        // C OverlapRecovery ctor defaults (:144-149).
        Self {
            base_count: 4,
            overlap: 0.25,
            extent: 0.5,
            speed: 3.0,
            hertz: 30.0,
            damping_ratio: 10.0,
        }
    }
}

struct RobustnessState {
    world: World,
    bodies: Vec<VisBody>,
    grab: MouseGrab,
    scene: u32,
    /// Overlap Recovery: the live tunables and the pyramid body ids (rebuilt by
    /// `CreateScene`; the ground box is kept out of this list).
    recovery: RecoveryParams,
    recovery_body_ids: Vec<BodyId>,
    /// Overflow Color Pile: neighbor count for the readout (`data.neighborCount`).
    overflow_neighbor_count: i32,
}

fn with_state<R>(f: impl FnOnce(&mut RobustnessState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("robustness not initialized — call robustness_reset_scene first"))
    })
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

/// C `Sample::AddGroundBox(extent)` (sample.cpp): a box with half-extents
/// `(extent, 1, extent)` centered at `{0, -1, 0}`, top surface at y = 0.
fn add_ground(state: &mut RobustnessState, extent: f32) {
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut state.world, &ground_def);
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape(&mut state.world, ground, &default_shape_def(), &hull.base);
    state
        .bodies
        .push(VisBody::box_body(ground.index1 - 1, extent, 1.0, extent));
}

/// Create one dynamic box at `position` with the given density, tracking a
/// `VisBody`. Returns the created `BodyId`.
fn create_dynamic_box(
    state: &mut RobustnessState,
    position: Pos,
    extent: f32,
    density: f32,
) -> BodyId {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = position;
    let body = create_body(&mut state.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = density;
    let hull = make_box_hull(extent, extent, extent);
    create_hull_shape(&mut state.world, body, &shape_def, &hull.base);
    state
        .bodies
        .push(VisBody::box_body(body.index1 - 1, extent, extent, extent));
    body
}

// ---------------------------------------------------------------------------
// Scene builders
// ---------------------------------------------------------------------------

/// HighMassRatio1 (:16): three box pyramids. Each pyramid's apex box carries
/// density `(j+1)·100` (100 / 200 / 300) over unit-density bodies below, so the
/// solver holds a 100:1 – 300:1 mass ratio without the stack collapsing.
fn build_high_mass_ratio(state: &mut RobustnessState) {
    add_ground(state, 50.0);
    let extent = 1.0f32;
    for j in 0..3 {
        let mut count = 10;
        let offset = -20.0 * extent + 2.0 * (count as f32 + 1.0) * extent * j as f32;
        let mut y = extent;
        while count > 0 {
            for i in 0..count {
                let coeff = i as f32 - 0.5 * count as f32;
                let yy = if count == 1 { y + 2.0 } else { y };
                let position = pos(2.0 * coeff * extent + offset, yy, 0.0);
                let density = if count == 1 {
                    (j as f32 + 1.0) * 100.0
                } else {
                    1.0
                };
                create_dynamic_box(state, position, extent, density);
            }
            count -= 1;
            y += 2.0 * extent;
        }
    }
}

/// TinyPyramid (:75): a `TINY_BASE_COUNT`-wide pyramid of 5 cm boxes. Stacking
/// shapes this small is hard because the AABB margin and linear slop are close to
/// the box size.
fn build_tiny_pyramid(state: &mut RobustnessState) {
    add_ground(state, 20.0);
    let extent = TINY_EXTENT;
    let base_count = TINY_BASE_COUNT;
    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * extent;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent
                - base_count as f32 * extent;
            create_dynamic_box(state, pos(x, y, 0.0), extent, 1.0);
        }
    }
}

/// OverlapRecovery `CreateScene` (:161): destroy the previous pyramid, apply the
/// contact tuning (hertz / damping / push-out speed), then build an intentionally
/// overlapping pyramid (`fraction = 1 - overlap` shrinks the lattice spacing so
/// boxes interpenetrate on the first step). The ground box is preserved.
fn build_overlap_recovery(state: &mut RobustnessState) {
    // Destroy the previous pyramid bodies and drop them from the render list.
    let ids = std::mem::take(&mut state.recovery_body_ids);
    for id in &ids {
        destroy_body(&mut state.world, *id);
    }
    let removed: std::collections::HashSet<i32> = ids.iter().map(|id| id.index1 - 1).collect();
    state.bodies.retain(|b| !removed.contains(&b.body_index));

    let p = state.recovery;
    world_set_contact_tuning(&mut state.world, p.hertz, p.damping_ratio, p.speed);

    let extent = p.extent;
    let fraction = 1.0 - p.overlap;
    let mut new_ids = Vec::new();
    let mut y = extent;
    for i in 0..p.base_count {
        let mut x = fraction * extent * (i - p.base_count) as f32;
        for _j in i..p.base_count {
            // C `bodyDef.position = { x, y }` leaves z = 0.
            let id = create_dynamic_box(state, pos(x, y, 0.0), extent, 1.0);
            new_ids.push(id);
            x += 2.0 * fraction * extent;
        }
        y += 2.0 * fraction * extent;
    }
    state.recovery_body_ids = new_ids;
}

/// `CreateOverflowColorPile` (`shared/overflow_color.c`): a static ground, a tall
/// heavy hub (density 50), and `OVERFLOW_RING_COUNT × OVERFLOW_PER_RING` small
/// boxes arranged in vertical rings that slightly overlap the hub, forcing enough
/// simultaneous contacts on one body to overflow the constraint-graph colors.
fn build_overflow_color_pile(state: &mut RobustnessState) {
    // Static ground (top surface at y = 0). overflow_color.c uses a 20×1×20 box.
    let mut ground_def = default_body_def();
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut state.world, &ground_def);
    let ground_hull = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(
        &mut state.world,
        ground,
        &default_shape_def(),
        &ground_hull.base,
    );
    state
        .bodies
        .push(VisBody::box_body(ground.index1 - 1, 20.0, 1.0, 20.0));

    // Tall, heavy hub.
    let hub_half_x = 0.5f32;
    let hub_half_y = 2.5f32;
    let hub_half_z = 0.5f32;
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(0.0, hub_half_y, 0.0);
        let hub = create_body(&mut state.world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.density = 50.0;
        let hull = make_box_hull(hub_half_x, hub_half_y, hub_half_z);
        create_hull_shape(&mut state.world, hub, &shape_def, &hull.base);
        state.bodies.push(VisBody::box_body(
            hub.index1 - 1,
            hub_half_x,
            hub_half_y,
            hub_half_z,
        ));
    }

    // Neighbors: vertical rings around the hub, each box slightly overlapping it.
    let neighbor_half = 0.2f32;
    let ring_radius = hub_half_x + neighbor_half - 0.03;
    let ring_spacing = 0.5f32;
    let base_y = neighbor_half + 0.05;
    let neighbor_shape = default_shape_def();
    let neighbor_hull = make_box_hull(neighbor_half, neighbor_half, neighbor_half);

    for ring in 0..OVERFLOW_RING_COUNT {
        let y = base_y + ring_spacing * ring as f32;
        // Offset alternate rings by half a slot.
        let theta_offset = if ring & 1 != 0 {
            std::f32::consts::PI / OVERFLOW_PER_RING as f32
        } else {
            0.0
        };
        for slot in 0..OVERFLOW_PER_RING {
            let theta = theta_offset
                + (2.0 * std::f32::consts::PI * slot as f32) / OVERFLOW_PER_RING as f32;
            // C uses libm cosf/sinf for the ring placement; f32 cos/sin is the
            // closest match (this is one-time scene setup, not solver math).
            let position = pos(ring_radius * theta.cos(), y, ring_radius * theta.sin());
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = position;
            let body = create_body(&mut state.world, &body_def);
            create_hull_shape(&mut state.world, body, &neighbor_shape, &neighbor_hull.base);
            state.bodies.push(VisBody::box_body(
                body.index1 - 1,
                neighbor_half,
                neighbor_half,
                neighbor_half,
            ));
        }
    }

    state.overflow_neighbor_count = OVERFLOW_RING_COUNT * OVERFLOW_PER_RING;
}

// ---------------------------------------------------------------------------
// wasm exports
// ---------------------------------------------------------------------------

/// Reset to `scene` (0 HighMassRatio1, 1 Tiny Pyramid, 2 Overlap Recovery,
/// 3 Overflow Color Pile). Returns the body count.
#[wasm_bindgen]
pub fn robustness_reset_scene(scene: u32) -> u32 {
    STATE.with(|cell| {
        let mut state = RobustnessState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
            scene,
            recovery: RecoveryParams::default(),
            recovery_body_ids: Vec::new(),
            overflow_neighbor_count: 0,
        };
        match scene {
            SCENE_HIGH_MASS_RATIO => build_high_mass_ratio(&mut state),
            SCENE_TINY_PYRAMID => build_tiny_pyramid(&mut state),
            SCENE_OVERLAP_RECOVERY => {
                add_ground(&mut state, 20.0);
                build_overlap_recovery(&mut state);
            }
            SCENE_OVERFLOW_COLOR_PILE => build_overflow_color_pile(&mut state),
            _ => build_high_mass_ratio(&mut state),
        }
        let n = state.bodies.len() as u32;
        *cell.borrow_mut() = Some(state);
        n
    })
}

/// Overlap Recovery sliders (C `DrawControls`, :204). Applies the tuning and
/// rebuilds the overlapping pyramid, matching C's `CreateScene` on any change.
/// No-op for the other three scenes.
#[wasm_bindgen]
pub fn robustness_set_overlap_params(
    extent: f32,
    base_count: i32,
    overlap: f32,
    speed: f32,
    hertz: f32,
    damping_ratio: f32,
) {
    with_state(|state| {
        if state.scene != SCENE_OVERLAP_RECOVERY {
            return;
        }
        state.recovery = RecoveryParams {
            base_count,
            overlap,
            extent,
            speed,
            hertz,
            damping_ratio,
        };
        build_overlap_recovery(state);
    });
}

#[wasm_bindgen]
pub fn robustness_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.bodies.len() as u32
    })
}

/// Pose buffer: 16 floats per body (see `vis` module).
#[wasm_bindgen]
pub fn robustness_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

/// Packed engine-driven style words parallel to [`robustness_poses`].
#[wasm_bindgen]
pub fn robustness_styles() -> Vec<u32> {
    with_state(|state| shape_styles(&mut state.world, &state.bodies))
}

#[wasm_bindgen]
pub fn robustness_body_count() -> u32 {
    with_state(|state| state.bodies.len() as u32)
}

/// Overflow Color Pile readout (C `OverflowColorPile::Step`, :261):
/// `[neighbor_count, overflow_contacts, total_contacts]`. `overflow_contacts` is
/// `colorCounts[B3_GRAPH_COLOR_COUNT - 1]` (the overflow color bucket).
#[wasm_bindgen]
pub fn robustness_overflow_stats() -> Vec<f32> {
    with_state(|state| {
        let counters = world_get_counters(&state.world);
        let overflow_index = box3d_rust::constants::GRAPH_COLOR_COUNT as usize - 1;
        vec![
            state.overflow_neighbor_count as f32,
            counters.color_counts[overflow_index] as f32,
            counters.contact_count as f32,
        ]
    })
}

/// Tiny Pyramid box edge length in centimetres (C label "%.1fcm boxes",
/// `200 * m_extent`).
#[wasm_bindgen]
pub fn robustness_tiny_cm() -> f32 {
    200.0 * TINY_EXTENT
}

crate::demo_shell! {
    with_state: with_state,
    state: RobustnessState,
    world: world,
    bodies: bodies,
    grab: grab,
    base: |_s| ZERO_POS,
    mouse_down: robustness_mouse_down,
    mouse_move: robustness_mouse_move,
    mouse_up: robustness_mouse_up,
    mouse_active: robustness_mouse_active,
    spawn_random: robustness_spawn_random = |state, spawned| {
        crate::interact::append_spawned_vis(&state.world, &mut state.bodies, spawned);
        match spawned.first() {
            Some(sp) => vec![1.0, sp.body_index as f32],
            None => vec![0.0, 0.0],
        }
    },
    delete_at_ray: robustness_delete_at_ray = |state, index| {
        state.recovery_body_ids.retain(|id| id.index1 - 1 != index);
    },
    counters: robustness_counters,
    debug_draw: robustness_debug_draw,
    debug_text: robustness_debug_text,
}

crate::demo_world_toggles! {
    with_state: with_state,
    world: world,
    set_enable_sleep: robustness_set_enable_sleep,
    set_enable_warm_starting: robustness_set_enable_warm_starting,
    set_enable_continuous: robustness_set_enable_continuous,
    set_recycle_distance: robustness_set_recycle_distance,
}
