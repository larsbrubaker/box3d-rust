//! Compound Village WASM bindings (C `sample_compound.cpp` Village).
//!
//! Split from `sim_compound` to keep files under the 800-line limit.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::sim_demo::{capsule_local_from_centers, new_sim, stop_recording_if_any, SimBody, SIM};
use box3d_rust::body::get_body_transform;
use box3d_rust::math_functions::{mul_transforms, Pos, Transform, Vec3, QUAT_IDENTITY};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    /// Compound Village character mover + sweeping query visualization (C Village
    /// embeds a `CharacterMover`). `None` unless the Village scene is active.
    static VILLAGE_MOVER: RefCell<Option<crate::village::VillageMover>> =
        const { RefCell::new(None) };
}

/// Compound / Village — C `sample_compound.cpp` Village with real `building.obj` meshes.
///
/// C uses `gridCount = 8` (debug) / `200` (release). 200 buildings × compound is
/// far too heavy for the serial wasm build, so this fixes the grid at the C debug
/// value 8. The C sample has no grid control, so this takes no argument.
#[wasm_bindgen]
pub fn sim_reset_village() -> u32 {
    // C Village debug build uses gridCount = 8 (release uses 200).
    let grid = 8i32;
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        let village = crate::village::build_village(&mut sim.world, grid);
        let a = village.tile_half;
        let parent_index = village.ground_body_index;

        for xf in &village.hull_transforms {
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [a, 0.5 * a, a],
                kind: 0,
                local: Some(*xf),
            });
        }
        for s in &village.spheres {
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [s.sphere.radius, s.sphere.radius, s.sphere.radius],
                kind: 1,
                local: Some(Transform {
                    p: s.sphere.center,
                    q: QUAT_IDENTITY,
                }),
            });
        }
        for c in &village.capsules {
            let (local, half) =
                capsule_local_from_centers(c.capsule.center1, c.capsule.center2, c.capsule.radius);
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: half,
                kind: 2,
                local: Some(local),
            });
        }

        sim.village_buildings = village.buildings;
        sim.village_stats = village.stats;
        sim.village_ground_index = parent_index;

        // C Village :502 sets launchSpeedScale = 2 (bullets spawn at half speed).
        crate::interact::set_launch_speed_scale(2.0);

        // C Village embeds a CharacterMover (start {0,10,0}) plus a sweeping
        // ray/shape/overlap query visualization; `m_worldWidth = 2*gridCount*a`.
        let world_width = 2.0 * grid as f32 * a;
        VILLAGE_MOVER.with(|m| {
            *m.borrow_mut() = Some(crate::village::VillageMover::new(
                Pos {
                    x: 0.0 as _,
                    y: 10.0 as _,
                    z: 0.0 as _,
                },
                world_width,
            ));
        });

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Feed the Village mover WASD throttle / jump / sprint and camera-relative axes.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn sim_village_set_input(
    throttle_x: f32,
    throttle_y: f32,
    jump: bool,
    sprint: bool,
    fwd_x: f32,
    fwd_z: f32,
    right_x: f32,
    right_z: f32,
) {
    VILLAGE_MOVER.with(|m| {
        if let Some(mover) = m.borrow_mut().as_mut() {
            mover.set_input(
                throttle_x, throttle_y, jump, sprint, fwd_x, fwd_z, right_x, right_z,
            );
        }
    });
}

/// Integrate the Village mover one step against the sim world (does not step the
/// world — call before `sim_step`, mirroring C's `mover.Step` before `Sample::Step`).
#[wasm_bindgen]
pub fn sim_village_mover_step(dt: f32) {
    VILLAGE_MOVER.with(|m| {
        let mut slot = m.borrow_mut();
        let Some(mover) = slot.as_mut() else {
            return;
        };
        SIM.with(|cell| {
            if let Some(sim) = cell.borrow_mut().as_mut() {
                mover.solve_move(&mut sim.world, dt);
            }
        });
    });
}

/// Advance the Village sweeping query one step (ray/shape/overlap visualization).
/// Pass `dt = 0` when paused so the sweep freezes (matching C).
#[wasm_bindgen]
pub fn sim_village_query_step(dt: f32) {
    VILLAGE_MOVER.with(|m| {
        let mut slot = m.borrow_mut();
        let Some(mover) = slot.as_mut() else {
            return;
        };
        SIM.with(|cell| {
            if let Some(sim) = cell.borrow().as_ref() {
                mover.query(&sim.world, dt);
            }
        });
    });
}

/// Village mover capsule pose: `[px,py,pz, c1x,c1y,c1z, c2x,c2y,c2z, radius]`
/// (empty when the Village scene is not active).
#[wasm_bindgen]
pub fn sim_village_mover_pose() -> Vec<f32> {
    VILLAGE_MOVER.with(|m| {
        m.borrow()
            .as_ref()
            .map(|mover| {
                vec![
                    mover.mover_pos.x as f32,
                    mover.mover_pos.y as f32,
                    mover.mover_pos.z as f32,
                    mover.capsule.center1.x,
                    mover.capsule.center1.y,
                    mover.capsule.center1.z,
                    mover.capsule.center2.x,
                    mover.capsule.center2.y,
                    mover.capsule.center2.z,
                    mover.capsule.radius,
                ]
            })
            .unwrap_or_default()
    })
}

/// Village sweeping-query visualization (see `village::VillageMover::query` for the
/// 39-float layout: ray, sphere shape cast, and overlap-shape data).
#[wasm_bindgen]
pub fn sim_village_query() -> Vec<f32> {
    VILLAGE_MOVER.with(|m| {
        m.borrow()
            .as_ref()
            .map(|mover| mover.query_viz().to_vec())
            .unwrap_or_default()
    })
}

/// Toggle the Village third-person camera flag (C 'T' key). Returns the new state.
#[wasm_bindgen]
pub fn sim_village_toggle_third_person() -> bool {
    VILLAGE_MOVER.with(|m| {
        if let Some(mover) = m.borrow_mut().as_mut() {
            mover.toggle_third_person();
            mover.third_person
        } else {
            false
        }
    })
}

/// Village building instances in world space:
/// `[px,py,pz, qx,qy,qz,qw, sx,sy,sz] * N` (matches C compound mesh children).
#[wasm_bindgen]
pub fn sim_village_buildings() -> Vec<f32> {
    SIM.with(|cell| {
        let slot = cell.borrow();
        let Some(sim) = slot.as_ref() else {
            return Vec::new();
        };
        if sim.village_ground_index < 0 || sim.village_buildings.is_empty() {
            return Vec::new();
        }
        let parent = get_body_transform(&sim.world, sim.village_ground_index);
        let parent_xf = Transform {
            p: Vec3 {
                x: parent.p.x as f32,
                y: parent.p.y as f32,
                z: parent.p.z as f32,
            },
            q: parent.q,
        };
        let mut out = Vec::with_capacity(sim.village_buildings.len() * 10);
        for b in &sim.village_buildings {
            let world_xf = mul_transforms(parent_xf, b.transform);
            out.push(world_xf.p.x);
            out.push(world_xf.p.y);
            out.push(world_xf.p.z);
            out.push(world_xf.q.v.x);
            out.push(world_xf.q.v.y);
            out.push(world_xf.q.v.z);
            out.push(world_xf.q.s);
            out.push(b.scale.x);
            out.push(b.scale.y);
            out.push(b.scale.z);
        }
        out
    })
}

/// Village compound stats: `[capsules, hulls, meshes, spheres, byte_count, tree_bytes, tree_height]`.
#[wasm_bindgen]
pub fn sim_village_stats() -> Vec<f32> {
    SIM.with(|cell| {
        cell.borrow()
            .as_ref()
            .map(|s| s.village_stats.to_vec())
            .unwrap_or_default()
    })
}
