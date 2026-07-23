//! Live `World::step` demos for the remaining Stacking samples (batch 3b):
//! Card House, Capsule Stack, Cylinder, Cylinder Stack, Dominoes, Wedge, Arch,
//! Double Domino.
//!
//! These reuse the shared [`crate::sim_demo`] `SimState`/`SimBody`/`SIM` machinery
//! (so `sim_step`, `sim_body_poses`, `sim_body_styles`, and the mouse interaction
//! exports work unchanged); this module only adds the per-scene `sim_reset_*`
//! builders plus [`sim_hull_geometry`] for the arbitrary-hull scenes.
//!
//! Split out of `sim_demo.rs` to keep that file under the 800-line gate; the split
//! is by scene builder into a real directory module (`card`, `cylinder`,
//! `dominoes`), exactly as the batch task prescribes. `mod.rs` owns the shared
//! body builders + the hull-triangulation export; each submodule owns its scenes.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

mod card;
mod cylinder;
mod dominoes;
mod edge;

use crate::sim_demo::{new_sim, stop_recording_if_any, with_sim, SimBody, SimState, SIM};
use box3d_rust::body::create_body;
use box3d_rust::core::NULL_INDEX;
use box3d_rust::hull::{make_box_hull, HullData};
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{Pos, Quat};
use box3d_rust::shape::{create_hull_shape, ShapeGeometry};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use wasm_bindgen::prelude::*;

/// `SimBody.kind` value used by this page's arbitrary-convex-hull bodies. The
/// existing kinds are 0 = box, 1 = sphere, 2 = capsule; hull meshes get their
/// triangle geometry from [`sim_hull_geometry`] instead of a parametric shape.
const KIND_HULL: u8 = 4;

fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

/// Create a dynamic box body with an explicit rotation and material, tracked as a
/// `kind = 0` (box) render body. Mirrors the C `b3CreateHullShape` on a
/// `b3MakeBoxHull` with the given `b3ShapeDef` material overrides.
#[allow(clippy::too_many_arguments)]
fn push_box_rot(
    sim: &mut SimState,
    position: Pos,
    rotation: Quat,
    hx: f32,
    hy: f32,
    hz: f32,
    density: f32,
    friction: f32,
    rolling: f32,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = position;
    body_def.rotation = rotation;
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = density;
    shape_def.base_material.friction = friction;
    shape_def.base_material.rolling_resistance = rolling;
    let hull = make_box_hull(hx, hy, hz);
    create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [hx, hy, hz],
        kind: 0,
        local: None,
    });
}

/// Create a dynamic body from an arbitrary convex hull, tracked as a `kind = 4`
/// (hull) render body. Returns the `BodyId` so callers can apply an impulse.
fn push_hull(
    sim: &mut SimState,
    position: Pos,
    rotation: Quat,
    hull: &HullData,
    density: f32,
    friction: f32,
    rolling: f32,
) -> BodyId {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = position;
    body_def.rotation = rotation;
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = density;
    shape_def.base_material.friction = friction;
    shape_def.base_material.rolling_resistance = rolling;
    create_hull_shape(&mut sim.world, body_id, &shape_def, hull);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [1.0, 1.0, 1.0],
        kind: KIND_HULL,
        local: None,
    });
    body_id
}

/// Set the current sim to a freshly built scene and return the body count.
fn install<F: FnOnce(&mut SimState)>(build: F) -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        build(&mut sim);
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

// --------------------------------------------------------------------------
// Hull triangulation for the arbitrary-hull render path
// --------------------------------------------------------------------------

/// Per-body local hull geometry for the current scene, aligned to
/// `sim_body_poses` order. Layout: for each body, one `floatCount` value
/// followed by `floatCount` triangle-vertex floats (`0` for non-hull bodies).
/// The consumer (stacking.ts) builds one `THREE.BufferGeometry` per hull body.
#[wasm_bindgen]
pub fn sim_hull_geometry() -> Vec<f32> {
    with_sim(|sim| {
        let mut out = Vec::new();
        for b in &sim.bodies {
            if b.kind != KIND_HULL {
                out.push(0.0);
                continue;
            }
            let mut sid = sim.world.bodies[b.body_index as usize].head_shape_id;
            let mut tris: Vec<f32> = Vec::new();
            while sid != NULL_INDEX {
                if let ShapeGeometry::Hull(h) = &sim.world.shapes[sid as usize].geometry {
                    tris = crate::vis::hull_triangles(h);
                    break;
                }
                sid = sim.world.shapes[sid as usize].next_shape_id;
            }
            out.push(tris.len() as f32);
            out.extend_from_slice(&tris);
        }
        out
    })
}
