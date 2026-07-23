//! Edge Crossing Stacking scene (`sample_stacking.cpp` :852). Three rows of thin
//! boxes dropped at crossing angles onto stacked base boxes. Shared body builders
//! live in the parent module.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{install, pos, push_box_rot};
use crate::sim_demo::{add_ground, SimState};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, normalize, Quat, Vec3, PI, QUAT_IDENTITY,
};
use wasm_bindgen::prelude::*;

/// Edge Crossing (`sample_stacking.cpp` :852). For each of three rows a base box
/// rests near the ground and a second box is dropped from `20 × h` above it,
/// rotated by every `0.1·π` step across `[-π, π]`, so their edges cross. Rows use
/// two thin box shapes: `box1 = (h.x, h.y, h.z)` and `box2 = (h.x, h.z, h.y)`.
/// Values are ported verbatim; every box uses the default material (friction 0.6,
/// density 1000).
#[wasm_bindgen]
pub fn sim_reset_edge_crossing() -> u32 {
    install(|sim| {
        add_ground(sim, 40.0);

        // h = { 0.2, 0.02, 0.04 } (sample_stacking.cpp :864).
        let h = Vec3 {
            x: 0.2,
            y: 0.02,
            z: 0.04,
        };
        // box1 = (h.x, h.y, h.z); box2 = (h.x, h.z, h.y).
        let box1 = (h.x, h.y, h.z);
        let box2 = (h.x, h.z, h.y);

        let axis = normalize(Vec3 {
            x: 0.1,
            y: 0.9,
            z: 0.0,
        });

        // One row: a base box (identity, at `base_y`) plus a dropped box (rotated,
        // at `20·base_y`) for every `0.1·π` angle across `[-π, π]`, marching +1 in x.
        let put = |sim: &mut SimState, x: f32, y: f32, z: f32, rot: Quat, dims: (f32, f32, f32)| {
            push_box_rot(
                sim,
                pos(x, y, z),
                rot,
                dims.0,
                dims.1,
                dims.2,
                1000.0,
                0.6,
                0.0,
            );
        };
        let row = |sim: &mut SimState,
                   z: f32,
                   base_dims: (f32, f32, f32),
                   base_y: f32,
                   drop_dims: (f32, f32, f32),
                   drop_y: f32| {
            let mut x = -10.0f32;
            let mut angle = -PI;
            while angle < PI + 0.001 {
                put(sim, x, base_y, z, QUAT_IDENTITY, base_dims);
                put(
                    sim,
                    x,
                    drop_y,
                    z,
                    make_quat_from_axis_angle(axis, angle),
                    drop_dims,
                );
                x += 1.0;
                angle += 0.1 * PI;
            }
        };

        // Row 1 (z = -2): base box1 at h.y, dropped box1 at 20·h.y.
        row(sim, -2.0, box1, h.y, box1, 20.0 * h.y);
        // Row 2 (z = 0): base box2 at h.z, dropped box2 at 20·h.z.
        row(sim, 0.0, box2, h.z, box2, 20.0 * h.z);
        // Row 3 (z = 2): base box1 at h.y, dropped box2 at 20·h.y.
        row(sim, 2.0, box1, h.y, box2, 20.0 * h.y);
    })
}
