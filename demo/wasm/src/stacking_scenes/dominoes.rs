//! Domino Stacking scenes: Dominoes (sample_stacking.cpp:574) and Double Domino
//! (:841). Shared body builders live in the parent module.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{install, pos};
use crate::sim_demo::{add_ground, SimBody};
use box3d_rust::body::{body_apply_linear_impulse, create_body};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    compute_cos_sin, make_quat_from_axis_angle, Vec3, DEG_TO_RAD, VEC3_AXIS_Y,
};
use box3d_rust::shape::create_hull_shape;
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use wasm_bindgen::prelude::*;

// --------------------------------------------------------------------------
// Dominoes (sample_stacking.cpp:574)
// --------------------------------------------------------------------------

/// Dominoes (sample_stacking.cpp:574). 30 concentric rings of thin box dominoes;
/// the first domino of each ring at `alpha == 0` gets a linear impulse kick that
/// topples the ring inward.
#[wasm_bindgen]
pub fn sim_reset_dominoes() -> u32 {
    install(|sim| {
        add_ground(sim, 80.0);
        // Release build uses n = 30 (debug would be 2).
        for ring in 0..30 {
            let radius = 7.0 + 1.1 * ring as f32;
            // b3ComputeCosSin sweep, alpha 0..=360 step 2 (181 dominoes/ring).
            let mut alpha = 0.0f32;
            while alpha <= 360.0 {
                let cs = compute_cos_sin(DEG_TO_RAD * alpha);
                let normal = Vec3 {
                    x: cs.cosine,
                    y: 0.0,
                    z: cs.sine,
                };
                let px = radius * cs.cosine - alpha / 630.0 * normal.x;
                let py = 0.8 - alpha / 630.0 * normal.y;
                let pz = radius * cs.sine - alpha / 630.0 * normal.z;
                let orientation = make_quat_from_axis_angle(VEC3_AXIS_Y, -DEG_TO_RAD * alpha);
                let mut body_def = default_body_def();
                body_def.type_ = BodyType::Dynamic;
                body_def.position = pos(px, py, pz);
                body_def.rotation = orientation;
                let body_id = create_body(&mut sim.world, &body_def);
                let shape_def = default_shape_def();
                let hull = make_box_hull(0.2, 0.8, 0.05);
                create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
                sim.bodies.push(SimBody {
                    body_index: body_id.index1 - 1,
                    half_extents: [0.2, 0.8, 0.05],
                    kind: 0,
                    local: None,
                });
                if alpha == 0.0 {
                    body_apply_linear_impulse(
                        &mut sim.world,
                        body_id,
                        Vec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 25.0,
                        },
                        pos(px, py + 0.8, pz),
                        true,
                    );
                }
                alpha += 2.0;
            }
        }
    })
}

// --------------------------------------------------------------------------
// Double Domino (sample_stacking.cpp:841)
// --------------------------------------------------------------------------

/// Double Domino (sample_stacking.cpp:841). 15 dominoes, friction 0.6, density 4;
/// the first domino gets a small linear impulse to start the chain.
#[wasm_bindgen]
pub fn sim_reset_double_domino() -> u32 {
    install(|sim| {
        add_ground(sim, 20.0);
        let count = 15;
        let mut x = -0.5 * count as f32;
        for i in 0..count {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = pos(x, 0.5, 0.0);
            let body_id = create_body(&mut sim.world, &body_def);
            let mut shape_def = default_shape_def();
            shape_def.base_material.friction = 0.6;
            shape_def.density = 4.0;
            let hull = make_box_hull(0.125, 0.5, 0.25);
            create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
            sim.bodies.push(SimBody {
                body_index: body_id.index1 - 1,
                half_extents: [0.125, 0.5, 0.25],
                kind: 0,
                local: None,
            });
            if i == 0 {
                body_apply_linear_impulse(
                    &mut sim.world,
                    body_id,
                    Vec3 {
                        x: 0.2,
                        y: 0.0,
                        z: 0.0,
                    },
                    pos(x, 1.0, 0.0),
                    true,
                );
            }
            x += 1.01;
        }
    })
}
