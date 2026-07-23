//! Card-stack Stacking scenes: Card House from PEEL (sample_stacking.cpp:15) and
//! Capsule Stack (:150). Shared body builders live in the parent module.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{install, pos, push_box_rot};
use crate::sim_demo::{add_ground, capsule_local_from_centers, SimBody, SimState};
use box3d_rust::body::create_body;
use box3d_rust::geometry::Capsule;
use box3d_rust::math_functions::{make_quat_from_axis_angle, Vec3, PI, VEC3_AXIS_Z};
use box3d_rust::shape::create_capsule_shape;
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use wasm_bindgen::prelude::*;

// --------------------------------------------------------------------------
// Card House (from PEEL) (sample_stacking.cpp:15)
// --------------------------------------------------------------------------

/// Card House (sample_stacking.cpp:92). Thin PEEL cards, friction 0.7. The C
/// comment notes the box hull limits the minimum thickness; the values are ported
/// verbatim (thickness half-extent 0.001).
#[wasm_bindgen]
pub fn sim_reset_card_house() -> u32 {
    install(|sim| {
        add_ground(sim, 10.0);

        let card_height = 0.2f32;
        let card_thickness = 0.001f32;
        let card_depth = 0.1f32;
        let angle0 = 25.0 * PI / 180.0;
        let angle1 = -25.0 * PI / 180.0;
        let angle2 = 0.5 * PI;
        let half = (card_thickness, card_height, card_depth);
        // friction 0.7, rollingResistance 0.05 (sample_stacking.cpp:29-30 at c52908c).
        let put = |sim: &mut SimState, x: f32, y: f32, angle: f32| {
            push_box_rot(
                sim,
                pos(x, y, 0.0),
                make_quat_from_axis_angle(VEC3_AXIS_Z, angle),
                half.0,
                half.1,
                half.2,
                1000.0,
                0.7,
                0.05,
            );
        };

        let mut nb = 5i32;
        let mut z0 = 0.0f32;
        let mut y = card_height - 0.02;
        while nb != 0 {
            let mut z = z0;
            for i in 0..nb {
                if i != nb - 1 {
                    put(sim, z + 0.25, y + card_height - 0.015, angle2);
                }
                put(sim, z, y, angle1);
                z += 0.175;
                put(sim, z, y, angle0);
                z += 0.175;
            }
            y += card_height * 2.0 - 0.03;
            z0 += 0.175;
            nb -= 1;
        }
    })
}

// --------------------------------------------------------------------------
// Capsule Stack (sample_stacking.cpp:226)
// --------------------------------------------------------------------------

/// Capsule Stack (sample_stacking.cpp:226). 20 capsules with `linearZ` + all
/// three angular motion locks (a planar-tumble stack).
#[wasm_bindgen]
pub fn sim_reset_capsule_stack() -> u32 {
    install(|sim| {
        add_ground(sim, 40.0);
        let r = 0.5f32;
        let capsule = Capsule {
            center1: Vec3 {
                x: -1.0,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            radius: r,
        };
        let (local, half) =
            capsule_local_from_centers(capsule.center1, capsule.center2, capsule.radius);

        let mut y = 1.5 * r;
        for _ in 0..20 {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.motion_locks.linear_z = true;
            body_def.motion_locks.angular_x = true;
            body_def.motion_locks.angular_y = true;
            body_def.motion_locks.angular_z = true;
            body_def.position = pos(0.0, y, 0.0);
            let body_id = create_body(&mut sim.world, &body_def);
            let shape_def = default_shape_def();
            create_capsule_shape(&mut sim.world, body_id, &shape_def, &capsule);
            sim.bodies.push(SimBody {
                body_index: body_id.index1 - 1,
                half_extents: half,
                kind: 2,
                local: Some(local),
            });
            y += 2.0 * r;
        }
    })
}
