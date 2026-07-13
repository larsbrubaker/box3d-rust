//! Card-stack Stacking scenes: Card House Thick (sample_stacking.cpp:13), Card
//! House from PEEL (:92), and Capsule Stack (:226). Shared body builders live in
//! the parent module.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{install, pos, push_box_rot};
use crate::sim_demo::{add_ground, capsule_local_from_centers, SimBody, SimState};
use box3d_rust::body::create_body;
use box3d_rust::geometry::Capsule;
use box3d_rust::math_functions::{
    cos, make_quat_from_axis_angle, sin, Vec3, DEG_TO_RAD, PI, VEC3_AXIS_Z,
};
use box3d_rust::shape::create_capsule_shape;
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use wasm_bindgen::prelude::*;

// --------------------------------------------------------------------------
// Card House Thick (sample_stacking.cpp:13)
// --------------------------------------------------------------------------

fn card_house_thick_vertical_row(
    sim: &mut SimState,
    n: i32,
    start_x: f32,
    offset_x: f32,
    start_y: f32,
    alpha: f32,
    half: (f32, f32, f32),
) {
    let mut start_x = start_x;
    for _ in 0..n {
        // friction 0.8, default density/rolling (sample_stacking.cpp:49).
        push_box_rot(
            sim,
            pos(start_x - offset_x, start_y, 0.0),
            make_quat_from_axis_angle(VEC3_AXIS_Z, -alpha),
            half.0,
            half.1,
            half.2,
            1000.0,
            0.8,
            0.0,
        );
        push_box_rot(
            sim,
            pos(start_x + offset_x, start_y, 0.0),
            make_quat_from_axis_angle(VEC3_AXIS_Z, alpha),
            half.0,
            half.1,
            half.2,
            1000.0,
            0.8,
            0.0,
        );
        start_x += 4.0 * offset_x;
    }
}

fn card_house_thick_horizontal_row(
    sim: &mut SimState,
    n: i32,
    start_x: f32,
    offset_x: f32,
    start_y: f32,
    half: (f32, f32, f32),
) {
    for index in 0..n {
        push_box_rot(
            sim,
            pos(start_x + index as f32 * offset_x, start_y, 0.0),
            make_quat_from_axis_angle(VEC3_AXIS_Z, 0.5 * PI),
            half.0,
            half.1,
            half.2,
            1000.0,
            0.8,
            0.0,
        );
    }
}

/// Card House Thick (sample_stacking.cpp:13). 25° hulls, friction 0.8, 7
/// alternating vertical/horizontal rows.
#[wasm_bindgen]
pub fn sim_reset_card_house_thick() -> u32 {
    install(|sim| {
        add_ground(sim, 10.0);
        let alpha = 25.0 * DEG_TO_RAD;
        let width = 0.38f32;
        let height = 0.98f32;
        let depth = 0.08f32;
        let offset_x = 0.5 * height * sin(alpha) + 0.045;
        let offset_y = 0.5 * height * cos(alpha) + 0.035;
        let half = (0.5 * depth, 0.5 * height, 0.5 * width);

        card_house_thick_vertical_row(sim, 4, -6.0 * offset_x, offset_x, offset_y, alpha, half);
        card_house_thick_horizontal_row(
            sim,
            3,
            -4.0 * offset_x,
            4.0 * offset_x,
            2.0 * offset_y + 0.04,
            half,
        );
        card_house_thick_vertical_row(
            sim,
            3,
            -4.0 * offset_x,
            offset_x,
            3.0 * offset_y + 0.08,
            alpha,
            half,
        );
        card_house_thick_horizontal_row(
            sim,
            2,
            -2.0 * offset_x,
            4.0 * offset_x,
            4.0 * offset_y + 0.12,
            half,
        );
        card_house_thick_vertical_row(
            sim,
            2,
            -2.0 * offset_x,
            offset_x,
            5.0 * offset_y + 0.16,
            alpha,
            half,
        );
        card_house_thick_horizontal_row(
            sim,
            1,
            -0.0 * offset_x,
            4.0 * offset_x,
            6.0 * offset_y + 0.20,
            half,
        );
        card_house_thick_vertical_row(
            sim,
            1,
            -0.0 * offset_x,
            offset_x,
            7.0 * offset_y + 0.24,
            alpha,
            half,
        );
    })
}

// --------------------------------------------------------------------------
// Card House (from PEEL) (sample_stacking.cpp:92)
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
        // friction 0.7 (sample_stacking.cpp:106).
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
                0.0,
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
