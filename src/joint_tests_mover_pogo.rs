//! Mover and pogo joint sub-tests from test_joint.c (TestMoverJoint, TestPogoJoint). They share
//! the joint fixture in joint_tests.rs and live in their own file for the file-length limit.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{create_joint_fixture, exercise_joint_base, finish_joint, set_common_frames};
use crate::body::{body_get_angular_velocity, body_get_linear_velocity, body_get_mass};
use crate::joint::*;
use crate::math_functions::{normalize, Vec3, VEC3_AXIS_Y};
use crate::types::{default_mover_joint_def, default_pogo_joint_def};
use crate::world::world_set_gravity;

/// (ENSURE_SMALL)
fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

/// (TestMoverJoint)
#[test]
fn test_mover_joint() {
    let mut f = create_joint_fixture();

    let mut def = default_mover_joint_def();
    set_common_frames(&mut def.base, &f);
    let joint_id = create_mover_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Mover,
    );

    let velocity = Vec3 {
        x: 1.0,
        y: 2.0,
        z: -3.0,
    };
    mover_joint_set_linear_velocity(&mut f.world, joint_id, velocity);
    let got_velocity = mover_joint_get_linear_velocity(&f.world, joint_id);
    assert!(got_velocity.x == 1.0 && got_velocity.y == 2.0 && got_velocity.z == -3.0);

    // The y axis has no force budget, so it must stay at rest while x and z reach the target
    let max_force = 1000.0;
    mover_joint_set_max_velocity_force(&mut f.world, joint_id, max_force);
    let got_force = mover_joint_get_max_velocity_force(&f.world, joint_id);
    assert!(got_force == 1000.0);

    for _ in 0..30 {
        f.world.step(1.0 / 60.0, 4);
    }

    let v = body_get_linear_velocity(&f.world, f.body_id);
    ensure_small(v.x - 1.0, 1.0e-4);
    ensure_small(v.y, 1.0e-4);
    ensure_small(v.z + 3.0, 1.0e-4);

    let w = body_get_angular_velocity(&f.world, f.body_id);
    assert!(w.x == 0.0 && w.y == 0.0 && w.z == 0.0);

    finish_joint(&mut f.world, joint_id);
}

/// A box held up by a pogo spring under gravity should settle at the rest length
/// (TestPogoJoint)
#[test]
fn test_pogo_joint() {
    let mut f = create_joint_fixture();
    world_set_gravity(
        &mut f.world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let rest_length = 2.0;
    let mass = body_get_mass(&f.world, f.body_id);

    let mut def = default_pogo_joint_def();
    def.base.body_id_a = f.ground_id;
    def.base.body_id_b = f.body_id;
    def.base.local_frame_a.p = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    def.base.local_frame_b.p = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    def.normal = VEC3_AXIS_Y;
    def.rest_length = rest_length;
    def.hertz = 2.0;
    def.damping_ratio = 1.0;
    def.max_compression_force = 100.0 * 10.0 * mass;
    def.max_tension_force = 100.0 * 10.0 * mass;
    let joint_id = create_pogo_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Pogo,
    );

    pogo_joint_set_rest_length(&mut f.world, joint_id, rest_length);
    assert!(pogo_joint_get_rest_length(&f.world, joint_id) == rest_length);

    pogo_joint_set_spring_hertz(&mut f.world, joint_id, 3.0);
    assert!(pogo_joint_get_spring_hertz(&f.world, joint_id) == 3.0);

    pogo_joint_set_spring_damping_ratio(&mut f.world, joint_id, 0.9);
    assert!(pogo_joint_get_spring_damping_ratio(&f.world, joint_id) == 0.9);

    let max_tension = 50.0 * 10.0 * mass;
    pogo_joint_set_max_tension_force(&mut f.world, joint_id, max_tension);
    assert!(pogo_joint_get_max_tension_force(&f.world, joint_id) == max_tension);

    let max_compression = 60.0 * 10.0 * mass;
    pogo_joint_set_max_compression_force(&mut f.world, joint_id, max_compression);
    assert!(pogo_joint_get_max_compression_force(&f.world, joint_id) == max_compression);

    let normal = normalize(Vec3 {
        x: 0.2,
        y: 1.0,
        z: 0.0,
    });
    pogo_joint_set_normal(&mut f.world, joint_id, normal);
    let got_normal = pogo_joint_get_normal(&f.world, joint_id);
    assert!(got_normal.x == normal.x && got_normal.y == normal.y && got_normal.z == normal.z);
    pogo_joint_set_normal(&mut f.world, joint_id, VEC3_AXIS_Y);

    // Starts stretched by one meter
    ensure_small(
        pogo_joint_get_current_length(&f.world, joint_id) - 3.0,
        1.0e-5,
    );

    for _ in 0..300 {
        f.world.step(1.0 / 60.0, 4);
    }

    ensure_small(
        pogo_joint_get_current_length(&f.world, joint_id) - rest_length,
        0.01,
    );
    ensure_small(body_get_linear_velocity(&f.world, f.body_id).y, 0.01);

    // Holding the weight takes an impulse of m * g * h each sub-step
    ensure_small(
        pogo_joint_get_impulse(&f.world, joint_id) - mass * 10.0 / 240.0,
        0.001 * mass,
    );

    let force = joint_get_constraint_force(&f.world, joint_id);
    ensure_small(force.y - mass * 10.0, 0.1 * mass);

    finish_joint(&mut f.world, joint_id);
}
