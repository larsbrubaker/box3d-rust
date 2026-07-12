//! Driving sample — chassis + sphere wheels on a wave height field.

use crate::joint_demo::{empty_state, new_world, JointScene, JointState};
use crate::vis::{pos, sphere, VisBody};
use box3d_rust::body::{body_set_awake, create_body};
use box3d_rust::height_field::{
    create_wave, get_height_field_triangle, get_height_field_triangle_count,
};
use box3d_rust::hull::make_box_hull;
use box3d_rust::joint::{
    create_parallel_joint, create_wheel_joint, wheel_joint_set_max_spin_torque,
    wheel_joint_set_spin_motor_speed, wheel_joint_set_target_steering_angle,
};
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, Vec3, PI, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z,
};
use box3d_rust::shape::{create_height_field_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_parallel_joint_def, default_shape_def, default_wheel_joint_def,
    BodyType,
};

pub(crate) fn build_driving() -> JointState {
    let mut world = new_world();
    let mut bodies = Vec::new();

    let rows = 33;
    let cols = 33;
    let scale = Vec3 {
        x: 2.0,
        y: 1.5,
        z: 2.0,
    };
    let hf = create_wave(rows, cols, scale, 0.02, 0.04, false);
    let hf_origin = Vec3 {
        x: -0.5 * scale.x * (cols - 1) as f32,
        y: 0.0,
        z: -0.5 * scale.z * (rows - 1) as f32,
    };

    let mut ground_def = default_body_def();
    ground_def.position = pos(hf_origin.x, hf_origin.y, hf_origin.z);
    let ground = create_body(&mut world, &ground_def);
    create_height_field_shape(&mut world, ground, &default_shape_def(), &hf);

    let mut body_def = default_body_def();
    body_def.position = pos(0.0, 2.5, 0.0);
    body_def.type_ = BodyType::Dynamic;
    let chassis = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 0.5;
    let box_hull = make_box_hull(2.0, 0.5, 1.0);
    create_hull_shape(&mut world, chassis, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_body(chassis.index1 - 1, 2.0, 0.5, 1.0));

    // Keep vehicle upright (C parallel joint to ground).
    let mut parallel = default_parallel_joint_def();
    parallel.base.body_id_a = ground;
    parallel.base.body_id_b = chassis;
    parallel.base.local_frame_a.q = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    parallel.base.local_frame_b.q = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    parallel.base.collide_connected = true;
    parallel.hertz = 0.5;
    parallel.damping_ratio = 1.0;
    create_parallel_joint(&mut world, &parallel);

    shape_def.density = 2.0;
    shape_def.base_material.friction = 3.0;

    body_def.type_ = BodyType::Dynamic;
    body_def.allow_fast_rotation = true;
    body_def.rotation = compute_quat_between_unit_vectors(VEC3_AXIS_Y, VEC3_AXIS_Z);

    let spin_speed = 30.0f32;
    let max_spin_torque = 5.0f32;
    let suspension_hertz = 4.0f32;
    let suspension_damping = 0.7f32;
    let lower_trans = -0.2f32;
    let upper_trans = 0.2f32;
    let steering_hertz = 10.0f32;
    let steering_damping = 0.7f32;
    let max_steering_torque = 5.0f32;
    let lower_steer = -45.0f32 * PI / 180.0;
    let upper_steer = 45.0f32 * PI / 180.0;

    let mut joint_def = default_wheel_joint_def();
    joint_def.base.body_id_a = chassis;
    joint_def.base.local_frame_a.q = compute_quat_between_unit_vectors(VEC3_AXIS_X, VEC3_AXIS_Y);
    joint_def.base.local_frame_b.q = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    joint_def.enable_suspension_limit = true;
    joint_def.lower_suspension_limit = lower_trans;
    joint_def.upper_suspension_limit = upper_trans;
    joint_def.enable_suspension_spring = true;
    joint_def.suspension_hertz = suspension_hertz;
    joint_def.suspension_damping_ratio = suspension_damping;
    joint_def.enable_spin_motor = true;
    joint_def.max_spin_torque = max_spin_torque;
    joint_def.enable_steering = true;
    joint_def.steering_hertz = steering_hertz;
    joint_def.steering_damping_ratio = steering_damping;
    joint_def.target_steering_angle = 0.0;
    joint_def.max_steering_torque = max_steering_torque;
    joint_def.enable_steering_limit = true;
    joint_def.lower_steering_limit = lower_steer;
    joint_def.upper_steering_limit = upper_steer;

    let wheel = sphere(0.4);

    body_def.position = pos(1.5, 2.0, 0.8);
    let fl_body = create_body(&mut world, &body_def);
    create_sphere_shape(&mut world, fl_body, &shape_def, &wheel);
    bodies.push(VisBody::sphere_body(fl_body.index1 - 1, 0.4));
    joint_def.base.body_id_b = fl_body;
    joint_def.base.local_frame_a.p = box3d_rust::math_functions::Vec3 {
        x: 1.5,
        y: -0.5,
        z: 0.8,
    };
    joint_def.enable_steering = true;
    joint_def.enable_spin_motor = false;
    let front_left = create_wheel_joint(&mut world, &joint_def);

    body_def.position = pos(1.5, 2.0, -0.8);
    let fr_body = create_body(&mut world, &body_def);
    create_sphere_shape(&mut world, fr_body, &shape_def, &wheel);
    bodies.push(VisBody::sphere_body(fr_body.index1 - 1, 0.4));
    joint_def.base.body_id_b = fr_body;
    joint_def.base.local_frame_a.p = box3d_rust::math_functions::Vec3 {
        x: 1.5,
        y: -0.5,
        z: -0.8,
    };
    let front_right = create_wheel_joint(&mut world, &joint_def);

    body_def.position = pos(-1.5, 2.0, 0.8);
    let rl_body = create_body(&mut world, &body_def);
    create_sphere_shape(&mut world, rl_body, &shape_def, &wheel);
    bodies.push(VisBody::sphere_body(rl_body.index1 - 1, 0.4));
    joint_def.base.body_id_b = rl_body;
    joint_def.base.local_frame_a.p = box3d_rust::math_functions::Vec3 {
        x: -1.5,
        y: -0.5,
        z: 0.8,
    };
    joint_def.enable_steering = false;
    joint_def.enable_spin_motor = true;
    let rear_left = create_wheel_joint(&mut world, &joint_def);

    body_def.position = pos(-1.5, 2.0, -0.8);
    let rr_body = create_body(&mut world, &body_def);
    create_sphere_shape(&mut world, rr_body, &shape_def, &wheel);
    bodies.push(VisBody::sphere_body(rr_body.index1 - 1, 0.4));
    joint_def.base.body_id_b = rr_body;
    joint_def.base.local_frame_a.p = box3d_rust::math_functions::Vec3 {
        x: -1.5,
        y: -0.5,
        z: -0.8,
    };
    let rear_right = create_wheel_joint(&mut world, &joint_def);

    let mut state = empty_state(world, bodies, JointScene::Driving);
    state.chassis = chassis;
    state.front_left = front_left;
    state.front_right = front_right;
    state.rear_left = rear_left;
    state.rear_right = rear_right;
    state.hf = Some(hf);
    state.hf_origin = hf_origin;
    state.spin_speed = spin_speed;
    state
}

pub(crate) fn apply_spin_torque(state: &mut JointState, torque: f32) {
    if state.rear_left.is_non_null() {
        wheel_joint_set_max_spin_torque(&mut state.world, state.rear_left, torque);
        wheel_joint_set_max_spin_torque(&mut state.world, state.rear_right, torque);
    }
}

pub(crate) fn pre_step(state: &mut JointState) {
    if !state.chassis.is_non_null() {
        return;
    }
    let throttle = state.throttle_x;
    let steer = state.throttle_y;
    if throttle.abs() > 0.01 || steer.abs() > 0.01 {
        body_set_awake(&mut state.world, state.chassis, true);
    }

    let max_steering = 0.25 * PI;
    wheel_joint_set_target_steering_angle(&mut state.world, state.front_left, max_steering * steer);
    wheel_joint_set_target_steering_angle(
        &mut state.world,
        state.front_right,
        max_steering * steer,
    );
    wheel_joint_set_spin_motor_speed(
        &mut state.world,
        state.rear_left,
        -state.spin_speed * throttle,
    );
    wheel_joint_set_spin_motor_speed(
        &mut state.world,
        state.rear_right,
        -state.spin_speed * throttle,
    );
}

pub(crate) fn terrain_wireframe(state: &JointState) -> Vec<f32> {
    let Some(ref hf) = state.hf else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let count = get_height_field_triangle_count(hf);
    for i in 0..count {
        let tri = get_height_field_triangle(hf, i);
        let verts = tri.vertices;
        for e in 0..3 {
            let a = verts[e];
            let b = verts[(e + 1) % 3];
            out.push(a.x + state.hf_origin.x);
            out.push(a.y + state.hf_origin.y);
            out.push(a.z + state.hf_origin.z);
            out.push(b.x + state.hf_origin.x);
            out.push(b.y + state.hf_origin.y);
            out.push(b.z + state.hf_origin.z);
        }
    }
    out
}
