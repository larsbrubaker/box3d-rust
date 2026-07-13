//! Prismatic, Spherical, Parallel Spring, Weld (sample_joint.cpp).
//!
//! Each is a single dynamic 0.5×1.5×0.25 box hung from a shapeless ground anchor
//! by one joint, exercised through that joint's limit / motor / spring controls.

use super::{add_ground_box, empty_state, install, new_world, JointScene};
use crate::vis::{pos, vec3, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::joint::{
    create_parallel_joint, create_prismatic_joint, create_spherical_joint, create_weld_joint,
    joint_wake_bodies, parallel_joint_set_spring_damping_ratio, parallel_joint_set_spring_hertz,
    prismatic_joint_enable_limit, prismatic_joint_enable_motor, prismatic_joint_enable_spring,
    prismatic_joint_set_limits, prismatic_joint_set_max_motor_force,
    prismatic_joint_set_motor_speed, prismatic_joint_set_spring_damping_ratio,
    prismatic_joint_set_spring_hertz, prismatic_joint_set_target_translation,
    spherical_joint_enable_cone_limit, spherical_joint_enable_motor, spherical_joint_enable_spring,
    spherical_joint_enable_twist_limit, spherical_joint_set_cone_limit,
    spherical_joint_set_max_motor_torque, spherical_joint_set_motor_velocity,
    spherical_joint_set_spring_damping_ratio, spherical_joint_set_spring_hertz,
    spherical_joint_set_target_rotation, spherical_joint_set_twist_limits,
    weld_joint_set_angular_damping_ratio, weld_joint_set_angular_hertz,
    weld_joint_set_linear_damping_ratio, weld_joint_set_linear_hertz,
};
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, inv_mul_quat, make_quat_from_axis_angle, mul_quat,
    Transform, DEG_TO_RAD, PI, QUAT_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z,
};
use box3d_rust::shape::create_hull_shape;
use box3d_rust::types::{
    default_body_def, default_parallel_joint_def, default_prismatic_joint_def, default_shape_def,
    default_spherical_joint_def, default_weld_joint_def, BodyType,
};
use wasm_bindgen::prelude::*;

/// Shared: AddGroundBox(20) + a shapeless ground anchor at (0,-1,0).
fn ground_and_anchor(
    world: &mut box3d_rust::world::World,
    bodies: &mut Vec<VisBody>,
) -> box3d_rust::id::BodyId {
    add_ground_box(world, bodies, 20.0);
    let mut def = default_body_def();
    def.position = pos(0.0, -1.0, 0.0);
    create_body(world, &def)
}

// --- Prismatic (C PrismaticJoint) ---

#[wasm_bindgen]
pub fn joint_reset_prismatic() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = ground_and_anchor(&mut world, &mut bodies);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 4.0, 0.0);
    body_def.gravity_scale = 0.0;
    let body = create_body(&mut world, &body_def);
    let box_hull = make_box_hull(0.5, 1.5, 0.25);
    create_hull_shape(&mut world, body, &default_shape_def(), &box_hull.base);
    bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 1.5, 0.25));

    let mut joint_def = default_prismatic_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = body;
    joint_def.base.local_frame_a.p = vec3(0.0, 6.5, 0.0);
    joint_def.base.local_frame_b.p = vec3(0.0, 1.5, 0.0);
    joint_def.base.constraint_hertz = 120.0;
    joint_def.enable_limit = false;
    joint_def.lower_translation = -1.0;
    joint_def.upper_translation = 1.0;
    joint_def.enable_spring = true;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 0.7;
    joint_def.target_translation = 0.0;
    joint_def.enable_motor = false;
    joint_def.max_motor_force = 20.0;
    joint_def.motor_speed = 0.0;
    let joint = create_prismatic_joint(&mut world, &joint_def);

    let mut state = empty_state(world, bodies, JointScene::Prismatic);
    state.control_joint = joint;
    install(state)
}

/// C PrismaticJoint::DrawControls. `flags`: bit0=limit, bit1=motor, bit2=spring.
#[allow(clippy::too_many_arguments)]
#[wasm_bindgen]
pub fn joint_set_prismatic_params(
    flags: u32,
    lower: f32,
    upper: f32,
    max_force: f32,
    speed: f32,
    hertz: f32,
    damping: f32,
    target: f32,
) {
    super::with_state(|state| {
        let jid = state.control_joint;
        if !jid.is_non_null() {
            return;
        }
        prismatic_joint_enable_limit(&mut state.world, jid, flags & 1 != 0);
        prismatic_joint_set_limits(&mut state.world, jid, lower, upper);
        prismatic_joint_enable_motor(&mut state.world, jid, flags & 2 != 0);
        prismatic_joint_set_max_motor_force(&mut state.world, jid, max_force);
        prismatic_joint_set_motor_speed(&mut state.world, jid, speed);
        prismatic_joint_enable_spring(&mut state.world, jid, flags & 4 != 0);
        prismatic_joint_set_spring_hertz(&mut state.world, jid, hertz);
        prismatic_joint_set_spring_damping_ratio(&mut state.world, jid, damping);
        prismatic_joint_set_target_translation(&mut state.world, jid, target);
        joint_wake_bodies(&mut state.world, jid);
    });
}

// --- Spherical (C SphericalJoint) ---

#[wasm_bindgen]
pub fn joint_reset_spherical() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = ground_and_anchor(&mut world, &mut bodies);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 4.0, 0.0);
    body_def.gravity_scale = 0.0;
    let body = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 100.0;
    let box_hull = make_box_hull(0.5, 1.5, 0.25);
    create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 1.5, 0.25));

    let mut joint_def = default_spherical_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = body;
    joint_def.base.draw_scale = 2.0;
    joint_def.base.local_frame_a.p = vec3(0.0, 6.5, 0.0);
    joint_def.base.local_frame_b.p = vec3(0.0, 1.5, 0.0);
    joint_def.enable_cone_limit = false;
    joint_def.cone_angle = DEG_TO_RAD * 30.0;
    joint_def.enable_twist_limit = false;
    joint_def.lower_twist_angle = DEG_TO_RAD * -35.0;
    joint_def.upper_twist_angle = DEG_TO_RAD * 35.0;
    joint_def.enable_spring = true;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 0.7;
    joint_def.enable_motor = false;
    joint_def.max_motor_torque = 20.0;
    joint_def.motor_velocity = vec3(0.0, 0.0, 0.0);
    let joint = create_spherical_joint(&mut world, &joint_def);

    let mut state = empty_state(world, bodies, JointScene::Spherical);
    state.control_joint = joint;
    install(state)
}

/// C SphericalJoint::DrawControls. `flags`: bit0=cone, bit1=twist, bit2=motor, bit3=spring.
#[allow(clippy::too_many_arguments)]
#[wasm_bindgen]
pub fn joint_set_spherical_params(
    flags: u32,
    cone_deg: f32,
    lower_twist_deg: f32,
    upper_twist_deg: f32,
    max_torque: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    hertz: f32,
    damping: f32,
    rot_x: f32,
    rot_y: f32,
    rot_z: f32,
) {
    super::with_state(|state| {
        let jid = state.control_joint;
        if !jid.is_non_null() {
            return;
        }
        spherical_joint_enable_cone_limit(&mut state.world, jid, flags & 1 != 0);
        spherical_joint_set_cone_limit(&mut state.world, jid, DEG_TO_RAD * cone_deg);
        spherical_joint_enable_twist_limit(&mut state.world, jid, flags & 2 != 0);
        spherical_joint_set_twist_limits(
            &mut state.world,
            jid,
            DEG_TO_RAD * lower_twist_deg,
            DEG_TO_RAD * upper_twist_deg,
        );
        spherical_joint_enable_motor(&mut state.world, jid, flags & 4 != 0);
        spherical_joint_set_max_motor_torque(&mut state.world, jid, max_torque);
        spherical_joint_set_motor_velocity(&mut state.world, jid, vec3(vel_x, vel_y, vel_z));
        spherical_joint_enable_spring(&mut state.world, jid, flags & 8 != 0);
        spherical_joint_set_spring_hertz(&mut state.world, jid, hertz);
        spherical_joint_set_spring_damping_ratio(&mut state.world, jid, damping);
        // C: q = qz * (qy * qx), matching the SliderFloat3 "Rotation" handler.
        let qx = make_quat_from_axis_angle(VEC3_AXIS_X, DEG_TO_RAD * rot_x);
        let qy = make_quat_from_axis_angle(VEC3_AXIS_Y, DEG_TO_RAD * rot_y);
        let qz = make_quat_from_axis_angle(VEC3_AXIS_Z, DEG_TO_RAD * rot_z);
        let q = mul_quat(qz, mul_quat(qy, qx));
        spherical_joint_set_target_rotation(&mut state.world, jid, q);
        joint_wake_bodies(&mut state.world, jid);
    });
}

// --- Parallel Spring (C ParallelJoint) ---

#[wasm_bindgen]
pub fn joint_reset_parallel() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    add_ground_box(&mut world, &mut bodies, 20.0);
    let mut ground_def = default_body_def();
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut world, &ground_def);
    let ground_index = ground.index1 - 1;

    // Four enclosing walls (on the shapeless anchor body at (0,-1,0)).
    let walls: [(f32, f32, f32, f32, f32, f32); 4] = [
        (20.0, 5.0, 0.1, 0.0, 5.0, -20.0),
        (20.0, 5.0, 0.1, 0.0, 5.0, 20.0),
        (0.1, 5.0, 20.0, -20.0, 5.0, 0.0),
        (0.1, 5.0, 20.0, 20.0, 5.0, 0.0),
    ];
    let shape_def = default_shape_def();
    for (hx, hy, hz, px, py, pz) in walls {
        let transform = Transform {
            p: vec3(px, py, pz),
            q: QUAT_IDENTITY,
        };
        let hull = make_transformed_box_hull(hx, hy, hz, transform);
        create_hull_shape(&mut world, ground, &shape_def, &hull.base);
        bodies.push(VisBody::box_local(ground_index, hx, hy, hz, transform));
    }

    let rotation = make_quat_from_axis_angle(VEC3_AXIS_X, 0.25 * PI);
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 4.0, 0.0);
    body_def.rotation = rotation;
    let body = create_body(&mut world, &body_def);
    let box_hull = make_box_hull(0.5, 1.5, 0.25);
    create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 1.5, 0.25));

    let mut joint_def = default_parallel_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = body;
    joint_def.base.local_frame_a.q = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    joint_def.base.local_frame_b.q = inv_mul_quat(rotation, joint_def.base.local_frame_a.q);
    joint_def.base.draw_scale = 2.0;
    joint_def.base.collide_connected = true;
    joint_def.hertz = 10.0;
    joint_def.damping_ratio = 0.7;
    let joint = create_parallel_joint(&mut world, &joint_def);

    let mut state = empty_state(world, bodies, JointScene::Parallel);
    state.control_joint = joint;
    install(state)
}

/// C ParallelJoint::DrawControls — spring hertz + damping.
#[wasm_bindgen]
pub fn joint_set_parallel_params(hertz: f32, damping: f32) {
    super::with_state(|state| {
        let jid = state.control_joint;
        if !jid.is_non_null() {
            return;
        }
        parallel_joint_set_spring_hertz(&mut state.world, jid, hertz);
        parallel_joint_set_spring_damping_ratio(&mut state.world, jid, damping);
        joint_wake_bodies(&mut state.world, jid);
    });
}

// --- Weld (C WeldJoint) ---

#[wasm_bindgen]
pub fn joint_reset_weld() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = ground_and_anchor(&mut world, &mut bodies);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 4.0, 0.0);
    body_def.gravity_scale = 0.0;
    let body = create_body(&mut world, &body_def);
    let box_hull = make_box_hull(0.5, 1.5, 0.25);
    create_hull_shape(&mut world, body, &default_shape_def(), &box_hull.base);
    bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 1.5, 0.25));

    let mut joint_def = default_weld_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = body;
    joint_def.base.local_frame_a.p = vec3(0.0, 6.5, 0.0);
    joint_def.base.local_frame_b.p = vec3(0.0, 1.5, 0.0);
    joint_def.base.constraint_hertz = 240.0;
    joint_def.linear_hertz = 0.0;
    joint_def.linear_damping_ratio = 0.0;
    joint_def.angular_hertz = 2.0;
    joint_def.angular_damping_ratio = 0.7;
    joint_def.base.draw_scale = 2.0;
    let joint = create_weld_joint(&mut world, &joint_def);

    let mut state = empty_state(world, bodies, JointScene::Weld);
    state.control_joint = joint;
    install(state)
}

/// C WeldJoint::DrawControls — linear/angular hertz + damping.
#[wasm_bindgen]
pub fn joint_set_weld_params(lin_hertz: f32, lin_damp: f32, ang_hertz: f32, ang_damp: f32) {
    super::with_state(|state| {
        let jid = state.control_joint;
        if !jid.is_non_null() {
            return;
        }
        weld_joint_set_linear_hertz(&mut state.world, jid, lin_hertz);
        weld_joint_set_linear_damping_ratio(&mut state.world, jid, lin_damp);
        weld_joint_set_angular_hertz(&mut state.world, jid, ang_hertz);
        weld_joint_set_angular_damping_ratio(&mut state.world, jid, ang_damp);
        joint_wake_bodies(&mut state.world, jid);
    });
}
