//! Distance Joint, Filter, Motor Joint, Top Down Friction (sample_joint.cpp).

use super::{add_ground_box, empty_state, install, new_world, JointScene, JointState};
use crate::vis::{pos, vec3, VisBody};
use box3d_rust::body::{
    body_apply_linear_impulse_to_center, body_get_local_point, body_set_target_transform,
    create_body,
};
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::joint::{
    create_distance_joint, create_filter_joint, create_motor_joint, distance_joint_enable_limit,
    distance_joint_enable_spring, distance_joint_set_length, distance_joint_set_length_range,
    distance_joint_set_spring_damping_ratio, distance_joint_set_spring_force_range,
    distance_joint_set_spring_hertz, joint_get_constraint_force, joint_get_constraint_torque,
    joint_wake_bodies, motor_joint_set_max_spring_force, motor_joint_set_max_spring_torque,
};
use box3d_rust::math_functions::{
    length, make_quat_from_axis_angle, Transform, Vec3, WorldTransform, QUAT_IDENTITY, VEC3_AXIS_Z,
};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::default_explosion_def;
use box3d_rust::types::{
    default_body_def, default_distance_joint_def, default_filter_joint_def,
    default_motor_joint_def, default_shape_def, BodyType,
};
use box3d_rust::world::world_explode;
use wasm_bindgen::prelude::*;

// --- Distance Joint (C DistanceJoint) ---

/// Rebuild the whole distance chain, matching C `DistanceJoint::CreateScene`. The
/// full member state is passed so the rebuild (which spaces bodies by `length`)
/// is exact for any live-edited value, just like the C `Count` slider.
#[allow(clippy::too_many_arguments)]
#[wasm_bindgen]
pub fn joint_reset_distance(
    count: u32,
    hertz: f32,
    damping: f32,
    length_v: f32,
    tension: f32,
    compression: f32,
    min_length: f32,
    max_length: f32,
    enable_spring: bool,
    enable_limit: bool,
) -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();

    // AddGroundBox( 20 ) then a shapeless ground body used as the chain anchor.
    add_ground_box(&mut world, &mut bodies, 20.0);
    let ground = create_body(&mut world, &default_body_def());

    let radius = 0.25f32;
    let sph = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius,
    };
    let mut shape_def = default_shape_def();
    shape_def.density = 20.0;
    let y_offset = 20.0f32;

    let mut joint_def = default_distance_joint_def();
    joint_def.hertz = hertz;
    joint_def.damping_ratio = damping;
    joint_def.length = length_v;
    joint_def.lower_spring_force = -tension;
    joint_def.upper_spring_force = compression;
    joint_def.min_length = min_length;
    joint_def.max_length = max_length;
    joint_def.enable_spring = enable_spring;
    joint_def.enable_limit = enable_limit;

    let mut joints = Vec::new();
    let mut prev = ground;
    for i in 0..count {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.angular_damping = 1.0;
        body_def.position = pos(length_v * (i as f32 + 1.0), y_offset, 0.0);
        let body = create_body(&mut world, &body_def);
        create_sphere_shape(&mut world, body, &shape_def, &sph);
        bodies.push(VisBody::sphere_body(body.index1 - 1, radius));

        let pivot_a = pos(length_v * i as f32, y_offset, 0.0);
        let pivot_b = pos(length_v * (i as f32 + 1.0), y_offset, 0.0);
        joint_def.base.body_id_a = prev;
        joint_def.base.body_id_b = body;
        joint_def.base.local_frame_a.p = body_get_local_point(&world, prev, pivot_a);
        joint_def.base.local_frame_b.p = body_get_local_point(&world, body, pivot_b);
        joints.push(create_distance_joint(&mut world, &joint_def));
        prev = body;
    }

    let mut state = empty_state(world, bodies, JointScene::Distance);
    state.joints = joints;
    install(state)
}

/// Live distance-joint controls (C DistanceJoint::DrawControls) applied to every
/// joint. The `Count` slider instead calls [`joint_reset_distance`].
#[allow(clippy::too_many_arguments)]
#[wasm_bindgen]
pub fn joint_set_distance_params(
    length_v: f32,
    enable_spring: bool,
    tension: f32,
    compression: f32,
    hertz: f32,
    damping: f32,
    enable_limit: bool,
    min_length: f32,
    max_length: f32,
) {
    super::with_state(|state| {
        let joints = state.joints.clone();
        for jid in joints {
            distance_joint_set_length(&mut state.world, jid, length_v);
            distance_joint_enable_spring(&mut state.world, jid, enable_spring);
            distance_joint_set_spring_force_range(&mut state.world, jid, -tension, compression);
            distance_joint_set_spring_hertz(&mut state.world, jid, hertz);
            distance_joint_set_spring_damping_ratio(&mut state.world, jid, damping);
            distance_joint_enable_limit(&mut state.world, jid, enable_limit);
            distance_joint_set_length_range(&mut state.world, jid, min_length, max_length);
            joint_wake_bodies(&mut state.world, jid);
        }
    });
}

// --- Filter (C FilterJoint) — two boxes overlapping, collision disabled. ---

#[wasm_bindgen]
pub fn joint_reset_filter() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    add_ground_box(&mut world, &mut bodies, 20.0);

    let shape_def = default_shape_def();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(2.0, 4.0, 0.0);
    let body1 = create_body(&mut world, &body_def);
    create_hull_shape(&mut world, body1, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_body(body1.index1 - 1, 0.5, 0.5, 0.5));

    body_def.position = pos(-2.0, 4.0, 0.0);
    let body2 = create_body(&mut world, &body_def);
    create_hull_shape(&mut world, body2, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_body(body2.index1 - 1, 0.5, 0.5, 0.5));

    let mut joint_def = default_filter_joint_def();
    joint_def.base.body_id_a = body1;
    joint_def.base.body_id_b = body2;
    create_filter_joint(&mut world, &joint_def);

    install(empty_state(world, bodies, JointScene::Filter))
}

// --- Motor Joint (C MotorJoint) ---

#[wasm_bindgen]
pub fn joint_reset_motor() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    add_ground_box(&mut world, &mut bodies, 20.0);

    // Shapeless ground body at y = -1 (parent of the spring body's motor joint).
    let mut ground_def = default_body_def();
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut world, &ground_def);

    // Kinematic target (no shape — invisible, drives the motorized body).
    let mut target_def = default_body_def();
    target_def.type_ = BodyType::Kinematic;
    target_def.position = pos(0.0, 10.0, 0.0);
    let target = create_body(&mut world, &target_def);

    // Motorized body.
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 10.0, 0.0);
    let body = create_body(&mut world, &body_def);
    let motor_box = make_box_hull(1.0, 0.25, 0.25);
    let shape_def = default_shape_def();
    create_hull_shape(&mut world, body, &shape_def, &motor_box.base);
    bodies.push(VisBody::box_body(body.index1 - 1, 1.0, 0.25, 0.25));

    let max_force = 400000.0f32;
    let max_torque = 500000.0f32;
    let mut joint_def = default_motor_joint_def();
    joint_def.base.body_id_a = target;
    joint_def.base.body_id_b = body;
    joint_def.linear_hertz = 4.0;
    joint_def.linear_damping_ratio = 0.7;
    joint_def.angular_hertz = 4.0;
    joint_def.angular_damping_ratio = 0.7;
    joint_def.max_spring_force = max_force;
    joint_def.max_spring_torque = max_torque;
    let joint = create_motor_joint(&mut world, &joint_def);

    // Spring body.
    let mut spring_def = default_body_def();
    spring_def.type_ = BodyType::Dynamic;
    spring_def.position = pos(-2.0, 2.0, 0.0);
    let spring_body = create_body(&mut world, &spring_def);
    let spring_box = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, spring_body, &shape_def, &spring_box.base);
    bodies.push(VisBody::box_body(spring_body.index1 - 1, 0.5, 0.5, 0.5));

    let mut spring_joint = default_motor_joint_def();
    spring_joint.base.body_id_a = ground;
    spring_joint.base.body_id_b = spring_body;
    spring_joint.base.local_frame_a.p = vec3(-1.75, 3.25, 0.0);
    spring_joint.base.local_frame_b.p = vec3(0.25, 0.25, 0.0);
    spring_joint.base.collide_connected = true;
    spring_joint.linear_hertz = 7.5;
    spring_joint.linear_damping_ratio = 0.7;
    spring_joint.angular_hertz = 7.5;
    spring_joint.angular_damping_ratio = 0.7;
    spring_joint.max_spring_force = 200000.0;
    spring_joint.max_spring_torque = 10000.0;
    create_motor_joint(&mut world, &spring_joint);

    let mut state = empty_state(world, bodies, JointScene::Motor);
    state.control_joint = joint;
    state.motor_target = target;
    state.motor_body = body;
    install(state)
}

/// C MotorJoint::Step — animate the kinematic target on a sinusoidal path and
/// drive it with `SetTargetTransform`. Called from `joint_step` each frame.
pub(crate) fn motor_pre_step(state: &mut JointState, dt: f32) {
    if !state.motor_target.is_non_null() || dt <= 0.0 {
        return;
    }
    state.motor_time += state.motor_speed * dt;
    let t = state.motor_time;
    let linear = vec3(6.0 * (2.0 * t).sin(), 10.0 + 4.0 * (1.0 * t).sin(), 0.0);
    let angular = 2.0 * t;
    let transform = WorldTransform {
        p: pos(linear.x, linear.y, linear.z),
        q: make_quat_from_axis_angle(VEC3_AXIS_Z, angular),
    };
    body_set_target_transform(&mut state.world, state.motor_target, transform, dt, true);
}

/// C DrawControls: speed slider + max force/torque setters.
#[wasm_bindgen]
pub fn joint_motor_set_params(speed: f32, max_force: f32, max_torque: f32) {
    super::with_state(|state| {
        state.motor_speed = speed;
        let jid = state.control_joint;
        if jid.is_non_null() {
            motor_joint_set_max_spring_force(&mut state.world, jid, max_force);
            motor_joint_set_max_spring_torque(&mut state.world, jid, max_torque);
        }
    });
}

/// C "Apply Impulse" button.
#[wasm_bindgen]
pub fn joint_motor_apply_impulse() {
    super::with_state(|state| {
        if state.motor_body.is_non_null() {
            body_apply_linear_impulse_to_center(
                &mut state.world,
                state.motor_body,
                vec3(100000.0, 0.0, 0.0),
                true,
            );
        }
    });
}

/// C Step HUD: `[|constraintForce|, |constraintTorque|]`.
#[wasm_bindgen]
pub fn joint_motor_readout() -> Vec<f32> {
    super::with_state(|state| {
        let jid = state.control_joint;
        if !jid.is_non_null() {
            return vec![0.0, 0.0];
        }
        // `..._torque` needs `&mut World`; `..._force` only `&World`. Take the
        // mutable one first so the borrows never overlap.
        let torque = joint_get_constraint_torque(&mut state.world, jid);
        let force = joint_get_constraint_force(&state.world, jid);
        vec![length(force), length(torque)]
    })
}

// --- Top Down Friction (C TopDownFriction) ---

#[wasm_bindgen]
pub fn joint_reset_top_down_friction() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();

    // Arena frame: four boxes on one shapeless ground body (no AddGroundBox).
    let ground = create_body(&mut world, &default_body_def());
    let shape_def = default_shape_def();
    let ground_index = ground.index1 - 1;
    let arena: [(f32, f32, f32, f32, f32, f32); 4] = [
        (10.0, 0.5, 4.0, 0.0, 0.0, 0.0),
        (0.5, 10.0, 4.0, -10.0, 10.0, 0.0),
        (0.5, 10.0, 4.0, 10.0, 10.0, 0.0),
        (10.0, 0.5, 4.0, 0.0, 20.0, 0.0),
    ];
    for (hx, hy, hz, px, py, pz) in arena {
        let transform = Transform {
            p: vec3(px, py, pz),
            q: QUAT_IDENTITY,
        };
        let hull = make_transformed_box_hull(hx, hy, hz, transform);
        create_hull_shape(&mut world, ground, &shape_def, &hull.base);
        bodies.push(VisBody::box_local(ground_index, hx, hy, hz, transform));
    }

    // Motor joint template: pin every body to ground with a velocity limit → top-down friction.
    let mut joint_def = default_motor_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.collide_connected = true;
    joint_def.max_velocity_force = 1000.0;
    joint_def.max_velocity_torque = 1000.0;

    let capsule = Capsule {
        center1: vec3(-0.25, 0.0, 0.0),
        center2: vec3(0.25, 0.0, 0.0),
        radius: 0.25,
    };
    let sphere = Sphere {
        center: vec3(0.0, 0.0, 0.0),
        radius: 0.35,
    };
    let cube = make_box_hull(0.35, 0.35, 0.35);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.gravity_scale = 0.0;
    let mut body_shape_def = default_shape_def();
    body_shape_def.base_material.restitution = 0.8;

    let n = 10i32;
    let mut x = -5.0f32;
    let mut y = 15.0f32;
    for i in 0..n {
        for j in 0..n {
            body_def.position = pos(x, y, 0.0);
            let body = create_body(&mut world, &body_def);
            let idx = body.index1 - 1;
            let remainder = (n * i + j) % 4;
            if remainder == 0 {
                create_capsule_shape(&mut world, body, &body_shape_def, &capsule);
                bodies.push(VisBody::capsule_body(idx, &capsule));
            } else if remainder == 1 {
                create_sphere_shape(&mut world, body, &body_shape_def, &sphere);
                bodies.push(VisBody::sphere_body(idx, 0.35));
            } else {
                create_hull_shape(&mut world, body, &body_shape_def, &cube.base);
                bodies.push(VisBody::box_body(idx, 0.35, 0.35, 0.35));
            }
            joint_def.base.body_id_b = body;
            create_motor_joint(&mut world, &joint_def);
            x += 1.0;
        }
        x = -5.0;
        y -= 1.0;
    }

    install(empty_state(world, bodies, JointScene::TopDownFriction))
}

/// C "Explode" button — radial impulse from `(0,10,0)`.
#[wasm_bindgen]
pub fn joint_explode() {
    super::with_state(|state| {
        let mut def = default_explosion_def();
        def.position = pos(0.0, 10.0, 0.0);
        def.radius = 10.0;
        def.falloff = 5.0;
        def.impulse_per_area = 10000.0;
        world_explode(&mut state.world, &def);
    });
}
