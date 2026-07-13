//! Door, Bridge, Motion Locks (sample_joint.cpp).

use super::{add_ground_box, empty_state, install, new_world, JointScene, JointState};
use crate::vis::{pos, vec3, VisBody};
use box3d_rust::body::{
    body_apply_linear_impulse, body_apply_linear_impulse_to_center, body_get_local_point,
    body_get_world_point, body_set_gravity_scale, create_body,
};
use box3d_rust::hull::make_box_hull;
use box3d_rust::id::{BodyId, JointId, NULL_JOINT_ID};
use box3d_rust::joint::{
    create_distance_joint, create_prismatic_joint, create_revolute_joint, create_spherical_joint,
    create_weld_joint, destroy_joint, joint_get_linear_separation, joint_is_valid,
    joint_set_constraint_tuning, revolute_joint_enable_limit,
};
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, mul_sv, normalize, Pos, Vec3, DEG_TO_RAD, VEC3_AXIS_Y,
    VEC3_AXIS_Z,
};
use box3d_rust::shape::{create_hull_shape, shape_get_body};
use box3d_rust::types::{
    default_body_def, default_distance_joint_def, default_prismatic_joint_def,
    default_query_filter, default_revolute_joint_def, default_shape_def,
    default_spherical_joint_def, default_weld_joint_def, BodyType, MotionLocks,
};
use box3d_rust::world::world_cast_ray_closest;
use wasm_bindgen::prelude::*;

// --- Door (C Door) ---

const DOOR_LOWER: f32 = DEG_TO_RAD * -90.0;
const DOOR_UPPER: f32 = DEG_TO_RAD * 90.0;

fn make_door_joint(
    world: &mut box3d_rust::world::World,
    ground: BodyId,
    door: BodyId,
    frame_a_y: f32,
    frame_b_y: f32,
    hertz: f32,
    damping: f32,
) -> JointId {
    let axis_quat = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    let mut jd = default_revolute_joint_def();
    jd.base.body_id_a = ground;
    jd.base.body_id_b = door;
    jd.base.local_frame_a.p = vec3(-0.75, frame_a_y, 0.0);
    jd.base.local_frame_a.q = axis_quat;
    jd.base.local_frame_b.p = vec3(-0.75, frame_b_y, 0.0);
    jd.base.local_frame_b.q = axis_quat;
    jd.base.constraint_hertz = hertz;
    jd.base.constraint_damping_ratio = damping;
    jd.enable_limit = true;
    jd.lower_angle = DOOR_LOWER;
    jd.upper_angle = DOOR_UPPER;
    jd.enable_spring = true;
    jd.hertz = 1.0;
    jd.damping_ratio = 0.5;
    jd.enable_motor = false;
    jd.max_motor_torque = 100.0;
    jd.motor_speed = 0.0;
    jd.base.draw_scale = 2.0;
    create_revolute_joint(world, &jd)
}

/// C Door::CreateJoints — (re)build the one or two hinges keeping the door body.
fn build_door_joints(state: &mut JointState) {
    if joint_is_valid(&state.world, state.door_joint1) {
        destroy_joint(&mut state.world, state.door_joint1, false);
        state.door_joint1 = NULL_JOINT_ID;
    }
    if joint_is_valid(&state.world, state.door_joint2) {
        destroy_joint(&mut state.world, state.door_joint2, false);
        state.door_joint2 = NULL_JOINT_ID;
    }
    let ground = state.door_ground;
    let door = state.door_id;
    let hertz = state.door_hertz;
    let damping = state.door_damping;
    state.door_joint1 = make_door_joint(&mut state.world, ground, door, 1.0, -1.5, hertz, damping);
    if state.door_two_joints {
        state.door_joint2 =
            make_door_joint(&mut state.world, ground, door, 4.0, 1.5, hertz, damping);
    }
}

#[wasm_bindgen]
pub fn joint_reset_door(magnitude: f32, two_joints: bool, hertz: f32, damping: f32) -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, &mut bodies, 20.0);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 1.5, 0.0);
    body_def.gravity_scale = 2.0;
    let door = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1000.0;
    let box_hull = make_box_hull(0.75, 1.5, 0.1);
    create_hull_shape(&mut world, door, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_body(door.index1 - 1, 0.75, 1.5, 0.1));

    let mut state = empty_state(world, bodies, JointScene::Door);
    state.door_ground = ground;
    state.door_id = door;
    state.door_magnitude = magnitude;
    state.door_two_joints = two_joints;
    state.door_hertz = hertz;
    state.door_damping = damping;
    build_door_joints(&mut state);
    install(state)
}

/// C Door::MouseDown — ctrl-click ray-pick applies a launch impulse.
pub(crate) fn door_ray_impulse(state: &mut JointState, origin: Pos, translation: Vec3) -> bool {
    let filter = default_query_filter();
    let result = world_cast_ray_closest(&state.world, origin, translation, &filter);
    if !result.hit {
        return false;
    }
    let body = shape_get_body(&state.world, result.shape_id);
    let impulse = mul_sv(state.door_magnitude, normalize(translation));
    body_apply_linear_impulse(&mut state.world, body, impulse, result.point, true);
    true
}

/// C "Impulse##Door" button — impulse at the door's `(0.75,0,0)` world point.
#[wasm_bindgen]
pub fn joint_door_impulse() {
    super::with_state(|state| {
        if !state.door_id.is_non_null() {
            return;
        }
        let p = body_get_world_point(&state.world, state.door_id, vec3(0.75, 0.0, 0.0));
        let impulse = vec3(0.0, 0.0, -state.door_magnitude);
        body_apply_linear_impulse(&mut state.world, state.door_id, impulse, p, true);
        state.door_error1 = 0.0;
        state.door_error2 = 0.0;
    });
}

#[wasm_bindgen]
pub fn joint_door_set_magnitude(magnitude: f32) {
    super::with_state(|state| state.door_magnitude = magnitude);
}

/// C "Limit##Door" checkbox — enable/disable limit on both hinges.
#[wasm_bindgen]
pub fn joint_door_set_limit(enable: bool) {
    super::with_state(|state| {
        state.door_enable_limit = enable;
        revolute_joint_enable_limit(&mut state.world, state.door_joint1, enable);
        if joint_is_valid(&state.world, state.door_joint2) {
            revolute_joint_enable_limit(&mut state.world, state.door_joint2, enable);
        }
    });
}

/// C "Two joints##Door" checkbox — rebuild the hinges.
#[wasm_bindgen]
pub fn joint_door_set_two_joints(two_joints: bool) {
    super::with_state(|state| {
        state.door_two_joints = two_joints;
        build_door_joints(state);
    });
}

/// C "Hertz##Door" / "Damping##Door" sliders — constraint tuning on both hinges.
#[wasm_bindgen]
pub fn joint_door_set_tuning(hertz: f32, damping: f32) {
    super::with_state(|state| {
        state.door_hertz = hertz;
        state.door_damping = damping;
        joint_set_constraint_tuning(&mut state.world, state.door_joint1, hertz, damping);
        if joint_is_valid(&state.world, state.door_joint2) {
            joint_set_constraint_tuning(&mut state.world, state.door_joint2, hertz, damping);
        }
    });
}

/// C Door::Step — track the running max linear separation of each hinge.
pub(crate) fn door_update_errors(state: &mut JointState) {
    let e1 = joint_get_linear_separation(&state.world, state.door_joint1);
    state.door_error1 = state.door_error1.max(e1);
    if joint_is_valid(&state.world, state.door_joint2) {
        let e2 = joint_get_linear_separation(&state.world, state.door_joint2);
        state.door_error2 = state.door_error2.max(e2);
    }
}

/// C Door::Step HUD/marker — `[error1, error2, hasTwoJoints, pointX, pointY, pointZ]`.
#[wasm_bindgen]
pub fn joint_door_readout() -> Vec<f32> {
    super::with_state(|state| {
        if !state.door_id.is_non_null() {
            return vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        }
        let p = body_get_world_point(&state.world, state.door_id, vec3(0.75, 0.0, 0.0));
        let has_two = if joint_is_valid(&state.world, state.door_joint2) {
            1.0
        } else {
            0.0
        };
        vec![
            state.door_error1,
            state.door_error2,
            has_two,
            p.x as f32,
            p.y as f32,
            p.z as f32,
        ]
    })
}

// --- Bridge (C Bridge) — a 150-plank suspension bridge. ---

const BRIDGE_COUNT: i32 = 150;

#[wasm_bindgen]
pub fn joint_reset_bridge() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    add_ground_box(&mut world, &mut bodies, 60.0);
    let ground = create_body(&mut world, &default_body_def());

    let a = 0.125f32;
    let box_hull = make_box_hull(a, 0.125, 0.5);
    let mut shape_def = default_shape_def();
    shape_def.density = 20.0;

    let mut joint_def = default_spherical_joint_def();
    joint_def.base.constraint_hertz = 1000.0;
    joint_def.enable_spring = true;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 1.0;

    let xbase = -160.0 * a;
    let mut planks = Vec::with_capacity(BRIDGE_COUNT as usize);
    let mut prev = ground;
    for i in 0..BRIDGE_COUNT {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(xbase + a * (1.0 + 2.0 * i as f32), 20.0, 0.0);
        body_def.linear_damping = 0.1;
        body_def.angular_damping = 0.1;
        let body = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, a, 0.125, 0.5));
        planks.push(body);

        for z in [-0.5f32, 0.5] {
            let pivot = pos(xbase + 2.0 * a * i as f32, 20.0, z);
            joint_def.base.body_id_a = prev;
            joint_def.base.body_id_b = body;
            joint_def.base.local_frame_a.p = body_get_local_point(&world, prev, pivot);
            joint_def.base.local_frame_b.p = body_get_local_point(&world, body, pivot);
            create_spherical_joint(&mut world, &joint_def);
        }
        prev = body;
    }

    for z in [-0.5f32, 0.5] {
        let pivot = pos(xbase + 2.0 * a * BRIDGE_COUNT as f32, 20.0, z);
        joint_def.base.body_id_a = prev;
        joint_def.base.body_id_b = ground;
        joint_def.base.local_frame_a.p = body_get_local_point(&world, prev, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(&world, ground, pivot);
        create_spherical_joint(&mut world, &joint_def);
    }

    let mut state = empty_state(world, bodies, JointScene::Bridge);
    state.aux_bodies = planks;
    install(state)
}

/// C Bridge::DrawControls — apply gravity scale to every plank.
#[wasm_bindgen]
pub fn joint_set_bridge_gravity(scale: f32) {
    super::with_state(|state| {
        let planks = state.aux_bodies.clone();
        for body in planks {
            body_set_gravity_scale(&mut state.world, body, scale);
        }
    });
}

// --- Motion Locks (C MotionLocks) ---

#[wasm_bindgen]
pub fn joint_reset_motion_locks() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    add_ground_box(&mut world, &mut bodies, 20.0);
    let ground = create_body(&mut world, &default_body_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.enable_sleep = false;

    let box_hull = make_box_hull(1.0, 1.0, 0.5);
    let force_threshold = 20000.0f32;
    let torque_threshold = 10000.0f32;
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;

    let mut aux = Vec::new();
    let mut position = pos(-12.5, 10.0, 0.0);

    // distance joint
    {
        body_def.position = position;
        let body = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, 1.0, 1.0, 0.5));
        aux.push(body);

        let length = 2.0f32;
        let pivot1 = pos(position.x, position.y + 1.0 + length, 0.0);
        let pivot2 = pos(position.x, position.y + 1.0, 0.0);
        let mut jd = default_distance_joint_def();
        jd.base.body_id_a = ground;
        jd.base.body_id_b = body;
        jd.base.local_frame_a.p = body_get_local_point(&world, ground, pivot1);
        jd.base.local_frame_b.p = body_get_local_point(&world, body, pivot2);
        jd.length = length;
        jd.base.force_threshold = force_threshold;
        jd.base.torque_threshold = torque_threshold;
        jd.base.collide_connected = true;
        create_distance_joint(&mut world, &jd);
    }
    position.x += 5.0;

    // prismatic joint
    {
        body_def.position = position;
        let body = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, 1.0, 1.0, 0.5));
        aux.push(body);

        let pivot = pos(position.x - 1.0, position.y, 0.0);
        let mut jd = default_prismatic_joint_def();
        jd.base.body_id_a = ground;
        jd.base.body_id_b = body;
        jd.base.local_frame_a.p = body_get_local_point(&world, ground, pivot);
        jd.base.local_frame_b.p = body_get_local_point(&world, body, pivot);
        jd.base.force_threshold = force_threshold;
        jd.base.torque_threshold = torque_threshold;
        jd.base.collide_connected = true;
        create_prismatic_joint(&mut world, &jd);
    }
    position.x += 5.0;

    // revolute joint
    {
        body_def.position = position;
        let body = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, 1.0, 1.0, 0.5));
        aux.push(body);

        let pivot = pos(position.x - 1.0, position.y, 0.0);
        let mut jd = default_revolute_joint_def();
        jd.base.body_id_a = ground;
        jd.base.body_id_b = body;
        jd.base.local_frame_a.p = body_get_local_point(&world, ground, pivot);
        jd.base.local_frame_b.p = body_get_local_point(&world, body, pivot);
        jd.base.force_threshold = force_threshold;
        jd.base.torque_threshold = torque_threshold;
        jd.base.collide_connected = true;
        create_revolute_joint(&mut world, &jd);
    }
    position.x += 5.0;

    // weld joint
    {
        body_def.position = position;
        let body = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, 1.0, 1.0, 0.5));
        aux.push(body);

        let pivot = pos(position.x - 1.0, position.y, 0.0);
        let mut jd = default_weld_joint_def();
        jd.base.body_id_a = ground;
        jd.base.body_id_b = body;
        jd.base.local_frame_a.p = body_get_local_point(&world, ground, pivot);
        jd.base.local_frame_b.p = body_get_local_point(&world, body, pivot);
        jd.angular_hertz = 2.0;
        jd.angular_damping_ratio = 0.5;
        jd.base.force_threshold = force_threshold;
        jd.base.torque_threshold = torque_threshold;
        jd.base.collide_connected = true;
        create_weld_joint(&mut world, &jd);
    }

    let mut state = empty_state(world, bodies, JointScene::MotionLocks);
    state.aux_bodies = aux;
    install(state)
}

/// C MotionLocks::DrawControls — apply the six lock checkboxes to every body.
#[wasm_bindgen]
pub fn joint_set_motion_locks(
    linear_x: bool,
    linear_y: bool,
    linear_z: bool,
    angular_x: bool,
    angular_y: bool,
    angular_z: bool,
) {
    super::with_state(|state| {
        let locks = MotionLocks {
            linear_x,
            linear_y,
            linear_z,
            angular_x,
            angular_y,
            angular_z,
        };
        super::apply_motion_locks(state, locks);
    });
}

/// C MotionLocks::DrawControls — `IsKeyDown( KEY_L )` impulse on the first body.
#[wasm_bindgen]
pub fn joint_motion_lock_impulse() {
    super::with_state(|state| {
        if let Some(&body) = state.aux_bodies.first() {
            body_apply_linear_impulse_to_center(
                &mut state.world,
                body,
                vec3(100.0, 0.0, 0.0),
                true,
            );
        }
    });
}
