//! Joint demos — Ball and Chain + Revolute motor hinge (sample_joint.cpp).

use crate::vis::{capsule_x, pos, push_poses, sphere, vec3, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::hull::make_box_hull;
use box3d_rust::id::JointId;
use box3d_rust::joint::{
    create_revolute_joint, create_spherical_joint, revolute_joint_enable_motor,
    revolute_joint_set_max_motor_torque, revolute_joint_set_motor_speed,
};
use box3d_rust::math_functions::{Transform, Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_revolute_joint_def, default_shape_def, default_spherical_joint_def,
    default_world_def, BodyType,
};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<JointState>> = const { RefCell::new(None) };
}

struct JointState {
    world: World,
    bodies: Vec<VisBody>,
    hinge_joint: Option<JointId>,
    motor_enabled: bool,
    motor_speed: f32,
    motor_torque: f32,
}

fn with_state<R>(f: impl FnOnce(&mut JointState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("joint demo not initialized — call joint_reset_* first"))
    })
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

fn xf_at(px: f32, py: f32, pz: f32) -> Transform {
    Transform {
        p: vec3(px, py, pz),
        q: QUAT_IDENTITY,
    }
}

/// Ball-and-chain: spherical-linked capsules with a heavy sphere tip.
#[wasm_bindgen]
pub fn joint_reset_chain(link_count: u32) -> u32 {
    let n = link_count.clamp(4, 24);
    STATE.with(|cell| {
        let mut world = new_world();
        let mut bodies = Vec::new();

        let mut body_def = default_body_def();
        let ground = create_body(&mut world, &body_def);

        let link_radius = 0.125f32;
        let link_extent = 0.5f32;
        let capsule = capsule_x(link_extent, link_radius);
        let shape_def = default_shape_def();

        body_def.type_ = BodyType::Dynamic;
        let mut parent = ground;
        let mut joint_def = default_spherical_joint_def();
        joint_def.base.local_frame_a = TRANSFORM_IDENTITY;
        joint_def.base.local_frame_b = xf_at(-link_extent, 0.0, 0.0);
        joint_def.enable_motor = true;
        joint_def.max_motor_torque = 10.0;

        for i in 0..n {
            body_def.position = pos((1.0 + 2.0 * i as f32) * link_extent, 0.0, 0.0);
            let child = create_body(&mut world, &body_def);
            create_capsule_shape(&mut world, child, &shape_def, &capsule);
            bodies.push(VisBody::capsule_body(child.index1 - 1, &capsule));

            joint_def.base.body_id_a = parent;
            joint_def.base.body_id_b = child;
            create_spherical_joint(&mut world, &joint_def);

            joint_def.base.local_frame_a = xf_at(link_extent, 0.0, 0.0);
            parent = child;
        }

        let sphere_radius = 1.5f32;
        body_def.position = pos(
            (1.0 + 2.0 * n as f32) * link_extent + sphere_radius - link_extent,
            0.0,
            0.0,
        );
        let tip = create_body(&mut world, &body_def);
        let sph = sphere(sphere_radius);
        create_sphere_shape(&mut world, tip, &shape_def, &sph);
        bodies.push(VisBody::sphere_body(tip.index1 - 1, sphere_radius));

        joint_def.base.body_id_a = parent;
        joint_def.base.body_id_b = tip;
        joint_def.base.local_frame_b = xf_at(-sphere_radius, 0.0, 0.0);
        create_spherical_joint(&mut world, &joint_def);

        let count = bodies.len() as u32;
        *cell.borrow_mut() = Some(JointState {
            world,
            bodies,
            hinge_joint: None,
            motor_enabled: false,
            motor_speed: 0.0,
            motor_torque: 5000.0,
        });
        count
    })
}

/// Revolute hinge with motor toggle (sample Revolute Joint).
#[wasm_bindgen]
pub fn joint_reset_hinge() -> u32 {
    STATE.with(|cell| {
        let mut world = new_world();
        let mut bodies = Vec::new();

        let mut ground_def = default_body_def();
        ground_def.position = pos(0.0, -1.0, 0.0);
        let ground = create_body(&mut world, &ground_def);
        let shape_def = default_shape_def();
        let ground_hull = make_box_hull(20.0, 1.0, 20.0);
        create_hull_shape(&mut world, ground, &shape_def, &ground_hull.base);
        bodies.push(VisBody::box_body(ground.index1 - 1, 20.0, 1.0, 20.0));

        // Anchor body (static reference for joint local frame A)
        let mut anchor_def = default_body_def();
        anchor_def.position = pos(0.0, -1.0, 0.0);
        let anchor = create_body(&mut world, &anchor_def);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(0.0, 4.0, 0.0);
        let door = create_body(&mut world, &body_def);
        let door_hull = make_box_hull(0.5, 1.5, 0.25);
        create_hull_shape(&mut world, door, &shape_def, &door_hull.base);
        bodies.push(VisBody::box_body(door.index1 - 1, 0.5, 1.5, 0.25));

        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = anchor;
        joint_def.base.body_id_b = door;
        // Anchor at y=-1; local +6.5 → world hinge at y=5.5 (matches sample Revolute Joint).
        joint_def.base.local_frame_a = xf_at(0.0, 6.5, 0.0);
        joint_def.base.local_frame_b = xf_at(0.0, 1.5, 0.0);
        joint_def.enable_motor = false;
        joint_def.max_motor_torque = 5000.0;
        joint_def.motor_speed = 0.0;
        let joint_id = create_revolute_joint(&mut world, &joint_def);

        let count = bodies.len() as u32;
        *cell.borrow_mut() = Some(JointState {
            world,
            bodies,
            hinge_joint: Some(joint_id),
            motor_enabled: false,
            motor_speed: 2.0,
            motor_torque: 5000.0,
        });
        count
    })
}

#[wasm_bindgen]
pub fn joint_set_motor(enabled: bool, speed: f32, torque: f32) {
    with_state(|state| {
        state.motor_enabled = enabled;
        state.motor_speed = speed;
        state.motor_torque = torque;
        if let Some(jid) = state.hinge_joint {
            revolute_joint_enable_motor(&mut state.world, jid, enabled);
            revolute_joint_set_motor_speed(&mut state.world, jid, speed);
            revolute_joint_set_max_motor_torque(&mut state.world, jid, torque);
        }
    });
}

#[wasm_bindgen]
pub fn joint_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.world.step(dt, sub_steps);
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn joint_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn joint_body_count() -> u32 {
    with_state(|state| state.bodies.len() as u32)
}
