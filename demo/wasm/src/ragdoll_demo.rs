//! Ragdoll demo — CreateHuman on a ground box (sample Ragdoll / Box).

use crate::vis::{pos, push_poses, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::core::NULL_INDEX;
use box3d_rust::geometry::Capsule;
use box3d_rust::hull::make_box_hull;
use box3d_rust::human::{
    create_human, destroy_human, human_set_joint_damping_ratio, human_set_joint_friction_torque,
    human_set_joint_spring_hertz, Human, BONE_COUNT,
};
use box3d_rust::math_functions::Vec3;
use box3d_rust::shape::{create_hull_shape, ShapeGeometry};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<RagdollState>> = const { RefCell::new(None) };
}

struct RagdollState {
    world: World,
    humans: Vec<Human>,
    bodies: Vec<VisBody>,
    friction: f32,
    hertz: f32,
    damping: f32,
}

fn with_state<R>(f: impl FnOnce(&mut RagdollState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("ragdoll not initialized — call ragdoll_reset first"))
    })
}

fn capsule_from_body(world: &World, body_index: i32) -> Option<Capsule> {
    let mut sid = world.bodies[body_index as usize].head_shape_id;
    while sid != NULL_INDEX {
        let shape = &world.shapes[sid as usize];
        if let ShapeGeometry::Capsule(c) = &shape.geometry {
            return Some(*c);
        }
        sid = shape.next_shape_id;
    }
    None
}

fn rebuild_vis(state: &mut RagdollState) {
    state.bodies.clear();
    // Ground is body index 0 (first created).
    state.bodies.push(VisBody::box_body(0, 20.0, 1.0, 20.0));
    for human in &state.humans {
        for i in 0..BONE_COUNT {
            let body_id = human.bones[i].body_id;
            if body_id.is_null() {
                continue;
            }
            let body_index = body_id.index1 - 1;
            if let Some(cap) = capsule_from_body(&state.world, body_index) {
                state.bodies.push(VisBody::capsule_body(body_index, &cap));
            }
        }
    }
}

fn spawn_human(state: &mut RagdollState) {
    for human in &mut state.humans {
        if human.is_spawned {
            destroy_human(human, &mut state.world);
        }
    }
    state.humans.clear();

    // C RagdollOnBox::Spawn: exactly one human at {0,2,0}, groupIndex 1,
    // userData null, colorize false (sample_ragdoll.cpp:35).
    let mut human = Human::default();
    create_human(
        &mut human,
        &mut state.world,
        pos(0.0, 2.0, 0.0),
        state.friction,
        state.hertz,
        state.damping,
        1,
        0,
        false,
    );
    state.humans.push(human);
    rebuild_vis(state);
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

/// Reset ragdoll scene (C RagdollOnBox: one human on a ground box).
#[wasm_bindgen]
pub fn ragdoll_reset() -> u32 {
    STATE.with(|cell| {
        let mut world = new_world();
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        ground_def.position = pos(0.0, -1.0, 0.0);
        let ground = create_body(&mut world, &ground_def);
        let shape_def = default_shape_def();
        let hull = make_box_hull(20.0, 1.0, 20.0);
        create_hull_shape(&mut world, ground, &shape_def, &hull.base);

        let mut state = RagdollState {
            world,
            humans: Vec::new(),
            bodies: Vec::new(),
            friction: 5.0,
            hertz: 1.0,
            damping: 0.7,
        };
        spawn_human(&mut state);
        let n = state.bodies.len() as u32;
        *cell.borrow_mut() = Some(state);
        n
    })
}

#[wasm_bindgen]
pub fn ragdoll_set_joint_params(friction: f32, hertz: f32, damping: f32) {
    with_state(|state| {
        state.friction = friction;
        state.hertz = hertz;
        state.damping = damping;
        for human in &mut state.humans {
            if !human.is_spawned {
                continue;
            }
            human_set_joint_friction_torque(human, &mut state.world, friction);
            human_set_joint_spring_hertz(human, &mut state.world, hertz);
            human_set_joint_damping_ratio(human, &mut state.world, damping);
        }
    });
}

#[wasm_bindgen]
pub fn ragdoll_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.world.step(dt, sub_steps);
        state.bodies.len() as u32
    })
}

/// Pose buffer: 15 floats per body (see `vis` module).
#[wasm_bindgen]
pub fn ragdoll_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn ragdoll_body_count() -> u32 {
    with_state(|state| state.bodies.len() as u32)
}
