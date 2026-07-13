//! RigidBodyCharacter sample (`sample_character.cpp:1313`) — the large s&box-style
//! dynamic character. Split into three modules along the C seams:
//!
//! - [`character`] — the `RigidbodyCharacter` velocity model + 4-phase trace step-up.
//! - [`scene`] — the level / obstacle-course builder (`build_rigid_body`).
//! - this module — the per-frame [`step`] glue the wasm `character_step` runs.

#![allow(clippy::unnecessary_cast)]

mod character;
mod scene;

pub(crate) use character::RigidbodyCharacter;
pub(crate) use scene::build_rigid_body;

use super::{mover_capsule, CharacterState, SceneKind, SceneState};
use box3d_rust::body::{body_get_linear_velocity, body_get_position};
use box3d_rust::math_functions::{get_length_and_normalize, Pos, Transform, Vec3, QUAT_IDENTITY};

pub(crate) fn step(state: &mut CharacterState, dt: f32, sub_steps: i32) {
    let (jump, want_sprint, throttle_x, throttle_y, forward, right) = {
        let i = &state.input;
        (
            i.jump,
            i.want_sprint,
            i.throttle_x,
            i.throttle_y,
            normalize_horizontal(i.forward),
            i.right,
        )
    };

    // Move the character out so pre/post-step get &mut world and &mut character.
    let SceneState::RigidBody(mut c) = std::mem::replace(&mut state.state, placeholder_scene())
    else {
        return;
    };

    c.debug_segs.clear();
    c.debug_pts.clear();

    if jump {
        c.jump(&mut state.world);
    }
    c.sprint = c.on_ground && want_sprint;

    c.pre_step(&mut state.world, dt, forward, right, throttle_x, throttle_y);
    state.world.step(dt, sub_steps);
    c.post_step(&mut state.world, dt);
    c.draw_debug(&state.world);

    // Merge the character's debug draw into the scene buffers.
    state.debug_segs.extend_from_slice(&c.debug_segs);
    state.debug_pts.extend_from_slice(&c.debug_pts);

    let pos = body_get_position(&state.world, c.body_id);
    let vel = body_get_linear_velocity(&state.world, c.body_id);
    state.status = vec![
        pos.x as f32,
        pos.y as f32,
        pos.z as f32,
        vel.x,
        vel.y,
        vel.z,
        if c.on_ground { 1.0 } else { 0.0 },
        if c.sprint { 1.0 } else { 0.0 },
    ];

    state.state = SceneState::RigidBody(c);
}

fn placeholder_scene() -> SceneState {
    // A never-used placeholder while the character is moved out for the step.
    SceneState::Drag(super::overlap::DragScene {
        kind: SceneKind::CapsulePlane,
        transform: Transform {
            p: Pos {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            q: QUAT_IDENTITY,
        },
        capsule: mover_capsule(),
        planes: Vec::new(),
        results: Vec::new(),
        capacity: 0,
    })
}

fn normalize_horizontal(v: Vec3) -> Vec3 {
    let mut len = 0.0;
    let n = get_length_and_normalize(
        &mut len,
        Vec3 {
            x: v.x,
            y: 0.0,
            z: v.z,
        },
    );
    if len < 1e-4 {
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        }
    } else {
        n
    }
}
