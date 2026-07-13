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

use super::{mover_capsule, with_state, CharacterState, SceneKind, SceneState};
use box3d_rust::body::{body_get_linear_velocity, body_get_position};
use box3d_rust::math_functions::{
    get_length_and_normalize, sub_pos, Pos, Transform, Vec3, QUAT_IDENTITY,
};
use box3d_rust::types::default_query_filter;
use box3d_rust::world::world_cast_ray_closest;
use wasm_bindgen::prelude::*;

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

/// Third-person camera-boom raycast (C `RigidBodyCharacter::Step`, sample_character.cpp
/// :1579-1602). Casts a world ray from the character (`from`) toward the desired eye
/// (`to`) with `b3DefaultQueryFilter`, so the browser page can clamp the boom length on
/// a hit and keep the eye from clipping through geometry — the same `b3World_CastRayClosest`
/// the C sample runs each frame. Returns the hit fraction along `to - from` in `[0, 1]`,
/// or `1.0` when the ray reaches the eye unobstructed (no hit / degenerate translation).
/// The JS side owns the C margins (0.15 m camera radius, 0.1 m floor) and the
/// `radius = min(saved, clamped)` restore, exactly as `m_camera` does.
#[wasm_bindgen]
pub fn character_camera_boom(
    from_x: f32,
    from_y: f32,
    from_z: f32,
    to_x: f32,
    to_y: f32,
    to_z: f32,
) -> f32 {
    with_state(|state| {
        let from = Pos {
            x: from_x as _,
            y: from_y as _,
            z: from_z as _,
        };
        let to = Pos {
            x: to_x as _,
            y: to_y as _,
            z: to_z as _,
        };
        let translation = sub_pos(to, from);
        let filter = default_query_filter();
        let result = world_cast_ray_closest(&state.world, from, translation, &filter);
        if result.hit {
            result.fraction
        } else {
            1.0
        }
    })
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
