//! Runtime control + mouse-interaction wasm exports for the Bodies demos, a
//! submodule of `bodies_demo` split off to keep each file under the 800-line
//! module gate. These call into the shared `BodiesState` (via `super::with_state`)
//! and mirror each C sample's `DrawControls` / `MouseDown`-`MouseMove`-`MouseUp`
//! handlers; the generic grab/spawn/delete/counters/debug exports come from the
//! shared [`crate::demo_shell!`] scaffold.

use super::{v3, with_state, SceneKind};
use crate::shell::ZERO_POS;
use crate::vis::VisBody;
use box3d_rust::body::{
    body_disable, body_enable, body_set_angular_velocity, body_set_awake, body_set_linear_velocity,
    body_set_transform, body_set_type,
};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, normalize, offset_pos, sub_pos, Pos, PI, VEC3_AXIS_Z, VEC3_ZERO,
};
use box3d_rust::types::{BodyType, ExplosionDef};
use box3d_rust::world::world_explode;
use wasm_bindgen::prelude::*;

// --- Body Type controls --------------------------------------------------------

/// Switch the Body Type sample's mutable bodies to `t` (0 static, 1 kinematic,
/// 2 dynamic). Mirrors `BodyType::DrawControls` radios (sample_bodies.cpp:180-214).
#[wasm_bindgen]
pub fn bodies_set_type(t: u32) {
    with_state(|state| {
        if state.kind != SceneKind::BodyType {
            return;
        }
        let bt = match t {
            0 => BodyType::Static,
            1 => BodyType::Kinematic,
            _ => BodyType::Dynamic,
        };
        state.body_type = bt;
        let ids = [
            state.bt_platform,
            state.bt_attach2,
            state.bt_payload2,
            state.bt_touching,
            state.bt_floating,
        ];
        for id in ids {
            body_set_type(&mut state.world, id, bt);
        }
        if bt == BodyType::Kinematic {
            body_set_linear_velocity(
                &mut state.world,
                state.bt_platform,
                v3(-state.speed, 0.0, 0.0),
            );
            body_set_angular_velocity(&mut state.world, state.bt_platform, VEC3_ZERO);
            body_set_linear_velocity(&mut state.world, state.bt_attach2, VEC3_ZERO);
            body_set_angular_velocity(&mut state.world, state.bt_attach2, VEC3_ZERO);
        }
    });
}

/// Toggle the Body Type sample's enable checkbox (attach1, crate2, floater).
#[wasm_bindgen]
pub fn bodies_set_enabled(flag: bool) {
    with_state(|state| {
        if state.kind != SceneKind::BodyType {
            return;
        }
        state.is_enabled = flag;
        let ids = [state.bt_attach1, state.bt_payload2, state.bt_floating];
        for id in ids {
            if flag {
                body_enable(&mut state.world, id);
            } else {
                body_disable(&mut state.world, id);
            }
        }
    });
}

// --- Disable controls ----------------------------------------------------------

#[wasm_bindgen]
pub fn bodies_enable_link(flag: bool) {
    with_state(|state| {
        if state.kind != SceneKind::Disable {
            return;
        }
        if flag {
            body_enable(&mut state.world, state.disable_ids[2]);
        } else {
            body_disable(&mut state.world, state.disable_ids[2]);
        }
    });
}

#[wasm_bindgen]
pub fn bodies_enable_ball(flag: bool) {
    with_state(|state| {
        if state.kind != SceneKind::Disable {
            return;
        }
        if flag {
            body_enable(&mut state.world, state.ball);
        } else {
            body_disable(&mut state.world, state.ball);
        }
    });
}

// --- Weeble controls -----------------------------------------------------------

#[wasm_bindgen]
pub fn bodies_teleport() {
    with_state(|state| {
        if state.kind != SceneKind::Weeble {
            return;
        }
        let q = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.95 * PI);
        body_set_transform(
            &mut state.world,
            state.weeble,
            Pos {
                x: 0.0,
                y: 5.0,
                z: 0.0,
            },
            q,
        );
        body_set_awake(&mut state.world, state.weeble, true);
    });
}

#[wasm_bindgen]
pub fn bodies_explode() {
    with_state(|state| {
        if state.kind != SceneKind::Weeble {
            return;
        }
        // Default mask bits come from default_explosion_def (C b3DefaultExplosionDef).
        let def = ExplosionDef {
            position: state.explosion_position,
            radius: state.explosion_radius,
            falloff: 0.1,
            impulse_per_area: state.explosion_magnitude,
            ..box3d_rust::types::default_explosion_def()
        };
        world_explode(&mut state.world, &def);
    });
}

#[wasm_bindgen]
pub fn bodies_set_magnitude(m: f32) {
    with_state(|state| {
        if state.kind == SceneKind::Weeble {
            state.explosion_magnitude = m;
        }
    });
}

// --- Cast target tracking (C BodyCast MouseDown/Move/Up, shift-drag) -----------

#[wasm_bindgen]
pub fn bodies_cast_track_down(ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32) {
    with_state(|state| {
        if state.kind != SceneKind::Cast {
            return;
        }
        let dir = normalize(v3(dx, dy, dz));
        state.cast_origin = offset_pos(
            Pos {
                x: ox as _,
                y: oy as _,
                z: oz as _,
            },
            v3(10.0 * dir.x, 10.0 * dir.y, 10.0 * dir.z),
        );
        state.cast_base = state.cast_transform.p;
        state.cast_tracking = true;
    });
}

#[wasm_bindgen]
pub fn bodies_cast_track_move(ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32) {
    with_state(|state| {
        if state.kind != SceneKind::Cast || !state.cast_tracking {
            return;
        }
        let dir = normalize(v3(dx, dy, dz));
        let origin = offset_pos(
            Pos {
                x: ox as _,
                y: oy as _,
                z: oz as _,
            },
            v3(10.0 * dir.x, 10.0 * dir.y, 10.0 * dir.z),
        );
        let delta = sub_pos(origin, state.cast_origin);
        state.cast_transform.p = offset_pos(state.cast_base, delta);
    });
}

#[wasm_bindgen]
pub fn bodies_cast_track_up() {
    with_state(|state| {
        state.cast_tracking = false;
    });
}

// --- Generic mouse grab (ctrl-drag) + spawn/delete -----------------------------

crate::demo_shell! {
    with_state: with_state,
    state: super::BodiesState,
    world: world,
    bodies: vis,
    grab: grab,
    base: |_s| ZERO_POS,
    mouse_down: bodies_mouse_down,
    mouse_move: bodies_mouse_move,
    mouse_up: bodies_mouse_up,
    mouse_active: bodies_mouse_active,
    spawn_random: bodies_spawn_random = |state, spawned| match spawned {
        Some(sp) => {
            // `interact::spawn_random` only ever returns the bullet sphere (kind 1);
            // the box arm mirrors the original dispatch for any future kind.
            let vb = match sp.kind {
                1 => VisBody::sphere_body(sp.body_index, sp.half_extents[0]),
                _ => VisBody::box_body(
                    sp.body_index,
                    sp.half_extents[0],
                    sp.half_extents[1],
                    sp.half_extents[2],
                ),
            };
            state.vis.push(vb);
            vec![1.0, sp.body_index as f32]
        }
        None => vec![0.0, 0.0],
    },
    delete_at_ray: bodies_delete_at_ray = |_state, _index| {},
    counters: bodies_counters,
    debug_draw: bodies_debug_draw,
    debug_text: bodies_debug_text,
}
