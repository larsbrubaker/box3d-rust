//! The wasm surface the Character page drives for the Dynamic Mover scene: reset, the
//! per-frame key / camera feed, the third-person mouse buttons, the K snap key, and the
//! `DrawControls` checkboxes and sliders (C `DynamicMoverSample::DrawControls`, :1701).

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use super::{build_dynamic_mover, DynamicMoverSample, GEM_COUNT};
use crate::character_demo::{install, with_state, CharacterState, SceneKind, SceneState};
use box3d_rust::body::{body_get_linear_velocity, body_get_position};
use wasm_bindgen::prelude::*;

/// `DrawControls` slider ids for [`character_dynamic_set_param`] /
/// [`character_dynamic_params`], in C order.
pub(crate) const PARAM_JUMP_SPEED: u32 = 0;
pub(crate) const PARAM_MAX_SPEED: u32 = 1;
pub(crate) const PARAM_ACCELERATE: u32 = 2;
pub(crate) const PARAM_FRICTION: u32 = 3;
pub(crate) const PARAM_AIR_STEER: u32 = 4;
pub(crate) const PARAM_GRAVITY_SCALE: u32 = 5;
pub(crate) const PARAM_POGO_HERTZ: u32 = 6;
pub(crate) const PARAM_POGO_DAMPING: u32 = 7;
pub(crate) const PARAM_GROUND_FORCE: u32 = 8;
pub(crate) const PARAM_BALL_SPEED: u32 = 9;

/// `DrawControls` checkbox ids for [`character_dynamic_set_flag`].
pub(crate) const FLAG_SKIRT_POGO: u32 = 0;
pub(crate) const FLAG_DRAW_PROBES: u32 = 1;
pub(crate) const FLAG_BOX_MOVER: u32 = 2;

/// Run `f` on the Dynamic Mover sample and the world beside it (no-op on other scenes).
fn with_sample<R>(f: impl FnOnce(&mut DynamicMoverSample, &mut CharacterParts) -> R) -> Option<R> {
    with_state(|state| with_sample_in(state, f))
}

/// The `CharacterState` fields a sample call may touch, split-borrowed beside the sample.
pub(crate) struct CharacterParts<'a> {
    pub world: &'a mut box3d_rust::world::World,
    pub bodies: &'a mut Vec<crate::vis::VisBody>,
    pub hulls: &'a mut Vec<crate::character_demo::hulls::HullVis>,
    pub debug_segs: &'a mut Vec<f32>,
    pub debug_pts: &'a mut Vec<f32>,
    pub status: &'a mut Vec<f32>,
    pub third_person: bool,
}

fn with_sample_in<R>(
    state: &mut CharacterState,
    f: impl FnOnce(&mut DynamicMoverSample, &mut CharacterParts) -> R,
) -> Option<R> {
    let CharacterState {
        world,
        bodies,
        hulls,
        debug_segs,
        debug_pts,
        status,
        third_person,
        state: scene,
        ..
    } = state;
    let SceneState::Dynamic(sample) = scene else {
        return None;
    };
    let mut parts = CharacterParts {
        world,
        bodies,
        hulls,
        debug_segs,
        debug_pts,
        status,
        third_person: *third_person,
    };
    Some(f(sample, &mut parts))
}

/// Reset the Dynamic Mover scene (C `DynamicMoverSample` constructor).
#[wasm_bindgen]
pub fn character_reset_dynamic_mover() -> u32 {
    let (world, sample) = build_dynamic_mover();
    let mut state = CharacterState::new(
        SceneKind::DynamicMover,
        world,
        SceneState::Dynamic(Box::new(sample)),
    );
    if let SceneState::Dynamic(sample) = &mut state.state {
        state.hulls = std::mem::take(&mut sample.hulls);
        let (bodies, _) = sample.render_bodies(&state.world);
        state.bodies = bodies;
    }
    state.third_person = true;
    install(state)
}

/// C `MouseDown` (third person): button 1 (right) starts aiming, button 0 (left)
/// shoots while aiming.
#[wasm_bindgen]
pub fn character_dynamic_mouse_down(button: i32) {
    with_sample(|sample, state| {
        sample.camera.third_person = state.third_person;
        sample.mouse_down(button);
    });
}

/// C `MouseUp`: releasing button 1 (right) stops aiming.
#[wasm_bindgen]
pub fn character_dynamic_mouse_up(button: i32) {
    with_sample(|sample, _| sample.mouse_up(button));
}

/// C `Keyboard` KEY_K — align movement for video. Returns the snapped `[yaw, pitch]`
/// (radians) for the page camera.
#[wasm_bindgen]
pub fn character_dynamic_snap() -> Vec<f32> {
    with_sample(|sample, state| {
        sample.snap_for_video(state.world);
        vec![sample.camera.yaw, sample.camera.pitch]
    })
    .unwrap_or_default()
}

/// Set one `DrawControls` slider (see the `PARAM_*` ids).
#[wasm_bindgen]
pub fn character_dynamic_set_param(id: u32, value: f32) {
    with_sample(|sample, state| {
        let m = &mut sample.mover;
        match id {
            PARAM_JUMP_SPEED => m.jump_speed = value,
            PARAM_MAX_SPEED => m.max_speed = value,
            PARAM_ACCELERATE => m.accelerate = value,
            PARAM_FRICTION => m.friction = value,
            PARAM_AIR_STEER => m.air_steer = value,
            PARAM_GRAVITY_SCALE => m.set_gravity_scale(state.world, value),
            PARAM_POGO_HERTZ => m.pogo_hertz = value,
            PARAM_POGO_DAMPING => m.pogo_damping_ratio = value,
            PARAM_GROUND_FORCE => m.max_ground_force = value,
            PARAM_BALL_SPEED => sample.ball_speed = value,
            _ => {}
        }
    });
}

/// Current slider values in `PARAM_*` order, then the checkbox states in `FLAG_*` order
/// (1.0 = checked) — the page seeds its controls from these.
#[wasm_bindgen]
pub fn character_dynamic_params() -> Vec<f32> {
    with_sample(|sample, _| {
        let m = &sample.mover;
        vec![
            m.jump_speed,
            m.max_speed,
            m.accelerate,
            m.friction,
            m.air_steer,
            m.gravity_scale,
            m.pogo_hertz,
            m.pogo_damping_ratio,
            m.max_ground_force,
            sample.ball_speed,
            if m.enable_skirt_pogo { 1.0 } else { 0.0 },
            if sample.draw_probes { 1.0 } else { 0.0 },
            if sample.use_box { 1.0 } else { 0.0 },
        ]
    })
    .unwrap_or_default()
}

/// Set one `DrawControls` checkbox (see the `FLAG_*` ids). "Box Mover" swaps the
/// mover shape exactly like C (`if ( ImGui::Checkbox( ... ) ) SwapMoverShape()`).
#[wasm_bindgen]
pub fn character_dynamic_set_flag(id: u32, on: bool) {
    with_sample(|sample, state| match id {
        FLAG_SKIRT_POGO => sample.mover.enable_skirt_pogo = on,
        FLAG_DRAW_PROBES => sample.draw_probes = on,
        FLAG_BOX_MOVER if sample.use_box != on => {
            sample.use_box = on;
            sample.swap_mover_shape(state.world);
        }
        _ => {}
    });
}

/// One Character-page step of the Dynamic Mover scene: sync the third-person toggle,
/// run C `Step`, then draw the probes / launcher and rebuild the render list.
///
/// Fills `state.status` with:
/// `[px,py,pz, vx,vy,vz, onGround, walkable, normalY, pogoLength, pogoRestLength,
///   timeSliding, gemsRemaining, gemCount, moverVisIndex, aiming, moverAlpha,
///   viewPivotX, viewPivotY, viewPivotZ, viewRadius]`.
pub(crate) fn step(state: &mut CharacterState, dt: f32, sub_steps: i32) {
    with_sample_in(state, |sample, state| {
        sample.camera.third_person = state.third_person;
        sample.step(state.world, dt, sub_steps);
        sample.draw(state.world, state.debug_segs, state.debug_pts, state.hulls);
        let (bodies, mover_index) = sample.render_bodies(state.world);
        *state.bodies = bodies;

        let mover_id = sample.mover.body_id();
        let p = body_get_position(state.world, mover_id);
        let v = body_get_linear_velocity(state.world, mover_id);
        let m = &sample.mover;
        let normal_y = if m.cast_result.hit {
            m.cast_result.normal.y
        } else {
            0.0
        };
        let pivot = sample.camera.pivot;
        *state.status = vec![
            p.x as f32,
            p.y as f32,
            p.z as f32,
            v.x,
            v.y,
            v.z,
            m.on_ground as i32 as f32,
            m.walkable as i32 as f32,
            normal_y,
            m.pogo_length,
            m.pogo_rest_length,
            m.time_sliding,
            sample.gems_remaining() as f32,
            GEM_COUNT as f32,
            mover_index as f32,
            sample.aiming as i32 as f32,
            sample.body_alpha,
            pivot.x as f32,
            pivot.y as f32,
            pivot.z as f32,
            sample.view_radius,
        ];
    });
}
