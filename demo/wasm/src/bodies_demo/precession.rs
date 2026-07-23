//! Gyroscopic Precession diagnostic (`sample_bodies.cpp` GyroscopicPrecession,
//! :376-578). Ported from PEEL: a field of tilted, fast-spinning tops that precess
//! under the gravity torque about their tips. The first top (`m_topId`) carries a
//! diagnostic that compares the measured precession rate against the classical
//! heavy symmetric-top solution (Goldstein 5.7).
//!
//! Split into its own submodule so `bodies_demo/mod.rs` stays under the file-length
//! gate; the per-step accumulators live in [`PrecessionState`], a single field on
//! `BodiesState`, and the diagnostic runs after `world.step` each frame (matching
//! C's `Sample::Step()` then diagnostic ordering).

use super::{BodiesState, SceneKind};
use box3d_rust::body::{
    body_get_angular_velocity, body_get_position, body_get_transform, body_is_awake,
    get_body_transform,
};
use box3d_rust::id::{BodyId, NULL_BODY_ID};
use box3d_rust::math_functions::{
    atan2, clamp_float, dot, rotate_vector, unwind_angle, Pos, Vec3, PI, VEC3_AXIS_Y, VEC3_ZERO,
};
use wasm_bindgen::prelude::*;

/// Heavy-top precession diagnostic state (`GyroscopicPrecession` members). The
/// measured top is `top_id`; `top_indices` is every top's render-body index (all
/// share one hull geometry, `hull_tris` / `hull_edges`, hull-local).
pub struct PrecessionState {
    pub top_id: BodyId,
    pub top_indices: Vec<i32>,
    pub hull_tris: Vec<f32>,
    pub hull_edges: Vec<f32>,

    // Mass properties of the measured top (constant after construction).
    pub mass: f32,
    pub gravity: f32,
    pub pivot_distance: f32,
    pub spin_inertia: f32,
    pub transverse_inertia: f32,

    // Running measurement (`m_azimuth`, `m_actualAngle`, ...).
    pub azimuth: f32,
    pub actual_angle: f32,
    pub expected_angle: f32,
    pub elapsed: f32,
    pub ground_time: f32,
    pub rate: f32,
    pub measuring: bool,

    // Per-step snapshot for the HUD text and the axis line overlay.
    pub awake: bool,
    pub tip: Pos,
    pub axis: Vec3,
    pub spin: f32,
    pub cos_tilt: f32,
    pub expected: f32,
}

impl Default for PrecessionState {
    fn default() -> Self {
        PrecessionState {
            top_id: NULL_BODY_ID,
            top_indices: Vec::new(),
            hull_tris: Vec::new(),
            hull_edges: Vec::new(),
            mass: 0.0,
            gravity: 0.0,
            pivot_distance: 0.0,
            spin_inertia: 0.0,
            transverse_inertia: 0.0,
            azimuth: 0.0,
            actual_angle: 0.0,
            expected_angle: 0.0,
            elapsed: 0.0,
            ground_time: 0.0,
            rate: 0.0,
            measuring: false,
            awake: false,
            tip: VEC3_ZERO,
            axis: VEC3_AXIS_Y,
            spin: 0.0,
            cos_tilt: 0.0,
            expected: 0.0,
        }
    }
}

impl PrecessionState {
    /// Steady precession rate of a heavy symmetric top about the vertical. The slow
    /// root of `I1 * W^2 * cos(tilt) - I3 * spin * W + M * g * d = 0` (Goldstein 5.7).
    /// Collapses to torque over spin momentum for a fast top.
    /// (`GyroscopicPrecession::ExpectedRate`, sample_bodies.cpp:461-479.)
    fn expected_rate(&self, spin: f32, cos_tilt: f32) -> f32 {
        let momentum = self.spin_inertia * spin;
        let torque = self.mass * self.gravity * self.pivot_distance;
        if momentum <= f32::EPSILON {
            return 0.0;
        }

        let a = self.transverse_inertia * cos_tilt;
        let discriminant = momentum * momentum - 4.0 * a * torque;
        if a <= f32::EPSILON || discriminant < 0.0 {
            // Axis at or past horizontal, or spinning too slowly to precess steadily.
            return torque / momentum;
        }

        (momentum - discriminant.sqrt()) / (2.0 * a)
    }
}

/// Advance the diagnostic one physics step (called after `world.step`, so the
/// state read here is post-step, matching `Sample::Step()` then the C diagnostic).
/// `time_step` is `1 / hertz` (the full world step dt), matching `m_context->hertz`.
pub fn step(state: &mut BodiesState, time_step: f32) {
    let top = state.prec.top_id;

    // A sleeping top gives no useful measurement; C prints "top is sleeping" and returns.
    if !body_is_awake(&state.world, top) {
        state.prec.awake = false;
        return;
    }

    let tip = body_get_position(&state.world, top);
    let quat = body_get_transform(&state.world, top).q;
    let axis = rotate_vector(quat, VEC3_AXIS_Y);
    let omega = body_get_angular_velocity(&state.world, top);
    let spin = dot(omega, axis);
    let cos_tilt = clamp_float(axis.y, -1.0, 1.0);
    let expected = state.prec.expected_rate(spin, cos_tilt);

    // C guards this with `m_didStep`; in the wasm model `world.step` always ran before
    // this call, so `m_didStep` is effectively always true here.
    //
    // Gravity exerts no torque about the center of mass while the top is airborne, so the
    // pivot solution only applies once the tip lands. Give the landing impulse time to wash
    // out too, otherwise it biases the average for the rest of the run.
    if tip.y < 0.05 {
        state.prec.ground_time += time_step;
    }

    if !state.prec.measuring {
        if state.prec.ground_time > 0.5 {
            state.prec.measuring = true;
            state.prec.azimuth = atan2(-axis.z, axis.x);
        }
    } else {
        // Right handed angle of the symmetry axis about the world up axis.
        let azimuth = atan2(-axis.z, axis.x);
        let delta = unwind_angle(azimuth - state.prec.azimuth);
        state.prec.azimuth = azimuth;

        state.prec.actual_angle += delta;
        state.prec.expected_angle += expected * time_step;
        state.prec.elapsed += time_step;

        // Nutation swings the instantaneous rate between zero and twice the mean, so filter
        // it with a one second time constant.
        let alpha = time_step / (time_step + 1.0);
        state.prec.rate += alpha * (delta / time_step - state.prec.rate);
    }

    state.prec.awake = true;
    state.prec.tip = tip;
    state.prec.axis = axis;
    state.prec.spin = spin;
    state.prec.cos_tilt = cos_tilt;
    state.prec.expected = expected;
}

/// The measured top's symmetry-axis line (`DrawLine(tip, tip + 5*axis, yellow)`,
/// sample_bodies.cpp:538). Drawn only while the top is awake, like C.
pub fn overlay(state: &BodiesState, ov: &mut super::Overlay) {
    let p = &state.prec;
    if !p.awake {
        return;
    }
    let tip = super::pos_to_v3(p.tip);
    let end = super::v3(
        tip.x + 5.0 * p.axis.x,
        tip.y + 5.0 * p.axis.y,
        tip.z + 5.0 * p.axis.z,
    );
    ov.seg(tip, end, super::C_YELLOW);
}

/// The heavy-top HUD text (`GyroscopicPrecession::Step` DrawTextLine calls,
/// sample_bodies.cpp:540-555), newline-joined; the TS scene renders one line each.
fn hud_string(state: &BodiesState) -> String {
    let p = &state.prec;
    if !p.awake {
        return "top is sleeping".to_string();
    }

    let mut lines: Vec<String> = Vec::new();
    lines.push(format!(
        "spin {:.1} rad/s, tilt {:.1} deg",
        p.spin,
        (180.0 / PI) * p.cos_tilt.acos()
    ));
    lines.push(format!(
        "precession: expected {:.4} rad/s, actual {:.4} rad/s",
        p.expected, p.rate
    ));

    if p.elapsed > 0.0 {
        // Average both sides over the same window. The spin decays, so the expected rate
        // drifts with it.
        let expected_average = p.expected_angle / p.elapsed;
        let actual_average = p.actual_angle / p.elapsed;
        let error = if expected_average != 0.0 {
            100.0 * (actual_average - expected_average) / expected_average
        } else {
            0.0
        };
        lines.push(format!(
            "{:.1} s average: expected {:.4}, actual {:.4}, error {:+.1}%",
            p.elapsed, expected_average, actual_average, error
        ));
    } else {
        lines.push("waiting for the tip to land".to_string());
    }

    lines.join("\n")
}

// --- Exports -------------------------------------------------------------------

/// Heavy-top HUD text for the Gyroscopic Precession scene (newline-separated lines);
/// `""` for the other scenes.
#[wasm_bindgen]
pub fn bodies_precession_hud() -> String {
    super::with_state(|state| {
        if state.kind == SceneKind::GyroscopicPrecession {
            hud_string(state)
        } else {
            String::new()
        }
    })
}

/// Shared top hull geometry, hull-local: `[triFloatCount, tris…, edgeFloatCount,
/// edges…]`. Emitted once (all tops share the same hull); empty for other scenes.
#[wasm_bindgen]
pub fn bodies_precession_hull() -> Vec<f32> {
    super::with_state(|state| {
        if state.kind != SceneKind::GyroscopicPrecession {
            return Vec::new();
        }
        let p = &state.prec;
        let mut out = Vec::with_capacity(2 + p.hull_tris.len() + p.hull_edges.len());
        out.push(p.hull_tris.len() as f32);
        out.extend_from_slice(&p.hull_tris);
        out.push(p.hull_edges.len() as f32);
        out.extend_from_slice(&p.hull_edges);
        out
    })
}

/// Live world transform per top, index-aligned to [`bodies_precession_hull`]'s
/// single geometry: `[px,py,pz, qx,qy,qz,qw] × N`.
#[wasm_bindgen]
pub fn bodies_precession_poses() -> Vec<f32> {
    super::with_state(|state| {
        let mut out = Vec::with_capacity(state.prec.top_indices.len() * 7);
        for &idx in &state.prec.top_indices {
            let xf = get_body_transform(&state.world, idx);
            out.extend_from_slice(&[
                xf.p.x as f32,
                xf.p.y as f32,
                xf.p.z as f32,
                xf.q.v.x,
                xf.q.v.y,
                xf.q.v.z,
                xf.q.s,
            ]);
        }
        out
    })
}
