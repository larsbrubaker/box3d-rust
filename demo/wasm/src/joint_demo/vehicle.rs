//! Wheel joint (sample_joint.cpp `WheelJoint`) — a single cylinder wheel hung from
//! ground with suspension / spin-motor / steering controls.

use super::{add_ground_box, empty_state, install, new_world, JointScene};
use crate::vis::{pos, vec3, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::hull::create_cylinder;
use box3d_rust::joint::{
    create_wheel_joint, joint_wake_bodies, wheel_joint_enable_spin_motor,
    wheel_joint_enable_steering, wheel_joint_enable_steering_limit, wheel_joint_enable_suspension,
    wheel_joint_enable_suspension_limit, wheel_joint_get_steering_angle,
    wheel_joint_set_max_spin_torque, wheel_joint_set_spin_motor_speed,
    wheel_joint_set_steering_hertz, wheel_joint_set_steering_limits,
    wheel_joint_set_suspension_damping_ratio, wheel_joint_set_suspension_hertz,
    wheel_joint_set_suspension_limits, wheel_joint_set_target_steering_angle,
};
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, Transform, PI, QUAT_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y,
    VEC3_AXIS_Z,
};
use box3d_rust::shape::create_hull_shape;
use box3d_rust::types::{default_body_def, default_shape_def, default_wheel_joint_def, BodyType};
use wasm_bindgen::prelude::*;

const WHEEL_HEIGHT: f32 = 0.25;
const WHEEL_RADIUS: f32 = 0.4;

#[wasm_bindgen]
pub fn joint_reset_wheel() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();
    add_ground_box(&mut world, &mut bodies, 20.0);
    let mut ground_def = default_body_def();
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut world, &ground_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 2.0, 0.0);
    body_def.rotation = compute_quat_between_unit_vectors(VEC3_AXIS_Y, VEC3_AXIS_Z);
    let body = create_body(&mut world, &body_def);

    let cyl = create_cylinder(WHEEL_HEIGHT, WHEEL_RADIUS, 0.0, 12).expect("wheel cylinder");
    create_hull_shape(&mut world, body, &default_shape_def(), &cyl);
    // The cylinder spans body-local y ∈ [0, height]; the render mesh is centered,
    // so offset it by +height/2 and use half the height.
    bodies.push(VisBody::cylinder_local(
        body.index1 - 1,
        WHEEL_RADIUS,
        0.5 * WHEEL_HEIGHT,
        Transform {
            p: vec3(0.0, 0.5 * WHEEL_HEIGHT, 0.0),
            q: QUAT_IDENTITY,
        },
        box3d_rust::debug_draw::HexColor::SLATE_GRAY.0,
    ));

    let mut joint_def = default_wheel_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = body;
    joint_def.base.local_frame_a.p = vec3(0.0, 3.0, 0.0);
    joint_def.base.local_frame_a.q = compute_quat_between_unit_vectors(VEC3_AXIS_X, VEC3_AXIS_Y);
    joint_def.base.local_frame_b.p = vec3(0.0, 0.0, 0.0);
    joint_def.base.local_frame_b.q = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    joint_def.base.collide_connected = true;
    joint_def.enable_suspension_limit = false;
    joint_def.lower_suspension_limit = -1.0;
    joint_def.upper_suspension_limit = 1.0;
    joint_def.enable_suspension_spring = false;
    joint_def.suspension_hertz = 2.0;
    joint_def.suspension_damping_ratio = 0.7;
    joint_def.enable_spin_motor = false;
    joint_def.max_spin_torque = 20.0;
    joint_def.spin_speed = 0.0;
    joint_def.enable_steering = false;
    joint_def.steering_hertz = 1.0;
    joint_def.steering_damping_ratio = 0.7;
    joint_def.target_steering_angle = 0.0;
    joint_def.max_steering_torque = 20.0;
    joint_def.enable_steering_limit = false;
    joint_def.lower_steering_limit = PI / 180.0 * -45.0;
    joint_def.upper_steering_limit = PI / 180.0 * 45.0;
    let joint = create_wheel_joint(&mut world, &joint_def);

    let mut state = empty_state(world, bodies, JointScene::Wheel);
    state.control_joint = joint;
    install(state)
}

/// C WheelJoint::DrawControls. `flags`: bit0=suspLimit, bit1=spinMotor,
/// bit2=suspSpring, bit3=steering, bit4=steeringLimit.
///
/// Note: C's steering "Damping" slider has a copy/paste bug (sample_joint.cpp:1463)
/// that never applies the steering damping and instead re-writes the suspension
/// damping. This setter reproduces that bug deliberately (strict-port rule): moving
/// the steering-damping slider does not change steering damping — see the inline
/// comment in the steering block below.
#[allow(clippy::too_many_arguments)]
#[wasm_bindgen]
pub fn joint_set_wheel_params(
    flags: u32,
    susp_min: f32,
    susp_max: f32,
    max_spin_torque: f32,
    spin_speed: f32,
    susp_hertz: f32,
    susp_damp: f32,
    steer_hertz: f32,
    steer_damp: f32,
    target_deg: f32,
    steer_min_deg: f32,
    steer_max_deg: f32,
) {
    super::with_state(|state| {
        let jid = state.control_joint;
        if !jid.is_non_null() {
            return;
        }
        let d2r = PI / 180.0;
        wheel_joint_enable_suspension_limit(&mut state.world, jid, flags & 1 != 0);
        wheel_joint_set_suspension_limits(&mut state.world, jid, susp_min, susp_max);
        wheel_joint_enable_spin_motor(&mut state.world, jid, flags & 2 != 0);
        wheel_joint_set_max_spin_torque(&mut state.world, jid, max_spin_torque);
        wheel_joint_set_spin_motor_speed(&mut state.world, jid, spin_speed);
        wheel_joint_enable_suspension(&mut state.world, jid, flags & 4 != 0);
        wheel_joint_set_suspension_hertz(&mut state.world, jid, susp_hertz);
        wheel_joint_set_suspension_damping_ratio(&mut state.world, jid, susp_damp);
        wheel_joint_enable_steering(&mut state.world, jid, flags & 8 != 0);
        wheel_joint_set_steering_hertz(&mut state.world, jid, steer_hertz);
        // C's steering "Damping" slider callback (sample_joint.cpp:1462-1463):
        //   ImGui::SliderFloat( "Damping##Steering", &m_steeringDampingRatio, ... )
        //   b3WheelJoint_SetSuspensionDampingRatio( m_jointId, m_suspensionDampingRatio );
        // A copy-paste bug: the slider edits `m_steeringDampingRatio` but the call
        // targets *suspension* damping (with the suspension value), so steering
        // damping is never updated and stays at its ctor value (0.7). We reproduce
        // this deliberately per the strict-port rule. `steer_damp` is kept in the
        // signature (unchanged TS setter shape) but intentionally not applied; the
        // call below mirrors C's re-write of suspension damping with the suspension
        // value (a no-op given it was already set above).
        let _ = steer_damp;
        wheel_joint_set_suspension_damping_ratio(&mut state.world, jid, susp_damp);
        wheel_joint_set_target_steering_angle(&mut state.world, jid, target_deg * d2r);
        wheel_joint_enable_steering_limit(&mut state.world, jid, flags & 16 != 0);
        wheel_joint_set_steering_limits(
            &mut state.world,
            jid,
            steer_min_deg * d2r,
            steer_max_deg * d2r,
        );
        joint_wake_bodies(&mut state.world, jid);
    });
}

/// C WheelJoint::Render HUD — steering angle in degrees.
#[wasm_bindgen]
pub fn joint_wheel_steering_angle() -> f32 {
    super::with_state(|state| {
        let jid = state.control_joint;
        if !jid.is_non_null() {
            return 0.0;
        }
        180.0 / PI * wheel_joint_get_steering_angle(&state.world, jid)
    })
}
