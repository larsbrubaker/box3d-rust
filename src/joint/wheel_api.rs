// Port of wheel_joint.c public accessors and force/torque reporting.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{get_body_state_index, get_body_transform};
use crate::id::JointId;
use crate::math_functions::{
    atan2, dot, make_matrix_from_quat, mul_quat, mul_sv, rotate_vector, sub, Vec2, Vec3, VEC3_AXIS_Z,
    VEC3_ZERO,
};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b3WheelJoint_EnableSuspension)
pub fn wheel_joint_enable_suspension(world: &mut World, joint_id: JointId, enable_spring: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel).wheel_mut();
    if enable_spring != joint.enable_suspension_spring {
        joint.enable_suspension_spring = enable_spring;
        joint.suspension_spring_impulse = 0.0;
    }
}

/// (b3WheelJoint_IsSuspensionEnabled)
pub fn wheel_joint_is_suspension_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .enable_suspension_spring
}

/// (b3WheelJoint_SetSuspensionHertz)
pub fn wheel_joint_set_suspension_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    get_joint_sim_check_type(world, joint_id, JointType::Wheel)
        .wheel_mut()
        .suspension_hertz = hertz;
}

/// (b3WheelJoint_GetSuspensionHertz)
pub fn wheel_joint_get_suspension_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .suspension_hertz
}

/// (b3WheelJoint_SetSuspensionDampingRatio)
pub fn wheel_joint_set_suspension_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    get_joint_sim_check_type(world, joint_id, JointType::Wheel)
        .wheel_mut()
        .suspension_damping_ratio = damping_ratio;
}

/// (b3WheelJoint_GetSuspensionDampingRatio)
pub fn wheel_joint_get_suspension_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .suspension_damping_ratio
}

/// (b3WheelJoint_EnableSuspensionLimit)
pub fn wheel_joint_enable_suspension_limit(
    world: &mut World,
    joint_id: JointId,
    enable_limit: bool,
) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel).wheel_mut();
    if joint.enable_suspension_limit != enable_limit {
        joint.lower_suspension_impulse = 0.0;
        joint.upper_suspension_impulse = 0.0;
        joint.enable_suspension_limit = enable_limit;
    }
}

/// (b3WheelJoint_IsSuspensionLimitEnabled)
pub fn wheel_joint_is_suspension_limit_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .enable_suspension_limit
}

/// (b3WheelJoint_GetLowerSuspensionLimit)
pub fn wheel_joint_get_lower_suspension_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .lower_suspension_limit
}

/// (b3WheelJoint_GetUpperSuspensionLimit)
pub fn wheel_joint_get_upper_suspension_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .upper_suspension_limit
}

/// (b3WheelJoint_SetSuspensionLimits)
pub fn wheel_joint_set_suspension_limits(
    world: &mut World,
    joint_id: JointId,
    lower: f32,
    upper: f32,
) {
    debug_assert!(lower <= upper);
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel).wheel_mut();
    if lower != joint.lower_suspension_limit || upper != joint.upper_suspension_limit {
        joint.lower_suspension_limit = lower;
        joint.upper_suspension_limit = upper;
        joint.lower_suspension_impulse = 0.0;
        joint.upper_suspension_impulse = 0.0;
    }
}

/// (b3WheelJoint_EnableSpinMotor)
pub fn wheel_joint_enable_spin_motor(world: &mut World, joint_id: JointId, enable_motor: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel).wheel_mut();
    if joint.enable_spin_motor != enable_motor {
        joint.spin_impulse = 0.0;
        joint.enable_spin_motor = enable_motor;
    }
}

/// (b3WheelJoint_IsSpinMotorEnabled)
pub fn wheel_joint_is_spin_motor_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .enable_spin_motor
}

/// (b3WheelJoint_SetSpinMotorSpeed)
pub fn wheel_joint_set_spin_motor_speed(world: &mut World, joint_id: JointId, motor_speed: f32) {
    get_joint_sim_check_type(world, joint_id, JointType::Wheel)
        .wheel_mut()
        .spin_speed = motor_speed;
}

/// (b3WheelJoint_GetSpinMotorSpeed)
pub fn wheel_joint_get_spin_motor_speed(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .spin_speed
}

/// (b3WheelJoint_SetMaxSpinTorque)
pub fn wheel_joint_set_max_spin_torque(world: &mut World, joint_id: JointId, torque: f32) {
    get_joint_sim_check_type(world, joint_id, JointType::Wheel)
        .wheel_mut()
        .max_spin_torque = torque;
}

/// (b3WheelJoint_GetMaxSpinTorque)
pub fn wheel_joint_get_max_spin_torque(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .max_spin_torque
}

/// (b3WheelJoint_EnableSteering)
pub fn wheel_joint_enable_steering(world: &mut World, joint_id: JointId, flag: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel).wheel_mut();
    if joint.enable_steering != flag {
        joint.angular_impulse = Vec2 { x: 0.0, y: 0.0 };
        joint.enable_steering = flag;
    }
}

/// (b3WheelJoint_IsSteeringEnabled)
pub fn wheel_joint_is_steering_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .enable_steering
}

/// (b3WheelJoint_SetSteeringHertz)
pub fn wheel_joint_set_steering_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    get_joint_sim_check_type(world, joint_id, JointType::Wheel)
        .wheel_mut()
        .steering_hertz = hertz;
}

/// (b3WheelJoint_GetSteeringHertz)
pub fn wheel_joint_get_steering_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .steering_hertz
}

/// (b3WheelJoint_SetSteeringDampingRatio)
pub fn wheel_joint_set_steering_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    get_joint_sim_check_type(world, joint_id, JointType::Wheel)
        .wheel_mut()
        .steering_damping_ratio = damping_ratio;
}

/// (b3WheelJoint_GetSteeringDampingRatio)
pub fn wheel_joint_get_steering_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .steering_damping_ratio
}

/// (b3WheelJoint_SetMaxSteeringTorque)
pub fn wheel_joint_set_max_steering_torque(world: &mut World, joint_id: JointId, max_torque: f32) {
    get_joint_sim_check_type(world, joint_id, JointType::Wheel)
        .wheel_mut()
        .max_steering_torque = max_torque;
}

/// (b3WheelJoint_GetMaxSteeringTorque)
pub fn wheel_joint_get_max_steering_torque(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .max_steering_torque
}

/// (b3WheelJoint_EnableSteeringLimit)
pub fn wheel_joint_enable_steering_limit(world: &mut World, joint_id: JointId, flag: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel).wheel_mut();
    if joint.enable_steering_limit != flag {
        joint.lower_steering_impulse = 0.0;
        joint.upper_steering_impulse = 0.0;
        joint.enable_steering_limit = flag;
    }
}

/// (b3WheelJoint_IsSteeringLimitEnabled)
pub fn wheel_joint_is_steering_limit_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .enable_steering_limit
}

/// (b3WheelJoint_GetLowerSteeringLimit)
pub fn wheel_joint_get_lower_steering_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .lower_steering_limit
}

/// (b3WheelJoint_GetUpperSteeringLimit)
pub fn wheel_joint_get_upper_steering_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .upper_steering_limit
}

/// (b3WheelJoint_SetSteeringLimits)
pub fn wheel_joint_set_steering_limits(
    world: &mut World,
    joint_id: JointId,
    lower_radians: f32,
    upper_radians: f32,
) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Wheel).wheel_mut();
    joint.lower_steering_limit = lower_radians;
    joint.upper_steering_limit = upper_radians;
}

/// (b3WheelJoint_SetTargetSteeringAngle)
pub fn wheel_joint_set_target_steering_angle(world: &mut World, joint_id: JointId, radians: f32) {
    get_joint_sim_check_type(world, joint_id, JointType::Wheel)
        .wheel_mut()
        .target_steering_angle = radians;
}

/// (b3WheelJoint_GetTargetSteeringAngle)
pub fn wheel_joint_get_target_steering_angle(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel)
        .wheel()
        .target_steering_angle
}

/// (b3WheelJoint_GetSpinSpeed)
pub fn wheel_joint_get_spin_speed(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    let id_a = base.body_id_a;
    let id_b = base.body_id_b;

    let body_b = &world.bodies[id_b as usize];
    let body_sim_b =
        &world.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];
    let quat_b = mul_quat(body_sim_b.transform.q, base.local_frame_b.q);
    let spin_axis = rotate_vector(quat_b, VEC3_AXIS_Z);

    let mut w_a = VEC3_ZERO;
    if let Some(local) = get_body_state_index(world, id_a) {
        w_a = world.solver_sets[AWAKE_SET as usize].body_states[local as usize].angular_velocity;
    }
    let mut w_b = VEC3_ZERO;
    if let Some(local) = get_body_state_index(world, id_b) {
        w_b = world.solver_sets[AWAKE_SET as usize].body_states[local as usize].angular_velocity;
    }

    dot(sub(w_b, w_a), spin_axis)
}

/// (b3WheelJoint_GetSpinTorque)
pub fn wheel_joint_get_spin_torque(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    world.inv_h * base.wheel().spin_impulse
}

/// (b3WheelJoint_GetSteeringAngle)
pub fn wheel_joint_get_steering_angle(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    let body_a = &world.bodies[base.body_id_a as usize];
    let body_b = &world.bodies[base.body_id_b as usize];
    let body_sim_a =
        &world.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
    let body_sim_b =
        &world.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];

    let quat_a = mul_quat(body_sim_a.transform.q, base.local_frame_a.q);
    let quat_b = mul_quat(body_sim_b.transform.q, base.local_frame_b.q);
    let matrix_a = make_matrix_from_quat(quat_a);
    let matrix_b = make_matrix_from_quat(quat_b);

    // Twist around x-axis
    let cs = dot(matrix_b.cz, matrix_a.cz);
    let ss = -dot(matrix_b.cz, matrix_a.cy);
    atan2(ss, cs)
}

/// (b3WheelJoint_GetSteeringTorque)
pub fn wheel_joint_get_steering_torque(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Wheel);
    world.inv_h * base.wheel().steering_spring_impulse
}

/// (b3GetWheelJointForce)
pub fn get_wheel_joint_force(world: &World, base: &JointSim) -> Vec3 {
    let transform_a = get_body_transform(world, base.body_id_a);
    let joint = base.wheel();

    // impulse in joint space (C uses lowerSuspensionLimit here — port exactly)
    let impulse = Vec3 {
        x: joint.linear_impulse.x,
        y: joint.linear_impulse.y,
        z: joint.lower_suspension_limit
            + joint.upper_suspension_impulse
            + joint.suspension_spring_impulse,
    };

    // convert impulse to force
    let force = mul_sv(world.inv_h, impulse);
    // convert to body space then world space
    let force = rotate_vector(base.local_frame_a.q, force);
    rotate_vector(transform_a.q, force)
}

/// (b3GetWheelJointTorque)
pub fn get_wheel_joint_torque(world: &World, base: &JointSim) -> Vec3 {
    debug_assert!(base.type_ == JointType::Wheel);

    let body_a = &world.bodies[base.body_id_a as usize];
    let body_sim_a =
        &world.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
    let q_a = mul_quat(body_sim_a.transform.q, base.local_frame_a.q);
    let matrix_a = make_matrix_from_quat(q_a);
    mul_sv(world.inv_h * base.wheel().spin_impulse, matrix_a.cz)
}
