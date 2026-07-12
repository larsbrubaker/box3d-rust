// Port of distance_joint.c: public accessors, force reporting, and the
// prepare/warm-start/solve simulation functions.
//
// The C accessors resolve the world from the id via the global registry; the
// Rust port takes `world` explicitly. Prepare takes &World (caller owns the
// JointSim); warm start and solve take the awake body states slice and copy
// states out/back, writing only under the dynamic-flag guard like C.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{body_flags, get_body_transform, BodyState, IDENTITY_BODY_STATE};
use crate::constants::{huge, linear_slop};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::math_functions::{
    add, clamp_float, cross, dot, length, max_float, min_float, mul_add, mul_mv, mul_sub, mul_sv,
    normalize, rotate_vector, sub, sub_pos, transform_world_point, Vec3,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b3DistanceJoint_SetLength)
pub fn distance_joint_set_length(world: &mut World, joint_id: JointId, length: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_set_length(joint_id, length);
    });
    let base = get_joint_sim_check_type(world, joint_id, JointType::Distance);
    let joint = base.distance_mut();

    joint.length = clamp_float(length, linear_slop(), huge());
    joint.impulse = 0.0;
    joint.lower_impulse = 0.0;
    joint.upper_impulse = 0.0;
}

/// (b3DistanceJoint_GetLength)
pub fn distance_joint_get_length(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .length
}

/// (b3DistanceJoint_EnableLimit)
pub fn distance_joint_enable_limit(world: &mut World, joint_id: JointId, enable_limit: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_enable_limit(joint_id, enable_limit);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Distance)
        .distance_mut()
        .enable_limit = enable_limit;
}

/// (b3DistanceJoint_IsLimitEnabled)
pub fn distance_joint_is_limit_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .enable_limit
}

/// (b3DistanceJoint_SetLengthRange)
pub fn distance_joint_set_length_range(
    world: &mut World,
    joint_id: JointId,
    min_length: f32,
    max_length: f32,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_set_length_range(joint_id, min_length, max_length);
    });
    let base = get_joint_sim_check_type(world, joint_id, JointType::Distance);
    let joint = base.distance_mut();

    let min_length = clamp_float(min_length, linear_slop(), huge());
    let max_length = clamp_float(max_length, linear_slop(), huge());
    joint.min_length = min_float(min_length, max_length);
    joint.max_length = max_float(min_length, max_length);
    joint.impulse = 0.0;
    joint.lower_impulse = 0.0;
    joint.upper_impulse = 0.0;
}

/// (b3DistanceJoint_GetMinLength)
pub fn distance_joint_get_min_length(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .min_length
}

/// (b3DistanceJoint_GetMaxLength)
pub fn distance_joint_get_max_length(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .max_length
}

/// (b3DistanceJoint_GetCurrentLength)
pub fn distance_joint_get_current_length(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Distance);

    let transform_a = get_body_transform(world, base.body_id_a);
    let transform_b = get_body_transform(world, base.body_id_b);

    let p_a = transform_world_point(transform_a, base.local_frame_a.p);
    let p_b = transform_world_point(transform_b, base.local_frame_b.p);
    let d = sub_pos(p_b, p_a);
    length(d)
}

/// (b3DistanceJoint_EnableSpring)
pub fn distance_joint_enable_spring(world: &mut World, joint_id: JointId, enable_spring: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_enable_spring(joint_id, enable_spring);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Distance)
        .distance_mut()
        .enable_spring = enable_spring;
}

/// (b3DistanceJoint_IsSpringEnabled)
pub fn distance_joint_is_spring_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .enable_spring
}

/// (b3DistanceJoint_SetSpringForceRange)
pub fn distance_joint_set_spring_force_range(
    world: &mut World,
    joint_id: JointId,
    lower_force: f32,
    upper_force: f32,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_set_spring_force_range(joint_id, lower_force, upper_force);
    });
    debug_assert!(lower_force <= upper_force);
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Distance).distance_mut();
    joint.lower_spring_force = lower_force;
    joint.upper_spring_force = upper_force;
}

/// (b3DistanceJoint_GetSpringForceRange)
pub fn distance_joint_get_spring_force_range(world: &World, joint_id: JointId) -> (f32, f32) {
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Distance).distance();
    (joint.lower_spring_force, joint.upper_spring_force)
}

/// (b3DistanceJoint_SetSpringHertz)
pub fn distance_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_set_spring_hertz(joint_id, hertz);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Distance)
        .distance_mut()
        .hertz = hertz;
}

/// (b3DistanceJoint_SetSpringDampingRatio)
pub fn distance_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_set_spring_damping_ratio(joint_id, damping_ratio);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Distance)
        .distance_mut()
        .damping_ratio = damping_ratio;
}

/// (b3DistanceJoint_GetSpringHertz)
pub fn distance_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .hertz
}

/// (b3DistanceJoint_GetSpringDampingRatio)
pub fn distance_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .damping_ratio
}

/// (b3DistanceJoint_EnableMotor)
pub fn distance_joint_enable_motor(world: &mut World, joint_id: JointId, enable_motor: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_enable_motor(joint_id, enable_motor);
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Distance).distance_mut();
    if enable_motor != joint.enable_motor {
        joint.enable_motor = enable_motor;
        joint.motor_impulse = 0.0;
    }
}

/// (b3DistanceJoint_IsMotorEnabled)
pub fn distance_joint_is_motor_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .enable_motor
}

/// (b3DistanceJoint_SetMotorSpeed)
pub fn distance_joint_set_motor_speed(world: &mut World, joint_id: JointId, motor_speed: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_set_motor_speed(joint_id, motor_speed);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Distance)
        .distance_mut()
        .motor_speed = motor_speed;
}

/// (b3DistanceJoint_GetMotorSpeed)
pub fn distance_joint_get_motor_speed(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .motor_speed
}

/// (b3DistanceJoint_GetMotorForce)
pub fn distance_joint_get_motor_force(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Distance);
    world.inv_h * base.distance().motor_impulse
}

/// (b3DistanceJoint_SetMaxMotorForce)
pub fn distance_joint_set_max_motor_force(world: &mut World, joint_id: JointId, force: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_distance_joint_set_max_motor_force(joint_id, force);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Distance)
        .distance_mut()
        .max_motor_force = force;
}

/// (b3DistanceJoint_GetMaxMotorForce)
pub fn distance_joint_get_max_motor_force(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Distance)
        .distance()
        .max_motor_force
}

/// (b3GetDistanceJointForce)
pub fn get_distance_joint_force(world: &World, base: &JointSim) -> Vec3 {
    let joint = base.distance();

    let transform_a = get_body_transform(world, base.body_id_a);
    let transform_b = get_body_transform(world, base.body_id_b);

    let p_a = transform_world_point(transform_a, base.local_frame_a.p);
    let p_b = transform_world_point(transform_b, base.local_frame_b.p);
    let d = sub_pos(p_b, p_a);
    let axis = normalize(d);
    let force = (joint.impulse + joint.lower_impulse - joint.upper_impulse + joint.motor_impulse)
        * world.inv_h;
    mul_sv(force, axis)
}

/// (b3PrepareDistanceJoint)
pub fn prepare_distance_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Distance);

    let id_a = base.body_id_a;
    let id_b = base.body_id_b;

    let body_a = &world.bodies[id_a as usize];
    let body_b = &world.bodies[id_b as usize];

    debug_assert!(body_a.set_index == AWAKE_SET || body_b.set_index == AWAKE_SET);

    let body_sim_a =
        &world.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
    let body_sim_b =
        &world.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];

    let m_a = body_sim_a.inv_mass;
    let i_a = body_sim_a.inv_inertia_world;
    let m_b = body_sim_b.inv_mass;
    let i_b = body_sim_b.inv_inertia_world;

    base.inv_mass_a = m_a;
    base.inv_mass_b = m_b;
    base.inv_i_a = i_a;
    base.inv_i_b = i_b;

    let local_frame_a_p = base.local_frame_a.p;
    let local_frame_b_p = base.local_frame_b.p;

    let index_a = if body_a.set_index == AWAKE_SET {
        body_a.local_index
    } else {
        NULL_INDEX
    };
    let index_b = if body_b.set_index == AWAKE_SET {
        body_b.local_index
    } else {
        NULL_INDEX
    };

    let anchor_a = rotate_vector(
        body_sim_a.transform.q,
        sub(local_frame_a_p, body_sim_a.local_center),
    );
    let anchor_b = rotate_vector(
        body_sim_b.transform.q,
        sub(local_frame_b_p, body_sim_b.local_center),
    );
    let delta_center = sub_pos(body_sim_b.center, body_sim_a.center);

    let joint = base.distance_mut();
    joint.index_a = index_a;
    joint.index_b = index_b;
    joint.anchor_a = anchor_a;
    joint.anchor_b = anchor_b;
    joint.delta_center = delta_center;

    let r_a = joint.anchor_a;
    let r_b = joint.anchor_b;
    let separation = add(sub(r_b, r_a), joint.delta_center);
    let axis = normalize(separation);

    let cr_a = cross(r_a, axis);
    let cr_b = cross(r_b, axis);
    let k = m_a + m_b + dot(cr_a, mul_mv(i_a, cr_a)) + dot(cr_b, mul_mv(i_b, cr_b));
    joint.axial_mass = if k > 0.0 { 1.0 / k } else { 0.0 };

    joint.distance_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

    if !context.enable_warm_starting {
        joint.impulse = 0.0;
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
        joint.motor_impulse = 0.0;
    }
}

/// (b3WarmStartDistanceJoint)
pub fn warm_start_distance_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Distance);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.distance_mut();

    let mut state_a = if joint.index_a == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_a as usize]
    };
    let mut state_b = if joint.index_b == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_b as usize]
    };

    let r_a = rotate_vector(state_a.delta_rotation, joint.anchor_a);
    let r_b = rotate_vector(state_b.delta_rotation, joint.anchor_b);

    let ds = add(
        sub(state_b.delta_position, state_a.delta_position),
        sub(r_b, r_a),
    );
    let separation = add(joint.delta_center, ds);
    let axis = normalize(separation);

    let axial_impulse =
        joint.impulse + joint.lower_impulse - joint.upper_impulse + joint.motor_impulse;
    let p = mul_sv(axial_impulse, axis);

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = mul_sub(state_a.linear_velocity, m_a, p);
        state_a.angular_velocity = sub(state_a.angular_velocity, mul_mv(i_a, cross(r_a, p)));
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = mul_add(state_b.linear_velocity, m_b, p);
        state_b.angular_velocity = add(state_b.angular_velocity, mul_mv(i_b, cross(r_b, p)));
        states[joint.index_b as usize] = state_b;
    }
}

/// (b3SolveDistanceJoint)
pub fn solve_distance_joint(
    base: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    debug_assert!(base.type_ == JointType::Distance);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;
    let constraint_softness = base.constraint_softness;

    let joint = base.distance_mut();

    let mut state_a = if joint.index_a == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_a as usize]
    };
    let mut state_b = if joint.index_b == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_b as usize]
    };

    let mut v_a = state_a.linear_velocity;
    let mut w_a = state_a.angular_velocity;
    let mut v_b = state_b.linear_velocity;
    let mut w_b = state_b.angular_velocity;

    let r_a = rotate_vector(state_a.delta_rotation, joint.anchor_a);
    let r_b = rotate_vector(state_b.delta_rotation, joint.anchor_b);

    let ds = add(
        sub(state_b.delta_position, state_a.delta_position),
        sub(r_b, r_a),
    );
    let separation = add(joint.delta_center, ds);

    let length_ = length(separation);
    let axis = normalize(separation);

    // soft if spring enabled and (limit disabled or limits unequal)
    if joint.enable_spring && (joint.min_length < joint.max_length || !joint.enable_limit) {
        if joint.hertz > 0.0 {
            let vr = add(sub(v_b, v_a), sub(cross(w_b, r_b), cross(w_a, r_a)));
            let c_dot = dot(axis, vr);
            let c = length_ - joint.length;
            let bias = joint.distance_softness.bias_rate * c;

            let m = joint.distance_softness.mass_scale * joint.axial_mass;
            let old_impulse = joint.impulse;
            let mut impulse =
                -m * (c_dot + bias) - joint.distance_softness.impulse_scale * old_impulse;
            let h = context.h;
            joint.impulse = clamp_float(
                joint.impulse + impulse,
                joint.lower_spring_force * h,
                joint.upper_spring_force * h,
            );
            impulse = joint.impulse - old_impulse;

            let p = mul_sv(impulse, axis);
            v_a = mul_sub(v_a, m_a, p);
            w_a = sub(w_a, mul_mv(i_a, cross(r_a, p)));
            v_b = mul_add(v_b, m_b, p);
            w_b = add(w_b, mul_mv(i_b, cross(r_b, p)));
        }

        if joint.enable_limit {
            // lower limit
            {
                let vr = add(sub(v_b, v_a), sub(cross(w_b, r_b), cross(w_a, r_a)));
                let c_dot = dot(axis, vr);
                let c = length_ - joint.min_length;

                let mut bias = 0.0;
                let mut mass_coeff = 1.0;
                let mut impulse_coeff = 0.0;
                if c > 0.0 {
                    bias = c * context.inv_h;
                } else if use_bias {
                    bias = constraint_softness.bias_rate * c;
                    mass_coeff = constraint_softness.mass_scale;
                    impulse_coeff = constraint_softness.impulse_scale;
                }

                let mut impulse = -mass_coeff * joint.axial_mass * (c_dot + bias)
                    - impulse_coeff * joint.lower_impulse;
                let new_impulse = max_float(0.0, joint.lower_impulse + impulse);
                impulse = new_impulse - joint.lower_impulse;
                joint.lower_impulse = new_impulse;

                let p = mul_sv(impulse, axis);
                v_a = mul_sub(v_a, m_a, p);
                w_a = sub(w_a, mul_mv(i_a, cross(r_a, p)));
                v_b = mul_add(v_b, m_b, p);
                w_b = add(w_b, mul_mv(i_b, cross(r_b, p)));
            }

            // upper
            {
                let vr = add(sub(v_a, v_b), sub(cross(w_a, r_a), cross(w_b, r_b)));
                let c_dot = dot(axis, vr);
                let c = joint.max_length - length_;

                let mut bias = 0.0;
                let mut mass_scale = 1.0;
                let mut impulse_scale = 0.0;
                if c > 0.0 {
                    bias = c * context.inv_h;
                } else if use_bias {
                    bias = constraint_softness.bias_rate * c;
                    mass_scale = constraint_softness.mass_scale;
                    impulse_scale = constraint_softness.impulse_scale;
                }

                let mut impulse = -mass_scale * joint.axial_mass * (c_dot + bias)
                    - impulse_scale * joint.upper_impulse;
                let new_impulse = max_float(0.0, joint.upper_impulse + impulse);
                impulse = new_impulse - joint.upper_impulse;
                joint.upper_impulse = new_impulse;

                let p = mul_sv(-impulse, axis);
                v_a = mul_sub(v_a, m_a, p);
                w_a = sub(w_a, mul_mv(i_a, cross(r_a, p)));
                v_b = mul_add(v_b, m_b, p);
                w_b = add(w_b, mul_mv(i_b, cross(r_b, p)));
            }
        }

        if joint.enable_motor {
            let vr = add(sub(v_b, v_a), sub(cross(w_b, r_b), cross(w_a, r_a)));
            let c_dot = dot(axis, vr);
            let mut impulse = joint.axial_mass * (joint.motor_speed - c_dot);
            let old_impulse = joint.motor_impulse;
            let max_impulse = context.h * joint.max_motor_force;
            joint.motor_impulse =
                clamp_float(joint.motor_impulse + impulse, -max_impulse, max_impulse);
            impulse = joint.motor_impulse - old_impulse;

            let p = mul_sv(impulse, axis);
            v_a = mul_sub(v_a, m_a, p);
            w_a = sub(w_a, mul_mv(i_a, cross(r_a, p)));
            v_b = mul_add(v_b, m_b, p);
            w_b = add(w_b, mul_mv(i_b, cross(r_b, p)));
        }
    } else {
        // rigid constraint
        let vr = add(sub(v_b, v_a), sub(cross(w_b, r_b), cross(w_a, r_a)));
        let c_dot = dot(axis, vr);
        let c = length_ - joint.length;

        let mut bias = 0.0;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias {
            bias = constraint_softness.bias_rate * c;
            mass_scale = constraint_softness.mass_scale;
            impulse_scale = constraint_softness.impulse_scale;
        }

        let impulse =
            -mass_scale * joint.axial_mass * (c_dot + bias) - impulse_scale * joint.impulse;
        joint.impulse += impulse;

        let p = mul_sv(impulse, axis);
        v_a = mul_sub(v_a, m_a, p);
        w_a = sub(w_a, mul_mv(i_a, cross(r_a, p)));
        v_b = mul_add(v_b, m_b, p);
        w_b = add(w_b, mul_mv(i_b, cross(r_b, p)));
    }

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = v_a;
        state_a.angular_velocity = w_a;
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = v_b;
        state_b.angular_velocity = w_b;
        states[joint.index_b as usize] = state_b;
    }
}
