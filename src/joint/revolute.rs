// Port of revolute_joint.c: public accessors, force/torque reporting, and the
// prepare/warm-start/solve simulation functions.
//
// The C accessors resolve the world from the id via the global registry; the
// Rust port takes `world` explicitly. Prepare takes &World (caller owns the
// JointSim); warm start and solve take the awake body states slice and copy
// states out/back, writing only under the dynamic-flag guard like C.
//
// Draw functions (b3DrawRevoluteJoint) are skipped per project convention.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{body_flags, get_body_transform, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::math_functions::{
    add, add2, add_mm, clamp_float, cross, det, dot, dot_quat, get_twist_angle, inv_mul_quat,
    is_valid_float, max_float, min_float, mul_add, mul_mm, mul_mv, mul_quat, mul_sub, mul_sv,
    mul_sv2, negate_mat3, negate_quat, rotate_vector, skew, solve2, solve3, sub, sub2, sub_pos,
    Mat2, Transform, Vec2, Vec3, PI, VEC2_ZERO, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b3RevoluteJoint_EnableLimit)
pub fn revolute_joint_enable_limit(world: &mut World, joint_id: JointId, enable_limit: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_enable_limit(joint_id, enable_limit);
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute).revolute_mut();
    if enable_limit != joint.enable_limit {
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
    }
    joint.enable_limit = enable_limit;
}

/// (b3RevoluteJoint_IsLimitEnabled)
pub fn revolute_joint_is_limit_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .enable_limit
}

/// (b3RevoluteJoint_GetLowerLimit)
pub fn revolute_joint_get_lower_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .lower_angle
}

/// (b3RevoluteJoint_GetUpperLimit)
pub fn revolute_joint_get_upper_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .upper_angle
}

/// (b3RevoluteJoint_SetLimits)
pub fn revolute_joint_set_limits(
    world: &mut World,
    joint_id: JointId,
    lower_limit_radians: f32,
    upper_limit_radians: f32,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_set_limits(joint_id, lower_limit_radians, upper_limit_radians);
    });
    debug_assert!(is_valid_float(lower_limit_radians) && is_valid_float(upper_limit_radians));

    let lower_angle = min_float(lower_limit_radians, upper_limit_radians);
    let upper_angle = max_float(lower_limit_radians, upper_limit_radians);

    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute).revolute_mut();
    joint.lower_angle = clamp_float(lower_angle, -0.99 * PI, 0.99 * PI);
    joint.upper_angle = clamp_float(upper_angle, -0.99 * PI, 0.99 * PI);
}

/// (b3RevoluteJoint_GetAngle)
pub fn revolute_joint_get_angle(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    let transform_a = get_body_transform(world, base.body_id_a);
    let transform_b = get_body_transform(world, base.body_id_b);

    let quat_a = mul_quat(transform_a.q, base.local_frame_a.q);
    let mut quat_b = mul_quat(transform_b.q, base.local_frame_b.q);

    if dot_quat(quat_a, quat_b) < 0.0 {
        // this keeps the twist angle in the range [-pi, pi]
        quat_b = negate_quat(quat_b);
    }

    let rel_q = inv_mul_quat(quat_a, quat_b);

    get_twist_angle(rel_q)
}

/// (b3RevoluteJoint_EnableSpring)
pub fn revolute_joint_enable_spring(world: &mut World, joint_id: JointId, enable_spring: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_enable_spring(joint_id, enable_spring);
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute).revolute_mut();
    if enable_spring != joint.enable_spring {
        joint.spring_impulse = 0.0;
    }
    joint.enable_spring = enable_spring;
}

/// (b3RevoluteJoint_IsSpringEnabled)
pub fn revolute_joint_is_spring_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .enable_spring
}

/// (b3RevoluteJoint_SetTargetAngle)
pub fn revolute_joint_set_target_angle(world: &mut World, joint_id: JointId, target_radians: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_set_target_angle(joint_id, target_radians);
    });
    debug_assert!(is_valid_float(target_radians) && (-PI..=PI).contains(&target_radians));
    get_joint_sim_check_type(world, joint_id, JointType::Revolute)
        .revolute_mut()
        .target_angle = target_radians;
}

/// (b3RevoluteJoint_GetTargetAngle)
pub fn revolute_joint_get_target_angle(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .target_angle
}

/// (b3RevoluteJoint_SetSpringHertz)
pub fn revolute_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_set_spring_hertz(joint_id, hertz);
    });
    debug_assert!(is_valid_float(hertz) && hertz >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Revolute)
        .revolute_mut()
        .hertz = hertz;
}

/// (b3RevoluteJoint_GetSpringHertz)
pub fn revolute_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .hertz
}

/// (b3RevoluteJoint_SetSpringDampingRatio)
pub fn revolute_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_set_spring_damping_ratio(joint_id, damping_ratio);
    });
    debug_assert!(is_valid_float(damping_ratio) && damping_ratio >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Revolute)
        .revolute_mut()
        .damping_ratio = damping_ratio;
}

/// (b3RevoluteJoint_GetSpringDampingRatio)
pub fn revolute_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .damping_ratio
}

/// (b3RevoluteJoint_EnableMotor)
pub fn revolute_joint_enable_motor(world: &mut World, joint_id: JointId, enable_motor: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_enable_motor(joint_id, enable_motor);
    });
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Revolute).revolute_mut();
    if enable_motor != joint.enable_motor {
        joint.motor_impulse = 0.0;
    }
    joint.enable_motor = enable_motor;
}

/// (b3RevoluteJoint_IsMotorEnabled)
pub fn revolute_joint_is_motor_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .enable_motor
}

/// (b3RevoluteJoint_SetMotorSpeed)
pub fn revolute_joint_set_motor_speed(world: &mut World, joint_id: JointId, motor_speed: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_set_motor_speed(joint_id, motor_speed);
    });
    debug_assert!(is_valid_float(motor_speed));
    get_joint_sim_check_type(world, joint_id, JointType::Revolute)
        .revolute_mut()
        .motor_speed = motor_speed;
}

/// (b3RevoluteJoint_GetMotorSpeed)
pub fn revolute_joint_get_motor_speed(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .motor_speed
}

/// (b3RevoluteJoint_SetMaxMotorTorque)
pub fn revolute_joint_set_max_motor_torque(world: &mut World, joint_id: JointId, max_force: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_revolute_joint_set_max_motor_torque(joint_id, max_force);
    });
    debug_assert!(is_valid_float(max_force) && max_force >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Revolute)
        .revolute_mut()
        .max_motor_torque = max_force;
}

/// (b3RevoluteJoint_GetMaxMotorTorque)
pub fn revolute_joint_get_max_motor_torque(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute)
        .revolute()
        .max_motor_torque
}

/// (b3RevoluteJoint_GetMotorTorque)
pub fn revolute_joint_get_motor_torque(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Revolute);
    world.inv_h * base.revolute().motor_impulse
}

/// (b3GetRevoluteJointForce)
pub fn get_revolute_joint_force(world: &World, base: &JointSim) -> Vec3 {
    mul_sv(world.inv_h, base.revolute().linear_impulse)
}

/// (b3GetRevoluteJointTorque — recomputes perpAxisX/perpAxisY for warm
/// starting as a side effect, so `base` is mutable like the C pointer)
pub fn get_revolute_joint_torque(world: &World, base: &mut JointSim) -> Vec3 {
    let transform_a = get_body_transform(world, base.body_id_a);
    let local_frame_a_q = base.local_frame_a.q;
    let mut axis = rotate_vector(local_frame_a_q, VEC3_AXIS_Z);
    axis = rotate_vector(transform_a.q, axis);

    let joint = base.revolute_mut();
    let rel_q = inv_mul_quat(joint.frame_a.q, joint.frame_b.q);

    // These are needed for warm starting
    joint.perp_axis_x = mul_sv(
        0.5,
        rotate_vector(
            joint.frame_a.q,
            add(mul_sv(rel_q.s, VEC3_AXIS_X), cross(rel_q.v, VEC3_AXIS_X)),
        ),
    );
    joint.perp_axis_y = mul_sv(
        0.5,
        rotate_vector(
            joint.frame_a.q,
            add(mul_sv(rel_q.s, VEC3_AXIS_Y), cross(rel_q.v, VEC3_AXIS_Y)),
        ),
    );

    let axial_impulse =
        joint.spring_impulse + joint.motor_impulse + joint.lower_impulse - joint.upper_impulse;
    let mut angular_impulse = add(
        mul_sv(joint.perp_impulse.x, joint.perp_axis_x),
        mul_sv(joint.perp_impulse.y, joint.perp_axis_y),
    );
    angular_impulse = mul_add(angular_impulse, axial_impulse, joint.rotation_axis_z);

    // todo add pivot torque
    let impulse = mul_add(
        angular_impulse,
        joint.spring_impulse + joint.motor_impulse + joint.lower_impulse - joint.upper_impulse,
        axis,
    );
    mul_sv(world.inv_h, impulse)
}

/// (b3PrepareRevoluteJoint)
pub fn prepare_revolute_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Revolute);

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

    let inv_inertia_sum = add_mm(i_a, i_b);
    base.fixed_rotation = det(inv_inertia_sum) < 1000.0 * f32::MIN_POSITIVE;

    let local_frame_a = base.local_frame_a;
    let local_frame_b = base.local_frame_b;

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

    // Compute joint anchor frames with world space rotation, relative to center of mass
    // Avoid round-off here as much as possible.
    let frame_a_q = mul_quat(body_sim_a.transform.q, local_frame_a.q);
    let frame_a_p = rotate_vector(
        body_sim_a.transform.q,
        sub(local_frame_a.p, body_sim_a.local_center),
    );
    let frame_b_q = mul_quat(body_sim_b.transform.q, local_frame_b.q);
    let frame_b_p = rotate_vector(
        body_sim_b.transform.q,
        sub(local_frame_b.p, body_sim_b.local_center),
    );

    let delta_center = sub_pos(body_sim_b.center, body_sim_a.center);

    let joint = base.revolute_mut();
    joint.index_a = index_a;
    joint.index_b = index_b;
    joint.frame_a = Transform {
        p: frame_a_p,
        q: frame_a_q,
    };
    joint.frame_b = Transform {
        p: frame_b_p,
        q: frame_b_q,
    };
    joint.delta_center = delta_center;

    {
        // Rotation axis is the z-axis of body A.
        let rotation_axis_z = rotate_vector(joint.frame_a.q, VEC3_AXIS_Z);
        let k = dot(rotation_axis_z, mul_mv(inv_inertia_sum, rotation_axis_z));
        joint.axial_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        joint.rotation_axis_z = rotation_axis_z;
    }

    let rel_q = inv_mul_quat(joint.frame_a.q, joint.frame_b.q);

    {
        // These are needed for warm starting
        joint.perp_axis_x = mul_sv(
            0.5,
            rotate_vector(
                joint.frame_a.q,
                add(mul_sv(rel_q.s, VEC3_AXIS_X), cross(rel_q.v, VEC3_AXIS_X)),
            ),
        );
        joint.perp_axis_y = mul_sv(
            0.5,
            rotate_vector(
                joint.frame_a.q,
                add(mul_sv(rel_q.s, VEC3_AXIS_Y), cross(rel_q.v, VEC3_AXIS_Y)),
            ),
        );
    }

    joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

    if !context.enable_warm_starting {
        joint.linear_impulse = VEC3_ZERO;
        joint.perp_impulse = VEC2_ZERO;
        joint.motor_impulse = 0.0;
        joint.spring_impulse = 0.0;
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
    }
}

/// (b3WarmStartRevoluteJoint)
pub fn warm_start_revolute_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Revolute);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.revolute_mut();

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

    let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
    let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

    let axial_impulse =
        joint.spring_impulse + joint.motor_impulse + joint.lower_impulse - joint.upper_impulse;
    let mut angular_impulse = add(
        mul_sv(joint.perp_impulse.x, joint.perp_axis_x),
        mul_sv(joint.perp_impulse.y, joint.perp_axis_y),
    );
    angular_impulse = mul_add(angular_impulse, axial_impulse, joint.rotation_axis_z);

    v_a = mul_sub(v_a, m_a, joint.linear_impulse);
    w_a = sub(
        w_a,
        mul_mv(i_a, add(cross(r_a, joint.linear_impulse), angular_impulse)),
    );

    v_b = mul_add(v_b, m_b, joint.linear_impulse);
    w_b = add(
        w_b,
        mul_mv(i_b, add(cross(r_b, joint.linear_impulse), angular_impulse)),
    );

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

/// (b3SolveRevoluteJoint ╬ô├ç├╢ the C source has no type assert on this function,
/// unlike prepare/warm-start)
pub fn solve_revolute_joint(
    base: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;
    let fixed_rotation = base.fixed_rotation;
    let constraint_softness = base.constraint_softness;

    let joint = base.revolute_mut();
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

    let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
    let mut quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

    if dot_quat(quat_a, quat_b) < 0.0 {
        // this keeps the rotation angle in the range [-pi, pi]
        quat_b = negate_quat(quat_b);
    }

    let rel_q = inv_mul_quat(quat_a, quat_b);

    // Solve spring
    if joint.enable_spring && !fixed_rotation {
        // Get the substep relative rotation
        let target_angle = joint.target_angle;
        let angle = get_twist_angle(rel_q);
        let c = angle - target_angle;

        let bias = joint.spring_softness.bias_rate * c;
        let mass_scale = joint.spring_softness.mass_scale;
        let impulse_scale = joint.spring_softness.impulse_scale;
        let cdot = dot(sub(w_b, w_a), joint.rotation_axis_z);

        let delta_impulse =
            -mass_scale * joint.axial_mass * (cdot + bias) - impulse_scale * joint.spring_impulse;
        joint.spring_impulse += delta_impulse;

        w_a = mul_sub(w_a, delta_impulse, mul_mv(i_a, joint.rotation_axis_z));
        w_b = mul_add(w_b, delta_impulse, mul_mv(i_b, joint.rotation_axis_z));
    }

    if joint.enable_motor && !fixed_rotation {
        let cdot = dot(sub(w_b, w_a), joint.rotation_axis_z) - joint.motor_speed;

        let mut delta_impulse = -joint.axial_mass * cdot;
        let mut new_impulse = joint.motor_impulse + delta_impulse;
        let max_impulse = joint.max_motor_torque * context.h;
        new_impulse = clamp_float(new_impulse, -max_impulse, max_impulse);
        delta_impulse = new_impulse - joint.motor_impulse;
        joint.motor_impulse = new_impulse;

        w_a = mul_sub(w_a, delta_impulse, mul_mv(i_a, joint.rotation_axis_z));
        w_b = mul_add(w_b, delta_impulse, mul_mv(i_b, joint.rotation_axis_z));
    }

    if joint.enable_limit && !fixed_rotation {
        let angle = get_twist_angle(rel_q);

        // todo does an updated twist axis help?
        let axis = joint.rotation_axis_z;

        // Lower limit
        {
            let c = angle - joint.lower_angle;
            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;
            if c > 0.0 {
                // speculation
                bias = c * context.inv_h;
            } else if use_bias {
                bias = constraint_softness.bias_rate * c;
                mass_scale = constraint_softness.mass_scale;
                impulse_scale = constraint_softness.impulse_scale;
            }

            let cdot = dot(sub(w_b, w_a), axis);
            let old_impulse = joint.lower_impulse;
            let mut delta_impulse =
                -mass_scale * joint.axial_mass * (cdot + bias) - impulse_scale * old_impulse;
            joint.lower_impulse = max_float(old_impulse + delta_impulse, 0.0);
            delta_impulse = joint.lower_impulse - old_impulse;

            w_a = mul_sub(w_a, delta_impulse, mul_mv(i_a, axis));
            w_b = mul_add(w_b, delta_impulse, mul_mv(i_b, axis));
        }

        // Upper limit
        {
            let c = joint.upper_angle - angle;
            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;
            if c > 0.0 {
                // speculation
                bias = c * context.inv_h;
            } else if use_bias {
                bias = constraint_softness.bias_rate * c;
                mass_scale = constraint_softness.mass_scale;
                impulse_scale = constraint_softness.impulse_scale;
            }

            // sign flipped on Cdot
            let cdot = dot(sub(w_a, w_b), axis);
            let old_impulse = joint.upper_impulse;
            let mut delta_impulse =
                -mass_scale * joint.axial_mass * (cdot + bias) - impulse_scale * old_impulse;
            joint.upper_impulse = max_float(old_impulse + delta_impulse, 0.0);
            delta_impulse = joint.upper_impulse - old_impulse;

            // sign flipped on applied impulse
            w_a = mul_add(w_a, delta_impulse, mul_mv(i_a, axis));
            w_b = mul_sub(w_b, delta_impulse, mul_mv(i_b, axis));
        }
    }

    // Collinearity constraint
    if !fixed_rotation {
        let mut bias = VEC2_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;

        if use_bias {
            let c = Vec2 {
                x: rel_q.v.x,
                y: rel_q.v.y,
            };
            bias = mul_sv2(constraint_softness.bias_rate, c);
            mass_scale = constraint_softness.mass_scale;
            impulse_scale = constraint_softness.impulse_scale;
        }

        // Collinearity constraint as 2-by-2
        let perp_axis_x = mul_sv(
            0.5,
            rotate_vector(
                quat_a,
                add(mul_sv(rel_q.s, VEC3_AXIS_X), cross(rel_q.v, VEC3_AXIS_X)),
            ),
        );
        let perp_axis_y = mul_sv(
            0.5,
            rotate_vector(
                quat_a,
                add(mul_sv(rel_q.s, VEC3_AXIS_Y), cross(rel_q.v, VEC3_AXIS_Y)),
            ),
        );
        joint.perp_axis_x = perp_axis_x;
        joint.perp_axis_y = perp_axis_y;

        let inv_inertia_sum = add_mm(i_a, i_b);
        let kxx = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_x));
        let kyy = dot(perp_axis_y, mul_mv(inv_inertia_sum, perp_axis_y));
        let kxy = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_y));

        let k = Mat2 {
            cx: Vec2 { x: kxx, y: kxy },
            cy: Vec2 { x: kxy, y: kyy },
        };

        let w_rel = sub(w_b, w_a);
        let cdot = Vec2 {
            x: dot(w_rel, perp_axis_x),
            y: dot(w_rel, perp_axis_y),
        };
        let old_impulse = joint.perp_impulse;
        let sol = solve2(k, add2(cdot, bias));
        let delta_impulse = sub2(
            mul_sv2(-mass_scale, sol),
            mul_sv2(impulse_scale, old_impulse),
        );
        joint.perp_impulse = add2(joint.perp_impulse, delta_impulse);

        let angular_impulse = add(
            mul_sv(delta_impulse.x, perp_axis_x),
            mul_sv(delta_impulse.y, perp_axis_y),
        );
        w_a = sub(w_a, mul_mv(i_a, angular_impulse));
        w_b = add(w_b, mul_mv(i_b, angular_impulse));
    }

    // Solve point-to-point constraint
    {
        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        let cdot = sub(sub(add(v_b, cross(w_b, r_b)), v_a), cross(w_a, r_a));

        let mut bias = VEC3_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias {
            let dc_a = state_a.delta_position;
            let dc_b = state_b.delta_position;

            let separation = add(add(sub(dc_b, dc_a), sub(r_b, r_a)), joint.delta_center);

            bias = mul_sv(constraint_softness.bias_rate, separation);
            mass_scale = constraint_softness.mass_scale;
            impulse_scale = constraint_softness.impulse_scale;
        }

        // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        let s_a = skew(r_a);
        let s_b = skew(r_b);
        let k_a = mul_mm(s_a, mul_mm(i_a, s_a));
        let k_b = mul_mm(s_b, mul_mm(i_b, s_b));
        let mut k = negate_mat3(add_mm(k_a, k_b));
        k.cx.x += m_a + m_b;
        k.cy.y += m_a + m_b;
        k.cz.z += m_a + m_b;

        let b = solve3(k, add(cdot, bias));

        let impulse = sub(
            mul_sv(-mass_scale, b),
            mul_sv(impulse_scale, joint.linear_impulse),
        );
        joint.linear_impulse = add(joint.linear_impulse, impulse);

        v_a = mul_sub(v_a, m_a, impulse);
        w_a = sub(w_a, mul_mv(i_a, cross(r_a, impulse)));
        v_b = mul_add(v_b, m_b, impulse);
        w_b = add(w_b, mul_mv(i_b, cross(r_b, impulse)));
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
