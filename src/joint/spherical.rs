// Port of spherical_joint.c: public accessors, force/torque reporting, and the
// prepare/warm-start/solve simulation functions.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{body_flags, get_body_transform, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::math_functions::{
    add, add_mm, clamp_float, cross, delta_quat_to_rotation, det, dot, dot_quat, get_swing_angle,
    get_twist_angle, inv_mul_quat, invert_matrix, is_valid_float, is_valid_quat, is_valid_vec3,
    length, max_float, min_float, mul_add, mul_mm, mul_mv, mul_quat, mul_sub, mul_sv, neg,
    negate_mat3, negate_quat, normalize, rotate_vector, skew, solve3, sub, sub_pos, Quat, Vec3,
    MAT3_ZERO, PI, VEC3_AXIS_Z, VEC3_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b3SphericalJoint_EnableConeLimit)
pub fn spherical_joint_enable_cone_limit(world: &mut World, joint_id: JointId, enable_limit: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Spherical).spherical_mut();
    if enable_limit != joint.enable_cone_limit {
        joint.swing_impulse = 0.0;
    }
    joint.enable_cone_limit = enable_limit;
}

/// (b3SphericalJoint_IsConeLimitEnabled)
pub fn spherical_joint_is_cone_limit_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .enable_cone_limit
}

/// (b3SphericalJoint_GetConeLimit)
pub fn spherical_joint_get_cone_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .cone_angle
}

/// (b3SphericalJoint_SetConeLimit)
pub fn spherical_joint_set_cone_limit(world: &mut World, joint_id: JointId, angle_radians: f32) {
    debug_assert!(is_valid_float(angle_radians) && (0.0..=0.5 * PI).contains(&angle_radians));
    get_joint_sim_check_type(world, joint_id, JointType::Spherical)
        .spherical_mut()
        .cone_angle = angle_radians;
}

/// (b3SphericalJoint_GetConeAngle)
pub fn spherical_joint_get_cone_angle(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical);
    let transform_a = get_body_transform(world, base.body_id_a);
    let transform_b = get_body_transform(world, base.body_id_b);

    let quat_a = mul_quat(transform_a.q, base.local_frame_a.q);
    let mut quat_b = mul_quat(transform_b.q, base.local_frame_b.q);

    if dot_quat(quat_a, quat_b) < 0.0 {
        // this keeps the swing angle in the range [0, pi]
        quat_b = negate_quat(quat_b);
    }

    let rel_q = inv_mul_quat(quat_a, quat_b);
    get_swing_angle(rel_q)
}

/// (b3SphericalJoint_EnableTwistLimit)
pub fn spherical_joint_enable_twist_limit(
    world: &mut World,
    joint_id: JointId,
    enable_limit: bool,
) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Spherical).spherical_mut();
    if enable_limit != joint.enable_twist_limit {
        joint.lower_twist_impulse = 0.0;
        joint.upper_twist_impulse = 0.0;
    }
    joint.enable_twist_limit = enable_limit;
}

/// (b3SphericalJoint_IsTwistLimitEnabled)
pub fn spherical_joint_is_twist_limit_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .enable_twist_limit
}

/// (b3SphericalJoint_GetLowerTwistLimit)
pub fn spherical_joint_get_lower_twist_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .lower_twist_angle
}

/// (b3SphericalJoint_GetUpperTwistLimit)
pub fn spherical_joint_get_upper_twist_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .upper_twist_angle
}

/// (b3SphericalJoint_SetTwistLimits)
pub fn spherical_joint_set_twist_limits(
    world: &mut World,
    joint_id: JointId,
    lower_limit_radians: f32,
    upper_limit_radians: f32,
) {
    debug_assert!(is_valid_float(lower_limit_radians) && is_valid_float(upper_limit_radians));

    let lower_angle = min_float(lower_limit_radians, upper_limit_radians);
    let upper_angle = max_float(lower_limit_radians, upper_limit_radians);

    let joint = get_joint_sim_check_type(world, joint_id, JointType::Spherical).spherical_mut();
    joint.lower_twist_angle = clamp_float(lower_angle, -0.99 * PI, 0.99 * PI);
    joint.upper_twist_angle = clamp_float(upper_angle, -0.99 * PI, 0.99 * PI);
}

/// (b3SphericalJoint_GetTwistAngle)
pub fn spherical_joint_get_twist_angle(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical);
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

/// (b3SphericalJoint_EnableSpring)
pub fn spherical_joint_enable_spring(world: &mut World, joint_id: JointId, enable_spring: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Spherical).spherical_mut();
    if enable_spring != joint.enable_spring {
        joint.spring_impulse = VEC3_ZERO;
    }
    joint.enable_spring = enable_spring;
}

/// (b3SphericalJoint_IsSpringEnabled)
pub fn spherical_joint_is_spring_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .enable_spring
}

/// (b3SphericalJoint_SetTargetRotation)
pub fn spherical_joint_set_target_rotation(
    world: &mut World,
    joint_id: JointId,
    target_rotation: Quat,
) {
    debug_assert!(is_valid_quat(target_rotation));
    get_joint_sim_check_type(world, joint_id, JointType::Spherical)
        .spherical_mut()
        .target_rotation = target_rotation;
}

/// (b3SphericalJoint_GetTargetRotation)
pub fn spherical_joint_get_target_rotation(world: &World, joint_id: JointId) -> Quat {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .target_rotation
}

/// (b3SphericalJoint_SetSpringHertz)
pub fn spherical_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    debug_assert!(is_valid_float(hertz) && hertz >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Spherical)
        .spherical_mut()
        .hertz = hertz;
}

/// (b3SphericalJoint_GetSpringHertz)
pub fn spherical_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .hertz
}

/// (b3SphericalJoint_SetSpringDampingRatio)
pub fn spherical_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    debug_assert!(is_valid_float(damping_ratio) && damping_ratio >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Spherical)
        .spherical_mut()
        .damping_ratio = damping_ratio;
}

/// (b3SphericalJoint_GetSpringDampingRatio)
pub fn spherical_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .damping_ratio
}

/// (b3SphericalJoint_EnableMotor)
pub fn spherical_joint_enable_motor(world: &mut World, joint_id: JointId, enable_motor: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Spherical).spherical_mut();
    if enable_motor != joint.enable_motor {
        joint.motor_impulse = VEC3_ZERO;
    }
    joint.enable_motor = enable_motor;
}

/// (b3SphericalJoint_IsMotorEnabled)
pub fn spherical_joint_is_motor_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .enable_motor
}

/// (b3SphericalJoint_SetMotorVelocity)
pub fn spherical_joint_set_motor_velocity(
    world: &mut World,
    joint_id: JointId,
    motor_velocity: Vec3,
) {
    debug_assert!(is_valid_vec3(motor_velocity));
    get_joint_sim_check_type(world, joint_id, JointType::Spherical)
        .spherical_mut()
        .motor_velocity = motor_velocity;
}

/// (b3SphericalJoint_GetMotorVelocity)
pub fn spherical_joint_get_motor_velocity(world: &World, joint_id: JointId) -> Vec3 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .motor_velocity
}

/// (b3SphericalJoint_SetMaxMotorTorque)
pub fn spherical_joint_set_max_motor_torque(world: &mut World, joint_id: JointId, max_force: f32) {
    debug_assert!(is_valid_float(max_force) && max_force >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Spherical)
        .spherical_mut()
        .max_motor_torque = max_force;
}

/// (b3SphericalJoint_GetMaxMotorTorque)
pub fn spherical_joint_get_max_motor_torque(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical)
        .spherical()
        .max_motor_torque
}

/// (b3SphericalJoint_GetMotorTorque)
pub fn spherical_joint_get_motor_torque(world: &World, joint_id: JointId) -> Vec3 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Spherical);
    mul_sv(world.inv_h, base.spherical().motor_impulse)
}

/// (b3GetSphericalJointForce)
pub fn get_spherical_joint_force(world: &World, base: &JointSim) -> Vec3 {
    mul_sv(world.inv_h, base.spherical().linear_impulse)
}

/// (b3GetSphericalJointTorque)
pub fn get_spherical_joint_torque(world: &World, base: &JointSim) -> Vec3 {
    let xf_a = get_body_transform(world, base.body_id_a);
    let xf_b = get_body_transform(world, base.body_id_b);
    let q_a = mul_quat(xf_a.q, base.local_frame_a.q);
    let q_b = mul_quat(xf_b.q, base.local_frame_b.q);

    // Cone axis is the z-axis of body A.
    let cone_axis = rotate_vector(q_a, VEC3_AXIS_Z);
    let twist_axis = rotate_vector(q_b, VEC3_AXIS_Z);
    let swing_axis = normalize(cross(cone_axis, twist_axis));

    let joint = base.spherical();
    let mut impulse = add(joint.spring_impulse, joint.motor_impulse);
    impulse = mul_add(
        impulse,
        joint.lower_twist_impulse - joint.upper_twist_impulse,
        twist_axis,
    );
    impulse = mul_add(impulse, joint.swing_impulse, swing_axis);
    mul_sv(world.inv_h, impulse)
}

/// (b3PrepareSphericalJoint)
pub fn prepare_spherical_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Spherical);

    let body_a = &world.bodies[base.body_id_a as usize];
    let body_b = &world.bodies[base.body_id_b as usize];

    debug_assert!(body_a.set_index == AWAKE_SET || body_b.set_index == AWAKE_SET);

    let body_sim_a =
        &world.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
    let body_sim_b =
        &world.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];

    base.inv_mass_a = body_sim_a.inv_mass;
    base.inv_mass_b = body_sim_b.inv_mass;
    base.inv_i_a = body_sim_a.inv_inertia_world;
    base.inv_i_b = body_sim_b.inv_inertia_world;

    let inv_inertia_sum = add_mm(base.inv_i_a, base.inv_i_b);
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

    // Cone axis is the z-axis of body A.
    let cone_axis = rotate_vector(frame_a_q, VEC3_AXIS_Z);
    // Twist axis is the z-axis of body B.
    let twist_axis = rotate_vector(frame_b_q, VEC3_AXIS_Z);

    let enable_cone_limit;
    let enable_twist_limit;
    let swing_axis;
    let swing_mass;
    let twist_jacobian;
    let twist_mass;
    {
        let joint = base.spherical();
        enable_cone_limit = joint.enable_cone_limit;
        enable_twist_limit = joint.enable_twist_limit;

        if enable_cone_limit {
            // Swing axis may be zero
            let axis = normalize(cross(cone_axis, twist_axis));
            let k = dot(axis, mul_mv(inv_inertia_sum, axis));
            swing_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
            swing_axis = axis;
        } else {
            swing_mass = 0.0;
            swing_axis = VEC3_ZERO;
        }

        if enable_twist_limit {
            let rel_q = inv_mul_quat(frame_a_q, frame_b_q);
            let tan_theta_over_2 = ((rel_q.v.x * rel_q.v.x + rel_q.v.y * rel_q.v.y)
                / (rel_q.v.z * rel_q.v.z + rel_q.s * rel_q.s))
                .sqrt();

            // todo verify this Jacobian using a finite difference, unit test?
            let swing = normalize(cross(cone_axis, twist_axis));
            let perp_axis = cross(swing, cone_axis);
            let jac = mul_add(cone_axis, tan_theta_over_2, perp_axis);
            let k = dot(jac, mul_mv(inv_inertia_sum, jac));
            twist_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
            twist_jacobian = jac;
        } else {
            twist_mass = 0.0;
            twist_jacobian = VEC3_ZERO;
        }
    }

    let rotation_mass = if !base.fixed_rotation {
        invert_matrix(inv_inertia_sum)
    } else {
        MAT3_ZERO
    };

    let joint = base.spherical_mut();
    joint.index_a = index_a;
    joint.index_b = index_b;
    joint.frame_a.q = frame_a_q;
    joint.frame_a.p = frame_a_p;
    joint.frame_b.q = frame_b_q;
    joint.frame_b.p = frame_b_p;
    joint.delta_center = delta_center;

    if enable_cone_limit {
        joint.swing_mass = swing_mass;
        joint.swing_axis = swing_axis;
    }
    if enable_twist_limit {
        joint.twist_mass = twist_mass;
        joint.twist_jacobian = twist_jacobian;
    }

    joint.rotation_mass = rotation_mass;
    joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

    if !context.enable_warm_starting {
        joint.linear_impulse = VEC3_ZERO;
        joint.motor_impulse = VEC3_ZERO;
        joint.spring_impulse = VEC3_ZERO;
        joint.swing_impulse = 0.0;
        joint.lower_twist_impulse = 0.0;
        joint.upper_twist_impulse = 0.0;
    }
}

/// (b3WarmStartSphericalJoint)
pub fn warm_start_spherical_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Spherical);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.spherical_mut();

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

    let mut angular_impulse = add(joint.spring_impulse, joint.motor_impulse);
    angular_impulse = mul_sub(angular_impulse, joint.swing_impulse, joint.swing_axis);
    angular_impulse = mul_add(
        angular_impulse,
        joint.lower_twist_impulse - joint.upper_twist_impulse,
        joint.twist_jacobian,
    );

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

/// (b3SolveSphericalJoint)
pub fn solve_spherical_joint(
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

    let joint = base.spherical_mut();
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
    let quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

    let rel_q = inv_mul_quat(quat_a, quat_b);

    // Solve spring
    if joint.enable_spring && !fixed_rotation {
        // Rotation constraint error
        let delta_rotation = delta_quat_to_rotation(rel_q, joint.target_rotation);
        let c = neg(rotate_vector(quat_a, delta_rotation));

        let bias = mul_sv(joint.spring_softness.bias_rate, c);
        let mass_scale = joint.spring_softness.mass_scale;
        let impulse_scale = joint.spring_softness.impulse_scale;
        let cdot = sub(w_b, w_a);

        let impulse = mul_sub(
            mul_sv(-mass_scale, mul_mv(joint.rotation_mass, add(cdot, bias))),
            impulse_scale,
            joint.spring_impulse,
        );
        joint.spring_impulse = add(joint.spring_impulse, impulse);

        w_a = sub(w_a, mul_mv(i_a, impulse));
        w_b = add(w_b, mul_mv(i_b, impulse));
    }

    if joint.enable_motor && !fixed_rotation {
        let cdot = sub(w_b, w_a);

        let mut lambda = neg(mul_mv(joint.rotation_mass, sub(cdot, joint.motor_velocity)));
        let mut new_impulse = add(joint.motor_impulse, lambda);
        let len = length(new_impulse);
        let max_impulse = joint.max_motor_torque * context.h;
        if len > max_impulse {
            new_impulse = mul_sv(max_impulse / len, new_impulse);
        }

        lambda = sub(new_impulse, joint.motor_impulse);
        joint.motor_impulse = new_impulse;

        w_a = sub(w_a, mul_mv(i_a, lambda));
        w_b = add(w_b, mul_mv(i_b, lambda));
    }

    if joint.enable_twist_limit && !fixed_rotation {
        let twist_angle = get_twist_angle(rel_q);

        // todo does an updated twist axis help?
        let twist_jacobian = joint.twist_jacobian;

        // Lower limit
        {
            let c = twist_angle - joint.lower_twist_angle;
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

            let cdot = dot(sub(w_b, w_a), twist_jacobian);
            let old_impulse = joint.lower_twist_impulse;
            let mut delta_impulse =
                -mass_scale * joint.twist_mass * (cdot + bias) - impulse_scale * old_impulse;
            joint.lower_twist_impulse = max_float(old_impulse + delta_impulse, 0.0);
            delta_impulse = joint.lower_twist_impulse - old_impulse;

            w_a = mul_sub(w_a, delta_impulse, mul_mv(i_a, twist_jacobian));
            w_b = mul_add(w_b, delta_impulse, mul_mv(i_b, twist_jacobian));
        }

        // Upper limit
        {
            let c = joint.upper_twist_angle - twist_angle;
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
            let cdot = dot(sub(w_a, w_b), twist_jacobian);
            let old_impulse = joint.upper_twist_impulse;
            let mut delta_impulse =
                -mass_scale * joint.twist_mass * (cdot + bias) - impulse_scale * old_impulse;
            joint.upper_twist_impulse = max_float(old_impulse + delta_impulse, 0.0);
            delta_impulse = joint.upper_twist_impulse - old_impulse;

            // sign flipped on applied impulse
            w_a = mul_add(w_a, delta_impulse, mul_mv(i_a, twist_jacobian));
            w_b = mul_sub(w_b, delta_impulse, mul_mv(i_b, twist_jacobian));
        }
    }

    if joint.enable_cone_limit && !fixed_rotation {
        let swing_angle = get_swing_angle(rel_q);

        // todo does an updated swing axis help?
        let swing_axis = joint.swing_axis;

        let c = joint.cone_angle - swing_angle;
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
        let cdot = dot(sub(w_a, w_b), swing_axis);
        let old_impulse = joint.swing_impulse;
        let mut delta_impulse =
            -mass_scale * joint.swing_mass * (cdot + bias) - impulse_scale * old_impulse;
        joint.swing_impulse = max_float(old_impulse + delta_impulse, 0.0);
        delta_impulse = joint.swing_impulse - old_impulse;

        // sign flipped on applied impulse
        w_a = mul_add(w_a, delta_impulse, mul_mv(i_a, swing_axis));
        w_b = mul_sub(w_b, delta_impulse, mul_mv(i_b, swing_axis));
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

            let mut separation = add(sub(dc_b, dc_a), sub(r_b, r_a));
            separation = add(separation, joint.delta_center);

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

        let impulse = mul_sub(mul_sv(-mass_scale, b), impulse_scale, joint.linear_impulse);
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
