// Port of prismatic_joint.c: public accessors, force/torque reporting, and
// the prepare/warm-start/solve simulation functions.
//
// The C accessors resolve the world from the id via the global registry; the
// Rust port takes `world` explicitly. Prepare takes &World (caller owns the
// JointSim); warm start and solve take the awake body states slice and copy
// states out/back, writing only under the dynamic-flag guard like C.
//
// Draw functions (b3DrawPrismaticJoint) are skipped per project convention.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{
    body_flags, get_body_sim, get_body_state_index, get_body_transform, BodyState,
    IDENTITY_BODY_STATE,
};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::math_functions::{
    add, add2, add_mm, blend2, blend3, clamp_float, cross, delta_quat_to_rotation, det, dot,
    inv_mul_quat, invert_matrix, is_valid_float, is_valid_vec3, make_matrix_from_quat, max_float,
    min_float, mul_add, mul_mv, mul_quat, mul_sub, mul_sv, mul_sv2, neg, rotate_vector, solve2,
    sub, sub2, sub_pos, Mat2, Transform, Vec2, Vec3, QUAT_IDENTITY, VEC3_AXIS_X, VEC3_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b3PrismaticJoint_EnableLimit)
pub fn prismatic_joint_enable_limit(world: &mut World, joint_id: JointId, enable_limit: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic).prismatic_mut();
    if enable_limit != joint.enable_limit {
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
    }
    joint.enable_limit = enable_limit;
}

/// (b3PrismaticJoint_IsLimitEnabled)
pub fn prismatic_joint_is_limit_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .enable_limit
}

/// (b3PrismaticJoint_GetLowerLimit)
pub fn prismatic_joint_get_lower_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .lower_translation
}

/// (b3PrismaticJoint_GetUpperLimit)
pub fn prismatic_joint_get_upper_limit(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .upper_translation
}

/// (b3PrismaticJoint_SetLimits)
pub fn prismatic_joint_set_limits(world: &mut World, joint_id: JointId, lower: f32, upper: f32) {
    debug_assert!(is_valid_float(lower) && is_valid_float(upper));

    let lower_angle = min_float(lower, upper);
    let upper_angle = max_float(lower, upper);

    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic).prismatic_mut();
    joint.lower_translation = lower_angle;
    joint.upper_translation = upper_angle;
}

/// (b3PrismaticJoint_GetTranslation)
pub fn prismatic_joint_get_translation(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    let transform_a = get_body_transform(world, base.body_id_a);
    let transform_b = get_body_transform(world, base.body_id_b);

    let mut joint_axis = rotate_vector(base.local_frame_a.q, VEC3_AXIS_X);
    joint_axis = rotate_vector(transform_a.q, joint_axis);

    let anchor_a = rotate_vector(transform_a.q, base.local_frame_a.p);
    let anchor_b = rotate_vector(transform_b.q, base.local_frame_b.p);
    let d = add(
        sub_pos(transform_b.p, transform_a.p),
        sub(anchor_b, anchor_a),
    );
    dot(d, joint_axis)
}

/// (b3PrismaticJoint_EnableSpring)
pub fn prismatic_joint_enable_spring(world: &mut World, joint_id: JointId, enable_spring: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic).prismatic_mut();
    if enable_spring != joint.enable_spring {
        joint.spring_impulse = 0.0;
    }
    joint.enable_spring = enable_spring;
}

/// (b3PrismaticJoint_IsSpringEnabled)
pub fn prismatic_joint_is_spring_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .enable_spring
}

/// (b3PrismaticJoint_SetTargetTranslation)
pub fn prismatic_joint_set_target_translation(
    world: &mut World,
    joint_id: JointId,
    target_translation: f32,
) {
    debug_assert!(is_valid_float(target_translation));
    get_joint_sim_check_type(world, joint_id, JointType::Prismatic)
        .prismatic_mut()
        .target_translation = target_translation;
}

/// (b3PrismaticJoint_GetTargetTranslation)
pub fn prismatic_joint_get_target_translation(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .target_translation
}

/// (b3PrismaticJoint_SetSpringHertz)
pub fn prismatic_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    debug_assert!(is_valid_float(hertz) && hertz >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Prismatic)
        .prismatic_mut()
        .hertz = hertz;
}

/// (b3PrismaticJoint_GetSpringHertz)
pub fn prismatic_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .hertz
}

/// (b3PrismaticJoint_SetSpringDampingRatio)
pub fn prismatic_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    debug_assert!(is_valid_float(damping_ratio) && damping_ratio >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Prismatic)
        .prismatic_mut()
        .damping_ratio = damping_ratio;
}

/// (b3PrismaticJoint_GetSpringDampingRatio)
pub fn prismatic_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .damping_ratio
}

/// (b3PrismaticJoint_EnableMotor)
pub fn prismatic_joint_enable_motor(world: &mut World, joint_id: JointId, enable_motor: bool) {
    let joint = get_joint_sim_check_type(world, joint_id, JointType::Prismatic).prismatic_mut();
    if enable_motor != joint.enable_motor {
        joint.motor_impulse = 0.0;
    }
    joint.enable_motor = enable_motor;
}

/// (b3PrismaticJoint_IsMotorEnabled)
pub fn prismatic_joint_is_motor_enabled(world: &World, joint_id: JointId) -> bool {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .enable_motor
}

/// (b3PrismaticJoint_SetMotorSpeed)
pub fn prismatic_joint_set_motor_speed(world: &mut World, joint_id: JointId, motor_speed: f32) {
    debug_assert!(is_valid_float(motor_speed));
    get_joint_sim_check_type(world, joint_id, JointType::Prismatic)
        .prismatic_mut()
        .motor_speed = motor_speed;
}

/// (b3PrismaticJoint_GetMotorSpeed)
pub fn prismatic_joint_get_motor_speed(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .motor_speed
}

/// (b3PrismaticJoint_SetMaxMotorForce)
pub fn prismatic_joint_set_max_motor_force(world: &mut World, joint_id: JointId, max_force: f32) {
    debug_assert!(is_valid_float(max_force) && max_force >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Prismatic)
        .prismatic_mut()
        .max_motor_force = max_force;
}

/// (b3PrismaticJoint_GetMaxMotorForce)
pub fn prismatic_joint_get_max_motor_force(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic)
        .prismatic()
        .max_motor_force
}

/// (b3PrismaticJoint_GetMotorForce)
pub fn prismatic_joint_get_motor_force(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    world.inv_h * base.prismatic().motor_impulse
}

/// (b3PrismaticJoint_GetSpeed)
pub fn prismatic_joint_get_speed(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Prismatic);
    let body_id_a = base.body_id_a;
    let body_id_b = base.body_id_b;

    let body_sim_a = get_body_sim(world, body_id_a);
    let body_sim_b = get_body_sim(world, body_id_b);

    let q_a = body_sim_a.transform.q;
    let q_b = body_sim_b.transform.q;

    let axis_a = rotate_vector(q_a, rotate_vector(base.local_frame_a.q, VEC3_AXIS_X));
    let r_a = rotate_vector(q_a, sub(base.local_frame_a.p, body_sim_a.local_center));
    let r_b = rotate_vector(q_b, sub(base.local_frame_b.p, body_sim_b.local_center));

    // Difference the centers in double so the speed stays exact far from the origin.
    let d = add(sub_pos(body_sim_b.center, body_sim_a.center), sub(r_b, r_a));

    let state_a = get_body_state_index(world, body_id_a)
        .map(|i| world.solver_sets[AWAKE_SET as usize].body_states[i as usize]);
    let state_b = get_body_state_index(world, body_id_b)
        .map(|i| world.solver_sets[AWAKE_SET as usize].body_states[i as usize]);

    let v_a = state_a.map_or(VEC3_ZERO, |s| s.linear_velocity);
    let v_b = state_b.map_or(VEC3_ZERO, |s| s.linear_velocity);
    let w_a = state_a.map_or(VEC3_ZERO, |s| s.angular_velocity);
    let w_b = state_b.map_or(VEC3_ZERO, |s| s.angular_velocity);

    let v_rel = sub(add(v_b, cross(w_b, r_b)), add(v_a, cross(w_a, r_a)));

    // The axis moves with body A, so account for its rotation.
    dot(d, cross(w_a, axis_a)) + dot(axis_a, v_rel)
}

/// (b3GetPrismaticJointForce)
pub fn get_prismatic_joint_force(world: &World, base: &JointSim) -> Vec3 {
    let transform_a = get_body_transform(world, base.body_id_a);
    let joint = base.prismatic();

    // impulse in joint space
    let impulse = Vec3 {
        x: joint.perp_impulse.x,
        y: joint.perp_impulse.y,
        z: joint.motor_impulse + joint.lower_impulse + joint.upper_impulse + joint.spring_impulse,
    };

    // convert impulse to force
    let mut force = mul_sv(world.inv_h, impulse);

    // convert to body space
    force = rotate_vector(base.local_frame_a.q, force);

    // convert to world space
    force = rotate_vector(transform_a.q, force);
    force
}

/// (b3GetPrismaticJointTorque)
pub fn get_prismatic_joint_torque(world: &World, base: &JointSim) -> Vec3 {
    let transform_a = get_body_transform(world, base.body_id_a);
    let joint = base.prismatic();

    let mut torque = mul_sv(world.inv_h, joint.angular_impulse);
    torque = rotate_vector(base.local_frame_a.q, torque);
    torque = rotate_vector(transform_a.q, torque);
    torque
}

/// (b3PreparePrismaticJoint)
pub fn prepare_prismatic_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Prismatic);

    let id_a = base.body_id_a;
    let id_b = base.body_id_b;

    let body_a = &world.bodies[id_a as usize];
    let body_b = &world.bodies[id_b as usize];

    debug_assert!(body_b.set_index == AWAKE_SET);

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
    let rotation_mass = invert_matrix(inv_inertia_sum);

    let joint = base.prismatic_mut();
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
    joint.rotation_mass = rotation_mass;

    // Initial joint axes in world space
    let matrix_a = make_matrix_from_quat(joint.frame_a.q);
    joint.joint_axis = matrix_a.cx;
    joint.perp_axis_y = matrix_a.cy;
    joint.perp_axis_z = matrix_a.cz;

    joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

    if !context.enable_warm_starting {
        joint.perp_impulse = Vec2 { x: 0.0, y: 0.0 };
        joint.angular_impulse = VEC3_ZERO;
        joint.motor_impulse = 0.0;
        joint.spring_impulse = 0.0;
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
    }
}

/// (b3WarmStartPrismaticJoint)
pub fn warm_start_prismatic_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Prismatic);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.prismatic_mut();

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

    // todo make this code and the wheel joint more similar

    let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
    let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);
    let d = add(
        add(
            sub(state_b.delta_position, state_a.delta_position),
            joint.delta_center,
        ),
        sub(r_b, r_a),
    );
    let joint_axis = rotate_vector(state_a.delta_rotation, joint.joint_axis);
    let s_ax = cross(add(r_a, d), joint_axis);
    let s_bx = cross(r_b, joint_axis);

    let perp_y = rotate_vector(state_a.delta_rotation, joint.perp_axis_y);
    let perp_z = rotate_vector(state_a.delta_rotation, joint.perp_axis_z);
    let s_ay = cross(add(r_a, d), perp_y);
    let s_by = cross(r_b, perp_y);
    let s_az = cross(add(r_a, d), perp_z);
    let s_bz = cross(r_b, perp_z);

    let axial_impulse =
        joint.spring_impulse + joint.motor_impulse + joint.lower_impulse - joint.upper_impulse;
    let perp_impulse = joint.perp_impulse;

    let p = blend3(
        axial_impulse,
        joint_axis,
        perp_impulse.x,
        perp_y,
        perp_impulse.y,
        perp_z,
    );
    let l_a = add(
        blend3(
            axial_impulse,
            s_ax,
            perp_impulse.x,
            s_ay,
            perp_impulse.y,
            s_az,
        ),
        joint.angular_impulse,
    );
    let l_b = add(
        blend3(
            axial_impulse,
            s_bx,
            perp_impulse.x,
            s_by,
            perp_impulse.y,
            s_bz,
        ),
        joint.angular_impulse,
    );

    let mut v_a = state_a.linear_velocity;
    let mut w_a = state_a.angular_velocity;
    let mut v_b = state_b.linear_velocity;
    let mut w_b = state_b.angular_velocity;
    v_a = mul_sub(v_a, m_a, p);
    w_a = sub(w_a, mul_mv(i_a, l_a));
    v_b = mul_add(v_b, m_b, p);
    w_b = add(w_b, mul_mv(i_b, l_b));

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

/// (b3SolvePrismaticJoint ╬ô├ç├╢ the C source has no type assert on this function,
/// unlike prepare/warm-start)
pub fn solve_prismatic_joint(
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

    let joint = base.prismatic_mut();
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

    let dc_a = state_a.delta_position;
    let dc_b = state_b.delta_position;
    let d = add(add(sub(dc_b, dc_a), joint.delta_center), sub(r_b, r_a));

    let joint_axis = rotate_vector(state_a.delta_rotation, joint.joint_axis);
    let s_ax = cross(add(r_a, d), joint_axis);
    let s_bx = cross(r_b, joint_axis);
    let joint_translation = dot(d, joint_axis);
    let target_translation = joint.target_translation;

    // The axial effective mass must be fresh to avoid divergence when the joint is stressed
    let ka = m_a + m_b + dot(s_ax, mul_mv(i_a, s_ax)) + dot(s_bx, mul_mv(i_b, s_bx));
    let axial_mass = if ka > 0.0 { 1.0 / ka } else { 0.0 };

    // Solve spring
    if joint.enable_spring && !fixed_rotation {
        // Get the substep relative rotation
        let c = joint_translation - target_translation;

        let bias = joint.spring_softness.bias_rate * c;
        let mass_scale = joint.spring_softness.mass_scale;
        let impulse_scale = joint.spring_softness.impulse_scale;

        let v_rel = sub(sub(add(v_b, cross(w_b, r_b)), v_a), cross(w_a, add(r_a, d)));
        let cdot = dot(v_rel, joint_axis);
        let delta_impulse =
            -mass_scale * axial_mass * (cdot + bias) - impulse_scale * joint.spring_impulse;
        joint.spring_impulse += delta_impulse;

        let p = mul_sv(delta_impulse, joint_axis);
        let l_a = mul_sv(delta_impulse, s_ax);
        let l_b = mul_sv(delta_impulse, s_bx);

        v_a = mul_sub(v_a, m_a, p);
        w_a = sub(w_a, mul_mv(i_a, l_a));
        v_b = mul_add(v_b, m_b, p);
        w_b = add(w_b, mul_mv(i_b, l_b));
    }

    if joint.enable_motor && !fixed_rotation {
        let v_rel = sub(sub(add(v_b, cross(w_b, r_b)), v_a), cross(w_a, add(r_a, d)));
        let cdot = dot(v_rel, joint_axis) - joint.motor_speed;

        let mut delta_impulse = -axial_mass * cdot;
        let mut new_impulse = joint.motor_impulse + delta_impulse;
        let max_impulse = joint.max_motor_force * context.h;
        new_impulse = clamp_float(new_impulse, -max_impulse, max_impulse);
        delta_impulse = new_impulse - joint.motor_impulse;
        joint.motor_impulse = new_impulse;

        let p = mul_sv(delta_impulse, joint_axis);
        let l_a = mul_sv(delta_impulse, s_ax);
        let l_b = mul_sv(delta_impulse, s_bx);

        v_a = mul_sub(v_a, m_a, p);
        w_a = sub(w_a, mul_mv(i_a, l_a));
        v_b = mul_add(v_b, m_b, p);
        w_b = add(w_b, mul_mv(i_b, l_b));
    }

    if joint.enable_limit && !fixed_rotation {
        let speculative_distance = 0.25 * (joint.upper_translation - joint.lower_translation);

        // Lower limit
        {
            let c = joint_translation - joint.lower_translation;

            if c < speculative_distance {
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

                let v_rel = sub(sub(add(v_b, cross(w_b, r_b)), v_a), cross(w_a, add(r_a, d)));
                let cdot = dot(v_rel, joint_axis);
                let old_impulse = joint.lower_impulse;
                let mut delta_impulse =
                    -mass_scale * axial_mass * (cdot + bias) - impulse_scale * old_impulse;
                joint.lower_impulse = max_float(old_impulse + delta_impulse, 0.0);
                delta_impulse = joint.lower_impulse - old_impulse;

                let p = mul_sv(delta_impulse, joint_axis);
                let l_a = mul_sv(delta_impulse, s_ax);
                let l_b = mul_sv(delta_impulse, s_bx);

                v_a = mul_sub(v_a, m_a, p);
                w_a = sub(w_a, mul_mv(i_a, l_a));
                v_b = mul_add(v_b, m_b, p);
                w_b = add(w_b, mul_mv(i_b, l_b));
            } else {
                joint.lower_impulse = 0.0;
            }
        }

        // Upper limit
        {
            let c = joint.upper_translation - joint_translation;

            if c < speculative_distance {
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
                let v_rel = sub(sub(add(v_b, cross(w_b, r_b)), v_a), cross(w_a, add(r_a, d)));
                let cdot = -dot(v_rel, joint_axis);
                let old_impulse = joint.upper_impulse;
                let delta_impulse =
                    -mass_scale * axial_mass * (cdot + bias) - impulse_scale * old_impulse;
                joint.upper_impulse = max_float(old_impulse + delta_impulse, 0.0);

                // sign flipped on applied impulse
                let neg_delta_impulse = old_impulse - joint.upper_impulse;
                let p = mul_sv(neg_delta_impulse, joint_axis);
                let l_a = mul_sv(neg_delta_impulse, s_ax);
                let l_b = mul_sv(neg_delta_impulse, s_bx);

                v_a = mul_sub(v_a, m_a, p);
                w_a = sub(w_a, mul_mv(i_a, l_a));
                v_b = mul_add(v_b, m_b, p);
                w_b = add(w_b, mul_mv(i_b, l_b));
            } else {
                joint.upper_impulse = 0.0;
            }
        }
    }

    // Rotation constraint
    if !fixed_rotation {
        let mut bias = VEC3_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;

        if use_bias {
            let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
            let quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

            let rel_q = inv_mul_quat(quat_a, quat_b);
            let target_quat = QUAT_IDENTITY;
            let delta_rotation = delta_quat_to_rotation(rel_q, target_quat);
            let c = neg(rotate_vector(quat_a, delta_rotation));

            bias = mul_sv(constraint_softness.bias_rate, c);
            mass_scale = constraint_softness.mass_scale;
            impulse_scale = constraint_softness.impulse_scale;
        }

        let cdot = sub(w_b, w_a);
        let impulse = sub(
            mul_sv(-mass_scale, mul_mv(joint.rotation_mass, add(cdot, bias))),
            mul_sv(impulse_scale, joint.angular_impulse),
        );
        joint.angular_impulse = add(joint.angular_impulse, impulse);

        w_a = sub(w_a, mul_mv(i_a, impulse));
        w_b = add(w_b, mul_mv(i_b, impulse));
    }

    // Solve point-to-line constraint
    {
        let perp_y = rotate_vector(state_a.delta_rotation, joint.perp_axis_y);
        let perp_z = rotate_vector(state_a.delta_rotation, joint.perp_axis_z);

        let mut bias = Vec2 { x: 0.0, y: 0.0 };
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias {
            let c = Vec2 {
                x: dot(perp_y, d),
                y: dot(perp_z, d),
            };
            bias = mul_sv2(constraint_softness.bias_rate, c);
            mass_scale = constraint_softness.mass_scale;
            impulse_scale = constraint_softness.impulse_scale;
        }

        let v_rel = sub(sub(add(v_b, cross(w_b, r_b)), v_a), cross(w_a, add(r_a, d)));
        let cdot = Vec2 {
            x: dot(perp_y, v_rel),
            y: dot(perp_z, v_rel),
        };

        // K = [(1/mA + 1/mB) * eye(2) - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]
        // Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
        let s_ay = cross(add(r_a, d), perp_y);
        let s_by = cross(r_b, perp_y);
        let s_az = cross(add(r_a, d), perp_z);
        let s_bz = cross(r_b, perp_z);

        let kyy = m_a + m_b + dot(s_ay, mul_mv(i_a, s_ay)) + dot(s_by, mul_mv(i_b, s_by));
        let kyz = dot(s_ay, mul_mv(i_a, s_az)) + dot(s_by, mul_mv(i_b, s_bz));
        let kzz = m_a + m_b + dot(s_az, mul_mv(i_a, s_az)) + dot(s_bz, mul_mv(i_b, s_bz));

        let k = Mat2 {
            cx: Vec2 { x: kyy, y: kyz },
            cy: Vec2 { x: kyz, y: kzz },
        };

        let old_impulse = joint.perp_impulse;
        let sol = solve2(k, add2(cdot, bias));
        let delta_impulse = sub2(
            mul_sv2(-mass_scale, sol),
            mul_sv2(impulse_scale, old_impulse),
        );
        joint.perp_impulse = add2(old_impulse, delta_impulse);

        let p = blend2(delta_impulse.x, perp_y, delta_impulse.y, perp_z);

        v_a = mul_sub(v_a, m_a, p);
        w_a = sub(
            w_a,
            mul_mv(i_a, blend2(delta_impulse.x, s_ay, delta_impulse.y, s_az)),
        );
        v_b = mul_add(v_b, m_b, p);
        w_b = add(
            w_b,
            mul_mv(i_b, blend2(delta_impulse.x, s_by, delta_impulse.y, s_bz)),
        );
    }

    debug_assert!(is_valid_vec3(v_a));
    debug_assert!(is_valid_vec3(w_a));
    debug_assert!(is_valid_vec3(v_b));
    debug_assert!(is_valid_vec3(w_b));

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
