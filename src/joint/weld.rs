// Port of weld_joint.c: public accessors, force/torque reporting, and the
// prepare/warm-start/solve simulation functions.
//
// The weld joint is a rigid connection (optionally softened by linear/angular
// springs) that locks the relative position and orientation of two frames.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::math_functions::{
    add, add_mm, cross, delta_quat_to_rotation, det, dot_quat, inv_mul_quat, invert_matrix,
    mul_add, mul_mm, mul_mv, mul_quat, mul_sub, mul_sv, neg, negate_mat3, negate_quat,
    rotate_vector, skew, solve3, sub, sub_pos, Vec3, QUAT_IDENTITY, VEC3_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b3WeldJoint_SetLinearHertz)
pub fn weld_joint_set_linear_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_weld_joint_set_linear_hertz(joint_id, hertz);
    });
    debug_assert!(hertz >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Weld)
        .weld_mut()
        .linear_hertz = hertz;
}

/// (b3WeldJoint_GetLinearHertz)
pub fn weld_joint_get_linear_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Weld)
        .weld()
        .linear_hertz
}

/// (b3WeldJoint_SetLinearDampingRatio)
pub fn weld_joint_set_linear_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_weld_joint_set_linear_damping_ratio(joint_id, damping_ratio);
    });
    debug_assert!(damping_ratio >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Weld)
        .weld_mut()
        .linear_damping_ratio = damping_ratio;
}

/// (b3WeldJoint_GetLinearDampingRatio)
pub fn weld_joint_get_linear_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Weld)
        .weld()
        .linear_damping_ratio
}

/// (b3WeldJoint_SetAngularHertz)
pub fn weld_joint_set_angular_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_weld_joint_set_angular_hertz(joint_id, hertz);
    });
    debug_assert!(hertz >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Weld)
        .weld_mut()
        .angular_hertz = hertz;
}

/// (b3WeldJoint_GetAngularHertz)
pub fn weld_joint_get_angular_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Weld)
        .weld()
        .angular_hertz
}

/// (b3WeldJoint_SetAngularDampingRatio)
pub fn weld_joint_set_angular_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_weld_joint_set_angular_damping_ratio(joint_id, damping_ratio);
    });
    debug_assert!(damping_ratio >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Weld)
        .weld_mut()
        .angular_damping_ratio = damping_ratio;
}

/// (b3WeldJoint_GetAngularDampingRatio)
pub fn weld_joint_get_angular_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Weld)
        .weld()
        .angular_damping_ratio
}

/// (b3GetWeldJointForce)
pub fn get_weld_joint_force(world: &World, base: &JointSim) -> Vec3 {
    mul_sv(world.inv_h, base.weld().linear_impulse)
}

/// (b3GetWeldJointTorque)
pub fn get_weld_joint_torque(world: &World, base: &JointSim) -> Vec3 {
    mul_sv(world.inv_h, base.weld().angular_impulse)
}

/// (b3PrepareWeldJoint)
pub fn prepare_weld_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Weld);

    let id_a = base.body_id_a;
    let id_b = base.body_id_b;

    let body_a = &world.bodies[id_a as usize];
    let body_b = &world.bodies[id_b as usize];

    debug_assert!(body_b.set_index == AWAKE_SET);

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
    let constraint_softness = base.constraint_softness;

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
    let angular_mass = invert_matrix(inv_inertia_sum);

    let joint = base.weld_mut();
    joint.index_a = index_a;
    joint.index_b = index_b;
    joint.frame_a.q = frame_a_q;
    joint.frame_a.p = frame_a_p;
    joint.frame_b.q = frame_b_q;
    joint.frame_b.p = frame_b_p;
    joint.delta_center = delta_center;
    joint.angular_mass = angular_mass;

    if joint.linear_hertz == 0.0 {
        joint.linear_spring = constraint_softness;
    } else {
        joint.linear_spring = make_soft(joint.linear_hertz, joint.linear_damping_ratio, context.h);
    }

    if joint.angular_hertz == 0.0 {
        joint.angular_spring = constraint_softness;
    } else {
        joint.angular_spring =
            make_soft(joint.angular_hertz, joint.angular_damping_ratio, context.h);
    }

    if !context.enable_warm_starting {
        joint.linear_impulse = VEC3_ZERO;
        joint.angular_impulse = VEC3_ZERO;
    }
}

/// (b3WarmStartWeldJoint)
pub fn warm_start_weld_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Weld);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.weld_mut();

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

    v_a = mul_sub(v_a, m_a, joint.linear_impulse);
    w_a = sub(
        w_a,
        mul_mv(
            i_a,
            add(cross(r_a, joint.linear_impulse), joint.angular_impulse),
        ),
    );

    v_b = mul_add(v_b, m_b, joint.linear_impulse);
    w_b = add(
        w_b,
        mul_mv(
            i_b,
            add(cross(r_b, joint.linear_impulse), joint.angular_impulse),
        ),
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

/// (b3SolveWeldJoint)
///
/// `context` is unused -- the C function also takes it but never reads it,
/// keeping the signature consistent with the other joint solve functions.
pub fn solve_weld_joint(
    base: &mut JointSim,
    _context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;
    let fixed_rotation = base.fixed_rotation;

    let joint = base.weld_mut();
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

    // angular constraint
    if !fixed_rotation {
        let mut bias = VEC3_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias || joint.angular_hertz > 0.0 {
            let target_quat = QUAT_IDENTITY;
            let delta_rotation = delta_quat_to_rotation(rel_q, target_quat);
            let c = neg(rotate_vector(quat_a, delta_rotation));

            bias = mul_sv(joint.angular_spring.bias_rate, c);
            mass_scale = joint.angular_spring.mass_scale;
            impulse_scale = joint.angular_spring.impulse_scale;
        }

        let cdot = sub(w_b, w_a);
        let impulse = mul_sub(
            mul_sv(-mass_scale, mul_mv(joint.angular_mass, add(cdot, bias))),
            impulse_scale,
            joint.angular_impulse,
        );
        joint.angular_impulse = add(joint.angular_impulse, impulse);

        w_a = sub(w_a, mul_mv(i_a, impulse));
        w_b = add(w_b, mul_mv(i_b, impulse));
    }

    // linear constraint
    {
        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        let cdot = sub(add(v_b, cross(w_b, r_b)), add(v_a, cross(w_a, r_a)));

        let mut bias = VEC3_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias || joint.linear_hertz > 0.0 {
            let dc_a = state_a.delta_position;
            let dc_b = state_b.delta_position;

            let separation = add(add(sub(dc_b, dc_a), sub(r_b, r_a)), joint.delta_center);

            bias = mul_sv(joint.linear_spring.bias_rate, separation);
            mass_scale = joint.linear_spring.mass_scale;
            impulse_scale = joint.linear_spring.impulse_scale;
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
