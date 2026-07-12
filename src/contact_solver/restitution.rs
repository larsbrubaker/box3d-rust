//! Apply restitution impulses (Mesh and Convex kernels).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::ContactConstraint;
use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::math_functions::{
    add, cross, dot, max_float, mul_add, mul_mv, mul_mv_sym, mul_sub, mul_sv, sub, Matrix3, Vec3,
};
use crate::solver::StepContext;

fn apply_restitution_one(
    constraints: &mut [ContactConstraint],
    states: &mut [BodyState],
    context: &StepContext,
    use_sym: bool,
) {
    let threshold = context.restitution_threshold;
    let mul: fn(Matrix3, Vec3) -> Vec3 = if use_sym { mul_mv_sym } else { mul_mv };

    for contact_constraint in constraints.iter_mut() {
        let restitution = contact_constraint.restitution;
        if restitution == 0.0 {
            continue;
        }

        let index_a = contact_constraint.index_a;
        let index_b = contact_constraint.index_b;

        let mut state_a = if index_a == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_a as usize]
        };
        let mut state_b = if index_b == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_b as usize]
        };

        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;

        let m_a = contact_constraint.inv_mass_a;
        let i_a = contact_constraint.inv_i_a;
        let m_b = contact_constraint.inv_mass_b;
        let i_b = contact_constraint.inv_i_b;

        let manifold_count = contact_constraint.manifold_count;
        for manifold_index in 0..manifold_count as usize {
            let cm = &mut contact_constraint.constraints[manifold_index];

            let normal = cm.normal;
            let point_count = cm.point_count;
            debug_assert!(
                0 < point_count && point_count as usize <= crate::constants::MAX_MANIFOLD_POINTS
            );

            for point_index in 0..point_count as usize {
                let cp = &mut cm.points[point_index];

                if cp.relative_velocity > -threshold || cp.total_normal_impulse == 0.0 {
                    continue;
                }

                let r_a = cp.r_a;
                let r_b = cp.r_b;

                let vr_b = add(v_b, cross(w_b, r_b));
                let vr_a = add(v_a, cross(w_a, r_a));
                let vn = dot(sub(vr_b, vr_a), normal);

                let mut impulse = -cp.normal_mass * (vn + restitution * cp.relative_velocity);

                let new_impulse = max_float(cp.normal_impulse + impulse, 0.0);
                impulse = new_impulse - cp.normal_impulse;
                cp.normal_impulse = new_impulse;
                cp.total_normal_impulse += impulse;

                let p = mul_sv(impulse, normal);
                v_a = mul_sub(v_a, m_a, p);
                w_a = sub(w_a, mul(i_a, cross(r_a, p)));
                v_b = mul_add(v_b, m_b, p);
                w_b = add(w_b, mul(i_b, cross(r_b, p)));
            }

            if (state_a.flags & body_flags::DYNAMIC_FLAG) != 0 {
                state_a.linear_velocity = v_a;
                state_a.angular_velocity = w_a;
                states[index_a as usize] = state_a;
            }

            if (state_b.flags & body_flags::DYNAMIC_FLAG) != 0 {
                state_b.linear_velocity = v_b;
                state_b.angular_velocity = w_b;
                states[index_b as usize] = state_b;
            }
        }
    }
}

/// (b3ApplyRestitution_Mesh)
pub fn apply_restitution(
    constraints: &mut [ContactConstraint],
    states: &mut [BodyState],
    context: &StepContext,
) {
    apply_restitution_one(constraints, states, context, false);
}

/// Scalar form of `b3ApplyRestitution_Convex` (symmetric inertia multiply).
pub fn apply_restitution_convex(
    constraints: &mut [ContactConstraint],
    states: &mut [BodyState],
    context: &StepContext,
) {
    apply_restitution_one(constraints, states, context, true);
}
