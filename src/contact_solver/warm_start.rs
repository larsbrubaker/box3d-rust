//! Warm-start Mesh / Convex contact constraints.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::ContactConstraint;
use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::math_functions::{add, cross, mul_add, mul_mv, mul_mv_sym, mul_sub, mul_sv, sub};

fn warm_start_one(contact_constraint: &ContactConstraint, states: &mut [BodyState], use_sym: bool) {
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

    let mul = if use_sym { mul_mv_sym } else { mul_mv };

    let manifold_count = contact_constraint.manifold_count;
    for manifold_index in 0..manifold_count as usize {
        let constraint = &contact_constraint.constraints[manifold_index];

        let normal = constraint.normal;
        let point_count = constraint.point_count;
        for j in 0..point_count as usize {
            let cp = &constraint.points[j];

            let r_a = cp.r_a;
            let r_b = cp.r_b;

            let impulse = mul_sv(cp.normal_impulse, normal);
            w_a = sub(w_a, mul(i_a, cross(r_a, impulse)));
            v_a = mul_sub(v_a, m_a, impulse);
            w_b = add(w_b, mul(i_b, cross(r_b, impulse)));
            v_b = mul_add(v_b, m_b, impulse);
        }

        // Central friction
        {
            let r_a = constraint.origin_a;
            let r_b = constraint.origin_b;
            let mut impulse = mul_sv(constraint.friction_impulse.x, constraint.tangent1);
            impulse = add(
                impulse,
                mul_sv(constraint.friction_impulse.y, constraint.tangent2),
            );

            w_a = sub(w_a, mul(i_a, cross(r_a, impulse)));
            v_a = mul_sub(v_a, m_a, impulse);
            w_b = add(w_b, mul(i_b, cross(r_b, impulse)));
            v_b = mul_add(v_b, m_b, impulse);
        }

        // Central twist friction
        {
            let impulse = mul_sv(constraint.twist_impulse, constraint.normal);
            w_a = sub(w_a, mul(i_a, impulse));
            w_b = add(w_b, mul(i_b, impulse));
        }

        // Rolling resistance
        {
            let impulse = constraint.rolling_impulse;
            w_a = sub(w_a, mul(i_a, impulse));
            w_b = add(w_b, mul(i_b, impulse));
        }
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

/// (b3WarmStartContacts_Mesh)
pub fn warm_start_contacts(constraints: &mut [ContactConstraint], states: &mut [BodyState]) {
    for contact_constraint in constraints.iter() {
        warm_start_one(contact_constraint, states, false);
    }
}

/// (b3WarmStartContacts_Convex) — SymMatrix inertia multiply.
pub fn warm_start_contacts_convex(constraints: &mut [ContactConstraint], states: &mut [BodyState]) {
    for contact_constraint in constraints.iter() {
        warm_start_one(contact_constraint, states, true);
    }
}
