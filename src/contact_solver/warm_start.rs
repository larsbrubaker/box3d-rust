//! Warm-start overflow contact constraints. (b3WarmStartContacts_Mesh)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::ContactConstraint;
use crate::body::{body_flags, BodyState};
use crate::core::NULL_INDEX;
use crate::math_functions::{add, cross, mul_add, mul_mv, mul_sub, mul_sv, sub, VEC3_ZERO};

fn warm_start_one(contact_constraint: &ContactConstraint, states: &mut [BodyState]) {
    // The impulses of all manifolds are summed and applied to the bodies once.
    let mut linear_impulse = VEC3_ZERO;
    let mut angular_impulse_a = VEC3_ZERO;
    let mut angular_impulse_b = VEC3_ZERO;

    let manifold_count = contact_constraint.manifold_count;
    for manifold_index in 0..manifold_count as usize {
        let constraint = &contact_constraint.constraints[manifold_index];

        // Normal impulses
        let normal = constraint.normal;
        let mut total_normal_impulse = 0.0f32;
        let mut moment_a = VEC3_ZERO;
        let mut moment_b = VEC3_ZERO;

        let point_count = constraint.point_count as usize;
        for j in 0..point_count {
            let cp = &constraint.points[j];

            // fixed anchors
            total_normal_impulse += cp.normal_impulse;
            moment_a = mul_add(moment_a, cp.normal_impulse, cp.r_a);
            moment_b = mul_add(moment_b, cp.normal_impulse, cp.r_b);
        }

        // Central friction
        let mut friction_impulse = mul_sv(constraint.friction_impulse.x, constraint.tangent1);
        friction_impulse = mul_add(
            friction_impulse,
            constraint.friction_impulse.y,
            constraint.tangent2,
        );

        linear_impulse = add(
            linear_impulse,
            mul_add(friction_impulse, total_normal_impulse, normal),
        );

        // Central twist friction
        let twist_impulse = mul_sv(constraint.twist_impulse, normal);

        let angular_a = add(
            add(
                cross(moment_a, normal),
                cross(constraint.center_a, friction_impulse),
            ),
            twist_impulse,
        );
        let angular_b = add(
            add(
                cross(moment_b, normal),
                cross(constraint.center_b, friction_impulse),
            ),
            twist_impulse,
        );

        // Rolling resistance
        angular_impulse_a = add(
            angular_impulse_a,
            add(angular_a, constraint.rolling_impulse),
        );
        angular_impulse_b = add(
            angular_impulse_b,
            add(angular_b, constraint.rolling_impulse),
        );
    }

    // C writes through a dummy state for a null body; its flags are zero, so it is never
    // written back.
    let index_a = contact_constraint.index_a;
    let index_b = contact_constraint.index_b;

    if index_a != NULL_INDEX {
        let state_a = &mut states[index_a as usize];
        if (state_a.flags & body_flags::DYNAMIC_FLAG) != 0 {
            state_a.linear_velocity = mul_sub(
                state_a.linear_velocity,
                contact_constraint.inv_mass_a,
                linear_impulse,
            );
            state_a.angular_velocity = sub(
                state_a.angular_velocity,
                mul_mv(contact_constraint.inv_i_a, angular_impulse_a),
            );
        }
    }

    if index_b != NULL_INDEX {
        let state_b = &mut states[index_b as usize];
        if (state_b.flags & body_flags::DYNAMIC_FLAG) != 0 {
            state_b.linear_velocity = mul_add(
                state_b.linear_velocity,
                contact_constraint.inv_mass_b,
                linear_impulse,
            );
            state_b.angular_velocity = add(
                state_b.angular_velocity,
                mul_mv(contact_constraint.inv_i_b, angular_impulse_b),
            );
        }
    }
}

/// (b3WarmStartContacts_Mesh)
pub fn warm_start_contacts(constraints: &mut [ContactConstraint], states: &mut [BodyState]) {
    for contact_constraint in constraints.iter() {
        warm_start_one(contact_constraint, states);
    }
}
