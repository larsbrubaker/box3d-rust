//! Solve Mesh contact constraints. (b3SolveContacts_Mesh)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::ContactConstraint;
use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::math_functions::{
    add, blend2, clamp_float, cross, dot, dot2, max_float, mul_add, mul_mv, mul_mv2, mul_sub,
    mul_sv, neg, rotate_vector, sub, sub2, Vec2,
};
use crate::solver::StepContext;

/// Merged normal and friction loops. (b3SolveContacts_Mesh)
pub fn solve_contacts(
    constraints: &mut [ContactConstraint],
    states: &mut [BodyState],
    context: &StepContext,
    use_bias: bool,
) {
    let inv_h = context.inv_h;
    let contact_speed = context.contact_speed;

    for contact_constraint in constraints.iter_mut() {
        let manifold_count = contact_constraint.manifold_count;

        let index_a = contact_constraint.index_a;
        let index_b = contact_constraint.index_b;

        let m_a = contact_constraint.inv_mass_a;
        let i_a = contact_constraint.inv_i_a;
        let m_b = contact_constraint.inv_mass_b;
        let i_b = contact_constraint.inv_i_b;

        let mut state_a = if index_a == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_a as usize]
        };
        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let dq_a = state_a.delta_rotation;

        let mut state_b = if index_b == NULL_INDEX {
            IDENTITY_BODY_STATE
        } else {
            states[index_b as usize]
        };
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;
        let dq_b = state_b.delta_rotation;

        let dp = sub(state_b.delta_position, state_a.delta_position);
        let softness = contact_constraint.softness;
        let friction = contact_constraint.friction;
        let rolling_resistance = contact_constraint.rolling_resistance;

        for j in 0..manifold_count as usize {
            let constraint = &mut contact_constraint.constraints[j];

            let point_count = constraint.point_count;
            let normal = constraint.normal;

            let mut total_normal_impulse = 0.0;
            let mut total_twist_limit = 0.0;

            for point_index in 0..point_count as usize {
                let cp = &mut constraint.points[point_index];

                let r_a = cp.r_a;
                let r_b = cp.r_b;

                let ds = add(dp, sub(rotate_vector(dq_b, r_b), rotate_vector(dq_a, r_a)));
                let s = dot(ds, normal) + cp.base_separation;

                let mut velocity_bias = 0.0;
                let mut mass_scale = 1.0;
                let mut impulse_scale = 0.0;
                if s > 0.0 {
                    velocity_bias = s * inv_h;
                } else if use_bias {
                    velocity_bias =
                        max_float(softness.mass_scale * softness.bias_rate * s, -contact_speed);
                    mass_scale = softness.mass_scale;
                    impulse_scale = softness.impulse_scale;
                }

                let vr_a = add(v_a, cross(w_a, r_a));
                let vr_b = add(v_b, cross(w_b, r_b));
                let vn = dot(sub(vr_b, vr_a), normal);

                let mut delta_impulse = -cp.normal_mass * (mass_scale * vn + velocity_bias)
                    - impulse_scale * cp.normal_impulse;

                let new_impulse = max_float(cp.normal_impulse + delta_impulse, 0.0);
                delta_impulse = new_impulse - cp.normal_impulse;
                cp.normal_impulse = new_impulse;
                cp.total_normal_impulse += new_impulse;

                total_normal_impulse += new_impulse;
                total_twist_limit += cp.lever_arm * cp.normal_impulse;

                let p = mul_sv(delta_impulse, normal);
                v_a = mul_sub(v_a, m_a, p);
                w_a = sub(w_a, mul_mv(i_a, cross(r_a, p)));

                v_b = mul_add(v_b, m_b, p);
                w_b = add(w_b, mul_mv(i_b, cross(r_b, p)));
            }

            // No friction when applying bias
            if use_bias {
                continue;
            }

            // Central twist friction
            {
                let twist_speed = dot(constraint.normal, sub(w_b, w_a));
                let max_impulse = friction * total_twist_limit;
                let mut delta_impulse = -constraint.twist_mass * twist_speed;
                let old_impulse = constraint.twist_impulse;
                constraint.twist_impulse =
                    clamp_float(old_impulse + delta_impulse, -max_impulse, max_impulse);
                delta_impulse = constraint.twist_impulse - old_impulse;

                w_a = sub(w_a, mul_mv(i_a, mul_sv(delta_impulse, constraint.normal)));
                w_b = add(w_b, mul_mv(i_b, mul_sv(delta_impulse, constraint.normal)));
            }

            // Rolling resistance
            if rolling_resistance > 0.0 {
                let mut delta_impulse = neg(mul_mv(contact_constraint.rolling_mass, sub(w_b, w_a)));
                let old_impulse = constraint.rolling_impulse;
                constraint.rolling_impulse = add(old_impulse, delta_impulse);

                let max_impulse = rolling_resistance * total_normal_impulse;
                let mag_sqr = dot(constraint.rolling_impulse, constraint.rolling_impulse);
                if mag_sqr > max_impulse * max_impulse + f32::EPSILON {
                    constraint.rolling_impulse =
                        mul_sv(max_impulse / mag_sqr.sqrt(), constraint.rolling_impulse);
                }

                delta_impulse = sub(constraint.rolling_impulse, old_impulse);

                w_a = sub(w_a, mul_mv(i_a, delta_impulse));
                w_b = add(w_b, mul_mv(i_b, delta_impulse));
            }

            // Central friction
            {
                let tangent1 = constraint.tangent1;
                let tangent2 = constraint.tangent2;

                let r_a = constraint.origin_a;
                let r_b = constraint.origin_b;

                let vr_a = add(v_a, cross(w_a, r_a));
                let vr_b = add(v_b, cross(w_b, r_b));
                let vr = sub(vr_b, vr_a);
                let vt = Vec2 {
                    x: dot(vr, tangent1) - constraint.tangent_velocity1,
                    y: dot(vr, tangent2) - constraint.tangent_velocity2,
                };

                let tm = mul_mv2(constraint.tangent_mass, vt);
                let mut delta_impulse = Vec2 { x: -tm.x, y: -tm.y };
                let mut new_impulse = Vec2 {
                    x: constraint.friction_impulse.x + delta_impulse.x,
                    y: constraint.friction_impulse.y + delta_impulse.y,
                };

                let max_impulse = friction * total_normal_impulse;

                let length_squared = dot2(new_impulse, new_impulse);
                if length_squared > max_impulse * max_impulse {
                    let scale = max_impulse / length_squared.sqrt();
                    new_impulse.x *= scale;
                    new_impulse.y *= scale;
                }
                delta_impulse = sub2(new_impulse, constraint.friction_impulse);
                constraint.friction_impulse = new_impulse;

                let p = blend2(delta_impulse.x, tangent1, delta_impulse.y, tangent2);
                v_a = mul_sub(v_a, m_a, p);
                w_a = sub(w_a, mul_mv(i_a, cross(r_a, p)));
                v_b = mul_add(v_b, m_b, p);
                w_b = add(w_b, mul_mv(i_b, cross(r_b, p)));
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
}
