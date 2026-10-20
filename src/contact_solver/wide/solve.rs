//! Warm start, push, and solve wide convex contact constraints. (b3WarmStartContacts_Convex,
//! b3PushContacts_Convex, b3SolveContacts_Convex)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    add_v2w, gather_bodies, inv_rotate_vector_w, mul_add_mvw, mul_mv2w, mul_mvw, mul_sub_mvw,
    scatter_bodies, ContactConstraintWide, Vec2W,
};
use crate::body::BodyState;
use crate::simd_scalar::{
    add_vw, add_w, all_zero_w, blend_w, cross_w, div_w, dot_w, greater_than_w, max_w, mul_add_svw,
    mul_add_w, mul_sub_svw, mul_svw, mul_w, neg_w, soft_mask_w, splat_w, sqrt_w, sub_vw, sub_w,
    sym_clamp_w, zero_w, Vec3W,
};
use crate::solver::StepContext;

/// (b3WarmStartContacts_Convex)
pub fn warm_start_contacts_convex(constraints: &[ContactConstraintWide], states: &mut [BodyState]) {
    for c in constraints {
        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        let point_count = c.point_count();

        let zero = zero_w();
        let mut total_normal_impulse = zero;
        let mut moment_a = Vec3W {
            x: zero,
            y: zero,
            z: zero,
        };
        let mut moment_b = moment_a;

        for cp in c.points.iter().take(point_count) {
            let normal_impulse = cp.normal_impulses;
            total_normal_impulse = add_w(total_normal_impulse, normal_impulse);
            moment_a = mul_add_svw(moment_a, normal_impulse, cp.anchor_as);
            moment_b = mul_add_svw(moment_b, normal_impulse, cp.anchor_bs);
        }

        let normal = c.normal;
        let mut friction_impulse = mul_svw(c.friction_impulse.x, c.tangent1);
        friction_impulse = mul_add_svw(friction_impulse, c.friction_impulse.y, c.tangent2);

        let linear_impulse = mul_add_svw(friction_impulse, total_normal_impulse, normal);

        let twist_impulse = mul_svw(c.twist_impulse, normal);
        let mut angular_impulse_a = add_vw(
            add_vw(
                cross_w(moment_a, normal),
                cross_w(c.center_a, friction_impulse),
            ),
            twist_impulse,
        );
        let mut angular_impulse_b = add_vw(
            add_vw(
                cross_w(moment_b, normal),
                cross_w(c.center_b, friction_impulse),
            ),
            twist_impulse,
        );

        if !all_zero_w(c.rolling_resistance) {
            angular_impulse_a = add_vw(angular_impulse_a, c.rolling_impulse);
            angular_impulse_b = add_vw(angular_impulse_b, c.rolling_impulse);
        }

        b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, angular_impulse_a);
        b_a.v = mul_sub_svw(b_a.v, c.inv_mass_a, linear_impulse);
        b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, angular_impulse_b);
        b_b.v = mul_add_svw(b_b.v, c.inv_mass_b, linear_impulse);

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}

/// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
/// (b3PushContacts_Convex)
pub fn push_contacts_convex(
    constraints: &mut [ContactConstraintWide],
    states: &mut [BodyState],
    context: &StepContext,
) {
    let inv_h = splat_w(context.inv_h);
    let contact_speed = splat_w(-context.contact_speed);
    let one_w = splat_w(1.0);

    // Stiffer for static contacts to avoid bodies getting pushed through the ground. Selected per
    // lane from the null body index instead of stored per constraint.
    let dynamic_bias_rate =
        splat_w(context.contact_softness.mass_scale * context.contact_softness.bias_rate);
    let dynamic_mass_scale = splat_w(context.contact_softness.mass_scale);
    let dynamic_impulse_scale = splat_w(context.contact_softness.impulse_scale);
    let static_bias_rate =
        splat_w(context.static_softness.mass_scale * context.static_softness.bias_rate);
    let static_mass_scale = splat_w(context.static_softness.mass_scale);
    let static_impulse_scale = splat_w(context.static_softness.impulse_scale);

    for c in constraints.iter_mut() {
        let point_count = c.point_count();

        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        let soft_mask = soft_mask_w(&c.index_a, &c.index_b);
        let bias_rate = blend_w(dynamic_bias_rate, static_bias_rate, soft_mask);
        let mass_scale = blend_w(dynamic_mass_scale, static_mass_scale, soft_mask);
        let impulse_scale = blend_w(dynamic_impulse_scale, static_impulse_scale, soft_mask);

        let dp = sub_vw(b_b.dp, b_a.dp);

        // Convert to normals to local space to reduce transform math.
        let normal_separation = dot_w(c.normal, dp);
        let normal_a = inv_rotate_vector_w(b_a.dq, c.normal);
        let normal_b = inv_rotate_vector_w(b_b.dq, c.normal);

        for cp in c.points.iter_mut().take(point_count) {
            // Fixed anchor points for applying impulses
            let r_a = cp.anchor_as;
            let r_b = cp.anchor_bs;

            let s = add_w(
                add_w(
                    normal_separation,
                    sub_w(dot_w(normal_b, r_b), dot_w(normal_a, r_a)),
                ),
                cp.base_separations,
            );

            // Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
            let separated = greater_than_w(s, zero_w());

            // Speculative bias - positive
            let spec_bias = mul_w(s, inv_h);

            // Overlap bias - negative
            let overlap_bias = max_w(mul_w(bias_rate, s), contact_speed);
            let velocity_bias = blend_w(overlap_bias, spec_bias, separated);

            let point_mass_scale = blend_w(mass_scale, one_w, separated);
            let point_impulse_scale = blend_w(impulse_scale, zero_w(), separated);

            // Relative velocity at contact
            let vr_a = add_vw(b_a.v, cross_w(b_a.w, r_a));
            let vr_b = add_vw(b_b.v, cross_w(b_b.w, r_b));
            let vn = dot_w(sub_vw(vr_b, vr_a), c.normal);

            // Compute normal impulse
            let neg_impulse = add_w(
                mul_w(
                    cp.normal_masses,
                    add_w(mul_w(point_mass_scale, vn), velocity_bias),
                ),
                mul_w(point_impulse_scale, cp.normal_impulses),
            );

            // Clamp the accumulated impulse
            let new_impulse = max_w(sub_w(cp.normal_impulses, neg_impulse), zero_w());
            let delta_impulse = sub_w(new_impulse, cp.normal_impulses);
            cp.normal_impulses = new_impulse;

            // Apply contact impulse
            let p = mul_svw(delta_impulse, c.normal);
            b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, cross_w(r_a, p));
            b_a.v = mul_sub_svw(b_a.v, c.inv_mass_a, p);
            b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, cross_w(r_b, p));
            b_b.v = mul_add_svw(b_b.v, c.inv_mass_b, p);
        }

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}

/// Solve the normal constraint, friction, and rolling resistance. (b3SolveContacts_Convex)
pub fn solve_contacts_convex(
    constraints: &mut [ContactConstraintWide],
    states: &mut [BodyState],
    context: &StepContext,
) {
    let inv_h = splat_w(context.inv_h);
    let one_w = splat_w(1.0);
    let epsilon_w = splat_w(f32::EPSILON);

    for c in constraints.iter_mut() {
        let point_count = c.point_count();

        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        let dp = sub_vw(b_b.dp, b_a.dp);
        let normal_separation = dot_w(c.normal, dp);
        let normal_a = inv_rotate_vector_w(b_a.dq, c.normal);
        let normal_b = inv_rotate_vector_w(b_b.dq, c.normal);

        let mut total_normal_impulse = zero_w();
        let mut total_twist_limit = zero_w();

        for cp in c.points.iter_mut().take(point_count) {
            // Fixed anchor points for applying impulses
            let r_a = cp.anchor_as;
            let r_b = cp.anchor_bs;

            let s = add_w(
                add_w(
                    normal_separation,
                    sub_w(dot_w(normal_b, r_b), dot_w(normal_a, r_a)),
                ),
                cp.base_separations,
            );

            // Speculative bias, positive if separated and zero if overlapped
            let velocity_bias = max_w(zero_w(), mul_w(s, inv_h));

            // Relative velocity at contact
            let vr_a = add_vw(b_a.v, cross_w(b_a.w, r_a));
            let vr_b = add_vw(b_b.v, cross_w(b_b.w, r_b));
            let vn = dot_w(sub_vw(vr_b, vr_a), c.normal);

            // Compute normal impulse
            let neg_impulse = mul_w(cp.normal_masses, add_w(vn, velocity_bias));

            // Clamp the accumulated impulse
            let new_impulse = max_w(sub_w(cp.normal_impulses, neg_impulse), zero_w());
            let delta_impulse = sub_w(new_impulse, cp.normal_impulses);
            cp.normal_impulses = new_impulse;
            cp.total_normal_impulses = add_w(cp.total_normal_impulses, new_impulse);
            total_normal_impulse = add_w(total_normal_impulse, new_impulse);
            total_twist_limit = add_w(total_twist_limit, mul_w(cp.lever_arms, new_impulse));

            // Apply contact impulse
            let p = mul_svw(delta_impulse, c.normal);
            b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, cross_w(r_a, p));
            b_a.v = mul_sub_svw(b_a.v, c.inv_mass_a, p);
            b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, cross_w(r_b, p));
            b_b.v = mul_add_svw(b_b.v, c.inv_mass_b, p);
        }

        // Rolling resistance
        if !all_zero_w(c.rolling_resistance) {
            // flip A/B order to negate
            let mut delta_impulse = mul_mvw(c.rolling_mass, sub_vw(b_a.w, b_b.w));
            let old_impulse = c.rolling_impulse;
            c.rolling_impulse = add_vw(old_impulse, delta_impulse);

            let max_impulse = mul_w(c.rolling_resistance, total_normal_impulse);
            let length_squared = dot_w(c.rolling_impulse, c.rolling_impulse);

            // if ( magSqr > maxLambda * maxLambda + FLT_EPSILON )
            //{
            //	c->rollingImpulse *= maxLambda / sqrtf( magSqr );
            // }

            let mask = greater_than_w(
                length_squared,
                mul_add_w(epsilon_w, max_impulse, max_impulse),
            );

            // No approximate _mm_rsqrt_ps here to maintain cross-platform determinism
            let normalize = div_w(max_impulse, add_w(sqrt_w(length_squared), epsilon_w));
            let scale = blend_w(one_w, normalize, mask);

            // Ensure zero rolling resistance yields no impulse
            let rolling_mask = greater_than_w(c.rolling_resistance, zero_w());

            // Blend the result to ensure there are no +/-0 determinism problems, since 0 * -1 == -0.
            let scaled_impulse = mul_svw(scale, c.rolling_impulse);
            c.rolling_impulse.x = blend_w(zero_w(), scaled_impulse.x, rolling_mask);
            c.rolling_impulse.y = blend_w(zero_w(), scaled_impulse.y, rolling_mask);
            c.rolling_impulse.z = blend_w(zero_w(), scaled_impulse.z, rolling_mask);

            delta_impulse = sub_vw(c.rolling_impulse, old_impulse);

            b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, delta_impulse);
            b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, delta_impulse);
        }

        // Central twist friction
        {
            let twist_speed = dot_w(c.normal, sub_vw(b_b.w, b_a.w));
            let max_lambda = mul_w(c.friction, total_twist_limit);
            let mut delta_impulse = neg_w(mul_w(c.twist_mass, twist_speed));
            let old_impulse = c.twist_impulse;
            c.twist_impulse = sym_clamp_w(add_w(old_impulse, delta_impulse), max_lambda);
            delta_impulse = sub_w(c.twist_impulse, old_impulse);

            let l = mul_svw(delta_impulse, c.normal);
            b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, l);
            b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, l);
        }

        // Central friction
        {
            let tangent1 = c.tangent1;
            let tangent2 = c.tangent2;

            // Fixed anchor points for applying impulses
            let r_a = c.center_a;
            let r_b = c.center_b;

            // Relative tangent velocity at contact
            let vr_a = add_vw(b_a.v, cross_w(b_a.w, r_a));
            let vr_b = add_vw(b_b.v, cross_w(b_b.w, r_b));
            let vr = sub_vw(vr_b, vr_a);
            let vt = Vec2W {
                x: sub_w(dot_w(vr, tangent1), c.tangent_velocity1),
                y: sub_w(dot_w(vr, tangent2), c.tangent_velocity2),
            };

            // Incremental tangent impulse
            let mut delta_impulse = mul_mv2w(c.tangent_mass, vt);
            delta_impulse = Vec2W {
                x: neg_w(delta_impulse.x),
                y: neg_w(delta_impulse.y),
            };
            let mut new_impulse = add_v2w(c.friction_impulse, delta_impulse);

            let friction = c.friction;
            let max_impulse = mul_w(friction, total_normal_impulse);

            // Clamp the accumulated impulse
            let length_squared = add_w(
                mul_w(new_impulse.x, new_impulse.x),
                mul_w(new_impulse.y, new_impulse.y),
            );

            // Max impulse can be zero
            let mask = greater_than_w(length_squared, mul_w(max_impulse, max_impulse));

            // No approximate _mm_rsqrt_ps here to maintain cross-platform determinism. Add epsilon to avoid divide by
            // zero.
            let normalize = div_w(max_impulse, add_w(sqrt_w(length_squared), epsilon_w));
            let scale = blend_w(one_w, normalize, mask);
            new_impulse = Vec2W {
                x: mul_w(scale, new_impulse.x),
                y: mul_w(scale, new_impulse.y),
            };

            delta_impulse = Vec2W {
                x: sub_w(new_impulse.x, c.friction_impulse.x),
                y: sub_w(new_impulse.y, c.friction_impulse.y),
            };

            c.friction_impulse = new_impulse;

            // Apply delta impulse
            let p = add_vw(
                mul_svw(delta_impulse.x, tangent1),
                mul_svw(delta_impulse.y, tangent2),
            );
            b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, cross_w(r_a, p));
            b_a.v = mul_sub_svw(b_a.v, c.inv_mass_a, p);
            b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, cross_w(r_b, p));
            b_b.v = mul_add_svw(b_b.v, c.inv_mass_b, p);
        }

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}
