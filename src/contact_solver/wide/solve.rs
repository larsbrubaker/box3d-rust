//! Warm start, push, and solve wide contact constraints. These are shared between convex and
//! mesh contacts. (b3WarmStartContactsW, b3PushContactsW, b3SolveContactsW)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    add_v2w, gather_bodies, inv_rotate_vector_w, mul_add_mvw, mul_mv2w, mul_mvw, mul_sub_mvw,
    scatter_bodies, Vec2W, WideContactConstraints, VEC3W_ZERO,
};
use crate::body::BodyState;
use crate::simd_scalar::{
    add_vw, add_w, all_zero_w, blend_w, cross_w, div_w, dot_w, greater_than_w, max_w, min_w,
    mul_add_svw, mul_add_w, mul_sub_svw, mul_svw, mul_w, neg_w, soft_mask_w, splat_w, sqrt_w,
    sub_vw, sub_w, zero_w,
};
use crate::solver::StepContext;

/// (b3WarmStartContactsW, b3WarmStartContacts_Convex, b3WarmStartContacts_MeshWide)
pub fn warm_start_contacts_w(set: &WideContactConstraints, states: &mut [BodyState]) {
    let zero = zero_w();

    for (i, c) in set.constraints.iter().enumerate() {
        let any_rolling = !all_zero_w(c.rolling_resistance);

        let mut linear_impulse = VEC3W_ZERO;
        let mut angular_impulse_a = VEC3W_ZERO;
        let mut angular_impulse_b = VEC3W_ZERO;

        let manifolds = &set.manifolds[set.manifold_range(i)];
        debug_assert!(manifolds.len() == c.manifold_count as usize);
        for cm in manifolds {
            let mut total_normal_impulse = zero;
            let mut moment_a = VEC3W_ZERO;
            let mut moment_b = VEC3W_ZERO;

            for cp in &cm.points[..cm.point_count as usize] {
                let normal_impulse = cp.normal_impulses;
                total_normal_impulse = add_w(total_normal_impulse, normal_impulse);
                moment_a = mul_add_svw(moment_a, normal_impulse, cp.anchor_as);
                moment_b = mul_add_svw(moment_b, normal_impulse, cp.anchor_bs);
            }

            let normal = cm.normal;
            let mut friction_impulse = mul_svw(cm.friction_impulse.x, cm.tangent1);
            friction_impulse = mul_add_svw(friction_impulse, cm.friction_impulse.y, cm.tangent2);

            let linear = mul_add_svw(friction_impulse, total_normal_impulse, normal);

            let twist_impulse = mul_svw(cm.twist_impulse, normal);
            let mut angular_a = add_vw(
                add_vw(
                    cross_w(moment_a, normal),
                    cross_w(cm.center_a, friction_impulse),
                ),
                twist_impulse,
            );
            let mut angular_b = add_vw(
                add_vw(
                    cross_w(moment_b, normal),
                    cross_w(cm.center_b, friction_impulse),
                ),
                twist_impulse,
            );

            if any_rolling {
                angular_a = add_vw(angular_a, cm.rolling_impulse);
                angular_b = add_vw(angular_b, cm.rolling_impulse);
            }

            linear_impulse = add_vw(linear_impulse, linear);
            angular_impulse_a = add_vw(angular_impulse_a, angular_a);
            angular_impulse_b = add_vw(angular_impulse_b, angular_b);
        }

        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, angular_impulse_a);
        b_a.v = mul_sub_svw(b_a.v, c.inv_mass_a, linear_impulse);
        b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, angular_impulse_b);
        b_b.v = mul_add_svw(b_b.v, c.inv_mass_b, linear_impulse);

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}

/// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
/// (b3PushContactsW, b3PushContacts_Convex, b3PushContacts_MeshWide)
pub fn push_contacts_w(
    set: &mut WideContactConstraints,
    states: &mut [BodyState],
    context: &StepContext,
) {
    let inv_h = splat_w(context.inv_h);
    let contact_speed = splat_w(-context.contact_speed);
    let one_w = splat_w(1.0);
    let zero = zero_w();

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

    let WideContactConstraints {
        constraints,
        manifolds,
        manifold_starts,
    } = set;

    for (i, c) in constraints.iter().enumerate() {
        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        let soft_mask = soft_mask_w(&c.index_a, &c.index_b);
        let bias_rate = blend_w(dynamic_bias_rate, static_bias_rate, soft_mask);
        let mass_scale = blend_w(dynamic_mass_scale, static_mass_scale, soft_mask);
        let impulse_scale = blend_w(dynamic_impulse_scale, static_impulse_scale, soft_mask);

        let dp = sub_vw(b_b.dp, b_a.dp);
        let inv_mass = add_w(c.inv_mass_a, c.inv_mass_b);

        let range = manifold_starts[i] as usize..manifold_starts[i + 1] as usize;
        for cm in &mut manifolds[range] {
            // Convert to normals to local space to reduce transform math.
            let normal_separation = dot_w(cm.normal, dp);
            let normal_a = inv_rotate_vector_w(b_a.dq, cm.normal);
            let normal_b = inv_rotate_vector_w(b_b.dq, cm.normal);
            let normal = cm.normal;

            let mut linear_normal_velocity = dot_w(normal, sub_vw(b_b.v, b_a.v));
            let mut manifold_impulse = zero;

            for cp in &mut cm.points[..cm.point_count as usize] {
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
                let separated = greater_than_w(s, zero);

                // Speculative bias - positive
                let spec_bias = mul_w(s, inv_h);

                // Overlap bias - negative
                let overlap_bias = max_w(mul_w(bias_rate, s), contact_speed);
                let velocity_bias = blend_w(overlap_bias, spec_bias, separated);

                let point_mass_scale = blend_w(mass_scale, one_w, separated);
                let point_impulse_scale = blend_w(impulse_scale, zero, separated);

                let u_a = cross_w(r_a, normal);
                let u_b = cross_w(r_b, normal);
                let g_a = mul_mvw(c.inv_i_a, u_a);
                let g_b = mul_mvw(c.inv_i_b, u_b);

                let vn = add_w(
                    linear_normal_velocity,
                    sub_w(dot_w(u_b, b_b.w), dot_w(u_a, b_a.w)),
                );

                let neg_impulse = add_w(
                    mul_w(
                        cp.normal_masses,
                        add_w(mul_w(point_mass_scale, vn), velocity_bias),
                    ),
                    mul_w(point_impulse_scale, cp.normal_impulses),
                );
                let impulse = max_w(neg_w(neg_impulse), neg_w(cp.normal_impulses));
                cp.normal_impulses = add_w(cp.normal_impulses, impulse);

                linear_normal_velocity = mul_add_w(linear_normal_velocity, inv_mass, impulse);
                manifold_impulse = add_w(manifold_impulse, impulse);
                b_a.w = mul_sub_svw(b_a.w, impulse, g_a);
                b_b.w = mul_add_svw(b_b.w, impulse, g_b);
            }

            let p = mul_svw(manifold_impulse, normal);
            b_a.v = mul_sub_svw(b_a.v, c.inv_mass_a, p);
            b_b.v = mul_add_svw(b_b.v, c.inv_mass_b, p);
        }

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}

/// Solve the normal constraint, friction, and rolling resistance.
/// (b3SolveContactsW, b3SolveContacts_Convex, b3SolveContacts_MeshWide)
pub fn solve_contacts_w(
    set: &mut WideContactConstraints,
    states: &mut [BodyState],
    context: &StepContext,
) {
    let inv_h = splat_w(context.inv_h);
    let one_w = splat_w(1.0);
    let zero = zero_w();
    let epsilon_w = splat_w(f32::EPSILON);

    let WideContactConstraints {
        constraints,
        manifolds,
        manifold_starts,
    } = set;

    for (i, c) in constraints.iter().enumerate() {
        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        let any_rolling = !all_zero_w(c.rolling_resistance);
        let dp = sub_vw(b_b.dp, b_a.dp);
        let inv_mass = add_w(c.inv_mass_a, c.inv_mass_b);

        let range = manifold_starts[i] as usize..manifold_starts[i + 1] as usize;
        for cm in &mut manifolds[range] {
            let normal_separation = dot_w(cm.normal, dp);
            let normal_a = inv_rotate_vector_w(b_a.dq, cm.normal);
            let normal_b = inv_rotate_vector_w(b_b.dq, cm.normal);
            let normal = cm.normal;

            let dv = sub_vw(b_b.v, b_a.v);
            let mut linear_normal_velocity = dot_w(normal, dv);
            let linear_tangent_velocity1 = sub_w(dot_w(cm.tangent1, dv), cm.tangent_velocity1);
            let linear_tangent_velocity2 = sub_w(dot_w(cm.tangent2, dv), cm.tangent_velocity2);
            let mut manifold_impulse = zero;

            let mut total_normal_impulse = zero;
            let mut total_twist_limit = zero;

            // Normal constraints
            for cp in &mut cm.points[..cm.point_count as usize] {
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
                let velocity_bias = max_w(zero, mul_w(s, inv_h));

                let u_a = cross_w(r_a, normal);
                let u_b = cross_w(r_b, normal);
                let g_a = mul_mvw(c.inv_i_a, u_a);
                let g_b = mul_mvw(c.inv_i_b, u_b);

                let vn = add_w(
                    linear_normal_velocity,
                    sub_w(dot_w(u_b, b_b.w), dot_w(u_a, b_a.w)),
                );

                let neg_impulse = mul_w(cp.normal_masses, add_w(vn, velocity_bias));
                let impulse = max_w(neg_w(neg_impulse), neg_w(cp.normal_impulses));
                let new_impulse = add_w(cp.normal_impulses, impulse);
                cp.normal_impulses = new_impulse;
                cp.total_normal_impulses = add_w(cp.total_normal_impulses, new_impulse);
                total_normal_impulse = add_w(total_normal_impulse, new_impulse);
                total_twist_limit = add_w(total_twist_limit, mul_w(cp.lever_arms, new_impulse));

                linear_normal_velocity = mul_add_w(linear_normal_velocity, inv_mass, impulse);
                manifold_impulse = add_w(manifold_impulse, impulse);
                b_a.w = mul_sub_svw(b_a.w, impulse, g_a);
                b_b.w = mul_add_svw(b_b.w, impulse, g_b);
            }

            // Rolling resistance. After normal constraints because it uses the normal impulse.
            if any_rolling {
                // flip A/B order to negate
                let mut delta_impulse = mul_mvw(c.rolling_mass, sub_vw(b_a.w, b_b.w));
                let old_impulse = cm.rolling_impulse;
                cm.rolling_impulse = add_vw(old_impulse, delta_impulse);

                let max_impulse = mul_w(c.rolling_resistance, total_normal_impulse);
                let length_squared = dot_w(cm.rolling_impulse, cm.rolling_impulse);

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

                // Blend the result to ensure there are no +/-0 determinism problems, since 0 * -1 == -0.
                let scaled_impulse = mul_svw(scale, cm.rolling_impulse);
                cm.rolling_impulse.x = blend_w(zero, scaled_impulse.x, cm.rolling_mask);
                cm.rolling_impulse.y = blend_w(zero, scaled_impulse.y, cm.rolling_mask);
                cm.rolling_impulse.z = blend_w(zero, scaled_impulse.z, cm.rolling_mask);

                delta_impulse = sub_vw(cm.rolling_impulse, old_impulse);

                b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, delta_impulse);
                b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, delta_impulse);
            }

            // Central twist friction. Friction goes last to improve stacking stability.
            {
                let twist_speed = dot_w(normal, sub_vw(b_b.w, b_a.w));
                let max_lambda = mul_w(c.friction, total_twist_limit);
                let old_impulse = cm.twist_impulse;
                let mut impulse = neg_w(mul_w(cm.twist_mass, twist_speed));
                impulse = max_w(
                    sub_w(neg_w(max_lambda), old_impulse),
                    min_w(impulse, sub_w(max_lambda, old_impulse)),
                );
                cm.twist_impulse = add_w(old_impulse, impulse);

                b_a.w = mul_sub_svw(b_a.w, impulse, mul_mvw(c.inv_i_a, normal));
                b_b.w = mul_add_svw(b_b.w, impulse, mul_mvw(c.inv_i_b, normal));
            }

            // Central friction. Friction goes last to improve stacking stability.
            {
                let tangent1 = cm.tangent1;
                let tangent2 = cm.tangent2;

                // Fixed anchor points for applying impulses
                let r_a = cm.center_a;
                let r_b = cm.center_b;

                let angular_relative_velocity = sub_vw(cross_w(b_b.w, r_b), cross_w(b_a.w, r_a));

                // Relative tangent velocity at contact
                let vt = Vec2W {
                    x: add_w(
                        linear_tangent_velocity1,
                        dot_w(angular_relative_velocity, tangent1),
                    ),
                    y: add_w(
                        linear_tangent_velocity2,
                        dot_w(angular_relative_velocity, tangent2),
                    ),
                };

                // Incremental tangent impulse
                let mut delta_impulse = mul_mv2w(cm.tangent_mass, vt);
                delta_impulse = Vec2W {
                    x: neg_w(delta_impulse.x),
                    y: neg_w(delta_impulse.y),
                };
                let mut new_impulse = add_v2w(cm.friction_impulse, delta_impulse);

                let max_impulse = mul_w(c.friction, total_normal_impulse);

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
                    x: sub_w(new_impulse.x, cm.friction_impulse.x),
                    y: sub_w(new_impulse.y, cm.friction_impulse.y),
                };

                cm.friction_impulse = new_impulse;

                // Apply delta impulse
                let mut p = add_vw(
                    mul_svw(delta_impulse.x, tangent1),
                    mul_svw(delta_impulse.y, tangent2),
                );
                b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, cross_w(r_a, p));
                b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, cross_w(r_b, p));

                p = mul_add_svw(p, manifold_impulse, normal);
                b_a.v = mul_sub_svw(b_a.v, c.inv_mass_a, p);
                b_b.v = mul_add_svw(b_b.v, c.inv_mass_b, p);
            }
        }

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}
