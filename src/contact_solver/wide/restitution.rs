//! Post-solve restitution for wide contact constraints. (b3ApplyRestitutionW,
//! b3ApplyRestitution_Convex, b3ApplyRestitution_MeshWide)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    gather_bodies, inv_rotate_vector_w, mul_add_mvw, mul_sub_mvw, scatter_bodies,
    WideContactConstraints,
};
use crate::body::BodyState;
use crate::simd_scalar::{
    add_vw, add_w, all_zero_w, and_w, blend_w, cross_w, dot_w, greater_than_w, less_than_w, max_w,
    min_w, mul_add_svw, mul_sub_svw, mul_svw, mul_w, neg_w, splat_w, sub_vw, sub_w, zero_w,
};
use crate::solver::StepContext;

/// Poisson kinetic restitution for the wide constraints of one color.
/// (b3ApplyRestitutionW)
pub fn apply_restitution_w(
    set: &mut WideContactConstraints,
    states: &mut [BodyState],
    context: &StepContext,
    restitution_threshold: f32,
    propagate: bool,
) {
    let inv_h = splat_w(context.inv_h);
    let neg_restitution_threshold = splat_w(-restitution_threshold);
    let zero = zero_w();

    let WideContactConstraints {
        constraints,
        manifolds,
        manifold_starts,
    } = set;

    for (i, c) in constraints.iter().enumerate() {
        if !propagate && all_zero_w(c.restitution) {
            continue;
        }

        let mut b_a = gather_bodies(states, &c.index_a);
        let mut b_b = gather_bodies(states, &c.index_b);

        let restitution_mask = greater_than_w(c.restitution, zero);
        let dp = sub_vw(b_b.dp, b_a.dp);

        let range = manifold_starts[i] as usize..manifold_starts[i + 1] as usize;
        for cm in &mut manifolds[range] {
            let normal_separation = dot_w(cm.normal, dp);
            let normal_a = inv_rotate_vector_w(b_a.dq, cm.normal);
            let normal_b = inv_rotate_vector_w(b_b.dq, cm.normal);
            let normal = cm.normal;

            for cp in &mut cm.points[..cm.point_count as usize] {
                let r_a = cp.anchor_as;
                let r_b = cp.anchor_bs;

                let s = add_w(
                    add_w(
                        normal_separation,
                        sub_w(dot_w(normal_b, r_b), dot_w(normal_a, r_a)),
                    ),
                    cp.base_separations,
                );

                let normal_mass = if propagate {
                    cp.normal_masses
                } else {
                    blend_w(zero, cp.normal_masses, restitution_mask)
                };

                let compression_impulse = sub_w(cp.total_normal_impulses, cp.restitution_impulses);
                let armed = and_w(
                    and_w(
                        restitution_mask,
                        less_than_w(cp.relative_velocities, neg_restitution_threshold),
                    ),
                    greater_than_w(compression_impulse, zero),
                );

                let spec_bias = max_w(zero, mul_w(s, inv_h));
                let velocity_bias = blend_w(
                    spec_bias,
                    mul_w(c.restitution, cp.relative_velocities),
                    armed,
                );

                let vr_a = add_vw(b_a.v, cross_w(b_a.w, r_a));
                let vr_b = add_vw(b_b.v, cross_w(b_b.w, r_b));
                let vn = dot_w(sub_vw(vr_b, vr_a), normal);

                let neg_impulse = mul_w(normal_mass, add_w(vn, velocity_bias));

                let new_impulse = max_w(sub_w(cp.normal_impulses, neg_impulse), zero);
                let mut impulse = sub_w(new_impulse, cp.normal_impulses);

                let approach_impulse = min_w(
                    max_w(neg_w(mul_w(normal_mass, vn)), zero),
                    max_w(impulse, zero),
                );
                let allowance = sub_w(
                    mul_w(c.restitution, add_w(compression_impulse, approach_impulse)),
                    cp.restitution_impulses,
                );
                let max_impulse = add_w(approach_impulse, max_w(allowance, zero));
                impulse = blend_w(impulse, min_w(impulse, max_impulse), armed);

                cp.normal_impulses = add_w(cp.normal_impulses, impulse);
                cp.restitution_impulses =
                    add_w(cp.restitution_impulses, sub_w(impulse, approach_impulse));
                cp.total_normal_impulses = add_w(cp.total_normal_impulses, impulse);

                let p = mul_svw(impulse, normal);
                b_a.w = mul_sub_mvw(b_a.w, c.inv_i_a, cross_w(r_a, p));
                b_a.v = mul_sub_svw(b_a.v, c.inv_mass_a, p);
                b_b.w = mul_add_mvw(b_b.w, c.inv_i_b, cross_w(r_b, p));
                b_b.v = mul_add_svw(b_b.v, c.inv_mass_b, p);
            }
        }

        scatter_bodies(states, &c.index_a, &b_a);
        scatter_bodies(states, &c.index_b, &b_b);
    }
}
