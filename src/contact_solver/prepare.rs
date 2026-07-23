//! Prepare Mesh contact constraints. (b3PrepareContacts_Mesh)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{ContactConstraint, ManifoldConstraint, ManifoldConstraintPoint};
use crate::body::{BodySim, BodyState};
use crate::contact::{contact_flags, Contact};
use crate::core::NULL_INDEX;
use crate::constants::{speculative_distance, MIN_FRICTION_WEIGHT};
use crate::math_functions::{
    add, add_mm, clamp_float, cross, distance, dot, invert2, invert_matrix, mul_add, mul_mv, mul_sv,
    perp, sub, Vec2, MAT2_ZERO, MAT3_ZERO, VEC3_ZERO,
};
use crate::solver::StepContext;

/// Build one contact constraint from a touching contact. (inner loop of
/// b3PrepareContacts_Mesh)
pub fn prepare_one_contact(
    contact: &Contact,
    sims: &[BodySim],
    states: &[BodyState],
    context: &StepContext,
    warm_start_scale: f32,
) -> ContactConstraint {
    let index_a = contact.body_sim_index_a;
    let index_b = contact.body_sim_index_b;

    let (m_a, i_a, v_a, w_a) = if index_a == NULL_INDEX {
        (0.0, MAT3_ZERO, VEC3_ZERO, VEC3_ZERO)
    } else {
        let sim_a = &sims[index_a as usize];
        let state_a = &states[index_a as usize];
        (
            sim_a.inv_mass,
            sim_a.inv_inertia_world,
            state_a.linear_velocity,
            state_a.angular_velocity,
        )
    };

    let (m_b, i_b, v_b, w_b) = if index_b == NULL_INDEX {
        (0.0, MAT3_ZERO, VEC3_ZERO, VEC3_ZERO)
    } else {
        let sim_b = &sims[index_b as usize];
        let state_b = &states[index_b as usize];
        (
            sim_b.inv_mass,
            sim_b.inv_inertia_world,
            state_b.linear_velocity,
            state_b.angular_velocity,
        )
    };

    // Used for friction center weighting.
    let inv_tau = 1.0 / speculative_distance();

    let manifold_count = contact.manifold_count();
    let softness = if (contact.flags & contact_flags::STATIC_FLAG) != 0 {
        context.static_softness
    } else {
        context.contact_softness
    };

    let mut constraint = ContactConstraint {
        constraints: Vec::with_capacity(manifold_count as usize),
        contact_id: contact.contact_id,
        index_a,
        index_b,
        inv_mass_a: m_a,
        inv_mass_b: m_b,
        inv_i_a: i_a,
        inv_i_b: i_b,
        softness,
        rolling_mass: invert_matrix(add_mm(i_a, i_b)),
        friction: contact.friction,
        restitution: contact.restitution,
        rolling_resistance: contact.rolling_resistance,
        manifold_count,
    };

    for manifold_index in 0..manifold_count as usize {
        let manifold = &contact.manifolds[manifold_index];
        let point_count = manifold.point_count;
        let normal = manifold.normal;
        let tangent1 = perp(normal);
        let tangent2 = cross(tangent1, normal);

        let mut mc = ManifoldConstraint {
            point_count,
            normal,
            tangent1,
            tangent2,
            tangent_velocity1: dot(contact.tangent_velocity, tangent1),
            tangent_velocity2: dot(contact.tangent_velocity, tangent2),
            ..ManifoldConstraint::default()
        };

        let mut center_a = VEC3_ZERO;
        let mut center_b = VEC3_ZERO;
        let mut total_friction_weight = 0.0;

        for point_index in 0..point_count as usize {
            let mp = &manifold.points[point_index];
            let s = mp.separation;
            let mut cp = ManifoldConstraintPoint {
                r_a: mp.anchor_a,
                r_b: mp.anchor_b,
                base_separation: s - dot(sub(mp.anchor_b, mp.anchor_a), normal),
                normal_impulse: warm_start_scale * mp.normal_impulse,
                total_normal_impulse: 0.0,
                ..ManifoldConstraintPoint::default()
            };

            let r_a = cp.r_a;
            let r_b = cp.r_b;

            let rn_a = cross(r_a, normal);
            let rn_b = cross(r_b, normal);
            let k_normal = m_a + m_b + dot(rn_a, mul_mv(i_a, rn_a)) + dot(rn_b, mul_mv(i_b, rn_b));
            cp.normal_mass = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

            let vr_a = add(v_a, cross(w_a, r_a));
            let vr_b = add(v_b, cross(w_b, r_b));
            cp.relative_velocity = dot(normal, sub(vr_b, vr_a));

            // C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
            // Contacts with separation greater than twice the speculative distance only matter for CCD and
            // should not contribute to the friction center. They are not important for jitter reduction. Closer
            // points may begin to touch on and off, so the friction center needs to move smoothly.
            // Epsilon to avoid a branch below (or divide by zero). Small enough to get washed out normally.
            let weight = clamp_float(2.0 - s * inv_tau, MIN_FRICTION_WEIGHT, 1.0);
            center_a = mul_add(center_a, weight, r_a);
            center_b = mul_add(center_b, weight, r_b);
            total_friction_weight += weight;

            mc.points[point_index] = cp;
        }

        let inv_weight = 1.0 / total_friction_weight;
        center_a = mul_sv(inv_weight, center_a);
        center_b = mul_sv(inv_weight, center_b);
        mc.center_a = center_a;
        mc.center_b = center_b;

        for point_index in 0..point_count as usize {
            mc.points[point_index].lever_arm = distance(mc.points[point_index].r_a, center_a);
        }

        let rt_a1 = cross(center_a, tangent1);
        let rt_a2 = cross(center_a, tangent2);
        let rt_b1 = cross(center_b, tangent1);
        let rt_b2 = cross(center_b, tangent2);

        {
            let mut k = MAT2_ZERO;
            k.cx.x = m_a + m_b + dot(rt_a1, mul_mv(i_a, rt_a1)) + dot(rt_b1, mul_mv(i_b, rt_b1));
            k.cy.y = m_a + m_b + dot(rt_a2, mul_mv(i_a, rt_a2)) + dot(rt_b2, mul_mv(i_b, rt_b2));
            k.cx.y = dot(rt_a1, mul_mv(i_a, rt_a2)) + dot(rt_b1, mul_mv(i_b, rt_b2));
            k.cy.x = k.cx.y;

            mc.tangent_mass = invert2(k);
            mc.friction_impulse = Vec2 {
                x: warm_start_scale * dot(manifold.friction_impulse, tangent1),
                y: warm_start_scale * dot(manifold.friction_impulse, tangent2),
            };
        }

        {
            let k = dot(normal, mul_mv(add_mm(i_a, i_b), normal));
            mc.twist_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
            mc.twist_impulse = warm_start_scale * manifold.twist_impulse;
        }

        mc.rolling_impulse = mul_sv(warm_start_scale, manifold.rolling_impulse);

        constraint.constraints.push(mc);
    }

    constraint
}

/// Prepare convex then mesh contacts into one constraint Vec.
///
/// Returns the convex prefix length so the solver can dispatch Convex vs Mesh
/// kernels. Matches C's per-color order: wide/convex blocks then mesh blocks.
/// (b3PrepareContacts_Convex + b3PrepareContacts_Mesh — serial)
pub fn prepare_color_contacts(
    constraints: &mut Vec<ContactConstraint>,
    convex_ids: &[i32],
    mesh_specs: &[crate::contact::ContactSpec],
    contacts: &[Contact],
    sims: &[BodySim],
    states: &[BodyState],
    context: &StepContext,
) -> usize {
    let warm_start_scale = if context.enable_warm_starting {
        1.0
    } else {
        0.0
    };

    constraints.clear();
    constraints.reserve(convex_ids.len() + mesh_specs.len());

    for &contact_id in convex_ids {
        let contact = &contacts[contact_id as usize];
        debug_assert!(contact.contact_id == contact_id);
        debug_assert!(contact.manifold_count() == 1);
        constraints.push(prepare_one_contact(
            contact,
            sims,
            states,
            context,
            warm_start_scale,
        ));
    }

    let convex_count = constraints.len();

    for spec in mesh_specs {
        let contact = &contacts[spec.contact_id as usize];
        debug_assert!(contact.contact_id == spec.contact_id);
        constraints.push(prepare_one_contact(
            contact,
            sims,
            states,
            context,
            warm_start_scale,
        ));
    }

    convex_count
}
