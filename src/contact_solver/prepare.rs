//! Prepare Mesh contact constraints. (b3PrepareContacts_Mesh)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{ContactConstraint, ManifoldConstraint, ManifoldConstraintPoint};
use crate::body::{decode_awake_index, BodySim, BodyState};
use crate::constants::{speculative_distance, MIN_FRICTION_WEIGHT};
use crate::contact::{contact_flags, Contact};
use crate::core::NULL_INDEX;
use crate::math_functions::{
    add, add_mm, clamp_float, cross, distance, dot, invert2, invert_matrix, mul_add, mul_mv,
    mul_sv, perp, sub, Vec2, MAT2_ZERO, MAT3_ZERO, VEC3_ZERO,
};
use crate::solver::StepContext;

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

/// Build one contact constraint from a touching contact. (inner loop of
/// b3PrepareContacts_Mesh)
///
/// Also writes `normal_velocity` onto the contact's manifold points, like C, because
/// the pre-solve normal velocity is only sampled here (and only when restitution or hit
/// events need it).
pub fn prepare_one_contact(
    contact: &mut Contact,
    sims: &[BodySim],
    states: &[BodyState],
    context: &StepContext,
    warm_start_scale: f32,
) -> ContactConstraint {
    let index_a = decode_awake_index(contact.encoded_body_sim_a);
    let index_b = decode_awake_index(contact.encoded_body_sim_b);

    // Body A data
    let (m_a, i_a) = if index_a == NULL_INDEX {
        (0.0, MAT3_ZERO)
    } else {
        let sim_a = &sims[index_a as usize];
        (sim_a.inv_mass, sim_a.inv_inertia_world)
    };

    // Body B data
    let (m_b, i_b) = if index_b == NULL_INDEX {
        (0.0, MAT3_ZERO)
    } else {
        let sim_b = &sims[index_b as usize];
        (sim_b.inv_mass, sim_b.inv_inertia_world)
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

    // Only sample contact point normal velocity if needed.
    let have_restitution = contact.restitution > 0.0;
    let hit_events = (contact.flags & contact_flags::SIM_ENABLE_HIT_EVENT) != 0;
    let sample_velocity = have_restitution || hit_events;

    let mut v_a = VEC3_ZERO;
    let mut w_a = VEC3_ZERO;
    let mut v_b = VEC3_ZERO;
    let mut w_b = VEC3_ZERO;

    if sample_velocity {
        if index_a != NULL_INDEX {
            v_a = states[index_a as usize].linear_velocity;
            w_a = states[index_a as usize].angular_velocity;
        }

        if index_b != NULL_INDEX {
            v_b = states[index_b as usize].linear_velocity;
            w_b = states[index_b as usize].angular_velocity;
        }
    }

    let tangent_velocity = contact.tangent_velocity;

    for manifold_index in 0..manifold_count as usize {
        let manifold = &mut contact.manifolds[manifold_index];
        let point_count = manifold.point_count;
        let normal = manifold.normal;
        let tangent1 = perp(normal);
        let tangent2 = cross(tangent1, normal);

        let mut mc = ManifoldConstraint {
            point_count,
            normal,
            tangent1,
            tangent2,
            tangent_velocity1: dot(tangent_velocity, tangent1),
            tangent_velocity2: dot(tangent_velocity, tangent2),
            ..ManifoldConstraint::default()
        };

        let mut center_a = VEC3_ZERO;
        let mut center_b = VEC3_ZERO;
        let mut total_friction_weight = 0.0;

        for point_index in 0..point_count as usize {
            // Copy data from manifold point
            let mp = &mut manifold.points[point_index];
            let s = mp.separation;
            let mut cp = ManifoldConstraintPoint {
                r_a: mp.anchor_a,
                r_b: mp.anchor_b,
                base_separation: s - dot(sub(mp.anchor_b, mp.anchor_a), normal),
                normal_impulse: warm_start_scale * mp.normal_impulse,
                total_normal_impulse: 0.0,
                restitution_impulse: 0.0,
                ..ManifoldConstraintPoint::default()
            };

            let r_a = cp.r_a;
            let r_b = cp.r_b;

            let rn_a = cross(r_a, normal);
            let rn_b = cross(r_b, normal);
            let k_normal = m_a + m_b + dot(rn_a, mul_mv(i_a, rn_a)) + dot(rn_b, mul_mv(i_b, rn_b));
            cp.normal_mass = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

            // Only compute the normal velocity terms if needed.
            if sample_velocity {
                let vr_a = add(v_a, cross(w_a, r_a));
                let vr_b = add(v_b, cross(w_b, r_b));
                let vn = dot(normal, sub(vr_b, vr_a));

                cp.relative_velocity = vn;
                mp.normal_velocity = if hit_events { vn } else { 0.0 };
            } else {
                cp.relative_velocity = 0.0;
                mp.normal_velocity = 0.0;
            }

            // C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
            // Contacts with separation greater than twice the speculative distance only matter for CCD and
            // should not contribute to the friction center. They are not important for jitter reduction. Closer
            // points may begin to touch on and off, so the friction center needs to move smoothly.
            // Epsilon to avoid a branch below (or divide by zero). Small enough to get washed out normally.
            //
            // Dirk suggested to weight the points by their normal impulse. This would use the total impulse from
            // the previous time step. I suspect this could jump around if the active point set jitters, leading
            // to a limit cycle.
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

/// Prepare overflow constraints. Returns true if any contact has restitution.
/// (b3PrepareContacts_Mesh — serial)
pub fn prepare_mesh_contacts(
    constraints: &mut Vec<ContactConstraint>,
    mesh_specs: &[crate::contact::ContactSpec],
    contacts: &mut [Contact],
    sims: &[BodySim],
    states: &[BodyState],
    context: &StepContext,
) -> bool {
    let warm_start_scale = if context.enable_warm_starting {
        1.0
    } else {
        0.0
    };
    let mut any_restitution = false;

    constraints.clear();
    constraints.reserve(mesh_specs.len());

    for spec in mesh_specs {
        let contact = &mut contacts[spec.contact_id as usize];
        debug_assert!(contact.contact_id == spec.contact_id);
        any_restitution = any_restitution || contact.restitution > 0.0;
        constraints.push(prepare_one_contact(
            contact,
            sims,
            states,
            context,
            warm_start_scale,
        ));
    }

    any_restitution
}
