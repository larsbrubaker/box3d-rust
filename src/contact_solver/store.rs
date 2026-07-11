//! Store solved impulses back onto contact manifolds. (b3StoreImpulses_Mesh)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::ContactConstraint;
use crate::contact::Contact;
use crate::math_functions::blend2;

/// Write impulses from constraints onto world contacts.
/// (b3StoreImpulses_Mesh — impulse writeback; hit-event flagging is separate)
pub fn store_impulses(constraints: &[ContactConstraint], contacts: &mut [Contact]) {
    for contact_constraint in constraints {
        let contact_id = contact_constraint.contact_id;
        let contact = &mut contacts[contact_id as usize];
        debug_assert!(contact.contact_id == contact_id);

        let manifold_count = contact_constraint.manifold_count;
        debug_assert!(manifold_count == contact.manifold_count());

        for manifold_index in 0..manifold_count as usize {
            let manifold = &mut contact.manifolds[manifold_index];
            let constraint = &contact_constraint.constraints[manifold_index];
            manifold.twist_impulse = constraint.twist_impulse;
            manifold.friction_impulse = blend2(
                constraint.friction_impulse.x,
                constraint.tangent1,
                constraint.friction_impulse.y,
                constraint.tangent2,
            );
            manifold.rolling_impulse = constraint.rolling_impulse;

            let count = constraint.point_count;
            debug_assert!(count == manifold.point_count);
            for point_index in 0..count as usize {
                let cp = &constraint.points[point_index];
                let mp = &mut manifold.points[point_index];
                mp.normal_impulse = cp.normal_impulse;
                mp.total_normal_impulse = cp.total_normal_impulse;
                mp.normal_velocity = cp.relative_velocity;
            }
        }
    }
}

/// Flag hit-event candidates after impulses are stored. (hit-event part of
/// b3StoreImpulses_Mesh for colored contacts)
pub fn flag_hit_events(
    constraints: &[ContactConstraint],
    contacts: &[Contact],
    hit_event_bit_set: &mut crate::bitset::BitSet,
    has_hit_events: &mut bool,
    neg_hit_threshold: f32,
) {
    use crate::contact::contact_flags;

    for contact_constraint in constraints {
        let contact = &contacts[contact_constraint.contact_id as usize];
        if (contact.flags & contact_flags::SIM_ENABLE_HIT_EVENT) == 0 {
            continue;
        }

        let mut flagged = false;
        let manifold_count = contact_constraint.manifold_count;
        for manifold_index in 0..manifold_count as usize {
            let constraint = &contact_constraint.constraints[manifold_index];
            let count = constraint.point_count;
            for point_index in 0..count as usize {
                let cp = &constraint.points[point_index];
                if !flagged
                    && cp.relative_velocity < neg_hit_threshold
                    && cp.total_normal_impulse > 0.0
                {
                    hit_event_bit_set.set_bit(contact.contact_id as u32);
                    *has_hit_events = true;
                    flagged = true;
                }
            }
        }
    }
}
