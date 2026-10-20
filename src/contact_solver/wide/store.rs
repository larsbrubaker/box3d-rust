//! Store wide impulses back onto the contact manifolds. (b3StoreImpulsesW,
//! b3StoreImpulses_Convex, b3StoreImpulses_MeshWide)
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{lane, WideContactConstraints, EMPTY_LANE};
use crate::bitset::BitSet;
use crate::contact::{contact_flags, Contact};
use crate::core::SIMD_WIDTH;
use crate::math_functions::Vec3;

/// Store impulses by contact constraint and flag hit-event candidates. (b3StoreImpulsesW)
pub fn store_impulses_w(
    set: &WideContactConstraints,
    contacts: &mut [Contact],
    hit_event_bit_set: &mut BitSet,
    has_hit_events: &mut bool,
    neg_hit_threshold: f32,
) {
    for (i, c) in set.constraints.iter().enumerate() {
        let manifold_count = c.manifold_count;
        let manifolds = &set.manifolds[set.manifold_range(i)];

        for l in 0..SIMD_WIDTH {
            let contact_id = c.contact_ids[l];
            if contact_id == EMPTY_LANE {
                continue;
            }

            let contact = &mut contacts[contact_id as usize];
            let lane_manifold_count = contact.manifold_count();
            debug_assert!(lane_manifold_count <= manifold_count);

            for manifold_index in 0..lane_manifold_count as usize {
                let cm = &manifolds[manifold_index];
                let m = &mut contact.manifolds[manifold_index];

                let f1 = lane(cm.friction_impulse.x, l);
                let f2 = lane(cm.friction_impulse.y, l);
                m.friction_impulse = Vec3 {
                    x: f1 * lane(cm.tangent1.x, l) + f2 * lane(cm.tangent2.x, l),
                    y: f1 * lane(cm.tangent1.y, l) + f2 * lane(cm.tangent2.y, l),
                    z: f1 * lane(cm.tangent1.z, l) + f2 * lane(cm.tangent2.z, l),
                };
                m.twist_impulse = lane(cm.twist_impulse, l);
                m.rolling_impulse = Vec3 {
                    x: lane(cm.rolling_impulse.x, l),
                    y: lane(cm.rolling_impulse.y, l),
                    z: lane(cm.rolling_impulse.z, l),
                };

                let point_count = m.point_count as usize;
                debug_assert!(point_count <= cm.point_count as usize);
                for point_index in 0..point_count {
                    let cp = &cm.points[point_index];
                    let mp = &mut m.points[point_index];
                    mp.normal_impulse = lane(cp.normal_impulses, l);
                    mp.total_normal_impulse = lane(cp.total_normal_impulses, l);
                }
            }

            if (contact.flags & contact_flags::SIM_ENABLE_HIT_EVENT) != 0 {
                let mut flagged = false;
                let mut manifold_index = 0;
                while manifold_index < lane_manifold_count as usize && !flagged {
                    let m = &contact.manifolds[manifold_index];
                    for mp in &m.points[..m.point_count as usize] {
                        // Need to check total impulse because the point may be speculative and not colliding
                        if mp.normal_velocity < neg_hit_threshold && mp.total_normal_impulse > 0.0 {
                            hit_event_bit_set.set_bit(contact.contact_id as u32);
                            *has_hit_events = true;
                            flagged = true;
                            break;
                        }
                    }
                    manifold_index += 1;
                }
            }
        }
    }
}
