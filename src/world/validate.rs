// b3ValidateSolverSets from physics_world.c, split out of world/mod.rs to
// keep modules focused.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::world::World;

impl World {
    /// Debug validation of solver set bookkeeping. (b3ValidateSolverSets)
    ///
    /// C gates this behind B3_VALIDATE; here it always runs and asserts in
    /// debug builds (release builds compile the checks away).
    pub fn validate_solver_sets(&self) {
        use crate::body::body_flags;
        use crate::broad_phase::proxy_type;
        use crate::constants::GRAPH_COLOR_COUNT;
        use crate::constraint_graph::OVERFLOW_INDEX;
        use crate::contact::contact_flags;
        use crate::core::NULL_INDEX;
        use crate::solver_set::{AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET, STATIC_SET};
        use crate::types::BodyType;

        debug_assert!(self.body_id_pool.id_capacity() == self.bodies.len() as i32);
        debug_assert!(self.contact_id_pool.id_capacity() == self.contacts.len() as i32);
        debug_assert!(self.joint_id_pool.id_capacity() == self.joints.len() as i32);
        debug_assert!(self.island_id_pool.id_capacity() == self.islands.len() as i32);
        debug_assert!(self.solver_set_id_pool.id_capacity() == self.solver_sets.len() as i32);

        let mut active_set_count = 0;
        let mut total_body_count = 0;
        let mut total_joint_count = 0;
        let mut total_contact_count = 0;
        let mut total_island_count = 0;

        // Validate all solver sets
        for (set_index, set) in self.solver_sets.iter().enumerate() {
            if set.set_index != NULL_INDEX {
                active_set_count += 1;

                if set_index == STATIC_SET as usize {
                    debug_assert!(set.contact_indices.is_empty());
                    debug_assert!(set.island_sims.is_empty());
                    debug_assert!(set.body_states.is_empty());
                } else if set_index == DISABLED_SET as usize {
                    debug_assert!(set.island_sims.is_empty());
                    debug_assert!(set.body_states.is_empty());
                } else if set_index == AWAKE_SET as usize {
                    debug_assert!(set.body_sims.len() == set.body_states.len());
                    debug_assert!(set.joint_sims.is_empty());
                } else {
                    debug_assert!(set.body_states.is_empty());
                }

                // Validate bodies
                {
                    total_body_count += set.body_sims.len() as i32;
                    for (local_index, body_sim) in set.body_sims.iter().enumerate() {
                        let body_id = body_sim.body_id;
                        debug_assert!(0 <= body_id && (body_id as usize) < self.bodies.len());
                        let body = &self.bodies[body_id as usize];
                        debug_assert!(body.set_index == set_index as i32);
                        debug_assert!(body.local_index == local_index as i32);
                        debug_assert!(body.id == body_id);

                        let synced_flags = body.flags & !body_flags::BODY_TRANSIENT_FLAGS;
                        debug_assert!((body_sim.flags & synced_flags) == synced_flags);

                        if set_index == AWAKE_SET as usize {
                            let body_state = &set.body_states[local_index];
                            debug_assert!((body_state.flags & synced_flags) == synced_flags);
                            let _ = body_state;
                        }

                        if body.type_ == BodyType::Dynamic {
                            debug_assert!((body.flags & body_flags::DYNAMIC_FLAG) != 0);
                        }

                        if set_index == DISABLED_SET as usize {
                            debug_assert!(body.head_contact_key == NULL_INDEX);
                        }

                        // Validate body shapes
                        let mut prev_shape_id = NULL_INDEX;
                        let mut shape_id = body.head_shape_id;
                        while shape_id != NULL_INDEX {
                            let shape = &self.shapes[shape_id as usize];
                            debug_assert!(shape.id == shape_id);
                            debug_assert!(shape.prev_shape_id == prev_shape_id);

                            if set_index == DISABLED_SET as usize {
                                debug_assert!(shape.proxy_key == NULL_INDEX);
                            } else if set_index == STATIC_SET as usize {
                                debug_assert!(proxy_type(shape.proxy_key) == BodyType::Static);
                            } else {
                                let proxy_type_ = proxy_type(shape.proxy_key);
                                debug_assert!(
                                    proxy_type_ == BodyType::Kinematic
                                        || proxy_type_ == BodyType::Dynamic
                                );
                                let _ = proxy_type_;
                            }

                            prev_shape_id = shape_id;
                            shape_id = shape.next_shape_id;
                        }

                        // Validate body contacts
                        let mut contact_key = body.head_contact_key;
                        while contact_key != NULL_INDEX {
                            let contact_id = contact_key >> 1;
                            let edge_index = (contact_key & 1) as usize;

                            let contact = &self.contacts[contact_id as usize];
                            debug_assert!(contact.set_index != STATIC_SET);
                            debug_assert!(
                                contact.edges[0].body_id == body_id
                                    || contact.edges[1].body_id == body_id
                            );
                            contact_key = contact.edges[edge_index].next_key;
                        }

                        // Validate body joints
                        let mut joint_key = body.head_joint_key;
                        while joint_key != NULL_INDEX {
                            let joint_id = joint_key >> 1;
                            let edge_index = (joint_key & 1) as usize;

                            let joint = &self.joints[joint_id as usize];

                            let other_edge_index = edge_index ^ 1;
                            let other_body =
                                &self.bodies[joint.edges[other_edge_index].body_id as usize];

                            if set_index == DISABLED_SET as usize
                                || other_body.set_index == DISABLED_SET
                            {
                                debug_assert!(joint.set_index == DISABLED_SET);
                            } else if set_index == STATIC_SET as usize
                                && other_body.set_index == STATIC_SET
                            {
                                debug_assert!(joint.set_index == STATIC_SET);
                            } else if body.type_ != BodyType::Dynamic
                                && other_body.type_ != BodyType::Dynamic
                            {
                                debug_assert!(joint.set_index == STATIC_SET);
                            } else if set_index == AWAKE_SET as usize {
                                debug_assert!(joint.set_index == AWAKE_SET);
                            } else if set_index >= FIRST_SLEEPING_SET as usize {
                                debug_assert!(joint.set_index == set_index as i32);
                            }

                            // (b3GetJointSim)
                            let joint_sim = if joint.set_index == AWAKE_SET {
                                debug_assert!(
                                    0 <= joint.color_index && joint.color_index < GRAPH_COLOR_COUNT
                                );
                                &self.constraint_graph.colors[joint.color_index as usize].joint_sims
                                    [joint.local_index as usize]
                            } else {
                                debug_assert!(joint.color_index == NULL_INDEX);
                                &self.solver_sets[joint.set_index as usize].joint_sims
                                    [joint.local_index as usize]
                            };
                            debug_assert!(joint_sim.joint_id == joint_id);
                            debug_assert!(joint_sim.body_id_a == joint.edges[0].body_id);
                            debug_assert!(joint_sim.body_id_b == joint.edges[1].body_id);
                            let _ = joint_sim;

                            joint_key = joint.edges[edge_index].next_key;
                        }

                        let _ = (body, local_index);
                    }
                }

                // Validate contacts
                {
                    total_contact_count += set.contact_indices.len() as i32;
                    for (local_index, &contact_index) in set.contact_indices.iter().enumerate() {
                        let contact = &self.contacts[contact_index as usize];
                        if set_index == AWAKE_SET as usize {
                            // contact should be non-touching if awake
                            // or it could be this contact hasn't been transferred yet
                            debug_assert!(
                                contact.manifold_count() == 0
                                    || (contact.flags & contact_flags::SIM_STARTED_TOUCHING) != 0
                            );
                        }
                        debug_assert!(contact.set_index == set_index as i32);
                        debug_assert!(contact.color_index == NULL_INDEX);
                        debug_assert!(contact.local_index == local_index as i32);
                        let _ = (contact, local_index);
                    }
                }

                // Validate joints
                {
                    total_joint_count += set.joint_sims.len() as i32;
                    for (local_index, joint_sim) in set.joint_sims.iter().enumerate() {
                        let joint = &self.joints[joint_sim.joint_id as usize];
                        debug_assert!(joint.set_index == set_index as i32);
                        debug_assert!(joint.color_index == NULL_INDEX);
                        debug_assert!(joint.local_index == local_index as i32);
                        let _ = (joint, local_index);
                    }
                }

                // Validate islands
                {
                    total_island_count += set.island_sims.len() as i32;
                    for (local_index, island_sim) in set.island_sims.iter().enumerate() {
                        let island = &self.islands[island_sim.island_id as usize];
                        debug_assert!(island.set_index == set_index as i32);
                        debug_assert!(island.local_index == local_index as i32);
                        let _ = (island, local_index);
                    }
                }
            } else {
                debug_assert!(set.body_sims.is_empty());
                debug_assert!(set.contact_indices.is_empty());
                debug_assert!(set.joint_sims.is_empty());
                debug_assert!(set.island_sims.is_empty());
                debug_assert!(set.body_states.is_empty());
            }
        }

        debug_assert!(active_set_count == self.solver_set_id_pool.id_count());
        debug_assert!(total_body_count == self.body_id_pool.id_count());
        debug_assert!(total_island_count == self.island_id_pool.id_count());

        // Validate constraint graph
        for color_index in 0..GRAPH_COLOR_COUNT {
            let color = &self.constraint_graph.colors[color_index as usize];
            let mut bit_count = 0;

            total_contact_count += color.convex_contacts.len() as i32;
            for (local_index, &contact_id) in color.convex_contacts.iter().enumerate() {
                let contact = &self.contacts[contact_id as usize];
                // contact should be touching in the constraint graph or awaiting transfer to non-touching
                debug_assert!(
                    contact.manifold_count() > 0
                        || (contact.flags
                            & (contact_flags::SIM_STOPPED_TOUCHING | contact_flags::SIM_DISJOINT))
                            != 0
                );
                debug_assert!(contact.set_index == AWAKE_SET);
                debug_assert!(contact.color_index == color_index);
                debug_assert!(contact.local_index == local_index as i32);

                if color_index < OVERFLOW_INDEX {
                    let body_a = &self.bodies[contact.edges[0].body_id as usize];
                    let body_b = &self.bodies[contact.edges[1].body_id as usize];
                    debug_assert!(
                        color.body_set.get_bit(contact.edges[0].body_id as u32)
                            == (body_a.type_ == BodyType::Dynamic)
                    );
                    debug_assert!(
                        color.body_set.get_bit(contact.edges[1].body_id as u32)
                            == (body_b.type_ == BodyType::Dynamic)
                    );

                    bit_count += if body_a.type_ == BodyType::Dynamic {
                        1
                    } else {
                        0
                    };
                    bit_count += if body_b.type_ == BodyType::Dynamic {
                        1
                    } else {
                        0
                    };
                }
                let _ = (contact, local_index);
            }

            total_contact_count += color.contacts.len() as i32;
            for (local_index, spec) in color.contacts.iter().enumerate() {
                let contact = &self.contacts[spec.contact_id as usize];
                // contact should be touching in the constraint graph or awaiting transfer to non-touching
                debug_assert!(
                    contact.manifold_count() > 0
                        || (contact.flags
                            & (contact_flags::SIM_STOPPED_TOUCHING | contact_flags::SIM_DISJOINT))
                            != 0
                );
                debug_assert!(contact.set_index == AWAKE_SET);
                debug_assert!(contact.color_index == color_index);
                debug_assert!(contact.local_index == local_index as i32);

                if color_index < OVERFLOW_INDEX {
                    let body_a = &self.bodies[contact.edges[0].body_id as usize];
                    let body_b = &self.bodies[contact.edges[1].body_id as usize];
                    debug_assert!(
                        color.body_set.get_bit(contact.edges[0].body_id as u32)
                            == (body_a.type_ == BodyType::Dynamic)
                    );
                    debug_assert!(
                        color.body_set.get_bit(contact.edges[1].body_id as u32)
                            == (body_b.type_ == BodyType::Dynamic)
                    );

                    bit_count += if body_a.type_ == BodyType::Dynamic {
                        1
                    } else {
                        0
                    };
                    bit_count += if body_b.type_ == BodyType::Dynamic {
                        1
                    } else {
                        0
                    };
                }
                let _ = (contact, local_index);
            }

            total_joint_count += color.joint_sims.len() as i32;
            for (local_index, joint_sim) in color.joint_sims.iter().enumerate() {
                let joint = &self.joints[joint_sim.joint_id as usize];
                debug_assert!(joint.set_index == AWAKE_SET);
                debug_assert!(joint.color_index == color_index);
                debug_assert!(joint.local_index == local_index as i32);

                if color_index < OVERFLOW_INDEX {
                    let body_a = &self.bodies[joint.edges[0].body_id as usize];
                    let body_b = &self.bodies[joint.edges[1].body_id as usize];
                    debug_assert!(
                        color.body_set.get_bit(joint.edges[0].body_id as u32)
                            == (body_a.type_ == BodyType::Dynamic)
                    );
                    debug_assert!(
                        color.body_set.get_bit(joint.edges[1].body_id as u32)
                            == (body_b.type_ == BodyType::Dynamic)
                    );

                    bit_count += if body_a.type_ == BodyType::Dynamic {
                        1
                    } else {
                        0
                    };
                    bit_count += if body_b.type_ == BodyType::Dynamic {
                        1
                    } else {
                        0
                    };
                }
                let _ = (joint, local_index);
            }

            // Validate the bit population for this graph color
            debug_assert!(bit_count == color.body_set.count_set_bits());
            let _ = bit_count;
        }

        debug_assert!(total_contact_count == self.contact_id_pool.id_count());
        debug_assert!(total_contact_count == self.broad_phase.pair_set.count());

        debug_assert!(total_joint_count == self.joint_id_pool.id_count());

        let _ = (
            active_set_count,
            total_body_count,
            total_joint_count,
            total_contact_count,
            total_island_count,
        );
    }
}
