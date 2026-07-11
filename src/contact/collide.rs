// Narrow-phase collide pass and contact state machine from physics_world.c.
//
// Serial path only (worker_count == 1). Mesh contacts clear through
// update_contact until mesh_contact.c lands.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::contact_flags;
use super::{destroy_contact, update_contact};
use crate::bitset::BitSet;
use crate::body::body_flags;
use crate::constants::{
    speculative_distance, CONTACT_MANIFOLD_COUNT_BUCKETS, CONTACT_RECYCLE_ANGULAR_DISTANCE,
    GRAPH_COLOR_COUNT,
};
use crate::constraint_graph::{add_contact_to_graph, remove_contact_from_graph};
use crate::core::{ctz64, NULL_INDEX};
use crate::events::{ContactBeginTouchEvent, ContactEndTouchEvent};
use crate::id::{ContactId, ShapeId};
use crate::island::{link_contact, unlink_contact};
use crate::math_functions::{
    aabb_overlaps, abs, conjugate, distance_squared, dot, dot_quat, inv_mul_quat,
    inv_mul_world_transforms, length_squared, make_matrix_from_quat, max, min_float, min_int,
    mul_mv, mul_quat, sub, sub_pos, Vec3,
};
use crate::solver_set::{AWAKE_SET, STATIC_SET};
use crate::types::BodyType;
use crate::world::World;

/// Modified cross used by contact recycling (simd.h b3ModifiedCross scalar).
fn modified_cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.y * b.z + a.z * b.y,
        y: a.z * b.x + a.x * b.z,
        z: a.x * b.y + a.y * b.x,
    }
}

/// (b3AddNonTouchingContact)
fn add_non_touching_contact(world: &mut World, contact_id: i32) {
    debug_assert!(world.contacts[contact_id as usize].set_index == AWAKE_SET);
    let local_index = world.solver_sets[AWAKE_SET as usize].contact_indices.len() as i32;
    {
        let contact = &mut world.contacts[contact_id as usize];
        contact.color_index = NULL_INDEX;
        contact.local_index = local_index;
        contact.body_sim_index_a = NULL_INDEX;
        contact.body_sim_index_b = NULL_INDEX;
    }
    world.solver_sets[AWAKE_SET as usize]
        .contact_indices
        .push(contact_id);
}

/// (b3RemoveNonTouchingContact)
fn remove_non_touching_contact(world: &mut World, set_index: i32, local_index: i32) {
    let set = &mut world.solver_sets[set_index as usize];
    let moved_index = set.contact_indices.len() as i32 - 1;
    set.contact_indices.swap_remove(local_index as usize);
    if moved_index != local_index {
        let moved_contact_id = set.contact_indices[local_index as usize];
        let moved = &mut world.contacts[moved_contact_id as usize];
        debug_assert!(moved.set_index == set_index);
        debug_assert!(moved.color_index == NULL_INDEX);
        debug_assert!(moved.local_index == moved_index);
        moved.local_index = local_index;
    }
}

/// Per-contact narrow phase. (b3CollideTask — serial over full range)
fn collide_task(world: &mut World, contact_indices: &[i32], worker_index: i32) {
    let recycle_distance = world.contact_recycle_distance;
    let speculative = speculative_distance();
    let recycle_distance_non_touching = min_float(recycle_distance, speculative);

    for &contact_index in contact_indices {
        debug_assert!((contact_index as usize) < world.contacts.len());
        debug_assert!(world.contacts[contact_index as usize].contact_id == contact_index);

        let shape_id_a = world.contacts[contact_index as usize].shape_id_a;
        let shape_id_b = world.contacts[contact_index as usize].shape_id_b;

        let overlap = aabb_overlaps(
            world.shapes[shape_id_a as usize].fat_aabb,
            world.shapes[shape_id_b as usize].fat_aabb,
        );
        if !overlap {
            world.contacts[contact_index as usize].flags |= contact_flags::SIM_DISJOINT;
            world.contacts[contact_index as usize].flags &= !contact_flags::SIM_TOUCHING;
            world.task_contexts[worker_index as usize]
                .contact_state_bit_set
                .set_bit(contact_index as u32);
            continue;
        }

        let body_id_a = world.shapes[shape_id_a as usize].body_id;
        let body_id_b = world.shapes[shape_id_b as usize].body_id;
        let is_static_a = world.bodies[body_id_a as usize].type_ == BodyType::Static;
        let is_static_b = world.bodies[body_id_b as usize].type_ == BodyType::Static;
        let was_touching =
            (world.contacts[contact_index as usize].flags & contact_flags::SIM_TOUCHING) != 0;
        let is_mesh_contact =
            (world.contacts[contact_index as usize].flags & contact_flags::SIM_MESH_CONTACT) != 0;

        let set_a = world.bodies[body_id_a as usize].set_index;
        let set_b = world.bodies[body_id_b as usize].set_index;
        let local_a = world.bodies[body_id_a as usize].local_index;
        let local_b = world.bodies[body_id_b as usize].local_index;

        if was_touching {
            debug_assert!(set_a == AWAKE_SET || set_a == STATIC_SET);
            debug_assert!(set_b == AWAKE_SET || set_b == STATIC_SET);
        }

        let sim_a = world.solver_sets[set_a as usize].body_sims[local_a as usize];
        let sim_b = world.solver_sets[set_b as usize].body_sims[local_b as usize];
        let transform_a = sim_a.transform;
        let transform_b = sim_b.transform;
        let is_fast =
            (sim_a.flags & body_flags::IS_FAST) != 0 || (sim_b.flags & body_flags::IS_FAST) != 0;

        {
            let contact = &mut world.contacts[contact_index as usize];
            contact.body_sim_index_a = if is_static_a { NULL_INDEX } else { local_a };
            contact.body_sim_index_b = if is_static_b { NULL_INDEX } else { local_b };
        }

        let recycle_tolerance = if was_touching {
            recycle_distance
        } else {
            recycle_distance_non_touching
        };

        let flags = world.contacts[contact_index as usize].flags;
        if (!is_fast || !is_mesh_contact)
            && recycle_distance > 0.0
            && (flags & contact_flags::RELATIVE_TRANSFORM_VALID) != 0
            && (flags & contact_flags::RECYCLE) != 0
        {
            let contact = &world.contacts[contact_index as usize];
            let angle_a = dot_quat(transform_a.q, contact.cached_rotation_a);
            let angle_b = dot_quat(transform_b.q, contact.cached_rotation_b);
            let angular_distance = min_float(angle_a * angle_a, angle_b * angle_b);

            let xf = inv_mul_world_transforms(transform_a, transform_b);
            let xfc = contact.cached_relative_pose;
            let max_extent_a = if is_static_a {
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                }
            } else {
                sim_a.max_extent
            };
            let max_extent_b = if is_static_b {
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                }
            } else {
                sim_b.max_extent
            };
            let max_extent = max(max_extent_a, max_extent_b);
            let dist_squared = distance_squared(xf.p, xfc.p);

            if angular_distance > CONTACT_RECYCLE_ANGULAR_DISTANCE
                && dist_squared < recycle_tolerance * recycle_tolerance
            {
                let distance = dist_squared.sqrt();
                let slack = recycle_tolerance - distance;
                let qr = inv_mul_quat(xfc.q, xf.q);
                let arc = modified_cross(abs(qr.v), max_extent);
                let arc_sq = 4.0 * length_squared(arc);
                if arc_sq < slack * slack {
                    let dq_a = mul_quat(transform_a.q, conjugate(contact.cached_rotation_a));
                    let dq_b = mul_quat(transform_b.q, conjugate(contact.cached_rotation_b));
                    let matrix_a = make_matrix_from_quat(dq_a);
                    let matrix_b = make_matrix_from_quat(dq_b);
                    let dc = sub_pos(sim_b.center, sim_a.center);

                    let manifold_count = contact.manifold_count();
                    for manifold_index in 0..manifold_count as usize {
                        let normal =
                            world.contacts[contact_index as usize].manifolds[manifold_index].normal;
                        let point_count = world.contacts[contact_index as usize].manifolds
                            [manifold_index]
                            .point_count;
                        for point_index in 0..point_count as usize {
                            let mp = &mut world.contacts[contact_index as usize].manifolds
                                [manifold_index]
                                .points[point_index];
                            let r_a = mul_mv(matrix_a, mp.anchor_a);
                            let r_b = mul_mv(matrix_b, mp.anchor_b);
                            let dp = crate::math_functions::add(dc, sub(r_b, r_a));
                            mp.separation = mp.base_separation + dot(dp, normal);
                            mp.persisted = true;
                        }
                    }

                    world.task_contexts[worker_index as usize].recycled_contact_count += 1;
                    let bucket_index =
                        min_int(manifold_count, CONTACT_MANIFOLD_COUNT_BUCKETS as i32 - 1);
                    if bucket_index > 0 {
                        world.task_contexts[worker_index as usize].manifold_counts
                            [(bucket_index - 1) as usize] += 1;
                    }
                    continue;
                }
            }
        }

        {
            let contact = &mut world.contacts[contact_index as usize];
            contact.cached_rotation_a = transform_a.q;
            contact.cached_rotation_b = transform_b.q;
            contact.cached_relative_pose = inv_mul_world_transforms(transform_a, transform_b);
            contact.flags |= contact_flags::RELATIVE_TRANSFORM_VALID;
        }

        let touching = update_contact(
            world,
            worker_index,
            contact_index,
            shape_id_a,
            sim_a.local_center,
            transform_a,
            shape_id_b,
            sim_b.local_center,
            transform_b,
            is_fast,
        );

        let manifold_count = world.contacts[contact_index as usize].manifold_count();
        let bucket_index = min_int(manifold_count, CONTACT_MANIFOLD_COUNT_BUCKETS as i32 - 1);
        if bucket_index > 0 {
            world.task_contexts[worker_index as usize].manifold_counts
                [(bucket_index - 1) as usize] += 1;
        }

        if touching
            && was_touching
            && (world.contacts[contact_index as usize].flags & contact_flags::SIM_MESH_CONTACT) != 0
        {
            let color_index = world.contacts[contact_index as usize].color_index;
            let local_index = world.contacts[contact_index as usize].local_index;
            debug_assert!(color_index != NULL_INDEX);
            debug_assert!(0 <= color_index && color_index < GRAPH_COLOR_COUNT);
            world.constraint_graph.colors[color_index as usize].contacts[local_index as usize]
                .manifold_count = manifold_count as u16;
        }

        if touching && !was_touching {
            world.contacts[contact_index as usize].flags |= contact_flags::SIM_STARTED_TOUCHING;
            world.task_contexts[worker_index as usize]
                .contact_state_bit_set
                .set_bit(contact_index as u32);
        } else if !touching && was_touching {
            world.contacts[contact_index as usize].flags |= contact_flags::SIM_STOPPED_TOUCHING;
            world.task_contexts[worker_index as usize]
                .contact_state_bit_set
                .set_bit(contact_index as u32);
        }

        let manifold_count = world.contacts[contact_index as usize].manifold_count();
        for manifold_index in 0..manifold_count as usize {
            let point_count =
                world.contacts[contact_index as usize].manifolds[manifold_index].point_count;
            for point_index in 0..point_count as usize {
                let mp = &mut world.contacts[contact_index as usize].manifolds[manifold_index]
                    .points[point_index];
                mp.base_separation = mp.separation;
            }
        }
    }
}

/// Narrow-phase collision. (b3Collide)
pub fn collide(world: &mut World, dt: f32) {
    debug_assert!(world.worker_count > 0);

    let mut touching_count = 0;
    for i in 0..GRAPH_COLOR_COUNT as usize {
        let color = &world.constraint_graph.colors[i];
        touching_count += color.convex_contacts.len() + color.contacts.len();
    }

    let non_touching_count = world.solver_sets[AWAKE_SET as usize].contact_indices.len();
    let contact_count = touching_count + non_touching_count;
    if contact_count == 0 {
        return;
    }

    let mut contact_indices = Vec::with_capacity(contact_count);
    for i in 0..GRAPH_COLOR_COUNT as usize {
        let color = &world.constraint_graph.colors[i];
        contact_indices.extend_from_slice(&color.convex_contacts);
        for spec in &color.contacts {
            contact_indices.push(spec.contact_id);
        }
    }
    debug_assert!(contact_indices.len() == touching_count);
    if non_touching_count > 0 {
        contact_indices.extend_from_slice(&world.solver_sets[AWAKE_SET as usize].contact_indices);
    }

    let contact_id_capacity = world.contact_id_pool.id_capacity();
    for task in &mut world.task_contexts {
        task.contact_state_bit_set
            .set_bit_count_and_clear(contact_id_capacity as u32);
        task.sat_call_count = 0;
        task.sat_cache_hit_count = 0;
        task.recycled_contact_count = 0;
        task.manifold_counts = [0; CONTACT_MANIFOLD_COUNT_BUCKETS];
    }

    // Serial: one worker covers the full range.
    collide_task(world, &contact_indices, 0);

    let sat_multiplier = if dt > 0.0 { 1 } else { 0 };
    world.sat_call_count = sat_multiplier * world.task_contexts[0].sat_call_count;
    world.sat_cache_hit_count = sat_multiplier * world.task_contexts[0].sat_cache_hit_count;
    world.manifold_counts = world.task_contexts[0].manifold_counts;

    // Union worker bitsets (serial: only worker 0).
    let bit_set: BitSet = world.task_contexts[0].contact_state_bit_set.clone();
    let end_event_array_index = world.end_event_array_index;
    let world_id = world.world_id;

    for k in 0..bit_set.block_count() {
        let mut bits = bit_set.block(k);
        while bits != 0 {
            let ctz = ctz64(bits);
            let contact_id = (64 * k + ctz) as i32;

            let (shape_id_a, shape_id_b, flags, generation) = {
                let contact = &world.contacts[contact_id as usize];
                debug_assert!(contact.set_index == AWAKE_SET);
                (
                    contact.shape_id_a,
                    contact.shape_id_b,
                    contact.flags,
                    contact.generation,
                )
            };

            let shape_a = &world.shapes[shape_id_a as usize];
            let shape_b = &world.shapes[shape_id_b as usize];
            let shape_id_a_full = ShapeId {
                index1: shape_a.id + 1,
                world0: world_id,
                generation: shape_a.generation,
            };
            let shape_id_b_full = ShapeId {
                index1: shape_b.id + 1,
                world0: world_id,
                generation: shape_b.generation,
            };
            let contact_full_id = ContactId {
                index1: contact_id + 1,
                world0: world_id,
                padding: 0,
                generation,
            };

            if (flags & contact_flags::SIM_DISJOINT) != 0 {
                destroy_contact(world, contact_id, false);
            } else if (flags & contact_flags::SIM_STARTED_TOUCHING) != 0 {
                debug_assert!(world.contacts[contact_id as usize].island_id == NULL_INDEX);

                if (flags & contact_flags::ENABLE_CONTACT_EVENTS) != 0 {
                    world.contact_begin_events.push(ContactBeginTouchEvent {
                        shape_id_a: shape_id_a_full,
                        shape_id_b: shape_id_b_full,
                        contact_id: contact_full_id,
                    });
                }

                debug_assert!(world.contacts[contact_id as usize].manifold_count() > 0);
                debug_assert!(world.contacts[contact_id as usize].set_index == AWAKE_SET);

                world.contacts[contact_id as usize].flags &= !contact_flags::SIM_STARTED_TOUCHING;
                world.contacts[contact_id as usize].flags |= contact_flags::TOUCHING;
                link_contact(world, contact_id);

                debug_assert!(world.contacts[contact_id as usize].color_index == NULL_INDEX);

                let old_local_index = world.contacts[contact_id as usize].local_index;
                add_contact_to_graph(world, contact_id);
                remove_non_touching_contact(world, AWAKE_SET, old_local_index);
            } else if (flags & contact_flags::SIM_STOPPED_TOUCHING) != 0 {
                world.contacts[contact_id as usize].flags &= !contact_flags::SIM_STOPPED_TOUCHING;
                world.contacts[contact_id as usize].flags &= !contact_flags::TOUCHING;

                if (world.contacts[contact_id as usize].flags
                    & contact_flags::ENABLE_CONTACT_EVENTS)
                    != 0
                {
                    world.contact_end_events[end_event_array_index as usize].push(
                        ContactEndTouchEvent {
                            shape_id_a: shape_id_a_full,
                            shape_id_b: shape_id_b_full,
                            contact_id: contact_full_id,
                        },
                    );
                }

                debug_assert!(world.contacts[contact_id as usize].manifolds.is_empty());

                let color_index = world.contacts[contact_id as usize].color_index;
                let local_index = world.contacts[contact_id as usize].local_index;
                let body_id_a = world.contacts[contact_id as usize].edges[0].body_id;
                let body_id_b = world.contacts[contact_id as usize].edges[1].body_id;
                let is_mesh = (world.contacts[contact_id as usize].flags
                    & contact_flags::SIM_MESH_CONTACT)
                    != 0;

                unlink_contact(world, contact_id);
                add_non_touching_contact(world, contact_id);
                remove_contact_from_graph(
                    world,
                    body_id_a,
                    body_id_b,
                    color_index,
                    local_index,
                    is_mesh,
                );
            }

            bits &= bits - 1;
        }
    }

    world.validate_solver_sets();
}
