//! Serial solve driver from solver.c: island split → prepare → sub-step loop →
//! restitution → store → finalize → joint events → hit events → broad-phase
//! enlarge → island sleep.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT
#![allow(clippy::needless_range_loop)]

use super::integrate::{finalize_bodies, integrate_positions, integrate_velocities};
use super::StepContext;
use crate::constants::{GRAPH_COLOR_COUNT, RELAX_ITERATIONS, SOLVER_ITERATIONS};
use crate::constraint_graph::OVERFLOW_INDEX;
use crate::contact_solver::{
    apply_restitution, flag_hit_events, prepare_color_contacts, solve_contacts, store_impulses,
    warm_start_contacts, ContactConstraint,
};
use crate::core::NULL_INDEX;
use crate::events::{BodyMoveEvent, JointEvent};
use crate::id::{BodyId, JointId};
use crate::island::split_island;
use crate::joint::{get_joint_reaction, prepare_joint, solve_joint, warm_start_joint};
use crate::math_functions::WORLD_TRANSFORM_IDENTITY;
use crate::shape::shape_flags;
use crate::solver_set::{try_sleep_island, AWAKE_SET};
use crate::world::World;

/// Solve joints then contacts for overflow + colors. During biased solve,
/// force/torque thresholds set bits in `joint_state_bit_set`. (serial of
/// b3SolveJointsTask / b3SolveContacts + overflow)
fn solve_joints_then_contacts(
    world: &mut World,
    color_constraints: &mut [Vec<ContactConstraint>],
    context: &StepContext,
    use_bias: bool,
) {
    // Overflow first
    {
        let mut joint_sims =
            std::mem::take(&mut world.constraint_graph.colors[OVERFLOW_INDEX as usize].joint_sims);
        {
            let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
            for joint in &mut joint_sims {
                solve_joint(joint, context, states, use_bias);
            }
        }
        if use_bias {
            for joint in &mut joint_sims {
                maybe_flag_joint_reaction(world, joint, context.inv_h);
            }
        }
        world.constraint_graph.colors[OVERFLOW_INDEX as usize].joint_sims = joint_sims;

        let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
        solve_contacts(
            &mut color_constraints[OVERFLOW_INDEX as usize],
            states,
            context,
            use_bias,
        );
    }

    for color_index in 0..OVERFLOW_INDEX as usize {
        let mut joint_sims =
            std::mem::take(&mut world.constraint_graph.colors[color_index].joint_sims);
        {
            let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
            for joint in &mut joint_sims {
                solve_joint(joint, context, states, use_bias);
            }
        }
        if use_bias {
            for joint in &mut joint_sims {
                maybe_flag_joint_reaction(world, joint, context.inv_h);
            }
        }
        world.constraint_graph.colors[color_index].joint_sims = joint_sims;

        let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
        solve_contacts(
            &mut color_constraints[color_index],
            states,
            context,
            use_bias,
        );
    }
}

fn maybe_flag_joint_reaction(
    world: &mut World,
    joint: &crate::joint::JointSim,
    inv_h: f32,
) {
    if !(joint.force_threshold < f32::MAX || joint.torque_threshold < f32::MAX) {
        return;
    }
    if world.task_contexts[0]
        .joint_state_bit_set
        .get_bit(joint.joint_id as u32)
    {
        return;
    }

    let (force, torque) = get_joint_reaction(world, joint, inv_h);

    // Check thresholds. A zero threshold means all awake joints get reported.
    if force >= joint.force_threshold || torque >= joint.torque_threshold {
        world.task_contexts[0]
            .joint_state_bit_set
            .set_bit(joint.joint_id as u32);
    }
}

/// Solve with graph coloring. (b3Solve — serial)
///
/// Does not increment `world.step_index` — World::step owns that.
pub fn solve(world: &mut World, context: &StepContext) {
    let awake_body_count = world.solver_sets[AWAKE_SET as usize].body_sims.len();
    if awake_body_count == 0 {
        world.broad_phase.validate_no_enlarged();
        return;
    }

    world.body_move_events.resize(
        awake_body_count,
        BodyMoveEvent {
            user_data: 0,
            transform: WORLD_TRANSFORM_IDENTITY,
            body_id: BodyId::default(),
            fell_asleep: false,
        },
    );

    let contact_id_capacity = world.contact_id_pool.id_capacity();
    let joint_id_capacity = world.joint_id_pool.id_capacity();
    {
        let task_context = &mut world.task_contexts[0];
        task_context
            .hit_event_bit_set
            .set_bit_count_and_clear(contact_id_capacity as u32);
        task_context.has_hit_events = false;
        task_context
            .joint_state_bit_set
            .set_bit_count_and_clear(joint_id_capacity as u32);
    }

    // Split an awake island. This modifies:
    // - world island array and solver set
    // - island indices on bodies, contacts, and joints
    // C runs this as a task in parallel with the constraint solve (it cannot
    // overlap FinalizeBodies); the serial port runs it here, at the point
    // where C enqueues it. (b3SplitIslandTask)
    if world.split_island_id != NULL_INDEX {
        split_island(world, world.split_island_id);
        world.split_island_id = NULL_INDEX;
    }

    // Prepare joints for every color (incl. overflow) before contacts.
    // mem::take avoids &World + &mut joint_sims from the same World.
    for color_index in 0..GRAPH_COLOR_COUNT as usize {
        let mut joint_sims =
            std::mem::take(&mut world.constraint_graph.colors[color_index].joint_sims);
        for joint in &mut joint_sims {
            prepare_joint(world, joint, context);
        }
        world.constraint_graph.colors[color_index].joint_sims = joint_sims;
    }

    // Prepare contact constraints for every color (convex + mesh → Mesh kernels).
    let mut color_constraints: Vec<Vec<ContactConstraint>> =
        (0..GRAPH_COLOR_COUNT as usize).map(|_| Vec::new()).collect();

    for color_index in 0..GRAPH_COLOR_COUNT as usize {
        let convex_ids = world.constraint_graph.colors[color_index]
            .convex_contacts
            .clone();
        let mesh_specs = world.constraint_graph.colors[color_index].contacts.clone();
        let contacts = &world.contacts;
        let sims = &world.solver_sets[AWAKE_SET as usize].body_sims;
        let states = &world.solver_sets[AWAKE_SET as usize].body_states;
        prepare_color_contacts(
            &mut color_constraints[color_index],
            &convex_ids,
            &mesh_specs,
            contacts,
            sims,
            states,
            context,
        );
    }

    // Sub-step loop
    let sub_step_count = context.sub_step_count;
    for _sub_step_index in 0..sub_step_count {
        integrate_velocities(world, context);

        // Warm start: joints then contacts; overflow first, then colors.
        // constraint_graph and solver_sets are distinct World fields.
        {
            let World {
                constraint_graph,
                solver_sets,
                ..
            } = world;
            let states = &mut solver_sets[AWAKE_SET as usize].body_states;
            for joint in &mut constraint_graph.colors[OVERFLOW_INDEX as usize].joint_sims {
                warm_start_joint(joint, states);
            }
            warm_start_contacts(&mut color_constraints[OVERFLOW_INDEX as usize], states);
            for color_index in 0..OVERFLOW_INDEX as usize {
                for joint in &mut constraint_graph.colors[color_index].joint_sims {
                    warm_start_joint(joint, states);
                }
                warm_start_contacts(&mut color_constraints[color_index], states);
            }
        }

        for _ in 0..SOLVER_ITERATIONS {
            let use_bias = true;
            solve_joints_then_contacts(world, &mut color_constraints, context, use_bias);
        }

        integrate_positions(world, context);

        for _ in 0..RELAX_ITERATIONS {
            let use_bias = false;
            solve_joints_then_contacts(world, &mut color_constraints, context, use_bias);
        }
    }

    // Restitution
    {
        let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
        apply_restitution(
            &mut color_constraints[OVERFLOW_INDEX as usize],
            states,
            context,
        );
        for color_index in 0..OVERFLOW_INDEX as usize {
            apply_restitution(&mut color_constraints[color_index], states, context);
        }
    }

    // Store impulses (overflow without hit-event flagging, colors with)
    {
        store_impulses(
            &color_constraints[OVERFLOW_INDEX as usize],
            &mut world.contacts,
        );

        for color_index in 0..OVERFLOW_INDEX as usize {
            store_impulses(&color_constraints[color_index], &mut world.contacts);
        }

        let neg_hit_threshold = -world.hit_event_threshold;
        let mut has_hit = world.task_contexts[0].has_hit_events;
        for color_index in 0..OVERFLOW_INDEX as usize {
            flag_hit_events(
                &color_constraints[color_index],
                &world.contacts,
                &mut world.task_contexts[0].hit_event_bit_set,
                &mut has_hit,
                neg_hit_threshold,
            );
        }
        world.task_contexts[0].has_hit_events = has_hit;
    }

    // Finalize bodies
    {
        let awake_island_count = world.solver_sets[AWAKE_SET as usize].island_sims.len();
        let task_context = &mut world.task_contexts[0];
        task_context.sensor_hits.clear();
        task_context
            .enlarged_sim_bit_set
            .set_bit_count_and_clear(awake_body_count as u32);
        task_context
            .awake_island_bit_set
            .set_bit_count_and_clear(awake_island_count as u32);
        task_context.split_island_id = NULL_INDEX;
        task_context.split_sleep_time = 0.0;
    }

    finalize_bodies(world, context);

    // Report joint events (C block between finalize and hit events).
    {
        debug_assert!(world.joint_events.is_empty());
        let world_id = world.world_id;
        let word_count = world.task_contexts[0].joint_state_bit_set.block_count();
        for k in 0..word_count {
            let mut word = world.task_contexts[0].joint_state_bit_set.block(k);
            while word != 0 {
                let ctz = word.trailing_zeros();
                let joint_id = (64 * k + ctz) as i32;

                debug_assert!((joint_id as usize) < world.joints.len());

                let joint = &world.joints[joint_id as usize];
                debug_assert!(joint.set_index == AWAKE_SET);

                world.joint_events.push(JointEvent {
                    joint_id: JointId {
                        index1: joint_id + 1,
                        world0: world_id,
                        generation: joint.generation,
                    },
                    user_data: joint.user_data,
                });

                word &= word - 1;
            }
        }
    }

    // Report hit events flagged during store impulses. C runs this after the
    // joint events pass.
    {
        use crate::events::ContactHitEvent;
        use crate::id::{ContactId, ShapeId};
        use crate::math_functions::{lerp, lerp_position, offset_pos, VEC3_ZERO};

        debug_assert!(world.contact_hit_events.is_empty());

        // Fast path: if no worker flagged any hit-event candidates during
        // b2StoreImpulsesTask, skip entirely.
        if world.task_contexts[0].has_hit_events {
            let threshold = world.hit_event_threshold;
            let world_id = world.world_id;

            let word_count = world.task_contexts[0].hit_event_bit_set.block_count();
            for k in 0..word_count {
                let mut word = world.task_contexts[0].hit_event_bit_set.block(k);
                while word != 0 {
                    let ctz = word.trailing_zeros();
                    let contact_id = (64 * k + ctz) as i32;

                    let contact = &world.contacts[contact_id as usize];
                    debug_assert!(
                        contact.set_index == AWAKE_SET && contact.color_index != NULL_INDEX
                    );

                    let shape_a = &world.shapes[contact.shape_id_a as usize];
                    let shape_b = &world.shapes[contact.shape_id_b as usize];
                    let body_a = &world.bodies[shape_a.body_id as usize];
                    let body_b = &world.bodies[shape_b.body_id as usize];
                    // (b3GetBodySim)
                    let sim_a = &world.solver_sets[body_a.set_index as usize].body_sims
                        [body_a.local_index as usize];
                    let sim_b = &world.solver_sets[body_b.set_index as usize].body_sims
                        [body_b.local_index as usize];
                    let mid_center = lerp_position(sim_a.center, sim_b.center, 0.5);

                    let mut approach_speed = threshold;
                    let mut point = mid_center;
                    let mut normal = VEC3_ZERO;
                    let mut found = false;
                    let mut triangle_index = 0;
                    for manifold in &contact.manifolds {
                        for p in 0..manifold.point_count as usize {
                            let mp = &manifold.points[p];
                            let mp_approach_speed = -mp.normal_velocity;

                            // Need to check total impulse because the point may be speculative and not colliding
                            if mp_approach_speed > approach_speed && mp.total_normal_impulse > 0.0
                            {
                                approach_speed = mp_approach_speed;
                                point = offset_pos(mid_center, lerp(mp.anchor_a, mp.anchor_b, 0.5));
                                normal = manifold.normal;
                                triangle_index = mp.triangle_index;
                                found = true;
                            }
                        }
                    }

                    if found {
                        let event = ContactHitEvent {
                            shape_id_a: ShapeId {
                                index1: shape_a.id + 1,
                                world0: world_id,
                                generation: shape_a.generation,
                            },
                            shape_id_b: ShapeId {
                                index1: shape_b.id + 1,
                                world0: world_id,
                                generation: shape_b.generation,
                            },
                            contact_id: ContactId {
                                index1: contact.contact_id + 1,
                                world0: world_id,
                                padding: 0,
                                generation: contact.generation,
                            },
                            point,
                            normal,
                            approach_speed,
                            // shapeB is never a compound today (asserted in b3CreateContact), so the
                            // childIndex argument is irrelevant for it. shapeA carries the compound.
                            user_material_id_a: shape_a
                                .get_shape_user_material_id(contact.child_index, triangle_index),
                            user_material_id_b: shape_b
                                .get_shape_user_material_id(0, triangle_index),
                        };
                        world.contact_hit_events.push(event);
                    }

                    word &= word - 1;
                }
            }
        }
    }

    // Enlarge broad-phase proxies for shapes whose fat AABB grew.
    {
        world.broad_phase.validate_no_enlarged();

        let word_count = world.task_contexts[0].enlarged_sim_bit_set.block_count();
        for k in 0..word_count {
            let mut word = world.task_contexts[0].enlarged_sim_bit_set.block(k);
            while word != 0 {
                let ctz = word.trailing_zeros();
                let body_sim_index = (64 * k + ctz) as usize;

                let body_id = world.solver_sets[AWAKE_SET as usize].body_sims[body_sim_index].body_id;

                let mut shape_id = world.bodies[body_id as usize].head_shape_id;
                while shape_id != NULL_INDEX {
                    if (world.shapes[shape_id as usize].flags & shape_flags::ENLARGED_AABB) != 0 {
                        let proxy_key = world.shapes[shape_id as usize].proxy_key;
                        let fat_aabb = world.shapes[shape_id as usize].fat_aabb;
                        world.broad_phase.enlarge_proxy(proxy_key, fat_aabb);
                        world.shapes[shape_id as usize].flags &= !shape_flags::ENLARGED_AABB;
                    }
                    shape_id = world.shapes[shape_id as usize].next_shape_id;
                }

                word &= word - 1;
            }
        }

        world.broad_phase.validate();
    }

    // Island sleeping
    // This must be done last because putting islands to sleep invalidates the enlarged body bits.
    if world.enable_sleep {
        // Collect split island candidate for the next time step. No need to split if sleeping is disabled.
        debug_assert!(world.split_island_id == NULL_INDEX);
        let mut split_sleep_timer = 0.0f32;
        {
            let task_context = &world.task_contexts[0];
            if task_context.split_island_id != NULL_INDEX
                && task_context.split_sleep_time >= split_sleep_timer
            {
                debug_assert!(task_context.split_sleep_time > 0.0);

                // Tie breaking for determinism. Largest island id wins. C needs this
                // due to work stealing across workers; kept for the serial port so a
                // multi-worker build later cannot change the outcome.
                let tied_but_smaller = task_context.split_sleep_time == split_sleep_timer
                    && task_context.split_island_id < world.split_island_id;
                if !tied_but_smaller {
                    world.split_island_id = task_context.split_island_id;
                    split_sleep_timer = task_context.split_sleep_time;
                }
            }
        }
        let _ = split_sleep_timer;

        // Need to process in reverse because this moves islands to sleeping solver sets.
        let count = world.solver_sets[AWAKE_SET as usize].island_sims.len();
        for island_index in (0..count).rev() {
            if world.task_contexts[0]
                .awake_island_bit_set
                .get_bit(island_index as u32)
            {
                // this island is still awake
                continue;
            }

            let island_id =
                world.solver_sets[AWAKE_SET as usize].island_sims[island_index].island_id;

            try_sleep_island(world, island_id);
        }

        world.validate_solver_sets();
    }
}
