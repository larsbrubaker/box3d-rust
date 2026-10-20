//! Serial solve driver from solver.c: island split → prepare → sub-step loop (push, relax
//! with deferred restitution) → store → finalize → joint events → hit events →
//! broad-phase enlarge → island sleep.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT
#![allow(clippy::needless_range_loop)]

use super::continuous::solve_continuous;
use super::integrate::{finalize_bodies, integrate_positions, integrate_velocities};
use super::mesh_lanes::compute_mesh_lanes;
use super::StepContext;
use crate::body::body_flags;
use crate::broad_phase::{proxy_id, proxy_type};
use crate::constants::{GRAPH_COLOR_COUNT, RELAX_ITERATIONS, SOLVER_ITERATIONS};
use crate::constraint_graph::OVERFLOW_INDEX;
use crate::contact_solver::wide::{
    apply_restitution_w, prepare_contacts_convex, prepare_contacts_mesh_wide, push_contacts_w,
    solve_contacts_w, store_impulses_w, warm_start_contacts_w, WideContactConstraints,
};
use crate::contact_solver::{
    apply_restitution, flag_hit_events, prepare_mesh_contacts, push_contacts, solve_contacts,
    store_impulses, warm_start_contacts, ContactConstraint,
};
use crate::core::NULL_INDEX;
use crate::events::{BodyMoveEvent, JointEvent};
use crate::id::{BodyId, JointId};
use crate::island::split_island;
use crate::joint::{get_joint_reaction, prepare_joint, solve_joint, warm_start_joint};
use crate::math_functions::{aabb_contains, WORLD_TRANSFORM_IDENTITY};
use crate::solver_set::{try_sleep_island, AWAKE_SET};
use crate::types::BodyType;
use crate::world::World;

/// Solve joints then contacts for overflow + colors. During biased solve,
/// force/torque thresholds set bits in `joint_state_bit_set`. (serial of
/// b3SolveJointsTask / b3PushContacts_* when `use_bias`, else b3SolveContacts_*, + overflow)
fn solve_joints_then_contacts(
    world: &mut World,
    overflow_constraints: &mut [ContactConstraint],
    wide_constraints: &mut [WideContactConstraints],
    mesh_wide_constraints: &mut [WideContactConstraints],
    context: &StepContext,
    use_bias: bool,
) {
    // Overflow first — always the scalar Mesh kernels (C: b3SolveContacts_Overflow)
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
        if use_bias {
            // (b3PushContacts_Overflow)
            push_contacts(overflow_constraints, states, context);
        } else {
            // (b3SolveContacts_Overflow)
            solve_contacts(overflow_constraints, states, context);
        }
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
        let convex = &mut wide_constraints[color_index];
        let mesh = &mut mesh_wide_constraints[color_index];
        // C graph blocks: joints → wide/convex → mesh (b3_graphContactBlock)
        if use_bias {
            // (b3PushContacts_Convex, b3PushContacts_MeshWide)
            push_contacts_w(convex, states, context);
            push_contacts_w(mesh, states, context);
        } else {
            // (b3SolveContacts_Convex, b3SolveContacts_MeshWide)
            solve_contacts_w(convex, states, context);
            solve_contacts_w(mesh, states, context);
        }
    }
}

fn maybe_flag_joint_reaction(world: &mut World, joint: &crate::joint::JointSim, inv_h: f32) {
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

    // Prepare contact constraints. Graph colors get wide constraints for the convex contacts
    // and for the mesh contacts, the overflow color gets scalar constraints.
    // (b3PrepareContacts_Convex / b3PrepareContacts_MeshWide / b3PrepareContacts_Overflow)
    // C dispatches the wide kernels on world->simdWidth (4 or 8). The scalar port only has
    // the width 4 kernels, and the world width can never be 8 here (b3SetSIMDWidth and
    // b3World_EnableSSE2Fallback fall back to 4 without AVX2).
    debug_assert!(world.simd_width == crate::core::SIMD_WIDTH as i32);
    let simd_shift = 2;
    let color_count = OVERFLOW_INDEX as usize;
    let mut overflow_constraints: Vec<ContactConstraint> = Vec::new();
    let mut wide_constraints: Vec<WideContactConstraints> = (0..color_count)
        .map(|_| WideContactConstraints::default())
        .collect();
    let mut mesh_wide_constraints: Vec<WideContactConstraints> = (0..color_count)
        .map(|_| WideContactConstraints::default())
        .collect();
    let mut any_restitution = false;

    // Sort mesh contacts by manifold count so that contacts with more manifolds are solved
    // together keeping the SIMD lanes fuller and less ragged.
    let mesh_lanes = {
        let color_specs: Vec<&[crate::contact::ContactSpec]> = world.constraint_graph.colors
            [..color_count]
            .iter()
            .map(|color| color.contacts.as_slice())
            .collect();
        compute_mesh_lanes(&color_specs, simd_shift)
    };

    for color_index in 0..color_count {
        let convex_ids = world.constraint_graph.colors[color_index]
            .convex_contacts
            .clone();
        let mesh_specs = world.constraint_graph.colors[color_index].contacts.clone();
        let contacts = &mut world.contacts;
        let sims = &world.solver_sets[AWAKE_SET as usize].body_sims;
        let states = &world.solver_sets[AWAKE_SET as usize].body_states;
        let wide_restitution = prepare_contacts_convex(
            &mut wide_constraints[color_index],
            &convex_ids,
            contacts,
            sims,
            states,
            context.enable_warm_starting,
        );
        let lanes = &mesh_lanes[color_index];
        let mesh_restitution = prepare_contacts_mesh_wide(
            &mut mesh_wide_constraints[color_index],
            &mesh_specs,
            &lanes.order,
            &lanes.manifold_starts,
            contacts,
            sims,
            states,
            context.enable_warm_starting,
        );
        any_restitution = any_restitution || wide_restitution || mesh_restitution;
    }

    // (b3PrepareContacts_Overflow)
    {
        let overflow = &world.constraint_graph.colors[OVERFLOW_INDEX as usize];
        debug_assert!(overflow.convex_contacts.is_empty());
        let overflow_specs = overflow.contacts.clone();
        let contacts = &mut world.contacts;
        let sims = &world.solver_sets[AWAKE_SET as usize].body_sims;
        let states = &world.solver_sets[AWAKE_SET as usize].body_states;
        let overflow_restitution = prepare_mesh_contacts(
            &mut overflow_constraints,
            &overflow_specs,
            contacts,
            sims,
            states,
            context,
        );
        any_restitution = any_restitution || overflow_restitution;
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
            warm_start_contacts(&mut overflow_constraints, states);
            for color_index in 0..OVERFLOW_INDEX as usize {
                for joint in &mut constraint_graph.colors[color_index].joint_sims {
                    warm_start_joint(joint, states);
                }
                // (b3WarmStartContacts_Convex, b3WarmStartContacts_MeshWide)
                warm_start_contacts_w(&wide_constraints[color_index], states);
                warm_start_contacts_w(&mesh_wide_constraints[color_index], states);
            }
        }

        for _ in 0..SOLVER_ITERATIONS {
            let use_bias = true;
            solve_joints_then_contacts(
                world,
                &mut overflow_constraints,
                &mut wide_constraints,
                &mut mesh_wide_constraints,
                context,
                use_bias,
            );
        }

        integrate_positions(world, context);

        for _ in 0..RELAX_ITERATIONS {
            let use_bias = false;
            solve_joints_then_contacts(
                world,
                &mut overflow_constraints,
                &mut wide_constraints,
                &mut mesh_wide_constraints,
                context,
                use_bias,
            );
        }
    }

    // Restitution. Overflow first, then the colors: wide blocks then mesh blocks.
    // (b3_stageRestitution)
    let restitution_iterations = world.restitution_iterations;
    if restitution_iterations > 0 && any_restitution {
        let threshold = world.restitution_threshold;
        let propagate = world.enable_restitution_propagation;
        let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
        for _ in 0..restitution_iterations {
            // (b3ApplyRestitution_Overflow)
            apply_restitution(
                &mut overflow_constraints,
                states,
                context,
                threshold,
                propagate,
            );

            for color_index in 0..OVERFLOW_INDEX as usize {
                // (b3ApplyRestitution_Convex)
                apply_restitution_w(
                    &mut wide_constraints[color_index],
                    states,
                    context,
                    threshold,
                    propagate,
                );
                // (b3ApplyRestitution_MeshWide)
                apply_restitution_w(
                    &mut mesh_wide_constraints[color_index],
                    states,
                    context,
                    threshold,
                    propagate,
                );
            }
        }
    }

    // Store impulses. C stores the overflow contacts with b3StoreImpulses_Overflow (the Mesh
    // kernel, which also flags hit events) and the colors in the store stages
    // (b3StoreImpulses_Convex, b3StoreImpulses_MeshWide).
    {
        let neg_hit_threshold = -world.hit_event_threshold;
        let mut has_hit = world.task_contexts[0].has_hit_events;

        store_impulses(&overflow_constraints, &mut world.contacts);
        flag_hit_events(
            &overflow_constraints,
            &world.contacts,
            &mut world.task_contexts[0].hit_event_bit_set,
            &mut has_hit,
            neg_hit_threshold,
        );

        for color_index in 0..OVERFLOW_INDEX as usize {
            store_impulses_w(
                &wide_constraints[color_index],
                &mut world.contacts,
                &mut world.task_contexts[0].hit_event_bit_set,
                &mut has_hit,
                neg_hit_threshold,
            );
            store_impulses_w(
                &mesh_wide_constraints[color_index],
                &mut world.contacts,
                &mut world.task_contexts[0].hit_event_bit_set,
                &mut has_hit,
                neg_hit_threshold,
            );
        }

        world.task_contexts[0].has_hit_events = has_hit;
    }

    // Finalize bodies (CCD for non-bullets; bullets queued)
    let mut bullet_bodies: Vec<i32> = Vec::with_capacity(awake_body_count);
    {
        let awake_island_count = world.solver_sets[AWAKE_SET as usize].island_sims.len();
        let task_context = &mut world.task_contexts[0];
        task_context.sensor_hits.clear();
        task_context
            .awake_island_bit_set
            .set_bit_count_and_clear(awake_island_count as u32);
        task_context.split_island_id = NULL_INDEX;
        task_context.split_sleep_time = 0.0;
    }

    // Finish the user tree task that was queued earlier in the time step. This must be
    // complete before touching the broad-phase. The serial port already ran the tree task
    // inline in update_broad_phase_pairs, so only the validation remains.
    world.broad_phase.validate_no_moved();

    // Finalize bodies. Must happen after the constraint solver and after island splitting.
    finalize_bodies(world, context, &mut bullet_bodies);

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
                            if mp_approach_speed > approach_speed && mp.total_normal_impulse > 0.0 {
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

    // Refit BVH
    {
        world.broad_phase.trees[BodyType::Kinematic as usize].refit();
        world.broad_phase.trees[BodyType::Dynamic as usize].refit();

        world.broad_phase.validate();
    }

    // Fast bullet bodies. Note: a bullet body may be moving slow
    if !bullet_bodies.is_empty() {
        for &sim_index in &bullet_bodies {
            solve_continuous(world, sim_index, context.dt);
        }

        // Serially enlarge broad-phase proxies for bullet shapes
        let dynamic_tree = BodyType::Dynamic as usize;

        // This loop has non-deterministic order but it shouldn't affect the result
        for &sim_index in &bullet_bodies {
            let bullet_sim =
                &mut world.solver_sets[AWAKE_SET as usize].body_sims[sim_index as usize];
            if (bullet_sim.flags & body_flags::ENLARGE_BULLET_BOUNDS) == 0 {
                continue;
            }

            // Clear flag
            bullet_sim.flags &= !body_flags::ENLARGE_BULLET_BOUNDS;

            let body_id = bullet_sim.body_id;
            debug_assert!(0 <= body_id && (body_id as usize) < world.bodies.len());

            let mut shape_id = world.bodies[body_id as usize].head_shape_id;
            while shape_id != NULL_INDEX {
                let proxy_key = world.shapes[shape_id as usize].proxy_key;
                let proxy_id_ = proxy_id(proxy_key);
                debug_assert!(proxy_type(proxy_key) == BodyType::Dynamic);

                let tree_aabb = world.broad_phase.trees[dynamic_tree].aabb(proxy_id_);
                let fat_aabb = world.fat_aabbs[shape_id as usize];

                if !aabb_contains(tree_aabb, fat_aabb) {
                    world.broad_phase.trees[dynamic_tree].enlarge_proxy(proxy_id_, fat_aabb);
                }

                shape_id = world.shapes[shape_id as usize].next_shape_id;
            }
        }
    }

    // Report sensor hits. This may include bullet sensor hits.
    {
        let mut drained: Vec<(i32, i32, u16)> = Vec::new();
        for task_context in &world.task_contexts {
            for hit in &task_context.sensor_hits {
                let sensor_index = world.shapes[hit.sensor_id as usize].sensor_index;
                let generation = world.shapes[hit.visitor_id as usize].generation;
                drained.push((sensor_index, hit.visitor_id, generation));
            }
        }
        for (sensor_index, visitor_id, generation) in drained {
            world.sensors[sensor_index as usize]
                .hits
                .push(crate::sensor::Visitor {
                    shape_id: visitor_id,
                    generation,
                });
        }
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
