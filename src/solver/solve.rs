//! Serial solve driver from solver.c: prepare → sub-step loop → restitution →
//! store → finalize → broad-phase enlarge → island sleep. Joints and island
//! splitting are not yet wired.
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
use crate::events::BodyMoveEvent;
use crate::id::BodyId;
use crate::math_functions::WORLD_TRANSFORM_IDENTITY;
use crate::shape::shape_flags;
use crate::solver_set::{try_sleep_island, AWAKE_SET};
use crate::world::World;

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
    {
        let task_context = &mut world.task_contexts[0];
        task_context
            .hit_event_bit_set
            .set_bit_count_and_clear(contact_id_capacity as u32);
        task_context.has_hit_events = false;
    }

    if world.split_island_id != NULL_INDEX {
        // Island split deferred with the sleep/island bring-up slice.
        world.split_island_id = NULL_INDEX;
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

        // Warm start: overflow first, then colors
        {
            let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
            warm_start_contacts(&mut color_constraints[OVERFLOW_INDEX as usize], states);
            for color_index in 0..OVERFLOW_INDEX as usize {
                warm_start_contacts(&mut color_constraints[color_index], states);
            }
        }

        for _ in 0..SOLVER_ITERATIONS {
            let use_bias = true;
            let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
            solve_contacts(
                &mut color_constraints[OVERFLOW_INDEX as usize],
                states,
                context,
                use_bias,
            );
            for color_index in 0..OVERFLOW_INDEX as usize {
                solve_contacts(
                    &mut color_constraints[color_index],
                    states,
                    context,
                    use_bias,
                );
            }
        }

        integrate_positions(world, context);

        for _ in 0..RELAX_ITERATIONS {
            let use_bias = false;
            let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;
            solve_contacts(
                &mut color_constraints[OVERFLOW_INDEX as usize],
                states,
                context,
                use_bias,
            );
            for color_index in 0..OVERFLOW_INDEX as usize {
                solve_contacts(
                    &mut color_constraints[color_index],
                    states,
                    context,
                    use_bias,
                );
            }
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
