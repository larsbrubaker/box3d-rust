//! Body integration and finalization from solver.c.
//! Serial loops over the whole awake body range (one worker, one block).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::continuous::solve_continuous;
use super::StepContext;
use crate::body::body_flags;
use crate::constants::{speculative_distance, MAX_ROTATION, TIME_TO_SLEEP};
use crate::core::NULL_INDEX;
use crate::events::BodyMoveEvent;
use crate::id::BodyId;
use crate::math_functions::{
    aabb_contains, abs, add, blend2, dot, integrate_rotation, inv_rotate_vector, invert_matrix,
    is_valid_vec3, length, make_matrix_from_quat, max_float, modified_cross, mul_add, mul_mm,
    mul_mv, mul_quat, mul_sv, neg, normalize_quat, offset_pos, rotate_vector, solve3, sub,
    transpose, Matrix3, Vec3, QUAT_IDENTITY, VEC3_ZERO,
};
use crate::shape::shape_flags;
use crate::solver_set::AWAKE_SET;
use crate::types::BodyType;
use crate::world::World;

/// Integrate velocities, apply damping, and gyroscopic torque.
/// (b3IntegrateVelocitiesTask)
pub(super) fn integrate_velocities(world: &mut World, context: &StepContext) {
    let gravity = world.gravity;
    let h = context.h;

    let set = &mut world.solver_sets[AWAKE_SET as usize];
    let sims = &set.body_sims;
    let states = &mut set.body_states;

    for i in 0..states.len() {
        let sim = &sims[i];
        let state = &mut states[i];

        let mut v = state.linear_velocity;
        let mut w = state.angular_velocity;

        let linear_damping = 1.0 / (1.0 + h * sim.linear_damping);
        let angular_damping = 1.0 / (1.0 + h * sim.angular_damping);

        let gravity_scale = if sim.inv_mass > 0.0 {
            sim.gravity_scale
        } else {
            0.0
        };

        let linear_velocity_delta = blend2(h * sim.inv_mass, sim.force, h * gravity_scale, gravity);
        v = mul_add(linear_velocity_delta, linear_damping, v);

        let angular_velocity_delta = mul_sv(h, mul_mv(sim.inv_inertia_world, sim.torque));
        w = mul_add(angular_velocity_delta, angular_damping, w);

        // Gyroscopic torque — one Newton iteration (matches C).
        {
            let q0 = sim.transform.q;
            let q = mul_quat(state.delta_rotation, q0);

            let inertia_local = invert_matrix(sim.inv_inertia_local);

            let omega1 = inv_rotate_vector(q, w);
            let mut omega2 = omega1;

            let i00 = inertia_local.cx.x;
            let i01 = inertia_local.cy.x;
            let i02 = inertia_local.cz.x;
            let i11 = inertia_local.cy.y;
            let i12 = inertia_local.cz.y;
            let i22 = inertia_local.cz.z;

            for _gyro_iteration in 0..1 {
                let w1 = omega2.x;
                let w2 = omega2.y;
                let w3 = omega2.z;

                let iw1 = i00 * w1 + i01 * w2 + i02 * w3;
                let iw2 = i01 * w1 + i11 * w2 + i12 * w3;
                let iw3 = i02 * w1 + i12 * w2 + i22 * w3;

                let dw = sub(omega2, omega1);
                let b = Vec3 {
                    x: i00 * dw.x + i01 * dw.y + i02 * dw.z + h * (w2 * iw3 - w3 * iw2),
                    y: i01 * dw.x + i11 * dw.y + i12 * dw.z + h * (w3 * iw1 - w1 * iw3),
                    z: i02 * dw.x + i12 * dw.y + i22 * dw.z + h * (w1 * iw2 - w2 * iw1),
                };

                let j = Matrix3 {
                    cx: Vec3 {
                        x: i00 + h * (w2 * i02 - w3 * i01),
                        y: i01 + h * (w3 * i00 - w1 * i02 - iw3),
                        z: i02 + h * (w1 * i01 - w2 * i00 + iw2),
                    },
                    cy: Vec3 {
                        x: i01 + h * (w2 * i12 - w3 * i11 + iw3),
                        y: i11 + h * (w3 * i01 - w1 * i12),
                        z: i12 + h * (w1 * i11 - w2 * i01 - iw1),
                    },
                    cz: Vec3 {
                        x: i02 + h * (w2 * i22 - w3 * i12 - iw2),
                        y: i12 + h * (w3 * i02 - w1 * i22 + iw1),
                        z: i22 + h * (w1 * i12 - w2 * i02),
                    },
                };

                omega2 = sub(omega2, solve3(j, b));
            }

            w = rotate_vector(q, omega2);
        }

        state.linear_velocity = v;
        state.angular_velocity = w;
    }
}

/// (b3IntegratePositionsTask)
pub(super) fn integrate_positions(world: &mut World, context: &StepContext) {
    let h = context.h;
    let max_linear_speed = context.max_linear_velocity;
    let max_angular_speed = MAX_ROTATION * context.inv_dt;
    let max_linear_speed_squared = max_linear_speed * max_linear_speed;
    let max_angular_speed_squared = max_angular_speed * max_angular_speed;

    let states = &mut world.solver_sets[AWAKE_SET as usize].body_states;

    for state in states.iter_mut() {
        let mut v = state.linear_velocity;
        let mut w = state.angular_velocity;

        if (state.flags & body_flags::LOCK_LINEAR_X) != 0 {
            v.x = 0.0;
        }
        if (state.flags & body_flags::LOCK_LINEAR_Y) != 0 {
            v.y = 0.0;
        }
        if (state.flags & body_flags::LOCK_LINEAR_Z) != 0 {
            v.z = 0.0;
        }
        if (state.flags & body_flags::LOCK_ANGULAR_X) != 0 {
            w.x = 0.0;
        }
        if (state.flags & body_flags::LOCK_ANGULAR_Y) != 0 {
            w.y = 0.0;
        }
        if (state.flags & body_flags::LOCK_ANGULAR_Z) != 0 {
            w.z = 0.0;
        }

        if dot(v, v) > max_linear_speed_squared {
            let ratio = max_linear_speed / length(v);
            v = mul_sv(ratio, v);
            state.flags |= body_flags::IS_SPEED_CAPPED;
        }

        if dot(w, w) > max_angular_speed_squared
            && (state.flags & body_flags::ALLOW_FAST_ROTATION) == 0
        {
            let ratio = max_angular_speed / length(w);
            w = mul_sv(ratio, w);
            state.flags |= body_flags::IS_SPEED_CAPPED;
        }

        state.linear_velocity = v;
        state.angular_velocity = w;
        state.delta_position = mul_add(state.delta_position, h, v);
        state.delta_rotation = integrate_rotation(state.delta_rotation, mul_sv(h, w));
    }
}

/// Finalize awake bodies after the constraint solve, including fast-body / bullet
/// detection for continuous collision. (b3FinalizeBodiesTask)
pub(super) fn finalize_bodies(
    world: &mut World,
    context: &StepContext,
    bullet_bodies: &mut Vec<i32>,
) {
    let enable_sleep = world.enable_sleep;
    let enable_continuous = world.enable_continuous;
    let time_step = context.dt;
    let inv_time_step = context.inv_dt;
    let world_id = world.world_id;
    let speculative = speculative_distance();

    let awake_body_count = world.solver_sets[AWAKE_SET as usize].body_sims.len();
    debug_assert!(awake_body_count <= world.body_move_events.len());

    for sim_index in 0..awake_body_count {
        let mut state = world.solver_sets[AWAKE_SET as usize].body_states[sim_index];
        let mut sim = world.solver_sets[AWAKE_SET as usize].body_sims[sim_index];

        let v = state.linear_velocity;
        let w = state.angular_velocity;
        let local_omega = inv_rotate_vector(sim.transform.q, w);
        let local_delta_rotation = inv_rotate_vector(sim.transform.q, state.delta_rotation.v);

        debug_assert!(is_valid_vec3(v));
        debug_assert!(is_valid_vec3(w));

        sim.center = offset_pos(sim.center, state.delta_position);
        sim.transform.q = normalize_quat(mul_quat(state.delta_rotation, sim.transform.q));

        let velocity_arc = modified_cross(abs(local_omega), sim.max_extent);
        let max_velocity = length(v) + length(velocity_arc);

        let rotation_arc = modified_cross(abs(local_delta_rotation), sim.max_extent);
        let max_delta_position = length(state.delta_position) + 2.0 * length(rotation_arc);

        let position_sleep_factor = 0.5;
        let sleep_velocity = max_float(
            max_velocity,
            position_sleep_factor * inv_time_step * max_delta_position,
        );

        state.delta_position = VEC3_ZERO;
        state.delta_rotation = QUAT_IDENTITY;

        sim.transform.p = offset_pos(
            sim.center,
            neg(rotate_vector(sim.transform.q, sim.local_center)),
        );

        let body_id = sim.body_id;

        {
            let body = &mut world.bodies[body_id as usize];
            body.body_move_index = sim_index as i32;
            body.sleep_velocity = sleep_velocity;
            world.body_move_events[sim_index] = BodyMoveEvent {
                user_data: body.user_data,
                transform: sim.transform,
                body_id: BodyId {
                    index1: body_id + 1,
                    world0: world_id,
                    generation: body.generation,
                },
                fell_asleep: false,
            };
        }

        sim.force = VEC3_ZERO;
        sim.torque = VEC3_ZERO;

        debug_assert!((world.bodies[body_id as usize].flags & body_flags::DIRTY_MASS) == 0);

        {
            let body = &mut world.bodies[body_id as usize];
            body.flags &= !body_flags::BODY_TRANSIENT_FLAGS;
            body.flags |=
                sim.flags & (body_flags::IS_SPEED_CAPPED | body_flags::HAD_TIME_OF_IMPACT);
            body.flags |=
                state.flags & (body_flags::IS_SPEED_CAPPED | body_flags::HAD_TIME_OF_IMPACT);
        }
        sim.flags &= !body_flags::BODY_TRANSIENT_FLAGS;
        state.flags &= !body_flags::BODY_TRANSIENT_FLAGS;

        let body_flags_now = world.bodies[body_id as usize].flags;
        let sleep_threshold = world.bodies[body_id as usize].sleep_threshold;
        let body_type = world.bodies[body_id as usize].type_;

        if !enable_sleep
            || (body_flags_now & body_flags::ENABLE_SLEEP) == 0
            || sleep_velocity > sleep_threshold
        {
            world.bodies[body_id as usize].sleep_time = 0.0;

            let safety_factor = 0.5;
            let max_motion = max_float(max_delta_position, max_velocity * time_step);
            if body_type == BodyType::Dynamic
                && enable_continuous
                && max_motion > safety_factor * sim.min_extent
            {
                // Retained for debug draw; also drives AABB handling below.
                sim.flags |= body_flags::IS_FAST;

                world.solver_sets[AWAKE_SET as usize].body_states[sim_index] = state;
                world.solver_sets[AWAKE_SET as usize].body_sims[sim_index] = sim;

                if (sim.flags & body_flags::IS_BULLET) != 0 {
                    bullet_bodies.push(sim_index as i32);
                } else {
                    solve_continuous(world, sim_index as i32);
                }

                sim = world.solver_sets[AWAKE_SET as usize].body_sims[sim_index];
                state = world.solver_sets[AWAKE_SET as usize].body_states[sim_index];
            } else {
                sim.center0 = sim.center;
                sim.rotation0 = sim.transform.q;
            }
        } else {
            sim.center0 = sim.center;
            sim.rotation0 = sim.transform.q;
            world.bodies[body_id as usize].sleep_time += time_step;
        }

        let rotation_matrix = make_matrix_from_quat(sim.transform.q);
        sim.inv_inertia_world = mul_mm(
            mul_mm(rotation_matrix, sim.inv_inertia_local),
            transpose(rotation_matrix),
        );

        world.solver_sets[AWAKE_SET as usize].body_states[sim_index] = state;
        world.solver_sets[AWAKE_SET as usize].body_sims[sim_index] = sim;

        let sleep_time = world.bodies[body_id as usize].sleep_time;
        let island_id = world.bodies[body_id as usize].island_id;
        if island_id != NULL_INDEX {
            if sleep_time < TIME_TO_SLEEP {
                let island_index = world.islands[island_id as usize].local_index;
                world.task_contexts[0]
                    .awake_island_bit_set
                    .set_bit(island_index as u32);
            } else if world.islands[island_id as usize].constraint_remove_count > 0 {
                let task_context = &mut world.task_contexts[0];
                if sleep_time > task_context.split_sleep_time
                    || (sleep_time == task_context.split_sleep_time
                        && island_id > task_context.split_island_id)
                {
                    task_context.split_island_id = island_id;
                    task_context.split_sleep_time = sleep_time;
                }
            }
        }

        let sim_now = world.solver_sets[AWAKE_SET as usize].body_sims[sim_index];
        let transform = sim_now.transform;
        let is_fast = (sim_now.flags & body_flags::IS_FAST) != 0;
        let mut shape_id = world.bodies[body_id as usize].head_shape_id;
        while shape_id != NULL_INDEX {
            if is_fast {
                // Fast non-bullet AABBs already updated in solve_continuous;
                // fast bullets update later. Always mark enlarged for move buffer.
                world.task_contexts[0]
                    .enlarged_sim_bit_set
                    .set_bit(sim_index as u32);
            } else {
                let aabb = crate::shape::compute_fat_shape_aabb(
                    &world.shapes[shape_id as usize],
                    transform,
                    speculative,
                );
                let shape = &mut world.shapes[shape_id as usize];
                shape.aabb = aabb;

                debug_assert!((shape.flags & shape_flags::ENLARGED_AABB) == 0);

                if !aabb_contains(shape.fat_aabb, aabb) {
                    let margin = shape.aabb_margin;
                    let aabb_margin = Vec3 {
                        x: margin,
                        y: margin,
                        z: margin,
                    };
                    shape.fat_aabb = crate::math_functions::Aabb {
                        lower_bound: sub(aabb.lower_bound, aabb_margin),
                        upper_bound: add(aabb.upper_bound, aabb_margin),
                    };
                    shape.flags |= shape_flags::ENLARGED_AABB;

                    world.task_contexts[0]
                        .enlarged_sim_bit_set
                        .set_bit(sim_index as u32);
                }
            }

            shape_id = world.shapes[shape_id as usize].next_shape_id;
        }
    }
}
