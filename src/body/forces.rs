//! Body force/impulse and transform/awake public API from body.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::lifecycle::{get_body_full_id, get_body_sim_mut, wake_body, wake_body_with_lock};
use crate::constants::speculative_distance;
use crate::core::NULL_INDEX;
use crate::id::BodyId;
use crate::island::split_island;
use crate::math_functions::{
    aabb_contains, add, conjugate, cross, dot_quat, inv_rotate_vector, is_valid_position,
    is_valid_quat, is_valid_vec3, is_valid_world_transform, length, length_squared,
    make_matrix_from_quat, mul, mul_add, mul_mm, mul_mv, mul_quat, mul_sv, negate_quat, normalize,
    rotate_vector, sub, sub_pos, transform_world_point, transpose, Aabb, Pos, Quat, Vec3,
    WorldTransform,
};
use crate::shape::compute_fat_shape_aabb;
use crate::solver_set::{try_sleep_island, AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET};
use crate::types::BodyType;
use crate::world::World;

/// (b3Body_ApplyForce)
pub fn body_apply_force(world: &mut World, body_id: BodyId, force: Vec3, point: Pos, wake: bool) {
    debug_assert!(is_valid_vec3(force));

    let body_index = get_body_full_id(world, body_id);

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body_with_lock(world, body_index);
    }

    if world.bodies[body_index as usize].set_index == AWAKE_SET {
        let body_sim = get_body_sim_mut(world, body_index);
        body_sim.force = add(body_sim.force, force);
        body_sim.torque = add(
            body_sim.torque,
            cross(sub_pos(point, body_sim.center), force),
        );
    }
}

/// (b3Body_ApplyForceToCenter)
pub fn body_apply_force_to_center(world: &mut World, body_id: BodyId, force: Vec3, wake: bool) {
    debug_assert!(is_valid_vec3(force));

    let body_index = get_body_full_id(world, body_id);

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body_with_lock(world, body_index);
    }

    if world.bodies[body_index as usize].set_index == AWAKE_SET {
        let body_sim = get_body_sim_mut(world, body_index);
        body_sim.force = add(body_sim.force, force);
    }
}

/// (b3Body_ApplyTorque)
pub fn body_apply_torque(world: &mut World, body_id: BodyId, torque: Vec3, wake: bool) {
    debug_assert!(is_valid_vec3(torque));

    let body_index = get_body_full_id(world, body_id);

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body_with_lock(world, body_index);
    }

    if world.bodies[body_index as usize].set_index == AWAKE_SET {
        let body_sim = get_body_sim_mut(world, body_index);
        body_sim.torque = add(body_sim.torque, torque);
    }
}

/// (b3Body_ApplyLinearImpulse)
pub fn body_apply_linear_impulse(
    world: &mut World,
    body_id: BodyId,
    impulse: Vec3,
    point: Pos,
    wake: bool,
) {
    debug_assert!(is_valid_vec3(impulse));
    debug_assert!(is_valid_position(point));

    let body_index = get_body_full_id(world, body_id);

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body_with_lock(world, body_index);
    }

    if world.bodies[body_index as usize].set_index == AWAKE_SET {
        let local_index = world.bodies[body_index as usize].local_index;
        let max_linear_speed = world.max_linear_speed;
        let set = &mut world.solver_sets[AWAKE_SET as usize];
        let state = &mut set.body_states[local_index as usize];
        let body_sim = &set.body_sims[local_index as usize];

        state.linear_velocity = mul_add(state.linear_velocity, body_sim.inv_mass, impulse);

        if length_squared(state.linear_velocity) > max_linear_speed * max_linear_speed {
            state.linear_velocity = mul_sv(max_linear_speed, normalize(state.linear_velocity));
        }

        let delta = mul_mv(
            body_sim.inv_inertia_world,
            cross(sub_pos(point, body_sim.center), impulse),
        );
        state.angular_velocity = add(state.angular_velocity, delta);
    }
}

/// (b3Body_ApplyLinearImpulseToCenter)
pub fn body_apply_linear_impulse_to_center(
    world: &mut World,
    body_id: BodyId,
    impulse: Vec3,
    wake: bool,
) {
    debug_assert!(is_valid_vec3(impulse));

    let body_index = get_body_full_id(world, body_id);

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        wake_body_with_lock(world, body_index);
    }

    if world.bodies[body_index as usize].set_index == AWAKE_SET {
        let local_index = world.bodies[body_index as usize].local_index;
        let max_linear_speed = world.max_linear_speed;
        let set = &mut world.solver_sets[AWAKE_SET as usize];
        let state = &mut set.body_states[local_index as usize];
        let body_sim = &set.body_sims[local_index as usize];
        state.linear_velocity = mul_add(state.linear_velocity, body_sim.inv_mass, impulse);

        if length_squared(state.linear_velocity) > max_linear_speed * max_linear_speed {
            state.linear_velocity = mul_sv(max_linear_speed, normalize(state.linear_velocity));
        }
    }
}

/// (b3Body_ApplyAngularImpulse)
pub fn body_apply_angular_impulse(world: &mut World, body_id: BodyId, impulse: Vec3, wake: bool) {
    debug_assert!(is_valid_vec3(impulse));
    debug_assert!(super::lifecycle::body_is_valid(world, body_id));

    let body_index = get_body_full_id(world, body_id);

    if wake && world.bodies[body_index as usize].set_index >= FIRST_SLEEPING_SET {
        // this will not invalidate body pointer
        wake_body_with_lock(world, body_index);
    }

    if world.bodies[body_index as usize].set_index == AWAKE_SET {
        let local_index = world.bodies[body_index as usize].local_index;
        let set = &mut world.solver_sets[AWAKE_SET as usize];
        let state = &mut set.body_states[local_index as usize];
        let body_sim = &set.body_sims[local_index as usize];

        let local_impulse = inv_rotate_vector(body_sim.transform.q, impulse);
        let local_angular_velocity_delta = mul_mv(body_sim.inv_inertia_local, local_impulse);
        state.angular_velocity = add(
            state.angular_velocity,
            rotate_vector(body_sim.transform.q, local_angular_velocity_delta),
        );
    }
}

/// (b3Body_SetTransform)
pub fn body_set_transform(world: &mut World, body_id: BodyId, position: Pos, rotation: Quat) {
    debug_assert!(is_valid_position(position));
    debug_assert!(is_valid_quat(rotation));
    debug_assert!(super::lifecycle::body_is_valid(world, body_id));
    debug_assert!(!world.locked);

    let body_index = get_body_full_id(world, body_id);

    {
        let body_sim = get_body_sim_mut(world, body_index);
        body_sim.transform.p = position;
        body_sim.transform.q = rotation;
        body_sim.center = transform_world_point(body_sim.transform, body_sim.local_center);

        let rotation_matrix = make_matrix_from_quat(body_sim.transform.q);
        body_sim.inv_inertia_world = mul_mm(
            mul_mm(rotation_matrix, body_sim.inv_inertia_local),
            transpose(rotation_matrix),
        );

        body_sim.rotation0 = body_sim.transform.q;
        body_sim.center0 = body_sim.center;
    }

    let transform = world.solver_sets[world.bodies[body_index as usize].set_index as usize]
        .body_sims[world.bodies[body_index as usize].local_index as usize]
        .transform;
    let speculative = speculative_distance();

    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next_shape_id;
        let aabb;
        let needs_move;
        let fat_aabb;
        let proxy_key;
        {
            let shape = &world.shapes[shape_id as usize];
            next_shape_id = shape.next_shape_id;
            aabb = compute_fat_shape_aabb(shape, transform, speculative);
            needs_move = !aabb_contains(shape.fat_aabb, aabb);
            let margin = shape.aabb_margin;
            fat_aabb = Aabb {
                lower_bound: crate::math_functions::Vec3 {
                    x: aabb.lower_bound.x - margin,
                    y: aabb.lower_bound.y - margin,
                    z: aabb.lower_bound.z - margin,
                },
                upper_bound: crate::math_functions::Vec3 {
                    x: aabb.upper_bound.x + margin,
                    y: aabb.upper_bound.y + margin,
                    z: aabb.upper_bound.z + margin,
                },
            };
            proxy_key = shape.proxy_key;
        }

        {
            let shape = &mut world.shapes[shape_id as usize];
            shape.aabb = aabb;
            if needs_move {
                shape.fat_aabb = fat_aabb;
            }
        }

        // The body could be disabled
        if needs_move && proxy_key != NULL_INDEX {
            world.broad_phase.move_proxy(proxy_key, fat_aabb);
        }

        shape_id = next_shape_id;
    }
}

/// (b3Body_SetAwake)
pub fn body_set_awake(world: &mut World, body_id: BodyId, awake: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;

    let body_index = get_body_full_id(world, body_id);
    let set_index = world.bodies[body_index as usize].set_index;
    let island_id = world.bodies[body_index as usize].island_id;

    if awake && set_index >= FIRST_SLEEPING_SET {
        wake_body(world, body_index);
    } else if !awake && set_index == AWAKE_SET {
        if world.islands[island_id as usize].constraint_remove_count > 0 {
            // Must split the island before sleeping. This is expensive.
            split_island(world, island_id);
        }

        try_sleep_island(world, island_id);
    }

    world.locked = false;
}

/// Set a kinematic/dynamic body velocity so it reaches `target` in `time_step`.
/// (b3Body_SetTargetTransform)
pub fn body_set_target_transform(
    world: &mut World,
    body_id: BodyId,
    target: WorldTransform,
    time_step: f32,
    wake: bool,
) {
    debug_assert!(is_valid_world_transform(target));

    let body_index = get_body_full_id(world, body_id);

    {
        let body = &world.bodies[body_index as usize];
        if body.set_index == DISABLED_SET {
            return;
        }

        if body.type_ == BodyType::Static || time_step <= 0.0 {
            return;
        }

        if body.set_index != AWAKE_SET && !wake {
            return;
        }
    }

    let (sim_local_center, sim_center, sim_q, sim_max_extent) = {
        let body = &world.bodies[body_index as usize];
        let sim = &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];
        (
            sim.local_center,
            sim.center,
            sim.transform.q,
            sim.max_extent,
        )
    };

    // Compute linear velocity. The center difference is taken in world precision then demoted.
    let center2 = transform_world_point(target, sim_local_center);
    let inv_time_step = 1.0 / time_step;
    let linear_velocity = mul_sv(inv_time_step, sub_pos(center2, sim_center));

    // Compute angular velocity:
    // q' = 0.5 * w * q
    // <~> ( q2 - q1 ) / dt =  0.5 * w * q1
    // <=> w = 2 * ( q2 - q1 ) * Conjugate( q1 ) / dt
    let q1 = sim_q;
    let mut q2 = target.q;

    // Use the shortest arc quaternion
    if dot_quat(q1, q2) < 0.0 {
        q2 = negate_quat(q2);
    }

    let dq = Quat {
        v: sub(q2.v, q1.v),
        s: q2.s - q1.s,
    };
    let omega = mul_quat(dq, conjugate(q1));
    let angular_velocity = mul_sv(2.0 * inv_time_step, omega.v);

    // Early out if the body is asleep already and the desired movement is small
    if world.bodies[body_index as usize].set_index != AWAKE_SET {
        let max_velocity =
            length(linear_velocity) + length(mul(angular_velocity, sim_max_extent));

        // Return if velocity would be sleepy
        if max_velocity < world.bodies[body_index as usize].sleep_threshold {
            return;
        }

        // Must wake for state to exist
        wake_body_with_lock(world, body_index);
    }

    debug_assert!(world.bodies[body_index as usize].set_index == AWAKE_SET);

    let local_index = world.bodies[body_index as usize].local_index;
    let state = &mut world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
    state.linear_velocity = linear_velocity;
    state.angular_velocity = angular_velocity;
}
