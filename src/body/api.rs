// Body mass and velocity public API from body.c (b3Body_*).
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::body_flags;
use super::lifecycle::{
    get_body_full_id, get_body_sim, get_body_sim_mut, get_body_state_index,
    get_body_transform_quick, sync_body_flags, wake_body,
};
use super::mass::update_body_extents_from_shapes;
use super::types::BodyPlaneResult;
use crate::core::NULL_INDEX;
use crate::geometry::{Capsule, MassData, PlaneResult};
use crate::id::{BodyId, ShapeId};
use crate::math_functions::{
    add, cross, det, inv_rotate_vector, inv_transform_world_point, invert_t, is_valid_float,
    is_valid_matrix3, is_valid_vec3, length_squared, make_matrix_from_quat, mul_mm, rotate_vector,
    sub, sub_pos, to_relative_transform, transform_world_point, transpose, Matrix3, Pos, Vec3,
    WorldTransform, MAT3_ZERO, VEC3_ZERO,
};
use crate::shape::{collide_mover, should_query_collide, ShapeGeometry};
use crate::solver_set::{AWAKE_SET, DISABLED_SET};
use crate::types::{BodyType, MotionLocks, QueryFilter};
use crate::world::World;

/// (b3Body_GetMass)
pub fn body_get_mass(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].mass
}

/// (b3Body_GetLocalRotationalInertia)
pub fn body_get_local_rotational_inertia(world: &World, body_id: BodyId) -> Matrix3 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].inertia
}

/// (b3Body_GetInverseMass)
pub fn body_get_inverse_mass(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].inv_mass
}

/// (b3Body_GetWorldInverseRotationalInertia)
pub fn body_get_world_inverse_rotational_inertia(world: &World, body_id: BodyId) -> Matrix3 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
        .inv_inertia_world
}

/// (b3Body_GetLocalCenter)
pub fn body_get_local_center(world: &World, body_id: BodyId) -> Vec3 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].local_center
}

/// (b3Body_GetWorldCenter)
pub fn body_get_world_center(world: &World, body_id: BodyId) -> Pos {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].center
}

/// (b3Body_GetMassData)
pub fn body_get_mass_data(world: &World, body_id: BodyId) -> MassData {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    let sim = &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize];
    MassData {
        mass: body.mass,
        center: sim.local_center,
        inertia: body.inertia,
    }
}

/// (b3Body_SetMassData)
pub fn body_set_mass_data(world: &mut World, body_id: BodyId, mass_data: MassData) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_mass_data(body_id, mass_data);
    });
    debug_assert!(is_valid_float(mass_data.mass) && mass_data.mass >= 0.0);
    debug_assert!(is_valid_matrix3(mass_data.inertia));
    debug_assert!(is_valid_vec3(mass_data.center));

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);

    world.bodies[body_index as usize].flags &= !body_flags::DIRTY_MASS;
    sync_body_flags(world, body_index);

    world.bodies[body_index as usize].mass = mass_data.mass;
    world.bodies[body_index as usize].inertia = mass_data.inertia;

    let old_center;
    {
        let sim = get_body_sim_mut(world, body_index);
        sim.local_center = mass_data.center;
        old_center = sim.center;
        let center = transform_world_point(sim.transform, mass_data.center);
        sim.center = center;
        sim.center0 = center;
        sim.inv_mass = if mass_data.mass > 0.0 {
            1.0 / mass_data.mass
        } else {
            0.0
        };
    }

    // Update center of mass velocity
    if let Some(local_index) = get_body_state_index(world, body_index) {
        let new_center = world.solver_sets[world.bodies[body_index as usize].set_index as usize]
            .body_sims[world.bodies[body_index as usize].local_index as usize]
            .center;
        let state = &mut world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
        let delta_linear = cross(state.angular_velocity, sub_pos(new_center, old_center));
        state.linear_velocity = add(state.linear_velocity, delta_linear);
    }

    let inertia = world.bodies[body_index as usize].inertia;
    let d = det(inertia);
    debug_assert!(d >= 0.0);

    {
        let sim = get_body_sim_mut(world, body_index);
        if d > 0.0 {
            sim.inv_inertia_local = invert_t(inertia);
            let rotation_matrix = make_matrix_from_quat(sim.transform.q);
            sim.inv_inertia_world = mul_mm(
                mul_mm(rotation_matrix, sim.inv_inertia_local),
                transpose(rotation_matrix),
            );
        } else {
            sim.inv_inertia_local = MAT3_ZERO;
            sim.inv_inertia_world = MAT3_ZERO;
        }
    }

    // Apply fixed rotation
    if (world.bodies[body_index as usize].flags & body_flags::FIXED_ROTATION)
        == body_flags::FIXED_ROTATION
    {
        world.bodies[body_index as usize].inertia = MAT3_ZERO;
        let sim = get_body_sim_mut(world, body_index);
        sim.inv_inertia_local = MAT3_ZERO;
        sim.inv_inertia_world = MAT3_ZERO;
    }

    // Update extents using supplied mass center.
    update_body_extents_from_shapes(world, body_index, mass_data.center);
}

/// (b3Body_GetLinearVelocity)
pub fn body_get_linear_velocity(world: &World, body_id: BodyId) -> Vec3 {
    let body_index = get_body_full_id(world, body_id);
    if let Some(local_index) = get_body_state_index(world, body_index) {
        world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize].linear_velocity
    } else {
        VEC3_ZERO
    }
}

/// (b3Body_GetAngularVelocity)
pub fn body_get_angular_velocity(world: &World, body_id: BodyId) -> Vec3 {
    let body_index = get_body_full_id(world, body_id);
    if let Some(local_index) = get_body_state_index(world, body_index) {
        world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize].angular_velocity
    } else {
        VEC3_ZERO
    }
}

/// (b3Body_SetLinearVelocity)
pub fn body_set_linear_velocity(world: &mut World, body_id: BodyId, linear_velocity: Vec3) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_linear_velocity(body_id, linear_velocity);
    });
    debug_assert!(is_valid_vec3(linear_velocity));

    let body_index = get_body_full_id(world, body_id);

    if world.bodies[body_index as usize].type_ == BodyType::Static {
        return;
    }

    if length_squared(linear_velocity) > 0.0 {
        wake_body(world, body_index);
    }

    if let Some(local_index) = get_body_state_index(world, body_index) {
        world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize].linear_velocity =
            linear_velocity;
    }
}

/// (b3Body_SetAngularVelocity)
pub fn body_set_angular_velocity(world: &mut World, body_id: BodyId, angular_velocity: Vec3) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_angular_velocity(body_id, angular_velocity);
    });
    debug_assert!(is_valid_vec3(angular_velocity));

    let body_index = get_body_full_id(world, body_id);

    if world.bodies[body_index as usize].type_ == BodyType::Static {
        return;
    }

    let flags = world.bodies[body_index as usize].flags;
    let w = Vec3 {
        x: if (flags & body_flags::LOCK_ANGULAR_X) != 0 {
            0.0
        } else {
            angular_velocity.x
        },
        y: if (flags & body_flags::LOCK_ANGULAR_Y) != 0 {
            0.0
        } else {
            angular_velocity.y
        },
        z: if (flags & body_flags::LOCK_ANGULAR_Z) != 0 {
            0.0
        } else {
            angular_velocity.z
        },
    };

    if length_squared(w) != 0.0 {
        wake_body(world, body_index);
    }

    if let Some(local_index) = get_body_state_index(world, body_index) {
        world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize].angular_velocity =
            w;
    }
}

/// (b3Body_GetPosition)
pub fn body_get_position(world: &World, body_id: BodyId) -> Pos {
    let body_index = get_body_full_id(world, body_id);
    get_body_transform_quick(world, &world.bodies[body_index as usize]).p
}

pub fn body_set_linear_damping(world: &mut World, body_id: BodyId, linear_damping: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_linear_damping(body_id, linear_damping);
    });
    debug_assert!(is_valid_float(linear_damping) && linear_damping >= 0.0);
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }
    let body_index = get_body_full_id(world, body_id);
    get_body_sim_mut(world, body_index).linear_damping = linear_damping;
}

/// (b3Body_GetLinearDamping)
pub fn body_get_linear_damping(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].linear_damping
}

/// (b3Body_SetAngularDamping)
pub fn body_set_angular_damping(world: &mut World, body_id: BodyId, angular_damping: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_angular_damping(body_id, angular_damping);
    });
    debug_assert!(is_valid_float(angular_damping) && angular_damping >= 0.0);
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }
    let body_index = get_body_full_id(world, body_id);
    get_body_sim_mut(world, body_index).angular_damping = angular_damping;
}

/// (b3Body_GetAngularDamping)
pub fn body_get_angular_damping(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].angular_damping
}

/// (b3Body_SetGravityScale)
pub fn body_set_gravity_scale(world: &mut World, body_id: BodyId, gravity_scale: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_gravity_scale(body_id, gravity_scale);
    });
    debug_assert!(is_valid_float(gravity_scale));
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }
    let body_index = get_body_full_id(world, body_id);
    get_body_sim_mut(world, body_index).gravity_scale = gravity_scale;
}

/// (b3Body_GetGravityScale)
pub fn body_get_gravity_scale(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].gravity_scale
}

/// (b3Body_IsSleepEnabled)
pub fn body_is_sleep_enabled(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    (world.bodies[body_index as usize].flags & body_flags::ENABLE_SLEEP) != 0
}

/// (b3Body_SetSleepThreshold)
pub fn body_set_sleep_threshold(world: &mut World, body_id: BodyId, sleep_threshold: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_sleep_threshold(body_id, sleep_threshold);
    });
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].sleep_threshold = sleep_threshold;
}

/// (b3Body_GetSleepThreshold)
pub fn body_get_sleep_threshold(world: &World, body_id: BodyId) -> f32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].sleep_threshold
}

/// (b3Body_EnableSleep)
///
/// No-op when the flag is already at the requested value — must not leave the
/// world locked (EnableSleepNoopUnlockTest regression).
pub fn body_enable_sleep(world: &mut World, body_id: BodyId, enable_sleep: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_enable_sleep(body_id, enable_sleep);
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    let flag = (world.bodies[body_index as usize].flags & body_flags::ENABLE_SLEEP) != 0;
    if enable_sleep == flag {
        return;
    }

    world.locked = true;

    if enable_sleep {
        world.bodies[body_index as usize].flags |= body_flags::ENABLE_SLEEP;
    } else {
        world.bodies[body_index as usize].flags &= !body_flags::ENABLE_SLEEP;
    }
    sync_body_flags(world, body_index);

    if !enable_sleep {
        wake_body(world, body_index);
    }

    world.locked = false;
}

pub fn body_get_transform(world: &World, body_id: BodyId) -> crate::math_functions::WorldTransform {
    let body_index = get_body_full_id(world, body_id);
    get_body_transform_quick(world, &world.bodies[body_index as usize])
}

/// (b3Body_SetBullet)
pub fn body_set_bullet(world: &mut World, body_id: BodyId, flag: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_bullet(body_id, flag);
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let new_flag = if flag { body_flags::IS_BULLET } else { 0 };
    let body_index = get_body_full_id(world, body_id);
    if (world.bodies[body_index as usize].flags & body_flags::IS_BULLET) == new_flag {
        return;
    }

    world.bodies[body_index as usize].flags &= !body_flags::IS_BULLET;
    world.bodies[body_index as usize].flags |= new_flag;
    sync_body_flags(world, body_index);
}

/// (b3Body_IsBullet)
pub fn body_is_bullet(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    (world.bodies[body_index as usize].flags & body_flags::IS_BULLET) != 0
}

/// Allow this body to rotate fast. Useful for axially symmetric bodies, such as vehicle
/// wheels. Normally rotation speed is clamped to improve CCD. However, this clamping is
/// unnecessary for bodies that only rotate fast around an axis of symmetry.
/// (b3Body_AllowFastRotation)
pub fn body_allow_fast_rotation(world: &mut World, body_id: BodyId, flag: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_allow_fast_rotation(body_id, flag);
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let new_flag = if flag {
        body_flags::ALLOW_FAST_ROTATION
    } else {
        0
    };
    let body_index = get_body_full_id(world, body_id);
    if (world.bodies[body_index as usize].flags & body_flags::ALLOW_FAST_ROTATION) == new_flag {
        return;
    }

    world.bodies[body_index as usize].flags &= !body_flags::ALLOW_FAST_ROTATION;
    world.bodies[body_index as usize].flags |= new_flag;
    sync_body_flags(world, body_index);
}

/// (b3Body_IsFastRotationAllowed)
pub fn body_is_fast_rotation_allowed(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    (world.bodies[body_index as usize].flags & body_flags::ALLOW_FAST_ROTATION) != 0
}

/// (b3Body_EnableContactRecycling)
pub fn body_enable_contact_recycling(world: &mut World, body_id: BodyId, flag: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_enable_contact_recycling(body_id, flag);
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let new_flag = if flag {
        body_flags::BODY_ENABLE_CONTACT_RECYCLING
    } else {
        0
    };
    let body_index = get_body_full_id(world, body_id);
    if (world.bodies[body_index as usize].flags & body_flags::BODY_ENABLE_CONTACT_RECYCLING)
        == new_flag
    {
        return;
    }

    world.bodies[body_index as usize].flags &= !body_flags::BODY_ENABLE_CONTACT_RECYCLING;
    world.bodies[body_index as usize].flags |= new_flag;
    sync_body_flags(world, body_index);
}

/// (b3Body_IsContactRecyclingEnabled)
pub fn body_is_contact_recycling_enabled(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    (world.bodies[body_index as usize].flags & body_flags::BODY_ENABLE_CONTACT_RECYCLING) != 0
}

/// Enable hit events on all shapes attached to this body. (b3Body_EnableHitEvents)
pub fn body_enable_hit_events(world: &mut World, body_id: BodyId, flag: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_enable_hit_events(body_id, flag);
    });
    use crate::core::NULL_INDEX;
    use crate::shape::shape_flags;

    let body_index = get_body_full_id(world, body_id);
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let shape = &mut world.shapes[shape_id as usize];
        if flag {
            shape.flags |= shape_flags::ENABLE_HIT_EVENTS;
        } else {
            shape.flags &= !shape_flags::ENABLE_HIT_EVENTS;
        }
        shape_id = shape.next_shape_id;
    }
}

/// (b3Body_SetMotionLocks)
pub fn body_set_motion_locks(world: &mut World, body_id: BodyId, locks: crate::types::MotionLocks) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_motion_locks(body_id, locks);
    });
    use super::mass::update_body_mass_data;

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let mut new_locks = 0u32;
    new_locks |= if locks.linear_x {
        body_flags::LOCK_LINEAR_X
    } else {
        0
    };
    new_locks |= if locks.linear_y {
        body_flags::LOCK_LINEAR_Y
    } else {
        0
    };
    new_locks |= if locks.linear_z {
        body_flags::LOCK_LINEAR_Z
    } else {
        0
    };
    new_locks |= if locks.angular_x {
        body_flags::LOCK_ANGULAR_X
    } else {
        0
    };
    new_locks |= if locks.angular_y {
        body_flags::LOCK_ANGULAR_Y
    } else {
        0
    };
    new_locks |= if locks.angular_z {
        body_flags::LOCK_ANGULAR_Z
    } else {
        0
    };

    let body_index = get_body_full_id(world, body_id);
    if (world.bodies[body_index as usize].flags & body_flags::ALL_LOCKS) == new_locks {
        return;
    }

    let fixed_rotation1 = (world.bodies[body_index as usize].flags & body_flags::FIXED_ROTATION)
        == body_flags::FIXED_ROTATION;
    let fixed_rotation2 = (new_locks & body_flags::FIXED_ROTATION) == body_flags::FIXED_ROTATION;

    world.bodies[body_index as usize].flags &= !body_flags::ALL_LOCKS;
    world.bodies[body_index as usize].flags |= new_locks;

    sync_body_flags(world, body_index);

    if let Some(local_index) = get_body_state_index(world, body_index) {
        let state = &mut world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
        if locks.linear_x {
            state.linear_velocity.x = 0.0;
        }
        if locks.linear_y {
            state.linear_velocity.y = 0.0;
        }
        if locks.linear_z {
            state.linear_velocity.z = 0.0;
        }
        if locks.angular_x {
            state.angular_velocity.x = 0.0;
        }
        if locks.angular_y {
            state.angular_velocity.y = 0.0;
        }
        if locks.angular_z {
            state.angular_velocity.z = 0.0;
        }
    }

    if fixed_rotation1 != fixed_rotation2 {
        update_body_mass_data(world, body_index);
    }
}

/// (b3Body_GetMotionLocks)
pub fn body_get_motion_locks(world: &World, body_id: BodyId) -> MotionLocks {
    let body_index = get_body_full_id(world, body_id);
    let flags = world.bodies[body_index as usize].flags;
    MotionLocks {
        linear_x: (flags & body_flags::LOCK_LINEAR_X) != 0,
        linear_y: (flags & body_flags::LOCK_LINEAR_Y) != 0,
        linear_z: (flags & body_flags::LOCK_LINEAR_Z) != 0,
        angular_x: (flags & body_flags::LOCK_ANGULAR_X) != 0,
        angular_y: (flags & body_flags::LOCK_ANGULAR_Y) != 0,
        angular_z: (flags & body_flags::LOCK_ANGULAR_Z) != 0,
    }
}

/// (b3Body_GetType)
pub fn body_get_type(world: &World, body_id: BodyId) -> BodyType {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].type_
}

/// (b3Body_IsAwake)
pub fn body_is_awake(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].set_index == AWAKE_SET
}

/// (b3Body_IsEnabled)
pub fn body_is_enabled(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].set_index != DISABLED_SET
}

/// Set the body name. Uses the world name cache rather than C's fixed char
/// buffer. (b3Body_SetName)
pub fn body_set_name(world: &mut World, body_id: BodyId, name: &str) {
    crate::recording::with_recording(world, |rec| {
        rec.write_body_set_name(body_id, name);
    });
    let name_id = world.names.add_name(name);
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].name_id = name_id;
}

/// Get the body name. Returns an empty string if the name isn't set.
/// (b3Body_GetName)
pub fn body_get_name(world: &World, body_id: BodyId) -> &str {
    let body_index = get_body_full_id(world, body_id);
    let name_id = world.bodies[body_index as usize].name_id;
    world.names.find_name_with_default(name_id, "")
}

/// (b3Body_SetUserData) — Rust stores `u64` instead of `void*`.
pub fn body_set_user_data(world: &mut World, body_id: BodyId, user_data: u64) {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].user_data = user_data;
}

/// (b3Body_GetUserData)
pub fn body_get_user_data(world: &World, body_id: BodyId) -> u64 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].user_data
}

/// (b3Body_GetLocalPoint)
pub fn body_get_local_point(world: &World, body_id: BodyId, world_point: Pos) -> Vec3 {
    let body_index = get_body_full_id(world, body_id);
    let transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);
    inv_transform_world_point(transform, world_point)
}

/// (b3Body_GetWorldPoint)
pub fn body_get_world_point(world: &World, body_id: BodyId, local_point: Vec3) -> Pos {
    let body_index = get_body_full_id(world, body_id);
    let transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);
    transform_world_point(transform, local_point)
}

/// (b3Body_GetLocalVector)
pub fn body_get_local_vector(world: &World, body_id: BodyId, world_vector: Vec3) -> Vec3 {
    let body_index = get_body_full_id(world, body_id);
    let transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);
    inv_rotate_vector(transform.q, world_vector)
}

/// (b3Body_GetWorldVector)
pub fn body_get_world_vector(world: &World, body_id: BodyId, local_vector: Vec3) -> Vec3 {
    let body_index = get_body_full_id(world, body_id);
    let transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);
    rotate_vector(transform.q, local_vector)
}

/// (b3Body_GetLocalPointVelocity)
pub fn body_get_local_point_velocity(world: &World, body_id: BodyId, local_point: Vec3) -> Vec3 {
    let body_index = get_body_full_id(world, body_id);
    let Some(local_index) = get_body_state_index(world, body_index) else {
        return VEC3_ZERO;
    };
    let state = &world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
    let body_sim = get_body_sim(world, body_index);
    let r = rotate_vector(
        body_sim.transform.q,
        sub(local_point, body_sim.local_center),
    );
    add(state.linear_velocity, cross(state.angular_velocity, r))
}

/// (b3Body_GetWorldPointVelocity)
pub fn body_get_world_point_velocity(world: &World, body_id: BodyId, world_point: Pos) -> Vec3 {
    let body_index = get_body_full_id(world, body_id);
    let Some(local_index) = get_body_state_index(world, body_index) else {
        return VEC3_ZERO;
    };
    let state = &world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
    let body_sim = get_body_sim(world, body_index);
    let r = sub_pos(world_point, body_sim.center);
    add(state.linear_velocity, cross(state.angular_velocity, r))
}

/// Collide a capsule mover against a single body's sphere/capsule/hull shapes.
/// (b3Body_CollideMover)
pub fn body_collide_mover(
    world: &World,
    body_id: BodyId,
    body_planes: &mut [BodyPlaneResult],
    origin: Pos,
    mover: &Capsule,
    filter: &QueryFilter,
    body_transform: WorldTransform,
) -> i32 {
    debug_assert!(!world.locked);
    if world.locked {
        return 0;
    }

    let plane_capacity = body_planes.len() as i32;
    if plane_capacity == 0 {
        return 0;
    }

    let mut result_count = 0i32;
    let body_index = get_body_full_id(world, body_id);
    let transform = to_relative_transform(body_transform, origin);

    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let shape = &world.shapes[shape_id as usize];
        shape_id = shape.next_shape_id;

        if !should_query_collide(&shape.filter, filter) {
            continue;
        }

        match &shape.geometry {
            ShapeGeometry::Sphere(_) | ShapeGeometry::Capsule(_) | ShapeGeometry::Hull(_) => {}
            _ => continue,
        }

        let mut plane = PlaneResult::default();
        let count = collide_mover(std::slice::from_mut(&mut plane), shape, transform, mover);

        if count > 0 {
            let id = ShapeId {
                index1: shape.id + 1,
                world0: body_id.world0,
                generation: shape.generation,
            };
            body_planes[result_count as usize] = BodyPlaneResult {
                shape_id: id,
                result: plane,
            };
            result_count += 1;
            if result_count == plane_capacity {
                return result_count;
            }
        }
    }

    result_count
}
