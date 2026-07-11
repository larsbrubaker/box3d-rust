// Body mass and velocity public API from body.c (b3Body_*).
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::lifecycle::{
    get_body_full_id, get_body_sim_mut, get_body_state_index, get_body_transform_quick,
    sync_body_flags, wake_body,
};
use super::mass::update_body_extents_from_shapes;
use super::body_flags;
use crate::geometry::MassData;
use crate::id::BodyId;
use crate::math_functions::{
    add, cross, det, invert_t, is_valid_float, is_valid_matrix3, is_valid_vec3, length_squared,
    make_matrix_from_quat, mul_mm, sub_pos, transform_world_point, transpose, Matrix3, Pos, Vec3,
    MAT3_ZERO, VEC3_ZERO,
};
use crate::solver_set::AWAKE_SET;
use crate::types::BodyType;
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
