// Body mass update from shapes (b3UpdateBodyMassData / ApplyMassFromShapes).
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::lifecycle::{
    get_body_full_id, get_body_sim_mut, get_body_state_index, sync_body_flags,
};
use super::{body_flags, BodySim};
use crate::constants::huge;
use crate::core::NULL_INDEX;
use crate::geometry::MassData;
use crate::id::BodyId;
use crate::math_functions::{
    add, add_mm, cross, det, invert_t, make_matrix_from_quat, max, min_float, mul_add, mul_mm,
    mul_sv, steiner, sub, sub_pos, transform_world_point, transpose, Vec3, MAT3_ZERO, VEC3_ZERO,
};
use crate::shape::{compute_shape_extent, compute_shape_mass};
use crate::solver_set::AWAKE_SET;
use crate::types::BodyType;
use crate::world::World;

/// Recompute mass, inertia, and extents from the body's shapes.
/// (b3UpdateBodyMassData)
pub fn update_body_mass_data(world: &mut World, body_index: i32) {
    let (set_index, local_index, body_type, head_shape_id, shape_count) = {
        let body = &mut world.bodies[body_index as usize];
        // Mass is no longer dirty
        body.flags &= !body_flags::DIRTY_MASS;
        body.mass = 0.0;
        body.inertia = MAT3_ZERO;
        (
            body.set_index,
            body.local_index,
            body.type_,
            body.head_shape_id,
            body.shape_count,
        )
    };
    sync_body_flags(world, body_index);

    {
        let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        sim.inv_mass = 0.0;
        sim.inv_inertia_local = MAT3_ZERO;
        sim.inv_inertia_world = MAT3_ZERO;
        sim.local_center = VEC3_ZERO;
        sim.min_extent = huge();
        sim.max_extent = VEC3_ZERO;
    }

    if head_shape_id == NULL_INDEX {
        return;
    }

    // Static and kinematic sims have zero mass.
    if body_type != BodyType::Dynamic {
        {
            let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
            sim.center = sim.transform.p;
            sim.center0 = sim.center;
        }

        // Need extents for kinematic bodies for sleeping to work correctly.
        if body_type == BodyType::Kinematic {
            let mut shape_id = head_shape_id;
            while shape_id != NULL_INDEX {
                let (extent, next) = {
                    let s = &world.shapes[shape_id as usize];
                    let e = compute_shape_extent(s, VEC3_ZERO);
                    (e, s.next_shape_id)
                };
                shape_id = next;
                let sim =
                    &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
                sim.min_extent = min_float(sim.min_extent, extent.min_extent);
                sim.max_extent = max(sim.max_extent, extent.max_extent);
            }
        }

        return;
    }

    // C uses arena scratch (b3StackAlloc); a Vec is the Rust equivalent.
    let mut masses: Vec<MassData> = Vec::with_capacity(shape_count as usize);

    // Accumulate mass over all shapes.
    let mut local_center = VEC3_ZERO;
    let mut shape_id = head_shape_id;
    while shape_id != NULL_INDEX {
        let (mass_data, next) = {
            let s = &world.shapes[shape_id as usize];
            let next = s.next_shape_id;
            if s.density == 0.0 {
                (MassData::default(), next)
            } else {
                (compute_shape_mass(s), next)
            }
        };
        shape_id = next;

        world.bodies[body_index as usize].mass += mass_data.mass;
        if mass_data.mass != 0.0 {
            local_center = mul_add(local_center, mass_data.mass, mass_data.center);
        }
        masses.push(mass_data);
    }

    // Compute center of mass.
    let body_mass = world.bodies[body_index as usize].mass;
    if body_mass > 0.0 {
        let inv_mass = 1.0 / body_mass;
        world.solver_sets[set_index as usize].body_sims[local_index as usize].inv_mass = inv_mass;
        local_center = mul_sv(inv_mass, local_center);
    }

    // Second loop to accumulate the rotational inertia about the center of mass
    for mass_data in &masses {
        if mass_data.mass == 0.0 {
            continue;
        }

        // Shift to center of mass. This is safe because it can only increase.
        let offset = sub(local_center, mass_data.center);
        let inertia = add_mm(mass_data.inertia, steiner(mass_data.mass, offset));
        world.bodies[body_index as usize].inertia =
            add_mm(world.bodies[body_index as usize].inertia, inertia);
    }

    let inertia = world.bodies[body_index as usize].inertia;
    let d = det(inertia);
    debug_assert!(d >= 0.0);

    if d > 0.0 {
        let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        sim.inv_inertia_local = invert_t(inertia);
        let rotation_matrix = make_matrix_from_quat(sim.transform.q);
        sim.inv_inertia_world = mul_mm(
            mul_mm(rotation_matrix, sim.inv_inertia_local),
            transpose(rotation_matrix),
        );
    }

    // Move center of mass.
    let old_center = {
        let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        let old = sim.center;
        sim.local_center = local_center;
        sim.center = transform_world_point(sim.transform, sim.local_center);
        sim.center0 = sim.center;
        old
    };

    // Update center of mass velocity
    if let Some(state_local) = get_body_state_index(world, body_index) {
        let new_center =
            world.solver_sets[set_index as usize].body_sims[local_index as usize].center;
        let state = &mut world.solver_sets[AWAKE_SET as usize].body_states[state_local as usize];
        let delta_linear = cross(state.angular_velocity, sub_pos(new_center, old_center));
        state.linear_velocity = add(state.linear_velocity, delta_linear);
    }

    // Compute body extents relative to center of mass
    shape_id = head_shape_id;
    while shape_id != NULL_INDEX {
        let (extent, next) = {
            let s = &world.shapes[shape_id as usize];
            let e = compute_shape_extent(s, local_center);
            (e, s.next_shape_id)
        };
        shape_id = next;
        let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        sim.min_extent = min_float(sim.min_extent, extent.min_extent);
        sim.max_extent = max(sim.max_extent, extent.max_extent);
    }

    // Apply fixed rotation
    if (world.bodies[body_index as usize].flags & body_flags::FIXED_ROTATION)
        == body_flags::FIXED_ROTATION
    {
        world.bodies[body_index as usize].inertia = MAT3_ZERO;
        let sim = &mut world.solver_sets[set_index as usize].body_sims[local_index as usize];
        sim.inv_inertia_local = MAT3_ZERO;
        sim.inv_inertia_world = MAT3_ZERO;
    }
}

/// This updates the mass properties to the sum of the mass properties of the shapes.
/// (b3Body_ApplyMassFromShapes)
pub fn body_apply_mass_from_shapes(world: &mut World, body_id: BodyId) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    update_body_mass_data(world, body_index);
}

/// Walk the body's shape list and update sim extents about `local_center`.
pub(crate) fn update_body_extents_from_shapes(
    world: &mut World,
    body_index: i32,
    local_center: Vec3,
) {
    let head = world.bodies[body_index as usize].head_shape_id;
    {
        let sim = get_body_sim_mut(world, body_index);
        sim.min_extent = huge();
        sim.max_extent = VEC3_ZERO;
    }

    let mut shape_id = head;
    while shape_id != NULL_INDEX {
        let (extent, next) = {
            let s = &world.shapes[shape_id as usize];
            let e = compute_shape_extent(s, local_center);
            (e, s.next_shape_id)
        };
        shape_id = next;
        let sim = get_body_sim_mut(world, body_index);
        sim.min_extent = min_float(sim.min_extent, extent.min_extent);
        sim.max_extent = max(sim.max_extent, extent.max_extent);
    }
}

/// Helper used by mass tests / internals to read BodySim.
#[allow(dead_code)]
pub(crate) fn body_sim(world: &World, body_id: BodyId) -> &BodySim {
    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
}
