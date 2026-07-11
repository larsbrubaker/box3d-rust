// Body SetType / Enable / Disable from body.c.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::body_flags;
use super::lifecycle::{
    create_island_for_body, destroy_body_contacts, get_body_full_id, get_body_transform_quick,
    remove_body_from_island, sync_body_flags, wake_body,
};
use super::mass::update_body_mass_data;
use crate::core::NULL_INDEX;
use crate::geometry::ShapeType;
use crate::id::BodyId;
use crate::island::{link_joint, unlink_joint, validate_island};
use crate::shape::{create_shape_proxy, destroy_shape_proxy};
use crate::solver_set::{transfer_body, transfer_joint, AWAKE_SET, DISABLED_SET, STATIC_SET};
use crate::types::BodyType;
use crate::world::World;

/// Change a body's simulation type. (b3Body_SetType)
pub fn body_set_type(world: &mut World, body_id: BodyId, type_: BodyType) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;
    let body_index = get_body_full_id(world, body_id);

    let original_type = world.bodies[body_index as usize].type_;
    if original_type == type_ {
        world.locked = false;
        return;
    }

    if type_ != BodyType::Static {
        let mut shape_id = world.bodies[body_index as usize].head_shape_id;
        while shape_id != NULL_INDEX {
            let shape = &world.shapes[shape_id as usize];
            if shape.shape_type() == ShapeType::Compound || shape.shape_type() == ShapeType::Height
            {
                // Setting the body type is not supported for bodies with compound/height shapes.
                // C forgets to unlock here — clear that bug.
                world.locked = false;
                return;
            }
            shape_id = shape.next_shape_id;
        }
    }

    // Stage 1: skip disabled bodies
    if world.bodies[body_index as usize].set_index == DISABLED_SET {
        world.bodies[body_index as usize].type_ = type_;

        if type_ == BodyType::Dynamic {
            world.bodies[body_index as usize].flags |= body_flags::DYNAMIC_FLAG;
        } else {
            world.bodies[body_index as usize].flags &= !body_flags::DYNAMIC_FLAG;
        }

        sync_body_flags(world, body_index);
        update_body_mass_data(world, body_index);
        world.locked = false;
        return;
    }

    // Stage 2: destroy all contacts but don't wake bodies
    destroy_body_contacts(world, body_index, false);

    // Stage 3: wake this body (does nothing if body is static)
    wake_body(world, body_index);

    // Stage 4: move joints to temporary storage (static set)
    let mut joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;
        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        // Joint may be disabled by other body
        if world.joints[joint_id as usize].set_index == DISABLED_SET {
            continue;
        }

        // Wake attached bodies. Wake on this body alone does not wake bodies
        // attached to a static body.
        let body_a = world.joints[joint_id as usize].edges[0].body_id;
        let body_b = world.joints[joint_id as usize].edges[1].body_id;
        wake_body(world, body_a);
        wake_body(world, body_b);

        unlink_joint(world, joint_id);

        // Transfer all joints to the static set so they can be recolored later.
        let joint_source_set = world.joints[joint_id as usize].set_index;
        transfer_joint(world, STATIC_SET, joint_source_set, joint_id);
    }

    // Stage 5: change the body type and transfer body
    world.bodies[body_index as usize].type_ = type_;

    if type_ == BodyType::Dynamic {
        world.bodies[body_index as usize].flags |= body_flags::DYNAMIC_FLAG;
    } else {
        world.bodies[body_index as usize].flags &= !body_flags::DYNAMIC_FLAG;
    }

    let source_set = world.bodies[body_index as usize].set_index;
    let target_set = if type_ == BodyType::Static {
        STATIC_SET
    } else {
        AWAKE_SET
    };
    transfer_body(world, target_set, source_set, body_index);

    // Stage 6: update island participation for the body
    if original_type == BodyType::Static {
        create_island_for_body(world, AWAKE_SET, body_index);
    } else if type_ == BodyType::Static {
        remove_body_from_island(world, body_index);
    }

    // Stage 7: Transfer joints to the target set
    joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;
        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        if world.joints[joint_id as usize].set_index == DISABLED_SET {
            continue;
        }

        // All joints were transferred to the static set in an earlier stage
        debug_assert!(world.joints[joint_id as usize].set_index == STATIC_SET);

        let body_a = world.joints[joint_id as usize].edges[0].body_id;
        let body_b = world.joints[joint_id as usize].edges[1].body_id;
        debug_assert!(
            world.bodies[body_a as usize].set_index == STATIC_SET
                || world.bodies[body_a as usize].set_index == AWAKE_SET
        );
        debug_assert!(
            world.bodies[body_b as usize].set_index == STATIC_SET
                || world.bodies[body_b as usize].set_index == AWAKE_SET
        );

        if world.bodies[body_a as usize].type_ == BodyType::Dynamic
            || world.bodies[body_b as usize].type_ == BodyType::Dynamic
        {
            transfer_joint(world, AWAKE_SET, STATIC_SET, joint_id);
        }
    }

    // Recreate shape proxies in broadphase
    let transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next_shape_id = world.shapes[shape_id as usize].next_shape_id;
        debug_assert!(world.shapes[shape_id as usize].shape_type() != ShapeType::Compound);

        {
            let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
            destroy_shape_proxy(&mut shapes[shape_id as usize], broad_phase);
            create_shape_proxy(
                &mut shapes[shape_id as usize],
                broad_phase,
                type_,
                transform,
                true,
            );
        }

        shape_id = next_shape_id;
    }

    // Relink all joints
    joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;
        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        let other_edge_index = edge_index ^ 1;
        let other_body_id =
            world.joints[joint_id as usize].edges[other_edge_index as usize].body_id;

        if world.bodies[other_body_id as usize].set_index == DISABLED_SET {
            continue;
        }

        if world.bodies[body_index as usize].type_ != BodyType::Dynamic
            && world.bodies[other_body_id as usize].type_ != BodyType::Dynamic
        {
            continue;
        }

        link_joint(world, joint_id);
    }

    sync_body_flags(world, body_index);
    update_body_mass_data(world, body_index);

    world.validate_solver_sets();
    let island_id = world.bodies[body_index as usize].island_id;
    validate_island(world, island_id);

    world.locked = false;
}

/// Disable a body (move to the disabled set, remove proxies). (b3Body_Disable)
pub fn body_disable(world: &mut World, body_id: BodyId) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;

    let body_index = get_body_full_id(world, body_id);
    if world.bodies[body_index as usize].set_index == DISABLED_SET {
        world.locked = false;
        return;
    }

    // Destroy contacts and wake bodies touching this body.
    destroy_body_contacts(world, body_index, true);

    let set_index = world.bodies[body_index as usize].set_index;

    // Unlink joints and transfer them to the disabled set
    let mut joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;
        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        if world.joints[joint_id as usize].set_index == DISABLED_SET {
            continue;
        }

        debug_assert!(
            world.joints[joint_id as usize].set_index == set_index || set_index == STATIC_SET
        );

        unlink_joint(world, joint_id);

        let joint_set = world.joints[joint_id as usize].set_index;
        transfer_joint(world, DISABLED_SET, joint_set, joint_id);
    }

    // Remove shapes from broad-phase
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next_shape_id = world.shapes[shape_id as usize].next_shape_id;
        {
            let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
            destroy_shape_proxy(&mut shapes[shape_id as usize], broad_phase);
        }
        shape_id = next_shape_id;
    }

    remove_body_from_island(world, body_index);
    transfer_body(world, DISABLED_SET, set_index, body_index);

    // C calls b3ValidateConnectivity here; that helper is not ported yet.
    world.validate_solver_sets();

    world.locked = false;
}

/// Enable a disabled body. (b3Body_Enable)
///
/// Matches C: does not lock the world for the duration of the operation.
pub fn body_enable(world: &mut World, body_id: BodyId) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let body_index = get_body_full_id(world, body_id);
    if world.bodies[body_index as usize].set_index != DISABLED_SET {
        return;
    }

    let set_id = if world.bodies[body_index as usize].type_ == BodyType::Static {
        STATIC_SET
    } else {
        AWAKE_SET
    };

    transfer_body(world, set_id, DISABLED_SET, body_index);

    let transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);
    let proxy_type = world.bodies[body_index as usize].type_;

    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next_shape_id = world.shapes[shape_id as usize].next_shape_id;
        {
            let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
            create_shape_proxy(
                &mut shapes[shape_id as usize],
                broad_phase,
                proxy_type,
                transform,
                true,
            );
        }
        shape_id = next_shape_id;
    }

    if set_id != STATIC_SET {
        create_island_for_body(world, set_id, body_index);
    }

    // Transfer joints. If the other body is disabled, don't transfer.
    let mut joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        debug_assert!(world.joints[joint_id as usize].set_index == DISABLED_SET);
        debug_assert!(world.joints[joint_id as usize].island_id == NULL_INDEX);

        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        let body_a = world.joints[joint_id as usize].edges[0].body_id;
        let body_b = world.joints[joint_id as usize].edges[1].body_id;

        if world.bodies[body_a as usize].set_index == DISABLED_SET
            || world.bodies[body_b as usize].set_index == DISABLED_SET
        {
            continue;
        }

        let joint_set_id = if world.bodies[body_a as usize].set_index == STATIC_SET
            && world.bodies[body_b as usize].set_index == STATIC_SET
        {
            STATIC_SET
        } else if world.bodies[body_a as usize].set_index == STATIC_SET {
            world.bodies[body_b as usize].set_index
        } else {
            world.bodies[body_a as usize].set_index
        };

        transfer_joint(world, joint_set_id, DISABLED_SET, joint_id);

        if joint_set_id != STATIC_SET {
            link_joint(world, joint_id);
        }
    }

    world.validate_solver_sets();
}
