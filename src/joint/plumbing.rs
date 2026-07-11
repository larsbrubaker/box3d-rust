// Joint storage plumbing from joint.c: id validation, sim lookup, and
// destruction.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::*;
use crate::body::wake_body;
use crate::constants::GRAPH_COLOR_COUNT;
use crate::constraint_graph::remove_joint_from_graph;
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::island::unlink_joint;
use crate::solver_set::{AWAKE_SET, DISABLED_SET};
use crate::world::World;

/// Validate a JointId and return the raw joint index. (b3GetJointFullId — C
/// returns a pointer; Rust returns the index into `world.joints`)
pub fn get_joint_full_id(world: &World, joint_id: JointId) -> i32 {
    let id = joint_id.index1 - 1;
    debug_assert!((id as usize) < world.joints.len());
    let joint = &world.joints[id as usize];
    debug_assert!(joint.joint_id == id && joint.generation == joint_id.generation);
    id
}

/// Joint id validity. (b3Joint_IsValid — world-registry checks collapse to the
/// world argument)
pub fn joint_is_valid(world: &World, id: JointId) -> bool {
    let joint_index = id.index1 - 1;
    if joint_index < 0 || world.joints.len() as i32 <= joint_index {
        return false;
    }

    let joint = &world.joints[joint_index as usize];
    if joint.joint_id == NULL_INDEX {
        return false;
    }

    debug_assert!(joint.joint_id == joint_index);
    id.generation == joint.generation
}

/// Borrow a joint's sim data mutably: constraint graph color for awake
/// joints, otherwise the owning solver set. (b3GetJointSim)
pub fn get_joint_sim(world: &mut World, joint_id: i32) -> &mut JointSim {
    let (set_index, color_index, local_index) = {
        let joint = &world.joints[joint_id as usize];
        (joint.set_index, joint.color_index, joint.local_index)
    };

    if set_index == AWAKE_SET {
        debug_assert!((0..GRAPH_COLOR_COUNT).contains(&color_index));
        &mut world.constraint_graph.colors[color_index as usize].joint_sims[local_index as usize]
    } else {
        &mut world.solver_sets[set_index as usize].joint_sims[local_index as usize]
    }
}

/// Shared-reference variant of get_joint_sim for read-only accessors.
pub fn get_joint_sim_ref(world: &World, joint_id: i32) -> &JointSim {
    let joint = &world.joints[joint_id as usize];

    if joint.set_index == AWAKE_SET {
        debug_assert!(0 <= joint.color_index && joint.color_index < GRAPH_COLOR_COUNT);
        &world.constraint_graph.colors[joint.color_index as usize].joint_sims
            [joint.local_index as usize]
    } else {
        &world.solver_sets[joint.set_index as usize].joint_sims[joint.local_index as usize]
    }
}

/// (b3GetJointSimCheckType)
pub fn get_joint_sim_check_type(
    world: &mut World,
    joint_id: JointId,
    joint_type: JointType,
) -> &mut JointSim {
    let id = get_joint_full_id(world, joint_id);
    debug_assert!(world.joints[id as usize].type_ == joint_type);
    let joint_sim = get_joint_sim(world, id);
    debug_assert!(joint_sim.type_ == joint_type);
    joint_sim
}

/// Shared-reference variant of get_joint_sim_check_type.
pub fn get_joint_sim_check_type_ref(
    world: &World,
    joint_id: JointId,
    joint_type: JointType,
) -> &JointSim {
    let id = get_joint_full_id(world, joint_id);
    debug_assert!(world.joints[id as usize].type_ == joint_type);
    let joint_sim = get_joint_sim_ref(world, id);
    debug_assert!(joint_sim.type_ == joint_type);
    joint_sim
}

/// Build the public JointId for a raw joint index.
pub(crate) fn make_joint_id(world: &World, joint_id: i32) -> JointId {
    JointId {
        index1: joint_id + 1,
        world0: world.world_id,
        generation: world.joints[joint_id as usize].generation,
    }
}

/// Destroy a joint: unlink it from both bodies' joint lists, the island
/// graph, and the solver set or constraint graph that owns its sim, then free
/// the id. (b3DestroyJointInternal — C takes the joint pointer; Rust takes the id.)
pub fn destroy_joint_internal(world: &mut World, joint_id: i32, wake_bodies: bool) {
    let (edge_a, edge_b) = {
        let joint = &world.joints[joint_id as usize];
        (joint.edges[0], joint.edges[1])
    };

    let id_a = edge_a.body_id;
    let id_b = edge_b.body_id;

    // Remove from body A
    if edge_a.prev_key != NULL_INDEX {
        let prev_joint = &mut world.joints[(edge_a.prev_key >> 1) as usize];
        prev_joint.edges[(edge_a.prev_key & 1) as usize].next_key = edge_a.next_key;
    }

    if edge_a.next_key != NULL_INDEX {
        let next_joint = &mut world.joints[(edge_a.next_key >> 1) as usize];
        next_joint.edges[(edge_a.next_key & 1) as usize].prev_key = edge_a.prev_key;
    }

    let edge_key_a = joint_id << 1;
    {
        let body_a = &mut world.bodies[id_a as usize];
        if body_a.head_joint_key == edge_key_a {
            body_a.head_joint_key = edge_a.next_key;
        }
        body_a.joint_count -= 1;
    }

    // Remove from body B
    if edge_b.prev_key != NULL_INDEX {
        let prev_joint = &mut world.joints[(edge_b.prev_key >> 1) as usize];
        prev_joint.edges[(edge_b.prev_key & 1) as usize].next_key = edge_b.next_key;
    }

    if edge_b.next_key != NULL_INDEX {
        let next_joint = &mut world.joints[(edge_b.next_key >> 1) as usize];
        next_joint.edges[(edge_b.next_key & 1) as usize].prev_key = edge_b.prev_key;
    }

    let edge_key_b = (joint_id << 1) | 1;
    {
        let body_b = &mut world.bodies[id_b as usize];
        if body_b.head_joint_key == edge_key_b {
            body_b.head_joint_key = edge_b.next_key;
        }
        body_b.joint_count -= 1;
    }

    if world.joints[joint_id as usize].island_id != NULL_INDEX {
        debug_assert!(world.joints[joint_id as usize].set_index > DISABLED_SET);
        unlink_joint(world, joint_id);
    } else {
        debug_assert!(world.joints[joint_id as usize].set_index <= DISABLED_SET);
    }

    // Remove joint from solver set that owns it
    let (set_index, local_index, color_index) = {
        let joint = &world.joints[joint_id as usize];
        (joint.set_index, joint.local_index, joint.color_index)
    };

    if set_index == AWAKE_SET {
        remove_joint_from_graph(world, id_a, id_b, color_index, local_index);
    } else {
        let set = &mut world.solver_sets[set_index as usize];
        let moved_index = set.joint_sims.len() as i32 - 1;
        set.joint_sims.swap_remove(local_index as usize);
        if moved_index != local_index {
            // Fix moved joint
            let moved_id =
                world.solver_sets[set_index as usize].joint_sims[local_index as usize].joint_id;
            let moved_joint = &mut world.joints[moved_id as usize];
            debug_assert!(moved_joint.local_index == moved_index);
            moved_joint.local_index = local_index;
        }
    }

    // Free joint and id (preserve joint generation)
    {
        let joint = &mut world.joints[joint_id as usize];
        joint.set_index = NULL_INDEX;
        joint.local_index = NULL_INDEX;
        joint.color_index = NULL_INDEX;
        joint.joint_id = NULL_INDEX;
    }
    world.joint_id_pool.free_id(joint_id);

    if wake_bodies {
        wake_body(world, id_a);
        wake_body(world, id_b);
    }

    world.validate_solver_sets();
}

/// (b3DestroyJoint)
pub fn destroy_joint(world: &mut World, joint_id: JointId, wake_attached: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let id = get_joint_full_id(world, joint_id);
    destroy_joint_internal(world, id, wake_attached);
}
