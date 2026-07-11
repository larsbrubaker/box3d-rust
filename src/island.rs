// Port of the island data model from box3d-cpp-reference/src/island.h
// plus create/destroy/validate from island.c needed by body lifecycle.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::solver_set::{AWAKE_SET, FIRST_SLEEPING_SET};
use crate::world::World;

/// Cached contact data stored in the island for fast contiguous iteration.
/// Avoids touching Contact during union-find in island splitting.
/// (b3ContactLink)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactLink {
    pub contact_id: i32,
    pub body_id_a: i32,
    pub body_id_b: i32,
}

/// Cached joint data stored in the island for fast contiguous iteration.
/// (b3JointLink)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JointLink {
    pub joint_id: i32,
    pub body_id_a: i32,
    pub body_id_b: i32,
}

/// Persistent island for awake bodies, joints, and contacts. Contacts are
/// touching. Contacts and joints may connect to static bodies, but static
/// bodies are not in the island. (b3Island)
///
/// <https://en.wikipedia.org/wiki/Component_(graph_theory)>
/// <https://en.wikipedia.org/wiki/Dynamic_connectivity>
#[derive(Debug, Clone)]
pub struct Island {
    /// Index of solver set stored in World. May be NULL_INDEX.
    pub set_index: i32,

    /// Island index within set. May be NULL_INDEX.
    pub local_index: i32,

    pub island_id: i32,

    /// How many contacts have been removed from this island. Used to determine
    /// if an island is a candidate for splitting.
    pub constraint_remove_count: i32,

    pub bodies: Vec<i32>,

    /// Contacts and joints that belong to this island. May connect to static
    /// bodies not in the island. Each link carries the two body ids so island
    /// splitting's union-find never needs to touch Contact/Joint.
    pub contacts: Vec<ContactLink>,
    pub joints: Vec<JointLink>,
}

impl Default for Island {
    fn default() -> Self {
        Island {
            set_index: NULL_INDEX,
            local_index: NULL_INDEX,
            island_id: NULL_INDEX,
            constraint_remove_count: 0,
            bodies: Vec::new(),
            contacts: Vec::new(),
            joints: Vec::new(),
        }
    }
}

/// Used to move islands across solver sets. (b3IslandSim)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IslandSim {
    pub island_id: i32,
}

impl Default for IslandSim {
    fn default() -> Self {
        IslandSim {
            island_id: NULL_INDEX,
        }
    }
}

/// Create an empty island in the given set. Returns the island id.
/// (b3CreateIsland — C returns a pointer; Rust returns the id)
pub fn create_island(world: &mut World, set_index: i32) -> i32 {
    debug_assert!(set_index == AWAKE_SET || set_index >= FIRST_SLEEPING_SET);

    let island_id = world.island_id_pool.alloc_id();

    if island_id == world.islands.len() as i32 {
        world.islands.push(Island::default());
    } else {
        debug_assert!(world.islands[island_id as usize].set_index == NULL_INDEX);
    }

    let set = &mut world.solver_sets[set_index as usize];
    let local_index = set.island_sims.len() as i32;
    set.island_sims.push(IslandSim { island_id });

    let island = &mut world.islands[island_id as usize];
    island.set_index = set_index;
    island.local_index = local_index;
    island.island_id = island_id;
    island.bodies = Vec::new();
    island.contacts = Vec::new();
    island.joints = Vec::new();
    island.constraint_remove_count = 0;

    island_id
}

/// (b3DestroyIsland)
pub fn destroy_island(world: &mut World, island_id: i32) {
    if world.split_island_id == island_id {
        world.split_island_id = NULL_INDEX;
    }

    let (set_index, local_index) = {
        let island = &world.islands[island_id as usize];
        (island.set_index, island.local_index)
    };
    let set = &mut world.solver_sets[set_index as usize];
    {
        let last_index = set.island_sims.len() - 1;
        debug_assert!(0 <= local_index && (local_index as usize) <= last_index);
        let move_island_id = set.island_sims[last_index].island_id;
        set.island_sims.swap_remove(local_index as usize);
        world.islands[move_island_id as usize].local_index = local_index;
    }

    let island = &mut world.islands[island_id as usize];
    island.bodies = Vec::new();
    island.contacts = Vec::new();
    island.joints = Vec::new();
    island.constraint_remove_count = 0;
    island.local_index = NULL_INDEX;
    island.island_id = NULL_INDEX;
    island.set_index = NULL_INDEX;

    world.island_id_pool.free_id(island_id);
}

/// Validate island connectivity and bookkeeping. (b3ValidateIsland)
///
/// C compiles this only with B3_VALIDATE; here it always runs and asserts in
/// debug builds.
pub fn validate_island(world: &World, island_id: i32) {
    if island_id == NULL_INDEX {
        return;
    }

    let island = &world.islands[island_id as usize];
    debug_assert!(island.island_id == island_id);
    debug_assert!(island.set_index != NULL_INDEX);

    {
        debug_assert!(!island.bodies.is_empty());
        debug_assert!(island.bodies.len() as i32 <= world.body_id_pool.id_count());

        for (i, &body_id) in island.bodies.iter().enumerate() {
            let body = &world.bodies[body_id as usize];
            debug_assert!(body.island_id == island_id);
            debug_assert!(body.island_index == i as i32);
            debug_assert!(body.set_index == island.set_index);
            let _ = (body, i);
        }
    }

    if !island.contacts.is_empty() {
        debug_assert!(island.contacts.len() as i32 <= world.contact_id_pool.id_count());

        for (i, link) in island.contacts.iter().enumerate() {
            let contact = &world.contacts[link.contact_id as usize];
            debug_assert!(contact.set_index == island.set_index);
            debug_assert!(contact.island_id == island_id);
            debug_assert!(contact.island_index == i as i32);
            let _ = (contact, i);
        }
    }

    if !island.joints.is_empty() {
        debug_assert!(island.joints.len() as i32 <= world.joint_id_pool.id_count());

        for (i, link) in island.joints.iter().enumerate() {
            let joint = &world.joints[link.joint_id as usize];
            debug_assert!(joint.set_index == island.set_index);
            debug_assert!(joint.island_id == island_id);
            debug_assert!(joint.island_index == i as i32);
            let _ = (joint, i);
        }
    }
}
