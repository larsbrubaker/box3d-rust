// Port of the island data model from box3d-cpp-reference/src/island.h
// plus create/destroy/validate from island.c needed by body lifecycle.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::solver_set::{AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET, STATIC_SET};
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

/// Merge two islands, keeping the larger. Either id may be NULL_INDEX (static).
/// (b3MergeIslands)
fn merge_islands(world: &mut World, island_id_a: i32, island_id_b: i32) -> i32 {
    if island_id_a == island_id_b {
        return island_id_a;
    }
    if island_id_a == NULL_INDEX {
        debug_assert!(island_id_b != NULL_INDEX);
        return island_id_b;
    }
    if island_id_b == NULL_INDEX {
        debug_assert!(island_id_a != NULL_INDEX);
        return island_id_a;
    }

    let (big_id, small_id) = {
        let count_a = world.islands[island_id_a as usize].bodies.len();
        let count_b = world.islands[island_id_b as usize].bodies.len();
        if count_a >= count_b {
            (island_id_a, island_id_b)
        } else {
            (island_id_b, island_id_a)
        }
    };

    let small_bodies = std::mem::take(&mut world.islands[small_id as usize].bodies);
    for body_id in small_bodies {
        debug_assert!(world.bodies[body_id as usize].island_id == small_id);
        let island_index = world.islands[big_id as usize].bodies.len() as i32;
        world.bodies[body_id as usize].island_id = big_id;
        world.bodies[body_id as usize].island_index = island_index;
        world.islands[big_id as usize].bodies.push(body_id);
    }

    let small_contacts = std::mem::take(&mut world.islands[small_id as usize].contacts);
    for link in small_contacts {
        let contact = &mut world.contacts[link.contact_id as usize];
        contact.island_id = big_id;
        contact.island_index = world.islands[big_id as usize].contacts.len() as i32;
        world.islands[big_id as usize].contacts.push(link);
    }

    let small_joints = std::mem::take(&mut world.islands[small_id as usize].joints);
    for link in small_joints {
        let joint = &mut world.joints[link.joint_id as usize];
        joint.island_id = big_id;
        joint.island_index = world.islands[big_id as usize].joints.len() as i32;
        world.islands[big_id as usize].joints.push(link);
    }

    world.islands[big_id as usize].constraint_remove_count +=
        world.islands[small_id as usize].constraint_remove_count;

    destroy_island(world, small_id);
    validate_island(world, big_id);
    big_id
}

/// (b3AddContactToIsland)
fn add_contact_to_island(world: &mut World, island_id: i32, contact_id: i32) {
    debug_assert!(world.contacts[contact_id as usize].island_id == NULL_INDEX);
    debug_assert!(world.contacts[contact_id as usize].island_index == NULL_INDEX);

    let island_index = world.islands[island_id as usize].contacts.len() as i32;
    let link = ContactLink {
        contact_id,
        body_id_a: world.contacts[contact_id as usize].edges[0].body_id,
        body_id_b: world.contacts[contact_id as usize].edges[1].body_id,
    };

    world.contacts[contact_id as usize].island_id = island_id;
    world.contacts[contact_id as usize].island_index = island_index;
    world.islands[island_id as usize].contacts.push(link);

    validate_island(world, island_id);
}

/// Link a touching contact into an island, waking sleeping partners and merging
/// as needed. (b3LinkContact)
pub fn link_contact(world: &mut World, contact_id: i32) {
    use crate::contact::contact_flags;
    use crate::solver_set::wake_solver_set;

    debug_assert!((world.contacts[contact_id as usize].flags & contact_flags::TOUCHING) != 0);

    let body_id_a = world.contacts[contact_id as usize].edges[0].body_id;
    let body_id_b = world.contacts[contact_id as usize].edges[1].body_id;

    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;
    debug_assert!(set_a != DISABLED_SET && set_b != DISABLED_SET);
    debug_assert!(set_a != STATIC_SET || set_b != STATIC_SET);

    // Wake bodyB if bodyA is awake and bodyB is sleeping
    if set_a == AWAKE_SET && set_b >= FIRST_SLEEPING_SET {
        wake_solver_set(world, set_b);
    }

    // Wake bodyA if bodyB is awake and bodyA is sleeping
    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;
    if set_b == AWAKE_SET && set_a >= FIRST_SLEEPING_SET {
        wake_solver_set(world, set_a);
    }

    let island_id_a = world.bodies[body_id_a as usize].island_id;
    let island_id_b = world.bodies[body_id_b as usize].island_id;

    debug_assert!(
        world.bodies[body_id_a as usize].set_index != STATIC_SET || island_id_a == NULL_INDEX
    );
    debug_assert!(
        world.bodies[body_id_b as usize].set_index != STATIC_SET || island_id_b == NULL_INDEX
    );
    debug_assert!(island_id_a != NULL_INDEX || island_id_b != NULL_INDEX);

    let final_island_id = merge_islands(world, island_id_a, island_id_b);
    add_contact_to_island(world, final_island_id, contact_id);
}

/// Remove a contact from its island. (b3UnlinkContact)
pub fn unlink_contact(world: &mut World, contact_id: i32) {
    let island_id = world.contacts[contact_id as usize].island_id;
    debug_assert!(island_id != NULL_INDEX);

    let remove_index = world.contacts[contact_id as usize].island_index;
    let island = &mut world.islands[island_id as usize];
    debug_assert!(0 <= remove_index && (remove_index as usize) < island.contacts.len());
    debug_assert!(island.contacts[remove_index as usize].contact_id == contact_id);

    let moved_index = island.contacts.len() as i32 - 1;
    island.contacts.swap_remove(remove_index as usize);
    if moved_index != remove_index {
        let moved_contact_id = island.contacts[remove_index as usize].contact_id;
        debug_assert!(world.contacts[moved_contact_id as usize].island_index == moved_index);
        world.contacts[moved_contact_id as usize].island_index = remove_index;
    }

    world.contacts[contact_id as usize].island_id = NULL_INDEX;
    world.contacts[contact_id as usize].island_index = NULL_INDEX;
    world.islands[island_id as usize].constraint_remove_count += 1;

    validate_island(world, island_id);
}

/// (b3AddJointToIsland)
fn add_joint_to_island(world: &mut World, island_id: i32, joint_id: i32) {
    debug_assert!(world.joints[joint_id as usize].island_id == NULL_INDEX);
    debug_assert!(world.joints[joint_id as usize].island_index == NULL_INDEX);

    let island_index = world.islands[island_id as usize].joints.len() as i32;
    let link = JointLink {
        joint_id,
        body_id_a: world.joints[joint_id as usize].edges[0].body_id,
        body_id_b: world.joints[joint_id as usize].edges[1].body_id,
    };

    world.joints[joint_id as usize].island_id = island_id;
    world.joints[joint_id as usize].island_index = island_index;
    world.islands[island_id as usize].joints.push(link);

    validate_island(world, island_id);
}

/// Link a joint into the island graph when it is created. (b3LinkJoint)
pub fn link_joint(world: &mut World, joint_id: i32) {
    use crate::solver_set::wake_solver_set;
    use crate::types::BodyType;

    let body_id_a = world.joints[joint_id as usize].edges[0].body_id;
    let body_id_b = world.joints[joint_id as usize].edges[1].body_id;

    debug_assert!(
        world.bodies[body_id_a as usize].type_ == BodyType::Dynamic
            || world.bodies[body_id_b as usize].type_ == BodyType::Dynamic
    );

    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;

    if set_a == AWAKE_SET && set_b >= FIRST_SLEEPING_SET {
        wake_solver_set(world, set_b);
    } else if set_b == AWAKE_SET && set_a >= FIRST_SLEEPING_SET {
        wake_solver_set(world, set_a);
    }

    let island_id_a = world.bodies[body_id_a as usize].island_id;
    let island_id_b = world.bodies[body_id_b as usize].island_id;

    debug_assert!(island_id_a != NULL_INDEX || island_id_b != NULL_INDEX);

    // Merge islands. This will destroy one of the islands.
    let final_island_id = merge_islands(world, island_id_a, island_id_b);

    // Add joint to the island that survived
    add_joint_to_island(world, final_island_id, joint_id);
}

/// Unlink a joint from the island graph when it is destroyed. (b3UnlinkJoint)
pub fn unlink_joint(world: &mut World, joint_id: i32) {
    let island_id = world.joints[joint_id as usize].island_id;
    if island_id == NULL_INDEX {
        return;
    }

    let remove_index = world.joints[joint_id as usize].island_index;
    let island = &mut world.islands[island_id as usize];
    debug_assert!(0 <= remove_index && (remove_index as usize) < island.joints.len());
    debug_assert!(island.joints[remove_index as usize].joint_id == joint_id);

    let moved_index = island.joints.len() as i32 - 1;
    island.joints.swap_remove(remove_index as usize);
    if moved_index != remove_index {
        // Fix islandIndex on the joint that was swapped into removeIndex
        let moved_joint_id = island.joints[remove_index as usize].joint_id;
        debug_assert!(world.joints[moved_joint_id as usize].island_index == moved_index);
        world.joints[moved_joint_id as usize].island_index = remove_index;
    }

    world.joints[joint_id as usize].island_id = NULL_INDEX;
    world.joints[joint_id as usize].island_index = NULL_INDEX;
    world.islands[island_id as usize].constraint_remove_count += 1;

    validate_island(world, island_id);
}

/// Find parent of a node. Use path halving to speed up further queries.
/// (b3IslandFindParent)
fn island_find_parent(parents: &mut [i32], mut node: i32) -> i32 {
    // Walk the chain of parents to find the node that is its own parent (the root)
    while parents[node as usize] != node {
        let grand_parent = parents[parents[node as usize] as usize];
        parents[node as usize] = grand_parent;
        node = grand_parent;
    }

    node
}

/// Connect the components containing node1 and node2.
/// Uses rank to keep tree balanced. Tracks per-component contact and joint counts.
/// (b3IslandUnion)
fn island_union(
    parents: &mut [i32],
    ranks: &mut [i32],
    node1: i32,
    node2: i32,
    contact_counts: &mut [i32],
    joint_counts: &mut [i32],
) {
    let root1 = island_find_parent(parents, node1) as usize;
    let root2 = island_find_parent(parents, node2) as usize;
    if root1 != root2 {
        if ranks[root1] < ranks[root2] {
            parents[root1] = root2 as i32;
            contact_counts[root2] += contact_counts[root1];
            joint_counts[root2] += joint_counts[root1];
        } else if ranks[root1] > ranks[root2] {
            parents[root2] = root1 as i32;
            contact_counts[root1] += contact_counts[root2];
            joint_counts[root1] += joint_counts[root2];
        } else {
            parents[root2] = root1 as i32;
            ranks[root1] += 1;
            contact_counts[root1] += contact_counts[root2];
            joint_counts[root1] += joint_counts[root2];
        }
    }
}

/// Split an island because some contacts and/or joints have been removed.
/// This uses union find and touches a lot of memory, so it can be slow.
/// (b3SplitIsland)
///
/// Note: contacts/joints connected to static bodies must belong to an island
/// but don't affect island connectivity.
/// Note: static bodies are never in an island.
///
/// <https://en.wikipedia.org/wiki/Disjoint-set_data_structure>
pub fn split_island(world: &mut World, base_id: i32) {
    debug_assert!(world.islands[base_id as usize].constraint_remove_count > 0);
    debug_assert!(world.islands[base_id as usize].set_index == AWAKE_SET);

    validate_island(world, base_id);

    // Take the base island's arrays. C detaches the raw buffers so
    // b3DestroyIsland won't free them; mem::take is the Rust equivalent.
    let base_body_ids = std::mem::take(&mut world.islands[base_id as usize].bodies);
    let base_contacts = std::mem::take(&mut world.islands[base_id as usize].contacts);
    let base_joints = std::mem::take(&mut world.islands[base_id as usize].joints);

    let base_body_count = base_body_ids.len();

    // C allocates the union-find scratch from the arena; the Rust step scratch
    // is plain Vecs.
    let mut parents: Vec<i32> = (0..base_body_count as i32).collect();
    let mut contact_counts: Vec<i32> = vec![0; base_body_count];
    let mut joint_counts: Vec<i32> = vec![0; base_body_count];
    let mut ranks: Vec<i32> = vec![0; base_body_count];

    // Union over contacts, tracking per-component contact counts
    for link in &base_contacts {
        debug_assert!(0 <= link.body_id_a && (link.body_id_a as usize) < world.bodies.len());
        debug_assert!(0 <= link.body_id_b && (link.body_id_b as usize) < world.bodies.len());
        let island_index_a = world.bodies[link.body_id_a as usize].island_index;
        let island_index_b = world.bodies[link.body_id_b as usize].island_index;

        // Only connect non-static bodies
        if island_index_a != NULL_INDEX && island_index_b != NULL_INDEX {
            debug_assert!(0 <= island_index_a && (island_index_a as usize) < base_body_count);
            debug_assert!(0 <= island_index_b && (island_index_b as usize) < base_body_count);
            island_union(
                &mut parents,
                &mut ranks,
                island_index_a,
                island_index_b,
                &mut contact_counts,
                &mut joint_counts,
            );
            let root = island_find_parent(&mut parents, island_index_a);
            contact_counts[root as usize] += 1;
        } else {
            let island_index = if island_index_a != NULL_INDEX {
                island_index_a
            } else {
                island_index_b
            };
            let root = island_find_parent(&mut parents, island_index);
            contact_counts[root as usize] += 1;
        }
    }

    // Union over joints, tracking per-component joint counts
    for link in &base_joints {
        debug_assert!(0 <= link.body_id_a && (link.body_id_a as usize) < world.bodies.len());
        debug_assert!(0 <= link.body_id_b && (link.body_id_b as usize) < world.bodies.len());
        let island_index_a = world.bodies[link.body_id_a as usize].island_index;
        let island_index_b = world.bodies[link.body_id_b as usize].island_index;

        // Only connect non-static bodies
        if island_index_a != NULL_INDEX && island_index_b != NULL_INDEX {
            debug_assert!(0 <= island_index_a && (island_index_a as usize) < base_body_count);
            debug_assert!(0 <= island_index_b && (island_index_b as usize) < base_body_count);
            island_union(
                &mut parents,
                &mut ranks,
                island_index_a,
                island_index_b,
                &mut contact_counts,
                &mut joint_counts,
            );
            let root = island_find_parent(&mut parents, island_index_a);
            joint_counts[root as usize] += 1;
        } else {
            let island_index = if island_index_a != NULL_INDEX {
                island_index_a
            } else {
                island_index_b
            };
            let root = island_find_parent(&mut parents, island_index);
            joint_counts[root as usize] += 1;
        }
    }

    // Done with ranks
    drop(ranks);

    // Flatten all parent indices and count connected components.
    let mut component_count = 0;
    for i in 0..base_body_count {
        parents[i] = island_find_parent(&mut parents, i as i32);
        if parents[i] == i as i32 {
            component_count += 1;
        }
    }

    // Early return — island is still fully connected, no split needed.
    if component_count == 1 {
        let base_island = &mut world.islands[base_id as usize];
        base_island.constraint_remove_count = 0;
        base_island.bodies = base_body_ids;
        base_island.contacts = base_contacts;
        base_island.joints = base_joints;
        return;
    }

    // Map from body index to new island index. Only set for root bodies.
    let mut root_map: Vec<i32> = vec![NULL_INDEX; base_body_count];

    let mut component_body_counts: Vec<i32> = vec![0; component_count];
    let mut component_contact_counts: Vec<i32> = vec![0; component_count];
    let mut component_joint_counts: Vec<i32> = vec![0; component_count];
    let mut island_count = 0usize;

    // Find the root body for each body and create islands as needed.
    // Extract per-component counts from the root nodes' accumulated counts.
    for &parent in parents.iter().take(base_body_count) {
        let root_index = parent as usize;
        if root_map[root_index] == NULL_INDEX {
            root_map[root_index] = island_count as i32;
            component_body_counts[island_count] = 0;
            component_contact_counts[island_count] = contact_counts[root_index];
            component_joint_counts[island_count] = joint_counts[root_index];
            island_count += 1;
        }

        component_body_counts[root_map[root_index] as usize] += 1;
    }

    debug_assert!(island_count == component_count);

    // Map from new island index to island id
    let mut island_ids: Vec<i32> = Vec::with_capacity(island_count);

    // Create new islands and reserve body/contact/joint arrays
    for i in 0..island_count {
        let new_island_id = create_island(world, AWAKE_SET);
        island_ids.push(new_island_id);

        // Reserve arrays to avoid wasteful growth.
        let new_island = &mut world.islands[new_island_id as usize];
        new_island.bodies.reserve(component_body_counts[i] as usize);
        new_island
            .contacts
            .reserve(component_contact_counts[i] as usize);
        new_island
            .joints
            .reserve(component_joint_counts[i] as usize);
    }

    // Assign bodies to new islands
    for (i, &body_id) in base_body_ids.iter().enumerate() {
        let root = island_find_parent(&mut parents, i as i32);
        let new_island_id = island_ids[root_map[root as usize] as usize];

        let island_index = world.islands[new_island_id as usize].bodies.len() as i32;
        debug_assert!(
            (island_index as usize) < world.islands[new_island_id as usize].bodies.capacity()
        );
        world.islands[new_island_id as usize].bodies.push(body_id);

        let body = &mut world.bodies[body_id as usize];
        body.island_id = new_island_id;
        body.island_index = island_index;
    }

    // Assign contacts to the island of their bodies
    for link in &base_contacts {
        // Static bodies don't have an island id.
        let island_id_a = world.bodies[link.body_id_a as usize].island_id;
        let target_island_id = if island_id_a != NULL_INDEX {
            island_id_a
        } else {
            world.bodies[link.body_id_b as usize].island_id
        };

        let target_island = &mut world.islands[target_island_id as usize];
        let island_index = target_island.contacts.len() as i32;
        debug_assert!((island_index as usize) < target_island.contacts.capacity());
        target_island.contacts.push(*link);

        let contact = &mut world.contacts[link.contact_id as usize];
        contact.island_id = target_island_id;
        contact.island_index = island_index;
    }

    // Assign joints to the island of their bodies
    for link in &base_joints {
        // Static bodies don't have an island id.
        let island_id_a = world.bodies[link.body_id_a as usize].island_id;
        let target_island_id = if island_id_a != NULL_INDEX {
            island_id_a
        } else {
            world.bodies[link.body_id_b as usize].island_id
        };

        let target_island = &mut world.islands[target_island_id as usize];
        let island_index = target_island.joints.len() as i32;
        debug_assert!((island_index as usize) < target_island.joints.capacity());
        target_island.joints.push(*link);

        let joint = &mut world.joints[link.joint_id as usize];
        joint.island_id = target_island_id;
        joint.island_index = island_index;
    }

    // Destroy the base island
    destroy_island(world, base_id);
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
