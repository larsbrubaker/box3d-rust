// Port of solver_set.h and the wake/sleep/transfer logic from solver_set.c.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{BodySim, BodyState};
use crate::core::NULL_INDEX;
use crate::island::IslandSim;
use crate::joint::JointSim;
use crate::world::World;

/// Static set for static bodies and joints between static bodies. (b3_staticSet)
pub const STATIC_SET: i32 = 0;
/// Disabled set for disabled bodies and their joints. (b3_disabledSet)
pub const DISABLED_SET: i32 = 1;
/// Awake set for awake bodies with body states. (b3_awakeSet)
pub const AWAKE_SET: i32 = 2;
/// Index of the first sleeping set. (b3_firstSleepingSet)
pub const FIRST_SLEEPING_SET: i32 = 3;

/// Solver set data for contiguous storage of sims.
///
/// Sets used:
/// - static set for all static bodies and joints between static bodies
/// - active set for all active bodies with body states (no contacts or joints)
/// - disabled set for disabled bodies and their joints
/// - all further sets are sleeping island sets along with their contacts and joints
///
/// Purpose: high memory locality.
/// <https://www.youtube.com/watch?v=nZNd5FjSquk> (b3SolverSet)
#[derive(Debug, Clone, Default)]
pub struct SolverSet {
    /// Body array. Empty for unused set.
    pub body_sims: Vec<BodySim>,

    /// Body state only exists for active set
    pub body_states: Vec<BodyState>,

    /// Sleeping/disabled joints. Empty for static/active set.
    pub joint_sims: Vec<JointSim>,

    /// All contacts for sleeping sets; non-touching contacts for the awake set.
    /// Empty for static and disabled sets. (C: b3Array(int) contactIndices)
    pub contact_indices: Vec<i32>,

    /// Awake set has an array of islands. Sleeping sets normally have a single
    /// island; joints between sleeping sets cause merges with multiple islands.
    /// Static and disabled sets have no islands.
    pub island_sims: Vec<IslandSim>,

    /// Aligns with World::solver_set_id_pool.
    pub set_index: i32,
}

impl SolverSet {
    /// Empty set marked free. (slot after destroy)
    pub fn free_slot() -> SolverSet {
        SolverSet {
            set_index: NULL_INDEX,
            ..Default::default()
        }
    }
}

/// (b3DestroySolverSet)
pub fn destroy_solver_set(world: &mut World, set_index: i32) {
    let set = &mut world.solver_sets[set_index as usize];
    *set = SolverSet::default();
    set.set_index = NULL_INDEX;
    world.solver_set_id_pool.free_id(set_index);
}

/// Wake a solver set. Does not merge islands. (b3WakeSolverSet)
///
/// Contacts can be in several places:
/// 1. non-touching contacts in the disabled set
/// 2. non-touching contacts already in the awake set
/// 3. touching contacts in the sleeping set
///
/// This handles contact types 1 and 3. Type 2 doesn't need any action.
pub fn wake_solver_set(world: &mut World, set_index: i32) {
    use crate::body::IDENTITY_BODY_STATE;
    use crate::constraint_graph::{add_contact_to_graph, add_joint_to_graph};
    use crate::contact::contact_flags;

    debug_assert!(set_index >= FIRST_SLEEPING_SET);

    let body_count = world.solver_sets[set_index as usize].body_sims.len();
    for i in 0..body_count {
        let sim_src = world.solver_sets[set_index as usize].body_sims[i];
        let body_id = sim_src.body_id;

        debug_assert!(world.bodies[body_id as usize].set_index == set_index);
        let awake_body_count = world.solver_sets[AWAKE_SET as usize].body_sims.len() as i32;
        let flags = {
            let body = &mut world.bodies[body_id as usize];
            body.set_index = AWAKE_SET;
            body.local_index = awake_body_count;
            // Reset sleep timer
            body.sleep_time = 0.0;
            body.flags
        };

        world.solver_sets[AWAKE_SET as usize]
            .body_sims
            .push(sim_src);

        let mut state = IDENTITY_BODY_STATE;
        state.flags = flags;
        world.solver_sets[AWAKE_SET as usize]
            .body_states
            .push(state);

        // move non-touching contacts from disabled set to awake set
        let mut contact_key = world.bodies[body_id as usize].head_contact_key;
        while contact_key != NULL_INDEX {
            let edge_index = (contact_key & 1) as usize;
            let contact_id = contact_key >> 1;

            contact_key = world.contacts[contact_id as usize].edges[edge_index].next_key;

            if world.contacts[contact_id as usize].set_index != DISABLED_SET {
                debug_assert!(
                    world.contacts[contact_id as usize].set_index == AWAKE_SET
                        || world.contacts[contact_id as usize].set_index == set_index
                );
                continue;
            }

            let local_index = world.contacts[contact_id as usize].local_index;

            debug_assert!(
                0 <= local_index
                    && (local_index as usize)
                        < world.solver_sets[DISABLED_SET as usize]
                            .contact_indices
                            .len()
            );
            debug_assert!(
                world.solver_sets[DISABLED_SET as usize].contact_indices[local_index as usize]
                    == contact_id
            );

            debug_assert!(
                (world.contacts[contact_id as usize].flags & contact_flags::TOUCHING) == 0
                    && world.contacts[contact_id as usize].manifold_count() == 0
            );

            {
                let awake_contact_count =
                    world.solver_sets[AWAKE_SET as usize].contact_indices.len() as i32;
                let contact = &mut world.contacts[contact_id as usize];
                contact.set_index = AWAKE_SET;
                contact.local_index = awake_contact_count;
            }
            world.solver_sets[AWAKE_SET as usize]
                .contact_indices
                .push(contact_id);

            let disabled_contacts = &mut world.solver_sets[DISABLED_SET as usize].contact_indices;
            let moved_local_index = disabled_contacts.len() as i32 - 1;
            disabled_contacts.swap_remove(local_index as usize);
            if moved_local_index != local_index {
                // fix moved element
                let moved_contact_index = disabled_contacts[local_index as usize];
                let moved_contact = &mut world.contacts[moved_contact_index as usize];
                debug_assert!(moved_contact.local_index == moved_local_index);
                moved_contact.local_index = local_index;
            }
        }
    }

    // Transfer touching contacts from sleeping set to constraint graph.
    {
        let contact_count = world.solver_sets[set_index as usize].contact_indices.len();
        for i in 0..contact_count {
            let contact_id = world.solver_sets[set_index as usize].contact_indices[i];
            debug_assert!(
                (world.contacts[contact_id as usize].flags & contact_flags::TOUCHING) != 0
            );
            debug_assert!(
                (world.contacts[contact_id as usize].flags & contact_flags::SIM_TOUCHING) != 0
            );
            debug_assert!(world.contacts[contact_id as usize].set_index == set_index);
            add_contact_to_graph(world, contact_id);
            world.contacts[contact_id as usize].set_index = AWAKE_SET;
        }
    }

    // transfer joints from sleeping set to awake set
    {
        let joint_count = world.solver_sets[set_index as usize].joint_sims.len();
        for i in 0..joint_count {
            let joint_sim = world.solver_sets[set_index as usize].joint_sims[i];
            let joint_id = joint_sim.joint_id;
            debug_assert!(world.joints[joint_id as usize].set_index == set_index);
            add_joint_to_graph(world, joint_sim, joint_id);
            world.joints[joint_id as usize].set_index = AWAKE_SET;
        }
    }

    // transfer island from sleeping set to awake set
    // Usually a sleeping set has only one island, but it is possible
    // that joints are created between sleeping islands and they
    // are moved to the same sleeping set.
    let island_count = world.solver_sets[set_index as usize].island_sims.len();
    for i in 0..island_count {
        let island_src = world.solver_sets[set_index as usize].island_sims[i];
        let island_id = island_src.island_id;
        let awake_island_count = world.solver_sets[AWAKE_SET as usize].island_sims.len() as i32;
        {
            let island = &mut world.islands[island_id as usize];
            island.set_index = AWAKE_SET;
            island.local_index = awake_island_count;
        }
        world.solver_sets[AWAKE_SET as usize]
            .island_sims
            .push(island_src);
    }

    // destroy the sleeping set
    destroy_solver_set(world, set_index);
}

/// Move a sleeping island (bodies, contacts, joints) from the awake set into a
/// new sleeping solver set. (b3TrySleepIsland)
pub fn try_sleep_island(world: &mut World, island_id: i32) {
    use crate::constants::GRAPH_COLOR_COUNT;
    use crate::constraint_graph::OVERFLOW_INDEX;
    use crate::contact::contact_flags;

    debug_assert!(world.islands[island_id as usize].set_index == AWAKE_SET);

    // Cannot put an island to sleep while it has a pending split and more than one body.
    if world.islands[island_id as usize].constraint_remove_count > 0
        && world.islands[island_id as usize].bodies.len() > 1
    {
        return;
    }

    // island is sleeping
    // - create new sleeping solver set
    // - move island to sleeping solver set
    // - identify non-touching contacts that should move to sleeping solver set or disabled set
    // - remove old island
    // - fix island
    let sleep_set_id = world.solver_set_id_pool.alloc_id();
    if sleep_set_id == world.solver_sets.len() as i32 {
        world.solver_sets.push(SolverSet::free_slot());
    }

    {
        let island = &world.islands[island_id as usize];
        debug_assert!(
            0 <= island.local_index
                && (island.local_index as usize)
                    < world.solver_sets[AWAKE_SET as usize].island_sims.len()
        );

        let body_capacity = island.bodies.len();
        let contact_capacity = island.contacts.len();
        let joint_capacity = island.joints.len();

        let sleep_set = &mut world.solver_sets[sleep_set_id as usize];
        *sleep_set = SolverSet::default();
        sleep_set.set_index = sleep_set_id;
        sleep_set.body_sims.reserve(body_capacity);
        sleep_set.contact_indices.reserve(contact_capacity);
        sleep_set.joint_sims.reserve(joint_capacity);
    }

    // move awake bodies to sleeping set
    // this shuffles around bodies in the awake set
    {
        let body_count = world.islands[island_id as usize].bodies.len();
        for i in 0..body_count {
            let body_id = world.islands[island_id as usize].bodies[i];
            debug_assert!(world.bodies[body_id as usize].set_index == AWAKE_SET);
            debug_assert!(world.bodies[body_id as usize].island_id == island_id);
            debug_assert!(world.bodies[body_id as usize].island_index == i as i32);

            // Update the body move event to indicate this body fell asleep
            // It could happen the body is forced asleep before it ever moves.
            let body_move_index = world.bodies[body_id as usize].body_move_index;
            if body_move_index != NULL_INDEX {
                let move_event = &mut world.body_move_events[body_move_index as usize];
                debug_assert!(move_event.body_id.index1 - 1 == body_id);
                debug_assert!(
                    move_event.body_id.generation == world.bodies[body_id as usize].generation
                );
                move_event.fell_asleep = true;
                world.bodies[body_id as usize].body_move_index = NULL_INDEX;
            }

            let awake_body_index = world.bodies[body_id as usize].local_index;

            // move body sim to sleep set
            let awake_sim =
                world.solver_sets[AWAKE_SET as usize].body_sims[awake_body_index as usize];
            let sleep_body_index = world.solver_sets[sleep_set_id as usize].body_sims.len() as i32;
            world.solver_sets[sleep_set_id as usize]
                .body_sims
                .push(awake_sim);

            {
                let awake_sims = &mut world.solver_sets[AWAKE_SET as usize].body_sims;
                let moved_index = awake_sims.len() as i32 - 1;
                awake_sims.swap_remove(awake_body_index as usize);
                if moved_index != awake_body_index {
                    // fix local index on moved element
                    let moved_id = awake_sims[awake_body_index as usize].body_id;
                    let moved_body = &mut world.bodies[moved_id as usize];
                    debug_assert!(moved_body.local_index == moved_index);
                    moved_body.local_index = awake_body_index;
                }
            }

            // destroy state, no need to clone
            world.solver_sets[AWAKE_SET as usize]
                .body_states
                .swap_remove(awake_body_index as usize);

            {
                let body = &mut world.bodies[body_id as usize];
                body.set_index = sleep_set_id;
                body.local_index = sleep_body_index;
            }

            // Move non-touching contacts to the disabled set.
            // Non-touching contacts may exist between sleeping islands and there is no clear ownership.
            let mut contact_key = world.bodies[body_id as usize].head_contact_key;
            while contact_key != NULL_INDEX {
                let contact_id = contact_key >> 1;
                let edge_index = (contact_key & 1) as usize;

                debug_assert!(
                    world.contacts[contact_id as usize].set_index == AWAKE_SET
                        || world.contacts[contact_id as usize].set_index == DISABLED_SET
                );
                contact_key = world.contacts[contact_id as usize].edges[edge_index].next_key;

                if world.contacts[contact_id as usize].set_index == DISABLED_SET {
                    // already moved to disabled set by another body in the island
                    continue;
                }

                if world.contacts[contact_id as usize].color_index != NULL_INDEX {
                    // contact is touching and will be moved separately
                    debug_assert!(
                        (world.contacts[contact_id as usize].flags & contact_flags::TOUCHING) != 0
                    );
                    continue;
                }

                // the other body may still be awake, it still may go to sleep and then it will be responsible
                // for moving this contact to the disabled set.
                let other_edge_index = edge_index ^ 1;
                let other_body_id =
                    world.contacts[contact_id as usize].edges[other_edge_index].body_id;
                if world.bodies[other_body_id as usize].set_index == AWAKE_SET {
                    continue;
                }

                let local_index = world.contacts[contact_id as usize].local_index;
                debug_assert!(
                    world.solver_sets[AWAKE_SET as usize].contact_indices[local_index as usize]
                        == contact_id
                );

                debug_assert!(world.contacts[contact_id as usize].manifold_count() == 0);
                debug_assert!(
                    (world.contacts[contact_id as usize].flags & contact_flags::TOUCHING) == 0
                );

                // Move the non-touching contact to the disabled set.
                {
                    let disabled_contact_count = world.solver_sets[DISABLED_SET as usize]
                        .contact_indices
                        .len() as i32;
                    let contact = &mut world.contacts[contact_id as usize];
                    contact.set_index = DISABLED_SET;

                    // This is mandatory for validation to work correctly
                    contact.local_index = disabled_contact_count;
                }
                world.solver_sets[DISABLED_SET as usize]
                    .contact_indices
                    .push(contact_id);

                let awake_contacts = &mut world.solver_sets[AWAKE_SET as usize].contact_indices;
                let moved_local_index = awake_contacts.len() as i32 - 1;
                awake_contacts.swap_remove(local_index as usize);
                if moved_local_index != local_index {
                    // fix moved element
                    let moved_contact_index = awake_contacts[local_index as usize];
                    let moved_contact = &mut world.contacts[moved_contact_index as usize];
                    debug_assert!(moved_contact.local_index == moved_local_index);
                    moved_contact.local_index = local_index;
                }
            }
        }
    }

    // move touching contacts to sleeping set
    // this shuffles contacts in the awake set
    {
        let contact_count = world.islands[island_id as usize].contacts.len();
        for i in 0..contact_count {
            let contact_id = world.islands[island_id as usize].contacts[i].contact_id;
            debug_assert!(world.contacts[contact_id as usize].set_index == AWAKE_SET);
            debug_assert!(world.contacts[contact_id as usize].island_id == island_id);
            debug_assert!(world.contacts[contact_id as usize].island_index == i as i32);
            let color_index = world.contacts[contact_id as usize].color_index;
            debug_assert!(0 <= color_index && color_index < GRAPH_COLOR_COUNT);

            // Remove bodies from graph coloring associated with this constraint
            if color_index != OVERFLOW_INDEX {
                // might clear a bit for a static body, but this has no effect
                let body_id_a = world.contacts[contact_id as usize].edges[0].body_id;
                let body_id_b = world.contacts[contact_id as usize].edges[1].body_id;
                let color = &mut world.constraint_graph.colors[color_index as usize];
                color.body_set.clear_bit(body_id_a as u32);
                color.body_set.clear_bit(body_id_b as u32);
            }

            let sleep_contact_index = world.solver_sets[sleep_set_id as usize]
                .contact_indices
                .len() as i32;
            world.solver_sets[sleep_set_id as usize]
                .contact_indices
                .push(contact_id);

            let local_index = world.contacts[contact_id as usize].local_index;
            let is_mesh =
                (world.contacts[contact_id as usize].flags & contact_flags::SIM_MESH_CONTACT) != 0;
            if is_mesh || color_index == OVERFLOW_INDEX {
                let color = &mut world.constraint_graph.colors[color_index as usize];
                let moved_local_index = color.contacts.len() as i32 - 1;
                color.contacts.swap_remove(local_index as usize);
                if moved_local_index != local_index {
                    // fix moved element
                    let moved_contact_id = color.contacts[local_index as usize].contact_id;
                    let moved_contact = &mut world.contacts[moved_contact_id as usize];
                    debug_assert!(moved_contact.local_index == moved_local_index);
                    moved_contact.local_index = local_index;
                }
            } else {
                let color = &mut world.constraint_graph.colors[color_index as usize];
                let moved_local_index = color.convex_contacts.len() as i32 - 1;
                color.convex_contacts.swap_remove(local_index as usize);
                if moved_local_index != local_index {
                    // fix moved element
                    let moved_contact_id = color.convex_contacts[local_index as usize];
                    let moved_contact = &mut world.contacts[moved_contact_id as usize];
                    debug_assert!(moved_contact.local_index == moved_local_index);
                    moved_contact.local_index = local_index;
                }
            }

            let contact = &mut world.contacts[contact_id as usize];
            contact.set_index = sleep_set_id;
            contact.color_index = NULL_INDEX;
            contact.local_index = sleep_contact_index;
        }
    }

    // move joints
    // this shuffles joints in the awake set
    {
        let joint_count = world.islands[island_id as usize].joints.len();
        for i in 0..joint_count {
            let joint_id = world.islands[island_id as usize].joints[i].joint_id;
            debug_assert!(world.joints[joint_id as usize].set_index == AWAKE_SET);
            debug_assert!(world.joints[joint_id as usize].island_id == island_id);
            debug_assert!(world.joints[joint_id as usize].island_index == i as i32);
            let color_index = world.joints[joint_id as usize].color_index;
            let local_index = world.joints[joint_id as usize].local_index;

            debug_assert!(0 <= color_index && color_index < GRAPH_COLOR_COUNT);

            let awake_joint_sim = world.constraint_graph.colors[color_index as usize].joint_sims
                [local_index as usize];

            if color_index != OVERFLOW_INDEX {
                // might clear a bit for a static body, but this has no effect
                let body_id_a = world.joints[joint_id as usize].edges[0].body_id;
                let body_id_b = world.joints[joint_id as usize].edges[1].body_id;
                let color = &mut world.constraint_graph.colors[color_index as usize];
                color.body_set.clear_bit(body_id_a as u32);
                color.body_set.clear_bit(body_id_b as u32);
            }

            let sleep_joint_index =
                world.solver_sets[sleep_set_id as usize].joint_sims.len() as i32;
            world.solver_sets[sleep_set_id as usize]
                .joint_sims
                .push(awake_joint_sim);

            {
                let color = &mut world.constraint_graph.colors[color_index as usize];
                let moved_index = color.joint_sims.len() as i32 - 1;
                color.joint_sims.swap_remove(local_index as usize);
                if moved_index != local_index {
                    // fix moved element
                    let moved_id = color.joint_sims[local_index as usize].joint_id;
                    let moved_joint = &mut world.joints[moved_id as usize];
                    debug_assert!(moved_joint.local_index == moved_index);
                    moved_joint.local_index = local_index;
                }
            }

            let joint = &mut world.joints[joint_id as usize];
            joint.set_index = sleep_set_id;
            joint.color_index = NULL_INDEX;
            joint.local_index = sleep_joint_index;
        }
    }

    // move island struct
    {
        debug_assert!(world.islands[island_id as usize].set_index == AWAKE_SET);

        let island_index = world.islands[island_id as usize].local_index;
        world.solver_sets[sleep_set_id as usize]
            .island_sims
            .push(IslandSim { island_id });

        let awake_islands = &mut world.solver_sets[AWAKE_SET as usize].island_sims;
        let moved_island_index = awake_islands.len() as i32 - 1;
        awake_islands.swap_remove(island_index as usize);
        if moved_island_index != island_index {
            // fix index on moved element
            let moved_island_id = awake_islands[island_index as usize].island_id;
            let moved_island = &mut world.islands[moved_island_id as usize];
            debug_assert!(moved_island.local_index == moved_island_index);
            moved_island.local_index = island_index;
        }

        let island = &mut world.islands[island_id as usize];
        island.set_index = sleep_set_id;
        island.local_index = 0;
    }

    if world.split_island_id == island_id {
        world.split_island_id = NULL_INDEX;
    }

    world.validate_solver_sets();
}

/// This is called when joints are created between sets. I want to allow the sets
/// to continue sleeping if both are asleep. Otherwise one set is waked.
/// Islands will get merged when the set is woke. (b3MergeSolverSets)
pub fn merge_solver_sets(world: &mut World, set_id1: i32, set_id2: i32) {
    debug_assert!(set_id1 >= FIRST_SLEEPING_SET);
    debug_assert!(set_id2 >= FIRST_SLEEPING_SET);

    // Move the fewest number of bodies
    let count1 = world.solver_sets[set_id1 as usize].body_sims.len();
    let count2 = world.solver_sets[set_id2 as usize].body_sims.len();
    let (set_id1, set_id2) = if count1 < count2 {
        (set_id2, set_id1)
    } else {
        (set_id1, set_id2)
    };

    // transfer bodies
    {
        let body_count = world.solver_sets[set_id2 as usize].body_sims.len();
        for i in 0..body_count {
            let sim_src = world.solver_sets[set_id2 as usize].body_sims[i];

            let target_count = world.solver_sets[set_id1 as usize].body_sims.len() as i32;
            let body = &mut world.bodies[sim_src.body_id as usize];
            debug_assert!(body.set_index == set_id2);
            body.set_index = set_id1;
            body.local_index = target_count;

            world.solver_sets[set_id1 as usize].body_sims.push(sim_src);
        }
    }

    // transfer contacts
    {
        let contact_count = world.solver_sets[set_id2 as usize].contact_indices.len();
        for i in 0..contact_count {
            let contact_index = world.solver_sets[set_id2 as usize].contact_indices[i];

            let target_count = world.solver_sets[set_id1 as usize].contact_indices.len() as i32;
            let contact = &mut world.contacts[contact_index as usize];
            debug_assert!(contact.set_index == set_id2);
            contact.set_index = set_id1;
            contact.local_index = target_count;

            world.solver_sets[set_id1 as usize]
                .contact_indices
                .push(contact_index);
        }
    }

    // transfer joints
    {
        let joint_count = world.solver_sets[set_id2 as usize].joint_sims.len();
        for i in 0..joint_count {
            let joint_src = world.solver_sets[set_id2 as usize].joint_sims[i];

            let target_count = world.solver_sets[set_id1 as usize].joint_sims.len() as i32;
            let joint = &mut world.joints[joint_src.joint_id as usize];
            debug_assert!(joint.set_index == set_id2);
            joint.set_index = set_id1;
            joint.local_index = target_count;

            world.solver_sets[set_id1 as usize]
                .joint_sims
                .push(joint_src);
        }
    }

    // transfer islands
    {
        let island_count = world.solver_sets[set_id2 as usize].island_sims.len();
        for i in 0..island_count {
            let island_src = world.solver_sets[set_id2 as usize].island_sims[i];
            let island_id = island_src.island_id;

            let target_count = world.solver_sets[set_id1 as usize].island_sims.len() as i32;
            let island = &mut world.islands[island_id as usize];
            island.set_index = set_id1;
            island.local_index = target_count;

            world.solver_sets[set_id1 as usize]
                .island_sims
                .push(island_src);
        }
    }

    // destroy the merged set
    destroy_solver_set(world, set_id2);

    world.validate_solver_sets();
}

/// Move a body sim between solver sets. (b3TransferBody)
pub fn transfer_body(
    world: &mut World,
    target_set_index: i32,
    source_set_index: i32,
    body_id: i32,
) {
    use crate::body::{body_flags, IDENTITY_BODY_STATE};

    if target_set_index == source_set_index {
        return;
    }

    let source_index = world.bodies[body_id as usize].local_index;
    let mut source_sim =
        world.solver_sets[source_set_index as usize].body_sims[source_index as usize];

    // Clear transient body flags
    source_sim.flags &=
        !(body_flags::IS_FAST | body_flags::IS_SPEED_CAPPED | body_flags::HAD_TIME_OF_IMPACT);

    let target_index = world.solver_sets[target_set_index as usize].body_sims.len() as i32;
    world.solver_sets[target_set_index as usize]
        .body_sims
        .push(source_sim);

    // Remove body sim from solver set that owns it
    {
        let source_sims = &mut world.solver_sets[source_set_index as usize].body_sims;
        let moved_index = source_sims.len() as i32 - 1;
        source_sims.swap_remove(source_index as usize);
        if moved_index != source_index {
            // Fix moved body index
            let moved_id = source_sims[source_index as usize].body_id;
            let moved_body = &mut world.bodies[moved_id as usize];
            debug_assert!(moved_body.local_index == moved_index);
            moved_body.local_index = source_index;
        }
    }

    if source_set_index == AWAKE_SET {
        world.solver_sets[source_set_index as usize]
            .body_states
            .swap_remove(source_index as usize);
    } else if target_set_index == AWAKE_SET {
        let mut state = IDENTITY_BODY_STATE;
        state.flags = world.bodies[body_id as usize].flags;
        world.solver_sets[target_set_index as usize]
            .body_states
            .push(state);
    }

    let body = &mut world.bodies[body_id as usize];
    body.set_index = target_set_index;
    body.local_index = target_index;
}

/// Move a joint sim between solver sets or the constraint graph. (b3TransferJoint)
pub fn transfer_joint(
    world: &mut World,
    target_set_index: i32,
    source_set_index: i32,
    joint_id: i32,
) {
    use crate::constants::GRAPH_COLOR_COUNT;
    use crate::constraint_graph::{add_joint_to_graph, remove_joint_from_graph};

    if target_set_index == source_set_index {
        return;
    }

    let local_index = world.joints[joint_id as usize].local_index;
    let color_index = world.joints[joint_id as usize].color_index;

    // Retrieve source.
    let source_sim = if source_set_index == AWAKE_SET {
        debug_assert!(0 <= color_index && color_index < GRAPH_COLOR_COUNT);
        world.constraint_graph.colors[color_index as usize].joint_sims[local_index as usize]
    } else {
        debug_assert!(color_index == NULL_INDEX);
        world.solver_sets[source_set_index as usize].joint_sims[local_index as usize]
    };

    // Create target and copy. Fix joint.
    if target_set_index == AWAKE_SET {
        add_joint_to_graph(world, source_sim, joint_id);
        world.joints[joint_id as usize].set_index = AWAKE_SET;
    } else {
        let target_count = world.solver_sets[target_set_index as usize]
            .joint_sims
            .len() as i32;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.set_index = target_set_index;
            joint.local_index = target_count;
            joint.color_index = NULL_INDEX;
        }
        world.solver_sets[target_set_index as usize]
            .joint_sims
            .push(source_sim);
    }

    // Destroy source.
    if source_set_index == AWAKE_SET {
        let body_id_a = world.joints[joint_id as usize].edges[0].body_id;
        let body_id_b = world.joints[joint_id as usize].edges[1].body_id;
        remove_joint_from_graph(world, body_id_a, body_id_b, color_index, local_index);
    } else {
        let source_sims = &mut world.solver_sets[source_set_index as usize].joint_sims;
        let moved_index = source_sims.len() as i32 - 1;
        source_sims.swap_remove(local_index as usize);
        if moved_index != local_index {
            // fix swapped element
            let moved_id = source_sims[local_index as usize].joint_id;
            world.joints[moved_id as usize].local_index = local_index;
        }
    }
}
