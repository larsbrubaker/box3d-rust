// Port of solver_set.h data model. Transfer/wake/sleep logic lands later.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{BodySim, BodyState};
use crate::core::NULL_INDEX;
use crate::island::IslandSim;
use crate::joint::JointSim;

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
pub fn destroy_solver_set(world: &mut crate::world::World, set_index: i32) {
    let set = &mut world.solver_sets[set_index as usize];
    *set = SolverSet::default();
    set.set_index = NULL_INDEX;
    world.solver_set_id_pool.free_id(set_index);
}

/// Wake a solver set. Does not merge islands. (b3WakeSolverSet)
///
/// Contact/joint graph transfer paths are unreachable until those create
/// slices land; this asserts those arrays are empty and moves bodies/islands.
pub fn wake_solver_set(world: &mut crate::world::World, set_index: i32) {
    use crate::body::IDENTITY_BODY_STATE;

    debug_assert!(set_index >= FIRST_SLEEPING_SET);

    // Graph transfer for touching contacts / joints lands with those modules.
    debug_assert!(world.solver_sets[set_index as usize]
        .contact_indices
        .is_empty());
    debug_assert!(world.solver_sets[set_index as usize].joint_sims.is_empty());

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

        // Disabled-contact migration: empty until contacts exist.
        debug_assert!(world.bodies[body_id as usize].head_contact_key == NULL_INDEX);
    }

    // Transfer islands from sleeping set to awake set.
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

    destroy_solver_set(world, set_index);
}
