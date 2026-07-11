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
