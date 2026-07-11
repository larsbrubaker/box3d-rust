// Port of the island data model from box3d-cpp-reference/src/island.h.
// Logic from island.c lands in a later bring-up commit.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;

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
