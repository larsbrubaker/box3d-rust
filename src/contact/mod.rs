// Port of the contact module from box3d-cpp-reference/src/contact.h + contact.c.
//
// This file holds the data model. Lifecycle (registers, create/destroy) is in
// lifecycle.rs. Narrow-phase update lands in a later collide slice.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::distance::SimplexCache;
use crate::manifold::{Manifold, SatCache};
use crate::math_functions::{Quat, Transform, Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO};

/// Contact flag bits. (enum b3ContactFlags)
pub mod contact_flags {
    /// Set when the solid shapes are touching.
    pub const TOUCHING: u32 = 0x0000_0001;
    /// Contact has a hit event
    pub const HIT_EVENT: u32 = 0x0000_0002;
    /// This contact wants contact events
    pub const ENABLE_CONTACT_EVENTS: u32 = 0x0000_0004;
    /// Contact is between a dynamic and static body
    pub const STATIC_FLAG: u32 = 0x0000_0008;
    pub const RECYCLE: u32 = 0x0000_0010;

    /// Set when the shapes are touching (sim flag)
    pub const SIM_TOUCHING: u32 = 0x0001_0000;
    /// This contact no longer has overlapping AABBs
    pub const SIM_DISJOINT: u32 = 0x0002_0000;
    /// This contact started touching
    pub const SIM_STARTED_TOUCHING: u32 = 0x0004_0000;
    /// This contact stopped touching
    pub const SIM_STOPPED_TOUCHING: u32 = 0x0008_0000;
    /// This contact has a hit event
    pub const SIM_ENABLE_HIT_EVENT: u32 = 0x0010_0000;
    /// This contact wants pre-solve events
    pub const SIM_ENABLE_PRE_SOLVE_EVENTS: u32 = 0x0020_0000;
    /// This is a mesh contact
    pub const SIM_MESH_CONTACT: u32 = 0x0040_0000;
    /// Relative transform is valid for recycling
    pub const RELATIVE_TRANSFORM_VALID: u32 = 0x0080_0000;
}

/// Contact cache: SAT or GJK simplex. (b3ContactCache)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ContactCache {
    Sat(SatCache),
    Simplex(SimplexCache),
}

impl Default for ContactCache {
    fn default() -> Self {
        ContactCache::Sat(SatCache::default())
    }
}

/// Per-triangle cache for mesh contacts. (b3TriangleCache)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TriangleCache {
    pub triangle_index: i32,
    pub cache: ContactCache,
}

impl Default for TriangleCache {
    fn default() -> Self {
        TriangleCache {
            triangle_index: NULL_INDEX,
            cache: ContactCache::default(),
        }
    }
}

/// Mesh contact state. (b3MeshContact)
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MeshContact {
    pub triangle_cache: Vec<TriangleCache>,
    pub query_bounds: crate::math_functions::Aabb,
}

/// Convex contact state. (b3ConvexContact)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ConvexContact {
    pub cache: ContactCache,
}

/// Contact geometry cache payload. (C union in b3Contact)
#[derive(Debug, Clone, PartialEq)]
pub enum ContactGeometry {
    Convex(ConvexContact),
    Mesh(MeshContact),
}

impl Default for ContactGeometry {
    fn default() -> Self {
        ContactGeometry::Convex(ConvexContact::default())
    }
}

/// A contact edge connects bodies and contacts in a contact graph where each
/// body is a node and each contact is an edge. (b3ContactEdge)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactEdge {
    pub body_id: i32,
    pub prev_key: i32,
    pub next_key: i32,
}

impl Default for ContactEdge {
    fn default() -> Self {
        ContactEdge {
            body_id: NULL_INDEX,
            prev_key: NULL_INDEX,
            next_key: NULL_INDEX,
        }
    }
}

/// Persistent interaction between two shapes. (b3Contact)
#[derive(Debug, Clone)]
pub struct Contact {
    /// Index of simulation set stored in World. NULL_INDEX when slot is free.
    pub set_index: i32,

    /// Index into the constraint graph color array. NULL_INDEX for non-touching
    /// or sleeping contacts, and when the slot is free.
    pub color_index: i32,

    /// Contact index within set or graph color. NULL_INDEX when slot is free.
    pub local_index: i32,

    pub edges: [ContactEdge; 2],
    pub shape_id_a: i32,
    pub shape_id_b: i32,
    pub child_index: i32,

    /// A contact only belongs to an island if touching, otherwise NULL_INDEX.
    pub island_id: i32,

    /// Index into the island's contacts array for O(1) swap-removal.
    /// NULL_INDEX when not in an island.
    pub island_index: i32,

    /// Back index into World::contacts
    pub contact_id: i32,

    /// Transient and cached for performance. NULL_INDEX for static bodies.
    pub body_sim_index_a: i32,
    pub body_sim_index_b: i32,

    /// contact_flags bits
    pub flags: u32,

    pub manifolds: Vec<Manifold>,

    /// Cache for contact recycling.
    pub cached_rotation_a: Quat,
    pub cached_rotation_b: Quat,
    pub cached_relative_pose: Transform,

    /// Mixed friction and restitution
    pub friction: f32,
    pub restitution: f32,
    pub rolling_resistance: f32,
    pub tangent_velocity: Vec3,

    /// Usage determined by SIM_MESH_CONTACT in flags
    pub geometry: ContactGeometry,

    /// Monotonically advanced when a contact is allocated in this slot.
    /// Used to check for invalid ContactId.
    pub generation: u32,
}

impl Contact {
    /// Manifold count. (C: manifoldCount)
    pub fn manifold_count(&self) -> i32 {
        self.manifolds.len() as i32
    }
}

impl Default for Contact {
    fn default() -> Self {
        Contact {
            set_index: NULL_INDEX,
            color_index: NULL_INDEX,
            local_index: NULL_INDEX,
            edges: [ContactEdge::default(); 2],
            shape_id_a: NULL_INDEX,
            shape_id_b: NULL_INDEX,
            child_index: 0,
            island_id: NULL_INDEX,
            island_index: NULL_INDEX,
            contact_id: NULL_INDEX,
            body_sim_index_a: NULL_INDEX,
            body_sim_index_b: NULL_INDEX,
            flags: 0,
            manifolds: Vec::new(),
            cached_rotation_a: QUAT_IDENTITY,
            cached_rotation_b: QUAT_IDENTITY,
            cached_relative_pose: TRANSFORM_IDENTITY,
            friction: 0.0,
            restitution: 0.0,
            rolling_resistance: 0.0,
            tangent_velocity: VEC3_ZERO,
            geometry: ContactGeometry::default(),
            generation: 0,
        }
    }
}

/// Contact descriptor for graph-color mesh/overflow constraints. (b3ContactSpec)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactSpec {
    pub contact_id: i32,
    /// Start of the global manifold constraint array
    pub manifold_start: i32,
    pub manifold_count: u16,
}

impl Default for ContactSpec {
    fn default() -> Self {
        ContactSpec {
            contact_id: NULL_INDEX,
            manifold_start: 0,
            manifold_count: 0,
        }
    }
}

mod collide;
mod lifecycle;
mod mesh_cache;
mod mesh_contact;
mod mesh_cull;
mod update;

pub use collide::*;
pub use lifecycle::*;
pub use mesh_contact::{apply_mesh_hit_flags, compute_mesh_manifolds};
pub use update::*;
