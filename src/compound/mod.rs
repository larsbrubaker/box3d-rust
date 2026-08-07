//! Baked compound collision shape.
//!
//! Port of `box3d-cpp-reference/src/compound.c` and the compound group of
//! `include/box3d/types.h` / `collision.h`.
//!
//! Layout:
//! - `types`     — CompoundDef/Data, child accessors, SurfaceMaterial consumers
//! - `create`    — create/destroy with material/hull/mesh sharing
//! - `serialize`   — convert to contiguous bytes
//! - `deserialize` — convert from contiguous bytes
//! - `cast`      — AABB, overlap, ray cast, shape cast
//! - `query`     — AABB query, collide mover
//!
//! Deferred: world shape attach. Compound CCD TOI lives in `shape::toi`
//! (matching the live path in C `shape.c`; compound.c's copy is `#if 0`).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

mod cast;
mod create;
mod deserialize;
mod query;
mod serialize;
mod types;

pub use cast::{
    compute_compound_aabb, make_compound_child_sweep, overlap_compound, ray_cast_compound,
    shape_cast_compound,
};
pub use create::{create_compound, destroy_compound};
pub use deserialize::convert_bytes_to_compound;
pub use query::{collide_mover_and_compound, query_compound};
pub use serialize::convert_compound_to_bytes;
pub use types::{
    get_compound_capsule, get_compound_child, get_compound_hull, get_compound_materials,
    get_compound_mesh, get_compound_sphere, ChildGeometry, ChildShape, CompoundCapsule,
    CompoundCapsuleDef, CompoundData, CompoundDef, CompoundHull, CompoundHullDef, CompoundMesh,
    CompoundMeshDef, CompoundSphere, CompoundSphereDef, COMPOUND_VERSION,
    MAX_COMPOUND_MESH_MATERIALS,
};
