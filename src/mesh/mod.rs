//! Triangle mesh collision shape.
//!
//! Port of `box3d-cpp-reference/src/mesh.c` and the mesh group of
//! `include/box3d/types.h` / `collision.h`.
//!
//! Layout:
//! - `types`   — MeshDef/Data/Node/Triangle, edge flags, blob serialization
//! - `bvh`     — vertex welding, SAH/median BVH build, DFS triangle sort
//! - `create`  — create/destroy, edge identification, validation
//! - `factory` — grid/wave/torus/box/hollow/platform helpers
//! - `cast`    — AABB, ray cast, shape cast
//! - `query`   — overlap, query, mover collide, triangle accessor
//!
//! Deferred: mesh_contact.c (needs world), world shape attach.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod bvh;
mod cast;
mod create;
mod factory;
mod query;
mod types;

pub use cast::{compute_mesh_aabb, ray_cast_mesh, shape_cast_mesh};
pub use create::{create_mesh, destroy_mesh, get_height, is_valid_mesh};
pub use factory::{
    create_box_mesh, create_grid_mesh, create_hollow_box_mesh, create_platform_mesh,
    create_torus_mesh, create_wave_mesh,
};
pub use query::{collide_mover_and_mesh, get_mesh_triangle, overlap_mesh, query_mesh};
pub use types::{
    get_mesh_flags, get_mesh_material_indices, get_mesh_nodes, get_mesh_triangles, get_mesh_vertices,
    Mesh, MeshData, MeshDef, MeshNode, MeshTriangle, ALL_CONCAVE_EDGES, ALL_FLAT_EDGES,
    CONCAVE_EDGE1, CONCAVE_EDGE2, CONCAVE_EDGE3, FLAT_EDGE1, FLAT_EDGE2, FLAT_EDGE3,
    INVERSE_CONCAVE_EDGE1, INVERSE_CONCAVE_EDGE2, INVERSE_CONCAVE_EDGE3, LEAF_NODE, MESH_DATA_SIZE,
    MESH_NODE_SIZE, MESH_STACK_SIZE, MESH_TRIANGLE_SIZE, MESH_VERSION,
};
