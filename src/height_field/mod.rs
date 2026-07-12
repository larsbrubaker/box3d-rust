//! Height field collision shape.
//!
//! Port of `box3d-cpp-reference/src/height_field.c` and the height-field group of
//! `include/box3d/types.h` / `collision.h`.
//!
//! Layout:
//! - `types`    — HeightFieldDef/Data, edge flags, blob serialization
//! - `create`   — create/destroy and convexity flags
//! - `factory`  — grid/wave helpers, dump/load
//! - `triangle` — cell corners and triangle accessors
//! - `cast`     — AABB, ray cast, shape cast
//! - `query`    — overlap, query, mover collide
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod cast;
mod create;
mod factory;
mod query;
mod triangle;
mod types;

pub use cast::{compute_height_field_aabb, ray_cast_height_field, shape_cast_height_field};
pub use create::{create_height_field, destroy_height_field};
pub use factory::{create_grid, create_wave, dump_height_data, load_height_field};
pub use query::{collide_mover_and_height_field, overlap_height_field, query_height_field};
pub use triangle::{get_height_field_material, get_height_field_triangle};
pub use types::{
    convert_bytes_to_height_field, get_height_field_compressed_heights, get_height_field_flags,
    get_height_field_material_indices, get_height_field_triangle_count, HeightFieldData,
    HeightFieldDef, CONCAVE_EDGE1, CONCAVE_EDGE2, CONCAVE_EDGE3, HEIGHT_FIELD_DATA_SIZE,
    HEIGHT_FIELD_HOLE, HEIGHT_FIELD_VERSION, INVERSE_CONCAVE_EDGE1, INVERSE_CONCAVE_EDGE2,
    INVERSE_CONCAVE_EDGE3,
};
