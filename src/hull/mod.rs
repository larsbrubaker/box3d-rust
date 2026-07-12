//! Convex hull construction and query module.
//!
//! Port of `box3d-cpp-reference/src/hull.c`: quickhull construction, box hulls,
//! validate, identity, plus mass/AABB/overlap/cast query APIs. The verstable hull
//! map is deferred.

mod box_hull;
mod builder_init;
mod builder_ops;
mod builder_pool;
mod create;
mod database;
mod identity;
mod queries;
mod types;
mod validate;

pub use box_hull::*;
pub use create::*;
pub use database::*;
pub use identity::*;
pub use queries::*;
pub use types::{convert_bytes_to_hull, *};
pub use validate::*;
