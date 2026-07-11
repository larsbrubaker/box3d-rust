//! Convex hull construction module.
//!
//! Port of `box3d-cpp-reference/src/hull.c` construction path (quickhull, box hulls,
//! validate, identity). Query APIs (mass/AABB/overlap/cast) and the verstable hull map
//! are deferred.

mod box_hull;
mod builder_init;
mod builder_ops;
mod builder_pool;
mod create;
mod identity;
mod types;
mod validate;

pub use box_hull::*;
pub use create::*;
pub use identity::*;
pub use types::*;
pub use validate::*;
