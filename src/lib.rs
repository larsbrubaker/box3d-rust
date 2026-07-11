//! Pure Rust port of [Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D physics
//! engine for games.
//!
//! The port targets exact behavioral match with the C source pinned in the
//! `box3d-cpp-reference/` submodule: same algorithms, same `f32` arithmetic, same edge
//! cases, including Box3D's hand-rolled cross-platform-deterministic trigonometry.
//!
//! Porting has just begun — modules land whole, in dependency order, together with their
//! portion of the upstream C test suite. See the repository README for live status.

pub mod aabb;
pub mod bitset;
pub mod body;
pub mod broad_phase;
pub mod compound;
pub mod constants;
pub mod constraint_graph;
pub mod contact;
pub mod contact_solver;
pub mod core;
pub mod distance;
pub mod dynamic_tree;
pub mod events;
pub mod geometry;
pub mod height_field;
pub mod hull;
pub mod id;
pub mod id_pool;
pub mod island;
pub mod joint;
pub mod manifold;
pub mod math_functions;
pub mod mesh;
pub mod name_cache;
pub mod sensor;
pub mod shape;
pub mod solver;
pub mod solver_set;
pub mod table;
pub mod types;
pub mod world;

pub use id::{BodyId, ContactId, JointId, ShapeId, WorldId};
pub use math_functions::{
    Aabb, CosSin, Matrix3, Plane, Pos, Quat, SegmentDistanceResult, Transform, Triangle, Vec2,
    Vec3, WorldTransform, MAT3_IDENTITY, MAT3_ZERO, PI, POS_ZERO, QUAT_IDENTITY,
    TRANSFORM_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ONE, VEC3_ZERO,
    WORLD_TRANSFORM_IDENTITY,
};
pub use types::{
    default_body_def, default_filter, default_query_filter, default_shape_def, default_world_def,
    BodyDef, BodyType, Capacity, Filter, MotionLocks, QueryFilter, ShapeDef, WorldDef,
    BODY_TYPE_COUNT,
};

/// Crate version, exposed so demos and downstream tools can report the exact port build.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod aabb_tests;

#[cfg(test)]
mod bitset_tests;

#[cfg(test)]
mod body_tests;

#[cfg(test)]
mod broad_phase_tests;

#[cfg(test)]
mod compound_tests;

#[cfg(test)]
mod distance_tests;

#[cfg(test)]
mod dynamic_tree_tests;

#[cfg(test)]
mod geometry_tests;

#[cfg(test)]
mod height_field_tests;

#[cfg(test)]
mod hull_tests;

#[cfg(test)]
mod id_tests;

#[cfg(test)]
mod manifold_tests;

#[cfg(test)]
mod math_functions_tests;

#[cfg(test)]
mod mesh_tests;

#[cfg(test)]
mod shape_tests;

#[cfg(test)]
mod table_tests;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_matches_cargo_manifest() {
        assert_eq!(VERSION, env!("CARGO_PKG_VERSION"));
        assert!(!VERSION.is_empty());
    }
}
