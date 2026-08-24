//! Pure Rust port of [Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D physics
//! engine for games.
//!
//! The port targets exact behavioral match with the C source pinned in the
//! `box3d-cpp-reference/` submodule: same algorithms, same `f32` arithmetic, same edge
//! cases, including Box3D's hand-rolled cross-platform-deterministic trigonometry.
//!
//! # API style
//!
//! For 0.1 the public API is a direct C-mirror (same shapes, defs, and call patterns as
//! Box3D / box2d-rust), not a Rust-ergonomic wrapper. A thin ergonomic layer may be
//! considered after 0.1 if downstream users ask for one.
//!
//! # Quick start
//!
//! Add the crate, create a [`world::World`], attach bodies and shapes, then step:
//!
//! ```rust
//! use box3d_rust::body::create_body;
//! use box3d_rust::geometry::Sphere;
//! use box3d_rust::hull::make_box_hull;
//! use box3d_rust::shape::{create_hull_shape, create_sphere_shape};
//! use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
//! use box3d_rust::world::World;
//! use box3d_rust::{Pos, VEC3_ZERO};
//!
//! let mut world = World::new(&default_world_def());
//!
//! let mut ground_def = default_body_def();
//! ground_def.type_ = BodyType::Static;
//! let ground = create_body(&mut world, &ground_def);
//!
//! let mut ball_def = default_body_def();
//! ball_def.type_ = BodyType::Dynamic;
//! ball_def.position = Pos {
//!     x: 0.0 as _,
//!     y: 0.5 as _,
//!     z: 0.0 as _,
//! };
//! let ball = create_body(&mut world, &ball_def);
//!
//! let shape_def = default_shape_def();
//! let ground_hull = make_box_hull(5.0, 0.5, 5.0);
//! create_hull_shape(&mut world, ground, &shape_def, &ground_hull.base);
//!
//! let mut ball_shape = default_shape_def();
//! ball_shape.density = 1.0;
//! create_sphere_shape(
//!     &mut world,
//!     ball,
//!     &ball_shape,
//!     &Sphere {
//!         center: VEC3_ZERO,
//!         radius: 0.5,
//!     },
//! );
//!
//! // 60 Hz, 1 sub-step
//! world.step(1.0 / 60.0, 1);
//! ```
//!
//! # Features
//!
//! Enable **`double-precision`** to mirror upstream `BOX3D_DOUBLE_PRECISION`
//! (large-world mode). World positions use a wider scalar while collision math
//! stays `f32`; both configurations are tested against the C reference.
//!
//! # Determinism
//!
//! Box3D is designed for cross-platform determinism. This port keeps the
//! hand-rolled approximations (for example `atan2` / `cos`/`sin`) bit-for-bit
//! with the C scalar path — do not replace them with `std` trig. The
//! [`determinism`] module and the falling-ragdoll scene gate bit-exact match
//! with the pinned reference build.
//!
//! # Key modules
//!
//! | Module | Role |
//! |---|---|
//! | [`world`] | Own a simulation, step it, run queries and casts |
//! | [`body`] | Create and configure rigid bodies |
//! | [`shape`] | Attach collision geometry to bodies |
//! | [`joint`] | Constraints between bodies |
//! | [`types`] | Public defs/defaults (`WorldDef`, `BodyDef`, `ShapeDef`, …) |
//! | [`math_functions`] | `Vec3`, `Quat`, `Transform`, and deterministic math |
//!
//! See the repository README for port status and the live wasm demo.

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
pub mod debug_draw;
pub mod determinism;
pub mod distance;
pub mod dynamic_tree;
pub mod events;
pub mod geometry;
pub mod height_field;
pub mod hull;
pub mod human;
pub mod id;
pub mod id_pool;
pub mod island;
pub mod joint;
pub mod manifold;
pub mod math_functions;
pub mod mesh;
pub mod mover;
pub mod name_cache;
pub mod rapidhash;
pub mod recording;
pub mod sensor;
pub mod shape;
pub mod solver;
pub mod solver_set;
pub mod table;
pub mod types;
pub mod world;

pub use debug_draw::{
    make_debug_color, CreateDebugShapeCallback, DebugDraw, DebugMaterial, DebugShape,
    DestroyDebugShapeCallback, HexColor,
};
pub use id::{BodyId, ContactId, JointId, ShapeId, WorldId};
pub use math_functions::{
    Aabb, CosSin, Matrix3, Plane, Pos, Quat, SegmentDistanceResult, Transform, Triangle, Vec2,
    Vec3, WorldTransform, MAT3_IDENTITY, MAT3_ZERO, PI, POS_ZERO, QUAT_IDENTITY,
    TRANSFORM_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ONE, VEC3_ZERO,
    WORLD_TRANSFORM_IDENTITY,
};
pub use types::{
    default_body_def, default_distance_joint_def, default_explosion_def, default_filter,
    default_filter_joint_def, default_motor_joint_def, default_parallel_joint_def,
    default_prismatic_joint_def, default_query_filter, default_revolute_joint_def,
    default_shape_def, default_spherical_joint_def, default_weld_joint_def,
    default_wheel_joint_def, default_world_def, BodyDef, BodyType, Capacity, Counters,
    DistanceJointDef, ExplosionDef, Filter, FilterJointDef, JointDef, MotionLocks, MotorJointDef,
    ParallelJointDef, PrismaticJointDef, QueryFilter, RayResult, RevoluteJointDef, ShapeDef,
    SphericalJointDef, WeldJointDef, WheelJointDef, WorldCastOutput, WorldDef, BODY_TYPE_COUNT,
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
mod body_api_tests;

#[cfg(test)]
mod body_query_tests;

#[cfg(test)]
mod broad_phase_tests;

#[cfg(test)]
mod compound_tests;

#[cfg(test)]
mod contact_tests;

#[cfg(test)]
mod debug_draw_tests;

#[cfg(test)]
mod distance_tests;

#[cfg(test)]
mod dynamic_tree_tests;

#[cfg(test)]
mod geometry_tests;

#[cfg(test)]
mod hash_tests;

#[cfg(test)]
mod height_field_tests;

#[cfg(test)]
mod hull_box_tests;

#[cfg(test)]
mod hull_tests;

#[cfg(test)]
mod id_tests;

#[cfg(test)]
mod joint_tests;

#[cfg(test)]
mod manifold_tests;

#[cfg(test)]
mod math_functions_tests;

#[cfg(test)]
mod mesh_tests;

#[cfg(test)]
mod mover_tests;

#[cfg(test)]
mod name_cache_tests;

#[cfg(test)]
mod serialize_corruption_tests;

#[cfg(test)]
mod shape_tests;

#[cfg(test)]
mod shape_api_tests;

#[cfg(test)]
mod table_tests;

#[cfg(test)]
mod world_tests;

#[cfg(test)]
mod world_api_tests;

#[cfg(test)]
mod large_world_tests;

#[cfg(test)]
mod determinism_tests;

#[cfg(test)]
mod recording_tests;

#[cfg(test)]
mod recording_replay_tests;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_matches_cargo_manifest() {
        assert_eq!(VERSION, env!("CARGO_PKG_VERSION"));
    }
}
