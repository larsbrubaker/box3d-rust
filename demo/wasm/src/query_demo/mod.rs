//! Collision samples — faithful ports of `box3d-cpp-reference/samples/sample_collision.cpp`.
//!
//! Each C `RegisterSample( "Collision", … )` becomes one submodule with its own
//! `wasm_bindgen` export set and its own thread-local `STATE`. The shared demo
//! page (`demo/src/demos/queries.ts`) dispatches to the matching export family by
//! the registry scene slug.
//!
//! - [`cast_world`] — Cast World (ray/sphere/capsule/box casts, verified values).
//! - [`shared`] — the `CastContext` accumulator + `RayCastClosestCallback` logic
//!   reused by the ray/shape-cast samples.

mod cast_world;
pub mod shared;

mod capsule_cast_ray;
mod distance_debug;
mod initial_overlap;
mod long_ray_cast;
mod mesh_scale;
mod overlap_world;
mod ray_curtain;
mod shape_cast;
mod shape_cast_debug;
mod shape_distance;
mod time_of_impact;
