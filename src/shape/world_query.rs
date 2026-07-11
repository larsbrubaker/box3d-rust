//! World-space shape queries from `shape.c` (thin wrappers over geometry kernels).
//!
//! `b3Shape_RayCast` is needed by `test_large_world.c`; fuller shape API remainders
//! stay on task-4.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::lifecycle::get_shape;
use super::query::ray_cast_shape;
use crate::body::get_body_transform_quick;
use crate::geometry::RayCastInput;
use crate::id::ShapeId;
use crate::math_functions::{
    is_valid_position, is_valid_vec3, offset_pos, to_relative_transform, Pos, Vec3, VEC3_ZERO,
};
use crate::world::World;

/// World-space ray/shape-cast output. Point is a world [`Pos`] so the result
/// stays precise far from the origin. (b3WorldCastOutput)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldCastOutput {
    pub normal: Vec3,
    pub point: Pos,
    pub fraction: f32,
    pub iterations: i32,
    pub triangle_index: i32,
    pub child_index: i32,
    pub material_index: i32,
    pub hit: bool,
}

impl Default for WorldCastOutput {
    fn default() -> Self {
        WorldCastOutput {
            normal: VEC3_ZERO,
            point: crate::math_functions::POS_ZERO,
            fraction: 0.0,
            iterations: 0,
            triangle_index: crate::core::NULL_INDEX,
            child_index: 0,
            material_index: 0,
            hit: false,
        }
    }
}

/// Ray cast against a single shape in world space. Re-centers on `origin` so the
/// cast stays in float precision far from the world origin. (b3Shape_RayCast)
pub fn shape_ray_cast(
    world: &World,
    shape_id: ShapeId,
    origin: Pos,
    translation: Vec3,
) -> WorldCastOutput {
    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec3(translation));

    let shape_index = get_shape(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    let body = &world.bodies[shape.body_id as usize];
    let transform = to_relative_transform(get_body_transform_quick(world, body), origin);

    let input = RayCastInput {
        origin: VEC3_ZERO,
        translation,
        max_fraction: 1.0,
    };

    let local = ray_cast_shape(shape, transform, &input);
    WorldCastOutput {
        normal: local.normal,
        point: offset_pos(origin, local.point),
        fraction: local.fraction,
        iterations: local.iterations,
        triangle_index: local.triangle_index,
        child_index: local.child_index,
        material_index: local.material_index,
        hit: local.hit,
    }
}
