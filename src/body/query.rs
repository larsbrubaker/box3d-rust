// Body-level query / cast / overlap API from body.c.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::lifecycle::get_body_full_id;
use super::types::BodyCastResult;
use crate::core::NULL_INDEX;
use crate::distance::ShapeProxy;
use crate::geometry::{RayCastInput, ShapeCastInput};
use crate::id::{BodyId, ShapeId};
use crate::math_functions::{
    clamp_int, offset_pos, to_relative_transform, Pos, Vec3, WorldTransform, VEC3_ZERO,
};
use crate::shape::{overlap_shape, ray_cast_shape, shape_cast_shape, should_query_collide};
use crate::types::QueryFilter;
use crate::world::World;

/// Cast a ray against a body's shapes. Results are in world space relative to
/// `origin`; `body_transform` (not the body's stored pose) drives the geometry.
/// (b3Body_CastRay)
pub fn body_cast_ray(
    world: &World,
    body_id: BodyId,
    origin: Pos,
    translation: Vec3,
    filter: &QueryFilter,
    max_fraction: f32,
    body_transform: WorldTransform,
) -> BodyCastResult {
    debug_assert!(!world.locked);
    if world.locked {
        return BodyCastResult::default();
    }

    let mut result = BodyCastResult::default();
    let body_index = get_body_full_id(world, body_id);

    // The consistent framing is to center on the ray origin.
    let mut shape_input = RayCastInput {
        origin: VEC3_ZERO,
        translation,
        max_fraction,
    };

    let transform = to_relative_transform(body_transform, origin);

    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let shape = &world.shapes[shape_id as usize];
        // Careful with id: advance before using shape fields below.
        shape_id = shape.next_shape_id;

        if !should_query_collide(&shape.filter, filter) {
            continue;
        }

        let shape_output = ray_cast_shape(shape, transform, &shape_input);

        if !shape_output.hit {
            continue;
        }

        if shape_output.fraction > shape_input.max_fraction {
            continue;
        }

        let id = ShapeId {
            index1: shape.id + 1,
            world0: body_id.world0,
            generation: shape.generation,
        };

        let material_index = clamp_int(shape_output.material_index, 0, shape.material_count() - 1);
        let user_material_id = shape.shape_materials()[material_index as usize].user_material_id;

        result = BodyCastResult {
            shape_id: id,
            point: offset_pos(origin, shape_output.point),
            normal: shape_output.normal,
            fraction: shape_output.fraction,
            triangle_index: shape_output.triangle_index,
            user_material_id,
            iterations: shape_output.iterations,
            hit: true,
        };

        shape_input.max_fraction = shape_output.fraction;
    }

    result
}

/// Cast a shape proxy against a body's shapes. (b3Body_CastShape)
pub fn body_cast_shape(
    world: &World,
    body_id: BodyId,
    origin: Pos,
    proxy: &ShapeProxy,
    translation: Vec3,
    filter: &QueryFilter,
    max_fraction: f32,
    can_encroach: bool,
    body_transform: WorldTransform,
) -> BodyCastResult {
    debug_assert!(!world.locked);
    if world.locked {
        return BodyCastResult::default();
    }

    let mut result = BodyCastResult::default();
    let body_index = get_body_full_id(world, body_id);

    let transform = to_relative_transform(body_transform, origin);

    let mut shape_input = ShapeCastInput {
        proxy: *proxy,
        translation,
        max_fraction,
        can_encroach,
    };

    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let shape = &world.shapes[shape_id as usize];
        shape_id = shape.next_shape_id;

        if !should_query_collide(&shape.filter, filter) {
            continue;
        }

        let shape_output = shape_cast_shape(shape, transform, &shape_input);

        if !shape_output.hit {
            continue;
        }

        if shape_output.fraction > shape_input.max_fraction {
            continue;
        }

        let id = ShapeId {
            index1: shape.id + 1,
            world0: body_id.world0,
            generation: shape.generation,
        };
        let material_index = clamp_int(shape_output.material_index, 0, shape.material_count() - 1);
        let user_material_id = shape.shape_materials()[material_index as usize].user_material_id;

        result = BodyCastResult {
            shape_id: id,
            point: offset_pos(origin, shape_output.point),
            normal: shape_output.normal,
            fraction: shape_output.fraction,
            triangle_index: shape_output.triangle_index,
            user_material_id,
            iterations: shape_output.iterations,
            hit: true,
        };

        shape_input.max_fraction = shape_output.fraction;
    }

    result
}

/// Test whether a shape proxy overlaps any of a body's shapes. (b3Body_OverlapShape)
pub fn body_overlap_shape(
    world: &World,
    body_id: BodyId,
    origin: Pos,
    proxy: &ShapeProxy,
    filter: &QueryFilter,
    body_transform: WorldTransform,
) -> bool {
    debug_assert!(!world.locked);
    if world.locked {
        return false;
    }

    let body_index = get_body_full_id(world, body_id);
    let transform = to_relative_transform(body_transform, origin);

    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let shape = &world.shapes[shape_id as usize];
        shape_id = shape.next_shape_id;

        if !should_query_collide(&shape.filter, filter) {
            continue;
        }

        if overlap_shape(shape, transform, proxy) {
            return true;
        }
    }

    false
}
