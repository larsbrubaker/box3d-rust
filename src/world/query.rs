//! World overlap and cast queries from physics_world.c (with recording capture).
//!
//! C passes function pointers plus a void* context; the Rust port passes
//! closures, matching the dynamic-tree callback style used across the crate.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::World;
use crate::body::get_body_transform_quick;
use crate::distance::{make_proxy, ShapeProxy};
use crate::dynamic_tree::{BoxCastInput, TreeStats};
use crate::geometry::{Capsule, PlaneResult, RayCastInput, ShapeCastInput};
use crate::id::ShapeId;
use crate::math_functions::{
    add, clamp_int, is_valid_aabb, is_valid_position, is_valid_vec3, make_aabb, max, min,
    offset_aabb, offset_pos, sub, to_relative_transform, to_vec3, Aabb, Pos, Vec3, VEC3_ZERO,
};
use crate::shape::{
    collide_mover, overlap_shape, ray_cast_shape, shape_cast_shape, should_query_collide, Shape,
};
use crate::types::{QueryFilter, RayResult, BODY_TYPE_COUNT};

/// Make a user-facing shape id for a shape. Queries hand these to callbacks.
fn query_shape_id(world: &World, shape: &Shape) -> ShapeId {
    ShapeId {
        index1: shape.id + 1,
        world0: world.world_id,
        generation: shape.generation,
    }
}

/// Overlap test for all shapes that potentially overlap the provided AABB.
/// The callback receives each overlapping shape id and returns false to
/// terminate the query. (b3World_OverlapAABB + static TreeQueryCallback)
pub fn world_overlap_aabb(
    world: &World,
    aabb: Aabb,
    filter: &QueryFilter,
    mut fcn: impl FnMut(ShapeId) -> bool,
) -> TreeStats {
    let mut tree_stats = TreeStats::default();

    debug_assert!(!world.locked);
    if world.locked {
        return tree_stats;
    }

    debug_assert!(is_valid_aabb(aabb));

    let mut rec = crate::recording::query_capture::maybe_begin(world, filter);
    if let Some(w) = rec.as_mut() {
        w.write_overlap_aabb_header(
            crate::recording::query_capture::world_id(world),
            aabb,
            filter,
        );
    }

    for i in 0..BODY_TYPE_COUNT {
        let tree_result =
            world.broad_phase.trees[i].query(aabb, filter.mask_bits, false, |_, user_data| {
                let shape_id = user_data as i32;
                let shape = &world.shapes[shape_id as usize];

                if !should_query_collide(&shape.filter, filter) {
                    return true;
                }

                let id = ShapeId {
                    index1: shape_id + 1,
                    world0: world.world_id,
                    generation: shape.generation,
                };
                let ret = fcn(id);
                if let Some(w) = rec.as_mut() {
                    w.append_overlap_hit(id, ret);
                }
                ret
            });

        tree_stats.node_visits += tree_result.node_visits;
        tree_stats.leaf_visits += tree_result.leaf_visits;
    }

    if let Some(mut w) = rec {
        w.finish_counted();
        w.append_tree_stats(tree_stats);
        crate::recording::query_capture::commit(
            world,
            w,
            crate::recording::ops::RecOp::QueryOverlapAABB,
        );
    }

    tree_stats
}

/// Overlap test for all shapes that overlap the provided shape proxy.
/// (b3World_OverlapShape + static TreeOverlapCallback)
pub fn world_overlap_shape(
    world: &World,
    origin: Pos,
    proxy: &ShapeProxy,
    filter: &QueryFilter,
    mut fcn: impl FnMut(ShapeId) -> bool,
) -> TreeStats {
    let mut tree_stats = TreeStats::default();

    debug_assert!(!world.locked);
    if world.locked {
        return tree_stats;
    }

    debug_assert!(is_valid_position(origin));

    let mut rec = crate::recording::query_capture::maybe_begin(world, filter);
    if let Some(w) = rec.as_mut() {
        w.write_overlap_shape_header(
            crate::recording::query_capture::world_id(world),
            origin,
            proxy,
            filter,
        );
    }

    // Bound the proxy in origin relative space then lift to a conservative world float box
    let aabb = offset_aabb(
        make_aabb(&proxy.points[..proxy.count as usize], proxy.radius),
        origin,
    );

    for i in 0..BODY_TYPE_COUNT {
        let tree_result =
            world.broad_phase.trees[i].query(aabb, filter.mask_bits, false, |_, user_data| {
                let shape_id = user_data as i32;
                let shape = &world.shapes[shape_id as usize];

                if !should_query_collide(&shape.filter, filter) {
                    return true;
                }

                // Re-center on the query origin so the overlap test stays in float precision
                let body = &world.bodies[shape.body_id as usize];
                let transform =
                    to_relative_transform(get_body_transform_quick(world, body), origin);

                if !overlap_shape(shape, transform, proxy) {
                    return true;
                }

                let id = query_shape_id(world, shape);
                let ret = fcn(id);
                if let Some(w) = rec.as_mut() {
                    w.append_overlap_hit(id, ret);
                }
                ret
            });

        tree_stats.node_visits += tree_result.node_visits;
        tree_stats.leaf_visits += tree_result.leaf_visits;
    }

    if let Some(mut w) = rec {
        w.finish_counted();
        w.append_tree_stats(tree_stats);
        crate::recording::query_capture::commit(
            world,
            w,
            crate::recording::ops::RecOp::QueryOverlapShape,
        );
    }

    tree_stats
}

/// Cast a ray into the world. The callback receives
/// `(shape_id, point, normal, fraction, user_material_id, triangle_index, child_index)`
/// and controls continuation like C's b3CastResultFcn: return -1 to ignore,
/// 0 to terminate, a fraction in 0..=1 to clip, or >1 to continue without clipping.
/// (b3World_CastRay + static RayCastCallback)
pub fn world_cast_ray(
    world: &World,
    origin: Pos,
    translation: Vec3,
    filter: &QueryFilter,
    mut fcn: impl FnMut(ShapeId, Pos, Vec3, f32, u64, i32, i32) -> f32,
) -> TreeStats {
    let mut rec = crate::recording::query_capture::maybe_begin(world, filter);
    if let Some(w) = rec.as_mut() {
        w.write_cast_ray_header(
            crate::recording::query_capture::world_id(world),
            origin,
            translation,
            filter,
        );
    }

    let tree_stats = cast_ray_impl(
        world,
        origin,
        translation,
        filter,
        |id, point, normal, fraction, mid, tri, child| {
            let user_fraction = fcn(id, point, normal, fraction, mid, tri, child);
            if let Some(w) = rec.as_mut() {
                w.append_cast_hit(id, point, normal, fraction, mid, tri, child, user_fraction);
            }
            user_fraction
        },
    );

    if let Some(mut w) = rec {
        w.finish_counted();
        w.append_tree_stats(tree_stats);
        crate::recording::query_capture::commit(
            world,
            w,
            crate::recording::ops::RecOp::QueryCastRay,
        );
    }

    tree_stats
}

fn cast_ray_impl(
    world: &World,
    origin: Pos,
    translation: Vec3,
    filter: &QueryFilter,
    mut fcn: impl FnMut(ShapeId, Pos, Vec3, f32, u64, i32, i32) -> f32,
) -> TreeStats {
    let mut tree_stats = TreeStats::default();

    debug_assert!(!world.locked);
    if world.locked {
        return tree_stats;
    }

    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec3(translation));

    let mut input = RayCastInput {
        origin: to_vec3(origin),
        translation,
        max_fraction: 1.0,
    };

    let mut fraction = 1.0f32;

    for i in 0..BODY_TYPE_COUNT {
        let tree_result = world.broad_phase.trees[i].ray_cast(
            &input,
            filter.mask_bits,
            false,
            |ray_input, _, user_data| {
                let shape_id = user_data as i32;
                let shape = &world.shapes[shape_id as usize];

                if !should_query_collide(&shape.filter, filter) {
                    return ray_input.max_fraction;
                }

                let body = &world.bodies[shape.body_id as usize];
                let body_transform = get_body_transform_quick(world, body);
                let transform = to_relative_transform(body_transform, origin);

                let local_input = RayCastInput {
                    origin: VEC3_ZERO,
                    translation: ray_input.translation,
                    max_fraction: ray_input.max_fraction,
                };
                let output = ray_cast_shape(shape, transform, &local_input);

                if output.hit {
                    debug_assert!(output.fraction <= ray_input.max_fraction);

                    let id = ShapeId {
                        index1: shape_id + 1,
                        world0: world.world_id,
                        generation: shape.generation,
                    };
                    let point = offset_pos(origin, output.point);
                    let material_index =
                        clamp_int(output.material_index, 0, shape.material_count() - 1);
                    let user_material_id =
                        shape.shape_materials()[material_index as usize].user_material_id;

                    let user_fraction = fcn(
                        id,
                        point,
                        output.normal,
                        output.fraction,
                        user_material_id,
                        output.triangle_index,
                        output.child_index,
                    );

                    // The user may return -1 to skip this shape
                    if (0.0..=1.0).contains(&user_fraction) {
                        fraction = user_fraction;
                    }

                    return user_fraction;
                }

                ray_input.max_fraction
            },
        );

        tree_stats.node_visits += tree_result.node_visits;
        tree_stats.leaf_visits += tree_result.leaf_visits;

        if fraction == 0.0 {
            break;
        }

        input.max_fraction = fraction;
    }

    tree_stats
}

/// Cast a ray into the world to collect the closest hit. Ignores initial
/// overlap (fraction == 0 returns -1). (b3World_CastRayClosest)
pub fn world_cast_ray_closest(
    world: &World,
    origin: Pos,
    translation: Vec3,
    filter: &QueryFilter,
) -> RayResult {
    let mut result = RayResult::default();

    debug_assert!(!world.locked);
    if world.locked {
        return result;
    }

    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec3(translation));

    // Use the unrecorded path so CastRayClosest records a single RayResult, matching C.
    let stats = cast_ray_impl(
        world,
        origin,
        translation,
        filter,
        |id, point, normal, fraction, user_material_id, triangle_index, child_index| {
            // Ignore initial overlap
            if fraction == 0.0 {
                return -1.0;
            }

            result.shape_id = id;
            result.point = point;
            result.normal = normal;
            result.fraction = fraction;
            result.user_material_id = user_material_id;
            result.triangle_index = triangle_index;
            result.child_index = child_index;
            result.hit = true;
            fraction
        },
    );

    result.node_visits = stats.node_visits;
    result.leaf_visits = stats.leaf_visits;

    if let Some(mut w) = crate::recording::query_capture::maybe_begin(world, filter) {
        w.write_cast_ray_closest_header(
            crate::recording::query_capture::world_id(world),
            origin,
            translation,
            filter,
        );
        w.append_ray_result(&result);
        crate::recording::query_capture::commit(
            world,
            w,
            crate::recording::ops::RecOp::QueryCastRayClosest,
        );
    }

    result
}

/// Cast a shape through the world. Callback contract matches [`world_cast_ray`].
/// (b3World_CastShape + static ShapeCastCallback)
pub fn world_cast_shape(
    world: &World,
    origin: Pos,
    proxy: &ShapeProxy,
    translation: Vec3,
    filter: &QueryFilter,
    mut fcn: impl FnMut(ShapeId, Pos, Vec3, f32, u64, i32, i32) -> f32,
) -> TreeStats {
    let mut tree_stats = TreeStats::default();

    debug_assert!(!world.locked);
    if world.locked {
        return tree_stats;
    }

    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec3(translation));

    let mut rec = crate::recording::query_capture::maybe_begin(world, filter);
    if let Some(w) = rec.as_mut() {
        w.write_cast_shape_header(
            crate::recording::query_capture::world_id(world),
            origin,
            proxy,
            translation,
            filter,
        );
    }

    let cast_input = ShapeCastInput {
        proxy: *proxy,
        translation,
        max_fraction: 1.0,
        can_encroach: false,
    };

    let mut fraction = 1.0f32;

    // Bound the proxy in origin relative space then lift to a conservative world float box
    let local_box = make_aabb(&proxy.points[..proxy.count as usize], proxy.radius);
    let mut tree_input = BoxCastInput {
        box_: offset_aabb(local_box, origin),
        translation,
        max_fraction: 1.0,
    };

    for i in 0..BODY_TYPE_COUNT {
        let tree_result = world.broad_phase.trees[i].box_cast(
            &tree_input,
            filter.mask_bits,
            false,
            |box_input, _, user_data| {
                let shape_id = user_data as i32;
                let shape = &world.shapes[shape_id as usize];

                if !should_query_collide(&shape.filter, filter) {
                    return box_input.max_fraction;
                }

                // Rebuild from the origin relative input, taking only the advancing
                // fraction from the tree.
                let mut local_input = cast_input;
                local_input.max_fraction = box_input.max_fraction;

                let body = &world.bodies[shape.body_id as usize];
                let transform =
                    to_relative_transform(get_body_transform_quick(world, body), origin);

                let output = shape_cast_shape(shape, transform, &local_input);

                if output.hit {
                    let id = ShapeId {
                        index1: shape_id + 1,
                        world0: world.world_id,
                        generation: shape.generation,
                    };
                    let material_index =
                        clamp_int(output.material_index, 0, shape.material_count() - 1);
                    let user_material_id =
                        shape.shape_materials()[material_index as usize].user_material_id;

                    let point = offset_pos(origin, output.point);
                    let user_fraction = fcn(
                        id,
                        point,
                        output.normal,
                        output.fraction,
                        user_material_id,
                        output.triangle_index,
                        output.child_index,
                    );

                    if let Some(w) = rec.as_mut() {
                        w.append_cast_hit(
                            id,
                            point,
                            output.normal,
                            output.fraction,
                            user_material_id,
                            output.triangle_index,
                            output.child_index,
                            user_fraction,
                        );
                    }

                    if (0.0..=1.0).contains(&user_fraction) {
                        fraction = user_fraction;
                    }

                    return user_fraction;
                }

                box_input.max_fraction
            },
        );

        tree_stats.node_visits += tree_result.node_visits;
        tree_stats.leaf_visits += tree_result.leaf_visits;

        if fraction == 0.0 {
            break;
        }

        tree_input.max_fraction = fraction;
    }

    if let Some(mut w) = rec {
        w.finish_counted();
        w.append_tree_stats(tree_stats);
        crate::recording::query_capture::commit(
            world,
            w,
            crate::recording::ops::RecOp::QueryCastShape,
        );
    }

    tree_stats
}

/// Collide a capsule mover with the world, gathering collision planes via callback.
/// The callback receives `(shape_id, planes)` and returns `false` to stop.
/// (b3World_CollideMover + static TreeCollideCallback)
pub fn world_collide_mover(
    world: &World,
    origin: Pos,
    mover: &Capsule,
    filter: &QueryFilter,
    mut fcn: impl FnMut(ShapeId, &[PlaneResult]) -> bool,
) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    debug_assert!(is_valid_position(origin));

    let mut rec = crate::recording::query_capture::maybe_begin(world, filter);
    if let Some(w) = rec.as_mut() {
        w.write_collide_mover_header(
            crate::recording::query_capture::world_id(world),
            origin,
            *mover,
            filter,
        );
    }

    let r = Vec3 {
        x: mover.radius,
        y: mover.radius,
        z: mover.radius,
    };

    // Relative box lifted to world float with outward rounding, conservative for the tree
    let rel_box = Aabb {
        lower_bound: sub(min(mover.center1, mover.center2), r),
        upper_bound: add(max(mover.center1, mover.center2), r),
    };
    let aabb = offset_aabb(rel_box, origin);

    for i in 0..BODY_TYPE_COUNT {
        world.broad_phase.trees[i].query(aabb, filter.mask_bits, false, |_, user_data| {
            let shape_id = user_data as i32;
            let shape = &world.shapes[shape_id as usize];

            if !should_query_collide(&shape.filter, filter) {
                return true;
            }

            // Re-center on the query origin, the mover and the resulting planes are origin relative
            let body = &world.bodies[shape.body_id as usize];
            let transform = to_relative_transform(get_body_transform_quick(world, body), origin);

            let mut buffer = [PlaneResult::default(); 64];
            let count = collide_mover(&mut buffer, shape, transform, mover);

            if count > 0 {
                let id = query_shape_id(world, shape);
                let planes = &buffer[..count as usize];
                let ret = fcn(id, planes);
                if let Some(w) = rec.as_mut() {
                    w.append_plane_hit(id, planes, ret);
                }
                return ret;
            }

            true
        });
    }

    if let Some(mut w) = rec {
        w.finish_counted();
        crate::recording::query_capture::commit(
            world,
            w,
            crate::recording::ops::RecOp::QueryCollideMover,
        );
    }
}

/// Cast a capsule mover through the world. Returns the earliest hit fraction in
/// `[0, 1]`. Overlapping shapes (fraction == 0) are ignored. An optional filter
/// callback can reject individual shapes. (b3World_CastMover + static MoverCastCallback)
pub fn world_cast_mover(
    world: &World,
    origin: Pos,
    mover: &Capsule,
    translation: Vec3,
    filter: &QueryFilter,
    mut filter_fcn: Option<&mut dyn FnMut(ShapeId) -> bool>,
) -> f32 {
    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec3(translation));

    debug_assert!(!world.locked);
    if world.locked {
        return 1.0;
    }

    let mut rec = crate::recording::query_capture::maybe_begin(world, filter);
    if let Some(w) = rec.as_mut() {
        w.write_cast_mover_header(
            crate::recording::query_capture::world_id(world),
            origin,
            *mover,
            translation,
            filter,
        );
    }

    let cast_input = ShapeCastInput {
        proxy: make_proxy(&[mover.center1, mover.center2], mover.radius),
        translation,
        max_fraction: 1.0,
        can_encroach: mover.radius > 0.0,
    };

    let mut fraction = 1.0f32;

    // Bound the capsule in origin relative space then lift to a conservative world float box
    let centers = [mover.center1, mover.center2];
    let mut tree_input = BoxCastInput {
        box_: offset_aabb(make_aabb(&centers, mover.radius), origin),
        translation,
        max_fraction: 1.0,
    };

    for i in 0..BODY_TYPE_COUNT {
        world.broad_phase.trees[i].box_cast(
            &tree_input,
            filter.mask_bits,
            false,
            |box_input, _, user_data| {
                let shape_id = user_data as i32;
                let shape = &world.shapes[shape_id as usize];

                if !should_query_collide(&shape.filter, filter) {
                    return fraction;
                }

                let id = ShapeId {
                    index1: shape_id + 1,
                    world0: world.world_id,
                    generation: shape.generation,
                };

                // When recording, always record the filter decision (accept-all if no user filter),
                // matching C's overlap trampoline installed even for NULL filters.
                let should_collide = if let Some(ref mut fcn) = filter_fcn {
                    fcn(id)
                } else {
                    true
                };
                if rec.is_some() || filter_fcn.is_some() {
                    if let Some(w) = rec.as_mut() {
                        w.append_overlap_hit(id, should_collide);
                    }
                    if !should_collide {
                        return fraction;
                    }
                }

                // Rebuild from the origin relative input, taking only the advancing fraction from the tree
                let mut local_input = cast_input;
                local_input.max_fraction = box_input.max_fraction;

                // Re-center on the query origin so the per-shape cast stays in float precision far from the origin
                let body = &world.bodies[shape.body_id as usize];
                let transform =
                    to_relative_transform(get_body_transform_quick(world, body), origin);

                let output = shape_cast_shape(shape, transform, &local_input);
                if output.fraction == 0.0 {
                    // Ignore overlapping shapes
                    return fraction;
                }

                fraction = output.fraction;
                output.fraction
            },
        );

        if fraction == 0.0 {
            break;
        }

        tree_input.max_fraction = fraction;
    }

    if let Some(mut w) = rec {
        w.finish_counted();
        w.append_f32(fraction);
        crate::recording::query_capture::commit(
            world,
            w,
            crate::recording::ops::RecOp::QueryCastMover,
        );
    }

    fraction
}
