//! Hull query APIs (mass, AABB, overlap, ray/shape cast).
//! Port of the query section at the end of box3d-cpp-reference/src/hull.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::overlap_slop;
use crate::core::NULL_INDEX;
use crate::distance::{
    make_proxy, shape_cast, shape_distance, CastOutput, DistanceInput, ShapeCastPairInput,
    ShapeProxy, SimplexCache,
};
use crate::geometry::types::{
    Capsule, MassData, PlaneResult, RayCastInput, ShapeCastInput, ShapeExtent,
};
use crate::hull::types::{get_hull_planes, get_hull_points, HullData};
use crate::math_functions::{
    aabb_transform, aabb_union, abs, add, dot, inv_mul_transforms, max, mul_sm, mul_sv, sub, Aabb,
    Plane, Transform, Vec3, TRANSFORM_IDENTITY, VEC3_ZERO,
};

/// Compute mass properties of a hull. (b3ComputeHullMass)
pub fn compute_hull_mass(shape: &HullData, density: f32) -> MassData {
    MassData {
        mass: density * shape.volume,
        center: shape.center,
        // Inertia about the center of mass
        inertia: mul_sm(density, shape.central_inertia),
    }
}

/// Compute the extent of a hull relative to an origin. (b3ComputeHullExtent)
pub fn compute_hull_extent(hull: &HullData, origin: Vec3) -> ShapeExtent {
    let points = get_hull_points(hull);

    let mut extent = ShapeExtent {
        min_extent: hull.inner_radius,
        max_extent: VEC3_ZERO,
    };
    for point in points {
        extent.max_extent = max(extent.max_extent, abs(sub(*point, origin)));
    }

    extent
}

/// Compute the AABB of a hull. (b3ComputeHullAABB)
pub fn compute_hull_aabb(shape: &HullData, transform: Transform) -> Aabb {
    aabb_transform(transform, shape.aabb)
}

/// Compute the swept AABB of a hull between two transforms.
/// (b3ComputeSweptHullAABB)
pub fn compute_swept_hull_aabb(shape: &HullData, xf1: Transform, xf2: Transform) -> Aabb {
    let aabb1 = aabb_transform(xf1, shape.aabb);
    let aabb2 = aabb_transform(xf2, shape.aabb);
    aabb_union(aabb1, aabb2)
}

/// Test overlap between a hull and a shape proxy. (b3OverlapHull)
pub fn overlap_hull(shape: &HullData, shape_transform: Transform, proxy: &ShapeProxy) -> bool {
    let points = get_hull_points(shape);

    let input = DistanceInput {
        proxy_a: make_proxy(points, 0.0),
        proxy_b: *proxy,
        transform: inv_mul_transforms(shape_transform, TRANSFORM_IDENTITY),
        use_radii: true,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);
    output.distance < overlap_slop()
}

/// Ray cast versus hull shape in local space. (b3RayCastHull)
pub fn ray_cast_hull(shape: &HullData, input: &RayCastInput) -> CastOutput {
    // Same checks as geometry::is_valid_ray (avoid a geometry↔hull import cycle).
    debug_assert!({
        use crate::constants::huge;
        use crate::math_functions::{is_valid_float, is_valid_vec3};
        is_valid_vec3(input.origin)
            && is_valid_vec3(input.translation)
            && is_valid_float(input.max_fraction)
            && 0.0 <= input.max_fraction
            && input.max_fraction < huge()
    });
    let mut output = CastOutput::default();

    let mut lower = 0.0;
    let mut upper = input.max_fraction;
    let mut best_face = NULL_INDEX;

    let planes = get_hull_planes(shape);

    for face_index in 0..shape.face_count {
        let plane = planes[face_index as usize];

        let distance = plane.offset - dot(plane.normal, input.origin);
        let denominator = dot(plane.normal, input.translation);

        if denominator == 0.0 {
            if distance < 0.0 {
                return output;
            }
        } else {
            let fraction = distance / denominator;

            if denominator < 0.0 {
                if fraction > lower {
                    best_face = face_index;
                    lower = fraction;
                }
            } else if fraction < upper {
                upper = fraction;
            }

            if upper < lower {
                return output;
            }
        }
    }

    if best_face >= 0 {
        output.point = add(input.origin, mul_sv(lower, input.translation));
        output.normal = planes[best_face as usize].normal;
        output.fraction = lower;
        output.hit = true;
    } else {
        output.point = input.origin;
        output.hit = true;
    }

    output
}

/// Shape cast versus a hull. (b3ShapeCastHull)
pub fn shape_cast_hull(shape: &HullData, input: &ShapeCastInput) -> CastOutput {
    let points = get_hull_points(shape);

    let pair_input = ShapeCastPairInput {
        proxy_a: make_proxy(points, 0.0),
        proxy_b: input.proxy,
        transform: TRANSFORM_IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };

    shape_cast(&pair_input)
}

/// Collide a capsule mover against a hull. (b3CollideMoverAndHull)
pub fn collide_mover_and_hull(
    result: &mut PlaneResult,
    shape: &HullData,
    mover: &Capsule,
) -> i32 {
    let points = get_hull_points(shape);
    let distance_input = DistanceInput {
        proxy_a: make_proxy(points, 0.0),
        proxy_b: make_proxy(&[mover.center1, mover.center2], mover.radius),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let total_radius = mover.radius;

    let mut cache = SimplexCache::default();
    let distance_output = shape_distance(&distance_input, &mut cache, None);

    if distance_output.distance == 0.0 {
        // Deep overlap on hulls is intentionally not handled (matches mesh behavior).
        return 0;
    }

    if distance_output.distance <= total_radius {
        result.plane = Plane {
            normal: distance_output.normal,
            offset: total_radius - distance_output.distance,
        };
        result.point = distance_output.point_a;
        return 1;
    }

    0
}
