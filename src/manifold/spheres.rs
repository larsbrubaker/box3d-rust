//! Sphere and hull-sphere contact manifolds from `convex_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{LocalManifold, FEATURE_PAIR_SINGLE};
use crate::constants::speculative_distance;
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::geometry::{Capsule, Sphere};
use crate::hull::{get_hull_planes, get_hull_points, HullData};
use crate::math_functions::{
    add, dot, length_squared, lerp, mul_add, mul_sub, mul_sv, normalize, plane_separation,
    point_to_segment_distance, sub, transform_point, Transform, Vec3, TRANSFORM_IDENTITY,
};

/// Collide two spheres. (b3CollideSpheres)
pub fn collide_spheres(
    manifold: &mut LocalManifold,
    capacity: i32,
    sphere_a: &Sphere,
    sphere_b: &Sphere,
    transform_b_to_a: Transform,
) {
    if capacity == 0 {
        return;
    }

    // Work in shapeB coordinates
    let center1 = sphere_a.center;
    let center2 = transform_point(transform_b_to_a, sphere_b.center);

    let total_radius = sphere_a.radius + sphere_b.radius;
    let offset = sub(center2, center1);
    let distance_sq = length_squared(offset);

    if distance_sq > total_radius * total_radius {
        // We found a separating axis
        return;
    }

    let mut normal = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    let distance = distance_sq.sqrt();
    if distance * distance > 1000.0 * f32::MIN_POSITIVE {
        normal = mul_sv(1.0 / distance, offset);
    }

    // Contact at the midpoint
    // 0.5 * ( ((c1 + rA*n) + c2) - rB*n )
    let point = mul_sv(
        0.5,
        mul_sub(
            add(
                mul_add(center1, sphere_a.radius, normal),
                center2,
            ),
            sphere_b.radius,
            normal,
        ),
    );

    // Manifold in frame B
    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = &mut manifold.points[0];
    pt.point = point;
    pt.separation = distance - total_radius;
    pt.pair = FEATURE_PAIR_SINGLE;
}

/// Collide a capsule and a sphere. (b3CollideCapsuleAndSphere)
pub fn collide_capsule_and_sphere(
    manifold: &mut LocalManifold,
    capacity: i32,
    capsule_a: &Capsule,
    sphere_b: &Sphere,
    transform_b_to_a: Transform,
) {
    manifold.point_count = 0;

    if capacity < 1 {
        return;
    }

    // Work in shape B coordinates
    let center = transform_point(transform_b_to_a, sphere_b.center);
    let center1 = capsule_a.center1;
    let center2 = capsule_a.center2;

    let total_radius = sphere_b.radius + capsule_a.radius;

    let closest_point = point_to_segment_distance(center1, center2, center);
    let offset = sub(center, closest_point);
    let distance_sq = length_squared(offset);

    if distance_sq > total_radius * total_radius {
        // We found a separating axis
        return;
    }

    let mut normal = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    let distance = distance_sq.sqrt();
    if distance * distance > 1000.0 * f32::MIN_POSITIVE {
        normal = mul_sv(1.0 / distance, offset);
    }

    // Contact at the midpoint
    // 0.5 * (((center - sB*n) + closestPoint) + cA*n)
    let point = mul_sv(
        0.5,
        mul_add(
            add(
                mul_sub(center, sphere_b.radius, normal),
                closest_point,
            ),
            capsule_a.radius,
            normal,
        ),
    );

    // Manifold in frame B
    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = &mut manifold.points[0];
    pt.point = point;
    pt.separation = distance - total_radius;
    pt.pair = FEATURE_PAIR_SINGLE;
}

/// Collide a hull and a sphere. (b3CollideHullAndSphere)
pub fn collide_hull_and_sphere(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    sphere_b: &Sphere,
    transform_b_to_a: Transform,
    cache: &mut SimplexCache,
) {
    manifold.point_count = 0;

    if capacity == 0 {
        return;
    }

    let center = transform_point(transform_b_to_a, sphere_b.center);

    let speculative = speculative_distance();

    // Work in shapeA coordinates
    let distance_input = DistanceInput {
        proxy_a: make_proxy(get_hull_points(hull_a), 0.0),
        proxy_b: make_proxy(&[center], 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let radius_a = 0.0;
    let radius_b = sphere_b.radius;
    let radius = radius_a + radius_b;

    let distance_output = shape_distance(&distance_input, cache, None);

    if distance_output.distance > radius + speculative {
        // We found a separating axis
        *cache = SimplexCache::default();
        return;
    }

    if distance_output.distance > 100.0 * f32::EPSILON {
        // Shallow penetration
        let normal = normalize(sub(distance_output.point_b, distance_output.point_a));

        // cA is the projection of the sphere center onto to the hull (pointA if radiusA == 0).
        let c_a = mul_add(
            center,
            radius_a - dot(sub(center, distance_output.point_a), normal),
            normal,
        );

        // cB is the deepest point on the sphere with respect to the reference f
        let c_b = mul_sub(center, radius_b, normal);

        let point = lerp(c_a, c_b, 0.5);

        // Manifold in frame A
        manifold.normal = normal;
        manifold.point_count = 1;

        let pt = &mut manifold.points[0];
        pt.point = point;
        pt.separation = distance_output.distance - radius;
        pt.pair = FEATURE_PAIR_SINGLE;
    } else {
        // Deep penetration
        let mut best_index = -1;
        let mut best_distance = -f32::MAX;
        let planes = get_hull_planes(hull_a);

        for (index, plane) in planes.iter().enumerate() {
            let distance = plane_separation(*plane, center);
            if distance > best_distance {
                best_index = index as i32;
                best_distance = distance;
            }
        }
        debug_assert!(best_index >= 0);

        let normal = planes[best_index as usize].normal;

        // cA is the projection of the sphere center onto to the hull
        let c_a = mul_add(
            center,
            radius_a - dot(sub(center, distance_output.point_a), normal),
            normal,
        );

        // cB is the deepest point on the sphere with respect to the reference f
        let c_b = mul_sub(center, radius_b, normal);

        let point = lerp(c_a, c_b, 0.5);

        // Manifold in frame A
        manifold.normal = normal;
        manifold.point_count = 1;

        let pt = &mut manifold.points[0];
        pt.point = point;
        pt.separation = best_distance - radius;
        pt.pair = FEATURE_PAIR_SINGLE;
    }
}
