//! Scalar ray-triangle and AABB-triangle overlap from simd.c (B3_SIMD_NONE path).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::*;

/// True if any component of `a` is strictly less than the corresponding component of `b`.
fn any_less3(a: Vec3, b: Vec3) -> bool {
    a.x < b.x || a.y < b.y || a.z < b.z
}

/// True if any component of `a` is strictly greater than the corresponding component of `b`.
fn any_greater3(a: Vec3, b: Vec3) -> bool {
    a.x > b.x || a.y > b.y || a.z > b.z
}

/// Transpose three column vectors in place (scalar B3_TRANSPOSE3).
fn transpose3(c1: &mut Vec3, c2: &mut Vec3, c3: &mut Vec3) {
    let temp1 = c1.y;
    let temp2 = c1.z;
    let temp3 = c2.z;

    c1.y = c2.x;
    c1.z = c3.x;
    c2.z = c3.y;

    c2.x = temp1;
    c3.x = temp2;
    c3.y = temp3;
}

/// Test overlap between an AABB (center + extent) and a triangle.
/// (simd.c: b3TestBoundsTriangleOverlap)
pub fn test_bounds_triangle_overlap(
    node_center: Vec3,
    node_extent: Vec3,
    mut vertex1: Vec3,
    mut vertex2: Vec3,
    mut vertex3: Vec3,
) -> bool {
    let two = Vec3 {
        x: 2.0,
        y: 2.0,
        z: 2.0,
    };

    // Setup triangle
    vertex1 = sub(vertex1, node_center);
    vertex2 = sub(vertex2, node_center);
    vertex3 = sub(vertex3, node_center);

    // Face separation
    let triangle_min = min(vertex1, min(vertex2, vertex3));
    let triangle_max = max(vertex1, max(vertex2, vertex3));

    let separation1 = sub(triangle_min, node_extent);
    let separation2 = add(triangle_max, node_extent);

    let face_separation = max(separation1, neg(separation2));
    if any_greater3(face_separation, VEC3_ZERO) {
        return false;
    }

    // SAT: Face separation
    let edge1 = sub(vertex2, vertex1);
    let edge2 = sub(vertex3, vertex2);
    let edge3 = sub(vertex1, vertex3);

    let normal = cross(edge1, edge2);

    let d = dot(normal, vertex1);
    let e = dot(abs(normal), node_extent);
    let triangle_separation = Vec3 {
        x: abs_float(d) - e,
        y: abs_float(d) - e,
        z: abs_float(d) - e,
    };
    if any_greater3(triangle_separation, VEC3_ZERO) {
        return false;
    }

    // SAT: Edge separation
    let edge_separation1 = sub(
        sub(
            abs(cross(edge1, add(vertex1, vertex3))),
            abs(cross(edge1, edge3)),
        ),
        mul(two, modified_cross(abs(edge1), node_extent)),
    );
    if any_greater3(edge_separation1, VEC3_ZERO) {
        return false;
    }

    let edge_separation2 = sub(
        sub(
            abs(cross(edge2, add(vertex1, vertex2))),
            abs(cross(edge2, edge1)),
        ),
        mul(two, modified_cross(abs(edge2), node_extent)),
    );
    if any_greater3(edge_separation2, VEC3_ZERO) {
        return false;
    }

    let edge_separation3 = sub(
        sub(
            abs(cross(edge3, add(vertex2, vertex3))),
            abs(cross(edge3, edge2)),
        ),
        mul(two, modified_cross(abs(edge3), node_extent)),
    );
    if any_greater3(edge_separation3, VEC3_ZERO) {
        return false;
    }

    true
}

/// True if all components of `a` are ≤ the corresponding components of `b`.
fn all_less_eq3(a: Vec3, b: Vec3) -> bool {
    a.x <= b.x && a.y <= b.y && a.z <= b.z
}

/// Test overlap between two AABBs given as min/max corners.
/// (simd.h: b3TestBoundsOverlap)
pub fn test_bounds_overlap(
    node_min1: Vec3,
    node_max1: Vec3,
    node_min2: Vec3,
    node_max2: Vec3,
) -> bool {
    let separation = max(sub(node_min2, node_max1), sub(node_min1, node_max2));
    all_less_eq3(separation, VEC3_ZERO)
}

/// Test a ray for edge separation with an AABB (Gino, p80).
/// (simd.h: b3TestBoundsRayOverlap)
pub fn test_bounds_ray_overlap(
    node_min: Vec3,
    node_max: Vec3,
    mut ray_start: Vec3,
    ray_delta: Vec3,
) -> bool {
    let node_center = mul_sv(0.5, add(node_min, node_max));
    let node_extent = sub(node_max, node_center);

    ray_start = sub(ray_start, node_center);

    let edge_separation = sub(
        abs(cross(ray_delta, ray_start)),
        modified_cross(abs(ray_delta), node_extent),
    );
    all_less_eq3(edge_separation, VEC3_ZERO)
}

/// Intersect a ray with a triangle. Returns the hit fraction in (0, 1], or 1.0 on miss.
/// (simd.c: b3IntersectRayTriangle, scalar path)
pub fn intersect_ray_triangle(
    ray_start: Vec3,
    ray_delta: Vec3,
    vertex1: Vec3,
    vertex2: Vec3,
    vertex3: Vec3,
) -> f32 {
    // Test if ray intersects this triangle sharing same calculations for each triangle
    {
        let edge1 = sub(vertex3, vertex2);
        let edge2 = sub(vertex1, vertex3);
        let edge3 = sub(vertex2, vertex1);

        let mid_point1 = mul_sv(0.5, add(vertex2, vertex3));
        let mid_point2 = mul_sv(0.5, add(vertex3, vertex1));
        let mid_point3 = mul_sv(0.5, add(vertex1, vertex2));

        let mut normal1 = cross(edge1, sub(mid_point1, ray_start));
        let mut normal2 = cross(edge2, sub(mid_point2, ray_start));
        let mut normal3 = cross(edge3, sub(mid_point3, ray_start));
        transpose3(&mut normal1, &mut normal2, &mut normal3);

        let ray_delta_x = Vec3 {
            x: ray_delta.x,
            y: ray_delta.x,
            z: ray_delta.x,
        };
        let ray_delta_y = Vec3 {
            x: ray_delta.y,
            y: ray_delta.y,
            z: ray_delta.y,
        };
        let ray_delta_z = Vec3 {
            x: ray_delta.z,
            y: ray_delta.z,
            z: ray_delta.z,
        };

        let volumes = add(
            add(mul(normal1, ray_delta_x), mul(normal2, ray_delta_y)),
            mul(normal3, ray_delta_z),
        );
        if any_less3(volumes, VEC3_ZERO) {
            return 1.0;
        }
    }

    // Compute intersection with triangle plane
    let edge1 = sub(vertex2, vertex1);
    let edge2 = sub(vertex3, vertex1);
    let normal = cross(edge1, edge2);

    let denominator = dot(normal, ray_delta);
    if denominator >= 0.0 {
        return 1.0;
    }

    let mut lambda = dot(normal, sub(vertex1, ray_start)) / denominator;
    if lambda <= 0.0 {
        return 1.0;
    }

    lambda = min_float(lambda, 1.0);
    lambda
}
