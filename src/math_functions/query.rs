// AABB helpers and segment/line/point distance queries.
// Part of the math_functions module.

use super::*;

/// Get the AABB of a point cloud.
pub fn make_aabb(points: &[Vec3], radius: f32) -> Aabb {
    debug_assert!(!points.is_empty());
    let mut a = Aabb {
        lower_bound: points[0],
        upper_bound: points[0],
    };
    for point in &points[1..] {
        a.lower_bound = min(a.lower_bound, *point);
        a.upper_bound = max(a.upper_bound, *point);
    }

    let r = Vec3 {
        x: radius,
        y: radius,
        z: radius,
    };
    a.lower_bound = sub(a.lower_bound, r);
    a.upper_bound = add(a.upper_bound, r);

    a
}

/// Does a fully contain b?
pub fn aabb_contains(a: Aabb, b: Aabb) -> bool {
    if a.lower_bound.x > b.lower_bound.x || b.upper_bound.x > a.upper_bound.x {
        return false;
    }
    if a.lower_bound.y > b.lower_bound.y || b.upper_bound.y > a.upper_bound.y {
        return false;
    }
    if a.lower_bound.z > b.lower_bound.z || b.upper_bound.z > a.upper_bound.z {
        return false;
    }

    true
}

/// Get the surface area of an axis-aligned bounding box.
pub fn aabb_area(a: Aabb) -> f32 {
    let delta = sub(a.upper_bound, a.lower_bound);
    2.0 * (delta.x * delta.y + delta.y * delta.z + delta.z * delta.x)
}

/// Get the center of an axis-aligned bounding box.
pub fn aabb_center(a: Aabb) -> Vec3 {
    mul_sv(0.5, add(a.upper_bound, a.lower_bound))
}

/// Get the extents (half-widths) of an axis-aligned bounding box.
pub fn aabb_extents(a: Aabb) -> Vec3 {
    mul_sv(0.5, sub(a.upper_bound, a.lower_bound))
}

/// Get the union of two axis-aligned bounding boxes.
pub fn aabb_union(a: Aabb, b: Aabb) -> Aabb {
    Aabb {
        lower_bound: min(a.lower_bound, b.lower_bound),
        upper_bound: max(a.upper_bound, b.upper_bound),
    }
}

/// Add uniform padding to an axis-aligned bounding box.
pub fn aabb_inflate(a: Aabb, extension: f32) -> Aabb {
    let radius = Vec3 {
        x: extension,
        y: extension,
        z: extension,
    };

    Aabb {
        lower_bound: sub(a.lower_bound, radius),
        upper_bound: add(a.upper_bound, radius),
    }
}

/// Do two axis-aligned boxes overlap?
pub fn aabb_overlaps(a: Aabb, b: Aabb) -> bool {
    // No intersection if separated along one axis
    if a.upper_bound.x < b.lower_bound.x || a.lower_bound.x > b.upper_bound.x {
        return false;
    }
    if a.upper_bound.y < b.lower_bound.y || a.lower_bound.y > b.upper_bound.y {
        return false;
    }
    if a.upper_bound.z < b.lower_bound.z || a.lower_bound.z > b.upper_bound.z {
        return false;
    }

    // Overlapping on all axis means bounds are intersecting
    true
}

/// Transform an axis-aligned bounding box. This can create a larger box
/// than if you recomputed the AABB of the original shape with the transform
/// applied.
pub fn aabb_transform(transform: Transform, a: Aabb) -> Aabb {
    let center = transform_point(transform, aabb_center(a));
    let m = make_matrix_from_quat(transform.q);
    let extent = mul_mv(abs_matrix3(m), aabb_extents(a));
    Aabb {
        lower_bound: sub(center, extent),
        upper_bound: add(center, extent),
    }
}

/// Get the closest point on an axis-aligned bounding box.
pub fn closest_point_to_aabb(point: Vec3, a: Aabb) -> Vec3 {
    clamp(point, a.lower_bound, a.upper_bound)
}

/// Compute the closest point on the segment a-b to the target q.
pub fn point_to_segment_distance(a: Vec3, b: Vec3, q: Vec3) -> Vec3 {
    let ab = sub(b, a);
    let aq = sub(q, a);

    let alpha = dot(ab, aq);

    if alpha <= 0.0 {
        // q projects outside interval [a, b] on the side of a
        a
    } else {
        let denominator = dot(ab, ab);
        if alpha > denominator {
            // q projects outside interval [a, b] on the side of b
            b
        } else {
            // q projects inside interval [a, b]
            let alpha = alpha / denominator;
            mul_add(a, alpha, ab)
        }
    }
}

/// Compute the closest points on two infinite lines.
pub fn line_distance(p1: Vec3, d1: Vec3, p2: Vec3, d2: Vec3) -> SegmentDistanceResult {
    // Solve A*x = b
    let a11 = dot(d1, d1);
    let a12 = -dot(d1, d2);
    let a21 = dot(d2, d1);
    let a22 = -dot(d2, d2);

    let w = sub(p1, p2);
    let b1 = -dot(d1, w);
    let b2 = -dot(d2, w);

    let det = a11 * a22 - a12 * a21;
    if det * det < 1000.0 * f32::MIN_POSITIVE {
        // Lines are parallel - project p2 onto line L1: x1 = p1 + s1 * d1
        let s1 = dot(sub(p2, p1), d1) / dot(d1, d1);
        let s2 = 0.0;

        return SegmentDistanceResult {
            point1: mul_add(p1, s1, d1),
            fraction1: s1,
            point2: mul_add(p2, s2, d2),
            fraction2: s2,
        };
    }

    let s1 = (a22 * b1 - a12 * b2) / det;
    let s2 = (a11 * b2 - a21 * b1) / det;

    SegmentDistanceResult {
        point1: mul_add(p1, s1, d1),
        fraction1: s1,
        point2: mul_add(p2, s2, d2),
        fraction2: s2,
    }
}

/// Compute the closest points on two line segments.
pub fn segment_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> SegmentDistanceResult {
    let d1 = sub(q1, p1);
    let d2 = sub(q2, p2);
    let r = sub(p1, p2);

    let a = dot(d1, d1);
    let b = dot(d1, d2);
    let c = dot(d1, r);
    let e = dot(d2, d2);
    let f = dot(d2, r);

    // Check if one of the segments degenerates into a point
    if a < 100.0 * f32::EPSILON && e < 100.0 * f32::EPSILON {
        // Both segments degenerate into points
        return SegmentDistanceResult {
            point1: p1,
            fraction1: 0.0,
            point2: p2,
            fraction2: 0.0,
        };
    }

    if a < 100.0 * f32::EPSILON {
        // First segment degenerates into a point
        let s2 = clamp_float(f / e, 0.0, 1.0);

        return SegmentDistanceResult {
            point1: p1,
            fraction1: 0.0,
            point2: mul_add(p2, s2, d2),
            fraction2: s2,
        };
    }

    if e < 100.0 * f32::EPSILON {
        // Second segment degenerates into a point
        let s1 = clamp_float(-c / a, 0.0, 1.0);

        return SegmentDistanceResult {
            point1: mul_add(p1, s1, d1),
            fraction1: s1,
            point2: p2,
            fraction2: 0.0,
        };
    }

    // Non-degenerate case
    let denom = a * e - b * b;
    let mut s1 = if denom > 1000.0 * f32::MIN_POSITIVE {
        clamp_float((b * f - c * e) / denom, 0.0, 1.0)
    } else {
        0.0
    };
    let mut s2 = (b * s1 + f) / e;

    // Clamp lambda2 and recompute lambda1 if necessary
    if s2 < 0.0 {
        s1 = clamp_float(-c / a, 0.0, 1.0);
        s2 = 0.0;
    } else if s2 > 1.0 {
        s1 = clamp_float((b - c) / a, 0.0, 1.0);
        s2 = 1.0;
    }

    SegmentDistanceResult {
        point1: mul_add(p1, s1, d1),
        fraction1: s1,
        point2: mul_add(p2, s2, d2),
        fraction2: s2,
    }
}
