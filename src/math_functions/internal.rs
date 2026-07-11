// Internal helpers from math_internal.h needed by tests (and later by collision).
// Part of the math_functions module.

use super::*;

/// Assume v is a unit vector. (math_internal.h: b3ArbitraryPerp)
pub fn arbitrary_perp(v: Vec3) -> Vec3 {
    // Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
    // Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
    // of a unit vector must be greater or equal to 0.57735.
    let p = if v.x < -0.5 || 0.5 < v.x {
        // x is non-zero and it should not go into the x component
        // dot([ay + bz, cx, dx], [x, y, z]) = ayx + bzx + cxy + dzx
        // for the dot product to be zero need: c = -a, d = -b
        let a = 0.67;
        let b = -0.42;
        Vec3 {
            x: a * v.y + b * v.z,
            y: -a * v.x,
            z: -b * v.x,
        }
    } else if v.y < -0.5 || 0.5 < v.y {
        // y is non-zero and it should not go into the y component
        // p = [ay, bx + cz, dy]
        // axy + bxy + cyz + dyz = 0
        // b = -a, d = -c
        let a = 0.67;
        let c = -0.42;
        Vec3 {
            x: a * v.y,
            y: -a * v.x + c * v.z,
            z: -c * v.y,
        }
    } else {
        // This would trip if the input is not a unit vector
        debug_assert!(v.z < -0.5 || 0.5 < v.z);

        // z is non-zero and it should not go into the z component
        // p = [az, bz, cx + dy]
        // axz + byz + cxz + dyz = 0
        // c = -a, d = -b
        let a = 0.67;
        let b = -0.42;
        Vec3 {
            x: a * v.z,
            y: b * v.z,
            z: -a * v.x - b * v.y,
        }
    };

    debug_assert!(length_squared(p) > 0.1);
    debug_assert!(abs_float(dot(p, v)) < 100.0 * f32::EPSILON);

    normalize(p)
}

/// Scalar triple product a · (b × c). (math_internal.h: b3ScalarTripleProduct)
pub fn scalar_triple_product(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let d = Vec3 {
        x: b.y * c.z - b.z * c.y,
        y: b.z * c.x - b.x * c.z,
        z: b.x * c.y - b.y * c.x,
    };
    a.x * d.x + a.y * d.y + a.z * d.z
}

/// √3. (math_internal.h: B3_SQRT3)
pub const SQRT3: f32 = 1.732050808;

/// Empty AABB (inverted bounds). (math_internal.h: B3_BOUNDS3_EMPTY)
pub const BOUNDS3_EMPTY: Aabb = Aabb {
    lower_bound: Vec3 {
        x: f32::MAX,
        y: f32::MAX,
        z: f32::MAX,
    },
    upper_bound: Vec3 {
        x: -f32::MAX,
        y: -f32::MAX,
        z: -f32::MAX,
    },
};

/// Align `x` up to a multiple of 8. (math_internal.h: b3AlignUp8)
pub fn align_up8(x: usize) -> usize {
    (x + 7) & !7
}

/// Index of the largest component. (math_internal.h: b3MaxElementIndex)
pub fn max_element_index(v: Vec3) -> i32 {
    if v.x < v.y {
        if v.y < v.z {
            2
        } else {
            1
        }
    } else if v.x < v.z {
        2
    } else {
        0
    }
}

/// Diagonal matrix. (math_internal.h: b3MakeDiagonalMatrix)
pub fn make_diagonal_matrix(a: f32, b: f32, c: f32) -> Matrix3 {
    Matrix3 {
        cx: Vec3 {
            x: a,
            y: 0.0,
            z: 0.0,
        },
        cy: Vec3 {
            x: 0.0,
            y: b,
            z: 0.0,
        },
        cz: Vec3 {
            x: 0.0,
            y: 0.0,
            z: c,
        },
    }
}

/// True if both closest-point fractions lie on their segments. (math_internal.h: b3IsWithinSegments)
pub fn is_within_segments(result: &SegmentDistanceResult) -> bool {
    (0.0 <= result.fraction1 && result.fraction1 <= 1.0)
        && (0.0 <= result.fraction2 && result.fraction2 <= 1.0)
}

/// Plane through `point` with given `normal`. (math_internal.h: b3MakePlaneFromNormalAndPoint)
pub fn make_plane_from_normal_and_point(normal: Vec3, point: Vec3) -> Plane {
    Plane {
        normal,
        offset: dot(normal, point),
    }
}

/// Plane through three points. (math_internal.h: b3MakePlaneFromPoints)
pub fn make_plane_from_points(point1: Vec3, point2: Vec3, point3: Vec3) -> Plane {
    let mut plane = Plane {
        normal: cross(sub(point2, point1), sub(point3, point1)),
        offset: 0.0,
    };
    plane.normal = normalize(plane.normal);
    plane.offset = dot(plane.normal, point1);
    plane
}

/// Transform a plane by a rigid transform. (math_internal.h: b3TransformPlane)
pub fn transform_plane(transform: Transform, plane: Plane) -> Plane {
    let normal = rotate_vector(transform.q, plane.normal);
    Plane {
        normal,
        offset: plane.offset + dot(normal, transform.p),
    }
}

/// Signed separation of a point from a plane. (math_internal.h: b3PlaneSeparation)
pub fn plane_separation(plane: Plane, point: Vec3) -> f32 {
    dot(plane.normal, point) - plane.offset
}

/// Rotate a central inertia tensor by a quaternion. (math_internal.h: b3RotateInertia)
pub fn rotate_inertia(q: Quat, central_inertia: Matrix3) -> Matrix3 {
    let rotation_matrix = make_matrix_from_quat(q);
    mul_mm(
        rotation_matrix,
        mul_mm(central_inertia, transpose(rotation_matrix)),
    )
}

/// Box inertia about the center for an AABB from `min` to `max`.
/// (math_functions.c: b3BoxInertia)
pub fn box_inertia(mass: f32, min: Vec3, max: Vec3) -> Matrix3 {
    let delta = sub(max, min);
    let ixx = mass * (delta.y * delta.y + delta.z * delta.z) / 12.0;
    let iyy = mass * (delta.x * delta.x + delta.z * delta.z) / 12.0;
    let izz = mass * (delta.x * delta.x + delta.y * delta.y) / 12.0;
    make_diagonal_matrix(ixx, iyy, izz)
}

/// Solid sphere inertia about its center. (math_functions.c: b3SphereInertia)
pub fn sphere_inertia(mass: f32, radius: f32) -> Matrix3 {
    let i = 0.4 * mass * radius * radius;
    make_diagonal_matrix(i, i, i)
}

/// Solid cylinder inertia about its center, axis along Y.
/// (math_functions.c: b3CylinderInertia)
pub fn cylinder_inertia(mass: f32, radius: f32, height: f32) -> Matrix3 {
    let ixx = mass * (3.0 * radius * radius + height * height) / 12.0;
    let iyy = 0.5 * mass * radius * radius;
    make_diagonal_matrix(ixx, iyy, ixx)
}

/// Closest point on a triangle and the feature that owns it.
/// (math_internal.h: b3TrianglePoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrianglePoint {
    pub point: Vec3,
    pub feature: crate::manifold::TriangleFeature,
}

/// Closest point on triangle ABC to query point Q (Ericson §5.1.5).
/// (math_functions.c: b3ClosestPointOnTriangle)
pub fn closest_point_on_triangle(a: Vec3, b: Vec3, c: Vec3, q: Vec3) -> TrianglePoint {
    use crate::manifold::TriangleFeature;

    // Check if P lies in vertex region of A
    let ab = sub(b, a);
    let ac = sub(c, a);
    let aq = sub(q, a);

    let d1 = dot(ab, aq);
    let d2 = dot(ac, aq);
    if d1 <= 0.0 && d2 <= 0.0 {
        return TrianglePoint {
            point: a,
            feature: TriangleFeature::Vertex1,
        };
    }

    // Check if P lies in vertex region of B
    let bq = sub(q, b);

    let d3 = dot(ab, bq);
    let d4 = dot(ac, bq);
    if d3 > 0.0 && d4 <= d3 {
        return TrianglePoint {
            point: b,
            feature: TriangleFeature::Vertex2,
        };
    }

    // Check if P lies in edge region AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let t = d1 / (d1 - d3);
        return TrianglePoint {
            point: mul_add(a, t, ab),
            feature: TriangleFeature::Edge1,
        };
    }

    // Check if P lies in vertex region of C
    let cq = sub(q, c);

    let d5 = dot(ab, cq);
    let d6 = dot(ac, cq);
    if d6 >= 0.0 && d5 <= d6 {
        return TrianglePoint {
            point: c,
            feature: TriangleFeature::Vertex3,
        };
    }

    // Check if P lies in edge region AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let t = d2 / (d2 - d6);
        return TrianglePoint {
            point: mul_add(a, t, ac),
            feature: TriangleFeature::Edge3,
        };
    }

    // Check if P lies in edge region of BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 >= d3 && d5 >= d6 {
        let bc = sub(c, b);
        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return TrianglePoint {
            point: mul_add(b, t, bc),
            feature: TriangleFeature::Edge2,
        };
    }

    // P inside face region ABC
    let t1 = vb / (va + vb + vc);
    let t2 = vc / (va + vb + vc);

    let mut p = mul_add(a, t1, ab);
    p = mul_add(p, t2, ac);
    TrianglePoint {
        point: p,
        feature: TriangleFeature::TriangleFace,
    }
}
