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
