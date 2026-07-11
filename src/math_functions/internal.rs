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
