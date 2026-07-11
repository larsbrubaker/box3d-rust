// Vec3 arithmetic, length/normalize, and closely related helpers.
// Part of the math_functions module.

use super::*;

/// Vector addition.
pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x + b.x,
        y: a.y + b.y,
        z: a.z + b.z,
    }
}

/// Vector subtraction.
pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
}

/// Vector component-wise multiplication.
pub fn mul(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x * b.x,
        y: a.y * b.y,
        z: a.z * b.z,
    }
}

/// Vector negation.
pub fn neg(a: Vec3) -> Vec3 {
    Vec3 {
        x: -a.x,
        y: -a.y,
        z: -a.z,
    }
}

/// Vector dot product.
pub fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Vector length.
pub fn length(v: Vec3) -> f32 {
    dot(v, v).sqrt()
}

/// Vector length squared.
pub fn length_squared(a: Vec3) -> f32 {
    a.x * a.x + a.y * a.y + a.z * a.z
}

/// Distance between two points.
pub fn distance(a: Vec3, b: Vec3) -> f32 {
    let dv = Vec3 {
        x: b.x - a.x,
        y: b.y - a.y,
        z: b.z - a.z,
    };
    length(dv)
}

/// Squared distance between two points.
pub fn distance_squared(a: Vec3, b: Vec3) -> f32 {
    let dv = Vec3 {
        x: b.x - a.x,
        y: b.y - a.y,
        z: b.z - a.z,
    };
    dv.x * dv.x + dv.y * dv.y + dv.z * dv.z
}

/// Normalize a vector. Returns a zero vector if the input vector is very small.
pub fn normalize(a: Vec3) -> Vec3 {
    let length_squared = a.x * a.x + a.y * a.y + a.z * a.z;

    if length_squared > 1000.0 * f32::MIN_POSITIVE {
        let s = 1.0 / length_squared.sqrt();
        Vec3 {
            x: s * a.x,
            y: s * a.y,
            z: s * a.z,
        }
    } else {
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

/// Normalize a vector and return the length. Returns a zero vector
/// if the input is very small.
pub fn get_length_and_normalize(length_out: &mut f32, a: Vec3) -> Vec3 {
    *length_out = length(a);
    if *length_out < f32::EPSILON {
        return VEC3_ZERO;
    }

    let inv_length = 1.0 / *length_out;
    Vec3 {
        x: inv_length * a.x,
        y: inv_length * a.y,
        z: inv_length * a.z,
    }
}

/// Get a unit vector that is perpendicular to the supplied vector.
pub fn perp(a: Vec3) -> Vec3 {
    // Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
    // Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
    // of a unit vector must be greater or equal to 0.57735.
    let p = if a.x < -0.5 || 0.5 < a.x {
        Vec3 {
            x: a.y,
            y: -a.x,
            z: 0.0,
        }
    } else {
        Vec3 {
            x: 0.0,
            y: a.z,
            z: -a.y,
        }
    };

    normalize(p)
}

/// Is a vector normalized? In other words, does it have unit length?
pub fn is_normalized(a: Vec3) -> bool {
    let aa = dot(a, a);
    abs_float(1.0 - aa) < 100.0 * f32::EPSILON
}

/// a + s * b
pub fn mul_add(a: Vec3, s: f32, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x + s * b.x,
        y: a.y + s * b.y,
        z: a.z + s * b.z,
    }
}

/// a - s * b
pub fn mul_sub(a: Vec3, s: f32, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.x - s * b.x,
        y: a.y - s * b.y,
        z: a.z - s * b.z,
    }
}

/// s * a
pub fn mul_sv(s: f32, a: Vec3) -> Vec3 {
    Vec3 {
        x: s * a.x,
        y: s * a.y,
        z: s * a.z,
    }
}

/// <https://en.wikipedia.org/wiki/Cross_product>
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x,
    }
}

/// Linearly interpolate between two vectors.
pub fn lerp(a: Vec3, b: Vec3, alpha: f32) -> Vec3 {
    debug_assert!((0.0..=1.0).contains(&alpha));

    Vec3 {
        x: (1.0 - alpha) * a.x + alpha * b.x,
        y: (1.0 - alpha) * a.y + alpha * b.y,
        z: (1.0 - alpha) * a.z + alpha * b.z,
    }
}

/// Blend two vectors: s * a + t * b
pub fn blend2(s: f32, a: Vec3, t: f32, b: Vec3) -> Vec3 {
    Vec3 {
        x: s * a.x + t * b.x,
        y: s * a.y + t * b.y,
        z: s * a.z + t * b.z,
    }
}

/// Component-wise absolute value.
pub fn abs(a: Vec3) -> Vec3 {
    Vec3 {
        x: abs_float(a.x),
        y: abs_float(a.y),
        z: abs_float(a.z),
    }
}

/// Component-wise -1 or 1 (1 if zero).
pub fn sign(a: Vec3) -> Vec3 {
    Vec3 {
        x: if a.x >= 0.0 { 1.0 } else { -1.0 },
        y: if a.y >= 0.0 { 1.0 } else { -1.0 },
        z: if a.z >= 0.0 { 1.0 } else { -1.0 },
    }
}

/// Component-wise minimum value.
pub fn min(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: min_float(a.x, b.x),
        y: min_float(a.y, b.y),
        z: min_float(a.z, b.z),
    }
}

/// Component-wise maximum value.
pub fn max(a: Vec3, b: Vec3) -> Vec3 {
    Vec3 {
        x: max_float(a.x, b.x),
        y: max_float(a.y, b.y),
        z: max_float(a.z, b.z),
    }
}

/// Component-wise clamped value.
pub fn clamp(a: Vec3, lower: Vec3, upper: Vec3) -> Vec3 {
    Vec3 {
        x: clamp_float(a.x, lower.x, upper.x),
        y: clamp_float(a.y, lower.y, upper.y),
        z: clamp_float(a.z, lower.z, upper.z),
    }
}

/// Create a safe scaling value for scaling collision. This allows
/// negative scale, but keeps scale sufficiently far from zero.
pub fn safe_scale(a: Vec3) -> Vec3 {
    let abs_scale = abs(a);
    let min_scale = Vec3 {
        x: MIN_SCALE,
        y: MIN_SCALE,
        z: MIN_SCALE,
    };
    mul(sign(a), max(abs_scale, min_scale))
}
