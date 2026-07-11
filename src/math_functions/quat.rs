// Quaternion operations.
// Part of the math_functions module.

use super::*;

/// Does the supplied quaternion have unit length?
pub fn is_normalized_quat(q: Quat) -> bool {
    let qq = q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z + q.s * q.s;
    1.0 - 20.0 * f32::EPSILON < qq && qq < 1.0 + 20.0 * f32::EPSILON
}

/// Rotate a vector.
pub fn rotate_vector(q: Quat, v: Vec3) -> Vec3 {
    // v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
    // B3_ASSERT( b3IsNormalizedQuat( q ) );
    let t1 = cross(q.v, v);
    let t2 = mul_add(t1, q.s, v);
    let t3 = cross(q.v, t2);
    mul_add(v, 2.0, t3)
}

/// Inverse rotate a vector.
pub fn inv_rotate_vector(q: Quat, v: Vec3) -> Vec3 {
    // v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
    // B3_ASSERT( b3IsNormalizedQuat( q ) );
    let t1 = cross(q.v, v);
    let t2 = mul_sub(t1, q.s, v);
    let t3 = cross(q.v, t2);
    mul_add(v, 2.0, t3)
}

/// Compute dot product of two quaternions. Useful for polarity tests.
pub fn dot_quat(a: Quat, b: Quat) -> f32 {
    a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s
}

/// Multiply two quaternions.
pub fn mul_quat(q1: Quat, q2: Quat) -> Quat {
    let t1 = cross(q1.v, q2.v);
    let t2 = mul_add(t1, q1.s, q2.v);
    let t3 = mul_add(t2, q2.s, q1.v);
    Quat {
        v: t3,
        s: q1.s * q2.s - dot(q1.v, q2.v),
    }
}

/// Compute a relative quaternion.
/// inv(q1) * q2
pub fn inv_mul_quat(q1: Quat, q2: Quat) -> Quat {
    let t1 = cross(q2.v, q1.v);
    let t2 = mul_add(t1, q1.s, q2.v);
    let t3 = mul_sub(t2, q2.s, q1.v);
    Quat {
        v: t3,
        s: q1.s * q2.s + dot(q1.v, q2.v),
    }
}

/// Quaternion conjugate (cheap inverse).
pub fn conjugate(q: Quat) -> Quat {
    Quat {
        v: Vec3 {
            x: -q.v.x,
            y: -q.v.y,
            z: -q.v.z,
        },
        s: q.s,
    }
}

/// Component-wise quaternion negation.
pub fn negate_quat(q: Quat) -> Quat {
    Quat {
        v: Vec3 {
            x: -q.v.x,
            y: -q.v.y,
            z: -q.v.z,
        },
        s: -q.s,
    }
}

/// Normalize a quaternion.
pub fn normalize_quat(q: Quat) -> Quat {
    let length_sq = dot_quat(q, q);
    if length_sq > 1000.0 * f32::MIN_POSITIVE {
        let s = 1.0 / length_sq.sqrt();
        Quat {
            v: Vec3 {
                x: s * q.v.x,
                y: s * q.v.y,
                z: s * q.v.z,
            },
            s: s * q.s,
        }
    } else {
        QUAT_IDENTITY
    }
}

/// Integrate rotation from angular velocity.
/// `delta_rotation` is the angular displacement in radians (ω · h).
/// q2 = q1 + 0.5 * omega * q1
/// (math_internal.h: b3IntegrateRotation)
pub fn integrate_rotation(q1: Quat, delta_rotation: Vec3) -> Quat {
    // https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
    let mut qd = Quat {
        v: mul_sv(0.5, delta_rotation),
        s: 0.0,
    };
    qd = mul_quat(qd, q1);
    let q2 = Quat {
        v: add(q1.v, qd.v),
        s: qd.s + q1.s,
    };
    normalize_quat(q2)
}

/// Make a quaternion that is equivalent to rotating around an axis by a specified angle.
pub fn make_quat_from_axis_angle(axis: Vec3, radians: f32) -> Quat {
    debug_assert!(is_normalized(axis));
    let cs = compute_cos_sin(0.5 * radians);
    Quat {
        v: Vec3 {
            x: cs.sine * axis.x,
            y: cs.sine * axis.y,
            z: cs.sine * axis.z,
        },
        s: cs.cosine,
    }
}

/// Get the axis and angle from a quaternion. Assumes the quaternion is normalized.
pub fn get_axis_angle(radians: &mut f32, q: Quat) -> Vec3 {
    let length = (q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z).sqrt();
    *radians = 2.0 * atan2(length, q.s);
    if length > 0.0 {
        let inv_length = 1.0 / length;
        Vec3 {
            x: inv_length * q.v.x,
            y: inv_length * q.v.y,
            z: inv_length * q.v.z,
        }
    } else {
        VEC3_ZERO
    }
}

/// Get the angle for a quaternion in radians
pub fn get_quat_angle(q: Quat) -> f32 {
    let length = (q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z).sqrt();
    2.0 * atan2(length, q.s)
}

/// Extract a quaternion from a rotation matrix.
pub fn make_quat_from_matrix(m: &Matrix3) -> Quat {
    let c1 = m.cx;
    let c2 = m.cy;
    let c3 = m.cz;

    let q: Quat;

    let trace = m.cx.x + m.cy.y + m.cz.z;
    if trace >= 0.0 {
        q = Quat {
            v: Vec3 {
                x: c2.z - c3.y,
                y: c3.x - c1.z,
                z: c1.y - c2.x,
            },
            s: trace + 1.0,
        };
    } else if c1.x > c2.y && c1.x > c3.z {
        q = Quat {
            v: Vec3 {
                x: c1.x - c2.y - c3.z + 1.0,
                y: c2.x + c1.y,
                z: c3.x + c1.z,
            },
            s: c2.z - c3.y,
        };
    } else if c2.y > c3.z {
        q = Quat {
            v: Vec3 {
                x: c1.y + c2.x,
                y: c2.y - c3.z - c1.x + 1.0,
                z: c3.y + c2.z,
            },
            s: c3.x - c1.z,
        };
    } else {
        q = Quat {
            v: Vec3 {
                x: c1.z + c3.x,
                y: c2.z + c3.y,
                z: c3.z - c1.x - c2.y + 1.0,
            },
            s: c1.y - c2.x,
        };
    }

    // The algorithm is simplified and made more accurate by normalizing at the end
    normalize_quat(q)
}

/// Find a quaternion that rotates one vector to another.
pub fn compute_quat_between_unit_vectors(v1: Vec3, v2: Vec3) -> Quat {
    debug_assert!(is_normalized(v1));
    debug_assert!(is_normalized(v2));

    let out: Quat;

    let m = lerp(v1, v2, 0.5);
    let tolerance = 100.0 * f32::EPSILON;
    if length_squared(m) > tolerance * tolerance {
        out = Quat {
            v: cross(v1, m),
            s: dot(v1, m),
        };
    } else {
        // Anti-parallel: Use a perpendicular vector
        if abs_float(v1.x) > 0.5 {
            out = Quat {
                v: Vec3 {
                    x: v1.y,
                    y: -v1.x,
                    z: 0.0,
                },
                s: 0.0,
            };
        } else {
            out = Quat {
                v: Vec3 {
                    x: 0.0,
                    y: v1.z,
                    z: -v1.y,
                },
                s: 0.0,
            };
        }
    }

    // The algorithm is simplified and made more accurate by normalizing at the end
    normalize_quat(out)
}

/// Twist angle around the z-axis, used for twist limit and revolute angle limit
pub fn get_twist_angle(q: Quat) -> f32 {
    // Account for polarity to keep the twist angle in range.
    // This is simpler than asking the user to check polarity or unwinding.
    let mut twist = if q.s < 0.0 {
        atan2(-q.v.z, -q.s)
    } else {
        atan2(q.v.z, q.s)
    };
    twist *= 2.0;
    debug_assert!((-PI..=PI).contains(&twist));
    twist
}

/// Swing angle used for cone limit
pub fn get_swing_angle(q: Quat) -> f32 {
    // Polarity should not matter because all terms are squared.
    let x = (q.v.z * q.v.z + q.s * q.s).sqrt();
    let y = (q.v.x * q.v.x + q.v.y * q.v.y).sqrt();
    let swing = 2.0 * atan2(y, x);
    debug_assert!((0.0..=PI).contains(&swing));
    swing
}

/// Linearly interpolate and normalize between two quaternions
pub fn nlerp(mut q1: Quat, q2: Quat, alpha: f32) -> Quat {
    debug_assert!((0.0..=1.0).contains(&alpha));
    if dot_quat(q1, q2) < 0.0 {
        q1 = Quat {
            v: Vec3 {
                x: -q1.v.x,
                y: -q1.v.y,
                z: -q1.v.z,
            },
            s: -q1.s,
        };
    }

    let q = Quat {
        v: lerp(q1.v, q2.v, alpha),
        s: (1.0 - alpha) * q1.s + alpha * q2.s,
    };

    normalize_quat(q)
}
