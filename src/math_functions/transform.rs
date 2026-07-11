// Rigid transforms and world-position (large-world) operations.
// Part of the math_functions module.

use super::*;

#[cfg(feature = "double-precision")]
type PosScalar = f64;
#[cfg(not(feature = "double-precision"))]
type PosScalar = f32;

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame. This is useful if frame B is a child of frame A.
pub fn mul_transforms(a: Transform, b: Transform) -> Transform {
    Transform {
        p: add(rotate_vector(a.q, b.p), a.p),
        q: mul_quat(a.q, b.q),
    }
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
/// This is useful for transforming points between the local spaces of two frames that are
/// in world space.
pub fn inv_mul_transforms(a: Transform, b: Transform) -> Transform {
    Transform {
        p: inv_rotate_vector(a.q, sub(b.p, a.p)),
        q: inv_mul_quat(a.q, b.q),
    }
}

/// Get the inverse of a transform.
pub fn invert_transform(t: Transform) -> Transform {
    Transform {
        p: inv_rotate_vector(t.q, neg(t.p)),
        q: conjugate(t.q),
    }
}

/// Transform a point.
pub fn transform_point(t: Transform, v: Vec3) -> Vec3 {
    let rv = rotate_vector(t.q, v);
    add(rv, t.p)
}

/// Inverse transform a point.
pub fn inv_transform_point(t: Transform, v: Vec3) -> Vec3 {
    inv_rotate_vector(t.q, sub(v, t.p))
}

// World position boundary. These cross between the double precision world space at the public
// boundary and the float interior. One set of bodies serves both modes: the typedefs collapse
// the types in float mode and the explicit float casts become no-ops.

/// Convert a vector to a world position.
pub fn to_pos(v: Vec3) -> Pos {
    Pos {
        x: v.x as PosScalar,
        y: v.y as PosScalar,
        z: v.z as PosScalar,
    }
}

/// Lossy conversion of a world position to a float vector.
pub fn to_vec3(p: Pos) -> Vec3 {
    Vec3 {
        x: p.x as f32,
        y: p.y as f32,
        z: p.z as f32,
    }
}

/// Narrow a world coordinate to float, rounding toward negative infinity. Use with
/// [`round_up_float`] to build a conservative float box that always contains the double bounds,
/// where plain rounding far from the origin could clip. nextafterf is an exact IEEE operation,
/// so this is cross-platform deterministic. With large world mode off this is a plain conversion.
#[cfg(feature = "double-precision")]
pub fn round_down_float(x: f64) -> f32 {
    let f = x as f32;
    if f as f64 > x {
        next_after_f32(f, f32::MIN)
    } else {
        f
    }
}

/// Narrow a world coordinate to float, rounding toward positive infinity.
#[cfg(feature = "double-precision")]
pub fn round_up_float(x: f64) -> f32 {
    let f = x as f32;
    if (f as f64) < x {
        next_after_f32(f, f32::MAX)
    } else {
        f
    }
}

#[cfg(not(feature = "double-precision"))]
pub fn round_down_float(x: f64) -> f32 {
    x as f32
}

#[cfg(not(feature = "double-precision"))]
pub fn round_up_float(x: f64) -> f32 {
    x as f32
}

/// C nextafterf: the next representable f32 after `from` in the direction of `to`.
#[cfg(feature = "double-precision")]
fn next_after_f32(from: f32, to: f32) -> f32 {
    if from.is_nan() || to.is_nan() {
        return f32::NAN;
    }
    if from == to {
        return to;
    }
    if from == 0.0 {
        return if to > 0.0 {
            f32::from_bits(1)
        } else {
            -f32::from_bits(1)
        };
    }
    let bits = from.to_bits();
    let next = if (from < to) == (from > 0.0) {
        bits + 1
    } else {
        bits - 1
    };
    f32::from_bits(next)
}

/// a - b, demoted to float. The primary precision boundary operation.
pub fn sub_pos(a: Pos, b: Pos) -> Vec3 {
    Vec3 {
        x: (a.x - b.x) as f32,
        y: (a.y - b.y) as f32,
        z: (a.z - b.z) as f32,
    }
}

/// p + d
pub fn offset_pos(p: Pos, d: Vec3) -> Pos {
    Pos {
        x: p.x + d.x as PosScalar,
        y: p.y + d.y as PosScalar,
        z: p.z + d.z as PosScalar,
    }
}

/// World position interpolation for sweeps and sampling.
pub fn lerp_position(a: Pos, b: Pos, t: f32) -> Pos {
    Pos {
        x: (1.0 - t) as PosScalar * a.x + t as PosScalar * b.x,
        y: (1.0 - t) as PosScalar * a.y + t as PosScalar * b.y,
        z: (1.0 - t) as PosScalar * a.z + t as PosScalar * b.z,
    }
}

/// Transform a local point to a world position. Rotation in float, translation in double.
pub fn transform_world_point(t: WorldTransform, p: Vec3) -> Pos {
    let r = rotate_vector(t.q, p);
    Pos {
        x: t.p.x + r.x as PosScalar,
        y: t.p.y + r.y as PosScalar,
        z: t.p.z + r.z as PosScalar,
    }
}

/// Transform a world position to a local point. One double subtraction, then float.
pub fn inv_transform_world_point(t: WorldTransform, p: Pos) -> Vec3 {
    let d = Vec3 {
        x: (p.x - t.p.x) as f32,
        y: (p.y - t.p.y) as f32,
        z: (p.z - t.p.z) as f32,
    };
    inv_rotate_vector(t.q, d)
}

/// Relative transform of frame B in frame A. The narrow phase boundary.
pub fn inv_mul_world_transforms(a: WorldTransform, b: WorldTransform) -> Transform {
    let d = Vec3 {
        x: (b.p.x - a.p.x) as f32,
        y: (b.p.y - a.p.y) as f32,
        z: (b.p.z - a.p.z) as f32,
    };
    Transform {
        q: inv_mul_quat(a.q, b.q),
        p: inv_rotate_vector(a.q, d),
    }
}

/// Compose a world transform with a local transform.
pub fn mul_world_transforms(a: WorldTransform, b: Transform) -> WorldTransform {
    let r = rotate_vector(a.q, b.p);
    WorldTransform {
        q: mul_quat(a.q, b.q),
        p: Pos {
            x: a.p.x + r.x as PosScalar,
            y: a.p.y + r.y as PosScalar,
            z: a.p.z + r.z as PosScalar,
        },
    }
}

/// Shift a world transform into the frame of a base position.
pub fn to_relative_transform(t: WorldTransform, base: Pos) -> Transform {
    Transform {
        q: t.q,
        p: Vec3 {
            x: (t.p.x - base.x) as f32,
            y: (t.p.y - base.y) as f32,
            z: (t.p.z - base.z) as f32,
        },
    }
}

/// Promote a float transform to a world transform. Lossless.
pub fn make_world_transform(t: Transform) -> WorldTransform {
    WorldTransform {
        p: to_pos(t.p),
        q: t.q,
    }
}

/// Translate a local AABB by a world origin, rounding outward so the float box always contains
/// the double box. Far from the origin a plain conversion could clip a shape out of its own box.
/// In float mode the origin is float and the rounding is a no-op.
pub fn offset_aabb(local_box: Aabb, origin: Pos) -> Aabb {
    Aabb {
        lower_bound: Vec3 {
            x: round_down_float(origin.x as f64 + local_box.lower_bound.x as f64),
            y: round_down_float(origin.y as f64 + local_box.lower_bound.y as f64),
            z: round_down_float(origin.z as f64 + local_box.lower_bound.z as f64),
        },
        upper_bound: Vec3 {
            x: round_up_float(origin.x as f64 + local_box.upper_bound.x as f64),
            y: round_up_float(origin.y as f64 + local_box.upper_bound.y as f64),
            z: round_up_float(origin.z as f64 + local_box.upper_bound.z as f64),
        },
    }
}
