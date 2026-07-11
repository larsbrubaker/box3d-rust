// Validity checks (is_valid_*). Part of the math_functions module.

use super::*;
use crate::constants;

/// Is this a valid number? Not NaN or infinity.
pub fn is_valid_float(a: f32) -> bool {
    if a.is_nan() {
        return false;
    }

    if a.is_infinite() {
        return false;
    }

    true
}

/// Is this a valid vector? Not NaN or infinity.
pub fn is_valid_vec3(a: Vec3) -> bool {
    if a.x.is_nan() || a.y.is_nan() || a.z.is_nan() {
        return false;
    }

    if a.x.is_infinite() || a.y.is_infinite() || a.z.is_infinite() {
        return false;
    }

    true
}

/// Is this a valid quaternion? Not NaN or infinity. Is normalized.
pub fn is_valid_quat(a: Quat) -> bool {
    if a.v.x.is_nan() || a.v.y.is_nan() || a.v.z.is_nan() || a.s.is_nan() {
        return false;
    }

    if a.v.x.is_infinite() || a.v.y.is_infinite() || a.v.z.is_infinite() || a.s.is_infinite() {
        return false;
    }

    is_normalized_quat(a)
}

/// Is this a valid transform? Not NaN or infinity. Is normalized.
pub fn is_valid_transform(a: Transform) -> bool {
    is_valid_vec3(a.p) && is_valid_quat(a.q)
}

/// Is this a valid matrix? Not NaN or infinity.
pub fn is_valid_matrix3(a: Matrix3) -> bool {
    is_valid_vec3(a.cx) && is_valid_vec3(a.cy) && is_valid_vec3(a.cz)
}

/// Is this a valid bounding box? Not Nan or infinity. Upper bound greater than or equal to lower bound.
pub fn is_valid_aabb(a: Aabb) -> bool {
    if !is_valid_vec3(a.lower_bound) {
        return false;
    }

    if !is_valid_vec3(a.upper_bound) {
        return false;
    }

    if a.lower_bound.x > a.upper_bound.x {
        return false;
    }

    if a.lower_bound.y > a.upper_bound.y {
        return false;
    }

    if a.lower_bound.z > a.upper_bound.z {
        return false;
    }

    true
}

/// Is this AABB reasonably close to the origin? See B3_HUGE.
pub fn is_bounded_aabb(a: Aabb) -> bool {
    let huge = constants::huge();
    if a.lower_bound.x < -huge || a.lower_bound.y < -huge || a.lower_bound.z < -huge {
        return false;
    }

    if a.upper_bound.x > huge || a.upper_bound.y > huge || a.upper_bound.z > huge {
        return false;
    }

    true
}

/// Is this AABB valid and reasonable?
pub fn is_sane_aabb(a: Aabb) -> bool {
    if !is_valid_aabb(a) {
        return false;
    }

    let huge = constants::huge();
    if a.lower_bound.x < -huge || a.lower_bound.y < -huge || a.lower_bound.z < -huge {
        return false;
    }

    if a.upper_bound.x > huge || a.upper_bound.y > huge || a.upper_bound.z > huge {
        return false;
    }

    true
}

/// Is this a valid plane? Normal is a unit vector. Not Nan or infinity.
pub fn is_valid_plane(a: Plane) -> bool {
    if !is_valid_vec3(a.normal) {
        return false;
    }

    if !is_normalized(a.normal) {
        return false;
    }

    is_valid_float(a.offset)
}

/// Is this a valid world position? Not NaN or infinity.
pub fn is_valid_position(p: Pos) -> bool {
    if p.x.is_nan() || p.y.is_nan() || p.z.is_nan() {
        return false;
    }

    if p.x.is_infinite() || p.y.is_infinite() || p.z.is_infinite() {
        return false;
    }

    true
}

/// Is this a valid world transform? Not NaN or infinity. Rotation is normalized.
pub fn is_valid_world_transform(t: WorldTransform) -> bool {
    is_valid_position(t.p) && is_valid_quat(t.q)
}
