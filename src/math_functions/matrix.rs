// Matrix3 operations, Steiner parallel-axis helper, and Mat2 helpers for tests.
// Part of the math_functions module.

use super::*;

/// A 2x2 matrix stored as columns. Ported from math_internal.h for tests and
/// internal use (`b3Matrix2`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat2 {
    pub cx: Vec2,
    pub cy: Vec2,
}

/// Compute the determinant of a 3-by-3 matrix.
pub fn det(m: Matrix3) -> f32 {
    dot(m.cx, cross(m.cy, m.cz))
}

/// Multiply a matrix times a column vector.
pub fn mul_mv(m: Matrix3, a: Vec3) -> Vec3 {
    Vec3 {
        x: m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
        y: m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
        z: m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z,
    }
}

/// Negate a matrix.
pub fn negate_mat3(a: Matrix3) -> Matrix3 {
    Matrix3 {
        cx: Vec3 {
            x: -a.cx.x,
            y: -a.cx.y,
            z: -a.cx.z,
        },
        cy: Vec3 {
            x: -a.cy.x,
            y: -a.cy.y,
            z: -a.cy.z,
        },
        cz: Vec3 {
            x: -a.cz.x,
            y: -a.cz.y,
            z: -a.cz.z,
        },
    }
}

/// Matrix addition.
/// @return a + b
pub fn add_mm(a: Matrix3, b: Matrix3) -> Matrix3 {
    Matrix3 {
        cx: Vec3 {
            x: a.cx.x + b.cx.x,
            y: a.cx.y + b.cx.y,
            z: a.cx.z + b.cx.z,
        },
        cy: Vec3 {
            x: a.cy.x + b.cy.x,
            y: a.cy.y + b.cy.y,
            z: a.cy.z + b.cy.z,
        },
        cz: Vec3 {
            x: a.cz.x + b.cz.x,
            y: a.cz.y + b.cz.y,
            z: a.cz.z + b.cz.z,
        },
    }
}

/// Matrix subtraction.
/// @return a - b
pub fn sub_mm(a: Matrix3, b: Matrix3) -> Matrix3 {
    Matrix3 {
        cx: Vec3 {
            x: a.cx.x - b.cx.x,
            y: a.cx.y - b.cx.y,
            z: a.cx.z - b.cx.z,
        },
        cy: Vec3 {
            x: a.cy.x - b.cy.x,
            y: a.cy.y - b.cy.y,
            z: a.cy.z - b.cy.z,
        },
        cz: Vec3 {
            x: a.cz.x - b.cz.x,
            y: a.cz.y - b.cz.y,
            z: a.cz.z - b.cz.z,
        },
    }
}

/// Multiply a matrix by a scalar, component-wise.
pub fn mul_sm(s: f32, a: Matrix3) -> Matrix3 {
    Matrix3 {
        cx: Vec3 {
            x: s * a.cx.x,
            y: s * a.cx.y,
            z: s * a.cx.z,
        },
        cy: Vec3 {
            x: s * a.cy.x,
            y: s * a.cy.y,
            z: s * a.cy.z,
        },
        cz: Vec3 {
            x: s * a.cz.x,
            y: s * a.cz.y,
            z: s * a.cz.z,
        },
    }
}

/// Matrix multiplication.
/// @return a * b
pub fn mul_mm(a: Matrix3, b: Matrix3) -> Matrix3 {
    Matrix3 {
        cx: mul_mv(a, b.cx),
        cy: mul_mv(a, b.cy),
        cz: mul_mv(a, b.cz),
    }
}

/// Matrix transpose.
pub fn transpose(m: Matrix3) -> Matrix3 {
    Matrix3 {
        cx: Vec3 {
            x: m.cx.x,
            y: m.cy.x,
            z: m.cz.x,
        },
        cy: Vec3 {
            x: m.cx.y,
            y: m.cy.y,
            z: m.cz.y,
        },
        cz: Vec3 {
            x: m.cx.z,
            y: m.cy.z,
            z: m.cz.z,
        },
    }
}

/// General matrix inverse.
pub fn invert_matrix(m: Matrix3) -> Matrix3 {
    let det = det(m);
    if abs_float(det) > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / det;
        let out = Matrix3 {
            cx: mul_sv(inv_det, cross(m.cy, m.cz)),
            cy: mul_sv(inv_det, cross(m.cz, m.cx)),
            cz: mul_sv(inv_det, cross(m.cx, m.cy)),
        };

        transpose(out)
    } else {
        MAT3_ZERO
    }
}

/// Solve a matrix equation.
/// @return inv(m) * a
pub fn solve3(m: Matrix3, a: Vec3) -> Vec3 {
    let det = det(m);
    if abs_float(det) > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / det;
        let s = Matrix3 {
            cx: cross(m.cy, m.cz),
            cy: cross(m.cz, m.cx),
            cz: cross(m.cx, m.cy),
        };

        Vec3 {
            x: inv_det * dot(s.cx, a),
            y: inv_det * dot(s.cy, a),
            z: inv_det * dot(s.cz, a),
        }
    } else {
        VEC3_ZERO
    }
}

/// Invert a matrix (returns the adjugate / det without the final transpose of
/// [`invert_matrix`]).
pub fn invert_t(m: Matrix3) -> Matrix3 {
    let det = det(m);
    if abs_float(det) > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / det;
        Matrix3 {
            cx: mul_sv(inv_det, cross(m.cy, m.cz)),
            cy: mul_sv(inv_det, cross(m.cz, m.cx)),
            cz: mul_sv(inv_det, cross(m.cx, m.cy)),
        }
    } else {
        MAT3_ZERO
    }
}

/// Get the component-wise absolute value of a matrix.
pub fn abs_matrix3(m: Matrix3) -> Matrix3 {
    Matrix3 {
        cx: abs(m.cx),
        cy: abs(m.cy),
        cz: abs(m.cz),
    }
}

/// Make a matrix from a quaternion. This is useful if you need to
/// rotate many vectors.
pub fn make_matrix_from_quat(q: Quat) -> Matrix3 {
    let xx = q.v.x * q.v.x;
    let yy = q.v.y * q.v.y;
    let zz = q.v.z * q.v.z;
    let xy = q.v.x * q.v.y;
    let xz = q.v.x * q.v.z;
    let xw = q.v.x * q.s;
    let yz = q.v.y * q.v.z;
    let yw = q.v.y * q.s;
    let zw = q.v.z * q.s;

    Matrix3 {
        cx: Vec3 {
            x: 1.0 - 2.0 * (yy + zz),
            y: 2.0 * (xy + zw),
            z: 2.0 * (xz - yw),
        },
        cy: Vec3 {
            x: 2.0 * (xy - zw),
            y: 1.0 - 2.0 * (xx + zz),
            z: 2.0 * (yz + xw),
        },
        cz: Vec3 {
            x: 2.0 * (xz + yw),
            y: 2.0 * (yz - xw),
            z: 1.0 - 2.0 * (xx + yy),
        },
    }
}

/// Get the inertia tensor of an offset point.
/// <https://en.wikipedia.org/wiki/Parallel_axis_theorem>
pub fn steiner(mass: f32, origin: Vec3) -> Matrix3 {
    // Usage: Io = Ic + Is and Ic = Io - Is
    let ixx = mass * (origin.y * origin.y + origin.z * origin.z);
    let iyy = mass * (origin.x * origin.x + origin.z * origin.z);
    let izz = mass * (origin.x * origin.x + origin.y * origin.y);
    let ixy = -mass * origin.x * origin.y;
    let ixz = -mass * origin.x * origin.z;
    let iyz = -mass * origin.y * origin.z;

    // Write
    let mut out = MAT3_ZERO;
    out.cx.x = ixx;
    out.cy.x = ixy;
    out.cz.x = ixz;
    out.cx.y = ixy;
    out.cy.y = iyy;
    out.cz.y = iyz;
    out.cx.z = ixz;
    out.cy.z = iyz;
    out.cz.z = izz;

    out
}

fn det2(m: Mat2) -> f32 {
    m.cx.x * m.cy.y - m.cx.y * m.cy.x
}

/// Multiply a 2-by-2 matrix times a 2D vector. (math_internal.h: b3MulMV2)
pub fn mul_mv2(m: Mat2, a: Vec2) -> Vec2 {
    Vec2 {
        x: m.cx.x * a.x + m.cy.x * a.y,
        y: m.cx.y * a.x + m.cy.y * a.y,
    }
}

/// Multiply two 2-by-2 matrices. (math_internal.h: b3MulMM2)
pub fn mul_mm2(m1: Mat2, m2: Mat2) -> Mat2 {
    Mat2 {
        cx: mul_mv2(m1, m2.cx),
        cy: mul_mv2(m1, m2.cy),
    }
}

/// Invert a 2-by-2 matrix. (math_internal.h: b3Invert2)
pub fn invert2(m: Mat2) -> Mat2 {
    let det = det2(m);
    if abs_float(det) > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / det;
        Mat2 {
            cx: Vec2 {
                x: inv_det * m.cy.y,
                y: -inv_det * m.cx.y,
            },
            cy: Vec2 {
                x: -inv_det * m.cy.x,
                y: inv_det * m.cx.x,
            },
        }
    } else {
        Mat2 {
            cx: Vec2 { x: 0.0, y: 0.0 },
            cy: Vec2 { x: 0.0, y: 0.0 },
        }
    }
}

/// Solve A * x = b for a 2-by-2 matrix. Assumes positive semi-definite.
/// (math_internal.h: b3Solve2)
pub fn solve2(m: Mat2, b: Vec2) -> Vec2 {
    let det = det2(m);
    if det > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / det;
        Vec2 {
            x: inv_det * m.cy.y * b.x - inv_det * m.cy.x * b.y,
            y: -inv_det * m.cx.y * b.x + inv_det * m.cx.x * b.y,
        }
    } else {
        Vec2 { x: 0.0, y: 0.0 }
    }
}
