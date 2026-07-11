// Math types, constants, and operator overloads.
// Part of the math_functions module (see mod.rs).

/// A 2D vector.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

/// A 3D vector.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Cosine and sine pair.
/// This uses a custom implementation designed for cross-platform determinism.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CosSin {
    /// cosine and sine
    pub cosine: f32,
    pub sine: f32,
}

/// A quaternion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    pub v: Vec3,
    pub s: f32,
}

/// A rigid transform.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub p: Vec3,
    pub q: Quat,
}

/// A world position. Double precision in large world mode so coordinates stay accurate far
/// from the origin.
#[cfg(feature = "double-precision")]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pos {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A world transform with double precision translation and float quaternion rotation. Rotation
/// is frame local and never needs the extra range, the same split as Jolt's DMat44.
#[cfg(feature = "double-precision")]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldTransform {
    pub p: Pos,
    pub q: Quat,
}

#[cfg(not(feature = "double-precision"))]
pub type Pos = Vec3;

#[cfg(not(feature = "double-precision"))]
pub type WorldTransform = Transform;

/// A 3x3 matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix3 {
    pub cx: Vec3,
    pub cy: Vec3,
    pub cz: Vec3,
}

/// Axis aligned bounding box.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Aabb {
    pub lower_bound: Vec3,
    pub upper_bound: Vec3,
}

/// A plane.
/// separation = dot(normal, point) - offset
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    pub normal: Vec3,
    pub offset: f32,
}

/// The closest points between two segments or infinite lines.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentDistanceResult {
    pub point1: Vec3,
    pub fraction1: f32,
    pub point2: Vec3,
    pub fraction2: f32,
}

/// <https://en.wikipedia.org/wiki/Pi>
/// The C `B3_PI` literal (3.14159265359f) rounds to exactly this f32 value.
pub const PI: f32 = core::f32::consts::PI;

/// Convenience constant to convert from degrees to radians. (B3_DEG_TO_RAD)
pub const DEG_TO_RAD: f32 = 0.01745329251;

/// Convenience constant to convert from radians to degrees. (B3_RAD_TO_DEG)
pub const RAD_TO_DEG: f32 = 57.2957795131;

/// Minimum scale used for scaling collision meshes, etc. (B3_MIN_SCALE)
pub const MIN_SCALE: f32 = 0.01;

pub const VEC3_ZERO: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};
pub const VEC3_ONE: Vec3 = Vec3 {
    x: 1.0,
    y: 1.0,
    z: 1.0,
};
pub const VEC3_AXIS_X: Vec3 = Vec3 {
    x: 1.0,
    y: 0.0,
    z: 0.0,
};
pub const VEC3_AXIS_Y: Vec3 = Vec3 {
    x: 0.0,
    y: 1.0,
    z: 0.0,
};
pub const VEC3_AXIS_Z: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 1.0,
};
pub const QUAT_IDENTITY: Quat = Quat {
    v: Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
    s: 1.0,
};
pub const TRANSFORM_IDENTITY: Transform = Transform {
    p: Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
    q: Quat {
        v: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        s: 1.0,
    },
};
pub const MAT3_ZERO: Matrix3 = Matrix3 {
    cx: Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
    cy: Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
    cz: Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
};
pub const MAT3_IDENTITY: Matrix3 = Matrix3 {
    cx: Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    },
    cy: Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    },
    cz: Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    },
};

// Valid in both modes: 0.0f promotes to double, the identity rotation stays float
#[cfg(feature = "double-precision")]
pub const POS_ZERO: Pos = Pos {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};
#[cfg(not(feature = "double-precision"))]
pub const POS_ZERO: Pos = VEC3_ZERO;

#[cfg(feature = "double-precision")]
pub const WORLD_TRANSFORM_IDENTITY: WorldTransform = WorldTransform {
    p: Pos {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
    q: Quat {
        v: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        s: 1.0,
    },
};
#[cfg(not(feature = "double-precision"))]
pub const WORLD_TRANSFORM_IDENTITY: WorldTransform = TRANSFORM_IDENTITY;

impl Vec2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Vec2 { x, y }
    }
}

impl Vec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Vec3 { x, y, z }
    }
}

impl Quat {
    pub const fn new(v: Vec3, s: f32) -> Self {
        Quat { v, s }
    }
}

impl Transform {
    pub const fn new(p: Vec3, q: Quat) -> Self {
        Transform { p, q }
    }
}

// Operator overloads mirroring the C++ operators in math_functions.h

impl core::ops::AddAssign for Vec3 {
    fn add_assign(&mut self, b: Vec3) {
        self.x += b.x;
        self.y += b.y;
        self.z += b.z;
    }
}

impl core::ops::SubAssign for Vec3 {
    fn sub_assign(&mut self, b: Vec3) {
        self.x -= b.x;
        self.y -= b.y;
        self.z -= b.z;
    }
}

impl core::ops::MulAssign<f32> for Vec3 {
    fn mul_assign(&mut self, b: f32) {
        self.x *= b;
        self.y *= b;
        self.z *= b;
    }
}

impl core::ops::Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl core::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, b: Vec3) -> Vec3 {
        Vec3 {
            x: self.x + b.x,
            y: self.y + b.y,
            z: self.z + b.z,
        }
    }
}

impl core::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, b: Vec3) -> Vec3 {
        Vec3 {
            x: self.x - b.x,
            y: self.y - b.y,
            z: self.z - b.z,
        }
    }
}

impl core::ops::Mul<Vec3> for f32 {
    type Output = Vec3;
    fn mul(self, b: Vec3) -> Vec3 {
        Vec3 {
            x: self * b.x,
            y: self * b.y,
            z: self * b.z,
        }
    }
}

impl core::ops::Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, b: f32) -> Vec3 {
        Vec3 {
            x: self.x * b,
            y: self.y * b,
            z: self.z * b,
        }
    }
}

impl core::ops::Mul for Vec3 {
    type Output = Vec3;
    fn mul(self, b: Vec3) -> Vec3 {
        Vec3 {
            x: self.x * b.x,
            y: self.y * b.y,
            z: self.z * b.z,
        }
    }
}
