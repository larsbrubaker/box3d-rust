//! Scalar (`B3_SIMD_NONE`) fallback for the wide-float helpers from `simd.h`.
//!
//! `b3FloatW` holds four `f32` lanes. Upstream selects an SSE2, Neon, or scalar
//! implementation at compile time; this port implements the scalar branch, which
//! is the behavioral reference (see CLAUDE.md — the port targets `B3_SIMD_NONE`).
//!
//! The logical helpers (`and_w`, `or_w`, comparisons) return `0.0`/`1.0` float
//! masks rather than bit-wise all-ones masks, exactly like the C scalar path.
//!
//! The scalar separating-axis face phase ([`super::separating_axis`]) consumes the
//! arithmetic and reduction helpers. The comparison, select, and fused helpers
//! (`and_w`, `or_w`, `blend_w`, `mul_add_w`, `sqrt_w`, `div_w`, `max_w`, `sym_clamp_w`,
//! `set_w`, `all_zero_w`, `any_true_w`, `equals_w`, `greater_than_w`, `less_than_w`)
//! back the wide (`b3_simdWidth > 1`) collide path from `convex_manifold.c`, which the
//! port does not target — the scalar path is the behavioral reference — so they are
//! retained here as a complete port of the `simd.h` scalar API but currently unused.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

/// Wide float holding four lanes. (`b3FloatW`, scalar path)
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct FloatW {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

/// (b3ZeroW)
#[inline]
pub fn zero_w() -> FloatW {
    FloatW {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 0.0,
    }
}

/// (b3SplatW)
#[inline]
pub fn splat_w(scalar: f32) -> FloatW {
    FloatW {
        x: scalar,
        y: scalar,
        z: scalar,
        w: scalar,
    }
}

/// (b3SetW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn set_w(a: f32, b: f32, c: f32, d: f32) -> FloatW {
    FloatW {
        x: a,
        y: b,
        z: c,
        w: d,
    }
}

/// (b3LoadW)
#[inline]
pub fn load_w(data: &[f32]) -> FloatW {
    FloatW {
        x: data[0],
        y: data[1],
        z: data[2],
        w: data[3],
    }
}

/// (b3StoreW)
#[inline]
pub fn store_w(data: &mut [f32], a: FloatW) {
    data[0] = a.x;
    data[1] = a.y;
    data[2] = a.z;
    data[3] = a.w;
}

/// (b3NegW)
#[inline]
pub fn neg_w(a: FloatW) -> FloatW {
    FloatW {
        x: -a.x,
        y: -a.y,
        z: -a.z,
        w: -a.w,
    }
}

/// (b3AddW)
#[inline]
pub fn add_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: a.x + b.x,
        y: a.y + b.y,
        z: a.z + b.z,
        w: a.w + b.w,
    }
}

/// (b3SubW)
#[inline]
pub fn sub_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
        w: a.w - b.w,
    }
}

/// (b3MulW)
#[inline]
pub fn mul_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: a.x * b.x,
        y: a.y * b.y,
        z: a.z * b.z,
        w: a.w * b.w,
    }
}

/// (b3DivW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn div_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: a.x / b.x,
        y: a.y / b.y,
        z: a.z / b.z,
        w: a.w / b.w,
    }
}

/// (b3SqrtW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn sqrt_w(a: FloatW) -> FloatW {
    FloatW {
        x: a.x.sqrt(),
        y: a.y.sqrt(),
        z: a.z.sqrt(),
        w: a.w.sqrt(),
    }
}

/// `a + b * c`. Cannot use a real FMA because it wouldn't match the non-SIMD path.
/// (b3MulAddW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn mul_add_w(a: FloatW, b: FloatW, c: FloatW) -> FloatW {
    FloatW {
        x: a.x + b.x * c.x,
        y: a.y + b.y * c.y,
        z: a.z + b.z * c.z,
        w: a.w + b.w * c.w,
    }
}

/// (b3MinW)
#[inline]
pub fn min_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x <= b.x { a.x } else { b.x },
        y: if a.y <= b.y { a.y } else { b.y },
        z: if a.z <= b.z { a.z } else { b.z },
        w: if a.w <= b.w { a.w } else { b.w },
    }
}

/// (b3MaxW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn max_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x >= b.x { a.x } else { b.x },
        y: if a.y >= b.y { a.y } else { b.y },
        z: if a.z >= b.z { a.z } else { b.z },
        w: if a.w >= b.w { a.w } else { b.w },
    }
}

/// Clamp `a` to `[-b, b]`. (b3SymClampW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn sym_clamp_w(a: FloatW, b: FloatW) -> FloatW {
    let mut r = FloatW {
        x: if a.x <= b.x { a.x } else { b.x },
        y: if a.y <= b.y { a.y } else { b.y },
        z: if a.z <= b.z { a.z } else { b.z },
        w: if a.w <= b.w { a.w } else { b.w },
    };
    r.x = if r.x <= -b.x { -b.x } else { r.x };
    r.y = if r.y <= -b.y { -b.y } else { r.y };
    r.z = if r.z <= -b.z { -b.z } else { r.z };
    r.w = if r.w <= -b.w { -b.w } else { r.w };
    r
}

// Logical operations on the scalar path are 0/1 float values. Not bit-wise like SIMD.

/// (b3AndW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn and_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x != 0.0 && b.x != 0.0 { 1.0 } else { 0.0 },
        y: if a.y != 0.0 && b.y != 0.0 { 1.0 } else { 0.0 },
        z: if a.z != 0.0 && b.z != 0.0 { 1.0 } else { 0.0 },
        w: if a.w != 0.0 && b.w != 0.0 { 1.0 } else { 0.0 },
    }
}

/// (b3OrW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn or_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x != 0.0 || b.x != 0.0 { 1.0 } else { 0.0 },
        y: if a.y != 0.0 || b.y != 0.0 { 1.0 } else { 0.0 },
        z: if a.z != 0.0 || b.z != 0.0 { 1.0 } else { 0.0 },
        w: if a.w != 0.0 || b.w != 0.0 { 1.0 } else { 0.0 },
    }
}

/// (b3GreaterThanW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn greater_than_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x > b.x { 1.0 } else { 0.0 },
        y: if a.y > b.y { 1.0 } else { 0.0 },
        z: if a.z > b.z { 1.0 } else { 0.0 },
        w: if a.w > b.w { 1.0 } else { 0.0 },
    }
}

/// (b3LessThanW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn less_than_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x < b.x { 1.0 } else { 0.0 },
        y: if a.y < b.y { 1.0 } else { 0.0 },
        z: if a.z < b.z { 1.0 } else { 0.0 },
        w: if a.w < b.w { 1.0 } else { 0.0 },
    }
}

/// (b3EqualsW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn equals_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x == b.x { 1.0 } else { 0.0 },
        y: if a.y == b.y { 1.0 } else { 0.0 },
        z: if a.z == b.z { 1.0 } else { 0.0 },
        w: if a.w == b.w { 1.0 } else { 0.0 },
    }
}

/// (b3AllZeroW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn all_zero_w(a: FloatW) -> bool {
    a.x == 0.0 && a.y == 0.0 && a.z == 0.0 && a.w == 0.0
}

/// (b3AnyTrueW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn any_true_w(mask: FloatW) -> bool {
    mask.x != 0.0 || mask.y != 0.0 || mask.z != 0.0 || mask.w != 0.0
}

/// Component-wise `mask ? b : a`. (b3BlendW)
// SIMD-width (`b3_simdWidth > 1`) collide path, not yet ported.
#[allow(dead_code)]
#[inline]
pub fn blend_w(a: FloatW, b: FloatW, mask: FloatW) -> FloatW {
    FloatW {
        x: if mask.x != 0.0 { b.x } else { a.x },
        y: if mask.y != 0.0 { b.y } else { a.y },
        z: if mask.z != 0.0 { b.z } else { a.z },
        w: if mask.w != 0.0 { b.w } else { a.w },
    }
}

/// Component-wise 3D dot product across the four lanes. (b3Dot3W)
#[inline]
pub fn dot3_w(ax: FloatW, ay: FloatW, az: FloatW, bx: FloatW, by: FloatW, bz: FloatW) -> FloatW {
    FloatW {
        x: ax.x * bx.x + (ay.x * by.x + az.x * bz.x),
        y: ax.y * bx.y + (ay.y * by.y + az.y * bz.y),
        z: ax.z * bx.z + (ay.z * by.z + az.z * bz.z),
        w: ax.w * bx.w + (ay.w * by.w + az.w * bz.w),
    }
}

/// Replace the low `bit_count` mantissa bits of each lane with `base_index + lane`.
/// The value must be positive so the embedded index sorts with the value, and ties
/// fall to the lower index. (b3EmbedIndexW)
#[inline]
pub fn embed_index_w(value: FloatW, base_index: i32, bit_count: i32) -> FloatW {
    let mask: u32 = (1u32 << bit_count) - 1;
    let lanes = [value.x, value.y, value.z, value.w];
    let mut out = [0.0f32; 4];
    for i in 0..4 {
        let mut bits = lanes[i].to_bits();
        bits = (bits & !mask) | (base_index.wrapping_add(i as i32) as u32);
        out[i] = f32::from_bits(bits);
    }
    FloatW {
        x: out[0],
        y: out[1],
        z: out[2],
        w: out[3],
    }
}

/// Recover the index embedded by [`embed_index_w`] from the lane holding the minimum.
/// (b3MinIndexW)
#[inline]
pub fn min_index_w(a: FloatW, bit_count: i32) -> i32 {
    let mut m = a.x;
    m = if a.y < m { a.y } else { m };
    m = if a.z < m { a.z } else { m };
    m = if a.w < m { a.w } else { m };

    let bits = m.to_bits();
    (bits & ((1u32 << bit_count) - 1)) as i32
}
