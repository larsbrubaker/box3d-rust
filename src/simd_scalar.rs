//! Scalar (`B3_SIMD_NONE`) fallback for the wide-float helpers from `simd.h`.
//!
//! `b3FloatW` holds four `f32` lanes. Upstream selects an SSE2, Neon, or scalar
//! implementation at compile time; this port implements the scalar branch, which
//! is the behavioral reference (see CLAUDE.md — the port targets `B3_SIMD_NONE`).
//!
//! The logical helpers (`and_w`, `or_w`, comparisons) return `0.0`/`1.0` float
//! masks rather than bit-wise all-ones masks, exactly like the C scalar path.
//!
//! The wide vec3 type and its helpers (`Vec3W`, `dot_w`, ...) also live here, as they do in
//! upstream `simd.h`. The scalar separating-axis face phase (`manifold::separating_axis`),
//! the hull support vertex search (`hull::find_hull_support_vertex`), and the wide convex
//! contact kernels (`contact_solver::wide`) consume these helpers. The rest back the wide
//! (`b3_simdWidth > 1`) collide path from `convex_manifold.c`, which the port does not
//! target, and are retained as a complete port of the `simd.h` scalar API.
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
#[inline]
// Kept for a complete scalar simd.h API; the 0cfc475 twist friction clamps the delta impulse with
// min/max instead.
#[allow(dead_code)]
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
#[inline]
// Kept for a complete scalar simd.h API; only the untargeted wide collide path uses it.
#[allow(dead_code)]
pub fn equals_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x == b.x { 1.0 } else { 0.0 },
        y: if a.y == b.y { 1.0 } else { 0.0 },
        z: if a.z == b.z { 1.0 } else { 0.0 },
        w: if a.w == b.w { 1.0 } else { 0.0 },
    }
}

/// (b3AllZeroW)
#[inline]
pub fn all_zero_w(a: FloatW) -> bool {
    a.x == 0.0 && a.y == 0.0 && a.z == 0.0 && a.w == 0.0
}

/// (b3AnyTrueW)
#[inline]
pub fn any_true_w(mask: FloatW) -> bool {
    mask.x != 0.0 || mask.y != 0.0 || mask.z != 0.0 || mask.w != 0.0
}

/// One bit per lane set when the lane mask is true, lane 0 in bit 0. (b3MoveMaskW4)
#[inline]
pub fn move_mask_w(mask: FloatW) -> i32 {
    (if mask.x != 0.0 { 1 } else { 0 })
        | (if mask.y != 0.0 { 2 } else { 0 })
        | (if mask.z != 0.0 { 4 } else { 0 })
        | (if mask.w != 0.0 { 8 } else { 0 })
}

/// Component-wise `mask ? b : a`. (b3BlendW)
#[inline]
pub fn blend_w(a: FloatW, b: FloatW, mask: FloatW) -> FloatW {
    FloatW {
        x: if mask.x != 0.0 { b.x } else { a.x },
        y: if mask.y != 0.0 { b.y } else { a.y },
        z: if mask.z != 0.0 { b.z } else { a.z },
        w: if mask.w != 0.0 { b.w } else { a.w },
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

/// Component-wise absolute value. (b3AbsW, scalar path)
#[inline]
pub fn abs_w(a: FloatW) -> FloatW {
    FloatW {
        x: if a.x < 0.0 { -a.x } else { a.x },
        y: if a.y < 0.0 { -a.y } else { a.y },
        z: if a.z < 0.0 { -a.z } else { a.z },
        w: if a.w < 0.0 { -a.w } else { a.w },
    }
}

/// `a & ~b` as 0/1 masks. (b3AndNotW, scalar path)
#[inline]
// Kept for a complete scalar simd.h API; the 79fc559 contact kernels no longer use it.
#[allow(dead_code)]
pub fn and_not_w(a: FloatW, b: FloatW) -> FloatW {
    FloatW {
        x: if a.x != 0.0 && b.x == 0.0 { 1.0 } else { 0.0 },
        y: if a.y != 0.0 && b.y == 0.0 { 1.0 } else { 0.0 },
        z: if a.z != 0.0 && b.z == 0.0 { 1.0 } else { 0.0 },
        w: if a.w != 0.0 && b.w == 0.0 { 1.0 } else { 0.0 },
    }
}

/// Lanes where either base-1 body index is null (0). Used to select the contact softness.
/// (b3SoftMaskW, scalar path)
#[inline]
pub fn soft_mask_w(index_a: &[i32; 4], index_b: &[i32; 4]) -> FloatW {
    FloatW {
        x: if index_a[0] == 0 || index_b[0] == 0 {
            1.0
        } else {
            0.0
        },
        y: if index_a[1] == 0 || index_b[1] == 0 {
            1.0
        } else {
            0.0
        },
        z: if index_a[2] == 0 || index_b[2] == 0 {
            1.0
        } else {
            0.0
        },
        w: if index_a[3] == 0 || index_b[3] == 0 {
            1.0
        } else {
            0.0
        },
    }
}

/// Wide vec3. (b3Vec3W)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3W {
    pub x: FloatW,
    pub y: FloatW,
    pub z: FloatW,
}

/// (b3LoadVW)
#[inline]
pub fn load_vw(x: &[f32], y: &[f32], z: &[f32]) -> Vec3W {
    Vec3W {
        x: load_w(x),
        y: load_w(y),
        z: load_w(z),
    }
}

/// (b3StoreVW)
#[inline]
pub fn store_vw(x: &mut [f32], y: &mut [f32], z: &mut [f32], v: Vec3W) {
    store_w(x, v.x);
    store_w(y, v.y);
    store_w(z, v.z);
}

/// (b3SplatVW)
#[inline]
pub fn splat_vw(v: crate::math_functions::Vec3) -> Vec3W {
    Vec3W {
        x: splat_w(v.x),
        y: splat_w(v.y),
        z: splat_w(v.z),
    }
}

/// (b3NegVW)
#[inline]
pub fn neg_vw(a: Vec3W) -> Vec3W {
    Vec3W {
        x: neg_w(a.x),
        y: neg_w(a.y),
        z: neg_w(a.z),
    }
}

/// s * a (b3MulSVW)
#[inline]
pub fn mul_svw(s: FloatW, a: Vec3W) -> Vec3W {
    Vec3W {
        x: mul_w(s, a.x),
        y: mul_w(s, a.y),
        z: mul_w(s, a.z),
    }
}

/// a - s * b (b3MulSubSVW)
#[inline]
pub fn mul_sub_svw(a: Vec3W, s: FloatW, b: Vec3W) -> Vec3W {
    Vec3W {
        x: sub_w(a.x, mul_w(s, b.x)),
        y: sub_w(a.y, mul_w(s, b.y)),
        z: sub_w(a.z, mul_w(s, b.z)),
    }
}

/// a + s * b (b3MulAddSVW)
#[inline]
pub fn mul_add_svw(a: Vec3W, s: FloatW, b: Vec3W) -> Vec3W {
    Vec3W {
        x: add_w(a.x, mul_w(s, b.x)),
        y: add_w(a.y, mul_w(s, b.y)),
        z: add_w(a.z, mul_w(s, b.z)),
    }
}

/// a - b (b3SubVW)
#[inline]
pub fn sub_vw(a: Vec3W, b: Vec3W) -> Vec3W {
    Vec3W {
        x: sub_w(a.x, b.x),
        y: sub_w(a.y, b.y),
        z: sub_w(a.z, b.z),
    }
}

/// a + b (b3AddVW)
#[inline]
pub fn add_vw(a: Vec3W, b: Vec3W) -> Vec3W {
    Vec3W {
        x: add_w(a.x, b.x),
        y: add_w(a.y, b.y),
        z: add_w(a.z, b.z),
    }
}

/// `(a.x * b.x + a.y * b.y) + a.z * b.z` per lane. (b3DotW)
#[inline]
pub fn dot_w(a: Vec3W, b: Vec3W) -> FloatW {
    add_w(add_w(mul_w(a.x, b.x), mul_w(a.y, b.y)), mul_w(a.z, b.z))
}

/// (b3CrossW)
#[inline]
pub fn cross_w(a: Vec3W, b: Vec3W) -> Vec3W {
    Vec3W {
        x: sub_w(mul_w(a.y, b.z), mul_w(a.z, b.y)),
        y: sub_w(mul_w(a.z, b.x), mul_w(a.x, b.z)),
        z: sub_w(mul_w(a.x, b.y), mul_w(a.y, b.x)),
    }
}

// ---------------------------------------------------------------------------
// Runtime SIMD width selection (simd.c).
//
// Upstream picks the wide kernels at runtime: 8 lanes (AVX2) when the x86 CPU supports
// them, otherwise 4 lanes (SSE2 / Neon / scalar). `B3_SIMD_HAS_WIDTH_8` is never defined
// for the scalar `B3_SIMD_NONE` build the port mirrors, so detection always reports 4 and
// requests for width 8 fall back to 4, exactly like a `BOX3D_DISABLE_SIMD` C build.
// ---------------------------------------------------------------------------

use core::sync::atomic::{AtomicI32, Ordering};

// C keeps this in a b3AtomicInt; 0 means "not detected yet".
static SIMD_WIDTH_STATE: AtomicI32 = AtomicI32::new(0);

/// The SIMD width supported by this CPU and build. Always 4 for the scalar path.
/// (static b3DetectSIMDWidth)
pub(crate) fn detect_simd_width() -> i32 {
    4
}

/// The SIMD width new worlds use, detected on first use. (b3GetSIMDWidth)
pub fn get_simd_width() -> i32 {
    let mut width = SIMD_WIDTH_STATE.load(Ordering::Relaxed);
    if width == 0 {
        width = detect_simd_width();
        SIMD_WIDTH_STATE.store(width, Ordering::Relaxed);
    }

    width
}

/// Force the SIMD width used by worlds created afterwards. 0 restores detection; 8 falls
/// back to 4 when the build or CPU lacks AVX2 (always, for the scalar path). This is for
/// testing and benchmarking. (b3SetSIMDWidth)
#[cfg_attr(not(test), allow(dead_code))]
pub fn set_simd_width(mut width: i32) {
    debug_assert!(width == 0 || width == 4 || width == 8);
    if width == 8 && detect_simd_width() != 8 {
        width = 4;
    }

    SIMD_WIDTH_STATE.store(width, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn move_mask_sets_one_bit_per_true_lane() {
        assert_eq!(move_mask_w(zero_w()), 0);
        assert_eq!(move_mask_w(set_w(1.0, 0.0, 0.0, 0.0)), 1);
        assert_eq!(move_mask_w(set_w(0.0, 1.0, 0.0, 1.0)), 10);
        assert_eq!(
            move_mask_w(greater_than_w(set_w(1.0, 2.0, 3.0, 4.0), splat_w(2.5))),
            12
        );
    }
}
