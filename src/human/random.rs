//! Deterministic PRNG from `shared/utils.h`, used by the human ragdoll helpers.
//!
//! SPDX-FileCopyrightText: 2023 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::math_functions::{compute_cos_sin, to_pos, Pos, Quat, Vec3, PI};
use std::cell::Cell;

const RAND_LIMIT: u32 = 32767;
const RAND_SEED: u32 = 12345;

thread_local! {
    static RANDOM_SEED: Cell<u32> = const { Cell::new(RAND_SEED) };
}

/// Reset the global random seed. Mirrors writing `g_randomSeed` in the C shared code.
pub fn set_random_seed(seed: u32) {
    RANDOM_SEED.with(|s| s.set(seed));
}

/// XorShift32 integer in `[0, RAND_LIMIT]`. (RandomInt)
pub fn random_int() -> i32 {
    RANDOM_SEED.with(|seed| {
        let mut x = seed.get();
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        seed.set(x);
        (x % (RAND_LIMIT + 1)) as i32
    })
}

/// Random number in range `[-1, 1]`. (RandomFloat)
pub fn random_float() -> f32 {
    let r = (random_int() as u32 & RAND_LIMIT) as f32;
    let r = r / RAND_LIMIT as f32;
    2.0 * r - 1.0
}

/// Random float in `[lo, hi]`. (RandomFloatRange)
pub fn random_float_range(lo: f32, hi: f32) -> f32 {
    let r = (random_int() as u32 & RAND_LIMIT) as f32;
    let r = r / RAND_LIMIT as f32;
    (hi - lo) * r + lo
}

/// Random vector with coordinates in `[lo, hi]` per axis. (RandomVec3)
pub fn random_vec3(lo: Vec3, hi: Vec3) -> Vec3 {
    Vec3 {
        x: random_float_range(lo.x, hi.x),
        y: random_float_range(lo.y, hi.y),
        z: random_float_range(lo.z, hi.z),
    }
}

/// Random world position with coordinates in range `[lo, hi]`. (RandomPos)
pub fn random_pos(lo: Vec3, hi: Vec3) -> Pos {
    // C fills a b3Pos component-wise from RandomFloatRange in x, y, z order; the
    // float result is promoted to the position scalar. to_pos preserves that order.
    to_pos(Vec3 {
        x: random_float_range(lo.x, hi.x),
        y: random_float_range(lo.y, hi.y),
        z: random_float_range(lo.z, hi.z),
    })
}

/// Random vector with all coordinates in `[lo, hi]`. (RandomVec3Uniform)
pub fn random_vec3_uniform(lo: f32, hi: f32) -> Vec3 {
    Vec3 {
        x: random_float_range(lo, hi),
        y: random_float_range(lo, hi),
        z: random_float_range(lo, hi),
    }
}

/// Uniformly distributed random unit vector using Shoemake's method.
/// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992.
/// (RandomUnitVector)
pub fn random_unit_vector() -> Vec3 {
    let u1 = random_float_range(0.0, 1.0);
    let u2 = random_float_range(0.0, 2.0 * PI);
    let u3 = random_float_range(0.0, 2.0 * PI);

    let sqrt1_minus_u1 = (1.0 - u1).sqrt();
    let sqrt_u1 = u1.sqrt();

    let cs2 = compute_cos_sin(u2);
    let cs3 = compute_cos_sin(u3);

    Vec3 {
        x: sqrt1_minus_u1 * cs2.sine,
        y: sqrt1_minus_u1 * cs2.cosine,
        z: sqrt_u1 * cs3.sine,
    }
}

/// Uniformly distributed random quaternion using Shoemake's method. (RandomQuat)
pub fn random_quat() -> Quat {
    let u1 = random_float_range(0.0, 1.0);
    let u2 = random_float_range(0.0, 2.0 * PI);
    let u3 = random_float_range(0.0, 2.0 * PI);

    let sqrt1_minus_u1 = (1.0 - u1).sqrt();
    let sqrt_u1 = u1.sqrt();

    let cs2 = compute_cos_sin(u2);
    let cs3 = compute_cos_sin(u3);

    Quat {
        v: Vec3 {
            x: sqrt1_minus_u1 * cs2.sine,
            y: sqrt1_minus_u1 * cs2.cosine,
            z: sqrt_u1 * cs3.sine,
        },
        s: sqrt_u1 * cs3.cosine,
    }
}
