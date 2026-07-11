//! Deterministic PRNG from `shared/utils.h`, used by the human ragdoll helpers.
//!
//! SPDX-FileCopyrightText: 2023 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::math_functions::Vec3;
use std::cell::Cell;

const RAND_LIMIT: u32 = 32767;
const RAND_SEED: u32 = 12345;

thread_local! {
    static RANDOM_SEED: Cell<u32> = const { Cell::new(RAND_SEED) };
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
