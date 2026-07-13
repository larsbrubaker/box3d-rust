//! Shared XorShift32 RNG for the demo samples — a bit-for-bit port of the C
//! samples' `RandomInt` / `RandomFloatRange` (`shared/utils.h`). This is
//! determinism-critical: every sample that spawns randomized piles (Wind gusts,
//! Far Mesh Drop velocities, Query ray fans, Gear Lift colors, Sensor rows) must
//! evolve the identical stream the C sample does, so there is exactly one copy of
//! the arithmetic here and every caller points at it.
//!
//! Two usage shapes share this one implementation:
//! - value-carrying [`XorShift32`] (Sensor, Shapes) — the seed lives in the struct;
//! - a thread-local `Cell<u32>` seed (Far, Query, Gear) — those modules reconstruct
//!   a [`XorShift32`] from the cell, draw, then store [`XorShift32::seed`] back,
//!   which is bit-identical to advancing the raw `u32` in place.

#![allow(dead_code)] // Not every accessor is used by every caller.

use box3d_rust::math_functions::Vec3;

/// C `RAND_LIMIT` (`shared/utils.h`): the modulus mask for the 15-bit output.
pub const RAND_LIMIT: u32 = 32767;

/// Value-carrying XorShift32 generator (C `g_randomSeed` stream).
pub struct XorShift32(u32);

impl XorShift32 {
    /// Seed the generator (C sets `g_randomSeed = 12345` on Sample construction).
    pub const fn with_seed(seed: u32) -> Self {
        Self(seed)
    }

    /// Current internal seed, for callers that persist the stream across rebuilds.
    pub fn seed(&self) -> u32 {
        self.0
    }

    /// C `RandomInt()` (`shared/utils.h`): advance the state and fold to `[0, RAND_LIMIT]`.
    pub fn next_int(&mut self) -> i32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        (x % (RAND_LIMIT + 1)) as i32
    }

    /// C `RandomFloatRange(lo, hi)` (`shared/utils.h`).
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let r = (self.next_int() as u32 & RAND_LIMIT) as f32 / RAND_LIMIT as f32;
        (hi - lo) * r + lo
    }

    /// C `RandomIntRange(lo, hi)` (`shared/utils.h`).
    pub fn range_int(&mut self, lo: i32, hi: i32) -> i32 {
        lo + self.next_int() % (hi - lo + 1)
    }

    /// Component-wise `range` over a `Vec3` box (Shapes' Wind gust noise).
    pub fn vec3(&mut self, lo: Vec3, hi: Vec3) -> Vec3 {
        Vec3 {
            x: self.range(lo.x, hi.x),
            y: self.range(lo.y, hi.y),
            z: self.range(lo.z, hi.z),
        }
    }
}
