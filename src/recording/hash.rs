//! World state hash for recording verification. (b3HashWorldState)
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{get_body_sim, get_body_state_index};
use crate::math_functions::Pos;
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// FNV-1a 64-bit constants. (B3_SNAP_FNV_*)
pub const SNAP_FNV_INIT: u64 = 14695981039346656037;
pub const SNAP_FNV_PRIME: u64 = 1099511628211;

/// Mix a world position at full width. (b3FnvMixPosition)
pub fn fnv_mix_position(mut hash: u64, p: Pos) -> u64 {
    #[cfg(feature = "double-precision")]
    {
        hash = (hash ^ p.x.to_bits()).wrapping_mul(SNAP_FNV_PRIME);
        hash = (hash ^ p.y.to_bits()).wrapping_mul(SNAP_FNV_PRIME);
        hash = (hash ^ p.z.to_bits()).wrapping_mul(SNAP_FNV_PRIME);
    }
    #[cfg(not(feature = "double-precision"))]
    {
        hash = (hash ^ p.x.to_bits() as u64).wrapping_mul(SNAP_FNV_PRIME);
        hash = (hash ^ p.y.to_bits() as u64).wrapping_mul(SNAP_FNV_PRIME);
        hash = (hash ^ p.z.to_bits() as u64).wrapping_mul(SNAP_FNV_PRIME);
    }
    hash
}

/// Hash live body transforms and awake velocities. (b3HashWorldState)
pub fn hash_world_state(world: &World) -> u64 {
    let mut hash = SNAP_FNV_INIT;
    let prime = SNAP_FNV_PRIME;

    for (i, body) in world.bodies.iter().enumerate() {
        if body.id != i as i32 {
            continue;
        }

        let sim = get_body_sim(world, i as i32);

        hash = fnv_mix_position(hash, sim.transform.p);

        macro_rules! hash_f32 {
            ($f:expr) => {{
                hash = (hash ^ ($f.to_bits() as u64)).wrapping_mul(prime);
            }};
        }

        hash_f32!(sim.transform.q.v.x);
        hash_f32!(sim.transform.q.v.y);
        hash_f32!(sim.transform.q.v.z);
        hash_f32!(sim.transform.q.s);

        if let Some(local_index) = get_body_state_index(world, i as i32) {
            let state = &world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
            hash_f32!(state.linear_velocity.x);
            hash_f32!(state.linear_velocity.y);
            hash_f32!(state.linear_velocity.z);
            hash_f32!(state.angular_velocity.x);
            hash_f32!(state.angular_velocity.y);
            hash_f32!(state.angular_velocity.z);
        }
    }

    hash
}
