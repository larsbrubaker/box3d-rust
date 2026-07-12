//! Port of `test/test_determinism.c`: falling-ragdoll sleep step and world
//! transform hash must match the C scalar (`BOX3D_DISABLE_SIMD`) reference.
//!
//! The Rust port is serial, so the C multithreading loop (workerCount 1..5)
//! collapses to a single worker-count=1 run — the same path `CrossPlatformTest`
//! exercises. Both precision modes have distinct golden values.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::determinism::{
    create_falling_ragdolls, destroy_falling_ragdolls, update_falling_ragdolls, FallingRagdollData,
};
use crate::types::default_world_def;
use crate::world::World;

// Double precision accumulates body positions in double, so the settle/sleep
// step and the state hash differ from the float build. Both modes are
// internally deterministic. Values from test_determinism.c.
#[cfg(feature = "double-precision")]
const EXPECTED_SLEEP_STEP: i32 = 301;
#[cfg(feature = "double-precision")]
const EXPECTED_HASH: u32 = 0xE484_4A97;

#[cfg(not(feature = "double-precision"))]
const EXPECTED_SLEEP_STEP: i32 = 269;
#[cfg(not(feature = "double-precision"))]
const EXPECTED_HASH: u32 = 0x5031_3037;

fn assert_expected(data: &FallingRagdollData, label: &str) {
    if data.sleep_step != EXPECTED_SLEEP_STEP || data.hash != EXPECTED_HASH {
        eprintln!(
            "  {label} sleepStep={} hash=0x{:08X} (expected sleepStep={} hash=0x{:08X})",
            data.sleep_step, data.hash, EXPECTED_SLEEP_STEP, EXPECTED_HASH
        );
    }
    assert_eq!(
        data.sleep_step, EXPECTED_SLEEP_STEP,
        "{label}: sleep_step mismatch"
    );
    assert_eq!(data.hash, EXPECTED_HASH, "{label}: hash mismatch");
}

/// Serial equivalent of C `SingleMultithreadingTest` / `MultithreadingTest`
/// at `workerCount == 1` (the only count the serial port supports).
fn single_multithreading_test() -> FallingRagdollData {
    let mut world = World::new(&default_world_def());
    let mut data = create_falling_ragdolls(&mut world);

    let time_step = 1.0 / 60.0;
    let step_limit = 500;
    for _ in 0..step_limit {
        world.step(time_step, 4);
        if update_falling_ragdolls(&world, &mut data) {
            break;
        }
    }

    destroy_falling_ragdolls(&mut data);
    data
}

/// Cross-platform determinism gate from C `CrossPlatformTest`.
fn cross_platform_run() -> FallingRagdollData {
    let mut world = World::new(&default_world_def());
    let mut data = create_falling_ragdolls(&mut world);

    let time_step = 1.0 / 60.0;
    loop {
        world.step(time_step, 4);
        if update_falling_ragdolls(&world, &mut data) {
            break;
        }
    }

    destroy_falling_ragdolls(&mut data);
    data
}

#[test]
fn multithreading_test() {
    let data = single_multithreading_test();
    assert_expected(&data, "workers=1");
}

#[test]
fn cross_platform_test() {
    let data = cross_platform_run();
    assert_expected(&data, "cross-platform");
}
