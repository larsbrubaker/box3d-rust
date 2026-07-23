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
    create_falling_ragdolls, create_mesh_drop, create_query_spawn, create_wave_pile,
    destroy_falling_ragdolls, destroy_mesh_drop, destroy_query_spawn, destroy_wave_pile,
    update_falling_ragdolls, update_mesh_drop, update_query_spawn, update_wave_pile,
    FallingRagdollData, MeshDropData, QuerySpawnData, WavePileData, QUERY_SPAWN_COUNT,
};
use crate::math_functions::POS_ZERO;
use crate::types::default_world_def;
use crate::world::World;

// Double precision accumulates body positions in double, so the settle/sleep
// step and the state hash differ from the float build. Both modes are
// internally deterministic. Values from test_determinism.c.
#[cfg(feature = "double-precision")]
const EXPECTED_SLEEP_STEP: i32 = 297;
#[cfg(feature = "double-precision")]
const EXPECTED_HASH: u32 = 0x27FF_38C1;
#[cfg(feature = "double-precision")]
const WAVE_PILE_SLEEP_STEP: i32 = 287;
#[cfg(feature = "double-precision")]
const WAVE_PILE_HASH: u32 = 0xFFC8_DA49;
#[cfg(feature = "double-precision")]
const QUERY_SPAWN_SLEEP_STEP: i32 = 242;
#[cfg(feature = "double-precision")]
const QUERY_SPAWN_HASH: u32 = 0x1737_F5BC;
#[cfg(feature = "double-precision")]
const QUERY_SPAWN_HIT_COUNT: i32 = 59;
#[cfg(feature = "double-precision")]
const QUERY_SPAWN_QUERY_HASH: u32 = 0x31F0_90DC;
#[cfg(feature = "double-precision")]
const MESH_DROP_SLEEP_STEP: i32 = 206;
#[cfg(feature = "double-precision")]
const MESH_DROP_HASH: u32 = 0xB6A9_E1DE;

#[cfg(not(feature = "double-precision"))]
const EXPECTED_SLEEP_STEP: i32 = 308;
#[cfg(not(feature = "double-precision"))]
const EXPECTED_HASH: u32 = 0x1E5E_DD79;
#[cfg(not(feature = "double-precision"))]
const WAVE_PILE_SLEEP_STEP: i32 = 285;
#[cfg(not(feature = "double-precision"))]
const WAVE_PILE_HASH: u32 = 0x4057_173C;
#[cfg(not(feature = "double-precision"))]
const QUERY_SPAWN_SLEEP_STEP: i32 = 242;
#[cfg(not(feature = "double-precision"))]
const QUERY_SPAWN_HASH: u32 = 0xB9F9_93A5;
#[cfg(not(feature = "double-precision"))]
const QUERY_SPAWN_HIT_COUNT: i32 = 59;
#[cfg(not(feature = "double-precision"))]
const QUERY_SPAWN_QUERY_HASH: u32 = 0xB9D3_863D;
#[cfg(not(feature = "double-precision"))]
const MESH_DROP_SLEEP_STEP: i32 = 205;
#[cfg(not(feature = "double-precision"))]
const MESH_DROP_HASH: u32 = 0x8F55_FB2D;

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

/// Serial equivalent of C `SingleWavePileTest` at `workerCount == 1`.
fn single_wave_pile_test() -> WavePileData {
    let mut world = World::new(&default_world_def());
    let mut data = create_wave_pile(&mut world);

    let time_step = 1.0 / 60.0;

    // Rolling resistance must put the pile to sleep within 500 steps.
    let mut done = false;
    for _ in 0..500 {
        if done {
            break;
        }
        world.step(time_step, 4);
        done = update_wave_pile(&world, &mut data);
    }

    destroy_wave_pile(&mut data);
    assert!(done, "wave pile did not settle within 500 steps");
    data
}

#[test]
fn wave_pile_test() {
    let data = single_wave_pile_test();
    if data.sleep_step != WAVE_PILE_SLEEP_STEP || data.hash != WAVE_PILE_HASH {
        eprintln!(
            "  wave pile sleepStep={} hash=0x{:08X} (expected sleepStep={} hash=0x{:08X})",
            data.sleep_step, data.hash, WAVE_PILE_SLEEP_STEP, WAVE_PILE_HASH
        );
    }
    assert_eq!(data.sleep_step, WAVE_PILE_SLEEP_STEP, "wave pile sleep step");
    assert_eq!(data.hash, WAVE_PILE_HASH, "wave pile hash");
}

/// Serial equivalent of C `SingleQuerySpawnTest` at `workerCount == 1`.
fn single_query_spawn_test() -> QuerySpawnData {
    let mut world = World::new(&default_world_def());
    let mut data = create_query_spawn(&mut world);

    let time_step = 1.0 / 60.0;

    let mut done = false;
    for _ in 0..1000 {
        if done {
            break;
        }
        world.step(time_step, 4);
        done = update_query_spawn(&mut world, &mut data);
    }

    destroy_query_spawn(&mut data);
    assert!(done, "query spawn did not settle within 1000 steps");
    data
}

#[test]
fn query_spawn_test() {
    let data = single_query_spawn_test();
    if data.sleep_step != QUERY_SPAWN_SLEEP_STEP
        || data.hash != QUERY_SPAWN_HASH
        || data.query_hit_count != QUERY_SPAWN_HIT_COUNT
        || data.query_hash != QUERY_SPAWN_QUERY_HASH
    {
        eprintln!(
            "  query spawn sleepStep={} hash=0x{:08X} hits={} queryHash=0x{:08X}\n  \
             expected  sleepStep={} hash=0x{:08X} hits={} queryHash=0x{:08X}",
            data.sleep_step,
            data.hash,
            data.query_hit_count,
            data.query_hash,
            QUERY_SPAWN_SLEEP_STEP,
            QUERY_SPAWN_HASH,
            QUERY_SPAWN_HIT_COUNT,
            QUERY_SPAWN_QUERY_HASH
        );
    }
    assert_eq!(
        data.spawn_count as usize, QUERY_SPAWN_COUNT,
        "query spawn count"
    );
    assert_eq!(
        data.sleep_step, QUERY_SPAWN_SLEEP_STEP,
        "query spawn sleep step"
    );
    assert_eq!(data.hash, QUERY_SPAWN_HASH, "query spawn hash");
    assert_eq!(
        data.query_hit_count, QUERY_SPAWN_HIT_COUNT,
        "query spawn hit count"
    );
    assert_eq!(
        data.query_hash, QUERY_SPAWN_QUERY_HASH,
        "query spawn query hash"
    );
}

/// Serial equivalent of C `SingleMeshDropTest` at `workerCount == 1`. C runs worker
/// counts {1, 4}; the serial port supports only 1.
fn single_mesh_drop_test() -> MeshDropData {
    let mut world = World::new(&default_world_def());
    let mut data = create_mesh_drop(&mut world, POS_ZERO);

    let time_step = 1.0 / 60.0;

    let mut done = false;
    for _ in 0..400 {
        if done {
            break;
        }
        world.step(time_step, 4);
        done = update_mesh_drop(&world, &mut data);
    }

    destroy_mesh_drop(&mut data);
    assert!(done, "mesh drop did not settle within 400 steps");
    data
}

#[test]
fn mesh_drop_test() {
    let data = single_mesh_drop_test();
    if data.sleep_step != MESH_DROP_SLEEP_STEP || data.hash != MESH_DROP_HASH {
        eprintln!(
            "  mesh drop sleepStep={} hash=0x{:08X} (expected sleepStep={} hash=0x{:08X})",
            data.sleep_step, data.hash, MESH_DROP_SLEEP_STEP, MESH_DROP_HASH
        );
    }
    assert_eq!(data.sleep_step, MESH_DROP_SLEEP_STEP, "mesh drop sleep step");
    assert_eq!(data.hash, MESH_DROP_HASH, "mesh drop hash");
}
