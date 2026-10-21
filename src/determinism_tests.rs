//! Port of `test/test_determinism.c`: falling-ragdoll sleep step and world
//! transform hash must match the C scalar (`BOX3D_DISABLE_SIMD`) reference.
//!
//! The Rust port is serial, so the C worker-count loops cannot vary the schedule.
//! Both precision modes have distinct golden values.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{
    body_get_angular_velocity, body_get_contact_data, body_get_linear_velocity, body_get_transform,
    body_set_angular_velocity, body_set_linear_velocity, create_body,
};
use crate::constants::MAX_MANIFOLD_POINTS;
use crate::core::{hash, HASH_INIT};
use crate::determinism::hash_world_transform;
use crate::determinism::{
    create_falling_ragdolls, create_mesh_drop, create_query_spawn, create_wave_pile,
    destroy_falling_ragdolls, destroy_mesh_drop, destroy_query_spawn, destroy_wave_pile,
    update_falling_ragdolls, update_mesh_drop, update_query_spawn, update_wave_pile,
    FallingRagdollData, QUERY_SPAWN_COUNT,
};
use crate::geometry::{Capsule, Sphere};
use crate::hull::make_box_hull;
use crate::id::BodyId;
use crate::math_functions::{Pos, Vec3, POS_ZERO, VEC3_ONE};
use crate::mesh::{create_wave_mesh, destroy_mesh};
use crate::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
};
use crate::simd_scalar::{get_simd_width, set_simd_width};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::World;

// Double precision accumulates body positions in double, so the settle/sleep
// step and the state hash differ from the float build. Both modes are
// internally deterministic. Values from test_determinism.c.
#[cfg(feature = "double-precision")]
const RAGDOLL_SLEEP_STEP: i32 = 263;
#[cfg(feature = "double-precision")]
const RAGDOLL_HASH: u32 = 0xA542_3F98;
#[cfg(feature = "double-precision")]
const WAVE_PILE_SLEEP_STEP: i32 = 272;
#[cfg(feature = "double-precision")]
const WAVE_PILE_HASH: u32 = 0xC8E5_F3D6;
#[cfg(feature = "double-precision")]
const QUERY_SPAWN_SLEEP_STEP: i32 = 242;
#[cfg(feature = "double-precision")]
const QUERY_SPAWN_HASH: u32 = 0x4895_85E6;
#[cfg(feature = "double-precision")]
const QUERY_SPAWN_HIT_COUNT: i32 = 59;
#[cfg(feature = "double-precision")]
const QUERY_SPAWN_QUERY_HASH: u32 = 0x5B44_29DC;
#[cfg(feature = "double-precision")]
const MESH_DROP_SLEEP_STEP: i32 = 218;
#[cfg(feature = "double-precision")]
const MESH_DROP_HASH: u32 = 0xD378_CF04;

#[cfg(not(feature = "double-precision"))]
const RAGDOLL_SLEEP_STEP: i32 = 291;
#[cfg(not(feature = "double-precision"))]
const RAGDOLL_HASH: u32 = 0xFF4C_605B;
#[cfg(not(feature = "double-precision"))]
const WAVE_PILE_SLEEP_STEP: i32 = 272;
#[cfg(not(feature = "double-precision"))]
const WAVE_PILE_HASH: u32 = 0x266F_83B0;
#[cfg(not(feature = "double-precision"))]
const QUERY_SPAWN_SLEEP_STEP: i32 = 242;
#[cfg(not(feature = "double-precision"))]
const QUERY_SPAWN_HASH: u32 = 0xF3AD_9BA6;
#[cfg(not(feature = "double-precision"))]
const QUERY_SPAWN_HIT_COUNT: i32 = 59;
#[cfg(not(feature = "double-precision"))]
const QUERY_SPAWN_QUERY_HASH: u32 = 0xE327_1F3D;
#[cfg(not(feature = "double-precision"))]
const MESH_DROP_SLEEP_STEP: i32 = 226;
#[cfg(not(feature = "double-precision"))]
const MESH_DROP_HASH: u32 = 0xD2FF_069D;

// SingleRollingMixTest has no golden in test_determinism.c (it only compares widths). These
// come from the scalar C build (BOX3D_DISABLE_SIMD) of efca0c1, run with one worker.
#[cfg(feature = "double-precision")]
const ROLLING_MIX_HASH: u32 = 0xB7FB_CFC0;
#[cfg(not(feature = "double-precision"))]
const ROLLING_MIX_HASH: u32 = 0x56B2_50F7;

// SingleMeshMixTest has no golden in test_determinism.c either. These come from the scalar C
// build (BOX3D_DISABLE_SIMD) of efca0c1, run with one worker.
#[cfg(feature = "double-precision")]
const MESH_MIX_HASH: u32 = 0x9DA0_B374;
#[cfg(not(feature = "double-precision"))]
const MESH_MIX_HASH: u32 = 0x246C_C2A7;

// C loops each scenario over several worker counts to prove every schedule reproduces the
// first run (EnsureRepeatable). The serial port ignores the worker count, so extra runs would
// be identical work; exactly two runs per scenario keep the useful part of the check (run 2
// in a fresh world must match run 1, which catches state leaking between worlds).
const REPEAT_RUNS: i32 = 2;

// The goldens above pin exact values for the default four point manifold. A build that
// overrides B3_MAX_MANIFOLD_POINTS produces a different contact set, so those checks drop
// out and only the run to run agreement below applies. (ENSURE_GOLDEN)
const ENSURE_GOLDEN: bool = MAX_MANIFOLD_POINTS == 4;

/// (DeterminismResult)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct DeterminismResult {
    sleep_step: i32,
    hash: u32,
    query_hit_count: i32,
    query_hash: u32,
    seeded: bool,
}

/// Every worker count has to reproduce the first run exactly. Scenarios without queries leave
/// the query fields zero in both the reference and the result. (EnsureRepeatable)
fn ensure_repeatable(
    reference: &mut DeterminismResult,
    mut result: DeterminismResult,
    label: &str,
) {
    if !reference.seeded {
        result.seeded = true;
        *reference = result;
        return;
    }

    assert_eq!(
        result.sleep_step, reference.sleep_step,
        "{label}: repeat sleep step"
    );
    assert_eq!(result.hash, reference.hash, "{label}: repeat hash");
    assert_eq!(
        result.query_hit_count, reference.query_hit_count,
        "{label}: repeat query hit count"
    );
    assert_eq!(
        result.query_hash, reference.query_hash,
        "{label}: repeat query hash"
    );
}

fn ensure_ragdoll_golden(data: &FallingRagdollData, label: &str) {
    if data.sleep_step != RAGDOLL_SLEEP_STEP || data.hash != RAGDOLL_HASH {
        eprintln!(
            "  {label} sleepStep={} hash=0x{:08X} (expected sleepStep={} hash=0x{:08X})",
            data.sleep_step, data.hash, RAGDOLL_SLEEP_STEP, RAGDOLL_HASH
        );
    }

    if ENSURE_GOLDEN {
        assert_eq!(
            data.sleep_step, RAGDOLL_SLEEP_STEP,
            "{label}: sleep_step mismatch"
        );
        assert_eq!(data.hash, RAGDOLL_HASH, "{label}: hash mismatch");
    }
}

/// Serial equivalent of C `SingleMultithreadingTest`. `run` only labels the output.
fn single_multithreading_test(run: i32, reference: &mut DeterminismResult) {
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

    let label = format!("run={run}");
    ensure_ragdoll_golden(&data, &label);
    ensure_repeatable(
        reference,
        DeterminismResult {
            sleep_step: data.sleep_step,
            hash: data.hash,
            ..Default::default()
        },
        &label,
    );

    destroy_falling_ragdolls(&mut data);
}

/// Test multithreaded determinism. (MultithreadingTest)
#[test]
fn multithreading_test() {
    let mut reference = DeterminismResult::default();
    for run in 1..=REPEAT_RUNS {
        single_multithreading_test(run, &mut reference);
    }
}

/// Test cross platform determinism. (CrossPlatformTest)
#[test]
fn cross_platform_test() {
    let mut world = World::new(&default_world_def());
    let mut data = create_falling_ragdolls(&mut world);

    let time_step = 1.0 / 60.0;
    loop {
        world.step(time_step, 4);
        if update_falling_ragdolls(&world, &mut data) {
            break;
        }
    }

    ensure_ragdoll_golden(&data, "cross-platform");

    destroy_falling_ragdolls(&mut data);
}

/// Serial equivalent of C `SingleWavePileTest`.
fn single_wave_pile_test(run: i32, reference: &mut DeterminismResult) {
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

    if data.sleep_step != WAVE_PILE_SLEEP_STEP || data.hash != WAVE_PILE_HASH {
        eprintln!(
            "  wave pile run={run} sleepStep={} hash=0x{:08X} (expected sleepStep={} hash=0x{:08X})",
            data.sleep_step, data.hash, WAVE_PILE_SLEEP_STEP, WAVE_PILE_HASH
        );
    }

    assert!(done, "wave pile did not settle within 500 steps");
    if ENSURE_GOLDEN {
        assert_eq!(
            data.sleep_step, WAVE_PILE_SLEEP_STEP,
            "wave pile sleep step"
        );
        assert_eq!(data.hash, WAVE_PILE_HASH, "wave pile hash");
    }
    ensure_repeatable(
        reference,
        DeterminismResult {
            sleep_step: data.sleep_step,
            hash: data.hash,
            ..Default::default()
        },
        "wave pile",
    );

    destroy_wave_pile(&mut data);
}

/// Test multithreaded determinism of a mixed convex pile on a wave height field.
/// (WavePileTest)
#[test]
fn wave_pile_test() {
    let mut reference = DeterminismResult::default();
    for run in 1..=REPEAT_RUNS {
        single_wave_pile_test(run, &mut reference);
    }
}

/// Serial equivalent of C `SingleQuerySpawnTest`.
fn single_query_spawn_test(run: i32, reference: &mut DeterminismResult) {
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

    if data.sleep_step != QUERY_SPAWN_SLEEP_STEP
        || data.hash != QUERY_SPAWN_HASH
        || data.query_hit_count != QUERY_SPAWN_HIT_COUNT
        || data.query_hash != QUERY_SPAWN_QUERY_HASH
    {
        eprintln!(
            "  query spawn run={run} sleepStep={} hash=0x{:08X} hits={} queryHash=0x{:08X}\n  \
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

    assert!(done, "query spawn did not settle within 1000 steps");
    assert_eq!(
        data.spawn_count as usize, QUERY_SPAWN_COUNT,
        "query spawn count"
    );
    if ENSURE_GOLDEN {
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
    ensure_repeatable(
        reference,
        DeterminismResult {
            sleep_step: data.sleep_step,
            hash: data.hash,
            query_hit_count: data.query_hit_count,
            query_hash: data.query_hash,
            seeded: false,
        },
        "query spawn",
    );

    destroy_query_spawn(&mut data);
}

/// Test determinism of world queries by feeding their results back into the simulation.
/// (QuerySpawnTest)
#[test]
fn query_spawn_test() {
    let mut reference = DeterminismResult::default();
    for run in 1..=REPEAT_RUNS {
        single_query_spawn_test(run, &mut reference);
    }
}

/// Serial equivalent of C `SingleMeshDropTest`.
fn single_mesh_drop_test(run: i32, reference: &mut DeterminismResult) {
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

    if data.sleep_step != MESH_DROP_SLEEP_STEP || data.hash != MESH_DROP_HASH {
        eprintln!(
            "  mesh drop run={run} sleepStep={} hash=0x{:08X} (expected sleepStep={} hash=0x{:08X})",
            data.sleep_step, data.hash, MESH_DROP_SLEEP_STEP, MESH_DROP_HASH
        );
    }

    assert!(done, "mesh drop did not settle within 400 steps");
    if ENSURE_GOLDEN {
        assert_eq!(
            data.sleep_step, MESH_DROP_SLEEP_STEP,
            "mesh drop sleep step"
        );
        assert_eq!(data.hash, MESH_DROP_HASH, "mesh drop hash");
    }
    ensure_repeatable(
        reference,
        DeterminismResult {
            sleep_step: data.sleep_step,
            hash: data.hash,
            ..Default::default()
        },
        "mesh drop",
    );

    destroy_mesh_drop(&mut data);
}

/// C runs only the single threaded and widest schedules because the scene is large.
/// (MeshDropTest)
#[test]
fn mesh_drop_test() {
    let mut reference = DeterminismResult::default();
    for run in 1..=REPEAT_RUNS {
        single_mesh_drop_test(run, &mut reference);
    }
}

/// Hash raw float bits, matching `b3Hash( hash, (const uint8_t*)values, count * sizeof( float ) )`.
/// (HashFloats)
fn hash_floats(h: u32, values: &[f32]) -> u32 {
    let mut bytes = Vec::with_capacity(4 * values.len());
    for v in values {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    hash(h, &bytes)
}

/// Rolling resistance is mixed per contact and the manifolds have one, two, and four points, so
/// a width 8 constraint can mix lanes that width 4 keeps in separate constraints. The hash covers
/// the stored contact impulses bitwise, including the sign of zero, because those survive into
/// warm starting and contact data. (SingleRollingMixTest)
fn single_rolling_mix_test(reference: &mut DeterminismResult) {
    let mut world = World::new(&default_world_def());

    let ground_def = default_body_def();
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(40.0, 1.0, 40.0);
    let ground_shape_def = default_shape_def();
    create_hull_shape(&mut world, ground_id, &ground_shape_def, &ground_box.base);

    const ROW_COUNT: usize = 8;
    const COLUMN_COUNT: usize = 8;
    const BODY_COUNT: usize = ROW_COUNT * COLUMN_COUNT;

    let mut body_ids = [BodyId::default(); BODY_COUNT];
    let box_hull = make_box_hull(0.4, 0.4, 0.4);
    let sphere = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.4,
    };
    let capsule = Capsule {
        center1: Vec3 {
            x: -0.3,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.3,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.25,
    };

    let mut seed: u32 = 271828;
    for (i, body_id) in body_ids.iter_mut().enumerate() {
        let row = i / COLUMN_COUNT;
        let column = i % COLUMN_COUNT;

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (2.0 * column as f32 - 7.0) as _,
            y: 1.45f32 as _,
            z: (2.0 * row as f32 - 7.0) as _,
        };
        *body_id = create_body(&mut world, &body_def);

        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        shape_def.base_material.rolling_resistance = if (seed >> 28) < 5 { 0.2 } else { 0.0 };

        match i % 3 {
            0 => {
                create_sphere_shape(&mut world, *body_id, &shape_def, &sphere);
            }
            1 => {
                create_capsule_shape(&mut world, *body_id, &shape_def, &capsule);
            }
            _ => {
                create_hull_shape(&mut world, *body_id, &shape_def, &box_hull.base);
            }
        }

        let sign = if (i & 1) != 0 { -1.0 } else { 1.0 };
        body_set_linear_velocity(
            &mut world,
            *body_id,
            Vec3 {
                x: -0.0,
                y: -0.5,
                z: sign * 0.5,
            },
        );
        body_set_angular_velocity(
            &mut world,
            *body_id,
            Vec3 {
                x: sign * 3.0,
                y: -0.0,
                z: -2.0 * sign,
            },
        );
    }

    let mut h = HASH_INIT;
    let step_count = 120;
    for _ in 0..step_count {
        world.step(1.0 / 60.0, 4);

        for body_id in &body_ids {
            let xf = body_get_transform(&world, *body_id);
            h = hash_world_transform(h, &xf);

            let contact_data = body_get_contact_data(&world, *body_id, 4);
            for data in &contact_data {
                for manifold in &data.manifolds {
                    let r = manifold.rolling_impulse;
                    let f = manifold.friction_impulse;
                    h = hash_floats(h, &[r.x, r.y, r.z]);
                    h = hash_floats(h, &[f.x, f.y, f.z]);
                    h = hash_floats(h, &[manifold.twist_impulse]);

                    for p in &manifold.points[..manifold.point_count as usize] {
                        h = hash_floats(h, &[p.normal_impulse]);
                        h = hash_floats(h, &[p.total_normal_impulse]);
                    }
                }
            }
        }
    }

    if ENSURE_GOLDEN {
        assert_eq!(
            h, ROLLING_MIX_HASH,
            "rolling mix hash 0x{h:08X} (expected 0x{ROLLING_MIX_HASH:08X})"
        );
    }

    ensure_repeatable(
        reference,
        DeterminismResult {
            sleep_step: step_count,
            hash: h,
            ..Default::default()
        },
        "rolling mix",
    );
}

/// Mesh contacts are grouped across lanes by manifold count and padded to the largest count in
/// the group. Width 8 groups differ from width 4 groups, so mixing large and small shapes on a
/// fine mesh gives each body a width dependent number of padded manifolds. The hash covers
/// velocities and impulses bitwise to catch a padded manifold flipping a signed zero.
/// (SingleMeshMixTest)
fn single_mesh_mix_test(reference: &mut DeterminismResult) {
    let mut world = World::new(&default_world_def());

    let mesh = create_wave_mesh(40, 40, 0.5, 0.3, 0.2, 0.3).expect("wave mesh");
    {
        let mut ground_def = default_body_def();
        ground_def.position = Pos {
            x: -10.0f32 as _,
            y: 0.0f32 as _,
            z: -10.0f32 as _,
        };
        let ground_id = create_body(&mut world, &ground_def);
        let shape_def = default_shape_def();
        create_mesh_shape(&mut world, ground_id, &shape_def, &mesh, VEC3_ONE);
    }

    const ROW_COUNT: usize = 8;
    const COLUMN_COUNT: usize = 8;
    const BODY_COUNT: usize = ROW_COUNT * COLUMN_COUNT;

    let mut body_ids = [BodyId::default(); BODY_COUNT];
    let small_box = make_box_hull(0.2, 0.2, 0.2);
    let large_box = make_box_hull(0.9, 0.3, 0.9);
    let sphere = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.3,
    };
    let capsule = Capsule {
        center1: Vec3 {
            x: -0.5,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut seed: u32 = 314159;
    for (i, body_id) in body_ids.iter_mut().enumerate() {
        let row = i / COLUMN_COUNT;
        let column = i % COLUMN_COUNT;

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (2.2 * column as f32 - 7.7) as _,
            y: 1.2f32 as _,
            z: (2.2 * row as f32 - 7.7) as _,
        };
        *body_id = create_body(&mut world, &body_def);

        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        shape_def.base_material.rolling_resistance = if (seed >> 28) < 5 { 0.2 } else { 0.0 };

        match (seed >> 24) & 3 {
            0 => {
                create_hull_shape(&mut world, *body_id, &shape_def, &large_box.base);
            }
            1 => {
                create_hull_shape(&mut world, *body_id, &shape_def, &small_box.base);
            }
            2 => {
                create_sphere_shape(&mut world, *body_id, &shape_def, &sphere);
            }
            _ => {
                create_capsule_shape(&mut world, *body_id, &shape_def, &capsule);
            }
        }

        let sign = if (i & 1) != 0 { -1.0 } else { 1.0 };
        body_set_linear_velocity(
            &mut world,
            *body_id,
            Vec3 {
                x: -0.0,
                y: -0.5,
                z: -0.0,
            },
        );
        body_set_angular_velocity(
            &mut world,
            *body_id,
            Vec3 {
                x: -0.0,
                y: sign * 0.5,
                z: -0.0,
            },
        );
    }

    let mut h = HASH_INIT;
    let step_count = 180;
    for _ in 0..step_count {
        world.step(1.0 / 60.0, 4);

        for body_id in &body_ids {
            let xf = body_get_transform(&world, *body_id);
            h = hash_world_transform(h, &xf);

            let v = body_get_linear_velocity(&world, *body_id);
            let w = body_get_angular_velocity(&world, *body_id);
            h = hash_floats(h, &[v.x, v.y, v.z]);
            h = hash_floats(h, &[w.x, w.y, w.z]);

            let contact_data = body_get_contact_data(&world, *body_id, 8);
            for data in &contact_data {
                for manifold in &data.manifolds {
                    let r = manifold.rolling_impulse;
                    let f = manifold.friction_impulse;
                    h = hash_floats(h, &[r.x, r.y, r.z]);
                    h = hash_floats(h, &[f.x, f.y, f.z]);
                    h = hash_floats(h, &[manifold.twist_impulse]);

                    for p in &manifold.points[..manifold.point_count as usize] {
                        h = hash_floats(h, &[p.normal_impulse]);
                        h = hash_floats(h, &[p.total_normal_impulse]);
                    }
                }
            }
        }
    }

    drop(world);
    destroy_mesh(mesh);

    if ENSURE_GOLDEN {
        assert_eq!(
            h, MESH_MIX_HASH,
            "mesh mix hash 0x{h:08X} (expected 0x{MESH_MIX_HASH:08X})"
        );
    }

    ensure_repeatable(
        reference,
        DeterminismResult {
            sleep_step: step_count,
            hash: h,
            ..Default::default()
        },
        "mesh mix",
    );
}

/// (SceneFcn)
type SceneFcn = fn(&mut DeterminismResult);

fn ragdoll_scene(result: &mut DeterminismResult) {
    single_multithreading_test(1, result);
}

fn wave_pile_scene(result: &mut DeterminismResult) {
    single_wave_pile_test(1, result);
}

fn query_spawn_scene(result: &mut DeterminismResult) {
    single_query_spawn_test(1, result);
}

fn mesh_drop_scene(result: &mut DeterminismResult) {
    single_mesh_drop_test(1, result);
}

/// (RunSceneAtWidth)
fn run_scene_at_width(scene: SceneFcn, width: i32, result: &mut DeterminismResult) {
    set_simd_width(width);
    scene(result);
    set_simd_width(0);
}

/// Width 4 and the native width must produce bitwise identical simulations. (SIMDWidthTest)
///
/// C skips this when the native width is 4, which is always the case for the scalar port. The
/// port still runs both passes: they then check run to run agreement and that forcing a width
/// through `set_simd_width` leaves the simulation untouched.
#[test]
fn simd_width_test() {
    set_simd_width(0);
    let native_width = get_simd_width();
    assert_eq!(native_width, 4, "the scalar port has no width 8 kernels");

    let scenes: [SceneFcn; 6] = [
        ragdoll_scene,
        wave_pile_scene,
        query_spawn_scene,
        mesh_drop_scene,
        single_rolling_mix_test,
        single_mesh_mix_test,
    ];

    for scene in scenes {
        let mut narrow = DeterminismResult::default();
        let mut wide = DeterminismResult::default();

        run_scene_at_width(scene, 4, &mut narrow);
        run_scene_at_width(scene, native_width, &mut wide);

        assert!(narrow.seeded && wide.seeded);
        assert_eq!(narrow.sleep_step, wide.sleep_step);
        assert_eq!(narrow.hash, wide.hash);
        assert_eq!(narrow.query_hit_count, wide.query_hit_count);
        assert_eq!(narrow.query_hash, wide.query_hash);
    }
}

/// The rolling mix scene against the scalar C reference. C only runs it inside SIMDWidthTest;
/// this pins it on its own so a failure names the scene.
#[test]
fn rolling_mix_test() {
    let mut reference = DeterminismResult::default();
    for _ in 1..=REPEAT_RUNS {
        single_rolling_mix_test(&mut reference);
    }
}

/// The mesh mix scene against the scalar C reference. C only runs it inside SIMDWidthTest; this
/// pins it on its own so a failure names the scene.
#[test]
fn mesh_mix_test() {
    let mut reference = DeterminismResult::default();
    for _ in 1..=REPEAT_RUNS {
        single_mesh_mix_test(&mut reference);
    }
}
