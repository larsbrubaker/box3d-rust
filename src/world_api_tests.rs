//! World API and query tests from test_world.c plus query acceptance coverage.
//!
//! Not ported: TestWorldRecycle (global world registry). Body-level casts live
//! in body_query_tests; TestHullDatabase SetHull path is covered in
//! shape_tests / shape_api_tests.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{
    body_get_angular_velocity, body_get_linear_velocity, body_is_valid, create_body,
};
use crate::distance::make_proxy;
use crate::geometry::Sphere;
use crate::hull::make_box_hull;
use crate::id::ShapeId;
use crate::math_functions::{abs_float, length, offset_pos, Aabb, Pos, Vec3, POS_ZERO, VEC3_ZERO};
use crate::shape::{
    create_hull_shape, create_sphere_shape, shape_get_user_data, shape_is_sensor, shape_is_valid,
};
use crate::types::{
    default_body_def, default_explosion_def, default_query_filter, default_shape_def,
    default_world_def, BodyType,
};
use crate::world::*;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

fn custom_filter(_world: &World, _shape_a: ShapeId, _shape_b: ShapeId, _context: u64) -> bool {
    true
}

fn pre_solve_static(
    _shape_a: ShapeId,
    _shape_b: ShapeId,
    _point: Pos,
    _normal: Vec3,
    _context: u64,
) -> bool {
    true
}

/// (test_world.c TestIsValid) — registry-less adaptation.
#[test]
fn test_is_valid() {
    let mut world = World::new(&default_world_def());
    assert!(world_is_valid(&world));

    let body1 = create_body(&mut world, &default_body_def());
    assert!(body_is_valid(&world, body1));

    let body2 = create_body(&mut world, &default_body_def());
    assert!(body_is_valid(&world, body2));

    // Destroyed bodies invalidate their ids; world stays valid while owned.
    crate::body::destroy_body(&mut world, body1);
    assert!(!body_is_valid(&world, body1));
    assert!(body_is_valid(&world, body2));

    crate::body::destroy_body(&mut world, body2);
    assert!(!body_is_valid(&world, body2));
    assert!(world_is_valid(&world));

    world.in_use = false;
    assert!(!world_is_valid(&world));
}

/// (test_world.c TestWorldCoverage)
#[test]
fn test_world_coverage() {
    let mut world = World::new(&default_world_def());
    assert!(world_is_valid(&world));

    world_enable_sleeping(&mut world, true);
    world_enable_sleeping(&mut world, false);
    assert!(!world_is_sleeping_enabled(&world));

    world_enable_continuous(&mut world, false);
    world_enable_continuous(&mut world, true);
    assert!(world_is_continuous_enabled(&world));

    world_set_restitution_threshold(&mut world, 0.0);
    world_set_restitution_threshold(&mut world, 2.0);
    assert_eq!(world_get_restitution_threshold(&world), 2.0);

    world_set_hit_event_threshold(&mut world, 0.0);
    world_set_hit_event_threshold(&mut world, 100.0);
    assert_eq!(world_get_hit_event_threshold(&world), 100.0);

    world_set_custom_filter_callback(&mut world, Some(custom_filter), 0);
    world_set_pre_solve_callback(&mut world, Some(pre_solve_static), 0);

    let g = Vec3 {
        x: 1.0,
        y: 2.0,
        z: 0.0,
    };
    world_set_gravity(&mut world, g);
    let v = world_get_gravity(&world);
    assert_eq!(v.x, g.x);
    assert_eq!(v.y, g.y);

    let explosion_def = default_explosion_def();
    world_explode(&mut world, &explosion_def);

    world_set_contact_tuning(&mut world, 10.0, 2.0, 4.0);

    world_set_maximum_linear_speed(&mut world, 10.0);
    assert_eq!(world_get_maximum_linear_speed(&world), 10.0);

    world_enable_warm_starting(&mut world, true);
    assert!(world_is_warm_starting_enabled(&world));

    assert_eq!(world_get_awake_body_count(&world), 0);

    world_set_user_data(&mut world, 42);
    assert_eq!(world_get_user_data(&world), 42);

    world.step(1.0, 1);

    let counters = world_get_counters(&world);
    assert_eq!(counters.body_count, 0);
}

struct ExplosionResult {
    linear_velocity: Vec3,
    angular_velocity: Vec3,
}

fn run_explosion(base: Pos) -> ExplosionResult {
    let mut world_def = default_world_def();
    world_def.gravity = VEC3_ZERO;
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = base;
    let body_id = create_body(&mut world, &body_def);

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };
    create_sphere_shape(&mut world, body_id, &default_shape_def(), &sphere);

    let mut explosion_def = default_explosion_def();
    explosion_def.position = offset_pos(
        base,
        Vec3 {
            x: 3.0,
            y: 0.0,
            z: 0.0,
        },
    );
    explosion_def.radius = 5.0;
    explosion_def.falloff = 0.0;
    explosion_def.impulse_per_area = 10.0;
    world_explode(&mut world, &explosion_def);

    ExplosionResult {
        linear_velocity: body_get_linear_velocity(&world, body_id),
        angular_velocity: body_get_angular_velocity(&world, body_id),
    }
}

/// (test_world.c TestExplosion)
#[test]
fn test_explosion() {
    let origin = run_explosion(POS_ZERO);

    assert!(origin.linear_velocity.x < -1.0e-4);
    assert!(abs_float(origin.linear_velocity.y) < 1.0e-6);
    assert!(abs_float(origin.linear_velocity.z) < 1.0e-6);
    assert!(length(origin.angular_velocity) < 1.0e-6);

    let far = run_explosion(Pos {
        x: 1.0e7 as _,
        y: 1.0e7 as _,
        z: 1.0e7 as _,
    });
    assert!(abs_float(far.linear_velocity.x - origin.linear_velocity.x) < 1.0e-5);
    assert!(abs_float(far.linear_velocity.y - origin.linear_velocity.y) < 1.0e-5);
    assert!(abs_float(far.linear_velocity.z - origin.linear_velocity.z) < 1.0e-5);
}

/// Acceptance coverage for world overlap / cast against a static 2×2×2 box.
#[test]
fn world_queries() {
    let mut world = World::new(&default_world_def());

    let body_id = create_body(&mut world, &default_body_def());
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    let shape_id = create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);
    assert!(shape_is_valid(&world, shape_id));

    let filter = default_query_filter();

    let mut count = 0;
    world_overlap_aabb(
        &world,
        Aabb {
            lower_bound: Vec3 {
                x: -0.5,
                y: -0.5,
                z: -0.5,
            },
            upper_bound: Vec3 {
                x: 0.5,
                y: 0.5,
                z: 0.5,
            },
        },
        &filter,
        |_| {
            count += 1;
            true
        },
    );
    assert_eq!(count, 1);

    count = 0;
    world_overlap_aabb(
        &world,
        Aabb {
            lower_bound: Vec3 {
                x: 100.0,
                y: -0.5,
                z: -0.5,
            },
            upper_bound: Vec3 {
                x: 101.0,
                y: 0.5,
                z: 0.5,
            },
        },
        &filter,
        |_| {
            count += 1;
            true
        },
    );
    assert_eq!(count, 0);

    let sphere_proxy = make_proxy(&[VEC3_ZERO], 0.25);
    count = 0;
    world_overlap_shape(&world, POS_ZERO, &sphere_proxy, &filter, |_| {
        count += 1;
        true
    });
    assert_eq!(count, 1);

    // Ray from x=-5 along +x hits the face at x=-1 → fraction 0.4.
    let result = world_cast_ray_closest(
        &world,
        Pos {
            x: -5.0 as _,
            y: 0.0 as _,
            z: 0.0 as _,
        },
        Vec3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        },
        &filter,
    );
    assert!(result.hit);
    assert!(abs_float(result.fraction - 0.4) < 1e-5);
    assert!(abs_float(result.normal.x - 1.0) < 1e-4 || abs_float(result.normal.x + 1.0) < 1e-4);

    let mut hit_frac = 1.0f32;
    world_cast_ray(
        &world,
        Pos {
            x: -5.0 as _,
            y: 0.0 as _,
            z: 0.0 as _,
        },
        Vec3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        },
        &filter,
        |_id, _point, _normal, fraction, _mat, _tri, _child| {
            hit_frac = fraction;
            fraction
        },
    );
    assert!(abs_float(hit_frac - 0.4) < 1e-5);

    // Sphere proxy of radius 0.5 cast into the box face at x=-1.
    let cast_proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let mut shape_hit = false;
    world_cast_shape(
        &world,
        Pos {
            x: -5.0 as _,
            y: 0.0 as _,
            z: 0.0 as _,
        },
        &cast_proxy,
        Vec3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        },
        &filter,
        |_id, _point, _normal, fraction, _mat, _tri, _child| {
            shape_hit = true;
            // Front face at -1; radius 0.5 → contact near fraction 0.35.
            assert!(abs_float(fraction - 0.35) < 1e-2);
            fraction
        },
    );
    assert!(shape_hit);
}

/// (test_world.c TestSetWorkerCount) — serial port stores/clamps the count.
#[test]
fn test_set_worker_count() {
    use crate::constants::MAX_WORKERS;

    let mut world = World::new(&default_world_def());
    assert_eq!(world_get_worker_count(&world), 1);

    world_set_worker_count(&mut world, 4);
    assert_eq!(world_get_worker_count(&world), 4);
    assert_eq!(world.task_contexts.len(), 4);
    assert_eq!(world.sensor_task_contexts.len(), 4);

    world_set_worker_count(&mut world, 4);
    assert_eq!(world_get_worker_count(&world), 4);

    world_set_worker_count(&mut world, 0);
    assert_eq!(world_get_worker_count(&world), 1);

    world_set_worker_count(&mut world, -5);
    assert_eq!(world_get_worker_count(&world), 1);

    world_set_worker_count(&mut world, MAX_WORKERS + 10);
    assert_eq!(world_get_worker_count(&world), MAX_WORKERS);
    assert_eq!(world.task_contexts.len(), MAX_WORKERS as usize);

    // Keep a body around so stepping with a non-1 stored count still works.
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    create_sphere_shape(
        &mut world,
        body,
        &shape_def,
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
    );
    world.step(1.0 / 60.0, 1);
}

/// (b3World_GetBounds)
#[test]
fn test_world_get_bounds() {
    let mut world = World::new(&default_world_def());
    let empty = world_get_bounds(&world);
    assert_eq!(empty.lower_bound, VEC3_ZERO);
    assert_eq!(empty.upper_bound, VEC3_ZERO);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 2.0 as _,
        z: 0.0 as _,
    };
    let body = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    create_sphere_shape(
        &mut world,
        body,
        &shape_def,
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
    );

    let bounds = world_get_bounds(&world);
    assert!(bounds.lower_bound.y < 2.0);
    assert!(bounds.upper_bound.y > 2.0);
}

// Custom-filter userData access, mirroring sample_benchmark.cpp
// `BenchmarkSensor::Filter` (:922), which reads a shape's userData mid-step
// through `b3Shape_GetUserData(shapeId)`. The Rust callback gets the same reach
// via the `&World` it now receives.

const FILTER_SENTINEL: u64 = 0xBEEF;
static FILTER_SAW_USER_DATA: AtomicU64 = AtomicU64::new(0);
static FILTER_SAW_SENSOR: AtomicBool = AtomicBool::new(false);

/// Reads both shapes' userData (and sensor flag) through `&World`, then suppresses
/// the pair when either shape carries [`FILTER_SENTINEL`].
fn user_data_filter(world: &World, shape_a: ShapeId, shape_b: ShapeId, _context: u64) -> bool {
    let ud_a = shape_get_user_data(world, shape_a);
    let ud_b = shape_get_user_data(world, shape_b);
    FILTER_SAW_USER_DATA.store(ud_a | ud_b, Ordering::Relaxed);
    FILTER_SAW_SENSOR.store(
        shape_is_sensor(world, shape_a) || shape_is_sensor(world, shape_b),
        Ordering::Relaxed,
    );
    ud_a != FILTER_SENTINEL && ud_b != FILTER_SENTINEL
}

/// Two overlapping dynamic boxes; shape A enables custom filtering and carries
/// `user_data_a`. Steps the world and returns `(max_speed, saw_user_data)`.
fn overlap_filter_run(user_data_a: u64) -> (f32, u64) {
    let mut world_def = default_world_def();
    world_def.gravity = VEC3_ZERO;
    let mut world = World::new(&world_def);
    world_set_custom_filter_callback(&mut world, Some(user_data_filter), 0);

    let box_hull = make_box_hull(0.5, 0.5, 0.5);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let body_a = create_body(&mut world, &body_def);
    let mut shape_def_a = default_shape_def();
    shape_def_a.density = 1.0;
    shape_def_a.enable_custom_filtering = true;
    shape_def_a.user_data = user_data_a;
    create_hull_shape(&mut world, body_a, &shape_def_a, &box_hull.base);

    body_def.position = Pos {
        x: 0.4,
        y: 0.0,
        z: 0.0,
    };
    let body_b = create_body(&mut world, &body_def);
    let mut shape_def_b = default_shape_def();
    shape_def_b.density = 1.0;
    shape_def_b.user_data = 3;
    create_hull_shape(&mut world, body_b, &shape_def_b, &box_hull.base);

    FILTER_SAW_USER_DATA.store(0, Ordering::Relaxed);
    FILTER_SAW_SENSOR.store(true, Ordering::Relaxed);
    for _ in 0..8 {
        world.step(1.0 / 60.0, 4);
    }

    let speed = length(body_get_linear_velocity(&world, body_a))
        .max(length(body_get_linear_velocity(&world, body_b)));
    (speed, FILTER_SAW_USER_DATA.load(Ordering::Relaxed))
}

/// The custom filter reads shape userData through `&World` and suppresses the
/// pair accordingly (mirrors `BenchmarkSensor::Filter`).
#[test]
fn test_custom_filter_reads_shape_user_data() {
    // Sentinel userData on shape A → the filter returns false → the overlapping
    // boxes never form a contact and never separate.
    let (suppressed_speed, saw) = overlap_filter_run(FILTER_SENTINEL);
    assert_eq!(
        saw,
        FILTER_SENTINEL | 3,
        "filter should have read both shapes' userData through &World"
    );
    assert!(
        !FILTER_SAW_SENSOR.load(Ordering::Relaxed),
        "neither box is a sensor"
    );
    assert!(
        suppressed_speed < 1e-4,
        "suppressed pair should not separate, got speed {suppressed_speed}"
    );

    // Non-sentinel userData → the filter allows the pair → the overlap resolves
    // and the boxes gain separating velocity. Confirms the suppression above was
    // caused by the userData the callback read, not by absence of collision.
    let (allowed_speed, _) = overlap_filter_run(5);
    assert!(
        allowed_speed > 1e-2,
        "allowed pair should separate, got speed {allowed_speed}"
    );
}
