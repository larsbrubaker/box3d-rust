//! World API and query tests from test_world.c plus query acceptance coverage.
//!
//! Not ported: TestWorldRecycle (global world registry), TestSetWorkerCount
//! (task system), and test_body_query.c (needs body-level cast APIs from
//! task-4). TestHullDatabase SetHull path deferred with remaining shape API.
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
use crate::math_functions::{
    abs_float, length, offset_pos, Aabb, Pos, Vec3, POS_ZERO, VEC3_ZERO,
};
use crate::shape::{create_hull_shape, create_sphere_shape, shape_is_valid};
use crate::types::{
    default_body_def, default_explosion_def, default_query_filter, default_shape_def,
    default_world_def, BodyType,
};
use crate::world::*;

fn custom_filter(_shape_a: ShapeId, _shape_b: ShapeId, _context: u64) -> bool {
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
