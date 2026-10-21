//! Contact recycling safety tests, ported from `test/test_world.c` (upstream commit a2e4816
//! "Fixes for #175"): RecycledSphereApproachTest, PreSolveRejectRecycleTest and
//! SpeculativeOffRecycleTest.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use std::cell::Cell;

use crate::body::{body_get_position, create_body};
use crate::constants::{linear_slop, speculative_distance};
use crate::geometry::{Capsule, Sphere};
use crate::hull::make_box_hull;
use crate::id::ShapeId;
use crate::math_functions::{min_float, Pos, Vec3, VEC3_ZERO};
use crate::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::{world_get_counters, world_set_pre_solve_callback, World};

fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

// A slow sphere closes on a static round shape from just outside contact. The step is too short
// to make the sphere fast, so the pair is eligible for recycling. A routine without a speculative
// margin left the pair empty and recycling then let the sphere sink without a contact. Discussion #175.
// (RunRecycledApproach)
fn run_recycled_approach(capsule_target: bool) {
    let mut world_def = default_world_def();
    world_def.gravity = VEC3_ZERO;
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    let target_id = create_body(&mut world, &body_def);

    let shape_def = default_shape_def();
    let target_radius = 0.5f32;
    if capsule_target {
        let capsule = Capsule {
            center1: Vec3 {
                x: -1.0,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            radius: target_radius,
        };
        create_capsule_shape(&mut world, target_id, &shape_def, &capsule);
    } else {
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: target_radius,
        };
        create_sphere_shape(&mut world, target_id, &shape_def, &sphere);
    }

    let radius = 0.25f32;
    let total_radius = target_radius + radius;
    let time_step = 1.0f32 / 60.0;

    // Half the speculative distance per step
    let speed = 0.5 * speculative_distance() / time_step;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, total_radius + 0.25 * speculative_distance(), 0.0);
    body_def.linear_velocity = Vec3 {
        x: 0.0,
        y: -speed,
        z: 0.0,
    };
    let body_id = create_body(&mut world, &body_def);

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius,
    };
    create_sphere_shape(&mut world, body_id, &shape_def, &sphere);

    let mut min_separation = f32::MAX;
    for _ in 0..20 {
        world.step(time_step, 4);

        let p = body_get_position(&world, body_id);
        min_separation = min_float(min_separation, p.y as f32 - total_radius);
    }

    assert!(
        min_separation > -linear_slop(),
        "capsule target {capsule_target}: min separation {min_separation}"
    );
}

/// (RecycledSphereApproachTest)
#[test]
fn recycled_sphere_approach_test() {
    run_recycled_approach(false);
    run_recycled_approach(true);
}

thread_local! {
    // C hands the callback a pointer to a local bool. The port's pre-solve context is an integer,
    // so the flag lives in a thread local that only this test's thread touches.
    static PRE_SOLVE_REJECT: Cell<bool> = const { Cell::new(true) };
}

/// (PreSolveReject)
fn pre_solve_reject(
    _shape_id_a: ShapeId,
    _shape_id_b: ShapeId,
    _point: Pos,
    _normal: Vec3,
    _context: u64,
) -> bool {
    !PRE_SOLVE_REJECT.with(|reject| reject.get())
}

// A rejected pre-solve frees the manifold while the shapes may already touch. The next step must
// rebuild the contact and ask again instead of recycling the empty pair.
// (PreSolveRejectRecycleTest)
#[test]
fn pre_solve_reject_recycle_test() {
    let mut world_def = default_world_def();
    world_def.enable_sleep = false;
    let mut world = World::new(&world_def);

    PRE_SOLVE_REJECT.with(|reject| reject.set(true));
    world_set_pre_solve_callback(&mut world, Some(pre_solve_reject), 0);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = pos(0.0, -0.5, 0.0);
    let ground_id = create_body(&mut world, &body_def);
    let ground_box = make_box_hull(10.0, 0.5, 10.0);
    let mut shape_def = default_shape_def();
    create_hull_shape(&mut world, ground_id, &shape_def, &ground_box.base);

    let radius = 0.25f32;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, radius, 0.0);
    let body_id = create_body(&mut world, &body_def);

    shape_def.enable_pre_solve_events = true;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius,
    };
    create_sphere_shape(&mut world, body_id, &shape_def, &sphere);

    world.step(1.0 / 60.0, 4);

    PRE_SOLVE_REJECT.with(|reject| reject.set(false));

    let mut min_separation = f32::MAX;
    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);

        let p = body_get_position(&world, body_id);
        min_separation = min_float(min_separation, p.y as f32 - radius);
    }

    assert!(
        min_separation > -linear_slop(),
        "min separation {min_separation}"
    );
}

/// (CountRecycledResting)
fn count_recycled_resting(enable_speculative: bool) -> i32 {
    let mut world_def = default_world_def();
    world_def.enable_sleep = false;
    let mut world = World::new(&world_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = pos(0.0, -0.5, 0.0);
    let ground_id = create_body(&mut world, &body_def);
    let ground_box = make_box_hull(10.0, 0.5, 10.0);
    let mut shape_def = default_shape_def();
    create_hull_shape(&mut world, ground_id, &shape_def, &ground_box.base);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 0.5, 0.0);
    let body_id = create_body(&mut world, &body_def);

    shape_def.enable_speculative_contact = enable_speculative;
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, body_id, &shape_def, &box_hull.base);

    let mut recycled_count = 0;
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
        recycled_count += world_get_counters(&world).recycled_contact_count;
    }

    recycled_count
}

// Without speculative points an empty pair may be closer than the speculative distance, so these
// contacts must never recycle
// (SpeculativeOffRecycleTest)
#[test]
fn speculative_off_recycle_test() {
    assert!(count_recycled_resting(true) > 0);
    assert_eq!(count_recycled_resting(false), 0);
}
