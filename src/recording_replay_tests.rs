//! Recording / replay tests ported from test_recording.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::create_body;
use crate::geometry::Sphere;
use crate::hull::{create_hull, make_box_hull};
use crate::math_functions::{Pos, Vec3, VEC3_ZERO};
use crate::recording::{
    validate_replay, Recording, REC_MAGIC,
};
use crate::shape::{create_hull_shape, create_sphere_shape};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::{world_set_gravity, world_start_recording, world_stop_recording, World};

/// Sphere round-trip: record/step/stop, then replay and validate. (SphereRoundTrip)
#[test]
fn sphere_round_trip() {
    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());

    world_start_recording(&mut world, &mut rec);

    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);

    let ground_box = make_box_hull(50.0, 1.0, 50.0);
    let ground_shape_def = default_shape_def();
    create_hull_shape(&mut world, ground_id, &ground_shape_def, &ground_box.base);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0,
        y: 5.0,
        z: 0.0,
    };
    let body_id = create_body(&mut world, &body_def);

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let mut sphere_def = default_shape_def();
    sphere_def.density = 1.0;
    create_sphere_shape(&mut world, body_id, &sphere_def, &sphere);

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for _ in 0..30 {
        world.step(time_step, sub_step_count);
    }

    world_stop_recording(&mut world);

    assert!(validate_replay(rec.data(), 1));
}

/// Empty world: start/stop with no ops beyond seed hash. (EmptyWorldRoundTrip)
#[test]
fn empty_world_round_trip() {
    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, &mut rec);
    world_stop_recording(&mut world);
    assert!(rec.size() >= 48);
    assert_eq!(
        u32::from_le_bytes(rec.data()[0..4].try_into().unwrap()),
        REC_MAGIC
    );
    assert!(validate_replay(rec.data(), 1));
}

/// Hull dedup: three bodies sharing the same hull → one registry entry. (HullDedup)
#[test]
fn hull_dedup() {
    let pts = [
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: -1.0,
        },
        Vec3 {
            x: 1.0,
            y: -1.0,
            z: -1.0,
        },
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: -1.0,
        },
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: -1.0,
        },
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: 1.0,
        },
        Vec3 {
            x: 1.0,
            y: -1.0,
            z: 1.0,
        },
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: 1.0,
        },
    ];
    let hull = create_hull(&pts, 8).expect("hull");

    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, &mut rec);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;

    for i in 0..3 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (i * 3) as f32,
            y: 5.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &shape_def, &hull);
    }

    let time_step = 1.0 / 60.0;
    for _ in 0..5 {
        world.step(time_step, 4);
    }

    world_stop_recording(&mut world);

    assert!(validate_replay(rec.data(), 1));

    let bytes = rec.data();
    assert!(bytes.len() >= 48);
    let reg_off = u64::from_le_bytes(bytes[32..40].try_into().unwrap()) as usize;
    assert!(reg_off != 0 && reg_off + 4 <= bytes.len());
    let entry_count = u32::from_le_bytes(bytes[reg_off..reg_off + 4].try_into().unwrap());
    assert_eq!(entry_count, 1);
}

/// Mid-stream snapshot with floating dynamics (no contacts). (MidStreamNoContacts)
#[test]
fn mid_stream_no_contacts() {
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;

    for i in 0..4 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (i * 10) as f32,
            y: 50.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_sphere_shape(&mut world, body_id, &shape_def, &sphere);
    }

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for _ in 0..10 {
        world.step(time_step, sub_step_count);
    }

    let mut rec = Recording::new(0);
    world_start_recording(&mut world, &mut rec);

    for _ in 0..30 {
        world.step(time_step, sub_step_count);
    }

    world_stop_recording(&mut world);
    assert!(validate_replay(rec.data(), 1));
}

/// Mid-stream snapshot with contacts / warm starts. (MidStreamContacts)
#[test]
fn mid_stream_contacts() {
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    {
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        let ground_id = create_body(&mut world, &ground_def);
        let ground_box = make_box_hull(50.0, 1.0, 50.0);
        let ground_shape = default_shape_def();
        create_hull_shape(&mut world, ground_id, &ground_shape, &ground_box.base);
    }

    let mut dynamic_shape = default_shape_def();
    dynamic_shape.density = 1.0;

    for i in 0..3 {
        let box_hull = make_box_hull(0.5, 0.5, 0.5);
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (i as f32 * 2.0) - 2.0,
            y: 5.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &dynamic_shape, &box_hull.base);
    }

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for _ in 0..60 {
        world.step(time_step, sub_step_count);
    }

    let mut rec = Recording::new(0);
    world_start_recording(&mut world, &mut rec);

    for _ in 0..30 {
        world.step(time_step, sub_step_count);
    }

    world_stop_recording(&mut world);
    assert!(validate_replay(rec.data(), 1));
}

include!("recording_replay_tests_extra.rs");

