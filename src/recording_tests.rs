//! World snapshot round-trip tests.
//!
//! Contract: snapshot → deserialize into shell → step N → hash equals
//! original world stepped N times.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::create_body;
use crate::geometry::Sphere;
use crate::hull::make_box_hull;
use crate::math_functions::{Pos, Vec3, VEC3_ZERO};
use crate::recording::{
    clone_world_via_snapshot, deserialize_into_shell, hash_world_state, serialize_world,
    GeometryRegistry, RecBuffer,
};
use crate::shape::{create_hull_shape, create_sphere_shape};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::World;

fn make_falling_spheres() -> World {
    let mut world = World::new(&default_world_def());
    world.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(50.0, 1.0, 50.0);
    let shape_def = default_shape_def();
    create_hull_shape(&mut world, ground, &shape_def, &ground_box.base);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };

    for i in 0..4 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: ((i as f32) * 2.0) as _,
            y: (5.0 + (i as f32)) as _,
            z: 0.0 as _,
        };
        let body = create_body(&mut world, &body_def);
        create_sphere_shape(&mut world, body, &shape_def, &sphere);
    }

    world
}

#[test]
fn snapshot_empty_world_roundtrip() {
    let world = World::new(&default_world_def());
    let mut registry = GeometryRegistry::new();
    let mut buf = RecBuffer::new();
    let n = serialize_world(&world, &mut buf, &mut registry);
    assert!(n > 16);

    let mut slots = registry.to_slots();
    let mut shell = World::new(&default_world_def());
    assert!(deserialize_into_shell(&buf.data, &mut shell, &mut slots));
    assert_eq!(hash_world_state(&world), hash_world_state(&shell));
}

#[test]
fn snapshot_midstream_no_contacts_step_hash() {
    let mut original = make_falling_spheres();
    for _ in 0..5 {
        original.step(1.0 / 60.0, 4);
    }

    let mut clone = clone_world_via_snapshot(&original).expect("snapshot restore");
    assert_eq!(
        hash_world_state(&original),
        hash_world_state(&clone),
        "hash must match immediately after restore"
    );

    for _ in 0..20 {
        original.step(1.0 / 60.0, 4);
        clone.step(1.0 / 60.0, 4);
    }

    assert_eq!(
        hash_world_state(&original),
        hash_world_state(&clone),
        "hash must match after stepping both worlds"
    );
}

#[test]
fn snapshot_midstream_with_contacts_step_hash() {
    let mut original = make_falling_spheres();
    for _ in 0..60 {
        original.step(1.0 / 60.0, 4);
    }

    let h0 = hash_world_state(&original);
    let mut clone = clone_world_via_snapshot(&original).expect("snapshot restore");
    assert_eq!(h0, hash_world_state(&clone));

    for _ in 0..30 {
        original.step(1.0 / 60.0, 4);
        clone.step(1.0 / 60.0, 4);
    }

    assert_eq!(
        hash_world_state(&original),
        hash_world_state(&clone),
        "warm-started contacts must survive snapshot"
    );
}

#[test]
fn snapshot_hull_geometry_interned() {
    let mut world = World::new(&default_world_def());
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 2.0 as _,
        z: 0.0 as _,
    };
    let body = create_body(&mut world, &body_def);
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
    create_hull_shape(&mut world, body, &shape_def, &box_hull.base);

    let mut registry = GeometryRegistry::new();
    let mut buf = RecBuffer::new();
    serialize_world(&world, &mut buf, &mut registry);
    assert_eq!(registry.entries.len(), 1);

    let mut clone = clone_world_via_snapshot(&world).expect("restore");
    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
        clone.step(1.0 / 60.0, 4);
    }
    assert_eq!(hash_world_state(&world), hash_world_state(&clone));
}
