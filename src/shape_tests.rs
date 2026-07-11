//! Hull shape create/destroy and world hull-database sharing tests.
//! Ported from box3d-cpp-reference/test/test_world.c TestHullDatabase (subset).

use crate::body::{create_body, destroy_body};
use crate::hull::make_box_hull;
use crate::shape::{create_hull_shape, destroy_shape, shape_get_hull, shape_is_valid};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::World;
use std::rc::Rc;

#[test]
fn hull_database_sharing() {
    let mut world = World::new(&default_world_def());

    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let stack_ptr = &box_hull.base as *const _;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body_a = create_body(&mut world, &body_def);
    let body_b = create_body(&mut world, &body_def);

    let shape_def = default_shape_def();

    let shape_a = create_hull_shape(&mut world, body_a, &shape_def, &box_hull.base);
    let shape_b = create_hull_shape(&mut world, body_b, &shape_def, &box_hull.base);

    let ptr_a = shape_get_hull(&world, shape_a).unwrap() as *const _;
    let ptr_b = shape_get_hull(&world, shape_b).unwrap() as *const _;

    // Both shapes point at the single shared copy
    assert_eq!(ptr_a, ptr_b);
    // The shared copy is owned by the world, not the caller's stack hull
    assert_ne!(ptr_a, stack_ptr);
    assert_eq!(world.hull_database.len(), 1);

    // Independent stack box with same content deduplicates.
    let box2 = make_box_hull(0.5, 0.5, 0.5);
    let body_c = create_body(&mut world, &body_def);
    let shape_c = create_hull_shape(&mut world, body_c, &shape_def, &box2.base);
    let ptr_c = shape_get_hull(&world, shape_c).unwrap() as *const _;
    assert_eq!(ptr_c, ptr_a);
    destroy_shape(&mut world, shape_c, true);
    assert!(!shape_is_valid(&world, shape_c));

    // Releasing one reference keeps the other alive
    destroy_shape(&mut world, shape_a, true);
    let ptr_still_b = shape_get_hull(&world, shape_b).unwrap() as *const _;
    assert_eq!(ptr_still_b, ptr_b);
    assert_eq!(world.hull_database.len(), 1);

    destroy_shape(&mut world, shape_b, true);
    assert!(world.hull_database.is_empty());

    // Destroy body with attached shapes
    let body_d = create_body(&mut world, &body_def);
    let shape_d = create_hull_shape(&mut world, body_d, &shape_def, &box_hull.base);
    assert!(shape_is_valid(&world, shape_d));
    destroy_body(&mut world, body_d);
    assert!(!shape_is_valid(&world, shape_d));
    assert!(world.hull_database.is_empty());
}

#[test]
fn hull_create_updates_mass_and_proxy() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let shape = create_hull_shape(&mut world, body, &shape_def, &box_hull.base);

    let body_index = body.index1 - 1;
    assert!(world.bodies[body_index as usize].mass > 0.0);
    assert_eq!(world.bodies[body_index as usize].shape_count, 1);

    let raw = (shape.index1 - 1) as usize;
    assert!(world.shapes[raw].proxy_key != crate::core::NULL_INDEX);

    // Shape's Rc is also held by the database.
    if let crate::shape::ShapeGeometry::Hull(rc) = &world.shapes[raw].geometry {
        assert!(Rc::strong_count(rc) >= 2);
    } else {
        panic!("expected hull geometry");
    }
}
