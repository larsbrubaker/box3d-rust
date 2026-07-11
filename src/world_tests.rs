//! World step tests. EmptyWorld and HelloWorld from test_world.c.

use crate::body::{body_get_position, create_body, get_body_transform_quick};
use crate::contact::contact_flags;
use crate::core::NULL_INDEX;
use crate::geometry::Sphere;
use crate::hull::{make_box_hull, make_cube_hull};
use crate::math_functions::{Pos, VEC3_ZERO};
use crate::shape::{create_hull_shape, create_sphere_shape};
use crate::solver_set::AWAKE_SET;
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::World;

/// (EmptyWorld)
#[test]
fn empty_world() {
    let mut world = World::new(&default_world_def());
    let time_step = 1.0 / 60.0;
    let sub_step_count = 1;

    for _ in 0..60 {
        world.step(time_step, sub_step_count);
    }

    assert_eq!(world.step_index, 60);
}

/// Collide pass promotes an overlapping non-touching contact into the graph.
#[test]
fn step_collide_marks_overlapping_contact_touching() {
    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: 0.0 as _,
        y: 0.5 as _,
        z: 0.0 as _,
    };
    let ball = create_body(&mut world, &ball_def);

    let shape_def = default_shape_def();
    let box_hull = make_box_hull(5.0, 0.5, 5.0);
    create_hull_shape(&mut world, ground, &shape_def, &box_hull.base);

    let mut ball_shape = default_shape_def();
    ball_shape.density = 1.0;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    create_sphere_shape(&mut world, ball, &ball_shape, &sphere);

    world.step(1.0 / 60.0, 1);

    assert_eq!(world.solver_sets[AWAKE_SET as usize].contact_indices.len(), 0);
    let mut found_touching = false;
    for contact in &world.contacts {
        if contact.contact_id == NULL_INDEX {
            continue;
        }
        if (contact.flags & contact_flags::TOUCHING) != 0 {
            found_touching = true;
            assert_ne!(contact.color_index, NULL_INDEX);
            assert_ne!(contact.island_id, NULL_INDEX);
            assert!(!contact.manifolds.is_empty());
        }
    }
    assert!(found_touching);
}

/// (HelloWorld)
#[test]
fn hello_world() {
    let mut world_def = default_world_def();
    world_def.gravity = crate::math_functions::Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    let mut world = World::new(&world_def);

    let mut ground_body_def = default_body_def();
    ground_body_def.position = Pos {
        x: 0.0 as _,
        y: -10.0 as _,
        z: 0.0 as _,
    };
    let ground_id = create_body(&mut world, &ground_body_def);

    let ground_box = make_box_hull(50.0, 10.0, 50.0);
    let ground_shape_def = default_shape_def();
    create_hull_shape(&mut world, ground_id, &ground_shape_def, &ground_box.base);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 4.0 as _,
        z: 0.0 as _,
    };
    let body_id = create_body(&mut world, &body_def);

    let dynamic_box = make_cube_hull(1.0);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.base_material.friction = 0.3;
    create_hull_shape(&mut world, body_id, &shape_def, &dynamic_box.base);

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;

    for _ in 0..90 {
        world.step(time_step, sub_step_count);
    }

    let position = body_get_position(&world, body_id);
    let body_index = crate::body::get_body_full_id(&world, body_id);
    let rotation = get_body_transform_quick(&world, &world.bodies[body_index as usize]).q;

    assert!(
        (position.y as f32 - 1.0).abs() < 0.01,
        "expected y ≈ 1.0, got {}",
        position.y
    );
    assert!(
        rotation.v.x.abs() < 0.01,
        "expected rotation.v.x ≈ 0, got {}",
        rotation.v.x
    );
    assert!(
        rotation.v.z.abs() < 0.01,
        "expected rotation.v.z ≈ 0, got {}",
        rotation.v.z
    );
}
