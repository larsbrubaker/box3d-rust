//! Contact create/destroy and broad-phase pair update tests.

use crate::body::create_body;
use crate::broad_phase::update_broad_phase_pairs;
use crate::contact::{can_collide, create_contact, destroy_contact};
use crate::core::NULL_INDEX;
use crate::geometry::{ShapeType, Sphere};
use crate::hull::make_box_hull;
use crate::shape::{create_hull_shape, create_sphere_shape, destroy_shape};
use crate::solver_set::AWAKE_SET;
use crate::table::shape_pair_key;
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::World;

#[test]
fn contact_registers_cover_convex_pairs() {
    assert!(can_collide(ShapeType::Sphere, ShapeType::Sphere));
    assert!(can_collide(ShapeType::Hull, ShapeType::Sphere));
    assert!(can_collide(ShapeType::Sphere, ShapeType::Hull));
    assert!(!can_collide(ShapeType::Mesh, ShapeType::Mesh));
    assert!(!can_collide(ShapeType::Height, ShapeType::Height));
}

#[test]
fn create_destroy_contact_links_bodies() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body_a = create_body(&mut world, &body_def);
    let body_b = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.invoke_contact_creation = false;

    let sphere = Sphere {
        center: crate::math_functions::VEC3_ZERO,
        radius: 0.5,
    };
    let shape_a = create_sphere_shape(&mut world, body_a, &shape_def, &sphere);
    let shape_b = create_sphere_shape(&mut world, body_b, &shape_def, &sphere);

    let id_a = shape_a.index1 - 1;
    let id_b = shape_b.index1 - 1;
    create_contact(&mut world, id_a, id_b, 0);

    assert_eq!(world.solver_sets[AWAKE_SET as usize].contact_indices.len(), 1);
    assert_eq!(world.bodies[body_a.index1 as usize - 1].contact_count, 1);
    assert_eq!(world.bodies[body_b.index1 as usize - 1].contact_count, 1);
    assert!(world
        .broad_phase
        .pair_set
        .contains_key(shape_pair_key(id_a, id_b, 0)));

    let contact_id = world.solver_sets[AWAKE_SET as usize].contact_indices[0];
    destroy_contact(&mut world, contact_id, false);

    assert!(world.solver_sets[AWAKE_SET as usize].contact_indices.is_empty());
    assert_eq!(world.bodies[body_a.index1 as usize - 1].contact_count, 0);
    assert_eq!(world.bodies[body_b.index1 as usize - 1].contact_count, 0);
    assert!(!world
        .broad_phase
        .pair_set
        .contains_key(shape_pair_key(id_a, id_b, 0)));
    assert_eq!(world.contacts[contact_id as usize].contact_id, NULL_INDEX);
}

#[test]
fn update_pairs_creates_contact_for_overlap() {
    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = crate::math_functions::Pos {
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
        center: crate::math_functions::VEC3_ZERO,
        radius: 0.5,
    };
    create_sphere_shape(&mut world, ball, &ball_shape, &sphere);

    // Shape create with invoke_contact_creation buffered moves; update pairs.
    update_broad_phase_pairs(&mut world);

    assert_eq!(world.solver_sets[AWAKE_SET as usize].contact_indices.len(), 1);
    assert!(world.broad_phase.move_array.is_empty());

    // Destroying the ball shape removes the contact.
    let shape_id = {
        let body = &world.bodies[ball.index1 as usize - 1];
        body.head_shape_id
    };
    let shape = crate::id::ShapeId {
        index1: shape_id + 1,
        world0: world.world_id,
        generation: world.shapes[shape_id as usize].generation,
    };
    destroy_shape(&mut world, shape, true);
    assert!(world.solver_sets[AWAKE_SET as usize].contact_indices.is_empty());
}
