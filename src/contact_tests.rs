//! Contact create/destroy and broad-phase pair update tests.

use crate::body::create_body;
use crate::broad_phase::update_broad_phase_pairs;
use crate::constraint_graph::{add_contact_to_graph, remove_contact_from_graph, OVERFLOW_INDEX};
use crate::contact::contact_flags;
use crate::contact::{can_collide, create_contact, destroy_contact};
use crate::core::NULL_INDEX;
use crate::geometry::{ShapeType, Sphere};
use crate::hull::make_box_hull;
use crate::island::{link_contact, unlink_contact};
use crate::manifold::{Manifold, ManifoldPoint};
use crate::math_functions::{Pos, VEC3_AXIS_Y, VEC3_ZERO};
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

#[test]
fn link_and_graph_dyn_static_contact() {
    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: 0.0 as _,
        y: 1.0 as _,
        z: 0.0 as _,
    };
    let ball = create_body(&mut world, &ball_def);

    let shape_def = default_shape_def();
    let box_hull = make_box_hull(5.0, 0.5, 5.0);
    let ground_shape = create_hull_shape(&mut world, ground, &shape_def, &box_hull.base);

    let mut ball_shape_def = default_shape_def();
    ball_shape_def.density = 1.0;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let ball_shape = create_sphere_shape(&mut world, ball, &ball_shape_def, &sphere);

    create_contact(
        &mut world,
        ground_shape.index1 - 1,
        ball_shape.index1 - 1,
        0,
    );
    let contact_id = world.solver_sets[AWAKE_SET as usize].contact_indices[0];

    // Simulate a touching manifold so link/graph can run without narrow phase.
    {
        let contact = &mut world.contacts[contact_id as usize];
        let mut manifold = Manifold::default();
        manifold.normal = VEC3_AXIS_Y;
        manifold.point_count = 1;
        manifold.points[0] = ManifoldPoint {
            anchor_a: VEC3_ZERO,
            anchor_b: VEC3_ZERO,
            separation: -0.01,
            ..ManifoldPoint::default()
        };
        contact.manifolds.push(manifold);
        contact.flags |= contact_flags::TOUCHING | contact_flags::SIM_TOUCHING;
    }

    link_contact(&mut world, contact_id);
    assert_ne!(world.contacts[contact_id as usize].island_id, NULL_INDEX);
    assert_eq!(
        world.contacts[contact_id as usize].island_id,
        world.bodies[ball.index1 as usize - 1].island_id
    );

    add_contact_to_graph(&mut world, contact_id);
    let color_index = world.contacts[contact_id as usize].color_index;
    let local_index = world.contacts[contact_id as usize].local_index;
    assert_ne!(color_index, NULL_INDEX);
    assert!(color_index < OVERFLOW_INDEX); // dyn-static prefers non-overflow colors

    let color = &world.constraint_graph.colors[color_index as usize];
    assert!(
        color.convex_contacts.contains(&contact_id)
            || color.contacts.iter().any(|s| s.contact_id == contact_id)
    );

    unlink_contact(&mut world, contact_id);
    assert_eq!(world.contacts[contact_id as usize].island_id, NULL_INDEX);

    remove_contact_from_graph(
        &mut world,
        ground.index1 - 1,
        ball.index1 - 1,
        color_index,
        local_index,
        false,
    );
    let color = &world.constraint_graph.colors[color_index as usize];
    assert!(!color.convex_contacts.contains(&contact_id));
    assert!(!color.contacts.iter().any(|s| s.contact_id == contact_id));
}
