//! Hull / mesh / height-field / compound shape create/destroy tests.
//! Ported from box3d-cpp-reference/test/test_world.c (TestHullDatabase + shape attach).

use crate::body::{create_body, destroy_body};
use crate::compound::{create_compound, CompoundDef, CompoundHullDef};
use crate::core::NULL_INDEX;
use crate::geometry::{default_surface_material, ShapeType};
use crate::height_field::create_grid;
use crate::hull::make_box_hull;
use crate::math_functions::{Transform, Vec3, QUAT_IDENTITY, VEC3_ONE, VEC3_ZERO};
use crate::mesh::create_box_mesh;
use crate::shape::{
    create_baked_compound_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    destroy_shape, shape_get_hull, shape_is_valid, shape_set_hull, ShapeGeometry,
};
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

    assert_eq!(ptr_a, ptr_b);
    assert_ne!(ptr_a, stack_ptr);
    assert_eq!(world.hull_database.len(), 1);

    let box2 = make_box_hull(0.5, 0.5, 0.5);
    let body_c = create_body(&mut world, &body_def);
    let shape_c = create_hull_shape(&mut world, body_c, &shape_def, &box2.base);
    let ptr_c = shape_get_hull(&world, shape_c).unwrap() as *const _;
    assert_eq!(ptr_c, ptr_a);
    destroy_shape(&mut world, shape_c, true);
    assert!(!shape_is_valid(&world, shape_c));

    // Setting a shape's hull to its own sole shared copy must not free it mid update.
    let box3 = make_box_hull(0.3, 0.3, 0.3);
    let body_d = create_body(&mut world, &body_def);
    let shape_d = create_hull_shape(&mut world, body_d, &shape_def, &box3.base);
    let got_d = shape_get_hull(&world, shape_d).unwrap() as *const _;
    {
        let shared_rc = match &world.shapes[(shape_d.index1 - 1) as usize].geometry {
            ShapeGeometry::Hull(rc) => Rc::clone(rc),
            _ => panic!("expected hull"),
        };
        shape_set_hull(&mut world, shape_d, shared_rc.as_ref());
        let still_d = shape_get_hull(&world, shape_d).unwrap() as *const _;
        assert_eq!(still_d, got_d);
        // Drop the temporary Rc so destroy_shape's release sees strong_count == 2.
    }
    destroy_shape(&mut world, shape_d, true);

    destroy_shape(&mut world, shape_a, true);
    let ptr_still_b = shape_get_hull(&world, shape_b).unwrap() as *const _;
    assert_eq!(ptr_still_b, ptr_b);
    assert_eq!(world.hull_database.len(), 1);

    destroy_shape(&mut world, shape_b, true);
    assert!(world.hull_database.is_empty());

    let body_e = create_body(&mut world, &body_def);
    let shape_e = create_hull_shape(&mut world, body_e, &shape_def, &box_hull.base);
    assert!(shape_is_valid(&world, shape_e));
    destroy_body(&mut world, body_e);
    assert!(!shape_is_valid(&world, shape_e));
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
    assert!(world.shapes[raw].proxy_key != NULL_INDEX);

    if let ShapeGeometry::Hull(rc) = &world.shapes[raw].geometry {
        assert!(Rc::strong_count(rc) >= 2);
    } else {
        panic!("expected hull geometry");
    }
}

#[test]
fn mesh_shape_attach() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    let body = create_body(&mut world, &body_def);

    let mesh = create_box_mesh(VEC3_ZERO, Vec3::new(1.0, 0.1, 1.0), false).expect("mesh");
    assert!(mesh.hash != 0);

    let mut shape_def = default_shape_def();
    shape_def.filter.category_bits = 1;
    let shape = create_mesh_shape(&mut world, body, &shape_def, &mesh, VEC3_ONE);
    assert!(shape_is_valid(&world, shape));

    let raw = (shape.index1 - 1) as usize;
    assert_eq!(world.shapes[raw].shape_type(), ShapeType::Mesh);
    assert!(world.shapes[raw].proxy_key != NULL_INDEX);
    match &world.shapes[raw].geometry {
        ShapeGeometry::Mesh { scale, .. } => assert_eq!(scale, &VEC3_ONE),
        _ => panic!("expected mesh geometry"),
    }

    destroy_shape(&mut world, shape, true);
    assert!(!shape_is_valid(&world, shape));
}

#[test]
fn height_field_shape_attach_static_only() {
    let mut world = World::new(&default_world_def());
    let hf = create_grid(4, 4, Vec3::new(1.0, 1.0, 1.0), false);
    assert!(hf.hash != 0);

    let mut dyn_def = default_body_def();
    dyn_def.type_ = BodyType::Dynamic;
    let dyn_body = create_body(&mut world, &dyn_def);
    let rejected = create_height_field_shape(&mut world, dyn_body, &default_shape_def(), &hf);
    assert_eq!(rejected.index1, 0);

    let mut static_def = default_body_def();
    static_def.type_ = BodyType::Static;
    let static_body = create_body(&mut world, &static_def);
    let shape = create_height_field_shape(&mut world, static_body, &default_shape_def(), &hf);
    assert!(shape_is_valid(&world, shape));
    let raw = (shape.index1 - 1) as usize;
    assert_eq!(world.shapes[raw].shape_type(), ShapeType::Height);
    assert!(world.shapes[raw].proxy_key != NULL_INDEX);
}

#[test]
fn compound_shape_copies_materials() {
    let mut world = World::new(&default_world_def());

    let box_a = make_box_hull(1.0, 1.0, 1.0);
    let box_b = make_box_hull(1.0, 1.0, 1.0);
    let mut mat_a = default_surface_material();
    mat_a.user_material_id = 11;
    let mut mat_b = default_surface_material();
    mat_b.user_material_id = 22;

    let hulls = [
        CompoundHullDef {
            hull: &box_a.base,
            transform: Transform {
                p: Vec3::new(-3.0, 0.0, 0.0),
                q: QUAT_IDENTITY,
            },
            material: mat_a,
        },
        CompoundHullDef {
            hull: &box_b.base,
            transform: Transform {
                p: Vec3::new(3.0, 0.0, 0.0),
                q: QUAT_IDENTITY,
            },
            material: mat_b,
        },
    ];
    let compound = create_compound(&CompoundDef {
        hulls: &hulls,
        ..Default::default()
    })
    .expect("compound");

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    let body = create_body(&mut world, &body_def);
    let shape = create_baked_compound_shape(&mut world, body, &default_shape_def(), &compound);
    assert!(shape_is_valid(&world, shape));

    let raw = (shape.index1 - 1) as usize;
    assert_eq!(world.shapes[raw].shape_type(), ShapeType::Compound);
    assert_eq!(world.shapes[raw].material_count(), 2);
    assert_eq!(world.shapes[raw].get_material(0).user_material_id, 11);
    assert_eq!(world.shapes[raw].get_material(1).user_material_id, 22);
    assert_eq!(world.shapes[raw].get_shape_user_material_id(0, 0), 11);
    assert_eq!(world.shapes[raw].get_shape_user_material_id(1, 0), 22);
}
