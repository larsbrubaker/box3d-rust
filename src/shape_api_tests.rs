//! Shape filter and material public API tests.
//!
//! Covers b3Shape_Get/SetFilter and friction/restitution/surface/mesh materials.

use crate::body::create_body;
use crate::broad_phase::{proxy_id, proxy_type};
use crate::contact::create_contact;
use crate::core::NULL_INDEX;
use crate::geometry::{default_surface_material, Sphere};
use crate::math_functions::{Vec3, VEC3_ONE, VEC3_ZERO};
use crate::mesh::create_box_mesh;
use crate::shape::{
    create_mesh_shape, create_sphere_shape, shape_get_filter, shape_get_friction,
    shape_get_mesh_material_count, shape_get_mesh_surface_material, shape_get_restitution,
    shape_get_surface_material, shape_set_filter, shape_set_friction, shape_set_mesh_material,
    shape_set_restitution, shape_set_surface_material,
};
use crate::solver_set::AWAKE_SET;
use crate::types::{
    default_body_def, default_filter, default_shape_def, default_world_def, BodyType, Filter,
};
use crate::world::World;

fn make_dynamic_sphere(world: &mut World) -> (crate::id::BodyId, crate::id::ShapeId) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.invoke_contact_creation = false;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let shape = create_sphere_shape(world, body, &shape_def, &sphere);
    (body, shape)
}

#[test]
fn friction_restitution_round_trip() {
    let mut world = World::new(&default_world_def());
    let (_body, shape) = make_dynamic_sphere(&mut world);

    assert!((shape_get_friction(&world, shape) - 0.6).abs() < 1e-6);
    assert_eq!(shape_get_restitution(&world, shape), 0.0);

    shape_set_friction(&mut world, shape, 0.3);
    shape_set_restitution(&mut world, shape, 0.5);
    assert!((shape_get_friction(&world, shape) - 0.3).abs() < 1e-6);
    assert!((shape_get_restitution(&world, shape) - 0.5).abs() < 1e-6);
}

#[test]
fn surface_material_round_trip_includes_rolling() {
    let mut world = World::new(&default_world_def());
    let (_body, shape) = make_dynamic_sphere(&mut world);

    let mut mat = default_surface_material();
    mat.friction = 0.25;
    mat.restitution = 0.75;
    mat.rolling_resistance = 0.1;
    mat.tangent_velocity = Vec3::new(1.0, 0.0, 0.0);
    mat.user_material_id = 42;
    mat.custom_color = 0x00FF00;

    shape_set_surface_material(&mut world, shape, mat);
    let got = shape_get_surface_material(&world, shape);
    assert_eq!(got, mat);
    assert!((shape_get_friction(&world, shape) - 0.25).abs() < 1e-6);
    assert!((shape_get_restitution(&world, shape) - 0.75).abs() < 1e-6);
}

#[test]
fn mesh_materials_get_set_by_index() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    let body = create_body(&mut world, &body_def);

    let mesh = create_box_mesh(VEC3_ZERO, Vec3::new(1.0, 0.1, 1.0), false).expect("mesh");

    let mut mat_a = default_surface_material();
    mat_a.friction = 0.2;
    mat_a.user_material_id = 1;
    let mut mat_b = default_surface_material();
    mat_b.friction = 0.8;
    mat_b.rolling_resistance = 0.05;
    mat_b.user_material_id = 2;

    let mut shape_def = default_shape_def();
    shape_def.materials = vec![mat_a, mat_b];
    let shape = create_mesh_shape(&mut world, body, &shape_def, &mesh, VEC3_ONE);

    assert_eq!(shape_get_mesh_material_count(&world, shape), 2);
    assert_eq!(shape_get_mesh_surface_material(&world, shape, 0).user_material_id, 1);
    assert_eq!(shape_get_mesh_surface_material(&world, shape, 1).user_material_id, 2);

    let mut mat_b2 = mat_b;
    mat_b2.restitution = 0.9;
    shape_set_mesh_material(&mut world, shape, mat_b2, 1);
    assert!((shape_get_mesh_surface_material(&world, shape, 1).restitution - 0.9).abs() < 1e-6);
    // Index 0 (base) unchanged.
    assert_eq!(shape_get_mesh_surface_material(&world, shape, 0).user_material_id, 1);
}

#[test]
fn filter_get_set_without_invoke() {
    let mut world = World::new(&default_world_def());
    let (_body, shape) = make_dynamic_sphere(&mut world);

    let default = default_filter();
    assert_eq!(shape_get_filter(&world, shape), default);

    let filter = Filter {
        category_bits: 0x4,
        mask_bits: 0x8,
        group_index: -1,
    };
    shape_set_filter(&mut world, shape, filter, false);
    assert_eq!(shape_get_filter(&world, shape), filter);

    let raw = (shape.index1 - 1) as usize;
    assert!(world.shapes[raw].proxy_key != NULL_INDEX);
    assert!(!world.locked);
}

#[test]
fn filter_noop_when_unchanged() {
    let mut world = World::new(&default_world_def());
    let (_body, shape) = make_dynamic_sphere(&mut world);
    let raw = (shape.index1 - 1) as usize;
    let proxy_before = world.shapes[raw].proxy_key;

    let current = shape_get_filter(&world, shape);
    shape_set_filter(&mut world, shape, current, true);
    assert_eq!(world.shapes[raw].proxy_key, proxy_before);
    assert!(!world.locked);
}

#[test]
fn filter_invoke_destroys_contacts_and_rebuilds_proxy() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body_a = create_body(&mut world, &body_def);
    let body_b = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.invoke_contact_creation = false;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let shape_a = create_sphere_shape(&mut world, body_a, &shape_def, &sphere);
    let shape_b = create_sphere_shape(&mut world, body_b, &shape_def, &sphere);

    let id_a = shape_a.index1 - 1;
    let id_b = shape_b.index1 - 1;
    create_contact(&mut world, id_a, id_b, 0);
    assert_eq!(
        world.solver_sets[AWAKE_SET as usize].contact_indices.len(),
        1
    );
    assert_eq!(world.bodies[body_a.index1 as usize - 1].contact_count, 1);

    let proxy_before = world.shapes[id_a as usize].proxy_key;
    let filter = Filter {
        category_bits: 0x2,
        mask_bits: default_filter().mask_bits,
        group_index: 0,
    };
    shape_set_filter(&mut world, shape_a, filter, true);

    assert!(world.solver_sets[AWAKE_SET as usize]
        .contact_indices
        .is_empty());
    assert_eq!(world.bodies[body_a.index1 as usize - 1].contact_count, 0);
    assert_eq!(shape_get_filter(&world, shape_a), filter);
    // Proxy is rebuilt with the new category bits (may recycle the same proxy id).
    let proxy_after = world.shapes[id_a as usize].proxy_key;
    assert!(proxy_after != NULL_INDEX);
    let ptype = proxy_type(proxy_after);
    let pid = proxy_id(proxy_after);
    assert_eq!(
        world.broad_phase.trees[ptype as usize].category_bits(pid),
        0x2
    );
    let _ = (proxy_before, shape_b);
    assert!(!world.locked);
}
