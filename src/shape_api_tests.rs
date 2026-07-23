//! Shape filter, material, flag, name, density, geometry, and query API tests.
//!
//! Covers b3Shape_Get/SetFilter, friction/restitution/surface/mesh materials,
//! ShapeFlagsTest / ShapeNameTest from test_shape.c, plus density/AABB/userData
//! and SetSphere/SetCapsule/SetHull round-trips.

use crate::body::{body_get_mass, create_body};
use crate::broad_phase::{proxy_id, proxy_type};
use crate::contact::create_contact;
use crate::core::NULL_INDEX;
use crate::geometry::{default_surface_material, Capsule, ShapeType, Sphere};
use crate::hull::make_box_hull;
use crate::math_functions::{Vec3, POS_ZERO, VEC3_ONE, VEC3_ZERO};
use crate::mesh::create_box_mesh;
use crate::shape::{
    create_hull_shape, create_mesh_shape, create_sphere_shape, shape_are_contact_events_enabled,
    shape_are_hit_events_enabled, shape_are_pre_solve_events_enabled,
    shape_are_sensor_events_enabled, shape_enable_contact_events, shape_enable_hit_events,
    shape_enable_pre_solve_events, shape_enable_sensor_events, shape_get_aabb, shape_get_capsule,
    shape_get_closest_point, shape_get_density, shape_get_filter, shape_get_friction,
    shape_get_hull, shape_get_mesh_material_count, shape_get_mesh_surface_material, shape_get_name,
    shape_get_restitution, shape_get_sphere, shape_get_surface_material, shape_get_type,
    shape_get_user_data, shape_ray_cast, shape_set_capsule, shape_set_density, shape_set_filter,
    shape_set_friction, shape_set_hull, shape_set_mesh, shape_set_mesh_material, shape_set_name,
    shape_set_restitution, shape_set_sphere, shape_set_surface_material, shape_set_user_data,
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

fn check_shape_name(world: &World, shape_id: crate::id::ShapeId, expected: &str) {
    // Names are interned at full length via the name cache; an unset name reads
    // back as the empty string. (CheckShapeName)
    let got = shape_get_name(world, shape_id);
    assert_eq!(got, expected);
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
    assert_eq!(
        shape_get_mesh_surface_material(&world, shape, 0).user_material_id,
        1
    );
    assert_eq!(
        shape_get_mesh_surface_material(&world, shape, 1).user_material_id,
        2
    );

    let mut mat_b2 = mat_b;
    mat_b2.restitution = 0.9;
    shape_set_mesh_material(&mut world, shape, mat_b2, 1);
    assert!((shape_get_mesh_surface_material(&world, shape, 1).restitution - 0.9).abs() < 1e-6);
    // Index 0 (base) unchanged.
    assert_eq!(
        shape_get_mesh_surface_material(&world, shape, 0).user_material_id,
        1
    );
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

/// ShapeFlagsTest from test_shape.c — each enable bit is independent.
#[test]
fn shape_flags_test() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_sensor_events = true;
    shape_def.enable_contact_events = true;
    shape_def.enable_hit_events = true;
    shape_def.enable_pre_solve_events = true;
    let shape = create_sphere_shape(&mut world, body, &shape_def, &sphere);

    assert!(shape_are_sensor_events_enabled(&world, shape));
    assert!(shape_are_contact_events_enabled(&world, shape));
    assert!(shape_are_hit_events_enabled(&world, shape));
    assert!(shape_are_pre_solve_events_enabled(&world, shape));

    shape_enable_sensor_events(&mut world, shape, false);
    assert!(!shape_are_sensor_events_enabled(&world, shape));
    assert!(shape_are_contact_events_enabled(&world, shape));
    assert!(shape_are_hit_events_enabled(&world, shape));
    assert!(shape_are_pre_solve_events_enabled(&world, shape));

    shape_enable_sensor_events(&mut world, shape, true);
    shape_enable_hit_events(&mut world, shape, false);
    assert!(shape_are_sensor_events_enabled(&world, shape));
    assert!(shape_are_contact_events_enabled(&world, shape));
    assert!(!shape_are_hit_events_enabled(&world, shape));
    assert!(shape_are_pre_solve_events_enabled(&world, shape));

    shape_enable_contact_events(&mut world, shape, false);
    shape_enable_pre_solve_events(&mut world, shape, false);
    assert!(shape_are_sensor_events_enabled(&world, shape));
    assert!(!shape_are_contact_events_enabled(&world, shape));
    assert!(!shape_are_hit_events_enabled(&world, shape));
    assert!(!shape_are_pre_solve_events_enabled(&world, shape));
}

/// ShapeNameTest from test_shape.c — def path, setter, long name, clear.
#[test]
fn shape_name_test() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let shape = create_sphere_shape(&mut world, body, &shape_def, &sphere);
    check_shape_name(&world, shape, "");

    shape_def.name = "box".to_string();
    let named = create_sphere_shape(&mut world, body, &shape_def, &sphere);
    check_shape_name(&world, named, "box");

    shape_set_name(&mut world, shape, "wheel");
    check_shape_name(&world, shape, "wheel");

    shape_set_name(&mut world, shape, "abcdefghijklmnopqrstuvwxyz");
    check_shape_name(&world, shape, "abcdefghijklmnopqrstuvwxyz");

    shape_set_name(&mut world, shape, "");
    check_shape_name(&world, shape, "");
}

#[test]
fn density_user_data_aabb_round_trip() {
    let mut world = World::new(&default_world_def());
    let (body, shape) = make_dynamic_sphere(&mut world);

    assert!((shape_get_density(&world, shape) - 1.0).abs() < 1e-6);
    let mass_before = body_get_mass(&world, body);

    shape_set_density(&mut world, shape, 3.0, true);
    assert!((shape_get_density(&world, shape) - 3.0).abs() < 1e-6);
    let mass_after = body_get_mass(&world, body);
    assert!((mass_after - 3.0 * mass_before).abs() < 1e-4);

    // Same density is a no-op.
    shape_set_density(&mut world, shape, 3.0, true);

    shape_set_user_data(&mut world, shape, 0xDEAD_BEEF);
    assert_eq!(shape_get_user_data(&world, shape), 0xDEAD_BEEF);

    let aabb = shape_get_aabb(&world, shape);
    assert!(aabb.upper_bound.x > aabb.lower_bound.x);
}

#[test]
fn set_sphere_capsule_hull_round_trip() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.invoke_contact_creation = false;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let shape = create_sphere_shape(&mut world, body, &shape_def, &sphere);
    assert_eq!(shape_get_sphere(&world, shape).radius, 0.5);

    let new_sphere = Sphere {
        center: Vec3::new(0.1, 0.0, 0.0),
        radius: 1.25,
    };
    shape_set_sphere(&mut world, shape, &new_sphere);
    assert_eq!(shape_get_sphere(&world, shape), new_sphere);

    let capsule = Capsule {
        center1: Vec3::new(0.0, -0.5, 0.0),
        center2: Vec3::new(0.0, 0.5, 0.0),
        radius: 0.25,
    };
    shape_set_capsule(&mut world, shape, &capsule);
    assert_eq!(shape_get_capsule(&world, shape), capsule);

    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    shape_set_hull(&mut world, shape, &box_hull.base);
    let got_hash = shape_get_hull(&world, shape).expect("hull").hash;
    assert_eq!(got_hash, box_hull.base.hash);
    assert_eq!(world.hull_database.len(), 1);

    // Same shared content is a no-op (no extra database entry).
    shape_set_hull(&mut world, shape, &box_hull.base);
    assert_eq!(world.hull_database.len(), 1);
}

#[test]
fn shape_ray_cast_and_closest_point() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    let body = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.invoke_contact_creation = false;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };
    let shape = create_sphere_shape(&mut world, body, &shape_def, &sphere);

    let origin = crate::math_functions::offset_pos(POS_ZERO, Vec3::new(-4.0, 0.0, 0.0));
    let translation = Vec3::new(8.0, 0.0, 0.0);
    let hit = shape_ray_cast(&world, shape, origin, translation);
    assert!(hit.hit);
    assert!((hit.fraction - 0.375).abs() < 1e-5);
    assert!((hit.normal.x + 1.0).abs() < 1e-5);

    let closest = shape_get_closest_point(&world, shape, Vec3::new(3.0, 0.0, 0.0));
    assert!((closest.x - 1.0).abs() < 1e-4);
    assert!(closest.y.abs() < 1e-4);
    assert!(closest.z.abs() < 1e-4);
}

#[test]
fn set_hull_from_sphere_rebuilds_proxy() {
    let mut world = World::new(&default_world_def());
    let (_body, shape) = make_dynamic_sphere(&mut world);
    let raw = (shape.index1 - 1) as usize;

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    shape_set_hull(&mut world, shape, &box_hull.base);
    assert!(shape_get_hull(&world, shape).is_some());
    assert!(world.shapes[raw].proxy_key != NULL_INDEX);
}

#[test]
fn set_mesh_retypes_and_rebuilds_proxy() {
    // b3Shape_SetMesh retypes a live shape to a mesh, swapping geometry + scale,
    // then rebuilds the broad-phase proxy (sample_collision.cpp MeshScale slider).
    let mut world = World::new(&default_world_def());
    let (_body, shape) = make_dynamic_sphere(&mut world);
    let raw = (shape.index1 - 1) as usize;
    assert_eq!(shape_get_type(&world, shape), ShapeType::Sphere);

    // A wide, thin box mesh — clearly larger in x/z than the 0.5 sphere.
    let mesh = create_box_mesh(VEC3_ZERO, Vec3::new(2.0, 0.1, 2.0), false).expect("mesh");
    let scale = Vec3::new(1.5, 1.0, 1.5);
    shape_set_mesh(&mut world, shape, &mesh, scale);

    assert_eq!(shape_get_type(&world, shape), ShapeType::Mesh);
    match &world.shapes[raw].geometry {
        crate::shape::ShapeGeometry::Mesh { data, scale: s } => {
            assert_eq!(data.hash, mesh.hash);
            assert_eq!(*s, scale);
        }
        other => panic!("expected mesh geometry, got {other:?}"),
    }

    // Proxy rebuilt and the AABB now spans the scaled mesh (x half-extent 2*1.5=3).
    assert!(world.shapes[raw].proxy_key != NULL_INDEX);
    let aabb = shape_get_aabb(&world, shape);
    assert!(
        aabb.upper_bound.x >= 3.0 && aabb.lower_bound.x <= -3.0,
        "mesh AABB should span the scaled extent, got {aabb:?}"
    );
    assert!(!world.locked);
}

#[test]
fn hull_set_shares_database_entry() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body_a = create_body(&mut world, &body_def);
    let body_b = create_body(&mut world, &body_def);

    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let shape_def = default_shape_def();
    let shape_a = create_hull_shape(&mut world, body_a, &shape_def, &box_hull.base);
    let shape_b = create_hull_shape(&mut world, body_b, &shape_def, &box_hull.base);
    assert_eq!(world.hull_database.len(), 1);

    let got_a = shape_get_hull(&world, shape_a).unwrap() as *const _;
    // Re-set from the stack hull; content match keeps the shared database entry.
    shape_set_hull(&mut world, shape_b, &box_hull.base);
    let got_b = shape_get_hull(&world, shape_b).unwrap() as *const _;
    assert_eq!(got_a, got_b);
    assert_eq!(world.hull_database.len(), 1);
}

#[test]
fn apply_wind_pushes_dynamic_sphere() {
    use crate::body::body_get_linear_velocity;
    use crate::shape::shape_apply_wind;

    let mut world = World::new(&default_world_def());
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.gravity_scale = 0.0;
    let body = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let shape = create_sphere_shape(&mut world, body, &shape_def, &sphere);

    shape_apply_wind(
        &mut world,
        shape,
        Vec3 {
            x: 50.0,
            y: 0.0,
            z: 0.0,
        },
        1.0,
        0.0,
        100.0,
        true,
    );
    world.step(1.0 / 60.0, 4);
    let velocity = body_get_linear_velocity(&world, body);
    assert!(velocity.x > 0.0, "wind pushes the body along +x");
}

#[test]
fn shape_contact_and_mass_introspection() {
    use crate::shape::{
        shape_compute_mass_data, shape_get_contact_capacity, shape_get_contact_data,
    };

    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let box_hull = make_box_hull(5.0, 0.5, 5.0);
    create_hull_shape(&mut world, ground, &default_shape_def(), &box_hull.base);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = crate::math_functions::Pos {
        x: 0.0 as _,
        y: 1.0 as _,
        z: 0.0 as _,
    };
    let body = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let shape = create_sphere_shape(
        &mut world,
        body,
        &shape_def,
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
    );

    let mass = shape_compute_mass_data(&world, shape);
    assert!(mass.mass > 0.0);

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }

    let capacity = shape_get_contact_capacity(&world, shape);
    let data = shape_get_contact_data(&world, shape, 8);
    assert!(data.len() as i32 <= capacity);
    assert!(!data.is_empty(), "resting sphere should touch ground");
    assert!(!data[0].manifolds.is_empty());
}

#[test]
fn sensor_data_accessors() {
    use crate::shape::{
        shape_get_contact_capacity, shape_get_sensor_capacity, shape_get_sensor_data,
        shape_get_sensor_overlaps, shape_is_sensor, shape_is_valid,
    };

    let mut world_def = default_world_def();
    world_def.gravity = VEC3_ZERO;
    let mut world = World::new(&world_def);

    let sensor_body = create_body(&mut world, &default_body_def());
    let mut sensor_shape_def = default_shape_def();
    sensor_shape_def.is_sensor = true;
    sensor_shape_def.enable_sensor_events = true;
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    let sensor_shape =
        create_hull_shape(&mut world, sensor_body, &sensor_shape_def, &box_hull.base);
    assert!(shape_is_sensor(&world, sensor_shape));
    assert_eq!(shape_get_contact_capacity(&world, sensor_shape), 0);

    let mut visitor_def = default_body_def();
    visitor_def.type_ = BodyType::Dynamic;
    visitor_def.gravity_scale = 0.0;
    visitor_def.position = crate::math_functions::Pos {
        x: 0.5 as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };
    let visitor_body = create_body(&mut world, &visitor_def);
    let mut visitor_shape_def = default_shape_def();
    visitor_shape_def.enable_sensor_events = true;
    let visitor_shape = create_hull_shape(
        &mut world,
        visitor_body,
        &visitor_shape_def,
        &make_box_hull(0.25, 0.25, 0.25).base,
    );

    world.step(1.0 / 60.0, 4);

    assert_eq!(shape_get_sensor_capacity(&world, sensor_shape), 1);
    let visitors = shape_get_sensor_data(&world, sensor_shape, 8);
    assert_eq!(visitors.len(), 1);
    assert_eq!(visitors[0], visitor_shape);
    assert!(shape_is_valid(&world, visitors[0]));
    assert_eq!(shape_get_sensor_overlaps(&world, sensor_shape, 8), visitors);
}
