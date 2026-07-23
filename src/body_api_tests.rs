//! Body public API tests for getters, forces, transform, and awake.
//!
//! Ported patterns from box3d-cpp-reference/test/test_body.c.

use crate::body::{
    body_apply_force, body_apply_force_to_center, body_apply_linear_impulse,
    body_apply_linear_impulse_to_center, body_apply_torque, body_disable, body_enable,
    body_enable_contact_recycling, body_get_local_point, body_get_local_point_velocity,
    body_get_local_vector, body_get_mass, body_get_motion_locks, body_get_name, body_get_type,
    body_get_user_data, body_get_world_point, body_get_world_point_velocity, body_get_world_vector,
    body_is_awake, body_is_contact_recycling_enabled, body_is_enabled, body_set_awake,
    body_set_motion_locks, body_set_name, body_set_transform, body_set_type, body_set_user_data,
    body_sim, create_body, get_body_full_id,
};
use crate::geometry::Sphere;
use crate::math_functions::{
    make_quat_from_axis_angle, Pos, Vec3, POS_ZERO, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ZERO,
};
use crate::shape::create_sphere_shape;
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType, MotionLocks};
use crate::world::World;

#[test]
fn motion_locks_round_trip() {
    let mut world = World::new(&default_world_def());
    let mut def = default_body_def();
    def.type_ = BodyType::Dynamic;
    let body_id = create_body(&mut world, &def);

    let locks = MotionLocks {
        linear_x: true,
        linear_y: false,
        linear_z: true,
        angular_x: false,
        angular_y: true,
        angular_z: false,
    };
    body_set_motion_locks(&mut world, body_id, locks);
    assert_eq!(body_get_motion_locks(&world, body_id), locks);
}

#[test]
fn name_and_user_data() {
    let mut world = World::new(&default_world_def());
    let body_id = create_body(&mut world, &default_body_def());

    assert_eq!(body_get_name(&world, body_id), "");
    body_set_name(&mut world, body_id, "hinge_a");
    assert_eq!(body_get_name(&world, body_id), "hinge_a");

    // Names are interned at full length via the name cache (no truncation).
    body_set_name(&mut world, body_id, "abcdefghijklmnopqrstuvwxyz");
    assert_eq!(body_get_name(&world, body_id), "abcdefghijklmnopqrstuvwxyz");

    body_set_user_data(&mut world, body_id, 0xCAFE_BABE);
    assert_eq!(body_get_user_data(&world, body_id), 0xCAFE_BABE);
}

#[test]
fn local_world_point_vector() {
    let mut world = World::new(&default_world_def());
    let mut def = default_body_def();
    def.position = Pos {
        x: 1.0 as _,
        y: 2.0 as _,
        z: 3.0 as _,
    };
    def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Y, 0.0);
    let body_id = create_body(&mut world, &def);

    let local = Vec3 {
        x: 0.5,
        y: -0.25,
        z: 1.0,
    };
    let world_point = body_get_world_point(&world, body_id, local);
    let back = body_get_local_point(&world, body_id, world_point);
    assert!((back.x - local.x).abs() < 1e-5);
    assert!((back.y - local.y).abs() < 1e-5);
    assert!((back.z - local.z).abs() < 1e-5);

    let local_v = Vec3 {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };
    let world_v = body_get_world_vector(&world, body_id, local_v);
    let back_v = body_get_local_vector(&world, body_id, world_v);
    assert!((back_v.x - local_v.x).abs() < 1e-5);
    assert!((back_v.y - local_v.y).abs() < 1e-5);
    assert!((back_v.z - local_v.z).abs() < 1e-5);
}

#[test]
fn point_velocity_and_forces() {
    let mut world = World::new(&default_world_def());
    let mut def = default_body_def();
    def.type_ = BodyType::Dynamic;
    def.linear_velocity = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    def.angular_velocity = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 2.0,
    };
    let body_id = create_body(&mut world, &def);
    create_sphere_shape(
        &mut world,
        body_id,
        &default_shape_def(),
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
    );

    assert_eq!(body_get_type(&world, body_id), BodyType::Dynamic);
    assert!(body_is_awake(&world, body_id));
    assert!(body_is_enabled(&world, body_id));

    let v_at_origin = body_get_local_point_velocity(&world, body_id, VEC3_ZERO);
    assert!((v_at_origin.x - 1.0).abs() < 1e-5);

    let world_v = body_get_world_point_velocity(&world, body_id, POS_ZERO);
    assert!((world_v.x - 1.0).abs() < 1e-5);

    body_apply_force_to_center(
        &mut world,
        body_id,
        Vec3 {
            x: 0.0,
            y: 10.0,
            z: 0.0,
        },
        true,
    );
    assert!((body_sim(&world, body_id).force.y - 10.0).abs() < 1e-5);

    body_apply_torque(
        &mut world,
        body_id,
        Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        true,
    );
    assert!((body_sim(&world, body_id).torque.x - 1.0).abs() < 1e-5);

    body_apply_force(
        &mut world,
        body_id,
        Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        },
        Pos {
            x: 0.0 as _,
            y: 1.0 as _,
            z: 0.0 as _,
        },
        true,
    );
    assert!((body_sim(&world, body_id).force.x - 2.0).abs() < 1e-5);

    body_apply_linear_impulse_to_center(
        &mut world,
        body_id,
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.5,
        },
        true,
    );
    body_apply_linear_impulse(
        &mut world,
        body_id,
        Vec3 {
            x: 0.1,
            y: 0.0,
            z: 0.0,
        },
        POS_ZERO,
        true,
    );
}

#[test]
fn set_transform_and_awake() {
    let mut world = World::new(&default_world_def());
    let mut def = default_body_def();
    def.type_ = BodyType::Dynamic;
    let body_id = create_body(&mut world, &def);
    create_sphere_shape(
        &mut world,
        body_id,
        &default_shape_def(),
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.25,
        },
    );

    let new_pos = Pos {
        x: 3.0 as _,
        y: 4.0 as _,
        z: 5.0 as _,
    };
    body_set_transform(&mut world, body_id, new_pos, QUAT_IDENTITY);
    let xf = body_sim(&world, body_id).transform;
    assert_eq!(xf.p, new_pos);

    body_set_awake(&mut world, body_id, false);
    assert!(!body_is_awake(&world, body_id));

    body_set_awake(&mut world, body_id, true);
    assert!(body_is_awake(&world, body_id));
}

#[test]
fn set_type_dynamic_static_round_trip() {
    use crate::core::NULL_INDEX;
    use crate::solver_set::{AWAKE_SET, STATIC_SET};

    let mut world = World::new(&default_world_def());
    let mut def = default_body_def();
    def.type_ = BodyType::Dynamic;
    let body_id = create_body(&mut world, &def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let shape_id = create_sphere_shape(
        &mut world,
        body_id,
        &shape_def,
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
    );

    let body_index = get_body_full_id(&world, body_id);
    let shape_index = (shape_id.index1 - 1) as usize;
    assert_eq!(body_get_type(&world, body_id), BodyType::Dynamic);
    assert_eq!(world.bodies[body_index as usize].set_index, AWAKE_SET);
    assert!(body_get_mass(&world, body_id) > 0.0);
    assert!(world.shapes[shape_index].proxy_key != NULL_INDEX);

    body_set_type(&mut world, body_id, BodyType::Static);
    assert!(!world.locked);
    assert_eq!(body_get_type(&world, body_id), BodyType::Static);
    assert_eq!(world.bodies[body_index as usize].set_index, STATIC_SET);
    assert_eq!(world.bodies[body_index as usize].island_id, NULL_INDEX);
    assert_eq!(body_get_mass(&world, body_id), 0.0);
    assert!(world.shapes[shape_index].proxy_key != NULL_INDEX);

    body_set_type(&mut world, body_id, BodyType::Dynamic);
    assert!(!world.locked);
    assert_eq!(body_get_type(&world, body_id), BodyType::Dynamic);
    assert_eq!(world.bodies[body_index as usize].set_index, AWAKE_SET);
    assert!(world.bodies[body_index as usize].island_id != NULL_INDEX);
    assert!(body_get_mass(&world, body_id) > 0.0);
    assert!(world.shapes[shape_index].proxy_key != NULL_INDEX);
}

#[test]
fn set_type_rejects_compound_on_non_static() {
    use crate::compound::{create_compound, CompoundDef, CompoundHullDef};
    use crate::geometry::default_surface_material;
    use crate::hull::make_box_hull;
    use crate::math_functions::Transform;
    use crate::shape::create_compound_shape;
    use crate::solver_set::STATIC_SET;

    let mut world = World::new(&default_world_def());
    let box_a = make_box_hull(0.5, 0.5, 0.5);
    let hulls = [CompoundHullDef {
        hull: &box_a.base,
        transform: Transform {
            p: VEC3_ZERO,
            q: QUAT_IDENTITY,
        },
        material: default_surface_material(),
    }];
    let compound = create_compound(&CompoundDef {
        hulls: &hulls,
        ..Default::default()
    })
    .expect("compound");

    let mut def = default_body_def();
    def.type_ = BodyType::Static;
    let body_id = create_body(&mut world, &def);
    create_compound_shape(&mut world, body_id, &default_shape_def(), &compound);

    body_set_type(&mut world, body_id, BodyType::Dynamic);
    assert!(!world.locked);
    assert_eq!(body_get_type(&world, body_id), BodyType::Static);
    assert_eq!(
        world.bodies[get_body_full_id(&world, body_id) as usize].set_index,
        STATIC_SET
    );
}

#[test]
fn shape_joint_contact_introspection() {
    use crate::body::{
        body_compute_aabb, body_get_contact_capacity, body_get_contact_data, body_get_joint_count,
        body_get_joints, body_get_shape_count, body_get_shapes, body_set_target_transform,
    };
    use crate::hull::make_box_hull;
    use crate::joint::create_revolute_joint;
    use crate::math_functions::{WorldTransform, QUAT_IDENTITY};
    use crate::shape::create_hull_shape;
    use crate::types::default_revolute_joint_def;

    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(5.0, 0.5, 5.0);
    create_hull_shape(&mut world, ground, &default_shape_def(), &ground_box.base);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 2.0 as _,
        z: 0.0 as _,
    };
    let body_id = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let shape_id = create_hull_shape(&mut world, body_id, &shape_def, &box_hull.base);

    assert_eq!(body_get_shape_count(&world, body_id), 1);
    let shapes = body_get_shapes(&world, body_id, 8);
    assert_eq!(shapes.len(), 1);
    assert_eq!(shapes[0], shape_id);

    let aabb = body_compute_aabb(&world, body_id);
    assert!(aabb.lower_bound.y < 2.0);
    assert!(aabb.upper_bound.y > 2.0);

    let mut joint_def = default_revolute_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = body_id;
    let joint_id = create_revolute_joint(&mut world, &joint_def);
    assert_eq!(body_get_joint_count(&world, body_id), 1);
    let joints = body_get_joints(&world, body_id, 8);
    assert_eq!(joints.len(), 1);
    assert_eq!(joints[0], joint_id);

    for _ in 0..40 {
        world.step(1.0 / 60.0, 4);
    }

    let contact_capacity = body_get_contact_capacity(&world, body_id);
    let contact_data = body_get_contact_data(&world, body_id, 8);
    assert!(contact_data.len() as i32 <= contact_capacity);
    // May or may not touch ground depending on joint; capacity is still valid.
    assert!(contact_capacity >= 0);

    // Kinematic target transform sets velocity toward the target.
    let mut kin_def = default_body_def();
    kin_def.type_ = BodyType::Kinematic;
    kin_def.position = Pos {
        x: 0.0 as _,
        y: 5.0 as _,
        z: 0.0 as _,
    };
    let kin = create_body(&mut world, &kin_def);
    let target = WorldTransform {
        p: Pos {
            x: 1.0 as _,
            y: 5.0 as _,
            z: 0.0 as _,
        },
        q: QUAT_IDENTITY,
    };
    body_set_target_transform(&mut world, kin, target, 1.0 / 60.0, true);
    let v = body_get_local_point_velocity(&world, kin, VEC3_ZERO);
    assert!(v.x > 0.0, "target transform should induce +x velocity");
}

#[test]
fn disable_enable_round_trip() {
    use crate::core::NULL_INDEX;
    use crate::solver_set::{AWAKE_SET, DISABLED_SET, STATIC_SET};

    let mut world = World::new(&default_world_def());

    let mut dyn_def = default_body_def();
    dyn_def.type_ = BodyType::Dynamic;
    let dyn_id = create_body(&mut world, &dyn_def);
    let shape_id = create_sphere_shape(
        &mut world,
        dyn_id,
        &default_shape_def(),
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
    );
    let shape_index = (shape_id.index1 - 1) as usize;
    let dyn_index = get_body_full_id(&world, dyn_id);

    assert!(body_is_enabled(&world, dyn_id));
    assert_eq!(world.bodies[dyn_index as usize].set_index, AWAKE_SET);
    assert!(world.shapes[shape_index].proxy_key != NULL_INDEX);

    body_disable(&mut world, dyn_id);
    assert!(!world.locked);
    assert!(!body_is_enabled(&world, dyn_id));
    assert_eq!(world.bodies[dyn_index as usize].set_index, DISABLED_SET);
    assert_eq!(world.bodies[dyn_index as usize].island_id, NULL_INDEX);
    assert_eq!(world.shapes[shape_index].proxy_key, NULL_INDEX);

    body_enable(&mut world, dyn_id);
    assert!(!world.locked);
    assert!(body_is_enabled(&world, dyn_id));
    assert_eq!(world.bodies[dyn_index as usize].set_index, AWAKE_SET);
    assert!(world.bodies[dyn_index as usize].island_id != NULL_INDEX);
    assert!(world.shapes[shape_index].proxy_key != NULL_INDEX);

    let static_id = create_body(&mut world, &default_body_def());
    let static_index = get_body_full_id(&world, static_id);
    assert_eq!(world.bodies[static_index as usize].set_index, STATIC_SET);

    body_disable(&mut world, static_id);
    assert_eq!(world.bodies[static_index as usize].set_index, DISABLED_SET);
    body_enable(&mut world, static_id);
    assert_eq!(world.bodies[static_index as usize].set_index, STATIC_SET);
    assert_eq!(world.bodies[static_index as usize].island_id, NULL_INDEX);
}

#[test]
fn disable_transfers_attached_joint() {
    use crate::core::NULL_INDEX;
    use crate::joint::{create_distance_joint, get_joint_full_id};
    use crate::solver_set::{AWAKE_SET, DISABLED_SET};
    use crate::types::default_distance_joint_def;

    let mut world = World::new(&default_world_def());
    let ground = create_body(&mut world, &default_body_def());
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 2.0 as _,
        z: 0.0 as _,
    };
    let body = create_body(&mut world, &body_def);

    let mut joint_def = default_distance_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = body;
    let joint_id = create_distance_joint(&mut world, &joint_def);
    let joint_index = get_joint_full_id(&world, joint_id);
    assert_eq!(world.joints[joint_index as usize].set_index, AWAKE_SET);
    assert!(world.joints[joint_index as usize].island_id != NULL_INDEX);

    body_disable(&mut world, body);
    assert_eq!(world.joints[joint_index as usize].set_index, DISABLED_SET);
    assert_eq!(world.joints[joint_index as usize].island_id, NULL_INDEX);

    body_enable(&mut world, body);
    assert_eq!(world.joints[joint_index as usize].set_index, AWAKE_SET);
    assert!(world.joints[joint_index as usize].island_id != NULL_INDEX);
}

/// Contact-recycling flag get/set and per-def opt-out. (EnableContactRecyclingTest)
#[test]
fn enable_contact_recycling() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    // Default is enabled
    let body_a = create_body(&mut world, &body_def);
    assert!(body_is_contact_recycling_enabled(&world, body_a));

    body_enable_contact_recycling(&mut world, body_a, false);
    assert!(!body_is_contact_recycling_enabled(&world, body_a));

    body_enable_contact_recycling(&mut world, body_a, true);
    assert!(body_is_contact_recycling_enabled(&world, body_a));

    // Per-def opt-out at creation
    body_def.enable_contact_recycling = false;
    let body_b = create_body(&mut world, &body_def);
    assert!(!body_is_contact_recycling_enabled(&world, body_b));

    // Stepping after toggling must not trip the flag-sync validator
    world.step(1.0 / 60.0, 4);
}
