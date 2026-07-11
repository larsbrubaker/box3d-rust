//! Body public API tests for getters, forces, transform, and awake.
//!
//! Ported patterns from box3d-cpp-reference/test/test_body.c.

use crate::body::{
    body_apply_force, body_apply_force_to_center, body_apply_linear_impulse,
    body_apply_linear_impulse_to_center, body_apply_torque, body_get_local_point,
    body_get_local_point_velocity, body_get_local_vector, body_get_motion_locks, body_get_name,
    body_get_type, body_get_user_data, body_get_world_point, body_get_world_point_velocity,
    body_get_world_vector, body_is_awake, body_is_enabled, body_set_awake, body_set_motion_locks,
    body_set_name, body_set_transform, body_set_user_data, body_sim, create_body,
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

    // Truncates to BODY_NAME_LENGTH (18).
    body_set_name(&mut world, body_id, "abcdefghijklmnopqrstuvwxyz");
    assert_eq!(body_get_name(&world, body_id), "abcdefghijklmnopqr");

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
        &Sphere { center: VEC3_ZERO, radius: 0.5 },
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
        &Sphere { center: VEC3_ZERO, radius: 0.25 },
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
