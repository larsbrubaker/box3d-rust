// Tests for src/joint/lifecycle.rs, split into its own file to keep
// lifecycle.rs under the 800-line limit as the joint-creation surface grows.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::*;
use crate::body::{create_body, destroy_body, get_body_full_id};
use crate::broad_phase::update_broad_phase_pairs;
use crate::constraint_graph::OVERFLOW_INDEX;
use crate::core::NULL_INDEX;
use crate::hull::make_cube_hull;
use crate::joint::{destroy_joint, joint_is_valid};
use crate::shape::create_hull_shape;
use crate::solver_set::AWAKE_SET;
use crate::types::{
    default_body_def, default_filter_joint_def, default_shape_def, default_world_def, BodyType,
};
use crate::world::World;

#[test]
fn create_and_destroy_filter_joint() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body_a = create_body(&mut world, &body_def);
    let body_b = create_body(&mut world, &body_def);
    let a_index = get_body_full_id(&world, body_a);
    let b_index = get_body_full_id(&world, body_b);

    let cube = make_cube_hull(0.5);
    let shape_def = default_shape_def();
    let _sa = create_hull_shape(&mut world, body_a, &shape_def, &cube.base);
    let _sb = create_hull_shape(&mut world, body_b, &shape_def, &cube.base);

    update_broad_phase_pairs(&mut world);
    assert_eq!(world.contact_id_pool.id_count(), 1);

    // The two dynamic bodies start in separate islands.
    assert_ne!(
        world.bodies[a_index as usize].island_id,
        world.bodies[b_index as usize].island_id
    );

    // A filter joint merges islands. Box3D C does not destroy the contact
    // on create; SetCollideConnected(false) does (already false here, so
    // toggle true then false to exercise destroy).
    let mut filter_def = default_filter_joint_def();
    filter_def.base.body_id_a = body_a;
    filter_def.base.body_id_b = body_b;
    let filter_id = create_filter_joint(&mut world, &filter_def);

    assert!(joint_is_valid(&world, filter_id));
    assert_eq!(world.joint_id_pool.id_count(), 1);
    assert_eq!(world.bodies[a_index as usize].joint_count, 1);
    assert_eq!(world.bodies[b_index as usize].joint_count, 1);
    assert_eq!(
        world.bodies[a_index as usize].island_id,
        world.bodies[b_index as usize].island_id
    );
    assert_eq!(joint_get_type(&world, filter_id), JointType::Filter);
    assert!(!joint_get_collide_connected(&world, filter_id));
    assert_eq!(joint_get_body_a(&world, filter_id), body_a);
    assert_eq!(joint_get_body_b(&world, filter_id), body_b);

    let raw = get_joint_full_id(&world, filter_id);
    {
        let joint = &world.joints[raw as usize];
        assert_eq!(joint.set_index, AWAKE_SET);
        assert!(joint.color_index != NULL_INDEX && joint.color_index <= OVERFLOW_INDEX);
        assert!(joint.island_id != NULL_INDEX);
        assert_eq!(joint.type_, JointType::Filter);
    }

    // Destroy existing contact via SetCollideConnected(false) after a
    // no-op true╬ô├Ñ├åfalse path is already default; force via true then false.
    assert_eq!(world.contact_id_pool.id_count(), 1);
    joint_set_collide_connected(&mut world, filter_id, true);
    assert!(joint_get_collide_connected(&world, filter_id));
    joint_set_collide_connected(&mut world, filter_id, false);
    assert!(!joint_get_collide_connected(&world, filter_id));
    assert_eq!(world.contact_id_pool.id_count(), 0);

    // Broad-phase pair update does not recreate the filtered contact.
    update_broad_phase_pairs(&mut world);
    assert_eq!(world.contact_id_pool.id_count(), 0);

    // Enabling collision re-buffers shapes so the broad phase can recreate.
    joint_set_collide_connected(&mut world, filter_id, true);
    update_broad_phase_pairs(&mut world);
    assert_eq!(world.contact_id_pool.id_count(), 1);

    destroy_joint(&mut world, filter_id, true);
    assert!(!joint_is_valid(&world, filter_id));
    assert_eq!(world.joint_id_pool.id_count(), 0);
    assert_eq!(world.bodies[a_index as usize].joint_count, 0);
    assert_eq!(world.bodies[b_index as usize].joint_count, 0);

    // Destroying body A with no joints is fine.
    destroy_body(&mut world, body_a);
    world.validate_solver_sets();
}

#[test]
fn destroy_body_destroys_attached_joints() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body_a = create_body(&mut world, &body_def);
    let body_b = create_body(&mut world, &body_def);

    let mut filter_def = default_filter_joint_def();
    filter_def.base.body_id_a = body_a;
    filter_def.base.body_id_b = body_b;
    let filter_id = create_filter_joint(&mut world, &filter_def);
    assert_eq!(world.joint_id_pool.id_count(), 1);

    destroy_body(&mut world, body_a);
    assert!(!joint_is_valid(&world, filter_id));
    assert_eq!(world.joint_id_pool.id_count(), 0);
    world.validate_solver_sets();
}

#[test]
fn create_distance_joint_and_accessors() {
    use crate::joint::{
        distance_joint_enable_limit, distance_joint_enable_motor, distance_joint_enable_spring,
        distance_joint_get_current_length, distance_joint_get_length,
        distance_joint_get_max_length, distance_joint_get_max_motor_force,
        distance_joint_get_min_length, distance_joint_get_motor_speed,
        distance_joint_get_spring_damping_ratio, distance_joint_get_spring_force_range,
        distance_joint_get_spring_hertz, distance_joint_is_limit_enabled,
        distance_joint_is_motor_enabled, distance_joint_is_spring_enabled,
        distance_joint_set_length, distance_joint_set_length_range,
        distance_joint_set_max_motor_force, distance_joint_set_motor_speed,
        distance_joint_set_spring_damping_ratio, distance_joint_set_spring_force_range,
        distance_joint_set_spring_hertz,
    };
    use crate::types::default_distance_joint_def;

    let mut world = World::new(&default_world_def());
    let ground = create_body(&mut world, &default_body_def());
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = crate::math_functions::Pos {
        x: 0.0 as _,
        y: 4.0 as _,
        z: 0.0 as _,
    };
    let body = create_body(&mut world, &body_def);

    let mut def = default_distance_joint_def();
    def.base.body_id_a = ground;
    def.base.body_id_b = body;
    def.base.local_frame_a.p = crate::math_functions::Vec3 {
        x: 0.0,
        y: 4.0,
        z: 0.0,
    };
    def.length = 2.0;
    let id = create_distance_joint(&mut world, &def);

    assert_eq!(joint_get_type(&world, id), JointType::Distance);
    assert_eq!(distance_joint_get_length(&world, id), 2.0);

    distance_joint_set_length(&mut world, id, 3.0);
    assert_eq!(distance_joint_get_length(&world, id), 3.0);

    distance_joint_enable_limit(&mut world, id, true);
    assert!(distance_joint_is_limit_enabled(&world, id));
    distance_joint_set_length_range(&mut world, id, 1.0, 5.0);
    assert_eq!(distance_joint_get_min_length(&world, id), 1.0);
    assert_eq!(distance_joint_get_max_length(&world, id), 5.0);

    distance_joint_enable_spring(&mut world, id, true);
    assert!(distance_joint_is_spring_enabled(&world, id));
    distance_joint_set_spring_hertz(&mut world, id, 4.0);
    assert_eq!(distance_joint_get_spring_hertz(&world, id), 4.0);
    distance_joint_set_spring_damping_ratio(&mut world, id, 0.5);
    assert_eq!(distance_joint_get_spring_damping_ratio(&world, id), 0.5);
    distance_joint_set_spring_force_range(&mut world, id, -10.0, 20.0);
    assert_eq!(
        distance_joint_get_spring_force_range(&world, id),
        (-10.0, 20.0)
    );

    distance_joint_enable_motor(&mut world, id, true);
    assert!(distance_joint_is_motor_enabled(&world, id));
    distance_joint_set_motor_speed(&mut world, id, 1.5);
    assert_eq!(distance_joint_get_motor_speed(&world, id), 1.5);
    distance_joint_set_max_motor_force(&mut world, id, 25.0);
    assert_eq!(distance_joint_get_max_motor_force(&world, id), 25.0);

    let current = distance_joint_get_current_length(&world, id);
    assert!((current - 0.0).abs() < 1e-5, "current={current}");

    destroy_joint(&mut world, id, true);
    world.validate_solver_sets();
}

#[test]
fn create_parallel_joint_and_accessors() {
    use crate::joint::{
        get_parallel_joint_torque, parallel_joint_get_max_torque,
        parallel_joint_get_spring_damping_ratio, parallel_joint_get_spring_hertz,
        parallel_joint_set_max_torque, parallel_joint_set_spring_damping_ratio,
        parallel_joint_set_spring_hertz,
    };
    use crate::types::default_parallel_joint_def;

    let mut world = World::new(&default_world_def());
    let ground = create_body(&mut world, &default_body_def());
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);

    let mut def = default_parallel_joint_def();
    def.base.body_id_a = ground;
    def.base.body_id_b = body;
    let id = create_parallel_joint(&mut world, &def);

    assert_eq!(joint_get_type(&world, id), JointType::Parallel);
    assert_eq!(
        parallel_joint_get_spring_hertz(&world, id),
        default_parallel_joint_def().hertz
    );
    assert_eq!(
        parallel_joint_get_spring_damping_ratio(&world, id),
        default_parallel_joint_def().damping_ratio
    );
    assert_eq!(
        parallel_joint_get_max_torque(&world, id),
        default_parallel_joint_def().max_torque
    );

    parallel_joint_set_spring_hertz(&mut world, id, 2.0);
    assert_eq!(parallel_joint_get_spring_hertz(&world, id), 2.0);
    parallel_joint_set_spring_damping_ratio(&mut world, id, 0.3);
    assert_eq!(parallel_joint_get_spring_damping_ratio(&world, id), 0.3);
    parallel_joint_set_max_torque(&mut world, id, 5.0);
    assert_eq!(parallel_joint_get_max_torque(&world, id), 5.0);

    // Torque is zero with no accumulated impulse.
    let inv_h = world.inv_h;
    let raw = get_joint_full_id(&world, id);
    let joint_sim = get_joint_sim(&mut world, raw);
    let torque = get_parallel_joint_torque(inv_h, joint_sim);
    assert_eq!(torque, crate::math_functions::VEC3_ZERO);

    destroy_joint(&mut world, id, true);
    world.validate_solver_sets();
}

#[test]
fn create_weld_joint_and_accessors() {
    use crate::joint::{
        get_joint_sim_ref, get_weld_joint_force, get_weld_joint_torque,
        weld_joint_get_angular_damping_ratio, weld_joint_get_angular_hertz,
        weld_joint_get_linear_damping_ratio, weld_joint_get_linear_hertz,
        weld_joint_set_angular_damping_ratio, weld_joint_set_angular_hertz,
        weld_joint_set_linear_damping_ratio, weld_joint_set_linear_hertz,
    };
    use crate::types::default_weld_joint_def;

    let mut world = World::new(&default_world_def());
    let ground = create_body(&mut world, &default_body_def());
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);

    let mut def = default_weld_joint_def();
    def.base.body_id_a = ground;
    def.base.body_id_b = body;
    let id = create_weld_joint(&mut world, &def);

    assert_eq!(joint_get_type(&world, id), JointType::Weld);
    assert_eq!(weld_joint_get_linear_hertz(&world, id), 0.0);
    assert_eq!(weld_joint_get_linear_damping_ratio(&world, id), 0.0);
    assert_eq!(weld_joint_get_angular_hertz(&world, id), 0.0);
    assert_eq!(weld_joint_get_angular_damping_ratio(&world, id), 0.0);

    weld_joint_set_linear_hertz(&mut world, id, 3.0);
    assert_eq!(weld_joint_get_linear_hertz(&world, id), 3.0);
    weld_joint_set_linear_damping_ratio(&mut world, id, 0.6);
    assert_eq!(weld_joint_get_linear_damping_ratio(&world, id), 0.6);
    weld_joint_set_angular_hertz(&mut world, id, 4.0);
    assert_eq!(weld_joint_get_angular_hertz(&world, id), 4.0);
    weld_joint_set_angular_damping_ratio(&mut world, id, 0.7);
    assert_eq!(weld_joint_get_angular_damping_ratio(&world, id), 0.7);

    // Force/torque are zero with no accumulated impulse.
    let raw = get_joint_full_id(&world, id);
    let joint_sim = get_joint_sim_ref(&world, raw);
    assert_eq!(
        get_weld_joint_force(&world, joint_sim),
        crate::math_functions::VEC3_ZERO
    );
    assert_eq!(
        get_weld_joint_torque(&world, joint_sim),
        crate::math_functions::VEC3_ZERO
    );

    destroy_joint(&mut world, id, true);
    world.validate_solver_sets();
}

#[test]
fn create_revolute_joint_and_accessors() {
    use crate::joint::{
        get_joint_sim_ref, get_revolute_joint_force,
        revolute_joint_enable_limit, revolute_joint_enable_motor, revolute_joint_enable_spring,
        revolute_joint_get_angle, revolute_joint_get_lower_limit,
        revolute_joint_get_max_motor_torque, revolute_joint_get_motor_speed,
        revolute_joint_get_motor_torque, revolute_joint_get_spring_damping_ratio,
        revolute_joint_get_spring_hertz, revolute_joint_get_target_angle,
        revolute_joint_get_upper_limit, revolute_joint_is_limit_enabled,
        revolute_joint_is_motor_enabled, revolute_joint_is_spring_enabled,
        revolute_joint_set_limits, revolute_joint_set_max_motor_torque,
        revolute_joint_set_motor_speed, revolute_joint_set_spring_damping_ratio,
        revolute_joint_set_spring_hertz, revolute_joint_set_target_angle,
    };
    use crate::types::default_revolute_joint_def;

    let mut world = World::new(&default_world_def());
    let ground = create_body(&mut world, &default_body_def());
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);

    let mut def = default_revolute_joint_def();
    def.base.body_id_a = ground;
    def.base.body_id_b = body;
    let id = create_revolute_joint(&mut world, &def);

    assert_eq!(joint_get_type(&world, id), JointType::Revolute);
    assert!((revolute_joint_get_angle(&world, id)).abs() < 1e-5);

    revolute_joint_enable_limit(&mut world, id, true);
    assert!(revolute_joint_is_limit_enabled(&world, id));
    revolute_joint_set_limits(&mut world, id, -1.0, 1.0);
    assert_eq!(revolute_joint_get_lower_limit(&world, id), -1.0);
    assert_eq!(revolute_joint_get_upper_limit(&world, id), 1.0);

    revolute_joint_enable_spring(&mut world, id, true);
    assert!(revolute_joint_is_spring_enabled(&world, id));
    revolute_joint_set_target_angle(&mut world, id, 0.5);
    assert_eq!(revolute_joint_get_target_angle(&world, id), 0.5);
    revolute_joint_set_spring_hertz(&mut world, id, 4.0);
    assert_eq!(revolute_joint_get_spring_hertz(&world, id), 4.0);
    revolute_joint_set_spring_damping_ratio(&mut world, id, 0.5);
    assert_eq!(revolute_joint_get_spring_damping_ratio(&world, id), 0.5);

    revolute_joint_enable_motor(&mut world, id, true);
    assert!(revolute_joint_is_motor_enabled(&world, id));
    revolute_joint_set_motor_speed(&mut world, id, 1.5);
    assert_eq!(revolute_joint_get_motor_speed(&world, id), 1.5);
    revolute_joint_set_max_motor_torque(&mut world, id, 25.0);
    assert_eq!(revolute_joint_get_max_motor_torque(&world, id), 25.0);
    assert_eq!(revolute_joint_get_motor_torque(&world, id), 0.0);

    let raw = get_joint_full_id(&world, id);
    assert_eq!(
        get_revolute_joint_force(&world, get_joint_sim_ref(&world, raw)),
        crate::math_functions::VEC3_ZERO
    );
    // get_revolute_joint_torque needs &World + &mut JointSim (C dual pointer).
    // With zero impulses the torque is zero without needing the dual borrow.
    {
        let joint = get_joint_sim_ref(&world, raw).revolute();
        assert_eq!(joint.perp_impulse, crate::math_functions::VEC2_ZERO);
        assert_eq!(joint.spring_impulse, 0.0);
        assert_eq!(joint.motor_impulse, 0.0);
        assert_eq!(joint.lower_impulse, 0.0);
        assert_eq!(joint.upper_impulse, 0.0);
    }

    destroy_joint(&mut world, id, true);
    world.validate_solver_sets();
}

#[test]
fn create_prismatic_joint_and_accessors() {
    use crate::joint::{
        get_joint_sim_ref, get_prismatic_joint_force, get_prismatic_joint_torque,
        prismatic_joint_enable_limit, prismatic_joint_enable_motor,
        prismatic_joint_enable_spring, prismatic_joint_get_lower_limit,
        prismatic_joint_get_max_motor_force, prismatic_joint_get_motor_force,
        prismatic_joint_get_motor_speed, prismatic_joint_get_spring_damping_ratio,
        prismatic_joint_get_spring_hertz, prismatic_joint_get_target_translation,
        prismatic_joint_get_translation, prismatic_joint_get_upper_limit,
        prismatic_joint_is_limit_enabled, prismatic_joint_is_motor_enabled,
        prismatic_joint_is_spring_enabled, prismatic_joint_set_limits,
        prismatic_joint_set_max_motor_force, prismatic_joint_set_motor_speed,
        prismatic_joint_set_spring_damping_ratio, prismatic_joint_set_spring_hertz,
        prismatic_joint_set_target_translation,
    };
    use crate::types::default_prismatic_joint_def;

    let mut world = World::new(&default_world_def());
    let ground = create_body(&mut world, &default_body_def());
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body = create_body(&mut world, &body_def);

    let mut def = default_prismatic_joint_def();
    def.base.body_id_a = ground;
    def.base.body_id_b = body;
    let id = create_prismatic_joint(&mut world, &def);

    assert_eq!(joint_get_type(&world, id), JointType::Prismatic);
    assert!((prismatic_joint_get_translation(&world, id)).abs() < 1e-5);

    prismatic_joint_enable_limit(&mut world, id, true);
    assert!(prismatic_joint_is_limit_enabled(&world, id));
    prismatic_joint_set_limits(&mut world, id, -1.0, 1.0);
    assert_eq!(prismatic_joint_get_lower_limit(&world, id), -1.0);
    assert_eq!(prismatic_joint_get_upper_limit(&world, id), 1.0);

    prismatic_joint_enable_spring(&mut world, id, true);
    assert!(prismatic_joint_is_spring_enabled(&world, id));
    prismatic_joint_set_target_translation(&mut world, id, 0.5);
    assert_eq!(prismatic_joint_get_target_translation(&world, id), 0.5);
    prismatic_joint_set_spring_hertz(&mut world, id, 4.0);
    assert_eq!(prismatic_joint_get_spring_hertz(&world, id), 4.0);
    prismatic_joint_set_spring_damping_ratio(&mut world, id, 0.5);
    assert_eq!(prismatic_joint_get_spring_damping_ratio(&world, id), 0.5);

    prismatic_joint_enable_motor(&mut world, id, true);
    assert!(prismatic_joint_is_motor_enabled(&world, id));
    prismatic_joint_set_motor_speed(&mut world, id, 1.5);
    assert_eq!(prismatic_joint_get_motor_speed(&world, id), 1.5);
    prismatic_joint_set_max_motor_force(&mut world, id, 25.0);
    assert_eq!(prismatic_joint_get_max_motor_force(&world, id), 25.0);
    assert_eq!(prismatic_joint_get_motor_force(&world, id), 0.0);

    let raw = get_joint_full_id(&world, id);
    let joint_sim = get_joint_sim_ref(&world, raw);
    assert_eq!(
        get_prismatic_joint_force(&world, joint_sim),
        crate::math_functions::VEC3_ZERO
    );
    assert_eq!(
        get_prismatic_joint_torque(&world, joint_sim),
        crate::math_functions::VEC3_ZERO
    );

    destroy_joint(&mut world, id, true);
    world.validate_solver_sets();
}

#[test]
fn default_joint_defs_match_c_cookies() {
    use crate::core::SECRET_COOKIE;
    use crate::types::{
        default_distance_joint_def, default_motor_joint_def, default_parallel_joint_def,
        default_prismatic_joint_def, default_revolute_joint_def, default_spherical_joint_def,
        default_weld_joint_def, default_wheel_joint_def,
    };

    assert_eq!(
        default_filter_joint_def().base.internal_value,
        SECRET_COOKIE
    );
    assert_eq!(
        default_distance_joint_def().base.internal_value,
        SECRET_COOKIE
    );
    assert_eq!(default_motor_joint_def().base.internal_value, SECRET_COOKIE);
    assert_eq!(
        default_parallel_joint_def().base.internal_value,
        SECRET_COOKIE
    );
    assert_eq!(
        default_prismatic_joint_def().base.internal_value,
        SECRET_COOKIE
    );
    assert_eq!(
        default_revolute_joint_def().base.internal_value,
        SECRET_COOKIE
    );
    assert_eq!(
        default_spherical_joint_def().base.internal_value,
        SECRET_COOKIE
    );
    assert_eq!(default_weld_joint_def().base.internal_value, SECRET_COOKIE);
    assert_eq!(default_wheel_joint_def().base.internal_value, SECRET_COOKIE);

    let wheel = default_wheel_joint_def();
    assert!(wheel.enable_suspension_spring);
    assert_eq!(wheel.suspension_hertz, 1.0);
    assert_eq!(wheel.suspension_damping_ratio, 0.7);

    let parallel = default_parallel_joint_def();
    assert_eq!(parallel.hertz, 1.0);
    assert_eq!(parallel.damping_ratio, 1.0);
    assert_eq!(parallel.max_torque, f32::MAX);

    let distance = default_distance_joint_def();
    assert_eq!(distance.length, 1.0);
    assert_eq!(distance.lower_spring_force, -f32::MAX);
    assert_eq!(distance.upper_spring_force, f32::MAX);
}
