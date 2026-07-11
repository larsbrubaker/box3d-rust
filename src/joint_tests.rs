//! Port of box3d-cpp-reference/test/test_joint.c essentials: fixture + step
//! each joint type, exercise the shared b3Joint_* API, and a sleeping-island
//! merge via a new joint.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{create_body, get_body_full_id, is_body_awake};
use crate::core::NULL_INDEX;
use crate::hull::make_cube_hull;
use crate::id::JointId;
use crate::joint::*;
use crate::math_functions::{Pos, Quat, Transform, Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY};
use crate::shape::create_hull_shape;
use crate::solver_set::FIRST_SLEEPING_SET;
use crate::types::{
    default_body_def, default_distance_joint_def, default_filter_joint_def,
    default_motor_joint_def, default_parallel_joint_def, default_prismatic_joint_def,
    default_revolute_joint_def, default_shape_def, default_spherical_joint_def,
    default_weld_joint_def, default_wheel_joint_def, default_world_def, BodyType, JointDef,
};
use crate::world::World;

struct JointFixture {
    world: World,
    ground_id: crate::id::BodyId,
    body_id: crate::id::BodyId,
}

fn create_joint_fixture() -> JointFixture {
    let mut world_def = default_world_def();
    world_def.gravity = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let mut world = World::new(&world_def);

    let ground_def = default_body_def();
    let ground_id = create_body(&mut world, &ground_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 4.0 as _,
        z: 0.0 as _,
    };
    let body_id = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let box_hull = make_cube_hull(0.5);
    create_hull_shape(&mut world, body_id, &shape_def, &box_hull.base);

    JointFixture {
        world,
        ground_id,
        body_id,
    }
}

fn set_common_frames(base: &mut JointDef, f: &JointFixture) {
    base.body_id_a = f.ground_id;
    base.body_id_b = f.body_id;
    base.local_frame_a = Transform {
        p: Vec3 {
            x: 0.0,
            y: 4.0,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    base.local_frame_b = TRANSFORM_IDENTITY;
}

fn finish_joint(world: &mut World, joint_id: JointId) {
    for _ in 0..8 {
        world.step(1.0 / 60.0, 4);
    }

    destroy_joint(world, joint_id, true);
    assert!(!joint_is_valid(world, joint_id));
}

fn exercise_joint_base(
    world: &mut World,
    joint_id: JointId,
    body_id_a: crate::id::BodyId,
    body_id_b: crate::id::BodyId,
    expected_type: JointType,
) {
    assert!(joint_is_valid(world, joint_id));
    assert_eq!(joint_get_type(world, joint_id), expected_type);
    assert_eq!(joint_get_body_a(world, joint_id), body_id_a);
    assert_eq!(joint_get_body_b(world, joint_id), body_id_b);

    let got_world = joint_get_world(world, joint_id);
    assert_eq!(got_world.index1, world.world_id.wrapping_add(1));
    assert_eq!(got_world.generation, world.generation);

    let original_a = joint_get_local_frame_a(world, joint_id);
    let original_b = joint_get_local_frame_b(world, joint_id);

    let frame_a = Transform {
        p: Vec3 {
            x: 0.1,
            y: 0.2,
            z: 0.3,
        },
        q: QUAT_IDENTITY,
    };
    joint_set_local_frame_a(world, joint_id, frame_a);
    let got_a = joint_get_local_frame_a(world, joint_id);
    assert_eq!(got_a.p.x, frame_a.p.x);
    assert_eq!(got_a.p.y, frame_a.p.y);
    assert_eq!(got_a.p.z, frame_a.p.z);

    let frame_b = Transform {
        p: Vec3 {
            x: -0.4,
            y: 0.5,
            z: -0.6,
        },
        q: QUAT_IDENTITY,
    };
    joint_set_local_frame_b(world, joint_id, frame_b);
    let got_b = joint_get_local_frame_b(world, joint_id);
    assert_eq!(got_b.p.x, frame_b.p.x);
    assert_eq!(got_b.p.y, frame_b.p.y);
    assert_eq!(got_b.p.z, frame_b.p.z);

    joint_set_collide_connected(world, joint_id, true);
    assert!(joint_get_collide_connected(world, joint_id));
    joint_set_collide_connected(world, joint_id, false);
    assert!(!joint_get_collide_connected(world, joint_id));

    joint_set_user_data(world, joint_id, 0xDEAD_BEEF);
    assert_eq!(joint_get_user_data(world, joint_id), 0xDEAD_BEEF);

    joint_set_constraint_tuning(world, joint_id, 90.0, 3.0);
    let (hertz, damping_ratio) = joint_get_constraint_tuning(world, joint_id);
    assert_eq!(hertz, 90.0);
    assert_eq!(damping_ratio, 3.0);

    joint_set_force_threshold(world, joint_id, 100.0);
    assert_eq!(joint_get_force_threshold(world, joint_id), 100.0);

    joint_set_torque_threshold(world, joint_id, 200.0);
    assert_eq!(joint_get_torque_threshold(world, joint_id), 200.0);

    joint_wake_bodies(world, joint_id);

    let _force = joint_get_constraint_force(world, joint_id);
    let _torque = joint_get_constraint_torque(world, joint_id);
    let _linear = joint_get_linear_separation(world, joint_id);

    // Wheel joint angular separation is an unimplemented todo in C that asserts.
    if expected_type != JointType::Wheel {
        let _angular = joint_get_angular_separation(world, joint_id);
    }

    joint_set_local_frame_a(world, joint_id, original_a);
    joint_set_local_frame_b(world, joint_id, original_b);
}

#[test]
fn test_parallel_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_parallel_joint_def();
    set_common_frames(&mut def.base, &f);
    def.hertz = 2.0;
    def.damping_ratio = 0.5;
    def.max_torque = 100.0;
    let joint_id = create_parallel_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Parallel,
    );

    parallel_joint_set_spring_hertz(&mut f.world, joint_id, 5.0);
    assert_eq!(parallel_joint_get_spring_hertz(&f.world, joint_id), 5.0);
    parallel_joint_set_spring_damping_ratio(&mut f.world, joint_id, 0.7);
    assert_eq!(
        parallel_joint_get_spring_damping_ratio(&f.world, joint_id),
        0.7
    );
    parallel_joint_set_max_torque(&mut f.world, joint_id, 250.0);
    assert_eq!(parallel_joint_get_max_torque(&f.world, joint_id), 250.0);

    finish_joint(&mut f.world, joint_id);
}

#[test]
fn test_distance_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_distance_joint_def();
    set_common_frames(&mut def.base, &f);
    def.length = 2.0;
    let joint_id = create_distance_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Distance,
    );

    distance_joint_set_length(&mut f.world, joint_id, 3.0);
    assert_eq!(distance_joint_get_length(&f.world, joint_id), 3.0);
    distance_joint_enable_spring(&mut f.world, joint_id, true);
    assert!(distance_joint_is_spring_enabled(&f.world, joint_id));
    distance_joint_enable_limit(&mut f.world, joint_id, true);
    assert!(distance_joint_is_limit_enabled(&f.world, joint_id));
    distance_joint_enable_motor(&mut f.world, joint_id, true);
    assert!(distance_joint_is_motor_enabled(&f.world, joint_id));

    finish_joint(&mut f.world, joint_id);
}

#[test]
fn test_filter_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_filter_joint_def();
    def.base.body_id_a = f.ground_id;
    def.base.body_id_b = f.body_id;
    let joint_id = create_filter_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Filter,
    );
    finish_joint(&mut f.world, joint_id);
}

#[test]
fn test_motor_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_motor_joint_def();
    set_common_frames(&mut def.base, &f);
    let joint_id = create_motor_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Motor,
    );

    motor_joint_set_linear_velocity(
        &mut f.world,
        joint_id,
        Vec3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
    );
    let got = motor_joint_get_linear_velocity(&f.world, joint_id);
    assert_eq!(got.x, 1.0);
    assert_eq!(got.y, 2.0);
    assert_eq!(got.z, 3.0);

    finish_joint(&mut f.world, joint_id);
}

#[test]
fn test_prismatic_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_prismatic_joint_def();
    set_common_frames(&mut def.base, &f);
    let joint_id = create_prismatic_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Prismatic,
    );

    prismatic_joint_enable_spring(&mut f.world, joint_id, true);
    assert!(prismatic_joint_is_spring_enabled(&f.world, joint_id));
    prismatic_joint_set_limits(&mut f.world, joint_id, -2.0, 2.0);
    assert_eq!(prismatic_joint_get_lower_limit(&f.world, joint_id), -2.0);
    assert_eq!(prismatic_joint_get_upper_limit(&f.world, joint_id), 2.0);

    finish_joint(&mut f.world, joint_id);
}

#[test]
fn test_revolute_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_revolute_joint_def();
    set_common_frames(&mut def.base, &f);
    let joint_id = create_revolute_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Revolute,
    );

    revolute_joint_enable_spring(&mut f.world, joint_id, true);
    assert!(revolute_joint_is_spring_enabled(&f.world, joint_id));
    revolute_joint_set_limits(&mut f.world, joint_id, -1.0, 1.0);
    assert_eq!(revolute_joint_get_lower_limit(&f.world, joint_id), -1.0);
    assert_eq!(revolute_joint_get_upper_limit(&f.world, joint_id), 1.0);

    finish_joint(&mut f.world, joint_id);
}

#[test]
fn test_spherical_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_spherical_joint_def();
    set_common_frames(&mut def.base, &f);
    let joint_id = create_spherical_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Spherical,
    );

    spherical_joint_enable_cone_limit(&mut f.world, joint_id, true);
    assert!(spherical_joint_is_cone_limit_enabled(&f.world, joint_id));
    let target = Quat {
        v: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.7071068,
        },
        s: 0.7071068,
    };
    spherical_joint_set_target_rotation(&mut f.world, joint_id, target);
    let got = spherical_joint_get_target_rotation(&f.world, joint_id);
    assert!((got.v.z - target.v.z).abs() < 1.0e-5);
    assert!((got.s - target.s).abs() < 1.0e-5);

    finish_joint(&mut f.world, joint_id);
}

#[test]
fn test_weld_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_weld_joint_def();
    set_common_frames(&mut def.base, &f);
    let joint_id = create_weld_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Weld,
    );

    weld_joint_set_linear_hertz(&mut f.world, joint_id, 3.0);
    assert_eq!(weld_joint_get_linear_hertz(&f.world, joint_id), 3.0);
    weld_joint_set_angular_hertz(&mut f.world, joint_id, 4.0);
    assert_eq!(weld_joint_get_angular_hertz(&f.world, joint_id), 4.0);

    finish_joint(&mut f.world, joint_id);
}

#[test]
fn test_wheel_joint() {
    let mut f = create_joint_fixture();
    let mut def = default_wheel_joint_def();
    set_common_frames(&mut def.base, &f);
    let joint_id = create_wheel_joint(&mut f.world, &def);

    exercise_joint_base(
        &mut f.world,
        joint_id,
        f.ground_id,
        f.body_id,
        JointType::Wheel,
    );

    wheel_joint_enable_suspension(&mut f.world, joint_id, true);
    assert!(wheel_joint_is_suspension_enabled(&f.world, joint_id));
    wheel_joint_set_suspension_hertz(&mut f.world, joint_id, 5.0);
    assert_eq!(wheel_joint_get_suspension_hertz(&f.world, joint_id), 5.0);
    wheel_joint_enable_spin_motor(&mut f.world, joint_id, true);
    assert!(wheel_joint_is_spin_motor_enabled(&f.world, joint_id));

    finish_joint(&mut f.world, joint_id);
}

/// Two sleeping islands joined by a new joint merge into one sleeping set
/// (merge_solver_sets / sleeping-joint transfer). Bodies stay asleep.
#[test]
fn sleeping_islands_merge_on_new_joint() {
    use crate::hull::make_box_hull;

    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let ground_hull = make_box_hull(20.0, 0.5, 5.0);
    create_hull_shape(
        &mut world,
        ground,
        &default_shape_def(),
        &ground_hull.base,
    );

    let cube = make_cube_hull(0.5);
    let mut cube_shape = default_shape_def();
    cube_shape.density = 1.0;

    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    box_def.position = Pos {
        x: -5.0 as _,
        y: 1.05 as _,
        z: 0.0 as _,
    };
    let box_a = create_body(&mut world, &box_def);
    create_hull_shape(&mut world, box_a, &cube_shape, &cube.base);

    box_def.position = Pos {
        x: 5.0 as _,
        y: 1.05 as _,
        z: 0.0 as _,
    };
    let box_b = create_body(&mut world, &box_def);
    create_hull_shape(&mut world, box_b, &cube_shape, &cube.base);

    let index_a = get_body_full_id(&world, box_a);
    let index_b = get_body_full_id(&world, box_b);

    let mut sleep_a = NULL_INDEX;
    let mut sleep_b = NULL_INDEX;
    for step in 0..400 {
        world.step(1.0 / 60.0, 4);
        if sleep_a == NULL_INDEX && !is_body_awake(&world, index_a) {
            sleep_a = step;
        }
        if sleep_b == NULL_INDEX && !is_body_awake(&world, index_b) {
            sleep_b = step;
        }
        if sleep_a != NULL_INDEX && sleep_b != NULL_INDEX {
            break;
        }
    }

    assert!(sleep_a != NULL_INDEX, "body A never fell asleep");
    assert!(sleep_b != NULL_INDEX, "body B never fell asleep");

    let set_a = world.bodies[index_a as usize].set_index;
    let set_b = world.bodies[index_b as usize].set_index;
    assert!(set_a >= FIRST_SLEEPING_SET);
    assert!(set_b >= FIRST_SLEEPING_SET);
    assert_ne!(
        set_a, set_b,
        "expected separate sleeping solver sets before the joint"
    );

    let mut def = default_distance_joint_def();
    def.base.body_id_a = box_a;
    def.base.body_id_b = box_b;
    def.base.local_frame_a = TRANSFORM_IDENTITY;
    def.base.local_frame_b = TRANSFORM_IDENTITY;
    def.length = 10.0;
    let joint_id = create_distance_joint(&mut world, &def);

    assert!(joint_is_valid(&world, joint_id));
    assert!(!is_body_awake(&world, index_a));
    assert!(!is_body_awake(&world, index_b));

    let merged_a = world.bodies[index_a as usize].set_index;
    let merged_b = world.bodies[index_b as usize].set_index;
    assert_eq!(merged_a, merged_b);
    assert!(merged_a >= FIRST_SLEEPING_SET);

    let raw = get_joint_full_id(&world, joint_id);
    assert_eq!(world.joints[raw as usize].set_index, merged_a);
    assert!(world.solver_sets[merged_a as usize]
        .joint_sims
        .iter()
        .any(|sim| sim.joint_id == raw));
}
