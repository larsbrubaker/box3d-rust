//! The larger self-contained blocks of the `DynamicMoverSample` constructor
//! (`sample_character.cpp:1056-1309`): the door, the ball pit, the pressure plate, and
//! the sweeper. Split from [`super::scene`] only for file length; [`super::scene`] calls
//! them in the C order.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use super::{DYNAMIC_BIT, MOVER_BIT, PLATE_CENTER, PLATE_TRAVEL, SWEEPER_CENTER};
use crate::character_demo::dynamic_mover::DynamicMover;
use crate::vis::VisBody;
use box3d_rust::body::{body_get_local_point, body_get_mass, create_body};
use box3d_rust::debug_draw::HexColor;
use box3d_rust::geometry::Sphere;
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::id::{BodyId, JointId};
use box3d_rust::joint::{create_prismatic_joint, create_revolute_joint, create_weld_joint};
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, length, make_quat_from_axis_angle, Pos, Transform, Vec3,
    DEG_TO_RAD, PI, QUAT_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ZERO,
};
use box3d_rust::shape::{create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_prismatic_joint_def, default_revolute_joint_def, default_shape_def,
    default_weld_joint_def, BodyType, Filter,
};
use box3d_rust::world::{world_get_gravity, World};

fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

fn v3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn index_of(id: BodyId) -> i32 {
    id.index1 - 1
}

/// The door block of the constructor (:1056): a slate-gray frame and a spring-loaded
/// peru door on a ±90° revolute.
pub(super) fn build_door(world: &mut World, vis: &mut Vec<VisBody>) {
    let door_position = pos(-10.0, 1.6, -8.0);
    let door_half_width = 0.75f32;
    let door_half_height = 1.5f32;

    // The door fits the frame tightly. The joint keeps the two from colliding.
    let mut frame_def = default_body_def();
    frame_def.position = door_position;
    let frame_id = create_body(world, &frame_def);
    let mut shape_def = default_shape_def();
    shape_def.base_material.custom_color = HexColor::SLATE_GRAY.0;

    let frame_half_width = 0.3f32;
    let post_bottom = -(door_position.y as f32);
    let post_half_height = 0.5 * (door_half_height - post_bottom);
    let post_offset = door_half_width + frame_half_width;

    for side in [-1.0f32, 1.0] {
        let transform = Transform {
            p: v3(side * post_offset, post_bottom + post_half_height, 0.0),
            q: QUAT_IDENTITY,
        };
        let post = make_transformed_box_hull(frame_half_width, post_half_height, 0.2, transform);
        create_hull_shape(world, frame_id, &shape_def, &post.base);
        vis.push(VisBody::box_local(
            index_of(frame_id),
            frame_half_width,
            post_half_height,
            0.2,
            transform,
        ));
    }

    let transform = Transform {
        p: v3(0.0, door_half_height + frame_half_width, 0.0),
        q: QUAT_IDENTITY,
    };
    let lintel = make_transformed_box_hull(
        post_offset + frame_half_width,
        frame_half_width,
        0.2,
        transform,
    );
    create_hull_shape(world, frame_id, &shape_def, &lintel.base);
    vis.push(VisBody::box_local(
        index_of(frame_id),
        post_offset + frame_half_width,
        frame_half_width,
        0.2,
        transform,
    ));

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = door_position;
    body_def.gravity_scale = 2.0;
    let door_id = create_body(world, &body_def);

    let mut door_shape_def = default_shape_def();
    door_shape_def.filter.category_bits = DYNAMIC_BIT;
    door_shape_def.density = 100.0;
    door_shape_def.base_material.custom_color = HexColor::PERU.0;
    let box_hull = make_box_hull(door_half_width, door_half_height, 0.1);
    create_hull_shape(world, door_id, &door_shape_def, &box_hull.base);
    vis.push(VisBody::box_body(
        index_of(door_id),
        door_half_width,
        door_half_height,
        0.1,
    ));

    let axis_quat = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    let offset = v3(-door_half_width, 0.0, 0.0);

    let mut joint_def = default_revolute_joint_def();
    joint_def.base.body_id_a = frame_id;
    joint_def.base.body_id_b = door_id;
    joint_def.base.local_frame_a = Transform {
        p: offset,
        q: axis_quat,
    };
    joint_def.base.local_frame_b = Transform {
        p: offset,
        q: axis_quat,
    };
    joint_def.enable_limit = true;
    joint_def.lower_angle = DEG_TO_RAD * -90.0;
    joint_def.upper_angle = DEG_TO_RAD * 90.0;
    joint_def.enable_spring = true;
    joint_def.hertz = 1.0;
    joint_def.damping_ratio = 0.5;
    joint_def.base.draw_scale = 2.0;
    create_revolute_joint(world, &joint_def);
}

/// The ball pit block (:1120): 20×20 zero-gravity spheres on soft weld springs.
pub(super) fn build_ball_pit(world: &mut World, vis: &mut Vec<VisBody>, ground_id: BodyId) {
    let mut sphere_shape_def = default_shape_def();
    sphere_shape_def.density = 100.0;
    sphere_shape_def.base_material.friction = 0.2;
    sphere_shape_def.base_material.custom_color = HexColor::LIGHT_SEA_GREEN.0;
    sphere_shape_def.filter.category_bits = DYNAMIC_BIT;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.35,
    };

    let mut joint_def = default_weld_joint_def();
    joint_def.linear_hertz = 0.5;
    joint_def.linear_damping_ratio = 0.2;
    joint_def.base.body_id_a = ground_id;
    joint_def.base.collide_connected = true;

    let grid_count = 20;
    for i in 0..grid_count {
        for j in 0..grid_count {
            let position = pos(-9.5 + 1.0 * i as f32, 1.3, 17.0 + 1.0 * j as f32);

            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = position;
            body_def.gravity_scale = 0.0;
            let sphere_id = create_body(world, &body_def);
            create_sphere_shape(world, sphere_id, &sphere_shape_def, &sphere);
            vis.push(VisBody::sphere_body(index_of(sphere_id), 0.35));

            joint_def.base.body_id_b = sphere_id;
            joint_def.base.local_frame_a.p = body_get_local_point(world, ground_id, position);
            joint_def.base.local_frame_b.p = VEC3_ZERO;
            create_weld_joint(world, &joint_def);
        }
    }
}

/// The pressure plate block (:1157): a fire-brick plate on an upright prismatic whose
/// motor works like a preloaded spring.
pub(super) fn build_pressure_plate(
    world: &mut World,
    vis: &mut Vec<VisBody>,
    ground_id: BodyId,
    mover: &DynamicMover,
) -> JointId {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = PLATE_CENTER;
    let plate_id = create_body(world, &body_def);

    let mut plate_shape_def = default_shape_def();
    plate_shape_def.density = 20.0;
    plate_shape_def.base_material.custom_color = HexColor::FIRE_BRICK.0;
    // Only allow the mover to press this.
    plate_shape_def.filter = Filter {
        category_bits: DYNAMIC_BIT,
        mask_bits: MOVER_BIT,
        group_index: 0,
    };
    plate_shape_def.explosion_scale = 0.0;

    let plate = make_box_hull(1.5, 0.1, 1.5);
    create_hull_shape(world, plate_id, &plate_shape_def, &plate.base);
    vis.push(VisBody::box_body(index_of(plate_id), 1.5, 0.1, 1.5));

    // The motor force works like a preloaded spring. The plate holds its own weight but
    // sinks under the mover.
    let gravity = length(world_get_gravity(world));
    let plate_weight = gravity * body_get_mass(world, plate_id);
    let mover_weight = mover.gravity_scale * gravity * body_get_mass(world, mover.body_id());

    // The prismatic axis is the local x-axis, so turn it upright
    let upright = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.5 * PI);

    let mut joint_def = default_prismatic_joint_def();
    joint_def.base.body_id_a = ground_id;
    joint_def.base.body_id_b = plate_id;
    joint_def.base.local_frame_a = Transform {
        p: body_get_local_point(world, ground_id, PLATE_CENTER),
        q: upright,
    };
    joint_def.base.local_frame_b = Transform {
        p: VEC3_ZERO,
        q: upright,
    };
    joint_def.enable_limit = true;
    joint_def.lower_translation = -PLATE_TRAVEL;
    joint_def.upper_translation = 0.0;
    joint_def.enable_motor = true;
    joint_def.motor_speed = 0.5;
    joint_def.max_motor_force = plate_weight + 0.5 * mover_weight;
    create_prismatic_joint(world, &joint_def)
}

/// The sweeper block (:1274): a crimson cross on a motorized upright revolute.
pub(super) fn build_sweeper(world: &mut World, vis: &mut Vec<VisBody>, ground_id: BodyId) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = SWEEPER_CENTER;
    let sweeper_id = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.filter.category_bits = DYNAMIC_BIT;
    shape_def.base_material.custom_color = HexColor::CRIMSON.0;

    let parts = [
        (0.3f32, 0.6f32, 0.3f32, v3(0.0, 0.6, 0.0)),
        (5.0, 0.25, 0.1, v3(0.0, 1.0, 0.0)),
        (0.1, 0.25, 5.0, v3(0.0, 1.0, 0.0)),
    ];
    for (hx, hy, hz, p) in parts {
        let local = Transform {
            p,
            q: QUAT_IDENTITY,
        };
        let part = make_transformed_box_hull(hx, hy, hz, local);
        create_hull_shape(world, sweeper_id, &shape_def, &part.base);
        vis.push(VisBody::box_local(index_of(sweeper_id), hx, hy, hz, local));
    }

    let upright = make_quat_from_axis_angle(VEC3_AXIS_X, -0.5 * PI);

    let mut joint_def = default_revolute_joint_def();
    joint_def.base.body_id_a = ground_id;
    joint_def.base.body_id_b = sweeper_id;
    joint_def.base.local_frame_a = Transform {
        p: body_get_local_point(world, ground_id, SWEEPER_CENTER),
        q: upright,
    };
    joint_def.base.local_frame_b = Transform {
        p: VEC3_ZERO,
        q: upright,
    };
    joint_def.enable_motor = true;
    joint_def.motor_speed = 1.0;
    joint_def.max_motor_torque = 3000.0;

    create_revolute_joint(world, &joint_def);
}
