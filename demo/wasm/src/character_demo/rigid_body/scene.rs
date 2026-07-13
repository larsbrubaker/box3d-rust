//! RigidBody scene builder (C `RigidBodyCharacter` constructor, `:1316`): the
//! dual feet-box + capsule dynamic character over `test_map01`, `stairs`,
//! `building`, two voxel meshes, the wave height field, and the ramp / platform /
//! step-lip / wall / dynamic-prop obstacle course.

#![allow(clippy::unnecessary_cast)]

use super::super::colors;
use super::super::{load_level_mesh, new_world, CharacterState, SceneKind, SceneState};
use super::character::{
    RigidbodyCharacter, BODY_RADIUS, CHARACTER_GRAVITY, CHARACTER_MASS, FEET_HEIGHT, TOTAL_HEIGHT,
};
use crate::vis::{mesh_triangle_edges_transform, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::height_field::create_wave;
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, Pos, Quat, Transform, Vec3, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ONE,
    VEC3_ZERO,
};
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    create_sphere_shape,
};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use box3d_rust::world::World;
use std::f32::consts::PI;

/// Build the RigidbodyCharacter body (C `RigidbodyCharacter::Initialize`, :702).
fn init_character(world: &mut World, position: Pos) -> (RigidbodyCharacter, Vec<VisBody>) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = position;
    body_def.motion_locks.angular_x = true;
    body_def.motion_locks.angular_y = true;
    body_def.motion_locks.angular_z = true;
    body_def.enable_sleep = false;
    body_def.enable_contact_recycling = false;
    body_def.name = String::from("character");
    body_def.gravity_scale = CHARACTER_GRAVITY / 10.0;
    let body_id = create_body(world, &body_def);
    let body_index = body_id.index1 - 1;

    // Feet box (lower half).
    let feet_half_x = BODY_RADIUS * 0.5;
    let feet_half_y = FEET_HEIGHT * 0.5;
    let feet_half_z = BODY_RADIUS * 0.5;
    let feet_local = Transform {
        p: Vec3 {
            x: 0.0,
            y: -TOTAL_HEIGHT * 0.5 + feet_half_y,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    let feet_box_id = {
        let mut shape_def = default_shape_def();
        shape_def.base_material.friction = 0.0;
        shape_def.base_material.restitution = 0.0;
        shape_def.base_material.custom_color = colors::LIME_GREEN;
        let feet_volume = 8.0 * feet_half_x * feet_half_y * feet_half_z;
        shape_def.density = (CHARACTER_MASS * 0.4) / feet_volume;
        let feet_box = make_transformed_box_hull(feet_half_x, feet_half_y, feet_half_z, feet_local);
        create_hull_shape(world, body_id, &shape_def, &feet_box.base)
    };

    // Body capsule (upper half).
    let capsule_radius = BODY_RADIUS * 0.707;
    let capsule_bottom = -TOTAL_HEIGHT * 0.5 + FEET_HEIGHT * 0.5 + capsule_radius;
    let capsule_top = TOTAL_HEIGHT * 0.5 - capsule_radius;
    let capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: capsule_bottom,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: capsule_top,
            z: 0.0,
        },
        radius: capsule_radius,
    };
    let body_capsule_id = {
        let mut shape_def = default_shape_def();
        shape_def.base_material.friction = 0.0;
        shape_def.base_material.restitution = 0.0;
        shape_def.base_material.custom_color = colors::CORNFLOWER_BLUE;
        let h = capsule_top - capsule_bottom;
        let r = capsule_radius;
        let capsule_volume = PI * r * r * (h + 4.0 * r / 3.0);
        shape_def.density = (CHARACTER_MASS * 0.6) / capsule_volume;
        create_capsule_shape(world, body_id, &shape_def, &capsule)
    };

    let own_shapes = vec![feet_box_id, body_capsule_id];

    let mut character = RigidbodyCharacter {
        body_id,
        feet_box_id,
        own_shapes,
        ground_normal: VEC3_AXIS_Y,
        ground_velocity: VEC3_ZERO,
        jump_cooldown: 0.0,
        on_ground: false,
        sprint: false,
        did_step: false,
        step_position: Pos {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        last_wish_velocity: VEC3_ZERO,
        mass_center_world: position,
        debug_segs: Vec::new(),
        debug_pts: Vec::new(),
    };
    character.update_mass_center(world, 0.0);

    // Render the two body shapes (feet lime, capsule cornflower blue).
    let render = vec![
        VisBody::box_local_colored(
            body_index,
            feet_half_x,
            feet_half_y,
            feet_half_z,
            feet_local,
            colors::LIME_GREEN,
        ),
        VisBody::capsule_colored(body_index, &capsule, colors::CORNFLOWER_BLUE),
    ];
    (character, render)
}

/// Build the full RigidBody scene (C `RigidBodyCharacter` constructor, :1316).
pub(crate) fn build_rigid_body(
    test_map_obj: &str,
    stairs_obj: &str,
    building_obj: &str,
    voxel1_obj: &str,
    voxel2_obj: &str,
) -> CharacterState {
    let mut world = new_world();
    let start = Pos {
        x: 7.5,
        y: 2.0,
        z: 9.0,
    };
    let (character, char_bodies) = init_character(&mut world, start);

    let mut bodies: Vec<VisBody> = char_bodies;
    let mut ground_edges: Vec<f32> = Vec::new();
    let ident = QUAT_IDENTITY;

    // Static collision meshes: (obj, position, scale, three_materials?).
    let meshes: [(&str, Vec3, Vec3, bool); 5] = [
        (test_map_obj, VEC3_ZERO, VEC3_ONE, true),
        (
            stairs_obj,
            Vec3 {
                x: -10.0,
                y: 0.0,
                z: 0.0,
            },
            VEC3_ONE,
            false,
        ),
        (
            building_obj,
            Vec3 {
                x: -5.0,
                y: 0.0,
                z: -10.0,
            },
            VEC3_ONE,
            false,
        ),
        (
            voxel1_obj,
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: -10.0,
            },
            VEC3_ONE,
            false,
        ),
        (
            voxel2_obj,
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: 10.0,
            },
            VEC3_ONE,
            false,
        ),
    ];
    for (obj, position, scale, mats) in meshes {
        let Some(mesh) = load_level_mesh(obj) else {
            continue;
        };
        let mut shape_def = default_shape_def();
        if mats {
            shape_def.materials = super::super::ground_materials();
        }
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: position.x as _,
            y: position.y as _,
            z: position.z as _,
        };
        let body = create_body(&mut world, &body_def);
        create_mesh_shape(&mut world, body, &shape_def, &mesh, scale);
        ground_edges.extend(mesh_triangle_edges_transform(
            &mesh,
            scale,
            Transform {
                p: position,
                q: ident,
            },
        ));
    }

    // Height field 50×50 at {20,0,0}.
    {
        let mut shape_def = default_shape_def();
        shape_def.materials = super::super::ground_materials();
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 20.0,
            y: 0.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        let hf = create_wave(50, 50, VEC3_ONE, 0.02, 0.04, true);
        create_height_field_shape(&mut world, body, &shape_def, &hf);
        ground_edges.extend(crate::vis::hf_triangle_edges(
            &hf,
            Vec3 {
                x: 20.0,
                y: 0.0,
                z: 0.0,
            },
        ));
    }

    // Hull obstacle course (C :1405-1485), friction 0.6.
    let push_hull = |world: &mut World,
                     bodies: &mut Vec<VisBody>,
                     pos: Vec3,
                     rot: Quat,
                     half: Vec3,
                     color: u32,
                     dynamic: bool| {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: pos.x as _,
            y: pos.y as _,
            z: pos.z as _,
        };
        body_def.rotation = rot;
        if dynamic {
            body_def.type_ = BodyType::Dynamic;
        }
        let body = create_body(world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.base_material.friction = 0.6;
        shape_def.base_material.custom_color = color;
        let hull = make_box_hull(half.x, half.y, half.z);
        create_hull_shape(world, body, &shape_def, &hull.base);
        bodies.push(VisBody::box_colored(
            body.index1 - 1,
            half.x,
            half.y,
            half.z,
            color,
        ));
    };

    let deg = |d: f32| d * PI / 180.0;

    // Ramp (tilted box).
    push_hull(
        &mut world,
        &mut bodies,
        Vec3 {
            x: 6.0,
            y: 1.0,
            z: 4.0,
        },
        make_quat_from_axis_angle(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            deg(-20.0),
        ),
        Vec3 {
            x: 3.0,
            y: 0.15,
            z: 1.5,
        },
        colors::OLIVE_DRAB,
        false,
    );
    // Steep ramp.
    push_hull(
        &mut world,
        &mut bodies,
        Vec3 {
            x: 6.0,
            y: 2.0,
            z: -4.0,
        },
        make_quat_from_axis_angle(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            deg(-50.0),
        ),
        Vec3 {
            x: 2.5,
            y: 0.15,
            z: 1.5,
        },
        colors::INDIAN_RED,
        false,
    );
    // Elevated platforms with gaps.
    for i in 0..3 {
        push_hull(
            &mut world,
            &mut bodies,
            Vec3 {
                x: -4.0 + 3.5 * i as f32,
                y: 1.2,
                z: -5.0,
            },
            ident,
            Vec3 {
                x: 1.2,
                y: 0.15,
                z: 1.2,
            },
            colors::SLATE_GRAY,
            false,
        );
    }
    // Step-height test (increasing lip heights).
    for i in 0..5 {
        let lip_height = 0.05 + 0.08 * i as f32;
        push_hull(
            &mut world,
            &mut bodies,
            Vec3 {
                x: -8.0,
                y: lip_height,
                z: -1.0 + 2.0 * i as f32,
            },
            ident,
            Vec3 {
                x: 1.0,
                y: lip_height,
                z: 0.6,
            },
            colors::CORNFLOWER_BLUE,
            false,
        );
    }
    // Wall.
    push_hull(
        &mut world,
        &mut bodies,
        Vec3 {
            x: 0.0,
            y: 1.5,
            z: 10.0,
        },
        ident,
        Vec3 {
            x: 4.0,
            y: 1.5,
            z: 0.2,
        },
        colors::DARK_SLATE_GRAY,
        false,
    );
    // Dynamic boxes to push around.
    for i in 0..3 {
        push_hull(
            &mut world,
            &mut bodies,
            Vec3 {
                x: 3.0 + 1.5 * i as f32,
                y: 0.5,
                z: 0.0,
            },
            ident,
            Vec3 {
                x: 0.4,
                y: 0.4,
                z: 0.4,
            },
            colors::GOLD,
            true,
        );
    }
    // Dynamic sphere.
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: -3.0,
            y: 1.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.base_material.friction = 0.6;
        shape_def.base_material.custom_color = colors::ORANGE;
        create_sphere_shape(
            &mut world,
            body,
            &shape_def,
            &Sphere {
                center: VEC3_ZERO,
                radius: 0.5,
            },
        );
        bodies.push(VisBody::sphere_colored(
            body.index1 - 1,
            0.5,
            colors::ORANGE,
        ));
    }

    let mut state = CharacterState::new(
        SceneKind::RigidBody,
        world,
        SceneState::RigidBody(character),
    );
    state.bodies = bodies;
    state.ground_edges = ground_edges;
    state.third_person = true;
    state
}
