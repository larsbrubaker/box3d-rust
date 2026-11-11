//! The `GeometricMoverSample` constructor (`sample_character.cpp:337`, upstream
//! 3362633) in C creation order: the `test_map01` level mesh + three box hulls, the
//! stairs mesh, the torus mesh, the wave height field, the falling tree, the enemy and
//! friendly capsules, the dynamic sphere, the falling box, the `{7,2,-3}` ignore box,
//! and the spring door.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)]

use super::super::colors;
use super::super::hulls::HullVis;
use super::super::{load_level_mesh, new_world, CharacterState, SceneKind, SceneState};
use super::{GeometricMoverScene, MoverImpact};
use crate::character_demo::dynamic::camera::SampleCamera;
use crate::geometric_mover::{GeometricMover, GeometricMoverDef, MoverShapeUserData};
use crate::vis::{hull_triangles, mesh_triangle_edges_transform, VisBody};
use box3d_rust::body::{
    body_apply_mass_from_shapes, body_get_world_center, body_set_angular_velocity,
    body_set_linear_velocity, create_body,
};
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::height_field::create_wave;
use box3d_rust::hull::{create_cylinder, make_box_hull, make_transformed_box_hull};
use box3d_rust::id::BodyId;
use box3d_rust::joint::create_revolute_joint;
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, cross, make_quat_from_axis_angle, normalize, offset_pos,
    sub_pos, to_vec3, Pos, Transform, Vec3, WorldTransform, DEG_TO_RAD, PI, POS_ZERO,
    QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ONE, VEC3_ZERO,
};
use box3d_rust::mesh::create_torus_mesh;
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    create_sphere_shape,
};
use box3d_rust::types::{
    default_body_def, default_revolute_joint_def, default_shape_def, BodyType,
};
use box3d_rust::world::World;

/// C `GeometricMoverSample` mover start (`moverPosition`).
pub(crate) const MOVER_POSITION: Pos = Pos {
    x: 7.5,
    y: 0.75,
    z: 9.0,
};

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

/// The 0.3 m enemy / friendly capsule.
fn npc_capsule() -> Capsule {
    Capsule {
        center1: v3(0.0, -0.5, 0.0),
        center2: v3(0.0, 0.5, 0.0),
        radius: 0.3,
    }
}

/// C `GeometricMoverSample::CreateFallingTree` (:622): one tree from the falling trees
/// benchmark — a stack of 22 shrinking 6-sided cylinders sharing a body, spun about its
/// base. Returns the body and the merged body-local hull triangles.
pub(super) fn create_falling_tree(
    world: &mut World,
    tree_spawn: Pos,
    tree_spin: Vec3,
    user_data: MoverShapeUserData,
) -> (BodyId, Vec<f32>) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = tree_spawn;
    body_def.sleep_threshold = 0.2;
    let tree_id = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.9;
    shape_def.base_material.rolling_resistance = 0.05;
    shape_def.update_body_mass = false;
    shape_def.user_data = user_data.pack();

    let mut tris = Vec::new();
    let mut y = 1.0f32;
    let mut r = 0.75f32;
    let l = 1.5f32;
    for _ in 0..22 {
        if let Some(hull) = create_cylinder(l + 2.0 * r, r, y - r, 6) {
            create_hull_shape(world, tree_id, &shape_def, &hull);
            tris.extend(hull_triangles(&hull));
        }

        y += l + 2.0 * r;
        r *= 0.95;
    }

    body_apply_mass_from_shapes(world, tree_id);

    let center = body_get_world_center(world, tree_id);
    body_set_angular_velocity(world, tree_id, tree_spin);
    body_set_linear_velocity(
        world,
        tree_id,
        cross(tree_spin, sub_pos(center, tree_spawn)),
    );
    (tree_id, tris)
}

/// Build the Geometric Mover level (C `GeometricMoverSample::GeometricMoverSample`).
pub(crate) fn build_mover(test_map_obj: &str, stairs_obj: &str) -> CharacterState {
    let mut world = new_world();
    let mut bodies: Vec<VisBody> = Vec::new();
    let mut ground_edges: Vec<f32> = Vec::new();
    let mut hulls: Vec<HullVis> = Vec::new();

    let mover_position = MOVER_POSITION;

    // C: m_camera->SetView( 120.0f, 30.0f, 5.0f, moverPosition ).
    let camera = SampleCamera::new_view(120.0, 30.0, 5.0, mover_position);

    let mut mover = GeometricMover::new();
    mover.create(&GeometricMoverDef {
        position: mover_position,
        ..GeometricMoverDef::default()
    });

    // --- Level mesh (test_map01) + 3 transformed box hulls on the same body ---
    {
        let body = create_body(&mut world, &default_body_def());
        let shape_def = default_shape_def();

        if let Some(mesh) = load_level_mesh(test_map_obj) {
            create_mesh_shape(&mut world, body, &shape_def, &mesh, VEC3_ONE);
            ground_edges.extend(mesh_triangle_edges_transform(
                &mesh,
                VEC3_ONE,
                Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                },
            ));
        }

        let boxes = [
            (v3(4.0, 1.0, 14.0), QUAT_IDENTITY),
            (v3(4.0, 1.0, 13.95), QUAT_IDENTITY),
            (
                v3(5.8, 1.0, 13.7),
                make_quat_from_axis_angle(VEC3_AXIS_Y, 0.1 * PI),
            ),
        ];
        for (p, q) in boxes {
            let xf = Transform { p, q };
            let hull = make_transformed_box_hull(1.0, 1.0, 1.0, xf);
            create_hull_shape(&mut world, body, &shape_def, &hull.base);
            bodies.push(VisBody::box_local(body.index1 - 1, 1.0, 1.0, 1.0, xf));
        }
    }

    // --- Stairs mesh at {-10,0,0}, scale {0.75,0.75,-1.5} ---
    {
        let mut body_def = default_body_def();
        body_def.position = pos(-10.0, 0.0, 0.0);
        let body = create_body(&mut world, &body_def);
        let scale = v3(0.75, 0.75, -1.5);
        if let Some(mesh) = load_level_mesh(stairs_obj) {
            create_mesh_shape(&mut world, body, &default_shape_def(), &mesh, scale);
            ground_edges.extend(mesh_triangle_edges_transform(
                &mesh,
                scale,
                Transform {
                    p: v3(-10.0, 0.0, 0.0),
                    q: QUAT_IDENTITY,
                },
            ));
        }
    }

    // --- Torus mesh at {-10,1,-8}, rot axisY 0.5π, scale {-0.75,1.5,0.5} ---
    if let Some(torus) = create_torus_mesh(10, 12, 2.0, 1.0) {
        let q = make_quat_from_axis_angle(VEC3_AXIS_Y, 0.5 * PI);
        let mut body_def = default_body_def();
        body_def.position = pos(-10.0, 1.0, -8.0);
        body_def.rotation = q;
        let body = create_body(&mut world, &body_def);
        let scale = v3(-0.75, 1.5, 0.5);
        create_mesh_shape(&mut world, body, &default_shape_def(), &torus, scale);
        ground_edges.extend(mesh_triangle_edges_transform(
            &torus,
            scale,
            Transform {
                p: v3(-10.0, 1.0, -8.0),
                q,
            },
        ));
    }

    // --- Wave height field 50×50 at {20,0,0} ---
    {
        let mut body_def = default_body_def();
        body_def.position = pos(20.0, 0.0, 0.0);
        let body = create_body(&mut world, &body_def);
        let hf = create_wave(50, 50, VEC3_ONE, 0.02, 0.04, true);
        create_height_field_shape(&mut world, body, &default_shape_def(), &hf);
        ground_edges.extend(crate::vis::hf_triangle_edges(&hf, v3(20.0, 0.0, 0.0)));
    }

    // The tree is longer than the height field is wide, so topple it along the diagonal.
    let axis = v3(1.0, 0.0, -1.0);
    let tree_spin = 0.5 * normalize(axis);
    let tree_spawn = pos(22.0, 1.5, 2.0);
    let tree_shape_user_data = MoverShapeUserData {
        max_push: 10.0,
        clip_velocity: true,
        can_mover_push: false,
    };
    let (tree_id, tree_tris) =
        create_falling_tree(&mut world, tree_spawn, tree_spin, tree_shape_user_data);
    hulls.push(HullVis::on_body(tree_id, tree_tris));

    // --- Enemy capsule at {0,1.4,6}: maxPush 1.0, clip true, no push, mediumVioletRed ---
    {
        let mut body_def = default_body_def();
        body_def.position = pos(0.0, 1.4, 6.0);
        let mut shape_def = default_shape_def();
        shape_def.user_data = MoverShapeUserData {
            max_push: 1.0,
            clip_velocity: true,
            can_mover_push: false,
        }
        .pack();
        shape_def.base_material.custom_color = colors::MEDIUM_VIOLET_RED;
        let body = create_body(&mut world, &body_def);
        create_capsule_shape(&mut world, body, &shape_def, &npc_capsule());
        bodies.push(VisBody::capsule_body(body.index1 - 1, &npc_capsule()));
    }

    // --- Friendly capsule at {0,1.4,5}: maxPush 0.01, clip false, no push, team 2 ---
    {
        let mut body_def = default_body_def();
        body_def.position = pos(0.0, 1.4, 5.0);
        let mut shape_def = default_shape_def();
        shape_def.user_data = MoverShapeUserData {
            max_push: 0.01,
            clip_velocity: false,
            can_mover_push: false,
        }
        .pack();
        // C `{ 2u, ~0u, 0 }`: ~0u is a 32-bit all-ones mask.
        shape_def.filter.category_bits = 2;
        shape_def.filter.mask_bits = 0xFFFF_FFFF;
        shape_def.filter.group_index = 0;
        shape_def.base_material.custom_color = colors::LIME_GREEN;
        let body = create_body(&mut world, &body_def);
        create_capsule_shape(&mut world, body, &shape_def, &npc_capsule());
        bodies.push(VisBody::capsule_body(body.index1 - 1, &npc_capsule()));
    }

    // --- Dynamic sphere at {7,5,0} radius 0.5 ---
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(7.0, 5.0, 0.0);
        let body = create_body(&mut world, &body_def);
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        };
        create_sphere_shape(&mut world, body, &default_shape_def(), &sphere);
        bodies.push(VisBody::sphere_body(body.index1 - 1, 0.5));
    }

    // Falls through the mover start position on a loop to feed the mover time of impact.
    let falling_extent = v3(1.0, 0.1, 1.0);
    let falling_spawn = pos(10.5, 30.0, 0.0);
    let falling_spin = VEC3_ZERO;
    let falling_box_id = {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = falling_spawn;
        body_def.angular_velocity = falling_spin;
        let body = create_body(&mut world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.base_material.custom_color = colors::GOLD;
        let hull = make_box_hull(falling_extent.x, falling_extent.y, falling_extent.z);
        create_hull_shape(&mut world, body, &shape_def, &hull.base);
        bodies.push(VisBody::box_body(
            body.index1 - 1,
            falling_extent.x,
            falling_extent.y,
            falling_extent.z,
        ));
        body
    };

    // C DrawCube( m_impactTransform, 2 * m_fallingExtent, yellow alpha 0.4 ): the box pose
    // at the mover's first touch, as a render-only ghost.
    let ghost = make_box_hull(falling_extent.x, falling_extent.y, falling_extent.z);
    let mut ghost_vis = HullVis::render_only(hull_triangles(&ghost.base), colors::YELLOW);
    ghost_vis.alpha = 0.4;
    hulls.push(ghost_vis);

    // --- Ignore box at {7,2,-3} half {0.5,0.25,0.5} floralWhite ---
    let ignore_shape = {
        let mut body_def = default_body_def();
        body_def.position = pos(7.0, 2.0, -3.0);
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.base_material.custom_color = colors::FLORAL_WHITE;
        let hull = make_box_hull(0.5, 0.25, 0.5);
        let sid = create_hull_shape(&mut world, body, &shape_def, &hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 0.25, 0.5));
        sid
    };
    mover.ignore_shape_ids = vec![ignore_shape];

    build_spring_door(&mut world, &mut bodies);

    let scene = GeometricMoverScene {
        mover,
        camera,
        keys: Default::default(),
        tree_id,
        tree_spawn,
        tree_spin,
        tree_timer: 0.0,
        tree_shape_user_data,
        tree_impact: MoverImpact::default(),
        falling_box_id,
        falling_spawn,
        falling_extent,
        falling_spin,
        respawn_timer: 0.0,
        impact_transform: WorldTransform {
            p: POS_ZERO,
            q: QUAT_IDENTITY,
        },
        box_impact: MoverImpact::default(),
    };

    let mut state =
        CharacterState::new(SceneKind::Mover, world, SceneState::Mover(Box::new(scene)));
    state.bodies = bodies;
    state.ground_edges = ground_edges;
    state.hulls = hulls;
    state
}

/// The spring revolute door (:527): a 1000 kg/m³ panel hinged at its left edge.
fn build_spring_door(world: &mut World, bodies: &mut Vec<VisBody>) {
    let ground_id = create_body(world, &default_body_def());

    let door_position = pos(-2.0, 1.6, 0.0);
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = door_position;
    body_def.gravity_scale = 2.0;
    let body_id = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1000.0;
    let hull = make_box_hull(0.75, 1.5, 0.1);
    create_hull_shape(world, body_id, &shape_def, &hull.base);
    bodies.push(VisBody::box_body(body_id.index1 - 1, 0.75, 1.5, 0.1));

    let axis_quat = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    let offset = v3(-0.75, 0.0, 0.0);

    let mut joint_def = default_revolute_joint_def();
    joint_def.base.body_id_a = ground_id;
    joint_def.base.body_id_b = body_id;
    // C `b3ToVec3( bodyDef.position + offset )` (b3Pos + b3Vec3, then demoted).
    joint_def.base.local_frame_a.p = to_vec3(offset_pos(door_position, offset));
    joint_def.base.local_frame_a.q = axis_quat;
    joint_def.base.local_frame_b.p = offset;
    joint_def.base.local_frame_b.q = axis_quat;

    joint_def.enable_limit = true;
    joint_def.lower_angle = DEG_TO_RAD * -90.0;
    joint_def.upper_angle = DEG_TO_RAD * 90.0;
    joint_def.enable_spring = true;
    joint_def.hertz = 1.0;
    joint_def.damping_ratio = 0.5;
    joint_def.enable_motor = false;
    joint_def.max_motor_torque = 100.0;
    joint_def.base.draw_scale = 2.0;

    create_revolute_joint(world, &joint_def);
}
