//! Near-origin Mesh scene builders: Grid, Big Box, Box, Reflection, Hollow Box.
//!
//! Every static mesh ground bakes its triangle edges (through the ground body's
//! world transform) into `state.ground_edges` for the page's wireframe render; the
//! dynamic bodies render from `mesh_poses` / `mesh_styles`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    destroy_drop, new_world, MeshScene, MeshState, SHAPE_BOX, SHAPE_CAPSULE, SHAPE_CYLINDER,
    SHAPE_SPHERE,
};
use crate::vis::{capsule_from_body, mesh_triangle_edges_transform, VisBody};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::{create_cylinder, make_box_hull};
use box3d_rust::human::{create_human, Human, BONE_COUNT};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, Transform, Vec3, PI, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ONE,
};
use box3d_rust::mesh::{create_box_mesh, create_grid_mesh, create_hollow_box_mesh, MeshData};
use box3d_rust::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};

fn pos(x: f32, y: f32, z: f32) -> box3d_rust::math_functions::Pos {
    box3d_rust::math_functions::Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

/// Triangle count + byte count readout (C Grid/Box `DrawTextLine`).
fn mesh_stats2(mesh: &MeshData) -> Vec<f32> {
    vec![mesh.triangle_count as f32, mesh.byte_count as f32]
}

/// Bake a mesh's triangle edges through a static ground body's world transform,
/// appending to `state.ground_edges` (base is zero for these near-origin scenes).
fn bake_ground(state: &mut MeshState, body_index: i32, mesh: &MeshData, scale: Vec3) {
    let xf = get_body_transform(&state.world, body_index);
    let t = Transform {
        p: Vec3 {
            x: xf.p.x as f32,
            y: xf.p.y as f32,
            z: xf.p.z as f32,
        },
        q: xf.q,
    };
    state
        .ground_edges
        .extend_from_slice(&mesh_triangle_edges_transform(mesh, scale, t));
}

/// Append every capsule bone of `human` to the render list (skips null bones).
fn push_human_bones(state: &mut MeshState, human: &Human) {
    for i in 0..BONE_COUNT {
        let body_id = human.bones[i].body_id;
        if body_id.is_null() {
            continue;
        }
        let body_index = body_id.index1 - 1;
        if let Some(cap) = capsule_from_body(&state.world, body_index) {
            state.bodies.push(VisBody::capsule_body(body_index, &cap));
        }
    }
}

// ---------------------------------------------------------------------------
// Grid / Big Box / Box — parametric scenes with the shape picker + Scale X/Z
// ---------------------------------------------------------------------------

/// GridMesh (:25): `b3CreateGridMesh(20,20,1,0,true)`, scale default (2,2,2).
pub(crate) fn build_grid(shape_type: u32, scale_x: f32, scale_z: f32) -> MeshState {
    let mut state = MeshState::base(new_world(), MeshScene::Grid);
    state.shape_type = shape_type;
    state.scale = Vec3 {
        x: scale_x,
        y: 2.0,
        z: scale_z,
    };

    let ground_def = default_body_def();
    let ground = create_body(&mut state.world, &ground_def);
    let mesh = create_grid_mesh(20, 20, 1.0, 0, true).expect("grid mesh");
    let shape_def = default_shape_def();
    let scale = state.scale;
    create_mesh_shape(&mut state.world, ground, &shape_def, &mesh, scale);
    bake_ground(&mut state, ground.index1 - 1, &mesh, scale);
    state.stats = mesh_stats2(&mesh);

    spawn_drop(&mut state);
    state
}

/// BigBoxMesh (:210): `b3CreateBoxMesh({0,-1,0},{50,1,50})`, friction 0.5.
pub(crate) fn build_big_box(shape_type: u32, scale_x: f32, scale_z: f32) -> MeshState {
    let mut state = MeshState::base(new_world(), MeshScene::BigBox);
    state.shape_type = shape_type;
    state.scale = Vec3 {
        x: scale_x,
        y: 1.0,
        z: scale_z,
    };

    let ground_def = default_body_def();
    let ground = create_body(&mut state.world, &ground_def);
    let mesh = create_box_mesh(
        Vec3 {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        },
        Vec3 {
            x: 50.0,
            y: 1.0,
            z: 50.0,
        },
        true,
    )
    .expect("box mesh");
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.5;
    let scale = state.scale;
    create_mesh_shape(&mut state.world, ground, &shape_def, &mesh, scale);
    bake_ground(&mut state, ground.index1 - 1, &mesh, scale);
    state.stats = mesh_stats2(&mesh);

    spawn_drop(&mut state);
    state
}

/// BoxMesh (:384): a 20-unit ground box plus a 45°-rotated 1×1×1 box-mesh ground.
pub(crate) fn build_box(shape_type: u32, scale_x: f32, scale_z: f32) -> MeshState {
    let mut state = MeshState::base(new_world(), MeshScene::Box);
    state.shape_type = shape_type;
    state.scale = Vec3 {
        x: scale_x,
        y: 1.0,
        z: scale_z,
    };

    // AddGroundBox(20): static box at (0,-1,0), half-extents (20,1,20).
    let mut ground_def = default_body_def();
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut state.world, &ground_def);
    let shape_def = default_shape_def();
    let hull = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut state.world, ground, &shape_def, &hull.base);
    state
        .bodies
        .push(VisBody::box_body(ground.index1 - 1, 20.0, 1.0, 20.0));

    // Rotated box-mesh ground (C: bodyDef.position {0,-1,0}, rotation axisY 0.25π).
    let mut mesh_def = default_body_def();
    mesh_def.position = pos(0.0, -1.0, 0.0);
    mesh_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Y, 0.25 * PI);
    let mesh_ground = create_body(&mut state.world, &mesh_def);
    let mesh = create_box_mesh(
        Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        VEC3_ONE,
        true,
    )
    .expect("box mesh");
    let scale = state.scale;
    create_mesh_shape(&mut state.world, mesh_ground, &shape_def, &mesh, scale);
    bake_ground(&mut state, mesh_ground.index1 - 1, &mesh, scale);
    state.stats = mesh_stats2(&mesh);

    spawn_drop(&mut state);
    state
}

/// (Re)spawn the picker drop body for Grid / Big Box / Box (C `Spawn()`).
pub(crate) fn spawn_drop(state: &mut MeshState) {
    destroy_drop(state);

    let (position, angular_damping) = match state.scene {
        MeshScene::Grid => (
            pos(0.1, 1.0, -0.1),
            if state.shape_type == SHAPE_CYLINDER {
                0.1
            } else {
                0.0
            },
        ),
        MeshScene::BigBox => (pos(0.5, 0.0, 0.0), 0.0),
        MeshScene::Box => {
            let y = if state.shape_type == SHAPE_CYLINDER {
                1.0
            } else {
                1.5
            };
            (pos(0.0, y, 0.0), 0.0)
        }
        _ => return,
    };

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = position;
    body_def.angular_damping = angular_damping;
    let body = create_body(&mut state.world, &body_def);
    let body_index = body.index1 - 1;

    let mut shape_def = default_shape_def();
    // Big Box applies rolling resistance 0.05 to every drop shape.
    if state.scene == MeshScene::BigBox {
        shape_def.base_material.rolling_resistance = 0.05;
    }

    match state.shape_type {
        SHAPE_SPHERE => {
            if state.scene != MeshScene::BigBox {
                shape_def.base_material.rolling_resistance = 0.05;
            }
            let sphere = Sphere {
                center: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                radius: 0.5,
            };
            create_sphere_shape(&mut state.world, body, &shape_def, &sphere);
            state.bodies.push(VisBody::sphere_body(body_index, 0.5));
        }
        SHAPE_CAPSULE => {
            let capsule = match state.scene {
                MeshScene::Box => Capsule {
                    center1: Vec3 {
                        x: -0.5,
                        y: 0.0,
                        z: 0.0,
                    },
                    center2: Vec3 {
                        x: 0.5,
                        y: 0.0,
                        z: 0.0,
                    },
                    radius: 0.1,
                },
                _ => Capsule {
                    center1: Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 1.276,
                    },
                    center2: Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.476,
                    },
                    radius: 0.15,
                },
            };
            // Grid: rolling resistance 0.05; Big Box: 0.1; Box: default 0.
            shape_def.base_material.rolling_resistance = match state.scene {
                MeshScene::Grid => 0.05,
                MeshScene::BigBox => 0.1,
                _ => 0.0,
            };
            create_capsule_shape(&mut state.world, body, &shape_def, &capsule);
            state
                .bodies
                .push(VisBody::capsule_body(body_index, &capsule));
        }
        SHAPE_BOX => {
            let hull = make_box_hull(0.5, 0.5, 0.5);
            create_hull_shape(&mut state.world, body, &shape_def, &hull.base);
            state
                .bodies
                .push(VisBody::box_body(body_index, 0.5, 0.5, 0.5));
        }
        _ => {
            // Cylinder (default). Geometry differs per scene.
            let (height, radius, sides) = match state.scene {
                MeshScene::Grid => (1.0f32, 0.25f32, 15),
                MeshScene::BigBox => (0.3, 0.15, 32),
                _ => (1.0, 0.75, 8), // Box
            };
            if state.scene == MeshScene::Grid {
                shape_def.base_material.rolling_resistance = 0.02;
            }
            let hull = create_cylinder(height, radius, 0.0, sides).expect("cylinder hull");
            create_hull_shape(&mut state.world, body, &shape_def, &hull);
            let local = Transform {
                p: Vec3 {
                    x: 0.0,
                    y: 0.5 * height,
                    z: 0.0,
                },
                q: QUAT_IDENTITY,
            };
            state.bodies.push(VisBody::cylinder_local(
                body_index,
                radius,
                0.5 * height,
                local,
                0,
            ));
        }
    }

    state.drop_index = body_index;
}

// ---------------------------------------------------------------------------
// Reflection
// ---------------------------------------------------------------------------

/// MeshReflection (:553): a grid mesh + `building.obj` + its mirrored copy (scale
/// radios, default `{-1,1,1}`) with three dynamic shapes and 20 humans.
pub(crate) fn build_reflection(mirror_scale: Vec3) -> MeshState {
    use box3d_rust::geometry::SurfaceMaterial;

    let mut state = MeshState::base(new_world(), MeshScene::Reflection);

    // Grid mesh ground at the origin.
    let grid_def = default_body_def();
    let grid_body = create_body(&mut state.world, &grid_def);
    let grid = create_grid_mesh(20, 20, 2.0, 2, true).expect("grid mesh");
    let shape_def = default_shape_def();
    create_mesh_shape(&mut state.world, grid_body, &shape_def, &grid, VEC3_ONE);
    bake_ground(&mut state, grid_body.index1 - 1, &grid, VEC3_ONE);

    // Building mesh (embedded `building.obj`) with C's three surface materials.
    let building = crate::obj_loader::load_building_mesh();
    let materials = vec![
        SurfaceMaterial {
            friction: 0.6,
            ..Default::default()
        },
        SurfaceMaterial {
            friction: 0.0,
            restitution: 0.95,
            user_material_id: 1,
            ..Default::default()
        },
        SurfaceMaterial {
            friction: 0.2,
            restitution: 0.2,
            user_material_id: 2,
            ..Default::default()
        },
    ];
    let mut mesh_shape_def = default_shape_def();
    mesh_shape_def.materials = materials;

    // Original building at (-10,0,0).
    let mut building_def = default_body_def();
    building_def.position = pos(-10.0, 0.0, 0.0);
    let b0 = create_body(&mut state.world, &building_def);
    create_mesh_shape(&mut state.world, b0, &mesh_shape_def, &building, VEC3_ONE);
    bake_ground(&mut state, b0.index1 - 1, &building, VEC3_ONE);

    // Mirrored building at (10,0,0), scale (default) {-1,1,1}.
    building_def.position = pos(10.0, 0.0, 0.0);
    let b1 = create_body(&mut state.world, &building_def);
    create_mesh_shape(
        &mut state.world,
        b1,
        &mesh_shape_def,
        &building,
        mirror_scale,
    );
    bake_ground(&mut state, b1.index1 - 1, &building, mirror_scale);

    state.stats = vec![building.triangle_count as f32];

    // Three dynamic shapes dropped in.
    {
        let mut d = default_body_def();
        d.type_ = BodyType::Dynamic;
        d.position = pos(6.0, 15.0, 0.0);
        let body = create_body(&mut state.world, &d);
        let mut sd = default_shape_def();
        sd.base_material.rolling_resistance = 0.2;
        sd.base_material.user_material_id = 42;
        let sphere = Sphere {
            center: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            radius: 0.5,
        };
        create_sphere_shape(&mut state.world, body, &sd, &sphere);
        state
            .bodies
            .push(VisBody::sphere_body(body.index1 - 1, 0.5));
    }
    {
        let mut d = default_body_def();
        d.type_ = BodyType::Dynamic;
        d.position = pos(9.0, 15.0, 0.0);
        let body = create_body(&mut state.world, &d);
        let mut sd = default_shape_def();
        sd.base_material.rolling_resistance = 0.2;
        sd.base_material.user_material_id = 11;
        let capsule = Capsule {
            center1: Vec3 {
                x: -0.5,
                y: 0.5,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
            radius: 0.25,
        };
        create_capsule_shape(&mut state.world, body, &sd, &capsule);
        state
            .bodies
            .push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }
    {
        let mut d = default_body_def();
        d.type_ = BodyType::Dynamic;
        d.position = pos(12.0, 15.0, 0.0);
        let body = create_body(&mut state.world, &d);
        let mut sd = default_shape_def();
        sd.base_material.user_material_id = 555;
        let hull = make_box_hull(0.25, 0.5, 0.75);
        create_hull_shape(&mut state.world, body, &sd, &hull.base);
        state
            .bodies
            .push(VisBody::box_body(body.index1 - 1, 0.25, 0.5, 0.75));
    }

    // 20 humans (friction 5 / hertz 1 / damping 0.7), groupIndex = i.
    for i in 0..20 {
        let position = pos(-14.0 + 1.5 * i as f32, 8.0, 0.0);
        let mut human = Human::default();
        create_human(
            &mut human,
            &mut state.world,
            position,
            5.0,
            1.0,
            0.7,
            i,
            0,
            false,
        );
        push_human_bones(&mut state, &human);
        state.humans.push(human);
    }

    state
}

// ---------------------------------------------------------------------------
// Hollow Box
// ---------------------------------------------------------------------------

/// HollowBox (:1498): a hollow box mesh with 6 cylinders + 8 capsules, all
/// zero-gravity and sleep-disabled so they hang in place for manifold inspection.
pub(crate) fn build_hollow_box() -> MeshState {
    let mut state = MeshState::base(new_world(), MeshScene::HollowBox);

    let ground_def = default_body_def();
    let ground = create_body(&mut state.world, &ground_def);
    let mesh = create_hollow_box_mesh(
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        Vec3 {
            x: 10.0,
            y: 10.0,
            z: 10.0,
        },
    )
    .expect("hollow box mesh");
    let shape_def = default_shape_def();
    create_mesh_shape(&mut state.world, ground, &shape_def, &mesh, VEC3_ONE);
    bake_ground(&mut state, ground.index1 - 1, &mesh, VEC3_ONE);
    state.stats = vec![mesh.triangle_count as f32];

    // Shared dynamic body def: zero gravity, no sleep.
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.gravity_scale = 0.0;
    body_def.enable_sleep = false;
    let shape_def = default_shape_def();

    // 6 cylinders (create_cylinder(1.0, 0.25, 0.0, 8)).
    let cyl = create_cylinder(1.0, 0.25, 0.0, 8).expect("cylinder hull");
    let cyl_positions = [
        (0.0f32, -10.2f32, 0.0f32),
        (0.0, 9.2, 0.0),
        (-9.8, 0.0, 0.0),
        (9.8, 0.0, 0.0),
        (0.0, 0.0, -9.8),
        (0.0, 0.0, 9.8),
    ];
    for (x, y, z) in cyl_positions {
        body_def.position = pos(x, y, z);
        let body = create_body(&mut state.world, &body_def);
        create_hull_shape(&mut state.world, body, &shape_def, &cyl);
        let local = Transform {
            p: Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
            q: QUAT_IDENTITY,
        };
        state.bodies.push(VisBody::cylinder_local(
            body.index1 - 1,
            0.25,
            0.5,
            local,
            0,
        ));
    }

    // 8 capsules ({0,0,0}-{0,1,0}, radius 0.25).
    let capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        radius: 0.25,
    };
    let cap_positions = [
        (0.0f32, -10.2f32, 2.0f32),
        (0.0, 9.2, 2.0),
        (0.0, -9.9, 4.0),
        (0.0, 8.9, 4.0),
        (-9.8, 2.0, 0.0),
        (9.8, 2.0, 0.0),
        (0.0, 2.0, -9.8),
        (0.0, 2.0, 9.8),
    ];
    for (x, y, z) in cap_positions {
        body_def.position = pos(x, y, z);
        let body = create_body(&mut state.world, &body_def);
        create_capsule_shape(&mut state.world, body, &shape_def, &capsule);
        state
            .bodies
            .push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }

    state
}
