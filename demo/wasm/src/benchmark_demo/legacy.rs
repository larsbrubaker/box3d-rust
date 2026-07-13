//! The two previously verified benchmark scenes, re-homed onto the shared
//! `BenchScene` / `VisBody` model with their geometry unchanged: Junkyard
//! (`benchmarks.c` CreateJunkyard :796 / StepJunkyard :872) and Falling Trees
//! (`benchmarks.c` CreateTrees :681).

use super::{empty_scene, new_world, BenchKind, BenchScene, JunkyardAnim};
use crate::vis::{mesh_triangle_edges, VisBody};
use box3d_rust::body::{
    body_apply_mass_from_shapes, body_get_world_center, body_set_angular_velocity,
    body_set_linear_velocity, body_set_target_transform, create_body,
};
use box3d_rust::hull::{create_cylinder, create_rock, make_box_hull, make_offset_box_hull};
use box3d_rust::math_functions::{
    compute_cos_sin, cross, sub_pos, Pos, Transform, Vec3, WorldTransform, PI, QUAT_IDENTITY,
    VEC3_ONE,
};
use box3d_rust::mesh::create_wave_mesh;
use box3d_rust::shape::{create_hull_shape, create_mesh_shape};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};

/// `CreateJunkyard` (`benchmarks.c` :796). Walled arena, a grid of `create_rock`
/// hulls, and a kinematic cylinder pusher orbiting the floor. `count = DEBUG 2 :
/// 24` rock layers; the browser uses **2** (release 24 layers = 24×21×21 ≈ 10 584
/// rocks does not hold interactively).
pub(crate) fn build_junkyard() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::Junkyard);

    let mut ground_def = default_body_def();
    ground_def.position = Pos {
        x: 0.0,
        y: -1.0,
        z: 0.0,
    };
    let ground = create_body(&mut scene.world, &ground_def);
    let shape_def = default_shape_def();

    // Floor + four perimeter walls (b3MakeOffsetBoxHull, on the ground body).
    let floor = make_box_hull(120.0, 1.0, 120.0);
    create_hull_shape(&mut scene.world, ground, &shape_def, &floor.base);
    scene
        .bodies
        .push(VisBody::box_body(ground.index1 - 1, 120.0, 1.0, 120.0));

    let walls = [
        (
            1.0f32,
            8.0f32,
            50.0f32,
            Vec3 {
                x: -50.0,
                y: 8.0,
                z: 0.0,
            },
        ),
        (
            1.0,
            8.0,
            50.0,
            Vec3 {
                x: 50.0,
                y: 8.0,
                z: 0.0,
            },
        ),
        (
            50.0,
            8.0,
            1.0,
            Vec3 {
                x: 0.0,
                y: 8.0,
                z: -50.0,
            },
        ),
        (
            50.0,
            8.0,
            1.0,
            Vec3 {
                x: 0.0,
                y: 8.0,
                z: 50.0,
            },
        ),
    ];
    for (hx, hy, hz, offset) in walls {
        let hull = make_offset_box_hull(hx, hy, hz, offset);
        create_hull_shape(&mut scene.world, ground, &shape_def, &hull.base);
        scene.bodies.push(VisBody::box_local(
            ground.index1 - 1,
            hx,
            hy,
            hz,
            Transform {
                p: offset,
                q: QUAT_IDENTITY,
            },
        ));
    }

    // Rock grid (create_rock(1.5)): DEBUG 2 layers × 21×21 = 882 rocks.
    let rock_hull = create_rock(1.5).expect("junkyard rock");
    let count = 2i32;
    let height = 24.0f32;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    for y in 0..count {
        for x in 0..=20 {
            for z in 0..=20 {
                body_def.position = Pos {
                    x: -40.0 + 4.0 * x as f32,
                    y: 4.0 * y as f32 + height + 1.0,
                    z: -40.0 + 4.0 * z as f32,
                };
                let body = create_body(&mut scene.world, &body_def);
                create_hull_shape(&mut scene.world, body, &shape_def, &rock_hull);
                scene
                    .bodies
                    .push(VisBody::icosahedron_colored(body.index1 - 1, 1.5, 0));
            }
        }
    }

    // Kinematic pusher cylinder (create_cylinder(24, 4, 0, 16)) orbiting at r=35.
    let pusher_radius = 35.0f32;
    let pusher_height = 24.0f32;
    let pusher_r = 4.0f32;
    let cyl = create_cylinder(pusher_height, pusher_r, 0.0, 16).expect("pusher cylinder");
    let mut kin_def = default_body_def();
    kin_def.type_ = BodyType::Kinematic;
    kin_def.position = Pos {
        x: pusher_radius,
        y: 0.0,
        z: 0.0,
    };
    let pusher = create_body(&mut scene.world, &kin_def);
    create_hull_shape(&mut scene.world, pusher, &default_shape_def(), &cyl);
    scene.bodies.push(VisBody::cylinder_local(
        pusher.index1 - 1,
        pusher_r,
        pusher_height * 0.5,
        Transform {
            p: Vec3 {
                x: 0.0,
                y: pusher_height * 0.5,
                z: 0.0,
            },
            q: QUAT_IDENTITY,
        },
        0,
    ));

    scene.junkyard = Some(JunkyardAnim {
        pusher_id: pusher,
        degrees: 0.0,
        radius: pusher_radius,
    });
    scene
}

/// `StepJunkyard` (`benchmarks.c` :872) — drive the kinematic pusher around the
/// floor at a fixed 1/60 timestep.
pub(crate) fn step_junkyard(scene: &mut BenchScene, _dt: f32) {
    let Some(anim) = scene.junkyard.as_mut() else {
        return;
    };
    let time_step = 1.0f32 / 60.0;
    const OMEGA: f32 = -6.0;
    anim.degrees += OMEGA * time_step;
    let cs = compute_cos_sin(anim.degrees * PI / 180.0);
    let r = anim.radius;
    let target = WorldTransform {
        p: Pos {
            x: r * cs.cosine,
            y: 0.0,
            z: r * cs.sine,
        },
        q: QUAT_IDENTITY,
    };
    let pusher = anim.pusher_id;
    body_set_target_transform(&mut scene.world, pusher, target, time_step, false);
}

/// `CreateTrees` (`benchmarks.c` :681). `grid_size` mirrors the C radio (100/50/25
/// cm cell) → `scale` 1/2/4; mesh is `scale·150 × scale·200`. `bodyCount = DEBUG
/// 10 : 50`; the browser uses **10** (the 22 tapering hulls per tree is fixed in
/// C). z start -15 (DEBUG; release -70).
pub(crate) fn build_trees(grid_size: u32) -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::Trees);

    let scale = match grid_size {
        25 => 4i32,
        50 => 2i32,
        _ => 1i32,
    };
    let x_count = scale * 150;
    let z_count = scale * 200;
    let cell_width = 1.0f32 / scale as f32;
    let mesh = create_wave_mesh(x_count, z_count, cell_width, 0.4, 0.05, 0.1).expect("trees mesh");

    let mut ground_def = default_body_def();
    let ground = create_body(&mut scene.world, &ground_def);
    create_mesh_shape(
        &mut scene.world,
        ground,
        &default_shape_def(),
        &mesh,
        VEC3_ONE,
    );
    scene.ground_edges = mesh_triangle_edges(&mesh, VEC3_ONE);
    ground_def.type_ = BodyType::Dynamic;

    let body_count = 10i32;
    let hull_count = 22i32;
    let mut hulls = Vec::with_capacity(hull_count as usize);
    let mut segs = Vec::with_capacity(hull_count as usize);
    let mut y = 1.0f32;
    let mut r = 0.75f32;
    let l = 1.5f32;
    for _ in 0..hull_count {
        let y_offset = y - r;
        let height = l + 2.0 * r;
        hulls.push(create_cylinder(height, r, y_offset, 6).expect("tree cylinder"));
        segs.push((r, y_offset + height * 0.5, height * 0.5));
        y += l + 2.0 * r;
        r *= 0.95;
    }

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.sleep_threshold = 0.2;
    body_def.rotation = QUAT_IDENTITY;

    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.9;
    shape_def.base_material.rolling_resistance = 0.05;
    shape_def.update_body_mass = false;
    shape_def.density = 1.0;

    let mut angular_velocity = -0.5f32;
    let mut z = -15.0f32;
    for body_index in 0..body_count {
        body_def.position = Pos { x: 0.0, y: 1.0, z };
        let body = create_body(&mut scene.world, &body_def);
        for hull in &hulls {
            create_hull_shape(&mut scene.world, body, &shape_def, hull);
        }
        for &(radius, y_center, half_h) in &segs {
            scene.bodies.push(VisBody::cylinder_local(
                body.index1 - 1,
                radius,
                half_h,
                Transform {
                    p: Vec3 {
                        x: 0.0,
                        y: y_center,
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                },
                0,
            ));
        }

        let velocity_scale = 0.5 + (0.5 * body_index as f32) / body_count as f32;
        body_apply_mass_from_shapes(&mut scene.world, body);
        let center = body_get_world_center(&scene.world, body);
        let omega = Vec3 {
            x: 0.0,
            y: 0.0,
            z: velocity_scale * angular_velocity,
        };
        let v = cross(omega, sub_pos(center, body_def.position));
        body_set_angular_velocity(&mut scene.world, body, omega);
        body_set_linear_velocity(&mut scene.world, body, v);

        z += 3.0;
        angular_velocity = -angular_velocity;
    }
    scene
}
