//! Mesh / height-field contact world tests (task-6).
//!
//! Height-field settle, edge-weld roll, and hull-on-mesh coverage. Upstream
//! commit c37cfe4 moved `TestMeshDrop` out of `test_world.c` into
//! `test_determinism.c` as the CCD-based `MeshDropTest`; that determinism scene
//! (shared/stability.c) is ported separately, so the old sleep-gate test is
//! dropped here to match the C reorganization.

#![allow(clippy::unnecessary_cast)] // Pos is f64 under double-precision

use crate::body::{body_get_position, create_body};
use crate::geometry::Sphere;
use crate::height_field::create_grid;
use crate::hull::make_box_hull;
use crate::math_functions::{Pos, Vec3, VEC3_ONE, VEC3_ZERO};
use crate::mesh::create_grid_mesh;
use crate::shape::{
    create_height_field_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::{world_get_body_events, World};

/// Sphere dropped on a height field settles and sleeps.
#[test]
fn test_height_field_drop_settles() {
    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);

    let hf = create_grid(
        17,
        17,
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        false,
    );
    create_height_field_shape(&mut world, ground, &default_shape_def(), &hf);

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: 0.0 as _,
        y: 3.0 as _,
        z: 0.0 as _,
    };
    let ball = create_body(&mut world, &ball_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    create_sphere_shape(
        &mut world,
        ball,
        &shape_def,
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
    );

    let time_step = 1.0 / 60.0;
    let step_limit = 300;
    let mut step_index = 0;
    while step_index < step_limit {
        world.step(time_step, 4);
        if world_get_body_events(&world).is_empty() {
            break;
        }
        step_index += 1;
    }

    assert!(
        step_index < step_limit,
        "height-field drop never slept (step_index={step_index})"
    );
}

/// Rolling across a welded flat grid does not snag on interior edges.
#[test]
fn test_mesh_edge_weld_roll() {
    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);

    let mesh = create_grid_mesh(20, 20, 1.0, 1, true).expect("grid mesh");
    create_mesh_shape(&mut world, ground, &default_shape_def(), &mesh, VEC3_ONE);

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: -8.0 as _,
        y: 1.0 as _,
        z: 0.0 as _,
    };
    ball_def.linear_velocity = Vec3 {
        x: 6.0,
        y: 0.0,
        z: 0.0,
    };
    ball_def.angular_velocity = Vec3 {
        x: 0.0,
        y: 0.0,
        z: -12.0,
    };
    let ball = create_body(&mut world, &ball_def);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.base_material.rolling_resistance = 0.05;
    create_sphere_shape(
        &mut world,
        ball,
        &shape_def,
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.4,
        },
    );

    let time_step = 1.0 / 60.0;
    let mut min_y = f32::MAX;
    let mut max_y = f32::MIN;
    let mut last_x = -8.0f32;

    for _ in 0..180 {
        world.step(time_step, 4);
        let pos = body_get_position(&world, ball);
        let y = pos.y as f32;
        let x = pos.x as f32;
        min_y = min_y.min(y);
        max_y = max_y.max(y);
        // Must keep making progress along +x (not stuck on a ghost edge).
        assert!(
            x + 0.01 >= last_x || y > 0.8,
            "sphere snagged: x={x} last_x={last_x} y={y}"
        );
        last_x = last_x.max(x);
    }

    // Sphere stayed near the plane (no hop-snag from interior edges).
    assert!(
        max_y - min_y < 0.75,
        "edge weld roll y span too large: min={min_y} max={max_y}"
    );
    assert!(
        last_x > 0.0,
        "sphere did not roll across the mesh (x={last_x})"
    );
}

/// Hull dropped on a flat welded mesh should settle (isolates hull-vs-mesh path).
#[test]
fn test_hull_on_flat_mesh_settles() {
    let mut world = World::new(&default_world_def());
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let mesh = create_grid_mesh(8, 8, 1.0, 1, true).expect("grid");
    create_mesh_shape(&mut world, ground, &default_shape_def(), &mesh, VEC3_ONE);

    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    box_def.position = Pos {
        x: 0.0 as _,
        y: 2.0 as _,
        z: 0.0 as _,
    };
    let box_id = create_body(&mut world, &box_def);
    let hull = make_box_hull(0.2, 0.2, 0.2);
    let mut sd = default_shape_def();
    sd.density = 1.0;
    create_hull_shape(&mut world, box_id, &sd, &hull.base);

    let mut slept = false;
    for _ in 0..240 {
        world.step(1.0 / 60.0, 4);
        if world_get_body_events(&world).is_empty() {
            slept = true;
            break;
        }
    }
    assert!(slept, "hull on flat mesh never slept");
    let pos = body_get_position(&world, box_id);
    assert!(
        (pos.y as f32) > 0.0 && (pos.y as f32) < 1.0,
        "hull did not settle on mesh (y={})",
        pos.y as f32
    );
}
