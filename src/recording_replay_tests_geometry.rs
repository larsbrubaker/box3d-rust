// Replay tests for the shape geometry mutator ops (recording minor version 4).
// `include!`d into `recording_replay_tests.rs`, so it shares that module's imports.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::{body_get_shapes, body_is_valid};
use crate::geometry::default_surface_material;
use crate::hull::HullData;
use crate::mesh::{create_grid_mesh, MeshData};
use crate::recording::{hash_world_state, RecPlayer};
use crate::shape::{
    create_mesh_shape, get_shape, shape_get_hull, shape_get_mesh_surface_material, shape_set_hull,
    shape_set_mesh, shape_set_mesh_material, ShapeGeometry,
};

/// A box sliding across a mesh floor, with the option to swap both geometries and retune a
/// per-triangle material part way through. Returns the final state hash so the caller can prove the
/// mutations move the simulation. Recording is optional so the same scene serves as the control.
///
/// `repeat_hull` doubles the b3Shape_SetHull call so the caller can prove the shared hull short
/// circuit leaves the stream alone. The C suite asserts that in `AllOps` instead, where the whole
/// stream is exercised at once; the Rust port has no `AllOps`, so the check rides here.
/// (RunGeometryMutatorScene)
fn run_geometry_mutator_scene(
    rec: Option<&mut Recording>,
    mutate: bool,
    repeat_hull: bool,
    mesh_a: &MeshData,
    mesh_b: &MeshData,
    swap_hull: &HullData,
    swap_friction: f32,
) -> u64 {
    let mut world = World::new(&default_world_def());

    let recording = rec.is_some();
    if let Some(rec) = rec {
        world_start_recording(&mut world, rec);
    }

    // The mesh body is created first so its ordinal is stable for the read back after replay.
    let mut mesh_materials = [default_surface_material(), default_surface_material()];
    mesh_materials[1].friction = 0.05;

    let mut mesh_body_def = default_body_def();
    mesh_body_def.type_ = BodyType::Static;
    let mesh_body_id = create_body(&mut world, &mesh_body_def);

    let mut mesh_shape_def = default_shape_def();
    mesh_shape_def.materials = mesh_materials.to_vec();
    let mesh_shape_id = create_mesh_shape(
        &mut world,
        mesh_body_id,
        &mesh_shape_def,
        mesh_a,
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
    );

    let mut box_body_def = default_body_def();
    box_body_def.type_ = BodyType::Dynamic;
    box_body_def.position = Pos {
        x: -2.0 as _,
        y: 2.0 as _,
        z: 0.0 as _,
    };
    box_body_def.linear_velocity = Vec3 {
        x: 4.0,
        y: 0.0,
        z: 0.0,
    };
    let box_body_id = create_body(&mut world, &box_body_def);

    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let mut box_shape_def = default_shape_def();
    box_shape_def.density = 1.0;
    let box_shape_id = create_hull_shape(&mut world, box_body_id, &box_shape_def, &box_hull.base);

    let time_step = 1.0 / 60.0;
    for _ in 0..10 {
        world.step(time_step, 4);
    }

    if mutate {
        shape_set_hull(&mut world, box_shape_id, swap_hull);
        if repeat_hull {
            // Geometry swaps intern into the registry at the record site. The repeated SetHull takes
            // the shared hull short circuit, which changes nothing and so must leave the stream alone.
            shape_set_hull(&mut world, box_shape_id, swap_hull);
        }
        shape_set_mesh(
            &mut world,
            mesh_shape_id,
            mesh_b,
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
        );

        let mut grippy = default_surface_material();
        grippy.friction = swap_friction;
        shape_set_mesh_material(&mut world, mesh_shape_id, grippy, 1);
    }

    for _ in 0..30 {
        world.step(time_step, 4);
    }

    let hash = hash_world_state(&world);

    if recording {
        world_stop_recording(&mut world);
    }

    hash
}

/// Swapping a shape's hull or mesh, or retuning one of its per-triangle materials, is a world
/// mutation like any other and has to ride the stream. The geometry pair interns into the registry
/// at the record site so replay rebuilds the same shape instead of running on the geometry it was
/// created with. The control run proves the mutations move the simulation, so the state hash gate
/// has teeth, and the read back covers each op on its own where dynamics alone would not.
/// (GeometryMutatorReplay)
#[test]
fn shape_geometry_mutator_replay() {
    // Two flat floors with different triangulations, both carrying two material slots so the
    // material index stays live across the swap.
    let mesh_a = create_grid_mesh(8, 8, 2.0, 2, false).expect("mesh a");
    let mesh_b = create_grid_mesh(12, 12, 1.5, 2, false).expect("mesh b");
    assert_ne!(mesh_a.triangle_count, mesh_b.triangle_count);

    let swap_hull = make_box_hull(0.25, 1.5, 0.25);
    let swap_friction = 0.95;

    let control_hash = run_geometry_mutator_scene(
        None,
        false,
        false,
        &mesh_a,
        &mesh_b,
        &swap_hull.base,
        swap_friction,
    );

    let mut rec = Recording::new(0);
    let mutated_hash = run_geometry_mutator_scene(
        Some(&mut rec),
        true,
        true,
        &mesh_a,
        &mesh_b,
        &swap_hull.base,
        swap_friction,
    );

    // Without this the replay gate below could pass on a recording that never carried the ops.
    assert_ne!(mutated_hash, control_hash);

    let size = rec.size();
    assert!(size > 0);

    assert!(validate_replay(rec.data(), 1));
    assert!(validate_replay(rec.data(), 4));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while !player.is_at_end() {
        player.step_frame();
    }
    assert!(!player.has_diverged());

    // Body ordinals follow creation order in the replayed world.
    let replay_mesh_body = player.get_body_id(0);
    let replay_box_body = player.get_body_id(1);
    let replay_world = player.world();
    assert!(body_is_valid(replay_world, replay_mesh_body) && body_is_valid(replay_world, replay_box_body));

    let mesh_shapes = body_get_shapes(replay_world, replay_mesh_body, 1);
    assert_eq!(mesh_shapes.len(), 1);
    let replay_mesh_shape = mesh_shapes[0];
    let box_shapes = body_get_shapes(replay_world, replay_box_body, 1);
    assert_eq!(box_shapes.len(), 1);
    let replay_box_shape = box_shapes[0];

    let replay_hull = shape_get_hull(replay_world, replay_box_shape).expect("replay hull");
    assert_eq!(replay_hull.hash, swap_hull.base.hash);

    // b3Shape_GetMesh has no Rust equivalent; read the geometry the shape holds directly.
    let replay_mesh_index = get_shape(replay_world, replay_mesh_shape);
    let ShapeGeometry::Mesh { data, .. } = &replay_world.shapes[replay_mesh_index as usize].geometry
    else {
        panic!("replayed shape is not a mesh");
    };
    assert_eq!(data.hash, mesh_b.hash);
    assert_eq!(data.triangle_count, mesh_b.triangle_count);

    let replay_material = shape_get_mesh_surface_material(replay_world, replay_mesh_shape, 1);
    assert_eq!(replay_material.friction, swap_friction);

    // The shared hull short circuit must not have written a second op: the stream recorded without
    // the repeat has to match byte for byte.
    let mut single_rec = Recording::new(0);
    let single_hash = run_geometry_mutator_scene(
        Some(&mut single_rec),
        true,
        false,
        &mesh_a,
        &mesh_b,
        &swap_hull.base,
        swap_friction,
    );
    assert_eq!(single_hash, mutated_hash);
    assert_eq!(single_rec.size(), size);
    assert_eq!(single_rec.data(), rec.data());
}
