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
/// mutations move the simulation, plus the finalized recording when one was passed in. Recording is
/// optional so the same scene serves as the control.
///
/// `repeat_hull` doubles the b3Shape_SetHull call so the caller can prove the shared hull short
/// circuit leaves the stream alone. The C suite asserts that in `AllOps` instead, where the whole
/// stream is exercised at once; the Rust port has no `AllOps`, so the check rides here.
/// (RunGeometryMutatorScene)
fn run_geometry_mutator_scene(
    rec: Option<Recording>,
    mutate: bool,
    repeat_hull: bool,
    mesh_a: &MeshData,
    mesh_b: &MeshData,
    swap_hull: &HullData,
    swap_friction: f32,
) -> (u64, Option<Recording>) {
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

    let finished = if recording {
        world_stop_recording(&mut world)
    } else {
        None
    };

    (hash, finished)
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

    let (control_hash, _) = run_geometry_mutator_scene(
        None,
        false,
        false,
        &mesh_a,
        &mesh_b,
        &swap_hull.base,
        swap_friction,
    );

    let (mutated_hash, rec) = run_geometry_mutator_scene(
        Some(Recording::new(0)),
        true,
        true,
        &mesh_a,
        &mesh_b,
        &swap_hull.base,
        swap_friction,
    );

    let rec = rec.expect("recording active");

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
    let (single_hash, single_rec) = run_geometry_mutator_scene(
        Some(Recording::new(0)),
        true,
        false,
        &mesh_a,
        &mesh_b,
        &swap_hull.base,
        swap_friction,
    );
    assert_eq!(single_hash, mutated_hash);
    let single_rec = single_rec.expect("recording active");
    assert_eq!(single_rec.size(), size);
    assert_eq!(single_rec.data(), rec.data());
}

/// The compound mesh child addition to the C `AllOps` session: a baked compound whose mesh
/// child carries five materials, more than the removed `B3_MAX_COMPOUND_MESH_MATERIALS`, so
/// the packed compound material map rides the recorded geometry blob. The replay world must
/// rebuild the same compound byte for byte, material map included.
#[test]
fn compound_mesh_material_map_round_trip() {
    use crate::compound::{
        create_compound, get_compound_child, CompoundDef, CompoundMeshDef, CompoundSphereDef,
    };
    use crate::math_functions::TRANSFORM_IDENTITY;
    use crate::shape::create_baked_compound_shape;

    let comp_sphere = CompoundSphereDef {
        sphere: Sphere {
            center: VEC3_ZERO,
            radius: 1.0,
        },
        material: default_surface_material(),
    };

    // Mesh child with five materials so the compound material map is recorded
    let compound_mesh_data = create_grid_mesh(1, 5, 1.0, 5, false).expect("grid mesh");
    assert_eq!(compound_mesh_data.material_count, 5);
    let compound_mesh_materials = [default_surface_material(); 5];
    let comp_mesh = CompoundMeshDef {
        mesh_data: &compound_mesh_data,
        transform: TRANSFORM_IDENTITY,
        scale: Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        materials: &compound_mesh_materials,
    };

    let compound = create_compound(&CompoundDef {
        spheres: &[comp_sphere],
        meshes: &[comp_mesh],
        ..Default::default()
    })
    .expect("compound");

    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, Recording::new(0));

    let mut compound_body_def = default_body_def();
    compound_body_def.type_ = BodyType::Static;
    let compound_body_id = create_body(&mut world, &compound_body_def);
    create_baked_compound_shape(
        &mut world,
        compound_body_id,
        &default_shape_def(),
        &compound,
    );

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: 0.5 as _,
        y: 2.5 as _,
        z: 2.5 as _,
    };
    let ball_id = create_body(&mut world, &ball_def);
    let mut ball_shape_def = default_shape_def();
    ball_shape_def.density = 1.0;
    create_sphere_shape(
        &mut world,
        ball_id,
        &ball_shape_def,
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.25,
        },
    );

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while !player.is_at_end() {
        player.step_frame();
    }
    assert!(!player.has_diverged());

    let replay_world = player.world();
    let replay_body = player.get_body_id(0);
    let shapes = body_get_shapes(replay_world, replay_body, 1);
    assert_eq!(shapes.len(), 1);
    let replay_index = get_shape(replay_world, shapes[0]);
    let ShapeGeometry::Compound(replayed) = &replay_world.shapes[replay_index as usize].geometry
    else {
        panic!("replayed shape is not a compound");
    };

    // Child order is capsules, hulls, meshes, spheres: the mesh is child 0.
    let mesh_child = get_compound_child(replayed, 0);
    assert_eq!(mesh_child.material_count, 5);
    assert_eq!(mesh_child.material_indices.len(), 5);
    assert_eq!(replayed.to_bytes(), compound.to_bytes());

    drop(world);
    crate::mesh::destroy_mesh(compound_mesh_data);
}
