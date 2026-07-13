//! Mesh Drop (`MeshDrop`, :413) and Mesh Drop Unit Test (`shared/stability.c`
//! CreateMeshDrop): 1024 small shapes rain into a wave-mesh basin. Shared state
//! and helpers live in the parent module.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{bake_ground_mesh, clear_ground_edges, p, with_extra};
use crate::rng::XorShift32;
use crate::sim_demo::{new_sim, stop_recording_if_any, with_sim, SimBody, SIM};
use box3d_rust::body::{body_get_world_center, create_body, make_body_id};
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::{create_cylinder, make_box_hull, make_transformed_box_hull};
use box3d_rust::math_functions::{Pos, Transform, Vec3, QUAT_IDENTITY, VEC3_ONE, VEC3_ZERO};
use box3d_rust::mesh::create_wave_mesh;
use box3d_rust::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use box3d_rust::world::world_get_body_events;
use wasm_bindgen::prelude::*;

#[cfg(test)]
#[path = "mesh_drop_tests.rs"]
mod mesh_drop_tests;
// ---------------------------------------------------------------------------
// Mesh Drop (sample_continuous.cpp MeshDrop, :413) and Unit Test (:744)
// ---------------------------------------------------------------------------

const MESH_DROP_GRID: i32 = 32;

/// Push one Mesh Drop projectile of the requested shape type at `pos` with the
/// given velocities. Matches C `MeshDrop::Generate` shape selection (:581).
fn push_mesh_drop_body(
    sim: &mut crate::sim_demo::SimState,
    shape: u32,
    position: Pos,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = position;
    body_def.linear_velocity = linear_velocity;
    body_def.angular_velocity = angular_velocity;
    let body = create_body(&mut sim.world, &body_def);

    let mut shape_def = default_shape_def();
    // C: rollingResistance = shapeType == capsule ? 0.4 : 0.1.
    shape_def.base_material.rolling_resistance = if shape == 1 { 0.4 } else { 0.1 };
    // Don't allow shapes to collide with each other (category 2, mask 1).
    shape_def.filter.category_bits = 2;
    shape_def.filter.mask_bits = 1;

    let index = body.index1 - 1;
    match shape {
        1 => {
            let capsule = Capsule {
                center1: Vec3 {
                    x: 0.0,
                    y: -0.2,
                    z: 0.0,
                },
                center2: Vec3 {
                    x: 0.0,
                    y: 0.2,
                    z: 0.0,
                },
                radius: 0.05,
            };
            create_capsule_shape(&mut sim.world, body, &shape_def, &capsule);
            sim.bodies.push(SimBody {
                body_index: index,
                half_extents: [0.05, 0.2, 0.05],
                kind: 2,
                local: None,
            });
        }
        2 => {
            // C `m_cylinder = b3CreateCylinder(0.4, 0.05, 0.0, 6)`: hex cylinder
            // along +Y spanning [0, 0.4]; render centered via a +0.2y local.
            let cyl = create_cylinder(0.4, 0.05, 0.0, 6).expect("cylinder hull");
            create_hull_shape(&mut sim.world, body, &shape_def, &cyl);
            sim.bodies.push(SimBody {
                body_index: index,
                half_extents: [0.05, 0.2, 0.0],
                kind: 3,
                local: Some(Transform {
                    p: Vec3 {
                        x: 0.0,
                        y: 0.2,
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                }),
            });
        }
        3 => {
            let sphere = Sphere {
                center: VEC3_ZERO,
                radius: 0.05,
            };
            create_sphere_shape(&mut sim.world, body, &shape_def, &sphere);
            sim.bodies.push(SimBody {
                body_index: index,
                half_extents: [0.05, 0.05, 0.05],
                kind: 1,
                local: None,
            });
        }
        _ => {
            let box_hull = make_box_hull(0.02, 0.2, 0.04);
            create_hull_shape(&mut sim.world, body, &shape_def, &box_hull.base);
            sim.bodies.push(SimBody {
                body_index: index,
                half_extents: [0.02, 0.2, 0.04],
                kind: 0,
                local: None,
            });
        }
    }
}

/// Build the Mesh Drop wave-mesh ground plus four bounding walls on one static
/// body. Matches `MeshDrop::CreateGround` (:459).
fn build_mesh_drop_ground(sim: &mut crate::sim_demo::SimState, amplitude: f32) {
    let body_def = default_body_def();
    let ground = create_body(&mut sim.world, &body_def);
    let ground_index = ground.index1 - 1;

    let grid_count = 40;
    let cell_width = 1.0f32;
    let mesh = create_wave_mesh(grid_count, grid_count, cell_width, amplitude, 0.1, 0.2)
        .expect("wave mesh");
    let mut shape_def = default_shape_def();
    shape_def.filter.category_bits = 1;
    create_mesh_shape(&mut sim.world, ground, &shape_def, &mesh, VEC3_ONE);
    bake_ground_mesh(&mesh, VEC3_ZERO);

    let extent = 0.5 * grid_count as f32 * cell_width;
    let half_height = 1.0f32;
    let walls: [(Vec3, f32, f32, f32); 4] = [
        (
            Vec3 {
                x: 0.0,
                y: half_height,
                z: -extent,
            },
            extent,
            half_height,
            0.1,
        ),
        (
            Vec3 {
                x: 0.0,
                y: half_height,
                z: extent,
            },
            extent,
            half_height,
            0.1,
        ),
        (
            Vec3 {
                x: -extent,
                y: half_height,
                z: 0.0,
            },
            0.1,
            half_height,
            extent,
        ),
        (
            Vec3 {
                x: extent,
                y: half_height,
                z: 0.0,
            },
            0.1,
            half_height,
            extent,
        ),
    ];
    for (wp, hx, hy, hz) in walls {
        let transform = Transform {
            p: wp,
            q: QUAT_IDENTITY,
        };
        let wall_box = make_transformed_box_hull(hx, hy, hz, transform);
        create_hull_shape(&mut sim.world, ground, &shape_def, &wall_box.base);
        sim.bodies.push(SimBody {
            body_index: ground_index,
            half_extents: [hx, hy, hz],
            kind: 0,
            local: Some(transform),
        });
    }
}

/// Spawn the 32×32 Mesh Drop projectile grid with velocities from `seed`.
/// Matches `MeshDrop::Generate` (:522) with `simulateAll = true`.
fn build_mesh_drop_bodies(sim: &mut crate::sim_demo::SimState, shape: u32, seed: u32) {
    let mut rng = XorShift32::with_seed(seed);
    let grid = MESH_DROP_GRID;
    for i in 0..grid {
        for j in 0..grid {
            let linear_velocity = Vec3 {
                x: rng.range(-1.0, 1.0),
                y: rng.range(-1.0, 1.0),
                z: rng.range(-1.0, 1.0),
            };
            let angular_velocity = Vec3 {
                x: rng.range(-5.0, 5.0),
                y: rng.range(-5.0, 5.0),
                z: rng.range(-5.0, 5.0),
            };
            let position = p(
                0.5 * (i as f32 - 0.5 * grid as f32),
                5.0,
                0.5 * (j as f32 - 0.5 * grid as f32),
            );
            push_mesh_drop_body(sim, shape, position, linear_velocity, angular_velocity);
        }
    }
}

fn build_mesh_drop_scene() -> u32 {
    clear_ground_edges();
    let (amplitude, shape, seed) = with_extra(|e| (e.md_amplitude, e.md_shape, e.md_seed));
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        // C MeshDrop ctor: GetGuiDraw()->forceScale = 0.1 (sample_continuous.cpp:430).
        // Re-applied after new_sim() restores the default draw scales, so the page
        // needn't know about it.
        crate::interact::set_draw_scales(1.0, 0.1);
        build_mesh_drop_ground(&mut sim, amplitude);
        build_mesh_drop_bodies(&mut sim, shape, seed);
        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Continuous / Mesh Drop — 1024 small shapes rain into a wave-mesh basin.
#[wasm_bindgen]
pub fn sim_reset_mesh_drop() -> u32 {
    build_mesh_drop_scene()
}

/// Mesh Drop / "Type" combo (C `DrawControls`, :615). 0 box, 1 capsule, 2
/// cylinder, 3 sphere. Rebuilds the projectile grid (ground is unchanged).
#[wasm_bindgen]
pub fn sim_cont_mesh_drop_set_type(shape: u32) -> u32 {
    with_extra(|e| e.md_shape = shape.min(3));
    build_mesh_drop_scene()
}

/// Mesh Drop / "Amplitude" slider (C `DrawControls`, :622). Rebuilds ground + grid.
#[wasm_bindgen]
pub fn sim_cont_mesh_drop_set_amplitude(amplitude: f32) -> u32 {
    with_extra(|e| e.md_amplitude = amplitude.clamp(0.0, 1.0));
    build_mesh_drop_scene()
}

/// Mesh Drop / "Generate" button (C `DrawControls`, :628). Reseeds and rebuilds.
#[wasm_bindgen]
pub fn sim_cont_mesh_drop_generate() -> u32 {
    with_extra(|e| {
        // Advance the seed one XorShift step so each press gives a fresh pile
        // (C uses `g_randomSeed = b3GetTicks()`, non-deterministic).
        let mut rng = XorShift32::with_seed(e.md_seed);
        rng.next_int();
        e.md_seed = rng.seed();
    });
    build_mesh_drop_scene()
}

/// Number of bodies that moved on the last step (`b3BodyEvents.moveCount`). The
/// Mesh Drop "Auto Generate" toggle regenerates when this reaches 0 (C `Step`,
/// :685) — the JS side drives the loop.
#[wasm_bindgen]
pub fn sim_cont_mesh_drop_move_count() -> u32 {
    with_sim(|sim| world_get_body_events(&sim.world).len() as u32)
}

/// Continuous / Mesh Drop Unit Test — the deterministic `CreateMeshDrop`
/// (`shared/stability.c`): wave-mesh ground (no walls) + a 32×32 box grid seeded
/// with the fixed `g_randomSeed = 3963634789`.
#[wasm_bindgen]
pub fn sim_reset_mesh_drop_unit() -> u32 {
    clear_ground_edges();
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        // C MeshDropUnitTest ctor: GetGuiDraw()->forceScale = 0.1
        // (sample_continuous.cpp:753). Re-applied after new_sim() restores defaults.
        crate::interact::set_draw_scales(1.0, 0.1);

        // Ground: wave mesh only (no bounding walls), category 1.
        let body_def = default_body_def();
        let ground = create_body(&mut sim.world, &body_def);
        let mesh = create_wave_mesh(40, 40, 1.0, 0.5, 0.1, 0.2).expect("wave mesh");
        let mut ground_shape = default_shape_def();
        ground_shape.filter.category_bits = 1;
        create_mesh_shape(&mut sim.world, ground, &ground_shape, &mesh, VEC3_ONE);
        bake_ground_mesh(&mesh, VEC3_ZERO);

        // 32×32 box grid, fixed seed 3963634789.
        build_mesh_drop_bodies(&mut sim, 0, 3963634789);

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Lowest projectile mass-center height, for the Unit Test failure readout
/// (C `MeshDropUnitTest::Step`: any body escaping is a failure). Returns the
/// minimum `y` over tracked dynamic bodies (skips the static walls/ground).
#[wasm_bindgen]
pub fn sim_cont_min_body_height() -> f32 {
    with_sim(|sim| {
        let mut min_y = f32::MAX;
        for b in &sim.bodies {
            let id = make_body_id(&sim.world, b.body_index);
            let center = body_get_world_center(&sim.world, id);
            min_y = min_y.min(center.y as f32);
        }
        if min_y == f32::MAX {
            0.0
        } else {
            min_y
        }
    })
}
