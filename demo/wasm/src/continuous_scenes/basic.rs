//! Continuous batch-2 scenes with box/mesh grounds: Spinning Stick, Needle Mesh,
//! Hump Mesh, Is Fast, Stall. Shared state and helpers live in the parent module.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{bake_ground_mesh, clear_ground_edges, p};
use crate::rng::XorShift32;
use crate::sim_demo::{add_ground, new_sim, stop_recording_if_any, with_sim, SimBody, SIM};
use box3d_rust::body::{create_body, destroy_body, make_body_id};
use box3d_rust::core::{get_stall_threshold, set_stall_threshold};
use box3d_rust::hull::{create_rock, make_box_hull};
use box3d_rust::math_functions::{compute_cos_sin, Vec3, PI, VEC3_ONE, VEC3_ZERO};
use box3d_rust::mesh::{create_mesh, create_torus_mesh, MeshData, MeshDef};
use box3d_rust::shape::{create_hull_shape, create_mesh_shape};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use wasm_bindgen::prelude::*;

// ---------------------------------------------------------------------------
// Spinning Stick (sample_continuous.cpp SpinningStick, :149)
// ---------------------------------------------------------------------------

/// Continuous / Spinning Stick — a long thin box dropped spinning onto a thin
/// wall. The angular velocity is `RandomVec3(-range, range)` with `range = 50`,
/// drawn from the C sample RNG seeded to `RAND_SEED = 12345` (the Sample ctor
/// resets `g_randomSeed`, `sample.cpp:332`).
#[wasm_bindgen]
pub fn sim_reset_spinning_stick() -> u32 {
    clear_ground_edges();
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 10.0);

        let mut body_def = default_body_def();
        body_def.position = p(0.0, 0.5, 0.0);
        let wall = create_body(&mut sim.world, &body_def);
        let wall_hull = make_box_hull(0.125, 0.5, 10.0);
        let mut shape_def = default_shape_def();
        create_hull_shape(&mut sim.world, wall, &shape_def, &wall_hull.base);
        sim.bodies.push(SimBody {
            body_index: wall.index1 - 1,
            half_extents: [0.125, 0.5, 10.0],
            kind: 0,
            local: None,
        });

        body_def.type_ = BodyType::Dynamic;
        body_def.position = p(0.0, 20.0, 0.5);
        body_def.linear_velocity = Vec3 {
            x: 0.0,
            y: -100.0,
            z: 0.0,
        };
        // C: b3Vec3 range = {50,50,50}; omega = RandomVec3(-range, range).
        let mut rng = XorShift32::with_seed(12345);
        let omega = Vec3 {
            x: rng.range(-50.0, 50.0),
            y: rng.range(-50.0, 50.0),
            z: rng.range(-50.0, 50.0),
        };
        body_def.angular_velocity = omega;
        let stick = create_body(&mut sim.world, &body_def);
        let stick_box = make_box_hull(2.0, 0.1, 0.1);
        shape_def.base_material.rolling_resistance = 0.1;
        create_hull_shape(&mut sim.world, stick, &shape_def, &stick_box.base);
        sim.bodies.push(SimBody {
            body_index: stick.index1 - 1,
            half_extents: [2.0, 0.1, 0.1],
            kind: 0,
            local: None,
        });

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

// ---------------------------------------------------------------------------
// Needle Mesh (sample_continuous.cpp NeedleMesh, :281)
// ---------------------------------------------------------------------------

/// Build a "needle" fan mesh — a tall apex vertex over a `slices`-gon base,
/// bit-for-bit port of `NeedleMesh::CreateNeedle` (`sample_continuous.cpp:330`).
fn create_needle(height: f32, radius: f32, center: Vec3, slices: i32) -> MeshData {
    let vertex_count = slices + 1;
    let mut vertices = Vec::with_capacity(vertex_count as usize);
    let mut alpha = 0.0f32;
    let delta_alpha = 2.0 * PI / slices as f32;

    vertices.push(Vec3 {
        x: center.x,
        y: height + center.y,
        z: center.z,
    });
    for _ in 1..vertex_count {
        let cs = compute_cos_sin(alpha);
        vertices.push(Vec3 {
            x: radius * cs.cosine + center.x,
            y: center.y,
            z: radius * cs.sine + center.z,
        });
        alpha += delta_alpha;
    }

    let triangle_count = slices;
    let mut indices = Vec::with_capacity((3 * triangle_count) as usize);
    let mut index1 = vertex_count - 1;
    for index in 0..triangle_count {
        let index2 = index + 1;
        indices.push(0);
        indices.push(index2);
        indices.push(index1);
        index1 = index2;
    }

    let def = MeshDef {
        vertices,
        indices,
        use_median_split: true,
        ..Default::default()
    };
    create_mesh(&def, None).expect("needle mesh")
}

/// Continuous / Needle Mesh — a flat plate dropped onto four thin needle meshes.
#[wasm_bindgen]
pub fn sim_reset_needle_mesh() -> u32 {
    clear_ground_edges();
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();

        let slices = 8;
        let needles = [
            create_needle(
                0.99,
                0.1,
                Vec3 {
                    x: 0.2,
                    y: 0.0,
                    z: 0.2,
                },
                slices,
            ),
            create_needle(
                1.01,
                0.1,
                Vec3 {
                    x: 0.2,
                    y: 0.0,
                    z: -0.2,
                },
                slices,
            ),
            create_needle(
                0.98,
                0.1,
                Vec3 {
                    x: -0.2,
                    y: 0.0,
                    z: -0.2,
                },
                slices,
            ),
            create_needle(
                1.02,
                0.1,
                Vec3 {
                    x: -0.2,
                    y: 0.0,
                    z: 0.2,
                },
                slices,
            ),
        ];

        let body_def = default_body_def();
        let ground = create_body(&mut sim.world, &body_def);
        let shape_def = default_shape_def();
        for needle in &needles {
            create_mesh_shape(&mut sim.world, ground, &shape_def, needle, VEC3_ONE);
            bake_ground_mesh(needle, VEC3_ZERO);
        }

        let mut dyn_def = default_body_def();
        dyn_def.type_ = BodyType::Dynamic;
        dyn_def.position = p(0.0, 5.0, 0.0);
        dyn_def.linear_velocity = Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        };
        let body = create_body(&mut sim.world, &dyn_def);
        let plate = make_box_hull(0.3, 0.01, 0.3);
        create_hull_shape(&mut sim.world, body, &shape_def, &plate.base);
        sim.bodies.push(SimBody {
            body_index: body.index1 - 1,
            half_extents: [0.3, 0.01, 0.3],
            kind: 0,
            local: None,
        });

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}
// ---------------------------------------------------------------------------
// Hump Mesh (sample_continuous.cpp HumpMesh, :799)
// ---------------------------------------------------------------------------

/// Build the 6-vertex "hump" ridge mesh, port of `HumpMesh::CreateHump` (:834).
fn create_hump(cell_width: f32) -> MeshData {
    let mut vertices = [VEC3_ZERO; 6];
    let mut index = 0usize;
    let mut x = -0.5 * cell_width;
    for _ix in 0..=1 {
        let mut z = -cell_width;
        for iz in 0..=2 {
            vertices[index] = Vec3 { x, y: 0.0, z };
            if iz == 1 {
                vertices[index].y = 0.05 * cell_width;
            }
            z += cell_width;
            index += 1;
        }
        x += cell_width;
    }

    let mut indices = [0i32; 12];
    let mut idx = 0usize;
    for _ix in 0..1 {
        for iz in 0..2 {
            let index1 = iz;
            let index2 = index1 + 1;
            let index3 = index2 + 3;
            let index4 = index3 - 1;
            indices[idx] = index1;
            indices[idx + 1] = index2;
            indices[idx + 2] = index3;
            indices[idx + 3] = index3;
            indices[idx + 4] = index4;
            indices[idx + 5] = index1;
            idx += 6;
        }
    }

    let def = MeshDef {
        vertices: vertices.to_vec(),
        indices: indices.to_vec(),
        use_median_split: true,
        identify_edges: true,
        ..Default::default()
    };
    create_mesh(&def, None).expect("hump mesh")
}

/// Continuous / Hump Mesh — a thin plate slams down onto a ridge mesh.
#[wasm_bindgen]
pub fn sim_reset_hump_mesh() -> u32 {
    clear_ground_edges();
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 20.0);

        let hump = create_hump(8.0);
        let body_def = default_body_def();
        let ground = create_body(&mut sim.world, &body_def);
        let shape_def = default_shape_def();
        create_mesh_shape(&mut sim.world, ground, &shape_def, &hump, VEC3_ONE);
        bake_ground_mesh(&hump, VEC3_ZERO);

        let mut dyn_def = default_body_def();
        dyn_def.type_ = BodyType::Dynamic;
        dyn_def.position = p(0.0, 5.0, 0.0);
        dyn_def.linear_velocity = Vec3 {
            x: 0.0,
            y: -50.0,
            z: 0.0,
        };
        let body = create_body(&mut sim.world, &dyn_def);
        let plate = make_box_hull(0.5, 0.05, 1.0);
        create_hull_shape(&mut sim.world, body, &shape_def, &plate.base);
        sim.bodies.push(SimBody {
            body_index: body.index1 - 1,
            half_extents: [0.5, 0.05, 1.0],
            kind: 0,
            local: None,
        });

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}
// ---------------------------------------------------------------------------
// Is Fast (sample_continuous.cpp IsFast, :911)
// ---------------------------------------------------------------------------

/// Continuous / Is Fast — three tall boxes spin about different axes with
/// gravity disabled, exercising the "is fast" fast-body classifier.
#[wasm_bindgen]
pub fn sim_reset_is_fast() -> u32 {
    clear_ground_edges();
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 40.0);

        let shape_def = default_shape_def();
        let specs: [(f32, Vec3); 3] = [
            (
                -12.0,
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 4.0,
                },
            ),
            (
                0.0,
                Vec3 {
                    x: 0.0,
                    y: 4.0,
                    z: 0.0,
                },
            ),
            (
                12.0,
                Vec3 {
                    x: 4.0,
                    y: 0.0,
                    z: 0.0,
                },
            ),
        ];
        for (x, omega) in specs {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = p(x, 20.0, 0.0);
            body_def.gravity_scale = 0.0;
            body_def.angular_velocity = omega;
            let body = create_body(&mut sim.world, &body_def);
            let box_hull = make_box_hull(0.5, 10.0, 0.5);
            create_hull_shape(&mut sim.world, body, &shape_def, &box_hull.base);
            sim.bodies.push(SimBody {
                body_index: body.index1 - 1,
                half_extents: [0.5, 10.0, 0.5],
                kind: 0,
                local: None,
            });
        }

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}
// ---------------------------------------------------------------------------
// Stall (sample_continuous.cpp Stall, :998)
// ---------------------------------------------------------------------------

/// Continuous / Stall — a fast rock bullet is fired at a dense torus mesh to
/// stress the continuous solver's swept-AABB query.
#[wasm_bindgen]
pub fn sim_reset_stall() -> u32 {
    clear_ground_edges();
    // C `Stall` ctor: b3SetStallThreshold(0.001f) — log any CCD step over 1 ms.
    // (The serial port emits no per-step wall-clock CCD log, but the threshold is
    // set and read back for the on-screen readout, matching the C sample setup.)
    set_stall_threshold(0.001);
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 500.0);

        // Dense torus at y = 2 (200×200, radius 2, thickness 1).
        let mut body_def = default_body_def();
        body_def.position = p(0.0, 2.0, 0.0);
        let torus_body = create_body(&mut sim.world, &body_def);
        let mesh = create_torus_mesh(200, 200, 2.0, 1.0).expect("torus mesh");
        let shape_def = default_shape_def();
        create_mesh_shape(&mut sim.world, torus_body, &shape_def, &mesh, VEC3_ONE);
        bake_ground_mesh(
            &mesh,
            Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
        );

        sim.bullet_body_index = -1;
        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Fire (or re-fire) the Stall rock bullet — C `Stall::Launch` (:1029): a
/// `b3CreateRock(0.25)` hull as an `is_bullet` body at 600 m/s.
#[wasm_bindgen]
pub fn sim_cont_stall_launch() -> u32 {
    with_sim(|sim| {
        sim.grab.end(&mut sim.world);
        if sim.bullet_body_index >= 0 {
            let index = sim.bullet_body_index;
            let id = make_body_id(&sim.world, index);
            destroy_body(&mut sim.world, id);
            sim.bodies.retain(|b| b.body_index != index);
            sim.bullet_body_index = -1;
        }

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = true;
        body_def.position = p(0.0, 1.0, -10.0);
        body_def.linear_velocity = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 600.0,
        };
        body_def.angular_velocity = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 20.0,
        };
        let bullet = create_body(&mut sim.world, &body_def);
        let shape_def = default_shape_def();
        let rock = create_rock(0.25).expect("rock hull");
        create_hull_shape(&mut sim.world, bullet, &shape_def, &rock);
        sim.bullet_body_index = bullet.index1 - 1;
        // The rock is a 10-point hull; rendered as a sphere of the same radius.
        sim.bodies.push(SimBody {
            body_index: sim.bullet_body_index,
            half_extents: [0.25, 0.25, 0.25],
            kind: 1,
            local: None,
        });
        sim.bodies.len() as u32
    })
}

/// The active CCD stall threshold in milliseconds (C `b3GetStallThreshold` × 1000,
/// the same scale solver.c/shape.c compare a step's duration against). The Stall
/// scene sets it to 1.0 ms in [`sim_reset_stall`]; the page shows it as the readout
/// the C sample configures.
#[wasm_bindgen]
pub fn sim_cont_stall_threshold_ms() -> f32 {
    1000.0 * get_stall_threshold()
}
