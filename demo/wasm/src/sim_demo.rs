//! Live World::step demos: Bodies and Stacking scenes.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::geometry::Sphere;
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{Pos, VEC3_ZERO};
use box3d_rust::shape::{create_hull_shape, create_sphere_shape};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static SIM: RefCell<Option<SimState>> = const { RefCell::new(None) };
}

struct SimBody {
    body_index: i32,
    half_extents: [f32; 3],
    /// 0 = hull box, 1 = sphere (radius in half_extents[0])
    kind: u8,
}

struct SimState {
    world: World,
    bodies: Vec<SimBody>,
}

fn with_sim<R>(f: impl FnOnce(&mut SimState) -> R) -> R {
    SIM.with(|cell| {
        let mut slot = cell.borrow_mut();
        let sim = slot
            .as_mut()
            .expect("sim not initialized — call sim_reset_* first");
        f(sim)
    })
}

fn push_dynamic_box(
    sim: &mut SimState,
    x: f32,
    y: f32,
    z: f32,
    hx: f32,
    hy: f32,
    hz: f32,
    density: f32,
    friction: f32,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    };
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = density;
    shape_def.base_material.friction = friction;
    let hull = make_box_hull(hx, hy, hz);
    create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [hx, hy, hz],
        kind: 0,
    });
}

fn push_dynamic_sphere(sim: &mut SimState, x: f32, y: f32, z: f32, radius: f32, density: f32) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    };
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = density;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius,
    };
    create_sphere_shape(&mut sim.world, body_id, &shape_def, &sphere);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [radius, radius, radius],
        kind: 1,
    });
}

fn add_ground(sim: &mut SimState, half_extent: f32) {
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = Pos {
        x: 0.0 as _,
        y: (-half_extent) as _,
        z: 0.0 as _,
    };
    let ground = create_body(&mut sim.world, &ground_def);
    let shape_def = default_shape_def();
    let hull = make_box_hull(half_extent * 2.0, half_extent, half_extent * 2.0);
    create_hull_shape(&mut sim.world, ground, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: ground.index1 - 1,
        half_extents: [half_extent * 2.0, half_extent, half_extent * 2.0],
        kind: 0,
    });
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = box3d_rust::math_functions::Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

/// Bodies demo: ground + falling cube (HelloWorld) + a few companions.
#[wasm_bindgen]
pub fn sim_reset_bodies() -> u32 {
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
        };
        add_ground(&mut sim, 10.0);
        push_dynamic_box(&mut sim, 0.0, 4.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.3);
        push_dynamic_box(&mut sim, -3.0, 6.0, 0.0, 0.5, 0.5, 0.5, 1.0, 0.4);
        push_dynamic_box(&mut sim, 3.0, 8.0, 1.0, 0.75, 0.4, 0.75, 1.5, 0.5);
        push_dynamic_sphere(&mut sim, 1.5, 10.0, -1.0, 0.6, 1.0);
        push_dynamic_sphere(&mut sim, -1.5, 12.0, 0.5, 0.4, 0.8);
        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Stacking demo: vertical box stack (mirrors sample BoxStack, fewer boxes for the browser).
#[wasm_bindgen]
pub fn sim_reset_stacking(count: u32) -> u32 {
    let n = count.clamp(1, 24);
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
        };
        add_ground(&mut sim, 20.0);
        let a = 0.5f32;
        for i in 0..n {
            let y = 1.5 * a + 2.5 * a * i as f32;
            push_dynamic_box(&mut sim, 0.0, y, 0.0, a, a, a, 1.0, 0.5);
        }
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Advance the simulation. Returns body count.
#[wasm_bindgen]
pub fn sim_step(dt: f32, sub_steps: i32) -> u32 {
    with_sim(|sim| {
        sim.world.step(dt, sub_steps);
        sim.bodies.len() as u32
    })
}

/// Interleaved body poses: for each body
/// `[px, py, pz, qx, qy, qz, qw, hx, hy, hz, kind]`.
#[wasm_bindgen]
pub fn sim_body_poses() -> Vec<f32> {
    with_sim(|sim| {
        let mut out = Vec::with_capacity(sim.bodies.len() * 11);
        for b in &sim.bodies {
            let xf = get_body_transform(&sim.world, b.body_index);
            out.push(xf.p.x as f32);
            out.push(xf.p.y as f32);
            out.push(xf.p.z as f32);
            out.push(xf.q.v.x);
            out.push(xf.q.v.y);
            out.push(xf.q.v.z);
            out.push(xf.q.s);
            out.push(b.half_extents[0]);
            out.push(b.half_extents[1]);
            out.push(b.half_extents[2]);
            out.push(b.kind as f32);
        }
        out
    })
}

/// Body count in the current scene.
#[wasm_bindgen]
pub fn sim_body_count() -> u32 {
    with_sim(|sim| sim.bodies.len() as u32)
}
