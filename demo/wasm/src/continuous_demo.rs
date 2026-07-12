//! Continuous collision demo — bullet vs thin wall with CCD toggle.

use crate::vis::{pos, push_poses, sphere, vec3, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::Vec3;
use box3d_rust::shape::{create_hull_shape, create_sphere_shape};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<CcdState>> = const { RefCell::new(None) };
}

struct CcdState {
    world: World,
    bodies: Vec<VisBody>,
    continuous: bool,
    bullet_x: f32,
}

fn with_state<R>(f: impl FnOnce(&mut CcdState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("ccd not initialized — call continuous_reset first"))
    })
}

fn build_scene(continuous: bool) -> CcdState {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    def.enable_continuous = continuous;
    let mut world = World::new(&def);
    world.enable_continuous(continuous);

    let mut bodies = Vec::new();

    // Thin wall at x = 0
    let mut wall_def = default_body_def();
    wall_def.type_ = BodyType::Static;
    wall_def.position = pos(0.0, 0.0, 0.0);
    let wall = create_body(&mut world, &wall_def);
    let wall_hull = make_box_hull(0.05, 5.0, 5.0);
    create_hull_shape(&mut world, wall, &default_shape_def(), &wall_hull.base);
    bodies.push(VisBody::box_body(wall.index1 - 1, 0.05, 5.0, 5.0));

    // Floor guide (visual only, thin)
    let mut floor_def = default_body_def();
    floor_def.type_ = BodyType::Static;
    floor_def.position = pos(0.0, -2.5, 0.0);
    let floor = create_body(&mut world, &floor_def);
    let floor_hull = make_box_hull(8.0, 0.05, 3.0);
    create_hull_shape(&mut world, floor, &default_shape_def(), &floor_hull.base);
    bodies.push(VisBody::box_body(floor.index1 - 1, 8.0, 0.05, 3.0));

    // Fast bullet sphere from the right
    let mut bullet_def = default_body_def();
    bullet_def.type_ = BodyType::Dynamic;
    bullet_def.is_bullet = true;
    bullet_def.gravity_scale = 0.0;
    bullet_def.position = pos(3.0, 0.0, 0.0);
    bullet_def.linear_velocity = vec3(-80.0, 0.0, 0.0);
    let bullet = create_body(&mut world, &bullet_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let sph = sphere(0.15);
    create_sphere_shape(&mut world, bullet, &shape_def, &sph);
    bodies.push(VisBody::sphere_body(bullet.index1 - 1, 0.15));

    CcdState {
        world,
        bodies,
        continuous,
        bullet_x: 3.0,
    }
}

/// Reset CCD scene. `continuous` enables world continuous collision.
#[wasm_bindgen]
pub fn continuous_reset(continuous: bool) -> u32 {
    STATE.with(|cell| {
        let state = build_scene(continuous);
        let n = state.bodies.len() as u32;
        *cell.borrow_mut() = Some(state);
        n
    })
}

#[wasm_bindgen]
pub fn continuous_set_enabled(continuous: bool) {
    with_state(|state| {
        state.continuous = continuous;
        state.world.enable_continuous(continuous);
    });
}

#[wasm_bindgen]
pub fn continuous_is_enabled() -> bool {
    with_state(|state| state.continuous)
}

#[wasm_bindgen]
pub fn continuous_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.world.step(dt, sub_steps);
        if let Some(b) = state.bodies.last() {
            let xf = box3d_rust::body::get_body_transform(&state.world, b.body_index);
            state.bullet_x = xf.p.x as f32;
        }
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn continuous_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

/// `[bullet_x, continuous_flag]` — readout helpers.
#[wasm_bindgen]
pub fn continuous_status() -> Vec<f32> {
    with_state(|state| {
        vec![
            state.bullet_x,
            if state.continuous { 1.0 } else { 0.0 },
        ]
    })
}
