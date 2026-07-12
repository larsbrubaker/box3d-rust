//! Sensor + events demo — begin/end touch visualization.

use crate::vis::{pos, push_poses, sphere, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::Vec3;
use box3d_rust::shape::{create_hull_shape, create_sphere_shape};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<SensorState>> = const { RefCell::new(None) };
}

struct SensorState {
    world: World,
    bodies: Vec<VisBody>,
    begin_total: u32,
    end_total: u32,
    last_begin: u32,
    last_end: u32,
    inside: bool,
}

fn with_state<R>(f: impl FnOnce(&mut SensorState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("sensor not initialized — call sensor_reset first"))
    })
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

/// Sensor zone + ground + falling spheres that trigger begin/end events.
#[wasm_bindgen]
pub fn sensor_reset() -> u32 {
    STATE.with(|cell| {
        let mut world = new_world();
        let mut bodies = Vec::new();

        // Ground
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        ground_def.position = pos(0.0, -1.0, 0.0);
        let ground = create_body(&mut world, &ground_def);
        let ground_hull = make_box_hull(12.0, 1.0, 12.0);
        create_hull_shape(&mut world, ground, &default_shape_def(), &ground_hull.base);
        bodies.push(VisBody::box_body(ground.index1 - 1, 12.0, 1.0, 12.0));

        // Sensor volume (static, no collision response)
        let mut sensor_body_def = default_body_def();
        sensor_body_def.type_ = BodyType::Static;
        sensor_body_def.position = pos(0.0, 1.5, 0.0);
        let sensor_body = create_body(&mut world, &sensor_body_def);
        let mut sensor_shape = default_shape_def();
        sensor_shape.is_sensor = true;
        sensor_shape.enable_sensor_events = true;
        let sensor_hull = make_box_hull(1.5, 1.5, 1.5);
        create_hull_shape(&mut world, sensor_body, &sensor_shape, &sensor_hull.base);
        bodies.push(VisBody::box_body(sensor_body.index1 - 1, 1.5, 1.5, 1.5));

        // Falling visitors
        for (i, &(x, y, z)) in [
            (-0.4f32, 6.0, 0.0),
            (0.5, 7.5, 0.3),
            (-0.2, 9.0, -0.4),
            (0.8, 10.5, 0.1),
        ]
        .iter()
        .enumerate()
        {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = pos(x, y, z);
            let body = create_body(&mut world, &body_def);
            let mut shape_def = default_shape_def();
            shape_def.density = 1.0;
            shape_def.enable_sensor_events = true;
            let r = 0.35 - i as f32 * 0.04;
            let sph = sphere(r);
            create_sphere_shape(&mut world, body, &shape_def, &sph);
            bodies.push(VisBody::sphere_body(body.index1 - 1, r));
        }

        let count = bodies.len() as u32;
        *cell.borrow_mut() = Some(SensorState {
            world,
            bodies,
            begin_total: 0,
            end_total: 0,
            last_begin: 0,
            last_end: 0,
            inside: false,
        });
        count
    })
}

#[wasm_bindgen]
pub fn sensor_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.world.step(dt, sub_steps);
        let events = state.world.get_sensor_events();
        state.last_begin = events.begin_events.len() as u32;
        state.last_end = events.end_events.len() as u32;
        state.begin_total += state.last_begin;
        state.end_total += state.last_end;
        if state.last_begin > 0 {
            state.inside = true;
        }
        if state.last_end > 0 && state.begin_total <= state.end_total {
            state.inside = false;
        }
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn sensor_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

/// `[begin_total, end_total, last_begin, last_end, inside_flag]`
#[wasm_bindgen]
pub fn sensor_event_stats() -> Vec<f32> {
    with_state(|state| {
        vec![
            state.begin_total as f32,
            state.end_total as f32,
            state.last_begin as f32,
            state.last_end as f32,
            if state.inside { 1.0 } else { 0.0 },
        ]
    })
}
