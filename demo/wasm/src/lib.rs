// WASM bindings for the browser demos. Every value shown on the demo site is
// computed by the ported Rust code, never re-implemented in JavaScript.

use wasm_bindgen::prelude::*;

mod benchmark_demo;
mod bodies_demo;
mod character_demo;
mod determinism_demo;
mod draw_data;
mod height_field_demo;
mod hull_demo;
mod interact;
mod joint_demo;
mod joint_drive;
mod joint_gear;
mod manifold_demo;
mod mesh_demo;
mod obj_loader;
mod query_demo;
mod ragdoll_demo;
mod rng;
mod sensor_demo;
#[macro_use]
mod shell;
mod continuous_scenes;
mod shapes_demo;
mod sim_compound;
mod sim_continuous;
mod sim_demo;
mod stacking_scenes;
mod village;
mod vis;
mod world_demo;

use box3d_rust::math_functions as m;

/// Version of the box3d-rust port this wasm build was compiled from.
#[wasm_bindgen]
pub fn version() -> String {
    box3d_rust::VERSION.to_string()
}

/// Deterministic cosine/sine from the ported `b3ComputeCosSin`. Returns [cos, sin].
#[wasm_bindgen]
pub fn compute_cos_sin(radians: f32) -> Vec<f32> {
    let cs = m::compute_cos_sin(radians);
    vec![cs.cosine, cs.sine]
}

/// Deterministic arctangent from the ported `b3Atan2`.
#[wasm_bindgen]
pub fn atan2(y: f32, x: f32) -> f32 {
    m::atan2(y, x)
}

/// Build a regular polygon in the XY plane, rotated by `angle` around Z, centered
/// at (cx, cy). Uses ported `compute_cos_sin` + `make_quat_from_axis_angle` +
/// `transform_point`. Returns interleaved [x0, y0, x1, y1, ...].
#[wasm_bindgen]
pub fn polygon_points(sides: u32, radius: f32, angle: f32, cx: f32, cy: f32) -> Vec<f32> {
    let q = m::make_quat_from_axis_angle(m::VEC3_AXIS_Z, angle);
    let t = m::Transform {
        p: m::Vec3 {
            x: cx,
            y: cy,
            z: 0.0,
        },
        q,
    };

    let mut out = Vec::with_capacity(2 * sides as usize);
    for i in 0..sides {
        let vertex_angle = 2.0 * m::PI * i as f32 / sides as f32;
        let cs = m::compute_cos_sin(vertex_angle);
        let local = m::Vec3 {
            x: radius * cs.cosine,
            y: radius * cs.sine,
            z: 0.0,
        };
        let world = m::transform_point(t, local);
        out.push(world.x);
        out.push(world.y);
    }
    out
}
