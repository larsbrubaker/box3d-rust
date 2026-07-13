//! Height Field demo — a 1:1 port of `sample_mesh.cpp` `HeightField` (:750).
//!
//! A wave (or flat grid) height field is built with `b3CreateWave` / `b3CreateGrid`,
//! attached to a static ground body centered on the origin, and probed every frame
//! by a ray cast (`b3World_CastRayClosest`) or, when the radius is non-zero, a sphere
//! shape cast (`b3World_CastShape`) — exactly the two branches of `HeightField::Step`.
//!
//! The C sample builds a 400×400 field in release and 10×10 in debug. Rendering the
//! full wireframe of a 400×400 field (~1.9 M edges) is infeasible in the browser, so
//! this serial-wasm build defaults to 40×40 and caps the row/column sliders at 100 —
//! a disclosed scaling of the C counts, not a change to any C constant.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use box3d_rust::body::create_body;
use box3d_rust::distance::ShapeProxy;
use box3d_rust::height_field::{
    create_grid, create_wave, get_height_field_triangle_count, HeightFieldData,
};
use box3d_rust::math_functions::{Pos, Vec3};
use box3d_rust::shape::create_height_field_shape;
use box3d_rust::types::{
    default_body_def, default_query_filter, default_shape_def, default_world_def,
};
use box3d_rust::world::{world_cast_ray_closest, world_cast_shape, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<HfState>> = const { RefCell::new(None) };
}

struct HfState {
    world: World,
    hf: HeightFieldData,
    /// Ground body position (the field is centered on the origin); the wireframe is
    /// baked with this offset so it lines up with the collision shape.
    ground_position: Vec3,
    row_count: i32,
    column_count: i32,
}

fn with_state<R>(f: impl FnOnce(&mut HfState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("height field demo not initialized — call hf_reset first"))
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

/// (Re)build the scene (C `HeightField::CreateScene`). `amplitude == 0` yields a flat
/// grid, otherwise a sinusoidal wave (row/column frequencies 0.1 / 0.03333, matching
/// C). Returns the height-field triangle count.
#[wasm_bindgen]
pub fn hf_reset(row_count: i32, column_count: i32, amplitude: f32, holes: bool) -> i32 {
    let mut world = new_world();

    // C: m_scale = { 2, 2 * m_amplitude, 2 }.
    let scale = Vec3 {
        x: 2.0,
        y: 2.0 * amplitude,
        z: 2.0,
    };
    let hf = if amplitude == 0.0 {
        create_grid(row_count, column_count, scale, holes)
    } else {
        create_wave(row_count, column_count, scale, 0.1, 0.03333, holes)
    };

    // C: bodyDef.position = { -0.5 * scale.x * (columnCount - 1), 0, -0.5 * scale.z * (rowCount - 1) }.
    let ground_position = Vec3 {
        x: -0.5 * hf.scale.x * (column_count - 1) as f32,
        y: 0.0,
        z: -0.5 * hf.scale.z * (row_count - 1) as f32,
    };

    let mut body_def = default_body_def();
    body_def.position = Pos {
        x: ground_position.x as _,
        y: ground_position.y as _,
        z: ground_position.z as _,
    };
    let ground = create_body(&mut world, &body_def);
    let shape_def = default_shape_def();
    create_height_field_shape(&mut world, ground, &shape_def, &hf);

    let count = get_height_field_triangle_count(&hf);
    STATE.with(|cell| {
        *cell.borrow_mut() = Some(HfState {
            world,
            hf,
            ground_position,
            row_count,
            column_count,
        });
    });
    count
}

/// World-space triangle-edge wireframe of the current field (`[x0,y0,z0, x1,y1,z1, ...]`).
#[wasm_bindgen]
pub fn hf_wireframe() -> Vec<f32> {
    with_state(|state| crate::vis::hf_triangle_edges(&state.hf, state.ground_position))
}

/// Field scale (the wireframe/ray sliders use it to size their ranges, matching the C
/// `DrawControls` clamps). Returns `[scale_x, scale_y, scale_z, columns, rows]`.
#[wasm_bindgen]
pub fn hf_info() -> Vec<f32> {
    with_state(|state| {
        vec![
            state.hf.scale.x,
            state.hf.scale.y,
            state.hf.scale.z,
            state.column_count as f32,
            state.row_count as f32,
        ]
    })
}

/// Cast the current field with the ray/shape-cast probe (C `HeightField::Step`).
///
/// `radius == 0` runs `b3World_CastRayClosest`; a non-zero radius runs
/// `b3World_CastShape` with a single-point sphere proxy, keeping the closest hit.
/// Returns `[hit, fraction, px, py, pz, nx, ny, nz]` (fraction is 1 when no hit).
#[wasm_bindgen]
pub fn hf_cast(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32, radius: f32) -> Vec<f32> {
    with_state(|state| {
        let origin = Pos {
            x: ox as _,
            y: oy as _,
            z: oz as _,
        };
        let translation = Vec3 {
            x: tx,
            y: ty,
            z: tz,
        };
        let filter = default_query_filter();

        if radius == 0.0 {
            let result = world_cast_ray_closest(&state.world, origin, translation, &filter);
            let (px, py, pz) = (
                result.point.x as f32,
                result.point.y as f32,
                result.point.z as f32,
            );
            vec![
                if result.hit { 1.0 } else { 0.0 },
                result.fraction,
                px,
                py,
                pz,
                result.normal.x,
                result.normal.y,
                result.normal.z,
            ]
        } else {
            // C ShapeProxy: a single point at m_rayOrigin with the sphere radius.
            let mut proxy = ShapeProxy::default();
            proxy.points[0] = Vec3 {
                x: ox,
                y: oy,
                z: oz,
            };
            proxy.count = 1;
            proxy.radius = radius;

            // Closest-hit accumulator (C `CastContext` + `CastCallback`, which clips
            // the cast by returning the current fraction).
            let mut hit = false;
            let mut best_fraction = 1.0f32;
            let mut point = Pos {
                x: 0.0 as _,
                y: 0.0 as _,
                z: 0.0 as _,
            };
            let mut normal = Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            // The proxy is expressed in origin-relative space, so the cast origin is
            // zero and the proxy carries the offset (matches C `b3Pos_zero`).
            world_cast_shape(
                &state.world,
                Pos {
                    x: 0.0 as _,
                    y: 0.0 as _,
                    z: 0.0 as _,
                },
                &proxy,
                translation,
                &filter,
                |_shape, p, n, fraction, _mat, _tri, _child| {
                    hit = true;
                    point = p;
                    normal = n;
                    best_fraction = fraction;
                    fraction
                },
            );

            vec![
                if hit { 1.0 } else { 0.0 },
                best_fraction,
                point.x as f32,
                point.y as f32,
                point.z as f32,
                normal.x,
                normal.y,
                normal.z,
            ]
        }
    })
}
