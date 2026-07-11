//! Height field demos.

use wasm_bindgen::prelude::*;

use box3d_rust::geometry::RayCastInput;
use box3d_rust::height_field::{
    create_wave, get_height_field_triangle, get_height_field_triangle_count, ray_cast_height_field,
};
use box3d_rust::math_functions::Vec3;
use std::cell::RefCell;

thread_local! {
    static HF: RefCell<Option<box3d_rust::height_field::HeightFieldData>> = const { RefCell::new(None) };
}

fn ensure_wave() {
    HF.with(|cell| {
        if cell.borrow().is_none() {
            let hf = create_wave(
                17,
                17,
                Vec3 {
                    x: 0.5,
                    y: 1.2,
                    z: 0.5,
                },
                0.12,
                0.12,
                false,
            );
            *cell.borrow_mut() = Some(hf);
        }
    });
}

/// Rebuild the demo height field as a wave. Returns triangle count.
#[wasm_bindgen]
pub fn hf_build_wave() -> i32 {
    HF.with(|cell| {
        let hf = create_wave(
            17,
            17,
            Vec3 {
                x: 0.5,
                y: 1.2,
                z: 0.5,
            },
            0.12,
            0.12,
            false,
        );
        let count = get_height_field_triangle_count(&hf);
        *cell.borrow_mut() = Some(hf);
        count
    })
}

/// Return wireframe triangle edges: interleaved [x0,y0,z0, x1,y1,z1, ...] for
/// every triangle edge (may duplicate shared edges).
#[wasm_bindgen]
pub fn hf_wireframe() -> Vec<f32> {
    ensure_wave();
    HF.with(|cell| {
        let borrow = cell.borrow();
        let hf = borrow.as_ref().unwrap();
        let count = get_height_field_triangle_count(hf);
        let mut out = Vec::new();
        for i in 0..count {
            let tri = get_height_field_triangle(hf, i);
            let verts = tri.vertices;
            for e in 0..3 {
                let a = verts[e];
                let b = verts[(e + 1) % 3];
                out.push(a.x);
                out.push(a.y);
                out.push(a.z);
                out.push(b.x);
                out.push(b.y);
                out.push(b.z);
            }
        }
        out
    })
}

/// Ray cast the height field. Returns [hit, fraction, px,py,pz, nx,ny,nz, tri].
#[wasm_bindgen]
pub fn hf_ray_cast(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    ensure_wave();
    HF.with(|cell| {
        let borrow = cell.borrow();
        let hf = borrow.as_ref().unwrap();
        let input = RayCastInput {
            origin: Vec3 {
                x: ox,
                y: oy,
                z: oz,
            },
            translation: Vec3 {
                x: tx,
                y: ty,
                z: tz,
            },
            max_fraction: 1.0,
        };
        let o = ray_cast_height_field(hf, &input);
        vec![
            if o.hit { 1.0 } else { 0.0 },
            o.fraction,
            o.point.x,
            o.point.y,
            o.point.z,
            o.normal.x,
            o.normal.y,
            o.normal.z,
            o.triangle_index as f32,
        ]
    })
}
