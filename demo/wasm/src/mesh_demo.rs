//! Mesh demos.

use wasm_bindgen::prelude::*;

use box3d_rust::geometry::RayCastInput;
use box3d_rust::math_functions::TRANSFORM_IDENTITY;
use box3d_rust::math_functions::{Aabb, Vec3, VEC3_ONE};
use box3d_rust::mesh::{
    compute_mesh_aabb, create_box_mesh, create_grid_mesh, get_mesh_triangles, get_mesh_vertices,
    ray_cast_mesh, Mesh,
};
use std::cell::RefCell;

thread_local! {
    static MESH: RefCell<Option<box3d_rust::mesh::MeshData>> = const { RefCell::new(None) };
}

fn ensure_box_mesh() {
    MESH.with(|cell| {
        if cell.borrow().is_none() {
            let m = create_box_mesh(
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 1.5,
                    y: 1.0,
                    z: 1.2,
                },
                true,
            )
            .expect("box mesh");
            *cell.borrow_mut() = Some(m);
        }
    });
}

/// Build a box mesh. Returns [vertex_count, triangle_count].
#[wasm_bindgen]
pub fn mesh_build_box() -> Vec<f32> {
    MESH.with(|cell| {
        let m = create_box_mesh(
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 1.5,
                y: 1.0,
                z: 1.2,
            },
            true,
        )
        .expect("box mesh");
        let out = vec![m.vertex_count as f32, m.triangle_count as f32];
        *cell.borrow_mut() = Some(m);
        out
    })
}

/// Build a grid mesh. Returns [vertex_count, triangle_count].
#[wasm_bindgen]
pub fn mesh_build_grid() -> Vec<f32> {
    MESH.with(|cell| {
        let m = create_grid_mesh(9, 9, 0.4, 1, true).expect("grid mesh");
        let out = vec![m.vertex_count as f32, m.triangle_count as f32];
        *cell.borrow_mut() = Some(m);
        out
    })
}

/// Wireframe edges for the current mesh.
#[wasm_bindgen]
pub fn mesh_wireframe() -> Vec<f32> {
    ensure_box_mesh();
    MESH.with(|cell| {
        let borrow = cell.borrow();
        let data = borrow.as_ref().unwrap();
        let verts = get_mesh_vertices(data);
        let tris = get_mesh_triangles(data);
        let mut out = Vec::new();
        for t in tris {
            let vs = [
                verts[t.index1 as usize],
                verts[t.index2 as usize],
                verts[t.index3 as usize],
            ];
            for e in 0..3 {
                let a = vs[e];
                let b = vs[(e + 1) % 3];
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

/// AABB of the current mesh: [lx,ly,lz, ux,uy,uz].
#[wasm_bindgen]
pub fn mesh_aabb() -> Vec<f32> {
    ensure_box_mesh();
    MESH.with(|cell| {
        let borrow = cell.borrow();
        let data = borrow.as_ref().unwrap();
        let aabb: Aabb = compute_mesh_aabb(data, TRANSFORM_IDENTITY, VEC3_ONE);
        vec![
            aabb.lower_bound.x,
            aabb.lower_bound.y,
            aabb.lower_bound.z,
            aabb.upper_bound.x,
            aabb.upper_bound.y,
            aabb.upper_bound.z,
        ]
    })
}

/// Ray cast the current mesh. Returns [hit, fraction, px,py,pz, nx,ny,nz, tri].
#[wasm_bindgen]
pub fn mesh_ray_cast(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    ensure_box_mesh();
    MESH.with(|cell| {
        let borrow = cell.borrow();
        let data = borrow.as_ref().unwrap();
        let mesh = Mesh::with_unit_scale(data);
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
        let o = ray_cast_mesh(&mesh, &input);
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
