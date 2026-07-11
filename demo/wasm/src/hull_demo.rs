//! Hull construction demos.

use wasm_bindgen::prelude::*;

use box3d_rust::hull::{create_hull, get_hull_edges, get_hull_points, make_box_hull};
use box3d_rust::math_functions::{compute_cos_sin, Vec3, PI};

/// Create a box hull and return wireframe segments as interleaved
/// [x0,y0,z0, x1,y1,z1, ...] for each unique edge (12 edges × 2 verts).
#[wasm_bindgen]
pub fn box_hull_edges(hx: f32, hy: f32, hz: f32) -> Vec<f32> {
    let hull = make_box_hull(hx, hy, hz);
    let points = get_hull_points(&hull.base);
    let edges = get_hull_edges(&hull.base);

    let mut out = Vec::new();
    // Half-edges come in twin pairs; emit each undirected edge once (even index).
    let mut i = 0;
    while i < edges.len() {
        let e = &edges[i];
        let twin = &edges[e.twin as usize];
        if (e.origin as i32) < (twin.origin as i32) {
            let a = points[e.origin as usize];
            let b = points[twin.origin as usize];
            out.push(a.x);
            out.push(a.y);
            out.push(a.z);
            out.push(b.x);
            out.push(b.y);
            out.push(b.z);
        }
        i += 1;
    }
    out
}

/// Create a hull from a ring of points (plus poles) via `create_hull`.
/// Returns [vertex_count, face_count, edge_count, then edge segments xyzxyz...].
#[wasm_bindgen]
pub fn create_hull_demo(sides: u32, radius: f32, height: f32) -> Vec<f32> {
    let n = sides.max(3).min(16) as i32;
    let mut points = Vec::with_capacity((n + 2) as usize);
    for i in 0..n {
        let angle = 2.0 * PI * i as f32 / n as f32;
        let cs = compute_cos_sin(angle);
        points.push(Vec3 {
            x: radius * cs.cosine,
            y: 0.0,
            z: radius * cs.sine,
        });
    }
    points.push(Vec3 {
        x: 0.0,
        y: height,
        z: 0.0,
    });
    points.push(Vec3 {
        x: 0.0,
        y: -height * 0.4,
        z: 0.0,
    });

    let Some(hull) = create_hull(&points, 0) else {
        return vec![0.0, 0.0, 0.0];
    };

    let pts = get_hull_points(&hull);
    let edges = get_hull_edges(&hull);
    let mut out = vec![
        hull.vertex_count as f32,
        hull.face_count as f32,
        hull.edge_count as f32,
    ];
    for e in edges {
        let twin = edges[e.twin as usize];
        if (e.origin as i32) < (twin.origin as i32) {
            let a = pts[e.origin as usize];
            let b = pts[twin.origin as usize];
            out.push(a.x);
            out.push(a.y);
            out.push(a.z);
            out.push(b.x);
            out.push(b.y);
            out.push(b.z);
        }
    }
    out
}
