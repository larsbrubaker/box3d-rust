//! Hull construction demos.

use wasm_bindgen::prelude::*;

use box3d_rust::hull::{get_hull_edges, get_hull_points, make_box_hull};

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
