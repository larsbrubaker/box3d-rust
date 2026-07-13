//! Mesh Viewer (C `MeshViewer`, :1021) and Creation Benchmark (C
//! `MeshCreationBenchmark`, :1304).
//!
//! The Viewer loads one of the four voxel meshes and exposes a per-level BVH AABB
//! inspector (`mesh_viewer_nodes`) plus the build stats the C sample prints. The
//! Creation Benchmark parses the four meshes once and rebuilds them on demand so
//! the page can time `b3CreateMesh` (C's `b3GetTicks` / `b3MinFloat` reduction).
//!
//! Both mirror `samples/mesh_loader.cpp` `LoadTempMesh`: z-up axis swap, 0.01 scale,
//! and a per-triangle material index cycling 0..2.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{new_world, MeshScene, MeshState};
use crate::obj_loader::{build_mesh_from_temp, parse_obj, TempMesh};
use crate::vis::mesh_triangle_edges;
use box3d_rust::body::create_body;
use box3d_rust::math_functions::{aabb_area, VEC3_ONE};
use box3d_rust::mesh::{get_height, get_mesh_nodes, MeshData};
use box3d_rust::shape::create_mesh_shape;
use box3d_rust::types::{default_body_def, default_shape_def};
use std::collections::VecDeque;

/// C `MeshViewer::m_degenerateCapacity`.
const DEGENERATE_CAPACITY: usize = 64;

/// Sum the internal (non-leaf) BVH node areas (C `ComputeInternalSurfaceArea`).
fn internal_surface_area(mesh: &MeshData) -> f32 {
    let nodes = get_mesh_nodes(mesh);
    let mut area = 0.0f32;
    for node in nodes.iter().skip(1) {
        if node.axis() == 3 {
            continue; // leaf
        }
        area += aabb_area(node.aabb());
    }
    area
}

/// Pack Viewer degenerate-triangle draw data from the pre-weld temp mesh
/// (C `MeshViewer::Render` uses `m_tempMesh` verts/indices + `m_degenerateTriangles`).
///
/// Stride 16: `[triIdx, i1, i2, i3, cx,cy,cz, v1x,v1y,v1z, v2x,v2y,v2z, v3x,v3y,v3z]`.
fn pack_degenerate_draw(temp: &TempMesh, degenerate_indices: &[i32], count: i32) -> Vec<f32> {
    let n = count.max(0) as usize;
    let mut out = Vec::with_capacity(n * 16);
    for &tri in degenerate_indices.iter().take(n) {
        let base = (3 * tri) as usize;
        if base + 2 >= temp.indices.len() {
            continue;
        }
        let i1 = temp.indices[base];
        let i2 = temp.indices[base + 1];
        let i3 = temp.indices[base + 2];
        if i1 < 0
            || i2 < 0
            || i3 < 0
            || (i1 as usize) >= temp.vertices.len()
            || (i2 as usize) >= temp.vertices.len()
            || (i3 as usize) >= temp.vertices.len()
        {
            continue;
        }
        let v1 = temp.vertices[i1 as usize];
        let v2 = temp.vertices[i2 as usize];
        let v3 = temp.vertices[i3 as usize];
        let cx = (1.0 / 3.0) * (v1.x + v2.x + v3.x);
        let cy = (1.0 / 3.0) * (v1.y + v2.y + v3.y);
        let cz = (1.0 / 3.0) * (v1.z + v2.z + v3.z);
        out.extend_from_slice(&[
            tri as f32, i1 as f32, i2 as f32, i3 as f32, cx, cy, cz, v1.x, v1.y, v1.z, v2.x, v2.y,
            v2.z, v3.x, v3.y, v3.z,
        ]);
    }
    out
}

/// MeshViewer::LoadMesh (:1053) for the selected voxel-mesh OBJ text.
pub(crate) fn build_viewer(
    obj_text: &str,
    median_split: bool,
    concave_edges: bool,
    weld_vertices: bool,
    weld_tolerance_mm: f32,
) -> MeshState {
    let mut state = MeshState::base(new_world(), MeshScene::Viewer);

    let temp = parse_obj(obj_text, 0.01, true);
    // C: weldTolerance = 0.001f * m_weldToleranceMillimeters.
    let weld_tolerance = 0.001 * weld_tolerance_mm;
    let mut degenerate_indices = [0i32; DEGENERATE_CAPACITY];
    let mesh = build_mesh_from_temp(
        &temp,
        median_split,
        concave_edges,
        weld_vertices,
        weld_tolerance,
        Some(&mut degenerate_indices),
    )
    .expect("viewer mesh");

    let ground_def = default_body_def();
    let body = create_body(&mut state.world, &ground_def);
    let shape_def = default_shape_def();
    create_mesh_shape(&mut state.world, body, &shape_def, &mesh, VEC3_ONE);
    state.ground_edges = mesh_triangle_edges(&mesh, VEC3_ONE);

    let height = get_height(&mesh);
    let node_area = internal_surface_area(&mesh);
    // Pack draw data for every reported degenerate (C loops `m_mesh->degenerateCount`,
    // capped by the capacity written into `m_degenerateTriangles`).
    let drawn = (mesh.degenerate_count as usize).min(DEGENERATE_CAPACITY);
    state.viewer_degenerates =
        pack_degenerate_draw(&temp, &degenerate_indices[..drawn], mesh.degenerate_count);

    // [triangle_count, vertex_count, degenerate_count, height, node_area].
    // Build time is measured on the page side (around the reset call).
    state.stats = vec![
        mesh.triangle_count as f32,
        mesh.vertex_count as f32,
        mesh.degenerate_count as f32,
        height as f32,
        node_area,
    ];
    state.viewer_mesh = Some(mesh);
    state
}

/// Viewer BVH height (max draw level). 0 when no mesh is loaded.
pub(crate) fn viewer_height(state: &mut MeshState) -> i32 {
    state.viewer_mesh.as_ref().map(get_height).unwrap_or(0)
}

/// BVH nodes at `level` (C `MeshViewer::DrawNodes` BFS). Each node emits
/// `[lx,ly,lz, ux,uy,uz, axis]` (axis 0/1/2 internal split, 3 leaf).
pub(crate) fn viewer_nodes(state: &MeshState, level: i32) -> Vec<f32> {
    let Some(mesh) = &state.viewer_mesh else {
        return Vec::new();
    };
    if level < 0 {
        return Vec::new();
    }
    let nodes = get_mesh_nodes(mesh);
    if nodes.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::new();
    let mut queue: VecDeque<(usize, i32)> = VecDeque::new();
    queue.push_back((0, 0));
    while let Some((idx, lvl)) = queue.pop_front() {
        let node = &nodes[idx];
        if lvl == level {
            let aabb = node.aabb();
            out.extend_from_slice(&[
                aabb.lower_bound.x,
                aabb.lower_bound.y,
                aabb.lower_bound.z,
                aabb.upper_bound.x,
                aabb.upper_bound.y,
                aabb.upper_bound.z,
                node.axis() as f32,
            ]);
        }
        if lvl > level {
            break;
        }
        let is_leaf = node.axis() == 3;
        if !is_leaf {
            queue.push_back((idx + 1, lvl + 1));
            queue.push_back((idx + node.child_offset() as usize, lvl + 1));
        }
    }
    out
}

/// Packed degenerate-triangle draw data (see [`MeshState::viewer_degenerates`]).
pub(crate) fn viewer_degenerates(state: &MeshState) -> Vec<f32> {
    state.viewer_degenerates.clone()
}

/// Creation Benchmark reset: parse the four voxel meshes once (C ctor).
pub(crate) fn build_benchmark(objs: &[&str]) -> MeshState {
    let mut state = MeshState::base(new_world(), MeshScene::Benchmark);
    for obj in objs {
        state.bench_meshes.push(parse_obj(obj, 0.01, true));
    }
    state
}

/// Build every benchmark mesh once (C `MeshCreationBenchmark::Step` inner loop:
/// useMedianSplit true, identifyEdges false, weldVertices true, weldTolerance
/// 0.0015) and return the total triangle count. Timed by the page.
pub(crate) fn benchmark_build(state: &mut MeshState) -> i32 {
    let mut triangle_count = 0i32;
    for temp in &state.bench_meshes {
        if let Some(mesh) = build_mesh_from_temp(temp, true, false, true, 0.0015, None) {
            triangle_count += mesh.triangle_count;
        }
    }
    triangle_count
}
