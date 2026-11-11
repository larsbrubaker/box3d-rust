//! Authored tests for mesh pure geometry, plus the port of
//! box3d-cpp-reference/test/test_mesh.c (stride, clockwise winding, welding,
//! and the built in creators) at the end of the file.
//!
//! Covers create/AABB/ray/overlap for box and grid meshes, plus BVH vs brute ray.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, CastOutput};
use crate::geometry::RayCastInput;
use crate::math_functions::{
    cross, distance_squared, intersect_ray_triangle, is_sane_aabb, mul, normalize, sub, Vec3,
    TRANSFORM_IDENTITY, VEC3_ONE, VEC3_ZERO,
};
use crate::mesh::{
    compute_mesh_aabb, create_box_mesh, create_grid_mesh, create_hollow_box_mesh, create_mesh,
    create_platform_mesh, create_torus_mesh, create_wave_mesh, destroy_mesh, get_height,
    get_mesh_flags, get_mesh_triangle, get_mesh_triangles, get_mesh_vertices, is_valid_mesh,
    overlap_mesh, ray_cast_mesh, Mesh, MeshData, MeshDef, ALL_CONCAVE_EDGES, MESH_VERSION,
};

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

#[test]
fn mesh_box_create() {
    let mesh = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), true).expect("box mesh");
    assert_eq!(mesh.version, MESH_VERSION);
    assert!(is_valid_mesh(Some(&mesh)));
    assert_eq!(mesh.vertex_count, 8);
    assert_eq!(mesh.triangle_count, 12);
    assert!(mesh.node_count >= 1);
    assert!(get_height(&mesh) >= 0);
    assert!(mesh.hash != 0);

    ensure_small(mesh.bounds.lower_bound.x + 1.0, 1e-5);
    ensure_small(mesh.bounds.lower_bound.y + 1.0, 1e-5);
    ensure_small(mesh.bounds.lower_bound.z + 1.0, 1e-5);
    ensure_small(mesh.bounds.upper_bound.x - 1.0, 1e-5);
    ensure_small(mesh.bounds.upper_bound.y - 1.0, 1e-5);
    ensure_small(mesh.bounds.upper_bound.z - 1.0, 1e-5);

    destroy_mesh(mesh);
}

#[test]
fn mesh_box_winding() {
    let mesh = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("box mesh");
    let view = Mesh::with_unit_scale(&mesh);

    for i in 0..mesh.triangle_count {
        let t = get_mesh_triangle(&view, i);
        let e1 = sub(t.vertices[1], t.vertices[0]);
        let e2 = sub(t.vertices[2], t.vertices[0]);
        let n = cross(e1, e2);
        // Outward-ish: normal should point away from origin for a unit box.
        let center = v(
            (t.vertices[0].x + t.vertices[1].x + t.vertices[2].x) / 3.0,
            (t.vertices[0].y + t.vertices[1].y + t.vertices[2].y) / 3.0,
            (t.vertices[0].z + t.vertices[1].z + t.vertices[2].z) / 3.0,
        );
        let outward = crate::math_functions::dot(n, center);
        assert!(outward > 0.0, "triangle {i} winding inverted");
    }

    destroy_mesh(mesh);
}

#[test]
fn mesh_box_aabb() {
    let mesh = create_box_mesh(v(2.0, 3.0, 4.0), v(1.0, 0.5, 0.25), false).expect("box");
    let aabb = compute_mesh_aabb(&mesh, TRANSFORM_IDENTITY, VEC3_ONE);
    ensure_small(aabb.lower_bound.x - 1.0, 1e-5);
    ensure_small(aabb.lower_bound.y - 2.5, 1e-5);
    ensure_small(aabb.lower_bound.z - 3.75, 1e-5);
    ensure_small(aabb.upper_bound.x - 3.0, 1e-5);
    ensure_small(aabb.upper_bound.y - 3.5, 1e-5);
    ensure_small(aabb.upper_bound.z - 4.25, 1e-5);

    let scaled = compute_mesh_aabb(&mesh, TRANSFORM_IDENTITY, v(2.0, 1.0, 1.0));
    ensure_small(scaled.lower_bound.x - 2.0, 1e-5);
    ensure_small(scaled.upper_bound.x - 6.0, 1e-5);

    destroy_mesh(mesh);
}

#[test]
fn mesh_box_ray_hit_miss() {
    let mesh = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("box");
    let view = Mesh::with_unit_scale(&mesh);

    let hit_input = RayCastInput {
        origin: v(0.0, 0.0, 3.0),
        translation: v(0.0, 0.0, -4.0),
        max_fraction: 1.0,
    };
    let hit = ray_cast_mesh(&view, &hit_input);
    assert!(hit.hit);
    assert!(hit.triangle_index != NULL_INDEX);
    ensure_small(hit.fraction - 0.5, 1e-4);
    ensure_small(hit.point.z - 1.0, 1e-4);

    let miss_input = RayCastInput {
        origin: v(0.0, 0.0, 3.0),
        translation: v(0.0, 0.0, 1.0),
        max_fraction: 1.0,
    };
    let miss = ray_cast_mesh(&view, &miss_input);
    assert!(!miss.hit);

    destroy_mesh(mesh);
}

#[test]
fn mesh_box_overlap() {
    let mesh = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("box");
    let view = Mesh::with_unit_scale(&mesh);

    // Mesh is a thin surface — probe near a face, not the hollow interior.
    let near_face = make_proxy(&[v(0.0, 0.0, 0.95)], 0.1);
    assert!(overlap_mesh(&view, TRANSFORM_IDENTITY, &near_face));

    let outside = make_proxy(&[v(5.0, 0.0, 0.0)], 0.1);
    assert!(!overlap_mesh(&view, TRANSFORM_IDENTITY, &outside));

    destroy_mesh(mesh);
}

#[test]
fn mesh_grid_create() {
    let mesh = create_grid_mesh(4, 3, 1.0, 0, false).expect("grid");
    assert_eq!(mesh.version, MESH_VERSION);
    assert_eq!(mesh.vertex_count, 5 * 4);
    assert_eq!(mesh.triangle_count, 2 * 4 * 3);
    assert!(mesh.node_count > 1);
    assert!(is_valid_mesh(Some(&mesh)));

    // Grid is centered: x in [-2, 2], z in [-1.5, 1.5]
    ensure_small(mesh.bounds.lower_bound.x + 2.0, 1e-5);
    ensure_small(mesh.bounds.upper_bound.x - 2.0, 1e-5);
    ensure_small(mesh.bounds.lower_bound.z + 1.5, 1e-5);
    ensure_small(mesh.bounds.upper_bound.z - 1.5, 1e-5);
    ensure_small(mesh.bounds.lower_bound.y, 1e-5);
    ensure_small(mesh.bounds.upper_bound.y, 1e-5);

    destroy_mesh(mesh);
}

#[test]
fn mesh_grid_ray_and_bvh_vs_brute() {
    let mesh = create_grid_mesh(8, 8, 1.0, 0, true).expect("grid");
    let view = Mesh::with_unit_scale(&mesh);

    let rays = [
        (v(0.0, 2.0, 0.0), v(0.0, -4.0, 0.0)),
        (v(-3.0, 1.0, -2.0), v(6.0, -2.0, 4.0)),
        (v(1.5, 5.0, -1.5), v(0.0, -10.0, 0.0)),
        (v(0.0, -1.0, 0.0), v(0.0, -1.0, 0.0)), // miss (below plane, going down)
        (v(10.0, 1.0, 10.0), v(1.0, 0.0, 0.0)), // miss
    ];

    for (origin, translation) in rays {
        let input = RayCastInput {
            origin,
            translation,
            max_fraction: 1.0,
        };
        let bvh = ray_cast_mesh(&view, &input);
        let brute = brute_force_ray_cast(&view, &input);

        assert_eq!(bvh.hit, brute.hit, "hit mismatch at {origin:?}");
        if bvh.hit {
            ensure_small(bvh.fraction - brute.fraction, 1e-5);
            // Coplanar grid edges can yield equal fractions on adjacent triangles;
            // require matching hit point rather than a unique triangle index.
            ensure_small(bvh.point.x - brute.point.x, 1e-4);
            ensure_small(bvh.point.y - brute.point.y, 1e-4);
            ensure_small(bvh.point.z - brute.point.z, 1e-4);
        }
    }

    destroy_mesh(mesh);
}

fn brute_force_ray_cast(mesh: &Mesh<'_>, input: &RayCastInput) -> CastOutput {
    let mut best = CastOutput {
        fraction: input.max_fraction,
        triangle_index: NULL_INDEX,
        ..Default::default()
    };
    let scale = mesh.scale;
    let clockwise = scale.x * scale.y * scale.z < 0.0;

    for triangle_index in 0..mesh.data.triangle_count {
        let tri = &mesh.data.triangles[triangle_index as usize];
        let vertex1 = mul(scale, mesh.data.vertices[tri.index1 as usize]);
        let (vertex2, vertex3) = if clockwise {
            (
                mul(scale, mesh.data.vertices[tri.index3 as usize]),
                mul(scale, mesh.data.vertices[tri.index2 as usize]),
            )
        } else {
            (
                mul(scale, mesh.data.vertices[tri.index2 as usize]),
                mul(scale, mesh.data.vertices[tri.index3 as usize]),
            )
        };

        let alpha =
            intersect_ray_triangle(input.origin, input.translation, vertex1, vertex2, vertex3);
        if alpha < best.fraction {
            let edge1 = sub(vertex2, vertex1);
            let edge2 = sub(vertex3, vertex1);
            best.normal = normalize(cross(edge1, edge2));
            best.point = crate::math_functions::add(
                input.origin,
                crate::math_functions::mul_sv(alpha, input.translation),
            );
            best.fraction = alpha;
            best.triangle_index = triangle_index;
            best.material_index = mesh.data.material_indices[triangle_index as usize] as i32;
            best.hit = true;
        }
    }
    best
}

#[test]
fn mesh_create_from_def() {
    let def = MeshDef {
        vertices: vec![v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)],
        indices: vec![0, 1, 2],
        identify_edges: false,
        use_median_split: false,
        ..Default::default()
    };
    let mesh = create_mesh(&def, None).expect("triangle mesh");
    assert_eq!(mesh.triangle_count, 1);
    assert_eq!(mesh.vertex_count, 3);
    assert_eq!(mesh.node_count, 1);
    assert!(mesh.nodes[0].is_leaf());
    destroy_mesh(mesh);
}

#[test]
fn mesh_negative_scale_winding() {
    let mesh = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("box");
    let view = Mesh::new(&mesh, v(1.0, -1.0, 1.0));
    let t = get_mesh_triangle(&view, 0);
    // Negative scale product swaps indices 2/3 relative to stored order.
    let stored = &mesh.triangles[0];
    assert_eq!(t.i1, stored.index1);
    assert_eq!(t.i2, stored.index3);
    assert_eq!(t.i3, stored.index2);
    destroy_mesh(mesh);
}

// Port of test_mesh.c ----------------------------------------------------------------------

// Two quads meeting at a concave crease along the shared edge 1-4. The crease exercises
// edge identification, which reads the baked winding rather than the input winding.
const VALLEY_VERTEX_COUNT: usize = 6;
const VALLEY_TRIANGLE_COUNT: usize = 4;

pub(crate) fn make_valley() -> (Vec<Vec3>, Vec<i32>) {
    let vertices = vec![
        v(-1.0, 1.0, -1.0),
        v(0.0, 0.0, -1.0),
        v(1.0, 1.0, -1.0),
        v(-1.0, 1.0, 1.0),
        v(0.0, 0.0, 1.0),
        v(1.0, 1.0, 1.0),
    ];

    #[rustfmt::skip]
    let indices = vec![
        0, 3, 1, // left quad
        3, 4, 1,
        1, 4, 2, // right quad
        4, 5, 2,
    ];

    (vertices, indices)
}

// A vertex laid out the way a renderer would hand it over. The position sits off the
// front of the struct so a non-zero base offset gets exercised too.
//
// C: struct FatVertex { float weight; b3Vec3 position; float uv[2]; }. The bytes are
// written field by field so the layout (24 bytes, position at byte 4) does not depend
// on Rust struct layout.
const FAT_VERTEX_SIZE: usize = 24;
const FAT_POSITION_OFFSET: usize = 4;

fn make_fat_vertices(vertices: &[Vec3]) -> Vec<u8> {
    let mut fat = Vec::with_capacity(FAT_VERTEX_SIZE * vertices.len());
    for (i, vertex) in vertices.iter().enumerate() {
        // Poison the padding so a stride mistake bakes obvious garbage instead of near misses
        let weight = 1000.0f32 + i as f32;
        fat.extend_from_slice(&weight.to_ne_bytes());
        fat.extend_from_slice(&vertex.x.to_ne_bytes());
        fat.extend_from_slice(&vertex.y.to_ne_bytes());
        fat.extend_from_slice(&vertex.z.to_ne_bytes());
        fat.extend_from_slice(&(-1000.0f32).to_ne_bytes());
        fat.extend_from_slice(&(-2000.0f32).to_ne_bytes());
    }
    fat
}

// The Rust stand-in for pointing at the first fat position: the bytes starting there.
fn fat_position_bytes(fat: &[u8]) -> Vec<u8> {
    fat[FAT_POSITION_OFFSET..].to_vec()
}

fn reverse_winding(indices: &mut [i32], triangle_count: usize) {
    for i in 0..triangle_count {
        indices.swap(3 * i + 1, 3 * i + 2);
    }
}

// The hash covers every byte of the mesh block, so this compares the tree, the vertices,
// the baked triangles and the edge flags in one shot.
fn meshes_match(mesh1: &MeshData, mesh2: &MeshData) -> bool {
    mesh1.byte_count == mesh2.byte_count && mesh1.hash == mesh2.hash
}

fn vertices_match(mesh: &MeshData, expected: &[Vec3], count: usize) -> bool {
    if mesh.vertex_count as usize != count {
        return false;
    }

    let vertices = get_mesh_vertices(mesh);
    for i in 0..count {
        if distance_squared(vertices[i], expected[i]) > f32::EPSILON {
            return false;
        }
    }

    true
}

// The valley faces up, so every baked triangle should wind counter clockwise about +Y
fn faces_up(mesh: &MeshData) -> bool {
    let vertices = get_mesh_vertices(mesh);
    let triangles = get_mesh_triangles(mesh);

    for i in 0..mesh.triangle_count as usize {
        let v1 = vertices[triangles[i].index1 as usize];
        let v2 = vertices[triangles[i].index2 as usize];
        let v3 = vertices[triangles[i].index3 as usize];

        let normal = cross(sub(v2, v1), sub(v3, v1));
        if normal.y <= 0.0 {
            return false;
        }
    }

    true
}

fn has_concave_edge(mesh: &MeshData) -> bool {
    let flags = get_mesh_flags(mesh);

    for i in 0..mesh.triangle_count as usize {
        if (flags[i] as i32 & ALL_CONCAVE_EDGES) != 0 {
            return true;
        }
    }

    false
}

pub(crate) fn make_valley_def(vertices: &[Vec3], indices: &[i32]) -> MeshDef {
    MeshDef {
        vertices: vertices.to_vec(),
        stride: core::mem::size_of::<Vec3>(),
        indices: indices.to_vec(),
        identify_edges: true,
        ..Default::default()
    }
}

// A def reading positions out of fat vertices, like C's MakeValleyDef( &fat[0].position, ... )
// with stride = sizeof( FatVertex ).
fn make_fat_valley_def(fat: &[u8], indices: &[i32]) -> MeshDef {
    MeshDef {
        vertex_bytes: fat_position_bytes(fat),
        stride: FAT_VERTEX_SIZE,
        indices: indices.to_vec(),
        identify_edges: true,
        ..Default::default()
    }
}

// Dense input is the reference the other cases are measured against
#[test]
fn mesh_dense_stride() {
    let (vertices, indices) = make_valley();

    let def = make_valley_def(&vertices, &indices);
    let mesh = create_mesh(&def, None).expect("mesh");

    assert_eq!(mesh.vertex_count as usize, VALLEY_VERTEX_COUNT);
    assert_eq!(mesh.triangle_count as usize, VALLEY_TRIANGLE_COUNT);
    assert_eq!(mesh.degenerate_count, 0);
    assert!(vertices_match(&mesh, &vertices, VALLEY_VERTEX_COUNT));
    assert!(faces_up(&mesh));
    assert!(has_concave_edge(&mesh));

    destroy_mesh(mesh);
}

// Interleaved input must bake to exactly the same mesh as dense input
#[test]
fn mesh_fat_stride() {
    let (vertices, indices) = make_valley();

    let dense_def = make_valley_def(&vertices, &indices);
    let dense_mesh = create_mesh(&dense_def, None).expect("dense mesh");

    let fat = make_fat_vertices(&vertices);

    let fat_def = make_fat_valley_def(&fat, &indices);
    let fat_mesh = create_mesh(&fat_def, None).expect("fat mesh");

    assert!(vertices_match(&fat_mesh, &vertices, VALLEY_VERTEX_COUNT));
    assert!(faces_up(&fat_mesh));
    assert!(meshes_match(&dense_mesh, &fat_mesh));

    destroy_mesh(fat_mesh);
    destroy_mesh(dense_mesh);
}

// Welding reads the source vertices through the stride as well
#[test]
fn mesh_stride_weld() {
    let (vertices, indices) = make_valley();

    // Split the crease so each quad owns a copy of the shared edge
    let mut split_vertices = vertices.clone();
    split_vertices.push(vertices[1]);
    split_vertices.push(vertices[4]);
    assert_eq!(split_vertices.len(), 8);

    // Point the right quad at the copies
    let mut split_indices = indices.clone();
    split_indices[6] = 6;
    split_indices[7] = 7;
    split_indices[9] = 7;

    let fat = make_fat_vertices(&split_vertices);

    let mut def = make_fat_valley_def(&fat, &split_indices);
    def.weld_vertices = true;
    def.weld_tolerance = 0.01;

    let mesh = create_mesh(&def, None).expect("mesh");

    assert_eq!(mesh.vertex_count as usize, VALLEY_VERTEX_COUNT);
    assert_eq!(mesh.triangle_count as usize, VALLEY_TRIANGLE_COUNT);
    assert!(vertices_match(&mesh, &vertices, VALLEY_VERTEX_COUNT));
    assert!(faces_up(&mesh));
    assert!(has_concave_edge(&mesh));

    destroy_mesh(mesh);
}

// Clockwise input plus the flag must bake to the same mesh as counter clockwise input
#[test]
fn mesh_clock_wise() {
    let (vertices, indices) = make_valley();

    let ccw_def = make_valley_def(&vertices, &indices);
    let ccw_mesh = create_mesh(&ccw_def, None).expect("ccw mesh");

    let mut reversed = indices.clone();
    reverse_winding(&mut reversed, VALLEY_TRIANGLE_COUNT);

    let mut cw_def = make_valley_def(&vertices, &reversed);
    cw_def.clock_wise_winding = true;
    let cw_mesh = create_mesh(&cw_def, None).expect("cw mesh");

    assert!(faces_up(&cw_mesh));
    assert!(has_concave_edge(&cw_mesh));
    assert!(meshes_match(&ccw_mesh, &cw_mesh));

    destroy_mesh(cw_mesh);
    destroy_mesh(ccw_mesh);
}

// Without the flag the same clockwise input must bake inside out
#[test]
fn mesh_clock_wise_ignored() {
    let (vertices, mut indices) = make_valley();
    reverse_winding(&mut indices, VALLEY_TRIANGLE_COUNT);

    let def = make_valley_def(&vertices, &indices);
    let mesh = create_mesh(&def, None).expect("mesh");

    assert!(!faces_up(&mesh));

    destroy_mesh(mesh);
}

// Winding, stride and welding have to compose
#[test]
fn mesh_clock_wise_stride_weld() {
    let (vertices, indices) = make_valley();

    let mut ccw_def = make_valley_def(&vertices, &indices);
    ccw_def.weld_vertices = true;
    ccw_def.weld_tolerance = 0.01;
    let ccw_mesh = create_mesh(&ccw_def, None).expect("ccw mesh");

    let mut reversed = indices.clone();
    reverse_winding(&mut reversed, VALLEY_TRIANGLE_COUNT);

    let fat = make_fat_vertices(&vertices);

    let mut cw_def = make_fat_valley_def(&fat, &reversed);
    cw_def.clock_wise_winding = true;
    cw_def.weld_vertices = true;
    cw_def.weld_tolerance = 0.01;
    let cw_mesh = create_mesh(&cw_def, None).expect("cw mesh");

    assert!(faces_up(&cw_mesh));
    assert!(meshes_match(&ccw_mesh, &cw_mesh));

    destroy_mesh(cw_mesh);
    destroy_mesh(ccw_mesh);
}

// The built in creators fill their own def, so a missed stride shows up here
#[test]
fn mesh_creators() {
    let center = v(1.0, 2.0, 3.0);
    let extent = v(0.5, 1.0, 1.5);

    let meshes = [
        create_grid_mesh(4, 4, 1.0, 2, true),
        create_wave_mesh(4, 4, 1.0, 0.5, 1.0, 1.0),
        create_torus_mesh(8, 6, 2.0, 0.5),
        create_box_mesh(center, extent, true),
        create_hollow_box_mesh(center, extent),
        create_platform_mesh(center, 2.0, 1.0, 2.0),
    ];

    for mesh in meshes {
        let mesh = mesh.expect("creator mesh");
        assert!(mesh.vertex_count >= 3);
        assert!(mesh.triangle_count >= 1);
        assert!(is_sane_aabb(mesh.bounds));

        destroy_mesh(mesh);
    }
}

// Stride validation (C rejects bad strides with NULL). The Rust port applies the stride only
// to `vertex_bytes`, so contiguous `vertices` with any stride other than 0 or 12 are rejected.
#[test]
fn mesh_stride_rejection() {
    let (vertices, indices) = make_valley();
    let fat = make_fat_vertices(&vertices);

    // Bad strides: too small, too large, not 4 aligned
    for stride in [8usize, 4100, 26] {
        let mut def = make_fat_valley_def(&fat, &indices);
        def.stride = stride;
        assert!(create_mesh(&def, None).is_none(), "stride {stride}");
    }

    // Non-12 stride with contiguous vertices and empty vertex_bytes
    let mut def = make_valley_def(&vertices, &indices);
    def.stride = FAT_VERTEX_SIZE;
    assert!(create_mesh(&def, None).is_none());

    // Stride 0 and 12 are both accepted for contiguous vertices
    for stride in [0usize, core::mem::size_of::<Vec3>()] {
        let mut def = make_valley_def(&vertices, &indices);
        def.stride = stride;
        assert!(create_mesh(&def, None).is_some(), "stride {stride}");
    }
}
