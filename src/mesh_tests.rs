//! Authored tests for mesh pure geometry (no upstream test_mesh.c).
//!
//! Covers create/AABB/ray/overlap for box and grid meshes, plus BVH vs brute ray.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, CastOutput};
use crate::geometry::RayCastInput;
use crate::math_functions::{
    cross, intersect_ray_triangle, mul, normalize, sub, Vec3, TRANSFORM_IDENTITY, VEC3_ONE,
    VEC3_ZERO,
};
use crate::mesh::{
    compute_mesh_aabb, create_box_mesh, create_grid_mesh, create_mesh, destroy_mesh, get_height,
    get_mesh_triangle, is_valid_mesh, overlap_mesh, ray_cast_mesh, Mesh, MeshDef, MESH_VERSION,
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
        (v(10.0, 1.0, 10.0), v(1.0, 0.0, 0.0)),  // miss
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
