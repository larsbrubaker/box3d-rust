//! Mesh / height-field triangle transform and hull bounds culling.
//!
//! Port of the survivor stage of `b3ComputeMeshManifolds` from
//! `box3d-cpp-reference/src/mesh_contact.c`: the cached triangles are transformed into the
//! shape frame four at a time, and for a hull, triangles whose bounds miss the hull bounds
//! are culled before the narrow phase.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{ContactCache, TriangleCache};
use crate::constants::speculative_distance;
use crate::core::ctz32;
use crate::height_field::get_height_field_triangle;
use crate::manifold::SatCache;
use crate::math_functions::{min_int, mul, Aabb, Matrix3, Transform, Vec3, VEC3_ONE, VEC3_ZERO};
use crate::shape::{Shape, ShapeGeometry};
use crate::simd_scalar::{
    add_w, greater_than_w, less_than_w, max_w, min_w, move_mask_w, mul_w, or_w, set_w, splat_w,
    store_w, sub_w, zero_w,
};

/// The triangles that survive culling, in cache order: the index into the triangle cache and
/// the triangle vertices in the shape frame.
pub(super) struct SurvivorTriangles {
    pub indices: Vec<i32>,
    pub vertices: Vec<[Vec3; 3]>,
}

/// Transform the cached triangles into the frame of `shape_b` and cull them against the hull
/// bounds. Culled triangles get their SAT cache reset. (survivor stage of
/// b3ComputeMeshManifolds)
pub(super) fn collect_survivor_triangles(
    shape_a: &Shape,
    shape_b: &Shape,
    triangle_caches: &mut [TriangleCache],
    transform_a_to_b: Transform,
    relative_matrix: Matrix3,
    enable_speculative: bool,
) -> SurvivorTriangles {
    let triangle_count = triangle_caches.len();
    let geom_b = &shape_b.geometry;

    let hull_b = match geom_b {
        ShapeGeometry::Hull(hull) => Some(hull),
        _ => None,
    };
    let hull_speculative_distance = if enable_speculative {
        speculative_distance()
    } else {
        0.0
    };

    // C bumps these from the arena.
    let mut survivor_indices: Vec<i32> = Vec::with_capacity(triangle_count);
    let mut survivor_vertices: Vec<[Vec3; 3]> = Vec::with_capacity(triangle_count);

    let (mesh_data, mesh_scale, mesh_flipped) = match &shape_a.geometry {
        ShapeGeometry::Mesh { data, scale } => {
            (Some(data), *scale, scale.x * scale.y * scale.z < 0.0)
        }
        _ => (None, VEC3_ONE, false),
    };

    let cxx = splat_w(relative_matrix.cx.x);
    let cxy = splat_w(relative_matrix.cx.y);
    let cxz = splat_w(relative_matrix.cx.z);
    let cyx = splat_w(relative_matrix.cy.x);
    let cyy = splat_w(relative_matrix.cy.y);
    let cyz = splat_w(relative_matrix.cy.z);
    let czx = splat_w(relative_matrix.cz.x);
    let czy = splat_w(relative_matrix.cz.y);
    let czz = splat_w(relative_matrix.cz.z);
    let px = splat_w(transform_a_to_b.p.x);
    let py = splat_w(transform_a_to_b.p.y);
    let pz = splat_w(transform_a_to_b.p.z);

    let margin_w = splat_w(hull_speculative_distance);
    let hull_bounds = match hull_b {
        Some(hull) => hull.aabb,
        None => Aabb::default(),
    };
    let hull_lower_x = splat_w(hull_bounds.lower_bound.x);
    let hull_lower_y = splat_w(hull_bounds.lower_bound.y);
    let hull_lower_z = splat_w(hull_bounds.lower_bound.z);
    let hull_upper_x = splat_w(hull_bounds.upper_bound.x);
    let hull_upper_y = splat_w(hull_bounds.upper_bound.y);
    let hull_upper_z = splat_w(hull_bounds.upper_bound.z);

    // Transform the triangles into the shape frame four at a time. For a hull, triangles whose
    // bounds (plus the speculative margin) miss the hull bounds are culled before the narrow
    // phase and their SAT cache is reset.
    for base_index in (0..triangle_count).step_by(4) {
        let lane_count = min_int(4, (triangle_count - base_index) as i32) as usize;

        let mut lanes = [[VEC3_ZERO; 4]; 3];
        for lane in 0..4 {
            let index = base_index + min_int(lane as i32, lane_count as i32 - 1) as usize;
            let triangle_index = triangle_caches[index].triangle_index;

            if let Some(data) = mesh_data {
                let mesh_triangle = data.triangles[triangle_index as usize];
                let mesh_vertices = &data.vertices;
                let index2 = if mesh_flipped {
                    mesh_triangle.index3
                } else {
                    mesh_triangle.index2
                };
                let index3 = if mesh_flipped {
                    mesh_triangle.index2
                } else {
                    mesh_triangle.index3
                };
                lanes[0][lane] = mul(mesh_scale, mesh_vertices[mesh_triangle.index1 as usize]);
                lanes[1][lane] = mul(mesh_scale, mesh_vertices[index2 as usize]);
                lanes[2][lane] = mul(mesh_scale, mesh_vertices[index3 as usize]);
            } else {
                let ShapeGeometry::HeightField(hf) = &shape_a.geometry else {
                    unreachable!()
                };
                let triangle = get_height_field_triangle(hf, triangle_index);
                lanes[0][lane] = triangle.vertices[0];
                lanes[1][lane] = triangle.vertices[1];
                lanes[2][lane] = triangle.vertices[2];
            }
        }

        // Transform triangle into the shape frame
        let mut vx = [zero_w(); 3];
        let mut vy = [zero_w(); 3];
        let mut vz = [zero_w(); 3];
        for k in 0..3 {
            let l = &lanes[k];
            let ax = set_w(l[0].x, l[1].x, l[2].x, l[3].x);
            let ay = set_w(l[0].y, l[1].y, l[2].y, l[3].y);
            let az = set_w(l[0].z, l[1].z, l[2].z, l[3].z);

            vx[k] = add_w(
                add_w(add_w(mul_w(cxx, ax), mul_w(cyx, ay)), mul_w(czx, az)),
                px,
            );
            vy[k] = add_w(
                add_w(add_w(mul_w(cxy, ax), mul_w(cyy, ay)), mul_w(czy, az)),
                py,
            );
            vz[k] = add_w(
                add_w(add_w(mul_w(cxz, ax), mul_w(cyz, ay)), mul_w(czz, az)),
                pz,
            );
        }

        let mut keep_mask: u32 = (1u32 << lane_count) - 1;
        if hull_b.is_some() {
            let lower_x = sub_w(min_w(vx[0], min_w(vx[1], vx[2])), margin_w);
            let lower_y = sub_w(min_w(vy[0], min_w(vy[1], vy[2])), margin_w);
            let lower_z = sub_w(min_w(vz[0], min_w(vz[1], vz[2])), margin_w);
            let upper_x = add_w(max_w(vx[0], max_w(vx[1], vx[2])), margin_w);
            let upper_y = add_w(max_w(vy[0], max_w(vy[1], vy[2])), margin_w);
            let upper_z = add_w(max_w(vz[0], max_w(vz[1], vz[2])), margin_w);

            let mut separated = or_w(
                less_than_w(upper_x, hull_lower_x),
                greater_than_w(lower_x, hull_upper_x),
            );
            separated = or_w(
                separated,
                or_w(
                    less_than_w(upper_y, hull_lower_y),
                    greater_than_w(lower_y, hull_upper_y),
                ),
            );
            separated = or_w(
                separated,
                or_w(
                    less_than_w(upper_z, hull_lower_z),
                    greater_than_w(lower_z, hull_upper_z),
                ),
            );

            let mut cull_mask = keep_mask & move_mask_w(separated) as u32;
            keep_mask &= !cull_mask;

            while cull_mask != 0 {
                let lane = ctz32(cull_mask) as usize;
                cull_mask &= cull_mask - 1;
                triangle_caches[base_index + lane].cache = ContactCache::Sat(SatCache::default());
            }
        }

        if keep_mask == 0 {
            continue;
        }

        let mut x = [[0.0f32; 4]; 3];
        let mut y = [[0.0f32; 4]; 3];
        let mut z = [[0.0f32; 4]; 3];
        for k in 0..3 {
            store_w(&mut x[k], vx[k]);
            store_w(&mut y[k], vy[k]);
            store_w(&mut z[k], vz[k]);
        }

        while keep_mask != 0 {
            let lane = ctz32(keep_mask) as usize;
            keep_mask &= keep_mask - 1;

            survivor_vertices.push([
                Vec3 {
                    x: x[0][lane],
                    y: y[0][lane],
                    z: z[0][lane],
                },
                Vec3 {
                    x: x[1][lane],
                    y: y[1][lane],
                    z: z[1][lane],
                },
                Vec3 {
                    x: x[2][lane],
                    y: y[2][lane],
                    z: z[2][lane],
                },
            ]);
            survivor_indices.push((base_index + lane) as i32);
        }
    }

    SurvivorTriangles {
        indices: survivor_indices,
        vertices: survivor_vertices,
    }
}
