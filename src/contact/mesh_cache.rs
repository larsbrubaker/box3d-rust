//! Mesh / height-field triangle cache refresh and BVH queries.
//!
//! Port of `b3RefreshCache` and the triangle query callback from
//! `box3d-cpp-reference/src/mesh_contact.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{ContactCache, MeshContact, TriangleCache};
use crate::constants::{max_aabb_margin, speculative_distance};
use crate::geometry::ShapeType;
use crate::height_field::query_height_field;
use crate::math_functions::{
    aabb_contains, aabb_transform, add, invert_transform, sub, to_relative_transform, Aabb, Vec3,
    WorldTransform, POS_ZERO,
};
use crate::mesh::query_mesh;
use crate::shape::{Shape, ShapeGeometry, TriangleShape};

/// (B3_INITIAL_TRIANGLE_CAPACITY)
pub(super) const INITIAL_TRIANGLE_CAPACITY: usize = 512;

/// Collect the indices of the triangles overlapping local `bounds`. C appends them with
/// `b3CollectTriangleIndicesCallback`, doubling an arena array from
/// [`INITIAL_TRIANGLE_CAPACITY`]; the port appends to a vector.
/// (b3TriangleQueryContext / b3CollectTriangleIndicesCallback)
pub(super) fn collect_triangle_indices(shape: &TriangleShape<'_>, bounds: Aabb) -> Vec<i32> {
    let mut indices = Vec::with_capacity(INITIAL_TRIANGLE_CAPACITY);
    let mut collect = |_a: Vec3, _b: Vec3, _c: Vec3, triangle_index: i32| {
        indices.push(triangle_index);
        true
    };
    match shape {
        TriangleShape::Mesh(mesh) => query_mesh(mesh, bounds, &mut collect),
        TriangleShape::HeightField(height_field) => {
            query_height_field(height_field, bounds, &mut collect)
        }
    }
    indices
}

/// Refresh the per-contact triangle cache when the mover leaves query bounds.
/// (b3RefreshCache)
pub(crate) fn refresh_cache(
    mesh_contact: &mut MeshContact,
    shape_a: &Shape,
    xf_a: WorldTransform,
    bounds: Aabb,
) {
    debug_assert!(
        shape_a.shape_type() == ShapeType::Mesh || shape_a.shape_type() == ShapeType::Height
    );

    // If the dynamic body didn't move out of the cached query bounds we are done!
    if aabb_contains(mesh_contact.query_bounds, bounds) {
        if let ShapeGeometry::Mesh { data, .. } = &shape_a.geometry {
            for cache in &mesh_contact.triangle_cache {
                debug_assert!(
                    0 <= cache.triangle_index && cache.triangle_index < data.triangle_count
                );
            }
        }
        return;
    }

    // Enlarge the query bounds to absorb small movement.
    let radius = max_aabb_margin() + speculative_distance();
    let extension = Vec3 {
        x: radius,
        y: radius,
        z: radius,
    };
    mesh_contact.query_bounds = Aabb {
        lower_bound: sub(bounds.lower_bound, extension),
        upper_bound: add(bounds.upper_bound, extension),
    };

    // Query triangles

    // Bounds are in world space. Convert to the local mesh frame. The broadphase bounds are
    // float, so the demoted mesh transform is the matching float world frame (exact in float
    // mode).
    let mesh_transform = to_relative_transform(xf_a, POS_ZERO);
    let local_bounds = aabb_transform(invert_transform(mesh_transform), mesh_contact.query_bounds);
    let shape = TriangleShape::from_geometry(&shape_a.geometry)
        .expect("mesh contact requires a mesh or height-field shape");
    let mut triangle_indices = collect_triangle_indices(&shape, local_bounds);
    let triangle_count = triangle_indices.len() as i32;

    // Triangle indices must be sorted to match caches (C validates this; BVH
    // traversal is not always monotonic, so sort explicitly).
    triangle_indices[..triangle_count as usize].sort_unstable();

    debug_assert!({
        let mut ok = true;
        for i in 0..(triangle_count - 1).max(0) {
            if triangle_indices[i as usize] >= triangle_indices[(i + 1) as usize] {
                ok = false;
                break;
            }
        }
        ok || triangle_count <= 1
    });

    // Create new contact cache and match with old one.
    let mut contact_caches = vec![ContactCache::default(); triangle_count as usize];
    let mut index2 = 0i32;
    let old_count = mesh_contact.triangle_cache.len() as i32;
    for index1 in 0..triangle_count {
        contact_caches[index1 as usize] = ContactCache::default();

        while index2 < old_count
            && mesh_contact.triangle_cache[index2 as usize].triangle_index
                < triangle_indices[index1 as usize]
        {
            index2 += 1;
        }

        if index2 < old_count
            && mesh_contact.triangle_cache[index2 as usize].triangle_index
                == triangle_indices[index1 as usize]
        {
            contact_caches[index1 as usize] = mesh_contact.triangle_cache[index2 as usize].cache;
        }
    }

    mesh_contact.triangle_cache.clear();
    mesh_contact.triangle_cache.reserve(triangle_count as usize);
    for i in 0..triangle_count {
        let triangle_index = triangle_indices[i as usize];
        if let ShapeGeometry::Mesh { data, .. } = &shape_a.geometry {
            debug_assert!(0 <= triangle_index && triangle_index < data.triangle_count);
        }
        mesh_contact.triangle_cache.push(TriangleCache {
            triangle_index,
            cache: contact_caches[i as usize],
        });
    }
}
