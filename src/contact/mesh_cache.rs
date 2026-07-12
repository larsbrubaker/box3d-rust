//! Mesh / height-field triangle cache refresh and BVH queries.
//!
//! Port of `b3RefreshCache` / query helpers from
//! `box3d-cpp-reference/src/mesh_contact.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{ContactCache, MeshContact, TriangleCache};
use crate::constants::{max_aabb_margin, speculative_distance, MAX_MESH_CONTACT_TRIANGLES};
use crate::geometry::ShapeType;
use crate::height_field::query_height_field;
use crate::math_functions::{
    aabb_contains, aabb_transform, add, invert_transform, sub, to_relative_transform, Aabb, Vec3,
    WorldTransform, POS_ZERO,
};
use crate::mesh::{query_mesh, Mesh};
use crate::shape::{Shape, ShapeGeometry};
use std::sync::Once;

/// Query mesh triangles overlapping `bounds`. (b3QueryMeshTriangles)
fn query_mesh_triangles(indices: &mut [i32], capacity: i32, mesh: &Mesh<'_>, bounds: Aabb) -> i32 {
    let mut count = 0i32;
    query_mesh(mesh, bounds, |_a, _b, _c, triangle_index| {
        if count == capacity {
            return false;
        }
        indices[count as usize] = triangle_index;
        count += 1;
        count < capacity
    });
    count
}

/// Query height-field triangles overlapping `bounds`. (b3QueryHeightFieldTriangles)
fn query_height_field_triangles(
    indices: &mut [i32],
    capacity: i32,
    height_field: &crate::height_field::HeightFieldData,
    bounds: Aabb,
) -> i32 {
    let mut count = 0i32;
    query_height_field(height_field, bounds, |_a, _b, _c, triangle_index| {
        if count == capacity {
            return false;
        }
        indices[count as usize] = triangle_index;
        count += 1;
        count < capacity
    });
    count
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

    let mut triangle_indices = [0i32; MAX_MESH_CONTACT_TRIANGLES];
    let triangle_capacity = MAX_MESH_CONTACT_TRIANGLES as i32;

    // Bounds are in world space. Convert to the local mesh frame.
    let mesh_transform = to_relative_transform(xf_a, POS_ZERO);
    let local_bounds = aabb_transform(invert_transform(mesh_transform), mesh_contact.query_bounds);

    let triangle_count = match &shape_a.geometry {
        ShapeGeometry::Mesh { data, scale } => {
            let mesh = Mesh {
                data,
                scale: *scale,
            };
            query_mesh_triangles(
                &mut triangle_indices,
                triangle_capacity,
                &mesh,
                local_bounds,
            )
        }
        ShapeGeometry::HeightField(hf) => {
            query_height_field_triangles(&mut triangle_indices, triangle_capacity, hf, local_bounds)
        }
        _ => unreachable!(),
    };

    if triangle_count == triangle_capacity {
        static ONCE: Once = Once::new();
        ONCE.call_once(|| {
            eprintln!(
                "WARNING: complex mesh detected, triangle buffer capacity of {triangle_capacity} reached"
            );
        });
    }

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
    let mut contact_caches = [ContactCache::default(); MAX_MESH_CONTACT_TRIANGLES];
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
