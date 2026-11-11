//! Mesh / height-field triangle narrow phase with ghost collision handling and clustering.
//!
//! Port of `b3CollideMeshTriangles` from `box3d-cpp-reference/src/mesh_contact.c`. The
//! triangles are collided with the convex shape, each triangle manifold that is not a face
//! contact is validated against the neighboring triangles (admitted, skipped to a neighbor,
//! snapped to the triangle face, or dropped), and the surviving manifolds are clustered by
//! normal and reduced.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::mesh_cull::{reduce_cluster, Cluster};
use super::mesh_neighbors::{
    collide_boundary_edge, test_hull_face_neighbors, test_neighbors, NeighborResult,
    NEIGHBOR_TOLERANCE,
};
use super::mesh_survivors::{collect_survivor_triangles, SurvivorTriangles};
use super::{ContactCache, TriangleCache};
use crate::constants::{
    linear_slop, mesh_rest_offset, speculative_distance, MAX_POINTS_PER_TRIANGLE,
};
use crate::core::NULL_INDEX;
use crate::distance::SimplexCache;
use crate::geometry::{Capsule, Sphere};
use crate::hull::{get_hull_points, HullData};
use crate::manifold::{
    build_triangle_and_capsule_face_contact, build_triangle_and_hull_face_contact,
    collide_triangle_and_capsule, collide_triangle_and_hull, collide_triangle_and_sphere,
    LocalManifold, LocalManifoldPoint, SatCache, SeparatingFeature, TriangleFeature,
    LOCAL_MANIFOLD_CAPACITY, MAX_CLIP_POINTS, MESH_CONTACT_DEEP, MESH_CONTACT_HULL_FACE,
    MESH_CONTACT_SNAPPED,
};
use crate::math_functions::{
    dot, make_matrix_from_quat, make_normal_from_points, make_plane_from_points, max_float,
    min_float, min_int, mul_mv, sub, Transform, Vec3,
};
use crate::shape::{get_neighbor_normal, ShapeGeometry, TriangleShape};
use crate::world::TaskContext;

/// A borrowed convex shape that collides with mesh triangles. (the `b3_sphereShape`,
/// `b3_capsuleShape`, and `b3_hullShape` cases of `const b3Shape*`)
#[derive(Debug, Clone, Copy)]
pub(crate) enum ConvexShape<'a> {
    Sphere(&'a Sphere),
    Capsule(&'a Capsule),
    Hull(&'a HullData),
}

impl<'a> ConvexShape<'a> {
    /// View a sphere, capsule, or hull geometry. `None` for other shape types.
    pub(crate) fn from_geometry(geometry: &'a ShapeGeometry) -> Option<Self> {
        match geometry {
            ShapeGeometry::Sphere(sphere) => Some(ConvexShape::Sphere(sphere)),
            ShapeGeometry::Capsule(capsule) => Some(ConvexShape::Capsule(capsule)),
            ShapeGeometry::Hull(hull) => Some(ConvexShape::Hull(hull)),
            _ => None,
        }
    }
}

/// The reduced clusters of a mesh collision. Each cluster owns a range of `points`.
pub(super) struct MeshClusters {
    pub clusters: Vec<Cluster>,
    pub points: Vec<LocalManifoldPoint>,
}

/// A triangle manifold that kept points. C keeps these in `manifoldBuffer` with their points
/// in `pointBuffer`; the port keeps the metadata and the range of the point buffer.
struct TriangleManifold {
    normal: Vec3,
    triangle_normal: Vec3,
    point_start: usize,
    point_count: i32,
    triangle_index: i32,
    feature: TriangleFeature,
}

fn ensure_sat(cache: &mut ContactCache) -> &mut SatCache {
    if !matches!(cache, ContactCache::Sat(_)) {
        *cache = ContactCache::Sat(SatCache::default());
    }
    match cache {
        ContactCache::Sat(s) => s,
        ContactCache::Simplex(_) => unreachable!(),
    }
}

fn ensure_simplex(cache: &mut ContactCache) -> &mut SimplexCache {
    if !matches!(cache, ContactCache::Simplex(_)) {
        *cache = ContactCache::Simplex(SimplexCache::default());
    }
    match cache {
        ContactCache::Simplex(s) => s,
        ContactCache::Sat(_) => unreachable!(),
    }
}

/// Collide the cached triangles with a convex shape and reduce the manifolds to clusters.
/// `shape_b` is in its own frame and `transform_a_to_b` brings the triangles into it. The SAT
/// and simplex caches update in place. (b3CollideMeshTriangles)
#[allow(clippy::too_many_arguments)]
pub(super) fn collide_mesh_triangles(
    shape_a: &TriangleShape<'_>,
    shape_b: &ConvexShape<'_>,
    transform_a_to_b: Transform,
    triangle_caches: &mut [TriangleCache],
    enable_speculative: bool,
    handle_ghosts: bool,
    is_fast: bool,
    mut context: Option<&mut TaskContext>,
) -> MeshClusters {
    let empty = MeshClusters {
        clusters: Vec::new(),
        points: Vec::new(),
    };

    let triangle_count = triangle_caches.len();

    let relative_matrix = make_matrix_from_quat(transform_a_to_b.q);
    let linear_slop = linear_slop();

    // This should push apart shapes after a time of impact event.
    // In the past I've called this `polygon skin`, but PhysX and Unreal
    // call it `rest offset` which seems appropriate in this case.
    // It leads to a small visual gap but seems to improve the quality of mesh
    // collision, especially for hull versus mesh.
    let rest_offset = mesh_rest_offset();

    let hull_b = match shape_b {
        ConvexShape::Hull(hull) => Some(*hull),
        _ => None,
    };
    let hull_speculative_distance = if enable_speculative {
        speculative_distance()
    } else {
        rest_offset
    };

    // Transform the triangles into the shape frame and cull them against the hull bounds.
    let SurvivorTriangles {
        indices: survivor_indices,
        vertices: survivor_vertices,
    } = collect_survivor_triangles(
        shape_a,
        shape_b,
        triangle_caches,
        transform_a_to_b,
        relative_matrix,
        enable_speculative,
    );

    let survivor_count = survivor_indices.len();
    if survivor_count == 0 {
        return empty;
    }

    // Make room for clip points. Additional capacity for a heavy clip.
    let point_buffer_capacity = MAX_POINTS_PER_TRIANGLE * survivor_count + 2 * MAX_CLIP_POINTS;

    // C bumps the whole buffer from the arena uninitialized and only its capacity matters, as
    // the loop guard and the room left for each manifold. The port grows the buffer only as far
    // as the window the current triangle manifold can write, and reserves one heavy clip window
    // plus as many kept points up front so typical contacts never reallocate.
    let mut point_buffer: Vec<LocalManifoldPoint> =
        Vec::with_capacity(point_buffer_capacity.min(2 * LOCAL_MANIFOLD_CAPACITY));
    let mut total_point_count = 0usize;

    let mut manifold_buffer: Vec<TriangleManifold> = Vec::with_capacity(survivor_count);

    let mut survivor_manifolds = vec![NULL_INDEX; survivor_count];

    let mut survivor_index = 0;
    while survivor_index < survivor_count && total_point_count + 3 < point_buffer_capacity {
        let index = survivor_indices[survivor_index] as usize;
        let triangle_index = triangle_caches[index].triangle_index;
        let vertices = &survivor_vertices[survivor_index];

        // C collides into `manifoldBuffer + manifoldCount` with its points at the end of the
        // point buffer and keeps the manifold by advancing the counts. No single clip writes
        // more than LOCAL_MANIFOLD_CAPACITY points, so a window that size keeps every point C
        // keeps.
        let point_capacity = min_int(
            (point_buffer_capacity - total_point_count) as i32,
            LOCAL_MANIFOLD_CAPACITY as i32,
        );
        let window_end = total_point_count + point_capacity as usize;
        if point_buffer.len() < window_end {
            point_buffer.resize(window_end, LocalManifoldPoint::default());
        }
        let mut manifold = LocalManifold::new(&mut point_buffer[total_point_count..window_end]);

        match shape_b {
            ConvexShape::Capsule(capsule) => {
                let cache = ensure_simplex(&mut triangle_caches[index].cache);
                collide_triangle_and_capsule(
                    &mut manifold,
                    point_capacity,
                    vertices,
                    capsule,
                    cache,
                );
            }

            ConvexShape::Hull(hull) => {
                let cache = ensure_sat(&mut triangle_caches[index].cache);

                // Cached edge contact is dangerous at high speed because the hull can rotate
                // around the edge and tunnel through the triangle.
                if is_fast && cache.type_ == SeparatingFeature::EdgePairAxis as u8 {
                    *cache = SatCache::default();
                }

                collide_triangle_and_hull(
                    &mut manifold,
                    point_capacity,
                    vertices[0],
                    vertices[1],
                    vertices[2],
                    hull,
                    cache,
                    enable_speculative,
                );
                if let Some(context) = context.as_deref_mut() {
                    context.sat_call_count += 1;
                    context.sat_cache_hit_count += cache.hit as i32;
                }
            }

            ConvexShape::Sphere(sphere) => {
                collide_triangle_and_sphere(&mut manifold, point_capacity, vertices, sphere);
            }
        }

        let mut near_feature = manifold.feature;
        let mut result = NeighborResult::Admit;
        let mut owner_edge = NULL_INDEX;
        let mut mesh_flags = if manifold.feature == TriangleFeature::HullFace {
            MESH_CONTACT_HULL_FACE
        } else {
            0
        };
        if handle_ghosts
            && manifold.point_count > 0
            && manifold.feature != TriangleFeature::TriangleFace
        {
            if manifold.feature == TriangleFeature::HullFace {
                (result, near_feature) = test_hull_face_neighbors(
                    shape_a,
                    triangle_index,
                    vertices,
                    relative_matrix,
                    &manifold,
                );
            } else {
                (result, owner_edge) = test_neighbors(
                    shape_a,
                    triangle_index,
                    vertices,
                    relative_matrix,
                    manifold.feature,
                    manifold.normal,
                );
            }

            // Check deep overlap before skipping.
            if result == NeighborResult::Skip {
                let mut deep = false;
                for point in &manifold.points[..manifold.point_count as usize] {
                    if point.separation < -2.0 * linear_slop {
                        deep = true;
                        break;
                    }
                }

                if deep {
                    mesh_flags |= MESH_CONTACT_DEEP;

                    // Handle deep overlap.
                    if let ConvexShape::Sphere(sphere) = shape_b {
                        // No way to snap a deep sphere normal.
                        if owner_edge != NULL_INDEX {
                            if let Some(neighbor_normal) =
                                get_neighbor_normal(shape_a, triangle_index, owner_edge)
                            {
                                let neighbor_normal = mul_mv(relative_matrix, neighbor_normal);
                                let neighbor_separation = dot(
                                    neighbor_normal,
                                    sub(sphere.center, vertices[owner_edge as usize]),
                                );
                                if neighbor_separation < 0.0 {
                                    // Neighbor would cull, so better keep this one.
                                    result = NeighborResult::Admit;
                                }
                            }
                        }
                    } else {
                        // In deep overlap the owning neighbor may apply backside culling.
                        // Experiments show that in this case the neighbor should snap the
                        // normal.
                        result = NeighborResult::Snap;
                    }
                }
            }

            if result == NeighborResult::Snap {
                mesh_flags |= MESH_CONTACT_SNAPPED;
            }

            let mut raw_separation = f32::MAX;
            for point in &manifold.points[..manifold.point_count as usize] {
                raw_separation = min_float(raw_separation, point.separation);
            }

            if result == NeighborResult::Skip
                || result == NeighborResult::Snap
                || result == NeighborResult::Drop
            {
                // Skip leaves the normal to the neighbor. A sphere off the face cannot snap.
                manifold.point_count = 0;
            }

            if result == NeighborResult::Snap {
                match shape_b {
                    ConvexShape::Capsule(capsule) => {
                        // Snap clips the segment to the triangle for a face contact. May clip
                        // all points.
                        let plane = make_plane_from_points(vertices[0], vertices[1], vertices[2]);
                        build_triangle_and_capsule_face_contact(
                            &mut manifold,
                            vertices,
                            plane,
                            capsule,
                        );
                    }
                    ConvexShape::Hull(hull) => {
                        let cache = ensure_sat(&mut triangle_caches[index].cache);
                        build_triangle_and_hull_face_contact(
                            &mut manifold,
                            point_capacity,
                            vertices,
                            hull,
                            cache,
                            enable_speculative,
                        );
                        if raw_separation > 0.0 {
                            collide_boundary_edge(
                                &mut manifold,
                                shape_a,
                                triangle_index,
                                vertices,
                                relative_matrix,
                                hull,
                                hull_speculative_distance,
                            );
                        }
                    }
                    ConvexShape::Sphere(_) => {}
                }
            }
        }

        let manifold_point_count = manifold.point_count;

        if manifold_point_count > 0 {
            debug_assert!(manifold.feature != TriangleFeature::None);

            let mut point_feature = manifold.feature;
            if point_feature == TriangleFeature::HullFace
                && near_feature != TriangleFeature::HullFace
                && near_feature != TriangleFeature::None
            {
                point_feature = near_feature;
            } else if point_feature == TriangleFeature::HullFace
                && near_feature == TriangleFeature::None
                && result == NeighborResult::Admit
            {
                point_feature = TriangleFeature::TriangleFace;
            }

            let count = manifold_point_count as usize;
            for point in &mut manifold.points[..count] {
                point.triangle_feature = point_feature as u8;
                point.mesh_flags = mesh_flags;
            }

            survivor_manifolds[survivor_index] = manifold_buffer.len() as i32;
            manifold_buffer.push(TriangleManifold {
                normal: manifold.normal,
                triangle_normal: make_normal_from_points(vertices[0], vertices[1], vertices[2]),
                point_start: total_point_count,
                point_count: manifold_point_count,
                triangle_index,
                feature: manifold.feature,
            });
            total_point_count += count;
        }

        survivor_index += 1;
    }

    if let (Some(hull_b), true) = (hull_b, handle_ghosts) {
        // Cull triangles fully below the hull face when it rests on a triangle and has a tilted
        // manifold normal. This helps when a box slides across a chamfered channel.
        let tolerance = NEIGHBOR_TOLERANCE;
        let cos_squared_tolerance = 1.0 - tolerance * tolerance;
        let mut resting = None;
        for (i, m) in manifold_buffer.iter().enumerate() {
            let cos_angle = dot(m.normal, m.triangle_normal);
            if m.point_count > 0
                && m.feature == TriangleFeature::HullFace
                && cos_angle > 0.0
                && cos_angle * cos_angle >= cos_squared_tolerance
            {
                resting = Some(i);
                break;
            }
        }

        if let Some(resting) = resting {
            let resting = &manifold_buffer[resting];
            let normal = resting.triangle_normal;
            let hull_points = get_hull_points(hull_b);
            let mut hull_bottom = f32::MAX;
            for &point in hull_points.iter().take(hull_b.vertex_count as usize) {
                hull_bottom = min_float(hull_bottom, dot(normal, point));
            }

            let mut resting_separation = f32::MAX;
            for point in &point_buffer
                [resting.point_start..resting.point_start + resting.point_count as usize]
            {
                resting_separation = min_float(resting_separation, point.separation);
            }

            // Padding
            hull_bottom -= max_float(0.0, resting_separation) + linear_slop;

            for i in 0..survivor_count {
                if survivor_manifolds[i] == NULL_INDEX {
                    continue;
                }

                let m = &mut manifold_buffer[survivor_manifolds[i] as usize];
                let vertices = &survivor_vertices[i];
                let cos_angle = dot(m.normal, normal);
                let height = max_float(
                    dot(normal, vertices[0]),
                    max_float(dot(normal, vertices[1]), dot(normal, vertices[2])),
                );
                if (cos_angle <= 0.0 || cos_angle * cos_angle < cos_squared_tolerance)
                    && height < hull_bottom
                {
                    m.point_count = 0;
                }
            }
        }
    }

    let accepted_manifolds: Vec<usize> = manifold_buffer
        .iter()
        .enumerate()
        .filter(|(_, m)| m.point_count > 0)
        .map(|(i, _)| i)
        .collect();

    debug_assert!(accepted_manifolds.len() <= triangle_count);

    if accepted_manifolds.is_empty() {
        return empty;
    }

    let accepted_manifold_count = accepted_manifolds.len();
    let mut clusters: Vec<Cluster> = Vec::with_capacity(accepted_manifold_count);
    let mut cluster_memberships = vec![NULL_INDEX; accepted_manifold_count];

    // Cluster tolerance is tighter than the warm starting manifold matching tolerance. These
    // serve different purposes.
    let cluster_threshold = 0.996f32;
    let mut cluster_point_count = 0i32;
    for i in 0..accepted_manifold_count {
        cluster_memberships[i] = NULL_INDEX;

        let m = &manifold_buffer[accepted_manifolds[i]];
        cluster_point_count += m.point_count;

        // Cluster based on the triangle normal and contact normal.
        // The first cluster found is accepted because the tolerance is tight.
        // todo consider requiring the triangles to be connect by an edge.
        // todo consider looking for the best cluster instead of the first one within tolerance
        // This bool is here to allow quick testing with and without clustering.
        let allow_clustering = true;
        let manifold_normal = m.normal;
        let triangle_normal = m.triangle_normal;
        let mut cluster_index = NULL_INDEX;
        for (j, cluster) in clusters.iter().enumerate() {
            if !allow_clustering {
                break;
            }

            let cos_manifold_angle = dot(cluster.manifold_normal, manifold_normal);
            let cos_triangle_angle = dot(cluster.triangle_normal, triangle_normal);
            if cos_manifold_angle <= cluster_threshold || cos_triangle_angle <= cluster_threshold {
                continue;
            }

            // Found a cluster
            cluster_index = j as i32;
            break;
        }

        if cluster_index != NULL_INDEX {
            cluster_memberships[i] = cluster_index;
            clusters[cluster_index as usize].point_capacity += m.point_count;
        } else {
            cluster_memberships[i] = clusters.len() as i32;
            clusters.push(Cluster {
                manifold_normal,
                triangle_normal,
                point_start: 0,
                point_capacity: m.point_count,
                point_count: 0,
            });
        }
    }

    if cluster_point_count == 0 {
        return empty;
    }

    // Setup clusters
    let mut cluster_points = vec![LocalManifoldPoint::default(); cluster_point_count as usize];
    let mut point_offset = 0usize;

    for cluster in &mut clusters {
        cluster.point_start = point_offset;
        cluster.point_count = 0;
        point_offset += cluster.point_capacity as usize;
    }

    // Populate clusters
    for i in 0..accepted_manifold_count {
        let cluster_index = cluster_memberships[i];
        if cluster_index == NULL_INDEX {
            continue;
        }

        debug_assert!(0 <= cluster_index && (cluster_index as usize) < clusters.len());

        let am = &manifold_buffer[accepted_manifolds[i]];
        let cm = &mut clusters[cluster_index as usize];
        for ap in &point_buffer[am.point_start..am.point_start + am.point_count as usize] {
            debug_assert!(cm.point_count < cm.point_capacity);
            let cp = &mut cluster_points[cm.point_start + cm.point_count as usize];

            cp.triangle_index = am.triangle_index;
            cp.point = ap.point;
            cp.separation = ap.separation;
            cp.pair = ap.pair;
            cp.triangle_feature = ap.triangle_feature;
            cp.mesh_flags = ap.mesh_flags;
            cm.point_count += 1;
        }
    }

    // Simplify clusters
    for cluster in &mut clusters {
        debug_assert!(cluster.point_count == cluster.point_capacity);
        let start = cluster.point_start;
        let count = cluster.point_count;
        let reduced_count = reduce_cluster(
            &mut cluster_points[start..start + count as usize],
            count,
            cluster.triangle_normal,
        );
        cluster.point_count = reduced_count;
    }

    MeshClusters {
        clusters,
        points: cluster_points,
    }
}
