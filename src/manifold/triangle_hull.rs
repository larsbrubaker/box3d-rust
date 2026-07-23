//! Hull vs triangle contact manifold from `triangle_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::triangle::get_triangle_feature;
use super::triangle_face::{
    collide_hull_and_triangle_edges, collide_hull_face, collide_triangle_face,
};
use super::types::{
    EdgeQuery, FaceQuery, LocalManifold, SatCache, SeparatingFeature, TriangleFeature,
    FEATURE_PAIR_SINGLE,
};
use crate::constants::{linear_slop, speculative_distance};
use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::hull::{
    find_hull_support_vertex, get_hull_edges, get_hull_planes, get_hull_points, HullData,
};
use crate::math_functions::{
    abs_float, dot, length_squared, lerp, make_plane_from_points, max_float, neg, normalize,
    plane_separation, sub, Plane, Vec3, TRANSFORM_IDENTITY, VEC3_ZERO,
};
pub(crate) struct TriangleData {
    pub(crate) v1: Vec3,
    pub(crate) v2: Vec3,
    pub(crate) v3: Vec3,
    pub(crate) e1: Vec3,
    pub(crate) e2: Vec3,
    pub(crate) e3: Vec3,
    pub(crate) plane: Plane,
    #[allow(dead_code)]
    pub(crate) flags: i32,
}

/// Support vertex of a triangle in a direction. (static b3GetTriangleSupport)
fn get_triangle_support(points: &[Vec3; 3], direction: Vec3) -> i32 {
    let mut index = 0;
    let mut distance = dot(points[0], direction);

    let d = dot(points[1], direction);
    if d > distance {
        distance = d;
        index = 1;
    }

    let d = dot(points[2], direction);
    if d > distance {
        return 2;
    }

    index
}

/// Face query: triangle plane vs hull. (static b3QueryTriangleFace)
fn query_triangle_face(triangle: &TriangleData, hull: &HullData) -> FaceQuery {
    let hull_points = get_hull_points(hull);
    let plane = triangle.plane;
    let vertex_index = find_hull_support_vertex(hull, neg(plane.normal));
    let support = hull_points[vertex_index as usize];
    let separation = plane_separation(plane, support);

    FaceQuery {
        separation,
        face_index: 0,
        vertex_index,
    }
}

/// Face query: hull faces vs triangle. (static b3QueryHullFace)
fn query_hull_face(triangle: &TriangleData, hull: &HullData) -> FaceQuery {
    let hull_planes = get_hull_planes(hull);
    let face_count = hull.face_count;

    let triangle_points = [triangle.v1, triangle.v2, triangle.v3];

    let mut max_face_index = -1;
    let mut max_vertex_index = -1;
    let mut max_face_separation = -f32::MAX;

    for face_index in 0..face_count {
        let plane = hull_planes[face_index as usize];
        let vertex_index = get_triangle_support(&triangle_points, neg(plane.normal));
        let support = triangle_points[vertex_index as usize];
        let separation = plane_separation(plane, support);
        if separation > max_face_separation {
            max_face_index = face_index;
            max_vertex_index = vertex_index;
            max_face_separation = separation;
        }
    }

    FaceQuery {
        separation: max_face_separation,
        face_index: max_face_index,
        vertex_index: max_vertex_index,
    }
}

/// Test all Minkowski edge pairs. (static b3QueryTriangleAndHullEdges)
fn query_triangle_and_hull_edges(triangle: &TriangleData, hull: &HullData) -> EdgeQuery {
    let mut result = EdgeQuery {
        normal: VEC3_ZERO,
        separation: -f32::MAX,
        index_a: NULL_INDEX,
        index_b: NULL_INDEX,
    };

    let triangle_points = [triangle.v1, triangle.v2, triangle.v3];
    let triangle_edges = [triangle.e1, triangle.e2, triangle.e3];

    let tri_normal = triangle.plane.normal;

    let hull_edges = get_hull_edges(hull);
    let hull_points = get_hull_points(hull);
    let hull_planes = get_hull_planes(hull);
    let edge_count = hull.edge_count;
    let squared_tolerance = 0.005 * 0.005;

    let mut i = 0;
    while i < edge_count {
        let edge = &hull_edges[i as usize];
        let twin = &hull_edges[(i + 1) as usize];
        debug_assert!(edge.twin as i32 == i + 1 && twin.twin as i32 == i);

        let hull_point = hull_points[edge.origin as usize];
        let hull_edge = sub(hull_points[twin.origin as usize], hull_point);

        let hull_normal1 = hull_planes[edge.face as usize].normal;
        let hull_normal2 = hull_planes[twin.face as usize].normal;

        for j in 0..3 {
            let tri_edge = triangle_edges[j];

            let cab = dot(hull_normal1, tri_edge);
            let dab = dot(hull_normal2, tri_edge);
            let bcd = dot(tri_normal, hull_edge);
            if cab * dab >= 0.0 || cab * bcd <= 0.0 {
                continue;
            }

            // Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
            if max_float(cab * cab, dab * dab) < squared_tolerance * length_squared(tri_edge) {
                continue;
            }

            // Similar to hull vs hull (b3QueryEdgeDirections)
            // dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
            // Normal points out of hull by construction.
            let t = cab / (cab - dab);
            let mut axis = lerp(hull_normal1, hull_normal2, t);
            debug_assert!(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
            axis = normalize(axis);
            let separation = dot(axis, sub(triangle_points[j], hull_point));

            // if ( separation > result.separation && ( edgeFlags[j] & triangleFlags ) == 0 )
            if separation > result.separation {
                // Note: We don't exit early if we find a separating axis here since we want to
                // find the best one for caching.
                // Flip normal to point from triangle to hull.
                result.normal = neg(axis);
                result.separation = separation;
                result.index_a = j as i32;
                result.index_b = i;
            }
        }

        i += 2;
    }

    result
}

/// Collide a hull and a triangle in the local space of the hull.
/// (b3CollideHullAndTriangle)
pub fn collide_hull_and_triangle(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    v1: Vec3,
    v2: Vec3,
    v3: Vec3,
    triangle_flags: i32,
    cache: &mut SatCache,
    enable_speculative: bool,
) {
    manifold.point_count = 0;
    manifold.feature = TriangleFeature::None;

    if capacity < 4 {
        return;
    }

    let triangle_plane = make_plane_from_points(v1, v2, v3);
    let linear_slop = linear_slop();

    let offset = plane_separation(triangle_plane, hull_a.center);
    if cache.type_ == SeparatingFeature::BacksideAxis as u8 {
        // Use hysteresis to avoid jitter on wavy meshes
        if abs_float(cache.separation - offset) < linear_slop {
            return;
        }

        cache.type_ = SeparatingFeature::InvalidAxis as u8;
    }

    if offset < -linear_slop {
        // Cull back side collision. Cache offset to add hysteresis.
        cache.type_ = SeparatingFeature::BacksideAxis as u8;
        cache.separation = offset;
        return;
    }

    let triangle_points = [v1, v2, v3];
    let triangle_edges = [sub(v2, v1), sub(v3, v2), sub(v1, v3)];

    let triangle = TriangleData {
        v1,
        v2,
        v3,
        e1: triangle_edges[0],
        e2: triangle_edges[1],
        e3: triangle_edges[2],
        plane: triangle_plane,
        flags: triangle_flags,
    };

    let edges = get_hull_edges(hull_a);
    let hull_planes = get_hull_planes(hull_a);
    let hull_points = get_hull_points(hull_a);

    let speculative = if enable_speculative {
        speculative_distance()
    } else {
        0.0
    };
    cache.hit = 1;

    // Attempt to use the cache to speed up collision
    match cache.type_ {
        t if t == SeparatingFeature::FaceAxisA as u8 => {
            debug_assert!(cache.index_a == 0);

            let vertex_index = find_hull_support_vertex(hull_a, neg(triangle_plane.normal));
            let support = hull_points[vertex_index as usize];
            let separation = plane_separation(triangle_plane, support);
            if separation > speculative {
                return;
            }

            let face_query = FaceQuery {
                separation,
                face_index: cache.index_a as i32,
                vertex_index,
            };

            let mut local_cache = *cache;
            let clipped_separation = collide_triangle_face(
                manifold,
                capacity,
                &triangle,
                hull_a,
                face_query,
                &mut local_cache,
                enable_speculative,
            );

            if manifold.point_count > 0
                && abs_float(cache.separation - clipped_separation) < linear_slop
            {
                return;
            }

            manifold.point_count = 0;
            *cache = SatCache::default();
        }
        t if t == SeparatingFeature::FaceAxisB as u8 => {
            debug_assert!((cache.index_b as i32) < hull_a.face_count);

            let plane = hull_planes[cache.index_b as usize];

            let mut vertex_index = 0;
            let mut distance = -dot(v1, plane.normal);
            for i in 1..3 {
                let d = -dot(triangle_points[i], plane.normal);
                if d > distance {
                    distance = d;
                    vertex_index = i;
                }
            }

            let support = triangle_points[vertex_index];
            let separation = plane_separation(plane, support);
            if separation > speculative {
                return;
            }

            // Deep overlap may lead to an invalid cache
            let is_deep = separation < -2.0 * linear_slop;

            if !is_deep {
                let face_query = FaceQuery {
                    separation,
                    face_index: cache.index_b as i32,
                    vertex_index: vertex_index as i32,
                };

                let mut local_cache = *cache;
                let clipped_separation = collide_hull_face(
                    manifold,
                    capacity,
                    &triangle,
                    hull_a,
                    face_query,
                    &mut local_cache,
                    enable_speculative,
                );

                if manifold.point_count > 0
                    && abs_float(cache.separation - clipped_separation) < linear_slop
                {
                    return;
                }
            }

            manifold.point_count = 0;
            *cache = SatCache::default();
        }
        t if t == SeparatingFeature::EdgePairAxis as u8 => {
            debug_assert!((cache.index_a as i32) < 3);
            let index_a = cache.index_a as i32;

            let tri_point = triangle_points[index_a as usize];
            let tri_edge = triangle_edges[index_a as usize];

            debug_assert!((cache.index_b as i32) < hull_a.edge_count - 1);
            let index_b = cache.index_b as i32;

            let edge2 = &edges[index_b as usize];
            let twin2 = &edges[(index_b + 1) as usize];
            debug_assert!(edge2.twin as i32 == index_b + 1 && twin2.twin as i32 == index_b);

            let hull_point = hull_points[edge2.origin as usize];
            let hull_edge = sub(hull_points[twin2.origin as usize], hull_point);
            let hull_normal1 = hull_planes[edge2.face as usize].normal;
            let hull_normal2 = hull_planes[twin2.face as usize].normal;

            // Confirm the edge pair is still a Minkowski face.
            // See "Collision Detection of Convex Polyhedra Based on Duality Transformation"
            // Simplified for triangle versus hull.
            let cab = dot(hull_normal1, tri_edge);
            let dab = dot(hull_normal2, tri_edge);
            let bcd = dot(triangle_plane.normal, hull_edge);

            if cab * dab < 0.0 && cab * bcd > 0.0 {
                let squared_tolerance = 0.005 * 0.005;

                // Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
                if max_float(cab * cab, dab * dab) >= squared_tolerance * length_squared(tri_edge) {
                    // Similar to hull vs hull (b3QueryEdgeDirections)
                    // dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
                    // Normal points out of hull by construction.
                    let t = cab / (cab - dab);
                    let mut axis = lerp(hull_normal1, hull_normal2, t);
                    debug_assert!(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
                    axis = normalize(axis);
                    let separation = dot(axis, sub(tri_point, hull_point));
                    if separation > speculative {
                        // Cache hit, shapes are separated
                        return;
                    }

                    if abs_float(cache.separation - separation) < linear_slop {
                        // Try to rebuild contact from last features
                        // Flip normal to point from triangle to hull
                        let edge_query = EdgeQuery {
                            normal: neg(axis),
                            separation,
                            index_a,
                            index_b,
                        };

                        // Read cache but don't modify it
                        let mut local_cache = *cache;
                        collide_hull_and_triangle_edges(
                            manifold,
                            capacity,
                            tri_point,
                            tri_edge,
                            hull_a,
                            edge_query,
                            &mut local_cache,
                        );

                        if manifold.point_count > 0 {
                            // Cache hit, contact point generated
                            return;
                        }
                    }
                }
            }

            // Invalidate cache and fall through
            *cache = SatCache::default();
        }
        t if t == SeparatingFeature::ManualFaceAxisA as u8 => {
            let face_query_a = query_triangle_face(&triangle, hull_a);
            collide_triangle_face(
                manifold,
                capacity,
                &triangle,
                hull_a,
                face_query_a,
                cache,
                enable_speculative,
            );
            return;
        }
        t if t == SeparatingFeature::ManualFaceAxisB as u8 => {
            let face_query_b = query_hull_face(&triangle, hull_a);
            collide_hull_face(
                manifold,
                capacity,
                &triangle,
                hull_a,
                face_query_b,
                cache,
                enable_speculative,
            );
            return;
        }
        t if t == SeparatingFeature::ManualEdgePairAxis as u8 => {
            let edge_query = query_triangle_and_hull_edges(&triangle, hull_a);
            if edge_query.index_a != NULL_INDEX {
                let triangle_point = triangle_points[edge_query.index_a as usize];
                let triangle_edge = triangle_edges[edge_query.index_a as usize];
                collide_hull_and_triangle_edges(
                    manifold,
                    capacity,
                    triangle_point,
                    triangle_edge,
                    hull_a,
                    edge_query,
                    cache,
                );
            }
            return;
        }
        _ => {
            debug_assert!(cache.type_ == SeparatingFeature::InvalidAxis as u8);
        }
    }

    // Cache miss
    cache.hit = 0;

    let face_query_a = query_triangle_face(&triangle, hull_a);
    if face_query_a.separation > speculative {
        cache.separation = face_query_a.separation;
        cache.type_ = SeparatingFeature::FaceAxisA as u8;
        cache.index_a = 0;
        cache.index_b = u8::MAX;
        return;
    }

    let face_query_b = query_hull_face(&triangle, hull_a);
    if face_query_b.separation > speculative {
        cache.separation = face_query_b.separation;
        cache.type_ = SeparatingFeature::FaceAxisB as u8;
        cache.index_a = u8::MAX;
        cache.index_b = face_query_b.face_index as u8;
        return;
    }

    let edge_query = query_triangle_and_hull_edges(&triangle, hull_a);
    if edge_query.separation > speculative {
        cache.separation = edge_query.separation;
        cache.type_ = SeparatingFeature::EdgePairAxis as u8;
        cache.index_a = edge_query.index_a as u8;
        cache.index_b = edge_query.index_b as u8;
        return;
    }

    // Don't admit a hull face significantly opposed to the triangle face.
    // Need a tolerance to avoid ghost collisions.
    let hull_normal = hull_planes[face_query_b.face_index as usize].normal;
    let pushing_down = dot(hull_normal, triangle_plane.normal) > 0.25;
    let clipped_face_separation =
        if face_query_b.separation >= face_query_a.separation && !pushing_down {
            collide_hull_face(
                manifold,
                capacity,
                &triangle,
                hull_a,
                face_query_b,
                cache,
                enable_speculative,
            )
        } else {
            collide_triangle_face(
                manifold,
                capacity,
                &triangle,
                hull_a,
                face_query_a,
                cache,
                enable_speculative,
            )
        };

    // Does an edge axis exist?
    if edge_query.index_a != NULL_INDEX {
        // When axes are aligned the edge separation can be garbage.
        // If a face axis has positive separation there may be no points.
        let max_face_separation = max_float(face_query_a.separation, face_query_b.separation);

        if (manifold.point_count == 0 && edge_query.separation > max_face_separation)
            || (manifold.point_count == 1
                && edge_query.separation > clipped_face_separation + linear_slop)
        {
            debug_assert!((0..3).contains(&edge_query.index_a));
            let triangle_point = triangle_points[edge_query.index_a as usize];
            let triangle_edge = triangle_edges[edge_query.index_a as usize];
            manifold.point_count = 0;
            collide_hull_and_triangle_edges(
                manifold,
                capacity,
                triangle_point,
                triangle_edge,
                hull_a,
                edge_query,
                cache,
            );
        }
    }

    // Using the speculative distance means that sometimes there are no valid contact points from SAT.
    // Fall back to GJK. This is important to prevent tunneling in rare cases.
    if manifold.point_count == 0 {
        let triangle_b = [v1, v2, v3];
        let input = DistanceInput {
            proxy_a: make_proxy(&triangle_b, 0.0),
            proxy_b: make_proxy(hull_points, 0.0),
            transform: TRANSFORM_IDENTITY,
            use_radii: false,
        };

        let mut simplex_cache = SimplexCache::default();
        let output = shape_distance(&input, &mut simplex_cache, None);

        if output.distance > 0.0 {
            debug_assert!((0 < simplex_cache.count) && (simplex_cache.count <= 3));

            manifold.point_count = 1;
            manifold.feature = get_triangle_feature(&simplex_cache);
            manifold.normal = output.normal;
            manifold.points[0].point = output.point_b;
            manifold.points[0].separation = output.distance;
            manifold.points[0].pair = FEATURE_PAIR_SINGLE;
        }
    }
}
