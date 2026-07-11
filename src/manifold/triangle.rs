//! Sphere / capsule vs triangle contact manifolds from `triangle_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::clip::edge_edge_separation;
use super::types::{
    make_feature_pair, ClipVertex, EdgeQuery, FaceQuery, FeatureOwner, LocalManifold,
    TriangleFeature, FEATURE_PAIR_SINGLE,
};
use crate::constants::speculative_distance;
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::geometry::{Capsule, Sphere};
use crate::math_functions::{
    abs_float, add, closest_point_on_triangle, cross, distance_squared, dot, lerp,
    make_plane_from_normal_and_point, make_plane_from_points, min_float, mul_sub, mul_sv, neg,
    normalize, plane_separation, sub, Plane, Vec3, TRANSFORM_IDENTITY,
};

/// Indexed by the 3-bit vertex mask from a GJK simplex on the triangle.
/// (static s_triangleFeatures)
const TRIANGLE_FEATURES: [TriangleFeature; 8] = [
    TriangleFeature::None,         // 000 (unreachable)
    TriangleFeature::Vertex1,      // 001
    TriangleFeature::Vertex2,      // 010
    TriangleFeature::Edge1,        // 011 v1,v2
    TriangleFeature::Vertex3,      // 100
    TriangleFeature::Edge3,        // 101 v1,v3
    TriangleFeature::Edge2,        // 110 v2,v3
    TriangleFeature::TriangleFace, // 111
];

/// Map a GJK simplex cache on the triangle to a triangle feature.
/// (static b3GetTriangleFeature)
pub(crate) fn get_triangle_feature(cache: &SimplexCache) -> TriangleFeature {
    let count = cache.count as i32;
    debug_assert!((0 < count) && (count < 4));

    // Bit i set means triangle vertex i participates in the simplex.
    let mut mask = 0i32;
    for i in 0..count {
        debug_assert!((cache.index_a[i as usize] as i32) < 3);
        mask |= 1 << cache.index_a[i as usize];
    }

    TRIANGLE_FEATURES[mask as usize]
}

/// Clip a 2-vertex capsule segment against the three side planes of a triangle.
/// (static b3ClipSegmentToTriangleFace)
fn clip_segment_to_triangle_face(
    segment: &mut [ClipVertex; 2],
    points: &[Vec3; 3],
    plane: Plane,
) -> bool {
    let mut vertex1 = points[2];
    for i in 0..3 {
        let vertex2 = points[i];
        let tangent = normalize(sub(vertex2, vertex1));
        let binormal = cross(tangent, plane.normal);
        let clip_plane = make_plane_from_normal_and_point(binormal, vertex1);

        let mut vertex_count = 0;
        let p1 = segment[0];
        let p2 = segment[1];

        let distance1 = plane_separation(clip_plane, p1.position);
        let distance2 = plane_separation(clip_plane, p2.position);

        if distance1 <= 0.0 {
            segment[vertex_count] = p1;
            vertex_count += 1;
        }
        if distance2 <= 0.0 {
            segment[vertex_count] = p2;
            vertex_count += 1;
        }

        if distance1 * distance2 < 0.0 {
            let t = distance1 / (distance1 - distance2);
            segment[vertex_count].position = lerp(p1.position, p2.position, t);
            segment[vertex_count].pair = if distance1 > 0.0 { p1.pair } else { p2.pair };
            vertex_count += 1;
        }

        if vertex_count != 2 {
            return false;
        }

        vertex1 = vertex2;
    }

    true
}

/// Face query of a triangle plane against a capsule. (static b3QueryTriangleFaceAndCapsule)
fn query_triangle_face_and_capsule(plane: Plane, capsule: &Capsule) -> FaceQuery {
    let separation1 = plane_separation(plane, capsule.center1);
    let separation2 = plane_separation(plane, capsule.center2);

    if separation1 < separation2 {
        FaceQuery {
            separation: separation1,
            face_index: 0,
            vertex_index: 0,
        }
    } else {
        FaceQuery {
            separation: separation2,
            face_index: 0,
            vertex_index: 1,
        }
    }
}

/// Edge-pair query of triangle edges vs capsule axis. (static b3QueryTriangleAndCapsuleEdges)
fn query_triangle_and_capsule_edges(vertices: &[Vec3; 3], capsule: &Capsule) -> EdgeQuery {
    let p1 = capsule.center1;
    let p2 = capsule.center2;
    let capsule_edge = sub(p2, p1);
    let capsule_center = lerp(p1, p2, 0.5);
    let triangle_center = mul_sv(1.0 / 3.0, add(vertices[0], add(vertices[1], vertices[2])));

    let mut max_separation = -f32::MAX;
    let mut max_index1 = u8::MAX as i32;
    let mut max_index2 = u8::MAX as i32;

    let mut edge_index = 2;
    let mut v1 = vertices[2];
    for index in 0..3 {
        let v2 = vertices[index];
        let triangle_edge = sub(v2, v1);

        let separation = edge_edge_separation(
            p1,
            capsule_edge,
            capsule_center,
            v1,
            triangle_edge,
            triangle_center,
        );
        if separation > max_separation {
            // Note: We don't exit early if we find a separating axis here since we want to
            // find the best one for caching and account for the convex radius later.
            max_separation = separation;
            max_index1 = edge_index;
            max_index2 = 0;
        }

        v1 = v2;
        edge_index = index as i32;
    }

    EdgeQuery {
        separation: max_separation,
        index_a: max_index1,
        index_b: max_index2,
    }
}

/// Build face contact between triangle and capsule. (static b3BuildTriangleAndCapsuleFaceContact)
fn build_triangle_and_capsule_face_contact(
    manifold: &mut LocalManifold,
    triangle: &[Vec3; 3],
    plane: Plane,
    capsule: &Capsule,
) {
    debug_assert!(manifold.point_count == 0);

    let mut segment = [
        ClipVertex {
            position: capsule.center1,
            separation: 0.0,
            pair: make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0),
        },
        ClipVertex {
            position: capsule.center2,
            separation: 0.0,
            pair: make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1),
        },
    ];

    if !clip_segment_to_triangle_face(&mut segment, triangle, plane) {
        return;
    }

    let radius = capsule.radius;
    let distance1 = plane_separation(plane, segment[0].position);
    let distance2 = plane_separation(plane, segment[1].position);

    let speculative = speculative_distance();
    if distance1 > speculative + radius && distance2 > speculative + radius {
        return;
    }

    // Average points. Half-way between capsule bottom and triangle plane.
    let point1 = mul_sub(
        segment[0].position,
        0.5 * (distance1 + capsule.radius),
        plane.normal,
    );
    let point2 = mul_sub(
        segment[1].position,
        0.5 * (distance2 + capsule.radius),
        plane.normal,
    );

    manifold.normal = plane.normal;
    manifold.feature = TriangleFeature::TriangleFace;
    manifold.point_count = 2;

    let pt = &mut manifold.points[0];
    pt.point = point1;
    pt.separation = distance1 - capsule.radius;
    pt.pair = segment[0].pair;

    let pt = &mut manifold.points[1];
    pt.point = point2;
    pt.separation = distance2 - capsule.radius;
    pt.pair = segment[1].pair;
}

/// Build edge contact between triangle and capsule. (static b3BuildTriangleAndCapsuleEdgeContact)
fn build_triangle_and_capsule_edge_contact(
    manifold: &mut LocalManifold,
    triangle: &[Vec3; 3],
    capsule: &Capsule,
    query: EdgeQuery,
) {
    debug_assert!((0..3).contains(&query.index_a));

    let p1 = capsule.center1;
    let p2 = capsule.center2;
    let capsule_edge = sub(p2, p1);

    let triangle_center = mul_sv(1.0 / 3.0, add(triangle[0], add(triangle[1], triangle[2])));
    let v1 = triangle[query.index_a as usize];
    let v2 = triangle[((query.index_a + 1) % 3) as usize];
    let triangle_edge = sub(v2, v1);

    let mut normal = cross(capsule_edge, triangle_edge);
    normal = normalize(normal);

    // Normal should point away from triangle center
    if dot(normal, sub(v1, triangle_center)) < 0.0 {
        normal = neg(normal);
    }

    let result = crate::math_functions::line_distance(v1, triangle_edge, p1, capsule_edge);

    if result.fraction1 < 0.0
        || 1.0 < result.fraction1
        || result.fraction2 < 0.0
        || 1.0 < result.fraction2
    {
        // closest point beyond end points
        return;
    }

    let point = lerp(
        mul_sub(result.point1, capsule.radius, normal),
        result.point2,
        0.5,
    );

    let separation = dot(normal, sub(result.point2, result.point1));

    manifold.normal = normal;
    manifold.point_count = 1;

    let edges_features = [
        TriangleFeature::Edge1,
        TriangleFeature::Edge2,
        TriangleFeature::Edge3,
    ];
    manifold.feature = edges_features[query.index_a as usize];

    let pt = &mut manifold.points[0];
    pt.point = point;
    pt.separation = separation - capsule.radius;
    pt.pair = make_feature_pair(
        FeatureOwner::ShapeA,
        query.index_a,
        FeatureOwner::ShapeB,
        query.index_b,
    );
}

/// Collide a sphere and a triangle. (b3CollideSphereAndTriangle)
pub fn collide_sphere_and_triangle(
    manifold: &mut LocalManifold,
    capacity: i32,
    sphere_a: &Sphere,
    triangle_b: &[Vec3; 3],
) {
    manifold.point_count = 0;

    if capacity == 0 {
        return;
    }

    let center = sphere_a.center;
    let v1 = triangle_b[0];
    let v2 = triangle_b[1];
    let v3 = triangle_b[2];
    let plane = make_plane_from_points(v1, v2, v3);

    let offset = plane_separation(plane, center);
    if offset < 0.0 {
        // Cull back side collision
        return;
    }

    let closest = closest_point_on_triangle(v1, v2, v3, center);

    let squared_distance = distance_squared(closest.point, center);
    let speculative = speculative_distance();
    let max_distance = sphere_a.radius + speculative;
    if squared_distance > max_distance * max_distance {
        return;
    }

    let distance = squared_distance.sqrt();
    let normal = if distance * distance > 1000.0 * f32::MIN_POSITIVE {
        mul_sv(1.0 / distance, sub(center, closest.point))
    } else {
        normalize(cross(sub(v2, v1), sub(v3, v1)))
    };

    // contact point mid-way
    let contact_point = mul_sv(
        0.5,
        add(mul_sub(center, sphere_a.radius, normal), closest.point),
    );

    manifold.normal = normal;
    manifold.point_count = 1;
    manifold.feature = closest.feature;
    manifold.squared_distance = squared_distance;

    let mp = &mut manifold.points[0];
    mp.point = contact_point;
    mp.separation = distance - sphere_a.radius;
    mp.pair = FEATURE_PAIR_SINGLE;
}

/// Collide a capsule and a triangle. (b3CollideCapsuleAndTriangle)
pub fn collide_capsule_and_triangle(
    manifold: &mut LocalManifold,
    capacity: i32,
    capsule_a: &Capsule,
    triangle_b: &[Vec3; 3],
    cache: &mut SimplexCache,
) {
    manifold.point_count = 0;

    if capacity < 2 {
        return;
    }

    let v1 = triangle_b[0];
    let v2 = triangle_b[1];
    let v3 = triangle_b[2];
    let plane = make_plane_from_points(v1, v2, v3);
    let capsule_center = lerp(capsule_a.center1, capsule_a.center2, 0.5);

    let offset = plane_separation(plane, capsule_center);
    if offset < 0.0 {
        // Cull back side collision
        return;
    }

    let distance_input = DistanceInput {
        proxy_a: make_proxy(triangle_b, 0.0),
        proxy_b: make_proxy(&[capsule_a.center1, capsule_a.center2], 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let distance_output = shape_distance(&distance_input, cache, None);

    let radius = capsule_a.radius;
    if distance_output.distance > radius + speculative_distance() {
        // Shapes are separated, persist the cache
        return;
    }

    if distance_output.distance > 100.0 * f32::EPSILON {
        // Shallow penetration
        let delta = normalize(sub(distance_output.point_b, distance_output.point_a));

        // Try to create two contact points if closest points difference is nearly parallel to face normal
        const K_TOLERANCE: f32 = 0.2;
        let cos_angle = abs_float(dot(plane.normal, delta));
        if cos_angle > K_TOLERANCE {
            let mut segment = [
                ClipVertex {
                    position: capsule_a.center1,
                    separation: 0.0,
                    pair: make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0),
                },
                ClipVertex {
                    position: capsule_a.center2,
                    separation: 0.0,
                    pair: make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1),
                },
            ];

            if clip_segment_to_triangle_face(&mut segment, triangle_b, plane) {
                let distance1 = plane_separation(plane, segment[0].position);
                let distance2 = plane_separation(plane, segment[1].position);

                let normal = plane.normal;
                let point1 = mul_sub(segment[0].position, 0.5 * (radius + distance1), normal);
                let point2 = mul_sub(segment[1].position, 0.5 * (radius + distance2), normal);

                manifold.normal = normal;
                manifold.feature = TriangleFeature::TriangleFace;
                manifold.point_count = 2;

                let mp = &mut manifold.points[0];
                mp.point = point1;
                mp.separation = distance1 - radius;
                mp.pair = segment[0].pair;

                let mp = &mut manifold.points[1];
                mp.point = point2;
                mp.separation = distance2 - radius;
                mp.pair = segment[1].pair;

                return;
            }
        }

        // Create contact from closest points
        let point = mul_sv(
            0.5,
            add(
                mul_sub(distance_output.point_a, radius, delta),
                distance_output.point_b,
            ),
        );

        manifold.normal = delta;
        manifold.point_count = 1;
        manifold.feature = get_triangle_feature(cache);

        let mp = &mut manifold.points[0];
        mp.point = point;
        mp.separation = distance_output.distance - radius;
        mp.pair = FEATURE_PAIR_SINGLE;

        return;
    }

    // Deep penetration
    let face_query = query_triangle_face_and_capsule(plane, capsule_a);
    if face_query.separation > radius {
        return;
    }

    let edge_query = query_triangle_and_capsule_edges(triangle_b, capsule_a);
    if edge_query.separation > radius {
        return;
    }

    // Create face contact
    let mut face_separation = face_query.separation - radius;
    build_triangle_and_capsule_face_contact(manifold, triangle_b, plane, capsule_a);
    if manifold.point_count == 2 {
        face_separation = min_float(manifold.points[0].separation, manifold.points[1].separation);
    }

    // Face contact can be empty if it does not realize the axis of minimum penetration.
    // Create edge contact if face contact fails or edge contact is significantly better!
    const K_REL_EDGE_TOLERANCE: f32 = 0.50;
    let k_abs_tolerance = 1.0 * crate::constants::linear_slop();
    let edge_separation = edge_query.separation - radius;
    if manifold.point_count == 0
        || edge_separation > K_REL_EDGE_TOLERANCE * face_separation + k_abs_tolerance
    {
        build_triangle_and_capsule_edge_contact(manifold, triangle_b, capsule_a, edge_query);
    }
}
