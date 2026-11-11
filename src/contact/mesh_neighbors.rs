//! Mesh ghost collision handling: Voronoi tests of a triangle manifold normal against the
//! neighboring triangles, and the boundary edge fallback.
//!
//! Port of `b3NeighborResult`, `b3GetNeighborTangent`, `b3IsInVertexRegion`,
//! `b3IsInVertexCone`, `b3AdmitsVertexNormal`, `b3TestNeighbors`, `b3ClassifyHullFacePoint`,
//! `b3IsSubset`, `b3TestHullFaceNeighbors`, and `b3CollideBoundaryEdge` from
//! `box3d-cpp-reference/src/mesh_contact.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::{linear_slop, overlap_slop};
use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::height_field::{get_height_field_triangle, get_height_field_vertex_triangles};
use crate::hull::{get_hull_points, HullData};
use crate::manifold::{
    make_feature_pair, FeatureOwner, FeaturePair, LocalManifold, TriangleFeature,
};
use crate::math_functions::{
    abs_float, add, clamp_float, cross, distance_squared, dot, length_squared, lerp,
    make_normal_from_points, min_float, mul_add, mul_mv, mul_sub, neg, normalize, sub, transpose,
    Matrix3, Vec3, TRANSFORM_IDENTITY,
};
use crate::mesh::is_in_mesh_vertex_ring;
use crate::shape::{get_neighbor_normal, is_convex_vertex, TriangleShape};

/// (b3NeighborResult)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum NeighborResult {
    /// Admit for flat edges. (b3_neighborAdmit)
    Admit,
    /// Skip for convex edges. (b3_neighborSkip)
    Skip,
    /// Snap for concave edges. (b3_neighborSnap)
    Snap,
    /// Drop for a cliff. (b3_neighborDrop)
    Drop,
}

/// 1.0027 degrees (B3_NEIGHBOR_TOLERANCE)
pub(super) const NEIGHBOR_TOLERANCE: f32 = 0.0175;

const VERTEX_FEATURES: [TriangleFeature; 3] = [
    TriangleFeature::Vertex1,
    TriangleFeature::Vertex2,
    TriangleFeature::Vertex3,
];

const EDGE_FEATURES: [TriangleFeature; 3] = [
    TriangleFeature::Edge1,
    TriangleFeature::Edge2,
    TriangleFeature::Edge3,
];

/// Cross product of triangle edge with neighbor triangle normal. A vector pointing away from
/// the edge in the neighbor triangle plane towards its center.
/// tangent = cross(e, n)
/// Returns whether the edge has a neighbor and the tangent. Without a neighbor the tangent is
/// the cliff axis. (b3GetNeighborTangent)
fn get_neighbor_tangent(
    shape: &TriangleShape<'_>,
    triangle_index: i32,
    vertices: &[Vec3; 3],
    mesh_to_local: Matrix3,
    edge_index: i32,
) -> (bool, Vec3) {
    let Some(mut neighbor_normal) = get_neighbor_normal(shape, triangle_index, edge_index) else {
        // Cliff edge axis along negative triangle normal.
        return (
            false,
            neg(make_normal_from_points(
                vertices[0],
                vertices[1],
                vertices[2],
            )),
        );
    };

    neighbor_normal = mul_mv(mesh_to_local, neighbor_normal);
    let edge = normalize(sub(
        vertices[((edge_index + 1) % 3) as usize],
        vertices[edge_index as usize],
    ));
    (true, cross(edge, neighbor_normal))
}

/// Is the normal in the vertex Voronoi region? (b3IsInVertexRegion)
fn is_in_vertex_region(vertices: &[Vec3; 3], vertex_index: i32, normal: Vec3) -> bool {
    let vertex = vertices[vertex_index as usize];
    let next = vertices[if vertex_index < 2 {
        vertex_index + 1
    } else {
        0
    } as usize];
    let previous = vertices[if vertex_index > 0 {
        vertex_index - 1
    } else {
        2
    } as usize];
    let e1 = sub(next, vertex);
    let e2 = sub(previous, vertex);
    dot(normal, e1) <= 0.0 && dot(normal, e2) <= 0.0
}

/// This is used to avoid ghost collisions at vertices. It is called rarely.
/// trees25 : 2.6%
/// mesh_drop : 0.04%
/// (b3IsInVertexCone)
fn is_in_vertex_cone(
    shape: &TriangleShape<'_>,
    triangle_index: i32,
    vertex_index: i32,
    mesh_to_local: Matrix3,
    normal: Vec3,
) -> bool {
    let mesh_normal = mul_mv(transpose(mesh_to_local), normal);
    let tolerance = NEIGHBOR_TOLERANCE;

    let height_field = match shape {
        TriangleShape::Mesh(mesh) => {
            return is_in_mesh_vertex_ring(
                mesh,
                triangle_index,
                vertex_index,
                mesh_normal,
                tolerance,
            );
        }
        TriangleShape::HeightField(height_field) => *height_field,
    };

    let tolerance_squared = tolerance * tolerance;

    let triangle = get_height_field_triangle(height_field, triangle_index);
    let indices = [triangle.i1, triangle.i2, triangle.i3];
    let vertex = indices[vertex_index as usize];

    let mut fan_triangles = [0i32; 6];
    let fan_count = get_height_field_vertex_triangles(height_field, vertex, &mut fan_triangles);

    for &fan_index in fan_triangles.iter().take(fan_count as usize) {
        let fan = get_height_field_triangle(height_field, fan_index);
        let fan_indices = [fan.i1, fan.i2, fan.i3];
        let k = if fan_indices[0] == vertex {
            0
        } else if fan_indices[1] == vertex {
            1
        } else {
            2
        };
        debug_assert!(fan_indices[k] == vertex);

        let next = if k < 2 { k + 1 } else { 0 };
        let previous = if k > 0 { k - 1 } else { 2 };
        let edges = [
            sub(fan.vertices[next], fan.vertices[k]),
            sub(fan.vertices[previous], fan.vertices[k]),
        ];
        let edge_indices = [k as i32, previous as i32];

        for j in 0..2 {
            let lean = dot(mesh_normal, edges[j]);
            if lean > 0.0 && lean * lean > tolerance_squared * length_squared(edges[j]) {
                return false;
            }

            if get_neighbor_normal(shape, fan_index, edge_indices[j]).is_none() {
                // Cliff at boundary or hole.
                let fan_normal =
                    make_normal_from_points(fan.vertices[0], fan.vertices[1], fan.vertices[2]);
                if dot(mesh_normal, fan_normal) < -tolerance {
                    return false;
                }
            }
        }
    }

    true
}

/// Check the vertex flags to determined if this normal is admissible. (b3AdmitsVertexNormal)
fn admits_vertex_normal(
    shape: &TriangleShape<'_>,
    triangle_index: i32,
    vertices: &[Vec3; 3],
    vertex_index: i32,
    mesh_to_local: Matrix3,
    normal: Vec3,
) -> bool {
    if !is_in_vertex_region(vertices, vertex_index, normal) {
        // Not in the vertex region so this vertex cannot admit the normal.
        return false;
    }

    if !is_convex_vertex(shape, triangle_index, vertex_index) {
        return false;
    }

    is_in_vertex_cone(shape, triangle_index, vertex_index, mesh_to_local, normal)
}

/// Voronoi test of an edge or vertex normal against the neighbor normals. Returns the result
/// and the owner edge, the first edge with a neighbor whose face region the normal leans into
/// (or [`NULL_INDEX`]). (b3TestNeighbors)
pub(super) fn test_neighbors(
    shape: &TriangleShape<'_>,
    triangle_index: i32,
    vertices: &[Vec3; 3],
    mesh_to_local: Matrix3,
    feature: TriangleFeature,
    candidate_normal: Vec3,
) -> (NeighborResult, i32) {
    let mut owner_edge = NULL_INDEX;

    let mut edges = [0i32; 2];
    let mut edge_count = 1;
    let mut vertex_index = NULL_INDEX;

    match feature {
        TriangleFeature::Edge1 => {
            edges[0] = 0;
        }

        TriangleFeature::Edge2 => {
            edges[0] = 1;
        }

        TriangleFeature::Edge3 => {
            edges[0] = 2;
        }

        TriangleFeature::Vertex1 => {
            edges[0] = 0;
            edges[1] = 2;
            edge_count = 2;
            vertex_index = 0;
        }

        TriangleFeature::Vertex2 => {
            edges[0] = 0;
            edges[1] = 1;
            edge_count = 2;
            vertex_index = 1;
        }

        TriangleFeature::Vertex3 => {
            edges[0] = 1;
            edges[1] = 2;
            edge_count = 2;
            vertex_index = 2;
        }

        _ => {
            return (NeighborResult::Admit, owner_edge);
        }
    }

    let triangle_normal = make_normal_from_points(vertices[0], vertices[1], vertices[2]);
    let tolerance = NEIGHBOR_TOLERANCE;
    let mut admit = true;
    let mut convex = false;

    for &edge in edges.iter().take(edge_count) {
        // neighborTangent = cross(edgeAxis, neighborTriangleNormal)
        let (have_neighbor, neighbor_tangent) =
            get_neighbor_tangent(shape, triangle_index, vertices, mesh_to_local, edge);

        // Does the candidate normal lean into the neighbor's face region?
        let admitted = dot(candidate_normal, neighbor_tangent) <= tolerance;
        if !admitted && have_neighbor && owner_edge == NULL_INDEX {
            owner_edge = edge;
        }

        admit = admit && admitted;

        // Convex if at least one edge is convex.
        convex = convex || dot(triangle_normal, neighbor_tangent) < -tolerance;
    }

    if admit {
        if vertex_index == NULL_INDEX
            || is_in_vertex_cone(
                shape,
                triangle_index,
                vertex_index,
                mesh_to_local,
                candidate_normal,
            )
        {
            return (NeighborResult::Admit, owner_edge);
        }

        let in_vertex_region = is_in_vertex_region(vertices, vertex_index, candidate_normal);
        let result = if in_vertex_region {
            NeighborResult::Skip
        } else {
            NeighborResult::Snap
        };
        return (result, owner_edge);
    }

    // Check vertex area first to avoid a saddle tunnel.
    if vertex_index != NULL_INDEX
        && is_in_vertex_region(vertices, vertex_index, candidate_normal)
        && is_in_vertex_cone(
            shape,
            triangle_index,
            vertex_index,
            mesh_to_local,
            candidate_normal,
        )
    {
        return (NeighborResult::Admit, owner_edge);
    }

    if convex {
        // Edge is convex. Drop for a cliff with no deep consideration.
        let result = if owner_edge == NULL_INDEX {
            NeighborResult::Drop
        } else {
            NeighborResult::Skip
        };
        return (result, owner_edge);
    }

    (NeighborResult::Snap, owner_edge)
}

/// Classify a clipped hull point relative to the triangle features. ORs into the vertex and
/// edge masks. Returns false if the point isn't close to an edge or vertex.
/// (b3ClassifyHullFacePoint)
fn classify_hull_face_point(
    vertices: &[Vec3; 3],
    triangle_normal: Vec3,
    point: Vec3,
    pair: FeaturePair,
    vertex_mask: &mut i32,
    edge_mask: &mut i32,
) -> bool {
    debug_assert!(pair.owner1 != pair.owner2 || pair.index1 != pair.index2);

    let linear_slop = linear_slop();

    let mut edges = [0i32; 3];
    let mut edge_count = 0;
    if pair.owner1 == FeatureOwner::ShapeA as u8 {
        edges[edge_count] = pair.index1 as i32;
        edge_count += 1;
    }

    if pair.owner2 == FeatureOwner::ShapeA as u8
        && (edge_count == 0 || pair.index2 as i32 != edges[0])
    {
        edges[edge_count] = pair.index2 as i32;
        edge_count += 1;
    }

    if edge_count == 0 {
        // Close to an edge?
        for i in 0..3 {
            let vertex = vertices[i];
            let edge = normalize(sub(vertices[if i < 2 { i + 1 } else { 0 }], vertex));
            let inward_tangent = cross(triangle_normal, edge);
            if dot(inward_tangent, sub(point, vertex)) <= linear_slop {
                edges[edge_count] = i as i32;
                edge_count += 1;
            }
        }
    }

    if edge_count == 0 || edge_count == 3 {
        // Not close to any triangle edge or triangle is very small.
        return false;
    }

    if edge_count == 2 {
        // Close to two edges, mask the shared vertex.
        let a = edges[0];
        let b = edges[1];
        let vertex = if (if a < 2 { a + 1 } else { 0 }) == b {
            b
        } else {
            a
        };
        *vertex_mask |= 1 << vertex;
        return true;
    }

    debug_assert!(edge_count == 1);

    // Close to only one edge. Determine if it should be classified as a vertex or edge point.
    let edge = edges[0];
    let next = if edge < 2 { edge + 1 } else { 0 };
    if distance_squared(point, vertices[edge as usize]) <= linear_slop * linear_slop {
        *vertex_mask |= 1 << edge;
    } else if distance_squared(point, vertices[next as usize]) <= linear_slop * linear_slop {
        *vertex_mask |= 1 << next;
    } else {
        *edge_mask |= 1 << edge;
    }

    true
}

/// (b3IsSubset)
#[inline]
fn is_subset(mask: i32, allowed: i32) -> bool {
    (mask & !allowed) == 0
}

/// Voronoi test of neighbors based on deepest point.
/// Ghost collisions arise when the chose manifold normal does not align with what GJK would
/// give with slight separation. A non-ghosting normal must be within the Voronoi regions the
/// triangles face, edges, and vertices. However, even normals within the Voronoi regions can
/// ghost. So we use the deepest point as a proxy for what GJK would yield with slight
/// separation. Returns the result and the near feature. (b3TestHullFaceNeighbors)
pub(super) fn test_hull_face_neighbors(
    shape: &TriangleShape<'_>,
    triangle_index: i32,
    vertices: &[Vec3; 3],
    mesh_to_local: Matrix3,
    manifold: &LocalManifold,
) -> (NeighborResult, TriangleFeature) {
    let normal = manifold.normal;
    let triangle_normal = make_normal_from_points(vertices[0], vertices[1], vertices[2]);
    let tolerance = NEIGHBOR_TOLERANCE;
    let cos_angle = dot(normal, triangle_normal);
    if cos_angle > 0.0 && cos_angle * cos_angle >= 1.0 - tolerance * tolerance {
        // Face normal (flipped to point from A to B) is parallel to triangle normal.
        return (NeighborResult::Admit, TriangleFeature::None);
    }

    let point_count = manifold.point_count as usize;
    let mut min_separation = manifold.points[0].separation;
    for point in &manifold.points[1..point_count] {
        min_separation = min_float(min_separation, point.separation);
    }

    // Determine if the overlapped points are on the triangle face or boundary. Classify
    // considering all points within slop of deepest point.
    let depth_limit = min_separation + linear_slop();
    let mut vertex_mask = 0;
    let mut edge_mask = 0;
    let mut on_face = false;
    for manifold_point in &manifold.points[..point_count] {
        if manifold_point.separation > depth_limit {
            continue;
        }

        // Clip points were moved onto the hull face, step back along the normal to the
        // triangle.
        let point = mul_sub(manifold_point.point, manifold_point.separation, normal);
        let on_boundary = classify_hull_face_point(
            vertices,
            triangle_normal,
            point,
            manifold_point.pair,
            &mut vertex_mask,
            &mut edge_mask,
        );

        on_face = on_face || !on_boundary;
    }

    // If the hull overlaps this triangle then determine if a deepest point is close to a
    // vertex.
    if min_separation < 0.0 {
        for i in 0..3 {
            if (vertex_mask & (1 << i)) == 0 {
                continue;
            }

            let admit =
                admits_vertex_normal(shape, triangle_index, vertices, i, mesh_to_local, normal);
            if admit {
                // This vertex tentatively accepts the normal.
                return (NeighborResult::Admit, VERTEX_FEATURES[i as usize]);
            }
        }
    }

    if on_face {
        // A tilted hull face over the triangle interior, the triangle face owns the normal.
        return (NeighborResult::Snap, TriangleFeature::None);
    }

    debug_assert!(edge_mask != 0 || vertex_mask != 0);

    let mut feature = TriangleFeature::None;
    let mut edges = [0i32; 2];
    let mut edge_count = 0;

    // Is the contact point close to only one vertex?
    if edge_mask == 0 && vertex_mask != 0 && (vertex_mask & (vertex_mask - 1)) == 0 {
        let vertex = if vertex_mask == 1 {
            0
        } else if vertex_mask == 2 {
            1
        } else {
            2
        };
        feature = VERTEX_FEATURES[vertex as usize];
        edges[0] = vertex;
        edges[1] = if vertex > 0 { vertex - 1 } else { 2 };
        edge_count = 2;
    } else {
        // Is the contact point close to an edge
        for i in 0..3 {
            let next = if i < 2 { i + 1 } else { 0 };
            let edge_bit = 1 << i;
            let vertex_bits = (1 << i) | (1 << next);

            // Is this edge and its vertices a subset? One mask can be zero and always a subset.
            if is_subset(edge_mask, edge_bit) && is_subset(vertex_mask, vertex_bits) {
                feature = EDGE_FEATURES[i as usize];
                edges[0] = i;
                edge_count = 1;
                break;
            }
        }
    }

    if feature == TriangleFeature::None {
        return (NeighborResult::Snap, TriangleFeature::None);
    }

    for &edge_index in edges.iter().take(edge_count) {
        let vertex = vertices[edge_index as usize];
        let edge = normalize(sub(
            vertices[if edge_index < 2 { edge_index + 1 } else { 0 } as usize],
            vertex,
        ));
        let inward_tangent = cross(triangle_normal, edge);

        // An edge or vertex normal cone tilts outward from the triangle and an edge cone is
        // perpendicular to the edge, a hull face normal need be neither.
        if dot(normal, inward_tangent) > tolerance {
            return (NeighborResult::Snap, TriangleFeature::None);
        }

        if edge_count == 1 && abs_float(dot(normal, edge)) > tolerance {
            return (NeighborResult::Snap, TriangleFeature::None);
        }
    }

    let near_feature = feature;

    // Never skip a hull face, its normal can miss the whole vertex cone so every triangle would
    // skip, the face contact is empty when the hull is off this triangle.
    let (result, _owner_edge) = test_neighbors(
        shape,
        triangle_index,
        vertices,
        mesh_to_local,
        feature,
        normal,
    );
    let result = if result == NeighborResult::Skip || result == NeighborResult::Drop {
        NeighborResult::Snap
    } else {
        result
    };
    (result, near_feature)
}

/// Special handling if a boundary collision drops all points. Rare. (b3CollideBoundaryEdge)
pub(super) fn collide_boundary_edge(
    manifold: &mut LocalManifold,
    shape_a: &TriangleShape<'_>,
    triangle_index: i32,
    vertices: &[Vec3; 3],
    mesh_to_local: Matrix3,
    hull: &HullData,
    speculative_distance: f32,
) {
    let linear_slop = linear_slop();

    let input = DistanceInput {
        proxy_a: make_proxy(vertices, 0.0),
        proxy_b: make_proxy(get_hull_points(hull), 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };
    let mut simplex_cache = SimplexCache::default();
    let output = shape_distance(&input, &mut simplex_cache, None);

    if output.distance < overlap_slop() || output.distance > speculative_distance {
        return;
    }

    let tolerance = 0.01 * linear_slop;
    let mut feature = TriangleFeature::None;
    let mut on_boundary = false;
    for i in 0..3 {
        let a = vertices[i];
        let e = sub(vertices[if i < 2 { i + 1 } else { 0 }], a);
        let t = clamp_float(dot(sub(output.point_a, a), e) / length_squared(e), 0.0, 1.0);
        let closest = mul_add(a, t, e);
        if distance_squared(closest, output.point_a) >= tolerance * tolerance {
            continue;
        }

        on_boundary =
            on_boundary || get_neighbor_normal(shape_a, triangle_index, i as i32).is_none();

        if distance_squared(a, output.point_a) < tolerance * tolerance {
            feature = VERTEX_FEATURES[i];
        } else if distance_squared(add(a, e), output.point_a) < tolerance * tolerance {
            feature = VERTEX_FEATURES[if i < 2 { i + 1 } else { 0 }];
        } else if feature == TriangleFeature::None {
            feature = EDGE_FEATURES[i];
        }
    }

    if !on_boundary || feature == TriangleFeature::None {
        return;
    }

    let (result, _owner_edge) = test_neighbors(
        shape_a,
        triangle_index,
        vertices,
        mesh_to_local,
        feature,
        output.normal,
    );
    if result != NeighborResult::Admit {
        return;
    }

    let point = &mut manifold.points[0];
    point.point = lerp(output.point_a, output.point_b, 0.5);
    point.separation = output.distance;
    point.pair = make_feature_pair(
        FeatureOwner::ShapeA,
        feature as i32,
        FeatureOwner::ShapeA,
        feature as i32,
    );
    manifold.normal = output.normal;
    manifold.point_count = 1;
    manifold.feature = feature;
}
