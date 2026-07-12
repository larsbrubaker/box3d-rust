//! Mesh / height-field narrow-phase manifolds.
//!
//! Port of `b3ComputeMeshManifolds` from
//! `box3d-cpp-reference/src/mesh_contact.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::mesh_cache::refresh_cache;
use super::mesh_cull::{
    claim_triangle_features, reduce_cluster, Cluster, FoundEdges, FoundVertices,
};
use super::{contact_flags, ContactCache, ContactGeometry, MeshContact};
use crate::constants::{linear_slop, mesh_rest_offset, MAX_POINTS_PER_TRIANGLE};
use crate::core::NULL_INDEX;
use crate::distance::SimplexCache;
use crate::geometry::ShapeType;
use crate::height_field::{get_height_field_material_indices, get_height_field_triangle};
use crate::manifold::{
    collide_capsule_and_triangle, collide_hull_and_triangle, collide_sphere_and_triangle,
    make_feature_id, LocalManifold, Manifold, SatCache, SeparatingFeature, TriangleFeature,
};
use crate::math_functions::{
    add, clamp_int, dot, inv_mul_world_transforms, make_matrix_from_quat, make_normal_from_points,
    min_float, min_int, mul_mv, mul_sv, rotate_vector, sub, sub_pos, WorldTransform, VEC3_ZERO,
};
use crate::mesh::{
    get_mesh_material_indices, get_mesh_triangle, Mesh, ALL_FLAT_EDGES, FLAT_EDGE1, FLAT_EDGE2,
    FLAT_EDGE3,
};
use crate::shape::{shape_flags, Shape, ShapeGeometry};
use crate::world::World;

const CLUSTER_THRESHOLD: f32 = 0.996;
const NORMAL_MATCH_TOLERANCE: f32 = 0.995;

struct TentativeTriangle {
    squared_distance: f32,
    index: i32,
}

fn mesh_contact_mut(geometry: &mut ContactGeometry) -> &mut MeshContact {
    match geometry {
        ContactGeometry::Mesh(m) => m,
        ContactGeometry::Convex(_) => {
            *geometry = ContactGeometry::Mesh(MeshContact::default());
            match geometry {
                ContactGeometry::Mesh(m) => m,
                ContactGeometry::Convex(_) => unreachable!(),
            }
        }
    }
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

/// Compute mesh/height-field manifolds for a contact. (b3ComputeMeshManifolds)
pub fn compute_mesh_manifolds(
    world: &mut World,
    worker_index: i32,
    contact_id: i32,
    shape_a: &Shape,
    material_map: Option<&[i32]>,
    xf_a: WorldTransform,
    shape_b: &Shape,
    xf_b: WorldTransform,
    is_fast: bool,
) -> bool {
    debug_assert!(
        shape_a.shape_type() == ShapeType::Mesh || shape_a.shape_type() == ShapeType::Height
    );

    let aabb_b = world.shapes[shape_b.id as usize].aabb;
    {
        let mesh = mesh_contact_mut(&mut world.contacts[contact_id as usize].geometry);
        refresh_cache(mesh, shape_a, xf_a, aabb_b);
    }

    let triangle_count = match &world.contacts[contact_id as usize].geometry {
        ContactGeometry::Mesh(m) => m.triangle_cache.len(),
        ContactGeometry::Convex(_) => 0,
    };

    let transform_a_to_b = inv_mul_world_transforms(xf_b, xf_a);
    let relative_matrix = make_matrix_from_quat(transform_a_to_b.q);
    let linear_slop = linear_slop();
    let rest_offset = mesh_rest_offset();

    let point_buffer_capacity = MAX_POINTS_PER_TRIANGLE * triangle_count;
    let mut point_buffer =
        vec![crate::manifold::LocalManifoldPoint::default(); point_buffer_capacity];
    let mut total_point_count = 0i32;

    let mut manifold_buffer = vec![LocalManifold::default(); triangle_count];
    let mut manifold_count = 0i32;

    let mut accepted: Vec<i32> = Vec::with_capacity(triangle_count);
    let mut tentative_manifolds: Vec<i32> = Vec::with_capacity(triangle_count);
    let mut tentative_triangles: Vec<TentativeTriangle> = Vec::with_capacity(triangle_count);

    let mut found_edges = FoundEdges::new();
    let mut found_vertices = FoundVertices::new();

    // Clone triangle cache for the collide loop (caches mutate SAT/simplex).
    let mut triangle_caches = match &world.contacts[contact_id as usize].geometry {
        ContactGeometry::Mesh(m) => m.triangle_cache.clone(),
        ContactGeometry::Convex(_) => Vec::new(),
    };

    let type_b = shape_b.shape_type();
    let geom_b = &shape_b.geometry;

    for index in 0..triangle_count {
        if total_point_count + 3 >= point_buffer_capacity as i32 {
            break;
        }

        let triangle_index = triangle_caches[index].triangle_index;
        let triangle = match &shape_a.geometry {
            ShapeGeometry::Mesh { data, scale } => get_mesh_triangle(
                &Mesh {
                    data,
                    scale: *scale,
                },
                triangle_index,
            ),
            ShapeGeometry::HeightField(hf) => get_height_field_triangle(hf, triangle_index),
            _ => unreachable!(),
        };

        let vertices = [
            add(
                mul_mv(relative_matrix, triangle.vertices[0]),
                transform_a_to_b.p,
            ),
            add(
                mul_mv(relative_matrix, triangle.vertices[1]),
                transform_a_to_b.p,
            ),
            add(
                mul_mv(relative_matrix, triangle.vertices[2]),
                transform_a_to_b.p,
            ),
        ];

        let point_capacity = min_int(
            point_buffer_capacity as i32 - total_point_count,
            MAX_POINTS_PER_TRIANGLE as i32,
        );
        let mut local = LocalManifold::default();
        local.triangle_flags = triangle.flags;
        local.feature = TriangleFeature::None;

        match type_b {
            ShapeType::Capsule => {
                let ShapeGeometry::Capsule(capsule) = geom_b else {
                    unreachable!()
                };
                let cache = ensure_simplex(&mut triangle_caches[index].cache);
                collide_capsule_and_triangle(&mut local, point_capacity, capsule, &vertices, cache);
            }
            ShapeType::Hull => {
                let ShapeGeometry::Hull(hull) = geom_b else {
                    unreachable!()
                };
                let cache = ensure_sat(&mut triangle_caches[index].cache);
                if is_fast && cache.type_ == SeparatingFeature::EdgePairAxis as u8 {
                    *cache = SatCache::default();
                }
                collide_hull_and_triangle(
                    &mut local,
                    point_capacity,
                    hull,
                    vertices[0],
                    vertices[1],
                    vertices[2],
                    triangle.flags,
                    cache,
                );
                world.task_contexts[worker_index as usize].sat_call_count += 1;
                world.task_contexts[worker_index as usize].sat_cache_hit_count += cache.hit as i32;
            }
            ShapeType::Sphere => {
                let ShapeGeometry::Sphere(sphere) = geom_b else {
                    unreachable!()
                };
                collide_sphere_and_triangle(&mut local, point_capacity, sphere, &vertices);
            }
            _ => {
                debug_assert!(false, "mesh contact expects sphere/capsule/hull B");
                return false;
            }
        }

        let manifold_point_count = local.point_count;
        if manifold_point_count > 0 {
            debug_assert!(local.feature != TriangleFeature::None);

            // Copy points into the shared buffer (C: manifold->points = pointBuffer + offset).
            for j in 0..manifold_point_count {
                point_buffer[(total_point_count + j) as usize] = local.points[j as usize];
            }

            local.triangle_index = triangle_index;
            local.triangle_normal = make_normal_from_points(vertices[0], vertices[1], vertices[2]);
            local.i1 = triangle.i1;
            local.i2 = triangle.i2;
            local.i3 = triangle.i3;
            // Stash buffer offset in unused squared_distance field? Keep points in local
            // and also track buffer range via manifold_buffer index.
            // Re-copy local points from buffer for later cluster population.
            for j in 0..manifold_point_count {
                local.points[j as usize] = point_buffer[(total_point_count + j) as usize];
            }

            manifold_buffer[manifold_count as usize] = local;
            let m_index = manifold_count;
            manifold_count += 1;
            total_point_count += manifold_point_count;

            let feature = manifold_buffer[m_index as usize].feature;
            if feature == TriangleFeature::TriangleFace {
                claim_triangle_features(
                    &mut found_edges,
                    &mut found_vertices,
                    triangle.i1,
                    triangle.i2,
                    triangle.i3,
                );
                accepted.push(m_index);
            } else if feature == TriangleFeature::HullFace {
                let cos_normal_angle = dot(
                    manifold_buffer[m_index as usize].triangle_normal,
                    manifold_buffer[m_index as usize].normal,
                );
                if cos_normal_angle > 0.5 {
                    claim_triangle_features(
                        &mut found_edges,
                        &mut found_vertices,
                        triangle.i1,
                        triangle.i2,
                        triangle.i3,
                    );
                    accepted.push(m_index);
                } else {
                    let mut min_separation = manifold_buffer[m_index as usize].points[0].separation;
                    for i in 1..manifold_point_count {
                        min_separation = min_float(
                            min_separation,
                            manifold_buffer[m_index as usize].points[i as usize].separation,
                        );
                    }

                    if min_separation < -2.0 * linear_slop {
                        claim_triangle_features(
                            &mut found_edges,
                            &mut found_vertices,
                            triangle.i1,
                            triangle.i2,
                            triangle.i3,
                        );
                        accepted.push(m_index);
                    } else {
                        let tentative_index = tentative_manifolds.len() as i32;
                        tentative_triangles.push(TentativeTriangle {
                            squared_distance: manifold_buffer[m_index as usize].squared_distance,
                            index: tentative_index,
                        });
                        tentative_manifolds.push(m_index);
                    }
                }
            } else {
                let tentative_index = tentative_manifolds.len() as i32;
                tentative_triangles.push(TentativeTriangle {
                    squared_distance: manifold_buffer[m_index as usize].squared_distance,
                    index: tentative_index,
                });
                tentative_manifolds.push(m_index);
            }
        }
    }

    // Write updated triangle caches back.
    if let ContactGeometry::Mesh(mesh) = &mut world.contacts[contact_id as usize].geometry {
        mesh.triangle_cache = triangle_caches;
    }

    debug_assert!(accepted.len() <= triangle_count);
    debug_assert!(tentative_manifolds.len() <= triangle_count);
    debug_assert!(tentative_triangles.len() <= triangle_count);

    if type_b == ShapeType::Sphere {
        tentative_triangles
            .sort_unstable_by(|a, b| a.squared_distance.total_cmp(&b.squared_distance));

        for t in &tentative_triangles {
            let m = &manifold_buffer[tentative_manifolds[t.index as usize] as usize];
            let added_edge1 = found_edges.add(m.i1, m.i2);
            let added_edge2 = found_edges.add(m.i2, m.i3);
            let added_edge3 = found_edges.add(m.i3, m.i1);
            let added_vertex1 = found_vertices.add(m.i1);
            let added_vertex2 = found_vertices.add(m.i2);
            let added_vertex3 = found_vertices.add(m.i3);

            let should_collide = match m.feature {
                TriangleFeature::None | TriangleFeature::TriangleFace => {
                    debug_assert!(false);
                    false
                }
                TriangleFeature::Edge1 => added_edge1,
                TriangleFeature::Edge2 => added_edge2,
                TriangleFeature::Edge3 => added_edge3,
                TriangleFeature::Vertex1 => added_vertex1,
                TriangleFeature::Vertex2 => added_vertex2,
                TriangleFeature::Vertex3 => added_vertex3,
                TriangleFeature::HullFace => {
                    debug_assert!(false);
                    false
                }
            };

            if should_collide {
                accepted.push(tentative_manifolds[t.index as usize]);
            }
        }
    } else {
        for &m_index in &tentative_manifolds {
            let m = &manifold_buffer[m_index as usize];
            let triangle_flags = m.triangle_flags;

            if (triangle_flags & ALL_FLAT_EDGES) == ALL_FLAT_EDGES {
                continue;
            }

            if (triangle_flags & FLAT_EDGE1) == FLAT_EDGE1 && found_edges.find(m.i1, m.i2) {
                continue;
            }
            if (triangle_flags & FLAT_EDGE2) == FLAT_EDGE2 && found_edges.find(m.i2, m.i3) {
                continue;
            }
            if (triangle_flags & FLAT_EDGE3) == FLAT_EDGE3 && found_edges.find(m.i3, m.i1) {
                continue;
            }

            accepted.push(m_index);
        }
    }

    debug_assert!(accepted.len() <= triangle_count);

    if accepted.is_empty() {
        world.contacts[contact_id as usize].manifolds.clear();
        return false;
    }

    let accepted_count = accepted.len();
    let mut clusters: Vec<Cluster> = Vec::with_capacity(accepted_count);
    let mut cluster_memberships = vec![NULL_INDEX; accepted_count];
    let mut cluster_point_count = 0i32;

    for i in 0..accepted_count {
        cluster_memberships[i] = NULL_INDEX;
        let manifold = &manifold_buffer[accepted[i] as usize];
        cluster_point_count += manifold.point_count;

        let manifold_normal = manifold.normal;
        let triangle_normal = manifold.triangle_normal;
        let mut cluster_index = NULL_INDEX;
        for (j, cluster) in clusters.iter().enumerate() {
            let cos_manifold = dot(cluster.manifold_normal, manifold_normal);
            let cos_triangle = dot(cluster.triangle_normal, triangle_normal);
            if cos_manifold <= CLUSTER_THRESHOLD || cos_triangle <= CLUSTER_THRESHOLD {
                continue;
            }
            cluster_index = j as i32;
            break;
        }

        if cluster_index != NULL_INDEX {
            cluster_memberships[i] = cluster_index;
            clusters[cluster_index as usize].point_capacity += manifold.point_count;
        } else {
            cluster_memberships[i] = clusters.len() as i32;
            clusters.push(Cluster::new(
                manifold_normal,
                triangle_normal,
                manifold.point_count,
            ));
        }
    }

    if cluster_point_count == 0 {
        return false;
    }

    let cluster_count = clusters.len();
    let mut cluster_points =
        vec![crate::manifold::LocalManifoldPoint::default(); cluster_point_count as usize];
    let mut point_offset = 0usize;
    for cluster in &mut clusters {
        cluster.point_start = point_offset;
        cluster.point_count = 0;
        point_offset += cluster.point_capacity as usize;
    }

    for i in 0..accepted_count {
        let cluster_index = cluster_memberships[i];
        if cluster_index == NULL_INDEX {
            continue;
        }
        debug_assert!(0 <= cluster_index && (cluster_index as usize) < cluster_count);

        let am = &manifold_buffer[accepted[i] as usize];
        let cm = &mut clusters[cluster_index as usize];
        for j in 0..am.point_count {
            debug_assert!(cm.point_count < cm.point_capacity);
            let ap = &am.points[j as usize];
            let slot = cm.point_start + cm.point_count as usize;
            cluster_points[slot] = crate::manifold::LocalManifoldPoint {
                triangle_index: am.triangle_index,
                point: ap.point,
                separation: ap.separation,
                pair: ap.pair,
            };
            cm.point_count += 1;
        }
    }

    for cluster in &mut clusters {
        debug_assert!(cluster.point_count == cluster.point_capacity);
        let start = cluster.point_start;
        let count = cluster.point_count;
        let reduced = reduce_cluster(
            &mut cluster_points[start..start + count as usize],
            count,
            cluster.triangle_normal,
        );
        cluster.point_count = reduced;
    }

    let old_manifolds = world.contacts[contact_id as usize].manifolds.clone();
    let old_manifold_count = old_manifolds.len() as i32;

    {
        let contact = &mut world.contacts[contact_id as usize];
        if old_manifold_count != cluster_count as i32 {
            contact.manifolds = vec![Manifold::default(); cluster_count];
        } else {
            for m in &mut contact.manifolds {
                *m = Manifold::default();
            }
        }
    }

    let mut consumed = vec![false; old_manifold_count.max(0) as usize];
    let matrix_b = make_matrix_from_quat(xf_b.q);
    let offset_a = sub_pos(xf_b.p, xf_a.p);

    for i in 0..cluster_count {
        let cm = &clusters[i];
        let point_count = cm.point_count;
        debug_assert!(
            0 < point_count && (point_count as usize) <= crate::constants::MAX_MANIFOLD_POINTS
        );

        let cluster_normal = mul_mv(matrix_b, cm.manifold_normal);
        let mut best_dot = NORMAL_MATCH_TOLERANCE;
        let mut best_index = NULL_INDEX;

        for j in 0..old_manifold_count {
            if consumed[j as usize] {
                continue;
            }
            let d = dot(old_manifolds[j as usize].normal, cluster_normal);
            if d > best_dot {
                best_index = j;
                best_dot = d;
            }
        }

        let matched = if best_index != NULL_INDEX {
            consumed[best_index as usize] = true;
            Some(best_index)
        } else {
            None
        };

        {
            let manifold = &mut world.contacts[contact_id as usize].manifolds[i];
            manifold.point_count = point_count;
            manifold.normal = cluster_normal;

            if let Some(mi) = matched {
                let matched_manifold = &old_manifolds[mi as usize];
                manifold.friction_impulse = matched_manifold.friction_impulse;
                manifold.rolling_impulse = matched_manifold.rolling_impulse;
                manifold.twist_impulse = matched_manifold.twist_impulse;
            }

            for j in 0..point_count {
                let source = &cluster_points[cm.point_start + j as usize];
                let target = &mut manifold.points[j as usize];

                target.anchor_b = mul_mv(matrix_b, source.point);
                target.anchor_a = add(target.anchor_b, offset_a);
                target.separation = source.separation - rest_offset;
                target.feature_id = make_feature_id(source.pair);
                target.triangle_index = source.triangle_index;
                target.normal_impulse = 0.0;
                target.persisted = false;
                target.total_normal_impulse = 0.0;
                target.normal_velocity = 0.0;
                target.base_separation = 0.0;
            }
        }

        if let Some(mi) = matched {
            // Match impulses by feature id + triangle index (claim with NULL_INDEX).
            let mut old_points = old_manifolds[mi as usize].points;
            let old_point_count = old_manifolds[mi as usize].point_count;
            let manifold = &mut world.contacts[contact_id as usize].manifolds[i];
            for j in 0..point_count {
                let target = &mut manifold.points[j as usize];
                for k in 0..old_point_count {
                    let old_pt = &mut old_points[k as usize];
                    if target.feature_id == old_pt.feature_id
                        && target.triangle_index == old_pt.triangle_index
                    {
                        target.normal_impulse = old_pt.normal_impulse;
                        target.persisted = true;
                        old_pt.triangle_index = NULL_INDEX;
                        break;
                    }
                }
            }
        }
    }

    apply_mesh_materials(
        world,
        contact_id,
        shape_a,
        material_map,
        shape_b,
        xf_a,
        xf_b,
    );

    true
}

fn apply_mesh_materials(
    world: &mut World,
    contact_id: i32,
    shape_a: &Shape,
    material_map: Option<&[i32]>,
    shape_b: &Shape,
    xf_a: WorldTransform,
    xf_b: WorldTransform,
) {
    let materials_a = shape_a.shape_materials();
    let material_b = shape_b.get_material(0);
    let mut tangent_velocity_a = VEC3_ZERO;

    let friction_cb = world
        .friction_callback
        .unwrap_or(crate::world::default_friction_callback);
    let restitution_cb = world
        .restitution_callback
        .unwrap_or(crate::world::default_restitution_callback);

    if shape_a.material_count() > 0 {
        let mut friction = 0.0;
        let mut restitution = 0.0;
        let mut sample_count = 0.0;

        let cluster_count = world.contacts[contact_id as usize].manifolds.len();
        for i in 0..cluster_count {
            let point_count = world.contacts[contact_id as usize].manifolds[i].point_count;
            for j in 0..point_count {
                let triangle_index = world.contacts[contact_id as usize].manifolds[i].points
                    [j as usize]
                    .triangle_index;

                let mut material_index = match &shape_a.geometry {
                    ShapeGeometry::Mesh { data, .. } => {
                        let mut mi =
                            get_mesh_material_indices(data)[triangle_index as usize] as i32;
                        if let Some(map) = material_map {
                            mi = map[mi as usize];
                        }
                        mi
                    }
                    ShapeGeometry::HeightField(hf) => {
                        get_height_field_material_indices(hf)[(triangle_index >> 1) as usize] as i32
                    }
                    _ => 0,
                };

                material_index = clamp_int(material_index, 0, shape_a.material_count() - 1);
                let material = materials_a[material_index as usize];
                friction += friction_cb(
                    material.friction,
                    material.user_material_id,
                    material_b.friction,
                    material_b.user_material_id,
                );
                restitution += restitution_cb(
                    material.restitution,
                    material.user_material_id,
                    material_b.restitution,
                    material_b.user_material_id,
                );
                tangent_velocity_a = add(tangent_velocity_a, material.tangent_velocity);
                sample_count += 1.0;
            }
        }

        if sample_count > 0.0 {
            let inv_count = 1.0 / sample_count;
            let contact = &mut world.contacts[contact_id as usize];
            contact.friction = inv_count * friction;
            contact.restitution = inv_count * restitution;
            tangent_velocity_a = mul_sv(inv_count, tangent_velocity_a);
        }
    } else {
        let material_a = materials_a[0];
        let contact = &mut world.contacts[contact_id as usize];
        contact.friction = friction_cb(
            material_a.friction,
            material_a.user_material_id,
            material_b.friction,
            material_b.user_material_id,
        );
        contact.restitution = restitution_cb(
            material_a.restitution,
            material_a.user_material_id,
            material_b.restitution,
            material_b.user_material_id,
        );
        tangent_velocity_a = material_a.tangent_velocity;
    }

    tangent_velocity_a = rotate_vector(xf_a.q, tangent_velocity_a);

    let radius_b = match &shape_b.geometry {
        ShapeGeometry::Sphere(s) => s.radius,
        ShapeGeometry::Capsule(c) => c.radius,
        ShapeGeometry::Hull(h) => h.inner_radius,
        _ => 0.0,
    };

    let contact = &mut world.contacts[contact_id as usize];
    contact.rolling_resistance = material_b.rolling_resistance * radius_b;

    let tangent_velocity_b = rotate_vector(xf_b.q, material_b.tangent_velocity);
    contact.tangent_velocity = sub(tangent_velocity_a, tangent_velocity_b);
}

/// Apply hit-event flags after mesh manifold compute. (contact.c mesh branch)
pub fn apply_mesh_hit_flags(world: &mut World, contact_id: i32, shape_a: &Shape, shape_b: &Shape) {
    if (shape_a.flags & shape_flags::ENABLE_HIT_EVENTS) != 0
        || (shape_b.flags & shape_flags::ENABLE_HIT_EVENTS) != 0
    {
        world.contacts[contact_id as usize].flags |= contact_flags::SIM_ENABLE_HIT_EVENT;
    } else {
        world.contacts[contact_id as usize].flags &= !contact_flags::SIM_ENABLE_HIT_EVENT;
    }
}
