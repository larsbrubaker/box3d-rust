//! Mesh / height-field narrow-phase manifolds.
//!
//! Port of `b3ComputeMeshManifolds`, `b3CollideMeshAndShape`, `b3CollideMeshAndHull`, and
//! `b3CollideMeshAndSphere` from `box3d-cpp-reference/src/mesh_contact.c`. The triangle
//! narrow phase with ghost handling is `b3CollideMeshTriangles` in `mesh_triangles.rs`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::mesh_cache::{collect_triangle_indices, refresh_cache};
use super::mesh_triangles::{collide_mesh_triangles, ConvexShape, MeshClusters};
use super::{contact_flags, ContactGeometry, MeshContact, TriangleCache};
use crate::constants::{mesh_rest_offset, speculative_distance, MAX_MANIFOLD_POINTS};
use crate::core::NULL_INDEX;
use crate::geometry::{ShapeType, Sphere};
use crate::height_field::get_height_field_material_indices;
use crate::hull::HullData;
use crate::manifold::{make_feature_id, Manifold};
use crate::math_functions::{
    aabb_transform, add, clamp_int, dot, inv_mul_world_transforms, invert_transform,
    make_matrix_from_quat, min_int, mul_mv, mul_sv, rotate_vector, sub, sub_pos, Aabb, Transform,
    Vec3, WorldTransform, VEC3_ZERO,
};
use crate::mesh::{get_mesh_material_indices, Mesh};
use crate::shape::{shape_flags, Shape, ShapeGeometry, TriangleShape};
use crate::world::World;

// When this is non-zero the mesh contact skips ghost collision handling, for comparison.
// (B3_FORCE_GHOST_COLLISIONS)
const FORCE_GHOST_COLLISIONS: bool = false;

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

/// Compute mesh/height-field manifolds for a contact. (b3ComputeMeshManifolds)
#[allow(clippy::too_many_arguments)]
pub fn compute_mesh_manifolds(
    world: &mut World,
    worker_index: i32,
    contact_id: i32,
    shape_a: &Shape,
    material_map: Option<&[u16]>,
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

    let transform_a_to_b = inv_mul_world_transforms(xf_b, xf_a);
    let enable_speculative =
        (world.contacts[contact_id as usize].flags & contact_flags::ENABLE_SPECULATIVE_POINTS) != 0;
    let handle_ghosts = !FORCE_GHOST_COLLISIONS;

    let triangle_shape = TriangleShape::from_geometry(&shape_a.geometry)
        .expect("mesh contact requires a mesh or height-field shape");
    let Some(convex_shape) = ConvexShape::from_geometry(&shape_b.geometry) else {
        debug_assert!(false, "mesh contact expects sphere/capsule/hull B");
        return false;
    };

    // The triangle caches update in place, as in C.
    let mut triangle_caches = match &mut world.contacts[contact_id as usize].geometry {
        ContactGeometry::Mesh(m) => core::mem::take(&mut m.triangle_cache),
        ContactGeometry::Convex(_) => Vec::new(),
    };

    let MeshClusters {
        clusters,
        points: cluster_points,
    } = collide_mesh_triangles(
        &triangle_shape,
        &convex_shape,
        transform_a_to_b,
        &mut triangle_caches,
        enable_speculative,
        handle_ghosts,
        is_fast,
        Some(&mut world.task_contexts[worker_index as usize]),
    );

    if let ContactGeometry::Mesh(m) = &mut world.contacts[contact_id as usize].geometry {
        m.triangle_cache = triangle_caches;
    }

    let cluster_count = clusters.len();
    if cluster_count == 0 {
        world.contacts[contact_id as usize].manifolds.clear();
        return false;
    }

    let rest_offset = mesh_rest_offset();

    // Make a temporary copy of previous manifolds
    let mut old_manifolds = world.contacts[contact_id as usize].manifolds.clone();
    let old_manifold_count = old_manifolds.len() as i32;

    {
        // Resize manifolds if needed, otherwise mem zero them.
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

    let normal_match_tolerance = 0.995f32;
    for (i, cm) in clusters.iter().enumerate() {
        let point_count = cm.point_count;
        debug_assert!(0 < point_count && (point_count as usize) <= MAX_MANIFOLD_POINTS);

        let manifold = &mut world.contacts[contact_id as usize].manifolds[i];
        manifold.point_count = point_count;
        manifold.normal = mul_mv(matrix_b, cm.manifold_normal);

        let cluster_normal = mul_mv(matrix_b, cm.manifold_normal);
        let mut best_dot = normal_match_tolerance;
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

        let mut matched_manifold = None;
        if best_index != NULL_INDEX {
            let matched = &old_manifolds[best_index as usize];
            manifold.friction_impulse = matched.friction_impulse;
            manifold.rolling_impulse = matched.rolling_impulse;
            manifold.twist_impulse = matched.twist_impulse;
            consumed[best_index as usize] = true;
            matched_manifold = Some(best_index as usize);
        }

        for j in 0..point_count as usize {
            let source = &cluster_points[cm.point_start + j];
            let target = &mut manifold.points[j];

            // Contact points are computed in frame B
            target.anchor_b = mul_mv(matrix_b, source.point);
            target.anchor_a = add(target.anchor_b, offset_a);
            target.separation = source.separation - rest_offset;
            target.feature_id = make_feature_id(source.pair);
            target.triangle_index = source.triangle_index;
            target.triangle_feature = source.triangle_feature;
            target.mesh_flags = source.mesh_flags;

            // Preserve normal impulse if possible
            if let Some(matched) = matched_manifold {
                let matched_manifold = &mut old_manifolds[matched];
                let old_point_count = matched_manifold.point_count as usize;
                for old_pt in &mut matched_manifold.points[..old_point_count] {
                    if target.feature_id == old_pt.feature_id
                        && target.triangle_index == old_pt.triangle_index
                    {
                        target.normal_impulse = old_pt.normal_impulse;
                        target.persisted = true;

                        // claimed
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
    material_map: Option<&[u16]>,
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
                            mi = map[mi as usize] as i32;
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

/// Mesh collision testing interface. Collides the shape with the mesh triangles overlapping
/// its bounds and writes up to `manifolds.len()` manifolds in the mesh frame, with the
/// normal pointing from the mesh to the shape. Returns the manifold count.
/// (b3CollideMeshAndShape)
fn collide_mesh_and_shape(
    manifolds: &mut [Manifold],
    mesh: &Mesh<'_>,
    shape_b: &ConvexShape<'_>,
    local_bounds: Aabb,
    transform_b_to_a: Transform,
    enable_speculative: bool,
    handle_ghosts: bool,
) -> i32 {
    let capacity = manifolds.len() as i32;
    if capacity <= 0 {
        return 0;
    }

    let mesh_shape = TriangleShape::Mesh(*mesh);

    let margin = speculative_distance();
    let extension = Vec3 {
        x: margin,
        y: margin,
        z: margin,
    };
    let shape_bounds = aabb_transform(transform_b_to_a, local_bounds);
    let query_bounds = Aabb {
        lower_bound: sub(shape_bounds.lower_bound, extension),
        upper_bound: add(shape_bounds.upper_bound, extension),
    };

    let triangle_indices = collect_triangle_indices(&mesh_shape, query_bounds);

    let mut triangle_caches: Vec<TriangleCache> = triangle_indices
        .iter()
        .map(|&triangle_index| TriangleCache {
            triangle_index,
            ..Default::default()
        })
        .collect();

    let transform_a_to_b = invert_transform(transform_b_to_a);

    let MeshClusters { clusters, points } = collide_mesh_triangles(
        &mesh_shape,
        shape_b,
        transform_a_to_b,
        &mut triangle_caches,
        enable_speculative,
        handle_ghosts,
        false,
        None,
    );

    let count = min_int(clusters.len() as i32, capacity);
    let matrix_b = make_matrix_from_quat(transform_b_to_a.q);
    let rest_offset = mesh_rest_offset();

    for i in 0..count as usize {
        let cluster = &clusters[i];
        let manifold = &mut manifolds[i];
        *manifold = Manifold::default();
        manifold.normal = mul_mv(matrix_b, cluster.manifold_normal);
        manifold.point_count = cluster.point_count;

        for j in 0..cluster.point_count as usize {
            let source = &points[cluster.point_start + j];
            let target = &mut manifold.points[j];
            target.anchor_b = mul_mv(matrix_b, source.point);
            target.anchor_a = add(target.anchor_b, transform_b_to_a.p);
            target.separation = source.separation - rest_offset;
            target.feature_id = make_feature_id(source.pair);
            target.triangle_index = source.triangle_index;
            target.triangle_feature = source.triangle_feature;
            target.mesh_flags = source.mesh_flags;
        }
    }

    count
}

/// Collide a mesh and hull for testing. The hull is in the mesh frame by `transform_b_to_a`.
/// Manifolds are written to `manifolds` (its length is the capacity) in the mesh frame, with
/// the normal pointing from the mesh to the hull. Returns the manifold count.
/// (b3CollideMeshAndHull)
pub fn collide_mesh_and_hull(
    manifolds: &mut [Manifold],
    mesh: &Mesh<'_>,
    hull: &HullData,
    transform_b_to_a: Transform,
    enable_speculative: bool,
    handle_ghosts: bool,
) -> i32 {
    collide_mesh_and_shape(
        manifolds,
        mesh,
        &ConvexShape::Hull(hull),
        hull.aabb,
        transform_b_to_a,
        enable_speculative,
        handle_ghosts,
    )
}

/// Collide a mesh and sphere for testing. Same conventions as [`collide_mesh_and_hull`].
/// (b3CollideMeshAndSphere)
pub fn collide_mesh_and_sphere(
    manifolds: &mut [Manifold],
    mesh: &Mesh<'_>,
    sphere: &Sphere,
    transform_b_to_a: Transform,
    enable_speculative: bool,
    handle_ghosts: bool,
) -> i32 {
    let extent = Vec3 {
        x: sphere.radius,
        y: sphere.radius,
        z: sphere.radius,
    };
    let bounds = Aabb {
        lower_bound: sub(sphere.center, extent),
        upper_bound: add(sphere.center, extent),
    };
    collide_mesh_and_shape(
        manifolds,
        mesh,
        &ConvexShape::Sphere(sphere),
        bounds,
        transform_b_to_a,
        enable_speculative,
        handle_ghosts,
    )
}
