//! Focused unit tests for the convex primitive manifold slice.
//!
//! Upstream C has almost no dedicated collide unit tests (samples + determinism
//! cover them). These mirror the invent-focused style of box2d-rust's
//! `manifold_tests.rs`: overlap, separation, normals, and point counts.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::MAX_MANIFOLD_POINTS;
use crate::distance::SimplexCache;
use crate::geometry::{Capsule, Sphere};
use crate::hull::make_box_hull;
use crate::manifold::{
    collide_capsule_and_sphere, collide_capsule_and_triangle, collide_capsules,
    collide_hull_and_capsule, collide_hull_and_sphere, collide_hull_and_triangle, collide_hulls,
    collide_sphere_and_triangle, collide_spheres, LocalManifold, SatCache, SeparatingFeature,
    TriangleFeature,
};
use crate::math_functions::{
    inv_mul_world_transforms, offset_pos, Transform, Vec3, WorldTransform, POS_ZERO, QUAT_IDENTITY,
    TRANSFORM_IDENTITY, VEC3_ZERO, WORLD_TRANSFORM_IDENTITY,
};

fn ensure_small(value: f32, tolerance: f32) {
    // Matches the C ENSURE_SMALL macro, which is inclusive: pass when
    // -tol <= value <= tol.
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn xf_at(p: Vec3) -> Transform {
    Transform {
        p,
        q: QUAT_IDENTITY,
    }
}

#[test]
fn collide_spheres_overlap_and_separation() {
    let a = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };
    let b = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };

    // Centers 1.5 apart → overlap of 0.5 along +x.
    let mut manifold = LocalManifold::default();
    collide_spheres(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf_at(v(1.5, 0.0, 0.0)),
    );
    assert_eq!(manifold.point_count, 1);
    ensure_small(manifold.normal.x - 1.0, f32::EPSILON);
    ensure_small(manifold.normal.y, f32::EPSILON);
    ensure_small(manifold.normal.z, f32::EPSILON);
    ensure_small(manifold.points[0].separation + 0.5, 1e-6);
    // Midpoint: 0.5 * ((0 + 1*n) + 1.5 - 1*n) = 0.75 on x
    ensure_small(manifold.points[0].point.x - 0.75, 1e-6);

    // Far apart: no contact (point_count left at 0 from default).
    let mut far = LocalManifold::default();
    collide_spheres(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf_at(v(10.0, 0.0, 0.0)),
    );
    assert_eq!(far.point_count, 0);

    // Coincident centers: fallback normal (0,1,0), separation = -2.
    let mut coincident = LocalManifold::default();
    collide_spheres(
        &mut coincident,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        TRANSFORM_IDENTITY,
    );
    assert_eq!(coincident.point_count, 1);
    ensure_small(coincident.normal.x, f32::EPSILON);
    ensure_small(coincident.normal.y - 1.0, f32::EPSILON);
    ensure_small(coincident.normal.z, f32::EPSILON);
    ensure_small(coincident.points[0].separation + 2.0, 1e-6);
}

#[test]
fn collide_capsule_and_sphere_overlap_and_separation() {
    let capsule = Capsule {
        center1: v(-1.0, 0.0, 0.0),
        center2: v(1.0, 0.0, 0.0),
        radius: 0.5,
    };
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };

    // Sphere above capsule mid-segment, centers 0.8 apart → overlap 0.2.
    let mut manifold = LocalManifold::default();
    collide_capsule_and_sphere(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &capsule,
        &sphere,
        xf_at(v(0.0, 0.8, 0.0)),
    );
    assert_eq!(manifold.point_count, 1);
    ensure_small(manifold.normal.x, 1e-6);
    ensure_small(manifold.normal.y - 1.0, 1e-6);
    ensure_small(manifold.normal.z, 1e-6);
    ensure_small(manifold.points[0].separation + 0.2, 1e-5);

    // Far above: separated.
    let mut far = LocalManifold::default();
    collide_capsule_and_sphere(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &capsule,
        &sphere,
        xf_at(v(0.0, 5.0, 0.0)),
    );
    assert_eq!(far.point_count, 0);

    // Sphere beyond an end-cap (along +x past center2).
    let mut end = LocalManifold::default();
    collide_capsule_and_sphere(
        &mut end,
        MAX_MANIFOLD_POINTS as i32,
        &capsule,
        &sphere,
        xf_at(v(1.8, 0.0, 0.0)),
    );
    assert_eq!(end.point_count, 1);
    ensure_small(end.normal.x - 1.0, 1e-5);
    ensure_small(end.points[0].separation + 0.2, 1e-5);
}

#[test]
fn collide_capsules_parallel_and_skew() {
    // Two horizontal capsules stacked with overlap → two contact points.
    let a = Capsule {
        center1: v(-1.0, 0.0, 0.0),
        center2: v(1.0, 0.0, 0.0),
        radius: 0.5,
    };
    let b = Capsule {
        center1: v(-1.0, 0.0, 0.0),
        center2: v(1.0, 0.0, 0.0),
        radius: 0.5,
    };

    let mut manifold = LocalManifold::default();
    collide_capsules(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf_at(v(0.0, 0.9, 0.0)),
    );
    assert_eq!(manifold.point_count, 2);
    ensure_small(manifold.normal.x, 1e-5);
    ensure_small(manifold.normal.y - 1.0, 1e-5);
    ensure_small(manifold.normal.z, 1e-5);
    ensure_small(manifold.points[0].separation + 0.1, 1e-4);
    ensure_small(manifold.points[1].separation + 0.1, 1e-4);

    // Far apart.
    let mut far = LocalManifold::default();
    collide_capsules(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf_at(v(0.0, 10.0, 0.0)),
    );
    assert_eq!(far.point_count, 0);

    // Skew (non-parallel) capsules: single closest-point contact.
    let skew_b = Capsule {
        center1: v(0.0, 0.0, -1.0),
        center2: v(0.0, 0.0, 1.0),
        radius: 0.5,
    };
    let mut skew = LocalManifold::default();
    collide_capsules(
        &mut skew,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &skew_b,
        xf_at(v(0.0, 0.8, 0.0)),
    );
    assert_eq!(skew.point_count, 1);
    ensure_small(skew.normal.y - 1.0, 1e-4);
    ensure_small(skew.points[0].separation + 0.2, 1e-4);
}

#[test]
fn collide_hull_and_sphere_via_gjk() {
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };

    // Sphere resting above the +y face with 0.05 penetration.
    // Box half-extent 1, sphere radius 0.5, center at y=1.45 → separation -0.05.
    let mut manifold = LocalManifold::default();
    let mut cache = SimplexCache::default();
    collide_hull_and_sphere(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        &sphere,
        xf_at(v(0.0, 1.45, 0.0)),
        &mut cache,
    );
    assert_eq!(manifold.point_count, 1);
    ensure_small(manifold.normal.x, 1e-5);
    ensure_small(manifold.normal.y - 1.0, 1e-5);
    ensure_small(manifold.normal.z, 1e-5);
    ensure_small(manifold.points[0].separation + 0.05, 1e-4);

    // Far above: separated, cache cleared.
    let mut far = LocalManifold::default();
    let mut far_cache = SimplexCache::default();
    // Seed cache so we can observe the clear-on-separation path.
    far_cache.count = 1;
    collide_hull_and_sphere(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        &sphere,
        xf_at(v(0.0, 10.0, 0.0)),
        &mut far_cache,
    );
    assert_eq!(far.point_count, 0);
    assert_eq!(far_cache.count, 0);

    // Deep penetration: sphere center inside the box → face-support path.
    let mut deep = LocalManifold::default();
    let mut deep_cache = SimplexCache::default();
    collide_hull_and_sphere(
        &mut deep,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        &sphere,
        xf_at(v(0.0, 0.25, 0.0)),
        &mut deep_cache,
    );
    assert_eq!(deep.point_count, 1);
    // Closest face is +y: plane sep of center = -0.75, then - radius → -1.25.
    ensure_small(deep.normal.y - 1.0, 1e-5);
    ensure_small(deep.points[0].separation + 1.25, 1e-4);
}

#[test]
fn collide_hulls_face_overlap_and_separation() {
    let box_a = make_box_hull(0.5, 0.5, 0.5);
    let box_b = make_box_hull(0.5, 0.5, 0.5);

    // Centers 0.9 apart → cubes overlap by 0.1 along +x; face clip yields 4 points.
    let mut manifold = LocalManifold::default();
    let mut cache = SatCache::default();
    collide_hulls(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &box_a.base,
        &box_b.base,
        xf_at(v(0.9, 0.0, 0.0)),
        &mut cache,
    );
    assert_eq!(manifold.point_count, 4);
    ensure_small(manifold.normal.x - 1.0, 1e-4);
    for i in 0..manifold.point_count {
        ensure_small(manifold.points[i as usize].separation + 0.1, 0.01);
    }

    // Far apart: no contact, cache stores separating face.
    let mut far = LocalManifold::default();
    let mut far_cache = SatCache::default();
    collide_hulls(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &box_a.base,
        &box_b.base,
        xf_at(v(10.0, 0.0, 0.0)),
        &mut far_cache,
    );
    assert_eq!(far.point_count, 0);
    assert!(far_cache.separation > 0.0);
}

#[test]
fn collide_hull_and_capsule_overlap_and_separation() {
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let capsule = Capsule {
        center1: v(0.0, -0.5, 0.0),
        center2: v(0.0, 0.5, 0.0),
        radius: 0.25,
    };

    // Capsule axis along y, shifted along +x so the segment is outside the box
    // but the radius overlaps the +x face.
    // Box face at x=0.5, capsule centers at x=0.65 → plane sep 0.15, minus radius 0.25 → -0.1.
    let mut manifold = LocalManifold::default();
    let mut cache = SimplexCache::default();
    collide_hull_and_capsule(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        &capsule,
        xf_at(v(0.65, 0.0, 0.0)),
        &mut cache,
    );
    assert!(manifold.point_count >= 1);
    ensure_small(manifold.normal.x - 1.0, 1e-3);
    for i in 0..manifold.point_count {
        ensure_small(manifold.points[i as usize].separation + 0.1, 0.05);
    }

    // Far apart.
    let mut far = LocalManifold::default();
    let mut far_cache = SimplexCache::default();
    far_cache.count = 1;
    collide_hull_and_capsule(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        &capsule,
        xf_at(v(10.0, 0.0, 0.0)),
        &mut far_cache,
    );
    assert_eq!(far.point_count, 0);
    assert_eq!(far_cache.count, 0);
}

/// Port of `LargeWorldManifoldTest` from `test_collision.c`.
///
/// Origin configuration always runs; the far-from-origin comparison is gated on
/// `double-precision` (mirrors `BOX3D_DOUBLE_PRECISION`).
#[test]
fn large_world_manifold_test() {
    let box_a = make_box_hull(0.5, 0.5, 0.5);
    let box_b = make_box_hull(0.5, 0.5, 0.5);

    // Centers 0.9 apart so the cubes overlap by 0.1 along x
    let sep = v(0.9, 0.0, 0.0);

    let mut m_origin = LocalManifold::default();
    let xf_ao = WORLD_TRANSFORM_IDENTITY;
    let xf_bo = WorldTransform {
        p: offset_pos(POS_ZERO, sep),
        q: QUAT_IDENTITY,
    };
    let mut cache_origin = SatCache::default();
    collide_hulls(
        &mut m_origin,
        8,
        &box_a.base,
        &box_b.base,
        inv_mul_world_transforms(xf_ao, xf_bo),
        &mut cache_origin,
    );

    // Two cube faces overlap, so the clipped manifold has four points
    assert_eq!(m_origin.point_count, 4);
    for i in 0..m_origin.point_count {
        ensure_small(m_origin.points[i as usize].separation + 0.1, 0.01);
    }

    #[cfg(feature = "double-precision")]
    {
        // Same relative configuration shifted far from the origin. The relative pose
        // differences the world positions in double, so in double the frame A manifold
        // is preserved to float precision.
        let base = offset_pos(POS_ZERO, v(1.0e7, 1.0e7, 1.0e7));

        let mut m_large = LocalManifold::default();
        let xf_al = WorldTransform {
            p: base,
            q: QUAT_IDENTITY,
        };
        let xf_bl = WorldTransform {
            p: offset_pos(base, sep),
            q: QUAT_IDENTITY,
        };
        let mut cache_large = SatCache::default();
        collide_hulls(
            &mut m_large,
            8,
            &box_a.base,
            &box_b.base,
            inv_mul_world_transforms(xf_al, xf_bl),
            &mut cache_large,
        );

        assert_eq!(m_large.point_count, m_origin.point_count);
        ensure_small(m_large.normal.x - m_origin.normal.x, 1e-4);
        ensure_small(m_large.normal.y - m_origin.normal.y, 1e-4);
        ensure_small(m_large.normal.z - m_origin.normal.z, 1e-4);
        for i in 0..m_large.point_count {
            let i = i as usize;
            ensure_small(
                m_large.points[i].separation - m_origin.points[i].separation,
                1e-4,
            );
            ensure_small(m_large.points[i].point.x - m_origin.points[i].point.x, 1e-4);
            ensure_small(m_large.points[i].point.y - m_origin.points[i].point.y, 1e-4);
            ensure_small(m_large.points[i].point.z - m_origin.points[i].point.z, 1e-4);
        }
    }
}

fn unit_triangle() -> [Vec3; 3] {
    // CCW when viewed from +y: Edge1 is the z=-1 edge, normal points +y
    [v(1.0, 0.0, -1.0), v(-1.0, 0.0, -1.0), v(0.0, 0.0, 1.0)]
}

#[test]
fn collide_sphere_and_triangle_face_edge_vertex() {
    let tri = unit_triangle();

    // Sphere above face center → face feature, normal +y.
    let sphere_face = Sphere {
        center: v(0.0, 0.4, 0.0),
        radius: 0.5,
    };
    let mut face = LocalManifold::default();
    collide_sphere_and_triangle(&mut face, MAX_MANIFOLD_POINTS as i32, &sphere_face, &tri);
    assert_eq!(face.point_count, 1);
    assert_eq!(face.feature, TriangleFeature::TriangleFace);
    ensure_small(face.normal.x, 1e-5);
    ensure_small(face.normal.y - 1.0, 1e-5);
    ensure_small(face.normal.z, 1e-5);
    ensure_small(face.points[0].separation + 0.1, 1e-4);

    // Sphere outside edge v1-v2 (along -z past the edge at z=-1).
    let sphere_edge = Sphere {
        center: v(0.0, 0.0, -1.4),
        radius: 0.5,
    };
    let mut edge = LocalManifold::default();
    collide_sphere_and_triangle(&mut edge, MAX_MANIFOLD_POINTS as i32, &sphere_edge, &tri);
    assert_eq!(edge.point_count, 1);
    assert_eq!(edge.feature, TriangleFeature::Edge1);
    ensure_small(edge.points[0].separation + 0.1, 1e-4);

    // Sphere near vertex v3 (0,0,1).
    let sphere_vert = Sphere {
        center: v(0.0, 0.0, 1.4),
        radius: 0.5,
    };
    let mut vert = LocalManifold::default();
    collide_sphere_and_triangle(&mut vert, MAX_MANIFOLD_POINTS as i32, &sphere_vert, &tri);
    assert_eq!(vert.point_count, 1);
    assert_eq!(vert.feature, TriangleFeature::Vertex3);
    ensure_small(vert.points[0].separation + 0.1, 1e-4);
}

#[test]
fn collide_sphere_and_triangle_backface_cull() {
    let tri = unit_triangle();
    // Sphere below the triangle (negative plane offset) must be culled.
    let sphere = Sphere {
        center: v(0.0, -0.2, 0.0),
        radius: 0.5,
    };
    let mut manifold = LocalManifold::default();
    collide_sphere_and_triangle(&mut manifold, MAX_MANIFOLD_POINTS as i32, &sphere, &tri);
    assert_eq!(manifold.point_count, 0);
}

#[test]
fn collide_capsule_and_triangle_face_vs_edge() {
    let tri = unit_triangle();

    // Capsule parallel to the face, above it → face contact with 2 points.
    let capsule_face = Capsule {
        center1: v(-0.3, 0.4, 0.0),
        center2: v(0.3, 0.4, 0.0),
        radius: 0.5,
    };
    let mut face = LocalManifold::default();
    let mut cache = SimplexCache::default();
    collide_capsule_and_triangle(
        &mut face,
        MAX_MANIFOLD_POINTS as i32,
        &capsule_face,
        &tri,
        &mut cache,
    );
    assert_eq!(face.point_count, 2);
    assert_eq!(face.feature, TriangleFeature::TriangleFace);
    ensure_small(face.normal.y - 1.0, 1e-4);
    ensure_small(face.points[0].separation + 0.1, 1e-3);
    ensure_small(face.points[1].separation + 0.1, 1e-3);

    // Capsule beside edge v1-v2 (z=-1), axis parallel to the edge → edge/closest contact.
    let capsule_edge = Capsule {
        center1: v(-0.3, 0.0, -1.4),
        center2: v(0.3, 0.0, -1.4),
        radius: 0.5,
    };
    let mut edge = LocalManifold::default();
    let mut edge_cache = SimplexCache::default();
    collide_capsule_and_triangle(
        &mut edge,
        MAX_MANIFOLD_POINTS as i32,
        &capsule_edge,
        &tri,
        &mut edge_cache,
    );
    assert!(edge.point_count >= 1);
    // Shallow path: feature from GJK simplex (edge1) or face clip.
    assert!(
        edge.feature == TriangleFeature::Edge1
            || edge.feature == TriangleFeature::TriangleFace
            || edge.point_count == 1
    );
    ensure_small(edge.points[0].separation + 0.1, 0.05);
}

#[test]
fn collide_hull_and_triangle_face_overlap_and_separation() {
    let box_hull = make_box_hull(0.5, 0.5, 0.5);

    // Box at origin; triangle at y=-0.4 so hull bottom (y=-0.5) penetrates by 0.1.
    // CCW from +y so the triangle normal points toward the hull.
    let mut manifold = LocalManifold::default();
    let mut cache = SatCache::default();
    collide_hull_and_triangle(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        v(2.0, -0.4, -2.0),
        v(-2.0, -0.4, -2.0),
        v(0.0, -0.4, 2.0),
        0,
        &mut cache,
        true,
    );
    assert!(manifold.point_count >= 1);
    // Normal should point roughly +y (triangle face) or from hull toward triangle.
    assert!(manifold.normal.y > 0.5);
    for i in 0..manifold.point_count {
        ensure_small(manifold.points[i as usize].separation + 0.1, 0.05);
    }

    // Far below: separated.
    let mut far = LocalManifold::default();
    let mut far_cache = SatCache::default();
    collide_hull_and_triangle(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        v(2.0, -10.0, -2.0),
        v(-2.0, -10.0, -2.0),
        v(0.0, -10.0, 2.0),
        0,
        &mut far_cache,
        true,
    );
    assert_eq!(far.point_count, 0);
    assert!(far_cache.separation > 0.0 || far_cache.type_ != 0);

    // Backface: triangle above the hull with +y normal → hull is behind the plane.
    let mut back = LocalManifold::default();
    let mut back_cache = SatCache::default();
    collide_hull_and_triangle(
        &mut back,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        v(2.0, 2.0, -2.0),
        v(-2.0, 2.0, -2.0),
        v(0.0, 2.0, 2.0),
        0,
        &mut back_cache,
        true,
    );
    assert_eq!(back.point_count, 0);
    assert_eq!(back_cache.type_, SeparatingFeature::BacksideAxis as u8);
}
