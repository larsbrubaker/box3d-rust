//! Body-level cast / overlap / mover query tests from test_body_query.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{
    body_cast_ray, body_cast_shape, body_collide_mover, body_overlap_shape, create_body,
    BodyPlaneResult,
};
use crate::distance::make_proxy;
use crate::geometry::{Capsule, Sphere};
use crate::hull::make_box_hull;
use crate::id::BodyId;
use crate::math_functions::{
    is_normalized, make_quat_from_axis_angle, offset_pos, to_vec3, Pos, Vec3, WorldTransform, PI,
    POS_ZERO, QUAT_IDENTITY, VEC3_ZERO,
};
use crate::shape::{create_hull_shape, create_sphere_shape, shape_is_valid};
use crate::types::{default_body_def, default_query_filter, default_shape_def, default_world_def};
use crate::world::World;

fn create_query_world() -> (World, BodyId) {
    let mut world = World::new(&default_world_def());
    let body_id = create_body(&mut world, &default_body_def());
    (world, body_id)
}

fn identity_at(x: f32, y: f32, z: f32) -> WorldTransform {
    WorldTransform {
        p: Pos {
            x: x as _,
            y: y as _,
            z: z as _,
        },
        q: QUAT_IDENTITY,
    }
}

// CastRay ----------------------------------------------------------------------------------

#[test]
fn cast_ray_hits_sphere() {
    let (mut world, body_id) = create_query_world();

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };
    create_sphere_shape(&mut world, body_id, &default_shape_def(), &sphere);

    // Body sphere at world (5,0,0), ray straight at it along +X.
    let body_transform = identity_at(5.0, 0.0, 0.0);
    let result = body_cast_ray(
        &world,
        body_id,
        POS_ZERO,
        Vec3::new(10.0, 0.0, 0.0),
        &default_query_filter(),
        1.0,
        body_transform,
    );

    assert!(result.hit);
    assert!(shape_is_valid(&world, result.shape_id));
    assert!((result.fraction - 0.4).abs() < 1e-5);
    assert!((result.normal.x + 1.0).abs() < 1e-5);
    assert!(result.normal.y.abs() < 1e-5);
    assert!(result.normal.z.abs() < 1e-5);

    let point = to_vec3(result.point);
    assert!((point.x - 4.0).abs() < 1e-4);
    assert!(point.y.abs() < 1e-4);
    assert!(point.z.abs() < 1e-4);
}

#[test]
fn cast_ray_miss() {
    let (mut world, body_id) = create_query_world();

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };
    create_sphere_shape(&mut world, body_id, &default_shape_def(), &sphere);

    // Ray runs parallel to the body, never reaching it.
    let body_transform = identity_at(5.0, 0.0, 0.0);
    let result = body_cast_ray(
        &world,
        body_id,
        POS_ZERO,
        Vec3::new(0.0, 10.0, 0.0),
        &default_query_filter(),
        1.0,
        body_transform,
    );

    assert!(!result.hit);
}

#[test]
fn cast_ray_closest_shape() {
    let (mut world, body_id) = create_query_world();

    let shape_def = default_shape_def();
    let near_sphere = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };
    let far_sphere = Sphere {
        center: Vec3::new(4.0, 0.0, 0.0),
        radius: 1.0,
    };
    let near_id = create_sphere_shape(&mut world, body_id, &shape_def, &near_sphere);
    create_sphere_shape(&mut world, body_id, &shape_def, &far_sphere);

    // Ray crosses both spheres; the loop must shrink maxFraction to the nearer hit.
    let body_transform = identity_at(0.0, 0.0, 0.0);
    let result = body_cast_ray(
        &world,
        body_id,
        Pos {
            x: (-5.0) as _,
            y: 0.0 as _,
            z: 0.0 as _,
        },
        Vec3::new(10.0, 0.0, 0.0),
        &default_query_filter(),
        1.0,
        body_transform,
    );

    assert!(result.hit);
    assert_eq!(result.shape_id.index1, near_id.index1);
    assert_eq!(result.shape_id.generation, near_id.generation);
    assert!((result.fraction - 0.4).abs() < 1e-5);
}

#[test]
fn cast_ray_rotated_body() {
    let (mut world, body_id) = create_query_world();

    // Local center (0,2,0) rotated +90 deg about Z lands at world (-2,0,0).
    let sphere = Sphere {
        center: Vec3::new(0.0, 2.0, 0.0),
        radius: 0.5,
    };
    create_sphere_shape(&mut world, body_id, &default_shape_def(), &sphere);

    let body_transform = WorldTransform {
        p: POS_ZERO,
        q: make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5 * PI),
    };
    let result = body_cast_ray(
        &world,
        body_id,
        POS_ZERO,
        Vec3::new(-4.0, 0.0, 0.0),
        &default_query_filter(),
        1.0,
        body_transform,
    );

    assert!(result.hit);
    assert!((result.fraction - 0.375).abs() < 1e-5);
    assert!((result.normal.x - 1.0).abs() < 1e-5);

    let point = to_vec3(result.point);
    assert!((point.x + 1.5).abs() < 1e-4);
}

#[test]
fn cast_ray_far_from_origin() {
    let (mut world, body_id) = create_query_world();

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };
    create_sphere_shape(&mut world, body_id, &default_shape_def(), &sphere);

    // Same geometry as CastRayHitsSphere shifted far from the world origin.
    let origin = Pos {
        x: 1.0e6 as _,
        y: (-2.0e6) as _,
        z: 5.0e5 as _,
    };
    let body_transform = WorldTransform {
        p: offset_pos(origin, Vec3::new(5.0, 0.0, 0.0)),
        q: QUAT_IDENTITY,
    };
    let result = body_cast_ray(
        &world,
        body_id,
        origin,
        Vec3::new(10.0, 0.0, 0.0),
        &default_query_filter(),
        1.0,
        body_transform,
    );

    assert!(result.hit);
    assert!((result.fraction - 0.4).abs() < 1e-5);
    assert!((result.normal.x + 1.0).abs() < 1e-5);
    assert!(result.normal.y.abs() < 1e-5);
    assert!(result.normal.z.abs() < 1e-5);
}

// CastShape --------------------------------------------------------------------------------

#[test]
fn cast_shape_hits_box() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    // Sphere proxy of radius 0.5 cast along +X into a box whose front face is at world x = 4.
    let proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let body_transform = identity_at(5.0, 0.0, 0.0);
    let result = body_cast_shape(
        &world,
        body_id,
        POS_ZERO,
        &proxy,
        Vec3::new(10.0, 0.0, 0.0),
        &default_query_filter(),
        1.0,
        false,
        body_transform,
    );

    // Front face at world x = 4. The fraction carries a small shape-cast skin.
    assert!(result.hit);
    assert!(shape_is_valid(&world, result.shape_id));
    assert!((result.fraction - 0.35).abs() < 1e-2);
    assert!((result.normal.x + 1.0).abs() < 1e-4);

    let hit = to_vec3(result.point);
    assert!((hit.x - 4.0).abs() < 1e-3);
}

#[test]
fn cast_shape_miss() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    let proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let body_transform = identity_at(5.0, 0.0, 0.0);
    let result = body_cast_shape(
        &world,
        body_id,
        POS_ZERO,
        &proxy,
        Vec3::new(0.0, 10.0, 0.0),
        &default_query_filter(),
        1.0,
        false,
        body_transform,
    );

    assert!(!result.hit);
}

#[test]
fn cast_shape_rotated_body() {
    let (mut world, body_id) = create_query_world();

    // Body sphere local center (0,2,0) rotated +90 deg about Z lands at world (-2,0,0).
    let sphere = Sphere {
        center: Vec3::new(0.0, 2.0, 0.0),
        radius: 1.0,
    };
    create_sphere_shape(&mut world, body_id, &default_shape_def(), &sphere);

    let proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let body_transform = WorldTransform {
        p: POS_ZERO,
        q: make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5 * PI),
    };
    let result = body_cast_shape(
        &world,
        body_id,
        POS_ZERO,
        &proxy,
        Vec3::new(-4.0, 0.0, 0.0),
        &default_query_filter(),
        1.0,
        false,
        body_transform,
    );

    assert!(result.hit);
    assert!((result.fraction - 0.125).abs() < 1e-2);
    assert!((result.normal.x - 1.0).abs() < 1e-4);

    let hit = to_vec3(result.point);
    assert!((hit.x + 1.0).abs() < 1e-3);
}

#[test]
fn cast_shape_far_from_origin() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    let proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let origin = Pos {
        x: 1.0e6 as _,
        y: (-2.0e6) as _,
        z: 5.0e5 as _,
    };
    let body_transform = WorldTransform {
        p: offset_pos(origin, Vec3::new(5.0, 0.0, 0.0)),
        q: QUAT_IDENTITY,
    };
    let result = body_cast_shape(
        &world,
        body_id,
        origin,
        &proxy,
        Vec3::new(10.0, 0.0, 0.0),
        &default_query_filter(),
        1.0,
        false,
        body_transform,
    );

    assert!(result.hit);
    assert!((result.fraction - 0.35).abs() < 1e-2);
    assert!((result.normal.x + 1.0).abs() < 1e-4);
}

// OverlapShape -----------------------------------------------------------------------------

#[test]
fn overlap_true() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    // Proxy sits at the box center.
    let proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let body_transform = identity_at(5.0, 0.0, 0.0);
    let overlaps = body_overlap_shape(
        &world,
        body_id,
        Pos {
            x: 5.0 as _,
            y: 0.0 as _,
            z: 0.0 as _,
        },
        &proxy,
        &default_query_filter(),
        body_transform,
    );

    assert!(overlaps);
}

#[test]
fn overlap_false() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    let proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let body_transform = identity_at(5.0, 0.0, 0.0);
    let overlaps = body_overlap_shape(
        &world,
        body_id,
        Pos {
            x: 20.0 as _,
            y: 0.0 as _,
            z: 0.0 as _,
        },
        &proxy,
        &default_query_filter(),
        body_transform,
    );

    assert!(!overlaps);
}

#[test]
fn overlap_respects_body_transform() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    // Fixed proxy and origin: only the supplied transform decides the overlap.
    let proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let origin = POS_ZERO;

    assert!(body_overlap_shape(
        &world,
        body_id,
        origin,
        &proxy,
        &default_query_filter(),
        identity_at(0.0, 0.0, 0.0),
    ));
    assert!(!body_overlap_shape(
        &world,
        body_id,
        origin,
        &proxy,
        &default_query_filter(),
        identity_at(20.0, 0.0, 0.0),
    ));
}

#[test]
fn overlap_filter() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    let proxy = make_proxy(&[VEC3_ZERO], 0.5);
    let body_transform = identity_at(0.0, 0.0, 0.0);

    // Geometry overlaps, but a zero mask rejects every category.
    let mut filter = default_query_filter();
    filter.mask_bits = 0;
    let overlaps = body_overlap_shape(&world, body_id, POS_ZERO, &proxy, &filter, body_transform);

    assert!(!overlaps);
}

// CollideMover -----------------------------------------------------------------------------

#[test]
fn mover_touches_box() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    // Mover core runs above the +Y face; its 0.2 radius reaches 0.1 into it.
    let mover = Capsule {
        center1: Vec3::new(-0.3, 0.6, 0.0),
        center2: Vec3::new(0.3, 0.6, 0.0),
        radius: 0.2,
    };
    let mut planes = [BodyPlaneResult::default(); 4];
    let body_transform = identity_at(0.0, 0.0, 0.0);
    let count = body_collide_mover(
        &world,
        body_id,
        &mut planes,
        POS_ZERO,
        &mover,
        &default_query_filter(),
        body_transform,
    );

    assert_eq!(count, 1);
    assert!(shape_is_valid(&world, planes[0].shape_id));
    assert!(is_normalized(planes[0].result.plane.normal));
    assert!(planes[0].result.plane.normal.y > 0.99);
    assert!((planes[0].result.plane.offset - 0.1).abs() < 1e-4);
}

#[test]
fn mover_separated() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    let mover = Capsule {
        center1: Vec3::new(-0.3, 5.0, 0.0),
        center2: Vec3::new(0.3, 5.0, 0.0),
        radius: 0.2,
    };
    let mut planes = [BodyPlaneResult::default(); 4];
    let body_transform = identity_at(0.0, 0.0, 0.0);
    let count = body_collide_mover(
        &world,
        body_id,
        &mut planes,
        POS_ZERO,
        &mover,
        &default_query_filter(),
        body_transform,
    );

    assert_eq!(count, 0);
}

#[test]
fn mover_rotated_body() {
    let (mut world, body_id) = create_query_world();

    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);

    // Rotating +90 deg about X turns the local +Y face toward world +Z.
    let mover = Capsule {
        center1: Vec3::new(-0.3, 0.0, 0.6),
        center2: Vec3::new(0.3, 0.0, 0.6),
        radius: 0.2,
    };
    let mut planes = [BodyPlaneResult::default(); 4];
    let body_transform = WorldTransform {
        p: POS_ZERO,
        q: make_quat_from_axis_angle(Vec3::new(1.0, 0.0, 0.0), 0.5 * PI),
    };
    let count = body_collide_mover(
        &world,
        body_id,
        &mut planes,
        POS_ZERO,
        &mover,
        &default_query_filter(),
        body_transform,
    );

    assert_eq!(count, 1);
    assert!(is_normalized(planes[0].result.plane.normal));
    assert!(planes[0].result.plane.normal.z > 0.99);
    assert!((planes[0].result.plane.offset - 0.1).abs() < 1e-4);
}

#[test]
fn mover_capacity() {
    let (mut world, body_id) = create_query_world();

    // Two spheres each touch a mover that runs between them along X at y = 0.
    let shape_def = default_shape_def();
    let left = Sphere {
        center: Vec3::new(-0.4, 0.6, 0.0),
        radius: 0.5,
    };
    let right = Sphere {
        center: Vec3::new(0.4, 0.6, 0.0),
        radius: 0.5,
    };
    create_sphere_shape(&mut world, body_id, &shape_def, &left);
    create_sphere_shape(&mut world, body_id, &shape_def, &right);

    let mover = Capsule {
        center1: Vec3::new(-1.0, 0.0, 0.0),
        center2: Vec3::new(1.0, 0.0, 0.0),
        radius: 0.2,
    };
    let mut planes = [BodyPlaneResult::default(); 4];
    let body_transform = identity_at(0.0, 0.0, 0.0);

    // Capacity caps the result and prevents writing past the buffer.
    let capped = body_collide_mover(
        &world,
        body_id,
        &mut planes[..1],
        POS_ZERO,
        &mover,
        &default_query_filter(),
        body_transform,
    );
    assert_eq!(capped, 1);

    let full = body_collide_mover(
        &world,
        body_id,
        &mut planes,
        POS_ZERO,
        &mover,
        &default_query_filter(),
        body_transform,
    );
    assert_eq!(full, 2);
}
