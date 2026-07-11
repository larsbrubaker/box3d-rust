//! Port of `test/test_large_world.c`: stack settle, bullet CCD, and origin-relative
//! queries must behave at the origin and (under `double-precision`) at 1e7 m.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{body_get_position, body_is_awake, create_body};
use crate::distance::make_proxy;
use crate::geometry::{Capsule, Sphere};
use crate::hull::{make_box_hull, make_cube_hull};
use crate::id::{BodyId, ShapeId};
use crate::math_functions::{offset_pos, sub_pos, Pos, Vec3, VEC3_ZERO};
use crate::shape::{create_hull_shape, create_sphere_shape, shape_ray_cast};
use crate::types::{
    default_body_def, default_query_filter, default_shape_def, default_world_def, BodyType,
};
use crate::world::{
    world_cast_mover, world_cast_ray_closest, world_cast_shape, world_collide_mover,
    world_overlap_shape, World,
};

const STACK_COUNT: usize = 6;
const MAX_STEPS: i32 = 400;

fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

struct StackResult {
    relative_positions: [Vec3; STACK_COUNT],
    sleep_step: i32,
}

/// Drop a short stack of boxes onto a ground box centered at `base_x`.
/// (static RunStack)
fn run_stack(base_x: f32) -> StackResult {
    let base = Pos {
        x: base_x as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };

    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.position = base;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(10.0, 1.0, 10.0);
    let ground_shape_def = default_shape_def();
    create_hull_shape(&mut world, ground_id, &ground_shape_def, &ground_box.base);

    let mut bodies = [BodyId::default(); STACK_COUNT];
    for i in 0..STACK_COUNT {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = offset_pos(
            base,
            Vec3 {
                x: 0.0,
                y: 2.0 + 1.05 * i as f32,
                z: 0.0,
            },
        );
        bodies[i] = create_body(&mut world, &body_def);

        let box_hull = make_cube_hull(0.5);
        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        create_hull_shape(&mut world, bodies[i], &shape_def, &box_hull.base);
    }

    let mut result = StackResult {
        relative_positions: [VEC3_ZERO; STACK_COUNT],
        sleep_step: -1,
    };

    for step in 0..MAX_STEPS {
        world.step(1.0 / 60.0, 4);
        if result.sleep_step < 0 && !body_is_awake(&world, bodies[STACK_COUNT - 1]) {
            result.sleep_step = step;
        }
    }

    for i in 0..STACK_COUNT {
        let p = body_get_position(&world, bodies[i]);
        result.relative_positions[i] = sub_pos(p, base);
    }

    result
}

/// Fire a fast bullet at a thin wall. Returns final x relative to the base.
/// (static RunBullet)
fn run_bullet(base_x: f32) -> f32 {
    let base = Pos {
        x: base_x as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };

    let mut world = World::new(&default_world_def());

    let mut wall_def = default_body_def();
    wall_def.type_ = BodyType::Static;
    wall_def.position = offset_pos(
        base,
        Vec3 {
            x: 5.0,
            y: 0.0,
            z: 0.0,
        },
    );
    let wall_id = create_body(&mut world, &wall_def);
    let wall_box = make_box_hull(0.05, 5.0, 5.0);
    let wall_shape_def = default_shape_def();
    create_hull_shape(&mut world, wall_id, &wall_shape_def, &wall_box.base);

    let mut bullet_def = default_body_def();
    bullet_def.type_ = BodyType::Dynamic;
    bullet_def.is_bullet = true;
    bullet_def.gravity_scale = 0.0;
    bullet_def.position = base;
    bullet_def.linear_velocity = Vec3 {
        x: 200.0,
        y: 0.0,
        z: 0.0,
    };
    let bullet_id = create_body(&mut world, &bullet_def);
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.1,
    };
    let mut bullet_shape_def = default_shape_def();
    bullet_shape_def.density = 1.0;
    create_sphere_shape(&mut world, bullet_id, &bullet_shape_def, &sphere);

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }

    sub_pos(body_get_position(&world, bullet_id), base).x
}

struct QueryResult {
    cast_hit: bool,
    cast_rel_x: f32,
    overlap_hit: bool,
    mover_fraction: f32,
    plane_count: i32,
    ray_hit: bool,
    ray_rel_x: f32,
    shape_ray_hit: bool,
    shape_ray_rel_x: f32,
}

/// Origin-relative spatial queries against a static box at `base_x`.
/// (static RunQueries)
fn run_queries(base_x: f32) -> QueryResult {
    let base = Pos {
        x: base_x as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };

    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = base;
    let body_id = create_body(&mut world, &body_def);
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    let shape_def = default_shape_def();
    let shape_id: ShapeId = create_hull_shape(&mut world, body_id, &shape_def, &box_hull.base);

    world.step(1.0 / 60.0, 1);

    let filter = default_query_filter();
    let mut result = QueryResult {
        cast_hit: false,
        cast_rel_x: 0.0,
        overlap_hit: false,
        mover_fraction: 1.0,
        plane_count: 0,
        ray_hit: false,
        ray_rel_x: 0.0,
        shape_ray_hit: false,
        shape_ray_rel_x: 0.0,
    };

    let cast_point = Vec3 {
        x: -5.0,
        y: 0.0,
        z: 0.0,
    };
    let cast_proxy = make_proxy(&[cast_point], 0.25);
    let mut cast_hit = false;
    let mut cast_rel_x = 0.0f32;
    world_cast_shape(
        &world,
        base,
        &cast_proxy,
        Vec3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        },
        &filter,
        |_shape_id, point, _normal, fraction, _mat, _tri, _child| {
            cast_hit = true;
            cast_rel_x = sub_pos(point, base).x;
            fraction
        },
    );
    result.cast_hit = cast_hit;
    result.cast_rel_x = cast_rel_x;

    let overlap_point = VEC3_ZERO;
    let overlap_proxy = make_proxy(&[overlap_point], 0.5);
    let mut overlap_hit = false;
    world_overlap_shape(&world, base, &overlap_proxy, &filter, |_| {
        overlap_hit = true;
        true
    });
    result.overlap_hit = overlap_hit;

    let mover_cast = Capsule {
        center1: Vec3 {
            x: -5.0,
            y: -0.3,
            z: 0.0,
        },
        center2: Vec3 {
            x: -5.0,
            y: 0.3,
            z: 0.0,
        },
        radius: 0.25,
    };
    result.mover_fraction = world_cast_mover(
        &world,
        base,
        &mover_cast,
        Vec3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        },
        &filter,
        None,
    );

    let mover_collide = Capsule {
        center1: Vec3 {
            x: -1.1,
            y: -0.3,
            z: 0.0,
        },
        center2: Vec3 {
            x: -1.1,
            y: 0.3,
            z: 0.0,
        },
        radius: 0.3,
    };
    let mut plane_count = 0i32;
    world_collide_mover(
        &world,
        base,
        &mover_collide,
        &filter,
        |_shape_id, planes| {
            plane_count += planes.len() as i32;
            true
        },
    );
    result.plane_count = plane_count;

    let ray_origin = offset_pos(
        base,
        Vec3 {
            x: -5.0,
            y: 0.0,
            z: 0.0,
        },
    );
    let ray = world_cast_ray_closest(
        &world,
        ray_origin,
        Vec3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        },
        &filter,
    );
    result.ray_hit = ray.hit;
    result.ray_rel_x = if ray.hit {
        sub_pos(ray.point, base).x
    } else {
        0.0
    };

    let shape_ray = shape_ray_cast(
        &world,
        shape_id,
        ray_origin,
        Vec3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        },
    );
    result.shape_ray_hit = shape_ray.hit;
    result.shape_ray_rel_x = if shape_ray.hit {
        sub_pos(shape_ray.point, base).x
    } else {
        0.0
    };

    result
}

/// Stack settle at origin; under double-precision also far away. (LargeWorldStackTest)
#[test]
fn large_world_stack() {
    let origin = run_stack(0.0);
    assert!(origin.sleep_step >= 0);

    #[cfg(feature = "double-precision")]
    {
        let far = run_stack(1.0e7);
        assert!(far.sleep_step >= 0);
        assert_eq!(far.sleep_step, origin.sleep_step);
        for i in 0..STACK_COUNT {
            ensure_small(
                far.relative_positions[i].x - origin.relative_positions[i].x,
                1.0e-3,
            );
            ensure_small(
                far.relative_positions[i].y - origin.relative_positions[i].y,
                1.0e-3,
            );
            ensure_small(
                far.relative_positions[i].z - origin.relative_positions[i].z,
                1.0e-3,
            );
        }
    }
}

/// Bullet must be caught by the wall. (LargeWorldBulletTest)
#[test]
fn large_world_bullet() {
    let origin_x = run_bullet(0.0);
    assert!(origin_x < 5.0);

    #[cfg(feature = "double-precision")]
    {
        let far_x = run_bullet(1.0e7);
        assert!(far_x < 5.0);
    }
}

/// Origin-relative queries stay precise far from the origin. (LargeWorldQueryTest)
#[test]
fn large_world_query() {
    let origin = run_queries(0.0);
    assert!(origin.cast_hit);
    assert!(origin.overlap_hit);
    assert!(origin.mover_fraction < 1.0);
    assert!(origin.plane_count > 0);
    ensure_small(origin.cast_rel_x + 1.0, 0.05);
    assert!(origin.ray_hit);
    ensure_small(origin.ray_rel_x + 1.0, 0.05);
    assert!(origin.shape_ray_hit);
    ensure_small(origin.shape_ray_rel_x + 1.0, 0.05);

    #[cfg(feature = "double-precision")]
    {
        let far = run_queries(1.0e7);
        assert!(far.cast_hit);
        assert!(far.overlap_hit);
        assert!(far.mover_fraction < 1.0);
        assert!(far.plane_count > 0);
        assert!(far.ray_hit);
        assert!(far.shape_ray_hit);

        ensure_small(far.cast_rel_x - origin.cast_rel_x, 1.0e-3);
        ensure_small(far.mover_fraction - origin.mover_fraction, 1.0e-3);
        assert_eq!(far.plane_count, origin.plane_count);
        ensure_small(far.ray_rel_x - origin.ray_rel_x, 1.0e-3);
        ensure_small(far.shape_ray_rel_x - origin.shape_ray_rel_x, 1.0e-3);
    }
}
