//! Continuous collision / bullet and overflow graph-color world tests.

use crate::body::{body_get_position, create_body, get_body_transform_quick};
use crate::constraint_graph::OVERFLOW_INDEX;
use crate::geometry::Sphere;
use crate::hull::{make_box_hull, make_cube_hull};
use crate::math_functions::{Pos, PI, VEC3_ZERO};
use crate::shape::{create_hull_shape, create_sphere_shape};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::World;
/// (TestOverflowColorPile) — exercises the overflow graph-color path.
#[test]
fn overflow_color_pile() {
    const RING_COUNT: i32 = 5;
    const PER_RING: i32 = 5;

    let mut world = World::new(&default_world_def());

    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0 as _,
            y: -1.0 as _,
            z: 0.0 as _,
        };
        let ground_id = create_body(&mut world, &body_def);
        let box_hull = make_box_hull(20.0, 1.0, 20.0);
        create_hull_shape(&mut world, ground_id, &default_shape_def(), &box_hull.base);
    }

    let hub_half_x = 0.5f32;
    let hub_half_y = 2.5f32;
    let hub_half_z = 0.5f32;
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 0.0 as _,
            y: hub_half_y as _,
            z: 0.0 as _,
        };
        let hub_id = create_body(&mut world, &body_def);
        let box_hull = make_box_hull(hub_half_x, hub_half_y, hub_half_z);
        let mut shape_def = default_shape_def();
        shape_def.density = 50.0;
        create_hull_shape(&mut world, hub_id, &shape_def, &box_hull.base);
    }

    let neighbor_half = 0.2f32;
    let ring_radius = hub_half_x + neighbor_half - 0.03;
    let neighbor_box = make_box_hull(neighbor_half, neighbor_half, neighbor_half);
    let neighbor_shape = default_shape_def();
    let ring_spacing = 0.5f32;
    let base_y = neighbor_half + 0.05;

    for ring in 0..RING_COUNT {
        let y = base_y + ring_spacing * ring as f32;
        let theta_offset = if (ring & 1) != 0 {
            PI / PER_RING as f32
        } else {
            0.0
        };

        for slot in 0..PER_RING {
            let theta = theta_offset + (2.0 * PI * slot as f32) / PER_RING as f32;
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = Pos {
                x: (ring_radius * theta.cos()) as _,
                y: y as _,
                z: (ring_radius * theta.sin()) as _,
            };
            let body_id = create_body(&mut world, &body_def);
            create_hull_shape(&mut world, body_id, &neighbor_shape, &neighbor_box.base);
        }
    }

    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }

    let overflow = &world.constraint_graph.colors[OVERFLOW_INDEX as usize];
    let overflow_contacts = overflow.contacts.len() + overflow.convex_contacts.len();
    assert!(
        overflow_contacts > 0,
        "expected contacts in overflow color, got 0"
    );
}

/// (HelloWorld)
#[test]
fn hello_world() {
    let mut world_def = default_world_def();
    world_def.gravity = crate::math_functions::Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    let mut world = World::new(&world_def);

    let mut ground_body_def = default_body_def();
    ground_body_def.position = Pos {
        x: 0.0 as _,
        y: -10.0 as _,
        z: 0.0 as _,
    };
    let ground_id = create_body(&mut world, &ground_body_def);

    let ground_box = make_box_hull(50.0, 10.0, 50.0);
    let ground_shape_def = default_shape_def();
    create_hull_shape(&mut world, ground_id, &ground_shape_def, &ground_box.base);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 4.0 as _,
        z: 0.0 as _,
    };
    let body_id = create_body(&mut world, &body_def);

    let dynamic_box = make_cube_hull(1.0);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.base_material.friction = 0.3;
    create_hull_shape(&mut world, body_id, &shape_def, &dynamic_box.base);

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;

    for _ in 0..90 {
        world.step(time_step, sub_step_count);
    }

    let position = body_get_position(&world, body_id);
    let body_index = crate::body::get_body_full_id(&world, body_id);
    let rotation = get_body_transform_quick(&world, &world.bodies[body_index as usize]).q;

    assert!(
        (position.y as f32 - 1.0).abs() < 0.01,
        "expected y ≈ 1.0, got {}",
        position.y
    );
    assert!(
        rotation.v.x.abs() < 0.01,
        "expected rotation.v.x ≈ 0, got {}",
        rotation.v.x
    );
    assert!(
        rotation.v.z.abs() < 0.01,
        "expected rotation.v.z ≈ 0, got {}",
        rotation.v.z
    );
}

/// SetBullet must sync body and bodySim flags so SetMotionLocks cannot wipe
/// or revive the bullet bit. (SetBulletDriftTest)
#[test]
fn set_bullet_drift_test() {
    use crate::body::{body_is_bullet, body_set_bullet, body_set_motion_locks};
    use crate::types::MotionLocks;

    let mut world = World::new(&default_world_def());

    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = false;
        let body_id = create_body(&mut world, &body_def);

        assert!(!body_is_bullet(&world, body_id));

        body_set_bullet(&mut world, body_id, true);
        assert!(body_is_bullet(&world, body_id));

        let mut locks = MotionLocks::default();
        locks.linear_x = true;
        body_set_motion_locks(&mut world, body_id, locks);

        assert!(body_is_bullet(&world, body_id));
    }

    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = true;
        let body_id = create_body(&mut world, &body_def);

        assert!(body_is_bullet(&world, body_id));

        body_set_bullet(&mut world, body_id, false);
        assert!(!body_is_bullet(&world, body_id));

        let mut locks = MotionLocks::default();
        locks.linear_x = true;
        body_set_motion_locks(&mut world, body_id, locks);

        assert!(!body_is_bullet(&world, body_id));
    }
}

/// Fast non-bullet sphere vs thin wall: CCD rewinds pose and move events match.
/// (TestContinuousMoveEvent)
#[test]
fn test_continuous_move_event() {
    use crate::body::body_get_transform;
    use crate::math_functions::Vec3;

    let mut world = World::new(&default_world_def());
    world.enable_continuous(true);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };
    let wall_id = create_body(&mut world, &body_def);
    let wall_box = make_box_hull(0.1, 5.0, 5.0);
    let shape_def = default_shape_def();
    create_hull_shape(&mut world, wall_id, &shape_def, &wall_box.base);

    body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.gravity_scale = 0.0;
    body_def.position = Pos {
        x: 3.0 as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };
    body_def.linear_velocity = Vec3 {
        x: -30.0,
        y: 0.0,
        z: 0.0,
    };
    let ball_id = create_body(&mut world, &body_def);
    let mut ball_shape = default_shape_def();
    ball_shape.density = 1.0;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.25,
    };
    create_sphere_shape(&mut world, ball_id, &ball_shape, &sphere);

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    let mut have_move = false;

    for _ in 0..30 {
        world.step(time_step, sub_step_count);

        let xf = body_get_transform(&world, ball_id);
        for event in &world.body_move_events {
            if event.body_id.index1 != ball_id.index1
                || event.body_id.generation != ball_id.generation
            {
                continue;
            }
            have_move = true;
            assert_eq!(event.transform.p.x, xf.p.x);
            assert_eq!(event.transform.p.y, xf.p.y);
            assert_eq!(event.transform.p.z, xf.p.z);
            assert_eq!(event.transform.q.v.x, xf.q.v.x);
            assert_eq!(event.transform.q.v.y, xf.q.v.y);
            assert_eq!(event.transform.q.v.z, xf.q.v.z);
            assert_eq!(event.transform.q.s, xf.q.s);
        }
    }

    assert!(have_move);

    let final_pos = body_get_position(&world, ball_id);
    assert!(
        0.2 < final_pos.x as f32 && (final_pos.x as f32) < 0.8,
        "expected ball stopped by wall in (0.2, 0.8), got {}",
        final_pos.x
    );
}

/// Fast bullet vs thin static wall: continuous on prevents tunneling; off tunnels.
#[test]
fn test_bullet_vs_thin_wall_toggle() {
    use crate::math_functions::Vec3;

    fn fire(continuous: bool) -> f32 {
        let mut world = World::new(&default_world_def());
        world.enable_continuous(continuous);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        let wall_id = create_body(&mut world, &body_def);
        let wall_box = make_box_hull(0.05, 5.0, 5.0);
        create_hull_shape(&mut world, wall_id, &default_shape_def(), &wall_box.base);

        body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = true;
        body_def.gravity_scale = 0.0;
        body_def.position = Pos {
            x: 2.0 as _,
            y: 0.0 as _,
            z: 0.0 as _,
        };
        body_def.linear_velocity = Vec3 {
            x: -100.0,
            y: 0.0,
            z: 0.0,
        };
        let bullet_id = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.1,
        };
        create_sphere_shape(&mut world, bullet_id, &shape_def, &sphere);

        for _ in 0..60 {
            world.step(1.0 / 60.0, 4);
        }
        body_get_position(&world, bullet_id).x as f32
    }

    let with_ccd = fire(true);
    let without_ccd = fire(false);

    assert!(
        with_ccd > -0.5 && with_ccd < 1.0,
        "CCD on should stop near wall, got {}",
        with_ccd
    );
    assert!(
        without_ccd < -1.0,
        "CCD off should tunnel past wall, got {}",
        without_ccd
    );
}
