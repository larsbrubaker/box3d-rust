//! World step tests. EmptyWorld and HelloWorld from test_world.c.

use crate::body::{body_get_position, create_body, get_body_transform_quick};
use crate::contact::contact_flags;
use crate::core::NULL_INDEX;
use crate::geometry::Sphere;
use crate::hull::{make_box_hull, make_cube_hull};
use crate::math_functions::{Pos, VEC3_ZERO};
use crate::shape::{create_hull_shape, create_sphere_shape};
use crate::solver_set::AWAKE_SET;
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::World;

/// (EmptyWorld)
#[test]
fn empty_world() {
    let mut world = World::new(&default_world_def());
    let time_step = 1.0 / 60.0;
    let sub_step_count = 1;

    for _ in 0..60 {
        world.step(time_step, sub_step_count);
    }

    assert_eq!(world.step_index, 60);
}

/// Collide pass promotes an overlapping non-touching contact into the graph.
#[test]
fn step_collide_marks_overlapping_contact_touching() {
    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: 0.0 as _,
        y: 0.5 as _,
        z: 0.0 as _,
    };
    let ball = create_body(&mut world, &ball_def);

    let shape_def = default_shape_def();
    let box_hull = make_box_hull(5.0, 0.5, 5.0);
    create_hull_shape(&mut world, ground, &shape_def, &box_hull.base);

    let mut ball_shape = default_shape_def();
    ball_shape.density = 1.0;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    create_sphere_shape(&mut world, ball, &ball_shape, &sphere);

    world.step(1.0 / 60.0, 1);

    assert_eq!(world.solver_sets[AWAKE_SET as usize].contact_indices.len(), 0);
    let mut found_touching = false;
    for contact in &world.contacts {
        if contact.contact_id == NULL_INDEX {
            continue;
        }
        if (contact.flags & contact_flags::TOUCHING) != 0 {
            found_touching = true;
            assert_ne!(contact.color_index, NULL_INDEX);
            assert_ne!(contact.island_id, NULL_INDEX);
            assert!(!contact.manifolds.is_empty());
        }
    }
    assert!(found_touching);
}

/// A settled body falls asleep: its island moves to a sleeping solver set,
/// touching contacts leave the constraint graph, and the move event reports
/// fell_asleep. (b3TrySleepIsland via the sleep pass in b3Solve)
#[test]
fn body_falls_asleep_after_settling() {
    use crate::body::is_body_awake;
    use crate::solver_set::FIRST_SLEEPING_SET;

    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let ground_hull = make_box_hull(5.0, 0.5, 5.0);
    create_hull_shape(&mut world, ground, &default_shape_def(), &ground_hull.base);

    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    box_def.position = Pos {
        x: 0.0 as _,
        y: 1.05 as _,
        z: 0.0 as _,
    };
    let box_id = create_body(&mut world, &box_def);
    let cube = make_cube_hull(0.5);
    let mut cube_shape = default_shape_def();
    cube_shape.density = 1.0;
    create_hull_shape(&mut world, box_id, &cube_shape, &cube.base);

    let box_index = crate::body::get_body_full_id(&world, box_id);

    let mut sleep_step = NULL_INDEX;
    for step in 0..300 {
        world.step(1.0 / 60.0, 4);
        if !is_body_awake(&world, box_index) {
            sleep_step = step;
            break;
        }
    }

    assert!(sleep_step != NULL_INDEX, "body never fell asleep");

    // The island moved to a sleeping solver set.
    let set_index = world.bodies[box_index as usize].set_index;
    assert!(set_index >= FIRST_SLEEPING_SET);
    assert_eq!(world.solver_sets[AWAKE_SET as usize].body_sims.len(), 0);
    assert_eq!(world.solver_sets[AWAKE_SET as usize].island_sims.len(), 0);

    // Touching contacts moved out of the constraint graph into the sleeping set.
    let sleep_set = &world.solver_sets[set_index as usize];
    assert!(!sleep_set.contact_indices.is_empty());
    for &contact_id in &sleep_set.contact_indices {
        let contact = &world.contacts[contact_id as usize];
        assert_eq!(contact.set_index, set_index);
        assert_eq!(contact.color_index, NULL_INDEX);
        assert!((contact.flags & contact_flags::TOUCHING) != 0);
    }
    for color in &world.constraint_graph.colors {
        assert!(color.convex_contacts.is_empty());
        assert!(color.contacts.is_empty());
    }

    // The sleep step reported the fell_asleep move event.
    assert!(world.body_move_events.iter().any(|e| e.fell_asleep));
}

/// A new touching contact wakes a sleeping island; the woken contacts return
/// to the constraint graph and the island can fall back asleep afterwards.
/// (b3WakeSolverSet via b3LinkContact)
#[test]
fn sleeping_body_wakes_on_new_touching_contact() {
    use crate::body::is_body_awake;

    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let ground_hull = make_box_hull(5.0, 0.5, 5.0);
    create_hull_shape(&mut world, ground, &default_shape_def(), &ground_hull.base);

    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    box_def.position = Pos {
        x: 0.0 as _,
        y: 1.05 as _,
        z: 0.0 as _,
    };
    let first_id = create_body(&mut world, &box_def);
    let cube = make_cube_hull(0.5);
    let mut cube_shape = default_shape_def();
    cube_shape.density = 1.0;
    create_hull_shape(&mut world, first_id, &cube_shape, &cube.base);

    let first_index = crate::body::get_body_full_id(&world, first_id);

    let mut asleep = false;
    for _ in 0..300 {
        world.step(1.0 / 60.0, 4);
        if !is_body_awake(&world, first_index) {
            asleep = true;
            break;
        }
    }
    assert!(asleep, "first body never fell asleep");

    // Drop a second box onto the sleeping one.
    box_def.position = Pos {
        x: 0.0 as _,
        y: 3.0 as _,
        z: 0.0 as _,
    };
    let second_id = create_body(&mut world, &box_def);
    create_hull_shape(&mut world, second_id, &cube_shape, &cube.base);
    let second_index = crate::body::get_body_full_id(&world, second_id);

    let mut woke = false;
    for _ in 0..300 {
        world.step(1.0 / 60.0, 4);
        if is_body_awake(&world, first_index) {
            woke = true;
            break;
        }
    }
    assert!(woke, "sleeping body never woke from the new contact");

    // Both boxes settle back to sleep in the same island.
    let mut both_asleep = false;
    for _ in 0..600 {
        world.step(1.0 / 60.0, 4);
        if !is_body_awake(&world, first_index) && !is_body_awake(&world, second_index) {
            both_asleep = true;
            break;
        }
    }
    assert!(both_asleep, "stack never settled back to sleep");
    assert_eq!(
        world.bodies[first_index as usize].set_index,
        world.bodies[second_index as usize].set_index
    );
}

/// Destroying the middle body of a three-cube row leaves one island with a
/// pending split; the split pass separates the survivors so both can sleep
/// in their own solver sets. (b3SplitIsland via b3SplitIslandTask)
#[test]
fn island_splits_after_constraint_removal_and_sleeps() {
    use crate::body::{destroy_body, is_body_awake};

    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let ground_hull = make_box_hull(10.0, 0.5, 10.0);
    create_hull_shape(&mut world, ground, &default_shape_def(), &ground_hull.base);

    // Three cubes in a row, overlapping slightly so neighbors touch.
    let cube = make_cube_hull(0.5);
    let mut cube_shape = default_shape_def();
    cube_shape.density = 1.0;

    let mut ids = Vec::new();
    let mut indices = Vec::new();
    for i in 0..3 {
        let mut box_def = default_body_def();
        box_def.type_ = BodyType::Dynamic;
        box_def.position = Pos {
            x: (i as f32 * 0.98) as _,
            y: 1.05 as _,
            z: 0.0 as _,
        };
        let id = create_body(&mut world, &box_def);
        create_hull_shape(&mut world, id, &cube_shape, &cube.base);
        ids.push(id);
        indices.push(crate::body::get_body_full_id(&world, id));
    }

    // Let contacts form so all three cubes join one island.
    world.step(1.0 / 60.0, 4);
    let island_a = world.bodies[indices[0] as usize].island_id;
    assert_ne!(island_a, NULL_INDEX);
    assert_eq!(island_a, world.bodies[indices[1] as usize].island_id);
    assert_eq!(island_a, world.bodies[indices[2] as usize].island_id);

    // Destroy the middle cube: its contacts unlink, marking the island for a split.
    destroy_body(&mut world, ids[1]);
    assert!(world.islands[island_a as usize].constraint_remove_count > 0);

    // Without the split the island could never sleep (pending split + two
    // bodies). Both survivors sleeping in different sets proves the split ran.
    let mut both_asleep = false;
    for _ in 0..600 {
        world.step(1.0 / 60.0, 4);
        if !is_body_awake(&world, indices[0]) && !is_body_awake(&world, indices[2]) {
            both_asleep = true;
            break;
        }
    }
    assert!(both_asleep, "survivors never fell asleep after the split");
    assert_ne!(
        world.bodies[indices[0] as usize].island_id,
        world.bodies[indices[2] as usize].island_id
    );
    assert_ne!(
        world.bodies[indices[0] as usize].set_index,
        world.bodies[indices[2] as usize].set_index
    );
}

/// Sleep disabled keeps a settled body awake. (world.enable_sleep == false)
#[test]
fn sleep_disabled_keeps_body_awake() {
    use crate::body::is_body_awake;

    let mut world_def = default_world_def();
    world_def.enable_sleep = false;
    let mut world = World::new(&world_def);

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    let ground_hull = make_box_hull(5.0, 0.5, 5.0);
    create_hull_shape(&mut world, ground, &default_shape_def(), &ground_hull.base);

    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    box_def.position = Pos {
        x: 0.0 as _,
        y: 1.05 as _,
        z: 0.0 as _,
    };
    let box_id = create_body(&mut world, &box_def);
    let cube = make_cube_hull(0.5);
    let mut cube_shape = default_shape_def();
    cube_shape.density = 1.0;
    create_hull_shape(&mut world, box_id, &cube_shape, &cube.base);

    let box_index = crate::body::get_body_full_id(&world, box_id);

    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
        assert!(is_body_awake(&world, box_index));
    }
}

/// (TestHitEvents)
#[test]
fn hit_events() {
    let mut world_def = default_world_def();
    world_def.hit_event_threshold = 1.0;
    let mut world = World::new(&world_def);

    // Static ground
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = Pos {
        x: 0.0 as _,
        y: -0.5 as _,
        z: 0.0 as _,
    };
    let ground_id = create_body(&mut world, &body_def);
    let ground_box = make_box_hull(10.0, 0.5, 10.0);
    create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);

    // Sphere driven into the ground fast enough to clear the hit threshold
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.gravity_scale = 0.0;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 2.0 as _,
        z: 0.0 as _,
    };
    body_def.linear_velocity = crate::math_functions::Vec3 {
        x: 0.0,
        y: -30.0,
        z: 0.0,
    };
    let sphere_body_id = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_hit_events = true;
    shape_def.base_material.user_material_id = 7;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    create_sphere_shape(&mut world, sphere_body_id, &shape_def, &sphere);

    let mut hit_count = 0;
    let mut captured_speed = 0.0f32;
    let mut captured_material_a = 0u64;
    let mut captured_material_b = 0u64;
    let mut captured_normal = VEC3_ZERO;

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);

        if !world.contact_hit_events.is_empty() && hit_count == 0 {
            let hit = &world.contact_hit_events[0];
            captured_speed = hit.approach_speed;
            captured_normal = hit.normal;
            captured_material_a = hit.user_material_id_a;
            captured_material_b = hit.user_material_id_b;
        }

        hit_count += world.contact_hit_events.len();
    }

    assert!(hit_count >= 1);
    assert!(captured_speed > 1.0);
    // Head-on vertical impact: normal lies along Y
    assert!(captured_normal.x.abs() < 0.01);
    assert!(captured_normal.z.abs() < 0.01);
    // One side of the contact carries the sphere's user material
    assert!(captured_material_a == 7 || captured_material_b == 7);
}

/// Sensor sphere bullet flies through a static wall with sensor events.
/// Expects exactly one begin and one end. (TestSensor)
#[test]
fn sensor() {
    let mut world = World::new(&default_world_def());

    // Wall from x = 1 to x = 2
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = Pos {
        x: 1.5 as _,
        y: 11.0 as _,
        z: 0.0 as _,
    };
    let wall_id = create_body(&mut world, &body_def);
    let box_hull = make_box_hull(0.5, 10.0, 1.0);
    let mut shape_def = default_shape_def();
    shape_def.enable_sensor_events = true;
    create_hull_shape(&mut world, wall_id, &shape_def, &box_hull.base);

    // Bullet fired towards the wall
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.is_bullet = true;
    body_def.gravity_scale = 0.0;
    body_def.position = Pos {
        x: 7.39814 as _,
        y: 4.0 as _,
        z: 0.0 as _,
    };
    body_def.linear_velocity = crate::math_functions::Vec3 {
        x: -20.0,
        y: 0.0,
        z: 0.0,
    };
    let bullet_id = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.1,
    };
    create_sphere_shape(&mut world, bullet_id, &shape_def, &sphere);

    let mut begin_count = 0;
    let mut end_count = 0;

    loop {
        world.step(1.0 / 60.0, 4);

        let bullet_pos = body_get_position(&world, bullet_id);
        let events = world.get_sensor_events();

        if !events.begin_events.is_empty() {
            begin_count += 1;
        }
        if !events.end_events.is_empty() {
            end_count += 1;
        }

        if (bullet_pos.x as f32) < -1.0 {
            break;
        }
    }

    assert_eq!(begin_count, 1);
    assert_eq!(end_count, 1);
}

/// A dynamic body overlapping a sensor must not spuriously end/begin when it
/// falls asleep — overlaps persist across sleep.
#[test]
fn sensor_events_persist_across_sleep() {
    use crate::body::is_body_awake;

    let mut world = World::new(&default_world_def());

    // Static sensor volume
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };
    let sensor_body = create_body(&mut world, &body_def);
    let sensor_box = make_box_hull(2.0, 2.0, 2.0);
    let mut sensor_def = default_shape_def();
    sensor_def.is_sensor = true;
    sensor_def.enable_sensor_events = true;
    create_hull_shape(&mut world, sensor_body, &sensor_def, &sensor_box.base);

    // Dynamic box that settles inside the sensor
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 0.5 as _,
        z: 0.0 as _,
    };
    let box_id = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_sensor_events = true;
    let cube = make_cube_hull(0.4);
    create_hull_shape(&mut world, box_id, &shape_def, &cube.base);

    // Also need a ground so the box can settle and sleep
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = Pos {
        x: 0.0 as _,
        y: -0.5 as _,
        z: 0.0 as _,
    };
    let ground = create_body(&mut world, &ground_def);
    let ground_hull = make_box_hull(5.0, 0.5, 5.0);
    let mut ground_shape = default_shape_def();
    ground_shape.enable_sensor_events = true;
    create_hull_shape(&mut world, ground, &ground_shape, &ground_hull.base);

    let mut saw_begin = false;
    for _ in 0..180 {
        world.step(1.0 / 60.0, 4);
        let events = world.get_sensor_events();
        if !events.begin_events.is_empty() {
            saw_begin = true;
        }
    }

    assert!(saw_begin, "expected at least one sensor begin while settling");
    let box_index = crate::body::get_body_full_id(&world, box_id);
    assert!(
        !is_body_awake(&world, box_index),
        "box should have fallen asleep"
    );

    // After sleep, further steps must not emit spurious begin/end
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
        let events = world.get_sensor_events();
        assert!(
            events.begin_events.is_empty(),
            "spurious sensor begin after sleep"
        );
        assert!(
            events.end_events.is_empty(),
            "spurious sensor end after sleep"
        );
    }
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
