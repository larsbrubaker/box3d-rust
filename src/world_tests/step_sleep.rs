//! World step and sleep tests from test_world.c.

use crate::body::{body_enable_sleep, body_is_sleep_enabled, create_body};
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

/// Create up to BODY_COUNT bodies then destroy them all while stepping.
/// (DestroyAllBodiesWorld)
#[test]
fn destroy_all_bodies_world() {
    const BODY_COUNT: usize = 10;

    let mut world = World::new(&default_world_def());

    let mut count = 0usize;
    let mut creating = true;
    let mut body_ids = [crate::id::NULL_BODY_ID; BODY_COUNT];

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let cube = make_cube_hull(0.5);

    for _ in 0..(2 * BODY_COUNT + 10) {
        if creating {
            if count < BODY_COUNT {
                body_ids[count] = create_body(&mut world, &body_def);
                let shape_def = default_shape_def();
                create_hull_shape(&mut world, body_ids[count], &shape_def, &cube.base);
                count += 1;
            } else {
                creating = false;
            }
        } else if count > 0 {
            crate::body::destroy_body(&mut world, body_ids[count - 1]);
            body_ids[count - 1] = crate::id::NULL_BODY_ID;
            count -= 1;
        }

        world.step(1.0 / 60.0, 3);
    }

    let counters = crate::world::world_get_counters(&world);
    assert_eq!(counters.body_count, 0);
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

    assert_eq!(
        world.solver_sets[AWAKE_SET as usize].contact_indices.len(),
        0
    );
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

/// (EnableSleepFlagSyncTest) — EnableSleep must sync bodySim/bodyState flags.
#[test]
fn enable_sleep_flag_sync() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.enable_sleep = false;
    let body_id = create_body(&mut world, &body_def);

    assert!(!body_is_sleep_enabled(&world, body_id));

    body_enable_sleep(&mut world, body_id, true);
    assert!(body_is_sleep_enabled(&world, body_id));

    // Would trip validate_solver_sets if bodySim/bodyState flags were stale.
    world.step(1.0 / 60.0, 4);
}

/// (EnableSleepNoopUnlockTest) — no-op EnableSleep must not leak world->locked.
#[test]
fn enable_sleep_noop_unlock() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.enable_sleep = true;
    let body_id = create_body(&mut world, &body_def);

    // No-op: enableSleep is already true. Must not leak the world lock.
    body_enable_sleep(&mut world, body_id, true);

    // Would fail if the lock had leaked (world.locked stays true).
    body_enable_sleep(&mut world, body_id, false);
    assert!(!body_is_sleep_enabled(&world, body_id));
    assert!(!world.locked);
}
