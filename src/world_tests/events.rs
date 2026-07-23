//! Contact hit, begin/end, and sensor event world tests from test_world.c.

use crate::body::{body_get_position, create_body};
use crate::compound::{
    create_compound, CompoundCapsuleDef, CompoundDef, CompoundHullDef, CompoundSphereDef,
};
use crate::contact::contact_is_valid;
use crate::geometry::{default_surface_material, Capsule, Sphere};
use crate::hull::{make_box_hull, make_cube_hull};
use crate::math_functions::{Pos, Transform, Vec3, QUAT_IDENTITY, VEC3_ZERO};
use crate::shape::{create_baked_compound_shape, create_hull_shape, create_sphere_shape};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::{world_get_contact_events, World};
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
    create_hull_shape(
        &mut world,
        ground_id,
        &default_shape_def(),
        &ground_box.base,
    );

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

    assert!(
        saw_begin,
        "expected at least one sensor begin while settling"
    );
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

/// (TestCompoundHitEvents)
#[test]
fn compound_hit_events() {
    const HULL_MATERIAL_A: u64 = 11;
    const HULL_MATERIAL_B: u64 = 22;
    const SPHERE_MATERIAL: u64 = 99;
    const HULL_CENTER_X: f32 = 3.0;

    for side in 0..2 {
        let expected_hull_material = if side == 0 {
            HULL_MATERIAL_A
        } else {
            HULL_MATERIAL_B
        };
        let spawn_x = if side == 0 {
            -HULL_CENTER_X
        } else {
            HULL_CENTER_X
        };

        let mut world_def = default_world_def();
        world_def.hit_event_threshold = 1.0;
        let mut world = World::new(&world_def);

        let box_a = make_box_hull(1.0, 1.0, 1.0);
        let box_b = make_box_hull(1.0, 1.0, 1.0);

        let mut mat_a = default_surface_material();
        mat_a.user_material_id = HULL_MATERIAL_A;
        let mut mat_b = default_surface_material();
        mat_b.user_material_id = HULL_MATERIAL_B;

        let hulls = [
            CompoundHullDef {
                hull: &box_a.base,
                transform: Transform {
                    p: Vec3::new(-HULL_CENTER_X, 0.0, 0.0),
                    q: QUAT_IDENTITY,
                },
                material: mat_a,
            },
            CompoundHullDef {
                hull: &box_b.base,
                transform: Transform {
                    p: Vec3::new(HULL_CENTER_X, 0.0, 0.0),
                    q: QUAT_IDENTITY,
                },
                material: mat_b,
            },
        ];
        let compound = create_compound(&CompoundDef {
            hulls: &hulls,
            ..Default::default()
        })
        .expect("compound");

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        let compound_body = create_body(&mut world, &body_def);
        create_baked_compound_shape(&mut world, compound_body, &default_shape_def(), &compound);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.gravity_scale = 0.0;
        body_def.position = Pos {
            x: spawn_x as _,
            y: 3.0 as _,
            z: 0.0 as _,
        };
        body_def.linear_velocity = Vec3 {
            x: 0.0,
            y: -30.0,
            z: 0.0,
        };
        let sphere_body = create_body(&mut world, &body_def);
        let mut sphere_shape_def = default_shape_def();
        sphere_shape_def.density = 1.0;
        sphere_shape_def.enable_hit_events = true;
        sphere_shape_def.base_material.user_material_id = SPHERE_MATERIAL;
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        };
        create_sphere_shape(&mut world, sphere_body, &sphere_shape_def, &sphere);

        let mut hit_count = 0;
        let mut captured_material_a = 0u64;
        let mut captured_material_b = 0u64;

        for _ in 0..30 {
            world.step(1.0 / 60.0, 4);

            if !world.contact_hit_events.is_empty() && hit_count == 0 {
                let hit = &world.contact_hit_events[0];
                captured_material_a = hit.user_material_id_a;
                captured_material_b = hit.user_material_id_b;
            }
            hit_count += world.contact_hit_events.len();
        }

        assert!(hit_count >= 1, "side {side}: expected hit events");
        assert!(
            captured_material_a == SPHERE_MATERIAL || captured_material_b == SPHERE_MATERIAL,
            "side {side}: sphere material missing"
        );
        assert!(
            captured_material_a == expected_hull_material
                || captured_material_b == expected_hull_material,
            "side {side}: expected child material {expected_hull_material}, got {captured_material_a}/{captured_material_b}"
        );
    }
}

const CHILD0_MATERIAL_ID: u64 = 101;
const CHILD1_MATERIAL_ID: u64 = 202;
const PROBE_MATERIAL_ID: u64 = 999;

#[derive(Default, Clone, Copy)]
struct MaterialCapture {
    call_count: i32,
    saw_child0: bool,
    saw_child1: bool,
    mixed_friction: f32,
}

// No context pointer on mixing callbacks, so capture through thread-local state
// (the C test uses file scope). The friction callback runs on the test thread
// during world.step, so the thread-local is isolated per test.
thread_local! {
    static MATERIAL_CAPTURE: std::cell::RefCell<MaterialCapture> =
        const { std::cell::RefCell::new(MaterialCapture {
            call_count: 0,
            saw_child0: false,
            saw_child1: false,
            mixed_friction: 0.0,
        }) };
}

fn capture_friction_mix(
    friction_a: f32,
    user_material_id_a: u64,
    friction_b: f32,
    user_material_id_b: u64,
) -> f32 {
    MATERIAL_CAPTURE.with(|capture| {
        let mut capture = capture.borrow_mut();
        capture.call_count += 1;

        if user_material_id_a == CHILD0_MATERIAL_ID || user_material_id_b == CHILD0_MATERIAL_ID {
            capture.saw_child0 = true;
        }

        if user_material_id_a == CHILD1_MATERIAL_ID || user_material_id_b == CHILD1_MATERIAL_ID {
            capture.saw_child1 = true;
            capture.mixed_friction = (friction_a * friction_b).sqrt();
        }
    });

    (friction_a * friction_b).sqrt()
}

/// Contact material selection must use the struck compound child's material, not
/// entry 0 of the compound material table. Child 0 gets low friction, child 1
/// high friction, and a unit friction sphere strikes only child 1, so the mixing
/// callback must see child 1's values. Covers sphere, capsule, and hull children.
/// Issue #69. (TestCompoundContactMaterials)
#[test]
fn compound_contact_materials() {
    let mut mat0 = default_surface_material();
    mat0.friction = 0.04;
    mat0.user_material_id = CHILD0_MATERIAL_ID;

    let mut mat1 = default_surface_material();
    mat1.friction = 0.81;
    mat1.user_material_id = CHILD1_MATERIAL_ID;

    let box_hull = make_box_hull(0.5, 0.5, 0.5);

    // Children at x = -3 and x = +3, all with their top face or surface at y = 0.5
    let spheres = [
        CompoundSphereDef {
            sphere: Sphere {
                center: Vec3::new(-3.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat0,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: Vec3::new(3.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat1,
        },
    ];

    let capsules = [
        CompoundCapsuleDef {
            capsule: Capsule {
                center1: Vec3::new(-3.0, 0.0, -0.5),
                center2: Vec3::new(-3.0, 0.0, 0.5),
                radius: 0.5,
            },
            material: mat0,
        },
        CompoundCapsuleDef {
            capsule: Capsule {
                center1: Vec3::new(3.0, 0.0, -0.5),
                center2: Vec3::new(3.0, 0.0, 0.5),
                radius: 0.5,
            },
            material: mat1,
        },
    ];

    let hulls = [
        CompoundHullDef {
            hull: &box_hull.base,
            transform: Transform {
                p: Vec3::new(-3.0, 0.0, 0.0),
                q: QUAT_IDENTITY,
            },
            material: mat0,
        },
        CompoundHullDef {
            hull: &box_hull.base,
            transform: Transform {
                p: Vec3::new(3.0, 0.0, 0.0),
                q: QUAT_IDENTITY,
            },
            material: mat1,
        },
    ];

    for child_type in 0..3 {
        let compound_def = match child_type {
            0 => CompoundDef {
                spheres: &spheres,
                ..Default::default()
            },
            1 => CompoundDef {
                capsules: &capsules,
                ..Default::default()
            },
            _ => CompoundDef {
                hulls: &hulls,
                ..Default::default()
            },
        };

        let compound = create_compound(&compound_def).expect("compound");

        MATERIAL_CAPTURE.with(|capture| *capture.borrow_mut() = MaterialCapture::default());

        let mut world_def = default_world_def();
        world_def.friction_callback = Some(capture_friction_mix);
        let mut world = World::new(&world_def);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        let compound_body = create_body(&mut world, &body_def);
        create_baked_compound_shape(&mut world, compound_body, &default_shape_def(), &compound);

        // Sphere driven straight down onto child 1
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.gravity_scale = 0.0;
        body_def.position = Pos {
            x: 3.0 as _,
            y: 3.0 as _,
            z: 0.0 as _,
        };
        body_def.linear_velocity = Vec3 {
            x: 0.0,
            y: -30.0,
            z: 0.0,
        };
        let sphere_body = create_body(&mut world, &body_def);
        let mut sphere_shape_def = default_shape_def();
        sphere_shape_def.density = 1.0;
        sphere_shape_def.base_material.friction = 1.0;
        sphere_shape_def.base_material.user_material_id = PROBE_MATERIAL_ID;
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        };
        create_sphere_shape(&mut world, sphere_body, &sphere_shape_def, &sphere);

        for _ in 0..30 {
            world.step(1.0 / 60.0, 4);
        }

        let capture = MATERIAL_CAPTURE.with(|capture| *capture.borrow());

        assert!(
            capture.call_count > 0,
            "child_type {child_type}: expected friction mixing calls"
        );
        // The pre-fix code fed material table entry 0 to the mixing callback for every child
        assert!(
            !capture.saw_child0,
            "child_type {child_type}: mixing saw child 0 material"
        );
        assert!(
            capture.saw_child1,
            "child_type {child_type}: mixing did not see child 1 material"
        );
        // sqrt(0.81 * 1.0)
        assert!(
            (capture.mixed_friction - 0.9).abs() < 1e-5,
            "child_type {child_type}: mixed friction {} != 0.9",
            capture.mixed_friction
        );
    }
}

/// Dynamic sphere with restitution bounces on ground; begin and end contact
/// events fire with valid shape/contact ids. (TestContactEvents)
#[test]
fn contact_events() {
    let mut world = World::new(&default_world_def());

    // Static ground
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = Pos {
        x: 0.0 as _,
        y: (-0.5) as _,
        z: 0.0 as _,
    };
    let ground_id = create_body(&mut world, &body_def);
    let ground_box = make_box_hull(10.0, 0.5, 10.0);
    let ground_shape_id = create_hull_shape(
        &mut world,
        ground_id,
        &default_shape_def(),
        &ground_box.base,
    );

    // Dynamic sphere dropped onto the ground; restitution causes bounce so we get end events
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0 as _,
        y: 5.0 as _,
        z: 0.0 as _,
    };
    let sphere_body_id = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.enable_contact_events = true;
    shape_def.base_material.restitution = 0.6;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let sphere_shape_id = create_sphere_shape(&mut world, sphere_body_id, &shape_def, &sphere);

    let mut begin_count = 0;
    let mut end_count = 0;
    let mut ids_checked = false;

    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);

        let events = world_get_contact_events(&world);

        if !events.begin_events.is_empty() && !ids_checked {
            let be = events.begin_events[0];
            let a_is_sphere = be.shape_id_a.id_equals(sphere_shape_id);
            let b_is_sphere = be.shape_id_b.id_equals(sphere_shape_id);
            let a_is_ground = be.shape_id_a.id_equals(ground_shape_id);
            let b_is_ground = be.shape_id_b.id_equals(ground_shape_id);
            assert!(
                (a_is_sphere && b_is_ground) || (a_is_ground && b_is_sphere),
                "begin event shapes must be sphere and ground"
            );
            assert!(contact_is_valid(&world, be.contact_id));
            ids_checked = true;
        }

        begin_count += events.begin_events.len();
        end_count += events.end_events.len();
    }

    assert!(ids_checked);
    assert!(begin_count >= 1);
    assert!(end_count >= 1);
}
