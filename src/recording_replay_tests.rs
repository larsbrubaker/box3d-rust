//! Recording / replay tests ported from test_recording.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::create_body;
use crate::geometry::Sphere;
use crate::hull::{create_hull, make_box_hull};
use crate::math_functions::{Pos, Vec3, VEC3_ZERO};
use crate::recording::{validate_replay, Recording, REC_MAGIC};
use crate::shape::{create_hull_shape, create_sphere_shape};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::{
    destroy_world, world_set_gravity, world_start_recording, world_stop_recording, World,
};

/// Sphere round-trip: record/step/stop, then replay and validate. (SphereRoundTrip)
#[test]
fn sphere_round_trip() {
    let rec = Recording::new(0);
    let mut world = World::new(&default_world_def());

    world_start_recording(&mut world, rec);

    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);

    let ground_box = make_box_hull(50.0, 1.0, 50.0);
    let ground_shape_def = default_shape_def();
    create_hull_shape(&mut world, ground_id, &ground_shape_def, &ground_box.base);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0,
        y: 5.0,
        z: 0.0,
    };
    let body_id = create_body(&mut world, &body_def);

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let mut sphere_def = default_shape_def();
    sphere_def.density = 1.0;
    create_sphere_shape(&mut world, body_id, &sphere_def, &sphere);

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for _ in 0..30 {
        world.step(time_step, sub_step_count);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");

    assert!(validate_replay(rec.data(), 1));
}

/// Empty world: start/stop with no ops beyond seed hash. (EmptyWorldRoundTrip)
#[test]
fn empty_world_round_trip() {
    let rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, rec);
    let rec = world_stop_recording(&mut world).expect("recording active");
    assert!(rec.size() >= 48);
    assert_eq!(
        u32::from_le_bytes(rec.data()[0..4].try_into().unwrap()),
        REC_MAGIC
    );
    assert!(validate_replay(rec.data(), 1));
}

/// Hull dedup: three bodies sharing the same hull → one registry entry. (HullDedup)
#[test]
fn hull_dedup() {
    let pts = [
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: -1.0,
        },
        Vec3 {
            x: 1.0,
            y: -1.0,
            z: -1.0,
        },
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: -1.0,
        },
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: -1.0,
        },
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: 1.0,
        },
        Vec3 {
            x: 1.0,
            y: -1.0,
            z: 1.0,
        },
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: 1.0,
        },
    ];
    let hull = create_hull(&pts, 8).expect("hull");

    let rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, rec);

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;

    for i in 0..3 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (i * 3) as _,
            y: 5.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &shape_def, &hull);
    }

    let time_step = 1.0 / 60.0;
    for _ in 0..5 {
        world.step(time_step, 4);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");

    assert!(validate_replay(rec.data(), 1));

    let bytes = rec.data();
    assert!(bytes.len() >= 48);
    let reg_off = u64::from_le_bytes(bytes[32..40].try_into().unwrap()) as usize;
    assert!(reg_off != 0 && reg_off + 4 <= bytes.len());
    let entry_count = u32::from_le_bytes(bytes[reg_off..reg_off + 4].try_into().unwrap());
    assert_eq!(entry_count, 1);
}

/// (ValidateReplayAtWidth)
fn validate_replay_at_width(rec: &Recording, width: i32) -> bool {
    crate::simd_scalar::set_simd_width(width);
    let valid = validate_replay(rec.data(), 1);
    crate::simd_scalar::set_simd_width(0);
    valid
}

/// A recording made at the native SIMD width must replay at width 4 and vice versa.
/// (CrossWidthReplay)
///
/// C skips this when the native width is 4, which is always the case for the scalar port. The
/// port still records and replays at both requested widths, which then proves that forcing the
/// width leaves replay validation intact.
#[test]
fn cross_width_replay() {
    use crate::hull::{create_complex_hull, create_cone, create_cylinder, create_rock};
    use crate::math_functions::make_quat_from_axis_angle;

    crate::simd_scalar::set_simd_width(0);
    let native_width = crate::simd_scalar::get_simd_width();
    assert_eq!(native_width, 4, "the scalar port has no width 8 kernels");

    const HULL_COUNT: usize = 6;
    let mut hulls: Vec<HullData> = Vec::with_capacity(HULL_COUNT);
    hulls.push(create_rock(0.5).expect("rock"));
    hulls.push(create_complex_hull(0.5).expect("complex hull"));
    hulls.push(create_cylinder(0.8, 0.4, 0.0, 8).expect("cylinder"));
    hulls.push(create_cone(0.8, 0.5, 0.2, 7).expect("cone"));

    let mut seed: u32 = 12345;
    for _ in 4..HULL_COUNT {
        let mut points = [VEC3_ZERO; 20];
        for point in points.iter_mut() {
            let mut v = [0.0f32; 3];
            for (k, value) in v.iter_mut().enumerate() {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                *value = ((seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5)
                    * if k == 1 { 0.6 } else { 1.0 };
            }
            *point = Vec3 {
                x: v[0],
                y: v[1],
                z: v[2],
            };
        }
        hulls.push(create_hull(&points, 20).expect("random hull"));
    }

    let rec = Recording::new(0);

    crate::simd_scalar::set_simd_width(native_width);

    let mut world = World::new(&default_world_def());

    world_start_recording(&mut world, rec);

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(20.0, 1.0, 20.0);
    let ground_shape = default_shape_def();
    create_hull_shape(&mut world, ground_id, &ground_shape, &ground_box.base);

    let mut dynamic_shape = default_shape_def();
    dynamic_shape.density = 1.0;

    for i in 0..18 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (0.4 * (i % 3) as f32 - 0.4) as _,
            y: (1.5 + 0.9 * i as f32) as _,
            z: (0.3 * (i % 2) as f32) as _,
        };
        body_def.rotation = make_quat_from_axis_angle(
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            0.37 * i as f32,
        );
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &dynamic_shape, &hulls[i % HULL_COUNT]);
    }

    let time_step = 1.0 / 60.0;
    for _ in 0..120 {
        world.step(time_step, 4);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    assert!(destroy_world(world).is_none());
    crate::simd_scalar::set_simd_width(0);

    let narrow_valid = validate_replay_at_width(&rec, 4);
    let native_valid = validate_replay_at_width(&rec, native_width);

    assert!(narrow_valid);
    assert!(native_valid);
}

/// Mid-stream snapshot with floating dynamics (no contacts). (MidStreamNoContacts)
#[test]
fn mid_stream_no_contacts() {
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;

    for i in 0..4 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (i * 10) as _,
            y: 50.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_sphere_shape(&mut world, body_id, &shape_def, &sphere);
    }

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for _ in 0..10 {
        world.step(time_step, sub_step_count);
    }

    let rec = Recording::new(0);
    world_start_recording(&mut world, rec);

    for _ in 0..30 {
        world.step(time_step, sub_step_count);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    assert!(validate_replay(rec.data(), 1));
}

/// Mid-stream snapshot with contacts / warm starts. (MidStreamContacts)
#[test]
fn mid_stream_contacts() {
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    {
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        let ground_id = create_body(&mut world, &ground_def);
        let ground_box = make_box_hull(50.0, 1.0, 50.0);
        let ground_shape = default_shape_def();
        create_hull_shape(&mut world, ground_id, &ground_shape, &ground_box.base);
    }

    let mut dynamic_shape = default_shape_def();
    dynamic_shape.density = 1.0;

    for i in 0..3 {
        let box_hull = make_box_hull(0.5, 0.5, 0.5);
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: ((i as f32 * 2.0) - 2.0) as _,
            y: 5.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &dynamic_shape, &box_hull.base);
    }

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for _ in 0..60 {
        world.step(time_step, sub_step_count);
    }

    let rec = Recording::new(0);
    world_start_recording(&mut world, rec);

    for _ in 0..30 {
        world.step(time_step, sub_step_count);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    assert!(validate_replay(rec.data(), 1));
}

/// Build the SphereRoundTrip scene with a recording attached and step it 30 frames.
fn recorded_sphere_world() -> World {
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, Recording::new(0));
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(50.0, 1.0, 50.0);
    create_hull_shape(
        &mut world,
        ground_id,
        &default_shape_def(),
        &ground_box.base,
    );

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 0.0,
        y: 5.0,
        z: 0.0,
    };
    let body_id = create_body(&mut world, &body_def);
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let mut sphere_def = default_shape_def();
    sphere_def.density = 1.0;
    create_sphere_shape(&mut world, body_id, &sphere_def, &sphere);

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }
    world
}

/// b3DestroyWorld detaches and finalizes an active recording (b3StopRecordingInternal) before
/// teardown. The finalized stream must be byte-identical to an explicit stop and replay cleanly.
#[test]
fn destroy_world_finalizes_active_recording() {
    let mut stopped_world = recorded_sphere_world();
    let stopped = world_stop_recording(&mut stopped_world).expect("recording active");

    let destroyed = destroy_world(recorded_sphere_world()).expect("recording finalized");

    assert_eq!(destroyed.data(), stopped.data());
    assert!(validate_replay(destroyed.data(), 1));
}

/// Without an active recording, destroy_world returns None and still releases every live shape
/// through b3DestroyShapeAllocations: the host debug-shape handles are destroyed and the hull
/// database drains.
#[test]
fn destroy_world_releases_shape_allocations() {
    use std::sync::atomic::{AtomicI32, Ordering};

    fn count_destroy(_user_shape: u64, ctx: u64) {
        // SAFETY: `ctx` is the address of an AtomicI32 that outlives destroy_world below.
        let counter = unsafe { &*(ctx as *const AtomicI32) };
        counter.fetch_add(1, Ordering::Relaxed);
    }

    let destroyed = AtomicI32::new(0);
    let mut def = default_world_def();
    def.destroy_debug_shape = Some(count_destroy);
    def.user_debug_shape_context = &destroyed as *const AtomicI32 as u64;
    let mut world = World::new(&def);

    let body_id = create_body(&mut world, &default_body_def());
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);
    create_hull_shape(&mut world, body_id, &default_shape_def(), &box_hull.base);
    assert_eq!(world.hull_database.len(), 1);

    // Stand in for the renderer handles world_draw would have created lazily.
    for shape in world.shapes.iter_mut() {
        shape.user_shape = 1;
    }

    assert!(destroy_world(world).is_none());
    assert_eq!(destroyed.load(Ordering::Relaxed), 2);
}

include!("recording_replay_tests_extra.rs");
include!("recording_replay_tests_draw.rs");
include!("recording_replay_tests_geometry.rs");
include!("recording_replay_tests_corrupt.rs");
include!("recording_replay_tests_compound.rs");
include!("recording_replay_tests_registry.rs");
include!("recording_replay_tests_bundled.rs");
include!("recording_replay_tests_keyframes.rs");
include!("recording_replay_tests_keyframe_seek.rs");
include!("recording_replay_tests_frame_queries.rs");

/// Safety factor of the named body in a replay world, or -1 if the body is not in the list.
/// (ReplaySafetyFactor)
fn replay_safety_factor(player: &crate::recording::RecPlayer, name: &str) -> f32 {
    use crate::body::{body_get_name, body_get_safety_factor, body_is_valid};

    let count = player.get_body_count();
    for i in 0..count {
        let body_id = player.get_body_id(i);
        if body_is_valid(player.world(), body_id) && body_get_name(player.world(), body_id) == name
        {
            return body_get_safety_factor(player.world(), body_id);
        }
    }

    -1.0
}

/// The safety factor additions to the C `AllOps` session (the Rust port has no single
/// `AllOps` test): a disabled/enabled body gets `body_set_safety_factor`, a fast body
/// records a non-default `BodyDef::safety_factor`, and the setter is injected at frame 6.
/// The replay world must restore both the def value and the setter value.
#[test]
fn safety_factor_round_trip() {
    use crate::body::{body_disable, body_enable, body_get_safety_factor, body_set_safety_factor};
    use crate::recording::RecPlayer;

    let rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, rec);

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(50.0, 1.0, 50.0);
    create_hull_shape(
        &mut world,
        ground_id,
        &default_shape_def(),
        &ground_box.base,
    );

    let sphere_shape_def = default_shape_def();

    let mut disable_def = default_body_def();
    disable_def.type_ = BodyType::Dynamic;
    disable_def.position = Pos {
        x: 9.0,
        y: 5.0,
        z: 0.0,
    };
    disable_def.name = "dropBody".to_string();
    let disable_id = create_body(&mut world, &disable_def);
    let disable_sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.3,
    };
    create_sphere_shape(&mut world, disable_id, &sphere_shape_def, &disable_sphere);
    body_disable(&mut world, disable_id);
    body_enable(&mut world, disable_id);
    body_set_safety_factor(&mut world, disable_id, 0.25);

    // A body that falls fast enough to stay on the continuous path for the whole session. The
    // non-default factor is what proves the widened body def payload round-trips, since every
    // other body records the default. Named so the replay check can find it.
    let mut fast_def = default_body_def();
    fast_def.type_ = BodyType::Dynamic;
    fast_def.position = Pos {
        x: -14.0,
        y: 30.0,
        z: 0.0,
    };
    fast_def.linear_velocity = Vec3 {
        x: 0.0,
        y: -8.0,
        z: 0.0,
    };
    fast_def.safety_factor = 0.1;
    fast_def.name = "fastBody".to_string();
    let fast_id = create_body(&mut world, &fast_def);
    assert_eq!(body_get_safety_factor(&world, fast_id), 0.1);
    let fast_box = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, fast_id, &sphere_shape_def, &fast_box.base);

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for i in 0..12 {
        // Inject mutators mid-simulation
        if i == 6 {
            body_set_safety_factor(&mut world, fast_id, 0.4);
        }

        world.step(time_step, sub_step_count);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    assert!(validate_replay(rec.data(), 1));

    // The state hash only proves the op stream stayed aligned. Read the safety factor back out of
    // the replay world so a value that is written but never restored is caught too.
    let mut player = RecPlayer::create(rec.data(), 1).expect("player");

    // Frame 0 replays the pre-step creates, so the def value is in place
    assert!(player.step_frame());
    assert_eq!(replay_safety_factor(&player, "fastBody"), 0.1);
    assert_eq!(replay_safety_factor(&player, "dropBody"), 0.25);

    // The setter is injected at frame 6
    while player.step_frame() {}
    assert_eq!(replay_safety_factor(&player, "fastBody"), 0.4);
}

/// The restitution world setters added to the C `AllOps` session: `WorldSetRestitutionIterations`
/// (0x0E) and `WorldEnableRestitutionPropagation` (0x0F) are recorded mid-session and the replay
/// world must end with the same settings and the same state.
#[test]
fn restitution_setters_round_trip() {
    use crate::recording::RecPlayer;
    use crate::world::{
        world_enable_restitution_propagation, world_get_restitution_iterations,
        world_is_restitution_propagation_enabled, world_set_restitution_iterations,
    };

    let rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, rec);

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(50.0, 1.0, 50.0);
    let mut ground_shape_def = default_shape_def();
    ground_shape_def.base_material.restitution = 0.5;
    create_hull_shape(&mut world, ground_id, &ground_shape_def, &ground_box.base);

    let mut shape_def = default_shape_def();
    shape_def.base_material.restitution = 0.8;
    for i in 0..3 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (2.0 * i as f32) as _,
            y: 3.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        };
        create_sphere_shape(&mut world, body_id, &shape_def, &sphere);
    }

    world_set_restitution_iterations(&mut world, 3);
    world_enable_restitution_propagation(&mut world, true);
    assert_eq!(world_get_restitution_iterations(&world), 3);
    assert!(world_is_restitution_propagation_enabled(&world));

    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while player.step_frame() {}
    assert_eq!(world_get_restitution_iterations(player.world()), 3);
    assert!(world_is_restitution_propagation_enabled(player.world()));
}

/// The isotropic inertia tolerance setter added to the C `AllOps` session:
/// `WorldSetIsotropicInertiaTolerance` (0x70). The tolerance only reaches bodies whose mass is
/// computed afterwards. A free spinning rod made after the change loses its gyroscopic torque,
/// so replay diverges if the op goes missing.
#[test]
fn isotropic_inertia_tolerance_round_trip() {
    use crate::body::{body_flags, get_body_full_id};
    use crate::recording::RecPlayer;
    use crate::world::{
        world_get_isotropic_inertia_tolerance, world_set_isotropic_inertia_tolerance,
    };

    let rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, rec);

    world_set_isotropic_inertia_tolerance(&mut world, 1.0);
    let rod_id = {
        let mut rod_def = default_body_def();
        rod_def.type_ = BodyType::Dynamic;
        rod_def.position = Pos {
            x: 40.0,
            y: 10.0,
            z: 0.0,
        };
        rod_def.gravity_scale = 0.0;
        rod_def.angular_velocity = Vec3 {
            x: 1.0,
            y: 5.0,
            z: 0.5,
        };
        let rod_id = create_body(&mut world, &rod_def);
        let rod = make_box_hull(0.1, 1.0, 0.1);
        let rod_shape_def = default_shape_def();
        create_hull_shape(&mut world, rod_id, &rod_shape_def, &rod.base);
        rod_id
    };

    // The default tolerance would enable gyroscopic torque on this rod.
    let rod_index = get_body_full_id(&world, rod_id);
    assert_eq!(
        world.bodies[rod_index as usize].flags & body_flags::ENABLE_GYROSCOPIC_TORQUE,
        0
    );

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while player.step_frame() {}
    let replay = player.world();
    assert_eq!(world_get_isotropic_inertia_tolerance(replay), 1.0);
    assert_eq!(
        replay.bodies[rod_index as usize].flags & body_flags::ENABLE_GYROSCOPIC_TORQUE,
        0
    );
}
