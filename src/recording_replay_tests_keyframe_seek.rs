// Keyframe seek equivalence and renderer handle reuse tests. `include!`d into
// `recording_replay_tests.rs`, so it shares that module's imports and the scene helpers of its
// sibling files (`record_box_stack_scene`, `rec_compound`).
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

/// A baked compound ground (`rec_compound`) with three spheres dropped onto it.
fn record_compound_drop(total_frames: i32) -> Recording {
    let compound = rec_compound();
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, Recording::new(0));

    let ground_id = create_body(&mut world, &default_body_def());
    crate::shape::create_baked_compound_shape(
        &mut world,
        ground_id,
        &default_shape_def(),
        &compound,
    );

    let mut sd = default_shape_def();
    sd.density = 1.0;
    for i in 0..3 {
        let mut ball_def = default_body_def();
        ball_def.type_ = BodyType::Dynamic;
        ball_def.position = Pos {
            x: (0.3 * i as f32 - 0.3) as _,
            y: (3.0 + 1.2 * i as f32) as _,
            z: 0.1 as _,
        };
        let ball_id = create_body(&mut world, &ball_def);
        create_sphere_shape(
            &mut world,
            ball_id,
            &sd,
            &Sphere {
                center: VEC3_ZERO,
                radius: 0.25,
            },
        );
    }

    for _ in 0..total_frames {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world).expect("recording active")
}

/// A pendulum chain of capsules on revolute joints (one motorized) plus a spherical joint, with a
/// body destroyed and another created mid-recording, so the outliner list gains a hole and a late
/// ordinal that keyframes must restore verbatim.
fn record_joint_chain(total_frames: i32) -> Recording {
    use crate::body::destroy_body;
    use crate::geometry::Capsule;
    use crate::joint::{create_revolute_joint, create_spherical_joint};
    use crate::math_functions::{Transform, QUAT_IDENTITY};
    use crate::shape::create_capsule_shape;
    use crate::types::{default_revolute_joint_def, default_spherical_joint_def};

    let v = |x: f32, y: f32, z: f32| Vec3 { x, y, z };
    let at = |p: Vec3| Transform {
        p,
        q: QUAT_IDENTITY,
    };

    let mut world = World::new(&default_world_def());
    world_set_gravity(&mut world, v(0.0, -10.0, 0.0));
    world_start_recording(&mut world, Recording::new(0));

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(20.0, 0.5, 20.0);
    create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);

    let capsule = Capsule {
        center1: v(-0.4, 0.0, 0.0),
        center2: v(0.4, 0.0, 0.0),
        radius: 0.15,
    };
    let mut sd = default_shape_def();
    sd.density = 1.0;

    let mut prev = ground_id;
    for i in 0..5 {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = Pos {
            x: (i as f32 + 0.5) as _,
            y: 6.0 as _,
            z: 0.0 as _,
        };
        let body = create_body(&mut world, &bd);
        create_capsule_shape(&mut world, body, &sd, &capsule);
        if i == 3 {
            let mut jd = default_spherical_joint_def();
            jd.base.body_id_a = prev;
            jd.base.body_id_b = body;
            jd.base.local_frame_a = at(v(0.5, 0.0, 0.0));
            jd.base.local_frame_b = at(v(-0.5, 0.0, 0.0));
            create_spherical_joint(&mut world, &jd);
        } else {
            let mut jd = default_revolute_joint_def();
            jd.base.body_id_a = prev;
            jd.base.body_id_b = body;
            jd.base.local_frame_a = if i == 0 {
                at(v(0.0, 6.0, 0.0))
            } else {
                at(v(0.5, 0.0, 0.0))
            };
            jd.base.local_frame_b = at(v(-0.5, 0.0, 0.0));
            jd.enable_motor = i == 1;
            jd.max_motor_torque = 5.0;
            jd.motor_speed = 1.0;
            create_revolute_joint(&mut world, &jd);
        }
        prev = body;
    }

    // A loose box that falls onto the ground and is destroyed mid-recording.
    let mut loose_def = default_body_def();
    loose_def.type_ = BodyType::Dynamic;
    loose_def.position = Pos {
        x: -3.0 as _,
        y: 2.0 as _,
        z: 0.0 as _,
    };
    let loose = create_body(&mut world, &loose_def);
    let loose_box = make_box_hull(0.3, 0.3, 0.3);
    create_hull_shape(&mut world, loose, &sd, &loose_box.base);

    for i in 0..total_frames {
        if i == 23 {
            destroy_body(&mut world, loose);
        }
        if i == 37 {
            let mut late_def = default_body_def();
            late_def.type_ = BodyType::Dynamic;
            late_def.position = Pos {
                x: 2.0 as _,
                y: 3.0 as _,
                z: 0.5 as _,
            };
            let late = create_body(&mut world, &late_def);
            create_sphere_shape(
                &mut world,
                late,
                &sd,
                &Sphere {
                    center: VEC3_ZERO,
                    radius: 0.3,
                },
            );
        }
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world).expect("recording active")
}

/// Hull boxes and spheres dropped onto a static grid mesh.
fn record_mesh_drop(total_frames: i32) -> Recording {
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );
    world_start_recording(&mut world, Recording::new(0));

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = Pos {
        x: -4.0 as _,
        y: 0.0 as _,
        z: -4.0 as _,
    };
    let ground_id = create_body(&mut world, &ground_def);
    let mesh = crate::mesh::create_grid_mesh(8, 8, 1.0, 1, false).expect("grid mesh");
    crate::shape::create_mesh_shape(
        &mut world,
        ground_id,
        &default_shape_def(),
        &mesh,
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
    );

    let mut sd = default_shape_def();
    sd.density = 1.0;
    let box_hull = make_box_hull(0.3, 0.3, 0.3);
    for i in 0..6 {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = Pos {
            x: (0.7 * (i % 3) as f32 - 0.7) as _,
            y: (1.0 + 0.8 * i as f32) as _,
            z: (0.4 * (i / 3) as f32) as _,
        };
        let body = create_body(&mut world, &bd);
        if i % 2 == 0 {
            create_hull_shape(&mut world, body, &sd, &box_hull.base);
        } else {
            create_sphere_shape(
                &mut world,
                body,
                &sd,
                &Sphere {
                    center: VEC3_ZERO,
                    radius: 0.3,
                },
            );
        }
    }

    for _ in 0..total_frames {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world).expect("recording active")
}

/// Everything a seek must reproduce about one frame: the world hash, the complete serialized
/// world image, and the outliner body list.
#[derive(Debug, PartialEq)]
struct FrameState {
    hash: u64,
    image: Vec<u8>,
    body_ids: Vec<crate::id::BodyId>,
}

fn frame_state(player: &crate::recording::RecPlayer) -> FrameState {
    use crate::recording::{hash_world_state, serialize_world, GeometryRegistry, RecBuffer};

    let mut buf = RecBuffer::new();
    let mut registry = GeometryRegistry::new();
    serialize_world(player.world(), &mut buf, &mut registry);
    FrameState {
        hash: hash_world_state(player.world()),
        image: buf.data,
        body_ids: (0..player.get_body_count())
            .map(|i| player.get_body_id(i))
            .collect(),
    }
}

/// Reference states for every frame from a single restart-and-replay pass on a player that never
/// captures a keyframe.
fn reference_states(data: &[u8]) -> Vec<FrameState> {
    use crate::recording::RecPlayer;

    let mut player = RecPlayer::create(data, 1).expect("reference player");
    player.set_keyframe_policy(0, i32::MAX);
    let mut states = vec![frame_state(&player)];
    while player.step_frame() {
        assert_eq!(player.get_frame() as usize, states.len());
        states.push(frame_state(&player));
    }
    assert!(!player.has_diverged());
    assert!(player.get_keyframe_frames().is_empty());
    states
}

/// Seeking through the keyframe ring must land on exactly the world a fresh restart-and-replay
/// reaches: bit-identical serialized image, world hash, and outliner list, for forward,
/// backward, repeated, out-of-range, and pseudo-random seeks, under generous, tight, and
/// mid-sized budgets. After every seek one more step must also match, proving the restored
/// op-stream cursor.
#[test]
fn keyframe_seek_matches_restart_and_replay() {
    use crate::recording::RecPlayer;

    let recordings = [
        ("box stack", record_box_stack_scene(80, false)),
        ("seeded box stack", record_box_stack_scene(80, true)),
        ("compound", record_compound_drop(48)),
        ("joints", record_joint_chain(60)),
        ("mesh", record_mesh_drop(60)),
    ];

    for (name, rec) in &recordings {
        let reference = reference_states(rec.data());
        let frame_count = reference.len() as i32 - 1;

        // The first keyframe's charge at a 4 frame interval sizes the mid budget.
        let mut probe = RecPlayer::create(rec.data(), 1).expect("probe");
        probe.set_keyframe_policy(0, 4);
        while probe.get_keyframe_frames().is_empty() && probe.step_frame() {}
        let k = probe.get_keyframe_bytes();
        assert!(k > 0, "{name}");
        drop(probe);

        let policies = [(0usize, 16), (1, 4), (1, 1), (3 * k, 4), (k * 5 / 2, 2)];

        let mut targets: Vec<i32> = (0..=frame_count).step_by(4).collect();
        targets.extend((0..=frame_count).rev().step_by(7));
        targets.extend([frame_count, frame_count, 7, 7, frame_count + 10, -4, 1, 0]);
        let mut seed = 0x2545_F491u32;
        for _ in 0..20 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            targets.push((seed >> 8) as i32 % (frame_count + 1));
        }

        for &(budget, interval) in &policies {
            let mut player = RecPlayer::create(rec.data(), 1).expect("player");
            // A zero budget keeps the 512 MiB default.
            player.set_keyframe_policy(budget, interval);

            for &target in &targets {
                player.seek_frame(target);
                let expected_frame = target.clamp(0, frame_count);
                assert_eq!(
                    player.get_frame(),
                    expected_frame,
                    "{name} budget {budget} interval {interval} target {target}"
                );
                assert!(!player.has_diverged(), "{name} target {target}");
                assert_eq!(player.get_diverge_frame(), -1);
                assert!(
                    frame_state(&player) == reference[expected_frame as usize],
                    "{name} budget {budget} interval {interval} seek {target}: state differs"
                );

                if expected_frame < frame_count {
                    assert!(player.step_frame());
                    let next = (expected_frame + 1) as usize;
                    assert!(
                        frame_state(&player) == reference[next],
                        "{name} budget {budget} interval {interval} step after seek {target}"
                    );
                }
            }

            // The ring respects the budget whenever it holds more than one keyframe (a single
            // keyframe is always stored, even over budget).
            let frames = player.get_keyframe_frames();
            assert!(!frames.is_empty(), "{name}");
            let effective_budget = player.get_keyframe_budget();
            if frames.len() > 1 {
                assert!(player.get_keyframe_bytes() <= effective_budget, "{name}");
            }
            for f in &frames {
                assert!(*f > 0 && *f <= frame_count, "{name}");
            }
        }
    }
}

/// A keyframe restore is a deterministic replay state, so shapes that persist must keep their
/// renderer handle rather than being torn down and rebuilt every seek. Record a snapshot-seeded
/// session (shapes exist at frame 0), replay with handle callbacks, scrub backward across keyframes
/// repeatedly, and verify no new handles are built per restore and the create/destroy counts still
/// balance at teardown. (KeyframeHandleReuse)
#[test]
fn keyframe_handle_reuse() {
    use crate::debug_draw::{DebugDraw, DebugShape, HexColor};
    use crate::dynamic_tree::DEFAULT_MASK_BITS;
    use crate::math_functions::{Aabb, WorldTransform};
    use crate::recording::RecPlayer;
    use crate::world::world_draw;
    use std::cell::Cell;

    struct DebugShapeCounters {
        created: Cell<i32>,
        destroyed: Cell<i32>,
    }

    fn create_cb(_shape: &DebugShape<'_>, ctx: u64) -> u64 {
        // SAFETY: `ctx` is the address of a `DebugShapeCounters` that outlives the player.
        let counters = unsafe { &*(ctx as *const DebugShapeCounters) };
        counters.created.set(counters.created.get() + 1);
        1
    }

    fn destroy_cb(_user_shape: u64, ctx: u64) {
        // SAFETY: see create_cb.
        let counters = unsafe { &*(ctx as *const DebugShapeCounters) };
        counters.destroyed.set(counters.destroyed.get() + 1);
    }

    struct NoDraw;

    impl DebugDraw for NoDraw {
        fn draw_shape(&mut self, _user_shape: u64, _transform: WorldTransform, _color: HexColor) {}

        fn drawing_bounds(&self) -> Aabb {
            let big = 1.0e6;
            Aabb {
                lower_bound: Vec3 {
                    x: -big,
                    y: -big,
                    z: -big,
                },
                upper_bound: Vec3 {
                    x: big,
                    y: big,
                    z: big,
                },
            }
        }

        fn draw_shapes(&self) -> bool {
            true
        }
    }

    let mut world = World::new(&default_world_def());
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
    let ground_box = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);

    let dynamic_count = 5;
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let mut box_shape = default_shape_def();
    box_shape.density = 1.0;
    for i in 0..dynamic_count {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 0.0,
            y: (1.0 + 1.1 * i as f32) as _,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &box_shape, &box_hull.base);
    }
    let shape_count = 1 + dynamic_count;

    // Settle, then record with a snapshot of the populated world so shapes exist at frame 0.
    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }
    world_start_recording(&mut world, Recording::new(0));
    let total_frames = 80;
    for _ in 0..total_frames {
        world.step(1.0 / 60.0, 4);
    }
    let rec = world_stop_recording(&mut world).expect("recording active");
    drop(world);

    let counters = DebugShapeCounters {
        created: Cell::new(0),
        destroyed: Cell::new(0),
    };
    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    player.set_debug_shape_callbacks(
        Some(create_cb),
        Some(destroy_cb),
        &counters as *const DebugShapeCounters as u64,
    );

    // Replay to the end and draw: one handle per shape.
    player.seek_frame(total_frames);
    world_draw(player.world_mut(), &mut NoDraw, DEFAULT_MASK_BITS);
    assert!(!player.has_diverged());
    assert_eq!(counters.created.get(), shape_count);
    assert_eq!(player.get_keyframe_frames(), vec![16, 32, 48, 64, 80]);

    // Scrub backward and forward across keyframes. Each restore keeps the persistent shapes'
    // handles, so drawing builds no new ones.
    let created_after_first_draw = counters.created.get();
    for target in [40, 70, 20, 60, 8, 75] {
        player.seek_frame(target);
        world_draw(player.world_mut(), &mut NoDraw, DEFAULT_MASK_BITS);
    }
    assert_eq!(counters.created.get(), created_after_first_draw);

    // Teardown releases exactly the live handles, so the leak-free invariant holds.
    drop(player);
    assert_eq!(counters.created.get(), counters.destroyed.get());
}
