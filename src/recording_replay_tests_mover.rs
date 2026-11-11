// Recording tests for the dynamic mover joints (upstream 3362633, "Dynamic Mover").
// `include!`d into `recording_replay_tests.rs`, so it shares that module's imports.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::id::{BodyId, JointId};
use crate::joint::{
    create_mover_joint, create_pogo_joint, destroy_joint, mover_joint_get_linear_velocity,
    mover_joint_get_max_velocity_force, mover_joint_set_linear_velocity,
    mover_joint_set_max_velocity_force, pogo_joint_get_current_speed, pogo_joint_get_impulse,
    pogo_joint_set_max_compression_force, pogo_joint_set_max_tension_force,
    pogo_joint_set_normal, pogo_joint_set_rest_length, pogo_joint_set_spring_damping_ratio,
    pogo_joint_set_spring_hertz,
};
use crate::types::{default_mover_joint_def, default_pogo_joint_def, MotionLocks, PogoJointDef};

/// The dynamic mover from the C `AllOps` session: a rotation locked capsule driven by a mover
/// joint and held up by a pogo joint. Returns (ground, mover body, mover joint, pogo joint, the
/// pogo def the mover rebuilds from).
fn create_dynamic_mover(world: &mut World) -> (BodyId, BodyId, JointId, JointId, PogoJointDef) {
    // Static ground with a box-hull shape
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(world, &ground_def);
    let ground_box = make_box_hull(50.0, 1.0, 50.0);
    create_hull_shape(world, ground_id, &default_shape_def(), &ground_box.base);

    // Dynamic mover: a rotation locked capsule driven by a mover joint and held up by a pogo joint
    let mut mover_body_def = default_body_def();
    mover_body_def.type_ = BodyType::Dynamic;
    mover_body_def.position = Pos {
        x: -20.0 as _,
        y: 3.0 as _,
        z: 0.0 as _,
    };
    mover_body_def.motion_locks = MotionLocks {
        angular_x: true,
        angular_y: true,
        angular_z: true,
        ..MotionLocks::default()
    };
    let mover_body_id = create_body(world, &mover_body_def);
    let mut mover_shape_def = default_shape_def();
    mover_shape_def.density = 1.0;
    mover_shape_def.base_material.friction = 0.0;
    let mover_capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -0.5,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.5,
            z: 0.0,
        },
        radius: 0.3,
    };
    crate::shape::create_capsule_shape(world, mover_body_id, &mover_shape_def, &mover_capsule);

    let mut mover_def = default_mover_joint_def();
    mover_def.base.body_id_a = ground_id;
    mover_def.base.body_id_b = mover_body_id;
    mover_def.base.collide_connected = true;
    mover_def.linear_velocity = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.5,
    };
    mover_def.max_force = 0.5;
    let mover_joint_id = create_mover_joint(world, &mover_def);

    let mut pogo_def = default_pogo_joint_def();
    pogo_def.base.body_id_a = ground_id;
    pogo_def.base.body_id_b = mover_body_id;
    pogo_def.base.local_frame_a.p = Vec3 {
        x: -20.0,
        y: 1.0,
        z: 0.0,
    };
    pogo_def.base.local_frame_b.p = mover_capsule.center1;
    pogo_def.base.collide_connected = true;
    pogo_def.rest_length = 1.0;
    pogo_def.hertz = 5.0;
    pogo_def.damping_ratio = 0.8;
    pogo_def.max_compression_force = 1000.0;
    pogo_def.max_tension_force = 1000.0;
    let pogo_joint_id = create_pogo_joint(world, &pogo_def);
    pogo_joint_set_rest_length(world, pogo_joint_id, 0.9);
    pogo_joint_set_spring_hertz(world, pogo_joint_id, 4.0);
    pogo_joint_set_spring_damping_ratio(world, pogo_joint_id, 0.7);
    pogo_joint_set_normal(
        world,
        pogo_joint_id,
        crate::math_functions::normalize(Vec3 {
            x: 0.2,
            y: 1.0,
            z: 0.0,
        }),
    );
    pogo_joint_set_max_tension_force(world, pogo_joint_id, 800.0);
    pogo_joint_set_max_compression_force(world, pogo_joint_id, 900.0);

    (
        ground_id,
        mover_body_id,
        mover_joint_id,
        pogo_joint_id,
        pogo_def,
    )
}

/// The dynamic mover additions to the C `AllOps` session (the Rust port has no single `AllOps`
/// test): the mover and pogo creates (0x81, 0x84), every pogo setter (0x85-0x8A), the mover
/// setters (0x82, 0x83) injected from frame 6, and from frame 6 on the pogo rebuilt every step,
/// warm started from the old one. Replay must reproduce the state hashes, and the 4 frame
/// keyframe interval C's `AllOps` sets makes the player snapshot and restore both joint types
/// across the backward and forward seeks.
#[test]
fn dynamic_mover_round_trip() {
    use crate::body::{body_get_local_point, body_get_position};
    use crate::recording::RecPlayer;

    let rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, rec);

    let (ground_id, mover_body_id, mover_joint_id, mut pogo_joint_id, mut pogo_def) =
        create_dynamic_mover(&mut world);

    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -9.8,
            z: 0.0,
        },
    );

    let time_step = 1.0 / 60.0;
    let sub_step_count = 4;
    for i in 0..12 {
        // Inject mutators mid-simulation
        if i == 6 {
            mover_joint_set_max_velocity_force(&mut world, mover_joint_id, 60.0);
        }

        // The dynamic mover rebuilds its pogo every step, warm started from the old one
        if i >= 6 {
            pogo_def.impulse = pogo_joint_get_impulse(&world, pogo_joint_id);
            pogo_def.velocity = pogo_joint_get_current_speed(&world, pogo_joint_id);
            pogo_def.base.local_frame_a.p = body_get_local_point(
                &world,
                ground_id,
                body_get_position(&world, mover_body_id),
            );
            pogo_def.base.local_frame_a.p.y = 1.0;
            destroy_joint(&mut world, pogo_joint_id, false);
            pogo_joint_id = create_pogo_joint(&mut world, &pogo_def);
            mover_joint_set_linear_velocity(
                &mut world,
                mover_joint_id,
                Vec3 {
                    x: -0.5,
                    y: 0.0,
                    z: 1.0,
                },
            );
        }

        world.step(time_step, sub_step_count);
    }

    let recorded_hash = crate::recording::hash::hash_world_state(&world);
    let rec = world_stop_recording(&mut world).expect("recording active");
    drop(world);

    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");

    // Capture keyframes while the mover and pogo joints are live. The default interval is longer
    // than this recording.
    let budget = player.get_keyframe_budget();
    player.set_keyframe_policy(budget, 4);

    let mut frames = 0;
    while player.step_frame() {
        frames += 1;
    }
    assert_eq!(frames, 12);
    assert!(player.is_at_end());
    assert!(!player.has_diverged());
    assert!(player.get_keyframe_bytes() > 0);
    assert_eq!(
        crate::recording::hash::hash_world_state(player.world()),
        recorded_hash
    );

    // Seek back onto the frame 4 keyframe, then skip forward onto the frame 8 keyframe.
    for target in [6, 9] {
        player.seek_frame(target);
        assert_eq!(player.get_frame(), target);
        assert!(!player.has_diverged());
    }

    while player.step_frame() {}
    assert!(!player.has_diverged());
    assert_eq!(
        crate::recording::hash::hash_world_state(player.world()),
        recorded_hash
    );

    let replay = player.world();
    let velocity = mover_joint_get_linear_velocity(replay, mover_joint_id);
    assert!(velocity.x == -0.5 && velocity.y == 0.0 && velocity.z == 1.0);
    assert_eq!(mover_joint_get_max_velocity_force(replay, mover_joint_id), 60.0);
}

/// Joint debug draw for the mover and pogo joints (b3DrawMoverJoint, b3DrawPogoJoint). The
/// generic joint-extras force segment and label are skipped for both types (b3DrawJoint); the
/// mover draws a 32 segment ring plus a force arrow, and the pogo draws a 12 segment spring, two
/// ground ticks, the foot segment, the normal, the anchor point, and its extras label.
#[test]
fn dynamic_mover_joint_draw() {
    use crate::debug_draw::{DebugDraw, HexColor};
    use crate::dynamic_tree::DEFAULT_MASK_BITS;
    use crate::math_functions::Aabb;
    use crate::world::world_draw;

    #[derive(Default)]
    struct JointDraw {
        segments: usize,
        orange_segments: usize,
        points: usize,
        strings: Vec<String>,
    }

    impl DebugDraw for JointDraw {
        fn draw_segment(&mut self, _p1: Pos, _p2: Pos, color: HexColor) {
            self.segments += 1;
            if color == HexColor::ORANGE {
                self.orange_segments += 1;
            }
        }
        fn draw_point(&mut self, _p: Pos, _size: f32, _color: HexColor) {
            self.points += 1;
        }
        fn draw_string(&mut self, _p: Pos, s: &str, _color: HexColor) {
            self.strings.push(s.to_string());
        }
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
        fn draw_joints(&self) -> bool {
            true
        }
        fn draw_joint_extras(&self) -> bool {
            true
        }
    }

    let mut world = World::new(&default_world_def());
    let (_ground_id, _mover_body_id, _mover_joint_id, _pogo_joint_id, _pogo_def) =
        create_dynamic_mover(&mut world);

    // The mover pushes the capsule along +x, away from the pogo anchor, so the foot segment
    // from the anchor to the pogo axis is drawn.
    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }

    let mut draw = JointDraw::default();
    world_draw(&mut world, &mut draw, DEFAULT_MASK_BITS);

    // Mover: 32 ring segments and a 3 segment arrow. Pogo: 12 spring segments, 2 ground ticks,
    // the foot segment, and the orange normal.
    assert_eq!(draw.segments, 32 + 3 + 12 + 2 + 1 + 1);
    assert_eq!(draw.orange_segments, 1);
    assert_eq!(draw.points, 1);
    assert_eq!(draw.strings.len(), 1);
    assert!(draw.strings[0].starts_with("length = "));
}
