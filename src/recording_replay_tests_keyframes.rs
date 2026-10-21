// Replay keyframe ring tests (b3RecCaptureKeyframe / b3RecPlayer_SeekFrame / keyframe policy).
// `include!`d into `recording_replay_tests.rs`, so it shares that module's imports (and the
// `rec_compound` / `record_all_geometry_ops` helpers of its sibling files).
//
// Expected keyframe frames, intervals, and world hashes below come from a scalar C build of
// efca0c1 (MSVC, BOX3D_DISABLE_SIMD, float and double precision) replaying the same scenes and
// reading `player->keyframes`. Keyframe byte counts are not compared with C: the port's images
// are its own field-by-field snapshot format, so they differ in size from C's struct images.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

/// Build the ScrubBackward scene: a static ground and a stack of four dynamic boxes. When
/// `seeded` is false recording starts first, so the bodies are created mid-stream; otherwise the
/// world settles for 10 steps and the recording is seeded from a snapshot (the PlayerAccessors
/// scene). Either way the recording covers `total_frames` steps.
fn record_box_stack_scene(total_frames: i32, seeded: bool) -> Recording {
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );
    if !seeded {
        world_start_recording(&mut world, Recording::new(0));
    }

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);

    let mut box_shape = default_shape_def();
    box_shape.density = 1.0;
    for i in 0..4 {
        let box_hull = make_box_hull(0.5, 0.5, 0.5);
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 0.0,
            y: (2.0 + i as f32 * 1.5) as _,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &box_shape, &box_hull.base);
    }

    if seeded {
        for _ in 0..10 {
            world.step(1.0 / 60.0, 4);
        }
        world_start_recording(&mut world, Recording::new(0));
    }

    for _ in 0..total_frames {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world).expect("recording active")
}

fn record_box_stack(total_frames: i32) -> Recording {
    record_box_stack_scene(total_frames, false)
}

/// One ring state: (frame, keyframe frames in ring order, keyframe interval).
type RingState = (i32, Vec<i32>, i32);

/// Play to the end, recording the ring each time a frame captured a keyframe.
fn trace_ring(player: &mut crate::recording::RecPlayer) -> Vec<RingState> {
    let mut states = Vec::new();
    let mut last = player.last_keyframe_frame;
    while player.step_frame() {
        if player.last_keyframe_frame != last {
            states.push((
                player.get_frame(),
                player.get_keyframe_frames(),
                player.get_keyframe_interval(),
            ));
            last = player.last_keyframe_frame;
        }
    }
    states
}

/// The player starts with upstream's default policy: a 16 frame interval and a 512 MiB budget.
/// The setter ignores a zero budget or a non-positive interval, keeping the current value.
/// (B3_REC_KEYFRAME_INTERVAL_DEFAULT, B3_REC_KEYFRAME_BUDGET_DEFAULT, b3RecPlayer_SetKeyframePolicy)
#[test]
fn keyframe_policy_defaults() {
    use crate::recording::RecPlayer;

    let rec = record_box_stack(20);
    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    assert_eq!(player.get_keyframe_min_interval(), 16);
    assert_eq!(player.get_keyframe_interval(), 16);
    assert_eq!(player.get_keyframe_budget(), 512 * 1024 * 1024);
    assert_eq!(player.get_keyframe_bytes(), 0);

    player.set_keyframe_policy(0, 0);
    assert_eq!(player.get_keyframe_min_interval(), 16);
    assert_eq!(player.get_keyframe_budget(), 512 * 1024 * 1024);

    player.set_keyframe_policy(1000, -3);
    assert_eq!(player.get_keyframe_min_interval(), 16);
    assert_eq!(player.get_keyframe_budget(), 1000);

    player.set_keyframe_policy(0, 5);
    assert_eq!(player.get_keyframe_min_interval(), 5);
    assert_eq!(player.get_keyframe_interval(), 5);
    assert_eq!(player.get_keyframe_budget(), 1000);
}

/// Stepping captures keyframes, and the setter drops the ring and resets the interval.
#[test]
fn keyframe_capture_and_policy_reset() {
    use crate::recording::RecPlayer;

    let rec = record_box_stack(40);
    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while player.step_frame() {}
    assert_eq!(player.get_frame(), 40);
    assert!(player.get_keyframe_bytes() > 0);
    assert_eq!(player.get_keyframe_frames(), vec![16, 32]);

    player.set_keyframe_policy(256 * 1024 * 1024, 8);
    assert_eq!(player.get_keyframe_bytes(), 0);
    assert_eq!(player.get_keyframe_interval(), 8);
    assert!(player.get_keyframe_frames().is_empty());
    assert_eq!(player.last_keyframe_frame, 0);

    // The ring repopulates under the new policy on the next replay.
    player.restart();
    while player.step_frame() {}
    assert_eq!(player.get_keyframe_frames(), vec![8, 16, 24, 32, 40]);
}

/// Each keyframe is charged its image allocation (C's b3RecBuffer growth rule applied to the
/// image) plus its outliner body list at sizeof( b3BodyId ) == 8 bytes per entry.
#[test]
fn keyframe_bytes_charge_image_capacity_and_body_list() {
    use crate::recording::{serialize_world, GeometryRegistry, RecBuffer, RecPlayer};

    assert_eq!(std::mem::size_of::<crate::id::BodyId>(), 8);

    let rec = record_box_stack(16);
    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while player.step_frame() {}
    assert_eq!(player.get_keyframe_frames(), vec![16]);
    assert_eq!(player.get_body_count(), 5);

    // Serialize the same frame-16 world the keyframe captured, on a player that captures nothing.
    let mut reference = RecPlayer::create(rec.data(), 1).expect("reference");
    reference.set_keyframe_policy(0, i32::MAX);
    reference.seek_frame(16);
    let mut buf = RecBuffer::new();
    let mut registry = GeometryRegistry::new();
    let size = serialize_world(reference.world(), &mut buf, &mut registry);
    assert!(buf.capacity >= size);

    assert_eq!(player.get_keyframe_bytes(), buf.capacity as usize + 5 * 8);
}

/// b3RecBufAppend grows to the larger of twice the capacity and size + n + 64.
#[test]
fn rec_buffer_capacity_follows_c_growth() {
    use crate::recording::RecBuffer;

    let mut buf = RecBuffer::new();
    buf.append(&[0u8; 16]);
    assert_eq!(buf.capacity, 80); // 0 * 2 < 16 + 64
    buf.append(&[0u8; 64]);
    assert_eq!(buf.capacity, 80); // fits exactly
    buf.append(&[0u8; 1]);
    assert_eq!(buf.capacity, 160); // 80 * 2 > 81 + 64
    buf.append(&[0u8; 300]);
    assert_eq!(buf.capacity, 445); // 81 + 300 + 64 > 320
    buf.append(&[]);
    assert_eq!(buf.capacity, 445); // empty appends are ignored

    // A preallocated buffer (b3CreateRecording) starts at its initial capacity.
    let mut buf = RecBuffer::with_capacity(100);
    assert_eq!(buf.capacity, 100);
    buf.append(&[0u8; 100]);
    assert_eq!(buf.capacity, 100);

    // Count-only buffers tally the size without allocating, so the capacity stays put.
    let mut buf = RecBuffer::new();
    buf.count_only = true;
    buf.append(&[0u8; 40]);
    assert_eq!(buf.size(), 40);
    assert_eq!(buf.capacity, 0);
}

/// Restart does not reset the last captured keyframe frame, so replaying the same frames again
/// does not capture duplicates: the ring is unchanged after a restart and a second full pass.
#[test]
fn keyframe_restart_does_not_recapture() {
    use crate::recording::RecPlayer;

    let rec = record_box_stack(80);
    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while player.step_frame() {}
    let bytes = player.get_keyframe_bytes();
    assert!(bytes > 0);
    let frames = player.get_keyframe_frames();
    assert_eq!(frames, vec![16, 32, 48, 64, 80]);

    player.restart();
    while player.step_frame() {}
    assert_eq!(player.get_keyframe_bytes(), bytes);
    assert_eq!(player.get_keyframe_frames(), frames);

    // Backward seeks land on keyframes and re-step frames that were already captured.
    player.seek_frame(20);
    player.seek_frame(80);
    assert_eq!(player.get_keyframe_bytes(), bytes);
    assert_eq!(player.get_keyframe_frames(), frames);
}

/// Seeking to the current frame is a no-op in C: neither a restore nor a restart happens, so the
/// per-frame query store of the current frame survives.
#[test]
fn seek_to_current_frame_keeps_state() {
    use crate::recording::RecPlayer;
    use crate::types::default_query_filter;
    use crate::world::world_cast_ray_closest;

    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, Recording::new(0));
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);
    for _ in 0..20 {
        world_cast_ray_closest(
            &world,
            Pos {
                x: 0.0,
                y: 5.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: -10.0,
                z: 0.0,
            },
            &default_query_filter(),
        );
        world.step(1.0 / 60.0, 4);
    }
    let rec = world_stop_recording(&mut world).expect("recording active");

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    player.seek_frame(10);
    assert_eq!(player.get_frame_query_count(), 1);
    player.seek_frame(10);
    assert_eq!(player.get_frame(), 10);
    assert_eq!(player.get_frame_query_count(), 1);
    assert!(player.step_frame());
    assert_eq!(player.get_frame(), 11);
}

/// b3InternGeometry walks the hash chain comparing byte counts and bytes only, never the kind. The
/// keyframe registry relies on this: it is seeded 1:1 from the reader slots (kind bytes included,
/// even ones no shape can resolve), and capture only needs a blob to map back to a slot index.
#[test]
fn intern_geometry_ignores_kind() {
    use crate::recording::{GeometryKind, GeometryRegistry};

    let mut reg = GeometryRegistry::new();
    let blob = vec![0x5Au8; 24];
    let id = reg.append(GeometryKind::Mesh, 0x1234, blob.clone());
    assert_eq!(id, 0);
    assert_eq!(reg.intern(GeometryKind::Hull, 0x1234, blob), 0);
    assert_eq!(reg.entries.len(), 1);
}

/// The keyframe registry mirrors the reader slots 1:1 (b3RecSeedKeyframeRegistry), and capturing
/// a keyframe while every geometry kind is live never grows it (the regCount assert in
/// b3RecCaptureKeyframe, a `debug_assert` here as well).
#[test]
fn keyframe_registry_mirrors_slots_and_never_grows() {
    use crate::recording::RecPlayer;

    let rec = record_all_geometry_ops();
    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    let slot_count = player.reader_slots.len();
    assert_eq!(slot_count, 6);
    assert_eq!(player.keyframe_registry.entries.len(), slot_count);
    for (i, (entry, slot)) in player
        .keyframe_registry
        .entries
        .iter()
        .zip(player.reader_slots.iter())
        .enumerate()
    {
        assert_eq!(entry.id, i as u32);
        assert_eq!(entry.kind as u8, slot.kind);
        assert_eq!(entry.bytes, slot.bytes);
    }

    // Capture a keyframe at every frame.
    player.set_keyframe_policy(0, 1);
    while player.step_frame() {}
    assert!(!player.has_diverged());
    assert_eq!(player.get_keyframe_frames(), vec![1, 2]);
    assert_eq!(player.keyframe_registry.entries.len(), slot_count);

    // Both keyframes restore and replay cleanly.
    player.seek_frame(1);
    assert_eq!(player.get_frame(), 1);
    player.seek_frame(2);
    assert_eq!(player.get_frame(), 2);
    assert!(!player.has_diverged());
}

/// The default policy captures every 16 frames, including the last frame, and never evicts under
/// the 512 MiB budget. Seeks through that ring land on the same world states as C. Matches the
/// scalar C build in both precisions.
#[test]
fn keyframe_default_policy_matches_c() {
    use crate::recording::{hash_world_state, RecPlayer};

    #[cfg(not(feature = "double-precision"))]
    let expected_hashes: [[u64; 10]; 2] = [
        [
            0x6f1181bbc43b0eb3,
            0xba945942b7e7f09f,
            0x3310b6638e4b48a7,
            0x56c0074e059dae93,
            0xcbf29ce484222325,
            0x65d7a4559c4d1dd1,
            0x706c9764b7f60edb,
            0x23f7c3c2a776920a,
            0xc5b2129ed9c3ba9c,
            0x2176affa5dbf446a,
        ],
        [
            0x5cedd4e8fc68496a,
            0x83d4b852a0471b86,
            0x29f08d05f9b0279c,
            0xedef15cba83f9953,
            0x6f1181bbc43b0eb3,
            0xba945942b7e7f09f,
            0x8ec0b425284c563b,
            0x077aadb3b5c99630,
            0xc8e8a6e1937b9b7f,
            0xc08c70079467b213,
        ],
    ];
    #[cfg(feature = "double-precision")]
    let expected_hashes: [[u64; 10]; 2] = [
        [
            0xd9c074a20fff909b,
            0x16ea9682f16f5535,
            0x9764a3f1f09a45d7,
            0x71180038e521be1b,
            0xcbf29ce484222325,
            0x30cdad43b2842a17,
            0xdb6f67b8611eed35,
            0x18759ca49e1aef6a,
            0x2598c184ed12a44d,
            0xbf5d986e559694d7,
        ],
        [
            0x10b0b676ca313a2d,
            0x9388abed015a225b,
            0x0a5ec1206192cb17,
            0x429df16190e67ba4,
            0xd9c074a20fff909b,
            0x16ea9682f16f5535,
            0x8231e1ad8b57e396,
            0xc856d16e812dbb11,
            0x244e02a388d6148d,
            0x753f4f93a5c42012,
        ],
    ];

    for (scene, seeded) in [false, true].into_iter().enumerate() {
        let rec = record_box_stack_scene(80, seeded);
        let mut player = RecPlayer::create(rec.data(), 1).expect("player");
        let trace = trace_ring(&mut player);
        let expected: Vec<RingState> = vec![
            (16, vec![16], 16),
            (32, vec![16, 32], 16),
            (48, vec![16, 32, 48], 16),
            (64, vec![16, 32, 48, 64], 16),
            (80, vec![16, 32, 48, 64, 80], 16),
        ];
        assert_eq!(trace, expected, "seeded {seeded}");

        let targets = [10, 50, 3, 79, 0, 40, 80, 25, 77, 64];
        for (k, &target) in targets.iter().enumerate() {
            player.seek_frame(target);
            assert_eq!(player.get_frame(), target);
            assert!(!player.has_diverged());
            assert_eq!(
                hash_world_state(player.world()),
                expected_hashes[scene][k],
                "seeded {seeded} target {target}"
            );
            assert_eq!(player.get_keyframe_frames(), vec![16, 32, 48, 64, 80]);
        }
    }
}

/// A one byte budget keeps a single keyframe: every capture doubles the interval until the old
/// keyframe falls off the grid and is evicted, then the new one is stored anyway. Seeks restore the
/// survivor or restart. The ring states and the world hashes after each seek match the scalar C
/// build in both precisions.
#[test]
fn keyframe_one_byte_budget_matches_c() {
    use crate::recording::{hash_world_state, RecPlayer};

    #[cfg(not(feature = "double-precision"))]
    let expected_hashes: [[u64; 8]; 2] = [
        [
            0x25b2ea1e5f3ca0ba,
            0x6f1181bbc43b0eb3,
            0xf71a12fe56f8f551,
            0x2176affa5dbf446a,
            0x806a64e44aa6bd64,
            0x706c9764b7f60edb,
            0xcbf29ce484222325,
            0x9d4b798618662b60,
        ],
        [
            0x706c9764b7f60edb,
            0x5cedd4e8fc68496a,
            0xbc9aa6c1e07d5a10,
            0xc08c70079467b213,
            0xf8d5f68cb6b4e02c,
            0x8ec0b425284c563b,
            0x6f1181bbc43b0eb3,
            0xa59affae405a3e32,
        ],
    ];
    #[cfg(feature = "double-precision")]
    let expected_hashes: [[u64; 8]; 2] = [
        [
            0xf599b02d03f2ed8e,
            0xd9c074a20fff909b,
            0xed6a9d8cc711c3c3,
            0xbf5d986e559694d7,
            0x2bca045cc07a4fb3,
            0xdb6f67b8611eed35,
            0xcbf29ce484222325,
            0xe2cecb34739e232c,
        ],
        [
            0xdb6f67b8611eed35,
            0x10b0b676ca313a2d,
            0x017896743f7d3e43,
            0x753f4f93a5c42012,
            0x4b72b9525bfb8a7e,
            0x8231e1ad8b57e396,
            0xd9c074a20fff909b,
            0x4df9f8c6a0466f29,
        ],
    ];

    for (scene, seeded) in [false, true].into_iter().enumerate() {
        let rec = record_box_stack_scene(80, seeded);
        let mut player = RecPlayer::create(rec.data(), 1).expect("player");
        player.set_keyframe_policy(1, 4);
        let trace = trace_ring(&mut player);
        let expected: Vec<RingState> = vec![
            (4, vec![4], 4),
            (8, vec![8], 8),
            (16, vec![16], 16),
            (32, vec![32], 32),
            (64, vec![64], 64),
        ];
        assert_eq!(trace, expected, "seeded {seeded}");

        let targets = [70, 10, 66, 64, 63, 80, 0, 65];
        for (k, &target) in targets.iter().enumerate() {
            player.seek_frame(target);
            assert_eq!(player.get_frame(), target);
            assert!(!player.has_diverged());
            assert_eq!(
                hash_world_state(player.world()),
                expected_hashes[scene][k],
                "seeded {seeded} target {target}"
            );
            assert_eq!(player.get_keyframe_frames(), vec![64]);
            assert_eq!(player.get_keyframe_interval(), 64);
        }
    }
}

/// Budgets of 2.5 and 5.5 times the first keyframe's charge, with a 4 frame interval: the ring
/// fills, then the interval doubles and off-grid keyframes are evicted. Matches the scalar C
/// float build.
///
/// These decisions depend on how each keyframe's charge compares with the first one's. In float,
/// C charges 12400 bytes for the frame-4 keyframe and 12400 to 12496 for later ones, and the port
/// charges 8744 for every keyframe, so the ratios agree for these budgets and the rings match. In
/// double precision C's frame-4 image fits a 7060 byte allocation while later images need 14152 or
/// more, so C's ring under a budget relative to the first keyframe differs from the port's, whose
/// charge stays flat; the comparison is float-only for that reason.
#[cfg(not(feature = "double-precision"))]
#[test]
fn keyframe_relative_budget_matches_c_float() {
    use crate::recording::RecPlayer;

    for seeded in [false, true] {
        let rec = record_box_stack_scene(80, seeded);

        // Probe the first keyframe's charge at a 4 frame interval.
        let mut probe = RecPlayer::create(rec.data(), 1).expect("probe");
        probe.set_keyframe_policy(0, 4);
        while probe.get_keyframe_frames().is_empty() && probe.step_frame() {}
        let k = probe.get_keyframe_bytes();
        drop(probe);

        let mut player = RecPlayer::create(rec.data(), 1).expect("player");
        player.set_keyframe_policy(k * 5 / 2, 4);
        let expected: Vec<RingState> = vec![
            (4, vec![4], 4),
            (8, vec![4, 8], 4),
            (12, vec![8, 12], 8),
            (16, vec![16], 16),
            (32, vec![16, 32], 16),
            (48, vec![32, 48], 32),
            (64, vec![64], 64),
        ];
        assert_eq!(trace_ring(&mut player), expected, "seeded {seeded} K*5/2");
        drop(player);

        let mut player = RecPlayer::create(rec.data(), 1).expect("player");
        player.set_keyframe_policy(k * 11 / 2, 4);
        let expected: Vec<RingState> = vec![
            (4, vec![4], 4),
            (8, vec![4, 8], 4),
            (12, vec![4, 8, 12], 4),
            (16, vec![4, 8, 12, 16], 4),
            (20, vec![4, 8, 12, 16, 20], 4),
            (24, vec![8, 16, 24], 8),
            (32, vec![8, 16, 24, 32], 8),
            (40, vec![8, 16, 24, 32, 40], 8),
            (48, vec![16, 32, 48], 16),
            (64, vec![16, 32, 48, 64], 16),
            (80, vec![16, 32, 48, 64, 80], 16),
        ];
        assert_eq!(trace_ring(&mut player), expected, "seeded {seeded} K*11/2");
    }
}
