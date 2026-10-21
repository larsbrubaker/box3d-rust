// Per-frame query store lifetime tests for the recording player: when b3RecPlayer_StepFrame and
// b3RecPlayer_Restart reset the stashed queries, and what Restart leaves untouched on the reader.
// `include!`d into `recording_replay_tests.rs`, so it shares that module's imports.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

/// Ground plus one falling sphere; every frame issues one untagged AABB overlap before stepping.
/// When `tag_last_frame` is set, the final frame also issues a ray cast tagged with `tag_filter`.
fn record_frame_query_scene(frame_count: i32, tag_last_frame: bool) -> Recording {
    use crate::math_functions::Aabb;
    use crate::types::default_query_filter;
    use crate::world::{world_cast_ray, world_overlap_aabb};

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

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: 0.0,
        y: 3.0 as _,
        z: 0.0,
    };
    let ball_id = create_body(&mut world, &ball_def);
    let mut sd = default_shape_def();
    sd.density = 1.0;
    create_sphere_shape(
        &mut world,
        ball_id,
        &sd,
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
    );

    world_start_recording(&mut world, Recording::new(0));

    let untagged = default_query_filter();
    let mut tagged = default_query_filter();
    tagged.id = 77;
    tagged.name = "probe".into();
    let aabb = Aabb {
        lower_bound: Vec3 {
            x: -5.0,
            y: -1.0,
            z: -5.0,
        },
        upper_bound: Vec3 {
            x: 5.0,
            y: 6.0,
            z: 5.0,
        },
    };

    for i in 0..frame_count {
        world_overlap_aabb(&world, aabb, &untagged, |_| true);
        if tag_last_frame && i == frame_count - 1 {
            world_cast_ray(
                &world,
                Pos {
                    x: 0.0,
                    y: 6.0 as _,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: -8.0,
                    z: 0.0,
                },
                &tagged,
                |_id, _p, _n, fraction, _m, _t, _c| fraction,
            );
        }
        world.step(1.0 / 60.0, 4);
    }

    world_stop_recording(&mut world).expect("recording active")
}

/// Restart rewinds to the pre-step frame-0 snapshot, which has no recorded queries, so the
/// per-frame query store must be empty afterward (b3RecPlayer_Restart clears frameQueryCount).
#[test]
fn restart_clears_frame_queries() {
    use crate::recording::RecPlayer;

    let rec = record_frame_query_scene(10, false);
    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    player.seek_frame(5);
    assert_eq!(player.get_frame(), 5);
    assert_eq!(player.get_frame_query_count(), 1);

    // No keyframe precedes frame 5 at the default interval, so a backward seek to 0 restarts.
    player.seek_frame(0);
    assert_eq!(player.get_frame(), 0);
    assert_eq!(player.get_frame_query_count(), 0);

    player.seek_frame(3);
    assert_eq!(player.get_frame_query_count(), 1);
    player.restart();
    assert_eq!(player.get_frame(), 0);
    assert_eq!(player.get_frame_query_count(), 0);
}

/// Cut the op stream right after the final frame's StateHash, dropping the trailing
/// RecordingBounds and DestroyWorld records, and move the registry up behind it. The step that
/// advances the last frame then reaches the end of the stream and latches atEnd itself, so the
/// last frame's queries are still in the per-frame store when atEnd becomes true.
fn cut_stream_after_last_state_hash(data: &[u8]) -> Vec<u8> {
    use crate::recording::ops::RecOp;
    use crate::recording::session::RecHeader;

    let hdr = RecHeader::from_bytes(data).expect("header");
    let header_end = 48 + hdr.snapshot_size as usize;
    let registry_offset = hdr.registry_offset as usize;
    assert!(registry_offset > header_end);

    let mut cursor = header_end;
    let mut cut = 0usize;
    while cursor + 4 <= registry_offset {
        let opcode = data[cursor];
        let payload = data[cursor + 1] as usize
            | ((data[cursor + 2] as usize) << 8)
            | ((data[cursor + 3] as usize) << 16);
        cursor += 4 + payload;
        if opcode == RecOp::StateHash as u8 {
            cut = cursor;
        }
    }
    assert!(cut > header_end);

    let mut out = data[..cut].to_vec();
    out.extend_from_slice(&data[registry_offset..]);
    out[32..40].copy_from_slice(&(cut as u64).to_le_bytes());
    out
}

/// StepFrame at the end of the recording returns before resetting the per-frame store, so the
/// last frame's queries survive further step attempts (b3RecPlayer_StepFrame checks atEnd first).
#[test]
fn step_frame_at_end_keeps_last_frame_queries() {
    use crate::recording::RecPlayer;

    let rec = record_frame_query_scene(6, true);
    let data = cut_stream_after_last_state_hash(rec.data());

    let mut player = RecPlayer::create(&data, 1).expect("player");
    assert_eq!(player.get_frame_count(), 6);
    for _ in 0..6 {
        assert!(player.step_frame());
    }
    assert!(player.is_at_end());
    assert!(!player.has_diverged());
    assert_eq!(player.get_frame(), 6);
    let count = player.get_frame_query_count();
    assert_eq!(count, 2);

    assert!(!player.step_frame());
    assert_eq!(player.get_frame_query_count(), count);

    // SubStepFrame shares the ordering: at the end it returns before touching the store.
    player.sub_step_frame();
    assert_eq!(player.get_frame_query_count(), count);
}

/// b3RecPlayer_Restart does not reset rdr.pendingQueryKey. A QueryTag left dangling (its query op
/// was never dispatched) therefore pairs with the first query stashed after a restart.
#[test]
fn restart_keeps_pending_query_key() {
    use crate::recording::ops::RecOp;
    use crate::recording::RecPlayer;

    let rec = record_frame_query_scene(4, true);
    let key = Recording::hash_query_tag(77, "probe");
    assert_ne!(key, 0);

    // Rewrite the opcode of the op following the only QueryTag to an unassigned opcode. The
    // dispatcher skips it, so the tag's key stays pending when the recording ends.
    let mut data = rec.data().to_vec();
    let (header_end, registry_end) = {
        let probe = RecPlayer::create(&data, 1).expect("player");
        (probe.header_end as usize, probe.registry_end as usize)
    };
    let mut cursor = header_end;
    let mut patched = false;
    while cursor + 4 <= registry_end {
        let opcode = data[cursor];
        let payload = data[cursor + 1] as usize
            | ((data[cursor + 2] as usize) << 8)
            | ((data[cursor + 3] as usize) << 16);
        let next = cursor + 4 + payload;
        if opcode == RecOp::QueryTag as u8 {
            assert_eq!(data[next], RecOp::QueryCastRay as u8);
            assert!(RecOp::from_u8(0xF0).is_none());
            data[next] = 0xF0;
            patched = true;
            break;
        }
        cursor = next;
    }
    assert!(patched);

    let mut player = RecPlayer::create(&data, 1).expect("player");
    while player.step_frame() {}
    assert!(player.is_at_end());
    assert_eq!(player.get_frame(), 4);

    player.restart();
    assert!(player.step_frame());
    assert_eq!(player.get_frame(), 1);
    assert_eq!(player.get_frame_query_count(), 1);
    assert_eq!(player.get_frame_query(0).unwrap().key, key);
}
