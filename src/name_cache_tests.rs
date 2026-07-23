//! Name cache tests ported from test_name_cache.c.
//!
//! The C suite guards against leaks with `b3GetByteCount()`; the Rust port owns
//! every interned name through `String`/`Vec`, so those buffers are freed on
//! drop and the byte-count assertions have no Rust analog. The functional
//! behavior (dedup, reverse lookup, empty handling, arbitrary length, and
//! survival through recording snapshots and keyframe rollback) is ported in
//! full.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{body_get_name, body_get_shapes, body_is_valid, create_body};
use crate::hull::make_box_hull;
use crate::math_functions::{Pos, Vec3};
use crate::name_cache::NameCache;
use crate::recording::{validate_replay, RecPlayer, Recording};
use crate::shape::{create_hull_shape, shape_get_name};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::{world_set_gravity, world_start_recording, world_stop_recording, World};

// Direct unit test of the cache: dedup, reverse lookup, empty, arbitrary length. (CacheUnit)
#[test]
fn cache_unit() {
    let mut cache = NameCache::new();

    // Empty maps to the null id, which resolves to no string.
    assert_eq!(cache.add_name(""), NameCache::NULL);
    assert_eq!(cache.find_name(NameCache::NULL), None);

    // Distinct strings get distinct ids and round-trip.
    let crate_id = cache.add_name("crate");
    let barrel = cache.add_name("barrel");
    assert!(crate_id != NameCache::NULL && barrel != NameCache::NULL && crate_id != barrel);
    assert_eq!(cache.find_name(crate_id), Some("crate"));
    assert_eq!(cache.find_name(barrel), Some("barrel"));

    // Re-interning the same bytes returns the same id and stores no second entry.
    assert_eq!(cache.add_name("crate"), crate_id);
    assert_eq!(cache.entries.len(), 2);

    // A name far longer than any inline buffer round-trips exactly.
    let long_name: String = (0..4095).map(|i| (b'a' + (i % 26) as u8) as char).collect();
    let long_id = cache.add_name(&long_name);
    assert_eq!(cache.find_name(long_id), Some(long_name.as_str()));
    assert_eq!(cache.entries.len(), 3);
}

// Build a ground plus named dynamic boxes. Two share a name to exercise dedup and one is longer
// than the old inline buffer to exercise variable length. Ground is ordinal 0, boxes 1..N.
const BODY_NAMES: [&str; 4] = [
    "crate",
    "barrel",
    "crate",
    "a_very_long_body_name_that_exceeds_any_inline_name_buffer",
];

// Each box also carries a named hull shape. Shapes could not hold a name at all before the cache.
// Two shapes share a name for dedup, one shape shares a body's name to prove shape ids intern
// independently, and one is far longer than any inline buffer.
const SHAPE_NAMES: [&str; 4] = [
    "box_hull",
    "box_hull",
    "crate",
    "a_very_long_shape_name_that_never_fit_the_old_inline_buffer",
];

fn build_named_scene(world: &mut World) {
    world_set_gravity(
        world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(world, &ground_def);
    let ground_box = make_box_hull(20.0, 1.0, 20.0);
    let ground_shape = default_shape_def();
    create_hull_shape(world, ground_id, &ground_shape, &ground_box.base);

    for i in 0..BODY_NAMES.len() {
        let box_hull = make_box_hull(0.5, 0.5, 0.5);
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        // C computes the height in float then assigns to b3Pos, which widens in
        // large-world mode; mirror that with a trailing coercion.
        body_def.position = Pos {
            x: 0.0,
            y: (1.0 + 1.1 * i as f32) as _,
            z: 0.0,
        };
        body_def.name = BODY_NAMES[i].to_string();
        let body_id = create_body(world, &body_def);

        let mut box_shape = default_shape_def();
        box_shape.density = 1.0;
        box_shape.name = SHAPE_NAMES[i].to_string();
        create_hull_shape(world, body_id, &box_shape, &box_hull.base);
    }
}

fn check_replay_names(player: &RecPlayer) {
    for i in 0..BODY_NAMES.len() {
        let id = player.get_body_id(1 + i as i32); // ordinal 0 is the ground
        assert!(body_is_valid(player.world(), id));
        let name = body_get_name(player.world(), id);
        assert_eq!(name, BODY_NAMES[i]);

        // Shape names ride the same world cache, so the snapshot must restore them too.
        let shapes = body_get_shapes(player.world(), id, 1);
        assert_eq!(shapes.len(), 1);
        let shape_name = shape_get_name(player.world(), shapes[0]);
        assert_eq!(shape_name, SHAPE_NAMES[i]);
    }
}

// Names live on the world, so the world snapshot must carry them. Record a snapshot-seeded session
// and confirm every body and shape name resolves on the reconstructed replay world. (NameRoundTrip)
#[test]
fn name_round_trip() {
    let mut rec = Recording::new(0);

    let mut world = World::new(&default_world_def());
    build_named_scene(&mut world);

    // Record from a snapshot of the populated world, so names ride in the frame-0 image.
    world_start_recording(&mut world, &mut rec);
    for _ in 0..20 {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world);
    drop(world);

    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    player.seek_frame(20);
    assert!(!player.has_diverged());
    check_replay_names(&player);
}

// Rollback preserves names: a backward seek restores from a keyframe, which rebuilds world->names
// on each restore. Scrub across keyframe boundaries and confirm names resolve every time.
// (RollbackNames)
#[test]
fn rollback_names() {
    let mut rec = Recording::new(0);

    let mut world = World::new(&default_world_def());
    build_named_scene(&mut world);

    // Settle, then record a snapshot-seeded session long enough to span several keyframes.
    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }
    world_start_recording(&mut world, &mut rec);
    let total_frames = 80;
    for _ in 0..total_frames {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world);
    drop(world);

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");

    // Play to the end so the keyframe ring is populated.
    player.seek_frame(total_frames);
    assert!(!player.has_diverged());

    // Scrub backward and forward across keyframe boundaries. Each restore rebuilds the name table.
    let targets = [10, 60, 5, 70, 1, 40];
    for &target in &targets {
        player.seek_frame(target);
        assert_eq!(player.get_frame(), target);
        assert!(!player.has_diverged());
        check_replay_names(&player);
    }
}
