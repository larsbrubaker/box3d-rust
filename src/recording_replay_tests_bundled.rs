// Load test for the recording bundled with the demo Replay viewer
// (`demo/public/recordings/sample.b3rec`). `include!`d into `recording_replay_tests.rs`, so it
// shares that module's imports.
//
// The file goes stale whenever the recording or snapshot format changes (REC_VERSION_*,
// SNAP_VERSION, SNAP_LAYOUT_VERSION, HULL_VERSION, ...): RecPlayer::create then rejects it and the
// demo's Replay page fails to load its default recording. When this test fails, regenerate the
// file with `bun run record:sample` in `demo/` (see `demo/scripts/record-sample.ts`).
//
// The file is read at runtime rather than with `include_bytes!` because `demo/` is excluded from
// the published crate; `include_bytes!` would break `cargo publish` verification.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

/// Read the bundled demo recording, or `None` when running from the packaged crate (no `demo/`).
fn read_bundled_demo_recording() -> Option<Vec<u8>> {
    let demo_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("demo");
    if !demo_dir.is_dir() {
        // Packaged crate: `demo/` is excluded from the crates.io tarball.
        return None;
    }
    let path = demo_dir.join("public/recordings/sample.b3rec");
    Some(std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display())))
}

/// The bundled recording is captured by the wasm32 demo build, so its header records a pointer
/// width of 4. RecPlayer::create gates on the host pointer width because the C snapshot memcpys
/// POD defs containing `void*`; the port serializes every field explicitly (pointers as u64), so
/// the wire layout is pointer-width independent and only the header byte differs. Rewrite that one
/// byte to the host width so the native test exercises the same bytes the wasm player loads.
fn with_host_pointer_width(mut data: Vec<u8>) -> Vec<u8> {
    let hdr = RecHeader::from_bytes(&data).expect("header");
    assert_eq!(
        hdr.pointer_width, 4,
        "bundled recording must come from the wasm32 build"
    );
    data[8] = std::mem::size_of::<*const ()>() as u8;
    data
}

/// The demo's default Replay recording (Box Stack, 300 frames at 60 Hz, 4 sub-steps) loads with
/// the current format and replays to the end without diverging.
#[cfg(not(feature = "double-precision"))]
#[test]
fn bundled_demo_recording_replays_without_divergence() {
    use crate::recording::{RecPlayer, REC_VERSION_MAJOR};

    let Some(raw) = read_bundled_demo_recording() else {
        return;
    };
    let hdr = RecHeader::from_bytes(&raw).expect("header");
    assert_eq!(hdr.magic, REC_MAGIC);
    assert_eq!(hdr.version_major, REC_VERSION_MAJOR);
    let data = with_host_pointer_width(raw);

    let mut player = RecPlayer::create(&data, 1)
        .expect("bundled sample.b3rec rejected; regenerate it (demo/scripts/record-sample.ts)");
    assert_eq!(player.get_frame_count(), 300);
    // Ground plus the 40-cube Box Stack.
    assert_eq!(player.get_body_count(), 41);

    while player.step_frame() {}
    assert!(player.is_at_end());
    assert_eq!(player.get_frame(), 300);
    assert!(
        !player.has_diverged(),
        "bundled recording diverged at frame {}",
        player.get_diverge_frame()
    );
    assert!(validate_replay(&data, 1));
}

/// Like C, a recording carries no precision flag in its header; the seed snapshot's
/// double-precision flag must match the build, so a large-world build rejects the float-build
/// demo recording instead of misreading its positions.
#[cfg(feature = "double-precision")]
#[test]
fn bundled_demo_recording_rejected_by_double_precision_build() {
    use crate::recording::{
        RecPlayer, REC_VERSION_MAJOR, SNAP_FLAG_DOUBLE_PRECISION, SNAP_MAGIC, SNAP_VERSION,
    };

    let Some(raw) = read_bundled_demo_recording() else {
        return;
    };
    // The recording header itself is current, so the rejection below cannot be a stale format.
    let hdr = RecHeader::from_bytes(&raw).expect("header");
    assert_eq!(hdr.magic, REC_MAGIC);
    assert_eq!(hdr.version_major, REC_VERSION_MAJOR);

    // The seed snapshot image follows the 48-byte header: magic, version, layout hash, flags.
    // Its magic and version are current and it was written by a float build, so the precision
    // flag is the specific mismatch deserialize_into_shell rejects.
    let snap_word = |i: usize| u32::from_le_bytes(raw[48 + 4 * i..52 + 4 * i].try_into().unwrap());
    assert_eq!(snap_word(0), SNAP_MAGIC);
    assert_eq!(snap_word(1), SNAP_VERSION);
    assert_eq!(snap_word(3) & SNAP_FLAG_DOUBLE_PRECISION, 0);

    let data = with_host_pointer_width(raw);
    assert!(RecPlayer::create(&data, 1).is_none());
}
