// Replay tests for loading the trailing geometry registry and query tag table
// (b3RecLoadSlots / b3RecLoadTags). `include!`d into `recording_replay_tests.rs`, so it shares
// that module's imports (and the corrupt-registry helpers).
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::recording::registry_load::load_tags;

/// Append one tag record (key, id, u16 length, name bytes) to a tag table under construction.
fn push_tag(buf: &mut Vec<u8>, key: u64, id: u64, len: u16, name: &[u8]) {
    buf.extend_from_slice(&key.to_le_bytes());
    buf.extend_from_slice(&id.to_le_bytes());
    buf.extend_from_slice(&len.to_le_bytes());
    buf.extend_from_slice(name);
}

/// Return a copy of `data` with the u32 registry entry count replaced.
fn with_registry_count(data: &[u8], count: u32) -> Vec<u8> {
    let hdr = RecHeader::from_bytes(data).expect("header");
    let offset = hdr.registry_offset as usize;
    let mut copy = data.to_vec();
    copy[offset..offset + 4].copy_from_slice(&count.to_le_bytes());
    copy
}

/// A tag count that cannot fit the remaining bytes (18 bytes minimum per tag) is a corrupt table,
/// so C loads no tags at all rather than the ones that happen to fit.
#[test]
fn oversized_tag_count_loads_no_tags() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&2u32.to_le_bytes());
    push_tag(&mut buf, 11, 1, 0, &[]);
    assert!(load_tags(&buf, 0, buf.len()).is_empty());

    // The same table with an honest count loads the tag.
    buf[0..4].copy_from_slice(&1u32.to_le_bytes());
    let tags = load_tags(&buf, 0, buf.len());
    assert_eq!(tags.len(), 1);
    assert_eq!(tags[0].key, 11);
}

/// C copies the stored name into a fixed buffer and reads it back as a C string, so the name ends
/// at the first NUL, while the cursor still advances by the full recorded length.
#[test]
fn tag_name_stops_at_nul() {
    let mut buf = Vec::new();
    buf.extend_from_slice(&2u32.to_le_bytes());
    push_tag(&mut buf, 21, 1, 5, b"ab\0cd");
    push_tag(&mut buf, 22, 2, 0xFFFF, &[]);
    let tags = load_tags(&buf, 0, buf.len());
    assert_eq!(tags.len(), 2);
    assert_eq!(tags[0].query_name, "ab");
    assert_eq!(tags[1].key, 22);
    assert_eq!(tags[1].query_name, "");
}

/// Duplicate keys both load, and the key lookup replaces on insert, so the last tag wins.
#[test]
fn duplicate_tag_key_last_wins() {
    use crate::recording::RecPlayer;

    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, Recording::new(0));
    world.step(1.0 / 60.0, 4);
    let rec = world_stop_recording(&mut world).expect("recording active");
    let data = rec.data().to_vec();

    // No geometry: the registry block is the zero entry count, then the tag table.
    let hdr = RecHeader::from_bytes(&data).expect("header");
    let offset = hdr.registry_offset as usize;
    assert_eq!(&data[offset..offset + 4], &0u32.to_le_bytes());
    let mut patched = data[..offset + 4].to_vec();
    patched.extend_from_slice(&2u32.to_le_bytes());
    let key = 0x1234_5678_9ABC_DEF0u64;
    push_tag(&mut patched, key, 1, 0xFFFF, &[]);
    push_tag(&mut patched, key, 2, 0xFFFF, &[]);
    let byte_count = (patched.len() - offset) as u64;
    patched[40..48].copy_from_slice(&byte_count.to_le_bytes());

    let tags = load_tags(&patched, offset + 4, patched.len());
    assert_eq!(tags.len(), 2);

    let player = RecPlayer::create(&patched, 1).expect("player");
    assert_eq!(player.resolve_tag(key).expect("tag").id, 2);
}

/// C keeps the raw kind byte, so an unknown kind matches no b3RecGetSlot request and the shape
/// op fails with "compound geometryId %u is invalid".
#[test]
fn unknown_geometry_kind_fails_replay() {
    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();
    let (kind, start) = registry_slot_spans(&data)[1];
    assert_eq!(kind, GeometryKind::Compound);
    let mut corrupt = data.clone();
    corrupt[start - 5] = 7;
    assert!(!validate_replay(&corrupt, 1));
}

/// A registry block that runs past the end of the file fails player creation.
/// ("registry block out of bounds")
#[test]
fn registry_block_out_of_bounds_fails_create() {
    use crate::recording::RecPlayer;

    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();
    let hdr = RecHeader::from_bytes(&data).expect("header");
    let mut corrupt = data.clone();
    let byte_count = data.len() as u64 - hdr.registry_offset + 1;
    corrupt[40..48].copy_from_slice(&byte_count.to_le_bytes());
    assert!(RecPlayer::create(&data, 1).is_some());
    assert!(RecPlayer::create(&corrupt, 1).is_none());
}

/// An entry count that cannot fit the registry bytes fails player creation.
/// ("registry count out of range")
#[test]
fn registry_count_out_of_range_fails_create() {
    use crate::recording::RecPlayer;

    let rec = record_all_geometry_ops();
    let corrupt = with_registry_count(rec.data(), u32::MAX);
    assert!(RecPlayer::create(&corrupt, 1).is_none());
}

/// One more entry than the registry holds runs into the 4 byte tag count and fails player
/// creation. ("registry truncated at entry %u")
#[test]
fn registry_truncated_entry_fails_create() {
    use crate::recording::RecPlayer;

    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();
    let count = registry_slot_spans(&data).len() as u32;
    let corrupt = with_registry_count(&data, count + 1);
    assert!(RecPlayer::create(&corrupt, 1).is_none());
}

/// A registry entry with no bytes fails player creation for every kind, not just compounds.
/// ("registry entry %u has an invalid byte count", upstream 5d83df8)
#[test]
fn zero_byte_registry_entry_fails_create() {
    use crate::recording::RecPlayer;

    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();
    let (kind, start) = registry_slot_spans(&data)[3];
    assert_eq!(kind, GeometryKind::Hull);
    let byte_count = read_i32_at(&data, start - 4) as usize;
    assert!(byte_count > 0);

    // Drop the hull blob and record a zero length, keeping the registry block consistent.
    let mut corrupt = data[..start].to_vec();
    corrupt.extend_from_slice(&data[start + byte_count..]);
    corrupt[start - 4..start].copy_from_slice(&0i32.to_le_bytes());
    let hdr = RecHeader::from_bytes(&data).expect("header");
    let registry_byte_count = hdr.registry_byte_count - byte_count as u64;
    corrupt[40..48].copy_from_slice(&registry_byte_count.to_le_bytes());

    assert!(RecPlayer::create(&data, 1).is_some());
    assert!(RecPlayer::create(&corrupt, 1).is_none());
}

/// Compound entries are validated when the registry loads (b3ValidateCompound), so a compound
/// whose tree node array runs off the blob fails player creation rather than the shape op.
/// ("registry entry %u is not a valid compound", upstream 5d83df8)
#[test]
fn invalid_compound_entry_fails_create() {
    use crate::recording::RecPlayer;

    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();
    let (kind, start) = registry_slot_spans(&data)[1];
    assert_eq!(kind, GeometryKind::Compound);

    // b3CompoundData::nodeCount is header byte 44.
    for node_count in [1, i32::MAX] {
        let corrupt = poison_i32(&data, start + 44, node_count);
        assert!(RecPlayer::create(&corrupt, 1).is_none());
    }
    assert!(RecPlayer::create(&data, 1).is_some());
}
