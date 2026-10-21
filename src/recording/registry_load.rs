//! Geometry registry and query tag table loading for the replay player. Port of
//! `b3RecLoadSlots` / `b3RecLoadTags` from replay.c, split from player.rs to satisfy the
//! file-length limit.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::compound::validate_compound;
use crate::recording::registry::{GeometryKind, RegistrySlot};
use crate::recording::session::RecTag;

/// Load the trailing registry block into reader slots, then the optional tag table. Returns
/// `None` if the registry is corrupt, in which case player creation fails. (b3RecLoadSlots)
pub(crate) fn load_slots(
    data: &[u8],
    registry_offset: u64,
    registry_byte_count: u64,
) -> Option<(Vec<RegistrySlot>, Vec<RecTag>)> {
    if registry_offset == 0 || registry_byte_count == 0 {
        return Some((Vec::new(), Vec::new()));
    }

    // Divergence from C: C narrows registryOffset and registryByteCount with (int) casts before
    // the bounds checks, so a huge value can wrap; the port checks in u64 with a saturating add.
    let size = data.len() as u64;
    let reg_start = registry_offset;
    let reg_end = reg_start.saturating_add(registry_byte_count);
    if reg_end > size {
        eprintln!("b3ReplayFile: registry block out of bounds");
        return None;
    }
    if reg_start + 4 > size {
        eprintln!("b3ReplayFile: registry too small");
        return None;
    }

    let end = reg_end as usize;
    let mut cursor = reg_start as usize;
    let count = u32::from_le_bytes(data[cursor..cursor + 4].try_into().unwrap());
    cursor += 4;

    if count == 0 {
        return Some((Vec::new(), load_tags(data, cursor, end)));
    }

    // Each entry is at least 5 bytes (kind + 4-byte length). A count that cannot fit the remaining
    // registry bytes is a corrupt header, so reject it before allocating.
    if cursor > end || count as usize > (end - cursor) / 5 {
        eprintln!("b3ReplayFile: registry count out of range");
        return None;
    }

    let mut slots = Vec::with_capacity(count as usize);
    for i in 0..count {
        if cursor + 5 > end {
            eprintln!("b3ReplayFile: registry truncated at entry {i}");
            return None;
        }
        let kind_byte = data[cursor];
        let byte_count = u32::from_le_bytes(data[cursor + 1..cursor + 5].try_into().unwrap());
        cursor += 5;
        let mut error = None;
        if byte_count == 0 || byte_count > i32::MAX as u32 {
            error = Some("has an invalid byte count");
        } else if byte_count as usize > end - cursor {
            error = Some("bytes out of bounds");
        } else {
            let bytes = data[cursor..cursor + byte_count as usize].to_vec();
            let mut compound = None;

            // Validate the compound.
            // todo validate other types?
            if kind_byte == GeometryKind::Compound as u8 {
                compound = validate_compound(&bytes);
                if compound.is_none() {
                    error = Some("is not a valid compound");
                }
            }

            slots.push(RegistrySlot {
                kind: kind_byte,
                bytes,
                compound,
            });
        }
        if let Some(error) = error {
            eprintln!("b3ReplayFile: registry entry {i} {error}");
            return None;
        }
        cursor += byte_count as usize;
    }

    Some((slots, load_tags(data, cursor, end)))
}

/// Load the optional query tag table that follows the registry entries, reading `[cursor, end)`.
/// (b3RecLoadTags)
///
/// Divergence from C: C stores the name bytes in a fixed `char` buffer; the port keeps a
/// `String`, so name bytes that are not valid UTF-8 are converted lossily. The name still ends at
/// the first NUL, as C reads it back as a C string.
pub(crate) fn load_tags(data: &[u8], cursor: usize, end: usize) -> Vec<RecTag> {
    let mut tags = Vec::new();
    if cursor > end || end - cursor < 4 {
        return tags;
    }
    let mut cursor = cursor;
    let count = u32::from_le_bytes(data[cursor..cursor + 4].try_into().unwrap());
    cursor += 4;
    if count == 0 {
        return tags;
    }

    // Each tag is at least 18 bytes (8 key + 8 id + 2 length). Reject a count that cannot fit the
    // remaining bytes so a corrupt table cannot request a wild allocation.
    if count as usize > (end - cursor) / 18 {
        return tags;
    }

    tags.reserve(count as usize);
    for _ in 0..count {
        if end - cursor < 18 {
            break;
        }
        let key = u64::from_le_bytes(data[cursor..cursor + 8].try_into().unwrap());
        let id = u64::from_le_bytes(data[cursor + 8..cursor + 16].try_into().unwrap());
        let mut len =
            u16::from_le_bytes(data[cursor + 16..cursor + 18].try_into().unwrap()) as usize;
        cursor += 18;
        if len == 0xFFFF {
            len = 0; // a null name is written as 0xFFFF
        }
        if cursor + len > end {
            break;
        }
        // C clamps the stored name to B3_MAX_QUERY_NAME_LENGTH but always advances the cursor by
        // the full recorded length.
        let n = len.min(crate::recording::session::MAX_QUERY_NAME_LENGTH);
        let name = &data[cursor..cursor + n];
        let name = match name.iter().position(|&b| b == 0) {
            Some(nul) => &name[..nul],
            None => name,
        };
        let query_name = String::from_utf8_lossy(name).into_owned();
        cursor += len;
        tags.push(RecTag {
            key,
            id,
            query_name,
        });
    }
    tags
}
