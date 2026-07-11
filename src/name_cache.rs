// Port of the name cache from box3d-cpp-reference/src/name_cache.h/.c.
// Remaining load/replay helpers land with the recording slice.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::constants::NULL_NAME;
use std::collections::HashMap;

/// A single name entry. (b3NameEntry)
#[derive(Debug, Clone, Default)]
pub struct NameEntry {
    pub hash: u32,
    pub length: i32,
    pub name: String,
}

/// Name cache for shape and body names. Works with recording.
/// (b3NameCache)
///
/// C stores a verstable map as `void* map`; Rust uses a std HashMap keyed by
/// name id for the same id→entry association. Observable name↔id behavior will
/// match once the logic slice lands.
#[derive(Debug, Clone, Default)]
pub struct NameCache {
    pub entries: Vec<NameEntry>,
    /// Maps name id → entry index. Id 0 is [`NULL_NAME`] and is never inserted.
    pub map: HashMap<u32, i32>,
}

impl NameCache {
    /// (b3CreateNameCache)
    pub fn new() -> NameCache {
        NameCache {
            entries: Vec::new(),
            map: HashMap::new(),
        }
    }

    /// Sentinel unused by real names. (B3_NULL_NAME)
    pub const NULL: u32 = NULL_NAME;

    /// FNV-1a + Murmur3 finalizer. Changing this breaks recordings. (b3Hash32)
    pub fn hash32(data: &[u8]) -> u32 {
        const FNV32_OFFSET_BASIS: u32 = 0x811c_9dc5;
        const FNV32_PRIME: u32 = 0x0100_0193;

        let mut h = FNV32_OFFSET_BASIS;
        for &b in data {
            h ^= b as u32;
            h = h.wrapping_mul(FNV32_PRIME);
        }

        h ^= h >> 16;
        h = h.wrapping_mul(0x85eb_ca6b);
        h ^= h >> 13;
        h = h.wrapping_mul(0xc2b2_ae35);
        h ^= h >> 16;
        h
    }

    /// Insert or look up a name, returning its id. Empty names yield NULL.
    /// (b3AddName)
    pub fn add_name(&mut self, name: &str) -> u32 {
        if name.is_empty() {
            return NULL_NAME;
        }

        let mut id = Self::hash32(name.as_bytes());
        if id == 0 {
            id = 1;
        }

        if let Some(&index) = self.map.get(&id) {
            if self.entries[index as usize].name != name {
                // Different name, same hash — C logs; keep returning the id.
            }
            return id;
        }

        let index = self.entries.len() as i32;
        self.entries.push(NameEntry {
            hash: id,
            length: name.len() as i32,
            name: name.to_string(),
        });
        self.map.insert(id, index);
        id
    }

    /// (b3FindName)
    pub fn find_name(&self, id: u32) -> Option<&str> {
        let index = *self.map.get(&id)?;
        Some(&self.entries[index as usize].name)
    }
}
