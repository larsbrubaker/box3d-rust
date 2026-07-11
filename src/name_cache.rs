// Port of the name cache data model from box3d-cpp-reference/src/name_cache.h.
// Lookup/insert logic lands with the body/shape lifecycle slices.
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
}
