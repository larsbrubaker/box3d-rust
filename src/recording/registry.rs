//! Geometry registry for snapshot/recording interning.
//! Port of `b3GeometryRegistry` / `b3InternGeometry` / `b3RecIntern*` from recording.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::compound::{convert_bytes_to_compound, CompoundData};
use crate::core::hash64_non_zero;
use crate::height_field::HeightFieldData;
use crate::hull::HullData;
use crate::mesh::MeshData;
use std::collections::HashMap;

/// Geometry kinds for the trailing registry section. (b3GeometryKind)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum GeometryKind {
    Hull = 0,
    Mesh = 1,
    HeightField = 2,
    Compound = 3,
}

/// One entry per unique geometry blob. (b3GeometryEntry)
#[derive(Debug, Clone)]
pub struct GeometryEntry {
    pub content_hash: u64,
    pub id: u32,
    pub kind: GeometryKind,
    pub bytes: Vec<u8>,
    /// Next entry sharing the same content hash; `NULL_INDEX` ends the chain.
    pub hash_next: i32,
}

/// Growable array of geometry entries with content-hash dedup. (b3GeometryRegistry)
#[derive(Debug, Default)]
pub struct GeometryRegistry {
    pub entries: Vec<GeometryEntry>,
    /// Maps content hash → first entry index for O(1) chain start.
    dedup: HashMap<u64, i32>,
}

impl GeometryRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append without dedup (id == slot index). (b3AppendGeometry)
    pub fn append(&mut self, kind: GeometryKind, content_hash: u64, bytes: Vec<u8>) -> u32 {
        let id = self.entries.len() as u32;
        let hash_next = self.dedup.get(&content_hash).copied().unwrap_or(-1);
        self.entries.push(GeometryEntry {
            content_hash,
            id,
            kind,
            bytes,
            hash_next,
        });
        self.dedup.insert(content_hash, id as i32);
        id
    }

    /// (b3InternGeometry)
    pub fn intern(&mut self, kind: GeometryKind, content_hash: u64, bytes: Vec<u8>) -> u32 {
        if let Some(&head) = self.dedup.get(&content_hash) {
            let mut idx = head;
            while idx >= 0 {
                let e = &self.entries[idx as usize];
                if e.kind == kind && e.bytes == bytes {
                    return e.id;
                }
                idx = e.hash_next;
            }
        }

        let id = self.entries.len() as u32;
        let hash_next = self.dedup.get(&content_hash).copied().unwrap_or(-1);
        self.entries.push(GeometryEntry {
            content_hash,
            id,
            kind,
            bytes,
            hash_next,
        });
        self.dedup.insert(content_hash, id as i32);
        id
    }

    /// (b3RecInternHull)
    pub fn intern_hull(&mut self, hull: &HullData) -> u32 {
        let bytes = hull.to_bytes();
        let h = hash64_non_zero(&bytes);
        self.intern(GeometryKind::Hull, h, bytes)
    }

    /// (b3RecInternMesh)
    pub fn intern_mesh(&mut self, mesh: &MeshData) -> u32 {
        let bytes = mesh.to_bytes();
        let h = hash64_non_zero(&bytes);
        self.intern(GeometryKind::Mesh, h, bytes)
    }

    /// (b3RecInternHeightField)
    pub fn intern_height_field(&mut self, hf: &HeightFieldData) -> u32 {
        let bytes = hf.to_bytes();
        let h = hash64_non_zero(&bytes);
        self.intern(GeometryKind::HeightField, h, bytes)
    }

    /// (b3RecInternCompound)
    pub fn intern_compound(&mut self, compound: &CompoundData) -> u32 {
        let bytes = compound.to_bytes();
        let h = hash64_non_zero(&bytes);
        self.intern(GeometryKind::Compound, h, bytes)
    }

    /// Build reader slots from registry entries (id == index).
    pub fn to_slots(&self) -> Vec<RegistrySlot> {
        self.entries
            .iter()
            .map(|e| RegistrySlot {
                kind: e.kind,
                bytes: e.bytes.clone(),
                live_compound: None,
            })
            .collect()
    }
}

/// Resolved geometry slot for deserialization. (b3RegistrySlot)
#[derive(Debug, Clone)]
pub struct RegistrySlot {
    pub kind: GeometryKind,
    pub bytes: Vec<u8>,
    /// Lazily converted compound (C: slot->live).
    pub live_compound: Option<CompoundData>,
}

impl RegistrySlot {
    pub fn ensure_compound(&mut self) -> Option<&CompoundData> {
        if self.live_compound.is_none() {
            self.live_compound = convert_bytes_to_compound(&self.bytes);
        }
        self.live_compound.as_ref()
    }
}
