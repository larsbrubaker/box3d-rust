//! World hull database: content-keyed sharing with reference counts.
//!
//! Port of `b3AddHullToDatabase` / `b3RemoveHullFromDatabase` from
//! `physics_world.c`. C uses a verstable map keyed by hull pointer with
//! content hash/equality; Rust keeps canonical `Rc<HullData>` entries and
//! deduplicates with [`compare_hull_data`].
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{clone_hull, compare_hull_data, destroy_hull, hash_hull_data, HullData};
use std::rc::Rc;

/// Shared hull store for a world. (b3HullMap + world->hullDatabase)
#[derive(Debug, Default)]
pub struct HullDatabase {
    /// Canonical strong refs — one per unique hull content.
    entries: Vec<Rc<HullData>>,
}

impl HullDatabase {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    /// Find a shared copy matching `src` by content, if any.
    fn find(&self, src: &HullData) -> Option<Rc<HullData>> {
        let h = hash_hull_data(src);
        for entry in &self.entries {
            if hash_hull_data(entry) == h && compare_hull_data(entry, src) {
                return Some(Rc::clone(entry));
            }
        }
        None
    }

    /// Clone `src` into the database on miss; share on hit.
    /// (b3AddHullToDatabase)
    pub fn add(&mut self, src: &HullData) -> Rc<HullData> {
        if let Some(existing) = self.find(src) {
            return existing;
        }

        let owned = clone_hull(src).expect("valid hull");
        let rc = Rc::new(owned);
        self.entries.push(Rc::clone(&rc));
        rc
    }

    /// Take ownership of `owned` on miss; destroy the duplicate on hit.
    /// (b3AddOwnedHullToDatabase)
    pub fn add_owned(&mut self, owned: HullData) -> Rc<HullData> {
        if let Some(existing) = self.find(&owned) {
            destroy_hull(owned);
            return existing;
        }

        let rc = Rc::new(owned);
        self.entries.push(Rc::clone(&rc));
        rc
    }

    /// Drop the database's strong ref once the last shape releases its clone.
    /// Call while the shape still holds its `Rc` (strong_count >= 2 if DB also holds).
    /// (b3RemoveHullFromDatabase)
    pub fn release(&mut self, hull: &Rc<HullData>) {
        // Shape + DB = 2. After this release, only the shape's Rc remains (or none).
        if Rc::strong_count(hull) == 2 {
            self.entries.retain(|e| !Rc::ptr_eq(e, hull));
        }
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }
}
