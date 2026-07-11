// Port of box3d-cpp-reference/src/table.h and src/table.c
//
// An open-addressing hash set of u64 keys. Unlike Box2D (which stores only the
// key and treats 0 as empty), Box3D stores `{ key, hash }` and empty slots are
// identified by `hash == 0`. Linear probing for lookup; backward-shift deletion
// to keep probe chains tight. The stored hash is reused on grow and on deletion
// repair so the home slot is known without recomputing.
//
// `b3CountSetBits` is defined in table.c in C but lives on `BitSet` in this port.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::constants::{CHILD_MASK, SHAPE_MASK, SHAPE_POWER};
use crate::core::round_up_power_of2;

/// One slot in the open-addressing table. (b3SetItem)
///
/// Empty slots have `hash == 0` (and typically `key == 0`). Occupied slots store
/// both the key and its Murmur-mixed hash so grow/remove can reuse it.
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct SetItem {
    pub key: u64,
    pub hash: u32,
}

/// Build a symmetric key from a pair of shape indices and a child index.
/// (b3ShapePairKey)
///
/// The smaller shape index goes in the high bits so `(a, b, c)` and `(b, a, c)`
/// map to the same key. Layout: `[shape_lo : 22][shape_hi : 22][child : 20]`.
pub fn shape_pair_key(s1: i32, s2: i32, c: i32) -> u64 {
    if s1 < s2 {
        ((SHAPE_MASK & s1 as u64) << (64 - SHAPE_POWER))
            | ((SHAPE_MASK & s2 as u64) << (64 - 2 * SHAPE_POWER))
            | (CHILD_MASK & c as u64)
    } else {
        ((SHAPE_MASK & s2 as u64) << (64 - SHAPE_POWER))
            | ((SHAPE_MASK & s1 as u64) << (64 - 2 * SHAPE_POWER))
            | (CHILD_MASK & c as u64)
    }
}

/// An open-addressing hash set of non-zero u64 keys. (b3HashSet)
#[derive(Debug, Clone, Default)]
pub struct HashSet {
    /// Backing slots; `items.len()` is the capacity. A slot with `hash == 0` is empty.
    pub(crate) items: Vec<SetItem>,
    pub(crate) count: u32,
}

// Murmur hash finalizer truncated to u32. A good mixer matters here because
// keys are built from pairs of increasing integers, where a simple XOR hash
// collides heavily.
// https://lemire.me/blog/2018/08/15/fast-strongly-universal-64-bit-hashing-everywhere/
fn key_hash(key: u64) -> u32 {
    let mut h = key;
    h ^= h >> 33;
    h = h.wrapping_mul(0xff51afd7ed558ccd);
    h ^= h >> 33;
    h = h.wrapping_mul(0xc4ceb9fe1a85ec53);
    h ^= h >> 33;
    h as u32
}

impl HashSet {
    /// Create a set with at least `capacity` slots, rounded up to a power of two
    /// (minimum 16). (b3CreateSet)
    pub fn new(capacity: i32) -> HashSet {
        // Capacity must be a power of 2
        let capacity = if capacity > 16 {
            round_up_power_of2(capacity)
        } else {
            16
        };

        HashSet {
            items: vec![SetItem::default(); capacity as usize],
            count: 0,
        }
    }

    /// Release the storage. (b3DestroySet)
    pub fn destroy(&mut self) {
        self.items = Vec::new();
        self.count = 0;
    }

    /// Remove all keys, keeping capacity. (b3ClearSet)
    pub fn clear(&mut self) {
        self.count = 0;
        self.items.iter_mut().for_each(|slot| *slot = SetItem::default());
    }

    /// Number of keys in the set.
    pub fn count(&self) -> i32 {
        self.count as i32
    }

    /// Number of allocated slots.
    pub fn capacity(&self) -> i32 {
        self.items.len() as i32
    }

    /// Byte size of the backing storage. (b3GetHashSetBytes)
    pub fn bytes(&self) -> i32 {
        self.items.len() as i32 * core::mem::size_of::<SetItem>() as i32
    }

    fn cap(&self) -> u32 {
        self.items.len() as u32
    }

    // Find the slot holding `key`, or the first empty slot on its probe chain.
    fn find_slot(&self, key: u64, hash: u32) -> usize {
        let capacity = self.cap();
        let mut index = hash & (capacity - 1);
        while self.items[index as usize].hash != 0 && self.items[index as usize].key != key {
            index = (index + 1) & (capacity - 1);
        }
        index as usize
    }

    fn add_key_have_capacity(&mut self, key: u64, hash: u32) {
        let index = self.find_slot(key, hash);
        debug_assert!(self.items[index].hash == 0);
        self.items[index].key = key;
        self.items[index].hash = hash;
        self.count += 1;
    }

    fn grow_table(&mut self) {
        let old_items = core::mem::take(&mut self.items);

        self.count = 0;
        // Capacity must be a power of 2
        self.items = vec![SetItem::default(); 2 * old_items.len()];

        // Transfer items into new array
        for item in &old_items {
            if item.hash == 0 {
                // this item was empty
                continue;
            }

            self.add_key_have_capacity(item.key, item.hash);
        }
    }

    /// True if `key` is present. (b3ContainsKey)
    pub fn contains_key(&self, key: u64) -> bool {
        // key of zero is a sentinel
        debug_assert!(key != 0);
        let hash = key_hash(key);
        let index = self.find_slot(key, hash);
        self.items[index].key == key
    }

    /// Add `key`. Returns true if it was already present. (b3AddKey)
    pub fn add_key(&mut self, key: u64) -> bool {
        // key of zero is a sentinel
        debug_assert!(key != 0);

        let hash = key_hash(key);
        debug_assert!(hash != 0);

        let index = self.find_slot(key, hash);
        if self.items[index].hash != 0 {
            // Already in set
            debug_assert!(self.items[index].hash == hash && self.items[index].key == key);
            return true;
        }

        if 2 * self.count >= self.cap() {
            self.grow_table();
        }

        self.add_key_have_capacity(key, hash);
        false
    }

    /// Remove `key`. Returns true if it was found. (b3RemoveKey)
    // See https://en.wikipedia.org/wiki/Open_addressing
    pub fn remove_key(&mut self, key: u64) -> bool {
        let hash = key_hash(key);
        let mut i = self.find_slot(key, hash);
        if self.items[i].hash == 0 {
            // Not in set
            return false;
        }

        // Mark item i as unoccupied
        self.items[i].key = 0;
        self.items[i].hash = 0;

        debug_assert!(self.count > 0);
        self.count -= 1;

        // Attempt to fill item i
        let capacity = self.cap() as usize;
        let mut j = i;
        loop {
            j = (j + 1) & (capacity - 1);
            if self.items[j].hash == 0 {
                break;
            }

            // k is the first item for the hash of j
            let k = (self.items[j].hash as usize) & (capacity - 1);

            // determine if k lies cyclically in (i,j]
            // i <= j: | i..k..j |
            // i > j: |.k..j  i....| or |....j     i..k.|
            if i <= j {
                if i < k && k <= j {
                    continue;
                }
            } else if i < k || k <= j {
                continue;
            }

            // Move j into i
            self.items[i] = self.items[j];

            // Mark item j as unoccupied
            self.items[j].key = 0;
            self.items[j].hash = 0;

            i = j;
        }

        true
    }
}
