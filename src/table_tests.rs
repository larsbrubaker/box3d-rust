// Port of box3d-cpp-reference/test/test_table.c
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::{bounding_power_of2, round_up_power_of2};
use crate::table::{shape_pair_key, HashSet, SetItem};

const SET_SPAN: i32 = 317;
const ITEM_COUNT: usize = ((SET_SPAN * SET_SPAN - SET_SPAN) / 2) as usize;

#[test]
fn helper_power_of_two() {
    let power = bounding_power_of2(3008);
    assert_eq!(power, 12);

    let next_power_of2 = round_up_power_of2(3008);
    assert_eq!(next_power_of2, 1 << power);
}

#[test]
fn basic_create_and_destroy() {
    let mut set = HashSet::new(16);
    assert_eq!(set.count(), 0);
    assert_eq!(set.capacity(), 16);

    set.destroy();
    assert_eq!(set.count(), 0);
    assert_eq!(set.capacity(), 0);
}

#[test]
fn capacity_rounds_to_power_of_two() {
    assert_eq!(HashSet::new(1).capacity(), 16); // minimum capacity
    assert_eq!(HashSet::new(15).capacity(), 16); // rounds up to 16
    assert_eq!(HashSet::new(32).capacity(), 32); // stays at 32
    assert_eq!(HashSet::new(33).capacity(), 64); // rounds up to 64
}

#[test]
fn set_item_stores_hash() {
    // Box3D slots are { key, hash }; empty is hash == 0, not key-only.
    assert_eq!(core::mem::size_of::<SetItem>(), 16);
    let mut set = HashSet::new(16);
    assert!(!set.add_key(42));
    let occupied = set.items.iter().find(|s| s.key == 42).expect("key present");
    assert_ne!(occupied.hash, 0);
    assert!(set.items.iter().any(|s| s.hash == 0));
}

#[test]
fn add_remove() {
    let mut set = HashSet::new(16);

    assert!(!set.add_key(42)); // new
    assert_eq!(set.count(), 1);

    assert!(!set.add_key(123)); // new
    assert_eq!(set.count(), 2);

    assert!(set.add_key(42)); // already exists
    assert_eq!(set.count(), 2); // count unchanged

    assert!(set.contains_key(42));
    assert!(set.contains_key(123));
    assert!(!set.contains_key(999));

    assert!(set.remove_key(42));
    assert_eq!(set.count(), 1);
    assert!(!set.contains_key(42));
    assert!(set.contains_key(123));

    assert!(!set.remove_key(999)); // non-existent
    assert_eq!(set.count(), 1);

    assert!(!set.remove_key(42)); // already removed
    assert_eq!(set.count(), 1);
}

#[test]
fn clear() {
    let mut set = HashSet::new(16);

    set.add_key(10);
    set.add_key(20);
    set.add_key(30);
    assert_eq!(set.count(), 3);

    set.clear();
    assert_eq!(set.count(), 0);
    assert!(!set.contains_key(10));
    assert!(!set.contains_key(20));
    assert!(!set.contains_key(30));

    set.add_key(40);
    assert_eq!(set.count(), 1);
    assert!(set.contains_key(40));
}

#[test]
fn growth_preserves_keys() {
    let mut set = HashSet::new(16);
    let initial_capacity = set.capacity();

    // Load factor is 0.5, so with capacity 16 growth happens at count 8.
    for i in 0..8u64 {
        set.add_key(i + 1);
    }

    assert!(set.capacity() >= initial_capacity);
    assert_eq!(set.count(), 8);

    for i in 1..=8u64 {
        assert!(set.contains_key(i));
    }
}

#[test]
fn shape_pair_key_is_symmetric() {
    let mut set = HashSet::new(16);

    let key1 = shape_pair_key(5, 10, 0);
    let key2 = shape_pair_key(10, 5, 0); // same as key1
    assert_eq!(key1, key2);

    set.add_key(key1);
    assert!(set.contains_key(key1));
    assert!(set.contains_key(key2));

    let key3 = shape_pair_key(1, 2, 0);
    let key4 = shape_pair_key(2, 3, 0);
    assert_ne!(key3, key4);

    let with_child = shape_pair_key(5, 10, 7);
    assert_ne!(key1, with_child);

    set.add_key(key3);
    set.add_key(key4);
    assert_eq!(set.count(), 3);
}

#[test]
fn bytes_tracks_capacity() {
    let mut set = HashSet::new(32);

    let expected = 32 * core::mem::size_of::<SetItem>() as i32;
    assert_eq!(set.bytes(), expected);

    set.add_key(100);
    set.add_key(200);
    assert_eq!(set.bytes(), expected);
}

// The large fill/remove/search cycle from test_table.c (TableTest), minus the
// optional probe timing. Exercises every shape pair over a 317-wide span.
#[test]
fn large_fill_remove_search() {
    let n = SET_SPAN;
    let item_count = ITEM_COUNT;
    let mut removed = vec![false; item_count];

    let mut set = HashSet::new(16);

    // Fill set with every (i, j) pair, i < j.
    for i in 0..n {
        for j in (i + 1)..n {
            let key = shape_pair_key(i, j, 0);
            assert!(!set.add_key(key));
        }
    }
    assert_eq!(set.count() as usize, item_count);

    // Remove the j == i + 1 (adjacent) pairs.
    let mut k = 0;
    let mut remove_count = 0;
    for i in 0..n {
        for j in (i + 1)..n {
            if j == i + 1 {
                let key = shape_pair_key(i, j, 0);
                assert!(set.remove_key(key));
                removed[k] = true;
                remove_count += 1;
            } else {
                removed[k] = false;
            }
            k += 1;
        }
    }
    assert_eq!(set.count() as usize, item_count - remove_count);

    // Every remaining key is found; removed ones may or may not be.
    // C searches with swapped indices (j, i) to exercise key symmetry.
    let mut k = 0;
    for i in 0..n {
        for j in (i + 1)..n {
            let key = shape_pair_key(j, i, 0);
            let found = set.contains_key(key);
            assert!(found || removed[k]);
            k += 1;
        }
    }

    // Remove everything.
    for i in 0..n {
        for j in (i + 1)..n {
            set.remove_key(shape_pair_key(i, j, 0));
        }
    }
    assert_eq!(set.count(), 0);

    set.destroy();
}
