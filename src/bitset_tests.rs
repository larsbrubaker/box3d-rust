// Port of box3d-cpp-reference/test/test_bitset.c
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::core::{clz32, lower_power_of_2_exponent};

const COUNT: usize = 169;

#[test]
fn bit_math() {
    let r1 = clz32(9);
    assert_eq!(r1, 31 - 3);

    for i in 1..1000 {
        let e1 = lower_power_of_2_exponent(i);
        let e2 = ((i as f32).log2().floor()) as i32;
        assert_eq!(e1, e2, "i = {i}");
    }
}

#[test]
fn bit_set_fibonacci_pattern() {
    let mut bit_set = BitSet::new(COUNT as u32);

    bit_set.set_bit_count_and_clear(COUNT as u32);
    let mut values = [false; COUNT];

    // Set the bits at Fibonacci indices.
    let (mut i1, mut i2) = (0i32, 1i32);
    bit_set.set_bit(i1 as u32);
    values[i1 as usize] = true;

    while i2 < COUNT as i32 {
        bit_set.set_bit(i2 as u32);
        values[i2 as usize] = true;
        let next = i1 + i2;
        i1 = i2;
        i2 = next;
    }

    for (i, &expected) in values.iter().enumerate() {
        let value = bit_set.get_bit(i as u32);
        assert_eq!(value, expected);
    }

    bit_set.destroy();
}

// Additional coverage for the operations test_bitset.c does not exercise:
// growth, union, clearing, and counting.
#[test]
fn grow_union_clear_count() {
    let mut a = BitSet::new(1);
    a.set_bit_count_and_clear(64);

    // set_bit_grow past the current block count grows the set.
    a.set_bit_grow(200);
    assert!(a.get_bit(200));
    a.set_bit_grow(5);
    assert!(a.get_bit(5));
    assert_eq!(a.count_set_bits(), 2);

    // Reads and clears past the logical range are no-ops, never panics.
    assert!(!a.get_bit(100_000));
    a.clear_bit(100_000);

    a.clear_bit(5);
    assert!(!a.get_bit(5));
    assert_eq!(a.count_set_bits(), 1);

    // Union ORs matching blocks together.
    let mut b = a.clone();
    b.set_bit(1);
    a.in_place_union(&b);
    assert!(a.get_bit(1));
    assert!(a.get_bit(200));
    assert_eq!(a.count_set_bits(), 2);
}
