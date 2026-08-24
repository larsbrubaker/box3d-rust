// Port of box3d-cpp-reference/test/test_hash.c
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
//
// The subtests that hash baked hulls, meshes and height fields (HashBoxHulls,
// HashTransformedBoxHulls, HashProceduralHulls, HashHeightFields, HashMeshes,
// HashVoxelDispersion, HashVoxelHullDatabase, and the hull half of
// HashStability) land with the geometry switch-over to the 64-bit hash. The
// subtests below only need `hash64_non_zero` and are ported here in full.

use crate::core::hash64_non_zero;
use crate::rapidhash::rapidhash;
use crate::table::HashSet;

// Separates the two things a repeated hash can mean. Identical bytes repeating is normal, since
// distinct generator parameters can still bake to the same geometry. Distinct bytes sharing a hash
// is the failure this file exists to catch, so the invariant under test is that different content
// gets different digests, never that different parameters do.
//
// Zero doubles as "no collision", since the hash reserves that value. Results are read back only
// after teardown so a failing test does not also dump a leak and bury which condition broke.
struct HashEntry {
    hash: u64,
    bytes: Vec<u8>,
}

struct HashProbe {
    set: HashSet,
    entries: Vec<HashEntry>,
    capacity: usize,
    collision: u64,
    duplicates: i32,
    saw_zero: bool,
    overflow: bool,
}

struct HashResult {
    collision: u64,
    count: usize,
    duplicates: i32,
    saw_zero: bool,
    overflow: bool,
}

fn probe_begin(capacity: usize) -> HashProbe {
    HashProbe {
        set: HashSet::new(capacity as i32),
        entries: Vec::with_capacity(capacity),
        capacity,
        collision: 0,
        duplicates: 0,
        saw_zero: false,
        overflow: false,
    }
}

// Pass None bytes when the caller builds provably distinct inputs, so any repeat is a collision by
// construction and there is nothing to compare against.
fn probe_add(probe: &mut HashProbe, hash: u64, bytes: Option<&[u8]>) {
    if hash == 0 {
        probe.saw_zero = true;
    }

    if probe.entries.len() == probe.capacity {
        probe.overflow = true;
        return;
    }

    if probe.set.add_key(hash) {
        let mut same_content = false;
        if let Some(bytes) = bytes {
            for entry in &probe.entries {
                if entry.hash != hash {
                    continue;
                }

                // C compares byteCount then memcmp, so two empty blobs count as same content.
                same_content = entry.bytes == bytes;
                break;
            }
        }

        if same_content {
            probe.duplicates += 1;
        } else if probe.collision == 0 {
            probe.collision = hash;
        }
    }

    // C stores byteCount 0 and a NULL pointer when the caller passes no bytes, which the
    // comparison above cannot tell apart from a stored empty blob.
    let stored = bytes.unwrap_or(&[]).to_vec();

    probe.entries.push(HashEntry {
        hash,
        bytes: stored,
    });
}

fn probe_end(probe: &mut HashProbe) -> HashResult {
    let result = HashResult {
        collision: probe.collision,
        count: probe.entries.len(),
        duplicates: probe.duplicates,
        saw_zero: probe.saw_zero,
        overflow: probe.overflow,
    };

    probe.entries.clear();
    probe.set.destroy();
    result
}

// Pins that the upper bytes of each 8 byte word reach the low bits of the hash. Multiply carries
// bits upward only, so a hash that keeps just the low half of its product leaves the low bits blind
// to those bytes, and blobs differing only up there collapse onto a tiny range no finalizer can
// spread back out. Varying the top byte of two words is the sharp case. A pairwise avalanche check
// cannot catch this, since a finalizer scatters any single difference across all bits no matter how
// weak the mixing behind it.
#[test]
fn hash_word_family() {
    const BLOB_SIZE: usize = 16;
    const FAMILY_SIZE: usize = 256 * 256;

    let mut probe = probe_begin(FAMILY_SIZE);

    let mut blob = [0u8; BLOB_SIZE];
    for (i, byte) in blob.iter_mut().enumerate() {
        *byte = (0x5A + i) as u8;
    }

    for a in 0..256 {
        for b in 0..256 {
            blob[7] = a as u8;
            blob[15] = b as u8;
            probe_add(&mut probe, hash64_non_zero(&blob), None);
        }
    }

    let result = probe_end(&mut probe);
    assert!(!result.overflow);
    assert_eq!(result.count, FAMILY_SIZE);
    assert!(!result.saw_zero);
    assert_eq!(result.collision, 0);

    // Bytes are not retained above, so a repeat can only be counted as a collision.
    assert_eq!(result.duplicates, 0);
}

// Every input bit must move the digest, at every offset. Flipping one byte in one position is a
// weak check: a mixer can pass it while still being blind to whole regions of a longer blob.
#[test]
fn hash_bit_sweep() {
    const BLOB_SIZE: usize = 32;
    const BIT_COUNT: usize = 8 * BLOB_SIZE;

    let mut probe = probe_begin(BIT_COUNT + 1);

    let mut blob = [0u8; BLOB_SIZE];
    for (i, byte) in blob.iter_mut().enumerate() {
        *byte = 0xA5 ^ (i * 17) as u8;
    }

    probe_add(&mut probe, hash64_non_zero(&blob), None);

    for bit in 0..BIT_COUNT {
        let mask = 1u8 << (bit & 7);
        blob[bit >> 3] ^= mask;
        probe_add(&mut probe, hash64_non_zero(&blob), None);
        blob[bit >> 3] ^= mask;
    }

    let result = probe_end(&mut probe);
    assert!(!result.overflow);
    assert_eq!(result.count, BIT_COUNT + 1);
    assert!(!result.saw_zero);
    assert_eq!(result.collision, 0);
    assert_eq!(result.duplicates, 0);
}

// Baked blobs carry explicit padding and unused array slots, so long runs of zeros are common and
// content alone cannot separate them. Length has to reach the digest.
#[test]
fn hash_zero_lengths() {
    const MAX_LENGTH: usize = 4096;

    let zeros = [0u8; MAX_LENGTH];

    let mut probe = probe_begin(MAX_LENGTH);
    for n in 0..MAX_LENGTH {
        probe_add(&mut probe, hash64_non_zero(&zeros[0..n]), None);
    }

    let result = probe_end(&mut probe);
    assert!(!result.overflow);
    assert_eq!(result.count, MAX_LENGTH);
    assert!(!result.saw_zero);
    assert_eq!(result.collision, 0);
    assert_eq!(result.duplicates, 0);
}

// Mirrored geometry differs from its original only in sign bits, which sit at the top of every
// float. A mixer that cannot carry high bits downward maps the whole family onto a few digests.
#[test]
fn hash_float_signs() {
    const FLOAT_COUNT: usize = 12;
    const COMBO_COUNT: usize = 1 << FLOAT_COUNT;

    let mut probe = probe_begin(COMBO_COUNT);

    let mut values = [0.0f32; FLOAT_COUNT];
    for (i, value) in values.iter_mut().enumerate() {
        *value = 1.0 + 0.25 * i as f32;
    }

    for mask in 0..COMBO_COUNT {
        let mut flipped = [0.0f32; FLOAT_COUNT];
        for i in 0..FLOAT_COUNT {
            flipped[i] = if mask & (1 << i) != 0 {
                -values[i]
            } else {
                values[i]
            };
        }

        probe_add(&mut probe, hash64_non_zero(&float_bytes(&flipped)), None);
    }

    let result = probe_end(&mut probe);
    assert!(!result.overflow);
    assert_eq!(result.count, COMBO_COUNT);
    assert!(!result.saw_zero);
    assert_eq!(result.collision, 0);
    assert_eq!(result.duplicates, 0);
}

// Procedurally placed vertices land one ulp apart. Those blobs differ in a single low mantissa bit
// buried in a long run of identical bytes.
#[test]
fn hash_float_ulp() {
    const FLOAT_COUNT: usize = 64;

    let mut probe = probe_begin(FLOAT_COUNT + 1);

    let mut values = [0.0f32; FLOAT_COUNT];
    for (i, value) in values.iter_mut().enumerate() {
        *value = 100.0 + i as f32;
    }

    probe_add(&mut probe, hash64_non_zero(&float_bytes(&values)), None);

    for i in 0..FLOAT_COUNT {
        let saved = values[i];
        let bits = saved.to_bits().wrapping_add(1);
        values[i] = f32::from_bits(bits);

        probe_add(&mut probe, hash64_non_zero(&float_bytes(&values)), None);
        values[i] = saved;
    }

    let result = probe_end(&mut probe);
    assert!(!result.overflow);
    assert_eq!(result.count, FLOAT_COUNT + 1);
    assert!(!result.saw_zero);
    assert_eq!(result.collision, 0);
    assert_eq!(result.duplicates, 0);
}

// Identical input must bake to an identical hash, or dedup silently stops working.
#[test]
fn hash_stability() {
    // An empty blob has no content to mix, so it takes the reserved value
    assert_eq!(hash64_non_zero(&[]), 1);

    let blob = b"box3d content hash";
    assert_eq!(hash64_non_zero(blob), hash64_non_zero(blob));
    assert_eq!(hash64_non_zero(blob), rapidhash(blob));
}

// Digests produced by the C reference (box3d-cpp-reference/src/rapidhash.h compiled with MSVC,
// default RAPIDHASH_COMPACT / RAPIDHASH_FAST configuration). These pin every length branch of
// rapidhash_internal: the short paths, the 16..112 tail ladder, and the 112 byte main loop.
#[test]
fn rapidhash_matches_c_reference() {
    let mut buf = [0u8; 600];
    for (i, byte) in buf.iter_mut().enumerate() {
        *byte = (i as u32 * 31 + 7) as u8;
    }

    const EXPECTED: [(usize, u64); 27] = [
        (0, 0x0338dc4be2cecdae),
        (1, 0x5c2caf7d68f06d3e),
        (2, 0xae51663c7995f8f8),
        (3, 0x3bd769fd1eeff68b),
        (4, 0x94c4e44cba1be502),
        (5, 0x20b60f776813eba8),
        (7, 0x3402f3bff7d5ef0f),
        (8, 0xefd148285045d1f3),
        (9, 0x2f8407ca55d0192d),
        (15, 0x94192c8d95e7a5a5),
        (16, 0x0811971e7cf397ba),
        (17, 0x6d84939d31572677),
        (20, 0xe4f443e74d5c7343),
        (32, 0xba48299a836e97da),
        (33, 0xaf3a3a115f66dba3),
        (48, 0xb647c688e65ab5d9),
        (64, 0xecac72effb517b5d),
        (80, 0x533c9fc90f0516dc),
        (96, 0xe2fc8588a9bfb097),
        (112, 0x7fe548224c71702a),
        (113, 0xdd9a928d4d5f38be),
        (128, 0xf0339dece659fbb5),
        (224, 0xb58fdb872bdb4e42),
        (225, 0x5e32d5a5f2c4ba73),
        (300, 0x0f2ff3650b9babc1),
        (512, 0x6432fa19db61a256),
        (600, 0x0c47484ef9fa49d3),
    ];

    for (len, expected) in EXPECTED {
        assert_eq!(rapidhash(&buf[0..len]), expected, "length {len}");
    }

    assert_eq!(rapidhash(b"abc"), 0xcb475beafa9c0da2);
    assert_eq!(rapidhash(b"hello world"), 0x2f27cb27d5240940);

    let zeros = [0u8; 300];
    assert_eq!(rapidhash(&zeros), 0xbb893cddbcfc0618);
    assert_eq!(rapidhash(&zeros[0..17]), 0xb2bcb4499009ffbc);
}

fn float_bytes(values: &[f32]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(values.len() * 4);
    for value in values {
        bytes.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    bytes
}
