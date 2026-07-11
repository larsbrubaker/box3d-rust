// Port of box3d-cpp-reference/src/bitset.h and src/bitset.c (plus b3CountSetBits
// from table.c). A bit set provides fast operations on large arrays of bits.
//
// The C struct owns a raw `uint64_t*` with separate `blockCapacity` and
// `blockCount`. The Rust port stores the blocks in a `Vec<u64>` whose length is
// always the capacity, and keeps `block_count` as the logical size. That
// preserves the C distinction exactly: reads and clears past `block_count` are
// no-ops even when the capacity is larger, and the growth policy matches.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::pop_count64;

/// Bit set providing fast operations on large arrays of bits.
#[derive(Debug, Clone, Default)]
pub struct BitSet {
    /// Backing storage. `blocks.len()` is the block capacity.
    pub(crate) blocks: Vec<u64>,
    /// Logical number of active blocks (`blockCount` in C).
    pub(crate) block_count: u32,
}

impl BitSet {
    /// (b3CreateBitSet)
    pub fn new(bit_capacity: u32) -> BitSet {
        let block_capacity = bit_capacity.div_ceil(u64::BITS);
        BitSet {
            blocks: vec![0; block_capacity as usize],
            block_count: 0,
        }
    }

    /// Release the storage. (b3DestroyBitSet)
    ///
    /// Rust would drop the storage automatically; this mirrors the C function so
    /// ported call sites read the same.
    pub fn destroy(&mut self) {
        self.blocks = Vec::new();
        self.block_count = 0;
    }

    /// The allocated block capacity (`blockCapacity` in C).
    fn block_capacity(&self) -> u32 {
        self.blocks.len() as u32
    }

    /// (b3SetBitCountAndClear)
    pub fn set_bit_count_and_clear(&mut self, bit_count: u32) {
        let block_count = bit_count.div_ceil(u64::BITS);
        if self.block_capacity() < block_count {
            self.destroy();
            let new_bit_capacity = bit_count + (bit_count >> 1);
            *self = BitSet::new(new_bit_capacity);
        }

        self.block_count = block_count;
        for block in &mut self.blocks[..block_count as usize] {
            *block = 0;
        }
    }

    /// (b3GrowBitSet)
    pub fn grow(&mut self, block_count: u32) {
        debug_assert!(block_count > self.block_count);
        if block_count > self.block_capacity() {
            // C: new capacity = blockCount + blockCount / 2, freshly zeroed, with
            // the old capacity's blocks copied in. Vec::resize preserves existing
            // elements [0, old_len) and zero-fills the new tail, which matches
            // because old_len == old capacity.
            let new_capacity = block_count + block_count / 2;
            self.blocks.resize(new_capacity as usize, 0);
        }

        self.block_count = block_count;
    }

    /// In-place union: `self |= other`. (b3InPlaceUnion)
    pub fn in_place_union(&mut self, other: &BitSet) {
        debug_assert!(self.block_count == other.block_count);
        let block_count = self.block_count as usize;
        for i in 0..block_count {
            self.blocks[i] |= other.blocks[i];
        }
    }

    /// Count the number of set bits. (b3CountSetBits, from table.c)
    pub fn count_set_bits(&self) -> i32 {
        let mut pop_count = 0;
        for i in 0..self.block_count as usize {
            pop_count += pop_count64(self.blocks[i]);
        }
        pop_count
    }

    /// Set a bit that must lie within the current block count. (b3SetBit)
    pub fn set_bit(&mut self, bit_index: u32) {
        let block_index = bit_index / 64;
        debug_assert!(block_index < self.block_count);
        self.blocks[block_index as usize] |= 1u64 << (bit_index % 64);
    }

    /// Set a bit, growing the set if needed. (b3SetBitGrow)
    pub fn set_bit_grow(&mut self, bit_index: u32) {
        let block_index = bit_index / 64;
        if block_index >= self.block_count {
            self.grow(block_index + 1);
        }
        self.blocks[block_index as usize] |= 1u64 << (bit_index % 64);
    }

    /// Clear a bit. Out-of-range indices are ignored. (b3ClearBit)
    pub fn clear_bit(&mut self, bit_index: u32) {
        let block_index = bit_index / 64;
        if block_index >= self.block_count {
            return;
        }
        self.blocks[block_index as usize] &= !(1u64 << (bit_index % 64));
    }

    /// Get a bit. Out-of-range indices read as false. (b3GetBit)
    pub fn get_bit(&self, bit_index: u32) -> bool {
        let block_index = bit_index / 64;
        if block_index >= self.block_count {
            return false;
        }
        (self.blocks[block_index as usize] & (1u64 << (bit_index % 64))) != 0
    }

    /// Byte size of the allocated storage. (b3GetBitSetBytes)
    pub fn bytes(&self) -> i32 {
        self.block_capacity() as i32 * core::mem::size_of::<u64>() as i32
    }

    /// Number of active 64-bit blocks (`blockCount` in C). Used together with
    /// [`BitSet::block`] to port the C word/CTZ set-bit iteration loops.
    pub fn block_count(&self) -> u32 {
        self.block_count
    }

    /// The k-th active 64-bit block (`bits[k]` in C).
    pub fn block(&self, block_index: u32) -> u64 {
        debug_assert!(block_index < self.block_count);
        self.blocks[block_index as usize]
    }
}
