// Port of box3d-cpp-reference/src/id_pool.h and src/id_pool.c
//
// An index allocator with a free list: ids are dense while allocated and
// recycled LIFO. The C b3Array(int) free array maps to Vec<i32>.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

/// (b3IdPool)
#[derive(Debug, Clone, Default)]
pub struct IdPool {
    /// Free-list of recycled ids. C: `b3Array(int) freeArray` → `Vec<i32>`.
    pub(crate) free_array: Vec<i32>,
    pub(crate) next_index: i32,
}

impl IdPool {
    /// (b3CreateIdPool)
    pub fn new() -> IdPool {
        IdPool {
            // C: b3Array_Reserve(pool.freeArray, 32) — capacity 32, count 0.
            free_array: Vec::with_capacity(32),
            next_index: 0,
        }
    }

    /// (b3DestroyIdPool)
    pub fn destroy(&mut self) {
        *self = IdPool {
            free_array: Vec::new(),
            next_index: 0,
        };
    }

    /// (b3AllocId)
    pub fn alloc_id(&mut self) -> i32 {
        if let Some(id) = self.free_array.pop() {
            return id;
        }

        let id = self.next_index;
        self.next_index += 1;
        id
    }

    /// (b3FreeId)
    ///
    /// C has a dead shrink branch (`if id == nextIndex`) that is unreachable
    /// under the asserts above it (`id < nextIndex`). Always push, matching
    /// the reachable C path and box2d-rust.
    pub fn free_id(&mut self, id: i32) {
        debug_assert!(self.next_index > 0);
        debug_assert!(0 <= id && id < self.next_index);
        self.free_array.push(id);
    }

    /// (b3GetIdCount)
    pub fn id_count(&self) -> i32 {
        self.next_index - self.free_array.len() as i32
    }

    /// (b3GetIdCapacity)
    pub fn id_capacity(&self) -> i32 {
        self.next_index
    }

    /// (b3GetIdBytes)
    pub fn id_bytes(&self) -> i32 {
        // C: b3Array_ByteCount(a) = capacity * sizeof(*data)
        (self.free_array.capacity() * core::mem::size_of::<i32>()) as i32
    }

    /// Debug check that `id` is currently free. (b3ValidateFreeId)
    pub fn validate_free_id(&self, id: i32) {
        if self.free_array.contains(&id) {
            return;
        }

        debug_assert!(false, "id {id} is not free");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // id_pool has no dedicated C unit test; this locks in the allocator behavior.
    #[test]
    fn alloc_free_recycle() {
        let mut pool = IdPool::new();

        assert_eq!(pool.alloc_id(), 0);
        assert_eq!(pool.alloc_id(), 1);
        assert_eq!(pool.alloc_id(), 2);
        assert_eq!(pool.id_count(), 3);
        assert_eq!(pool.id_capacity(), 3);

        // Free list is LIFO: last freed id is reused first.
        pool.free_id(1);
        pool.validate_free_id(1);
        assert_eq!(pool.id_count(), 2);

        assert_eq!(pool.alloc_id(), 1);
        assert_eq!(pool.id_count(), 3);

        // Exhausted free list resumes bump allocation.
        assert_eq!(pool.alloc_id(), 3);
        assert_eq!(pool.id_capacity(), 4);

        pool.destroy();
        assert_eq!(pool.id_count(), 0);
    }
}
