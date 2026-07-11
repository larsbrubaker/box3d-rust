// Port of box3d-cpp-reference/src/broad_phase.h and broad_phase.c: storage,
// proxy operations, move buffering, overlap testing, and validation.
//
// `b3UpdateBroadPhasePairs` is deferred until World/contact exist — it needs
// shape arrays and contact creation. Proxy ops and move buffering are complete.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::dynamic_tree::DynamicTree;
use crate::math_functions::{aabb_overlaps, max_int, Aabb};
use crate::table::HashSet;
use crate::types::{BodyType, Capacity, BODY_TYPE_COUNT};

/// Store the proxy type in the lower 2 bits of the proxy key. (B3_PROXY_TYPE)
pub fn proxy_type(key: i32) -> BodyType {
    match key & 3 {
        0 => BodyType::Static,
        1 => BodyType::Kinematic,
        _ => BodyType::Dynamic,
    }
}

/// (B3_PROXY_ID)
pub fn proxy_id(key: i32) -> i32 {
    key >> 2
}

/// (B3_PROXY_KEY)
pub fn proxy_key(id: i32, type_: BodyType) -> i32 {
    (id << 2) | (type_ as i32)
}

/// The broad-phase is used for computing pairs and performing volume queries
/// and ray casts. It does not persist pairs; it reports potentially new pairs.
/// (b3BroadPhase)
#[derive(Debug)]
pub struct BroadPhase {
    pub trees: [DynamicTree; BODY_TYPE_COUNT],

    /// Per body-type bit sets indexed by proxyId, marking proxies moved this
    /// step. Paired with move_array which preserves deterministic insertion
    /// order for pair queries.
    pub moved_proxies: [BitSet; BODY_TYPE_COUNT],
    pub move_array: Vec<i32>,

    /// Tracks shape pairs that have a Contact.
    pub pair_set: HashSet,
}

impl BroadPhase {
    /// (b3CreateBroadPhase)
    pub fn new(capacity: &Capacity) -> BroadPhase {
        debug_assert!(BODY_TYPE_COUNT == 3);

        let static_capacity = max_int(16, capacity.static_shape_count);
        let kinematic_capacity = 16;
        let dynamic_capacity = max_int(16, capacity.dynamic_shape_count);

        let mut move_array = Vec::new();
        move_array.reserve(capacity.dynamic_shape_count.max(0) as usize);

        BroadPhase {
            trees: [
                DynamicTree::new(static_capacity),
                DynamicTree::new(kinematic_capacity),
                DynamicTree::new(dynamic_capacity),
            ],
            moved_proxies: [
                BitSet::new(max_int(16, capacity.static_shape_count) as u32),
                BitSet::new(16),
                BitSet::new(max_int(16, capacity.dynamic_shape_count) as u32),
            ],
            move_array,
            // C: b3CreateSet(2 * capacity->contactCount)
            pair_set: HashSet::new(2 * capacity.contact_count),
        }
    }

    /// (b3DestroyBroadPhase)
    pub fn destroy(&mut self) {
        *self = BroadPhase {
            trees: [
                DynamicTree::new(0),
                DynamicTree::new(0),
                DynamicTree::new(0),
            ],
            moved_proxies: [BitSet::new(0), BitSet::new(0), BitSet::new(0)],
            move_array: Vec::new(),
            pair_set: HashSet::new(16),
        };
    }

    /// This triggers new contact pairs to be created. Must be called in
    /// deterministic order. (static inline b3BufferMove)
    pub fn buffer_move(&mut self, query_proxy: i32) {
        let proxy_type_ = proxy_type(query_proxy);
        let proxy_id_ = proxy_id(query_proxy);
        let set = &mut self.moved_proxies[proxy_type_ as usize];
        if !set.get_bit(proxy_id_ as u32) {
            set.set_bit_grow(proxy_id_ as u32);
            self.move_array.push(query_proxy);
        }
    }

    /// (b3BroadPhase_CreateProxy)
    pub fn create_proxy(
        &mut self,
        proxy_type_: BodyType,
        aabb: Aabb,
        category_bits: u64,
        shape_index: i32,
        force_pair_creation: bool,
    ) -> i32 {
        debug_assert!((proxy_type_ as usize) < BODY_TYPE_COUNT);
        let proxy_id_ =
            self.trees[proxy_type_ as usize].create_proxy(aabb, category_bits, shape_index as u64);
        let proxy_key_ = proxy_key(proxy_id_, proxy_type_);
        if proxy_type_ != BodyType::Static || force_pair_creation {
            self.buffer_move(proxy_key_);
        }
        proxy_key_
    }

    /// (static b3UnBufferMove)
    fn unbuffer_move(&mut self, proxy_key_: i32) {
        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);
        let set = &mut self.moved_proxies[proxy_type_ as usize];

        if set.get_bit(proxy_id_ as u32) {
            set.clear_bit(proxy_id_ as u32);

            // Purge from move buffer. Linear search. (b3Array_RemoveSwap)
            if let Some(index) = self.move_array.iter().position(|&k| k == proxy_key_) {
                self.move_array.swap_remove(index);
            }
        }
    }

    /// (b3BroadPhase_DestroyProxy)
    pub fn destroy_proxy(&mut self, proxy_key_: i32) {
        self.unbuffer_move(proxy_key_);

        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);

        debug_assert!((proxy_type_ as usize) <= BODY_TYPE_COUNT);
        self.trees[proxy_type_ as usize].destroy_proxy(proxy_id_);
    }

    /// (b3BroadPhase_MoveProxy)
    pub fn move_proxy(&mut self, proxy_key_: i32, aabb: Aabb) {
        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);

        self.trees[proxy_type_ as usize].move_proxy(proxy_id_, aabb);
        self.buffer_move(proxy_key_);
    }

    /// (b3BroadPhase_EnlargeProxy)
    pub fn enlarge_proxy(&mut self, proxy_key_: i32, aabb: Aabb) {
        debug_assert!(proxy_key_ != crate::core::NULL_INDEX);
        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);

        debug_assert!(proxy_type_ != BodyType::Static);

        self.trees[proxy_type_ as usize].enlarge_proxy(proxy_id_, aabb);
        self.buffer_move(proxy_key_);
    }

    /// (b3BroadPhase_GetShapeIndex)
    pub fn shape_index(&self, proxy_key_: i32) -> i32 {
        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);

        self.trees[proxy_type_ as usize].user_data(proxy_id_) as i32
    }

    /// (b3BroadPhase_TestOverlap)
    pub fn test_overlap(&self, proxy_key_a: i32, proxy_key_b: i32) -> bool {
        let type_a = proxy_type(proxy_key_a);
        let id_a = proxy_id(proxy_key_a);
        let type_b = proxy_type(proxy_key_b);
        let id_b = proxy_id(proxy_key_b);

        let aabb_a = self.trees[type_a as usize].aabb(id_a);
        let aabb_b = self.trees[type_b as usize].aabb(id_b);
        aabb_overlaps(aabb_a, aabb_b)
    }

    /// (b3ValidateBroadPhase)
    pub fn validate(&self) {
        self.trees[BodyType::Dynamic as usize].validate();
        self.trees[BodyType::Kinematic as usize].validate();
    }

    /// (b3ValidateNoEnlarged — C compiles the body under B3_ENABLE_VALIDATION;
    /// here the check runs in debug builds only)
    pub fn validate_no_enlarged(&self) {
        if cfg!(debug_assertions) {
            for tree in &self.trees {
                tree.validate_no_enlarged();
            }
        }
    }
}
