// Port of box3d-cpp-reference/src/broad_phase.h and broad_phase.c: storage,
// proxy operations, move buffering, overlap testing, validation, and serial
// update_broad_phase_pairs.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::core::NULL_INDEX;
use crate::dynamic_tree::{DynamicTree, DEFAULT_MASK_BITS};
use crate::geometry::ShapeType;
use crate::id::ShapeId;
use crate::math_functions::{aabb_overlaps, max_int, Aabb};
use crate::shape::shape_flags;
use crate::table::{shape_pair_key, HashSet};
use crate::types::{BodyType, Capacity, BODY_TYPE_COUNT};
use crate::world::World;

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

    /// Invariant: bit set in movedProxies[type] iff proxyKey is in moveArray.
    pub fn validate_moved_proxies(&self) {
        if cfg!(debug_assertions) {
            for &proxy_key_ in &self.move_array {
                let proxy_type_ = proxy_type(proxy_key_);
                let proxy_id_ = proxy_id(proxy_key_);
                debug_assert!(self.moved_proxies[proxy_type_ as usize].get_bit(proxy_id_ as u32));
            }

            let mut total_set_bits = 0;
            for i in 0..BODY_TYPE_COUNT {
                total_set_bits += self.moved_proxies[i].count_set_bits();
            }
            debug_assert!(total_set_bits == self.move_array.len() as i32);
        }
    }
}

/// Query one tree for new pairs against a moved proxy.
/// (b3PairQueryCallback — serial; compound child recursion deferred)
fn query_tree_for_pairs(
    world: &World,
    tree_type: BodyType,
    query_proxy_key: i32,
    query_shape_index: i32,
    fat_aabb: Aabb,
    pair_list: &mut Vec<(i32, i32, i32)>,
) {
    let query_proxy_type = proxy_type(query_proxy_key);

    world.broad_phase.trees[tree_type as usize].query(
        fat_aabb,
        DEFAULT_MASK_BITS,
        false,
        |proxy_id_, user_data| {
            let shape_index = user_data as i32;
            if shape_index == query_shape_index {
                return true;
            }

            // Compound child pairing lands with compound world attach.
            if world.shapes[shape_index as usize].shape_type() == ShapeType::Compound {
                return true;
            }

            let proxy_key_ = proxy_key(proxy_id_, tree_type);
            debug_assert!(proxy_key_ != query_proxy_key);

            let bp = &world.broad_phase;

            // De-duplication when both proxies are moving.
            if query_proxy_type == BodyType::Dynamic {
                if tree_type == BodyType::Dynamic && proxy_key_ < query_proxy_key {
                    if bp.moved_proxies[tree_type as usize].get_bit(proxy_id_ as u32) {
                        return true;
                    }
                }
            } else {
                debug_assert!(tree_type == BodyType::Dynamic);
                if bp.moved_proxies[tree_type as usize].get_bit(proxy_id_ as u32) {
                    return true;
                }
            }

            let child_index = 0;
            let pair_key = shape_pair_key(shape_index, query_shape_index, child_index);
            if bp.pair_set.contains_key(pair_key) {
                return true;
            }

            let shape_id_a = shape_index;
            let shape_id_b = query_shape_index;
            let shape_a = &world.shapes[shape_id_a as usize];
            let shape_b = &world.shapes[shape_id_b as usize];
            let body_id_a = shape_a.body_id;
            let body_id_b = shape_b.body_id;

            if body_id_a == body_id_b {
                return true;
            }

            if shape_a.sensor_index != NULL_INDEX || shape_b.sensor_index != NULL_INDEX {
                return true;
            }

            if !crate::shape::should_shapes_collide(shape_a.filter, shape_b.filter) {
                return true;
            }

            if !crate::body::should_bodies_collide(world, body_id_a, body_id_b) {
                return true;
            }

            if (shape_a.flags & shape_flags::ENABLE_CUSTOM_FILTERING) != 0
                || (shape_b.flags & shape_flags::ENABLE_CUSTOM_FILTERING) != 0
            {
                if let Some(custom_filter_fcn) = world.custom_filter_fcn {
                    let id_a = ShapeId {
                        index1: shape_id_a + 1,
                        world0: world.world_id,
                        generation: shape_a.generation,
                    };
                    let id_b = ShapeId {
                        index1: shape_id_b + 1,
                        world0: world.world_id,
                        generation: shape_b.generation,
                    };
                    if !custom_filter_fcn(id_a, id_b, world.custom_filter_context) {
                        return true;
                    }
                }
            }

            if !crate::contact::can_collide(shape_a.shape_type(), shape_b.shape_type()) {
                return true;
            }

            pair_list.push((shape_id_a, shape_id_b, child_index));
            true
        },
    );
}

/// Find new proxy pairs and create contacts in deterministic move-array order.
/// (b3UpdateBroadPhasePairs — serial)
pub fn update_broad_phase_pairs(world: &mut World) {
    world.broad_phase.validate_moved_proxies();

    let move_count = world.broad_phase.move_array.len();
    if move_count == 0 {
        return;
    }

    let mut move_results: Vec<Vec<(i32, i32, i32)>> = Vec::with_capacity(move_count);
    for i in 0..move_count {
        let mut pair_list: Vec<(i32, i32, i32)> = Vec::new();
        let proxy_key_ = world.broad_phase.move_array[i];
        if proxy_key_ == NULL_INDEX {
            move_results.push(pair_list);
            continue;
        }

        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);
        let base_tree = &world.broad_phase.trees[proxy_type_ as usize];
        let fat_aabb = base_tree.aabb(proxy_id_);
        let query_shape_index = base_tree.user_data(proxy_id_) as i32;

        debug_assert!(
            world.shapes[query_shape_index as usize].shape_type() != ShapeType::Compound
        );

        if proxy_type_ == BodyType::Dynamic {
            query_tree_for_pairs(
                world,
                BodyType::Kinematic,
                proxy_key_,
                query_shape_index,
                fat_aabb,
                &mut pair_list,
            );
            query_tree_for_pairs(
                world,
                BodyType::Static,
                proxy_key_,
                query_shape_index,
                fat_aabb,
                &mut pair_list,
            );
        }

        query_tree_for_pairs(
            world,
            BodyType::Dynamic,
            proxy_key_,
            query_shape_index,
            fat_aabb,
            &mut pair_list,
        );

        move_results.push(pair_list);
    }

    world.broad_phase.trees[BodyType::Dynamic as usize].rebuild(false);
    world.broad_phase.trees[BodyType::Kinematic as usize].rebuild(false);

    // C prepends to a linked list then walks head-first (= reverse discovery).
    for pair_list in &move_results {
        for &(shape_id_a, shape_id_b, child_index) in pair_list.iter().rev() {
            crate::contact::create_contact(world, shape_id_a, shape_id_b, child_index);
        }
    }

    for i in 0..world.broad_phase.move_array.len() {
        let proxy_key_ = world.broad_phase.move_array[i];
        let proxy_type_ = proxy_type(proxy_key_);
        let proxy_id_ = proxy_id(proxy_key_);
        world.broad_phase.moved_proxies[proxy_type_ as usize].clear_bit(proxy_id_ as u32);
    }
    world.broad_phase.move_array.clear();

    world.validate_solver_sets();
}
