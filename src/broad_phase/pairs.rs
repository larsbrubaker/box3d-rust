// The broad-phase pair finder from broad_phase.c (b3UpdateBroadPhasePairs and helpers).
//
// The dynamic body tree is collided against itself (moved sibling pairs) and against the
// static and kinematic trees (seeded tree-against-tree traversal). Candidate pairs are
// batched, culled against the pair set, filtered, and collected as 64-bit pair keys. The
// keys are sorted and the contacts are created in sorted order.
//
// The C runs the cross and self traversals through b3ParallelFor, pushing into per-worker
// `pairKeys` arrays. The port is serial and mirrors C's default task system through
// `parallel_for`: the same block partition (from `world.worker_count` and the task's
// minRange), every block run in order on worker 0. The block partition is observable:
// each task flushes its candidate batch at the end of every block (and between the static
// and kinematic sub-ranges of a cross block), and a flush expands compound candidates
// before filtering plain ones, so the block boundaries fix the order of custom filter
// calls. The generated key set itself does not depend on the partition, and the keys are
// sorted before any contact is created.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::body::get_body_transform;
use crate::compound::get_compound_tree_view;
use crate::constants::{CHILD_MASK, SHAPE_MASK, SHAPE_POWER};
use crate::core::NULL_INDEX;
use crate::dynamic_tree::{
    overlap_aabb_v, overlap_v, DynamicTree, TreeNode, DEFAULT_MASK_BITS, MOVED_NODE, ROOT_NODE,
    TREE_STACK_SIZE,
};
use crate::geometry::ShapeType;
use crate::id::ShapeId;
use crate::math_functions::{
    aabb_transform, invert_transform, max_int, min_int, to_relative_transform, POS_ZERO,
};
use crate::parallel_for::parallel_for;
use crate::shape::{shape_flags, ShapeGeometry};
use crate::table::{key_hash, shape_pair_key};
use crate::types::BodyType;
use crate::world::World;

use super::{proxy_id, proxy_type};

/// (B3_CANDIDATE_BATCH)
const CANDIDATE_BATCH: usize = 32;

/// (B3_CROSS_SEED_COUNT)
const CROSS_SEED_COUNT: usize = 64;
const _: () = assert!(
    (CROSS_SEED_COUNT & (CROSS_SEED_COUNT - 1)) == 0,
    "must be power of 2"
);

/// (b3CandidatePair)
#[derive(Clone, Copy, Default)]
struct CandidatePair {
    shape_id_a: i32,
    shape_id_b: i32,
}

/// (b3PairContext)
struct PairContext<'a> {
    world: &'a World,
    pair_keys: &'a mut Vec<u64>,
    batch: [CandidatePair; CANDIDATE_BATCH],
    batch_count: usize,

    // Compound shapes are static only, so only the static cross pass can meet one.
    // Elsewhere this stays false and the batch never reads a shape type.
    check_compounds: bool,
}

impl<'a> PairContext<'a> {
    fn new(world: &'a World, pair_keys: &'a mut Vec<u64>) -> PairContext<'a> {
        PairContext {
            world,
            pair_keys,
            batch: [CandidatePair::default(); CANDIDATE_BATCH],
            batch_count: 0,
            check_compounds: false,
        }
    }
}

/// (b3NodePair)
#[derive(Clone, Copy)]
struct NodePair {
    a: TreeNode,
    b: TreeNode,
}

/// (b3IndexPair)
#[derive(Clone, Copy, Default)]
struct IndexPair {
    a: i32,
    b: i32,
}

/// Gather the sibling pairs with a moved node. This is done serially, it is cache
/// friendly. (static b3GatherMovedSiblings)
fn gather_moved_siblings(tree: &DynamicTree, pair_indices: &mut Vec<i32>) -> usize {
    let nodes = &tree.nodes;
    let node_end = tree.node_end;

    pair_indices.clear();

    // Skip the root.
    let mut pair = 2;
    while pair < node_end {
        // Push when either sibling moved.
        if (nodes[pair as usize].flag_index | nodes[pair as usize + 1].flag_index) & MOVED_NODE != 0
        {
            pair_indices.push(pair);
        }
        pair += 2;
    }

    pair_indices.len()
}

/// The shape pair gauntlet. Everything here is a property of the pair, not of a compound
/// child, so a compound runs it once for the whole pair. (static b3ShouldCreatePair)
fn should_create_pair(world: &World, shape_id_a: i32, shape_id_b: i32) -> bool {
    let shape_a = &world.shapes[shape_id_a as usize];
    let shape_b = &world.shapes[shape_id_b as usize];

    let body_id_a = shape_a.body_id;
    let body_id_b = shape_b.body_id;

    // Are the shapes on the same body?
    if body_id_a == body_id_b {
        return false;
    }

    // Sensors are handled elsewhere
    if shape_a.sensor_index != NULL_INDEX || shape_b.sensor_index != NULL_INDEX {
        return false;
    }

    if !crate::shape::should_shapes_collide(shape_a.filter, shape_b.filter) {
        return false;
    }

    // Does a joint override collision?
    if !crate::body::should_bodies_collide(world, body_id_a, body_id_b) {
        return false;
    }

    // Custom user filter
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
            if !custom_filter_fcn(world, id_a, id_b, world.custom_filter_context) {
                return false;
            }
        }
    }

    true
}

/// One of the two shapes is a compound, so the pair becomes one pair per overlapping
/// child. The child boxes live in the compound frame, so the other shape's box has to be
/// demoted and pulled into that frame first. (static b3EmitCompoundPairs, with the
/// b3CompoundChildCallback body inlined into the query closure)
fn emit_compound_pairs(context: &mut PairContext<'_>, compound_shape_id: i32, other_shape_id: i32) {
    let world = context.world;

    let compound_shape = &world.shapes[compound_shape_id as usize];
    let other_shape = &world.shapes[other_shape_id as usize];

    let other_proxy_key = other_shape.proxy_key;
    debug_assert!(other_proxy_key != NULL_INDEX);
    let other_aabb = world.broad_phase.trees[proxy_type(other_proxy_key) as usize]
        .aabb(proxy_id(other_proxy_key));

    // Query bounds are float world space, so the demoted transform is the matching float
    // frame
    let compound_transform =
        to_relative_transform(get_body_transform(world, compound_shape.body_id), POS_ZERO);
    let local_aabb = aabb_transform(invert_transform(compound_transform), other_aabb);

    let pair_set = &world.broad_phase.pair_set;

    let start_count = context.pair_keys.len();

    if let ShapeGeometry::Compound(compound) = &compound_shape.geometry {
        let pair_keys = &mut *context.pair_keys;
        let view = get_compound_tree_view(compound);
        view.query(
            local_aabb,
            DEFAULT_MASK_BITS,
            false,
            |_proxy_id, user_data| {
                let child_index = user_data as i32;

                let key = shape_pair_key(compound_shape_id, other_shape_id, child_index);
                if !pair_set.contains_key(key) {
                    pair_keys.push(key);
                }

                true
            },
        );
    }

    if context.pair_keys.len() > start_count
        && !should_create_pair(world, compound_shape_id, other_shape_id)
    {
        context.pair_keys.truncate(start_count);
    }
}

/// (static b3FlushCandidatePairs)
fn flush_candidate_pairs(context: &mut PairContext<'_>) {
    let world = context.world;
    let bp = &world.broad_phase;

    let count1 = context.batch_count;
    context.batch_count = 0;

    let mut plain = [CandidatePair::default(); CANDIDATE_BATCH];
    let mut plain_count = 0usize;

    if context.check_compounds {
        // A compound expands into one pair per child, so its key is not known yet and it
        // cannot ride the batched cull below.
        for i in 0..count1 {
            let candidate = context.batch[i];
            let compound_a =
                world.shapes[candidate.shape_id_a as usize].shape_type() == ShapeType::Compound;
            let compound_b =
                world.shapes[candidate.shape_id_b as usize].shape_type() == ShapeType::Compound;

            if compound_a {
                // Compound versus compound is not supported
                debug_assert!(!compound_b);
                emit_compound_pairs(context, candidate.shape_id_a, candidate.shape_id_b);
            } else if compound_b {
                emit_compound_pairs(context, candidate.shape_id_b, candidate.shape_id_a);
            } else {
                plain[plain_count] = candidate;
                plain_count += 1;
            }
        }
    } else {
        plain[..count1].copy_from_slice(&context.batch[..count1]);
        plain_count = count1;
    }

    // C prefetches the hash set entries here (less than 1% gain in Box2D but the 3D shape
    // is fatter). Prefetch is a hint with no semantic effect, so the port only computes
    // the keys and hashes.
    let mut keys = [0u64; CANDIDATE_BATCH];
    let mut hashes = [0u32; CANDIDATE_BATCH];
    for i in 0..plain_count {
        keys[i] = shape_pair_key(plain[i].shape_id_a, plain[i].shape_id_b, 0);
        hashes[i] = key_hash(keys[i]);
    }

    // Cull existing pairs.
    let mut candidates = [CandidatePair::default(); CANDIDATE_BATCH];
    let mut count2 = 0usize;
    for i in 0..plain_count {
        if !bp.pair_set.contains_hashed_key(keys[i], hashes[i]) {
            candidates[count2] = plain[i];
            keys[count2] = keys[i];
            count2 += 1;
        }
    }

    // C prefetches the shapes here. No-op in the port.

    // Filter candidates.
    for i in 0..count2 {
        if should_create_pair(world, candidates[i].shape_id_a, candidates[i].shape_id_b) {
            // The pair passed the gauntlet. A new contact will be created.
            context.pair_keys.push(keys[i]);
        }
    }
}

/// (b3AddCandidatePair)
#[inline]
fn add_candidate_pair(shape_id_a: i32, shape_id_b: i32, context: &mut PairContext<'_>) {
    // Follow shape index order.
    let candidate = &mut context.batch[context.batch_count];
    candidate.shape_id_a = min_int(shape_id_a, shape_id_b);
    candidate.shape_id_b = max_int(shape_id_a, shape_id_b);
    context.batch_count += 1;
    if context.batch_count == CANDIDATE_BATCH {
        flush_candidate_pairs(context);
    }
}

/// Did either move and if so do they overlap? (b3TestPair)
#[inline]
fn test_pair(a: &TreeNode, b: &TreeNode) -> bool {
    if ((a.flag_index | b.flag_index) & MOVED_NODE) == 0 {
        return false;
    }

    overlap_v(&a.aabb, &b.aabb)
}

/// (static b3CollideProxyAndSubtree)
fn collide_proxy_and_subtree(
    proxy: &TreeNode,
    nodes: &[TreeNode],
    mut pair: i32,
    context: &mut PairContext<'_>,
) {
    let proxy_mark = proxy.flag_index & MOVED_NODE;
    let boxv = proxy.aabb;
    let shape_id = proxy.shape_index();

    let mut stack = [0i32; TREE_STACK_SIZE];
    let mut stack_count = 0usize;
    stack[stack_count] = pair;
    stack_count += 1;

    while stack_count > 0 {
        stack_count -= 1;
        pair = stack[stack_count];
        for i in 0..2 {
            let node = &nodes[pair as usize + i];
            if ((node.flag_index | proxy_mark) & MOVED_NODE) == 0 {
                continue;
            }

            if !overlap_aabb_v(&boxv, &node.aabb) {
                continue;
            }

            if node.is_leaf() {
                add_candidate_pair(shape_id, node.shape_index(), context);
            } else if stack_count < TREE_STACK_SIZE {
                stack[stack_count] = node.left_child();
                stack_count += 1;
            } else {
                debug_assert!(stack_count < TREE_STACK_SIZE);
            }
        }
    }
}

/// Helper for collide_cross_pairs to avoid code duplication. (b3VisitPair)
#[inline]
fn visit_pair(
    array_a: &[TreeNode],
    array_b: &[TreeNode],
    node_a: &TreeNode,
    node_b: &TreeNode,
    stack: &mut [IndexPair; TREE_STACK_SIZE],
    stack_count: &mut usize,
    context: &mut PairContext<'_>,
) {
    if !test_pair(node_a, node_b) {
        return;
    }

    let leaf_a = node_a.is_leaf();
    let leaf_b = node_b.is_leaf();
    if leaf_a && leaf_b {
        add_candidate_pair(node_a.shape_index(), node_b.shape_index(), context);
    } else if leaf_a {
        collide_proxy_and_subtree(node_a, array_b, node_b.left_child(), context);
    } else if leaf_b {
        collide_proxy_and_subtree(node_b, array_a, node_a.left_child(), context);
    } else if *stack_count < TREE_STACK_SIZE {
        stack[*stack_count] = IndexPair {
            a: node_a.left_child(),
            b: node_b.left_child(),
        };
        *stack_count += 1;
    } else {
        debug_assert!(*stack_count < TREE_STACK_SIZE);
    }
}

/// This collides two sub-trees against each other. They can live in the same dynamic
/// tree. This can only generate pairs cross sub-tree, but not within a sub-tree. This fact
/// means this does not generate duplicate pairs.
/// For example consider the full binary tree A (B (D  E) C (F G))
/// Colliding children of A (B and C) can give pairs (D,F) (D,G) (E,F) and (E,G).
/// Then colliding children of B can give the pair (D,E) and for C (F,G).
/// So no duplicates even when used for self-collision.
/// Whenever a proxy is moved, the flag is propagated up the hierarchy to the root. So
/// self collision gathers all those moved internal nodes and collides their subtrees
/// together. See Real-time collision detection section 6.3.2. This is faster than querying
/// every moved proxy against the whole tree. Scaling is linear instead of linear * log.
/// Many other physics engines do this (Bepu, Rapier, etc). So nothing new here.
/// (static b3CollideCrossPairs)
fn collide_cross_pairs(
    array_a: &[TreeNode],
    array_b: &[TreeNode],
    subtree_a: &TreeNode,
    subtree_b: &TreeNode,
    context: &mut PairContext<'_>,
) {
    let mut stack = [IndexPair::default(); TREE_STACK_SIZE];
    let mut stack_count = 0usize;

    // Seed the stack.
    visit_pair(
        array_a,
        array_b,
        subtree_a,
        subtree_b,
        &mut stack,
        &mut stack_count,
        context,
    );

    while stack_count > 0 {
        stack_count -= 1;
        let pair = stack[stack_count];
        for i in 0..2 {
            for j in 0..2 {
                visit_pair(
                    array_a,
                    array_b,
                    &array_a[pair.a as usize + i],
                    &array_b[pair.b as usize + j],
                    &mut stack,
                    &mut stack_count,
                    context,
                );
            }
        }
    }
}

/// This takes moved internal nodes and collides their sub-trees against each other.
/// (static b3SelfPairsTask)
fn self_pairs_task(
    world: &World,
    sibling_indices: &[i32],
    start_index: usize,
    end_index: usize,
    pair_keys: &mut Vec<u64>,
) {
    let tree = &world.broad_phase.trees[BodyType::Dynamic as usize];
    let nodes = &tree.nodes[..];

    let mut pair_context = PairContext::new(world, pair_keys);

    for &node_index in &sibling_indices[start_index..end_index] {
        collide_cross_pairs(
            nodes,
            nodes,
            &nodes[node_index as usize],
            &nodes[node_index as usize + 1],
            &mut pair_context,
        );
    }

    flush_candidate_pairs(&mut pair_context);
}

/// This does a serial cross-tree breadth first search until the queue is full. Then it
/// returns the queue pairs as seeds for a parallel search. (static b3GatherCrossSeeds)
fn gather_cross_seeds(tree_a: &DynamicTree, tree_b: &DynamicTree, seeds: &mut [NodePair]) -> usize {
    let nodes_a = &tree_a.nodes;
    let nodes_b = &tree_b.nodes;

    // Breadth-first search from the two roots as one pair. An empty tree has a sentinel
    // root that survives nothing.
    let empty = NodePair {
        a: TreeNode::zeroed(),
        b: TreeNode::zeroed(),
    };
    let mut queue = [empty; 2 * CROSS_SEED_COUNT];
    let mask = 2 * CROSS_SEED_COUNT - 1;
    let mut head = 0usize;
    let mut tail = 0usize;

    let root_a = &nodes_a[ROOT_NODE as usize];
    let root_b = &nodes_b[ROOT_NODE as usize];
    if test_pair(root_a, root_b) {
        queue[tail & mask] = NodePair {
            a: *root_a,
            b: *root_b,
        };
        tail += 1;
    }

    let mut seed_count = 0usize;
    while head < tail && seed_count + (tail - head) + 3 < CROSS_SEED_COUNT {
        let pair = queue[head & mask];
        head += 1;

        if pair.a.is_leaf() || pair.b.is_leaf() {
            seeds[seed_count] = pair;
            seed_count += 1;
            continue;
        }

        let a = pair.a.left_child() as usize;
        let b = pair.b.left_child() as usize;

        // Nodes have two children each, so four combinations.
        for i in 0..2 {
            for j in 0..2 {
                if test_pair(&nodes_a[a + i], &nodes_b[b + j]) {
                    queue[tail & mask] = NodePair {
                        a: nodes_a[a + i],
                        b: nodes_b[b + j],
                    };
                    tail += 1;
                }
            }
        }
    }

    while head < tail {
        seeds[seed_count] = queue[head & mask];
        seed_count += 1;
        head += 1;
    }

    seed_count
}

/// (b3CrossContext)
struct CrossContext<'a> {
    world: &'a World,
    seeds: &'a [NodePair],
    static_seed_count: usize,
    check_compounds: bool,
}

/// (static b3CrossPairsTask)
fn cross_pairs_task(
    cross_context: &CrossContext<'_>,
    start_index: usize,
    end_index: usize,
    pair_keys: &mut Vec<u64>,
) {
    let world = cross_context.world;
    let bp = &world.broad_phase;
    let static_nodes = &bp.trees[BodyType::Static as usize].nodes[..];
    let kinematic_nodes = &bp.trees[BodyType::Kinematic as usize].nodes[..];
    let dynamic_nodes = &bp.trees[BodyType::Dynamic as usize].nodes[..];

    let mut pair_context = PairContext::new(world, pair_keys);

    // Static seeds come first, so the two passes are contiguous sub-ranges. The batch is
    // drained between them because only the static pass can meet a compound.
    let static_end = end_index.min(cross_context.static_seed_count);

    pair_context.check_compounds = cross_context.check_compounds;
    for i in start_index..static_end {
        let seed = cross_context.seeds[i];
        collide_cross_pairs(
            dynamic_nodes,
            static_nodes,
            &seed.a,
            &seed.b,
            &mut pair_context,
        );
    }
    flush_candidate_pairs(&mut pair_context);

    pair_context.check_compounds = false;
    for i in start_index.max(cross_context.static_seed_count)..end_index {
        let seed = cross_context.seeds[i];
        collide_cross_pairs(
            dynamic_nodes,
            kinematic_nodes,
            &seed.a,
            &seed.b,
            &mut pair_context,
        );
    }
    flush_candidate_pairs(&mut pair_context);
}

/// Rebuild the collision tree for dynamic and kinematic bodies to keep their query
/// performance good. (static b3UpdateTreesTask)
fn update_trees_task(world: &mut World) {
    world.broad_phase.trees[BodyType::Dynamic as usize].rebuild(false);
    world.broad_phase.trees[BodyType::Kinematic as usize].rebuild(false);
}

/// Task that can be done in parallel with the narrow-phase
/// - rebuild the collision tree for dynamic and kinematic bodies to keep their query
///   performance good
///
/// The port is serial, so this runs the task inline, like C's default task system
/// (b3DefaultAddTaskFcn runs the task and returns NULL). (static b3EnqueueTreeUpdate)
fn enqueue_tree_update(world: &mut World) {
    update_trees_task(world);
}

/// Find new proxy pairs and create contacts in sorted pair-key order.
/// (b3UpdateBroadPhasePairs — serial)
pub fn update_broad_phase_pairs(world: &mut World) {
    {
        let bp = &world.broad_phase;
        let mut need_update = bp.trees[BodyType::Static as usize].has_tree_moved();
        need_update = need_update || bp.trees[BodyType::Kinematic as usize].needs_rebuild();
        need_update = need_update || bp.trees[BodyType::Dynamic as usize].needs_rebuild();

        if !need_update {
            return;
        }
    }

    for task_context in &mut world.task_contexts {
        task_context.pair_keys.clear();
    }

    // The serial tasks all run as worker 0. The worker's key array and the moved sibling
    // scratch are detached while the world is borrowed for the traversal.
    let mut worker_keys = std::mem::take(&mut world.task_contexts[0].pair_keys);
    let mut moved_siblings = std::mem::take(&mut world.broad_phase.moved_siblings);

    // Generate pairs by querying the dynamic body tree against itself and against the
    // kinematic and static trees.
    {
        let world_ref: &World = world;
        let bp = &world_ref.broad_phase;

        // Get the sibling pairs of the dynamic body tree that have moved.
        let dynamic_tree = &bp.trees[BodyType::Dynamic as usize];
        let pair_capacity = max_int(dynamic_tree.node_end / 2, 1);
        moved_siblings.reserve(pair_capacity as usize);
        let dynamic_move_count = gather_moved_siblings(dynamic_tree, &mut moved_siblings);

        // Get seeds for colliding against the static and kinematic trees.
        let mut cross_seeds = [NodePair {
            a: TreeNode::zeroed(),
            b: TreeNode::zeroed(),
        }; 2 * CROSS_SEED_COUNT];
        let static_seed_count = gather_cross_seeds(
            dynamic_tree,
            &bp.trees[BodyType::Static as usize],
            &mut cross_seeds,
        );
        debug_assert!(static_seed_count <= CROSS_SEED_COUNT);
        let kinematic_seed_count = gather_cross_seeds(
            dynamic_tree,
            &bp.trees[BodyType::Kinematic as usize],
            &mut cross_seeds[static_seed_count..],
        );
        debug_assert!(kinematic_seed_count <= CROSS_SEED_COUNT);
        let cross_move_count = static_seed_count + kinematic_seed_count;

        // Collide the dynamic body tree against the static and kinematic trees.
        let cross_context = CrossContext {
            world: world_ref,
            seeds: &cross_seeds,
            static_seed_count,
            check_compounds: world_ref.compound_shape_count > 0,
        };
        // The serial parallel-for runs every block on worker 0, whose keys are detached
        // above.
        let worker_count = world_ref.worker_count;
        parallel_for(
            worker_count,
            cross_move_count as i32,
            1,
            |start, end, worker_index| {
                debug_assert!(worker_index == 0);
                cross_pairs_task(
                    &cross_context,
                    start as usize,
                    end as usize,
                    &mut worker_keys,
                );
            },
        );

        // Collide the dynamic body tree against itself.
        parallel_for(
            worker_count,
            dynamic_move_count as i32,
            64,
            |start, end, worker_index| {
                debug_assert!(worker_index == 0);
                self_pairs_task(
                    world_ref,
                    &moved_siblings,
                    start as usize,
                    end as usize,
                    &mut worker_keys,
                );
            },
        );
    }

    world.broad_phase.trees[BodyType::Static as usize].clear_moved();

    // Update stale trees.
    enqueue_tree_update(world);

    world.task_contexts[0].pair_keys = worker_keys;

    // Pairs arrive in deterministic order but scrambled relative to body and shape order.
    // Sorting them here improves solver performance and makes contact order independent
    // of tree structure.
    let pair_count: usize = world
        .task_contexts
        .iter()
        .map(|task_context| task_context.pair_keys.len())
        .sum();

    let mut pair_keys: Vec<u64> = Vec::with_capacity(pair_count.max(1));
    for task_context in &world.task_contexts {
        pair_keys.extend_from_slice(&task_context.pair_keys);
    }

    debug_assert!(pair_keys.len() == pair_count);

    // C sorts with QSORT (an introsort) on the raw u64 keys with `<`. The keys are plain
    // integers, so equal keys are indistinguishable and every correct sort yields the
    // same sequence; an unstable std sort is equivalent.
    pair_keys.sort_unstable();

    for i in 0..pair_keys.len() {
        let key = pair_keys[i];

        // The traversal is cross subtree and the seeds are disjoint, so a duplicate key
        // means a real bug. Trap it in a validation build, and skip rather than double add
        // to the pair set, which a later destroy would then remove once.
        let duplicate = i > 0 && key == pair_keys[i - 1];
        debug_assert!(!duplicate);
        if duplicate {
            continue;
        }

        let shape_id_a = ((key >> (64 - SHAPE_POWER)) & SHAPE_MASK) as i32;
        let shape_id_b = ((key >> (64 - 2 * SHAPE_POWER)) & SHAPE_MASK) as i32;
        let child_index = (key & CHILD_MASK) as i32;

        crate::contact::create_contact(world, shape_id_a, shape_id_b, child_index);
    }

    moved_siblings.clear();
    world.broad_phase.moved_siblings = moved_siblings;

    world.validate_solver_sets();
}
