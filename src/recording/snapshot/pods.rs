//! Shared POD helpers for world snapshots.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::dynamic_tree::{DynamicTree, TreeNode};
use crate::id_pool::IdPool;
use crate::recording::buffer::{RecBuffer, SnapReader};
use crate::table::{HashSet, SetItem};
use crate::types::Capacity;
use crate::world::World;

pub fn ser_id_pool(buf: &mut RecBuffer, pool: &IdPool) {
    buf.append_i32(pool.next_index);
    ser_i32_array(buf, &pool.free_array);
}

pub fn des_id_pool(r: &mut SnapReader<'_>) -> IdPool {
    let next_index = r.i32();
    let free_array = des_i32_array(r);
    IdPool {
        free_array,
        next_index,
    }
}

pub fn ser_i32_array(buf: &mut RecBuffer, arr: &[i32]) {
    buf.append_i32(arr.len() as i32);
    for &v in arr {
        buf.append_i32(v);
    }
}

pub fn des_i32_array(r: &mut SnapReader<'_>) -> Vec<i32> {
    let cnt = r.i32();
    if !r.ok || !r.check_count(cnt, 4, 4) {
        r.ok = false;
        return Vec::new();
    }
    let mut out = Vec::with_capacity(cnt.max(0) as usize);
    for _ in 0..cnt.max(0) {
        out.push(r.i32());
    }
    out
}

pub fn ser_bit_set(buf: &mut RecBuffer, bs: &BitSet) {
    buf.append_u32(bs.block_count);
    if bs.block_count > 0 {
        for i in 0..bs.block_count as usize {
            buf.append_u64(bs.blocks[i]);
        }
    }
}

pub fn des_bit_set(r: &mut SnapReader<'_>) -> BitSet {
    let block_count = r.u32();
    if r.ok && !r.check_count(block_count as i32, 8, 8) {
        r.ok = false;
    }
    if !r.ok {
        return BitSet::new(0);
    }
    let block_capacity = if block_count > 0 { block_count } else { 1 };
    let mut blocks = vec![0u64; block_capacity as usize];
    for i in 0..block_count as usize {
        blocks[i] = r.u64();
    }
    BitSet {
        blocks,
        block_count,
    }
}

pub fn ser_hash_set(buf: &mut RecBuffer, hs: &HashSet) {
    let cap = hs.items.len() as u32;
    buf.append_u32(cap);
    buf.append_u32(hs.count);
    for item in &hs.items {
        buf.append_u64(item.key);
        buf.append_u32(item.hash);
        buf.append_u32(0); // pad to 16 bytes like C SetItem
    }
}

pub fn des_hash_set(r: &mut SnapReader<'_>) -> HashSet {
    let cap = r.u32();
    let cnt = r.u32();
    let valid =
        r.check_count(cap as i32, 16, 16) && (cap == 0 || (cap & (cap - 1)) == 0) && cnt <= cap;
    if r.ok && !valid && (cap != 0 || cnt != 0) {
        r.ok = false;
    }
    if !r.ok {
        return HashSet::new(16);
    }
    if cap == 0 {
        return HashSet {
            items: Vec::new(),
            count: 0,
        };
    }
    let mut items = Vec::with_capacity(cap as usize);
    for _ in 0..cap {
        let key = r.u64();
        let hash = r.u32();
        let _pad = r.u32();
        items.push(SetItem { key, hash });
    }
    HashSet { items, count: cnt }
}

pub fn ser_tree(buf: &mut RecBuffer, tree: &DynamicTree) {
    buf.append_u64(tree.version);
    buf.append_i32(tree.root);
    buf.append_i32(tree.node_count);
    buf.append_i32(tree.nodes.len() as i32); // nodeCapacity
    buf.append_i32(tree.free_list);
    buf.append_i32(tree.proxy_count);
    for node in &tree.nodes {
        ser_tree_node(buf, node);
    }
}

fn ser_tree_node(buf: &mut RecBuffer, n: &TreeNode) {
    buf.append_aabb(n.aabb);
    buf.append_u64(n.category_bits);
    buf.append_i32(n.child1);
    buf.append_i32(n.child2);
    buf.append_u64(n.user_data);
    buf.append_i32(n.parent);
    buf.append_i32(n.next);
    buf.append_u16(n.height);
    buf.append_u16(n.flags);
}

pub fn des_tree(r: &mut SnapReader<'_>) -> DynamicTree {
    let version = r.u64();
    let root = r.i32();
    let node_count = r.i32();
    let node_capacity = r.i32();
    let free_list = r.i32();
    let proxy_count = r.i32();
    if r.ok && !r.check_count(node_capacity, 64, 64) {
        r.ok = false;
    }
    if !r.ok {
        return DynamicTree::new(0);
    }
    let mut nodes = Vec::with_capacity(node_capacity.max(0) as usize);
    for _ in 0..node_capacity.max(0) {
        nodes.push(des_tree_node(r));
    }
    DynamicTree {
        version,
        nodes,
        root,
        node_count,
        free_list,
        proxy_count,
        leaf_indices: Vec::new(),
        leaf_centers: Vec::new(),
        rebuild_capacity: 0,
    }
}

fn des_tree_node(r: &mut SnapReader<'_>) -> TreeNode {
    TreeNode {
        aabb: r.aabb(),
        category_bits: r.u64(),
        child1: r.i32(),
        child2: r.i32(),
        user_data: r.u64(),
        parent: r.i32(),
        next: r.i32(),
        height: r.u16(),
        flags: r.u16(),
    }
}

pub fn ser_world_config(buf: &mut RecBuffer, world: &World) {
    buf.append_vec3(world.gravity);
    buf.append_f32(world.hit_event_threshold);
    buf.append_f32(world.restitution_threshold);
    buf.append_f32(world.max_linear_speed);
    buf.append_f32(world.contact_speed);
    buf.append_f32(world.contact_hertz);
    buf.append_f32(world.contact_damping_ratio);
    buf.append_f32(world.contact_recycle_distance);
    buf.append_u64(world.step_index);
    buf.append_i32(world.split_island_id);
    buf.append_f32(world.inv_h);
    buf.append_f32(world.inv_dt);
    buf.append_i32(world.end_event_array_index);
    ser_capacity(buf, &world.max_capacity);
    let mut flags = 0u8;
    if world.enable_sleep {
        flags |= 0x01;
    }
    if world.enable_warm_starting {
        flags |= 0x02;
    }
    if world.enable_continuous {
        flags |= 0x04;
    }
    if world.enable_speculative {
        flags |= 0x08;
    }
    buf.append_u8(flags);
}

pub fn des_world_config(r: &mut SnapReader<'_>, world: &mut World) {
    world.gravity = r.vec3();
    world.hit_event_threshold = r.f32();
    world.restitution_threshold = r.f32();
    world.max_linear_speed = r.f32();
    world.contact_speed = r.f32();
    world.contact_hertz = r.f32();
    world.contact_damping_ratio = r.f32();
    world.contact_recycle_distance = r.f32();
    world.step_index = r.u64();
    world.split_island_id = r.i32();
    world.inv_h = r.f32();
    world.inv_dt = r.f32();
    world.end_event_array_index = r.i32();
    world.max_capacity = des_capacity(r);
    let flags = r.u8();
    world.enable_sleep = flags & 0x01 != 0;
    world.enable_warm_starting = flags & 0x02 != 0;
    world.enable_continuous = flags & 0x04 != 0;
    world.enable_speculative = flags & 0x08 != 0;
}

fn ser_capacity(buf: &mut RecBuffer, c: &Capacity) {
    buf.append_i32(c.static_shape_count);
    buf.append_i32(c.dynamic_shape_count);
    buf.append_i32(c.static_body_count);
    buf.append_i32(c.dynamic_body_count);
    buf.append_i32(c.contact_count);
}

fn des_capacity(r: &mut SnapReader<'_>) -> Capacity {
    Capacity {
        static_shape_count: r.i32(),
        dynamic_shape_count: r.i32(),
        static_body_count: r.i32(),
        dynamic_body_count: r.i32(),
        contact_count: r.i32(),
    }
}
