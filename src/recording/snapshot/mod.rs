//! World snapshot serialize/deserialize.
//! Port of world_snapshot.c — field-by-field for Vec-backed Rust pools.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod joints;
mod pods;

use crate::constants::GRAPH_COLOR_COUNT;
use crate::constraint_graph::OVERFLOW_INDEX;
use crate::island::Island;
use crate::recording::buffer::{RecBuffer, SnapReader};
use crate::recording::registry::GeometryRegistry;
use crate::sensor::Sensor;
use crate::types::{default_world_def, BODY_TYPE_COUNT};
use crate::world::World;

use pods::*;

/// Snapshot image magic 'BNS3'. (B3_SNAP_MAGIC)
pub const SNAP_MAGIC: u32 = 0x3353_4E42;
/// Snapshot format version. (B3_SNAP_VERSION)
pub const SNAP_VERSION: u32 = 1;
/// Rust field-layout version — bump when serialized record layouts change.
pub const SNAP_LAYOUT_VERSION: u32 = 1;

pub const SNAP_FLAG_VALIDATION: u32 = 0x1;
pub const SNAP_FLAG_DOUBLE_PRECISION: u32 = 0x2;

/// Layout hash for the Rust field-by-field format. (adapted from b3ComputeLayoutHash)
pub fn compute_layout_hash() -> u32 {
    let mut h = 2166136261u32;
    macro_rules! mix {
        ($x:expr) => {{
            h ^= $x as u32;
            h = h.wrapping_mul(16777619);
        }};
    }
    mix!(SNAP_LAYOUT_VERSION);
    mix!(GRAPH_COLOR_COUNT);
    mix!(BODY_TYPE_COUNT);
    mix!(core::mem::size_of::<i32>());
    mix!(core::mem::size_of::<f32>());
    mix!(core::mem::size_of::<u64>());
    #[cfg(feature = "double-precision")]
    mix!(1u32);
    #[cfg(not(feature = "double-precision"))]
    mix!(0u32);
    h
}

/// Serialize the live world into `buf`, interning shape geometry into `registry`.
/// Returns the byte count written. (b3SerializeWorld)
pub fn serialize_world(world: &World, buf: &mut RecBuffer, registry: &mut GeometryRegistry) -> i32 {
    let start = buf.size();

    buf.append_u32(SNAP_MAGIC);
    buf.append_u32(SNAP_VERSION);
    buf.append_u32(compute_layout_hash());
    let mut flags = 0u32;
    #[cfg(debug_assertions)]
    {
        flags |= SNAP_FLAG_VALIDATION;
    }
    #[cfg(feature = "double-precision")]
    {
        flags |= SNAP_FLAG_DOUBLE_PRECISION;
    }
    buf.append_u32(flags);

    ser_world_config(buf, world);

    ser_id_pool(buf, &world.body_id_pool);
    ser_id_pool(buf, &world.shape_id_pool);
    ser_id_pool(buf, &world.contact_id_pool);
    ser_id_pool(buf, &world.joint_id_pool);
    ser_id_pool(buf, &world.island_id_pool);
    ser_id_pool(buf, &world.solver_set_id_pool);

    buf.append_i32(world.solver_sets.len() as i32);
    for set in &world.solver_sets {
        ser_solver_set(buf, set);
    }

    buf.append_i32(world.bodies.len() as i32);
    for body in &world.bodies {
        ser_body(buf, body);
    }

    ser_shapes(buf, world, registry);
    ser_contacts(buf, world);

    buf.append_i32(world.joints.len() as i32);
    for joint in &world.joints {
        ser_joint(buf, joint);
    }

    buf.append_i32(world.sensors.len() as i32);
    for s in &world.sensors {
        buf.append_i32(s.shape_id);
        ser_visitors(buf, &s.hits);
        ser_visitors(buf, &s.overlaps1);
        ser_visitors(buf, &s.overlaps2);
    }

    buf.append_i32(world.islands.len() as i32);
    for island in &world.islands {
        buf.append_i32(island.set_index);
        buf.append_i32(island.local_index);
        buf.append_i32(island.island_id);
        buf.append_i32(island.constraint_remove_count);
        ser_i32_array(buf, &island.bodies);
        ser_contact_links(buf, &island.contacts);
        ser_joint_links(buf, &island.joints);
    }

    for t in 0..BODY_TYPE_COUNT {
        ser_tree(buf, &world.broad_phase.trees[t]);
    }
    for t in 0..BODY_TYPE_COUNT {
        ser_bit_set(buf, &world.broad_phase.moved_proxies[t]);
    }
    ser_i32_array(buf, &world.broad_phase.move_array);
    ser_hash_set(buf, &world.broad_phase.pair_set);

    for c in 0..GRAPH_COLOR_COUNT as usize {
        ser_graph_color(
            buf,
            &world.constraint_graph.colors[c],
            c == OVERFLOW_INDEX as usize,
        );
    }

    buf.size() - start
}

/// Overwrite a freshly-created (shell) world with the simulation state in the
/// snapshot image. Geometry references resolve via `slots`. (b3DeserializeIntoShell)
pub fn deserialize_into_shell(
    data: &[u8],
    world: &mut World,
    slots: &mut [crate::recording::registry::RegistrySlot],
) -> bool {
    if data.len() < 16 {
        return false;
    }

    let mut r = SnapReader::new(data);
    let magic = r.u32();
    let version = r.u32();
    let layout_hash = r.u32();
    let flags = r.u32();
    if magic != SNAP_MAGIC || version != SNAP_VERSION {
        return false;
    }
    let image_double = (flags & SNAP_FLAG_DOUBLE_PRECISION) != 0;
    #[cfg(feature = "double-precision")]
    let build_double = true;
    #[cfg(not(feature = "double-precision"))]
    let build_double = false;
    if image_double != build_double {
        return false;
    }
    if layout_hash != compute_layout_hash() {
        return false;
    }

    free_live_shapes(world);
    // Clear contact/sensor/island heap before overwrite
    world.contacts.clear();
    world.sensors.clear();
    world.islands.clear();

    des_world_config(&mut r, world);

    world.body_id_pool = des_id_pool(&mut r);
    world.shape_id_pool = des_id_pool(&mut r);
    world.contact_id_pool = des_id_pool(&mut r);
    world.joint_id_pool = des_id_pool(&mut r);
    world.island_id_pool = des_id_pool(&mut r);
    world.solver_set_id_pool = des_id_pool(&mut r);

    let set_count = r.i32();
    if r.ok && !r.check_count(set_count, 24, 24) {
        r.ok = false;
    }
    if !r.ok {
        return false;
    }
    world.solver_sets.clear();
    world.solver_sets.reserve(set_count.max(0) as usize);
    for _ in 0..set_count.max(0) {
        world.solver_sets.push(des_solver_set(&mut r));
    }
    if !r.ok {
        return false;
    }

    let body_count = r.i32();
    if r.ok && !r.check_count(body_count, 64, 64) {
        r.ok = false;
    }
    if !r.ok {
        return false;
    }
    world.bodies.clear();
    world.bodies.reserve(body_count.max(0) as usize);
    for _ in 0..body_count.max(0) {
        world.bodies.push(des_body(&mut r));
    }
    if !r.ok {
        return false;
    }

    des_shapes(&mut r, world, slots);
    if !r.ok {
        return false;
    }

    world.contacts = des_contacts(&mut r);
    if !r.ok {
        return false;
    }

    let joint_count = r.i32();
    if r.ok && !r.check_count(joint_count, 32, 32) {
        r.ok = false;
    }
    if !r.ok {
        return false;
    }
    world.joints.clear();
    world.joints.reserve(joint_count.max(0) as usize);
    for _ in 0..joint_count.max(0) {
        world.joints.push(des_joint(&mut r));
    }

    let sensor_count = r.i32();
    world.sensors.clear();
    world.sensors.reserve(sensor_count.max(0) as usize);
    for _ in 0..sensor_count.max(0) {
        let shape_id = r.i32();
        let hits = des_visitors(&mut r);
        let overlaps1 = des_visitors(&mut r);
        let overlaps2 = des_visitors(&mut r);
        world.sensors.push(Sensor {
            shape_id,
            hits,
            overlaps1,
            overlaps2,
        });
    }

    let island_count = r.i32();
    world.islands.clear();
    world.islands.reserve(island_count.max(0) as usize);
    for _ in 0..island_count.max(0) {
        let set_index = r.i32();
        let local_index = r.i32();
        let island_id = r.i32();
        let constraint_remove_count = r.i32();
        let bodies = des_i32_array(&mut r);
        let contacts = des_contact_links(&mut r);
        let joints = des_joint_links(&mut r);
        world.islands.push(Island {
            set_index,
            local_index,
            island_id,
            constraint_remove_count,
            bodies,
            contacts,
            joints,
        });
    }

    for t in 0..BODY_TYPE_COUNT {
        world.broad_phase.trees[t] = des_tree(&mut r);
    }
    for t in 0..BODY_TYPE_COUNT {
        world.broad_phase.moved_proxies[t] = des_bit_set(&mut r);
    }
    world.broad_phase.move_array = des_i32_array(&mut r);
    world.broad_phase.pair_set = des_hash_set(&mut r);

    for c in 0..GRAPH_COLOR_COUNT as usize {
        world.constraint_graph.colors[c] = des_graph_color(&mut r, c == OVERFLOW_INDEX as usize);
    }

    r.ok
}

/// Convenience: serialize then deserialize into a fresh shell world.
pub fn clone_world_via_snapshot(world: &World) -> Option<World> {
    let mut registry = GeometryRegistry::new();
    let mut buf = RecBuffer::new();
    serialize_world(world, &mut buf, &mut registry);
    let mut slots = registry.to_slots();
    let mut shell = World::new(&default_world_def());
    if deserialize_into_shell(&buf.data, &mut shell, &mut slots) {
        Some(shell)
    } else {
        None
    }
}
