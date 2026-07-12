//! world ops handlers for the replay dispatcher, split from
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::recording::dispatch::RecReader;
use crate::recording::ops::RecOp;

#[allow(unused_imports)]
use crate::recording::query_replay;

/// Handle one world op. Returns false if `op` is not in this family.
pub(super) fn dispatch(
    op: RecOp,
    rdr: &mut RecReader<'_>,
    payload_start: i32,
    payload_size: u32,
) -> bool {
    let world = unsafe { &mut *rdr.world };
    let _ = world;
    match op {
        RecOp::DestroyWorld => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            rdr.sync_from(&s);
            if rdr.ok {
                // end-of-session marker
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::Step => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let dt = s.f32();
            let sub_step_count = s.i32();
            rdr.sync_from(&s);
            if rdr.ok {
                world.step(dt, sub_step_count);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldEnableSleeping => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_enable_sleeping(world, flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldEnableContinuous => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_enable_continuous(world, flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldSetRestitutionThreshold => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let value = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_restitution_threshold(world, value);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldSetHitEventThreshold => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let value = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_hit_event_threshold(world, value);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldSetGravity => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let gravity = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_gravity(world, gravity);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldExplode => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.explosion_def();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_explode(world, &def);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldSetContactTuning => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let hertz = s.f32();
            let damping_ratio = s.f32();
            let contact_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_contact_tuning(world, hertz, damping_ratio, contact_speed);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldSetContactRecycleDistance => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let recycle_distance = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_contact_recycle_distance(world, recycle_distance);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldSetMaximumLinearSpeed => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let maximum_linear_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_maximum_linear_speed(world, maximum_linear_speed);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldEnableWarmStarting => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_enable_warm_starting(world, flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldRebuildStaticTree => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_rebuild_static_tree(world);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WorldEnableSpeculative => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_enable_speculative(world, flag);
            }
            let _ = (payload_start, payload_size);
        }
        _ => return false,
    }
    true
}
