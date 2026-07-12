//! query ops handlers for the replay dispatcher, split from
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::recording::dispatch::RecReader;
use crate::recording::hash::hash_world_state;
use crate::recording::ops::RecOp;

#[allow(unused_imports)]
use crate::recording::query_replay;

/// Handle one query op. Returns false if `op` is not in this family.
pub(super) fn dispatch(
    op: RecOp,
    rdr: &mut RecReader<'_>,
    payload_start: i32,
    payload_size: u32,
) -> bool {
    let world = unsafe { &mut *rdr.world };
    let _ = world;
    match op {
        RecOp::QueryOverlapAABB => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let aabb = s.aabb();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_overlap_aabb(
                    rdr, world, aabb, filter,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::QueryOverlapShape => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let proxy = s.shape_proxy();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_overlap_shape(
                    rdr, world, origin, proxy, filter,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::QueryCastRay => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let translation = s.vec3();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_cast_ray(
                    rdr,
                    world,
                    origin,
                    translation,
                    filter,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::QueryCastShape => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let proxy = s.shape_proxy();
            let translation = s.vec3();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_cast_shape(
                    rdr,
                    world,
                    origin,
                    proxy,
                    translation,
                    filter,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::QueryCastRayClosest => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let translation = s.vec3();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_cast_ray_closest(
                    rdr,
                    world,
                    origin,
                    translation,
                    filter,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::QueryCastMover => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let mover = s.capsule();
            let translation = s.vec3();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_cast_mover(
                    rdr,
                    world,
                    origin,
                    mover,
                    translation,
                    filter,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::QueryCollideMover => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let mover = s.capsule();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_collide_mover(
                    rdr, world, origin, mover, filter,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::QueryTag => {
            let mut s = rdr.snap();
            let key = s.u64();
            rdr.sync_from(&s);
            if rdr.ok {
                rdr.pending_query_key = key;
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::StateHash => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let hash = s.u64();
            rdr.sync_from(&s);
            if rdr.ok {
                if hash_world_state(world) != hash {
                    rdr.diverged = true;
                }
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RecordingBounds => {
            let mut s = rdr.snap();
            let bounds = s.aabb();
            rdr.sync_from(&s);
            if rdr.ok {
                if let Some(owner) = rdr.owner {
                    // SAFETY: owner is the RecPlayer for this dispatch.
                    unsafe { (*owner).bounds = bounds };
                }
            }
            let _ = (payload_start, payload_size);
        }
        _ => return false,
    }
    true
}
