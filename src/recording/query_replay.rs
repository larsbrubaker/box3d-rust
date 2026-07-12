//! Spatial-query replay helpers. Port of recording_replay.c query dispatch.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::geometry::{Capsule, PlaneResult};
use crate::id::ShapeId;
use crate::math_functions::{Aabb, Plane, Pos, Vec3, POS_ZERO, VEC3_ZERO};
use crate::recording::dispatch::RecReader;
use crate::distance::ShapeProxy;
use crate::types::{QueryFilter, RayResult};
use crate::world::{
    world_cast_mover, world_cast_ray, world_cast_ray_closest, world_cast_shape,
    world_collide_mover, world_overlap_aabb, world_overlap_shape, World,
};

#[derive(Clone, Default)]
struct RecordedHit {
    id: ShapeId,
    point: Pos,
    normal: Vec3,
    fraction: f32,
    user_material_id: u64,
    triangle_index: i32,
    child_index: i32,
    user_return_f: f32,
    user_return_b: bool,
    plane: PlaneResult,
    plane_count: i32,
}

struct ReplayCtx<'a> {
    rdr: *mut RecReader<'a>,
    hits: Vec<RecordedHit>,
    cursor: usize,
}

fn f32_differs(a: f32, b: f32) -> bool {
    a.to_bits() != b.to_bits()
}

fn vec3_differs(a: Vec3, b: Vec3) -> bool {
    f32_differs(a.x, b.x) || f32_differs(a.y, b.y) || f32_differs(a.z, b.z)
}

fn pos_differs(a: Pos, b: Pos) -> bool {
    #[cfg(feature = "double-precision")]
    {
        a.x.to_bits() != b.x.to_bits() || a.y.to_bits() != b.y.to_bits() || a.z.to_bits() != b.z.to_bits()
    }
    #[cfg(not(feature = "double-precision"))]
    {
        vec3_differs(a, b)
    }
}

fn mark_diverged(rdr: &mut RecReader<'_>) {
    rdr.diverged = true;
}

fn make_shape_id(rdr: &RecReader<'_>, recorded: ShapeId) -> ShapeId {
    rdr.make_shape_id(recorded)
}

/// (b3RecDispatch_QueryOverlapAABB)
pub fn dispatch_query_overlap_aabb(
    rdr: &mut RecReader<'_>,
    world: &mut World,
    aabb: Aabb,
    filter: QueryFilter,
) {
    let mut s = rdr.snap();
    let n = s.u32() as usize;
    let mut hits = Vec::with_capacity(n);
    for _ in 0..n {
        let id = make_shape_id(rdr, s.shape_id());
        let user_return_b = s.bool();
        hits.push(RecordedHit {
            id,
            user_return_b,
            ..Default::default()
        });
    }
    let _ = s.i32(); // nodeVisits
    let _ = s.i32(); // leafVisits
    rdr.sync_from(&s);
    if !rdr.ok {
        return;
    }

    let mut ctx = ReplayCtx {
        rdr: rdr as *mut _,
        hits,
        cursor: 0,
    };
    world_overlap_aabb(world, aabb, &filter, |id| {
        if ctx.cursor >= ctx.hits.len() {
            unsafe { mark_diverged(&mut *ctx.rdr) };
            return false;
        }
        let h = &ctx.hits[ctx.cursor];
        ctx.cursor += 1;
        if id.index1 != h.id.index1 || id.generation != h.id.generation {
            unsafe { mark_diverged(&mut *ctx.rdr) };
        }
        h.user_return_b
    });
    if ctx.cursor != ctx.hits.len() {
        rdr.diverged = true;
    }
    stash_simple(rdr, RecQueryKind::OverlapAabb, &ctx.hits, aabb, POS_ZERO, VEC3_ZERO, filter);
}

/// (b3RecDispatch_QueryOverlapShape)
pub fn dispatch_query_overlap_shape(
    rdr: &mut RecReader<'_>,
    world: &mut World,
    origin: Pos,
    proxy: ShapeProxy,
    filter: QueryFilter,
) {
    let mut s = rdr.snap();
    let n = s.u32() as usize;
    let mut hits = Vec::with_capacity(n);
    for _ in 0..n {
        let id = make_shape_id(rdr, s.shape_id());
        let user_return_b = s.bool();
        hits.push(RecordedHit {
            id,
            user_return_b,
            ..Default::default()
        });
    }
    let _ = s.i32();
    let _ = s.i32();
    rdr.sync_from(&s);
    if !rdr.ok {
        return;
    }

    let mut ctx = ReplayCtx {
        rdr: rdr as *mut _,
        hits,
        cursor: 0,
    };
    world_overlap_shape(world, origin, &proxy, &filter, |id| {
        if ctx.cursor >= ctx.hits.len() {
            unsafe { mark_diverged(&mut *ctx.rdr) };
            return false;
        }
        let h = &ctx.hits[ctx.cursor];
        ctx.cursor += 1;
        if id.index1 != h.id.index1 || id.generation != h.id.generation {
            unsafe { mark_diverged(&mut *ctx.rdr) };
        }
        h.user_return_b
    });
    if ctx.cursor != ctx.hits.len() {
        rdr.diverged = true;
    }
    stash_simple(rdr, RecQueryKind::OverlapShape, &ctx.hits, Aabb::default(), origin, VEC3_ZERO, filter);
}

/// (b3RecDispatch_QueryCastRay)
pub fn dispatch_query_cast_ray(
    rdr: &mut RecReader<'_>,
    world: &mut World,
    origin: Pos,
    translation: Vec3,
    filter: QueryFilter,
) {
    let hits = read_cast_hits(rdr);
    if !rdr.ok {
        return;
    }
    let mut ctx = ReplayCtx {
        rdr: rdr as *mut _,
        hits,
        cursor: 0,
    };
    world_cast_ray(world, origin, translation, &filter, |id, point, normal, fraction, mid, tri, child| {
        if ctx.cursor >= ctx.hits.len() {
            unsafe { mark_diverged(&mut *ctx.rdr) };
            return 0.0;
        }
        let h = &ctx.hits[ctx.cursor];
        ctx.cursor += 1;
        if id.index1 != h.id.index1
            || id.generation != h.id.generation
            || pos_differs(point, h.point)
            || vec3_differs(normal, h.normal)
            || f32_differs(fraction, h.fraction)
            || mid != h.user_material_id
            || tri != h.triangle_index
            || child != h.child_index
        {
            unsafe { mark_diverged(&mut *ctx.rdr) };
        }
        h.user_return_f
    });
    if ctx.cursor != ctx.hits.len() {
        rdr.diverged = true;
    }
    stash_simple(rdr, RecQueryKind::CastRay, &ctx.hits, Aabb::default(), origin, translation, filter);
}

/// (b3RecDispatch_QueryCastShape)
pub fn dispatch_query_cast_shape(
    rdr: &mut RecReader<'_>,
    world: &mut World,
    origin: Pos,
    proxy: ShapeProxy,
    translation: Vec3,
    filter: QueryFilter,
) {
    let hits = read_cast_hits(rdr);
    if !rdr.ok {
        return;
    }
    let mut ctx = ReplayCtx {
        rdr: rdr as *mut _,
        hits,
        cursor: 0,
    };
    world_cast_shape(
        world,
        origin,
        &proxy,
        translation,
        &filter,
        |id, point, normal, fraction, mid, tri, child| {
            if ctx.cursor >= ctx.hits.len() {
                unsafe { mark_diverged(&mut *ctx.rdr) };
                return 0.0;
            }
            let h = &ctx.hits[ctx.cursor];
            ctx.cursor += 1;
            if id.index1 != h.id.index1
                || id.generation != h.id.generation
                || pos_differs(point, h.point)
                || vec3_differs(normal, h.normal)
                || f32_differs(fraction, h.fraction)
                || mid != h.user_material_id
                || tri != h.triangle_index
                || child != h.child_index
            {
                unsafe { mark_diverged(&mut *ctx.rdr) };
            }
            h.user_return_f
        },
    );
    if ctx.cursor != ctx.hits.len() {
        rdr.diverged = true;
    }
    stash_simple(rdr, RecQueryKind::CastShape, &ctx.hits, Aabb::default(), origin, translation, filter);
}

/// (b3RecDispatch_QueryCastRayClosest)
pub fn dispatch_query_cast_ray_closest(
    rdr: &mut RecReader<'_>,
    world: &mut World,
    origin: Pos,
    translation: Vec3,
    filter: QueryFilter,
) {
    let mut s = rdr.snap();
    let rec_shape = make_shape_id(rdr, s.shape_id());
    let rec = RayResult {
        shape_id: rec_shape,
        point: s.pos(),
        normal: s.vec3(),
        user_material_id: s.u64(),
        fraction: s.f32(),
        triangle_index: s.i32(),
        child_index: s.i32(),
        hit: s.bool(),
        node_visits: 0,
        leaf_visits: 0,
    };
    rdr.sync_from(&s);
    if !rdr.ok {
        return;
    }

    let got = world_cast_ray_closest(world, origin, translation, &filter);
    if got.hit != rec.hit
        || (got.hit
            && (got.shape_id.index1 != rec.shape_id.index1
                || got.shape_id.generation != rec.shape_id.generation
                || pos_differs(got.point, rec.point)
                || vec3_differs(got.normal, rec.normal)
                || f32_differs(got.fraction, rec.fraction)
                || got.user_material_id != rec.user_material_id))
    {
        rdr.diverged = true;
    }

    let hits = if rec.hit {
        vec![RecordedHit {
            id: rec.shape_id,
            point: rec.point,
            normal: rec.normal,
            fraction: rec.fraction,
            ..Default::default()
        }]
    } else {
        Vec::new()
    };
    stash_simple(
        rdr,
        RecQueryKind::CastRayClosest,
        &hits,
        Aabb::default(),
        origin,
        translation,
        filter,
    );
}

/// (b3RecDispatch_QueryCastMover)
pub fn dispatch_query_cast_mover(
    rdr: &mut RecReader<'_>,
    world: &mut World,
    origin: Pos,
    mover: Capsule,
    translation: Vec3,
    filter: QueryFilter,
) {
    let mut s = rdr.snap();
    let n = s.u32() as usize;
    let mut hits = Vec::with_capacity(n);
    for _ in 0..n {
        let id = make_shape_id(rdr, s.shape_id());
        let user_return_b = s.bool();
        hits.push(RecordedHit {
            id,
            user_return_b,
            ..Default::default()
        });
    }
    let rec_fraction = s.f32();
    rdr.sync_from(&s);
    if !rdr.ok {
        return;
    }

    let mut ctx = ReplayCtx {
        rdr: rdr as *mut _,
        hits,
        cursor: 0,
    };
    let mut filter_fcn = |id: ShapeId| -> bool {
        if ctx.cursor >= ctx.hits.len() {
            unsafe { mark_diverged(&mut *ctx.rdr) };
            return false;
        }
        let h = &ctx.hits[ctx.cursor];
        ctx.cursor += 1;
        if id.index1 != h.id.index1 || id.generation != h.id.generation {
            unsafe { mark_diverged(&mut *ctx.rdr) };
        }
        h.user_return_b
    };
    let got = world_cast_mover(
        world,
        origin,
        &mover,
        translation,
        &filter,
        Some(&mut filter_fcn),
    );
    if ctx.cursor != ctx.hits.len() || f32_differs(got, rec_fraction) {
        rdr.diverged = true;
    }
    stash_simple(rdr, RecQueryKind::CastMover, &ctx.hits, Aabb::default(), origin, translation, filter);
}

/// (b3RecDispatch_QueryCollideMover)
pub fn dispatch_query_collide_mover(
    rdr: &mut RecReader<'_>,
    world: &mut World,
    origin: Pos,
    mover: Capsule,
    filter: QueryFilter,
) {
    let mut s = rdr.snap();
    let shape_count = s.u32() as usize;
    let mut hits = Vec::new();
    for _ in 0..shape_count {
        let id = make_shape_id(rdr, s.shape_id());
        let plane_count = s.i32().max(0);
        let mut planes = Vec::with_capacity(plane_count as usize);
        for _ in 0..plane_count {
            planes.push(PlaneResult {
                plane: Plane {
                    normal: s.vec3(),
                    offset: s.f32(),
                },
                point: s.vec3(),
            });
        }
        let user_return_b = s.bool();
        for p in &planes {
            hits.push(RecordedHit {
                id,
                plane: *p,
                plane_count,
                user_return_b,
                ..Default::default()
            });
        }
    }
    rdr.sync_from(&s);
    if !rdr.ok {
        return;
    }

    let total = hits.len();
    let mut ctx = ReplayCtx {
        rdr: rdr as *mut _,
        hits,
        cursor: 0,
    };
    world_collide_mover(world, origin, &mover, &filter, |id, planes| {
        if ctx.cursor >= ctx.hits.len() {
            unsafe { mark_diverged(&mut *ctx.rdr) };
            return true;
        }
        let head = &ctx.hits[ctx.cursor];
        let recorded_count = head.plane_count;
        let ret = head.user_return_b;
        if id.index1 != head.id.index1
            || id.generation != head.id.generation
            || recorded_count != planes.len() as i32
        {
            unsafe { mark_diverged(&mut *ctx.rdr) };
        }
        let n = recorded_count.min(planes.len() as i32) as usize;
        for i in 0..n {
            let h = &ctx.hits[ctx.cursor + i];
            if vec3_differs(h.plane.plane.normal, planes[i].plane.normal)
                || f32_differs(h.plane.plane.offset, planes[i].plane.offset)
                || vec3_differs(h.plane.point, planes[i].point)
            {
                unsafe { mark_diverged(&mut *ctx.rdr) };
            }
        }
        ctx.cursor += recorded_count as usize;
        ret
    });
    if ctx.cursor != total {
        rdr.diverged = true;
    }
    stash_simple(rdr, RecQueryKind::CollideMover, &ctx.hits, Aabb::default(), origin, VEC3_ZERO, filter);
}

fn read_cast_hits(rdr: &mut RecReader<'_>) -> Vec<RecordedHit> {
    let mut s = rdr.snap();
    let n = s.u32() as usize;
    let mut hits = Vec::with_capacity(n);
    for _ in 0..n {
        let id = make_shape_id(rdr, s.shape_id());
        hits.push(RecordedHit {
            id,
            point: s.pos(),
            normal: s.vec3(),
            fraction: s.f32(),
            user_material_id: s.u64(),
            triangle_index: s.i32(),
            child_index: s.i32(),
            user_return_f: s.f32(),
            ..Default::default()
        });
    }
    let _ = s.i32();
    let _ = s.i32();
    rdr.sync_from(&s);
    hits
}

/// Public query kind for the player frame-query store. (b3RecQueryType)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum RecQueryKind {
    OverlapAabb = 0,
    OverlapShape = 1,
    CastRay = 2,
    CastShape = 3,
    CastRayClosest = 4,
    CastMover = 5,
    CollideMover = 6,
}

/// Stashed per-frame query for the player. (b3RecDrawQuery subset)
#[derive(Clone)]
pub struct FrameQuery {
    pub kind: RecQueryKind,
    pub hit_count: i32,
    pub key: u64,
    pub filter: QueryFilter,
    pub origin: Pos,
    pub translation: Vec3,
    pub aabb: Aabb,
}

fn stash_simple(
    rdr: &mut RecReader<'_>,
    kind: RecQueryKind,
    hits: &[RecordedHit],
    aabb: Aabb,
    origin: Pos,
    translation: Vec3,
    filter: QueryFilter,
) {
    let Some(owner) = rdr.owner else {
        rdr.pending_query_key = 0;
        return;
    };
    // SAFETY: owner points at the RecPlayer driving this reader for the duration of step_frame.
    let player = unsafe { &mut *owner };
    let key = rdr.pending_query_key;
    rdr.pending_query_key = 0;
    player.frame_queries.push(FrameQuery {
        kind,
        hit_count: hits.len() as i32,
        key,
        filter,
        origin,
        translation,
        aabb,
    });
}
