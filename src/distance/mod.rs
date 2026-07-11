// Port of box3d-cpp-reference/src/distance.c and the distance/query group of
// include/box3d/collision.h and types.h.
//
// Split to satisfy the 800-line file limit:
// - types.rs    — proxies, simplex cache, distance/cast/TOI inputs and outputs
// - simplex.rs  — barycentric coords and simplex region solvers
// - gjk.rs      — GJK distance (b3ShapeDistance) and support functions
// - cast.rs     — linear shape cast and sweep evaluation
// - toi.rs      — time of impact via local separating axes
//
// ShapeProxy owns its point buffer (C holds a const pointer). Use [`make_proxy`]
// to build one from a slice.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

mod cast;
mod gjk;
mod simplex;
mod toi;
mod types;

pub use cast::{get_sweep_transform, shape_cast};
pub use gjk::{get_point_support, get_proxy_support, shape_distance};
pub use toi::time_of_impact;
pub use types::{
    CastOutput, DistanceInput, DistanceOutput, ShapeCastPairInput, ShapeProxy, Simplex,
    SimplexCache, SimplexVertex, Sweep, ToiInput, ToiOutput, ToiState,
};

use crate::constants::MAX_SHAPE_CAST_POINTS;
use crate::math_functions::{
    add, invert_transform, make_aabb, make_matrix_from_quat, min_int, mul_mv, Aabb, Transform, Vec3,
};

/// Make a proxy for use in overlap, shape cast, and related functions. This is
/// a deep copy of the points. (box2d-style helper; C uses a pointer in
/// `b3ShapeProxy`)
pub fn make_proxy(points: &[Vec3], radius: f32) -> ShapeProxy {
    let count = min_int(points.len() as i32, MAX_SHAPE_CAST_POINTS as i32);
    let mut proxy = ShapeProxy {
        count,
        radius,
        ..Default::default()
    };
    proxy.points[..count as usize].copy_from_slice(&points[..count as usize]);
    proxy
}

/// Transform a proxy into the local frame of `transform`.
/// (shape.c: b3MakeLocalProxy)
pub fn make_local_proxy(proxy: &ShapeProxy, transform: Transform) -> ShapeProxy {
    let inv_transform = invert_transform(transform);
    let r = make_matrix_from_quat(inv_transform.q);

    let count = min_int(proxy.count, MAX_SHAPE_CAST_POINTS as i32);
    let mut local = ShapeProxy {
        count,
        radius: proxy.radius,
        ..Default::default()
    };
    for i in 0..count as usize {
        local.points[i] = add(mul_mv(r, proxy.points[i]), inv_transform.p);
    }
    local
}

/// Compute the AABB of a shape proxy. (shape.c: b3ComputeProxyAABB)
pub fn compute_proxy_aabb(proxy: &ShapeProxy) -> Aabb {
    debug_assert!(proxy.count > 0);
    make_aabb(&proxy.points[..proxy.count as usize], proxy.radius)
}
