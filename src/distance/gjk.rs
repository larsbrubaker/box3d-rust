// GJK distance (b3ShapeDistance) and support functions from distance.c.
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::simplex::{
    compute_witness_points, get_metric, solve_simplex2, solve_simplex3, solve_simplex4, write_cache,
};
use super::types::{DistanceInput, DistanceOutput, ShapeProxy, Simplex, SimplexCache};
use crate::math_functions::{
    add, blend2, blend3, cross, distance, dot, is_normalized, length_squared,
    make_matrix_from_quat, max_float, mul_mv, mul_sv, neg, normalize, sub, transpose, Vec3,
    VEC3_ZERO,
};

const MAX_SIMPLEX_VERTICES: i32 = 4;
const MAX_GJK_ITERATIONS: i32 = 32;

/// Support index for a shape proxy along an axis.
///
/// We move the first vertex into the origin for improved precision.
/// This is necessary since we don't have shape transforms and
/// vertices can potentially be far away from the origin (large).
/// (b3GetProxySupport)
pub fn get_proxy_support(proxy: &ShapeProxy, axis: Vec3) -> i32 {
    let count = proxy.count;
    debug_assert!(count > 0);

    // We move the first vertex into the origin for improved precision.
    let origin = proxy.points[0];
    let mut max_index = 0;
    let mut max_projection = 0.0;

    for index in 1..count {
        // We subtract the first vertex since we are shifting into the origin.
        let projection = dot(axis, sub(proxy.points[index as usize], origin));
        if projection > max_projection {
            max_index = index;
            max_projection = projection;
        }
    }

    max_index
}

/// Support index for a point cloud along an axis. (b3GetPointSupport)
pub fn get_point_support(points: &[Vec3], axis: Vec3) -> i32 {
    let count = points.len() as i32;
    debug_assert!(count > 0);

    // We move the first vertex into the origin for improved precision.
    let origin = points[0];
    let mut max_index = 0;
    let mut max_projection = 0.0;

    for index in 1..count {
        // We subtract the first vertex since we are shifting into the origin.
        let projection = dot(axis, sub(points[index as usize], origin));
        if projection > max_projection {
            max_index = index;
            max_projection = projection;
        }
    }

    max_index
}

/// Compute the closest points between two shapes represented as point clouds.
/// `cache` is input/output. On the first call set `SimplexCache::count` to zero.
/// The query runs in frame A, so the witness points and normal are returned in
/// frame A. The underlying GJK algorithm may be debugged by passing in debug
/// simplexes; pass `None` normally. (b3ShapeDistance)
pub fn shape_distance(
    input: &DistanceInput,
    cache: &mut SimplexCache,
    mut simplexes: Option<&mut [Simplex]>,
) -> DistanceOutput {
    // The query runs in frame A using the relative pose of B in A.
    let xf = input.transform;

    // Use matrices for faster math
    let m = make_matrix_from_quat(xf.q);
    let mt = transpose(m);

    let proxy_a = &input.proxy_a;
    let proxy_b = &input.proxy_b;

    // Compute initial simplex from cache
    debug_assert!(cache.count as i32 <= MAX_SIMPLEX_VERTICES);

    let mut simplex = Simplex::default();

    simplex.count = cache.count as i32;
    for i in 0..cache.count as usize {
        let index1 = cache.index_a[i] as i32;
        let index2 = cache.index_b[i] as i32;

        debug_assert!(0 <= index1 && index1 < proxy_a.count);
        debug_assert!(0 <= index2 && index2 < proxy_b.count);

        let vertex1 = proxy_a.points[index1 as usize];
        let vertex2 = add(mul_mv(m, proxy_b.points[index2 as usize]), xf.p);

        simplex.vertices[i].index_a = index1;
        simplex.vertices[i].index_b = index2;
        simplex.vertices[i].w_a = vertex1;
        simplex.vertices[i].w_b = vertex2;
        simplex.vertices[i].w = sub(vertex2, vertex1);
        simplex.vertices[i].a = 0.0;
    }

    // Compute the new simplex metric, if it is substantially
    // different than the old metric flush the simplex.
    if simplex.count > 0 {
        let metric1 = cache.metric;
        let metric2 = get_metric(&simplex);

        // todo the tetrahedron metric can be negative
        if 2.0 * metric1 < metric2 || metric2 < 0.5 * metric1 || metric2 < f32::EPSILON {
            // Flush the simplex
            simplex.count = 0;
        }
    }

    // If the cache is invalid or empty
    if simplex.count == 0 {
        let vertex1 = proxy_a.points[0];
        let vertex2 = add(mul_mv(m, proxy_b.points[0]), xf.p);

        simplex.count = 1;
        simplex.vertices[0].index_a = 0;
        simplex.vertices[0].index_b = 0;
        simplex.vertices[0].w_a = vertex1;
        simplex.vertices[0].w_b = vertex2;
        simplex.vertices[0].w = sub(vertex2, vertex1);
        simplex.vertices[0].a = 0.0;
    }

    let mut backup = Simplex::default();

    let mut simplex_index = 0usize;
    if let Some(buffer) = simplexes.as_mut() {
        if simplex_index < buffer.len() {
            buffer[simplex_index] = simplex;
            simplex_index += 1;
        }
    }

    let mut distance_output = DistanceOutput::default();

    // Keep track of squared distance
    let mut distance_sq = f32::MAX;

    let mut normal = VEC3_ZERO;

    // Run GJK
    let mut iteration = 0i32;
    while iteration < MAX_GJK_ITERATIONS {
        // Solve simplex
        let solved = match simplex.count {
            1 => {
                simplex.vertices[0].a = 1.0;
                true
            }
            2 => solve_simplex2(&mut simplex),
            3 => solve_simplex3(&mut simplex),
            4 => solve_simplex4(&mut simplex),
            _ => {
                debug_assert!(false, "Should never get here!");
                false
            }
        };

        if !solved {
            // No progress - reconstruct last simplex
            debug_assert!(backup.count != 0);
            simplex = backup;
            break;
        }

        if let Some(buffer) = simplexes.as_mut() {
            if simplex_index < buffer.len() {
                buffer[simplex_index] = simplex;
                simplex_index += 1;
                distance_output.iterations = iteration;
                distance_output.simplex_count = simplex_index as i32;
            }
        }

        if simplex.count == MAX_SIMPLEX_VERTICES {
            // Overlap
            let (local_point_a, local_point_b) = compute_witness_points(&simplex);
            distance_output.point_a = local_point_a;
            distance_output.point_b = local_point_b;
            return distance_output;
        }

        // Assure distance progression
        let old_distance_sq = distance_sq;

        // Compute closest point
        let closest_point = match simplex.count {
            1 => simplex.vertices[0].w,
            2 => blend2(
                simplex.vertices[0].a,
                simplex.vertices[0].w,
                simplex.vertices[1].a,
                simplex.vertices[1].w,
            ),
            3 => blend3(
                simplex.vertices[0].a,
                simplex.vertices[0].w,
                simplex.vertices[1].a,
                simplex.vertices[1].w,
                simplex.vertices[2].a,
                simplex.vertices[2].w,
            ),
            4 => add(
                blend2(
                    simplex.vertices[0].a,
                    simplex.vertices[0].w,
                    simplex.vertices[1].a,
                    simplex.vertices[1].w,
                ),
                blend2(
                    simplex.vertices[2].a,
                    simplex.vertices[2].w,
                    simplex.vertices[3].a,
                    simplex.vertices[3].w,
                ),
            ),
            _ => {
                debug_assert!(false, "Should never get here!");
                VEC3_ZERO
            }
        };

        distance_sq = dot(closest_point, closest_point);

        if distance_sq >= old_distance_sq {
            // No progress - reconstruct last simplex
            debug_assert!(backup.count != 0);
            simplex = backup;
            break;
        }

        // Build new tentative support point
        let search_direction = match simplex.count {
            1 => {
                // v = -A
                neg(simplex.vertices[0].w)
            }
            2 => {
                // v = (AB x AO) x AB
                let a = simplex.vertices[0].w;
                let b = simplex.vertices[1].w;
                let ab = sub(b, a);
                cross(cross(ab, neg(a)), ab)
            }
            3 => {
                // v = AB x AC or v = AC x AB
                let a = simplex.vertices[0].w;
                let b = simplex.vertices[1].w;
                let c = simplex.vertices[2].w;
                let ab = sub(b, a);
                let ac = sub(c, a);
                let n = cross(ab, ac);
                if dot(n, a) < 0.0 {
                    n
                } else {
                    neg(n)
                }
            }
            _ => {
                debug_assert!(false, "Should never get here!");
                VEC3_ZERO
            }
        };

        if length_squared(search_direction) < 1000.0 * f32::MIN_POSITIVE {
            // The origin is probably contained by a line segment or triangle.
            // Thus the shapes are overlapped.
            let (local_point_a, local_point_b) = compute_witness_points(&simplex);
            distance_output.point_a = local_point_a;
            distance_output.point_b = local_point_b;
            debug_assert!(distance(local_point_a, local_point_b) < f32::EPSILON);
            return distance_output;
        }

        normal = neg(search_direction);

        // Get new support points
        let search_direction1 = search_direction;
        let index_a = get_proxy_support(&input.proxy_a, neg(search_direction1));
        let support_a = input.proxy_a.points[index_a as usize];
        let search_direction2 = mul_mv(mt, search_direction);
        let index_b = get_proxy_support(&input.proxy_b, search_direction2);
        let support_b = add(mul_mv(m, input.proxy_b.points[index_b as usize]), xf.p);

        // Save current simplex and add new vertex - this can fail if we detect cycling
        backup = simplex;

        // Check for duplicate support points. This is the main termination criteria.
        let mut duplicate = false;
        for i in 0..simplex.count as usize {
            if simplex.vertices[i].index_a == index_a && simplex.vertices[i].index_b == index_b {
                duplicate = true;
                break;
            }
        }

        if duplicate {
            break;
        }

        let count = simplex.count as usize;
        simplex.vertices[count].index_a = index_a;
        simplex.vertices[count].index_b = index_b;
        simplex.vertices[count].w_a = support_a;
        simplex.vertices[count].w_b = support_b;
        simplex.vertices[count].w = sub(support_b, support_a);
        simplex.count += 1;

        iteration += 1;
    }

    normal = normalize(normal);
    if !is_normalized(normal) {
        // Treat as overlap
        return distance_output;
    }

    // Build witness points and safe cache
    let (local_point_a, local_point_b) = compute_witness_points(&simplex);
    write_cache(cache, &simplex);

    // Results stay in frame A
    distance_output.point_a = local_point_a;
    distance_output.point_b = local_point_b;
    distance_output.distance = distance(local_point_a, local_point_b);
    distance_output.normal = normal;
    distance_output.iterations = iteration;
    distance_output.simplex_count = simplex_index as i32;

    // Apply radii if requested
    if input.use_radii {
        let r_a = input.proxy_a.radius;
        let r_b = input.proxy_b.radius;
        distance_output.distance = max_float(0.0, distance_output.distance - r_a - r_b);

        // Keep closest points on perimeter even if overlapped, this way the points move smoothly.
        distance_output.point_a = add(distance_output.point_a, mul_sv(r_a, normal));
        distance_output.point_b = sub(distance_output.point_b, mul_sv(r_b, normal));
    }

    distance_output
}
