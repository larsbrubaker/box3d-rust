//! Separated-pose oracle for mesh ghost handling.
//!
//! Test-support port of `box3d-cpp-reference/shared/ghost_oracle.h` / `ghost_oracle.c`, used
//! by the `MeshGhostOracle` port in `mesh_tests_ghost.rs`. With the hull clear of the mesh
//! the union of the triangles is the reference surface, and none of the Voronoi logic under
//! test is needed to judge a contact against it.
//!
//! A normal is valid at a mesh location when the union has it there: the face normal of a
//! triangle containing the location, or a proximal normal, meaning the location is the
//! closest union point to a point a little way out along the normal. That second test gives
//! the convex edge wedge and the vertex cone exactly, and nothing at a concave crease beyond
//! the two face normals. A manifold shares one normal across its points, so it is a ghost when
//! the normal is valid at none of its points within linear slop of its deepest.
//!
//! The contacts the union itself has are the local minima of the distance between the hull
//! and the union. A triangle's closest pair (GJK) is one unless a triangle that also contains
//! its mesh point is closer to the hull. The distance to a triangle is convex over the
//! triangle, so that test is exact. The union closest pair must be represented by a collided
//! point, otherwise the contact was missed.
//!
//! The C pose records (`OraclePose`, `OracleResult`) are trimmed to what the scan reads. The
//! scene meshes are built in `scenes.rs`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod scenes;

pub(crate) use scenes::create_oracle_mesh;

use crate::constants::{linear_slop, mesh_rest_offset, speculative_distance};
use crate::contact::collide_mesh_and_hull;
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::hull::{compute_hull_aabb, get_hull_points, make_box_hull, BoxHull, HullData};
use crate::manifold::Manifold;
use crate::math_functions::{
    abs_float, add, compute_cos_sin, compute_quat_between_unit_vectors, cross, distance_squared,
    dot, inv_rotate_vector, length, length_squared, make_quat_from_axis_angle, min_float, mul_add,
    mul_quat, mul_sv, neg, normalize, rotate_vector, sub, Quat, Transform, Vec3, PI, VEC3_AXIS_Y,
    VEC3_ZERO,
};
use crate::mesh::{query_mesh, Mesh, MeshData};

/// (OracleScene)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum OracleScene {
    Flat,
    Fan,
    Crest,
    Valley,
    Ridge,
    SaddleRidge,
    Rail,
    Wave,
}

/// (e_oracleSceneCount)
pub(crate) const ORACLE_SCENE_COUNT: usize = 8;

/// Every scene in order.
pub(crate) const ORACLE_SCENES: [OracleScene; ORACLE_SCENE_COUNT] = [
    OracleScene::Flat,
    OracleScene::Fan,
    OracleScene::Crest,
    OracleScene::Valley,
    OracleScene::Ridge,
    OracleScene::SaddleRidge,
    OracleScene::Rail,
    OracleScene::Wave,
];

/// (g_oracleSceneNames)
pub(crate) const ORACLE_SCENE_NAMES: [&str; ORACLE_SCENE_COUNT] = [
    "flat",
    "fan",
    "crest",
    "valley",
    "ridge",
    "saddle ridge",
    "rail",
    "wave",
];

/// (ORACLE_MAX_TRIANGLES)
const ORACLE_MAX_TRIANGLES: usize = 256;

/// (ORACLE_MAX_POINTS)
const ORACLE_MAX_POINTS: usize = 128;

/// Poses per scene in the unit test, and in the sample scan so the two report the same counts.
/// (ORACLE_SCAN_POSES)
pub(crate) const ORACLE_SCAN_POSES: i32 = 500;

/// (ORACLE_MAX_MANIFOLDS)
const ORACLE_MAX_MANIFOLDS: usize = 128;

/// (ORACLE_CONTAIN_TOLERANCE)
const ORACLE_CONTAIN_TOLERANCE: f32 = 1.0e-4;

/// The poses keep the hull separated by at least this much, where the closest direction is
/// well defined. (ORACLE_MIN_DISTANCE)
fn oracle_min_distance() -> f32 {
    0.1 * linear_slop()
}

/// (OracleVerdict)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OracleVerdict {
    /// Face normal of a triangle containing the point
    Face,

    /// Proximal normal at an edge or vertex
    Feature,

    /// Not valid here, but valid at a deep point of the same manifold
    Shared,

    /// Point of a manifold whose normal is valid at none of its deep points
    Ghost,
}

/// (OraclePair)
#[derive(Debug, Clone, Copy)]
struct OraclePair {
    triangle_index: i32,
    vertices: [Vec3; 3],
    face_normal: Vec3,
    mesh_point: Vec3,
    distance: f32,
}

/// The union of the triangles near the hull. Depends only on the pose, not on the collider.
/// (OracleReference)
struct OracleReference {
    /// Distance between the hull and the mesh
    distance: f32,
    closest_mesh_point: Vec3,
    closest_hull_point: Vec3,

    pairs: Vec<OraclePair>,
}

/// (OraclePoint)
#[derive(Debug, Clone, Copy)]
struct OraclePoint {
    point: Vec3,
    normal: Vec3,

    /// Geometric separation, without the mesh rest offset the collider subtracts
    separation: f32,
    triangle_index: i32,
    verdict: OracleVerdict,
}

/// The collider output judged against the reference. (OracleResult)
struct OracleResult {
    /// Ghost manifolds
    ghost_count: i32,

    /// The union closest pair has no collided point
    missed: bool,
}

/// (OracleScan)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct OracleScan {
    /// Separated poses
    pub valid_count: i32,

    /// Poses with a ghost manifold, without and with ghost handling
    pub raw_ghost_poses: i32,
    pub ghost_poses: i32,

    /// Poses where ghost handling lost the closest pair
    pub missed_poses: i32,

    /// Pose indices, -1 for none
    pub first_ghost: i32,
    pub first_missed: i32,
}

/// Local state and 24 bits per draw, which the global 15 bit generator in utils.h does not
/// give. (OracleRandom)
struct OracleRandom {
    state: u32,
}

impl OracleRandom {
    /// (RandomUnit)
    fn unit(&mut self) -> f32 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        (x >> 8) as f32 / 16777216.0
    }

    /// (RandomRange)
    fn range(&mut self, lower: f32, upper: f32) -> f32 {
        lower + (upper - lower) * self.unit()
    }

    /// (RandomDirection)
    fn direction(&mut self) -> Vec3 {
        for _ in 0..16 {
            let x = self.range(-1.0, 1.0);
            let y = self.range(-1.0, 1.0);
            let z = self.range(-1.0, 1.0);
            let v = Vec3 { x, y, z };
            let length_squared = length_squared(v);
            if 0.01 < length_squared && length_squared <= 1.0 {
                return mul_sv(1.0 / length_squared.sqrt(), v);
            }
        }

        VEC3_AXIS_Y
    }

    /// Direction within the given angle of the axis. (RandomTilt)
    fn tilt(&mut self, axis: Vec3, max_angle: f32) -> Vec3 {
        let mut perp = cross(axis, self.direction());
        let length = length(perp);
        if length < 1.0e-4 {
            return axis;
        }

        perp = mul_sv(1.0 / length, perp);
        let cs = compute_cos_sin(self.range(0.0, max_angle));
        add(mul_sv(cs.cosine, axis), mul_sv(cs.sine, perp))
    }
}

/// From Real-time Collision Detection, p141. (ClosestPointOnTriangle)
fn closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = sub(p, b);
    let d3 = dot(ab, bp);
    let d4 = dot(ac, bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return mul_add(a, d1 / (d1 - d3), ab);
    }

    let cp = sub(p, c);
    let d5 = dot(ab, cp);
    let d6 = dot(ac, cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return mul_add(a, d2 / (d2 - d6), ac);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return mul_add(b, w, sub(c, b));
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    add(a, add(mul_sv(v, ab), mul_sv(w, ac)))
}

/// The closest pair of each nearby triangle and the union distance. (ComputePairs)
fn compute_pairs(
    reference: &mut OracleReference,
    mesh: &MeshData,
    hull: &HullData,
    transform: Transform,
) -> f32 {
    reference.pairs.clear();

    let mut bounds = compute_hull_aabb(hull, transform);
    // Collided points can be well past the speculative distance and the proximal test looks
    // around them
    let margin = 0.25;
    let extension = Vec3 {
        x: margin,
        y: margin,
        z: margin,
    };
    bounds.lower_bound = sub(bounds.lower_bound, extension);
    bounds.upper_bound = add(bounds.upper_bound, extension);

    // (CollectPair)
    let mesh_shape = Mesh::with_unit_scale(mesh);
    let pairs = &mut reference.pairs;
    query_mesh(&mesh_shape, bounds, |a, b, c, triangle_index| {
        // A truncated union would misjudge the pose
        debug_assert!(pairs.len() < ORACLE_MAX_TRIANGLES);
        if pairs.len() == ORACLE_MAX_TRIANGLES {
            return false;
        }

        pairs.push(OraclePair {
            triangle_index,
            vertices: [a, b, c],
            face_normal: normalize(cross(sub(b, a), sub(c, a))),
            mesh_point: VEC3_ZERO,
            distance: 0.0,
        });
        true
    });

    let mut min_distance = f32::MAX;
    for pair in reference.pairs.iter_mut() {
        let input = DistanceInput {
            proxy_a: make_proxy(&pair.vertices, 0.0),
            proxy_b: make_proxy(get_hull_points(hull), 0.0),
            transform,
            use_radii: false,
        };
        let mut cache = SimplexCache::default();
        let output = shape_distance(&input, &mut cache, None);
        pair.mesh_point = output.point_a;
        pair.distance = output.distance;

        if output.distance < min_distance {
            min_distance = output.distance;
            reference.closest_mesh_point = output.point_a;
            reference.closest_hull_point = output.point_b;
        }
    }

    reference.distance = min_distance;
    min_distance
}

/// (TriangleContains)
fn triangle_contains(pair: &OraclePair, point: Vec3) -> bool {
    let closest =
        closest_point_on_triangle(point, pair.vertices[0], pair.vertices[1], pair.vertices[2]);
    distance_squared(closest, point) < ORACLE_CONTAIN_TOLERANCE * ORACLE_CONTAIN_TOLERANCE
}

/// On success the reference holds the closest pairs of the returned pose. (MakeOraclePose)
fn make_oracle_pose(
    reference: &mut OracleReference,
    mesh: &MeshData,
    scene: OracleScene,
    index: i32,
) -> Option<(BoxHull, Transform)> {
    let mut random = OracleRandom {
        state: 0x9E37_79B9u32
            ^ 2654435761u32.wrapping_mul((index + 1) as u32)
            ^ 40503u32.wrapping_mul(scene as u32),
    };
    for _ in 0..4 {
        random.unit();
    }

    let vertices = &mesh.vertices;
    let triangles = &mesh.triangles;
    let mut triangle_index = (random.unit() * mesh.triangle_count as f32) as i32;
    triangle_index = if triangle_index < mesh.triangle_count {
        triangle_index
    } else {
        mesh.triangle_count - 1
    };
    let triangle = triangles[triangle_index as usize];
    let a = vertices[triangle.index1 as usize];
    let b = vertices[triangle.index2 as usize];
    let c = vertices[triangle.index3 as usize];
    let face_normal = normalize(cross(sub(b, a), sub(c, a)));

    let mut u = random.unit();
    let mut v = random.unit();
    if u + v > 1.0 {
        u = 1.0 - u;
        v = 1.0 - v;
    }

    let target = add(a, add(mul_sv(u, sub(b, a)), mul_sv(v, sub(c, a))));

    let hx = random.range(0.05, 0.4);
    let hy = random.range(0.05, 0.4);
    let hz = random.range(0.05, 0.4);
    let box_hull = make_box_hull(hx, hy, hz);

    // Half the boxes rest nearly flat on the face, where parallel faces make ties
    let rotation: Quat = if random.unit() < 0.5 {
        let up = random.tilt(face_normal, 0.2);
        let align = compute_quat_between_unit_vectors(VEC3_AXIS_Y, up);
        let yaw = make_quat_from_axis_angle(VEC3_AXIS_Y, random.range(-PI, PI));
        mul_quat(align, yaw)
    } else {
        let axis = random.direction();
        make_quat_from_axis_angle(axis, random.range(-PI, PI))
    };

    let approach = random.tilt(face_normal, 1.0);

    // Put the hull support point against the target
    let hull_points = get_hull_points(&box_hull.base);
    let local_direction = inv_rotate_vector(rotation, neg(approach));
    let mut support = hull_points[0];
    for &point in hull_points
        .iter()
        .take(box_hull.base.vertex_count as usize)
        .skip(1)
    {
        if dot(point, local_direction) > dot(support, local_direction) {
            support = point;
        }
    }

    let linear_slop = linear_slop();
    let gap = linear_slop * (0.2 + 3.4 * random.unit() * random.unit());
    let mut pose = Transform {
        p: sub(target, rotate_vector(rotation, support)),
        q: rotation,
    };
    pose.p = mul_add(pose.p, gap, approach);

    for _ in 0..20 {
        let distance = compute_pairs(reference, mesh, &box_hull.base, pose);
        if reference.pairs.is_empty() {
            return None;
        }

        if distance <= 0.0 {
            pose.p = mul_add(pose.p, 2.0 * linear_slop, approach);
            continue;
        }

        if abs_float(distance - gap) < 0.05 * linear_slop {
            return Some((box_hull, pose));
        }

        let direction = normalize(sub(
            reference.closest_hull_point,
            reference.closest_mesh_point,
        ));
        pose.p = mul_add(pose.p, gap - distance, direction);
    }

    let distance = compute_pairs(reference, mesh, &box_hull.base, pose);
    if !reference.pairs.is_empty()
        && oracle_min_distance() <= distance
        && distance <= speculative_distance()
    {
        Some((box_hull, pose))
    } else {
        None
    }
}

/// Smallest squared sine between the normal and the face normal of a triangle that contains
/// the mesh point and faces the normal. f32::MAX if there is no such triangle.
/// (FacingSineSquared)
fn facing_sine_squared(reference: &OracleReference, mesh_point: Vec3, normal: Vec3) -> f32 {
    let mut best = f32::MAX;
    for pair in &reference.pairs {
        if dot(pair.face_normal, normal) <= 0.0 || !triangle_contains(pair, mesh_point) {
            continue;
        }

        best = min_float(best, length_squared(cross(pair.face_normal, normal)));
    }

    best
}

/// The mesh point is the closest union point to a point a little way out along the normal.
/// (IsProximalNormal)
fn is_proximal_normal(
    reference: &OracleReference,
    mesh_point: Vec3,
    normal: Vec3,
    sin_tolerance: f32,
) -> bool {
    let probe_distance = 0.01;
    let probe = mul_add(mesh_point, probe_distance, normal);
    let mut best_distance_squared = f32::MAX;
    let mut best = mesh_point;
    for pair in &reference.pairs {
        let closest =
            closest_point_on_triangle(probe, pair.vertices[0], pair.vertices[1], pair.vertices[2]);
        let distance_squared = distance_squared(closest, probe);
        if distance_squared < best_distance_squared {
            best_distance_squared = distance_squared;
            best = closest;
        }
    }

    let offset = sin_tolerance * probe_distance;
    distance_squared(best, mesh_point) < offset * offset
}

/// Collides the hull and judges every manifold against the reference. (JudgeCollision)
fn judge_collision(
    reference: &OracleReference,
    mesh: &MeshData,
    hull: &HullData,
    transform: Transform,
    handle_ghosts: bool,
) -> OracleResult {
    let mut result = OracleResult {
        ghost_count: 0,
        missed: false,
    };
    let mut points: Vec<OraclePoint> = Vec::with_capacity(ORACLE_MAX_POINTS);

    let mut manifolds = vec![Manifold::default(); ORACLE_MAX_MANIFOLDS];
    let mesh_shape = Mesh::with_unit_scale(mesh);
    let manifold_count = collide_mesh_and_hull(
        &mut manifolds,
        &mesh_shape,
        hull,
        transform,
        true,
        handle_ghosts,
    );
    debug_assert!((manifold_count as usize) < ORACLE_MAX_MANIFOLDS);

    // Clustering merges manifolds whose normals are within about 5 degrees
    let sin_tolerance = 0.1;
    let mut min_separation = f32::MAX;
    for manifold in &manifolds[..manifold_count as usize] {
        let first_point = points.len();
        for manifold_point in &manifold.points[..manifold.point_count as usize] {
            debug_assert!(points.len() < ORACLE_MAX_POINTS);
            if points.len() == ORACLE_MAX_POINTS {
                break;
            }

            let mut point = OraclePoint {
                point: manifold_point.anchor_a,
                normal: manifold.normal,
                separation: manifold_point.separation + mesh_rest_offset(),
                triangle_index: manifold_point.triangle_index,
                verdict: OracleVerdict::Ghost,
            };

            min_separation = min_float(min_separation, point.separation);

            // The reference bounds contain the collider bounds, so every collided triangle has
            // a pair
            let pair = reference
                .pairs
                .iter()
                .find(|pair| pair.triangle_index == point.triangle_index);
            debug_assert!(pair.is_some());
            if let Some(pair) = pair {
                let mesh_point = closest_point_on_triangle(
                    point.point,
                    pair.vertices[0],
                    pair.vertices[1],
                    pair.vertices[2],
                );

                // A proximal normal also has to be in front of a face here, or an inverted face
                // normal would pass
                let sine_squared = facing_sine_squared(reference, mesh_point, point.normal);
                if sine_squared < sin_tolerance * sin_tolerance {
                    point.verdict = OracleVerdict::Face;
                } else if sine_squared < f32::MAX
                    && is_proximal_normal(reference, mesh_point, point.normal, sin_tolerance)
                {
                    point.verdict = OracleVerdict::Feature;
                }
            }

            points.push(point);
        }

        let mut deepest = f32::MAX;
        for point in &points[first_point..] {
            deepest = min_float(deepest, point.separation);
        }

        let mut valid = false;
        for point in &points[first_point..] {
            if point.separation <= deepest + linear_slop() && point.verdict != OracleVerdict::Ghost
            {
                valid = true;
                break;
            }
        }

        if valid {
            for point in &mut points[first_point..] {
                if point.verdict == OracleVerdict::Ghost {
                    point.verdict = OracleVerdict::Shared;
                }
            }
        }

        if !valid && points.len() > first_point {
            result.ghost_count += 1;
        }
    }

    // The union closest pair has to be represented. SAT separation is a lower bound on
    // distance.
    let linear_slop = linear_slop();
    let distance = reference.distance;
    if distance < speculative_distance() - linear_slop && min_separation > distance + linear_slop {
        result.missed = true;
    }

    result
}

/// Poses [0, pose_count) of the scene. (ScanGhostOracle)
pub(crate) fn scan_ghost_oracle(
    mesh: &MeshData,
    scene: OracleScene,
    pose_count: i32,
) -> OracleScan {
    let mut scan = OracleScan {
        first_ghost: -1,
        first_missed: -1,
        ..Default::default()
    };

    let mut reference = OracleReference {
        distance: 0.0,
        closest_mesh_point: VEC3_ZERO,
        closest_hull_point: VEC3_ZERO,
        pairs: Vec::with_capacity(ORACLE_MAX_TRIANGLES),
    };

    for i in 0..pose_count {
        // (EvaluateOraclePose)
        let Some((box_hull, transform)) = make_oracle_pose(&mut reference, mesh, scene, i) else {
            continue;
        };

        debug_assert!(reference.distance >= oracle_min_distance());

        // C also computes the pair ownership here (ComputeOwnership) for the sample's display;
        // the scan does not read it.
        let raw = judge_collision(&reference, mesh, &box_hull.base, transform, false);
        let handled = judge_collision(&reference, mesh, &box_hull.base, transform, true);

        scan.valid_count += 1;
        scan.raw_ghost_poses += if raw.ghost_count > 0 { 1 } else { 0 };

        if handled.ghost_count > 0 {
            scan.ghost_poses += 1;
            scan.first_ghost = if scan.first_ghost < 0 {
                i
            } else {
                scan.first_ghost
            };
        }

        // A miss counts against ghost handling only when the collider represents the closest
        // pair without it (IsOracleCullingMiss)
        if handled.missed && !raw.missed {
            scan.missed_poses += 1;
            scan.first_missed = if scan.first_missed < 0 {
                i
            } else {
                scan.first_missed
            };
        }
    }

    scan
}
