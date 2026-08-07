//! Baked box hull tests from box3d-cpp-reference/test/test_hull.c.
//!
//! Split out of `hull_tests` to keep each file under the 800-line module gate.

use crate::hull::make_transformed_box_hull;
use crate::math_functions::{
    aabb_contains, abs_float, dot, make_aabb, neg, rotate_vector, transform_point, Quat, Transform,
    Vec3, PI, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ZERO,
};

fn ensure_small(v: f32, tol: f32) {
    assert!(abs_float(v) <= tol, "expected |{v}| <= {tol}");
}

// b3ComputeCosSin is a coarse approximation, so build rotations from libm to keep the
// analytic corner and plane positions exact.
fn exact_quat(axis: Vec3, radians: f32) -> Quat {
    let half = 0.5 * radians;
    let s = half.sin();
    Quat {
        v: Vec3 {
            x: s * axis.x,
            y: s * axis.y,
            z: s * axis.z,
        },
        s: half.cos(),
    }
}

// Corner sign pattern baked by b3MakeTransformedBoxHull, in order.
const BOX_CORNER_SIGNS: [Vec3; 8] = [
    Vec3 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    },
    Vec3 {
        x: -1.0,
        y: 1.0,
        z: 1.0,
    },
    Vec3 {
        x: -1.0,
        y: -1.0,
        z: 1.0,
    },
    Vec3 {
        x: 1.0,
        y: -1.0,
        z: 1.0,
    },
    Vec3 {
        x: 1.0,
        y: 1.0,
        z: -1.0,
    },
    Vec3 {
        x: -1.0,
        y: 1.0,
        z: -1.0,
    },
    Vec3 {
        x: -1.0,
        y: -1.0,
        z: -1.0,
    },
    Vec3 {
        x: 1.0,
        y: -1.0,
        z: -1.0,
    },
];

// The baked box hull is only exercised elsewhere through mass properties, which are analytic
// and never read boxPoints or boxPlanes. Pin the geometry itself against the transform so an
// axis swap in the point or plane bake cannot pass silently.
fn check_transformed_box(h: Vec3, xf: Transform) {
    let box_hull = make_transformed_box_hull(h.x, h.y, h.z, xf);

    const TOL: f32 = 1e-5;

    // Each corner is the transform of the signed half extent.
    for i in 0..8 {
        let local = Vec3 {
            x: BOX_CORNER_SIGNS[i].x * h.x,
            y: BOX_CORNER_SIGNS[i].y * h.y,
            z: BOX_CORNER_SIGNS[i].z * h.z,
        };
        let expected = transform_point(xf, local);
        ensure_small(box_hull.box_points[i].x - expected.x, TOL);
        ensure_small(box_hull.box_points[i].y - expected.y, TOL);
        ensure_small(box_hull.box_points[i].z - expected.z, TOL);
    }

    // Face normals rotate with the box, offsets carry the rotated normal through the translation.
    let local_normals = [
        neg(VEC3_AXIS_X),
        VEC3_AXIS_X,
        neg(VEC3_AXIS_Y),
        VEC3_AXIS_Y,
        neg(VEC3_AXIS_Z),
        VEC3_AXIS_Z,
    ];
    let local_offsets = [h.x, h.x, h.y, h.y, h.z, h.z];
    for i in 0..6 {
        let n = rotate_vector(xf.q, local_normals[i]);
        let offset = local_offsets[i] + dot(n, xf.p);
        ensure_small(box_hull.box_planes[i].normal.x - n.x, TOL);
        ensure_small(box_hull.box_planes[i].normal.y - n.y, TOL);
        ensure_small(box_hull.box_planes[i].normal.z - n.z, TOL);
        ensure_small(box_hull.box_planes[i].offset - offset, TOL);
    }

    // SoA vertex mirrors match the AoS points on all eight lanes.
    for i in 0..8 {
        ensure_small(box_hull.vx[i] - box_hull.box_points[i].x, TOL);
        ensure_small(box_hull.vy[i] - box_hull.box_points[i].y, TOL);
        ensure_small(box_hull.vz[i] - box_hull.box_points[i].z, TOL);
    }

    // SoA normal mirrors carry the six real faces and zero the two pad lanes.
    for i in 0..6 {
        ensure_small(box_hull.nx[i] - box_hull.box_planes[i].normal.x, TOL);
        ensure_small(box_hull.ny[i] - box_hull.box_planes[i].normal.y, TOL);
        ensure_small(box_hull.nz[i] - box_hull.box_planes[i].normal.z, TOL);
    }
    assert!(box_hull.nx[6] == 0.0 && box_hull.nx[7] == 0.0);
    assert!(box_hull.ny[6] == 0.0 && box_hull.ny[7] == 0.0);
    assert!(box_hull.nz[6] == 0.0 && box_hull.nz[7] == 0.0);

    // The stored AABB bounds every baked corner.
    let point_box = make_aabb(&box_hull.box_points, 0.0);
    assert!(aabb_contains(box_hull.base.aabb, point_box));
}

#[test]
fn transformed_box_hull_test() {
    let h = Vec3 {
        x: 0.25,
        y: 0.5,
        z: 0.3,
    };

    // Identity.
    check_transformed_box(h, TRANSFORM_IDENTITY);

    // Translation only.
    let translated = Transform {
        p: Vec3 {
            x: 0.4,
            y: -0.7,
            z: 0.1,
        },
        q: QUAT_IDENTITY,
    };
    check_transformed_box(h, translated);

    // Rotation only. A quarter turn about Y swaps the world X and Z extents.
    let rotated = Transform {
        p: VEC3_ZERO,
        q: exact_quat(VEC3_AXIS_Y, 0.25 * PI),
    };
    check_transformed_box(h, rotated);

    // Translation and rotation together.
    let transformed = Transform {
        p: Vec3 {
            x: 3.0,
            y: -2.0,
            z: 1.5,
        },
        q: exact_quat(VEC3_AXIS_Z, 0.25 * PI),
    };
    check_transformed_box(h, transformed);
}
