//! Geometry category demos (sample_geometry.cpp): five static hull / mass
//! viewers. Every hull, plane, mass and inertia value shown on the site is
//! computed by the ported `box3d_rust::hull` / `box3d_rust::geometry` code —
//! the wasm layer only walks the resulting half-edge mesh into triangle + line
//! buffers for the WebGL renderer.
//!
//! # Packed hull block
//!
//! Each scene returns a flat `Vec<f32>` built from one or more *hull blocks*.
//! A block is an 8-float header followed by its triangle then line floats:
//!
//! ```text
//!   [ surfaceArea, volume, innerRadius,
//!     vertexCount, faceCount, uniqueEdgeCount,   (integers as f32)
//!     triFloatLen, wireFloatLen,                 (integers as f32)
//!     ...tri floats  (9 per triangle: 3 verts × xyz),
//!     ...wire floats (6 per edge:    2 verts × xyz) ]
//! ```
//!
//! `triFloatLen == 0 && wireFloatLen == 0` marks an empty block (hull creation
//! failed / degenerate), so the TS side can skip drawing it. Triangles are a
//! fan per face from its first ring vertex; edges are each undirected half-edge
//! pair emitted once, matching the C `DrawHull` (draw.c:100) winding.

use wasm_bindgen::prelude::*;

use box3d_rust::geometry::{compute_capsule_mass, Capsule};
use box3d_rust::hull::{
    clone_and_transform_hull, compute_hull_mass, create_cylinder, create_hull, make_box_hull,
    make_scaled_box_hull, HullData,
};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, mul, safe_scale, transform_point, Quat, Transform, Vec3, DEG_TO_RAD,
    PI, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ZERO,
};

use crate::rng::XorShift32;

/// Header size of a packed hull block (see module docs).
const HULL_BLOCK_HEADER: usize = 8;

/// Append a hull's render + property data as one packed block. (see module docs)
fn pack_hull(out: &mut Vec<f32>, hull: &HullData) {
    // Solid faces (fan-triangulated) and twin-deduped wireframe edges, both in
    // hull-local space, from the shared `vis` helpers (see C DrawHull, draw.c:100).
    let tris = crate::vis::hull_triangles(hull);
    let wire = crate::vis::hull_edges(hull);

    out.push(hull.surface_area);
    out.push(hull.volume);
    out.push(hull.inner_radius);
    out.push(hull.vertex_count as f32);
    out.push(hull.face_count as f32);
    out.push((hull.edge_count / 2) as f32);
    out.push(tris.len() as f32);
    out.push(wire.len() as f32);
    out.extend_from_slice(&tris);
    out.extend_from_slice(&wire);
}

/// Append an empty hull block (creation failed / degenerate input).
fn pack_empty(out: &mut Vec<f32>) {
    out.extend_from_slice(&[0.0; HULL_BLOCK_HEADER]);
}

/// Compose `qz * (qy * qx)` from degrees using `b3MakeQuatFromAxisAngle` and
/// `B3_DEG_TO_RAD`, matching `BoxHull::UpdateRotation` (sample_geometry.cpp:39).
fn rotation_from_degrees_deg2rad(rx: f32, ry: f32, rz: f32) -> Quat {
    let qx = make_quat_from_axis_angle(VEC3_AXIS_X, DEG_TO_RAD * rx);
    let qy = make_quat_from_axis_angle(VEC3_AXIS_Y, DEG_TO_RAD * ry);
    let qz = make_quat_from_axis_angle(VEC3_AXIS_Z, DEG_TO_RAD * rz);
    box3d_rust::math_functions::mul_quat(qz, box3d_rust::math_functions::mul_quat(qy, qx))
}

/// Compose `qz * (qy * qx)` from degrees using the literal `deg * B3_PI / 180`,
/// matching `HullTransform::UpdateHull` (sample_geometry.cpp:396).
fn rotation_from_degrees_pi180(rx: f32, ry: f32, rz: f32) -> Quat {
    let qx = make_quat_from_axis_angle(VEC3_AXIS_X, rx * PI / 180.0);
    let qy = make_quat_from_axis_angle(VEC3_AXIS_Y, ry * PI / 180.0);
    let qz = make_quat_from_axis_angle(VEC3_AXIS_Z, rz * PI / 180.0);
    box3d_rust::math_functions::mul_quat(qz, box3d_rust::math_functions::mul_quat(qy, qx))
}

/// Box Hull (sample_geometry.cpp:12-136). Two hull blocks: a `b3CreateHull` of
/// eight transformed + post-scaled box corners (yellow) compared against
/// `b3MakeScaledBoxHull` (cyan). `h` half-widths, `c` center, `r` rotation
/// (degrees), `s` post-scale.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn geometry_box_hull(
    hx: f32,
    hy: f32,
    hz: f32,
    cx: f32,
    cy: f32,
    cz: f32,
    rx: f32,
    ry: f32,
    rz: f32,
    sx: f32,
    sy: f32,
    sz: f32,
) -> Vec<f32> {
    let h = Vec3 {
        x: hx,
        y: hy,
        z: hz,
    };
    let post_scale = Vec3 {
        x: sx,
        y: sy,
        z: sz,
    };
    let transform = Transform {
        p: Vec3 {
            x: cx,
            y: cy,
            z: cz,
        },
        q: rotation_from_degrees_deg2rad(rx, ry, rz),
    };

    let scale = safe_scale(post_scale);
    let corners = [
        Vec3 {
            x: h.x,
            y: h.y,
            z: h.z,
        },
        Vec3 {
            x: h.x,
            y: h.y,
            z: -h.z,
        },
        Vec3 {
            x: h.x,
            y: -h.y,
            z: h.z,
        },
        Vec3 {
            x: h.x,
            y: -h.y,
            z: -h.z,
        },
        Vec3 {
            x: -h.x,
            y: h.y,
            z: h.z,
        },
        Vec3 {
            x: -h.x,
            y: h.y,
            z: -h.z,
        },
        Vec3 {
            x: -h.x,
            y: -h.y,
            z: h.z,
        },
        Vec3 {
            x: -h.x,
            y: -h.y,
            z: -h.z,
        },
    ];
    let mut pts = [VEC3_ZERO; 8];
    for (i, corner) in corners.iter().enumerate() {
        pts[i] = mul(scale, transform_point(transform, *corner));
    }

    let mut out = vec![2.0];
    match create_hull(&pts, 8) {
        Some(hull) => pack_hull(&mut out, &hull),
        None => pack_empty(&mut out),
    }
    let box_hull = make_scaled_box_hull(h, transform, post_scale);
    pack_hull(&mut out, &box_hull.base);
    out
}

/// Hull (sample_geometry.cpp:138-231). A fixed 48-point cloud scaled ×0.01 and
/// reduced to at most 16 vertices via `b3CreateHull`. One yellow block (empty
/// if the reduction fails, as the C sample's `m_hull != nullptr` guard allows).
#[wasm_bindgen]
pub fn geometry_hull() -> Vec<f32> {
    // The exact cloud from the C sample (sample_geometry.cpp:152-177).
    const RAW: [[f32; 3]; 48] = [
        [-3.9866004, 75.4595108, 28.3783073],
        [-13.1079493, 73.080368, 28.296587],
        [-18.6611958, 72.0040894, 16.9292431],
        [4.82537603, 79.2908554, 22.2369995],
        [-12.7315464, 79.2187576, 2.94275379],
        [-21.806488, 78.7758865, 0.985544085],
        [-27.7619209, 73.3481522, 11.9647141],
        [-22.3994541, 72.2203826, 21.4116211],
        [-25.3797474, 76.7417755, 27.9124985],
        [-22.7552319, 77.0559006, 29.4733639],
        [-6.81736374, 78.3484726, 36.8649979],
        [3.62397718, 85.5270843, 29.2077713],
        [7.90363788, 84.121231, 18.2612896],
        [-12.3809223, 84.5280533, -0.43230924],
        [5.83599472, 95.2908325, 4.4423275],
        [-22.5541401, 89.9094467, -4.87791252],
        [-43.9060402, 78.5287094, 1.32877088],
        [-42.6015129, 76.7829742, 7.67437983],
        [-25.735527, 78.1218796, 27.908411],
        [-23.5183544, 77.6326675, 29.1178799],
        [2.0977366, 100.430191, 34.3929482],
        [1.09743047, 103.952553, 35.5656395],
        [8.50175952, 96.0529861, 8.73674774],
        [2.52570295, 103.303696, 32.2314339],
        [-20.099781, 89.4923248, -4.15468454],
        [2.8092947, 123.516098, -1.12693477],
        [-43.9318161, 79.1106186, 1.39006138],
        [-23.358511, 90.9599686, -4.25683546],
        [2.10804915, 123.603645, -1.38435471],
        [-44.1329117, 78.7192383, 1.54941654],
        [-42.4365158, 77.725357, 8.14835929],
        [-43.204792, 77.5811691, 7.14319515],
        [-44.17416, 78.7810363, 2.50146222],
        [-32.8975143, 99.1221771, 7.55588436],
        [-0.624746263, 110.070351, 32.7381058],
        [0.00431228895, 109.14341, 33.6411133],
        [-0.58865279, 122.980537, 16.6554794],
        [2.18539238, 124.324593, -0.620266676],
        [-1.02177501, 123.881721, 16.8230057],
        [1.9842999, 124.571777, -0.321986318],
        [1.86570692, 124.365791, -0.599836588],
        [-43.591507, 78.1373291, 6.1135149],
        [-43.8235397, 79.2239074, 3.48619604],
        [-43.591507, 78.50811, 5.54555655],
        [1.21086729, 124.49453, 1.07543683],
        [-1.86223853, 124.195847, 15.6257992],
        [-1.46520972, 124.355492, 16.9864483],
        [1.654302, 124.612976, 0.621887207],
    ];

    let points: Vec<Vec3> = RAW
        .iter()
        .map(|p| Vec3 {
            x: 0.01 * p[0],
            y: 0.01 * p[1],
            z: 0.01 * p[2],
        })
        .collect();

    let mut out = vec![1.0];
    match create_hull(&points, 16) {
        Some(hull) => pack_hull(&mut out, &hull),
        None => pack_empty(&mut out),
    }
    out
}

/// A `Vec3` box `[lo, hi]` draw, matching `RandomVec3` (shared/utils.h:65).
fn random_vec3(rng: &mut XorShift32, lo: Vec3, hi: Vec3) -> Vec3 {
    rng.vec3(lo, hi)
}

/// Shoemake uniform unit vector, matching `RandomUnitVector` (shared/utils.h:93).
/// Sample-local trig (libc `sinf`/`cosf` in C) — not the deterministic library
/// path, so ordinary `f32` sin/cos mirror it.
fn random_unit_vector(rng: &mut XorShift32) -> Vec3 {
    let u1 = rng.range(0.0, 1.0);
    let u2 = rng.range(0.0, 2.0 * PI);
    let u3 = rng.range(0.0, 2.0 * PI);
    let sqrt1_minus_u1 = (1.0 - u1).sqrt();
    let sqrt_u1 = u1.sqrt();
    Vec3 {
        x: sqrt1_minus_u1 * u2.sin(),
        y: sqrt1_minus_u1 * u2.cos(),
        z: sqrt_u1 * u3.sin(),
    }
}

/// Hull Reduction (sample_geometry.cpp:233-360). A 128-point box (`kind == 0`)
/// or sphere (`kind == 1`) cloud, seed 42, reduced to `count` (4..=128) vertices
/// via `b3CreateHull`. One yellow block; the v/f/e counts ride in its header.
#[wasm_bindgen]
pub fn geometry_hull_reduction(kind: u32, count: i32) -> Vec<f32> {
    const CAPACITY: usize = 128;
    let mut rng = XorShift32::with_seed(42);
    let mut points = [VEC3_ZERO; CAPACITY];

    if kind == 0 {
        let lower = Vec3 {
            x: -2.0,
            y: -2.0,
            z: -2.0,
        };
        let upper = Vec3 {
            x: 2.0,
            y: 2.0,
            z: 2.0,
        };
        let box_lo = Vec3 {
            x: -1.0,
            y: -1.0,
            z: -1.0,
        };
        let box_hi = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let a = 0.001f32;
        let noise_lo = Vec3 {
            x: -a,
            y: -a,
            z: -a,
        };
        let noise_hi = Vec3 { x: a, y: a, z: a };

        for point in points.iter_mut() {
            let p = random_vec3(&mut rng, lower, upper);
            let p = box3d_rust::math_functions::clamp(p, box_lo, box_hi);
            let f = random_vec3(&mut rng, noise_lo, noise_hi);
            *point = box3d_rust::math_functions::add(p, f);
        }
    } else {
        for point in points.iter_mut() {
            *point = random_unit_vector(&mut rng);
        }
    }

    let mut out = vec![1.0];
    match create_hull(&points, count) {
        Some(hull) => pack_hull(&mut out, &hull),
        None => pack_empty(&mut out),
    }
    out
}

/// Hull Transform (sample_geometry.cpp:362-488). A 9-sided cylinder cloned by
/// `b3CloneAndTransformHull` under a 9-slider transform. Two blocks: the green
/// original and the yellow transformed clone (the TS side offsets them ±2 in x
/// for the side-by-side compare, mirroring the C `transform1`/`transform2`).
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn geometry_hull_transform(
    sx: f32,
    sy: f32,
    sz: f32,
    rx: f32,
    ry: f32,
    rz: f32,
    px: f32,
    py: f32,
    pz: f32,
) -> Vec<f32> {
    let mut out = vec![2.0];

    let original = create_cylinder(1.0, 0.5, 0.0, 9);
    match &original {
        Some(hull) => pack_hull(&mut out, hull),
        None => pack_empty(&mut out),
    }

    let scale = Vec3 {
        x: sx,
        y: sy,
        z: sz,
    };
    let transform = Transform {
        p: Vec3 {
            x: px,
            y: py,
            z: pz,
        },
        q: rotation_from_degrees_pi180(rx, ry, rz),
    };

    match original
        .as_ref()
        .and_then(|hull| clone_and_transform_hull(hull, transform, scale))
    {
        Some(clone) => pack_hull(&mut out, &clone),
        None => pack_empty(&mut out),
    }
    out
}

/// Capsule Mass (sample_geometry.cpp:490-648). Compares mass + diagonal inertia
/// of a `sides`-tessellated capsule hull, the analytic capsule, and a box hull.
///
/// Layout: `[ capsuleHullBlock, boxHullBlock, capLen, capRadius,
///            massHull, massCap, massBox,
///            ixxHull, ixxCap, ixxBox,  iyyHull, iyyCap, iyyBox,
///            izzHull, izzCap, izzBox ]`. `capLen`/`capRadius` size the analytic
/// capsule the TS side draws (from `x = -capLen/2` to `x = +capLen/2`).
#[wasm_bindgen]
pub fn geometry_capsule_mass(sides: i32) -> Vec<f32> {
    const MAX_SIDES: i32 = 6;
    let radius = 1.0f32;
    let length = 2.0f32;

    let capsule = Capsule {
        center1: Vec3 {
            x: -0.5 * length,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.5 * length,
            y: 0.0,
            z: 0.0,
        },
        radius,
    };
    let box_hull = make_box_hull(radius + 0.5 * length, radius, radius);

    // C `CreateCapsuleHull`: two hemisphere point rings at x = ±1, sampled with
    // sample-local libc trig (mirrored here by ordinary f32 sin/cos).
    let hull = if (3..=MAX_SIDES).contains(&sides) {
        let count = (2 * sides * sides) as usize;
        let mut points = Vec::with_capacity(count);
        let d = PI / (sides as f32 - 1.0);

        let mut angle1 = -0.5 * PI;
        for _ in 0..sides {
            let s1 = angle1.sin();
            let c1 = angle1.cos();
            let mut angle2 = -0.5 * PI;
            for _ in 0..sides {
                points.push(Vec3 {
                    x: 1.0 + radius * c1,
                    y: radius * s1 * angle2.cos(),
                    z: radius * s1 * angle2.sin(),
                });
                angle2 += d;
            }
            angle1 += d;
        }

        angle1 = 0.5 * PI;
        for _ in 0..sides {
            let s1 = angle1.sin();
            let c1 = angle1.cos();
            let mut angle2 = -0.5 * PI;
            for _ in 0..sides {
                points.push(Vec3 {
                    x: -1.0 + radius * c1,
                    y: radius * s1 * angle2.cos(),
                    z: radius * s1 * angle2.sin(),
                });
                angle2 += d;
            }
            angle1 += d;
        }

        create_hull(&points, count as i32)
    } else {
        None
    };

    let mut out = vec![2.0];
    match &hull {
        Some(h) => pack_hull(&mut out, h),
        None => pack_empty(&mut out),
    }
    pack_hull(&mut out, &box_hull.base);

    out.push(length);
    out.push(radius);

    let cap_mass = compute_capsule_mass(&capsule, 1.0);
    let box_mass = compute_hull_mass(&box_hull.base, 1.0);
    let hull_mass = hull.as_ref().map(|h| compute_hull_mass(h, 1.0));

    let hull_val =
        |f: fn(&box3d_rust::geometry::MassData) -> f32| hull_mass.as_ref().map(f).unwrap_or(0.0);

    // mass: hull, capsule, box
    out.push(hull_val(|m| m.mass));
    out.push(cap_mass.mass);
    out.push(box_mass.mass);
    // Ixx
    out.push(hull_val(|m| m.inertia.cx.x));
    out.push(cap_mass.inertia.cx.x);
    out.push(box_mass.inertia.cx.x);
    // Iyy
    out.push(hull_val(|m| m.inertia.cy.y));
    out.push(cap_mass.inertia.cy.y);
    out.push(box_mass.inertia.cy.y);
    // Izz
    out.push(hull_val(|m| m.inertia.cz.z));
    out.push(cap_mass.inertia.cz.z);
    out.push(box_mass.inertia.cz.z);

    out
}
