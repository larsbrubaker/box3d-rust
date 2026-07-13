//! Contact-manifold viewers — a 1:1 port of `samples/sample_manifold.cpp`.
//!
//! Nine interactive collision viewers exercise every `b3Collide*` narrow-phase
//! entry point. Unlike the dynamics demos there is no world and no stepping: shape
//! A is fixed, shape B is posed by the mouse (drag translates, Shift-drag rotates),
//! and each frame recomputes one manifold. The C samples derive from two shared
//! bases: `Manifold` (points/normal live in frame A) and `TriangleManifold`
//! (points/normal live in frame B, with a triangle drawn in frame A). The optional
//! `b3SimplexCache` / `b3SATCache` persist across frames only while "Use cache" is
//! on, and the manual-feature radios override the SAT axis — both faithfully kept
//! here because the cache-hit / SAT-type readouts depend on that persistence.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use std::cell::RefCell;

use wasm_bindgen::prelude::*;

use box3d_rust::constants::MAX_MANIFOLD_POINTS;
use box3d_rust::distance::SimplexCache;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull, BoxHull};
use box3d_rust::manifold::{
    collide_capsule_and_sphere, collide_capsule_and_triangle, collide_capsules,
    collide_hull_and_capsule, collide_hull_and_sphere, collide_hull_and_triangle, collide_hulls,
    collide_sphere_and_triangle, collide_spheres, LocalManifold, SatCache, SeparatingFeature,
};
use box3d_rust::math_functions::{
    inv_mul_transforms, make_quat_from_axis_angle, rotate_vector, transform_point, Quat, Transform,
    Vec3, PI, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ZERO,
};

// --- Scene / shape kind tags (kept in sync with manifolds.ts) ----------------

/// Render base: points/normal in frame A (Manifold) vs frame B (TriangleManifold).
const BASE_MANIFOLD: f32 = 0.0;
const BASE_TRIANGLE: f32 = 1.0;

const SHAPE_SPHERE: f32 = 0.0;
const SHAPE_CAPSULE: f32 = 1.0;
const SHAPE_BOX: f32 = 2.0;
const SHAPE_TRIANGLE: f32 = 3.0;

/// A collide entry point paired with the shapes it consumes. One variant per C
/// sample class; the discriminant is the scene id used by the TS route.
#[derive(Clone, Copy)]
enum Kind {
    SphereSphere,
    CapsuleSphere,
    HullSphere,
    TriangleSphere,
    CapsuleCapsule,
    CapsuleHull,
    TriangleCapsule,
    HullHull,
    TriangleHull,
}

impl Kind {
    fn from_id(id: i32) -> Kind {
        match id {
            0 => Kind::SphereSphere,
            1 => Kind::CapsuleSphere,
            2 => Kind::HullSphere,
            3 => Kind::TriangleSphere,
            4 => Kind::CapsuleCapsule,
            5 => Kind::CapsuleHull,
            6 => Kind::TriangleCapsule,
            7 => Kind::HullHull,
            _ => Kind::TriangleHull,
        }
    }

    /// True for the `TriangleManifold`-derived samples (frame-B rendering).
    fn is_triangle(self) -> bool {
        matches!(
            self,
            Kind::TriangleSphere | Kind::TriangleCapsule | Kind::TriangleHull
        )
    }
}

fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

/// A camera pose (`b3Camera::SetView`, degrees) the scene wants applied on entry.
struct Camera {
    yaw: f32,
    pitch: f32,
    dist: f32,
    target: Vec3,
}

/// The base `Manifold`/`TriangleManifold` camera unless a sample overrides it.
const BASE_CAMERA: Camera = Camera {
    yaw: 35.0,
    pitch: 30.0,
    dist: 50.0,
    target: Vec3 {
        x: 0.0,
        y: 5.0,
        z: 0.0,
    },
};

/// All state for the currently-selected viewer.
struct ManifoldDemo {
    kind: Kind,
    transform_a: Transform,
    transform_b: Transform,
    /// Shape A geometry (sphere/capsule/hull) — some scenes leave unused fields.
    sphere_a: Sphere,
    capsule_a: Capsule,
    hull_a: BoxHull,
    /// Shape B geometry.
    sphere_b: Sphere,
    capsule_b: Capsule,
    hull_b: BoxHull,
    /// Triangle in frame A (TriangleManifold samples).
    triangle: [Vec3; 3],
    triangle_flags: i32,
    camera: Camera,
    simplex_cache: SimplexCache,
    sat_cache: SatCache,
    use_cache: bool,
    manual_feature: i32,
    manifold: LocalManifold,
}

impl ManifoldDemo {
    fn new(scene: i32) -> ManifoldDemo {
        let kind = Kind::from_id(scene);

        // Defaults shared by the two C bases (Manifold / TriangleManifold ctors).
        let mut demo = ManifoldDemo {
            kind,
            transform_a: Transform {
                p: vec3(3.5, 0.5, 0.0),
                q: make_quat_from_axis_angle(VEC3_AXIS_Y, 0.5 * PI),
            },
            transform_b: Transform {
                p: vec3(0.0, 1.5, 3.5),
                q: QUAT_IDENTITY,
            },
            sphere_a: Sphere {
                center: VEC3_ZERO,
                radius: 1.0,
            },
            capsule_a: Capsule {
                center1: VEC3_ZERO,
                center2: VEC3_ZERO,
                radius: 1.0,
            },
            hull_a: make_box_hull(1.0, 1.0, 1.0),
            sphere_b: Sphere {
                center: VEC3_ZERO,
                radius: 1.0,
            },
            capsule_b: Capsule {
                center1: VEC3_ZERO,
                center2: VEC3_ZERO,
                radius: 1.0,
            },
            hull_b: make_box_hull(1.0, 1.0, 1.0),
            triangle: [VEC3_ZERO; 3],
            triangle_flags: 0,
            camera: BASE_CAMERA,
            simplex_cache: SimplexCache::default(),
            sat_cache: SatCache::default(),
            use_cache: false,
            manual_feature: 0,
            manifold: LocalManifold::default(),
        };

        demo.setup();
        demo
    }

    /// Configure shapes + transforms exactly as the C sample constructor does.
    fn setup(&mut self) {
        match self.kind {
            // SphereAndSphere: one sphere used for both bodies (ctor :367).
            Kind::SphereSphere => {
                let sphere = Sphere {
                    center: vec3(0.5, 0.0, -0.25),
                    radius: 2.0,
                };
                self.sphere_a = sphere;
                self.sphere_b = sphere;
            }
            // CapsuleAndSphere (:405-409).
            Kind::CapsuleSphere => {
                self.capsule_a = Capsule {
                    center1: vec3(-2.0, 0.0, 0.0),
                    center2: vec3(2.0, 0.0, 0.0),
                    radius: 1.0,
                };
                self.sphere_b = Sphere {
                    center: VEC3_ZERO,
                    radius: 2.0,
                };
                self.transform_a = Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                };
                self.transform_b = Transform {
                    p: vec3(-4.0, 0.0, 0.0),
                    q: QUAT_IDENTITY,
                };
            }
            // HullAndSphere (:438-442).
            Kind::HullSphere => {
                self.sphere_b = Sphere {
                    center: VEC3_ZERO,
                    radius: 1.0,
                };
                self.hull_a = make_box_hull(2.0, 0.5, 0.5);
                self.transform_a = Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                };
                self.transform_b = Transform {
                    p: vec3(1.5, 0.0, 0.0),
                    q: QUAT_IDENTITY,
                };
            }
            // TriangleAndSphere (:488-500).
            Kind::TriangleSphere => {
                self.camera = Camera {
                    yaw: 0.0,
                    pitch: 30.0,
                    dist: 10.0,
                    target: VEC3_ZERO,
                };
                self.sphere_b = Sphere {
                    center: VEC3_ZERO,
                    radius: 0.25,
                };
                self.triangle = [
                    vec3(0.0, 0.0, 0.0),
                    vec3(4.0, 0.0, 4.0),
                    vec3(4.0, 0.0, 0.0),
                ];
                self.transform_a = Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                };
                self.transform_b = Transform {
                    p: vec3(2.0, 0.5, 1.0),
                    q: QUAT_IDENTITY,
                };
            }
            // CapsuleAndCapsule: one capsule for both bodies (:534-537).
            Kind::CapsuleCapsule => {
                let capsule = Capsule {
                    center1: vec3(-2.0, 0.0, 0.0),
                    center2: vec3(2.0, 0.0, 0.0),
                    radius: 1.0,
                };
                self.capsule_a = capsule;
                self.capsule_b = capsule;
                self.transform_a = Transform {
                    p: vec3(1.0, 1.0, 0.0),
                    q: QUAT_IDENTITY,
                };
                self.transform_b = Transform {
                    p: vec3(-4.0, 1.0, 0.0),
                    q: QUAT_IDENTITY,
                };
            }
            // CapsuleAndHull: hull is body A, capsule is body B (:572-590).
            Kind::CapsuleHull => {
                self.camera = Camera {
                    yaw: 0.0,
                    pitch: 30.0,
                    dist: 5.0,
                    target: VEC3_ZERO,
                };
                self.capsule_b = Capsule {
                    center1: vec3(-1.0, 0.0, 0.0),
                    center2: vec3(1.0, 0.0, 0.0),
                    radius: 0.5,
                };
                self.hull_a = make_box_hull(1.0, 0.5, 0.5);
                self.transform_a = Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                };
                // The exact debug pose the C ctor pins (:589-590).
                self.transform_b = Transform {
                    p: vec3(1.585_237_74, 0.729_615_57, 0.451_690_67),
                    q: Quat {
                        v: vec3(-0.002_565_550_8, -0.020_182_582, 0.126_076_24),
                        s: 0.991_812_0,
                    },
                };
            }
            // TriangleAndCapsule (:639-649).
            Kind::TriangleCapsule => {
                self.camera = Camera {
                    yaw: 0.0,
                    pitch: 30.0,
                    dist: 10.0,
                    target: VEC3_ZERO,
                };
                self.capsule_b = Capsule {
                    center1: vec3(0.0, -0.2, 0.0),
                    center2: vec3(0.0, 0.2, 0.0),
                    radius: 0.05,
                };
                self.triangle = [
                    vec3(-4.0, 0.0, -4.0),
                    vec3(-4.0, 0.0, 0.0),
                    vec3(0.0, 0.0, 0.0),
                ];
                self.transform_a = Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                };
                // The exact debug pose the C ctor pins (:648-649).
                self.transform_b = Transform {
                    p: vec3(-0.5, 0.123_778_24, -0.5),
                    q: Quat {
                        v: vec3(-0.157_559_35, 0.294_042_3, 0.821_513_65),
                        s: -0.462_417_0,
                    },
                };
            }
            // HullAndHull (:696-716).
            Kind::HullHull => {
                self.camera = Camera {
                    yaw: 0.0,
                    pitch: 15.0,
                    dist: 4.0,
                    target: VEC3_ZERO,
                };
                let transform = Transform {
                    p: vec3(1.0, 0.5, 0.0),
                    q: QUAT_IDENTITY,
                };
                self.hull_a = make_transformed_box_hull(0.5, 1.0, 1.0, transform);
                self.hull_b = make_box_hull(0.5, 0.5, 0.5);
                self.transform_a = Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                };
                self.transform_b = Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                };
            }
            // TriangleAndHull (:832-840).
            Kind::TriangleHull => {
                self.camera = Camera {
                    yaw: 0.0,
                    pitch: 30.0,
                    dist: 3.0,
                    target: VEC3_ZERO,
                };
                self.triangle = [
                    vec3(1.0, 0.0, 1.0),
                    vec3(1.0, 0.0, 0.0),
                    vec3(0.0, 0.0, 0.0),
                ];
                self.hull_b = make_box_hull(0.5, 0.5, 0.5);
                self.triangle_flags = 0;
                self.transform_a = Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                };
                self.transform_b = Transform {
                    p: vec3(0.0, 0.45, 0.1),
                    q: QUAT_IDENTITY,
                };
            }
        }
    }

    /// Recompute the manifold in the current pose (C `Sample::Step`).
    fn step(&mut self) {
        let cap = MAX_MANIFOLD_POINTS as i32;

        match self.kind {
            Kind::SphereSphere => {
                let b_to_a = inv_mul_transforms(self.transform_a, self.transform_b);
                collide_spheres(
                    &mut self.manifold,
                    cap,
                    &self.sphere_a,
                    &self.sphere_b,
                    b_to_a,
                );
            }
            Kind::CapsuleSphere => {
                let b_to_a = inv_mul_transforms(self.transform_a, self.transform_b);
                collide_capsule_and_sphere(
                    &mut self.manifold,
                    cap,
                    &self.capsule_a,
                    &self.sphere_b,
                    b_to_a,
                );
            }
            Kind::HullSphere => {
                if !self.use_cache {
                    self.simplex_cache = SimplexCache::default();
                }
                let b_to_a = inv_mul_transforms(self.transform_a, self.transform_b);
                collide_hull_and_sphere(
                    &mut self.manifold,
                    cap,
                    &self.hull_a.base,
                    &self.sphere_b,
                    b_to_a,
                    &mut self.simplex_cache,
                );
            }
            Kind::TriangleSphere => {
                let local = self.local_triangle_in_b();
                collide_sphere_and_triangle(&mut self.manifold, cap, &self.sphere_b, &local);
            }
            Kind::CapsuleCapsule => {
                let b_to_a = inv_mul_transforms(self.transform_a, self.transform_b);
                collide_capsules(
                    &mut self.manifold,
                    cap,
                    &self.capsule_a,
                    &self.capsule_b,
                    b_to_a,
                );
            }
            Kind::CapsuleHull => {
                if !self.use_cache {
                    self.simplex_cache = SimplexCache::default();
                }
                let b_to_a = inv_mul_transforms(self.transform_a, self.transform_b);
                collide_hull_and_capsule(
                    &mut self.manifold,
                    cap,
                    &self.hull_a.base,
                    &self.capsule_b,
                    b_to_a,
                    &mut self.simplex_cache,
                );
            }
            Kind::TriangleCapsule => {
                if !self.use_cache {
                    self.simplex_cache = SimplexCache::default();
                }
                let local = self.local_triangle_in_b();
                collide_capsule_and_triangle(
                    &mut self.manifold,
                    cap,
                    &self.capsule_b,
                    &local,
                    &mut self.simplex_cache,
                );
            }
            Kind::HullHull => {
                self.apply_manual_sat_feature();
                let b_to_a = inv_mul_transforms(self.transform_a, self.transform_b);
                collide_hulls(
                    &mut self.manifold,
                    cap,
                    &self.hull_a.base,
                    &self.hull_b.base,
                    b_to_a,
                    &mut self.sat_cache,
                );
            }
            Kind::TriangleHull => {
                self.apply_manual_sat_feature();
                let local = self.local_triangle_in_b();
                collide_hull_and_triangle(
                    &mut self.manifold,
                    cap,
                    &self.hull_b.base,
                    local[0],
                    local[1],
                    local[2],
                    self.triangle_flags,
                    &mut self.sat_cache,
                );
            }
        }
    }

    /// Put the frame-A triangle into frame B (C `xf = b3InvMulWorldTransforms(B, A)`).
    fn local_triangle_in_b(&self) -> [Vec3; 3] {
        let xf = inv_mul_transforms(self.transform_b, self.transform_a);
        [
            transform_point(xf, self.triangle[0]),
            transform_point(xf, self.triangle[1]),
            transform_point(xf, self.triangle[2]),
        ]
    }

    /// Override the SAT axis from the manual-feature radios, or clear the cache
    /// when "Use cache" is off (C HullAndHull/TriangleAndHull `Step`).
    fn apply_manual_sat_feature(&mut self) {
        if self.use_cache {
            match self.manual_feature {
                1 => self.sat_cache.type_ = SeparatingFeature::ManualFaceAxisA as u8,
                2 => self.sat_cache.type_ = SeparatingFeature::ManualFaceAxisB as u8,
                3 => self.sat_cache.type_ = SeparatingFeature::ManualEdgePairAxis as u8,
                _ => {}
            }
        } else {
            self.sat_cache = SatCache::default();
        }
    }

    /// The world transform the manifold points/normal are expressed in: frame A
    /// for the Manifold base, frame B for the TriangleManifold base.
    fn render_transform(&self) -> Transform {
        if self.kind.is_triangle() {
            self.transform_b
        } else {
            self.transform_a
        }
    }

    /// Pack the manifold into world space for rendering. Layout:
    /// `[count, nx, ny, nz, sat_type, cache_hit, feature, (wx, wy, wz, sep, o1, i1, o2, i2)*]`.
    fn pack_step(&self) -> Vec<f32> {
        let xf = self.render_transform();
        let normal = rotate_vector(xf.q, self.manifold.normal);

        let mut out = vec![
            self.manifold.point_count as f32,
            normal.x,
            normal.y,
            normal.z,
            self.sat_cache.type_ as f32,
            self.sat_cache.hit as f32,
            self.manifold.feature as u8 as f32,
        ];

        for i in 0..self.manifold.point_count as usize {
            let mp = &self.manifold.points[i];
            let world = transform_point(xf, mp.point);
            out.push(world.x);
            out.push(world.y);
            out.push(world.z);
            out.push(mp.separation);
            out.push(mp.pair.owner1 as f32);
            out.push(mp.pair.index1 as f32);
            out.push(mp.pair.owner2 as f32);
            out.push(mp.pair.index2 as f32);
        }
        out
    }

    /// One-time scene description for the TS renderer. See module docs for layout.
    fn pack_info(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(41);
        out.push(if self.kind.is_triangle() {
            BASE_TRIANGLE
        } else {
            BASE_MANIFOLD
        });
        out.extend_from_slice(&[
            self.camera.yaw,
            self.camera.pitch,
            self.camera.dist,
            self.camera.target.x,
            self.camera.target.y,
            self.camera.target.z,
        ]);
        push_transform(&mut out, self.transform_a);
        push_transform(&mut out, self.transform_b);
        self.push_shape_a(&mut out);
        self.push_shape_b(&mut out);
        out
    }

    /// Shape A descriptor (9-slot params region): the triangle for the
    /// TriangleManifold samples, otherwise the sphere/capsule/hull body A.
    fn push_shape_a(&self, out: &mut Vec<f32>) {
        match self.kind {
            Kind::SphereSphere => push_sphere(out, self.sphere_a),
            Kind::CapsuleSphere | Kind::CapsuleCapsule => push_capsule(out, self.capsule_a),
            Kind::HullSphere | Kind::CapsuleHull => push_box(out, &self.hull_a),
            Kind::HullHull => push_box(out, &self.hull_a),
            Kind::TriangleSphere | Kind::TriangleCapsule | Kind::TriangleHull => {
                push_triangle(out, self.triangle)
            }
        }
    }

    /// Shape B descriptor: the convex body drawn in frame B.
    fn push_shape_b(&self, out: &mut Vec<f32>) {
        match self.kind {
            Kind::SphereSphere | Kind::CapsuleSphere | Kind::HullSphere | Kind::TriangleSphere => {
                push_sphere(out, self.sphere_b)
            }
            Kind::CapsuleCapsule | Kind::CapsuleHull | Kind::TriangleCapsule => {
                push_capsule(out, self.capsule_b)
            }
            Kind::HullHull | Kind::TriangleHull => push_box(out, &self.hull_b),
        }
    }
}

fn push_transform(out: &mut Vec<f32>, t: Transform) {
    out.extend_from_slice(&[t.p.x, t.p.y, t.p.z, t.q.v.x, t.q.v.y, t.q.v.z, t.q.s]);
}

/// Append `kind` then a fixed 9-float params region (unused slots are zero).
fn push_shape(out: &mut Vec<f32>, kind: f32, params: &[f32]) {
    out.push(kind);
    for i in 0..9 {
        out.push(params.get(i).copied().unwrap_or(0.0));
    }
}

fn push_sphere(out: &mut Vec<f32>, s: Sphere) {
    push_shape(
        out,
        SHAPE_SPHERE,
        &[s.center.x, s.center.y, s.center.z, s.radius],
    );
}

fn push_capsule(out: &mut Vec<f32>, c: Capsule) {
    push_shape(
        out,
        SHAPE_CAPSULE,
        &[
            c.center1.x,
            c.center1.y,
            c.center1.z,
            c.center2.x,
            c.center2.y,
            c.center2.z,
            c.radius,
        ],
    );
}

/// Append a box hull as half-extents + local center (the transformed-box-hull
/// samples bake a center offset into the hull; every other box has center 0).
fn push_box(out: &mut Vec<f32>, hull: &BoxHull) {
    let center = hull.base.center;
    // Recover half-extents from the hull points: max coordinate magnitude minus
    // center on each axis (box vertices are center ± h).
    let points = box3d_rust::hull::get_hull_points(&hull.base);
    let mut h = VEC3_ZERO;
    for p in points {
        h.x = h.x.max((p.x - center.x).abs());
        h.y = h.y.max((p.y - center.y).abs());
        h.z = h.z.max((p.z - center.z).abs());
    }
    push_shape(
        out,
        SHAPE_BOX,
        &[h.x, h.y, h.z, center.x, center.y, center.z],
    );
}

fn push_triangle(out: &mut Vec<f32>, t: [Vec3; 3]) {
    push_shape(
        out,
        SHAPE_TRIANGLE,
        &[
            t[0].x, t[0].y, t[0].z, t[1].x, t[1].y, t[1].z, t[2].x, t[2].y, t[2].z,
        ],
    );
}

thread_local! {
    static DEMO: RefCell<Option<ManifoldDemo>> = const { RefCell::new(None) };
}

fn with_demo<R>(f: impl FnOnce(&mut ManifoldDemo) -> R) -> R {
    DEMO.with(|cell| {
        let mut borrow = cell.borrow_mut();
        let demo = borrow.get_or_insert_with(|| ManifoldDemo::new(0));
        f(demo)
    })
}

/// Select a manifold viewer (scene id 0..=8, C registration order) and return its
/// one-time scene descriptor for the renderer. See module docs for the layout.
#[wasm_bindgen]
pub fn manifold_reset(scene: i32) -> Vec<f32> {
    DEMO.with(|cell| {
        let demo = ManifoldDemo::new(scene);
        let info = demo.pack_info();
        *cell.borrow_mut() = Some(demo);
        info
    })
}

/// Push shape B's mouse-driven world transform (position + quaternion x,y,z,w).
#[wasm_bindgen]
pub fn manifold_set_transform_b(px: f32, py: f32, pz: f32, qx: f32, qy: f32, qz: f32, qw: f32) {
    with_demo(|demo| {
        demo.transform_b = Transform {
            p: vec3(px, py, pz),
            q: Quat {
                v: vec3(qx, qy, qz),
                s: qw,
            },
        };
    });
}

/// Set the "Use cache" checkbox and the manual-feature radio (0 auto, 1 faceA,
/// 2 faceB, 3 edgePair).
#[wasm_bindgen]
pub fn manifold_set_cache(use_cache: bool, manual_feature: i32) {
    with_demo(|demo| {
        demo.use_cache = use_cache;
        demo.manual_feature = manual_feature;
    });
}

/// Recompute the manifold in the current pose and return it packed in world space.
#[wasm_bindgen]
pub fn manifold_step() -> Vec<f32> {
    with_demo(|demo| {
        demo.step();
        demo.pack_step()
    })
}
