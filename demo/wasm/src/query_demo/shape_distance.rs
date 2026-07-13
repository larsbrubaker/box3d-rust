//! Collision / Shape Distance — faithful port of `sample_collision.cpp`
//! `ShapeDistance` (line 2050). Shape A is fixed at the origin, shape B is dragged
//! / rotated by the mouse; `b3ShapeDistance` reports the closest points, normal,
//! and (optionally) the GJK simplex history. `transformA` is identity, so A-frame
//! points are world points as-is.

use box3d_rust::distance::{
    shape_distance, DistanceInput, ShapeProxy, Simplex, SimplexCache, SimplexVertex,
};
use box3d_rust::hull::{make_box_hull, BoxHull};
use box3d_rust::math_functions::{Quat, Vec3, WorldTransform, VEC3_ZERO, WORLD_TRANSFORM_IDENTITY};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

// C ShapeType enum (line 2053): point, segment, triangle, box.
const TYPE_POINT: i32 = 0;
const TYPE_SEGMENT: i32 = 1;
const TYPE_TRIANGLE: i32 = 2;

const SIMPLEX_CAPACITY: usize = 20;

struct State {
    box_hull: BoxHull,
    type_a: i32,
    type_b: i32,
    radius_a: f32,
    radius_b: f32,
    transform_b: WorldTransform,
    cache: SimplexCache,
    use_cache: bool,
    draw_simplex: bool,
    show_indices: bool,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    STATE.with(|cell| {
        f(cell
            .borrow_mut()
            .as_mut()
            .expect("sd not initialized - call sd_reset first"))
    })
}

const POINT: Vec3 = VEC3_ZERO;
const SEGMENT: [Vec3; 2] = [
    Vec3 {
        x: -0.5,
        y: 0.0,
        z: 0.0,
    },
    Vec3 {
        x: 0.5,
        y: 0.0,
        z: 0.0,
    },
];
const TRIANGLE: [Vec3; 3] = [
    Vec3 {
        x: -1.5,
        y: 0.0,
        z: 0.0,
    },
    Vec3 {
        x: 1.5,
        y: 0.0,
        z: 0.0,
    },
    Vec3 {
        x: 0.0,
        y: 0.0,
        z: 2.0,
    },
];

fn make_proxy(state: &State, shape_type: i32, radius: f32) -> ShapeProxy {
    let mut p = ShapeProxy {
        radius,
        ..Default::default()
    };
    match shape_type {
        TYPE_POINT => {
            p.points[0] = POINT;
            p.count = 1;
        }
        TYPE_SEGMENT => {
            p.points[..2].copy_from_slice(&SEGMENT);
            p.count = 2;
        }
        TYPE_TRIANGLE => {
            p.points[..3].copy_from_slice(&TRIANGLE);
            p.count = 3;
        }
        _ => {
            let count = state.box_hull.base.vertex_count;
            for i in 0..count as usize {
                p.points[i] = state.box_hull.box_points[i];
            }
            p.count = count;
        }
    }
    p
}

/// C `ComputeWitnessPoints` (line 2308): barycentric blend of the simplex.
fn compute_witness(simplex: &Simplex) -> (Vec3, Vec3) {
    let vs = &simplex.vertices;
    let blend = |get: fn(&SimplexVertex) -> Vec3| -> Vec3 {
        let mut r = VEC3_ZERO;
        let n = if simplex.count == 4 { 4 } else { simplex.count } as usize;
        for v in vs.iter().take(n) {
            let g = get(v);
            r.x += v.a * g.x;
            r.y += v.a * g.y;
            r.z += v.a * g.z;
        }
        r
    };
    match simplex.count {
        1 => (vs[0].w_a, vs[0].w_b),
        // count 4 forces identical points; wA blend used for both (line 2333).
        4 => {
            let a = blend(|v| v.w_a);
            (a, a)
        }
        _ => (blend(|v| v.w_a), blend(|v| v.w_b)),
    }
}

#[wasm_bindgen]
pub fn sd_reset() {
    crate::interact::reset_scene_scales();
    STATE.with(|cell| {
        *cell.borrow_mut() = Some(State {
            // C: m_box = b3MakeBoxHull(0.125, 0.25, 0.5) (line 2076).
            box_hull: make_box_hull(0.125, 0.25, 0.5),
            type_a: TYPE_TRIANGLE,
            type_b: 3, // e_box
            radius_a: 0.0,
            radius_b: 0.0,
            // C: m_transformB = {{0,1,0}, identity} (line 2079).
            transform_b: WorldTransform {
                p: Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                q: box3d_rust::math_functions::QUAT_IDENTITY,
            },
            cache: SimplexCache::default(),
            use_cache: false,
            draw_simplex: false,
            show_indices: false,
        });
    });
}

#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn sd_set_params(
    type_a: i32,
    type_b: i32,
    radius_a: f32,
    radius_b: f32,
    use_cache: i32,
    show_indices: i32,
    draw_simplex: i32,
) {
    with_state(|state| {
        state.type_a = type_a.clamp(0, 3);
        state.type_b = type_b.clamp(0, 3);
        state.radius_a = radius_a.clamp(0.0, 0.5);
        state.radius_b = radius_b.clamp(0.0, 0.5);
        state.use_cache = use_cache != 0;
        state.show_indices = show_indices != 0;
        state.draw_simplex = draw_simplex != 0;
    });
}

/// Set shape B's world transform (TS computes the drag/rotate math with the
/// camera, mirroring C `MouseMove`, line 2289).
#[wasm_bindgen]
pub fn sd_set_transform_b(px: f32, py: f32, pz: f32, qx: f32, qy: f32, qz: f32, qw: f32) {
    with_state(|state| {
        state.transform_b = WorldTransform {
            p: Vec3 {
                x: px,
                y: py,
                z: pz,
            },
            q: Quat {
                v: Vec3 {
                    x: qx,
                    y: qy,
                    z: qz,
                },
                s: qw,
            },
        };
    });
}

/// Run the distance query and pack the drawer geometry. See module docs / the TS
/// consumer for the layout.
#[wasm_bindgen]
pub fn sd_step(simplex_index: i32) -> Vec<f32> {
    with_state(|state| {
        let proxy_a = make_proxy(state, state.type_a, state.radius_a);
        let proxy_b = make_proxy(state, state.type_b, state.radius_b);

        let input = DistanceInput {
            proxy_a,
            proxy_b,
            transform: box3d_rust::math_functions::inv_mul_world_transforms(
                WORLD_TRANSFORM_IDENTITY,
                state.transform_b,
            ),
            use_radii: state.radius_a > 0.0 || state.radius_b > 0.0,
        };

        if !state.use_cache {
            state.cache.count = 0;
        }

        let mut simplexes = [Simplex::default(); SIMPLEX_CAPACITY];
        let output = shape_distance(&input, &mut state.cache, Some(&mut simplexes[..]));
        let simplex_count = output.simplex_count;

        let tb = state.transform_b;
        let mut out = vec![
            state.type_a as f32,
            state.radius_a,
            state.type_b as f32,
            state.radius_b,
            tb.p.x,
            tb.p.y,
            tb.p.z,
            tb.q.v.x,
            tb.q.v.y,
            tb.q.v.z,
            tb.q.s,
            output.distance,
            output.iterations as f32,
            if state.draw_simplex { 1.0 } else { 0.0 },
            if state.show_indices { 1.0 } else { 0.0 },
            simplex_count as f32,
            // closest-point path (A-frame == world, transformA identity)
            output.point_a.x,
            output.point_a.y,
            output.point_a.z,
            output.point_b.x,
            output.point_b.y,
            output.point_b.z,
            output.normal.x,
            output.normal.y,
            output.normal.z,
        ];

        // Simplex path.
        if state.draw_simplex && simplex_count > 0 {
            let idx = simplex_index.clamp(0, simplex_count - 1);
            let simplex = &simplexes[idx as usize];
            out.push(1.0);
            out.push(idx as f32);
            // The first recorded simplex has no valid barycentric coords (line 2385).
            if idx > 0 {
                let (a, b) = compute_witness(simplex);
                out.push(1.0);
                out.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
            } else {
                out.push(0.0);
                out.extend_from_slice(&[0.0; 6]);
            }
            out.push(simplex.count as f32);
            for i in 0..simplex.count as usize {
                let v = &simplex.vertices[i];
                out.extend_from_slice(&[v.w_a.x, v.w_a.y, v.w_a.z, v.w_b.x, v.w_b.y, v.w_b.z]);
            }
        } else {
            out.push(0.0);
            out.push(0.0);
            out.push(0.0);
            out.extend_from_slice(&[0.0; 6]);
            out.push(0.0);
        }

        // Cache indices (C text readout, line 2440).
        out.push(state.cache.count as f32);
        for i in 0..state.cache.count as usize {
            out.push(state.cache.index_a[i] as f32);
            out.push(state.cache.index_b[i] as f32);
        }

        out
    })
}
