//! Collision / Initial Overlap — faithful port of `sample_collision.cpp`
//! `InitialOverlap` (line 1575). A capsule is shape-cast with a zero-length
//! translation against a scaled quad mesh; the `initial overlap` toggle decides
//! whether an already-touching start reports a hit.

use crate::vis::mesh_triangle_edges;
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::distance::ShapeProxy;
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, transform_point, Transform, Vec3, DEG_TO_RAD, VEC3_AXIS_Z,
};
use box3d_rust::mesh::{create_mesh, MeshData, MeshDef};
use box3d_rust::shape::create_mesh_shape;
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def};
use box3d_rust::world::{world_cast_shape, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

use super::shared::{cast_closest, CastContext};

const SCALE: Vec3 = Vec3 {
    x: 4.0,
    y: 4.0,
    z: 4.0,
};

struct State {
    world: World,
    initial_overlap: bool,
    /// Cached zero-length cast result + world-space mesh wireframe. The cast depends
    /// only on the `initial overlap` toggle, and the mesh body is static, so C
    /// computes both on input; we cache them instead of recomputing each render.
    cast: Vec<f32>,
    wireframe: Vec<f32>,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    STATE.with(|cell| {
        f(cell
            .borrow_mut()
            .as_mut()
            .expect("io not initialized - call io_reset first"))
    })
}

#[wasm_bindgen]
pub fn io_reset() {
    crate::interact::reset_scene_scales();
    let mut world = World::new(&box3d_rust::types::default_world_def());

    // Quad mesh: two triangles (line 1586).
    let def = MeshDef {
        vertices: vec![
            Vec3 {
                x: -0.5,
                y: 0.5,
                z: 0.5,
            },
            Vec3 {
                x: -0.5,
                y: 0.5,
                z: -0.5,
            },
            Vec3 {
                x: -0.5,
                y: -0.5,
                z: -0.5,
            },
            Vec3 {
                x: -0.5,
                y: -0.5,
                z: 0.5,
            },
        ],
        indices: vec![0, 1, 2, 2, 3, 0],
        ..Default::default()
    };
    let mesh = create_mesh(&def, None).expect("quad mesh");

    let mut body_def = default_body_def();
    body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, 10.0 * DEG_TO_RAD);
    let body_id = create_body(&mut world, &body_def);
    let shape_def = default_shape_def();
    create_mesh_shape(&mut world, body_id, &shape_def, &mesh, SCALE);

    let initial_overlap = true;
    let cast = compute_cast(&world, initial_overlap);
    let wireframe = compute_wireframe(&world, &mesh, body_id);

    STATE.with(|cell| {
        *cell.borrow_mut() = Some(State {
            world,
            initial_overlap,
            cast,
            wireframe,
        });
    });
}

#[wasm_bindgen]
pub fn io_set_initial_overlap(flag: i32) {
    with_state(|state| {
        state.initial_overlap = flag != 0;
        // Only the cast depends on the toggle; the mesh body is static (wireframe
        // stays as cached at reset).
        state.cast = compute_cast(&state.world, state.initial_overlap);
    });
}

/// Cached mesh wireframe edges transformed to world by the (rotated) body.
#[wasm_bindgen]
pub fn io_surface_wireframe() -> Vec<f32> {
    with_state(|state| state.wireframe.clone())
}

/// Mesh wireframe edges transformed to world by the (rotated) static body.
fn compute_wireframe(world: &World, mesh: &MeshData, body_id: BodyId) -> Vec<f32> {
    let edges = mesh_triangle_edges(mesh, SCALE);
    let xf = get_body_transform(world, body_id.index1 - 1);
    let local = Transform {
        p: Vec3 {
            x: xf.p.x as f32,
            y: xf.p.y as f32,
            z: xf.p.z as f32,
        },
        q: xf.q,
    };
    let mut out = Vec::with_capacity(edges.len());
    for i in (0..edges.len()).step_by(6) {
        let a = transform_point(
            local,
            Vec3 {
                x: edges[i],
                y: edges[i + 1],
                z: edges[i + 2],
            },
        );
        let b = transform_point(
            local,
            Vec3 {
                x: edges[i + 3],
                y: edges[i + 4],
                z: edges[i + 5],
            },
        );
        out.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
    }
    out
}

/// The cached fixed capsule + zero-length cast result.
/// Packing: `[ c1(3), c2(3), radius, hit, fraction, px,py,pz, nx,ny,nz ]`.
#[wasm_bindgen]
pub fn io_cast() -> Vec<f32> {
    with_state(|state| state.cast.clone())
}

/// The fixed capsule + zero-length cast result.
/// `[ c1(3), c2(3), radius, hit, fraction, px,py,pz, nx,ny,nz ]`.
/// `fraction` mirrors C `context.count > 0 ? context.fractions[0] : 1.0`.
fn compute_cast(world: &World, initial_overlap: bool) -> Vec<f32> {
    // C: offset = {-2.1,-0.8,0.95}, capsule ±y along up, radius 0.25 (line 1638).
    let offset = Vec3 {
        x: -2.1,
        y: -0.8,
        z: 0.95,
    };
    let c1 = offset;
    let c2 = Vec3 {
        x: offset.x,
        y: offset.y + 1.0,
        z: offset.z,
    };
    let radius = 0.25;

    let mut proxy = ShapeProxy {
        count: 2,
        radius,
        ..Default::default()
    };
    proxy.points[0] = c1;
    proxy.points[1] = c2;

    let mut ctx = CastContext {
        initial_overlap,
        ..Default::default()
    };
    let filter = default_query_filter();
    // Zero-length cast (line 1650).
    let translation = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    world_cast_shape(
        world,
        box3d_rust::math_functions::POS_ZERO,
        &proxy,
        translation,
        &filter,
        |sid, p, n, f, m, t, _c| cast_closest(world, &mut ctx, sid, p, n, f, m, t),
    );

    // C Step: fraction = count > 0 ? fractions[0] : 1.0
    let fraction = if ctx.count > 0 {
        ctx.fractions[0]
    } else {
        1.0
    };

    let mut out = vec![c1.x, c1.y, c1.z, c2.x, c2.y, c2.z, radius];
    if ctx.count > 0 {
        out.push(1.0);
        out.push(fraction);
        out.extend_from_slice(&[
            ctx.points[0].x as f32,
            ctx.points[0].y as f32,
            ctx.points[0].z as f32,
            ctx.normals[0].x,
            ctx.normals[0].y,
            ctx.normals[0].z,
        ]);
    } else {
        out.push(0.0);
        out.push(fraction);
        out.extend_from_slice(&[0.0; 6]);
    }
    out
}
