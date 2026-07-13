//! Collision / Mesh Scale — faithful port of `sample_collision.cpp` `MeshScale`
//! (line 774). A scalable box mesh is cast against by a sphere (or ray); the scale
//! and cast start animate via sliders.
//!
//! The C sample re-scales the live shape with `b3Shape_SetMesh`; the port now
//! mirrors that exactly, mutating the existing mesh shape in place on each slider
//! change (no body rebuild, no proxy flicker).

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::vis::mesh_triangle_edges;
use box3d_rust::body::create_body;
use box3d_rust::id::ShapeId;
use box3d_rust::math_functions::{Pos, Vec3, VEC3_ZERO};
use box3d_rust::mesh::{create_box_mesh, MeshData};
use box3d_rust::shape::{create_mesh_shape, shape_set_mesh};
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def};
use box3d_rust::world::{world_cast_ray_closest, world_cast_shape, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

use super::shared::{cast_closest, CastContext};

struct State {
    world: World,
    mesh: MeshData,
    mesh_shape: ShapeId,
    scale: Vec3,
    start: Pos,
    sphere_cast: bool,
    /// Cached cast result + mesh wireframe. Both depend only on values set at reset
    /// or in `msc_set_params`, so C computes them on input; we recompute the cache
    /// only when a slider/toggle changes instead of every render frame.
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
            .expect("msc not initialized - call msc_reset first"))
    })
}

fn build_mesh_body(world: &mut World, mesh: &MeshData, scale: Vec3) -> ShapeId {
    let body_def = default_body_def();
    let shape_def = default_shape_def();
    let body = create_body(world, &body_def);
    create_mesh_shape(world, body, &shape_def, mesh, scale)
}

#[wasm_bindgen]
pub fn msc_reset() {
    crate::interact::reset_scene_scales();
    let mut world = World::new(&box3d_rust::types::default_world_def());
    // C: m_mesh = b3CreateBoxMesh({0,0,0},{0.5,0.5,0.5}, true) (line 790).
    let mesh = create_box_mesh(
        VEC3_ZERO,
        Vec3 {
            x: 0.5,
            y: 0.5,
            z: 0.5,
        },
        true,
    )
    .expect("box mesh");
    let scale = Vec3 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    };
    let mesh_shape = build_mesh_body(&mut world, &mesh, scale);
    // C: m_start = {-2,0,0} (line 798).
    let start = Pos {
        x: -2.0,
        y: 0.0,
        z: 0.0,
    };
    let sphere_cast = true;
    let cast = compute_cast(&world, start, sphere_cast);
    let wireframe = mesh_triangle_edges(&mesh, scale);

    STATE.with(|cell| {
        *cell.borrow_mut() = Some(State {
            world,
            mesh,
            mesh_shape,
            scale,
            start,
            sphere_cast,
            cast,
            wireframe,
        });
    });
}

/// Scale sliders + start Y/Z + sphere-cast toggle (C `DrawControls`, line 812).
#[wasm_bindgen]
pub fn msc_set_params(sx: f32, sy: f32, sz: f32, start_y: f32, start_z: f32, sphere_cast: i32) {
    with_state(|state| {
        let new_scale = Vec3 {
            x: sx.clamp(-2.0, 2.0),
            y: sy.clamp(-2.0, 2.0),
            z: sz.clamp(-2.0, 2.0),
        };
        if new_scale != state.scale {
            // C `MeshScale::DrawControls`: b3Shape_SetMesh(m_meshShapeId, m_mesh,
            // m_scale) mutates the live shape in place. Disjoint field borrows:
            // world (mut), mesh + mesh_shape (shared) are separate fields.
            state.scale = new_scale;
            shape_set_mesh(&mut state.world, state.mesh_shape, &state.mesh, new_scale);
        }
        // C: start.x stays -2 (delta.x from init); Y/Z from sliders (line 826).
        state.start = Pos {
            x: -2.0,
            y: start_y.clamp(-2.0, 2.0),
            z: start_z.clamp(-2.0, 2.0),
        };
        state.sphere_cast = sphere_cast != 0;

        // Refresh the cached cast + wireframe from the new inputs (see State docs).
        state.cast = compute_cast(&state.world, state.start, state.sphere_cast);
        state.wireframe = mesh_triangle_edges(&state.mesh, state.scale);
    });
}

/// Cached mesh wireframe edges at the current scale (recomputed on scale change).
#[wasm_bindgen]
pub fn msc_wireframe() -> Vec<f32> {
    with_state(|state| state.wireframe.clone())
}

/// The cached cast visualization (recomputed on any slider/toggle change).
#[wasm_bindgen]
pub fn msc_cast() -> Vec<f32> {
    with_state(|state| state.cast.clone())
}

/// Cast visualization.
/// `[ sx,sy,sz, ex,ey,ez, sphereCast, hit, spherePx,spherePy,spherePz, radius,
///    hasPoint, px,py,pz, nx,ny,nz ]`.
fn compute_cast(world: &World, start: Pos, sphere_cast: bool) -> Vec<f32> {
    let translation = Vec3 {
        x: 4.0,
        y: 0.0,
        z: 0.0,
    };
    let filter = default_query_filter();
    let end = Pos {
        x: start.x + translation.x,
        y: start.y + translation.y,
        z: start.z + translation.z,
    };
    let mut out = vec![
        start.x as f32,
        start.y as f32,
        start.z as f32,
        end.x as f32,
        end.y as f32,
        end.z as f32,
        if sphere_cast { 1.0 } else { 0.0 },
    ];

    if sphere_cast {
        let radius = 0.25;
        let mut ctx = CastContext::default();
        let mut proxy = box3d_rust::distance::ShapeProxy {
            count: 1,
            radius,
            ..Default::default()
        };
        proxy.points[0] = VEC3_ZERO;
        world_cast_shape(
            world,
            start,
            &proxy,
            translation,
            &filter,
            |sid, p, n, f, m, t, _c| cast_closest(world, &mut ctx, sid, p, n, f, m, t),
        );
        if ctx.count > 0 {
            // C draws the sphere at fraction*translation (line 856).
            let f = ctx.fractions[0];
            out.push(1.0);
            out.extend_from_slice(&[f * translation.x, f * translation.y, f * translation.z]);
            out.push(radius);
            out.push(1.0);
            out.extend_from_slice(&[
                ctx.points[0].x as f32,
                ctx.points[0].y as f32,
                ctx.points[0].z as f32,
                ctx.normals[0].x,
                ctx.normals[0].y,
                ctx.normals[0].z,
            ]);
        } else {
            // Miss: gray sphere at translation (line 865).
            out.push(0.0);
            out.extend_from_slice(&[translation.x, translation.y, translation.z]);
            out.push(radius);
            out.push(0.0);
            out.extend_from_slice(&[0.0; 6]);
        }
    } else {
        // Ray cast (line 871).
        let result = world_cast_ray_closest(world, start, translation, &filter);
        out.push(if result.hit { 1.0 } else { 0.0 }); // hit
        out.extend_from_slice(&[0.0, 0.0, 0.0]); // spherePos unused
        out.push(0.25); // radius unused
        if result.hit {
            out.push(1.0);
            out.extend_from_slice(&[
                result.point.x as f32,
                result.point.y as f32,
                result.point.z as f32,
                result.normal.x,
                result.normal.y,
                result.normal.z,
            ]);
        } else {
            out.push(0.0);
            out.extend_from_slice(&[0.0; 6]);
        }
    }
    out
}
