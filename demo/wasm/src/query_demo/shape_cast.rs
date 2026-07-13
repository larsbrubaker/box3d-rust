//! Collision / Shape Cast — faithful port of `sample_collision.cpp` `ShapeCast`
//! (line 890). Rows of static shapes (sphere / capsule / hull / torus mesh) are
//! swept by rows of sphere / capsule / hull casts whose start follows a mouse drag.

use crate::vis::{mesh_triangle_edges, push_poses, VisBody};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::distance::ShapeProxy;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, mul_quat, transform_point, Pos, Transform, Vec3, PI, VEC3_AXIS_X,
    VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ONE, VEC3_ZERO,
};
use box3d_rust::mesh::{create_torus_mesh, MeshData};
use box3d_rust::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
};
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def};
use box3d_rust::world::{world_cast_shape, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

use super::shared::{cast_closest, CastContext};

// translation = 10 * axisZ (line 952).
const TRANSLATION: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 10.0,
};

struct State {
    world: World,
    vis: Vec<VisBody>,
    mesh: MeshData,
    mesh_bodies: Vec<BodyId>,
    offset: Vec3,
    initial_overlap: bool,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    STATE.with(|cell| {
        f(cell
            .borrow_mut()
            .as_mut()
            .expect("sc not initialized - call sc_reset first"))
    })
}

#[wasm_bindgen]
pub fn sc_reset() {
    crate::interact::reset_scene_scales();
    let mut world = World::new(&box3d_rust::types::default_world_def());

    let box_hull = make_box_hull(0.6, 0.6, 0.6);
    let mesh = create_torus_mesh(10, 12, 0.65, 0.35).expect("torus mesh");

    let mut body_def = default_body_def();
    let shape_def = default_shape_def();
    let mut vis = Vec::new();
    let mut mesh_bodies = Vec::new();

    for index in 0..3 {
        let y = 3.0 + 2.0 * index as f32;

        // Sphere r=0.9, rot axisX 0.5PI.
        body_def.position = Pos { x: -6.0, y, z: 0.0 };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, 0.5 * PI);
        let b = create_body(&mut world, &body_def);
        create_sphere_shape(
            &mut world,
            b,
            &shape_def,
            &Sphere {
                center: VEC3_ZERO,
                radius: 0.9,
            },
        );
        vis.push(VisBody::sphere_body(b.index1 - 1, 0.9));

        // Capsule r=0.7, rot axisZ 0.25PI.
        body_def.position = Pos { x: -2.0, y, z: 0.0 };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.25 * PI);
        let b = create_body(&mut world, &body_def);
        let capsule = Capsule {
            center1: Vec3 {
                x: -0.5,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            },
            radius: 0.7,
        };
        create_capsule_shape(&mut world, b, &shape_def, &capsule);
        vis.push(VisBody::capsule_body(b.index1 - 1, &capsule));

        // Hull (box), rot axisZ 0.25PI.
        body_def.position = Pos { x: 2.0, y, z: 0.0 };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.25 * PI);
        let b = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, b, &shape_def, &box_hull.base);
        vis.push(VisBody::box_body(b.index1 - 1, 0.6, 0.6, 0.6));

        // Torus mesh, rot axisX 0.5PI, scale ones.
        body_def.position = Pos { x: 6.0, y, z: 0.0 };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, 0.5 * PI);
        let b = create_body(&mut world, &body_def);
        create_mesh_shape(&mut world, b, &shape_def, &mesh, VEC3_ONE);
        mesh_bodies.push(b);
    }

    STATE.with(|cell| {
        *cell.borrow_mut() = Some(State {
            world,
            vis,
            mesh,
            mesh_bodies,
            offset: VEC3_ZERO,
            initial_overlap: false,
        });
    });
}

#[wasm_bindgen]
pub fn sc_step(dt: f32, sub_steps: i32) {
    with_state(|state| state.world.step(dt, sub_steps));
}

#[wasm_bindgen]
pub fn sc_set_offset(y: f32, z: f32) {
    with_state(|state| {
        state.offset = Vec3 { x: 0.0, y, z };
    });
}

#[wasm_bindgen]
pub fn sc_set_initial_overlap(flag: i32) {
    with_state(|state| state.initial_overlap = flag != 0);
}

#[wasm_bindgen]
pub fn sc_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.vis, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn sc_surface_wireframe() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        for &body in &state.mesh_bodies {
            let edges = mesh_triangle_edges(&state.mesh, VEC3_ONE);
            let xf = get_body_transform(&state.world, body.index1 - 1);
            let local = Transform {
                p: Vec3 {
                    x: xf.p.x as f32,
                    y: xf.p.y as f32,
                    z: xf.p.z as f32,
                },
                q: xf.q,
            };
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
        }
        out
    })
}

fn run_cast(world: &World, proxy: &ShapeProxy, initial_overlap: bool) -> CastContext {
    let mut ctx = CastContext {
        initial_overlap,
        ..Default::default()
    };
    let filter = default_query_filter();
    world_cast_shape(
        world,
        box3d_rust::math_functions::POS_ZERO,
        proxy,
        TRANSLATION,
        &filter,
        |sid, p, n, f, m, t, _c| cast_closest(world, &mut ctx, sid, p, n, f, m, t),
    );
    ctx
}

/// The 4 sphere + 4 capsule + 4 hull casts.
/// Sphere (14): `offset(3), r, hit, moved(3), point(3), normal(3)`.
/// Capsule (17): `c1(3), c2(3), r, hit, movedOff(3), point(3), normal(3)`.
/// Hull (18): `center(3), q(4), half, hit, movedOff(3), point(3), normal(3)`.
#[wasm_bindgen]
pub fn sc_casts() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        let world = &state.world;
        let io = state.initial_overlap;
        let co = state.offset;

        // Spheres (line 944): r=0.3 at {-6+4ci,3,-5}+offset.
        for ci in 0..4 {
            let offset = Vec3 {
                x: -6.0 + 4.0 * ci as f32 + co.x,
                y: 3.0 + co.y,
                z: -5.0 + co.z,
            };
            let mut proxy = ShapeProxy {
                count: 1,
                radius: 0.3,
                ..Default::default()
            };
            proxy.points[0] = offset;
            let ctx = run_cast(world, &proxy, io);
            let frac = if ctx.count > 0 { ctx.fractions[0] } else { 1.0 };
            let moved = Vec3 {
                x: offset.x + frac * TRANSLATION.x,
                y: offset.y + frac * TRANSLATION.y,
                z: offset.z + frac * TRANSLATION.z,
            };
            out.extend_from_slice(&[offset.x, offset.y, offset.z, 0.3]);
            push_hit(&mut out, &ctx, moved);
        }

        // Capsules (line 983): local ±0.2, r=0.2 at {-6+4ci,5,-5}+offset.
        for ci in 0..4 {
            let offset = Vec3 {
                x: -6.0 + 4.0 * ci as f32 + co.x,
                y: 5.0 + co.y,
                z: -5.0 + co.z,
            };
            let c1 = Vec3 {
                x: offset.x - 0.2,
                y: offset.y - 0.2,
                z: offset.z - 0.2,
            };
            let c2 = Vec3 {
                x: offset.x + 0.2,
                y: offset.y + 0.2,
                z: offset.z + 0.2,
            };
            let mut proxy = ShapeProxy {
                count: 2,
                radius: 0.2,
                ..Default::default()
            };
            proxy.points[0] = c1;
            proxy.points[1] = c2;
            let ctx = run_cast(world, &proxy, io);
            let frac = if ctx.count > 0 { ctx.fractions[0] } else { 1.0 };
            let moved_off = Vec3 {
                x: frac * TRANSLATION.x,
                y: frac * TRANSLATION.y,
                z: frac * TRANSLATION.z,
            };
            out.extend_from_slice(&[c1.x, c1.y, c1.z, c2.x, c2.y, c2.z, 0.2]);
            push_hit_off(&mut out, &ctx, moved_off);
        }

        // Hulls (line 1021): box 0.3 rotated qx*qy*qz at {-6+4ci,7,-5}+offset.
        let qx = make_quat_from_axis_angle(VEC3_AXIS_X, 0.25 * PI);
        let qy = make_quat_from_axis_angle(VEC3_AXIS_Y, 0.25 * PI);
        let qz = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.25 * PI);
        let q = mul_quat(qx, mul_quat(qy, qz));
        for ci in 0..4 {
            let offset = Vec3 {
                x: -6.0 + 4.0 * ci as f32 + co.x,
                y: 7.0 + co.y,
                z: -5.0 + co.z,
            };
            let box_hull = make_transformed_box_hull(0.3, 0.3, 0.3, Transform { p: offset, q });
            let count = box_hull.base.vertex_count;
            let mut proxy = ShapeProxy {
                count,
                radius: 0.0,
                ..Default::default()
            };
            for j in 0..count as usize {
                proxy.points[j] = box_hull.box_points[j];
            }
            let ctx = run_cast(world, &proxy, io);
            let frac = if ctx.count > 0 { ctx.fractions[0] } else { 1.0 };
            let moved_off = Vec3 {
                x: frac * TRANSLATION.x,
                y: frac * TRANSLATION.y,
                z: frac * TRANSLATION.z,
            };
            out.extend_from_slice(&[offset.x, offset.y, offset.z, q.v.x, q.v.y, q.v.z, q.s, 0.3]);
            push_hit_off(&mut out, &ctx, moved_off);
        }

        out
    })
}

/// Push `hit, moved(3), point(3), normal(3)` (sphere layout: moved is a world center).
fn push_hit(out: &mut Vec<f32>, ctx: &CastContext, moved: Vec3) {
    if ctx.count > 0 {
        out.push(1.0);
        out.extend_from_slice(&[moved.x, moved.y, moved.z]);
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
        out.extend_from_slice(&[moved.x, moved.y, moved.z]);
        out.extend_from_slice(&[0.0; 6]);
    }
}

/// Push `hit, movedOff(3), point(3), normal(3)` (capsule/hull layout: an offset).
fn push_hit_off(out: &mut Vec<f32>, ctx: &CastContext, moved_off: Vec3) {
    if ctx.count > 0 {
        out.push(1.0);
        out.extend_from_slice(&[moved_off.x, moved_off.y, moved_off.z]);
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
        out.extend_from_slice(&[moved_off.x, moved_off.y, moved_off.z]);
        out.extend_from_slice(&[0.0; 6]);
    }
}
