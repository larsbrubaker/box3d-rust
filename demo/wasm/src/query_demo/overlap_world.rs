//! Collision / Overlap World — faithful port of `sample_collision.cpp`
//! `OverlapWorld` (line 1147). A 3x-body-type grid of shapes is probed by rows of
//! sphere / capsule / hull overlap queries whose z-offset follows a mouse drag.

use crate::vis::{hf_triangle_edges, mesh_triangle_edges, push_poses, VisBody};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::distance::ShapeProxy;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::height_field::create_wave;
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, transform_point, Pos, Transform, Vec3, PI, VEC3_AXIS_X, VEC3_AXIS_Z,
    VEC3_ZERO,
};
use box3d_rust::mesh::create_torus_mesh;
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    create_sphere_shape,
};
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def, BodyType};
use box3d_rust::world::{world_overlap_shape, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

struct Surface {
    body_id: BodyId,
    local_edges: Vec<f32>,
}

struct State {
    world: World,
    vis: Vec<VisBody>,
    surfaces: Vec<Surface>,
    offset: f32,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    STATE.with(|cell| {
        f(cell
            .borrow_mut()
            .as_mut()
            .expect("ow not initialized - call ow_reset first"))
    })
}

fn body_type_from_index(index: i32) -> BodyType {
    // C: b3BodyType(index) (line 1181). Enum: 0=static, 1=kinematic, 2=dynamic.
    match index {
        1 => BodyType::Kinematic,
        2 => BodyType::Dynamic,
        _ => BodyType::Static,
    }
}

#[wasm_bindgen]
pub fn ow_reset() {
    crate::interact::reset_scene_scales();
    let mut def = box3d_rust::types::default_world_def();
    def.gravity = VEC3_ZERO; // C: b3World_SetGravity(zero) (line 1169).
    let mut world = World::new(&def);

    let box_hull = make_box_hull(0.6, 0.6, 0.6);
    let mesh = create_torus_mesh(10, 12, 0.65, 0.35).expect("torus mesh");
    let height_field = create_wave(
        10,
        10,
        Vec3 {
            x: 0.2,
            y: 0.2,
            z: 0.2,
        },
        0.03,
        0.09,
        false,
    );

    let mut body_def = default_body_def();
    let shape_def = default_shape_def();
    let mut vis = Vec::new();
    let mut surfaces = Vec::new();

    for index in 0..3 {
        let ty = body_type_from_index(index);
        let y = 3.0 + 2.0 * index as f32;

        // Sphere r=0.8.
        body_def.type_ = ty;
        body_def.position = Pos { x: -6.0, y, z: 0.0 };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, 0.5 * PI);
        let b = create_body(&mut world, &body_def);
        create_sphere_shape(
            &mut world,
            b,
            &shape_def,
            &Sphere {
                center: VEC3_ZERO,
                radius: 0.8,
            },
        );
        vis.push(VisBody::sphere_body(b.index1 - 1, 0.8));

        // Capsule.
        body_def.type_ = ty;
        body_def.position = Pos { x: -3.0, y, z: 0.0 };
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
            radius: 0.5,
        };
        create_capsule_shape(&mut world, b, &shape_def, &capsule);
        vis.push(VisBody::capsule_body(b.index1 - 1, &capsule));

        // Hull (box).
        body_def.type_ = ty;
        body_def.position = Pos { x: 0.0, y, z: 0.0 };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.25 * PI);
        let b = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, b, &shape_def, &box_hull.base);
        vis.push(VisBody::box_body(b.index1 - 1, 0.6, 0.6, 0.6));

        // Mesh (torus), scale {-0.5,1.5,-1.0}.
        body_def.type_ = ty;
        body_def.position = Pos { x: 3.0, y, z: 0.0 };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, 0.5 * PI);
        let b = create_body(&mut world, &body_def);
        let scale = Vec3 {
            x: -0.5,
            y: 1.5,
            z: -1.0,
        };
        create_mesh_shape(&mut world, b, &shape_def, &mesh, scale);
        surfaces.push(Surface {
            body_id: b,
            local_edges: mesh_triangle_edges(&mesh, scale),
        });

        // Height field (static only), pos {5,2+2i,0} rot axisX -0.5PI.
        body_def.type_ = BodyType::Static;
        body_def.position = Pos {
            x: 5.0,
            y: 2.0 + 2.0 * index as f32,
            z: 0.0,
        };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, -0.5 * PI);
        let b = create_body(&mut world, &body_def);
        create_height_field_shape(&mut world, b, &shape_def, &height_field);
        surfaces.push(Surface {
            body_id: b,
            local_edges: hf_triangle_edges(&height_field, VEC3_ZERO),
        });
    }

    STATE.with(|cell| {
        *cell.borrow_mut() = Some(State {
            world,
            vis,
            surfaces,
            offset: 0.0,
        });
    });
}

#[wasm_bindgen]
pub fn ow_step(dt: f32, sub_steps: i32) {
    with_state(|state| state.world.step(dt, sub_steps));
}

#[wasm_bindgen]
pub fn ow_set_offset(offset: f32) {
    with_state(|state| state.offset = offset);
}

#[wasm_bindgen]
pub fn ow_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.vis, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn ow_surface_wireframe() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        for surf in &state.surfaces {
            let xf = get_body_transform(&state.world, surf.body_id.index1 - 1);
            let local = Transform {
                p: Vec3 {
                    x: xf.p.x as f32,
                    y: xf.p.y as f32,
                    z: xf.p.z as f32,
                },
                q: xf.q,
            };
            let e = &surf.local_edges;
            for i in (0..e.len()).step_by(6) {
                let a = transform_point(
                    local,
                    Vec3 {
                        x: e[i],
                        y: e[i + 1],
                        z: e[i + 2],
                    },
                );
                let b = transform_point(
                    local,
                    Vec3 {
                        x: e[i + 3],
                        y: e[i + 4],
                        z: e[i + 5],
                    },
                );
                out.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
            }
        }
        out
    })
}

fn overlaps_at(world: &World, proxy: &ShapeProxy) -> bool {
    let filter = default_query_filter();
    let mut hit = false;
    world_overlap_shape(world, Pos::default(), proxy, &filter, |_shape_id| {
        hit = true;
        false // terminate the query (C OverlapResultFcn, line 1228)
    });
    hit
}

/// The 5 sphere + 5 capsule + 5 hull probes with overlap flags.
/// Spheres: `cx,cy,cz, r, overlap` (5). Capsules: `c1(3), c2(3), r, overlap` (9).
/// Hulls: `cx,cy,cz, halfExt, overlap` (5). Fixed 5 each, in that order.
#[wasm_bindgen]
pub fn ow_overlaps() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        let off = state.offset;

        // Spheres (line 1237): {-6+3i, 3, -5+offset}, r=0.3.
        for i in 0..5 {
            let center = Vec3 {
                x: -6.0 + 3.0 * i as f32,
                y: 3.0,
                z: -5.0 + off,
            };
            let mut proxy = ShapeProxy {
                count: 1,
                radius: 0.3,
                ..Default::default()
            };
            proxy.points[0] = center;
            let overlap = overlaps_at(&state.world, &proxy);
            out.extend_from_slice(&[center.x, center.y, center.z, 0.3, overlap as i32 as f32]);
        }

        // Capsules (line 1249): offset {-6+3i, 5, -5+offset}, local ±0.2, r=0.2.
        for i in 0..5 {
            let o = Vec3 {
                x: -6.0 + 3.0 * i as f32,
                y: 5.0,
                z: -5.0 + off,
            };
            let c1 = Vec3 {
                x: o.x - 0.2,
                y: o.y - 0.2,
                z: o.z - 0.2,
            };
            let c2 = Vec3 {
                x: o.x + 0.2,
                y: o.y + 0.2,
                z: o.z + 0.2,
            };
            let mut proxy = ShapeProxy {
                count: 2,
                radius: 0.2,
                ..Default::default()
            };
            proxy.points[0] = c1;
            proxy.points[1] = c2;
            let overlap = overlaps_at(&state.world, &proxy);
            out.extend_from_slice(&[
                c1.x,
                c1.y,
                c1.z,
                c2.x,
                c2.y,
                c2.z,
                0.2,
                overlap as i32 as f32,
            ]);
        }

        // Hulls (line 1265): box 0.3 at {-6+3i, 7, -5+offset}, identity.
        for i in 0..5 {
            let center = Vec3 {
                x: -6.0 + 3.0 * i as f32,
                y: 7.0,
                z: -5.0 + off,
            };
            let box_hull = make_transformed_box_hull(
                0.3,
                0.3,
                0.3,
                Transform {
                    p: center,
                    q: box3d_rust::math_functions::QUAT_IDENTITY,
                },
            );
            let count = box_hull.base.vertex_count;
            let mut proxy = ShapeProxy {
                count,
                radius: 0.0,
                ..Default::default()
            };
            for j in 0..count as usize {
                proxy.points[j] = box_hull.box_points[j];
            }
            let overlap = overlaps_at(&state.world, &proxy);
            out.extend_from_slice(&[center.x, center.y, center.z, 0.3, overlap as i32 as f32]);
        }

        out
    })
}
