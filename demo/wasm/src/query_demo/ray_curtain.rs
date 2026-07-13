//! Collision / Ray Curtain — faithful port of `sample_collision.cpp` `RayCurtain`
//! (line 14). Four rotating kinematic bodies (sphere / capsule / hull / torus mesh)
//! are swept by a curtain of downward rays whose z-offset animates back and forth.

use crate::vis::{mesh_triangle_edges, pos, push_poses, VisBody};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::make_box_hull;
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{transform_point, Pos, Transform, Vec3, VEC3_ONE};
use box3d_rust::mesh::MeshData;
use box3d_rust::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
};
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def, BodyType};
use box3d_rust::world::{world_cast_ray_closest, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

struct State {
    world: World,
    vis: Vec<VisBody>,
    mesh: MeshData,
    mesh_body: BodyId,
    offset: f32,
    abs_speed: f32,
    speed: f32,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    STATE.with(|cell| {
        f(cell
            .borrow_mut()
            .as_mut()
            .expect("rc not initialized - call rc_reset first"))
    })
}

#[wasm_bindgen]
pub fn rc_reset() {
    crate::interact::reset_scene_scales();
    let mut world = World::new(&box3d_rust::types::default_world_def());

    let box_hull = make_box_hull(0.6, 0.6, 0.6);
    let mesh = box3d_rust::mesh::create_torus_mesh(10, 12, 0.65, 0.35).expect("torus mesh");

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Kinematic;
    let shape_def = default_shape_def();
    let ang = Vec3 {
        x: 0.8,
        y: 0.4,
        z: 0.8,
    };

    let mut vis = Vec::new();

    // Sphere at {-6,3,0}.
    body_def.position = Pos {
        x: -6.0,
        y: 3.0,
        z: 0.0,
    };
    body_def.angular_velocity = ang;
    let sphere_body = create_body(&mut world, &body_def);
    let sphere = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.9,
    };
    create_sphere_shape(&mut world, sphere_body, &shape_def, &sphere);
    vis.push(VisBody::sphere_body(sphere_body.index1 - 1, 0.9));

    // Capsule at {-2,3,0}.
    body_def.position = Pos {
        x: -2.0,
        y: 3.0,
        z: 0.0,
    };
    let capsule_body = create_body(&mut world, &body_def);
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
        radius: 0.8,
    };
    create_capsule_shape(&mut world, capsule_body, &shape_def, &capsule);
    vis.push(VisBody::capsule_body(capsule_body.index1 - 1, &capsule));

    // Hull (box) at {2,3,0}.
    body_def.position = Pos {
        x: 2.0,
        y: 3.0,
        z: 0.0,
    };
    let hull_body = create_body(&mut world, &body_def);
    create_hull_shape(&mut world, hull_body, &shape_def, &box_hull.base);
    vis.push(VisBody::box_body(hull_body.index1 - 1, 0.6, 0.6, 0.6));

    // Torus mesh at {6,3,0}.
    body_def.position = Pos {
        x: 6.0,
        y: 3.0,
        z: 0.0,
    };
    let mesh_body = create_body(&mut world, &body_def);
    create_mesh_shape(&mut world, mesh_body, &shape_def, &mesh, VEC3_ONE);

    STATE.with(|cell| {
        *cell.borrow_mut() = Some(State {
            world,
            vis,
            mesh,
            mesh_body,
            offset: 2.0,
            abs_speed: 0.015,
            speed: -0.015,
        });
    });
}

#[wasm_bindgen]
pub fn rc_step(dt: f32, sub_steps: i32) {
    with_state(|state| {
        state.world.step(dt, sub_steps);
        // C offset bounce (line 97): reverse at ±2, advance by speed each frame.
        if state.offset > 2.0 {
            state.speed = -state.abs_speed;
        } else if state.offset < -2.0 {
            state.speed = state.abs_speed;
        }
        state.offset += state.speed;
    });
}

#[wasm_bindgen]
pub fn rc_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.vis, &mut out);
        out
    })
}

/// Torus mesh edges transformed to world (the mesh body has no VisBody).
#[wasm_bindgen]
pub fn rc_surface_wireframe() -> Vec<f32> {
    with_state(|state| {
        let edges = mesh_triangle_edges(&state.mesh, VEC3_ONE);
        let xf = get_body_transform(&state.world, state.mesh_body.index1 - 1);
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
    })
}

/// The curtain of rays for the current z-offset, with closest-hit results.
/// Per ray (13 floats): `ox,oy,oz, ex,ey,ez, hit, px,py,pz, nx,ny,nz`.
#[wasm_bindgen]
pub fn rc_rays() -> Vec<f32> {
    with_state(|state| {
        let filter = default_query_filter();
        let mut out = Vec::new();
        // C: for x = -8 .. 8 step 0.1 (line 80).
        let mut x = -8.0f32;
        while x <= 8.0 {
            let origin = pos(x, 8.0, state.offset);
            let end = pos(x, 0.0, state.offset);
            // rayEnd - rayOrigin is {0,-8,0} for every ray.
            let translation = Vec3 {
                x: 0.0,
                y: -8.0,
                z: 0.0,
            };
            let result = world_cast_ray_closest(&state.world, origin, translation, &filter);
            out.extend_from_slice(&[
                origin.x as f32,
                origin.y as f32,
                origin.z as f32,
                end.x as f32,
                end.y as f32,
                end.z as f32,
            ]);
            if result.hit {
                out.extend_from_slice(&[
                    1.0,
                    result.point.x as f32,
                    result.point.y as f32,
                    result.point.z as f32,
                    result.normal.x,
                    result.normal.y,
                    result.normal.z,
                ]);
            } else {
                out.extend_from_slice(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
            }
            x += 0.1;
        }
        out
    })
}
