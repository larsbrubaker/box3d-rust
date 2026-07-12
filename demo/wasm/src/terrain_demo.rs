//! Terrain settle demos — bodies falling onto a grid mesh or height-field wave
//! (mirrors sample_mesh Grid / Height Field with dynamics enabled).

use crate::vis::{pos, push_poses, sphere, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::geometry::Capsule;
use box3d_rust::height_field::{
    create_wave, get_height_field_triangle, get_height_field_triangle_count, HeightFieldData,
};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::Vec3;
use box3d_rust::mesh::{create_grid_mesh, get_mesh_triangles, get_mesh_vertices, MeshData};
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    create_sphere_shape,
};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<TerrainState>> = const { RefCell::new(None) };
}

enum TerrainKind {
    Mesh { mesh: MeshData, scale: Vec3 },
    HeightField { hf: HeightFieldData, origin: Vec3 },
}

struct TerrainState {
    world: World,
    bodies: Vec<VisBody>,
    terrain: TerrainKind,
}

fn with_state<R>(f: impl FnOnce(&mut TerrainState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("terrain not initialized — call terrain_reset first"))
    })
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

fn push_box(state: &mut TerrainState, x: f32, y: f32, z: f32, hx: f32, hy: f32, hz: f32) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(x, y, z);
    let body = create_body(&mut state.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.base_material.friction = 0.4;
    let hull = make_box_hull(hx, hy, hz);
    create_hull_shape(&mut state.world, body, &shape_def, &hull.base);
    state
        .bodies
        .push(VisBody::box_body(body.index1 - 1, hx, hy, hz));
}

fn push_sphere(state: &mut TerrainState, x: f32, y: f32, z: f32, radius: f32) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(x, y, z);
    let body = create_body(&mut state.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.base_material.rolling_resistance = 0.05;
    let sph = sphere(radius);
    create_sphere_shape(&mut state.world, body, &shape_def, &sph);
    state
        .bodies
        .push(VisBody::sphere_body(body.index1 - 1, radius));
}

fn push_capsule(state: &mut TerrainState, x: f32, y: f32, z: f32) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(x, y, z);
    let body = create_body(&mut state.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    shape_def.base_material.rolling_resistance = 0.05;
    let capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.4,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.0,
            z: -0.4,
        },
        radius: 0.15,
    };
    create_capsule_shape(&mut state.world, body, &shape_def, &capsule);
    state
        .bodies
        .push(VisBody::capsule_body(body.index1 - 1, &capsule));
}

fn spawn_dynamics(state: &mut TerrainState) {
    push_box(state, 0.1, 3.0, -0.1, 0.4, 0.4, 0.4);
    push_sphere(state, -1.2, 4.0, 0.5, 0.35);
    push_sphere(state, 1.5, 5.0, -0.8, 0.45);
    push_capsule(state, 0.5, 6.0, 1.0);
    push_box(state, -0.8, 7.0, -1.2, 0.3, 0.3, 0.3);
    push_sphere(state, 0.0, 8.0, 0.0, 0.5);
}

fn build_mesh_scene() -> TerrainState {
    let mut world = new_world();
    let mesh = create_grid_mesh(16, 16, 1.0, 0, true).expect("grid mesh");
    let scale = Vec3 {
        x: 1.5,
        y: 1.5,
        z: 1.5,
    };

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = pos(0.0, 0.0, 0.0);
    let ground = create_body(&mut world, &ground_def);
    let shape_def = default_shape_def();
    create_mesh_shape(&mut world, ground, &shape_def, &mesh, scale);

    let mut state = TerrainState {
        world,
        bodies: Vec::new(),
        terrain: TerrainKind::Mesh { mesh, scale },
    };
    spawn_dynamics(&mut state);
    state
}

fn build_height_field_scene() -> TerrainState {
    let mut world = new_world();
    let rows = 21;
    let cols = 21;
    let scale = Vec3 {
        x: 1.0,
        y: 1.5,
        z: 1.0,
    };
    let hf = create_wave(rows, cols, scale, 0.1, 0.03333, false);
    let origin = Vec3 {
        x: -0.5 * scale.x * (cols - 1) as f32,
        y: 0.0,
        z: -0.5 * scale.z * (rows - 1) as f32,
    };

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = pos(origin.x, origin.y, origin.z);
    let ground = create_body(&mut world, &ground_def);
    let shape_def = default_shape_def();
    create_height_field_shape(&mut world, ground, &shape_def, &hf);

    let mut state = TerrainState {
        world,
        bodies: Vec::new(),
        terrain: TerrainKind::HeightField { hf, origin },
    };
    spawn_dynamics(&mut state);
    state
}

/// Reset terrain settle scene. `mode`: 0 = grid mesh, 1 = height field.
#[wasm_bindgen]
pub fn terrain_reset(mode: u32) -> u32 {
    STATE.with(|cell| {
        let state = if mode == 0 {
            build_mesh_scene()
        } else {
            build_height_field_scene()
        };
        let n = state.bodies.len() as u32;
        *cell.borrow_mut() = Some(state);
        n
    })
}

#[wasm_bindgen]
pub fn terrain_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.world.step(dt, sub_steps);
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn terrain_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

/// Terrain triangle wireframe edges: interleaved [x0,y0,z0, x1,y1,z1, ...].
#[wasm_bindgen]
pub fn terrain_wireframe() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        match &state.terrain {
            TerrainKind::Mesh { mesh, scale } => {
                let verts = get_mesh_vertices(mesh);
                let tris = get_mesh_triangles(mesh);
                for t in tris {
                    let vs = [
                        verts[t.index1 as usize],
                        verts[t.index2 as usize],
                        verts[t.index3 as usize],
                    ];
                    for e in 0..3 {
                        let a = vs[e];
                        let b = vs[(e + 1) % 3];
                        out.push(a.x * scale.x);
                        out.push(a.y * scale.y);
                        out.push(a.z * scale.z);
                        out.push(b.x * scale.x);
                        out.push(b.y * scale.y);
                        out.push(b.z * scale.z);
                    }
                }
            }
            TerrainKind::HeightField { hf, origin } => {
                let count = get_height_field_triangle_count(hf);
                for i in 0..count {
                    let tri = get_height_field_triangle(hf, i);
                    let verts = tri.vertices;
                    for e in 0..3 {
                        let a = verts[e];
                        let b = verts[(e + 1) % 3];
                        out.push(a.x + origin.x);
                        out.push(a.y + origin.y);
                        out.push(a.z + origin.z);
                        out.push(b.x + origin.x);
                        out.push(b.y + origin.y);
                        out.push(b.z + origin.z);
                    }
                }
            }
        }
        out
    })
}
