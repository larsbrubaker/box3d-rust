//! Mesh demos — a 1:1 port of `box3d-cpp-reference/samples/sample_mesh.cpp`.
//!
//! The eight Mesh samples hosted here (Height Field has its own page):
//!
//! - Grid     (GridMesh, :25)          — `b3CreateGridMesh(20,20,1,0,true)` scale (2,2,2),
//!   drops a dynamic sphere/capsule/box/cylinder; radio shape picker + Scale X/Z.
//! - Big Box  (BigBoxMesh, :210)        — `b3CreateBoxMesh({0,-1,0},{50,1,50})`, same picker.
//! - Box      (BoxMesh, :384)           — ground box + 45°-rotated box-mesh ground, drops a box.
//! - Reflection (MeshReflection, :553)  — grid + `building.obj` + its mirrored copy + 20 humans.
//! - Viewer   (MeshViewer, :1021)       — voxel meshes with a per-level BVH AABB inspector.
//! - Creation Benchmark (:1304)         — times `b3CreateMesh` over the four voxel meshes.
//! - Voxel    (VoxelMesh, :1381)        — `collision_mesh_01.obj` terrain at a large-world offset.
//! - Hollow Box (HollowBox, :1498)      — `b3CreateHollowBoxMesh` with 14 zero-gravity bodies.
//!
//! Every scene keeps its own `World` + `VisBody` render list and reuses the shared
//! interaction helpers (mouse grab, spawn/delete, debug draw, engine-driven styles)
//! exactly like `shapes_demo` / `ragdoll_demo`. Static mesh grounds render as a baked
//! world-space wireframe (`mesh_ground_wireframe`); the OBJ-backed scenes (Viewer,
//! Creation Benchmark, Voxel) are reset from OBJ text the page fetches, mirroring the
//! Shapes Conveyor Mesh async pattern.
//!
//! Voxel sits 5000+ m from the origin, so its poses / picker rays / debug draw run in
//! a per-scene base frame (`base = origin`, `sub_pos` before the `f32` truncation),
//! the same large-world trick as `world_demo::far`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

mod scenes;
mod viewer;
mod voxel;

use crate::interact::{self, MouseGrab};
use crate::shell::ZERO_POS;
use crate::vis::VisBody;
use box3d_rust::body::{destroy_body, get_body_transform, make_body_id};
use box3d_rust::human::{
    human_set_joint_damping_ratio, human_set_joint_friction_torque, human_set_joint_spring_hertz,
    Human,
};
use box3d_rust::math_functions::{mul_transforms, sub_pos, Pos, Transform, Vec3};
use box3d_rust::mesh::MeshData;
use box3d_rust::types::default_world_def;
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// Scene identifiers (mirror the RegisterSample rows hosted by the `mesh` route).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum MeshScene {
    Grid,
    BigBox,
    Box,
    Reflection,
    Viewer,
    Benchmark,
    Voxel,
    HollowBox,
}

/// Drop-body shape picker (C `ShapeType`), shared by Grid / Big Box / Box.
pub(crate) const SHAPE_SPHERE: u32 = 0;
pub(crate) const SHAPE_CAPSULE: u32 = 1;
pub(crate) const SHAPE_BOX: u32 = 2;
pub(crate) const SHAPE_CYLINDER: u32 = 3;

thread_local! {
    static STATE: RefCell<Option<MeshState>> = const { RefCell::new(None) };
}

pub(crate) struct MeshState {
    pub world: World,
    /// Renderable bodies in the scene's base frame (dynamic drops, rendered hull
    /// grounds, human bones). Mesh grounds are drawn from `ground_edges` instead.
    pub bodies: Vec<VisBody>,
    /// Baked static mesh/height-field wireframe(s), already shifted into the base
    /// frame (`[x0,y0,z0, x1,y1,z1, ...]`). Empty for scenes with no mesh ground.
    pub ground_edges: Vec<f32>,
    pub grab: MouseGrab,
    /// Large-world base (Voxel = origin); `ZERO_POS` for near-origin scenes.
    pub base: Pos,
    pub scene: MeshScene,
    pub step_count: u32,

    // Grid / Big Box / Box picker + scale state.
    pub shape_type: u32,
    pub scale: Vec3,
    /// Render-list index of the current dropped body, or -1 (C `m_bodyId`).
    pub drop_index: i32,

    // Reflection humans (C `MeshReflection::m_humans`).
    pub humans: Vec<Human>,

    // Viewer / Creation Benchmark data.
    /// The Viewer's active collision mesh (for per-level BVH node draw + stats).
    pub viewer_mesh: Option<MeshData>,
    /// Loaded temp meshes for the Creation Benchmark (parsed once, rebuilt on run).
    pub bench_meshes: Vec<crate::obj_loader::TempMesh>,
    /// Scene stats readout (`mesh_stats`): meaning is scene-specific (see accessor).
    pub stats: Vec<f32>,

    /// Voxel's single dynamic hull, rendered as a live wireframe: `(body_index,
    /// flat local edge endpoints [ax,ay,az,bx,by,bz, ...])`. The hull is not a
    /// `vis` primitive kind, so it is drawn separately from `bodies`.
    pub voxel_hull: Option<(i32, Vec<f32>)>,
}

impl MeshState {
    pub(crate) fn base(world: World, scene: MeshScene) -> Self {
        MeshState {
            world,
            bodies: Vec::new(),
            ground_edges: Vec::new(),
            grab: MouseGrab::default(),
            base: ZERO_POS,
            scene,
            step_count: 0,
            shape_type: SHAPE_CYLINDER,
            scale: Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            drop_index: -1,
            humans: Vec::new(),
            viewer_mesh: None,
            bench_meshes: Vec::new(),
            stats: Vec::new(),
            voxel_hull: None,
        }
    }
}

pub(crate) fn with_state<R>(f: impl FnOnce(&mut MeshState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("mesh demo not initialized — call a mesh_reset_* first"))
    })
}

pub(crate) fn new_world() -> World {
    interact::reset_scene_scales();
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

pub(crate) fn install(state: MeshState) -> u32 {
    let count = state.bodies.len() as u32;
    STATE.with(|cell| {
        *cell.borrow_mut() = Some(state);
    });
    count
}

// ---------------------------------------------------------------------------
// Scene resets
// ---------------------------------------------------------------------------

/// Reset a parametric scene: 0 = Grid, 1 = Big Box, 2 = Box. `shape_type` picks the
/// dropped body; `scale_x` / `scale_z` are the ground-mesh Scale X/Z sliders (Scale
/// Y stays 1, matching the C `DrawControls`). This is the scene *reset* entry: the
/// mesh page (`demos/mesh.ts`) drives both scene entry and each Scale slider change
/// through it, so it rebuilds the whole scene from scratch. C re-scales the live
/// ground shape with `b3Shape_SetMesh` (now ported as `shape_set_mesh`) instead; a
/// full reset yields the collision-identical ground plus a fresh drop body.
#[wasm_bindgen]
pub fn mesh_reset(scene: u32, shape_type: u32, scale_x: f32, scale_z: f32) -> u32 {
    let state = match scene {
        1 => scenes::build_big_box(shape_type, scale_x, scale_z),
        2 => scenes::build_box(shape_type, scale_x, scale_z),
        _ => scenes::build_grid(shape_type, scale_x, scale_z),
    };
    install(state)
}

/// Reflection reset (C `MeshReflection`) with the mirrored building's scale radios
/// (default `{-1, 1, 1}`). C toggles the mirror live with `b3Shape_SetMesh` (now
/// ported as `shape_set_mesh`); the mesh page drives this radio through a full
/// scene reset instead (see [`mesh_reset`]).
#[wasm_bindgen]
pub fn mesh_reset_reflection(scale_x: f32, scale_y: f32, scale_z: f32) -> u32 {
    install(scenes::build_reflection(Vec3 {
        x: scale_x,
        y: scale_y,
        z: scale_z,
    }))
}

/// Hollow Box reset (C `HollowBox`) — 14 zero-gravity bodies inside a hollow box mesh.
#[wasm_bindgen]
pub fn mesh_reset_hollow_box() -> u32 {
    install(scenes::build_hollow_box())
}

/// Voxel reset (C `VoxelMesh`) from the fetched `collision_mesh_01.obj` text.
#[wasm_bindgen]
pub fn mesh_reset_voxel(obj_text: &str) -> u32 {
    install(voxel::build_voxel(obj_text))
}

/// Viewer reset (C `MeshViewer::LoadMesh`) from the selected voxel-mesh OBJ text
/// plus the BVH-build controls. Returns the render-body count.
#[wasm_bindgen]
pub fn mesh_reset_viewer(
    obj_text: &str,
    median_split: bool,
    concave_edges: bool,
    weld_vertices: bool,
    weld_tolerance_mm: f32,
) -> u32 {
    install(viewer::build_viewer(
        obj_text,
        median_split,
        concave_edges,
        weld_vertices,
        weld_tolerance_mm,
    ))
}

/// Creation Benchmark reset (C `MeshCreationBenchmark`) — parse the four voxel
/// meshes once so `mesh_benchmark_build` can time repeated `b3CreateMesh` calls.
#[wasm_bindgen]
pub fn mesh_reset_benchmark(obj1: &str, obj2: &str, obj3: &str, obj4: &str) -> u32 {
    install(viewer::build_benchmark(&[obj1, obj2, obj3, obj4]))
}

// ---------------------------------------------------------------------------
// Live controls
// ---------------------------------------------------------------------------

/// Grid / Big Box / Box shape picker (C `DrawControls` radios + `Spawn`): destroy
/// the current drop body and re-create it with the selected shape, leaving the
/// ground untouched (byte-identical to the C `Spawn()` flow).
#[wasm_bindgen]
pub fn mesh_set_shape(shape_type: u32) {
    with_state(|state| {
        state.shape_type = shape_type;
        scenes::spawn_drop(state);
    });
}

/// Reflection surface-material readout (C `MeshReflection::Render` `surface type`).
/// Returns the last `userMaterialId` reported by a contact, or 0.
#[wasm_bindgen]
pub fn mesh_reflection_material() -> u32 {
    // The C sample never updates m_userMaterialId (left 0); mirror that.
    0
}

// ---------------------------------------------------------------------------
// Step + render packing
// ---------------------------------------------------------------------------

#[wasm_bindgen]
pub fn mesh_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.wrapping_add(1);
        state.bodies.len() as u32
    })
}

/// Pack renderable poses in the base frame (16-float `vis` stride). Bodies with a
/// compound-child local transform (the drop cylinders) go through the `f32`
/// `mul_transforms` path — those scenes all use a zero base — while plain bodies
/// (Voxel's hull, at 5000+ m) get the full-precision `sub_pos` base shift.
#[wasm_bindgen]
pub fn mesh_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::with_capacity(state.bodies.len() * 16);
        for b in &state.bodies {
            let xf = get_body_transform(&state.world, b.body_index);
            let (px, py, pz, qx, qy, qz, qw) = if let Some(local) = b.local {
                let parent = Transform {
                    p: Vec3 {
                        x: xf.p.x as f32,
                        y: xf.p.y as f32,
                        z: xf.p.z as f32,
                    },
                    q: xf.q,
                };
                let w = mul_transforms(parent, local);
                let rel = sub_pos(
                    Pos {
                        x: w.p.x as _,
                        y: w.p.y as _,
                        z: w.p.z as _,
                    },
                    state.base,
                );
                (
                    rel.x as f32,
                    rel.y as f32,
                    rel.z as f32,
                    w.q.v.x,
                    w.q.v.y,
                    w.q.v.z,
                    w.q.s,
                )
            } else {
                let rel = sub_pos(xf.p, state.base);
                (
                    rel.x as f32,
                    rel.y as f32,
                    rel.z as f32,
                    xf.q.v.x,
                    xf.q.v.y,
                    xf.q.v.z,
                    xf.q.s,
                )
            };
            out.push(px);
            out.push(py);
            out.push(pz);
            out.push(qx);
            out.push(qy);
            out.push(qz);
            out.push(qw);
            out.extend_from_slice(&b.params);
            out.push(b.kind as f32);
            out.push(b.color as f32);
        }
        out
    })
}

#[wasm_bindgen]
pub fn mesh_styles() -> Vec<u32> {
    with_state(|state| crate::draw_data::shape_styles(&mut state.world, &state.bodies))
}

#[wasm_bindgen]
pub fn mesh_body_count() -> u32 {
    with_state(|state| state.bodies.len() as u32)
}

/// Baked static-mesh ground wireframe for the current scene (base-frame relative).
#[wasm_bindgen]
pub fn mesh_ground_wireframe() -> Vec<f32> {
    with_state(|state| state.ground_edges.clone())
}

/// Voxel's dynamic hull as a live base-frame wireframe (`[x0,y0,z0, x1,y1,z1, ...]`),
/// transformed through the body's current world transform. Empty for other scenes.
#[wasm_bindgen]
pub fn mesh_voxel_hull_wireframe() -> Vec<f32> {
    with_state(|state| {
        let Some((body_index, local)) = &state.voxel_hull else {
            return Vec::new();
        };
        let xf = get_body_transform(&state.world, *body_index);
        let t = Transform {
            p: Vec3 {
                x: xf.p.x as f32,
                y: xf.p.y as f32,
                z: xf.p.z as f32,
            },
            q: xf.q,
        };
        let mut out = Vec::with_capacity(local.len());
        let mut i = 0;
        while i + 3 <= local.len() {
            let v = box3d_rust::math_functions::transform_point(
                t,
                Vec3 {
                    x: local[i],
                    y: local[i + 1],
                    z: local[i + 2],
                },
            );
            // Shift into the base frame in Pos space, then truncate to f32.
            let rel = sub_pos(
                Pos {
                    x: v.x as _,
                    y: v.y as _,
                    z: v.z as _,
                },
                state.base,
            );
            out.push(rel.x as f32);
            out.push(rel.y as f32);
            out.push(rel.z as f32);
            i += 3;
        }
        out
    })
}

/// Scene stats readout (`mesh_stats`). Layout is scene-specific:
/// - Grid/Big Box/Box: `[triangle_count, byte_count]`
/// - Reflection: `[triangle_count]`
/// - Viewer: `[triangle_count, vertex_count, degenerate_count, height, node_area]`
///   (build time is measured on the page around the reset call)
/// - Voxel/Hollow Box: `[triangle_count]`
/// - Benchmark: filled by `mesh_benchmark_build` timing on the page side.
#[wasm_bindgen]
pub fn mesh_stats() -> Vec<f32> {
    with_state(|state| state.stats.clone())
}

// ---------------------------------------------------------------------------
// Viewer BVH inspector + Creation Benchmark
// ---------------------------------------------------------------------------

/// Viewer BVH height (max draw level for the slider). 0 when no mesh is loaded.
#[wasm_bindgen]
pub fn mesh_viewer_height() -> i32 {
    with_state(viewer::viewer_height)
}

/// Viewer BVH nodes at `level` (C `MeshViewer::DrawNodes`). For each node at the
/// target level: `[lx, ly, lz, ux, uy, uz, axis]` (axis 0/1/2 internal, 3 leaf).
#[wasm_bindgen]
pub fn mesh_viewer_nodes(level: i32) -> Vec<f32> {
    with_state(|state| viewer::viewer_nodes(state, level))
}

/// Build all four Creation-Benchmark meshes once (C `MeshCreationBenchmark::Step`
/// inner loop) and return the total triangle count. The page times this call with
/// `performance.now()` and keeps the minimum over its iterations, matching C's
/// `b3MinFloat` reduction over `b3GetMilliseconds`.
#[wasm_bindgen]
pub fn mesh_benchmark_build() -> i32 {
    with_state(viewer::benchmark_build)
}

// ---------------------------------------------------------------------------
// Reflection joint sliders (parity with the ragdoll page; C has none but the
// humans use the same builder, so expose them for symmetry / experimentation).
// ---------------------------------------------------------------------------

/// Apply joint friction / hertz / damping to every Reflection human (no-op for
/// other scenes). C `MeshReflection` uses fixed 5 / 1 / 0.7; this lets the page
/// keep the humans lively if desired.
#[wasm_bindgen]
pub fn mesh_set_joint_params(friction: f32, hertz: f32, damping: f32) {
    with_state(|state| {
        for human in &mut state.humans {
            if !human.is_spawned {
                continue;
            }
            human_set_joint_friction_torque(human, &mut state.world, friction);
            human_set_joint_spring_hertz(human, &mut state.world, hertz);
            human_set_joint_damping_ratio(human, &mut state.world, damping);
        }
    });
}

// ---------------------------------------------------------------------------
// Shared helpers used by the scene builders
// ---------------------------------------------------------------------------

/// Destroy the tracked drop body (if any), clearing it from the render list.
pub(crate) fn destroy_drop(state: &mut MeshState) {
    if state.drop_index < 0 {
        return;
    }
    let id = make_body_id(&state.world, state.drop_index);
    destroy_body(&mut state.world, id);
    let removed = state.drop_index;
    state.bodies.retain(|b| b.body_index != removed);
    state.drop_index = -1;
}

crate::demo_shell! {
    with_state: with_state,
    state: MeshState,
    world: world,
    bodies: bodies,
    grab: grab,
    base: |s| s.base,
    mouse_down: mesh_mouse_down,
    mouse_move: mesh_mouse_move,
    mouse_up: mesh_mouse_up,
    mouse_active: mesh_mouse_active,
    spawn_random: mesh_spawn_random = |state, spawned| {
        crate::interact::append_spawned_vis(&state.world, &mut state.bodies, spawned);
        crate::interact::spawn_ok_payload(spawned)
    },
    delete_at_ray: mesh_delete_at_ray = |state, index| {
        // Keep the drop-body handle honest if the picker body was the one deleted.
        if state.drop_index == index {
            state.drop_index = -1;
        }
    },
    counters: mesh_counters,
    debug_draw: mesh_debug_draw,
    debug_text: mesh_debug_text,
}

crate::demo_world_toggles! {
    with_state: with_state,
    world: world,
    set_enable_sleep: mesh_set_enable_sleep,
    set_enable_warm_starting: mesh_set_enable_warm_starting,
    set_enable_continuous: mesh_set_enable_continuous,
    set_recycle_distance: mesh_set_recycle_distance,
}
