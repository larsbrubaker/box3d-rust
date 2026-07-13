//! Character samples — a 1:1 port of `box3d-cpp-reference/samples/sample_character.cpp`.
//!
//! Four scenes, mirroring the four `RegisterSample( "Character", ... )` rows:
//!
//! - **CapsulePlane** (`:145`) — drag a green capsule into a static box hull at
//!   `{0,1,1}`; `b3World_CollideMover` returns planes (yellow points/lines) and a
//!   **Solve** button pushes the capsule out with `b3SolvePlanes`.
//! - **MoverOverlap** (`:312`) — a static sphere / capsule / box at `x = -3 / 0 / 3`;
//!   drag the yellow mover capsule into each and watch the returned plane normals
//!   (lime = valid, red = degenerate) plus the cyan solved push-out pose. A HUD
//!   counter reports plane count and degenerate-normal count (must stay 0).
//! - **Mover** (`BasicMover`, `:314`) — the real level: `test_map01.obj` mesh,
//!   `stairs.obj`, a torus mesh, the `{7,2,-3}` ignore box, a spring-revolute door,
//!   the full 50×50 `b3CreateWave` height field, plus enemy/friendly capsules and a
//!   dynamic sphere. The C-exact `CharacterMover` algorithm drives a kinematic
//!   capsule; Third Person + Clip Velocity controls match the C `DrawControls`.
//! - **Rigid Body** (`RigidBodyCharacter`, `:1313`) — the large s&box-style dynamic
//!   character (dual feet-box + capsule, 4-phase trace step-up) over `test_map01`,
//!   `stairs`, `building`, two voxel meshes, the wave height field, and the ramp /
//!   platform / step-lip / wall / dynamic-prop obstacle course.
//!
//! Static mesh and height-field grounds render as a baked world-space wireframe
//! (`character_ground_wireframe`); the OBJ-backed scenes are reset from OBJ text the
//! page fetches, mirroring the Mesh Voxel / Shapes Conveyor async pattern.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

mod mover;
mod overlap;
mod rigid_body;

use crate::draw_data::STYLE_BODY_TYPE_SHIFT;
use crate::obj_loader::create_mesh_data_from_obj;
use crate::vis::{push_poses, VisBody, KIND_CAPSULE};
use box3d_rust::geometry::Capsule;
use box3d_rust::math_functions::{Pos, Transform, Vec3};
use box3d_rust::mesh::MeshData;
use box3d_rust::types::default_world_def;
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

pub(crate) use mover::MoverController;
pub(crate) use rigid_body::RigidbodyCharacter;

/// Box3D `b3HexColor` values used by the Character samples (draw.h enum).
pub(crate) mod colors {
    pub const GREEN: u32 = 0x008000;
    pub const RED: u32 = 0xFF0000;
    pub const BLUE: u32 = 0x0000FF;
    pub const YELLOW: u32 = 0xFFFF00;
    pub const CYAN: u32 = 0x00FFFF;
    pub const GRAY: u32 = 0x808080;
    pub const LIME_GREEN: u32 = 0x32CD32;
    pub const MEDIUM_VIOLET_RED: u32 = 0xC71585;
    pub const FLORAL_WHITE: u32 = 0xFFFAF0;
    pub const CORNFLOWER_BLUE: u32 = 0x6495ED;
    pub const OLIVE_DRAB: u32 = 0x6B8E23;
    pub const INDIAN_RED: u32 = 0xCD5C5C;
    pub const SLATE_GRAY: u32 = 0x708090;
    pub const DARK_SLATE_GRAY: u32 = 0x2F4F4F;
    pub const GOLD: u32 = 0xFFD700;
    pub const ORANGE: u32 = 0xFFA500;
    pub const PURPLE: u32 = 0x800080;
}

/// Scene discriminant, mirroring the four RegisterSample rows.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum SceneKind {
    CapsulePlane,
    MoverOverlap,
    Mover,
    RigidBody,
}

/// Camera-relative input the page feeds each step (C reads keyboard + camera).
#[derive(Clone, Copy)]
pub(crate) struct InputState {
    pub throttle_x: f32,
    pub throttle_y: f32,
    pub jump: bool,
    pub want_sprint: bool,
    pub forward: Vec3,
    pub right: Vec3,
}

impl Default for InputState {
    fn default() -> Self {
        InputState {
            throttle_x: 0.0,
            throttle_y: 0.0,
            jump: false,
            want_sprint: false,
            forward: Vec3 {
                x: 0.0,
                y: 0.0,
                z: -1.0,
            },
            right: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
        }
    }
}

/// A capsule rendered on top of the physics bodies (the query mover, the solved /
/// pushed-out ghost, the enemy/friendly avatars) — packed after `bodies` in the
/// pose / style streams. `style` is a fully packed `draw_data` style word.
pub(crate) struct RenderCapsule {
    pub transform: Transform,
    pub capsule: Capsule,
    pub style: u32,
}

/// Pack an `0xRRGGBB` color + body-type index into a `draw_data` style word so a
/// render-only capsule matches the engine-driven style stream.
pub(crate) fn capsule_style(rgb: u32, body_type_index: u32) -> u32 {
    (rgb & 0x00FF_FFFF) | (body_type_index << STYLE_BODY_TYPE_SHIFT)
}

/// Per-scene extra state. Only one variant is active at a time.
pub(crate) enum SceneState {
    /// CapsulePlane / MoverOverlap: a draggable query capsule + solved planes.
    Drag(overlap::DragScene),
    /// BasicMover kinematic controller.
    Mover(MoverController),
    /// s&box-style dynamic rigid-body character.
    RigidBody(RigidbodyCharacter),
}

pub(crate) struct CharacterState {
    pub world: World,
    /// Physics bodies rendered via `push_poses`.
    pub bodies: Vec<VisBody>,
    /// Baked static mesh / height-field wireframe (world space), or empty.
    pub ground_edges: Vec<f32>,
    /// Render-only capsules appended after `bodies` in the pose stream.
    pub render_capsules: Vec<RenderCapsule>,
    /// Debug line segments: `[x0,y0,z0, x1,y1,z1, color]` septuples.
    pub debug_segs: Vec<f32>,
    /// Debug points: `[x,y,z, color, size]` quintuples.
    pub debug_pts: Vec<f32>,
    /// Scalar status readouts (scene-specific; see `character_status`).
    pub status: Vec<f32>,
    pub input: InputState,
    pub clip_velocity: bool,
    pub third_person: bool,
    pub state: SceneState,
}

impl CharacterState {
    pub(crate) fn new(_scene: SceneKind, world: World, state: SceneState) -> Self {
        let third_person = matches!(state, SceneState::RigidBody(_));
        CharacterState {
            world,
            bodies: Vec::new(),
            ground_edges: Vec::new(),
            render_capsules: Vec::new(),
            debug_segs: Vec::new(),
            debug_pts: Vec::new(),
            status: Vec::new(),
            input: InputState::default(),
            clip_velocity: true,
            third_person,
            state,
        }
    }

    /// Push a debug line segment `origin → end` in the given `0xRRGGBB` color.
    pub(crate) fn draw_line(&mut self, a: Vec3, b: Vec3, color: u32) {
        self.debug_segs
            .extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z, color as f32]);
    }

    /// Push a debug point at `p` (world) with `size` px in the given color.
    pub(crate) fn draw_point(&mut self, p: Vec3, size: f32, color: u32) {
        self.debug_pts
            .extend_from_slice(&[p.x, p.y, p.z, color as f32, size]);
    }
}

thread_local! {
    static STATE: RefCell<Option<CharacterState>> = const { RefCell::new(None) };
}

pub(crate) fn with_state<R>(f: impl FnOnce(&mut CharacterState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("character not initialized — call a character_reset* first"))
    })
}

pub(crate) fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

/// The shared mover capsule (`CharacterMover::Initialize`, C `{ {0,-0.5,0}, {0,0.5,0}, 0.3 }`).
pub(crate) fn mover_capsule() -> Capsule {
    Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -0.5,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.5,
            z: 0.0,
        },
        radius: 0.3,
    }
}

/// Parse an OBJ into collision [`MeshData`] with the C `CreateMeshData(path, 1, false,
/// false, true, true)` flags used by every Character mesh (weld + identify edges).
pub(crate) fn load_level_mesh(obj_text: &str) -> Option<MeshData> {
    create_mesh_data_from_obj(obj_text, 1.0, false, false, true, true)
}

/// The 3-material set the C level mesh + height field use
/// (`{0.6,0,0}`, `{0.6,1,1}`, `{0.1,0,2}` — friction, restitution, rolling).
pub(crate) fn ground_materials() -> Vec<box3d_rust::geometry::SurfaceMaterial> {
    use box3d_rust::geometry::{default_surface_material, SurfaceMaterial};
    let base = default_surface_material();
    let m = |friction: f32, restitution: f32, rolling: f32| SurfaceMaterial {
        friction,
        restitution,
        rolling_resistance: rolling,
        ..base
    };
    vec![m(0.6, 0.0, 0.0), m(0.6, 1.0, 1.0), m(0.1, 0.0, 2.0)]
}

fn install(state: CharacterState) -> u32 {
    let n = state.bodies.len() as u32 + state.render_capsules.len() as u32;
    STATE.with(|cell| *cell.borrow_mut() = Some(state));
    n
}

// ---------------------------------------------------------------------------
// Scene resets
// ---------------------------------------------------------------------------

/// Reset an asset-free scene: 0 = CapsulePlane, 1 = MoverOverlap.
#[wasm_bindgen]
pub fn character_reset(scene: u32) -> u32 {
    let state = match scene {
        1 => overlap::build_mover_overlap(),
        _ => overlap::build_capsule_plane(),
    };
    install(state)
}

/// Reset the BasicMover scene from the fetched level meshes.
#[wasm_bindgen]
pub fn character_reset_mover(test_map_obj: &str, stairs_obj: &str) -> u32 {
    install(mover::build_mover(test_map_obj, stairs_obj))
}

/// Reset the Rigid Body character scene from its fetched meshes.
#[wasm_bindgen]
pub fn character_reset_rigid_body(
    test_map_obj: &str,
    stairs_obj: &str,
    building_obj: &str,
    voxel1_obj: &str,
    voxel2_obj: &str,
) -> u32 {
    install(rigid_body::build_rigid_body(
        test_map_obj,
        stairs_obj,
        building_obj,
        voxel1_obj,
        voxel2_obj,
    ))
}

// ---------------------------------------------------------------------------
// Live controls
// ---------------------------------------------------------------------------

/// Set WASD throttle, jump edge, sprint, and camera-relative axes (XZ plane).
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn character_set_input(
    throttle_x: f32,
    throttle_y: f32,
    jump: bool,
    sprint: bool,
    fwd_x: f32,
    fwd_z: f32,
    right_x: f32,
    right_z: f32,
) {
    with_state(|state| {
        state.input.throttle_x = throttle_x;
        state.input.throttle_y = throttle_y;
        // Held state each frame (C reads `IsKeyDown(KEY_SPACE)` fresh per step).
        state.input.jump = jump;
        state.input.want_sprint = sprint;
        state.input.forward = Vec3 {
            x: fwd_x,
            y: 0.0,
            z: fwd_z,
        };
        state.input.right = Vec3 {
            x: right_x,
            y: 0.0,
            z: right_z,
        };
    });
}

/// BasicMover "Clip Velocity" checkbox.
#[wasm_bindgen]
pub fn character_set_clip_velocity(clip: bool) {
    with_state(|state| state.clip_velocity = clip);
}

/// Third Person (T) toggle — the page follows the mover when true.
#[wasm_bindgen]
pub fn character_set_third_person(third_person: bool) {
    with_state(|state| state.third_person = third_person);
}

/// Drag scenes: set the query capsule world position (page pick-ray drag).
#[wasm_bindgen]
pub fn character_set_drag(x: f32, y: f32, z: f32) {
    with_state(|state| {
        if let SceneState::Drag(scene) = &mut state.state {
            scene.transform.p = Pos {
                x: x as _,
                y: y as _,
                z: z as _,
            };
        }
    });
}

/// CapsulePlane "Solve" button — push the capsule out of the stored planes.
#[wasm_bindgen]
pub fn character_solve() {
    with_state(|state| {
        if let SceneState::Drag(scene) = &mut state.state {
            scene.solve();
        }
    });
}

// ---------------------------------------------------------------------------
// Step + render packing
// ---------------------------------------------------------------------------

#[wasm_bindgen]
pub fn character_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.render_capsules.clear();
        state.debug_segs.clear();
        state.debug_pts.clear();
        state.status.clear();

        match state.state {
            SceneState::Drag(_) => overlap::step(state),
            SceneState::Mover(_) => mover::step(state, dt, sub_steps),
            SceneState::RigidBody(_) => rigid_body::step(state, dt, sub_steps),
        }

        (state.bodies.len() + state.render_capsules.len()) as u32
    })
}

/// Poses: physics bodies (16-float `vis` stride), then render-only capsules.
#[wasm_bindgen]
pub fn character_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        for rc in &state.render_capsules {
            let t = rc.transform;
            out.push(t.p.x as f32);
            out.push(t.p.y as f32);
            out.push(t.p.z as f32);
            out.push(t.q.v.x);
            out.push(t.q.v.y);
            out.push(t.q.v.z);
            out.push(t.q.s);
            out.push(rc.capsule.center1.x);
            out.push(rc.capsule.center1.y);
            out.push(rc.capsule.center1.z);
            out.push(rc.capsule.center2.x);
            out.push(rc.capsule.center2.y);
            out.push(rc.capsule.center2.z);
            out.push(rc.capsule.radius);
            out.push(KIND_CAPSULE as f32);
            out.push(0.0); // color slot unused; the style stream drives color
        }
        out
    })
}

/// Style words parallel to [`character_poses`]: engine-driven per body, then the
/// packed style of each render-only capsule.
#[wasm_bindgen]
pub fn character_styles() -> Vec<u32> {
    with_state(|state| {
        let mut out = crate::draw_data::shape_styles(&mut state.world, &state.bodies);
        for rc in &state.render_capsules {
            out.push(rc.style);
        }
        out
    })
}

/// Baked static mesh / height-field ground wireframe (world space).
#[wasm_bindgen]
pub fn character_ground_wireframe() -> Vec<f32> {
    with_state(|state| state.ground_edges.clone())
}

/// Debug line segments: `[x0,y0,z0, x1,y1,z1, color]` septuples (color is `0xRRGGBB`).
#[wasm_bindgen]
pub fn character_debug_segments() -> Vec<f32> {
    with_state(|state| state.debug_segs.clone())
}

/// Debug points: `[x,y,z, color, size]` quintuples.
#[wasm_bindgen]
pub fn character_debug_points() -> Vec<f32> {
    with_state(|state| state.debug_pts.clone())
}

/// Scene-specific scalar readouts (see each scene's `step`):
/// - Drag scenes: `[plane_count, degenerate_count]`
/// - Mover / Rigid Body: `[px,py,pz, vx,vy,vz, on_ground, sprint]`
#[wasm_bindgen]
pub fn character_status() -> Vec<f32> {
    with_state(|state| state.status.clone())
}

/// Follow target for the third-person camera: `[x,y,z]` of the character, or empty
/// for the drag scenes.
#[wasm_bindgen]
pub fn character_follow_target() -> Vec<f32> {
    with_state(|state| match &state.state {
        SceneState::Mover(m) => vec![
            m.transform.p.x as f32,
            m.transform.p.y as f32,
            m.transform.p.z as f32,
        ],
        SceneState::RigidBody(c) => {
            let p = c.position(&state.world);
            vec![p.x as f32, p.y as f32, p.z as f32]
        }
        SceneState::Drag(_) => Vec::new(),
    })
}

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
