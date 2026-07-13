//! Recording-replay viewer bindings. Port of `samples/sample_replay.cpp` (the
//! `.b3rec` keyframe player) for the browser demo.
//!
//! The C viewer (`ReplayViewer`) drives `b3RecPlayer` one recorded step at a time
//! and draws the replayed world through the same debug-draw path the live samples
//! use. These bindings expose the ported [`box3d_rust::recording::RecPlayer`] to
//! JavaScript: load a `.b3rec` byte buffer, read frame/rate/bounds, drive the
//! transport (step / seek / restart), and extract per-frame body transforms plus
//! per-shape geometry and engine-resolved colors for rendering.
//!
//! # What is faithfully ported
//! - Transport + scrubber (`replay_step` / `replay_seek` / `replay_restart`,
//!   `replay_frame` / `replay_frame_count`) driving the real `RecPlayer`.
//! - Playback rendering: every recorded body's transform each frame, and every
//!   recorded shape's geometry (sphere / capsule parametric; hull / mesh /
//!   height-field as triangle lists) colored by the engine's own
//!   `shape_debug_color` state machine — the same awake / sleeping / static /
//!   sensor rules the C debug adapter reads (see [`crate::draw_data`]).
//! - The per-step `StateHash` divergence check the player performs is surfaced via
//!   [`replay_has_diverged`] / [`replay_diverge_frame`], mirroring the C viewer's
//!   `****DIVERGED****` readout.
//!
//! # Disclosed partials (the C viewer's heavier UI surfaces)
//! - Compound shapes are not individually tessellated here (their children carry
//!   local transforms and per-child geometry); a recorded compound body is skipped
//!   in both the geometry and style streams. Every other shape type renders.
//! - **Shipped:** the Outline scene tree ([`replay_outline`]) and the per-selection
//!   Inspector ([`replay_body_detail`]) — bodies grouped by creation ordinal with
//!   their shapes, selectable from the tree or the 3D view, showing the selected
//!   body's live transform / velocity / mass / awake-enabled-bullet state read from
//!   the current frame (C `DrawOutlineTree` / `DrawBodyDetail`). Selection is at body
//!   granularity: a shape row selects its owning body (C's finer shape-only detail
//!   pane is folded into the body detail, disclosed).
//! - **Disclosed-skipped:** the whole-recording query **search index** (C
//!   `BuildQueryIndex` replays the entire recording once to index every recorded
//!   spatial query — a heavy scan our player *could* drive via `get_frame_query*`,
//!   scoped out as too large for the browser demo) and the **keyframe-policy popup**.
//!   The popup only configures `b3RecPlayer_SetKeyframePolicy` (a backward-seek
//!   keyframe-ring budget); our [`replay_seek`] implements backward seeks by
//!   restart-and-replay-forward and never consumes that ring, so a policy control
//!   would configure a knob with no observable effect — skipped rather than faked.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use std::cell::RefCell;

use box3d_rust::body::{
    body_get_angular_velocity, body_get_gravity_scale, body_get_joint_count,
    body_get_linear_velocity, body_get_mass, body_get_name, body_get_shape_count, body_get_type,
    body_is_awake, body_is_bullet, body_is_enabled, body_is_valid, get_body_transform,
};
use box3d_rust::core::NULL_INDEX;
use box3d_rust::height_field::{
    get_height_field_triangle, get_height_field_triangle_count, HeightFieldData,
};
use box3d_rust::math_functions::Vec3;
use box3d_rust::math_functions::{get_axis_angle, length};
use box3d_rust::mesh::{get_mesh_triangles, get_mesh_vertices, MeshData};
use box3d_rust::recording::RecPlayer;
use box3d_rust::shape::ShapeGeometry;
use box3d_rust::types::BodyType;
use wasm_bindgen::prelude::*;

use crate::vis::hull_triangles;

// Geometry-stream shape kinds handed to the JS renderer. Hulls, meshes, and
// height fields all arrive as triangle lists (kind 0); spheres and capsules stay
// parametric so Three.js can build smooth geometry for them.
const RKIND_TRI: f32 = 0.0;
const RKIND_SPHERE: f32 = 1.0;
const RKIND_CAPSULE: f32 = 2.0;

thread_local! {
    /// The single active player. Replaced by [`replay_load`], dropped by
    /// [`replay_unload`] (the `Drop` impl restores the global length scale the
    /// recording temporarily set on load).
    static PLAYER: RefCell<Option<Box<RecPlayer>>> = const { RefCell::new(None) };
}

fn with_player<R>(f: impl FnOnce(&mut RecPlayer) -> R) -> Option<R> {
    PLAYER.with(|cell| cell.borrow_mut().as_mut().map(|p| f(p)))
}

// --------------------------------------------------------------------------
// Load / transport
// --------------------------------------------------------------------------

/// Load a `.b3rec` byte buffer into a fresh player, adopting its replayed world.
/// Returns false on a missing / corrupt / version-mismatched recording (the
/// player logs the reason). The previous player is dropped first so its `Drop`
/// restores the base length scale before the new recording applies its own.
#[wasm_bindgen]
pub fn replay_load(data: &[u8]) -> bool {
    PLAYER.with(|cell| *cell.borrow_mut() = None);
    match RecPlayer::create(data, 1) {
        Some(player) => {
            PLAYER.with(|cell| *cell.borrow_mut() = Some(player));
            true
        }
        None => false,
    }
}

/// Drop the active player (restoring the global length scale). Called by the page
/// on teardown so navigating away never leaks a tilted length unit.
#[wasm_bindgen]
pub fn replay_unload() {
    PLAYER.with(|cell| *cell.borrow_mut() = None);
}

#[wasm_bindgen]
pub fn replay_loaded() -> bool {
    PLAYER.with(|cell| cell.borrow().is_some())
}

#[wasm_bindgen]
pub fn replay_frame_count() -> i32 {
    with_player(|p| p.get_frame_count()).unwrap_or(0)
}

#[wasm_bindgen]
pub fn replay_frame() -> i32 {
    with_player(|p| p.get_frame()).unwrap_or(0)
}

/// Recorded fixed timestep (`b3RecPlayerInfo::timeStep`); `1/timeStep` is the
/// recording's frame rate.
#[wasm_bindgen]
pub fn replay_time_step() -> f32 {
    with_player(|p| p.get_info().time_step).unwrap_or(0.0)
}

#[wasm_bindgen]
pub fn replay_sub_step_count() -> i32 {
    with_player(|p| p.get_info().sub_step_count).unwrap_or(0)
}

/// Recorded world bounds as `[lx, ly, lz, ux, uy, uz]`, for camera framing. Empty
/// when no recording is loaded or the recording carries no bounds record.
#[wasm_bindgen]
pub fn replay_bounds() -> Vec<f32> {
    with_player(|p| {
        let b = p.get_info().bounds;
        vec![
            b.lower_bound.x,
            b.lower_bound.y,
            b.lower_bound.z,
            b.upper_bound.x,
            b.upper_bound.y,
            b.upper_bound.z,
        ]
    })
    .unwrap_or_default()
}

#[wasm_bindgen]
pub fn replay_body_count() -> i32 {
    with_player(|p| p.get_body_count()).unwrap_or(0)
}

#[wasm_bindgen]
pub fn replay_is_at_end() -> bool {
    with_player(|p| p.is_at_end()).unwrap_or(false)
}

#[wasm_bindgen]
pub fn replay_has_diverged() -> bool {
    with_player(|p| p.has_diverged()).unwrap_or(false)
}

/// Frame at which the replay first diverged from the recorded `StateHash`, or -1
/// when it has not (mirrors the C scrubber's divergence marker).
#[wasm_bindgen]
pub fn replay_diverge_frame() -> i32 {
    with_player(|p| p.get_diverge_frame()).unwrap_or(-1)
}

/// Advance one recorded step (no-op at end). `b3RecPlayer_StepFrame`.
#[wasm_bindgen]
pub fn replay_step() {
    with_player(|p| {
        if !p.is_at_end() {
            p.step_frame();
        }
    });
}

/// Seek to an absolute frame, clamped to `[0, frameCount]`. Backward seeks restart
/// and replay forward, exactly like `b3RecPlayer_SeekFrame`.
#[wasm_bindgen]
pub fn replay_seek(frame: i32) {
    with_player(|p| {
        let target = frame.clamp(0, p.get_frame_count());
        p.seek_frame(target);
    });
}

/// Rewind to frame 0, re-deserializing the seed snapshot (`b3RecPlayer_Restart`).
#[wasm_bindgen]
pub fn replay_restart() {
    with_player(|p| p.restart());
}

// --------------------------------------------------------------------------
// Geometry / transform / style streams
// --------------------------------------------------------------------------

/// Static per-shape geometry for the current world topology, in a self-describing
/// stream. Walk it record by record: each record starts with `[bodyOrdinal, kind]`
/// then a kind-specific payload —
/// - `kind 1` (sphere): `cx, cy, cz, radius` (body-local center)
/// - `kind 2` (capsule): `c1x,c1y,c1z, c2x,c2y,c2z, radius` (body-local centers)
/// - `kind 0` (triangles): `floatCount`, then `floatCount` body-local triangle
///   vertex floats (a multiple of 9). Hull, mesh, and height-field shapes.
///
/// Shapes are emitted in body-creation-ordinal order, head shape first, so the
/// stream is parallel to [`replay_shape_styles`]. The JS side rebuilds its meshes
/// whenever the body count changes (create/destroy); within a fixed topology the
/// stream is stable and only the transforms/styles update per frame.
#[wasm_bindgen]
pub fn replay_scene_geometry() -> Vec<f32> {
    let mut out = Vec::new();
    with_player(|p| {
        let count = p.get_body_count();
        for ord in 0..count {
            let body_id = p.get_body_id(ord);
            if body_id.index1 <= 0 || !body_is_valid(p.world(), body_id) {
                continue;
            }
            let world = p.world();
            let body_index = (body_id.index1 - 1) as usize;
            let mut sid = world.bodies[body_index].head_shape_id;
            while sid != NULL_INDEX {
                let shape = &world.shapes[sid as usize];
                match &shape.geometry {
                    ShapeGeometry::Sphere(s) => {
                        out.push(ord as f32);
                        out.push(RKIND_SPHERE);
                        out.push(s.center.x);
                        out.push(s.center.y);
                        out.push(s.center.z);
                        out.push(s.radius);
                    }
                    ShapeGeometry::Capsule(c) => {
                        out.push(ord as f32);
                        out.push(RKIND_CAPSULE);
                        out.push(c.center1.x);
                        out.push(c.center1.y);
                        out.push(c.center1.z);
                        out.push(c.center2.x);
                        out.push(c.center2.y);
                        out.push(c.center2.z);
                        out.push(c.radius);
                    }
                    ShapeGeometry::Hull(h) => push_tri_record(&mut out, ord, hull_triangles(h)),
                    ShapeGeometry::Mesh { data, scale } => {
                        push_tri_record(&mut out, ord, mesh_triangles(data, *scale))
                    }
                    ShapeGeometry::HeightField(hf) => {
                        push_tri_record(&mut out, ord, height_field_triangles(hf))
                    }
                    // Disclosed partial: compound children are not tessellated here.
                    ShapeGeometry::Compound(_) => {}
                }
                sid = world.shapes[sid as usize].next_shape_id;
            }
        }
    });
    out
}

/// Per-body transform for the current frame, one 8-float record per creation
/// ordinal: `[valid, px, py, pz, qx, qy, qz, qw]`. `valid` is 0 for an ordinal
/// whose body does not exist at this frame (destroyed or not yet spawned), 1
/// otherwise. Indexed by the `bodyOrdinal` carried in [`replay_scene_geometry`].
#[wasm_bindgen]
pub fn replay_body_transforms() -> Vec<f32> {
    let mut out = Vec::new();
    with_player(|p| {
        let count = p.get_body_count();
        out.reserve(count as usize * 8);
        for ord in 0..count {
            let body_id = p.get_body_id(ord);
            if body_id.index1 <= 0 || !body_is_valid(p.world(), body_id) {
                out.extend_from_slice(&[0.0; 8]);
                continue;
            }
            let xf = get_body_transform(p.world(), body_id.index1 - 1);
            out.push(1.0);
            out.push(xf.p.x as f32);
            out.push(xf.p.y as f32);
            out.push(xf.p.z as f32);
            out.push(xf.q.v.x);
            out.push(xf.q.v.y);
            out.push(xf.q.v.z);
            out.push(xf.q.s);
        }
    });
    out
}

/// Engine-resolved packed style word per shape, parallel to
/// [`replay_scene_geometry`]. Same bit layout as [`crate::draw_data`] /
/// `applyShapeStyle`: `0xRRGGBB` in bits 0..24, material preset in 24..27, body
/// type in 27..29. Produced by running the real `world_draw` color pass over the
/// replayed world, so sleep/wake/sensor recoloring matches the live samples.
#[wasm_bindgen]
pub fn replay_shape_styles() -> Vec<u32> {
    let mut out = Vec::new();
    with_player(|p| {
        // Snapshot the body ids before the world is mutably borrowed for the draw.
        let count = p.get_body_count();
        let body_ids: Vec<_> = (0..count).map(|ord| p.get_body_id(ord)).collect();
        // Reuse the shared engine-color capture + style packing (`crate::draw_data`)
        // so the replay world resolves colors through the exact same `world_draw`
        // pass and bit layout as the live samples — no forked capture logic here.
        crate::draw_data::with_engine_colors(p.world_mut(), |world, colors| {
            for body_id in body_ids {
                if body_id.index1 <= 0 || !body_is_valid(world, body_id) {
                    continue;
                }
                let body_index = (body_id.index1 - 1) as usize;
                let body_type = world.bodies[body_index].type_;
                let mut sid = world.bodies[body_index].head_shape_id;
                while sid != NULL_INDEX {
                    let next = world.shapes[sid as usize].next_shape_id;
                    // Skip compound shapes so the style stream stays parallel to the
                    // geometry stream (which also skips them).
                    if !matches!(
                        world.shapes[sid as usize].geometry,
                        ShapeGeometry::Compound(_)
                    ) {
                        let engine = colors.get(&((sid + 1) as u64)).copied().unwrap_or(0);
                        // Replay never uses the transparent-dynamic view toggle.
                        out.push(crate::draw_data::pack_style(engine, body_type, false));
                    }
                    sid = next;
                }
            }
        });
    });
    out
}

// --------------------------------------------------------------------------
// Outline scene tree + selection inspector (C DrawOutlineTree / DrawBodyDetail)
// --------------------------------------------------------------------------

/// Body-type display name (C `ReplayBodyTypeName`).
fn body_type_name(t: BodyType) -> &'static str {
    match t {
        BodyType::Static => "static",
        BodyType::Kinematic => "kinematic",
        BodyType::Dynamic => "dynamic",
    }
}

/// Shape-type display name (C `ReplayShapeTypeName`).
fn shape_type_name(g: &ShapeGeometry) -> &'static str {
    match g {
        ShapeGeometry::Sphere(_) => "sphere",
        ShapeGeometry::Capsule(_) => "capsule",
        ShapeGeometry::Hull(_) => "hull",
        ShapeGeometry::Mesh { .. } => "mesh",
        ShapeGeometry::HeightField(_) => "height field",
        ShapeGeometry::Compound(_) => "compound",
    }
}

/// Minimal JSON string escape (recording names are plain, but stay safe).
fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

/// The recorded scene tree as JSON, in body-creation-ordinal order (C
/// `DrawOutlineTree`). One entry per body currently valid at this frame:
/// `{ "ord": n, "name": "...", "type": "dynamic", "shapes": ["hull", ...] }`.
/// Shapes are listed head-first, matching [`replay_scene_geometry`]. Rebuilt by the
/// page whenever the body count changes (create/destroy), like the geometry stream.
#[wasm_bindgen]
pub fn replay_outline() -> String {
    let mut out = String::from("[");
    with_player(|p| {
        let world = p.world();
        let count = p.get_body_count();
        let mut first = true;
        for ord in 0..count {
            let body_id = p.get_body_id(ord);
            if body_id.index1 <= 0 || !body_is_valid(world, body_id) {
                continue;
            }
            let body_index = (body_id.index1 - 1) as usize;
            let name = body_get_name(world, body_id);
            let type_name = body_type_name(body_get_type(world, body_id));

            if !first {
                out.push(',');
            }
            first = false;
            out.push_str(&format!(
                "{{\"ord\":{},\"name\":\"{}\",\"type\":\"{}\",\"shapes\":[",
                ord,
                json_escape(name),
                type_name,
            ));
            let mut sid = world.bodies[body_index].head_shape_id;
            let mut first_shape = true;
            while sid != NULL_INDEX {
                if !first_shape {
                    out.push(',');
                }
                first_shape = false;
                out.push_str(&format!(
                    "\"{}\"",
                    shape_type_name(&world.shapes[sid as usize].geometry)
                ));
                sid = world.shapes[sid as usize].next_shape_id;
            }
            out.push_str("]}");
        }
    });
    out.push(']');
    out
}

/// Selection inspector for the body at creation ordinal `ord`, read from the current
/// frame (C `DrawBodyDetail`). Returns JSON `{ "present": true, ... }` with the body's
/// live transform / velocity / mass / state, or `{ "present": false }` when the body
/// does not exist at this frame (destroyed or not yet spawned) — matching the C
/// "Not present at this frame." readout. Selection is stored as an ordinal so it
/// survives the backward-seek world rebuild, exactly like the C viewer.
#[wasm_bindgen]
pub fn replay_body_detail(ord: i32) -> String {
    with_player(|p| {
        let world = p.world();
        let body_id = p.get_body_id(ord);
        if body_id.index1 <= 0 || !body_is_valid(world, body_id) {
            return String::from("{\"present\":false}");
        }
        let xf = get_body_transform(world, body_id.index1 - 1);
        let v = body_get_linear_velocity(world, body_id);
        let w = body_get_angular_velocity(world, body_id);
        let mut spin = 0.0f32;
        let _axis = get_axis_angle(&mut spin, xf.q);
        let rad_to_deg = 180.0 / std::f32::consts::PI;
        let name = body_get_name(world, body_id);

        format!(
            "{{\"present\":true,\"id\":{id},\"name\":\"{name}\",\"type\":\"{ty}\",\
             \"pos\":[{px},{py},{pz}],\"spinDeg\":{spin},\
             \"vel\":[{vx},{vy},{vz}],\"omega\":[{wx},{wy},{wz}],\
             \"speed\":{speed},\"spinRate\":{spin_rate},\"mass\":{mass},\
             \"awake\":{awake},\"enabled\":{enabled},\"bullet\":{bullet},\
             \"gravityScale\":{gscale},\"shapeCount\":{sc},\"jointCount\":{jc}}}",
            id = body_id.index1,
            name = json_escape(name),
            ty = body_type_name(body_get_type(world, body_id)),
            px = xf.p.x as f32,
            py = xf.p.y as f32,
            pz = xf.p.z as f32,
            spin = spin * rad_to_deg,
            vx = v.x,
            vy = v.y,
            vz = v.z,
            wx = w.x,
            wy = w.y,
            wz = w.z,
            speed = length(v),
            spin_rate = length(w),
            mass = body_get_mass(world, body_id),
            awake = body_is_awake(world, body_id),
            enabled = body_is_enabled(world, body_id),
            bullet = body_is_bullet(world, body_id),
            gscale = body_get_gravity_scale(world, body_id),
            sc = body_get_shape_count(world, body_id),
            jc = body_get_joint_count(world, body_id),
        )
    })
    .unwrap_or_else(|| String::from("{\"present\":false}"))
}

// --------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------

/// Append one triangle-geometry record: `[ord, kind=0, floatCount, tris...]`.
fn push_tri_record(out: &mut Vec<f32>, ord: i32, tris: Vec<f32>) {
    out.push(ord as f32);
    out.push(RKIND_TRI);
    out.push(tris.len() as f32);
    out.extend_from_slice(&tris);
}

/// Flat, non-indexed body-local triangle-vertex list for a mesh shape, each vertex
/// scaled by the shape's per-instance `scale` (mirrors [`crate::vis`] mesh edges,
/// but filled triangles rather than edges).
fn mesh_triangles(mesh: &MeshData, scale: Vec3) -> Vec<f32> {
    let verts = get_mesh_vertices(mesh);
    let tris = get_mesh_triangles(mesh);
    let mut out = Vec::with_capacity(tris.len() * 9);
    for t in tris {
        for idx in [t.index1, t.index2, t.index3] {
            let v = verts[idx as usize];
            out.push(v.x * scale.x);
            out.push(v.y * scale.y);
            out.push(v.z * scale.z);
        }
    }
    out
}

/// Flat, non-indexed body-local triangle-vertex list for a height-field shape.
fn height_field_triangles(hf: &HeightFieldData) -> Vec<f32> {
    let count = get_height_field_triangle_count(hf);
    let mut out = Vec::with_capacity(count as usize * 9);
    for i in 0..count {
        let tri = get_height_field_triangle(hf, i);
        for v in tri.vertices {
            out.push(v.x);
            out.push(v.y);
            out.push(v.z);
        }
    }
    out
}
