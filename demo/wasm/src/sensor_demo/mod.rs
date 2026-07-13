//! Faithful C event demos hosted on the `sensors` route.
//!
//! Sensor scenes (sample_events.cpp / sample_benchmark.cpp — verified exact):
//! Sensor Visit, Sensor Hits, Benchmark Sensor — see [`sensor_scenes`].
//!
//! Events scenes (sample_events.cpp): Hit, Move, Joint, Persistent Contact —
//! see [`event_scenes`]. These add the overlay (segments/points), 3D debug text
//! (`sensor_debug_text`), ground-mesh wireframe, and HUD readout channels that
//! the sensor scenes do not need.

mod event_scenes;
mod sensor_scenes;

use crate::interact::{self, MouseGrab};
use crate::rng::XorShift32;
use crate::vis::pos;
use crate::vis::{push_poses, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::events::ContactHitEvent;
use box3d_rust::hull::make_box_hull;
use box3d_rust::id::{
    BodyId, ContactId, JointId, ShapeId, NULL_BODY_ID, NULL_CONTACT_ID, NULL_JOINT_ID,
    NULL_SHAPE_ID,
};
use box3d_rust::math_functions::{Vec3, VEC3_ZERO};
use box3d_rust::mesh::MeshData;
use box3d_rust::shape::create_hull_shape;
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

const RAND_SEED: u32 = 12345;

thread_local! {
    static STATE: RefCell<Option<SensorState>> = const { RefCell::new(None) };
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum SceneKind {
    Visit,
    Hits,
    Benchmark,
    Hit,
    Move,
    Joint,
    PersistentContact,
}

/// Parallel visualization arrays for every scene plus an index that maps a body's
/// `index1 - 1` to its slot, so per-event colour lookups and removals are O(1)
/// instead of linear scans over the (up to ~3200-entry) body list.
#[derive(Default)]
pub(crate) struct VisSet {
    pub bodies: Vec<VisBody>,
    pub colors: Vec<u32>,
    pub is_sensor: Vec<bool>,
    /// `body_index` (`id.index1 - 1`) → slot in the parallel arrays.
    index: std::collections::HashMap<i32, usize>,
    /// Bumps whenever a sensor slot is added, removed, or relocated by a swap-remove, so the
    /// JS side can cache `sensor_sensor_indices()` and refetch it only when it changes.
    sensor_topo: u32,
}

impl VisSet {
    pub fn push(&mut self, mut body: VisBody, color: u32, sensor: bool) {
        body.color = color;
        let slot = self.bodies.len();
        self.index.insert(body.body_index, slot);
        self.bodies.push(body);
        self.colors.push(color);
        self.is_sensor.push(sensor);
        if sensor {
            self.sensor_topo = self.sensor_topo.wrapping_add(1);
        }
    }

    /// Remove the slot for `body_index` via swap-remove (O(1)); keeps `index` consistent by
    /// re-pointing the element that the swap moved into the freed slot.
    pub fn remove_body(&mut self, body_index: i32) {
        let Some(slot) = self.index.remove(&body_index) else {
            return;
        };
        let last = self.bodies.len() - 1;
        let removed_sensor = self.is_sensor[slot];
        let moved_sensor = self.is_sensor[last];
        self.bodies.swap_remove(slot);
        self.colors.swap_remove(slot);
        self.is_sensor.swap_remove(slot);
        if slot != last {
            // The former last element now lives at `slot`; fix its index entry.
            self.index.insert(self.bodies[slot].body_index, slot);
            if moved_sensor {
                self.sensor_topo = self.sensor_topo.wrapping_add(1);
            }
        }
        if removed_sensor {
            self.sensor_topo = self.sensor_topo.wrapping_add(1);
        }
    }

    pub fn find(&self, body_index: i32) -> Option<usize> {
        self.index.get(&body_index).copied()
    }

    pub fn set_color(&mut self, slot: usize, color: u32) {
        self.colors[slot] = color;
        self.bodies[slot].color = color;
    }
}

pub(crate) struct SensorState {
    pub world: box3d_rust::world::World,
    pub vis: VisSet,
    pub kind: SceneKind,
    /// Global mouse-grab (C Sample mouse body + motor joint). Drives the Events
    /// Joint scene's throw-to-break interaction; a no-op default for the other
    /// scenes, which never begin a grab.
    pub grab: MouseGrab,
    pub begin_total: u32,
    pub end_total: u32,
    pub last_begin: u32,
    pub last_end: u32,
    pub max_begin: u32,
    pub max_end: u32,
    pub visit_sensor: ShapeId,
    #[allow(dead_code)]
    pub grid_mesh: Option<MeshData>,
    pub kinematic_body: BodyId,
    pub joint_id: JointId,
    pub bullet_body: BodyId,
    pub is_bullet: bool,
    pub step_count: u32,
    pub last_step_count: u32,
    pub filter_row: i32,
    pub rng: XorShift32,
    // --- Events scenes (Hit / Move / Joint / Persistent Contact) ---
    /// Move: the spinning tall box; also reused as the tracked body for a scene.
    pub event_body: BodyId,
    /// Move: local frame of the pivot point used for the velocity readout.
    pub move_local_pivot: Vec3,
    /// Joint: the (up to 6) joint ids, some null for the disabled motor/wheel slots.
    pub joint_ids: Vec<JointId>,
    /// Persistent Contact: the currently tracked contact id.
    pub contact_id: ContactId,
    /// Hit: accumulated contact hit events (capped at [`MAX_HIT_EVENTS`]).
    pub hit_events: Vec<ContactHitEvent>,
    /// Packed overlay segments+points for the debug-draw channel (DebugDrawOverlay layout).
    pub overlay: Vec<f32>,
    /// JSON `[{x,y,z,color,text}]` 3D debug-text labels for the current step.
    pub labels: String,
    /// JSON `[{label,value}]` HUD readout rows for the current step.
    pub hud: String,
    /// Ground mesh triangle edges for Hit / Persistent Contact (empty otherwise).
    pub ground_wire: Vec<f32>,
}

/// Max accumulated hit markers in the Hit sample (C `e_maxEvents` / `m_maxEvents`).
pub(crate) const MAX_HIT_EVENTS: usize = 32;

impl SensorState {
    /// Build a blank state for `kind` around a fresh `world`/`vis`; each reset
    /// function overrides the fields that its scene actually uses. Keeps the
    /// (large) field list in one place so the sensor scenes' exact values are
    /// unchanged from before the events scenes were added.
    pub(crate) fn blank(world: box3d_rust::world::World, vis: VisSet, kind: SceneKind) -> Self {
        SensorState {
            world,
            vis,
            kind,
            grab: MouseGrab::default(),
            begin_total: 0,
            end_total: 0,
            last_begin: 0,
            last_end: 0,
            max_begin: 0,
            max_end: 0,
            visit_sensor: NULL_SHAPE_ID,
            grid_mesh: None,
            kinematic_body: NULL_BODY_ID,
            joint_id: NULL_JOINT_ID,
            bullet_body: NULL_BODY_ID,
            is_bullet: true,
            step_count: 0,
            last_step_count: 0,
            filter_row: 0,
            rng: XorShift32::with_seed(RAND_SEED),
            event_body: NULL_BODY_ID,
            move_local_pivot: VEC3_ZERO,
            joint_ids: Vec::new(),
            contact_id: NULL_CONTACT_ID,
            hit_events: Vec::new(),
            overlay: Vec::new(),
            labels: String::from("[]"),
            hud: String::from("[]"),
            ground_wire: Vec::new(),
        }
    }
}

fn with_state<R>(f: impl FnOnce(&mut SensorState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("sensor not initialized — call sensor_reset first"))
    })
}

fn new_world() -> box3d_rust::world::World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    box3d_rust::world::World::new(&def)
}

/// (Sample::AddGroundBox) — a static box at y = -1 with half extents
/// `(extent, 1, extent)`. Returns the ground body id.
fn add_ground_box(world: &mut box3d_rust::world::World, extent: f32) -> BodyId {
    let mut body_def = default_body_def();
    body_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(world, &body_def);
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape(world, ground, &default_shape_def(), &hull.base);
    ground
}

// --- Overlay + JSON builders shared by the events scenes ----------------------

/// Debug-draw overlay buffer matching the JS `DebugDrawOverlay` layout
/// `[seg_count, point_count, ...segments(7 floats), ...points(5 floats)]`. Colors
/// are packed as `f32::from_bits(0xRRGGBB)` (the JS side reinterprets the bits).
#[derive(Default)]
pub(crate) struct OverlayBuf {
    segments: Vec<f32>,
    points: Vec<f32>,
}

impl OverlayBuf {
    pub fn seg(&mut self, a: Vec3, b: Vec3, color: u32) {
        self.segments
            .extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z, f32::from_bits(color)]);
    }
    pub fn point(&mut self, p: Vec3, size: f32, color: u32) {
        self.points
            .extend_from_slice(&[p.x, p.y, p.z, size, f32::from_bits(color)]);
    }
    pub fn pack(self) -> Vec<f32> {
        let mut out = Vec::with_capacity(2 + self.segments.len() + self.points.len());
        out.push((self.segments.len() / 7) as f32);
        out.push((self.points.len() / 5) as f32);
        out.extend_from_slice(&self.segments);
        out.extend_from_slice(&self.points);
        out
    }
}

/// One 3D debug-text label (C `DrawString3D`).
pub(crate) struct Label {
    pub p: Vec3,
    pub color: u32,
    pub text: String,
}

/// Serialize labels to the `sensor_debug_text` JSON shape
/// `[{"x":..,"y":..,"z":..,"color":<u32>,"text":".."}]`.
pub(crate) fn labels_json(labels: &[Label]) -> String {
    let mut out = String::from("[");
    for (i, l) in labels.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str("{\"x\":");
        push_num(&mut out, l.p.x);
        out.push_str(",\"y\":");
        push_num(&mut out, l.p.y);
        out.push_str(",\"z\":");
        push_num(&mut out, l.p.z);
        out.push_str(",\"color\":");
        out.push_str(&(l.color & 0x00FF_FFFF).to_string());
        out.push_str(",\"text\":\"");
        push_escaped(&mut out, &l.text);
        out.push_str("\"}");
    }
    out.push(']');
    out
}

/// Serialize HUD rows to `[{"label":"..","value":".."}]` for `updateReadout`.
pub(crate) fn hud_json(rows: &[(String, String)]) -> String {
    let mut out = String::from("[");
    for (i, (label, value)) in rows.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str("{\"label\":\"");
        push_escaped(&mut out, label);
        out.push_str("\",\"value\":\"");
        push_escaped(&mut out, value);
        out.push_str("\"}");
    }
    out.push(']');
    out
}

fn push_num(out: &mut String, v: f32) {
    if v.is_finite() {
        out.push_str(&v.to_string());
    } else {
        out.push('0');
    }
}

fn push_escaped(out: &mut String, s: &str) {
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
}

// --- wasm exports -------------------------------------------------------------

#[wasm_bindgen]
pub fn sensor_reset(scene: u32) -> u32 {
    STATE.with(|cell| {
        let prev_bullet = cell.borrow().as_ref().map(|s| s.is_bullet).unwrap_or(true);
        let state = match scene {
            1 => sensor_scenes::reset_hits(prev_bullet),
            2 => sensor_scenes::reset_benchmark(),
            3 => event_scenes::reset_hit(),
            4 => event_scenes::reset_move(),
            5 => event_scenes::reset_joint(),
            6 => event_scenes::reset_persistent(),
            _ => sensor_scenes::reset_visit(),
        };
        let count = state.vis.bodies.len() as u32;
        *cell.borrow_mut() = Some(state);
        count
    })
}

#[wasm_bindgen]
pub fn sensor_set_bullet(flag: bool) {
    with_state(|state| {
        state.is_bullet = flag;
    });
}

#[wasm_bindgen]
pub fn sensor_is_bullet() -> bool {
    with_state(|state| state.is_bullet)
}

#[wasm_bindgen]
pub fn sensor_launch() {
    with_state(sensor_scenes::launch_bullet);
}

#[wasm_bindgen]
pub fn sensor_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        // Drive the mouse-grab body toward its target before stepping, like
        // joint_demo (C Sample::Step). A no-op when no grab is active.
        state.grab.pre_step(&mut state.world, dt);
        if state.kind == SceneKind::Hits {
            sensor_scenes::process_hits_pre_step(state);
        }
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.wrapping_add(1);
        match state.kind {
            SceneKind::Visit => sensor_scenes::process_visit_events(state),
            SceneKind::Hits => sensor_scenes::process_hits_events(state),
            SceneKind::Benchmark => sensor_scenes::process_benchmark_events(state),
            SceneKind::Hit => event_scenes::process_hit(state),
            SceneKind::Move => event_scenes::process_move(state),
            SceneKind::Joint => event_scenes::process_joint(state),
            SceneKind::PersistentContact => event_scenes::process_persistent(state),
        }
        state.vis.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn sensor_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.vis.bodies, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn sensor_colors() -> Vec<f32> {
    with_state(|state| state.vis.colors.iter().map(|&c| c as f32).collect())
}

#[wasm_bindgen]
pub fn sensor_sensor_indices() -> Vec<u32> {
    with_state(|state| {
        state
            .vis
            .is_sensor
            .iter()
            .enumerate()
            .filter(|(_, s)| **s)
            .map(|(i, _)| i as u32)
            .collect()
    })
}

/// Monotonic counter that changes only when a sensor slot is added, removed, or relocated.
/// The JS side caches `sensor_sensor_indices()` and refetches it only when this value moves,
/// so the ~1600-entry benchmark set is marshaled once per reset instead of once per frame.
#[wasm_bindgen]
pub fn sensor_topology_version() -> u32 {
    with_state(|state| state.vis.sensor_topo)
}

#[wasm_bindgen]
pub fn sensor_event_stats() -> Vec<f32> {
    with_state(|state| {
        let (a, b, flag) = match state.kind {
            SceneKind::Benchmark => (state.max_begin, state.max_end, 1.0),
            _ => (state.begin_total, state.end_total, 0.0),
        };
        vec![
            a as f32,
            b as f32,
            state.last_begin as f32,
            state.last_end as f32,
            flag,
        ]
    })
}

/// Packed overlay geometry (segments + points) for the current step's event
/// markers — Hit (yellow hit points + approach-speed rays) and Persistent
/// Contact (crimson manifold-impulse rays). Empty (`[0, 0]`) for other scenes.
#[wasm_bindgen]
pub fn sensor_overlay() -> Vec<f32> {
    with_state(|state| {
        if state.overlay.is_empty() {
            vec![0.0, 0.0]
        } else {
            state.overlay.clone()
        }
    })
}

/// JSON `[{x,y,z,color,text}]` 3D debug-text labels for the current step
/// (C `DrawString3D`). `"[]"` for scenes that draw no strings.
#[wasm_bindgen]
pub fn sensor_debug_text() -> String {
    with_state(|state| state.labels.clone())
}

/// JSON `[{label,value}]` HUD readout rows for the events scenes (C `DrawTextLine`
/// output). `"[]"` for the sensor scenes, which use `sensor_event_stats` instead.
#[wasm_bindgen]
pub fn sensor_hud() -> String {
    with_state(|state| state.hud.clone())
}

/// Ground mesh triangle edges for Hit / Persistent Contact (built at reset).
/// Empty for scenes whose ground is a box drawn from the pose stream.
#[wasm_bindgen]
pub fn sensor_ground_wireframe() -> Vec<f32> {
    with_state(|state| state.ground_wire.clone())
}

// --- Mouse interaction shell (standard demo grab/spawn/delete) -----------------
//
// The Events Joint scene lets users throw bodies to snap the joints, matching C's
// global mouse interaction. Sensor scenes are near-origin, so no large-world base
// offset is applied (mirrors joint_demo's manual exports; the shell macro does not
// fit because the render list is a `VisSet`, not a plain `Vec<VisBody>`).

/// Begin a ctrl-drag grab: raycast from the ray, and if a dynamic body is hit
/// create the kinematic mouse body + motor joint. Returns `[hit, px, py, pz]`.
#[wasm_bindgen]
pub fn sensor_mouse_down(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_state(|state| {
        if state.grab.begin(
            &mut state.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            let p = state.grab.mouse_point;
            vec![1.0, p.x as f32, p.y as f32, p.z as f32]
        } else {
            vec![0.0, 0.0, 0.0, 0.0]
        }
    })
}

/// Update the grab target to a world point (camera-facing plane from JS).
#[wasm_bindgen]
pub fn sensor_mouse_move(px: f32, py: f32, pz: f32) {
    with_state(|state| {
        state.grab.move_to(interact::pos(px, py, pz));
    });
}

/// Release the grab (the body keeps its velocity → fling).
#[wasm_bindgen]
pub fn sensor_mouse_up() {
    with_state(|state| {
        state.grab.end(&mut state.world);
    });
}

/// Whether a grab joint is currently active.
#[wasm_bindgen]
pub fn sensor_mouse_active() -> bool {
    with_state(|state| state.grab.is_active())
}

/// Shift-click spawn the C bullet sphere along the pick ray, tracked as a
/// non-sensor render body. Returns `[hit, index, hx, hy, hz, kind]`.
#[wasm_bindgen]
pub fn sensor_spawn_random(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_state(|state| {
        match interact::spawn_random(
            &mut state.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            Some(spawned) => {
                let idx = spawned.body_index;
                let r = spawned.half_extents[0];
                state.vis.push(VisBody::sphere_body(idx, r), 0, false);
                vec![1.0, idx as f32, r, r, r, spawned.kind as f32]
            }
            None => vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    })
}

/// Destroy the dynamic body under the pick ray and prune it from the render set.
/// Returns 1 on a hit, 0 otherwise.
#[wasm_bindgen]
pub fn sensor_delete_at_ray(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> u32 {
    with_state(|state| {
        let index = interact::delete_at_ray(
            &mut state.world,
            &mut state.grab,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        );
        if index < 0 {
            return 0;
        }
        state.vis.remove_body(index);
        1
    })
}
