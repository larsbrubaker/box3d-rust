//! Shared mouse-grab, spawn/delete, counters, and debug-draw helpers for demo worlds.
//!
//! Mirrors the C samples' Sample::Mouse* grab (kinematic body + motor joint) and
//! `b3World_GetCounters` / `b3World_Draw` diagnostics.

use box3d_rust::body::{
    body_get_local_point, body_get_mass_data, body_get_type, body_is_valid, body_set_awake,
    body_set_target_transform, create_body, destroy_body, is_body_awake,
};
use box3d_rust::debug_draw::{DebugDraw, HexColor};
use box3d_rust::geometry::Sphere;
use box3d_rust::id::{BodyId, JointId, NULL_BODY_ID, NULL_JOINT_ID};
use box3d_rust::joint::{create_motor_joint, destroy_joint, joint_is_valid};
use box3d_rust::math_functions::{
    length, Aabb, Pos, Transform, Vec3, WorldTransform, QUAT_IDENTITY, VEC3_ZERO,
};
use box3d_rust::shape::{create_sphere_shape, shape_get_body};
use box3d_rust::types::{
    default_body_def, default_motor_joint_def, default_query_filter, default_shape_def, BodyType,
};
use box3d_rust::world::{world_cast_ray_closest, world_draw, world_get_counters, World};
use std::cell::Cell;
use wasm_bindgen::prelude::*;

/// Mouse-grab state for one demo world (C Sample mouse body + motor joint).
#[derive(Clone, Copy)]
pub struct MouseGrab {
    pub mouse_body_id: BodyId,
    pub mouse_joint_id: JointId,
    pub mouse_point: Pos,
    pub force_scale: f32,
}

impl Default for MouseGrab {
    fn default() -> Self {
        Self {
            mouse_body_id: NULL_BODY_ID,
            mouse_joint_id: NULL_JOINT_ID,
            mouse_point: Pos {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            force_scale: 100.0,
        }
    }
}

impl MouseGrab {
    /// Clear stale handles after a world reset/destroy.
    #[allow(dead_code)]
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    /// Drop the grab joint/body if either side was destroyed by the sim.
    pub fn validate(&mut self, world: &mut World) {
        if self.mouse_joint_id.is_non_null() && !joint_is_valid(world, self.mouse_joint_id) {
            self.mouse_joint_id = NULL_JOINT_ID;
            if self.mouse_body_id.is_non_null() && body_is_valid(world, self.mouse_body_id) {
                destroy_body(world, self.mouse_body_id);
            }
            self.mouse_body_id = NULL_BODY_ID;
        }
    }

    /// Drive the kinematic mouse body toward the current target (call before step).
    pub fn pre_step(&mut self, world: &mut World, time_step: f32) {
        self.validate(world);
        if self.mouse_body_id.is_non_null()
            && body_is_valid(world, self.mouse_body_id)
            && time_step > 0.0
        {
            let target = WorldTransform {
                p: self.mouse_point,
                q: QUAT_IDENTITY,
            };
            body_set_target_transform(world, self.mouse_body_id, target, time_step, true);
        }
    }

    /// Begin a grab: raycast, create kinematic mouse body + motor joint.
    /// Returns true if a dynamic body was grabbed.
    pub fn begin(&mut self, world: &mut World, origin: Pos, translation: Vec3) -> bool {
        self.end(world);

        let filter = default_query_filter();
        let result = world_cast_ray_closest(world, origin, translation, &filter);
        if !result.hit {
            return false;
        }

        let body_id = shape_get_body(world, result.shape_id);
        if body_get_type(world, body_id) != BodyType::Dynamic {
            return false;
        }

        self.mouse_point = result.point;

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = self.mouse_point;
        body_def.enable_sleep = false;
        self.mouse_body_id = create_body(world, &body_def);

        let mut joint_def = default_motor_joint_def();
        joint_def.base.body_id_a = self.mouse_body_id;
        joint_def.base.body_id_b = body_id;
        joint_def.base.local_frame_b = Transform {
            p: body_get_local_point(world, body_id, result.point),
            q: QUAT_IDENTITY,
        };
        joint_def.linear_hertz = 7.5;
        joint_def.linear_damping_ratio = 1.0;

        let mass_data = body_get_mass_data(world, body_id);
        let g = length(world.gravity);
        let mg = mass_data.mass * g;
        joint_def.max_spring_force = self.force_scale * mg;

        if mass_data.mass > 0.0 {
            let trace = mass_data.inertia.cx.x + mass_data.inertia.cy.y + mass_data.inertia.cz.z;
            let lever = (trace / (3.0 * mass_data.mass)).sqrt();
            joint_def.max_velocity_torque = 0.5 * lever * mg;
        }

        self.mouse_joint_id = create_motor_joint(world, &joint_def);
        body_set_awake(world, body_id, true);
        true
    }

    /// Update the grab target to a world-space point (camera-facing plane from JS).
    pub fn move_to(&mut self, point: Pos) {
        if self.mouse_joint_id.is_non_null() {
            self.mouse_point = point;
        }
    }

    /// Release the grab (destroy joint then kinematic body). Body keeps its velocity → fling.
    pub fn end(&mut self, world: &mut World) {
        if self.mouse_joint_id.is_non_null() && joint_is_valid(world, self.mouse_joint_id) {
            destroy_joint(world, self.mouse_joint_id, true);
        }
        if self.mouse_body_id.is_non_null() && body_is_valid(world, self.mouse_body_id) {
            destroy_body(world, self.mouse_body_id);
        }
        self.mouse_joint_id = NULL_JOINT_ID;
        self.mouse_body_id = NULL_BODY_ID;
    }

    pub fn is_active(&self) -> bool {
        self.mouse_joint_id.is_non_null()
    }
}

/// Descriptor for a body the demo renderer should track after spawn.
#[derive(Clone, Copy)]
pub struct SpawnedBody {
    pub body_index: i32,
    pub half_extents: [f32; 3],
    /// 0 = box, 1 = sphere, 2 = capsule
    pub kind: u8,
}

/// `m_launchSpeedScale` from the base `Sample` constructor (`sample.cpp` :330).
/// Individual samples override it (e.g. Compound Village sets 2.0) — a
/// per-demo override hook can arrive with those sample ports.
const LAUNCH_SPEED_SCALE: f32 = 5.0;
/// Projectile launch speed: `20.0 * m_launchSpeedScale` (`sample.cpp` :1243).
const PROJECTILE_SPEED: f32 = 20.0 * LAUNCH_SPEED_SCALE;
/// Fixed bullet-sphere radius (`sample.cpp` :1247, `b3Sphere{ zero, 0.25f }`).
const PROJECTILE_RADIUS: f32 = 0.25;
/// Density multiplier applied to the default shape density (`sample.cpp` :1248,
/// `shapeDef.density *= 4.0f`).
const PROJECTILE_DENSITY_SCALE: f32 = 4.0;

/// Spawn the C sample's shift-click projectile along a pick ray: a dynamic
/// bullet **sphere** of radius 0.25 at `origin + 2·direction`, launched at
/// `20·launchSpeedScale·direction`, with the default shape density boosted ×4.
/// Mirrors `Sample::MouseDown`'s plain shift branch (`sample.cpp` :1238-1250) —
/// no `MOD_CTRL` (cylinder) / `MOD_ALT` (ragdoll) variant. Returns a render
/// descriptor (`kind = 1`, sphere).
pub fn spawn_random(world: &mut World, origin: Pos, translation: Vec3) -> Option<SpawnedBody> {
    let len = length(translation);
    if len < 1e-8 {
        return None;
    }
    let direction = Vec3 {
        x: translation.x / len,
        y: translation.y / len,
        z: translation.z / len,
    };

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    // position = pickRay.origin + 2.0f * direction (sample.cpp :1242)
    body_def.position = Pos {
        x: origin.x + 2.0 * direction.x,
        y: origin.y + 2.0 * direction.y,
        z: origin.z + 2.0 * direction.z,
    };
    // linearVelocity = (20.0f * m_launchSpeedScale) * direction (sample.cpp :1243)
    body_def.linear_velocity = Vec3 {
        x: PROJECTILE_SPEED * direction.x,
        y: PROJECTILE_SPEED * direction.y,
        z: PROJECTILE_SPEED * direction.z,
    };
    body_def.is_bullet = true; // sample.cpp :1244

    let body_id = create_body(world, &body_def);

    // b3Sphere sphere = { b3Vec3_zero, 0.25f }; shapeDef.density *= 4.0f (:1247-1249)
    let mut shape_def = default_shape_def();
    shape_def.density *= PROJECTILE_DENSITY_SCALE;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: PROJECTILE_RADIUS,
    };
    create_sphere_shape(world, body_id, &shape_def, &sphere);

    Some(SpawnedBody {
        body_index: body_id.index1 - 1,
        half_extents: [PROJECTILE_RADIUS, PROJECTILE_RADIUS, PROJECTILE_RADIUS],
        kind: 1, // sphere
    })
}

/// Destroy the dynamic body under a pick ray. Returns the destroyed body index, or -1.
pub fn delete_at_ray(
    world: &mut World,
    grab: &mut MouseGrab,
    origin: Pos,
    translation: Vec3,
) -> i32 {
    grab.end(world);

    let filter = default_query_filter();
    let result = world_cast_ray_closest(world, origin, translation, &filter);
    if !result.hit {
        return -1;
    }

    let body_id = shape_get_body(world, result.shape_id);
    if body_get_type(world, body_id) != BodyType::Dynamic {
        return -1;
    }

    let index = body_id.index1 - 1;
    destroy_body(world, body_id);
    index
}

/// Counters + awake/sleeping dynamic body counts.
///
/// Layout: `[body, shape, contact, joint, island, awake_dynamic, sleeping_dynamic]`
pub fn counters_with_sleep(world: &World) -> [f32; 7] {
    let c = world_get_counters(world);
    let mut awake = 0i32;
    let mut sleeping = 0i32;
    for i in 0..world.bodies.len() {
        let body = &world.bodies[i];
        if body.set_index == box3d_rust::core::NULL_INDEX {
            continue;
        }
        if body.type_ != BodyType::Dynamic {
            continue;
        }
        if is_body_awake(world, i as i32) {
            awake += 1;
        } else {
            sleeping += 1;
        }
    }
    [
        c.body_count as f32,
        c.shape_count as f32,
        c.contact_count as f32,
        c.joint_count as f32,
        c.island_count as f32,
        awake as f32,
        sleeping as f32,
    ]
}

/// Menu view-flag bits, in the exact order the demo menu bar emits them and that
/// [`sim_set_debug_flags`] consumes. Mirrors the C sample's `ApplyGuiFlags`
/// option set (`debug_adapter.c` :212-232) plus the two style-path modes
/// (`shapes`, `transparent`) that the mesh + style pipeline handles instead of
/// the overlay collector.
pub const MENU_SHAPES: u32 = 1 << 0;
pub const MENU_TRANSPARENT: u32 = 1 << 1;
pub const MENU_JOINTS: u32 = 1 << 2;
pub const MENU_JOINT_EXTRAS: u32 = 1 << 3;
pub const MENU_BOUNDS: u32 = 1 << 4;
pub const MENU_MASS: u32 = 1 << 5;
pub const MENU_SLEEP: u32 = 1 << 6;
pub const MENU_BODY_NAMES: u32 = 1 << 7;
pub const MENU_GRAPH_COLORS: u32 = 1 << 8;
pub const MENU_ISLANDS: u32 = 1 << 9;
pub const MENU_CONTACTS: u32 = 1 << 10;
pub const MENU_CONTACT_NORMALS: u32 = 1 << 11;
pub const MENU_CONTACT_FEATURES: u32 = 1 << 12;
pub const MENU_CONTACT_FORCES: u32 = 1 << 13;
pub const MENU_FRICTION_FORCES: u32 = 1 << 14;
pub const MENU_ANCHOR_A: u32 = 1 << 15;

thread_local! {
    /// Current 16-bit menu mask (see `MENU_*`). Global + demo-agnostic: the menu
    /// sets it once via `sim_set_debug_flags`; every demo's overlay + style pass
    /// reads it. Default 0 (no overlays, opaque dynamics).
    static DEBUG_FLAGS: Cell<u32> = const { Cell::new(0) };
    /// `b3DebugDraw.jointScale` (default 1).
    static JOINT_SCALE: Cell<f32> = const { Cell::new(1.0) };
    /// `b3DebugDraw.forceScale` (default 1).
    static FORCE_SCALE: Cell<f32> = const { Cell::new(1.0) };
}

/// Set the global 16-bit view-flag mask (see `MENU_*`).
pub fn set_debug_flags(mask: u32) {
    DEBUG_FLAGS.with(|c| c.set(mask));
}

/// Read the global 16-bit view-flag mask.
pub fn debug_flags() -> u32 {
    DEBUG_FLAGS.with(|c| c.get())
}

/// Set the global joint/force draw scales (`b3DebugDraw.jointScale`/`forceScale`).
pub fn set_draw_scales(joint_scale: f32, force_scale: f32) {
    JOINT_SCALE.with(|c| c.set(joint_scale));
    FORCE_SCALE.with(|c| c.set(force_scale));
}

/// Whether dynamic bodies should draw translucent this frame — the
/// transparent-dynamic view mode (`MENU_TRANSPARENT`, C `SetTransparentDynamic`).
/// The style-word builders read this for `push_shape_styles`' `transparent_dynamic`.
pub fn transparent_dynamic() -> bool {
    debug_flags() & MENU_TRANSPARENT != 0
}

/// Set the global view-flag mask. See `MENU_*` for the bit order.
#[wasm_bindgen]
pub fn sim_set_debug_flags(mask: u32) {
    set_debug_flags(mask);
}

/// Set the global joint/force draw scales for the debug overlay.
#[wasm_bindgen]
pub fn sim_set_draw_scales(joint_scale: f32, force_scale: f32) {
    set_draw_scales(joint_scale, force_scale);
}

/// One engine `draw_string` label: world position, packed color, and the text.
struct TextEntry {
    p: Pos,
    color: HexColor,
    text: String,
}

struct CollectDraw {
    /// 16-bit menu mask (see `MENU_*`).
    flags: u32,
    joint_scale: f32,
    force_scale: f32,
    /// Interleaved segments: x1,y1,z1, x2,y2,z2, rgb_u32_as_f32
    segments: Vec<f32>,
    /// Interleaved points: x,y,z, size, rgb_u32_as_f32
    points: Vec<f32>,
    /// Recorded `draw_string` labels (overlay text channel).
    strings: Vec<TextEntry>,
}

impl CollectDraw {
    fn push_color(out: &mut Vec<f32>, color: HexColor) {
        out.push(f32::from_bits(color.0 & 0x00FF_FFFF));
    }
}

impl DebugDraw for CollectDraw {
    fn draw_segment(&mut self, p1: Pos, p2: Pos, color: HexColor) {
        self.segments.push(p1.x as f32);
        self.segments.push(p1.y as f32);
        self.segments.push(p1.z as f32);
        self.segments.push(p2.x as f32);
        self.segments.push(p2.y as f32);
        self.segments.push(p2.z as f32);
        Self::push_color(&mut self.segments, color);
    }

    fn draw_point(&mut self, p: Pos, size: f32, color: HexColor) {
        self.points.push(p.x as f32);
        self.points.push(p.y as f32);
        self.points.push(p.z as f32);
        self.points.push(size);
        Self::push_color(&mut self.points, color);
    }

    fn draw_string(&mut self, p: Pos, s: &str, color: HexColor) {
        self.strings.push(TextEntry {
            p,
            color,
            text: s.to_string(),
        });
    }

    fn draw_bounds(&mut self, aabb: Aabb, color: HexColor) {
        let l = aabb.lower_bound;
        let u = aabb.upper_bound;
        let corners = [
            [l.x, l.y, l.z],
            [u.x, l.y, l.z],
            [u.x, u.y, l.z],
            [l.x, u.y, l.z],
            [l.x, l.y, u.z],
            [u.x, l.y, u.z],
            [u.x, u.y, u.z],
            [l.x, u.y, u.z],
        ];
        let edges = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ];
        for (a, b) in edges {
            let pa = corners[a];
            let pb = corners[b];
            self.draw_segment(
                Pos {
                    x: pa[0] as _,
                    y: pa[1] as _,
                    z: pa[2] as _,
                },
                Pos {
                    x: pb[0] as _,
                    y: pb[1] as _,
                    z: pb[2] as _,
                },
                color,
            );
        }
    }

    fn draw_transform(&mut self, transform: WorldTransform) {
        let scale = 0.35f32;
        let origin = transform.p;
        let end = |axis: Vec3| -> Pos {
            let p = box3d_rust::math_functions::transform_point(
                Transform {
                    p: Vec3 {
                        x: origin.x as f32,
                        y: origin.y as f32,
                        z: origin.z as f32,
                    },
                    q: transform.q,
                },
                axis,
            );
            Pos {
                x: p.x as _,
                y: p.y as _,
                z: p.z as _,
            }
        };
        self.draw_segment(
            origin,
            end(Vec3 {
                x: scale,
                y: 0.0,
                z: 0.0,
            }),
            HexColor(0xFF4444),
        );
        self.draw_segment(
            origin,
            end(Vec3 {
                x: 0.0,
                y: scale,
                z: 0.0,
            }),
            HexColor(0x44FF44),
        );
        self.draw_segment(
            origin,
            end(Vec3 {
                x: 0.0,
                y: 0.0,
                z: scale,
            }),
            HexColor(0x4444FF),
        );
    }

    fn draw_box(&mut self, extents: Vec3, transform: WorldTransform, color: HexColor) {
        let hx = extents.x;
        let hy = extents.y;
        let hz = extents.z;
        let local = [
            [-hx, -hy, -hz],
            [hx, -hy, -hz],
            [hx, hy, -hz],
            [-hx, hy, -hz],
            [-hx, -hy, hz],
            [hx, -hy, hz],
            [hx, hy, hz],
            [-hx, hy, hz],
        ];
        let parent = Transform {
            p: Vec3 {
                x: transform.p.x as f32,
                y: transform.p.y as f32,
                z: transform.p.z as f32,
            },
            q: transform.q,
        };
        let mut world_pts = [[0.0f32; 3]; 8];
        for i in 0..8 {
            let lp = Vec3 {
                x: local[i][0],
                y: local[i][1],
                z: local[i][2],
            };
            let wp = box3d_rust::math_functions::transform_point(parent, lp);
            world_pts[i] = [wp.x, wp.y, wp.z];
        }
        let edges = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ];
        for (a, b) in edges {
            self.draw_segment(
                Pos {
                    x: world_pts[a][0] as _,
                    y: world_pts[a][1] as _,
                    z: world_pts[a][2] as _,
                },
                Pos {
                    x: world_pts[b][0] as _,
                    y: world_pts[b][1] as _,
                    z: world_pts[b][2] as _,
                },
                color,
            );
        }
    }

    fn drawing_bounds(&self) -> Aabb {
        Aabb {
            lower_bound: Vec3 {
                x: -1.0e6,
                y: -1.0e6,
                z: -1.0e6,
            },
            upper_bound: Vec3 {
                x: 1.0e6,
                y: 1.0e6,
                z: 1.0e6,
            },
        }
    }

    // Shapes are rendered as solid meshes from the pose/style stream, never as
    // overlay geometry, so the overlay collector always leaves draw_shapes off.
    fn force_scale(&self) -> f32 {
        self.force_scale
    }
    fn joint_scale(&self) -> f32 {
        self.joint_scale
    }
    fn draw_joints(&self) -> bool {
        self.flags & MENU_JOINTS != 0
    }
    fn draw_joint_extras(&self) -> bool {
        self.flags & MENU_JOINT_EXTRAS != 0
    }
    fn draw_bounds_boxes(&self) -> bool {
        self.flags & MENU_BOUNDS != 0
    }
    fn draw_mass(&self) -> bool {
        self.flags & MENU_MASS != 0
    }
    fn draw_sleep(&self) -> bool {
        self.flags & MENU_SLEEP != 0
    }
    fn draw_body_names(&self) -> bool {
        self.flags & MENU_BODY_NAMES != 0
    }
    fn draw_graph_colors(&self) -> bool {
        self.flags & MENU_GRAPH_COLORS != 0
    }
    fn draw_islands(&self) -> bool {
        self.flags & MENU_ISLANDS != 0
    }
    fn draw_contacts(&self) -> bool {
        self.flags & MENU_CONTACTS != 0
    }
    fn draw_contact_normals(&self) -> bool {
        self.flags & MENU_CONTACT_NORMALS != 0
    }
    fn draw_contact_features(&self) -> bool {
        self.flags & MENU_CONTACT_FEATURES != 0
    }
    fn draw_contact_forces(&self) -> bool {
        self.flags & MENU_CONTACT_FORCES != 0
    }
    fn draw_friction_forces(&self) -> bool {
        self.flags & MENU_FRICTION_FORCES != 0
    }
    fn draw_anchor_a(&self) -> bool {
        self.flags & MENU_ANCHOR_A != 0
    }
}

/// Collect debug-draw geometry.
///
/// Layout: `[seg_count, point_count, ...segments (7 floats each), ...points (5 floats each)]`
pub fn collect_debug_draw(world: &mut World) -> Vec<f32> {
    let flags = debug_flags();
    // `shapes` and `transparent` drive the solid-mesh + style path, not the
    // overlay; only the remaining bits produce overlay segments/points.
    if flags & !(MENU_SHAPES | MENU_TRANSPARENT) == 0 {
        return vec![0.0, 0.0];
    }
    let mut draw = CollectDraw {
        flags,
        joint_scale: JOINT_SCALE.with(|c| c.get()),
        force_scale: FORCE_SCALE.with(|c| c.get()),
        segments: Vec::new(),
        points: Vec::new(),
        strings: Vec::new(),
    };
    world_draw(world, &mut draw, u64::MAX);
    let seg_count = (draw.segments.len() / 7) as f32;
    let point_count = (draw.points.len() / 5) as f32;
    let mut out = Vec::with_capacity(2 + draw.segments.len() + draw.points.len());
    out.push(seg_count);
    out.push(point_count);
    out.extend_from_slice(&draw.segments);
    out.extend_from_slice(&draw.points);
    out
}

/// Menu bits that make `world_draw` emit `draw_string` labels: mass (`draw.rs`
/// :440), sleep (:473), body names (:421), contact separation/feature/force text
/// (:237/:252/:249/:288), and joint force/torque labels (`joint/draw.rs` :611).
/// When none are set the engine produces no text, so we skip the draw pass.
const TEXT_FLAGS: u32 = MENU_MASS
    | MENU_SLEEP
    | MENU_BODY_NAMES
    | MENU_CONTACT_NORMALS
    | MENU_CONTACT_FEATURES
    | MENU_CONTACT_FORCES
    | MENU_JOINT_EXTRAS;

/// Collect the engine's `draw_string` overlay text for the currently-enabled
/// view flags and serialize it as a JSON array.
///
/// # Schema
///
/// ```json
/// [{"x":1.0,"y":2.0,"z":3.0,"color":16777215,"text":"  0.42"}]
/// ```
///
/// One object per label the engine draws this frame, in `world_draw` emission
/// order. `x`/`y`/`z` are the label's world-space anchor (`f32`); `color` is the
/// packed 0xRRGGBB color as a decimal `u32` (e.g. `16777215` = white); `text` is
/// the JSON-escaped label. Returns `"[]"` when no text-relevant flag is set.
pub fn collect_debug_text(world: &mut World) -> String {
    let flags = debug_flags();
    if flags & TEXT_FLAGS == 0 {
        return "[]".to_string();
    }
    let mut draw = CollectDraw {
        flags,
        joint_scale: JOINT_SCALE.with(|c| c.get()),
        force_scale: FORCE_SCALE.with(|c| c.get()),
        segments: Vec::new(),
        points: Vec::new(),
        strings: Vec::new(),
    };
    world_draw(world, &mut draw, u64::MAX);

    let mut out = String::from("[");
    for (i, e) in draw.strings.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str("{\"x\":");
        push_json_number(&mut out, e.p.x as f32);
        out.push_str(",\"y\":");
        push_json_number(&mut out, e.p.y as f32);
        out.push_str(",\"z\":");
        push_json_number(&mut out, e.p.z as f32);
        out.push_str(",\"color\":");
        out.push_str(&(e.color.0 & 0x00FF_FFFF).to_string());
        out.push_str(",\"text\":\"");
        push_json_escaped(&mut out, &e.text);
        out.push_str("\"}");
    }
    out.push(']');
    out
}

/// Append a finite-or-not `f32` as a JSON number, falling back to `0` for
/// non-finite values (JSON has no NaN/Infinity literal).
fn push_json_number(out: &mut String, v: f32) {
    if v.is_finite() {
        out.push_str(&v.to_string());
    } else {
        out.push('0');
    }
}

/// Append `s` to `out` with the JSON string escapes required inside `"..."`.
fn push_json_escaped(out: &mut String, s: &str) {
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

/// Raycast closest hit for demos that only need hit info.
/// Returns `[hit, px, py, pz, nx, ny, nz, fraction]` (hit is 0/1).
#[allow(dead_code)]
pub fn ray_closest(world: &World, origin: Pos, translation: Vec3) -> [f32; 8] {
    let filter = default_query_filter();
    let r = world_cast_ray_closest(world, origin, translation, &filter);
    if r.hit {
        [
            1.0,
            r.point.x as f32,
            r.point.y as f32,
            r.point.z as f32,
            r.normal.x,
            r.normal.y,
            r.normal.z,
            r.fraction,
        ]
    } else {
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }
}

/// Helper: build Pos/Vec3 from floats.
pub fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

pub fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}
