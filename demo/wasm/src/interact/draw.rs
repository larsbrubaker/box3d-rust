//! Debug-draw collector for the demo overlay: adapts the engine's `world_draw`
//! into interleaved segment/point buffers and a JSON text channel, and owns the
//! global 16-bit view-flag mask + joint/force draw scales the menu bar drives.
//! Split out of `interact` to keep each file focused (and under the line limit).

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use box3d_rust::debug_draw::{DebugDraw, HexColor};
use box3d_rust::math_functions::{Aabb, Pos, Transform, Vec3, WorldTransform};
use box3d_rust::world::{world_draw, World};
use std::cell::Cell;
use wasm_bindgen::prelude::*;

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
    /// Large-world draw origin subtracted from every emitted point/segment/label,
    /// in `b3Pos` space (f64 under `double-precision`) *before* the `f32`
    /// truncation — the shared far-sample mechanism (C's draw-origin trick). The
    /// far modules set this to their scene base for the duration of a collect and
    /// clear it after; every other demo leaves it at the origin, where the
    /// subtraction is a bit-identical no-op. See [`with_draw_base`].
    static DRAW_BASE: Cell<Pos> = const { Cell::new(Pos { x: 0.0, y: 0.0, z: 0.0 }) };
}

/// Current large-world draw origin (see `DRAW_BASE`).
fn draw_base() -> Pos {
    DRAW_BASE.with(|c| c.get())
}

/// Run `f` with the draw origin set to `base`, restoring the previous value
/// afterward. The far samples wrap their `collect_debug_draw` / `collect_debug_text`
/// in this so the collector subtracts the base in `b3Pos` space at emission time;
/// for `base == 0` this changes nothing (bit-identical output).
pub fn with_draw_base<R>(base: Pos, f: impl FnOnce() -> R) -> R {
    let prev = DRAW_BASE.with(|c| c.replace(base));
    let out = f();
    DRAW_BASE.with(|c| c.set(prev));
    out
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

/// Restore the debug-draw joint/force scales to their `b3DebugDraw` defaults (both
/// 1.0). These live in thread-locals, so without an explicit reset a scene that set
/// a small `forceScale` (e.g. the Cylinder stacks' 0.001, Mesh Drop's 0.1) would
/// leak its value onto every subsequent demo. Called at each world-construction
/// seam via [`crate::interact::reset_scene_scales`]; a scene needing a non-default
/// scale re-applies it after reset (via `set_draw_scales` / `sim_set_draw_scales`).
pub fn reset_draw_scales() {
    set_draw_scales(1.0, 1.0);
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
    /// Large-world draw origin subtracted from every emitted position in `b3Pos`
    /// space before the `f32` truncation (`DRAW_BASE`; zero for near-origin demos).
    base: Pos,
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

    /// Subtract the draw origin in `b3Pos` space, then truncate to `f32`. For a
    /// zero base this is exactly `p.x as f32` (bit-identical).
    fn rx(&self, p: Pos) -> f32 {
        (p.x - self.base.x) as f32
    }
    fn ry(&self, p: Pos) -> f32 {
        (p.y - self.base.y) as f32
    }
    fn rz(&self, p: Pos) -> f32 {
        (p.z - self.base.z) as f32
    }
}

impl DebugDraw for CollectDraw {
    fn draw_segment(&mut self, p1: Pos, p2: Pos, color: HexColor) {
        self.segments.push(self.rx(p1));
        self.segments.push(self.ry(p1));
        self.segments.push(self.rz(p1));
        self.segments.push(self.rx(p2));
        self.segments.push(self.ry(p2));
        self.segments.push(self.rz(p2));
        Self::push_color(&mut self.segments, color);
    }

    fn draw_point(&mut self, p: Pos, size: f32, color: HexColor) {
        self.points.push(self.rx(p));
        self.points.push(self.ry(p));
        self.points.push(self.rz(p));
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
        base: draw_base(),
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
        base: draw_base(),
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
        push_json_number(&mut out, draw.rx(e.p));
        out.push_str(",\"y\":");
        push_json_number(&mut out, draw.ry(e.p));
        out.push_str(",\"z\":");
        push_json_number(&mut out, draw.rz(e.p));
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
