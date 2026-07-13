//! Engine-driven per-shape draw style, captured the way the C samples app gets
//! it: run `b3World_Draw` and let the debug adapter record the `b3HexColor` the
//! engine emits for every shape, plus the PBR material the adapter derives from
//! that color and the body type.
//!
//! This replaces deriving body colors in TypeScript. The colors come straight
//! from our ported `world_draw` -> `DrawQueryCallback` state machine
//! (`src/world/draw.rs` `shape_debug_color`, itself a port of
//! `physics_world.c` `DrawQueryCallback` :1223-1298), so the awake / sleeping /
//! static / kinematic / sensor / bullet / fast / speed-capped / custom-color
//! rules stay in one place — the engine — instead of being reimplemented.
//!
//! # Why a capture pass instead of recomputing per body
//!
//! The color depends on transient per-step body flags (`b3_isFast`,
//! `b3_hadTimeOfImpact`, `b3_isSpeedCapped`) and on `set_index` (awake vs
//! sleeping) that the library does not expose through its public API. The only
//! way to get the *exact* color the engine would draw is to run the real draw
//! path. That is precisely what the C debug adapter does
//! (`samples/gfx/debug_adapter.c`): it registers `createDebugShape` /
//! `DrawShapeFcn` callbacks and reads the color the engine hands it.
//!
//! # Style word layout (`vis_shape_style`)
//!
//! One `u32` per pose entry, in the same order as [`crate::vis::push_poses`].
//! Parallel array — the pose stream (`POSE_STRIDE = 16` floats) is unchanged,
//! so existing TypeScript readers keep working untouched this wave.
//!
//! ```text
//!  bit  31..30  reserved (0)
//!  bit  29      transparent flag  (1 => alpha 0.5, else 1.0)
//!  bits 28..27  body type         (0 static, 1 kinematic, 2 dynamic)
//!  bits 26..24  material preset   (b3DebugMaterial, effective 0..5)
//!  bits 23..0   RGB color         (0xRRGGBB, sRGB, material byte stripped)
//! ```
//!
//! # Reproducing the C adapter's material resolution in TypeScript
//!
//! Mirror `debug_adapter.c` `DrawShape` :832-861 exactly:
//!
//! ```text
//! const KDebugMaterialMetallic  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.85]; // debug_adapter.c:145
//! const KDebugMaterialRoughness = [0.50, 0.85, 0.65, 0.95, 0.30, 0.35]; // debug_adapter.c:146
//! const KBodyTypeMetallic  = [0.0, 0.0, 0.0];        // debug_adapter.c:139
//! const KBodyTypeRoughness = [0.70, 0.55, 0.40];     // debug_adapter.c:140
//!
//! const rgb        =  style        & 0xFFFFFF;
//! const material   = (style >> 24) & 0x7;
//! const bodyType   = (style >> 27) & 0x3;
//! const transparent= (style >> 29) & 0x1;
//!
//! let metallic, roughness;
//! if (material >= 1 && material <= 5) {         // preset rides in the color high byte
//!     metallic  = KDebugMaterialMetallic[material];
//!     roughness = KDebugMaterialRoughness[material];
//! } else {                                       // fall back to the per-bodyType table
//!     metallic  = KBodyTypeMetallic[bodyType];
//!     roughness = KBodyTypeRoughness[bodyType];
//! }
//! const alpha = transparent ? 0.5 : 1.0;
//! // Base color is sRGB; convert to linear before lighting exactly like
//! // debug_adapter.c HexColorToLinear (:409) when rendering lit shapes.
//! ```

// Consumed by the demo state structs during the later TS-integration wave.
#![allow(dead_code)]

use std::cell::RefCell;
use std::collections::HashMap;

use box3d_rust::debug_draw::{DebugDraw, DebugShape, HexColor};
use box3d_rust::math_functions::{Aabb, Vec3, WorldTransform};
use box3d_rust::types::BodyType;
use box3d_rust::world::{world_draw, World};

use crate::vis::VisBody;

/// Mask isolating the 0xRRGGBB color (bits 0..24).
pub const STYLE_COLOR_MASK: u32 = 0x00FF_FFFF;
/// Shift for the material-preset field (bits 24..27).
pub const STYLE_MATERIAL_SHIFT: u32 = 24;
/// Width mask (post-shift) for the material-preset field.
pub const STYLE_MATERIAL_MASK: u32 = 0x7;
/// Shift for the body-type field (bits 27..29).
pub const STYLE_BODY_TYPE_SHIFT: u32 = 27;
/// Width mask (post-shift) for the body-type field.
pub const STYLE_BODY_TYPE_MASK: u32 = 0x3;
/// Transparent-dynamic flag bit (bit 29): alpha 0.5 when set.
pub const STYLE_TRANSPARENT_BIT: u32 = 1 << 29;

/// b3DebugMaterial values, kept here so the packing is self-documenting.
/// (`debug_draw.rs` `DebugMaterial` / `types.h` `b3DebugMaterial`.)
const MATERIAL_DEFAULT: u32 = 0;
const MATERIAL_METALLIC: u32 = 5;

/// Body-type index used by the per-bodyType material table.
/// (`debug_adapter.c` :138 — static 0, kinematic 1, dynamic 2.)
fn body_type_index(t: BodyType) -> u32 {
    match t {
        BodyType::Static => 0,
        BodyType::Kinematic => 1,
        BodyType::Dynamic => 2,
    }
}

/// `createDebugShape` callback: stamp the shape's `index1` as the user handle so
/// the draw callback can key the emitted color back to the shape. Non-zero for
/// every real shape (`index1 = shape.id + 1 >= 1`), which is what `world_draw`
/// requires before it will call `DrawShapeFcn`. Mirrors the C adapter allocating
/// a pool slot per shape (`debug_adapter.c` `AdapterCreateDebugShape`).
fn capture_create(shape: &DebugShape<'_>, _ctx: u64) -> u64 {
    shape.shape_id.index1 as u64
}

/// `destroyDebugShape` callback: nothing to release — the handle is just an int.
fn capture_destroy(_handle: u64, _ctx: u64) {}

/// Capturing `DebugDraw`: records the engine color per shape and nothing else.
/// Every option flag is off except `draw_shapes`, so `world_draw` runs only the
/// shape pass (no joints / bounds / contacts / names), exactly like the C
/// adapter configured with `drawShapes = true`.
struct StyleCapture<'a> {
    /// shape `index1` (the user handle) -> packed engine color `0xMMRRGGBB`.
    /// Borrows the reused thread-local scratch map so no allocation happens per
    /// capture (the map is cleared before each pass, retaining its capacity).
    colors: &'a mut HashMap<u64, u32>,
}

impl DebugDraw for StyleCapture<'_> {
    fn draw_shapes(&self) -> bool {
        true
    }

    fn draw_shape(&mut self, user_shape: u64, _transform: WorldTransform, color: HexColor) -> bool {
        // `color` is the packed 32-bit engine color: 0xRRGGBB with the
        // b3DebugMaterial preset in the high byte (physics_world.c :1298
        // b3MakeDebugColor, or a shape custom color passed through :1229).
        self.colors.insert(user_shape, color.0);
        true
    }

    fn drawing_bounds(&self) -> Aabb {
        // Cover the whole world so no shape is culled, regardless of world
        // offset (e.g. the far-origin samples). 1e18 is finite (passes
        // is_valid_aabb) and far below f32::MAX, so broad-phase math cannot
        // overflow.
        const H: f32 = 1.0e18;
        Aabb {
            lower_bound: Vec3 {
                x: -H,
                y: -H,
                z: -H,
            },
            upper_bound: Vec3 { x: H, y: H, z: H },
        }
    }
}

thread_local! {
    /// Reused scratch for the per-frame shape-color capture. Cleared (capacity
    /// retained) before each `world_draw` pass so the many `*_styles()` builders
    /// that run every frame don't churn a fresh `HashMap` allocation each call.
    static COLOR_SCRATCH: RefCell<HashMap<u64, u32>> = RefCell::new(HashMap::new());
}

/// Run the real engine draw path once and hand a fresh shape-`index1` → packed
/// engine color map to `f`. The map is the reused thread-local scratch
/// ([`COLOR_SCRATCH`]), cleared before the pass, so no allocation happens per
/// capture; its contents are only valid for the duration of `f`.
///
/// Installs the capture callbacks on the world if they are not already present.
/// The handles it stamps (`user_shape = shape index1`) persist on the shapes and
/// are reused on later calls; only the freshly recomputed colors change.
fn with_engine_colors<R>(world: &mut World, f: impl FnOnce(&World, &HashMap<u64, u32>) -> R) -> R {
    if world.create_debug_shape.is_none() {
        world.create_debug_shape = Some(capture_create);
    }
    if world.destroy_debug_shape.is_none() {
        world.destroy_debug_shape = Some(capture_destroy);
    }

    COLOR_SCRATCH.with(|cell| {
        let mut colors = cell.borrow_mut();
        colors.clear();
        {
            // ~0 mask: draw every collision category.
            let mut capture = StyleCapture {
                colors: &mut colors,
            };
            world_draw(world, &mut capture, u64::MAX);
        }
        f(world, &colors)
    })
}

/// Pack one style word from an engine color and a body type.
///
/// The material preset rides in the engine color's high byte; C only honors
/// presets in `(Default, Metallic]` and otherwise falls back to the per-bodyType
/// table (`debug_adapter.c` :844-855). We store the *effective* preset (the raw
/// value when 1..=5, else 0) so the TypeScript resolver can branch identically.
fn pack_style(engine_color: u32, body_type: BodyType, transparent_dynamic: bool) -> u32 {
    let rgb = engine_color & STYLE_COLOR_MASK;

    let raw_preset = (engine_color >> 24) & 0xFF;
    let material = if raw_preset > MATERIAL_DEFAULT && raw_preset <= MATERIAL_METALLIC {
        raw_preset
    } else {
        MATERIAL_DEFAULT
    };

    let bt = body_type_index(body_type);

    let mut style = rgb | (material << STYLE_MATERIAL_SHIFT) | (bt << STYLE_BODY_TYPE_SHIFT);

    // debug_adapter.c :858 — non-static dynamic bodies go translucent when the
    // transparent-dynamic view mode is on.
    if transparent_dynamic && body_type == BodyType::Dynamic {
        style |= STYLE_TRANSPARENT_BIT;
    }

    style
}

/// Fill `out` with one packed style word per `VisBody`, in the same order as
/// [`crate::vis::push_poses`], from a single engine draw pass.
///
/// `transparent_dynamic` mirrors the C adapter's transparent-dynamic view toggle
/// (`debug_adapter.c` `SetTransparentDynamic`): when true, dynamic bodies are
/// flagged for alpha 0.5.
///
/// Each `VisBody` is keyed to its body's primary (head) shape. Compound children
/// and multi-primitive bodies that share one physics shape therefore inherit
/// that shape's color, matching the C adapter, where compound children copy the
/// parent shape's resolved color (`debug_adapter.c` `CreateCompoundChild`
/// :498-501). If a body's shape was not drawn (e.g. it has no shape), the color
/// bits are 0 and only the body-type / transparent fields are populated.
pub fn push_shape_styles(
    world: &mut World,
    bodies: &[VisBody],
    transparent_dynamic: bool,
    out: &mut Vec<u32>,
) {
    with_engine_colors(world, |world, colors| {
        out.clear();
        out.reserve(bodies.len());
        for b in bodies {
            let (head_color, body_type) = resolve_body_style(world, colors, b.body_index);
            // A `VisBody` cosmetic color takes precedence over the collapsed
            // head-shape capture. Demos set this to the *same* value they pass as
            // the shape's `custom_color` (see `joint_gear`), so multi-shape bodies
            // — e.g. a Gear Lift gear (saddle-brown disks + slate-gray axle + gray
            // teeth all sharing one physics body) — keep their per-part colors
            // instead of every part inheriting whichever shape happens to be the
            // body's head.
            let engine_color = if b.color != 0 {
                b.color & STYLE_COLOR_MASK
            } else {
                head_color
            };
            out.push(pack_style(engine_color, body_type, transparent_dynamic));
        }
    });
}

/// Convenience wrapper: build the per-`VisBody` style words for one frame,
/// reading the global transparent-dynamic view flag. Keeps the many
/// `*_styles()` wasm exports down to one line each (they differ only in how they
/// reach their world/body list).
pub fn shape_styles(world: &mut World, bodies: &[VisBody]) -> Vec<u32> {
    let mut out = Vec::new();
    push_shape_styles(
        world,
        bodies,
        crate::interact::transparent_dynamic(),
        &mut out,
    );
    out
}

/// Body-index convenience wrapper mirroring [`shape_styles`], for demos whose
/// render list is body indices rather than a `VisBody` slice.
pub fn shape_styles_indexed(
    world: &mut World,
    body_indices: impl IntoIterator<Item = i32>,
) -> Vec<u32> {
    let mut out = Vec::new();
    push_shape_styles_indexed(
        world,
        body_indices,
        crate::interact::transparent_dynamic(),
        &mut out,
    );
    out
}

/// Body-index variant of [`push_shape_styles`] for demos whose render list is not
/// a `VisBody` slice (`sim_demo`'s `SimBody`, `benchmark_demo`/`world_demo`'s own
/// structs). Emits one packed style word per body index, in iteration order — the
/// same order the matching `*_poses()` builder walks its bodies, so the two arrays
/// stay parallel. These render lists carry no cosmetic color, so every word is
/// the engine-captured head-shape color.
pub fn push_shape_styles_indexed(
    world: &mut World,
    body_indices: impl IntoIterator<Item = i32>,
    transparent_dynamic: bool,
    out: &mut Vec<u32>,
) {
    with_engine_colors(world, |world, colors| {
        out.clear();
        for body_index in body_indices {
            let (engine_color, body_type) = resolve_body_style(world, colors, body_index);
            out.push(pack_style(engine_color, body_type, transparent_dynamic));
        }
    });
}

/// Resolve a body's captured head-shape color and body type. Returns
/// `(0, Static)` for an out-of-range or shapeless body.
fn resolve_body_style(
    world: &World,
    colors: &HashMap<u64, u32>,
    body_index: i32,
) -> (u32, BodyType) {
    if body_index >= 0 && (body_index as usize) < world.bodies.len() {
        let body = &world.bodies[body_index as usize];
        let head_shape_id = body.head_shape_id;
        let engine_color = if head_shape_id >= 0 {
            colors
                .get(&((head_shape_id + 1) as u64))
                .copied()
                .unwrap_or(0)
        } else {
            0
        };
        (engine_color, body.type_)
    } else {
        (0, BodyType::Static)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use box3d_rust::body::create_body;
    use box3d_rust::hull::make_box_hull;
    use box3d_rust::math_functions::Pos;
    use box3d_rust::shape::create_hull_shape;
    use box3d_rust::types::{default_body_def, default_shape_def, default_world_def};

    fn add_box(world: &mut World, body: box3d_rust::id::BodyId, hx: f32, hy: f32, hz: f32) {
        let hull = make_box_hull(hx, hy, hz);
        create_hull_shape(world, body, &default_shape_def(), &hull.base);
    }

    #[test]
    fn packs_static_and_dynamic_defaults() {
        // Static default: DARK_GRAY + Matte; dynamic awake: TAN + Soft.
        // (physics_world.c :1269-1291 -> debug_draw material presets.)
        let mut def = default_world_def();
        def.gravity = Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        };
        let mut world = World::new(&def);

        // Static ground.
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        ground_def.position = Pos {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let ground = create_body(&mut world, &ground_def);
        add_box(&mut world, ground, 10.0, 1.0, 10.0);

        // Dynamic body resting above.
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 0.0,
            y: 5.0,
            z: 0.0,
        };
        let dynamic = create_body(&mut world, &body_def);
        add_box(&mut world, dynamic, 0.5, 0.5, 0.5);

        let bodies = vec![
            VisBody::box_body(ground.index1 - 1, 10.0, 1.0, 10.0),
            VisBody::box_body(dynamic.index1 - 1, 0.5, 0.5, 0.5),
        ];

        let mut styles = Vec::new();
        push_shape_styles(&mut world, &bodies, false, &mut styles);
        assert_eq!(styles.len(), 2);

        // Ground: static -> DARK_GRAY (0xA9A9A9), Matte (1), body type 0.
        let g = styles[0];
        assert_eq!(g & STYLE_COLOR_MASK, 0x00A9_A9A9);
        assert_eq!(
            (g >> STYLE_MATERIAL_SHIFT) & STYLE_MATERIAL_MASK,
            1,
            "static ground should be Matte"
        );
        assert_eq!((g >> STYLE_BODY_TYPE_SHIFT) & STYLE_BODY_TYPE_MASK, 0);
        assert_eq!(g & STYLE_TRANSPARENT_BIT, 0);

        // Dynamic (awake): TAN (0xD2B48C), Soft (2), body type 2.
        let d = styles[1];
        assert_eq!(d & STYLE_COLOR_MASK, 0x00D2_B48C);
        assert_eq!(
            (d >> STYLE_MATERIAL_SHIFT) & STYLE_MATERIAL_MASK,
            2,
            "awake dynamic should be Soft"
        );
        assert_eq!((d >> STYLE_BODY_TYPE_SHIFT) & STYLE_BODY_TYPE_MASK, 2);
    }

    #[test]
    fn transparent_flag_only_on_dynamic() {
        let def = default_world_def();
        let mut world = World::new(&def);

        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        let ground = create_body(&mut world, &ground_def);
        add_box(&mut world, ground, 10.0, 1.0, 10.0);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 0.0,
            y: 5.0,
            z: 0.0,
        };
        let dynamic = create_body(&mut world, &body_def);
        add_box(&mut world, dynamic, 0.5, 0.5, 0.5);

        let bodies = vec![
            VisBody::box_body(ground.index1 - 1, 10.0, 1.0, 10.0),
            VisBody::box_body(dynamic.index1 - 1, 0.5, 0.5, 0.5),
        ];

        let mut styles = Vec::new();
        push_shape_styles(&mut world, &bodies, true, &mut styles);

        // Only the dynamic body gets the transparent flag.
        assert_eq!(styles[0] & STYLE_TRANSPARENT_BIT, 0);
        assert_ne!(styles[1] & STYLE_TRANSPARENT_BIT, 0);
    }

    #[test]
    fn vis_body_color_overrides_head_shape() {
        // A VisBody cosmetic color wins over the engine's captured head color, so
        // multi-shape colored bodies (gears) keep their per-part colors.
        let mut def = default_world_def();
        def.gravity = Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        };
        let mut world = World::new(&def);

        // Dynamic body, plain shape (no custom color) → engine would draw TAN.
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 0.0,
            y: 5.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        add_box(&mut world, body, 0.5, 0.5, 0.5);

        // VisBody carries a distinct cosmetic color.
        let mut vis = VisBody::box_body(body.index1 - 1, 0.5, 0.5, 0.5);
        vis.color = 0x00AA_BBCC;
        let bodies = vec![vis];

        let mut styles = Vec::new();
        push_shape_styles(&mut world, &bodies, false, &mut styles);

        assert_eq!(styles[0] & STYLE_COLOR_MASK, 0x00AA_BBCC);
        // Cosmetic colors carry no material preset byte → bodyType fallback.
        assert_eq!((styles[0] >> STYLE_MATERIAL_SHIFT) & STYLE_MATERIAL_MASK, 0);
        assert_eq!(
            (styles[0] >> STYLE_BODY_TYPE_SHIFT) & STYLE_BODY_TYPE_MASK,
            2
        );
    }

    #[test]
    fn custom_color_passes_through() {
        // A shape custom color is emitted verbatim by the engine
        // (physics_world.c :1229) and must survive the round trip.
        let def = default_world_def();
        let mut world = World::new(&def);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        let ground = create_body(&mut world, &body_def);
        let mut sdef = default_shape_def();
        sdef.base_material.custom_color = 0x00_12_34_56;
        let hull = make_box_hull(1.0, 1.0, 1.0);
        create_hull_shape(&mut world, ground, &sdef, &hull.base);

        let bodies = vec![VisBody::box_body(ground.index1 - 1, 1.0, 1.0, 1.0)];
        let mut styles = Vec::new();
        push_shape_styles(&mut world, &bodies, false, &mut styles);

        assert_eq!(styles[0] & STYLE_COLOR_MASK, 0x0012_3456);
        // High byte 0x00 => material Default, falls back to bodyType table.
        assert_eq!((styles[0] >> STYLE_MATERIAL_SHIFT) & STYLE_MATERIAL_MASK, 0);
    }
}
