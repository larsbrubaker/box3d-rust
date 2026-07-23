//! Debug draw interface (`b3DebugDraw`), color palette (`b3HexColor`), and
//! debug-material packing from `types.h` / `types.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::get_length_units_per_meter;
use crate::geometry::ShapeType;
use crate::id::ShapeId;
use crate::math_functions::{Aabb, Pos, Vec3, WorldTransform};
use crate::shape::ShapeGeometry;

/// Color for debug drawing, packed 0xRRGGBB (b3HexColor). C treats the enum as
/// a plain integer (shape custom colors are arbitrary u32 values), so the Rust
/// port is a newtype over u32 with the named palette as associated constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HexColor(pub u32);

impl HexColor {
    pub const ALICE_BLUE: HexColor = HexColor(0xF0F8FF);
    pub const ANTIQUE_WHITE: HexColor = HexColor(0xFAEBD7);
    pub const AQUA: HexColor = HexColor(0x00FFFF);
    pub const AQUAMARINE: HexColor = HexColor(0x7FFFD4);
    pub const AZURE: HexColor = HexColor(0xF0FFFF);
    pub const BEIGE: HexColor = HexColor(0xF5F5DC);
    pub const BISQUE: HexColor = HexColor(0xFFE4C4);
    pub const BLACK: HexColor = HexColor(0x000000);
    pub const BLANCHED_ALMOND: HexColor = HexColor(0xFFEBCD);
    pub const BLUE: HexColor = HexColor(0x0000FF);
    pub const BLUE_VIOLET: HexColor = HexColor(0x8A2BE2);
    pub const BROWN: HexColor = HexColor(0xA52A2A);
    pub const BURLYWOOD: HexColor = HexColor(0xDEB887);
    pub const CADET_BLUE: HexColor = HexColor(0x5F9EA0);
    pub const CHARTREUSE: HexColor = HexColor(0x7FFF00);
    pub const CHOCOLATE: HexColor = HexColor(0xD2691E);
    pub const CORAL: HexColor = HexColor(0xFF7F50);
    pub const CORNFLOWER_BLUE: HexColor = HexColor(0x6495ED);
    pub const CORNSILK: HexColor = HexColor(0xFFF8DC);
    pub const CRIMSON: HexColor = HexColor(0xDC143C);
    pub const CYAN: HexColor = HexColor(0x00FFFF);
    pub const DARK_BLUE: HexColor = HexColor(0x00008B);
    pub const DARK_CYAN: HexColor = HexColor(0x008B8B);
    pub const DARK_GOLDEN_ROD: HexColor = HexColor(0xB8860B);
    pub const DARK_GRAY: HexColor = HexColor(0xA9A9A9);
    pub const DARK_GREEN: HexColor = HexColor(0x006400);
    pub const DARK_KHAKI: HexColor = HexColor(0xBDB76B);
    pub const DARK_MAGENTA: HexColor = HexColor(0x8B008B);
    pub const DARK_OLIVE_GREEN: HexColor = HexColor(0x556B2F);
    pub const DARK_ORANGE: HexColor = HexColor(0xFF8C00);
    pub const DARK_ORCHID: HexColor = HexColor(0x9932CC);
    pub const DARK_RED: HexColor = HexColor(0x8B0000);
    pub const DARK_SALMON: HexColor = HexColor(0xE9967A);
    pub const DARK_SEA_GREEN: HexColor = HexColor(0x8FBC8F);
    pub const DARK_SLATE_BLUE: HexColor = HexColor(0x483D8B);
    pub const DARK_SLATE_GRAY: HexColor = HexColor(0x2F4F4F);
    pub const DARK_TURQUOISE: HexColor = HexColor(0x00CED1);
    pub const DARK_VIOLET: HexColor = HexColor(0x9400D3);
    pub const DEEP_PINK: HexColor = HexColor(0xFF1493);
    pub const DEEP_SKY_BLUE: HexColor = HexColor(0x00BFFF);
    pub const DIM_GRAY: HexColor = HexColor(0x696969);
    pub const DODGER_BLUE: HexColor = HexColor(0x1E90FF);
    pub const FIRE_BRICK: HexColor = HexColor(0xB22222);
    pub const FLORAL_WHITE: HexColor = HexColor(0xFFFAF0);
    pub const FOREST_GREEN: HexColor = HexColor(0x228B22);
    pub const FUCHSIA: HexColor = HexColor(0xFF00FF);
    pub const GAINSBORO: HexColor = HexColor(0xDCDCDC);
    pub const GHOST_WHITE: HexColor = HexColor(0xF8F8FF);
    pub const GOLD: HexColor = HexColor(0xFFD700);
    pub const GOLDEN_ROD: HexColor = HexColor(0xDAA520);
    pub const GRAY: HexColor = HexColor(0x808080);
    pub const GREEN: HexColor = HexColor(0x008000);
    pub const GREEN_YELLOW: HexColor = HexColor(0xADFF2F);
    pub const HONEY_DEW: HexColor = HexColor(0xF0FFF0);
    pub const HOT_PINK: HexColor = HexColor(0xFF69B4);
    pub const INDIAN_RED: HexColor = HexColor(0xCD5C5C);
    pub const INDIGO: HexColor = HexColor(0x4B0082);
    pub const IVORY: HexColor = HexColor(0xFFFFF0);
    pub const KHAKI: HexColor = HexColor(0xF0E68C);
    pub const LAVENDER: HexColor = HexColor(0xE6E6FA);
    pub const LAVENDER_BLUSH: HexColor = HexColor(0xFFF0F5);
    pub const LAWN_GREEN: HexColor = HexColor(0x7CFC00);
    pub const LEMON_CHIFFON: HexColor = HexColor(0xFFFACD);
    pub const LIGHT_BLUE: HexColor = HexColor(0xADD8E6);
    pub const LIGHT_CORAL: HexColor = HexColor(0xF08080);
    pub const LIGHT_CYAN: HexColor = HexColor(0xE0FFFF);
    pub const LIGHT_GOLDEN_ROD_YELLOW: HexColor = HexColor(0xFAFAD2);
    pub const LIGHT_GRAY: HexColor = HexColor(0xD3D3D3);
    pub const LIGHT_GREEN: HexColor = HexColor(0x90EE90);
    pub const LIGHT_PINK: HexColor = HexColor(0xFFB6C1);
    pub const LIGHT_SALMON: HexColor = HexColor(0xFFA07A);
    pub const LIGHT_SEA_GREEN: HexColor = HexColor(0x20B2AA);
    pub const LIGHT_SKY_BLUE: HexColor = HexColor(0x87CEFA);
    pub const LIGHT_SLATE_GRAY: HexColor = HexColor(0x778899);
    pub const LIGHT_STEEL_BLUE: HexColor = HexColor(0xB0C4DE);
    pub const LIGHT_YELLOW: HexColor = HexColor(0xFFFFE0);
    pub const LIME: HexColor = HexColor(0x00FF00);
    pub const LIME_GREEN: HexColor = HexColor(0x32CD32);
    pub const LINEN: HexColor = HexColor(0xFAF0E6);
    pub const MAGENTA: HexColor = HexColor(0xFF00FF);
    pub const MAROON: HexColor = HexColor(0x800000);
    pub const MEDIUM_AQUA_MARINE: HexColor = HexColor(0x66CDAA);
    pub const MEDIUM_BLUE: HexColor = HexColor(0x0000CD);
    pub const MEDIUM_ORCHID: HexColor = HexColor(0xBA55D3);
    pub const MEDIUM_PURPLE: HexColor = HexColor(0x9370DB);
    pub const MEDIUM_SEA_GREEN: HexColor = HexColor(0x3CB371);
    pub const MEDIUM_SLATE_BLUE: HexColor = HexColor(0x7B68EE);
    pub const MEDIUM_SPRING_GREEN: HexColor = HexColor(0x00FA9A);
    pub const MEDIUM_TURQUOISE: HexColor = HexColor(0x48D1CC);
    pub const MEDIUM_VIOLET_RED: HexColor = HexColor(0xC71585);
    pub const MIDNIGHT_BLUE: HexColor = HexColor(0x191970);
    pub const MINT_CREAM: HexColor = HexColor(0xF5FFFA);
    pub const MISTY_ROSE: HexColor = HexColor(0xFFE4E1);
    pub const MOCCASIN: HexColor = HexColor(0xFFE4B5);
    pub const NAVAJO_WHITE: HexColor = HexColor(0xFFDEAD);
    pub const NAVY: HexColor = HexColor(0x000080);
    pub const OLD_LACE: HexColor = HexColor(0xFDF5E6);
    pub const OLIVE: HexColor = HexColor(0x808000);
    pub const OLIVE_DRAB: HexColor = HexColor(0x6B8E23);
    pub const ORANGE: HexColor = HexColor(0xFFA500);
    pub const ORANGE_RED: HexColor = HexColor(0xFF4500);
    pub const ORCHID: HexColor = HexColor(0xDA70D6);
    pub const PALE_GOLDEN_ROD: HexColor = HexColor(0xEEE8AA);
    pub const PALE_GREEN: HexColor = HexColor(0x98FB98);
    pub const PALE_TURQUOISE: HexColor = HexColor(0xAFEEEE);
    pub const PALE_VIOLET_RED: HexColor = HexColor(0xDB7093);
    pub const PAPAYA_WHIP: HexColor = HexColor(0xFFEFD5);
    pub const PEACH_PUFF: HexColor = HexColor(0xFFDAB9);
    pub const PERU: HexColor = HexColor(0xCD853F);
    pub const PINK: HexColor = HexColor(0xFFC0CB);
    pub const PLUM: HexColor = HexColor(0xDDA0DD);
    pub const POWDER_BLUE: HexColor = HexColor(0xB0E0E6);
    pub const PURPLE: HexColor = HexColor(0x800080);
    pub const REBECCA_PURPLE: HexColor = HexColor(0x663399);
    pub const RED: HexColor = HexColor(0xFF0000);
    pub const ROSY_BROWN: HexColor = HexColor(0xBC8F8F);
    pub const ROYAL_BLUE: HexColor = HexColor(0x4169E1);
    pub const SADDLE_BROWN: HexColor = HexColor(0x8B4513);
    pub const SALMON: HexColor = HexColor(0xFA8072);
    pub const SANDY_BROWN: HexColor = HexColor(0xF4A460);
    pub const SEA_GREEN: HexColor = HexColor(0x2E8B57);
    pub const SEA_SHELL: HexColor = HexColor(0xFFF5EE);
    pub const SIENNA: HexColor = HexColor(0xA0522D);
    pub const SILVER: HexColor = HexColor(0xC0C0C0);
    pub const SKY_BLUE: HexColor = HexColor(0x87CEEB);
    pub const SLATE_BLUE: HexColor = HexColor(0x6A5ACD);
    pub const SLATE_GRAY: HexColor = HexColor(0x708090);
    pub const SNOW: HexColor = HexColor(0xFFFAFA);
    pub const SPRING_GREEN: HexColor = HexColor(0x00FF7F);
    pub const STEEL_BLUE: HexColor = HexColor(0x4682B4);
    pub const TAN: HexColor = HexColor(0xD2B48C);
    pub const TEAL: HexColor = HexColor(0x008080);
    pub const THISTLE: HexColor = HexColor(0xD8BFD8);
    pub const TOMATO: HexColor = HexColor(0xFF6347);
    pub const TURQUOISE: HexColor = HexColor(0x40E0D0);
    pub const VIOLET: HexColor = HexColor(0xEE82EE);
    pub const WHEAT: HexColor = HexColor(0xF5DEB3);
    pub const WHITE: HexColor = HexColor(0xFFFFFF);
    pub const WHITE_SMOKE: HexColor = HexColor(0xF5F5F5);
    pub const YELLOW: HexColor = HexColor(0xFFFF00);
    pub const YELLOW_GREEN: HexColor = HexColor(0x9ACD32);
    pub const BOX2D_RED: HexColor = HexColor(0xDC3132);
    pub const BOX2D_BLUE: HexColor = HexColor(0x30AEBF);
    pub const BOX2D_GREEN: HexColor = HexColor(0x8CC924);
    pub const BOX2D_YELLOW: HexColor = HexColor(0xFFEE8C);
}

/// Debug draw material preset packed into the high byte of a color.
/// (`b3DebugMaterial`)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum DebugMaterial {
    Default = 0,
    Matte = 1,
    Soft = 2,
    Dead = 3,
    Glossy = 4,
    Metallic = 5,
}

/// Pack an RGB color with a material preset. (`b3MakeDebugColor`)
pub fn make_debug_color(rgb: HexColor, material: DebugMaterial) -> HexColor {
    HexColor((rgb.0 & 0x00FF_FFFF) | ((material as u32) << 24))
}

/// Geometry snapshot passed to [`CreateDebugShapeCallback`]. (`b3DebugShape`)
#[derive(Debug, Clone, Copy)]
pub struct DebugShape<'a> {
    pub shape_id: ShapeId,
    pub shape_type: ShapeType,
    pub geometry: &'a ShapeGeometry,
}

/// WorldDef / World callback that builds a GPU (or test) shape handle.
/// (`b3CreateDebugShapeCallback`)
pub type CreateDebugShapeCallback = fn(&DebugShape<'_>, u64) -> u64;

/// Destroys a user shape handle when the physics shape is destroyed or changed.
/// (`b3DestroyDebugShapeCallback`)
pub type DestroyDebugShapeCallback = fn(u64, u64);

/// Debug drawing callbacks and options (`b3DebugDraw`).
///
/// Defaults match `b3DefaultDebugDraw`: draw callbacks are no-ops, option flags
/// are off, `drawing_bounds` is a cube of half-extent `100 * lengthUnits`, and
/// force/joint scales are 1.
pub trait DebugDraw {
    /// Draws a previously created user shape. (`DrawShapeFcn`)
    /// Returns whether drawing should continue (unused by `world_draw` today).
    fn draw_shape(&mut self, user_shape: u64, transform: WorldTransform, color: HexColor) -> bool {
        let _ = (user_shape, transform, color);
        true
    }

    /// Draw a line segment. (`DrawSegmentFcn`)
    fn draw_segment(&mut self, p1: Pos, p2: Pos, color: HexColor) {
        let _ = (p1, p2, color);
    }

    /// Draw a transform; choose your own length scale. (`DrawTransformFcn`)
    fn draw_transform(&mut self, transform: WorldTransform) {
        let _ = transform;
    }

    /// Draw a point. (`DrawPointFcn`)
    fn draw_point(&mut self, p: Pos, size: f32, color: HexColor) {
        let _ = (p, size, color);
    }

    /// Draw a sphere. (`DrawSphereFcn`)
    fn draw_sphere(&mut self, p: Pos, radius: f32, color: HexColor, alpha: f32) {
        let _ = (p, radius, color, alpha);
    }

    /// Draw a capsule. (`DrawCapsuleFcn`)
    fn draw_capsule(&mut self, p1: Pos, p2: Pos, radius: f32, color: HexColor, alpha: f32) {
        let _ = (p1, p2, radius, color, alpha);
    }

    /// Draw a bounding box. (`DrawBoundsFcn`)
    fn draw_bounds(&mut self, aabb: Aabb, color: HexColor) {
        let _ = (aabb, color);
    }

    /// Draw an oriented box. (`DrawBoxFcn`)
    fn draw_box(&mut self, extents: Vec3, transform: WorldTransform, color: HexColor) {
        let _ = (extents, transform, color);
    }

    /// Draw a string in world space. (`DrawStringFcn`)
    fn draw_string(&mut self, p: Pos, s: &str, color: HexColor) {
        let _ = (p, s, color);
    }

    /// World bounds used for culling. (`drawingBounds`)
    fn drawing_bounds(&self) -> Aabb {
        let h = 100.0 * get_length_units_per_meter();
        Aabb {
            lower_bound: Vec3 {
                x: -h,
                y: -h,
                z: -h,
            },
            upper_bound: Vec3 { x: h, y: h, z: h },
        }
    }

    /// Scale for drawing forces. (`forceScale`)
    fn force_scale(&self) -> f32 {
        1.0
    }

    /// Global scaling for joint drawing. (`jointScale`)
    fn joint_scale(&self) -> f32 {
        1.0
    }

    fn draw_shapes(&self) -> bool {
        false
    }
    fn draw_joints(&self) -> bool {
        false
    }
    fn draw_joint_extras(&self) -> bool {
        false
    }
    fn draw_bounds_boxes(&self) -> bool {
        false
    }
    fn draw_mass(&self) -> bool {
        false
    }
    fn draw_sleep(&self) -> bool {
        false
    }
    fn draw_body_names(&self) -> bool {
        false
    }
    fn draw_contacts(&self) -> bool {
        false
    }
    /// Draw contact anchor A instead of B. (`drawAnchorA`)
    fn draw_anchor_a(&self) -> bool {
        false
    }
    fn draw_graph_colors(&self) -> bool {
        false
    }
    fn draw_contact_features(&self) -> bool {
        false
    }
    fn draw_contact_normals(&self) -> bool {
        false
    }
    fn draw_contact_forces(&self) -> bool {
        false
    }
    fn draw_islands(&self) -> bool {
        false
    }
}

/// Internal solver debug point drawn at the end of `world_draw`. (`b3DebugPoint`)
#[derive(Debug, Clone, Copy)]
pub struct DebugPoint {
    pub p: Pos,
    pub color: HexColor,
    pub label: i32,
    pub value: f32,
}

impl Default for DebugPoint {
    fn default() -> Self {
        DebugPoint {
            p: Pos::default(),
            color: HexColor::BLACK,
            label: 0,
            value: 0.0,
        }
    }
}

/// Internal solver debug line drawn at the end of `world_draw`. (`b3DebugLine`)
#[derive(Debug, Clone, Copy)]
pub struct DebugLine {
    pub p1: Pos,
    pub p2: Pos,
    pub color: HexColor,
    pub label: i32,
}

impl Default for DebugLine {
    fn default() -> Self {
        DebugLine {
            p1: Pos::default(),
            p2: Pos::default(),
            color: HexColor::BLACK,
            label: 0,
        }
    }
}

/// (`B3_DEBUG_POINT_CAPACITY`)
pub const DEBUG_POINT_CAPACITY: usize = 64;
/// (`B3_DEBUG_LINE_CAPACITY`)
pub const DEBUG_LINE_CAPACITY: usize = 64;
