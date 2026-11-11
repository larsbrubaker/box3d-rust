//! Arbitrary convex-hull render channel for the Character page.
//!
//! The 16-float `vis` pose stream only knows boxes / spheres / capsules / smooth
//! cylinders, but several Character scenes draw real convex hulls: the Dynamic Mover's
//! 12-sided pillars and its 5-sided launcher (`b3CreateCylinder`), and the Geometric
//! Mover's falling tree (22 stacked 6-sided cylinder hulls on one body). Each
//! [`HullVis`] carries its hull-local fan triangles once (`character_hull_geometry`,
//! fetched by the page on reset) and a live pose + style per frame
//! (`character_hull_poses` / `character_hull_styles`), so the faces drawn are exactly
//! the collision hulls.

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use super::with_state;
use crate::draw_data::{pack_style, push_shape_styles_indexed};
use box3d_rust::body::{body_get_transform, body_is_valid};
use box3d_rust::id::{BodyId, NULL_BODY_ID};
use box3d_rust::math_functions::{WorldTransform, POS_ZERO, QUAT_IDENTITY};
use box3d_rust::types::BodyType;
use wasm_bindgen::prelude::*;

/// One hull mesh: either riding a body (`body_id` non-null, live body transform each
/// frame) or render-only at `transform` (`body_id` null).
pub(crate) struct HullVis {
    pub body_id: BodyId,
    pub transform: WorldTransform,
    /// Hull-local (or body-local, for merged multi-hull bodies) fan triangles.
    pub tris: Vec<f32>,
    /// `0xRRGGBB` for render-only hulls; `0` uses the body's engine style.
    pub color: u32,
    pub visible: bool,
    /// Opacity in `[0, 1]` (render-only hulls; body hulls draw opaque).
    pub alpha: f32,
}

impl HullVis {
    /// A hull drawn with `body_id`'s live transform and engine color.
    pub(crate) fn on_body(body_id: BodyId, tris: Vec<f32>) -> Self {
        HullVis {
            body_id,
            transform: WorldTransform {
                p: POS_ZERO,
                q: QUAT_IDENTITY,
            },
            tris,
            color: 0,
            visible: true,
            alpha: 1.0,
        }
    }

    /// A render-only hull (no body) in a fixed color, hidden until placed.
    pub(crate) fn render_only(tris: Vec<f32>, color: u32) -> Self {
        HullVis {
            body_id: NULL_BODY_ID,
            transform: WorldTransform {
                p: POS_ZERO,
                q: QUAT_IDENTITY,
            },
            tris,
            color,
            visible: false,
            alpha: 1.0,
        }
    }
}

/// Hull geometry: `[count, then per hull: color, float_count, ...tris]`. Fetch once
/// per reset; the list is fixed for a scene's lifetime.
#[wasm_bindgen]
pub fn character_hull_geometry() -> Vec<f32> {
    with_state(|state| {
        let mut out = vec![state.hulls.len() as f32];
        for h in &state.hulls {
            out.push(h.color as f32);
            out.push(h.tris.len() as f32);
            out.extend_from_slice(&h.tris);
        }
        out
    })
}

/// Hull poses, 8 floats per hull: `[px,py,pz, qx,qy,qz,qw, alpha]`; `alpha == 0` hides
/// the mesh (an invisible render-only hull or a destroyed body).
#[wasm_bindgen]
pub fn character_hull_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::with_capacity(state.hulls.len() * 8);
        for h in &state.hulls {
            let (xf, alpha) = if h.body_id.is_non_null() {
                if body_is_valid(&state.world, h.body_id) {
                    (body_get_transform(&state.world, h.body_id), 1.0)
                } else {
                    (h.transform, 0.0)
                }
            } else {
                (h.transform, if h.visible { h.alpha } else { 0.0 })
            };
            out.extend_from_slice(&[
                xf.p.x as f32,
                xf.p.y as f32,
                xf.p.z as f32,
                xf.q.v.x,
                xf.q.v.y,
                xf.q.v.z,
                xf.q.s,
                alpha,
            ]);
        }
        out
    })
}

/// Packed `draw_data` style words, one per hull: the engine color of the hull's body
/// (awake / sleeping / custom color, exactly as `b3World_Draw` emits it), or the fixed
/// color of a render-only hull.
#[wasm_bindgen]
pub fn character_hull_styles() -> Vec<u32> {
    with_state(|state| {
        let body_indices: Vec<i32> = state
            .hulls
            .iter()
            .map(|h| {
                if h.body_id.is_non_null() && body_is_valid(&state.world, h.body_id) {
                    h.body_id.index1 - 1
                } else {
                    -1
                }
            })
            .collect();
        let mut engine = Vec::new();
        push_shape_styles_indexed(
            &mut state.world,
            body_indices.iter().copied(),
            crate::interact::transparent_dynamic(),
            &mut engine,
        );
        state
            .hulls
            .iter()
            .zip(engine)
            .map(|(h, style)| {
                if h.body_id.is_non_null() && body_is_valid(&state.world, h.body_id) {
                    style
                } else {
                    pack_style(h.color, BodyType::Dynamic, false)
                }
            })
            .collect()
    })
}
