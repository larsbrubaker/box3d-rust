//! Continuous collision demos, batch 2 — the mesh-heavy and spinning samples that
//! extend the original Thin Wall / Bounce House / Bullet vs Stack trio in
//! `sim_continuous.rs`:
//!
//! - Spinning Stick   (`sample_continuous.cpp` SpinningStick, :149) — [`basic`]
//! - Needle Mesh      (NeedleMesh, :281) — [`basic`]
//! - Mesh Drop        (MeshDrop, :413) — [`mesh_drop`]
//! - Hump Mesh        (HumpMesh, :799) — [`basic`]
//! - Is Fast          (IsFast, :911) — [`basic`]
//! - Stall            (Stall, :998) — [`basic`]
//!
//! These reuse the shared [`crate::sim_demo`] `SimState` / `SIM` infrastructure
//! (grab, spawn, pose packing) exactly like `sim_continuous.rs`; the only extra
//! state is the baked ground-mesh wireframe (static meshes render as line/triangle
//! geometry in JS, not as `SimBody` boxes) plus the Mesh Drop control parameters,
//! all held in [`ContExtra`] here and shared with the two scene submodules.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

mod basic;
mod mesh_drop;

use crate::vis::mesh_triangle_edges_offset;
use box3d_rust::math_functions::{Pos, Vec3, VEC3_ONE};
use box3d_rust::mesh::MeshData;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// Extra per-scene state that `SimState` does not carry: the baked world-space
/// ground-mesh wireframe (`[x0,y0,z0, x1,y1,z1, ...]`) and the Mesh Drop controls.
struct ContExtra {
    ground_edges: Vec<f32>,
    /// Mesh Drop / current wave amplitude (`m_groundAmplitude`, default 0.5).
    md_amplitude: f32,
    /// Mesh Drop / shape type (0 box, 1 capsule, 2 cylinder, 3 sphere).
    md_shape: u32,
    /// Mesh Drop / RNG seed; advanced on each Generate press (C uses `b3GetTicks`).
    md_seed: u32,
    /// Mesh Drop / "Collide" checkbox (`m_collide`, default true). When false, the
    /// projectile shapes get filter category 2 / mask 1 so they cannot collide with
    /// each other (only the ground), matching C `MeshDrop::Generate` (:551).
    md_collide: bool,
}

impl Default for ContExtra {
    fn default() -> Self {
        Self {
            ground_edges: Vec::new(),
            md_amplitude: 0.5,
            md_shape: 0,
            md_seed: 12345,
            md_collide: true,
        }
    }
}

thread_local! {
    static CONT: RefCell<ContExtra> = RefCell::new(ContExtra::default());
}

fn p(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

fn with_extra<R>(f: impl FnOnce(&mut ContExtra) -> R) -> R {
    CONT.with(|c| f(&mut c.borrow_mut()))
}

/// Reset the baked ground wireframe (called at the start of every scene build).
/// Also called by the batch-1 continuous scenes in [`crate::sim_continuous`], which
/// share [`sim_cont_ground_wireframe`] but bake no mesh ground — clearing here keeps
/// the shared export empty for those box-ground scenes instead of leaking a prior
/// mesh scene's stale edges.
pub(crate) fn clear_ground_edges() {
    with_extra(|e| e.ground_edges.clear());
}

/// Append a static mesh's triangle edges (offset into world space) to the baked
/// ground wireframe. Continuous mesh grounds are all axis-aligned (identity
/// rotation), so a translation `offset` reproduces the world transform exactly.
fn bake_ground_mesh(mesh: &MeshData, offset: Vec3) {
    let edges = mesh_triangle_edges_offset(mesh, VEC3_ONE, offset);
    with_extra(|e| e.ground_edges.extend_from_slice(&edges));
}

/// World-space ground-mesh edges (`[x0,y0,z0, x1,y1,z1, ...]`) for the current
/// continuous scene, or empty for the box-only scenes. Static per scene, so JS
/// fetches it once per reset. The current-scene body transforms never move the
/// ground, so no per-frame update is needed.
///
/// Contract: this is **empty until a mesh-ground scene has baked its wireframe**.
/// A fresh module (no reset yet) and every box-ground scene return `[]`; every
/// scene reset clears the buffer first (see [`clear_ground_edges`]). The page must
/// not build a `THREE.BufferGeometry` from an empty buffer — `computeBoundingSphere`
/// on zero positions yields a NaN radius (the reported fresh-load NaN). The floats
/// are always finite; see `mesh_drop_tests`.
#[wasm_bindgen]
pub fn sim_cont_ground_wireframe() -> Vec<f32> {
    with_extra(|e| e.ground_edges.clone())
}
