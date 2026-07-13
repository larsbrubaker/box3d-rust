//! Live World::step demos: Bodies, Stacking, Pyramid, and related scenes.
//! Compound gallery: `sim_compound`. Continuous scenes: `sim_continuous`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::interact::{self, MouseGrab};
use box3d_rust::body::{
    body_get_type, create_body, destroy_body, get_body_transform, is_body_awake, make_body_id,
};
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, get_length_and_normalize, make_quat_from_axis_angle,
    mul_transforms, Pos, Transform, Vec3, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::recording::{start_recording, stop_recording, Recording};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::{
    world_enable_continuous, world_enable_sleeping, world_enable_warm_starting,
    world_set_contact_recycle_distance, World,
};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// Tiny LCG for village prop placement (demo-only; not the C Random* stream).
pub(crate) struct DemoRng(pub(crate) u32);

impl DemoRng {
    pub(crate) fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
        self.0
    }

    pub(crate) fn next_f32(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }

    pub(crate) fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.next_f32()
    }
}

pub(crate) fn capsule_local_from_centers(c1: Vec3, c2: Vec3, radius: f32) -> (Transform, [f32; 3]) {
    let mut dir = Vec3 {
        x: c2.x - c1.x,
        y: c2.y - c1.y,
        z: c2.z - c1.z,
    };
    let mut len = 0.0;
    dir = get_length_and_normalize(&mut len, dir);
    let mid = Vec3 {
        x: 0.5 * (c1.x + c2.x),
        y: 0.5 * (c1.y + c2.y),
        z: 0.5 * (c1.z + c2.z),
    };
    let q = if len > 1e-6 {
        compute_quat_between_unit_vectors(VEC3_AXIS_Y, dir)
    } else {
        QUAT_IDENTITY
    };
    (Transform { p: mid, q }, [radius, 0.5 * len, radius])
}

thread_local! {
    pub(crate) static SIM: RefCell<Option<SimState>> = const { RefCell::new(None) };
}

pub(crate) struct SimBody {
    pub(crate) body_index: i32,
    pub(crate) half_extents: [f32; 3],
    /// 0 = hull box, 1 = sphere (radius in half_extents[0]), 2 = capsule
    pub(crate) kind: u8,
    /// Optional local transform relative to the body (compound children).
    pub(crate) local: Option<Transform>,
}

pub(crate) struct SimState {
    pub(crate) world: World,
    pub(crate) bodies: Vec<SimBody>,
    pub(crate) grab: MouseGrab,
    /// Owns the active recording buffer; world holds a raw pointer into it.
    recording: Option<Box<Recording>>,
    record_start_step: i32,
    step_count: i32,
    /// Bullet vs Stack projectile body index, or -1 when none (C `m_bulletId`).
    pub(crate) bullet_body_index: i32,
    /// Village building instances (compound-local) for Three.js InstancedMesh.
    pub(crate) village_buildings: Vec<crate::village::BuildingInstance>,
    /// Village compound stats readout (C DrawTextLine overlays).
    pub(crate) village_stats: [f32; 7],
    pub(crate) village_ground_index: i32,
}

pub(crate) fn new_sim() -> SimState {
    // Restore the base Sample launch-speed scale (5.0) and the default debug-draw
    // joint/force scales on every scene reset; a scene that overrides either
    // re-applies its value after the reset returns.
    interact::reset_scene_scales();
    SimState {
        world: new_world(),
        bodies: Vec::new(),
        grab: MouseGrab::default(),
        recording: None,
        record_start_step: 0,
        step_count: 0,
        bullet_body_index: -1,
        village_buildings: Vec::new(),
        village_stats: [0.0; 7],
        village_ground_index: -1,
    }
}

pub(crate) fn stop_recording_if_any(sim: &mut SimState) {
    if sim.recording.is_some() {
        stop_recording(&mut sim.world);
        sim.recording = None;
    }
}

pub(crate) fn with_sim<R>(f: impl FnOnce(&mut SimState) -> R) -> R {
    SIM.with(|cell| {
        let mut slot = cell.borrow_mut();
        let sim = slot
            .as_mut()
            .expect("sim not initialized â€” call sim_reset_* first");
        f(sim)
    })
}

pub(crate) fn push_dynamic_box_ex(
    sim: &mut SimState,
    x: f32,
    y: f32,
    z: f32,
    hx: f32,
    hy: f32,
    hz: f32,
    density: f32,
    friction: f32,
    rolling_resistance: f32,
    angular_velocity: Vec3,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    };
    body_def.angular_velocity = angular_velocity;
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = density;
    shape_def.base_material.friction = friction;
    shape_def.base_material.rolling_resistance = rolling_resistance;
    let hull = make_box_hull(hx, hy, hz);
    create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [hx, hy, hz],
        kind: 0,
        local: None,
    });
}

fn push_dynamic_box_locked(sim: &mut SimState, x: f32, y: f32, z: f32, hx: f32, hy: f32, hz: f32) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    };
    body_def.motion_locks.linear_z = true;
    body_def.motion_locks.angular_x = true;
    body_def.motion_locks.angular_y = true;
    let body_id = create_body(&mut sim.world, &body_def);
    // C Pyramid2D uses b3DefaultShapeDef() (density = water); no override.
    let shape_def = default_shape_def();
    let hull = make_box_hull(hx, hy, hz);
    create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [hx, hy, hz],
        kind: 0,
        local: None,
    });
}

/// Push a dynamic sphere using `b3DefaultShapeDef()` (density = water). Only the
/// rolling resistance varies between C samples: Sphere Stack sets 0.1, the
/// Compound samples leave it at the default 0.
pub(crate) fn push_dynamic_sphere(
    sim: &mut SimState,
    x: f32,
    y: f32,
    z: f32,
    radius: f32,
    rolling_resistance: f32,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    };
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.base_material.rolling_resistance = rolling_resistance;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius,
    };
    create_sphere_shape(&mut sim.world, body_id, &shape_def, &sphere);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [radius, radius, radius],
        kind: 1,
        local: None,
    });
}

/// Static ground box matching C `Sample::AddGroundBox(extent)` (sample.cpp:543):
/// a box with half-extents `(extent, 1, extent)` centered at `{0, -1, 0}`, so the
/// top surface sits at y = 0. Callers pass the same `extent` their C sample does.
pub(crate) fn add_ground(sim: &mut SimState, extent: f32) {
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = Pos {
        x: 0.0 as _,
        y: (-1.0) as _,
        z: 0.0 as _,
    };
    let ground = create_body(&mut sim.world, &ground_def);
    let shape_def = default_shape_def();
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape(&mut sim.world, ground, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: ground.index1 - 1,
        half_extents: [extent, 1.0, extent],
        kind: 0,
        local: None,
    });
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

/// Exact Single Box sample: cube half-extents 0.5 at y=0.5, Ï‰y = 10.
#[wasm_bindgen]
pub fn sim_reset_single_box() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 20.0);
        // C SingleBox uses b3DefaultShapeDef(): density = water (1000), friction 0.6.
        push_dynamic_box_ex(
            &mut sim,
            0.0,
            0.5,
            0.0,
            0.5,
            0.5,
            0.5,
            1000.0,
            0.6,
            0.0,
            Vec3 {
                x: 0.0,
                y: 10.0,
                z: 0.0,
            },
        );
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Box Stack sample (`BoxStack`, C `m_size` is a fixed 40 — light in serial wasm).
/// The C sample has no count control, so this takes no argument.
#[wasm_bindgen]
pub fn sim_reset_stacking() -> u32 {
    // C BoxStack builds exactly 40 cubes.
    let n = 40u32;
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 40.0);
        let a = 0.5f32;
        for i in 0..n {
            let y = 1.5 * a + 2.5 * a * i as f32;
            // C BoxStack uses b3DefaultShapeDef() (density = water, friction 0.6)
            // plus rollingResistance = 0.1 on each cube.
            push_dynamic_box_ex(&mut sim, 0.0, y, 0.0, a, a, a, 1000.0, 0.6, 0.1, VEC3_ZERO);
        }
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Pyramid2D stacking (motion-locked to XY plane). C `m_size` is a fixed 12.
/// Matches C layout: `(-10 + 2*column + row) * a` with `a = 1`. The C sample has
/// no size control, so this takes no argument.
#[wasm_bindgen]
pub fn sim_reset_pyramid() -> u32 {
    // C Pyramid2D uses m_size = 12.
    let n = 12i32;
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 40.0);
        let a = 1.0f32;
        for row in 0..n {
            for column in 0..(n - row) {
                let x = (-10.0 + 2.0 * column as f32 + row as f32) * a;
                let y = (1.5 + 2.5 * row as f32) * a;
                push_dynamic_box_locked(&mut sim, x, y, 0.0, a, a, a);
            }
        }
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Sphere Stack sample (`SphereStack`, C builds a fixed 30 spheres).
/// C uses r = 0.5, spacing `y += 3 * r`, and rollingResistance 0.1. The C sample
/// has no count control, so this takes no argument.
#[wasm_bindgen]
pub fn sim_reset_sphere_stack() -> u32 {
    // C SphereStack builds exactly 30 spheres.
    let n = 30u32;
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 15.0);
        let r = 0.5f32;
        let mut y = 1.5 * r;
        for _ in 0..n {
            push_dynamic_sphere(&mut sim, 0.0, y, 0.0, r, 0.1);
            y += 3.0 * r;
        }
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Jenga Stack sample (`JengaStack`, C `m_size` = 40 rows, two bodies per row).
/// `shape_type` selects the C `DrawControls` radio: 0 = Hull (box 2.5Ã—0.25Ã—0.25,
/// rollingResistance 0.01), 1 = Capsule (capsule Â±2.5 on X, radius 0.25,
/// rollingResistance 0.1). Alternating X/Z placement â€” the 3D showcase (no locks).
#[wasm_bindgen]
pub fn sim_reset_jenga(shape_type: u32) -> u32 {
    let capsule_mode = shape_type == 1;
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 60.0);

        // C JengaStack builds m_size = 40 rows.
        let n = 40i32;
        let mut shape_def = default_shape_def();
        shape_def.base_material.rolling_resistance = if capsule_mode { 0.1 } else { 0.01 };
        let hull = make_box_hull(2.5, 0.25, 0.25);
        let capsule = Capsule {
            center1: Vec3 {
                x: -2.5,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: 2.5,
                y: 0.0,
                z: 0.0,
            },
            radius: 0.25,
        };
        // Render orientation for the capsule child (maps geometry-Y to local X).
        let (capsule_local, capsule_half) =
            capsule_local_from_centers(capsule.center1, capsule.center2, capsule.radius);
        let half_pi = 0.5 * std::f32::consts::PI;

        for i in 0..n {
            let alpha = if (i & 1) == 1 { 0.0 } else { half_pi };
            let x = if (i & 1) == 0 { 1.75 } else { 0.0 };
            let z = if (i & 1) == 0 { 0.0 } else { 1.75 };
            let y = 0.5 * i as f32 + 0.25;
            let rotation = make_quat_from_axis_angle(VEC3_AXIS_Y, alpha);

            for &(px, pz) in &[(x, z), (-x, -z)] {
                let mut body_def = default_body_def();
                body_def.type_ = BodyType::Dynamic;
                body_def.position = Pos {
                    x: px as _,
                    y: y as _,
                    z: pz as _,
                };
                body_def.rotation = rotation;
                let body_id = create_body(&mut sim.world, &body_def);
                if capsule_mode {
                    create_capsule_shape(&mut sim.world, body_id, &shape_def, &capsule);
                    sim.bodies.push(SimBody {
                        body_index: body_id.index1 - 1,
                        half_extents: capsule_half,
                        kind: 2,
                        local: Some(capsule_local),
                    });
                } else {
                    create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
                    sim.bodies.push(SimBody {
                        body_index: body_id.index1 - 1,
                        half_extents: [2.5, 0.25, 0.25],
                        kind: 0,
                        local: None,
                    });
                }
            }
        }

        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Advance the simulation. Returns body count.
#[wasm_bindgen]
pub fn sim_step(dt: f32, sub_steps: i32) -> u32 {
    with_sim(|sim| {
        sim.grab.pre_step(&mut sim.world, dt);
        sim.world.step(dt, sub_steps);
        sim.step_count = sim.step_count.saturating_add(1);
        sim.bodies.len() as u32
    })
}

/// Interleaved body poses: for each body
/// `[px, py, pz, qx, qy, qz, qw, hx, hy, hz, kind, body_type, awake]`.
/// `body_type`: 0 static, 1 kinematic, 2 dynamic. `awake`: 1/0.
#[wasm_bindgen]
pub fn sim_body_poses() -> Vec<f32> {
    with_sim(|sim| {
        let mut out = Vec::with_capacity(sim.bodies.len() * 13);
        for b in &sim.bodies {
            let xf = get_body_transform(&sim.world, b.body_index);
            let (px, py, pz, qx, qy, qz, qw) = if let Some(local) = b.local {
                // WorldTransform Ã— local Transform for compound children.
                let parent = Transform {
                    p: Vec3 {
                        x: xf.p.x as f32,
                        y: xf.p.y as f32,
                        z: xf.p.z as f32,
                    },
                    q: xf.q,
                };
                let world = mul_transforms(parent, local);
                (
                    world.p.x,
                    world.p.y,
                    world.p.z,
                    world.q.v.x,
                    world.q.v.y,
                    world.q.v.z,
                    world.q.s,
                )
            } else {
                (
                    xf.p.x as f32,
                    xf.p.y as f32,
                    xf.p.z as f32,
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
            out.push(b.half_extents[0]);
            out.push(b.half_extents[1]);
            out.push(b.half_extents[2]);
            out.push(b.kind as f32);
            let id = make_body_id(&sim.world, b.body_index);
            out.push(body_get_type(&sim.world, id) as u8 as f32);
            out.push(if is_body_awake(&sim.world, b.body_index) {
                1.0
            } else {
                0.0
            });
        }
        out
    })
}

/// Packed engine-driven style words, one per body, parallel to
/// [`sim_body_poses`]. Bit layout in `draw_data`; consumed by the TS
/// `applyShapeStyle` resolver. Colors track the live engine draw state
/// (awake/sleep/fast/bullet/custom), so this is polled every frame.
#[wasm_bindgen]
pub fn sim_body_styles() -> Vec<u32> {
    with_sim(|sim| {
        crate::draw_data::shape_styles_indexed(
            &mut sim.world,
            sim.bodies.iter().map(|b| b.body_index),
        )
    })
}

/// Overlay text labels (mass / sleep / body names / contact + joint labels) as a
/// JSON array. Schema documented on [`interact::collect_debug_text`]. Empty
/// (`"[]"`) when no text-relevant view flag is set.
#[wasm_bindgen]
pub fn sim_debug_text() -> String {
    with_sim(|sim| interact::collect_debug_text(&mut sim.world))
}

/// Body count in the current scene.
#[wasm_bindgen]
pub fn sim_body_count() -> u32 {
    with_sim(|sim| sim.bodies.len() as u32)
}

/// Begin mouse grab along a pick ray.
/// Returns `[grabbed, px, py, pz]` (grabbed is 0/1; point is the hit when grabbed).
#[wasm_bindgen]
pub fn sim_mouse_down(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_sim(|sim| {
        if sim.grab.begin(
            &mut sim.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            let p = sim.grab.mouse_point;
            vec![1.0, p.x, p.y, p.z]
        } else {
            vec![0.0, 0.0, 0.0, 0.0]
        }
    })
}

/// Move the grab target to a world-space point (camera-facing plane from JS).
#[wasm_bindgen]
pub fn sim_mouse_move(px: f32, py: f32, pz: f32) {
    with_sim(|sim| {
        sim.grab.move_to(interact::pos(px, py, pz));
    })
}

/// Release the mouse grab (body keeps velocity â†’ fling).
#[wasm_bindgen]
pub fn sim_mouse_up() {
    with_sim(|sim| {
        sim.grab.end(&mut sim.world);
    })
}

/// True if a grab is active.
#[wasm_bindgen]
pub fn sim_mouse_active() -> bool {
    with_sim(|sim| sim.grab.is_active())
}

/// Shift-click spawn: sphere / cylinder / human along the pick ray.
/// `variant`: 0 = sphere, 1 = cylinder (Ctrl), 2 = human (Alt).
/// Returns `[ok, body_index, hx, hy, hz, kind]` for the first body (ok is 0/1).
#[wasm_bindgen]
pub fn sim_spawn_random(
    ox: f32,
    oy: f32,
    oz: f32,
    tx: f32,
    ty: f32,
    tz: f32,
    variant: u8,
) -> Vec<f32> {
    with_sim(|sim| {
        let spawned = interact::spawn_projectile(
            &mut sim.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
            interact::LaunchVariant::from_u8(variant),
        );
        push_spawned_sim(sim, &spawned);
        interact::spawn_ok_payload(&spawned)
    })
}

/// Append spawned projectile descriptors to the SimBody render list.
fn push_spawned_sim(sim: &mut SimState, spawned: &[interact::SpawnedBody]) {
    use crate::vis::{capsule_from_body, KIND_CAPSULE, KIND_CYLINDER, KIND_SPHERE};
    for sp in spawned {
        match sp.kind {
            KIND_SPHERE => {
                sim.bodies.push(SimBody {
                    body_index: sp.body_index,
                    half_extents: sp.half_extents,
                    kind: 1,
                    local: None,
                });
            }
            KIND_CAPSULE => {
                if let Some(cap) = capsule_from_body(&sim.world, sp.body_index) {
                    let (local, half) =
                        capsule_local_from_centers(cap.center1, cap.center2, cap.radius);
                    sim.bodies.push(SimBody {
                        body_index: sp.body_index,
                        half_extents: half,
                        kind: 2,
                        local: Some(local),
                    });
                }
            }
            KIND_CYLINDER => {
                // Physics hull spans y∈[0, height]; offset the centered cylinder
                // mesh by half-height. SimBody pages that lack a kind-3 mesh path
                // still get a visible stand-in via continuous's cylinder branch /
                // box fallback with non-zero Z extent.
                let local = Transform {
                    p: Vec3 {
                        x: 0.0,
                        y: sp.half_extents[2],
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                };
                sim.bodies.push(SimBody {
                    body_index: sp.body_index,
                    half_extents: [sp.half_extents[0], sp.half_extents[1], sp.half_extents[0]],
                    kind: 3,
                    local: Some(local),
                });
            }
            _ => {
                sim.bodies.push(SimBody {
                    body_index: sp.body_index,
                    half_extents: sp.half_extents,
                    kind: 0,
                    local: None,
                });
            }
        }
    }
}

/// Ctrl-click delete: destroy the dynamic body under the pick ray. Returns 1 on success.
#[wasm_bindgen]
pub fn sim_delete_at_ray(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> u32 {
    with_sim(|sim| {
        let index = interact::delete_at_ray(
            &mut sim.world,
            &mut sim.grab,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        );
        if index < 0 {
            return 0;
        }
        if sim.bullet_body_index == index {
            sim.bullet_body_index = -1;
        }
        sim.bodies.retain(|b| b.body_index != index);
        1
    })
}

/// Counters: `[body, shape, contact, joint, island, awake, sleeping]`
#[wasm_bindgen]
pub fn sim_counters() -> Vec<f32> {
    with_sim(|sim| interact::counters_with_sleep(&sim.world).to_vec())
}

/// Debug-draw overlay geometry for the current global view flags.
/// The `_flags` argument is legacy; the mask now comes from `sim_set_debug_flags`.
#[wasm_bindgen]
pub fn sim_debug_draw(_flags: u32) -> Vec<f32> {
    with_sim(|sim| interact::collect_debug_draw(&mut sim.world))
}

/// Destroy a tracked body by render-list index (unused by UI; available for tests).
#[wasm_bindgen]
pub fn sim_destroy_body_index(body_index: i32) -> u32 {
    with_sim(|sim| {
        sim.grab.end(&mut sim.world);
        if !sim.bodies.iter().any(|b| b.body_index == body_index) {
            return 0;
        }
        let id = make_body_id(&sim.world, body_index);
        destroy_body(&mut sim.world, id);
        if sim.bullet_body_index == body_index {
            sim.bullet_body_index = -1;
        }
        sim.bodies.retain(|b| b.body_index != body_index);
        1
    })
}

/// Solver step count since the last scene reset.
#[wasm_bindgen]
pub fn sim_step_count() -> i32 {
    with_sim(|sim| sim.step_count)
}

/// Enable/disable sleeping (b3World_EnableSleeping).
#[wasm_bindgen]
pub fn sim_set_enable_sleep(flag: bool) {
    with_sim(|sim| world_enable_sleeping(&mut sim.world, flag));
}

/// Enable/disable warm starting (b3World_EnableWarmStarting).
#[wasm_bindgen]
pub fn sim_set_enable_warm_starting(flag: bool) {
    with_sim(|sim| world_enable_warm_starting(&mut sim.world, flag));
}

/// Enable/disable continuous collision (b3World_EnableContinuous).
#[wasm_bindgen]
pub fn sim_set_enable_continuous(flag: bool) {
    with_sim(|sim| world_enable_continuous(&mut sim.world, flag));
}

/// Contact recycle distance in meters (b3World_SetContactRecycleDistance).
#[wasm_bindgen]
pub fn sim_set_recycle_distance(meters: f32) {
    with_sim(|sim| world_set_contact_recycle_distance(&mut sim.world, meters));
}

/// Start recording the sim world into an in-memory `.b3rec` buffer.
#[wasm_bindgen]
pub fn sim_start_recording() {
    with_sim(|sim| {
        stop_recording_if_any(sim);
        let mut rec = Box::new(Recording::new(0));
        start_recording(&mut sim.world, &mut rec);
        sim.record_start_step = sim.step_count;
        sim.recording = Some(rec);
    });
}

/// Stop recording and return the `.b3rec` bytes (empty if not recording).
#[wasm_bindgen]
pub fn sim_stop_recording() -> Vec<u8> {
    with_sim(|sim| {
        if sim.recording.is_none() {
            return Vec::new();
        }
        stop_recording(&mut sim.world);
        let rec = sim.recording.take().expect("recording present");
        rec.data().to_vec()
    })
}

/// True while a recording session is active.
#[wasm_bindgen]
pub fn sim_is_recording() -> bool {
    with_sim(|sim| sim.recording.is_some())
}

/// Step index when the current recording started (0 if idle).
#[wasm_bindgen]
pub fn sim_record_start_step() -> i32 {
    with_sim(|sim| {
        if sim.recording.is_some() {
            sim.record_start_step
        } else {
            0
        }
    })
}
