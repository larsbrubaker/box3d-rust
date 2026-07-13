//! Far Stack / Far Ragdolls / Far Mesh Drop (`sample_world.cpp` FarStack :122,
//! FarRagdolls :245, FarMeshDrop :308). One scene is active at a time, so all
//! three share a single `FarScene` state and one generic `world_far_*` export
//! set; the page calls the matching `world_far_reset_*` to switch scenes.
//!
//! Every world position is shifted back into the base frame (`sub_pos(p, base)`)
//! before it crosses the wasm boundary — done in `b3Pos` space (f64 under the
//! `double-precision` feature) so the far-out scenes keep full resolution, then
//! truncated to `f32` for the renderer, exactly like C's draw-origin trick.

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::draw_data::shape_styles_indexed;
use crate::interact::{self, MouseGrab};
use crate::rng::XorShift32;
use crate::vis::{capsule_from_body, mesh_triangle_edges_offset, VisBody, POSE_STRIDE};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::hull::make_box_hull;
use box3d_rust::human::{create_human, Human, BONE_COUNT};
use box3d_rust::math_functions::{offset_pos, sub_pos, Pos, Vec3, VEC3_ONE};
use box3d_rust::mesh::{create_grid_mesh, create_wave_mesh};
use box3d_rust::shape::{create_hull_shape, create_mesh_shape};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::{world_get_body_events, World};
use std::cell::{Cell, RefCell};
use wasm_bindgen::prelude::*;

thread_local! {
    static FAR: RefCell<Option<FarScene>> = const { RefCell::new(None) };
    /// XorShift seed for Far Mesh Drop's per-box velocities (`g_randomSeed`,
    /// `stability.c` :48). Reset at every mesh-drop build so the pile is identical
    /// on restart, matching the C sample.
    static RAND_SEED: Cell<u32> = const { Cell::new(3_963_634_789) };
}

struct FarScene {
    world: World,
    /// Renderable bodies in the base frame (dynamic bodies + Far Stack's ground
    /// box). Mesh grounds are drawn from `ground_edges` instead.
    bodies: Vec<VisBody>,
    /// Ground mesh triangle edges, already shifted into the base frame
    /// (`[x0,y0,z0, x1,y1,z1, ...]`). Empty for Far Stack.
    ground_edges: Vec<f32>,
    grab: MouseGrab,
    base: Pos,
    step_count: u32,
    offset_km: f32,
    /// True only for Far Mesh Drop — the sole scene that runs the settle check and
    /// reports a failure line (Far Stack / Far Ragdolls never do).
    is_mesh_drop: bool,
    /// Far Mesh Drop failure latch (`m_failed`, `sample_world.cpp` :282).
    failed: bool,
}

fn with_far<R>(f: impl FnOnce(&mut FarScene) -> R) -> R {
    FAR.with(|cell| {
        let mut slot = cell.borrow_mut();
        let state = slot.as_mut().expect("far scene not initialized");
        f(state)
    })
}

fn new_world() -> World {
    // Restore the base Sample launch-speed scale (5.0) on every scene reset.
    interact::reset_scene_scales();
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

/// XorShift `RandomFloatRange(lo, hi)` (`shared/utils.h` :56) driving Far Mesh
/// Drop's random velocities. Kept bit-identical to the C stream so the pile
/// settles the same way it does at the origin.
fn random_float_range(lo: f32, hi: f32) -> f32 {
    RAND_SEED.with(|seed| {
        let mut rng = XorShift32::with_seed(seed.get());
        let v = rng.range(lo, hi);
        seed.set(rng.seed());
        v
    })
}

fn random_vec3_uniform(lo: f32, hi: f32) -> Vec3 {
    Vec3 {
        x: random_float_range(lo, hi),
        y: random_float_range(lo, hi),
        z: random_float_range(lo, hi),
    }
}

// ---------------------------------------------------------------------------
// Scene builders
// ---------------------------------------------------------------------------

/// Far Stack (`sample_world.cpp` FarStack :122): a 6-box skewed stack on a box
/// ground, at `offset_km` kilometers along +x.
fn build_stack(offset_km: f32) -> FarScene {
    let base = Pos {
        x: (1000.0 * offset_km) as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };
    let mut world = new_world();
    let mut bodies = Vec::new();

    // Ground (b3MakeBoxHull(12, 1, 12) at base + {0,-1,0}).
    let mut body_def = default_body_def();
    body_def.position = offset_pos(
        base,
        Vec3 {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        },
    );
    let ground = create_body(&mut world, &body_def);
    let shape_def = default_shape_def();
    let ground_hull = make_box_hull(12.0, 1.0, 12.0);
    create_hull_shape(&mut world, ground, &shape_def, &ground_hull.base);
    bodies.push(VisBody::box_body(ground.index1 - 1, 12.0, 1.0, 12.0));

    // Six boxes with a small alternating skew (columnCount = 6).
    let column_count = 6;
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let box_shape = default_shape_def();
    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    for i in 0..column_count {
        let skew = 0.02 * if i & 1 != 0 { 1.0 } else { -1.0 };
        box_def.position = offset_pos(
            base,
            Vec3 {
                x: skew,
                y: 0.5 + 1.0 * i as f32,
                z: 0.0,
            },
        );
        let body = create_body(&mut world, &box_def);
        create_hull_shape(&mut world, body, &box_shape, &box_hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 0.5, 0.5));
    }

    FarScene {
        world,
        bodies,
        ground_edges: Vec::new(),
        grab: MouseGrab::default(),
        base,
        step_count: 0,
        offset_km,
        is_mesh_drop: false,
        failed: false,
    }
}

/// Far Ragdolls (`sample_world.cpp` FarRagdolls :245): 20 humans dropped onto a
/// grid mesh 1000 km out.
fn build_ragdolls() -> FarScene {
    let offset_km = 1000.0f32;
    let base = Pos {
        x: (1000.0 * offset_km) as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };
    let mut world = new_world();
    let mut bodies = Vec::new();

    // Ground body + grid mesh (b3CreateGridMesh(20, 20, 1.0, 1, true)) at base+{0,-1,0}.
    let mut body_def = default_body_def();
    body_def.position = offset_pos(
        base,
        Vec3 {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        },
    );
    let ground = create_body(&mut world, &body_def);
    let shape_def = default_shape_def();
    let grid_mesh = create_grid_mesh(20, 20, 1.0, 1, true).expect("grid mesh");
    create_mesh_shape(&mut world, ground, &shape_def, &grid_mesh, VEC3_ONE);
    let ground_edges = mesh_triangle_edges_offset(
        &grid_mesh,
        VEC3_ONE,
        Vec3 {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        },
    );

    let count = 20i32;
    for i in 0..count {
        let offset = Vec3 {
            x: 0.15 * (i as f32 - 0.5 * count as f32),
            y: 2.0 + 0.25 * i as f32,
            z: 0.15 * (0.5 * count as f32 - i as f32),
        };
        let position = offset_pos(base, offset);
        let mut human = Human::default();
        create_human(
            &mut human, &mut world, position, 10.0, 0.5, 0.7, i, 0, false,
        );
        for b in 0..BONE_COUNT {
            let body_id = human.bones[b].body_id;
            if body_id.is_null() {
                continue;
            }
            let body_index = body_id.index1 - 1;
            if let Some(cap) = capsule_from_body(&world, body_index) {
                bodies.push(VisBody::capsule_body(body_index, &cap));
            }
        }
    }

    FarScene {
        world,
        bodies,
        ground_edges,
        grab: MouseGrab::default(),
        base,
        step_count: 0,
        offset_km,
        is_mesh_drop: false,
        failed: false,
    }
}

/// Far Mesh Drop (`sample_world.cpp` FarMeshDrop :308 → `stability.c`
/// CreateMeshDrop): a field of thin boxes rains onto a wave mesh 1000 km out in
/// both x and z.
fn build_mesh_drop() -> FarScene {
    let offset_km = 1000.0f32;
    let base = Pos {
        x: (1000.0 * offset_km) as _,
        y: 0.0 as _,
        z: (1000.0 * offset_km) as _,
    };
    let mut world = new_world();
    let mut bodies = Vec::new();

    // Ground wave mesh (b3CreateWaveMesh(40, 40, 1.0, 0.5, 0.1, 0.2)) at `origin`.
    let mut ground_def = default_body_def();
    ground_def.position = base;
    let ground = create_body(&mut world, &ground_def);
    let wave_mesh = create_wave_mesh(40, 40, 1.0, 0.5, 0.1, 0.2).expect("wave mesh");
    let mut ground_shape = default_shape_def();
    ground_shape.filter.category_bits = 1;
    create_mesh_shape(&mut world, ground, &ground_shape, &wave_mesh, VEC3_ONE);
    let ground_edges = mesh_triangle_edges_offset(
        &wave_mesh,
        VEC3_ONE,
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    );

    // Thin boxes: b3MakeBoxHull(0.02, 0.2, 0.04), rollingResistance 0.1,
    // categoryBits 2 / maskBits 1 (no box-box collisions), random spawn velocities.
    RAND_SEED.with(|c| c.set(3_963_634_789));
    let box_hull = make_box_hull(0.02, 0.2, 0.04);
    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    let mut box_shape = default_shape_def();
    box_shape.base_material.rolling_resistance = 0.1;
    box_shape.filter.category_bits = 2;
    box_shape.filter.mask_bits = 1;

    let grid_count = 32i32;
    for i in 0..grid_count {
        for j in 0..grid_count {
            let linear_velocity = random_vec3_uniform(-1.0, 1.0);
            let angular_velocity = random_vec3_uniform(-5.0, 5.0);
            box_def.position = offset_pos(
                base,
                Vec3 {
                    x: 0.5 * (i as f32 - 0.5 * grid_count as f32),
                    y: 5.0,
                    z: 0.5 * (j as f32 - 0.5 * grid_count as f32),
                },
            );
            box_def.linear_velocity = linear_velocity;
            box_def.angular_velocity = angular_velocity;
            let body = create_body(&mut world, &box_def);
            create_hull_shape(&mut world, body, &box_shape, &box_hull.base);
            bodies.push(VisBody::box_body(body.index1 - 1, 0.02, 0.2, 0.04));
        }
    }

    FarScene {
        world,
        bodies,
        ground_edges,
        grab: MouseGrab::default(),
        base,
        step_count: 0,
        offset_km,
        is_mesh_drop: true,
        failed: false,
    }
}

/// Pack renderable poses in the base frame (16-float `vis` stride). The world
/// position shift happens in `b3Pos` space (`sub_pos`) before the `f32`
/// truncation so far-out scenes keep resolution.
fn push_far_poses(state: &FarScene) -> Vec<f32> {
    let mut out = Vec::with_capacity(state.bodies.len() * POSE_STRIDE);
    for b in &state.bodies {
        let xf = get_body_transform(&state.world, b.body_index);
        let rel = sub_pos(xf.p, state.base);
        out.push(rel.x);
        out.push(rel.y);
        out.push(rel.z);
        out.push(xf.q.v.x);
        out.push(xf.q.v.y);
        out.push(xf.q.v.z);
        out.push(xf.q.s);
        out.extend_from_slice(&b.params);
        out.push(b.kind as f32);
        out.push(b.color as f32);
    }
    out
}

// ---------------------------------------------------------------------------
// Exports (generic — one active scene)
// ---------------------------------------------------------------------------

#[wasm_bindgen]
pub fn world_far_reset_stack(offset_km: f32) -> u32 {
    let scene = build_stack(offset_km);
    let n = scene.bodies.len() as u32;
    FAR.with(|cell| *cell.borrow_mut() = Some(scene));
    n
}

#[wasm_bindgen]
pub fn world_far_reset_ragdolls() -> u32 {
    let scene = build_ragdolls();
    let n = scene.bodies.len() as u32;
    FAR.with(|cell| *cell.borrow_mut() = Some(scene));
    n
}

#[wasm_bindgen]
pub fn world_far_reset_mesh_drop() -> u32 {
    let scene = build_mesh_drop();
    let n = scene.bodies.len() as u32;
    FAR.with(|cell| *cell.borrow_mut() = Some(scene));
    n
}

#[wasm_bindgen]
pub fn world_far_step(dt: f32, sub_steps: i32) -> u32 {
    with_far(|state| {
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.saturating_add(1);
        // Far Mesh Drop failure check: past 300 steps any body still moving means
        // the pile did not settle far from the origin (`sample_world.cpp` :282).
        if state.is_mesh_drop && !state.failed && state.step_count >= 300 {
            let moved = !world_get_body_events(&state.world).is_empty();
            if moved {
                state.failed = true;
            }
        }
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn world_far_step_count() -> u32 {
    with_far(|s| s.step_count)
}

#[wasm_bindgen]
pub fn world_far_poses() -> Vec<f32> {
    with_far(|state| push_far_poses(state))
}

#[wasm_bindgen]
pub fn world_far_styles() -> Vec<u32> {
    with_far(|state| {
        shape_styles_indexed(&mut state.world, state.bodies.iter().map(|b| b.body_index))
    })
}

/// Ground mesh triangle edges in the base frame (empty for Far Stack).
#[wasm_bindgen]
pub fn world_far_ground_wireframe() -> Vec<f32> {
    with_far(|s| s.ground_edges.clone())
}

#[wasm_bindgen]
pub fn world_far_offset_km() -> f32 {
    with_far(|s| s.offset_km)
}

/// Far Mesh Drop failure latch (`m_failed`); always false for the other scenes.
#[wasm_bindgen]
pub fn world_far_mesh_drop_failed() -> bool {
    with_far(|s| s.failed)
}

crate::demo_shell! {
    with_state: with_far,
    state: FarScene,
    world: world,
    bodies: bodies,
    grab: grab,
    base: |s| s.base,
    mouse_down: world_far_mouse_down,
    mouse_move: world_far_mouse_move,
    mouse_up: world_far_mouse_up,
    mouse_active: world_far_mouse_active,
    // Pack sphere params as [r, r, r] (not the default [r, 0, 0]) so the Far Mesh
    // Drop instanced box renderer gives a cube-shaped instance, not a degenerate
    // sliver — matching the Far Pyramid spawn packing. Non-sphere variants use the
    // shared VisBody helper.
    spawn_random: world_far_spawn_random = |state, spawned| {
        for sp in spawned {
            if sp.kind == crate::vis::KIND_SPHERE {
                let r = sp.half_extents[0];
                let mut vb = VisBody::sphere_body(sp.body_index, r);
                vb.params = [r, r, r, 0.0, 0.0, 0.0, 0.0];
                state.bodies.push(vb);
            } else {
                crate::interact::append_spawned_vis(
                    &state.world,
                    &mut state.bodies,
                    std::slice::from_ref(sp),
                );
            }
        }
        match spawned.first() {
            Some(sp) => vec![1.0, sp.body_index as f32],
            None => vec![0.0, 0.0],
        }
    },
    delete_at_ray: world_far_delete_at_ray = |_state, _index| {},
    counters: world_far_counters,
    debug_draw: world_far_debug_draw,
    debug_text: world_far_debug_text,
}

crate::demo_world_toggles! {
    with_state: with_far,
    world: world,
    set_enable_sleep: world_far_set_enable_sleep,
    set_enable_warm_starting: world_far_set_enable_warm_starting,
    set_enable_continuous: world_far_set_enable_continuous,
    set_recycle_distance: world_far_set_recycle_distance,
}
