//! Shapes demos — a 1:1 port of `box3d-cpp-reference/samples/sample_shapes.cpp`.
//!
//! The 12 samples: Inclined Plane, Rolling Resistance, High Resistance, Isotropic
//! Friction, Slide Twist, Restitution, Static Invoke, Conveyor Belt, Conveyor Mesh,
//! Wind, Wind Drop, Wind Flap. Each keeps its own `World` + `VisBody` list and
//! reuses the shared interaction helpers (mouse grab, spawn/delete, debug draw,
//! engine-driven styles) exactly like `joint_demo`.
//!
//! Scene builders live in [`scenes`]; the Conveyor Mesh (its collision mesh, the 7
//! tangent-velocity materials, and the render buffers) lives in [`conveyor`].

mod conveyor;
mod scenes;

use crate::interact::{self, MouseGrab};
use crate::rng::XorShift32;
use crate::shell::ZERO_POS;
use crate::vis::{push_poses, VisBody};
use box3d_rust::body::{body_get_shapes, create_body, destroy_body};
use box3d_rust::id::{BodyId, JointId, ShapeId, NULL_BODY_ID, NULL_JOINT_ID, NULL_SHAPE_ID};
use box3d_rust::math_functions::{
    add, get_length_and_normalize, lerp, mul_sv, sin, Vec3, VEC3_ZERO,
};
use box3d_rust::shape::shape_apply_wind;
use box3d_rust::types::default_world_def;
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<ShapeState>> = const { RefCell::new(None) };
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum ShapeScene {
    InclinedPlane,
    RollingResistance,
    HighResistance,
    IsotropicFriction,
    SlideTwist,
    Restitution,
    StaticInvoke,
    ConveyorBelt,
    ConveyorMesh,
    Wind,
    WindDrop,
    WindFlap,
}

/// Wind shape combo (C `Wind::ShapeType`).
pub(crate) const WIND_SHAPE_SPHERE: u32 = 0;
pub(crate) const WIND_SHAPE_CAPSULE: u32 = 1;
pub(crate) const WIND_SHAPE_BOX: u32 = 2;
/// Max wind bodies (C `Wind::m_maxCount`).
pub(crate) const WIND_MAX_COUNT: i32 = 60;

pub(crate) struct ShapeState {
    pub world: World,
    pub bodies: Vec<VisBody>,
    pub grab: MouseGrab,
    pub scene: ShapeScene,
    pub rng: XorShift32,
    pub step_count: u32,

    // Static Invoke (C StaticInvoke).
    pub invoke: bool,
    pub static_body: BodyId,

    // Wind (C Wind).
    pub wind_shape_type: u32,
    pub wind: Vec3,
    pub drag: f32,
    pub lift: f32,
    pub count: i32,
    pub noise: Vec3,
    pub ground_id: BodyId,
    pub wind_body_ids: Vec<BodyId>,
    /// Last wind arrow endpoints `[x1,y1,z1,x2,y2,z2]` (C `DrawArrow`), or empty.
    pub wind_arrow: Vec<f32>,

    // Wind Drop / Wind Flap (fixed C constants — no UI).
    pub flap_shape1: ShapeId,
    pub flap_shape2: ShapeId,
    pub flap_joint1: JointId,
    pub flap_joint2: JointId,
    pub flap_drop_shape: ShapeId,
    pub time: f32,

    // Conveyor Mesh render buffers (world-space triangles + per-triangle color).
    pub conveyor_tris: Vec<f32>,
    pub conveyor_colors: Vec<u32>,
    pub conveyor_vel_lines: Vec<f32>,
}

impl ShapeState {
    /// Base state with every per-scene field defaulted; builders fill the rest.
    pub(crate) fn base(world: World, scene: ShapeScene) -> Self {
        ShapeState {
            world,
            bodies: Vec::new(),
            grab: MouseGrab::default(),
            scene,
            rng: XorShift32::with_seed(12345),
            step_count: 0,
            invoke: false,
            static_body: NULL_BODY_ID,
            wind_shape_type: WIND_SHAPE_BOX,
            wind: Vec3 {
                x: 6.0,
                y: 0.0,
                z: 0.0,
            },
            drag: 1.0,
            lift: 0.75,
            count: 10,
            noise: VEC3_ZERO,
            ground_id: NULL_BODY_ID,
            wind_body_ids: Vec::new(),
            wind_arrow: Vec::new(),
            flap_shape1: NULL_SHAPE_ID,
            flap_shape2: NULL_SHAPE_ID,
            flap_joint1: NULL_JOINT_ID,
            flap_joint2: NULL_JOINT_ID,
            flap_drop_shape: NULL_SHAPE_ID,
            time: 0.0,
            conveyor_tris: Vec::new(),
            conveyor_colors: Vec::new(),
            conveyor_vel_lines: Vec::new(),
        }
    }
}

pub(crate) fn with_state<R>(f: impl FnOnce(&mut ShapeState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("shapes demo not initialized — call shapes_reset first"))
    })
}

pub(crate) fn new_world() -> World {
    // Restore the base Sample launch-speed scale (5.0) on every scene reset, at the
    // shared `new_world` seam (matching the other demo categories). No Shapes scene
    // overrides the scale, so every builder routes through here.
    interact::reset_launch_speed_scale();
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

/// C `Sample::AddGroundBox(extent)` — a static box at (0,-1,0), half-extents
/// (extent, 1, extent). Returns the ground body id (always render index 0).
pub(crate) fn add_ground_box(world: &mut World, extent: f32) -> BodyId {
    use box3d_rust::hull::make_box_hull;
    use box3d_rust::types::{default_body_def, default_shape_def};
    let mut body_def = default_body_def();
    body_def.position = crate::vis::pos(0.0, -1.0, 0.0);
    let ground = create_body(world, &body_def);
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape_at(world, ground, &default_shape_def(), &hull);
    ground
}

/// Small helper to create a hull shape given a `BoxHull`.
pub(crate) fn create_hull_shape_at(
    world: &mut World,
    body: BodyId,
    shape_def: &box3d_rust::types::ShapeDef,
    hull: &box3d_rust::hull::BoxHull,
) -> ShapeId {
    box3d_rust::shape::create_hull_shape(world, body, shape_def, &hull.base)
}

fn install(state: ShapeState) -> u32 {
    let count = state.bodies.len() as u32;
    STATE.with(|cell| {
        *cell.borrow_mut() = Some(state);
    });
    count
}

/// Reset one of the 11 non-Conveyor-Mesh scenes. The Conveyor Mesh needs its OBJ
/// text (fetched by the page), so it uses [`shapes_reset_conveyor`] instead.
#[wasm_bindgen]
pub fn shapes_reset(scene: u32) -> u32 {
    let state = match scene {
        0 => scenes::build_inclined_plane(),
        1 => scenes::build_rolling_resistance(),
        2 => scenes::build_high_resistance(),
        3 => scenes::build_isotropic_friction(),
        4 => scenes::build_slide_twist(),
        5 => scenes::build_restitution(false),
        6 => scenes::build_static_invoke(),
        7 => scenes::build_conveyor_belt(),
        9 => scenes::build_wind(WIND_SHAPE_BOX, 10),
        10 => scenes::build_wind_drop(),
        11 => scenes::build_wind_flap(),
        _ => scenes::build_inclined_plane(),
    };
    install(state)
}

/// Restitution shape toggle (C `Restitution::DrawControls`): 0 = sphere, 1 = box.
#[wasm_bindgen]
pub fn shapes_reset_restitution(box_shape: bool) -> u32 {
    install(scenes::build_restitution(box_shape))
}

/// Wind rebuild (C `Wind::CreateScene`): rebuild on shape-type / count change.
/// `shape_type`: 0 circle, 1 capsule, 2 box.
#[wasm_bindgen]
pub fn shapes_reset_wind(shape_type: u32, count: i32) -> u32 {
    // C reseeds `g_randomSeed = 12345` only on Sample construction (scene entry);
    // `DrawControls` rebuilds call `CreateScene` but continue the gust-noise
    // stream. So an in-place rebuild (this export) must carry the current RNG
    // forward rather than reset it — only entering Wind via `shapes_reset(9)`
    // starts fresh (`ShapeState::base` seeds 12345).
    let prev_seed = STATE.with(|cell| {
        cell.borrow()
            .as_ref()
            .filter(|s| s.scene == ShapeScene::Wind)
            .map(|s| s.rng.seed())
    });
    let mut state = scenes::build_wind(shape_type, count);
    if let Some(seed) = prev_seed {
        state.rng = XorShift32::with_seed(seed);
    }
    install(state)
}

/// Conveyor Mesh reset (C `ConveyorMesh`), built from the fetched OBJ text.
#[wasm_bindgen]
pub fn shapes_reset_conveyor(obj_text: &str) -> u32 {
    install(conveyor::build_conveyor_mesh(obj_text))
}

// --- Wind live sliders (C `Wind::DrawControls` — no rebuild) ---
#[wasm_bindgen]
pub fn shapes_set_wind_live(wind_x: f32, drag: f32, lift: f32) {
    with_state(|state| {
        state.wind.x = wind_x;
        state.drag = drag;
        state.lift = lift;
    });
}

// --- Static Invoke controls (C `StaticInvoke::DrawControls`) ---
#[wasm_bindgen]
pub fn shapes_set_invoke(invoke: bool) {
    with_state(|state| state.invoke = invoke);
}

#[wasm_bindgen]
pub fn shapes_create_static() {
    with_state(scenes::create_static);
}

#[wasm_bindgen]
pub fn shapes_destroy_static() {
    with_state(|state| {
        if state.static_body.is_non_null() {
            let idx = state.static_body.index1 - 1;
            destroy_body(&mut state.world, state.static_body);
            state.bodies.retain(|b| b.body_index != idx);
            state.static_body = NULL_BODY_ID;
        }
    });
}

#[wasm_bindgen]
pub fn shapes_static_exists() -> bool {
    with_state(|state| state.static_body.is_non_null())
}

/// Per-step wind application for the Wind sample (C `Wind::Step`).
fn step_wind(state: &mut ShapeState) {
    let mut speed = 0.0;
    let direction = get_length_and_normalize(&mut speed, state.wind);
    let wind = mul_sv(speed, add(direction, state.noise));

    for i in 0..state.count as usize {
        if i >= state.wind_body_ids.len() {
            break;
        }
        let body = state.wind_body_ids[i];
        let shapes = body_get_shapes(&state.world, body, 1);
        for &shape in &shapes {
            shape_apply_wind(
                &mut state.world,
                shape,
                wind,
                state.drag,
                state.lift,
                10.0,
                true,
            );
        }
    }

    let rand = state.rng.vec3(
        Vec3 {
            x: -0.3,
            y: -0.3,
            z: -0.3,
        },
        Vec3 {
            x: 0.3,
            y: 0.3,
            z: 0.3,
        },
    );
    state.noise = lerp(state.noise, rand, 0.05);

    // C DrawArrow( {0,0.5,0}, {0,0.5,0} + 0.2*wind, fuchsia ).
    let p1 = Vec3 {
        x: 0.0,
        y: 0.5,
        z: 0.0,
    };
    let p2 = add(p1, mul_sv(0.2, wind));
    state.wind_arrow = vec![p1.x, p1.y, p1.z, p2.x, p2.y, p2.z];
}

#[wasm_bindgen]
pub fn shapes_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.wrapping_add(1);

        match state.scene {
            ShapeScene::StaticInvoke => {
                // C StaticInvoke::Step — create the static body on step 20.
                if state.step_count == 20 {
                    scenes::create_static(state);
                }
            }
            ShapeScene::Wind => step_wind(state),
            ShapeScene::WindDrop => {
                // C WindDrop::Step — wind is zero; only drag/lift act.
                if state.flap_drop_shape.is_non_null() {
                    shape_apply_wind(
                        &mut state.world,
                        state.flap_drop_shape,
                        VEC3_ZERO,
                        state.drag,
                        state.lift,
                        10.0,
                        true,
                    );
                }
            }
            ShapeScene::WindFlap => step_wind_flap(state, dt),
            _ => {}
        }
        state.bodies.len() as u32
    })
}

/// C `WindFlap::Step` — apply wind to both wings and drive the sinusoidal flap.
fn step_wind_flap(state: &mut ShapeState, dt: f32) {
    use box3d_rust::joint::revolute_joint_set_target_angle;
    let max_speed = 10.0;
    let wake = false;
    shape_apply_wind(
        &mut state.world,
        state.flap_shape1,
        VEC3_ZERO,
        state.drag,
        state.lift,
        max_speed,
        wake,
    );
    shape_apply_wind(
        &mut state.world,
        state.flap_shape2,
        VEC3_ZERO,
        state.drag,
        state.lift,
        max_speed,
        wake,
    );

    let angle = sin(10.0 * state.time);
    revolute_joint_set_target_angle(&mut state.world, state.flap_joint1, angle);
    revolute_joint_set_target_angle(&mut state.world, state.flap_joint2, -angle);

    // C: m_time += hertz > 0 ? 1/hertz : 0. The step dt IS 1/hertz, so advance by it.
    state.time += dt;
}

#[wasm_bindgen]
pub fn shapes_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn shapes_styles() -> Vec<u32> {
    with_state(|state| crate::draw_data::shape_styles(&mut state.world, &state.bodies))
}

/// Wind arrow endpoints for the current step (C `Wind::Step` `DrawArrow`), or empty.
#[wasm_bindgen]
pub fn shapes_wind_arrow() -> Vec<f32> {
    with_state(|state| state.wind_arrow.clone())
}

/// Conveyor Mesh render triangles (world space, 9 floats per triangle), or empty.
#[wasm_bindgen]
pub fn shapes_conveyor_mesh() -> Vec<f32> {
    with_state(|state| state.conveyor_tris.clone())
}

/// Per-triangle 0xRRGGBB color parallel to [`shapes_conveyor_mesh`].
#[wasm_bindgen]
pub fn shapes_conveyor_colors() -> Vec<u32> {
    with_state(|state| state.conveyor_colors.clone())
}

/// Conveyor tangent-velocity direction lines (C `ConveyorMesh::Render`), interleaved
/// `[x1,y1,z1,x2,y2,z2]`.
#[wasm_bindgen]
pub fn shapes_conveyor_velocity_lines() -> Vec<f32> {
    with_state(|state| state.conveyor_vel_lines.clone())
}

crate::demo_shell! {
    with_state: with_state,
    state: ShapeState,
    world: world,
    bodies: bodies,
    grab: grab,
    base: |_s| ZERO_POS,
    mouse_down: shapes_mouse_down,
    mouse_move: shapes_mouse_move,
    mouse_up: shapes_mouse_up,
    mouse_active: shapes_mouse_active,
    // Shapes' spawn payload also reports the half-extents + kind so the page can
    // size the instanced mesh; the picker only ever spawns the bullet sphere.
    spawn_random: shapes_spawn_random = |state, spawned| match spawned {
        Some(sp) => {
            let idx = sp.body_index;
            let hx = sp.half_extents[0];
            state.bodies.push(VisBody::sphere_body(idx, hx));
            vec![1.0, idx as f32, hx, hx, hx, sp.kind as f32]
        }
        None => vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    // On delete, clear the Static Invoke handle if it was removed, and prune the
    // Wind chain id list so a stale `BodyId` never reaches `shape_apply_wind`
    // (a reused slot would trip `body/lifecycle.rs`'s validity assert).
    delete_at_ray: shapes_delete_at_ray = |state, index| {
        if state.static_body.is_non_null() && state.static_body.index1 - 1 == index {
            state.static_body = NULL_BODY_ID;
        }
        state.wind_body_ids.retain(|id| id.index1 - 1 != index);
    },
    counters: shapes_counters,
    debug_draw: shapes_debug_draw,
    debug_text: shapes_debug_text,
}

#[wasm_bindgen]
pub fn shapes_body_count() -> u32 {
    with_state(|state| state.bodies.len() as u32)
}
