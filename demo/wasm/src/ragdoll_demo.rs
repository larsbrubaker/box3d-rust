//! Ragdoll demos — the four `sample_ragdoll.cpp` samples driving the ported
//! `create_human` builder:
//!
//! - Box     (RagdollOnBox, :11)  — one human on a ground box (unchanged original).
//! - Mesh    (RagdollOnMesh, :81) — one human on a walled grid mesh, parallel anchors.
//! - Pile    (RagdollPile, :208)  — `e_count` humans scattered on a grid mesh.
//! - Incline (RagdollIncline, :264) — a human slides down two tilted grid grounds,
//!   de-motorized after 2 s.
//!
//! Ground meshes render as a baked world-space wireframe (`ragdoll_ground_wireframe`);
//! walls and the Box ground render as `VisBody` boxes. This module does not use the
//! `demo_shell!` macro — the ragdoll page has no grab/spawn interaction.

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::vis::{capsule_from_body, pos, push_poses, VisBody};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::hull::make_box_hull;
use box3d_rust::human::{
    create_human, human_create_parallel_anchors, human_set_joint_damping_ratio,
    human_set_joint_friction_torque, human_set_joint_spring_hertz, random_vec3, set_random_seed,
    Human, BONE_COUNT,
};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, Transform, Vec3, PI, QUAT_IDENTITY, VEC3_AXIS_Z, VEC3_ONE,
};
use box3d_rust::mesh::{create_grid_mesh, MeshData};
use box3d_rust::shape::{create_hull_shape, create_mesh_shape};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// C `RagdollPile::e_count` (NDEBUG path). The wasm build is compiled `--release`,
/// so this mirrors the release count; 20 humans = 280 bones is comparable to the
/// determinism soak and holds up in serial wasm.
const PILE_COUNT: usize = 20;

/// Scene identifiers (mirrors the four RegisterSample rows).
const SCENE_BOX: u32 = 0;
const SCENE_MESH: u32 = 1;
const SCENE_PILE: u32 = 2;
const SCENE_INCLINE: u32 = 3;

thread_local! {
    static STATE: RefCell<Option<RagdollState>> = const { RefCell::new(None) };
}

struct RagdollState {
    world: World,
    humans: Vec<Human>,
    bodies: Vec<VisBody>,
    /// Baked world-space ground-mesh wireframe (`[x0,y0,z0, x1,y1,z1, ...]`).
    ground_edges: Vec<f32>,
    scene: u32,
    friction: f32,
    hertz: f32,
    damping: f32,
    /// RagdollIncline timer / motor latch (C `m_time`, `m_motorized`).
    time: f32,
    motorized: bool,
}

fn with_state<R>(f: impl FnOnce(&mut RagdollState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("ragdoll not initialized — call ragdoll_reset_scene first"))
    })
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

/// Append every capsule bone of `human` to `bodies` (skips null bones).
fn push_human_bones(world: &World, human: &Human, bodies: &mut Vec<VisBody>) {
    for i in 0..BONE_COUNT {
        let body_id = human.bones[i].body_id;
        if body_id.is_null() {
            continue;
        }
        let body_index = body_id.index1 - 1;
        if let Some(cap) = capsule_from_body(world, body_index) {
            bodies.push(VisBody::capsule_body(body_index, &cap));
        }
    }
}

/// Bake a mesh's triangle edges through a body's full world transform (handles
/// the Incline's rotated/scaled grounds), appending to `edges`.
fn bake_mesh_world(
    world: &World,
    body_index: i32,
    mesh: &MeshData,
    scale: Vec3,
    edges: &mut Vec<f32>,
) {
    let xf = get_body_transform(world, body_index);
    let t = Transform {
        p: Vec3 {
            x: xf.p.x as f32,
            y: xf.p.y as f32,
            z: xf.p.z as f32,
        },
        q: xf.q,
    };
    edges.extend_from_slice(&crate::vis::mesh_triangle_edges_transform(mesh, scale, t));
}

// ---------------------------------------------------------------------------
// Scene builders
// ---------------------------------------------------------------------------

/// RagdollOnBox (:11): a single human on a static ground box, joint defaults
/// friction 5 / hertz 1 / damping 0.7. Byte-identical to the original scene.
fn build_box(state: &mut RagdollState) {
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut state.world, &ground_def);
    let shape_def = default_shape_def();
    let hull = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut state.world, ground, &shape_def, &hull.base);
    state.bodies.push(VisBody::box_body(0, 20.0, 1.0, 20.0));

    let mut human = Human::default();
    create_human(
        &mut human,
        &mut state.world,
        pos(0.0, 2.0, 0.0),
        state.friction,
        state.hertz,
        state.damping,
        1,
        0,
        false,
    );
    push_human_bones(&state.world, &human, &mut state.bodies);
    state.humans.push(human);
}

/// RagdollOnMesh (:81): a human on a 20×20 grid mesh boxed in by four walls,
/// with kinematic parallel anchors. Joint defaults friction 5 / hertz 2 / damping
/// 0.7.
fn build_mesh(state: &mut RagdollState) {
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut state.world, &ground_def);
    let ground_index = ground.index1 - 1;
    let shape_def = default_shape_def();

    let mesh = create_grid_mesh(20, 20, 2.0, 2, true).expect("grid mesh");
    create_mesh_shape(&mut state.world, ground, &shape_def, &mesh, VEC3_ONE);
    bake_mesh_world(
        &state.world,
        ground_index,
        &mesh,
        VEC3_ONE,
        &mut state.ground_edges,
    );

    // Four walls (half-extents, center) on the same ground body.
    let walls: [(Vec3, f32, f32, f32); 4] = [
        (
            Vec3 {
                x: 0.0,
                y: 5.0,
                z: -20.0,
            },
            20.0,
            5.0,
            0.1,
        ),
        (
            Vec3 {
                x: 0.0,
                y: 5.0,
                z: 20.0,
            },
            20.0,
            5.0,
            0.1,
        ),
        (
            Vec3 {
                x: -20.0,
                y: 5.0,
                z: 0.0,
            },
            0.1,
            5.0,
            20.0,
        ),
        (
            Vec3 {
                x: 20.0,
                y: 5.0,
                z: 0.0,
            },
            0.1,
            5.0,
            20.0,
        ),
    ];
    for (wp, hx, hy, hz) in walls {
        let transform = Transform {
            p: wp,
            q: QUAT_IDENTITY,
        };
        let wall_box = box3d_rust::hull::make_transformed_box_hull(hx, hy, hz, transform);
        create_hull_shape(&mut state.world, ground, &shape_def, &wall_box.base);
        state
            .bodies
            .push(VisBody::box_local(ground_index, hx, hy, hz, transform));
    }

    let mut human = Human::default();
    create_human(
        &mut human,
        &mut state.world,
        pos(0.0, 1.0, 0.0),
        state.friction,
        state.hertz,
        state.damping,
        1,
        0,
        false,
    );
    human_create_parallel_anchors(&mut human, &mut state.world);
    push_human_bones(&state.world, &human, &mut state.bodies);
    state.humans.push(human);
}

/// RagdollPile (:208): `PILE_COUNT` humans scattered over a grid-mesh ground at
/// `{0, -1, 0}` via a seeded `RandomVec3` (g_randomSeed = 42), each with
/// `groupIndex = i`, torque 10 / hertz 0.5 / damping 0.7.
fn build_pile(state: &mut RagdollState) {
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut state.world, &ground_def);
    let ground_index = ground.index1 - 1;
    let shape_def = default_shape_def();
    let mesh = create_grid_mesh(20, 20, 1.0, 1, true).expect("grid mesh");
    create_mesh_shape(&mut state.world, ground, &shape_def, &mesh, VEC3_ONE);
    bake_mesh_world(
        &state.world,
        ground_index,
        &mesh,
        VEC3_ONE,
        &mut state.ground_edges,
    );

    // C c52908c seeds g_randomSeed = 42 and scatters each human via RandomVec3.
    // RandomVec3 consumes three RandomFloatRange draws (x, y, z) per iteration even
    // though only x and z are used, so the seeded call order must be preserved.
    set_random_seed(42);
    let a = 0.1 * PILE_COUNT as f32;
    let lower = Vec3 {
        x: -a,
        y: -a,
        z: -a,
    };
    let upper = Vec3 { x: a, y: a, z: a };
    for i in 0..PILE_COUNT {
        let offset = random_vec3(lower, upper);
        let position = pos(offset.x, 2.0, offset.z);
        let mut human = Human::default();
        create_human(
            &mut human,
            &mut state.world,
            position,
            10.0,
            0.5,
            0.7,
            i as i32,
            0,
            false,
        );
        push_human_bones(&state.world, &human, &mut state.bodies);
        state.humans.push(human);
    }
}

/// RagdollIncline (:264): two tilted/scaled grid-mesh grounds; a human dropped at
/// `{-12, 6, 0}` slides down. Motorized (torque 10 / hertz 2) until t > 2 s.
fn build_incline(state: &mut RagdollState) {
    let shape_def = default_shape_def();
    let mesh = create_grid_mesh(4, 4, 2.0, 1, true).expect("grid mesh");

    {
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        ground_def.position = pos(-10.0, 2.0, 0.0);
        ground_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, -0.2 * PI);
        let ground = create_body(&mut state.world, &ground_def);
        create_mesh_shape(&mut state.world, ground, &shape_def, &mesh, VEC3_ONE);
        bake_mesh_world(
            &state.world,
            ground.index1 - 1,
            &mesh,
            VEC3_ONE,
            &mut state.ground_edges,
        );
    }

    {
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        ground_def.position = pos(0.0, 0.0, 0.0);
        let ground = create_body(&mut state.world, &ground_def);
        let scale = Vec3 {
            x: 4.0,
            y: 4.0,
            z: 4.0,
        };
        create_mesh_shape(&mut state.world, ground, &shape_def, &mesh, scale);
        bake_mesh_world(
            &state.world,
            ground.index1 - 1,
            &mesh,
            scale,
            &mut state.ground_edges,
        );
    }

    let mut human = Human::default();
    create_human(
        &mut human,
        &mut state.world,
        pos(-12.0, 6.0, 0.0),
        10.0,
        2.0,
        0.7,
        1,
        0,
        false,
    );
    push_human_bones(&state.world, &human, &mut state.bodies);
    state.humans.push(human);
    state.time = 0.0;
    state.motorized = true;
}

/// Reset to `scene` (0 Box, 1 Mesh, 2 Pile, 3 Incline). Returns the body count.
#[wasm_bindgen]
pub fn ragdoll_reset_scene(scene: u32) -> u32 {
    STATE.with(|cell| {
        // Box/Mesh expose joint sliders; C defaults are friction 5 / damping 0.7,
        // hertz 1 for Box and 2 for Mesh.
        let hertz = if scene == SCENE_MESH { 2.0 } else { 1.0 };
        let mut state = RagdollState {
            world: new_world(),
            humans: Vec::new(),
            bodies: Vec::new(),
            ground_edges: Vec::new(),
            scene,
            friction: 5.0,
            hertz,
            damping: 0.7,
            time: 0.0,
            motorized: true,
        };
        match scene {
            SCENE_MESH => build_mesh(&mut state),
            SCENE_PILE => build_pile(&mut state),
            SCENE_INCLINE => build_incline(&mut state),
            _ => build_box(&mut state),
        }
        let n = state.bodies.len() as u32;
        *cell.borrow_mut() = Some(state);
        n
    })
}

/// Box scene reset (kept for API stability; equivalent to `ragdoll_reset_scene(0)`).
#[wasm_bindgen]
pub fn ragdoll_reset() -> u32 {
    ragdoll_reset_scene(SCENE_BOX)
}

/// Box / Mesh joint sliders (C `DrawControls`). Applies to every human in the
/// scene; no-op for Pile / Incline which have fixed joint parameters.
#[wasm_bindgen]
pub fn ragdoll_set_joint_params(friction: f32, hertz: f32, damping: f32) {
    with_state(|state| {
        state.friction = friction;
        state.hertz = hertz;
        state.damping = damping;
        for human in &mut state.humans {
            if !human.is_spawned {
                continue;
            }
            human_set_joint_friction_torque(human, &mut state.world, friction);
            human_set_joint_spring_hertz(human, &mut state.world, hertz);
            human_set_joint_damping_ratio(human, &mut state.world, damping);
        }
    });
}

#[wasm_bindgen]
pub fn ragdoll_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        // RagdollIncline::Step: de-motorize the human once t > 2 s.
        if state.scene == SCENE_INCLINE && state.time > 2.0 && state.motorized {
            if let Some(human) = state.humans.first_mut() {
                human_set_joint_friction_torque(human, &mut state.world, 0.5);
                human_set_joint_spring_hertz(human, &mut state.world, 0.5);
            }
            state.motorized = false;
        }
        if state.scene == SCENE_INCLINE {
            // C RagdollIncline::Step (sample_ragdoll.cpp:321):
            //   m_time += m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
            // The browser drives the sim at the step rate, so the context hertz is
            // 1/dt. Advance by 1/hertz (mirroring C's ternary), not the raw dt, so
            // the t > 2 s de-motorize fires at the same simulated time as C for any
            // Hertz setting. For the fixed 1/60 step this is bit-identical to dt.
            let hertz = if dt > 0.0 { 1.0 / dt } else { 0.0 };
            state.time += if hertz > 0.0 { 1.0 / hertz } else { 0.0 };
        }
        state.world.step(dt, sub_steps);
        state.bodies.len() as u32
    })
}

/// Pose buffer: 16 floats per body (see `vis` module).
#[wasm_bindgen]
pub fn ragdoll_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

/// Packed engine-driven style words parallel to [`ragdoll_poses`].
#[wasm_bindgen]
pub fn ragdoll_styles() -> Vec<u32> {
    with_state(|state| crate::draw_data::shape_styles(&mut state.world, &state.bodies))
}

#[wasm_bindgen]
pub fn ragdoll_body_count() -> u32 {
    with_state(|state| state.bodies.len() as u32)
}

/// Counters + awake/sleeping dynamic body counts (`counters_with_sleep`), so the
/// ragdolls page can adopt the shared `makeStyleGate`. Layout matches
/// `sim_counters` / `joint_counters`:
/// `[body, shape, contact, joint, island, awake_dynamic, sleeping]`.
#[wasm_bindgen]
pub fn ragdoll_counters() -> Vec<f32> {
    with_state(|state| crate::interact::counters_with_sleep(&state.world).to_vec())
}

/// Baked world-space ground-mesh wireframe for the current scene (empty for Box).
#[wasm_bindgen]
pub fn ragdoll_ground_wireframe() -> Vec<f32> {
    with_state(|state| state.ground_edges.clone())
}
