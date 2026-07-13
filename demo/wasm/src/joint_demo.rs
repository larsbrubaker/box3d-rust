//! Joint demos — Ball and Chain, Revolute, Gear Lift, Driving (sample_joint.cpp).

use crate::interact::{self, MouseGrab};
use crate::joint_drive;
use crate::joint_gear;
use crate::vis::{capsule_x, pos, push_poses, sphere, vec3, VisBody};
use box3d_rust::body::{
    body_get_angular_velocity, body_get_linear_velocity, body_get_mass_data, body_get_world_center,
    create_body,
};
use box3d_rust::height_field::HeightFieldData;
use box3d_rust::hull::make_box_hull;
use box3d_rust::id::{BodyId, JointId, NULL_BODY_ID, NULL_JOINT_ID};
use box3d_rust::joint::{
    create_revolute_joint, create_spherical_joint, joint_wake_bodies, revolute_joint_enable_limit,
    revolute_joint_enable_motor, revolute_joint_enable_spring, revolute_joint_set_limits,
    revolute_joint_set_max_motor_torque, revolute_joint_set_motor_speed,
    revolute_joint_set_spring_damping_ratio, revolute_joint_set_spring_hertz,
    revolute_joint_set_target_angle,
};
use box3d_rust::math_functions::{
    dot, mul_mv, Transform, Vec3, DEG_TO_RAD, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO,
};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_revolute_joint_def, default_shape_def, default_spherical_joint_def,
    default_world_def, BodyType,
};
use box3d_rust::world::{world_get_gravity, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<JointState>> = const { RefCell::new(None) };
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum JointScene {
    Chain,
    Revolute,
    GearLift,
    Driving,
}

pub(crate) struct JointState {
    pub world: World,
    pub bodies: Vec<VisBody>,
    pub grab: MouseGrab,
    pub scene: JointScene,
    /// Revolute hinge or Gear Lift driver.
    pub control_joint: JointId,
    /// Revolute sample plank body, for the energy readout (C Render()).
    pub hinge_body: BodyId,
    pub chassis: BodyId,
    pub front_left: JointId,
    pub front_right: JointId,
    pub rear_left: JointId,
    pub rear_right: JointId,
    pub hf: Option<HeightFieldData>,
    pub hf_origin: Vec3,
    /// Precomputed gear-lift basin wireframe segments `[x0,y0,z0,x1,y1,z1]*N`.
    pub terrain_wire: Vec<f32>,
    pub spin_speed: f32,
    pub throttle_x: f32,
    pub throttle_y: f32,
}

pub(crate) fn with_state<R>(f: impl FnOnce(&mut JointState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("joint demo not initialized — call joint_reset_* first"))
    })
}

pub(crate) fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

pub(crate) fn xf_at(px: f32, py: f32, pz: f32) -> Transform {
    Transform {
        p: vec3(px, py, pz),
        q: QUAT_IDENTITY,
    }
}

pub(crate) fn empty_state(world: World, bodies: Vec<VisBody>, scene: JointScene) -> JointState {
    JointState {
        world,
        bodies,
        grab: MouseGrab::default(),
        scene,
        control_joint: NULL_JOINT_ID,
        hinge_body: NULL_BODY_ID,
        chassis: NULL_BODY_ID,
        front_left: NULL_JOINT_ID,
        front_right: NULL_JOINT_ID,
        rear_left: NULL_JOINT_ID,
        rear_right: NULL_JOINT_ID,
        hf: None,
        hf_origin: VEC3_ZERO,
        terrain_wire: Vec::new(),
        spin_speed: 30.0,
        throttle_x: 0.0,
        throttle_y: 0.0,
    }
}

fn install(state: JointState) -> u32 {
    let count = state.bodies.len() as u32;
    STATE.with(|cell| {
        *cell.borrow_mut() = Some(state);
    });
    count
}

/// Ball-and-chain: spherical-linked capsules with a heavy sphere tip.
///
/// C hard-codes `linkCount = 32` with no control (sample_joint.cpp:1557).
#[wasm_bindgen]
pub fn joint_reset_chain() -> u32 {
    let n = 32u32;
    let mut world = new_world();
    let mut bodies = Vec::new();

    let mut body_def = default_body_def();
    let ground = create_body(&mut world, &body_def);

    let link_radius = 0.125f32;
    let link_extent = 0.5f32;
    let capsule = capsule_x(link_extent, link_radius);
    let shape_def = default_shape_def();

    body_def.type_ = BodyType::Dynamic;
    let mut parent = ground;
    let mut joint_def = default_spherical_joint_def();
    joint_def.base.local_frame_a = TRANSFORM_IDENTITY;
    joint_def.base.local_frame_b = xf_at(-link_extent, 0.0, 0.0);
    joint_def.enable_motor = true;
    joint_def.max_motor_torque = 10.0;

    for i in 0..n {
        body_def.position = pos((1.0 + 2.0 * i as f32) * link_extent, 0.0, 0.0);
        let child = create_body(&mut world, &body_def);
        create_capsule_shape(&mut world, child, &shape_def, &capsule);
        bodies.push(VisBody::capsule_body(child.index1 - 1, &capsule));

        joint_def.base.body_id_a = parent;
        joint_def.base.body_id_b = child;
        create_spherical_joint(&mut world, &joint_def);

        joint_def.base.local_frame_a = xf_at(link_extent, 0.0, 0.0);
        parent = child;
    }

    let sphere_radius = 2.0f32;
    body_def.position = pos(
        (1.0 + 2.0 * n as f32) * link_extent + sphere_radius - link_extent,
        0.0,
        0.0,
    );
    let tip = create_body(&mut world, &body_def);
    let sph = sphere(sphere_radius);
    create_sphere_shape(&mut world, tip, &shape_def, &sph);
    bodies.push(VisBody::sphere_body(tip.index1 - 1, sphere_radius));

    joint_def.base.body_id_a = parent;
    joint_def.base.body_id_b = tip;
    joint_def.base.local_frame_b = xf_at(-sphere_radius, 0.0, 0.0);
    create_spherical_joint(&mut world, &joint_def);

    install(empty_state(world, bodies, JointScene::Chain))
}

/// Revolute hinge matching C `RevoluteJoint` (ground + 0.5×1.5×0.25 plank).
#[wasm_bindgen]
pub fn joint_reset_hinge() -> u32 {
    let mut world = new_world();
    let mut bodies = Vec::new();

    // Visual/collision ground (Sample::AddGroundBox).
    let mut ground_def = default_body_def();
    ground_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut world, &ground_def);
    let shape_def = default_shape_def();
    let ground_hull = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground, &shape_def, &ground_hull.base);
    bodies.push(VisBody::box_body(ground.index1 - 1, 20.0, 1.0, 20.0));

    // Shapeless joint parent body at the same pose (C RevoluteJoint).
    let mut anchor_def = default_body_def();
    anchor_def.position = pos(0.0, -1.0, 0.0);
    let anchor = create_body(&mut world, &anchor_def);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 4.0, 0.0);
    let plank = create_body(&mut world, &body_def);
    let plank_hull = make_box_hull(0.5, 1.5, 0.25);
    create_hull_shape(&mut world, plank, &shape_def, &plank_hull.base);
    bodies.push(VisBody::box_body(plank.index1 - 1, 0.5, 1.5, 0.25));

    let mut joint_def = default_revolute_joint_def();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = plank;
    joint_def.base.local_frame_a = xf_at(0.0, 6.5, 0.0);
    joint_def.base.local_frame_b = xf_at(0.0, 1.5, 0.0);
    joint_def.base.draw_scale = 2.0;
    joint_def.enable_motor = false;
    joint_def.max_motor_torque = 5000.0;
    joint_def.motor_speed = 0.0;
    joint_def.enable_limit = false;
    joint_def.lower_angle = -35.0 * DEG_TO_RAD;
    joint_def.upper_angle = 35.0 * DEG_TO_RAD;
    joint_def.enable_spring = false;
    joint_def.hertz = 2.0;
    joint_def.damping_ratio = 0.7;
    let joint_id = create_revolute_joint(&mut world, &joint_def);

    let mut state = empty_state(world, bodies, JointScene::Revolute);
    state.control_joint = joint_id;
    state.hinge_body = plank;
    install(state)
}

#[wasm_bindgen]
pub fn joint_reset_gear_lift() -> u32 {
    install(joint_gear::build_gear_lift())
}

#[wasm_bindgen]
pub fn joint_reset_driving() -> u32 {
    install(joint_drive::build_driving())
}

/// Revolute / Gear Lift motor + limit + spring controls.
/// `flags`: bit0=limit, bit1=motor, bit2=spring.
#[wasm_bindgen]
pub fn joint_set_revolute_params(
    flags: u32,
    lower_deg: f32,
    upper_deg: f32,
    motor_speed: f32,
    motor_torque: f32,
    hertz: f32,
    damping: f32,
    target_deg: f32,
) {
    with_state(|state| {
        let jid = state.control_joint;
        if !jid.is_non_null() {
            return;
        }
        let enable_limit = flags & 1 != 0;
        let enable_motor = flags & 2 != 0;
        let enable_spring = flags & 4 != 0;
        revolute_joint_enable_limit(&mut state.world, jid, enable_limit);
        revolute_joint_set_limits(
            &mut state.world,
            jid,
            lower_deg * DEG_TO_RAD,
            upper_deg * DEG_TO_RAD,
        );
        revolute_joint_enable_motor(&mut state.world, jid, enable_motor);
        revolute_joint_set_motor_speed(&mut state.world, jid, motor_speed);
        revolute_joint_set_max_motor_torque(&mut state.world, jid, motor_torque);
        revolute_joint_enable_spring(&mut state.world, jid, enable_spring);
        revolute_joint_set_spring_hertz(&mut state.world, jid, hertz);
        revolute_joint_set_spring_damping_ratio(&mut state.world, jid, damping);
        revolute_joint_set_target_angle(&mut state.world, jid, target_deg * DEG_TO_RAD);
        joint_wake_bodies(&mut state.world, jid);
    });
}

/// Legacy motor toggle used by older hinge UI.
#[wasm_bindgen]
pub fn joint_set_motor(enabled: bool, speed: f32, torque: f32) {
    let flags = if enabled { 2 } else { 0 };
    joint_set_revolute_params(flags, -35.0, 35.0, speed, torque, 2.0, 0.7, 0.0);
}

/// Driving throttle: x = forward/back, y = steer (matches C Driving::Step).
#[wasm_bindgen]
pub fn joint_set_drive_input(throttle_x: f32, throttle_y: f32) {
    with_state(|state| {
        state.throttle_x = throttle_x.clamp(-1.0, 1.0);
        state.throttle_y = throttle_y.clamp(-1.0, 1.0);
    });
}

#[wasm_bindgen]
pub fn joint_set_drive_params(spin_speed: f32, max_spin_torque: f32) {
    with_state(|state| {
        state.spin_speed = spin_speed;
        joint_drive::apply_spin_torque(state, max_spin_torque);
    });
}

/// Driving "Suspension" sliders (C DrawControls): lower/upper translation limits,
/// hertz, damping ratio, applied to all four wheels.
#[wasm_bindgen]
pub fn joint_set_driving_suspension(lower: f32, upper: f32, hertz: f32, damping: f32) {
    with_state(|state| joint_drive::set_suspension(state, lower, upper, hertz, damping));
}

/// Driving "Steering" sliders (C DrawControls): hertz, damping ratio, max torque,
/// and lower/upper steering limits in degrees, applied to the two front wheels.
#[wasm_bindgen]
pub fn joint_set_driving_steering(
    hertz: f32,
    damping: f32,
    torque: f32,
    lower_deg: f32,
    upper_deg: f32,
) {
    with_state(|state| {
        joint_drive::set_steering(
            state,
            hertz,
            damping,
            torque,
            lower_deg * DEG_TO_RAD,
            upper_deg * DEG_TO_RAD,
        );
    });
}

/// Driving telemetry HUD (C Render()): see [`joint_drive::telemetry`] for layout.
#[wasm_bindgen]
pub fn joint_drive_telemetry() -> Vec<f32> {
    with_state(|state| joint_drive::telemetry(state))
}

/// Revolute energy readout (C RevoluteJoint::Render, sample_joint.cpp:1155-1170).
/// Returns `[kinetic, potential, total]` for the hanging plank.
#[wasm_bindgen]
pub fn joint_revolute_energy() -> Vec<f32> {
    with_state(|state| {
        if !state.hinge_body.is_non_null() {
            return vec![0.0, 0.0, 0.0];
        }
        let body = state.hinge_body;
        let mass_data = body_get_mass_data(&state.world, body);
        let angular_velocity = body_get_angular_velocity(&state.world, body);
        let linear_velocity = body_get_linear_velocity(&state.world, body);
        let mut kinetic = 0.5
            * dot(
                angular_velocity,
                mul_mv(mass_data.inertia, angular_velocity),
            );
        kinetic += 0.5 * mass_data.mass * dot(linear_velocity, linear_velocity);
        let center = body_get_world_center(&state.world, body);
        let gravity = world_get_gravity(&state.world);
        // C (sample_joint.cpp:1166): `-mass * center.y * gravity.y`, keeping C's exact
        // multiply order `((-mass) * center.y) * gravity.y` and narrowing to f32 only at the
        // final assignment (never `center.y as f32` before the products). The demo always links
        // box3d-rust in single precision, so `center.y` is f32 and this is C's f32 path exactly.
        let potential = (-mass_data.mass * center.y * gravity.y) as f32;
        vec![kinetic, potential, kinetic + potential]
    })
}

#[wasm_bindgen]
pub fn joint_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        if state.scene == JointScene::Driving {
            joint_drive::pre_step(state);
        }
        state.world.step(dt, sub_steps);
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn joint_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

/// Packed engine-driven style words parallel to [`joint_poses`]. Gear Lift and
/// other joint scenes set `custom_color` on their shapes, so those ride through
/// the engine color here.
#[wasm_bindgen]
pub fn joint_styles() -> Vec<u32> {
    with_state(|state| crate::draw_data::shape_styles(&mut state.world, &state.bodies))
}

/// Overlay text labels (mass / sleep / body names / contact + joint labels) as a
/// JSON array. Schema documented on [`crate::interact::collect_debug_text`].
/// Empty (`"[]"`) when no text-relevant view flag is set.
#[wasm_bindgen]
pub fn joint_debug_text() -> String {
    with_state(|state| crate::interact::collect_debug_text(&mut state.world))
}

#[wasm_bindgen]
pub fn joint_body_count() -> u32 {
    with_state(|state| state.bodies.len() as u32)
}

#[wasm_bindgen]
pub fn joint_chassis_pose() -> Vec<f32> {
    with_state(|state| {
        if !state.chassis.is_non_null() {
            return vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        }
        let xf = box3d_rust::body::get_body_transform(&state.world, state.chassis.index1 - 1);
        vec![
            xf.p.x as f32,
            xf.p.y as f32,
            xf.p.z as f32,
            xf.q.v.x,
            xf.q.v.y,
            xf.q.v.z,
            xf.q.s,
        ]
    })
}

#[wasm_bindgen]
pub fn joint_terrain_wireframe() -> Vec<f32> {
    with_state(|state| {
        if !state.terrain_wire.is_empty() {
            state.terrain_wire.clone()
        } else {
            joint_drive::terrain_wireframe(state)
        }
    })
}

#[wasm_bindgen]
pub fn joint_mouse_down(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_state(|state| {
        if state.grab.begin(
            &mut state.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            let p = state.grab.mouse_point;
            vec![1.0, p.x as f32, p.y as f32, p.z as f32]
        } else {
            vec![0.0, 0.0, 0.0, 0.0]
        }
    })
}

#[wasm_bindgen]
pub fn joint_mouse_move(px: f32, py: f32, pz: f32) {
    with_state(|state| {
        state.grab.move_to(interact::pos(px, py, pz));
    });
}

#[wasm_bindgen]
pub fn joint_mouse_up() {
    with_state(|state| {
        state.grab.end(&mut state.world);
    });
}

#[wasm_bindgen]
pub fn joint_mouse_active() -> bool {
    with_state(|state| state.grab.is_active())
}

#[wasm_bindgen]
pub fn joint_spawn_random(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_state(|state| {
        match interact::spawn_random(
            &mut state.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            Some(spawned) => {
                let kind = spawned.kind;
                let hx = spawned.half_extents[0];
                let hy = spawned.half_extents[1];
                let hz = spawned.half_extents[2];
                let idx = spawned.body_index;
                if kind == 1 {
                    state.bodies.push(VisBody::sphere_body(idx, hx));
                } else if kind == 2 {
                    state.bodies.push(VisBody::capsule_body(
                        idx,
                        &box3d_rust::geometry::Capsule {
                            center1: Vec3 {
                                x: 0.0,
                                y: -hy,
                                z: 0.0,
                            },
                            center2: Vec3 {
                                x: 0.0,
                                y: hy,
                                z: 0.0,
                            },
                            radius: hx,
                        },
                    ));
                } else {
                    state.bodies.push(VisBody::box_body(idx, hx, hy, hz));
                }
                vec![1.0, idx as f32, hx, hy, hz, kind as f32]
            }
            None => vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    })
}

#[wasm_bindgen]
pub fn joint_delete_at_ray(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> u32 {
    with_state(|state| {
        let index = interact::delete_at_ray(
            &mut state.world,
            &mut state.grab,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        );
        if index < 0 {
            return 0;
        }
        state.bodies.retain(|b| b.body_index != index);
        1
    })
}

#[wasm_bindgen]
pub fn joint_counters() -> Vec<f32> {
    with_state(|state| interact::counters_with_sleep(&state.world).to_vec())
}

#[wasm_bindgen]
pub fn joint_debug_draw(_flags: u32) -> Vec<f32> {
    with_state(|state| interact::collect_debug_draw(&mut state.world))
}
