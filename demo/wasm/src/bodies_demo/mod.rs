//! Faithful C `Bodies` category demos (`box3d-cpp-reference/samples/sample_bodies.cpp`):
//! Body Type, Spinning Book, Gyroscopic Torque, Weeble, Disable, Cast, Kinematic,
//! Lock Mixing, Fixed Rotation. (Gyroscopic Precession is `#if 0` in C and skipped.)
//!
//! Own-state pattern (mirrors `sensor_demo` / `world_demo`): each scene owns a
//! `World` + `VisBody` list; the TS page renders bodies through the shared pose /
//! style stream and reads a per-scene overlay channel for the extra debug geometry
//! the C `Step()` / `Render()` draws (rays, casts, explosion sphere, velocity
//! lines, target markers). The 9 scene constructors live in `bodies_scenes.rs`.

use crate::interact::{self, MouseGrab};
use crate::vis::{push_poses, VisBody};
use box3d_rust::body::{
    body_apply_linear_impulse_to_center, body_cast_ray, body_cast_shape, body_collide_mover,
    body_get_linear_velocity, body_get_local_point_velocity, body_get_position,
    body_get_world_center, body_get_world_point, body_overlap_shape, body_set_linear_velocity,
    body_set_target_transform, BodyPlaneResult,
};
use box3d_rust::distance::ShapeProxy;
use box3d_rust::geometry::Capsule;
use box3d_rust::id::{BodyId, NULL_BODY_ID};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, transform_point, Pos, Transform, Vec3, WorldTransform, PI,
    QUAT_IDENTITY, VEC3_AXIS_Z, VEC3_ZERO,
};
use box3d_rust::types::{default_query_filter, default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

// Box3D debug palette (src/debug_draw.rs HexColor) — the exact colors C draws with.
const C_CYAN: u32 = 0x00_FFFF;
const C_YELLOW: u32 = 0xFF_FF00;
const C_GREEN: u32 = 0x00_8000;
const C_RED: u32 = 0xFF_0000;
const C_WHITE: u32 = 0xFF_FFFF;
const C_BLUE: u32 = 0x00_00FF;
const C_AZURE: u32 = 0xF0_FFFF;
const C_PLUM: u32 = 0xDD_A0DD;
const C_ORANGE: u32 = 0xFF_A500;
const C_GRAY: u32 = 0x80_8080;
const C_AXIS_X: u32 = 0xFF_4444;
const C_AXIS_Y: u32 = 0x44_FF44;
const C_AXIS_Z: u32 = 0x44_44FF;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SceneKind {
    BodyType,
    SpinningBook,
    Gyroscopic,
    Weeble,
    Disable,
    Cast,
    Kinematic,
    LockMixing,
    FixedRotation,
}

/// Per-step Cast query results (recomputed against `cast_transform` each step),
/// mirroring `BodyCast::Step` (sample_bodies.cpp:733-818).
#[derive(Default, Clone)]
pub struct CastResults {
    pub ray_hit: bool,
    pub ray_point: Vec3,
    pub ray_normal: Vec3,
    pub sphere_hit: bool,
    pub sphere_fraction: f32,
    pub sphere_point: Vec3,
    pub sphere_normal: Vec3,
    pub overlaps: bool,
    /// CollideMover planes: `(point, normal)` world-space, up to 4.
    pub planes: Vec<(Vec3, Vec3)>,
}

/// Kinematic Lissajous target marker (`Kinematic::Step`, sample_bodies.cpp:885-894).
#[derive(Clone, Copy)]
pub struct KinematicTarget {
    pub point: Pos,
    pub axis: Vec3,
}

pub struct BodiesState {
    pub world: World,
    pub vis: Vec<VisBody>,
    pub kind: SceneKind,
    pub grab: MouseGrab,
    pub step_count: u32,

    // Body Type
    pub body_type: BodyType,
    pub is_enabled: bool,
    pub speed: f32,
    pub bt_platform: BodyId,
    pub bt_attach1: BodyId,
    pub bt_attach2: BodyId,
    pub bt_payload2: BodyId,
    pub bt_touching: BodyId,
    pub bt_floating: BodyId,

    // Gyroscopic Torque
    pub gyro_body: BodyId,

    // Weeble
    pub weeble: BodyId,
    pub explosion_position: Pos,
    pub explosion_radius: f32,
    pub explosion_magnitude: f32,

    // Disable
    pub disable_ids: [BodyId; 4],
    pub ball: BodyId,

    // Cast
    pub cast_body: BodyId,
    pub cast_transform: WorldTransform,
    pub cast_cyl_height: f32,
    pub cast_cyl_radius: f32,
    pub cast_tracking: bool,
    pub cast_base: Pos,
    pub cast_origin: Pos,
    pub cast: CastResults,

    // Kinematic
    pub kin_body: BodyId,
    pub kin_amplitude: f32,
    pub kin_time: f32,
    pub kin_target: Option<KinematicTarget>,
}

impl BodiesState {
    /// Base state with all per-scene handles null; scene builders fill in fields.
    pub fn base(world: World, kind: SceneKind) -> Self {
        BodiesState {
            world,
            vis: Vec::new(),
            kind,
            grab: MouseGrab::default(),
            step_count: 0,
            body_type: BodyType::Dynamic,
            is_enabled: true,
            speed: 0.0,
            bt_platform: NULL_BODY_ID,
            bt_attach1: NULL_BODY_ID,
            bt_attach2: NULL_BODY_ID,
            bt_payload2: NULL_BODY_ID,
            bt_touching: NULL_BODY_ID,
            bt_floating: NULL_BODY_ID,
            gyro_body: NULL_BODY_ID,
            weeble: NULL_BODY_ID,
            explosion_position: Pos {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            explosion_radius: 0.0,
            explosion_magnitude: 0.0,
            disable_ids: [NULL_BODY_ID; 4],
            ball: NULL_BODY_ID,
            cast_body: NULL_BODY_ID,
            cast_transform: WorldTransform {
                p: Pos {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                q: QUAT_IDENTITY,
            },
            cast_cyl_height: 0.0,
            cast_cyl_radius: 0.0,
            cast_tracking: false,
            cast_base: Pos {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            cast_origin: Pos {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            cast: CastResults::default(),
            kin_body: NULL_BODY_ID,
            kin_amplitude: 0.0,
            kin_time: 0.0,
            kin_target: None,
        }
    }
}

mod controls;
mod scenes;

thread_local! {
    static STATE: RefCell<Option<BodiesState>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut BodiesState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("bodies not initialized — call bodies_reset first"))
    })
}

/// Standard demo world: gravity (0, -10, 0). (C AddGroundBox worlds use the default.)
pub fn new_world() -> World {
    interact::reset_launch_speed_scale();
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

fn to_transform(t: WorldTransform) -> Transform {
    Transform {
        p: Vec3 {
            x: t.p.x as f32,
            y: t.p.y as f32,
            z: t.p.z as f32,
        },
        q: t.q,
    }
}

// --- Overlay buffer (same layout as interact::collect_debug_draw) --------------
// `[seg_count, point_count, ...segments(7 floats), ...points(5 floats)]`.
#[derive(Default)]
struct Overlay {
    segments: Vec<f32>,
    points: Vec<f32>,
}

impl Overlay {
    fn seg(&mut self, a: Vec3, b: Vec3, color: u32) {
        self.segments
            .extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z, f32::from_bits(color)]);
    }
    fn point(&mut self, p: Vec3, size: f32, color: u32) {
        self.points
            .extend_from_slice(&[p.x, p.y, p.z, size, f32::from_bits(color)]);
    }
    fn into_vec(self) -> Vec<f32> {
        let mut out = Vec::with_capacity(2 + self.segments.len() + self.points.len());
        out.push((self.segments.len() / 7) as f32);
        out.push((self.points.len() / 5) as f32);
        out.extend_from_slice(&self.segments);
        out.extend_from_slice(&self.points);
        out
    }
}

fn v3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn pos_to_v3(p: Pos) -> Vec3 {
    v3(p.x as f32, p.y as f32, p.z as f32)
}

/// Push a wireframe great circle (radius `r`) in the plane spanned by `u`, `v`,
/// centered at `c`, `segs` segments. Used for the Weeble explosion sphere.
fn wire_circle(ov: &mut Overlay, c: Vec3, u: Vec3, v: Vec3, r: f32, segs: i32, color: u32) {
    let mut prev = v3(c.x + r * u.x, c.y + r * u.y, c.z + r * u.z);
    for i in 1..=segs {
        let a = 2.0 * PI * i as f32 / segs as f32;
        let (s, co) = (a.sin(), a.cos());
        let p = v3(
            c.x + r * (co * u.x + s * v.x),
            c.y + r * (co * u.y + s * v.y),
            c.z + r * (co * u.z + s * v.z),
        );
        ov.seg(prev, p, color);
        prev = p;
    }
}

/// Cylinder wireframe (local Y from 0..height, radius `r`) transformed by `xf`.
/// Approximates C `DrawHull` of `b3CreateCylinder` for the Cast target cylinder.
fn wire_cylinder(ov: &mut Overlay, xf: Transform, height: f32, r: f32, sides: i32, color: u32) {
    let ring = |y: f32| -> Vec<Vec3> {
        (0..sides)
            .map(|i| {
                let a = 2.0 * PI * i as f32 / sides as f32;
                transform_point(xf, v3(r * a.cos(), y, r * a.sin()))
            })
            .collect()
    };
    let bottom = ring(0.0);
    let top = ring(height);
    for i in 0..sides as usize {
        let j = (i + 1) % sides as usize;
        ov.seg(bottom[i], bottom[j], color);
        ov.seg(top[i], top[j], color);
        ov.seg(bottom[i], top[i], color);
    }
}

// --- Step ----------------------------------------------------------------------

fn drive_body_type(state: &mut BodiesState) {
    // Reverse the kinematic platform at the track ends (sample_bodies.cpp:238-247).
    if state.body_type != BodyType::Kinematic {
        return;
    }
    let p = body_get_position(&state.world, state.bt_platform);
    let mut v = body_get_linear_velocity(&state.world, state.bt_platform);
    if (p.x < -14.0 && v.x < 0.0) || (p.x > 6.0 && v.x > 0.0) {
        v.x = -v.x;
        body_set_linear_velocity(&mut state.world, state.bt_platform, v);
    }
}

fn drive_kinematic(state: &mut BodiesState, time_step: f32) {
    // Drive the kinematic body along a Lissajous path (sample_bodies.cpp:871-900).
    let delay = 2.0;
    if time_step > 0.0 && state.kin_time > delay {
        let t = state.kin_time - delay;
        let point = Pos {
            x: (2.0 * state.kin_amplitude * t.cos()) as _,
            y: (state.kin_amplitude * ((2.0 * t).sin() + 1.0) + 1.0) as _,
            z: 0.0,
        };
        let rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, 2.0 * t);
        let axis = box3d_rust::math_functions::rotate_vector(rotation, v3(0.0, 1.0, 0.0));
        state.kin_target = Some(KinematicTarget { point, axis });
        body_set_target_transform(
            &mut state.world,
            state.kin_body,
            WorldTransform {
                p: point,
                q: rotation,
            },
            time_step,
            true,
        );
    }
    state.kin_time += time_step;
}

fn recompute_cast(state: &mut BodiesState) {
    let filter = default_query_filter();
    let xf = state.cast_transform;
    let mut res = CastResults::default();

    // Ray cast (sample_bodies.cpp:735-754).
    {
        let origin = Pos {
            x: -9.75,
            y: 3.0,
            z: -4.0,
        };
        let translation = v3(0.0, 0.0, 8.0);
        let r = body_cast_ray(
            &state.world,
            state.cast_body,
            origin,
            translation,
            &filter,
            1.0,
            xf,
        );
        res.ray_hit = r.hit;
        res.ray_point = pos_to_v3(r.point);
        res.ray_normal = r.normal;
    }
    // Sphere cast (sample_bodies.cpp:757-786).
    {
        let origin = Pos {
            x: -14.5,
            y: 2.5,
            z: 0.5,
        };
        let center = VEC3_ZERO;
        let mut points = [VEC3_ZERO; box3d_rust::constants::MAX_SHAPE_CAST_POINTS];
        points[0] = center;
        let proxy = ShapeProxy {
            points,
            count: 1,
            radius: 0.2,
        };
        let translation = v3(8.0, 0.0, 0.0);
        let r = body_cast_shape(
            &state.world,
            state.cast_body,
            origin,
            &proxy,
            translation,
            &filter,
            1.0,
            true,
            xf,
        );
        res.sphere_hit = r.hit;
        res.sphere_fraction = if r.hit { r.fraction } else { 1.0 };
        res.sphere_point = pos_to_v3(r.point);
        res.sphere_normal = r.normal;
    }
    // Overlap capsule (sample_bodies.cpp:789-803).
    {
        let origin = Pos {
            x: -10.0,
            y: 1.0,
            z: 0.5,
        };
        let capsule = Capsule {
            center1: v3(-0.5, 1.0, 0.0),
            center2: v3(0.5, 0.0, 0.0),
            radius: 0.5,
        };
        let mut points = [VEC3_ZERO; box3d_rust::constants::MAX_SHAPE_CAST_POINTS];
        points[0] = capsule.center1;
        points[1] = capsule.center2;
        let proxy = ShapeProxy {
            points,
            count: 2,
            radius: capsule.radius,
        };
        res.overlaps =
            body_overlap_shape(&state.world, state.cast_body, origin, &proxy, &filter, xf);
    }
    // Collide capsule / mover (sample_bodies.cpp:806-818).
    {
        let origin = Pos {
            x: -10.0,
            y: 2.0,
            z: -0.75,
        };
        let capsule = Capsule {
            center1: v3(-0.25, 0.0, 0.0),
            center2: v3(0.25, 1.0, 0.0),
            radius: 0.3,
        };
        let mut planes = [BodyPlaneResult::default(); 4];
        let count = body_collide_mover(
            &state.world,
            state.cast_body,
            &mut planes,
            origin,
            &capsule,
            &filter,
            xf,
        );
        for pr in planes.iter().take(count as usize) {
            res.planes.push((pr.result.point, pr.result.plane.normal));
        }
    }
    state.cast = res;
}

// --- Exports -------------------------------------------------------------------

#[wasm_bindgen]
pub fn bodies_reset(scene: u32) -> u32 {
    STATE.with(|cell| {
        // Preserve the Weeble magnitude slider across restarts of the same scene.
        let prev_mag = cell
            .borrow()
            .as_ref()
            .filter(|s| s.kind == SceneKind::Weeble)
            .map(|s| s.explosion_magnitude);
        let mut state = scenes::build(scene);
        if state.kind == SceneKind::Weeble {
            if let Some(m) = prev_mag {
                state.explosion_magnitude = m;
            }
        }
        let count = state.vis.len() as u32;
        *cell.borrow_mut() = Some(state);
        count
    })
}

#[wasm_bindgen]
pub fn bodies_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        // C `BodyCast::Step` (sample_bodies.cpp:733-819) overrides `Step` and never
        // calls `Sample::Step`, so the world does not advance for the Cast scene —
        // only the ray/shape/overlap/mover queries run (against `m_transform`, which
        // the shift-drag tracking updates). Match that: no grab pre-step, no
        // `world.step`, no step-count bump; just recompute the cast results.
        if state.kind == SceneKind::Cast {
            recompute_cast(state);
            return state.vis.len() as u32;
        }
        match state.kind {
            SceneKind::BodyType => drive_body_type(state),
            SceneKind::Kinematic => drive_kinematic(state, dt),
            SceneKind::Disable => {
                // Nudge the middle link every step (sample_bodies.cpp:636).
                body_apply_linear_impulse_to_center(
                    &mut state.world,
                    state.disable_ids[2],
                    v3(0.0, 0.1, 0.0),
                    true,
                );
            }
            _ => {}
        }
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.wrapping_add(1);
        state.vis.len() as u32
    })
}

#[wasm_bindgen]
pub fn bodies_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.vis, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn bodies_styles() -> Vec<u32> {
    with_state(|state| crate::draw_data::shape_styles(&mut state.world, &state.vis))
}

/// A HUD readout line for the Gyroscopic Torque scene: the body's world center of
/// mass (`DrawTextLine("center %.3g %.3g %.3g")`, sample_bodies.cpp:358-359).
/// Returns `""` for the other scenes.
#[wasm_bindgen]
pub fn bodies_hud() -> String {
    with_state(|state| {
        if state.kind == SceneKind::Gyroscopic {
            let c = body_get_world_center(&state.world, state.gyro_body);
            format!("center {:.3} {:.3} {:.3}", c.x, c.y, c.z)
        } else {
            String::new()
        }
    })
}

/// Per-scene always-on overlay geometry (segments + points), independent of the
/// View-menu debug flags. Layout matches `interact::collect_debug_draw`.
#[wasm_bindgen]
pub fn bodies_overlay() -> Vec<f32> {
    with_state(|state| {
        let mut ov = Overlay::default();
        match state.kind {
            SceneKind::Weeble => weeble_overlay(state, &mut ov),
            SceneKind::Kinematic => kinematic_overlay(state, &mut ov),
            SceneKind::Cast => cast_overlay(state, &mut ov),
            _ => {}
        }
        ov.into_vec()
    })
}

/// Cast solid proxy shapes for the Cast scene, drawn as real meshes on the TS side.
/// Layout: `[count, then per shape: kind, c1x,c1y,c1z, c2x,c2y,c2z, radius, colorBits]`.
/// kind 0 = sphere (center = c1), kind 1 = capsule (endpoints c1, c2).
#[wasm_bindgen]
pub fn bodies_cast_shapes() -> Vec<f32> {
    with_state(|state| {
        if state.kind != SceneKind::Cast {
            return vec![0.0];
        }
        let mut out: Vec<f32> = Vec::new();
        let mut count = 0u32;
        let push = |out: &mut Vec<f32>, kind: f32, c1: Vec3, c2: Vec3, r: f32, color: u32| {
            out.extend_from_slice(&[
                kind,
                c1.x,
                c1.y,
                c1.z,
                c2.x,
                c2.y,
                c2.z,
                r,
                f32::from_bits(color),
            ]);
        };
        // Sphere cast proxy at origin + fraction·translation (radius 0.2).
        {
            let origin = v3(-14.5, 2.5, 0.5);
            let t = v3(8.0, 0.0, 0.0);
            let f = state.cast.sphere_fraction;
            let center = v3(origin.x + f * t.x, origin.y + f * t.y, origin.z + f * t.z);
            let color = if state.cast.sphere_hit {
                C_GREEN
            } else {
                C_WHITE
            };
            push(&mut out, 0.0, center, VEC3_ZERO, 0.2, color);
            count += 1;
        }
        // Overlap capsule (origin -10,1,0.5; green if overlapping else gray).
        {
            let origin = v3(-10.0, 1.0, 0.5);
            let c1 = v3(origin.x - 0.5, origin.y + 1.0, origin.z);
            let c2 = v3(origin.x + 0.5, origin.y, origin.z);
            let color = if state.cast.overlaps { C_GREEN } else { C_GRAY };
            push(&mut out, 1.0, c1, c2, 0.5, color);
            count += 1;
        }
        // Collide-mover capsule (origin -10,2,-0.75; always purple).
        {
            let origin = v3(-10.0, 2.0, -0.75);
            let c1 = v3(origin.x - 0.25, origin.y, origin.z);
            let c2 = v3(origin.x + 0.25, origin.y + 1.0, origin.z);
            push(&mut out, 1.0, c1, c2, 0.3, 0x80_0080);
            count += 1;
        }
        let mut result = vec![count as f32];
        result.extend_from_slice(&out);
        result
    })
}

fn weeble_overlay(state: &BodiesState, ov: &mut Overlay) {
    // Explosion wire sphere (azure) at m_explosionPosition.
    let c = pos_to_v3(state.explosion_position);
    let r = state.explosion_radius;
    wire_circle(ov, c, v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), r, 48, C_AZURE);
    wire_circle(ov, c, v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0), r, 48, C_AZURE);
    wire_circle(ov, c, v3(1.0, 0.0, 0.0), v3(0.0, 0.0, 1.0), r, 48, C_AZURE);

    // Point velocities at localPoint (0,2,0): v1 (red, local), v2 (green, world).
    let local_point = v3(0.0, 2.0, 0.0);
    let world_point = body_get_world_point(&state.world, state.weeble, local_point);
    let v1 = body_get_local_point_velocity(&state.world, state.weeble, local_point);
    let v2 =
        box3d_rust::body::body_get_world_point_velocity(&state.world, state.weeble, world_point);
    let wp = pos_to_v3(world_point);
    let offset = v3(0.05, 0.0, 0.0);
    ov.seg(wp, v3(wp.x + v1.x, wp.y + v1.y, wp.z + v1.z), C_RED);
    ov.seg(
        v3(wp.x + offset.x, wp.y, wp.z),
        v3(wp.x + v2.x + offset.x, wp.y + v2.y, wp.z + v2.z),
        C_GREEN,
    );
}

fn kinematic_overlay(state: &BodiesState, ov: &mut Overlay) {
    if let Some(t) = state.kin_target {
        let p = pos_to_v3(t.point);
        let a = t.axis;
        ov.seg(
            v3(p.x - 0.5 * a.x, p.y - 0.5 * a.y, p.z - 0.5 * a.z),
            v3(p.x + 0.5 * a.x, p.y + 0.5 * a.y, p.z + 0.5 * a.z),
            C_PLUM,
        );
        ov.point(p, 10.0, C_PLUM);
    }
}

fn cast_overlay(state: &BodiesState, ov: &mut Overlay) {
    // Ground axes at (0, 0.1, 0), length 4 (DrawAxes).
    let axes_origin = v3(0.0, 0.1, 0.0);
    ov.seg(axes_origin, v3(4.0, 0.1, 0.0), C_AXIS_X);
    ov.seg(axes_origin, v3(0.0, 4.1, 0.0), C_AXIS_Y);
    ov.seg(axes_origin, v3(0.0, 0.1, 4.0), C_AXIS_Z);

    // Blue target cylinder wireframe at m_transform (DrawHull).
    wire_cylinder(
        ov,
        to_transform(state.cast_transform),
        state.cast_cyl_height,
        state.cast_cyl_radius,
        16,
        C_BLUE,
    );

    // Ray cast: cyan ray, endpoints green/red, hit point + normal yellow.
    let r_origin = v3(-9.75, 3.0, -4.0);
    let r_end = v3(r_origin.x, r_origin.y, r_origin.z + 8.0);
    ov.seg(r_origin, r_end, C_CYAN);
    ov.point(r_origin, 10.0, C_GREEN);
    ov.point(r_end, 10.0, C_RED);
    if state.cast.ray_hit {
        let hp = state.cast.ray_point;
        let n = state.cast.ray_normal;
        ov.seg(
            hp,
            v3(hp.x + 0.2 * n.x, hp.y + 0.2 * n.y, hp.z + 0.2 * n.z),
            C_YELLOW,
        );
        ov.point(hp, 10.0, C_YELLOW);
    }

    // Sphere cast: white ray (sphereCenter = origin, sphere.center zero), endpoints.
    let s_origin = v3(-14.5, 2.5, 0.5);
    let s_end = v3(s_origin.x + 8.0, s_origin.y, s_origin.z);
    ov.seg(s_origin, s_end, C_WHITE);
    ov.point(s_origin, 10.0, C_GREEN);
    ov.point(s_end, 10.0, C_RED);
    if state.cast.sphere_hit {
        let hp = state.cast.sphere_point;
        let n = state.cast.sphere_normal;
        ov.seg(
            hp,
            v3(hp.x + 0.2 * n.x, hp.y + 0.2 * n.y, hp.z + 0.2 * n.z),
            C_YELLOW,
        );
    }

    // CollideMover planes (orange): normal segment + a small point at the contact.
    for (point, normal) in &state.cast.planes {
        ov.seg(
            *point,
            v3(
                point.x + 0.5 * normal.x,
                point.y + 0.5 * normal.y,
                point.z + 0.5 * normal.z,
            ),
            C_ORANGE,
        );
        ov.point(*point, 8.0, C_ORANGE);
    }
}
