//! Shared mouse-grab, spawn/delete, counters, and launch-scale helpers for demo
//! worlds. The debug-draw collector (view-flag mask, draw scales, and the
//! `b3World_Draw` → overlay adapter) lives in the [`draw`] submodule and is
//! re-exported here so `interact::collect_debug_draw` / `interact::debug_flags`
//! keep resolving.
//!
//! Mirrors the C samples' Sample::Mouse* grab (kinematic body + motor joint) and
//! `b3World_GetCounters` diagnostics.

use box3d_rust::body::{
    body_get_local_point, body_get_mass_data, body_get_type, body_is_valid, body_set_awake,
    body_set_target_transform, create_body, destroy_body, is_body_awake,
};
use box3d_rust::geometry::Sphere;
use box3d_rust::hull::create_cylinder;
use box3d_rust::human::{
    create_human, human_set_bullet, human_set_velocity, Human, BONE_COUNT,
};
use box3d_rust::id::{BodyId, JointId, NULL_BODY_ID, NULL_JOINT_ID};
use box3d_rust::joint::{create_motor_joint, destroy_joint, joint_is_valid};
use box3d_rust::math_functions::{
    length, Pos, Transform, Vec3, WorldTransform, QUAT_IDENTITY, VEC3_ZERO,
};
use box3d_rust::shape::{create_hull_shape, create_sphere_shape, shape_get_body};
use box3d_rust::types::{
    default_body_def, default_motor_joint_def, default_query_filter, default_shape_def, BodyType,
};
use box3d_rust::world::{world_cast_ray_closest, world_get_counters, World};
use std::cell::Cell;
use wasm_bindgen::prelude::*;

use crate::vis::{capsule_from_body, VisBody, KIND_CAPSULE, KIND_CYLINDER, KIND_SPHERE};

mod draw;
pub use draw::*;

/// Mouse-grab state for one demo world (C Sample mouse body + motor joint). The
/// grab strength (`m_mouseForceScale`) is not per-instance state: it lives in a
/// thread-local ([`grab_force_scale`]) that resets to the C base default at each
/// world seam, mirroring how [`launch_speed_scale`] tracks `m_launchSpeedScale`.
#[derive(Clone, Copy)]
pub struct MouseGrab {
    pub mouse_body_id: BodyId,
    pub mouse_joint_id: JointId,
    pub mouse_point: Pos,
}

impl Default for MouseGrab {
    fn default() -> Self {
        Self {
            mouse_body_id: NULL_BODY_ID,
            mouse_joint_id: NULL_JOINT_ID,
            mouse_point: Pos {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        }
    }
}

impl MouseGrab {
    /// Clear stale handles after a world reset/destroy.
    #[allow(dead_code)]
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    /// Drop the grab joint/body if either side was destroyed by the sim.
    pub fn validate(&mut self, world: &mut World) {
        if self.mouse_joint_id.is_non_null() && !joint_is_valid(world, self.mouse_joint_id) {
            self.mouse_joint_id = NULL_JOINT_ID;
            if self.mouse_body_id.is_non_null() && body_is_valid(world, self.mouse_body_id) {
                destroy_body(world, self.mouse_body_id);
            }
            self.mouse_body_id = NULL_BODY_ID;
        }
    }

    /// Drive the kinematic mouse body toward the current target (call before step).
    pub fn pre_step(&mut self, world: &mut World, time_step: f32) {
        self.validate(world);
        if self.mouse_body_id.is_non_null()
            && body_is_valid(world, self.mouse_body_id)
            && time_step > 0.0
        {
            let target = WorldTransform {
                p: self.mouse_point,
                q: QUAT_IDENTITY,
            };
            body_set_target_transform(world, self.mouse_body_id, target, time_step, true);
        }
    }

    /// Begin a grab: raycast, create kinematic mouse body + motor joint.
    /// Returns true if a dynamic body was grabbed.
    pub fn begin(&mut self, world: &mut World, origin: Pos, translation: Vec3) -> bool {
        self.end(world);

        let filter = default_query_filter();
        let result = world_cast_ray_closest(world, origin, translation, &filter);
        if !result.hit {
            return false;
        }

        let body_id = shape_get_body(world, result.shape_id);
        if body_get_type(world, body_id) != BodyType::Dynamic {
            return false;
        }

        self.mouse_point = result.point;

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = self.mouse_point;
        body_def.enable_sleep = false;
        self.mouse_body_id = create_body(world, &body_def);

        let mut joint_def = default_motor_joint_def();
        joint_def.base.body_id_a = self.mouse_body_id;
        joint_def.base.body_id_b = body_id;
        joint_def.base.local_frame_b = Transform {
            p: body_get_local_point(world, body_id, result.point),
            q: QUAT_IDENTITY,
        };
        joint_def.linear_hertz = 7.5;
        joint_def.linear_damping_ratio = 1.0;

        let mass_data = body_get_mass_data(world, body_id);
        let g = length(world.gravity);
        let mg = mass_data.mass * g;
        // C `Sample::MouseDown` (:1194): `jointDef.maxSpringForce = m_mouseForceScale * mg`.
        joint_def.max_spring_force = grab_force_scale() * mg;

        if mass_data.mass > 0.0 {
            let trace = mass_data.inertia.cx.x + mass_data.inertia.cy.y + mass_data.inertia.cz.z;
            let lever = (trace / (3.0 * mass_data.mass)).sqrt();
            joint_def.max_velocity_torque = 0.5 * lever * mg;
        }

        self.mouse_joint_id = create_motor_joint(world, &joint_def);
        body_set_awake(world, body_id, true);
        true
    }

    /// Update the grab target to a world-space point (camera-facing plane from JS).
    pub fn move_to(&mut self, point: Pos) {
        if self.mouse_joint_id.is_non_null() {
            self.mouse_point = point;
        }
    }

    /// Release the grab (destroy joint then kinematic body). Body keeps its velocity → fling.
    pub fn end(&mut self, world: &mut World) {
        if self.mouse_joint_id.is_non_null() && joint_is_valid(world, self.mouse_joint_id) {
            destroy_joint(world, self.mouse_joint_id, true);
        }
        if self.mouse_body_id.is_non_null() && body_is_valid(world, self.mouse_body_id) {
            destroy_body(world, self.mouse_body_id);
        }
        self.mouse_joint_id = NULL_JOINT_ID;
        self.mouse_body_id = NULL_BODY_ID;
    }

    pub fn is_active(&self) -> bool {
        self.mouse_joint_id.is_non_null()
    }
}

/// Descriptor for a body the demo renderer should track after spawn.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpawnedBody {
    pub body_index: i32,
    pub half_extents: [f32; 3],
    /// 0 = box, 1 = sphere, 2 = capsule, 3 = cylinder (`VisBody` kinds).
    pub kind: u8,
}

/// Shift-click launch variant (`sample.cpp` :1211-1250). Modifier precedence matches
/// C: Ctrl → cylinder, else Alt → ragdoll, else sphere.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum LaunchVariant {
    Sphere = 0,
    Cylinder = 1,
    Human = 2,
}

impl LaunchVariant {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Cylinder,
            2 => Self::Human,
            _ => Self::Sphere,
        }
    }
}

/// `m_launchSpeedScale` default from the base `Sample` constructor
/// (`sample.cpp` :330). The base ctor sets 5.0; individual samples override it in
/// their own ctor (e.g. Compound Village sets 2.0).
const DEFAULT_LAUNCH_SPEED_SCALE: f32 = 5.0;

/// `m_mouseForceScale` default from the base `Sample` constructor
/// (`sample.cpp` :312). The base ctor sets 100.0; individual samples override it
/// (e.g. Issues / Multiple Prismatic sets 1e6 for a much stronger picker pull,
/// `sample_issues.cpp` :163).
const DEFAULT_GRAB_FORCE_SCALE: f32 = 100.0;

thread_local! {
    /// Current `m_launchSpeedScale`. Matches C semantics: scene resets restore the
    /// base default (via [`reset_launch_speed_scale`]) and a scene that overrides
    /// it re-applies its value after reset (via [`set_launch_speed_scale`] /
    /// [`sim_set_launch_speed_scale`]). Read only at projectile-spawn time.
    static LAUNCH_SPEED_SCALE: Cell<f32> = const { Cell::new(DEFAULT_LAUNCH_SPEED_SCALE) };

    /// Current `m_mouseForceScale`. Same seam semantics as [`LAUNCH_SPEED_SCALE`]:
    /// scene resets restore the base default (via [`reset_grab_force_scale`]) and a
    /// scene that overrides it re-applies its value after reset (via
    /// [`set_grab_force_scale`]). Read only at grab-begin time
    /// ([`MouseGrab::begin`]).
    static GRAB_FORCE_SCALE: Cell<f32> = const { Cell::new(DEFAULT_GRAB_FORCE_SCALE) };
}

/// Current projectile launch-speed scale (`m_launchSpeedScale`).
pub fn launch_speed_scale() -> f32 {
    LAUNCH_SPEED_SCALE.with(|c| c.get())
}

/// Override the active scene's launch-speed scale (a C sample setting
/// `m_launchSpeedScale` in its ctor). Call *after* a scene reset, which restores
/// the default first.
pub fn set_launch_speed_scale(scale: f32) {
    LAUNCH_SPEED_SCALE.with(|c| c.set(scale));
}

/// Restore the base `Sample` default (5.0). Every scene reset calls this so a
/// prior scene's override never leaks across a scene switch; a scene needing a
/// different value re-applies it after reset.
pub fn reset_launch_speed_scale() {
    LAUNCH_SPEED_SCALE.with(|c| c.set(DEFAULT_LAUNCH_SPEED_SCALE));
}

/// Current mouse-grab strength (`m_mouseForceScale`), read at grab-begin time.
pub fn grab_force_scale() -> f32 {
    GRAB_FORCE_SCALE.with(|c| c.get())
}

/// Override the active scene's mouse-grab strength (a C sample setting
/// `m_mouseForceScale` in its ctor, e.g. Issues / Multiple Prismatic). Call
/// *after* a scene reset, which restores the default first.
pub fn set_grab_force_scale(scale: f32) {
    GRAB_FORCE_SCALE.with(|c| c.set(scale));
}

/// Restore the base `Sample` default (100.0). Called from [`reset_scene_scales`]
/// at every world seam so a prior scene's override never leaks across a switch.
pub fn reset_grab_force_scale() {
    GRAB_FORCE_SCALE.with(|c| c.set(DEFAULT_GRAB_FORCE_SCALE));
}

/// Reset every per-scene scale that lives in a thread-local — the projectile
/// launch-speed scale and the debug-draw joint/force scales — back to its base
/// default. Called at each world-construction seam (`new_world` / `new_sim` /
/// scene `install`) so no scene's override leaks across a page/world switch; a
/// scene that overrides one re-applies it *after* the reset. Consolidated into one
/// helper so a new demo cannot reset one scale and forget the other.
pub fn reset_scene_scales() {
    reset_launch_speed_scale();
    reset_grab_force_scale();
    reset_draw_scales();
}

/// Fixed bullet-sphere radius (`sample.cpp` :1247, `b3Sphere{ zero, 0.25f }`).
const PROJECTILE_RADIUS: f32 = 0.25;
/// Density multiplier applied to the default shape density (`sample.cpp` :1248,
/// `shapeDef.density *= 4.0f`).
const PROJECTILE_DENSITY_SCALE: f32 = 4.0;

/// Spinning-cylinder projectile: `b3CreateCylinder(2.0, 0.15, 0.0, 6)` (`sample.cpp`
/// :1226). Half-height for the centered render mesh is `height / 2`.
const CYLINDER_HEIGHT: f32 = 2.0;
const CYLINDER_RADIUS: f32 = 0.15;
const CYLINDER_SIDES: i32 = 6;
const CYLINDER_HALF_HEIGHT: f32 = CYLINDER_HEIGHT * 0.5;

/// Normalize the pick-ray translation; `None` when the ray is degenerate.
fn pick_direction(translation: Vec3) -> Option<Vec3> {
    let len = length(translation);
    if len < 1e-8 {
        return None;
    }
    Some(Vec3 {
        x: translation.x / len,
        y: translation.y / len,
        z: translation.z / len,
    })
}

fn spawn_position(origin: Pos, direction: Vec3) -> Pos {
    // position = pickRay.origin + 2.0f * direction (sample.cpp :1221 / :1242 / :1232)
    Pos {
        x: origin.x + 2.0 * direction.x,
        y: origin.y + 2.0 * direction.y,
        z: origin.z + 2.0 * direction.z,
    }
}

fn scaled_velocity(direction: Vec3, speed_factor: f32) -> Vec3 {
    let speed = speed_factor * launch_speed_scale();
    Vec3 {
        x: speed * direction.x,
        y: speed * direction.y,
        z: speed * direction.z,
    }
}

/// Spawn the C sample's plain Shift-click bullet sphere (`sample.cpp` :1238-1250):
/// radius 0.25, density ×4, speed `20·launchSpeedScale`. Thin wrapper over
/// [`spawn_projectile`] for call sites that only want the sphere branch.
#[allow(dead_code)]
pub fn spawn_random(world: &mut World, origin: Pos, translation: Vec3) -> Option<SpawnedBody> {
    spawn_projectile(world, origin, translation, LaunchVariant::Sphere)
        .into_iter()
        .next()
}

/// Spawn a Shift-click projectile along a pick ray. Variant selects the C branch:
/// sphere (plain Shift), spinning cylinder (Shift+Ctrl), or ragdoll human
/// (Shift+Alt). Returns one descriptor per render body (many for a human).
pub fn spawn_projectile(
    world: &mut World,
    origin: Pos,
    translation: Vec3,
    variant: LaunchVariant,
) -> Vec<SpawnedBody> {
    let Some(direction) = pick_direction(translation) else {
        return Vec::new();
    };
    match variant {
        LaunchVariant::Sphere => spawn_sphere_projectile(world, origin, direction)
            .into_iter()
            .collect(),
        LaunchVariant::Cylinder => spawn_cylinder_projectile(world, origin, direction)
            .into_iter()
            .collect(),
        LaunchVariant::Human => spawn_human_projectile(world, origin, direction),
    }
}

fn spawn_sphere_projectile(
    world: &mut World,
    origin: Pos,
    direction: Vec3,
) -> Option<SpawnedBody> {
    // Projectile launch speed: `20.0 * m_launchSpeedScale` (`sample.cpp` :1243).
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = spawn_position(origin, direction);
    body_def.linear_velocity = scaled_velocity(direction, 20.0);
    body_def.is_bullet = true; // sample.cpp :1244

    let body_id = create_body(world, &body_def);

    // b3Sphere sphere = { b3Vec3_zero, 0.25f }; shapeDef.density *= 4.0f (:1247-1249)
    let mut shape_def = default_shape_def();
    shape_def.density *= PROJECTILE_DENSITY_SCALE;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: PROJECTILE_RADIUS,
    };
    create_sphere_shape(world, body_id, &shape_def, &sphere);

    Some(SpawnedBody {
        body_index: body_id.index1 - 1,
        half_extents: [PROJECTILE_RADIUS, PROJECTILE_RADIUS, PROJECTILE_RADIUS],
        kind: KIND_SPHERE,
    })
}

fn spawn_cylinder_projectile(
    world: &mut World,
    origin: Pos,
    direction: Vec3,
) -> Option<SpawnedBody> {
    // sample.cpp :1217-1228 — dynamic bullet, speed `10 * m_launchSpeedScale`,
    // hull = b3CreateCylinder(2.0, 0.15, 0.0, 6).
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = spawn_position(origin, direction);
    body_def.linear_velocity = scaled_velocity(direction, 10.0);
    body_def.is_bullet = true;

    let body_id = create_body(world, &body_def);
    let shape_def = default_shape_def();
    let hull = create_cylinder(CYLINDER_HEIGHT, CYLINDER_RADIUS, 0.0, CYLINDER_SIDES)?;
    create_hull_shape(world, body_id, &shape_def, &hull);

    // Render as a Y-axis cylinder. Physics hull spans y∈[0, height]; offset the
    // centered mesh by half-height so it lines up with the hull.
    Some(SpawnedBody {
        body_index: body_id.index1 - 1,
        // [radius, half_height, local_y_offset]
        half_extents: [CYLINDER_RADIUS, CYLINDER_HALF_HEIGHT, CYLINDER_HALF_HEIGHT],
        kind: KIND_CYLINDER,
    })
}

fn spawn_human_projectile(world: &mut World, origin: Pos, direction: Vec3) -> Vec<SpawnedBody> {
    // sample.cpp :1230-1236 — CreateHuman(..., 1,1,1, group 0, null, true),
    // Human_SetBullet(true), Human_SetVelocity((10 * scale) * dir).
    let position = spawn_position(origin, direction);
    let mut human = Human::default();
    create_human(
        &mut human, world, position, 1.0, 1.0, 1.0, 0, 0, true,
    );
    human_set_bullet(&human, world, true);
    human_set_velocity(&human, world, scaled_velocity(direction, 10.0));

    let mut out = Vec::with_capacity(BONE_COUNT);
    for i in 0..BONE_COUNT {
        let body_id = human.bones[i].body_id;
        if body_id.is_null() {
            continue;
        }
        let body_index = body_id.index1 - 1;
        let Some(cap) = capsule_from_body(world, body_index) else {
            continue;
        };
        // Pack radius + half-segment for adapters that don't re-query the world.
        let half_len = 0.5
            * length(Vec3 {
                x: cap.center2.x - cap.center1.x,
                y: cap.center2.y - cap.center1.y,
                z: cap.center2.z - cap.center1.z,
            });
        out.push(SpawnedBody {
            body_index,
            half_extents: [cap.radius, half_len, cap.radius],
            kind: KIND_CAPSULE,
        });
    }
    out
}

/// Append spawned projectile bodies to a `VisBody` render list (sphere / capsule /
/// cylinder), looking up live capsule geometry from the world when needed.
pub fn append_spawned_vis(world: &World, bodies: &mut Vec<VisBody>, spawned: &[SpawnedBody]) {
    for sp in spawned {
        match sp.kind {
            KIND_SPHERE => {
                bodies.push(VisBody::sphere_body(sp.body_index, sp.half_extents[0]));
            }
            KIND_CAPSULE => {
                if let Some(cap) = capsule_from_body(world, sp.body_index) {
                    bodies.push(VisBody::capsule_body(sp.body_index, &cap));
                }
            }
            KIND_CYLINDER => {
                let local = Transform {
                    p: Vec3 {
                        x: 0.0,
                        y: sp.half_extents[2],
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                };
                bodies.push(VisBody::cylinder_local(
                    sp.body_index,
                    sp.half_extents[0],
                    sp.half_extents[1],
                    local,
                    0,
                ));
            }
            _ => {
                bodies.push(VisBody::box_body(
                    sp.body_index,
                    sp.half_extents[0],
                    sp.half_extents[1],
                    sp.half_extents[2],
                ));
            }
        }
    }
}

/// Default wasm return payload: `[ok, body_index, hx, hy, hz, kind]` for the
/// first spawned body (ok=0 when empty).
pub fn spawn_ok_payload(spawned: &[SpawnedBody]) -> Vec<f32> {
    match spawned.first() {
        Some(sp) => vec![
            1.0,
            sp.body_index as f32,
            sp.half_extents[0],
            sp.half_extents[1],
            sp.half_extents[2],
            sp.kind as f32,
        ],
        None => vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
}

/// Destroy the dynamic body under a pick ray. Returns the destroyed body index, or -1.
pub fn delete_at_ray(
    world: &mut World,
    grab: &mut MouseGrab,
    origin: Pos,
    translation: Vec3,
) -> i32 {
    grab.end(world);

    let filter = default_query_filter();
    let result = world_cast_ray_closest(world, origin, translation, &filter);
    if !result.hit {
        return -1;
    }

    let body_id = shape_get_body(world, result.shape_id);
    if body_get_type(world, body_id) != BodyType::Dynamic {
        return -1;
    }

    let index = body_id.index1 - 1;
    destroy_body(world, body_id);
    index
}

/// Counters + awake/sleeping dynamic body counts.
///
/// Layout: `[body, shape, contact, joint, island, awake_dynamic, sleeping_dynamic]`
pub fn counters_with_sleep(world: &World) -> [f32; 7] {
    let c = world_get_counters(world);
    let mut awake = 0i32;
    let mut sleeping = 0i32;
    for i in 0..world.bodies.len() {
        let body = &world.bodies[i];
        if body.set_index == box3d_rust::core::NULL_INDEX {
            continue;
        }
        if body.type_ != BodyType::Dynamic {
            continue;
        }
        if is_body_awake(world, i as i32) {
            awake += 1;
        } else {
            sleeping += 1;
        }
    }
    [
        c.body_count as f32,
        c.shape_count as f32,
        c.contact_count as f32,
        c.joint_count as f32,
        c.island_count as f32,
        awake as f32,
        sleeping as f32,
    ]
}

/// Override the active scene's projectile launch-speed scale (`m_launchSpeedScale`,
/// `sample.cpp` :330). Scene resets restore the base default of 5.0, so a scene
/// calls this *after* its reset when the matching C sample overrides the scale
/// (e.g. Compound Village sets 2.0).
#[wasm_bindgen]
pub fn sim_set_launch_speed_scale(scale: f32) {
    set_launch_speed_scale(scale);
}

/// Helper: build Pos/Vec3 from floats.
pub fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

pub fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

#[cfg(test)]
mod tests {
    use super::*;
    use box3d_rust::body::{body_get_linear_velocity, body_is_bullet, make_body_id};
    use box3d_rust::types::default_world_def;

    fn test_world() -> World {
        World::new(&default_world_def())
    }

    fn unit_z() -> Vec3 {
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        }
    }

    #[test]
    fn spawn_cylinder_is_bullet_with_c_dims_and_speed() {
        reset_launch_speed_scale();
        let mut world = test_world();
        let origin = Pos {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        let spawned = spawn_projectile(&mut world, origin, unit_z(), LaunchVariant::Cylinder);
        assert_eq!(spawned.len(), 1);
        let sp = spawned[0];
        assert_eq!(sp.kind, KIND_CYLINDER);
        assert!((sp.half_extents[0] - CYLINDER_RADIUS).abs() < 1e-6);
        assert!((sp.half_extents[1] - CYLINDER_HALF_HEIGHT).abs() < 1e-6);

        let body_id = make_body_id(&world, sp.body_index);
        assert!(body_is_bullet(&world, body_id));

        let vel = body_get_linear_velocity(&world, body_id);
        let expected = 10.0 * launch_speed_scale();
        let mag = length(vel);
        assert!(
            (mag - expected).abs() < 1e-4,
            "cylinder speed {mag} != 10*scale ({expected})"
        );
        assert!((vel.z - expected).abs() < 1e-4);
    }

    #[test]
    fn spawn_human_is_bullet_with_c_velocity() {
        reset_launch_speed_scale();
        let mut world = test_world();
        let origin = Pos {
            x: 0.0,
            y: 5.0,
            z: 0.0,
        };
        let spawned = spawn_projectile(&mut world, origin, unit_z(), LaunchVariant::Human);
        assert!(
            spawned.len() > 1,
            "human should spawn multiple bone bodies, got {}",
            spawned.len()
        );
        assert!(spawned.iter().all(|s| s.kind == KIND_CAPSULE));

        let expected = 10.0 * launch_speed_scale();
        for sp in &spawned {
            let body_id = make_body_id(&world, sp.body_index);
            assert!(
                body_is_bullet(&world, body_id),
                "bone {} should be bullet",
                sp.body_index
            );
            let vel = body_get_linear_velocity(&world, body_id);
            let mag = length(vel);
            assert!(
                (mag - expected).abs() < 1e-3,
                "bone {} speed {mag} != 10*scale ({expected})",
                sp.body_index
            );
        }
    }

    #[test]
    fn spawn_sphere_still_uses_20x_scale() {
        reset_launch_speed_scale();
        let mut world = test_world();
        let spawned = spawn_projectile(
            &mut world,
            Pos {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            unit_z(),
            LaunchVariant::Sphere,
        );
        assert_eq!(spawned.len(), 1);
        let body_id = make_body_id(&world, spawned[0].body_index);
        let expected = 20.0 * launch_speed_scale();
        let mag = length(body_get_linear_velocity(&world, body_id));
        assert!((mag - expected).abs() < 1e-4);
    }

    #[test]
    fn launch_variant_ctrl_precedes_alt() {
        // Document the C precedence: from_u8 only encodes a single choice; TS
        // picks Ctrl over Alt before calling. Sphere is the default fallback.
        assert_eq!(LaunchVariant::from_u8(0), LaunchVariant::Sphere);
        assert_eq!(LaunchVariant::from_u8(1), LaunchVariant::Cylinder);
        assert_eq!(LaunchVariant::from_u8(2), LaunchVariant::Human);
        assert_eq!(LaunchVariant::from_u8(99), LaunchVariant::Sphere);
    }
}
