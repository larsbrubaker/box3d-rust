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
use box3d_rust::id::{BodyId, JointId, NULL_BODY_ID, NULL_JOINT_ID};
use box3d_rust::joint::{create_motor_joint, destroy_joint, joint_is_valid};
use box3d_rust::math_functions::{
    length, Pos, Transform, Vec3, WorldTransform, QUAT_IDENTITY, VEC3_ZERO,
};
use box3d_rust::shape::{create_sphere_shape, shape_get_body};
use box3d_rust::types::{
    default_body_def, default_motor_joint_def, default_query_filter, default_shape_def, BodyType,
};
use box3d_rust::world::{world_cast_ray_closest, world_get_counters, World};
use std::cell::Cell;
use wasm_bindgen::prelude::*;

mod draw;
pub use draw::*;

/// Mouse-grab state for one demo world (C Sample mouse body + motor joint).
#[derive(Clone, Copy)]
pub struct MouseGrab {
    pub mouse_body_id: BodyId,
    pub mouse_joint_id: JointId,
    pub mouse_point: Pos,
    pub force_scale: f32,
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
            force_scale: 100.0,
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
        joint_def.max_spring_force = self.force_scale * mg;

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
#[derive(Clone, Copy)]
pub struct SpawnedBody {
    pub body_index: i32,
    pub half_extents: [f32; 3],
    /// 0 = box, 1 = sphere, 2 = capsule
    pub kind: u8,
}

/// `m_launchSpeedScale` default from the base `Sample` constructor
/// (`sample.cpp` :330). The base ctor sets 5.0; individual samples override it in
/// their own ctor (e.g. Compound Village sets 2.0).
const DEFAULT_LAUNCH_SPEED_SCALE: f32 = 5.0;

thread_local! {
    /// Current `m_launchSpeedScale`. Matches C semantics: scene resets restore the
    /// base default (via [`reset_launch_speed_scale`]) and a scene that overrides
    /// it re-applies its value after reset (via [`set_launch_speed_scale`] /
    /// [`sim_set_launch_speed_scale`]). Read only at projectile-spawn time.
    static LAUNCH_SPEED_SCALE: Cell<f32> = const { Cell::new(DEFAULT_LAUNCH_SPEED_SCALE) };
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

/// Reset every per-scene scale that lives in a thread-local — the projectile
/// launch-speed scale and the debug-draw joint/force scales — back to its base
/// default. Called at each world-construction seam (`new_world` / `new_sim` /
/// scene `install`) so no scene's override leaks across a page/world switch; a
/// scene that overrides one re-applies it *after* the reset. Consolidated into one
/// helper so a new demo cannot reset one scale and forget the other.
pub fn reset_scene_scales() {
    reset_launch_speed_scale();
    reset_draw_scales();
}

/// Fixed bullet-sphere radius (`sample.cpp` :1247, `b3Sphere{ zero, 0.25f }`).
const PROJECTILE_RADIUS: f32 = 0.25;
/// Density multiplier applied to the default shape density (`sample.cpp` :1248,
/// `shapeDef.density *= 4.0f`).
const PROJECTILE_DENSITY_SCALE: f32 = 4.0;

/// Spawn the C sample's shift-click projectile along a pick ray: a dynamic
/// bullet **sphere** of radius 0.25 at `origin + 2·direction`, launched at
/// `20·launchSpeedScale·direction`, with the default shape density boosted ×4.
/// Mirrors `Sample::MouseDown`'s plain shift branch (`sample.cpp` :1238-1250) —
/// no `MOD_CTRL` (cylinder) / `MOD_ALT` (ragdoll) variant. Returns a render
/// descriptor (`kind = 1`, sphere).
pub fn spawn_random(world: &mut World, origin: Pos, translation: Vec3) -> Option<SpawnedBody> {
    let len = length(translation);
    if len < 1e-8 {
        return None;
    }
    let direction = Vec3 {
        x: translation.x / len,
        y: translation.y / len,
        z: translation.z / len,
    };

    // Projectile launch speed: `20.0 * m_launchSpeedScale` (`sample.cpp` :1243),
    // read at spawn time so per-scene overrides (`sim_set_launch_speed_scale`) apply.
    let projectile_speed = 20.0 * launch_speed_scale();

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    // position = pickRay.origin + 2.0f * direction (sample.cpp :1242)
    body_def.position = Pos {
        x: origin.x + 2.0 * direction.x,
        y: origin.y + 2.0 * direction.y,
        z: origin.z + 2.0 * direction.z,
    };
    // linearVelocity = (20.0f * m_launchSpeedScale) * direction (sample.cpp :1243)
    body_def.linear_velocity = Vec3 {
        x: projectile_speed * direction.x,
        y: projectile_speed * direction.y,
        z: projectile_speed * direction.z,
    };
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
        kind: 1, // sphere
    })
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
