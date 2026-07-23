//! Convex-pile-on-wave scene (`shared/determinism.c`).
//!
//! Drops a mixed convex pile on a wave height field and hashes the settled
//! transforms.

use super::shared::hash_world_transform;
use crate::body::{body_get_transform, create_body};
use crate::core::HASH_INIT;
use crate::geometry::{Capsule, Sphere};
use crate::height_field::{create_wave, HeightFieldData};
use crate::hull::{create_rock, make_box_hull};
use crate::human::{random_quat, random_vec3_uniform, set_random_seed};
use crate::id::BodyId;
use crate::math_functions::{Vec3, VEC3_ZERO};
use crate::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_sphere_shape,
};
use crate::types::{default_body_def, default_shape_def, BodyType};
use crate::world::{world_get_awake_body_count, World};

/// Convex bodies dropped on a wave height field. (WAVE_PILE_BODY_COUNT)
pub const WAVE_PILE_BODY_COUNT: usize = 100;

const WAVE_PILE_GRID: i32 = 5;
const WAVE_PILE_LAYERS: i32 = 4;

/// Convex-pile-on-wave scene state. (WavePileData)
#[derive(Debug)]
pub struct WavePileData {
    pub bodies: [BodyId; WAVE_PILE_BODY_COUNT],
    pub height_field: Option<HeightFieldData>,
    pub step_count: i32,
    pub sleep_step: i32,
    pub hash: u32,
}

impl Default for WavePileData {
    fn default() -> Self {
        WavePileData {
            bodies: [BodyId::default(); WAVE_PILE_BODY_COUNT],
            height_field: None,
            step_count: 0,
            sleep_step: 0,
            hash: 0,
        }
    }
}

/// Drop a mixed convex pile on a wave height field. (CreateWavePile)
pub fn create_wave_pile(world: &mut World) -> WavePileData {
    let mut data = WavePileData::default();

    set_random_seed(52977);

    // Height fields grow from a corner, offset the body to center the patch.
    let field_count = 21;
    let field_scale = Vec3 {
        x: 1.0,
        y: 0.6,
        z: 1.0,
    };
    let height_field = create_wave(field_count, field_count, field_scale, 0.08, 0.06, false);

    {
        let extent = field_scale.x * (field_count - 1) as f32;
        let mut body_def = default_body_def();
        body_def.position.x = (-0.5 * extent) as _;
        body_def.position.z = (-0.5 * extent) as _;
        let ground_id = create_body(world, &body_def);

        let shape_def = default_shape_def();
        create_height_field_shape(world, ground_id, &shape_def, &height_field);
    }

    data.height_field = Some(height_field);

    let rock = create_rock(0.55).expect("rock hull");
    let box_hull = make_box_hull(0.45, 0.3, 0.55);
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -0.3,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.3,
            z: 0.0,
        },
        radius: 0.35,
    };

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    // Rolling resistance so the pile sleeps quickly.
    let mut shape_def = default_shape_def();
    shape_def.base_material.rolling_resistance = 0.3;

    let spacing = 1.7f32;
    let mut index = 0usize;
    for layer in 0..WAVE_PILE_LAYERS {
        for i in 0..WAVE_PILE_GRID {
            for j in 0..WAVE_PILE_GRID {
                let jitter = random_vec3_uniform(-0.3, 0.3);
                body_def.position.x =
                    (spacing * (i as f32 - 0.5 * (WAVE_PILE_GRID - 1) as f32) + jitter.x) as _;
                body_def.position.y = (2.5 + 1.6 * layer as f32 + 0.3 * jitter.y) as _;
                body_def.position.z =
                    (spacing * (j as f32 - 0.5 * (WAVE_PILE_GRID - 1) as f32) + jitter.z) as _;
                body_def.rotation = random_quat();

                let body_id = create_body(world, &body_def);
                data.bodies[index] = body_id;

                match index % 4 {
                    0 => {
                        create_sphere_shape(world, body_id, &shape_def, &sphere);
                    }
                    1 => {
                        create_capsule_shape(world, body_id, &shape_def, &capsule);
                    }
                    2 => {
                        create_hull_shape(world, body_id, &shape_def, &box_hull.base);
                    }
                    _ => {
                        create_hull_shape(world, body_id, &shape_def, &rock);
                    }
                }

                index += 1;
            }
        }
    }

    debug_assert!(index == WAVE_PILE_BODY_COUNT);

    // The world keeps its own copy (C calls b3DestroyHull on the rock here).
    data
}

/// Advance sleep/hash tracking for the wave pile. (UpdateWavePile)
pub fn update_wave_pile(world: &World, data: &mut WavePileData) -> bool {
    if data.hash == 0 && world_get_awake_body_count(world) == 0 {
        data.hash = HASH_INIT;
        for i in 0..WAVE_PILE_BODY_COUNT {
            let xf = body_get_transform(world, data.bodies[i]);
            data.hash = hash_world_transform(data.hash, &xf);
        }

        data.sleep_step = data.step_count;
    }

    data.step_count += 1;

    data.hash != 0
}

/// Release the owned height field. (DestroyWavePile)
pub fn destroy_wave_pile(data: &mut WavePileData) {
    data.height_field = None;
}
