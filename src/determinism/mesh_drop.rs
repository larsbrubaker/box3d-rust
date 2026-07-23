//! Thin-box mesh drop scene (`shared/stability.c`).
//!
//! Drops a grid of thin fast boxes onto a wave mesh. Stresses continuous collision
//! and mesh contact stability, and doubles as a determinism scenario via the sleep
//! hash.

use super::shared::hash_world_transform;
use crate::body::{body_get_transform, create_body};
use crate::core::HASH_INIT;
use crate::hull::make_box_hull;
use crate::human::{random_vec3_uniform, set_random_seed};
use crate::id::BodyId;
use crate::math_functions::{offset_pos, Pos, Vec3, VEC3_ONE};
use crate::mesh::{create_wave_mesh, destroy_mesh, MeshData};
use crate::shape::{create_hull_shape, create_mesh_shape};
use crate::types::{default_body_def, default_shape_def, BodyType, Filter};
use crate::world::{world_get_awake_body_count, World};

/// Grid dimension for the thin-box drop. (MESH_DROP_GRID_COUNT)
pub const MESH_DROP_GRID_COUNT: usize = 20;

/// Total thin boxes dropped on the wave mesh.
pub const MESH_DROP_BODY_COUNT: usize = MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT;

/// Thin fast boxes dropped on a wave mesh. Stresses continuous collision and mesh contact
/// stability, and doubles as a determinism scenario via the sleep hash. (MeshDropData)
#[derive(Debug)]
pub struct MeshDropData {
    pub mesh: Option<MeshData>,
    pub bodies: [BodyId; MESH_DROP_BODY_COUNT],
    pub step_count: i32,
    pub sleep_step: i32,
    pub hash: u32,
}

impl Default for MeshDropData {
    fn default() -> Self {
        MeshDropData {
            mesh: None,
            bodies: [BodyId::default(); MESH_DROP_BODY_COUNT],
            step_count: 0,
            sleep_step: 0,
            hash: 0,
        }
    }
}

/// Drop a grid of thin fast boxes onto a wave mesh. (CreateMeshDrop)
pub fn create_mesh_drop(world: &mut World, origin: Pos) -> MeshDropData {
    let mut data = MeshDropData::default();

    {
        let mut body_def = default_body_def();
        body_def.position = origin;
        let ground_id = create_body(world, &body_def);

        let grid_count = 40;
        let cell_width = 1.0;
        let row_hz = 0.1;
        let column_hz = 0.2;
        let ground_amplitude = 0.5;

        let mesh = create_wave_mesh(
            grid_count,
            grid_count,
            cell_width,
            ground_amplitude,
            row_hz,
            column_hz,
        )
        .expect("wave mesh");
        let mut shape_def = default_shape_def();
        shape_def.filter.category_bits = 1;
        create_mesh_shape(world, ground_id, &shape_def, &mesh, VEC3_ONE);
        data.mesh = Some(mesh);
    }

    {
        let box_hull = make_box_hull(0.02, 0.2, 0.04);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;

        let mut shape_def = default_shape_def();
        shape_def.base_material.rolling_resistance = 0.1;

        // Don't allow shapes to collide with each other.
        shape_def.filter = Filter {
            category_bits: 2,
            mask_bits: 1,
            group_index: 0,
        };

        set_random_seed(3963634789);

        let grid_count = MESH_DROP_GRID_COUNT;

        for i in 0..grid_count {
            for j in 0..grid_count {
                let linear_velocity = random_vec3_uniform(-1.0, 1.0);
                let angular_velocity = random_vec3_uniform(-5.0, 5.0);

                body_def.position = offset_pos(
                    origin,
                    Vec3 {
                        x: 0.5 * (i as f32 - 0.5 * grid_count as f32),
                        y: 5.0,
                        z: 0.5 * (j as f32 - 0.5 * grid_count as f32),
                    },
                );
                body_def.linear_velocity = linear_velocity;
                body_def.angular_velocity = angular_velocity;
                let body_id = create_body(world, &body_def);
                data.bodies[i * grid_count + j] = body_id;

                create_hull_shape(world, body_id, &shape_def, &box_hull.base);
            }
        }
    }

    data
}

/// Advance sleep/hash tracking for the mesh drop. (UpdateMeshDrop)
pub fn update_mesh_drop(world: &World, data: &mut MeshDropData) -> bool {
    if data.hash == 0 && world_get_awake_body_count(world) == 0 {
        data.hash = HASH_INIT;
        for i in 0..MESH_DROP_BODY_COUNT {
            let xf = body_get_transform(world, data.bodies[i]);
            data.hash = hash_world_transform(data.hash, &xf);
        }

        data.sleep_step = data.step_count;
    }

    data.step_count += 1;

    data.hash != 0
}

/// Release the owned wave mesh. (DestroyMeshDrop)
pub fn destroy_mesh_drop(data: &mut MeshDropData) {
    if let Some(mesh) = data.mesh.take() {
        destroy_mesh(mesh);
    }
}
