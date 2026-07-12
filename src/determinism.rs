//! Falling-ragdoll determinism scene helpers from `shared/determinism.c`.
//!
//! Builds a grid of ragdolls over mesh grounds and hashes settled transforms.
//! The final EXPECTED_HASH gate in `test_determinism.c` stays deferred until
//! task-6 lands mesh narrow-phase (bodies currently fall through meshes).
//!
//! The content hash itself (`b3Hash` / [`crate::core::hash`]) already lives in
//! [`crate::core`]; this module adds the world-transform byte layout used by
//! the scene.
//!
//! SPDX-FileCopyrightText: 2022 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under double-precision

use crate::body::{body_get_transform, create_body};
use crate::core::{hash, HASH_INIT};
use crate::human::{create_human, Human, BONE_COUNT};
use crate::math_functions::{Pos, WorldTransform, VEC3_ONE};
use crate::mesh::{create_grid_mesh, create_torus_mesh, destroy_mesh, MeshData};
use crate::shape::create_mesh_shape;
use crate::types::{default_body_def, default_shape_def};
use crate::world::{world_get_awake_body_count, world_get_body_events, World};

/// Humans per ground cell. (RAGDOLL_GROUP_SIZE)
pub const RAGDOLL_GROUP_SIZE: usize = 2;

/// Grid dimension (NxN cells). (RAGDOLL_GRID_COUNT)
pub const RAGDOLL_GRID_COUNT: usize = 2;

const GRID_SIZE: f32 = 15.0;

/// One cell of ragdolls. (RagdollGroup)
#[derive(Debug, Clone, Default)]
pub struct RagdollGroup {
    pub humans: [Human; RAGDOLL_GROUP_SIZE],
}

/// Falling-ragdoll scene state. (FallingRagdollData)
#[derive(Debug)]
pub struct FallingRagdollData {
    pub groups: [RagdollGroup; RAGDOLL_GRID_COUNT * RAGDOLL_GRID_COUNT],
    pub grid_mesh: Option<MeshData>,
    pub torus_mesh: Option<MeshData>,
    pub column_count: i32,
    pub column_index: i32,
    pub step_count: i32,
    pub sleep_step: i32,
    pub hash: u32,
}

impl Default for FallingRagdollData {
    fn default() -> Self {
        FallingRagdollData {
            groups: std::array::from_fn(|_| RagdollGroup::default()),
            grid_mesh: None,
            torus_mesh: None,
            column_count: 0,
            column_index: 0,
            step_count: 0,
            sleep_step: 0,
            hash: 0,
        }
    }
}

/// Hash a world transform's raw little-endian field bytes in C declaration
/// order, matching `b3Hash(hash, (uint8_t*)&xf, sizeof(b3WorldTransform))` on
/// little-endian hosts. (determinism.c UpdateFallingRagdolls)
pub fn hash_world_transform(h: u32, xf: &WorldTransform) -> u32 {
    let mut bytes = Vec::with_capacity(40);
    bytes.extend_from_slice(&xf.p.x.to_le_bytes());
    bytes.extend_from_slice(&xf.p.y.to_le_bytes());
    bytes.extend_from_slice(&xf.p.z.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.x.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.y.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.z.to_le_bytes());
    bytes.extend_from_slice(&xf.q.s.to_le_bytes());
    hash(h, &bytes)
}

fn create_group(
    data: &mut FallingRagdollData,
    world: &mut World,
    row_index: usize,
    column_index: usize,
) {
    debug_assert!(row_index < RAGDOLL_GRID_COUNT && column_index < RAGDOLL_GRID_COUNT);

    let group_index = row_index * RAGDOLL_GRID_COUNT + column_index;

    let span = RAGDOLL_GRID_COUNT as f32 * GRID_SIZE;
    let group_distance = 1.0 * span / RAGDOLL_GRID_COUNT as f32;

    let mut position = Pos {
        x: (-0.5 * span + group_distance * (column_index as f32 + 0.5)) as _,
        y: 15.0 as _,
        z: (-0.5 * span + group_distance * (row_index as f32 + 0.5)) as _,
    };

    let friction_torque = 5.0;
    let hertz = 1.0;
    let damping_ratio = 0.7;
    let colorize = false;

    for i in 0..RAGDOLL_GROUP_SIZE {
        let human = &mut data.groups[group_index].humans[i];
        create_human(
            human,
            world,
            position,
            friction_torque,
            hertz,
            damping_ratio,
            group_index as i32,
            0,
            colorize,
        );
        position.x = (position.x as f32 + 0.75) as _;
    }
}

/// Build mesh grounds and spawn the ragdoll grid. (CreateFallingRagdolls)
pub fn create_falling_ragdolls(world: &mut World) -> FallingRagdollData {
    let mut data = FallingRagdollData::default();

    let half_mesh_grid_rows = 4;
    let mesh_grid_cell_width = GRID_SIZE / (2.0 * half_mesh_grid_rows as f32);
    data.grid_mesh = create_grid_mesh(
        2 * half_mesh_grid_rows,
        2 * half_mesh_grid_rows,
        mesh_grid_cell_width,
        0,
        true,
    );
    data.torus_mesh = create_torus_mesh(16, 16, 0.25 * GRID_SIZE, 1.0);

    let span = GRID_SIZE * RAGDOLL_GRID_COUNT as f32;
    let mut body_def = default_body_def();
    let shape_def = default_shape_def();

    // Clone mesh data for shape attach; FallingRagdollData keeps the originals
    // for DestroyFallingRagdolls (C keeps borrowed pointers).
    let grid_mesh = data.grid_mesh.as_ref().expect("grid mesh").clone();
    let torus_mesh = data.torus_mesh.as_ref().expect("torus mesh").clone();

    body_def.position.x = (-0.5 * span + 0.5 * GRID_SIZE) as _;
    for i in 0..RAGDOLL_GRID_COUNT {
        body_def.position.z = (-0.5 * span + 0.5 * GRID_SIZE) as _;
        for j in 0..RAGDOLL_GRID_COUNT {
            let body = create_body(world, &body_def);
            create_mesh_shape(world, body, &shape_def, &grid_mesh, VEC3_ONE);
            create_mesh_shape(world, body, &shape_def, &torus_mesh, VEC3_ONE);

            create_group(&mut data, world, i, j);

            body_def.position.z = (body_def.position.z as f32 + GRID_SIZE) as _;
        }

        body_def.position.x = (body_def.position.x as f32 + GRID_SIZE) as _;
    }

    data
}

/// Advance sleep/hash tracking after a world step. Returns true once settled.
/// (UpdateFallingRagdolls)
pub fn update_falling_ragdolls(world: &World, data: &mut FallingRagdollData) -> bool {
    if data.hash == 0 {
        let body_events = world_get_body_events(world);

        if body_events.is_empty() {
            let awake_count = world_get_awake_body_count(world);
            debug_assert!(awake_count == 0);

            data.hash = HASH_INIT;
            for i in 0..RAGDOLL_GRID_COUNT {
                for j in 0..RAGDOLL_GRID_COUNT {
                    for k in 0..RAGDOLL_GROUP_SIZE {
                        let group_index = i * RAGDOLL_GRID_COUNT + j;
                        let human = &data.groups[group_index].humans[k];

                        for b in 0..BONE_COUNT {
                            let body_id = human.bones[b].body_id;
                            let xf = body_get_transform(world, body_id);
                            data.hash = hash_world_transform(data.hash, &xf);
                        }
                    }
                }
            }

            data.sleep_step = data.step_count;
        }
    }

    data.step_count += 1;

    data.hash != 0
}

/// Release owned mesh data. (DestroyFallingRagdolls)
pub fn destroy_falling_ragdolls(data: &mut FallingRagdollData) {
    if let Some(mesh) = data.grid_mesh.take() {
        destroy_mesh(mesh);
    }
    if let Some(mesh) = data.torus_mesh.take() {
        destroy_mesh(mesh);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::default_world_def;

    #[test]
    fn create_and_destroy_falling_ragdolls() {
        let mut world = World::new(&default_world_def());
        let mut data = create_falling_ragdolls(&mut world);
        assert!(data.grid_mesh.is_some());
        assert!(data.torus_mesh.is_some());
        assert!(!data.groups[0].humans[0].bones[0].body_id.is_null());
        destroy_falling_ragdolls(&mut data);
        assert!(data.grid_mesh.is_none());
    }

    #[test]
    fn hash_world_transform_is_stable() {
        let xf = WorldTransform {
            p: Pos {
                x: 1.0 as _,
                y: 2.0 as _,
                z: 3.0 as _,
            },
            q: crate::math_functions::QUAT_IDENTITY,
        };
        let h1 = hash_world_transform(HASH_INIT, &xf);
        let h2 = hash_world_transform(HASH_INIT, &xf);
        assert_eq!(h1, h2);
        assert_ne!(h1, HASH_INIT);
    }
}
