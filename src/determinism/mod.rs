//! Determinism scene helpers from `shared/determinism.c`.
//!
//! Builds the falling-ragdoll grid over mesh grounds, the convex wave pile, the
//! query-driven spawn scene, and the thin-box mesh drop, hashing settled
//! transforms. These feed the `test_determinism.c` cross-platform gate.
//!
//! The content hash itself (`b3Hash` / [`crate::core::hash`]) already lives in
//! [`crate::core`]; the [`shared`] submodule adds the world-transform byte layout
//! used by the scenes. Each scene lives in its own submodule and is re-exported
//! here so callers keep using flat `crate::determinism::*` paths.
//!
//! SPDX-FileCopyrightText: 2022 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under double-precision

mod falling_ragdolls;
mod mesh_drop;
mod query_spawn;
mod shared;
mod wave_pile;

pub use falling_ragdolls::{
    create_falling_ragdolls, destroy_falling_ragdolls, update_falling_ragdolls, FallingRagdollData,
    RagdollGroup, RAGDOLL_GRID_COUNT, RAGDOLL_GROUP_SIZE,
};
pub use mesh_drop::{
    create_mesh_drop, destroy_mesh_drop, update_mesh_drop, MeshDropData, MESH_DROP_BODY_COUNT,
    MESH_DROP_GRID_COUNT,
};
pub use query_spawn::{
    create_query_spawn, destroy_query_spawn, update_query_spawn, QuerySpawnData,
    QUERY_SPAWN_CAST_RADIUS, QUERY_SPAWN_COUNT,
};
pub use shared::hash_world_transform;
pub use wave_pile::{
    create_wave_pile, destroy_wave_pile, update_wave_pile, WavePileData, WAVE_PILE_BODY_COUNT,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::HASH_INIT;
    use crate::math_functions::{Pos, WorldTransform};
    use crate::types::default_world_def;
    use crate::world::World;

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
