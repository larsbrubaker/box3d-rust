//! Human ragdoll builder from `shared/human.c` / `shared/human.h`.
//!
//! Capsule bones + spherical/revolute joint tree used by the determinism scene
//! and (later) joint sample demos.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod api;
mod create;
mod random;
mod types;

pub use api::*;
pub use create::create_human;
pub use types::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math_functions::POS_ZERO;
    use crate::types::default_world_def;
    use crate::world::World;

    #[test]
    fn create_and_destroy_human() {
        let mut world = World::new(&default_world_def());
        let mut human = Human::default();
        create_human(&mut human, &mut world, POS_ZERO, 5.0, 1.0, 0.7, 1, 0, false);
        assert!(human.is_spawned);
        assert!(!human.bones[BoneId::Pelvis as usize].body_id.is_null());
        assert!(!human.bones[BoneId::Head as usize].joint_id.is_null());
        assert_eq!(human.filter_joint_count, 1);
        destroy_human(&mut human, &mut world);
        assert!(!human.is_spawned);
    }
}
