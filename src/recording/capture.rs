//! Capture-hook helpers for recording mutators.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::recording::session::{with_recording, world_public_id, Recording};
use crate::world::World;

/// Record a void mutator if a session is active.
#[inline]
pub fn rec(world: &mut World, write: impl FnOnce(&mut Recording, crate::id::WorldId)) {
    let wid = world_public_id(world);
    with_recording(world, |r| write(r, wid));
}
