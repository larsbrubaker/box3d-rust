//! Rigid bodies: create, destroy, and configure simulation state.
//!
//! Use [`create_body`] with a [`crate::types::BodyDef`] (from
//! [`crate::types::default_body_def`]) to add a body to a [`crate::world::World`].
//! Attach collision with [`crate::shape`], then apply forces / set transforms
//! through the accessors in this module.
//!
//! Port of `body.h` / `body.c` (lifecycle, mass, velocity, and query API).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

mod access;
mod api;
mod forces;
mod lifecycle;
mod mass;
mod query;
mod set_type;
mod types;

pub use access::*;
pub use api::*;
pub use forces::*;
pub use lifecycle::*;
pub use mass::*;
pub use query::*;
pub use set_type::*;
pub use types::*;
