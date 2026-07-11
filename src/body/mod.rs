// Port of body.h data model and body.c lifecycle / mass / velocity API.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

mod api;
mod lifecycle;
mod types;

pub use api::*;
pub use lifecycle::*;
pub use types::*;
