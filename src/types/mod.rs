// Port of public definition types from box3d-cpp-reference/include/box3d/types.h
// and defaults from src/types.c. Split by domain; flat re-exports for callers.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

mod body;
mod joint;
mod shape;
mod world;

pub use body::*;
pub use joint::*;
pub use shape::*;
pub use world::*;
