//! Test-support ports of the C `shared/` scene code that the ported unit tests depend on.
//!
//! `test_mesh.c` links the shared ghost collision scene and the ghost oracle. Only the parts
//! those tests drive are ported here; the samples and benchmarks that use the rest are not.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

pub(crate) mod ghost_collisions;
pub(crate) mod ghost_oracle;
