//! Pure Rust port of [Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D physics
//! engine for games.
//!
//! The port targets exact behavioral match with the C source pinned in the
//! `box3d-cpp-reference/` submodule: same algorithms, same `f32` arithmetic, same edge
//! cases, including Box3D's hand-rolled cross-platform-deterministic trigonometry.
//!
//! Porting has just begun — modules land whole, in dependency order, together with their
//! portion of the upstream C test suite. See the repository README for live status.

/// Crate version, exposed so demos and downstream tools can report the exact port build.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_matches_cargo_manifest() {
        assert_eq!(VERSION, env!("CARGO_PKG_VERSION"));
        assert!(!VERSION.is_empty());
    }
}
