//! World samples (`sample_world.cpp`): Far Stack, Far Pyramid, Far Ragdolls,
//! Far Mesh Drop. Each builds an identical scene a long way from the origin to
//! exercise the large-world (double-precision) coordinate path.
//!
//! `far_pyramid` is the reference-quality exact port (its own STRIDE-11 pose
//! path + `world_far_pyramid_*` exports). The other three scenes share the
//! generic `far` module (one active scene at a time), which packs the standard
//! 16-float `vis` pose stride. Both shift every world position back into the base
//! frame so the float renderer works near the origin — poses via `sub_pos` and
//! the debug overlay via the shared collection-time draw origin
//! ([`crate::interact::with_draw_base`]).

mod far;
mod far_pyramid;
