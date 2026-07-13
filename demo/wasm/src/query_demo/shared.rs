//! Shared cast accumulator + closest-hit callback for the ray/shape-cast samples
//! (`sample_collision.cpp` `CastContext` + `RayCastClosestCallback`).

use box3d_rust::id::ShapeId;
use box3d_rust::math_functions::{Pos, Vec3, POS_ZERO, VEC3_ZERO};
use box3d_rust::shape::shape_get_user_data;
use box3d_rust::world::World;

/// Mirror of C `CastContext` (`sample_collision.cpp:121`). Holds up to three hits;
/// the closest/any callbacks only fill `[0]`.
pub struct CastContext {
    pub points: [Pos; 3],
    pub normals: [Vec3; 3],
    pub fractions: [f32; 3],
    pub material_ids: [u64; 3],
    pub triangle_indices: [i32; 3],
    pub count: i32,
    pub initial_overlap: bool,
}

impl Default for CastContext {
    fn default() -> Self {
        Self {
            points: [POS_ZERO; 3],
            normals: [VEC3_ZERO; 3],
            // C initializes fractions to FLT_MAX before sorting; harmless for closest.
            fractions: [f32::MAX; 3],
            material_ids: [0; 3],
            triangle_indices: [0; 3],
            count: 0,
            initial_overlap: false,
        }
    }
}

/// Port of C `RayCastClosestCallback` (`sample_collision.cpp:134`): find the
/// closest hit, honoring the `initialOverlap` gate and the `userData == 1` ignore
/// flag. Returns the value the caller should clip the ray/cast to.
#[allow(clippy::too_many_arguments)]
pub fn cast_closest(
    world: &World,
    ctx: &mut CastContext,
    shape_id: ShapeId,
    point: Pos,
    normal: Vec3,
    fraction: f32,
    material_id: u64,
    triangle_index: i32,
) -> f32 {
    // Check for initial overlap.
    if !ctx.initial_overlap && fraction == 0.0 {
        return -1.0;
    }
    // Ignore a specific shape. Also ignore initial overlap.
    if shape_get_user_data(world, shape_id) == 1 {
        return -1.0;
    }

    ctx.points[0] = point;
    ctx.normals[0] = normal;
    ctx.fractions[0] = fraction;
    ctx.material_ids[0] = material_id;
    ctx.triangle_indices[0] = triangle_index;
    ctx.count = 1;

    // Clip the ray to this fraction and continue to the next shape.
    fraction
}
