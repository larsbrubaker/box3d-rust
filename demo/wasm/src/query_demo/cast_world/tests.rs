//! Guards Cast World spawn / cast packing against silent regressions.
//!
//! Layout of `query_cast`: `[count, ox,oy,oz, tx,ty,tz, cast_type, radius, hits...]`
//! each hit: `px,py,pz, nx,ny,nz, fraction, material, triangle` (stride 9).
//! Ignore AABBs: bodies whose slot index satisfies `(i & 0x7) == 0x7`.

use super::*;

#[test]
fn reset_starts_empty_with_default_ray() {
    query_reset();
    assert_eq!(
        query_step(1.0 / 60.0, 4),
        0,
        "C CastWorld starts with no bodies"
    );

    let data = query_cast();
    assert!(data.len() >= 9);
    assert_eq!(data[0], 0.0, "empty world -> zero hits");
    assert_eq!(data[1], -20.0);
    assert_eq!(data[2], 10.0);
    assert_eq!(data[3], 0.0);
    assert_eq!(data[4], 20.0);
    assert_eq!(data[5], 10.0);
    assert_eq!(data[6], 0.0);
    assert_eq!(data[7], CAST_RAY as f32);
    assert_eq!(data[8], 0.5);
}

#[test]
fn spawn_counts_and_destroy_match_c_buttons() {
    query_reset();
    // ShapeType discriminant values match geometry::ShapeType / TS SHAPE_* constants.
    assert_eq!(query_add_shapes(5, 10), 10, "Spheres x 10");
    assert_eq!(query_add_shapes(0, 10), 20, "Capsules x 10");
    assert_eq!(query_add_shapes(3, 10), 30, "Hulls x 10");
    assert_eq!(query_add_shapes(4, 1), 31, "Meshes x 1");
    assert_eq!(query_add_shapes(2, 1), 32, "Height Field x 1");
    assert_eq!(query_destroy_shape(), 31, "Destroy Shape removes one body");
}

#[test]
fn ignore_aabb_slots_match_ignore_base() {
    query_reset();
    // Indices 0..9 filled; only slot 7 satisfies (i & 0x7) == 0x7.
    assert_eq!(query_add_shapes(5, 10), 10);
    let aabbs = query_ignore_aabbs();
    assert_eq!(aabbs[0] as i32, 1, "exactly one ignored sphere in first 10");
    assert_eq!(aabbs.len(), 1 + 6);
}

#[test]
fn cast_packing_reports_hit_when_sphere_on_ray() {
    query_reset();
    assert_eq!(query_add_shapes(5, 1), 1);
    let poses = query_poses();
    // VisBody packing: [px, py, pz, qx, qy, qz, qw, a0..a6, kind, color]
    assert!(
        poses.len() >= crate::vis::POSE_STRIDE,
        "expected at least one pose group, got {}",
        poses.len()
    );
    let px = poses[0];
    let py = poses[1];
    let pz = poses[2];
    query_set_ray(px - 10.0, py, pz, 20.0, 0.0, 0.0);
    query_set_params(CAST_RAY, MODE_CLOSEST, 0.5, 0);

    let data = query_cast();
    assert_eq!(data[0] as i32, 1, "ray through sphere center should hit");
    assert_eq!(data.len(), 9 + HIT_STRIDE);
    // material id for spheres is 11
    assert_eq!(data[9 + 7] as i32, 11);
}
