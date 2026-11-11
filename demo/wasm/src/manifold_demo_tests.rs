//! Point-capacity regression tests for the manifold viewers. Guards the demo against
//! truncating manifolds the C samples show in full: `b3CollideTriangleFace` /
//! `b3CollideHullFace` keep only the first `capacity` clip points without reducing,
//! so a too-small buffer silently drops points. `#[path]`-included by `manifold_demo.rs`.

use super::*;

/// Every scene's point buffer and collide capacity match its C base class:
/// `Manifold::m_pointCapacity = 64`, `TriangleManifold::m_pointCapacity = 8`.
#[test]
fn scene_point_buffers_match_c_capacities() {
    for scene in 0..=8 {
        let demo = ManifoldDemo::new(scene);
        let expected = if demo.kind.is_triangle() { 8 } else { 64 };
        assert_eq!(
            demo.manifold.points.len(),
            expected,
            "scene {scene} point buffer length"
        );
        assert_eq!(
            demo.kind.point_capacity(),
            expected,
            "scene {scene} collide capacity"
        );
    }
}

/// TriangleAndHull at the C constructor pose clips to 5 face points. The demo must
/// report all of them, matching an uncapped `collide_triangle_and_hull` call.
#[test]
fn triangle_and_hull_default_pose_keeps_every_clip_point() {
    let mut demo = ManifoldDemo::new(8);

    let local = demo.local_triangle_in_b();
    let mut reference_points = [LocalManifoldPoint::default(); 64];
    let mut reference = LocalManifold::new(&mut reference_points);
    collide_triangle_and_hull(
        &mut reference,
        64,
        local[0],
        local[1],
        local[2],
        &demo.hull_b.base,
        &mut SatCache::default(),
        true,
    );
    assert!(
        reference.point_count > 4,
        "pose no longer exercises truncation: {} points",
        reference.point_count
    );

    demo.step();
    assert_eq!(demo.manifold.point_count, reference.point_count);
    for i in 0..reference.point_count as usize {
        let got = demo.manifold.points[i];
        let want = reference.points[i];
        assert_eq!(got.point, want.point, "point {i}");
        assert_eq!(
            got.separation.to_bits(),
            want.separation.to_bits(),
            "separation {i}"
        );
        assert_eq!(got.pair, want.pair, "pair {i}");
    }

    let packed = demo.pack_step();
    assert_eq!(packed[0] as i32, reference.point_count);
    assert_eq!(packed.len(), 7 + 8 * reference.point_count as usize);
}
