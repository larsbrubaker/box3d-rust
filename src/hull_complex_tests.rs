//! `b3CreateComplexHull` (upstream 9f998c8 "SAT samples"): 32 points on a sphere from PEEL's
//! BasicRandom LCG with seed 42, hulled with `b3CreateHull`.
//!
//! Expected values come from the scalar C build (`BOX3D_DISABLE_SIMD`) of the reference at
//! 9f998c8, printed as raw float bits. The hash was re-derived at 51f056e, whose hull layout
//! adds the edge cosine section, and again at e77352c, which pads the SOA arrays to 8.

use crate::hull::{create_complex_hull, get_hull_points, is_valid_hull};

const EXPECTED_POINTS: [[u32; 3]; 32] = [
    [0x3dc1f230, 0x3e086512, 0x3ef1f038],
    [0x3eee4586, 0xbdf7b838, 0xbe0c63d6],
    [0x3d805da8, 0xbec698e4, 0x3e9e51f8],
    [0x3e3be4b8, 0x3ee96d44, 0x3dbc8cb9],
    [0x3ba1afc0, 0x3e437318, 0xbeec998f],
    [0xbe4df43c, 0xbecae50a, 0xbe6aa71e],
    [0xbe96c475, 0xbcab56e8, 0x3ece9de9],
    [0xbe39cdaa, 0x3eda226a, 0x3e411b8c],
    [0x3ee1b596, 0x3e0ce103, 0x3e4442c8],
    [0x3e864272, 0xbe8af71e, 0xbea7ecf5],
    [0x3ea88f36, 0x3ea7f250, 0xbe3cddf3],
    [0xbecafc1e, 0xbe93136a, 0x3dcfead9],
    [0x3e908eda, 0xbecb8d74, 0x3de27683],
    [0x3eabfe76, 0x3e0f454b, 0xbeaf902d],
    [0x3ea770f6, 0x3ca6f626, 0x3ec15dc6],
    [0xba78dc00, 0xbea3db8a, 0xbec4b051],
    [0xbec6c82c, 0x3dba41a7, 0x3e9a7227],
    [0xbe04cd8c, 0xbea63d28, 0x3eb701bc],
    [0x3e55fd14, 0xbec97980, 0xbe6858ac],
    [0x3ee78810, 0x3e1d9183, 0xbe174a6c],
    [0x3e4f3798, 0xbe97488c, 0x3eb2a586],
    [0xbe4dd1a8, 0xbee1fef8, 0xbdf8e264],
    [0xbef20696, 0x3e25f800, 0xbc88d8ac],
    [0x3e8e3b00, 0x3e22876b, 0x3ec4bad0],
    [0xbefb802e, 0xbd834b41, 0xbd8ae945],
    [0x3ed77206, 0xbd8177f4, 0xbe866e4a],
    [0x3e904be6, 0x3e875f27, 0xbea2726f],
    [0xbe9c7b36, 0x3d51de9e, 0x3ec8e6c6],
    [0x3e769e70, 0xbed3a848, 0x3e14c10e],
    [0x3ed4317a, 0x3e08fc64, 0xbe7b8b92],
    [0x3c7ed1c0, 0xbeaa0fcc, 0xbebf2f4c],
    [0xbefb9652, 0x3d598904, 0xbd9af1df],
];

#[test]
fn complex_hull_matches_c() {
    let hull = create_complex_hull(0.5).expect("complex hull");
    assert!(is_valid_hull(&hull));

    assert_eq!(hull.vertex_count, 32);
    assert_eq!(hull.edge_count, 178);
    assert_eq!(hull.face_count, 59);
    assert_eq!(hull.hash, 0x7be7_e0b0_8f6b_251a);
    assert_eq!(hull.inner_radius.to_bits(), 0x3ebd97a2);
    assert_eq!(hull.volume.to_bits(), 0x3ebb3c07);
    assert_eq!(hull.surface_area.to_bits(), 0x40280b96);
    assert_eq!(hull.center.x.to_bits(), 0x3c91d1ec);
    assert_eq!(hull.center.y.to_bits(), 0xbc8bf8d8);
    assert_eq!(hull.center.z.to_bits(), 0x3c941b10);

    let points = get_hull_points(&hull);
    for (i, (p, e)) in points.iter().zip(EXPECTED_POINTS.iter()).enumerate() {
        assert_eq!(
            [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()],
            *e,
            "point {i}"
        );
    }
}
