//! Per-edge face normal cosines on hulls (upstream 51f056e "SAT benchmark"):
//! `b3GetHullEdgeCosines`, the `edgeCosineOffset` section and the grown `b3HullData`
//! (152 bytes) / `b3BoxHull` (696 bytes) layouts.
//!
//! Expected values come from the scalar C build (`BOX3D_DISABLE_SIMD`) of the reference at
//! 51f056e, printed as raw float bits. The hashes and the version were re-derived at e77352c,
//! which pads the SOA arrays to a multiple of 8.

use crate::hull::{
    clone_and_transform_hull, convert_bytes_to_hull, create_complex_hull, create_cylinder,
    create_rock, get_hull_edge_cosines, make_box_hull, BOX_HULL_SIZE, HULL_BYTE_COUNT_OFFSET,
    HULL_DATA_SIZE, HULL_VERSION,
};
use crate::math_functions::{normalize_quat, Quat, Transform, Vec3};

/// b3GetHullEdgeCosines( b3CreateComplexHull( 0.5f ) ), one per full edge.
const COMPLEX_EDGE_COSINES: [u32; 89] = [
    0x3f6472fd, 0x3f7e7e88, 0x3ed0ea25, 0x3f7c82ad, 0x3f75b85c, 0x3f06d200, 0x3f6daac5, 0x3f7ee83a,
    0x3f511d00, 0x3f640c82, 0x3f7fdd1a, 0x3f7a5a8c, 0x3f7e730b, 0x3f67834f, 0x3f7e2de8, 0x3f4d5e56,
    0x3f2e335c, 0x3f7ffff7, 0x3f283f77, 0x3f79b1ff, 0x3f74defc, 0x3f74163b, 0x3f7cd402, 0x3f714e8c,
    0x3f3fab14, 0x3f4fb83e, 0x3f55a7c7, 0x3f79f792, 0x3f7c9d8a, 0x3f59681f, 0x3f63c7a0, 0x3f56d96a,
    0x3f6f4fcc, 0x3f36d70b, 0x3f71a64a, 0x3f7810d9, 0x3f797809, 0x3f28af2e, 0x3f583ede, 0x3f730ba9,
    0x3f7cfc98, 0x3f40e1dd, 0x3f7f6f31, 0x3f24e750, 0x3f7ffcd8, 0x3f70c2c4, 0x3f771c9e, 0x3f52e0e8,
    0x3f67106f, 0x3f264d38, 0x3f656f62, 0x3f5c0730, 0x3ee3c198, 0x3f7ffdf2, 0x3f65af96, 0x3f65405e,
    0x3f43733a, 0x3f64bfdd, 0x3f39bfcf, 0x3f7bd19d, 0x3ebf49a5, 0x3f7f6cec, 0x3f3fa927, 0x3f7f8978,
    0x3f5cdf4f, 0x3f58ec10, 0x3f6b233a, 0x3f6250f6, 0x3f2b5612, 0x3f2038ee, 0x3f51d7d6, 0x3f4f6424,
    0x3f669c7c, 0x3f77a2a2, 0x3f794a3e, 0x3f7ab0d5, 0x3f3c767e, 0x3f737130, 0x3f546812, 0x3f74ef9a,
    0x3f6f7e81, 0x3f7f235a, 0x3f7e9d60, 0x3f67e872, 0x3f64f007, 0x3f736b6a, 0x3f4dfa19, 0x3f75ce9e,
    0x3f719c2f,
];

#[test]
fn hull_layout_constants_match_c() {
    assert_eq!(HULL_VERSION, 0x6B1E_39D4_A87C_25F3);
    assert_eq!(HULL_DATA_SIZE, 152);
    assert_eq!(HULL_BYTE_COUNT_OFFSET, 144);
    assert_eq!(BOX_HULL_SIZE, 696);
}

#[test]
fn complex_hull_edge_cosines_match_c() {
    let hull = create_complex_hull(0.5).expect("complex hull");
    assert_eq!(hull.edge_count, 178);

    let cosines = get_hull_edge_cosines(&hull);
    assert_eq!(cosines.len(), COMPLEX_EDGE_COSINES.len());
    for (i, (c, e)) in cosines.iter().zip(COMPLEX_EDGE_COSINES.iter()).enumerate() {
        assert_eq!(c.to_bits(), *e, "edge {i}");
    }

    // The cosine section follows the SOA normals and closes the blob.
    let soa_normal_count = ((hull.face_count + 7) & !7) as usize;
    let soa_normal_end = hull.soa_normal_offset as usize + 3 * soa_normal_count * 4;
    assert_eq!(hull.edge_cosine_offset as usize, (soa_normal_end + 7) & !7);
    assert_eq!(
        hull.byte_count as usize,
        hull.edge_cosine_offset as usize + ((89 * 4 + 7) & !7)
    );
}

#[test]
fn hull_hashes_cover_edge_cosines() {
    assert_eq!(
        make_box_hull(1.0, 2.0, 3.0).base.hash,
        0xfc96_8302_32f7_676a
    );
    assert_eq!(create_rock(0.5).expect("rock").hash, 0xb2ce_028d_99c2_136c);
    assert_eq!(
        create_cylinder(1.0, 0.1, -0.5, 32).expect("cylinder").hash,
        0xd895_1069_4118_1615
    );
}

// b3CloneAndTransformHull recomputes the cosines after rebuilding the planes. A reflecting
// scale reverses the winding, so the cosines follow the transformed face normals.
#[test]
fn clone_and_transform_hull_updates_edge_cosines() {
    let complex = create_complex_hull(0.5).expect("complex hull");
    let q = normalize_quat(Quat {
        v: Vec3 {
            x: 0.1,
            y: 0.2,
            z: 0.3,
        },
        s: 0.9,
    });
    let xf = Transform {
        p: Vec3 {
            x: 0.5,
            y: -1.0,
            z: 2.0,
        },
        q,
    };
    let scale = Vec3 {
        x: -1.5,
        y: 1.0,
        z: 2.0,
    };
    let moved = clone_and_transform_hull(&complex, xf, scale).expect("moved hull");
    assert_eq!(moved.edge_count, 178);
    assert_eq!(moved.hash, 0x0e06_fb28_6fcb_2053);

    let expected = [
        0x3f690f70u32,
        0x3f7ef19e,
        0x3f4abacb,
        0x3f7e49b9,
        0x3f7a000b,
        0x3f3e25d6,
        0x3f785bc8,
        0x3f7f0e6e,
    ];
    let cosines = get_hull_edge_cosines(&moved);
    for (i, e) in expected.iter().enumerate() {
        assert_eq!(cosines[i].to_bits(), *e, "edge {i}");
    }
}

// C leaves the box template's edge cosines zero (static storage) and b3MakeTransformedBoxHull
// never fills them in, so box hulls carry zero cosines even though the offset is set.
#[test]
fn box_hull_edge_cosines_are_zero() {
    let box_hull = make_box_hull(1.0, 2.0, 3.0);
    assert_eq!(box_hull.base.edge_cosine_offset, 648);
    assert_eq!(box_hull.base.byte_count as usize, BOX_HULL_SIZE);

    let cosines = get_hull_edge_cosines(&box_hull.base);
    assert_eq!(cosines.len(), 12);
    assert!(cosines.iter().all(|c| c.to_bits() == 0));

    let bytes = box_hull.to_bytes();
    assert_eq!(bytes.len(), BOX_HULL_SIZE);
    assert!(bytes[648..696].iter().all(|b| *b == 0));
}

#[test]
fn convert_bytes_to_hull_round_trips_edge_cosines() {
    let hull = create_complex_hull(0.5).expect("complex hull");
    let bytes = hull.to_bytes();
    assert_eq!(bytes.len(), hull.byte_count as usize);

    let restored = convert_bytes_to_hull(&bytes).expect("restored hull");
    assert_eq!(restored.edge_cosine_offset, hull.edge_cosine_offset);
    assert_eq!(restored.edge_cosines, hull.edge_cosines);
    assert_eq!(restored.to_bytes(), bytes);
}

// A blob with edges but a zero edge cosine offset would leave the cosine slice empty and
// panic the separating axis test, so the conversion rejects it.
#[test]
fn convert_bytes_to_hull_rejects_missing_edge_cosines() {
    let hull = create_complex_hull(0.5).expect("complex hull");
    let mut bytes = hull.to_bytes();
    bytes[140..144].copy_from_slice(&0i32.to_le_bytes());
    assert!(convert_bytes_to_hull(&bytes).is_none());
}
