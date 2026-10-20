//! Upstream e77352c "Add support for AVX2 and runtime feature detection": hull SOA arrays are
//! padded to a multiple of 8 (`B3_HULL_SOA_PADDING`, `b3GetHullSoaStride`) so the AVX2 kernels
//! can load full lanes, the hull version changes, and the runtime SIMD width queries
//! (`b3IsAVX2Available`, `b3GetSIMDWidth`, `b3SetSIMDWidth`, `b3World_EnableSSE2Fallback`)
//! report the scalar build's answers.
//!
//! Expected values come from the scalar C build (`BOX3D_DISABLE_SIMD`) of the reference at
//! e77352c.

use crate::core::is_avx2_available;
use crate::hull::{
    convert_bytes_to_hull, create_complex_hull, create_cone, create_cylinder, create_rock,
    find_hull_support_vertex, get_hull_points, get_hull_soa_normals, get_hull_soa_stride,
    get_hull_soa_vertices, hull_soa_normal_count, hull_soa_vertex_count, make_box_hull, HullData,
    HULL_SOA_PADDING, HULL_VERSION,
};
use crate::math_functions::{dot, Vec3};
use crate::simd_scalar::{get_simd_width, set_simd_width};
use crate::types::default_world_def;
use crate::world::{world_enable_sse2_fallback, World};

/// Layout fields printed by the C harness: byteCount, soaVertexOffset, soaNormalOffset,
/// edgeCosineOffset, hash.
fn assert_layout(hull: &HullData, expected: (i32, i32, i32, i32, u64), name: &str) {
    assert_eq!(hull.version, HULL_VERSION, "{name} version");
    assert_eq!(hull.byte_count, expected.0, "{name} byte count");
    assert_eq!(
        hull.soa_vertex_offset, expected.1,
        "{name} soa vertex offset"
    );
    assert_eq!(
        hull.soa_normal_offset, expected.2,
        "{name} soa normal offset"
    );
    assert_eq!(
        hull.edge_cosine_offset, expected.3,
        "{name} edge cosine offset"
    );
    assert_eq!(hull.hash, expected.4, "{name} hash");
}

#[test]
fn hull_soa_stride_pads_to_eight() {
    assert_eq!(HULL_SOA_PADDING, 8);
    assert_eq!(get_hull_soa_stride(0), 0);
    assert_eq!(get_hull_soa_stride(1), 8);
    assert_eq!(get_hull_soa_stride(8), 8);
    assert_eq!(get_hull_soa_stride(9), 16);
    assert_eq!(get_hull_soa_stride(59), 64);
    assert_eq!(HULL_VERSION, 0x6B1E_39D4_A87C_25F3);
}

#[test]
fn hull_soa_layouts_match_c() {
    let box_hull = make_box_hull(1.0, 2.0, 3.0);
    assert_layout(
        &box_hull.base,
        (696, 456, 552, 648, 0xfc96_8302_32f7_676a),
        "box",
    );

    let rock = create_rock(0.5).expect("rock");
    assert_eq!((rock.vertex_count, rock.face_count), (10, 16));
    assert_layout(&rock, (1232, 752, 944, 1136, 0xb2ce_028d_99c2_136c), "rock");

    let cylinder = create_cylinder(1.0, 0.1, -0.5, 32).expect("cylinder");
    assert_eq!((cylinder.vertex_count, cylinder.face_count), (64, 34));
    assert_layout(
        &cylinder,
        (3968, 2336, 3104, 3584, 0xd895_1069_4118_1615),
        "cylinder",
    );

    let complex = create_complex_hull(0.5).expect("complex hull");
    assert_eq!((complex.vertex_count, complex.face_count), (32, 59));
    assert_layout(
        &complex,
        (3800, 2288, 2672, 3440, 0x7be7_e0b0_8f6b_251a),
        "complex",
    );

    let cone = create_cone(0.8, 0.5, 0.2, 7).expect("cone");
    assert_eq!((cone.vertex_count, cone.face_count), (14, 9));
    assert_layout(&cone, (1136, 664, 856, 1048, 0xc97d_e9b6_f6f3_abc3), "cone");
}

// Padded vertex lanes repeat the first vertex and padded normal lanes are zero, now out to the
// multiple of 8. The cone has 14 vertices and 9 faces, so both strides are 16.
#[test]
fn cone_soa_tail_padding_matches_c() {
    let cone = create_cone(0.8, 0.5, 0.2, 7).expect("cone");
    let vertex_stride = hull_soa_vertex_count(&cone);
    let normal_stride = hull_soa_normal_count(&cone);
    assert_eq!(vertex_stride, 16);
    assert_eq!(normal_stride, 16);

    let soa_vertices = get_hull_soa_vertices(&cone);
    assert_eq!(soa_vertices.len(), 3 * vertex_stride);
    assert_eq!(soa_vertices[0].to_bits(), 0xbde5_80e0);
    for axis in 0..3 {
        let first = soa_vertices[axis * vertex_stride];
        for i in cone.vertex_count as usize..vertex_stride {
            assert_eq!(
                soa_vertices[axis * vertex_stride + i].to_bits(),
                first.to_bits(),
                "vertex axis {axis} lane {i}"
            );
        }
    }

    let soa_normals = get_hull_soa_normals(&cone);
    assert_eq!(soa_normals.len(), 3 * normal_stride);
    for axis in 0..3 {
        for i in cone.face_count as usize..normal_stride {
            assert_eq!(
                soa_normals[axis * normal_stride + i].to_bits(),
                0,
                "normal axis {axis} lane {i}"
            );
        }
    }

    // The serialized blob carries the wider stride and converts back unchanged.
    let bytes = cone.to_bytes();
    assert_eq!(bytes.len(), cone.byte_count as usize);
    let restored = convert_bytes_to_hull(&bytes).expect("restored cone");
    assert_eq!(restored.soa_vertices, cone.soa_vertices);
    assert_eq!(restored.soa_normals, cone.soa_normals);
    assert_eq!(restored.to_bytes(), bytes);
}

// b3FindHullSupportVertex keeps its 4 wide loop over the 8 padded stride, so the support
// vertex is still the first vertex with the largest dot product.
#[test]
fn support_vertex_with_wider_stride() {
    let cone = create_cone(0.8, 0.5, 0.2, 7).expect("cone");
    let points = get_hull_points(&cone);
    let directions = [
        Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        Vec3 {
            x: -0.3,
            y: 0.8,
            z: 0.5,
        },
        Vec3 {
            x: 0.2,
            y: -1.0,
            z: -0.7,
        },
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        },
    ];

    for d in directions {
        let mut best = 0usize;
        for (i, p) in points.iter().enumerate().skip(1) {
            if dot(*p, d) > dot(points[best], d) {
                best = i;
            }
        }
        assert_eq!(find_hull_support_vertex(&cone, d), best as i32);
    }
}

// The scalar build never has AVX2 kernels: b3IsAVX2Available is false, the detected width is
// 4, and asking for 8 falls back to 4.
#[test]
fn simd_width_queries_report_scalar_build() {
    assert!(!is_avx2_available());
    assert_eq!(get_simd_width(), 4);

    set_simd_width(8);
    assert_eq!(get_simd_width(), 4);
    set_simd_width(4);
    assert_eq!(get_simd_width(), 4);
    set_simd_width(0);
    assert_eq!(get_simd_width(), 4);
}

#[test]
fn world_sse2_fallback_keeps_width_four() {
    let mut world = World::new(&default_world_def());
    assert_eq!(world.simd_width, 4);

    world_enable_sse2_fallback(&mut world, true);
    assert_eq!(world.simd_width, 4);

    world_enable_sse2_fallback(&mut world, false);
    assert_eq!(world.simd_width, get_simd_width());
}
