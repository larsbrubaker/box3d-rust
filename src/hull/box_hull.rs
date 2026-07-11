//! Box hull constructors. (hull.c: b3Make*BoxHull, b3ScaleBox)

use super::types::{
    BoxHull, HullData, HullFace, HullHalfEdge, HullVertex, BOX_HULL_SIZE, HULL_VERSION,
};
use crate::constants::linear_slop;
use crate::core::{hash, non_zero_hash, HASH_INIT};
use crate::math_functions::{
    aabb_transform, abs, box_inertia, inv_rotate_vector, make_matrix_from_quat,
    make_plane_from_normal_and_point, make_quat_from_matrix, max, min, min_float, mul, mul_sv, neg,
    normalize, rotate_inertia, rotate_vector, transform_plane, transform_point, Aabb, Transform,
    Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ZERO,
};

fn box_hull_template() -> BoxHull {
    let edges: [HullHalfEdge; 24] = [
        HullHalfEdge {
            next: 2,
            twin: 1,
            origin: 2,
            face: 0,
        },
        HullHalfEdge {
            next: 17,
            twin: 0,
            origin: 1,
            face: 5,
        },
        HullHalfEdge {
            next: 4,
            twin: 3,
            origin: 1,
            face: 0,
        },
        HullHalfEdge {
            next: 20,
            twin: 2,
            origin: 5,
            face: 3,
        },
        HullHalfEdge {
            next: 6,
            twin: 5,
            origin: 5,
            face: 0,
        },
        HullHalfEdge {
            next: 23,
            twin: 4,
            origin: 6,
            face: 4,
        },
        HullHalfEdge {
            next: 0,
            twin: 7,
            origin: 6,
            face: 0,
        },
        HullHalfEdge {
            next: 18,
            twin: 6,
            origin: 2,
            face: 2,
        },
        HullHalfEdge {
            next: 10,
            twin: 9,
            origin: 0,
            face: 1,
        },
        HullHalfEdge {
            next: 21,
            twin: 8,
            origin: 3,
            face: 5,
        },
        HullHalfEdge {
            next: 12,
            twin: 11,
            origin: 3,
            face: 1,
        },
        HullHalfEdge {
            next: 16,
            twin: 10,
            origin: 7,
            face: 2,
        },
        HullHalfEdge {
            next: 14,
            twin: 13,
            origin: 7,
            face: 1,
        },
        HullHalfEdge {
            next: 19,
            twin: 12,
            origin: 4,
            face: 4,
        },
        HullHalfEdge {
            next: 8,
            twin: 15,
            origin: 4,
            face: 1,
        },
        HullHalfEdge {
            next: 22,
            twin: 14,
            origin: 0,
            face: 3,
        },
        HullHalfEdge {
            next: 7,
            twin: 17,
            origin: 3,
            face: 2,
        },
        HullHalfEdge {
            next: 9,
            twin: 16,
            origin: 2,
            face: 5,
        },
        HullHalfEdge {
            next: 11,
            twin: 19,
            origin: 6,
            face: 2,
        },
        HullHalfEdge {
            next: 5,
            twin: 18,
            origin: 7,
            face: 4,
        },
        HullHalfEdge {
            next: 15,
            twin: 21,
            origin: 1,
            face: 3,
        },
        HullHalfEdge {
            next: 1,
            twin: 20,
            origin: 0,
            face: 5,
        },
        HullHalfEdge {
            next: 3,
            twin: 23,
            origin: 4,
            face: 3,
        },
        HullHalfEdge {
            next: 13,
            twin: 22,
            origin: 5,
            face: 4,
        },
    ];

    // Offsets match offsetof in C b3BoxHull.
    let vertex_offset = 136;
    let point_offset = 144;
    let edge_offset = 240;
    let face_offset = 336;
    let plane_offset = 344;

    BoxHull {
        base: HullData {
            version: HULL_VERSION,
            byte_count: BOX_HULL_SIZE as i32,
            hash: 0,
            aabb: Aabb::default(),
            surface_area: 0.0,
            volume: 0.0,
            inner_radius: 0.0,
            center: VEC3_ZERO,
            central_inertia: crate::math_functions::MAT3_ZERO,
            vertex_count: 8,
            vertex_offset,
            point_offset,
            edge_count: 24,
            edge_offset,
            face_count: 6,
            face_offset,
            plane_offset,
            padding: 0,
            vertices: Vec::new(),
            points: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            planes: Vec::new(),
        },
        box_vertices: [
            HullVertex { edge: 8 },
            HullVertex { edge: 1 },
            HullVertex { edge: 0 },
            HullVertex { edge: 9 },
            HullVertex { edge: 13 },
            HullVertex { edge: 3 },
            HullVertex { edge: 5 },
            HullVertex { edge: 11 },
        ],
        box_points: [VEC3_ZERO; 8],
        box_edges: edges,
        box_faces: [
            HullFace { edge: 0 },
            HullFace { edge: 8 },
            HullFace { edge: 16 },
            HullFace { edge: 20 },
            HullFace { edge: 19 },
            HullFace { edge: 21 },
        ],
        padding: [0, 0],
        box_planes: [crate::math_functions::Plane {
            normal: VEC3_ZERO,
            offset: 0.0,
        }; 6],
    }
}

/// Make a transformed box as a hull. (b3MakeTransformedBoxHull)
pub fn make_transformed_box_hull(hx: f32, hy: f32, hz: f32, transform: Transform) -> BoxHull {
    let mut box_hull = box_hull_template();

    let min_h = 0.2 * linear_slop();
    let h = max(
        Vec3 {
            x: min_h,
            y: min_h,
            z: min_h,
        },
        Vec3 {
            x: hx,
            y: hy,
            z: hz,
        },
    );

    box_hull.base.aabb = aabb_transform(
        transform,
        Aabb {
            lower_bound: neg(h),
            upper_bound: h,
        },
    );
    box_hull.base.surface_area = 8.0 * (h.x * h.y + h.x * h.z + h.y * h.z);
    box_hull.base.volume = 8.0 * h.x * h.y * h.z;
    box_hull.base.inner_radius = min_float(h.x, min_float(h.y, h.z));
    box_hull.base.center = transform.p;

    let box_inertia = box_inertia(box_hull.base.volume, neg(h), h);
    box_hull.base.central_inertia = rotate_inertia(transform.q, box_inertia);

    let lower = neg(h);
    let upper = h;

    box_hull.box_planes[0] = transform_plane(
        transform,
        make_plane_from_normal_and_point(neg(VEC3_AXIS_X), lower),
    );
    box_hull.box_planes[1] = transform_plane(
        transform,
        make_plane_from_normal_and_point(VEC3_AXIS_X, upper),
    );
    box_hull.box_planes[2] = transform_plane(
        transform,
        make_plane_from_normal_and_point(neg(VEC3_AXIS_Y), lower),
    );
    box_hull.box_planes[3] = transform_plane(
        transform,
        make_plane_from_normal_and_point(VEC3_AXIS_Y, upper),
    );
    box_hull.box_planes[4] = transform_plane(
        transform,
        make_plane_from_normal_and_point(neg(VEC3_AXIS_Z), lower),
    );
    box_hull.box_planes[5] = transform_plane(
        transform,
        make_plane_from_normal_and_point(VEC3_AXIS_Z, upper),
    );

    box_hull.box_points[0] = transform_point(
        transform,
        Vec3 {
            x: h.x,
            y: h.y,
            z: h.z,
        },
    );
    box_hull.box_points[1] = transform_point(
        transform,
        Vec3 {
            x: -h.x,
            y: h.y,
            z: h.z,
        },
    );
    box_hull.box_points[2] = transform_point(
        transform,
        Vec3 {
            x: -h.x,
            y: -h.y,
            z: h.z,
        },
    );
    box_hull.box_points[3] = transform_point(
        transform,
        Vec3 {
            x: h.x,
            y: -h.y,
            z: h.z,
        },
    );
    box_hull.box_points[4] = transform_point(
        transform,
        Vec3 {
            x: h.x,
            y: h.y,
            z: -h.z,
        },
    );
    box_hull.box_points[5] = transform_point(
        transform,
        Vec3 {
            x: -h.x,
            y: h.y,
            z: -h.z,
        },
    );
    box_hull.box_points[6] = transform_point(
        transform,
        Vec3 {
            x: -h.x,
            y: -h.y,
            z: -h.z,
        },
    );
    box_hull.box_points[7] = transform_point(
        transform,
        Vec3 {
            x: h.x,
            y: -h.y,
            z: -h.z,
        },
    );

    // Keep base Vec accessors in sync with the embedded arrays (C offsets into
    // the same allocation). Hash still uses the contiguous byte layout.
    box_hull.sync_base_arrays();

    box_hull.base.hash = 0;
    let bytes = box_hull.to_bytes_with_hash(0);
    box_hull.base.hash = non_zero_hash(hash(HASH_INIT, &bytes));

    box_hull
}

/// Make a cube as a hull. (b3MakeCubeHull)
pub fn make_cube_hull(half_width: f32) -> BoxHull {
    make_box_hull(half_width, half_width, half_width)
}

/// Make an offset box as a hull. (b3MakeOffsetBoxHull)
pub fn make_offset_box_hull(hx: f32, hy: f32, hz: f32, offset: Vec3) -> BoxHull {
    let transform = Transform {
        p: offset,
        q: QUAT_IDENTITY,
    };
    make_transformed_box_hull(hx, hy, hz, transform)
}

/// Make a box as a hull. (b3MakeBoxHull)
pub fn make_box_hull(hx: f32, hy: f32, hz: f32) -> BoxHull {
    make_transformed_box_hull(hx, hy, hz, TRANSFORM_IDENTITY)
}

/// Scale a box's half-widths and transform by a post-scale. (b3ScaleBox)
pub fn scale_box(
    half_widths: &mut Vec3,
    transform: &mut Transform,
    post_scale: Vec3,
    min_half_width: f32,
) {
    debug_assert!(crate::math_functions::is_valid_float(min_half_width) && min_half_width > 0.0);

    let mut q = transform.q;

    if post_scale.x < 0.0 || post_scale.y < 0.0 || post_scale.z < 0.0 {
        let mut m = make_matrix_from_quat(q);
        m.cx.x *= post_scale.x;
        m.cy.x *= post_scale.x;
        m.cz.x *= post_scale.x;
        m.cx.y *= post_scale.y;
        m.cy.y *= post_scale.y;
        m.cz.y *= post_scale.y;
        m.cx.z *= post_scale.z;
        m.cy.z *= post_scale.z;
        m.cz.z *= post_scale.z;
        m.cx = normalize(m.cx);
        m.cy = normalize(m.cy);
        m.cz = normalize(m.cz);
        m.cx = if post_scale.x < 0.0 { neg(m.cx) } else { m.cx };
        m.cy = if post_scale.y < 0.0 { neg(m.cy) } else { m.cy };
        m.cz = if post_scale.z < 0.0 { neg(m.cz) } else { m.cz };
        q = make_quat_from_matrix(&m);
    }

    let abs_scale = abs(post_scale);
    let h = *half_widths;
    let p1 = mul(abs_scale, rotate_vector(q, neg(h)));
    let p2 = mul(abs_scale, rotate_vector(q, h));

    let local_p1 = inv_rotate_vector(q, p1);
    let local_p2 = inv_rotate_vector(q, p2);

    let lower = min(local_p1, local_p2);
    let upper = max(local_p1, local_p2);

    let scaled_half_width = mul_sv(0.5, crate::math_functions::sub(upper, lower));
    let m_limit = Vec3 {
        x: min_half_width,
        y: min_half_width,
        z: min_half_width,
    };
    *half_widths = max(scaled_half_width, m_limit);
    transform.p = mul(post_scale, transform.p);
    transform.q = q;
}

/// Make a scaled box hull. (b3MakeScaledBoxHull)
pub fn make_scaled_box_hull(half_widths: Vec3, transform: Transform, post_scale: Vec3) -> BoxHull {
    let mut h = half_widths;
    let mut xf = transform;
    scale_box(&mut h, &mut xf, post_scale, 4.0 * linear_slop());
    make_transformed_box_hull(h.x, h.y, h.z, xf)
}
