//! Port of box3d-cpp-reference/test/test_hull.c

use crate::hull::{
    clone_hull, compare_hull_data, create_cylinder, create_hull, destroy_hull, make_box_hull,
};
use crate::math_functions::{abs_float, sub_mm, Vec3, PI};

const CUBE_CORNERS: [Vec3; 8] = [
    Vec3 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    },
    Vec3 {
        x: -1.0,
        y: 1.0,
        z: 1.0,
    },
    Vec3 {
        x: -1.0,
        y: -1.0,
        z: 1.0,
    },
    Vec3 {
        x: 1.0,
        y: -1.0,
        z: 1.0,
    },
    Vec3 {
        x: 1.0,
        y: 1.0,
        z: -1.0,
    },
    Vec3 {
        x: -1.0,
        y: 1.0,
        z: -1.0,
    },
    Vec3 {
        x: -1.0,
        y: -1.0,
        z: -1.0,
    },
    Vec3 {
        x: 1.0,
        y: -1.0,
        z: -1.0,
    },
];

fn ensure_small(v: f32, tol: f32) {
    assert!(
        abs_float(v) <= tol,
        "expected |{v}| <= {tol}"
    );
}

#[test]
fn create_hull_cube() {
    let hull = create_hull(&CUBE_CORNERS, 8).expect("cube hull");
    assert_eq!(hull.vertex_count, 8);
    assert_eq!(hull.edge_count, 24);
    assert_eq!(hull.face_count, 6);
    assert_eq!(
        hull.vertex_count - hull.edge_count / 2 + hull.face_count,
        2
    );

    let ref_hull = make_box_hull(1.0, 1.0, 1.0);
    ensure_small(hull.volume - ref_hull.base.volume, 1e-4);
    ensure_small(hull.surface_area - ref_hull.base.surface_area, 1e-4);
    ensure_small(hull.inner_radius - ref_hull.base.inner_radius, f32::EPSILON);
    ensure_small(hull.center.x - ref_hull.base.center.x, 1e-5);
    ensure_small(hull.center.y - ref_hull.base.center.y, 1e-5);
    ensure_small(hull.center.z - ref_hull.base.center.z, 1e-5);

    ensure_small(hull.aabb.lower_bound.x + 1.0, f32::EPSILON);
    ensure_small(hull.aabb.lower_bound.y + 1.0, f32::EPSILON);
    ensure_small(hull.aabb.lower_bound.z + 1.0, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.x - 1.0, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.y - 1.0, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.z - 1.0, f32::EPSILON);

    let d = sub_mm(hull.central_inertia, ref_hull.base.central_inertia);
    ensure_small(d.cx.x, 1e-4);
    ensure_small(d.cy.y, 1e-4);
    ensure_small(d.cz.z, 1e-4);
    ensure_small(d.cx.y, 1e-4);
    ensure_small(d.cx.z, 1e-4);
    ensure_small(d.cy.z, 1e-4);
    ensure_small(d.cy.x, 1e-4);
    ensure_small(d.cz.x, 1e-4);
    ensure_small(d.cz.y, 1e-4);

    destroy_hull(hull);
}

#[test]
fn create_hull_tetrahedron() {
    let points = [
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
    ];
    let hull = create_hull(&points, 4).expect("tet hull");
    assert_eq!(hull.vertex_count, 4);
    assert_eq!(hull.edge_count, 12);
    assert_eq!(hull.face_count, 4);
    assert_eq!(
        hull.vertex_count - hull.edge_count / 2 + hull.face_count,
        2
    );

    let expected_volume = 1.0 / 6.0;
    let expected_surface_area = 1.5 + 0.5 * 3.0f32.sqrt();
    let expected_inner_radius = 0.25 / 3.0f32.sqrt();

    ensure_small(hull.volume - expected_volume, 1e-5);
    ensure_small(hull.surface_area - expected_surface_area, 1e-5);
    ensure_small(hull.inner_radius - expected_inner_radius, 1e-5);
    ensure_small(hull.center.x - 0.25, 1e-5);
    ensure_small(hull.center.y - 0.25, 1e-5);
    ensure_small(hull.center.z - 0.25, 1e-5);

    ensure_small(hull.aabb.lower_bound.x, f32::EPSILON);
    ensure_small(hull.aabb.lower_bound.y, f32::EPSILON);
    ensure_small(hull.aabb.lower_bound.z, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.x - 1.0, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.y - 1.0, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.z - 1.0, f32::EPSILON);

    destroy_hull(hull);
}

#[test]
fn create_hull_determinism() {
    let h1 = create_hull(&CUBE_CORNERS, 8).expect("h1");
    let h2 = create_hull(&CUBE_CORNERS, 8).expect("h2");
    assert_eq!(h1.byte_count, h2.byte_count);
    assert_ne!(h1.hash, 0);
    assert_eq!(h1.hash, h2.hash);
    assert!(compare_hull_data(&h1, &h2));
    destroy_hull(h1);
    destroy_hull(h2);
}

const SPHERE_N: usize = 6;

#[test]
fn create_hull_max_vertex() {
    let mut points = [Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }; SPHERE_N * SPHERE_N];
    let mut index = 0;
    for i in 0..SPHERE_N {
        let theta = PI * i as f32 / (SPHERE_N - 1) as f32;
        for j in 0..SPHERE_N {
            let phi = 2.0 * PI * j as f32 / SPHERE_N as f32;
            // Use libm-style sin/cos via Box3D determinism helpers for theta/phi? C uses sinf/cosf.
            // Sphere sample in the C test uses sinf/cosf (platform), not b3Sin — match that.
            points[index] = Vec3 {
                x: theta.sin() * phi.cos(),
                y: theta.sin() * phi.sin(),
                z: theta.cos(),
            };
            index += 1;
        }
    }
    let _count = (SPHERE_N * SPHERE_N) as i32;

    let h1 = create_hull(&points, 8).expect("h1");
    assert!(h1.vertex_count <= 8);
    destroy_hull(h1);

    let h2 = create_hull(&points, 1).expect("h2");
    assert!(h2.vertex_count >= 4 && h2.vertex_count <= 255);
    destroy_hull(h2);

    let h3 = create_hull(&points, 1000).expect("h3");
    assert!(h3.vertex_count >= 4 && h3.vertex_count <= 255);
    destroy_hull(h3);
}

#[test]
fn create_hull_redundant_input() {
    let points = [
        Vec3 { x: 1.0, y: 1.0, z: 1.0 },
        Vec3 { x: -1.0, y: 1.0, z: 1.0 },
        Vec3 { x: -1.0, y: -1.0, z: 1.0 },
        Vec3 { x: 1.0, y: -1.0, z: 1.0 },
        Vec3 { x: 1.0, y: 1.0, z: -1.0 },
        Vec3 { x: -1.0, y: 1.0, z: -1.0 },
        Vec3 { x: -1.0, y: -1.0, z: -1.0 },
        Vec3 { x: 1.0, y: -1.0, z: -1.0 },
        Vec3 { x: 1.0, y: 1.0, z: 1.0 },
        Vec3 { x: 1.0, y: 1.0, z: 1.0 },
        Vec3 { x: 0.0, y: 0.0, z: 0.0 },
        Vec3 { x: 0.5, y: 0.0, z: 0.0 },
        Vec3 { x: 0.0, y: 0.5, z: 0.0 },
        Vec3 { x: 0.0, y: 0.0, z: 0.5 },
        Vec3 { x: -0.5, y: 0.0, z: 0.0 },
        Vec3 { x: 0.0, y: -0.5, z: 0.0 },
        Vec3 { x: 0.0, y: 0.0, z: -0.5 },
        Vec3 { x: 0.25, y: 0.25, z: 0.25 },
        Vec3 { x: -0.25, y: -0.25, z: -0.25 },
        Vec3 { x: 0.5, y: 0.5, z: 0.5 },
    ];

    let hull = create_hull(&points, 8).expect("redundant");
    assert_eq!(hull.vertex_count, 8);
    assert_eq!(hull.edge_count, 24);
    assert_eq!(hull.face_count, 6);

    let ref_hull = make_box_hull(1.0, 1.0, 1.0);
    ensure_small(hull.volume - ref_hull.base.volume, 1e-4);
    ensure_small(hull.surface_area - ref_hull.base.surface_area, 1e-4);
    ensure_small(hull.inner_radius - ref_hull.base.inner_radius, f32::EPSILON);
    ensure_small(hull.center.x - ref_hull.base.center.x, 1e-5);
    ensure_small(hull.center.y - ref_hull.base.center.y, 1e-5);
    ensure_small(hull.center.z - ref_hull.base.center.z, 1e-5);
    ensure_small(hull.aabb.lower_bound.x + 1.0, f32::EPSILON);
    ensure_small(hull.aabb.lower_bound.y + 1.0, f32::EPSILON);
    ensure_small(hull.aabb.lower_bound.z + 1.0, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.x - 1.0, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.y - 1.0, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.z - 1.0, f32::EPSILON);

    destroy_hull(hull);
}

#[test]
fn create_hull_clone() {
    let original = create_hull(&CUBE_CORNERS, 8).expect("original");
    let clone = clone_hull(&original).expect("clone");
    assert_eq!(clone.byte_count, original.byte_count);
    assert!(compare_hull_data(&clone, &original));
    destroy_hull(clone);
    destroy_hull(original);
}

#[test]
fn create_hull_cylinder() {
    let height = 2.0f32;
    let radius = 1.0f32;
    let sides = 8;
    let y_offset = 0.0f32;

    let hull = create_cylinder(height, radius, y_offset, sides).expect("cylinder");
    assert_eq!(hull.vertex_count, 2 * sides);
    assert_eq!(hull.edge_count, 6 * sides);
    assert_eq!(hull.face_count, sides + 2);

    let half_angle = PI / sides as f32;
    let cap_area = sides as f32 * 0.5 * radius * radius * (2.0 * half_angle).sin();
    let chord_len = 2.0 * radius * half_angle.sin();
    let lateral_area = sides as f32 * chord_len * height;
    let expected_volume = cap_area * height;
    let expected_surface_area = 2.0 * cap_area + lateral_area;
    let expected_inner_radius = radius * half_angle.cos();

    ensure_small((hull.volume - expected_volume) / expected_volume, 1e-4);
    ensure_small(
        (hull.surface_area - expected_surface_area) / expected_surface_area,
        1e-4,
    );
    ensure_small(hull.inner_radius - expected_inner_radius, 1e-5);
    ensure_small(hull.center.x, 1e-5);
    ensure_small(hull.center.y - (y_offset + 0.5 * height), 1e-5);
    ensure_small(hull.center.z, 1e-5);

    ensure_small(hull.aabb.lower_bound.x + radius, f32::EPSILON);
    ensure_small(hull.aabb.lower_bound.y - y_offset, f32::EPSILON);
    ensure_small(hull.aabb.lower_bound.z + radius, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.x - radius, f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.y - (y_offset + height), f32::EPSILON);
    ensure_small(hull.aabb.upper_bound.z - radius, f32::EPSILON);

    destroy_hull(hull);
}

fn fill_sphere_sample(points: &mut [Vec3], seed: u32) {
    const RAND_LIMIT_LOCAL: u32 = 32767;
    let mut seed = seed;
    for point in points.iter_mut() {
        let mut u = [0.0f32; 3];
        for k in 0..3 {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            let mut r = (seed & RAND_LIMIT_LOCAL) as f32;
            r /= RAND_LIMIT_LOCAL as f32;
            u[k] = r;
        }
        let u1 = u[0];
        let u2 = 2.0 * PI * u[1];
        let u3 = 2.0 * PI * u[2];
        let sqrt1_minus_u1 = (1.0 - u1).sqrt();
        let sqrt_u1 = u1.sqrt();
        // C uses sinf/cosf here.
        point.x = sqrt1_minus_u1 * u2.sin();
        point.y = sqrt1_minus_u1 * u2.cos();
        point.z = sqrt_u1 * u3.sin();
    }
}

fn fill_cube_sample(points: &mut [Vec3], seed: u32) {
    const RAND_LIMIT_LOCAL: u32 = 32767;
    let mut seed = seed;
    for point in points.iter_mut() {
        let mut v = [0.0f32; 3];
        for k in 0..3 {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            let mut r = (seed & RAND_LIMIT_LOCAL) as f32;
            r /= RAND_LIMIT_LOCAL as f32;
            v[k] = 2.0 * r - 1.0;
        }
        point.x = v[0];
        point.y = v[1];
        point.z = v[2];
    }
}

#[test]
fn create_hull_sphere_reduction() {
    let mut points = [Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }; 64];
    fill_sphere_sample(&mut points, 12345);
    let hull = create_hull(&points, 20).expect("sphere reduction");
    assert!(hull.vertex_count >= 4 && hull.vertex_count <= 20);
    assert_eq!(
        hull.vertex_count - hull.edge_count / 2 + hull.face_count,
        2
    );
    destroy_hull(hull);
}

#[test]
fn create_hull_sphere_stress() {
    const N: usize = 512;
    let mut points = [Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }; N];
    let seeds = [12345u32, 1, 0xdeadbeef, 0xcafef00d];
    let m_values = [16, 24, 32, 40];

    for &seed in &seeds {
        fill_sphere_sample(&mut points, seed);
        for &m in &m_values {
            let hull = create_hull(&points, m).expect("sphere stress");
            assert!(hull.vertex_count >= 4 && hull.vertex_count <= m);
            assert_eq!(
                hull.vertex_count - hull.edge_count / 2 + hull.face_count,
                2
            );
            assert!(hull.face_count >= 4);
            destroy_hull(hull);
        }
    }
}

#[test]
fn create_hull_merge_churn_stress() {
    const N: usize = 4096;
    let mut points = vec![
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        N
    ];
    let seeds = [12345u32, 0xdeadbeef];

    for &seed in &seeds {
        fill_cube_sample(&mut points, seed);
        for c in 0..8 {
            points[N - 8 + c] = Vec3 {
                x: if (c & 1) != 0 { 1.0 } else { -1.0 },
                y: if (c & 2) != 0 { 1.0 } else { -1.0 },
                z: if (c & 4) != 0 { 1.0 } else { -1.0 },
            };
        }
        let hull = create_hull(&points, 64).expect("merge churn");
        assert_eq!(hull.vertex_count, 8);
        assert_eq!(hull.edge_count, 24);
        assert_eq!(hull.face_count, 6);
        destroy_hull(hull);
    }
}

#[test]
fn create_hull_degenerate() {
    let collinear = [
        Vec3 { x: 0.0, y: 0.0, z: 0.0 },
        Vec3 { x: 1.0, y: 0.0, z: 0.0 },
        Vec3 { x: 2.0, y: 0.0, z: 0.0 },
        Vec3 { x: 3.0, y: 0.0, z: 0.0 },
        Vec3 { x: 4.0, y: 0.0, z: 0.0 },
        Vec3 { x: 5.0, y: 0.0, z: 0.0 },
        Vec3 { x: 6.0, y: 0.0, z: 0.0 },
        Vec3 { x: 7.0, y: 0.0, z: 0.0 },
    ];
    assert!(create_hull(&collinear[..0], 8).is_none());
    assert!(create_hull(&collinear[..3], 8).is_none());

    let coincident = [Vec3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    }; 8];
    assert!(create_hull(&coincident, 8).is_none());
    assert!(create_hull(&collinear, 8).is_none());

    let coplanar = [
        Vec3 { x: 0.0, y: 0.0, z: 0.0 },
        Vec3 { x: 1.0, y: 0.0, z: 0.0 },
        Vec3 { x: 0.0, y: 1.0, z: 0.0 },
        Vec3 { x: 1.0, y: 1.0, z: 0.0 },
        Vec3 { x: 2.0, y: 0.5, z: 0.0 },
        Vec3 { x: 0.5, y: 2.0, z: 0.0 },
    ];
    assert!(create_hull(&coplanar, 8).is_none());
}
