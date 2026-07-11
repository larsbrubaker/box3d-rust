use super::{box_hull, capsule, ensure_small, sphere, v};
use crate::geometry::{
    compute_capsule_aabb, compute_capsule_mass, compute_hull_aabb, compute_hull_mass,
    compute_sphere_aabb, compute_sphere_mass,
};
use crate::hull::{create_hull, destroy_hull, make_box_hull, make_transformed_box_hull};
use crate::math_functions::{
    compute_quat_between_unit_vectors, distance, sub_mm, Transform, PI, QUAT_IDENTITY,
    TRANSFORM_IDENTITY, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ZERO,
};
#[test]
fn shape_mass_test() {
    // Sphere
    {
        let md = compute_sphere_mass(&sphere(), 1.0);
        let mass = 4.0 / 3.0 * PI;
        ensure_small(md.mass - mass, f32::EPSILON);
        assert_eq!(md.center.x, 1.0);
        assert_eq!(md.center.y, 0.0);

        // Inertia is now about the shape center of mass, so the offset does not appear.
        let inertia = 2.0 / 5.0 * mass;
        ensure_small(md.inertia.cx.x - inertia, f32::EPSILON);
        ensure_small(md.inertia.cy.y - inertia, f32::EPSILON);
        ensure_small(md.inertia.cz.z - inertia, f32::EPSILON);
    }

    // Analytic box hull
    {
        let box_h = box_hull();
        let md = compute_hull_mass(&box_h.base, 1.0);
        let mass = 2.0 * 2.0 * 2.0;
        ensure_small(md.mass - mass, f32::EPSILON);
        ensure_small(md.center.x, f32::EPSILON);
        ensure_small(md.center.y, f32::EPSILON);
        ensure_small(md.center.z, f32::EPSILON);
        let inertia = (1.0 / 12.0) * mass * (2.0 * 2.0 + 2.0 * 2.0);
        ensure_small(md.inertia.cx.x - inertia, 2.0 * f32::EPSILON);
        ensure_small(md.inertia.cy.y - inertia, 2.0 * f32::EPSILON);
        ensure_small(md.inertia.cz.z - inertia, 2.0 * f32::EPSILON);
    }

    // Translated box
    {
        let offset = v(0.4, -0.7, 0.1);
        let transform = Transform {
            p: offset,
            q: QUAT_IDENTITY,
        };
        let h = v(0.25, 0.5, 0.3);
        let b1 = make_box_hull(h.x, h.y, h.z);
        let b2 = make_transformed_box_hull(h.x, h.y, h.z, transform);

        let m1 = compute_hull_mass(&b1.base, 1.0);
        let m2 = compute_hull_mass(&b2.base, 1.0);

        ensure_small(m1.mass - m2.mass, f32::EPSILON);

        let d = sub_mm(b1.base.central_inertia, b2.base.central_inertia);
        ensure_small(d.cx.x, f32::EPSILON);
        ensure_small(d.cx.y, f32::EPSILON);
        ensure_small(d.cx.z, f32::EPSILON);
        ensure_small(d.cy.x, f32::EPSILON);
        ensure_small(d.cy.y, f32::EPSILON);
        ensure_small(d.cy.z, f32::EPSILON);
        ensure_small(d.cz.x, f32::EPSILON);
        ensure_small(d.cz.y, f32::EPSILON);
        ensure_small(d.cz.z, f32::EPSILON);

        ensure_small(m2.center.x - offset.x, f32::EPSILON);
        ensure_small(m2.center.y - offset.y, f32::EPSILON);
        ensure_small(m2.center.z - offset.z, f32::EPSILON);
    }

    // Rotated box
    {
        let h1 = v(0.25, 0.5, 0.3);
        let h2 = v(0.25, 0.3, 0.5);
        let q = compute_quat_between_unit_vectors(VEC3_AXIS_Y, VEC3_AXIS_Z);
        let transform = Transform {
            p: VEC3_ZERO,
            q,
        };
        let b1 = make_transformed_box_hull(h1.x, h1.y, h1.z, transform);
        let b2 = make_box_hull(h2.x, h2.y, h2.z);

        let m1 = compute_hull_mass(&b1.base, 1.0);
        let m2 = compute_hull_mass(&b2.base, 1.0);

        ensure_small(m1.mass - m2.mass, f32::EPSILON);

        let d = sub_mm(b1.base.central_inertia, b2.base.central_inertia);
        ensure_small(d.cx.x, f32::EPSILON);
        ensure_small(d.cx.y, f32::EPSILON);
        ensure_small(d.cx.z, f32::EPSILON);
        ensure_small(d.cy.x, f32::EPSILON);
        ensure_small(d.cy.y, f32::EPSILON);
        ensure_small(d.cy.z, f32::EPSILON);
        ensure_small(d.cz.x, f32::EPSILON);
        ensure_small(d.cz.y, f32::EPSILON);
        ensure_small(d.cz.z, f32::EPSILON);

        ensure_small(m1.center.x - m2.center.x, f32::EPSILON);
        ensure_small(m1.center.y - m2.center.y, f32::EPSILON);
        ensure_small(m1.center.z - m2.center.z, f32::EPSILON);
    }

    // Transformed box
    {
        let offset = v(0.4, -0.7, 0.1);
        let h1 = v(0.25, 0.5, 0.3);
        let h2 = v(0.25, 0.3, 0.5);
        let q = compute_quat_between_unit_vectors(VEC3_AXIS_Y, VEC3_AXIS_Z);
        let transform = Transform { p: offset, q };
        let b1 = make_transformed_box_hull(h1.x, h1.y, h1.z, transform);
        let b2 = make_box_hull(h2.x, h2.y, h2.z);

        let m1 = compute_hull_mass(&b1.base, 1.0);
        let m2 = compute_hull_mass(&b2.base, 1.0);

        ensure_small(m1.mass - m2.mass, f32::EPSILON);

        let d = sub_mm(b1.base.central_inertia, b2.base.central_inertia);
        ensure_small(d.cx.x, f32::EPSILON);
        ensure_small(d.cx.y, f32::EPSILON);
        ensure_small(d.cx.z, f32::EPSILON);
        ensure_small(d.cy.x, f32::EPSILON);
        ensure_small(d.cy.y, f32::EPSILON);
        ensure_small(d.cy.z, f32::EPSILON);
        ensure_small(d.cz.x, f32::EPSILON);
        ensure_small(d.cz.y, f32::EPSILON);
        ensure_small(d.cz.z, f32::EPSILON);

        ensure_small(m1.center.x - offset.x, f32::EPSILON);
        ensure_small(m1.center.y - offset.y, f32::EPSILON);
        ensure_small(m1.center.z - offset.z, f32::EPSILON);
    }

    // Capsule
    {
        const N: usize = 4;
        let cap = capsule();
        let radius = cap.radius;
        let length = distance(cap.center1, cap.center2);

        // Capsule along x-axis
        let md = compute_capsule_mass(&cap, 1.0);

        // Box that fully contains capsule. Upper bound on capsule mass.
        let r = make_box_hull(radius + 0.5 * length, radius, radius);
        let md_upper = compute_hull_mass(&r.base, 1.0);

        // Approximate capsule using convex hull. This should be a lower bound on the
        // capsule mass.
        let mut points = [VEC3_ZERO; 2 * N * N];
        let d = PI / (N as f32 - 1.0);
        let mut angle1 = -0.5 * PI;
        let mut index = 0;
        for _i in 0..N {
            let s1 = angle1.sin();
            let c1 = angle1.cos();
            let mut angle2 = -0.5 * PI;
            for _j in 0..N {
                points[index].x = 1.0 + radius * c1;
                points[index].y = radius * s1 * angle2.cos();
                points[index].z = radius * s1 * angle2.sin();
                angle2 += d;
                index += 1;
            }
            angle1 += d;
        }

        angle1 = 0.5 * PI;
        for _i in 0..N {
            let s1 = angle1.sin();
            let c1 = angle1.cos();
            let mut angle2 = -0.5 * PI;
            for _j in 0..N {
                points[index].x = -1.0 + radius * c1;
                points[index].y = radius * s1 * angle2.cos();
                points[index].z = radius * s1 * angle2.sin();
                angle2 += d;
                index += 1;
            }
            angle1 += d;
        }

        assert_eq!(index, 2 * N * N);

        let hull = create_hull(&points, (2 * N * N) as i32).expect("capsule approx hull");
        let md_lower = compute_hull_mass(&hull, 1.0);

        assert!(md_lower.mass < md.mass && md.mass < md_upper.mass);
        assert!(
            md_lower.inertia.cx.x < md.inertia.cx.x && md.inertia.cx.x < md_upper.inertia.cx.x
        );
        assert!(
            md_lower.inertia.cy.y < md.inertia.cy.y && md.inertia.cy.y < md_upper.inertia.cy.y
        );
        assert!(
            md_lower.inertia.cz.z < md.inertia.cz.z && md.inertia.cz.z < md_upper.inertia.cz.z
        );

        destroy_hull(hull);
    }
}

#[test]
fn shape_aabb_test() {
    {
        let b = compute_sphere_aabb(&sphere(), TRANSFORM_IDENTITY);
        ensure_small(b.lower_bound.x, f32::EPSILON);
        ensure_small(b.lower_bound.y + 1.0, f32::EPSILON);
        ensure_small(b.lower_bound.z + 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.x - 2.0, f32::EPSILON);
        ensure_small(b.upper_bound.y - 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.z - 1.0, f32::EPSILON);
    }

    {
        let b = compute_capsule_aabb(&capsule(), TRANSFORM_IDENTITY);
        ensure_small(b.lower_bound.x + 2.0, f32::EPSILON);
        ensure_small(b.lower_bound.y + 1.0, f32::EPSILON);
        ensure_small(b.lower_bound.z + 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.x - 2.0, f32::EPSILON);
        ensure_small(b.upper_bound.y - 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.z - 1.0, f32::EPSILON);
    }
    {
        let box_h = box_hull();
        let b = compute_hull_aabb(&box_h.base, TRANSFORM_IDENTITY);
        ensure_small(b.lower_bound.x + 1.0, f32::EPSILON);
        ensure_small(b.lower_bound.y + 1.0, f32::EPSILON);
        ensure_small(b.lower_bound.z + 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.x - 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.y - 1.0, f32::EPSILON);
        ensure_small(b.upper_bound.z - 1.0, f32::EPSILON);
    }
}
