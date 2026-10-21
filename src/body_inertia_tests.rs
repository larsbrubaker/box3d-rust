//! Body inertia tests ported from box3d-cpp-reference/test/test_body.c:
//! CompoundInertiaConsistency and IsotropicInertiaTolerance (upstream efca0c1).
//! Split from body_tests.rs to satisfy the file-length limit.

use crate::body::{
    body_apply_mass_from_shapes, body_flags, body_get_angular_velocity,
    body_get_local_rotational_inertia, body_get_position,
    body_get_world_inverse_rotational_inertia, body_set_motion_locks, body_set_transform,
    create_body, get_body_full_id,
};
use crate::hull::{
    create_complex_hull, create_rock, destroy_hull, make_box_hull, make_cube_hull,
    make_offset_box_hull, make_transformed_box_hull, HullData,
};
use crate::id::BodyId;
use crate::math_functions::{
    length, make_matrix_from_quat, make_quat_from_axis_angle, mul_mm, normalize, sub, transpose,
    Transform, Vec3, VEC3_ZERO,
};
use crate::shape::create_hull_shape;
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType, MotionLocks};
use crate::world::World;

fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

#[test]
fn compound_inertia_consistency() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.rotation = make_quat_from_axis_angle(
        normalize(Vec3 {
            x: 0.3,
            y: -0.8,
            z: 0.5,
        }),
        1.3,
    );
    let body_id = create_body(&mut world, &body_def);

    let shape_def = default_shape_def();
    let box_a = make_offset_box_hull(
        0.5,
        0.25,
        0.5,
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 0.0,
        },
    );
    let box_b = make_offset_box_hull(
        0.3,
        0.6,
        0.2,
        Vec3 {
            x: -0.8,
            y: -0.4,
            z: 0.3,
        },
    );
    create_hull_shape(&mut world, body_id, &shape_def, &box_a.base);
    create_hull_shape(&mut world, body_id, &shape_def, &box_b.base);
    body_apply_mass_from_shapes(&mut world, body_id);

    let local = body_get_local_rotational_inertia(&world, body_id);
    assert!(local.cy.x.abs() > 0.1);

    let mut r = make_matrix_from_quat(body_def.rotation);
    let mut world_inertia = mul_mm(mul_mm(r, local), transpose(r));
    let mut inv_world = body_get_world_inverse_rotational_inertia(&world, body_id);
    let mut product = mul_mm(inv_world, world_inertia);

    ensure_small(product.cx.x - 1.0, 1.0e-4);
    ensure_small(product.cy.y - 1.0, 1.0e-4);
    ensure_small(product.cz.z - 1.0, 1.0e-4);
    ensure_small(product.cx.y, 1.0e-4);
    ensure_small(product.cx.z, 1.0e-4);
    ensure_small(product.cy.x, 1.0e-4);
    ensure_small(product.cy.z, 1.0e-4);
    ensure_small(product.cz.x, 1.0e-4);
    ensure_small(product.cz.y, 1.0e-4);

    let q2 = make_quat_from_axis_angle(
        normalize(Vec3 {
            x: -1.0,
            y: 0.2,
            z: 0.4,
        }),
        -2.1,
    );
    let position = body_get_position(&world, body_id);
    body_set_transform(&mut world, body_id, position, q2);
    r = make_matrix_from_quat(q2);
    world_inertia = mul_mm(mul_mm(r, local), transpose(r));
    inv_world = body_get_world_inverse_rotational_inertia(&world, body_id);
    product = mul_mm(inv_world, world_inertia);
    ensure_small(product.cx.x - 1.0, 1.0e-4);
    ensure_small(product.cy.y - 1.0, 1.0e-4);
    ensure_small(product.cz.z - 1.0, 1.0e-4);
    ensure_small(product.cy.x, 1.0e-4);
    ensure_small(product.cz.y, 1.0e-4);
}

fn create_spinning_hull_body(world: &mut World, x: f32, hull: &HullData) -> BodyId {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position.x = x as _;
    body_def.rotation = make_quat_from_axis_angle(
        normalize(Vec3 {
            x: 0.3,
            y: -0.8,
            z: 0.5,
        }),
        1.3,
    );
    body_def.angular_velocity = Vec3 {
        x: 1.0,
        y: -2.0,
        z: 3.0,
    };
    let body_id = create_body(world, &body_def);

    let shape_def = default_shape_def();
    create_hull_shape(world, body_id, &shape_def, hull);
    body_id
}

fn has_gyroscopic_torque(world: &World, body_id: BodyId) -> bool {
    let body_index = get_body_full_id(world, body_id);
    (world.bodies[body_index as usize].flags & body_flags::ENABLE_GYROSCOPIC_TORQUE) != 0
}

// Bodies within the isotropic tolerance skip the gyroscopic solve, so in free flight the world
// angular velocity is untouched. Anisotropic bodies precess.
#[test]
fn isotropic_inertia_tolerance() {
    let w0 = Vec3 {
        x: 1.0,
        y: -2.0,
        z: 3.0,
    };

    let rock = create_rock(0.5).expect("rock");
    let complex = create_complex_hull(0.5).expect("complex hull");
    let cube = make_cube_hull(0.5);
    let cube_transform = Transform {
        p: VEC3_ZERO,
        q: make_quat_from_axis_angle(
            normalize(Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            }),
            0.7,
        ),
    };
    let rotated_cube = make_transformed_box_hull(0.5, 0.5, 0.5, cube_transform);
    let plank = make_box_hull(0.5, 1.0, 0.25);

    {
        let mut world_def = default_world_def();
        world_def.gravity = VEC3_ZERO;
        let mut world = World::new(&world_def);

        let rock_id = create_spinning_hull_body(&mut world, 0.0, &rock);
        let complex_id = create_spinning_hull_body(&mut world, 5.0, &complex);
        let cube_id = create_spinning_hull_body(&mut world, 10.0, &cube.base);
        let rotated_cube_id = create_spinning_hull_body(&mut world, 15.0, &rotated_cube.base);
        let plank_id = create_spinning_hull_body(&mut world, 20.0, &plank.base);

        assert!(!has_gyroscopic_torque(&world, rock_id));
        assert!(!has_gyroscopic_torque(&world, complex_id));
        assert!(!has_gyroscopic_torque(&world, cube_id));
        assert!(!has_gyroscopic_torque(&world, rotated_cube_id));
        assert!(has_gyroscopic_torque(&world, plank_id));

        for _ in 0..10 {
            world.step(1.0 / 60.0, 4);
        }

        let mut w = body_get_angular_velocity(&world, rock_id);
        assert!(w.x == w0.x && w.y == w0.y && w.z == w0.z);
        w = body_get_angular_velocity(&world, complex_id);
        assert!(w.x == w0.x && w.y == w0.y && w.z == w0.z);
        w = body_get_angular_velocity(&world, rotated_cube_id);
        assert!(w.x == w0.x && w.y == w0.y && w.z == w0.z);
        w = body_get_angular_velocity(&world, plank_id);
        assert!(length(sub(w, w0)) > 0.01);

        body_set_motion_locks(
            &mut world,
            plank_id,
            MotionLocks {
                angular_x: true,
                angular_y: true,
                angular_z: true,
                ..Default::default()
            },
        );
        assert!(!has_gyroscopic_torque(&world, plank_id));

        // Gyroscopic torque stays off until a mass update proves the body anisotropic
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position.x = 25.0 as _;
        body_def.angular_velocity = w0;
        let empty_id = create_body(&mut world, &body_def);
        assert!(!has_gyroscopic_torque(&world, empty_id));

        body_def.position.x = 30.0 as _;
        let deferred_id = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.update_body_mass = false;
        create_hull_shape(&mut world, deferred_id, &shape_def, &plank.base);
        assert!(!has_gyroscopic_torque(&world, deferred_id));

        body_apply_mass_from_shapes(&mut world, deferred_id);
        assert!(has_gyroscopic_torque(&world, deferred_id));
    }

    {
        let mut world_def = default_world_def();
        world_def.gravity = VEC3_ZERO;
        world_def.isotropic_inertia_tolerance = 0.0;
        let mut world = World::new(&world_def);

        let rock_id = create_spinning_hull_body(&mut world, 0.0, &rock);
        let cube_id = create_spinning_hull_body(&mut world, 10.0, &cube.base);

        assert!(has_gyroscopic_torque(&world, rock_id));
        assert!(!has_gyroscopic_torque(&world, cube_id));

        for _ in 0..10 {
            world.step(1.0 / 60.0, 4);
        }

        let w = body_get_angular_velocity(&world, rock_id);
        assert!(length(sub(w, w0)) > 0.001);
    }

    destroy_hull(rock);
    destroy_hull(complex);
}
