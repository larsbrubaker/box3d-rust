//! Body lifecycle and SetMassData tests ported from
//! box3d-cpp-reference/test/test_body.c (shape-free subset) plus create/destroy
//! coverage for the bring-up path.
//!
//! Sphere-mass / deferred-extent tests need shape create and land with that slice.

use crate::body::{
    body_flags, body_get_angular_velocity, body_get_inverse_mass, body_get_linear_velocity,
    body_get_local_center, body_get_local_rotational_inertia, body_get_mass, body_get_mass_data,
    body_get_world_center, body_get_world_inverse_rotational_inertia, body_is_valid,
    body_set_angular_velocity, body_set_linear_velocity, body_set_mass_data, create_body,
    destroy_body,
};
use crate::geometry::MassData;
use crate::math_functions::{
    make_quat_from_axis_angle, Matrix3, Pos, Vec3, MAT3_ZERO, PI, VEC3_AXIS_Z, VEC3_ZERO,
};
use crate::solver_set::{AWAKE_SET, STATIC_SET};
use crate::types::{default_body_def, default_world_def, BodyType};
use crate::world::World;

fn diag_inertia() -> Matrix3 {
    Matrix3 {
        cx: Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        },
        cy: Vec3 {
            x: 0.0,
            y: 4.0,
            z: 0.0,
        },
        cz: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 8.0,
        },
    }
}

#[test]
fn create_destroy_static_and_dynamic() {
    let mut world = World::new(&default_world_def());

    let mut static_def = default_body_def();
    static_def.type_ = BodyType::Static;
    static_def.position = Pos {
        x: 1.0 as _,
        y: 2.0 as _,
        z: 3.0 as _,
    };
    let static_id = create_body(&mut world, &static_def);
    assert!(body_is_valid(&world, static_id));
    assert_eq!(
        world.bodies[static_id.index1 as usize - 1].set_index,
        STATIC_SET
    );
    assert_eq!(world.bodies[static_id.index1 as usize - 1].island_id, -1);

    let mut dyn_def = default_body_def();
    dyn_def.type_ = BodyType::Dynamic;
    dyn_def.linear_velocity = Vec3 {
        x: 0.5,
        y: 0.0,
        z: -0.25,
    };
    let dyn_id = create_body(&mut world, &dyn_def);
    assert!(body_is_valid(&world, dyn_id));
    let dyn_body = &world.bodies[dyn_id.index1 as usize - 1];
    assert_eq!(dyn_body.set_index, AWAKE_SET);
    assert!(dyn_body.island_id >= 0);
    assert_eq!(
        world.solver_sets[AWAKE_SET as usize].body_states[dyn_body.local_index as usize]
            .linear_velocity,
        dyn_def.linear_velocity
    );

    destroy_body(&mut world, dyn_id);
    assert!(!body_is_valid(&world, dyn_id));

    destroy_body(&mut world, static_id);
    assert!(!body_is_valid(&world, static_id));

    // Id recycle: next create reuses the freed slot with a new generation.
    let recycled = create_body(&mut world, &static_def);
    assert!(body_is_valid(&world, recycled));
    assert_eq!(recycled.index1, static_id.index1);
    assert_ne!(recycled.generation, static_id.generation);
    assert!(!body_is_valid(&world, static_id));
}

#[test]
fn set_mass_data_round_trip() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 5.0 as _,
        y: -3.0 as _,
        z: 2.0 as _,
    };
    let body_id = create_body(&mut world, &body_def);

    let center = Vec3 {
        x: 0.1,
        y: 0.2,
        z: 0.3,
    };
    let mass_data = MassData {
        mass: 3.0,
        center,
        inertia: diag_inertia(),
    };
    body_set_mass_data(&mut world, body_id, mass_data);

    assert!((body_get_mass(&world, body_id) - 3.0).abs() < 1e-6);
    assert!((body_get_inverse_mass(&world, body_id) - 1.0 / 3.0).abs() < 1e-6);

    let md = body_get_mass_data(&world, body_id);
    assert!((md.mass - 3.0).abs() < 1e-6);
    assert!((md.center.x - center.x).abs() < 1e-6);
    assert!((md.center.y - center.y).abs() < 1e-6);
    assert!((md.center.z - center.z).abs() < 1e-6);
    assert!((md.inertia.cx.x - 2.0).abs() < 1e-6);
    assert!((md.inertia.cy.y - 4.0).abs() < 1e-6);
    assert!((md.inertia.cz.z - 8.0).abs() < 1e-6);

    let local_center = body_get_local_center(&world, body_id);
    assert!((local_center.x - center.x).abs() < 1e-6);
    assert!((local_center.y - center.y).abs() < 1e-6);
    assert!((local_center.z - center.z).abs() < 1e-6);

    let local_inertia = body_get_local_rotational_inertia(&world, body_id);
    assert!((local_inertia.cx.x - 2.0).abs() < 1e-6);
    assert!((local_inertia.cy.y - 4.0).abs() < 1e-6);
    assert!((local_inertia.cz.z - 8.0).abs() < 1e-6);

    let inv_world = body_get_world_inverse_rotational_inertia(&world, body_id);
    assert!((inv_world.cx.x - 0.5).abs() < 1e-5);
    assert!((inv_world.cy.y - 0.25).abs() < 1e-5);
    assert!((inv_world.cz.z - 0.125).abs() < 1e-5);
    assert!(inv_world.cy.x.abs() < 1e-5);
    assert!(inv_world.cz.x.abs() < 1e-5);
    assert!(inv_world.cx.y.abs() < 1e-5);
    assert!(inv_world.cz.y.abs() < 1e-5);
    assert!(inv_world.cx.z.abs() < 1e-5);
    assert!(inv_world.cy.z.abs() < 1e-5);

    let world_center = body_get_world_center(&world, body_id);
    assert!(((world_center.x as f32) - (5.0 + center.x)).abs() < 1e-5);
    assert!(((world_center.y as f32) - (-3.0 + center.y)).abs() < 1e-5);
    assert!(((world_center.z as f32) - (2.0 + center.z)).abs() < 1e-5);
}

#[test]
fn set_mass_data_world_inertia_rotated() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.5 * PI);
    let body_id = create_body(&mut world, &body_def);

    let mass_data = MassData {
        mass: 1.0,
        center: VEC3_ZERO,
        inertia: diag_inertia(),
    };
    body_set_mass_data(&mut world, body_id, mass_data);

    let local_inertia = body_get_local_rotational_inertia(&world, body_id);
    assert!((local_inertia.cx.x - 2.0).abs() < 1e-6);
    assert!((local_inertia.cy.y - 4.0).abs() < 1e-6);
    assert!((local_inertia.cz.z - 8.0).abs() < 1e-6);

    let inv_world = body_get_world_inverse_rotational_inertia(&world, body_id);
    assert!((inv_world.cx.x - 0.25).abs() < 1e-4);
    assert!((inv_world.cy.y - 0.5).abs() < 1e-4);
    assert!((inv_world.cz.z - 0.125).abs() < 1e-4);
    assert!(inv_world.cy.x.abs() < 1e-4);
    assert!(inv_world.cz.x.abs() < 1e-4);
    assert!(inv_world.cx.y.abs() < 1e-4);
    assert!(inv_world.cz.y.abs() < 1e-4);
    assert!(inv_world.cx.z.abs() < 1e-4);
    assert!(inv_world.cy.z.abs() < 1e-4);
}

#[test]
fn set_mass_data_fixed_rotation() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.motion_locks.angular_x = true;
    body_def.motion_locks.angular_y = true;
    body_def.motion_locks.angular_z = true;
    let body_id = create_body(&mut world, &body_def);

    let mass_data = MassData {
        mass: 5.0,
        center: VEC3_ZERO,
        inertia: diag_inertia(),
    };
    body_set_mass_data(&mut world, body_id, mass_data);

    assert!((body_get_mass(&world, body_id) - 5.0).abs() < 1e-6);
    assert!((body_get_inverse_mass(&world, body_id) - 0.2).abs() < 1e-6);

    let local_inertia = body_get_local_rotational_inertia(&world, body_id);
    assert!(local_inertia.cx.x.abs() < 1e-6);
    assert!(local_inertia.cy.y.abs() < 1e-6);
    assert!(local_inertia.cz.z.abs() < 1e-6);

    let inv_world = body_get_world_inverse_rotational_inertia(&world, body_id);
    assert!(inv_world.cx.x.abs() < 1e-6);
    assert!(inv_world.cy.y.abs() < 1e-6);
    assert!(inv_world.cz.z.abs() < 1e-6);

    let md = body_get_mass_data(&world, body_id);
    assert!(md.inertia.cx.x.abs() < 1e-6);
    assert!(md.inertia.cy.y.abs() < 1e-6);
    assert!(md.inertia.cz.z.abs() < 1e-6);

    let flags = world.bodies[body_id.index1 as usize - 1].flags;
    assert_eq!(
        flags & body_flags::FIXED_ROTATION,
        body_flags::FIXED_ROTATION
    );
}

#[test]
fn set_mass_data_zero_mass() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let body_id = create_body(&mut world, &body_def);

    let mass_data = MassData {
        mass: 0.0,
        center: VEC3_ZERO,
        inertia: MAT3_ZERO,
    };
    body_set_mass_data(&mut world, body_id, mass_data);

    assert!(body_get_inverse_mass(&world, body_id).abs() < 1e-6);

    let local_inertia = body_get_local_rotational_inertia(&world, body_id);
    assert!(local_inertia.cx.x.abs() < 1e-6);
    assert!(local_inertia.cy.y.abs() < 1e-6);
    assert!(local_inertia.cz.z.abs() < 1e-6);

    let inv_world = body_get_world_inverse_rotational_inertia(&world, body_id);
    assert!(inv_world.cx.x.abs() < 1e-6);
    assert!(inv_world.cy.y.abs() < 1e-6);
    assert!(inv_world.cz.z.abs() < 1e-6);
}

#[test]
fn set_mass_data_consistent_velocity() {
    let mut world = World::new(&default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 7.0 as _,
        y: 1.0 as _,
        z: -4.0 as _,
    };
    let body_id = create_body(&mut world, &body_def);

    let omega = Vec3 {
        x: 1.0,
        y: 2.0,
        z: 4.0,
    };
    body_set_linear_velocity(
        &mut world,
        body_id,
        Vec3 {
            x: 1.0,
            y: -2.0,
            z: 3.0,
        },
    );
    body_set_angular_velocity(&mut world, body_id, omega);

    let center = Vec3 {
        x: 0.5,
        y: 0.25,
        z: 0.125,
    };
    let mass_data = MassData {
        mass: 3.0,
        center,
        inertia: diag_inertia(),
    };
    body_set_mass_data(&mut world, body_id, mass_data);

    // omega x center = (-0.75, 1.875, -0.75)
    let v = body_get_linear_velocity(&world, body_id);
    assert!((v.x - (1.0 - 0.75)).abs() < 1e-6);
    assert!((v.y - (-2.0 + 1.875)).abs() < 1e-6);
    assert!((v.z - (3.0 - 0.75)).abs() < 1e-6);

    let w = body_get_angular_velocity(&world, body_id);
    assert!((w.x - omega.x).abs() < 1e-6);
    assert!((w.y - omega.y).abs() < 1e-6);
    assert!((w.z - omega.z).abs() < 1e-6);
}
