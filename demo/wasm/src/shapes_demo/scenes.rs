//! Scene builders for the Shapes samples (`sample_shapes.cpp`), one per C ctor.

use super::{
    add_ground_box, create_hull_shape_at, new_world, ShapeScene, ShapeState, WIND_MAX_COUNT,
    WIND_SHAPE_BOX, WIND_SHAPE_CAPSULE, WIND_SHAPE_SPHERE,
};
use crate::vis::{pos, sphere, vec3, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::geometry::Capsule;
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::id::NULL_BODY_ID;
use box3d_rust::joint::{create_filter_joint, create_revolute_joint, create_spherical_joint};
use box3d_rust::math_functions::{
    compute_cos_sin, make_quat_from_axis_angle, mul_sv, rotate_vector, Transform, Vec3, DEG_TO_RAD,
    PI, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z,
};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_filter_joint_def, default_revolute_joint_def, default_shape_def,
    default_spherical_joint_def, BodyType,
};

/// C `InclinedPlane` (sample_shapes.cpp:13-52).
pub(crate) fn build_inclined_plane() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 50.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 50.0, 1.0, 50.0));

    let mut body_def = default_body_def();
    body_def.position = pos(0.0, 7.5, -5.0);
    body_def.rotation = make_quat_from_axis_angle(vec3(1.0, 0.0, 0.0), 40.0 * DEG_TO_RAD);
    let plane_body = create_body(&mut world, &body_def);
    let plane_box = make_box_hull(16.0, 0.5, 10.0);
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 1.0;
    create_hull_shape_at(&mut world, plane_body, &shape_def, &plane_box);
    bodies.push(VisBody::box_body(plane_body.index1 - 1, 16.0, 0.5, 10.0));

    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    body_def.type_ = BodyType::Dynamic;
    for index in 0..5 {
        body_def.position = pos(-10.0 + 5.0 * index as f32, 15.75, -10.6);
        let box_body = create_body(&mut world, &body_def);
        shape_def.base_material.friction = (index + 1) as f32 * (index + 1) as f32 * 0.04;
        create_hull_shape_at(&mut world, box_body, &shape_def, &box_hull);
        bodies.push(VisBody::box_body(box_body.index1 - 1, 1.0, 1.0, 1.0));
    }

    finish(world, bodies, ShapeScene::InclinedPlane)
}

/// C `RollingResistance` (sample_shapes.cpp:56-108).
pub(crate) fn build_rolling_resistance() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 50.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 50.0, 1.0, 50.0));

    let mut body_def = default_body_def();
    let mut shape_def = default_shape_def();

    body_def.position = pos(0.0, 2.0, -20.0);
    body_def.rotation = make_quat_from_axis_angle(vec3(1.0, 0.0, 0.0), 10.0 * DEG_TO_RAD);
    let plane_body = create_body(&mut world, &body_def);
    let plane = make_box_hull(32.0, 0.5, 15.0);
    create_hull_shape_at(&mut world, plane_body, &shape_def, &plane);
    bodies.push(VisBody::box_body(plane_body.index1 - 1, 32.0, 0.5, 15.0));

    let sph = sphere(1.0);
    body_def.type_ = BodyType::Dynamic;
    for index in 0..5 {
        body_def.position = pos(-25.0 + 5.0 * index as f32, 8.0, -24.0);
        let body = create_body(&mut world, &body_def);
        shape_def.base_material.rolling_resistance = 0.05 * index as f32;
        create_sphere_shape(&mut world, body, &shape_def, &sph);
        bodies.push(VisBody::sphere_body(body.index1 - 1, 1.0));
    }

    let capsule = Capsule {
        center1: vec3(-1.0, 0.0, 0.0),
        center2: vec3(1.0, 0.0, 0.0),
        radius: 0.5,
    };
    for index in 0..5 {
        body_def.position = pos(2.0 + 5.0 * index as f32, 8.0, -24.0);
        let body = create_body(&mut world, &body_def);
        shape_def.base_material.rolling_resistance = 0.05 * index as f32;
        create_capsule_shape(&mut world, body, &shape_def, &capsule);
        bodies.push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }

    finish(world, bodies, ShapeScene::RollingResistance)
}

/// C `HighResistance` (sample_shapes.cpp:112-149).
pub(crate) fn build_high_resistance() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 50.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 50.0, 1.0, 50.0));

    let mut body_def = default_body_def();
    let mut shape_def = default_shape_def();
    let capsule = Capsule {
        center1: vec3(0.0, -1.0, 0.0),
        center2: vec3(0.0, 1.0, 0.0),
        radius: 0.5,
    };
    body_def.type_ = BodyType::Dynamic;
    body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, DEG_TO_RAD * 30.0);
    for index in 0..10 {
        body_def.position = pos(-22.0 + 5.0 * index as f32, 1.5, 0.0);
        let body = create_body(&mut world, &body_def);
        shape_def.base_material.rolling_resistance = 0.2 * index as f32;
        create_capsule_shape(&mut world, body, &shape_def, &capsule);
        bodies.push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }

    finish(world, bodies, ShapeScene::HighResistance)
}

/// C `IsotropicFriction` (sample_shapes.cpp:151-193).
pub(crate) fn build_isotropic_friction() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 100.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 100.0, 1.0, 100.0));

    let mut body_def = default_body_def();
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    body_def.type_ = BodyType::Dynamic;
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.6;
    for index in 0..32 {
        let alpha = PI / 16.0 * index as f32;
        let cs = compute_cos_sin(alpha);
        body_def.position = pos(15.0 * cs.cosine, 1.0, 15.0 * cs.sine);
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Y, -alpha);
        body_def.linear_velocity = vec3(25.0 * cs.cosine, 0.0, 25.0 * cs.sine);
        let box_body = create_body(&mut world, &body_def);
        create_hull_shape_at(&mut world, box_body, &shape_def, &box_hull);
        bodies.push(VisBody::box_body(box_body.index1 - 1, 1.0, 1.0, 1.0));
    }

    finish(world, bodies, ShapeScene::IsotropicFriction)
}

/// C `SlideTwist` (sample_shapes.cpp:196-239).
pub(crate) fn build_slide_twist() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 50.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 50.0, 1.0, 50.0));

    let orientation = make_quat_from_axis_angle(VEC3_AXIS_X, 20.0 * DEG_TO_RAD);

    let mut body_def = default_body_def();
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 1.0;

    body_def.position = pos(0.0, 4.0, 0.0);
    body_def.rotation = orientation;
    let plane_body = create_body(&mut world, &body_def);
    let plane = make_box_hull(10.0, 0.5, 10.0);
    shape_def.base_material.friction = 0.6;
    create_hull_shape_at(&mut world, plane_body, &shape_def, &plane);
    bodies.push(VisBody::box_body(plane_body.index1 - 1, 10.0, 0.5, 10.0));

    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 5.0, 0.0);
    body_def.rotation = orientation;
    body_def.angular_velocity = mul_sv(25.0, rotate_vector(orientation, VEC3_AXIS_Y));
    let box_body = create_body(&mut world, &body_def);
    let m_box = make_box_hull(1.0, 0.5, 1.0);
    shape_def.base_material.friction = 0.3;
    create_hull_shape_at(&mut world, box_body, &shape_def, &m_box);
    bodies.push(VisBody::box_body(box_body.index1 - 1, 1.0, 0.5, 1.0));

    finish(world, bodies, ShapeScene::SlideTwist)
}

/// C `Restitution::CreateBodies` (sample_shapes.cpp:241-337). 40 falling bodies with
/// ascending restitution; `box_shape` toggles sphere vs box.
pub(crate) fn build_restitution(box_shape: bool) -> ShapeState {
    let count = 40;
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 50.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 50.0, 1.0, 50.0));

    let sph = sphere(0.5);
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let mut shape_def = default_shape_def();
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    let dr = 1.0 / if count > 1 { (count - 1) as f32 } else { 1.0 };
    let mut x = -1.0 * (count - 1) as f32;
    let dx = 2.0;

    for _ in 0..count {
        body_def.position = pos(x, 40.0, 0.0);
        let body = create_body(&mut world, &body_def);
        if box_shape {
            create_hull_shape_at(&mut world, body, &shape_def, &box_hull);
            bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 0.5, 0.5));
        } else {
            create_sphere_shape(&mut world, body, &shape_def, &sph);
            bodies.push(VisBody::sphere_body(body.index1 - 1, 0.5));
        }
        shape_def.base_material.restitution += dr;
        x += dx;
    }

    finish(world, bodies, ShapeScene::Restitution)
}

/// C `StaticInvoke` ctor (sample_shapes.cpp:341-366) — the rolling dynamic sphere.
pub(crate) fn build_static_invoke() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 20.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 20.0, 1.0, 20.0));

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.25, 1.0, 0.0);
    let body = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.base_material.rolling_resistance = 0.2;
    let sph = sphere(0.5);
    create_sphere_shape(&mut world, body, &shape_def, &sph);
    bodies.push(VisBody::sphere_body(body.index1 - 1, 0.5));

    let mut state = finish(world, bodies, ShapeScene::StaticInvoke);
    state.invoke = false;
    state.static_body = NULL_BODY_ID;
    state
}

/// C `StaticInvoke::CreateStatic` (sample_shapes.cpp:368-384). Destroys any existing
/// static body then creates a static sphere at (0,0.5,0) with the current invoke flag.
pub(crate) fn create_static(state: &mut ShapeState) {
    use box3d_rust::body::destroy_body;
    if state.static_body.is_non_null() {
        let idx = state.static_body.index1 - 1;
        destroy_body(&mut state.world, state.static_body);
        state.bodies.retain(|b| b.body_index != idx);
        state.static_body = NULL_BODY_ID;
    }

    let mut body_def = default_body_def();
    body_def.position = pos(0.0, 0.5, 0.0);
    let body = create_body(&mut state.world, &body_def);
    let sph = sphere(0.5);
    let mut shape_def = default_shape_def();
    shape_def.invoke_contact_creation = state.invoke;
    create_sphere_shape(&mut state.world, body, &shape_def, &sph);
    state.static_body = body;
    state
        .bodies
        .push(VisBody::sphere_body(body.index1 - 1, 0.5));
}

/// C `ConveyorBelt` (sample_shapes.cpp:438-486).
pub(crate) fn build_conveyor_belt() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 20.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 20.0, 1.0, 20.0));

    // Platform.
    {
        let mut body_def = default_body_def();
        body_def.position = pos(-5.0, 5.0, 0.0);
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Y, 0.2);
        let body = create_body(&mut world, &body_def);
        let box_hull = make_box_hull(10.0, 0.25, 2.0);
        let mut shape_def = default_shape_def();
        shape_def.base_material.friction = 0.8;
        shape_def.base_material.tangent_velocity = vec3(2.0, 0.0, 0.0);
        create_hull_shape_at(&mut world, body, &shape_def, &box_hull);
        bodies.push(VisBody::box_body(body.index1 - 1, 10.0, 0.25, 2.0));
    }

    // Boxes.
    let shape_def = default_shape_def();
    let cube = make_box_hull(0.5, 0.5, 0.5);
    for i in 0..5 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(-10.0 + 2.0 * i as f32, 7.0, 0.0);
        let body = create_body(&mut world, &body_def);
        create_hull_shape_at(&mut world, body, &shape_def, &cube);
        bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 0.5, 0.5));
    }

    finish(world, bodies, ShapeScene::ConveyorBelt)
}

/// C `Wind::CreateScene` (sample_shapes.cpp:687-772). Builds a chain of `count`
/// shapes on spherical joints anchored above the ground, with the chosen shape type.
pub(crate) fn build_wind(shape_type: u32, count: i32) -> ShapeState {
    let count = count.clamp(1, WIND_MAX_COUNT);
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 20.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 20.0, 1.0, 20.0));

    // A shapeless static body is the first joint anchor (C m_groundId).
    let ground_id = create_body(&mut world, &default_body_def());

    let radius = 0.1f32;
    let vertical_offset = 2.0f32;

    let sph = sphere(radius);
    let capsule = Capsule {
        center1: vec3(-radius, 0.0, 0.0),
        center2: vec3(radius, 0.0, 0.0),
        radius: 0.5 * radius,
    };
    let box_hull = make_box_hull(1.25 * radius, 0.75 * radius, 0.125 * radius);

    let mut joint_def = default_spherical_joint_def();
    joint_def.base.body_id_a = ground_id;
    joint_def.base.local_frame_a.p = vec3(0.0, vertical_offset, 0.0);
    joint_def.base.draw_scale = 0.1;

    let mut shape_def = default_shape_def();
    shape_def.density = 20.0;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.gravity_scale = 0.5;
    body_def.enable_sleep = false;

    let mut wind_body_ids = Vec::with_capacity(count as usize);
    for i in 0..count {
        body_def.position = pos((2.0 * i as f32 + 1.0) * radius, vertical_offset, 0.0);
        let body = create_body(&mut world, &body_def);
        wind_body_ids.push(body);

        match shape_type {
            WIND_SHAPE_SPHERE => {
                create_sphere_shape(&mut world, body, &shape_def, &sph);
                bodies.push(VisBody::sphere_body(body.index1 - 1, radius));
            }
            WIND_SHAPE_CAPSULE => {
                create_capsule_shape(&mut world, body, &shape_def, &capsule);
                bodies.push(VisBody::capsule_body(body.index1 - 1, &capsule));
            }
            _ => {
                create_hull_shape_at(&mut world, body, &shape_def, &box_hull);
                bodies.push(VisBody::box_body(
                    body.index1 - 1,
                    1.25 * radius,
                    0.75 * radius,
                    0.125 * radius,
                ));
            }
        }

        joint_def.base.body_id_b = body;
        joint_def.base.local_frame_b.p = vec3(-radius, 0.0, 0.0);
        create_spherical_joint(&mut world, &joint_def);

        joint_def.base.body_id_a = body;
        joint_def.base.local_frame_a.p = vec3(radius, 0.0, 0.0);
    }

    let mut state = finish(world, bodies, ShapeScene::Wind);
    state.wind_shape_type = if shape_type <= WIND_SHAPE_BOX {
        shape_type
    } else {
        WIND_SHAPE_BOX
    };
    state.count = count;
    state.ground_id = ground_id;
    state.wind_body_ids = wind_body_ids;
    state
}

/// C `WindDrop` (sample_shapes.cpp:844-903). A thin plate that flutters under lift.
pub(crate) fn build_wind_drop() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 15.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 15.0, 1.0, 15.0));

    let radius = 0.1f32;
    let (hx, hy, hz) = (4.0 * radius, 0.1 * radius, 4.0 * radius);
    let box_hull = make_box_hull(hx, hy, hz);

    let mut shape_def = default_shape_def();
    shape_def.density = 2.0;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, 0.25);
    body_def.gravity_scale = 0.5;
    body_def.position = pos(0.0, 10.0, 0.0);
    let body = create_body(&mut world, &body_def);
    let shape_id = create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_body(body.index1 - 1, hx, hy, hz));

    let mut state = finish(world, bodies, ShapeScene::WindDrop);
    state.drag = 1.0;
    state.lift = 4.0;
    state.flap_drop_shape = shape_id;
    state
}

/// C `WindFlap` (sample_shapes.cpp:905-1019). A torso capsule with two spring-limited
/// wings flapping under a sinusoidal target angle plus wind lift.
pub(crate) fn build_wind_flap() -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = add_ground_box(&mut world, 50.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 50.0, 1.0, 50.0));

    let a = 0.4f32;
    let capsule = Capsule {
        center1: vec3(0.0, 0.0, -a),
        center2: vec3(0.0, 0.0, a),
        radius: 0.25 * a,
    };
    let wing_transform = Transform {
        p: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        q: make_quat_from_axis_angle(VEC3_AXIS_X, 0.1),
    };
    let box1 = make_transformed_box_hull(2.0 * a, 0.01, a, wing_transform);
    let box2 = make_transformed_box_hull(2.0 * a, 0.01, a, wing_transform);

    let y = 20.0f32;
    let mut shape_def = default_shape_def();
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    shape_def.density = 5.0;
    body_def.position = pos(-2.0 * a, y, 0.0);
    let wing_body1 = create_body(&mut world, &body_def);
    let shape1 = create_hull_shape(&mut world, wing_body1, &shape_def, &box1.base);
    bodies.push(VisBody::box_local(
        wing_body1.index1 - 1,
        2.0 * a,
        0.01,
        a,
        wing_transform,
    ));

    body_def.position = pos(2.0 * a, y, 0.0);
    let wing_body2 = create_body(&mut world, &body_def);
    let shape2 = create_hull_shape(&mut world, wing_body2, &shape_def, &box2.base);
    bodies.push(VisBody::box_local(
        wing_body2.index1 - 1,
        2.0 * a,
        0.01,
        a,
        wing_transform,
    ));

    body_def.position = pos(0.0, y, 0.0);
    let torso_body = create_body(&mut world, &body_def);
    shape_def.density = 10.0;
    create_capsule_shape(&mut world, torso_body, &shape_def, &capsule);
    bodies.push(VisBody::capsule_body(torso_body.index1 - 1, &capsule));

    let mut joint_def = default_revolute_joint_def();
    joint_def.base.draw_scale = 0.1;
    joint_def.base.body_id_a = torso_body;
    joint_def.base.local_frame_a.p = vec3(0.0, 0.0, 0.0);
    joint_def.base.body_id_b = wing_body1;
    joint_def.base.local_frame_b.p = vec3(2.0 * a, 0.0, 0.0);
    joint_def.enable_spring = true;
    joint_def.hertz = 6.0;
    joint_def.damping_ratio = 0.5;
    joint_def.enable_limit = true;
    joint_def.lower_angle = -30.0 * PI / 180.0;
    joint_def.upper_angle = 30.0 * PI / 180.0;
    let joint1 = create_revolute_joint(&mut world, &joint_def);

    joint_def.base.body_id_b = wing_body2;
    joint_def.base.local_frame_b.p = vec3(-2.0 * a, 0.0, 0.0);
    let joint2 = create_revolute_joint(&mut world, &joint_def);

    let mut filter_def = default_filter_joint_def();
    filter_def.base.body_id_a = wing_body1;
    filter_def.base.body_id_b = wing_body2;
    create_filter_joint(&mut world, &filter_def);

    let mut state = finish(world, bodies, ShapeScene::WindFlap);
    state.drag = 1.0;
    state.lift = 2.0;
    state.flap_shape1 = shape1;
    state.flap_shape2 = shape2;
    state.flap_joint1 = joint1;
    state.flap_joint2 = joint2;
    state.time = 0.0;
    state
}

/// Shared tail: wrap a built world + render list into a base [`ShapeState`].
fn finish(world: box3d_rust::world::World, bodies: Vec<VisBody>, scene: ShapeScene) -> ShapeState {
    let mut state = ShapeState::base(world, scene);
    state.bodies = bodies;
    state
}
