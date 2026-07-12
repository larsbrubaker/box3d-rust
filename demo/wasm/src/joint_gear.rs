//! Gear Lift sample — faithful reduced basin; gears/chains/door match C.

use crate::joint_demo::{empty_state, new_world, JointScene, JointState};
use crate::vis::{pos, vec3, VisBody};
use box3d_rust::body::{body_get_local_point, create_body};
use box3d_rust::geometry::Capsule;
use box3d_rust::hull::{create_hull, create_rock, make_box_hull};
use box3d_rust::id::BodyId;
use box3d_rust::joint::{create_prismatic_joint, create_revolute_joint};
use box3d_rust::math_functions::{
    add, compute_quat_between_unit_vectors, make_quat_from_axis_angle, rotate_vector, Pos, Transform,
    Vec3, PI, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z,
};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape};
use box3d_rust::types::{
    default_body_def, default_prismatic_joint_def, default_revolute_joint_def, default_shape_def,
    BodyType,
};
use box3d_rust::world::World;

const GEAR_RADIUS: f32 = 1.0;
const GEAR_HALF_DEPTH: f32 = 0.125;
const GEAR_Z: f32 = 1.5;
const AXLE_RADIUS: f32 = 0.2;
const TOOTH_HALF_WIDTH: f32 = 0.11;
const TOOTH_HALF_HEIGHT: f32 = 0.09;
const TOOTH_RADIUS: f32 = 0.03;
const LINK_HALF_LENGTH: f32 = 0.07;
const LINK_RADIUS: f32 = 0.05;
const LINK_COUNT: i32 = 28;
const DOOR_HALF_HEIGHT: f32 = 1.5;
const DOOR_HALF_DEPTH: f32 = 1.95;
const GEAR_SIDES: i32 = 20;
const AXLE_SIDES: i32 = 10;
const ROCK_RADIUS: f32 = 0.3;

fn make_z_cylinder(radius: f32, z_min: f32, z_max: f32, sides: i32) -> box3d_rust::hull::HullData {
    let mut points = Vec::with_capacity((2 * sides) as usize);
    for i in 0..sides {
        let angle = 2.0 * PI * i as f32 / sides as f32;
        let c = angle.cos();
        let s = angle.sin();
        points.push(Vec3 {
            x: radius * c,
            y: radius * s,
            z: z_min,
        });
        points.push(Vec3 {
            x: radius * c,
            y: radius * s,
            z: z_max,
        });
    }
    create_hull(&points, 2 * sides).expect("z-cylinder hull")
}

fn add_teeth(world: &mut World, body_id: BodyId, tooth_center_radius: f32, z_center: f32) {
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.1;
    let count = 16;
    let delta = 2.0 * PI / count as f32;
    let hx = TOOTH_HALF_WIDTH;
    let hz = GEAR_HALF_DEPTH;
    let base_half = TOOTH_HALF_HEIGHT;
    let tip_half = TOOTH_HALF_HEIGHT - TOOTH_RADIUS;

    for i in 0..count {
        let q = make_quat_from_axis_angle(VEC3_AXIS_Z, i as f32 * delta);
        let mut center = rotate_vector(q, Vec3 {
            x: tooth_center_radius,
            y: 0.0,
            z: 0.0,
        });
        center.z = z_center;
        let local = [
            Vec3 { x: -hx, y: -base_half, z: -hz },
            Vec3 { x: -hx, y: base_half, z: -hz },
            Vec3 { x: -hx, y: base_half, z: hz },
            Vec3 { x: -hx, y: -base_half, z: hz },
            Vec3 { x: hx, y: -tip_half, z: -hz },
            Vec3 { x: hx, y: tip_half, z: -hz },
            Vec3 { x: hx, y: tip_half, z: hz },
            Vec3 { x: hx, y: -tip_half, z: hz },
        ];
        let points: Vec<Vec3> = local
            .iter()
            .map(|p| add(center, rotate_vector(q, *p)))
            .collect();
        if let Some(tooth) = create_hull(&points, 8) {
            create_hull_shape(world, body_id, &shape_def, &tooth);
        }
    }
}

fn build_gear_body(
    world: &mut World,
    bodies: &mut Vec<VisBody>,
    position: Pos,
    tooth_center_radius: f32,
    disk_near: &box3d_rust::hull::HullData,
    disk_far: &box3d_rust::hull::HullData,
    axle: &box3d_rust::hull::HullData,
) -> BodyId {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = position;
    let body_id = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.1;
    create_hull_shape(world, body_id, &shape_def, disk_near);
    create_hull_shape(world, body_id, &shape_def, disk_far);
    create_hull_shape(world, body_id, &shape_def, axle);
    add_teeth(world, body_id, tooth_center_radius, -GEAR_Z);
    add_teeth(world, body_id, tooth_center_radius, GEAR_Z);

    let r = tooth_center_radius + TOOTH_HALF_WIDTH;
    bodies.push(VisBody::box_body(body_id.index1 - 1, r, r, GEAR_Z + GEAR_HALF_DEPTH));
    body_id
}

fn create_chain(
    world: &mut World,
    bodies: &mut Vec<VisBody>,
    top_body: BodyId,
    attach: Pos,
) -> BodyId {
    let capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -LINK_HALF_LENGTH,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: LINK_HALF_LENGTH,
            z: 0.0,
        },
        radius: LINK_RADIUS,
    };
    let shape_def = default_shape_def();
    let mut joint_def = default_revolute_joint_def();
    joint_def.max_motor_torque = 0.05;
    joint_def.enable_motor = true;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let mut position = Pos {
        x: attach.x,
        y: attach.y - LINK_HALF_LENGTH,
        z: attach.z,
    };
    let mut prev = top_body;
    for _ in 0..LINK_COUNT {
        body_def.position = position;
        let body_id = create_body(world, &body_def);
        create_capsule_shape(world, body_id, &shape_def, &capsule);
        bodies.push(VisBody::capsule_body(body_id.index1 - 1, &capsule));

        let pivot = Pos {
            x: position.x,
            y: position.y + LINK_HALF_LENGTH,
            z: attach.z,
        };
        joint_def.base.body_id_a = prev;
        joint_def.base.body_id_b = body_id;
        joint_def.base.local_frame_a.p = body_get_local_point(world, prev, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(world, body_id, pivot);
        create_revolute_joint(world, &joint_def);

        position.y -= 2.0 * LINK_HALF_LENGTH;
        prev = body_id;
    }
    prev
}

fn create_door(
    world: &mut World,
    bodies: &mut Vec<VisBody>,
    ground: BodyId,
    door_position: Pos,
    near_link: BodyId,
    far_link: BodyId,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = door_position;
    let door = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.density *= 0.5;
    shape_def.base_material.friction = 0.1;
    let box_hull = make_box_hull(0.05, DOOR_HALF_HEIGHT, DOOR_HALF_DEPTH);
    create_hull_shape(world, door, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_body(
        door.index1 - 1,
        0.05,
        DOOR_HALF_HEIGHT,
        DOOR_HALF_DEPTH,
    ));

    let links = [near_link, far_link];
    let depths = [-GEAR_Z, GEAR_Z];
    for i in 0..2 {
        let pivot = Pos {
            x: door_position.x,
            y: door_position.y + DOOR_HALF_HEIGHT,
            z: depths[i],
        };
        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = links[i];
        joint_def.base.body_id_b = door;
        joint_def.base.local_frame_a.p = body_get_local_point(world, links[i], pivot);
        joint_def.base.local_frame_b.p = vec3(0.0, DOOR_HALF_HEIGHT, depths[i]);
        joint_def.enable_motor = true;
        joint_def.max_motor_torque = 50.0;
        create_revolute_joint(world, &joint_def);
    }

    let slide = compute_quat_between_unit_vectors(VEC3_AXIS_X, VEC3_AXIS_Y);
    let mut joint_def = default_prismatic_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = door;
    joint_def.base.local_frame_a.p = body_get_local_point(world, ground, door_position);
    joint_def.base.local_frame_a.q = slide;
    joint_def.base.local_frame_b = Transform {
        p: VEC3_ZERO_LOCAL,
        q: slide,
    };
    joint_def.max_motor_force = 200.0;
    joint_def.enable_motor = true;
    joint_def.base.collide_connected = true;
    create_prismatic_joint(world, &joint_def);
}

const VEC3_ZERO_LOCAL: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

/// Simplified stairwell basin from boxes (C uses an earcut mesh extrusion).
fn create_basin(world: &mut World, bodies: &mut Vec<VisBody>) {
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.6;

    let mut add_box = |cx: f32, cy: f32, cz: f32, hx: f32, hy: f32, hz: f32| {
        let mut body_def = default_body_def();
        body_def.position = pos(cx, cy, cz);
        let body = create_body(world, &body_def);
        let hull = make_box_hull(hx, hy, hz);
        create_hull_shape(world, body, &shape_def, &hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, hx, hy, hz));
    };

    // Floor
    add_box(-1.0, -0.05, 0.0, 10.3, 0.15, 2.0);
    // Left tall wall
    add_box(-11.1, 3.5, 0.0, 0.2, 3.7, 2.0);
    // Right ledge wall
    add_box(9.1, 3.7, 0.0, 0.25, 3.5, 2.0);
    // Stair steps
    for i in 0..12 {
        let t = i as f32;
        add_box(-0.2 - 0.53 * t, 0.55 + 0.53 * t, 0.0, 0.28, 0.12, 2.0);
    }
    // Back wall
    add_box(-1.0, 3.5, -2.05, 10.3, 3.7, 0.05);
}

fn create_debris(world: &mut World, bodies: &mut Vec<VisBody>) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let mut shape_def = default_shape_def();
    shape_def.base_material.rolling_resistance = 0.3;
    let rock = create_rock(ROCK_RADIUS).expect("rock hull");

    let mut x = -5.0f32;
    for i in 0..8 {
        let mut y = 6.5 - 0.25 * i as f32;
        for j in 0..6 {
            let z = -1.0 + 0.35 * (j as f32 % 5.0);
            body_def.position = pos(x, y, z);
            body_def.rotation = make_quat_from_axis_angle(
                VEC3_AXIS_Y,
                0.3 * (i + j) as f32,
            );
            let body_id = create_body(world, &body_def);
            create_hull_shape(world, body_id, &shape_def, &rock);
            bodies.push(VisBody::sphere_body(body_id.index1 - 1, ROCK_RADIUS));
            y += 0.25;
        }
        x += 0.35;
    }
}

pub(crate) fn build_gear_lift() -> JointState {
    let mut world = new_world();
    let mut bodies = Vec::new();

    // AddGroundBox
    let mut ground_box_def = default_body_def();
    ground_box_def.position = pos(0.0, -1.0, 0.0);
    let ground_box = create_body(&mut world, &ground_box_def);
    let gh = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground_box, &default_shape_def(), &gh.base);
    bodies.push(VisBody::box_body(ground_box.index1 - 1, 20.0, 1.0, 20.0));

    let ground = create_body(&mut world, &default_body_def());
    create_basin(&mut world, &mut bodies);

    let disk_near = make_z_cylinder(
        GEAR_RADIUS,
        -GEAR_Z - GEAR_HALF_DEPTH,
        -GEAR_Z + GEAR_HALF_DEPTH,
        GEAR_SIDES,
    );
    let disk_far = make_z_cylinder(
        GEAR_RADIUS,
        GEAR_Z - GEAR_HALF_DEPTH,
        GEAR_Z + GEAR_HALF_DEPTH,
        GEAR_SIDES,
    );
    let axle = make_z_cylinder(AXLE_RADIUS, -GEAR_Z, GEAR_Z, AXLE_SIDES);

    let gear_position1 = pos(-4.25, 9.75, 0.0);
    let gear_position2 = pos(-2.25, 10.75, 0.0);

    let driver = build_gear_body(
        &mut world,
        &mut bodies,
        gear_position1,
        GEAR_RADIUS + TOOTH_HALF_HEIGHT,
        &disk_near,
        &disk_far,
        &axle,
    );
    let follower = build_gear_body(
        &mut world,
        &mut bodies,
        gear_position2,
        GEAR_RADIUS + TOOTH_HALF_WIDTH,
        &disk_near,
        &disk_far,
        &axle,
    );

    let mut revolute = default_revolute_joint_def();
    revolute.base.body_id_a = ground;
    revolute.base.body_id_b = driver;
    revolute.base.local_frame_a.p = body_get_local_point(&world, ground, gear_position1);
    revolute.base.local_frame_b.p = VEC3_ZERO_LOCAL;
    revolute.enable_motor = true;
    revolute.max_motor_torque = 30000.0;
    revolute.motor_speed = -0.3;
    let driver_joint = create_revolute_joint(&mut world, &revolute);

    revolute.base.body_id_b = follower;
    revolute.base.local_frame_a.p = body_get_local_point(&world, ground, gear_position2);
    revolute.base.local_frame_a.q = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.25 * PI);
    revolute.enable_motor = true;
    revolute.max_motor_torque = 0.5;
    revolute.motor_speed = 0.0;
    revolute.lower_angle = -0.3 * PI;
    revolute.upper_angle = 0.8 * PI;
    revolute.enable_limit = true;
    create_revolute_joint(&mut world, &revolute);

    let link_attach = Pos {
        x: gear_position2.x + GEAR_RADIUS + 2.0 * TOOTH_HALF_WIDTH + TOOTH_RADIUS,
        y: gear_position2.y,
        z: 0.0,
    };
    let door_position = Pos {
        x: link_attach.x,
        y: link_attach.y - (2.0 * LINK_COUNT as f32 * LINK_HALF_LENGTH + DOOR_HALF_HEIGHT),
        z: 0.0,
    };

    let near_link = create_chain(
        &mut world,
        &mut bodies,
        follower,
        Pos {
            x: link_attach.x,
            y: link_attach.y,
            z: -GEAR_Z,
        },
    );
    let far_link = create_chain(
        &mut world,
        &mut bodies,
        follower,
        Pos {
            x: link_attach.x,
            y: link_attach.y,
            z: GEAR_Z,
        },
    );
    create_door(
        &mut world,
        &mut bodies,
        ground,
        door_position,
        near_link,
        far_link,
    );
    create_debris(&mut world, &mut bodies);

    let mut state = empty_state(world, bodies, JointScene::GearLift);
    state.control_joint = driver_joint;
    state
}
