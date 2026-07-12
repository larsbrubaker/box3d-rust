//! Gear Lift sample — faithful port of C `GearLift` from `sample_joint.cpp`.

use crate::joint_demo::{empty_state, new_world, JointScene, JointState};
use crate::vis::{pos, vec3, VisBody};
use box3d_rust::body::{body_get_local_point, create_body};
use box3d_rust::debug_draw::HexColor;
use box3d_rust::geometry::Capsule;
use box3d_rust::hull::{create_hull, create_rock, make_box_hull, make_offset_box_hull};
use box3d_rust::id::BodyId;
use box3d_rust::joint::{create_prismatic_joint, create_revolute_joint};
use box3d_rust::math_functions::{
    add, compute_quat_between_unit_vectors, make_quat_from_axis_angle, rotate_vector, Pos, Quat,
    Transform, Vec3, PI, QUAT_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ONE, VEC3_ZERO,
};
use box3d_rust::mesh::{create_mesh, MeshData, MeshDef};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_mesh_shape};
use box3d_rust::types::{
    default_body_def, default_prismatic_joint_def, default_revolute_joint_def, default_shape_def,
    BodyType,
};
use box3d_rust::world::World;
use std::cell::Cell;

const GEAR_RADIUS: f32 = 1.0;
const GEAR_HALF_DEPTH: f32 = 0.125;
const GEAR_Z: f32 = 1.5;
const AXLE_RADIUS: f32 = 0.2;
const TOOTH_HALF_WIDTH: f32 = 0.11;
const TOOTH_HALF_HEIGHT: f32 = 0.09;
const TOOTH_RADIUS: f32 = 0.03;
const LINK_HALF_LENGTH: f32 = 0.07;
const LINK_RADIUS: f32 = 0.05;
const LINK_COUNT: i32 = 40;
const DOOR_HALF_HEIGHT: f32 = 1.5;
const DOOR_HALF_DEPTH: f32 = 1.95;
const GEAR_SIDES: i32 = 24;
const AXLE_SIDES: i32 = 12;
const ROCK_RADIUS: f32 = 0.3;

const COLOR_SADDLE_BROWN: u32 = HexColor::SADDLE_BROWN.0;
const COLOR_SLATE_GRAY: u32 = HexColor::SLATE_GRAY.0;
const COLOR_GRAY: u32 = HexColor::GRAY.0;
const COLOR_LIGHT_STEEL_BLUE: u32 = HexColor::LIGHT_STEEL_BLUE.0;
const COLOR_DARK_CYAN: u32 = HexColor::DARK_CYAN.0;
const COLOR_DARK_SEA_GREEN: u32 = HexColor::DARK_SEA_GREEN.0;

const BASIN_CAP: [i32; 90] = [
    1, 2, 3, 1, 3, 4, 0, 1, 4, 0, 4, 5, 31, 0, 5, 5, 6, 7, 31, 5, 7, 7, 8, 9, 31, 7, 9, 9, 10, 11,
    31, 9, 11, 11, 12, 13, 31, 11, 13, 13, 14, 15, 31, 13, 15, 15, 16, 17, 31, 15, 17, 17, 18, 19,
    31, 17, 19, 19, 20, 21, 31, 19, 21, 21, 22, 23, 31, 21, 23, 23, 24, 25, 31, 23, 25, 25, 26, 27,
    31, 25, 27, 27, 28, 29, 31, 27, 29, 29, 30, 31,
];

const BASIN_POINTS: [(f32, f32); 32] = [
    (-11.3000, -0.2167),
    (9.3375, -0.2167),
    (9.3375, 7.1917),
    (8.8083, 7.1917),
    (8.8083, 0.3125),
    (0.3417, 0.3125),
    (0.3417, 0.8417),
    (-0.1875, 0.8417),
    (-0.1875, 1.3708),
    (-0.7167, 1.3708),
    (-0.7167, 1.9000),
    (-1.2458, 1.9000),
    (-1.2458, 2.4292),
    (-1.7750, 2.4292),
    (-1.7750, 2.9583),
    (-2.3042, 2.9583),
    (-2.3042, 3.4875),
    (-2.8333, 3.4875),
    (-2.8333, 4.0167),
    (-3.3625, 4.0167),
    (-3.3625, 4.5458),
    (-3.8917, 4.5458),
    (-3.8917, 5.0750),
    (-4.4208, 5.0750),
    (-4.4208, 5.6042),
    (-4.9500, 5.6042),
    (-4.9500, 6.1333),
    (-5.4792, 6.1333),
    (-5.4792, 6.6625),
    (-6.0083, 6.6625),
    (-6.0083, 7.1917),
    (-11.3000, 7.1917),
];

const RAND_LIMIT: u32 = 32767;
thread_local! {
    static RANDOM_SEED: Cell<u32> = const { Cell::new(12345) };
}

fn random_int() -> i32 {
    RANDOM_SEED.with(|seed| {
        let mut x = seed.get();
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        seed.set(x);
        (x % (RAND_LIMIT + 1)) as i32
    })
}

fn random_int_range(lo: i32, hi: i32) -> i32 {
    lo + random_int() % (hi - lo + 1)
}

fn random_float_range(lo: f32, hi: f32) -> f32 {
    let r = (random_int() as u32 & RAND_LIMIT) as f32;
    let r = r / RAND_LIMIT as f32;
    (hi - lo) * r + lo
}

fn random_quat() -> Quat {
    let u1 = random_float_range(0.0, 1.0);
    let u2 = random_float_range(0.0, 2.0 * PI);
    let u3 = random_float_range(0.0, 2.0 * PI);
    crate::vis::random_quat_from(u1, u2, u3)
}

fn cyl_z_to_y() -> Quat {
    compute_quat_between_unit_vectors(VEC3_AXIS_Y, VEC3_AXIS_Z)
}

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

fn push_cap(
    indices: &mut Vec<i32>,
    r0: i32,
    r1: i32,
    r2: i32,
    v_offset: i32,
    want_positive_z: bool,
) {
    let p0 = BASIN_POINTS[r0 as usize];
    let p1 = BASIN_POINTS[r1 as usize];
    let p2 = BASIN_POINTS[r2 as usize];
    let cross = (p1.0 - p0.0) * (p2.1 - p0.1) - (p1.1 - p0.1) * (p2.0 - p0.0);
    let positive = cross > 0.0;
    let v0 = 2 * r0 + v_offset;
    let v1 = 2 * r1 + v_offset;
    let v2 = 2 * r2 + v_offset;
    indices.push(v0);
    if positive == want_positive_z {
        indices.push(v1);
        indices.push(v2);
    } else {
        indices.push(v2);
        indices.push(v1);
    }
}

fn mesh_wireframe(mesh: &MeshData) -> Vec<f32> {
    crate::vis::mesh_triangle_edges(mesh, VEC3_ONE)
}

fn create_basin_mesh(world: &mut World, ground: BodyId, bodies: &mut Vec<VisBody>) -> Vec<f32> {
    let z_min = -2.0f32;
    let z_max = 2.0f32;
    let mut vertices = Vec::with_capacity(64);
    for &(x, y) in &BASIN_POINTS {
        vertices.push(Vec3 { x, y, z: z_min });
        vertices.push(Vec3 { x, y, z: z_max });
    }
    let mut indices = Vec::new();
    for i in 0..32 {
        let j = (i + 1) % 32;
        let a_lo = 2 * i;
        let a_hi = 2 * i + 1;
        let b_lo = 2 * j;
        let b_hi = 2 * j + 1;
        indices.extend_from_slice(&[a_lo, b_lo, b_hi, a_lo, b_hi, a_hi]);
    }
    let mut k = 0;
    while k + 2 < BASIN_CAP.len() {
        let r0 = BASIN_CAP[k];
        let r1 = BASIN_CAP[k + 1];
        let r2 = BASIN_CAP[k + 2];
        push_cap(&mut indices, r0, r1, r2, 1, true);
        push_cap(&mut indices, r0, r1, r2, 0, false);
        k += 3;
    }
    let def = MeshDef {
        vertices,
        indices,
        identify_edges: true,
        ..Default::default()
    };
    let mesh = create_mesh(&def, None).expect("basin mesh");
    let mut shape_def = default_shape_def();
    shape_def.base_material.custom_color = COLOR_DARK_SEA_GREEN;
    create_mesh_shape(world, ground, &shape_def, &mesh, VEC3_ONE);

    let mut lower_x = BASIN_POINTS[0].0;
    let mut lower_y = BASIN_POINTS[0].1;
    let mut upper_x = BASIN_POINTS[0].0;
    let mut upper_y = BASIN_POINTS[0].1;
    for &(x, y) in &BASIN_POINTS[1..] {
        lower_x = lower_x.min(x);
        lower_y = lower_y.min(y);
        upper_x = upper_x.max(x);
        upper_y = upper_y.max(y);
    }
    let wall_half_thick = 0.05f32;
    let wall_center = Vec3 {
        x: 0.5 * (lower_x + upper_x),
        y: 0.5 * (lower_y + upper_y),
        z: -z_max - wall_half_thick,
    };
    let wall = make_offset_box_hull(
        0.5 * (upper_x - lower_x),
        0.5 * (upper_y - lower_y),
        wall_half_thick,
        wall_center,
    );
    create_hull_shape(world, ground, &shape_def, &wall.base);
    bodies.push(VisBody::box_local_colored(
        ground.index1 - 1,
        0.5 * (upper_x - lower_x),
        0.5 * (upper_y - lower_y),
        wall_half_thick,
        Transform {
            p: wall_center,
            q: QUAT_IDENTITY,
        },
        COLOR_DARK_SEA_GREEN,
    ));
    mesh_wireframe(&mesh)
}

fn add_teeth(
    world: &mut World,
    bodies: &mut Vec<VisBody>,
    body_id: BodyId,
    tooth_center_radius: f32,
    z_center: f32,
) {
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.1;
    shape_def.base_material.custom_color = COLOR_GRAY;
    let count = 16;
    let delta = 2.0 * PI / count as f32;
    let hx = TOOTH_HALF_WIDTH;
    let hz = GEAR_HALF_DEPTH;
    let base_half = TOOTH_HALF_HEIGHT;
    let tip_half = TOOTH_HALF_HEIGHT - TOOTH_RADIUS;
    let body_index = body_id.index1 - 1;
    for i in 0..count {
        let q = make_quat_from_axis_angle(VEC3_AXIS_Z, i as f32 * delta);
        let mut center = rotate_vector(
            q,
            Vec3 {
                x: tooth_center_radius,
                y: 0.0,
                z: 0.0,
            },
        );
        center.z = z_center;
        let local = [
            Vec3 {
                x: -hx,
                y: -base_half,
                z: -hz,
            },
            Vec3 {
                x: -hx,
                y: base_half,
                z: -hz,
            },
            Vec3 {
                x: -hx,
                y: base_half,
                z: hz,
            },
            Vec3 {
                x: -hx,
                y: -base_half,
                z: hz,
            },
            Vec3 {
                x: hx,
                y: -tip_half,
                z: -hz,
            },
            Vec3 {
                x: hx,
                y: tip_half,
                z: -hz,
            },
            Vec3 {
                x: hx,
                y: tip_half,
                z: hz,
            },
            Vec3 {
                x: hx,
                y: -tip_half,
                z: hz,
            },
        ];
        let points: Vec<Vec3> = local
            .iter()
            .map(|p| add(center, rotate_vector(q, *p)))
            .collect();
        if let Some(tooth) = create_hull(&points, 8) {
            create_hull_shape(world, body_id, &shape_def, &tooth);
        }
        bodies.push(VisBody::box_local_colored(
            body_index,
            hx,
            base_half,
            hz,
            Transform { p: center, q },
            COLOR_GRAY,
        ));
    }
}

fn push_gear_vis(bodies: &mut Vec<VisBody>, body_index: i32) {
    let q_yz = cyl_z_to_y();
    bodies.push(VisBody::cylinder_local(
        body_index,
        GEAR_RADIUS,
        GEAR_HALF_DEPTH,
        Transform {
            p: Vec3 {
                x: 0.0,
                y: 0.0,
                z: -GEAR_Z,
            },
            q: q_yz,
        },
        COLOR_SADDLE_BROWN,
    ));
    bodies.push(VisBody::cylinder_local(
        body_index,
        GEAR_RADIUS,
        GEAR_HALF_DEPTH,
        Transform {
            p: Vec3 {
                x: 0.0,
                y: 0.0,
                z: GEAR_Z,
            },
            q: q_yz,
        },
        COLOR_SADDLE_BROWN,
    ));
    bodies.push(VisBody::cylinder_local(
        body_index,
        AXLE_RADIUS,
        GEAR_Z,
        Transform {
            p: VEC3_ZERO,
            q: q_yz,
        },
        COLOR_SLATE_GRAY,
    ));
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
    let body_index = body_id.index1 - 1;
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.1;
    shape_def.base_material.custom_color = COLOR_SADDLE_BROWN;
    create_hull_shape(world, body_id, &shape_def, disk_near);
    create_hull_shape(world, body_id, &shape_def, disk_far);
    shape_def.base_material.custom_color = COLOR_SLATE_GRAY;
    create_hull_shape(world, body_id, &shape_def, axle);
    push_gear_vis(bodies, body_index);
    add_teeth(world, bodies, body_id, tooth_center_radius, -GEAR_Z);
    add_teeth(world, bodies, body_id, tooth_center_radius, GEAR_Z);
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
    let mut shape_def = default_shape_def();
    shape_def.base_material.custom_color = COLOR_LIGHT_STEEL_BLUE;
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
        bodies.push(VisBody::capsule_colored(
            body_id.index1 - 1,
            &capsule,
            COLOR_LIGHT_STEEL_BLUE,
        ));
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
    shape_def.base_material.custom_color = COLOR_DARK_CYAN;
    let box_hull = make_box_hull(0.05, DOOR_HALF_HEIGHT, DOOR_HALF_DEPTH);
    create_hull_shape(world, door, &shape_def, &box_hull.base);
    bodies.push(VisBody::box_colored(
        door.index1 - 1,
        0.05,
        DOOR_HALF_HEIGHT,
        DOOR_HALF_DEPTH,
        COLOR_DARK_CYAN,
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
        p: VEC3_ZERO,
        q: slide,
    };
    joint_def.max_motor_force = 200.0;
    joint_def.enable_motor = true;
    joint_def.base.collide_connected = true;
    create_prismatic_joint(world, &joint_def);
}

fn create_debris(world: &mut World, bodies: &mut Vec<VisBody>) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let mut shape_def = default_shape_def();
    shape_def.base_material.rolling_resistance = 0.3;
    let colors = [
        HexColor::GRAY.0,
        HexColor::GAINSBORO.0,
        HexColor::LIGHT_GRAY.0,
        HexColor::LIGHT_SLATE_GRAY.0,
        HexColor::DARK_GRAY.0,
    ];
    let rock = create_rock(ROCK_RADIUS).expect("rock hull");
    let mut x = -5.0f32;
    for i in 0..12 {
        let mut y = 6.5 - 0.25 * i as f32;
        for _j in 0..10 {
            body_def.position = pos(x, y, random_float_range(-1.65, 0.35));
            body_def.rotation = random_quat();
            let body_id = create_body(world, &body_def);
            let color = colors[random_int_range(0, 4) as usize];
            shape_def.base_material.custom_color = color;
            create_hull_shape(world, body_id, &shape_def, &rock);
            bodies.push(VisBody::icosahedron_colored(
                body_id.index1 - 1,
                ROCK_RADIUS,
                color,
            ));
            y += 0.2;
        }
        x += 0.3;
    }
}

pub(crate) fn build_gear_lift() -> JointState {
    let mut world = new_world();
    let mut bodies = Vec::new();

    let mut ground_box_def = default_body_def();
    ground_box_def.position = pos(0.0, -1.0, 0.0);
    let ground_box = create_body(&mut world, &ground_box_def);
    let gh = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground_box, &default_shape_def(), &gh.base);
    bodies.push(VisBody::box_body(ground_box.index1 - 1, 20.0, 1.0, 20.0));

    let ground = create_body(&mut world, &default_body_def());
    let terrain_wire = create_basin_mesh(&mut world, ground, &mut bodies);

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
    revolute.base.local_frame_b.p = VEC3_ZERO;
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
    state.terrain_wire = terrain_wire;
    state
}
