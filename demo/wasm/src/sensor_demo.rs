//! Faithful C sensor demos: Sensor Visit, Sensor Hits (sample_events.cpp),
//! and Benchmark Sensor (sample_benchmark.cpp, browser-scaled 12×12).

use crate::vis::{pos, push_poses, sphere, vec3, VisBody};
use box3d_rust::body::{
    body_get_local_point, body_get_position, body_set_linear_velocity, create_body, destroy_body,
    make_body_id,
};
use box3d_rust::debug_draw::HexColor;
use box3d_rust::geometry::Capsule;
use box3d_rust::hull::{make_box_hull, make_cube_hull, make_transformed_box_hull};
use box3d_rust::id::{BodyId, JointId, ShapeId, NULL_BODY_ID, NULL_JOINT_ID, NULL_SHAPE_ID};
use box3d_rust::joint::{
    create_prismatic_joint, prismatic_joint_get_translation, prismatic_joint_set_motor_speed,
};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, offset_pos, Transform, Vec3, PI, QUAT_IDENTITY, VEC3_AXIS_Z, VEC3_ONE,
};
use box3d_rust::mesh::{create_grid_mesh, MeshData};
use box3d_rust::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape, shape_get_body,
    shape_get_user_data, shape_is_valid,
};
use box3d_rust::types::{
    default_body_def, default_prismatic_joint_def, default_shape_def, default_world_def, BodyType,
};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

const RAND_SEED: u32 = 12345;
const RAND_LIMIT: u32 = 32767;
const ACTIVE_SENSOR_COLOR: u32 = 0x505050;
const ACTIVE_USER_DATA: u64 = 1;
const BENCH_COLUMNS: i32 = 12;
const BENCH_ROWS: i32 = 12;

thread_local! {
    static STATE: RefCell<Option<SensorState>> = const { RefCell::new(None) };
}

struct XorShift32(u32);
impl XorShift32 {
    fn with_seed(seed: u32) -> Self { Self(seed) }
    fn next_int(&mut self) -> i32 {
        let mut x = self.0;
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        self.0 = x;
        (x % (RAND_LIMIT + 1)) as i32
    }
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let r = (self.next_int() as u32 & RAND_LIMIT) as f32 / RAND_LIMIT as f32;
        (hi - lo) * r + lo
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SceneKind { Visit, Hits, Benchmark }

struct SensorState {
    world: World,
    bodies: Vec<VisBody>,
    colors: Vec<u32>,
    is_sensor: Vec<bool>,
    kind: SceneKind,
    begin_total: u32,
    end_total: u32,
    last_begin: u32,
    last_end: u32,
    max_begin: u32,
    max_end: u32,
    visit_sensor: ShapeId,
    #[allow(dead_code)]
    grid_mesh: Option<MeshData>,
    kinematic_body: BodyId,
    joint_id: JointId,
    bullet_body: BodyId,
    is_bullet: bool,
    step_count: u32,
    last_step_count: u32,
    filter_row: i32,
    rng: XorShift32,
}

fn with_state<R>(f: impl FnOnce(&mut SensorState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot.as_mut().expect("sensor not initialized — call sensor_reset first"))
    })
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 { x: 0.0, y: -10.0, z: 0.0 };
    World::new(&def)
}

fn push_vis(
    bodies: &mut Vec<VisBody>, colors: &mut Vec<u32>, is_sensor: &mut Vec<bool>,
    mut body: VisBody, color: u32, sensor: bool,
) {
    body.color = color;
    bodies.push(body);
    colors.push(color);
    is_sensor.push(sensor);
}

fn remove_vis_for_body(
    bodies: &mut Vec<VisBody>, colors: &mut Vec<u32>, is_sensor: &mut Vec<bool>, body_index: i32,
) {
    if let Some(i) = bodies.iter().position(|b| b.body_index == body_index) {
        bodies.remove(i); colors.remove(i); is_sensor.remove(i);
    }
}

fn find_vis_index(bodies: &[VisBody], body_index: i32) -> Option<usize> {
    bodies.iter().position(|b| b.body_index == body_index)
}

fn set_vis_color(state: &mut SensorState, index: usize, color: u32) {
    state.colors[index] = color;
    state.bodies[index].color = color;
}

fn add_ground_box(world: &mut World, extent: f32) -> BodyId {
    let mut body_def = default_body_def();
    body_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(world, &body_def);
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape(world, ground, &default_shape_def(), &hull.base);
    ground
}

fn reset_visit() -> SensorState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let mut colors = Vec::new();
    let mut is_sensor = Vec::new();

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 12.5, 0.0);
    let visitor = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.enable_sensor_events = true;
    let dynamic_box = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, visitor, &shape_def, &dynamic_box.base);
    push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::box_body(visitor.index1 - 1, 0.5, 0.5, 0.5), 0, false);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Kinematic;
    body_def.position = pos(0.0, 2.0, 0.0);
    let sensor_body = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    let sensor_box = make_box_hull(2.0, 2.0, 2.0);
    let visit_sensor = create_hull_shape(&mut world, sensor_body, &shape_def, &sensor_box.base);
    push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::box_body(sensor_body.index1 - 1, 2.0, 2.0, 2.0), 0, true);

    SensorState {
        world, bodies, colors, is_sensor, kind: SceneKind::Visit,
        begin_total: 0, end_total: 0, last_begin: 0, last_end: 0, max_begin: 0, max_end: 0,
        visit_sensor, grid_mesh: None, kinematic_body: NULL_BODY_ID, joint_id: NULL_JOINT_ID,
        bullet_body: NULL_BODY_ID, is_bullet: true, step_count: 0, last_step_count: 0,
        filter_row: 0, rng: XorShift32::with_seed(RAND_SEED),
    }
}

fn reset_hits(is_bullet: bool) -> SensorState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let mut colors = Vec::new();
    let mut is_sensor = Vec::new();

    let ground = add_ground_box(&mut world, 10.0);
    push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::box_body(ground.index1 - 1, 10.0, 1.0, 10.0), 0, false);

    let body_def = default_body_def();
    let wall_body = create_body(&mut world, &body_def);
    let wall_xf = Transform { p: Vec3 { x: 10.0, y: 5.0, z: 0.0 }, q: QUAT_IDENTITY };
    let wall_box = make_transformed_box_hull(0.1, 5.0, 5.0, wall_xf);
    create_hull_shape(&mut world, wall_body, &default_shape_def(), &wall_box.base);
    push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::box_local(wall_body.index1 - 1, 0.1, 5.0, 5.0, wall_xf), 0, false);

    let grid_mesh = create_grid_mesh(2, 2, 5.0, 0, true).expect("hits grid mesh");
    let mesh_rot = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.5 * PI);
    let (mesh_hx, mesh_hy, mesh_hz) = (5.0, 0.05, 5.0);

    {
        let mut body_def = default_body_def();
        body_def.position = pos(-4.0, 6.0, 0.0);
        body_def.rotation = mesh_rot;
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        create_mesh_shape(&mut world, body, &shape_def, &grid_mesh, VEC3_ONE);
        push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::box_body(body.index1 - 1, mesh_hx, mesh_hy, mesh_hz), 0, true);
    }

    let kinematic_body = {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = pos(0.0, 6.0, 0.0);
        body_def.rotation = mesh_rot;
        body_def.linear_velocity = vec3(0.5, 0.0, 0.0);
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        create_mesh_shape(&mut world, body, &shape_def, &grid_mesh, VEC3_ONE);
        push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::box_body(body.index1 - 1, mesh_hx, mesh_hy, mesh_hz), 0, true);
        body
    };

    let joint_id = {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(4.0, 1.0, 0.0);
        let dynamic_body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        let capsule = Capsule {
            center1: Vec3 { x: 0.0, y: 1.0, z: 0.0 },
            center2: Vec3 { x: 0.0, y: 9.0, z: 0.0 },
            radius: 0.1,
        };
        create_capsule_shape(&mut world, dynamic_body, &shape_def, &capsule);
        push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::capsule_body(dynamic_body.index1 - 1, &capsule), 0, true);

        let pivot = offset_pos(body_def.position, Vec3 { x: 0.0, y: 6.0, z: 0.0 });
        let mut joint_def = default_prismatic_joint_def();
        joint_def.base.body_id_a = wall_body;
        joint_def.base.body_id_b = dynamic_body;
        joint_def.base.local_frame_a.p = body_get_local_point(&world, wall_body, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(&world, dynamic_body, pivot);
        joint_def.enable_motor = true;
        joint_def.max_motor_force = 1000.0;
        joint_def.motor_speed = 0.5;
        create_prismatic_joint(&mut world, &joint_def)
    };

    let mut state = SensorState {
        world, bodies, colors, is_sensor, kind: SceneKind::Hits,
        begin_total: 0, end_total: 0, last_begin: 0, last_end: 0, max_begin: 0, max_end: 0,
        visit_sensor: NULL_SHAPE_ID, grid_mesh: Some(grid_mesh), kinematic_body, joint_id,
        bullet_body: NULL_BODY_ID, is_bullet, step_count: 0, last_step_count: 0,
        filter_row: 0, rng: XorShift32::with_seed(RAND_SEED),
    };
    launch_bullet(&mut state);
    state
}

fn launch_bullet(state: &mut SensorState) {
    if state.kind != SceneKind::Hits { return; }
    if !state.bullet_body.is_null() {
        let idx = state.bullet_body.index1 - 1;
        destroy_body(&mut state.world, state.bullet_body);
        remove_vis_for_body(&mut state.bodies, &mut state.colors, &mut state.is_sensor, idx);
        state.bullet_body = NULL_BODY_ID;
    }
    state.begin_total = 0; state.end_total = 0; state.last_begin = 0; state.last_end = 0;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(-26.7, 6.0, 0.0);
    let speed = state.rng.range(200.0, 300.0);
    body_def.linear_velocity = vec3(speed, 0.0, 0.0);
    body_def.is_bullet = state.is_bullet;
    let body = create_body(&mut state.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.enable_sensor_events = true;
    shape_def.base_material.friction = 0.8;
    shape_def.base_material.rolling_resistance = 0.01;
    let sph = sphere(0.25);
    create_sphere_shape(&mut state.world, body, &shape_def, &sph);
    push_vis(&mut state.bodies, &mut state.colors, &mut state.is_sensor, VisBody::sphere_body(body.index1 - 1, 0.25), 0, false);
    state.bullet_body = body;
}

fn reset_benchmark() -> SensorState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let mut colors = Vec::new();
    let mut is_sensor = Vec::new();
    let mut rng = XorShift32::with_seed(42);

    let grid_size = 3.0;
    let half = 0.48 * grid_size;
    let box_hull = make_cube_hull(half);
    {
        let mut body_def = default_body_def();
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        shape_def.user_data = ACTIVE_USER_DATA;
        let count = BENCH_COLUMNS * 2 + 1;
        let mut x = -(BENCH_COLUMNS as f32) * grid_size;
        for _ in 0..count {
            body_def.position = pos(x, 0.0, 0.0);
            let id = create_body(&mut world, &body_def);
            create_hull_shape(&mut world, id, &shape_def, &box_hull.base);
            push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::box_body(id.index1 - 1, half, half, half), ACTIVE_SENSOR_COLOR, true);
            x += grid_size;
        }
    }

    let shift = 5.0;
    let x_center = 0.5 * shift * BENCH_COLUMNS as f32;
    let filter_row = BENCH_ROWS >> 1;
    let cube = make_cube_hull(0.5);
    let y_start = 10.0;
    {
        let mut body_def = default_body_def();
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        for j in 0..BENCH_ROWS {
            shape_def.user_data = ((j as u64) + 1) << 1;
            let color = if j == filter_row { HexColor::FUCHSIA.0 } else { 0 };
            let y = j as f32 * shift + y_start;
            for i in 0..BENCH_COLUMNS {
                let x = i as f32 * shift - x_center;
                body_def.position = pos(x, y, 0.0);
                let id = create_body(&mut world, &body_def);
                create_hull_shape(&mut world, id, &shape_def, &cube.base);
                push_vis(&mut bodies, &mut colors, &mut is_sensor, VisBody::box_body(id.index1 - 1, 0.5, 0.5, 0.5), color, true);
            }
        }
    }
    let _ = &mut rng;

    SensorState {
        world, bodies, colors, is_sensor, kind: SceneKind::Benchmark,
        begin_total: 0, end_total: 0, last_begin: 0, last_end: 0, max_begin: 0, max_end: 0,
        visit_sensor: NULL_SHAPE_ID, grid_mesh: None, kinematic_body: NULL_BODY_ID,
        joint_id: NULL_JOINT_ID, bullet_body: NULL_BODY_ID, is_bullet: true,
        step_count: 0, last_step_count: 0, filter_row, rng,
    }
}

fn create_bench_row(state: &mut SensorState, y: f32) {
    let shift = 5.0;
    let x_center = 0.5 * shift * BENCH_COLUMNS as f32;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.gravity_scale = 0.0;
    body_def.linear_velocity = vec3(0.0, -5.0, 0.0);
    let mut shape_def = default_shape_def();
    shape_def.enable_sensor_events = true;
    let sph = sphere(0.5);
    for i in 0..BENCH_COLUMNS {
        let y_offset = state.rng.range(-1.0, 1.0);
        body_def.position = pos(shift * i as f32 - x_center, y + y_offset, 0.0);
        let body = create_body(&mut state.world, &body_def);
        create_sphere_shape(&mut state.world, body, &shape_def, &sph);
        push_vis(&mut state.bodies, &mut state.colors, &mut state.is_sensor, VisBody::sphere_body(body.index1 - 1, 0.5), 0, false);
    }
}

fn process_visit_events(state: &mut SensorState) {
    let begins: Vec<_> = state.world.get_sensor_events().begin_events.to_vec();
    let ends: Vec<_> = state.world.get_sensor_events().end_events.to_vec();
    state.last_begin = begins.len() as u32;
    state.last_end = ends.len() as u32;
    state.begin_total = state.begin_total.wrapping_add(state.last_begin);
    state.end_total = state.end_total.wrapping_add(state.last_end);
    for event in &begins {
        if event.sensor_shape_id == state.visit_sensor {
            let visitor = shape_get_body(&state.world, event.visitor_shape_id);
            let idx = visitor.index1 - 1;
            destroy_body(&mut state.world, visitor);
            remove_vis_for_body(&mut state.bodies, &mut state.colors, &mut state.is_sensor, idx);
            break;
        }
    }
}

fn process_hits_pre_step(state: &mut SensorState) {
    let p = body_get_position(&state.world, state.kinematic_body);
    if (p.x as f32) > 1.0 {
        body_set_linear_velocity(&mut state.world, state.kinematic_body, vec3(-0.5, 0.0, 0.0));
    } else if (p.x as f32) < -1.0 {
        body_set_linear_velocity(&mut state.world, state.kinematic_body, vec3(0.5, 0.0, 0.0));
    }
    let x = prismatic_joint_get_translation(&state.world, state.joint_id);
    if x > 1.0 {
        prismatic_joint_set_motor_speed(&mut state.world, state.joint_id, -0.5);
    } else if x < -1.0 {
        prismatic_joint_set_motor_speed(&mut state.world, state.joint_id, 0.5);
    }
}

fn process_hits_events(state: &mut SensorState) {
    let begin_count = state.world.get_sensor_events().begin_events.len() as u32;
    let end_count = state.world.get_sensor_events().end_events.len() as u32;
    state.last_begin = begin_count;
    state.last_end = end_count;
    state.begin_total = state.begin_total.wrapping_add(begin_count);
    state.end_total = state.end_total.wrapping_add(end_count);
}

fn process_benchmark_events(state: &mut SensorState) {
    if state.step_count == state.last_step_count { return; }
    let begins: Vec<_> = state.world.get_sensor_events().begin_events.to_vec();
    let ends: Vec<_> = state.world.get_sensor_events().end_events.to_vec();
    state.last_begin = begins.len() as u32;
    state.last_end = ends.len() as u32;
    state.begin_total = state.begin_total.wrapping_add(state.last_begin);
    state.end_total = state.end_total.wrapping_add(state.last_end);
    state.max_begin = state.max_begin.max(state.last_begin);
    state.max_end = state.max_end.max(state.last_end);

    let mut zombie_indices: Vec<i32> = Vec::new();
    for event in &begins {
        let user_data = shape_get_user_data(&state.world, event.sensor_shape_id);
        if user_data == ACTIVE_USER_DATA {
            zombie_indices.push(shape_get_body(&state.world, event.visitor_shape_id).index1 - 1);
        } else if let Some(i) = find_vis_index(
            &state.bodies,
            shape_get_body(&state.world, event.visitor_shape_id).index1 - 1,
        ) {
            set_vis_color(state, i, HexColor::LIME.0);
            let _ = state.filter_row;
        }
    }
    for event in &ends {
        if !shape_is_valid(&state.world, event.visitor_shape_id) { continue; }
        let visitor = shape_get_body(&state.world, event.visitor_shape_id);
        if let Some(i) = find_vis_index(&state.bodies, visitor.index1 - 1) {
            set_vis_color(state, i, 0);
        }
    }
    zombie_indices.sort_unstable();
    zombie_indices.dedup();
    for idx in zombie_indices {
        let body_id = make_body_id(&state.world, idx);
        destroy_body(&mut state.world, body_id);
        remove_vis_for_body(&mut state.bodies, &mut state.colors, &mut state.is_sensor, idx);
    }
    if (state.step_count & 0x1F) == 0 {
        create_bench_row(state, 10.0 + BENCH_ROWS as f32 * 5.0);
    }
    state.last_step_count = state.step_count;
}

#[wasm_bindgen]
pub fn sensor_reset(scene: u32) -> u32 {
    STATE.with(|cell| {
        let prev_bullet = cell.borrow().as_ref().map(|s| s.is_bullet).unwrap_or(true);
        let state = match scene {
            1 => reset_hits(prev_bullet),
            2 => reset_benchmark(),
            _ => reset_visit(),
        };
        let count = state.bodies.len() as u32;
        *cell.borrow_mut() = Some(state);
        count
    })
}

#[wasm_bindgen]
pub fn sensor_set_bullet(flag: bool) {
    with_state(|state| { state.is_bullet = flag; });
}

#[wasm_bindgen]
pub fn sensor_is_bullet() -> bool {
    with_state(|state| state.is_bullet)
}

#[wasm_bindgen]
pub fn sensor_launch() {
    with_state(launch_bullet);
}

#[wasm_bindgen]
pub fn sensor_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        if state.kind == SceneKind::Hits { process_hits_pre_step(state); }
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.wrapping_add(1);
        match state.kind {
            SceneKind::Visit => process_visit_events(state),
            SceneKind::Hits => process_hits_events(state),
            SceneKind::Benchmark => process_benchmark_events(state),
        }
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn sensor_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn sensor_colors() -> Vec<f32> {
    with_state(|state| state.colors.iter().map(|&c| c as f32).collect())
}

#[wasm_bindgen]
pub fn sensor_sensor_indices() -> Vec<u32> {
    with_state(|state| {
        state.is_sensor.iter().enumerate().filter(|(_, s)| **s).map(|(i, _)| i as u32).collect()
    })
}

#[wasm_bindgen]
pub fn sensor_event_stats() -> Vec<f32> {
    with_state(|state| {
        let (a, b, flag) = match state.kind {
            SceneKind::Benchmark => (state.max_begin, state.max_end, 1.0),
            _ => (state.begin_total, state.end_total, 0.0),
        };
        vec![a as f32, b as f32, state.last_begin as f32, state.last_end as f32, flag]
    })
}
