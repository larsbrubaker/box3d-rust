//! Ghost collision scenes for the mesh ghost handling tests.
//!
//! Test-support port of the part of `box3d-cpp-reference/shared/ghost_collisions.h` /
//! `ghost_collisions.c` that `MeshPeelCylinder` in `test_mesh.c` drives: the PEEL InternalEdges
//! mesh scene (`e_ghostInternal`), the body launch for that scene, and the per-step ghost
//! count. The other grounds (strip, rail, and height-field variants) serve the samples and the
//! headless ghost scan, which are not ported, so `create_ghost_ground` only builds the
//! internal-edge scene.
//!
//! Each ground is a profile in the xy-plane extruded along z and finely tessellated, so a
//! sliding body crosses many internal edges. A manifold that pushes on the body is a ghost
//! when its normal points outside the span of face normals under the body. Those are the ones
//! that block or launch a body.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{body_get_contact_data, body_get_world_center, create_body, destroy_body};
use crate::geometry::{Capsule, ShapeType, Sphere};
use crate::hull::{create_cylinder, make_box_hull, make_offset_box_hull, HullData};
use crate::id::{BodyId, ShapeId, NULL_BODY_ID, NULL_SHAPE_ID};
use crate::math_functions::{
    abs_float, make_quat_from_axis_angle, max_float, min_float, neg, Aabb, Pos, Vec2, Vec3, PI,
    VEC3_AXIS_Y, VEC3_ONE, VEC3_ZERO,
};
use crate::mesh::{create_mesh, MeshData, MeshDef};
use crate::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
    shape_get_aabb, shape_get_type,
};
use crate::types::{default_body_def, default_shape_def, BodyType};
use crate::world::World;

/// A conveyor side rail along x, the floor is on the -z side. (GHOST_RAIL_HEIGHT)
pub(crate) const GHOST_RAIL_HEIGHT: f32 = 0.35;

/// (GHOST_RAIL_THICKNESS)
pub(crate) const GHOST_RAIL_THICKNESS: f32 = 0.08;

/// (GHOST_HALF_LENGTH)
const GHOST_HALF_LENGTH: f32 = 20.0;

/// (GHOST_HALF_WIDTH)
const GHOST_HALF_WIDTH: f32 = 3.0;

/// Mesh variants of the PEEL InternalEdges presets. (GHOST_PEEL_PRESET_COUNT)
const GHOST_PEEL_PRESET_COUNT: usize = 4;

/// Chamfer has the longest profile. (GHOST_MAX_PROFILE_POINTS)
const GHOST_MAX_PROFILE_POINTS: usize = 128;

/// (GhostScene)
// The full C scene list; the ported tests build only the internal-edge scene.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum GhostScene {
    Flat,
    Concave,
    Cliff,
    Crest,
    Bumps,
    Internal,
    Chamfer,
    Rail,
}

/// (GhostBody)
// The full C body list; the ported tests launch only the cylinder.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum GhostBody {
    Box,
    HCapsule,
    VCapsule,
    Sphere,
    Cylinder,
}

/// (PeelPreset)
struct PeelPreset {
    column_count: i32,
    scale_x: f32,
    start_z: f32,
}

/// (s_peelPresets)
const PEEL_PRESETS: [PeelPreset; GHOST_PEEL_PRESET_COUNT] = [
    PeelPreset {
        column_count: 4,
        scale_x: 0.1,
        start_z: -55.0,
    },
    PeelPreset {
        column_count: 4,
        scale_x: 0.1,
        start_z: -54.0,
    },
    PeelPreset {
        column_count: 4,
        scale_x: 0.01,
        start_z: -55.0,
    },
    PeelPreset {
        column_count: 16,
        scale_x: 0.01,
        start_z: -54.0,
    },
];

/// (GhostSettings)
// The cell size and speed drive the strip grounds and the sliding launch, which the
// internal-edge scene does not use.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub(crate) struct GhostSettings {
    pub scene: GhostScene,
    pub body: GhostBody,
    pub peel_preset: usize,
    pub mirror: bool,

    // Build the ground from height fields instead of meshes. Each height field is its own
    // shape.
    pub height_field: bool,

    pub cell_size: f32,
    pub speed: f32,

    // Degrees
    pub yaw: f32,

    pub friction: f32,
    pub fixed_rotation: bool,
    pub speculative: bool,
}

/// (GetDefaultGhostSettings)
pub(crate) fn get_default_ghost_settings() -> GhostSettings {
    GhostSettings {
        scene: GhostScene::Flat,
        body: GhostBody::Box,
        peel_preset: 0,
        mirror: false,
        height_field: false,
        cell_size: 0.25,
        speed: 6.0,
        yaw: 0.0,
        friction: 0.0,
        fixed_rotation: false,
        speculative: true,
    }
}

/// (GhostCollisionsData)
pub(crate) struct GhostCollisionsData {
    pub settings: GhostSettings,
    profile: Vec<Vec2>,

    // The ground spans x in [-halfLength, halfLength] and z in [-halfWidth, halfWidth]
    half_length: f32,
    half_width: f32,
    meshes: Vec<MeshData>,

    // C also creates the 0.4 m cylinder that the other scenes launch.
    peel_cylinder: HullData,
    ground_id: BodyId,
    body_id: BodyId,
    shape_id: ShapeId,

    // Body bounds at the start of the step, where the contacts were computed. A fast body can
    // cross a fold within a step, so the bounds after the step can miss the face a contact
    // came from.
    contact_bounds: Aabb,

    // Steps in the current run
    run_steps: i32,

    // Completed runs, each ending when the body reaches the end or falls off
    pub run_count: i32,

    pub run_ghost_count: i32,
    pub ghost_count: i32,
}

/// The profile runs in +x with the solid below it. The mirror flips x in the data and the mesh
/// scale flips it back, so the world geometry is the same either way. (AddGrid)
fn add_grid(
    vertices: &mut Vec<Vec3>,
    indices: &mut Vec<i32>,
    points: &[Vec2],
    half_width: f32,
    z_count: i32,
    alternate: bool,
    mirror: bool,
) {
    let sign = if mirror { -1.0 } else { 1.0 };
    let stride = z_count + 1;
    let base = vertices.len() as i32;
    let point_count = points.len() as i32;

    for point in points {
        for j in 0..=z_count {
            let z = -half_width + 2.0 * half_width * j as f32 / z_count as f32;
            vertices.push(Vec3 {
                x: sign * point.x,
                y: point.y,
                z,
            });
        }
    }

    for i in 0..point_count - 1 {
        for j in 0..z_count {
            let a = base + i * stride + j;
            let b = a + 1;
            let c = a + stride + 1;
            let d = a + stride;

            let mut corners = [a, b, c, c, d, a];

            // Alternate the diagonal like cooked level geometry
            if alternate && ((i + j) & 1) != 0 {
                corners = [a, b, d, b, c, d];
            }

            for k in (0..6).step_by(3) {
                indices.push(corners[k]);
                indices.push(if mirror {
                    corners[k + 2]
                } else {
                    corners[k + 1]
                });
                indices.push(if mirror {
                    corners[k + 1]
                } else {
                    corners[k + 2]
                });
            }
        }
    }
}

/// (AddMesh)
fn add_mesh(
    world: &mut World,
    data: &mut GhostCollisionsData,
    vertices: Vec<Vec3>,
    indices: Vec<i32>,
    frictionless: bool,
) {
    let mesh_def = MeshDef {
        vertices,
        indices,
        weld_vertices: true,
        weld_tolerance: 0.001,
        identify_edges: true,
        ..Default::default()
    };

    let mesh = create_mesh(&mesh_def, None).expect("ghost ground mesh");

    let scale = if data.settings.mirror {
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: 1.0,
        }
    } else {
        VEC3_ONE
    };
    let mut shape_def = default_shape_def();
    if frictionless {
        shape_def.base_material.friction = 0.0;
    }
    create_mesh_shape(world, data.ground_id, &shape_def, &mesh, scale);
    data.meshes.push(mesh);
}

/// (PushProfile)
fn push_profile(data: &mut GhostCollisionsData, x: f32, y: f32) {
    if data.profile.len() < GHOST_MAX_PROFILE_POINTS {
        data.profile.push(Vec2 { x, y });
    }
}

/// Build the PEEL InternalEdges mesh ground. This is the `e_ghostInternal` mesh path of
/// `CreateGhostGround`, the only scene the ported tests use. (CreateGhostGround)
fn create_ghost_ground(world: &mut World, data: &mut GhostCollisionsData) {
    debug_assert!(data.settings.scene == GhostScene::Internal && !data.settings.height_field);

    if data.ground_id != NULL_BODY_ID {
        destroy_body(world, data.ground_id);
    }

    data.meshes.clear();

    let body_def = default_body_def();
    data.ground_id = create_body(world, &body_def);

    let settings = data.settings;
    data.profile.clear();
    data.half_length = GHOST_HALF_LENGTH;
    data.half_width = GHOST_HALF_WIDTH;

    // PEEL's curved mesh: 32 rows 4 m apart, each step up 1.25 times the last, sliding toward
    // +x
    for i in (0..32).rev() {
        push_profile(
            data,
            62.0 - 4.0 * i as f32,
            1.0 + 0.1 * 1.25f32.powf(i as f32),
        );
    }

    // The mesh has no friction and the ground plane under it does, as in PEEL
    let preset = &PEEL_PRESETS[settings.peel_preset];
    let z_count = preset.column_count - 1;
    let half_width = 20.0 * preset.scale_x * z_count as f32;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let profile = data.profile.clone();
    add_grid(
        &mut vertices,
        &mut indices,
        &profile,
        half_width,
        z_count,
        false,
        settings.mirror,
    );
    data.half_width = half_width;
    add_mesh(world, data, vertices, indices, true);

    let box_hull = make_offset_box_hull(
        140.0,
        1.0,
        30.0,
        Vec3 {
            x: 65.0,
            y: -1.0,
            z: 0.0,
        },
    );
    let shape_def = default_shape_def();
    create_hull_shape(world, data.ground_id, &shape_def, &box_hull.base);
}

/// Launch a new body from the start. PEEL drops its bodies from rest, sized in meters rather
/// than tenths. This is the `e_ghostInternal` branch of the launch position.
/// (LaunchGhostBody)
fn launch_ghost_body(world: &mut World, data: &mut GhostCollisionsData) {
    if data.body_id != NULL_BODY_ID {
        destroy_body(world, data.body_id);
    }

    let settings = data.settings;
    debug_assert!(settings.scene == GhostScene::Internal);

    let box_half = 1.0;
    let capsule_radius = 1.0;
    let capsule_half = 2.0;
    let sphere_radius = 1.0;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.enable_contact_recycling = false;
    let yaw = make_quat_from_axis_angle(VEC3_AXIS_Y, settings.yaw * PI / 180.0);

    body_def.position = Pos {
        x: PEEL_PRESETS[settings.peel_preset].start_z as _,
        y: 80.0 as _,
        z: 0.0 as _,
    };
    body_def.rotation = yaw;

    body_def.motion_locks.angular_x = settings.fixed_rotation;
    body_def.motion_locks.angular_y = settings.fixed_rotation;
    body_def.motion_locks.angular_z = settings.fixed_rotation;
    body_def.enable_sleep = false;
    body_def.name = "slider".to_string();
    data.body_id = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = settings.friction;
    shape_def.enable_speculative_contact = settings.speculative;

    data.shape_id = match settings.body {
        GhostBody::Box => {
            let box_hull = make_box_hull(box_half, box_half, box_half);
            create_hull_shape(world, data.body_id, &shape_def, &box_hull.base)
        }
        GhostBody::HCapsule => {
            let capsule = Capsule {
                center1: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: -capsule_half,
                },
                center2: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: capsule_half,
                },
                radius: capsule_radius,
            };
            create_capsule_shape(world, data.body_id, &shape_def, &capsule)
        }
        GhostBody::VCapsule => {
            let capsule = Capsule {
                center1: Vec3 {
                    x: 0.0,
                    y: -capsule_half,
                    z: 0.0,
                },
                center2: Vec3 {
                    x: 0.0,
                    y: capsule_half,
                    z: 0.0,
                },
                radius: capsule_radius,
            };
            create_capsule_shape(world, data.body_id, &shape_def, &capsule)
        }
        GhostBody::Sphere => {
            let sphere = Sphere {
                center: VEC3_ZERO,
                radius: sphere_radius,
            };
            create_sphere_shape(world, data.body_id, &shape_def, &sphere)
        }
        GhostBody::Cylinder => {
            create_hull_shape(world, data.body_id, &shape_def, &data.peel_cylinder)
        }
    };

    data.contact_bounds = shape_get_aabb(world, data.shape_id);
    data.run_ghost_count = 0;
    data.run_steps = 0;
}

/// Creates the ground and launches the body. (CreateGhostCollisions)
pub(crate) fn create_ghost_collisions(
    world: &mut World,
    settings: GhostSettings,
) -> GhostCollisionsData {
    let mut data = GhostCollisionsData {
        settings,
        profile: Vec::with_capacity(GHOST_MAX_PROFILE_POINTS),
        half_length: 0.0,
        half_width: 0.0,
        meshes: Vec::new(),
        peel_cylinder: create_cylinder(2.0, 1.0, -1.0, 16).expect("peel cylinder"),
        ground_id: NULL_BODY_ID,
        body_id: NULL_BODY_ID,
        shape_id: NULL_SHAPE_ID,
        contact_bounds: Aabb::default(),
        run_steps: 0,
        run_count: 0,
        run_ghost_count: 0,
        ghost_count: 0,
    };

    create_ghost_ground(world, &mut data);
    launch_ghost_body(world, &mut data);
    data
}

/// (SegmentAngle)
fn segment_angle(data: &GhostCollisionsData, k: usize) -> f32 {
    let a = data.profile[k];
    let b = data.profile[k + 1];
    (b.x - a.x).atan2(a.y - b.y)
}

/// The normal points from the ground to the body. A hull face can be the reference face, so any
/// normal within the span of face normals under the body is valid. At a side edge of the ground
/// the normal may also lean off that side, perpendicular to the edge. (IsGhost)
fn is_ghost(
    data: &GhostCollisionsData,
    normal: Vec3,
    x_min: f32,
    x_max: f32,
    z_min: f32,
    z_max: f32,
) -> bool {
    let tolerance = 5.0 * PI / 180.0;
    let sin_tolerance = tolerance.sin();
    if abs_float(normal.z) > sin_tolerance {
        // The manifold must reach the side it leans off
        let margin = 0.05;
        let at_side = if normal.z < 0.0 {
            z_min < -data.half_width + margin
        } else {
            z_max > data.half_width - margin
        };
        if !at_side {
            return true;
        }
    }

    // A normal leaning off a side has a short projection, which leaves little room to lean
    // along the edge
    let planar_length = (normal.x * normal.x + normal.y * normal.y).sqrt();
    if planar_length < sin_tolerance {
        return false;
    }

    let body = data.settings.body;
    let chamfer_hull = data.settings.scene == GhostScene::Chamfer
        && (body == GhostBody::Box || body == GhostBody::Cylinder);

    let mut lower = PI;
    let mut upper = -PI;
    let segment_count = data.profile.len() - 1;
    for k in 0..segment_count {
        let x0 = min_float(data.profile[k].x, data.profile[k + 1].x);
        let x1 = max_float(data.profile[k].x, data.profile[k + 1].x);
        if x1 < x_min || x_max < x0 {
            continue;
        }

        // A flat hull bottom resting on the top edges must keep its own normal, so for a hull
        // only the tops count. A round body's normal comes from the closest ground feature,
        // which can be a top edge.
        if chamfer_hull && (data.profile[k].y < 0.0 || data.profile[k + 1].y < 0.0) {
            continue;
        }

        let angle = segment_angle(data, k);
        lower = min_float(lower, angle);
        upper = max_float(upper, angle);
    }

    let angle = normal.y.atan2(normal.x);
    let deviation = max_float(lower - angle, angle - upper);
    if deviation <= 0.0 {
        return false;
    }

    // Measure the lean outside the span in 3D so a steep side normal is not over counted
    deviation > 0.5 * PI || planar_length * deviation.sin() > sin_tolerance
}

/// Call after each world step. Relaunches a body that finished its run, then counts the ghost
/// manifolds on the body. Returns the ghost count for this step. C also writes ghost reports
/// for the sample's display; the tests pass no report buffer. (UpdateGhostCollisions)
pub(crate) fn update_ghost_collisions(world: &mut World, data: &mut GhostCollisionsData) -> i32 {
    let mut center = body_get_world_center(world, data.body_id);
    let mut done = center.x > (data.half_length - 1.0) as _ || center.y < -3.0 as _;
    if data.settings.scene == GhostScene::Internal {
        // Run out onto the ground plane, or give up on a body knocked off the side
        done = center.x > 100.0 as _ || center.y < -3.0 as _ || data.run_steps > 900;
    }

    if done {
        data.run_count += 1;
        launch_ghost_body(world, data);
        center = body_get_world_center(world, data.body_id);
    }

    data.run_steps += 1;

    let bounds = data.contact_bounds;
    let x_min = bounds.lower_bound.x - 0.1;
    let x_max = bounds.upper_bound.x + 0.1;
    data.contact_bounds = shape_get_aabb(world, data.shape_id);

    let mut ghost_count = 0;
    let contacts = body_get_contact_data(world, data.body_id, 8);
    for contact in &contacts {
        let body_is_a = contact.shape_id_a == data.shape_id;
        let other_id = if body_is_a {
            contact.shape_id_b
        } else {
            contact.shape_id_a
        };
        let other_type = shape_get_type(world, other_id);
        if other_type != ShapeType::Mesh && other_type != ShapeType::Height {
            continue;
        }

        for manifold in &contact.manifolds {
            // Speculative points that never pushed are harmless
            let mut impulse = 0.0f32;
            for point in &manifold.points[..manifold.point_count as usize] {
                impulse += point.total_normal_impulse;
            }

            if manifold.point_count == 0 || impulse <= 0.0 {
                continue;
            }

            let mut z_min = f32::MAX;
            let mut z_max = -f32::MAX;
            for point in &manifold.points[..manifold.point_count as usize] {
                let anchor = if body_is_a {
                    point.anchor_a
                } else {
                    point.anchor_b
                };
                let z = center.z as f32 + anchor.z;
                z_min = min_float(z_min, z);
                z_max = max_float(z_max, z);
            }

            let normal = if body_is_a {
                neg(manifold.normal)
            } else {
                manifold.normal
            };
            if data.settings.scene == GhostScene::Rail {
                // The rail is the same all along x, so no valid normal leans along it
                if abs_float(normal.x) <= (5.0 * PI / 180.0).sin() {
                    continue;
                }
            } else if !is_ghost(data, normal, x_min, x_max, z_min, z_max) {
                continue;
            }

            ghost_count += 1;
        }
    }

    data.run_ghost_count += ghost_count;
    data.ghost_count += ghost_count;
    ghost_count
}
