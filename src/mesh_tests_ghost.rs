//! Port of the mesh ghost collision tests from box3d-cpp-reference/test/test_mesh.c
//! (MeshWideHullSlide, MeshManyTriangles, MeshSphereOnRidge, MeshHullOnSaddleRidge,
//! MeshCapsuleOverCrest, MeshApexContact, MeshPeelCylinder, MeshGhostOracle). Split from
//! mesh_tests.rs to keep files under the length limit.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{
    body_get_contact_data, body_get_linear_velocity, body_get_position, body_get_transform,
    body_get_world_point, create_body,
};
use crate::constants::linear_slop;
use crate::contact::collide_mesh_and_hull;
use crate::geometry::{Capsule, Sphere};
use crate::hull::make_box_hull;
use crate::id::BodyId;
use crate::manifold::Manifold;
use crate::math_functions::{
    abs_float, clamp_float, distance_squared, dot, length, length_squared, max, max_float, min,
    min_float, mul_add, mul_sv, neg, normalize, sub, to_pos, to_vec3, Pos, Transform, Vec2, Vec3,
    PI, QUAT_IDENTITY, VEC3_ONE, VEC3_ZERO,
};
use crate::mesh::{create_grid_mesh, create_mesh, Mesh, MeshData, MeshDef};
use crate::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape, shape_get_aabb,
};
use crate::test_support::ghost_collisions::{
    create_ghost_collisions, get_default_ghost_settings, update_ghost_collisions, GhostBody,
    GhostScene,
};
use crate::test_support::ghost_oracle::{
    create_oracle_mesh, scan_ghost_oracle, ORACLE_SCAN_POSES, ORACLE_SCENES, ORACLE_SCENE_COUNT,
    ORACLE_SCENE_NAMES,
};
use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use crate::world::World;

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

/// A static ground body holding the mesh at the given scale.
fn create_mesh_ground(world: &mut World, mesh: &MeshData, scale: Vec3) -> BodyId {
    let body_def = default_body_def();
    let ground_id = create_body(world, &body_def);
    let shape_def = default_shape_def();
    create_mesh_shape(world, ground_id, &shape_def, mesh, scale);
    ground_id
}

/// A wide box sliding over a fine grid sits on hundreds of flat internal edges. With no
/// friction any manifold off the face normal is a ghost that slows or lifts the box.
/// (WideHullSlide)
fn wide_hull_slide(mesh_scale: Vec3) {
    let mut world = World::new(&default_world_def());

    let mesh = create_grid_mesh(120, 40, 0.25, 0, true).expect("grid mesh");
    create_mesh_ground(&mut world, &mesh, mesh_scale);

    let speed = 4.0;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(-10.0, 0.5, 0.0);
    body_def.linear_velocity = v(speed, 0.0, 0.0);
    let box_id = create_body(&mut world, &body_def);

    let box_hull = make_box_hull(1.25, 0.5, 1.25);
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.0;
    create_hull_shape(&mut world, box_id, &shape_def, &box_hull.base);

    let mut ghost_count = 0;
    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);

        let contacts = body_get_contact_data(&world, box_id, 4);
        for contact in &contacts {
            for manifold in &contact.manifolds {
                if abs_float(manifold.normal.y) < 0.99 {
                    ghost_count += 1;
                }
            }
        }
    }

    let velocity = body_get_linear_velocity(&world, box_id);

    assert_eq!(ghost_count, 0);
    assert!(velocity.x > 0.99 * speed);
    assert!(abs_float(velocity.y) < 0.01);
}

/// The box overlaps more than a thousand grid triangles. All of them must reach the narrow
/// phase, so the reduced manifold reaches all four sides of the footprint. (MeshManyTriangles)
#[test]
fn mesh_many_triangles() {
    let mut world = World::new(&default_world_def());

    let mesh = create_grid_mesh(40, 40, 0.25, 0, true).expect("grid mesh");
    create_mesh_ground(&mut world, &mesh, VEC3_ONE);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 0.5, 0.0);
    let box_id = create_body(&mut world, &body_def);

    let box_hull = make_box_hull(3.0, 0.5, 3.0);
    let shape_def = default_shape_def();
    let box_shape_id = create_hull_shape(&mut world, box_id, &shape_def, &box_hull.base);

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }

    let position = body_get_position(&world, box_id);
    let rotation = body_get_transform(&world, box_id).q;

    assert!(abs_float(position.y as f32 - 0.5) < 0.02);
    assert!(abs_float(rotation.v.x) < 0.001);
    assert!(abs_float(rotation.v.z) < 0.001);

    let mut lower = v(f32::MAX, f32::MAX, f32::MAX);
    let mut upper = v(-f32::MAX, -f32::MAX, -f32::MAX);

    let contacts = body_get_contact_data(&world, box_id, 4);
    for contact in &contacts {
        let box_is_a = contact.shape_id_a == box_shape_id;
        for manifold in &contact.manifolds {
            for point in &manifold.points[..manifold.point_count as usize] {
                let anchor = if box_is_a {
                    point.anchor_a
                } else {
                    point.anchor_b
                };
                lower = min(lower, anchor);
                upper = max(upper, anchor);
            }
        }
    }

    assert!(lower.x < -2.5 && upper.x > 2.5);
    assert!(lower.z < -2.5 && upper.z > 2.5);
}

/// The grid is symmetric in x, so the mirror only changes the winding. (MeshWideHullSlide)
#[test]
fn mesh_wide_hull_slide() {
    wide_hull_slide(VEC3_ONE);
    wide_hull_slide(v(-1.0, 1.0, 1.0));
}

/// (HasVertex)
fn has_vertex(mesh: &MeshData, triangle_index: i32, point: Vec3) -> bool {
    let triangle = mesh.triangles[triangle_index as usize];
    let vertices = &mesh.vertices;
    [triangle.index1, triangle.index2, triangle.index3]
        .iter()
        .any(|&index| distance_squared(vertices[index as usize], point) < 1.0e-8)
}

/// A box face rests on a ridge that runs into a rising wall, so the ridge end is a saddle
/// vertex. The ridge sags slightly away from the saddle, leaving the saddle the deepest point
/// by less than linear slop. The ridge edge admits the up normal and the saddle cone does not,
/// so the deepest points have to be taken as the edge they span, not as the single deepest
/// vertex, or the contact snaps to a slope. (MeshHullOnSaddleRidge)
#[test]
fn mesh_hull_on_saddle_ridge() {
    let ridge_start = v(-2.0, -0.004, 0.0);
    let saddle = v(0.0, 0.0, 0.0);

    let vertices = vec![
        v(-2.0, -0.7, -1.0),
        ridge_start,
        v(-2.0, -0.7, 1.0),
        v(0.0, -0.7, -1.0),
        saddle,
        v(0.0, -0.7, 1.0),
        v(1.0, -0.4, -1.0),
        v(1.0, 0.3, 0.0),
        v(1.0, -0.4, 1.0),
    ];

    let indices = vec![
        0, 1, 3, 3, 1, 4, 1, 2, 4, 4, 2, 5, 3, 4, 6, 6, 4, 7, 4, 5, 7, 7, 5, 8,
    ];

    let def = MeshDef {
        vertices,
        indices,
        identify_edges: true,
        ..Default::default()
    };
    let mesh_data = create_mesh(&def, None).expect("saddle ridge mesh");

    // The footprint covers the saddle and a stretch of the ridge, slightly overlapping
    let hy = 0.1;
    let box_hull = make_box_hull(0.46, hy, 0.5);
    let transform = Transform {
        p: v(-0.44, hy - 0.004, 0.0),
        q: QUAT_IDENTITY,
    };

    let mesh = Mesh::new(&mesh_data, VEC3_ONE);
    let mut manifolds = vec![Manifold::default(); 16];
    let manifold_count =
        collide_mesh_and_hull(&mut manifolds, &mesh, &box_hull.base, transform, true, true);

    let mut ridge_point_count = 0;
    for manifold in &manifolds[..manifold_count as usize] {
        for point in &manifold.points[..manifold.point_count as usize] {
            let triangle_index = point.triangle_index;
            if has_vertex(&mesh_data, triangle_index, ridge_start)
                && has_vertex(&mesh_data, triangle_index, saddle)
            {
                ridge_point_count += 1;
                assert!(manifold.normal.y > 0.999);
            }
        }
    }

    assert!(ridge_point_count > 0);
}

/// A sphere balanced on a convex ridge is held up by edge contacts alone. Their normals lie
/// between the two face normals, so neighbor validation must keep them. (MeshSphereOnRidge)
#[test]
fn mesh_sphere_on_ridge() {
    let vertices = vec![
        v(-2.0, -0.7, -1.0),
        v(0.0, 0.0, -1.0),
        v(2.0, -0.7, -1.0),
        v(-2.0, -0.7, 1.0),
        v(0.0, 0.0, 1.0),
        v(2.0, -0.7, 1.0),
    ];

    let indices = vec![0, 3, 1, 3, 4, 1, 1, 4, 2, 4, 5, 2];

    let def = MeshDef {
        vertices,
        indices,
        identify_edges: true,
        ..Default::default()
    };
    let mesh = create_mesh(&def, None).expect("ridge mesh");

    let mut world = World::new(&default_world_def());
    create_mesh_ground(&mut world, &mesh, VEC3_ONE);

    let radius = 0.25;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, radius, 0.0);
    let sphere_id = create_body(&mut world, &body_def);
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius,
    };
    let shape_def = default_shape_def();
    create_sphere_shape(&mut world, sphere_id, &shape_def, &sphere);

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);

        let contacts = body_get_contact_data(&world, sphere_id, 4);
        assert_eq!(contacts.len(), 1);
        assert!(!contacts[0].manifolds.is_empty());

        for manifold in &contacts[0].manifolds {
            assert!(abs_float(manifold.normal.y) > 0.99);
        }
    }

    let position = body_get_position(&world, sphere_id);
    assert!(position.y as f32 > 0.9 * radius);
}

/// The normal points from the mesh to the body. Any normal in the span of the face normals
/// under the body is real, anything else is a ghost. (IsProfileGhost)
fn is_profile_ghost(profile: &[Vec2], normal: Vec3, x_min: f32, x_max: f32) -> bool {
    let tolerance = 5.0 * PI / 180.0;
    if abs_float(normal.z) > tolerance.sin() {
        return true;
    }

    let mut lower = PI;
    let mut upper = -PI;
    for k in 0..profile.len() - 1 {
        if profile[k + 1].x < x_min || x_max < profile[k].x {
            continue;
        }

        let a = profile[k];
        let b = profile[k + 1];
        let angle = (b.x - a.x).atan2(a.y - b.y);
        lower = min_float(lower, angle);
        upper = max_float(upper, angle);
    }

    let angle = normal.y.atan2(normal.x);
    angle < lower - tolerance || upper + tolerance < angle
}

/// A capsule lying across the slide direction crosses a 5.7 degree crest at speed, like the
/// crest scene in the ghost collision sample. The edges are flat, convex and concave, and a
/// capsule edge contact on the wrong side of a fold kicks the capsule. (CapsuleOverCrest)
fn capsule_over_crest(mirror: bool) {
    let profile = [
        Vec2 { x: -20.0, y: 0.0 },
        Vec2 { x: -10.0, y: 0.0 },
        Vec2 { x: 0.0, y: 1.0 },
        Vec2 { x: 10.0, y: 0.0 },
        Vec2 { x: 20.0, y: 0.0 },
    ];

    // Quarter meter cells with alternating diagonals, like cooked level geometry
    let cell_size = 0.25f32;
    let mut points = vec![profile[0]];
    for k in 0..profile.len() - 1 {
        let a = profile[k];
        let b = profile[k + 1];
        let length = ((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)).sqrt();
        let n = (length / cell_size - 0.01).ceil() as i32;
        for s in 1..=n {
            assert!(points.len() < 256);
            let t = s as f32 / n as f32;
            points.push(Vec2 {
                x: a.x + t * (b.x - a.x),
                y: a.y + t * (b.y - a.y),
            });
        }
    }

    // Mirror the data and flip it back with the mesh scale, so the world geometry is the same
    let half_width = 3.0;
    let z_count = 24;
    let stride = z_count + 1;
    let sign = if mirror { -1.0 } else { 1.0 };

    let point_count = points.len() as i32;
    let mut vertices = Vec::with_capacity((point_count * stride) as usize);
    for point in &points {
        for j in 0..=z_count {
            let z = -half_width + 2.0 * half_width * j as f32 / z_count as f32;
            vertices.push(v(sign * point.x, point.y, z));
        }
    }

    let mut indices = Vec::new();
    for i in 0..point_count - 1 {
        for j in 0..z_count {
            let a = i * stride + j;
            let b = a + 1;
            let c = a + stride + 1;
            let d = a + stride;

            let mut corners = [a, b, c, c, d, a];
            if ((i + j) & 1) != 0 {
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

    let def = MeshDef {
        vertices,
        indices,
        identify_edges: true,
        ..Default::default()
    };
    let mesh = create_mesh(&def, None).expect("crest mesh");

    let mut world = World::new(&default_world_def());
    let scale = if mirror { v(-1.0, 1.0, 1.0) } else { VEC3_ONE };
    create_mesh_ground(&mut world, &mesh, scale);

    let radius = 0.2;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(-16.0, radius + 0.01, 0.0);
    body_def.linear_velocity = v(6.0, 0.0, 0.0);
    body_def.enable_sleep = false;
    let capsule_id = create_body(&mut world, &body_def);

    let capsule = Capsule {
        center1: v(0.0, 0.0, -0.4),
        center2: v(0.0, 0.0, 0.4),
        radius,
    };
    let mut shape_def = default_shape_def();
    shape_def.base_material.friction = 0.0;
    let capsule_shape_id = create_capsule_shape(&mut world, capsule_id, &shape_def, &capsule);

    let mut ghost_count = 0;
    let mut step = 0;
    while step < 600 && (body_get_position(&world, capsule_id).x as f32) < 15.0 {
        world.step(1.0 / 60.0, 4);

        let bounds = shape_get_aabb(&world, capsule_shape_id);
        let x_min = bounds.lower_bound.x - 0.1;
        let x_max = bounds.upper_bound.x + 0.1;

        let contacts = body_get_contact_data(&world, capsule_id, 4);
        for contact in &contacts {
            let capsule_is_a = contact.shape_id_a == capsule_shape_id;
            for manifold in &contact.manifolds {
                // Speculative points that never pushed are harmless
                let mut impulse = 0.0f32;
                for point in &manifold.points[..manifold.point_count as usize] {
                    impulse += point.total_normal_impulse;
                }

                let normal = if capsule_is_a {
                    neg(manifold.normal)
                } else {
                    manifold.normal
                };
                if impulse > 0.0 && is_profile_ghost(&profile, normal, x_min, x_max) {
                    ghost_count += 1;
                }
            }
        }

        step += 1;
    }

    assert_eq!(ghost_count, 0);
    assert!(body_get_position(&world, capsule_id).x as f32 >= 15.0);
}

/// (MeshCapsuleOverCrest)
#[test]
fn mesh_capsule_over_crest() {
    capsule_over_crest(false);
    capsule_over_crest(true);
}

/// Fires a sphere or the end of a capsule at the apex along the given direction, without
/// gravity, and returns the deepest separation from the apex. (ApexImpact)
fn apex_impact(mesh: &MeshData, direction: Vec3, use_capsule: bool) -> f32 {
    let mut world_def = default_world_def();
    world_def.gravity = VEC3_ZERO;
    let mut world = World::new(&world_def);

    create_mesh_ground(&mut world, mesh, VEC3_ONE);

    let radius = 0.25;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = to_pos(mul_sv(radius + 0.1, direction));
    body_def.linear_velocity = mul_sv(-2.0, direction);
    let body_id = create_body(&mut world, &body_def);

    // The capsule points away from the apex, so its lower end is the closest point
    let tip = mul_sv(0.5, direction);
    let shape_def = default_shape_def();
    if use_capsule {
        let capsule = Capsule {
            center1: VEC3_ZERO,
            center2: tip,
            radius,
        };
        create_capsule_shape(&mut world, body_id, &shape_def, &capsule);
    } else {
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius,
        };
        create_sphere_shape(&mut world, body_id, &shape_def, &sphere);
    }

    let mut min_separation = f32::MAX;
    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);

        let p1 = body_get_world_point(&world, body_id, VEC3_ZERO);
        let p2 = body_get_world_point(&world, body_id, if use_capsule { tip } else { VEC3_ZERO });
        let a = to_vec3(p1);
        let segment = sub(to_vec3(p2), a);
        let length_squared = length_squared(segment);
        let t = if length_squared > 0.0 {
            clamp_float(-dot(a, segment) / length_squared, 0.0, 1.0)
        } else {
            0.0
        };
        let distance = length(mul_add(a, t, segment));
        min_separation = min_float(min_separation, distance - radius);
    }

    min_separation
}

/// A vertex manifold that fails a neighbor test is dropped. That relies on some triangle around
/// a convex vertex admitting every normal in the vertex normal cone. On this irregular valence
/// 6 apex each direction is inside the cone and exactly one of the six triangles admits it.
/// (MeshApexContact)
#[test]
fn mesh_apex_contact() {
    let vertices = vec![
        v(0.0, 0.0, 0.0),
        v(1.08, -1.9, -0.45),
        v(-1.1, -0.89, -0.35),
        v(-1.73, -1.91, -0.06),
        v(-0.8, -1.45, 0.27),
        v(0.35, -1.63, 0.35),
        v(0.79, -1.96, 0.07),
    ];

    let indices = vec![0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 1];

    let def = MeshDef {
        vertices,
        indices,
        identify_edges: true,
        ..Default::default()
    };
    let mesh = create_mesh(&def, None).expect("apex mesh");

    let directions = [v(-0.5703, 0.7369, 0.3629), v(0.8019, 0.4956, 0.3337)];

    // Dropping every vertex manifold here lets the body sink a full step before the deep
    // overlap fallback catches it
    for direction in directions {
        let direction = normalize(direction);
        assert!(apex_impact(&mesh, direction, false) > -linear_slop());
        assert!(apex_impact(&mesh, direction, true) > -linear_slop());
    }
}

/// Runs a Mesh / Ghost Collisions scene with the cylinder and returns the ghost count and the
/// run count. (CountCylinderGhosts)
fn count_cylinder_ghosts(scene: GhostScene, peel_preset: usize, mirror: bool) -> (i32, i32) {
    let mut settings = get_default_ghost_settings();
    settings.scene = scene;
    settings.peel_preset = peel_preset;
    settings.body = GhostBody::Cylinder;
    settings.mirror = mirror;

    let mut world = World::new(&default_world_def());
    let mut data = create_ghost_collisions(&mut world, settings);

    for _ in 0..900 {
        world.step(1.0 / 60.0, 4);
        update_ghost_collisions(&mut world, &mut data);
    }

    (data.ghost_count, data.run_count)
}

/// The PEEL cylinder slides along its axis on a polygon edge that runs along an internal edge
/// of the strip. Deep edge contacts on the far side fail the neighbor test and their face snap
/// is empty, so the triangles under the cylinder own the contact. (MeshPeelCylinder)
#[test]
fn mesh_peel_cylinder() {
    let (ghost_count, run_count) = count_cylinder_ghosts(GhostScene::Internal, 2, false);
    assert_eq!(ghost_count, 0);
    assert!(run_count >= 1);
}

/// Separated poses have an exact reference, the union of the triangles, see ghost_oracle.rs.
/// Without ghost handling the oracle has to find ghosts in every scene, which shows it has
/// teeth.
///
/// With ghost handling a pose fails as a ghost when a manifold carries a normal the union lacks
/// at its deep points, and as a miss when the union closest pair goes unrepresented with ghost
/// handling but not without it. Both are rare rather than absent, so each has a budget holding
/// the current count. The counts belong to the first ORACLE_SCAN_POSES poses, so a change to a
/// scene or to the pose stream moves them without the collider changing. Lower a budget when a
/// case is fixed.
///
/// The remaining ghost, wave pose 291, is a hull face normal admitted by a vertex cone with the
/// deepest point half a linear slop from the vertex, where the oracle only accepts the normal
/// at the vertex. (MeshGhostOracle)
#[test]
fn mesh_ghost_oracle() {
    let ghost_budget: [i32; ORACLE_SCENE_COUNT] = [0, 0, 0, 0, 0, 0, 0, 1];
    let missed_budget: [i32; ORACLE_SCENE_COUNT] = [0, 0, 0, 0, 0, 0, 0, 0];

    for (scene_index, &scene) in ORACLE_SCENES.iter().enumerate() {
        let mesh = create_oracle_mesh(scene).expect("oracle mesh");

        let scan = scan_ghost_oracle(&mesh, scene, ORACLE_SCAN_POSES);

        if scan.valid_count <= ORACLE_SCAN_POSES / 2
            || scan.raw_ghost_poses == 0
            || scan.ghost_poses != ghost_budget[scene_index]
            || scan.missed_poses != missed_budget[scene_index]
        {
            println!(
                "  oracle {}: valid {}, unhandled ghosts {}, ghosts {} of {} (first pose {}), \
                 missed {} of {} (first pose {})",
                ORACLE_SCENE_NAMES[scene_index],
                scan.valid_count,
                scan.raw_ghost_poses,
                scan.ghost_poses,
                ghost_budget[scene_index],
                scan.first_ghost,
                scan.missed_poses,
                missed_budget[scene_index],
                scan.first_missed
            );
        }

        assert!(scan.valid_count > ORACLE_SCAN_POSES / 2);
        assert!(scan.raw_ghost_poses > 0);
        assert!(scan.ghost_poses <= ghost_budget[scene_index]);
        assert!(scan.missed_poses <= missed_budget[scene_index]);
    }
}
