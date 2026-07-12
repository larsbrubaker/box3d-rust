//! Acceptance smoke tests for debug draw (`b3World_Draw`).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{body_set_name, create_body};
use crate::compound::{create_compound, CompoundDef, CompoundHullDef};
use crate::debug_draw::{DebugDraw, DebugShape, HexColor};
use crate::dynamic_tree::DEFAULT_MASK_BITS;
use crate::geometry::{default_surface_material, Capsule, ShapeType, Sphere};
use crate::height_field::create_grid;
use crate::hull::make_box_hull;
use crate::joint::{
    create_distance_joint, create_filter_joint, create_motor_joint, create_parallel_joint,
    create_prismatic_joint, create_revolute_joint, create_spherical_joint, create_weld_joint,
    create_wheel_joint,
};
use crate::math_functions::{
    to_pos, Aabb, Pos, Transform, Vec3, WorldTransform, QUAT_IDENTITY, VEC3_ONE, VEC3_ZERO,
};
use crate::mesh::create_box_mesh;
use crate::shape::{
    create_capsule_shape, create_compound_shape, create_height_field_shape, create_hull_shape,
    create_mesh_shape, create_sphere_shape,
};
use crate::types::{
    default_body_def, default_distance_joint_def, default_filter_joint_def,
    default_motor_joint_def, default_parallel_joint_def, default_prismatic_joint_def,
    default_revolute_joint_def, default_shape_def, default_spherical_joint_def,
    default_weld_joint_def, default_wheel_joint_def, default_world_def, BodyType,
};
use crate::world::{world_draw, World};

#[derive(Default)]
struct CountingDraw {
    shapes: usize,
    /// Indexed by `ShapeType as i32` (Capsule..Sphere).
    shape_types: [usize; 6],
    segments: usize,
    transforms: usize,
    points: usize,
    bounds: usize,
    boxes: usize,
    strings: Vec<String>,
    everything: bool,
    drawing_bounds: Option<Aabb>,
}

impl DebugDraw for CountingDraw {
    fn draw_shape(
        &mut self,
        user_shape: u64,
        _transform: WorldTransform,
        _color: HexColor,
    ) -> bool {
        self.shapes += 1;
        let type_tag = (user_shape >> 32) as i32;
        if (0..6).contains(&type_tag) {
            self.shape_types[type_tag as usize] += 1;
        }
        true
    }

    fn draw_segment(&mut self, _p1: Pos, _p2: Pos, _color: HexColor) {
        self.segments += 1;
    }

    fn draw_transform(&mut self, _transform: WorldTransform) {
        self.transforms += 1;
    }

    fn draw_point(&mut self, _p: Pos, _size: f32, _color: HexColor) {
        self.points += 1;
    }

    fn draw_bounds(&mut self, _aabb: Aabb, _color: HexColor) {
        self.bounds += 1;
    }

    fn draw_box(&mut self, _extents: Vec3, _transform: WorldTransform, _color: HexColor) {
        self.boxes += 1;
    }

    fn draw_string(&mut self, _p: Pos, s: &str, _color: HexColor) {
        self.strings.push(s.to_string());
    }

    fn drawing_bounds(&self) -> Aabb {
        self.drawing_bounds.unwrap_or(Aabb {
            lower_bound: Vec3 {
                x: -1.0e6,
                y: -1.0e6,
                z: -1.0e6,
            },
            upper_bound: Vec3 {
                x: 1.0e6,
                y: 1.0e6,
                z: 1.0e6,
            },
        })
    }

    fn draw_shapes(&self) -> bool {
        true
    }
    fn draw_joints(&self) -> bool {
        self.everything
    }
    fn draw_joint_extras(&self) -> bool {
        self.everything
    }
    fn draw_bounds_boxes(&self) -> bool {
        self.everything
    }
    fn draw_mass(&self) -> bool {
        self.everything
    }
    fn draw_sleep(&self) -> bool {
        self.everything
    }
    fn draw_body_names(&self) -> bool {
        self.everything
    }
    fn draw_contacts(&self) -> bool {
        self.everything
    }
    fn draw_graph_colors(&self) -> bool {
        self.everything
    }
    fn draw_contact_features(&self) -> bool {
        self.everything
    }
    fn draw_contact_normals(&self) -> bool {
        self.everything
    }
    fn draw_contact_forces(&self) -> bool {
        self.everything
    }
    fn draw_islands(&self) -> bool {
        self.everything
    }
}

fn create_debug_shape(debug: &DebugShape<'_>, _ctx: u64) -> u64 {
    ((debug.shape_type as i32 as u64) << 32) | (debug.shape_id.index1 as u64)
}

fn destroy_debug_shape(_user_shape: u64, _ctx: u64) {}

/// Ground + every shape type, with createDebugShape wired so DrawShape fires.
fn build_shape_scene() -> World {
    let mut def = default_world_def();
    def.create_debug_shape = Some(create_debug_shape);
    def.destroy_debug_shape = Some(destroy_debug_shape);
    let mut world = World::new(&def);

    // Static ground hull (named)
    {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Static;
        bd.position = to_pos(Vec3 {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        });
        let body = create_body(&mut world, &bd);
        body_set_name(&mut world, body, "ground");
        let box_hull = make_box_hull(5.0, 0.5, 5.0);
        create_hull_shape(&mut world, body, &default_shape_def(), &box_hull.base);
    }

    // Sphere
    {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = to_pos(Vec3 {
            x: 0.0,
            y: 2.0,
            z: 0.0,
        });
        let body = create_body(&mut world, &bd);
        create_sphere_shape(
            &mut world,
            body,
            &default_shape_def(),
            &Sphere {
                center: VEC3_ZERO,
                radius: 0.5,
            },
        );
    }

    // Capsule
    {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = to_pos(Vec3 {
            x: 2.0,
            y: 2.0,
            z: 0.0,
        });
        let body = create_body(&mut world, &bd);
        create_capsule_shape(
            &mut world,
            body,
            &default_shape_def(),
            &Capsule {
                center1: Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                },
                center2: Vec3 {
                    x: 0.0,
                    y: 0.5,
                    z: 0.0,
                },
                radius: 0.25,
            },
        );
    }

    // Dynamic hull
    {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = to_pos(Vec3 {
            x: -2.0,
            y: 2.0,
            z: 0.0,
        });
        let body = create_body(&mut world, &bd);
        let box_hull = make_box_hull(0.4, 0.4, 0.4);
        create_hull_shape(&mut world, body, &default_shape_def(), &box_hull.base);
    }

    // Mesh (static)
    {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Static;
        bd.position = to_pos(Vec3 {
            x: 4.0,
            y: 0.0,
            z: 0.0,
        });
        let body = create_body(&mut world, &bd);
        let mesh = create_box_mesh(VEC3_ZERO, Vec3::new(1.0, 0.1, 1.0), false).expect("mesh");
        create_mesh_shape(&mut world, body, &default_shape_def(), &mesh, VEC3_ONE);
    }

    // Height field (static)
    {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Static;
        bd.position = to_pos(Vec3 {
            x: -4.0,
            y: 0.0,
            z: 0.0,
        });
        let body = create_body(&mut world, &bd);
        let hf = create_grid(4, 4, Vec3::new(1.0, 0.2, 1.0), false);
        create_height_field_shape(&mut world, body, &default_shape_def(), &hf);
    }

    // Compound (static only)
    {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Static;
        bd.position = to_pos(Vec3 {
            x: 0.0,
            y: 4.0,
            z: 2.0,
        });
        let body = create_body(&mut world, &bd);
        let box_a = make_box_hull(0.3, 0.3, 0.3);
        let box_b = make_box_hull(0.3, 0.3, 0.3);
        let hulls = [
            CompoundHullDef {
                hull: &box_a.base,
                transform: Transform {
                    p: Vec3::new(-0.4, 0.0, 0.0),
                    q: QUAT_IDENTITY,
                },
                material: default_surface_material(),
            },
            CompoundHullDef {
                hull: &box_b.base,
                transform: Transform {
                    p: Vec3::new(0.4, 0.0, 0.0),
                    q: QUAT_IDENTITY,
                },
                material: default_surface_material(),
            },
        ];
        let compound = create_compound(&CompoundDef {
            hulls: &hulls,
            ..Default::default()
        })
        .expect("compound");
        create_compound_shape(&mut world, body, &default_shape_def(), &compound);
    }

    world
}

#[test]
fn draw_hits_every_shape_type() {
    let mut world = build_shape_scene();
    let mut draw = CountingDraw::default();

    world_draw(&mut world, &mut draw, DEFAULT_MASK_BITS);

    assert!(draw.shapes >= 6, "expected at least one DrawShape per type");
    assert!(draw.shape_types[ShapeType::Sphere as usize] >= 1, "sphere");
    assert!(
        draw.shape_types[ShapeType::Capsule as usize] >= 1,
        "capsule"
    );
    assert!(draw.shape_types[ShapeType::Hull as usize] >= 1, "hull");
    assert!(draw.shape_types[ShapeType::Mesh as usize] >= 1, "mesh");
    assert!(draw.shape_types[ShapeType::Height as usize] >= 1, "height");
    assert!(
        draw.shape_types[ShapeType::Compound as usize] >= 1,
        "compound"
    );

    // Second draw reuses user shapes (create not called again) and still draws.
    let first = draw.shapes;
    world_draw(&mut world, &mut draw, DEFAULT_MASK_BITS);
    assert_eq!(draw.shapes, first * 2);
}

#[test]
fn draw_with_all_options_does_not_panic() {
    let mut world = build_shape_scene();

    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = to_pos(Vec3 {
        x: 1.0,
        y: 3.0,
        z: 1.0,
    });
    let body_a = create_body(&mut world, &bd);
    create_sphere_shape(
        &mut world,
        body_a,
        &default_shape_def(),
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.3,
        },
    );

    bd.position = to_pos(Vec3 {
        x: 2.0,
        y: 3.0,
        z: 1.0,
    });
    let body_b = create_body(&mut world, &bd);
    create_sphere_shape(
        &mut world,
        body_b,
        &default_shape_def(),
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.3,
        },
    );

    let mut dj = default_distance_joint_def();
    dj.base.body_id_a = body_a;
    dj.base.body_id_b = body_b;
    create_distance_joint(&mut world, &dj);

    let mut fj = default_filter_joint_def();
    fj.base.body_id_a = body_a;
    fj.base.body_id_b = body_b;
    create_filter_joint(&mut world, &fj);

    let mut mj = default_motor_joint_def();
    mj.base.body_id_a = body_a;
    mj.base.body_id_b = body_b;
    create_motor_joint(&mut world, &mj);

    let mut pj = default_parallel_joint_def();
    pj.base.body_id_a = body_a;
    pj.base.body_id_b = body_b;
    create_parallel_joint(&mut world, &pj);

    let mut rj = default_revolute_joint_def();
    rj.base.body_id_a = body_a;
    rj.base.body_id_b = body_b;
    create_revolute_joint(&mut world, &rj);

    let mut sj = default_spherical_joint_def();
    sj.base.body_id_a = body_a;
    sj.base.body_id_b = body_b;
    create_spherical_joint(&mut world, &sj);

    let mut wj = default_weld_joint_def();
    wj.base.body_id_a = body_a;
    wj.base.body_id_b = body_b;
    create_weld_joint(&mut world, &wj);

    let mut prj = default_prismatic_joint_def();
    prj.base.body_id_a = body_a;
    prj.base.body_id_b = body_b;
    create_prismatic_joint(&mut world, &prj);

    let mut whj = default_wheel_joint_def();
    whj.base.body_id_a = body_a;
    whj.base.body_id_b = body_b;
    create_wheel_joint(&mut world, &whj);

    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }

    let mut draw = CountingDraw {
        everything: true,
        ..Default::default()
    };
    world_draw(&mut world, &mut draw, DEFAULT_MASK_BITS);

    assert!(draw.shapes > 0);
    assert!(draw.segments > 0, "joints should emit segments");
    assert!(draw.bounds > 0, "bounds/islands should emit AABBs");
    assert!(
        draw.strings.iter().any(|s| s.contains("ground")),
        "body name should be drawn"
    );
}

#[test]
fn drawing_bounds_cull_shapes() {
    let mut world = build_shape_scene();
    let mut draw = CountingDraw {
        drawing_bounds: Some(Aabb {
            lower_bound: Vec3 {
                x: 1000.0,
                y: 1000.0,
                z: 1000.0,
            },
            upper_bound: Vec3 {
                x: 1001.0,
                y: 1001.0,
                z: 1001.0,
            },
        }),
        ..Default::default()
    };
    world_draw(&mut world, &mut draw, DEFAULT_MASK_BITS);
    assert_eq!(draw.shapes, 0);
}
