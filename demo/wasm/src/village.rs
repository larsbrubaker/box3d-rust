//! Shared Compound / Village scene builder (C `sample_compound.cpp` Village).
//!
//! Browser-scaled grid (default 8..=40 vs C debug 8 / release 200) plus the real
//! `building.obj` compound meshes. Physics matches C layout; RNG is demo-local.

use crate::obj_loader::load_building_mesh;
use box3d_rust::body::create_body;
use box3d_rust::compound::{
    create_compound, CompoundCapsuleDef, CompoundDef, CompoundHullDef, CompoundMeshDef,
    CompoundSphereDef,
};
use box3d_rust::geometry::{default_surface_material, Capsule, Sphere, SurfaceMaterial};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, Pos, Transform, Vec3, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::shape::create_compound_shape;
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use box3d_rust::world::World;

/// Tiny LCG for village prop / building placement (demo-only; not C Random*).
pub struct DemoRng(pub u32);

impl DemoRng {
    fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
        self.0
    }

    fn next_f32(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }

    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.next_f32()
    }

    pub fn vec3_range(&mut self, lo: Vec3, hi: Vec3) -> Vec3 {
        Vec3 {
            x: self.range(lo.x, hi.x),
            y: self.range(lo.y, hi.y),
            z: self.range(lo.z, hi.z),
        }
    }
}

/// One building instance in compound-local space (for Three.js InstancedMesh).
#[derive(Clone, Copy)]
pub struct BuildingInstance {
    pub transform: Transform,
    pub scale: Vec3,
}

/// Compound / Village bake result for physics + visualization.
pub struct VillageScene {
    pub ground_body_index: i32,
    pub hull_transforms: Vec<Transform>,
    pub capsules: Vec<CompoundCapsuleDef>,
    pub spheres: Vec<CompoundSphereDef>,
    pub buildings: Vec<BuildingInstance>,
    pub tile_half: f32,
    /// `[capsules, hulls, meshes, spheres, byte_count, tree_bytes, tree_height]`
    pub stats: [f32; 7],
}

/// Build the Village compound ground + building meshes.
///
/// `grid` is clamped to 8..=40 (C uses 8 debug / 200 release).
pub fn build_village(world: &mut World, grid: i32) -> VillageScene {
    let grid = grid.clamp(8, 40);
    let a = 4.0f32;
    let mut rng = DemoRng(0xB111_A6E7);
    let material = default_surface_material();
    let box_hull = make_box_hull(a, 0.5 * a, a);

    let hull_count = (grid * grid) as usize;
    let prop_capacity = hull_count / 8 + 1;
    let mut capsules: Vec<CompoundCapsuleDef> = Vec::with_capacity(prop_capacity);
    let mut spheres: Vec<CompoundSphereDef> = Vec::with_capacity(prop_capacity);
    let mut hull_transforms: Vec<Transform> = Vec::with_capacity(hull_count);

    let mut transform = Transform {
        p: VEC3_ZERO,
        q: QUAT_IDENTITY,
    };

    for i in 0..grid {
        transform.p.x = (2.0 * i as f32 - grid as f32) * a;
        for j in 0..grid {
            transform.p.z = (2.0 * j as f32 - grid as f32) * a;
            transform.p.y = rng.range(-0.25, 0.125) * a;

            if (i & 1) != 0 && (j & 1) != 0 {
                let base = transform.p;
                let p1 = base
                    + rng.vec3_range(
                        Vec3 { x: -a, y: a, z: -a },
                        Vec3 {
                            x: a,
                            y: 2.0 * a,
                            z: a,
                        },
                    );
                let p2 = base
                    + rng.vec3_range(
                        Vec3 { x: -a, y: a, z: -a },
                        Vec3 {
                            x: a,
                            y: 2.0 * a,
                            z: a,
                        },
                    );
                let radius = rng.range(0.1, 0.5);
                if capsules.len() < spheres.len() {
                    if capsules.len() < prop_capacity {
                        capsules.push(CompoundCapsuleDef {
                            capsule: Capsule {
                                center1: p1,
                                center2: p2,
                                radius,
                            },
                            material,
                        });
                    }
                } else if spheres.len() < prop_capacity {
                    spheres.push(CompoundSphereDef {
                        sphere: Sphere { center: p1, radius },
                        material,
                    });
                }
            }

            hull_transforms.push(transform);
        }
    }

    let building_mesh = load_building_mesh();
    let material_count = building_mesh.material_count.max(1).min(4) as usize;
    let mut mesh_materials: Vec<SurfaceMaterial> = Vec::with_capacity(material_count);
    for i in 0..material_count {
        let mut mat = default_surface_material();
        if i == 0 {
            mat.friction = 0.0;
        } else if i == 1 {
            mat.restitution = 0.5;
        }
        mat.user_material_id = (i as u64) + 42;
        mesh_materials.push(mat);
    }

    let mesh_grid = grid / 4;
    let mesh_count = (mesh_grid * mesh_grid) as usize;
    let b = 4.0 * a;
    let mut buildings: Vec<BuildingInstance> = Vec::with_capacity(mesh_count);
    let mut mesh_defs: Vec<CompoundMeshDef<'_>> = Vec::with_capacity(mesh_count);

    transform = Transform {
        p: VEC3_ZERO,
        q: QUAT_IDENTITY,
    };
    let mut mesh_index = 0usize;
    for i in 0..mesh_grid {
        transform.p.x = (2.0 * i as f32 - mesh_grid as f32) * b + 0.5 * b;
        for j in 0..mesh_grid {
            transform.p.y = 0.5 * a;
            transform.p.z = (2.0 * j as f32 - mesh_grid as f32) * b + 0.5 * b;
            transform.q = make_quat_from_axis_angle(
                VEC3_AXIS_Y,
                rng.range(-std::f32::consts::PI, std::f32::consts::PI),
            );

            let mut scale = rng.vec3_range(
                Vec3 {
                    x: 0.5,
                    y: 0.5,
                    z: 0.5,
                },
                Vec3 {
                    x: 2.0,
                    y: 2.0,
                    z: 2.0,
                },
            );
            if (mesh_index & 1) != 0 {
                scale.x = -scale.x;
            }
            if (mesh_index & 3) != 0 {
                scale.z = -scale.z;
            }

            buildings.push(BuildingInstance { transform, scale });
            mesh_defs.push(CompoundMeshDef {
                mesh_data: &building_mesh,
                transform,
                scale,
                materials: &mesh_materials,
            });
            mesh_index += 1;
        }
    }

    let hulls: Vec<CompoundHullDef<'_>> = hull_transforms
        .iter()
        .map(|xf| CompoundHullDef {
            hull: &box_hull.base,
            transform: *xf,
            material,
        })
        .collect();

    let compound = create_compound(&CompoundDef {
        capsules: &capsules,
        hulls: &hulls,
        meshes: &mesh_defs,
        spheres: &spheres,
    })
    .expect("village compound");

    let stats = [
        compound.capsule_count as f32,
        compound.hull_count as f32,
        compound.mesh_count as f32,
        compound.sphere_count as f32,
        compound.byte_count as f32,
        compound.tree.byte_count() as f32,
        compound.tree.height() as f32,
    ];

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Static;
    body_def.position = Pos {
        x: (-1.0) as _,
        y: (-0.5) as _,
        z: 2.0 as _,
    };
    body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Y, -1.15 * std::f32::consts::PI);
    let ground = create_body(world, &body_def);
    create_compound_shape(world, ground, &default_shape_def(), &compound);

    VillageScene {
        ground_body_index: ground.index1 - 1,
        hull_transforms,
        capsules,
        spheres,
        buildings,
        tile_half: a,
        stats,
    }
}
