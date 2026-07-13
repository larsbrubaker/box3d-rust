//! Shared Compound / Village scene builder (C `sample_compound.cpp` Village).
//!
//! Fixed grid at the C debug value 8 (C release uses 200, which is far too heavy
//! for serial wasm) plus the real `building.obj` compound meshes. Physics matches
//! C layout; RNG is demo-local (not the C `Random*` stream).
//!
//! Also hosts the Compound Village character mover + sweeping query visualization
//! (see [`VillageMover`]).

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::mover_shared::{self, MoverBody, MoverDraw, MoverParams, JUMP_SPEED};
use crate::obj_loader::load_building_mesh;
use box3d_rust::body::create_body;
use box3d_rust::compound::{
    create_compound, CompoundCapsuleDef, CompoundDef, CompoundHullDef, CompoundMeshDef,
    CompoundSphereDef,
};
use box3d_rust::geometry::{default_surface_material, Capsule, Sphere, SurfaceMaterial};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    get_length_and_normalize, make_quat_from_axis_angle, mul_sv, offset_pos, sub_pos, Pos,
    Transform, Vec3, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::shape::create_compound_shape;
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def, BodyType};
use box3d_rust::world::{world_cast_ray_closest, world_cast_shape, world_overlap_shape, World};

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
/// `grid` is the C `gridCount` (C uses 8 debug / 200 release). The sole caller,
/// `sim_reset_village` (`sim_compound.rs:516`), passes the fixed C debug value 8 —
/// the 200-wide release grid is far too heavy for the serial wasm build.
pub fn build_village(world: &mut World, grid: i32) -> VillageScene {
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

// ---------------------------------------------------------------------------
// Village character mover (C `sample_compound.cpp` Village embeds a CharacterMover)
// ---------------------------------------------------------------------------
//
// This is the same kinematic `CharacterMover` the Character page uses: both the
// `VillageMover` here and `character_demo`'s `MoverController` delegate the whole
// integration (friction, accelerate, pogo spring, plane slide, dynamic-body push,
// clip) to the one shared port in [`crate::mover_shared`]. The Village mover only
// differs in its inputs: it collides with everything (default query filters), has
// no shapes to ignore, and always clips its velocity.

/// Compound Village character mover + the C sweeping query visualization.
pub struct VillageMover {
    pub mover_pos: Pos,
    pub velocity: Vec3,
    pub capsule: Capsule,
    pogo_velocity: f32,
    on_ground: bool,
    sprint: bool,
    throttle_x: f32,
    throttle_y: f32,
    jump: bool,
    want_sprint: bool,
    forward: Vec3,
    right: Vec3,
    pub third_person: bool,
    /// Sweeping query origin (C `m_rayOrigin`).
    ray_origin: Pos,
    /// C `m_worldWidth = 2 * gridCount * a`.
    world_width: f32,
    /// Last computed query visualization (see [`VillageMover::query`]).
    query_viz: [f32; VILLAGE_QUERY_LEN],
}

/// Flat layout length of [`VillageMover::query_viz`]. See `sim_village_query`.
pub const VILLAGE_QUERY_LEN: usize = 39;

impl VillageMover {
    /// Initialize at `start` for a world of half-width `world_width / 2`.
    pub fn new(start: Pos, world_width: f32) -> Self {
        let ray_origin = Pos {
            x: (-0.45 * world_width) as _,
            y: 20.0 as _,
            z: (-0.45 * world_width) as _,
        };
        Self {
            mover_pos: start,
            velocity: VEC3_ZERO,
            capsule: mover_shared::mover_capsule(),
            pogo_velocity: 0.0,
            on_ground: false,
            sprint: false,
            throttle_x: 0.0,
            throttle_y: 0.0,
            jump: false,
            want_sprint: false,
            forward: Vec3 {
                x: 0.0,
                y: 0.0,
                z: -1.0,
            },
            right: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            third_person: false,
            ray_origin,
            world_width,
            query_viz: [0.0; VILLAGE_QUERY_LEN],
        }
    }

    /// Feed WASD throttle, jump edge, sprint, and camera-relative axes (XZ).
    #[allow(clippy::too_many_arguments)]
    pub fn set_input(
        &mut self,
        throttle_x: f32,
        throttle_y: f32,
        jump: bool,
        sprint: bool,
        fwd_x: f32,
        fwd_z: f32,
        right_x: f32,
        right_z: f32,
    ) {
        self.throttle_x = throttle_x;
        self.throttle_y = throttle_y;
        if jump {
            self.jump = true;
        }
        self.want_sprint = sprint;

        let mut len = 0.0;
        let mut fwd = get_length_and_normalize(
            &mut len,
            Vec3 {
                x: fwd_x,
                y: 0.0,
                z: fwd_z,
            },
        );
        if len < 1e-4 {
            fwd = Vec3 {
                x: 0.0,
                y: 0.0,
                z: -1.0,
            };
        }
        let mut right = get_length_and_normalize(
            &mut len,
            Vec3 {
                x: right_x,
                y: 0.0,
                z: right_z,
            },
        );
        if len < 1e-4 {
            right = Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            };
        }
        self.forward = fwd;
        self.right = right;
    }

    pub fn toggle_third_person(&mut self) {
        self.third_person = !self.third_person;
    }

    /// One mover integration step against `world` (does not step the world).
    /// Delegates to the shared [`crate::mover_shared::solve_move`] (C
    /// `CharacterMover::SolveMove`), so the Village mover picks up the friction,
    /// dynamic-body push loop, and clip behavior identical to the Character page.
    pub fn solve_move(&mut self, world: &mut World, time_step: f32) {
        if time_step <= 0.0 {
            return;
        }

        // Jump / sprint from the previous frame's ground state, matching C's `Step`
        // order (applied before SolveMove). A queued jump persists until grounded.
        if self.jump && self.on_ground {
            self.velocity.y = JUMP_SPEED;
            self.on_ground = false;
            self.jump = false;
        }
        self.sprint = self.on_ground && self.want_sprint;

        let mut body = MoverBody {
            position: self.mover_pos,
            velocity: self.velocity,
            capsule: self.capsule,
            pogo_velocity: self.pogo_velocity,
            on_ground: self.on_ground,
            sprint: self.sprint,
        };
        // The Village mover collides with everything, ignores nothing, and always
        // clips its velocity (C Village uses the default CharacterMover behavior).
        let params = MoverParams {
            forward: self.forward,
            right: self.right,
            throttle_x: self.throttle_x,
            throttle_y: self.throttle_y,
            clip_velocity: true,
            ignore_shapes: &[],
            pogo_filter: default_query_filter(),
            mover_filter: default_query_filter(),
            cast_filter: default_query_filter(),
        };
        let mut draw = MoverDraw::default();
        mover_shared::solve_move(world, &mut body, &params, &mut draw, time_step);

        self.mover_pos = body.position;
        self.velocity = body.velocity;
        self.pogo_velocity = body.pogo_velocity;
        self.on_ground = body.on_ground;
        self.sprint = body.sprint;
    }

    /// C Village::Step query sweep (:711-779): a moving ray cast, a sphere shape
    /// cast, and an overlap-shape test, then advance the sweep origin. `time_step`
    /// is 0 when paused (the sweep freezes, matching C). Fills [`query_viz`].
    pub fn query(&mut self, world: &World, time_step: f32) {
        let translation = Vec3 {
            x: 10.0,
            y: -40.0,
            z: -5.0,
        };
        let filter = default_query_filter();
        let mut viz = [0.0f32; VILLAGE_QUERY_LEN];

        let ray_origin = self.ray_origin;
        let ray_end = offset_pos(ray_origin, translation);
        viz[0] = ray_origin.x as f32;
        viz[1] = ray_origin.y as f32;
        viz[2] = ray_origin.z as f32;
        viz[3] = ray_end.x as f32;
        viz[4] = ray_end.y as f32;
        viz[5] = ray_end.z as f32;

        // Ray cast (closest).
        let ray = world_cast_ray_closest(world, ray_origin, translation, &filter);
        if ray.hit {
            viz[6] = 1.0;
            viz[7] = ray.point.x as f32;
            viz[8] = ray.point.y as f32;
            viz[9] = ray.point.z as f32;
            viz[10] = ray.normal.x;
            viz[11] = ray.normal.y;
            viz[12] = ray.normal.z;
            viz[13] = ray.triangle_index as f32;
            viz[14] = ray.child_index as f32;
            viz[15] = ray.user_material_id as f32;
        }

        // Sphere shape cast (closest), origin = m_rayOrigin - {1,0,1}.
        let shape_origin = sub_pos(
            ray_origin,
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 1.0,
            },
        );
        let shape_end = offset_pos(shape_origin, translation);
        viz[16] = shape_origin.x as f32;
        viz[17] = shape_origin.y as f32;
        viz[18] = shape_origin.z as f32;
        viz[19] = shape_end.x as f32;
        viz[20] = shape_end.y as f32;
        viz[21] = shape_end.z as f32;

        let mut proxy = box3d_rust::distance::ShapeProxy::default();
        proxy.count = 1;
        proxy.radius = 0.25;
        proxy.points[0] = VEC3_ZERO;

        let mut best_fraction = f32::MAX;
        let mut best: Option<(Pos, Vec3, i32, i32, u64, f32)> = None;
        world_cast_shape(
            world,
            shape_origin,
            &proxy,
            translation,
            &filter,
            |_id, point, normal, fraction, mid, tri, child| {
                if fraction < best_fraction {
                    best_fraction = fraction;
                    best = Some((point, normal, tri, child, mid, fraction));
                }
                fraction
            },
        );
        if let Some((point, normal, tri, child, mid, fraction)) = best {
            let sphere_pos = offset_pos(shape_origin, mul_sv(fraction, translation));
            viz[22] = 1.0;
            viz[23] = sphere_pos.x as f32;
            viz[24] = sphere_pos.y as f32;
            viz[25] = sphere_pos.z as f32;
            viz[26] = point.x as f32;
            viz[27] = point.y as f32;
            viz[28] = point.z as f32;
            viz[29] = normal.x;
            viz[30] = normal.y;
            viz[31] = normal.z;
            viz[32] = tri as f32;
            viz[33] = child as f32;
            viz[34] = mid as f32;
        }

        // Overlap shape (radius 0.3) at {rayOrigin.x-1, 2, rayOrigin.z-1}.
        let overlap_origin = Pos {
            x: (ray_origin.x as f32 - 1.0) as _,
            y: 2.0 as _,
            z: (ray_origin.z as f32 - 1.0) as _,
        };
        let mut oproxy = box3d_rust::distance::ShapeProxy::default();
        oproxy.count = 1;
        oproxy.radius = 0.3;
        oproxy.points[0] = VEC3_ZERO;
        let mut overlap = false;
        world_overlap_shape(world, overlap_origin, &oproxy, &filter, |_id| {
            overlap = true;
            false
        });
        viz[35] = overlap_origin.x as f32;
        viz[36] = overlap_origin.y as f32;
        viz[37] = overlap_origin.z as f32;
        viz[38] = if overlap { 1.0 } else { 0.0 };

        self.query_viz = viz;

        // Advance the sweep origin (C :770-787).
        let half = 0.45 * self.world_width;
        if (self.ray_origin.x as f32) > half {
            self.ray_origin.x = (-half) as _;
            self.ray_origin.z = (self.ray_origin.z as f32 + 8.0) as _;
        }
        if (self.ray_origin.z as f32) > half {
            self.ray_origin.z = (-half) as _;
        }
        self.ray_origin.x = (self.ray_origin.x as f32 + 2.0 * time_step) as _;
    }

    pub fn query_viz(&self) -> [f32; VILLAGE_QUERY_LEN] {
        self.query_viz
    }
}
