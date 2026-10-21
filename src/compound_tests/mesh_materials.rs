//! Compound mesh children with more materials than the removed
//! `B3_MAX_COMPOUND_MESH_MATERIALS` limit (CompoundMeshManyMaterials,
//! CompoundMeshContactMaterial).

use super::{make_material, v};
use crate::compound::{
    create_compound, destroy_compound, get_compound_child, get_compound_materials,
    ray_cast_compound, CompoundCapsuleDef, CompoundDef, CompoundMeshDef,
};
use crate::geometry::{Capsule, RayCastInput, ShapeType, Sphere, SurfaceMaterial};
use crate::math_functions::{Vec3, TRANSFORM_IDENTITY, VEC3_ZERO};
use crate::mesh::{create_mesh, destroy_mesh, get_mesh_material_indices, MeshData, MeshDef};
use std::sync::atomic::{AtomicU64, Ordering};

/// Six separated upward facing triangles, one per material index 0 to 5.
/// (MakeSixMaterialMesh)
fn make_six_material_mesh() -> Option<MeshData> {
    let mut vertices = Vec::with_capacity(18);
    let mut indices = Vec::with_capacity(18);
    let mut material_indices = Vec::with_capacity(6);

    for i in 0..6 {
        let x = 3.0 * i as f32;
        vertices.push(v(x - 1.0, 0.0, -1.0));
        vertices.push(v(x, 0.0, 1.0));
        vertices.push(v(x + 1.0, 0.0, -1.0));

        indices.push(3 * i);
        indices.push(3 * i + 1);
        indices.push(3 * i + 2);

        material_indices.push(i as u8);
    }

    let def = MeshDef {
        vertices,
        stride: std::mem::size_of::<Vec3>(),
        indices,
        material_indices,
        ..Default::default()
    };

    create_mesh(&def, None)
}

/// (CompoundMeshManyMaterials)
#[test]
fn compound_mesh_many_materials() {
    // The capsule material bakes first so the mesh map is not the identity
    let capsule_mat = make_material(0.4, 1000);

    let md = make_six_material_mesh().unwrap();
    assert_eq!(md.material_count, 6);

    let mesh_mats: Vec<SurfaceMaterial> = (0..6)
        .map(|i| make_material(0.2 + 0.1 * i as f32, (i + 1) as u64))
        .collect();

    let cap = CompoundCapsuleDef {
        capsule: Capsule {
            center1: v(-10.0, 0.0, 0.0),
            center2: v(-9.0, 0.0, 0.0),
            radius: 0.25,
        },
        material: capsule_mat,
    };
    let mesh = CompoundMeshDef {
        mesh_data: &md,
        transform: TRANSFORM_IDENTITY,
        scale: v(1.0, 1.0, 1.0),
        materials: &mesh_mats,
    };

    let c = create_compound(&CompoundDef {
        capsules: &[cap],
        meshes: &[mesh],
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.material_count, 7);

    let table = get_compound_materials(&c);

    let cap_child = get_compound_child(&c, 0);
    assert_eq!(cap_child.shape_type, ShapeType::Capsule);
    assert_eq!(cap_child.material_count, 1);
    assert_eq!(
        table[cap_child.material_indices[0] as usize].user_material_id,
        1000
    );

    let mesh_child = get_compound_child(&c, 1);
    assert_eq!(mesh_child.shape_type, ShapeType::Mesh);
    assert_eq!(mesh_child.material_count, 6);
    for k in 0..6 {
        assert_eq!(
            table[mesh_child.material_indices[k] as usize].user_material_id,
            (k + 1) as u64
        );
    }

    // Sixth triangle, material index 5
    let triangle_materials = get_mesh_material_indices(&md);
    let ray = RayCastInput {
        origin: v(15.0, 5.0, 0.0),
        translation: v(0.0, -10.0, 0.0),
        max_fraction: 1.0,
    };
    let out = ray_cast_compound(&c, &ray);
    assert!(out.hit);
    assert_eq!(out.child_index, 1);
    let raw_index = triangle_materials[out.triangle_index as usize] as i32;
    assert!(raw_index >= 4);
    assert_eq!(
        table[out.material_index as usize].user_material_id,
        (raw_index + 1) as u64
    );

    destroy_compound(c);
    destroy_mesh(md);
}

/// Flat quad with every triangle on material index 5, so the mesh has six materials.
/// (MakeFlatMaterial5Mesh)
fn make_flat_material5_mesh() -> Option<MeshData> {
    let def = MeshDef {
        vertices: vec![
            v(-5.0, 0.0, -5.0),
            v(-5.0, 0.0, 5.0),
            v(5.0, 0.0, -5.0),
            v(5.0, 0.0, 5.0),
        ],
        stride: std::mem::size_of::<Vec3>(),
        indices: vec![0, 1, 3, 3, 2, 0],
        material_indices: vec![5, 5],
        ..Default::default()
    };

    create_mesh(&def, None)
}

/// (s_compoundContactMaterialId) The friction callback is a plain function pointer, so the
/// observed material id travels through a static like the C test.
static COMPOUND_CONTACT_MATERIAL_ID: AtomicU64 = AtomicU64::new(0);

/// (CompoundContactFrictionCallback)
fn compound_contact_friction_callback(
    friction_a: f32,
    user_material_id_a: u64,
    friction_b: f32,
    user_material_id_b: u64,
) -> f32 {
    if user_material_id_a == 6 || user_material_id_b == 6 {
        COMPOUND_CONTACT_MATERIAL_ID.store(6, Ordering::Relaxed);
    }
    0.5 * (friction_a + friction_b)
}

/// (CompoundMeshContactMaterial)
#[test]
fn compound_mesh_contact_material() {
    use crate::body::create_body;
    use crate::shape::{create_baked_compound_shape, create_sphere_shape, shape_get_compound};
    use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
    use crate::world::{world_set_friction_callback, World};

    COMPOUND_CONTACT_MATERIAL_ID.store(0, Ordering::Relaxed);

    let mut world = World::new(&default_world_def());
    world_set_friction_callback(&mut world, Some(compound_contact_friction_callback));

    let mut ground_body_def = default_body_def();
    ground_body_def.type_ = BodyType::Static;
    let ground_body_id = create_body(&mut world, &ground_body_def);

    let md = make_flat_material5_mesh().unwrap();
    assert_eq!(md.material_count, 6);

    let mesh_mats: Vec<SurfaceMaterial> =
        (0..6).map(|i| make_material(0.5, (i + 1) as u64)).collect();

    let mesh_child = CompoundMeshDef {
        mesh_data: &md,
        transform: TRANSFORM_IDENTITY,
        scale: v(1.0, 1.0, 1.0),
        materials: &mesh_mats,
    };
    let compound = create_compound(&CompoundDef {
        meshes: &[mesh_child],
        ..Default::default()
    })
    .unwrap();

    let ground_shape_def = default_shape_def();
    let compound_shape_id =
        create_baked_compound_shape(&mut world, ground_body_id, &ground_shape_def, &compound);
    // C checks the getter returns the baked compound pointer; the shape owns a copy here, so
    // compare the compound bytes.
    let shape_compound = shape_get_compound(&world, compound_shape_id).expect("compound shape");
    assert_eq!(shape_compound.to_bytes(), compound.to_bytes());

    let mut sphere_body_def = default_body_def();
    sphere_body_def.type_ = BodyType::Dynamic;
    sphere_body_def.position = crate::math_functions::Pos {
        x: 0.0 as _,
        y: 3.0 as _,
        z: 0.0 as _,
    };
    let sphere_body_id = create_body(&mut world, &sphere_body_def);
    let mut sphere_shape_def = default_shape_def();
    sphere_shape_def.density = 1.0;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    create_sphere_shape(&mut world, sphere_body_id, &sphere_shape_def, &sphere);

    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
    }

    assert_eq!(COMPOUND_CONTACT_MATERIAL_ID.load(Ordering::Relaxed), 6);

    destroy_compound(compound);
    destroy_mesh(md);
}
