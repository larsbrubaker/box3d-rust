//! Compound serialization roundtrip tests.

use super::v;
use crate::compound::{
    compute_compound_aabb, create_compound, destroy_compound, ray_cast_compound, validate_compound,
    CompoundCapsuleDef, CompoundData, CompoundDef, CompoundHullDef, CompoundMeshDef,
    CompoundSphereDef,
};
use crate::geometry::{default_surface_material, Capsule, RayCastInput, Sphere};
use crate::hull::make_box_hull;
use crate::math_functions::{Transform, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO};
use crate::mesh::{create_box_mesh, create_grid_mesh, destroy_mesh, MeshData};

fn build_serializable_compound(md: &MeshData) -> CompoundData {
    let mat = default_surface_material();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let cap = CompoundCapsuleDef {
        capsule: Capsule {
            center1: v(-2.0, 0.0, 0.0),
            center2: v(-1.0, 0.0, 0.0),
            radius: 0.2,
        },
        material: mat,
    };
    let hull = CompoundHullDef {
        hull: &box_hull.base,
        transform: Transform {
            p: v(5.0, 0.0, 0.0),
            q: QUAT_IDENTITY,
        },
        material: mat,
    };
    let mesh = CompoundMeshDef {
        mesh_data: md,
        transform: Transform {
            p: v(0.0, 0.0, 5.0),
            q: QUAT_IDENTITY,
        },
        scale: v(1.0, 1.0, 1.0),
        materials: &[mat],
    };
    let sph = CompoundSphereDef {
        sphere: Sphere {
            center: v(-5.0, 0.0, 0.0),
            radius: 0.5,
        },
        material: mat,
    };
    create_compound(&CompoundDef {
        capsules: &[cap],
        hulls: &[hull],
        meshes: &[mesh],
        spheres: &[sph],
    })
    .unwrap()
}

#[test]
fn compound_serialize_roundtrip() {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let a = build_serializable_compound(&md);
    let aabb_a = compute_compound_aabb(&a, TRANSFORM_IDENTITY);
    let ray_input = RayCastInput {
        origin: VEC3_ZERO,
        translation: v(20.0, 0.0, 0.0),
        max_fraction: 1.0,
    };
    let ray_a = ray_cast_compound(&a, &ray_input);
    assert!(ray_a.hit);
    let byte_count = a.byte_count;
    let buffer = a.to_bytes();
    let reference = buffer.clone();
    destroy_compound(a);
    let b = validate_compound(&buffer).expect("deserialize");
    assert_eq!(b.byte_count, byte_count);
    assert_eq!(
        (b.capsule_count, b.hull_count, b.mesh_count, b.sphere_count),
        (1, 1, 1, 1)
    );

    // Validation must not touch the bytes. C also checks the validated compound is the input
    // address; the port parses an owned compound, so it checks the compound writes back the
    // same bytes instead.
    assert_eq!(buffer, reference);
    assert_eq!(b.to_bytes(), buffer);
    let aabb_b = compute_compound_aabb(&b, TRANSFORM_IDENTITY);
    assert!((aabb_a.lower_bound.x - aabb_b.lower_bound.x).abs() < 1e-5);
    assert!((aabb_a.upper_bound.z - aabb_b.upper_bound.z).abs() < 1e-5);
    let ray_b = ray_cast_compound(&b, &ray_input);
    assert_eq!(ray_b.hit, ray_a.hit);
    assert!((ray_b.fraction - ray_a.fraction).abs() < 1e-5);
    assert_eq!(ray_b.child_index, ray_a.child_index);
    destroy_mesh(md);
}

/// The mesh blob nested inside a compound is a byte-for-byte copy of the standalone mesh
/// blob (compound.c line 606 memcpy's `meshData->byteCount` bytes), so its material index
/// section holds one `uint8_t` per triangle (mesh.c line 1690), not one per distinct
/// material. A multi-material mesh child must survive the round trip intact.
#[test]
fn compound_serialize_multi_material_mesh_child() {
    let md = create_grid_mesh(4, 4, 1.0, 2, false).expect("grid mesh");
    assert_eq!(md.material_count, 2);
    assert!(md.triangle_count > md.material_count);
    assert_eq!(md.material_indices.len(), md.triangle_count as usize);

    let mat = default_surface_material();
    let mut mat2 = default_surface_material();
    mat2.friction = 0.25;
    let mesh = CompoundMeshDef {
        mesh_data: &md,
        transform: TRANSFORM_IDENTITY,
        scale: v(1.0, 1.0, 1.0),
        materials: &[mat, mat2],
    };
    let a = create_compound(&CompoundDef {
        capsules: &[],
        hulls: &[],
        meshes: &[mesh],
        spheres: &[],
    })
    .expect("compound");
    let buffer = a.to_bytes();
    destroy_compound(a);
    let b = validate_compound(&buffer).expect("deserialize");
    assert_eq!(b.shared_meshes.len(), 1);
    let restored = &b.shared_meshes[0];
    assert_eq!(
        restored.material_indices.len(),
        md.material_indices.len(),
        "nested mesh material indices must be one per triangle"
    );
    assert_eq!(restored.material_indices, md.material_indices);
    assert_eq!(restored.flags.len(), md.flags.len());
    assert_eq!(restored.flags, md.flags);
    destroy_mesh(md);
}

/// End-to-end version of the above: a sphere resting on a deserialized compound whose mesh
/// child has more triangles than materials. `b3ComputeMeshManifolds` looks the child's
/// material up by triangle index (mesh_contact.rs, mirroring mesh_contact.c line 1131), so a
/// material index array sized by the distinct material count panics on the first contact.
#[test]
fn compound_deserialized_mesh_child_material_lookup() {
    use crate::body::{body_get_position, create_body};
    use crate::shape::{create_baked_compound_shape, create_sphere_shape};
    use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
    use crate::world::World;

    let md = create_grid_mesh(8, 8, 1.0, 2, false).expect("grid mesh");
    let mat = default_surface_material();
    let mut mat2 = default_surface_material();
    mat2.friction = 0.9;
    let a = create_compound(&CompoundDef {
        meshes: &[CompoundMeshDef {
            mesh_data: &md,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &[mat, mat2],
        }],
        ..Default::default()
    })
    .expect("compound");
    let buffer = a.to_bytes();
    destroy_compound(a);
    let restored = validate_compound(&buffer).expect("deserialize");

    let mut world = World::new(&default_world_def());
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground = create_body(&mut world, &ground_def);
    create_baked_compound_shape(&mut world, ground, &default_shape_def(), &restored);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = crate::math_functions::Pos {
        x: 0.5 as _,
        y: 1.0 as _,
        z: 0.5 as _,
    };
    let body = create_body(&mut world, &body_def);
    create_sphere_shape(
        &mut world,
        body,
        &default_shape_def(),
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.25,
        },
    );

    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }
    // The sphere lands on the mesh instead of falling through, and the per-triangle
    // material lookup ran for every contact along the way.
    let p = body_get_position(&world, body);
    assert!(
        p.y > 0.2,
        "sphere fell through the compound mesh: y={}",
        p.y
    );
    destroy_mesh(md);
}

#[test]
fn compound_serialize_bad_version() {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let a = build_serializable_compound(&md);
    let mut buffer = a.to_bytes();
    destroy_compound(a);
    buffer[0] ^= 1;
    assert!(validate_compound(&buffer).is_none());
    destroy_mesh(md);
}

#[test]
fn compound_serialize_wrong_byte_count() {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let a = build_serializable_compound(&md);
    let mut buffer = a.to_bytes();
    destroy_compound(a);
    buffer.push(0);
    assert!(validate_compound(&buffer).is_none());
    destroy_mesh(md);
}

/// `b3CompoundSphere` is a 16 byte `b3Sphere` plus two `uint16_t` (compound.c
/// `_Static_assert( sizeof( b3CompoundSphere ) == sizeof( b3Sphere ) + 2 * sizeof( uint16_t ) )`),
/// so the sphere array stride is 20 bytes, not the 32 byte capsule stride. Several spheres
/// must survive the round trip and the sphere section must be sized like C's.
#[test]
fn compound_serialize_multiple_spheres() {
    let mat = default_surface_material();
    let mut mat2 = default_surface_material();
    mat2.friction = 0.25;
    let spheres = [
        CompoundSphereDef {
            sphere: Sphere {
                center: v(-5.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: v(0.0, 1.0, 2.0),
                radius: 0.75,
            },
            material: mat2,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: v(3.0, -1.0, 4.0),
                radius: 1.25,
            },
            material: mat,
        },
    ];
    let a = create_compound(&CompoundDef {
        spheres: &spheres,
        ..Default::default()
    })
    .expect("compound");
    // The sphere array is the last section: alignUp8( 3 * 20 ) = 64 bytes.
    assert_eq!(a.byte_count - a.sphere_offset, 64);
    let buffer = a.to_bytes();
    let b = validate_compound(&buffer).expect("deserialize");
    assert_eq!(b.spheres, a.spheres);
    assert_eq!(b.to_bytes(), buffer);
    destroy_compound(a);
    destroy_compound(b);
}

/// A node count that runs the node array off the end of the buffer, including one whose byte
/// size overflows 32-bit arithmetic, is rejected. (CompoundSerializeBadNodeCount)
#[test]
fn compound_serialize_bad_node_count() {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let a = build_serializable_compound(&md);
    let byte_count = a.byte_count;
    let mut buffer = a.to_bytes();
    destroy_compound(a);

    // b3CompoundData::nodeCount sits after the version, byteCount, bounds, treeHeight, and
    // nodeOffset.
    const NODE_COUNT_OFFSET: usize = 44;
    let node_count = i32::from_le_bytes(
        buffer[NODE_COUNT_OFFSET..NODE_COUNT_OFFSET + 4]
            .try_into()
            .unwrap(),
    );
    let set_node_count = |buffer: &mut Vec<u8>, value: i32| {
        buffer[NODE_COUNT_OFFSET..NODE_COUNT_OFFSET + 4].copy_from_slice(&value.to_le_bytes());
    };

    // Node array runs off the end of the buffer
    set_node_count(&mut buffer, byte_count);
    assert!(validate_compound(&buffer).is_none());

    // Overflow in 32 bit arithmetic must still be caught
    set_node_count(&mut buffer, i32::MAX);
    assert!(validate_compound(&buffer).is_none());

    set_node_count(&mut buffer, node_count);
    assert!(validate_compound(&buffer).is_some());

    destroy_mesh(md);
}
