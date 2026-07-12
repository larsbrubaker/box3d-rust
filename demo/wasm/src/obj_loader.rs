//! Minimal Wavefront OBJ → [`MeshData`] loader for demo assets.
//!
//! Mirrors `samples/mesh_loader.cpp` `CreateMeshData` for already-authored sample
//! meshes (triangulate n-gons via fan, cycle materials 0..2, weld + identify edges).
//!
//! Source assets: `box3d-cpp-reference/data/meshes/` (MIT, Erin Catto).

use box3d_rust::math_functions::Vec3;
use box3d_rust::mesh::{create_mesh, MeshData, MeshDef};

/// Parse a Wavefront OBJ buffer into collision [`MeshData`].
///
/// `scale` multiplies vertex positions. `z_up` swaps axes like the C loader
/// (`{y,z,x}`). Passes `identify_edges` / `weld_vertices` through to `create_mesh`
/// with C's `weldTolerance = 0.002`.
pub fn create_mesh_data_from_obj(
    obj_text: &str,
    scale: f32,
    z_up: bool,
    use_median_split: bool,
    identify_edges: bool,
    weld_vertices: bool,
) -> Option<MeshData> {
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut indices: Vec<i32> = Vec::new();
    let mut material_indices: Vec<u8> = Vec::new();
    let mut material_index: u8 = 0;

    for raw in obj_text.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut parts = line.split_whitespace();
        let Some(tag) = parts.next() else {
            continue;
        };
        match tag {
            "v" => {
                let x: f32 = parts.next()?.parse().ok()?;
                let y: f32 = parts.next()?.parse().ok()?;
                let z: f32 = parts.next()?.parse().ok()?;
                let (x, y, z) = if z_up {
                    (scale * y, scale * z, scale * x)
                } else {
                    (scale * x, scale * y, scale * z)
                };
                vertices.push(Vec3 { x, y, z });
            }
            "f" => {
                let mut face: Vec<i32> = Vec::new();
                for tok in parts {
                    // `v`, `v/vt`, `v/vt/vn`, or `v//vn`
                    let v_str = tok.split('/').next()?;
                    let mut vi: i32 = v_str.parse().ok()?;
                    if vi < 0 {
                        vi = vertices.len() as i32 + vi + 1;
                    }
                    face.push(vi - 1);
                }
                if face.len() < 3 {
                    continue;
                }
                // Fan triangulation (C uses earcut; sample meshes are planar convex faces).
                for i in 1..face.len() - 1 {
                    indices.push(face[0]);
                    indices.push(face[i]);
                    indices.push(face[i + 1]);
                    material_indices.push(material_index);
                    material_index = (material_index + 1) % 3;
                }
            }
            _ => {}
        }
    }

    if vertices.len() < 3 || indices.len() < 3 {
        return None;
    }

    let def = MeshDef {
        vertices,
        indices,
        material_indices,
        weld_tolerance: 0.002,
        weld_vertices,
        use_median_split,
        identify_edges,
    };
    create_mesh(&def, None)
}

/// Embedded Village building mesh (`data/meshes/building.obj`).
pub fn load_building_mesh() -> MeshData {
    const BUILDING_OBJ: &str = include_str!("../assets/building.obj");
    create_mesh_data_from_obj(BUILDING_OBJ, 1.0, false, false, true, true)
        .expect("building.obj → MeshData")
}
