//! Minimal Wavefront OBJ → [`MeshData`] loader for demo assets.
//!
//! Mirrors `samples/mesh_loader.cpp` `CreateMeshData` for already-authored sample
//! meshes (triangulate n-gons via fan, cycle materials 0..2, weld + identify edges).
//!
//! Source assets: `box3d-cpp-reference/data/meshes/` (MIT, Erin Catto).

use box3d_rust::math_functions::Vec3;
use box3d_rust::mesh::{create_mesh, MeshData, MeshDef};

/// Raw parsed OBJ data (C `TempMesh`): the vertex/index/material arrays before the
/// BVH build. Kept separate from the build stage so a caller can parse once (C
/// `LoadTempMesh`) and rebuild the collision mesh many times with different BVH
/// options — the Creation Benchmark times exactly that inner rebuild.
pub(crate) struct TempMesh {
    pub vertices: Vec<Vec3>,
    pub indices: Vec<i32>,
    pub material_indices: Vec<u8>,
}

/// Parse a Wavefront OBJ buffer into a [`TempMesh`] (C `mesh_loader.cpp`
/// `LoadTempMesh`).
///
/// `scale` multiplies vertex positions; `z_up` swaps axes like the C loader
/// (`{y,z,x}`, applied after scaling — algebraically the same as scaling the
/// swapped components). Faces are fan-triangulated (C uses earcut; the sample
/// meshes are planar convex faces) and the per-triangle material index cycles
/// 0..2. Malformed vertex/face lines are skipped, matching the sample loader's
/// tolerance.
pub(crate) fn parse_obj(obj_text: &str, scale: f32, z_up: bool) -> TempMesh {
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
                let Some(x) = parts.next().and_then(|s| s.parse::<f32>().ok()) else {
                    continue;
                };
                let Some(y) = parts.next().and_then(|s| s.parse::<f32>().ok()) else {
                    continue;
                };
                let Some(z) = parts.next().and_then(|s| s.parse::<f32>().ok()) else {
                    continue;
                };
                let (x, y, z) = (scale * x, scale * y, scale * z);
                let v = if z_up {
                    Vec3 { x: y, y: z, z: x }
                } else {
                    Vec3 { x, y, z }
                };
                vertices.push(v);
            }
            "f" => {
                let mut face: Vec<i32> = Vec::new();
                for tok in parts {
                    // `v`, `v/vt`, `v/vt/vn`, or `v//vn`
                    let Some(v_str) = tok.split('/').next() else {
                        continue;
                    };
                    let Ok(mut vi) = v_str.parse::<i32>() else {
                        continue;
                    };
                    if vi < 0 {
                        vi = vertices.len() as i32 + vi + 1;
                    }
                    face.push(vi - 1);
                }
                if face.len() < 3 {
                    continue;
                }
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

    TempMesh {
        vertices,
        indices,
        material_indices,
    }
}

/// Build collision [`MeshData`] from a [`TempMesh`] with the given BVH controls.
pub(crate) fn build_mesh_from_temp(
    temp: &TempMesh,
    use_median_split: bool,
    identify_edges: bool,
    weld_vertices: bool,
    weld_tolerance: f32,
) -> Option<MeshData> {
    let def = MeshDef {
        vertices: temp.vertices.clone(),
        indices: temp.indices.clone(),
        material_indices: temp.material_indices.clone(),
        weld_tolerance,
        weld_vertices,
        use_median_split,
        identify_edges,
    };
    create_mesh(&def, None)
}

/// Parse a Wavefront OBJ buffer straight into collision [`MeshData`] (parse +
/// build in one step, C's `weldTolerance = 0.002`). The conveyor, voxel, and
/// building paths use this; the Viewer / Creation Benchmark parse once with
/// [`parse_obj`] and rebuild via [`build_mesh_from_temp`].
pub fn create_mesh_data_from_obj(
    obj_text: &str,
    scale: f32,
    z_up: bool,
    use_median_split: bool,
    identify_edges: bool,
    weld_vertices: bool,
) -> Option<MeshData> {
    let temp = parse_obj(obj_text, scale, z_up);
    if temp.vertices.len() < 3 || temp.indices.len() < 3 {
        return None;
    }
    build_mesh_from_temp(
        &temp,
        use_median_split,
        identify_edges,
        weld_vertices,
        0.002,
    )
}

/// Embedded Village building mesh (`data/meshes/building.obj`).
pub fn load_building_mesh() -> MeshData {
    const BUILDING_OBJ: &str = include_str!("../assets/building.obj");
    create_mesh_data_from_obj(BUILDING_OBJ, 1.0, false, false, true, true)
        .expect("building.obj → MeshData")
}
