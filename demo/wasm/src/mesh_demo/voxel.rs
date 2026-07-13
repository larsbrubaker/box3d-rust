//! Voxel scene (C `VoxelMesh`, :1381): the `collision_mesh_01.obj` terrain sits at
//! a 5000+ m large-world offset with a single dynamic hull resting on it. Poses,
//! picker rays, and both wireframes render in a base frame anchored at the origin.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{new_world, MeshScene, MeshState};
use crate::obj_loader::create_mesh_data_from_obj;
use crate::vis::mesh_triangle_edges;
use box3d_rust::body::create_body;
use box3d_rust::hull::{create_hull, HullData};
use box3d_rust::math_functions::{mul_sv, Pos, Quat, Vec3, VEC3_ONE};
use box3d_rust::shape::{create_hull_shape, create_mesh_shape};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};

/// The 16 hull points from the C sample, already remapped `{p.y, p.z, p.x}` and
/// scaled by 0.01 (C `VoxelMesh` ctor).
fn hull_points() -> [Vec3; 16] {
    let raw = [
        [-3.13548756, 3.81141949, 237.289047],
        [-16.2333279, -23.4977913, 235.486603],
        [-13.8834839, 6.20244455, 23.7760544],
        [14.0794125, 4.63170528, 24.9530792],
        [3.98322797, -16.4192238, 236.704071],
        [-23.3520412, -3.26714420, 236.071594],
        [13.4517860, -6.94963741, 24.4085312],
        [-5.24953651, 13.9316301, 24.5058060],
        [-4.65071201, -24.1484108, 235.974121],
        [-14.5111103, -5.37889385, 23.2315063],
        [6.33307076, 13.2810068, 24.9935150],
        [4.81784487, -14.6788225, 23.6787796],
        [-14.7180958, 4.46204281, 236.801331],
        [-23.9796677, -14.8484812, 235.527039],
        [4.61085415, -4.83788204, 237.248611],
        [-6.76476669, -14.0281992, 23.1910706],
    ];
    let mut points = [Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }; 16];
    for (i, p) in raw.iter().enumerate() {
        points[i] = mul_sv(
            0.01,
            Vec3 {
                x: p[1],
                y: p[2],
                z: p[0],
            },
        );
    }
    points
}

/// Deduped undirected hull edges as flat local endpoints `[ax,ay,az,bx,by,bz, ...]`.
fn hull_wireframe_local(hull: &HullData) -> Vec<f32> {
    let mut out = Vec::new();
    for e in &hull.edges {
        let a = e.origin as usize;
        let twin = &hull.edges[e.twin as usize];
        let b = twin.origin as usize;
        if a >= b {
            continue; // emit each undirected edge once
        }
        let pa = hull.points[a];
        let pb = hull.points[b];
        out.extend_from_slice(&[pa.x, pa.y, pa.z, pb.x, pb.y, pb.z]);
    }
    out
}

/// VoxelMesh (:1381) built from the fetched `collision_mesh_01.obj` text.
pub(crate) fn build_voxel(obj_text: &str) -> MeshState {
    let origin = Pos {
        x: 5000.0 as _,
        y: 3500.0 as _,
        z: (-7000.0) as _,
    };

    let mut state = MeshState::base(new_world(), MeshScene::Voxel);
    state.base = origin;

    // Ground terrain: z-up, 0.01 scale, median split + edge identification + weld.
    let mesh = create_mesh_data_from_obj(obj_text, 0.01, true, true, true, true)
        .expect("collision_mesh_01.obj → MeshData");
    let mut ground_def = default_body_def();
    ground_def.name = "ground".to_string();
    ground_def.position = origin;
    let ground = create_body(&mut state.world, &ground_def);
    let shape_def = default_shape_def();
    create_mesh_shape(&mut state.world, ground, &shape_def, &mesh, VEC3_ONE);
    // The ground body sits at `origin` = base, so its local mesh coords are already
    // base-relative; bake with a zero offset.
    state.ground_edges = mesh_triangle_edges(&mesh, VEC3_ONE);
    state.stats = vec![mesh.triangle_count as f32];

    // Single dynamic hull resting on the terrain.
    let hull = create_hull(&hull_points(), 16).expect("voxel hull");
    let mut body_def = default_body_def();
    body_def.name = "cylinder".to_string();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: 5020.27734 as _,
        y: 3506.22559 as _,
        z: (-6986.48584) as _,
    };
    body_def.rotation = Quat {
        v: Vec3 {
            x: 0.664546967,
            y: 0.669287264,
            z: 0.135021493,
        },
        s: 0.303646326,
    };
    let body = create_body(&mut state.world, &body_def);
    let mut hull_shape_def = default_shape_def();
    hull_shape_def.base_material.rolling_resistance = 0.1;
    create_hull_shape(&mut state.world, body, &hull_shape_def, &hull);
    state.voxel_hull = Some((body.index1 - 1, hull_wireframe_local(&hull)));

    state
}
