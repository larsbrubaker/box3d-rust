//! Conveyor Mesh sample (`sample_shapes.cpp:488-675`): a static triangle mesh with
//! 7 tangent-velocity materials, plus 20 dynamic cylinders that ride the belts.

use super::{new_world, ShapeScene, ShapeState};
use crate::obj_loader::create_mesh_data_from_obj;
use crate::vis::{pos, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::debug_draw::HexColor;
use box3d_rust::geometry::{default_surface_material, SurfaceMaterial};
use box3d_rust::hull::create_cylinder;
use box3d_rust::math_functions::{
    add, cross, make_quat_from_axis_angle, mul_sv, normalize, rotate_vector, Transform, Vec3, PI,
    QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ONE,
};
use box3d_rust::shape::{create_hull_shape, create_mesh_shape};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};

/// The 7 per-material conveyor velocities (C `m_velocities`).
const VELOCITIES: [Vec3; 7] = [
    Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
    Vec3 {
        x: 0.7,
        y: 0.0,
        z: -0.2,
    },
    Vec3 {
        x: 0.6,
        y: 0.0,
        z: 0.4,
    },
    Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.3,
    },
    Vec3 {
        x: -0.6,
        y: 0.0,
        z: 0.4,
    },
    Vec3 {
        x: -0.75,
        y: 0.0,
        z: -0.4,
    },
    Vec3 {
        x: 0.0,
        y: 0.0,
        z: -1.3,
    },
];

/// The 7 per-material colors (C `colors[]`).
const COLORS: [HexColor; 7] = [
    HexColor::GREEN,
    HexColor::GREEN_YELLOW,
    HexColor::HONEY_DEW,
    HexColor::HOT_PINK,
    HexColor::INDIAN_RED,
    HexColor::INDIGO,
    HexColor::IVORY,
];

/// C `ConveyorMesh` ctor. `obj_text` is the fetched `conveyor.obj`.
pub(crate) fn build_conveyor_mesh(obj_text: &str) -> ShapeState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let ground = super::add_ground_box(&mut world, 20.0);
    bodies.push(VisBody::box_body(ground.index1 - 1, 20.0, 1.0, 20.0));

    // C CreateMeshData( path, 1.0, false, true, true, true ).
    let mut mesh_data = create_mesh_data_from_obj(obj_text, 1.0, false, true, true, true)
        .expect("conveyor.obj -> MeshData");

    // C: memset the material indices to 0, then stamp specific triangles.
    for m in mesh_data.material_indices.iter_mut() {
        *m = 0;
    }
    let set = |mi: &mut [u8], i: usize, v: u8| {
        if i < mi.len() {
            mi[i] = v;
        }
    };
    let mi = &mut mesh_data.material_indices;
    set(mi, 0, 1);
    set(mi, 4, 1);
    set(mi, 9, 2);
    set(mi, 12, 2);
    set(mi, 21, 3);
    set(mi, 38, 3);
    set(mi, 43, 4);
    set(mi, 46, 4);
    set(mi, 30, 5);
    set(mi, 33, 5);
    set(mi, 18, 6);
    set(mi, 24, 6);

    // 7 materials: friction 0.8, tangentVelocity = 2·velocity, customColor.
    let mut materials: Vec<SurfaceMaterial> = Vec::with_capacity(7);
    for i in 0..7 {
        let mut m = default_surface_material();
        m.friction = 0.8;
        m.tangent_velocity = mul_sv(2.0, VELOCITIES[i]);
        m.custom_color = COLORS[i].0;
        materials.push(m);
    }

    // Mesh transform: p = {0,0.5,6}, q = axisY(0.5π).
    let mesh_p = Vec3 {
        x: 0.0,
        y: 0.5,
        z: 6.0,
    };
    let mesh_q = make_quat_from_axis_angle(VEC3_AXIS_Y, 0.5 * PI);
    let mesh_xf = Transform {
        p: mesh_p,
        q: mesh_q,
    };

    let mut body_def = default_body_def();
    body_def.position = pos(mesh_p.x, mesh_p.y, mesh_p.z);
    body_def.rotation = mesh_q;
    let mesh_body = create_body(&mut world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.materials = materials;
    create_mesh_shape(&mut world, mesh_body, &shape_def, &mesh_data, VEC3_ONE);

    // Precompute the static render buffers (world-space triangles + per-triangle
    // color + tangent-velocity lines) from the final, BVH-ordered triangle list.
    let (conveyor_tris, conveyor_colors, conveyor_vel_lines) =
        build_render_buffers(&mesh_data, mesh_xf);

    // 32-sided cylinders (C: high side count to stress collision).
    let cylinder_hull = create_cylinder(0.3, 0.15, 0.0, 32).expect("conveyor cylinder");
    let cyl_shape_def = default_shape_def();
    // Render as a cylinder along local Y, spanning [0,0.3] → offset +0.15.
    let cyl_local = Transform {
        p: Vec3 {
            x: 0.0,
            y: 0.15,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    for i in 0..20 {
        let mut cyl_def = default_body_def();
        cyl_def.type_ = BodyType::Dynamic;
        cyl_def.position = pos(-8.5 + 0.9 * i as f32, 1.5, -5.5);
        let body = create_body(&mut world, &cyl_def);
        create_hull_shape(&mut world, body, &cyl_shape_def, &cylinder_hull);
        bodies.push(VisBody::cylinder_local(
            body.index1 - 1,
            0.15,
            0.15,
            cyl_local,
            0,
        ));
    }

    let mut state = ShapeState::base(world, ShapeScene::ConveyorMesh);
    state.bodies = bodies;
    state.conveyor_tris = conveyor_tris;
    state.conveyor_colors = conveyor_colors;
    state.conveyor_vel_lines = conveyor_vel_lines;
    state
}

/// Build the static render buffers for the conveyor mesh:
/// - world-space triangle vertices (9 floats/tri)
/// - per-triangle 0xRRGGBB color
/// - tangent-velocity direction lines for up-facing triangles (C `Render`)
fn build_render_buffers(
    mesh: &box3d_rust::mesh::MeshData,
    xf: Transform,
) -> (Vec<f32>, Vec<u32>, Vec<f32>) {
    let verts = &mesh.vertices;
    let tris = &mesh.triangles;
    let mut tri_out = Vec::with_capacity(tris.len() * 9);
    let mut color_out = Vec::with_capacity(tris.len());
    let mut vel_out = Vec::new();

    let to_world = |v: Vec3| -> Vec3 { add(rotate_vector(xf.q, v), xf.p) };

    for (i, t) in tris.iter().enumerate() {
        let v1 = verts[t.index1 as usize];
        let v2 = verts[t.index2 as usize];
        let v3 = verts[t.index3 as usize];
        let w1 = to_world(v1);
        let w2 = to_world(v2);
        let w3 = to_world(v3);
        tri_out.extend_from_slice(&[w1.x, w1.y, w1.z, w2.x, w2.y, w2.z, w3.x, w3.y, w3.z]);

        let mat_index = mesh.material_indices[i] as usize;
        color_out.push(COLORS[mat_index].0 & 0x00FF_FFFF);

        // C Render(): only up-facing triangles (local normal.y >= 0.9) get a line.
        let n = normalize(cross(
            Vec3 {
                x: v2.x - v1.x,
                y: v2.y - v1.y,
                z: v2.z - v1.z,
            },
            Vec3 {
                x: v3.x - v1.x,
                y: v3.y - v1.y,
                z: v3.z - v1.z,
            },
        ));
        if n.y < 0.9 {
            continue;
        }
        let center_local = mul_sv(1.0 / 3.0, add(add(v1, v2), v3));
        let p = to_world(center_local);
        let v = rotate_vector(xf.q, VELOCITIES[mat_index]);
        let p2 = add(p, v);
        vel_out.extend_from_slice(&[p.x, p.y, p.z, p2.x, p2.y, p2.z]);
    }

    (tri_out, color_out, vel_out)
}

#[cfg(test)]
mod tests {
    //! Guards the Conveyor Mesh material-stamp seam.
    //!
    //! `build_conveyor_mesh` stamps the 7 tangent-velocity materials onto specific
    //! *post-split* triangle indices (0, 4, 9, 12, 21, 38, 43, 46, 30, 33, 18, 24).
    //! Those indices are only correct because `create_mesh`'s median-split BVH
    //! reorders `conveyor.obj`'s triangles into the exact permutation the C sample
    //! sees (`create_mesh` builds the same BVH as upstream). If that ordering ever
    //! drifts, the wrong belts would move — a silent visual regression. This test
    //! pins the centroid of every stamped triangle (visually verified against the C
    //! sample this batch), so any reordering trips here instead of on screen.

    /// Centroids of the stamped triangles, in `conveyor.obj` local space, at the
    /// post-median-split indices used by `build_conveyor_mesh`.
    const STAMPED: &[(usize, [f32; 3])] = &[
        (0, [2.0171, 1.0000, -12.8862]),
        (4, [5.5216, 1.0000, -11.6567]),
        (9, [8.0840, 1.0000, -11.4164]),
        (12, [11.4131, 1.0000, -12.0585]),
        (21, [12.1100, 1.0000, -3.8048]),
        (38, [14.3127, 1.0000, 2.9384]),
        (43, [7.9122, 1.0000, 11.5165]),
        (46, [11.1603, 1.0000, 12.3294]),
        (30, [1.6820, 1.0000, 12.5311]),
        (33, [5.2676, 1.0000, 11.4161]),
        (18, [0.0694, 1.0000, -4.7329]),
        (24, [-1.9993, 1.0000, 2.5527]),
    ];

    #[test]
    fn conveyor_mesh_triangle_order_is_pinned() {
        // Exactly the `build_conveyor_mesh` load: CreateMeshData(scale 1, no z-up,
        // median split, identify edges, weld).
        let obj = include_str!("../../../../box3d-cpp-reference/data/meshes/conveyor.obj");
        let mesh = crate::obj_loader::create_mesh_data_from_obj(obj, 1.0, false, true, true, true)
            .expect("conveyor mesh");

        assert_eq!(
            mesh.triangles.len(),
            48,
            "conveyor mesh triangle count changed — the stamp indices assume 48 tris"
        );

        for &(i, [ex, ey, ez]) in STAMPED {
            let t = &mesh.triangles[i];
            let a = mesh.vertices[t.index1 as usize];
            let b = mesh.vertices[t.index2 as usize];
            let c = mesh.vertices[t.index3 as usize];
            let cx = (a.x + b.x + c.x) / 3.0;
            let cy = (a.y + b.y + c.y) / 3.0;
            let cz = (a.z + b.z + c.z) / 3.0;
            let tol = 5.0e-3;
            assert!(
                (cx - ex).abs() < tol && (cy - ey).abs() < tol && (cz - ez).abs() < tol,
                "stamped triangle {i} moved: got ({cx:.4}, {cy:.4}, {cz:.4}), \
                 expected ({ex:.4}, {ey:.4}, {ez:.4}). The median-split triangle order \
                 changed, so build_conveyor_mesh now stamps the wrong belts."
            );
        }
    }
}
