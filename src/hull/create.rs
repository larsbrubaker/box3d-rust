//! Hull creation: CreateHull, Clone, Destroy, Cylinder/Cone/Rock.

use super::builder_pool::{compute_hull_work_sizes, HullBuilder, HULL_LIMIT, SENTINEL};
use super::types::{HullData, HullFace, HullHalfEdge, HullVertex, HULL_DATA_SIZE, HULL_VERSION};
use super::validate::is_valid_hull;
use crate::core::{hash, non_zero_hash, HASH_INIT};
use crate::math_functions::{
    add, align_up8, clamp_int, cos, cross, length, min, max, mul_sv, plane_separation,
    scalar_triple_product, sin, steiner, sub, sub_mm, mul_sm, compute_cos_sin, Vec3, VEC3_ZERO,
    PI,
};

fn update_hull_bounds(hull: &mut HullData) {
    let points = &hull.points;
    let vertex_count = hull.vertex_count as usize;
    debug_assert!(vertex_count > 0);
    let mut bounds = crate::math_functions::Aabb {
        lower_bound: points[0],
        upper_bound: points[0],
    };
    for i in 1..vertex_count {
        let p = points[i];
        bounds.lower_bound = min(bounds.lower_bound, p);
        bounds.upper_bound = max(bounds.upper_bound, p);
    }
    hull.aabb = bounds;
}

/// M. Kallay — moment of inertia of a solid defined by a triangle mesh.
fn update_hull_bulk_properties(hull: &mut HullData) -> bool {
    let points = &hull.points;
    let faces = &hull.faces;
    let edges = &hull.edges;
    let planes = &hull.planes;

    let mut area = 0.0f32;
    let mut volume = 0.0f32;
    let mut center = VEC3_ZERO;
    let origin = points[0];

    let mut xx = 0.0f32;
    let mut xy = 0.0f32;
    let mut yy = 0.0f32;
    let mut xz = 0.0f32;
    let mut zz = 0.0f32;
    let mut yz = 0.0f32;

    let face_count = hull.face_count as usize;
    for face_index in 0..face_count {
        let face = faces[face_index];
        let edge1_i = face.edge as usize;
        let edge2_i = edges[edge1_i].next as usize;
        let mut edge3 = edges[edge2_i].next as usize;

        debug_assert!(edge1_i != edge3);
        debug_assert!((edges[edge1_i].origin as i32) < hull.vertex_count);

        let v1 = sub(points[edges[edge1_i].origin as usize], origin);
        let mut edge2 = edge2_i;

        loop {
            debug_assert!((edges[edge2].origin as i32) < hull.vertex_count);
            debug_assert!((edges[edge3].origin as i32) < hull.vertex_count);

            let v2 = sub(points[edges[edge2].origin as usize], origin);
            let v3 = sub(points[edges[edge3].origin as usize], origin);

            area += length(cross(sub(v2, v1), sub(v3, v1)));

            let det = scalar_triple_product(v1, v2, v3);
            volume += det;

            let v4 = add(v1, add(v2, v3));
            center = add(center, mul_sv(det, v4));

            xx += det * (v1.x * v1.x + v2.x * v2.x + v3.x * v3.x + v4.x * v4.x);
            yy += det * (v1.y * v1.y + v2.y * v2.y + v3.y * v3.y + v4.y * v4.y);
            zz += det * (v1.z * v1.z + v2.z * v2.z + v3.z * v3.z + v4.z * v4.z);
            xy += det * (v1.x * v1.y + v2.x * v2.y + v3.x * v3.y + v4.x * v4.y);
            xz += det * (v1.x * v1.z + v2.x * v2.z + v3.x * v3.z + v4.x * v4.z);
            yz += det * (v1.y * v1.z + v2.y * v2.z + v3.y * v3.z + v4.y * v4.z);

            edge2 = edge3;
            edge3 = edges[edge3].next as usize;
            if edge1_i == edge3 {
                break;
            }
        }
    }

    debug_assert!(volume > 0.0);

    let local_center = if volume > 0.0 {
        mul_sv(0.25 / volume, center)
    } else {
        VEC3_ZERO
    };
    center = add(local_center, origin);

    let mut radius = f32::MAX;
    for face_index in 0..face_count {
        let plane = planes[face_index];
        let distance = plane_separation(plane, center);
        debug_assert!(distance < 0.0);
        radius = crate::math_functions::min_float(radius, -distance);
    }

    debug_assert!(0.0 < radius && radius < f32::MAX);

    let mut inertia = crate::math_functions::MAT3_ZERO;
    inertia.cx.x = yy + zz;
    inertia.cy.x = -xy;
    inertia.cz.x = -xz;
    inertia.cx.y = -xy;
    inertia.cy.y = xx + zz;
    inertia.cz.y = -yz;
    inertia.cx.z = -xz;
    inertia.cy.z = -yz;
    inertia.cz.z = xx + yy;

    let mass = volume / 6.0;
    let mut central_inertia = mul_sm(1.0 / 120.0, inertia);
    central_inertia = sub_mm(central_inertia, steiner(mass, local_center));

    hull.center = center;
    hull.central_inertia = central_inertia;
    hull.volume = mass;
    hull.surface_area = 0.5 * area;
    hull.inner_radius = radius;

    mass > 0.0 && volume > 0.0 && area > 0.0 && radius > 0.0
}

fn finalize_hash(hull: &mut HullData) {
    hull.hash = 0;
    let bytes = hull.to_bytes_with_hash(0);
    hull.hash = non_zero_hash(hash(HASH_INIT, &bytes));
}

/// Create a convex hull from a point cloud. (b3CreateHull)
pub fn create_hull(points: &[Vec3], max_vertex_count: i32) -> Option<HullData> {
    let point_count = points.len() as i32;
    if point_count < 4 {
        return None;
    }

    let origin = points[0];
    let clamped_max_count = clamp_int(max_vertex_count, 4, HULL_LIMIT);
    let sizes = compute_hull_work_sizes(point_count, clamped_max_count);
    let mut builder = HullBuilder::new(&sizes);
    let mut shifted_points = vec![VEC3_ZERO; point_count as usize];

    if !builder.construct(points, clamped_max_count, origin, &mut shifted_points) {
        return None;
    }

    if builder.final_vertex_count >= HULL_LIMIT
        || builder.final_face_count >= HULL_LIMIT
        || builder.final_half_edge_count >= HULL_LIMIT
    {
        return None;
    }

    let mut temp_vertices = Vec::with_capacity(HULL_LIMIT as usize);
    let mut vertex_count = 0i32;
    let mut node = builder.vertex_list.next;
    while node != SENTINEL {
        debug_assert!(vertex_count <= HULL_LIMIT - 1);
        builder.vertices[node as usize].final_index = vertex_count;
        temp_vertices.push(node);
        vertex_count += 1;
        node = builder.vertices[node as usize].link.next;
    }

    let mut temp_faces = Vec::with_capacity(HULL_LIMIT as usize);
    let mut temp_edges = vec![0i32; HULL_LIMIT as usize];
    let mut face_count = 0i32;
    let mut edge_count = 0i32;

    let mut face_node = builder.face_list.next;
    while face_node != SENTINEL {
        debug_assert!(face_count <= HULL_LIMIT - 1);
        let face = face_node;
        builder.faces[face as usize].final_index = face_count;
        temp_faces.push(face);
        face_count += 1;

        let start = builder.faces[face as usize].edge;
        let mut edge = start;
        loop {
            if builder.edges[edge as usize].final_index < 0 {
                debug_assert!(edge_count + 1 <= HULL_LIMIT - 1);
                builder.edges[edge as usize].final_index = edge_count;
                temp_edges[edge_count as usize] = edge;
                edge_count += 1;
                let twin = builder.edges[edge as usize].twin;
                builder.edges[twin as usize].final_index = edge_count;
                temp_edges[edge_count as usize] = twin;
                edge_count += 1;
            }
            edge = builder.edges[edge as usize].next;
            if edge == start {
                break;
            }
        }

        face_node = builder.faces[face as usize].link.next;
    }

    let mut byte_count = align_up8(HULL_DATA_SIZE);
    let vertex_offset = byte_count as i32;
    byte_count += align_up8(vertex_count as usize * core::mem::size_of::<HullVertex>());
    let point_offset = byte_count as i32;
    byte_count += align_up8(vertex_count as usize * core::mem::size_of::<Vec3>());
    let edge_offset = byte_count as i32;
    byte_count += align_up8(edge_count as usize * core::mem::size_of::<HullHalfEdge>());
    let face_offset = byte_count as i32;
    byte_count += align_up8(face_count as usize * core::mem::size_of::<HullFace>());
    let plane_offset = byte_count as i32;
    byte_count += align_up8(face_count as usize * core::mem::size_of::<crate::math_functions::Plane>());

    let mut hull = HullData {
        version: HULL_VERSION,
        byte_count: byte_count as i32,
        hash: 0,
        aabb: Default::default(),
        surface_area: 0.0,
        volume: 0.0,
        inner_radius: 0.0,
        center: VEC3_ZERO,
        central_inertia: crate::math_functions::MAT3_ZERO,
        vertex_count,
        vertex_offset,
        point_offset,
        edge_count,
        edge_offset,
        face_count,
        face_offset,
        plane_offset,
        padding: 0,
        vertices: vec![HullVertex { edge: 0 }; vertex_count as usize],
        points: vec![VEC3_ZERO; vertex_count as usize],
        edges: vec![HullHalfEdge::default(); edge_count as usize],
        faces: vec![HullFace { edge: 0 }; face_count as usize],
        planes: vec![
            crate::math_functions::Plane {
                normal: VEC3_ZERO,
                offset: 0.0,
            };
            face_count as usize
        ],
    };

    for index in 0..vertex_count as usize {
        hull.vertices[index].edge = 0;
        hull.points[index] = builder.vertices[temp_vertices[index] as usize].position;
    }

    for index in 0..edge_count as usize {
        let edge = temp_edges[index];
        let e = &builder.edges[edge as usize];
        hull.edges[index] = HullHalfEdge {
            next: builder.edges[e.next as usize].final_index as u8,
            twin: builder.edges[e.twin as usize].final_index as u8,
            face: builder.faces[e.face as usize].final_index as u8,
            origin: builder.vertices[e.origin as usize].final_index as u8,
        };
        hull.vertices[builder.vertices[e.origin as usize].final_index as usize].edge = index as u8;
    }

    for index in 0..face_count as usize {
        let face = temp_faces[index];
        hull.faces[index].edge = builder.edges[builder.faces[face as usize].edge as usize].final_index as u8;
        hull.planes[index] = builder.faces[face as usize].plane;
    }

    update_hull_bounds(&mut hull);
    if !update_hull_bulk_properties(&mut hull) {
        return None;
    }
    if !is_valid_hull(&hull) {
        return None;
    }

    finalize_hash(&mut hull);
    Some(hull)
}

/// Clone a hull. (b3CloneHull)
pub fn clone_hull(hull: &HullData) -> Option<HullData> {
    if !is_valid_hull(hull) {
        return None;
    }
    Some(hull.clone())
}

/// Destroy is a no-op for owned Rust hulls; kept for API parity. (b3DestroyHull)
pub fn destroy_hull(_hull: HullData) {}

/// Create a cylinder hull. (b3CreateCylinder)
pub fn create_cylinder(height: f32, radius: f32, y_offset: f32, sides: i32) -> Option<HullData> {
    debug_assert!(height > 0.0);
    debug_assert!(radius > 0.0);
    debug_assert!((3..=32).contains(&sides));

    let point_count = 2 * sides;
    let mut points = Vec::with_capacity(point_count as usize);
    let mut alpha = 0.0f32;
    let delta_alpha = 2.0 * PI / sides as f32;

    for _ in 0..sides {
        let sin_alpha = sin(alpha);
        let cos_alpha = cos(alpha);
        points.push(Vec3 {
            x: radius * cos_alpha,
            y: y_offset,
            z: radius * sin_alpha,
        });
        points.push(Vec3 {
            x: radius * cos_alpha,
            y: y_offset + height,
            z: radius * sin_alpha,
        });
        alpha += delta_alpha;
    }

    let hull = create_hull(&points, point_count)?;
    debug_assert!(hull.vertex_count == point_count);
    debug_assert!(hull.edge_count == 6 * sides);
    debug_assert!(hull.face_count == sides + 2);
    Some(hull)
}

/// Create a cone/frustum hull. (b3CreateCone)
pub fn create_cone(height: f32, radius1: f32, radius2: f32, slices: i32) -> Option<HullData> {
    debug_assert!(height > 0.0);
    debug_assert!(radius1 > 0.0);
    debug_assert!(radius2 > 0.0);
    debug_assert!((4..=32).contains(&slices));

    let point_count = 2 * slices;
    let mut points = Vec::with_capacity(point_count as usize);
    let mut alpha = 0.0f32;
    let delta_alpha = 2.0 * PI / slices as f32;

    for _ in 0..slices {
        let sin_alpha = sin(alpha);
        let cos_alpha = cos(alpha);
        points.push(Vec3 {
            x: radius1 * cos_alpha,
            y: 0.0,
            z: radius1 * sin_alpha,
        });
        points.push(Vec3 {
            x: radius2 * cos_alpha,
            y: height,
            z: radius2 * sin_alpha,
        });
        alpha += delta_alpha;
    }

    let hull = create_hull(&points, point_count)?;
    debug_assert!(hull.vertex_count == point_count);
    debug_assert!(hull.edge_count == 6 * slices);
    debug_assert!(hull.face_count == slices + 2);
    Some(hull)
}

/// Create a rock-like hull from a Fibonacci lattice. (b3CreateRock)
pub fn create_rock(radius: f32) -> Option<HullData> {
    let point_count = 10;
    let phi = (1.0 + 5.0f32.sqrt()) / 2.0;
    let theta = 2.0 * PI / phi;
    let mut cs = crate::math_functions::CosSin {
        cosine: 1.0,
        sine: 0.0,
    };
    let delta_cs = compute_cos_sin(theta);
    let mut points = [VEC3_ZERO; 10];

    for i in 0..point_count {
        let z = 1.0 - (2.0 * i as f32 + 1.0) / point_count as f32;
        let radius_xy = (1.0 - z * z).sqrt();
        points[i] = Vec3 {
            x: radius * radius_xy * cs.cosine,
            y: radius * radius_xy * cs.sine,
            z: radius * z,
        };
        let cs0 = cs;
        cs.cosine = delta_cs.cosine * cs0.cosine - delta_cs.sine * cs0.sine;
        cs.sine = delta_cs.sine * cs0.cosine + delta_cs.cosine * cs0.sine;
    }

    create_hull(&points, point_count as i32)
}
