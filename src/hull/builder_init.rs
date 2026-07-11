//! Initial hull construction: tolerance, farthest-point search, tetrahedron seed.

use super::builder_pool::*;
use crate::core::NULL_INDEX;
use crate::math_functions::{
    abs, abs_float, add, cross, dot, make_plane_from_points, max, max_element_index, max_float,
    min_float, mul_sv, plane_separation, scalar_triple_product, sub, Aabb, Vec3, BOUNDS3_EMPTY,
    SQRT3, VEC3_ZERO,
};

const AXIS_X: usize = 0;
const AXIS_Y: usize = 1;
const AXIS_Z: usize = 2;

fn build_bounds(vertices: &[Vec3]) -> Aabb {
    let mut bounds = BOUNDS3_EMPTY;
    for v in vertices {
        bounds.lower_bound = crate::math_functions::min(bounds.lower_bound, *v);
        bounds.upper_bound = max(bounds.upper_bound, *v);
    }
    bounds
}

fn find_farthest_points_along_cardinal_axes(
    tolerance: f32,
    vertices: &[Vec3],
) -> (i32, i32) {
    let mut index1 = NULL_INDEX;
    let mut index2 = NULL_INDEX;

    let v0 = vertices[0];
    let mut min_pt = [v0; 3];
    let mut max_pt = [v0; 3];
    let mut min_index = [0i32; 3];
    let mut max_index = [0i32; 3];

    for (i, &v) in vertices.iter().enumerate().skip(1) {
        let i = i as i32;
        if v.x < min_pt[AXIS_X].x {
            min_pt[AXIS_X] = v;
            min_index[AXIS_X] = i;
        } else if v.x > max_pt[AXIS_X].x {
            max_pt[AXIS_X] = v;
            max_index[AXIS_X] = i;
        }

        if v.y < min_pt[AXIS_Y].y {
            min_pt[AXIS_Y] = v;
            min_index[AXIS_Y] = i;
        } else if v.y > max_pt[AXIS_Y].y {
            max_pt[AXIS_Y] = v;
            max_index[AXIS_Y] = i;
        }

        if v.z < min_pt[AXIS_Z].z {
            min_pt[AXIS_Z] = v;
            min_index[AXIS_Z] = i;
        } else if v.z > max_pt[AXIS_Z].z {
            max_pt[AXIS_Z] = v;
            max_index[AXIS_Z] = i;
        }
    }

    let distance = Vec3 {
        x: max_pt[AXIS_X].x - min_pt[AXIS_X].x,
        y: max_pt[AXIS_Y].y - min_pt[AXIS_Y].y,
        z: max_pt[AXIS_Z].z - min_pt[AXIS_Z].z,
    };
    let distance_array = [distance.x, distance.y, distance.z];
    let max_element = max_element_index(distance) as usize;

    if distance_array[max_element] > 2.0 * tolerance {
        index1 = min_index[max_element];
        index2 = max_index[max_element];
    }

    (index1, index2)
}

fn find_farthest_point_from_line(
    index1: i32,
    index2: i32,
    tolerance: f32,
    vertices: &[Vec3],
) -> i32 {
    let a = vertices[index1 as usize];
    let b = vertices[index2 as usize];
    let ab = sub(b, a);
    let ab_length_sqr = dot(ab, ab);
    debug_assert!(ab_length_sqr > 0.0);

    let inv_ab_length_sqr = 1.0 / ab_length_sqr;
    let mut max_distance_sqr = 4.0 * tolerance * tolerance;
    let mut max_index = NULL_INDEX;

    for (i, &p) in vertices.iter().enumerate() {
        let i = i as i32;
        if i == index1 || i == index2 {
            continue;
        }
        let ap = sub(p, a);
        let c = cross(ap, ab);
        let distance_sqr = dot(c, c) * inv_ab_length_sqr;
        if distance_sqr > max_distance_sqr {
            max_distance_sqr = distance_sqr;
            max_index = i;
        }
    }
    max_index
}

fn find_farthest_point_from_plane(
    index1: i32,
    index2: i32,
    index3: i32,
    tolerance: f32,
    vertices: &[Vec3],
) -> i32 {
    let plane = make_plane_from_points(
        vertices[index1 as usize],
        vertices[index2 as usize],
        vertices[index3 as usize],
    );
    let mut max_distance = 2.0 * tolerance;
    let mut max_index = NULL_INDEX;

    for (i, &p) in vertices.iter().enumerate() {
        let i = i as i32;
        if i == index1 || i == index2 || i == index3 {
            continue;
        }
        let distance = abs_float(plane_separation(plane, p));
        if distance > max_distance {
            max_distance = distance;
            max_index = i;
        }
    }
    max_index
}

pub(super) fn link_face(b: &mut HullBuilder, face: i32, mut index: i32, twin: i32) {
    debug_assert!(face != b.edges[twin as usize].face);
    let mut edge = b.faces[face as usize].edge;
    while index > 0 {
        debug_assert!(b.edges[edge as usize].face == face);
        edge = b.edges[edge as usize].next;
        index -= 1;
    }
    debug_assert!(edge != twin);
    b.edges[edge as usize].twin = twin;
    b.edges[twin as usize].twin = edge;
}

pub(super) fn link_faces(
    b: &mut HullBuilder,
    face1: i32,
    mut index1: i32,
    face2: i32,
    mut index2: i32,
) {
    debug_assert!(face1 != face2);
    let mut edge1 = b.faces[face1 as usize].edge;
    while index1 > 0 {
        edge1 = b.edges[edge1 as usize].next;
        index1 -= 1;
    }
    let mut edge2 = b.faces[face2 as usize].edge;
    while index2 > 0 {
        edge2 = b.edges[edge2 as usize].next;
        index2 -= 1;
    }
    debug_assert!(edge1 != edge2);
    b.edges[edge1 as usize].twin = edge2;
    b.edges[edge2 as usize].twin = edge1;
}

impl HullBuilder {
    pub fn compute_tolerance(&mut self, points: &[Vec3]) {
        let bounds = build_bounds(points);
        let max_abs = max(abs(bounds.lower_bound), abs(bounds.upper_bound));
        let max_sum = max_abs.x + max_abs.y + max_abs.z;
        let max_coord = max_float(max_abs.x, max_float(max_abs.y, max_abs.z));
        let max_distance = min_float(SQRT3 * max_coord, max_sum);
        let tolerance = (3.0 * max_distance * 1.01 + max_coord) * f32::EPSILON;
        self.tolerance = tolerance;
        self.min_radius = 4.0 * self.tolerance;
        self.min_outside = 2.0 * self.min_radius;
        debug_assert!(self.min_radius < self.min_outside + 3.0 * f32::EPSILON);
    }

    pub fn build_initial_hull(&mut self, points: &[Vec3]) -> bool {
        let (index1, mut index2) =
            find_farthest_points_along_cardinal_axes(self.tolerance, points);
        if index1 < 0 || index2 < 0 {
            return false;
        }

        let mut index3 = find_farthest_point_from_line(index1, index2, self.tolerance, points);
        if index3 < 0 {
            return false;
        }

        let index4 =
            find_farthest_point_from_plane(index1, index2, index3, self.tolerance, points);
        if index4 < 0 {
            return false;
        }

        let v1 = sub(points[index1 as usize], points[index4 as usize]);
        let v2 = sub(points[index2 as usize], points[index4 as usize]);
        let v3 = sub(points[index3 as usize], points[index4 as usize]);

        if scalar_triple_product(v1, v2, v3) < 0.0 {
            core::mem::swap(&mut index2, &mut index3);
        }

        self.interior_point = VEC3_ZERO;
        self.interior_point = add(self.interior_point, points[index1 as usize]);
        self.interior_point = add(self.interior_point, points[index2 as usize]);
        self.interior_point = add(self.interior_point, points[index3 as usize]);
        self.interior_point = add(self.interior_point, points[index4 as usize]);
        self.interior_point = mul_sv(0.25, self.interior_point);

        let vertex1 = self.new_vertex(points[index1 as usize]);
        self.vertex_list_push_back(vertex1);
        let vertex2 = self.new_vertex(points[index2 as usize]);
        self.vertex_list_push_back(vertex2);
        let vertex3 = self.new_vertex(points[index3 as usize]);
        self.vertex_list_push_back(vertex3);
        let vertex4 = self.new_vertex(points[index4 as usize]);
        self.vertex_list_push_back(vertex4);

        let face1 = self.new_face(vertex1, vertex2, vertex3);
        self.face_list_push_back(face1);
        let face2 = self.new_face(vertex4, vertex2, vertex1);
        self.face_list_push_back(face2);
        let face3 = self.new_face(vertex4, vertex3, vertex2);
        self.face_list_push_back(face3);
        let face4 = self.new_face(vertex4, vertex1, vertex3);
        self.face_list_push_back(face4);

        link_faces(self, face1, 0, face2, 1);
        link_faces(self, face1, 1, face3, 1);
        link_faces(self, face1, 2, face4, 1);
        link_faces(self, face2, 0, face3, 2);
        link_faces(self, face3, 0, face4, 2);
        link_faces(self, face4, 0, face2, 2);

        for (index, &point) in points.iter().enumerate() {
            let index = index as i32;
            if index == index1 || index == index2 || index == index3 || index == index4 {
                continue;
            }

            let mut max_distance = self.min_outside;
            let mut max_face = NULL_INDEX;

            let mut node = self.face_list.next;
            while node != SENTINEL {
                let face = node;
                let distance = plane_separation(self.faces[face as usize].plane, point);
                if distance > max_distance {
                    max_distance = distance;
                    max_face = face;
                }
                node = self.faces[face as usize].link.next;
            }

            if max_face != NULL_INDEX {
                let vertex = self.new_vertex(point);
                self.vertices[vertex as usize].conflict_face = max_face;
                self.conflict_push_back(max_face, vertex);
                if max_distance > self.faces[max_face as usize].max_conflict_distance {
                    self.faces[max_face as usize].max_conflict_distance = max_distance;
                    self.faces[max_face as usize].max_conflict = vertex;
                }
            }
        }

        true
    }
}
