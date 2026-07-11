//! Quickhull incremental ops: horizon, cone, merge, resolve, construct.

use super::builder_init::{link_face, link_faces};
use super::builder_pool::*;
use crate::core::NULL_INDEX;
use crate::math_functions::{
    add, clamp_int, dot, length, make_plane_from_normal_and_point, mul_sv, plane_separation, sub,
    Vec3, VEC3_ZERO,
};

fn is_edge_convex(b: &HullBuilder, edge: i32, tolerance: f32) -> bool {
    let twin = b.edges[edge as usize].twin;
    let face = b.edges[edge as usize].face;
    let twin_face = b.edges[twin as usize].face;
    let distance = plane_separation(
        b.faces[face as usize].plane,
        b.faces[twin_face as usize].centroid,
    );
    distance < -tolerance
}

fn is_edge_concave(b: &HullBuilder, edge: i32, tolerance: f32) -> bool {
    let twin = b.edges[edge as usize].twin;
    let face = b.edges[edge as usize].face;
    let twin_face = b.edges[twin as usize].face;
    let distance = plane_separation(
        b.faces[face as usize].plane,
        b.faces[twin_face as usize].centroid,
    );
    distance > tolerance
}

fn vertex_count_of_face(b: &HullBuilder, face: i32) -> i32 {
    let mut count = 0;
    let start = b.faces[face as usize].edge;
    let mut edge = start;
    loop {
        count += 1;
        edge = b.edges[edge as usize].next;
        if edge == start {
            break;
        }
    }
    count
}

fn newell_plane(b: &mut HullBuilder, face: i32) {
    let mut count = 0;
    let mut centroid = VEC3_ZERO;
    let mut normal = VEC3_ZERO;

    let start = b.faces[face as usize].edge;
    debug_assert!(b.edges[start as usize].face == face);
    let origin = b.vertices[b.edges[start as usize].origin as usize].position;

    let mut edge = start;
    loop {
        let twin = b.edges[edge as usize].twin;
        debug_assert!(b.edges[twin as usize].twin == edge);

        let v1 = sub(
            b.vertices[b.edges[edge as usize].origin as usize].position,
            origin,
        );
        let v2 = sub(
            b.vertices[b.edges[twin as usize].origin as usize].position,
            origin,
        );

        count += 1;
        centroid = add(centroid, v1);
        normal.x += (v1.y - v2.y) * (v1.z + v2.z);
        normal.y += (v1.z - v2.z) * (v1.x + v2.x);
        normal.z += (v1.x - v2.x) * (v1.y + v2.y);

        edge = b.edges[edge as usize].next;
        if edge == start {
            break;
        }
    }

    debug_assert!(count > 0);
    centroid = mul_sv(1.0 / count as f32, centroid);
    centroid = add(centroid, origin);

    let len = length(normal);
    debug_assert!(len > 0.0);
    normal = mul_sv(1.0 / len, normal);

    b.faces[face as usize].centroid = centroid;
    b.faces[face as usize].plane = make_plane_from_normal_and_point(normal, centroid);
    b.faces[face as usize].area = 0.5 * len;
}

fn recache_conflicts(b: &mut HullBuilder, face: i32, min_outside: f32) {
    let mut max_vertex = NULL_INDEX;
    let mut max_distance = min_outside;

    let mut node = b.faces[face as usize].conflict_list_head.next;
    while node != SENTINEL {
        let vertex = node;
        let distance = plane_separation(
            b.faces[face as usize].plane,
            b.vertices[vertex as usize].position,
        );
        if distance > max_distance {
            max_distance = distance;
            max_vertex = vertex;
        }
        node = b.vertices[vertex as usize].link.next;
    }

    b.faces[face as usize].max_conflict = max_vertex;
    b.faces[face as usize].max_conflict_distance = max_distance;
}

impl HullBuilder {
    pub fn next_conflict_vertex(&self) -> i32 {
        let mut max_vertex = NULL_INDEX;
        let mut max_distance = self.min_outside;

        let mut face_node = self.face_list.next;
        while face_node != SENTINEL {
            let face = face_node;
            let mc = self.faces[face as usize].max_conflict;
            let mcd = self.faces[face as usize].max_conflict_distance;
            if mc != NULL_INDEX && mcd > max_distance {
                max_distance = mcd;
                max_vertex = mc;
            }
            face_node = self.faces[face as usize].link.next;
        }
        max_vertex
    }

    fn drain_conflict_list(&mut self, face: i32) {
        let mut node = self.faces[face as usize].conflict_list_head.next;
        while node != SENTINEL {
            let orphan = node;
            node = self.vertices[orphan as usize].link.next;

            self.vertices[orphan as usize].conflict_face = NULL_INDEX;
            self.conflict_remove(face, orphan);
            self.orphaned_list_push_back(orphan);
        }
        debug_assert!(list_empty(&self.faces[face as usize].conflict_list_head));
    }

    fn enter_horizon_face(&mut self, face: i32, entry_edge: i32, frame_out: &mut HorizonFrame) {
        self.faces[face as usize].mark = MARK_DELETE;
        self.drain_conflict_list(face);

        frame_out.face = face;
        frame_out.started = false;
        if entry_edge != NULL_INDEX {
            frame_out.start_edge = entry_edge;
            frame_out.edge = self.edges[entry_edge as usize].next;
        } else {
            frame_out.start_edge = self.faces[face as usize].edge;
            frame_out.edge = self.faces[face as usize].edge;
        }
    }

    fn build_horizon(&mut self, apex: i32, seed: i32) {
        let mut top = 0i32;
        debug_assert!(top < self.horizon_stack.len() as i32);
        let mut frame = HorizonFrame {
            face: 0,
            start_edge: 0,
            edge: 0,
            started: false,
        };
        self.enter_horizon_face(seed, NULL_INDEX, &mut frame);
        self.horizon_stack[top as usize] = frame;
        top += 1;

        while top > 0 {
            let mut f = self.horizon_stack[(top - 1) as usize];
            if f.started && f.edge == f.start_edge {
                top -= 1;
                continue;
            }
            f.started = true;

            let edge = f.edge;
            let twin = self.edges[edge as usize].twin;
            f.edge = self.edges[edge as usize].next;
            self.horizon_stack[(top - 1) as usize] = f;

            let twin_face = self.edges[twin as usize].face;
            if self.faces[twin_face as usize].mark != MARK_VISIBLE {
                continue;
            }

            let distance = plane_separation(
                self.faces[twin_face as usize].plane,
                self.vertices[apex as usize].position,
            );
            if distance > self.min_radius {
                debug_assert!(top < self.horizon_stack.len() as i32);
                let mut child = HorizonFrame {
                    face: 0,
                    start_edge: 0,
                    edge: 0,
                    started: false,
                };
                self.enter_horizon_face(twin_face, twin, &mut child);
                self.horizon_stack[top as usize] = child;
                top += 1;
            } else {
                debug_assert!(self.horizon_count < self.horizon.len() as i32);
                self.horizon[self.horizon_count as usize] = edge;
                self.horizon_count += 1;
            }
        }
    }

    fn build_cone(&mut self, apex: i32) {
        for i in 0..self.horizon_count {
            let edge = self.horizon[i as usize];
            debug_assert!(self.edges[self.edges[edge as usize].twin as usize].twin == edge);

            let origin = self.edges[edge as usize].origin;
            let twin_origin = self.edges[self.edges[edge as usize].twin as usize].origin;
            let face = self.new_face(apex, origin, twin_origin);
            debug_assert!(self.cone_count < self.cone.len() as i32);
            self.cone[self.cone_count as usize] = face;
            self.cone_count += 1;

            link_face(self, face, 1, self.edges[edge as usize].twin);
        }

        let mut face1 = self.cone[(self.cone_count - 1) as usize];
        for i in 0..self.cone_count {
            let face2 = self.cone[i as usize];
            link_faces(self, face1, 2, face2, 0);
            face1 = face2;
        }
    }

    fn destroy_edges(&mut self, begin: i32, end: i32) {
        let mut edge = begin;
        while edge != end {
            let next = self.edges[edge as usize].next;
            self.retire_edge(edge);
            edge = next;
        }
    }

    fn connect_edges(&mut self, prev: i32, next: i32) {
        debug_assert!(prev != next);
        debug_assert!(self.edges[prev as usize].face == self.edges[next as usize].face);

        let prev_twin_face = self.edges[self.edges[prev as usize].twin as usize].face;
        let next_twin_face = self.edges[self.edges[next as usize].twin as usize].face;

        if prev_twin_face == next_twin_face {
            if self.faces[self.edges[next as usize].face as usize].edge == next {
                self.faces[self.edges[next as usize].face as usize].edge = prev;
            }

            let twin;
            if vertex_count_of_face(self, prev_twin_face) == 3 {
                let dead_edge0 = self.edges[prev as usize].twin;
                let dead_edge1 = self.edges[next as usize].twin;
                let dead_edge2 = self.edges[dead_edge1 as usize].prev;

                twin = self.edges[dead_edge2 as usize].twin;
                debug_assert!(
                    self.faces[self.edges[twin as usize].face as usize].mark != MARK_DELETE
                );

                let opposing_face = prev_twin_face;
                self.faces[opposing_face as usize].mark = MARK_DELETE;
                debug_assert!(self.merged_faces_count < self.merged_faces.len() as i32);
                self.merged_faces[self.merged_faces_count as usize] = opposing_face;
                self.merged_faces_count += 1;

                let next_next = self.edges[next as usize].next;
                self.edges[prev as usize].next = next_next;
                self.edges[next_next as usize].prev = prev;

                self.edges[prev as usize].twin = twin;
                self.edges[twin as usize].twin = prev;

                let drop_v = self.edges[next as usize].origin;
                self.vertex_list_remove(drop_v);

                self.retire_edge(dead_edge0);
                self.retire_edge(dead_edge1);
                self.retire_edge(dead_edge2);
            } else {
                twin = self.edges[next as usize].twin;

                if self.faces[self.edges[twin as usize].face as usize].edge
                    == self.edges[prev as usize].twin
                {
                    self.faces[self.edges[twin as usize].face as usize].edge = twin;
                }

                let prev_twin = self.edges[prev as usize].twin;
                let prev_twin_next = self.edges[prev_twin as usize].next;
                self.edges[twin as usize].next = prev_twin_next;
                self.edges[prev_twin_next as usize].prev = twin;
                self.retire_edge(prev_twin);

                let next_next = self.edges[next as usize].next;
                self.edges[prev as usize].next = next_next;
                self.edges[next_next as usize].prev = prev;

                self.edges[prev as usize].twin = twin;
                self.edges[twin as usize].twin = prev;

                let drop_v = self.edges[next as usize].origin;
                self.vertex_list_remove(drop_v);
            }

            let twin_face = self.edges[twin as usize].face;
            newell_plane(self, twin_face);
            let min_outside = self.min_outside;
            recache_conflicts(self, twin_face, min_outside);
        } else {
            self.edges[prev as usize].next = next;
            self.edges[next as usize].prev = prev;
        }
    }

    fn absorb_faces(&mut self, face: i32) {
        for i in 0..self.merged_faces_count {
            let merged = self.merged_faces[i as usize];
            debug_assert!(self.faces[merged as usize].mark == MARK_DELETE);

            let mut node = self.faces[merged as usize].conflict_list_head.next;
            while node != SENTINEL {
                let vertex = node;
                node = self.vertices[vertex as usize].link.next;

                self.conflict_remove(merged, vertex);

                let distance = plane_separation(
                    self.faces[face as usize].plane,
                    self.vertices[vertex as usize].position,
                );
                if distance > self.min_outside {
                    self.conflict_push_back(face, vertex);
                    self.vertices[vertex as usize].conflict_face = face;
                    if distance > self.faces[face as usize].max_conflict_distance {
                        self.faces[face as usize].max_conflict_distance = distance;
                        self.faces[face as usize].max_conflict = vertex;
                    }
                } else {
                    self.orphaned_list_push_back(vertex);
                    self.vertices[vertex as usize].conflict_face = NULL_INDEX;
                }
            }

            debug_assert!(list_empty(&self.faces[merged as usize].conflict_list_head));
            self.retire_face(merged);
        }
    }

    fn connect_faces(&mut self, edge: i32) {
        let face = self.edges[edge as usize].face;
        let twin = self.edges[edge as usize].twin;

        let mut edge_prev = self.edges[edge as usize].prev;
        let mut edge_next = self.edges[edge as usize].next;
        let mut twin_prev = self.edges[twin as usize].prev;
        let mut twin_next = self.edges[twin as usize].next;

        while self.edges[self.edges[edge_prev as usize].twin as usize].face
            == self.edges[twin as usize].face
        {
            debug_assert!(self.edges[edge_prev as usize].twin == twin_next);
            debug_assert!(self.edges[twin_next as usize].twin == edge_prev);
            edge_prev = self.edges[edge_prev as usize].prev;
            twin_next = self.edges[twin_next as usize].next;
        }
        debug_assert!(self.edges[edge_prev as usize].face != self.edges[twin_next as usize].face);

        while self.edges[self.edges[edge_next as usize].twin as usize].face
            == self.edges[twin as usize].face
        {
            debug_assert!(self.edges[edge_next as usize].twin == twin_prev);
            debug_assert!(self.edges[twin_prev as usize].twin == edge_next);
            edge_next = self.edges[edge_next as usize].next;
            twin_prev = self.edges[twin_prev as usize].prev;
        }
        debug_assert!(self.edges[edge_next as usize].face != self.edges[twin_prev as usize].face);

        self.faces[face as usize].edge = edge_prev;

        self.merged_faces_count = 0;
        debug_assert!(self.merged_faces_count < self.merged_faces.len() as i32);
        let twin_face = self.edges[twin as usize].face;
        self.merged_faces[self.merged_faces_count as usize] = twin_face;
        self.merged_faces_count += 1;
        self.faces[twin_face as usize].mark = MARK_DELETE;
        self.faces[twin_face as usize].edge = NULL_INDEX;

        let mut absorbed = twin_next;
        let stop = self.edges[twin_prev as usize].next;
        while absorbed != stop {
            self.edges[absorbed as usize].face = face;
            absorbed = self.edges[absorbed as usize].next;
        }

        let ep_next = self.edges[edge_prev as usize].next;
        self.destroy_edges(ep_next, edge_next);
        let tp_next = self.edges[twin_prev as usize].next;
        self.destroy_edges(tp_next, twin_next);

        self.connect_edges(edge_prev, twin_next);
        self.connect_edges(twin_prev, edge_next);

        newell_plane(self, face);
        let min_outside = self.min_outside;
        recache_conflicts(self, face, min_outside);
        self.absorb_faces(face);
    }

    fn merge_concave(&mut self, face: i32) -> bool {
        let start = self.faces[face as usize].edge;
        let mut edge = start;
        loop {
            let twin = self.edges[edge as usize].twin;
            if is_edge_concave(self, edge, self.min_radius)
                || is_edge_concave(self, twin, self.min_radius)
            {
                self.connect_faces(edge);
                return true;
            }
            edge = self.edges[edge as usize].next;
            if edge == start {
                break;
            }
        }
        false
    }

    fn merge_coplanar(&mut self, face: i32) -> bool {
        let start = self.faces[face as usize].edge;
        let mut edge = start;
        loop {
            let twin = self.edges[edge as usize].twin;
            if !is_edge_convex(self, edge, self.min_radius)
                || !is_edge_convex(self, twin, self.min_radius)
            {
                self.connect_faces(edge);
                return true;
            }
            edge = self.edges[edge as usize].next;
            if edge == start {
                break;
            }
        }
        false
    }

    fn merge_faces(&mut self) {
        for i in 0..self.cone_count {
            let face = self.cone[i as usize];
            if self.faces[face as usize].mark == MARK_VISIBLE && self.faces[face as usize].flipped {
                self.faces[face as usize].flipped = false;

                let mut best_area = 0.0f32;
                let mut best_edge = NULL_INDEX;

                let start = self.faces[face as usize].edge;
                let mut edge = start;
                loop {
                    let twin = self.edges[edge as usize].twin;
                    let area = self.faces[self.edges[twin as usize].face as usize].area;
                    if area > best_area {
                        best_area = area;
                        best_edge = edge;
                    }
                    edge = self.edges[edge as usize].next;
                    if edge == start {
                        break;
                    }
                }

                debug_assert!(best_edge != NULL_INDEX);
                self.connect_faces(best_edge);
            }
        }

        for i in 0..self.cone_count {
            let face = self.cone[i as usize];
            if self.faces[face as usize].mark == MARK_VISIBLE {
                while self.merge_concave(face) {}
            }
        }

        for i in 0..self.cone_count {
            let face = self.cone[i as usize];
            if self.faces[face as usize].mark == MARK_VISIBLE {
                while self.merge_coplanar(face) {}
            }
        }
    }

    fn resolve_vertices(&mut self) {
        let mut node = self.orphaned_list.next;
        while node != SENTINEL {
            let vertex = node;
            node = self.vertices[vertex as usize].link.next;
            self.orphaned_list_remove(vertex);

            let mut max_distance = self.min_outside;
            let mut max_face = NULL_INDEX;

            for i in 0..self.cone_count {
                let face = self.cone[i as usize];
                if self.faces[face as usize].mark == MARK_VISIBLE {
                    let distance = plane_separation(
                        self.faces[face as usize].plane,
                        self.vertices[vertex as usize].position,
                    );
                    if distance > max_distance {
                        max_distance = distance;
                        max_face = face;
                    }
                }
            }

            if max_face != NULL_INDEX {
                debug_assert!(self.faces[max_face as usize].mark == MARK_VISIBLE);
                self.conflict_push_back(max_face, vertex);
                self.vertices[vertex as usize].conflict_face = max_face;
                if max_distance > self.faces[max_face as usize].max_conflict_distance {
                    self.faces[max_face as usize].max_conflict_distance = max_distance;
                    self.faces[max_face as usize].max_conflict = vertex;
                }
            }
        }
        debug_assert!(list_empty(&self.orphaned_list));
    }

    fn resolve_faces(&mut self) {
        let mut node = self.face_list.next;
        while node != SENTINEL {
            let face = node;
            node = self.faces[face as usize].link.next;

            if self.faces[face as usize].mark == MARK_DELETE
                && list_contains(&self.faces[face as usize].link)
            {
                debug_assert!(list_empty(&self.faces[face as usize].conflict_list_head));
                self.face_list_remove(face);
            }
        }

        for i in 0..self.cone_count {
            let face = self.cone[i as usize];
            if self.faces[face as usize].mark == MARK_DELETE {
                continue;
            }
            self.face_list_push_back(face);
        }
    }

    pub fn add_vertex_to_hull(&mut self, vertex: i32) {
        let face = self.vertices[vertex as usize].conflict_face;
        self.vertices[vertex as usize].conflict_face = NULL_INDEX;
        self.conflict_remove(face, vertex);
        self.vertex_list_push_back(vertex);

        self.horizon_count = 0;
        self.build_horizon(vertex, face);
        debug_assert!(self.horizon_count >= 3);

        self.cone_count = 0;
        self.build_cone(vertex);
        debug_assert!(self.cone_count >= 3);

        self.merge_faces();
        self.resolve_vertices();
        self.resolve_faces();
    }

    pub fn clean_hull(&mut self, origin: Vec3) {
        let mut face_count = 0;
        let mut half_edge_count = 0;

        let mut face_node = self.face_list.next;
        while face_node != SENTINEL {
            let face = face_node;
            let start = self.faces[face as usize].edge;
            let mut edge = start;
            loop {
                let origin_v = self.edges[edge as usize].origin;
                self.vertices[origin_v as usize].reachable = true;
                edge = self.edges[edge as usize].next;
                half_edge_count += 1;
                if edge == start {
                    break;
                }
            }

            let n = self.faces[face as usize].plane.normal;
            self.faces[face as usize].plane.offset += dot(n, origin);
            self.faces[face as usize].centroid = add(self.faces[face as usize].centroid, origin);
            face_count += 1;
            face_node = self.faces[face as usize].link.next;
        }

        let mut vertex_count = 0;
        let mut node = self.vertex_list.next;
        while node != SENTINEL {
            let vertex = node;
            node = self.vertices[vertex as usize].link.next;

            if !self.vertices[vertex as usize].reachable {
                self.vertex_list_remove(vertex);
            } else {
                self.vertices[vertex as usize].position =
                    add(self.vertices[vertex as usize].position, origin);
                vertex_count += 1;
            }
        }

        self.interior_point = add(self.interior_point, origin);
        self.final_vertex_count = vertex_count;
        self.final_half_edge_count = half_edge_count;
        self.final_face_count = face_count;
    }

    pub fn has_hull(&self) -> bool {
        let v = self.final_vertex_count;
        let e = self.final_half_edge_count / 2;
        let f = self.final_face_count;
        v - e + f == 2 && f >= 4
    }

    pub fn construct(
        &mut self,
        points: &[Vec3],
        max_vertex_count: i32,
        origin: Vec3,
        shifted_points: &mut [Vec3],
    ) -> bool {
        if points.len() < 4 {
            return false;
        }

        for i in 0..points.len() {
            shifted_points[i] = sub(points[i], origin);
        }

        self.compute_tolerance(shifted_points);
        if !self.build_initial_hull(shifted_points) {
            return false;
        }

        let mut budget = clamp_int(max_vertex_count - 4, 0, HULL_LIMIT - 4);
        let mut vertex = self.next_conflict_vertex();
        while vertex != NULL_INDEX && budget > 0 {
            self.add_vertex_to_hull(vertex);
            vertex = self.next_conflict_vertex();
            budget -= 1;
        }

        self.clean_hull(origin);
        self.has_hull()
    }
}
