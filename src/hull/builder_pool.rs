//! Quickhull working pools and intrusive list helpers.
//! Indices replace C pointers; `SENTINEL` is the list head, `UNLINKED` is C's NULL.

use crate::core::NULL_INDEX;
use crate::math_functions::{Plane, Vec3, VEC3_ZERO};

pub(super) const MARK_VISIBLE: i32 = 0;
pub(super) const MARK_DELETE: i32 = 1;
pub(super) const HULL_LIMIT: i32 = 255;

/// Not in a list (C NULL).
pub(super) const UNLINKED: i32 = -2;
/// Points at the list sentinel / head.
pub(super) const SENTINEL: i32 = -1;

#[derive(Clone, Copy)]
pub(super) struct ListNode {
    pub prev: i32,
    pub next: i32,
}

impl ListNode {
    pub fn unlinked() -> Self {
        Self {
            prev: UNLINKED,
            next: UNLINKED,
        }
    }

    pub fn sentinel() -> Self {
        Self {
            prev: SENTINEL,
            next: SENTINEL,
        }
    }
}

#[derive(Clone)]
pub(super) struct QhVertex {
    pub link: ListNode,
    pub conflict_face: i32, // face index or NULL_INDEX
    pub position: Vec3,
    pub final_index: i32,
    pub reachable: bool,
}

#[derive(Clone)]
pub(super) struct QhHalfEdge {
    pub prev: i32,
    pub next: i32,
    pub origin: i32,
    pub face: i32,
    pub twin: i32,
    pub final_index: i32,
}

#[derive(Clone)]
pub(super) struct QhFace {
    pub link: ListNode,
    pub edge: i32,
    pub mark: i32,
    pub area: f32,
    pub plane: Plane,
    pub centroid: Vec3,
    pub max_conflict_distance: f32,
    /// Conflict-list sentinel (only `link` is used), matching C's embedded QHVertex.
    pub conflict_list_head: ListNode,
    pub max_conflict: i32, // vertex index or NULL_INDEX
    pub final_index: i32,
    pub flipped: bool,
}

#[derive(Clone, Copy)]
pub(super) struct HorizonFrame {
    pub face: i32,
    pub start_edge: i32,
    pub edge: i32,
    pub started: bool,
}

pub(super) struct HullBuilder {
    pub tolerance: f32,
    pub min_radius: f32,
    pub min_outside: f32,
    pub interior_point: Vec3,

    pub orphaned_list: ListNode,
    pub vertex_list: ListNode,
    pub face_list: ListNode,

    pub vertices: Vec<QhVertex>,
    pub vertex_capacity: i32,

    pub edges: Vec<QhHalfEdge>,
    pub edge_capacity: i32,
    pub edge_count: i32,
    pub edge_free_head: i32,

    pub faces: Vec<QhFace>,
    pub face_capacity: i32,
    pub face_count: i32,
    pub face_free_head: i32,

    pub horizon: Vec<i32>,
    pub horizon_count: i32,

    pub cone: Vec<i32>,
    pub cone_count: i32,

    pub merged_faces: Vec<i32>,
    pub merged_faces_count: i32,

    pub horizon_stack: Vec<HorizonFrame>,

    pub final_vertex_count: i32,
    pub final_half_edge_count: i32,
    pub final_face_count: i32,
}

pub(super) struct HullWorkSizes {
    pub vertex_capacity: i32,
    pub edge_capacity: i32,
    pub face_capacity: i32,
    pub horizon_capacity: i32,
    pub cone_capacity: i32,
    pub merged_faces_capacity: i32,
    pub horizon_stack_capacity: i32,
}

pub(super) fn compute_hull_work_sizes(point_count: i32, clamped_max_count: i32) -> HullWorkSizes {
    let m = clamped_max_count;
    let mut edge_capacity = 24 * m - 48;
    if edge_capacity < 48 {
        edge_capacity = 48;
    }
    let mut face_capacity = 5 * m - 10;
    if face_capacity < 16 {
        face_capacity = 16;
    }
    let mut horizon_capacity = 3 * m - 6;
    if horizon_capacity < 6 {
        horizon_capacity = 6;
    }
    let mut merged_faces_capacity = 2 * m - 4;
    if merged_faces_capacity < 4 {
        merged_faces_capacity = 4;
    }
    let mut horizon_stack_capacity = 2 * m - 4;
    if horizon_stack_capacity < 4 {
        horizon_stack_capacity = 4;
    }

    HullWorkSizes {
        vertex_capacity: point_count + 4,
        edge_capacity,
        face_capacity,
        horizon_capacity,
        cone_capacity: horizon_capacity,
        merged_faces_capacity,
        horizon_stack_capacity,
    }
}

impl HullBuilder {
    pub fn new(s: &HullWorkSizes) -> Self {
        Self {
            tolerance: 0.0,
            min_radius: 0.0,
            min_outside: 0.0,
            interior_point: VEC3_ZERO,
            orphaned_list: ListNode::sentinel(),
            vertex_list: ListNode::sentinel(),
            face_list: ListNode::sentinel(),
            vertices: Vec::with_capacity(s.vertex_capacity as usize),
            vertex_capacity: s.vertex_capacity,
            edges: Vec::with_capacity(s.edge_capacity as usize),
            edge_capacity: s.edge_capacity,
            edge_count: 0,
            edge_free_head: NULL_INDEX,
            faces: Vec::with_capacity(s.face_capacity as usize),
            face_capacity: s.face_capacity,
            face_count: 0,
            face_free_head: NULL_INDEX,
            horizon: vec![0; s.horizon_capacity as usize],
            horizon_count: 0,
            cone: vec![0; s.cone_capacity as usize],
            cone_count: 0,
            merged_faces: vec![0; s.merged_faces_capacity as usize],
            merged_faces_count: 0,
            horizon_stack: vec![
                HorizonFrame {
                    face: 0,
                    start_edge: 0,
                    edge: 0,
                    started: false,
                };
                s.horizon_stack_capacity as usize
            ],
            final_vertex_count: 0,
            final_half_edge_count: 0,
            final_face_count: 0,
        }
    }
}

#[inline]
pub(super) fn list_empty(head: &ListNode) -> bool {
    head.next == SENTINEL
}

#[inline]
pub(super) fn list_contains(node: &ListNode) -> bool {
    node.prev != UNLINKED && node.next != UNLINKED
}

/// Insert `node_idx` before `where_link` in a list whose head is reached via `get(SENTINEL)`.
pub(super) fn list_insert_with<F>(mut get: F, node_idx: i32, where_link: i32)
where
    F: FnMut(i32) -> *mut ListNode,
{
    debug_assert!(node_idx >= 0);
    unsafe {
        let node = &mut *get(node_idx);
        debug_assert!(!list_contains(node));
        let where_node = &mut *get(where_link);
        debug_assert!(list_contains(where_node));

        let prev_link = where_node.prev;
        node.prev = prev_link;
        node.next = where_link;

        (*get(prev_link)).next = node_idx;
        (*get(where_link)).prev = node_idx;
    }
}

pub(super) fn list_remove_with<F>(mut get: F, node_idx: i32)
where
    F: FnMut(i32) -> *mut ListNode,
{
    unsafe {
        let node = &mut *get(node_idx);
        debug_assert!(list_contains(node));
        let prev = node.prev;
        let next = node.next;
        (*get(prev)).next = next;
        (*get(next)).prev = prev;
        node.prev = UNLINKED;
        node.next = UNLINKED;
    }
}

pub(super) fn list_push_back_with<F>(mut get: F, head: i32, node_idx: i32)
where
    F: FnMut(i32) -> *mut ListNode,
{
    // C: Insert(node, head->prev) — insert before current last (PushFront for iteration).
    let where_link = unsafe { (*get(head)).prev };
    list_insert_with(get, node_idx, where_link);
}

impl HullBuilder {
    pub fn new_vertex(&mut self, position: Vec3) -> i32 {
        debug_assert!(self.vertices.len() < self.vertex_capacity as usize);
        let idx = self.vertices.len() as i32;
        self.vertices.push(QhVertex {
            link: ListNode::unlinked(),
            conflict_face: NULL_INDEX,
            position,
            final_index: NULL_INDEX,
            reachable: false,
        });
        idx
    }

    pub fn new_edge(&mut self) -> i32 {
        let edge = if self.edge_free_head != NULL_INDEX {
            let e = self.edge_free_head;
            self.edge_free_head = self.edges[e as usize].next;
            e
        } else {
            debug_assert!(self.edge_count < self.edge_capacity);
            let e = self.edge_count;
            self.edge_count += 1;
            if self.edges.len() as i32 <= e {
                self.edges.push(QhHalfEdge {
                    prev: NULL_INDEX,
                    next: NULL_INDEX,
                    origin: NULL_INDEX,
                    face: NULL_INDEX,
                    twin: NULL_INDEX,
                    final_index: NULL_INDEX,
                });
            }
            e
        };
        self.edges[edge as usize].final_index = NULL_INDEX;
        edge
    }

    pub fn retire_edge(&mut self, edge: i32) {
        self.edges[edge as usize].next = self.edge_free_head;
        self.edge_free_head = edge;
    }

    pub fn new_face(&mut self, v1: i32, v2: i32, v3: i32) -> i32 {
        use crate::math_functions::{
            add, cross, dot, get_length_and_normalize, mul_sv, plane_separation, sub,
        };

        let face = if self.face_free_head != NULL_INDEX {
            let f = self.face_free_head;
            // link.next was free-list pointer
            self.face_free_head = self.faces[f as usize].link.next;
            f
        } else {
            debug_assert!(self.face_count < self.face_capacity);
            let f = self.face_count;
            self.face_count += 1;
            if self.faces.len() as i32 <= f {
                self.faces.push(QhFace {
                    link: ListNode::unlinked(),
                    edge: NULL_INDEX,
                    mark: MARK_VISIBLE,
                    area: 0.0,
                    plane: Plane {
                        normal: VEC3_ZERO,
                        offset: 0.0,
                    },
                    centroid: VEC3_ZERO,
                    max_conflict_distance: 0.0,
                    conflict_list_head: ListNode::sentinel(),
                    max_conflict: NULL_INDEX,
                    final_index: NULL_INDEX,
                    flipped: false,
                });
            }
            f
        };

        {
            let f = &mut self.faces[face as usize];
            f.link = ListNode::unlinked();
            f.max_conflict = NULL_INDEX;
            f.max_conflict_distance = 0.0;
            f.final_index = NULL_INDEX;
        }

        let edge1 = self.new_edge();
        let edge2 = self.new_edge();
        let edge3 = self.new_edge();

        let p1 = self.vertices[v1 as usize].position;
        let p2 = self.vertices[v2 as usize].position;
        let p3 = self.vertices[v3 as usize].position;

        let mut length = 0.0;
        let normal = get_length_and_normalize(&mut length, cross(sub(p2, p1), sub(p3, p1)));
        let plane = Plane {
            normal,
            offset: dot(normal, p1),
        };
        let area = 0.5 * length;
        let centroid = mul_sv(1.0 / 3.0, add(p1, add(p2, p3)));
        let flipped = plane_separation(plane, self.interior_point) > 0.0;

        {
            let f = &mut self.faces[face as usize];
            f.edge = edge1;
            f.mark = MARK_VISIBLE;
            f.area = area;
            f.centroid = centroid;
            f.plane = plane;
            f.flipped = flipped;
            f.conflict_list_head = ListNode::sentinel();
        }

        self.edges[edge1 as usize] = QhHalfEdge {
            prev: edge3,
            next: edge2,
            origin: v1,
            face,
            twin: NULL_INDEX,
            final_index: NULL_INDEX,
        };
        self.edges[edge2 as usize] = QhHalfEdge {
            prev: edge1,
            next: edge3,
            origin: v2,
            face,
            twin: NULL_INDEX,
            final_index: NULL_INDEX,
        };
        self.edges[edge3 as usize] = QhHalfEdge {
            prev: edge2,
            next: edge1,
            origin: v3,
            face,
            twin: NULL_INDEX,
            final_index: NULL_INDEX,
        };

        face
    }

    pub fn retire_face(&mut self, face: i32) {
        if list_contains(&self.faces[face as usize].link) {
            self.face_list_remove(face);
        }
        self.faces[face as usize].edge = NULL_INDEX;
        self.faces[face as usize].link.next = self.face_free_head;
        self.faces[face as usize].link.prev = UNLINKED;
        self.face_free_head = face;
    }

    pub fn vertex_list_push_back(&mut self, idx: i32) {
        let head = &mut self.vertex_list as *mut ListNode;
        let verts = self.vertices.as_mut_ptr();
        list_push_back_with(
            |link| {
                if link == SENTINEL {
                    head
                } else {
                    unsafe { &mut (*verts.add(link as usize)).link as *mut ListNode }
                }
            },
            SENTINEL,
            idx,
        );
    }

    pub fn vertex_list_remove(&mut self, idx: i32) {
        let head = &mut self.vertex_list as *mut ListNode;
        let verts = self.vertices.as_mut_ptr();
        list_remove_with(
            |link| {
                if link == SENTINEL {
                    head
                } else {
                    unsafe { &mut (*verts.add(link as usize)).link as *mut ListNode }
                }
            },
            idx,
        );
    }

    pub fn orphaned_list_push_back(&mut self, idx: i32) {
        let head = &mut self.orphaned_list as *mut ListNode;
        let verts = self.vertices.as_mut_ptr();
        list_push_back_with(
            |link| {
                if link == SENTINEL {
                    head
                } else {
                    unsafe { &mut (*verts.add(link as usize)).link as *mut ListNode }
                }
            },
            SENTINEL,
            idx,
        );
    }

    pub fn orphaned_list_remove(&mut self, idx: i32) {
        let head = &mut self.orphaned_list as *mut ListNode;
        let verts = self.vertices.as_mut_ptr();
        list_remove_with(
            |link| {
                if link == SENTINEL {
                    head
                } else {
                    unsafe { &mut (*verts.add(link as usize)).link as *mut ListNode }
                }
            },
            idx,
        );
    }

    pub fn face_list_push_back(&mut self, idx: i32) {
        let head = &mut self.face_list as *mut ListNode;
        let faces = self.faces.as_mut_ptr();
        list_push_back_with(
            |link| {
                if link == SENTINEL {
                    head
                } else {
                    unsafe { &mut (*faces.add(link as usize)).link as *mut ListNode }
                }
            },
            SENTINEL,
            idx,
        );
    }

    pub fn face_list_remove(&mut self, idx: i32) {
        let head = &mut self.face_list as *mut ListNode;
        let faces = self.faces.as_mut_ptr();
        list_remove_with(
            |link| {
                if link == SENTINEL {
                    head
                } else {
                    unsafe { &mut (*faces.add(link as usize)).link as *mut ListNode }
                }
            },
            idx,
        );
    }

    pub fn conflict_push_back(&mut self, face: i32, vertex: i32) {
        let faces = self.faces.as_mut_ptr();
        let verts = self.vertices.as_mut_ptr();
        list_push_back_with(
            |link| {
                if link == SENTINEL {
                    unsafe { &mut (*faces.add(face as usize)).conflict_list_head as *mut ListNode }
                } else {
                    unsafe { &mut (*verts.add(link as usize)).link as *mut ListNode }
                }
            },
            SENTINEL,
            vertex,
        );
    }

    pub fn conflict_remove(&mut self, face: i32, vertex: i32) {
        let faces = self.faces.as_mut_ptr();
        let verts = self.vertices.as_mut_ptr();
        list_remove_with(
            |link| {
                if link == SENTINEL {
                    unsafe { &mut (*faces.add(face as usize)).conflict_list_head as *mut ListNode }
                } else {
                    unsafe { &mut (*verts.add(link as usize)).link as *mut ListNode }
                }
            },
            vertex,
        );
    }
}

