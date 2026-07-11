//! Mesh BVH construction: vertex welding, SAH/median splits, DFS triangle sort.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{MeshData, MeshNode, MeshTriangle, MESH_STACK_SIZE};
use crate::constants::linear_slop;
use crate::core::NULL_INDEX;
use crate::math_functions::{
    aabb_add_point, aabb_area, aabb_center, aabb_extents, aabb_union, get_by_index, major_axis, max,
    min, Aabb, Vec3, BOUNDS3_EMPTY,
};
use std::collections::HashMap;

const BIN_COUNT: i32 = 8;
const DESIRED_TRIANGLES_PER_LEAF: i32 = 4;
const MAXIMUM_TRIANGLES_PER_LEAF: i32 = 8;

#[derive(Clone, Copy)]
pub(super) struct Primitive {
    pub aabb: Aabb,
    pub center: Vec3,
    pub triangle_index: i32,
}

struct Bucket {
    count: i32,
    bounds: Aabb,
}

struct Split {
    left_bounds: Aabb,
    right_bounds: Aabb,
    axis: i32,
    index: i32,
}

#[derive(Clone, Copy)]
struct VertexNode {
    vertex_index: i32,
    next_node_index: i32,
}

struct SpatialHash<'a> {
    nodes: Vec<VertexNode>,
    vertices: &'a [Vec3],
    tolerance: f32,
    cell_size: f32,
    vertex_map: HashMap<u64, i32>,
}

fn cell_key(x: i32, y: i32, z: i32) -> u64 {
    let mut key: u64 = 0;
    key ^= (x as u64).wrapping_add(0x9e3779b9).wrapping_add(key << 6).wrapping_add(key >> 2);
    key ^= (y as u64).wrapping_add(0x9e3779b9).wrapping_add(key << 6).wrapping_add(key >> 2);
    key ^= (z as u64).wrapping_add(0x9e3779b9).wrapping_add(key << 6).wrapping_add(key >> 2);
    key
}

impl<'a> SpatialHash<'a> {
    fn create(vertices: &'a [Vec3], tolerance: f32) -> Self {
        let cell_size = 2.0 * tolerance;
        debug_assert!(cell_size > 0.0);
        Self {
            nodes: Vec::with_capacity(vertices.len()),
            vertices,
            tolerance,
            cell_size,
            vertex_map: HashMap::with_capacity(vertices.len()),
        }
    }

    fn find_duplicate(&mut self, current_index: i32) -> i32 {
        debug_assert!((current_index as usize) < self.vertices.len());
        let vertex = self.vertices[current_index as usize];
        let cell_size = self.cell_size;
        let tolerance = self.tolerance;

        let base_x = (vertex.x / cell_size).floor() as i32;
        let base_y = (vertex.y / cell_size).floor() as i32;
        let base_z = (vertex.z / cell_size).floor() as i32;

        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let x = base_x + dx;
                    let y = base_y + dy;
                    let z = base_z + dz;
                    let key = cell_key(x, y, z);

                    if let Some(&mut_node_index) = self.vertex_map.get(&key) {
                        let mut node_index = mut_node_index;
                        while node_index != NULL_INDEX {
                            let node = self.nodes[node_index as usize];
                            let existing_index = node.vertex_index;
                            debug_assert!(existing_index < current_index);
                            let other = self.vertices[existing_index as usize];
                            if (vertex.x - other.x).abs() <= tolerance
                                && (vertex.y - other.y).abs() <= tolerance
                                && (vertex.z - other.z).abs() <= tolerance
                            {
                                return existing_index;
                            }
                            node_index = node.next_node_index;
                        }
                    }
                }
            }
        }

        let current_key = cell_key(base_x, base_y, base_z);
        if let Some(head) = self.vertex_map.get_mut(&current_key) {
            let node = VertexNode {
                vertex_index: current_index,
                next_node_index: *head,
            };
            *head = self.nodes.len() as i32;
            self.nodes.push(node);
        } else {
            let node = VertexNode {
                vertex_index: current_index,
                next_node_index: NULL_INDEX,
            };
            self.vertex_map.insert(current_key, self.nodes.len() as i32);
            self.nodes.push(node);
        }

        NULL_INDEX
    }
}

/// Weld nearby vertices. Returns the unique vertex count.
/// (mesh.c: b3WeldVertices)
pub(super) fn weld_vertices(
    src_vertices: &[Vec3],
    src_indices: &[i32],
    dst_vertices: &mut [Vec3],
    dst_indices: &mut [i32],
    tolerance: f32,
) -> i32 {
    let vertex_count = src_vertices.len();
    let mut unique_count = 0i32;
    let mut spatial_hash = SpatialHash::create(src_vertices, tolerance);
    let mut vertex_mapping = vec![0i32; vertex_count];

    for i in 0..vertex_count {
        let duplicate_index = spatial_hash.find_duplicate(i as i32);
        if duplicate_index == NULL_INDEX {
            vertex_mapping[i] = unique_count;
            dst_vertices[unique_count as usize] = src_vertices[i];
            unique_count += 1;
        } else {
            vertex_mapping[i] = vertex_mapping[duplicate_index as usize];
        }
    }

    for (i, &src_index) in src_indices.iter().enumerate() {
        debug_assert!((src_index as usize) < vertex_count);
        dst_indices[i] = vertex_mapping[src_index as usize];
    }

    unique_count
}

fn split_binned_sah(count: i32, primitives: &mut [Primitive]) -> Split {
    let mut split = Split {
        left_bounds: Aabb::default(),
        right_bounds: Aabb::default(),
        axis: -1,
        index: -1,
    };

    let mut bounds = Aabb {
        lower_bound: primitives[0].center,
        upper_bound: primitives[0].center,
    };
    for i in 1..count as usize {
        bounds = aabb_add_point(bounds, primitives[i].center);
    }

    let mut best_bucket = -1i32;
    let mut best_cost = f32::MAX;

    for axis in 0..3 {
        let extent = aabb_extents(bounds);
        if get_by_index(extent, axis) < linear_slop() {
            continue;
        }

        let mut buckets: [Bucket; BIN_COUNT as usize] = std::array::from_fn(|_| Bucket {
            count: 0,
            bounds: BOUNDS3_EMPTY,
        });

        let factor = (BIN_COUNT as f32) * (1.0 - f32::EPSILON)
            / (get_by_index(bounds.upper_bound, axis) - get_by_index(bounds.lower_bound, axis));

        for i in 0..count as usize {
            let center = primitives[i].center;
            let index = (factor
                * (get_by_index(center, axis) - get_by_index(bounds.lower_bound, axis)))
                as i32;
            debug_assert!((0..BIN_COUNT).contains(&index));
            buckets[index as usize].count += 1;
            buckets[index as usize].bounds =
                aabb_union(buckets[index as usize].bounds, primitives[i].aabb);
        }

        for i in 0..BIN_COUNT - 1 {
            let mut left_count = 0i32;
            let mut left_bounds = BOUNDS3_EMPTY;
            for k in 0..=i {
                left_count += buckets[k as usize].count;
                left_bounds = aabb_union(left_bounds, buckets[k as usize].bounds);
            }

            let mut right_count = 0i32;
            let mut right_bounds = BOUNDS3_EMPTY;
            for k in (i + 1)..BIN_COUNT {
                right_count += buckets[k as usize].count;
                right_bounds = aabb_union(right_bounds, buckets[k as usize].bounds);
            }

            debug_assert!(left_count + right_count == count);
            if left_count > 0 && right_count > 0 {
                let cost = (left_count as f32) * aabb_area(left_bounds)
                    + (right_count as f32) * aabb_area(right_bounds);
                if cost < best_cost {
                    best_bucket = i;
                    best_cost = cost;
                    split.axis = axis;
                    split.index = left_count;
                    split.left_bounds = left_bounds;
                    split.right_bounds = right_bounds;
                }
            }
        }
    }

    if best_bucket >= 0 {
        let axis = split.axis;
        let factor = (BIN_COUNT as f32) * (1.0 - f32::EPSILON)
            / (get_by_index(bounds.upper_bound, axis) - get_by_index(bounds.lower_bound, axis));

        let mut split_index = 0i32;
        for i in 0..count as usize {
            let center = primitives[i].center;
            let index = (factor
                * (get_by_index(center, axis) - get_by_index(bounds.lower_bound, axis)))
                as i32;
            if index <= best_bucket {
                primitives.swap(i, split_index as usize);
                split_index += 1;
            }
        }
        debug_assert!(split_index == split.index);
    }

    split
}

fn split_half(count: i32, primitives: &[Primitive]) -> Split {
    let split_index = count / 2;

    let mut left_bounds = BOUNDS3_EMPTY;
    for i in 0..split_index as usize {
        left_bounds = aabb_union(left_bounds, primitives[i].aabb);
    }

    let mut right_bounds = BOUNDS3_EMPTY;
    for i in split_index as usize..count as usize {
        right_bounds = aabb_union(right_bounds, primitives[i].aabb);
    }

    let bounds = aabb_union(left_bounds, right_bounds);
    let axis = major_axis(aabb_extents(bounds));

    Split {
        axis,
        index: split_index,
        left_bounds,
        right_bounds,
    }
}

fn split_median(count: i32, primitives: &mut [Primitive]) -> Split {
    debug_assert!(count > 2);

    let mut lower_bound = primitives[0].center;
    let mut upper_bound = primitives[0].center;
    for i in 1..count as usize {
        lower_bound = min(lower_bound, primitives[i].center);
        upper_bound = max(upper_bound, primitives[i].center);
    }

    let d = crate::math_functions::sub(upper_bound, lower_bound);
    let c = crate::math_functions::mul_sv(0.5, crate::math_functions::add(lower_bound, upper_bound));

    let mut split = Split {
        left_bounds: Aabb::default(),
        right_bounds: Aabb::default(),
        axis: 0,
        index: -1,
    };

    let mut i1 = 0i32;
    let mut i2 = count;

    if d.x >= d.y && d.x >= d.z {
        split.axis = 0;
        let pivot = c.x;
        while i1 < i2 {
            while i1 < i2 && primitives[i1 as usize].center.x < pivot {
                i1 += 1;
            }
            while i1 < i2 && primitives[(i2 - 1) as usize].center.x >= pivot {
                i2 -= 1;
            }
            if i1 < i2 {
                primitives.swap(i1 as usize, (i2 - 1) as usize);
                i1 += 1;
                i2 -= 1;
            }
        }
    } else if d.y >= d.z {
        split.axis = 1;
        let pivot = c.y;
        while i1 < i2 {
            while i1 < i2 && primitives[i1 as usize].center.y < pivot {
                i1 += 1;
            }
            while i1 < i2 && primitives[(i2 - 1) as usize].center.y >= pivot {
                i2 -= 1;
            }
            if i1 < i2 {
                primitives.swap(i1 as usize, (i2 - 1) as usize);
                i1 += 1;
                i2 -= 1;
            }
        }
    } else {
        split.axis = 2;
        let pivot = c.z;
        while i1 < i2 {
            while i1 < i2 && primitives[i1 as usize].center.z < pivot {
                i1 += 1;
            }
            while i1 < i2 && primitives[(i2 - 1) as usize].center.z >= pivot {
                i2 -= 1;
            }
            if i1 < i2 {
                primitives.swap(i1 as usize, (i2 - 1) as usize);
                i1 += 1;
                i2 -= 1;
            }
        }
    }

    debug_assert!(i1 == i2);
    debug_assert!((0..count).contains(&i1));

    if i1 == 0 || i1 == count - 1 {
        i1 = count / 2;
    }

    let mut left_bounds = BOUNDS3_EMPTY;
    for i in 0..i1 as usize {
        left_bounds = aabb_union(left_bounds, primitives[i].aabb);
    }
    let mut right_bounds = BOUNDS3_EMPTY;
    for i in i1 as usize..count as usize {
        right_bounds = aabb_union(right_bounds, primitives[i].aabb);
    }

    split.index = i1;
    split.left_bounds = left_bounds;
    split.right_bounds = right_bounds;
    split
}

/// Build the BVH recursively. Returns the node index and writes tree height.
/// (mesh.c: b3BuildRecursive)
pub(super) fn build_recursive(
    nodes: &mut Vec<MeshNode>,
    count: i32,
    primitives: &mut [Primitive],
    base_offset: i32,
    use_median_split: bool,
    height: &mut i32,
) -> i32 {
    if count > DESIRED_TRIANGLES_PER_LEAF {
        let mut split = if use_median_split {
            split_median(count, primitives)
        } else {
            split_binned_sah(count, primitives)
        };

        if split.axis < 0 {
            if count > MAXIMUM_TRIANGLES_PER_LEAF {
                split = split_half(count, primitives);
            } else {
                let mut bounds = BOUNDS3_EMPTY;
                for i in 0..count as usize {
                    bounds = aabb_union(bounds, primitives[i].aabb);
                }
                let index = nodes.len() as i32;
                nodes.push(MeshNode::store_leaf(bounds, count, base_offset));
                return index;
            }
        }

        let index = nodes.len() as i32;
        // Placeholder; filled after children are built.
        nodes.push(MeshNode::default());

        let mut height_left = 0;
        let mut height_right = 0;
        let left_index = build_recursive(
            nodes,
            split.index,
            &mut primitives[..split.index as usize],
            base_offset,
            use_median_split,
            &mut height_left,
        );
        let right_index = build_recursive(
            nodes,
            count - split.index,
            &mut primitives[split.index as usize..],
            base_offset + split.index,
            use_median_split,
            &mut height_right,
        );

        *height = height_left.max(height_right) + 1;

        debug_assert!(left_index - index == 1 && right_index - index > 1);

        let aabb = aabb_union(split.left_bounds, split.right_bounds);
        nodes[index as usize] = MeshNode::store_internal(aabb, split.axis, right_index - index);
        return index;
    }

    let mut aabb = BOUNDS3_EMPTY;
    for i in 0..count as usize {
        aabb = aabb_union(aabb, primitives[i].aabb);
    }

    let index = nodes.len() as i32;
    nodes.push(MeshNode::store_leaf(aabb, count, base_offset));
    *height = 1;
    index
}

/// Sort triangles into DFS leaf order. Returns false if the BVH is too deep.
/// (mesh.c: b3SortMeshTriangles)
pub(super) fn sort_mesh_triangles(mesh: &mut MeshData) -> bool {
    let mut offset = 0i32;
    let mut temp_triangles = Vec::with_capacity(mesh.triangle_count as usize);
    let mut temp_material_indices = Vec::with_capacity(mesh.triangle_count as usize);

    let mut stack: [i32; MESH_STACK_SIZE] = [0; MESH_STACK_SIZE];
    let mut count = 0usize;
    stack[count] = 0;
    count += 1;

    while count > 0 {
        count -= 1;
        let node_index = stack[count] as usize;
        let is_leaf = mesh.nodes[node_index].is_leaf();

        if !is_leaf {
            if count >= MESH_STACK_SIZE - 2 {
                return false;
            }
            let child_offset = mesh.nodes[node_index].child_offset() as i32;
            stack[count] = node_index as i32 + child_offset;
            count += 1;
            stack[count] = node_index as i32 + 1;
            count += 1;
        } else {
            let triangle_count = mesh.nodes[node_index].triangle_count() as i32;
            let triangle_offset = mesh.nodes[node_index].triangle_offset as i32;
            for triangle in 0..triangle_count {
                let index = (triangle_offset + triangle) as usize;
                temp_triangles.push(mesh.triangles[index]);
                temp_material_indices.push(mesh.material_indices[index]);
            }
            mesh.nodes[node_index].triangle_offset = offset as u32;
            offset += triangle_count;
        }
    }

    debug_assert!(offset == temp_triangles.len() as i32);
    debug_assert!(temp_triangles.len() == mesh.triangle_count as usize);

    mesh.triangles.copy_from_slice(&temp_triangles);
    mesh.material_indices.copy_from_slice(&temp_material_indices);
    true
}

/// Collect primitives for BVH build from welded vertices/indices.
pub(super) fn collect_primitives(
    vertices: &[Vec3],
    indices: &[i32],
    material_indices: Option<&[u8]>,
    triangle_count: i32,
) -> (Vec<Primitive>, i32, f32, i32, Aabb) {
    let mut primitives = Vec::with_capacity(triangle_count as usize);
    let mut degenerate_count = 0i32;
    let min_area = 0.01 * linear_slop() * linear_slop();
    let mut surface_area = 0.0f32;
    let mut material_count = 1i32;
    let mut mesh_bounds = BOUNDS3_EMPTY;

    for index in 0..triangle_count {
        let index1 = indices[(3 * index) as usize];
        let index2 = indices[(3 * index + 1) as usize];
        let index3 = indices[(3 * index + 2) as usize];

        let vertex1 = vertices[index1 as usize];
        let vertex2 = vertices[index2 as usize];
        let vertex3 = vertices[index3 as usize];

        let normal = crate::math_functions::cross(
            crate::math_functions::sub(vertex2, vertex1),
            crate::math_functions::sub(vertex3, vertex1),
        );
        let area = 0.5 * crate::math_functions::length(normal);

        if area < min_area {
            if index1 != index2 && index1 != index3 && index2 != index3 {
                degenerate_count += 1;
            }
            continue;
        }

        surface_area += area;

        let box_ = Aabb {
            lower_bound: min(vertex1, min(vertex2, vertex3)),
            upper_bound: max(vertex1, max(vertex2, vertex3)),
        };
        let center = aabb_center(box_);

        primitives.push(Primitive {
            aabb: box_,
            center,
            triangle_index: index,
        });

        if let Some(mats) = material_indices {
            material_count = material_count.max(mats[index as usize] as i32 + 1);
        }

        mesh_bounds = aabb_union(mesh_bounds, box_);
    }

    (
        primitives,
        degenerate_count,
        surface_area,
        material_count,
        mesh_bounds,
    )
}

/// Fill triangle/material/flag arrays from primitives (pre-sort order).
pub(super) fn fill_triangles(
    triangles: &mut [MeshTriangle],
    material_indices: &mut [u8],
    flags: &mut [u8],
    primitives: &[Primitive],
    indices: &[i32],
    src_materials: Option<&[u8]>,
) {
    for (index, primitive) in primitives.iter().enumerate() {
        let ti = primitive.triangle_index;
        triangles[index] = MeshTriangle {
            index1: indices[(3 * ti) as usize],
            index2: indices[(3 * ti + 1) as usize],
            index3: indices[(3 * ti + 2) as usize],
        };
        flags[index] = 0;
        if let Some(mats) = src_materials {
            material_indices[index] = mats[ti as usize];
        }
    }
}
