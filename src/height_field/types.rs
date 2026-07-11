//! Height field types and blob serialization.
//!
//! Maps C's `b3HeightFieldData` header + trailing arrays (heights, materials, flags).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::math_functions::{Aabb, Vec3, VEC3_ZERO};

/// This material index designates holes in a height field. (B3_HEIGHT_FIELD_HOLE)
pub const HEIGHT_FIELD_HOLE: u8 = 0xFF;

/// 64-bit height-field version. (B3_HEIGHT_FIELD_VERSION)
pub const HEIGHT_FIELD_VERSION: u64 = 0x8B18CBD138A6BC84;

/// Size of the C `b3HeightFieldData` header (explicit padding, no unnamed gaps).
pub const HEIGHT_FIELD_DATA_SIZE: usize = 88;

/// Triangle mesh edge flags. (b3MeshEdgeFlags)
pub const CONCAVE_EDGE1: i32 = 0x01;
pub const CONCAVE_EDGE2: i32 = 0x02;
pub const CONCAVE_EDGE3: i32 = 0x04;
pub const INVERSE_CONCAVE_EDGE1: i32 = 0x10;
pub const INVERSE_CONCAVE_EDGE2: i32 = 0x20;
pub const INVERSE_CONCAVE_EDGE3: i32 = 0x40;

/// Data used to create a height field. (b3HeightFieldDef)
#[derive(Debug, Clone)]
pub struct HeightFieldDef {
    /// Grid point heights; length = count_x * count_z.
    pub heights: Vec<f32>,
    /// Grid cell material; length = (count_x - 1) * (count_z - 1).
    /// A value of [`HEIGHT_FIELD_HOLE`] marks a hole.
    pub material_indices: Vec<u8>,
    /// The height field scale. All components must be positive.
    pub scale: Vec3,
    /// Number of grid lines along the x-axis.
    pub count_x: i32,
    /// Number of grid lines along the z-axis.
    pub count_z: i32,
    /// Global minimum height for quantization (unscaled).
    pub global_minimum_height: f32,
    /// Global maximum height for quantization (unscaled).
    pub global_maximum_height: f32,
    /// Clock-wise winding (inverts the height-field along y).
    pub clockwise_winding: bool,
}

impl Default for HeightFieldDef {
    fn default() -> Self {
        Self {
            heights: Vec::new(),
            material_indices: Vec::new(),
            scale: VEC3_ZERO,
            count_x: 0,
            count_z: 0,
            global_minimum_height: 0.0,
            global_maximum_height: 0.0,
            clockwise_winding: false,
        }
    }
}

/// A height field with compressed storage.
///
/// Maps to C's `b3HeightFieldData` + trailing blob. Offsets and `byte_count` match
/// C so [`HeightFieldData::to_bytes`] reproduces the contiguous layout used by
/// `b3Hash`.
#[derive(Debug, Clone)]
pub struct HeightFieldData {
    pub version: u64,
    pub byte_count: i32,
    pub hash: u32,
    pub aabb: Aabb,
    pub min_height: f32,
    pub max_height: f32,
    pub height_scale: f32,
    pub scale: Vec3,
    pub column_count: i32,
    pub row_count: i32,
    pub heights_offset: i32,
    pub material_offset: i32,
    pub flags_offset: i32,
    pub clockwise: bool,
    pub padding: [u8; 3],
    pub compressed_heights: Vec<u16>,
    pub material_indices: Vec<u8>,
    pub flags: Vec<u8>,
}

impl Default for HeightFieldData {
    fn default() -> Self {
        Self {
            version: HEIGHT_FIELD_VERSION,
            byte_count: 0,
            hash: 0,
            aabb: Aabb::default(),
            min_height: 0.0,
            max_height: 0.0,
            height_scale: 0.0,
            scale: VEC3_ZERO,
            column_count: 0,
            row_count: 0,
            heights_offset: 0,
            material_offset: 0,
            flags_offset: 0,
            clockwise: false,
            padding: [0; 3],
            compressed_heights: Vec::new(),
            material_indices: Vec::new(),
            flags: Vec::new(),
        }
    }
}

/// Compressed height array. (b3GetHeightFieldCompressedHeights)
pub fn get_height_field_compressed_heights(hf: &HeightFieldData) -> &[u16] {
    &hf.compressed_heights
}

/// Material index array. (b3GetHeightFieldMaterialIndices)
pub fn get_height_field_material_indices(hf: &HeightFieldData) -> &[u8] {
    &hf.material_indices
}

/// Triangle flag array. (b3GetHeightFieldFlags)
pub fn get_height_field_flags(hf: &HeightFieldData) -> &[u8] {
    &hf.flags
}

/// Triangle count for a height field. (b3GetHeightFieldTriangleCount)
pub fn get_height_field_triangle_count(hf: &HeightFieldData) -> i32 {
    2 * (hf.column_count - 1) * (hf.row_count - 1)
}

fn write_u64_le(buf: &mut Vec<u8>, v: u64) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn write_u32_le(buf: &mut Vec<u8>, v: u32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn write_i32_le(buf: &mut Vec<u8>, v: i32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn write_f32_le(buf: &mut Vec<u8>, v: f32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn write_vec3(buf: &mut Vec<u8>, v: Vec3) {
    write_f32_le(buf, v.x);
    write_f32_le(buf, v.y);
    write_f32_le(buf, v.z);
}

fn write_aabb(buf: &mut Vec<u8>, a: Aabb) {
    write_vec3(buf, a.lower_bound);
    write_vec3(buf, a.upper_bound);
}

fn pad_to(buf: &mut Vec<u8>, len: usize) {
    if buf.len() < len {
        buf.resize(len, 0);
    }
}

fn write_header(buf: &mut Vec<u8>, h: &HeightFieldData, hash_override: Option<u32>) {
    write_u64_le(buf, h.version);
    write_i32_le(buf, h.byte_count);
    write_u32_le(buf, hash_override.unwrap_or(h.hash));
    write_aabb(buf, h.aabb);
    write_f32_le(buf, h.min_height);
    write_f32_le(buf, h.max_height);
    write_f32_le(buf, h.height_scale);
    write_vec3(buf, h.scale);
    write_i32_le(buf, h.column_count);
    write_i32_le(buf, h.row_count);
    write_i32_le(buf, h.heights_offset);
    write_i32_le(buf, h.material_offset);
    write_i32_le(buf, h.flags_offset);
    buf.push(u8::from(h.clockwise));
    buf.extend_from_slice(&h.padding);
    debug_assert_eq!(buf.len(), HEIGHT_FIELD_DATA_SIZE);
}

impl HeightFieldData {
    /// Serialize to the C contiguous trailing-blob layout (for hash parity).
    pub fn to_bytes(&self) -> Vec<u8> {
        self.to_bytes_with_hash(self.hash)
    }

    /// Like [`to_bytes`], but with an explicit hash field (use 0 when computing the hash).
    pub fn to_bytes_with_hash(&self, hash: u32) -> Vec<u8> {
        let mut buf = Vec::with_capacity(self.byte_count as usize);
        write_header(&mut buf, self, Some(hash));
        pad_to(&mut buf, self.heights_offset as usize);
        for h in &self.compressed_heights {
            buf.extend_from_slice(&h.to_le_bytes());
        }
        pad_to(&mut buf, self.material_offset as usize);
        buf.extend_from_slice(&self.material_indices);
        pad_to(&mut buf, self.flags_offset as usize);
        buf.extend_from_slice(&self.flags);
        pad_to(&mut buf, self.byte_count as usize);
        debug_assert_eq!(buf.len(), self.byte_count as usize);
        buf
    }
}
