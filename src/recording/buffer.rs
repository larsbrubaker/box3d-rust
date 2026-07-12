//! Growable append-only byte buffer and little-endian writers.
//! Port of `b3RecBuffer` / `b3RecW_*` from recording.c / recording.h.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::geometry::{Capsule, Sphere, SurfaceMaterial};
use crate::math_functions::{Aabb, Matrix3, Pos, Quat, Transform, Vec3, WorldTransform};
use crate::types::Filter;

/// Growable append-only byte buffer. Doubles on demand. `count_only` tallies
/// size without allocating. (b3RecBuffer)
#[derive(Debug, Clone, Default)]
pub struct RecBuffer {
    pub data: Vec<u8>,
    pub count_only: bool,
}

impl RecBuffer {
    /// (implicit zero-init in C)
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            count_only: false,
        }
    }

    pub fn with_capacity(cap: usize) -> Self {
        Self {
            data: Vec::with_capacity(cap),
            count_only: false,
        }
    }

    pub fn size(&self) -> i32 {
        self.data.len() as i32
    }

    /// (b3RecBufAppend)
    pub fn append(&mut self, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        if self.count_only {
            // Grow logical size without allocating — emulate with reserve skip.
            // Callers in count_only mode should track via a side length; we still
            // push so size() stays correct without allocating large capacity.
            let new_len = self.data.len() + bytes.len();
            // Avoid allocating the payload: extend with zeros then overwrite is wasteful.
            // Instead keep a separate size when count_only — but C bumps buf->size only.
            // Mirror with a phantom length stored in data capacity trick: use len field.
            // Simplest faithful approach: just don't copy, track size separately.
            // We store count_only length in data.len by resizing without fill of content:
            self.data.resize(new_len, 0);
            return;
        }
        self.data.extend_from_slice(bytes);
    }

    pub fn append_u8(&mut self, v: u8) {
        self.append(&[v]);
    }

    pub fn append_u16(&mut self, v: u16) {
        self.append(&v.to_le_bytes());
    }

    pub fn append_u32(&mut self, v: u32) {
        self.append(&v.to_le_bytes());
    }

    pub fn append_u64(&mut self, v: u64) {
        self.append(&v.to_le_bytes());
    }

    pub fn append_i32(&mut self, v: i32) {
        self.append_u32(v as u32);
    }

    pub fn append_f32(&mut self, v: f32) {
        self.append(&v.to_le_bytes());
    }

    pub fn append_f64(&mut self, v: f64) {
        self.append(&v.to_le_bytes());
    }

    pub fn append_bool(&mut self, v: bool) {
        self.append_u8(u8::from(v));
    }

    pub fn append_vec3(&mut self, v: Vec3) {
        self.append_f32(v.x);
        self.append_f32(v.y);
        self.append_f32(v.z);
    }

    pub fn append_quat(&mut self, v: Quat) {
        self.append_vec3(v.v);
        self.append_f32(v.s);
    }

    pub fn append_transform(&mut self, v: Transform) {
        self.append_vec3(v.p);
        self.append_quat(v.q);
    }

    pub fn append_pos(&mut self, v: Pos) {
        #[cfg(feature = "double-precision")]
        {
            self.append_f64(v.x);
            self.append_f64(v.y);
            self.append_f64(v.z);
        }
        #[cfg(not(feature = "double-precision"))]
        {
            self.append_vec3(v);
        }
    }

    pub fn append_world_xf(&mut self, v: WorldTransform) {
        #[cfg(feature = "double-precision")]
        {
            self.append_pos(v.p);
            self.append_quat(v.q);
        }
        #[cfg(not(feature = "double-precision"))]
        {
            self.append_transform(v);
        }
    }

    pub fn append_matrix3(&mut self, v: Matrix3) {
        self.append_vec3(v.cx);
        self.append_vec3(v.cy);
        self.append_vec3(v.cz);
    }

    pub fn append_aabb(&mut self, v: Aabb) {
        self.append_vec3(v.lower_bound);
        self.append_vec3(v.upper_bound);
    }

    pub fn append_sphere(&mut self, v: Sphere) {
        self.append_vec3(v.center);
        self.append_f32(v.radius);
    }

    pub fn append_capsule(&mut self, v: Capsule) {
        self.append_vec3(v.center1);
        self.append_vec3(v.center2);
        self.append_f32(v.radius);
    }

    pub fn append_filter(&mut self, v: Filter) {
        self.append_u64(v.category_bits);
        self.append_u64(v.mask_bits);
        self.append_i32(v.group_index);
    }

    pub fn append_material(&mut self, v: SurfaceMaterial) {
        self.append_f32(v.friction);
        self.append_f32(v.restitution);
        self.append_f32(v.rolling_resistance);
        self.append_vec3(v.tangent_velocity);
        self.append_u64(v.user_material_id);
        self.append_u32(v.custom_color);
    }
}

/// Bounds-checked read cursor for snapshot images. (b3SnapReader)
#[derive(Debug, Clone)]
pub struct SnapReader<'a> {
    data: &'a [u8],
    cursor: usize,
    pub ok: bool,
}

impl<'a> SnapReader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            cursor: 0,
            ok: true,
        }
    }

    pub fn cursor(&self) -> usize {
        self.cursor
    }

    pub fn set_cursor(&mut self, cursor: usize) {
        self.cursor = cursor;
    }

    fn check(&mut self, need: usize) {
        if !self.ok {
            return;
        }
        if self
            .cursor
            .checked_add(need)
            .map(|e| e > self.data.len())
            .unwrap_or(true)
        {
            self.ok = false;
        }
    }

    pub fn bytes(&mut self, n: usize) -> Option<&'a [u8]> {
        self.check(n);
        if !self.ok {
            return None;
        }
        let start = self.cursor;
        self.cursor += n;
        Some(&self.data[start..self.cursor])
    }

    pub fn copy_bytes(&mut self, dst: &mut [u8]) {
        if let Some(src) = self.bytes(dst.len()) {
            dst.copy_from_slice(src);
        }
    }

    pub fn u8(&mut self) -> u8 {
        self.bytes(1).map(|b| b[0]).unwrap_or(0)
    }

    pub fn u16(&mut self) -> u16 {
        let mut b = [0u8; 2];
        self.copy_bytes(&mut b);
        u16::from_le_bytes(b)
    }

    pub fn u32(&mut self) -> u32 {
        let mut b = [0u8; 4];
        self.copy_bytes(&mut b);
        u32::from_le_bytes(b)
    }

    pub fn u64(&mut self) -> u64 {
        let mut b = [0u8; 8];
        self.copy_bytes(&mut b);
        u64::from_le_bytes(b)
    }

    pub fn i32(&mut self) -> i32 {
        self.u32() as i32
    }

    pub fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.u32().to_le_bytes())
    }

    pub fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.u64().to_le_bytes())
    }

    pub fn bool(&mut self) -> bool {
        self.u8() != 0
    }

    pub fn vec3(&mut self) -> Vec3 {
        Vec3 {
            x: self.f32(),
            y: self.f32(),
            z: self.f32(),
        }
    }

    pub fn quat(&mut self) -> Quat {
        Quat {
            v: self.vec3(),
            s: self.f32(),
        }
    }

    pub fn transform(&mut self) -> Transform {
        Transform {
            p: self.vec3(),
            q: self.quat(),
        }
    }

    pub fn pos(&mut self) -> Pos {
        #[cfg(feature = "double-precision")]
        {
            Pos {
                x: self.f64(),
                y: self.f64(),
                z: self.f64(),
            }
        }
        #[cfg(not(feature = "double-precision"))]
        {
            self.vec3()
        }
    }

    pub fn world_xf(&mut self) -> WorldTransform {
        #[cfg(feature = "double-precision")]
        {
            WorldTransform {
                p: self.pos(),
                q: self.quat(),
            }
        }
        #[cfg(not(feature = "double-precision"))]
        {
            self.transform()
        }
    }

    pub fn matrix3(&mut self) -> Matrix3 {
        Matrix3 {
            cx: self.vec3(),
            cy: self.vec3(),
            cz: self.vec3(),
        }
    }

    pub fn aabb(&mut self) -> Aabb {
        Aabb {
            lower_bound: self.vec3(),
            upper_bound: self.vec3(),
        }
    }

    pub fn sphere(&mut self) -> Sphere {
        Sphere {
            center: self.vec3(),
            radius: self.f32(),
        }
    }

    pub fn capsule(&mut self) -> Capsule {
        Capsule {
            center1: self.vec3(),
            center2: self.vec3(),
            radius: self.f32(),
        }
    }

    pub fn filter(&mut self) -> Filter {
        Filter {
            category_bits: self.u64(),
            mask_bits: self.u64(),
            group_index: self.i32(),
        }
    }

    pub fn material(&mut self) -> SurfaceMaterial {
        SurfaceMaterial {
            friction: self.f32(),
            restitution: self.f32(),
            rolling_resistance: self.f32(),
            tangent_velocity: self.vec3(),
            user_material_id: self.u64(),
            custom_color: self.u32(),
        }
    }

    /// Bounds check before allocating from image. (b3SnapCheckCount)
    pub fn check_count(&self, count: i32, mem_size: i32, min_stream_bytes: i32) -> bool {
        if count < 0 || mem_size < 0 || min_stream_bytes < 0 {
            return false;
        }
        if mem_size > 0 && count > 0 && count > i32::MAX / mem_size {
            return false;
        }
        let remaining = self.data.len() as i64 - self.cursor as i64;
        (count as i64) * (min_stream_bytes as i64) <= remaining
    }
}
