//! Convex hull types and accessors.
//!
//! # C trailing-blob layout
//!
//! In C, `b3HullData` is a 144-byte header with vertex/point/edge/plane/face arrays plus
//! structure-of-array (SOA) vertices and normals hanging off the end at the recorded byte
//! offsets (`b3AlignUp8` between sections). `b3BoxHull` embeds the same header plus fixed
//! arrays (total 648 bytes).
//!
//! Rust stores the header fields plus owned `Vec`s for the arrays. Offsets and
//! `byte_count` are kept identical to C so [`HullData::to_bytes`] / [`BoxHull::to_bytes`]
//! reproduce the contiguous layout used by `b3Hash` and `memcmp`.
//!
//! # SOA arrays
//!
//! The SOA vertex array stores all x values, then all y values, then all z values, each
//! sub-array padded to a multiple of 4 with a repeat of the first value. The SOA normal
//! array is laid out the same way, but its padded lanes are zero. These arrays back the
//! SIMD hull collision path (`b3GetHullSoaVertices` / `b3GetHullSoaNormals`).

use crate::math_functions::{Aabb, Matrix3, Plane, Vec3, MAT3_ZERO, VEC3_ZERO};

/// 64-bit hull version. Useful for validating serialized data. (B3_HULL_VERSION)
pub const HULL_VERSION: u64 = 0xDA5150191B994C01;

/// Size of the C `b3HullData` header. (_Static_assert in hull.c)
pub const HULL_DATA_SIZE: usize = 144;

/// Size of the C `b3BoxHull`. (_Static_assert in hull.c)
pub const BOX_HULL_SIZE: usize = 648;

/// A hull vertex. Identified by a half-edge with this vertex as its tail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct HullVertex {
    /// A half-edge that has this vertex as the origin.
    pub edge: u8,
}

/// Half-edge for hull data structure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct HullHalfEdge {
    /// Next edge index CCW.
    pub next: u8,
    /// Twin edge index.
    pub twin: u8,
    /// Index of origin vertex and point.
    pub origin: u8,
    /// Face to the left of this edge.
    pub face: u8,
}

/// A hull face. Hulls use a half-edge data structure, so a face
/// can be determined from a single half-edge index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct HullFace {
    /// An arbitrary half-edge on this face.
    pub edge: u8,
}

/// A convex hull.
///
/// Maps to C's `b3HullData` + trailing blob. See module docs for the layout mapping.
#[derive(Debug, Clone)]
pub struct HullData {
    pub version: u64,
    pub byte_count: i32,
    pub hash: u32,
    pub aabb: Aabb,
    pub surface_area: f32,
    pub volume: f32,
    pub inner_radius: f32,
    pub center: Vec3,
    pub central_inertia: Matrix3,
    pub vertex_count: i32,
    pub vertex_offset: i32,
    pub point_offset: i32,
    pub edge_count: i32,
    pub edge_offset: i32,
    pub face_count: i32,
    pub plane_offset: i32,
    pub face_offset: i32,
    pub soa_vertex_offset: i32,
    pub soa_normal_offset: i32,
    pub padding: i32,
    pub vertices: Vec<HullVertex>,
    pub points: Vec<Vec3>,
    pub edges: Vec<HullHalfEdge>,
    pub faces: Vec<HullFace>,
    pub planes: Vec<Plane>,
    /// SOA vertex components: `vx[..]`, then `vy[..]`, then `vz[..]`, each length
    /// `(vertex_count + 3) & !3`. See module docs.
    pub soa_vertices: Vec<f32>,
    /// SOA normal components: `nx[..]`, then `ny[..]`, then `nz[..]`, each length
    /// `(face_count + 3) & !3`. See module docs.
    pub soa_normals: Vec<f32>,
}

impl Default for HullData {
    fn default() -> Self {
        Self {
            version: HULL_VERSION,
            byte_count: 0,
            hash: 0,
            aabb: Aabb::default(),
            surface_area: 0.0,
            volume: 0.0,
            inner_radius: 0.0,
            center: VEC3_ZERO,
            central_inertia: MAT3_ZERO,
            vertex_count: 0,
            vertex_offset: 0,
            point_offset: 0,
            edge_count: 0,
            edge_offset: 0,
            face_count: 0,
            plane_offset: 0,
            face_offset: 0,
            soa_vertex_offset: 0,
            soa_normal_offset: 0,
            padding: 0,
            vertices: Vec::new(),
            points: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            planes: Vec::new(),
            soa_vertices: Vec::new(),
            soa_normals: Vec::new(),
        }
    }
}

/// Efficient box hull with embedded arrays matching C `b3BoxHull`.
#[derive(Debug, Clone)]
pub struct BoxHull {
    pub base: HullData,
    pub box_vertices: [HullVertex; 8],
    pub box_points: [Vec3; 8],
    pub box_edges: [HullHalfEdge; 24],
    pub box_planes: [Plane; 6],
    pub box_faces: [HullFace; 6],
    pub padding: [u8; 10],
    pub vx: [f32; 8],
    pub vy: [f32; 8],
    pub vz: [f32; 8],
    pub nx: [f32; 8],
    pub ny: [f32; 8],
    pub nz: [f32; 8],
}

impl BoxHull {
    /// View the embedded arrays through the base `HullData` accessors pattern.
    pub fn as_hull_data(&self) -> HullDataView<'_> {
        HullDataView {
            header: &self.base,
            vertices: &self.box_vertices,
            points: &self.box_points,
            edges: &self.box_edges,
            faces: &self.box_faces,
            planes: &self.box_planes,
        }
    }

    /// Mirror the embedded box arrays into `base`'s owned `Vec`s so
    /// [`get_hull_points`] / [`get_hull_planes`] work on `&box.base`, matching
    /// C's contiguous `b3BoxHull` layout where offsets point into the same
    /// allocation.
    pub fn sync_base_arrays(&mut self) {
        self.base.vertices = self.box_vertices.to_vec();
        self.base.points = self.box_points.to_vec();
        self.base.edges = self.box_edges.to_vec();
        self.base.faces = self.box_faces.to_vec();
        self.base.planes = self.box_planes.to_vec();

        // SOA arrays: vx ++ vy ++ vz (soaVertexCount = 8), nx ++ ny ++ nz (soaNormalCount = 8).
        let mut soa_vertices = Vec::with_capacity(24);
        soa_vertices.extend_from_slice(&self.vx);
        soa_vertices.extend_from_slice(&self.vy);
        soa_vertices.extend_from_slice(&self.vz);
        self.base.soa_vertices = soa_vertices;

        let mut soa_normals = Vec::with_capacity(24);
        soa_normals.extend_from_slice(&self.nx);
        soa_normals.extend_from_slice(&self.ny);
        soa_normals.extend_from_slice(&self.nz);
        self.base.soa_normals = soa_normals;
    }
}

/// Borrowed view of hull geometry (owned hull or box hull).
pub struct HullDataView<'a> {
    pub header: &'a HullData,
    pub vertices: &'a [HullVertex],
    pub points: &'a [Vec3],
    pub edges: &'a [HullHalfEdge],
    pub faces: &'a [HullFace],
    pub planes: &'a [Plane],
}

/// Get hull vertices. (collision.h: b3GetHullVertices)
pub fn get_hull_vertices(hull: &HullData) -> &[HullVertex] {
    &hull.vertices[..hull.vertex_count as usize]
}

/// Get hull points. (collision.h: b3GetHullPoints)
pub fn get_hull_points(hull: &HullData) -> &[Vec3] {
    &hull.points[..hull.vertex_count as usize]
}

/// Get hull half-edges. (collision.h: b3GetHullEdges)
pub fn get_hull_edges(hull: &HullData) -> &[HullHalfEdge] {
    &hull.edges[..hull.edge_count as usize]
}

/// Get hull faces. (collision.h: b3GetHullFaces)
pub fn get_hull_faces(hull: &HullData) -> &[HullFace] {
    &hull.faces[..hull.face_count as usize]
}

/// Get hull face planes. (collision.h: b3GetHullPlanes)
pub fn get_hull_planes(hull: &HullData) -> &[Plane] {
    &hull.planes[..hull.face_count as usize]
}

/// SOA vertex sub-array length: `vertex_count` padded up to a multiple of 4.
pub fn hull_soa_vertex_count(hull: &HullData) -> usize {
    ((hull.vertex_count + 3) & !3) as usize
}

/// SOA normal sub-array length: `face_count` padded up to a multiple of 4.
pub fn hull_soa_normal_count(hull: &HullData) -> usize {
    ((hull.face_count + 3) & !3) as usize
}

/// Get read only SOA vertices. This is an array of vertices with all x values,
/// y values, and z values as separate sub-arrays. The sub-array lengths are padded
/// to a multiple of 4. The padded values are repeats of the first value.
/// (collision.h: b3GetHullSoaVertices)
pub fn get_hull_soa_vertices(hull: &HullData) -> &[f32] {
    &hull.soa_vertices
}

/// Get read only SOA unit normal vectors. This is an array of normals with all x values,
/// y values, and z values as separate sub-arrays. The sub-array lengths are padded to
/// a multiple of 4. The padded values are zero.
/// (collision.h: b3GetHullSoaNormals)
pub fn get_hull_soa_normals(hull: &HullData) -> &[f32] {
    &hull.soa_normals
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

fn write_u64_le(buf: &mut Vec<u8>, v: u64) {
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

fn write_matrix3(buf: &mut Vec<u8>, m: Matrix3) {
    write_vec3(buf, m.cx);
    write_vec3(buf, m.cy);
    write_vec3(buf, m.cz);
}

fn write_plane(buf: &mut Vec<u8>, p: Plane) {
    write_vec3(buf, p.normal);
    write_f32_le(buf, p.offset);
}

fn write_header(buf: &mut Vec<u8>, h: &HullData, hash_override: Option<u32>) {
    write_u64_le(buf, h.version);
    write_i32_le(buf, h.byte_count);
    write_u32_le(buf, hash_override.unwrap_or(h.hash));
    write_aabb(buf, h.aabb);
    write_f32_le(buf, h.surface_area);
    write_f32_le(buf, h.volume);
    write_f32_le(buf, h.inner_radius);
    write_vec3(buf, h.center);
    write_matrix3(buf, h.central_inertia);
    write_i32_le(buf, h.vertex_count);
    write_i32_le(buf, h.vertex_offset);
    write_i32_le(buf, h.point_offset);
    write_i32_le(buf, h.edge_count);
    write_i32_le(buf, h.edge_offset);
    write_i32_le(buf, h.face_count);
    write_i32_le(buf, h.plane_offset);
    write_i32_le(buf, h.face_offset);
    write_i32_le(buf, h.soa_vertex_offset);
    write_i32_le(buf, h.soa_normal_offset);
    write_i32_le(buf, h.padding);
    debug_assert_eq!(buf.len(), HULL_DATA_SIZE);
}

fn pad_to(buf: &mut Vec<u8>, len: usize) {
    if buf.len() < len {
        buf.resize(len, 0);
    }
}

fn read_u32_le(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
}

fn read_i32_le(buf: &[u8], off: usize) -> i32 {
    read_u32_le(buf, off) as i32
}

fn read_u64_le(buf: &[u8], off: usize) -> u64 {
    u64::from_le_bytes(buf[off..off + 8].try_into().unwrap())
}

fn read_f32_le(buf: &[u8], off: usize) -> f32 {
    f32::from_le_bytes(buf[off..off + 4].try_into().unwrap())
}

fn read_vec3(buf: &[u8], off: usize) -> Vec3 {
    Vec3 {
        x: read_f32_le(buf, off),
        y: read_f32_le(buf, off + 4),
        z: read_f32_le(buf, off + 8),
    }
}

fn read_aabb(buf: &[u8], off: usize) -> Aabb {
    Aabb {
        lower_bound: read_vec3(buf, off),
        upper_bound: read_vec3(buf, off + 12),
    }
}

fn read_matrix3(buf: &[u8], off: usize) -> Matrix3 {
    Matrix3 {
        cx: read_vec3(buf, off),
        cy: read_vec3(buf, off + 12),
        cz: read_vec3(buf, off + 24),
    }
}

fn read_plane(buf: &[u8], off: usize) -> Plane {
    Plane {
        normal: read_vec3(buf, off),
        offset: read_f32_le(buf, off + 12),
    }
}

/// Restore a hull from a contiguous blob. (inverse of [`HullData::to_bytes`])
pub fn convert_bytes_to_hull(bytes: &[u8]) -> Option<HullData> {
    if bytes.len() < HULL_DATA_SIZE {
        return None;
    }
    let version = read_u64_le(bytes, 0);
    if version != HULL_VERSION {
        return None;
    }
    let byte_count = read_i32_le(bytes, 8);
    if byte_count < HULL_DATA_SIZE as i32 || bytes.len() != byte_count as usize {
        return None;
    }
    let hash = read_u32_le(bytes, 12);
    let aabb = read_aabb(bytes, 16);
    let surface_area = read_f32_le(bytes, 40);
    let volume = read_f32_le(bytes, 44);
    let inner_radius = read_f32_le(bytes, 48);
    let center = read_vec3(bytes, 52);
    let central_inertia = read_matrix3(bytes, 64);
    let vertex_count = read_i32_le(bytes, 100);
    let vertex_offset = read_i32_le(bytes, 104);
    let point_offset = read_i32_le(bytes, 108);
    let edge_count = read_i32_le(bytes, 112);
    let edge_offset = read_i32_le(bytes, 116);
    let face_count = read_i32_le(bytes, 120);
    let plane_offset = read_i32_le(bytes, 124);
    let face_offset = read_i32_le(bytes, 128);
    let soa_vertex_offset = read_i32_le(bytes, 132);
    let soa_normal_offset = read_i32_le(bytes, 136);
    let padding = read_i32_le(bytes, 140);

    if vertex_count < 0 || edge_count < 0 || face_count < 0 {
        return None;
    }
    let vc = vertex_count as usize;
    let ec = edge_count as usize;
    let fc = face_count as usize;

    let mut vertices = Vec::with_capacity(vc);
    let voff = vertex_offset as usize;
    if voff + vc > bytes.len() {
        return None;
    }
    for i in 0..vc {
        vertices.push(HullVertex {
            edge: bytes[voff + i],
        });
    }

    let mut points = Vec::with_capacity(vc);
    let poff = point_offset as usize;
    if poff + vc * 12 > bytes.len() {
        return None;
    }
    for i in 0..vc {
        points.push(read_vec3(bytes, poff + i * 12));
    }

    let mut edges = Vec::with_capacity(ec);
    let eoff = edge_offset as usize;
    if eoff + ec * 4 > bytes.len() {
        return None;
    }
    for i in 0..ec {
        let o = eoff + i * 4;
        edges.push(HullHalfEdge {
            next: bytes[o],
            twin: bytes[o + 1],
            origin: bytes[o + 2],
            face: bytes[o + 3],
        });
    }

    let mut faces = Vec::with_capacity(fc);
    let foff = face_offset as usize;
    if foff + fc > bytes.len() {
        return None;
    }
    for i in 0..fc {
        faces.push(HullFace {
            edge: bytes[foff + i],
        });
    }

    let mut planes = Vec::with_capacity(fc);
    let ploff = plane_offset as usize;
    if ploff + fc * 16 > bytes.len() {
        return None;
    }
    for i in 0..fc {
        planes.push(read_plane(bytes, ploff + i * 16));
    }

    let soa_vertex_count = (vertex_count as usize + 3) & !3;
    let mut soa_vertices = Vec::with_capacity(3 * soa_vertex_count);
    let svoff = soa_vertex_offset as usize;
    if svoff + 3 * soa_vertex_count * 4 > bytes.len() {
        return None;
    }
    for i in 0..3 * soa_vertex_count {
        soa_vertices.push(read_f32_le(bytes, svoff + i * 4));
    }

    let soa_normal_count = (face_count as usize + 3) & !3;
    let mut soa_normals = Vec::with_capacity(3 * soa_normal_count);
    let snoff = soa_normal_offset as usize;
    if snoff + 3 * soa_normal_count * 4 > bytes.len() {
        return None;
    }
    for i in 0..3 * soa_normal_count {
        soa_normals.push(read_f32_le(bytes, snoff + i * 4));
    }

    Some(HullData {
        version,
        byte_count,
        hash,
        aabb,
        surface_area,
        volume,
        inner_radius,
        center,
        central_inertia,
        vertex_count,
        vertex_offset,
        point_offset,
        edge_count,
        edge_offset,
        face_count,
        plane_offset,
        face_offset,
        soa_vertex_offset,
        soa_normal_offset,
        padding,
        vertices,
        points,
        edges,
        faces,
        planes,
        soa_vertices,
        soa_normals,
    })
}

impl HullData {
    /// Serialize to the C contiguous trailing-blob layout (for hash / memcmp parity).
    pub fn to_bytes(&self) -> Vec<u8> {
        self.to_bytes_with_hash(self.hash)
    }

    /// Like [`to_bytes`], but with an explicit hash field (use 0 when computing the hash).
    pub fn to_bytes_with_hash(&self, hash: u32) -> Vec<u8> {
        let mut buf = Vec::with_capacity(self.byte_count as usize);
        write_header(&mut buf, self, Some(hash));
        pad_to(&mut buf, self.vertex_offset as usize);
        for v in &self.vertices {
            buf.push(v.edge);
        }
        pad_to(&mut buf, self.point_offset as usize);
        for p in &self.points {
            write_vec3(&mut buf, *p);
        }
        pad_to(&mut buf, self.edge_offset as usize);
        for e in &self.edges {
            buf.push(e.next);
            buf.push(e.twin);
            buf.push(e.origin);
            buf.push(e.face);
        }
        pad_to(&mut buf, self.plane_offset as usize);
        for p in &self.planes {
            write_plane(&mut buf, *p);
        }
        pad_to(&mut buf, self.face_offset as usize);
        for f in &self.faces {
            buf.push(f.edge);
        }
        pad_to(&mut buf, self.soa_vertex_offset as usize);
        for v in &self.soa_vertices {
            write_f32_le(&mut buf, *v);
        }
        pad_to(&mut buf, self.soa_normal_offset as usize);
        for n in &self.soa_normals {
            write_f32_le(&mut buf, *n);
        }
        pad_to(&mut buf, self.byte_count as usize);
        debug_assert_eq!(buf.len(), self.byte_count as usize);
        buf
    }
}

impl BoxHull {
    /// Serialize to the C `b3BoxHull` layout (440 bytes).
    pub fn to_bytes(&self) -> Vec<u8> {
        self.to_bytes_with_hash(self.base.hash)
    }

    pub fn to_bytes_with_hash(&self, hash: u32) -> Vec<u8> {
        let mut buf = Vec::with_capacity(BOX_HULL_SIZE);
        write_header(&mut buf, &self.base, Some(hash));
        for v in &self.box_vertices {
            buf.push(v.edge);
        }
        for p in &self.box_points {
            write_vec3(&mut buf, *p);
        }
        for e in &self.box_edges {
            buf.push(e.next);
            buf.push(e.twin);
            buf.push(e.origin);
            buf.push(e.face);
        }
        for p in &self.box_planes {
            write_plane(&mut buf, *p);
        }
        for f in &self.box_faces {
            buf.push(f.edge);
        }
        buf.extend_from_slice(&self.padding);
        for v in &self.vx {
            write_f32_le(&mut buf, *v);
        }
        for v in &self.vy {
            write_f32_le(&mut buf, *v);
        }
        for v in &self.vz {
            write_f32_le(&mut buf, *v);
        }
        for n in &self.nx {
            write_f32_le(&mut buf, *n);
        }
        for n in &self.ny {
            write_f32_le(&mut buf, *n);
        }
        for n in &self.nz {
            write_f32_le(&mut buf, *n);
        }
        debug_assert_eq!(buf.len(), BOX_HULL_SIZE);
        buf
    }
}
