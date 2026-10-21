# Changelog

All notable changes to box3d-rust are documented here. This project adheres to
semantic versioning (0.x: minor-compatible additive changes bump the patch number).

## Unreleased

Syncing the port to newer upstream Box3D commits, one upstream commit at a
time. Ported so far:

- `51f056e` SAT benchmark (#177)
- `e77352c` Add support for AVX2 and runtime feature detection (#174)
- `16f7f4c` Tree perf (#179)
- `f8c4fe8` Shape cast support for box movers (#180)
- `003a3f7` Fix recording crash on compounds (#181)
- `3d9db79` Various recording fixes (#182)

### Breaking

- Hulls store the dot product of the two adjacent face normals for each full
  edge (upstream `51f056e`). `HullData` gains `edge_cosine_offset`,
  `edge_cosines`, and an explicit `padding` field; `BoxHull` gains
  `edge_cosines: [f32; 12]` (left zero, as upstream never fills them for
  boxes). The serialized layout grows: the `HullData` header is 152 bytes
  (was 144), `HULL_BYTE_COUNT_OFFSET` is 144, `BoxHull` is 696 bytes (was
  640), and `HULL_VERSION` changes to `0x9D3E61B7C2A4F085`. Hull hashes
  change, and serialized hulls, baked compounds, and recordings that embed
  hulls from earlier versions no longer load.
- Hull SOA vertex and normal arrays are padded to a multiple of 8 (was 4) so
  upstream's AVX2 kernels can load full lanes (upstream `e77352c`,
  `B3_HULL_SOA_PADDING`). New `HULL_SOA_PADDING` and `get_hull_soa_stride`;
  `hull_soa_vertex_count` / `hull_soa_normal_count` now return the 8-padded
  stride. Hulls whose vertex or face count is not a multiple of 8 grow, and
  `HULL_VERSION` changes again to `0x6B1E39D4A87C25F3`. Hull hashes change
  (box hulls included, through the version), and hull blobs from earlier
  versions no longer load.
- `core::set_stall_threshold` / `core::get_stall_threshold` are removed, as
  upstream removed `b3SetStallThreshold` / `b3GetStallThreshold` and the CCD
  stall logging (upstream `16f7f4c`). The demo's Continuous / Stall readout
  goes with them.
- The wide contact solver types change shape (upstream `16f7f4c`).
  `contact_solver::wide::ContactConstraintWide` keeps the per-lane body data
  and gains `manifold_count`; the per-manifold data moves to the new
  `ContactManifoldWide` (with `rolling_mask`), and a color's constraints live
  in `WideContactConstraints` (constraints, manifolds, manifold starts). The
  convex kernels are replaced by kernels shared with mesh contacts:
  `warm_start_contacts_w`, `push_contacts_w`, `solve_contacts_w`,
  `apply_restitution_w`, `store_impulses_w`, plus
  `prepare_contacts_mesh_wide`. `make_matrix_from_quat_w` is removed.

### Added

- `get_hull_edge_cosines` (upstream `b3GetHullEdgeCosines`).
- `core::is_avx2_available` (upstream `b3IsAVX2Available`). It returns false:
  the port implements the scalar path, matching a `BOX3D_DISABLE_SIMD` build.
- `world_enable_sse2_fallback` (upstream `b3World_EnableSSE2Fallback`). The
  world's SIMD width (crate-internal, like upstream's `world->simdWidth`) is
  always 4 in the port, so the call has no effect on the simulation. Internally the port also gains upstream's
  `b3GetSIMDWidth` / `b3SetSIMDWidth`, `b3ComputeSeparatingAxisAtWidth`, and
  `b3CollideHullsAtWidth`; contacts collide hulls at the world's width.
- `shape::compute_shape_min_extent` (upstream `b3ComputeShapeMinExtent`):
  the radius for spheres and capsules, the inner radius for hulls, and the
  `compute_shape_extent` minimum otherwise.

### Changed

- The separating axis test culls edge candidates with upstream's inscribed
  sphere test `b3TestEdgeCandidateSorted` (replacing `b3TestEdgeCandidate`),
  using per-hull plane separations of the other hull's center and the stored
  edge cosines. With early return it re-culls the candidates against the
  support points of the best face axes when there are enough edge pair
  tests, and it reuses the seed face separation instead of recomputing it.
  Contact manifolds are unchanged on 30,000 randomized hull pairs; the
  early-return query can report a different (non-winning) edge axis.
  Checked bit-exact against a scalar C build of `51f056e` on those pairs.
- The wide contact solver blends the clamped rolling resistance impulse to
  +0 in lanes without rolling resistance instead of scaling it by zero, which
  produced -0 for negative impulses (upstream `e77352c`). Stored rolling
  impulses can change from -0 to +0, which can change later results bitwise
  in scenes that mix rolling and non-rolling contacts. The loops over the hull
  SOA arrays in the separating axis test and `find_hull_support_vertex` keep
  running to the count rounded up to 4, so their results are unchanged.
  Checked against a scalar C build of `e77352c` with upstream's new rolling
  mix determinism scene (both precision modes).
- Graph-color mesh contacts are solved by the wide (four lane) contact
  solver instead of the scalar mesh kernels (upstream `16f7f4c`). Each color's
  mesh contacts are sorted by manifold count (stable, largest first), grouped
  four at a time, and each group is padded to the largest manifold count in
  it. Only overflow contacts still use the scalar kernels. The wide warm start,
  restitution, and rolling resistance now work per manifold; restitution
  computes the current separation from the normal rotated into each body frame
  (like push and solve) instead of rotating the anchors.
- The overflow (scalar) contact warm start sums the impulses of all manifolds
  and applies them to the bodies once, instead of applying every point
  impulse in turn.
- Mesh and height field contacts transform the cached triangles into the
  shape frame four at a time and, for hulls, cull triangles whose bounds plus
  the speculative distance miss the hull bounds before the narrow phase,
  resetting their SAT cache. `MAX_MESH_CONTACT_TRIANGLES` grows from 256 to
  512, and the per-contact triangle buffers move to the heap (C moved them to
  the step arena).
- `query_mesh` tests the root bounds once and then tests both children before
  descending, pushing only overlapping children. The triangle visit order is
  unchanged.
- Continuous collision skips non-bullet shapes that cannot tunnel
  (`b3IsShapeFast`): the centroid travel plus the rotation chord times the
  shape radius must exceed the body's `safety_factor` times the shape's
  minimum extent. The continuous fallback radius uses
  `compute_shape_min_extent`.
- Simulation results change. The determinism goldens move to upstream's new
  values: falling ragdolls 267 / `0x9BCA269D` (double 266 / `0x68FC7728`),
  wave pile 270 / `0xAD7C3901` (double 312 / `0xFD6BD36B`), mesh drop 218 /
  `0x0A23CFA2` (double 218 / `0x26F9A566`). Query spawn and the rolling mix
  hash are unchanged. Adds upstream's mesh mix scene (`SingleMeshMixTest`)
  with hashes `0xC24D812B` (double `0x0151E8CA`). All values were confirmed
  with a scalar C build of `16f7f4c` (`BOX3D_DISABLE_SIMD`) in both precision
  modes.
- `shape_cast` with `can_encroach` no longer reports an initial overlap when
  the shapes start separated but within the cast target distance and no more
  than `2 * linear_slop` apart (upstream `f8c4fe8`, for box movers). It
  advances by first order while retaining half a linear slop of separation:
  sweeps that cannot consume the free gap (receding, tangential, grazing)
  miss, as do hits at or beyond `max_fraction`; otherwise it reports a hit at
  that fraction with the frame A point and normal. Gaps over
  `2 * linear_slop` still reduce the target as before, and overlapping or
  touching shapes still report an initial overlap. Ports upstream's eight new
  shape cast tests. Checked bit-exact against a scalar C build of `f8c4fe8`
  on 200,000 randomized near-contact casts (about 53,000 reach the new
  branch).
- `GeometryRegistry::intern_compound` scrubs the compound through
  `convert_compound_to_bytes`, mirroring upstream's `b3RecInternCompound`
  (upstream `003a3f7`). Upstream previously nulled only `tree.nodes`, so C
  recordings embedded the live `tree.proxies` pointer and its hash, which
  broke registry dedup during keyframe capture (an assert in
  `b3RecCaptureKeyframe`). The port always wrote every tree pointer as zero,
  so its recorded bytes are unchanged and now match C exactly: the interned
  blob and hash of a capsule plus three spheres compound were confirmed
  against a scalar C build of `003a3f7`. Adds tests for the pointer-free blob
  and for a seek across a live compound that mirrors the keyframe-policy call
  upstream added to `AllOps`. The port's player seeks by restart-and-replay
  and captures no keyframes, so that test checks seek and replay only.
- Replay checks the kind of every geometry registry slot it resolves
  (upstream `3d9db79`, `b3RecGetSlot`). The seed snapshot path (`des_shapes`,
  upstream `b3DesShapes`) used to check only the range and reinterpret a
  slot of the wrong kind, so a relabeled hull, mesh, height field, or
  compound slot restored the seed; now `RecPlayer::create` returns `None`.
  The shape op dispatch already checked kinds; its messages now match C
  ("`<kind> geometryId <id> is invalid`", "`<kind> geometry <id> is
  corrupt`"). New `recording::registry::get_slot`; `RegistrySlot::
  ensure_compound` is renamed `get_live_compound` (`b3RecGetLiveCompound`).
- Loading the geometry registry follows upstream's `b3RecLoadSlots`: a block
  past the end of the file, a count that cannot fit, a truncated or out of
  bounds entry, or a compound entry smaller than `b3CompoundData` (new in
  `3d9db79`) fails `RecPlayer::create` with upstream's message. The port
  used to stop loading silently and keep going, so a compound entry with a
  shortened length loaded and the following bytes were parsed as more
  entries. The registry and tag table are now bounded by the header's
  `registryByteCount`, as in C. The loader moves to
  `recording::registry_load` (`load_slots`, `load_tags`). Two deliberate
  divergences: the bounds are checked in `u64` where C narrows the header
  offsets with `(int)` casts first, and a failed `RecPlayer::create`
  restores the previous length scale where C leaves the recording's scale
  set globally.
- `RegistrySlot::kind` is now the raw `u8` kind byte, as C stores it. An
  unknown kind (above 3) matches no `get_slot` request and fails the shape
  op with "`<kind> geometryId <id> is invalid`"; the port used to treat it as
  a compound.
- The query tag table loads like upstream's `b3RecLoadTags`: a count that
  cannot fit the remaining bytes (18 per tag) loads no tags instead of the
  ones that fit, names end at the first NUL, and `RecPlayer::resolve_tag`
  returns the last tag loaded for a duplicate key (C's tag map replaces on
  insert). Names that are not valid UTF-8 are still converted lossily, the
  one remaining difference from C's raw `char` buffer.
- Ports upstream's `CorruptCompound` test and the `AllOps` seek addition
  (seek to frames 6 and 9, ray cast the compound). Both, plus a new test that
  relabels each seeded registry slot and one that sets a compound slot's kind
  to 7, were checked against a scalar C build
  of `3d9db79`: same messages, same failures. `b3CreateCompound` now copies
  the tree header fields explicitly instead of copying the whole struct and
  fixing it up; the port always wrote that header this way, and the interned
  compound blob and hash are unchanged against C.

### Not ported

- The new GJK-versus-SAT benchmark (`shared/sat_benchmark.c`, `benchmark
  --sat`) and its sample, plus the other samples-only changes (ImGuizmo).
  The Rust benches do not mirror the C benchmark harness.
- `b3World_GetCounters` now reports `byteCount` as the world memory
  footprint (`b3GetWorldByteCount`, factored out of
  `b3World_DumpMemoryStats`). `Counters::byte_count` stays zero in the port.
- The SSE2, AVX2, and Neon kernels (`simd_sse2.h`, `simd_avx2.h`,
  `simd_neon.h`, `wide8.c`), CPUID based AVX2 detection, the 32 byte
  allocation alignment, the benchmark `--simd-width` option, and the
  sample's SSE2 fallback checkbox. The port keeps the scalar width 4 path.
  Upstream's `SIMDWidthTest` and `CrossWidthReplay` skip when the native width
  is 4; the port runs both widths anyway (both are 4), which reduces them to
  repeatability checks.
- The new mesh drop benchmark (`CreateMeshDropBenchmark`, `benchmark/main.c`,
  `sample_benchmark.cpp`; listed as planned in the demo registry) and the
  refreshed benchmark CSVs. The width 8
  (`W8`) mesh kernels and `b3MoveMaskW4` for SSE2 / Neon are SIMD-only.
- The new Encroach Cast collision sample (upstream `f8c4fe8`); it is listed
  as planned in the demo registry.
- The compound pointer scrub in `b3RecSeedKeyframeRegistry` (upstream
  `003a3f7`). The port's player has no keyframe ring or keyframe registry
  (`seek_frame` restarts from frame 0 and replays), so there is nothing to
  seed.
- The `b3RecPlayer_GetKeyframeBytes( player ) > 0` check upstream added to
  `AllOps` (upstream `3d9db79`); the port's player captures no keyframes.
- The `b3Free` zero-size fix in `core.c` (upstream `3d9db79`). The port has
  no custom allocator hooks.
- The rename of `recording_replay.c` / `.h` to `replay.c` / `.h` and the
  `CMakeLists.txt` reorder (upstream `3d9db79`) have no Rust counterpart
  beyond updated doc comments; the Rust module names stay as they were.

## 0.4.1

Bug-fix release; no public API changes. The C reference pin is unchanged
(`9f998c8`).

### Fixed

- `body_set_awake(world, id, false)` on a body whose island had a pending
  split used the island id read before `split_island` ran, so
  `try_sleep_island` was handed a stale island id and tripped
  a `debug_assert` panic. The body's island id is now re-read after the
  split, matching C `b3Body_SetAwake`. Reported by @KennethWerner in #2.

## 0.4.0

This release syncs the port to upstream Box3D `9f998c8` and makes `World`
`Send` (#1). The C reference pin advances to `9f998c8` by way of `30c67b5`
(upstream v0.2.0: 64-bit rapidhash geometry hashes and triangle manifold
collision fixes), and this port follows its API and behavior. The sync ran one
upstream commit at a time. Simulation results are not bit-compatible with
0.3.0. Ported:

- `47d7f7c` clockwise option, joint is awake (#130)
- `954cf87` Fixes (#157)
- `f555ee4` Optimize Broad-Phase (#159)
- `9e5a4cd` Optimizations (#161)
- `79fc559` Optimizations (#168)
- `8fda30e` (samples only, nothing to port)
- `adec25b` SAT optimizations (#170)
- `5643cd8` Remove compound mesh child material limit (#172)
- `9f998c8` SAT samples (#173)

### Breaking

- Sensor visitors must now be convex (upstream `b3IsConvex` check); mesh,
  height-field, and compound shapes no longer generate sensor overlaps as
  visitors.
- `MAX_SHAPE_CAST_POINTS` now equals `MAX_HULL_VERTICES` (128, previously 64).
- Recording format minor version advances to 4. Older recordings still
  replay (the loader gates on major version only), but recordings that use
  the new shape-mutator ops are not readable by 0.3.0.
- Geometry content hashes are now 64-bit rapidhash: the `HullData`,
  `MeshData`, and `HeightFieldData` hash fields are `u64`, struct byte
  layouts and version constants changed (`BoxHull` 648 -> 640 bytes), and
  serialized geometry from older versions no longer loads.
- `core::non_zero_hash` and `recording::hash64_blob` are removed;
  `hash_hull_data` returns the content hash directly.
- Shape creation now rejects geometry with mismatched version constants at
  runtime.
- The world now owns an active recording (#1).
  `world_start_recording(world, recording: Recording)` and
  `recording::start_recording` take the `Recording` by value, and
  `world_stop_recording(world)` / `recording::stop_recording` return the
  finalized `Option<Recording>`. The world previously held a raw pointer to
  a caller-owned `Recording`, which was unsound if the caller moved or dropped
  it mid-session. `World::recording` is now
  `Option<Box<RefCell<Recording>>>`.
- Shared hull geometry is now `Arc<HullData>` instead of `Rc<HullData>`
  (`ShapeGeometry::Hull`, the world hull database).
- Recording format major version advances to 5 (upstream `47d7f7c`): recorded
  mover planes carry the new `PlaneResult` triangle, child, and material
  indices. Recordings from major version 4 no longer replay.
- `PlaneResult` gains `triangle_index`, `child_index`, and `material_index`,
  and `MeshDef` gains `stride`, `vertex_bytes`, and `clock_wise_winding`
  (upstream `47d7f7c`). Struct literals without `..Default::default()` must
  add the new fields.
- `get_length_and_normalize` now treats vectors with squared length at or
  below `1000 * f32::MIN_POSITIVE` as zero (was length below `f32::EPSILON`)
  and writes a zero length on failure, matching upstream.
- Recording format major version advances to 6 (upstream `954cf87`): the
  recorded body def carries `safety_factor`, and the new op `0x3B`
  `BodySetSafetyFactor` is recorded and replayed. Recordings from major
  version 5 no longer replay. The world snapshot version advances to 3
  (`Body::safety_factor`); older snapshots are rejected.
- `BodyDef` gains `safety_factor` (default 0.5) and `Body` gains
  `safety_factor` (upstream `954cf87`). Struct literals without
  `..Default::default()` must add the field.
- The shared human ragdoll (`human` module) is rewritten to match upstream
  `954cf87`: the `Spine02` and `Head` bones are gone (the neck bone carries the
  head capsule), `BoneId` values are renumbered, `BONE_COUNT` is 12,
  `FILTER_JOINT_COUNT` is 9, and `create_human` now creates three filter
  joints (thighs, neck and each upper arm). Bone frames, capsules, per-bone
  friction, joint limits, and foot capsules on the calves all changed.
- The dynamic tree is rewritten to upstream `f555ee4`'s layout: `TreeNode` is
  the 32-byte `{ aabb, flag_index, height }` node (siblings paired at even
  indices, root at 0, index 1 always empty), and category bits and user data
  live in the new `TreeProxy` array. `DYNAMIC_TREE_VERSION` is now
  `0x1D6F4C2A73B80E91`, which also changes `COMPOUND_VERSION`; baked compounds
  and saved trees from earlier versions no longer load. The `b3TreeNodeFlags`
  constants (`ALLOCATED_NODE`, `ENLARGED_NODE`, the old `LEAF_NODE`) are
  replaced by the `MOVED_NODE` / `LEAF_NODE` / `EMPTY_NODE` flag-index bits.
- `DynamicTree::validate_no_enlarged` is renamed `validate_no_moved`
  (b3DynamicTree_ValidateNoMoved), and `BroadPhase::validate_no_enlarged` is
  renamed `validate_no_moved`. `DynamicTree::node_count` and `root_index` are
  removed (use `node_end` and `ROOT_NODE`). `user_data` / `aabb` now resolve
  the proxy id through the proxy array, so proxy ids are no longer node
  indices.
- `TreeNodeView` (from `DynamicTree::node_views`) now describes the sibling
  pair layout: `left_child`, `proxy_id`, `user_data`, `is_leaf`, `is_empty`,
  `is_moved`, and covers only nodes below `node_end`.
- `BroadPhase` drops `moved_proxies`, `move_array`, `buffer_move`, and
  `enlarge_proxy`; moves are marks in the trees (`mark_proxy_moved`,
  `mark_proxy_moved_serial`). `TaskContext::enlarged_sim_bit_set` is replaced
  by `TaskContext::pair_keys`, the shape flag `ENLARGED_AABB` is removed, and
  `World` gains `compound_shape_count`.
- The baked compound blob gains `proxy_offset` and stores only the live node
  range plus the proxy array (header 144 -> 184 bytes).
- Recording format minor version advances to 6 and the world snapshot version
  to 5 (upstream `f555ee4`): snapshots store the tree node end, pair free
  list, proxy capacity and free list, DFS order flag, nodes and parents up to
  the node end, the proxies, and `World::compound_shape_count`; the move
  buffer is gone. Older snapshots are rejected.
- Shape fat AABBs move out of `Shape` into the parallel `World::fat_aabbs`
  array (upstream `9e5a4cd`); `Shape::fat_aabb` is removed and
  `create_shape_proxy` now takes the world and a shape id.
- `Contact::body_sim_index_a/b` are replaced by the persistent
  `encoded_body_sim_a/b` locators (awake: local index, static:
  `-(local index + 2)`, otherwise `NULL_INDEX`), maintained by the new
  `refresh_body_contact_indices` and the wake, sleep, transfer, destroy, and
  graph paths. New helpers `encode_body_sim_index`, `is_static_sim_index`,
  `decode_awake_index`, and `sleep_body_sim_index`.
- `body_flags::ENLARGE_BOUNDS` is renamed `ENLARGE_BULLET_BOUNDS` and is only
  set for bullets; `IS_FAST` is no longer a transient body flag and only lives
  on `BodySim`.
- The restitution solver stage is gone: `RESTITUTION_ITERATIONS`,
  `Profile::apply_restitution`, `apply_restitution`, and
  `apply_restitution_convex` are removed, and `ContactConstraint::restitution`
  is removed. `ManifoldConstraintPoint::relative_velocity` becomes
  `restitution_velocity`. The contact kernels split into `push_contacts` /
  `push_contacts_convex` (biased) and `solve_contacts` /
  `solve_contacts_convex` (relax with deferred restitution); none take a
  `use_bias` flag. `prepare_color_contacts` takes the contacts mutably.
- `Counters::byte_count` is `i64` and is now the first field (upstream
  `b3Counters` layout). The port has no allocator hooks, so upstream's
  `b3AllocFcn` / `b3FreeFcn` / `b3GetByteCount` signature changes have no Rust
  counterpart.
- Recording format minor version advances to 7 and the world snapshot version
  to 7 (upstream `9e5a4cd`): snapshots store `World::fat_aabbs` after the
  shapes and keep the contact body sim locators. Older snapshots are rejected.
- The same-step restitution from `9e5a4cd` is replaced by a post-solve
  restitution stage again (upstream `79fc559`): `ManifoldConstraintPoint`
  carries `relative_velocity` and `restitution_impulse` (instead of
  `restitution_velocity`) and `ContactConstraint` regains `restitution`.
  `apply_restitution` (Mesh/overflow) returns with a new signature taking the
  restitution threshold and the propagation flag, and `StepContext` loses
  `restitution_threshold` (the solver reads the world's value, like C).
- Graph-color convex contacts are solved by a four lane wide constraint
  (`contact_solver::wide`, `ContactConstraintWide`) that runs the scalar
  `b3FloatW` path, replacing the per-lane `solve_contacts_convex` /
  `push_contacts_convex` / `warm_start_contacts_convex` on `ContactConstraint`.
  `prepare_color_contacts` is replaced by `prepare_mesh_contacts` and
  `wide::prepare_contacts_convex`. The `b3FloatW` helpers move from
  `manifold` to the crate-level `simd_scalar` module.
- `world_rebuild_static_tree` is now crate-internal (upstream removed
  `b3World_RebuildStaticTree` from the public API); the recording op still
  replays it.
- Recording format minor version advances to 8 (upstream `79fc559`) with the
  new ops `0x0E` `WorldSetRestitutionIterations` and `0x0F`
  `WorldEnableRestitutionPropagation`. The world snapshot version advances to
  10: the world config stores `restitution_iterations` and the restitution
  propagation flag (bit `0x10`). Older snapshots are rejected.
- Compound mesh children no longer have a material limit (upstream
  `5643cd8`). `MAX_COMPOUND_MESH_MATERIALS` is removed. `CompoundCapsule`,
  `CompoundHull`, and `CompoundSphere` carry a `u16` `material_index` plus a
  `u16` `padding` field (was `i32`). `CompoundMesh::material_indices` and
  `ChildShape::material_indices` are now `&[u16]` slices into the compound
  with a `material_count` field (was a fixed `[i32; 4]`), and
  `compute_mesh_manifolds` takes its material map as `Option<&[u16]>`.
- The baked compound blob layout changes (upstream `5643cd8`): the convex
  child and hull instance material indices are `u16` + padding, mesh
  instances store a `material_offset` / `material_count` pair (60 -> 52
  bytes) into a packed `u16` mesh material map stored after the mesh
  instance array, and `COMPOUND_VERSION` changes (seed
  `0x7A2F41C9E6D0B358`). Baked compounds from earlier versions no longer
  load.
- The compatibility re-exports in `contact_solver::wide` are removed:
  `contact_solver::wide::SIMD_WIDTH` (use `core::SIMD_WIDTH`) and the wide
  vec3 helpers `Vec3W`, `add_vw`, `sub_vw`, `cross_w`, `dot_w`, `mul_svw`,
  `mul_add_svw`, and `mul_sub_svw`, which moved to the crate-internal
  `simd_scalar` module with upstream `9f998c8`.

### Changed

- Mover collision against meshes and height fields is one sided (upstream
  `47d7f7c`): `collide_mover_and_mesh` / `collide_mover_and_height_field`
  skip triangles whose back faces the mover center, respecting mirrored mesh
  scale and clockwise height fields. Mesh and height-field shape casts cull
  back-facing triangles the same way.
- Mover planes report the mesh or height-field triangle and its material.
  Compound movers report the child index and remap the child material through
  the compound material table; `collide_mover` clamps the material index to
  the shape's material count.
- `shape_distance` fills `normal`, `distance`, `iterations`, and
  `simplex_count` on its overlap early outs, builds witness points before the
  normal check, and only writes the simplex cache on success.
- `time_of_impact` reports an averaged hit point and normal when the shapes
  start overlapped.
- Height-field casts swap the clockwise cell corners once up front, as
  upstream now does, instead of choosing vertices and normals per triangle.
  Clockwise ray-cast triangles are geometrically unchanged; the second
  triangle's normal is computed from different edges, so it may differ in the
  last bits.
- `World` is now `Send`, so a world can be built on one thread and moved to
  another (#1).
- Simplified face-vs-edge contact selection in the hull-capsule and
  triangle-capsule manifolds, matching upstream.
- Continuous collision uses each body's `safety_factor` (upstream `954cf87`)
  instead of the fixed 0.5 when deciding whether a body is fast.
- Contact recycling is skipped for any contact touching a fast body (upstream
  `954cf87`); previously only fast mesh contacts skipped it.
- The broad phase rebuilds the dynamic and kinematic trees every step, even
  when no proxy moved (upstream `954cf87`). The empty-tree validation moved
  from `solve` into `World::step`, where C finishes the tree task.
- `reduce_manifold_points` and the mesh cluster reduction follow upstream's
  `B3_MAX_MANIFOLD_POINTS == 4` split: the default keeps the fast
  approximate reduction, and a larger `MAX_MANIFOLD_POINTS` would build a
  2D hull and simplify it to the target.
- The human ragdoll rewrite changes the ragdoll and wave-pile determinism
  scenes; both precision modes match upstream's new golden hashes.
- New broad-phase pair finder (upstream `f555ee4`, ported from Box2D): the
  dynamic tree collides against itself through its moved sibling pairs and
  against the static and kinematic trees through a seeded tree-against-tree
  traversal. Candidates are batched, culled against the pair set, and
  collected as pair keys that are sorted before contacts are created, so
  contact creation order follows shape order instead of tree structure. The
  ragdoll, wave-pile, and query-spawn goldens change and match upstream in
  both precision modes.
- The tree rebuild puts the whole tree in DFS order and keeps unmoved
  subtrees; the solver refits the kinematic and dynamic trees with a sweep
  over the moved marks instead of enlarging proxies one by one. Bullets only
  enlarge a proxy when the tree box no longer contains the shape's fat box.
- Same-step restitution (upstream `9e5a4cd`): the bounce is armed in prepare
  from the pre-solve normal velocity and solved inside the relax pass, which
  sweeps the normal constraints once per manifold point while a bounce is
  armed and retires it once every bouncing point separates. The convex path
  makes the sweep count and the rolling resistance decision per group of four
  contacts, like upstream's wide constraints. `ManifoldPoint::total_normal_impulse`
  now only accumulates in the relax pass and includes the restitution impulse.
- `ManifoldPoint::normal_velocity` is only computed when hit events are
  enabled on the contact (zero otherwise), and is written during prepare.
- Continuous collision returns the gravity lost to the time of impact (the
  fraction of the step cut short), respecting linear motion locks.
- Contacts cache `base_separation` only while touching.
- The narrow phase resolves body sims through the contact locators and tests
  proxy overlap on `World::fat_aabbs`.
- Debug draw contact forces no longer halve the total normal impulse.
- Finalize clears the transient flags on the body and sim up front and only
  copies speed capping back to the body; `HAD_TIME_OF_IMPACT` stays on the
  sim.
- The ragdoll and mesh-drop determinism goldens change and match upstream in
  both precision modes.
- Post-solve Poisson kinetic restitution (upstream `79fc559`): after the
  sub-steps, a restitution stage runs `World::restitution_iterations` times
  (overflow, then each color's wide and mesh contacts) when any contact has
  restitution. A bounce is armed from the sampled pre-solve normal velocity
  and limited by the compression impulse, so it never gains energy. With
  `enable_restitution_propagation` the stage also sweeps contacts without
  restitution. The relax pass is back to a single sweep.
- The wide convex prepare gathers straight into the lanes (perpendicular
  tangents, symmetric inverse inertia and rolling mass, friction impulse
  projected onto the tangents), the wide warm start applies one accumulated
  impulse per body, and push/solve compute separation with normals rotated
  into body space, matching upstream's operation order.
- Contact recycling zeroes the manifold point `normal_velocity`.
- `HAD_TIME_OF_IMPACT` is set on the body sim when continuous collision
  moves the body (not per query hit), survives `sync_body_flags`, and debug
  draw reads it from the sim. The continuous query skips bullets by the body
  flag.
- `DynamicTree::rebuild(true)` (full build) keeps the moved marks, so a
  static rebuild between a static mutation and the next step no longer loses
  the pending pairs.
- Joints between two non-dynamic bodies go to the static set when a body is
  enabled; the prismatic and weld prepare asserts accept either body awake.
- The ragdoll, wave-pile, and query-spawn determinism goldens change and
  match upstream in both precision modes.
- Hull-vs-hull SAT culling (upstream `adec25b`): the separating axis query
  bounds each candidate axis by the hull inner radii and center distance,
  seeds a separation floor from the face best aligned with the center
  delta, skips faces that cannot beat it, and pre-culls edge pairs whose
  Gauss map arcs cannot reach the best separation (returning early when
  either hull has no candidates). When face clipping produces no points,
  `collide_hulls` reruns a full query for the best edge pair.
  `find_hull_support_vertex` scans the SoA vertices four lanes at a time
  and breaks ties toward the lowest vertex index. Verified bit-identical to
  the C reference on randomized hull pairs in both precision modes;
  determinism goldens are unchanged.
- SAT revision (upstream `9f998c8`): the `adec25b` full-query fallback in
  `collide_hulls` is removed; an empty face contact now switches to the edge
  contact only when the edge separation beats both face separations (or the
  clip separation by the linear slop). The edge candidate test
  (`test_edge_candidate`, was `arc_can_reach`) drops the center-distance
  term. The scalar edge phase gathers each B edge from the transformed SoA
  arrays and uses `dot` ordering, and its Gauss map tolerance is
  `-linear_slop^2` (was `-0.0001`). Wide dot products use `b3DotW` order
  `(x + y) + z` (the SoA transform, face dots, and support search), and the
  support search starts its minimum at infinity. Verified bit-identical to the
  C reference on randomized hull pairs in both precision modes.
- The wide vec3 type and its helpers (`Vec3W`, `dot_w`, `cross_w`, `add_vw`,
  `sub_vw`, `mul_svw`, `mul_add_svw`, `mul_sub_svw`, plus the new `load_vw`,
  `store_vw`, `splat_vw`, `neg_vw`) now live in `simd_scalar` alongside the
  other `simd.h` helpers, matching upstream; `contact_solver::wide`
  re-exports the ones it used. `dot3_w` (b3Dot3W) is removed.
- The wave-pile determinism golden changes and matches upstream in both
  precision modes (float: sleep step 314, hash `0x8CB8F656`; double: hash
  `0x03B509B6`).

### Fixed

- `should_bodies_collide` now rejects pairs where neither body is dynamic,
  like upstream's `b3ShouldBodiesCollide` (found by the new broad-phase
  stress test).
- Disabling a kinematic body attached to a static body by a joint no longer
  trips the joint set assertion (upstream issue #164).
- Overflow-color contacts now flag hit events, like upstream's
  `b3StoreImpulses_Overflow` (the port only flagged graph-color contacts).
- Body extents (upstream `47d7f7c`): capsule and sphere `max_extent` now
  bound the shape about the center of mass on every axis (the capsule used a
  signed max, the sphere subtracted the local center twice), and mesh extents
  measure from the local center instead of the body origin.
- Compound creation caps each mesh child at `MAX_COMPOUND_MESH_MATERIALS`
  materials instead of indexing past the child material table.
- Replayed-mesh material-index deserialization read the wrong element count
  (one index per triangle is correct); replayed mesh contacts previously
  panicked.
- The same bug in the compound deserializer's duplicated nested-mesh reader:
  multi-material mesh children restored from bytes panicked on first contact.
  The duplicated readers are gone; nested blobs now go through the standalone
  converters.
- Corrupt or kind-mismatched geometry in a recording file now fails the
  replay gracefully instead of panicking (a deliberate, documented divergence
  from C, which trusts the blob).
- Triangle manifold collision fixes from upstream: plane-side test at exact
  zero, the capsule-edge parallel-skip bug, corrected capsule contact points,
  and an early return when no valid edge contact exists. Simulation results
  change; determinism hashes match upstream's new constants in both
  precision modes.
- Restoring a snapshot over a live world (recording seek / re-seed) leaked the
  hull-database entries of the overwritten hull shapes: the release cloned the
  shape's `Arc` first, so the reference-count check never matched. The shape's
  hull reference is now moved out and released, as C's
  `b3FreeLiveSimElements` does.
- Tearing down a world now finalizes an active recording, matching
  `b3DestroyWorld` calling `b3StopRecordingInternal`: use the new
  `world::destroy_world` (see Added). Merely dropping a `World` still
  discards an active recording unfinalized.
- `RecPlayer` now destroys its replay world the way C does: dropping the
  player (b3DestroyPlayer) and a failed snapshot seed in `RecPlayer::create`
  go through `destroy_world`, so the host's `destroy_debug_shape` fires for
  every live replayed shape at teardown.
- Snapshot restore no longer releases live shapes a second time inside the
  shape deserializer; the single release in `deserialize_into_shell` matches
  C's `b3DesShapes` / `b3FreeLiveSimElements`.
- Creating a joint with `collide_connected` false now destroys the existing
  contacts between the two bodies first (upstream `954cf87`, box3d issue
  #117). Previously those contacts survived until the joint's collide flag was
  toggled.
- Changing a body's mass data (`body_set_mass_data`, `body_apply_mass_from_shapes`,
  and shape changes that update mass) now invalidates the cached relative
  transform on the body's contacts, so a recycled manifold is never reused
  after the center of mass moves (upstream `954cf87`).
- A proxy enlarged in one step and destroyed before the next no longer leaves
  enlarged tree nodes behind (upstream `954cf87`, box3d issue #149).
- `overlap_compound` now brings the world-space proxy into the compound frame
  before querying the compound tree and children, so compounds on moved or
  rotated bodies report correct overlaps (upstream `954cf87`). The compound
  shape cast builds the child-local input with `make_local_proxy` and
  `inv_rotate_vector` and keeps `can_encroach`.
- The compound sphere array now uses the C `b3CompoundSphere` stride of 20
  bytes (it used the 32 byte capsule stride), so compounds with more than one
  sphere child round-trip through `convert_compound_to_bytes` /
  `convert_bytes_to_compound` and the blob is byte-identical to C.
- Casts, mover collision, contacts, and `get_shape_user_material_id` on a
  compound mesh child with more than four materials now resolve the
  triangle's own material instead of clamping to the fourth (upstream
  `5643cd8`).
- `convert_bytes_to_compound` rejects (returns `None`) blobs whose mesh
  material runs fall outside the packed map section, whose mesh instances
  have fewer map entries than their mesh's materials, or whose map entries
  or convex child material indices fall outside the compound material
  table, so corrupt compounds fail the conversion instead of panicking in
  casts, mover collision, or contacts. C trusts the blob here.
- The wasm demo crate now builds with `double-precision` enabled
  (`cargo check --workspace --all-targets --features double-precision`, where
  workspace feature unification turns large world mode on for the demo's
  box3d-rust dependency). The demo passes and receives world positions as
  `Pos` / `WorldTransform` (widening f32 inputs, narrowing to f32 only for
  render data) instead of assuming they are `f32`. CI now checks the whole
  workspace in both precision modes.
- `demo/build-wasm.ps1` and `scripts/pre-commit-check.ps1` are now pure ASCII,
  so Windows PowerShell 5.1 (which reads BOM-less scripts as ANSI) parses
  them. A non-ASCII em dash had broken `bun run build:wasm` on 5.1.

### Added

- Upstream `79fc559` API: `WorldDef::restitution_iterations` (default 2) and
  `enable_restitution_propagation`, `world_set_restitution_iterations` /
  `world_get_restitution_iterations` (clamped to the new
  `MAX_RESTITUTION_ITERATIONS` = 63), `world_enable_restitution_propagation` /
  `world_is_restitution_propagation_enabled`, and `Profile::restitution`.
- Upstream `47d7f7c` API: `body_time_of_impact_mover` with `BodyToiResult`
  (b3Body_TimeOfImpactMover), `body_get_min_extent`, `body_get_max_extent`,
  `body_get_max_extent_origin`, and `joint_is_awake`.
- Mesh creation reads interleaved vertex buffers (`MeshDef::vertex_bytes`
  read through `MeshDef::stride`) and accepts clockwise input
  (`MeshDef::clock_wise_winding`). The stride applies only to
  `vertex_bytes`; contiguous `vertices` require a stride of 0 or 12. Like C,
  a non-zero stride below 12 bytes, above 4096, or not a multiple of 4 is
  rejected.
- Ported upstream tests: `test_mesh.c` (7), body `ShapeExtents`, body query
  `MoverTOI*` (11), height-field backside and clockwise casts (4), and mover
  material / one sided collision (10).
- `world::destroy_world(world: World) -> Option<Recording>` (b3DestroyWorld):
  consumes the world, finalizes and returns any active recording, then
  releases every live shape's allocations (hull-database references and
  `destroy_debug_shape` host handles).
- `RecPlayer::set_debug_shape_callbacks` (b3RecPlayer_SetDebugShapeCallbacks):
  destroys the current replay world, rebuilds frame 0 under the new
  debug-shape callbacks, and rewinds the player.

- New `rapidhash` module and `core::hash64_non_zero`, with the `test_hash.c`
  suite ported.
- `ShapeSetMeshMaterial`, `ShapeSetHull`, and `ShapeSetMesh` are now recorded
  and replayed.
- New tests: overlap hull proxy, transformed box hull, capsule face-deep
  manifolds, and geometry mutator replay.
- New "Class Ring" demo scene in the Bodies category.
- Upstream `954cf87` API: `body_set_safety_factor` / `body_get_safety_factor`
  (b3Body_SetSafetyFactor / GetSafetyFactor), `core::get_max_manifold_points`
  (b3GetMaxManifoldPoints), and the 2D hull used for manifold simplification:
  `hull::Point2D`, `hull::hull_2d`, and `hull::simplify_hull_2d`
  (b3Point2D, b3Hull2D, b3SimplifyHull2D).
- Ported upstream `954cf87` tests: 2D hull and simplification (6), compound
  shape cast / material remap / transformed overlap / rotated-child mover (7),
  body query `OverlapCompoundRotatedBody`, joint
  `TestCreateJointClearsContacts`, manifold `MaxPointCountTest` with the
  `ENSURE_FACE_POINTS` range check, world `TestEnlargedProxyDestroyed`, the
  safety-factor additions to the recording `AllOps` session (as a focused
  round-trip test), and the determinism `EnsureRepeatable` /
  `ENSURE_GOLDEN` structure.
- Upstream `f555ee4` tree API: `DynamicTree::save` / `load` (b3DynamicTree_Save
  / Load, byte-compatible with the C file format) plus `save_to_bytes` /
  `load_from_bytes`, `refit`, `mark_proxy_moved`, `mark_proxy_moved_serial`,
  `clear_moved`, `gather_moved_proxies`, `create_proxy_internal`,
  `move_proxy_internal`, `has_tree_moved`, `needs_rebuild`, `node_end`,
  `dfs_ordered`, `proxy_capacity`, and `table::HashSet::contains_hashed_key`.
- Ported upstream `f555ee4` tests: `test_dynamic_tree.c` (8), world
  `TestCompoundShapeCount` and the reworked `TestEnlargedProxyDestroyed`
  (`CountMovedNodes`), and the compound tree asserts.
- Ported upstream `5643cd8` tests: compound `CompoundMeshManyMaterials` and
  `CompoundMeshContactMaterial`, and the five-material compound mesh child
  addition to the recording `AllOps` session (as a focused round-trip test).
- `create_complex_hull(radius)` (b3CreateComplexHull, upstream `9f998c8`): the
  hull of 32 pseudo random points on a sphere from PEEL's convex pile.

## 0.3.0

Upstream sync: the C reference pin advances to `c52908c` and this port follows
its API and behavior. Simulation results are not bit-compatible with 0.2.x.
This version was never published to crates.io.

### Breaking

- `create_compound_shape` is now `create_baked_compound_shape` (upstream
  baked-compound rename, box3d #77).
- `collide_sphere_and_triangle` / `collide_capsule_and_triangle` /
  `collide_hull_and_triangle` are now `collide_triangle_and_sphere` /
  `collide_triangle_and_capsule` / `collide_triangle_and_hull`, matching the
  upstream argument-order flip.
- `BODY_NAME_LENGTH` / `SHAPE_NAME_LENGTH` (18) are replaced by
  `MAX_NAME_LENGTH` (256) and `MAX_QUERY_NAME_LENGTH` (64).
- `edge_edge_separation` was removed by the upstream edge-edge optimization
  port (box3d #63).
- Recording/snapshot formats advanced (`REC_VERSION` 4.3, `SNAP_VERSION` 2,
  new `HULL_VERSION`); recordings from 0.2.x do not replay.

### Changed

- Upstream behavior ports: ghost-collision improvements (#61), edge-edge
  optimization (#63), friction center weighted average (#71), SIMD hull
  collision scalar path (#93), contact and manifold fixes (#94), hull
  stacking (#97) and hull builder (#98) fixes.

### Added

- `DebugShapeCallbacks` test port, with a replay debug-shape release fix.
- Demo parity batches syncing the sample app to `c52908c` (new samples,
  Jenga rework, determinism scenes split into modules).
- 30 new tests; suite green in both precision modes (332 / 333 with
  `double-precision`).

## 0.2.1

No public API or behavioral changes; existing code upgrades without modification.

- Added unit tests for the mesh creation path (degenerate-triangle filtering and
  vertex weld / identify-edges via `create_mesh`).
- Release-profile tuning (fat LTO, single codegen unit) for in-repo builds.
- Demo site: zoom-to-cursor camera navigation, InstancedMesh sample coverage,
  `M` diagnostics drawer with full debug-draw flags, and Mesh Viewer /
  benchmark fidelity fixes matching the C reference.

## 0.2.0

### Breaking

- `CustomFilterFcn` is now `fn(&World, ShapeId, ShapeId, u64) -> bool` (previously
  `fn(ShapeId, ShapeId, u64) -> bool`), matching C's `b3CustomFilterFcn`, whose
  callback can read shape state mid-step.
  **Migration:** add `&World` as the first parameter of your filter callback. The
  world reference gives callbacks the same shape-state access C callbacks have,
  e.g. `shape_get_user_data(world, id)`.

### Added

- `shape_set_mesh` (`b3Shape_SetMesh` port) to replace the mesh geometry of an
  existing mesh shape.
- `set_stall_threshold` / `get_stall_threshold` (`b3Set/GetStallThreshold` port).
- Tests for the above.

### Demos

- All 20 sample categories now render at full fidelity or honestly-disclosed
  partial coverage. Camera right-drag fix, plus arrow-key aliases for navigation.

## 0.1.2

Additive-only public API and tooling; no behavioral changes to existing APIs.

- Ported `clone_and_transform_hull` (`hull.c:2265`) in `src/hull/create.rs`, with tests.
- Added `DynamicTree::node_views` and `DynamicTree::root_index` accessors for BVH
  inspection.
- Demo site now mirrors the complete C `samples` app (~150 samples) at the project
  homepage: https://larsbrubaker.github.io/box3d-rust/

## 0.1.1 / 0.1.0

Initial crates.io releases — see git history for details.
