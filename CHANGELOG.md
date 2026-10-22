# Changelog

All notable changes to box3d-rust are documented here. This project adheres to
semantic versioning (0.x: minor-compatible additive changes bump the patch number).

## 0.5.0

This release syncs the port to upstream Box3D `efca0c1`. The C reference pin
advances from `9f998c8`, and this port follows its API and behavior. The sync
ran one upstream commit at a time. Simulation results are not bit-compatible
with 0.4.x, and hull, compound, recording, and snapshot blobs from earlier
versions no longer load (see Breaking). The release also ports the replay
keyframe ring (`RecPlayer::seek_frame`), regenerates the demo's bundled replay
recording, and fixes the recording code generators. Ported:

- `51f056e` SAT benchmark (#177)
- `e77352c` Add support for AVX2 and runtime feature detection (#174)
- `16f7f4c` Tree perf (#179)
- `f8c4fe8` Shape cast support for box movers (#180)
- `003a3f7` Fix recording crash on compounds (#181)
- `3d9db79` Various recording fixes (#182)
- `5d83df8` Improve compound serialization (#183)
- `a2e4816` Fixes for #175 (#187)
- `8128d74` Discussion template (#185) (GitHub discussion template only, nothing to port)
- `0cfc475` Solver stuff (#189)
- `efca0c1` Gyro tolerance (#191)

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
- Baked compounds no longer carry a dynamic tree or tree proxies (upstream
  `5d83df8`). `CompoundData` loses `proxy_offset` and `tree` and gains
  `bounds`, `tree_height`, `node_count`, and `nodes` (the live tree nodes;
  leaves hold the child index). The blob header is 96 bytes (was 184; the
  embedded `b3DynamicTree` and `COMPOUND_TREE_OFFSET` are gone), the proxy
  array is dropped, and `COMPOUND_VERSION` no longer depends on the tree
  version: it is now `0x9E4B17D3A25C68F1 ^ MESH_VERSION ^ HULL_VERSION`
  (`0x5FFEB407FB88E7F5`). Compound blobs and hashes change, and compound
  blobs and recordings from earlier versions no longer load.
- `convert_compound_to_bytes` and `convert_bytes_to_compound` are replaced by
  `CompoundData::to_bytes` and `validate_compound` (upstream
  `b3ValidateCompound`, replacing `b3ConvertCompoundToBytes` /
  `b3ConvertBytesToCompound`). In C the compound is now a pointer-free blob
  used in place; the port still parses it into an owned `CompoundData`.
- `RegistrySlot::live_compound` and `RegistrySlot::get_live_compound` are
  replaced by `RegistrySlot::compound`, filled when the registry loads
  (upstream removed `b3RegistrySlot::live` and `b3RecGetLiveCompound`).
- The local rotational inertia moves from `Body::inertia` to
  `BodySim::inertia` (upstream `efca0c1`), so the solver reads it directly.
  `body_flags::BODY_ENABLE_CONTACT_RECYCLING` is renamed
  `body_flags::ENABLE_CONTACT_RECYCLING`, and `math_functions::skew` is
  removed (upstream removed `b3Skew`). The snapshot version becomes 11 (the
  world config stores the isotropic inertia tolerance) and the Rust snapshot
  layout version 5, so snapshots from earlier versions no longer load. The
  recording minor version becomes 9.
- The `RecPlayer` keyframe ring fields `keyframe_interval`,
  `keyframe_min_interval`, `keyframe_budget`, `keyframe_bytes`, and
  `last_keyframe_frame` (public in 0.4.1) are now crate-private, as C keeps
  them private. Set the budget and minimum interval with
  `set_keyframe_policy` and read the first four with the `get_keyframe_*`
  getters; `last_keyframe_frame` has no public replacement.

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
- `shape_get_compound` (upstream `b3Shape_GetCompound`). The port's shape
  owns a copy of the compound, so it returns that copy.
- `dynamic_tree::TreeView` and `make_tree_view` (upstream `b3TreeView`,
  `b3MakeTreeView`, `b3TreeView_Query` / `_QueryClosest` / `_RayCast` /
  `_BoxCast`): the queries and casts run on a node array with optional
  proxies. A view without proxies skips the category test and reports the
  leaf's shape index as the user data; compound queries use it. The
  `DynamicTree` queries wrap it and are unchanged.
- `WorldDef::isotropic_inertia_tolerance` (default 0.2, clamped to [0, 1])
  with `world_set_isotropic_inertia_tolerance` /
  `world_get_isotropic_inertia_tolerance` (upstream `efca0c1`,
  `b3World_SetIsotropicInertiaTolerance` / `_GetIsotropicInertiaTolerance`),
  recorded as the new `WorldSetIsotropicInertiaTolerance` op (0x70). A
  change only reaches bodies whose mass is computed afterwards. New body
  flag `body_flags::ENABLE_GYROSCOPIC_TORQUE`.
- The replay keyframe ring (upstream `b3RecCaptureKeyframe`,
  `b3RecPlayerRestoreKeyframe`, `b3RecSeedKeyframeRegistry`). `RecPlayer`
  now snapshots the world every keyframe interval while stepping (also in
  `sub_step_frame` and `validate_replay`), within a byte budget: when a
  capture would exceed it, the interval doubles and keyframes off the new
  grid are evicted. `seek_frame` restores the nearest keyframe strictly
  before the target (or restarts) and re-steps only the gap, and seeking to
  the current frame is now a no-op, as in C. The policy defaults to a 16
  frame interval and a 512 MiB budget (`get_keyframe_budget` used to report
  0); `set_keyframe_policy` keeps the current value for a zero budget or a
  non-positive interval and drops the ring, and `restart` keeps the ring.
  Captures serialize through a registry pre-seeded 1:1 from the player's
  geometry slots, so geometry ids stay stable and the registry never grows.
  `get_keyframe_bytes` charges each keyframe its image allocation plus its
  outliner body list, as C does; the images are the port's field-by-field
  snapshots, so the totals differ from C's (in float, 8744 bytes per
  keyframe of the four box stack where C charges 12400 to 12496), and a
  budget sized between keyframes can keep a different ring than C's.
  Capture cadence, eviction, and seek targets under the default and a one
  byte budget, and the world hashes after each seek, were confirmed against
  a scalar C build of `efca0c1` in both precision modes; two budgets sized
  relative to the first keyframe also keep C's ring in float.
  `RecBuffer` gains `capacity`, the allocation size C's `b3RecBufAppend`
  growth rule would reach, which the keyframe budget charges.

### Changed

- `GeometryRegistry::intern` matches an existing entry by bytes only, as
  upstream's `b3InternGeometry` does; it used to also require the same
  kind.
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
  upstream added to `AllOps`.
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
- `validate_compound` runs upstream's checks in upstream's order: at least
  the header size, the version, a stored byte count equal to the buffer
  length, a node offset past the header and 8 byte aligned, at least two
  nodes, and the node array inside the buffer (in 64-bit arithmetic). It
  also keeps the port's existing section, nested blob, and material index
  checks, and adds node checks C does not make: every internal node must
  point at a sibling pair after it inside the node array, every node with
  the leaf bit (empty nodes included) must name an existing child, and the
  tree may be at most 256 levels deep (half the traversal stack; C's
  balanced median split trees stay far below it), so a corrupt tree is
  rejected instead of panicking, looping, or overflowing the stack in a
  query. The compound blobs of four test compounds (including a
  40 sphere compound with a 7 level tree), their hashes, a ray cast, the
  bounds, and the header check outcomes were confirmed against a scalar C
  build of `5d83df8` in both precision modes; the blobs are byte-identical.
- Compound AABBs come from the stored `bounds`, and compound queries, casts,
  the mover, and the broad-phase child pairs run on the compound tree view.
  The traversal order and results are unchanged. The demo's compound stats
  report the tree byte count as `node_count` times the 32 byte node and the
  stored `tree_height`, as the C samples now do.
- Loading the geometry registry follows upstream's new `b3RecLoadSlots`: an
  entry of any kind with zero bytes (or more than `i32::MAX`) fails
  `RecPlayer::create` ("has an invalid byte count"), as does a compound
  entry that fails `validate_compound` ("is not a valid compound"). Both
  used to load: an empty hull, mesh, or height field entry failed only its
  shape op, and a corrupt compound only when a shape used it. The "too small
  for a compound" check is now part of compound validation, and the replay's
  "compound geometry <id> is corrupt" message is gone, as in C. Hull, mesh,
  and height field slots are still converted (and checked) when a shape
  uses them, with "geometry <id> is corrupt", where C uses the bytes as they
  are.
- The snapshot layout hash also mixes in the geometry blob sizes the port
  tracks and the hull, mesh, height field, and compound versions, following
  upstream's `b3ComputeLayoutHash`. Snapshot images from earlier builds no
  longer load.
- Ports upstream's `CompoundSerializeBadNodeCount` test, the updated
  `CompoundCreateMixed`, `CompoundSerializeRoundtrip`, and
  `CompoundMeshContactMaterial` checks, and the zero byte count case in
  `CorruptCompound`. The compound part of `TreeSaveLoadRoundtrip` is removed,
  as upstream did. Adds tests for a zero byte hull entry and an invalid
  compound entry failing player creation, and for the tree node checks.

- Contact recycling no longer skips real overlap (upstream `a2e4816`,
  discussion #175). `collide_spheres` and `collide_capsule_and_sphere` now
  report pairs within the speculative distance, like the other manifold
  routines, instead of only overlapping pairs. A contact recycles only when
  speculative points are enabled for the pair, and a pre-solve rejection
  clears the cached relative transform so the next step rebuilds the
  contact. Reproducing tests (`SpeculativeMarginTest`,
  `RecycledSphereApproachTest`, `PreSolveRejectRecycleTest`,
  `SpeculativeOffRecycleTest`, ported from upstream) failed before the fix.
  The wave pile determinism goldens change to upstream's values, and the
  mesh mix hash changes; all confirmed against a scalar C build of
  `a2e4816` in both precision modes. `World::step` documents that a zero
  time step still runs collision and reports contact and sensor events.
- The solver accumulates velocity changes instead of applying every impulse
  to the bodies in turn (upstream `0cfc475`). The wide contact push and solve
  track the linear normal velocity per manifold, apply the normal impulses
  to the angular velocities through precomputed `invI * (r x n)` terms, and
  apply the summed normal impulse to the linear velocities once (in solve,
  together with the central friction impulse). The normal clamp computes
  the delta impulse directly with max, and the twist friction clamp with
  min / max. Revolute
  spring, motor, and limits share one axial velocity and apply one combined
  axial impulse; spherical spring, motor, twist limits, and cone limit do
  the same with the angular velocity. The revolute, spherical, motor, and
  weld point constraints solve the 3x3 system with the new
  `joint::solve_point_constraint` (upstream `b3SolvePointConstraint`)
  instead of forming the effective mass from skew products and calling
  `solve3`. Simulation results change. The determinism goldens move to
  upstream's values: falling ragdolls 265 / `0xB98B3638` (double 267 /
  `0x01EB8FDB`), wave pile 288 / `0xFC1CD2DC` (double 333 / `0x80D5266B`),
  query spawn hash `0x58F47348` (double `0x90EE3E7C`), mesh drop
  `0x983F27EA` (double `0x10051259`). The rolling mix hash becomes
  `0x3447FD01` (double `0x6686EFA8`) and the mesh mix hash `0x141DBC7E`
  (double `0xD802486E`). All values were confirmed with a scalar C build of
  `0cfc475` (`BOX3D_DISABLE_SIMD`) in both precision modes.
- Gyroscopic torque is skipped for nearly isotropic bodies (upstream
  `efca0c1`). Each mass update (`update_body_mass_data`,
  `body_set_mass_data`) measures the deviatoric part of the local inertia
  (upstream `b3UpdateIsotropicInertia`, the squared Frobenius norm against
  the mean of the diagonal) and sets `ENABLE_GYROSCOPIC_TORQUE` only when
  the anisotropy exceeds the world tolerance. `update_body_mass_data`
  clears the flag first, so bodies without shapes and static or kinematic
  bodies stay unflagged; fixed rotation zeroes the inertia, which clears it
  too. The
  velocity integration runs the gyroscopic Newton solve only for flagged
  bodies and reads `BodySim::inertia` instead of inverting the inverse
  inertia. Ports `CompoundInertiaConsistency` and
  `IsotropicInertiaTolerance` from `test_body.c`, the new
  `TestWorldCoverage` checks, and the `AllOps` tolerance addition as a
  replay round trip. Simulation results change. The determinism goldens
  move to upstream's values: falling ragdolls 291 / `0xFF4C605B` (double
  263 / `0xA5423F98`), wave pile 272 / `0x266F83B0` (double 272 /
  `0xC8E5F3D6`), query spawn hash `0xF3AD9BA6` (double `0x489585E6`), mesh
  drop 226 / `0xD2FF069D` (double 218 / `0xD378CF04`). The rolling mix hash
  becomes `0x56B250F7` (double `0xB7FBCFC0`) and the mesh mix hash
  `0x246CC2A7` (double `0x9DA0B374`). All values were confirmed with a
  scalar C build of `efca0c1` (`BOX3D_DISABLE_SIMD`) in both precision
  modes.

### Fixed

- The demo Replay viewer's bundled `demo/public/recordings/sample.b3rec` was
  stale after the upstream format bumps (recording 6.9, snapshot 11, hull and
  compound layouts) and was rejected by `RecPlayer::create`. It is regenerated
  from the current wasm build (same Box Stack scene, 300 frames at 60 Hz, 4
  sub-steps) by the new reproducible generator `bun run record:sample` in
  `demo/` (`demo/scripts/record-sample.ts`, run after `bun run build:wasm`),
  and the new test `bundled_demo_recording_replays_without_divergence` fails
  whenever a future format change makes the file stale again.
- The recording code generators (`scripts/gen_recording_ops.py`,
  `scripts/gen_recording_dispatch.py`) failed with `KeyError: STR` after
  upstream merged the `BODYSTR`/`SHAPESTR` argument tags into `STR`. They now
  handle `STR`, fail with a clear message on any unknown tag, emit the
  current split `src/recording` layout (run through rustfmt), and share the
  op-manifest parser in the new `scripts/recording_manifest.py`.
  Regenerating reproduces the checked-in files; the only change is that the
  body setter writers and dispatch arms now follow manifest order.
- `RecPlayer` per-frame query store now matches C `b3RecPlayer_StepFrame` /
  `b3RecPlayer_Restart`: `restart` (and a backward `seek_frame` that falls
  back to it) clears the stashed queries, `step_frame` at the end of the
  recording no longer wipes the last frame's queries, and `restart` keeps
  the pending query-tag key as C does. The keyframe ring tuning fields
  (`keyframe_interval`, `keyframe_min_interval`, `keyframe_budget`,
  `keyframe_bytes`, `last_keyframe_frame`) are now crate-private (see
  Breaking); external code could set `keyframe_interval` to 0 and panic on
  a divide by zero.

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
- The `b3Free` zero-size fix in `core.c` (upstream `3d9db79`). The port has
  no custom allocator hooks.
- The rename of `recording_replay.c` / `.h` to `replay.c` / `.h` and the
  `CMakeLists.txt` reorder (upstream `3d9db79`) have no Rust counterpart
  beyond updated doc comments; the Rust module names stay as they were.
- The new Village benchmark (upstream `5d83df8`: `CreateVillage` /
  `StepVillage` in `shared/benchmarks.c`, `benchmark/main.c`, and the
  Benchmark / Village sample; listed as planned in the demo registry), the
  compound panel in the replay sample's shape inspector, the
  `debug_adapter.c` culling switch to `b3QueryCompound`, and the
  `docs/compound.md` update.
- `b3InternGeometry` / `b3AppendGeometry` now copy the bytes instead of
  taking ownership (upstream `5d83df8`). The port's registry already owns
  `Vec` copies. The compound pointer scrub upstream added to
  `b3RecSeedKeyframeRegistry` in `003a3f7` and removed again in `5d83df8`
  needs no port: the port's keyframe seed hashes and appends the slot bytes
  as they are, like upstream's current seed.
- The replay sample's keyframe-policy popup (budget, minimum interval, and
  pre-generating the ring behind a progress bar). The demo's replay viewer
  seeks through the ring under the default policy.
- The new cylinder questions in `docs/faq.md` (upstream `a2e4816`).
- The new Bodies / Spinning Capsule sample (upstream `efca0c1`); it is
  listed as planned in the demo registry.

## 0.4.1

Bug-fix release; no public API changes. The C reference pin is unchanged
(`9f998c8`).

### Fixed

- `body_set_awake(world, id, false)` on a body whose island had a pending
  split used the island id read before `split_island` ran, so
  `try_sleep_island` was handed a stale island id and tripped
  a `debug_assert` panic. The body's island id is now re-read after the
  split, matching C `b3Body_SetAwake`. Reported by @KennethWerner in #2.

## Earlier releases

Releases 0.4.0 and earlier are in [CHANGELOG-ARCHIVE.md](CHANGELOG-ARCHIVE.md).
