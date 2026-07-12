# Task 9 — Recording, replay, and world snapshots

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

The last full C subsystem: `world_snapshot.c`, `recording.c` +
`recording_ops.inl`, `recording_replay.c`, and `test_recording.c` (the largest
remaining test file). The determinism gate has passed, so replay has a solid
foundation — a recorded op stream replayed on the deterministic core must
reproduce states exactly.

Port order matters: snapshot first (replay bootstraps from a snapshot), then
op capture, then replay.

## Snapshots (world_snapshot.c)

- [ ] World state serialization (bodies, shapes, joints, contacts, islands,
      solver sets, id pools — the C layout, adapted to Vec-backed pools)
- [ ] Deserialization that reconstructs an identical stepping world
      (snapshot → step N → hash equals original → step N → hash)

## Recording (recording.c, recording_ops.inl)

- [ ] Op encoding for every public mutator (`recording_ops.inl` is the
      op table — port it as an enum + encode/decode pairs)
- [ ] `b3World_StartRecording` / `b3World_StopRecording` API
- [ ] Capture hooks in the public API layer (C wraps each mutator; decide the
      Rust shape — likely a `record_op` call at the top of each `pub fn`)

## Replay (recording_replay.c)

- [ ] Reader/dispatcher that re-executes an op stream against a fresh world
- [ ] Bit-exact verification path (replayed world hash matches recorded)

## Tests

- [ ] Port `test/test_recording.c` module by module alongside the above
      (it covers op round-trips per subsystem — body, shape, joint, world)
