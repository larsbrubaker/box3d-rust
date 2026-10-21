# Bundled replay recordings

`sample.b3rec` is a Box3D recording (`.b3rec`) played by the Replay viewer
(`#/replay`, port of `samples/sample_replay.cpp`).

## Provenance

It was **recorded from this port itself**, in-browser, through the same WebAssembly
build and recording exports the demo's Recording panel uses — not imported from the
C samples app. Concretely, it was captured by driving the wasm module directly:
reset the Stacking **Box Stack** scene (`sim_reset_stacking`), then
`sim_start_recording()` → 300 × `sim_step(1/60, 4)` → `sim_stop_recording()`, i.e.
**5 seconds at 60 Hz with 4 sub-steps** of a 41-body box stack settling under
gravity.

## Regenerating

The file goes stale whenever the recording or snapshot format changes; the Rust
test `bundled_demo_recording_replays_without_divergence`
(`src/recording_replay_tests_bundled.rs`) fails when it does. Regenerate it from
`demo/` with:

    bun run build:wasm
    bun run record:sample

`scripts/record-sample.ts` performs exactly the capture above through the
release wasm build in `public/pkg`, then loads the result back with the Replay
viewer's loader (`replay_load`) and plays it to the end, failing on rejection or
divergence.

Recording it through the wasm build (rather than a native 64-bit binary) matters.
The port's `.b3rec` wire format is pointer-width independent (every field is
serialized explicitly), but `RecPlayer` keeps C's header pointer-width gate
(`replay.c`) and rejects a recording whose header pointer width differs from the
host's. The bundled file is therefore recorded through wasm32 so its header
carries a pointer width of 4, matching the browser player it ships with.

You can also load any `.b3rec` you record yourself: use the **Record** buttons on
any dynamics demo (e.g. Stacking) to capture and download a recording, then load it
into the Replay viewer with the **Choose File** picker. Loading a local file via
the picker is ordinary in-page behavior — nothing is uploaded anywhere.
