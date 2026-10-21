// Regenerates public/recordings/sample.b3rec, the Replay viewer's bundled recording.
//
// Run `bun run record:sample` from demo/ after `bun run build:wasm` whenever the recording or
// snapshot format changes (REC_VERSION_*, SNAP_VERSION, SNAP_LAYOUT_VERSION, HULL_VERSION, ...).
// The Rust test `bundled_demo_recording_replays_without_divergence`
// (src/recording_replay_tests_bundled.rs) fails when the bundled file has gone stale.
//
// It records through the release wasm32 build in public/pkg - the same module and exports the
// demo's Recording panel uses - so the header carries the wasm pointer width (4) the browser
// player expects. Scene: Stacking Box Stack (sim_reset_stacking, 40 cubes on a ground box),
// 300 frames of sim_step(1/60, 4) = 5 seconds at 60 Hz with 4 sub-steps.
//
// After writing the file it loads it back with the Replay viewer's own loader (replay_load ->
// RecPlayer::create) and plays it to the end, failing on rejection or divergence.

import { readFileSync, writeFileSync } from "fs";
import { join } from "path";
import * as wasm from "../public/pkg/box3d_wasm.js";

const ROOT = join(import.meta.dir, "..");
const OUT = join(ROOT, "public/recordings/sample.b3rec");
const FRAMES = 300;
const TIME_STEP = 1 / 60;
const SUB_STEPS = 4;
// Ground plus the 40-cube Box Stack; the Rust test asserts the same count.
const BODIES = 41;

wasm.initSync({ module: readFileSync(join(ROOT, "public/pkg/box3d_wasm_bg.wasm")) });

const bodies = wasm.sim_reset_stacking();
if (bodies !== BODIES) {
  throw new Error(`expected ${BODIES} bodies from sim_reset_stacking, got ${bodies}`);
}
wasm.sim_start_recording();
for (let i = 0; i < FRAMES; i++) {
  wasm.sim_step(TIME_STEP, SUB_STEPS);
}
const bytes = wasm.sim_stop_recording();
if (bytes.length === 0) {
  throw new Error("recording is empty");
}

if (!wasm.replay_load(bytes)) {
  throw new Error("replay_load rejected the fresh recording");
}
const frameCount = wasm.replay_frame_count();
while (!wasm.replay_is_at_end()) {
  wasm.replay_step();
}
if (wasm.replay_has_diverged()) {
  throw new Error(`replay diverged at frame ${wasm.replay_diverge_frame()}`);
}
if (frameCount !== FRAMES || wasm.replay_frame() !== FRAMES) {
  throw new Error(`expected ${FRAMES} frames, got ${frameCount} (ended at ${wasm.replay_frame()})`);
}
wasm.replay_unload();

writeFileSync(OUT, bytes);
console.log(`wrote ${OUT}: ${bytes.length} bytes, ${bodies} bodies, ${frameCount} frames, replay clean`);
