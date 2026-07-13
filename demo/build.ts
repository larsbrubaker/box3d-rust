// Static build script for GitHub Pages deployment
// Usage: bun run build.ts

import { cpSync, mkdirSync, readFileSync, writeFileSync } from "fs";
import { join } from "path";
import { versionDefines } from "./build-version.ts";

const ROOT = import.meta.dir;
const DIST = join(ROOT, "dist");

// Clean and create dist directory
mkdirSync(DIST, { recursive: true });

// 1. Bundle TypeScript with code splitting
console.log("Bundling TypeScript...");
const result = await Bun.build({
  entrypoints: [join(ROOT, "src/main.ts")],
  outdir: DIST,
  splitting: true,
  target: "browser",
  format: "esm",
  minify: true,
  naming: "[dir]/[name].[ext]",
  // Embed crate version + git hash for the sidebar footer (src/version.ts).
  define: versionDefines(ROOT),
});

if (!result.success) {
  console.error("Build failed:");
  for (const msg of result.logs) {
    console.error(msg);
  }
  process.exit(1);
}

console.log(`  ${result.outputs.length} files generated`);

// 2. Copy index.html with script tag pointing to bundled JS
console.log("Copying index.html...");
let html = readFileSync(join(ROOT, "index.html"), "utf-8");
html = html.replace(
  '<script type="module" src="./src/main.ts"></script>',
  '<script type="module" src="./main.js"></script>'
);
writeFileSync(join(DIST, "index.html"), html);

// 3. Copy styles
console.log("Copying styles...");
cpSync(join(ROOT, "styles"), join(DIST, "styles"), { recursive: true });

// 4. Copy WASM package
console.log("Copying WASM package...");
cpSync(join(ROOT, "public/pkg"), join(DIST, "public/pkg"), { recursive: true });

// 5. Copy sample mesh assets (MIT, from box3d-cpp-reference/data/meshes)
console.log("Copying sample meshes...");
cpSync(join(ROOT, "public/meshes"), join(DIST, "public/meshes"), { recursive: true });

// 6. Copy sample data assets (MIT, from box3d-cpp-reference/data) — e.g. the Tree
//    Benchmark AABB record files under data/trees/, fetched by demos/tree.ts.
console.log("Copying sample data...");
cpSync(join(ROOT, "public/data"), join(DIST, "public/data"), { recursive: true });

// 7. Copy bundled .b3rec recordings (recorded from this port itself) — fetched
//    cold by demos/replay.ts. Binary; cpSync preserves bytes.
console.log("Copying sample recordings...");
cpSync(join(ROOT, "public/recordings"), join(DIST, "public/recordings"), { recursive: true });

console.log(`Build complete → ${DIST}`);
