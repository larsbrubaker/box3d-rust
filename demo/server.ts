import { mkdirSync, readFileSync, existsSync, statSync, rmSync } from "fs";
import { join, extname } from "path";
import { versionDefines } from "./build-version.ts";

const PORT = parseInt(process.env.PORT || "3001");
const ROOT = import.meta.dir;
const DEV_CACHE = join(ROOT, ".dev-cache");

const MIME_TYPES: Record<string, string> = {
  ".html": "text/html",
  ".css": "text/css",
  ".js": "application/javascript",
  ".map": "application/json",
  ".wasm": "application/wasm",
  ".json": "application/json",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".ico": "image/x-icon",
  ".obj": "text/plain",
  ".b3rec": "application/octet-stream",
};

function readIndexHtml(): string {
  let html = readFileSync(join(ROOT, "index.html"), "utf-8");
  // Dev serves the Bun-bundled entry (resolves bare "three" / "three/addons/...")
  html = html.replace(
    '<script type="module" src="./src/main.ts"></script>',
    '<script type="module" src="./main.js"></script>',
  );
  return html;
}

async function buildDevBundle(): Promise<void> {
  if (existsSync(DEV_CACHE)) {
    rmSync(DEV_CACHE, { recursive: true, force: true });
  }
  mkdirSync(DEV_CACHE, { recursive: true });

  console.log("[server] Bundling TypeScript (resolves node_modules)...");
  const result = await Bun.build({
    entrypoints: [join(ROOT, "src/main.ts")],
    outdir: DEV_CACHE,
    splitting: true,
    target: "browser",
    format: "esm",
    sourcemap: "inline",
    naming: "[dir]/[name].[ext]",
    // Embed crate version + git hash for the sidebar footer (src/version.ts).
    define: versionDefines(ROOT),
  });

  if (!result.success) {
    console.error("[server] Dev bundle failed:");
    for (const msg of result.logs) {
      console.error(msg);
    }
    throw new Error("Dev bundle failed");
  }

  console.log(`[server] Dev bundle ready (${result.outputs.length} files)`);
}

await buildDevBundle();

function tryServe(pathname: string): { content: string | Uint8Array; mime: string } | null {
  if (pathname === "/" || pathname === "/index.html") {
    return { content: readIndexHtml(), mime: "text/html" };
  }

  // Bundled app JS (entry + code-split chunks, includes three.js)
  const cachePath = join(DEV_CACHE, pathname.replace(/^\//, ""));
  if (existsSync(cachePath) && statSync(cachePath).isFile()) {
    const ext = extname(cachePath);
    const mime = MIME_TYPES[ext] || "application/octet-stream";
    return { content: readFileSync(cachePath, "utf-8"), mime };
  }

  // Static files relative to demo root (styles, public/pkg, etc.)
  const filePath = join(ROOT, pathname);
  if (existsSync(filePath) && statSync(filePath).isFile()) {
    const ext = extname(filePath);
    const mime = MIME_TYPES[ext] || "application/octet-stream";
    // Binary assets must be served as bytes, not re-encoded through utf-8 (which
    // corrupts and inflates them). .b3rec recordings join .wasm here.
    if (ext === ".wasm" || ext === ".b3rec") {
      return { content: new Uint8Array(readFileSync(filePath)), mime };
    }
    return { content: readFileSync(filePath, "utf-8"), mime };
  }

  return null;
}

const server = Bun.serve({
  port: PORT,
  fetch(req) {
    const url = new URL(req.url);
    const resolved = tryServe(url.pathname);
    if (resolved) {
      return new Response(resolved.content, {
        headers: {
          "Content-Type": resolved.mime,
          "Access-Control-Allow-Origin": "*",
          "Cache-Control": "no-cache",
        },
      });
    }

    // SPA fallback: serve index.html for unresolved routes (client-side routing)
    return new Response(readIndexHtml(), {
      headers: { "Content-Type": "text/html", "Cache-Control": "no-cache" },
    });
  },
});

console.log(`Box3D Rust Demo running at http://localhost:${server.port}`);
