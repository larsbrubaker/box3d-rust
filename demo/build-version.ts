// Build-time provenance: crate version (workspace Cargo.toml) + git hash.
// Shared by build.ts (production) and server.ts (dev) so both bundles embed the
// same footer identifiers via Bun's `define`. Every lookup is best-effort: a
// missing Cargo.toml or absent git yields a placeholder rather than failing the
// build (the footer degrades to just the version, or "dev").

import { readFileSync } from "fs";
import { join } from "path";
import { execSync } from "child_process";

export interface VersionInfo {
  version: string;
  short: string;
  full: string;
}

/** Read `version = "…"` from the workspace Cargo.toml (`root`/.. == repo root). */
function readCrateVersion(root: string): string {
  try {
    const cargo = readFileSync(join(root, "..", "Cargo.toml"), "utf-8");
    const m = cargo.match(/^version\s*=\s*"([^"]+)"/m);
    if (m?.[1]) return m[1];
  } catch {
    /* Cargo.toml unreadable — fall through to placeholder. */
  }
  return "dev";
}

function gitRev(root: string, args: string): string {
  try {
    return execSync(`git rev-parse ${args}`, { cwd: root, stdio: ["ignore", "pipe", "ignore"] })
      .toString()
      .trim();
  } catch {
    return "";
  }
}

export function versionInfo(root: string): VersionInfo {
  return {
    version: readCrateVersion(root),
    short: gitRev(root, "--short HEAD"),
    full: gitRev(root, "HEAD"),
  };
}

/** `define` map replacing the `__…__` tokens read by src/version.ts. */
export function versionDefines(root: string): Record<string, string> {
  const { version, short, full } = versionInfo(root);
  return {
    __APP_VERSION__: JSON.stringify(version),
    __GIT_HASH__: JSON.stringify(short),
    __GIT_HASH_FULL__: JSON.stringify(full),
  };
}
