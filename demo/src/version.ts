// Build-time version + git-hash constants for the sidebar footer.
//
// The three `__…__` identifiers are substituted at bundle time by Bun's `define`
// (see build.ts and server.ts, which read the crate version from the workspace
// Cargo.toml and the hash from `git rev-parse`). When the module is evaluated
// *without* that substitution — e.g. `bun test` imports it directly, or git is
// unavailable so the define resolves to an empty string — the `typeof` guards
// fall back to safe placeholders instead of throwing on an undeclared name
// (`typeof someUndeclared` is "undefined", never a ReferenceError).

declare const __APP_VERSION__: string;
declare const __GIT_HASH__: string;
declare const __GIT_HASH_FULL__: string;

/** Crate version, e.g. "0.1.1". Falls back to "dev" outside a defined build. */
export const APP_VERSION: string =
  typeof __APP_VERSION__ !== "undefined" && __APP_VERSION__ ? __APP_VERSION__ : "dev";

/** Short git hash, e.g. "be74cf6". Empty when git is unavailable at build time. */
export const GIT_HASH: string =
  typeof __GIT_HASH__ !== "undefined" && __GIT_HASH__ ? __GIT_HASH__ : "";

/** Full git hash, used to build the GitHub commit link. Empty when unavailable. */
export const GIT_HASH_FULL: string =
  typeof __GIT_HASH_FULL__ !== "undefined" && __GIT_HASH_FULL__ ? __GIT_HASH_FULL__ : "";
