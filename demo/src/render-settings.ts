// Render settings contract + module-level state, split out of three-scene.ts so
// the entry module (main.ts) can drive applyRenderSettings without importing
// three-scene.ts (and therefore Three.js). three-scene.ts is imported both
// statically (by the demo chunks) and — before this split — dynamically (by
// main.ts's render.setting bus consumer). That mixed static/dynamic import made
// Bun's `splitting:true` dev bundle double-emit three-scene's export list,
// producing a `SyntaxError: Duplicate export`. Routing main.ts through this
// three-free leaf keeps three-scene imported one consistent way.

/**
 * Renderer feature settings. This is the exact contract the Render menu (built
 * by the UI-shell agent) drives; names are verbatim and must not change.
 * Defaults mirror the C SampleContext: shadows on, GTAO on at medium quality,
 * IBL on, 0 exposure stops, sun strength 1, debug view "lit".
 */
export interface RenderSettings {
  shadows: boolean;
  gtao: boolean;
  gtaoQuality: 0 | 1 | 2;
  ibl: boolean;
  exposureStops: number;
  sunStrength: number;
  debugView: number;
}

export const DEFAULT_RENDER_SETTINGS: RenderSettings = {
  shadows: true,
  gtao: true,
  gtaoQuality: 0, // 0 = medium (C default GTAO on)
  ibl: true,
  exposureStops: 0, // EXPOSURE_STOPS_DEFAULT (0 EV) — the calibrated baseline
  sunStrength: 1,
  debugView: 0,
};

/** The slice of DemoScene that receiving render settings needs (kept structural
 * so this module has zero import dependency on three-scene.ts / Three.js). */
interface SettingsTarget {
  applySettings(s: RenderSettings): void;
}

// Module-level settings shared across routes. applyRenderSettings pushes changes
// into the currently-active DemoScene (only one route renders at a time).
let g_settings: RenderSettings = { ...DEFAULT_RENDER_SETTINGS };
let g_activeScene: SettingsTarget | null = null;

/** Register the scene that should receive render-setting changes (C: one route
 * renders at a time). Called from the DemoScene constructor. */
export function setActiveScene(scene: SettingsTarget): void {
  g_activeScene = scene;
}

/** Clear the active scene on dispose, but only if it is still the current one. */
export function clearActiveScene(scene: SettingsTarget): void {
  if (g_activeScene === scene) g_activeScene = null;
}

/** Update renderer settings and apply them to the active scene. */
export function applyRenderSettings(s: Partial<RenderSettings>): void {
  g_settings = { ...g_settings, ...s };
  g_activeScene?.applySettings(g_settings);
}

/** Read the current renderer settings (a copy). */
export function getRenderSettings(): RenderSettings {
  return { ...g_settings };
}
