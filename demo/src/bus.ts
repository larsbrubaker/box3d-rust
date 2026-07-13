// Typed event bus — the contract between the SPA shell (main.ts) and the
// integration (interaction) layer. Menu actions and global keys emit here; the
// interaction layer subscribes. Kept in its own module (not main.ts) so the
// interaction layer can import the bus without pulling in the entry module,
// which would form an import cycle (main → demos → interaction → main).

import { type SampleEntry } from "./registry.ts";
import { VIEW_FLAG_DEFAULTS } from "./view-flags.ts";
import { DEFAULT_RENDER_SETTINGS } from "./render-settings.ts";

// ---------------------------------------------------------------------------
// Typed event bus — the contract with the integration (interaction) layer.
// Menu actions emit here; do NOT reach into interaction.ts from this file.
// ---------------------------------------------------------------------------

/** Payload types keyed by event name. `void` events carry no payload. */
export interface DemoEventMap {
  "sim.pause": void;
  "sim.step": void;
  "sim.restart": void;
  /** Frame the current selection (or the whole scene when nothing is selected). */
  "sim.frame": void;
  "sim.prevSample": { entry: SampleEntry | null };
  "sim.nextSample": { entry: SampleEntry | null };
  /** Toggle the bottom Diagnostics drawer (C showMetrics / M key). */
  "ui.metrics": void;
  "view.flag": { name: string; value: boolean };
  "view.scale": { name: string; value: number };
  "render.setting": { key: string; value: number | boolean };
}

export type DemoEventName = keyof DemoEventMap;
type Handler<K extends DemoEventName> = (payload: DemoEventMap[K]) => void;

class DemoBus {
  private handlers = new Map<DemoEventName, Set<(p: unknown) => void>>();

  on<K extends DemoEventName>(name: K, fn: Handler<K>): () => void {
    let set = this.handlers.get(name);
    if (!set) {
      set = new Set();
      this.handlers.set(name, set);
    }
    const wrapped = fn as (p: unknown) => void;
    set.add(wrapped);
    return () => set!.delete(wrapped);
  }

  emit<K extends DemoEventName>(
    name: K,
    ...payload: DemoEventMap[K] extends void ? [] : [DemoEventMap[K]]
  ): void {
    const set = this.handlers.get(name);
    if (!set) return;
    const arg = (payload[0] ?? undefined) as unknown;
    for (const fn of set) fn(arg);
  }
}

/** Global bus consumed by the interaction layer (and any future subscribers). */
export const demoBus = new DemoBus();

// ---------------------------------------------------------------------------
// Draw-flag / render defaults (from the C samples app; see sample.cpp/types.c).
// These seed the View and Render menu widgets and are re-emitted on load so a
// subscriber can sync without duplicating the numbers.
// ---------------------------------------------------------------------------

// C uses unbounded ImGui::InputFloat for these (default 1.0). We surface bounded
// sliders; some C samples set forceScale = 0.1 in their own setup.
const viewScaleDefaults: Record<string, number> = {
  force: 1.0, // b3DefaultDebugDraw.forceScale
  joint: 1.0, // b3DefaultDebugDraw.jointScale
  drawDistance: 100.0, // sample.h SampleContext.drawDistance (meters)
};

// Live mutable copies (menu widgets read/write these). Exported so main.ts's
// View/Render menus stay the single source of truth that emitInitialState reads.
// The flag/render defaults live in their own leaf modules (view-flags.ts,
// render-settings.ts) so every consumer shares one authoritative table.
export const viewFlags: Record<string, boolean> = { ...VIEW_FLAG_DEFAULTS };
export const viewScales: Record<string, number> = { ...viewScaleDefaults };
export const renderState = { ...DEFAULT_RENDER_SETTINGS };

// Emit the initial view/render state so a subscriber can sync. Exported so the
// interaction layer can pull current menu state right after it subscribes (it
// mounts long after this module's first boot-time emit).
export function emitInitialState(): void {
  for (const [key, value] of Object.entries(viewFlags)) {
    demoBus.emit("view.flag", { name: key, value });
  }
  for (const [name, value] of Object.entries(viewScales)) {
    demoBus.emit("view.scale", { name, value });
  }
  for (const [key, value] of Object.entries(renderState)) {
    demoBus.emit("render.setting", { key, value });
  }
}
