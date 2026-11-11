// Shared sample Info panel (C DrawInfoPanel, laid out for the web). Every sample
// page gets the same right-hand panel, built once by `demoPage`:
//
//   head  — category breadcrumb, ◀ sample name ▶, C-source link, PAUSED badge.
//           The name/category/link come from the registry entry of the scene
//           that is *on screen*, so a multi-scene page always shows the current
//           sample, never just its category.
//   desc  — the active sample's description only, plus an optional collapsed
//           "About this page" for page-wide notes not tied to one sample.
//   body  — the page's own controls (returned to pages as `controls`), so a
//           sample's parameters sit directly under its description.
//   tail  — shared simulation chrome, in a fixed order regardless of who adds
//           it when: transport (Pause/Step, Time scale) → the demoted scene
//           switcher (a compact <select>; the sidebar tree and ◀/▶ are the
//           primary navigation) → stats / Solver / Recording / Debug / Keyboard.
//
// In-place scene switches (no hashchange) rewrite the URL with replaceState and
// emit `sample.active`, so the deep link, the sidebar highlight, and the [ / ]
// walk all follow the sample on screen.
//
// Three-free leaf: imports only the registry and the bus, so pages and the
// interaction layer can both use it without import cycles.

import { demoBus } from "./bus.ts";
import {
  type SampleEntry,
  cSourceUrl,
  entryHref,
  findByRouteName,
  findByRouteScene,
  findByRouteSlug,
  firstEntryForRoute,
  neighborOf,
} from "./registry.ts";

export type SamplePanelOpts = {
  /** Fallback sample name when no registry entry resolves (the page title). */
  title: string;
  /** Fallback category when no registry entry resolves. */
  category?: string;
  /** The canvas hint element whose text follows the active scene. */
  hintEl?: HTMLElement | null;
  /** Page-wide hint, used for scenes without their own `sceneHints` entry. */
  hint?: string;
  /**
   * Description of the active sample: one HTML string for a single-sample page,
   * or a map keyed by the page's scene key for a multi-scene page.
   */
  info?: string | Record<string, string>;
  /** Per-scene canvas hints for scenes with their own controls (falls back to `hint`). */
  sceneHints?: Record<string, string>;
  /** Page-wide notes not tied to one sample — shown in a collapsed "About this page". */
  about?: string;
};

export type SceneOption = { label: string; value: string };

const panels = new WeakMap<HTMLElement, SamplePanel>();

/** The panel that owns `el` (its root or its body), if any. */
export function getSamplePanel(el: HTMLElement): SamplePanel | undefined {
  return panels.get(el) ?? panels.get(el.closest(".demo-controls") as HTMLElement);
}

/** The `<route>` segment of the current `#/<route>/<slug>` hash ("" at home). */
function hashParts(): string[] {
  return window.location.hash.replace(/^#\/?/, "").split("/").filter(Boolean);
}

export class SamplePanel {
  readonly root: HTMLElement;
  readonly head: HTMLElement;
  readonly desc: HTMLElement;
  /** The page's own controls. Pages receive this as `controls`. */
  readonly body: HTMLElement;
  readonly tail: HTMLElement;
  /** Tail slot: Pause / Step + Time scale (attachInteraction). */
  readonly transportSlot: HTMLElement;
  /** Tail slot: the demoted scene <select>. */
  readonly switcherSlot: HTMLElement;
  /** Tail slot: stats, Solver, Recording, Debug draw, Keyboard (attachInteraction). */
  readonly simSlot: HTMLElement;
  /** PAUSED badge in the head (driven by attachInteraction). */
  readonly pausedEl: HTMLElement;

  private readonly route: string;
  private readonly opts: SamplePanelOpts;
  private readonly nameEl: HTMLElement;
  private readonly categoryEl: HTMLElement;
  private readonly statusEl: HTMLElement;
  private readonly cSourceLink: HTMLAnchorElement;
  private readonly prevBtn: HTMLButtonElement;
  private readonly nextBtn: HTMLButtonElement;
  private readonly descBox: HTMLElement;
  private sceneSelect: HTMLSelectElement | null = null;
  private hintOverride: string | null = null;
  private entryValue: SampleEntry | undefined;
  private scene: string | undefined;

  constructor(root: HTMLElement, opts: SamplePanelOpts) {
    this.root = root;
    this.opts = opts;
    const parts = hashParts();
    this.route = parts[0] ?? "";
    // Start on the deep-linked entry; pages refine it with setScene once they
    // know their scene key.
    this.entryValue =
      (parts[1] ? findByRouteSlug(this.route, parts[1]) : undefined) ?? firstEntryForRoute(this.route);
    this.scene = this.entryValue?.scene;

    root.classList.add("sample-panel");

    this.head = document.createElement("div");
    this.head.className = "samples-info-head";
    this.head.innerHTML =
      `<div class="sample-breadcrumb"><span class="sample-category"></span>` +
      `<span class="s-tag s-partial sample-status" hidden title="Disclosed divergence from the C sample — see the description">PARTIAL</span></div>` +
      `<div class="sample-title-row">` +
      `<button class="sample-nav-btn sample-prev" type="button" title="Previous sample ([)" aria-label="Previous sample">◀</button>` +
      `<h2 class="sample-name"></h2>` +
      `<button class="sample-nav-btn sample-next" type="button" title="Next sample (])" aria-label="Next sample">▶</button>` +
      `</div>` +
      `<a class="sample-csource" target="_blank" rel="noopener" hidden></a>` +
      `<div class="sample-paused" hidden>PAUSED <span class="sample-paused-hint">(P)</span></div>`;
    this.nameEl = this.head.querySelector(".sample-name") as HTMLElement;
    this.categoryEl = this.head.querySelector(".sample-category") as HTMLElement;
    this.statusEl = this.head.querySelector(".sample-status") as HTMLElement;
    this.cSourceLink = this.head.querySelector(".sample-csource") as HTMLAnchorElement;
    this.prevBtn = this.head.querySelector(".sample-prev") as HTMLButtonElement;
    this.nextBtn = this.head.querySelector(".sample-next") as HTMLButtonElement;
    this.pausedEl = this.head.querySelector(".sample-paused") as HTMLElement;
    this.prevBtn.addEventListener("click", () => this.gotoNeighbor(-1));
    this.nextBtn.addEventListener("click", () => this.gotoNeighbor(1));

    this.desc = document.createElement("div");
    this.desc.className = "panel-desc";
    this.descBox = document.createElement("div");
    this.descBox.className = "info-box sample-desc";
    this.desc.appendChild(this.descBox);
    if (opts.about) {
      const about = document.createElement("details");
      about.className = "about-page";
      about.innerHTML =
        `<summary>About this page</summary><div class="about-page-body">${opts.about}</div>`;
      this.desc.appendChild(about);
    }

    this.body = document.createElement("div");
    this.body.className = "panel-body";

    this.tail = document.createElement("div");
    this.tail.className = "panel-tail";
    this.transportSlot = document.createElement("div");
    this.transportSlot.className = "panel-transport";
    this.switcherSlot = document.createElement("div");
    this.switcherSlot.className = "panel-switcher";
    this.simSlot = document.createElement("div");
    this.simSlot.className = "panel-sim";
    this.tail.append(this.transportSlot, this.switcherSlot, this.simSlot);

    root.append(this.head, this.desc, this.body, this.tail);
    panels.set(root, this);
    panels.set(this.body, this);
    this.render();
  }

  /** The registry entry currently shown (undefined on a route with no entry). */
  get entry(): SampleEntry | undefined {
    return this.entryValue;
  }

  /**
   * Show the sample for the page's scene key `scene`: header name / category /
   * C source / ◀▶ from its registry entry, its description, its canvas hint,
   * and the matching deep link + sidebar highlight.
   */
  setScene(scene: string): void {
    this.scene = scene;
    const entry = this.route ? findByRouteScene(this.route, scene) : undefined;
    if (entry) this.entryValue = entry;
    this.render();
    this.syncLocation();
  }

  /**
   * Same as {@link setScene} but resolved by the C sample name (for pages that
   * track the displayed sample by name). Names with no registry entry on this
   * route are ignored, so a page-level label never displaces the real sample.
   */
  setSampleName(name: string): void {
    const entry = this.route ? findByRouteName(this.route, name) : undefined;
    if (!entry) return;
    this.entryValue = entry;
    if (entry.scene) this.scene = entry.scene;
    this.render();
    this.syncLocation();
  }

  /**
   * The demoted in-page scene switcher: a compact <select> placed after the
   * sample's own controls. Option labels prefer the registry name so they read
   * exactly like the sidebar tree. Returns the element; `setScene` keeps its
   * value in sync with the scene on screen.
   */
  addSceneSelect(
    options: SceneOption[],
    value: string,
    onChange: (value: string) => void,
  ): HTMLSelectElement {
    this.switcherSlot.innerHTML = "";
    const group = document.createElement("div");
    group.className = "control-group scene-switcher";
    const id = `scene-switcher-${Math.random().toString(36).slice(2, 8)}`;
    const label = document.createElement("label");
    label.htmlFor = id;
    label.textContent = "Other samples on this page";
    const select = document.createElement("select");
    select.id = id;
    for (const o of options) {
      const name = (this.route && findByRouteScene(this.route, o.value)?.name) || o.label;
      select.add(new Option(name, o.value));
    }
    select.value = value;
    select.addEventListener("change", () => {
      // Drop focus so the global P/O/R keys and the scenes' WASD/arrow handlers
      // work again immediately (a focused <select> swallows them, and arrows
      // would silently switch scenes).
      select.blur();
      onChange(select.value);
    });
    group.append(label, select);
    this.switcherSlot.appendChild(group);
    this.sceneSelect = select;
    return select;
  }

  /** Temporarily replace the canvas hint (e.g. "Loading…"); null restores it. */
  setHintOverride(text: string | null): void {
    this.hintOverride = text;
    this.renderHint();
  }

  private gotoNeighbor(dir: -1 | 1): void {
    const entry = neighborOf(this.entryValue, dir);
    if (!entry || entry === this.entryValue) return;
    window.location.hash = entryHref(entry);
    // Emit for parity with the menu / keyboard paths (same bus events).
    demoBus.emit(dir === 1 ? "sim.nextSample" : "sim.prevSample", { entry });
  }

  private render(): void {
    const e = this.entryValue;
    this.nameEl.textContent = e?.name ?? this.opts.title;
    this.categoryEl.textContent = e?.category ?? this.opts.category ?? "";
    this.statusEl.hidden = e?.status !== "partial";
    if (e) {
      this.cSourceLink.href = cSourceUrl(e);
      this.cSourceLink.textContent = `C source: ${e.cSource} ↗`;
      this.cSourceLink.hidden = false;
    } else {
      this.cSourceLink.hidden = true;
    }
    const prev = neighborOf(e, -1);
    const next = neighborOf(e, 1);
    this.prevBtn.disabled = prev == null || prev === e;
    this.nextBtn.disabled = next == null || next === e;

    const info = this.opts.info;
    const html =
      typeof info === "string" ? info : info && this.scene != null ? (info[this.scene] ?? "") : "";
    this.descBox.innerHTML = html;
    this.descBox.hidden = html === "";

    if (this.sceneSelect && this.scene != null) {
      const has = Array.from(this.sceneSelect.options).some((o) => o.value === this.scene);
      if (has) this.sceneSelect.value = this.scene;
    }
    this.renderHint();
  }

  private renderHint(): void {
    const el = this.opts.hintEl;
    if (!el) return;
    const sceneHint = this.scene != null ? this.opts.sceneHints?.[this.scene] : undefined;
    el.innerHTML = this.hintOverride ?? sceneHint ?? this.opts.hint ?? "";
  }

  /** Point the deep link + sidebar at the sample on screen (no re-navigation). */
  private syncLocation(): void {
    const e = this.entryValue;
    if (!e?.route || !this.root.isConnected) return;
    // Only while this page still owns the hash (an async scene load could land
    // after the user has already navigated elsewhere).
    if (hashParts()[0] !== e.route) return;
    const href = entryHref(e);
    if (window.location.hash === href) return;
    history.replaceState(history.state, "", href);
    demoBus.emit("sample.active", { entry: e });
  }
}
