// Main entry — SPA shell mirroring the C Box3D samples app: a menu bar
// (Sim/View/Render/Samples/Help), a category→sample tree, and a hash router
// with per-scene deep links (`#/<route>/<slug>`). Draw-flag / render toggles
// dispatch through the exported `demoBus`; the interaction layer subscribes.

import { loadWasm } from "./wasm.ts";
import {
  samplesByCategory,
  categoryStats,
  totalStats,
  findByRouteSlug,
  firstEntryForRoute,
  entryHref,
  neighborOf,
  cSourceUrl,
  type SampleEntry,
  type CategoryStats,
} from "./registry.ts";
import { APP_VERSION, GIT_HASH, GIT_HASH_FULL } from "./version.ts";
// The typed event bus and its view/render state live in bus.ts so the
// interaction layer can import them without importing this entry module
// (which would form an import cycle: main → demos → interaction → main).
import {
  demoBus,
  emitInitialState,
  viewFlags,
  viewScales,
  renderState,
} from "./bus.ts";
// Three-free render-settings leaf. Imported directly (not via three-scene.ts) so
// the entry never pulls in Three.js and three-scene stays imported one way only.
import { applyRenderSettings } from "./render-settings.ts";
// The 16-bit view-flag table (labels + bit order + defaults) is the shared leaf
// consumed here, in bus.ts, and in interaction.ts — one source, no cycle.
import { MAIN_FLAG_LABELS, CONTACT_FLAG_LABELS } from "./view-flags.ts";
// Shared control primitives (menu builders reuse the same input-wiring / tick
// button as the side-panel controls).
import { buildRangeInput, createMenuTickItem } from "./controls.ts";

// ---------------------------------------------------------------------------
// Demo module registry. Each page's `init` may accept an optional initial
// scene key (the page's own `mode`/`scene`/`kind` value). Single-scene pages
// simply ignore the extra argument.
// ---------------------------------------------------------------------------

type DemoInit = (
  container: HTMLElement,
  initialScene?: string,
) => (() => void) | void;

const demoModules: Record<string, () => Promise<{ init: DemoInit }>> = {
  manifolds: () => import("./demos/manifolds.ts"),
  geometry: () => import("./demos/geometry.ts"),
  issues: () => import("./demos/issues.ts"),
  "height-field": () => import("./demos/height-field.ts"),
  mesh: () => import("./demos/mesh.ts"),
  compound: () => import("./demos/compound.ts"),
  stacking: () => import("./demos/stacking.ts"),
  tree: () => import("./demos/tree.ts"),
  benchmark: () => import("./demos/benchmark.ts"),
  bodies: () => import("./demos/bodies.ts"),
  ragdolls: () => import("./demos/ragdolls.ts"),
  robustness: () => import("./demos/robustness.ts"),
  joints: () => import("./demos/joints.ts"),
  continuous: () => import("./demos/continuous.ts"),
  determinism: () => import("./demos/determinism.ts"),
  sensors: () => import("./demos/sensors.ts"),
  shapes: () => import("./demos/shapes.ts"),
  queries: () => import("./demos/queries.ts"),
  world: () => import("./demos/world.ts"),
  replay: () => import("./demos/replay.ts"),
  character: () => import("./demos/character.ts"),
};

let currentCleanup: (() => void) | null = null;

// ---------------------------------------------------------------------------
// Hash routing. `#/` → home. `#/<route>` → page default scene.
// `#/<route>/<slug>` → deep link that selects the scene within a page.
// ---------------------------------------------------------------------------

interface ParsedRoute {
  route: string;
  slug?: string;
}

function parseHash(): ParsedRoute {
  const raw = window.location.hash.replace(/^#\/?/, "");
  if (raw === "") return { route: "home" };
  const parts = raw.split("/").filter(Boolean);
  return { route: parts[0] ?? "home", slug: parts[1] };
}

/** The registry entry the current hash points at, if any. */
function currentEntry(parsed: ParsedRoute): SampleEntry | undefined {
  if (parsed.route === "home") return undefined;
  if (parsed.slug) return findByRouteSlug(parsed.route, parsed.slug);
  return firstEntryForRoute(parsed.route);
}

// ---------------------------------------------------------------------------
// Sidebar tree — collapsible categories in C sort order.
// ---------------------------------------------------------------------------

const treeRoot = document.getElementById("sample-tree")!;
const expanded = new Set<string>();

// The registry is immutable at runtime, so its grouping / per-category stats are
// computed once and memoized. Every consumer (tree, home grid, Samples menu)
// reads through these instead of re-scanning SAMPLES on each navigation.
let g_byCat: Map<string, SampleEntry[]> | null = null;
function byCatMemo(): Map<string, SampleEntry[]> {
  return (g_byCat ??= samplesByCategory());
}
const g_catStats = new Map<string, CategoryStats>();
function catStatsMemo(category: string): CategoryStats {
  let stats = g_catStats.get(category);
  if (!stats) {
    stats = categoryStats(category);
    g_catStats.set(category, stats);
  }
  return stats;
}

function statusTag(status: SampleEntry["status"]): string {
  if (status === "live") return `<span class="s-tag s-live">LIVE</span>`;
  if (status === "partial") return `<span class="s-tag s-partial">PARTIAL</span>`;
  return `<span class="s-tag s-planned">PLANNED</span>`;
}

// --- Tree DOM, built once. Navigation only toggles active/open classes. ---
const itemKey = (route: string | undefined, slug: string): string => `${route ?? ""}/${slug}`;
const treeItemEls = new Map<string, HTMLElement>();
const treeCatEls = new Map<string, HTMLElement>();
let treeHomeEl: HTMLAnchorElement | null = null;
let activeItemEl: HTMLElement | null = null;
let treeBuilt = false;

function buildTree(): void {
  const byCat = byCatMemo();
  const frag = document.createDocumentFragment();

  const home = document.createElement("a");
  home.href = "#/";
  home.className = "tree-home";
  home.textContent = "Home";
  treeHomeEl = home;
  frag.appendChild(home);

  for (const [category, entries] of byCat) {
    const stats = catStatsMemo(category);
    const countClass = stats.live + stats.partial > 0 ? "has-live" : "";
    const catDiv = document.createElement("div");
    catDiv.className = "tree-cat";
    catDiv.dataset.cat = category;
    catDiv.innerHTML =
      `<button class="tree-cat-head" data-cat="${category}">` +
      `<span class="tree-chevron">▾</span>` +
      `<span class="tree-cat-name">${category}</span>` +
      `<span class="tree-cat-count ${countClass}">${stats.live + stats.partial}/${stats.total}</span>` +
      `</button>`;
    const body = document.createElement("div");
    body.className = "tree-cat-body";
    for (const e of entries) {
      const item = document.createElement(e.route ? "a" : "span");
      item.innerHTML = `<span class="tree-item-name">${escapeHtml(e.name)}</span>${statusTag(e.status)}`;
      if (e.route) {
        (item as HTMLAnchorElement).href = entryHref(e);
        item.className = `tree-item ${e.status}`;
        treeItemEls.set(itemKey(e.route, e.slug), item);
      } else {
        item.className = "tree-item planned";
        item.title = "Not ported yet";
      }
      body.appendChild(item);
    }
    catDiv.appendChild(body);
    treeCatEls.set(category, catDiv);
    frag.appendChild(catDiv);
  }

  treeRoot.innerHTML = "";
  treeRoot.appendChild(frag);
  treeBuilt = true;
}

/** Reflect the current route in the pre-built tree: active item + open cats. */
function renderTree(active?: SampleEntry): void {
  if (!treeBuilt) buildTree();

  const atHome = location.hash === "" || location.hash === "#/";
  treeHomeEl?.classList.toggle("active", atHome);

  if (activeItemEl) activeItemEl.classList.remove("active");
  activeItemEl = null;
  if (active?.route) {
    const el = treeItemEls.get(itemKey(active.route, active.slug));
    if (el) {
      el.classList.add("active");
      activeItemEl = el;
    }
  }

  // The `expanded` set is the source of truth for open categories (delegated
  // clicks + the navigate() auto-open both write it); mirror it onto the DOM.
  for (const [category, el] of treeCatEls) {
    el.classList.toggle("open", expanded.has(category));
  }
}

function escapeHtml(s: string): string {
  return s.replace(/[&<>"']/g, (c) =>
    c === "&" ? "&amp;" : c === "<" ? "&lt;" : c === ">" ? "&gt;" : c === '"' ? "&quot;" : "&#39;",
  );
}

// Toggle categories (event-delegated so it survives re-renders).
treeRoot.addEventListener("click", (e) => {
  const head = (e.target as HTMLElement).closest(".tree-cat-head") as HTMLElement | null;
  if (head) {
    const cat = head.dataset.cat!;
    if (expanded.has(cat)) expanded.delete(cat);
    else expanded.add(cat);
    head.parentElement!.classList.toggle("open", expanded.has(cat));
    return;
  }
  if ((e.target as HTMLElement).closest(".tree-item, .tree-home")) closeSidebar();
});

// ---------------------------------------------------------------------------
// Menu bar (Sim / View / Render / Samples / Help).
// ---------------------------------------------------------------------------

const menubar = document.getElementById("menubar")!;
let openMenu: HTMLElement | null = null;
// When a hover switches menus while one is open, the follow-up click on the
// same button must not toggle it closed. Track the last hover-opened menu.
let pointerOpened: HTMLElement | null = null;

function closeMenus(): void {
  if (openMenu) {
    openMenu.classList.remove("open");
    openMenu = null;
  }
  // Drop any stale hover-open marker so the next click on that button isn't swallowed.
  pointerOpened = null;
}

function checkItem(label: string, key: string, checked: boolean, onToggle: (v: boolean) => void): HTMLElement {
  const { el, tick } = createMenuTickItem(label, checked, "✓", key);
  el.addEventListener("click", (ev) => {
    ev.stopPropagation();
    const next = tick.textContent !== "✓";
    tick.textContent = next ? "✓" : "";
    onToggle(next);
  });
  return el;
}

function actionItem(label: string, shortcut: string, onClick: () => void): HTMLElement {
  const el = document.createElement("button");
  el.className = "menu-item";
  el.innerHTML = `<span>${label}</span><span class="menu-shortcut">${shortcut}</span>`;
  el.addEventListener("click", (ev) => {
    ev.stopPropagation();
    closeMenus();
    onClick();
  });
  return el;
}

function radioItem(label: string, checked: boolean, group: HTMLElement[], onPick: () => void): HTMLElement {
  const { el, tick } = createMenuTickItem(label, checked, "●");
  el.addEventListener("click", (ev) => {
    ev.stopPropagation();
    for (const g of group) g.querySelector(".menu-tick")!.textContent = "";
    tick.textContent = "●";
    onPick();
  });
  return el;
}

function sliderItem(
  label: string,
  min: number,
  max: number,
  step: number,
  value: number,
  fmt: (v: number) => string,
  onInput: (v: number) => void,
): HTMLElement {
  const el = document.createElement("div");
  el.className = "menu-slider";
  const val = document.createElement("span");
  val.className = "menu-slider-val";
  const head = document.createElement("div");
  head.className = "menu-slider-head";
  head.innerHTML = `<span>${label}</span>`;
  head.appendChild(val);
  const input = buildRangeInput(min, max, step, value, val, fmt, onInput, { stopClick: true });
  el.appendChild(head);
  el.appendChild(input);
  return el;
}

function separator(): HTMLElement {
  const el = document.createElement("div");
  el.className = "menu-sep";
  return el;
}

function subLabel(text: string): HTMLElement {
  const el = document.createElement("div");
  el.className = "menu-sublabel";
  el.textContent = text;
  return el;
}

/** Build one top-level menu; `fill` populates the dropdown panel. */
function buildMenu(label: string, fill: (panel: HTMLElement) => void, wide = false): HTMLElement {
  const menu = document.createElement("div");
  menu.className = "menu";
  const button = document.createElement("button");
  button.className = "menu-button";
  button.textContent = label;
  const panel = document.createElement("div");
  panel.className = "menu-panel" + (wide ? " wide" : "");
  fill(panel);
  button.addEventListener("click", (ev) => {
    ev.stopPropagation();
    // If a hover just opened this menu, the click should keep it open, not close.
    if (pointerOpened === menu) {
      pointerOpened = null;
      return;
    }
    if (openMenu === menu) {
      closeMenus();
    } else {
      closeMenus();
      menu.classList.add("open");
      openMenu = menu;
    }
  });
  button.addEventListener("mouseenter", () => {
    if (openMenu && openMenu !== menu) {
      closeMenus();
      menu.classList.add("open");
      openMenu = menu;
      pointerOpened = menu;
    }
  });
  menu.appendChild(button);
  menu.appendChild(panel);
  return menu;
}

function selectSampleByOffset(dir: -1 | 1): SampleEntry | null {
  const entry = neighborOf(currentEntry(parseHash()), dir);
  if (entry) window.location.hash = entryHref(entry);
  return entry;
}

function buildMenuBar(): void {
  menubar.innerHTML = "";

  const brand = document.createElement("a");
  brand.className = "menubar-brand";
  brand.href = "#/";
  brand.textContent = "Box3D.rs";
  menubar.appendChild(brand);

  // --- Sim ---
  menubar.appendChild(
    buildMenu("Sim", (p) => {
      p.appendChild(actionItem("Pause", "P", () => demoBus.emit("sim.pause")));
      p.appendChild(actionItem("Single Step", "O", () => demoBus.emit("sim.step")));
      p.appendChild(actionItem("Restart", "R", () => demoBus.emit("sim.restart")));
      p.appendChild(separator());
      p.appendChild(
        actionItem("Previous Sample", "[", () => {
          const entry = selectSampleByOffset(-1);
          demoBus.emit("sim.prevSample", { entry });
        }),
      );
      p.appendChild(
        actionItem("Next Sample", "]", () => {
          const entry = selectSampleByOffset(1);
          demoBus.emit("sim.nextSample", { entry });
        }),
      );
    }),
  );

  // --- View ---
  menubar.appendChild(
    buildMenu("View", (p) => {
      for (const [key, label] of MAIN_FLAG_LABELS) {
        p.appendChild(
          checkItem(label, key, viewFlags[key]!, (v) => {
            viewFlags[key] = v;
            demoBus.emit("view.flag", { name: key, value: v });
          }),
        );
      }
      p.appendChild(separator());
      for (const [key, label] of CONTACT_FLAG_LABELS) {
        p.appendChild(
          checkItem(label, key, viewFlags[key]!, (v) => {
            viewFlags[key] = v;
            demoBus.emit("view.flag", { name: key, value: v });
          }),
        );
      }
      // Anchor A / B radio (C drawAnchorA: 1 = A, 0 = B; default B).
      p.appendChild(subLabel("Anchor"));
      const anchorGroup: HTMLElement[] = [];
      const anchorA = radioItem("Anchor A", viewFlags.anchorA === true, anchorGroup, () => {
        viewFlags.anchorA = true;
        demoBus.emit("view.flag", { name: "anchorA", value: true });
      });
      const anchorB = radioItem("Anchor B", viewFlags.anchorA === false, anchorGroup, () => {
        viewFlags.anchorA = false;
        demoBus.emit("view.flag", { name: "anchorA", value: false });
      });
      anchorGroup.push(anchorA, anchorB);
      p.appendChild(anchorA);
      p.appendChild(anchorB);
      p.appendChild(separator());
      p.appendChild(
        actionItem("Diagnostics", "M", () => demoBus.emit("ui.metrics")),
      );
      p.appendChild(subLabel("Scale"));
      p.appendChild(
        sliderItem("Joint", 0, 2, 0.05, viewScales.joint!, (v) => v.toFixed(2), (v) => {
          viewScales.joint = v;
          demoBus.emit("view.scale", { name: "joint", value: v });
        }),
      );
      p.appendChild(
        sliderItem("Force", 0, 1, 0.01, viewScales.force!, (v) => v.toFixed(2), (v) => {
          viewScales.force = v;
          demoBus.emit("view.scale", { name: "force", value: v });
        }),
      );
      p.appendChild(
        sliderItem("Draw Distance", 10, 1000, 10, viewScales.drawDistance!, (v) => `${v.toFixed(0)} m`, (v) => {
          viewScales.drawDistance = v;
          demoBus.emit("view.scale", { name: "drawDistance", value: v });
        }),
      );
    }),
  );

  // --- Render ---
  menubar.appendChild(
    buildMenu("Render", (p) => {
      p.appendChild(
        checkItem("Shadows", "shadows", renderState.shadows, (v) => {
          renderState.shadows = v;
          demoBus.emit("render.setting", { key: "shadows", value: v });
        }),
      );
      p.appendChild(
        checkItem("GTAO", "gtao", renderState.gtao, (v) => {
          renderState.gtao = v;
          demoBus.emit("render.setting", { key: "gtao", value: v });
        }),
      );
      p.appendChild(subLabel("GTAO Quality"));
      const qNames = ["Medium", "High", "Ultra"];
      const qGroup: HTMLElement[] = [];
      qNames.forEach((name, i) => {
        const item = radioItem(name, renderState.gtaoQuality === i, qGroup, () => {
          renderState.gtaoQuality = i;
          demoBus.emit("render.setting", { key: "gtaoQuality", value: i });
        });
        qGroup.push(item);
        p.appendChild(item);
      });
      p.appendChild(separator());
      p.appendChild(
        checkItem("IBL", "ibl", renderState.ibl, (v) => {
          renderState.ibl = v;
          demoBus.emit("render.setting", { key: "ibl", value: v });
        }),
      );
      p.appendChild(
        sliderItem("Exposure", -8, 4, 0.25, renderState.exposureStops, (v) => `${v.toFixed(2)} EV`, (v) => {
          renderState.exposureStops = v;
          demoBus.emit("render.setting", { key: "exposureStops", value: v });
        }),
      );
      p.appendChild(
        sliderItem("Sun", 0, 1, 0.05, renderState.sunStrength, (v) => v.toFixed(2), (v) => {
          renderState.sunStrength = v;
          demoBus.emit("render.setting", { key: "sunStrength", value: v });
        }),
      );
      p.appendChild(separator());
      p.appendChild(subLabel("Debug View"));
      const dvNames = ["0 — lit", "1 — view distance", "2 — cascade", "3 — normal", "4 — raw AO"];
      const dvGroup: HTMLElement[] = [];
      dvNames.forEach((name, i) => {
        const item = radioItem(name, renderState.debugView === i, dvGroup, () => {
          renderState.debugView = i;
          demoBus.emit("render.setting", { key: "debugView", value: i });
        });
        dvGroup.push(item);
        p.appendChild(item);
      });
    }),
  );

  // --- Samples (the same tree as a dropdown) ---
  menubar.appendChild(
    buildMenu(
      "Samples",
      (p) => {
        const byCat = byCatMemo();
        for (const [category, entries] of byCat) {
          const stats = catStatsMemo(category);
          const cat = document.createElement("div");
          cat.className = "menu-cat";
          const head = document.createElement("button");
          head.className = "menu-item menu-cat-head";
          head.innerHTML =
            `<span class="menu-tick"></span><span>${category}</span>` +
            `<span class="menu-shortcut">${stats.live + stats.partial}/${stats.total}</span>`;
          head.addEventListener("click", (ev) => {
            ev.stopPropagation();
            cat.classList.toggle("open");
          });
          cat.appendChild(head);
          const body = document.createElement("div");
          body.className = "menu-cat-body";
          for (const e of entries) {
            const item = document.createElement(e.route ? "a" : "span");
            item.className = `menu-item menu-sample ${e.status}`;
            item.innerHTML = `<span>${escapeHtml(e.name)}</span>${statusTag(e.status)}`;
            if (e.route) {
              (item as HTMLAnchorElement).href = entryHref(e);
              item.addEventListener("click", () => closeMenus());
            }
            body.appendChild(item);
          }
          cat.appendChild(body);
          p.appendChild(cat);
        }
      },
      true,
    ),
  );

  // --- Help ---
  menubar.appendChild(
    buildMenu(
      "Help",
      (p) => {
        p.appendChild(subLabel("Keyboard"));
        const keys: [string, string][] = [
          ["P", "Pause"],
          ["O / Shift+O", "Single / 5× step"],
          ["R", "Restart sample"],
          ["[ / ]", "Previous / next sample"],
          ["F", "Frame selection"],
          ["Tab", "Hide UI"],
          ["M", "Diagnostics"],
          ["Ctrl+O", "Sample picker"],
          ["Esc", "Clear selection"],
        ];
        const mouse: [string, string][] = [
          ["Scroll", "Zoom to cursor"],
          ["Right-drag", "Orbit at cursor"],
          ["Middle-drag", "Pan at cursor"],
          ["Alt + drag", "Orbit / pan / zoom (C)"],
          ["Ctrl + click", "Grab body"],
          ["Shift + click", "Spawn body"],
        ];
        const table = document.createElement("table");
        table.className = "help-table";
        for (const [k, v] of [...keys, ...mouse]) {
          const tr = document.createElement("tr");
          tr.innerHTML = `<td>${k}</td><td>${v}</td>`;
          table.appendChild(tr);
        }
        p.appendChild(table);
        p.appendChild(separator());
        p.appendChild(subLabel("About"));
        const about = document.createElement("p");
        about.className = "menu-about";
        about.innerHTML =
          "A pure-Rust port of Erin Catto's <strong>Box3D</strong>, exact to the C math and " +
          "deterministic trig. Solver defaults: 4 substeps, 60 Hz, 5 cm contact recycle, " +
          "sleep/warm-start/continuous on. Ported by Lars Brubaker, sponsored by MatterHackers.";
        p.appendChild(about);
      },
      true,
    ),
  );

  // Close menus on outside click / Escape.
  document.addEventListener("click", () => closeMenus());
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") closeMenus();
  });
}

// ---------------------------------------------------------------------------
// Mobile sidebar toggle.
// ---------------------------------------------------------------------------

const menuToggle = document.getElementById("menu-toggle")!;
const sidebar = document.getElementById("sidebar")!;
const sidebarOverlay = document.getElementById("sidebar-overlay")!;

function openSidebar(): void {
  sidebar.classList.add("open");
  menuToggle.classList.add("open");
  sidebarOverlay.classList.add("visible");
}
function closeSidebar(): void {
  sidebar.classList.remove("open");
  menuToggle.classList.remove("open");
  sidebarOverlay.classList.remove("visible");
}
menuToggle.addEventListener("click", () => {
  if (sidebar.classList.contains("open")) closeSidebar();
  else openSidebar();
});
sidebarOverlay.addEventListener("click", closeSidebar);

// ---------------------------------------------------------------------------
// Home page — registry-derived category grid.
// ---------------------------------------------------------------------------

function renderHome(container: HTMLElement): void {
  const total = totalStats();
  const byCat = byCatMemo();

  const cards: string[] = [];
  for (const [category, entries] of byCat) {
    const stats = catStatsMemo(category);
    const first = entries.find((e) => e.route);
    const done = stats.live + stats.partial;
    const badge =
      stats.live > 0
        ? `<span class="card-badge badge-live">LIVE</span>`
        : done > 0
          ? `<span class="card-badge badge-partial">PARTIAL</span>`
          : `<span class="card-badge badge-planned">PLANNED</span>`;
    const tag = first ? "a" : "div";
    const href = first ? ` href="${entryHref(first)}"` : "";
    cards.push(
      `<${tag} class="cat-card${first ? "" : " disabled"}"${href}>` +
        badge +
        `<h3>${category}</h3>` +
        `<div class="cat-meter"><span class="cat-meter-fill" style="width:${
          stats.total ? Math.round((done / stats.total) * 100) : 0
        }%"></span></div>` +
        `<p class="cat-count">${done} of ${stats.total} in-browser</p>` +
        `</${tag}>`,
    );
  }

  container.innerHTML = `
    <div class="home-page">
      <div class="github-badge">
        <a href="https://github.com/larsbrubaker/box3d-rust" target="_blank" class="github-badge-link">
          <svg height="20" viewBox="0 0 16 16" width="20" fill="currentColor"><path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0016 8c0-4.42-3.58-8-8-8z"/></svg>
          <span>larsbrubaker/box3d-rust</span>
        </a>
      </div>
      <div class="hero">
        <h1>Box3D <span>for Rust</span></h1>
        <p>
          A pure Rust port of Erin Catto's Box3D physics engine with exact behavioral
          matching, including its cross-platform deterministic math. This site mirrors
          the structure of Erin's C samples app — pick a category and sample from the tree.
        </p>
      </div>

      <h2 style="font-size:18px;font-weight:700;margin-bottom:4px;">
        Sample coverage
        <span class="badge-live">${total.live} exact</span>
        <span class="badge-partial">${total.partial} partial</span>
        <span class="badge-planned">${total.planned} planned</span>
      </h2>
      <p style="color:var(--text-secondary);font-size:13px;margin-bottom:16px;">
        ${total.total} samples across ${byCat.size} categories, one per upstream
        <code>RegisterSample</code>. Honest status: <em>exact</em> means bit-matching the C
        sample; <em>partial</em> means a working scene that still diverges; <em>planned</em>
        is not yet ported.
      </p>
      <div class="cat-grid">
        ${cards.join("")}
      </div>

      <div class="about-section">
        <h2>About This Project</h2>
        <p>
          Module-by-module Rust port of
          <a href="https://github.com/erincatto/box3d" target="_blank">Box3D</a> by Erin Catto,
          with the C test suite ported alongside each module. Dynamics samples run here in
          wasm, following the path our finished
          <a href="https://larsbrubaker.github.io/box2d-rust/" target="_blank">box2d-rust</a> demos took.
        </p>
        <p style="margin-top: 12px">
          Ported by <strong>Lars Brubaker</strong>, sponsored by
          <a href="https://www.matterhackers.com" target="_blank">MatterHackers</a>.
        </p>
        <div class="stats-row">
          <div class="stat">
            <div class="stat-value" id="stat-version">…</div>
            <div class="stat-label">Port version</div>
          </div>
          <div class="stat">
            <div class="stat-value">${total.live}</div>
            <div class="stat-label">Exact samples</div>
          </div>
          <div class="stat">
            <div class="stat-value">f32 + f64</div>
            <div class="stat-label">Precision modes</div>
          </div>
          <div class="stat">
            <div class="stat-value">WASM</div>
            <div class="stat-label">In-browser</div>
          </div>
        </div>
      </div>
    </div>
  `;

  loadWasm()
    .then((wasm) => {
      const el = document.getElementById("stat-version");
      if (el) el.textContent = `v${wasm.version()}`;
    })
    .catch(() => {});
}

// ---------------------------------------------------------------------------
// Navigation.
// ---------------------------------------------------------------------------

async function navigate(): Promise<void> {
  const parsed = parseHash();
  const container = document.getElementById("main-content")!;

  // Legacy alias: the old `#/far-pyramid` route is now a scene of the World page
  // (`#/world/far-pyramid`). Rewrite the hash and let the hashchange re-navigate.
  if (parsed.route === "far-pyramid") {
    window.location.hash = "#/world/far-pyramid";
    return;
  }

  // Legacy alias: the old single-sample `#/hull` route (Box Hull) is now the
  // Box Hull scene of the Geometry page (`#/geometry/box-hull`).
  if (parsed.route === "hull") {
    window.location.hash = "#/geometry/box-hull";
    return;
  }

  if (currentCleanup) {
    currentCleanup();
    currentCleanup = null;
  }

  const active = currentEntry(parsed);
  if (active && !expanded.has(active.category)) expanded.add(active.category);
  renderTree(active);

  if (parsed.route === "home") {
    renderHome(container);
    return;
  }

  const loader = demoModules[parsed.route];
  if (!loader) {
    container.innerHTML = `<div class="home-page"><h2>Page not found</h2><p>Unknown route: ${escapeHtml(
      parsed.route,
    )}</p></div>`;
    return;
  }

  container.innerHTML = `<div class="home-page" style="display:flex;align-items:center;justify-content:center;height:80vh;"><p style="color:var(--text-muted)">Loading demo...</p></div>`;

  try {
    await loadWasm();
    const mod = await loader();
    container.innerHTML = "";
    // Deep-link scene: prefer the resolved entry's scene, else the raw slug.
    const scene = active?.scene ?? parsed.slug;
    const cleanup = mod.init(container, scene);
    if (cleanup) currentCleanup = cleanup;
    ensureSampleNav(container, active);
  } catch (e) {
    console.error("Failed to load demo:", e);
    container.innerHTML = `<div class="home-page"><h2>Error loading demo</h2><pre style="color:var(--clip-stroke)">${escapeHtml(
      String(e),
    )}</pre></div>`;
  }
}

/**
 * Fallback Info-panel nav header (◀ name ▶ + "C source" link) for sample pages
 * that build their own panel instead of using the shared `attachInteraction`
 * header (which already carries these). Skipped when that header is present (its
 * `.sample-csource` marks it), so the 13 dynamics pages keep their integrated,
 * per-scene version and every other sample page still gets the same affordance.
 * The buttons walk the shared C-sorted `NAVIGABLE_SAMPLES` order via `neighborOf`,
 * anchored to the deep-link entry (like the `[`/`]` keys and Sim menu).
 */
function ensureSampleNav(container: HTMLElement, entry: SampleEntry | undefined): void {
  if (!entry) return;
  const panel = container.querySelector(".demo-controls") as HTMLElement | null;
  if (!panel || panel.querySelector(".sample-csource")) return;

  const header = document.createElement("div");
  header.className = "sample-nav-header";
  header.innerHTML =
    `<div class="sample-title-row">` +
    `<button class="sample-nav-btn sample-prev" type="button" title="Previous sample ([)" aria-label="Previous sample">◀</button>` +
    `<span class="sample-name">${escapeHtml(entry.name)}</span>` +
    `<button class="sample-nav-btn sample-next" type="button" title="Next sample (])" aria-label="Next sample">▶</button>` +
    `</div>` +
    `<a class="sample-csource" href="${cSourceUrl(entry)}" target="_blank" rel="noopener">C source: ${escapeHtml(
      entry.cSource,
    )} ↗</a>`;

  const prevBtn = header.querySelector(".sample-prev") as HTMLButtonElement;
  const nextBtn = header.querySelector(".sample-next") as HTMLButtonElement;
  const prev = neighborOf(entry, -1);
  const next = neighborOf(entry, 1);
  prevBtn.disabled = prev == null || prev === entry;
  nextBtn.disabled = next == null || next === entry;
  prevBtn.addEventListener("click", () => {
    if (prev && prev !== entry) window.location.hash = entryHref(prev);
  });
  nextBtn.addEventListener("click", () => {
    if (next && next !== entry) window.location.hash = entryHref(next);
  });
  panel.prepend(header);
}

// ---------------------------------------------------------------------------
// Boot.
// ---------------------------------------------------------------------------

buildMenuBar();

// --- Sidebar footer: build provenance -------------------------------------
// "v<version> · <hash>" with the hash linking to its GitHub commit. Injected
// from build-time constants (src/version.ts); degrades to just the version when
// git was unavailable at bundle time (GIT_HASH empty → fallback text).
function renderBuildInfo(): void {
  const el = document.getElementById("build-info");
  if (!el) return;
  const version = `v${escapeHtml(APP_VERSION)}`;
  if (GIT_HASH && GIT_HASH_FULL) {
    el.innerHTML =
      `${version} · <a href="https://github.com/larsbrubaker/box3d-rust/commit/${escapeHtml(
        GIT_HASH_FULL,
      )}" target="_blank" rel="noopener">${escapeHtml(GIT_HASH)}</a>`;
  } else {
    el.textContent = version;
  }
}
renderBuildInfo();

// --- Render settings: global consumer -------------------------------------
// The Render menu affects every DemoScene route, not just the dynamics demos
// that attach the interaction layer, so render.setting is wired here (globally)
// rather than in interaction.ts. applyRenderSettings updates the shared
// RenderSettings and pushes them to the active scene. It lives in the three-free
// render-settings leaf, so importing it here does NOT pull Three.js into the
// landing page — and, crucially, keeps main.ts from importing three-scene.ts,
// which (imported both statically by demos and dynamically here) made Bun's
// dev-bundle splitting double-emit three-scene's exports (duplicate-export crash).
demoBus.on("render.setting", ({ key, value }) => {
  applyRenderSettings({ [key]: value });
});

// --- View-menu ticks: reflect flag changes from anywhere ------------------
// The side-panel Debug-draw checkboxes emit the same `view.flag` event the menu
// does, so the two UIs share one flag state. Keep the menu's check ticks in sync
// when the flag flips from the panel (or any future emitter). Anchor A/B are
// radios that manage their own group and carry no `data-flag`, so they're left
// untouched here; render-menu items use disjoint keys and ignore view.flag.
demoBus.on("view.flag", ({ name, value }) => {
  const tick = menubar.querySelector(`.menu-check[data-flag="${name}"] .menu-tick`);
  if (tick) tick.textContent = value ? "✓" : "";
});

emitInitialState();

// --- Global keyboard shortcuts (C samples: main.cpp:152-274) ----------------
// P pause, O step (Shift+O = 5), R restart, [ / ] prev/next sample, F frame,
// M diagnostics, Tab hide/show UI chrome. Dispatched via demoBus so the menu
// and the keys share one path. Ctrl+O fuzzy picker is out of scope this wave.
// Space stays unbound on purpose (the Mover sample uses it to jump), matching C
// main.cpp:204-206; S stays unbound too (camera fly + character strafe own it).
let chromeHidden = false;

function toggleChrome(): void {
  chromeHidden = !chromeHidden;
  const menubarEl = document.getElementById("menubar");
  const sidebarEl = document.getElementById("sidebar");
  const mainEl = document.getElementById("main-content");
  const demoPageEl = mainEl?.querySelector(".demo-page") as HTMLElement | null;
  const controlsEl = mainEl?.querySelector(".demo-controls") as HTMLElement | null;
  const metricsEl = mainEl?.querySelector(".sample-metrics") as HTMLElement | null;
  if (chromeHidden) {
    if (menubarEl) menubarEl.style.display = "none";
    if (sidebarEl) sidebarEl.style.display = "none";
    if (mainEl) {
      mainEl.style.marginLeft = "0";
      mainEl.style.marginTop = "0";
    }
    if (demoPageEl) demoPageEl.style.height = "100vh";
    if (controlsEl) controlsEl.style.display = "none";
    if (metricsEl) metricsEl.style.display = "none";
  } else {
    if (menubarEl) menubarEl.style.display = "";
    if (sidebarEl) sidebarEl.style.display = "";
    if (mainEl) {
      mainEl.style.marginLeft = "";
      mainEl.style.marginTop = "";
    }
    if (demoPageEl) demoPageEl.style.height = "";
    if (controlsEl) controlsEl.style.display = "";
    if (metricsEl) metricsEl.style.display = "";
  }
}

window.addEventListener("keydown", (e) => {
  // Never steal keys from text entry.
  const tag = (e.target as HTMLElement)?.tagName;
  if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
  if (e.ctrlKey || e.metaKey || e.altKey) return; // leave modified combos alone

  switch (e.code) {
    case "KeyP":
      e.preventDefault();
      demoBus.emit("sim.pause");
      break;
    case "KeyO":
      e.preventDefault();
      // Shift+O single-steps five times (C main.cpp:199).
      if (e.shiftKey) for (let i = 0; i < 5; i++) demoBus.emit("sim.step");
      else demoBus.emit("sim.step");
      break;
    case "KeyR":
      e.preventDefault();
      demoBus.emit("sim.restart");
      break;
    case "KeyF":
      e.preventDefault();
      demoBus.emit("sim.frame");
      break;
    case "KeyM":
      e.preventDefault();
      demoBus.emit("ui.metrics");
      break;
    case "Tab":
      e.preventDefault();
      toggleChrome();
      break;
    case "BracketLeft": {
      e.preventDefault();
      const entry = selectSampleByOffset(-1);
      demoBus.emit("sim.prevSample", { entry });
      break;
    }
    case "BracketRight": {
      e.preventDefault();
      const entry = selectSampleByOffset(1);
      demoBus.emit("sim.nextSample", { entry });
      break;
    }
    default:
      break;
  }
});

window.addEventListener("hashchange", () => navigate());
navigate();
