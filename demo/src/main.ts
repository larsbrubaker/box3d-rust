// Main entry — SPA router and WASM initialization (box2d-rust shell, Box3D content).

import { loadWasm } from "./wasm.ts";

type DemoInit = (container: HTMLElement) => (() => void) | void;
const demoModules: Record<string, () => Promise<{ init: DemoInit }>> = {
  math: () => import("./demos/math.ts"),
  geometry: () => import("./demos/geometry.ts"),
  manifolds: () => import("./demos/manifolds.ts"),
  hull: () => import("./demos/hull.ts"),
  "height-field": () => import("./demos/height-field.ts"),
  mesh: () => import("./demos/mesh.ts"),
  tree: () => import("./demos/tree.ts"),
  bodies: () => import("./demos/bodies.ts"),
  compound: () => import("./demos/compound.ts"),
  stacking: () => import("./demos/stacking.ts"),
  benchmark: () => import("./demos/benchmark.ts"),
  ragdolls: () => import("./demos/ragdolls.ts"),
  joints: () => import("./demos/joints.ts"),
  continuous: () => import("./demos/continuous.ts"),
  sensors: () => import("./demos/sensors.ts"),
  queries: () => import("./demos/queries.ts"),
  terrain: () => import("./demos/terrain.ts"),
  character: () => import("./demos/character.ts"),
  roadmap: () => import("./demos/roadmap.ts"),
};

let currentCleanup: (() => void) | null = null;

const menuToggle = document.getElementById("menu-toggle")!;
const sidebar = document.getElementById("sidebar")!;
const sidebarOverlay = document.getElementById("sidebar-overlay")!;

function openSidebar() {
  sidebar.classList.add("open");
  menuToggle.classList.add("open");
  sidebarOverlay.classList.add("visible");
}

function closeSidebar() {
  sidebar.classList.remove("open");
  menuToggle.classList.remove("open");
  sidebarOverlay.classList.remove("visible");
}

menuToggle.addEventListener("click", () => {
  if (sidebar.classList.contains("open")) closeSidebar();
  else openSidebar();
});
sidebarOverlay.addEventListener("click", closeSidebar);
document.querySelectorAll(".nav-link").forEach((link) => {
  link.addEventListener("click", closeSidebar);
});

function getRoute(): string {
  const hash = window.location.hash.slice(2) || "";
  return hash || "home";
}

function updateNav(route: string) {
  document.querySelectorAll(".nav-link").forEach((el) => {
    const r = (el as HTMLElement).dataset.route;
    el.classList.toggle("active", r === route);
  });
}

function renderHome(container: HTMLElement) {
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
          matching, including its cross-platform deterministic math. The collision layer
          is live in your browser via WebAssembly — explore the demos below.
        </p>
      </div>

      <h2 style="font-size:18px;font-weight:700;margin-bottom:12px;">
        Live now <span class="badge-live">17 demos</span>
      </h2>
      <div class="feature-grid">
        <a href="#/math" class="feature-card">
          <span class="card-icon">&#9881;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Deterministic Math</h3>
          <p>Hand-rolled <code>b3Atan2</code> / <code>b3ComputeCosSin</code> — bit-identical across platforms.</p>
        </a>
        <a href="#/geometry" class="feature-card">
          <span class="card-icon">&#10140;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Geometry Queries</h3>
          <p>Ray casts vs sphere, capsule, hull, and AABB, plus GJK closest points.</p>
        </a>
        <a href="#/manifolds" class="feature-card">
          <span class="card-icon">&#9649;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Contact Manifolds</h3>
          <p>Sphere, capsule, and hull contact points &amp; normals from the narrow phase.</p>
        </a>
        <a href="#/hull" class="feature-card">
          <span class="card-icon">&#11042;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Hull</h3>
          <p><code>b3MakeBoxHull</code> and <code>b3CreateHull</code> wireframes from the Rust half-edge mesh.</p>
        </a>
        <a href="#/height-field" class="feature-card">
          <span class="card-icon">&#8776;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Height Field</h3>
          <p>Wave height field with live ray cast hits from the ported HF module.</p>
        </a>
        <a href="#/mesh" class="feature-card">
          <span class="card-icon">&#9638;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Mesh</h3>
          <p>Box and grid triangle meshes with BVH ray casts and AABB.</p>
        </a>
        <a href="#/tree" class="feature-card">
          <span class="card-icon">&#9752;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Dynamic Tree</h3>
          <p>Broad-phase AABB tree: insert proxies, watch query hits and metrics.</p>
        </a>
        <a href="#/bodies" class="feature-card">
          <span class="card-icon">&#9632;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Bodies</h3>
          <p>Live <code>World::step</code> with gravity, collide, and contact solve.</p>
        </a>
        <a href="#/compound" class="feature-card">
          <span class="card-icon">&#9638;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Compound</h3>
          <p>Simple / Spheres / Hulls / Village — compound shapes from the C samples.</p>
        </a>
        <a href="#/stacking" class="feature-card">
          <span class="card-icon">&#8801;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Stacking</h3>
          <p>Jenga Stack, Box Stack, Pyramid2D (planar), and Sphere Stack from the scalar solver.</p>
        </a>
        <a href="#/benchmark" class="feature-card">
          <span class="card-icon">&#9881;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Benchmark</h3>
          <p>Large Pyramid, Junkyard, and Falling Trees — browser-scaled from benchmarks.c.</p>
        </a>
        <a href="#/ragdolls" class="feature-card">
          <span class="card-icon">&#9823;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Ragdolls</h3>
          <p>Capsule-bone humans with spherical/revolute joints — determinism soak in the browser.</p>
        </a>
        <a href="#/joints" class="feature-card">
          <span class="card-icon">&#9878;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Joints</h3>
          <p>Ball-and-chain and a revolute hinge with motor on/off.</p>
        </a>
        <a href="#/continuous" class="feature-card">
          <span class="card-icon">&#9889;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Continuous</h3>
          <p>Bullet vs thin wall — toggle CCD and watch tunneling.</p>
        </a>
        <a href="#/sensors" class="feature-card">
          <span class="card-icon">&#9673;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Sensors</h3>
          <p>Begin/end touch events as spheres fall through a sensor volume.</p>
        </a>
        <a href="#/queries" class="feature-card">
          <span class="card-icon">&#9678;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Queries</h3>
          <p>Animated closest ray cast with hit point and normal.</p>
        </a>
        <a href="#/terrain" class="feature-card">
          <span class="card-icon">&#9650;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Terrain Settle</h3>
          <p>Bodies settle on a grid mesh or wave height field.</p>
        </a>
        <a href="#/character" class="feature-card">
          <span class="card-icon">&#9823;</span>
          <span class="card-badge badge-live">LIVE</span>
          <h3>Character</h3>
          <p>BasicMover capsule with pogo ray, static capsules, and Village walk mode.</p>
        </a>
        <a href="#/roadmap" class="feature-card">
          <span class="card-icon">&#9776;</span>
          <h3>Demo Roadmap</h3>
          <p>Every upstream sample category with LIVE vs PLANNED badges as the port advances.</p>
        </a>
      </div>

      <div class="about-section">
        <h2>About This Project</h2>
        <p>
          This is a module-by-module Rust port of
          <a href="https://github.com/erincatto/box3d" target="_blank">Box3D</a> by Erin Catto,
          with the C test suite ported alongside each module. Dynamics samples — ragdolls, joints,
          continuous collision, sensors, and queries — run here in wasm, following the same path
          our finished
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
            <div class="stat-value">17</div>
            <div class="stat-label">Live demos</div>
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
        <p style="margin-top: 16px; color: var(--text-secondary); font-size: 0.95rem;">
          <strong>Determinism is a feature</strong> — Box3D hand-rolls its trig functions for
          cross-platform reproducibility. This port keeps them bit-for-bit.
        </p>
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

async function navigate(route: string) {
  const container = document.getElementById("main-content")!;

  if (currentCleanup) {
    currentCleanup();
    currentCleanup = null;
  }

  updateNav(route);

  if (route === "home") {
    renderHome(container);
    return;
  }

  const loader = demoModules[route];
  if (!loader) {
    container.innerHTML = `<div class="home-page"><h2>Page not found</h2><p>Unknown route: ${route}</p></div>`;
    return;
  }

  container.innerHTML = `<div class="home-page" style="display:flex;align-items:center;justify-content:center;height:80vh;"><p style="color:var(--text-muted)">Loading demo...</p></div>`;

  try {
    await loadWasm();
    const mod = await loader();
    container.innerHTML = "";
    const cleanup = mod.init(container);
    if (cleanup) currentCleanup = cleanup;
  } catch (e) {
    console.error("Failed to load demo:", e);
    container.innerHTML = `<div class="home-page"><h2>Error loading demo</h2><pre style="color:var(--clip-stroke)">${e}</pre></div>`;
  }
}

window.addEventListener("hashchange", () => navigate(getRoute()));
navigate(getRoute());
