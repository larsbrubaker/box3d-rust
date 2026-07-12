// Demo Roadmap — Erin's Samples App categories. Only real RegisterSample ports get routes.
// Do not invent demos; placeholders stay PLANNED until the C sample is ported. Counts come
// from the verified inventory in task-12.md (full audit, 2026-07-12).

type Status = "partial" | "planned";

const CATEGORIES: Array<{
  name: string;
  blurb: string;
  status: Status;
  count: string;
  route?: string;
}> = [
  // Collision layer
  { name: "Geometry", blurb: "Rays, GJK distance, Box Hull. Box Hull is a partial port (reachable from the nav); the full gallery is not ported.", status: "planned", count: "0/5 (Box Hull partial)" },
  { name: "Manifold", blurb: "Contact points and normals — wrong radii/extents/cameras vs C; triangle manifolds absent.", status: "partial", count: "4/9 partial", route: "manifolds" },
  { name: "Mesh", blurb: "Triangle mesh + height field as static raycast viewers, not the C dynamics scenes.", status: "partial", count: "3/9 partial", route: "mesh" },
  { name: "Tree", blurb: "Dynamic AABB tree — C sample is a bounds-file benchmark (1024 queries, profiling); not ported.", status: "planned", count: "0/1" },
  { name: "Collision", blurb: "Cast World ray/shape casts — exact match to sample_collision.cpp.", status: "partial", count: "1/12 (Cast World exact)", route: "queries" },
  // Dynamics
  { name: "Compound", blurb: "Simple, Spheres, Hulls, Village (building.obj) — partial (Village has extra bodies, no mover viz).", status: "partial", count: "4/6 partial", route: "compound" },
  { name: "Bodies", blurb: "Body Type gallery and 8 more — not yet ported (invented drop scene removed).", status: "planned", count: "0/9" },
  { name: "Shapes", blurb: "Inclined Plane, Restitution, Wind, … — not yet ported.", status: "planned", count: "0/12" },
  { name: "Stacking", blurb: "Jenga, Box Stack, Pyramid2D (planar), Sphere Stack — friction/density/count differ from C.", status: "partial", count: "5/14 partial", route: "stacking" },
  { name: "Joints", blurb: "Gear Lift (exact), Revolute (near-exact), Ball and Chain + Driving (partial).", status: "partial", count: "2/16 (+2 partial)", route: "joints" },
  { name: "Continuous", blurb: "Thin Wall, Bounce House, Bullet vs Stack — exact.", status: "partial", count: "3/10 exact", route: "continuous" },
  { name: "Events", blurb: "Sensor Visit + Sensor Hits (both exact). 4 more Events samples not ported.", status: "partial", count: "2/6 exact", route: "sensors" },
  { name: "Character", blurb: "BasicMover + Village walk — filler boxes instead of the C test maps.", status: "partial", count: "1/4 partial", route: "character" },
  { name: "World", blurb: "Far Pyramid at 10 000 km — large-world float stress (exact). Far Stack/Ragdolls/Mesh missing.", status: "partial", count: "1/4 (Far Pyramid exact)", route: "far-pyramid" },
  { name: "Determinism", blurb: "Falling Ragdolls hash soak — not yet ported.", status: "planned", count: "0/1" },
  { name: "Robustness", blurb: "HighMassRatio, Tiny Pyramid, … — not yet ported.", status: "planned", count: "0/4" },
  { name: "Benchmark", blurb: "Large Pyramid, Junkyard, Falling Trees, Benchmark Sensor — scaled with camera/count divergences.", status: "partial", count: "4/17 partial", route: "benchmark" },
  { name: "Ragdoll", blurb: "Articulated capsule-bone humans — Box contaminated by an invented multi-human slider.", status: "partial", count: "1/4 partial", route: "ragdolls" },
  { name: "Issues", blurb: "Regression / bug-repro scenes — not yet ported.", status: "planned", count: "0/7" },
];

export function init(container: HTMLElement) {
  const partial = CATEGORIES.filter((c) => c.status === "partial").length;
  const cards = CATEGORIES.map((cat) => {
    const badge =
      cat.status === "partial"
        ? `<span class="badge-planned" style="color:#b45309;border-color:#b45309;">PARTIAL</span>`
        : `<span class="badge-planned">PLANNED</span>`;
    const meta = `<p style="font-size:0.8rem;color:var(--text-muted);margin-top:6px;">${cat.count}</p>`;
    if (cat.route) {
      return `
        <a href="#/${cat.route}" class="feature-card">
          <h3>${cat.name} ${badge}</h3>
          <p>${cat.blurb}</p>
          ${meta}
        </a>`;
    }
    return `
      <div class="feature-card" style="opacity:0.65;cursor:default;">
        <h3>${cat.name} ${badge}</h3>
        <p>${cat.blurb}</p>
        ${meta}
      </div>`;
  }).join("");

  container.innerHTML = `
    <div class="home-page">
      <div class="hero">
        <h1>Demo <span>Roadmap</span></h1>
        <p>
          Goal: port Erin's Box3D <code>samples</code> app <strong>1:1</strong> (no invented demos).
          <strong>${partial} categories</strong> have at least one browser scene, all still partial;
          PLANNED means the C <code>RegisterSample</code> gallery is not ported yet. Counts are
          ported/total from the audit in <code>task-12.md</code>.
        </p>
      </div>
      <div class="feature-grid">${cards}</div>
    </div>
  `;
}
