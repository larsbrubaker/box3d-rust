// Demo Roadmap — Erin’s Samples App categories. Only real RegisterSample ports get LIVE routes.
// Do not invent demos; placeholders stay PLANNED until the C sample is ported.

const CATEGORIES: Array<{ name: string; blurb: string; route?: string }> = [
  { name: "Geometry", blurb: "Rays, GJK distance, shape queries", route: "geometry" },
  { name: "Manifold", blurb: "Contact points and normals", route: "manifolds" },
  { name: "Mesh", blurb: "Triangle meshes / height field (partial)", route: "terrain" },
  { name: "Tree", blurb: "Dynamic AABB tree broad-phase", route: "tree" },
  { name: "Collision", blurb: "Cast World ray/shape casts", route: "queries" },
  { name: "Compound", blurb: "Simple, Spheres, Hulls, Village (building.obj)", route: "compound" },
  { name: "Bodies", blurb: "Body Type gallery (more samples TBD)", route: "bodies" },
  { name: "Shapes", blurb: "Inclined Plane, Restitution, Wind, … — not yet ported" },
  { name: "Stacking", blurb: "Jenga, Box Stack, Pyramid2D (planar), Sphere Stack", route: "stacking" },
  { name: "Joints", blurb: "Revolute, Gear Lift, Driving", route: "joints" },
  { name: "Continuous", blurb: "Thin Wall, Bounce House, Bullet vs Stack", route: "continuous" },
  { name: "Events", blurb: "Sensor Visit, Sensor Hits; Benchmark Sensor", route: "sensors" },
  { name: "Character", blurb: "BasicMover + Village walk", route: "character" },
  { name: "World", blurb: "Far Pyramid at 10 000 km — large-world float stress", route: "far-pyramid" },
  { name: "Determinism", blurb: "sample_determinism readout — TBD" },
  { name: "Robustness", blurb: "HighMassRatio, Tiny Pyramid, … — not yet ported" },
  { name: "Benchmark", blurb: "Large Pyramid, Junkyard, Falling Trees", route: "benchmark" },
  { name: "Ragdoll", blurb: "Articulated bodies (partial)", route: "ragdolls" },
  { name: "Issues", blurb: "Regression / bug-repro scenes — not yet ported" },
];

export function init(container: HTMLElement) {
  const live = CATEGORIES.filter((c) => c.route).length;
  const cards = CATEGORIES.map((cat) => {
    if (cat.route) {
      return `
        <a href="#/${cat.route}" class="feature-card">
          <h3>${cat.name} <span class="badge-live">LIVE</span></h3>
          <p>${cat.blurb}</p>
        </a>`;
    }
    return `
      <div class="feature-card" style="opacity:0.65;cursor:default;">
        <h3>${cat.name} <span class="badge-planned">PLANNED</span></h3>
        <p>${cat.blurb}</p>
      </div>`;
  }).join("");

  container.innerHTML = `
    <div class="home-page">
      <div class="hero">
        <h1>Demo <span>Roadmap</span></h1>
        <p>
          Goal: port Erin’s Box3D <code>samples</code> app <strong>1:1</strong> (no invented demos).
          <strong>${live} categories</strong> have at least one live browser scene; PLANNED means
          the C <code>RegisterSample</code> gallery is not ported yet. See <code>task-12.md</code>.
        </p>
      </div>
      <div class="feature-grid">${cards}</div>
    </div>
  `;
}
