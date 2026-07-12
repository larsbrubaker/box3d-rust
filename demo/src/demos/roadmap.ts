// Demo Roadmap — upstream sample categories with LIVE / PLANNED badges.

const CATEGORIES: Array<{ name: string; blurb: string; route?: string }> = [
  { name: "Geometry", blurb: "Rays, GJK distance, shape queries", route: "geometry" },
  { name: "Manifold", blurb: "Contact points and normals", route: "manifolds" },
  { name: "Mesh", blurb: "Triangle meshes, casts, AABB, terrain settle", route: "terrain" },
  { name: "Tree", blurb: "Dynamic AABB tree broad-phase", route: "tree" },
  { name: "Collision", blurb: "Hulls, height fields, casting", route: "hull" },
  { name: "Compound", blurb: "Simple, Spheres, Hulls, Village", route: "compound" },
  { name: "Bodies", blurb: "Body types, sleeping, user data", route: "bodies" },
  { name: "Shapes", blurb: "Spheres, capsules, hulls", route: "bodies" },
  { name: "Stacking", blurb: "Single Box, Box Stack, Pyramid2D, Sphere Stack", route: "stacking" },
  { name: "Joints", blurb: "Revolute, spherical, motor hinge", route: "joints" },
  { name: "Continuous", blurb: "Fast bodies without tunneling", route: "continuous" },
  { name: "Events", blurb: "Contacts, sensors, hit events", route: "sensors" },
  { name: "Character", blurb: "BasicMover + Village walk", route: "character" },
  { name: "World", blurb: "Queries, gravity, large worlds", route: "queries" },
  { name: "Determinism", blurb: "Cross-platform reproducibility", route: "math" },
  { name: "Robustness", blurb: "Degenerate input, overlap recovery" },
  { name: "Benchmark", blurb: "Large Pyramid, Junkyard, Falling Trees", route: "benchmark" },
  { name: "Ragdoll", blurb: "Articulated bodies", route: "ragdolls" },
  { name: "Issues", blurb: "Regression / bug-repro scenes" },
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
          Each category of the upstream Box3D <code>samples</code> app becomes an interactive
          browser demo as its module lands. <strong>${live} sample categories are LIVE</strong>
          — including ragdolls, joints, continuous collision, sensors, queries, terrain settle,
          and a character mover.
        </p>
      </div>
      <div class="feature-grid">${cards}</div>
    </div>
  `;
}
