// Single source of truth for the 16-bit debug-draw / view-flag mask shared by
// the View menu (main.ts), the side-panel Debug-draw checkboxes (interaction.ts),
// the event-bus defaults (bus.ts), and the wasm `sim_set_debug_flags` contract.
//
// Pure leaf: this module imports nothing, so every consumer can import it without
// forming an import cycle. The bit order is authoritative and mirrors the Rust
// `interact::MENU_*` constants (demo/wasm/src/interact.rs) and the C
// `ApplyGuiFlags` option set. `shapes` + `transparent` drive the solid-mesh /
// style path; every other bit drives the client-side debug overlay.

/** One entry per view-flag bit: menu label, mask bit, boot default, and the
 *  menu section it renders in. */
export interface ViewFlagDef {
  /** Stable key used across the bus, the menu, and the panel. */
  key: string;
  /** Human label shown in the View menu. */
  label: string;
  /** Bit position in the 16-bit mask. */
  bit: number;
  /** Whether the flag starts on (C `b3DefaultDebugDraw`: only `shapes`). */
  default: boolean;
  /** Which View-menu group this flag renders in. */
  section: "main" | "contact" | "anchor";
}

export const VIEW_FLAGS: ViewFlagDef[] = [
  { key: "shapes", label: "Shapes", bit: 1 << 0, default: true, section: "main" },
  { key: "transparent", label: "Transparent", bit: 1 << 1, default: false, section: "main" },
  { key: "joints", label: "Joints", bit: 1 << 2, default: false, section: "main" },
  { key: "jointExtras", label: "Joint Extras", bit: 1 << 3, default: false, section: "main" },
  { key: "bounds", label: "Bounds", bit: 1 << 4, default: false, section: "main" },
  { key: "mass", label: "Mass", bit: 1 << 5, default: false, section: "main" },
  { key: "sleep", label: "Sleep", bit: 1 << 6, default: false, section: "main" },
  { key: "bodyNames", label: "Body Names", bit: 1 << 7, default: false, section: "main" },
  { key: "graphColors", label: "Graph Colors", bit: 1 << 8, default: false, section: "main" },
  { key: "islands", label: "Islands", bit: 1 << 9, default: false, section: "main" },
  { key: "contacts", label: "Contact Points", bit: 1 << 10, default: false, section: "contact" },
  { key: "contactNormals", label: "Contact Normals", bit: 1 << 11, default: false, section: "contact" },
  { key: "contactFeatures", label: "Contact Features", bit: 1 << 12, default: false, section: "contact" },
  { key: "contactForces", label: "Contact Forces", bit: 1 << 13, default: false, section: "contact" },
  { key: "frictionForces", label: "Friction Forces", bit: 1 << 14, default: false, section: "contact" },
  // drawAnchorA: 1 = A, 0 = B (default B). Rendered as an Anchor A / B radio pair.
  { key: "anchorA", label: "Anchor A", bit: 1 << 15, default: false, section: "anchor" },
];

/** key → bit lookup for the mask (consumed by interaction.ts). */
export const VIEW_BITS: Record<string, number> = Object.fromEntries(
  VIEW_FLAGS.map((f) => [f.key, f.bit]),
);

/** key → boot default (consumed by bus.ts to seed the live view-flag state). */
export const VIEW_FLAG_DEFAULTS: Record<string, boolean> = Object.fromEntries(
  VIEW_FLAGS.map((f) => [f.key, f.default]),
);

/** [key, label] pairs for the main View-menu block (Shapes … Islands). */
export const MAIN_FLAG_LABELS: [key: string, label: string][] = VIEW_FLAGS.filter(
  (f) => f.section === "main",
).map((f) => [f.key, f.label]);

/** [key, label] pairs for the contact View-menu block. */
export const CONTACT_FLAG_LABELS: [key: string, label: string][] = VIEW_FLAGS.filter(
  (f) => f.section === "contact",
).map((f) => [f.key, f.label]);

/** Bits that draw solid geometry rather than overlay lines/points/text. The
 *  overlay + text channels stay idle while only these are set. */
export const SHAPES_BIT = VIEW_BITS.shapes!;
export const TRANSPARENT_BIT = VIEW_BITS.transparent!;

/** Any overlay-relevant bit (everything except the solid-mesh bits). Used to
 *  gate the `sim_debug_draw` overlay fetch. */
export const OVERLAY_MASK = 0xffff & ~(SHAPES_BIT | TRANSPARENT_BIT);

/** Bits whose debug draw emits billboarded text labels (body names / mass /
 *  sleep timers / contact feature ids). Gates the `sim_debug_text` fetch. */
export const TEXT_MASK =
  VIEW_BITS.mass! | VIEW_BITS.sleep! | VIEW_BITS.bodyNames! | VIEW_BITS.contactFeatures!;
