// Rigid Body Debug (V) gate — matches C `m_showDebug` default + DrawDebug guard
// (sample_character.cpp :1488 / :1609-1612).

import { test, expect } from "bun:test";
import {
  RIGID_BODY_SHOW_DEBUG_DEFAULT,
  shouldDrawCharacterDebug,
} from "../src/demos/character.ts";

test("Rigid Body showDebug defaults true like C m_showDebug", () => {
  expect(RIGID_BODY_SHOW_DEBUG_DEFAULT).toBe(true);
});

test("shouldDrawCharacterDebug gates only the Rigid Body scene", () => {
  expect(shouldDrawCharacterDebug("rigid-body", true)).toBe(true);
  expect(shouldDrawCharacterDebug("rigid-body", false)).toBe(false);

  // Other character scenes always draw their own overlays (no Debug (V) gate).
  for (const scene of ["capsule-plane", "mover-overlap", "mover"] as const) {
    expect(shouldDrawCharacterDebug(scene, false)).toBe(true);
    expect(shouldDrawCharacterDebug(scene, true)).toBe(true);
  }
});
