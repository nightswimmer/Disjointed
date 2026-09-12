/**
 * The manual's illustrations, one entry each. Regenerate with `npm run manual`.
 *
 * "svg" shots capture the canvas as a vector drawing cropped to the mechanism (theme
 * colours as CSS variables, so one file serves both themes). "png" shots screenshot a
 * DOM element (`selector`) in both themes. `run` performs real mouse gestures through
 * Playwright after the fixture is loaded and the mode / tool set; `after` cleans up.
 */
import type { Page } from "playwright";
import type { Fixture } from "./fixtures";

export type Theme = "dark" | "light";

export interface ShotCtx {
  page: Page;
  theme: Theme;
  /** Call an automation-hook function (host or helper) by name. */
  call<T = unknown>(name: string, ...args: unknown[]): Promise<T>;
  /** Wait for the app to render a frame. */
  frame(): Promise<void>;
  fixture(name: string): Fixture;
  load(fx: Fixture): Promise<void>;
  jointScreen(id: number): Promise<{ x: number; y: number } | null>;
  bodyScreen(id: number): Promise<{ x: number; y: number } | null>;
  /** A left-button drag in page pixels; `hold` leaves the button down (mid-drag capture). */
  drag(from: { x: number; y: number }, to: { x: number; y: number }, opts?: { steps?: number; hold?: boolean }): Promise<void>;
}

export interface Shot {
  id: string;
  kind: "svg" | "png";
  /** png: the element to capture. */
  selector?: string;
  /** Fixture to load first (see fixtures.ts); none = empty scene. */
  fixture?: string;
  mode?: "draw" | "sim";
  tool?: string;
  grid?: boolean;
  /** svg: margin (CSS px) around the mechanism's bounding box. */
  margin?: number;
  /** svg: adjust the automatic crop (canvas-relative CSS px). */
  crop?(auto: Rect, canvas: Rect): Rect;
  run?(ctx: ShotCtx): Promise<void>;
  after?(ctx: ShotCtx): Promise<void>;
}
export interface Rect { x: number; y: number; w: number; h: number }

const pt = (p: { x: number; y: number } | null, what: string) => {
  if (!p) throw new Error(`${what} not found`);
  return p;
};

export const SHOTS: Shot[] = [
  // --- UI (png, both themes): panels and dialogs go here; toolbar buttons are drawn from
  // their own SVG glyphs by the manual page (see glyphs.js), not screenshotted.

  // --- mechanisms (svg) ------------------------------------------------------------------
  {
    id: "fourbar-draw",
    kind: "svg",
    fixture: "fourBar",
    mode: "draw",
  },
  {
    id: "fourbar-sim-drag",
    kind: "svg",
    fixture: "fourBar",
    mode: "sim",
    margin: 60,
    run: async (ctx) => {
      // Grab the crank pin and pull it round: captured mid-drag, button still down.
      // Zoom out first so the moved linkage stays inside the canvas.
      await ctx.call("zoom", 0.6);
      await ctx.frame();
      const fx = ctx.fixture("fourBar");
      const b = pt(await ctx.jointScreen(fx.joints.B), "crank pin");
      const a = pt(await ctx.jointScreen(fx.joints.A), "crank pivot");
      // Swing the pin about 60 degrees clockwise about the pivot.
      const dx = b.x - a.x, dy = b.y - a.y;
      const c = Math.cos(Math.PI / 3), s = Math.sin(Math.PI / 3);
      const to = { x: a.x + dx * c - dy * s, y: a.y + dx * s + dy * c };
      await ctx.drag(b, to, { steps: 20, hold: true });
    },
    after: async (ctx) => {
      await ctx.page.mouse.up();
    },
  },
  {
    id: "slider-crank",
    kind: "svg",
    fixture: "sliderCrank",
    mode: "draw",
    margin: 50,
  },
  {
    id: "legend",
    kind: "svg",
    fixture: "legend",
    mode: "draw",
    margin: 36,
  },
  {
    id: "plate-selected",
    kind: "svg",
    fixture: "plate",
    mode: "draw",
    run: async (ctx) => {
      const fx = ctx.fixture("plate");
      await ctx.call("setSelection", { kind: "body", id: fx.bodies.plate });
      await ctx.frame();
    },
  },
];
