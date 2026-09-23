/**
 * The manual's illustrations, one entry each. Regenerate with `npm run manual`.
 *
 * "svg" shots capture the canvas as a vector drawing cropped to the drawing (theme
 * colours as CSS variables, so one file serves both themes). "html" shots copy a DOM
 * element's markup (`selector`) into public/help/ui.js, which the manual renders with the
 * app's own stylesheet (public/ui.css) - the way to show any piece of the UI, since it
 * then follows the theme and every style change by itself. "png" shots screenshot an
 * element in both themes: the fallback for something the stylesheet alone can't render.
 * `run` performs real mouse gestures through Playwright after the fixture is loaded and
 * the mode / tool set; `after` cleans up.
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
  kind: "svg" | "html" | "png";
  /** html / png: the element to capture. */
  selector?: string;
  /** Fixture to load first (see fixtures.ts); none = empty scene. */
  fixture?: string;
  mode?: "draw" | "sim";
  tool?: string;
  grid?: boolean;
  /**
   * svg: zoom factor applied after fitting the bodies to the canvas. Below 1 makes room
   * for annotations around them and keeps badges legible at the manual's width; above 1
   * enlarges a drawing with no bodies (fit view leaves those at the default scale).
   */
  zoom?: number;
  /** svg: margin (CSS px) around the drawing's bounding box. */
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

/** Page position of a world point (the fixtures' coordinates). */
const world = (ctx: ShotCtx, x: number, y: number) => ctx.call<{ x: number; y: number }>("screenOf", { x, y });

/** Select one body (its handles, tags and rims show). */
const selectBody = async (ctx: ShotCtx, id: number) => {
  await ctx.call("setSelection", { kind: "body", id });
  await ctx.frame();
};

/** Walk the held mouse from where it is to `to`, rendering as it goes. */
async function glide(ctx: ShotCtx, from: { x: number; y: number }, to: { x: number; y: number }, steps = 12): Promise<void> {
  for (let i = 1; i <= steps; i++) {
    await ctx.page.mouse.move(from.x + ((to.x - from.x) * i) / steps, from.y + ((to.y - from.y) * i) / steps);
    await ctx.frame();
  }
}

export const SHOTS: Shot[] = [
  // --- UI (html snapshots): toolbar sections with non-button controls, panels, overlays. ---
  {
    id: "grid-group",
    kind: "html",
    selector: "#sec-grid",
  },
  {
    id: "keymap-panel",
    kind: "html",
    selector: "#keymap-panel",
    run: async (ctx) => {
      await ctx.page.click("#keymap-btn");
      await ctx.page.waitForSelector("#keymap-panel:not(.hidden)");
    },
    after: async (ctx) => {
      await ctx.page.click("#keymap-close");
    },
  },
  {
    id: "crumb-group",
    kind: "html",
    selector: "#crumb-bar",
    fixture: "group",
    run: async (ctx) => {
      const fx = ctx.fixture("group");
      const c = pt(await ctx.bodyScreen(fx.bodies.barA), "group member");
      await ctx.page.mouse.dblclick(c.x, c.y);
      await ctx.page.waitForSelector("#crumb-bar:not(.hidden)");
      await ctx.frame();
    },
    after: async (ctx) => {
      await ctx.page.keyboard.press("Escape");
      await ctx.frame();
    },
  },

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
    zoom: 0.5,
    fixture: "plate",
    mode: "draw",
    run: async (ctx) => selectBody(ctx, ctx.fixture("plate").bodies.plate),
  },

  // --- shapes ----------------------------------------------------------------------------
  {
    id: "polygon-sides",
    kind: "svg",
    zoom: 0.45,
    fixture: "polygon",
    mode: "draw",
    margin: 50,
    run: async (ctx) => selectBody(ctx, ctx.fixture("polygon").bodies.hex),
  },

  // --- Boolean operations ----------------------------------------------------------------
  {
    id: "boolean-before",
    kind: "svg",
    zoom: 0.45,
    fixture: "booleanBefore",
    mode: "draw",
    run: async (ctx) => {
      const fx = ctx.fixture("booleanBefore");
      await ctx.call("setMulti", [fx.bodies.plate, fx.bodies.inner, fx.bodies.outer], []);
      await ctx.frame();
    },
  },
  {
    id: "boolean-subtract",
    kind: "svg",
    zoom: 0.45,
    fixture: "booleanSubtract",
    mode: "draw",
  },
  {
    id: "boolean-intersect",
    kind: "svg",
    zoom: 0.45,
    fixture: "booleanIntersect",
    mode: "draw",
  },

  // --- patterns --------------------------------------------------------------------------
  {
    id: "pattern-linear",
    kind: "svg",
    zoom: 0.42,
    fixture: "patternLinear",
    mode: "draw",
    margin: 50,
  },
  {
    id: "pattern-circular",
    kind: "svg",
    zoom: 0.5,
    fixture: "patternCircular",
    mode: "draw",
    margin: 50,
  },

  // --- constraints -----------------------------------------------------------------------
  {
    id: "constraint-fixed",
    kind: "svg",
    zoom: 0.4,
    fixture: "fixed",
    mode: "draw",
    margin: 44,
  },
  {
    id: "constraint-symmetric",
    kind: "svg",
    zoom: 0.42,
    fixture: "symmetric",
    mode: "draw",
    margin: 44,
  },
  {
    id: "constraint-tangent",
    kind: "svg",
    zoom: 0.42,
    fixture: "tangent",
    mode: "draw",
    margin: 44,
  },
  {
    id: "tangent-blend",
    kind: "svg",
    zoom: 3,
    fixture: "blend",
    mode: "draw",
    margin: 44,
  },
  {
    id: "implicit-drag",
    kind: "svg",
    zoom: 0.45,
    fixture: "implicit",
    mode: "draw",
    margin: 50,
    run: async (ctx) => {
      // Drag the free joint onto the plate's top-right corner, hold it there until the
      // corner arms as an alignment candidate, then move straight up from it: the V
      // alignment previews. Captured mid-drag, button still down.
      await ctx.call("setSnap", false);
      await ctx.call("setObjSnap", false);
      const fx = ctx.fixture("implicit");
      const start = pt(await ctx.jointScreen(fx.joints.free), "free joint");
      const corner = await world(ctx, 160, 0);
      await ctx.page.mouse.move(start.x, start.y);
      await ctx.page.mouse.down();
      await glide(ctx, start, corner, 14);
      await ctx.page.waitForTimeout(700); // ALIGN_HOVER_MS is 400
      await ctx.frame();
      await glide(ctx, corner, { x: corner.x, y: corner.y - 90 }, 10);
      await ctx.frame();
    },
    after: async (ctx) => {
      await ctx.page.mouse.up();
      await ctx.frame();
    },
  },

  // --- dimensions ------------------------------------------------------------------------
  {
    id: "dimension-directions",
    kind: "svg",
    zoom: 0.4,
    fixture: "dimensions",
    mode: "draw",
    margin: 50,
  },
  {
    id: "dimension-sizes",
    kind: "svg",
    zoom: 0.45,
    fixture: "sizes",
    mode: "draw",
    margin: 50,
  },

  // --- groups ----------------------------------------------------------------------------
  {
    id: "group-edit",
    kind: "svg",
    zoom: 0.5,
    fixture: "group",
    mode: "draw",
    margin: 50,
    run: async (ctx) => {
      const fx = ctx.fixture("group");
      const c = pt(await ctx.bodyScreen(fx.bodies.barA), "group member");
      await ctx.page.mouse.dblclick(c.x, c.y);
      await ctx.frame();
      // Select one member so the picture shows parts being edited one at a time.
      await selectBody(ctx, fx.bodies.barB);
    },
    after: async (ctx) => {
      await ctx.page.keyboard.press("Escape");
      await ctx.page.keyboard.press("Escape");
      await ctx.frame();
    },
  },
];
