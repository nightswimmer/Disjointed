/**
 * Automation hook for the manual generator (scripts/manual) - never part of normal use.
 *
 * main.ts loads this module lazily, and only in Vite's dev mode or when the page URL
 * carries `?automation`, then calls `installAutomation` with a bag of the functions the
 * UI itself uses (load a document, set the mode / tool, fit the view, ...). Nothing here
 * changes how the app behaves; it only makes the same operations callable from a
 * Playwright script through `window.__disjointed`, and adds two things the UI has no
 * need for: coordinate helpers (world -> page pixels, so scripted clicks land on real
 * features) and a vector capture of the canvas (`svg()`), which replays one frame of the
 * live renderer into an SVG recorder (src/svgcontext.ts).
 */
import { Scene, SceneData } from "./model";
import { RenderInput, render, DARK_THEME, LIGHT_THEME, Theme } from "./renderer";
import { View, worldToScreen, zoomAt } from "./view";
import { Vec2 } from "./geometry";
import { SvgRecorder } from "./svgcontext";

export interface AutomationHost {
  scene: Scene;
  view: View;
  canvas: HTMLCanvasElement;
  loadDocument(data: SceneData): void;
  setMode(mode: "draw" | "sim"): void;
  setTool(tool: string): void;
  disarmTool(): void;
  fitView(): void;
  setTheme(theme: "dark" | "light"): void;
  getTheme(): "dark" | "light";
  setGridVisible(on: boolean): void;
  setSnap(on: boolean): void;
  setObjSnap(on: boolean): void;
  setGridStep(step: number): void;
  setSelection(sel: { kind: string; id: number } | null): void;
  setMulti(bodies: number[], joints: number[]): void;
  enterComponent(defId: number, via: number | null, withGhost: boolean): void;
  exitComponent(levels: number): void;
  setCompPanelVisible(on: boolean): void;
  setAnimating(on: boolean): void;
  setCursor(p: Vec2 | null): void;
  /** The exact input the next frame would render with. */
  renderInput(): RenderInput;
  /** A summary of the interaction state, for assertions in scripts. */
  state(): Record<string, unknown>;
}

/** Theme colours -> CSS variable names used by the manual's stylesheet. */
const THEME_VAR_NAMES: Record<keyof Theme, string> = {
  ink: "dj-ink",
  surface: "dj-surface",
  grid: "dj-grid",
  jointFill: "dj-joint",
};

export interface SvgOptions {
  /** Canvas-relative crop in CSS pixels. */
  crop?: { x: number; y: number; w: number; h: number };
  /** Emit theme colours as CSS variables (default true). */
  themeVars?: boolean;
}

export function installAutomation(host: AutomationHost): void {
  const canvasRect = () => {
    const r = host.canvas.getBoundingClientRect();
    return { x: r.left, y: r.top, w: r.width, h: r.height };
  };
  /** World point -> page (viewport) CSS pixels, where Playwright's mouse works. */
  const screenOf = (p: Vec2) => {
    const s = worldToScreen(host.view, p);
    const r = canvasRect();
    return { x: s.x + r.x, y: s.y + r.y };
  };
  /** Canvas-relative CSS px -> page CSS px. */
  const pageOf = (p: Vec2) => {
    const r = canvasRect();
    return { x: p.x + r.x, y: p.y + r.y };
  };

  const api = {
    host,
    scene: host.scene,
    view: host.view,
    canvasRect,
    screenOf,
    pageOf,
    /** Screen (page) position of a joint by id. */
    jointScreen(id: number) {
      const j = host.scene.getJoint(id);
      return j ? screenOf(host.scene.jointWorld(j)) : null;
    },
    /** Screen (page) position of a body's world-vertex centroid. */
    bodyScreen(id: number) {
      const b = host.scene.getBody(id);
      if (!b) return null;
      const vs = host.scene.bodyWorldVerts(b);
      const c = vs.reduce((a, v) => ({ x: a.x + v.x / vs.length, y: a.y + v.y / vs.length }), { x: 0, y: 0 });
      return screenOf(c);
    },
    /** Canvas-relative CSS-px bounding box of the whole mechanism (bodies + joints). */
    contentBox(margin = 0) {
      let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      const include = (wp: Vec2) => {
        const s = worldToScreen(host.view, wp);
        minX = Math.min(minX, s.x); minY = Math.min(minY, s.y);
        maxX = Math.max(maxX, s.x); maxY = Math.max(maxY, s.y);
      };
      for (const b of host.scene.bodies) host.scene.bodyWorldVerts(b).forEach(include);
      for (const j of host.scene.joints) include(host.scene.jointWorld(j));
      for (const c of host.scene.constraints) if (c.kind === "ground") include(c.anchor);
      if (!Number.isFinite(minX)) return null;
      return { x: minX - margin, y: minY - margin, w: maxX - minX + 2 * margin, h: maxY - minY + 2 * margin };
    },
    load(data: SceneData) {
      host.loadDocument(data);
    },
    setView(v: Partial<View>) {
      Object.assign(host.view, v);
    },
    /** Zoom by `factor` about the canvas centre (room around a mechanism for a gesture). */
    zoom(factor: number) {
      zoomAt(host.view, { x: host.canvas.clientWidth / 2, y: host.canvas.clientHeight / 2 }, factor);
    },
    /** Resolve after the app has rendered at least one more frame. */
    frame(): Promise<void> {
      return new Promise((res) => requestAnimationFrame(() => requestAnimationFrame(() => res())));
    },
    /** Replay one frame of the live renderer into an SVG document. */
    svg(opts: SvgOptions = {}): string {
      const dpr = window.devicePixelRatio || 1;
      const theme = host.getTheme() === "light" ? LIGHT_THEME : DARK_THEME;
      const themeVars: Record<string, string> = {};
      if (opts.themeVars !== false) {
        for (const k of Object.keys(THEME_VAR_NAMES) as (keyof Theme)[]) themeVars[theme[k]] = THEME_VAR_NAMES[k];
      }
      const rec = new SvgRecorder({
        width: host.canvas.clientWidth,
        height: host.canvas.clientHeight,
        dpr,
        themeVars,
      });
      render(rec as unknown as CanvasRenderingContext2D, host.renderInput());
      return rec.toSvg(opts.crop);
    },
  };
  (window as unknown as { __disjointed: typeof api }).__disjointed = api;
}

export type AutomationApi = ReturnType<typeof installAutomation>;
