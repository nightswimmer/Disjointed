/**
 * Manual illustration generator.
 *
 *   npm run manual                 -> regenerates every illustration into public/help/img
 *   npm run manual -- --only slider-crank,legend
 *   npm run manual -- --check-dir <dir>   also rasterises every SVG (both themes) into
 *                                          <dir> as PNGs, for eyeballing the vector output
 *   npm run manual -- --headed     watch it happen
 *
 * Starts a Vite dev server on a spare port, drives the real app in the installed Chrome
 * through Playwright with the automation hook (src/automation.ts, `?automation`), and:
 *   - "svg" shots replay the live renderer into an SVG (vector, theme-following via CSS
 *     variables, cropped to the mechanism);
 *   - "png" shots screenshot a DOM element (panels, dialogs) at 2x in both themes;
 *   - toolbar button glyphs and group membership are read from the live DOM into
 *     public/help/glyphs.js, so the manual draws toolbar buttons as vectors.
 * It then checks that every help topic the app can ask for exists in public/help/index.html.
 */
import { chromium, Browser, Page } from "playwright";
import { createServer } from "vite";
import { mkdirSync, writeFileSync, readFileSync, existsSync } from "node:fs";
import { resolve, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { SHOTS, Shot, ShotCtx, Theme } from "./shots";
import { FIXTURES, Fixture } from "./fixtures";
import { allTopics, ID_TOPICS } from "../../src/helpmap";

const here = dirname(fileURLToPath(import.meta.url));
const ROOT = resolve(here, "../..");
const OUT = resolve(ROOT, "public/help/img");
const PORT = 5199;
// Wide enough for the whole toolbar: an element screenshot scrolls its target into view,
// and a scrolled page would put the canvas (and the scripted mouse) out of the viewport.
const VIEWPORT = { width: 1920, height: 1000 };

const args = process.argv.slice(2);
const flag = (name: string) => {
  const i = args.indexOf(name);
  return i >= 0 ? args[i + 1] : undefined;
};
const only = flag("--only")?.split(",").filter(Boolean);
const checkDir = flag("--check-dir");
const headed = args.includes("--headed");

/** Everything a shot's `run` step can use. */
function makeCtx(page: Page, theme: Theme): ShotCtx {
  const call = async <T>(name: string, ...a: unknown[]): Promise<T> =>
    page.evaluate(
      ([n, args]) => {
        const api = (window as unknown as { __disjointed: Record<string, unknown> }).__disjointed;
        const host = api.host as Record<string, (...x: unknown[]) => unknown>;
        const fn = (api as Record<string, unknown>)[n as string] ?? host[n as string];
        if (typeof fn !== "function") throw new Error(`no automation function ${n}`);
        return (fn as (...x: unknown[]) => unknown).apply(fn === host[n as string] ? host : api, args as unknown[]) as T;
      },
      [name, a] as [string, unknown[]]
    );
  return {
    page,
    theme,
    call,
    frame: () => call<void>("frame"),
    fixture: (name: string): Fixture => {
      const f = FIXTURES[name];
      if (!f) throw new Error(`unknown fixture ${name}`);
      return f();
    },
    load: async (fx: Fixture) => {
      await call("load", fx.data);
      await call("fitView");
      await call("frame");
    },
    jointScreen: (id: number) => call<{ x: number; y: number } | null>("jointScreen", id),
    bodyScreen: (id: number) => call<{ x: number; y: number } | null>("bodyScreen", id),
    drag: async (from, to, opts = {}) => {
      const steps = opts.steps ?? 12;
      await page.mouse.move(from.x, from.y);
      await page.mouse.down();
      for (let i = 1; i <= steps; i++) {
        await page.mouse.move(from.x + ((to.x - from.x) * i) / steps, from.y + ((to.y - from.y) * i) / steps);
        await call("frame");
      }
      if (!opts.hold) await page.mouse.up();
      await call("frame");
    },
  };
}

async function setupPage(browser: Browser, theme: Theme): Promise<Page> {
  const context = await browser.newContext({ viewport: VIEWPORT, deviceScaleFactor: 2 });
  const page = await context.newPage();
  page.on("pageerror", (e) => console.error("  [page error]", e.message));
  await page.goto(`http://localhost:${PORT}/?automation`);
  await page.waitForFunction(() => "__disjointed" in window);
  await page.evaluate((t) => (window as unknown as { __disjointed: { host: { setTheme(t: string): void } } }).__disjointed.host.setTheme(t), theme);
  return page;
}

async function prepare(ctx: ShotCtx, shot: Shot): Promise<void> {
  const { call } = ctx;
  await ctx.page.evaluate(() => window.scrollTo(0, 0));
  await call("setMode", "draw");
  await call("disarmTool");
  await call("setSelection", null);
  await call("setGridVisible", shot.grid ?? true);
  await call("setCompPanelVisible", false);
  if (shot.fixture) await ctx.load(ctx.fixture(shot.fixture));
  else {
    await call("load", { version: 1, bodies: [], joints: [], constraints: [] });
    await call("fitView");
  }
  if (shot.mode) await call("setMode", shot.mode);
  if (shot.tool) await call("setTool", shot.tool);
  await call("frame");
}

async function captureSvg(ctx: ShotCtx, shot: Shot): Promise<string> {
  const margin = shot.margin ?? 40;
  const box = await ctx.call<{ x: number; y: number; w: number; h: number } | null>("contentBox", margin);
  const rect = await ctx.call<{ x: number; y: number; w: number; h: number }>("canvasRect");
  let crop = box ?? { x: 0, y: 0, w: rect.w, h: rect.h };
  // Keep the crop inside the canvas.
  const x0 = Math.max(0, crop.x), y0 = Math.max(0, crop.y);
  const x1 = Math.min(rect.w, crop.x + crop.w), y1 = Math.min(rect.h, crop.y + crop.h);
  crop = { x: Math.round(x0), y: Math.round(y0), w: Math.round(x1 - x0), h: Math.round(y1 - y0) };
  if (shot.crop) crop = shot.crop(crop, rect);
  return ctx.call<string>("svg", { crop });
}

/** Rasterise an SVG file on a page with the manual's theme variables, for inspection. */
async function rasterise(browser: Browser, svgPath: string, outPath: string, theme: Theme): Promise<void> {
  const css = readFileSync(resolve(ROOT, "public/help/help.css"), "utf8");
  const svg = readFileSync(svgPath, "utf8");
  const context = await browser.newContext({ viewport: VIEWPORT, deviceScaleFactor: 2 });
  const page = await context.newPage();
  await page.setContent(
    `<!doctype html><html data-theme="${theme}"><head><style>${css} body{padding:12px} figure{display:inline-block;margin:0}</style></head>` +
      `<body><figure>${svg}</figure></body></html>`
  );
  await page.locator("figure").screenshot({ path: outPath });
  await context.close();
}

async function main(): Promise<void> {
  mkdirSync(OUT, { recursive: true });
  if (checkDir) mkdirSync(checkDir, { recursive: true });

  const server = await createServer({
    root: ROOT,
    configFile: resolve(ROOT, "vite.config.ts"),
    server: { port: PORT, strictPort: true, open: false },
    logLevel: "warn",
  });
  await server.listen();
  const browser = await chromium.launch({ channel: "chrome", headless: !headed });
  let failures = 0;
  // Toolbar glyphs for the manual: every toolbar button's inline SVG (keyed by its topic
  // tool-x / mode-x, else its element id) and the buttons of each toolbar group, read from
  // the live DOM so they are exactly what the app shows.
  let toolbar: { glyphs: Record<string, string[]>; groups: Record<string, string[]> } | null = null;
  try {
    const pages: Record<Theme, Page> = { dark: await setupPage(browser, "dark"), light: await setupPage(browser, "light") };
    const shots = SHOTS.filter((s) => !only || only.includes(s.id));
    for (const shot of shots) {
      const themes: Theme[] = shot.kind === "svg" ? ["dark"] : ["dark", "light"];
      for (const theme of themes) {
        const page = pages[theme];
        const ctx = makeCtx(page, theme);
        const label = shot.kind === "svg" ? shot.id : `${shot.id}-${theme}`;
        try {
          await prepare(ctx, shot);
          if (shot.run) await shot.run(ctx);
          if (shot.kind === "svg") {
            const svg = await captureSvg(ctx, shot);
            const file = resolve(OUT, `${shot.id}.svg`);
            writeFileSync(file, svg);
            console.log(`  svg  ${shot.id}.svg  (${(svg.length / 1024).toFixed(0)} KB)`);
            if (checkDir) {
              for (const t of ["dark", "light"] as Theme[]) {
                await rasterise(browser, file, resolve(checkDir, `${shot.id}-${t}.png`), t);
              }
            }
          } else {
            const file = resolve(OUT, `${label}.png`);
            const target = page.locator(shot.selector!).first();
            await target.screenshot({ path: file, animations: "disabled" });
            console.log(`  png  ${label}.png`);
          }
          if (shot.after) await shot.after(ctx);
        } catch (err) {
          failures++;
          console.error(`  FAIL ${label}: ${(err as Error).message}`);
          await page.mouse.up().catch(() => undefined);
        }
      }
    }
    // Passed as source text, not a closure: tsx would inject a `__name` helper into a
    // compiled closure, which does not exist inside the page.
    toolbar = await pages.dark.evaluate(`(() => {
      const glyphs = {}, groups = {};
      document.querySelectorAll("#toolbar button").forEach((b) => {
        const key = b.dataset.tool ? "tool-" + b.dataset.tool : b.dataset.mode ? "mode-" + b.dataset.mode : b.id || null;
        const svgs = [...b.querySelectorAll(":scope > svg")].map((s) => s.outerHTML);
        if (!key || !svgs.length) return;
        glyphs[key] = svgs;
        const group = b.closest(".group[id]");
        if (group) (groups[group.id] = groups[group.id] || []).push(key);
      });
      return { glyphs, groups };
    })()`);
  } finally {
    await browser.close();
    await server.close();
  }

  if (toolbar) {
    writeFileSync(
      resolve(ROOT, "public/help/glyphs.js"),
      `// Generated by scripts/manual/shoot.ts from the toolbar in index.html - do not edit.
` +
        `window.DISJOINTED_GLYPHS = ${JSON.stringify(toolbar.glyphs, null, 1)};
` +
        `window.DISJOINTED_GROUPS = ${JSON.stringify(toolbar.groups, null, 1)};
`
    );
    console.log(`  glyphs.js  (${Object.keys(toolbar.glyphs).length} buttons, ${Object.keys(toolbar.groups).length} groups)`);
  }
  // ...and which element ids lead to each topic (the inverse of helpmap's ID_TOPICS), so a
  // topic heading can show every button that opens it.
  const topicIds: Record<string, string[]> = {};
  for (const [id, topic] of Object.entries(ID_TOPICS)) (topicIds[topic] ??= []).push(id);
  writeFileSync(
    resolve(ROOT, "public/help/topics.js"),
    `// Generated by scripts/manual/shoot.ts from src/helpmap.ts - do not edit.\n` +
      `window.DISJOINTED_TOPIC_IDS = ${JSON.stringify(topicIds, null, 1)};\n`
  );

  // Every topic the app can ask for must exist in the manual.
  const html = readFileSync(resolve(ROOT, "public/help/index.html"), "utf8");
  const indexHtml = readFileSync(resolve(ROOT, "index.html"), "utf8");
  const tools = [...indexHtml.matchAll(/data-tool="([^"]+)"/g)].map((m) => m[1]);
  const modes = [...indexHtml.matchAll(/data-mode="([^"]+)"/g)].map((m) => m[1]);
  const missing = allTopics(tools, modes).filter((t) => !new RegExp(`id="${t}"`).test(html));
  if (missing.length) {
    failures++;
    console.error(`  Missing manual topics: ${missing.join(", ")}`);
  }
  // Every illustration the manual references must exist.
  const refs = [...html.matchAll(/(?:data-svg|data-dark|data-light|src)="(img\/[^"]+)"/g)].map((m) => m[1]);
  const missingImgs = [...new Set(refs)].filter((r) => !existsSync(resolve(ROOT, "public/help", r)));
  if (missingImgs.length && !only) {
    failures++;
    console.error(`  Missing illustrations: ${missingImgs.join(", ")}`);
  }
  console.log(failures ? `\n${failures} problem(s)` : "\nAll illustrations generated, all topics present.");
  process.exit(failures ? 1 : 0);
}

void main();
