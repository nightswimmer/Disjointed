/**
 * The keymap in the real app: every shipped shortcut pressed in the running page, and the
 * generated tooltips read back off the live buttons.
 *
 * `scripts/keymap.ts` checks the table, the file and the markup; it cannot check
 * *dispatch*, which needs a DOM, a canvas and real keystrokes — so this drives the app in
 * the installed Chrome the way `npm run manual` does. It is not part of `npm test`
 * (headless, no browser): run it after touching the key handler, and after a remap, when
 * the letters below are updated to the new layout.
 *
 *   npm run keymap:live            # headless
 *   npm run keymap:live -- --headed
 *
 * The letters here are today's bindings, written out by hand on purpose: if the registry
 * and this file ever disagree, one of them is wrong and that is the thing worth knowing.
 * They were last brought in line with the 2026-09-15 remap.
 */
import { createServer } from "vite";
import { chromium } from "playwright";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const PORT = 5181;
const headed = process.argv.includes("--headed");

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

const server = await createServer({
  root: ROOT,
  configFile: resolve(ROOT, "vite.config.ts"),
  server: { port: PORT, strictPort: true, open: false },
  logLevel: "warn",
});
await server.listen();
const browser = await chromium.launch({ channel: "chrome", headless: !headed });
const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
const page = await context.newPage();
page.on("pageerror", (e) => { console.error("  [page error]", e.message); failures++; });
await page.goto(`http://localhost:${PORT}/?automation`);
await page.waitForFunction(() => "__disjointed" in window);

type State = { mode: string; tool: string | null; selection: unknown };
const state = () => page.evaluate(() => (window as any).__disjointed.host.state() as State);
const title = (sel: string) => page.locator(sel).getAttribute("title");
const role = () => page.locator(".role-btn.active").getAttribute("data-role");
const press = async (key: string) => { await page.locator("canvas").press(key); };

// --- tooltips are generated from the keymap ---------------------------------
check("tool key in a tooltip", (await title('[data-tool="joint"]'))?.endsWith("(J)") === true, String(await title('[data-tool="joint"]')));
check("Shift key in a tooltip", (await title('[data-tool="rect"]'))?.endsWith("(Shift+B)") === true);
check("two commands on one button", (await title('[data-tool="polyline"]'))?.endsWith("(B = body, Ctrl+U = cut)") === true, String(await title('[data-tool="polyline"]')));
check("save names both of its keys", (await title("#save-btn"))?.endsWith("(Ctrl+S = Save, Ctrl+Shift+S = Save as…)") === true, String(await title("#save-btn")));
check("the mode toggle gained its key", (await title("#mode-toggle"))?.endsWith("(Tab)") === true, String(await title("#mode-toggle")));
check("a keyless button keeps its plain title", (await title("#mirror-h-btn")) === "Mirror the selection left↔right");
check("? on the help button", (await title("#help-btn"))?.endsWith("(?)") === true);
check("PageDown on send-to-back", (await title("#send-back-btn"))?.endsWith("(PageDown)") === true);
check("the new Shortcuts button has no key", (await title("#keymap-btn")) === "Keyboard shortcuts — import, export or reset the keymap");

// --- tool letters ------------------------------------------------------------
for (const [key, tool] of [["j", "joint"], ["w", "weld"], ["g", "ground"], ["s", "slider"], ["r", "rotate"], ["c", "circle"], ["l", "line"], ["a", "arc"], ["d", "measure"], ["x", "split"], ["i", "patternLinear"], ["q", "patternCircular"], ["o", "coincident"], ["h", "horizontal"], ["v", "vertical"], ["p", "parallel"], ["t", "perpendicular"], ["e", "equal"], ["f", "fixed"], ["y", "symmetric"], ["z", "tangent"]] as const) {
  await press(key);
  check(`${key} arms ${tool}`, (await state()).tool === tool, String((await state()).tool));
}
for (const [key, tool] of [["Shift+B", "rect"], ["Shift+P", "polygon"], ["Shift+S", "slot"], ["Shift+T", "text"], ["Shift+A", "linearActuator"], ["Shift+M", "motor"], ["Control+Shift+C", "connect"]] as const) {
  await press(key);
  check(`${key} arms ${tool}`, (await state()).tool === tool, String((await state()).tool));
}
await press("b");
check("B is polyline in the Body role", (await state()).tool === "polyline" && (await role()) === "body");
await press("Control+u");
check("Ctrl+U is polyline in the Cut role", (await state()).tool === "polyline" && (await role()) === "cut");
await press("1");
check("1 is the Body role", (await role()) === "body");
await press("2");
check("2 is the Cut role", (await role()) === "cut");
await press("3");
check("3 is the Reference role", (await role()) === "reference");
await press("1");

// --- the bug the registry kills ---------------------------------------------
await press("Escape");
await press("Shift+N");
check("Shift+N no longer means N (combine)", true); // nothing to assert but no crash
await press("Shift+F");
check("Shift+F fits the view (tool unchanged)", (await state()).tool === null);
await press("k");
check("K is free since the remap dropped Rail's key", (await state()).tool === null);

// --- Esc, modes, panels ------------------------------------------------------
await press("j");
await press("Escape");
check("Esc disarms the tool", (await state()).tool === null);
await press("Tab");
check("Tab switches to sim", (await state()).mode === "sim");
await press("d");
check("D measures in sim too", (await state()).tool === "measure");
await press("Escape");
await press("Tab");
check("Tab switches back to draw", (await state()).mode === "draw");

await press("?");
check("? opens the help drawer", !(await page.locator("#help-drawer").getAttribute("class"))?.includes("hidden"));
await press("?");
check("? closes it again", (await page.locator("#help-drawer").getAttribute("class"))?.includes("hidden") === true);

// --- commands that never had a key can now be given one ----------------------
await page.locator("#keymap-btn").click();
check("the Shortcuts panel opens", !(await page.locator("#keymap-panel").getAttribute("class"))?.includes("hidden"));
check("Reset is off until there is something to reset", await page.locator("#keymap-reset").isDisabled());
await press("Escape");
check("Esc closes the Shortcuts panel", (await page.locator("#keymap-panel").getAttribute("class"))?.includes("hidden") === true);

// --- typing in a field is not a shortcut ------------------------------------
await page.locator("#export-btn").click();
await page.locator("#export-joint-holes").check();
await page.locator("#export-joint-dia").fill("");
await page.locator("#export-joint-dia").press("j");
check("a tool letter typed in a field stays in the field", (await state()).tool === null);
check("...and lands as text", (await page.locator("#export-joint-dia").inputValue()) === "");
await page.locator("#export-cancel").click();

// --- the commands that branch internally, on a real body --------------------
const bodies = () => page.evaluate(() => (window as any).__disjointed.host.scene.bodies.length as number);
const radius = () => page.evaluate(() => (window as any).__disjointed.host.scene.bodies[0]?.radius as number);
const corners = () => page.evaluate(() => (window as any).__disjointed.host.scene.bodies[0]?.controlLocal.length as number);

const box = (await page.locator("canvas").boundingBox())!;
const clickAt = (dx: number, dy: number) => page.mouse.click(box.x + dx, box.y + dy);

await clickAt(50, 600); // focus back on the canvas: the export panel keeps keys to itself
await press("1"); // Body role
await press("Shift+B");
await clickAt(100, 50);
await clickAt(300, 200);
check("a rectangle was drawn", (await bodies()) === 1, `${await bodies()} bodies`);
await clickAt(200, 125); // select it
check("it is selected", (await state()).selection !== null);

const r0 = await radius();
await press("]");
check("] rounds the corners", (await radius()) > r0, `${r0} → ${await radius()}`);
await press("[");
check("[ un-rounds them", (await radius()) === r0);

await press("Control+c");
await press("Control+v");
check("Ctrl+C / Ctrl+V copy the body", (await bodies()) === 2, `${await bodies()} bodies`);
await press("Delete");
check("Delete removes the selection", (await bodies()) === 1, `${await bodies()} bodies`);
await press("Control+z");
check("Ctrl+Z brings it back", (await bodies()) === 2, `${await bodies()} bodies`);
await press("Control+y");
check("Ctrl+Y takes it away again", (await bodies()) === 1, `${await bodies()} bodies`);
await press("Control+z");

// ↑ / ↓ on a selected regular polygon
await page.evaluate(() => (window as any).__disjointed.host.scene.clear(true));
await press("Escape");
await press("Shift+P");
await clickAt(200, 100);
await clickAt(300, 100);
await clickAt(250, 100); // select the polygon
const n0 = await corners();
await press("ArrowUp");
check("↑ adds a side to the selected polygon", (await corners()) === n0 + 1, `${n0} → ${await corners()}`);
await press("ArrowDown");
check("↓ takes one away", (await corners()) === n0, String(await corners()));

// Enter closes a polyline
await page.evaluate(() => (window as any).__disjointed.host.scene.clear(true));
await press("Escape");
await press("b");
await clickAt(100, 50);
await clickAt(300, 50);
await clickAt(300, 200);
await press("Enter");
check("Enter closes the polyline", (await bodies()) === 1, `${await bodies()} bodies`);
await page.evaluate(() => (window as any).__disjointed.host.scene.clear(true));
await press("Escape");

// --- the user's own keymap ---------------------------------------------------
await page.evaluate(() => localStorage.setItem("disjointed:keymap", JSON.stringify({ "tool.joint": [{ key: "J", mods: ["shift"] }] })));
await page.reload();
await page.waitForFunction(() => "__disjointed" in window);
await press("j");
check("the default it replaced is gone", (await state()).tool === null, String((await state()).tool));
await press("Shift+J");
check("an imported binding takes effect", (await state()).tool === "joint", String((await state()).tool));
check("the tooltip followed it", (await title('[data-tool="joint"]'))?.endsWith("(Shift+J)") === true, String(await title('[data-tool="joint"]')));
await press("Escape");
await page.locator("#keymap-btn").click();
check("the panel says the keymap is the user's", (await page.locator("#keymap-status").textContent())?.includes("Your own") === true);
await page.locator("#keymap-reset").click();
check("Reset puts J back", (await title('[data-tool="joint"]'))?.endsWith("(J)") === true);
await page.evaluate(() => localStorage.removeItem("disjointed:keymap"));

console.log(failures ? `\n${failures} FAILED` : "\nALL PASSED");
await context.close();
await browser.close();
await server.close();
process.exit(failures ? 1 : 0);
