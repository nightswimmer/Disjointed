/**
 * Shortcut lists written from the registry, so no document can go stale.
 *
 * Two targets, both regenerated in place — neither carries a marker comment, because both
 * already have a shape the generator can find:
 *
 * - **README.md** — the Shortcut column of the tools table (the one headed
 *   `| Tool | Shortcut | Action |`). Rows are matched by the bold name in their first
 *   column, through `ROWS` below; an unmatched row is an error rather than a silent skip.
 * - **public/help/index.html** — the `<ul>` inside `<section id="shortcuts">`, rebuilt as
 *   one line per command group.
 *
 *   tsx scripts/keymap-docs.ts            # write both
 *   tsx scripts/keymap-docs.ts --check    # report staleness, change nothing (exit 1 if stale)
 *   tsx scripts/keymap-docs.ts --readme   # one target only (also --manual)
 *
 * The manual is generated, not hand-edited: run this before `npm run manual`, not after.
 */
import { readFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import { formatChord } from "../src/keymap";
import { COMMAND_LIST, CommandSpec, GROUPS, commandById, defaultBindings } from "../src/commands";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const args = process.argv.slice(2);
const check = args.includes("--check");
const only = args.includes("--readme") ? "readme" : args.includes("--manual") ? "manual" : "both";

/** The tools table's rows: the bold name in column 1 → the commands that arm it. */
const ROWS: Record<string, string[]> = {
  "Role switch": ["role.body", "role.cut", "role.reference"],
  Polyline: ["shape.polylineBody", "shape.polylineCut"],
  Rectangle: ["shape.rect"],
  Circle: ["shape.circle"],
  "Regular polygon": ["shape.polygon"],
  Slot: ["shape.slot"],
  Line: ["shape.line"],
  Arc: ["shape.arc"],
  Text: ["shape.text"],
  "Linear pattern": ["tool.patternLinear"],
  "Circular pattern": ["tool.patternCircular"],
  Split: ["tool.split"],
  Joint: ["tool.joint"],
  Weld: ["tool.weld"],
  Connect: ["tool.connect"],
  Ground: ["tool.ground"],
  Rail: ["tool.rail"],
  Slider: ["tool.slider"],
  Rotate: ["tool.rotate"],
  "Linear actuator": ["tool.linearActuator"],
  Motor: ["tool.motor"],
  Measure: ["tool.measure"],
  Coincident: ["tool.coincident"],
  "Horizontal** / **Vertical": ["tool.horizontal", "tool.vertical"],
  "Parallel** / **Perpendicular** / **Equal": ["tool.parallel", "tool.perpendicular", "tool.equal"],
  Fixed: ["tool.fixed"],
  Symmetrical: ["tool.symmetric"],
  Tangential: ["tool.tangent"],
};

const chordsOf = (c: CommandSpec): string[] => defaultBindings(c).map(formatChord);
/** "Polyline (body)" → "body"; a label with no parenthetical qualifies as itself. */
const qualifier = (c: CommandSpec): string => /\(([^)]+)\)\s*$/.exec(c.label)?.[1] ?? c.label;

/**
 * A row's Shortcut cell. One command is its keys; several are separated — with the
 * qualifier when the labels carry one ("`B` body · `U` cut"), else plainly ("`H` / `V`").
 */
function shortcutCell(ids: string[]): string {
  const cmds = ids.map((id) => {
    const c = commandById(id);
    if (!c) throw new Error(`README row names an unknown command: ${id}`);
    return c;
  });
  const qualified = cmds.length > 1 && cmds.every((c) => /\([^)]+\)\s*$/.test(c.label));
  const cells = cmds.map((c) => {
    const keys = chordsOf(c).map((k) => `\`${k}\``).join(" / ") || "—";
    return qualified ? `${keys} ${qualifier(c)}` : keys;
  });
  return cells.join(qualified ? " · " : " / ");
}

function rewriteReadme(text: string): string {
  const lines = text.split("\n");
  const header = lines.findIndex((l) => l.startsWith("| Tool | Shortcut | Action |"));
  if (header < 0) throw new Error("README.md: the tools table is gone — no `| Tool | Shortcut | Action |` header.");
  const seen = new Set<string>();
  for (let i = header + 2; i < lines.length && lines[i].startsWith("|"); i++) {
    const m = /^\| \*\*(.+?)\*\* \| ([^|]*?) \| /.exec(lines[i]);
    if (!m) throw new Error(`README.md line ${i + 1}: a table row the generator cannot read.`);
    const ids = ROWS[m[1]];
    if (!ids) throw new Error(`README.md line ${i + 1}: no commands mapped for the row "${m[1]}" — add it to ROWS.`);
    seen.add(m[1]);
    lines[i] = lines[i].replace(`| **${m[1]}** | ${m[2]} | `, `| **${m[1]}** | ${shortcutCell(ids)} | `);
  }
  const missing = Object.keys(ROWS).filter((k) => !seen.has(k));
  if (missing.length) throw new Error(`README.md: ROWS names rows the table no longer has: ${missing.join(", ")}`);
  return lines.join("\n");
}

/** "Ctrl+Shift+S" → "<kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>S</kbd>". */
const kbd = (chord: string): string =>
  chord.split("+").map((part) => `<kbd>${part === "" ? "+" : part}</kbd>`).join("+");

/** The manual's list: one line per group that has a shortcut in it. */
function shortcutList(indent: string): string {
  const out: string[] = [];
  for (const g of GROUPS) {
    const entries = COMMAND_LIST.filter((c) => c.group === g.id && chordsOf(c).length > 0).map(
      (c) => `${chordsOf(c).map(kbd).join(" / ")} ${c.label.toLowerCase()}`
    );
    if (entries.length) out.push(`${indent}<li><strong>${g.label}</strong>: ${entries.join(" &middot; ")}</li>`);
  }
  return out.join("\n");
}

function rewriteManual(text: string): string {
  const section = text.indexOf('<section class="topic" id="shortcuts">');
  if (section < 0) throw new Error("public/help/index.html: no #shortcuts section.");
  const open = text.indexOf("<ul>", section);
  const close = text.indexOf("</ul>", open);
  if (open < 0 || close < 0) throw new Error("public/help/index.html: the #shortcuts list is gone.");
  const indent = " ".repeat(text.slice(text.lastIndexOf("\n", open) + 1).search(/\S/) + 2);
  // Keep whatever newline the checkout uses (core.autocrlf hands these files back as CRLF).
  const nl = text.includes("\r\n") ? "\r\n" : "\n";
  const list = shortcutList(indent).split("\n").join(nl);
  return `${text.slice(0, open + 4)}${nl}${list}${nl}${indent.slice(2)}${text.slice(close)}`;
}

let stale = 0;
for (const [name, path, rewrite] of [
  ["readme", join(root, "README.md"), rewriteReadme],
  ["manual", join(root, "public", "help", "index.html"), rewriteManual],
] as const) {
  if (only !== "both" && only !== name) continue;
  const before = readFileSync(path, "utf8");
  const after = rewrite(before);
  if (before === after) {
    console.log(`${name}: up to date`);
    continue;
  }
  stale++;
  if (check) {
    console.log(`${name}: STALE — run npm run keymap:docs`);
  } else {
    writeFileSync(path, after, "utf8");
    console.log(`${name}: rewritten from the registry`);
  }
}
if (check && stale) process.exit(1);
