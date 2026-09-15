/**
 * The keymap format and the command registry: chord spelling, event normalization, slot
 * resolution, validation, overrides — then the three agreements that keep a shortcut in
 * one place: the registry against itself, `public/keymap.json` against the registry, and
 * `index.html`'s `data-cmd` / `data-tool` attributes against both.
 *
 * `tsx scripts/keymap.ts --write` regenerates `public/keymap.json` from the registry
 * before checking (that is `npm run keymap:file`).
 */
import { readFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import {
  Binding, KeyEventLike, MODS, SCHEMA, bindingOfEvent, contextsOverlap, findConflicts,
  formatChord, keyKind, normalizeKeyName, overridesOfFile, parseChord, parseKeymap,
  parseOverrides, shiftIsModifier, slotOf, slotOfChord, slotOfEvent,
} from "../src/keymap";
import {
  COMMAND_LIST, CONTEXTS, GROUPS, TOOLS, commandById, defaultBindings, toKeymapFile,
} from "../src/commands";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const KEYMAP_PATH = join(root, "public", "keymap.json");
const write = process.argv.includes("--write");
/** Newline-blind: a checkout with core.autocrlf on hands these files back as CRLF. */
const lf = (text: string): string => text.split("\r\n").join("\n");

// --- key spelling ------------------------------------------------------------

check("a letter is spelled uppercase", normalizeKeyName("b") === "B");
check("an uppercase letter stays", normalizeKeyName("B") === "B");
check("Escape is spelled Esc", normalizeKeyName("Escape") === "Esc");
check("a space is the named key Space", normalizeKeyName(" ") === "Space");
check("arrows are spelled as glyphs", normalizeKeyName("ArrowUp") === "↑" && normalizeKeyName("ArrowLeft") === "←");
check("function keys pass through", normalizeKeyName("F1") === "F1" && normalizeKeyName("F12") === "F12");
check("punctuation is itself", normalizeKeyName("?") === "?" && normalizeKeyName("[") === "[");
check("digits are themselves", normalizeKeyName("0") === "0");
check("a bare modifier is not a key", normalizeKeyName("Shift") === null && normalizeKeyName("Control") === null);
check("a named key typed by hand is accepted", normalizeKeyName("pageup") === "PageUp");

check("letters take Shift", keyKind("B") === "letter" && shiftIsModifier("B"));
check("named keys take Shift", keyKind("Enter") === "named" && shiftIsModifier("Enter"));
check("characters do not take Shift", keyKind("?") === "char" && !shiftIsModifier("?"));

// --- chords ------------------------------------------------------------------

const chords: [string, string][] = [
  ["B", "B|"],
  ["Shift+B", "B|shift"],
  ["Ctrl+S", "S|ctrl"],
  ["Ctrl+Shift+S", "S|ctrl+shift"],
  ["Alt+Shift+Ctrl+S", "S|ctrl+shift+alt"], // modifiers are re-ordered into the slot's order
  ["cmd+z", "Z|ctrl"], // Cmd is Ctrl, so one file serves both platforms
  ["F1", "F1|"],
  ["↑", "↑|"],
  ["Esc", "Esc|"],
  ["?", "?|"],
  ["[", "[|"],
];
for (const [chord, slot] of chords) {
  check(`slot of ${chord} is ${slot}`, slotOfChord(chord) === slot, String(slotOfChord(chord)));
}
check("a Shift on a character key is dropped", slotOfChord("Shift+?") === "?|");
check("an unknown modifier is refused", parseChord("Hyper+B") === null);
check("an empty chord is refused", parseChord("") === null && parseChord("   ") === null);
check("a lone + is a key", slotOfChord("Ctrl++") === "+|ctrl", String(slotOfChord("Ctrl++")));
check("formatChord round-trips", formatChord(parseChord("ctrl+shift+s")!) === "Ctrl+Shift+S");
check("formatChord orders modifiers", formatChord({ key: "B", mods: ["alt", "ctrl"] }) === "Ctrl+Alt+B");

// --- events ------------------------------------------------------------------

const ev = (e: KeyEventLike) => slotOfEvent(e);
check("plain letter", ev({ key: "b" }) === "B|");
check("Shift+letter is its own slot", ev({ key: "B", shiftKey: true }) === "B|shift");
check("Ctrl and Cmd are the same modifier", ev({ key: "s", ctrlKey: true }) === ev({ key: "s", metaKey: true }));
check("? arrives with Shift held and must not record it", ev({ key: "?", shiftKey: true }) === "?|");
check("Space", ev({ key: " " }) === "Space|");
check("Escape", ev({ key: "Escape" }) === "Esc|");
check("ArrowDown", ev({ key: "ArrowDown" }) === "↓|");
check("a bare modifier resolves to nothing", ev({ key: "Shift", shiftKey: true }) === null);
check("Alt is recorded", ev({ key: "b", altKey: true }) === "B|alt");
check("modifier order in a slot is fixed", ev({ key: "b", altKey: true, shiftKey: true, ctrlKey: true }) === "B|ctrl+shift+alt");
check(
  "a keystroke and its chord agree",
  ev({ key: "S", ctrlKey: true, shiftKey: true }) === slotOfChord("Ctrl+Shift+S")
);
check("bindingOfEvent gives the binding a file would carry", JSON.stringify(bindingOfEvent({ key: "b", shiftKey: true })) === JSON.stringify({ key: "B", mods: ["shift"] }));

// --- contexts and conflicts --------------------------------------------------

check("any overlaps everything", contextsOverlap("any", "draw") && contextsOverlap("sim", "any"));
check("two named contexts don't overlap", !contextsOverlap("draw", "sim"));
check("a context overlaps itself", contextsOverlap("draw", "draw"));
check("an absent context counts as any", contextsOverlap(undefined, "draw"));

const B = (chord: string): Binding[] => [parseChord(chord)!];
check(
  "same slot, overlapping contexts is a conflict",
  findConflicts([
    { id: "a", context: "draw", bindings: B("B") },
    { id: "b", context: "any", bindings: B("B") },
  ]).length === 1
);
check(
  "same slot, different modes is not",
  findConflicts([
    { id: "a", context: "draw", bindings: B("B") },
    { id: "b", context: "sim", bindings: B("B") },
  ]).length === 0
);
check(
  "Shift+B does not collide with B",
  findConflicts([
    { id: "a", context: "any", bindings: B("B") },
    { id: "b", context: "any", bindings: B("Shift+B") },
  ]).length === 0
);

// --- reading a file ----------------------------------------------------------

const minimal = {
  schema: SCHEMA,
  commands: [{ id: "x", label: "X", bindings: [{ key: "b", mods: ["shift"] }], mystery: { deep: 1 } }],
};
{
  const { file, errors } = parseKeymap(minimal);
  check("a minimal file reads", !!file && errors.length === 0, errors.map((e) => e.message).join("; "));
  check("keys are normalized on load", file ? slotOf(file.commands[0].bindings[0]) === "B|shift" : false);
  check(
    "unknown fields survive",
    JSON.stringify(file?.commands[0].mystery) === JSON.stringify({ deep: 1 })
  );
}
check("a foreign schema is refused", parseKeymap({ schema: "keymap/2", commands: [] }).file === null);
check("a file with no commands is refused", parseKeymap({ schema: SCHEMA, commands: [] }).file === null);
check("a command with no id is refused", parseKeymap({ schema: SCHEMA, commands: [{ bindings: [] }] }).file === null);
check(
  "a duplicate id is refused",
  parseKeymap({ schema: SCHEMA, commands: [{ id: "a", bindings: [] }, { id: "a", bindings: [] }] }).file === null
);
check(
  "missing bindings is refused (an empty list is the way to say 'no shortcut')",
  parseKeymap({ schema: SCHEMA, commands: [{ id: "a" }] }).file === null
);
check(
  "zero bindings is a real state",
  parseKeymap({ schema: SCHEMA, commands: [{ id: "a", bindings: [] }] }).file?.commands[0].bindings.length === 0
);
check("not an object", parseKeymap("nope").file === null && parseKeymap(null).file === null);

// --- overrides ---------------------------------------------------------------

check("overrides read", JSON.stringify(parseOverrides('{"a":[{"key":"q","mods":[]}]}')) === '{"a":[{"key":"Q","mods":[]}]}');
check("broken overrides are ignored, not fatal", parseOverrides("{oops") === null && parseOverrides(null) === null);
check("an unreadable binding inside overrides is dropped", parseOverrides('{"a":[{"key":"Shift"}]}')?.a.length === 0);
{
  const { file } = parseKeymap({
    schema: SCHEMA,
    commands: [{ id: "known", bindings: [{ key: "q" }] }, { id: "alien", bindings: [{ key: "w" }] }],
  });
  const map = overridesOfFile(file!, new Set(["known"]));
  check("an imported file yields overrides for known commands only", JSON.stringify(Object.keys(map)) === '["known"]');
}

// --- the registry ------------------------------------------------------------

{
  const ids = COMMAND_LIST.map((c) => c.id);
  check("ids are unique", new Set(ids).size === ids.length);
  check("ids are stable-looking (group.name)", ids.every((id) => /^[a-z]+\.[A-Za-z]+$/.test(id)), ids.filter((id) => !/^[a-z]+\.[A-Za-z]+$/.test(id)).join(" "));

  const groupIds = new Set(GROUPS.map((g) => g.id as string));
  const contextIds = new Set(CONTEXTS.map((c) => c.id as string));
  const toolIds = new Set<string>(TOOLS);
  for (const c of COMMAND_LIST) {
    if (!groupIds.has(c.group)) check(`${c.id} has a real group`, false, c.group);
    if (!contextIds.has(c.context)) check(`${c.id} has a real context`, false, c.context);
    if (c.tool && !toolIds.has(c.tool)) check(`${c.id} names a real tool`, false, c.tool);
    if (!c.label || !c.description) check(`${c.id} is described`, false);
    for (const chord of c.keys ?? []) {
      if (!parseChord(chord)) check(`${c.id} has a readable chord`, false, chord);
    }
  }
  check("every group, context, tool and chord in the registry is real", true);

  check("every group is used", GROUPS.every((g) => COMMAND_LIST.some((c) => c.group === g.id)), GROUPS.filter((g) => !COMMAND_LIST.some((c) => c.group === g.id)).map((g) => g.id).join(" "));
  check("every tool is reachable from some command", TOOLS.every((t) => COMMAND_LIST.some((c) => c.tool === t)), TOOLS.filter((t) => !COMMAND_LIST.some((c) => c.tool === t)).join(" "));

  const conflicts = findConflicts(COMMAND_LIST.map((c) => ({ ...c, bindings: defaultBindings(c) })));
  check("no two shipped commands share a slot in an overlapping context", conflicts.length === 0,
    conflicts.map((c) => `${c.slot}: ${c.ids.join(" / ")}`).join("; "));

  // The bug the registry exists to kill: resolution is by exact slot, so a Shift+letter
  // can never be swallowed by the plain letter's command.
  const shiftSlots = COMMAND_LIST.flatMap((c) => defaultBindings(c)).filter((b) => b.mods.includes("shift"));
  check("Shift bindings are recorded as their own slots", shiftSlots.every((b) => slotOf(b).endsWith("|shift") || slotOf(b).includes("+shift")));
  check("modifier list is the documented one", MODS.join(",") === "ctrl,shift,alt");
}

// --- public/keymap.json ------------------------------------------------------

const generated = JSON.stringify(toKeymapFile(), null, 2) + "\n";
if (write) {
  writeFileSync(KEYMAP_PATH, generated, "utf8");
  console.log(`wrote public/keymap.json (${COMMAND_LIST.length} commands)`);
}
{
  let onDisk: string | null = null;
  try {
    onDisk = readFileSync(KEYMAP_PATH, "utf8");
  } catch {
    onDisk = null;
  }
  check("public/keymap.json exists", onDisk !== null, "run npm run keymap:file");
  if (onDisk !== null) {
    // Compared newline-blind: a checkout with core.autocrlf on hands back CRLF.
    check("public/keymap.json is what the registry generates", lf(onDisk) === generated, "run npm run keymap:file");
    const { file, errors } = parseKeymap(JSON.parse(onDisk));
    check("public/keymap.json is a valid keymap/1 file", !!file, errors.map((e) => e.message).join("; "));
  }
}

// --- index.html --------------------------------------------------------------

{
  const html = readFileSync(join(root, "index.html"), "utf8");
  const cmdAttrs = [...html.matchAll(/data-cmd="([^"]+)"/g)].flatMap((m) => m[1].trim().split(/\s+/));
  check("index.html carries data-cmd attributes", cmdAttrs.length > 0);
  const unknown = cmdAttrs.filter((id) => !commandById(id));
  check("every data-cmd resolves to a command", unknown.length === 0, unknown.join(" "));

  const toolAttrs = [...html.matchAll(/data-tool="([^"]+)"/g)].map((m) => m[1]);
  const unknownTools = toolAttrs.filter((t) => !COMMAND_LIST.some((c) => c.tool === t));
  check("every data-tool button has a command", unknownTools.length === 0, unknownTools.join(" "));

  // Shortcuts are appended from the registry at startup, so no title may carry its own.
  const titles = [...html.matchAll(/title="([^"]*)"/g)].map((m) => m[1]);
  const chordish = /\((?:Ctrl|Shift|Alt|F\d|Esc|Tab|Space|Page(?:Up|Down)|Delete|[A-Z0-9])(?:\+[A-Za-z]+)?\)/;
  const stale = titles.filter((t) => chordish.test(t));
  check("no tooltip spells a shortcut itself", stale.length === 0, stale.slice(0, 3).join(" | "));
}

console.log(failures ? `\n${failures} FAILED` : "\nALL PASSED");
if (failures) process.exit(1);
