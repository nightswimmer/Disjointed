/**
 * The keymap format — `keymap/1`, the file Disjointed shares with **KeyMapper**.
 *
 * This module is the *format*, not the behaviour: chord spelling, event normalization,
 * slot resolution, validation, the user's overrides, and the serialisation of a whole
 * keymap file. It knows nothing about the app's state or the DOM, so `scripts/keymap.ts`
 * can exercise every rule headless. The command table itself (labels, groups, default
 * chords) is `src/commands.ts`; what a command *does* lives beside the key handler in
 * `src/main.ts`.
 *
 * The format is specified normatively in KeyMapper's README — this file implements it and
 * does not restate it. Two rules matter enough to repeat, because getting them wrong is
 * silent:
 *
 * - A **slot** is `KEY|mods` with the mods in the fixed order `ctrl+shift+alt`, e.g.
 *   `"B|shift"`, `"S|ctrl+shift"`, `"?|"`. Both sides of a lookup build it the same way,
 *   which is the whole point of having one spelling.
 * - Shift is recorded **only** for letters and named keys. For every other printable
 *   character the character already encodes it: a browser reports `?` as the key `"?"`
 *   with `shiftKey` true, so a binding that also listed `shift` could never match.
 */

/** Modifier names, in the order a slot spells them. */
export const MODS = ["ctrl", "shift", "alt"] as const;
export type Mod = (typeof MODS)[number];

/** One keystroke: a key spelling plus the modifiers held with it. */
export interface Binding {
  key: string;
  mods: Mod[];
}

/** A command as the *file* carries it (extension fields included). */
export interface KeymapCommand {
  id: string;
  label?: string;
  description?: string;
  group?: string;
  context?: string;
  bindings: Binding[];
  fixed?: boolean;
  tags?: string[];
  /** Disjointed extension: the command arms this tool. */
  tool?: string;
  /** Disjointed extension: the command answers even while focus is in a field. */
  whileTyping?: boolean;
  /** Anything else the file carried is preserved verbatim. */
  [extra: string]: unknown;
}

export interface KeymapFile {
  schema: string;
  app?: { name?: string; version?: string; [extra: string]: unknown };
  contexts?: { id: string; label?: string }[];
  groups?: { id: string; label?: string }[];
  commands: KeymapCommand[];
  [extra: string]: unknown;
}

export const SCHEMA = "keymap/1";

/** Named keys, in the file's spelling. Shift is a modifier on all of them. */
export const NAMED_KEYS = [
  "Tab", "Enter", "Esc", "Space", "Backspace", "Delete", "Insert",
  "Home", "End", "PageUp", "PageDown",
  "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
  "↑", "↓", "←", "→",
] as const;

const NAMED = new Set<string>(NAMED_KEYS);

/** `KeyboardEvent.key` spellings that are a named key under another name. */
const KEY_ALIASES: Record<string, string> = {
  Escape: "Esc",
  " ": "Space",
  Spacebar: "Space",
  ArrowUp: "↑",
  ArrowDown: "↓",
  ArrowLeft: "←",
  ArrowRight: "→",
  Del: "Delete",
  Up: "↑",
  Down: "↓",
  Left: "←",
  Right: "→",
};

/** How a key spelling behaves: letters and named keys take Shift, characters don't. */
export type KeyKind = "letter" | "named" | "char";

export function keyKind(key: string): KeyKind | null {
  if (NAMED.has(key)) return "named";
  if (key.length === 1) {
    if (/[A-Za-z]/.test(key)) return "letter";
    // Any other single printable character. Whitespace is out: a space is the named
    // key "Space", and a tab is "Tab".
    if (key.trim() !== "") return "char";
  }
  return null;
}

/** Whether a binding on `key` records Shift as a modifier (see the header). */
export const shiftIsModifier = (key: string): boolean => keyKind(key) !== "char";

/**
 * Canonical file spelling for a key, or null if it isn't one we can bind.
 * Accepts a `KeyboardEvent.key` (`"Escape"`, `"a"`, `"ArrowUp"`) or a file spelling.
 */
export function normalizeKeyName(raw: string): string | null {
  if (!raw) return null;
  const alias = KEY_ALIASES[raw];
  if (alias) return alias;
  if (NAMED.has(raw)) return raw;
  // Case-insensitive match for named keys typed by hand ("esc", "pageup").
  const named = (NAMED_KEYS as readonly string[]).find((n) => n.toLowerCase() === raw.toLowerCase());
  if (named) return named;
  if (raw.length === 1) {
    if (/[a-z]/.test(raw)) return raw.toUpperCase();
    return keyKind(raw) ? raw : null;
  }
  return null;
}

// --- chords ------------------------------------------------------------------

/** Sort modifiers into the slot's fixed order and drop duplicates. */
function orderMods(mods: readonly string[]): Mod[] {
  return MODS.filter((m) => mods.includes(m));
}

/**
 * Parse a human chord — `"Shift+B"`, `"Ctrl+Shift+S"`, `"F1"`, `"↑"`, `"?"` — into a
 * binding. Returns null when the key isn't bindable or a modifier is unknown; a Shift
 * listed on a character key is dropped (the character already encodes it), which keeps
 * `"Shift+/"` from becoming a slot nothing can produce.
 */
export function parseChord(chord: string): Binding | null {
  const text = chord.trim();
  if (!text) return null;
  // Split on "+" but keep a trailing "+" as the key itself ("Ctrl++").
  const parts = text.split("+");
  if (parts.length > 1 && parts[parts.length - 1] === "") {
    parts.pop();
    parts[parts.length - 1] = "+";
  }
  const mods: Mod[] = [];
  let key: string | null = null;
  for (let i = 0; i < parts.length; i++) {
    const part = parts[i].trim();
    const low = part.toLowerCase();
    const isLast = i === parts.length - 1;
    if (!isLast) {
      if (low === "ctrl" || low === "control" || low === "cmd" || low === "meta") mods.push("ctrl");
      else if (low === "shift") mods.push("shift");
      else if (low === "alt" || low === "option") mods.push("alt");
      else return null;
    } else {
      key = normalizeKeyName(part);
    }
  }
  if (!key) return null;
  const ordered = orderMods(mods);
  return { key, mods: shiftIsModifier(key) ? ordered : ordered.filter((m) => m !== "shift") };
}

/** A binding as a person reads it: `"Ctrl+Shift+S"`, `"Shift+B"`, `"F1"`. */
export function formatChord(b: Binding): string {
  const names: Record<Mod, string> = { ctrl: "Ctrl", shift: "Shift", alt: "Alt" };
  return [...orderMods(b.mods).map((m) => names[m]), b.key].join("+");
}

// --- slots -------------------------------------------------------------------

/** The lookup key for a binding: `KEY|mods`, mods in `ctrl+shift+alt` order. */
export function slotOf(b: Binding): string {
  return `${b.key}|${orderMods(b.mods).join("+")}`;
}

/** The slot a chord resolves to, or null if the chord is unbindable. */
export function slotOfChord(chord: string): string | null {
  const b = parseChord(chord);
  return b ? slotOf(b) : null;
}

/** The parts of a `KeyboardEvent` this module reads (so tests need no DOM). */
export interface KeyEventLike {
  key: string;
  ctrlKey?: boolean;
  metaKey?: boolean;
  shiftKey?: boolean;
  altKey?: boolean;
}

/**
 * The binding a keystroke asks for, or null for a keystroke that can't carry one
 * (a bare modifier, a dead key). `ctrl` covers Control and Command, so one file works
 * on both platforms.
 */
export function bindingOfEvent(e: KeyEventLike): Binding | null {
  const key = normalizeKeyName(e.key);
  if (!key) return null;
  const mods: Mod[] = [];
  if (e.ctrlKey || e.metaKey) mods.push("ctrl");
  if (e.shiftKey && shiftIsModifier(key)) mods.push("shift");
  if (e.altKey) mods.push("alt");
  return { key, mods };
}

/** The slot a keystroke resolves to, or null (see `bindingOfEvent`). */
export function slotOfEvent(e: KeyEventLike): string | null {
  const b = bindingOfEvent(e);
  return b ? slotOf(b) : null;
}

// --- validation --------------------------------------------------------------

export interface KeymapIssue {
  /** The command the issue is about, when it belongs to one. */
  id?: string;
  message: string;
}

/** Two contexts overlap when they are the same or either is "any" (or unset). */
export function contextsOverlap(a: string | undefined, b: string | undefined): boolean {
  const ca = a ?? "any";
  const cb = b ?? "any";
  return ca === "any" || cb === "any" || ca === cb;
}

export interface Conflict {
  slot: string;
  ids: string[];
}

/** The least a conflict check needs to know about a command. */
export interface BoundCommand {
  id: string;
  context?: string;
  bindings: readonly Binding[];
}

/** Commands that share a slot with an overlapping context — the file's only hard clash. */
export function findConflicts(commands: readonly BoundCommand[]): Conflict[] {
  const bySlot = new Map<string, BoundCommand[]>();
  for (const c of commands) {
    for (const b of c.bindings) {
      const slot = slotOf(b);
      const list = bySlot.get(slot);
      if (list) list.push(c);
      else bySlot.set(slot, [c]);
    }
  }
  const out: Conflict[] = [];
  for (const [slot, list] of bySlot) {
    for (let i = 0; i < list.length; i++) {
      for (let j = i + 1; j < list.length; j++) {
        if (contextsOverlap(list[i].context, list[j].context)) {
          out.push({ slot, ids: [list[i].id, list[j].id] });
        }
      }
    }
  }
  return out;
}

/**
 * Read an unknown value as a keymap file. Returns the file and the problems found;
 * a file with any issue in `errors` must not be used. Unknown fields are kept
 * untouched — that guarantee is what lets a project carry its own wiring in the file.
 */
export function parseKeymap(data: unknown): { file: KeymapFile | null; errors: KeymapIssue[] } {
  const errors: KeymapIssue[] = [];
  const fail = (message: string, id?: string) => {
    errors.push(id === undefined ? { message } : { id, message });
  };
  if (typeof data !== "object" || data === null || Array.isArray(data)) {
    fail("Not a keymap object.");
    return { file: null, errors };
  }
  const raw = data as Record<string, unknown>;
  if (raw.schema !== SCHEMA) {
    fail(`Unsupported schema ${JSON.stringify(raw.schema ?? null)} — this app reads ${SCHEMA}.`);
    return { file: null, errors };
  }
  if (!Array.isArray(raw.commands) || raw.commands.length === 0) {
    fail("The file lists no commands.");
    return { file: null, errors };
  }
  const seen = new Set<string>();
  const commands: KeymapCommand[] = [];
  for (const entry of raw.commands as unknown[]) {
    if (typeof entry !== "object" || entry === null || Array.isArray(entry)) {
      fail("A command is not an object.");
      continue;
    }
    const cmd = { ...(entry as Record<string, unknown>) } as KeymapCommand;
    if (typeof cmd.id !== "string" || !cmd.id) {
      fail("A command has no id.");
      continue;
    }
    if (seen.has(cmd.id)) {
      fail("Duplicate command id.", cmd.id);
      continue;
    }
    seen.add(cmd.id);
    const bindings = (entry as Record<string, unknown>).bindings;
    if (!Array.isArray(bindings)) {
      fail("`bindings` is missing (use [] for a command with no shortcut).", cmd.id);
      continue;
    }
    const ok: Binding[] = [];
    for (const b of bindings as unknown[]) {
      const parsed = readBinding(b);
      if (!parsed) {
        fail(`Unreadable binding ${JSON.stringify(b)}.`, cmd.id);
        continue;
      }
      ok.push(parsed);
    }
    cmd.bindings = ok;
    commands.push(cmd);
  }
  if (errors.length) return { file: null, errors };
  return { file: { ...(raw as object), schema: SCHEMA, commands } as KeymapFile, errors };
}

/** One `{key, mods}` entry, normalized; null when it names no bindable keystroke. */
function readBinding(value: unknown): Binding | null {
  if (typeof value !== "object" || value === null || Array.isArray(value)) return null;
  const raw = value as { key?: unknown; mods?: unknown };
  if (typeof raw.key !== "string") return null;
  const key = normalizeKeyName(raw.key);
  if (!key) return null;
  const mods: Mod[] = [];
  if (raw.mods !== undefined) {
    if (!Array.isArray(raw.mods)) return null;
    for (const m of raw.mods as unknown[]) {
      if (typeof m !== "string") return null;
      const low = m.toLowerCase();
      if (low === "ctrl" || low === "cmd" || low === "meta" || low === "control") mods.push("ctrl");
      else if (low === "shift") mods.push("shift");
      else if (low === "alt" || low === "option") mods.push("alt");
      else return null;
    }
  }
  const ordered = orderMods(mods);
  return { key, mods: shiftIsModifier(key) ? ordered : ordered.filter((m) => m !== "shift") };
}

// --- the user's overrides ----------------------------------------------------

/**
 * A user's keymap in localStorage: the whole `{commandId: bindings[]}` map, not a diff
 * against the shipped defaults — a diff against a moving default is a migration problem
 * nobody wants. A command the map doesn't mention (one added by a later version) keeps
 * its default.
 */
export type KeymapOverrides = Record<string, Binding[]>;

/** Read an overrides map, dropping anything unreadable rather than failing the app. */
export function parseOverrides(text: string | null): KeymapOverrides | null {
  if (!text) return null;
  let data: unknown;
  try {
    data = JSON.parse(text);
  } catch {
    return null;
  }
  if (typeof data !== "object" || data === null || Array.isArray(data)) return null;
  const out: KeymapOverrides = {};
  for (const [id, value] of Object.entries(data as Record<string, unknown>)) {
    if (!Array.isArray(value)) continue;
    const list: Binding[] = [];
    for (const b of value as unknown[]) {
      const parsed = readBinding(b);
      if (parsed) list.push(parsed);
    }
    out[id] = list;
  }
  return out;
}

/** The overrides map a loaded keymap file stands for: every command it knows about. */
export function overridesOfFile(file: KeymapFile, known: ReadonlySet<string>): KeymapOverrides {
  const out: KeymapOverrides = {};
  for (const c of file.commands) {
    if (known.has(c.id)) out[c.id] = c.bindings;
  }
  return out;
}
