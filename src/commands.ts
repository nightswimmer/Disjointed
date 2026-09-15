/**
 * The command registry — everything the app can be *asked* to do, as data.
 *
 * One entry per command: a stable id, what it is called, one line saying what it does,
 * the group and mode it belongs to, the tool it arms (if any) and the keys it ships with.
 * `public/keymap.json` is generated from this table, tooltips and the documentation read
 * it, and `src/main.ts` supplies the other half — the `run()` that closes over the app's
 * state, keyed by the same ids and checked for completeness by the compiler.
 *
 * The split is *data* vs *behaviour*, not file size: this module must stay free of the
 * DOM and of app state so `scripts/keymap.ts` can check the table, the file and the
 * markup against each other headless.
 *
 * Adding a command: add it here, then add its action in `main.ts` (the compiler will ask
 * for it), give its button a `data-cmd`, and run `npm run keymap:file`.
 *
 * The **id is the contract** — a saved user keymap keys on it. Rename a label freely;
 * never rename an id.
 */
import { CONSTRAINT_NAME, SketchConstraintKind } from "./model";
import { Binding, KeymapFile, SCHEMA, parseChord } from "./keymap";

// --- the tool vocabulary -----------------------------------------------------

/** The shape tools: role-neutral geometry the armed role turns into a body, a cut or a reference. */
export const SHAPE_TOOLS = ["polyline", "rect", "circle", "polygon", "slot", "line", "arc", "text"] as const;
export type ShapeTool = (typeof SHAPE_TOOLS)[number];

/** Everything else that can be armed, apart from the one-shot constraint tools. */
const OTHER_TOOLS = [
  "split", "joint", "weld", "connect", "ground", "rail", "slider", "rotate",
  "linearActuator", "motor", "measure", "patternLinear", "patternCircular",
] as const;

/** Each sketch-constraint kind is its own one-shot tool (tool name = constraint kind). */
export const CONSTRAINT_KINDS = Object.keys(CONSTRAINT_NAME) as SketchConstraintKind[];

export type Tool = ShapeTool | (typeof OTHER_TOOLS)[number] | SketchConstraintKind;

/** Every tool id, for the checks a type cannot make at run time. */
export const TOOLS: readonly Tool[] = [...SHAPE_TOOLS, ...OTHER_TOOLS, ...CONSTRAINT_KINDS];

/** What a finished shape becomes. */
export type ShapeRole = "body" | "cut" | "reference";

// --- groups and contexts -----------------------------------------------------

/** Command groups — the toolbar's sections, plus the two that have no section of their own. */
export const GROUPS = [
  { id: "file", label: "File" },
  { id: "edit", label: "Edit" },
  { id: "mode", label: "Mode" },
  { id: "anim", label: "Animation" },
  { id: "role", label: "Shape role" },
  { id: "shapes", label: "Shapes" },
  { id: "pattern", label: "Patterns" },
  { id: "boolean", label: "Body operations" },
  { id: "mating", label: "Joints & mating" },
  { id: "actuator", label: "Actuators" },
  { id: "transform", label: "Transform" },
  { id: "component", label: "Components" },
  { id: "constraints", label: "Constraints" },
  { id: "grid", label: "Grid" },
  { id: "measure", label: "Measure" },
  { id: "snap", label: "Snapping" },
  { id: "view", label: "View" },
  { id: "help", label: "Help" },
] as const;
export type GroupId = (typeof GROUPS)[number]["id"];

/**
 * The states the app can be in. Declaring them is what makes conflict detection honest:
 * one letter can serve Draw and Simulate without that being a clash.
 */
export const CONTEXTS = [
  { id: "draw", label: "Draw mode" },
  { id: "sim", label: "Simulate mode" },
  { id: "any", label: "Anywhere" },
] as const;
export type Context = (typeof CONTEXTS)[number]["id"];

export interface CommandSpec {
  /** Stable and unique; a saved keymap keys on it. */
  id: string;
  /** Short name, as a menu would show it. */
  label: string;
  /** One line, in the user's terms — tooltips and the manual read this. */
  description: string;
  group: GroupId;
  context: Context;
  /** The command arms this tool (some also set the role first — see `main.ts`). */
  tool?: Tool;
  /** The shipped shortcuts, as chords. Absent or empty: the command has no key. */
  keys?: readonly string[];
  /** The command answers even while focus is in an input / select / textarea. */
  whileTyping?: boolean;
  /** Hardwired: editors must show it and refuse to move it. */
  fixed?: boolean;
  /** Badges for the editor: "planned" is a command whose action doesn't exist yet. */
  tags?: readonly string[];
}

/**
 * The table. Order is the order the board and the documentation show, and — for the two
 * commands that could ever share a slot — the order dispatch resolves in.
 */
export const COMMANDS = [
  // --- File ---
  {
    id: "file.open", label: "Open…", group: "file", context: "any", keys: ["Ctrl+O"], whileTyping: true,
    description: "Load a mechanism from a file.",
  },
  {
    id: "file.save", label: "Save", group: "file", context: "any", keys: ["Ctrl+S"], whileTyping: true,
    description: "Write the mechanism to its file.",
  },
  {
    id: "file.saveAs", label: "Save as…", group: "file", context: "any", keys: ["Ctrl+Shift+S"], whileTyping: true,
    description: "Write the mechanism to a new file.",
  },
  {
    id: "file.export", label: "Export cut file", group: "file", context: "any",
    description: "Open the cut-file panel — DXF or SVG of the selected bodies, or of all of them.",
  },
  {
    id: "file.backup", label: "Auto-backup", group: "file", context: "any",
    description: "Open the auto-backup settings: timestamped copies written to a folder.",
  },
  {
    id: "file.clear", label: "Delete everything", group: "file", context: "draw",
    description: "Empty the document — inside a component, only that definition.",
  },
  {
    id: "file.shortcuts", label: "Shortcuts…", group: "file", context: "any",
    description: "Open the keyboard-shortcut panel: import, export or reset the keymap.",
  },

  // --- Edit ---
  {
    id: "edit.undo", label: "Undo", group: "edit", context: "any", keys: ["Ctrl+Z"],
    description: "Take back the last change.",
  },
  {
    id: "edit.redo", label: "Redo", group: "edit", context: "any", keys: ["Ctrl+Shift+Z", "Ctrl+Y"],
    description: "Put back the change that was taken back.",
  },
  {
    id: "edit.copy", label: "Copy", group: "edit", context: "draw", keys: ["Ctrl+C"],
    description: "Copy the selected body or selection.",
  },
  {
    id: "edit.paste", label: "Paste", group: "edit", context: "draw", keys: ["Ctrl+V"],
    description: "Paste the copied bodies at the cursor.",
  },
  {
    id: "edit.group", label: "Group / ungroup", group: "edit", context: "draw", keys: ["Ctrl+G"],
    description: "Group the selected bodies — or dissolve the group that is selected.",
  },
  {
    id: "edit.delete", label: "Delete", group: "edit", context: "any", keys: ["Delete", "Backspace"],
    description: "Delete the selection, or the selected measurement (either mode).",
  },
  {
    id: "edit.cancel", label: "Cancel", group: "edit", context: "any", keys: ["Esc"], fixed: true,
    description: "Step back out: close a panel, drop an armed constraint, disarm the tool, leave a group or component.",
  },
  {
    id: "edit.finish", label: "Finish", group: "edit", context: "draw", keys: ["Enter"],
    description: "Close the shape being drawn, or accept a pattern's count.",
  },
  {
    id: "edit.cornerRadiusUp", label: "More corner radius", group: "edit", context: "draw", keys: ["]"],
    description: "Round the selected body's corners one step further (a disk grows).",
  },
  {
    id: "edit.cornerRadiusDown", label: "Less corner radius", group: "edit", context: "draw", keys: ["["],
    description: "Take a step of rounding off the selected body's corners (a disk shrinks).",
  },

  // --- Mode ---
  {
    id: "mode.toggle", label: "Draw / Simulate", group: "mode", context: "any", keys: ["Tab"],
    description: "Switch between building the mechanism and running it.",
  },

  // --- Animation ---
  {
    id: "anim.run", label: "Run / pause animation", group: "anim", context: "sim", keys: ["Space"],
    description: "Start or stop the actuators and motors.",
  },
  {
    id: "anim.autoPause", label: "Auto-pause", group: "anim", context: "sim",
    description: "Pause the animation as soon as the assembly becomes impossible.",
  },

  // --- Shape role ---
  {
    id: "role.body", label: "Body role", group: "role", context: "draw", keys: ["1"],
    description: "Every shape drawn from now on becomes a rigid body.",
  },
  {
    id: "role.cut", label: "Cut role", group: "role", context: "draw", keys: ["2"],
    description: "Every shape is subtracted from the body under it — a hole inside, a notch across the outline.",
  },
  {
    id: "role.reference", label: "Reference role", group: "role", context: "draw", keys: ["3"],
    description: "Every shape becomes construction geometry: snappable and measurable, never simulated or exported.",
  },

  // --- Shapes ---
  {
    id: "shape.polylineBody", label: "Polyline (body)", group: "shapes", context: "draw", keys: ["B"], tool: "polyline",
    description: "Draw a free shape corner by corner, in the Body role.",
  },
  {
    id: "shape.polylineCut", label: "Polyline (cut)", group: "shapes", context: "draw", keys: ["Ctrl+U"], tool: "polyline",
    description: "Draw a free shape corner by corner, in the Cut role.",
  },
  {
    id: "shape.rect", label: "Rectangle", group: "shapes", context: "draw", keys: ["Shift+B"], tool: "rect",
    description: "Two opposite corners — Shift for a square, Alt to draw from the centre.",
  },
  {
    id: "shape.circle", label: "Circle", group: "shapes", context: "draw", keys: ["C"], tool: "circle",
    description: "Centre, then the rim: a disk, a round hole or a reference circle.",
  },
  {
    id: "shape.polygon", label: "Regular polygon", group: "shapes", context: "draw", keys: ["Shift+P"], tool: "polygon",
    description: "Centre, then a corner; the side count shows beside the cursor.",
  },
  {
    id: "shape.slot", label: "Slot", group: "shapes", context: "draw", keys: ["Shift+S"], tool: "slot",
    description: "Both ends, then the width — a capsule.",
  },
  {
    id: "shape.line", label: "Line", group: "shapes", context: "draw", keys: ["L"], tool: "line",
    description: "A reference segment between two points.",
  },
  {
    id: "shape.arc", label: "Arc", group: "shapes", context: "draw", keys: ["A"], tool: "arc",
    description: "A reference arc: its two ends, then a point it passes through.",
  },
  {
    id: "shape.text", label: "Text", group: "shapes", context: "draw", keys: ["Shift+T"], tool: "text",
    description: "A label; clicked on a body it rides with that body.",
  },
  {
    id: "shape.sidesMore", label: "More sides", group: "shapes", context: "draw", keys: ["↑"],
    description: "One more side on the armed polygon tool, or on the selected regular polygon.",
  },
  {
    id: "shape.sidesFewer", label: "Fewer sides", group: "shapes", context: "draw", keys: ["↓"],
    description: "One side fewer on the armed polygon tool, or on the selected regular polygon.",
  },

  // --- Patterns ---
  {
    id: "tool.patternLinear", label: "Linear pattern", group: "pattern", context: "draw", keys: ["I"], tool: "patternLinear",
    description: "Repeat a hole or a joint along one or two directions.",
  },
  {
    id: "tool.patternCircular", label: "Circular pattern", group: "pattern", context: "draw", keys: ["Q"], tool: "patternCircular",
    description: "Repeat a hole or a joint around a centre.",
  },

  // --- Body operations ---
  {
    id: "tool.split", label: "Split", group: "boolean", context: "draw", keys: ["X"], tool: "split",
    description: "Cut a body in two along a path drawn across it.",
  },
  {
    id: "edit.combine", label: "Combine", group: "boolean", context: "draw", keys: ["N"],
    description: "Merge the selected bodies into one — they must overlap or share an edge.",
  },
  {
    id: "boolean.subtract", label: "Subtract", group: "boolean", context: "draw", tags: ["planned"],
    description: "Cut one selected body out of another.",
  },
  {
    id: "boolean.intersect", label: "Intersect", group: "boolean", context: "draw", tags: ["planned"],
    description: "Keep only the overlap of the selected bodies.",
  },

  // --- Joints & mating ---
  {
    id: "tool.joint", label: "Joint", group: "mating", context: "draw", keys: ["J"], tool: "joint",
    description: "Add a joint point to a body, where two bodies overlap, or free in space.",
  },
  {
    id: "tool.weld", label: "Weld", group: "mating", context: "draw", keys: ["W"], tool: "weld",
    description: "Lock overlapping bodies rigidly together at a point, or toggle a pin weld ↔ revolute.",
  },
  {
    id: "tool.connect", label: "Connect", group: "mating", context: "draw", keys: ["Ctrl+Shift+C"], tool: "connect",
    description: "Pin two joints together, or attach a joint to a rail as a rider.",
  },
  {
    id: "tool.ground", label: "Ground", group: "mating", context: "draw", keys: ["G"], tool: "ground",
    description: "Fix a joint's position, or a whole body; click again to release it.",
  },
  {
    id: "tool.rail", label: "Rail", group: "mating", context: "draw", tool: "rail",
    description: "Draw a track for pin-in-slot riders between two joints.",
  },
  {
    id: "tool.slider", label: "Slider", group: "mating", context: "draw", keys: ["S"], tool: "slider",
    description: "Make a body slide along an arrow without rotating.",
  },

  // --- Actuators ---
  {
    id: "tool.linearActuator", label: "Linear actuator", group: "actuator", context: "draw", keys: ["Shift+A"], tool: "linearActuator",
    description: "Make a slider or rail self-driving: its carriage travels back and forth in animation.",
  },
  {
    id: "tool.motor", label: "Motor", group: "actuator", context: "draw", keys: ["Shift+M"], tool: "motor",
    description: "Spin a crank pin about a pivot on the same body.",
  },

  // --- Transform ---
  {
    id: "edit.mirrorH", label: "Mirror left↔right", group: "transform", context: "draw",
    description: "Flip the selection horizontally.",
  },
  {
    id: "edit.mirrorV", label: "Mirror top↔bottom", group: "transform", context: "draw",
    description: "Flip the selection vertically.",
  },
  {
    id: "edit.sendBack", label: "Send to back", group: "transform", context: "draw", keys: ["PageDown"],
    description: "Put the selection behind everything else.",
  },
  {
    id: "edit.bringFront", label: "Bring to front", group: "transform", context: "draw", keys: ["PageUp"],
    description: "Put the selection in front of everything else.",
  },
  {
    id: "tool.rotate", label: "Rotate", group: "transform", context: "draw", keys: ["R"], tool: "rotate",
    description: "Drag a body to turn it about its centroid, or a node to turn it about that node.",
  },

  // --- Components ---
  {
    id: "component.create", label: "Create component", group: "component", context: "draw",
    description: "Pack the selection into a reusable definition — or fork the selected instance.",
  },
  {
    id: "component.browser", label: "Component browser", group: "component", context: "draw",
    description: "Insert, edit, rename or delete component definitions.",
  },

  // --- Constraints ---
  {
    id: "sketch.autoConstraints", label: "Auto constraints", group: "constraints", context: "draw",
    description: "Arm alignment constraints by holding a drag over a corner, joint or edge.",
  },
  {
    id: "tool.coincident", label: "Coincident", group: "constraints", context: "draw", keys: ["O"], tool: "coincident",
    description: "Make two points share a position, or hold a point on a line.",
  },
  {
    id: "tool.equal", label: "Equal length", group: "constraints", context: "draw", keys: ["E"], tool: "equal",
    description: "Give two lines the same length.",
  },
  {
    id: "tool.horizontal", label: "Horizontal", group: "constraints", context: "draw", keys: ["H"], tool: "horizontal",
    description: "Hold a line — or two points — horizontal.",
  },
  {
    id: "tool.vertical", label: "Vertical", group: "constraints", context: "draw", keys: ["V"], tool: "vertical",
    description: "Hold a line — or two points — vertical.",
  },
  {
    id: "tool.parallel", label: "Parallel", group: "constraints", context: "draw", keys: ["P"], tool: "parallel",
    description: "Hold two lines parallel.",
  },
  {
    id: "tool.perpendicular", label: "Perpendicular", group: "constraints", context: "draw", keys: ["T"], tool: "perpendicular",
    description: "Hold two lines at a right angle.",
  },
  {
    id: "tool.tangent", label: "Tangential", group: "constraints", context: "draw", keys: ["Z"], tool: "tangent",
    description: "Hold a line tangent to a circle or arc.",
  },
  {
    id: "tool.symmetric", label: "Symmetrical", group: "constraints", context: "draw", keys: ["Y"], tool: "symmetric",
    description: "Make two points, or two lines, mirror images across a third line.",
  },
  {
    id: "tool.fixed", label: "Fixed", group: "constraints", context: "draw", keys: ["F"], tool: "fixed",
    description: "Lock a point where it is, or lock a line's angle and position.",
  },
  {
    id: "sketch.badges", label: "Constraint badges", group: "constraints", context: "draw",
    description: "Show or hide every constraint badge.",
  },

  // --- Grid ---
  {
    id: "grid.show", label: "Show the grid", group: "grid", context: "any",
    description: "Show or hide the grid.",
  },
  {
    id: "grid.size", label: "Grid size", group: "grid", context: "any",
    description: "Open the grid-spacing list.",
  },
  {
    id: "grid.style", label: "Grid line style", group: "grid", context: "any",
    description: "Open the grid line-style list: solid, dashed, dotted or points.",
  },

  // --- Measure ---
  {
    id: "tool.measure", label: "Measure", group: "measure", context: "any", keys: ["D"], tool: "measure",
    description: "Dimension a line, or the distance between two references — in either mode.",
  },
  {
    id: "measure.show", label: "Show measurements", group: "measure", context: "any",
    description: "Show or hide every measurement.",
  },

  // --- Snapping ---
  {
    id: "snap.grid", label: "Snap to grid", group: "snap", context: "any",
    description: "Snap placement and dragging to the grid.",
  },
  {
    id: "snap.object", label: "Object snap", group: "snap", context: "any",
    description: "Snap onto corners, edges, midpoints and centres of other objects.",
  },

  // --- View ---
  {
    id: "view.fit", label: "Fit to screen", group: "view", context: "any", keys: ["Shift+F"],
    description: "Frame the whole mechanism.",
  },
  {
    id: "view.rotate", label: "Rotate the view", group: "view", context: "any", keys: ["Ctrl+R"],
    description: "Open or close the dial that turns the whole picture.",
  },
  {
    id: "view.rotateZero", label: "View upright", group: "view", context: "any", keys: ["0"],
    description: "Turn the view back to 0° (while the dial is open).",
  },
  {
    id: "view.zoomIn", label: "Zoom in", group: "view", context: "any",
    description: "Zoom in about the middle of the canvas.",
  },
  {
    id: "view.zoomOut", label: "Zoom out", group: "view", context: "any",
    description: "Zoom out about the middle of the canvas.",
  },
  {
    id: "view.theme", label: "Light / dark theme", group: "view", context: "any",
    description: "Switch between the light and dark themes.",
  },

  // --- Help ---
  {
    id: "help.toggle", label: "Help drawer", group: "help", context: "any", keys: ["?"],
    description: "Open or close the manual beside the canvas; while it is open, clicking a control explains it.",
  },
  {
    id: "help.contents", label: "Manual contents", group: "help", context: "any", keys: ["F1"], whileTyping: true,
    description: "Open the manual at its table of contents.",
  },
] as const satisfies readonly CommandSpec[];

/** Every command id — `main.ts` owes an action for each one. */
export type CommandId = (typeof COMMANDS)[number]["id"];

/** The same table, widened: `as const` narrows each entry to its own literal shape. */
export const COMMAND_LIST: readonly CommandSpec[] = COMMANDS;

const BY_ID = new Map<string, CommandSpec>(COMMAND_LIST.map((c) => [c.id, c]));

export function commandById(id: string): CommandSpec | undefined {
  return BY_ID.get(id);
}

/** The command ids, in table order. */
export const COMMAND_IDS: readonly string[] = COMMAND_LIST.map((c) => c.id);

/** The shortcuts a command ships with. An unreadable chord is dropped (the test catches it). */
export function defaultBindings(c: CommandSpec): Binding[] {
  const out: Binding[] = [];
  for (const chord of c.keys ?? []) {
    const b = parseChord(chord);
    if (b) out.push(b);
  }
  return out;
}

/**
 * The whole registry as a `keymap/1` file. `bindingsOf` decides which shortcuts it
 * carries: the shipped defaults for `public/keymap.json`, the effective ones (defaults
 * plus the user's overrides) for the file the Shortcuts panel exports.
 */
export function toKeymapFile(
  bindingsOf: (c: CommandSpec) => Binding[] = defaultBindings
): KeymapFile {
  return {
    schema: SCHEMA,
    app: { name: "Disjointed" },
    contexts: CONTEXTS.map((c) => ({ ...c })),
    groups: GROUPS.map((g) => ({ ...g })),
    commands: COMMAND_LIST.map((c) => ({
      id: c.id,
      label: c.label,
      description: c.description,
      group: c.group,
      context: c.context,
      bindings: bindingsOf(c),
      ...(c.fixed ? { fixed: true } : {}),
      ...(c.tags ? { tags: [...c.tags] } : {}),
      ...(c.tool ? { tool: c.tool } : {}),
      ...(c.whileTyping ? { whileTyping: true } : {}),
    })),
  };
}
