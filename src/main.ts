import "./style.css";
import { notify, NotifyKind } from "./notify";
import {
  FSDirectoryHandle,
  FSFileHandle,
  ensurePermission,
  fsDirectorySupported,
  fsSupported,
  handleFromDrop,
  listFiles,
  loadHandle,
  pickDirectory,
  pickOpenFile,
  pickSaveFile,
  storeHandle,
  writeFile,
  writeToDirectory,
} from "./filestore";
import {
  Scene,
  SceneData,
  Body,
  RoundMode,
  SelectionClip,
  FeatureClip,
  BodyGroup,
  ComponentInstance,
  ComponentOccurrence,
  InstanceTransform,
  LinearActuatorConstraint,
  MotorConstraint,
  Measurement,
  MeasureInfo,
  MeasureHighlight,
  VERTEX_LINK_EPS,
  MeasureRef,
  MeasureAxis,
  ResolvedMeasureRef,
  SketchConstraint,
  SketchConstraintKind,
  sameMeasureRef,
  refHost,
  sketchRefs,
  isMidpointHost,
  isCircleRef,
  isLineRef,
  isEqualRadiusConstraint,
  CONSTRAINT_NAME,
  measureInfoFor,
  measureAxisForPlacement,
  refCenter,
  Unit,
  UNIT_TO_MM,
  cascadeComponentChange,
  reexpandData,
  PatternSeed,
  Guide,
  MidpointHost,
  BooleanOp,
} from "./model";
import { buildContextGhost, GhostSource } from "./context";
import { parseDxf, nestLoops, loopSignedArea } from "./dxf";
import { collectCutSheet, toDxf, toSvg } from "./export";
import { solve, Driver, ConstraintBreak, SolveStats, SolveFreeze, solverConfig, resetPoseBaselines } from "./solver";
import {
  solveSketch, tryAddConstraint, autoConstrainBody, SketchBreak, AUTO_HV_TOL,
  anchorVarsForBody, anchorVarsForJoint, anchorVarsForGuide, anchorVarForGuidePoint, anchorVarForVertex,
  enforceEqualRadii, equalRadiusPartners, equalRadiusViolated,
} from "./sketch";
import { applyDimensionValue, enforcePose, placeConstraint, poseConstraintViolated } from "./pose";
import { render, RenderInput, PatternView, DARK_THEME, LIGHT_THEME, SketchGlyphView, GridStyle, GRID_STYLES, dimensionLabelHit, dimensionLabelWidth, LabelHit } from "./renderer";
import { Vec2, add, dist, sub, vec, dot, cross, lenSq, scale, rotate, normalize, perp, roundedConvexBody, filletCornerArcs, distToSegment, distToLine, distToArc, regularPolygon, arcThrough, sampleArc } from "./geometry";
import { View, MIN_SCALE, MAX_SCALE, screenToWorld, worldToScreen, zoomAt, rotateViewTo, rotateToScreen, rotateToWorld } from "./view";
import {
  COMMAND_IDS, COMMAND_LIST, CommandId, CommandSpec, SHAPE_TOOLS, ShapeRole, ShapeTool, Tool,
  commandById, defaultBindings, keymapFileError, toKeymapFile,
} from "./commands";
import {
  Binding, KeymapOverrides, findConflicts, formatChord, overridesOfFile, parseKeymap,
  parseOverrides, slotOf, slotOfEvent,
} from "./keymap";
import { installHelp } from "./help";
import { installToolbar } from "./toolbar";
import { CanvasTopic } from "./helpmap";

type Mode = "draw" | "sim";
/** `Tool`, `ShapeTool`, `SHAPE_TOOLS` and `ShapeRole` live in `commands.ts`: the command
 *  registry has to name the tool each command arms, and must stay free of the DOM. */
/** Shape tools whose product is always reference geometry (no material role). */
const REFERENCE_ONLY: ReadonlySet<string> = new Set(["line", "arc", "text"]);
/** An existing element picked in normal/select mode. */
type Selection = { kind: "body" | "joint" | "rail" | "measure" | "sketch" | "guide" | "pattern" | "tempDim"; id: number };

/** The tools that place a sketch constraint (tool name = constraint kind). */
const CONSTRAINT_TOOLS = new Set<Tool>([
  "coincident", "horizontal", "vertical", "parallel", "perpendicular", "equal", "fixed", "symmetric", "tangent",
]);

/**
 * The user's own keymap, if they have one: the whole `{commandId: bindings[]}` map (see
 * the Shortcuts panel near the key handler). It lives up here because titles are
 * generated during start-up, long before that block runs.
 */
const KEYMAP_KEY = "disjointed:keymap";
let keymapOverrides: KeymapOverrides = (() => {
  try {
    return parseOverrides(localStorage.getItem(KEYMAP_KEY)) ?? {};
  } catch {
    return {};
  }
})();
/** slot → the commands bound to it, in registry order (built by `rebuildKeymap`). */
let commandSlots = new Map<string, CommandSpec[]>();

/** Pick / close thresholds in screen (CSS) pixels — converted to world units via the view. */
const PICK_RADIUS = 12;
const CLOSE_RADIUS = 12;
/** Smallest outward margin (world units) when expanding a body built from joints. */
const JOINT_BODY_MIN_MARGIN = 4;
/** How much the [ and ] keys change a selected body's corner radius, per press. */
const RADIUS_STEP = 4;
/** Rotate snaps to a 45° multiple when the body's angle is within this of one (≈2°). */
const ROTATE_SNAP_TOL = (2 * Math.PI) / 180;

const canvas = document.getElementById("scene") as HTMLCanvasElement;
const ctx = canvas.getContext("2d")!;
const hintEl = document.getElementById("hint")!;
const simErrorEl = document.getElementById("sim-error")!;
const gridBtn = document.getElementById("grid-btn") as HTMLButtonElement;
const snapBtn = document.getElementById("snap-btn") as HTMLButtonElement;
const osnapBtn = document.getElementById("osnap-btn") as HTMLButtonElement;
const autoConBtn = document.getElementById("autocon-btn") as HTMLButtonElement;
const gridSizeBtn = document.getElementById("grid-size-btn") as HTMLButtonElement;
const gridSizeValue = document.getElementById("grid-size-value") as HTMLSpanElement;
const gridSizeMenu = document.getElementById("grid-size-menu") as HTMLDivElement;
const gridSizeList = document.getElementById("grid-size-list") as HTMLDivElement;
const gridSizeAddForm = document.getElementById("grid-size-add") as HTMLFormElement;
const gridSizeNew = document.getElementById("grid-size-new") as HTMLInputElement;
const gridColorInput = document.getElementById("grid-color") as HTMLInputElement;
const gridStyleBtn = document.getElementById("grid-style-btn") as HTMLButtonElement;
const gridStyleMenu = document.getElementById("grid-style-menu") as HTMLDivElement;
const gridStyleList = document.getElementById("grid-style-list") as HTMLDivElement;
const themeBtn = document.getElementById("theme-btn") as HTMLButtonElement;
const modeToggle = document.getElementById("mode-toggle") as HTMLButtonElement;
const modeCap = document.getElementById("mode-cap")!;
/** Toolbar sections that only make sense in one mode (class on the section element). */
const drawOnlySections = [...document.querySelectorAll<HTMLElement>(".tb-sec.draw-only")];
const simOnlySections = [...document.querySelectorAll<HTMLElement>(".tb-sec.sim-only")];
const colorInput = document.getElementById("body-color") as HTMLInputElement;
const actuatorProps = document.getElementById("actuator-props")!;
const motorProps = document.getElementById("motor-props")!;
const actuatorSpeedInput = document.getElementById("actuator-speed") as HTMLInputElement;
const motorSpeedInput = document.getElementById("motor-speed") as HTMLInputElement;
const profileToggle = document.getElementById("actuator-profile")!;
const runBtn = document.getElementById("run-btn") as HTMLButtonElement;
const autopauseBtn = document.getElementById("autopause-btn") as HTMLButtonElement;
const animIterCtrl = document.getElementById("anim-iter-ctrl")!;
const animIterInput = document.getElementById("anim-iter") as HTMLInputElement;
const animIterValue = document.getElementById("anim-iter-value")!;
const cleanupMaxCtrl = document.getElementById("cleanup-max-ctrl")!;
const cleanupMaxInput = document.getElementById("cleanup-max") as HTMLInputElement;
const cleanupMaxValue = document.getElementById("cleanup-max-value")!;
const structTolCtrl = document.getElementById("struct-tol-ctrl")!;
const structTolInput = document.getElementById("struct-tol") as HTMLInputElement;
const breakTolCtrl = document.getElementById("break-tol-ctrl")!;
const breakTolInput = document.getElementById("break-tol") as HTMLInputElement;
const dimEditInput = document.getElementById("dim-edit") as HTMLInputElement;
const sketchVisBtn = document.getElementById("sketch-vis-btn") as HTMLButtonElement;
const measureVisBtn = document.getElementById("measure-vis-btn") as HTMLButtonElement;
const unitSelect = document.getElementById("unit-select") as HTMLSelectElement;
const makeCompBtn = document.getElementById("make-comp-btn") as HTMLButtonElement;
const compPanelBtn = document.getElementById("comp-panel-btn") as HTMLButtonElement;
const compPanel = document.getElementById("comp-panel")!;
const compList = document.getElementById("comp-list")!;
const crumbBar = document.getElementById("crumb-bar")!;

const scene = new Scene();

// --- component editing context ---------------------------------------------
/**
 * Component-editing context stack (def ids, outermost first). Empty = editing the root
 * assembly. The live `scene` always holds the innermost context; `rootData` keeps the
 * root context's snapshot while a definition is open (refreshed by every markDirty, so
 * saves/undo always see a consistent document), and ancestor definitions' data lives in
 * `scene.components` (kept fresh by the same cascade).
 */
let editPath: number[] = [];
let rootData: SceneData | null = null;
/**
 * Context ghost (see context.ts): while a definition is open, the enclosing assembly
 * can be shown faded in the definition's own frame. `viaStack[k]` is the instance in
 * context k (0 = root) through which definition `editPath[k]` was entered — null when
 * entered from the component browser with no instance to place by (the placement chain
 * breaks there). `ghostDepth` = how many enclosing levels are shown, counted outward
 * from the immediate parent (0 = off; Infinity = the whole assembly); the breadcrumb
 * eyes set it. `ghostLevels` is the built ghost, aligned with the contexts (rebuilt
 * lazily after any change — `ghostDirty`), and `ghostTargets` caches its object-snap
 * features (refs stripped: snapping onto the ghost never mints a constraint).
 */
let viaStack: (number | null)[] = [];
let ghostDepth = 0;
let ghostLevels: (Scene | null)[] = [];
let ghostDirty = true;
let ghostTargets: { points: SnapPoint[]; lines: SnapLine[] } | null = null;
/** A measure reference into one level of the context ghost (never persisted). */
type GhostRef = { kind: "ghost"; level: number; ref: MeasureRef };
/** A reference the measure tool can pick: live geometry, or a ghost feature. */
type TempRef = MeasureRef | GhostRef;
/**
 * A temporary context dimension: at least one end on the ghost. Read-only in the CAD
 * sense (always driven), never saved, lives only while its definition level is open.
 * Typing a value performs a one-shot move of the live side (see applyTempDimValue).
 * Ids are negative so they never collide with the scene's measurement ids.
 */
interface TempDim {
  id: number;
  refA: TempRef;
  refB: TempRef;
  labelOffset: Vec2;
  axis: MeasureAxis;
}
/** Temporary dimensions per open definition level (`tempDims[editPath.length - 1]`). */
let tempDims: TempDim[][] = [];
let tempDimSeq = -1;
/** Camera saved per context depth, restored when exiting back to it. */
const savedViews: View[] = [];
/** One-shot pending placement: the next canvas click drops an instance of this def. */
let pendingInsert: number | null = null;

// --- working units ---------------------------------------------------------
// Declarative only: 1 world unit = 1 <unit>. Nothing moves when it changes — the
// grid, measurements and DXF import just interpret world units through it. Part of
// the scene data (saved / autosaved / undoable), so the swatch re-syncs each frame.
unitSelect.addEventListener("change", () => {
  scene.unit = unitSelect.value as Unit;
  markDirty();
});
function syncUnitSelect(): void {
  if (unitSelect.value !== scene.unit) unitSelect.value = scene.unit;
}

// --- theme (light/dark) --------------------------------------------------
// Chrome is themed via a `data-theme` attribute on <html> (CSS vars); the canvas reads the
// matching palette below. Preference persists across sessions (separate from scene autosave).
const THEME_KEY = "disjointed:theme";
let theme: "dark" | "light" =
  localStorage.getItem(THEME_KEY) === "light" ? "light" : "dark";
function applyTheme(): void {
  document.documentElement.dataset.theme = theme;
}
function setTheme(next: "dark" | "light"): void {
  theme = next;
  localStorage.setItem(THEME_KEY, theme);
  applyTheme();
  syncGridLook(); // the grid colour is per theme: show the one now in force
}
function toggleTheme(): void {
  setTheme(theme === "dark" ? "light" : "dark");
}
applyTheme();
themeBtn.addEventListener("click", toggleTheme);

// --- body colour ---------------------------------------------------------
// The toolbar colour input does double duty: with bodies selected it recolours them (the
// single selection, or every body of a multi-selection / group at once); with nothing
// selected it sets the colour applied to newly drawn bodies. The swatch is kept in sync
// with the current selection by `syncColorPicker` (called each frame).
let defaultBodyColor = colorInput.value;
/**
 * The bodies the colour picker acts on, and how many of the selected ones it had to
 * leave alone. Component-instance material is skipped: its colour is copied from the
 * definition on every re-expansion, so recolouring it here would silently revert.
 */
function colorTargets(): { ids: number[]; skipped: number } {
  const sel = multiSel
    ? [...multiSel.bodies]
    : selection?.kind === "body"
      ? [selection.id]
      : [];
  const ids = sel.filter((id) => scene.getBody(id) && !scene.instanceOfBody(id));
  return { ids, skipped: sel.length - ids.length };
}
/** Selection signature — the instance warning is shown once per selection, not per event. */
let colorWarnKey = "";
colorInput.addEventListener("input", () => {
  const c = colorInput.value;
  const { ids, skipped } = colorTargets();
  if (ids.length === 0 && skipped === 0) {
    defaultBodyColor = c;
    return;
  }
  for (const id of ids) scene.getBody(id)!.color = c;
  if (skipped > 0) {
    const key = `${ids.join(",")}/${skipped}`;
    if (key !== colorWarnKey) {
      colorWarnKey = key;
      notify(
        ids.length === 0
          ? "Component instances take their colour from their definition — open the definition to recolour them."
          : `Recoloured ${ids.length} ${ids.length === 1 ? "body" : "bodies"}; ${skipped} belong to component instances and keep the definition's colour.`,
        ids.length === 0 ? "warn" : "info"
      );
    }
  }
  if (ids.length > 0) markDirty();
});
/**
 * Reflect the selection's colour (or the new-body default) in the swatch. A mixed
 * multi-selection has no one colour to show, so the first member's stands in — dragging
 * the picker then makes the whole selection that colour, which is what the swatch shows.
 */
let colorSyncKey = "";
function syncColorPicker(): void {
  const { ids } = colorTargets();
  const cols = ids.map((id) => scene.getBody(id)!.color);
  const key = cols.length === 0 ? `d:${defaultBodyColor}` : `b${ids.join(",")}:${cols.join(",")}`;
  if (key === colorSyncKey) return; // avoid clobbering the picker mid-drag
  colorSyncKey = key;
  colorInput.value = cols.length === 0 ? defaultBodyColor : cols[0];
}

// --- interaction state ---------------------------------------------------
let mode: Mode = "draw";
/** Armed draw tool, or null for normal/select mode. Tools disarm after one use. */
let tool: Tool | null = null;
let draftBody: Vec2[] = []; // polyline tool: freehand vertices placed so far
/**
 * Per freehand draft vertex: the existing point (joint / body corner) the click landed
 * on, or null. On finish, each recorded pick becomes a coincident auto-constraint
 * between the new body's corner and that point (the vertex is placed exactly on it).
 */
let draftBodySnaps: (MeasureRef | null)[] = [];
// --- shape tools ----------------------------------------------------------------
/** The sticky role every finished shape takes (toolbar switch / 1 / 2 / 3). */
let shapeRole: ShapeRole = "body";
/** One-shot role for the shape being drawn (Ctrl on its first click flips Body ↔ Cut). */
let roleOverride: ShapeRole | null = null;
/** Point-defined shape tools (rect / circle / polygon / slot / line / arc): the points placed so far + their picks. */
let shapePts: Vec2[] = [];
let shapeSnaps: (MeasureRef | null)[] = [];
/** Cut role: the body the shape will be subtracted from (picked at the first click), or null until known. */
let shapeTarget: number | null = null;
/**
 * Two-point shape tools, press-and-drag gesture: the press that may become a drag
 * (screen point pressed, where the first point would land, its pick). Once the pointer
 * has clearly moved the press is the first point and the release the second; a press
 * released in place just places the first point (the click flow).
 */
let shapePress: { screen: Vec2; at: Vec2; pick: MeasureRef | null; ctrl: boolean; dragging: boolean } | null = null;
/** Modifier keys as of the last mouse event (Shift squares a rectangle, Alt draws it from the centre, Ctrl flips the role). */
let mods = { shift: false, alt: false, ctrl: false };
/** Regular polygon tool: side count (↑ / ↓ while the tool is armed; shown beside the cursor badge). */
let polySides = 6;
/** Text tool: label height in world units (toolbar field). */
let textSize = 10;
const shapePropsGroup = document.getElementById("shape-props") as HTMLDivElement;
const textSizeLabel = document.getElementById("text-size-label") as HTMLLabelElement;
const textSizeInput = document.getElementById("text-size") as HTMLInputElement;
// --- pattern tools ------------------------------------------------------------
/** What the armed pattern tool is replicating: the picked features (holes / attached joints of one body), in pick order — the first is the layout's anchor. */
let patternSeeds: PatternSeed[] = [];
/** Linear tool: the row just created, still armed for an optional second direction (a grid). */
let patternDraft: number | null = null;
/** Screen-pixel travel before a shape-tool press counts as a drag (vs a click). */
const SHAPE_DRAG_PX = 4;
/** Split tool: the cut path so far (first point on the body's outline) and the body being cut. */
let splitDraft: Vec2[] = [];
let splitBodyId: number | null = null;
let jointDraftIds: number[] = []; // joints picked to build a body (body tool, joint start)
let jointDraftCreated: number[] = []; // joints made on rails during that draft (removed if aborted)
let jointDraftExpanding = false; // body-from-joints: sizing the outward margin
let cursor: Vec2 | null = null; // world coordinates
let hoverJoint: number | null = null;
let hoverBody: number | null = null; // body under the cursor in normal mode
/** Object snap on, select mode: the reference a drag started at the cursor would use (preview). */
let hoverObjSnap: ResolvedMeasureRef | null = null;
let hoverDef: number | null = null; // component definition hovered in the browser list
let selectedJoint: number | null = null; // first pick for connect
let railDraftIds: number[] = []; // rail joints picked so far for the rail tool (0–2)
/** Slider tool (v20): the owner body + start point of the travel picked by the first click
 *  (`riderId` when the click landed on one of the owner's existing joints — it becomes the rider). */
let sliderDraft: { bodyId: number; at: Vec2; riderId: number | null } | null = null;
let selection: Selection | null = null; // element selected in normal mode
/**
 * Draw-mode multi-selection (Ctrl+click toggles; box select replaces/extends): bodies and
 * free joints that move together while selected. Permanent groups are selection-atomic —
 * touching any member selects all of them. Mutually exclusive with `selection`.
 */
let multiSel: { bodies: Set<number>; joints: Set<number> } | null = null;
/** In-progress box selection (world corners); `additive` = started with Ctrl/Cmd held. */
let boxSelect: { start: Vec2; end: Vec2; additive: boolean; moved: boolean } | null = null;
let driver: Driver | null = null;
/** Constraints the last solve couldn't satisfy (impossible assembly); drives the red overlay + banner. */
let solveBreaks: ConstraintBreak[] = [];
/** Body poses saved when entering simulation, restored when leaving. */
let savedPoses: Map<number, { pos: Vec2; angle: number }> | null = null;
/** Last selection copied with Ctrl+C: plain material as a `SelectionClip`, plus any
 *  copied component instances as placements (pasting creates new instances of the same
 *  definitions); pasted at the cursor with Ctrl+V. */
let clipboard:
  | {
      kind: "selection";
      clip: SelectionClip | null;
      instances: { defId: number; t: InstanceTransform }[];
      center: Vec2;
    }
  // Features of one body (whole holes + joints, with what's internal to them), pasted
  // into the selected body with Ctrl+V.
  | { kind: "features"; clip: FeatureClip }
  | null = null;
/**
 * Feature selection within the singly-selected body (draw mode): control vertices —
 * outer outline (`hole` null) or a hole's — plus attached joints, all of `bodyId`. Pattern
 * members resolve to their seed's matching feature (members are derived geometry), so the
 * set names seeds only. Built by a Shift+drag box from empty space (Ctrl+Shift extends);
 * members move together (drag any of them), delete together, and copy as a `FeatureClip`.
 */
let featureSel: { bodyId: number; verts: { hole: number | null; index: number }[]; joints: number[] } | null = null;
/** In-progress feature box (world corners); `additive` = Ctrl+Shift (extends the set). */
let featureBox: { start: Vec2; end: Vec2; additive: boolean; moved: boolean } | null = null;
/**
 * Active rotate (rotate tool): turning `bodyIds` (plus any multi-selected free `jointIds`)
 * about a fixed `pivot`. `grabAngle` is the first body's angle at grab; `prevPointer` /
 * `accum` track the pointer's accumulated swing about the pivot (unwrapped); `lastTotal`
 * is the rotation applied so far (lets us apply only the incremental delta each move
 * while snapping the absolute angle to 45°).
 */
type RotateDrag = {
  bodyIds: number[];
  jointIds: number[];
  pivot: Vec2;
  grabAngle: number;
  prevPointer: number;
  accum: number;
  lastTotal: number;
  moved: boolean;
};
let rotateDrag: RotateDrag | null = null;
/** Motor tool: first click picks the pivot joint, second the crank pin on the same body. */
let motorPivotDraft: number | null = null;
/** Measure tool: the references picked so far (0–2); the third click places the label. */
let measurePicks: TempRef[] = [];
/** How close (screen px) a click must land to a measurement's value label to pick it. */
const LABEL_PICK_RADIUS = 16;
/** Constraint tools: the reference(s) picked so far (0–1; the finishing pick commits). */
let constraintPicks: MeasureRef[] = [];
/** Rejected sketch edit: the conflicting item ids flash red until `until` (ms clock). */
let sketchFlash: { ids: Set<number>; until: number } | null = null;
/** Long enough to find the flashing items after reading the toast that names them —
 *  the flash is what points at *which* constraint / dimension is in the way. */
const SKETCH_FLASH_MS = 4000;
/** Last frame's computed constraint badges (world positions) — reused for hit-testing. */
let sketchGlyphCache: SketchGlyphView[] = [];
/** How close (screen px) a click must land to a constraint badge to pick it. */
const GLYPH_PICK_RADIUS = 10;
/** Measurement being edited in the inline dimension-value input, or null. */
let dimEditId: number | null = null;
/** The temporary context dimension the value editor is open on (one-shot move), or null. */
let dimEditTemp: number | null = null;
/**
 * Visibility toggles (session-only, like the grid): hiding is purely visual — hidden
 * constraints still solve, hidden measurements still exist — but the hidden layer isn't
 * hit-testable, so it can't be clicked, dragged, or edited until shown again.
 */
let sketchVisible = true;
let measureVisible = true;

function setSketchVisible(on: boolean): void {
  sketchVisible = on;
  sketchVisBtn.classList.toggle("active", on);
  if (!on && selection?.kind === "sketch") selection = null;
}

function setMeasureVisible(on: boolean): void {
  measureVisible = on;
  measureVisBtn.classList.toggle("active", on);
  if (!on) {
    closeDimEditor();
    if (selection?.kind === "measure" || selection?.kind === "tempDim") selection = null;
  }
}

// --- animation (actuators / motors) -------------------------------------
/**
 * Animation state. Driven by the Run-animation toggle in sim mode. While `running`,
 * each frame advances `phaseAccum` for every actuator/motor by `speed * dt`, computes a
 * target world position for the driven joint(s), and the solver pulls everything else
 * onto those targets (they're passed as `anchors` to `solve`). When paused, no phases
 * advance and the scene drives by mouse only. `phaseAccum` carries cycles for linear
 * actuators and radians for motors; pressing play refits each phase to the joint's
 * current state so the motion picks up smoothly from wherever the user left it.
 */
let animating = false;
let animLastTimestamp: number | null = null;
const animPhase = new Map<number, number>(); // constraint id → phase accumulator
// Auto-pause: when on, the animation halts after the assembly reports breaks for a few
// consecutive frames. Session-only (not persisted), only affects the animation tick.
// A short debounce filters solver chatter — complex closed loops can occasionally miss
// convergence in a single 60-iteration frame even when geometrically solvable.
let pauseOnImpossible = false;
let impossibleFrames = 0;
const IMPOSSIBLE_PAUSE_FRAMES = 3;
// Phase-A sweep count used by the animation tick. Tunable live via the toolbar slider so the
// trade-off between convergence (higher = more accurate, fewer spurious breaks) and per-frame
// cost (lower = cheaper) can be felt on the actual mechanism.
let animIterations = 100;
// Rolling stats for the per-frame animation solve, surfaced in the debug log. Reset on stop.
let animSolveMin = Infinity;
let animSolveMax = 0;
let animSolveSum = 0;
let animSolveCount = 0;
let animErrorFrames = 0; // frames whose solve reported at least one break
let animCleanupSum = 0;
let animCleanupMax = 0;
let animPhaseASum = 0;
let animPhaseAMax = 0;
let animResidualSum = 0;
let animResidualMax = 0;

// --- grid appearance -------------------------------------------------------
// Colour and line style of the world grid. A display preference like the theme, so it
// lives in localStorage rather than in the document — nothing about it belongs to the
// mechanism. The colour is kept **per theme**: a tone that reads against the dark
// background disappears against the light one, so the swatch always edits the theme you
// are in and switching themes brings that theme's grid back.
const GRID_LOOK_KEY = "disjointed:gridLook";
interface GridLook {
  /** Per-theme colour override; null = that theme's own grid tone. */
  color: { dark: string | null; light: string | null };
  style: GridStyle;
}
function loadGridLook(): GridLook {
  const hex = (c: unknown): string | null =>
    typeof c === "string" && /^#[0-9a-f]{6}$/i.test(c) ? c : null;
  try {
    const raw = localStorage.getItem(GRID_LOOK_KEY);
    const v = raw ? (JSON.parse(raw) as Partial<GridLook>) : null;
    return {
      color: { dark: hex(v?.color?.dark), light: hex(v?.color?.light) },
      style: GRID_STYLES.includes(v?.style as GridStyle) ? (v!.style as GridStyle) : "solid",
    };
  } catch {
    return { color: { dark: null, light: null }, style: "solid" };
  }
}
let gridLook: GridLook = loadGridLook();
function saveGridLook(): void {
  try {
    localStorage.setItem(GRID_LOOK_KEY, JSON.stringify(gridLook));
  } catch {
    /* storage unavailable (private mode): the setting just doesn't outlive the session */
  }
}
/** The current theme's own grid tone — what the swatch shows while there is no override. */
const themeGridColor = (): string => (theme === "light" ? LIGHT_THEME : DARK_THEME).grid;
/** The grid colour in force, or undefined to leave it to the theme (see RenderInput). */
const gridColorOverride = (): string | undefined => gridLook.color[theme] ?? undefined;
/** Put the current theme's colour and the chosen style back into the two controls. */
function syncGridLook(): void {
  gridColorInput.value = gridLook.color[theme] ?? themeGridColor();
  // The style picker is a picture, not a word: the gs-* class on the button draws the
  // sample (see style.css), and the matching row is marked selected.
  for (const s of GRID_STYLES) gridStyleBtn.classList.toggle(`gs-${s}`, s === gridLook.style);
  for (const item of gridStyleList.querySelectorAll<HTMLElement>(".combo-item")) {
    const on = item.querySelector<HTMLElement>(".gs-opt")?.dataset.style === gridLook.style;
    item.classList.toggle("selected", on);
    item.setAttribute("aria-selected", String(on));
  }
}
gridColorInput.addEventListener("input", () => {
  gridLook.color[theme] = gridColorInput.value;
  saveGridLook();
});
// Right-click restores the theme default: a colour input can't express "no override" by
// itself, and a double-click can't be used for it — the first click already opens the
// browser's colour dialog.
gridColorInput.addEventListener("contextmenu", (e) => {
  e.preventDefault();
  if (gridLook.color[theme] === null) return;
  gridLook.color[theme] = null;
  saveGridLook();
  syncGridLook();
  notify("Grid colour back to the theme default.", "info");
});
// The style picker is a combo like the grid size's: a button showing the current sample,
// dropping a list of the four samples (no text — the picture is the label, its name is on
// the tooltip).
function openGridStyleMenu(): void {
  closeGridSizeMenu(); // the two combos sit side by side: only one is ever open
  gridStyleMenu.classList.remove("hidden");
  gridStyleBtn.setAttribute("aria-expanded", "true");
  gridStyleBtn.classList.add("active");
  // Focus the current style's row: Tab then walks the samples, and Escape (handled on the
  // menu) reaches it — the size combo gets this for free from its Custom… field.
  gridStyleList.querySelector<HTMLButtonElement>(".combo-item.selected .gs-opt")?.focus();
}
function closeGridStyleMenu(): void {
  if (gridStyleMenu.classList.contains("hidden")) return;
  gridStyleMenu.classList.add("hidden");
  gridStyleBtn.setAttribute("aria-expanded", "false");
  gridStyleBtn.classList.remove("active");
}
function toggleGridStyleMenu(): void {
  if (gridStyleMenu.classList.contains("hidden")) openGridStyleMenu();
  else closeGridStyleMenu();
}
gridStyleBtn.addEventListener("click", toggleGridStyleMenu);
for (const opt of gridStyleList.querySelectorAll<HTMLButtonElement>(".gs-opt")) {
  opt.addEventListener("click", () => {
    const v = opt.dataset.style as GridStyle;
    if (GRID_STYLES.includes(v)) {
      gridLook.style = v;
      saveGridLook();
      syncGridLook();
    }
    closeGridStyleMenu();
    gridStyleBtn.focus();
  });
}
gridStyleMenu.addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    e.stopPropagation();
    closeGridStyleMenu();
    gridStyleBtn.focus();
  }
});
// Click anywhere outside the combo dismisses it.
document.addEventListener("pointerdown", (e) => {
  if (gridStyleMenu.classList.contains("hidden")) return;
  const t = e.target;
  if (t instanceof Node && (gridStyleMenu.contains(t) || gridStyleBtn.contains(t))) return;
  closeGridStyleMenu();
});
syncGridLook();

// --- grid / snapping -------------------------------------------------------
/** Grid spacing (and snap increment) in world units; mirrors the renderer's grid. */
let gridStep = 40;
/** When true, placements and drags land on the nearest grid intersection. */
let snapEnabled = false;
/** When true, the world-locked grid is drawn (snapping still works when hidden). */
let gridVisible = true;
/**
 * Object snap (draw-mode drags): when true, a body / joint / multi-selection drag picks a
 * reference feature of what's grabbed — a control vertex, an edge midpoint, a control edge,
 * or (default) the object's centre — and that feature snaps onto the same features of the
 * other objects (plus rails and reference segments, with their midpoints). A point target
 * always wins over a line it sits on (a midpoint over its own edge). Takes precedence over
 * the grid/guide snap.
 */
let objSnapEnabled = false;
/** Screen-px capture range for object snapping (dragged reference onto a target feature). */
const OBJ_SNAP_PX = 12;
/** A line reference only snaps onto (near-)parallel lines: within this angle (≈2°). */
const OBJ_SNAP_PARALLEL_TOL = (2 * Math.PI) / 180;

/**
 * Implicit constraints while dragging (draw mode; any body / joint / vertex / multi drag
 * whose reference feature can take a sketch constraint — a joint, a control vertex, a
 * regular polygon's centre, an edge midpoint, a guide point or a control edge): holding
 * the dragged reference over another point / line — a line's midpoint included — for
 * `ALIGN_HOVER_MS` arms that element as an *alignment candidate*. Up to
 * `ALIGN_MAX_CANDS` stay armed at once — arming one more drops the oldest, Esc drops the
 * newest — so a single drag can pick up two alignments at different references (vertical
 * to one, horizontal to another). Releasing the drag with the reference H/V-aligned with
 * a candidate point, or on the infinite line of a candidate line, within `ALIGN_TOL_PX`
 * creates the matching sketch constraints automatically — a dotted line with the
 * constraint's badge previews each of them during the drag.
 *
 * A dragged point released **on** a candidate point (within `ALIGN_TOL_PX` on both axes)
 * takes a coincident instead of an axis alignment — the drop says "these two are the same
 * point", which is what the candidate was armed for.
 *
 * Two matches only preview together when their exact corrections are **independent**: the
 * drag has two degrees of freedom, each alignment spends one along its own normal (a
 * point-on-point coincident spends both), and two that pull the same way can't both be
 * met. A redundant second one would leave a gap for the solver to close, which is exactly
 * what the pre-placement correction exists to avoid; on such a clash the newer candidate's
 * match wins and the older candidate stays armed but silent.
 */
const ALIGN_HOVER_MS = 400;
const ALIGN_TOL_PX = 10;
/** How many alignment candidates one drag keeps armed at the same time. */
const ALIGN_MAX_CANDS = 2;
/** Two corrections count as independent when their unit normals differ by ≥ this (≈3°). */
const ALIGN_INDEPENDENT_TOL = Math.sin((3 * Math.PI) / 180);
/**
 * Whether implicit constraints are offered at all — the switch in the Constraints group.
 * Off, a drag carries no alignment state: nothing is scanned, previewed or placed (the
 * auto-constraints applied while *drawing* are a separate thing and stay on). Session
 * state, like the snap toggles.
 */
let autoConstrain = true;

function setAutoConstrain(on: boolean): void {
  autoConstrain = on;
  autoConBtn.classList.toggle("active", on);
  autoConBtn.setAttribute("aria-pressed", String(on));
}

/** Screen-px capture range for snapping onto reference geometry. */
const GUIDE_SNAP_PX = 10;

/**
 * Snap a world point (identity when snap is off). Reference geometry takes precedence
 * over the grid: within capture range of one reference edge the point projects onto it
 * (within its span); within range of two, it lands on their intersection; near a
 * reference circle / arc it lands on the rim. Away from any reference it snaps to the
 * nearest grid intersection. `excludeGuide` leaves one guide out, so dragging a guide
 * never snaps it onto itself.
 */
function snap(p: Vec2, excludeGuide?: number): Vec2 {
  if (!snapEnabled) return p;
  const r = GUIDE_SNAP_PX / view.scale;
  const near: { o: Vec2; d: Vec2; dist: number; proj: Vec2 }[] = [];
  const curved: { dist: number; proj: Vec2 }[] = [];
  for (const g of scene.guides) {
    if (g.id === excludeGuide) continue;
    // Polyline edges: project onto the edge, within its span.
    for (const l of scene.guideLines(g)) {
      const d = normalize(sub(l.b, l.a));
      if (d.x === 0 && d.y === 0) continue;
      const t = dot(sub(p, l.a), d);
      if (t < 0 || t > dist(l.a, l.b)) continue;
      const proj = add(l.a, scale(d, t));
      const dd = dist(p, proj);
      if (dd <= r) near.push({ o: l.a, d, dist: dd, proj });
    }
    // Circles and arcs: project radially onto the rim (an arc only along its sweep).
    if (g.kind === "circle" || g.kind === "arc") {
      const arc = g.kind === "circle" ? { c: g.c, r: g.r, a0: 0, sweep: Math.PI * 2 } : scene.guideArc(g);
      if (!arc) continue;
      const dc = dist(p, arc.c);
      if (dc < 1e-9 || Math.abs(dc - arc.r) > r) continue;
      const proj = add(arc.c, scale(normalize(sub(p, arc.c)), arc.r));
      if (g.kind === "arc" && distToArc(proj, arc) > 1e-6) continue;
      curved.push({ dist: Math.abs(dc - arc.r), proj });
    }
  }
  if (near.length > 0) {
    near.sort((x, y) => x.dist - y.dist);
    // Two (non-parallel) reference edges in range: land exactly on their intersection.
    for (let i = 1; i < near.length; i++) {
      const den = cross(near[0].d, near[i].d);
      if (Math.abs(den) < 1e-6) continue; // (near-)parallel — no usable intersection
      const t = cross(sub(near[i].o, near[0].o), near[i].d) / den;
      const q = add(near[0].o, scale(near[0].d, t));
      if (dist(p, q) <= r) return q;
    }
    return near[0].proj;
  }
  if (curved.length > 0) {
    curved.sort((x, y) => x.dist - y.dist);
    return curved[0].proj;
  }
  return vec(Math.round(p.x / gridStep) * gridStep, Math.round(p.y / gridStep) * gridStep);
}

// --- camera ---------------------------------------------------------------
const view: View = { scale: 1, tx: 0, ty: 0, angle: 0 };
/**
 * View-rotation dial (Shift+R / toolbar, both modes): while open, a big ticked ring with
 * a crosshair along the world axes sits over the canvas; dragging the ring or an arm
 * turns the whole picture about the screen centre (snapping to 5° unless Shift is held),
 * the input under the centre takes an exact angle, double-clicking the centre resets to
 * 0°. `drag` holds the pointer's bearing and the view angle at the grab (the turn is the
 * bearing's change, so nothing jumps on press). `hot`: the pointer is over a grab handle.
 */
let viewRotate: { drag: { startBearing: number; startAngle: number } | null; hot: boolean } | null = null;
const viewAngleInput = document.getElementById("view-angle-edit") as HTMLInputElement;
const rotateViewBtn = document.getElementById("rotate-view-btn") as HTMLButtonElement;
/** Active right-button view pan. */
let pan: { lastScreen: Vec2 } | null = null;
/**
 * Active left-button drag of a selected element in draw/select mode. `grabOffset` is
 * the cursor-minus-anchor offset captured at grab time, so the dragged anchor can be
 * snapped to the grid in absolute terms. For a whole-body move the anchor is whichever
 * of the centroid / control vertices was closest to the grab point, stored as a fixed
 * `anchorOffset` from the centroid (a plain move only translates, so it stays constant).
 */
/**
 * Object-snap state carried by a body / joint / multi drag: `ref` is the feature of the
 * dragged object that snaps (a control vertex, an edge `midpoint`, the centre as a
 * `bodyPoint`, a control edge, or a joint), re-resolved live each frame — the drag anchor
 * is its point (a line's midpoint). `hit` is what it last snapped onto (for the
 * highlight), drawn as an infinite line when `hitInfinite`.
 */
type DragObjSnap = { ref: MeasureRef; hit: ResolvedMeasureRef | null; hitInfinite: boolean };

/**
 * Implicit-constraint state carried by a body / joint / vertex / multi drag: `ref` is the
 * dragged reference (the feature object snap would use, when it can take a constraint —
 * independent of the object-snap toggle), `hover` the target it currently sits on and
 * since when, `cands` the armed candidates (oldest first, at most `ALIGN_MAX_CANDS`) and
 * `matches` the constraints a release right now would create — at most one per candidate,
 * in candidate order. `slip` is the last snap correction (unsnapped − snapped anchor), so a
 * hover is judged where the cursor put the reference, not where the grid moved it.
 */
type DragAlign = {
  ref: MeasureRef;
  hover: { ref: MeasureRef; since: number } | null;
  cands: MeasureRef[];
  matches: AlignMatch[];
  slip: Vec2;
};
/** A previewed implicit constraint: its kind, the candidate it holds against and the
 *  dotted preview line's endpoints. */
type AlignMatch = { kind: SketchConstraintKind; cand: MeasureRef; from: Vec2; to: Vec2 };

/** A drag anchor spec: a fixed offset from a body's centroid, or a joint. */
type DragAnchorSpec = { bodyId: number; offset: Vec2 } | { jointId: number };

type LeftDrag =
  | { kind: "body"; id: number; anchorOffset: Vec2; grabOffset: Vec2; moved: boolean; osnap?: DragObjSnap; align?: DragAlign }
  | { kind: "joint"; id: number; grabOffset: Vec2; moved: boolean; osnap?: DragObjSnap; align?: DragAlign }
  // `hole` scopes a vertex/fillet drag to one of the body's holes (null = the outer outline).
  // With object snap on, the dragged vertex is its own snapping reference (`osnap`).
  | { kind: "vertex"; bodyId: number; index: number; hole: number | null; grabOffset: Vec2; moved: boolean; osnap?: DragObjSnap; align?: DragAlign }
  // Per-corner radius handle: the cursor's position maps straight to that corner's radius.
  | { kind: "fillet"; bodyId: number; index: number; hole: number | null; moved: boolean }
  // `temp` marks a temporary context dimension's label (main-side list, not the scene).
  | { kind: "measureLabel"; id: number; grabOffset: Vec2; moved: boolean; temp?: boolean }
  // A pattern's handle: an axis end (re-aims + re-spaces that direction) or the circular centre.
  | { kind: "patternHandle"; id: number; axis: number | "centre"; moved: boolean }
  // Whole-guideline move (angle preserved; anchored on its point `a`)…
  | { kind: "guide"; id: number; grabOffset: Vec2; moved: boolean }
  // …or one of its two defining points (re-aims the line).
  | { kind: "guidePoint"; id: number; which: string; grabOffset: Vec2; moved: boolean }
  // Whole multi-selection move: `anchor` names the snap-anchor landmark (nearest
  // centroid / corner / free joint to the grab). Its world position is re-read live
  // each move — a stored position would go stale if a sketch solve nudged a member,
  // and the accumulated increments would let the members drift apart.
  | {
      kind: "multi";
      bodies: number[];
      joints: number[];
      anchor: DragAnchorSpec;
      grabOffset: Vec2;
      moved: boolean;
      osnap?: DragObjSnap;
      align?: DragAlign;
    }
  // Feature-selection move: the selected control vertices + joints of one body shift by
  // the same delta; `anchor` is the grabbed feature (a vertex or joint ref) — the snap
  // anchor, the object-snap reference and what implicit constraints align.
  | {
      kind: "features";
      bodyId: number;
      verts: { hole: number | null; index: number }[];
      joints: number[];
      anchor: MeasureRef;
      grabOffset: Vec2;
      moved: boolean;
      osnap?: DragObjSnap;
      align?: DragAlign;
    }
  // Rigid (Shift) drag: the grabbed selection moves like in simulation — the solver drives
  // it each frame, grounds hold, and the rest of the scene is frozen (`freeze`).
  | { kind: "rigid"; driver: Driver; freeze: SolveFreeze; moved: boolean };
let leftDrag: LeftDrag | null = null;

/** Current world position of a drag's anchor (the point that snaps to the grid). */
function dragAnchorWorld(d: LeftDrag): Vec2 {
  if (d.kind === "vertex") {
    const body = scene.getBody(d.bodyId)!;
    return d.hole === null
      ? scene.bodyControlWorld(body)[d.index]
      : scene.bodyHoleControlWorld(body, d.hole)[d.index];
  }
  if (d.kind === "body") return add(scene.getBody(d.id)!.pos, d.anchorOffset);
  if (d.kind === "measureLabel") {
    if (d.temp) {
      const td = getTempDim(d.id);
      return (td && tempDimLabelPos(td)) ?? vec(0, 0);
    }
    return scene.measurementLabelPos(scene.getMeasurement(d.id)!) ?? vec(0, 0);
  }
  if (d.kind === "patternHandle") {
    const info = scene.patternInfo(d.id);
    if (!info) return vec(0, 0);
    return d.axis === "centre" ? info.circular?.centre ?? info.anchor : info.axes[d.axis]?.end ?? info.anchor;
  }
  if (d.kind === "multi") {
    return "jointId" in d.anchor
      ? scene.jointWorld(scene.getJoint(d.anchor.jointId)!)
      : add(scene.getBody(d.anchor.bodyId)!.pos, d.anchor.offset);
  }
  if (d.kind === "features") {
    const r = scene.resolveMeasureRef(d.anchor);
    return r?.kind === "point" ? r.p : vec(0, 0);
  }
  if (d.kind === "rigid") return d.driver.target; // solver-driven; no snap anchor
  if (d.kind === "guide") return guideAnchorWorld(scene.getGuide(d.id)!);
  if (d.kind === "guidePoint") return scene.guidePointWorld(scene.getGuide(d.id)!, d.which) ?? vec(0, 0);
  // Fillet drags map the cursor to a radius directly (never snapped); the corner anchors.
  if (d.kind === "fillet") {
    const body = scene.getBody(d.bodyId)!;
    return d.hole === null
      ? scene.bodyControlWorld(body)[d.index]
      : scene.bodyHoleControlWorld(body, d.hole)[d.index];
  }
  return scene.jointWorld(scene.getJoint(d.id)!);
}

/** Body-move snap anchor: the centroid or nearest control vertex to the grab point. */
function bodyDragAnchor(bodyId: number, grab: Vec2): Vec2 {
  const body = scene.getBody(bodyId)!;
  let best = body.pos; // centroid is always a candidate
  let bestD = dist(grab, best);
  for (const v of scene.bodyControlWorld(body)) {
    const d = dist(grab, v);
    if (d < bestD) {
      bestD = d;
      best = v;
    }
  }
  return best;
}

// --- object snapping ----------------------------------------------------------
/** A candidate object-snap reference: the ref, its world point (the drag anchor) and spec. */
type ObjSnapPick = { ref: MeasureRef; anchor: Vec2; spec: DragAnchorSpec };

/** The control loops (outer outline + holes) of a body in world space. */
function bodyControlLoops(body: Body, s: Scene = scene): { verts: Vec2[]; hole: number | null }[] {
  const loops = [{ verts: s.bodyControlWorld(body), hole: null as number | null }];
  for (let hi = 0; hi < (body.holes?.length ?? 0); hi++) {
    loops.push({ verts: s.bodyHoleControlWorld(body, hi), hole: hi });
  }
  return loops;
}

/**
 * Choose the object-snap reference for a drag of `bodyIds` + `jointIds` grabbed at
 * `grab`. The nearest feature within pick range wins, by priority: a control vertex (or
 * a dragged free joint), then an edge midpoint, then a control edge (its midpoint becomes
 * the anchor); with nothing in range the object's centre is the default — a single
 * body's centroid, or the bounding-box centre of a multi-selection. Null only when
 * nothing dragged still exists.
 */
function pickObjSnapRef(bodyIds: number[], jointIds: number[], grab: Vec2): ObjSnapPick | null {
  const r = pickRadius();
  const bodies: Body[] = [];
  for (const id of bodyIds) {
    const b = scene.getBody(id);
    if (b) bodies.push(b);
  }
  const joints = jointIds.map((id) => scene.getJoint(id)).filter((j) => !!j);
  type Cand = ObjSnapPick & { d: number };
  let vert: Cand | null = null;
  let mid: Cand | null = null;
  let edge: Cand | null = null;
  const better = (cur: Cand | null, c: Cand): Cand | null => (c.d <= r && (!cur || c.d < cur.d) ? c : cur);
  const bodyRef = (body: Body, at: Vec2, ref: MeasureRef, d: number): Cand => ({
    ref,
    anchor: at,
    spec: { bodyId: body.id, offset: sub(at, body.pos) },
    d,
  });
  for (const body of bodies) {
    for (const { verts, hole } of bodyControlLoops(body)) {
      const n = verts.length;
      for (let i = 0; i < n; i++) {
        const v = verts[i];
        const vref: MeasureRef =
          hole === null ? { kind: "vertex", bodyId: body.id, index: i } : { kind: "vertex", bodyId: body.id, index: i, hole };
        vert = better(vert, bodyRef(body, v, vref, dist(grab, v)));
        if (n < 2) continue; // a 1-point loop (disk hole) has no edges
        const w = verts[(i + 1) % n];
        const m = scale(add(v, w), 0.5);
        const eref: MidpointHost =
          hole === null ? { kind: "edge", bodyId: body.id, index: i } : { kind: "edge", bodyId: body.id, index: i, hole };
        // The midpoint is an element of its own (alignments can bind it), not a bodyPoint.
        mid = better(mid, bodyRef(body, m, { kind: "midpoint", of: eref }, dist(grab, m)));
        edge = better(edge, bodyRef(body, m, eref, distToSegment(grab, v, w)));
      }
    }
  }
  for (const j of joints) {
    const p = scene.jointWorld(j);
    vert = better(vert, { ref: { kind: "joint", jointId: j.id }, anchor: p, spec: { jointId: j.id }, d: dist(grab, p) });
  }
  const pick = vert ?? mid ?? edge;
  if (pick) return pick;

  // Default: the centre of what's dragged.
  if (bodies.length === 0) {
    // Free joints only: the nearest joint stands in for the centre.
    let best = joints[0];
    if (!best) return null;
    let bd = Infinity;
    for (const j of joints) {
      const d = dist(grab, scene.jointWorld(j));
      if (d < bd) {
        bd = d;
        best = j;
      }
    }
    return { ref: { kind: "joint", jointId: best.id }, anchor: scene.jointWorld(best), spec: { jointId: best.id } };
  }
  let centre = bodies[0].pos;
  if (bodies.length > 1 || joints.length > 0) {
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    const extend = (p: Vec2): void => {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
    };
    for (const body of bodies) for (const v of scene.bodyControlWorld(body)) extend(v);
    for (const j of joints) extend(scene.jointWorld(j));
    centre = vec((minX + maxX) / 2, (minY + maxY) / 2);
  }
  // The centre rides the first body (a plain move only translates, so it stays put).
  const host = bodies[0];
  // A lone regular polygon is grabbed by its centre — a real point reference (alignments can bind it).
  if (bodies.length === 1 && joints.length === 0 && host.regular) return bodyRef(host, centre, { kind: "centre", bodyId: host.id }, 0);
  return bodyRef(host, centre, { kind: "bodyPoint", bodyId: host.id, local: rotate(sub(centre, host.pos), -host.angle) }, 0);
}

/** What a drag moves (so those features are left out of the object-snap targets). */
function dragMembers(d: LeftDrag): { bodies: Set<number>; joints: Set<number> } {
  const bodies = new Set<number>();
  const joints = new Set<number>();
  if (d.kind === "body") bodies.add(d.id);
  else if (d.kind === "joint") joints.add(d.id);
  else if (d.kind === "multi") {
    d.bodies.forEach((id) => bodies.add(id));
    d.joints.forEach((id) => joints.add(id));
  } else if (d.kind === "features") d.joints.forEach((id) => joints.add(id));
  return { bodies, joints };
}

/**
 * What a drag's snapping / alignment must ignore: the dragged members; for a vertex
 * reshape drag also the joints stuck to that vertex (the joint↔node link — e.g. a shaft
 * joint at a hole's centre — travels with it) and, via `vertex`, the vertex itself
 * plus the two edges it ends.
 */
function dragSnapExclusions(d: LeftDrag): {
  bodies: Set<number>;
  joints: Set<number>;
  vertex?: { bodyId: number; keys: Set<string> };
} {
  const excl = dragMembers(d);
  if (d.kind !== "vertex" && d.kind !== "features") return excl;
  // A vertex reshape / feature-set drag: the moving vertices (and the joints stuck to
  // them) are excluded from the targets; see objSnapTargets for the edges they end.
  const body = scene.getBody(d.bodyId);
  if (!body) return excl;
  const verts = d.kind === "vertex" ? [{ hole: d.hole, index: d.index }] : d.verts;
  const worlds = verts.map((v) => outlineControlWorld(body, v.hole)[v.index]).filter((p): p is Vec2 => !!p);
  for (const j of scene.joints) {
    if (j.bodyId === d.bodyId && worlds.some((w) => dist(scene.jointWorld(j), w) < VERTEX_LINK_EPS)) excl.joints.add(j.id);
  }
  return { ...excl, vertex: { bodyId: d.bodyId, keys: new Set(verts.map((v) => vertKey(v.hole, v.index))) } };
}

/** Key naming one control vertex of a body (outer outline or hole `hole`) in an exclusion set. */
function vertKey(hole: number | null, index: number): string {
  return `${hole ?? "o"}:${index}`;
}

/** An object-snap target feature: its geometry, and the reference it stands for as a
 *  constraint / measurement element (null for features no sketch constraint can take —
 *  the centre of a free-form body — and for the context ghost's features). */
type SnapPoint = { p: Vec2; ref: MeasureRef | null };
type SnapLine = { a: Vec2; b: Vec2; ref: MeasureRef | null };

/**
 * Object-snap targets: the same features on everything that isn't being dragged —
 * other bodies' control vertices, edge midpoints, centroids and control edges (outer +
 * holes), joints (not on a dragged body), rails and reference polyline edges (each with
 * its midpoint), and reference points.
 */
function objSnapTargets(
  excludeBodies: Set<number>,
  excludeJoints: Set<number>,
  excludeVertex?: { bodyId: number; keys: Set<string> }
): { points: SnapPoint[]; lines: SnapLine[] } {
  const own = objSnapTargetsOf(scene, excludeBodies, excludeJoints, excludeVertex);
  // The context ghost's features snap too (a hole onto the pin that will hold it), as
  // positions only: their refs are stripped, so no implicit constraint, guide
  // auto-coincident or measurement can ever bind live geometry to the ghost.
  const ghost = ghostSnapTargets();
  if (!ghost) return own;
  return { points: own.points.concat(ghost.points), lines: own.lines.concat(ghost.lines) };
}

/** The ghost's object-snap features (cached per build), or null when no ghost is shown. */
function ghostSnapTargets(): { points: SnapPoint[]; lines: SnapLine[] } | null {
  const levels = ghostScenes();
  if (levels.length === 0) return null;
  if (!ghostTargets) {
    const points: SnapPoint[] = [];
    const lines: SnapLine[] = [];
    for (const g of levels) {
      const t = objSnapTargetsOf(g, new Set(), new Set());
      for (const p of t.points) points.push({ p: p.p, ref: null });
      for (const l of t.lines) lines.push({ a: l.a, b: l.b, ref: null });
    }
    ghostTargets = { points, lines };
  }
  return ghostTargets;
}

/** The object-snap features of one scene (the live scene, or a context-ghost level). */
function objSnapTargetsOf(
  s: Scene,
  excludeBodies: Set<number>,
  excludeJoints: Set<number>,
  excludeVertex?: { bodyId: number; keys: Set<string> }
): { points: SnapPoint[]; lines: SnapLine[] } {
  const points: SnapPoint[] = [];
  const lines: SnapLine[] = [];
  for (const body of s.bodies) {
    if (excludeBodies.has(body.id)) continue;
    // A vertex reshape / feature-set drag: its own body's other features are fair
    // targets (a hole centre onto a corner, say), but not the moving vertices, the edges
    // they end (and their midpoints), or the centroid — all of which move and would stick.
    const ex = excludeVertex?.bodyId === body.id ? excludeVertex.keys : null;
    if (!ex) points.push({ p: body.pos, ref: body.regular ? { kind: "centre", bodyId: body.id } : null });
    for (const { verts, hole } of bodyControlLoops(body, s)) {
      const n = verts.length;
      const exAt = (i: number): boolean => !!ex && ex.has(vertKey(hole, i));
      if (!ex && hole !== null && s.outlineRegular(body, hole) !== null) {
        const c = scale(verts.reduce((acc, q) => add(acc, q), vec(0, 0)), 1 / n);
        points.push({ p: c, ref: { kind: "centre", bodyId: body.id, hole } });
      }
      for (let i = 0; i < n; i++) {
        const v = verts[i];
        if (!exAt(i)) {
          points.push({
            p: v,
            ref: hole === null ? { kind: "vertex", bodyId: body.id, index: i } : { kind: "vertex", bodyId: body.id, index: i, hole },
          });
        }
        if (n < 2) continue;
        if (exAt(i) || exAt((i + 1) % n)) continue;
        const w = verts[(i + 1) % n];
        const eref: MidpointHost =
          hole === null ? { kind: "edge", bodyId: body.id, index: i } : { kind: "edge", bodyId: body.id, index: i, hole };
        points.push({ p: scale(add(v, w), 0.5), ref: { kind: "midpoint", of: eref } });
        lines.push({ a: v, b: w, ref: eref });
      }
    }
  }
  const jointExcluded = (id: number): boolean => {
    const j = s.getJoint(id);
    return !j || excludeJoints.has(id) || (j.bodyId !== null && excludeBodies.has(j.bodyId));
  };
  for (const j of s.joints) {
    if (!jointExcluded(j.id)) points.push({ p: s.jointWorld(j), ref: { kind: "joint", jointId: j.id } });
  }
  for (const c of s.constraints) {
    if (c.kind !== "slider" || jointExcluded(c.railA) || jointExcluded(c.railB)) continue;
    const a = s.jointWorld(s.getJoint(c.railA)!);
    const b = s.jointWorld(s.getJoint(c.railB)!);
    const rref: MidpointHost = { kind: "rail", sliderId: c.id };
    lines.push({ a, b, ref: rref });
    points.push({ p: scale(add(a, b), 0.5), ref: { kind: "midpoint", of: rref } });
  }
  for (const g of s.guides) {
    for (const which of s.guidePointKeys(g)) {
      const q = s.guidePointWorld(g, which);
      if (q) points.push({ p: q, ref: { kind: "guidePoint", guideId: g.id, which } });
    }
    for (const l of s.guideLines(g)) {
      const lref = guideLineRef(g.id, l.edge);
      lines.push({ a: l.a, b: l.b, ref: lref });
      points.push({ p: scale(add(l.a, l.b), 0.5), ref: { kind: "midpoint", of: lref } });
    }
  }
  return { points, lines };
}

/**
 * Object-snap a drag: `raw` is where the drag anchor (the reference point, or a line
 * reference's midpoint) would land unsnapped. A point reference lands on the nearest
 * target point within range, else projects onto the nearest target segment (clamped
 * to its span). A line reference translates perpendicular onto the
 * nearest (near-)parallel target line so the two become collinear — motion along the
 * line stays free. Records the hit for the highlight; null (hit cleared) when nothing
 * is within range, so the caller falls back to the grid/guide snap.
 */
function objSnapTarget(d: LeftDrag, os: DragObjSnap, raw: Vec2): Vec2 | null {
  os.hit = null;
  os.hitInfinite = false;
  const cur = scene.resolveMeasureRef(os.ref);
  if (!cur) return null;
  const excl = dragSnapExclusions(d);
  const targets = objSnapTargets(excl.bodies, excl.joints, excl.vertex);
  const r = OBJ_SNAP_PX / view.scale;
  if (cur.kind === "point") {
    let bestP: Vec2 | null = null;
    let bd = r;
    for (const t of targets.points) {
      const dd = dist(raw, t.p);
      if (dd <= bd) {
        bd = dd;
        bestP = t.p;
      }
    }
    if (bestP) {
      os.hit = { kind: "point", p: bestP };
      return bestP;
    }
    let bestL: { a: Vec2; b: Vec2 } | null = null;
    let proj: Vec2 | null = null;
    bd = r;
    for (const l of targets.lines) {
      const ab = sub(l.b, l.a);
      const L = lenSq(ab);
      if (L < 1e-12) continue;
      const t = Math.max(0, Math.min(1, dot(sub(raw, l.a), ab) / L));
      const q = add(l.a, scale(ab, t));
      const dd = dist(raw, q);
      if (dd <= bd) {
        bd = dd;
        bestL = l;
        proj = q;
      }
    }
    if (bestL && proj) {
      os.hit = { kind: "line", a: bestL.a, b: bestL.b };
      return proj;
    }
    return null;
  }
  // Line reference: the dragged edge, translated so its midpoint sits at `raw`.
  const dir = normalize(sub(cur.b, cur.a));
  if (dir.x === 0 && dir.y === 0) return null;
  const sinTol = Math.sin(OBJ_SNAP_PARALLEL_TOL);
  let bestL: { a: Vec2; b: Vec2 } | null = null;
  let corr: Vec2 | null = null;
  let bd = r;
  for (const l of targets.lines) {
    const e = normalize(sub(l.b, l.a));
    if (e.x === 0 && e.y === 0) continue;
    if (Math.abs(cross(dir, e)) > sinTol) continue; // not parallel — can't align by translation
    const n = perp(e);
    const off = dot(sub(raw, l.a), n); // signed perpendicular distance to the target line
    if (Math.abs(off) <= bd) {
      bd = Math.abs(off);
      bestL = l;
      corr = scale(n, -off);
    }
  }
  if (bestL && corr) {
    os.hit = { kind: "line", a: bestL.a, b: bestL.b };
    os.hitInfinite = true;
    return add(raw, corr);
  }
  return null;
}

/**
 * Hover preview for object snap (select mode): the reference a plain drag started at
 * `p` would use — for the multi-selection when `p` is on it, else the joint or body
 * under the cursor. None over a selected body's edit handles (those drags reshape).
 */
function hoverObjSnapRef(p: Vec2): ResolvedMeasureRef | null {
  if (selectedBodyFilletHandleAt(p)) return null;
  let ref: MeasureRef | null = null;
  const node = selectedBodyNodeAt(p);
  if (node && selection?.kind === "body") {
    // A vertex handle: the reshape drag snaps that vertex itself.
    ref =
      node.hole === null
        ? { kind: "vertex", bodyId: selection.id, index: node.index }
        : { kind: "vertex", bodyId: selection.id, index: node.index, hole: node.hole };
  } else if (multiSel && multiHitAt(p)) ref = pickObjSnapRef([...multiSel.bodies], [...multiSel.joints], p)?.ref ?? null;
  else if (hoverJoint !== null) ref = { kind: "joint", jointId: hoverJoint };
  else if (hoverBody !== null) ref = pickObjSnapRef([hoverBody], [], p)?.ref ?? null;
  return ref ? scene.resolveMeasureRef(ref) : null;
}

/**
 * Object-snap highlight for the renderer: during a drag, the dragged reference (solid)
 * and what it's snapped onto (dashed); before one, the reference a drag would pick.
 */
function dragSnapView(): { ref: ResolvedMeasureRef; hit: ResolvedMeasureRef | null; hitInfinite: boolean } | null {
  if (mode !== "draw" || !objSnapEnabled) return null;
  if (leftDrag) {
    if (!("osnap" in leftDrag) || !leftDrag.osnap) return null;
    const ref = scene.resolveMeasureRef(leftDrag.osnap.ref);
    return ref ? { ref, hit: leftDrag.osnap.hit, hitInfinite: leftDrag.osnap.hitInfinite } : null;
  }
  if (tool === null && !boxSelect && !rotateDrag && hoverObjSnap) return { ref: hoverObjSnap, hit: null, hitInfinite: false };
  return null;
}

// --- implicit constraints (alignment while dragging) ---------------------------------
/** Whether a reference can take a sketch constraint (see `Scene.addSketchConstraint`). */
function alignPointRef(r: MeasureRef): boolean {
  return r.kind === "joint" || r.kind === "vertex" || r.kind === "centre" || r.kind === "midpoint" || r.kind === "guidePoint";
}
function alignLineRef(r: MeasureRef): boolean {
  return r.kind === "rail" || r.kind === "edge" || r.kind === "guideLine" || r.kind === "patternAxis";
}

/** Fresh implicit-constraint state for a drag whose reference is `ref` — none when the
 *  reference can't take a constraint (a free-form body's centre, a `bodyPoint`). */
function newDragAlign(ref: MeasureRef | null | undefined): DragAlign | undefined {
  if (!autoConstrain || !ref || !(alignPointRef(ref) || alignLineRef(ref))) return undefined;
  return { ref, hover: null, cands: [], matches: [], slip: vec(0, 0) };
}

/** Whether an equivalent sketch constraint (same kind, same two refs, either order) exists. */
function sketchConstraintExists(kind: SketchConstraintKind, a: MeasureRef, b: MeasureRef): boolean {
  return scene.sketch.some(
    (c) =>
      c.kind === kind &&
      c.refB !== null &&
      ((sameMeasureRef(c.refA, a) && sameMeasureRef(c.refB, b)) || (sameMeasureRef(c.refA, b) && sameMeasureRef(c.refB, a)))
  );
}

/**
 * Advance a drag's implicit-constraint state — on every move, and every frame while the
 * cursor rests (a hover is a matter of time, not motion).
 *
 * Hovering: the nearest constraint-capable target under the dragged reference — a point
 * reference over a target point, else over a target line (a point in range always beats
 * a line, so a midpoint wins over the edge it sits on); a line reference over a target
 * point along its segment — within the object-snap range of where the *cursor* put the
 * reference (`slip` undoes the grid / object snap). Held for `ALIGN_HOVER_MS` it joins the
 * armed candidates; the oldest drops once there are more than `ALIGN_MAX_CANDS`, and
 * moving off a target keeps every candidate armed.
 *
 * Matching (one per candidate): point↔point within `ALIGN_TOL_PX` of the same y →
 * horizontal, same x → vertical (sitting right on top of it is a placement, not an
 * alignment — no match); a point on a candidate line's infinite line, or a candidate
 * point on the dragged line's, → point-on-line coincident. Judged on the geometry as
 * placed (snapped, live-solved) — that's what the release commits; the constraint's own
 * solve then closes the residual. A constraint that already exists never re-matches, and
 * candidates are matched newest first, so of two matches that aren't independent (see
 * `ALIGN_INDEPENDENT_TOL`) it is the older candidate's that drops.
 */
function updateDragAlign(d: LeftDrag, al: DragAlign, now: number): void {
  al.matches = [];
  const cur = autoConstrain ? scene.resolveMeasureRef(al.ref) : null;
  // Switched off mid-drag (or the reference is gone): forget what was armed and stop scanning.
  if (!cur) {
    al.hover = null;
    al.cands = [];
    return;
  }
  // --- hover → candidate ---
  const excl = dragSnapExclusions(d);
  const targets = objSnapTargets(excl.bodies, excl.joints, excl.vertex);
  const r = OBJ_SNAP_PX / view.scale;
  let over: MeasureRef | null = null;
  let bd = r;
  if (cur.kind === "point") {
    const at = add(cur.p, al.slip);
    for (const t of targets.points) {
      if (!t.ref) continue;
      const dd = dist(at, t.p);
      if (dd <= bd) {
        bd = dd;
        over = t.ref;
      }
    }
    if (!over) {
      for (const t of targets.lines) {
        if (!t.ref) continue;
        const ab = sub(t.b, t.a);
        const L = lenSq(ab);
        if (L < 1e-12) continue;
        const u = Math.max(0, Math.min(1, dot(sub(at, t.a), ab) / L));
        const dd = dist(at, add(t.a, scale(ab, u)));
        if (dd <= bd) {
          bd = dd;
          over = t.ref;
        }
      }
    }
  } else {
    const a = add(cur.a, al.slip);
    const b = add(cur.b, al.slip);
    for (const t of targets.points) {
      if (!t.ref) continue;
      const dd = distToSegment(t.p, a, b);
      if (dd <= bd) {
        bd = dd;
        over = t.ref;
      }
    }
  }
  if (!over) al.hover = null;
  else if (!al.hover || !sameMeasureRef(al.hover.ref, over)) al.hover = { ref: over, since: now };
  const held = al.hover;
  if (held && now - held.since >= ALIGN_HOVER_MS && !al.cands.some((c) => sameMeasureRef(c, held.ref))) {
    al.cands.push(held.ref);
    if (al.cands.length > ALIGN_MAX_CANDS) al.cands.shift(); // the oldest makes room
  }
  // --- candidates → matches ---
  al.cands = al.cands.filter((c) => scene.resolveMeasureRef(c) !== null);
  const tol = ALIGN_TOL_PX / view.scale;
  const kept: AlignMatch[] = [];
  const eqs: AlignEquation[] = [];
  // Newest candidate first: when two matches aren't independent, the one just armed is
  // the live intent and the older candidate stays armed but stops previewing.
  for (let i = al.cands.length - 1; i >= 0; i--) {
    const candRef = al.cands[i];
    const cand = scene.resolveMeasureRef(candRef);
    if (!cand) continue;
    const m = alignMatch(cur, cand, candRef, tol);
    if (!m || sketchConstraintExists(m.kind, candRef, al.ref)) continue;
    // Keep it only while an exact correction for every kept match still exists: the drag
    // has two degrees of freedom, and equations that pull the same way can't both be met.
    const es = alignEquations(m.kind, cur, cand);
    if (es.length === 0 || eqs.length + es.length > 2) continue;
    if (es.some((e) => eqs.some((k) => Math.abs(cross(k.n, e.n)) < ALIGN_INDEPENDENT_TOL))) continue;
    eqs.push(...es);
    kept.push(m);
  }
  kept.reverse(); // back into arm order, so a standing preview keeps its place
  al.matches = kept;
}

/** The alignment a release would constrain between the dragged reference `cur` and one
 *  candidate — null when the two aren't aligned within `tol`, or can't be constrained. */
function alignMatch(
  cur: ResolvedMeasureRef,
  cand: ResolvedMeasureRef,
  candRef: MeasureRef,
  tol: number
): AlignMatch | null {
  if (cur.kind === "point" && cand.kind === "point") {
    const dx = Math.abs(cur.p.x - cand.p.x);
    const dy = Math.abs(cur.p.y - cand.p.y);
    // Dropped on the candidate itself: pin the two together instead of aligning one axis.
    if (dx <= tol && dy <= tol) return { kind: "coincident", cand: candRef, from: cand.p, to: cur.p };
    if (dy <= tol) return { kind: "horizontal", cand: candRef, from: cand.p, to: cur.p };
    if (dx <= tol) return { kind: "vertical", cand: candRef, from: cand.p, to: cur.p };
    return null;
  }
  if (cur.kind === "point" && cand.kind === "line") return pointOnLineMatch(cur.p, cand, candRef, tol);
  if (cur.kind === "line" && cand.kind === "point") return pointOnLineMatch(cand.p, cur, candRef, tol);
  return null;
}

/** Point-on-line match: `p` within `tol` of `line`'s infinite line. The dotted preview
 *  runs from the nearer end of the defining segment out to the point (collapsed to the
 *  point when it lies within the segment's span — the badge alone marks it then). */
function pointOnLineMatch(
  p: Vec2,
  line: Extract<ResolvedMeasureRef, { kind: "line" }>,
  candRef: MeasureRef,
  tol: number
): AlignMatch | null {
  const ab = sub(line.b, line.a);
  const L = lenSq(ab);
  if (L < 1e-12) return null;
  const u = dot(sub(p, line.a), ab) / L;
  const foot = add(line.a, scale(ab, u));
  if (dist(p, foot) > tol) return null;
  const from = u < 0 ? line.a : u > 1 ? line.b : foot;
  return { kind: "coincident", cand: candRef, from, to: p };
}

/** One previewed alignment as a line constraint on the drag's correction: translating the
 *  dragged geometry by `delta` makes it exact iff `dot(n, delta) === d`, `n` a unit normal. */
type AlignEquation = { n: Vec2; d: number };

/**
 * The correction equations of a previewed alignment: the y axis for a horizontal, the x
 * axis for a vertical, the line's normal for a point-on-line (whichever side the line is
 * on — moving the dragged point onto it, or the dragged line onto the candidate point),
 * and **both** axes for a point-on-point coincident, which leaves the drag no freedom.
 * Empty when that pair can't take that alignment, a degenerate line included.
 */
function alignEquations(
  kind: SketchConstraintKind,
  cur: ResolvedMeasureRef,
  cand: ResolvedMeasureRef
): AlignEquation[] {
  if (cur.kind === "point" && cand.kind === "point") {
    const ex = { n: vec(1, 0), d: cand.p.x - cur.p.x };
    const ey = { n: vec(0, 1), d: cand.p.y - cur.p.y };
    if (kind === "horizontal") return [ey];
    if (kind === "vertical") return [ex];
    if (kind === "coincident") return [ex, ey];
    return [];
  }
  if (kind !== "coincident") return [];
  if (cur.kind === "point" && cand.kind === "line") {
    const n = lineNormal(cand.a, cand.b);
    return n ? [{ n, d: dot(n, sub(cand.a, cur.p)) }] : [];
  }
  if (cur.kind === "line" && cand.kind === "point") {
    const n = lineNormal(cur.a, cur.b);
    return n ? [{ n, d: dot(n, sub(cand.p, cur.a)) }] : [];
  }
  return [];
}

/** Unit normal of the line through `a`–`b` (null when degenerate). */
function lineNormal(a: Vec2, b: Vec2): Vec2 | null {
  const ab = sub(b, a);
  if (lenSq(ab) < 1e-12) return null;
  return normalize(perp(ab));
}

/**
 * The translation that makes every previewed alignment exact at once: one equation pins
 * the dragged geometry along its own normal (the shortest such move), two independent
 * ones pin it outright (Cramer on the 2×2 system) — whether they come from two alignments
 * or from the two axes of one point-on-point coincident. Applied before the constraints are
 * placed, so the solver starts from satisfied constraints — asked to close even a small
 * gap itself it splits the correction with the other side, and when that side is pinned
 * by dimensions (a fully dimensioned part) the solve can fail and the placement gets
 * rejected.
 */
function alignCorrection(eqs: AlignEquation[]): Vec2 | null {
  if (eqs.length === 0) return null;
  if (eqs.length === 1) return scale(eqs[0].n, eqs[0].d); // |n| = 1, so this is the foot
  const [p, q] = eqs;
  const det = cross(p.n, q.n);
  // Parallel normals never pair up when matching; if one slips through, honour the first.
  if (Math.abs(det) < 1e-9) return scale(p.n, p.d);
  return vec((p.d * q.n.y - p.n.y * q.d) / det, (p.n.x * q.d - p.d * q.n.x) / det);
}

/** Translate a drag's geometry by `delta` — the same movers the drag itself uses. */
function moveDragged(d: LeftDrag, delta: Vec2): void {
  if (d.kind === "vertex") scene.moveBodyVertex(d.bodyId, d.index, delta, d.hole);
  else if (d.kind === "body") scene.moveBody(d.id, delta);
  else if (d.kind === "joint") scene.moveJoint(d.id, delta); // an arrow start brings its carriage home (model)
  else if (d.kind === "multi") {
    for (const id of d.bodies) scene.moveBody(id, delta);
    for (const id of d.joints) scene.moveJoint(id, delta);
  } else if (d.kind === "features") moveFeatures(d, delta);
}

/**
 * Place the constraints the implicit alignments previewed (on drag release). Every
 * previewed gap is closed by one correction first, then each constraint is placed in
 * turn with the dragged reference second, so a pose constraint (both ends on components)
 * moves the dragged part rather than the candidate's. Selection stays on what was
 * dragged; a rejected placement flashes the conflicts like any constraint placement and
 * leaves the other one standing.
 */
function placeAlignConstraint(d: LeftDrag, al: DragAlign): void {
  if (al.matches.length === 0) return;
  // Close the (sub-tolerance) gaps exactly first — see alignCorrection. The settle solve
  // has moved the geometry since the matches were found, so read the equations live.
  const cur = scene.resolveMeasureRef(al.ref);
  if (!cur) return;
  const live: { match: AlignMatch; eqs: AlignEquation[] }[] = [];
  for (const match of al.matches) {
    const cand = scene.resolveMeasureRef(match.cand);
    const eqs = cand ? alignEquations(match.kind, cur, cand) : [];
    if (eqs.length > 0) live.push({ match, eqs });
  }
  if (live.length === 0) return;
  const corr = alignCorrection(live.flatMap((l) => l.eqs));
  if (corr && (corr.x !== 0 || corr.y !== 0)) moveDragged(d, corr);
  let placed = 0;
  const conflicts: SketchBreak[] = [];
  const refused: string[] = [];
  for (const { match } of live) {
    const problem = scene.sketchConstraintProblem(match.kind, match.cand, al.ref);
    const { constraint, breaks } = placeConstraint(scene, match.kind, match.cand, al.ref);
    if (constraint) placed++;
    else if (breaks.length) conflicts.push(...breaks);
    else refused.push(problem ?? `${CONSTRAINT_NAME[match.kind]} isn't possible between these elements.`);
  }
  if (placed > 0) setSketchVisible(true); // a constraint placed while the layer is hidden would be invisible
  // A silent no-op would read as "nothing happened": say why.
  if (conflicts.length > 0) {
    flashSketchItems(conflicts);
    const what = describeBreaks(conflicts);
    notify(
      what
        ? `Constraint not applied: it conflicts with ${what} (flashing red).`
        : "Constraint not applied: it can't be satisfied without breaking an existing dimension or constraint (the conflict is flashing red).",
      "error"
    );
  }
  for (const r of new Set(refused)) notifyThrottled(r);
  markDirty(); // the alignment correction above moved the geometry
}

/** Implicit-constraint preview for the renderer: the armed candidates, the dragged
 *  reference, and the alignments a release would constrain (none while nothing is armed). */
function dragAlignView(): RenderInput["dragAlign"] {
  if (mode !== "draw" || !leftDrag || !("align" in leftDrag) || !leftDrag.align?.cands.length) return null;
  const ref = scene.resolveMeasureRef(leftDrag.align.ref);
  if (!ref) return null;
  const cands: ResolvedMeasureRef[] = [];
  for (const c of leftDrag.align.cands) {
    const resolved = scene.resolveMeasureRef(c);
    if (resolved) cands.push(resolved);
  }
  return cands.length > 0 ? { ref, cands, matches: leftDrag.align.matches } : null;
}

/**
 * Placement snap for a new point (a joint, a round hole's centre): with object snap on,
 * the nearest object-snap target point within range (corners, edge midpoints, centroids,
 * hole centres, joints, guide points) wins; otherwise the usual grid / guideline snap.
 */
function placeSnap(p: Vec2): Vec2 {
  if (objSnapEnabled) {
    const r = OBJ_SNAP_PX / view.scale;
    let best: Vec2 | null = null;
    let bd = r;
    for (const { p: q } of objSnapTargets(new Set(), new Set()).points) {
      const d = dist(p, q);
      if (d <= bd) {
        bd = d;
        best = q;
      }
    }
    if (best) return best;
  }
  return snap(p);
}

// --- group isolation (editing inside a group) -------------------------------
/**
 * The permanent group opened for editing "from the inside" (double-click one of its
 * members). A group is normally selection-atomic — clicking any member selects, drags
 * and transforms the whole thing — which makes the parts inside it unreachable; opening
 * it suspends that for **this one group**, so its bodies and joints select, drag and
 * reshape one at a time, while the rest of the drawing is veiled and inert. The same
 * idea as opening a component definition, but with no context switch: the material is
 * the scene's own, so nothing is serialized and undo / save never see this.
 *
 * Draw mode only (in sim a group *is* one rigid body), and dropped as soon as the group
 * stops existing (ungrouped, deleted, undone, or a context switch).
 */
let groupEdit: number | null = null;

/** The open group, or null (also null once its id no longer names a group). */
function editedGroup(): BodyGroup | null {
  return groupEdit === null ? null : scene.groups.find((g) => g.id === groupEdit) ?? null;
}

/**
 * The group a body belongs to **for selection and dragging** — undefined while that
 * same group is the open one, which is exactly what makes its members individually
 * editable. Every atomicity site goes through this pair rather than `scene.groupOf`.
 */
function selGroupOf(bodyId: number): BodyGroup | undefined {
  const g = scene.groupOf(bodyId);
  return g && g.id === groupEdit ? undefined : g;
}
function selGroupOfJoint(jointId: number): BodyGroup | undefined {
  const g = scene.groupOfJoint(jointId);
  return g && g.id === groupEdit ? undefined : g;
}

/** Whether a body is material of the open group (always true when none is open). */
function bodyInScope(id: number): boolean {
  const g = editedGroup();
  return !g || g.bodyIds.includes(id);
}
/** Whether a joint belongs to the open group — its own free joint, or one on a member. */
function jointInScope(id: number): boolean {
  const g = editedGroup();
  if (!g) return true;
  const j = scene.getJoint(id);
  if (!j) return false;
  return j.bodyId === null ? g.jointIds.includes(j.id) : g.bodyIds.includes(j.bodyId);
}

/**
 * Whether a measurement / constraint reference names material of the open group. Guides
 * (reference geometry) never belong to a group, so they are always outside one.
 */
function refInScope(r: MeasureRef): boolean {
  switch (r.kind) {
    case "joint":
      return jointInScope(r.jointId);
    case "vertex":
    case "edge":
    case "bodyPoint":
    case "centre":
    case "disk":
      return bodyInScope(r.bodyId);
    case "rail": {
      const c = scene.constraints.find((cc) => cc.id === r.sliderId);
      return !!c && c.kind === "slider" && jointInScope(c.railA) && jointInScope(c.railB);
    }
    case "midpoint":
      return refInScope(r.of);
    case "patternAxis":
      return patternInScope(r.patternId);
    default:
      return false; // guidePoint / guideLine: reference geometry is nobody's group
  }
}

/**
 * Whether an annotation on these references is live inside the open group. It counts as
 * inside when it names **any** of the group's own material: a dimension or constraint
 * anchored to a member is part of editing that member, one entirely outside is
 * surroundings — faded with them, and not pickable until the group is left.
 */
function refsInScope(refs: (MeasureRef | null | undefined)[]): boolean {
  return !editedGroup() || refs.some((r) => !!r && refInScope(r));
}
const measurementInScope = (m: Measurement): boolean => refsInScope([m.refA, m.refB]);
const sketchInScope = (c: SketchConstraint): boolean => refsInScope(sketchRefs(c));
/** A pattern belongs to the body it was laid out on. */
function patternInScope(id: number): boolean {
  const p = scene.getPattern(id);
  return !editedGroup() || (!!p && bodyInScope(p.bodyId));
}

/** Ids of the annotations the open group owns — what the renderer keeps at full strength. */
function isolateItems(): number[] {
  const out: number[] = [];
  for (const c of scene.sketch) if (sketchInScope(c)) out.push(c.id);
  for (const m of scene.measurements) if (measurementInScope(m)) out.push(m.id);
  for (const p of scene.patterns) if (patternInScope(p.id)) out.push(p.id);
  return out;
}

/** Open a group for editing from the inside. */
function enterGroup(id: number): void {
  if (mode !== "draw" || !scene.groups.some((g) => g.id === id)) return;
  groupEdit = id;
  selection = null;
  multiSel = null;
  featureSel = null;
  updateCrumbBar();
  updateHint();
}

/** Leave the open group; it becomes one object again, and is left selected as one. */
function leaveGroup(): void {
  if (groupEdit === null) return;
  const g = editedGroup();
  groupEdit = null;
  featureSel = null;
  if (g) setMulti(new Set(g.bodyIds), new Set(g.jointIds));
  else {
    selection = null;
    multiSel = null;
  }
  updateCrumbBar();
  updateHint();
}

// --- multi-selection (Ctrl+click / box select) + permanent groups -----------
/**
 * Commit a multi-selection: expand permanent groups (selection-atomic), drop dead ids,
 * and collapse trivial results — a single ungrouped body / free joint becomes a normal
 * single selection, an empty set clears everything. Otherwise `multiSel` is set and the
 * single `selection` cleared (they're mutually exclusive).
 */
/** Add every member of a component instance (bodies + its free joints) to a selection. */
function addInstanceMembers(inst: ComponentInstance, bodies: Set<number>, joints: Set<number>): void {
  for (const e of inst.bodyMap) bodies.add(e.id);
  for (const e of [...inst.jointMap, ...inst.anchorMap]) {
    if (scene.getJoint(e.id)?.bodyId === null) joints.add(e.id);
  }
}

function setMulti(bodies: Set<number>, joints: Set<number>): void {
  // Selection-atomic units expand: groups (bodies + locked free joints) and component
  // instances (everything they expanded). Repeat until stable — an instance member can
  // pull in a group and vice versa. The group opened for editing doesn't expand: that
  // suspension is what makes its members individually selectable (see selGroupOf).
  let grew = true;
  while (grew) {
    const before = bodies.size + joints.size;
    for (const id of [...bodies]) {
      const g = selGroupOf(id);
      if (g) {
        for (const b of g.bodyIds) bodies.add(b);
        for (const j of g.jointIds) joints.add(j);
      }
      const inst = scene.instanceOfBody(id);
      if (inst) addInstanceMembers(inst, bodies, joints);
    }
    for (const id of [...joints]) {
      const g = selGroupOfJoint(id);
      if (g) {
        for (const b of g.bodyIds) bodies.add(b);
        for (const j of g.jointIds) joints.add(j);
      }
      const inst = scene.instanceOfJoint(id);
      if (inst) addInstanceMembers(inst, bodies, joints);
    }
    grew = bodies.size + joints.size > before;
  }
  for (const id of [...bodies]) if (!scene.getBody(id)) bodies.delete(id);
  for (const id of [...joints]) {
    const j = scene.getJoint(id);
    if (!j || j.bodyId !== null) joints.delete(id);
  }
  selection = null;
  if (bodies.size === 0 && joints.size === 0) {
    multiSel = null;
    return;
  }
  // A single ungrouped, non-instance body / free joint collapses to a normal selection
  // (instance material stays multi-selected so it never grows edit handles).
  if (bodies.size === 1 && joints.size === 0 && !scene.instanceOfBody([...bodies][0])) {
    multiSel = null;
    selection = { kind: "body", id: [...bodies][0] };
    return;
  }
  if (bodies.size === 0 && joints.size === 1 && !scene.instanceOfJoint([...joints][0])) {
    multiSel = null;
    selection = { kind: "joint", id: [...joints][0] };
    return;
  }
  multiSel = { bodies, joints };
}

/**
 * Ctrl/Cmd+click: toggle the body or free joint under `p` in the multi-selection (seeded
 * from the current single selection, so Ctrl+click naturally extends it). A body toggles
 * together with its whole permanent group. Returns false when nothing is under the cursor
 * (the caller starts an additive box select instead).
 */
function toggleMultiAt(p: Vec2): boolean {
  const bodies = new Set(multiSel?.bodies ?? []);
  const joints = new Set(multiSel?.joints ?? []);
  if (selection?.kind === "body") bodies.add(selection.id);
  if (selection?.kind === "joint") {
    const j = scene.getJoint(selection.id);
    if (j && j.bodyId === null) joints.add(j.id);
  }
  // A unit = a lone element, a whole group, or a whole instance — toggled together.
  const unitOf = (bodyId: number | null, jointId: number | null): { b: number[]; j: number[] } => {
    const ub = new Set<number>();
    const uj = new Set<number>();
    const inst = bodyId !== null ? scene.instanceOfBody(bodyId) : jointId !== null ? scene.instanceOfJoint(jointId) : undefined;
    if (inst) addInstanceMembers(inst, ub, uj);
    const g = bodyId !== null ? selGroupOf(bodyId) : jointId !== null ? selGroupOfJoint(jointId) : undefined;
    if (g) {
      g.bodyIds.forEach((x) => ub.add(x));
      g.jointIds.forEach((x) => uj.add(x));
    }
    if (ub.size + uj.size === 0) {
      if (bodyId !== null) ub.add(bodyId);
      if (jointId !== null) uj.add(jointId);
    }
    return { b: [...ub], j: [...uj] };
  };
  const toggleUnit = (bodyId: number | null, jointId: number | null): void => {
    const u = unitOf(bodyId, jointId);
    const on = u.b.some((x) => bodies.has(x)) || u.j.some((x) => joints.has(x));
    for (const x of u.b) (on ? bodies.delete(x) : bodies.add(x));
    for (const x of u.j) (on ? joints.delete(x) : joints.add(x));
  };
  const j = scene.jointAt(p, pickRadius());
  if (j) {
    if (j.bodyId === null) toggleUnit(null, j.id);
    else toggleUnit(j.bodyId, null);
  } else {
    const b = scene.bodyAt(p);
    if (!b) return false;
    toggleUnit(b.id, null);
  }
  setMulti(bodies, joints);
  return true;
}

/**
 * Whether a click at `p` lands on an element of the current multi-selection (a selected
 * body, a joint on one, or a selected free joint) — such a click drags the whole selection.
 * Measurement labels / constraint badges stay the topmost pick, as in single selection.
 */
function multiHitAt(p: Vec2): boolean {
  if (!multiSel) return false;
  if (measurementLabelAt(p) || sketchGlyphAt(p) !== null) return false;
  const j = scene.jointAt(p, pickRadius());
  if (j) {
    return j.bodyId === null ? multiSel.joints.has(j.id) : multiSel.bodies.has(j.bodyId);
  }
  const b = scene.bodyAt(p);
  return !!b && multiSel.bodies.has(b.id);
}

/** Begin dragging the whole multi-selection; the snap anchor is the nearest landmark to the grab. */
function startMultiDrag(grab: Vec2): void {
  if (!multiSel) return;
  let anchor: DragAnchorSpec | null = null;
  let anchorPos = grab;
  let bestD = Infinity;
  const consider = (c: Vec2, spec: DragAnchorSpec): void => {
    const d = dist(grab, c);
    if (d < bestD) {
      bestD = d;
      anchor = spec;
      anchorPos = c;
    }
  };
  for (const id of multiSel.bodies) {
    const body = scene.getBody(id);
    if (!body) continue;
    consider(body.pos, { bodyId: id, offset: vec(0, 0) });
    for (const v of scene.bodyControlWorld(body)) {
      consider(v, { bodyId: id, offset: sub(v, body.pos) });
    }
  }
  for (const id of multiSel.joints) {
    const j = scene.getJoint(id);
    if (j) consider(scene.jointWorld(j), { jointId: id });
  }
  // Object snap on: the reference feature nearest the grab becomes the anchor instead
  // (either way it's the feature implicit constraints align).
  const pick = pickObjSnapRef([...multiSel.bodies], [...multiSel.joints], grab);
  const os = objSnapEnabled ? pick : null;
  if (os) {
    anchor = os.spec;
    anchorPos = os.anchor;
  }
  if (!anchor) return; // no live members — nothing to drag
  // A free joint that is a selected body's own slider track is carried by moveBody — don't
  // move it a second time as a selected joint.
  const carried = scene.ownedTrackJointsOf(multiSel.bodies);
  leftDrag = {
    kind: "multi",
    bodies: [...multiSel.bodies],
    joints: [...multiSel.joints].filter((id) => !carried.has(id)),
    anchor,
    grabOffset: sub(grab, anchorPos),
    moved: false,
    osnap: os ? { ref: os.ref, hit: null, hitInfinite: false } : undefined,
    align: newDragAlign(pick?.ref),
  };
  canvas.style.cursor = "move";
}

/**
 * Begin a rigid (Shift) drag: the current selection — the multi-selection / group, or
 * the single body / joint under the cursor (a joint drags its body, with its whole
 * group) — is driven by the sim solver while everything else in the scene is frozen.
 * Grounds hold, pins/sliders to the frozen world constrain the motion, and the poses
 * the drag ends in become the new drawn layout. Returns false when nothing under the
 * cursor is rigid-draggable (the caller falls back to the normal click handling).
 */
function startRigidDrag(grab: Vec2): boolean {
  // What moves: the multi-selection when the grab lands on it, else the single selection.
  const bodies = new Set<number>();
  const joints = new Set<number>();
  const addBodyWithGroup = (id: number): void => {
    const g = selGroupOf(id);
    if (g) {
      g.bodyIds.forEach((b) => bodies.add(b));
      g.jointIds.forEach((jt) => joints.add(jt)); // locked free joints ride the group
    } else {
      bodies.add(id);
    }
  };
  if (multiSel && multiHitAt(grab)) {
    multiSel.bodies.forEach((id) => bodies.add(id));
    multiSel.joints.forEach((id) => joints.add(id));
  } else if (selection?.kind === "body") {
    addBodyWithGroup(selection.id);
  } else if (selection?.kind === "joint") {
    const j = scene.getJoint(selection.id);
    if (!j) return false;
    if (j.bodyId !== null) {
      addBodyWithGroup(j.bodyId);
    } else {
      const g = selGroupOfJoint(j.id);
      if (g) {
        g.bodyIds.forEach((b) => bodies.add(b));
        g.jointIds.forEach((jt) => joints.add(jt));
      } else {
        joints.add(j.id);
      }
    }
  } else {
    return false;
  }
  // The driver grabs what's under the cursor: a joint of the moving set, or a point on
  // one of its bodies (driven like the sim-mode body grab).
  const j = scene.jointAt(grab, pickRadius());
  let drv: Driver;
  if (j && (j.bodyId === null ? joints.has(j.id) : bodies.has(j.bodyId))) {
    drv = { jointId: j.id, target: grab };
  } else {
    const b = scene.bodyAt(grab);
    if (!b || !bodies.has(b.id)) return false;
    drv = { bodyId: b.id, local: rotate(sub(grab, b.pos), -b.angle), target: grab };
  }
  // Freeze the rest of the scene: every body and free joint not being dragged.
  const freeze: SolveFreeze = {
    bodies: new Set(scene.bodies.filter((b) => !bodies.has(b.id)).map((b) => b.id)),
    joints: new Set(
      scene.joints.filter((jt) => jt.bodyId === null && !joints.has(jt.id)).map((jt) => jt.id)
    ),
  };
  leftDrag = { kind: "rigid", driver: drv, freeze, moved: false };
  canvas.style.cursor = "grabbing";
  return true;
}

/** Box-select result: bodies fully inside the rectangle, plus free joints inside it. */
function applyBoxSelect(): void {
  if (!boxSelect) return;
  const x0 = Math.min(boxSelect.start.x, boxSelect.end.x);
  const x1 = Math.max(boxSelect.start.x, boxSelect.end.x);
  const y0 = Math.min(boxSelect.start.y, boxSelect.end.y);
  const y1 = Math.max(boxSelect.start.y, boxSelect.end.y);
  const inside = (p: Vec2) => p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1;
  const bodies = new Set(boxSelect.additive && multiSel ? multiSel.bodies : []);
  const joints = new Set(boxSelect.additive && multiSel ? multiSel.joints : []);
  if (boxSelect.additive && selection?.kind === "body") bodies.add(selection.id);
  // Inside an open group the box only catches that group's own material.
  for (const b of scene.bodies) {
    if (bodyInScope(b.id) && scene.bodyWorldVerts(b).every(inside)) bodies.add(b.id);
  }
  for (const j of scene.joints) {
    if (j.bodyId === null && jointInScope(j.id) && inside(scene.jointWorld(j))) joints.add(j.id);
  }
  setMulti(bodies, joints);
}

/** Whether the current selection contains component-instance material (grouping and
 *  ungrouping don't apply to it — edit the definition instead). */
function selectionTouchesInstance(): boolean {
  if (multiSel) {
    for (const id of multiSel.bodies) if (scene.instanceOfBody(id)) return true;
    for (const id of multiSel.joints) if (scene.instanceOfJoint(id)) return true;
  }
  if (selection?.kind === "body" && scene.instanceOfBody(selection.id)) return true;
  if (selection?.kind === "joint" && scene.instanceOfJoint(selection.id)) return true;
  return false;
}

/** The single component instance the current selection consists of, or null when the
 *  selection is empty, plain material, or mixes an instance with anything else.
 *  (Instance selection is atomic, so a click on one selects exactly its members.) */
function selectionInstance(): ComponentInstance | null {
  const bodies = multiSel ? [...multiSel.bodies] : selection?.kind === "body" ? [selection.id] : [];
  const joints = multiSel ? [...multiSel.joints] : selection?.kind === "joint" ? [selection.id] : [];
  if (bodies.length + joints.length === 0) return null;
  let inst: ComponentInstance | null = null;
  for (const id of bodies) {
    const i = scene.instanceOfBody(id);
    if (!i || (inst && i.id !== inst.id)) return null;
    inst = i;
  }
  for (const id of joints) {
    const i = scene.instanceOfJoint(id);
    if (!i || (inst && i.id !== inst.id)) return null;
    inst = i;
  }
  return inst;
}

/** With 2+ members multi-selected: make (or extend) a permanent group over them
 *  (free joints become locked group members). */
function groupSelection(): void {
  if (mode !== "draw" || !multiSel) return;
  if (multiSel.bodies.size + multiSel.joints.size < 2) return;
  const g = scene.addGroup([...multiSel.bodies], [...multiSel.joints]);
  if (!g) return;
  multiSel = { bodies: new Set(g.bodyIds), joints: new Set(g.jointIds) };
  markDirty();
}

/** Dissolve every permanent group the current selection touches. */
function ungroupSelection(): void {
  if (mode !== "draw") return;
  const ids: number[] = [];
  const jids: number[] = [];
  if (multiSel) {
    ids.push(...multiSel.bodies);
    jids.push(...multiSel.joints);
  }
  if (selection?.kind === "body") ids.push(selection.id);
  if (selection?.kind === "joint") jids.push(selection.id);
  if ((ids.length || jids.length) && scene.ungroup(ids, jids)) markDirty();
}

/**
 * Ctrl+G: group / ungroup toggle. A multi-selection of 2+ members (bodies and free
 * joints) becomes a permanent group (merging any groups it touches) — unless it already
 * is exactly one group, which dissolves instead. With fewer members selected (a single
 * grouped element counts, since groups are selection-atomic) it ungroups; otherwise
 * it's a no-op. Component-instance material is excluded — its grouping belongs to the
 * definition.
 */
function toggleGroupSelection(): void {
  if (mode !== "draw" || selectionTouchesInstance()) return;
  if (multiSel && multiSel.bodies.size + multiSel.joints.size >= 2) {
    const first =
      multiSel.bodies.size > 0
        ? scene.groupOf([...multiSel.bodies][0])
        : scene.groupOfJoint([...multiSel.joints][0]);
    const isOneGroup =
      first !== undefined &&
      [...multiSel.bodies].every((id) => scene.groupOf(id)?.id === first.id) &&
      [...multiSel.joints].every((id) => scene.groupOfJoint(id)?.id === first.id);
    if (isOneGroup) ungroupSelection();
    else groupSelection();
    return;
  }
  ungroupSelection();
}

// --- canvas sizing -------------------------------------------------------
function resize(): void {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(canvas.clientWidth * dpr);
  canvas.height = Math.floor(canvas.clientHeight * dpr);
  if (viewRotate) placeViewAngleInput(); // the dial sits at the canvas centre
}
window.addEventListener("resize", resize);

/**
 * Fit the whole mechanism (body outlines, joints, ground anchors) in the canvas with a
 * screen-pixel margin, centered. An empty scene just recenters the world origin at scale 1.
 */
function fitView(): void {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  // Bounds are taken in the screen-aligned frame (world turned by the view angle), so a
  // rotated view fits the tilted picture rather than its world-axis box.
  const include = (wp: Vec2) => {
    const p = rotateToScreen(wp, view.angle);
    if (p.x < minX) minX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.x > maxX) maxX = p.x;
    if (p.y > maxY) maxY = p.y;
  };
  for (const b of scene.bodies) scene.bodyWorldVerts(b).forEach(include);
  for (const j of scene.joints) include(scene.jointWorld(j));
  for (const c of scene.constraints) if (c.kind === "ground") include(c.anchor);
  if (!Number.isFinite(minX)) {
    view.scale = 1;
    view.tx = w / 2;
    view.ty = h / 2;
    return;
  }
  const MARGIN = 60; // screen px kept clear around the mechanism
  const fit = Math.min(
    Math.max(w - 2 * MARGIN, 40) / Math.max(maxX - minX, 1e-6),
    Math.max(h - 2 * MARGIN, 40) / Math.max(maxY - minY, 1e-6)
  );
  view.scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, fit));
  // The box centre is already in the screen-aligned frame: offset it directly.
  view.tx = w / 2 - ((minX + maxX) / 2) * view.scale;
  view.ty = h / 2 - ((minY + maxY) / 2) * view.scale;
}

// --- view rotation dial -----------------------------------------------------
/** Snap step for the dial's drag (Shift bypasses it). */
const VIEW_ROTATE_SNAP = (5 * Math.PI) / 180;
/** Grab band around the ring and along the crosshair arms (screen px). */
const VIEW_ROTATE_GRAB_PX = 14;

function dialCentre(): Vec2 {
  return vec(canvas.clientWidth / 2, canvas.clientHeight / 2);
}
function dialRadius(): number {
  return Math.min(canvas.clientWidth, canvas.clientHeight) * 0.38;
}
/** The pointer (screen px) is on the dial's ring or on one of its crosshair arms. */
function dialHit(s: Vec2): boolean {
  const c = dialCentre();
  const d = sub(s, c);
  const onRing = Math.abs(Math.hypot(d.x, d.y) - dialRadius()) <= VIEW_ROTATE_GRAB_PX;
  // Arms run along the world axes: the pointer's offset expressed in world axes has one
  // component near zero when it sits on an arm.
  const w = rotateToWorld(d, view.angle); // undo the view turn → world-axis components
  const onArm = Math.abs(w.x) <= VIEW_ROTATE_GRAB_PX || Math.abs(w.y) <= VIEW_ROTATE_GRAB_PX;
  return onRing || onArm;
}
/** The pointer's bearing about the dial centre, counter-clockwise on screen (radians). */
function dialBearing(s: Vec2): number {
  const c = dialCentre();
  return -Math.atan2(s.y - c.y, s.x - c.x);
}
/** Turn the view to `angle` about the dial centre (the world point there stays put). */
function setViewAngle(angle: number): void {
  rotateViewTo(view, dialCentre(), wrapAngle(angle));
  syncViewAngleInput();
}
/** Toggle the dial. Opening it leaves the armed tool / draft alone (the dial captures
 *  the left button while it is up, so nothing underneath fires). */
function setViewRotateOpen(open: boolean): void {
  if (open === (viewRotate !== null)) return;
  viewRotate = open ? { drag: null, hot: false } : null;
  rotateViewBtn.classList.toggle("active", open);
  viewAngleInput.classList.toggle("hidden", !open);
  if (open) {
    placeViewAngleInput();
    syncViewAngleInput();
  } else viewAngleInput.blur();
  canvas.style.cursor = open ? "default" : defaultCursor();
  updateHint();
}
/** The angle box sits just under the dial's centre dot. */
function placeViewAngleInput(): void {
  const c = dialCentre();
  viewAngleInput.style.left = `${c.x}px`;
  viewAngleInput.style.top = `${c.y + 30}px`;
}
/** The readout follows the view unless the user is typing in it. */
function syncViewAngleInput(): void {
  if (!viewRotate || document.activeElement === viewAngleInput) return;
  const deg = (wrapAngle(view.angle) * 180) / Math.PI;
  const rounded = Math.round(deg * 10) / 10;
  viewAngleInput.value = `${Math.abs(rounded) < 0.05 ? 0 : rounded}°`;
}
function commitViewAngleInput(): void {
  const raw = viewAngleInput.value.trim().replace(/[°º]/g, "").replace(",", ".");
  const deg = Number(raw);
  if (raw !== "" && Number.isFinite(deg)) setViewAngle((deg * Math.PI) / 180);
  viewAngleInput.blur();
  syncViewAngleInput();
}
viewAngleInput.addEventListener("keydown", (e) => {
  e.stopPropagation(); // keep canvas shortcuts out of the field
  if (e.key === "Enter") commitViewAngleInput();
  else if (e.key === "Escape") {
    viewAngleInput.blur();
    syncViewAngleInput();
  }
});
viewAngleInput.addEventListener("focus", () => {
  viewAngleInput.value = viewAngleInput.value.replace(/[°º]/g, "");
  viewAngleInput.select();
});
viewAngleInput.addEventListener("blur", () => {
  if (viewRotate) commitViewAngleInput();
});

function eventScreen(e: MouseEvent): Vec2 {
  const rect = canvas.getBoundingClientRect();
  return vec(e.clientX - rect.left, e.clientY - rect.top);
}

function eventWorld(e: MouseEvent): Vec2 {
  return screenToWorld(view, eventScreen(e));
}

/** Joint/vertex pick radius in world units (constant on screen across zoom). */
const pickRadius = () => PICK_RADIUS / view.scale;

function defaultCursor(): string {
  if (tool === "measure") return "crosshair";
  return mode === "sim" ? "grab" : "crosshair";
}

// --- hint text -----------------------------------------------------------
const HINTS: Record<Mode | Tool | "select" | "viewRotate", string> = {
  draw: "",
  viewRotate: "Rotate the view: drag the ring or a crosshair arm to turn the whole picture (snaps to 5°, hold Shift for any angle) · type an exact angle in the box under the centre · double-click the centre for 0° · click elsewhere, Esc or Ctrl+R to close. The drawing itself does not change.",
  sim: "Drag any joint, or part of a body, to drive the mechanism. Space to run / pause actuators.",
  select: "Click to select · drag to move · Shift+drag to move rigidly (sim-style: grounds hold, connections constrain, the rest stays put) · Object snap (toolbar) drags by the highlighted corner / midpoint / edge / centre nearest the grab and snaps it onto other objects · Ctrl+click or drag a box to select several bodies (they move together) · Ctrl+G groups them permanently / ungroups a group · with a body selected, Shift+drag a box from empty space to select several of its corners / holes / joints (Ctrl+Shift adds) — drag any of them to move the set, Delete removes it, Ctrl+C copies its holes + joints (with their constraints) and Ctrl+V pastes them into the selected body at the cursor · drag a selected body's corner handles to reshape · drag a round handle to round just that corner (double-click it to reset to the body's radius) · double-click an edge to add a node / a node to remove it · double-click a dimension to set its value · double-click a component instance to edit its definition (Ctrl+double-click shows the surrounding assembly faded in its frame — a context ghost to snap and dimension to; the breadcrumb eyes set how far out it reaches) · [ and ] round all corners · N combines the selected bodies into one, Subtract / Intersect (Boolean group) cut the others out of the first-selected body / keep only their overlap · Delete to remove.",
  polyline: "Click each corner; click the first corner again, double-click or press Enter to finish. Body role: start on existing joints to build a body around them (click a picked joint again to finish, then move out to set the margin and click).",
  rect: "Click one corner, then the opposite corner — or press and drag. Shift for a square, Alt to draw from the centre.",
  circle: "Click the centre, then a point on the rim — or press and drag the radius out.",
  polygon: "Click the centre, then one corner — or press and drag. The side count shows beside the cursor: ↑ / ↓ change it (later, double-click the polygon's \"n sides\" tag).",
  slot: "Click where the slot starts and where it ends — or press and drag — then move out to set its width and click. (Reference role: the axis segment.)",
  line: "Click two points for a reference segment — or press and drag. Points land on joints, corners and edges (a coincident holds them there).",
  arc: "Click the arc's start and end points, then a point it should pass through.",
  text: "Click where the label goes (on a body: the label rides with it), type the text and press Enter. Size: the field in the toolbar's properties strip. Double-click a label later to edit it.",
  split: "Click a point on a body's outline (edge or corner) to start the cut, click inside to route it, then click the outline again to split the body in two along the path.",
  patternLinear: "Click a hole or a joint on a body to repeat it along a line — Ctrl+click to pick several features of one body and repeat them together.", // live stage hint: patternHint()
  patternCircular: "Click a hole or a joint on a body to repeat it around a centre — Ctrl+click to pick several features of one body and repeat them together.",
  joint: "Click inside a body to attach a joint, or empty space to place a free joint.",
  weld: "Click where bodies overlap to weld them rigidly together at that point (no relative rotation) — or click an existing pinned joint to toggle it weld ↔ pin.",
  connect: "Click a joint, then another joint to pin them — or a rail to attach the joint to it as a rider.",
  ground: "Click a joint to lock its position (it can still rotate), or a body / group to fix it entirely; click again to unground.",
  rail: "Click two joints on the same body (a moving rail) — or two free joints (a fixed track) — to create a rail that joints and sliders can ride along (a pin-in-slot: riders placed on it slide and rotate).",
  slider: "Click a body where the slider starts (that body is the part that moves), then click where the travel ends — over another body the track rides that body, otherwise it is fixed in the world. On an existing rail: click it to add a slider there, or click a rider to toggle its rotation lock.",
  rotate: "Drag a body to rotate it about its centroid, or drag a selected body's node to rotate about that node. A multi-selection or group rotates as one about its centre. Snaps to 45°.",
  linearActuator: "Click a slider or rail to make it self-driving — its carriage travels back and forth when animation runs (a rail with no rider gets a free one).",
  motor: "Click a joint to set the pivot, then another joint on the same body for the crank pin.",
  measure: "Click a line (body or hole edge, rail, reference segment) and then where its length should sit — or two references (joints, corners, edges, rails, reference points, a point on a body), then place the value. A round hole's rim or a rounded corner's arc as the first pick makes a diameter / radius dimension. Inside a component with the context ghost shown, a faded joint / corner / edge / rail makes a temporary dimension (double-click its value to move your geometry there).",
  coincident: "Click two points (joints, body corners, polygon centres or reference points) to make them share a position — or a point and a line (body edge, rail or reference segment) to hold the point on that line.",
  horizontal: "Click a body edge, rail or reference segment — or two points — to make it horizontal.",
  vertical: "Click a body edge, rail or reference segment — or two points — to make it vertical.",
  parallel: "Click two lines (body edges, rails or reference segments) to make them parallel.",
  perpendicular: "Click two lines (body edges, rails or reference segments) to make them perpendicular.",
  equal: "Click two lines (body edges, rails or reference segments) to make their lengths equal — or two circles / rounded corners (a disk body's rim, a round hole, a rounded corner's arc, a reference circle or arc) to make their radii equal. The second takes the first one's value.", // live stage hint: equalHint()
  fixed: "Click a point (joint, body corner, polygon centre or reference point) to lock it where it is — or a line (body edge, rail or reference segment) to lock the line itself: its ends can still slide along it and stretch it, but the line can never turn or shift.",
  symmetric: "Click two points (joints, body corners, polygon centres or reference points) — or two lines (body edges, rails or reference segments) — then the mirror line: the two become mirror images across it.", // live stage hint: symmetricHint()
  tangent: "Click a circle or arc (a disk body's rim, a round hole, a reference circle or arc) and a line (body edge, rail or reference segment), in either order: the line becomes tangent to the circle.", // live stage hint: tangentHint()
};

/** Stage hint for the Equal tool: what the second click is for, matching the first. */
function equalHint(): string {
  if (constraintPicks.length === 0) return HINTS.equal;
  return isLineRef(constraintPicks[0])
    ? "Equal: now click the second line (a body edge, rail or reference segment) — it takes the first one's length."
    : "Equal: now click the second circle or rounded corner (a disk's rim, a round hole, a rounded corner's arc, a reference circle) — it takes the first one's radius.";
}

/** Stage hint for the Tangential tool: what the second click is for. */
function tangentHint(): string {
  if (constraintPicks.length === 0) return HINTS.tangent;
  return isCircleRef(constraintPicks[0])
    ? "Tangential: now click the line (a body edge, rail or reference segment) to hold tangent to that circle."
    : "Tangential: now click the circle or arc (a disk body's rim, a round hole, a reference circle or arc) the line should touch.";
}

/** Stage hint for the Symmetrical tool: what the next click is for. */
function symmetricHint(): string {
  if (constraintPicks.length === 0) return HINTS.symmetric;
  const first = constraintPicks[0];
  const line = first.kind === "rail" || first.kind === "edge" || first.kind === "guideLine" || first.kind === "patternAxis";
  if (constraintPicks.length === 1) {
    return line
      ? "Symmetrical: now click the second line (a body edge, rail or reference segment), then the mirror line."
      : "Symmetrical: now click the second point (a joint, body corner, polygon centre or reference point), then the mirror line.";
  }
  return "Symmetrical: now click the mirror line (a body edge, rail or reference segment) the two are reflected across.";
}

/**
 * Attached joints stranded outside their body's outline (a component edit cascading
 * into instances, or a removed vertex, reshaped the body under them). Recomputed every
 * frame in draw mode; the renderer paints them red and the hint line warns while any
 * exist. Nothing is auto-moved — the user drags the joint back in (drags clamp to the
 * outline) or fixes the shape / definition.
 */
let containmentErrors: Set<number> = new Set();

function containmentWarning(): string {
  const n = containmentErrors.size;
  if (n === 0 || mode !== "draw") return "";
  const what = n === 1 ? "1 joint lies outside its body" : `${n} joints lie outside their bodies`;
  const fix = n === 1 ? "drag it back inside" : "drag them back inside";
  return `⚠ ${what} (red) — ${fix}, or fix the body / component shape. · `;
}

/**
 * A toast that doesn't stack up when the same thing is done twice: clicking the same
 * wrong element again should re-state the reason, not pile four copies of it up.
 */
let lastNotice = { text: "", at: -Infinity };
function notifyThrottled(text: string, kind: NotifyKind = "warn"): void {
  const now = performance.now();
  if (text === lastNotice.text && now - lastNotice.at < NOTICE_REPEAT_MS) return;
  lastNotice = { text, at: now };
  notify(text, kind);
}
const NOTICE_REPEAT_MS = 3000;

/** Status-bar line; the bar shows one line, so the whole text goes on its tooltip. */
function setHint(text: string): void {
  hintEl.textContent = text;
  hintEl.title = text;
}

/** Status-bar prefix while a group is open for editing. */
function groupScopeNote(): string {
  return groupEdit === null
    ? ""
    : "Inside a group — its parts select, move and reshape one by one; everything faded is out of reach. Esc (or a double-click outside) leaves it. · ";
}

function updateHint(): void {
  const base =
    viewRotate ? HINTS.viewRotate
    : tool === "measure" ? HINTS.measure
    : mode === "sim" ? HINTS.sim
    : tool === null ? HINTS.select
    : isPatternTool(tool) ? patternHint()
    : isShapeTool(tool) ? shapeHint()
    : tool === "symmetric" ? symmetricHint()
    : tool === "tangent" ? tangentHint()
    : tool === "equal" ? equalHint()
    : HINTS[tool];
  setHint(containmentWarning() + groupScopeNote() + base);
}

// --- toolbar wiring ------------------------------------------------------
document.querySelectorAll<HTMLButtonElement>(".mode-btn").forEach((btn) => {
  btn.addEventListener("click", () => setMode(btn.dataset.mode as Mode));
});
document.querySelectorAll<HTMLButtonElement>(".tool-btn").forEach((btn) => {
  btn.addEventListener("click", () => setTool(btn.dataset.tool as Tool));
});
document.querySelectorAll<HTMLButtonElement>(".role-btn").forEach((btn) => {
  btn.addEventListener("click", () => setRole(btn.dataset.role as ShapeRole));
});
// Sections are draggable by their caption; the order is remembered across sessions.
const toolbarApi = installToolbar({
  blocked: () => document.body.classList.contains("help-armed"),
  onReset: () => notify("Toolbar groups back in their default order."),
});
void toolbarApi;
textSizeInput.addEventListener("change", () => {
  const v = Number(textSizeInput.value);
  if (Number.isFinite(v) && v > 0) textSize = v;
  else textSizeInput.value = String(textSize);
});
/** Empty the document — in a definition context, only that definition's content (the
 *  document's component list survives); at the root, the whole document. */
function clearDocument(): void {
  if (mode === "sim") return;
  scene.clear(editPath.length === 0);
  if (editPath.length === 0) setDocFile(null, null); // a fresh document: Save asks where to put it
  resetTransient();
  markDirty();
  updateCompPanel();
}
document.getElementById("clear-btn")!.addEventListener("click", clearDocument);
document.getElementById("fit-btn")!.addEventListener("click", fitView);
rotateViewBtn.addEventListener("click", () => setViewRotateOpen(viewRotate === null));
document.getElementById("save-btn")!.addEventListener("click", (e) => void saveToFile(e.shiftKey));
document.getElementById("load-btn")!.addEventListener("click", () => void openFile());
// Copy/paste are keyboard-only (Ctrl/Cmd+C / V); no toolbar buttons.
document.getElementById("mirror-h-btn")!.addEventListener("click", () => mirrorSelection("h"));
document.getElementById("mirror-v-btn")!.addEventListener("click", () => mirrorSelection("v"));
document.getElementById("combine-btn")!.addEventListener("click", combineSelection);
document.getElementById("subtract-btn")!.addEventListener("click", () => booleanSelection("subtract"));
document.getElementById("intersect-btn")!.addEventListener("click", () => booleanSelection("intersect"));
document.getElementById("send-back-btn")!.addEventListener("click", () => reorderSelection("back"));
document.getElementById("bring-front-btn")!.addEventListener("click", () => reorderSelection("front"));

runBtn.addEventListener("click", () => setAnimating(!animating));
autopauseBtn.addEventListener("click", () => setPauseOnImpossible(!pauseOnImpossible));
animIterInput.addEventListener("input", () => {
  const n = parseInt(animIterInput.value, 10);
  if (Number.isFinite(n) && n > 0) {
    animIterations = n;
    animIterValue.textContent = String(n);
  }
});
cleanupMaxInput.addEventListener("input", () => {
  const n = parseInt(cleanupMaxInput.value, 10);
  if (Number.isFinite(n) && n >= 0) {
    solverConfig.maxCleanupSweeps = n;
    cleanupMaxValue.textContent = String(n);
  }
});
structTolInput.addEventListener("input", () => {
  const n = parseFloat(structTolInput.value);
  if (Number.isFinite(n) && n >= 0) solverConfig.structuralTol = n;
});
breakTolInput.addEventListener("input", () => {
  const n = parseFloat(breakTolInput.value);
  if (Number.isFinite(n) && n >= 0) solverConfig.breakTol = n;
});
// Seed the tuning controls from the actual runtime values (solverConfig / animIterations are
// the single source of truth — the HTML carries no defaults, so they can't drift apart).
animIterInput.value = String(animIterations);
animIterValue.textContent = String(animIterations);
cleanupMaxInput.value = String(solverConfig.maxCleanupSweeps);
cleanupMaxValue.textContent = String(solverConfig.maxCleanupSweeps);
structTolInput.value = String(solverConfig.structuralTol);
breakTolInput.value = String(solverConfig.breakTol);

// Inline speed / profile editing for whatever actuator or motor the selection identifies.
actuatorSpeedInput.addEventListener("input", () => {
  const a = selectedLinearActuator();
  if (!a) return;
  const v = Number(actuatorSpeedInput.value);
  if (Number.isFinite(v) && v >= 0) {
    a.speed = v;
    markDirty();
  }
});
motorSpeedInput.addEventListener("input", () => {
  const m = selectedMotor();
  if (!m) return;
  const v = Number(motorSpeedInput.value);
  if (Number.isFinite(v) && v >= 0) {
    m.speed = v;
    markDirty();
  }
});
profileToggle.querySelectorAll<HTMLButtonElement>("button").forEach((btn) => {
  btn.addEventListener("click", () => {
    const a = selectedLinearActuator();
    if (!a) return;
    a.profile = (btn.dataset.profile === "sine" ? "sine" : "triangle");
    syncPropsPanel();
    markDirty();
  });
});

function toggleGrid(): void {
  gridVisible = !gridVisible;
  gridBtn.classList.toggle("active", gridVisible);
}
function toggleSnap(): void {
  snapEnabled = !snapEnabled;
  snapBtn.classList.toggle("active", snapEnabled);
}
function toggleObjectSnap(): void {
  objSnapEnabled = !objSnapEnabled;
  osnapBtn.classList.toggle("active", objSnapEnabled);
}
gridBtn.addEventListener("click", toggleGrid);
snapBtn.addEventListener("click", toggleSnap);
osnapBtn.addEventListener("click", toggleObjectSnap);
autoConBtn.addEventListener("click", () => setAutoConstrain(!autoConstrain));
sketchVisBtn.addEventListener("click", () => setSketchVisible(!sketchVisible));
measureVisBtn.addEventListener("click", () => setMeasureVisible(!measureVisible));
const GRID_MIN = 1;
const GRID_MAX = 200;
/** Built-in grid sizes; the user's own additions are appended and kept in localStorage. */
const GRID_BASE_PRESETS = [1, 2, 5, 10, 20, 25, 40, 50, 100, 200];
const GRID_PRESETS_KEY = "disjointed:gridPresets";
let gridCustomPresets: number[] = (() => {
  try {
    const raw = localStorage.getItem(GRID_PRESETS_KEY);
    const arr = raw ? (JSON.parse(raw) as unknown) : [];
    return Array.isArray(arr)
      ? arr.filter((n): n is number => typeof n === "number" && Number.isFinite(n) && n >= GRID_MIN && n <= GRID_MAX)
      : [];
  } catch {
    return [];
  }
})();
function saveGridPresets(): void {
  try {
    localStorage.setItem(GRID_PRESETS_KEY, JSON.stringify(gridCustomPresets));
  } catch {
    /* storage unavailable — presets just don't persist */
  }
}
/** Short label for a grid size (trailing zeros trimmed). */
function fmtGrid(n: number): string {
  return String(Math.round(n * 1000) / 1000);
}
function setGridStep(n: number): void {
  gridStep = Math.min(GRID_MAX, Math.max(GRID_MIN, n));
  gridSizeValue.textContent = fmtGrid(gridStep);
  if (!gridSizeMenu.classList.contains("hidden")) renderGridSizeList();
}
/** Rebuild the preset list: base presets, then custom ones (removable), sorted ascending. */
function renderGridSizeList(): void {
  gridSizeList.replaceChildren();
  const all = [...GRID_BASE_PRESETS, ...gridCustomPresets].sort((a, b) => a - b);
  for (const n of all) {
    const row = document.createElement("div");
    row.className = "combo-item";
    row.setAttribute("role", "option");
    row.setAttribute("aria-selected", String(n === gridStep));
    if (n === gridStep) row.classList.add("selected");
    const pick = document.createElement("button");
    pick.type = "button";
    pick.className = "combo-pick";
    pick.textContent = fmtGrid(n);
    pick.addEventListener("click", () => {
      setGridStep(n);
      closeGridSizeMenu();
    });
    row.appendChild(pick);
    if (!GRID_BASE_PRESETS.includes(n)) {
      row.classList.add("custom");
      const rm = document.createElement("button");
      rm.type = "button";
      rm.className = "combo-remove";
      rm.title = "Remove this custom size from the list";
      rm.setAttribute("aria-label", `Remove grid size ${fmtGrid(n)}`);
      rm.textContent = "×";
      rm.addEventListener("click", (e) => {
        e.stopPropagation();
        gridCustomPresets = gridCustomPresets.filter((v) => v !== n);
        saveGridPresets();
        renderGridSizeList();
      });
      row.appendChild(rm);
    }
    gridSizeList.appendChild(row);
  }
}
function openGridSizeMenu(): void {
  closeGridStyleMenu(); // one open combo at a time (they are neighbours in the group)
  renderGridSizeList();
  gridSizeMenu.classList.remove("hidden");
  gridSizeBtn.setAttribute("aria-expanded", "true");
  gridSizeBtn.classList.add("active");
  gridSizeNew.value = "";
  // Scroll the current value into view, then hand focus to the custom field.
  gridSizeList.querySelector(".selected")?.scrollIntoView({ block: "nearest" });
  gridSizeNew.focus();
}
function closeGridSizeMenu(): void {
  if (gridSizeMenu.classList.contains("hidden")) return;
  gridSizeMenu.classList.add("hidden");
  gridSizeBtn.setAttribute("aria-expanded", "false");
  gridSizeBtn.classList.remove("active");
}
function toggleGridSizeMenu(): void {
  if (gridSizeMenu.classList.contains("hidden")) openGridSizeMenu();
  else closeGridSizeMenu();
}
gridSizeBtn.addEventListener("click", toggleGridSizeMenu);
// Adding a custom value applies it immediately and (if new) keeps it in the list.
gridSizeAddForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const n = Number(gridSizeNew.value);
  if (gridSizeNew.value.trim() === "" || !Number.isFinite(n)) return;
  const v = Math.min(GRID_MAX, Math.max(GRID_MIN, n));
  if (!GRID_BASE_PRESETS.includes(v) && !gridCustomPresets.includes(v)) {
    gridCustomPresets.push(v);
    saveGridPresets();
  }
  setGridStep(v);
  closeGridSizeMenu();
});
gridSizeMenu.addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    e.stopPropagation();
    closeGridSizeMenu();
    gridSizeBtn.focus();
  }
});
// Click anywhere outside the combo dismisses it.
document.addEventListener("pointerdown", (e) => {
  if (gridSizeMenu.classList.contains("hidden")) return;
  const t = e.target;
  if (t instanceof Node && (gridSizeMenu.contains(t) || gridSizeBtn.contains(t))) return;
  closeGridSizeMenu();
});

const fileInput = document.getElementById("file-input") as HTMLInputElement;
fileInput.addEventListener("change", () => {
  const file = fileInput.files?.[0];
  if (file) loadFromFile(file);
  fileInput.value = ""; // allow re-loading the same file later
});

function setMode(next: Mode): void {
  if (next === mode) return;
  resetTransient();
  if (next === "sim") {
    leaveGroup(); // in simulation a group *is* one rigid body again
    savedPoses = scene.snapshotPoses();
    resetPoseBaselines(); // slider locks + welds capture the drawn relative angles afresh
    timedSolve("settle", null, 40); // settle so pins/grounds/rails are satisfied
  } else if (savedPoses) {
    scene.restorePoses(savedPoses); // restore the drawn layout for editing
    savedPoses = null;
    solveBreaks = []; // leaving sim: clear any impossible-assembly markers
  }
  mode = next;
  // Animation is sim-only; always start sim with it off so dragging-to-drive works first.
  setAnimating(false);
  syncModeToggle();
  for (const s of drawOnlySections) s.classList.toggle("hidden", mode === "sim");
  for (const s of simOnlySections) s.classList.toggle("hidden", mode === "draw");
  animIterCtrl.classList.toggle("hidden", mode === "draw");
  cleanupMaxCtrl.classList.toggle("hidden", mode === "draw");
  structTolCtrl.classList.toggle("hidden", mode === "draw");
  breakTolCtrl.classList.toggle("hidden", mode === "draw");
  canvas.style.cursor = mode === "sim" ? "grab" : "crosshair";
  updateHint();
  updateSimError(); // show/hide the banner for the mode we just entered
  syncPropsPanel(); // selection cleared → properties panels hide
}

/** The mode toggle shows — and switches to — the mode you are *not* in; its caption names the one you are. */
function syncModeToggle(): void {
  const other = mode === "draw" ? "sim" : "draw";
  modeToggle.dataset.mode = other;
  applyShortcutTitle(
    modeToggle,
    other === "sim" ? "Simulate mode — drag to drive the mechanism" : "Draw mode — build the mechanism"
  );
  modeToggle.setAttribute("aria-label", other === "sim" ? "Switch to simulate mode" : "Switch to draw mode");
  modeCap.textContent = mode === "draw" ? "Draw" : "Simulate";
}

function setTool(next: Tool): void {
  // Rotate operates on the current selection, so keep an existing body selection (lets
  // you grab one of its control nodes as the pivot right away) — or multi-selection /
  // group (so R then drag rotates the whole set) — when arming it.
  // A shape tool armed in the Cut role keeps a selected body too: it is the cut's target.
  const keepSel =
    (next === "rotate" || (isShapeTool(next) && effectiveRole(next) === "cut")) && selection?.kind === "body"
      ? selection
      : null;
  const keepMulti = next === "rotate" ? multiSel : null;
  // Pattern works selection-first too: an already selected joint becomes the seed.
  const seedJoint = isPatternTool(next) && selection?.kind === "joint" ? selection.id : null;
  tool = next;
  resetTransient();
  selection = keepSel;
  multiSel = keepMulti;
  if (seedJoint !== null) togglePatternSeed({ kind: "joint", jointId: seedJoint }, false);
  document.querySelectorAll<HTMLButtonElement>(".tool-btn").forEach((b) =>
    b.classList.toggle("active", b.dataset.tool === tool)
  );
  syncRoleButtons();
  updateHint();
}

/** Return to normal/select mode after a tool finishes placing one element. */
function disarmTool(): void {
  tool = null;
  resetTransient();
  document
    .querySelectorAll<HTMLButtonElement>(".tool-btn")
    .forEach((b) => b.classList.remove("active"));
  syncRoleButtons();
  updateHint();
}

function resetTransient(): void {
  closeDimEditor();
  closeTextEditor();
  draftBody = [];
  draftBodySnaps = [];
  clearShapeDraft();
  patternSeeds = [];
  patternDraft = null;
  splitDraft = [];
  splitBodyId = null;
  constraintPicks = [];
  // Discard any slider-rail joints made for an unfinished body-from-joints draft (a finished
  // build clears this list first, so its absorbed joints survive).
  for (const id of jointDraftCreated) scene.removeJoint(id);
  jointDraftCreated = [];
  jointDraftIds = [];
  jointDraftExpanding = false;
  selectedJoint = null;
  railDraftIds = [];
  sliderDraft = null;
  motorPivotDraft = null;
  measurePicks = [];
  selection = null;
  multiSel = null;
  boxSelect = null;
  featureSel = null;
  featureBox = null;
  driver = null;
  rotateDrag = null;
  pendingInsert = null;
}

// --- persistence (save / load / autosave) --------------------------------
const AUTOSAVE_KEY = "disjointed:autosave:v1";
let autosaveTimer: number | undefined;

/**
 * Snapshot of the *current editing context* (drawn layout: in simulation the pre-sim
 * poses are restored, so a simulated configuration is never captured).
 */
function contextData(): SceneData {
  const data = scene.serializeContext();
  if (savedPoses) {
    for (const b of data.bodies) {
      const s = savedPoses.get(b.id);
      if (s) {
        b.pos = { x: s.pos.x, y: s.pos.y };
        b.angle = s.angle;
      }
    }
  }
  return data;
}

/**
 * While a component definition is being edited: store the live context into its def,
 * cascade the change through every definition that uses it, and refresh the stashed
 * root context — so the document stays consistent on every mutation and instances
 * everywhere follow the definition ("cascade update").
 */
function syncComponentContext(): void {
  if (editPath.length === 0) return;
  const def = scene.getComponent(editPath[editPath.length - 1]);
  if (!def) return;
  def.data = contextData();
  const changed = cascadeComponentChange(scene.components, [def.id]);
  if (rootData) rootData = reexpandData(rootData, scene.components, changed);
}

/**
 * Canonical document to persist: the ROOT context plus the component definitions. In a
 * definition context the root snapshot is `rootData` (kept fresh by markDirty's sync).
 */
function canonicalData(): SceneData {
  const root = editPath.length === 0 ? contextData() : (rootData as SceneData);
  return { ...root, components: scene.components };
}

/** Debounced autosave of the drawn layout to localStorage. */
function scheduleAutosave(): void {
  clearTimeout(autosaveTimer);
  autosaveTimer = window.setTimeout(() => {
    try {
      localStorage.setItem(AUTOSAVE_KEY, JSON.stringify(canonicalData()));
    } catch {
      /* storage unavailable / full — ignore */
    }
  }, 300);
}

// --- undo / redo (snapshot history of the drawn layout) ------------------
const HISTORY_LIMIT = 100;
/** JSON document snapshots + the editing path they were taken in (and the instances
 *  each level was entered through, so undo / redo restore the same context ghost). */
const history: { snap: string; path: number[]; via: (number | null)[] }[] = [];
let historyIndex = -1;

/** Record the current state as a history step (deduped) and drop the redo branch. */
function pushHistory(): void {
  const snap = JSON.stringify(canonicalData());
  const cur = history[historyIndex];
  if (cur && cur.snap === snap && cur.path.length === editPath.length && cur.path.every((d, i) => d === editPath[i])) {
    return; // nothing actually changed
  }
  history.splice(historyIndex + 1); // discard any redo entries past the current point
  history.push({ snap, path: [...editPath], via: [...viaStack] });
  if (history.length > HISTORY_LIMIT) history.shift();
  historyIndex = history.length - 1;
}

/** A scene mutation occurred: sync any open component definition (cascading the change
 *  everywhere), record an undo step and schedule an autosave. */
function markDirty(): void {
  // The drawn layout changed, so any captured slider-lock baselines are stale: the next
  // solve (sim entry, or a rigid Shift-drag) re-locks the relative angles as now drawn.
  resetPoseBaselines();
  syncComponentContext();
  invalidateGhost(); // the cascade refreshed the enclosing snapshots the ghost is built from
  pushHistory();
  scheduleAutosave();
  updateCompPanel();
  setDocModified(true);
  armBackupTimer();
}

/** Load a whole document and re-enter the given editing path (root when empty); `via`
 *  names the instance each level was entered through (context ghost placement). */
function setDocument(doc: SceneData, path: number[], via: (number | null)[] = []): void {
  resetPoseBaselines(); // new document, new drawn poses — stale baselines must go
  scene.load(doc); // validates; loads the root context + component definitions
  editPath = [];
  viaStack = [];
  tempDims = []; // temporary context dimensions don't survive a document swap
  rootData = null;
  savedViews.length = 0;
  for (const defId of path) {
    const def = scene.getComponent(defId);
    if (!def) break;
    if (editPath.length === 0) rootData = scene.serializeContext();
    viaStack.push(via[editPath.length] ?? null);
    tempDims.push([]);
    editPath.push(defId);
    savedViews.push({ ...view });
    scene.loadContext(def.data);
  }
  invalidateGhost();
  resetTransient(); // selection / drafts may reference ids that no longer exist
  updateCrumbBar();
  updateCompPanel();
  scheduleAutosave();
  setDocModified(true); // undo / redo / load: the on-disk file no longer matches
  armBackupTimer();
}

// Undo / redo apply to the drawn layout only (draw mode), not a running simulation.
function undo(): void {
  if (mode !== "draw" || historyIndex <= 0) return;
  historyIndex--;
  const e = history[historyIndex];
  setDocument(JSON.parse(e.snap) as SceneData, e.path, e.via);
}

function redo(): void {
  if (mode !== "draw" || historyIndex >= history.length - 1) return;
  historyIndex++;
  const e = history[historyIndex];
  setDocument(JSON.parse(e.snap) as SceneData, e.path, e.via);
}

/** Offer `text` as a file download named `name`. */
function downloadText(text: string, name: string, mime: string): void {
  const blob = new Blob([text], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  URL.revokeObjectURL(url);
}

function timeStamp(): string {
  return new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");
}

// --- the document's file (Ctrl+S) -------------------------------------------
/**
 * Where Ctrl+S writes. In Chromium browsers the File System Access API gives a real
 * file handle: the first save (or Save as…) asks for a location, later saves overwrite
 * that file silently, and Load / drop bind the loaded file the same way. The handle is
 * remembered in IndexedDB so a reload (which restores the localStorage autosave) keeps
 * the binding. Without the API (Firefox, Safari) a save is a download named after the
 * document with a timestamp, as before.
 */
const DOC_HANDLE_KEY = "document";
const saveBtn = document.getElementById("save-btn") as HTMLButtonElement;
let docHandle: FSFileHandle | null = null;
/** Display name of the document's file (also set for files loaded without a handle). */
let docName: string | null = null;
/** Changed since the last save to / load from a file (title shows a bullet). */
let docModified = false;

/** The document as saved to disk (pretty JSON, same for saves and backups). */
function documentText(): string {
  return JSON.stringify(canonicalData(), null, 2);
}

/** File-name stem of the document: its file's name without `.json`, else `mechanism`. */
function docStem(): string {
  const base = (docName ?? "")
    .replace(/\.json$/i, "")
    .replace(/[\\/:*?"<>|]+/g, "_")
    .trim();
  return base || "mechanism";
}

function setDocFile(handle: FSFileHandle | null, name: string | null): void {
  docHandle = handle;
  docName = name ?? handle?.name ?? null;
  if (fsSupported()) void storeHandle(DOC_HANDLE_KEY, handle);
  updateDocTitle();
}

function setDocModified(on: boolean): void {
  if (docModified === on) return;
  docModified = on;
  updateDocTitle();
}

function updateDocTitle(): void {
  document.title = `${docModified ? "• " : ""}${docName ?? "Untitled"} — Disjointed`;
  const target = docHandle ? `to ${docHandle.name}` : "mechanism to a file";
  applyShortcutTitle(saveBtn, `Save ${target} — Shift-click to save as a new file`);
}

/** The document on disk now matches `text`: clear the modified mark, nothing to back up. */
function markSaved(text: string): void {
  lastBackupText = text;
  clearBackupTimer(); // the file is current — the next change starts a new countdown
  refreshBackupPanel();
  setDocModified(false);
}

/**
 * Save the document. Writes the bound file in place; `saveAs` (or no bound file, or
 * lost access to it) asks for a location first. Falls back to a download.
 */
async function saveToFile(saveAs = false): Promise<void> {
  const text = documentText();
  if (!fsSupported()) {
    downloadText(text, `${docStem()}-${timeStamp()}.json`, "application/json");
    markSaved(text);
    return;
  }
  let handle = saveAs ? null : docHandle;
  try {
    if (handle && !(await ensurePermission(handle, true))) handle = null; // access lost — pick again
    if (!handle) {
      handle = await pickSaveFile(docName ?? `mechanism-${timeStamp()}.json`);
      if (!handle) return; // cancelled
    }
    await writeFile(handle, text);
  } catch (err) {
    notify(`Could not save${handle ? ` ${handle.name}` : ""}: ${(err as Error).message}`, "error");
    return;
  }
  setDocFile(handle, null);
  markSaved(text);
  notify(`Saved ${handle.name}`, "info");
}

/** Load a document via the file picker (binding the file for Ctrl+S when possible). */
async function openFile(): Promise<void> {
  if (!fsSupported()) {
    fileInput.click();
    return;
  }
  try {
    const handle = await pickOpenFile();
    if (!handle) return; // cancelled
    await loadFromFile(await handle.getFile(), handle);
  } catch (err) {
    notify(`Could not open file: ${(err as Error).message}`, "error");
  }
}

// --- cut-file export (src/export.ts) -----------------------------------------
const exportPanel = document.getElementById("export-panel")!;
const exportBtn = document.getElementById("export-btn") as HTMLButtonElement;
const exportScopeLabel = document.getElementById("export-scope")!;
const exportJointHoles = document.getElementById("export-joint-holes") as HTMLInputElement;
const exportJointDia = document.getElementById("export-joint-dia") as HTMLInputElement;
const exportJointUnit = document.getElementById("export-joint-unit")!;

/** The bodies an export covers: the selected one(s), or every body when nothing is selected. */
function exportTargets(): Body[] {
  if (selection?.kind === "body") {
    const b = scene.getBody(selection.id);
    return b ? [b] : [];
  }
  if (multiSel && multiSel.bodies.size) {
    return scene.bodies.filter((b) => multiSel!.bodies.has(b.id));
  }
  return scene.bodies;
}

function setExportPanelVisible(on: boolean): void {
  if (on) {
    setBackupPanelVisible(false); // the two panels share the corner
    const targets = exportTargets();
    const all = targets.length === scene.bodies.length;
    exportScopeLabel.textContent =
      targets.length === 0
        ? "No bodies to export"
        : `${targets.length} ${targets.length === 1 ? "body" : "bodies"} (${all ? "all" : "selected"}), in ${scene.unit}`;
    exportJointUnit.textContent = scene.unit;
    exportJointDia.disabled = !exportJointHoles.checked;
  }
  exportPanel.classList.toggle("hidden", !on);
  exportBtn.classList.toggle("active", on);
}

/**
 * File stem of an export. The bodies' **component** is the single instance they all
 * belong to (in any context), else the definition being edited (`editPath`), else none.
 * With a component: its name when the export covers every body of it (a whole instance,
 * or "export all" inside a definition), `<component>-body_<n>` for one body of several
 * (n = its position in the component), `<component>-bodies` for a partial selection.
 * Without one: `body` for a single body, `bodies` otherwise.
 */
function exportFileStem(targets: Body[]): string {
  const safe = (name: string) => name.trim().replace(/[^\w\-]+/g, "_").replace(/^_+|_+$/g, "") || "component";
  let comp: { name: string; bodyIds: number[] } | null = null;
  const insts = targets.map((b) => scene.instanceOfBody(b.id));
  if (insts[0] && insts.every((i) => i === insts[0])) {
    comp = { name: scene.getComponent(insts[0].defId)?.name ?? `component_${insts[0].defId}`, bodyIds: insts[0].bodyMap.map((e) => e.id) };
  } else if (editPath.length) {
    const defId = editPath[editPath.length - 1];
    comp = { name: scene.getComponent(defId)?.name ?? "component", bodyIds: scene.bodies.map((b) => b.id) };
  }
  if (!comp) return targets.length === 1 ? "body" : "bodies";
  const members = comp.bodyIds.filter((id) => scene.getBody(id));
  const name = safe(comp.name);
  if (targets.every((b) => members.includes(b.id)) && members.every((id) => targets.some((b) => b.id === id))) return name;
  if (targets.length === 1) return `${name}-body_${members.indexOf(targets[0].id) + 1}`;
  return `${name}-bodies`;
}

function runExport(): void {
  const targets = exportTargets();
  if (!targets.length) {
    notify("Nothing to export — draw a body first.");
    return;
  }
  const picked = document.querySelector('input[name="export-format"]:checked') as HTMLInputElement | null;
  const format = picked?.value ?? "dxf";
  const dia = exportJointHoles.checked ? Math.max(0, parseFloat(exportJointDia.value) || 0) : 0;
  const sheet = collectCutSheet(scene, targets, { jointHoleDiameter: dia });
  const stem = `${exportFileStem(targets)}-${timeStamp()}`;
  if (format === "svg") downloadText(toSvg(sheet), `${stem}.svg`, "image/svg+xml");
  else downloadText(toDxf(sheet), `${stem}.dxf`, "application/dxf");
  setExportPanelVisible(false);
  const holes = sheet.joints.length ? `, ${sheet.joints.length} joint hole${sheet.joints.length === 1 ? "" : "s"}` : "";
  notify(`Exported ${targets.length} ${targets.length === 1 ? "body" : "bodies"} as ${format.toUpperCase()}${holes}.`, "info");
}

exportBtn.addEventListener("click", () => setExportPanelVisible(exportPanel.classList.contains("hidden")));
document.getElementById("export-cancel")!.addEventListener("click", () => setExportPanelVisible(false));
document.getElementById("export-go")!.addEventListener("click", runExport);
exportJointHoles.addEventListener("change", () => {
  exportJointDia.disabled = !exportJointHoles.checked;
});
exportPanel.addEventListener("keydown", (e) => {
  e.stopPropagation(); // keep canvas shortcuts out of the panel's fields
  if (e.key === "Escape") setExportPanelVisible(false);
  else if (e.key === "Enter" && e.target instanceof HTMLInputElement) runExport();
});

// --- auto-backup: timestamped copies into a folder ---------------------------------
/**
 * The first change after a backup (or save) arms a one-shot timer; `intervalMin`
 * minutes later `<stem>-backup-<timestamp>.json` is written into the chosen folder and
 * that document's older backups are pruned down to `keep` (0 = keep all). Saving cancels
 * a pending backup (the file is up to date). Needs a folder handle
 * (File System Access API — Chromium); the panel explains when that's unavailable.
 * Settings live in localStorage, the folder handle in IndexedDB.
 */
const BACKUP_SETTINGS_KEY = "disjointed:backup";
const BACKUP_DIR_KEY = "backupDir";
const BACKUP_INTERVALS = [1, 2, 5, 10, 15, 30];
const BACKUP_KEEPS = [0, 5, 10, 20, 50];

interface BackupSettings {
  enabled: boolean;
  intervalMin: number;
  keep: number;
}

const backupPanel = document.getElementById("backup-panel")!;
const backupBtn = document.getElementById("backup-btn") as HTMLButtonElement;
const backupEnabledInput = document.getElementById("backup-enabled") as HTMLInputElement;
const backupIntervalSelect = document.getElementById("backup-interval") as HTMLSelectElement;
const backupKeepSelect = document.getElementById("backup-keep") as HTMLSelectElement;
const backupFolderEl = document.getElementById("backup-folder")!;
const backupChooseBtn = document.getElementById("backup-choose") as HTMLButtonElement;
const backupStatusEl = document.getElementById("backup-status")!;
const backupNextEl = document.getElementById("backup-next")!;
const backupNowBtn = document.getElementById("backup-now") as HTMLButtonElement;

let backupSettings: BackupSettings = loadBackupSettings();
let backupDir: FSDirectoryHandle | null = null;
/** Read/write permission on `backupDir` is currently granted. */
let backupDirOk = false;
/** Pending one-shot backup (armed by the first change since the last backup / save). */
let backupTimer: number | undefined;
/** Document text of the last backup or save — while unchanged there's nothing to back up. */
let lastBackupText = "";
/** Last outcome, shown in the panel. */
let backupStatus = "";
/** When the pending backup fires (ms epoch), while one is armed. */
let backupNextAt: number | null = null;
let backupBusy = false;

function loadBackupSettings(): BackupSettings {
  const def: BackupSettings = { enabled: false, intervalMin: 5, keep: 10 };
  try {
    const raw = localStorage.getItem(BACKUP_SETTINGS_KEY);
    if (!raw) return def;
    const p = JSON.parse(raw) as Partial<BackupSettings>;
    return {
      enabled: p.enabled === true,
      intervalMin: BACKUP_INTERVALS.includes(p.intervalMin as number) ? (p.intervalMin as number) : def.intervalMin,
      keep: BACKUP_KEEPS.includes(p.keep as number) ? (p.keep as number) : def.keep,
    };
  } catch {
    return def;
  }
}

function saveBackupSettings(): void {
  try {
    localStorage.setItem(BACKUP_SETTINGS_KEY, JSON.stringify(backupSettings));
  } catch {
    /* storage unavailable — settings are session-only */
  }
}

/** Backups can actually run: enabled, with a folder we may write to. */
function backupActive(): boolean {
  return backupSettings.enabled && backupDir !== null && backupDirOk;
}

function clearBackupTimer(): void {
  clearTimeout(backupTimer);
  backupTimer = undefined;
  backupNextAt = null;
}

/** A change happened: start the countdown to a backup, unless one is already pending. */
function armBackupTimer(): void {
  if (backupTimer !== undefined || !backupActive()) return;
  const period = backupSettings.intervalMin * 60_000;
  backupNextAt = Date.now() + period;
  backupTimer = window.setTimeout(() => {
    backupTimer = undefined;
    backupNextAt = null;
    void runBackup(false).then(refreshBackupPanel);
  }, period);
  refreshBackupPanel();
}

/** Settings / folder changed: drop any pending countdown and, if the document already
 *  differs from the last backup, start a fresh one from now. */
function restartBackupTimer(): void {
  clearBackupTimer();
  if (backupActive() && documentText() !== lastBackupText) armBackupTimer();
  else refreshBackupPanel();
}

/** Write a backup now. `manual` (the panel button) also backs up an unchanged document
 *  and may prompt for folder access; the timer stays silent and skips unchanged ones. */
async function runBackup(manual: boolean): Promise<void> {
  if (backupBusy) return;
  if (!backupDir) {
    if (manual) notify("Choose a backup folder first.");
    return;
  }
  const text = documentText();
  if (!manual && text === lastBackupText) return; // nothing changed since the last backup / save
  backupBusy = true;
  try {
    if (!(await ensurePermission(backupDir, manual))) {
      backupDirOk = false;
      restartBackupTimer();
      setBackupStatus(`Access to “${backupDir.name}” needs to be re-granted — click “Allow access…”.`);
      return;
    }
    backupDirOk = true;
    const name = `${docStem()}-backup-${timeStamp()}.json`;
    await writeToDirectory(backupDir, name, text);
    lastBackupText = text;
    await pruneBackups();
    setBackupStatus(`Last backup ${new Date().toLocaleTimeString()} — ${name}`);
    if (manual) notify(`Backed up to ${backupDir.name}/${name}`, "info");
  } catch (err) {
    setBackupStatus(`Backup failed: ${(err as Error).message}`);
    if (manual) notify(`Backup failed: ${(err as Error).message}`, "error");
  } finally {
    backupBusy = false;
  }
}

/** Delete this document's oldest backups beyond the keep count (timestamps sort by name). */
async function pruneBackups(): Promise<void> {
  if (!backupDir || backupSettings.keep <= 0) return;
  const prefix = `${docStem()}-backup-`;
  const names = (await listFiles(backupDir)).filter((n) => n.startsWith(prefix) && n.endsWith(".json")).sort();
  for (const n of names.slice(0, Math.max(0, names.length - backupSettings.keep))) {
    try {
      await backupDir.removeEntry(n);
    } catch {
      /* leave it — pruning is best-effort */
    }
  }
}

function setBackupStatus(text: string): void {
  backupStatus = text;
  refreshBackupPanel();
}

/** Tint the toolbar button while backups are wanted but can't run. */
function refreshBackupButton(): void {
  backupBtn.classList.toggle("paused", backupSettings.enabled && !backupActive());
}

function refreshBackupPanel(): void {
  refreshBackupButton();
  if (backupPanel.classList.contains("hidden")) return;
  const supported = fsDirectorySupported();
  backupEnabledInput.checked = backupSettings.enabled;
  backupIntervalSelect.value = String(backupSettings.intervalMin);
  backupKeepSelect.value = String(backupSettings.keep);
  backupEnabledInput.disabled = backupIntervalSelect.disabled = backupKeepSelect.disabled = !supported;
  backupChooseBtn.disabled = backupNowBtn.disabled = !supported;
  backupFolderEl.textContent = backupDir ? backupDir.name : "none chosen";
  backupFolderEl.title = backupDir ? backupDir.name : "";
  backupChooseBtn.textContent = backupDir ? (backupDirOk ? "Change…" : "Allow access…") : "Choose…";
  let status: string;
  if (!supported) status = "Not available in this browser — auto-backup needs the File System Access API (Chrome, Edge, Opera).";
  else if (backupStatus) status = backupStatus;
  else if (backupActive()) status = `Waiting for changes — a backup is written ${backupSettings.intervalMin} min after the first change.`;
  else if (!backupSettings.enabled) status = "Off. Backups are named <file>-backup-<date-time>.json.";
  else if (!backupDir) status = "Choose a folder to start backing up.";
  else status = `Access to “${backupDir.name}” needs to be re-granted — click “Allow access…”.`;
  backupStatusEl.textContent = status;
  backupNextEl.textContent = backupNextAt !== null ? `Next backup at ${new Date(backupNextAt).toLocaleTimeString()}.` : "";
  backupNextEl.hidden = backupNextAt === null;
}

function setBackupPanelVisible(on: boolean): void {
  if (on) setExportPanelVisible(false); // the two panels share the corner
  backupPanel.classList.toggle("hidden", !on);
  backupBtn.classList.toggle("active", on);
  if (on) refreshBackupPanel();
}

/** Pick (or re-authorize) the backup folder; a user gesture is required. */
async function chooseBackupFolder(): Promise<void> {
  try {
    if (backupDir && !backupDirOk && (await ensurePermission(backupDir, true))) {
      backupDirOk = true;
    } else {
      const dir = await pickDirectory();
      if (!dir) return; // cancelled
      backupDir = dir;
      backupDirOk = true;
      void storeHandle(BACKUP_DIR_KEY, dir);
    }
    backupStatus = "";
    restartBackupTimer();
    refreshBackupPanel();
  } catch (err) {
    notify(`Could not use that folder: ${(err as Error).message}`, "error");
  }
}

backupBtn.addEventListener("click", () => setBackupPanelVisible(backupPanel.classList.contains("hidden")));
document.getElementById("backup-close")!.addEventListener("click", () => setBackupPanelVisible(false));
backupChooseBtn.addEventListener("click", () => void chooseBackupFolder());
backupNowBtn.addEventListener("click", () => void runBackup(true));
backupEnabledInput.addEventListener("change", () => {
  backupSettings.enabled = backupEnabledInput.checked;
  saveBackupSettings();
  restartBackupTimer();
  refreshBackupPanel();
  // Turning it on without a folder (or with a forgotten permission) goes straight to the picker.
  if (backupSettings.enabled && !backupActive()) void chooseBackupFolder();
});
backupIntervalSelect.addEventListener("change", () => {
  backupSettings.intervalMin = Number(backupIntervalSelect.value);
  saveBackupSettings();
  restartBackupTimer();
  refreshBackupPanel();
});
backupKeepSelect.addEventListener("change", () => {
  backupSettings.keep = Number(backupKeepSelect.value);
  saveBackupSettings();
  refreshBackupPanel();
});
backupPanel.addEventListener("keydown", (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "s") return; // let Ctrl+S save
  e.stopPropagation(); // keep canvas shortcuts out of the panel's fields
  if (e.key === "Escape") setBackupPanelVisible(false);
});

async function loadFromFile(file: File, handle: FSFileHandle | null = null): Promise<void> {
  try {
    const data = JSON.parse(await file.text()) as SceneData;
    applyLoadedScene(data);
  } catch (err) {
    notify(`Could not load file: ${(err as Error).message}`, "error");
    return;
  }
  setDocFile(handle, file.name);
  markSaved(documentText()); // freshly loaded: on disk already, nothing to back up yet
}

/** Replace the scene with loaded data, returning to a clean draw-mode state at the root. */
function applyLoadedScene(data: SceneData): void {
  // Leave simulation first (restores the current scene's poses, clears savedPoses)
  // so it can't run against the bodies we're about to replace.
  if (mode === "sim") setMode("draw");
  savedPoses = null;
  setDocument(data, []); // validates; throws on bad data — loading always opens the root
  view.scale = 1;
  view.tx = 0;
  view.ty = 0;
  view.angle = 0;
  markDirty();
}

/** On startup, restore the last autosaved layout if present and valid. */
function restoreAutosave(): boolean {
  try {
    const raw = localStorage.getItem(AUTOSAVE_KEY);
    if (!raw) return false;
    scene.load(JSON.parse(raw) as SceneData);
    return true;
  } catch {
    return false; /* corrupt autosave — start empty */
  }
}

/**
 * Startup: re-bind the document's file and the backup folder remembered from the last
 * session. The file binding only makes sense when the autosave restored that document;
 * the folder may come back without permission (browsers forget it) — backups then wait
 * until the user re-grants access from the panel.
 */
async function initFileState(restored: boolean): Promise<void> {
  if (fsSupported()) {
    const h = await loadHandle<FSFileHandle>(DOC_HANDLE_KEY);
    if (h && restored) {
      docHandle = h;
      docName = h.name;
    } else if (h) {
      void storeHandle(DOC_HANDLE_KEY, null);
    }
  }
  setDocModified(false);
  updateDocTitle();
  if (fsDirectorySupported()) {
    backupDir = await loadHandle<FSDirectoryHandle>(BACKUP_DIR_KEY);
    backupDirOk = backupDir ? await ensurePermission(backupDir, false) : false;
  }
  lastBackupText = documentText(); // what's restored is what the last session had; back up changes only
  restartBackupTimer();
  refreshBackupButton();
  if (backupSettings.enabled && backupDir && !backupDirOk) {
    notify("Auto-backup is paused until folder access is re-granted — open the auto-backup panel (clock button) and click “Allow access…”.");
  }
}

// --- components: editing contexts, browser panel, instance placement -------

/** Select a whole instance (multi-selection of everything it expanded). */
function selectInstance(inst: ComponentInstance): void {
  const bodies = new Set<number>();
  const joints = new Set<number>();
  addInstanceMembers(inst, bodies, joints);
  setMulti(bodies, joints);
}

/**
 * Enter a definition's own editing context (all normal tools work inside it). `via` is
 * the instance (in the context being left) the definition is entered through — it
 * places the context ghost; null (component browser) leaves the ghost unplaceable at
 * this level. `withGhost` shows the whole enclosing assembly straight away
 * (Ctrl+double-click); a plain entry starts with the ghost off (the breadcrumb eyes
 * turn it on).
 */
function enterComponent(defId: number, via: number | null = null, withGhost = false): void {
  const def = scene.getComponent(defId);
  if (!def) return;
  if (editPath[editPath.length - 1] === defId) return; // already editing this definition
  if (mode === "sim") setMode("draw");
  leaveGroup(); // an open group belongs to the context being left
  syncComponentContext(); // store whatever definition we're leaving behind
  if (editPath.length === 0) rootData = scene.serializeContext();
  editPath.push(defId);
  viaStack.push(via);
  tempDims.push([]);
  ghostDepth = withGhost ? Infinity : 0;
  invalidateGhost();
  savedViews.push({ ...view });
  scene.loadContext(def.data);
  resetTransient();
  fitView();
  updateCrumbBar();
  updateCompPanel();
  pushHistory(); // a context switch is an undo step (undo returns to the parent)
}

/** Exit `levels` definition contexts (all changes cascade to every instance). */
function exitComponent(levels = 1): void {
  if (editPath.length === 0) return;
  if (mode === "sim") setMode("draw");
  leaveGroup(); // an open group belongs to the context being left
  for (let k = 0; k < levels && editPath.length > 0; k++) {
    syncComponentContext(); // def.data updated + cascaded; rootData refreshed
    editPath.pop();
    viaStack.pop();
    tempDims.pop(); // this level's temporary context dimensions end with it
    const v = savedViews.pop();
    const parentData =
      editPath.length === 0 ? rootData! : scene.getComponent(editPath[editPath.length - 1])!.data;
    scene.loadContext(parentData);
    if (editPath.length === 0) rootData = null;
    if (v) {
      view.scale = v.scale;
      view.tx = v.tx;
      view.ty = v.ty;
      view.angle = v.angle;
    }
  }
  // The cascade snapped instance poses back to their definitions — re-assert the pose
  // dimensions + constraints in the context we landed in (one that can't hold renders
  // violated), then let free geometry follow the moved instances.
  enforcePose(scene);
  solveSketch(scene);
  invalidateGhost();
  resetTransient();
  updateCrumbBar();
  updateCompPanel();
  pushHistory();
}

/**
 * The breadcrumb bar: one crumb per context on the editing path, and beside every
 * ancestor crumb an **eye** that shows / hides that context in the context ghost —
 * clicking an eye sets the ghost depth to reach exactly that level (a shown level's
 * eye hides it and everything outside it). Eyes are disabled from the first level
 * whose placement is unknown (entered from the browser, no instance to place by).
 */
function updateCrumbBar(): void {
  const group = editedGroup();
  crumbBar.classList.toggle("hidden", editPath.length === 0 && !group);
  crumbBar.innerHTML = "";
  if (editPath.length === 0 && !group) return;
  const n = editPath.length;
  const names = ["Assembly", ...editPath.map((id) => scene.getComponent(id)?.name ?? `#${id}`)];
  names.forEach((name, i) => {
    if (i > 0) {
      const sep = document.createElement("span");
      sep.className = "crumb-sep";
      sep.textContent = "▸";
      crumbBar.appendChild(sep);
    }
    // With a group open, even the innermost context is a step back out (to the group's
    // own level), so it gets a button too — only the group itself is "current".
    if (i === names.length - 1 && !group) {
      const cur = document.createElement("span");
      cur.className = "crumb-current";
      cur.textContent = name;
      crumbBar.appendChild(cur);
    } else {
      const btn = document.createElement("button");
      btn.className = "crumb";
      btn.textContent = name;
      btn.title = `Back to ${name}`;
      btn.addEventListener("click", () => {
        leaveGroup(); // the group lives in the context being left (a no-op with none open)
        if (i < n) exitComponent(n - i);
      });
      crumbBar.appendChild(btn);
      if (i === names.length - 1) return; // the innermost context has no ghost eye
      // Context i is placeable only if every placement from it inward is known.
      const placeable = viaStack.slice(i, n).every((v) => v !== null);
      const shown = placeable && n - i <= ghostDepth;
      const eye = document.createElement("button");
      eye.className = "crumb-eye" + (shown ? " on" : "") + (placeable ? "" : " off");
      eye.textContent = shown ? "◉" : "◌";
      eye.disabled = !placeable;
      eye.title = !placeable
        ? `Can't show ${name} here — this level was entered without an instance to place it by (Ctrl+double-click an instance to edit it in context)`
        : shown
          ? `Hide ${name} (and everything outside it) from the context ghost`
          : `Show the context ghost out to ${name}`;
      eye.addEventListener("click", () => setGhostDepth(shown ? n - i - 1 : n - i));
      crumbBar.appendChild(eye);
    }
  });
  // The open group is the last crumb: everything outside it is veiled until it's left.
  if (group) {
    const sep = document.createElement("span");
    sep.className = "crumb-sep";
    sep.textContent = "▸";
    crumbBar.appendChild(sep);
    const cur = document.createElement("span");
    cur.className = "crumb-current";
    const parts = group.bodyIds.length + group.jointIds.length;
    cur.textContent = `Group (${parts} ${parts === 1 ? "part" : "parts"})`;
    cur.title = "Editing inside this group — Esc, a double-click outside, or a crumb leaves it";
    crumbBar.appendChild(cur);
  }
}

// --- context ghost (the enclosing assembly, faded, in the definition's frame) ---------
/** Set how many enclosing levels the context ghost shows (0 = off) and refresh. */
function setGhostDepth(depth: number): void {
  ghostDepth = Math.max(0, depth);
  invalidateGhost();
  updateCrumbBar();
}

function invalidateGhost(): void {
  ghostDirty = true;
  ghostTargets = null;
}

/** The enclosing contexts of the open definition, outermost first (empty at the root). */
function ghostSources(): GhostSource[] {
  const out: GhostSource[] = [];
  for (let k = 0; k < editPath.length; k++) {
    const data = k === 0 ? rootData : scene.getComponent(editPath[k - 1])?.data ?? null;
    if (!data) break;
    out.push({ data, via: viaStack[k] ?? null });
  }
  return out;
}

/** Rebuild the ghost if anything it depends on changed since the last build. */
function ensureGhost(): void {
  if (!ghostDirty) return;
  ghostDirty = false;
  ghostTargets = null;
  ghostLevels =
    editPath.length > 0 && ghostDepth > 0 ? buildContextGhost(ghostSources(), scene.components, ghostDepth) : [];
}

/** The shown ghost levels (scratch scenes in this definition's frame), for drawing and snapping. */
function ghostScenes(): Scene[] {
  ensureGhost();
  return ghostLevels.filter((g): g is Scene => g !== null);
}

/**
 * The ghost feature a measure-tool click at `p` would pick (innermost level first):
 * a joint, a body corner, a rail, a body edge. Nothing else — no body interiors, disk
 * rims, guides or pattern axes: the ghost offers alignment features, not material.
 */
function ghostRefAt(p: Vec2): GhostRef | null {
  if (mode !== "draw") return null;
  ensureGhost();
  for (let k = ghostLevels.length - 1; k >= 0; k--) {
    const g = ghostLevels[k];
    if (!g) continue;
    const j = g.jointAt(p, pickRadius());
    if (j) return { kind: "ghost", level: k, ref: { kind: "joint", jointId: j.id } };
    const v = bodyVertexRefAt(p, g);
    if (v) return { kind: "ghost", level: k, ref: v };
    const sl = g.sliderAt(p, pickRadius());
    if (sl) return { kind: "ghost", level: k, ref: { kind: "rail", sliderId: sl.id } };
    const e = bodyEdgeRefAt(p, g);
    if (e) return { kind: "ghost", level: k, ref: e };
  }
  return null;
}

/** Resolve a live or ghost reference to world geometry (null when its element is gone). */
function resolveTemp(r: TempRef): ResolvedMeasureRef | null {
  if (r.kind !== "ghost") return scene.resolveMeasureRef(r);
  ensureGhost();
  return ghostLevels[r.level]?.resolveMeasureRef(r.ref) ?? null;
}

function sameTempRef(a: TempRef, b: TempRef): boolean {
  if (a.kind === "ghost" || b.kind === "ghost") {
    return a.kind === "ghost" && b.kind === "ghost" && a.level === b.level && sameMeasureRef(a.ref, b.ref);
  }
  return sameMeasureRef(a, b);
}

// --- temporary context dimensions -----------------------------------------------------
/** This definition level's temporary dimensions (none at the root). */
function curTempDims(): TempDim[] {
  return editPath.length > 0 ? (tempDims[editPath.length - 1] ??= []) : [];
}

function getTempDim(id: number): TempDim | undefined {
  return curTempDims().find((t) => t.id === id);
}

function tempDimAnchor(td: TempDim): Vec2 | null {
  const a = resolveTemp(td.refA);
  const b = resolveTemp(td.refB);
  return a && b ? scale(add(refCenter(a), refCenter(b)), 0.5) : null;
}

function tempDimLabelPos(td: TempDim): Vec2 | null {
  const anchor = tempDimAnchor(td);
  return anchor ? add(anchor, td.labelOffset) : null;
}

/** A temporary dimension's value + drawing geometry this frame (null if an end is gone). */
function tempDimInfo(td: TempDim): MeasureInfo | null {
  const a = resolveTemp(td.refA);
  const b = resolveTemp(td.refB);
  if (!a || !b) return null;
  const labelPos = add(scale(add(refCenter(a), refCenter(b)), 0.5), td.labelOffset);
  const info = measureInfoFor(td.id, a, b, td.axis, labelPos);
  if (info) info.temp = true;
  return info;
}

/** Preview for the measure tool once one pick is on the ghost (label at the cursor). */
function tempDimPreview(refA: TempRef, refB: TempRef, labelPos: Vec2): MeasureInfo | null {
  const a = resolveTemp(refA);
  const b = resolveTemp(refB);
  if (!a || !b) return null;
  const axis = a.kind === "point" && b.kind === "point" ? measureAxisForPlacement(a.p, b.p, labelPos) : "direct";
  const info = measureInfoFor(-1, a, b, axis, labelPos);
  if (info) info.temp = true;
  return info;
}

/** Create a temporary dimension between two picks (at least one on the ghost). */
function addTempDim(refA: TempRef, refB: TempRef, labelPos: Vec2): TempDim | null {
  const a = resolveTemp(refA);
  const b = resolveTemp(refB);
  if (!a || !b) return null;
  const anchor = scale(add(refCenter(a), refCenter(b)), 0.5);
  const td: TempDim = {
    id: tempDimSeq--,
    refA,
    refB,
    labelOffset: sub(labelPos, anchor),
    axis: a.kind === "point" && b.kind === "point" ? measureAxisForPlacement(a.p, b.p, labelPos) : "direct",
  };
  curTempDims().push(td);
  return td;
}

/** Move a temporary dimension's label. Only the label moves — the axis a point pair was
 *  placed with is fixed, like the scene's (`Scene.setMeasurementLabel`). */
function setTempDimLabel(td: TempDim, labelPos: Vec2): void {
  const anchor = tempDimAnchor(td);
  if (anchor) td.labelOffset = sub(labelPos, anchor);
}

function removeTempDim(id: number): void {
  const list = curTempDims();
  const i = list.findIndex((t) => t.id === id);
  if (i >= 0) list.splice(i, 1);
}

/** The temporary dimension whose label pill sits under `p` (topmost first) and which part of it. */
function tempDimLabelHitAt(p: Vec2): { td: TempDim; hit: LabelHit } | null {
  if (!measureVisible || mode !== "draw") return null;
  const list = curTempDims();
  for (let i = list.length - 1; i >= 0; i--) {
    const info = tempDimInfo(list[i]);
    const hit = info ? labelHitAt(info, p) : null;
    if (hit) return { td: list[i], hit };
  }
  return null;
}

/** The temporary dimension whose label sits under `p` (topmost first), or null. */
function tempDimLabelAt(p: Vec2): TempDim | null {
  return tempDimLabelHitAt(p)?.td ?? null;
}

/** The temporary dimension whose label's direction glyph sits under `p`, or null. */
function tempDimGlyphAt(p: Vec2): TempDim | null {
  const h = tempDimLabelHitAt(p);
  return h?.hit === "glyph" ? h.td : null;
}

/** Drop temporary dimensions whose ends no longer resolve (a def edit removed a corner,
 *  a cascade removed the sibling body a ghost ref named…); called once per frame. */
function pruneTempDims(): void {
  const list = curTempDims();
  if (list.length === 0) return;
  const keep = list.filter((t) => resolveTemp(t.refA) && resolveTemp(t.refB));
  if (keep.length !== list.length) {
    list.length = 0;
    list.push(...keep);
    if (selection?.kind === "tempDim" && !keep.some((t) => t.id === selection!.id)) selection = null;
  }
}

/** The selection as the renderer sees it: in sim only a measurement selection is
 *  meaningful (labels stay editable there); a selected temporary context dimension
 *  highlights like a measurement (its negative id can't collide with the scene's). */
function renderSelection(): RenderInput["selection"] {
  if (!selection) return null;
  const { kind, id } = selection;
  if (kind === "tempDim") return mode === "draw" ? { kind: "measure", id } : null;
  if (mode === "draw" || kind === "measure") return { kind, id };
  return null;
}

/** Renderer feed: this level's temporary dimensions (draw mode, measurements visible). */
function tempDimsView(): MeasureInfo[] {
  if (!measureVisible || mode !== "draw") return [];
  const out: MeasureInfo[] = [];
  for (const td of curTempDims()) {
    const info = tempDimInfo(td);
    if (info) out.push(info);
  }
  return out;
}

/**
 * The rigid unit a live reference moves with when a temporary dimension is typed into:
 * a component instance inside the definition moves whole (its shape is design-locked),
 * a grouped body takes its group (bodies + locked free joints), else the body alone.
 */
function moveUnitOfBody(bodyId: number): { bodies: number[]; joints: number[]; instanceId: number | null } {
  const inst = scene.instanceOfBody(bodyId);
  if (inst) {
    const bodies = new Set<number>();
    const joints = new Set<number>();
    addInstanceMembers(inst, bodies, joints);
    return { bodies: [...bodies], joints: [...joints], instanceId: inst.id };
  }
  const g = selGroupOf(bodyId);
  if (g) return { bodies: [...g.bodyIds], joints: [...g.jointIds], instanceId: null };
  return { bodies: [bodyId], joints: [], instanceId: null };
}

/**
 * One-shot move for a temporary context dimension: type a value → the **live** side
 * moves so the dimension measures it, exactly as if it had been dragged there (the
 * ghost side is the surroundings and never moves; the dimension stays driven). What
 * moves follows the picked feature, like a drag of it would: a corner reshapes its
 * body, a joint slides within its body, an edge / body point / rail carries the whole
 * rigid unit (body, group or instance). The definition's own constraints and driving
 * dimensions hold through the anchored sketch solve; if they leave the value off
 * target the edit is rejected (scene restored, red flash). Returns whether it applied.
 */
function applyTempDimValue(td: TempDim, target: number): boolean {
  const reject = (why: string): boolean => {
    flashSketchItems([{ id: td.id, kind: "dimension", error: Infinity }]);
    notify(why, "error");
    return false;
  };
  const held = "Can't move to that value: this definition's own constraints and driving dimensions hold the geometry where it is.";
  const liveIsA = td.refA.kind !== "ghost";
  const liveIsB = td.refB.kind !== "ghost";
  // Both ends on the ghost: the surroundings never move, so there is nothing to place.
  if (liveIsA === liveIsB) return reject("Both ends of this dimension are on the surroundings — nothing in the definition can move to meet it.");
  const liveRef = (liveIsA ? td.refA : td.refB) as MeasureRef;
  const a = resolveTemp(td.refA);
  const b = resolveTemp(td.refB);
  const info = tempDimInfo(td);
  if (!a || !b || !info || info.kind !== "distance") return reject("This dimension has no distance to set (an angle can't drive yet).");
  const live = liveIsA ? a : b;
  const fixed = liveIsA ? b : a;
  const sgn = (x: number): number => (x < 0 ? -1 : 1);
  let delta: Vec2;
  if (live.kind === "point" && fixed.kind === "point") {
    if (td.axis === "h") delta = vec(fixed.p.x + sgn(live.p.x - fixed.p.x) * target - live.p.x, 0);
    else if (td.axis === "v") delta = vec(0, fixed.p.y + sgn(live.p.y - fixed.p.y) * target - live.p.y);
    else {
      const d = sub(live.p, fixed.p);
      const l = Math.hypot(d.x, d.y);
      const u = l > 1e-9 ? scale(d, 1 / l) : vec(1, 0);
      delta = sub(add(fixed.p, scale(u, target)), live.p);
    }
  } else if (live.kind === "point" && fixed.kind === "line") {
    const n = perp(normalize(sub(fixed.b, fixed.a)));
    const d = dot(sub(live.p, fixed.a), n);
    delta = scale(n, sgn(d) * target - d);
  } else if (live.kind === "line" && fixed.kind === "point") {
    // Moving the line by δ along its normal changes the point's signed distance by −δ.
    const n = perp(normalize(sub(live.b, live.a)));
    const d = dot(sub(fixed.p, live.a), n);
    delta = scale(n, d - sgn(d) * target);
  } else if (live.kind === "line" && fixed.kind === "line") {
    const n = perp(normalize(sub(fixed.b, fixed.a)));
    const d = dot(sub(refCenter(live), fixed.a), n);
    delta = scale(n, sgn(d) * target - d);
  } else return reject("Nothing here can be moved to that value.");
  if (!Number.isFinite(delta.x) || !Number.isFinite(delta.y)) return reject("That value can't be reached from this geometry.");

  const before = JSON.stringify(scene.serializeContext());
  const anchors = new Set<string>();
  const movedInstances = new Set<number>();
  const moveUnit = (unit: { bodies: number[]; joints: number[]; instanceId: number | null }): void => {
    const carried = scene.ownedTrackJointsOf(unit.bodies);
    for (const id of unit.bodies) {
      scene.moveBody(id, delta);
      for (const k of anchorVarsForBody(scene, id)) anchors.add(k);
    }
    for (const id of unit.joints) {
      const j = scene.getJoint(id);
      if (!j || j.bodyId !== null || carried.has(id)) continue;
      scene.moveJoint(id, delta);
      for (const k of anchorVarsForJoint(scene, id)) anchors.add(k);
    }
    if (unit.instanceId !== null) movedInstances.add(unit.instanceId);
  };
  // A midpoint moves as its line does — the line is the element a drag would grab.
  const moved = refHost(liveRef);
  switch (moved.kind) {
    case "vertex":
      if (scene.instanceOfBody(moved.bodyId)) moveUnit(moveUnitOfBody(moved.bodyId));
      else {
        scene.moveBodyVertex(moved.bodyId, moved.index, delta, moved.hole ?? null);
        anchors.add(anchorVarForVertex(moved.bodyId, moved.index, moved.hole ?? null));
      }
      break;
    case "joint": {
      const j = scene.getJoint(moved.jointId);
      if (!j) return reject("That joint no longer exists.");
      const inst = scene.instanceOfJoint(j.id);
      if (inst) {
        const bodies = new Set<number>();
        const joints = new Set<number>();
        addInstanceMembers(inst, bodies, joints);
        moveUnit({ bodies: [...bodies], joints: [...joints], instanceId: inst.id });
      } else if (j.bodyId === null && selGroupOfJoint(j.id)) {
        const g = selGroupOfJoint(j.id)!;
        moveUnit({ bodies: [...g.bodyIds], joints: [...g.jointIds], instanceId: null });
      } else {
        scene.moveJoint(j.id, delta);
        for (const k of anchorVarsForJoint(scene, j.id)) anchors.add(k);
      }
      break;
    }
    case "edge":
    case "bodyPoint":
    case "centre":
      moveUnit(moveUnitOfBody(moved.bodyId));
      break;
    case "rail": {
      const c = scene.constraints.find((cc) => cc.id === moved.sliderId);
      if (!c || c.kind !== "slider") return reject("That rail no longer exists.");
      const done = new Set<number>();
      for (const jid of [c.railA, c.railB]) {
        const j = scene.getJoint(jid);
        if (!j) continue;
        if (j.bodyId !== null) {
          const unit = moveUnitOfBody(j.bodyId);
          if (unit.bodies.some((id) => done.has(id))) continue;
          unit.bodies.forEach((id) => done.add(id));
          moveUnit(unit);
        } else if (!done.has(-jid)) {
          done.add(-jid);
          scene.moveJoint(jid, delta);
          for (const k of anchorVarsForJoint(scene, jid)) anchors.add(k);
        }
      }
      break;
    }
    case "guidePoint": {
      const g = scene.getGuide(moved.guideId);
      const q = g ? scene.guidePointWorld(g, moved.which) : null;
      if (!g || !q) return reject("That reference point no longer exists.");
      scene.moveGuidePoint(moved.guideId, moved.which, add(q, delta));
      anchors.add(anchorVarForGuidePoint(moved.guideId, moved.which));
      break;
    }
    case "guideLine":
      scene.moveGuide(moved.guideId, delta);
      for (const k of anchorVarsForGuide(scene, moved.guideId)) anchors.add(k);
      break;
    default:
      return reject("A pattern axis is derived geometry — set the spacing on the pattern instead.");
  }
  // Like a drag: pose partners follow the moved instances, then the anchored sketch
  // solve lets free geometry adapt; if the anchored solve is infeasible the symmetric
  // one decides where things can actually go.
  enforcePose(scene, movedInstances.size ? movedInstances : undefined);
  if (anchors.size === 0 || solveSketch(scene, anchors).length > 0) solveSketch(scene);
  const after = tempDimInfo(td);
  if (!after || Math.abs(after.value - target) > TEMP_DIM_TOL) {
    scene.loadContext(JSON.parse(before) as SceneData);
    return reject(held);
  }
  markDirty();
  return true;
}

/** How far a one-shot move may land from the typed value before it counts as refused. */
const TEMP_DIM_TOL = 1e-3;

// --- component browser panel -------------------------------------------------
let compPanelVisible = false;

function setCompPanelVisible(on: boolean): void {
  compPanelVisible = on;
  compPanel.classList.toggle("hidden", !on);
  compPanelBtn.classList.toggle("active", on);
  if (on) updateCompPanel();
}

/** Browser rows by definition id (rebuilt with the panel; highlighted per frame). */
const compRows = new Map<number, HTMLElement>();

function updateCompPanel(): void {
  if (!compPanelVisible) return;
  compList.innerHTML = "";
  compRows.clear();
  hoverDef = null; // the rows are recreated — the cursor re-enters one on its next move
  if (scene.components.length === 0) {
    const empty = document.createElement("div");
    empty.className = "comp-empty";
    empty.textContent = "No components yet.";
    compList.appendChild(empty);
    return;
  }
  scene.components.forEach((def, index) => {
    const row = document.createElement("div");
    row.className = "comp-row";
    compRows.set(def.id, row);
    // Hovering a row highlights that definition's instances on the canvas (while a row
    // is being dragged the highlight stays on the dragged definition).
    row.addEventListener("mouseenter", () => {
      if (!compList.classList.contains("reordering")) hoverDef = def.id;
    });
    row.addEventListener("mouseleave", () => {
      if (hoverDef === def.id && !compList.classList.contains("reordering")) hoverDef = null;
    });
    const grip = document.createElement("span");
    grip.className = "comp-grip";
    grip.textContent = "⋮⋮";
    grip.title = "Drag to reorder the list";
    grip.addEventListener("mousedown", (e) => startCompRowDrag(e, index));
    row.appendChild(grip);
    const name = document.createElement("input");
    name.className = "comp-name";
    name.value = def.name;
    name.title = "Component name — click to rename";
    name.addEventListener("change", () => {
      const v = name.value.trim();
      if (v && v !== def.name) {
        def.name = v;
        markDirty();
        updateCrumbBar();
      } else {
        name.value = def.name;
      }
    });
    const mkBtn = (label: string, title: string, cls: string, onClick: () => void): HTMLButtonElement => {
      const b = document.createElement("button");
      b.className = `comp-btn ${cls}`.trim();
      b.textContent = label;
      b.title = title;
      b.addEventListener("click", onClick);
      return b;
    };
    row.appendChild(name);
    row.appendChild(mkBtn("＋", "Insert an instance — then click the canvas to place it", "", () => startInsertInstance(def.id)));
    row.appendChild(mkBtn("✎", "Edit this component's definition", "", () => enterComponent(def.id)));
    row.appendChild(mkBtn("×", "Delete this component — its instances become plain bodies (each keeps its rigid group)", "danger", () => deleteComponentUI(def.id)));
    compList.appendChild(row);
  });
  syncCompPanelHighlight();
}

/**
 * Drag-to-reorder in the component browser: the grabbed row follows the cursor (kept
 * within the list) while the other rows slide out of its way; on release
 * `scene.components` is re-spliced to the new order. This is list order only — it is
 * persisted with the document and undoable, but nothing else refers to it.
 */
function startCompRowDrag(e: MouseEvent, from: number): void {
  if (e.button !== 0) return;
  e.preventDefault();
  const rows = [...compList.querySelectorAll<HTMLElement>(".comp-row")];
  const dragged = rows[from];
  if (!dragged || rows.length < 2) return;
  const rects = rows.map((r) => r.getBoundingClientRect());
  const h = rects[from].height;
  const minDy = rects[0].top - rects[from].top;
  const maxDy = rects[rects.length - 1].bottom - rects[from].bottom;
  const startY = e.clientY;
  let to = from;
  hoverDef = scene.components[from]?.id ?? null;
  compList.classList.add("reordering");
  dragged.classList.add("dragging");
  const onMove = (ev: MouseEvent): void => {
    const dy = Math.max(minDy, Math.min(maxDy, ev.clientY - startY));
    dragged.style.transform = `translateY(${dy}px)`;
    // Insertion index = how many other rows have their midpoint above the dragged one's.
    const center = rects[from].top + h / 2 + dy;
    to = 0;
    rects.forEach((r, i) => {
      if (i !== from && r.top + r.height / 2 < center) to++;
    });
    rows.forEach((r, i) => {
      if (i === from) return;
      const shift = from < to && i > from && i <= to ? -h : to < from && i >= to && i < from ? h : 0;
      r.style.transform = shift ? `translateY(${shift}px)` : "";
    });
  };
  const onUp = (): void => {
    window.removeEventListener("mousemove", onMove);
    window.removeEventListener("mouseup", onUp);
    compList.classList.remove("reordering");
    dragged.classList.remove("dragging");
    rows.forEach((r) => (r.style.transform = ""));
    if (to !== from) {
      const [def] = scene.components.splice(from, 1);
      scene.components.splice(to, 0, def);
      markDirty(); // rebuilds the list in the new order and records an undo step
    }
  };
  window.addEventListener("mousemove", onMove);
  window.addEventListener("mouseup", onUp);
}

/** Definitions whose instances are part of the current canvas selection. */
function selectedDefIds(): Set<number> {
  const out = new Set<number>();
  const bodies = multiSel ? [...multiSel.bodies] : selection?.kind === "body" ? [selection.id] : [];
  for (const id of bodies) {
    const inst = scene.instanceOfBody(id);
    if (inst) out.add(inst.defId);
  }
  return out;
}

/** The definitions of the component material under the cursor (either mode): the
 *  enclosing instance's def plus every nested def down to the element's own owner. */
function hoveredCanvasDefs(): number[] {
  if (hoverJoint !== null) {
    const j = scene.getJoint(hoverJoint);
    const chain = scene.componentChainOf("joint", hoverJoint);
    if (chain.length > 0) return chain;
    if (j && j.bodyId !== null) return scene.componentChainOf("body", j.bodyId);
  }
  const bodyId = hoverBody ?? (cursor ? scene.bodyAt(cursor)?.id ?? null : null);
  return bodyId !== null ? scene.componentChainOf("body", bodyId) : [];
}

/** Per frame: mirror the canvas selection / hover onto the browser rows. */
function syncCompPanelHighlight(): void {
  if (!compPanelVisible || compRows.size === 0) return;
  const selected = selectedDefIds();
  const hovered = hoveredCanvasDefs();
  for (const [defId, row] of compRows) {
    row.classList.toggle("selected", selected.has(defId));
    row.classList.toggle("hover", hovered.includes(defId));
  }
}

/** Every occurrence of the definition hovered in the browser — direct instances and
 *  ones nested inside other components — highlighted on the canvas. */
function highlightedOccurrences(): ComponentOccurrence[] | null {
  if (hoverDef === null || !compPanelVisible) return null;
  const occ = scene.componentOccurrences(hoverDef);
  return occ.length > 0 ? occ : null;
}

/** Pack the current selection into a new component definition (replaced by an instance).
 *  With exactly one component instance selected, forks instead: the definition is copied
 *  into a new independent component and the selected instance re-pointed at the copy.
 *  With nothing selected: creates a completely empty component and opens it for editing
 *  (the way to build a component made entirely of other components). */
function makeComponentFromSelection(): void {
  if (mode !== "draw") return;
  const inst = selectionInstance();
  if (inst) {
    const copy = scene.makeInstanceUnique(inst.id);
    if (copy) {
      markDirty();
      setCompPanelVisible(true);
      setHint(`Forked into new component “${copy.name}” — the selected instance now follows it.`);
    }
    return;
  }
  const bodies = multiSel ? [...multiSel.bodies] : selection?.kind === "body" ? [selection.id] : [];
  const joints = multiSel ? [...multiSel.joints] : [];
  if (bodies.length === 0 && joints.length === 0) {
    // Nothing selected: start an empty component and edit it straight away.
    const def = scene.createEmptyComponent(`Component ${scene.components.length + 1}`);
    markDirty();
    setCompPanelVisible(true);
    enterComponent(def.id);
    setHint(`Created empty component “${def.name}” — draw bodies or place instances of other components, then navigate back.`);
    return;
  }
  if (bodies.length === 0) {
    notify("A component needs at least one body — select bodies (and free joints), or select nothing to create an empty component.");
    return;
  }
  if (selectionTouchesInstance()) {
    notify("The selection mixes component instances with other material — select exactly one instance to fork it, or plain bodies to build a new component.");
    return;
  }
  const name = `Component ${scene.components.length + 1}`;
  const res = scene.createComponentFromSelection(name, bodies, joints);
  if (!res) return;
  selectInstance(res.instance);
  markDirty();
  setCompPanelVisible(true);
}

/** Arm a one-shot instance placement: the next draw-mode canvas click drops it there. */
function startInsertInstance(defId: number): void {
  const def = scene.getComponent(defId);
  if (def && def.data.bodies.length === 0 && def.data.joints.length === 0) {
    notify(`“${def.name}” is still empty — edit it and add some content before placing instances.`);
    return;
  }
  if (mode === "sim") setMode("draw");
  disarmTool();
  pendingInsert = defId;
  canvas.style.cursor = "copy";
  setHint(`Click the canvas to place an instance of “${def?.name ?? "?"}” — Esc cancels.`);
}

/** Handle a canvas click while an instance placement is pending. Returns whether it hit. */
function placePendingInsert(p: Vec2): boolean {
  if (pendingInsert === null) return false;
  const defId = pendingInsert;
  pendingInsert = null;
  updateHint();
  canvas.style.cursor = defaultCursor();
  // Cycle guard: a definition can't be instantiated into a context it (transitively) uses.
  const ctxDef = editPath[editPath.length - 1];
  if (ctxDef !== undefined && (defId === ctxDef || scene.componentUses(defId).has(ctxDef))) {
    notify("That would make the component contain itself (circular reference).");
    return true;
  }
  const at = snap(p);
  const inst = scene.instantiateComponent(defId, {
    pos: sub(at, scene.componentCenter(defId)),
    angle: 0,
  });
  if (inst) {
    selectInstance(inst);
    markDirty();
  }
  return true;
}

/** Delete a definition from the browser. Every instance of it — in the live context,
 *  the stashed root, and inside other definitions — is converted into plain elements
 *  (its material stays put; the chassis becomes an ordinary rigid group). Refused only
 *  while the definition itself is open for editing. */
function deleteComponentUI(defId: number): void {
  if (editPath.includes(defId)) {
    notify("This component is being edited — close its context first.");
    return;
  }
  const name = scene.getComponent(defId)?.name ?? "?";
  let n = 0;
  if (rootData) {
    const insts = rootData.instances ?? [];
    const kept = insts.filter((i) => i.defId !== defId);
    n += insts.length - kept.length;
    rootData = { ...rootData, instances: kept };
  }
  const live = scene.dissolveComponent(defId);
  if (live < 0) return;
  n += live;
  // A selected instance of this def keeps its bodies, so the selection stays valid as-is.
  markDirty();
  updateCompPanel();
  notify(
    n === 0
      ? `Deleted component “${name}”.`
      : `Deleted component “${name}” — ${n} instance${n === 1 ? "" : "s"} converted to plain bodies.`
  );
}

makeCompBtn.addEventListener("click", makeComponentFromSelection);
compPanelBtn.addEventListener("click", () => setCompPanelVisible(!compPanelVisible));

// --- DXF import (drag-and-drop) -------------------------------------------
/**
 * Import a DXF file's closed outlines as bodies, centred at `at` (world coords).
 * Coordinates are flipped from DXF's y-up to the canvas's y-down frame and converted
 * from the file's `$INSUNITS` into the document's working unit, so shapes arrive at
 * true scale (a unitless file is taken to already be in working units). Each closed
 * loop becomes one body; the batch lands multi-selected, ready to move or group.
 */
async function importDxfFile(file: File, at: Vec2): Promise<void> {
  try {
    const res = parseDxf(await file.text());
    if (res.skippedEntities > 0)
      console.info(`DXF import: ${res.skippedEntities} unsupported entit${res.skippedEntities === 1 ? "y" : "ies"} ignored.`);
    if (!res.loops.length) {
      notify(
        "No closed shapes found in the DXF — only closed outlines (polylines, circles, or lines/arcs that chain into a loop) can become bodies."
      );
      return;
    }
    if (mode === "sim") setMode("draw");
    const factor = res.unitToMm !== null ? res.unitToMm / UNIT_TO_MM[scene.unit] : 1;
    // Scale into working units and flip y (DXF is y-up, the canvas world is y-down).
    // Reconstructed fillet controls, radii and circles transform the same way (radii
    // and circle radius by the positive scale factor; the flip doesn't affect them).
    const xf = (p: Vec2): Vec2 => vec(p.x * factor, -p.y * factor);
    const loops = res.loops.map((loop) => ({
      pts: loop.pts.map(xf),
      fillet: loop.fillet
        ? { control: loop.fillet.control.map(xf), radii: loop.fillet.radii.map((r) => r * factor) }
        : null,
      circle: loop.circle ? { c: xf(loop.circle.c), r: loop.circle.r * factor } : null,
    }));
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const loop of loops)
      for (const p of loop.pts) {
        minX = Math.min(minX, p.x); minY = Math.min(minY, p.y);
        maxX = Math.max(maxX, p.x); maxY = Math.max(maxY, p.y);
      }
    const off = sub(snap(at), vec((minX + maxX) / 2, (minY + maxY) / 2));
    // Nest the loops: a loop inside another becomes a hole of it (an island inside a
    // hole starts a new solid), so a plate with cut-outs arrives as ONE body.
    const solids = nestLoops(loops.map((l) => l.pts));
    const byPts = new Map(loops.map((l) => [l.pts, l])); // outer loop → its arc metadata
    const bodyIds = new Set<number>();
    for (const s of solids) {
      const meta = byPts.get(s.outer);
      const holes = s.holes.map((loopPts) => {
        const hm = byPts.get(loopPts);
        if (hm?.circle) {
          // A circular cut-out imports as a parametric disk hole (centre + radius) —
          // drag its rim handle to resize it, its centre node to move it.
          return { control: [add(hm.circle.c, off)], radius: hm.circle.r, round: "offset" as const };
        }
        if (hm?.fillet) {
          // Tangent corner arcs stay parametric on holes too (control + per-corner radii).
          return {
            control: hm.fillet.control.map((p) => add(p, off)),
            radii: hm.fillet.radii.map((r) => (r > 0 ? r : null)),
          };
        }
        const h = loopPts.map((p) => add(p, off));
        if (loopSignedArea(h) > 0) h.reverse(); // sampled holes wound opposite the outer, by convention
        return h;
      });
      let body;
      if (meta?.circle) {
        // A circle imports as a parametric disk: one control point + offset radius,
        // so its rim handle resizes it like any other round body.
        body = scene.addBody([add(meta.circle.c, off)], meta.circle.r, "offset", holes);
      } else if (meta?.fillet) {
        // Tangent corner arcs come back as sharp control corners + per-corner radii —
        // the fillet regenerates the same arcs, and each corner stays editable.
        let control = meta.fillet.control.map((p) => add(p, off));
        let radii: (number | null)[] = meta.fillet.radii.map((r) => (r > 0 ? r : null));
        if (loopSignedArea(control) < 0) {
          control = [...control].reverse(); // consistent winding for the corner rounding
          radii = [...radii].reverse(); // ...with the overrides on the renumbered corners
        }
        body = scene.addBody(control, 0, "fillet", holes, radii);
      } else {
        const outer = s.outer.map((p) => add(p, off));
        if (loopSignedArea(outer) < 0) outer.reverse(); // consistent winding for later corner rounding
        body = scene.addBody(outer, 0, "fillet", holes);
      }
      body.color = defaultBodyColor;
      bodyIds.add(body.id);
    }
    setMulti(bodyIds, new Set());
    markDirty();
    if (res.skippedPaths > 0)
      notify(
        `Imported ${solids.length} shape${solids.length === 1 ? "" : "s"}; ` +
          `${res.skippedPaths} open path${res.skippedPaths === 1 ? "" : "s"} couldn't be chained into a closed loop and ${res.skippedPaths === 1 ? "was" : "were"} skipped.`,
        "info"
      );
  } catch (err) {
    notify(`Could not import ${file.name}: ${(err as Error).message}`, "error");
  }
}

// Drag-and-drop onto the canvas: a .dxf imports as bodies at the drop point; a .json
// loads as a scene (same as the Load button).
canvas.addEventListener("dragover", (e) => {
  e.preventDefault();
  if (e.dataTransfer) e.dataTransfer.dropEffect = "copy";
});
canvas.addEventListener("drop", (e) => {
  e.preventDefault();
  const file = e.dataTransfer?.files[0];
  if (!file) return;
  const name = file.name.toLowerCase();
  if (name.endsWith(".dxf")) void importDxfFile(file, eventWorld(e));
  else if (name.endsWith(".json")) {
    // Must be requested synchronously inside the drop event; binds the file for Ctrl+S.
    void handleFromDrop(e.dataTransfer).then((h) => loadFromFile(file, h));
  }
  else notify("Unsupported file type — drop a .dxf (imports as bodies) or a .json (loads a scene).");
});

// --- drawing-mode click handling ----------------------------------------
function handleDrawClick(p: Vec2): void {
  // `placed` marks a fully completed element; the tool then disarms to normal mode.
  // The body tool spans many clicks, so it disarms itself in finishBody().
  let placed = false;
  switch (tool) {
    case "polyline":
      // Spans many clicks; disarms itself in finishPolyline() / finalizeJointBody().
      handlePolylineClick(p);
      break;
    case "rect":
    case "circle":
    case "polygon":
    case "slot":
    case "line":
    case "arc":
    case "text":
      // Two or three clicks (or a press-and-drag); commit in the effective role.
      handleShapeClick(p);
      break;
    case "split":
      // Spans many clicks too; disarms itself when the cut lands back on the outline.
      handleSplitClick(p);
      break;
    case "joint": {
      // Inside bodies: a joint in each overlapping body, pinned together (a shared
      // revolute). On empty space: a free, body-less joint (a movable point).
      const bodies = scene.bodiesAt(p);
      // Place on the grid; hit-test against the raw click point. If snapping would push
      // the joint outside a body it's being attached to, fall back to the click point
      // (inside every hit body by construction).
      let at = placeSnap(p); // object snap on: corners / centres / hole centres first
      if (bodies.length > 0 && !bodies.every((b) => scene.pointInBody(b, at))) at = p;
      let created: ReturnType<typeof scene.addFreeJoint>;
      if (bodies.length > 0) {
        const joints = bodies.map((b) => scene.addJoint(b.id, at));
        for (let i = 1; i < joints.length; i++) scene.addPin(joints[0].id, joints[i].id);
        created = joints[0];
      } else {
        created = scene.addFreeJoint(at);
      }
      // If the node landed on a rail (or rail node), confine it to that rail as a
      // rider — unless it's rigid to the rail's own body (which would do nothing).
      const onSlider = scene.sliderAt(p, pickRadius());
      if (onSlider) {
        const railBodyId = scene.getJoint(onSlider.railA)!.bodyId;
        if (created.bodyId === null || created.bodyId !== railBodyId) {
          scene.attachSliderRider(onSlider.id, created.id);
        }
      }
      placed = true;
      break;
    }
    case "weld": {
      // On an existing joint: toggle the rigidity of every pin it participates in
      // (weld ↔ revolute), mirroring how the Slider tool toggles a rider's lock.
      const j = scene.jointAt(p, pickRadius());
      if (j) {
        const pins = scene.pinsOfJoint(j.id);
        if (pins.length > 0) {
          // Mixed state → make all rigid; all rigid → back to revolute.
          const makeRigid = pins.some((c) => c.rigid !== true);
          for (const c of pins) scene.setPinRigid(c.id, makeRigid);
          selection = { kind: "joint", id: j.id };
          placed = true;
        }
        break;
      }
      // Same placement as the Joint tool, but the joints are welded (rigid pins): the
      // bodies lock completely together at that point. Needs ≥ 2 overlapping bodies —
      // a lone joint has nothing to weld to.
      const bodies = scene.bodiesAt(p);
      if (bodies.length < 2) break;
      // Place on the grid; hit-test against the raw click point (same fallback as the
      // Joint tool: a snap that would leave a hit body uses the exact click point).
      let at = snap(p);
      if (!bodies.every((b) => scene.pointInBody(b, at))) at = p;
      const joints = bodies.map((b) => scene.addJoint(b.id, at));
      for (let i = 1; i < joints.length; i++) scene.addPin(joints[0].id, joints[i].id, true);
      selection = { kind: "joint", id: joints[0].id };
      placed = true;
      break;
    }
    case "connect": {
      const j = scene.jointAt(p, pickRadius());
      if (selectedJoint === null) {
        // First pick: the joint to connect.
        if (j) selectedJoint = j.id;
        break;
      }
      // Second pick: a *different* joint → pin them; or a rail → attach as a
      // rider. A hit on the selected joint itself is ignored so the click can fall
      // through to the rail underneath (the rider often sits right on the rail).
      if (j && j.id !== selectedJoint) {
        const a = scene.getJoint(selectedJoint)!;
        if (a.bodyId !== j.bodyId) {
          scene.addPin(selectedJoint, j.id);
          placed = true;
        }
        selectedJoint = null;
        break;
      }
      const s = scene.sliderAt(p, pickRadius());
      if (s) {
        const rider = scene.getJoint(selectedJoint)!;
        const railBodyId = scene.getJoint(s.railA)!.bodyId;
        // Attach unless the rider is rigid to the rail's own body (it would do nothing).
        // A free rider on a free/fixed rail (both bodyId null) is allowed.
        if (rider.bodyId === null || rider.bodyId !== railBodyId) {
          scene.attachSliderRider(s.id, selectedJoint);
          placed = true;
        }
        selectedJoint = null;
        break;
      }
      selectedJoint = null; // clicked empty space — cancel the pending pick
      break;
    }
    case "ground": {
      const j = scene.jointAt(p, pickRadius());
      if (j) {
        const existing = scene.constraints.filter(
          (c) => c.kind === "ground" && c.joint === j.id
        );
        if (existing.length > 0) {
          // Already grounded — toggle the ground off. Exception: a free joint serving as
          // a world-fixed slider rail endpoint must stay anchored (addSlider's invariant).
          const isFreeRailEnd =
            j.bodyId === null &&
            scene.constraints.some(
              (c) => c.kind === "slider" && (c.railA === j.id || c.railB === j.id)
            );
          if (!isFreeRailEnd) {
            for (const g of existing) scene.removeConstraint(g.id);
            placed = true;
          }
        } else {
          scene.addGround(j.id, scene.jointWorld(j));
          placed = true;
        }
        break;
      }
      // No joint under the cursor: ground/unground the body there — and, if it belongs
      // to a permanent group, the whole group (a grounded body is fixed in simulation).
      // Grounding any body of a component instance grounds the instance's chassis (the
      // group its grounded-in-def material forms), which is what "fix this instance"
      // means — internal mechanism parts keep moving.
      const b = scene.bodyAt(p);
      if (b) {
        const inst = scene.instanceOfBody(b.id);
        const chassisGroup =
          inst && inst.groupId !== null ? scene.groups.find((g) => g.id === inst.groupId) : undefined;
        const target = chassisGroup?.bodyIds[0] ?? b.id;
        if (scene.toggleBodyGround(target)) placed = true;
      }
      break;
    }
    case "rail": {
      // A rail is two joints on the same body (moves with it), or two free joints (a track
      // fixed in world space — addSlider grounds them). Attach riders later via Connect,
      // the Joint tool (a joint placed on the rail auto-rides it), or the Slider tool.
      const j = scene.jointAt(p, pickRadius());
      if (!j) break;
      if (railDraftIds.length === 0) {
        railDraftIds = [j.id];
      } else {
        const a = scene.getJoint(railDraftIds[0])!;
        if (j.id === a.id) break; // same joint clicked again — ignore
        const sameBody = a.bodyId !== null && j.bodyId === a.bodyId;
        const bothFree = a.bodyId === null && j.bodyId === null;
        if (sameBody || bothFree) {
          scene.addSlider(a.id, j.id);
          placed = true;
        } else {
          railDraftIds = [j.id]; // mismatched (different bodies, or free + body) — restart here
        }
      }
      break;
    }
    case "slider": {
      // Two-click slider (v20). First click on a body: that body is the owner — the part
      // that moves — and the click is where its travel starts. Second click anywhere: the
      // end of the travel. The result is a rail from start to end with the owner's new
      // joint riding it orientation-locked (prismatic — the body translates along the
      // arrow keeping its angle). Over another body that reaches both points, the rail
      // joints attach to that body (a moving track: the owner slides relative to it);
      // otherwise they are free joints grounded in place (a world-fixed track).
      if (sliderDraft) {
        const owner = scene.getBody(sliderDraft.bodyId);
        if (!owner) {
          sliderDraft = null;
          break;
        }
        const end = sliderEndAt(p);
        if (dist(end, sliderDraft.at) < 1e-6) return; // same point — wait for a distinct end
        // The topmost body under the click other than the owner carries the track — if it
        // contains the start too (joint containment); otherwise the track is world-fixed.
        const track = scene.bodiesAt(p).find((b) => b.id !== owner.id) ?? null;
        let trackId: number | null = null;
        if (track) {
          if (scene.pointInBody(track, sliderDraft.at) && scene.pointInBody(track, end)) trackId = track.id;
          else notify("That body doesn't reach the slider's start point — the track is fixed in the world instead.");
        }
        const made = scene.createSlider(owner.id, sliderDraft.at, end, trackId, sliderDraft.riderId);
        sliderDraft = null;
        if (made) {
          selection = { kind: "rail", id: made.slider.id };
          placed = true;
        }
        break;
      }
      // Existing rails keep their affordances: on a rider, toggle its lock; on another
      // joint near a rail, attach it as a locked rider; on a bare rail, mint a locked
      // rider there — attached to the topmost body under the cursor (excluding the rail's
      // own body), else free (the lock activates once the joint gains a body). A rail
      // under the click wins over starting a new slider (a second carriage on one track).
      const s = scene.sliderAt(p, pickRadius());
      const j = scene.jointAt(p, pickRadius(), "rider"); // a start pair means its rider here
      if (j) {
        const owner = scene.sliderOfRider(j.id);
        if (owner) {
          scene.setSliderRiderLocked(owner.id, j.id, !owner.locked.includes(j.id));
          selection = { kind: "joint", id: j.id };
          placed = true;
          break;
        }
        if (s && s.railA !== j.id && s.railB !== j.id) {
          const railBodyId = scene.getJoint(s.railA)!.bodyId;
          // Attach unless the joint is rigid to the rail's own body (it would do nothing).
          if (j.bodyId === null || j.bodyId !== railBodyId) {
            scene.attachSliderRider(s.id, j.id, true);
            selection = { kind: "joint", id: j.id };
            placed = true;
          }
          break;
        }
        // An existing joint of a body, away from any rail: start the slider *at that joint*
        // — it becomes the rider (no second joint stacked on it). Rail joints and free
        // joints have no body to move, so they can't start one.
        if (j.bodyId === null || scene.constraints.some((c) => c.kind === "slider" && (c.railA === j.id || c.railB === j.id))) {
          notify("Click on a body to start a slider — that body is the part that will move.");
          break;
        }
        sliderDraft = { bodyId: j.bodyId, at: scene.jointWorld(j), riderId: j.id };
        return; // nothing committed yet
      }
      if (!s) {
        // First click: pick the owner body and the start of the travel (snapped like a
        // Joint-tool placement — a snap that would leave the body uses the exact click).
        const owner = scene.bodyAt(p);
        if (!owner) {
          notify("Click on a body to start a slider — that body is the part that will move.");
          break;
        }
        let at = placeSnap(p);
        if (!scene.pointInBody(owner, at)) at = p;
        sliderDraft = { bodyId: owner.id, at, riderId: null };
        return; // nothing committed yet
      }
      const ja = scene.getJoint(s.railA)!;
      const jb = scene.getJoint(s.railB)!;
      const a = scene.jointWorld(ja);
      const b = scene.jointWorld(jb);
      const d = sub(b, a);
      const dl = Math.hypot(d.x, d.y);
      if (dl < 1e-9) break; // degenerate rail
      // The slider must sit on the rail: project the click onto the segment.
      const t = Math.max(0, Math.min(dl, (d.x * (p.x - a.x) + d.y * (p.y - a.y)) / dl));
      const at = vec(a.x + (d.x / dl) * t, a.y + (d.y / dl) * t);
      // bodiesAt returns topmost-first; skip the rail's own body (rigid to the rail).
      const host = scene.bodiesAt(at).find((body) => body.id !== ja.bodyId);
      const created = host ? scene.addJoint(host.id, at) : scene.addFreeJoint(at);
      scene.attachSliderRider(s.id, created.id, true);
      selection = { kind: "joint", id: created.id };
      placed = true;
      break;
    }
    case "linearActuator": {
      // Single click on a rail: make it self-driving. The rail's own rider nearest the
      // click (a two-click slider's carriage, or any rider not yet driven) becomes the
      // actuator's rider; a rail with none gets a new free rider dropped at the click.
      const s = scene.sliderAt(p, pickRadius());
      if (!s) break;
      const driven = new Set(linearActuators().map((a) => a.riderId));
      let riderId: number | null = null;
      let bestD = Infinity;
      for (const rid of s.riders) {
        if (driven.has(rid)) continue;
        const rj = scene.getJoint(rid);
        if (!rj) continue;
        const d = dist(scene.jointWorld(rj), p);
        if (d < bestD) {
          bestD = d;
          riderId = rid;
        }
      }
      const created =
        riderId !== null ? scene.addLinearActuatorOn(s.id, riderId) : scene.addLinearActuator(s.id, snap(p));
      if (created) {
        selection = { kind: "joint", id: created.riderId };
        placed = true;
      }
      break;
    }
    case "motor": {
      // Two clicks: pivot joint, then crank pin (both on the same body).
      const j = scene.jointAt(p, pickRadius());
      if (!j || j.bodyId === null) break; // motor lives on a body — free joints aren't pivots
      if (motorPivotDraft === null) {
        motorPivotDraft = j.id;
        break;
      }
      if (j.id === motorPivotDraft) break; // same joint clicked again — ignore
      const pivot = scene.getJoint(motorPivotDraft)!;
      if (pivot.bodyId !== j.bodyId) {
        // Cranked at a joint that isn't on the pivot's body — restart with this as the new pivot.
        motorPivotDraft = j.id;
        break;
      }
      const motor = scene.addMotor(pivot.bodyId!, motorPivotDraft, j.id);
      if (motor) {
        selection = { kind: "body", id: pivot.bodyId! };
        placed = true;
      }
      motorPivotDraft = null;
      break;
    }
    case "measure":
      handleMeasureClick(p);
      return; // manages its own dirty-marking and disarm
    case "coincident":
    case "horizontal":
    case "vertical":
    case "parallel":
    case "perpendicular":
    case "equal":
    case "fixed":
    case "symmetric":
    case "tangent":
      handleConstraintClick(p);
      return; // manages its own dirty-marking and disarm
  }
  markDirty();
  if (placed) disarmTool();
}

// --- sketch-constraint tools -------------------------------------------------
/** The point reference a constraint click would pick: a joint, then a body control
 *  vertex, then a reference-geometry point. `excludeGuide` leaves one guide out
 *  (so a dragged guide point never picks itself). */
function constraintPointRefAt(p: Vec2, excludeGuide?: number): MeasureRef | null {
  const j = scene.jointAt(p, pickRadius());
  if (j) return { kind: "joint", jointId: j.id };
  const v = bodyVertexRefAt(p);
  if (v) return v;
  const rc = regularCentreRefAt(p);
  if (rc) return rc;
  const gp = scene.guidePointAt(p, pickRadius(), excludeGuide);
  if (gp && scene.guidePointIsRef(gp.guide, gp.which)) return { kind: "guidePoint", guideId: gp.guide.id, which: gp.which };
  return null;
}

/** A line reference onto a guide: a reference polyline's edge (a midpoint host). */
function guideLineRef(guideId: number, edge: number): MidpointHost {
  return { kind: "guideLine", guideId, edge };
}

/** The centre of a regular-polygon outline (outer or hole) within pick range of `p`, topmost body first. */
function regularCentreRefAt(p: Vec2, s: Scene = scene): MeasureRef | null {
  const r = pickRadius();
  for (let i = s.bodies.length - 1; i >= 0; i--) {
    const body = s.bodies[i];
    const c = s.regularCentreWorld(body.id);
    if (c && dist(c, p) <= r) return { kind: "centre", bodyId: body.id };
    for (let hi = 0; hi < (body.holes?.length ?? 0); hi++) {
      const hc = s.regularCentreWorld(body.id, hi);
      if (hc && dist(hc, p) <= r) return { kind: "centre", bodyId: body.id, hole: hi };
    }
  }
  return null;
}

/** Topmost body control vertex — outer outline or hole — within pick range, as a ref. */
function bodyVertexRefAt(p: Vec2, s: Scene = scene): MeasureRef | null {
  for (let i = s.bodies.length - 1; i >= 0; i--) {
    const body = s.bodies[i];
    const verts = s.bodyControlWorld(body);
    for (let vi = 0; vi < verts.length; vi++) {
      if (dist(verts[vi], p) <= pickRadius()) return { kind: "vertex", bodyId: body.id, index: vi };
    }
    for (let hi = 0; hi < (body.holes?.length ?? 0); hi++) {
      const hv = s.bodyHoleControlWorld(body, hi);
      for (let vi = 0; vi < hv.length; vi++) {
        if (dist(hv[vi], p) <= pickRadius())
          return { kind: "vertex", bodyId: body.id, index: vi, hole: hi };
      }
    }
  }
  return null;
}

/** Topmost body control edge — outer outline or hole — within pick range, as a ref. */
function bodyEdgeRefAt(p: Vec2, s: Scene = scene): MeasureRef | null {
  for (let i = s.bodies.length - 1; i >= 0; i--) {
    const body = s.bodies[i];
    const scanEdges = (verts: Vec2[], hole: number | null): MeasureRef | null => {
      if (verts.length < 2) return null;
      for (let ei = 0; ei < verts.length; ei++) {
        if (distToSegment(p, verts[ei], verts[(ei + 1) % verts.length]) <= pickRadius()) {
          return hole === null
            ? { kind: "edge", bodyId: body.id, index: ei }
            : { kind: "edge", bodyId: body.id, index: ei, hole };
        }
      }
      return null;
    };
    const outer = scanEdges(s.bodyControlWorld(body), null);
    if (outer) return outer;
    for (let hi = 0; hi < (body.holes?.length ?? 0); hi++) {
      const hit = scanEdges(s.bodyHoleControlWorld(body, hi), hi);
      if (hit) return hit;
    }
  }
  return null;
}

/** The line reference a constraint click would pick: a slider rail, then a body control
 *  edge, then a reference polyline edge. */
function constraintLineRefAt(p: Vec2): MeasureRef | null {
  const s = scene.sliderAt(p, pickRadius());
  if (s) return { kind: "rail", sliderId: s.id };
  const edge = bodyEdgeRefAt(p);
  if (edge) return edge;
  const pax = patternAxisRefAt(p);
  if (pax) return pax;
  const gl = scene.guideLineAt(p, pickRadius());
  if (gl) return guideLineRef(gl.guide.id, gl.edge);
  return null;
}

/**
 * Where a guide-point click (or the placement preview) lands: exactly on a picked point
 * element (joint / body corner / another guide's point / the midpoint of a rail, body
 * edge or reference segment — returned as `pick` for the auto-coincident), projected
 * onto a picked slider rail / body edge, else grid/guide-snapped like any placement. A
 * point always wins over the line it sits on, so a click near an edge's middle takes
 * the midpoint rather than the projection.
 */
function guidePlacementAt(p: Vec2): { at: Vec2; pick: MeasureRef | null } {
  const pick = constraintPointRefAt(p);
  if (pick) {
    const res = scene.resolveMeasureRef(pick);
    if (res?.kind === "point") return { at: res.p, pick };
  }
  const s = scene.sliderAt(p, pickRadius());
  // Body control edge under the cursor (outer or hole — same pick as the line refs).
  const line: MeasureRef | null = s ? { kind: "rail", sliderId: s.id } : bodyEdgeRefAt(p);
  const gl = line ? null : scene.guideLineAt(p, pickRadius());
  const host = line ?? (gl ? guideLineRef(gl.guide.id, gl.edge) : null);
  if (host && isMidpointHost(host)) {
    const midRef: MeasureRef = { kind: "midpoint", of: host };
    const mid = scene.resolveMeasureRef(midRef);
    if (mid?.kind === "point" && dist(mid.p, p) <= pickRadius()) return { at: mid.p, pick: midRef };
  }
  const lineRes = line ? scene.resolveMeasureRef(line) : null;
  const seg = lineRes?.kind === "line" ? lineRes : null;
  if (seg) {
    const ab = sub(seg.b, seg.a);
    const t = Math.max(0, Math.min(1, dot(sub(p, seg.a), ab) / Math.max(lenSq(ab), 1e-9)));
    return { at: add(seg.a, scale(ab, t)), pick: null };
  }
  return { at: snap(p), pick: null };
}

/**
 * The circle reference a constraint click would pick: the rim of a disk body or a
 * circular hole (topmost body first — the measure tool's diameter pick), then a
 * reference circle or arc under the cursor.
 */
function constraintCircleRefAt(p: Vec2): MeasureRef | null {
  const rim = diskRimAt(p);
  if (rim && rim.ref.kind === "vertex") {
    return rim.ref.hole === undefined
      ? { kind: "disk", bodyId: rim.ref.bodyId }
      : { kind: "disk", bodyId: rim.ref.bodyId, hole: rim.ref.hole };
  }
  const g = scene.guideCircleAt(p, pickRadius());
  return g ? { kind: "guideCircle", guideId: g.id } : null;
}

/**
 * The radius reference an Equal click would pick: a disk body's rim or a round hole
 * (a `disk` ref), a reference circle or arc, or a rounded corner's arc (the corner's
 * `vertex` — the measure tool's radius pick). A sharp corner has no arc to pick.
 */
function constraintRadiusRefAt(p: Vec2): MeasureRef | null {
  return constraintCircleRefAt(p) ?? cornerArcAt(p)?.ref ?? null;
}

/** The reference the armed constraint tool would pick at `p` (for hover + clicks). */
function constraintRefAt(p: Vec2): MeasureRef | null {
  const kind = tool as SketchConstraintKind;
  if (kind === "parallel" || kind === "perpendicular") {
    return constraintLineRefAt(p);
  }
  if (kind === "equal") {
    // Two lines, or two radii: the first pick prefers a rim / arc (the more specific
    // target where one meets an edge) but takes a line; the second must match it.
    if (constraintPicks.length === 0) return constraintRadiusRefAt(p) ?? constraintLineRefAt(p);
    return isLineRef(constraintPicks[0]) ? constraintLineRefAt(p) : constraintRadiusRefAt(p);
  }
  if (kind === "tangent") {
    // A circle and a line in either order: the first pick prefers a circle (a rim is
    // the more specific target where it crosses an edge) but takes a line; the second
    // must be the other kind.
    if (constraintPicks.length === 0) return constraintCircleRefAt(p) ?? constraintLineRefAt(p);
    return isCircleRef(constraintPicks[0]) ? constraintLineRefAt(p) : constraintCircleRefAt(p);
  }
  // Fixed takes one reference of either shape and commits on that first click.
  if (kind === "fixed") return constraintPointRefAt(p) ?? constraintLineRefAt(p);
  if (kind === "symmetric") {
    // Two objects of one kind, then the mirror line: the first pick prefers a point but
    // takes a line; the second must match it; the third is always a line.
    if (constraintPicks.length === 0) return constraintPointRefAt(p) ?? constraintLineRefAt(p);
    if (constraintPicks.length >= 2) return constraintLineRefAt(p);
    const first = constraintPicks[0];
    const firstIsLine = first.kind === "rail" || first.kind === "edge" || first.kind === "guideLine" || first.kind === "patternAxis";
    return firstIsLine ? constraintLineRefAt(p) : constraintPointRefAt(p);
  }
  if (kind === "coincident") {
    // Point + point, or point + line (either pick order): a pick prefers a point but
    // also takes a line — unless a line is already picked (a line pair is invalid).
    if (constraintPicks.length === 0) return constraintPointRefAt(p) ?? constraintLineRefAt(p);
    const firstIsLine =
      constraintPicks[0].kind === "rail" ||
      constraintPicks[0].kind === "edge" ||
      constraintPicks[0].kind === "guideLine" ||
      constraintPicks[0].kind === "patternAxis";
    if (firstIsLine) return constraintPointRefAt(p);
    return constraintPointRefAt(p) ?? constraintLineRefAt(p);
  }
  // Horizontal / vertical: the first pick prefers a point but also takes a line (which
  // commits immediately); the second pick must be the pair's other point.
  if (constraintPicks.length === 0) return constraintPointRefAt(p) ?? constraintLineRefAt(p);
  return constraintPointRefAt(p);
}

/**
 * A constraint-tool click that picked nothing the tool can use: say what it wants
 * instead of ignoring the click. Only speaks when the click landed on *something* —
 * clicking bare canvas is how you look around while a tool is armed.
 */
function reportUnusablePick(kind: SketchConstraintKind, p: Vec2): void {
  const name = CONSTRAINT_NAME[kind];
  if (kind === "tangent") {
    // Tangential wants a circle and a line; say which one is still missing here.
    const wantLine = constraintPicks.length === 1 && isCircleRef(constraintPicks[0]);
    const wantCircle = constraintPicks.length === 1 && !wantLine;
    if (constraintCircleRefAt(p)) notifyThrottled(`${name} already has its circle — click the line here: a body edge, a rail or a reference segment.`);
    else if (constraintLineRefAt(p)) notifyThrottled(`${name} already has its line — click the circle or arc here: a disk body's rim, a round hole, a reference circle or arc.`);
    else if (constraintPointRefAt(p))
      notifyThrottled(
        wantLine
          ? `${name} needs a line here — a body edge, a rail or a reference segment — not a point.`
          : wantCircle
            ? `${name} needs a circle or arc here — a disk body's rim, a round hole, a reference circle or arc — not a point.`
            : `${name} relates a line to a circle or arc — click a rim or an edge, not a point.`
      );
    else if (scene.bodyAt(p))
      notifyThrottled(`${name} attaches to elements, not to material — click a disk's rim, a round hole, an edge or a rail (a body's inside isn't one).`);
    return;
  }
  if (kind === "equal") {
    // Equal wants two lines or two radii; after the first pick, say which kind the second must match.
    const first = constraintPicks[0];
    if (first && isLineRef(first)) {
      if (constraintRadiusRefAt(p)) notifyThrottled(`${name} has its first line — click the second line here (a body edge, a rail or a reference segment), not a circle: a length and a radius can't be made equal.`);
      else if (constraintPointRefAt(p)) notifyThrottled(`${name} needs a line here — a body edge, a rail or a reference segment — not a point.`);
      else if (scene.bodyAt(p)) notifyThrottled(`${name} attaches to elements, not to material — click an edge or a rail (a body's inside isn't one).`);
    } else if (first) {
      if (constraintLineRefAt(p)) notifyThrottled(`${name} has its first circle — click the second circle or rounded corner here (a disk's rim, a round hole, a rounded corner's arc, a reference circle), not a line: a radius and a length can't be made equal.`);
      else if (constraintPointRefAt(p)) notifyThrottled(`${name} needs a circle or rounded corner here — a rim, a round hole or a rounded corner's arc — not a point.`);
      else if (scene.bodyAt(p)) notifyThrottled(`${name} attaches to elements, not to material — click a rim, a rounded corner's arc or a reference circle (a body's inside isn't one).`);
    } else if (constraintPointRefAt(p)) {
      notifyThrottled(`${name} needs a line or a circle here — a body edge, a rail, a reference segment, a disk's rim or a rounded corner's arc — not a point (for a rounded corner, click its arc).`);
    } else if (scene.bodyAt(p)) {
      notifyThrottled(`${name} attaches to elements, not to material — click an edge, a rail, a rim or a rounded corner's arc (a body's inside isn't one).`);
    }
    return;
  }
  // `constraintRefAt` already said no, so whichever shape the tool wants isn't here:
  // whatever the *other* picker finds is what the click actually landed on.
  if (constraintPointRefAt(p)) {
    notifyThrottled(`${name} needs a line here — a body edge, a rail or a reference segment.`);
    return;
  }
  if (constraintLineRefAt(p)) {
    notifyThrottled(`${name} needs a point here — a joint, a body corner, a polygon centre or a reference point.`);
    return;
  }
  if (scene.bodyAt(p)) {
    notifyThrottled(
      `${name} attaches to elements, not to material — click a corner, joint, edge or rail (a body's inside isn't one).`
    );
  }
}

/**
 * Constraint tool click. Line-pair and point-pair kinds take two picks; horizontal /
 * vertical on a line — and Fixed, on anything — commit on the first; Symmetrical takes
 * the pair and then the mirror line, three picks. The commit adds the constraint and runs a
 * sketch solve — geometry moves to satisfy it, or (unsatisfiable) the constraint is
 * removed again and the conflicting items flash red (reject semantics). A click that
 * picks nothing usable says what the tool wants instead of doing nothing.
 */
function handleConstraintClick(p: Vec2): void {
  const kind = tool as SketchConstraintKind;
  const ref = constraintRefAt(p);
  if (!ref) {
    reportUnusablePick(kind, p); // empty space stays silent; a wrong pick says why
    return;
  }
  const isLine = ref.kind === "rail" || ref.kind === "edge" || ref.kind === "guideLine" || ref.kind === "patternAxis";
  if (constraintPicks.length === 0) {
    if (kind === "fixed" || ((kind === "horizontal" || kind === "vertical") && isLine)) {
      commitConstraint(kind, ref);
      return;
    }
    constraintPicks = [ref];
    updateHint();
    return;
  }
  if (constraintPicks.some((r) => sameMeasureRef(r, ref))) {
    notifyThrottled(`${CONSTRAINT_NAME[kind]} needs two different elements — that one is already picked.`);
    return;
  }
  if (kind === "symmetric") {
    if (constraintPicks.length === 1) {
      constraintPicks = [constraintPicks[0], ref];
      updateHint();
      return;
    }
    commitConstraint(kind, constraintPicks[0], constraintPicks[1], ref);
    return;
  }
  commitConstraint(kind, constraintPicks[0], ref);
}

/**
 * Name the items a rejected sketch edit collided with, for the toast that reports it
 * ("the Horizontal constraint and the 40 mm dimension"). They flash red at the same
 * time, so the phrase only has to make them recognisable — at most three, then a count.
 * `skip` drops an item from the list (the dimension whose own value was being set).
 */
function describeBreaks(breaks: SketchBreak[], skip?: number): string {
  const items = breaks.filter((b) => b.id !== skip);
  // Identical items collapse into a count ("2 Fixed constraints"): a list that repeats
  // the same phrase says nothing the count doesn't.
  const counts = new Map<string, { one: string; many: string; n: number }>();
  for (const b of items) {
    let one: string;
    let many: string;
    if (b.kind === "constraint") {
      const c = scene.sketch.find((s) => s.id === b.id);
      const label = c ? CONSTRAINT_NAME[c.kind] : "";
      one = label ? `the ${label} constraint` : "a constraint";
      many = label ? `${label} constraints` : "constraints";
    } else {
      const m = scene.getMeasurement(b.id);
      const info = m ? scene.measureInfo(m) : null;
      const value = m?.driving && m.target !== undefined ? m.target : info?.value;
      const text =
        value === undefined ? "" : info?.kind === "angle" ? `${Math.round(value * 10) / 10}°` : `${Math.round(value * 10) / 10} ${scene.unit}`;
      one = text ? `the ${text} dimension` : "a dimension";
      many = "dimensions";
    }
    const seen = counts.get(one);
    if (seen) seen.n++;
    else counts.set(one, { one, many, n: 1 });
  }
  const names = [...counts.values()].slice(0, 3).map((c) => (c.n === 1 ? c.one : `${c.n} ${c.many}`));
  const extra = counts.size - names.length;
  if (extra > 0) names.push(extra === 1 ? "1 more" : `${extra} more`);
  if (names.length === 0) return "";
  if (names.length === 1) return names[0];
  return `${names.slice(0, -1).join(", ")} and ${names[names.length - 1]}`;
}

/** Add + solve a sketch constraint; on an unsatisfiable solve it's removed again and flashes.
 *  Every end on component-instance geometry makes it a pose constraint (rigid parts move
 *  instead of shape — pose.ts routes it). Either way the refusal is explained: the model
 *  says why a constraint can't exist between these elements, the flashing items say what
 *  a solvable one collided with. */
function commitConstraint(kind: SketchConstraintKind, refA: MeasureRef, refB?: MeasureRef, mirror?: MeasureRef): void {
  // Asked before the placement: the pose route restores a snapshot on failure, and the
  // reason must describe the scene the user clicked in.
  const problem = scene.sketchConstraintProblem(kind, refA, refB, mirror);
  const { constraint, breaks } = placeConstraint(scene, kind, refA, refB, mirror);
  disarmTool(); // clears the picks (and, via resetTransient, the selection)
  if (!constraint) {
    if (breaks.length) {
      flashSketchItems(breaks);
      const what = describeBreaks(breaks);
      notify(
        what
          ? `${CONSTRAINT_NAME[kind]} can't be applied: it conflicts with ${what} (flashing red).`
          : `${CONSTRAINT_NAME[kind]} can't be applied: the rest of the sketch leaves no way to satisfy it (the conflict is flashing red).`,
        "error"
      );
    } else {
      notifyThrottled(problem ?? `${CONSTRAINT_NAME[kind]} can't be applied to these elements.`);
    }
    return;
  }
  setSketchVisible(true); // placing a constraint while hidden would be invisible
  selection = { kind: "sketch", id: constraint.id };
  markDirty();
}

/** Flash the items a rejected sketch edit couldn't satisfy (painted red on the canvas). */
function flashSketchItems(breaks: SketchBreak[]): void {
  // The flash is the only feedback a rejected edit gives — reveal any hidden layer it
  // needs, so the conflicting items are actually visible.
  if (breaks.some((b) => b.kind === "constraint")) setSketchVisible(true);
  if (breaks.some((b) => b.kind === "dimension")) setMeasureVisible(true);
  sketchFlash = {
    ids: new Set(breaks.map((b) => b.id)),
    until: performance.now() + SKETCH_FLASH_MS,
  };
}

/** Whether the sketch has anything to solve (constraints or driving dimensions). */
function sketchActive(): boolean {
  return (
    scene.sketch.length > 0 ||
    scene.measurements.some((m) => m.mode === "draw" && m.driving === true)
  );
}

/**
 * Live-solve the sketch during a draw-mode edit (drags, rotates): the moved geometry
 * stays where the user put it as far as the constraints allow, and everything
 * constrained to it follows — CAD-style sketch dragging. The dragged geometry is
 * passed to the solver as *anchored*, so constraints never tug it back mid-drag —
 * free elements (guidelines especially) absorb the whole correction and follow
 * exactly, which keeps a dragged group rigid.
 *
 * When the anchored solve is *infeasible* — satisfying the constraints would require
 * moving the dragged geometry itself (e.g. pulling the free point of a vertical
 * guideline sideways when its other point is bound to a joint) — the constraints win:
 * a symmetric re-solve runs right away, so the drag can only move things along the
 * directions the constraints leave free. Geometry is never allowed to sit in a
 * constraint-breaking pose mid-drag.
 */
function solveSketchLive(): void {
  if (mode !== "draw" || !sketchActive()) return;
  // Pose dimensions + constraints first: partner instances move rigidly so they keep
  // holding while the user drags (the dragged instances are anchored — partners follow,
  // never the other way). One that can't hold (grounded partner) just renders violated.
  // The sketch solve below then adapts free geometry to the moved instances.
  enforcePose(scene, draggedInstanceIds());
  const anchors = dragAnchorVars();
  if (!anchors) {
    solveSketch(scene);
    return;
  }
  if (solveSketch(scene, anchors).length > 0) solveSketch(scene);
}

/** Instances pinned by the active drag / rotate: they never move to satisfy a pose
 *  dimension mid-drag — their dimension partners follow instead (undefined when idle). */
function draggedInstanceIds(): Set<number> | undefined {
  const out = new Set<number>();
  const addBody = (id: number): void => {
    const inst = scene.instanceOfBody(id);
    if (inst) out.add(inst.id);
  };
  const addJoint = (id: number): void => {
    const inst = scene.instanceOfJoint(id);
    if (inst) out.add(inst.id);
  };
  if (rotateDrag) {
    rotateDrag.bodyIds.forEach(addBody);
    rotateDrag.jointIds.forEach(addJoint);
  } else if (leftDrag) {
    switch (leftDrag.kind) {
      case "body":
        addBody(leftDrag.id);
        break;
      case "joint":
        addJoint(leftDrag.id);
        break;
      case "vertex":
        addBody(leftDrag.bodyId);
        break;
      case "multi":
        leftDrag.bodies.forEach(addBody);
        leftDrag.joints.forEach(addJoint);
        break;
    }
  }
  return out.size ? out : undefined;
}

/** The sketch variables pinned by the active drag / rotate (undefined when idle). */
function dragAnchorVars(): Set<string> | undefined {
  const keys: string[] = [];
  if (rotateDrag) {
    for (const id of rotateDrag.bodyIds) keys.push(...anchorVarsForBody(scene, id));
    for (const id of rotateDrag.jointIds) keys.push(...anchorVarsForJoint(scene, id));
  } else if (leftDrag) {
    switch (leftDrag.kind) {
      case "body":
        keys.push(...anchorVarsForBody(scene, leftDrag.id));
        break;
      case "joint":
        keys.push(...anchorVarsForJoint(scene, leftDrag.id));
        break;
      case "vertex":
        keys.push(anchorVarForVertex(leftDrag.bodyId, leftDrag.index, leftDrag.hole));
        break;
      case "multi":
        for (const id of leftDrag.bodies) keys.push(...anchorVarsForBody(scene, id));
        for (const id of leftDrag.joints) keys.push(...anchorVarsForJoint(scene, id));
        break;
      case "features":
        for (const v of leftDrag.verts) keys.push(anchorVarForVertex(leftDrag.bodyId, v.index, v.hole));
        for (const id of leftDrag.joints) keys.push(...anchorVarsForJoint(scene, id));
        break;
      case "guide":
        keys.push(...anchorVarsForGuide(scene, leftDrag.id));
        break;
      case "guidePoint":
        keys.push(anchorVarForGuidePoint(leftDrag.id, leftDrag.which));
        break;
    }
  }
  return keys.length ? new Set(keys) : undefined;
}

// --- inline dimension-value editing ------------------------------------------
/**
 * Open the floating value input over a draw-mode dimension's label (double-click).
 * Enter commits: a number drives the dimension to that value (sketch solve; rejected
 * edits flash red); an empty value turns a driving dimension back into a reference.
 */
function openDimEditor(m: Measurement): void {
  const info = scene.measureInfo(m);
  const lp = scene.measurementLabelPos(m);
  if (!info || !lp || info.kind !== "distance") return; // angle dimensions can't drive (v1)
  dimEditId = m.id;
  showDimEditorAt(lp, info.value);
}

/** The value editor over a temporary context dimension: Enter performs a one-shot move
 *  of the live side to the typed value (the dimension itself stays driven). */
function openTempDimEditor(td: TempDim): void {
  const info = tempDimInfo(td);
  const lp = tempDimLabelPos(td);
  if (!info || !lp || info.kind !== "distance") return;
  dimEditTemp = td.id;
  showDimEditorAt(lp, info.value);
}

function showDimEditorAt(lp: Vec2, value: number): void {
  const sp = worldToScreen(view, lp);
  dimEditInput.style.left = `${sp.x}px`;
  dimEditInput.style.top = `${sp.y}px`;
  dimEditInput.value = String(Math.round(value * 10) / 10);
  dimEditInput.classList.remove("hidden");
  dimEditInput.focus();
  dimEditInput.select();
}

function closeDimEditor(): void {
  dimEditId = null;
  dimEditTemp = null;
  patternEdit = null;
  sidesEdit = null;
  dimEditInput.classList.add("hidden");
  dimEditInput.blur();
}

function commitDimEditor(): void {
  if (patternEdit) {
    commitPatternEditor();
    return;
  }
  if (sidesEdit) {
    commitSidesEditor();
    return;
  }
  const id = dimEditId;
  const tempId = dimEditTemp;
  const raw = dimEditInput.value.trim();
  closeDimEditor(); // nulls dimEditId first, so the blur listener doesn't re-commit
  if (tempId !== null) {
    // Temporary context dimension: a value is a one-shot move; an empty field is a no-op
    // (it is always driven — there is nothing to clear).
    const td = getTempDim(tempId);
    if (!td || raw === "") return;
    const target = Number(raw);
    if (!Number.isFinite(target) || target <= 0) {
      flashSketchItems([{ id: tempId, kind: "dimension", error: Infinity }]);
      notify("A dimension needs a positive number.");
      return;
    }
    applyTempDimValue(td, target);
    return;
  }
  if (id === null) return;
  const m = scene.getMeasurement(id);
  if (!m) return;
  if (raw === "") {
    // Cleared value: back to a driven (reference) dimension.
    if (m.driving) {
      scene.clearMeasurementDriving(id);
      markDirty();
    }
    return;
  }
  const target = Number(raw);
  if (!Number.isFinite(target) || target <= 0) {
    flashSketchItems([{ id, kind: "dimension", error: Infinity }]);
    notify("A dimension needs a positive number.");
    return;
  }
  const breaks = applyDimensionValue(scene, id, target);
  if (breaks.length) {
    flashSketchItems(breaks);
    // A distance between two instances of one pattern is the pattern's layout, not
    // shape: the model refuses it outright, so say where that value lives instead.
    if (scene.patternSpannedBy(m.refA, m.refB)) {
      notify("Can't drive that value: the distance between pattern instances is the pattern's own spacing — double-click the pattern's label to change it.", "error");
      return;
    }
    // The edited dimension itself is always among the conflicts — what matters is what
    // else is holding the geometry, so it's named only when nothing else is.
    const what = describeBreaks(breaks, id);
    notify(
      what
        ? `Can't set that value: it conflicts with ${what} (flashing red).`
        : "Can't set that value: the rest of the sketch holds this geometry where it is.",
      "error"
    );
  } else markDirty();
}

dimEditInput.addEventListener("keydown", (e) => {
  e.stopPropagation(); // keep canvas shortcuts (tools, Delete…) out of the text field
  if (e.key === "Enter") commitDimEditor();
  else if (e.key === "Escape") closeDimEditor();
});
dimEditInput.addEventListener("blur", () => {
  if (dimEditId !== null || dimEditTemp !== null || patternEdit || sidesEdit) commitDimEditor();
});

// --- measure tool ----------------------------------------------------------
/**
 * The topmost disk outline (a circular hole, or a one-point offset disk body) whose rim
 * passes within pick range of `p`: its centre vertex ref plus the circle itself (for the
 * highlight). Used by the measure tool to pick a diameter.
 */
function diskRimAt(p: Vec2): { ref: MeasureRef; c: Vec2; r: number } | null {
  const tol = pickRadius();
  for (let i = scene.bodies.length - 1; i >= 0; i--) {
    const body = scene.bodies[i];
    const loops: (number | null)[] = [null];
    for (let hi = 0; hi < (body.holes?.length ?? 0); hi++) loops.push(hi);
    for (const hole of loops) {
      const ref: MeasureRef =
        hole === null ? { kind: "vertex", bodyId: body.id, index: 0 } : { kind: "vertex", bodyId: body.id, index: 0, hole };
      const disk = scene.diskOfRef(ref);
      if (disk && Math.abs(dist(p, disk.c) - disk.r) <= tol) return { ref, c: disk.c, r: disk.r };
    }
  }
  return null;
}

/** Whether `angle` lies on the arc starting at `a0` sweeping `sweep` (signed) radians. */
function angleOnArc(angle: number, a0: number, sweep: number): boolean {
  const twoPi = Math.PI * 2;
  const d = sweep >= 0 ? angle - a0 : a0 - angle;
  return ((d % twoPi) + twoPi) % twoPi <= Math.abs(sweep) + 1e-6;
}

/**
 * The topmost rounded corner (outer outline or hole of any body) whose drawn arc passes
 * within pick range of `p`: its vertex ref plus the arc (for the highlight). Used by
 * the measure tool to pick a corner radius. A sharp corner has no arc to pick.
 */
function cornerArcAt(p: Vec2): { ref: MeasureRef; arc: { c: Vec2; r: number; a0: number; sweep: number } } | null {
  const tol = pickRadius();
  for (let i = scene.bodies.length - 1; i >= 0; i--) {
    const body = scene.bodies[i];
    const loops: (number | null)[] = [null];
    for (let hi = 0; hi < (body.holes?.length ?? 0); hi++) loops.push(hi);
    for (const hole of loops) {
      for (const corner of scene.outlineCorners(body.id, hole)) {
        const arc = corner.arc;
        if (!arc || Math.abs(dist(p, arc.c) - arc.r) > tol) continue;
        if (!angleOnArc(Math.atan2(p.y - arc.c.y, p.x - arc.c.x), arc.a0, arc.sweep)) continue;
        const ref: MeasureRef =
          hole === null
            ? { kind: "vertex", bodyId: body.id, index: corner.index }
            : { kind: "vertex", bodyId: body.id, index: corner.index, hole };
        return { ref, arc };
      }
    }
  }
  return null;
}

/**
 * The measure reference a click at `p` would pick, by priority: a joint, a body control
 * vertex, a slider rail, a body control-polygon edge, and finally any point inside a
 * body (fixed in that body's frame, grid-snapped when snapping keeps it inside). Empty
 * space picks nothing — references live on existing geometry only.
 */
function measureRefAt(p: Vec2): MeasureRef | null {
  const j = scene.jointAt(p, pickRadius());
  if (j) return { kind: "joint", jointId: j.id };
  const v = bodyVertexRefAt(p);
  if (v) return v;
  const rc = regularCentreRefAt(p);
  if (rc) return rc;
  // A disk rim (circular hole / disk body) picks the disk's centre as a point ref —
  // and, as the *first* pick, a diameter dimension (see handleMeasureClick).
  const rim = diskRimAt(p);
  if (rim) return rim.ref;
  // A rounded corner's arc picks its control vertex as a point ref — and, as the
  // *first* pick, a radius dimension (see handleMeasureClick).
  const arc = cornerArcAt(p);
  if (arc) return arc.ref;
  // Guides are draw-mode-only aids (invisible in sim), so only draw-mode picks see them.
  if (mode === "draw") {
    const gp = scene.guidePointAt(p, pickRadius());
    if (gp && scene.guidePointIsRef(gp.guide, gp.which)) return { kind: "guidePoint", guideId: gp.guide.id, which: gp.which };
  }
  const s = scene.sliderAt(p, pickRadius());
  if (s) return { kind: "rail", sliderId: s.id };
  const edge = bodyEdgeRefAt(p);
  if (edge) return edge;
  if (mode === "draw") {
    const pax = patternAxisRefAt(p);
    if (pax) return pax;
    const gl = scene.guideLineAt(p, pickRadius());
    if (gl) return guideLineRef(gl.guide.id, gl.edge);
  }
  const body = scene.bodyAt(p);
  if (body) {
    const snapped = snap(p);
    const at = scene.pointInBody(body, snapped) ? snapped : p;
    return { kind: "bodyPoint", bodyId: body.id, local: rotate(sub(at, body.pos), -body.angle) };
  }
  return null;
}

/**
 * The endpoint pair a single line pick (body / hole edge, reference-line segment, rail)
 * stands for, or null: no pick yet, two picks, a point, a ghost feature, a pattern axis.
 */
function pickedLineEnds(): [MeasureRef, MeasureRef] | null {
  const first = measurePicks.length === 1 ? measurePicks[0] : undefined;
  return first && first.kind !== "ghost" ? scene.lineEndRefs(first) : null;
}

/**
 * With a single line picked, the reference a click / hover at `p` would take as the second
 * one — or null when it would place the line's own length dimension instead. A bare body
 * interior doesn't count here (it would swallow every label placed over a plate): to
 * dimension a line against an arbitrary body point, pick the point first.
 */
function measureSecondRefAt(p: Vec2): TempRef | null {
  const ref: TempRef | null = measureRefAt(p) ?? ghostRefAt(p);
  return ref && ref.kind === "bodyPoint" ? null : ref;
}

/**
 * Measure tool click: two reference picks, then a third click places the value label.
 * A single pick on a line followed by a click on nothing places the line's own length
 * (a dimension between its ends) — see `pickedLineEnds`.
 */
function handleMeasureClick(p: Vec2): void {
  if (measurePicks.length < 2) {
    // First pick on a disk rim: a diameter dimension — both refs are the disk's centre
    // vertex (the model reads the pair as "diameter"); the next click places the label.
    if (measurePicks.length === 0) {
      const rim = diskRimAt(p);
      if (rim) {
        measurePicks.push(rim.ref, rim.ref);
        return;
      }
      // First pick on a rounded corner's arc: a radius dimension, the same way (the
      // corner's vertex twice — the model reads such a pair as "radius").
      const arc = cornerArcAt(p);
      if (arc) {
        measurePicks.push(arc.ref, arc.ref);
        return;
      }
    }
    // Live geometry first; with nothing there, a feature of the context ghost (a
    // temporary dimension onto the surroundings — see addTempDim).
    const ends = pickedLineEnds();
    const ref: TempRef | null = ends ? measureSecondRefAt(p) : measureRefAt(p) ?? ghostRefAt(p);
    if (!ref) {
      // Nothing under the click after a single line pick: the line's own length, the
      // label where the click landed. After a point pick, keep waiting for a reference.
      if (ends) placeMeasurement(ends[0], ends[1], p);
      return;
    }
    if (measurePicks.length === 1 && sameTempRef(measurePicks[0], ref)) return;
    measurePicks.push(ref);
    return;
  }
  const [pa, pb] = measurePicks;
  placeMeasurement(pa, pb, p);
}

/** Finish the measure tool: a dimension between `pa` and `pb` with its label at `p`. */
function placeMeasurement(pa: TempRef, pb: TempRef, p: Vec2): void {
  if (pa.kind === "ghost" || pb.kind === "ghost") {
    const td = addTempDim(pa, pb, p);
    disarmTool();
    if (td) {
      setMeasureVisible(true);
      selection = { kind: "tempDim", id: td.id };
    }
    return;
  }
  const m = scene.addMeasurement(mode === "sim" ? "sim" : "draw", pa, pb, p);
  disarmTool(); // clears the picks (and, via resetTransient, the selection)
  if (m) {
    setMeasureVisible(true); // placing a measurement while hidden would be invisible
    selection = { kind: "measure", id: m.id };
    markDirty();
  }
}

/**
 * Where world point `p` falls on the label pill drawn for `info`: `"glyph"` (the direction
 * arrow, which is a button), `"value"` (the rest of the pill), or null. The pill's real
 * extent is the pick target (`dimensionLabelHit`, on the renderer's own layout), with the
 * old round zone about the centre kept as a floor so a short pill grabs as easily as ever.
 */
function labelHitAt(info: MeasureInfo, p: Vec2): LabelHit | null {
  const paren = mode === "draw" && !info.driving;
  const hit = dimensionLabelHit(ctx, info, paren, scene.unit, worldToScreen(view, info.labelPos), worldToScreen(view, p));
  if (hit) return hit;
  return dist(info.labelPos, p) <= LABEL_PICK_RADIUS / view.scale ? "value" : null;
}

/** The current mode's measurement whose label pill sits under `p` (topmost first) and which part of it. */
function measurementLabelHitAt(p: Vec2): { m: Measurement; hit: LabelHit } | null {
  if (!measureVisible) return null; // hidden measurements aren't clickable
  const mm = mode === "sim" ? "sim" : "draw";
  for (let i = scene.measurements.length - 1; i >= 0; i--) {
    const m = scene.measurements[i];
    if (m.mode !== mm) continue;
    if (!measurementInScope(m)) continue; // faded with the surroundings of an open group
    const info = scene.measureInfo(m); // null = not drawn this frame, so not clickable either
    const hit = info ? labelHitAt(info, p) : null;
    if (hit) return { m, hit };
  }
  return null;
}

/** The current mode's measurement whose value label sits under `p`, or null (topmost first). */
function measurementLabelAt(p: Vec2): Measurement | null {
  return measurementLabelHitAt(p)?.m ?? null;
}

/** The current mode's measurement whose label's direction glyph sits under `p`, or null. */
function measurementGlyphAt(p: Vec2): Measurement | null {
  const h = measurementLabelHitAt(p);
  return h?.hit === "glyph" ? h.m : null;
}

/** The glyph click's cycle: horizontal → vertical → direct → horizontal. */
const NEXT_AXIS: Record<"h" | "v" | "direct", "h" | "v" | "direct"> = { h: "v", v: "direct", direct: "h" };

/**
 * A press on a dimension label's direction glyph: cycle the point–point axis (the one
 * deliberate way to change what a dimension measures — dragging its label never does)
 * and select the dimension. A driving dimension keeps its target and the sketch
 * re-solves along the new axis with the usual reject semantics: when the held value
 * can't be met that way the direction stays and the dimension flashes. Context
 * dimensions onto the ghost cycle the same way (session state, nothing to record).
 * Returns false when no glyph is under `p`, so the press falls through to the normal
 * label grab / selection.
 */
function clickDimensionGlyph(p: Vec2): boolean {
  const td = tempDimGlyphAt(p);
  if (td) {
    if (td.axis === "h" || td.axis === "v" || td.axis === "direct") {
      const before = tempDimInfo(td);
      td.axis = NEXT_AXIS[td.axis];
      const after = tempDimInfo(td);
      const lp = before && after ? keepGlyphPut(before, after) : null;
      if (lp) setTempDimLabel(td, lp);
    }
    multiSel = null;
    selection = { kind: "tempDim", id: td.id };
    return true;
  }
  const m = measurementGlyphAt(p);
  if (!m) return false;
  multiSel = null;
  selection = { kind: "measure", id: m.id };
  if (m.axis !== "h" && m.axis !== "v" && m.axis !== "direct") return true;
  const prev = m.axis;
  const before = scene.measureInfo(m);
  if (!scene.setMeasurementAxis(m.id, NEXT_AXIS[prev])) return true;
  if (m.driving && m.target !== undefined) {
    const breaks = applyDimensionValue(scene, m.id, m.target);
    if (breaks.length) {
      scene.setMeasurementAxis(m.id, prev); // the geometry is untouched, so the side re-captures as it was
      flashSketchItems(breaks);
      const what = describeBreaks(breaks, m.id);
      notify(
        what
          ? `That direction can't drive this value: it conflicts with ${what} (flashing red).`
          : "That direction can't drive this value — the geometry can't reach it along that axis.",
        "error"
      );
      return true;
    }
  }
  const after = scene.measureInfo(m);
  const lp = before && after ? keepGlyphPut(before, after) : null;
  if (lp) scene.setMeasurementLabel(m.id, lp);
  markDirty();
  return true;
}

/** Screen width of a dimension's label pill as drawn in the current mode. */
function labelWidth(info: MeasureInfo): number {
  return dimensionLabelWidth(ctx, info, mode === "draw" && !info.driving, scene.unit);
}

/**
 * The label position that keeps a pill's glyph end where it was after its value text
 * changed length. The pill is centred on the label position, so a longer / shorter value
 * would slide its left end — the button just pressed — sideways from under the cursor,
 * and the next click would land on the value instead (a 2-digit → 5-character value moves
 * it ~11 px, about the glyph's own width). Shifting the label by half the width change,
 * along screen x, makes the pill grow and shrink at its value end only. Null when the
 * width didn't change.
 */
function keepGlyphPut(before: MeasureInfo, after: MeasureInfo): Vec2 | null {
  const dw = labelWidth(after) - labelWidth(before);
  if (Math.abs(dw) < 0.01) return null;
  const s = worldToScreen(view, after.labelPos);
  return screenToWorld(view, vec(s.x + dw / 2, s.y));
}

/** The sketch constraint whose badge sits under `p` (using last frame's badge layout), or null. */
function sketchGlyphAt(p: Vec2): number | null {
  const r = GLYPH_PICK_RADIUS / view.scale;
  for (let i = sketchGlyphCache.length - 1; i >= 0; i--) {
    const id = sketchGlyphCache[i].id;
    const c = scene.getSketchConstraint(id);
    if (c && !sketchInScope(c)) continue; // faded with the surroundings of an open group
    for (const b of sketchGlyphCache[i].badges) {
      if (dist(b, p) <= r) return id;
    }
  }
  return null;
}

/** Normal/select mode: pick a measurement label (topmost overlay), then a joint, a slider rail, a body. */
function handleSelectClick(p: Vec2): void {
  multiSel = null; // a plain click rebuilds the selection from what's under the cursor
  const inGroup = editedGroup() !== null;
  const tl = tempDimLabelAt(p);
  if (tl) {
    selection = { kind: "tempDim", id: tl.id };
    return;
  }
  const rt = regularTagAt(p);
  if (rt) {
    selection = { kind: "body", id: rt.bodyId }; // the tag belongs to the selected body (double-click edits)
    return;
  }
  const ml = measurementLabelAt(p);
  if (ml) {
    selection = { kind: "measure", id: ml.id };
    return;
  }
  const pl = patternLabelAt(p);
  if (pl) {
    // The rotation badge is a toggle; the other labels select (double-click edits).
    if (pl.field === "rotate") {
      const info = scene.patternInfo(pl.id);
      if (info?.circular && scene.setPatternRotate(pl.id, !info.circular.rotate)) markDirty();
    }
    selection = { kind: "pattern", id: pl.id };
    return;
  }
  const sg = sketchGlyphAt(p);
  if (sg !== null) {
    selection = { kind: "sketch", id: sg };
    return;
  }
  const j = scene.jointAt(p, pickRadius());
  if (j && jointInScope(j.id)) {
    // A joint owned by a component instance selects the whole instance (its material is
    // atomic); a free joint locked to a group selects the whole group.
    const inst = scene.instanceOfJoint(j.id);
    if (inst) {
      selectInstance(inst);
      return;
    }
    const g = j.bodyId === null ? selGroupOfJoint(j.id) : undefined;
    if (g) {
      setMulti(new Set(g.bodyIds), new Set(g.jointIds));
      return;
    }
    selection = { kind: "joint", id: j.id };
    return;
  }
  // A guideline's defining points are small point targets — they beat the line picks.
  // (Reference geometry belongs to no group, so inside an open group it is surroundings:
  // faded and unpickable, though still a snap / constraint target like any outside edge.)
  const gp = inGroup ? null : scene.guidePointAt(p, pickRadius());
  if (gp) {
    selection = { kind: "guide", id: gp.guide.id };
    return;
  }
  const s = scene.sliderAt(p, pickRadius());
  if (s && jointInScope(s.railA) && jointInScope(s.railB)) {
    selection = { kind: "rail", id: s.id };
    return;
  }
  // Guidelines are thin precise targets, so (like rails) they win over body areas.
  const gl = inGroup ? null : scene.guideAt(p, pickRadius());
  if (gl) {
    selection = { kind: "guide", id: gl.id };
    return;
  }
  // Keep the selected body when clicking on/near its control polygon (its edges sit on
  // the boundary, so a click there can land just outside the filled shape). This lets a
  // double-click on an edge reach the vertex-edit handler without deselecting first.
  if (selection?.kind === "body" && (selectedBodyNodeAt(p) || selectedBodyEdgeAt(p))) {
    return;
  }
  // A pattern's dotted axis line selects the pattern — between its instances only, so a
  // click on a member hole still reaches the body underneath.
  // (Inside an open group an outside pattern is faded: its axis stays a constraint /
  // measure target, but selecting it is out of reach — the click falls through.)
  const pax = scene.holeAt(p) ? null : patternAxisRefAt(p);
  if (pax && patternInScope(pax.patternId)) {
    selection = { kind: "pattern", id: pax.patternId };
    return;
  }
  const body = scene.bodyAt(p);
  if (body && bodyInScope(body.id)) {
    // Instance material is selection-atomic: clicking any member selects the instance.
    const inst = scene.instanceOfBody(body.id);
    if (inst) {
      selectInstance(inst);
      return;
    }
    const g = selGroupOf(body.id);
    if (g) {
      // A grouped body is selection-atomic: clicking any member selects the whole group.
      setMulti(new Set(g.bodyIds), new Set(g.jointIds));
      return;
    }
    selection = { kind: "body", id: body.id };
    return;
  }
  // Nothing selectable here — inside an open group, material outside it counts as
  // nothing: it is veiled and can't be picked until the group is left.
  selection = null;
}

/** The selected body's nearest control vertex — outer outline or any hole — within
 *  pick range of `p`, or null. `hole` is the hole index (null = the outer outline). */
function selectedBodyNodeAt(p: Vec2): { index: number; hole: number | null; at: Vec2 } | null {
  if (selection?.kind !== "body") return null;
  const body = scene.getBody(selection.id);
  if (!body) return null;
  let best: { index: number; hole: number | null; at: Vec2 } | null = null;
  let bestD = pickRadius();
  const scan = (verts: Vec2[], hole: number | null): void => {
    verts.forEach((v, i) => {
      const d = dist(v, p);
      if (d <= bestD) {
        bestD = d;
        best = { index: i, hole, at: v };
      }
    });
  };
  scan(scene.bodyControlWorld(body), null);
  body.holes?.forEach((_, hi) => scan(scene.bodyHoleControlWorld(body, hi), hi));
  return best;
}

// --- per-corner radius handles -----------------------------------------------
/** Minimum screen-px offset of a corner's radius handle from its vertex (grabbable at r ≈ 0). */
const FILLET_HANDLE_MIN_PX = 16;

/** Half interior angle + bisector direction of control corner `i`, or null when degenerate. */
function cornerHalfBisector(verts: Vec2[], i: number): { half: number; bis: Vec2 } | null {
  const n = verts.length;
  if (n < 3) return null;
  const v = verts[i];
  const u1 = normalize(sub(verts[(i - 1 + n) % n], v));
  const u2 = normalize(sub(verts[(i + 1) % n], v));
  const angle = Math.acos(Math.max(-1, Math.min(1, dot(u1, u2))));
  if (angle < 1e-3 || angle > Math.PI - 1e-3) return null; // degenerate / nearly straight
  const bis = normalize(add(u1, u2));
  return bis.x === 0 && bis.y === 0 ? null : { half: angle / 2, bis };
}

/** One per-corner radius handle: its world position + the corner it controls. */
interface FilletHandle {
  at: Vec2;
  index: number;
  /** Hole index the corner belongs to (null = the outer outline). */
  hole: number | null;
}

/** The round mode of one of a body's outlines (outer, or hole `hole`). */
function outlineRound(body: Body, hole: number | null): RoundMode {
  return (hole === null ? body.round : body.holes?.[hole]?.round) ?? "fillet";
}

/** World control vertices of one of a body's outlines. */
function outlineControlWorld(body: Body, hole: number | null): Vec2[] {
  return hole === null ? scene.bodyControlWorld(body) : scene.bodyHoleControlWorld(body, hole);
}

/**
 * Every per-corner radius handle of `body` — outer outline and holes. Fillet mode: the
 * midpoint of the corner's drawn arc, pushed out to a minimum screen offset along the
 * bisector so a sharp corner's handle is still grabbable. Offset mode: on the point's
 * circle, outward from the outline's own centre (so a one-point disk gets a rim handle).
 */
function filletHandleList(body: Body): FilletHandle[] {
  const out: FilletHandle[] = [];
  const minOff = FILLET_HANDLE_MIN_PX / view.scale;
  const addOutline = (hole: number | null): void => {
    const verts = outlineControlWorld(body, hole);
    if (!verts.length) return;
    const radii = scene.bodyCornerRadii(body, hole);
    if (outlineRound(body, hole) === "offset") {
      const centre =
        hole === null
          ? body.pos
          : scale(verts.reduce((acc, v) => add(acc, v), vec(0, 0)), 1 / verts.length);
      verts.forEach((v, i) => {
        const outd = normalize(sub(v, centre));
        const dir = outd.x === 0 && outd.y === 0 ? vec(1, 0) : outd;
        out.push({ at: add(v, scale(dir, Math.max(radii[i], minOff))), index: i, hole });
      });
      return;
    }
    const arcs = filletCornerArcs(verts, radii);
    verts.forEach((v, i) => {
      const arc = arcs[i];
      if (arc) {
        const am = arc.a1 + arc.da / 2;
        const mid = vec(arc.center.x + arc.r * Math.cos(am), arc.center.y + arc.r * Math.sin(am));
        if (dist(mid, v) >= minOff) {
          out.push({ at: mid, index: i, hole });
          return;
        }
        // Arc hugs the vertex: fall through to the grabbable min-offset spot on the bisector.
      }
      const cb = cornerHalfBisector(verts, i);
      if (cb) out.push({ at: add(v, scale(cb.bis, minOff)), index: i, hole });
    });
  };
  addOutline(null);
  body.holes?.forEach((_, hi) => addOutline(hi));
  return out;
}

/** The selected body's radius handle within pick range of `p` (nearest), or null. */
function selectedBodyFilletHandleAt(p: Vec2): FilletHandle | null {
  if (mode !== "draw" || tool !== null || selection?.kind !== "body") return null;
  const body = scene.getBody(selection.id);
  if (!body) return null;
  let best: FilletHandle | null = null;
  let bestD = pickRadius();
  for (const h of filletHandleList(body)) {
    const d = dist(p, h.at);
    if (d <= bestD) {
      bestD = d;
      best = h;
    }
  }
  return best;
}

/**
 * The corner radius a fillet-handle drag to `cursor` asks for (null = leave it alone).
 * Offset mode measures straight from the control point; fillet mode projects the cursor
 * onto the corner's bisector and inverts the arc-midpoint distance d = r·(1−sin h)/sin h.
 * Dropping the cursor (nearly) onto the vertex snaps the corner sharp (radius 0).
 */
/**
 * A direct resize of an outline (rim / radius-handle drag, `[` / `]` keys) overrides
 * any driving diameter or radius dimension it changes: those go back to driven
 * (reference) dimensions, so the new size stands instead of reading as a violation.
 * `corner` limits the radius dimensions demoted to the corners it accepts (null = every
 * corner of the outline). Returns true when one was cleared.
 */
function demoteSizeDims(bodyId: number, hole: number | null, corner: ((index: number) => boolean) | null): boolean {
  let any = false;
  for (const m of scene.measurements) {
    if (m.mode !== "draw" || !m.driving) continue;
    if (m.axis === "diameter") {
      const disk = scene.diskOfRef(m.refA);
      if (!disk || disk.bodyId !== bodyId || disk.hole !== hole) continue;
    } else if (m.axis === "radius") {
      const c = scene.cornerOfRef(m.refA);
      if (!c || c.bodyId !== bodyId || c.hole !== hole || (corner && !corner(c.index))) continue;
    } else continue;
    scene.clearMeasurementDriving(m.id);
    any = true;
  }
  return any;
}

/**
 * A direct resize of a radius — a rim / radius-handle drag, `[` / `]`, a reference
 * circle's rim handle — is the reference for every radius made Equal to it: the
 * partners' driving diameter / radius dimensions are demoted like the resized
 * outline's own (the handle now sets the size), and the partners take the new value.
 * Returns true when the element has partners at all (a partner disk's tangent line
 * still needs a solve — the caller settles once the gesture ends).
 */
function propagateEqualRadii(ref: MeasureRef): boolean {
  const partners = equalRadiusPartners(scene, ref);
  if (!partners.length) return false;
  for (const p of partners) {
    if (p.kind === "disk") demoteSizeDims(p.bodyId, p.hole ?? null, null);
    else if (p.kind === "vertex") {
      const corner = scene.cornerOfRef(p);
      if (corner) demoteSizeDims(corner.bodyId, corner.hole, scene.outlineRadiiUniform(corner.bodyId, corner.hole) ? null : (i) => i === corner.index);
      else demoteSizeDims(p.bodyId, p.hole ?? null, null); // a disk named by its centre
    }
  }
  enforceEqualRadii(scene, ref);
  return true;
}

/** Whether the sketch says anything about rims — a tangent, an Equal between radii — so a
 *  resize needs a settle solve (a plain radius change moves no vertex or joint). */
const sketchNamesRadii = (): boolean => scene.sketch.some((c) => c.kind === "tangent" || isEqualRadiusConstraint(c));

function filletDragRadius(body: Body, index: number, cursor: Vec2, hole: number | null): number | null {
  const verts = outlineControlWorld(body, hole);
  const v = verts[index];
  if (!v) return null;
  const sharpZone = (FILLET_HANDLE_MIN_PX * 0.5) / view.scale;
  if (outlineRound(body, hole) === "offset") {
    const m = dist(cursor, v);
    // A one-point offset outline (a disk hole) collapses at radius 0 — keep it a disk.
    return m <= sharpZone && verts.length > 1 ? 0 : m;
  }
  const cb = cornerHalfBisector(verts, index);
  if (!cb) return null;
  const d = Math.max(0, dot(sub(cursor, v), cb.bis));
  if (d <= sharpZone) return 0;
  const s = Math.sin(cb.half);
  if (1 - s < 1e-4) return null; // nearly straight corner: no meaningful fillet
  return (d * s) / (1 - s);
}

/** Radius handles to show alongside the vertex squares (select mode only). */
function filletHandlesView(): Vec2[] | null {
  if (mode !== "draw" || tool !== null || selection?.kind !== "body") return null;
  const body = scene.getBody(selection.id);
  if (!body) return null;
  return filletHandleList(body).map((h) => h.at);
}

/**
 * Nearest control-polygon edge of the selected body — outer outline or any hole —
 * within pick range of `p`. Returns the edge's later-vertex index (insertion slot),
 * the closest point on the segment, and the hole index (null = outer), or null.
 * Vertices within pick range are excluded so an edge hit doesn't shadow a vertex hit
 * (which means "remove" instead of "add").
 */
function selectedBodyEdgeAt(p: Vec2): { index: number; point: Vec2; hole: number | null } | null {
  if (selection?.kind !== "body") return null;
  const body = scene.getBody(selection.id);
  if (!body) return null;
  if (selectedBodyNodeAt(p)) return null;
  let best: { index: number; point: Vec2; hole: number | null } | null = null;
  let bestD = pickRadius();
  const scan = (verts: Vec2[], hole: number | null): void => {
    if (verts.length < 2) return;
    for (let i = 0; i < verts.length; i++) {
      const a = verts[i];
      const b = verts[(i + 1) % verts.length];
      const ab = sub(b, a);
      const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / Math.max(lenSq(ab), 1e-9)));
      const point = add(a, scale(ab, t));
      const d = dist(p, point);
      if (d < bestD) {
        bestD = d;
        best = { index: i + 1, point, hole };
      }
    }
  };
  scan(scene.bodyControlWorld(body), null);
  body.holes?.forEach((_, hi) => scan(scene.bodyHoleControlWorld(body, hi), hi));
  return best;
}

/** Delete the currently selected element and its dependent features. Instance material
 *  deletes as whole instances (their records go with their elements). */
function deleteSelection(): void {
  if (featureSel) {
    deleteFeatures(); // the selected corners / holes / joints only — the body stays
    return;
  }
  if (multiSel) {
    const instIds = new Set<number>();
    for (const id of multiSel.bodies) {
      const inst = scene.instanceOfBody(id);
      if (inst) instIds.add(inst.id);
    }
    for (const id of multiSel.joints) {
      const inst = scene.instanceOfJoint(id);
      if (inst) instIds.add(inst.id);
    }
    for (const iid of instIds) scene.removeInstance(iid);
    for (const id of multiSel.bodies) if (scene.getBody(id)) scene.deleteBody(id);
    for (const id of multiSel.joints) if (scene.getJoint(id)) scene.deleteJoint(id);
    multiSel = null;
    selection = null;
    markDirty();
    return;
  }
  if (!selection) return;
  if (selection.kind === "body" || selection.kind === "joint") {
    const inst =
      selection.kind === "body"
        ? scene.instanceOfBody(selection.id)
        : scene.instanceOfJoint(selection.id);
    if (inst) {
      scene.removeInstance(inst.id);
      selection = null;
      markDirty();
      return;
    }
  }
  if (selection.kind === "rail" && scene.instanceOfConstraint(selection.id)) {
    notify("This rail belongs to a component instance — edit the definition, or delete the whole instance.");
    return;
  }
  if (selection.kind === "tempDim") {
    removeTempDim(selection.id); // not part of the document: no undo step, nothing to save
    selection = null;
    return;
  }
  // Bodies, joints and rails delete through the whole-slider cascade (v20): any part of a
  // slider takes the rest of it — rail joints and riders with no other role — along.
  if (selection.kind === "body") scene.deleteBody(selection.id);
  else if (selection.kind === "joint") scene.deleteJoint(selection.id);
  else if (selection.kind === "measure") scene.removeMeasurement(selection.id);
  else if (selection.kind === "sketch") scene.removeSketchConstraint(selection.id);
  else if (selection.kind === "guide") scene.removeGuide(selection.id);
  else if (selection.kind === "pattern") scene.removePattern(selection.id); // members go, the seed stays
  else scene.deleteSlider(selection.id); // rail: the arrow, its endpoints and its riders
  selection = null;
  markDirty();
}

// --- feature selection (Shift+drag box with a body selected) ---------------------------
/**
 * Whether a Shift+press at `p` starts a feature box: a single plain body is selected and
 * the press lands on empty space — not on a handle, joint or body (those keep their
 * reshape / rigid-drag meanings).
 */
function featureBoxStartAt(p: Vec2): boolean {
  if (mode !== "draw" || tool !== null || multiSel || selection?.kind !== "body") return false;
  if (scene.instanceOfBody(selection.id)) return false;
  return (
    !selectedBodyNodeAt(p) &&
    !selectedBodyFilletHandleAt(p) &&
    !scene.jointAt(p, pickRadius()) &&
    !scene.bodyAt(p)
  );
}

/** The feature selection `verts` + `joints` of `bodyId` name, with pattern members
 *  resolved to their seed's matching feature and duplicates dropped (null when empty). */
function normalizeFeatureSel(
  bodyId: number,
  verts: { hole: number | null; index: number }[],
  joints: number[]
): typeof featureSel {
  const vmap = new Map<string, { hole: number | null; index: number }>();
  for (const v of verts) {
    const hole = v.hole === null ? null : scene.patternSeedHole(bodyId, v.hole);
    vmap.set(vertKey(hole, v.index), { hole, index: v.index });
  }
  const jset = new Set<number>();
  for (const id of joints) {
    jset.add(scene.patternSeedJoint(id));
  }
  return vmap.size + jset.size ? { bodyId, verts: [...vmap.values()], joints: [...jset] } : null;
}

/**
 * Feature box result: the selected body's control vertices (outer outline + holes) and
 * attached joints inside the rectangle — pattern members count as their seed's feature.
 * `additive` (Ctrl+Shift) extends the current feature selection of the same body.
 */
function applyFeatureBox(additive: boolean): void {
  if (!featureBox || selection?.kind !== "body") return;
  const body = scene.getBody(selection.id);
  if (!body) return;
  const x0 = Math.min(featureBox.start.x, featureBox.end.x);
  const x1 = Math.max(featureBox.start.x, featureBox.end.x);
  const y0 = Math.min(featureBox.start.y, featureBox.end.y);
  const y1 = Math.max(featureBox.start.y, featureBox.end.y);
  const inside = (p: Vec2) => p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1;
  const keep = additive && featureSel?.bodyId === body.id ? featureSel : null;
  const verts = [...(keep?.verts ?? [])];
  const joints = [...(keep?.joints ?? [])];
  scene.bodyControlWorld(body).forEach((p, i) => {
    if (inside(p)) verts.push({ hole: null, index: i });
  });
  body.holes?.forEach((_, hi) => {
    scene.bodyHoleControlWorld(body, hi).forEach((p, i) => {
      if (inside(p)) verts.push({ hole: hi, index: i });
    });
  });
  for (const j of scene.joints) {
    if (j.bodyId === body.id && inside(scene.jointWorld(j))) joints.push(j.id);
  }
  featureSel = normalizeFeatureSel(body.id, verts, joints);
}

/**
 * Keep the feature selection consistent with the scene (every frame): it lives only
 * while its body is the single selection in draw mode, and members that no longer exist
 * (a deleted joint, a removed vertex, an undo) are pruned — empty → cleared.
 */
function pruneFeatureSel(): void {
  if (!featureSel) return;
  const body =
    mode === "draw" && selection?.kind === "body" && selection.id === featureSel.bodyId
      ? scene.getBody(featureSel.bodyId)
      : undefined;
  if (!body) {
    featureSel = null;
    return;
  }
  const count = (hole: number | null): number =>
    hole === null ? body.controlLocal.length : body.holes?.[hole]?.controlLocal.length ?? 0;
  const verts = featureSel.verts.filter((v) => v.index < count(v.hole));
  const joints = featureSel.joints.filter((id) => scene.getJoint(id)?.bodyId === body.id);
  if (verts.length + joints.length === 0) featureSel = null;
  else if (verts.length !== featureSel.verts.length || joints.length !== featureSel.joints.length) {
    featureSel = { bodyId: body.id, verts, joints };
  }
}

/**
 * The selected feature under `p` — a handle of a selected vertex (a member's handle
 * stands for its seed's), or a selected joint — as the reference a drag of the whole
 * set would anchor on. Null when the press is on nothing selected (a plain drag then).
 */
function featureHitAt(p: Vec2): MeasureRef | null {
  if (!featureSel) return null;
  const bodyId = featureSel.bodyId;
  const node = selectedBodyNodeAt(p);
  if (node) {
    const hole = node.hole === null ? null : scene.patternSeedHole(bodyId, node.hole);
    if (!featureSel.verts.some((v) => v.hole === hole && v.index === node.index)) return null;
    return hole === null ? { kind: "vertex", bodyId, index: node.index } : { kind: "vertex", bodyId, index: node.index, hole };
  }
  const j = scene.jointAt(p, pickRadius());
  if (j && j.bodyId === bodyId) {
    const id = scene.patternSeedJoint(j.id);
    if (featureSel.joints.includes(id)) return { kind: "joint", jointId: id };
  }
  return null;
}

/** Begin dragging the feature selection by its selected feature `ref` grabbed at `grab`:
 *  that feature is the snap anchor, the object-snap reference and the alignment reference. */
function startFeatureDrag(grab: Vec2, ref: MeasureRef): void {
  if (!featureSel) return;
  const at = scene.resolveMeasureRef(ref);
  if (at?.kind !== "point") return;
  leftDrag = {
    kind: "features",
    bodyId: featureSel.bodyId,
    verts: [...featureSel.verts],
    joints: [...featureSel.joints],
    anchor: ref,
    grabOffset: sub(grab, at.p),
    moved: false,
    osnap: objSnapEnabled ? { ref, hit: null, hitInfinite: false } : undefined,
    align: newDragAlign(ref),
  };
  canvas.style.cursor = "move";
}

/**
 * Move a feature-set drag by `delta`: every selected vertex (a joint stuck to one rides
 * along, as in a single vertex drag), then every selected joint not already carried
 * that way — so nothing moves twice. Seed features carry their pattern's members.
 */
function moveFeatures(d: Extract<LeftDrag, { kind: "features" }>, delta: Vec2): void {
  const body = scene.getBody(d.bodyId);
  if (!body) return;
  // A regular outline is one rigid shape: any of its corners in the selection moves the
  // whole outline (corner by corner it would re-fit about its centre and wobble).
  const regularOutlines = new Set(
    d.verts.filter((v) => scene.outlineRegular(body, v.hole) !== null).map((v) => v.hole ?? "o")
  );
  const worlds: Vec2[] = [];
  for (const v of d.verts) {
    if (regularOutlines.has(v.hole ?? "o")) continue;
    const w = outlineControlWorld(body, v.hole)[v.index];
    if (w) worlds.push(w);
  }
  for (const key of regularOutlines) worlds.push(...outlineControlWorld(body, key === "o" ? null : key));
  const carried = new Set(
    scene.joints
      .filter((j) => j.bodyId === body.id && worlds.some((w) => dist(w, scene.jointWorld(j)) < VERTEX_LINK_EPS))
      .map((j) => j.id)
  );
  for (const key of regularOutlines) scene.moveOutline(d.bodyId, key === "o" ? null : key, delta);
  for (const v of d.verts) {
    if (!regularOutlines.has(v.hole ?? "o")) scene.moveBodyVertex(d.bodyId, v.index, delta, v.hole);
  }
  for (const id of d.joints) if (!carried.has(id)) scene.moveJoint(id, delta);
}

/** Delete the feature selection: its joints and vertices (a hole left too small goes
 *  whole); the outer outline keeps at least 3 corners. The body itself stays selected. */
function deleteFeatures(): void {
  if (!featureSel) return;
  const { outerRefused } = scene.removeBodyFeatures(featureSel.bodyId, featureSel.verts, featureSel.joints);
  if (outerRefused) notify("A body keeps at least 3 corners — the selected outline corners were left in place.");
  featureSel = null;
  markDirty();
}

/** Copy the feature selection to the clipboard: the holes whose every control vertex is
 *  selected, plus the selected joints (with what's internal to them — see FeatureClip).
 *  Outline corners stay with their body, so a corners-only selection copies nothing. */
function copyFeatures(): void {
  if (!featureSel) return;
  const body = scene.getBody(featureSel.bodyId);
  if (!body) return;
  const holes: number[] = [];
  body.holes?.forEach((h, hi) => {
    const n = featureSel!.verts.filter((v) => v.hole === hi).length;
    if (n >= h.controlLocal.length) holes.push(hi);
  });
  const clip = scene.extractFeatures(body.id, holes, featureSel.joints);
  if (!clip) {
    notify("Nothing copied: only whole holes and joints copy as features — outline corners stay with their body.", "info");
    return;
  }
  clipboard = { kind: "features", clip };
}

/** Paste copied features into the selected body, the clip's centre landing at `drop`;
 *  the pasted holes + joints become the new feature selection. */
function pasteFeatures(clip: FeatureClip, drop: Vec2): void {
  if (selection?.kind !== "body" || multiSel) {
    notify("Select a body to paste the copied features into.", "info");
    return;
  }
  if (scene.instanceOfBody(selection.id)) {
    notify("Features can't be pasted into a component instance — edit the definition instead.");
    return;
  }
  const res = scene.insertFeatures(selection.id, clip, drop);
  if (!res) return;
  if (res.holes.length + res.joints.length === 0) {
    notify("Nothing pasted: the copied features don't fit inside the selected body at the cursor.");
    return;
  }
  if (res.skipped > 0) {
    notify(`${res.skipped} copied feature${res.skipped === 1 ? "" : "s"} didn't fit inside the body and ${res.skipped === 1 ? "was" : "were"} skipped.`, "info");
  }
  const body = scene.getBody(selection.id)!;
  const verts: { hole: number | null; index: number }[] = [];
  for (const hi of res.holes) body.holes?.[hi]?.controlLocal.forEach((_, i) => verts.push({ hole: hi, index: i }));
  featureSel = normalizeFeatureSel(body.id, verts, res.joints);
  markDirty();
}

/** Renderer view of the feature selection: the selected handles' positions (a seed's
 *  handles mirrored on its pattern members, which move with it) + the selected joints. */
function featureSelectedView(): RenderInput["featureSelected"] {
  if (!featureSel || mode !== "draw" || tool !== null) return null;
  const body = scene.getBody(featureSel.bodyId);
  if (!body) return null;
  const vertices: Vec2[] = [];
  for (const v of featureSel.verts) {
    const p = outlineControlWorld(body, v.hole)[v.index];
    if (p) vertices.push(p);
  }
  const joints = [...featureSel.joints];
  for (const p of scene.patterns) {
    if (p.bodyId !== body.id) continue;
    for (const slot of p.slots) {
      if (slot.seed.kind === "hole") {
        const seed = slot.seed.hole;
        const idx = featureSel.verts.filter((v) => v.hole === seed).map((v) => v.index);
        if (idx.length === 0) continue;
        for (const m of slot.members) {
          const verts = outlineControlWorld(body, m);
          for (const i of idx) if (verts[i]) vertices.push(verts[i]);
        }
      } else if (featureSel.joints.includes(slot.seed.jointId)) joints.push(...slot.members);
    }
  }
  return { vertices, joints };
}

/** Copy the selection (bodies / groups / free joints, plus whole component instances) to
 *  the clipboard. Instance material copies as instance *placements* — pasting creates new
 *  instances of the same definitions. */
function copySelection(): void {
  if (mode !== "draw") return;
  if (featureSel) {
    copyFeatures();
    return;
  }
  const bodies = multiSel ? [...multiSel.bodies] : selection?.kind === "body" ? [selection.id] : [];
  const joints = multiSel ? [...multiSel.joints] : [];
  if (bodies.length === 0 && joints.length === 0) return;
  const instIds = new Set<number>();
  const plainBodies = bodies.filter((id) => {
    const inst = scene.instanceOfBody(id);
    if (inst) {
      instIds.add(inst.id);
      return false;
    }
    return true;
  });
  const plainJoints = joints.filter((id) => {
    const inst = scene.instanceOfJoint(id);
    if (inst) {
      instIds.add(inst.id);
      return false;
    }
    return true;
  });
  const clip =
    plainBodies.length || plainJoints.length
      ? scene.extractSelection(plainBodies, plainJoints)
      : null;
  const instances: { defId: number; t: InstanceTransform }[] = [];
  for (const iid of instIds) {
    const inst = scene.instances.find((i) => i.id === iid);
    const t = scene.instancePlacement(iid);
    if (inst && t) instances.push({ defId: inst.defId, t });
  }
  if (!clip && instances.length === 0) return;
  // Paste reference: the combined bounding-box centre of everything copied.
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  const include = (q: Vec2): void => {
    minX = Math.min(minX, q.x);
    minY = Math.min(minY, q.y);
    maxX = Math.max(maxX, q.x);
    maxY = Math.max(maxY, q.y);
  };
  for (const id of bodies) {
    const b = scene.getBody(id);
    if (b) scene.bodyWorldVerts(b).forEach(include);
  }
  for (const id of joints) {
    const j = scene.getJoint(id);
    if (j) include(scene.jointWorld(j));
  }
  const center = Number.isFinite(minX) ? vec((minX + maxX) / 2, (minY + maxY) / 2) : vec(0, 0);
  clipboard = { kind: "selection", clip, instances, center };
}

/** Paste the clipboard so its centre lands at `at` (grid-snapped), then select the copy. */
function pasteAt(at: Vec2 | null): void {
  if (mode !== "draw" || !clipboard) return;
  const drop = snap(at ?? screenToWorld(view, vec(canvas.clientWidth / 2, canvas.clientHeight / 2)));
  if (clipboard.kind === "features") {
    pasteFeatures(clipboard.clip, drop);
    return;
  }
  const offset = sub(drop, clipboard.center);
  const bodies = new Set<number>();
  const joints = new Set<number>();
  if (clipboard.clip) {
    const res = scene.insertSelection(clipboard.clip, add(clipboard.clip.center, offset));
    if (res) {
      res.bodyIds.forEach((id) => bodies.add(id));
      res.freeJointIds.forEach((id) => joints.add(id));
    }
  }
  for (const entry of clipboard.instances) {
    // Cycle guard: skip instances whose definition would contain the context being edited.
    const ctxDef = editPath[editPath.length - 1];
    if (ctxDef !== undefined && (entry.defId === ctxDef || scene.componentUses(entry.defId).has(ctxDef))) continue;
    const inst = scene.instantiateComponent(entry.defId, {
      pos: add(entry.t.pos, offset),
      angle: entry.t.angle,
      ...(entry.t.mirrored ? { mirrored: true } : {}),
    });
    if (inst) addInstanceMembers(inst, bodies, joints);
  }
  if (bodies.size || joints.size) {
    // A single pasted body collapses to a normal selection; a fragment stays multi-selected.
    setMulti(bodies, joints);
    markDirty();
  }
}

/** Mirror the selection in place: a single body about its centroid, a multi-selection /
 *  group / component instance about the centre of its combined bounding box. Instance
 *  material mirrors as whole instances (`Scene.mirrorInstance`, via `mirrorBodies`): the
 *  instance becomes its definition's mirror image — the definition and its other
 *  instances are untouched, and a definition edit still cascades into it. */
function mirrorSelection(axis: "h" | "v"): void {
  if (mode !== "draw") return;
  if (multiSel) {
    scene.mirrorBodies([...multiSel.bodies], [...multiSel.joints], axis);
    markDirty();
    return;
  }
  if (selection?.kind !== "body") return;
  if (scene.instanceOfBody(selection.id)) scene.mirrorBodies([selection.id], [], axis);
  else scene.mirrorBody(selection.id, axis);
  markDirty();
}

/** Send the selection (a body, or a whole multi-selection / group) to the back or front
 *  of the z-order — e.g. push an imported reference body behind the mechanism so it stops
 *  stealing clicks. */
function reorderSelection(where: "back" | "front"): void {
  if (mode !== "draw") return;
  const ids = multiSel
    ? [...multiSel.bodies]
    : selection?.kind === "body"
      ? [selection.id]
      : [];
  if (ids.length === 0) return;
  if (scene.reorderBodies(ids, where)) markDirty();
}

/**
 * Begin a rotate (rotate tool). A control node of the already-selected body rotates that
 * body about the node. Otherwise the body under the cursor decides: a body of the current
 * multi-selection — or of a permanent group, which gets selected — rotates the whole
 * selection about the centre of its combined bounding box; a lone body rotates about its
 * centroid (and becomes the selection). No body → nothing happens.
 */
function startRotate(p: Vec2): void {
  const node = selectedBodyNodeAt(p);
  if (node && selection?.kind === "body") {
    beginRotate([selection.id], [], node.at, p);
    return;
  }
  const body = scene.bodyAt(p);
  if (!body) return;
  if (!multiSel?.bodies.has(body.id)) {
    // Not part of the current multi-selection: select it — a grouped body selects its
    // whole group; setMulti collapses an ungrouped body to a normal single selection.
    setMulti(new Set([body.id]), new Set());
  }
  if (multiSel) {
    // Rotate the whole selection about the centre of its combined bounding box.
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    const include = (q: Vec2): void => {
      if (q.x < minX) minX = q.x;
      if (q.y < minY) minY = q.y;
      if (q.x > maxX) maxX = q.x;
      if (q.y > maxY) maxY = q.y;
    };
    for (const id of multiSel.bodies) {
      const b = scene.getBody(id);
      if (b) scene.bodyWorldVerts(b).forEach(include);
    }
    for (const id of multiSel.joints) {
      const j = scene.getJoint(id);
      if (j) include(scene.jointWorld(j));
    }
    beginRotate([...multiSel.bodies], [...multiSel.joints], vec((minX + maxX) / 2, (minY + maxY) / 2), p);
  } else {
    beginRotate([body.id], [], body.pos, p); // centroid
  }
}

function beginRotate(bodyIds: number[], jointIds: number[], pivot: Vec2, grab: Vec2): void {
  const ref = scene.getBody(bodyIds[0]);
  if (!ref) return;
  // Selected free joints that are a selected body's own slider track turn with the body
  // (rotateBody carries them) — leave them out of the explicit orbit.
  const carried = scene.ownedTrackJointsOf(bodyIds);
  rotateDrag = {
    bodyIds,
    jointIds: jointIds.filter((id) => !carried.has(id)),
    pivot,
    grabAngle: ref.angle,
    prevPointer: Math.atan2(grab.y - pivot.y, grab.x - pivot.x),
    accum: 0,
    lastTotal: 0,
    moved: false,
  };
  canvas.style.cursor = "grabbing";
}

/** Wrap an angle to (−π, π]. */
function wrapAngle(a: number): number {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a <= -Math.PI) a += 2 * Math.PI;
  return a;
}

/** Snap an angle to the nearest multiple of 45° when within ROTATE_SNAP_TOL of one. */
function snapAngle(a: number): number {
  const step = Math.PI / 4;
  const nearest = Math.round(a / step) * step;
  return Math.abs(wrapAngle(a - nearest)) < ROTATE_SNAP_TOL ? nearest : a;
}

// --- shape tools (polyline / rectangle / circle / regular polygon / slot / line / arc / text) ---
// Every shape tool draws role-neutral geometry; the armed **role** decides what the
// finished shape becomes: a Body, a Cut out of the body under it, or Reference
// geometry. Line, arc and text are reference-only. The role is sticky (toolbar switch,
// 1 / 2 / 3); Ctrl on a shape's first click flips Body ↔ Cut for that one shape.

/** Whether `t` is a shape tool (draws geometry the role interprets). */
const isShapeTool = (t: Tool | null): t is ShapeTool =>
  t !== null && (SHAPE_TOOLS as readonly string[]).includes(t);

/** The role the next finished shape takes: reference-only tools force Reference. */
function effectiveRole(t: Tool | null = tool): ShapeRole {
  if (t !== null && REFERENCE_ONLY.has(t)) return "reference";
  return roleOverride ?? shapeRole;
}

function setRole(r: ShapeRole): void {
  shapeRole = r;
  roleOverride = null;
  syncRoleButtons();
  updateHint();
}

/** Toolbar role switch: shows the effective role (dimmed when a reference-only tool forces it). */
function syncRoleButtons(): void {
  const eff = effectiveRole();
  const forced = tool !== null && REFERENCE_ONLY.has(tool);
  document.querySelectorAll<HTMLButtonElement>(".role-btn").forEach((b) => {
    b.classList.toggle("active", b.dataset.role === eff);
    b.classList.toggle("forced", forced);
  });
  shapePropsGroup.classList.toggle("hidden", tool !== "text");
  textSizeLabel.classList.toggle("hidden", tool !== "text");
}

function setPolySides(n: number): void {
  polySides = Math.max(3, Math.min(Scene.REGULAR_MAX_SIDES, Math.round(n)));
}

/** Hint for the armed shape tool, prefixed by the role it will apply. */
function shapeHint(): string {
  const r = effectiveRole();
  const what = r === "body" ? "Body" : r === "cut" ? "Cut" : "Reference";
  const how = REFERENCE_ONLY.has(tool!)
    ? " (always reference geometry)"
    : " — 1 / 2 / 3 or the toolbar switch the role; Ctrl on the first click flips Body ↔ Cut for this one shape";
  return `${what}${how} · ${HINTS[tool!]}`;
}

/**
 * First click of a shape: capture the one-shot role override (Ctrl) and, for a Cut,
 * the target body — the selected body if one is selected (select the plate, then cut),
 * else the topmost body under the click; a cut started on empty space finds its body
 * when it closes (the body containing the shape).
 */
function beginShape(first: Vec2, ctrl = mods.ctrl): void {
  const forced = tool !== null && REFERENCE_ONLY.has(tool);
  roleOverride = ctrl && !forced ? (shapeRole === "cut" ? "body" : "cut") : null;
  shapeTarget = null;
  if (effectiveRole() === "cut") {
    const sel = selection?.kind === "body" && !multiSel ? scene.getBody(selection.id) : undefined;
    shapeTarget = (sel ?? scene.bodyAt(first))?.id ?? null;
  }
}

/** Where a shape point lands (exactly on a picked point / projected onto a picked line / snapped). */
function shapePointAt(p: Vec2): { at: Vec2; pick: MeasureRef | null } {
  return guidePlacementAt(p);
}

/** A closed material shape as the Body / Cut roles consume it (a `HoleSpec`-shaped control polygon). */
type ShapeSpec = { control: Vec2[]; radius: number; round: RoundMode; regular?: number };

/**
 * Finish a closed shape in a material role. Body: a new body (freehand colour, clicked
 * points coincident with what they landed on, near-H/V edges constrained). Cut: the
 * shape is subtracted from the target body (see `Scene.cutBody`). Returns whether the
 * shape was consumed — a refused cut explains why and leaves the tool armed.
 */
function commitMaterial(spec: ShapeSpec, snaps: (MeasureRef | null)[] = [], hv = true, centrePick: MeasureRef | null = null): boolean {
  const r = effectiveRole();
  if (r === "body") {
    const body = scene.addBody(spec.control, spec.radius, spec.round, undefined, undefined, spec.regular);
    body.color = defaultBodyColor;
    for (let i = 0; i < snaps.length && i < spec.control.length; i++) {
      const ref = snaps[i];
      if (ref) tryAddConstraint(scene, "coincident", { kind: "vertex", bodyId: body.id, index: i }, ref);
    }
    // A regular polygon whose centre click landed on a point element sticks to it by the centre.
    if (spec.regular && centrePick) tryAddConstraint(scene, "coincident", { kind: "centre", bodyId: body.id }, centrePick);
    if (hv && spec.round === "fillet") autoConstrainBody(scene, body.id);
    markDirty();
    disarmTool();
    return true;
  }
  // Cut: the target picked at the first click, else the body the shape lies in.
  let target = shapeTarget !== null ? scene.getBody(shapeTarget) : undefined;
  if (!target) {
    const loop = spec.round === "offset" ? roundedConvexBody(spec.control, spec.radius) : spec.control;
    const centre = scale(loop.reduce((acc, q) => add(acc, q), vec(0, 0)), 1 / Math.max(1, loop.length));
    target = scene.bodyAt(centre) ?? loop.map((q) => scene.bodyAt(q)).find((b) => b !== undefined);
  }
  if (!target) {
    notify("Nothing to cut here — draw the shape over a body (or select the body first).");
    return false;
  }
  const res = scene.cutBody(target.id, spec);
  if (!res.ok) {
    notify(res.reason);
    return false;
  }
  if (spec.regular && centrePick && res.hole !== null) {
    tryAddConstraint(scene, "coincident", { kind: "centre", bodyId: target.id, hole: res.hole }, centrePick);
  }
  markDirty();
  disarmTool();
  selection = { kind: "body", id: target.id }; // show the new hole's / notch's handles right away
  return true;
}

/** Finish a reference guide: select it and disarm. */
function commitReference(g: Guide | null): boolean {
  if (!g) return false;
  markDirty();
  disarmTool();
  selection = { kind: "guide", id: g.id };
  return true;
}

/** Clear the multi-point shape draft (the tool stays armed). */
function clearShapeDraft(): void {
  shapePts = [];
  shapeSnaps = [];
  shapePress = null;
  shapeTarget = null;
  roleOverride = null;
}

/**
 * Polyline tool click. Body role, first click on an existing joint (or a rail) → build
 * a body from joints (each click adds a joint; clicking one already picked starts the
 * outward-margin phase; a click then finalizes). Otherwise: freehand vertices, closed by
 * clicking the first one, double-clicking or Enter.
 */
function handlePolylineClick(p: Vec2): void {
  if (jointDraftExpanding) {
    finalizeJointBody(p);
    return;
  }
  if (jointDraftIds.length > 0) {
    // Collecting joints. A new joint is added; clicking one already in the outline
    // finishes picking and begins the outward-expansion phase (needs ≥2 joints).
    const j = scene.jointAt(p, pickRadius());
    if (j) {
      if (jointDraftIds.includes(j.id)) {
        if (jointDraftIds.length >= 2) jointDraftExpanding = true;
      } else {
        jointDraftIds.push(j.id);
      }
      return;
    }
    if (addSliderRiderToDraft(p)) return; // landed on a rail
    // No joint and no rail under the cursor: mint a joint at the click point for the
    // new body to use. On top of an existing body, the joint is added to that body and
    // the build later gives the new body a coincident pinned twin (joining them). On
    // empty space, a free joint that gets absorbed into the new body.
    const under = scene.bodyAt(p);
    // Snap to the grid — unless snapping would land outside the body being clicked,
    // in which case the exact click point (inside by hit-test) is used instead.
    let at = snap(p);
    if (under && !scene.pointInBody(under, at)) at = p;
    const created = under ? scene.addJoint(under.id, at) : scene.addFreeJoint(at);
    jointDraftIds.push(created.id);
    jointDraftCreated.push(created.id);
    return;
  }
  if (draftBody.length > 0) {
    addPolylinePoint(p); // already drawing a freehand polyline
    return;
  }
  // Fresh start.
  beginShape(p);
  if (effectiveRole() === "body") {
    // A joint (or a slider rail) begins joint-build mode; anything else a freehand polygon.
    const j = scene.jointAt(p, pickRadius());
    if (j) {
      jointDraftIds = [j.id];
      return;
    }
    if (addSliderRiderToDraft(p)) return;
  }
  addPolylinePoint(p);
}

/**
 * If `p` lands on a slider rail (but not on an existing joint), drop a grid-snapped free
 * joint there, attach it to that slider as a rider, and add it to the body-from-joints
 * draft. The joint is tracked in `jointDraftCreated` so an aborted draft removes it; on a
 * finished build it gets absorbed into the body and stays a rider. Returns whether it hit.
 */
function addSliderRiderToDraft(p: Vec2): boolean {
  const s = scene.sliderAt(p, pickRadius());
  if (!s) return false;
  const rider = scene.addFreeJoint(snap(p));
  scene.attachSliderRider(s.id, rider.id);
  jointDraftIds.push(rider.id);
  jointDraftCreated.push(rider.id);
  return true;
}

/** Finalize a body-from-joints: margin = how far the cursor is from the last joint. */
function finalizeJointBody(p: Vec2): void {
  const lastId = jointDraftIds[jointDraftIds.length - 1];
  const last = scene.jointWorld(scene.getJoint(lastId)!);
  const margin = Math.max(JOINT_BODY_MIN_MARGIN, dist(p, last));
  const body = scene.buildBodyFromJoints(jointDraftIds, margin);
  if (body) {
    body.color = defaultBodyColor;
    jointDraftCreated = []; // absorbed into the body now — don't clean them up
  }
  markDirty();
  disarmTool();
}

function addPolylinePoint(p: Vec2): void {
  if (draftBody.length >= 3 && dist(p, draftBody[0]) < CLOSE_RADIUS / view.scale) {
    finishPolyline();
    return;
  }
  // A click on an existing point (a joint or another body's corner) lands the vertex
  // exactly there and records the pick — finishing the draft turns it into a coincident
  // auto-constraint; a click near an edge / rail projects onto it. Otherwise freehand
  // vertices land on the grid when snap is on.
  const { at, pick } = shapePointAt(p);
  // Ignore near-duplicate points (also de-dupes the 2nd click of a double-click).
  const last = draftBody[draftBody.length - 1];
  if (last && dist(at, last) < 4 / view.scale) return;
  draftBody.push(at);
  draftBodySnaps.push(pick);
}

/**
 * Close the polyline. Body / Cut: a polygon (≥ 3 points). Reference: a closed polygon
 * with ≥ 3 points, a plain segment with 2. Fewer points just clear the draft.
 */
function finishPolyline(): void {
  const pts = draftBody;
  const snaps = draftBodySnaps;
  const r = effectiveRole();
  const done =
    r === "reference"
      ? pts.length >= 2 && commitReferencePoly(pts, pts.length >= 3, snaps)
      : pts.length >= 3 && commitMaterial({ control: pts, radius: 0, round: "fillet" }, snaps);
  if (!done) {
    draftBody = [];
    draftBodySnaps = [];
    clearShapeDraft();
  }
}

/** A reference polyline / polygon whose clicked points stick to what they landed on. */
function commitReferencePoly(pts: Vec2[], closed: boolean, snaps: (MeasureRef | null)[]): boolean {
  const g = scene.addGuidePoly(pts, closed);
  if (!g) return false;
  for (let i = 0; i < snaps.length && i < pts.length; i++) {
    const ref = snaps[i];
    if (ref) tryAddConstraint(scene, "coincident", { kind: "guidePoint", guideId: g.id, which: String(i) }, ref);
  }
  return commitReference(g);
}

/** Rectangle corners from a first corner and the opposite one: Shift squares it, Alt draws from the centre. */
function rectCorners(a: Vec2, b: Vec2): Vec2[] {
  let d = sub(b, a);
  if (mods.shift) {
    const m = Math.max(Math.abs(d.x), Math.abs(d.y));
    d = vec((d.x < 0 ? -1 : 1) * m, (d.y < 0 ? -1 : 1) * m);
  }
  if (mods.alt) return [sub(a, d), vec(a.x + d.x, a.y - d.y), add(a, d), vec(a.x - d.x, a.y + d.y)];
  return [a, vec(a.x + d.x, a.y), add(a, d), vec(a.x, a.y + d.y)];
}

/** `n` points around a circle (preview / reference sampling). */
function circlePoints(c: Vec2, r: number, n = 64): Vec2[] {
  const out: Vec2[] = [];
  for (let i = 0; i < n; i++) {
    const t = (i / n) * Math.PI * 2;
    out.push(vec(c.x + r * Math.cos(t), c.y + r * Math.sin(t)));
  }
  return out;
}

/** Slot width from the cursor: twice its distance to the axis a–b (at least the joint-body minimum). */
function slotWidthAt(a: Vec2, b: Vec2, p: Vec2): number {
  return Math.max(JOINT_BODY_MIN_MARGIN * 2, 2 * distToLine(p, a, normalize(sub(b, a))));
}

/**
 * Click for the point-defined shape tools (rectangle, circle, regular polygon, slot,
 * line, arc, text). The first click starts the shape (see `beginShape`); the last one
 * commits it in the effective role. Points land like polyline vertices.
 */
function handleShapeClick(p: Vec2): void {
  if (tool === "text") {
    if (textEdit) return; // typing already
    const { at } = shapePointAt(p);
    const bodyId = scene.bodyAt(p)?.id ?? null;
    // Deferred past the mousedown's default action, which would blur the field straight away.
    setTimeout(() => { if (tool === "text" && !textEdit) openTextEditor(at, bodyId, null); }, 0);
    return;
  }
  const { at, pick } = shapePointAt(p);
  if (shapePts.length === 0) {
    beginShape(p);
    shapePts = [at];
    shapeSnaps = [pick];
    return;
  }
  const a = shapePts[0];
  const tiny = 4 / view.scale;
  switch (tool) {
    case "rect": {
      const corners = rectCorners(a, at);
      if (Math.abs(at.x - a.x) < tiny || Math.abs(at.y - a.y) < tiny) return; // degenerate — wait
      if (effectiveRole() === "reference") commitReferencePoly(corners, true, [shapeSnaps[0]]);
      else commitMaterial({ control: corners, radius: 0, round: "fillet" }, mods.alt ? [] : [shapeSnaps[0]]);
      return;
    }
    case "circle": {
      const r = dist(a, at);
      if (r < tiny) return;
      if (effectiveRole() === "reference") commitReference(scene.addGuideCircle(a, r));
      else commitMaterial({ control: [a], radius: r, round: "offset" });
      return;
    }
    case "polygon": {
      if (dist(a, at) < tiny) return;
      const pts = regularPolygon(a, at, polySides);
      // (The first click is the centre, not a corner: no corner takes its pick.)
      if (effectiveRole() === "reference") commitReferencePoly(pts, true, []);
      else commitMaterial({ control: pts, radius: 0, round: "fillet", regular: pts.length }, [], false, shapeSnaps[0]);
      return;
    }
    case "line": {
      if (dist(a, at) < tiny) return;
      commitReferencePoly([a, at], false, [shapeSnaps[0], pick]);
      return;
    }
    case "slot": {
      if (shapePts.length === 1) {
        if (dist(a, at) < tiny) return;
        shapePts.push(at);
        shapeSnaps.push(pick);
        // Reference role: the slot's axis is the reference (no width to set).
        if (effectiveRole() === "reference") commitReferencePoly([a, at], false, shapeSnaps);
        return;
      }
      const b = shapePts[1];
      const w = slotWidthAt(a, b, p);
      commitMaterial({ control: [a, b], radius: w / 2, round: "offset" }, shapeSnaps, false);
      return;
    }
    case "arc": {
      if (shapePts.length === 1) {
        if (dist(a, at) < tiny) return;
        shapePts.push(at);
        shapeSnaps.push(pick);
        return;
      }
      const g = scene.addGuideArc(a, at, shapePts[1]);
      if (!g) return; // collinear — wait for a point off the chord
      for (const [which, ref] of [["a", shapeSnaps[0]], ["b", shapeSnaps[1]], ["m", pick]] as const) {
        if (ref) tryAddConstraint(scene, "coincident", { kind: "guidePoint", guideId: g.id, which }, ref);
      }
      commitReference(g);
      return;
    }
  }
}

/** Two-point tools also take a press-and-drag: the press is the first point once the
 *  pointer has clearly moved, the release the second (see mousedown / mousemove / mouseup). */
const isDragShapeTool = (t: Tool | null): boolean =>
  t === "rect" || t === "circle" || t === "polygon" || t === "slot" || t === "line";

/** The shape preview for the renderer, in the effective role's style (see RenderInput.shapeDraft). */
function shapeDraftView(): RenderInput["shapeDraft"] {
  if (mode !== "draw" || !isShapeTool(tool)) return null;
  const d: NonNullable<RenderInput["shapeDraft"]> = {
    role: effectiveRole(),
    fill: defaultBodyColor,
    outline: [],
    closed: false,
    points: [],
    aux: [],
    text: null,
    target: shapeTarget,
    hint: null,
  };
  const cur = cursor ? shapePointAt(cursor).at : null;
  const a = shapePts[0];
  switch (tool) {
    case "polyline":
      if (draftBody.length) {
        d.outline = cur ? [...draftBody, cur] : [...draftBody];
        d.points = [...draftBody];
      }
      break;
    case "rect":
      if (a && cur) {
        d.outline = rectCorners(a, cur);
        d.closed = true;
        d.points = [a];
      }
      break;
    case "circle":
      if (a && cur) {
        d.outline = circlePoints(a, dist(a, cur));
        d.closed = true;
        d.aux = [[a, cur]];
        d.points = [a];
      }
      break;
    case "polygon":
      d.hint = `${polySides} sides`;
      if (a && cur) {
        d.outline = regularPolygon(a, cur, polySides);
        d.closed = true;
        d.aux = [[a, cur]];
        d.points = [a];
      }
      break;
    case "slot":
      if (a && shapePts.length === 1 && cur) {
        d.outline = [a, cur];
        d.points = [a];
      } else if (shapePts.length >= 2 && cursor) {
        const b = shapePts[1];
        d.outline = roundedConvexBody([a, b], slotWidthAt(a, b, cursor) / 2);
        d.closed = true;
        d.aux = [[a, b]];
        d.points = [a, b];
      }
      break;
    case "line":
      if (a && cur) {
        d.outline = [a, cur];
        d.points = [a];
      }
      break;
    case "arc":
      if (a && shapePts.length === 1 && cur) {
        d.outline = [a, cur];
        d.points = [a];
      } else if (shapePts.length >= 2 && cur) {
        const b = shapePts[1];
        const arc = arcThrough(a, cur, b);
        d.outline = arc ? sampleArc(arc, 48) : [a, cur, b];
        d.aux = [[a, b]];
        d.points = [a, b];
      }
      break;
    case "text":
      break;
  }
  return d;
}

// --- text labels: inline editor -------------------------------------------------------
const textEditInput = document.getElementById("text-edit") as HTMLInputElement;
/** The label being typed: where it goes (and the body it rides), or the existing label being edited. */
let textEdit: { at: Vec2; bodyId: number | null; guideId: number | null } | null = null;

function openTextEditor(at: Vec2, bodyId: number | null, guideId: number | null, initial = ""): void {
  closeDimEditor();
  textEdit = { at, bodyId, guideId };
  const sp = worldToScreen(view, at);
  textEditInput.style.left = `${sp.x}px`;
  textEditInput.style.top = `${sp.y}px`;
  textEditInput.value = initial;
  textEditInput.classList.remove("hidden");
  textEditInput.focus();
  textEditInput.select();
}

function closeTextEditor(): void {
  textEdit = null;
  textEditInput.classList.add("hidden");
  textEditInput.blur();
}

/** Enter / blur: create the label (Text tool) or rewrite the edited one. Empty text does nothing. */
function commitTextEditor(): void {
  const te = textEdit;
  const raw = textEditInput.value;
  closeTextEditor(); // nulls textEdit first, so the blur listener doesn't re-commit
  if (!te) return;
  if (te.guideId !== null) {
    if (raw.trim() && scene.setGuideText(te.guideId, raw)) markDirty();
    return;
  }
  if (!raw.trim()) return; // nothing typed — the tool stays armed
  commitReference(scene.addGuideText(te.at, raw, textSize, te.bodyId));
}

textEditInput.addEventListener("keydown", (e) => {
  e.stopPropagation(); // keep canvas shortcuts (tools, Delete…) out of the text field
  if (e.key === "Enter") commitTextEditor();
  else if (e.key === "Escape") closeTextEditor();
});
textEditInput.addEventListener("blur", () => {
  if (textEdit) commitTextEditor();
});

// --- pattern tools ---------------------------------------------------------------
const PATTERN_DEFAULT_LINEAR_COUNT = 3;
const PATTERN_DEFAULT_CIRCULAR_COUNT = 6;
/** Perpendicular offset of a linear pattern's dimension line from its axis (screen px). */
const PATTERN_DIM_OFFSET_PX = 28;

const isPatternTool = (t: Tool | null): t is "patternLinear" | "patternCircular" =>
  t === "patternLinear" || t === "patternCircular";

/** Short number for pattern labels (two decimals, trailing zeros trimmed). */
function fmtNum(v: number): string {
  return String(Math.round(v * 100) / 100);
}

/** Stage-aware hint for the pattern tools. */
function patternHint(): string {
  const circular = tool === "patternCircular";
  if (patternSeeds.length === 0) {
    return `${circular ? "Circular" : "Linear"} pattern: click a hole (inside the cut-out) or a joint on a body to repeat it — Ctrl+click to pick several features of one body and repeat them together. The count / spacing labels are edited on the canvas afterwards (double-click a label; drag the end handle or centre to re-aim).`;
  }
  const n = patternSeeds.length;
  const what = n > 1 ? `${n} features` : patternSeeds[0].kind === "hole" ? "hole" : "joint";
  const more = " Ctrl+click adds or removes features.";
  if (circular) {
    return `Circular pattern of the ${what}: click the centre of rotation (snaps to joints, hole centres, corners and the grid).${more}`;
  }
  if (patternDraft !== null) {
    return `Row created — click where the first instance of a second direction should go to make a grid, or press Enter / Esc to keep a single row.${more} Type the count in the label, or double-click the ×count / spacing labels later.`;
  }
  return `Linear pattern of the ${what}: click where the next instance should go (snaps to the grid / objects).${more}`;
}

/** Whether two UI seeds name the same feature. */
const samePatternSeed = (a: PatternSeed, b: PatternSeed): boolean =>
  a.kind === "hole" ? b.kind === "hole" && a.bodyId === b.bodyId && a.hole === b.hole : b.kind === "joint" && a.jointId === b.jointId;

/** The pattern-tool feature under `p`: a joint wins (as everywhere), else the hole whose cut-out contains the point. */
function patternFeatureAt(p: Vec2): PatternSeed | null {
  const j = scene.jointAt(p, pickRadius());
  if (j) return { kind: "joint", jointId: j.id };
  const hit = scene.holeAt(p);
  return hit ? { kind: "hole", bodyId: hit.body.id, hole: hit.hole } : null;
}

/**
 * Pick a feature for the armed pattern tool — or, `additive` (Ctrl), toggle it in the
 * seed set: several features of one body repeat together, keeping their relative
 * placement. Refused: free joints, component-instance material, a feature already in a
 * pattern, a feature on another body than the seeds so far. While a row already exists
 * (the linear draft) the feature joins / leaves that pattern at once, copies included.
 */
function togglePatternSeed(seed: PatternSeed, additive: boolean): void {
  const body = scene.patternSeedBody(seed);
  if (!body) {
    if (seed.kind === "joint") notify("A pattern repeats a joint across its body — free joints have no body to pattern on.");
    return;
  }
  if (scene.instanceOfBody(body.id)) {
    notify("This belongs to a component instance — edit the definition to pattern it.");
    disarmTool();
    return;
  }
  const at = patternSeeds.findIndex((s) => samePatternSeed(s, seed));
  if (at >= 0) {
    if (!additive) return; // a plain re-click on a seed: nothing to do
    if (patternDraft !== null && !scene.removePatternSeed(patternDraft, seed)) return;
    patternSeeds.splice(at, 1);
    if (patternDraft !== null) {
      if (!scene.getPattern(patternDraft)) patternDraft = null; // the last seed left: the row is gone
      markDirty();
    }
    updateHint();
    return;
  }
  const taken = seed.kind === "joint" ? scene.patternOfJoint(seed.jointId) : scene.patternOfHole(seed.bodyId, seed.hole);
  if (taken) {
    notify(`This ${seed.kind} is already part of a pattern — edit that pattern's labels, or delete it first.`);
    return;
  }
  if (patternSeeds.length && scene.patternSeedBody(patternSeeds[0])?.id !== body.id) {
    notify("A pattern repeats features of one body — this one sits on another body.");
    return;
  }
  if (patternDraft !== null) {
    if (!scene.addPatternSeed(patternDraft, seed)) return;
    markDirty();
  }
  patternSeeds.push(seed);
  updateHint();
}

/**
 * Pattern-tool click. Ctrl held, or no seed yet: pick the feature under the cursor (Ctrl
 * toggles it in the seed set — see togglePatternSeed). Otherwise create the pattern at
 * the clicked layout point (linear: where the next instance goes; circular: the centre)
 * and open its count label for typing. A linear pattern stays armed for an optional
 * second direction (a grid) and for more Ctrl+clicks; Enter / Esc keep it as it is.
 */
function handlePatternClick(p: Vec2): void {
  if (mods.ctrl || patternSeeds.length === 0) {
    const f = patternFeatureAt(p);
    if (f) togglePatternSeed(f, mods.ctrl);
    else if (scene.bodyAt(p)) {
      notify(
        mods.ctrl
          ? "Ctrl+click inside a hole (the cut-out itself) or on a joint to add it to the pattern."
          : "Click inside a hole (the cut-out itself) or on a joint to pattern it.",
        "info"
      );
    }
    return; // empty space: keep the tool armed
  }
  const target = patternTarget(p);
  if (!target) return;
  const seeds = patternSeeds;
  if (tool === "patternCircular") {
    const created = scene.createCircularPattern(seeds, target, PATTERN_DEFAULT_CIRCULAR_COUNT);
    if (!created) return;
    markDirty();
    disarmTool();
    selection = { kind: "pattern", id: created.id };
    openPatternEditorSoon(created.id, "count", 0);
    return;
  }
  if (patternDraft === null) {
    const created = scene.createLinearPattern(seeds, target, PATTERN_DEFAULT_LINEAR_COUNT);
    if (!created) return;
    patternDraft = created.id;
    autoConstrainPatternAxis(created.id, 0);
    markDirty();
    selection = { kind: "pattern", id: created.id };
    updateHint();
    openPatternEditorSoon(created.id, "count", 0);
    return;
  }
  // Second direction: a grid. (A direction parallel to the first is silently ignored.)
  if (!scene.addPatternAxis(patternDraft, target, PATTERN_DEFAULT_LINEAR_COUNT)) return;
  const id = patternDraft;
  autoConstrainPatternAxis(id, 1);
  markDirty();
  disarmTool();
  selection = { kind: "pattern", id };
  openPatternEditorSoon(id, "count", 1);
}

/**
 * Body-style auto-constraint for a pattern direction: drawn within `AUTO_HV_TOL` of an
 * axis, it gets a horizontal / vertical constraint (the solve snaps it exactly).
 */
function autoConstrainPatternAxis(id: number, axis: number): void {
  const info = scene.patternInfo(id);
  const ax = info?.axes[axis];
  if (!info || !ax) return;
  const d = sub(ax.end, info.anchor);
  const ang = Math.abs(Math.atan2(d.y, d.x)); // 0..π
  const kind =
    ang < AUTO_HV_TOL || Math.PI - ang < AUTO_HV_TOL ? ("horizontal" as const)
    : Math.abs(ang - Math.PI / 2) < AUTO_HV_TOL ? ("vertical" as const)
    : null;
  if (kind) tryAddConstraint(scene, kind, { kind: "patternAxis", patternId: id, axis });
}

/** The pattern axis (dotted seed → last-instance line) within pick range of `p`, or null. */
function patternAxisRefAt(p: Vec2): Extract<MeasureRef, { kind: "patternAxis" }> | null {
  const r = pickRadius();
  for (let i = patternViewCache.length - 1; i >= 0; i--) {
    const v = patternViewCache[i];
    for (let a = 0; a < v.axes.length; a++) {
      if (distToSegment(p, v.axes[a].line.a, v.axes[a].line.b) <= r) return { kind: "patternAxis", patternId: v.id, axis: a };
    }
  }
  return null;
}

/** End the pattern tool keeping the pattern it made selected (Enter). */
function finishPatternTool(): void {
  const keep = patternDraft;
  disarmTool();
  if (keep !== null && scene.getPattern(keep)) selection = { kind: "pattern", id: keep };
}

/**
 * The layout point a click / the cursor means: placement-snapped (objects, guides, grid).
 * Null when it coincides with the seed's anchor (no direction / a degenerate centre).
 */
function patternTarget(p: Vec2): Vec2 | null {
  if (patternSeeds.length === 0) return null;
  const anchor = scene.patternSeedAnchor(patternSeeds[0]);
  if (!anchor) return null;
  const at = placeSnap(p);
  return dist(at, anchor) < 1e-6 ? null : at;
}

/** Pattern-tool overlay for the renderer: hover candidate, seeds, layout point, instances. */
function patternPreviewView(): RenderInput["patternPreview"] {
  if (mode !== "draw" || !isPatternTool(tool)) return null;
  const kind = tool === "patternCircular" ? "circular" : "linear";
  // Hover feedback while picking — before the first seed, and whenever Ctrl is held to add
  // or remove seeds (the layout preview waits until Ctrl is released): the hole under the
  // cursor (joints highlight anyway).
  const picking = patternSeeds.length === 0 || mods.ctrl;
  let candidate: NonNullable<RenderInput["patternPreview"]>["candidate"] = null;
  if (picking && cursor && hoverJoint === null) {
    const hit = scene.holeAt(cursor);
    if (hit) candidate = { loop: scene.bodyHolesWorld(hit.body)[hit.hole], point: cursor };
  }
  if (patternSeeds.length === 0) {
    return candidate ? { kind, anchor: null, seedLoops: [], seedPoints: [], candidate, target: null, instances: [] } : null;
  }
  const anchor = scene.patternSeedAnchor(patternSeeds[0]);
  if (!anchor) return null;
  const seedLoops: Vec2[][] = [];
  const seedPoints: Vec2[] = [];
  for (const s of patternSeeds) {
    if (s.kind === "hole") {
      const loop = scene.bodyHolesWorld(scene.getBody(s.bodyId)!)[s.hole];
      if (loop) seedLoops.push(loop);
    } else {
      const j = scene.getJoint(s.jointId);
      if (j) seedPoints.push(scene.jointWorld(j));
    }
  }
  const target = cursor && !mods.ctrl ? patternTarget(cursor) : null;
  const pv = !target
    ? null
    : kind === "circular"
    ? scene.patternPreview(patternSeeds, { kind: "circular", centre: target, count: PATTERN_DEFAULT_CIRCULAR_COUNT })
    : scene.patternPreview(patternSeeds, {
        kind: "linear",
        target,
        count: PATTERN_DEFAULT_LINEAR_COUNT,
        ...(patternDraft !== null ? { axis: patternDraft } : {}),
      });
  return { kind, anchor, seedLoops, seedPoints, candidate, target, instances: pv?.instances ?? [] };
}

// --- pattern labels / handles (draw mode) ----------------------------------------
/** Last frame's pattern overlays, for hit-testing labels and handles. */
let patternViewCache: PatternView[] = [];

/** One pattern's canvas overlay: dimension-style axis lines with count + spacing labels, or
 *  the centre with count / angle / rotation labels; members that don't fit are flagged. */
function patternViewOf(id: number): PatternView | null {
  const info = scene.patternInfo(id);
  if (!info) return null;
  const px = (n: number) => n / view.scale;
  const selected = selection?.kind === "pattern" && selection.id === id;
  const axes: PatternView["axes"] = info.axes.map((ax, i) => {
    const dir = normalize(sub(ax.end, info.anchor));
    let n = perp(dir);
    const other = info.axes[1 - i];
    // The dimension line sits outside the grid (away from the other axis), or above a lone row.
    if (other) {
      if (dot(n, sub(other.end, info.anchor)) > 0) n = scale(n, -1);
    } else if (n.y > 0) n = scale(n, -1);
    const off = scale(n, px(PATTERN_DIM_OFFSET_PX));
    // The spacing dimension spans the first step only (seed → 2nd instance); the dotted
    // axis line runs on to the last instance, where the count label and handle sit.
    const first = add(info.anchor, scale(dir, ax.step));
    const a = add(info.anchor, off);
    const b = add(first, off);
    return {
      line: { a: info.anchor, b: ax.end },
      dim: { a, b },
      ext: [{ a: info.anchor, b: a }, { a: first, b }],
      stepLabel: add(a, scale(dir, ax.step / 2)),
      stepText: fmtNum(ax.step),
      countLabel: add(add(ax.end, off), scale(dir, px(22))),
      countText: `×${ax.count}`,
      handle: ax.end,
    };
  });
  let circular: PatternView["circular"] = null;
  if (info.circular) {
    const c = info.circular;
    const col = px(34);
    circular = {
      centre: c.centre,
      radius: c.radius,
      countLabel: add(c.centre, vec(col, -px(20))),
      countText: `×${c.count}`,
      angleLabel: add(c.centre, vec(col, 0)),
      angleText: c.angleDeg === null ? "even" : `${fmtNum(c.angleDeg)}°`,
      rotateLabel: add(c.centre, vec(col, px(20))),
      rotateText: c.rotate ? "↻ turn" : "↑ fixed",
    };
  }
  return { id, selected, anchor: info.anchor, axes, circular, bad: info.members.filter((m) => !m.ok).map((m) => m.point) };
}

/** Every pattern's overlay for this frame (draw mode only); refreshes the pick cache. */
function patternViews(): PatternView[] {
  if (mode !== "draw") {
    patternViewCache = [];
    return [];
  }
  patternViewCache = scene.patterns
    .map((p) => patternViewOf(p.id))
    .filter((v): v is PatternView => v !== null);
  return patternViewCache;
}

type PatternLabelHit = { id: number; field: "count" | "step" | "angle" | "rotate"; axis: number };

/** The pattern label under `p` (last frame's layout), or null. */
function patternLabelAt(p: Vec2): PatternLabelHit | null {
  const r = LABEL_PICK_RADIUS / view.scale;
  for (let i = patternViewCache.length - 1; i >= 0; i--) {
    const v = patternViewCache[i];
    if (!patternInScope(v.id)) continue; // faded with the surroundings of an open group
    for (let a = 0; a < v.axes.length; a++) {
      if (dist(v.axes[a].countLabel, p) <= r) return { id: v.id, field: "count", axis: a };
      if (dist(v.axes[a].stepLabel, p) <= r) return { id: v.id, field: "step", axis: a };
    }
    if (v.circular) {
      if (dist(v.circular.countLabel, p) <= r) return { id: v.id, field: "count", axis: 0 };
      if (dist(v.circular.angleLabel, p) <= r) return { id: v.id, field: "angle", axis: 0 };
      if (dist(v.circular.rotateLabel, p) <= r) return { id: v.id, field: "rotate", axis: 0 };
    }
  }
  return null;
}

/** A handle of the *selected* pattern under `p`: an axis end (re-aim / re-space) or the centre. */
function patternHandleAt(p: Vec2): { id: number; axis: number | "centre" } | null {
  if (selection?.kind !== "pattern") return null;
  const id = selection.id;
  const v = patternViewCache.find((x) => x.id === id) ?? patternViewOf(id);
  if (!v) return null;
  const r = pickRadius();
  for (let a = 0; a < v.axes.length; a++) if (dist(v.axes[a].handle, p) <= r) return { id, axis: a };
  if (v.circular && dist(v.circular.centre, p) <= r) return { id, axis: "centre" };
  return null;
}

// --- inline pattern-label editing (shares the dimension editor's input) ----------------
let patternEdit: { id: number; field: "count" | "step" | "angle"; axis: number } | null = null;

/** Open the floating input over a pattern label (double-click, or right after creation). */
function openPatternEditor(id: number, field: "count" | "step" | "angle", axis: number): void {
  const v = patternViewOf(id);
  const info = scene.patternInfo(id);
  if (!v || !info) return;
  let pos: Vec2;
  let text: string;
  if (info.kind === "linear") {
    const ax = info.axes[axis];
    if (!ax || field === "angle") return;
    pos = field === "count" ? v.axes[axis].countLabel : v.axes[axis].stepLabel;
    text = field === "count" ? String(ax.count) : fmtNum(ax.step);
  } else {
    const c = info.circular!;
    if (field === "step") return;
    pos = field === "count" ? v.circular!.countLabel : v.circular!.angleLabel;
    text = field === "count" ? String(c.count) : c.angleDeg === null ? "even" : fmtNum(c.angleDeg);
  }
  closeDimEditor();
  patternEdit = { id, field, axis };
  const sp = worldToScreen(view, pos);
  dimEditInput.style.left = `${sp.x}px`;
  dimEditInput.style.top = `${sp.y}px`;
  dimEditInput.value = text;
  dimEditInput.classList.remove("hidden");
  dimEditInput.focus();
  dimEditInput.select();
}

/**
 * Open the pattern editor once the current mouse event has finished: opening it inside a
 * mousedown handler would be undone at once (the browser moves focus on mousedown, and
 * the input's blur commits + closes it).
 */
function openPatternEditorSoon(id: number, field: "count" | "step" | "angle", axis: number): void {
  window.setTimeout(() => {
    if (mode === "draw" && scene.getPattern(id)) openPatternEditor(id, field, axis);
  }, 0);
}

/** Commit the pattern-label editor: count (whole number ≥ 2), spacing (> 0) or angle (degrees / "even"). */
function commitPatternEditor(): void {
  const edit = patternEdit;
  const raw = dimEditInput.value.trim();
  closeDimEditor(); // clears patternEdit first, so the blur listener doesn't re-commit
  if (!edit || !scene.getPattern(edit.id)) return;
  const info = scene.patternInfo(edit.id);
  if (!info || raw === "" && edit.field !== "angle") return;
  let ok = false;
  if (edit.field === "count") {
    const n = Number(raw);
    ok = Number.isInteger(n) && n >= 2 &&
      (info.kind === "linear" ? scene.setPatternAxisCount(edit.id, edit.axis, n) : scene.setPatternCount(edit.id, n));
    if (!ok) notify("The count must be a whole number of 2 or more.");
  } else if (edit.field === "step") {
    const v = Number(raw);
    ok = Number.isFinite(v) && v > 0 && scene.setPatternAxisStep(edit.id, edit.axis, v);
    if (!ok) notify("The spacing must be a positive length.");
  } else if (raw === "" || raw.toLowerCase().startsWith("e")) {
    ok = scene.setPatternAngle(edit.id, null);
  } else {
    const d = Number(raw.replace("°", ""));
    ok = Number.isFinite(d) && scene.setPatternAngle(edit.id, d);
    if (!ok) notify('The angle must be a non-zero number of degrees (counter-clockwise), or "even".');
  }
  if (ok) markDirty();
}

/**
 * The outline the Split tool cuts: the editable control polygon of a fillet-mode body,
 * or the sampled shape of an offset-mode one (its control polygon is smaller than the
 * drawn shape — the model bakes such bodies before splitting).
 */
function splitOutlineOf(body: Body): Vec2[] {
  return body.round === "offset" ? scene.bodyWorldVerts(body) : scene.bodyControlWorld(body);
}

/** Nearest corner (preferred) or edge point of `body`'s split outline within the pick radius. */
function splitOutlineHit(body: Body, p: Vec2): Vec2 | null {
  const verts = splitOutlineOf(body);
  const r = pickRadius();
  let best: Vec2 | null = null;
  let bestD = r;
  for (const v of verts) {
    const d = dist(p, v);
    if (d < bestD) { bestD = d; best = v; }
  }
  if (best) return best;
  for (let i = 0; i < verts.length; i++) {
    const a = verts[i], b = verts[(i + 1) % verts.length];
    const ab = sub(b, a);
    const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / Math.max(lenSq(ab), 1e-9)));
    const q = add(a, scale(ab, t));
    const d = dist(p, q);
    if (d < bestD) { bestD = d; best = q; }
  }
  return best;
}

/**
 * Split tool: the first click lands on a body's outline (topmost body whose corner or
 * edge is under the cursor) and starts the cut there; later clicks inside the body add
 * path vertices (grid-snapped, kept inside); a click back on the outline ends the cut
 * and splits the body. Rejected cuts explain why and restart the draft.
 */
function handleSplitClick(p: Vec2): void {
  if (splitBodyId === null) {
    for (let i = scene.bodies.length - 1; i >= 0; i--) {
      const body = scene.bodies[i];
      const hit = splitOutlineHit(body, p);
      if (!hit) continue;
      if (scene.instanceOfBody(body.id)) {
        notify("This body belongs to a component instance — edit the definition to split it.");
        disarmTool();
        return;
      }
      splitBodyId = body.id;
      splitDraft = [hit];
      return;
    }
    return; // no outline under the cursor — keep the tool armed
  }
  const body = scene.getBody(splitBodyId);
  if (!body) { disarmTool(); return; }
  const last = splitDraft[splitDraft.length - 1];
  const onOutline = splitOutlineHit(body, p);
  if (onOutline && dist(onOutline, last) > 4 / view.scale) {
    const result = scene.splitBody(body.id, [...splitDraft, onOutline]);
    if (result.ok) {
      markDirty();
      disarmTool();
      selection = { kind: "body", id: result.a.id };
    } else {
      notify(`Can't split here: ${result.reason}`);
      splitDraft = [];
      splitBodyId = null;
    }
    return;
  }
  // Interior vertex: grid-snap unless that would leave the body; ignore clicks outside.
  let at = snap(p);
  if (!scene.pointInBody(body, at)) at = p;
  if (!scene.pointInBody(body, at)) return;
  if (dist(at, last) < 4 / view.scale) return;
  splitDraft.push(at);
}

/**
 * Combine the multi-selected bodies into one (polygon union — see
 * `Scene.combineBodies`). The first-selected body survives. Explains a refusal.
 */
function combineSelection(): void {
  if (mode !== "draw") return;
  const ids = multiSel ? [...multiSel.bodies] : [];
  if (ids.length < 2) {
    notify("Select two or more bodies (Ctrl+click, or drag a box) to combine them.");
    return;
  }
  if (selectionTouchesInstance()) {
    notify("Component instances can't be combined — edit the definition, or fork the instance first.");
    return;
  }
  const result = scene.combineBodies(ids);
  if (!result.ok) {
    notify(`Can't combine: ${result.reason}`);
    return;
  }
  multiSel = null;
  selection = { kind: "body", id: result.body.id };
  markDirty();
}

/**
 * Subtract / Intersect over the multi-selected bodies (see `Scene.booleanBodies`): the
 * first-selected body is the subject and survives, the others are the tools and are
 * consumed. Explains a refusal.
 */
function booleanSelection(op: BooleanOp): void {
  if (mode !== "draw") return;
  const ids = multiSel ? [...multiSel.bodies] : [];
  if (ids.length < 2) {
    notify(op === "subtract"
      ? "Select the body to keep first, then Ctrl+click the bodies to subtract from it."
      : "Select two or more bodies (Ctrl+click, or drag a box) to keep only their overlap.");
    return;
  }
  if (selectionTouchesInstance()) {
    notify(`Component instances can't be ${op}ed — edit the definition, or fork the instance first.`);
    return;
  }
  const result = scene.booleanBodies(op, ids);
  if (!result.ok) {
    notify(`Can't ${op}: ${result.reason}`);
    return;
  }
  multiSel = null;
  selection = { kind: "body", id: result.body.id };
  markDirty();
}

// --- pointer events ------------------------------------------------------
canvas.addEventListener("mousedown", (e) => {
  const world = eventWorld(e);
  cursor = world;
  mods = { shift: e.shiftKey, alt: e.altKey, ctrl: e.ctrlKey || e.metaKey };

  if (e.button === 2) {
    // Right button always pans the view.
    e.preventDefault();
    pan = { lastScreen: eventScreen(e) };
    canvas.style.cursor = "grabbing";
    return;
  }

  if (e.button !== 0) return;
  if (viewRotate) {
    // The dial owns the left button: a grab on the ring / an arm starts turning the
    // view, a press anywhere else closes the dial (and does nothing underneath).
    const s = eventScreen(e);
    if (dialHit(s)) {
      viewRotate.drag = { startBearing: dialBearing(s), startAngle: view.angle };
      canvas.style.cursor = "grabbing";
    } else if (dist(s, dialCentre()) > 12) setViewRotateOpen(false);
    return;
  }
  if (mode === "draw") {
    if (placePendingInsert(world)) return; // pending component-instance placement
    if (tool === "rotate") {
      startRotate(world);
    } else if (isPatternTool(tool)) {
      handlePatternClick(world);
    } else if (tool === null) {
      // A selected body shows draggable corner handles — outer outline and holes alike;
      // grabbing one reshapes the body (square = move the vertex, circle = adjust that
      // corner's radius; nearest wins).
      const node = selectedBodyNodeAt(world);
      const fh = selectedBodyFilletHandleAt(world);
      const onFillet = !!fh && selection?.kind === "body" && (!node || dist(world, fh.at) < dist(world, node.at));
      // Ctrl/Cmd+click: toggle what's under the cursor in the multi-selection; on empty
      // space, start an additive box select instead. (Ctrl on a radius handle is the
      // "round every corner" drag — see the fillet drag in mousemove — so it falls through.)
      if ((e.ctrlKey || e.metaKey) && !onFillet) {
        // Ctrl+Shift+drag from empty space extends the selected body's feature selection.
        if (e.shiftKey && featureBoxStartAt(world)) {
          featureBox = { start: world, end: world, additive: true, moved: false };
          return;
        }
        if (!toggleMultiAt(world)) {
          boxSelect = { start: world, end: world, additive: true, moved: false };
        }
        return;
      }
      // A press on a member of the feature selection drags the whole set (a radius
      // handle still wins — it never moves geometry).
      const fref = onFillet ? null : featureHitAt(world);
      if (fref) {
        startFeatureDrag(world, fref);
        return;
      }
      // Shift+drag from empty space with a body selected: box-select its features
      // (a Shift press on a body / joint stays the rigid drag below).
      if (e.shiftKey && featureBoxStartAt(world)) {
        featureBox = { start: world, end: world, additive: false, moved: false };
        return;
      }
      featureSel = null; // any other press rebuilds the selection from what's under the cursor
      if (onFillet && fh && selection?.kind === "body") {
        // A pattern member's handle edits the seed (members are derived from it).
        const hole = fh.hole === null ? null : scene.patternSeedHole(selection.id, fh.hole);
        leftDrag = { kind: "fillet", bodyId: selection.id, index: fh.index, hole, moved: false };
        canvas.style.cursor = "move";
      } else if (node && selection?.kind === "body") {
        // A pattern member's node drags the seed's matching node: the whole array follows.
        const hole = node.hole === null ? null : scene.patternSeedHole(selection.id, node.hole);
        const at = hole === node.hole ? node.at : scene.bodyHoleControlWorld(scene.getBody(selection.id)!, hole!)[node.index];
        // With object snap on, the vertex itself is the snapping reference: a hole
        // centre or a corner lands on other objects' corners / centres / edges.
        const vref: MeasureRef =
          hole === null
            ? { kind: "vertex", bodyId: selection.id, index: node.index }
            : { kind: "vertex", bodyId: selection.id, index: node.index, hole };
        leftDrag = {
          kind: "vertex",
          bodyId: selection.id,
          index: node.index,
          hole,
          grabOffset: sub(world, at),
          moved: false,
          osnap: objSnapEnabled ? { ref: vref, hit: null, hitInfinite: false } : undefined,
          align: newDragAlign(vref),
        };
        canvas.style.cursor = "move";
      } else if (selection?.kind === "pattern" && patternHandleAt(world)) {
        const h = patternHandleAt(world)!;
        leftDrag = { kind: "patternHandle", id: h.id, axis: h.axis, moved: false };
        canvas.style.cursor = "move";
      } else if (!e.shiftKey && clickDimensionGlyph(world)) {
        // The direction glyph on a dimension label is a button: the press cycled the
        // axis and selected the dimension — nothing to drag.
      } else if (e.shiftKey) {
        // Shift+drag: rigid drag — what's grabbed moves like in simulation (grounds
        // hold, connections constrain) while the rest of the scene stays frozen.
        if (!multiHitAt(world)) handleSelectClick(world); // select what's under the cursor first
        startRigidDrag(world); // a miss (empty space / a label) leaves the click as a plain select
      } else if (multiHitAt(world)) {
        // Clicking any element of the multi-selection drags the whole selection.
        startMultiDrag(world);
      } else {
        // Otherwise select what's under the cursor; if it's movable, begin a drag of it.
        handleSelectClick(world);
        if (selection?.kind === "measure") {
          const m = scene.getMeasurement(selection.id)!;
          const anchor = scene.measurementLabelPos(m) ?? world;
          leftDrag = { kind: "measureLabel", id: m.id, grabOffset: sub(world, anchor), moved: false };
          canvas.style.cursor = "move";
        } else if (selection?.kind === "tempDim") {
          const td = getTempDim(selection.id);
          const anchor = (td && tempDimLabelPos(td)) ?? world;
          leftDrag = { kind: "measureLabel", id: selection.id, grabOffset: sub(world, anchor), moved: false, temp: true };
          canvas.style.cursor = "move";
        } else if (selection?.kind === "body") {
          // Object snap on: the reference feature nearest the grab is the anchor (and
          // what snaps); otherwise the centroid or nearest corner grid-snaps.
          const body = scene.getBody(selection.id)!;
          // The reference feature nearest the grab is also what implicit constraints align.
          const pick = pickObjSnapRef([body.id], [], world);
          const os = objSnapEnabled ? pick : null;
          const anchor = os ? os.anchor : bodyDragAnchor(selection.id, world);
          leftDrag = {
            kind: "body",
            id: selection.id,
            anchorOffset: sub(anchor, body.pos),
            grabOffset: sub(world, anchor),
            moved: false,
            osnap: os ? { ref: os.ref, hit: null, hitInfinite: false } : undefined,
            align: newDragAlign(pick?.ref),
          };
          canvas.style.cursor = "move";
        } else if (selection?.kind === "joint") {
          // A pattern member drags as its seed: the whole array moves together.
          const dragId = scene.patternSeedJoint(selection.id);
          const anchor = scene.jointWorld(scene.getJoint(dragId)!);
          leftDrag = {
            kind: "joint",
            id: dragId,
            grabOffset: sub(world, anchor),
            moved: false,
            // A joint is its own object-snap reference.
            osnap: objSnapEnabled ? { ref: { kind: "joint", jointId: dragId }, hit: null, hitInfinite: false } : undefined,
            align: newDragAlign({ kind: "joint", jointId: dragId }),
          };
          canvas.style.cursor = "move";
        } else if (selection?.kind === "guide") {
          // On a defining point: re-aim the line; elsewhere on the line: move it whole.
          const g = scene.getGuide(selection.id)!;
          const gp = scene.guidePointAt(world, pickRadius());
          if (gp && gp.guide.id === g.id) {
            leftDrag = {
              kind: "guidePoint",
              id: g.id,
              which: gp.which,
              grabOffset: sub(world, scene.guidePointWorld(g, gp.which)!),
              moved: false,
            };
          } else {
            leftDrag = { kind: "guide", id: g.id, grabOffset: sub(world, guideAnchorWorld(g)), moved: false };
          }
          canvas.style.cursor = "move";
        } else if (multiHitAt(world)) {
          // A plain click on a grouped body selected its whole group — drag it as one.
          startMultiDrag(world);
        } else if (!selection) {
          // Empty space: begin a box selection (a plain click, no drag, just deselects).
          boxSelect = { start: world, end: world, additive: false, moved: false };
        }
      }
    } else if (isDragShapeTool(tool) && shapePts.length === 0) {
      // Two-point shape tool, nothing placed yet: the press may become a drag (first
      // point → second point), so the first point waits for the release (see mouseup).
      const { at, pick } = shapePointAt(world);
      shapePress = { screen: eventScreen(e), at, pick, ctrl: mods.ctrl, dragging: false };
    } else {
      handleDrawClick(world);
    }
  } else {
    // Sim mode. The measure tool works here too (sim keeps its own measurement set).
    if (tool === "measure") {
      handleMeasureClick(world);
      return;
    }
    // A measurement's value label is the topmost overlay: grab it to reposition,
    // select it to delete — without disturbing the mechanism underneath. Its direction
    // glyph is a button (cycles h / v / direct).
    if (clickDimensionGlyph(world)) return;
    const ml = measurementLabelAt(world);
    if (ml) {
      selection = { kind: "measure", id: ml.id };
      const anchor = scene.measurementLabelPos(ml) ?? world;
      leftDrag = { kind: "measureLabel", id: ml.id, grabOffset: sub(world, anchor), moved: false };
      canvas.style.cursor = "move";
      return;
    }
    // On a slider's start pair, grab the rider: the rail joint is grounded / on the track.
    const j = scene.jointAt(world, pickRadius(), "rider");
    if (j) {
      driver = { jointId: j.id, target: world };
      canvas.style.cursor = "grabbing";
    } else {
      // No joint under the cursor: grab the body itself and drive the grabbed point.
      const b = scene.bodyAt(world);
      if (b) {
        driver = { bodyId: b.id, local: rotate(sub(world, b.pos), -b.angle), target: world };
        canvas.style.cursor = "grabbing";
      }
    }
  }
});

canvas.addEventListener("mousemove", (e) => {
  const world = eventWorld(e);
  cursor = world;
  mods = { shift: e.shiftKey, alt: e.altKey, ctrl: e.ctrlKey || e.metaKey };

  if (pan) {
    const s = eventScreen(e);
    view.tx += s.x - pan.lastScreen.x;
    view.ty += s.y - pan.lastScreen.y;
    pan.lastScreen = s;
    return;
  }

  if (viewRotate) {
    const s = eventScreen(e);
    if (viewRotate.drag) {
      // The turn is the bearing's change since the grab; snap to 5° unless Shift is held.
      let a = viewRotate.drag.startAngle + (dialBearing(s) - viewRotate.drag.startBearing);
      if (!e.shiftKey) a = Math.round(a / VIEW_ROTATE_SNAP) * VIEW_ROTATE_SNAP;
      setViewAngle(a);
      return;
    }
    viewRotate.hot = dialHit(s);
    canvas.style.cursor = viewRotate.hot ? "grab" : "default";
    return;
  }

  if (shapePress) {
    // Press-and-drag shape: once the pointer has clearly moved, the press is the first
    // point (the preview then follows the cursor as the second).
    if (!shapePress.dragging && dist(eventScreen(e), shapePress.screen) > SHAPE_DRAG_PX) {
      shapePress.dragging = true;
      beginShape(shapePress.at, shapePress.ctrl);
      shapePts = [shapePress.at];
      shapeSnaps = [shapePress.pick];
    }
    return;
  }

  if (featureBox) {
    featureBox.end = world;
    if (dist(featureBox.start, world) * view.scale > 4) featureBox.moved = true;
    return;
  }

  if (boxSelect) {
    boxSelect.end = world;
    // Only count it as a box once the pointer has clearly moved (else it's a plain click).
    if (dist(boxSelect.start, world) * view.scale > 4) boxSelect.moved = true;
    return;
  }

  if (rotateDrag) {
    // Accumulate the pointer's swing about the pivot (unwrapped so it survives crossing ±π),
    // snap the resulting absolute body angle to 45°, then apply only the incremental delta.
    const ptr = Math.atan2(world.y - rotateDrag.pivot.y, world.x - rotateDrag.pivot.x);
    rotateDrag.accum += wrapAngle(ptr - rotateDrag.prevPointer);
    rotateDrag.prevPointer = ptr;
    const total = snapAngle(rotateDrag.grabAngle + rotateDrag.accum) - rotateDrag.grabAngle;
    const delta = total - rotateDrag.lastTotal;
    // Every selected body turns about the shared pivot by the same delta (a rigid
    // rotation of the whole selection); selected free joints orbit the pivot with it.
    for (const id of rotateDrag.bodyIds) scene.rotateBody(id, rotateDrag.pivot, delta);
    for (const id of rotateDrag.jointIds) {
      const j = scene.getJoint(id);
      if (!j) continue;
      const w = scene.jointWorld(j);
      const nw = add(rotateDrag.pivot, rotate(sub(w, rotateDrag.pivot), delta));
      scene.moveJoint(id, sub(nw, w));
    }
    rotateDrag.lastTotal = total;
    rotateDrag.moved = true;
    solveSketchLive(); // constraints (e.g. an H edge) pull back against the rotation
    return;
  }

  if (leftDrag) {
    // A measurement label follows the cursor exactly (no grid snap — it's an annotation).
    // Only the label moves: a point–point dimension keeps the h / v / direct axis it was
    // placed with, whatever zone the label lands in.
    if (leftDrag.kind === "measureLabel") {
      if (leftDrag.temp) {
        const td = getTempDim(leftDrag.id);
        if (td) setTempDimLabel(td, sub(world, leftDrag.grabOffset));
      } else scene.setMeasurementLabel(leftDrag.id, sub(world, leftDrag.grabOffset));
      leftDrag.moved = true;
      return;
    }
    // Pattern handle: the axis end / centre lands on the placement snap (objects, guides, grid).
    if (leftDrag.kind === "patternHandle") {
      const to = placeSnap(world);
      const ok =
        leftDrag.axis === "centre"
          ? scene.setPatternCentre(leftDrag.id, to)
          : scene.setPatternAxisEnd(leftDrag.id, leftDrag.axis, to);
      if (ok) leftDrag.moved = true;
      return;
    }
    // Rigid drag: just aim the driver at the cursor — the frame loop runs the scoped
    // solve (sim-style, so sketch constraints don't apply; a grabbed joint's target
    // snaps to the grid, a grabbed body point follows the cursor exactly).
    if (leftDrag.kind === "rigid") {
      leftDrag.driver.target = leftDrag.driver.jointId !== undefined ? snap(world) : world;
      leftDrag.moved = true;
      return;
    }
    // Fillet-handle drag: the cursor's position maps straight to that corner's radius
    // (absolute per move, so the handle tracks the cursor; the sketch is untouched —
    // a radius change never moves control vertices or joints).
    if (leftDrag.kind === "fillet") {
      const body = scene.getBody(leftDrag.bodyId);
      const r = body ? filletDragRadius(body, leftDrag.index, world, leftDrag.hole) : null;
      if (r !== null) {
        const { bodyId, index, hole } = leftDrag;
        // A disk (one-point offset outline) has one radius: set it as the outline default
        // so a diameter dimension / `[` `]` keys and the rim handle all move the same value.
        if (body && scene.diskOfRef({ kind: "vertex", bodyId: body.id, index: 0, hole: hole ?? undefined })) {
          scene.setDiskRadius(bodyId, r, hole);
          demoteSizeDims(bodyId, hole, null); // the handle now sets the size
          propagateEqualRadii(hole === null ? { kind: "disk", bodyId } : { kind: "disk", bodyId, hole });
        } else if (e.ctrlKey || e.metaKey) {
          // Ctrl: every corner of this outline follows the handle — the default is set
          // and per-corner overrides dropped, so the corners read as uniform from here on
          // (a radius dimension on any of them then drives them all).
          scene.setOutlineRadiusUniform(bodyId, r, hole);
          demoteSizeDims(bodyId, hole, null);
          // Every corner changed: each carries the radii made Equal to it along.
          for (const corner of scene.outlineCorners(bodyId, hole))
            propagateEqualRadii(hole === null ? { kind: "vertex", bodyId, index: corner.index } : { kind: "vertex", bodyId, index: corner.index, hole });
        } else {
          scene.setBodyCornerRadius(bodyId, index, r, hole);
          demoteSizeDims(bodyId, hole, (i) => i === index); // the handle now sets this corner
          propagateEqualRadii(hole === null ? { kind: "vertex", bodyId, index } : { kind: "vertex", bodyId, index, hole });
        }
        leftDrag.moved = true;
      }
      return;
    }
    // A dragged guide point lands on joints / body corners / other guides' points
    // (exact, like placement) before falling back to the grid/guide snap — its own
    // guideline is excluded from every pick (it can't snap to itself).
    if (leftDrag.kind === "guidePoint") {
      const raw = sub(world, leftDrag.grabOffset);
      const pick = constraintPointRefAt(raw, leftDrag.id);
      const res = pick ? scene.resolveMeasureRef(pick) : null;
      const to = res?.kind === "point" ? res.p : snap(raw, leftDrag.id);
      scene.moveGuidePoint(leftDrag.id, leftDrag.which, to);
      // A reference circle's rim handle is a direct resize — radii made Equal to it
      // follow; so do the radii made equal to an arc, whose every point sets its radius.
      const g = scene.getGuide(leftDrag.id);
      if (g && (g.kind === "arc" || (g.kind === "circle" && leftDrag.which === "r"))) propagateEqualRadii({ kind: "guideCircle", guideId: g.id });
      leftDrag.moved = true;
      solveSketchLive(); // constraints on the guide hold while it follows
      return;
    }
    // Snap the dragged anchor (in absolute terms, preserving where it was grabbed): the
    // object-snap reference onto the other objects' features when enabled and something
    // is in range, else the grid / guidelines. A dragged guideline is left out of the
    // snap targets (it can't snap to itself).
    const raw = sub(world, leftDrag.grabOffset);
    const osnapped = "osnap" in leftDrag && leftDrag.osnap ? objSnapTarget(leftDrag, leftDrag.osnap, raw) : null;
    const target = osnapped ?? snap(raw, leftDrag.kind === "guide" ? leftDrag.id : undefined);
    const delta = sub(target, dragAnchorWorld(leftDrag));
    if (leftDrag.kind === "vertex") scene.moveBodyVertex(leftDrag.bodyId, leftDrag.index, delta, leftDrag.hole);
    else if (leftDrag.kind === "body") scene.moveBody(leftDrag.id, delta);
    else if (leftDrag.kind === "guide") scene.moveGuide(leftDrag.id, delta);
    else if (leftDrag.kind === "multi") {
      // Move the whole multi-selection together by the same delta (the anchor is
      // re-read live in dragAnchorWorld, so the delta always closes the real gap).
      for (const id of leftDrag.bodies) scene.moveBody(id, delta);
      for (const id of leftDrag.joints) scene.moveJoint(id, delta);
    } else if (leftDrag.kind === "features") moveFeatures(leftDrag, delta);
    else scene.moveJoint(leftDrag.id, delta); // an arrow start brings its carriage home (model)
    leftDrag.moved = true;
    solveSketchLive(); // sketch dragging: constraints hold while the geometry follows
    if ("align" in leftDrag && leftDrag.align) {
      leftDrag.align.slip = sub(raw, target); // where the cursor put it vs. where it snapped
      updateDragAlign(leftDrag, leftDrag.align, performance.now());
    }
    return;
  }

  hoverJoint = scene.jointAt(world, pickRadius(), mode === "sim" ? "rider" : "rail")?.id ?? null;
  // In normal/select mode, also highlight the body under the cursor (when no joint is).
  hoverBody =
    mode === "draw" && tool === null && hoverJoint === null
      ? scene.bodyAt(world)?.id ?? null
      : null;
  // Object snap preview: which feature a plain drag from here would snap by (Shift = rigid drag, no snap).
  hoverObjSnap = mode === "draw" && tool === null && objSnapEnabled && !e.shiftKey ? hoverObjSnapRef(world) : null;
  // Hint that elements are grabbable: a move cursor over a joint/body/handle/label in select mode.
  if (mode === "draw" && tool === null) {
    const grabbable =
      selectedBodyNodeAt(world) !== null ||
      selectedBodyFilletHandleAt(world) !== null ||
      hoverJoint !== null ||
      hoverBody !== null ||
      measurementLabelAt(world) !== null ||
      patternLabelAt(world) !== null ||
      patternHandleAt(world) !== null ||
      scene.guidePointAt(world, pickRadius()) !== null ||
      scene.guideAt(world, pickRadius()) !== undefined;
    // With Shift held a drag would be rigid (sim-style), so hint with the sim grab cursor.
    canvas.style.cursor = grabbable ? (e.shiftKey ? "grab" : "move") : "crosshair";
    // The direction glyph on a dimension label is a button (a click cycles h / v / direct).
    if (!e.shiftKey && (measurementGlyphAt(world) || tempDimGlyphAt(world))) canvas.style.cursor = "pointer";
  }
  // Rotate tool: a grab cursor over a node of the selected body or any body.
  if (mode === "draw" && tool === "rotate") {
    const rotatable = selectedBodyNodeAt(world) !== null || scene.bodyAt(world) !== undefined;
    canvas.style.cursor = rotatable ? "grab" : "crosshair";
  }
  if (mode === "sim") {
    if (driver) driver.target = world;
    else if (tool === "measure") canvas.style.cursor = "crosshair";
    else if (measurementLabelAt(world)) canvas.style.cursor = measurementGlyphAt(world) ? "pointer" : "move";
    else {
      // Hint that joints and bodies are both grabbable to drive the mechanism.
      const grabbable = hoverJoint !== null || scene.bodyAt(world) !== undefined;
      canvas.style.cursor = grabbable ? "grab" : "default";
    }
  }
});

window.addEventListener("mouseup", (e) => {
  if (e.button === 0 && viewRotate?.drag) {
    viewRotate.drag = null;
    canvas.style.cursor = viewRotate.hot ? "grab" : "default";
  }
  if (e.button === 0 && shapePress) {
    const press = shapePress;
    shapePress = null;
    if (press.dragging) handleShapeClick(eventWorld(e)); // the release is the second point
    else {
      // A plain click: the first point, placed now (the click flow continues on the next click).
      beginShape(press.at, press.ctrl);
      shapePts = [press.at];
      shapeSnaps = [press.pick];
    }
  }
  if (e.button === 2 && pan) {
    pan = null;
    canvas.style.cursor = defaultCursor();
  }
  if (e.button === 0 && featureBox) {
    // A dragged box selects the body's features inside it; a plain Shift+click on empty
    // space deselects, as it always did (Ctrl+Shift+click on empty space is a no-op).
    if (featureBox.moved) applyFeatureBox(featureBox.additive);
    else if (!featureBox.additive) {
      selection = null;
      featureSel = null;
    }
    featureBox = null;
    canvas.style.cursor = defaultCursor();
  }
  if (e.button === 0 && boxSelect) {
    // A dragged box selects its contents; a plain click already deselected on mousedown
    // (and a Ctrl+click on empty space is a no-op).
    if (boxSelect.moved) applyBoxSelect();
    boxSelect = null;
    canvas.style.cursor = defaultCursor();
  }
  if (e.button === 0 && leftDrag) {
    const finished = leftDrag;
    leftDrag = null; // cleared first: the settle solve below must run un-anchored
    if (finished.moved) {
      // A rigid drag solves in the frame loop; run one last solve so the pose the user
      // released at is exactly the pose that gets persisted.
      if (finished.kind === "rigid") {
        timedSolve("rigidDrag", finished.driver, 100, undefined, finished.freeze);
      } else if (finished.kind === "fillet") {
        // A radius change moves no control vertex or joint, so the settle is only for
        // what the sketch says about rims: a tangent line follows the resized disk, or
        // a disk made Equal to the dragged radius (and any tangent on that one).
        if (sketchNamesRadii()) solveSketchLive();
      } else if (finished.kind !== "measureLabel" && finished.kind !== "patternHandle") {
        // Settle: one symmetric sketch solve at rest, repairing anything the anchored
        // live solves couldn't satisfy without moving the dragged geometry.
        solveSketchLive();
      }
      // Persist a reposition (a plain click just selects). A temporary context
      // dimension's label isn't document state — nothing to record.
      if (!(finished.kind === "measureLabel" && finished.temp)) markDirty();
      // Released on a previewed alignment: place the implicit constraint (after the
      // settle, so it's added to — and solved against — the resting geometry).
      if ("align" in finished && finished.align) placeAlignConstraint(finished, finished.align);
    }
    canvas.style.cursor = defaultCursor();
  }
  if (e.button === 0 && rotateDrag) {
    const didRotate = rotateDrag.moved;
    rotateDrag = null; // cleared first, as above
    if (didRotate) {
      solveSketchLive(); // symmetric settle at rest
      markDirty(); // a plain click (no drag) only selected the body
    }
    canvas.style.cursor = defaultCursor();
  }
  if (e.button === 0 && mode === "sim" && driver) {
    driver = null;
    canvas.style.cursor = defaultCursor();
  }
});

canvas.addEventListener("contextmenu", (e) => e.preventDefault());

canvas.addEventListener(
  "wheel",
  (e) => {
    e.preventDefault();
    // Wheel up (deltaY < 0) zooms in, anchored at the cursor.
    zoomAt(view, eventScreen(e), Math.exp(-e.deltaY * 0.0015));
  },
  { passive: false }
);

canvas.addEventListener("mouseleave", () => {
  cursor = null;
});

canvas.addEventListener("dblclick", (e) => {
  if (viewRotate) {
    // Double-click the dial's centre to go back to 0°; elsewhere the dial swallows it.
    if (dist(eventScreen(e), dialCentre()) <= 12) setViewAngle(0);
    return;
  }
  featureSel = null; // a node / edge edit shifts vertex indices — the feature selection can't follow
  // Freehand polygons still close on double-click; joint-built bodies finish by
  // clicking a previously-added node (handled in handleBodyClick).
  if (mode === "draw" && tool === "polyline" && jointDraftIds.length === 0) {
    finishPolyline();
    return;
  }
  // Select mode: double-click a draw-mode dimension label to edit its value inline
  // (typing a number makes it a driving dimension; clearing it makes it driven again).
  if (mode === "draw" && tool === null) {
    const rt = regularTagAt(eventWorld(e));
    if (rt) {
      leftDrag = null;
      openSidesEditor(rt);
      return;
    }
    const pl = patternLabelAt(eventWorld(e));
    if (pl && pl.field !== "rotate") {
      leftDrag = null;
      openPatternEditor(pl.id, pl.field, pl.axis);
      return;
    }
    // (A double-click on a label's direction glyph is two button presses — the axis
    // cycled twice on the mousedowns — not a request to edit the value.)
    const tl = tempDimLabelAt(eventWorld(e));
    if (tl) {
      leftDrag = null;
      if (!tempDimGlyphAt(eventWorld(e))) openTempDimEditor(tl); // one-shot move of the live side to the typed value
      return;
    }
    const ml = measurementLabelAt(eventWorld(e));
    if (ml) {
      leftDrag = null; // the double-click's mousedowns started a label drag — cancel it
      if (!measurementGlyphAt(eventWorld(e))) openDimEditor(ml);
      return;
    }
    // Double-click a text label to edit its text in place.
    const tg = scene.guideAt(eventWorld(e), pickRadius());
    if (tg && tg.kind === "text") {
      leftDrag = null;
      const at = scene.guideTextWorld(tg);
      if (at) openTextEditor(at.p, null, tg.id, tg.text);
      return;
    }
    // Double-click a component instance to open its definition for editing — through
    // this instance, so the context ghost can place the surroundings; with Ctrl the
    // whole enclosing assembly shows faded straight away.
    const world = eventWorld(e);
    const dj = scene.jointAt(world, pickRadius());
    const b = scene.bodyAt(world);
    const inst = b ? scene.instanceOfBody(b.id) : undefined;
    if (inst) {
      leftDrag = null; // cancel the drag the double-click's mousedowns started
      enterComponent(inst.defId, inst.id, e.ctrlKey || e.metaKey);
      return;
    }
    // Double-click a grouped body / joint to edit *inside* the group: its parts become
    // individually selectable and everything else fades out. Already inside it, the
    // double-click falls through to the ordinary node editing below.
    const dg =
      dj && dj.bodyId === null
        ? scene.groupOfJoint(dj.id)
        : dj
          ? scene.groupOf(dj.bodyId!)
          : b
            ? scene.groupOf(b.id)
            : undefined;
    // (A component instance's own group — its chassis — isn't enterable: the instance's
    // parts belong to the definition, so a double-click there opens *that* instead.)
    const dgOwned =
      !!dg && (dg.bodyIds.some((id) => scene.instanceOfBody(id)) || dg.jointIds.some((id) => scene.instanceOfJoint(id)));
    if (dg && !dgOwned && dg.id !== groupEdit) {
      leftDrag = null;
      enterGroup(dg.id);
      return;
    }
    // Inside a group, a double-click on empty space (or on the veiled surroundings)
    // leaves it — the same gesture that entered it, one level out.
    if (groupEdit !== null && !dj && !b) {
      leftDrag = null;
      leaveGroup();
      return;
    }
  }
  // Select mode, body selected: double-click edits the control polygon. On a vertex →
  // remove it (kept ≥ 3); on an edge → add a node at the click point (grid-snapped).
  if (mode === "draw" && tool === null && selection?.kind === "body") {
    const world = eventWorld(e);
    const node = selectedBodyNodeAt(world);
    // Double-click a radius handle (when it isn't shadowed by a nearer vertex square):
    // clear that corner's override, back to the outline default.
    const fh = selectedBodyFilletHandleAt(world);
    if (fh && (!node || dist(world, fh.at) < dist(world, node.at))) {
      leftDrag = null; // cancel the fillet drag the double-click's mousedowns started
      // (A pattern member's handle edits the seed — members copy its corner radii.)
      scene.setBodyCornerRadius(selection.id, fh.index, null, fh.hole === null ? null : scene.patternSeedHole(selection.id, fh.hole));
      markDirty();
      return;
    }
    {
      // A regular polygon keeps its corner count: nodes can't be added or removed.
      const rBody = scene.getBody(selection.id);
      const rEdge = node ? null : selectedBodyEdgeAt(world);
      const rHole = node ? node.hole : rEdge ? rEdge.hole : undefined;
      if (rBody && rHole !== undefined && scene.outlineRegular(rBody, rHole === null ? null : scene.patternSeedHole(selection.id, rHole)) !== null) {
        notify("A regular polygon keeps its corners — change its side count instead (double-click the count tag, or ↑ / ↓).");
        return;
      }
    }
    if (node) {
      // Removing the last removable node of a hole deletes the hole itself (a fillet
      // hole keeps ≥ 3 vertices, a disk keeps its 1 — so a no-op removal means "the
      // user wants the hole gone"). On a pattern member the node edit goes to the seed
      // (every member follows); deleting a member deletes the whole array, seed kept.
      const body = scene.getBody(selection.id)!;
      const ph = node.hole === null ? undefined : scene.patternOfHole(selection.id, node.hole);
      const hole = node.hole === null ? null : scene.patternSeedHole(selection.id, node.hole);
      const before = hole === null ? null : body.holes?.[hole]?.controlLocal.length;
      scene.removeBodyVertex(selection.id, node.index, hole);
      const after = hole === null ? null : body.holes?.[hole]?.controlLocal.length;
      if (hole !== null && before !== undefined && before === after) {
        if (ph?.role === "member") scene.removePattern(ph.pattern.id);
        else scene.removeBodyHole(selection.id, hole);
      }
      markDirty();
      return;
    }
    const edge = selectedBodyEdgeAt(world);
    if (edge) {
      const hole = edge.hole === null ? null : scene.patternSeedHole(selection.id, edge.hole);
      // A node added on a member's edge is added on the seed's matching edge (members follow).
      const point = hole === edge.hole
        ? edge.point
        : scene.bodyHoleControlWorld(scene.getBody(selection.id)!, hole!)[edge.index] ?? edge.point;
      scene.insertBodyVertex(selection.id, edge.index, hole === edge.hole ? snap(edge.point) : point, hole);
      markDirty();
    }
  }
});

// --- the command registry's other half: what each command does ---------------
/**
 * The table of what the app can be *asked* to do — ids, labels, groups, contexts, the
 * tool a command arms and the keys it ships with — is `src/commands.ts`, and
 * `public/keymap.json` is generated from it. The **actions** stay here, where the key
 * handler they replace used to be: every one of them closes over this module's state
 * (`tool`, `selection`, `mode`, `view`, `scene`, `leftDrag`…), and handing all of that to
 * another module would need a facade far bigger than the win. `Record<CommandId, …>` is
 * what holds the two halves together — add a command to the table and the compiler asks
 * for its action here.
 *
 * `run` wins over the spec's `tool`, so a command can arm a tool *and* do something first
 * (the two polyline presets set the role). `enabled` guards a command: when it says no
 * the keystroke is left alone rather than swallowed, and the handful of commands that
 * branch internally — Esc, Delete, ↑ / ↓, Enter, [ / ] — stay **one** command each
 * rather than several bindings that would race each other.
 */
interface CommandAction {
  run?: () => void;
  enabled?: () => boolean;
}

/** Esc: step back out of whatever is innermost. */
function cancelOrStepOut(): void {
  if (!keymapPanel.classList.contains("hidden")) {
    setKeymapPanelVisible(false);
    return;
  }
  if (!exportPanel.classList.contains("hidden")) {
    setExportPanelVisible(false);
    return;
  }
  if (viewRotate) {
    setViewRotateOpen(false);
    return;
  }
  // Mid-drag with armed alignment candidates: Esc only drops the newest one — the drag
  // goes on, and its release creates whatever is still armed (press Esc again to clear
  // that too). Parking the hover timer at +∞ keeps the same element from re-arming
  // while the cursor still rests on it; the re-run re-previews what is left.
  if (leftDrag && "align" in leftDrag && leftDrag.align?.cands.length) {
    const al = leftDrag.align;
    al.cands.pop();
    if (al.hover) al.hover.since = Infinity;
    updateDragAlign(leftDrag, al, performance.now());
    return;
  }
  // Abort the current placement / drag and return to the mode's normal state
  // (in sim this also disarms the measure tool). With nothing armed or selected,
  // Esc steps out one level: first out of an open group, then out of a component
  // definition.
  const idle =
    tool === null && selection === null && multiSel === null && pendingInsert === null &&
    draftBody.length === 0 && jointDraftIds.length === 0 && !leftDrag && !rotateDrag;
  disarmTool();
  if (idle && mode === "draw") {
    if (groupEdit !== null) leaveGroup();
    else if (editPath.length > 0) exitComponent(1);
  }
}

/** Enter: close the polyline being drawn, or commit a pattern's count. */
function finishDraft(): void {
  if (tool === "polyline") finishPolyline();
  else if (isPatternTool(tool)) finishPatternTool();
}

/** Delete: a measurement goes in either mode; anything else needs draw mode and no tool. */
function canDeleteSelection(): boolean {
  if (selection?.kind === "measure" || selection?.kind === "tempDim") return true;
  return mode === "draw" && tool === null && (selection !== null || multiSel !== null);
}

/** ↑ / ↓: the armed polygon tool's side count, else the selected regular polygon's. */
function regularSidesTarget(): { body: Body; hole: number | null } | null {
  if (tool !== null || selection?.kind !== "body") return null;
  const body = scene.getBody(selection.id);
  const target = regularEditTarget(body);
  return body && target ? { body, hole: target.hole } : null;
}
function nudgeSides(delta: number): void {
  if (tool === "polygon") {
    setPolySides(polySides + delta);
    return;
  }
  const t = regularSidesTarget();
  if (t) applyRegularSides(t.body.id, t.hole, scene.outlineRegular(t.body, t.hole)! + delta);
}
const canNudgeSides = (): boolean => tool === "polygon" || regularSidesTarget() !== null;

/** [ and ]: round or un-round the selected body a step (a disk resizes from its rim). */
function nudgeCornerRadius(step: number): void {
  const body = selection?.kind === "body" ? scene.getBody(selection.id) : undefined;
  if (!body) return;
  const disk = scene.diskOfRef({ kind: "vertex", bodyId: body.id, index: 0 });
  if (disk) {
    scene.setDiskRadius(body.id, disk.r + step); // from the *effective* radius
    demoteSizeDims(body.id, null, null); // a direct resize overrides a driving diameter
    propagateEqualRadii({ kind: "disk", bodyId: body.id });
  } else {
    scene.setBodyRadius(body.id, body.radius + step);
    // The default moved: radius dimensions on corners without their own override
    // were just overridden directly (overridden corners didn't change).
    demoteSizeDims(body.id, null, (i) => typeof body.radii?.[i] !== "number");
    // …and those corners carry the radii made Equal to them along.
    for (const corner of scene.outlineCorners(body.id, null)) {
      if (typeof body.radii?.[corner.index] !== "number") propagateEqualRadii({ kind: "vertex", bodyId: body.id, index: corner.index });
    }
  }
  if (sketchNamesRadii()) solveSketchLive(); // a tangent line follows a resized rim
  markDirty();
}
const hasSelectedBody = (): boolean =>
  mode === "draw" && tool === null && selection?.kind === "body" && !!scene.getBody(selection.id);

/** Zoom about the middle of the canvas, the way the wheel does about the cursor. */
function zoomStep(factor: number): void {
  zoomAt(view, vec(canvas.clientWidth / 2, canvas.clientHeight / 2), factor);
}

const COMMAND_ACTIONS: Record<CommandId, CommandAction> = {
  // --- File ---
  "file.open": { run: () => void openFile() },
  "file.save": { run: () => void saveToFile(false) },
  "file.saveAs": { run: () => void saveToFile(true) },
  "file.export": { run: () => setExportPanelVisible(exportPanel.classList.contains("hidden")) },
  "file.backup": { run: () => setBackupPanelVisible(backupPanel.classList.contains("hidden")) },
  "file.clear": { run: clearDocument },
  "file.shortcuts": { run: () => setKeymapPanelVisible(keymapPanel.classList.contains("hidden")) },

  // --- Edit ---
  "edit.undo": { run: undo },
  "edit.redo": { run: redo },
  "edit.copy": { run: copySelection, enabled: () => selection?.kind === "body" || multiSel !== null },
  "edit.paste": { run: () => pasteAt(cursor), enabled: () => clipboard !== null },
  "edit.group": { run: toggleGroupSelection },
  "edit.delete": { run: deleteSelection, enabled: canDeleteSelection },
  "edit.cancel": { run: cancelOrStepOut },
  "edit.finish": { run: finishDraft, enabled: () => tool === "polyline" || isPatternTool(tool) },
  "edit.cornerRadiusUp": { run: () => nudgeCornerRadius(RADIUS_STEP), enabled: hasSelectedBody },
  "edit.cornerRadiusDown": { run: () => nudgeCornerRadius(-RADIUS_STEP), enabled: hasSelectedBody },

  // --- Mode & animation ---
  "mode.toggle": { run: () => setMode(mode === "draw" ? "sim" : "draw") },
  "anim.run": { run: () => setAnimating(!animating) },
  "anim.autoPause": { run: () => setPauseOnImpossible(!pauseOnImpossible) },

  // --- Shape role ---
  "role.body": { run: () => setRole("body") },
  "role.cut": { run: () => setRole("cut") },
  "role.reference": { run: () => setRole("reference") },

  // --- Shapes (the rest arm their spec's tool) ---
  "shape.polylineBody": { run: () => { setRole("body"); setTool("polyline"); } },
  "shape.polylineCut": { run: () => { setRole("cut"); setTool("polyline"); } },
  "shape.rect": {},
  "shape.circle": {},
  "shape.polygon": {},
  "shape.slot": {},
  "shape.line": {},
  "shape.arc": {},
  "shape.text": {},
  "shape.sidesMore": { run: () => nudgeSides(1), enabled: canNudgeSides },
  "shape.sidesFewer": { run: () => nudgeSides(-1), enabled: canNudgeSides },

  // --- Patterns ---
  "tool.patternLinear": {},
  "tool.patternCircular": {},

  // --- Body operations ---
  "tool.split": {},
  "edit.combine": { run: combineSelection },
  "boolean.subtract": { run: () => booleanSelection("subtract") },
  "boolean.intersect": { run: () => booleanSelection("intersect") },

  // --- Joints & mating ---
  "tool.joint": {},
  "tool.weld": {},
  "tool.connect": {},
  "tool.ground": {},
  "tool.rail": {},
  "tool.slider": {},

  // --- Actuators ---
  "tool.linearActuator": {},
  "tool.motor": {},

  // --- Transform ---
  "edit.mirrorH": { run: () => mirrorSelection("h") },
  "edit.mirrorV": { run: () => mirrorSelection("v") },
  "edit.sendBack": { run: () => reorderSelection("back"), enabled: () => selection?.kind === "body" || multiSel !== null },
  "edit.bringFront": { run: () => reorderSelection("front"), enabled: () => selection?.kind === "body" || multiSel !== null },
  "tool.rotate": {},

  // --- Components ---
  "component.create": { run: makeComponentFromSelection },
  "component.browser": { run: () => setCompPanelVisible(!compPanelVisible) },

  // --- Constraints ---
  "sketch.autoConstraints": { run: () => setAutoConstrain(!autoConstrain) },
  "tool.coincident": {},
  "tool.equal": {},
  "tool.horizontal": {},
  "tool.vertical": {},
  "tool.parallel": {},
  "tool.perpendicular": {},
  "tool.tangent": {},
  "tool.symmetric": {},
  "tool.fixed": {},
  "sketch.badges": { run: () => setSketchVisible(!sketchVisible) },

  // --- Grid, measure, snapping ---
  "grid.show": { run: toggleGrid },
  "grid.size": { run: toggleGridSizeMenu },
  "grid.style": { run: toggleGridStyleMenu },
  "tool.measure": {},
  "measure.show": { run: () => setMeasureVisible(!measureVisible) },
  "snap.grid": { run: toggleSnap },
  "snap.object": { run: toggleObjectSnap },

  // --- View ---
  "view.fit": { run: fitView },
  "view.rotate": { run: () => setViewRotateOpen(viewRotate === null) },
  "view.rotateZero": { run: () => setViewAngle(0), enabled: () => viewRotate !== null },
  "view.zoomIn": { run: () => zoomStep(1.25) },
  "view.zoomOut": { run: () => zoomStep(1 / 1.25) },
  "view.theme": { run: toggleTheme },

  // --- Help ---
  "help.toggle": { run: () => help.toggle() },
  "help.contents": { run: () => help.open("toc") },
};

const actionOf = (c: CommandSpec): CommandAction =>
  (COMMAND_ACTIONS as Record<string, CommandAction | undefined>)[c.id] ?? {};

/** A command with neither an action nor a tool is a placeholder (tagged "planned"). */
const isRunnable = (c: CommandSpec): boolean => !!actionOf(c).run || !!c.tool;

function runCommand(c: CommandSpec): void {
  const action = actionOf(c);
  if (action.run) action.run();
  else if (c.tool) setTool(c.tool);
}

// --- the keymap: defaults, the user's overrides, dispatch --------------------
/** The shortcuts a command answers to now: the user's, else the ones it ships with. */
function bindingsOf(c: CommandSpec): Binding[] {
  return keymapOverrides[c.id] ?? defaultBindings(c);
}

/** Index every binding by slot, so a keystroke is one map lookup. */
function rebuildKeymap(): void {
  commandSlots = new Map();
  for (const c of COMMAND_LIST) {
    for (const b of bindingsOf(c)) {
      const slot = slotOf(b);
      const list = commandSlots.get(slot);
      if (list) list.push(c);
      else commandSlots.set(slot, [c]);
    }
  }
  applyShortcutTitles();
}

/**
 * The command a slot runs right now: the first one whose context matches the mode
 * (`any` matches both) and whose guard is happy. `typing` narrows the field to the
 * commands that answer from inside an input.
 */
function commandForSlot(slot: string, typing: boolean): CommandSpec | null {
  for (const c of commandSlots.get(slot) ?? []) {
    if (typing && !c.whileTyping) continue;
    if (c.context !== "any" && c.context !== mode) continue;
    if (!isRunnable(c)) continue;
    const enabled = actionOf(c).enabled;
    if (enabled && !enabled()) continue;
    return c;
  }
  return null;
}

// Ctrl / Cmd pressed or released with the mouse still: the pattern tool's overlay switches
// between picking seeds and laying them out on it, so the modifier state can't wait for
// the next mouse event.
window.addEventListener("keydown", (e) => {
  if (e.key === "Control" || e.key === "Meta") mods.ctrl = true;
});
window.addEventListener("keyup", (e) => {
  if (e.key === "Control" || e.key === "Meta") mods.ctrl = e.ctrlKey || e.metaKey;
});
window.addEventListener("blur", () => {
  mods.ctrl = false;
});

window.addEventListener("keydown", (e) => {
  const slot = slotOfEvent(e);
  if (!slot) return; // a bare modifier, or a key nothing can be bound to
  // Keys typed into a toolbar field (or the inline dimension / label editor) belong to
  // that field. Only the four commands marked `whileTyping` — save, save as, open and
  // the manual — answer from there, and they preventDefault so the browser doesn't
  // offer to save the web page itself.
  const t = e.target;
  const typing =
    t instanceof HTMLInputElement || t instanceof HTMLSelectElement || t instanceof HTMLTextAreaElement;
  const cmd = commandForSlot(slot, typing);
  if (!cmd) return;
  e.preventDefault();
  runCommand(cmd);
});

// --- tooltips generated from the keymap -------------------------------------
/**
 * Every control that stands for a command carries `data-cmd` (tool buttons are matched
 * by their `data-tool` instead), and its `title` in `index.html` holds only the base
 * text: the shortcut is appended here, from the keymap in force. That is the whole
 * reason a remap is one edit — no tooltip can go stale because none of them spells a
 * key. A control that stands for several commands names each one, which is how
 * "(B = body, U = cut)" writes itself.
 */
function shortcutSuffix(cmds: CommandSpec[]): string {
  const parts: string[] = [];
  for (const c of cmds) {
    const chords = bindingsOf(c).map(formatChord);
    if (!chords.length) continue;
    parts.push(cmds.length > 1 ? `${chords.join(" / ")} = ${commandQualifier(c)}` : chords.join(" / "));
  }
  return parts.length ? ` (${parts.join(", ")})` : "";
}

/** How a command is named when a control carries several: "Polyline (body)" → "body". */
function commandQualifier(c: CommandSpec): string {
  return /\(([^)]+)\)\s*$/.exec(c.label)?.[1] ?? c.label;
}

/** The commands a control stands for: its `data-cmd` list, else whatever arms its tool. */
function commandsOfElement(el: HTMLElement): CommandSpec[] {
  const ids = el.dataset.cmd?.trim().split(/\s+/).filter(Boolean) ?? [];
  if (ids.length) return ids.map(commandById).filter((c): c is CommandSpec => !!c);
  const t = el.dataset.tool;
  return t ? COMMAND_LIST.filter((c) => c.tool === t) : [];
}

/** Retitle one control. Callers that rewrite a title (the save button names the bound
 *  file, the mode toggle names the other mode) pass the new base text through here. */
function applyShortcutTitle(el: HTMLElement, base: string): void {
  el.dataset.titleBase = base;
  el.title = base + shortcutSuffix(commandsOfElement(el));
}

function applyShortcutTitles(): void {
  for (const el of document.querySelectorAll<HTMLElement>("[data-cmd], [data-tool]")) {
    if (!commandsOfElement(el).length) continue;
    applyShortcutTitle(el, el.dataset.titleBase ?? el.title);
  }
}

// --- the Shortcuts panel: import / export / reset ----------------------------
/**
 * A user's keymap is the whole `{commandId: bindings[]}` map in localStorage, not a diff
 * against the shipped defaults — a diff against a moving default is a migration problem
 * nobody wants. A command the map doesn't mention (one a later version adds) keeps its
 * default. Export hands KeyMapper the effective keymap as a `keymap/1` file; import
 * takes that file back.
 *
 * Precedence, all of it: **this map → `public/keymap.json` → no shortcut.** A command
 * neither of them binds simply has no key, which is a legitimate state — roughly a
 * quarter of the registry ships that way — so there is no third fallback under this.
 */
const keymapBtn = document.getElementById("keymap-btn") as HTMLButtonElement;
const keymapPanel = document.getElementById("keymap-panel")!;
const keymapStatus = document.getElementById("keymap-status")!;
const keymapImport = document.getElementById("keymap-import") as HTMLButtonElement;
const keymapExport = document.getElementById("keymap-export") as HTMLButtonElement;
const keymapReset = document.getElementById("keymap-reset") as HTMLButtonElement;
const keymapClose = document.getElementById("keymap-close") as HTMLButtonElement;
const keymapInput = document.getElementById("keymap-input") as HTMLInputElement;

function setKeymapPanelVisible(on: boolean): void {
  keymapPanel.classList.toggle("hidden", !on);
  keymapBtn.classList.toggle("active", on);
  if (on) refreshKeymapPanel();
}

function refreshKeymapPanel(): void {
  const custom = Object.keys(keymapOverrides).length > 0;
  keymapStatus.textContent = custom
    ? "Your own keymap (kept in this browser)"
    : keymapFileError
      ? "The shipped keymap file is unreadable — no shortcuts. Import one."
      : "The shortcuts Disjointed ships with";
  keymapReset.disabled = !custom;
}

function saveKeymapOverrides(): void {
  try {
    if (Object.keys(keymapOverrides).length === 0) localStorage.removeItem(KEYMAP_KEY);
    else localStorage.setItem(KEYMAP_KEY, JSON.stringify(keymapOverrides));
  } catch {
    /* a browser with storage turned off still gets the keymap, just not across reloads */
  }
  rebuildKeymap();
  refreshKeymapPanel();
}

function exportKeymap(): void {
  downloadText(JSON.stringify(toKeymapFile(bindingsOf), null, 2), "disjointed-keymap.json", "application/json");
}

async function importKeymap(file: File): Promise<void> {
  let data: unknown;
  try {
    data = JSON.parse(await file.text());
  } catch {
    notify("That file isn't JSON.", "error");
    return;
  }
  const { file: keymap, errors } = parseKeymap(data);
  if (!keymap) {
    notify(`Not a keymap this version can read: ${errors[0]?.message ?? "unknown format"}`, "error");
    return;
  }
  const known = new Set(COMMAND_IDS);
  keymapOverrides = overridesOfFile(keymap, known);
  saveKeymapOverrides();
  // Clashes are the editor's business, not a reason to refuse the file — but say so,
  // because on a clashing slot only the first command in registry order answers.
  const clashes = findConflicts(COMMAND_LIST.map((c) => ({ ...c, bindings: bindingsOf(c) })));
  const missing = keymap.commands.filter((c) => !known.has(c.id)).length;
  const notes = [
    `${Object.keys(keymapOverrides).length} commands`,
    ...(missing ? [`${missing} the app doesn't know, ignored`] : []),
    ...(clashes.length ? [`${clashes.length} clashing ${clashes.length === 1 ? "key" : "keys"}`] : []),
  ];
  notify(`Keymap loaded — ${notes.join("; ")}.`, clashes.length ? "warn" : "info");
}

function resetKeymap(): void {
  keymapOverrides = {};
  saveKeymapOverrides();
  notify("Shortcuts back to the shipped defaults.", "info");
}

keymapBtn.addEventListener("click", () => setKeymapPanelVisible(keymapPanel.classList.contains("hidden")));
keymapClose.addEventListener("click", () => setKeymapPanelVisible(false));
keymapExport.addEventListener("click", exportKeymap);
keymapReset.addEventListener("click", resetKeymap);
keymapImport.addEventListener("click", () => keymapInput.click());
keymapInput.addEventListener("change", () => {
  const file = keymapInput.files?.[0];
  keymapInput.value = ""; // allow re-loading the same file later
  if (file) void importKeymap(file);
});

rebuildKeymap();
// The shipped shortcuts are a file now, so a bad edit to it costs every key at once
// rather than a build error. Say so where it can be seen without one: the Shortcuts
// panel, which explains it, may not be reachable from the keyboard at all.
if (keymapFileError) {
  notify(
    `The shipped shortcut file could not be read, so no command has a key — ${keymapFileError} ` +
      "Import a keymap from the Shortcuts panel (File group) to get shortcuts back.",
    "error"
  );
}

// --- actuators / motors --------------------------------------------------
/** All linear-actuator constraints in the scene. */
function linearActuators(): LinearActuatorConstraint[] {
  return scene.constraints.filter(
    (c): c is LinearActuatorConstraint => c.kind === "linearActuator"
  );
}
function motors(): MotorConstraint[] {
  return scene.constraints.filter((c): c is MotorConstraint => c.kind === "motor");
}

/** The actuator that owns the currently selected joint (its rider), or null. */
function selectedLinearActuator(): LinearActuatorConstraint | null {
  if (!selection) return null;
  if (selection.kind === "joint") {
    return linearActuators().find((a) => a.riderId === selection!.id) ?? null;
  }
  if (selection.kind === "rail") {
    return linearActuators().find((a) => a.sliderId === selection!.id) ?? null;
  }
  return null;
}

/** The motor identified by the current selection (body / pivot / crank joint), or null. */
function selectedMotor(): MotorConstraint | null {
  if (!selection) return null;
  const ms = motors();
  if (selection.kind === "body") return ms.find((m) => m.bodyId === selection!.id) ?? null;
  if (selection.kind === "joint") {
    return ms.find(
      (m) => m.pivotJointId === selection!.id || m.crankJointId === selection!.id
    ) ?? null;
  }
  return null;
}

/**
 * Triangle wave that traces 0 → 1 → 0 over one cycle (`p` in cycles). The natural "linear
 * actuator" motion: constant speed end-to-end, instant reverse at each endstop.
 */
function triangleWave(p: number): number {
  const f = ((p % 1) + 1) % 1; // wrap into [0,1)
  return f < 0.5 ? 2 * f : 2 * (1 - f);
}
/** Phase at which the triangle wave equals `s` ∈ [0,1], ascending branch (so it moves toward 1 next). */
function triangleInverse(s: number): number {
  return Math.max(0, Math.min(0.5, s / 2));
}
/** Sine-shaped wave 0 → 1 → 0 over one cycle: smooth ease in/out at the endstops. */
function sineWave(p: number): number {
  return 0.5 * (1 - Math.cos(2 * Math.PI * p));
}
function sineInverse(s: number): number {
  return Math.acos(Math.max(-1, Math.min(1, 1 - 2 * s))) / (2 * Math.PI);
}

/**
 * Fit each actuator/motor's phase accumulator so the next-frame target matches its current
 * world state — called when toggling animation **on**, so play picks up smoothly from
 * whatever pose the user left in sim (incl. after a drag while paused). Linear: phase in
 * cycles, fit to current rider position fraction along the rail. Motor: phase in radians,
 * fit to the current crank-relative-to-pivot angle.
 */
function fitPhases(): void {
  animPhase.clear();
  for (const a of linearActuators()) {
    const slider = scene.constraints.find((c) => c.id === a.sliderId && c.kind === "slider");
    const rider = scene.getJoint(a.riderId);
    if (!slider || slider.kind !== "slider" || !rider) continue;
    const ja = scene.getJoint(slider.railA);
    const jb = scene.getJoint(slider.railB);
    if (!ja || !jb) continue;
    const pa = scene.jointWorld(ja);
    const pb = scene.jointWorld(jb);
    const dx = pb.x - pa.x;
    const dy = pb.y - pa.y;
    const dl = Math.hypot(dx, dy);
    if (dl < 1e-9) continue;
    const q = scene.jointWorld(rider);
    const s = Math.max(0, Math.min(1, ((q.x - pa.x) * dx + (q.y - pa.y) * dy) / (dl * dl)));
    animPhase.set(a.id, a.profile === "sine" ? sineInverse(s) : triangleInverse(s));
  }
  for (const m of motors()) {
    const jp = scene.getJoint(m.pivotJointId);
    const jc = scene.getJoint(m.crankJointId);
    if (!jp || !jc) continue;
    const pp = scene.jointWorld(jp);
    const pc = scene.jointWorld(jc);
    animPhase.set(m.id, Math.atan2(pc.y - pp.y, pc.x - pp.x)); // radians
  }
}

/**
 * Build this frame's anchors map (joint id → world target) for the solver. One target per
 * linear-actuator rider (computed from its phase + the current rail), and two targets per
 * motor (pivot fixed + crank on its orbit). Anchors are only emitted while animation is
 * running; otherwise the scene runs purely under mouse-drag drivers.
 */
function computeAnchors(): Map<number, Vec2> {
  const anchors = new Map<number, Vec2>();
  if (!animating) return anchors;
  for (const a of linearActuators()) {
    const slider = scene.constraints.find((c) => c.id === a.sliderId && c.kind === "slider");
    if (!slider || slider.kind !== "slider") continue;
    const ja = scene.getJoint(slider.railA);
    const jb = scene.getJoint(slider.railB);
    if (!ja || !jb) continue;
    const pa = scene.jointWorld(ja);
    const pb = scene.jointWorld(jb);
    const dl = Math.hypot(pb.x - pa.x, pb.y - pa.y);
    if (dl < 1e-9) continue;
    const phase = animPhase.get(a.id) ?? 0;
    const s = a.profile === "sine" ? sineWave(phase) : triangleWave(phase);
    anchors.set(a.riderId, vec(pa.x + (pb.x - pa.x) * s, pa.y + (pb.y - pa.y) * s));
  }
  for (const m of motors()) {
    const jp = scene.getJoint(m.pivotJointId);
    const jc = scene.getJoint(m.crankJointId);
    if (!jp || !jc) continue;
    const pp = scene.jointWorld(jp);
    const pc = scene.jointWorld(jc);
    const r = dist(pp, pc); // current crank radius (frozen by the pivot anchor below)
    if (r < 1e-6) continue;
    const theta = animPhase.get(m.id) ?? 0;
    // Pivot stays exactly where it is now; crank orbits at the current radius.
    anchors.set(m.pivotJointId, vec(pp.x, pp.y));
    anchors.set(m.crankJointId, vec(pp.x + r * Math.cos(theta), pp.y + r * Math.sin(theta)));
  }
  return anchors;
}

/** Advance every actuator/motor phase by its speed * dt. */
function advancePhases(dt: number): void {
  for (const a of linearActuators()) {
    animPhase.set(a.id, (animPhase.get(a.id) ?? 0) + a.speed * dt);
  }
  for (const m of motors()) {
    animPhase.set(m.id, (animPhase.get(m.id) ?? 0) + 2 * Math.PI * m.speed * dt);
  }
}

/** Toggle animation on / off. On start, fit phases so the wave resumes from the current state. */
function setAnimating(on: boolean): void {
  if (on === animating) return;
  animating = on;
  animLastTimestamp = null;
  impossibleFrames = 0; // any stretch of impossible frames is per-run
  if (on) {
    fitPhases();
  } else {
    // Stop: clear the rolling solve-time stats so the next run starts fresh.
    animSolveMin = Infinity;
    animSolveMax = 0;
    animSolveSum = 0;
    animSolveCount = 0;
    animErrorFrames = 0;
    animCleanupSum = 0;
    animCleanupMax = 0;
    animPhaseASum = 0;
    animPhaseAMax = 0;
    animResidualSum = 0;
    animResidualMax = 0;
  }
  runBtn.classList.toggle("running", animating);
}

/** Toggle the auto-pause-on-impossible safety. Affects only the animation loop. */
function setPauseOnImpossible(on: boolean): void {
  pauseOnImpossible = on;
  autopauseBtn.classList.toggle("armed", pauseOnImpossible);
  autopauseBtn.setAttribute("aria-pressed", pauseOnImpossible ? "true" : "false");
}

/**
 * Sync the inline actuator / motor properties panels to the current selection. Hidden when
 * nothing relevant is selected. Like `syncColorPicker`, change-detected so we don't clobber
 * the input value mid-edit.
 */
let propsSyncKey = "";
function syncPropsPanel(): void {
  const a = selectedLinearActuator();
  const m = a ? null : selectedMotor(); // actuator panel wins when both could apply (the joint case)
  const key = a
    ? `a${a.id}:${a.speed}:${a.profile}`
    : m
    ? `m${m.id}:${m.speed}`
    : "";
  if (key === propsSyncKey) return;
  propsSyncKey = key;
  actuatorProps.classList.toggle("hidden", !a);
  motorProps.classList.toggle("hidden", !m);
  if (a) {
    actuatorSpeedInput.value = String(a.speed);
    profileToggle.querySelectorAll<HTMLButtonElement>("button").forEach((b) =>
      b.classList.toggle("active", b.dataset.profile === a.profile)
    );
  }
  if (m) {
    motorSpeedInput.value = String(m.speed);
  }
}

// --- main loop -----------------------------------------------------------
/** Run a solve and log how long the calculation took (debug). For the per-frame animation
 * solve, also tracks min/max/avg across the run so the line shows the rolling distribution. */
function timedSolve(
  label: string,
  drv: Driver | null,
  iterations = 100,
  anchors?: Map<number, Vec2>,
  freeze?: SolveFreeze
): void {
  const t0 = performance.now();
  const stats: SolveStats = { phaseASweeps: 0, cleanupSweeps: 0, finalResidual: 0 };
  solveBreaks = solve(scene, drv, iterations, 1, anchors, stats, freeze);
  const dt = performance.now() - t0;
  if (label === "anim") {
    animSolveMin = Math.min(animSolveMin, dt);
    animSolveMax = Math.max(animSolveMax, dt);
    animSolveSum += dt;
    animSolveCount++;
    if (solveBreaks.length > 0) animErrorFrames++;
    animPhaseASum += stats.phaseASweeps;
    animPhaseAMax = Math.max(animPhaseAMax, stats.phaseASweeps);
    animCleanupSum += stats.cleanupSweeps;
    animCleanupMax = Math.max(animCleanupMax, stats.cleanupSweeps);
    animResidualSum += stats.finalResidual;
    animResidualMax = Math.max(animResidualMax, stats.finalResidual);
    const avg = animSolveSum / animSolveCount;
    const phaseAAvg = animPhaseASum / animSolveCount;
    const cleanupAvg = animCleanupSum / animSolveCount;
    const residualAvg = animResidualSum / animSolveCount;
    const errPct = (animErrorFrames / animSolveCount) * 100;
    console.log(
      `[Disjointed] ${label} solve: ${dt.toFixed(3)} ms ` +
        `(min ${animSolveMin.toFixed(3)} / max ${animSolveMax.toFixed(3)} / avg ${avg.toFixed(3)}) ` +
        `phaseA: ${stats.phaseASweeps} (avg ${phaseAAvg.toFixed(1)} / max ${animPhaseAMax}) ` +
        `cleanup: ${stats.cleanupSweeps} (avg ${cleanupAvg.toFixed(1)} / max ${animCleanupMax}) ` +
        `residual: ${stats.finalResidual.toExponential(2)} (avg ${residualAvg.toExponential(2)} / max ${animResidualMax.toExponential(2)}) ` +
        `errors: ${animErrorFrames} / ${animSolveCount} (${errPct.toFixed(3)}%)`
    );
  } else {
    console.log(`[Disjointed] ${label} solve: ${dt.toFixed(3)} ms`);
  }
  updateSimError();
}

/** Show/hide the red "assembly impossible" banner from the last solve's unsatisfied constraints. */
function updateSimError(): void {
  const show = mode === "sim" && solveBreaks.length > 0;
  simErrorEl.classList.toggle("hidden", !show);
  if (show) {
    simErrorEl.textContent =
      solveBreaks.length === 1
        ? "Assembly impossible — a constraint can't be satisfied"
        : `Assembly impossible — ${solveBreaks.length} constraints can't be satisfied`;
  }
}

/** Joints currently highlighted as in-progress tool picks. */
function activeJoints(): number[] {
  if (mode !== "draw") return [];
  if (tool === "connect") return selectedJoint !== null ? [selectedJoint] : [];
  if (tool === "rail") return railDraftIds;
  if (tool === "slider") return sliderDraft?.riderId !== null && sliderDraft?.riderId !== undefined ? [sliderDraft.riderId] : [];
  if (tool === "polyline") return jointDraftIds;
  if (tool === "motor") return motorPivotDraft !== null ? [motorPivotDraft] : [];
  return [];
}

/** Rail-joint positions picked so far for the rail tool, with the live cursor — or the
 *  slider tool's start point with where its second click would land. */
function railDraftView(): { rail: Vec2[]; cursor: Vec2 } | null {
  if (mode !== "draw" || !cursor) return null;
  if (tool === "slider" && sliderDraft) return { rail: [sliderDraft.at], cursor: sliderEndAt(cursor) };
  if (tool !== "rail" || railDraftIds.length === 0) return null;
  return { rail: railDraftIds.map((id) => scene.jointWorld(scene.getJoint(id)!)), cursor };
}

/** Where the slider tool's second click at `p` lands: object snap (corners / joints /
 *  centres) first, then the grid — the same landing as a Joint-tool placement. */
function sliderEndAt(p: Vec2): Vec2 {
  return placeSnap(p);
}

/** The point a whole-guide drag is anchored on (its first handle). */
function guideAnchorWorld(g: Guide): Vec2 {
  return scene.guidePointWorld(g, scene.guideHandleKeys(g)[0]) ?? vec(0, 0);
}

/** Control-vertex handles to show for the body selected in select / rotate mode (else
 *  null) — the outer outline's plus every hole's. */
function editVerticesView(): Vec2[] | null {
  if (mode !== "draw" || (tool !== null && tool !== "rotate") || selection?.kind !== "body")
    return null;
  const body = scene.getBody(selection.id);
  if (!body) return null;
  const out = [...scene.bodyControlWorld(body)];
  body.holes?.forEach((_, hi) => out.push(...scene.bodyHoleControlWorld(body, hi)));
  return out;
}

// --- regular polygons: centres and side-count tags ---------------------------------
/** Centres of every regular-polygon outline (draw mode): crosshair markers, pickable as point refs. */
function regularCentresView(): { at: Vec2; bodyId: number }[] {
  if (mode !== "draw") return [];
  const out: { at: Vec2; bodyId: number }[] = [];
  for (const body of scene.bodies) {
    if (body.regular) {
      const c = scene.regularCentreWorld(body.id);
      if (c) out.push({ at: c, bodyId: body.id });
    }
    body.holes?.forEach((h, hi) => {
      if (!h.regular) return;
      const c = scene.regularCentreWorld(body.id, hi);
      if (c) out.push({ at: c, bodyId: body.id });
    });
  }
  return out;
}

/** Side-count tags of the selected body's regular outlines (last frame's layout, for picking). */
let regularTagCache: { bodyId: number; hole: number | null; at: Vec2 }[] = [];

/** One "n sides" tag per regular outline of the selected body, just above the outline. */
function regularTagsView(): { at: Vec2; text: string }[] {
  regularTagCache = [];
  if (mode !== "draw" || (tool !== null && tool !== "rotate") || selection?.kind !== "body") return [];
  const body = scene.getBody(selection.id);
  if (!body) return [];
  const out: { at: Vec2; text: string }[] = [];
  const place = (pts: Vec2[], hole: number | null, n: number): void => {
    let top = Infinity;
    for (const p of pts) top = Math.min(top, worldToScreen(view, p).y);
    const c = scale(pts.reduce((acc, q) => add(acc, q), vec(0, 0)), 1 / pts.length);
    const at = screenToWorld(view, vec(worldToScreen(view, c).x, top - 16));
    regularTagCache.push({ bodyId: body.id, hole, at });
    out.push({ at, text: `${n} sides` });
  };
  if (body.regular) place(scene.bodyControlWorld(body), null, body.regular);
  body.holes?.forEach((h, hi) => {
    if (h.regular) place(scene.bodyHoleControlWorld(body, hi), hi, h.regular);
  });
  return out;
}

/** The side-count tag under `p` (last frame's layout), or null. */
function regularTagAt(p: Vec2): { bodyId: number; hole: number | null } | null {
  const r = LABEL_PICK_RADIUS / view.scale;
  for (let i = regularTagCache.length - 1; i >= 0; i--) {
    if (dist(regularTagCache[i].at, p) <= r) return regularTagCache[i];
  }
  return null;
}

/** Which regular outline of `body` the keyboard edits: its outer outline, else the one
 *  regular hole the feature selection is on. */
function regularEditTarget(body: Body | undefined): { hole: number | null } | null {
  if (!body) return null;
  if (body.regular) return { hole: null };
  if (featureSel?.bodyId === body.id && featureSel.verts.length) {
    const h = featureSel.verts[0].hole;
    if (h !== null && featureSel.verts.every((v) => v.hole === h) && scene.outlineRegular(body, h) !== null) return { hole: h };
  }
  return null;
}

/** The side-count editor (shares the dimension editor's input). */
let sidesEdit: { bodyId: number; hole: number | null } | null = null;

/** Open the floating input over a regular outline's side-count tag (double-click). */
function openSidesEditor(tag: { bodyId: number; hole: number | null }): void {
  const body = scene.getBody(tag.bodyId);
  const n = body ? scene.outlineRegular(body, tag.hole) : null;
  const at = regularTagCache.find((t) => t.bodyId === tag.bodyId && t.hole === tag.hole)?.at;
  if (n === null || !at) return;
  closeDimEditor();
  sidesEdit = { bodyId: tag.bodyId, hole: tag.hole };
  const sp = worldToScreen(view, at);
  dimEditInput.style.left = `${sp.x}px`;
  dimEditInput.style.top = `${sp.y}px`;
  dimEditInput.value = String(n);
  dimEditInput.classList.remove("hidden");
  dimEditInput.focus();
  dimEditInput.select();
}

/** Commit the side-count editor: a whole number of sides (3 … REGULAR_MAX_SIDES). */
function commitSidesEditor(): void {
  const edit = sidesEdit;
  const raw = dimEditInput.value.trim();
  closeDimEditor(); // clears sidesEdit first, so the blur listener doesn't re-commit
  if (!edit || raw === "") return;
  const n = Number(raw);
  if (!Number.isInteger(n) || n < 3 || n > Scene.REGULAR_MAX_SIDES) {
    notify(`The side count must be a whole number from 3 to ${Scene.REGULAR_MAX_SIDES}.`);
    return;
  }
  applyRegularSides(edit.bodyId, edit.hole, n);
}

/**
 * Change a regular outline's side count (a pattern member's edit goes to its seed) and
 * re-solve the sketch; a result the constraints can't take is reverted and reported.
 */
function applyRegularSides(bodyId: number, hole: number | null, n: number): void {
  const seedHole = hole === null ? null : scene.patternSeedHole(bodyId, hole);
  const before = JSON.stringify(scene.serialize());
  if (!scene.setRegularSides(bodyId, seedHole, n)) return;
  const breaks = solveSketch(scene);
  if (breaks.length) {
    scene.load(JSON.parse(before));
    flashSketchItems(breaks);
    const what = describeBreaks(breaks);
    notify(
      what
        ? `That side count conflicts with ${what} (flashing red).`
        : "That side count breaks a constraint or dimension on the polygon.",
      "error"
    );
    return;
  }
  markDirty();
}

/** Resolved measurements of the current mode (a ref that can't resolve just isn't drawn). */
function measurementsView(): MeasureInfo[] {
  if (!measureVisible) return [];
  const mm = mode === "sim" ? "sim" : "draw";
  const out: MeasureInfo[] = [];
  for (const m of scene.measurements) {
    if (m.mode !== mm) continue;
    const info = scene.measureInfo(m);
    if (info) out.push(info);
  }
  return out;
}

/** Measure-tool overlay: picked refs, the ref under the cursor, and the placement preview. */
function measureDraftView(): {
  refs: MeasureHighlight[];
  hover: MeasureHighlight | null;
  preview: MeasureInfo | null;
} | null {
  if (tool !== "measure") return null;
  // A diameter pick (the same disk vertex twice) highlights the disk's rim, not its
  // centre; a radius pick (the same corner vertex twice) highlights the corner's arc.
  const same = measurePicks.length === 2 && sameTempRef(measurePicks[0], measurePicks[1]);
  const first = measurePicks[0];
  const plainFirst = first && first.kind !== "ghost" ? first : null;
  const disk = same && plainFirst ? scene.diskOfRef(plainFirst) : null;
  const cornerArc = same && plainFirst && !disk ? (scene.cornerOfRef(plainFirst)?.arc ?? null) : null;
  const refs: MeasureHighlight[] = disk
    ? [{ kind: "circle", c: disk.c, r: disk.r }]
    : cornerArc
      ? [{ kind: "arc", ...cornerArc }]
      : measurePicks
          .map((r) => resolveTemp(r))
          .filter((r): r is ResolvedMeasureRef => r !== null);
  let hover: MeasureHighlight | null = null;
  let preview: MeasureInfo | null = null;
  if (cursor) {
    if (measurePicks.length < 2) {
      const rim = measurePicks.length === 0 ? diskRimAt(cursor) : null;
      const arc = measurePicks.length === 0 && !rim ? cornerArcAt(cursor) : null;
      const ends = pickedLineEnds();
      const h: TempRef | null =
        rim || arc ? null : ends ? measureSecondRefAt(cursor) : measureRefAt(cursor) ?? ghostRefAt(cursor);
      hover = rim
        ? { kind: "circle", c: rim.c, r: rim.r }
        : arc
          ? { kind: "arc", ...arc.arc }
          : h
            ? resolveTemp(h)
            : null;
      // One line picked and nothing under the cursor: the line's own length, previewed
      // the way a click would place it.
      if (ends && !h) preview = scene.measurePreview(ends[0], ends[1], cursor);
    } else if (measurePicks[0].kind === "ghost" || measurePicks[1].kind === "ghost") {
      preview = tempDimPreview(measurePicks[0], measurePicks[1], cursor);
    } else {
      preview = scene.measurePreview(measurePicks[0], measurePicks[1], cursor);
    }
  }
  return { refs, hover, preview };
}

/** Stable stacking key for a constraint reference (badges on one element stack sideways). */
function sketchRefKey(ref: MeasureRef): string {
  switch (ref.kind) {
    case "joint": return `j:${ref.jointId}`;
    case "vertex": return `v:${ref.bodyId}:${ref.index}${ref.hole !== undefined ? `:${ref.hole}` : ""}`;
    case "centre": return `c:${ref.bodyId}${ref.hole !== undefined ? `:${ref.hole}` : ""}`;
    case "edge": return `e:${ref.bodyId}:${ref.index}${ref.hole !== undefined ? `:${ref.hole}` : ""}`;
    case "rail": return `r:${ref.sliderId}`;
    case "guidePoint": return `gp:${ref.guideId}:${ref.which}`;
    case "guideLine": return `gl:${ref.guideId}:${ref.edge}`;
    case "patternAxis": return `px:${ref.patternId}:${ref.axis}`;
    case "midpoint": return `m:${sketchRefKey(ref.of)}`;
    case "disk": return `d:${ref.bodyId}${ref.hole !== undefined ? `:${ref.hole}` : ""}`;
    case "guideCircle": return `gc:${ref.guideId}`;
    default: return "?";
  }
}

/**
 * How a constraint reference is highlighted: a circle reference as its rim / arc (the
 * measure tool's diameter-pick picture), anything else as its resolved point or line.
 */
function highlightOfRef(ref: MeasureRef): MeasureHighlight | null {
  return isCircleRef(ref) ? scene.circleHighlightOfRef(ref) : scene.resolveMeasureRef(ref);
}

/**
 * How a reference picked *for its radius* is highlighted (the Equal tool, an Equal
 * between radii): a rounded corner as its arc — the measure tool's radius-pick picture
 * — and anything else as `highlightOfRef` does (a circle as its rim).
 */
function radiusHighlightOfRef(ref: MeasureRef): MeasureHighlight | null {
  if (ref.kind === "vertex") {
    const arc = scene.cornerOfRef(ref)?.arc;
    if (arc) return { kind: "arc", ...arc };
  }
  return highlightOfRef(ref);
}

/**
 * Whether the cursor is over the element a constraint reference names — a joint or a
 * body corner within pick range; for a vertex/edge, anywhere on the owning body counts
 * too (the whole body is the "parent" whose hover reveals its constraints); a rail's
 * segment within pick range.
 */
function refHovered(ref: MeasureRef, p: Vec2): boolean {
  const r = pickRadius();
  const res = scene.resolveMeasureRef(ref);
  if (!res) return false;
  // A circle: its rim (a circle resolves to its centre, which is not where you point).
  const circ = isCircleRef(ref) ? scene.circleOfRef(ref) : null;
  if (circ && Math.abs(dist(p, circ.c) - circ.r) <= r) return true;
  if (res.kind === "point" && dist(res.p, p) <= r) return true;
  if (res.kind === "line" && distToSegment(p, res.a, res.b) <= r) return true;
  if (ref.kind === "midpoint") return refHovered(ref.of, p); // its line's hover reveals it
  if (ref.kind === "vertex" || ref.kind === "edge" || ref.kind === "disk") return scene.bodyAt(p)?.id === ref.bodyId;
  // Hovering anywhere on a reference element (or one of its points) reveals its constraints.
  if (ref.kind === "guideLine" || ref.kind === "guidePoint" || ref.kind === "guideCircle") {
    const g = scene.getGuide(ref.guideId);
    if (!g) return false;
    return scene.guideAt(p, r)?.id === g.id || scene.guidePointAt(p, r)?.guide.id === g.id;
  }
  return false;
}

/**
 * On-canvas badges for every sketch constraint (draw mode only): one badge per referenced
 * element, offset from it in screen terms so it stays put at any zoom — beside a point,
 * off the midpoint of a line. Multiple badges on one element stack sideways. Coincident
 * gets a single badge on refA — always the point: two coincident points share a
 * position, and a point-on-line coincident normalizes the point into refA. Badges
 * render faded unless the
 * cursor is over one of the constraint's elements (or a badge itself). The result is
 * cached for click hit-testing (`sketchGlyphAt`).
 */
function sketchGlyphsView(): SketchGlyphView[] {
  if (mode !== "draw" || !sketchVisible) {
    sketchGlyphCache = []; // hidden badges aren't hit-testable either
    return [];
  }
  const px = (n: number) => n / view.scale;
  const stack = new Map<string, number>();
  const out: SketchGlyphView[] = [];
  let hoveredBadge = -1; // index in `out` of the topmost constraint whose badge is under the cursor
  for (const c of scene.sketch) {
    const allRefs = sketchRefs(c); // a symmetry badges its mirror line too
    // A tangent gets one badge at the contact point, just off the line on the side away
    // from the circle — the one place that names both elements at once.
    const badgeRefs = c.kind === "coincident" || c.kind === "tangent" ? [] : allRefs;
    const badges: Vec2[] = [];
    if (c.kind === "coincident") badgeRefs.push(c.refA);
    if (c.kind === "tangent" && c.refB) {
      const circ = scene.circleOfRef(c.refA);
      const ln = scene.resolveMeasureRef(c.refB);
      if (circ && ln?.kind === "line" && dist(ln.a, ln.b) > 1e-9) {
        const d = normalize(sub(ln.b, ln.a));
        const n = vec(-d.y, d.x);
        const s = dot(sub(circ.c, ln.a), n); // the centre's signed offset off the line
        const foot = sub(circ.c, scale(n, s)); // the contact point (the centre's foot)
        badges.push(add(foot, scale(n, s >= 0 ? -px(14) : px(14))));
      }
    }
    for (const ref of badgeRefs) {
      const r = scene.resolveMeasureRef(ref);
      if (!r) continue;
      const key = sketchRefKey(ref);
      const i = stack.get(key) ?? 0;
      stack.set(key, i + 1);
      if (r.kind === "point") {
        badges.push(add(r.p, vec(px(14 + i * 20), -px(14))));
      } else {
        const mid = scale(add(r.a, r.b), 0.5);
        const d = normalize(sub(r.b, r.a));
        let n = vec(-d.y, d.x);
        // A pattern axis carries its spacing dimension on one side: badge on the other.
        if (ref.kind === "patternAxis") {
          const ax = patternViewCache.find((v) => v.id === ref.patternId)?.axes[ref.axis];
          if (ax) {
            const side = sub(ax.dim.a, ax.line.a);
            if (dot(side, n) > 0) n = scale(n, -1);
          }
        }
        badges.push(add(add(mid, scale(n, px(14))), scale(d, px(i * 20))));
      }
    }
    if (!badges.length) continue;
    // A constraint outside an open group is surroundings: faded and unpickable, so it
    // never lights up under the cursor either.
    const badgeHot =
      sketchInScope(c) && cursor !== null && badges.some((b) => dist(b, cursor!) <= GLYPH_PICK_RADIUS / view.scale);
    const hot = badgeHot || (cursor !== null && allRefs.some((ref) => refHovered(ref, cursor!)));
    // A pose constraint that can't currently hold (grounded partner, a def-edit reset,
    // an instance rotated against it) shows in the error style, like a violated dim —
    // so does an Equal between radii whose two radii differ (a member nothing can set).
    const violated = poseConstraintViolated(scene, c) || equalRadiusViolated(scene, c);
    out.push({ id: c.id, kind: c.kind, badges, faded: !hot && !violated, violated });
    if (badgeHot) hoveredBadge = out.length - 1;
  }
  // Hovering a badge reveals what it constrains: the elements light up and, when they
  // sit apart, a dotted line joins them. Only the topmost badge under the cursor (the
  // one a click would select) gets it, so stacked badges don't all fire at once.
  if (hoveredBadge >= 0) {
    const c = scene.sketch.find((k) => k.id === out[hoveredBadge].id)!;
    const refs = sketchRefs(c)
      .map(isEqualRadiusConstraint(c) ? radiusHighlightOfRef : highlightOfRef)
      .filter((r): r is MeasureHighlight => r !== null);
    // The link joins the two related elements (a symmetry's pair — its mirror is the
    // third; a tangent's circle and line touch by definition, so none).
    const [a, b] = refs;
    const linkable = (r: MeasureHighlight | undefined): r is ResolvedMeasureRef => !!r && (r.kind === "point" || r.kind === "line");
    const link = linkable(a) && linkable(b) ? sketchLink(a, b) : null;
    out[hoveredBadge].hover = { refs, link };
  }
  sketchGlyphCache = out;
  return out;
}

/** Closest point to `p` on a resolved reference. */
function closestOnRef(p: Vec2, r: ResolvedMeasureRef): Vec2 {
  if (r.kind === "point") return r.p;
  const ab = sub(r.b, r.a);
  const l2 = lenSq(ab);
  if (l2 < 1e-12) return r.a;
  const t = Math.max(0, Math.min(1, dot(sub(p, r.a), ab) / l2));
  return add(r.a, scale(ab, t));
}

/**
 * The shortest segment joining two resolved references, or null when they already
 * touch (a shared corner, a point on its line, crossing lines) — nothing to draw then.
 */
function sketchLink(a: ResolvedMeasureRef, b: ResolvedMeasureRef): [Vec2, Vec2] | null {
  if (a.kind === "line" && b.kind === "line") {
    // Non-parallel lines meet where their parameters both fall within range.
    const da = sub(a.b, a.a);
    const db = sub(b.b, b.a);
    const den = cross(da, db);
    if (Math.abs(den) > 1e-12) {
      const w = sub(b.a, a.a);
      const t = cross(w, db) / den;
      const u = cross(w, da) / den;
      const inA = t >= -1e-9 && t <= 1 + 1e-9;
      const inB = u >= -1e-9 && u <= 1 + 1e-9;
      if (inA && inB) return null;
    }
  }
  // Otherwise the closest pair involves one element's defining point: try each point
  // of either element against the other and keep the shortest.
  const ptsA = a.kind === "point" ? [a.p] : [a.a, a.b];
  const ptsB = b.kind === "point" ? [b.p] : [b.a, b.b];
  let best: [Vec2, Vec2] | null = null;
  let bestD = Infinity;
  const consider = (p: Vec2, q: Vec2) => {
    const d = dist(p, q);
    if (d < bestD) {
      bestD = d;
      best = [p, q];
    }
  };
  for (const p of ptsA) consider(p, closestOnRef(p, b));
  for (const q of ptsB) consider(closestOnRef(q, a), q);
  return bestD > 2 / view.scale ? best : null;
}

/** Constraint-tool overlay: the picked reference(s) and the one under the cursor. */
function sketchDraftView(): { refs: MeasureHighlight[]; hover: MeasureHighlight | null } | null {
  if (tool === null || !CONSTRAINT_TOOLS.has(tool)) return null;
  const highlight = tool === "equal" ? radiusHighlightOfRef : highlightOfRef; // Equal picks a corner by its arc
  const refs = constraintPicks.map(highlight).filter((r): r is MeasureHighlight => r !== null);
  let hover: MeasureHighlight | null = null;
  if (cursor) {
    const h = constraintRefAt(cursor);
    hover = h ? highlight(h) : null;
  }
  return { refs, hover };
}

/** Body-from-joints overlay: the picked-joint outline, plus the expanded preview when sizing. */
function bodyJointDraftView(): { outline: Vec2[]; preview: Vec2[] | null } | null {
  if (mode !== "draw" || tool !== "polyline" || jointDraftIds.length === 0) return null;
  const outline = jointDraftIds.map((id) => scene.jointWorld(scene.getJoint(id)!));
  let preview: Vec2[] | null = null;
  if (jointDraftExpanding && cursor) {
    const margin = Math.max(JOINT_BODY_MIN_MARGIN, dist(cursor, outline[outline.length - 1]));
    preview = roundedConvexBody(outline, margin);
  }
  return { outline, preview };
}

function frame(now?: number): void {
  // Animation tick: advance every actuator/motor phase by dt (capped to avoid huge jumps
  // after the tab is backgrounded), then solve once per frame with the computed anchors so
  // the driven joints reach their targets and propagate motion through pins/sliders.
  if (mode === "sim" && animating) {
    const t = now ?? performance.now();
    const dt = animLastTimestamp === null ? 0 : Math.min(0.1, (t - animLastTimestamp) / 1000);
    animLastTimestamp = t;
    if (dt > 0) advancePhases(dt);
    timedSolve("anim", driver, animIterations, computeAnchors());
    // Safety: stop the animation once the assembly has reported breaks for a few frames
    // in a row (a brief debounce filters single-frame solver chatter on complex loops).
    impossibleFrames = solveBreaks.length > 0 ? impossibleFrames + 1 : 0;
    if (pauseOnImpossible && impossibleFrames >= IMPOSSIBLE_PAUSE_FRAMES) setAnimating(false);
  } else if (mode === "sim" && driver) {
    timedSolve("drive", driver);
  } else if (mode === "draw" && leftDrag?.kind === "rigid" && leftDrag.moved) {
    // Draw-mode rigid (Shift) drag: drive the grabbed selection with the rest of the
    // scene frozen. The solved poses ARE the drawn layout (persisted on mouseup).
    timedSolve("rigidDrag", leftDrag.driver, 100, undefined, leftDrag.freeze);
  }
  syncColorPicker();
  syncPropsPanel();
  syncUnitSelect();
  syncCompPanelHighlight();
  pruneTempDims();
  if (sketchFlash && performance.now() >= sketchFlash.until) sketchFlash = null;
  // A hover is a matter of time: while the cursor rests mid-drag (no pointer events
  // arrive then) promote a hover that has lasted long enough — only then, so the target
  // scan doesn't run every frame on a big scene.
  if (mode === "draw" && leftDrag && "align" in leftDrag && leftDrag.align) {
    const al = leftDrag.align;
    const now = performance.now();
    const held = al.hover;
    const due =
      held !== null && now - held.since >= ALIGN_HOVER_MS && !al.cands.some((c) => sameMeasureRef(c, held.ref));
    if (due) updateDragAlign(leftDrag, al, now);
  }
  // Containment check (draw mode): flag joints a shape change stranded outside their
  // body. Refresh the hint when the count changes so the warning appears/clears itself.
  const prevOutside = containmentErrors.size;
  containmentErrors = mode === "draw" ? new Set(scene.jointsOutsideBody()) : new Set();
  if (containmentErrors.size !== prevOutside) updateHint();
  pruneFeatureSel(); // the feature selection follows the single body selection + live geometry
  // An open group can vanish under the editor (ungrouped, deleted, undone, loaded over):
  // isolation ends with it rather than veiling the scene against nothing. The selection
  // is left alone — whatever removed the group already decided what stays selected.
  if (groupEdit !== null && !editedGroup()) {
    groupEdit = null;
    updateCrumbBar();
    updateHint();
  }
  render(ctx, renderInput());
  requestAnimationFrame(frame);
}

/** Everything the renderer needs for the current frame (also replayed by the automation
 *  hook to capture the canvas as SVG for the manual). */
function renderInput(): RenderInput {
  return {
    scene,
    view,
    mode,
    isolate: (() => {
      const g = mode === "draw" ? editedGroup() : null;
      return g ? { bodies: g.bodyIds, joints: g.jointIds, items: isolateItems() } : null;
    })(),
    shapeDraft: shapeDraftView(),
    patternPreview: patternPreviewView(),
    patterns: patternViews(),
    draftBody: mode === "draw" && tool === "split" ? splitDraft : null,
    cursor,
    hoverJoint,
    hoverBody: mode === "draw" && tool === null ? hoverBody : null,
    highlightOccurrences: highlightedOccurrences(),
    ghostScenes: ghostScenes(),
    activeJoints: activeJoints(),
    // In sim only a measurement selection is meaningful (labels stay editable there). A
    // selected temporary context dimension highlights like a measurement (its negative
    // id can't collide with the scene's).
    selection: renderSelection(),
    multiSelected:
      mode === "draw" && multiSel
        ? { bodies: [...multiSel.bodies], joints: [...multiSel.joints] }
        : null,
    marquee: boxSelect?.moved
      ? { a: boxSelect.start, b: boxSelect.end }
      : featureBox?.moved
        ? { a: featureBox.start, b: featureBox.end }
        : null,
    viewRotate: viewRotate
      ? { centre: dialCentre(), radius: dialRadius(), dragging: viewRotate.drag !== null, hot: viewRotate.hot }
      : null,
    featureSelected: featureSelectedView(),
    editVertices: editVerticesView(),
    regularCentres: regularCentresView(),
    regularTags: regularTagsView(),
    filletHandles: filletHandlesView(),
    railDraft: railDraftView(),
    bodyJointDraft: bodyJointDraftView(),
    driverJoint: driver?.jointId ?? null,
    rotatePivot: rotateDrag?.pivot ?? null,
    gridStep,
    gridVisible,
    gridStyle: gridLook.style,
    gridColor: gridColorOverride(),
    breaks: mode === "sim" ? solveBreaks : [],
    containmentErrors,
    measurements: measurementsView().concat(tempDimsView()),
    measureDraft: measureDraftView(),
    sketchGlyphs: sketchGlyphsView(),
    sketchDraft: sketchDraftView(),
    dragSnap: dragSnapView(),
    dragAlign: dragAlignView(),
    flash: sketchFlash?.ids ?? null,
    theme: theme === "light" ? LIGHT_THEME : DARK_THEME,
  };
}

resize();
const restored = restoreAutosave();
pushHistory(); // seed the undo history with the initial (restored) layout
void initFileState(restored);
syncModeToggle();
updateHint();
requestAnimationFrame(frame);

// --- in-app help (src/help.ts) -----------------------------------------------------
/**
 * Help mode: the manual topic for whatever is drawn under a canvas point (screen px).
 * Same hit tests and priorities as a selecting click (labels and badges first, then
 * joints, guides, rails, holes, bodies), with a joint's role deciding its topic. Empty
 * canvas explains the armed tool, else the current mode.
 */
function canvasTopicAt(s: Vec2): string {
  const el = (t: CanvasTopic): string => t;
  const p = screenToWorld(view, s);
  const r = pickRadius();
  const cs = scene.constraints;
  if (viewRotate && dialHit(s)) return "view";
  if (tempDimLabelAt(p)) return el("el-context-dimension");
  if (measurementLabelAt(p)) return el("el-dimension");
  if (patternLabelAt(p) || patternHandleAt(p)) return el("el-pattern");
  const badge = sketchGlyphAt(p);
  if (badge !== null) {
    // Each constraint kind has its own topic (shared with its tool); the badge overview
    // is the fallback for a badge whose constraint is gone.
    const kind = scene.getSketchConstraint(badge)?.kind;
    return kind ? `tool-${kind}` : el("el-constraint");
  }
  if (selection?.kind === "body" && (selectedBodyFilletHandleAt(p) || selectedBodyNodeAt(p))) return el("el-handles");
  const j = scene.jointAt(p, r, mode === "sim" ? "rider" : "rail");
  if (j) {
    const id = j.id;
    if (cs.some((c) => c.kind === "motor" && (c.pivotJointId === id || c.crankJointId === id))) return el("el-motor");
    if (cs.some((c) => c.kind === "linearActuator" && c.riderId === id)) return el("el-actuator");
    if (cs.some((c) => c.kind === "slider" && c.locked.includes(id))) return el("el-slider");
    if (cs.some((c) => c.kind === "slider" && c.riders.includes(id))) return el("el-rider");
    if (cs.some((c) => c.kind === "ground" && c.joint === id)) return el("el-ground");
    if (cs.some((c) => c.kind === "pin" && c.rigid === true && (c.jointA === id || c.jointB === id))) return el("el-weld");
    if (cs.some((c) => c.kind === "pin" && (c.jointA === id || c.jointB === id))) return el("el-pin");
    if (cs.some((c) => c.kind === "slider" && (c.railA === id || c.railB === id))) return el("el-rail");
    if (scene.instanceOfJoint(id)) return el("el-instance");
    return el(j.bodyId === null ? "el-free-joint" : "el-joint");
  }
  if (scene.guidePointAt(p, r) || scene.guideAt(p, r)) return el("el-guide");
  const rail = scene.sliderAt(p, r);
  if (rail) return el(cs.some((c) => c.kind === "linearActuator" && c.sliderId === rail.id) ? "el-actuator" : "el-rail");
  const hole = scene.holeAt(p);
  if (!hole && patternAxisRefAt(p)) return el("el-pattern");
  if (hole) return el("el-hole");
  const body = scene.bodyAt(p);
  if (body) {
    if (scene.instanceOfBody(body.id)) return el("el-instance");
    if (scene.groupOf(body.id)) return el("el-group");
    if (body.grounded) return el("el-grounded-body");
    return el("el-body");
  }
  if (ghostRefAt(p)) return el("el-ghost");
  if (mode === "sim" && solveBreaks.some((b) => distToSegment(p, b.a, b.b) <= r)) return "impossible";
  return tool ? `tool-${tool}` : `mode-${mode}`;
}
const help = installHelp({ canvasTopic: (at) => canvasTopicAt(vec(at.x, at.y)) });

// --- automation hook (src/automation.ts) ---------------------------------------------
// Only for the manual generator (scripts/manual): dev server, or `?automation` in the URL.
// Loaded lazily so the production bundle carries none of it unless asked for.
if (import.meta.env.DEV || new URLSearchParams(location.search).has("automation")) {
  void import("./automation").then(({ installAutomation }) =>
    installAutomation({
      scene,
      view,
      canvas,
      loadDocument: applyLoadedScene,
      setMode,
      setTool: (t) => setTool(t as Tool),
      disarmTool,
      fitView,
      setTheme,
      getTheme: () => theme,
      setGridVisible: (on) => {
        gridVisible = on;
        gridBtn.classList.toggle("active", on);
      },
      setSnap: (on) => {
        snapEnabled = on;
        snapBtn.classList.toggle("active", on);
      },
      setObjSnap: (on) => {
        objSnapEnabled = on;
        osnapBtn.classList.toggle("active", on);
      },
      setGridStep,
      setSelection: (sel) => {
        multiSel = null;
        selection = sel as Selection | null;
      },
      setMulti: (bodies, joints) => setMulti(new Set(bodies), new Set(joints)),
      enterComponent,
      exitComponent,
      setCompPanelVisible,
      setAnimating,
      setCursor: (p) => {
        cursor = p;
      },
      renderInput,
      state: () => ({
        mode,
        tool,
        selection,
        multiSel: multiSel ? { bodies: [...multiSel.bodies], joints: [...multiSel.joints] } : null,
        editPath: [...editPath],
        groupEdit,
        theme,
        animating,
        gridVisible,
        snapEnabled,
        objSnapEnabled,
        helpOpen: help.isOpen(),
      }),
    })
  );
}
