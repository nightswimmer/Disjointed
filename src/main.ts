import "./style.css";
import { notify } from "./notify";
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
  SketchConstraintKind,
  sameMeasureRef,
  measureInfoFor,
  measureAxisForPlacement,
  refCenter,
  Unit,
  UNIT_TO_MM,
  cascadeComponentChange,
  reexpandData,
  PatternSeed,
} from "./model";
import { buildContextGhost, GhostSource } from "./context";
import { parseDxf, nestLoops, loopSignedArea } from "./dxf";
import { collectCutSheet, toDxf, toSvg } from "./export";
import { solve, Driver, ConstraintBreak, SolveStats, SolveFreeze, solverConfig, resetPoseBaselines } from "./solver";
import {
  solveSketch, tryAddConstraint, autoConstrainBody, SketchBreak, AUTO_HV_TOL,
  anchorVarsForBody, anchorVarsForJoint, anchorVarsForGuide, anchorVarForGuidePoint, anchorVarForVertex,
} from "./sketch";
import { applyDimensionValue, enforcePose, placeConstraint, poseConstraintViolated } from "./pose";
import { render, RenderInput, PatternView, DARK_THEME, LIGHT_THEME, SketchGlyphView } from "./renderer";
import { Vec2, add, dist, sub, vec, dot, cross, lenSq, scale, rotate, normalize, perp, roundedConvexBody, filletCornerArcs, distToSegment, distToLine } from "./geometry";
import { View, MIN_SCALE, MAX_SCALE, screenToWorld, worldToScreen, zoomAt, rotateViewTo, rotateToScreen, rotateToWorld } from "./view";
import { installHelp } from "./help";
import { CanvasTopic } from "./helpmap";

type Mode = "draw" | "sim";
type Tool =
  | "body" | "hole" | "split" | "joint" | "weld" | "connect" | "ground" | "rail" | "slider" | "rotate" | "guide"
  | "linearActuator" | "motor" | "measure" | "patternLinear" | "patternCircular"
  | SketchConstraintKind; // each sketch-constraint kind is its own one-shot tool
/** An existing element picked in normal/select mode. */
type Selection = { kind: "body" | "joint" | "rail" | "measure" | "sketch" | "guide" | "pattern" | "tempDim"; id: number };

/** The tools that place a sketch constraint (tool name = constraint kind). */
const CONSTRAINT_TOOLS = new Set<Tool>([
  "coincident", "horizontal", "vertical", "parallel", "perpendicular", "equal",
]);

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
const toolGroup = document.getElementById("tool-group")!;
const editGroup = document.getElementById("edit-group")!;
const gridBtn = document.getElementById("grid-btn") as HTMLButtonElement;
const snapBtn = document.getElementById("snap-btn") as HTMLButtonElement;
const osnapBtn = document.getElementById("osnap-btn") as HTMLButtonElement;
const gridSizeBtn = document.getElementById("grid-size-btn") as HTMLButtonElement;
const gridSizeValue = document.getElementById("grid-size-value") as HTMLSpanElement;
const gridSizeMenu = document.getElementById("grid-size-menu") as HTMLDivElement;
const gridSizeList = document.getElementById("grid-size-list") as HTMLDivElement;
const gridSizeAddForm = document.getElementById("grid-size-add") as HTMLFormElement;
const gridSizeNew = document.getElementById("grid-size-new") as HTMLInputElement;
const themeBtn = document.getElementById("theme-btn") as HTMLButtonElement;
const colorGroup = document.getElementById("color-group")!;
const colorInput = document.getElementById("body-color") as HTMLInputElement;
const actuatorGroup = document.getElementById("actuator-group")!;
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
const sketchGroup = document.getElementById("sketch-group")!;
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
}
function toggleTheme(): void {
  setTheme(theme === "dark" ? "light" : "dark");
}
applyTheme();
themeBtn.addEventListener("click", toggleTheme);

// --- body colour ---------------------------------------------------------
// The toolbar colour input does double duty: with a body selected it recolours that body;
// with nothing selected it sets the colour applied to newly drawn bodies. The swatch is
// kept in sync with the current selection by `syncColorPicker` (called each frame).
let defaultBodyColor = colorInput.value;
colorInput.addEventListener("input", () => {
  const c = colorInput.value;
  if (selection?.kind === "body") {
    const body = scene.getBody(selection.id);
    if (body) {
      body.color = c;
      markDirty();
    }
  } else {
    defaultBodyColor = c;
  }
});
/** Reflect the selected body's colour (or the new-body default) in the swatch. */
let colorSyncKey = "";
function syncColorPicker(): void {
  const body = selection?.kind === "body" ? scene.getBody(selection.id) : null;
  const key = body ? `b${body.id}:${body.color}` : `d:${defaultBodyColor}`;
  if (key === colorSyncKey) return; // avoid clobbering the picker mid-drag
  colorSyncKey = key;
  colorInput.value = body ? body.color : defaultBodyColor;
}

// --- interaction state ---------------------------------------------------
let mode: Mode = "draw";
/** Armed draw tool, or null for normal/select mode. Tools disarm after one use. */
let tool: Tool | null = null;
let draftBody: Vec2[] = []; // freehand polygon vertices (body tool, empty-space start)
/**
 * Per freehand draft vertex: the existing point (joint / body corner) the click landed
 * on, or null. On finish, each recorded pick becomes a coincident auto-constraint
 * between the new body's corner and that point (the vertex is placed exactly on it).
 */
let draftBodySnaps: (MeasureRef | null)[] = [];
/** Hole tool: freehand cut-out vertices, and the body being cut (set by the first click). */
let holeDraft: Vec2[] = [];
let holeDraftBodyId: number | null = null;
/**
 * Hole tool, round-hole gesture: the press that may become a circle drag (body picked,
 * snapped centre, and the screen point pressed), then the circle being dragged out
 * (radius clamped to `maxR`, the largest disk that fits the body around that centre).
 * A press released without dragging falls back to the polygon path (first vertex).
 */
let holePress: { bodyId: number; centre: Vec2; screen: Vec2; maxR: number } | null = null;
// --- pattern tools ------------------------------------------------------------
/** What the armed pattern tool is replicating (a hole or an attached joint), or null before the pick. */
let patternSeed: PatternSeed | null = null;
/** Linear tool: the row just created, still armed for an optional second direction (a grid). */
let patternDraft: number | null = null;
let holeCircle: { c: Vec2; r: number } | null = null;
/** Screen-pixel travel before a hole-tool press counts as a circle drag. */
const HOLE_DRAG_PX = 4;
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
let guideDraft: Vec2 | null = null; // guide tool: the first defining point placed
/** The existing point element the first guide click landed on (→ coincident on commit). */
let guideDraftPick: MeasureRef | null = null;
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
const SKETCH_FLASH_MS = 1200;
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
 * other objects (plus guidelines and rails). Takes precedence over the grid/guide snap.
 */
let objSnapEnabled = false;
/** Screen-px capture range for object snapping (dragged reference onto a target feature). */
const OBJ_SNAP_PX = 12;
/** A line reference only snaps onto (near-)parallel lines: within this angle (≈2°). */
const OBJ_SNAP_PARALLEL_TOL = (2 * Math.PI) / 180;

/**
 * Implicit constraints while dragging (draw mode; any body / joint / vertex / multi drag
 * whose reference feature can take a sketch constraint — a joint, a control vertex, a
 * guide point or a control edge): holding the dragged reference over another point /
 * line for `ALIGN_HOVER_MS` arms that element as the *alignment candidate* (a later
 * hover over something else replaces it; Esc drops it). Releasing the drag with the
 * reference H/V-aligned with a candidate point, or on the infinite line of a candidate
 * line, within `ALIGN_TOL_PX` creates the matching sketch constraint automatically — a
 * dotted line with the constraint's badge previews it during the drag.
 */
const ALIGN_HOVER_MS = 400;
const ALIGN_TOL_PX = 10;

/** Screen-px capture range for snapping onto a construction guideline. */
const GUIDE_SNAP_PX = 10;

/**
 * Snap a world point (identity when snap is off). Construction guidelines take
 * precedence over the grid: within capture range of one guideline the point projects
 * onto its infinite line; within range of two, it lands on their intersection. Away
 * from any guideline it snaps to the nearest grid intersection. `excludeGuide` leaves
 * one guideline out, so dragging a guide never snaps it onto itself.
 */
function snap(p: Vec2, excludeGuide?: number): Vec2 {
  if (!snapEnabled) return p;
  const r = GUIDE_SNAP_PX / view.scale;
  const near: { o: Vec2; d: Vec2; dist: number; proj: Vec2 }[] = [];
  for (const g of scene.guides) {
    if (g.id === excludeGuide) continue;
    const d = normalize(sub(g.b, g.a));
    if (d.x === 0 && d.y === 0) continue;
    const proj = add(g.a, scale(d, dot(sub(p, g.a), d)));
    const dd = dist(p, proj);
    if (dd <= r) near.push({ o: g.a, d, dist: dd, proj });
  }
  if (near.length > 0) {
    near.sort((x, y) => x.dist - y.dist);
    // Two (non-parallel) guidelines in range: land exactly on their intersection.
    for (let i = 1; i < near.length; i++) {
      const den = cross(near[0].d, near[i].d);
      if (Math.abs(den) < 1e-6) continue; // (near-)parallel — no usable intersection
      const t = cross(sub(near[i].o, near[0].o), near[i].d) / den;
      const q = add(near[0].o, scale(near[0].d, t));
      if (dist(p, q) <= r) return q;
    }
    return near[0].proj;
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
 * dragged object that snaps (a control vertex, an edge midpoint or the centre as a
 * `bodyPoint`, a control edge, or a joint), re-resolved live each frame — the drag anchor
 * is its point (a line's midpoint). `hit` is what it last snapped onto (for the
 * highlight), drawn as an infinite line when `hitInfinite`.
 */
type DragObjSnap = { ref: MeasureRef; hit: ResolvedMeasureRef | null; hitInfinite: boolean };

/**
 * Implicit-constraint state carried by a body / joint / vertex / multi drag: `ref` is the
 * dragged reference (the feature object snap would use, when it can take a constraint —
 * independent of the object-snap toggle), `hover` the target it currently sits on and
 * since when, `cand` the armed candidate, `match` the constraint a release right now
 * would create. `slip` is the last snap correction (unsnapped − snapped anchor), so a
 * hover is judged where the cursor put the reference, not where the grid moved it.
 */
type DragAlign = {
  ref: MeasureRef;
  hover: { ref: MeasureRef; since: number } | null;
  cand: MeasureRef | null;
  match: AlignMatch | null;
  slip: Vec2;
};
/** A previewed implicit constraint: its kind and the dotted preview line's endpoints. */
type AlignMatch = { kind: SketchConstraintKind; from: Vec2; to: Vec2 };

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
  | { kind: "guidePoint"; id: number; which: "a" | "b"; grabOffset: Vec2; moved: boolean }
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
  if (d.kind === "guide") return scene.getGuide(d.id)!.a;
  if (d.kind === "guidePoint") return scene.getGuide(d.id)![d.which];
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
        const mref: MeasureRef = { kind: "bodyPoint", bodyId: body.id, local: rotate(sub(m, body.pos), -body.angle) };
        mid = better(mid, bodyRef(body, m, mref, dist(grab, m)));
        const eref: MeasureRef =
          hole === null ? { kind: "edge", bodyId: body.id, index: i } : { kind: "edge", bodyId: body.id, index: i, hole };
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
 *  body centres and edge midpoints). */
type SnapPoint = { p: Vec2; ref: MeasureRef | null };
type SnapLine = { a: Vec2; b: Vec2; infinite: boolean; ref: MeasureRef | null };

/**
 * Object-snap targets: the same features on everything that isn't being dragged —
 * other bodies' control vertices, edge midpoints, centroids and control edges (outer +
 * holes), joints (not on a dragged body), rails, and guidelines (defining points +
 * the infinite line).
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
      for (const l of t.lines) lines.push({ a: l.a, b: l.b, infinite: l.infinite, ref: null });
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
    if (!ex) points.push({ p: body.pos, ref: null });
    for (const { verts, hole } of bodyControlLoops(body, s)) {
      const n = verts.length;
      const exAt = (i: number): boolean => !!ex && ex.has(vertKey(hole, i));
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
        points.push({ p: scale(add(v, w), 0.5), ref: null });
        lines.push({
          a: v,
          b: w,
          infinite: false,
          ref: hole === null ? { kind: "edge", bodyId: body.id, index: i } : { kind: "edge", bodyId: body.id, index: i, hole },
        });
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
    lines.push({
      a: s.jointWorld(s.getJoint(c.railA)!),
      b: s.jointWorld(s.getJoint(c.railB)!),
      infinite: false,
      ref: { kind: "rail", sliderId: c.id },
    });
  }
  for (const g of s.guides) {
    points.push({ p: g.a, ref: { kind: "guidePoint", guideId: g.id, which: "a" } });
    points.push({ p: g.b, ref: { kind: "guidePoint", guideId: g.id, which: "b" } });
    lines.push({ a: g.a, b: g.b, infinite: true, ref: { kind: "guideLine", guideId: g.id } });
  }
  return { points, lines };
}

/**
 * Object-snap a drag: `raw` is where the drag anchor (the reference point, or a line
 * reference's midpoint) would land unsnapped. A point reference lands on the nearest
 * target point within range, else projects onto the nearest target line (segments
 * clamped, guidelines infinite). A line reference translates perpendicular onto the
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
    let bestL: { a: Vec2; b: Vec2; infinite: boolean } | null = null;
    let proj: Vec2 | null = null;
    bd = r;
    for (const l of targets.lines) {
      const ab = sub(l.b, l.a);
      const L = lenSq(ab);
      if (L < 1e-12) continue;
      let t = dot(sub(raw, l.a), ab) / L;
      if (!l.infinite) t = Math.max(0, Math.min(1, t));
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
      os.hitInfinite = bestL.infinite;
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
  return r.kind === "joint" || r.kind === "vertex" || r.kind === "guidePoint";
}
function alignLineRef(r: MeasureRef): boolean {
  return r.kind === "rail" || r.kind === "edge" || r.kind === "guideLine" || r.kind === "patternAxis";
}

/** Fresh implicit-constraint state for a drag whose reference is `ref` — none when the
 *  reference can't take a constraint (a body centre / edge midpoint `bodyPoint`). */
function newDragAlign(ref: MeasureRef | null | undefined): DragAlign | undefined {
  if (!ref || !(alignPointRef(ref) || alignLineRef(ref))) return undefined;
  return { ref, hover: null, cand: null, match: null, slip: vec(0, 0) };
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
 * reference over a target point, else over a target line; a line reference over a target
 * point along its segment — within the object-snap range of where the *cursor* put the
 * reference (`slip` undoes the grid / object snap). Held for `ALIGN_HOVER_MS` it becomes
 * the candidate, replacing any earlier one; leaving it keeps the candidate armed.
 *
 * Matching (needs a candidate): point↔point within `ALIGN_TOL_PX` of the same y →
 * horizontal, same x → vertical (sitting right on top of it is a placement, not an
 * alignment — no match); a point on a candidate line's infinite line, or a candidate
 * point on the dragged line's, → point-on-line coincident. Judged on the geometry as
 * placed (snapped, live-solved) — that's what the release commits; the constraint's own
 * solve then closes the residual. A constraint that already exists never re-matches.
 */
function updateDragAlign(d: LeftDrag, al: DragAlign, now: number): void {
  al.match = null;
  const cur = scene.resolveMeasureRef(al.ref);
  if (!cur) {
    al.hover = null;
    al.cand = null;
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
        let u = dot(sub(at, t.a), ab) / L;
        if (!t.infinite) u = Math.max(0, Math.min(1, u));
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
  if (al.hover && now - al.hover.since >= ALIGN_HOVER_MS && !(al.cand && sameMeasureRef(al.cand, al.hover.ref))) {
    al.cand = al.hover.ref;
  }
  // --- candidate → match ---
  if (!al.cand) return;
  const cand = scene.resolveMeasureRef(al.cand);
  if (!cand) {
    al.cand = null;
    return;
  }
  const tol = ALIGN_TOL_PX / view.scale;
  let m: AlignMatch | null = null;
  if (cur.kind === "point" && cand.kind === "point") {
    const dx = Math.abs(cur.p.x - cand.p.x);
    const dy = Math.abs(cur.p.y - cand.p.y);
    if (dx <= tol && dy <= tol) m = null; // on top of it: a placement, not an alignment
    else if (dy <= tol) m = { kind: "horizontal", from: cand.p, to: cur.p };
    else if (dx <= tol) m = { kind: "vertical", from: cand.p, to: cur.p };
  } else if (cur.kind === "point" && cand.kind === "line") {
    m = pointOnLineMatch(cur.p, cand, tol);
  } else if (cur.kind === "line" && cand.kind === "point") {
    m = pointOnLineMatch(cand.p, cur, tol);
  }
  if (m && sketchConstraintExists(m.kind, al.cand, al.ref)) m = null;
  al.match = m;
}

/** Point-on-line match: `p` within `tol` of `line`'s infinite line. The dotted preview
 *  runs from the nearer end of the defining segment out to the point (collapsed to the
 *  point when it lies within the segment's span — the badge alone marks it then). */
function pointOnLineMatch(p: Vec2, line: Extract<ResolvedMeasureRef, { kind: "line" }>, tol: number): AlignMatch | null {
  const ab = sub(line.b, line.a);
  const L = lenSq(ab);
  if (L < 1e-12) return null;
  const u = dot(sub(p, line.a), ab) / L;
  const foot = add(line.a, scale(ab, u));
  if (dist(p, foot) > tol) return null;
  const from = u < 0 ? line.a : u > 1 ? line.b : foot;
  return { kind: "coincident", from, to: p };
}

/** Perpendicular foot of `p` on the infinite line through `a`–`b` (null when degenerate). */
function footOnLine(p: Vec2, a: Vec2, b: Vec2): Vec2 | null {
  const ab = sub(b, a);
  const L = lenSq(ab);
  if (L < 1e-12) return null;
  return add(a, scale(ab, dot(sub(p, a), ab) / L));
}

/**
 * The translation that makes a previewed alignment exact: the dragged reference onto
 * the candidate's y (horizontal) / x (vertical), a point onto the candidate line, or the
 * dragged line onto the candidate point. Applied before the constraint is placed, so the
 * solver starts from a satisfied constraint — asked to close even a small gap itself it
 * splits the correction with the other side, and when that side is pinned by dimensions
 * (a fully dimensioned part) the solve can fail and the placement gets rejected.
 */
function alignCorrection(kind: SketchConstraintKind, cur: ResolvedMeasureRef, cand: ResolvedMeasureRef): Vec2 | null {
  if (cur.kind === "point" && cand.kind === "point") {
    if (kind === "horizontal") return vec(0, cand.p.y - cur.p.y);
    if (kind === "vertical") return vec(cand.p.x - cur.p.x, 0);
    return null;
  }
  if (kind !== "coincident") return null;
  if (cur.kind === "point" && cand.kind === "line") {
    const foot = footOnLine(cur.p, cand.a, cand.b);
    return foot ? sub(foot, cur.p) : null;
  }
  if (cur.kind === "line" && cand.kind === "point") {
    const foot = footOnLine(cand.p, cur.a, cur.b);
    return foot ? sub(cand.p, foot) : null;
  }
  return null;
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
 * Place the constraint an implicit alignment previewed (on drag release). The dragged
 * reference goes second, so a pose constraint (both ends on components) moves the
 * dragged part rather than the candidate's. Selection stays on what was dragged; a
 * rejected solve flashes the conflicts like any constraint placement.
 */
function placeAlignConstraint(d: LeftDrag, al: DragAlign): void {
  if (!al.match || !al.cand) return;
  // Close the (sub-tolerance) gap exactly first — see alignCorrection.
  const cur = scene.resolveMeasureRef(al.ref);
  const cand = scene.resolveMeasureRef(al.cand);
  if (!cur || !cand) return;
  const corr = alignCorrection(al.match.kind, cur, cand);
  if (corr && (corr.x !== 0 || corr.y !== 0)) moveDragged(d, corr);
  const { constraint, breaks } = placeConstraint(scene, al.match.kind, al.cand, al.ref);
  if (!constraint) {
    // A silent no-op would read as "nothing happened": say why.
    if (breaks.length) {
      flashSketchItems(breaks);
      notify("Constraint not applied: it can't be satisfied without breaking an existing dimension or constraint.", "error");
    } else notify("Constraint not applied: not possible between these elements.", "error");
    markDirty(); // the alignment correction above moved the geometry
    return;
  }
  setSketchVisible(true); // a constraint placed while the layer is hidden would be invisible
  markDirty();
}

/** Implicit-constraint preview for the renderer: the armed candidate, the dragged
 *  reference, and the alignment a release would constrain (none without a candidate). */
function dragAlignView(): RenderInput["dragAlign"] {
  if (mode !== "draw" || !leftDrag || !("align" in leftDrag) || !leftDrag.align?.cand) return null;
  const ref = scene.resolveMeasureRef(leftDrag.align.ref);
  const cand = scene.resolveMeasureRef(leftDrag.align.cand);
  return ref && cand ? { ref, cand, match: leftDrag.align.match } : null;
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
  // pull in a group and vice versa.
  let grew = true;
  while (grew) {
    const before = bodies.size + joints.size;
    for (const id of [...bodies]) {
      const g = scene.groupOf(id);
      if (g) {
        for (const b of g.bodyIds) bodies.add(b);
        for (const j of g.jointIds) joints.add(j);
      }
      const inst = scene.instanceOfBody(id);
      if (inst) addInstanceMembers(inst, bodies, joints);
    }
    for (const id of [...joints]) {
      const g = scene.groupOfJoint(id);
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
    const g = bodyId !== null ? scene.groupOf(bodyId) : jointId !== null ? scene.groupOfJoint(jointId) : undefined;
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
    const g = scene.groupOf(id);
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
      const g = scene.groupOfJoint(j.id);
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
  for (const b of scene.bodies) {
    if (scene.bodyWorldVerts(b).every(inside)) bodies.add(b.id);
  }
  for (const j of scene.joints) {
    if (j.bodyId === null && inside(scene.jointWorld(j))) joints.add(j.id);
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
  viewRotate: "Rotate the view: drag the ring or a crosshair arm to turn the whole picture (snaps to 5°, hold Shift for any angle) · type an exact angle in the box under the centre · double-click the centre for 0° · click elsewhere, Esc or Shift+R to close. The drawing itself does not change.",
  sim: "Drag any joint, or part of a body, to drive the mechanism. Space to run / pause actuators.",
  select: "Click to select · drag to move · Shift+drag to move rigidly (sim-style: grounds hold, connections constrain, the rest stays put) · Object snap (toolbar) drags by the highlighted corner / midpoint / edge / centre nearest the grab and snaps it onto other objects · Ctrl+click or drag a box to select several bodies (they move together) · Ctrl+G groups them permanently / ungroups a group · with a body selected, Shift+drag a box from empty space to select several of its corners / holes / joints (Ctrl+Shift adds) — drag any of them to move the set, Delete removes it, Ctrl+C copies its holes + joints (with their constraints) and Ctrl+V pastes them into the selected body at the cursor · drag a selected body's corner handles to reshape · drag a round handle to round just that corner (double-click it to reset to the body's radius) · double-click an edge to add a node / a node to remove it · double-click a dimension to set its value · double-click a component instance to edit its definition (Ctrl+double-click shows the surrounding assembly faded in its frame — a context ghost to snap and dimension to; the breadcrumb eyes set how far out it reaches) · [ and ] round all corners · N combines the selected bodies into one · Delete to remove.",
  body: "Empty space: click vertices to draw a polygon. Joints: click joints to build a body, click a node again to finish, then move out to set thickness and click.",
  hole: "Round hole: press inside a body and drag out the radius. Polygon hole: click inside a body to start a cut-out, then click more vertices (all inside that body); click the first vertex (or press Enter) to close it.",
  split: "Click a point on a body's outline (edge or corner) to start the cut, click inside to route it, then click the outline again to split the body in two along the path.",
  patternLinear: "Click a hole or a joint on a body to repeat it along a line.", // live stage hint: patternHint()
  patternCircular: "Click a hole or a joint on a body to repeat it around a centre.",
  joint: "Click inside a body to attach a joint, or empty space to place a free joint.",
  weld: "Click where bodies overlap to weld them rigidly together at that point (no relative rotation) — or click an existing pinned joint to toggle it weld ↔ pin.",
  connect: "Click a joint, then another joint to pin them — or a rail to attach the joint to it as a rider.",
  ground: "Click a joint to lock its position (it can still rotate), or a body / group to fix it entirely; click again to unground.",
  rail: "Click two joints on the same body (a moving rail) — or two free joints (a fixed track) — to create a rail that joints and sliders can ride along (a pin-in-slot: riders placed on it slide and rotate).",
  slider: "Click a body where the slider starts (that body is the part that moves), then click where the travel ends — over another body the track rides that body, otherwise it is fixed in the world. On an existing rail: click it to add a slider there, or click a rider to toggle its rotation lock.",
  guide: "Click two points to place an infinite construction guideline — clicks land on joints, body corners and edges (points get a coincident constraint). Drag the line to move it (angle kept), or drag one of its two points to re-aim it. With snap on, placements prefer guidelines over the grid.",
  rotate: "Drag a body to rotate it about its centroid, or drag a selected body's node to rotate about that node. A multi-selection or group rotates as one about its centre. Snaps to 45°.",
  linearActuator: "Click a slider or rail to make it self-driving — its carriage travels back and forth when animation runs (a rail with no rider gets a free one).",
  motor: "Click a joint to set the pivot, then another joint on the same body for the crank pin.",
  measure: "Click two references — a joint, body corner, body edge, rail, guideline, or a point on a body — then click where the value should sit. Inside a component with the context ghost shown, a faded joint / corner / edge / rail makes a temporary dimension (double-click its value to move your geometry there).",
  coincident: "Click two points (joints, body corners, or guideline points) to make them share a position — or a point and a line (body edge, rail or guideline) to hold the point on the infinite line.",
  horizontal: "Click a body edge, rail or guideline — or two points — to make it horizontal.",
  vertical: "Click a body edge, rail or guideline — or two points — to make it vertical.",
  parallel: "Click two lines (body edges, rails or guidelines) to make them parallel.",
  perpendicular: "Click two lines (body edges, rails or guidelines) to make them perpendicular.",
  equal: "Click two lines (body edges or rails) to make their lengths equal.",
};

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

function updateHint(): void {
  const base =
    viewRotate ? HINTS.viewRotate
    : tool === "measure" ? HINTS.measure
    : mode === "sim" ? HINTS.sim
    : tool === null ? HINTS.select
    : isPatternTool(tool) ? patternHint()
    : HINTS[tool];
  hintEl.textContent = containmentWarning() + base;
}

// --- toolbar wiring ------------------------------------------------------
document.querySelectorAll<HTMLButtonElement>(".mode-btn").forEach((btn) => {
  btn.addEventListener("click", () => setMode(btn.dataset.mode as Mode));
});
document.querySelectorAll<HTMLButtonElement>(".tool-btn").forEach((btn) => {
  btn.addEventListener("click", () => setTool(btn.dataset.tool as Tool));
});
document.getElementById("clear-btn")!.addEventListener("click", () => {
  if (mode === "sim") return;
  // In a definition context, clear only that definition's content (the document's
  // component list survives); at the root, clear the whole document.
  scene.clear(editPath.length === 0);
  if (editPath.length === 0) setDocFile(null, null); // a fresh document: Ctrl+S asks where to save
  resetTransient();
  markDirty();
  updateCompPanel();
});
document.getElementById("fit-btn")!.addEventListener("click", fitView);
rotateViewBtn.addEventListener("click", () => setViewRotateOpen(viewRotate === null));
document.getElementById("save-btn")!.addEventListener("click", (e) => void saveToFile(e.shiftKey));
document.getElementById("load-btn")!.addEventListener("click", () => void openFile());
// Copy/paste are keyboard-only (Ctrl/Cmd+C / V); no toolbar buttons.
document.getElementById("mirror-h-btn")!.addEventListener("click", () => mirrorSelection("h"));
document.getElementById("mirror-v-btn")!.addEventListener("click", () => mirrorSelection("v"));
document.getElementById("combine-btn")!.addEventListener("click", combineSelection);
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

gridBtn.addEventListener("click", () => {
  gridVisible = !gridVisible;
  gridBtn.classList.toggle("active", gridVisible);
});
snapBtn.addEventListener("click", () => {
  snapEnabled = !snapEnabled;
  snapBtn.classList.toggle("active", snapEnabled);
});
osnapBtn.addEventListener("click", () => {
  objSnapEnabled = !objSnapEnabled;
  osnapBtn.classList.toggle("active", objSnapEnabled);
});
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
gridSizeBtn.addEventListener("click", () => {
  if (gridSizeMenu.classList.contains("hidden")) openGridSizeMenu();
  else closeGridSizeMenu();
});
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
  document.querySelectorAll<HTMLButtonElement>(".mode-btn").forEach((b) =>
    b.classList.toggle("active", b.dataset.mode === mode)
  );
  toolGroup.classList.toggle("hidden", mode === "sim");
  editGroup.classList.toggle("hidden", mode === "sim");
  colorGroup.classList.toggle("hidden", mode === "sim");
  actuatorGroup.classList.toggle("hidden", mode === "sim");
  sketchGroup.classList.toggle("hidden", mode === "sim");
  document.getElementById("component-group")!.classList.toggle("hidden", mode === "sim");
  runBtn.classList.toggle("hidden", mode === "draw");
  autopauseBtn.classList.toggle("hidden", mode === "draw");
  animIterCtrl.classList.toggle("hidden", mode === "draw");
  cleanupMaxCtrl.classList.toggle("hidden", mode === "draw");
  structTolCtrl.classList.toggle("hidden", mode === "draw");
  breakTolCtrl.classList.toggle("hidden", mode === "draw");
  canvas.style.cursor = mode === "sim" ? "grab" : "crosshair";
  updateHint();
  updateSimError(); // show/hide the banner for the mode we just entered
  syncPropsPanel(); // selection cleared → properties panels hide
}

function setTool(next: Tool): void {
  // Rotate operates on the current selection, so keep an existing body selection (lets
  // you grab one of its control nodes as the pivot right away) — or multi-selection /
  // group (so R then drag rotates the whole set) — when arming it.
  const keepSel = next === "rotate" && selection?.kind === "body" ? selection : null;
  const keepMulti = next === "rotate" ? multiSel : null;
  // Pattern works selection-first too: an already selected joint becomes the seed.
  const seedJoint = isPatternTool(next) && selection?.kind === "joint" ? selection.id : null;
  tool = next;
  resetTransient();
  selection = keepSel;
  multiSel = keepMulti;
  if (seedJoint !== null) seedPatternJoint(seedJoint);
  document.querySelectorAll<HTMLButtonElement>(".tool-btn").forEach((b) =>
    b.classList.toggle("active", b.dataset.tool === tool)
  );
  updateHint();
}

/** Return to normal/select mode after a tool finishes placing one element. */
function disarmTool(): void {
  tool = null;
  resetTransient();
  document
    .querySelectorAll<HTMLButtonElement>(".tool-btn")
    .forEach((b) => b.classList.remove("active"));
  updateHint();
}

function resetTransient(): void {
  closeDimEditor();
  draftBody = [];
  draftBodySnaps = [];
  holeDraft = [];
  holeDraftBodyId = null;
  holePress = null;
  holeCircle = null;
  patternSeed = null;
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
  guideDraft = null;
  guideDraftPick = null;
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
  saveBtn.title = `Save ${target} (Ctrl+S) — Shift-click or Ctrl+Shift+S to save as a new file`;
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
  crumbBar.classList.toggle("hidden", editPath.length === 0);
  crumbBar.innerHTML = "";
  if (editPath.length === 0) return;
  const n = editPath.length;
  const names = ["Assembly", ...editPath.map((id) => scene.getComponent(id)?.name ?? `#${id}`)];
  names.forEach((name, i) => {
    if (i > 0) {
      const sep = document.createElement("span");
      sep.className = "crumb-sep";
      sep.textContent = "▸";
      crumbBar.appendChild(sep);
    }
    if (i === names.length - 1) {
      const cur = document.createElement("span");
      cur.className = "crumb-current";
      cur.textContent = name;
      crumbBar.appendChild(cur);
    } else {
      const btn = document.createElement("button");
      btn.className = "crumb";
      btn.textContent = name;
      btn.title = `Back to ${name}`;
      btn.addEventListener("click", () => exitComponent(n - i));
      crumbBar.appendChild(btn);
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

/** Move a temporary dimension's label (a point pair re-derives h / v / direct, like the scene's). */
function setTempDimLabel(td: TempDim, labelPos: Vec2): void {
  const a = resolveTemp(td.refA);
  const b = resolveTemp(td.refB);
  if (!a || !b) return;
  const anchor = scale(add(refCenter(a), refCenter(b)), 0.5);
  td.labelOffset = sub(labelPos, anchor);
  if (a.kind === "point" && b.kind === "point") td.axis = measureAxisForPlacement(a.p, b.p, labelPos);
}

function removeTempDim(id: number): void {
  const list = curTempDims();
  const i = list.findIndex((t) => t.id === id);
  if (i >= 0) list.splice(i, 1);
}

/** The temporary dimension whose label sits under `p` (topmost first), or null. */
function tempDimLabelAt(p: Vec2): TempDim | null {
  if (!measureVisible || mode !== "draw") return null;
  const list = curTempDims();
  for (let i = list.length - 1; i >= 0; i--) {
    const lp = tempDimLabelPos(list[i]);
    if (lp && dist(lp, p) <= LABEL_PICK_RADIUS / view.scale) return list[i];
  }
  return null;
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
  const g = scene.groupOf(bodyId);
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
  const reject = (): boolean => {
    flashSketchItems([{ id: td.id, kind: "dimension", error: Infinity }]);
    return false;
  };
  const liveIsA = td.refA.kind !== "ghost";
  const liveIsB = td.refB.kind !== "ghost";
  if (liveIsA === liveIsB) return reject(); // both ends on the ghost: nothing here can move
  const liveRef = (liveIsA ? td.refA : td.refB) as MeasureRef;
  const a = resolveTemp(td.refA);
  const b = resolveTemp(td.refB);
  const info = tempDimInfo(td);
  if (!a || !b || !info || info.kind !== "distance") return reject();
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
  } else return reject();
  if (!Number.isFinite(delta.x) || !Number.isFinite(delta.y)) return reject();

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
  switch (liveRef.kind) {
    case "vertex":
      if (scene.instanceOfBody(liveRef.bodyId)) moveUnit(moveUnitOfBody(liveRef.bodyId));
      else {
        scene.moveBodyVertex(liveRef.bodyId, liveRef.index, delta, liveRef.hole ?? null);
        anchors.add(anchorVarForVertex(liveRef.bodyId, liveRef.index, liveRef.hole ?? null));
      }
      break;
    case "joint": {
      const j = scene.getJoint(liveRef.jointId);
      if (!j) return reject();
      const inst = scene.instanceOfJoint(j.id);
      if (inst) {
        const bodies = new Set<number>();
        const joints = new Set<number>();
        addInstanceMembers(inst, bodies, joints);
        moveUnit({ bodies: [...bodies], joints: [...joints], instanceId: inst.id });
      } else if (j.bodyId === null && scene.groupOfJoint(j.id)) {
        const g = scene.groupOfJoint(j.id)!;
        moveUnit({ bodies: [...g.bodyIds], joints: [...g.jointIds], instanceId: null });
      } else {
        scene.moveJoint(j.id, delta);
        for (const k of anchorVarsForJoint(scene, j.id)) anchors.add(k);
      }
      break;
    }
    case "edge":
    case "bodyPoint":
      moveUnit(moveUnitOfBody(liveRef.bodyId));
      break;
    case "rail": {
      const c = scene.constraints.find((cc) => cc.id === liveRef.sliderId);
      if (!c || c.kind !== "slider") return reject();
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
      const g = scene.getGuide(liveRef.guideId);
      if (!g) return reject();
      scene.moveGuidePoint(liveRef.guideId, liveRef.which, add(liveRef.which === "a" ? g.a : g.b, delta));
      anchors.add(anchorVarForGuidePoint(liveRef.guideId, liveRef.which));
      break;
    }
    case "guideLine":
      scene.moveGuide(liveRef.guideId, delta);
      for (const k of anchorVarsForGuide(liveRef.guideId)) anchors.add(k);
      break;
    default:
      return reject(); // a pattern axis is derived geometry — edit the pattern instead
  }
  // Like a drag: pose partners follow the moved instances, then the anchored sketch
  // solve lets free geometry adapt; if the anchored solve is infeasible the symmetric
  // one decides where things can actually go.
  enforcePose(scene, movedInstances.size ? movedInstances : undefined);
  if (anchors.size === 0 || solveSketch(scene, anchors).length > 0) solveSketch(scene);
  const after = tempDimInfo(td);
  if (!after || Math.abs(after.value - target) > TEMP_DIM_TOL) {
    scene.loadContext(JSON.parse(before) as SceneData);
    return reject();
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
      hintEl.textContent = `Forked into new component “${copy.name}” — the selected instance now follows it.`;
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
    hintEl.textContent = `Created empty component “${def.name}” — draw bodies or place instances of other components, then navigate back.`;
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
  hintEl.textContent = `Click the canvas to place an instance of “${def?.name ?? "?"}” — Esc cancels.`;
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
    case "body":
      handleBodyClick(p);
      break;
    case "hole":
      // Spans many clicks like the body tool; disarms itself in finishHole().
      handleHoleClick(p);
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
    case "guide": {
      // Two clicks define an infinite construction guideline. Each click lands exactly
      // on an existing point element (joint / body corner / guide point — recorded for
      // an auto-coincident), projects onto a rail / body edge, or grid/guide-snaps.
      const { at, pick } = guidePlacementAt(p);
      if (guideDraft === null) {
        guideDraft = at;
        guideDraftPick = pick;
        return; // nothing committed yet
      }
      if (dist(at, guideDraft) < 1e-6) return; // same point — wait for a distinct second one
      const g = scene.addGuide(guideDraft, at);
      const firstPick = guideDraftPick;
      disarmTool(); // clears the draft (and, via resetTransient, the selection)
      if (g) {
        // CAD-style auto-constraints: a defining point placed on an existing point
        // sticks to it with a coincident (skipped if the sketch can't take it).
        if (firstPick) {
          tryAddConstraint(scene, "coincident", { kind: "guidePoint", guideId: g.id, which: "a" }, firstPick);
        }
        if (pick) {
          tryAddConstraint(scene, "coincident", { kind: "guidePoint", guideId: g.id, which: "b" }, pick);
        }
        selection = { kind: "guide", id: g.id };
        markDirty();
      }
      return;
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
      handleConstraintClick(p);
      return; // manages its own dirty-marking and disarm
  }
  markDirty();
  if (placed) disarmTool();
}

// --- sketch-constraint tools -------------------------------------------------
/** The point reference a constraint click would pick: a joint, then a body control
 *  vertex, then a guideline defining point. `excludeGuide` leaves one guideline out
 *  (so a dragged guide point never picks itself). */
function constraintPointRefAt(p: Vec2, excludeGuide?: number): MeasureRef | null {
  const j = scene.jointAt(p, pickRadius());
  if (j) return { kind: "joint", jointId: j.id };
  const v = bodyVertexRefAt(p);
  if (v) return v;
  const gp = scene.guidePointAt(p, pickRadius(), excludeGuide);
  if (gp) return { kind: "guidePoint", guideId: gp.guide.id, which: gp.which };
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
 *  edge, then a guideline (its infinite line). */
function constraintLineRefAt(p: Vec2): MeasureRef | null {
  const s = scene.sliderAt(p, pickRadius());
  if (s) return { kind: "rail", sliderId: s.id };
  const edge = bodyEdgeRefAt(p);
  if (edge) return edge;
  const pax = patternAxisRefAt(p);
  if (pax) return pax;
  const gl = scene.guideAt(p, pickRadius());
  if (gl) return { kind: "guideLine", guideId: gl.id };
  return null;
}

/**
 * Where a guide-point click (or the placement preview) lands: exactly on a picked point
 * element (joint / body corner / another guide's point — returned as `pick` for the
 * auto-coincident), projected onto a picked slider rail / body edge, else grid/guide-
 * snapped like any placement.
 */
function guidePlacementAt(p: Vec2): { at: Vec2; pick: MeasureRef | null } {
  const pick = constraintPointRefAt(p);
  if (pick) {
    const res = scene.resolveMeasureRef(pick);
    if (res?.kind === "point") return { at: res.p, pick };
  }
  const s = scene.sliderAt(p, pickRadius());
  const lineRes = s ? scene.resolveMeasureRef({ kind: "rail", sliderId: s.id }) : null;
  let seg = lineRes?.kind === "line" ? lineRes : null;
  if (!seg) {
    // Body control edge under the cursor (outer or hole — same pick as the line refs).
    const edge = bodyEdgeRefAt(p);
    const res = edge ? scene.resolveMeasureRef(edge) : null;
    if (res?.kind === "line") seg = res;
  }
  if (seg) {
    const ab = sub(seg.b, seg.a);
    const t = Math.max(0, Math.min(1, dot(sub(p, seg.a), ab) / Math.max(lenSq(ab), 1e-9)));
    return { at: add(seg.a, scale(ab, t)), pick: null };
  }
  return { at: snap(p), pick: null };
}

/** The reference the armed constraint tool would pick at `p` (for hover + clicks). */
function constraintRefAt(p: Vec2): MeasureRef | null {
  const kind = tool as SketchConstraintKind;
  if (kind === "parallel" || kind === "perpendicular" || kind === "equal") {
    return constraintLineRefAt(p);
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
 * Constraint tool click. Line-pair and point-pair kinds take two picks; horizontal /
 * vertical on a line commits on the first. The commit adds the constraint and runs a
 * sketch solve — geometry moves to satisfy it, or (unsatisfiable) the constraint is
 * removed again and the conflicting items flash red (reject semantics).
 */
function handleConstraintClick(p: Vec2): void {
  const kind = tool as SketchConstraintKind;
  const ref = constraintRefAt(p);
  if (!ref) return; // empty space — keep waiting for a reference
  const isLine = ref.kind === "rail" || ref.kind === "edge" || ref.kind === "guideLine" || ref.kind === "patternAxis";
  if (constraintPicks.length === 0) {
    if ((kind === "horizontal" || kind === "vertical") && isLine) {
      commitConstraint(kind, ref);
      return;
    }
    constraintPicks = [ref];
    return;
  }
  if (sameMeasureRef(constraintPicks[0], ref)) return;
  commitConstraint(kind, constraintPicks[0], ref);
}

/** Add + solve a sketch constraint; on an unsatisfiable solve it's removed again and flashes.
 *  Every end on component-instance geometry makes it a pose constraint (rigid parts move
 *  instead of shape — pose.ts routes it). */
function commitConstraint(kind: SketchConstraintKind, refA: MeasureRef, refB?: MeasureRef): void {
  const { constraint, breaks } = placeConstraint(scene, kind, refA, refB);
  disarmTool(); // clears the picks (and, via resetTransient, the selection)
  if (!constraint) {
    if (breaks.length) flashSketchItems(breaks);
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
        keys.push(...anchorVarsForGuide(leftDrag.id));
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
  dimEditInput.classList.add("hidden");
  dimEditInput.blur();
}

function commitDimEditor(): void {
  if (patternEdit) {
    commitPatternEditor();
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
    return;
  }
  const breaks = applyDimensionValue(scene, id, target);
  if (breaks.length) flashSketchItems(breaks);
  else markDirty();
}

dimEditInput.addEventListener("keydown", (e) => {
  e.stopPropagation(); // keep canvas shortcuts (tools, Delete…) out of the text field
  if (e.key === "Enter") commitDimEditor();
  else if (e.key === "Escape") closeDimEditor();
});
dimEditInput.addEventListener("blur", () => {
  if (dimEditId !== null || dimEditTemp !== null || patternEdit) commitDimEditor();
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
    if (gp) return { kind: "guidePoint", guideId: gp.guide.id, which: gp.which };
  }
  const s = scene.sliderAt(p, pickRadius());
  if (s) return { kind: "rail", sliderId: s.id };
  const edge = bodyEdgeRefAt(p);
  if (edge) return edge;
  if (mode === "draw") {
    const pax = patternAxisRefAt(p);
    if (pax) return pax;
    const gl = scene.guideAt(p, pickRadius());
    if (gl) return { kind: "guideLine", guideId: gl.id };
  }
  const body = scene.bodyAt(p);
  if (body) {
    const snapped = snap(p);
    const at = scene.pointInBody(body, snapped) ? snapped : p;
    return { kind: "bodyPoint", bodyId: body.id, local: rotate(sub(at, body.pos), -body.angle) };
  }
  return null;
}

/** Measure tool click: two reference picks, then a third click places the value label. */
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
    const ref: TempRef | null = measureRefAt(p) ?? ghostRefAt(p);
    if (!ref) return; // empty space — keep waiting for a reference
    if (measurePicks.length === 1 && sameTempRef(measurePicks[0], ref)) return;
    measurePicks.push(ref);
    return;
  }
  const [pa, pb] = measurePicks;
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

/** The current mode's measurement whose value label sits under `p`, or null (topmost first). */
function measurementLabelAt(p: Vec2): Measurement | null {
  if (!measureVisible) return null; // hidden measurements aren't clickable
  const mm = mode === "sim" ? "sim" : "draw";
  for (let i = scene.measurements.length - 1; i >= 0; i--) {
    const m = scene.measurements[i];
    if (m.mode !== mm) continue;
    const lp = scene.measurementLabelPos(m);
    if (lp && dist(lp, p) <= LABEL_PICK_RADIUS / view.scale) return m;
  }
  return null;
}

/** The sketch constraint whose badge sits under `p` (using last frame's badge layout), or null. */
function sketchGlyphAt(p: Vec2): number | null {
  const r = GLYPH_PICK_RADIUS / view.scale;
  for (let i = sketchGlyphCache.length - 1; i >= 0; i--) {
    for (const b of sketchGlyphCache[i].badges) {
      if (dist(b, p) <= r) return sketchGlyphCache[i].id;
    }
  }
  return null;
}

/** Normal/select mode: pick a measurement label (topmost overlay), then a joint, a slider rail, a body. */
function handleSelectClick(p: Vec2): void {
  multiSel = null; // a plain click rebuilds the selection from what's under the cursor
  const tl = tempDimLabelAt(p);
  if (tl) {
    selection = { kind: "tempDim", id: tl.id };
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
  if (j) {
    // A joint owned by a component instance selects the whole instance (its material is
    // atomic); a free joint locked to a group selects the whole group.
    const inst = scene.instanceOfJoint(j.id);
    if (inst) {
      selectInstance(inst);
      return;
    }
    const g = j.bodyId === null ? scene.groupOfJoint(j.id) : undefined;
    if (g) {
      setMulti(new Set(g.bodyIds), new Set(g.jointIds));
      return;
    }
    selection = { kind: "joint", id: j.id };
    return;
  }
  // A guideline's defining points are small point targets — they beat the line picks.
  const gp = scene.guidePointAt(p, pickRadius());
  if (gp) {
    selection = { kind: "guide", id: gp.guide.id };
    return;
  }
  const s = scene.sliderAt(p, pickRadius());
  if (s) {
    selection = { kind: "rail", id: s.id };
    return;
  }
  // Guidelines are thin precise targets, so (like rails) they win over body areas.
  const gl = scene.guideAt(p, pickRadius());
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
  const pax = scene.holeAt(p) ? null : patternAxisRefAt(p);
  if (pax) {
    selection = { kind: "pattern", id: pax.patternId };
    return;
  }
  const body = scene.bodyAt(p);
  if (body) {
    // Instance material is selection-atomic: clicking any member selects the instance.
    const inst = scene.instanceOfBody(body.id);
    if (inst) {
      selectInstance(inst);
      return;
    }
    const g = scene.groupOf(body.id);
    if (g) {
      // A grouped body is selection-atomic: clicking any member selects the whole group.
      setMulti(new Set(g.bodyIds), new Set(g.jointIds));
      return;
    }
    selection = { kind: "body", id: body.id };
    return;
  }
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
    const pj = scene.patternOfJoint(id);
    jset.add(pj?.role === "member" && pj.pattern.seed.kind === "joint" ? pj.pattern.seed.jointId : id);
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
    const pj = scene.patternOfJoint(j.id);
    const id = pj?.role === "member" && pj.pattern.seed.kind === "joint" ? pj.pattern.seed.jointId : j.id;
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
  const worlds = d.verts.map((v) => outlineControlWorld(body, v.hole)[v.index]).filter((p): p is Vec2 => !!p);
  const carried = new Set(
    scene.joints
      .filter((j) => j.bodyId === body.id && worlds.some((w) => dist(w, scene.jointWorld(j)) < VERTEX_LINK_EPS))
      .map((j) => j.id)
  );
  for (const v of d.verts) scene.moveBodyVertex(d.bodyId, v.index, delta, v.hole);
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
    if (p.seed.kind === "hole") {
      const seed = p.seed.hole;
      const idx = featureSel.verts.filter((v) => v.hole === seed).map((v) => v.index);
      if (idx.length === 0) continue;
      for (const m of p.members) {
        const verts = outlineControlWorld(body, m);
        for (const i of idx) if (verts[i]) vertices.push(verts[i]);
      }
    } else if (featureSel.joints.includes(p.seed.jointId)) joints.push(...p.members);
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

/**
 * Body tool click. The first click decides the mode: on an existing joint → build a
 * body from joints; on empty space → freehand polygon. While building from joints,
 * each click adds a joint; once expanding, a click finalizes with the current margin.
 */
function handleBodyClick(p: Vec2): void {
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
    addBodyPoint(p); // already drawing a freehand polygon
    return;
  }
  // Fresh start: a joint (or a slider rail) begins joint-build mode; empty space begins a
  // freehand polygon.
  const j = scene.jointAt(p, pickRadius());
  if (j) jointDraftIds = [j.id];
  else if (!addSliderRiderToDraft(p)) addBodyPoint(p);
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

function addBodyPoint(p: Vec2): void {
  if (draftBody.length >= 3 && dist(p, draftBody[0]) < CLOSE_RADIUS / view.scale) {
    finishBody();
    return;
  }
  // A click on an existing point (a joint or another body's corner) lands the vertex
  // exactly there and records the pick — finishing the draft turns it into a coincident
  // auto-constraint. Otherwise freehand vertices land on the grid when snap is on.
  const pick = constraintPointRefAt(p);
  const picked = pick ? scene.resolveMeasureRef(pick) : null;
  const at = picked && picked.kind === "point" ? picked.p : snap(p);
  // Ignore near-duplicate points (also de-dupes the 2nd click of a double-click).
  const last = draftBody[draftBody.length - 1];
  if (last && dist(at, last) < 4 / view.scale) return;
  draftBody.push(at);
  draftBodySnaps.push(picked && picked.kind === "point" ? pick : null);
}

function finishBody(): void {
  if (draftBody.length >= 3) {
    const body = scene.addBody(draftBody);
    body.color = defaultBodyColor;
    // Auto-constraints: clicked-on existing points become coincident; near-horizontal /
    // near-vertical edges get H/V (each solved in as it's added; unsatisfiable ones are
    // skipped). Control-vertex order matches the draft order, so indices line up.
    for (let i = 0; i < draftBodySnaps.length; i++) {
      const ref = draftBodySnaps[i];
      if (ref) {
        tryAddConstraint(scene, "coincident", { kind: "vertex", bodyId: body.id, index: i }, ref);
      }
    }
    autoConstrainBody(scene, body.id);
    draftBody = [];
    draftBodySnaps = [];
    markDirty();
    disarmTool();
    return;
  }
  draftBody = [];
  draftBodySnaps = [];
}

/**
 * Hole tool: the first click picks the body to cut (topmost under the cursor) and
 * starts a freehand cut-out polygon; later clicks add vertices, each kept inside that
 * body. Clicking the first vertex (or Enter) closes the hole.
 */
function handleHoleClick(p: Vec2): void {
  if (holeDraftBodyId === null) {
    const body = scene.bodyAt(p);
    if (!body) return; // a hole needs a body — keep the tool armed
    if (scene.instanceOfBody(body.id)) {
      notify("This body belongs to a component instance — edit the definition to cut a hole in it.");
      disarmTool();
      return;
    }
    holeDraftBodyId = body.id;
  }
  if (holeDraft.length >= 3 && dist(p, holeDraft[0]) < CLOSE_RADIUS / view.scale) {
    finishHole();
    return;
  }
  const body = scene.getBody(holeDraftBodyId);
  if (!body) { disarmTool(); return; }
  // Snap to the grid — unless snapping would land outside the body being cut, in which
  // case the exact click point is used; a click outside the body is ignored entirely.
  let at = snap(p);
  if (!scene.pointInBody(body, at)) at = p;
  if (!scene.pointInBody(body, at)) return;
  // Ignore near-duplicate points (also de-dupes the 2nd click of a double-click).
  const last = holeDraft[holeDraft.length - 1];
  if (last && dist(at, last) < 4 / view.scale) return;
  holeDraft.push(at);
}

/**
 * Hole tool press with nothing drawn yet: pick the body (topmost under the cursor; an
 * instance body is refused) and remember the snapped centre. A drag from here sizes a
 * round hole (mousemove); a plain release adds the polygon's first vertex instead.
 * The centre object-snaps (a hole centred on a joint or a corner) with the usual
 * containment fallback: a snap that would leave the body uses the exact press point.
 */
function startHolePress(p: Vec2, screen: Vec2): void {
  const body = scene.bodyAt(p);
  if (!body) return; // a hole needs a body — keep the tool armed
  if (scene.instanceOfBody(body.id)) {
    notify("This body belongs to a component instance — edit the definition to cut a hole in it.");
    disarmTool();
    return;
  }
  let centre = placeSnap(p);
  if (!scene.pointInBody(body, centre)) centre = p;
  holePress = { bodyId: body.id, centre, screen, maxR: scene.bodyInscribedRadius(body, centre) };
  holeCircle = null;
}

/**
 * Release of a hole-tool press: a dragged circle becomes a parametric disk hole (one
 * offset-mode control point + radius — resizable by its rim handle, movable by its
 * centre node, dimensionable by diameter); an undragged press starts the polygon.
 */
function finishHolePress(): void {
  const press = holePress;
  const circle = holeCircle;
  holePress = null;
  holeCircle = null;
  if (!press) return;
  const body = scene.getBody(press.bodyId);
  if (!body) return;
  if (circle) {
    // Too small to be a hole (a twitch): drop it, keep the tool armed.
    if (circle.r < HOLE_DRAG_PX / view.scale) return;
    if (scene.addBodyHole(body.id, { control: [circle.c], radius: circle.r, round: "offset" }) !== null) {
      markDirty();
      disarmTool();
      selection = { kind: "body", id: body.id }; // show the new hole's handles right away
    }
    return;
  }
  holeDraftBodyId = body.id;
  holeDraft.push(press.centre);
}

/** Close the hole draft: cut it into its body as an editable radius-0 hole outline. */
function finishHole(): void {
  const bodyId = holeDraftBodyId;
  if (bodyId !== null && holeDraft.length >= 3 && scene.addBodyHole(bodyId, holeDraft) !== null) {
    markDirty();
    disarmTool();
    selection = { kind: "body", id: bodyId }; // show the new hole's handles right away
    return;
  }
  holeDraft = [];
  holeDraftBodyId = null;
}

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
  if (!patternSeed) {
    return `${circular ? "Circular" : "Linear"} pattern: click a hole (inside the cut-out) or a joint on a body to repeat it. The count / spacing labels are edited on the canvas afterwards (double-click a label; drag the end handle or centre to re-aim).`;
  }
  const what = patternSeed.kind === "hole" ? "hole" : "joint";
  if (circular) {
    return `Circular pattern of the ${what}: click the centre of rotation (snaps to joints, hole centres, corners and the grid).`;
  }
  if (patternDraft !== null) {
    return "Row created — click where the first instance of a second direction should go to make a grid, or press Enter / Esc to keep a single row. Type the count in the label, or double-click the ×count / spacing labels later.";
  }
  return `Linear pattern of the ${what}: click where the next instance should go (snaps to the grid / objects).`;
}

/** Arm a pattern tool on an attached joint (refused: free joints, instance-owned, already patterned). */
function seedPatternJoint(jointId: number): void {
  const j = scene.getJoint(jointId);
  if (!j) return;
  if (j.bodyId === null) {
    notify("A pattern repeats a joint across its body — free joints have no body to pattern on.");
    return;
  }
  if (scene.instanceOfBody(j.bodyId)) {
    notify("This joint belongs to a component instance — edit the definition to pattern it.");
    disarmTool();
    return;
  }
  if (scene.patternOfJoint(jointId)) {
    notify("This joint is already part of a pattern — edit that pattern's labels, or delete it first.");
    return;
  }
  patternSeed = { kind: "joint", jointId };
  updateHint();
}

/**
 * Pattern-tool click. No seed yet: pick one — a joint under the cursor wins (as
 * everywhere), else the hole whose cut-out contains the point. With a seed: create the
 * pattern at the clicked layout point (linear: where the next instance goes; circular:
 * the centre) and open its count label for typing. A linear pattern stays armed for an
 * optional second direction (a grid); Enter / Esc keep the single row.
 */
function handlePatternClick(p: Vec2): void {
  if (!patternSeed) {
    const j = scene.jointAt(p, pickRadius());
    if (j) {
      seedPatternJoint(j.id);
      return;
    }
    const hit = scene.holeAt(p);
    if (hit) {
      if (scene.instanceOfBody(hit.body.id)) {
        notify("This body belongs to a component instance — edit the definition to pattern its holes.");
        disarmTool();
        return;
      }
      if (scene.patternOfHole(hit.body.id, hit.hole)) {
        notify("This hole is already part of a pattern — edit that pattern's labels, or delete it first.");
        return;
      }
      patternSeed = { kind: "hole", bodyId: hit.body.id, hole: hit.hole };
      updateHint();
    } else if (scene.bodyAt(p)) {
      notify("Click inside a hole (the cut-out itself) or on a joint to pattern it.", "info");
    }
    return; // empty space: keep the tool armed
  }
  const target = patternTarget(p);
  if (!target) return;
  const seed = patternSeed;
  if (tool === "patternCircular") {
    const created = scene.createCircularPattern(seed, target, PATTERN_DEFAULT_CIRCULAR_COUNT);
    if (!created) return;
    markDirty();
    disarmTool();
    selection = { kind: "pattern", id: created.id };
    openPatternEditorSoon(created.id, "count", 0);
    return;
  }
  if (patternDraft === null) {
    const created = scene.createLinearPattern(seed, target, PATTERN_DEFAULT_LINEAR_COUNT);
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
  if (!patternSeed) return null;
  const anchor = scene.patternSeedAnchor(patternSeed);
  if (!anchor) return null;
  const at = placeSnap(p);
  return dist(at, anchor) < 1e-6 ? null : at;
}

/** Pattern-tool overlay for the renderer: hover candidate, seed, layout point, instances. */
function patternPreviewView(): RenderInput["patternPreview"] {
  if (mode !== "draw" || !isPatternTool(tool)) return null;
  const kind = tool === "patternCircular" ? "circular" : "linear";
  if (!patternSeed) {
    // Hover feedback while picking: the hole under the cursor (joints highlight anyway).
    const hit = cursor && hoverJoint === null ? scene.holeAt(cursor) : undefined;
    if (!hit) return null;
    return { kind, anchor: null, seedLoop: scene.bodyHolesWorld(hit.body)[hit.hole], target: null, instances: [] };
  }
  const anchor = scene.patternSeedAnchor(patternSeed);
  if (!anchor) return null;
  const seedLoop =
    patternSeed.kind === "hole" ? scene.bodyHolesWorld(scene.getBody(patternSeed.bodyId)!)[patternSeed.hole] ?? null : null;
  const target = cursor ? patternTarget(cursor) : null;
  const pv = !target
    ? null
    : kind === "circular"
    ? scene.patternPreview(patternSeed, { kind: "circular", centre: target, count: PATTERN_DEFAULT_CIRCULAR_COUNT })
    : scene.patternPreview(patternSeed, {
        kind: "linear",
        target,
        count: PATTERN_DEFAULT_LINEAR_COUNT,
        ...(patternDraft !== null ? { axis: patternDraft } : {}),
      });
  return { kind, anchor, seedLoop, target, instances: pv?.instances ?? [] };
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

// --- pointer events ------------------------------------------------------
canvas.addEventListener("mousedown", (e) => {
  const world = eventWorld(e);
  cursor = world;

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
          const pj = scene.patternOfJoint(selection.id);
          const dragId = pj?.role === "member" && pj.pattern.seed.kind === "joint" ? pj.pattern.seed.jointId : selection.id;
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
              grabOffset: sub(world, g[gp.which]),
              moved: false,
            };
          } else {
            leftDrag = { kind: "guide", id: g.id, grabOffset: sub(world, g.a), moved: false };
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
    } else if (tool === "hole" && holeDraft.length === 0) {
      // Hole tool, nothing drawn yet: the press may become a round-hole drag (centre →
      // rim), so the polygon's first vertex waits for the release (see mouseup).
      startHolePress(world, eventScreen(e));
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
    // select it to delete — without disturbing the mechanism underneath.
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

  if (holePress) {
    // Round-hole gesture: once the pointer has clearly moved, the press is a circle drag
    // — the radius runs from the centre to the (grid-snapped) cursor, clamped so the
    // disk stays inside the body's material.
    if (holeCircle || dist(eventScreen(e), holePress.screen) > HOLE_DRAG_PX) {
      const r = Math.min(holePress.maxR, dist(holePress.centre, snap(world)));
      holeCircle = { c: holePress.centre, r };
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
    // A measurement label follows the cursor exactly (no grid snap — it's an annotation,
    // and a new placement re-derives h/v/direct for a point–point measurement).
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
        } else if (e.ctrlKey || e.metaKey) {
          // Ctrl: every corner of this outline follows the handle — the default is set
          // and per-corner overrides dropped, so the corners read as uniform from here on
          // (a radius dimension on any of them then drives them all).
          scene.setOutlineRadiusUniform(bodyId, r, hole);
          demoteSizeDims(bodyId, hole, null);
        } else {
          scene.setBodyCornerRadius(bodyId, index, r, hole);
          demoteSizeDims(bodyId, hole, (i) => i === index); // the handle now sets this corner
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
  }
  // Rotate tool: a grab cursor over a node of the selected body or any body.
  if (mode === "draw" && tool === "rotate") {
    const rotatable = selectedBodyNodeAt(world) !== null || scene.bodyAt(world) !== undefined;
    canvas.style.cursor = rotatable ? "grab" : "crosshair";
  }
  if (mode === "sim") {
    if (driver) driver.target = world;
    else if (tool === "measure") canvas.style.cursor = "crosshair";
    else if (measurementLabelAt(world)) canvas.style.cursor = "move";
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
  if (e.button === 0 && holePress) finishHolePress();
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
      } else if (finished.kind !== "measureLabel" && finished.kind !== "fillet" && finished.kind !== "patternHandle") {
        // Settle: one symmetric sketch solve at rest, repairing anything the anchored
        // live solves couldn't satisfy without moving the dragged geometry. (A fillet
        // drag needs none: a radius change moves no control vertices or joints.)
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
  if (mode === "draw" && tool === "body" && jointDraftIds.length === 0) {
    finishBody();
    return;
  }
  // A hole draft closes on double-click too (the second click was de-duped as a vertex).
  if (mode === "draw" && tool === "hole") {
    finishHole();
    return;
  }
  // Select mode: double-click a draw-mode dimension label to edit its value inline
  // (typing a number makes it a driving dimension; clearing it makes it driven again).
  if (mode === "draw" && tool === null) {
    const pl = patternLabelAt(eventWorld(e));
    if (pl && pl.field !== "rotate") {
      leftDrag = null;
      openPatternEditor(pl.id, pl.field, pl.axis);
      return;
    }
    const tl = tempDimLabelAt(eventWorld(e));
    if (tl) {
      leftDrag = null;
      openTempDimEditor(tl); // one-shot move of the live side to the typed value
      return;
    }
    const ml = measurementLabelAt(eventWorld(e));
    if (ml) {
      leftDrag = null; // the double-click's mousedowns started a label drag — cancel it
      openDimEditor(ml);
      return;
    }
    // Double-click a component instance to open its definition for editing — through
    // this instance, so the context ghost can place the surroundings; with Ctrl the
    // whole enclosing assembly shows faded straight away.
    const b = scene.bodyAt(eventWorld(e));
    const inst = b ? scene.instanceOfBody(b.id) : undefined;
    if (inst) {
      leftDrag = null; // cancel the drag the double-click's mousedowns started
      enterComponent(inst.defId, inst.id, e.ctrlKey || e.metaKey);
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

/** Draw-tool shortcuts: mostly the first letter of the tool's name (L = guideLine —
 *  G is Ground; the actuator moved to A when L was given to guidelines). */
const TOOL_KEYS: Record<string, Tool> = {
  b: "body",
  u: "hole", // cUt-out (H is the horizontal constraint)
  i: "patternLinear", // repeat a hole / joint along one or two directions (Instances)
  q: "patternCircular", // ...or around a centre
  x: "split", // cut a body in two along a drawn path
  j: "joint",
  w: "weld",
  c: "connect",
  g: "ground",
  s: "slider", // S is the slider itself (the prismatic carriage that rides a rail)
  k: "rail", // the tracK the sliders ride along (S belongs to the slider)
  r: "rotate",
  l: "guide",
  a: "linearActuator",
  m: "motor",
  d: "measure",
  o: "coincident",
  h: "horizontal",
  v: "vertical",
  p: "parallel",
  t: "perpendicular",
  e: "equal",
};

window.addEventListener("keydown", (e) => {
  // Ctrl/Cmd+S saves (Shift: save as…), Ctrl/Cmd+O opens — from anywhere, fields included,
  // and always intercepted so the browser doesn't offer to save the web page itself.
  if ((e.ctrlKey || e.metaKey) && !e.altKey && e.key.toLowerCase() === "s") {
    e.preventDefault();
    void saveToFile(e.shiftKey);
    return;
  }
  if ((e.ctrlKey || e.metaKey) && !e.altKey && !e.shiftKey && e.key.toLowerCase() === "o") {
    e.preventDefault();
    void openFile();
    return;
  }
  // F1 opens the manual at its table of contents (from anywhere, fields included).
  if (e.key === "F1" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    help.open("toc");
    return;
  }
  // Keys typed into a toolbar field (or the inline dimension editor) belong to that
  // field — not to canvas shortcuts like Delete or the tool letters.
  const t = e.target;
  if (
    t instanceof HTMLInputElement ||
    t instanceof HTMLSelectElement ||
    t instanceof HTMLTextAreaElement
  ) {
    return;
  }
  // ? toggles the help drawer; while it is open, clicking a control shows its topic.
  if (e.key === "?" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    help.toggle();
    return;
  }
  // Space toggles the actuator animation (sim mode only).
  if (e.code === "Space" && mode === "sim" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    setAnimating(!animating);
    return;
  }
  // Tab toggles between draw and simulate mode (kept away from the browser's focus cycle).
  if (e.key === "Tab" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    setMode(mode === "draw" ? "sim" : "draw");
    return;
  }
  // F fits the mechanism to the screen (both modes).
  if (e.key.toLowerCase() === "f" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    fitView();
    return;
  }
  // Shift+R opens / closes the view-rotation dial (both modes; plain R is the Rotate tool).
  if (e.key.toLowerCase() === "r" && e.shiftKey && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    setViewRotateOpen(viewRotate === null);
    return;
  }
  if (viewRotate && e.key === "0" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    setViewAngle(0);
    return;
  }
  // Undo / redo: Ctrl/Cmd+Z, Ctrl/Cmd+Shift+Z or Ctrl/Cmd+Y.
  const mod = e.ctrlKey || e.metaKey;
  if (mod && e.key.toLowerCase() === "z") {
    e.preventDefault();
    if (e.shiftKey) redo();
    else undo();
    return;
  }
  if (mod && e.key.toLowerCase() === "y") {
    e.preventDefault();
    redo();
    return;
  }
  // Copy / paste the selection (draw mode). Paste lands at the cursor.
  if (mod && e.key.toLowerCase() === "c" && mode === "draw" && (selection?.kind === "body" || multiSel)) {
    e.preventDefault();
    copySelection();
    return;
  }
  if (mod && e.key.toLowerCase() === "v" && mode === "draw" && clipboard) {
    e.preventDefault();
    pasteAt(cursor);
    return;
  }
  // Ctrl/Cmd+G toggles grouping: 2+ selected bodies group (merging any groups touched)
  // unless the selection already is exactly one group, which dissolves — as does a
  // single selected grouped body. Plain G is the Ground tool only (see TOOL_KEYS).
  if (mod && e.key.toLowerCase() === "g" && mode === "draw") {
    e.preventDefault();
    toggleGroupSelection();
    return;
  }
  if (e.key === "Escape") {
    if (!exportPanel.classList.contains("hidden")) {
      setExportPanelVisible(false);
      return;
    }
    if (viewRotate) {
      setViewRotateOpen(false);
      return;
    }
    // Mid-drag with an armed alignment candidate: Esc only drops the candidate — the drag
    // goes on and its release creates nothing. Parking the hover timer at +∞ keeps the
    // same element from re-arming while the cursor still rests on it.
    if (leftDrag && "align" in leftDrag && leftDrag.align?.cand) {
      leftDrag.align.cand = null;
      leftDrag.align.match = null;
      if (leftDrag.align.hover) leftDrag.align.hover.since = Infinity;
      return;
    }
    // Abort the current placement / drag and return to the mode's normal state
    // (in sim this also disarms the measure tool). With nothing armed or selected
    // while editing a component definition, Esc steps back out one level.
    const idle =
      tool === null && selection === null && multiSel === null && pendingInsert === null &&
      draftBody.length === 0 && jointDraftIds.length === 0 && !leftDrag && !rotateDrag;
    disarmTool();
    if (idle && mode === "draw" && editPath.length > 0) exitComponent(1);
    return;
  }
  if (e.key === "Enter" && mode === "draw" && tool === "body") {
    finishBody();
    return;
  }
  if (e.key === "Enter" && mode === "draw" && tool === "hole") {
    finishHole();
    return;
  }
  if (e.key === "Enter" && mode === "draw" && isPatternTool(tool)) {
    finishPatternTool();
    return;
  }
  // A selected measurement is deletable in either mode (sim keeps its own set).
  if ((e.key === "Delete" || e.key === "Backspace") && (selection?.kind === "measure" || selection?.kind === "tempDim")) {
    deleteSelection();
    return;
  }
  if (
    (e.key === "Delete" || e.key === "Backspace") &&
    mode === "draw" &&
    tool === null &&
    (selection || multiSel)
  ) {
    deleteSelection();
    return;
  }
  // PageDown / PageUp reorder the selection in the z-order (send to back / bring to front).
  if (
    (e.key === "PageDown" || e.key === "PageUp") &&
    mode === "draw" &&
    (selection?.kind === "body" || multiSel)
  ) {
    reorderSelection(e.key === "PageDown" ? "back" : "front");
    e.preventDefault();
    return;
  }
  // [ and ] adjust the selected body's corner radius (round / un-round it).
  if ((e.key === "[" || e.key === "]") && mode === "draw" && tool === null && selection?.kind === "body") {
    const body = scene.getBody(selection.id);
    if (body) {
      const step = e.key === "]" ? RADIUS_STEP : -RADIUS_STEP;
      const disk = scene.diskOfRef({ kind: "vertex", bodyId: body.id, index: 0 });
      if (disk) {
        scene.setDiskRadius(body.id, disk.r + step); // from the *effective* radius
        demoteSizeDims(body.id, null, null); // a direct resize overrides a driving diameter
      } else {
        scene.setBodyRadius(body.id, body.radius + step);
        // The default moved: radius dimensions on corners without their own override
        // were just overridden directly (overridden corners didn't change).
        demoteSizeDims(body.id, null, (i) => typeof body.radii?.[i] !== "number");
      }
      markDirty();
    }
    e.preventDefault();
    return;
  }
  // N combines the multi-selected bodies into one (an action on the selection, like mirror).
  if (e.key.toLowerCase() === "n" && mode === "draw" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    combineSelection();
    e.preventDefault();
    return;
  }
  // Tool shortcuts (draw mode only; ignore browser/OS modifier combos).
  if (mode === "draw" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    const t = TOOL_KEYS[e.key.toLowerCase()];
    if (t) {
      setTool(t);
      e.preventDefault();
    }
  }
  // Measure is the one tool that also works in sim mode.
  if (mode === "sim" && !e.ctrlKey && !e.metaKey && !e.altKey && e.key.toLowerCase() === "d") {
    setTool("measure");
    e.preventDefault();
  }
});

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
  if (tool === "body") return jointDraftIds;
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

/** Guide tool: the first defining point placed, with the live cursor (line preview,
 *  landing where the click would — on elements, projections, or the grid). */
function guideDraftView(): { a: Vec2; cursor: Vec2 } | null {
  if (mode !== "draw" || tool !== "guide" || guideDraft === null || !cursor) return null;
  return { a: guideDraft, cursor: guidePlacementAt(cursor).at };
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
      const h: TempRef | null = rim || arc ? null : measureRefAt(cursor) ?? ghostRefAt(cursor);
      hover = rim
        ? { kind: "circle", c: rim.c, r: rim.r }
        : arc
          ? { kind: "arc", ...arc.arc }
          : h
            ? resolveTemp(h)
            : null;
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
    case "edge": return `e:${ref.bodyId}:${ref.index}${ref.hole !== undefined ? `:${ref.hole}` : ""}`;
    case "rail": return `r:${ref.sliderId}`;
    case "guidePoint": return `gp:${ref.guideId}:${ref.which}`;
    case "guideLine": return `gl:${ref.guideId}`;
    case "patternAxis": return `px:${ref.patternId}:${ref.axis}`;
    default: return "?";
  }
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
  if (res.kind === "point" && dist(res.p, p) <= r) return true;
  if (res.kind === "line" && distToSegment(p, res.a, res.b) <= r) return true;
  if (ref.kind === "vertex" || ref.kind === "edge") return scene.bodyAt(p)?.id === ref.bodyId;
  // A guideline is infinite: hovering anywhere along it (not just the defining
  // segment) reveals its constraints; so does hovering either defining point.
  if (ref.kind === "guideLine" || ref.kind === "guidePoint") {
    const g = scene.getGuide(ref.guideId);
    if (!g) return false;
    return distToLine(p, g.a, normalize(sub(g.b, g.a))) <= r || dist(g.a, p) <= r || dist(g.b, p) <= r;
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
    const allRefs = c.refB ? [c.refA, c.refB] : [c.refA];
    const badgeRefs = c.kind === "coincident" ? [c.refA] : allRefs;
    const badges: Vec2[] = [];
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
    const badgeHot = cursor !== null && badges.some((b) => dist(b, cursor!) <= GLYPH_PICK_RADIUS / view.scale);
    const hot = badgeHot || (cursor !== null && allRefs.some((ref) => refHovered(ref, cursor!)));
    // A pose constraint that can't currently hold (grounded partner, a def-edit reset,
    // an instance rotated against it) shows in the error style, like a violated dim.
    const violated = poseConstraintViolated(scene, c);
    out.push({ id: c.id, kind: c.kind, badges, faded: !hot && !violated, violated });
    if (badgeHot) hoveredBadge = out.length - 1;
  }
  // Hovering a badge reveals what it constrains: the elements light up and, when they
  // sit apart, a dotted line joins them. Only the topmost badge under the cursor (the
  // one a click would select) gets it, so stacked badges don't all fire at once.
  if (hoveredBadge >= 0) {
    const c = scene.sketch.find((k) => k.id === out[hoveredBadge].id)!;
    const refs = (c.refB ? [c.refA, c.refB] : [c.refA])
      .map((ref) => scene.resolveMeasureRef(ref))
      .filter((r): r is ResolvedMeasureRef => r !== null);
    const link = refs.length === 2 ? sketchLink(refs[0], refs[1]) : null;
    out[hoveredBadge].hover = { refs, link };
  }
  sketchGlyphCache = out;
  return out;
}

/** Closest point to `p` on a resolved reference (a guide extends without end). */
function closestOnRef(p: Vec2, r: ResolvedMeasureRef): Vec2 {
  if (r.kind === "point") return r.p;
  const ab = sub(r.b, r.a);
  const l2 = lenSq(ab);
  if (l2 < 1e-12) return r.a;
  let t = dot(sub(p, r.a), ab) / l2;
  if (!r.infinite) t = Math.max(0, Math.min(1, t));
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
      const inA = a.infinite || (t >= -1e-9 && t <= 1 + 1e-9);
      const inB = b.infinite || (u >= -1e-9 && u <= 1 + 1e-9);
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
function sketchDraftView(): { refs: ResolvedMeasureRef[]; hover: ResolvedMeasureRef | null } | null {
  if (tool === null || !CONSTRAINT_TOOLS.has(tool)) return null;
  const refs = constraintPicks
    .map((r) => scene.resolveMeasureRef(r))
    .filter((r): r is ResolvedMeasureRef => r !== null);
  let hover: ResolvedMeasureRef | null = null;
  if (cursor) {
    const h = constraintRefAt(cursor);
    hover = h ? scene.resolveMeasureRef(h) : null;
  }
  return { refs, hover };
}

/** Body-from-joints overlay: the picked-joint outline, plus the expanded preview when sizing. */
function bodyJointDraftView(): { outline: Vec2[]; preview: Vec2[] | null } | null {
  if (mode !== "draw" || tool !== "body" || jointDraftIds.length === 0) return null;
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
    const due =
      al.hover !== null && now - al.hover.since >= ALIGN_HOVER_MS && !(al.cand && sameMeasureRef(al.cand, al.hover.ref));
    if (due) updateDragAlign(leftDrag, al, now);
  }
  // Containment check (draw mode): flag joints a shape change stranded outside their
  // body. Refresh the hint when the count changes so the warning appears/clears itself.
  const prevOutside = containmentErrors.size;
  containmentErrors = mode === "draw" ? new Set(scene.jointsOutsideBody()) : new Set();
  if (containmentErrors.size !== prevOutside) updateHint();
  pruneFeatureSel(); // the feature selection follows the single body selection + live geometry
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
    // The hole tool's cut-out draft previews through the same dashed-polyline channel;
    // its round-hole drag previews as a dashed circle.
    draftCircle: mode === "draw" && tool === "hole" ? holeCircle : null,
    patternPreview: patternPreviewView(),
    patterns: patternViews(),
    draftBody:
      mode !== "draw" ? null
      : tool === "body" ? draftBody
      : tool === "hole" ? holeDraft
      : tool === "split" ? splitDraft
      : null,
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
    filletHandles: filletHandlesView(),
    railDraft: railDraftView(),
    guideDraft: guideDraftView(),
    bodyJointDraft: bodyJointDraftView(),
    driverJoint: driver?.jointId ?? null,
    rotatePivot: rotateDrag?.pivot ?? null,
    gridStep,
    gridVisible,
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
