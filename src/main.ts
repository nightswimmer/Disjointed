import "./style.css";
import {
  Scene,
  SceneData,
  SelectionClip,
  LinearActuatorConstraint,
  MotorConstraint,
  Measurement,
  MeasureInfo,
  MeasureRef,
  ResolvedMeasureRef,
  SketchConstraintKind,
  sameMeasureRef,
} from "./model";
import { solve, Driver, ConstraintBreak, SolveStats, SolveFreeze, solverConfig } from "./solver";
import {
  solveSketch, applyDrivingDimension, tryAddConstraint, autoConstrainBody, SketchBreak,
  anchorVarsForBody, anchorVarsForJoint, anchorVarsForGuide, anchorVarForGuidePoint, anchorVarForVertex,
} from "./sketch";
import { render, DARK_THEME, LIGHT_THEME, SketchGlyphView } from "./renderer";
import { Vec2, add, dist, sub, vec, dot, cross, lenSq, scale, rotate, normalize, roundedConvexBody, distToSegment, distToLine } from "./geometry";
import { View, MIN_SCALE, MAX_SCALE, screenToWorld, worldToScreen, zoomAt } from "./view";

type Mode = "draw" | "sim";
type Tool =
  | "body" | "joint" | "connect" | "ground" | "slider" | "rotate" | "guide"
  | "linearActuator" | "motor" | "measure"
  | SketchConstraintKind; // each sketch-constraint kind is its own one-shot tool
/** An existing element picked in normal/select mode. */
type Selection = { kind: "body" | "joint" | "slider" | "measure" | "sketch" | "guide"; id: number };

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
const gridSizeInput = document.getElementById("grid-size") as HTMLInputElement;
const gridSizePresets = document.getElementById("grid-size-presets") as HTMLSelectElement;
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

const scene = new Scene();

// --- theme (light/dark) --------------------------------------------------
// Chrome is themed via a `data-theme` attribute on <html> (CSS vars); the canvas reads the
// matching palette below. Preference persists across sessions (separate from scene autosave).
const THEME_KEY = "disjointed:theme";
let theme: "dark" | "light" =
  localStorage.getItem(THEME_KEY) === "light" ? "light" : "dark";
function applyTheme(): void {
  document.documentElement.dataset.theme = theme;
}
function toggleTheme(): void {
  theme = theme === "dark" ? "light" : "dark";
  localStorage.setItem(THEME_KEY, theme);
  applyTheme();
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
let jointDraftIds: number[] = []; // joints picked to build a body (body tool, joint start)
let jointDraftCreated: number[] = []; // joints made on slider rails during that draft (removed if aborted)
let jointDraftExpanding = false; // body-from-joints: sizing the outward margin
let cursor: Vec2 | null = null; // world coordinates
let hoverJoint: number | null = null;
let hoverBody: number | null = null; // body under the cursor in normal mode
let selectedJoint: number | null = null; // first pick for connect
let sliderDraftIds: number[] = []; // rail joints picked so far for the slider tool (0–2)
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
/** Last selection copied with Ctrl+C (one body, or a multi-selection / permanent group,
 *  with everything internal to it); pasted at the cursor with Ctrl+V. */
let clipboard: SelectionClip | null = null;
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
let measurePicks: MeasureRef[] = [];
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
    if (selection?.kind === "measure") selection = null;
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
const view: View = { scale: 1, tx: 0, ty: 0 };
/** Active right-button view pan. */
let pan: { lastScreen: Vec2 } | null = null;
/**
 * Active left-button drag of a selected element in draw/select mode. `grabOffset` is
 * the cursor-minus-anchor offset captured at grab time, so the dragged anchor can be
 * snapped to the grid in absolute terms. For a whole-body move the anchor is whichever
 * of the centroid / control vertices was closest to the grab point, stored as a fixed
 * `anchorOffset` from the centroid (a plain move only translates, so it stays constant).
 */
type LeftDrag =
  | { kind: "body"; id: number; anchorOffset: Vec2; grabOffset: Vec2; moved: boolean }
  | { kind: "joint"; id: number; grabOffset: Vec2; moved: boolean }
  | { kind: "vertex"; bodyId: number; index: number; grabOffset: Vec2; moved: boolean }
  | { kind: "measureLabel"; id: number; grabOffset: Vec2; moved: boolean }
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
      anchor: { bodyId: number; offset: Vec2 } | { jointId: number };
      grabOffset: Vec2;
      moved: boolean;
    }
  // Rigid (Shift) drag: the grabbed selection moves like in simulation — the solver drives
  // it each frame, grounds hold, and the rest of the scene is frozen (`freeze`).
  | { kind: "rigid"; driver: Driver; freeze: SolveFreeze; moved: boolean };
let leftDrag: LeftDrag | null = null;

/** Current world position of a drag's anchor (the point that snaps to the grid). */
function dragAnchorWorld(d: LeftDrag): Vec2 {
  if (d.kind === "vertex") return scene.bodyControlWorld(scene.getBody(d.bodyId)!)[d.index];
  if (d.kind === "body") return add(scene.getBody(d.id)!.pos, d.anchorOffset);
  if (d.kind === "measureLabel") {
    return scene.measurementLabelPos(scene.getMeasurement(d.id)!) ?? vec(0, 0);
  }
  if (d.kind === "multi") {
    return "jointId" in d.anchor
      ? scene.jointWorld(scene.getJoint(d.anchor.jointId)!)
      : add(scene.getBody(d.anchor.bodyId)!.pos, d.anchor.offset);
  }
  if (d.kind === "rigid") return d.driver.target; // solver-driven; no snap anchor
  if (d.kind === "guide") return scene.getGuide(d.id)!.a;
  if (d.kind === "guidePoint") return scene.getGuide(d.id)![d.which];
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

// --- multi-selection (Ctrl+click / box select) + permanent groups -----------
/**
 * Commit a multi-selection: expand permanent groups (selection-atomic), drop dead ids,
 * and collapse trivial results — a single ungrouped body / free joint becomes a normal
 * single selection, an empty set clears everything. Otherwise `multiSel` is set and the
 * single `selection` cleared (they're mutually exclusive).
 */
function setMulti(bodies: Set<number>, joints: Set<number>): void {
  for (const id of [...bodies]) {
    const g = scene.groupOf(id);
    if (g) for (const b of g.bodyIds) bodies.add(b);
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
  if (bodies.size === 1 && joints.size === 0) {
    multiSel = null;
    selection = { kind: "body", id: [...bodies][0] };
    return;
  }
  if (bodies.size === 0 && joints.size === 1) {
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
  const toggleBody = (id: number): void => {
    const g = scene.groupOf(id);
    const ids = g ? g.bodyIds : [id];
    const on = ids.some((x) => bodies.has(x));
    for (const x of ids) {
      if (on) bodies.delete(x);
      else bodies.add(x);
    }
  };
  const j = scene.jointAt(p, pickRadius());
  if (j) {
    if (j.bodyId === null) {
      if (joints.has(j.id)) joints.delete(j.id);
      else joints.add(j.id);
    } else {
      toggleBody(j.bodyId);
    }
  } else {
    const b = scene.bodyAt(p);
    if (!b) return false;
    toggleBody(b.id);
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
  type MultiAnchor = { bodyId: number; offset: Vec2 } | { jointId: number };
  let anchor: MultiAnchor | null = null;
  let anchorPos = grab;
  let bestD = Infinity;
  const consider = (c: Vec2, spec: MultiAnchor): void => {
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
  if (!anchor) return; // no live members — nothing to drag
  leftDrag = {
    kind: "multi",
    bodies: [...multiSel.bodies],
    joints: [...multiSel.joints],
    anchor,
    grabOffset: sub(grab, anchorPos),
    moved: false,
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
    for (const b of g ? g.bodyIds : [id]) bodies.add(b);
  };
  if (multiSel && multiHitAt(grab)) {
    multiSel.bodies.forEach((id) => bodies.add(id));
    multiSel.joints.forEach((id) => joints.add(id));
  } else if (selection?.kind === "body") {
    addBodyWithGroup(selection.id);
  } else if (selection?.kind === "joint") {
    const j = scene.getJoint(selection.id);
    if (!j) return false;
    if (j.bodyId !== null) addBodyWithGroup(j.bodyId);
    else joints.add(j.id);
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

/** With 2+ bodies multi-selected: make (or extend) a permanent group over them. */
function groupSelection(): void {
  if (mode !== "draw" || !multiSel || multiSel.bodies.size < 2) return;
  const g = scene.addGroup([...multiSel.bodies]);
  if (!g) return;
  multiSel = { bodies: new Set(g.bodyIds), joints: multiSel.joints };
  markDirty();
}

/** Dissolve every permanent group the current selection touches. */
function ungroupSelection(): void {
  if (mode !== "draw") return;
  const ids: number[] = [];
  if (multiSel) ids.push(...multiSel.bodies);
  if (selection?.kind === "body") ids.push(selection.id);
  if (ids.length && scene.ungroup(ids)) markDirty();
}

/**
 * Ctrl+G: group / ungroup toggle. A multi-selection of 2+ bodies becomes a permanent
 * group (merging any groups it touches) — unless it already is exactly one group,
 * which dissolves instead. With fewer bodies selected (a single grouped body counts,
 * since groups are selection-atomic) it ungroups; otherwise it's a no-op.
 */
function toggleGroupSelection(): void {
  if (mode !== "draw") return;
  if (multiSel && multiSel.bodies.size >= 2) {
    const first = scene.groupOf([...multiSel.bodies][0]);
    const isOneGroup =
      first !== undefined &&
      [...multiSel.bodies].every((id) => scene.groupOf(id)?.id === first.id);
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
  const include = (p: Vec2) => {
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
  view.tx = w / 2 - ((minX + maxX) / 2) * view.scale;
  view.ty = h / 2 - ((minY + maxY) / 2) * view.scale;
}

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
const HINTS: Record<Mode | Tool | "select", string> = {
  draw: "",
  sim: "Drag any joint, or part of a body, to drive the mechanism. Space to run / pause actuators.",
  select: "Click to select · drag to move · Shift+drag to move rigidly (sim-style: grounds hold, connections constrain, the rest stays put) · Ctrl+click or drag a box to select several bodies (they move together) · Ctrl+G groups them permanently / ungroups a group · drag a selected body's corner handles to reshape · double-click an edge to add a node / a node to remove it · double-click a dimension to set its value · [ and ] round corners · Delete to remove.",
  body: "Empty space: click vertices to draw a polygon. Joints: click joints to build a body, click a node again to finish, then move out to set thickness and click.",
  joint: "Click inside a body to attach a joint, or empty space to place a free joint.",
  connect: "Click a joint, then another joint to pin them — or a slider line to attach the joint to it.",
  ground: "Click a joint to lock its position (it can still rotate), or a body / group to fix it entirely; click again to unground.",
  slider: "Click two joints on the same body to create a slider rail.",
  guide: "Click two points to place an infinite construction guideline — clicks land on joints, body corners and edges (points get a coincident constraint). Drag the line to move it (angle kept), or drag one of its two points to re-aim it. With snap on, placements prefer guidelines over the grid.",
  rotate: "Drag a body to rotate it about its centroid, or drag a selected body's node to rotate about that node. A multi-selection or group rotates as one about its centre. Snaps to 45°.",
  linearActuator: "Click a slider rail to drop a self-driving rider — it travels back and forth when animation runs.",
  motor: "Click a joint to set the pivot, then another joint on the same body for the crank pin.",
  measure: "Click two references — a joint, body corner, body edge, slider rail, guideline, or a point on a body — then click where the value should sit.",
  coincident: "Click two points (joints, body corners, or guideline points) to make them share a position.",
  horizontal: "Click a body edge, slider rail or guideline — or two points — to make it horizontal.",
  vertical: "Click a body edge, slider rail or guideline — or two points — to make it vertical.",
  parallel: "Click two lines (body edges, slider rails or guidelines) to make them parallel.",
  perpendicular: "Click two lines (body edges, slider rails or guidelines) to make them perpendicular.",
  equal: "Click two lines (body edges or slider rails) to make their lengths equal.",
};

function updateHint(): void {
  if (tool === "measure") hintEl.textContent = HINTS.measure;
  else if (mode === "sim") hintEl.textContent = HINTS.sim;
  else hintEl.textContent = tool === null ? HINTS.select : HINTS[tool];
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
  scene.clear();
  resetTransient();
  markDirty();
});
document.getElementById("fit-btn")!.addEventListener("click", fitView);
document.getElementById("save-btn")!.addEventListener("click", saveToFile);
document.getElementById("load-btn")!.addEventListener("click", () => fileInput.click());
// Copy/paste are keyboard-only (Ctrl/Cmd+C / V); no toolbar buttons.
document.getElementById("mirror-h-btn")!.addEventListener("click", () => mirrorSelection("h"));
document.getElementById("mirror-v-btn")!.addEventListener("click", () => mirrorSelection("v"));

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
sketchVisBtn.addEventListener("click", () => setSketchVisible(!sketchVisible));
measureVisBtn.addEventListener("click", () => setMeasureVisible(!measureVisible));
const GRID_MIN = 1;
const GRID_MAX = 200;
/** Read the grid-size field, clamped to [GRID_MIN, GRID_MAX]; null while it's empty/invalid. */
function parseGridSize(): number | null {
  const n = Number(gridSizeInput.value);
  if (!Number.isFinite(n) || gridSizeInput.value.trim() === "") return null;
  return Math.min(GRID_MAX, Math.max(GRID_MIN, n));
}
// Live-update the grid while typing a valid value; normalize the field text on commit.
gridSizeInput.addEventListener("input", () => {
  const n = parseGridSize();
  if (n !== null) gridStep = n;
});
gridSizeInput.addEventListener("change", () => {
  gridStep = parseGridSize() ?? gridStep;
  gridSizeInput.value = String(gridStep);
});
// Picking a preset fills the number field; reset the select so the same preset re-fires.
gridSizePresets.addEventListener("change", () => {
  gridStep = Number(gridSizePresets.value) || gridStep;
  gridSizeInput.value = String(gridStep);
  gridSizePresets.value = "";
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
    timedSolve("settle", null, 40); // settle so pins/grounds/sliders are satisfied
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
  tool = next;
  resetTransient();
  selection = keepSel;
  multiSel = keepMulti;
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
  constraintPicks = [];
  // Discard any slider-rail joints made for an unfinished body-from-joints draft (a finished
  // build clears this list first, so its absorbed joints survive).
  for (const id of jointDraftCreated) scene.removeJoint(id);
  jointDraftCreated = [];
  jointDraftIds = [];
  jointDraftExpanding = false;
  selectedJoint = null;
  sliderDraftIds = [];
  guideDraft = null;
  guideDraftPick = null;
  motorPivotDraft = null;
  measurePicks = [];
  selection = null;
  multiSel = null;
  boxSelect = null;
  driver = null;
  rotateDrag = null;
}

// --- persistence (save / load / autosave) --------------------------------
const AUTOSAVE_KEY = "disjointed:autosave:v1";
let autosaveTimer: number | undefined;

/**
 * Canonical snapshot to persist: the drawn layout. In simulation we serialize
 * the pre-sim poses (savedPoses), so a simulated configuration is never saved.
 */
function canonicalData(): SceneData {
  const data = scene.serialize();
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
const history: string[] = []; // JSON snapshots; history[historyIndex] is the current state
let historyIndex = -1;

/** Record the current state as a history step (deduped) and drop the redo branch. */
function pushHistory(): void {
  const snap = JSON.stringify(canonicalData());
  if (historyIndex >= 0 && history[historyIndex] === snap) return; // nothing actually changed
  history.splice(historyIndex + 1); // discard any redo entries past the current point
  history.push(snap);
  if (history.length > HISTORY_LIMIT) history.shift();
  historyIndex = history.length - 1;
}

/** A scene mutation occurred: record an undo step and schedule an autosave. */
function markDirty(): void {
  pushHistory();
  scheduleAutosave();
}

/** Restore a history snapshot without recording a new step. */
function loadSnapshot(snap: string): void {
  scene.load(JSON.parse(snap) as SceneData);
  resetTransient(); // selection / drafts may reference ids that no longer exist
  scheduleAutosave();
}

// Undo / redo apply to the drawn layout only (draw mode), not a running simulation.
function undo(): void {
  if (mode !== "draw" || historyIndex <= 0) return;
  historyIndex--;
  loadSnapshot(history[historyIndex]);
}

function redo(): void {
  if (mode !== "draw" || historyIndex >= history.length - 1) return;
  historyIndex++;
  loadSnapshot(history[historyIndex]);
}

function saveToFile(): void {
  const json = JSON.stringify(canonicalData(), null, 2);
  const blob = new Blob([json], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const stamp = new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");
  const a = document.createElement("a");
  a.href = url;
  a.download = `mechanism-${stamp}.json`;
  a.click();
  URL.revokeObjectURL(url);
}

async function loadFromFile(file: File): Promise<void> {
  try {
    const data = JSON.parse(await file.text()) as SceneData;
    applyLoadedScene(data);
  } catch (err) {
    window.alert(`Could not load file: ${(err as Error).message}`);
  }
}

/** Replace the scene with loaded data, returning to a clean draw-mode state. */
function applyLoadedScene(data: SceneData): void {
  // Leave simulation first (restores the current scene's poses, clears savedPoses)
  // so it can't run against the bodies we're about to replace.
  if (mode === "sim") setMode("draw");
  savedPoses = null;
  scene.load(data); // validates; throws on bad data
  resetTransient();
  view.scale = 1;
  view.tx = 0;
  view.ty = 0;
  markDirty();
}

/** On startup, restore the last autosaved layout if present and valid. */
function restoreAutosave(): void {
  try {
    const raw = localStorage.getItem(AUTOSAVE_KEY);
    if (raw) scene.load(JSON.parse(raw) as SceneData);
  } catch {
    /* corrupt autosave — start empty */
  }
}

// --- drawing-mode click handling ----------------------------------------
function handleDrawClick(p: Vec2): void {
  // `placed` marks a fully completed element; the tool then disarms to normal mode.
  // The body tool spans many clicks, so it disarms itself in finishBody().
  let placed = false;
  switch (tool) {
    case "body":
      handleBodyClick(p);
      break;
    case "joint": {
      // Inside bodies: a joint in each overlapping body, pinned together (a shared
      // revolute). On empty space: a free, body-less joint (a movable point).
      const bodies = scene.bodiesAt(p);
      // Place on the grid; hit-test against the raw click point. If snapping would push
      // the joint outside a body it's being attached to, fall back to the click point
      // (inside every hit body by construction).
      let at = snap(p);
      if (bodies.length > 0 && !bodies.every((b) => scene.pointInBody(b, at))) at = p;
      let created: ReturnType<typeof scene.addFreeJoint>;
      if (bodies.length > 0) {
        const joints = bodies.map((b) => scene.addJoint(b.id, at));
        for (let i = 1; i < joints.length; i++) scene.addPin(joints[0].id, joints[i].id);
        created = joints[0];
      } else {
        created = scene.addFreeJoint(at);
      }
      // If the node landed on a slider rail (or rail node), confine it to that slider as a
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
    case "connect": {
      const j = scene.jointAt(p, pickRadius());
      if (selectedJoint === null) {
        // First pick: the joint to connect.
        if (j) selectedJoint = j.id;
        break;
      }
      // Second pick: a *different* joint → pin them; or a slider line → attach as a
      // rider. A hit on the selected joint itself is ignored so the click can fall
      // through to the slider underneath (the rider often sits right on the rail).
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
      const b = scene.bodyAt(p);
      if (b && scene.toggleBodyGround(b.id)) placed = true;
      break;
    }
    case "slider": {
      // A rail is two joints on the same body (moves with it), or two free joints (a track
      // fixed in world space — addSlider grounds them). Attach riders later via Connect.
      const j = scene.jointAt(p, pickRadius());
      if (!j) break;
      if (sliderDraftIds.length === 0) {
        sliderDraftIds = [j.id];
      } else {
        const a = scene.getJoint(sliderDraftIds[0])!;
        if (j.id === a.id) break; // same joint clicked again — ignore
        const sameBody = a.bodyId !== null && j.bodyId === a.bodyId;
        const bothFree = a.bodyId === null && j.bodyId === null;
        if (sameBody || bothFree) {
          scene.addSlider(a.id, j.id);
          placed = true;
        } else {
          sliderDraftIds = [j.id]; // mismatched (different bodies, or free + body) — restart here
        }
      }
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
      // Single click on a slider rail: drop a self-driving rider on it (a free joint
      // attached as a rider) and create the actuator constraint that will drive it.
      const s = scene.sliderAt(p, pickRadius());
      if (!s) break;
      const created = scene.addLinearActuator(s.id, snap(p));
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
  for (let i = scene.bodies.length - 1; i >= 0; i--) {
    const body = scene.bodies[i];
    const verts = scene.bodyControlWorld(body);
    for (let vi = 0; vi < verts.length; vi++) {
      if (dist(verts[vi], p) <= pickRadius()) return { kind: "vertex", bodyId: body.id, index: vi };
    }
  }
  const gp = scene.guidePointAt(p, pickRadius(), excludeGuide);
  if (gp) return { kind: "guidePoint", guideId: gp.guide.id, which: gp.which };
  return null;
}

/** The line reference a constraint click would pick: a slider rail, then a body control
 *  edge, then a guideline (its infinite line). */
function constraintLineRefAt(p: Vec2): MeasureRef | null {
  const s = scene.sliderAt(p, pickRadius());
  if (s) return { kind: "rail", sliderId: s.id };
  for (let i = scene.bodies.length - 1; i >= 0; i--) {
    const body = scene.bodies[i];
    const verts = scene.bodyControlWorld(body);
    for (let ei = 0; ei < verts.length; ei++) {
      if (distToSegment(p, verts[ei], verts[(ei + 1) % verts.length]) <= pickRadius()) {
        return { kind: "edge", bodyId: body.id, index: ei };
      }
    }
  }
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
    // Body control edge under the cursor (same walk as constraintLineRefAt).
    outer: for (let i = scene.bodies.length - 1; i >= 0; i--) {
      const verts = scene.bodyControlWorld(scene.bodies[i]);
      for (let ei = 0; ei < verts.length; ei++) {
        const a = verts[ei];
        const b = verts[(ei + 1) % verts.length];
        if (distToSegment(p, a, b) <= pickRadius()) {
          seg = { kind: "line", a, b };
          break outer;
        }
      }
    }
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
  if (kind === "coincident") return constraintPointRefAt(p);
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
  const isLine = ref.kind === "rail" || ref.kind === "edge" || ref.kind === "guideLine";
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

/** Add + solve a sketch constraint; on an unsatisfiable solve it's removed again and flashes. */
function commitConstraint(kind: SketchConstraintKind, refA: MeasureRef, refB?: MeasureRef): void {
  const { constraint, breaks } = tryAddConstraint(scene, kind, refA, refB);
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
  const anchors = dragAnchorVars();
  if (!anchors) {
    solveSketch(scene);
    return;
  }
  if (solveSketch(scene, anchors).length > 0) solveSketch(scene);
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
        keys.push(anchorVarForVertex(leftDrag.bodyId, leftDrag.index));
        break;
      case "multi":
        for (const id of leftDrag.bodies) keys.push(...anchorVarsForBody(scene, id));
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
  const sp = worldToScreen(view, lp);
  dimEditInput.style.left = `${sp.x}px`;
  dimEditInput.style.top = `${sp.y}px`;
  dimEditInput.value = String(Math.round(info.value * 10) / 10);
  dimEditInput.classList.remove("hidden");
  dimEditInput.focus();
  dimEditInput.select();
}

function closeDimEditor(): void {
  dimEditId = null;
  dimEditInput.classList.add("hidden");
  dimEditInput.blur();
}

function commitDimEditor(): void {
  const id = dimEditId;
  const raw = dimEditInput.value.trim();
  closeDimEditor(); // nulls dimEditId first, so the blur listener doesn't re-commit
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
  const breaks = applyDrivingDimension(scene, id, target);
  if (breaks.length) flashSketchItems(breaks);
  else markDirty();
}

dimEditInput.addEventListener("keydown", (e) => {
  e.stopPropagation(); // keep canvas shortcuts (tools, Delete…) out of the text field
  if (e.key === "Enter") commitDimEditor();
  else if (e.key === "Escape") closeDimEditor();
});
dimEditInput.addEventListener("blur", () => {
  if (dimEditId !== null) commitDimEditor();
});

// --- measure tool ----------------------------------------------------------
/**
 * The measure reference a click at `p` would pick, by priority: a joint, a body control
 * vertex, a slider rail, a body control-polygon edge, and finally any point inside a
 * body (fixed in that body's frame, grid-snapped when snapping keeps it inside). Empty
 * space picks nothing — references live on existing geometry only.
 */
function measureRefAt(p: Vec2): MeasureRef | null {
  const j = scene.jointAt(p, pickRadius());
  if (j) return { kind: "joint", jointId: j.id };
  for (let i = scene.bodies.length - 1; i >= 0; i--) {
    const body = scene.bodies[i];
    const verts = scene.bodyControlWorld(body);
    for (let vi = 0; vi < verts.length; vi++) {
      if (dist(verts[vi], p) <= pickRadius()) return { kind: "vertex", bodyId: body.id, index: vi };
    }
  }
  // Guides are draw-mode-only aids (invisible in sim), so only draw-mode picks see them.
  if (mode === "draw") {
    const gp = scene.guidePointAt(p, pickRadius());
    if (gp) return { kind: "guidePoint", guideId: gp.guide.id, which: gp.which };
  }
  const s = scene.sliderAt(p, pickRadius());
  if (s) return { kind: "rail", sliderId: s.id };
  for (let i = scene.bodies.length - 1; i >= 0; i--) {
    const body = scene.bodies[i];
    const verts = scene.bodyControlWorld(body);
    for (let ei = 0; ei < verts.length; ei++) {
      if (distToSegment(p, verts[ei], verts[(ei + 1) % verts.length]) <= pickRadius()) {
        return { kind: "edge", bodyId: body.id, index: ei };
      }
    }
  }
  if (mode === "draw") {
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
    const ref = measureRefAt(p);
    if (!ref) return; // empty space — keep waiting for a reference
    if (measurePicks.length === 1 && sameMeasureRef(measurePicks[0], ref)) return;
    measurePicks.push(ref);
    return;
  }
  const m = scene.addMeasurement(mode === "sim" ? "sim" : "draw", measurePicks[0], measurePicks[1], p);
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
  const ml = measurementLabelAt(p);
  if (ml) {
    selection = { kind: "measure", id: ml.id };
    return;
  }
  const sg = sketchGlyphAt(p);
  if (sg !== null) {
    selection = { kind: "sketch", id: sg };
    return;
  }
  const j = scene.jointAt(p, pickRadius());
  if (j) {
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
    selection = { kind: "slider", id: s.id };
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
  if (selection?.kind === "body" && (selectedBodyVertexAt(p) >= 0 || selectedBodyEdgeAt(p))) {
    return;
  }
  const body = scene.bodyAt(p);
  if (body) {
    const g = scene.groupOf(body.id);
    if (g) {
      // A grouped body is selection-atomic: clicking any member selects the whole group.
      setMulti(new Set(g.bodyIds), new Set());
      return;
    }
    selection = { kind: "body", id: body.id };
    return;
  }
  selection = null;
}

/** Index of the selected body's control vertex within pick range of `p`, or -1. */
function selectedBodyVertexAt(p: Vec2): number {
  if (selection?.kind !== "body") return -1;
  const body = scene.getBody(selection.id);
  if (!body) return -1;
  let best = -1;
  let bestD = pickRadius();
  scene.bodyControlWorld(body).forEach((v, i) => {
    const d = dist(v, p);
    if (d <= bestD) {
      bestD = d;
      best = i;
    }
  });
  return best;
}

/**
 * Nearest control-polygon edge of the selected body within pick range of `p`.
 * Returns the edge's later-vertex index (insertion slot) and the closest point on
 * the segment, or null. Vertices within pick range are excluded so an edge hit
 * doesn't shadow a vertex hit (which means "remove" instead of "add").
 */
function selectedBodyEdgeAt(p: Vec2): { index: number; point: Vec2 } | null {
  if (selection?.kind !== "body") return null;
  const body = scene.getBody(selection.id);
  if (!body) return null;
  const verts = scene.bodyControlWorld(body);
  if (selectedBodyVertexAt(p) >= 0) return null;
  const r = pickRadius();
  let best: { index: number; point: Vec2 } | null = null;
  let bestD = r;
  for (let i = 0; i < verts.length; i++) {
    const a = verts[i];
    const b = verts[(i + 1) % verts.length];
    const ab = sub(b, a);
    const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / Math.max(lenSq(ab), 1e-9)));
    const point = add(a, scale(ab, t));
    const d = dist(p, point);
    if (d < bestD) {
      bestD = d;
      best = { index: i + 1, point };
    }
  }
  return best;
}

/** Delete the currently selected element and its dependent features. */
function deleteSelection(): void {
  if (multiSel) {
    for (const id of multiSel.bodies) scene.removeBody(id);
    for (const id of multiSel.joints) scene.removeJoint(id);
    multiSel = null;
    selection = null;
    markDirty();
    return;
  }
  if (!selection) return;
  if (selection.kind === "body") scene.removeBody(selection.id);
  else if (selection.kind === "joint") scene.removeJoint(selection.id);
  else if (selection.kind === "measure") scene.removeMeasurement(selection.id);
  else if (selection.kind === "sketch") scene.removeSketchConstraint(selection.id);
  else if (selection.kind === "guide") scene.removeGuide(selection.id);
  else scene.removeConstraint(selection.id); // slider: remove it, keep the joints
  selection = null;
  markDirty();
}

/** Copy the selection (a body, or a whole multi-selection / group) to the clipboard. */
function copySelection(): void {
  if (mode !== "draw") return;
  if (multiSel) {
    clipboard = scene.extractSelection([...multiSel.bodies], [...multiSel.joints]);
  } else if (selection?.kind === "body") {
    clipboard = scene.extractSelection([selection.id]);
  }
}

/** Paste the clipboard so its centre lands at `at` (grid-snapped), then select the copy. */
function pasteAt(at: Vec2 | null): void {
  if (mode !== "draw" || !clipboard) return;
  const drop = snap(at ?? screenToWorld(view, vec(canvas.clientWidth / 2, canvas.clientHeight / 2)));
  const res = scene.insertSelection(clipboard, drop);
  if (res) {
    // A single pasted body collapses to a normal selection; a fragment stays multi-selected.
    setMulti(new Set(res.bodyIds), new Set(res.freeJointIds));
    markDirty();
  }
}

/** Mirror the selection in place: a single body about its centroid, a multi-selection /
 *  group about the centre of its combined bounding box. */
function mirrorSelection(axis: "h" | "v"): void {
  if (mode !== "draw") return;
  if (multiSel) {
    scene.mirrorBodies([...multiSel.bodies], [...multiSel.joints], axis);
    markDirty();
    return;
  }
  if (selection?.kind !== "body") return;
  scene.mirrorBody(selection.id, axis);
  markDirty();
}

/**
 * Begin a rotate (rotate tool). A control node of the already-selected body rotates that
 * body about the node. Otherwise the body under the cursor decides: a body of the current
 * multi-selection — or of a permanent group, which gets selected — rotates the whole
 * selection about the centre of its combined bounding box; a lone body rotates about its
 * centroid (and becomes the selection). No body → nothing happens.
 */
function startRotate(p: Vec2): void {
  const vi = selectedBodyVertexAt(p);
  if (vi >= 0 && selection?.kind === "body") {
    const pivot = scene.bodyControlWorld(scene.getBody(selection.id)!)[vi];
    beginRotate([selection.id], [], pivot, p);
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
  rotateDrag = {
    bodyIds,
    jointIds,
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
    if (addSliderRiderToDraft(p)) return; // landed on a slider rail
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
  if (mode === "draw") {
    if (tool === "rotate") {
      startRotate(world);
    } else if (tool === null) {
      // Ctrl/Cmd+click: toggle what's under the cursor in the multi-selection; on empty
      // space, start an additive box select instead.
      if (e.ctrlKey || e.metaKey) {
        if (!toggleMultiAt(world)) {
          boxSelect = { start: world, end: world, additive: true, moved: false };
        }
        return;
      }
      // A selected body shows draggable corner handles; grabbing one reshapes the body.
      const vi = selectedBodyVertexAt(world);
      if (vi >= 0 && selection?.kind === "body") {
        const anchor = scene.bodyControlWorld(scene.getBody(selection.id)!)[vi];
        leftDrag = { kind: "vertex", bodyId: selection.id, index: vi, grabOffset: sub(world, anchor), moved: false };
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
        } else if (selection?.kind === "body") {
          const anchor = bodyDragAnchor(selection.id, world); // centroid or nearest corner
          leftDrag = {
            kind: "body",
            id: selection.id,
            anchorOffset: sub(anchor, scene.getBody(selection.id)!.pos),
            grabOffset: sub(world, anchor),
            moved: false,
          };
          canvas.style.cursor = "move";
        } else if (selection?.kind === "joint") {
          const anchor = scene.jointWorld(scene.getJoint(selection.id)!);
          leftDrag = { kind: "joint", id: selection.id, grabOffset: sub(world, anchor), moved: false };
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
    const j = scene.jointAt(world, pickRadius());
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
      scene.setMeasurementLabel(leftDrag.id, sub(world, leftDrag.grabOffset));
      leftDrag.moved = true;
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
    // Snap the dragged anchor to the grid (in absolute terms), preserving where it was
    // grabbed. A dragged guideline is left out of the snap targets (it can't snap to itself).
    const target = snap(
      sub(world, leftDrag.grabOffset),
      leftDrag.kind === "guide" ? leftDrag.id : undefined
    );
    const delta = sub(target, dragAnchorWorld(leftDrag));
    if (leftDrag.kind === "vertex") scene.moveBodyVertex(leftDrag.bodyId, leftDrag.index, delta);
    else if (leftDrag.kind === "body") scene.moveBody(leftDrag.id, delta);
    else if (leftDrag.kind === "guide") scene.moveGuide(leftDrag.id, delta);
    else if (leftDrag.kind === "multi") {
      // Move the whole multi-selection together by the same delta (the anchor is
      // re-read live in dragAnchorWorld, so the delta always closes the real gap).
      for (const id of leftDrag.bodies) scene.moveBody(id, delta);
      for (const id of leftDrag.joints) scene.moveJoint(id, delta);
    } else scene.moveJoint(leftDrag.id, delta);
    leftDrag.moved = true;
    solveSketchLive(); // sketch dragging: constraints hold while the geometry follows
    return;
  }

  hoverJoint = scene.jointAt(world, pickRadius())?.id ?? null;
  // In normal/select mode, also highlight the body under the cursor (when no joint is).
  hoverBody =
    mode === "draw" && tool === null && hoverJoint === null
      ? scene.bodyAt(world)?.id ?? null
      : null;
  // Hint that elements are grabbable: a move cursor over a joint/body/handle/label in select mode.
  if (mode === "draw" && tool === null) {
    const grabbable =
      selectedBodyVertexAt(world) >= 0 ||
      hoverJoint !== null ||
      hoverBody !== null ||
      measurementLabelAt(world) !== null ||
      scene.guidePointAt(world, pickRadius()) !== null ||
      scene.guideAt(world, pickRadius()) !== undefined;
    // With Shift held a drag would be rigid (sim-style), so hint with the sim grab cursor.
    canvas.style.cursor = grabbable ? (e.shiftKey ? "grab" : "move") : "crosshair";
  }
  // Rotate tool: a grab cursor over a node of the selected body or any body.
  if (mode === "draw" && tool === "rotate") {
    const rotatable = selectedBodyVertexAt(world) >= 0 || scene.bodyAt(world) !== undefined;
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
  if (e.button === 2 && pan) {
    pan = null;
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
      } else if (finished.kind !== "measureLabel") {
        // Settle: one symmetric sketch solve at rest, repairing anything the anchored
        // live solves couldn't satisfy without moving the dragged geometry.
        solveSketchLive();
      }
      markDirty(); // persist a reposition (a plain click just selects)
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
  // Freehand polygons still close on double-click; joint-built bodies finish by
  // clicking a previously-added node (handled in handleBodyClick).
  if (mode === "draw" && tool === "body" && jointDraftIds.length === 0) {
    finishBody();
    return;
  }
  // Select mode: double-click a draw-mode dimension label to edit its value inline
  // (typing a number makes it a driving dimension; clearing it makes it driven again).
  if (mode === "draw" && tool === null) {
    const ml = measurementLabelAt(eventWorld(e));
    if (ml) {
      leftDrag = null; // the double-click's mousedowns started a label drag — cancel it
      openDimEditor(ml);
      return;
    }
  }
  // Select mode, body selected: double-click edits the control polygon. On a vertex →
  // remove it (kept ≥ 3); on an edge → add a node at the click point (grid-snapped).
  if (mode === "draw" && tool === null && selection?.kind === "body") {
    const world = eventWorld(e);
    const vi = selectedBodyVertexAt(world);
    if (vi >= 0) {
      scene.removeBodyVertex(selection.id, vi);
      markDirty();
      return;
    }
    const edge = selectedBodyEdgeAt(world);
    if (edge) {
      scene.insertBodyVertex(selection.id, edge.index, snap(edge.point));
      markDirty();
    }
  }
});

/** Draw-tool shortcuts: mostly the first letter of the tool's name (L = guideLine —
 *  G is Ground; the actuator moved to A when L was given to guidelines). */
const TOOL_KEYS: Record<string, Tool> = {
  b: "body",
  j: "joint",
  c: "connect",
  g: "ground",
  s: "slider",
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
    // Abort the current placement / drag and return to the mode's normal state
    // (in sim this also disarms the measure tool).
    disarmTool();
    return;
  }
  if (e.key === "Enter" && mode === "draw" && tool === "body") {
    finishBody();
    return;
  }
  // A selected measurement is deletable in either mode (sim keeps its own set).
  if ((e.key === "Delete" || e.key === "Backspace") && selection?.kind === "measure") {
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
  // [ and ] adjust the selected body's corner radius (round / un-round it).
  if ((e.key === "[" || e.key === "]") && mode === "draw" && tool === null && selection?.kind === "body") {
    const body = scene.getBody(selection.id);
    if (body) {
      scene.setBodyRadius(body.id, body.radius + (e.key === "]" ? RADIUS_STEP : -RADIUS_STEP));
      markDirty();
    }
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
  if (selection.kind === "slider") {
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
  if (tool === "slider") return sliderDraftIds;
  if (tool === "body") return jointDraftIds;
  if (tool === "motor") return motorPivotDraft !== null ? [motorPivotDraft] : [];
  return [];
}

/** Rail-joint positions picked so far for the slider tool, with the live cursor. */
function sliderDraftView(): { rail: Vec2[]; cursor: Vec2 } | null {
  if (mode !== "draw" || tool !== "slider" || sliderDraftIds.length === 0 || !cursor) return null;
  return { rail: sliderDraftIds.map((id) => scene.jointWorld(scene.getJoint(id)!)), cursor };
}

/** Guide tool: the first defining point placed, with the live cursor (line preview,
 *  landing where the click would — on elements, projections, or the grid). */
function guideDraftView(): { a: Vec2; cursor: Vec2 } | null {
  if (mode !== "draw" || tool !== "guide" || guideDraft === null || !cursor) return null;
  return { a: guideDraft, cursor: guidePlacementAt(cursor).at };
}

/** Control-vertex handles to show for the body selected in select / rotate mode (else null). */
function editVerticesView(): Vec2[] | null {
  if (mode !== "draw" || (tool !== null && tool !== "rotate") || selection?.kind !== "body")
    return null;
  const body = scene.getBody(selection.id);
  return body ? scene.bodyControlWorld(body) : null;
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
  refs: ResolvedMeasureRef[];
  hover: ResolvedMeasureRef | null;
  preview: MeasureInfo | null;
} | null {
  if (tool !== "measure") return null;
  const refs = measurePicks
    .map((r) => scene.resolveMeasureRef(r))
    .filter((r): r is ResolvedMeasureRef => r !== null);
  let hover: ResolvedMeasureRef | null = null;
  let preview: MeasureInfo | null = null;
  if (cursor) {
    if (measurePicks.length < 2) {
      const h = measureRefAt(cursor);
      hover = h ? scene.resolveMeasureRef(h) : null;
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
    case "vertex": return `v:${ref.bodyId}:${ref.index}`;
    case "edge": return `e:${ref.bodyId}:${ref.index}`;
    case "rail": return `r:${ref.sliderId}`;
    case "guidePoint": return `gp:${ref.guideId}:${ref.which}`;
    case "guideLine": return `gl:${ref.guideId}`;
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
 * gets a single badge (its two points share a position). Badges render faded unless the
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
        const n = vec(-d.y, d.x);
        badges.push(add(add(mid, scale(n, px(14))), scale(d, px(i * 20))));
      }
    }
    if (!badges.length) continue;
    const hot =
      cursor !== null &&
      (allRefs.some((ref) => refHovered(ref, cursor!)) ||
        badges.some((b) => dist(b, cursor!) <= GLYPH_PICK_RADIUS / view.scale));
    out.push({ id: c.id, kind: c.kind, badges, faded: !hot });
  }
  sketchGlyphCache = out;
  return out;
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
  if (sketchFlash && performance.now() >= sketchFlash.until) sketchFlash = null;
  render(ctx, {
    scene,
    view,
    mode,
    draftBody: mode === "draw" && tool === "body" ? draftBody : null,
    cursor,
    hoverJoint,
    hoverBody: mode === "draw" && tool === null ? hoverBody : null,
    activeJoints: activeJoints(),
    // In sim only a measurement selection is meaningful (labels stay editable there).
    selection: mode === "draw" ? selection : selection?.kind === "measure" ? selection : null,
    multiSelected:
      mode === "draw" && multiSel
        ? { bodies: [...multiSel.bodies], joints: [...multiSel.joints] }
        : null,
    marquee: boxSelect?.moved ? { a: boxSelect.start, b: boxSelect.end } : null,
    editVertices: editVerticesView(),
    sliderDraft: sliderDraftView(),
    guideDraft: guideDraftView(),
    bodyJointDraft: bodyJointDraftView(),
    driverJoint: driver?.jointId ?? null,
    rotatePivot: rotateDrag?.pivot ?? null,
    gridStep,
    gridVisible,
    breaks: mode === "sim" ? solveBreaks : [],
    measurements: measurementsView(),
    measureDraft: measureDraftView(),
    sketchGlyphs: sketchGlyphsView(),
    sketchDraft: sketchDraftView(),
    flash: sketchFlash?.ids ?? null,
    theme: theme === "light" ? LIGHT_THEME : DARK_THEME,
  });
  requestAnimationFrame(frame);
}

resize();
restoreAutosave();
pushHistory(); // seed the undo history with the initial (restored) layout
updateHint();
requestAnimationFrame(frame);
