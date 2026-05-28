import "./style.css";
import { Scene, SceneData, BodyClip, LinearActuatorConstraint, MotorConstraint } from "./model";
import { solve, Driver, ConstraintBreak } from "./solver";
import { render, DARK_THEME, LIGHT_THEME } from "./renderer";
import { Vec2, add, dist, sub, vec, dot, lenSq, scale, rotate, roundedConvexBody } from "./geometry";
import { View, screenToWorld, zoomAt } from "./view";

type Mode = "draw" | "sim";
type Tool = "body" | "joint" | "connect" | "ground" | "slider" | "rotate" | "linearActuator" | "motor";
/** An existing element picked in normal/select mode. */
type Selection = { kind: "body" | "joint" | "slider"; id: number };

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
let jointDraftIds: number[] = []; // joints picked to build a body (body tool, joint start)
let jointDraftCreated: number[] = []; // joints made on slider rails during that draft (removed if aborted)
let jointDraftExpanding = false; // body-from-joints: sizing the outward margin
let cursor: Vec2 | null = null; // world coordinates
let hoverJoint: number | null = null;
let hoverBody: number | null = null; // body under the cursor in normal mode
let selectedJoint: number | null = null; // first pick for connect
let sliderDraftIds: number[] = []; // rail joints picked so far for the slider tool (0–2)
let selection: Selection | null = null; // element selected in normal mode
let driver: Driver | null = null;
/** Constraints the last solve couldn't satisfy (impossible assembly); drives the red overlay + banner. */
let solveBreaks: ConstraintBreak[] = [];
/** Body poses saved when entering simulation, restored when leaving. */
let savedPoses: Map<number, { pos: Vec2; angle: number }> | null = null;
/** Last body copied (Ctrl+C / Copy button); pasted at the cursor with Ctrl+V / Paste. */
let clipboard: BodyClip | null = null;
/**
 * Active rotate (rotate tool): turning `bodyId` about a fixed `pivot`. `grabAngle` is the
 * body's angle at grab; `prevPointer` / `accum` track the pointer's accumulated swing about
 * the pivot (unwrapped); `lastTotal` is the rotation applied so far (lets us apply only the
 * incremental delta each move while snapping the absolute angle to 45°).
 */
type RotateDrag = {
  bodyId: number;
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

// --- grid / snapping -------------------------------------------------------
/** Grid spacing (and snap increment) in world units; mirrors the renderer's grid. */
let gridStep = 40;
/** When true, placements and drags land on the nearest grid intersection. */
let snapEnabled = false;
/** When true, the world-locked grid is drawn (snapping still works when hidden). */
let gridVisible = true;

/** Snap a world point to the nearest grid intersection (identity when snap is off). */
function snap(p: Vec2): Vec2 {
  if (!snapEnabled) return p;
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
  | { kind: "vertex"; bodyId: number; index: number; grabOffset: Vec2; moved: boolean };
let leftDrag: LeftDrag | null = null;

/** Current world position of a drag's anchor (the point that snaps to the grid). */
function dragAnchorWorld(d: LeftDrag): Vec2 {
  if (d.kind === "vertex") return scene.bodyControlWorld(scene.getBody(d.bodyId)!)[d.index];
  if (d.kind === "body") return add(scene.getBody(d.id)!.pos, d.anchorOffset);
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

// --- canvas sizing -------------------------------------------------------
function resize(): void {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(canvas.clientWidth * dpr);
  canvas.height = Math.floor(canvas.clientHeight * dpr);
}
window.addEventListener("resize", resize);

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
  return mode === "sim" ? "grab" : "crosshair";
}

// --- hint text -----------------------------------------------------------
const HINTS: Record<Mode | Tool | "select", string> = {
  draw: "",
  sim: "Drag any joint, or part of a body, to drive the mechanism. Space to run / pause actuators.",
  select: "Click to select · drag to move · drag a selected body's corner handles to reshape · double-click an edge to add a node / a node to remove it · [ and ] round corners · Delete to remove.",
  body: "Empty space: click vertices to draw a polygon. Joints: click joints to build a body, click a node again to finish, then move out to set thickness and click.",
  joint: "Click inside a body to attach a joint, or empty space to place a free joint.",
  connect: "Click a joint, then another joint to pin them — or a slider line to attach the joint to it.",
  ground: "Click a joint to lock its position (it can still rotate).",
  slider: "Click two joints on the same body to create a slider rail.",
  rotate: "Drag a body to rotate it about its centroid, or drag a selected body's node to rotate about that node. Snaps to 45°.",
  linearActuator: "Click a slider rail to drop a self-driving rider — it travels back and forth when animation runs.",
  motor: "Click a joint to set the pivot, then another joint on the same body for the crank pin.",
};

function updateHint(): void {
  if (mode === "sim") hintEl.textContent = HINTS.sim;
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
document.getElementById("save-btn")!.addEventListener("click", saveToFile);
document.getElementById("load-btn")!.addEventListener("click", () => fileInput.click());
// Copy/paste are keyboard-only (Ctrl/Cmd+C / V); no toolbar buttons.
document.getElementById("mirror-h-btn")!.addEventListener("click", () => mirrorSelection("h"));
document.getElementById("mirror-v-btn")!.addEventListener("click", () => mirrorSelection("v"));

runBtn.addEventListener("click", () => setAnimating(!animating));
autopauseBtn.addEventListener("click", () => setPauseOnImpossible(!pauseOnImpossible));

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
const GRID_MIN = 1;
const GRID_MAX = 200;
/** Read the grid-size field, clamped to [GRID_MIN, GRID_MAX]; null while it's empty/invalid. */
function parseGridSize(): number | null {
  const n = Math.round(Number(gridSizeInput.value));
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
  runBtn.classList.toggle("hidden", mode === "draw");
  autopauseBtn.classList.toggle("hidden", mode === "draw");
  canvas.style.cursor = mode === "sim" ? "grab" : "crosshair";
  updateHint();
  updateSimError(); // show/hide the banner for the mode we just entered
  syncPropsPanel(); // selection cleared → properties panels hide
}

function setTool(next: Tool): void {
  // Rotate operates on a selected body, so keep an existing body selection when arming it
  // (lets you grab one of its control nodes as the pivot right away).
  const keepSel = next === "rotate" && selection?.kind === "body" ? selection : null;
  tool = next;
  resetTransient();
  selection = keepSel;
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
  draftBody = [];
  // Discard any slider-rail joints made for an unfinished body-from-joints draft (a finished
  // build clears this list first, so its absorbed joints survive).
  for (const id of jointDraftCreated) scene.removeJoint(id);
  jointDraftCreated = [];
  jointDraftIds = [];
  jointDraftExpanding = false;
  selectedJoint = null;
  sliderDraftIds = [];
  motorPivotDraft = null;
  selection = null;
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
      const at = snap(p); // place on the grid; hit-test against the raw click point
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
        scene.addGround(j.id, scene.jointWorld(j));
        placed = true;
      }
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
  }
  markDirty();
  if (placed) disarmTool();
}

/** Normal/select mode: pick a joint (preferred), then a slider rail, then a body. */
function handleSelectClick(p: Vec2): void {
  const j = scene.jointAt(p, pickRadius());
  if (j) {
    selection = { kind: "joint", id: j.id };
    return;
  }
  const s = scene.sliderAt(p, pickRadius());
  if (s) {
    selection = { kind: "slider", id: s.id };
    return;
  }
  // Keep the selected body when clicking on/near its control polygon (its edges sit on
  // the boundary, so a click there can land just outside the filled shape). This lets a
  // double-click on an edge reach the vertex-edit handler without deselecting first.
  if (selection?.kind === "body" && (selectedBodyVertexAt(p) >= 0 || selectedBodyEdgeAt(p))) {
    return;
  }
  const body = scene.bodyAt(p);
  selection = body ? { kind: "body", id: body.id } : null;
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
  if (!selection) return;
  if (selection.kind === "body") scene.removeBody(selection.id);
  else if (selection.kind === "joint") scene.removeJoint(selection.id);
  else scene.removeConstraint(selection.id); // slider: remove it, keep the joints
  selection = null;
  markDirty();
}

/** Copy the selected body (with its joints and own constraints) to the clipboard. */
function copySelection(): void {
  if (mode !== "draw" || selection?.kind !== "body") return;
  clipboard = scene.extractBody(selection.id);
}

/** Paste the clipboard body so its centroid lands at `at` (grid-snapped), then select it. */
function pasteAt(at: Vec2 | null): void {
  if (mode !== "draw" || !clipboard) return;
  const drop = snap(at ?? screenToWorld(view, vec(canvas.clientWidth / 2, canvas.clientHeight / 2)));
  const id = scene.insertBody(clipboard, drop);
  if (id !== null) {
    selection = { kind: "body", id };
    markDirty();
  }
}

/** Mirror the selected body in place across the given axis through its centroid. */
function mirrorSelection(axis: "h" | "v"): void {
  if (mode !== "draw" || selection?.kind !== "body") return;
  scene.mirrorBody(selection.id, axis);
  markDirty();
}

/**
 * Begin a rotate (rotate tool). Pivot: a control node of the already-selected body if the
 * grab lands on one, otherwise the centroid of whichever body is under the cursor (which
 * also becomes the selection). No body → nothing happens.
 */
function startRotate(p: Vec2): void {
  let bodyId: number | null = null;
  let pivot: Vec2 | null = null;
  const vi = selectedBodyVertexAt(p);
  if (vi >= 0 && selection?.kind === "body") {
    bodyId = selection.id;
    pivot = scene.bodyControlWorld(scene.getBody(bodyId)!)[vi];
  } else {
    const body = scene.bodyAt(p);
    if (body) {
      bodyId = body.id;
      pivot = body.pos; // centroid
      selection = { kind: "body", id: bodyId };
    }
  }
  if (bodyId === null || pivot === null) return;
  rotateDrag = {
    bodyId,
    pivot,
    grabAngle: scene.getBody(bodyId)!.angle,
    prevPointer: Math.atan2(p.y - pivot.y, p.x - pivot.x),
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
    const at = snap(p);
    const under = scene.bodyAt(p);
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
  const at = snap(p); // freehand vertices land on the grid when snap is on
  // Ignore near-duplicate points (also de-dupes the 2nd click of a double-click).
  const last = draftBody[draftBody.length - 1];
  if (last && dist(at, last) < 4 / view.scale) return;
  draftBody.push(at);
}

function finishBody(): void {
  if (draftBody.length >= 3) {
    scene.addBody(draftBody).color = defaultBodyColor;
    draftBody = [];
    markDirty();
    disarmTool();
    return;
  }
  draftBody = [];
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
      // A selected body shows draggable corner handles; grabbing one reshapes the body.
      const vi = selectedBodyVertexAt(world);
      if (vi >= 0 && selection?.kind === "body") {
        const anchor = scene.bodyControlWorld(scene.getBody(selection.id)!)[vi];
        leftDrag = { kind: "vertex", bodyId: selection.id, index: vi, grabOffset: sub(world, anchor), moved: false };
        canvas.style.cursor = "move";
      } else {
        // Otherwise select what's under the cursor; if it's movable, begin a drag of it.
        handleSelectClick(world);
        if (selection?.kind === "body") {
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
        }
      }
    } else {
      handleDrawClick(world);
    }
  } else {
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

  if (rotateDrag) {
    // Accumulate the pointer's swing about the pivot (unwrapped so it survives crossing ±π),
    // snap the resulting absolute body angle to 45°, then apply only the incremental delta.
    const ptr = Math.atan2(world.y - rotateDrag.pivot.y, world.x - rotateDrag.pivot.x);
    rotateDrag.accum += wrapAngle(ptr - rotateDrag.prevPointer);
    rotateDrag.prevPointer = ptr;
    const total = snapAngle(rotateDrag.grabAngle + rotateDrag.accum) - rotateDrag.grabAngle;
    scene.rotateBody(rotateDrag.bodyId, rotateDrag.pivot, total - rotateDrag.lastTotal);
    rotateDrag.lastTotal = total;
    rotateDrag.moved = true;
    return;
  }

  if (leftDrag) {
    // Snap the dragged anchor to the grid (in absolute terms), preserving where it was grabbed.
    const target = snap(sub(world, leftDrag.grabOffset));
    const delta = sub(target, dragAnchorWorld(leftDrag));
    if (leftDrag.kind === "vertex") scene.moveBodyVertex(leftDrag.bodyId, leftDrag.index, delta);
    else if (leftDrag.kind === "body") scene.moveBody(leftDrag.id, delta);
    else scene.moveJoint(leftDrag.id, delta);
    leftDrag.moved = true;
    return;
  }

  hoverJoint = scene.jointAt(world, pickRadius())?.id ?? null;
  // In normal/select mode, also highlight the body under the cursor (when no joint is).
  hoverBody =
    mode === "draw" && tool === null && hoverJoint === null
      ? scene.bodyAt(world)?.id ?? null
      : null;
  // Hint that elements are grabbable: a move cursor over a joint/body/handle in select mode.
  if (mode === "draw" && tool === null) {
    const grabbable = selectedBodyVertexAt(world) >= 0 || hoverJoint !== null || hoverBody !== null;
    canvas.style.cursor = grabbable ? "move" : "crosshair";
  }
  // Rotate tool: a grab cursor over a node of the selected body or any body.
  if (mode === "draw" && tool === "rotate") {
    const rotatable = selectedBodyVertexAt(world) >= 0 || scene.bodyAt(world) !== undefined;
    canvas.style.cursor = rotatable ? "grab" : "crosshair";
  }
  if (mode === "sim") {
    if (driver) driver.target = world;
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
  if (e.button === 0 && leftDrag) {
    if (leftDrag.moved) markDirty(); // persist a reposition (a plain click just selects)
    leftDrag = null;
    canvas.style.cursor = defaultCursor();
  }
  if (e.button === 0 && rotateDrag) {
    if (rotateDrag.moved) markDirty(); // a plain click (no drag) only selected the body
    rotateDrag = null;
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

/** Draw-tool shortcuts: each tool is armed by the first letter of its name. */
const TOOL_KEYS: Record<string, Tool> = {
  b: "body",
  j: "joint",
  c: "connect",
  g: "ground",
  s: "slider",
  r: "rotate",
  l: "linearActuator",
  m: "motor",
};

window.addEventListener("keydown", (e) => {
  // Space toggles the actuator animation (sim mode only).
  if (e.code === "Space" && mode === "sim" && !e.ctrlKey && !e.metaKey && !e.altKey) {
    e.preventDefault();
    setAnimating(!animating);
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
  // Copy / paste the selected body (draw mode). Paste lands at the cursor.
  if (mod && e.key.toLowerCase() === "c" && mode === "draw" && selection?.kind === "body") {
    e.preventDefault();
    copySelection();
    return;
  }
  if (mod && e.key.toLowerCase() === "v" && mode === "draw" && clipboard) {
    e.preventDefault();
    pasteAt(cursor);
    return;
  }
  if (e.key === "Escape") {
    // Abort the current placement and return to normal/select mode.
    if (mode === "draw") disarmTool();
    else resetTransient();
    return;
  }
  if (e.key === "Enter" && mode === "draw" && tool === "body") {
    finishBody();
    return;
  }
  if (
    (e.key === "Delete" || e.key === "Backspace") &&
    mode === "draw" &&
    tool === null &&
    selection
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
  if (on) fitPhases();
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
/** Run a solve and log how long the calculation took (debug). */
function timedSolve(
  label: string,
  drv: Driver | null,
  iterations = 100,
  anchors?: Map<number, Vec2>
): void {
  const t0 = performance.now();
  solveBreaks = solve(scene, drv, iterations, 1, anchors);
  console.log(`[Disjointed] ${label} solve: ${(performance.now() - t0).toFixed(3)} ms`);
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

/** Control-vertex handles to show for the body selected in select / rotate mode (else null). */
function editVerticesView(): Vec2[] | null {
  if (mode !== "draw" || (tool !== null && tool !== "rotate") || selection?.kind !== "body")
    return null;
  const body = scene.getBody(selection.id);
  return body ? scene.bodyControlWorld(body) : null;
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
    timedSolve("anim", driver, 60, computeAnchors());
    // Safety: stop the animation once the assembly has reported breaks for a few frames
    // in a row (a brief debounce filters single-frame solver chatter on complex loops).
    impossibleFrames = solveBreaks.length > 0 ? impossibleFrames + 1 : 0;
    if (pauseOnImpossible && impossibleFrames >= IMPOSSIBLE_PAUSE_FRAMES) setAnimating(false);
  } else if (mode === "sim" && driver) {
    timedSolve("drive", driver);
  }
  syncColorPicker();
  syncPropsPanel();
  render(ctx, {
    scene,
    view,
    mode,
    draftBody: mode === "draw" && tool === "body" ? draftBody : null,
    cursor,
    hoverJoint,
    hoverBody: mode === "draw" && tool === null ? hoverBody : null,
    activeJoints: activeJoints(),
    selection: mode === "draw" ? selection : null,
    editVertices: editVerticesView(),
    sliderDraft: sliderDraftView(),
    bodyJointDraft: bodyJointDraftView(),
    driverJoint: driver?.jointId ?? null,
    rotatePivot: rotateDrag?.pivot ?? null,
    gridStep,
    gridVisible,
    breaks: mode === "sim" ? solveBreaks : [],
    theme: theme === "light" ? LIGHT_THEME : DARK_THEME,
  });
  requestAnimationFrame(frame);
}

resize();
restoreAutosave();
pushHistory(); // seed the undo history with the initial (restored) layout
updateHint();
requestAnimationFrame(frame);
