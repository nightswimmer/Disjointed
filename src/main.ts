import "./style.css";
import { Scene, SceneData } from "./model";
import { solve, Driver } from "./solver";
import { render } from "./renderer";
import { Vec2, dist, sub, vec, roundedConvexBody } from "./geometry";
import { View, screenToWorld, zoomAt } from "./view";

type Mode = "draw" | "sim";
type Tool = "body" | "joint" | "connect" | "ground" | "slider";
/** An existing element picked in normal/select mode. */
type Selection = { kind: "body" | "joint" | "slider"; id: number };

/** Pick / close thresholds in screen (CSS) pixels — converted to world units via the view. */
const PICK_RADIUS = 12;
const CLOSE_RADIUS = 12;
/** Smallest outward margin (world units) when expanding a body built from joints. */
const JOINT_BODY_MIN_MARGIN = 4;
/** How much the [ and ] keys change a selected body's corner radius, per press. */
const RADIUS_STEP = 4;

const canvas = document.getElementById("scene") as HTMLCanvasElement;
const ctx = canvas.getContext("2d")!;
const hintEl = document.getElementById("hint")!;
const toolGroup = document.getElementById("tool-group")!;

const scene = new Scene();

// --- interaction state ---------------------------------------------------
let mode: Mode = "draw";
/** Armed draw tool, or null for normal/select mode. Tools disarm after one use. */
let tool: Tool | null = null;
let draftBody: Vec2[] = []; // freehand polygon vertices (body tool, empty-space start)
let jointDraftIds: number[] = []; // joints picked to build a body (body tool, joint start)
let jointDraftExpanding = false; // body-from-joints: sizing the outward margin
let cursor: Vec2 | null = null; // world coordinates
let hoverJoint: number | null = null;
let hoverBody: number | null = null; // body under the cursor in normal mode
let selectedJoint: number | null = null; // first pick for connect
let sliderDraftIds: number[] = []; // rail joints picked so far for the slider tool (0–2)
let selection: Selection | null = null; // element selected in normal mode
let driver: Driver | null = null;
/** Body poses saved when entering simulation, restored when leaving. */
let savedPoses: Map<number, { pos: Vec2; angle: number }> | null = null;

// --- camera ---------------------------------------------------------------
const view: View = { scale: 1, tx: 0, ty: 0 };
/** Active right-button view pan. */
let pan: { lastScreen: Vec2 } | null = null;
/** Active left-button drag of a selected element in draw/select mode. */
type LeftDrag =
  | { kind: "body" | "joint"; id: number; lastWorld: Vec2; moved: boolean }
  | { kind: "vertex"; bodyId: number; index: number; lastWorld: Vec2; moved: boolean };
let leftDrag: LeftDrag | null = null;

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
  sim: "Drag any joint to drive the mechanism.",
  select: "Click to select · drag to move · drag a selected body's corner handles to reshape · [ and ] round corners · Delete to remove.",
  body: "Empty space: click vertices to draw a polygon. Joints: click joints to build a body, click a node again to finish, then move out to set thickness and click.",
  joint: "Click inside a body to attach a joint, or empty space to place a free joint.",
  connect: "Click a joint, then another joint to pin them — or a slider line to attach the joint to it.",
  ground: "Click a joint to lock its position (it can still rotate).",
  slider: "Click two joints on the same body to create a slider rail.",
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
  }
  mode = next;
  document.querySelectorAll<HTMLButtonElement>(".mode-btn").forEach((b) =>
    b.classList.toggle("active", b.dataset.mode === mode)
  );
  toolGroup.classList.toggle("hidden", mode === "sim");
  canvas.style.cursor = mode === "sim" ? "grab" : "crosshair";
  updateHint();
}

function setTool(next: Tool): void {
  tool = next;
  resetTransient();
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
  jointDraftIds = [];
  jointDraftExpanding = false;
  selectedJoint = null;
  sliderDraftIds = [];
  selection = null;
  driver = null;
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
      if (bodies.length > 0) {
        const joints = bodies.map((b) => scene.addJoint(b.id, p));
        for (let i = 1; i < joints.length; i++) scene.addPin(joints[0].id, joints[i].id);
      } else {
        scene.addFreeJoint(p);
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
        if (rider.bodyId !== scene.getJoint(s.railA)!.bodyId) {
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
      // Two joints on the same body define a slider rail. Attach riders later via Connect.
      const j = scene.jointAt(p, pickRadius());
      if (!j) break;
      if (sliderDraftIds.length === 0) {
        sliderDraftIds = [j.id];
      } else {
        const a = scene.getJoint(sliderDraftIds[0])!;
        if (j.id === a.id) break; // same joint clicked again — ignore
        // A rail is two joints on the same (real) body; free joints can't be a rail.
        if (a.bodyId !== null && j.bodyId === a.bodyId) {
          scene.addSlider(a.id, j.id);
          placed = true;
        } else {
          sliderDraftIds = [j.id]; // different/free body — restart the rail here
        }
      }
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

/** Delete the currently selected element and its dependent features. */
function deleteSelection(): void {
  if (!selection) return;
  if (selection.kind === "body") scene.removeBody(selection.id);
  else if (selection.kind === "joint") scene.removeJoint(selection.id);
  else scene.removeConstraint(selection.id); // slider: remove it, keep the joints
  selection = null;
  markDirty();
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
    if (!j) return; // empty space: ignore
    if (jointDraftIds.includes(j.id)) {
      if (jointDraftIds.length >= 2) jointDraftExpanding = true;
    } else {
      jointDraftIds.push(j.id);
    }
    return;
  }
  if (draftBody.length > 0) {
    addBodyPoint(p); // already drawing a freehand polygon
    return;
  }
  // Fresh start: a joint begins joint-build mode; empty space begins a freehand polygon.
  const j = scene.jointAt(p, pickRadius());
  if (j) jointDraftIds = [j.id];
  else addBodyPoint(p);
}

/** Finalize a body-from-joints: margin = how far the cursor is from the last joint. */
function finalizeJointBody(p: Vec2): void {
  const lastId = jointDraftIds[jointDraftIds.length - 1];
  const last = scene.jointWorld(scene.getJoint(lastId)!);
  const margin = Math.max(JOINT_BODY_MIN_MARGIN, dist(p, last));
  scene.buildBodyFromJoints(jointDraftIds, margin);
  markDirty();
  disarmTool();
}

function addBodyPoint(p: Vec2): void {
  if (draftBody.length >= 3 && dist(p, draftBody[0]) < CLOSE_RADIUS / view.scale) {
    finishBody();
    return;
  }
  // Ignore near-duplicate points (also de-dupes the 2nd click of a double-click).
  const last = draftBody[draftBody.length - 1];
  if (last && dist(p, last) < 4 / view.scale) return;
  draftBody.push(p);
}

function finishBody(): void {
  if (draftBody.length >= 3) {
    scene.addBody(draftBody);
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
    if (tool === null) {
      // A selected body shows draggable corner handles; grabbing one reshapes the body.
      const vi = selectedBodyVertexAt(world);
      if (vi >= 0 && selection?.kind === "body") {
        leftDrag = { kind: "vertex", bodyId: selection.id, index: vi, lastWorld: world, moved: false };
        canvas.style.cursor = "move";
      } else {
        // Otherwise select what's under the cursor; if it's movable, begin a drag of it.
        handleSelectClick(world);
        if (selection && (selection.kind === "body" || selection.kind === "joint")) {
          leftDrag = { kind: selection.kind, id: selection.id, lastWorld: world, moved: false };
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

  if (leftDrag) {
    const delta = sub(world, leftDrag.lastWorld);
    if (leftDrag.kind === "vertex") scene.moveBodyVertex(leftDrag.bodyId, leftDrag.index, delta);
    else if (leftDrag.kind === "body") scene.moveBody(leftDrag.id, delta);
    else scene.moveJoint(leftDrag.id, delta);
    leftDrag.lastWorld = world;
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
  if (mode === "sim" && driver) driver.target = world;
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

canvas.addEventListener("dblclick", () => {
  // Freehand polygons still close on double-click; joint-built bodies finish by
  // clicking a previously-added node (handled in handleBodyClick).
  if (mode === "draw" && tool === "body" && jointDraftIds.length === 0) finishBody();
});

/** Draw-tool shortcuts: each tool is armed by the first letter of its name. */
const TOOL_KEYS: Record<string, Tool> = {
  b: "body",
  j: "joint",
  c: "connect",
  g: "ground",
  s: "slider",
};

window.addEventListener("keydown", (e) => {
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

// --- main loop -----------------------------------------------------------
/** Run a solve and log how long the calculation took (debug). */
function timedSolve(label: string, drv: Driver | null, iterations = 100): void {
  const t0 = performance.now();
  solve(scene, drv, iterations);
  console.log(`[Disjointed] ${label} solve: ${(performance.now() - t0).toFixed(3)} ms`);
}

/** Joints currently highlighted as in-progress tool picks. */
function activeJoints(): number[] {
  if (mode !== "draw") return [];
  if (tool === "connect") return selectedJoint !== null ? [selectedJoint] : [];
  if (tool === "slider") return sliderDraftIds;
  if (tool === "body") return jointDraftIds;
  return [];
}

/** Rail-joint positions picked so far for the slider tool, with the live cursor. */
function sliderDraftView(): { rail: Vec2[]; cursor: Vec2 } | null {
  if (mode !== "draw" || tool !== "slider" || sliderDraftIds.length === 0 || !cursor) return null;
  return { rail: sliderDraftIds.map((id) => scene.jointWorld(scene.getJoint(id)!)), cursor };
}

/** Control-vertex handles to show for the body selected in select mode (else null). */
function editVerticesView(): Vec2[] | null {
  if (mode !== "draw" || tool !== null || selection?.kind !== "body") return null;
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

function frame(): void {
  if (mode === "sim" && driver) timedSolve("drive", driver);
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
  });
  requestAnimationFrame(frame);
}

resize();
restoreAutosave();
pushHistory(); // seed the undo history with the initial (restored) layout
updateHint();
requestAnimationFrame(frame);
