import "./style.css";
import { Scene, SceneData } from "./model";
import { solve, Driver } from "./solver";
import { render } from "./renderer";
import { Vec2, add, dist, sub, vec } from "./geometry";
import { View, screenToWorld, zoomAt } from "./view";

type Mode = "draw" | "sim";
type Tool = "body" | "joint" | "connect" | "ground" | "slider";
/** An existing element picked in normal/select mode. */
type Selection = { kind: "body" | "joint" | "slider"; id: number };

/** Pick / close thresholds in screen (CSS) pixels — converted to world units via the view. */
const PICK_RADIUS = 12;
const CLOSE_RADIUS = 12;

const canvas = document.getElementById("scene") as HTMLCanvasElement;
const ctx = canvas.getContext("2d")!;
const hintEl = document.getElementById("hint")!;
const toolGroup = document.getElementById("tool-group")!;

const scene = new Scene();

// --- interaction state ---------------------------------------------------
let mode: Mode = "draw";
/** Armed draw tool, or null for normal/select mode. Tools disarm after one use. */
let tool: Tool | null = null;
let draftBody: Vec2[] = [];
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
/** Active right-button drag: pan the view, or translate a single body. */
type RightDrag =
  | { kind: "pan"; lastScreen: Vec2 }
  | { kind: "body"; bodyId: number; lastWorld: Vec2 };
let rightDrag: RightDrag | null = null;

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
  select: "Click a body or joint to select it · Delete to remove it · pick a tool to add elements.",
  body: "Click to add vertices · click the first point or double-click to close · Esc to cancel.",
  joint: "Click inside a body to attach a joint point.",
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
function markDirty(): void {
  clearTimeout(autosaveTimer);
  autosaveTimer = window.setTimeout(() => {
    try {
      localStorage.setItem(AUTOSAVE_KEY, JSON.stringify(canonicalData()));
    } catch {
      /* storage unavailable / full — ignore */
    }
  }, 300);
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
      addBodyPoint(p);
      break;
    case "joint": {
      // Place a joint in every overlapping body and pin them all together, so a
      // click in a shared zone creates a revolute connection between those bodies.
      const bodies = scene.bodiesAt(p);
      if (bodies.length > 0) {
        const joints = bodies.map((b) => scene.addJoint(b.id, p));
        for (let i = 1; i < joints.length; i++) scene.addPin(joints[0].id, joints[i].id);
        placed = true;
      }
      break;
    }
    case "connect": {
      const j = scene.jointAt(p, pickRadius());
      if (selectedJoint === null) {
        // First pick: the joint to connect.
        if (j) selectedJoint = j.id;
        break;
      }
      // Second pick: another joint → pin them; or a slider line → attach as a rider.
      if (j) {
        if (j.id !== selectedJoint) {
          const a = scene.getJoint(selectedJoint)!;
          if (a.bodyId !== j.bodyId) {
            scene.addPin(selectedJoint, j.id);
            placed = true;
          }
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
        if (j.bodyId === a.bodyId) {
          scene.addSlider(a.id, j.id);
          placed = true;
        } else {
          sliderDraftIds = [j.id]; // different body — restart the rail here
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

/** Delete the currently selected element and its dependent features. */
function deleteSelection(): void {
  if (!selection) return;
  if (selection.kind === "body") scene.removeBody(selection.id);
  else if (selection.kind === "joint") scene.removeJoint(selection.id);
  else scene.removeConstraint(selection.id); // slider: remove it, keep the joints
  selection = null;
  markDirty();
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
    // Right button: move a body if one is under the cursor, otherwise pan.
    e.preventDefault();
    const body = scene.bodyAt(world);
    rightDrag = body
      ? { kind: "body", bodyId: body.id, lastWorld: world }
      : { kind: "pan", lastScreen: eventScreen(e) };
    canvas.style.cursor = body ? "move" : "grabbing";
    return;
  }

  if (e.button !== 0) return;
  if (mode === "draw") {
    if (tool === null) handleSelectClick(world);
    else handleDrawClick(world);
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

  if (rightDrag) {
    if (rightDrag.kind === "pan") {
      const s = eventScreen(e);
      view.tx += s.x - rightDrag.lastScreen.x;
      view.ty += s.y - rightDrag.lastScreen.y;
      rightDrag.lastScreen = s;
    } else {
      const delta = sub(world, rightDrag.lastWorld);
      if (mode === "draw") {
        // Draw mode: persistent move — drag the ground anchors along with the body.
        scene.moveBody(rightDrag.bodyId, delta);
      } else {
        // Simulate mode: non-destructive nudge — only the pose moves (restored on exit).
        const body = scene.getBody(rightDrag.bodyId);
        if (body) body.pos = add(body.pos, delta);
      }
      rightDrag.lastWorld = world;
    }
    return;
  }

  hoverJoint = scene.jointAt(world, pickRadius())?.id ?? null;
  // In normal/select mode, also highlight the body under the cursor (when no joint is).
  hoverBody =
    mode === "draw" && tool === null && hoverJoint === null
      ? scene.bodyAt(world)?.id ?? null
      : null;
  if (mode === "sim" && driver) driver.target = world;
});

window.addEventListener("mouseup", (e) => {
  if (e.button === 2 && rightDrag) {
    // Persist a body relocation (but not view panning, and not sim-only moves).
    if (rightDrag.kind === "body" && mode === "draw") markDirty();
    rightDrag = null;
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
  if (mode === "draw" && tool === "body") finishBody();
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
  return [];
}

/** Rail-joint positions picked so far for the slider tool, with the live cursor. */
function sliderDraftView(): { rail: Vec2[]; cursor: Vec2 } | null {
  if (mode !== "draw" || tool !== "slider" || sliderDraftIds.length === 0 || !cursor) return null;
  return { rail: sliderDraftIds.map((id) => scene.jointWorld(scene.getJoint(id)!)), cursor };
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
    sliderDraft: sliderDraftView(),
    driverJoint: driver?.jointId ?? null,
  });
  requestAnimationFrame(frame);
}

resize();
restoreAutosave();
updateHint();
requestAnimationFrame(frame);
