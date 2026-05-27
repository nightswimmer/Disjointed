import "./style.css";
import { Scene, SceneData } from "./model";
import { solve, Driver } from "./solver";
import { render } from "./renderer";
import { Vec2, add, dist, normalize, sub, vec } from "./geometry";
import { View, screenToWorld, zoomAt } from "./view";

type Mode = "draw" | "sim";
type Tool = "polygon" | "joint" | "connect" | "ground" | "slider";

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
let tool: Tool = "polygon";
let draftPolygon: Vec2[] = [];
let cursor: Vec2 | null = null; // world coordinates
let hoverJoint: number | null = null;
let selectedJoint: number | null = null; // first pick for connect/slider
let sliderDraftJoint: number | null = null;
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
const HINTS: Record<Mode | Tool, string> = {
  draw: "",
  sim: "Drag any joint to drive the mechanism.",
  polygon: "Click to add vertices · click the first point or double-click to close · Esc to cancel.",
  joint: "Click inside a body to attach a joint point.",
  connect: "Click two joints on different bodies to pin them together.",
  ground: "Click a joint to lock its position (it can still rotate).",
  slider: "Click a joint, then click again to set the direction of its rail.",
};

function updateHint(): void {
  hintEl.textContent = mode === "sim" ? HINTS.sim : HINTS[tool];
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
    solve(scene, null, 40); // settle so pins/grounds/sliders are satisfied
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

function resetTransient(): void {
  draftPolygon = [];
  selectedJoint = null;
  sliderDraftJoint = null;
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
  switch (tool) {
    case "polygon":
      addPolygonPoint(p);
      break;
    case "joint": {
      const body = scene.bodyAt(p);
      if (body) scene.addJoint(body.id, p);
      break;
    }
    case "connect": {
      const j = scene.jointAt(p, pickRadius());
      if (!j) {
        selectedJoint = null;
        break;
      }
      if (selectedJoint === null) {
        selectedJoint = j.id;
      } else if (selectedJoint !== j.id) {
        const a = scene.getJoint(selectedJoint)!;
        if (a.bodyId !== j.bodyId) scene.addPin(selectedJoint, j.id);
        selectedJoint = null;
      }
      break;
    }
    case "ground": {
      const j = scene.jointAt(p, pickRadius());
      if (j) scene.addGround(j.id, scene.jointWorld(j));
      break;
    }
    case "slider": {
      const j = scene.jointAt(p, pickRadius());
      if (sliderDraftJoint === null) {
        if (j) sliderDraftJoint = j.id;
      } else {
        const jt = scene.getJoint(sliderDraftJoint)!;
        const origin = scene.jointWorld(jt);
        const dir = normalize(sub(p, origin));
        if (dir.x !== 0 || dir.y !== 0) scene.addSlider(jt.id, origin, dir);
        sliderDraftJoint = null;
      }
      break;
    }
  }
  markDirty();
}

function addPolygonPoint(p: Vec2): void {
  if (draftPolygon.length >= 3 && dist(p, draftPolygon[0]) < CLOSE_RADIUS / view.scale) {
    finishPolygon();
    return;
  }
  // Ignore near-duplicate points (also de-dupes the 2nd click of a double-click).
  const last = draftPolygon[draftPolygon.length - 1];
  if (last && dist(p, last) < 4 / view.scale) return;
  draftPolygon.push(p);
}

function finishPolygon(): void {
  if (draftPolygon.length >= 3) {
    scene.addBody(draftPolygon);
    markDirty();
  }
  draftPolygon = [];
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
    handleDrawClick(world);
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
      const body = scene.getBody(rightDrag.bodyId);
      if (body) body.pos = add(body.pos, sub(world, rightDrag.lastWorld));
      rightDrag.lastWorld = world;
    }
    return;
  }

  hoverJoint = scene.jointAt(world, pickRadius())?.id ?? null;
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
  if (mode === "draw" && tool === "polygon") finishPolygon();
});

window.addEventListener("keydown", (e) => {
  if (e.key === "Escape") resetTransient();
  else if (e.key === "Enter" && mode === "draw" && tool === "polygon") finishPolygon();
});

// --- main loop -----------------------------------------------------------
function frame(): void {
  if (mode === "sim" && driver) solve(scene, driver);
  render(ctx, {
    scene,
    view,
    mode,
    draftPolygon: mode === "draw" && tool === "polygon" ? draftPolygon : null,
    cursor,
    hoverJoint,
    selectedJoint,
    sliderDraft:
      sliderDraftJoint !== null && cursor
        ? { joint: scene.jointWorld(scene.getJoint(sliderDraftJoint)!), cursor }
        : null,
    driverJoint: driver?.jointId ?? null,
  });
  requestAnimationFrame(frame);
}

resize();
restoreAutosave();
updateHint();
requestAnimationFrame(frame);
