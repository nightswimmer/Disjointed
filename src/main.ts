import "./style.css";
import { Scene } from "./model";
import { solve, Driver } from "./solver";
import { render } from "./renderer";
import { Vec2, dist, normalize, sub, vec } from "./geometry";

type Mode = "draw" | "sim";
type Tool = "polygon" | "joint" | "connect" | "ground" | "slider";

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
let cursor: Vec2 | null = null;
let hoverJoint: number | null = null;
let selectedJoint: number | null = null; // first pick for connect/slider
let sliderDraftJoint: number | null = null;
let driver: Driver | null = null;
/** Body poses saved when entering simulation, restored when leaving. */
let savedPoses: Map<number, { pos: Vec2; angle: number }> | null = null;

// --- canvas sizing -------------------------------------------------------
function resize(): void {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(canvas.clientWidth * dpr);
  canvas.height = Math.floor(canvas.clientHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener("resize", resize);

function eventPos(e: MouseEvent): Vec2 {
  const rect = canvas.getBoundingClientRect();
  return vec(e.clientX - rect.left, e.clientY - rect.top);
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
      const j = scene.jointAt(p, PICK_RADIUS);
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
      const j = scene.jointAt(p, PICK_RADIUS);
      if (j) scene.addGround(j.id, scene.jointWorld(j));
      break;
    }
    case "slider": {
      const j = scene.jointAt(p, PICK_RADIUS);
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
}

function addPolygonPoint(p: Vec2): void {
  if (draftPolygon.length >= 3 && dist(p, draftPolygon[0]) < CLOSE_RADIUS) {
    finishPolygon();
    return;
  }
  // Ignore near-duplicate points (also de-dupes the 2nd click of a double-click).
  const last = draftPolygon[draftPolygon.length - 1];
  if (last && dist(p, last) < 4) return;
  draftPolygon.push(p);
}

function finishPolygon(): void {
  if (draftPolygon.length >= 3) scene.addBody(draftPolygon);
  draftPolygon = [];
}

// --- pointer events ------------------------------------------------------
canvas.addEventListener("mousedown", (e) => {
  const p = eventPos(e);
  cursor = p;
  if (mode === "draw") {
    handleDrawClick(p);
  } else {
    const j = scene.jointAt(p, PICK_RADIUS);
    if (j) {
      driver = { jointId: j.id, target: p };
      canvas.style.cursor = "grabbing";
    }
  }
});

canvas.addEventListener("mousemove", (e) => {
  const p = eventPos(e);
  cursor = p;
  hoverJoint = scene.jointAt(p, PICK_RADIUS)?.id ?? null;
  if (mode === "sim" && driver) driver.target = p;
});

window.addEventListener("mouseup", () => {
  if (mode === "sim" && driver) {
    driver = null;
    canvas.style.cursor = "grab";
  }
});

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
updateHint();
requestAnimationFrame(frame);
