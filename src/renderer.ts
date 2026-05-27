/** Canvas rendering of the scene plus transient editor/sim overlays. */
import { Scene } from "./model";
import { Vec2, add, scale, normalize, sub } from "./geometry";
import { View } from "./view";

export interface RenderInput {
  scene: Scene;
  view: View;
  mode: "draw" | "sim";
  draftPolygon: Vec2[] | null;
  cursor: Vec2 | null;
  hoverJoint: number | null;
  /** First joint picked for the connect/slider tools. */
  selectedJoint: number | null;
  /** While defining a slider line: the joint position and current cursor. */
  sliderDraft: { joint: Vec2; cursor: Vec2 } | null;
  /** Joint currently being dragged in simulation. */
  driverJoint: number | null;
}

/** On-screen joint radius in CSS pixels (kept constant regardless of zoom). */
const JOINT_R = 6;
const GRID_STEP = 40; // world units

interface JointRoles {
  pinned: Set<number>;
  grounded: Set<number>;
  slider: Set<number>;
}

function collectRoles(scene: Scene): JointRoles {
  const roles: JointRoles = { pinned: new Set(), grounded: new Set(), slider: new Set() };
  for (const c of scene.constraints) {
    if (c.kind === "pin") {
      roles.pinned.add(c.jointA);
      roles.pinned.add(c.jointB);
    } else if (c.kind === "ground") {
      roles.grounded.add(c.joint);
    } else if (c.kind === "slider") {
      roles.slider.add(c.joint);
    }
  }
  return roles;
}

export function render(ctx: CanvasRenderingContext2D, input: RenderInput): void {
  const { scene, view } = input;
  const dpr = window.devicePixelRatio || 1;
  const w = ctx.canvas.clientWidth;
  const h = ctx.canvas.clientHeight;
  const s = view.scale;
  // Cosmetic sizes are authored in screen px; divide by scale to keep them constant.
  const px = (n: number) => n / s;

  // Clear in device space, then switch to the world transform for everything else.
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.setTransform(dpr * s, 0, 0, dpr * s, dpr * view.tx, dpr * view.ty);

  // Visible world rectangle (for the grid and infinite rails).
  const left = -view.tx / s;
  const top = -view.ty / s;
  const right = (w - view.tx) / s;
  const bottom = (h - view.ty) / s;

  drawGrid(ctx, left, top, right, bottom, px(1));

  // Bodies.
  for (const body of scene.bodies) {
    const verts = scene.bodyWorldVerts(body);
    ctx.beginPath();
    verts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    ctx.closePath();
    ctx.fillStyle = body.color + "33";
    ctx.fill();
    ctx.strokeStyle = body.color;
    ctx.lineWidth = px(2);
    ctx.stroke();
  }

  const span = Math.hypot(right - left, bottom - top);

  // Slider rails (drawn under joints).
  ctx.lineWidth = px(1.5);
  for (const c of scene.constraints) {
    if (c.kind !== "slider") continue;
    drawRail(ctx, c.origin, c.dir, span);
  }

  // In-progress slider line.
  if (input.sliderDraft) {
    const dir = normalize(sub(input.sliderDraft.cursor, input.sliderDraft.joint));
    if (dir.x !== 0 || dir.y !== 0) {
      ctx.setLineDash([px(6), px(4)]);
      drawRail(ctx, input.sliderDraft.joint, dir, span);
      ctx.setLineDash([]);
    }
  }

  // Draft polygon being drawn.
  if (input.draftPolygon && input.draftPolygon.length > 0) {
    const pts = input.draftPolygon;
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = px(1.5);
    ctx.setLineDash([px(5), px(4)]);
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    if (input.cursor) ctx.lineTo(input.cursor.x, input.cursor.y);
    ctx.stroke();
    ctx.setLineDash([]);
    for (const p of pts) dot(ctx, p, px(3), "#ffffff");
  }

  // Ground anchors.
  for (const c of scene.constraints) {
    if (c.kind === "ground") drawGroundSymbol(ctx, c.anchor, s);
  }

  // Joints.
  const roles = collectRoles(scene);
  for (const j of scene.joints) {
    const p = scene.jointWorld(j);
    const isHover = input.hoverJoint === j.id;
    const isSelected = input.selectedJoint === j.id;
    const isDriver = input.driverJoint === j.id;

    let fill = "#e6e8ee";
    if (roles.pinned.has(j.id)) fill = "#4f9dff";
    if (roles.slider.has(j.id)) fill = "#5bd6a6";
    if (roles.grounded.has(j.id)) fill = "#ffd166";

    const r = px(isHover || isSelected || isDriver ? JOINT_R + 2 : JOINT_R);
    dot(ctx, p, r, fill);
    ctx.lineWidth = px(2);
    ctx.strokeStyle = isDriver ? "#ff4d4d" : isSelected ? "#ffffff" : "#1e1f24";
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.stroke();
    // Hollow center marks a revolute pin.
    if (roles.pinned.has(j.id)) dot(ctx, p, px(2), "#1e1f24");
  }
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  left: number,
  top: number,
  right: number,
  bottom: number,
  lineWidth: number
): void {
  ctx.strokeStyle = "#26282f";
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  for (let x = Math.floor(left / GRID_STEP) * GRID_STEP; x <= right; x += GRID_STEP) {
    ctx.moveTo(x, top);
    ctx.lineTo(x, bottom);
  }
  for (let y = Math.floor(top / GRID_STEP) * GRID_STEP; y <= bottom; y += GRID_STEP) {
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
  }
  ctx.stroke();
}

function dot(ctx: CanvasRenderingContext2D, p: Vec2, r: number, color: string): void {
  ctx.beginPath();
  ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

/** Draw an infinite-ish line clipped to the visible world span. */
function drawRail(ctx: CanvasRenderingContext2D, origin: Vec2, dir: Vec2, span: number): void {
  const a = add(origin, scale(dir, span));
  const b = add(origin, scale(dir, -span));
  ctx.strokeStyle = "#5bd6a6";
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
}

function drawGroundSymbol(ctx: CanvasRenderingContext2D, p: Vec2, s: number): void {
  const px = (n: number) => n / s;
  ctx.strokeStyle = "#ffd166";
  ctx.lineWidth = px(2);
  const halfW = px(12);
  const base = px(8);
  const tick = px(6);
  const depth = px(8);
  ctx.beginPath();
  ctx.moveTo(p.x - halfW, p.y + base);
  ctx.lineTo(p.x + halfW, p.y + base);
  ctx.stroke();
  ctx.lineWidth = px(1.5);
  ctx.beginPath();
  for (let i = -halfW; i <= halfW; i += tick) {
    ctx.moveTo(p.x + i, p.y + base);
    ctx.lineTo(p.x + i - tick, p.y + base + depth);
  }
  ctx.stroke();
}
