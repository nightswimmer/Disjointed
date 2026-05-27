/** Canvas rendering of the scene plus transient editor/sim overlays. */
import { Scene } from "./model";
import { Vec2, add, scale, normalize, sub } from "./geometry";

export interface RenderInput {
  scene: Scene;
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

const JOINT_R = 6;

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
  const { scene } = input;
  const canvas = ctx.canvas;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;

  ctx.clearRect(0, 0, w, h);
  drawGrid(ctx, w, h);

  // Bodies.
  for (const body of scene.bodies) {
    const verts = scene.bodyWorldVerts(body);
    ctx.beginPath();
    verts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    ctx.closePath();
    ctx.fillStyle = body.color + "33";
    ctx.fill();
    ctx.strokeStyle = body.color;
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  // Slider rails (drawn under joints).
  ctx.lineWidth = 1.5;
  for (const c of scene.constraints) {
    if (c.kind !== "slider") continue;
    drawRail(ctx, c.origin, c.dir, w, h);
  }

  // In-progress slider line.
  if (input.sliderDraft) {
    const dir = normalize(sub(input.sliderDraft.cursor, input.sliderDraft.joint));
    if (dir.x !== 0 || dir.y !== 0) {
      ctx.setLineDash([6, 4]);
      drawRail(ctx, input.sliderDraft.joint, dir, w, h);
      ctx.setLineDash([]);
    }
  }

  // Draft polygon being drawn.
  if (input.draftPolygon && input.draftPolygon.length > 0) {
    const pts = input.draftPolygon;
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    if (input.cursor) ctx.lineTo(input.cursor.x, input.cursor.y);
    ctx.stroke();
    ctx.setLineDash([]);
    for (const p of pts) dot(ctx, p, 3, "#ffffff");
  }

  // Ground anchors.
  for (const c of scene.constraints) {
    if (c.kind === "ground") drawGroundSymbol(ctx, c.anchor);
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

    const r = isHover || isSelected || isDriver ? JOINT_R + 2 : JOINT_R;
    dot(ctx, p, r, fill);
    ctx.lineWidth = 2;
    ctx.strokeStyle = isDriver ? "#ff4d4d" : isSelected ? "#ffffff" : "#1e1f24";
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.stroke();
    // Hollow center marks a revolute pin.
    if (roles.pinned.has(j.id)) dot(ctx, p, 2, "#1e1f24");
  }
}

function drawGrid(ctx: CanvasRenderingContext2D, w: number, h: number): void {
  const step = 40;
  ctx.strokeStyle = "#26282f";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = 0; x <= w; x += step) {
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
  }
  for (let y = 0; y <= h; y += step) {
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
  }
  ctx.stroke();
}

function dot(ctx: CanvasRenderingContext2D, p: Vec2, r: number, color: string): void {
  ctx.beginPath();
  ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

/** Draw an infinite-ish line clipped to the viewport. */
function drawRail(ctx: CanvasRenderingContext2D, origin: Vec2, dir: Vec2, w: number, h: number): void {
  const span = Math.hypot(w, h);
  const a = add(origin, scale(dir, span));
  const b = add(origin, scale(dir, -span));
  ctx.strokeStyle = "#5bd6a6";
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
}

function drawGroundSymbol(ctx: CanvasRenderingContext2D, p: Vec2): void {
  ctx.strokeStyle = "#ffd166";
  ctx.lineWidth = 2;
  const halfW = 12;
  ctx.beginPath();
  ctx.moveTo(p.x - halfW, p.y + 8);
  ctx.lineTo(p.x + halfW, p.y + 8);
  ctx.stroke();
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  for (let i = -halfW; i <= halfW; i += 6) {
    ctx.moveTo(p.x + i, p.y + 8);
    ctx.lineTo(p.x + i - 6, p.y + 16);
  }
  ctx.stroke();
}
