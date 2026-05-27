/** Canvas rendering of the scene plus transient editor/sim overlays. */
import { Scene } from "./model";
import { Vec2, sub } from "./geometry";
import { View } from "./view";

export interface RenderInput {
  scene: Scene;
  view: View;
  mode: "draw" | "sim";
  draftBody: Vec2[] | null;
  cursor: Vec2 | null;
  hoverJoint: number | null;
  /** Body hovered in normal/select mode (for pre-selection feedback). */
  hoverBody: number | null;
  /** Joints highlighted as in-progress tool picks (connect's first pick, slider rail picks). */
  activeJoints: number[];
  /** The element selected in normal/select mode (highlighted, deletable). */
  selection: { kind: "body" | "joint" | "slider"; id: number } | null;
  /** Control-vertex handles to draw for the selected body (draggable to reshape it). */
  editVertices: Vec2[] | null;
  /**
   * While defining a slider: the world positions of the rail joints picked so far
   * (1 → previewing toward the cursor; 2 → rail set, awaiting the riding joint).
   */
  sliderDraft: { rail: Vec2[]; cursor: Vec2 } | null;
  /**
   * While building a body from joints: `outline` are the picked joints; `preview` is
   * the expanded body boundary once the user is sizing its margin (else null).
   */
  bodyJointDraft: { outline: Vec2[]; preview: Vec2[] | null } | null;
  /** Joint currently being dragged in simulation. */
  driverJoint: number | null;
  /** Pivot point of an in-progress rotate (drawn as a crosshair), or null. */
  rotatePivot: Vec2 | null;
  /** Spacing of the world-locked grid (and the snap increment), in world units. */
  gridStep: number;
  /** Whether to draw the world-locked grid. */
  gridVisible: boolean;
}

/** On-screen joint radius in CSS pixels (kept constant regardless of zoom). */
const JOINT_R = 6;

interface JointRoles {
  pinned: Set<number>;
  grounded: Set<number>;
  slider: Set<number>; // joints that ride a rail
  rail: Set<number>; // joints that define a rail
}

function collectRoles(scene: Scene): JointRoles {
  const roles: JointRoles = {
    pinned: new Set(),
    grounded: new Set(),
    slider: new Set(),
    rail: new Set(),
  };
  for (const c of scene.constraints) {
    if (c.kind === "pin") {
      roles.pinned.add(c.jointA);
      roles.pinned.add(c.jointB);
    } else if (c.kind === "ground") {
      roles.grounded.add(c.joint);
    } else if (c.kind === "slider") {
      for (const r of c.riders) roles.slider.add(r);
      roles.rail.add(c.railA);
      roles.rail.add(c.railB);
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

  if (input.gridVisible) drawGrid(ctx, left, top, right, bottom, px(1), input.gridStep);

  // Bodies.
  const selectedBody =
    input.selection?.kind === "body" ? input.selection.id : null;
  for (const body of scene.bodies) {
    const verts = scene.bodyWorldVerts(body);
    const isSelected = body.id === selectedBody;
    const isHover = body.id === input.hoverBody;
    ctx.beginPath();
    verts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    ctx.closePath();
    ctx.fillStyle = body.color + (isSelected ? "55" : isHover ? "44" : "33");
    ctx.fill();
    ctx.strokeStyle = isSelected ? "#ffffff" : body.color;
    ctx.lineWidth = px(isSelected || isHover ? 3 : 2);
    ctx.stroke();
  }

  // Slider rails (drawn under joints): a bounded segment between the two rail joints,
  // with end-caps marking the stops that the riding joint is clamped between.
  const selectedSlider =
    input.selection?.kind === "slider" ? input.selection.id : null;
  for (const c of scene.constraints) {
    if (c.kind !== "slider") continue;
    const ja = scene.getJoint(c.railA);
    const jb = scene.getJoint(c.railB);
    if (!ja || !jb) continue;
    const sel = c.id === selectedSlider;
    drawRailSegment(
      ctx,
      scene.jointWorld(ja),
      scene.jointWorld(jb),
      sel ? "#ffffff" : "#5bd6a6",
      px(sel ? 2.5 : 1.5),
      px(6)
    );
  }

  // In-progress slider: dashed preview of the rail segment being defined.
  if (input.sliderDraft) {
    const { rail, cursor } = input.sliderDraft;
    ctx.setLineDash([px(6), px(4)]);
    drawRailSegment(ctx, rail[0], rail.length >= 2 ? rail[1] : cursor, "#9aa0ac", px(1.5), px(6));
    ctx.setLineDash([]);
  }

  // Draft body being drawn (freehand polygon).
  if (input.draftBody && input.draftBody.length > 0) {
    const pts = input.draftBody;
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

  // Body-from-joints: dashed outline through the picked joints, and the expanded preview.
  if (input.bodyJointDraft) {
    const { outline, preview } = input.bodyJointDraft;
    ctx.setLineDash([px(5), px(4)]);
    if (!preview) {
      // Still picking joints: a dashed path through them, trailing to the cursor.
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = px(1.5);
      ctx.beginPath();
      outline.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
      if (input.cursor) ctx.lineTo(input.cursor.x, input.cursor.y);
      ctx.stroke();
    } else {
      // Sizing the margin: preview the final (expanded) boundary.
      ctx.strokeStyle = "#5bd6a6";
      ctx.lineWidth = px(2);
      ctx.beginPath();
      preview.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
      ctx.closePath();
      ctx.stroke();
    }
    ctx.setLineDash([]);
    for (const p of outline) dot(ctx, p, px(3), "#ffffff");
  }

  // Ground anchors.
  for (const c of scene.constraints) {
    if (c.kind === "ground") drawGroundSymbol(ctx, c.anchor, s);
  }

  // Joints.
  const selectedJointId =
    input.selection?.kind === "joint" ? input.selection.id : null;
  const roles = collectRoles(scene);
  for (const j of scene.joints) {
    const p = scene.jointWorld(j);
    const isHover = input.hoverJoint === j.id;
    const isSelected = input.activeJoints.includes(j.id) || j.id === selectedJointId;
    const isDriver = input.driverJoint === j.id;
    // A body-less joint reads as "loose" (muted dashed ring) only while unconstrained;
    // once it rides a slider or defines a (grounded) rail it's anchored, so it renders
    // like any constrained joint rather than a loose point.
    const isFree = j.bodyId === null && !roles.slider.has(j.id) && !roles.rail.has(j.id);

    let fill = "#e6e8ee";
    if (roles.pinned.has(j.id)) fill = "#4f9dff";
    if (roles.slider.has(j.id)) fill = "#5bd6a6";
    if (roles.grounded.has(j.id)) fill = "#ffd166";

    const r = px(isHover || isSelected || isDriver ? JOINT_R + 2 : JOINT_R);
    dot(ctx, p, r, fill);
    ctx.lineWidth = px(2);
    ctx.strokeStyle = isDriver
      ? "#ff4d4d"
      : isSelected
      ? "#ffffff"
      : roles.rail.has(j.id)
      ? "#5bd6a6" // rail-defining joints get a green ring
      : isFree
      ? "#9aa0ac" // free joints get a muted dashed ring
      : "#1e1f24";
    // A dashed outline marks a free (body-less) joint.
    if (isFree && !isSelected && !isDriver) ctx.setLineDash([px(3), px(3)]);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    // Hollow center marks a revolute pin.
    if (roles.pinned.has(j.id)) dot(ctx, p, px(2), "#1e1f24");
  }

  // Rotate pivot crosshair (drawn over everything while rotating about a point).
  if (input.rotatePivot) {
    const p = input.rotatePivot;
    const r = px(8);
    ctx.strokeStyle = "#ffd166";
    ctx.lineWidth = px(1.5);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.moveTo(p.x - r * 1.6, p.y);
    ctx.lineTo(p.x + r * 1.6, p.y);
    ctx.moveTo(p.x, p.y - r * 1.6);
    ctx.lineTo(p.x, p.y + r * 1.6);
    ctx.stroke();
  }

  // Control-vertex handles for the selected body (square = draggable corner).
  if (input.editVertices) {
    const h = px(5);
    ctx.lineWidth = px(2);
    ctx.strokeStyle = "#1e1f24";
    ctx.fillStyle = "#ffffff";
    for (const v of input.editVertices) {
      ctx.beginPath();
      ctx.rect(v.x - h, v.y - h, 2 * h, 2 * h);
      ctx.fill();
      ctx.stroke();
    }
  }
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  left: number,
  top: number,
  right: number,
  bottom: number,
  lineWidth: number,
  step: number
): void {
  ctx.strokeStyle = "#26282f";
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  for (let x = Math.floor(left / step) * step; x <= right; x += step) {
    ctx.moveTo(x, top);
    ctx.lineTo(x, bottom);
  }
  for (let y = Math.floor(top / step) * step; y <= bottom; y += step) {
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

/** Draw the bounded rail segment a→b with perpendicular end-caps marking the stops. */
function drawRailSegment(
  ctx: CanvasRenderingContext2D,
  a: Vec2,
  b: Vec2,
  color: string,
  lineWidth: number,
  cap: number
): void {
  const d = sub(b, a);
  const l = Math.hypot(d.x, d.y);
  if (l < 1e-6) return;
  const n = { x: (-d.y / l) * cap, y: (d.x / l) * cap }; // perpendicular cap offset
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.moveTo(a.x - n.x, a.y - n.y);
  ctx.lineTo(a.x + n.x, a.y + n.y);
  ctx.moveTo(b.x - n.x, b.y - n.y);
  ctx.lineTo(b.x + n.x, b.y + n.y);
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
