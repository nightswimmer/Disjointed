/** Canvas rendering of the scene plus transient editor/sim overlays. */
import { Scene, MeasureInfo, ResolvedMeasureRef, SketchConstraintKind } from "./model";
import { Vec2, sub, distToSegment, normalize, scale, convexHull } from "./geometry";
import { View } from "./view";
import { ConstraintBreak } from "./solver";

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
  selection: { kind: "body" | "joint" | "slider" | "measure" | "sketch" | "guide"; id: number } | null;
  /** Draw-mode multi-selection (Ctrl+click / box select): bodies + free joints, highlighted. */
  multiSelected: { bodies: number[]; joints: number[] } | null;
  /** In-progress box selection: the rectangle's two world corners, or null. */
  marquee: { a: Vec2; b: Vec2 } | null;
  /** Resolved measurements of the current mode (values update live in sim). */
  measurements: MeasureInfo[];
  /**
   * Measure-tool state: references picked so far (highlighted), the reference the
   * cursor would pick next, and the live preview once both references are chosen.
   */
  measureDraft: {
    refs: ResolvedMeasureRef[];
    hover: ResolvedMeasureRef | null;
    preview: MeasureInfo | null;
  } | null;
  /** Sketch-constraint badges to draw (draw mode only; positions resolved by main). */
  sketchGlyphs: SketchGlyphView[];
  /** Constraint-tool state: references picked so far and the one under the cursor. */
  sketchDraft: { refs: ResolvedMeasureRef[]; hover: ResolvedMeasureRef | null } | null;
  /** Ids of sketch constraints / dimensions flashing red after a rejected edit. */
  flash: Set<number> | null;
  /** Control-vertex handles to draw for the selected body (draggable to reshape it). */
  editVertices: Vec2[] | null;
  /**
   * While defining a slider: the world positions of the rail joints picked so far
   * (1 → previewing toward the cursor; 2 → rail set, awaiting the riding joint).
   */
  sliderDraft: { rail: Vec2[]; cursor: Vec2 } | null;
  /** Guide tool: the first defining point placed (line previews toward the cursor). */
  guideDraft: { a: Vec2; cursor: Vec2 } | null;
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
  /** Unsatisfiable constraints (impossible assembly): red dotted lines between points that can't meet. */
  breaks: ConstraintBreak[];
  /** Structural colour palette (light/dark). */
  theme: Theme;
}

/**
 * Theme-dependent canvas colours. Only the structural tones flip between light and dark;
 * the semantic accents (pin blue, slider/rail green, ground/rotate yellow, error red) and
 * per-body colours read fine on either background and stay hardcoded below.
 */
export interface Theme {
  /** High-contrast "ink": selection highlights, draft outlines, edit-handle fill. */
  ink: string;
  /** Background-matching tone: joint rings, pin centres, edit-handle outline. */
  surface: string;
  /** World-grid line colour. */
  grid: string;
  /** Default (roleless) joint fill. */
  jointFill: string;
}

export const DARK_THEME: Theme = {
  ink: "#ffffff",
  surface: "#1e1f24",
  grid: "#26282f",
  jointFill: "#e6e8ee",
};

export const LIGHT_THEME: Theme = {
  ink: "#1f2329",
  surface: "#f4f5f7",
  grid: "#d9dce2",
  jointFill: "#3a3d46",
};

/** One sketch constraint's on-canvas badges (world positions, computed by main). */
export interface SketchGlyphView {
  id: number;
  kind: SketchConstraintKind;
  badges: Vec2[];
  /**
   * Badges are faded by default and fully visible only while the cursor is over one of
   * the constraint's referenced elements (or a badge itself) — computed by main.
   */
  faded: boolean;
}

/** On-screen joint radius in CSS pixels (kept constant regardless of zoom). */
const JOINT_R = 6;

interface JointRoles {
  pinned: Set<number>;
  grounded: Set<number>;
  slider: Set<number>; // joints that ride a rail
  rail: Set<number>; // joints that define a rail
  /** Joints that are a linear actuator's rider (self-driving in animation). */
  actuator: Set<number>;
  /** Joints used as a motor's pivot (stationary in animation). */
  motorPivot: Set<number>;
  /** Joints used as a motor's crank pin (orbits the pivot in animation). */
  motorCrank: Set<number>;
}

function collectRoles(scene: Scene): JointRoles {
  const roles: JointRoles = {
    pinned: new Set(),
    grounded: new Set(),
    slider: new Set(),
    rail: new Set(),
    actuator: new Set(),
    motorPivot: new Set(),
    motorCrank: new Set(),
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
    } else if (c.kind === "linearActuator") {
      roles.actuator.add(c.riderId);
    } else if (c.kind === "motor") {
      roles.motorPivot.add(c.pivotJointId);
      roles.motorCrank.add(c.crankJointId);
    }
  }
  return roles;
}

export function render(ctx: CanvasRenderingContext2D, input: RenderInput): void {
  const { scene, view, theme } = input;
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

  if (input.gridVisible) drawGrid(ctx, left, top, right, bottom, px(1), input.gridStep, theme.grid);

  // Construction guidelines (draw mode only): infinite dash-dot lines drawn under the
  // geometry, with small handles on the two defining points (drag one to re-aim the line).
  if (input.mode === "draw") {
    const selectedGuide = input.selection?.kind === "guide" ? input.selection.id : null;
    for (const g of scene.guides) {
      const sel = g.id === selectedGuide;
      const color = sel ? theme.ink : GUIDE_COLOR;
      drawGuideLine(ctx, g.a, g.b, left, top, right, bottom, px, color, sel);
      dot(ctx, g.a, px(sel ? 4.5 : 3.5), color);
      dot(ctx, g.b, px(sel ? 4.5 : 3.5), color);
    }
    if (input.guideDraft) {
      const { a, cursor } = input.guideDraft;
      drawGuideLine(ctx, a, cursor, left, top, right, bottom, px, GUIDE_COLOR, false);
      dot(ctx, a, px(3.5), theme.ink);
    }
  }

  // Bodies.
  const selectedBody =
    input.selection?.kind === "body" ? input.selection.id : null;
  const multiBodies = new Set(input.multiSelected?.bodies ?? []);
  const multiJoints = new Set(input.multiSelected?.joints ?? []);
  for (const body of scene.bodies) {
    const verts = scene.bodyWorldVerts(body);
    const holes = scene.bodyHolesWorld(body);
    const isSelected = body.id === selectedBody || multiBodies.has(body.id);
    const isHover = body.id === input.hoverBody;
    // Outer outline + hole loops as subpaths: even-odd fill leaves the holes empty.
    ctx.beginPath();
    verts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    ctx.closePath();
    for (const loop of holes) {
      loop.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
      ctx.closePath();
    }
    ctx.fillStyle = body.color + (isSelected ? "55" : isHover ? "44" : "33");
    ctx.fill("evenodd");
    ctx.strokeStyle = isSelected ? theme.ink : body.color;
    ctx.lineWidth = px(isSelected || isHover ? 3 : 2);
    ctx.stroke(); // strokes every subpath, so hole rims get the outline too
  }

  // Permanent groups: a faint dashed convex hull around a group's members, drawn only
  // while the group is selected (groups are selection-atomic, so any selected member
  // means the whole group is).
  for (const g of scene.groups) {
    if (!g.bodyIds.some((id) => multiBodies.has(id))) continue;
    const pts: Vec2[] = [];
    for (const id of g.bodyIds) {
      const body = scene.getBody(id);
      if (body) pts.push(...scene.bodyWorldVerts(body));
    }
    if (pts.length < 3) continue;
    const hull = convexHull(pts);
    ctx.save();
    ctx.globalAlpha = 0.45;
    ctx.strokeStyle = theme.ink;
    ctx.lineWidth = px(1.2);
    ctx.setLineDash([px(6), px(5)]);
    ctx.beginPath();
    hull.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
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
      sel ? theme.ink : "#5bd6a6",
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
    ctx.strokeStyle = theme.ink;
    ctx.lineWidth = px(1.5);
    ctx.setLineDash([px(5), px(4)]);
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    if (input.cursor) ctx.lineTo(input.cursor.x, input.cursor.y);
    ctx.stroke();
    ctx.setLineDash([]);
    for (const p of pts) dot(ctx, p, px(3), theme.ink);
  }

  // Body-from-joints: dashed outline through the picked joints, and the expanded preview.
  if (input.bodyJointDraft) {
    const { outline, preview } = input.bodyJointDraft;
    ctx.setLineDash([px(5), px(4)]);
    if (!preview) {
      // Still picking joints: a dashed path through them, trailing to the cursor.
      ctx.strokeStyle = theme.ink;
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
    for (const p of outline) dot(ctx, p, px(3), theme.ink);
  }

  // Ground anchors.
  for (const c of scene.constraints) {
    if (c.kind === "ground") drawGroundSymbol(ctx, c.anchor, s);
  }
  // Grounded bodies: the same ground symbol at the body's centroid (every member of a
  // grounded group carries the flag, so each shows its own symbol).
  for (const body of scene.bodies) {
    if (body.grounded) drawGroundSymbol(ctx, body.pos, s);
  }

  // Motors: a thin yellow arm from pivot to crank (the rotation arm) plus a curved
  // arrow at the pivot indicating that side spins. Drawn before the joints so the
  // joint dots sit on top.
  for (const c of scene.constraints) {
    if (c.kind !== "motor") continue;
    const jp = scene.getJoint(c.pivotJointId);
    const jc = scene.getJoint(c.crankJointId);
    if (!jp || !jc) continue;
    const pp = scene.jointWorld(jp);
    const pc = scene.jointWorld(jc);
    ctx.strokeStyle = "#ffd166";
    ctx.lineWidth = px(1.5);
    ctx.setLineDash([px(5), px(3)]);
    ctx.beginPath();
    ctx.moveTo(pp.x, pp.y);
    ctx.lineTo(pc.x, pc.y);
    ctx.stroke();
    ctx.setLineDash([]);
    // Curved-arrow rotation badge centred on the pivot.
    const r = px(11);
    ctx.lineWidth = px(1.6);
    ctx.beginPath();
    ctx.arc(pp.x, pp.y, r, -Math.PI * 0.65, Math.PI * 0.65);
    ctx.stroke();
    // Tiny arrowhead at the open end of the arc, pointing along the rotation direction.
    const ah = px(4);
    const ang = Math.PI * 0.65;
    const tip = { x: pp.x + r * Math.cos(ang), y: pp.y + r * Math.sin(ang) };
    const tx = -Math.sin(ang); // tangent direction
    const ty = Math.cos(ang);
    ctx.beginPath();
    ctx.moveTo(tip.x, tip.y);
    ctx.lineTo(tip.x - tx * ah - Math.cos(ang) * ah * 0.5, tip.y - ty * ah - Math.sin(ang) * ah * 0.5);
    ctx.moveTo(tip.x, tip.y);
    ctx.lineTo(tip.x - tx * ah + Math.cos(ang) * ah * 0.5, tip.y - ty * ah + Math.sin(ang) * ah * 0.5);
    ctx.stroke();
  }

  // Draw mode: connections aren't solved here, so a constraint whose endpoints sit apart
  // gets a dotted connector — it reads as linked even though the points don't touch.
  if (input.mode === "draw") {
    ctx.lineWidth = px(1.5);
    ctx.setLineDash([px(4), px(4)]);
    // Pins (blue): line between the two joints when their dots don't overlap.
    ctx.strokeStyle = "#4f9dff";
    const pinTouch = px(2 * JOINT_R); // centres closer than two radii → the dots overlap
    for (const c of scene.constraints) {
      if (c.kind !== "pin") continue;
      const ja = scene.getJoint(c.jointA);
      const jb = scene.getJoint(c.jointB);
      if (!ja || !jb) continue;
      const pa = scene.jointWorld(ja);
      const pb = scene.jointWorld(jb);
      if (Math.hypot(pb.x - pa.x, pb.y - pa.y) <= pinTouch) continue;
      ctx.beginPath();
      ctx.moveTo(pa.x, pa.y);
      ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
    }
    // Sliders (green): line from each rider to the rail's midpoint when the rider is off
    // the rail. The rider's dot sitting on the rail line counts as touching.
    ctx.strokeStyle = "#5bd6a6";
    const railTouch = px(JOINT_R);
    for (const c of scene.constraints) {
      if (c.kind !== "slider") continue;
      const ra = scene.getJoint(c.railA);
      const rb = scene.getJoint(c.railB);
      if (!ra || !rb) continue;
      const a = scene.jointWorld(ra);
      const b = scene.jointWorld(rb);
      const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
      for (const riderId of c.riders) {
        const rj = scene.getJoint(riderId);
        if (!rj) continue;
        const p = scene.jointWorld(rj);
        if (distToSegment(p, a, b) <= railTouch) continue; // already on the rail
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(mid.x, mid.y);
        ctx.stroke();
      }
    }
    ctx.setLineDash([]);
  }

  // Joints.
  const selectedJointId =
    input.selection?.kind === "joint" ? input.selection.id : null;
  const roles = collectRoles(scene);
  // Joints involved in any unsatisfiable constraint — painted red to flag the stuck points.
  const brokenJoints = new Set<number>();
  for (const b of input.breaks) for (const id of b.joints) brokenJoints.add(id);
  for (const j of scene.joints) {
    const p = scene.jointWorld(j);
    const isHover = input.hoverJoint === j.id;
    const isSelected =
      input.activeJoints.includes(j.id) || j.id === selectedJointId || multiJoints.has(j.id);
    const isDriver = input.driverJoint === j.id;
    const isBroken = brokenJoints.has(j.id);
    // A body-less joint reads as "loose" (muted dashed ring) only while unconstrained;
    // once it rides a slider or defines a (grounded) rail it's anchored, so it renders
    // like any constrained joint rather than a loose point.
    const isFree = j.bodyId === null && !roles.slider.has(j.id) && !roles.rail.has(j.id);

    let fill = theme.jointFill;
    if (roles.pinned.has(j.id)) fill = "#4f9dff";
    if (roles.slider.has(j.id)) fill = "#5bd6a6";
    if (roles.grounded.has(j.id)) fill = "#ffd166";
    if (isBroken) fill = "#ff4d4d"; // unsatisfiable constraint endpoint — overrides role colour

    const r = px(isHover || isSelected || isDriver || isBroken ? JOINT_R + 2 : JOINT_R);
    dot(ctx, p, r, fill);
    ctx.lineWidth = px(isBroken ? 2.5 : 2);
    ctx.strokeStyle = isBroken
      ? "#ff4d4d" // unsatisfiable constraint endpoint — red ring matches the break line
      : isDriver
      ? "#ff4d4d"
      : isSelected
      ? theme.ink
      : roles.rail.has(j.id)
      ? "#5bd6a6" // rail-defining joints get a green ring
      : isFree
      ? "#9aa0ac" // free joints get a muted dashed ring
      : theme.surface;
    // A dashed outline marks a free (body-less) joint — but a broken joint uses a solid red ring.
    if (isFree && !isSelected && !isDriver && !isBroken) ctx.setLineDash([px(3), px(3)]);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    // Hollow center marks a revolute pin.
    if (roles.pinned.has(j.id)) dot(ctx, p, px(2), theme.surface);
  }

  // Linear-actuator riders: a green dashed ring around the joint badges it as self-driving
  // along the rail. Drawn after the joints so the badge ring sits on top of the joint dot.
  if (roles.actuator.size > 0) {
    ctx.strokeStyle = "#5bd6a6";
    ctx.lineWidth = px(1.5);
    ctx.setLineDash([px(3), px(3)]);
    for (const id of roles.actuator) {
      const j = scene.getJoint(id);
      if (!j) continue;
      const p = scene.jointWorld(j);
      ctx.beginPath();
      ctx.arc(p.x, p.y, px(JOINT_R + 4), 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }

  // Impossible-assembly markers: a red dotted line between each pair of points that, given
  // the (hard) grounds and other constraints, can't be brought together.
  if (input.breaks.length > 0) {
    ctx.strokeStyle = "#ff4d4d";
    ctx.lineWidth = px(2);
    ctx.setLineDash([px(5), px(4)]);
    for (const b of input.breaks) {
      ctx.beginPath();
      ctx.moveTo(b.a.x, b.a.y);
      ctx.lineTo(b.b.x, b.b.y);
      ctx.stroke();
    }
    ctx.setLineDash([]);
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
    ctx.strokeStyle = theme.surface;
    ctx.fillStyle = theme.ink;
    for (const v of input.editVertices) {
      ctx.beginPath();
      ctx.rect(v.x - h, v.y - h, 2 * h, 2 * h);
      ctx.fill();
      ctx.stroke();
    }
  }

  // Box-selection rectangle (drawn over the geometry, under the annotations).
  if (input.marquee) {
    const { a, b } = input.marquee;
    const x = Math.min(a.x, b.x);
    const y = Math.min(a.y, b.y);
    const bw = Math.abs(b.x - a.x);
    const bh = Math.abs(b.y - a.y);
    ctx.save();
    ctx.fillStyle = theme.ink;
    ctx.globalAlpha = 0.06;
    ctx.fillRect(x, y, bw, bh);
    ctx.globalAlpha = 0.7;
    ctx.strokeStyle = theme.ink;
    ctx.lineWidth = px(1);
    ctx.setLineDash([px(5), px(4)]);
    ctx.strokeRect(x, y, bw, bh);
    ctx.restore();
  }

  // Sketch-constraint badges (draw mode; annotations, so they sit over the geometry).
  const selectedSketch = input.selection?.kind === "sketch" ? input.selection.id : null;
  for (const g of input.sketchGlyphs) {
    drawSketchGlyph(ctx, g, view, dpr, theme, g.id === selectedSketch, !!input.flash?.has(g.id));
  }
  if (input.sketchDraft) {
    const { refs, hover } = input.sketchDraft;
    if (hover) drawMeasureRefHighlight(ctx, hover, px, true, SKETCH_COLOR);
    for (const r of refs) drawMeasureRefHighlight(ctx, r, px, false, SKETCH_COLOR);
  }

  // Measurements (drawn last: dimension annotations sit on top of everything).
  const selectedMeasure = input.selection?.kind === "measure" ? input.selection.id : null;
  for (const info of input.measurements) {
    // CAD convention in draw mode: a driven (reference) dimension shows in parentheses,
    // a driving one plain. Sim-mode values are always plain read-outs.
    const paren = input.mode === "draw" && !info.driving;
    drawMeasurement(
      ctx, info, view, dpr, theme,
      info.id === selectedMeasure, false, paren, !!input.flash?.has(info.id), input.scene.unit
    );
  }
  if (input.measureDraft) {
    const { refs, hover, preview } = input.measureDraft;
    if (hover) drawMeasureRefHighlight(ctx, hover, px, true);
    for (const r of refs) drawMeasureRefHighlight(ctx, r, px, false);
    if (preview) drawMeasurement(ctx, preview, view, dpr, theme, false, true, false, false, input.scene.unit);
  }
}

/** Accent colour for measurements (fixed across themes, like the other semantic accents). */
const MEASURE_COLOR = "#46c2cb";
/** Construction guidelines: muted, CAD-centre-line grey (reads on both themes). */
const GUIDE_COLOR = "#9aa0ac";
/** Accent colour for sketch constraints (violet, distinct from every other accent). */
const SKETCH_COLOR = "#b48cff";
/** Rejected sketch edits flash the conflicting items in the error red. */
const FLASH_COLOR = "#ff4d4d";

/** Badge symbol per sketch-constraint kind (drawn in the glyph pill). */
const SKETCH_SYMBOL: Record<SketchConstraintKind, string> = {
  coincident: "◎",
  horizontal: "H",
  vertical: "V",
  parallel: "∥",
  perpendicular: "⊥",
  equal: "=",
};

/**
 * Compact value text: one decimal, trailing zero dropped; degrees get a ° suffix and
 * distances the document's working unit. `paren` wraps the value in parentheses
 * (a driven/reference dimension in draw mode).
 */
function measureText(info: MeasureInfo, paren: boolean, unit: string): string {
  const v = Math.round(info.value * 10) / 10;
  const t = info.kind === "angle" ? `${v}°` : `${v} ${unit}`;
  return paren ? `(${t})` : t;
}

/** Draw one sketch constraint's badges: constant-size pills with the kind's symbol. */
function drawSketchGlyph(
  ctx: CanvasRenderingContext2D,
  g: SketchGlyphView,
  view: View,
  dpr: number,
  theme: Theme,
  selected: boolean,
  flashed: boolean
): void {
  const s = view.scale;
  const color = flashed ? FLASH_COLOR : selected ? theme.ink : SKETCH_COLOR;
  // Faded unless the cursor is on the constrained element — selection / a reject flash
  // always shows at full strength.
  const alpha = g.faded && !selected && !flashed ? 0.2 : 1;
  for (const b of g.badges) {
    const sx = b.x * s + view.tx;
    const sy = b.y * s + view.ty;
    ctx.save();
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.globalAlpha = alpha;
    ctx.font = "11px ui-sans-serif, system-ui, sans-serif";
    const half = 8;
    ctx.beginPath();
    ctx.roundRect(sx - half, sy - half, 2 * half, 2 * half, 4);
    ctx.fillStyle = theme.surface + "e6";
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = selected || flashed ? 1.6 : 1;
    ctx.stroke();
    ctx.fillStyle = color;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(SKETCH_SYMBOL[g.kind], sx, sy + 0.5);
    ctx.restore();
  }
}

/** Highlight a measure reference: a ring around a point, a soft thick stroke over a line. */
function drawMeasureRefHighlight(
  ctx: CanvasRenderingContext2D,
  ref: ResolvedMeasureRef,
  px: (n: number) => number,
  isHover: boolean,
  color: string = MEASURE_COLOR
): void {
  ctx.strokeStyle = color;
  if (ref.kind === "point") {
    ctx.lineWidth = px(2);
    if (isHover) ctx.setLineDash([px(3), px(3)]);
    ctx.beginPath();
    ctx.arc(ref.p.x, ref.p.y, px(10), 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
  } else {
    ctx.save();
    ctx.globalAlpha = isHover ? 0.4 : 0.7;
    ctx.lineWidth = px(5);
    ctx.beginPath();
    ctx.moveTo(ref.a.x, ref.a.y);
    ctx.lineTo(ref.b.x, ref.b.y);
    ctx.stroke();
    ctx.restore();
  }
}

/** A small dimension arrowhead: tip at `tip`, wings sweeping back against `dir`. */
function drawArrowHead(
  ctx: CanvasRenderingContext2D,
  tip: Vec2,
  dir: Vec2,
  size: number
): void {
  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(tip.x - dir.x * size - dir.y * size * 0.45, tip.y - dir.y * size + dir.x * size * 0.45);
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(tip.x - dir.x * size + dir.y * size * 0.45, tip.y - dir.y * size - dir.x * size * 0.45);
  ctx.stroke();
}

/**
 * Draw one measurement: dashed extension lines, an arrowed dimension line (distance) or
 * an arc (angle), and the value label — rendered in screen space at a constant size.
 */
function drawMeasurement(
  ctx: CanvasRenderingContext2D,
  info: MeasureInfo,
  view: View,
  dpr: number,
  theme: Theme,
  selected: boolean,
  draft: boolean,
  paren: boolean,
  flashed: boolean,
  unit: string
): void {
  const s = view.scale;
  const px = (n: number) => n / s;
  const color = flashed ? FLASH_COLOR : selected ? theme.ink : MEASURE_COLOR;
  ctx.strokeStyle = color;

  // Extension / leader lines: thin and dashed.
  ctx.lineWidth = px(1);
  ctx.setLineDash([px(4), px(3)]);
  for (const e of info.ext) {
    ctx.beginPath();
    ctx.moveTo(e.a.x, e.a.y);
    ctx.lineTo(e.b.x, e.b.y);
    ctx.stroke();
  }
  ctx.setLineDash(draft ? [px(5), px(4)] : []);
  ctx.lineWidth = px(1.5);

  if (info.dim) {
    const { a, b } = info.dim;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
    ctx.setLineDash([]);
    const u = normalize(sub(a, b));
    if ((u.x !== 0 || u.y !== 0) && Math.hypot(b.x - a.x, b.y - a.y) > px(4)) {
      drawArrowHead(ctx, a, u, px(7));
      drawArrowHead(ctx, b, scale(u, -1), px(7));
    }
  }
  if (info.arc) {
    const { c, r, a0, sweep } = info.arc;
    ctx.beginPath();
    ctx.arc(c.x, c.y, r, a0, a0 + sweep);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Value label: a pill + text drawn in screen space so it stays legible at any zoom.
  const text = measureText(info, paren, unit);
  const sx = info.labelPos.x * s + view.tx;
  const sy = info.labelPos.y * s + view.ty;
  ctx.save();
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.font = "12px ui-sans-serif, system-ui, sans-serif";
  const tw = ctx.measureText(text).width;
  const pw = tw + 12;
  const ph = 18;
  ctx.beginPath();
  ctx.roundRect(sx - pw / 2, sy - ph / 2, pw, ph, 5);
  ctx.fillStyle = theme.surface + (draft ? "cc" : "e6");
  ctx.fill();
  ctx.strokeStyle = color;
  ctx.lineWidth = selected || info.driving ? 1.6 : 1; // a driving dimension reads bolder
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, sx, sy + 0.5);
  ctx.restore();
}

/**
 * Draw the **infinite** line through `a`–`b`, clipped to the visible world rect, as a
 * dash-dot construction line. The segment drawn is centred on the viewport (projection
 * of the view centre onto the line ± the viewport diagonal), so it always spans the view.
 */
function drawGuideLine(
  ctx: CanvasRenderingContext2D,
  a: Vec2,
  b: Vec2,
  left: number,
  top: number,
  right: number,
  bottom: number,
  px: (n: number) => number,
  color: string,
  selected: boolean
): void {
  const d = normalize(sub(b, a));
  if (d.x === 0 && d.y === 0) return;
  const tc = (((left + right) / 2 - a.x) * d.x + ((bottom + top) / 2 - a.y) * d.y);
  const half = Math.hypot(right - left, bottom - top);
  ctx.strokeStyle = color;
  ctx.lineWidth = px(selected ? 2 : 1.2);
  ctx.setLineDash([px(12), px(5), px(3), px(5)]);
  ctx.beginPath();
  ctx.moveTo(a.x + d.x * (tc - half), a.y + d.y * (tc - half));
  ctx.lineTo(a.x + d.x * (tc + half), a.y + d.y * (tc + half));
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawGrid(
  ctx: CanvasRenderingContext2D,
  left: number,
  top: number,
  right: number,
  bottom: number,
  lineWidth: number,
  step: number,
  color: string
): void {
  ctx.strokeStyle = color;
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
