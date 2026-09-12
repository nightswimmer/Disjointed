/** Canvas rendering of the scene plus transient editor/sim overlays. */
import {
  Scene,
  Body,
  Joint,
  ComponentInstance,
  ComponentOccurrence,
  MeasureInfo,
  MeasureHighlight,
  ResolvedMeasureRef,
  SketchConstraintKind,
  Guide,
} from "./model";
import { Vec2, add, sub, vec, dist, distToSegment, normalize, scale, convexHull, Arc } from "./geometry";
import { View, viewMatrix, visibleWorldRect, worldToScreen, rotateToScreen } from "./view";
import { ConstraintBreak } from "./solver";

export interface RenderInput {
  scene: Scene;
  view: View;
  mode: "draw" | "sim";
  /** Split tool: the cut path drawn so far (dashed polyline to the cursor). */
  draftBody: Vec2[] | null;
  /**
   * Shape tools (draw mode): the shape being drawn, styled by its **role** — Body (tint
   * of the colour it will get), Cut (hatched, with the target body highlighted) or
   * Reference (construction dash-dot). `outline` is the preview geometry so far (already
   * sampled for circles / arcs / slots), closed or open; `points` are the placed
   * defining points; `aux` extra construction segments (a radius line, a slot axis);
   * `text` a label being placed. Present whenever a shape tool is armed (the role badge
   * follows the cursor even before the first click).
   */
  shapeDraft: {
    role: "body" | "cut" | "reference";
    fill: string;
    outline: Vec2[];
    closed: boolean;
    points: Vec2[];
    aux: [Vec2, Vec2][];
    text: { p: Vec2; size: number; text: string } | null;
    /** The body a Cut will subtract from, once known. */
    target: number | null;
  } | null;
  /**
   * Pattern tool: the seed hole's outline (or, with no seed yet, the hole under the
   * cursor — `anchor` null), the seed anchor, the linear target / circular centre being
   * picked or picked, and every instance after the seed with whether it fits.
   */
  patternPreview: {
    kind: "linear" | "circular";
    anchor: Vec2 | null;
    seedLoop: Vec2[] | null;
    target: Vec2 | null;
    instances: { point: Vec2; loop: Vec2[] | null; ok: boolean }[];
  } | null;
  cursor: Vec2 | null;
  hoverJoint: number | null;
  /** Body hovered in normal/select mode (for pre-selection feedback). */
  hoverBody: number | null;
  /** Occurrences of the component hovered in the component browser (direct instances and
   *  ones nested inside other instances): the rest of the picture fades, these draw on
   *  top with the hover tint plus a dashed hull each. */
  highlightOccurrences: ComponentOccurrence[] | null;
  /**
   * Context ghost while a component definition is edited: the enclosing contexts'
   * material (scratch scenes, already in this definition's frame — see context.ts),
   * drawn faded under everything and never interactive. Empty at the root / when off.
   */
  ghostScenes: Scene[];
  /** Joints highlighted as in-progress tool picks (connect's first pick, rail picks). */
  activeJoints: number[];
  /** The element selected in normal/select mode (highlighted, deletable). */
  selection: { kind: "body" | "joint" | "rail" | "measure" | "sketch" | "guide" | "pattern"; id: number } | null;
  /** Live patterns' on-canvas layout labels / handles (draw mode; computed by main). */
  patterns: PatternView[];
  /** Draw-mode multi-selection (Ctrl+click / box select): bodies + free joints, highlighted. */
  multiSelected: { bodies: number[]; joints: number[] } | null;
  /** In-progress box selection: the rectangle's two world corners, or null. */
  marquee: { a: Vec2; b: Vec2 } | null;
  /**
   * View-rotation overlay (screen space, both modes): the dial's centre and radius in
   * screen px, whether a drag is turning it, and whether the pointer is over its grabbable
   * parts. The angle itself is `view.angle`; the numeric readout is a DOM input.
   */
  viewRotate: { centre: Vec2; radius: number; dragging: boolean; hot: boolean } | null;
  /** Draw-mode feature selection within the selected body (Shift+drag box): the selected
   *  control-vertex handles' positions (drawn filled in the selection accent) and the
   *  selected joints' ids (ringed like a selected joint). */
  featureSelected: { vertices: Vec2[]; joints: number[] } | null;
  /** Resolved measurements of the current mode (values update live in sim). */
  measurements: MeasureInfo[];
  /**
   * Measure-tool state: references picked so far (highlighted), the reference the
   * cursor would pick next, and the live preview once both references are chosen.
   */
  measureDraft: {
    refs: MeasureHighlight[];
    hover: MeasureHighlight | null;
    preview: MeasureInfo | null;
  } | null;
  /** Sketch-constraint badges to draw (draw mode only; positions resolved by main). */
  sketchGlyphs: SketchGlyphView[];
  /** Constraint-tool state: references picked so far and the one under the cursor. */
  sketchDraft: { refs: ResolvedMeasureRef[]; hover: ResolvedMeasureRef | null } | null;
  /**
   * Object-snap highlight (draw mode): the dragged object's snapping reference (solid)
   * and, while snapped, the target feature it landed on (dashed; a line target is
   * extended across the view when `hitInfinite`). Before a drag, just the reference
   * a drag from the cursor would use.
   */
  dragSnap: { ref: ResolvedMeasureRef; hit: ResolvedMeasureRef | null; hitInfinite: boolean } | null;
  /** Implicit-constraint preview during a drag: the armed alignment candidate and the
   *  dragged reference (sketch violet), plus the alignment a release would constrain —
   *  a dotted line with the constraint's badge. Null while no candidate is armed. */
  dragAlign: {
    ref: ResolvedMeasureRef;
    cand: ResolvedMeasureRef;
    match: { kind: SketchConstraintKind; from: Vec2; to: Vec2 } | null;
  } | null;
  /** Ids of sketch constraints / dimensions flashing red after a rejected edit. */
  flash: Set<number> | null;
  /** Control-vertex handles to draw for the selected body (draggable to reshape it). */
  editVertices: Vec2[] | null;
  /** Per-corner radius handles for the selected body (circle = drag to round that corner). */
  filletHandles: Vec2[] | null;
  /**
   * While defining a rail: the world positions of the rail joints picked so far
   * (1 → previewing toward the cursor; 2 → rail set, awaiting the riding joint).
   */
  railDraft: { rail: Vec2[]; cursor: Vec2 } | null;
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
  /**
   * Draw mode: attached joints stranded outside their body's outline (a component edit
   * or vertex removal reshaped the body under them) — red fill + dashed red ring, vs
   * the solid red ring of a sim break.
   */
  containmentErrors: Set<number>;
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

/**
 * A pattern's canvas overlay (world positions, computed by main): per linear axis a
 * dimension-style line beside the axis with its count and spacing labels and the
 * re-aim handle at the last instance; for a circular layout the centre (also the handle),
 * the orbit radius and the count / angle / rotation labels. `bad` marks members that
 * don't fit (outside the body, overlapping another hole or joint).
 */
export interface PatternView {
  id: number;
  selected: boolean;
  anchor: Vec2;
  axes: {
    /** The dotted axis line: seed anchor → last instance (also the constraint reference). */
    line: { a: Vec2; b: Vec2 };
    /** The arrowed spacing dimension beside the first step (seed → 2nd instance). */
    dim: { a: Vec2; b: Vec2 };
    ext: { a: Vec2; b: Vec2 }[];
    stepLabel: Vec2;
    stepText: string;
    countLabel: Vec2;
    countText: string;
    handle: Vec2;
  }[];
  circular: {
    centre: Vec2;
    radius: number;
    countLabel: Vec2;
    countText: string;
    angleLabel: Vec2;
    angleText: string;
    rotateLabel: Vec2;
    rotateText: string;
  } | null;
  bad: Vec2[];
}

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
  /** A pose constraint (on component instances) that currently fails to hold — drawn
   *  in the error style until re-applied, like a violated driving dimension. */
  violated?: boolean;
  /**
   * Set on the one constraint whose badge is under the cursor: its referenced elements
   * (highlighted in the sketch violet) and, when those elements don't touch, the shortest
   * segment between them (drawn dotted) — computed by main.
   */
  hover?: { refs: ResolvedMeasureRef[]; link: [Vec2, Vec2] | null };
}

/** On-screen joint radius in CSS pixels (kept constant regardless of zoom). */
const JOINT_R = 6;
/** Length (px) of a rail arrow's head wings — long enough to show past a joint dot. */
const RAIL_HEAD = 13;

interface JointRoles {
  pinned: Set<number>;
  /** Joints of a rigid pin (a weld — no relative rotation). Subset of `pinned`. */
  welded: Set<number>;
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
    welded: new Set(),
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
      if (c.rigid === true) {
        roles.welded.add(c.jointA);
        roles.welded.add(c.jointB);
      }
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
  ctx.setTransform(...viewMatrix(view, dpr));

  // Visible world rectangle (for the grid and infinite rails): the world-axis bounding
  // box of the screen — a superset of what shows when the view is rotated.
  const viewRect = visibleWorldRect(view, w, h);
  const { left, top, right, bottom } = viewRect;

  if (input.gridVisible) drawGrid(ctx, left, top, right, bottom, px(1), input.gridStep, theme.grid);

  // Construction geometry (draw mode only): infinite guidelines, reference polylines /
  // circles / arcs and text labels, dash-dot under the geometry, with crosshair handles on
  // their defining points (drag one to reshape; a circle's rim handle shows when selected).
  if (input.mode === "draw") {
    const selectedGuide = input.selection?.kind === "guide" ? input.selection.id : null;
    for (const g of scene.guides) {
      const sel = g.id === selectedGuide;
      drawGuide(ctx, scene, g, left, top, right, bottom, px, sel ? theme.ink : GUIDE_COLOR, sel);
    }
    if (input.guideDraft) {
      const { a, cursor } = input.guideDraft;
      drawGuideLine(ctx, a, cursor, left, top, right, bottom, px, GUIDE_COLOR, false);
      crosshair(ctx, a, px, theme.ink, false);
    }
  }

  // Context ghost: the enclosing assembly, faded, under the live definition. Bodies as
  // thin outlines with a whisper of fill, joints as hollow rings, rails as thin arrows —
  // enough to align to, muted enough never to be mistaken for editable material.
  for (const g of input.ghostScenes) drawGhostScene(ctx, g, px, theme);

  // Bodies.
  const selectedBody =
    input.selection?.kind === "body" ? input.selection.id : null;
  const multiBodies = new Set(input.multiSelected?.bodies ?? []);
  const multiJoints = new Set(input.multiSelected?.joints ?? []);
  const featureJoints = new Set(input.featureSelected?.joints ?? []);
  const highlightBodies = new Set<number>();
  for (const occ of input.highlightOccurrences ?? []) for (const id of occ.bodyIds) highlightBodies.add(id);
  const drawBodyShape = (body: Body): void => {
    const verts = scene.bodyWorldVerts(body);
    const holes = scene.bodyHolesWorld(body);
    const isSelected = body.id === selectedBody || multiBodies.has(body.id);
    const isHover = body.id === input.hoverBody || highlightBodies.has(body.id) || body.id === input.shapeDraft?.target;
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
  };
  for (const body of scene.bodies) drawBodyShape(body);

  // Permanent groups: a faint dashed convex hull around a group's members (bodies +
  // locked free joints), drawn only while the group is selected (groups are
  // selection-atomic, so any selected member means the whole group is). Groups owned by
  // a component instance (its chassis, or groups recreated from the definition) are
  // skipped — the instance draws one hull for all of its material instead.
  const instanceGroupIds = new Set<number>();
  for (const inst of scene.instances) {
    if (inst.groupId !== null) instanceGroupIds.add(inst.groupId);
    for (const e of inst.groupMap) instanceGroupIds.add(e.id);
  }
  const drawHull = (pts: Vec2[]): void => {
    if (pts.length < 3) return;
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
  };
  for (const g of scene.groups) {
    if (instanceGroupIds.has(g.id)) continue;
    if (!g.bodyIds.some((id) => multiBodies.has(id)) && !g.jointIds.some((id) => multiJoints.has(id))) continue;
    const pts: Vec2[] = [];
    for (const id of g.bodyIds) {
      const body = scene.getBody(id);
      if (body) pts.push(...scene.bodyWorldVerts(body));
    }
    for (const id of g.jointIds) {
      const j = scene.getJoint(id);
      if (j) pts.push(scene.jointWorld(j));
    }
    drawHull(pts);
  }
  // Selected component instances (and the ones highlighted from the component browser):
  // one dashed hull around everything the instance expanded.
  const drawMaterialHull = (bodyIds: Iterable<number>, jointIds: Iterable<number>): void => {
    const pts: Vec2[] = [];
    for (const id of bodyIds) {
      const body = scene.getBody(id);
      if (body) pts.push(...scene.bodyWorldVerts(body));
    }
    for (const id of jointIds) {
      const j = scene.getJoint(id);
      if (j && j.bodyId === null) pts.push(scene.jointWorld(j));
    }
    drawHull(pts);
  };
  const drawInstanceHull = (inst: ComponentInstance): void =>
    drawMaterialHull(
      inst.bodyMap.map((e) => e.id),
      [...inst.jointMap, ...inst.anchorMap].map((e) => e.id)
    );
  for (const inst of scene.instances) {
    const selected =
      inst.bodyMap.some((e) => multiBodies.has(e.id)) ||
      inst.jointMap.some((e) => multiJoints.has(e.id)) ||
      inst.anchorMap.some((e) => multiJoints.has(e.id));
    if (selected) drawInstanceHull(inst);
  }

  // Rails (drawn under joints): a double-headed arrow between the two rail joints — the
  // range a rider travels, its heads at the stops the rider is clamped between.
  const selectedRail =
    input.selection?.kind === "rail" ? input.selection.id : null;
  for (const c of scene.constraints) {
    if (c.kind !== "slider") continue;
    const ja = scene.getJoint(c.railA);
    const jb = scene.getJoint(c.railB);
    if (!ja || !jb) continue;
    const a = scene.jointWorld(ja);
    const b = scene.jointWorld(jb);
    const sel = c.id === selectedRail;
    drawRailArrow(ctx, a, b, sel ? theme.ink : "#5bd6a6", px(sel ? 2.5 : 1.5), px(RAIL_HEAD));
    // Orientation-locked riders (prismatic sliders) get a rail-aligned carriage
    // rectangle on top of the rail so they read differently from pin-in-slot riders.
    for (const riderId of c.locked) {
      const rj = scene.getJoint(riderId);
      if (!rj) continue;
      drawCarriage(ctx, scene.jointWorld(rj), sub(b, a), px(10), px(6), "#5bd6a6", px(1.5));
    }
  }

  // In-progress rail / slider: dashed preview of the arrow being defined.
  if (input.railDraft) {
    const { rail, cursor } = input.railDraft;
    ctx.setLineDash([px(6), px(4)]);
    drawRailArrow(ctx, rail[0], rail.length >= 2 ? rail[1] : cursor, "#9aa0ac", px(1.5), px(RAIL_HEAD));
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

  // Shape being drawn, in its role's style (see RenderInput.shapeDraft).
  if (input.shapeDraft) drawShapeDraft(ctx, input.shapeDraft, px, theme);

  // Pattern tool: seed highlight, the direction line / rotation centre, and every instance
  // (dashed; red where it would leave the body or overlap another hole / joint).
  if (input.patternPreview) {
    const pv = input.patternPreview;
    const loopPath = (loop: Vec2[]): void => {
      ctx.beginPath();
      loop.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
      ctx.closePath();
    };
    if (pv.anchor === null) {
      // Hovering a candidate hole: a faint dashed outline says "this one is pickable".
      if (pv.seedLoop) {
        ctx.strokeStyle = theme.ink;
        ctx.globalAlpha = 0.55;
        ctx.lineWidth = px(1.5);
        ctx.setLineDash([px(4), px(3)]);
        loopPath(pv.seedLoop);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
      }
    } else {
      // The seed: solid outline (hole) or a ring (joint).
      ctx.strokeStyle = theme.ink;
      ctx.lineWidth = px(2);
      if (pv.seedLoop) {
        loopPath(pv.seedLoop);
        ctx.stroke();
      } else {
        ctx.beginPath();
        ctx.arc(pv.anchor.x, pv.anchor.y, px(JOINT_R + 3), 0, Math.PI * 2);
        ctx.stroke();
      }
      if (pv.target) {
        ctx.lineWidth = px(1);
        ctx.setLineDash([px(4), px(4)]);
        if (pv.kind === "linear") {
          // Direction line from the seed to the picked point.
          ctx.strokeStyle = theme.ink;
          ctx.beginPath();
          ctx.moveTo(pv.anchor.x, pv.anchor.y);
          ctx.lineTo(pv.target.x, pv.target.y);
          ctx.stroke();
        } else {
          // The orbit through the seed, and the centre as a crosshair (same look as the
          // rotate pivot — it is a rotation centre).
          const c = pv.target;
          ctx.strokeStyle = theme.ink;
          ctx.beginPath();
          ctx.arc(c.x, c.y, dist(c, pv.anchor), 0, Math.PI * 2);
          ctx.stroke();
          ctx.setLineDash([]);
          const r = px(8);
          ctx.strokeStyle = "#ffd166";
          ctx.lineWidth = px(1.5);
          ctx.beginPath();
          ctx.arc(c.x, c.y, r, 0, Math.PI * 2);
          ctx.moveTo(c.x - r * 1.6, c.y);
          ctx.lineTo(c.x + r * 1.6, c.y);
          ctx.moveTo(c.x, c.y - r * 1.6);
          ctx.lineTo(c.x, c.y + r * 1.6);
          ctx.stroke();
        }
        ctx.setLineDash([]);
      }
      // The instances.
      ctx.lineWidth = px(1.5);
      ctx.setLineDash([px(5), px(4)]);
      for (const inst of pv.instances) {
        ctx.strokeStyle = inst.ok ? theme.ink : "#ff5c5c";
        if (inst.loop) loopPath(inst.loop);
        else {
          ctx.beginPath();
          ctx.arc(inst.point.x, inst.point.y, px(JOINT_R), 0, Math.PI * 2);
        }
        ctx.stroke();
      }
      ctx.setLineDash([]);
    }
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
  const drawJoint = (j: Joint): void => {
    const p = scene.jointWorld(j);
    const isHover = input.hoverJoint === j.id;
    const isSelected =
      input.activeJoints.includes(j.id) || j.id === selectedJointId || multiJoints.has(j.id) || featureJoints.has(j.id);
    const isDriver = input.driverJoint === j.id;
    const isBroken = brokenJoints.has(j.id);
    // Stranded outside its body's outline (draw mode) — same error red as a break,
    // but with a dashed ring so the two states read differently.
    const isOutside = !isBroken && input.containmentErrors.has(j.id);
    // A body-less joint reads as "loose" (muted dashed ring) only while unconstrained;
    // once it rides a slider, defines a (grounded) rail, or is locked to a group (a
    // component chassis point) it's anchored, so it renders like any constrained joint.
    const isFree =
      j.bodyId === null &&
      !roles.slider.has(j.id) &&
      !roles.rail.has(j.id) &&
      !scene.groupOfJoint(j.id);

    let fill = theme.jointFill;
    if (roles.pinned.has(j.id)) fill = "#4f9dff";
    if (roles.slider.has(j.id)) fill = "#5bd6a6";
    if (roles.grounded.has(j.id)) fill = "#ffd166";
    if (isBroken || isOutside) fill = "#ff4d4d"; // error red — overrides role colour

    const r = px(isHover || isSelected || isDriver || isBroken || isOutside ? JOINT_R + 2 : JOINT_R);
    dot(ctx, p, r, fill);
    ctx.lineWidth = px(isBroken || isOutside ? 2.5 : 2);
    ctx.strokeStyle = isBroken || isOutside
      ? "#ff4d4d" // error red — a break's ring is solid, a containment error's dashed
      : isDriver
      ? "#ff4d4d"
      : isSelected
      ? theme.ink
      : roles.rail.has(j.id)
      ? "#5bd6a6" // rail-defining joints get a green ring
      : isFree
      ? "#9aa0ac" // free joints get a muted dashed ring
      : theme.surface;
    // A dashed outline marks a free (body-less) joint — a broken joint uses a solid red
    // ring, and a containment error a dashed red one.
    if ((isFree && !isSelected && !isDriver && !isBroken) || isOutside)
      ctx.setLineDash([px(3), px(3)]);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    // Hollow center marks a revolute pin; a weld (rigid pin) gets a square center
    // instead — no rotation, no circle — plus a square outline echoing it.
    if (roles.welded.has(j.id)) {
      const c = px(2.2);
      ctx.fillStyle = theme.surface;
      ctx.fillRect(p.x - c, p.y - c, c * 2, c * 2);
      const h = px(JOINT_R + 3.5);
      ctx.strokeStyle = "#4f9dff";
      ctx.lineWidth = px(1.5);
      ctx.strokeRect(p.x - h, p.y - h, h * 2, h * 2);
    } else if (roles.pinned.has(j.id)) dot(ctx, p, px(2), theme.surface);
  };
  for (const j of scene.joints) drawJoint(j);

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

  // Feature selection: the selected vertex handles redrawn filled in the selection
  // accent (over the plain ink squares), slightly larger so they read as "selected".
  if (input.featureSelected && input.featureSelected.vertices.length) {
    const h = px(6);
    ctx.lineWidth = px(2);
    ctx.strokeStyle = theme.ink;
    ctx.fillStyle = FEATURE_SEL_COLOR;
    for (const v of input.featureSelected.vertices) {
      ctx.beginPath();
      ctx.rect(v.x - h, v.y - h, 2 * h, 2 * h);
      ctx.fill();
      ctx.stroke();
    }
  }

  // Per-corner radius handles (circle = drag along the corner's bisector to round it).
  // Inverted colours vs the vertex squares so the two handle kinds read apart.
  if (input.filletHandles) {
    const r = px(4.5);
    ctx.lineWidth = px(2);
    ctx.strokeStyle = theme.ink;
    ctx.fillStyle = theme.surface;
    for (const h of input.filletHandles) {
      ctx.beginPath();
      ctx.arc(h.x, h.y, r, 0, Math.PI * 2);
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
  // Badge hover: light up the constraint's elements and, when they sit apart, join them
  // with a dotted line so the relationship reads at a glance.
  for (const g of input.sketchGlyphs) {
    if (!g.hover) continue;
    for (const r of g.hover.refs) drawMeasureRefHighlight(ctx, r, px, false, viewRect, SKETCH_COLOR);
    if (g.hover.link) {
      const [from, to] = g.hover.link;
      ctx.save();
      ctx.strokeStyle = SKETCH_COLOR;
      ctx.lineWidth = px(1.5);
      ctx.setLineDash([px(3), px(4)]);
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();
      ctx.restore();
    }
  }
  if (input.sketchDraft) {
    const { refs, hover } = input.sketchDraft;
    if (hover) drawMeasureRefHighlight(ctx, hover, px, true, viewRect, SKETCH_COLOR);
    for (const r of refs) drawMeasureRefHighlight(ctx, r, px, false, viewRect, SKETCH_COLOR);
  }
  // Object snap: the target it snapped onto (dashed, a line target extended when it acts
  // as an infinite line), then the dragged reference itself on top.
  if (input.dragSnap) {
    const { ref, hit, hitInfinite } = input.dragSnap;
    if (hit) {
      if (hit.kind === "line" && hitInfinite) {
        drawGuideLine(ctx, hit.a, hit.b, left, top, right, bottom, px, OSNAP_COLOR, false);
      }
      drawMeasureRefHighlight(ctx, hit, px, true, viewRect, OSNAP_COLOR);
    }
    drawMeasureRefHighlight(ctx, ref, px, false, viewRect, OSNAP_COLOR);
  }
  // Implicit constraints: the armed alignment candidate and the dragged reference in the
  // sketch violet, then the previewed alignment — a dotted line carrying the badge of the
  // constraint a release would create.
  if (input.dragAlign) {
    const { ref, cand, match } = input.dragAlign;
    drawMeasureRefHighlight(ctx, cand, px, false, viewRect, SKETCH_COLOR);
    drawMeasureRefHighlight(ctx, ref, px, false, viewRect, SKETCH_COLOR);
    if (match) {
      const span = dist(match.from, match.to);
      if (span > px(1)) {
        ctx.strokeStyle = SKETCH_COLOR;
        ctx.lineWidth = px(1.5);
        ctx.setLineDash([px(3), px(4)]);
        ctx.beginPath();
        ctx.moveTo(match.from.x, match.from.y);
        ctx.lineTo(match.to.x, match.to.y);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      // The badge sits at the dotted line's midpoint, or beside the point when the line is
      // too short to host it (a point already within a candidate line's span).
      const at = span > px(24) ? scale(add(match.from, match.to), 0.5) : add(match.to, vec(px(14), -px(14)));
      drawSketchBadge(ctx, at, match.kind, view, dpr, theme, SKETCH_COLOR, 1, true);
    }
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
  // Live patterns: axis dimension lines with count / spacing labels, or the centre with
  // its labels; handles when selected; members that don't fit ringed in red.
  for (const pv of input.patterns) drawPattern(ctx, pv, view, dpr, theme);

  if (input.measureDraft) {
    const { refs, hover, preview } = input.measureDraft;
    if (hover) drawMeasureRefHighlight(ctx, hover, px, true, viewRect);
    for (const r of refs) drawMeasureRefHighlight(ctx, r, px, false, viewRect);
    if (preview) drawMeasurement(ctx, preview, view, dpr, theme, false, true, false, false, input.scene.unit);
  }

  // Focus pass for the component browser's hover: veil the whole picture towards the
  // background (everything else drops to ~20% strength), then redraw the highlighted
  // occurrences — their bodies, hulls and joints — on top at full strength, so they
  // stand out even in a crowded drawing. Occurrences nested inside other instances get
  // their own hull, drawn around just their material.
  if (input.highlightOccurrences && highlightBodies.size > 0) {
    ctx.save();
    ctx.globalAlpha = FOCUS_VEIL_ALPHA;
    ctx.fillStyle = theme.surface;
    ctx.fillRect(left, top, right - left, bottom - top);
    ctx.restore();
    for (const body of scene.bodies) if (highlightBodies.has(body.id)) drawBodyShape(body);
    const focusJoints = new Set<number>();
    for (const occ of input.highlightOccurrences) {
      drawMaterialHull(occ.bodyIds, occ.jointIds);
      for (const id of occ.jointIds) focusJoints.add(id);
    }
    // Joints attached to a highlighted body from outside the instance belong to the
    // picture too (an assembly-level pin placed on an instance part).
    for (const j of scene.joints) {
      if (focusJoints.has(j.id) || (j.bodyId !== null && highlightBodies.has(j.bodyId))) drawJoint(j);
    }
  }

  // Shape tools: the active role rides beside the cursor (screen space), so the mode
  // the next shape will take is visible right where the eye is.
  if (input.shapeDraft && input.cursor && input.mode === "draw") {
    drawRoleBadge(ctx, worldToScreen(view, input.cursor), input.shapeDraft.role, dpr, theme);
  }

  // View-rotation dial: drawn last, in screen space, over everything.
  if (input.viewRotate) drawViewRotateDial(ctx, input.viewRotate, view, dpr, theme);
}

/** Cut-role accent (a warm "material removed" tone; reads on both themes). */
const CUT_COLOR = "#e5643a";

const ROLE_LABEL = { body: "Body", cut: "Cut", reference: "Ref" } as const;

/** The role badge: a small pill with the role's name, offset down-right of the cursor. */
function drawRoleBadge(
  ctx: CanvasRenderingContext2D,
  at: Vec2,
  role: "body" | "cut" | "reference",
  dpr: number,
  theme: Theme
): void {
  ctx.save();
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const label = ROLE_LABEL[role];
  ctx.font = "600 11px system-ui, sans-serif";
  ctx.textBaseline = "middle";
  ctx.textAlign = "left";
  const w = ctx.measureText(label).width + 12;
  const h = 18;
  const x = at.x + 14;
  const y = at.y + 14;
  const color = role === "cut" ? CUT_COLOR : role === "reference" ? GUIDE_COLOR : theme.ink;
  ctx.fillStyle = theme.surface + "e6";
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.roundRect(x, y, w, h, 9);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.fillText(label, x + 6, y + h / 2 + 0.5);
  ctx.restore();
}

/**
 * The shape being drawn. Body: dashed ink outline over a faint tint of the colour the
 * body will get. Cut: dashed accent outline over diagonal hatching (material to remove).
 * Reference: construction dash-dot with crosshair points, like a finished guide.
 */
function drawShapeDraft(
  ctx: CanvasRenderingContext2D,
  d: NonNullable<RenderInput["shapeDraft"]>,
  px: (n: number) => number,
  theme: Theme
): void {
  const color = d.role === "cut" ? CUT_COLOR : d.role === "reference" ? GUIDE_COLOR : theme.ink;
  const path = (): void => {
    ctx.beginPath();
    d.outline.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    if (d.closed && d.outline.length > 2) ctx.closePath();
  };
  if (d.outline.length >= 2) {
    if (d.closed && d.outline.length > 2) {
      if (d.role === "body") {
        path();
        ctx.fillStyle = d.fill + "33";
        ctx.fill();
      } else if (d.role === "cut") {
        // Diagonal hatching clipped to the shape: the material that will be removed.
        ctx.save();
        path();
        ctx.clip();
        let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
        for (const p of d.outline) {
          minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
          minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
        }
        const step = px(7);
        const span = maxX - minX + (maxY - minY);
        ctx.strokeStyle = CUT_COLOR + "88";
        ctx.lineWidth = px(1);
        ctx.setLineDash([]);
        ctx.beginPath();
        for (let k = 0; k <= span; k += step) {
          ctx.moveTo(minX + k, minY);
          ctx.lineTo(minX, minY + k);
        }
        ctx.stroke();
        ctx.restore();
      }
    }
    path();
    ctx.strokeStyle = color;
    ctx.lineWidth = px(d.role === "reference" ? 1.2 : 1.5);
    ctx.setLineDash(d.role === "reference" ? [px(12), px(5), px(3), px(5)] : [px(5), px(4)]);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  if (d.aux.length) {
    ctx.strokeStyle = color;
    ctx.lineWidth = px(1);
    ctx.setLineDash([px(3), px(3)]);
    ctx.beginPath();
    for (const [a, b] of d.aux) {
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }
  for (const p of d.points) {
    if (d.role === "reference") crosshair(ctx, p, px, color, false);
    else dot(ctx, p, px(3), color);
  }
  if (d.text) drawGuideText(ctx, d.text.p, 0, d.text.text, d.text.size, color);
}

/** One reference element in its dash-dot style, with crosshairs on its defining points. */
function drawGuide(
  ctx: CanvasRenderingContext2D,
  scene: Scene,
  g: Guide,
  left: number,
  top: number,
  right: number,
  bottom: number,
  px: (n: number) => number,
  color: string,
  selected: boolean
): void {
  const dash = (): void => {
    ctx.strokeStyle = color;
    ctx.lineWidth = px(selected ? 2 : 1.2);
    ctx.setLineDash([px(12), px(5), px(3), px(5)]);
  };
  switch (g.kind) {
    case "line":
      drawGuideLine(ctx, g.a, g.b, left, top, right, bottom, px, color, selected);
      crosshair(ctx, g.a, px, color, selected);
      crosshair(ctx, g.b, px, color, selected);
      return;
    case "poly":
      dash();
      ctx.beginPath();
      g.pts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
      if (g.closed) ctx.closePath();
      ctx.stroke();
      ctx.setLineDash([]);
      for (const p of g.pts) crosshair(ctx, p, px, color, selected);
      return;
    case "circle":
      dash();
      ctx.beginPath();
      ctx.arc(g.c.x, g.c.y, g.r, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
      crosshair(ctx, g.c, px, color, selected);
      if (selected) {
        // The rim handle (drag to resize) shows only while the circle is selected.
        const rim = scene.guidePointWorld(g, "r");
        if (rim) squareHandle(ctx, rim, px(4), color);
      }
      return;
    case "arc": {
      const arc = scene.guideArc(g);
      dash();
      ctx.beginPath();
      if (arc) drawArcPath(ctx, arc);
      else {
        ctx.moveTo(g.a.x, g.a.y);
        ctx.lineTo(g.m.x, g.m.y);
        ctx.lineTo(g.b.x, g.b.y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      crosshair(ctx, g.a, px, color, selected);
      crosshair(ctx, g.m, px, color, selected);
      crosshair(ctx, g.b, px, color, selected);
      return;
    }
    case "text": {
      const at = scene.guideTextWorld(g);
      if (!at) return;
      drawGuideText(ctx, at.p, at.angle, g.text, g.size, color);
      if (selected) crosshair(ctx, at.p, px, color, true);
      return;
    }
  }
}

/** Append an arc (centre / radius / start / signed sweep) to the current path. */
function drawArcPath(ctx: CanvasRenderingContext2D, arc: Arc): void {
  ctx.arc(arc.c.x, arc.c.y, arc.r, arc.a0, arc.a0 + arc.sweep, arc.sweep < 0);
}

/** A text label: baseline at `p`, `size` world units tall, turned by `angle` (its body's). */
function drawGuideText(
  ctx: CanvasRenderingContext2D,
  p: Vec2,
  angle: number,
  text: string,
  size: number,
  color: string
): void {
  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.rotate(angle);
  ctx.font = `${size}px system-ui, sans-serif`;
  ctx.textBaseline = "alphabetic";
  ctx.textAlign = "left";
  ctx.fillStyle = color;
  ctx.fillText(text, 0, 0);
  ctx.restore();
}

/** A small hollow square handle (a guide's resize grip). */
function squareHandle(ctx: CanvasRenderingContext2D, p: Vec2, half: number, color: string): void {
  ctx.strokeStyle = color;
  ctx.lineWidth = half / 3;
  ctx.setLineDash([]);
  ctx.strokeRect(p.x - half, p.y - half, half * 2, half * 2);
}

const VIEW_ROTATE_COLOR = "#46c2cb";
/**
 * The view-rotation dial: a big ring ticked every 5° (longer every 45°), a full-size
 * crosshair along the world x / y axes (so it reads as the picture's H / V) with X / Y
 * labels at the arm ends, and a centre dot. The ring and the arms are the grab handles.
 */
function drawViewRotateDial(
  ctx: CanvasRenderingContext2D,
  d: { centre: Vec2; radius: number; dragging: boolean; hot: boolean },
  view: View,
  dpr: number,
  theme: Theme
): void {
  const { centre: c, radius: r } = d;
  const w = ctx.canvas.clientWidth;
  const h = ctx.canvas.clientHeight;
  const armLen = Math.hypot(w, h); // long enough to cross the whole canvas
  ctx.save();
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.lineCap = "round";
  const strong = d.dragging || d.hot;

  // Crosshair along the world axes.
  const ex = rotateToScreen({ x: 1, y: 0 }, view.angle); // world +x on screen
  const ey = rotateToScreen({ x: 0, y: 1 }, view.angle); // world +y on screen (down at 0°)
  ctx.strokeStyle = VIEW_ROTATE_COLOR;
  ctx.globalAlpha = strong ? 0.95 : 0.7;
  ctx.lineWidth = strong ? 2 : 1.5;
  ctx.beginPath();
  ctx.moveTo(c.x - ex.x * armLen, c.y - ex.y * armLen);
  ctx.lineTo(c.x + ex.x * armLen, c.y + ex.y * armLen);
  ctx.moveTo(c.x - ey.x * armLen, c.y - ey.y * armLen);
  ctx.lineTo(c.x + ey.x * armLen, c.y + ey.y * armLen);
  ctx.stroke();

  // Ring + ticks.
  ctx.globalAlpha = strong ? 0.95 : 0.75;
  ctx.lineWidth = strong ? 2.5 : 2;
  ctx.beginPath();
  ctx.arc(c.x, c.y, r, 0, Math.PI * 2);
  ctx.stroke();
  ctx.lineWidth = 1;
  ctx.globalAlpha = 0.6;
  ctx.beginPath();
  for (let deg = 0; deg < 360; deg += 5) {
    const a = (deg * Math.PI) / 180;
    const len = deg % 90 === 0 ? 16 : deg % 45 === 0 ? 11 : 5;
    const ux = Math.cos(a), uy = Math.sin(a);
    ctx.moveTo(c.x + ux * r, c.y + uy * r);
    ctx.lineTo(c.x + ux * (r - len), c.y + uy * (r - len));
  }
  ctx.stroke();

  // Arm markers where the axes cross the ring: X and Y pills (upright), plus a small
  // filled dot on the positive ends so the picture's orientation is unambiguous.
  ctx.globalAlpha = 1;
  ctx.font = "bold 12px ui-sans-serif, system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  const label = (text: string, dir: Vec2, positive: boolean) => {
    const px = c.x + dir.x * (r + 18);
    const py = c.y + dir.y * (r + 18);
    ctx.beginPath();
    ctx.roundRect(px - 11, py - 9, 22, 18, 5);
    ctx.fillStyle = theme.surface + "e6";
    ctx.fill();
    ctx.strokeStyle = VIEW_ROTATE_COLOR;
    ctx.lineWidth = positive ? 1.6 : 1;
    ctx.stroke();
    ctx.fillStyle = VIEW_ROTATE_COLOR;
    ctx.fillText(text, px, py + 0.5);
  };
  label("X", ex, true);
  label("Y", ey, true);
  label("-X", { x: -ex.x, y: -ex.y }, false);
  label("-Y", { x: -ey.x, y: -ey.y }, false);
  ctx.fillStyle = VIEW_ROTATE_COLOR;
  for (const dir of [ex, ey]) {
    ctx.beginPath();
    ctx.arc(c.x + dir.x * r, c.y + dir.y * r, 4, 0, Math.PI * 2);
    ctx.fill();
  }

  // Centre dot (the pivot).
  ctx.beginPath();
  ctx.arc(c.x, c.y, 3.5, 0, Math.PI * 2);
  ctx.fillStyle = VIEW_ROTATE_COLOR;
  ctx.fill();
  ctx.restore();
}

/** Focus pass veil: how far everything outside the highlighted instances fades
 *  towards the background (0.8 leaves ~20% of the original strength). */
const FOCUS_VEIL_ALPHA = 0.8;

/**
 * Draw one context-ghost level: every body outline (holes included), every joint as a
 * hollow ring and every rail as a thin double arrow, all in the ink tone at low alpha
 * so the picture reads as "surroundings", not as material of this definition.
 */
function drawGhostScene(
  ctx: CanvasRenderingContext2D,
  g: Scene,
  px: (n: number) => number,
  theme: Theme
): void {
  const stroke = theme.ink + GHOST_STROKE_ALPHA;
  ctx.save();
  ctx.setLineDash([]);
  for (const body of g.bodies) {
    const verts = g.bodyWorldVerts(body);
    const holes = g.bodyHolesWorld(body);
    ctx.beginPath();
    verts.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
    ctx.closePath();
    for (const loop of holes) {
      loop.forEach((p, i) => (i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)));
      ctx.closePath();
    }
    ctx.fillStyle = theme.ink + GHOST_FILL_ALPHA;
    ctx.fill("evenodd");
    ctx.strokeStyle = stroke;
    ctx.lineWidth = px(1.2);
    ctx.stroke();
  }
  for (const c of g.constraints) {
    if (c.kind !== "slider") continue;
    const ja = g.getJoint(c.railA);
    const jb = g.getJoint(c.railB);
    if (!ja || !jb) continue;
    drawRailArrow(ctx, g.jointWorld(ja), g.jointWorld(jb), stroke, px(1), px(RAIL_HEAD * 0.8));
  }
  ctx.strokeStyle = stroke;
  ctx.lineWidth = px(1.2);
  for (const j of g.joints) {
    const p = g.jointWorld(j);
    ctx.beginPath();
    ctx.arc(p.x, p.y, px(JOINT_R - 1), 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.restore();
}

/** Context-ghost alphas (hex suffixes on the theme ink): a whisper of fill, a light stroke. */
const GHOST_FILL_ALPHA = "0d";
const GHOST_STROKE_ALPHA = "55";

/** Accent colour for pattern overlays (fixed across themes, like the other semantic accents). */
const PATTERN_COLOR = "#f28cb1";

/** A pattern's overlay: see `PatternView`. */
function drawPattern(
  ctx: CanvasRenderingContext2D,
  pv: PatternView,
  view: View,
  dpr: number,
  theme: Theme
): void {
  const s = view.scale;
  const px = (n: number) => n / s;
  const color = pv.selected ? theme.ink : PATTERN_COLOR;
  ctx.strokeStyle = color;
  for (const ax of pv.axes) {
    // The axis itself: a dotted line from the seed through every instance.
    ctx.lineWidth = px(1.2);
    ctx.setLineDash([px(2), px(4)]);
    ctx.beginPath();
    ctx.moveTo(ax.line.a.x, ax.line.a.y);
    ctx.lineTo(ax.line.b.x, ax.line.b.y);
    ctx.stroke();
    // The spacing dimension beside the first step: extension lines + arrowed segment.
    ctx.lineWidth = px(1);
    ctx.setLineDash([px(4), px(3)]);
    for (const e of ax.ext) {
      ctx.beginPath();
      ctx.moveTo(e.a.x, e.a.y);
      ctx.lineTo(e.b.x, e.b.y);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.lineWidth = px(1.5);
    ctx.beginPath();
    ctx.moveTo(ax.dim.a.x, ax.dim.a.y);
    ctx.lineTo(ax.dim.b.x, ax.dim.b.y);
    ctx.stroke();
    const u = normalize(sub(ax.dim.a, ax.dim.b));
    if ((u.x !== 0 || u.y !== 0) && Math.hypot(ax.dim.b.x - ax.dim.a.x, ax.dim.b.y - ax.dim.a.y) > px(4)) {
      drawArrowHead(ctx, ax.dim.a, u, px(7));
      drawArrowHead(ctx, ax.dim.b, scale(u, -1), px(7));
    }
    drawLabelPill(ctx, ax.stepText, ax.stepLabel, view, dpr, theme, color, pv.selected);
    drawLabelPill(ctx, ax.countText, ax.countLabel, view, dpr, theme, color, pv.selected);
  }
  if (pv.circular) {
    const c = pv.circular;
    // The orbit through the seed (dashed) and the centre as a crosshair.
    ctx.lineWidth = px(1);
    ctx.setLineDash([px(4), px(3)]);
    ctx.beginPath();
    ctx.arc(c.centre.x, c.centre.y, c.radius, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    const r = px(7);
    ctx.lineWidth = px(1.5);
    ctx.beginPath();
    ctx.arc(c.centre.x, c.centre.y, r, 0, Math.PI * 2);
    ctx.moveTo(c.centre.x - r * 1.6, c.centre.y);
    ctx.lineTo(c.centre.x + r * 1.6, c.centre.y);
    ctx.moveTo(c.centre.x, c.centre.y - r * 1.6);
    ctx.lineTo(c.centre.x, c.centre.y + r * 1.6);
    ctx.stroke();
    drawLabelPill(ctx, c.countText, c.countLabel, view, dpr, theme, color, pv.selected);
    drawLabelPill(ctx, c.angleText, c.angleLabel, view, dpr, theme, color, pv.selected);
    drawLabelPill(ctx, c.rotateText, c.rotateLabel, view, dpr, theme, color, pv.selected);
  }
  // Members that don't fit: a red dashed ring.
  ctx.strokeStyle = FLASH_COLOR;
  ctx.lineWidth = px(1.5);
  ctx.setLineDash([px(3), px(3)]);
  for (const b of pv.bad) {
    ctx.beginPath();
    ctx.arc(b.x, b.y, px(JOINT_R + 4), 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.setLineDash([]);
  // Handles (selected only): squares at each axis end and the centre, like vertex handles.
  if (pv.selected) {
    const h = px(5);
    const handles = [...pv.axes.map((ax) => ax.handle), ...(pv.circular ? [pv.circular.centre] : [])];
    ctx.lineWidth = px(2);
    for (const p of handles) {
      ctx.fillStyle = theme.ink;
      ctx.strokeStyle = theme.surface;
      ctx.beginPath();
      ctx.rect(p.x - h, p.y - h, 2 * h, 2 * h);
      ctx.fill();
      ctx.stroke();
    }
  }
}

/** A screen-space text pill at a world position (the dimension-label look). */
function drawLabelPill(
  ctx: CanvasRenderingContext2D,
  text: string,
  at: Vec2,
  view: View,
  dpr: number,
  theme: Theme,
  color: string,
  bold: boolean
): void {
  const { x: sx, y: sy } = worldToScreen(view, at);
  ctx.save();
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.font = "12px ui-sans-serif, system-ui, sans-serif";
  const tw = ctx.measureText(text).width;
  const pw = tw + 12;
  const ph = 18;
  ctx.beginPath();
  ctx.roundRect(sx - pw / 2, sy - ph / 2, pw, ph, 5);
  ctx.fillStyle = theme.surface + "e6";
  ctx.fill();
  ctx.strokeStyle = color;
  ctx.lineWidth = bold ? 1.6 : 1;
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, sx, sy + 0.5);
  ctx.restore();
}

/** Accent colour for measurements (fixed across themes, like the other semantic accents). */
const MEASURE_COLOR = "#46c2cb";
/** Temporary context dimensions (definition ↔ enclosing-assembly ghost): a muted measure tint. */
const TEMP_MEASURE_COLOR = "#7fa6a9";
/** Construction guidelines: muted, CAD-centre-line grey (reads on both themes). */
const GUIDE_COLOR = "#9aa0ac";
/** Accent colour for sketch constraints (violet, distinct from every other accent). */
const SKETCH_COLOR = "#b48cff";
/** Object-snap highlights (dragged reference + snapped target): warm orange. */
const OSNAP_COLOR = "#ff9f43";
/** Fill of a feature-selected vertex handle (draw mode) — the pin blue, as a "selected" accent. */
const FEATURE_SEL_COLOR = "#4f9dff";
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
  const t =
    info.kind === "angle" ? `${v}°` : info.circle ? `⌀${v} ${unit}` : info.fillet ? `R${v} ${unit}` : `${v} ${unit}`;
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
  const color = flashed || g.violated ? FLASH_COLOR : selected ? theme.ink : SKETCH_COLOR;
  // Faded unless the cursor is on the constrained element — selection / a reject flash /
  // a violated pose constraint always shows at full strength.
  const alpha = g.faded && !selected && !flashed && !g.violated ? 0.2 : 1;
  const bold = selected || flashed || !!g.violated;
  for (const b of g.badges) drawSketchBadge(ctx, b, g.kind, view, dpr, theme, color, alpha, bold);
}

/** One constant-size sketch badge (a pill with the kind's symbol) centred on world `at`. */
function drawSketchBadge(
  ctx: CanvasRenderingContext2D,
  at: Vec2,
  kind: SketchConstraintKind,
  view: View,
  dpr: number,
  theme: Theme,
  color: string,
  alpha: number,
  bold: boolean
): void {
  const { x: sx, y: sy } = worldToScreen(view, at);
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
  ctx.lineWidth = bold ? 1.6 : 1;
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(SKETCH_SYMBOL[kind], sx, sy + 0.5);
  ctx.restore();
}

/** Highlight a measure reference: a ring around a point, a soft thick stroke over a line
 *  (or around a whole disk rim, for a diameter pick). */
function drawMeasureRefHighlight(
  ctx: CanvasRenderingContext2D,
  ref: MeasureHighlight,
  px: (n: number) => number,
  isHover: boolean,
  viewRect: { left: number; top: number; right: number; bottom: number },
  color: string = MEASURE_COLOR
): void {
  ctx.strokeStyle = color;
  if (ref.kind === "line" && ref.infinite) {
    // A guide is an infinite construction line: highlight it right across the view,
    // and ring its two defining points so they stay easy to find along it.
    const d = normalize(sub(ref.b, ref.a));
    if (d.x !== 0 || d.y !== 0) {
      const { left, top, right, bottom } = viewRect;
      const tc = ((left + right) / 2 - ref.a.x) * d.x + ((bottom + top) / 2 - ref.a.y) * d.y;
      const half = Math.hypot(right - left, bottom - top);
      ctx.save();
      ctx.globalAlpha = isHover ? 0.4 : 0.7;
      ctx.lineWidth = px(5);
      ctx.beginPath();
      ctx.moveTo(ref.a.x + d.x * (tc - half), ref.a.y + d.y * (tc - half));
      ctx.lineTo(ref.a.x + d.x * (tc + half), ref.a.y + d.y * (tc + half));
      ctx.stroke();
      ctx.restore();
    }
    ctx.lineWidth = px(2);
    if (isHover) ctx.setLineDash([px(3), px(3)]);
    for (const p of [ref.a, ref.b]) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, px(7), 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    return;
  }
  if (ref.kind === "circle" || ref.kind === "arc") {
    ctx.save();
    ctx.globalAlpha = isHover ? 0.4 : 0.7;
    ctx.lineWidth = px(5);
    ctx.beginPath();
    if (ref.kind === "arc") ctx.arc(ref.c.x, ref.c.y, ref.r, ref.a0, ref.a0 + ref.sweep, ref.sweep < 0);
    else ctx.arc(ref.c.x, ref.c.y, ref.r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  } else if (ref.kind === "point") {
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
  // A violated driving dimension (its measured value drifted from the target — e.g. a
  // definition edit reset instance poses, or a grounded partner couldn't follow a drag)
  // stays in the error red until re-applied.
  // A temporary context dimension (onto the enclosing-assembly ghost) takes a muted
  // tint of the measure colour: it reads as a reference to the surroundings, not as
  // part of this definition's dimension scheme.
  const color =
    flashed || info.violated ? FLASH_COLOR : selected ? theme.ink : info.temp ? TEMP_MEASURE_COLOR : MEASURE_COLOR;
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
      if (!info.singleArrow) drawArrowHead(ctx, a, u, px(7)); // a radius line starts bare at the centre
      drawArrowHead(ctx, b, scale(u, -1), px(7));
    }
  }
  if (info.fillet) {
    // A radius dimension marks the arc centre it measures from with a small dot.
    ctx.save();
    ctx.setLineDash([]);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(info.fillet.c.x, info.fillet.c.y, px(2.5), 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
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
  const { x: sx, y: sy } = worldToScreen(view, info.labelPos);
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

/**
 * Construction-point marker for guideline base points: a thin "+" with a small open ring
 * at the centre. Reads as a reference point (CAD convention) rather than a joint, whose
 * marker is a filled disk. Slightly larger and heavier when its guide is selected.
 */
function crosshair(
  ctx: CanvasRenderingContext2D,
  p: Vec2,
  px: (n: number) => number,
  color: string,
  selected: boolean
): void {
  const arm = px(selected ? 8 : 6.5);
  const ring = px(selected ? 3 : 2.5);
  ctx.strokeStyle = color;
  ctx.lineWidth = px(selected ? 1.6 : 1.2);
  ctx.setLineDash([]);
  ctx.beginPath();
  ctx.moveTo(p.x - arm, p.y);
  ctx.lineTo(p.x + arm, p.y);
  ctx.moveTo(p.x, p.y - arm);
  ctx.lineTo(p.x, p.y + arm);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(p.x, p.y, ring, 0, Math.PI * 2);
  ctx.stroke();
}

function dot(ctx: CanvasRenderingContext2D, p: Vec2, r: number, color: string): void {
  ctx.beginPath();
  ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

/**
 * Draw a slider carriage: a rail-aligned rectangle centered on an orientation-locked
 * rider, badging it as prismatic (travels along the rail, no rotation) rather than a
 * pin-in-slot. `along` is the rail direction (any length).
 */
function drawCarriage(
  ctx: CanvasRenderingContext2D,
  at: Vec2,
  along: Vec2,
  halfLen: number,
  halfWid: number,
  color: string,
  lineWidth: number
): void {
  const l = Math.hypot(along.x, along.y);
  if (l < 1e-9) return;
  ctx.save();
  // Rotate the canvas so the rectangle's long axis follows the rail.
  ctx.transform(along.x / l, along.y / l, -along.y / l, along.x / l, at.x, at.y);
  ctx.beginPath();
  ctx.rect(-halfLen, -halfWid, halfLen * 2, halfWid * 2);
  ctx.globalAlpha = 0.25;
  ctx.fillStyle = color;
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.stroke();
  ctx.restore();
}

/**
 * Draw a rail as a double-headed arrow a↔b: the line between the two rail joints with an
 * open chevron head at each end (tips at the joints, wings `head` long at 30°, so they
 * stay visible around the joint dots drawn on top).
 */
function drawRailArrow(
  ctx: CanvasRenderingContext2D,
  a: Vec2,
  b: Vec2,
  color: string,
  lineWidth: number,
  head: number
): void {
  const d = sub(b, a);
  const l = Math.hypot(d.x, d.y);
  if (l < 1e-6) return;
  const u = { x: d.x / l, y: d.y / l };
  const wing = (tip: Vec2, back: Vec2, s: number): Vec2 => ({
    x: tip.x + back.x * head * Math.cos(Math.PI / 6) - back.y * head * Math.sin(Math.PI / 6) * s,
    y: tip.y + back.y * head * Math.cos(Math.PI / 6) + back.x * head * Math.sin(Math.PI / 6) * s,
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineJoin = "round";
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  // Head at b: wings point back along -u; head at a: wings point along +u.
  for (const [tip, back] of [
    [b, { x: -u.x, y: -u.y }],
    [a, u],
  ] as [Vec2, Vec2][]) {
    const w1 = wing(tip, back, 1);
    const w2 = wing(tip, back, -1);
    ctx.moveTo(w1.x, w1.y);
    ctx.lineTo(tip.x, tip.y);
    ctx.lineTo(w2.x, w2.y);
  }
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
