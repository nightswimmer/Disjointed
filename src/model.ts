/**
 * Scene data model: rigid bodies (polygons), joint points attached to bodies,
 * and the constraints that couple them. All world geometry is derived from a
 * body's pose (centroid position + angle) plus each joint's local offset.
 */
import {
  Vec2,
  add,
  rotate,
  vec,
  clone,
  sub,
  dist,
  scale,
  dot,
  cross,
  len,
  lenSq,
  normalize,
  perp,
  distToLine,
  distToSegment,
  filletPolygon,
  roundedConvexBody,
  polygonCentroid,
  polygonArea,
  polygonInertiaAboutCentroid,
  pointInPolygon,
  closestPointOnPolygon,
} from "./geometry";
import { unionRegions, segmentsCross, pointOnSegment, PolyRegion } from "./boolean";

/**
 * A joint exactly coincident with a body control vertex is "stuck" to it — they move
 * together (how joint-built bodies keep their joints and nodes linked). Coincidence is
 * exact up to float error from frame transforms, so the tolerance can be tiny.
 */
export const VERTEX_LINK_EPS = 1e-6;

/**
 * Slack for the joint-containment warning (`jointsOutsideBody`): a drag clamps a joint
 * exactly onto the outline, so boundary points must never count as "outside" — only
 * float error from the local↔world round-trips needs absorbing.
 */
export const CONTAINMENT_WARN_EPS = 1e-6;

/** How a body's `radius` shapes it: round the corners in place, or offset the hull outward. */
export type RoundMode = "fillet" | "offset";

/**
 * One editable hole outline (v16): a control polygon + rounding, exactly like the
 * body's outer shape. Lives in the body's local frame (relative to the centroid).
 */
export interface BodyHole {
  /** Editable control polygon of the hole, relative to the centroid, local frame. */
  controlLocal: Vec2[];
  /** Corner radius (fillet) or outward margin (offset mode) — the hole's default. */
  radius: number;
  /** Per-corner radius overrides, parallel to `controlLocal` (null = use `radius`). */
  radii?: (number | null)[];
  /**
   * "offset" derives the hole as the rounded hull of its control points (one point =
   * a perfect circular hole); default "fillet" rounds the polygon's corners in place.
   */
  round?: RoundMode;
}

/**
 * A hole passed to `addBody` (world coords): a plain sampled loop (becomes a radius-0
 * hole control polygon) or a control polygon + rounding.
 */
export type HoleSpec =
  | Vec2[]
  | { control: Vec2[]; radius?: number; radii?: (number | null)[]; round?: RoundMode };

/** What a pattern replicates, as the UI picks it: one hole of a body, or one attached joint. */
export type PatternSeed =
  | { kind: "hole"; bodyId: number; hole: number }
  | { kind: "joint"; jointId: number };

/** One direction of a linear pattern: `count` instances (the seed included) every `step` (body-local). */
export interface PatternAxis {
  count: number;
  step: Vec2;
}

/**
 * A pattern's layout, in its body's local frame (so the pattern rides with the body).
 * Linear: one or two axes (two = a grid). Circular: `count` instances around `centre`
 * (an offset from the seed's anchor), `angleStep` radians apart — absent = spread evenly
 * over the full circle; `rotate` turns each instance's shape with the arc, otherwise
 * shapes keep the seed's orientation and only their anchors orbit.
 */
export type PatternLayout =
  | { kind: "linear"; axes: PatternAxis[] }
  | { kind: "circular"; centre: Vec2; count: number; angleStep?: number; rotate: boolean };

/**
 * A live pattern (v19): the seed (a hole or joint of `bodyId`) plus the layout, and the
 * **members** it owns — real holes (indices into `Body.holes`) or joints (ids) of the same
 * body, re-derived from the seed whenever the seed or its body changes (`syncPattern`).
 * Members are derived geometry: the sketch solver never moves them (rank-immovable, like
 * instance material), a drag on one moves the whole pattern, and the layout is edited
 * through its on-canvas labels. Deleting the seed dissolves the pattern (members stay as
 * plain holes / joints); deleting a member deletes every member (the seed stays).
 */
export interface Pattern {
  id: number;
  bodyId: number;
  seed: { kind: "hole"; hole: number } | { kind: "joint"; jointId: number };
  layout: PatternLayout;
  /** Instances after the seed, in layout order (see `patternLocalMotions`). */
  members: number[];
}

/** A pattern resolved to world geometry for rendering / picking (see `Scene.patternInfo`). */
export interface PatternInfo {
  id: number;
  bodyId: number;
  kind: "linear" | "circular";
  /** The seed's anchor: a hole's control-polygon centroid (a disk's centre) or the joint. */
  anchor: Vec2;
  /** Linear axes: where the last instance along each axis sits, the count and the spacing. */
  axes: { end: Vec2; count: number; step: number }[];
  /** Circular layout: the centre, the orbit radius, count, angle in degrees (null = even), rotation. */
  circular: { centre: Vec2; radius: number; count: number; angleDeg: number | null; rotate: boolean } | null;
  /** Every member's anchor and whether it fits (inside the body, clear of other holes / joints). */
  members: { point: Vec2; ok: boolean }[];
}

/** Preview geometry of a layout that isn't (yet) a pattern (see `Scene.patternPreview`). */
export interface PatternPreview {
  anchor: Vec2;
  seedLoop: Vec2[] | null;
  instances: { point: Vec2; loop: Vec2[] | null; ok: boolean }[];
}

/** Largest instance count along one axis / around a circle (guards runaway inputs). */
export const PATTERN_MAX_COUNT = 200;
/** Largest total member count of a two-axis grid. */
export const PATTERN_MAX_MEMBERS = 400;
/** Two joints on one body closer than this are the same point — a member never stacks on one. */
const PATTERN_JOINT_EPS = 1e-6;

/**
 * The rigid motions (local frame) taking the seed to every member, in member order:
 * linear = row-major over the axes (axis 0 fastest), the seed's own cell skipped;
 * circular = increasing angle. `anchor` is the seed's local anchor (the orbit point).
 */
export function patternLocalMotions(layout: PatternLayout, anchor: Vec2): ((p: Vec2) => Vec2)[] {
  const out: ((p: Vec2) => Vec2)[] = [];
  if (layout.kind === "linear") {
    const a = layout.axes[0];
    const b = layout.axes[1];
    if (!a) return out;
    const nA = Math.max(1, Math.floor(a.count));
    const nB = b ? Math.max(1, Math.floor(b.count)) : 1;
    for (let j = 0; j < nB; j++) {
      for (let i = 0; i < nA; i++) {
        if (i === 0 && j === 0) continue;
        const d = add(scale(a.step, i), b ? scale(b.step, j) : vec(0, 0));
        out.push((p) => add(p, d));
      }
    }
    return out;
  }
  const n = Math.max(1, Math.floor(layout.count));
  const step = layout.angleStep ?? (2 * Math.PI) / n;
  const c = add(anchor, layout.centre);
  for (let k = 1; k < n; k++) {
    const ang = step * k;
    if (layout.rotate) out.push((p) => add(c, rotate(sub(p, c), ang)));
    else {
      const d = sub(add(c, rotate(sub(anchor, c), ang)), anchor);
      out.push((p) => add(p, d));
    }
  }
  return out;
}

/** Plain average of a point set (a 1- or 2-point "polygon" has no area centroid). */
function centroidOfPoints(pts: Vec2[]): Vec2 {
  let x = 0;
  let y = 0;
  for (const p of pts) { x += p.x; y += p.y; }
  return vec(x / pts.length, y / pts.length);
}

/** Deep copy of a pattern layout. */
function cloneLayout(l: PatternLayout): PatternLayout {
  return l.kind === "linear"
    ? { kind: "linear", axes: l.axes.map((a) => ({ count: a.count, step: vec(a.step.x, a.step.y) })) }
    : { kind: "circular", centre: vec(l.centre.x, l.centre.y), count: l.count, rotate: l.rotate, ...(l.angleStep !== undefined ? { angleStep: l.angleStep } : {}) };
}

/** Axis-aligned bounds of a point set. */
function boundsOf(pts: Vec2[]): { min: Vec2; max: Vec2 } {
  const min = vec(Infinity, Infinity);
  const max = vec(-Infinity, -Infinity);
  for (const p of pts) {
    min.x = Math.min(min.x, p.x); min.y = Math.min(min.y, p.y);
    max.x = Math.max(max.x, p.x); max.y = Math.max(max.y, p.y);
  }
  return { min, max };
}

/** A layout with its vectors turned by `ang` (into / out of a body frame). */
function rotateLayout(l: PatternLayout, ang: number): PatternLayout {
  if (l.kind === "linear") return { kind: "linear", axes: l.axes.map((a) => ({ count: a.count, step: rotate(a.step, ang) })) };
  return { ...l, centre: rotate(l.centre, ang) };
}

/** Outcome of `Scene.splitBody`: the two sides (A keeps the original id), or why it was refused. */
export type SplitResult = { ok: true; a: Body; b: Body } | { ok: false; reason: string };

/** Outcome of `Scene.combineBodies`: the surviving (first) body, or why it was refused. */
export type CombineResult = { ok: true; body: Body } | { ok: false; reason: string };

/** Effective per-corner radii of a hole (overrides over its default), or the default. */
function holeRadii(h: BodyHole): number | number[] {
  if (!h.radii) return h.radius;
  return h.controlLocal.map((_, i) => {
    const r = h.radii![i];
    return typeof r === "number" ? r : h.radius;
  });
}

/** Derive a hole's sampled outline from its control polygon (in the control's frame). */
function deriveHoleOutline(control: Vec2[], h: BodyHole): Vec2[] {
  const radii = holeRadii(h);
  return h.round === "offset" ? roundedConvexBody(control, radii) : filletPolygon(control, radii);
}

/** Deep copy of a body's hole shapes. */
function cloneBodyHoles(holes: BodyHole[]): BodyHole[] {
  return holes.map((h) => {
    const out: BodyHole = {
      controlLocal: h.controlLocal.map((p) => vec(p.x, p.y)),
      radius: h.radius,
    };
    if (h.radii) out.radii = [...h.radii];
    if (h.round) out.round = h.round;
    return out;
  });
}

export interface Body {
  id: number;
  /** Editable control polygon (the corners), relative to the centroid, in the local frame. */
  controlLocal: Vec2[];
  /** Corner radius (fillet) or outward margin (offset) applied to the control polygon. 0 = sharp. */
  radius: number;
  /**
   * Per-corner radius overrides, parallel to `controlLocal` (v15): a number overrides
   * `radius` for that corner (0 = sharp), `null` means "use the body default". Present
   * only while at least one corner is overridden; absent = every corner uses `radius`.
   */
  radii?: (number | null)[];
  round: RoundMode;
  /** Derived render/physics polygon, relative to the centroid — rebuilt from control + radius. */
  local: Vec2[];
  /**
   * Inner cut-outs (v16): editable hole outlines, each a control polygon + rounding
   * like the body's outer shape. Holes render as cut-outs, subtract from
   * mass/centroid/inertia, mirror/rotate/scale/copy with the body, and their control
   * vertices/edges are first-class measurement + sketch-constraint references
   * (`MeasureRef.hole`). Picking + joint containment still use the outer outline only
   * (so a joint can sit at the centre of a shaft hole). Present iff `holesLocal` is.
   */
  holes?: BodyHole[];
  /**
   * Derived sampled hole loops (parallel to `holes`), relative to the centroid, in the
   * local frame — rebuilt from each hole's control + radius, like `local` is from the
   * outer control polygon. What the renderer / mass properties consume.
   */
  holesLocal?: Vec2[][];
  /** World position of the centroid (the body's local origin). */
  pos: Vec2;
  angle: number;
  invMass: number;
  invInertia: number;
  color: string;
  /**
   * Grounded body: fixed in the world during simulation (the solver treats it as
   * immovable, like a ground anchor — sacred, never disabled). Draw mode still edits
   * and moves it freely. Grounding any member of a permanent group fixes the whole
   * group (the group is rigid). Absent pre-v10 (loads as false).
   */
  grounded: boolean;
}

export interface Joint {
  id: number;
  /** Owning body, or `null` for a free joint (a body-less movable point). */
  bodyId: number | null;
  /**
   * For an attached joint: offset from the body's centroid, in the body's local
   * frame. For a free joint (`bodyId === null`): the joint's own world position.
   */
  local: Vec2;
}

/**
 * Two joints (on different bodies) share a world position; free relative rotation —
 * unless `rigid` (v18): a **weld**, which also locks the two bodies' relative angle,
 * joining them completely (no relative motion at all). The welded-in relative angle is
 * captured from the drawn pose when simulation starts (see solver.ts weld baselines) —
 * the same philosophy as pins and slider locks: the drawn pose is the intended assembly.
 * A weld where either joint is free (body-less) behaves as a plain pin until the joint
 * gains a body (a free point has no orientation to lock).
 */
export interface PinConstraint {
  kind: "pin";
  id: number;
  jointA: number;
  jointB: number;
  /** Present (true) only for welds; a plain revolute pin has no flag. */
  rigid?: boolean;
}

/** A joint is locked to a fixed world point; the body may rotate about it. */
export interface GroundConstraint {
  kind: "ground";
  id: number;
  joint: number;
  anchor: Vec2;
}

/**
 * A rail (user-facing name; historically "slider"): the segment between `railA` and
 * `railB` — two joints on one body. Any joint in `riders` (on other bodies) is confined
 * to that segment and slides along it. The rail moves with its body, so it couples the
 * rail's body to each rider's body. (For a world-fixed track, put the rail joints on a
 * grounded body.) A rail with no riders is just a (selectable, deletable) guide.
 *
 * `locked` (v17) names the riders that are **sliders** (prismatic carriages): besides
 * riding the rail, the rider's body keeps its drawn orientation relative to the rail —
 * it translates along the rail but cannot rotate. An unlocked rider is a pin-in-slot
 * (slides AND rotates). The locked-in relative angle is captured from the drawn pose
 * when simulation starts (see solver.ts lock baselines). A lock on a free (body-less)
 * rider is inert until the joint gains a body (e.g. absorbed by build-body-from-joints).
 * Always a subset of `riders`.
 */
export interface SliderConstraint {
  kind: "slider";
  id: number;
  railA: number;
  railB: number;
  riders: number[];
  /** Riders whose body's orientation is locked to the rail (prismatic sliders). */
  locked: number[];
}

/**
 * A linear actuator: a free joint (the `rider`) confined to `sliderId`'s rail that travels
 * back and forth along it at `speed` cycles per second when animation is running. Off-animation
 * it behaves like any other rider on that slider (draggable, pinnable). `profile` picks the
 * motion: `"triangle"` is end-to-end at constant velocity (the natural mechanical actuator),
 * `"sine"` eases in/out at each endstop.
 */
export interface LinearActuatorConstraint {
  kind: "linearActuator";
  id: number;
  sliderId: number;
  riderId: number;
  speed: number;
  profile: "triangle" | "sine";
}

/**
 * A motor: a body whose `pivotJointId` joint stays at its current world position while
 * `crankJointId` orbits it at `speed` revolutions per second when animation is running. Both
 * joints must belong to `bodyId`. Off-animation, the body behaves normally. The motor only
 * acts during animation, where it temporarily anchors both joints (pivot fixed + crank on its
 * orbit) — two anchors on one body fully determine its pose, which the existing solver handles.
 */
export interface MotorConstraint {
  kind: "motor";
  id: number;
  bodyId: number;
  pivotJointId: number;
  crankJointId: number;
  speed: number;
}

export type Constraint =
  | PinConstraint
  | GroundConstraint
  | SliderConstraint
  | LinearActuatorConstraint
  | MotorConstraint;

/**
 * A permanent group of bodies (created from a multi-selection in draw mode). Grouped
 * bodies are selected and moved as one in draw mode, and behave as a **single rigid
 * body** in simulation: the solver applies every impulse to the whole group about its
 * combined centroid, so the members never move relative to each other. A body belongs
 * to at most one group; groups need at least 2 members (smaller ones dissolve).
 * `jointIds` (v14) are **free joints locked rigidly to the group**: they translate and
 * rotate with it in simulation exactly like body material (the chassis anchor points of
 * component instances are these). Absent in older files → none.
 */
export interface BodyGroup {
  id: number;
  bodyIds: number[];
  jointIds: number[];
}

// --- construction guidelines ------------------------------------------------

/**
 * A construction guideline: an **infinite** line defined by two points `a` and `b`
 * (world coordinates). A drawing aid only — guidelines never participate in
 * simulation. Dragging the line translates both points (the angle is preserved);
 * dragging either defining point re-aims the line. With snapping enabled,
 * placements and drags snap onto guidelines in preference to the grid.
 */
export interface Guide {
  id: number;
  a: Vec2;
  b: Vec2;
}

// --- measurements ---------------------------------------------------------

/** Which mode a measurement belongs to — draw and sim each keep their own set. */
export type MeasureMode = "draw" | "sim";

/**
 * For a point–point measurement, which distance the label placement selected:
 * `"h"` horizontal (|Δx|), `"v"` vertical (|Δy|), `"direct"` straight-line.
 * `"diameter"` is the special case of a dimension whose two refs are the *same*
 * control vertex of a **disk outline** (a one-point offset-mode outer outline or hole):
 * it measures — and, when driving, sets — that disk's diameter.
 */
export type MeasureAxis = "direct" | "h" | "v" | "diameter";

/**
 * A measurement reference — a point or a line, anchored to scene *elements* (never to
 * bare coordinates), so its world geometry is re-resolved every frame and the value
 * tracks the mechanism as it moves in simulation.
 */
export type MeasureRef =
  | { kind: "joint"; jointId: number } // point: a joint
  | { kind: "vertex"; bodyId: number; index: number; hole?: number } // point: a control vertex (of hole `hole`, or the outer outline)
  | { kind: "bodyPoint"; bodyId: number; local: Vec2 } // point: fixed in a body's frame
  | { kind: "rail"; sliderId: number } // line: a slider rail
  | { kind: "edge"; bodyId: number; index: number; hole?: number } // line: control edge index → index+1 (of hole `hole`, or the outer)
  | { kind: "guidePoint"; guideId: number; which: "a" | "b" } // point: a guideline defining point
  | { kind: "guideLine"; guideId: number } // line: a construction guideline (infinite)
  | { kind: "patternAxis"; patternId: number; axis: number }; // line: a linear pattern's direction (seed anchor → last instance)

/**
 * A dimension between two references. What it measures follows from the reference kinds:
 * point+point → distance along `axis`; point+line → perpendicular distance to the
 * infinite line; line+line → distance while (near-)parallel, angle otherwise — resolved
 * dynamically each frame, so a line pair can flip between the two in simulation.
 * `labelOffset` positions the value display relative to the references' midpoint, so
 * the label travels with the geometry it measures.
 *
 * A **draw-mode** dimension can be *driving* (`driving: true` + a `target` value): it acts
 * as a sketch constraint — the sketch solver moves geometry so the measured value equals
 * `target`. A driven dimension (the default) is a read-only reference. Sim-mode
 * measurements are always driven.
 */
export interface Measurement {
  id: number;
  mode: MeasureMode;
  refA: MeasureRef;
  refB: MeasureRef;
  labelOffset: Vec2;
  axis: MeasureAxis;
  /** Absent/false = driven (read-only). Only draw-mode distance dimensions can drive. */
  driving?: boolean;
  /** The value a driving dimension holds the geometry to (world units). */
  target?: number;
  /**
   * The relative direction a driving dimension holds, captured when it starts driving:
   * for an h/v pair the sign of (B − A) along the axis, for point+line / line+line the
   * sign of the signed perpendicular distance. Solvers enforce `side * target` instead
   * of re-deriving the sign from current geometry, so a fast drag can never flip the
   * two sides through each other. Absent for direct point-point dimensions (a pure
   * distance is free to rotate — it has no side) and on driven dimensions.
   */
  side?: 1 | -1;
}

// --- sketch constraints -----------------------------------------------------

/**
 * Kinds of CAD-style sketch constraints (draw mode only):
 * `coincident` — two points share a position, or a point lies on an infinite line;
 * `horizontal`/`vertical` — a line (or a point pair) is axis-aligned;
 * `parallel`/`perpendicular` — two lines' directions; `equal` — two lines have equal
 * length.
 */
export type SketchConstraintKind =
  | "coincident"
  | "horizontal"
  | "vertical"
  | "parallel"
  | "perpendicular"
  | "equal";

/**
 * A sketch constraint between one or two references (reusing the measurement reference
 * system, so constraints track their elements the same way measurements do — including
 * index remapping across control-vertex edits and prune-on-delete). `refB` is null only
 * for horizontal/vertical applied to a single line reference. Solvable point refs are
 * joints and body control vertices (`bodyPoint` refs are measurement-only); line refs
 * are slider rails and body control edges.
 */
export interface SketchConstraint {
  kind: SketchConstraintKind;
  id: number;
  refA: MeasureRef;
  refB: MeasureRef | null;
}

/** A reference resolved to current world geometry. */
export type ResolvedMeasureRef =
  | { kind: "point"; p: Vec2 }
  /** `infinite` marks a construction line (a guide): `a`–`b` are its defining points, but
   *  the element itself extends without end (highlights draw it across the view). */
  | { kind: "line"; a: Vec2; b: Vec2; infinite?: boolean };

/**
 * A measure-tool highlight: a resolved reference, or a whole disk outline (the rim of
 * a circular hole / disk body picked for a diameter dimension).
 */
export type MeasureHighlight = ResolvedMeasureRef | { kind: "circle"; c: Vec2; r: number };

/** A disk outline (one-point offset-mode outer outline or hole) found from a vertex ref. */
export interface DiskRef {
  bodyId: number;
  /** The hole index, or null for the body's outer outline. */
  hole: number | null;
  /** World centre (the outline's single control point). */
  c: Vec2;
  /** Current radius. */
  r: number;
}

/** Everything needed to display a measurement this frame (value + drawing geometry). */
export interface MeasureInfo {
  id: number;
  kind: "distance" | "angle";
  /** World units for a distance; degrees for an angle. */
  value: number;
  /** True when the source dimension is driving (drawn without the CAD parentheses). */
  driving?: boolean;
  /** True when a driving dimension's measured value has drifted from its target (e.g.
   *  a definition edit reset instance poses, or a held partner couldn't follow a drag)
   *  — rendered in an error style until re-applied. */
  violated?: boolean;
  labelPos: Vec2;
  /** The disk a diameter dimension measures (its value is 2·r; drawn with a ⌀ prefix). */
  circle?: { c: Vec2; r: number };
  /** Arrowed dimension segment (distance only). */
  dim?: { a: Vec2; b: Vec2 };
  /** Dashed extension / leader segments. */
  ext: { a: Vec2; b: Vec2 }[];
  /** Angle arc (angle only): centre, radius, start angle, positive CCW sweep. */
  arc?: { c: Vec2; r: number; a0: number; sweep: number };
}

/**
 * The document's working unit: what one world unit means physically. Purely a
 * declaration — no coordinate ever changes when the unit changes — but measurements
 * display it and the DXF importer converts incoming files into it.
 */
export type Unit = "mm" | "cm" | "m" | "in";

/** Millimetres per working unit (also used to convert DXF `$INSUNITS` on import). */
export const UNIT_TO_MM: Record<Unit, number> = { mm: 1, cm: 10, m: 1000, in: 25.4 };

const DEFAULT_UNIT: Unit = "mm";

// --- components (hierarchical design) ---------------------------------------

/**
 * A reusable component definition (v14): a complete sub-scene with its own coordinate
 * frame, edited in its own context with every normal tool. Its sketch constraints,
 * driving dimensions, measurements and guides live **only here** — they never expand
 * into a parent context, so an instance carries the designed shapes without the
 * design-time constraints. Grounding inside a definition means "fixed to the
 * component's frame": on expansion the grounded material becomes one rigid chassis
 * group per instance (never grounded to the world). A definition's data may itself
 * contain instances of other definitions (a DAG — cycles are rejected).
 */
export interface ComponentDef {
  id: number;
  name: string;
  data: SceneData;
}

/** One expanded element of a component instance: def-local src id → id in this context. */
export interface InstanceMapEntry {
  src: number;
  id: number;
}

/** A body entry also caches the body's def-frame pose at the last expansion, so the
 *  instance's placement transform can be re-derived when the definition changes. */
export interface InstanceBodyEntry extends InstanceMapEntry {
  defPos: Vec2;
  defAngle: number;
  /** Part of the rigid chassis (grounded in the definition) — pose follows the def rigidly. */
  chassis: boolean;
}

/**
 * A materialized instance of a component definition. The mapped elements are *real*
 * bodies / joints / constraints of the owning context (the solver, renderer and
 * hit-testing see nothing special); the maps carry the provenance that re-expansion
 * uses to reconcile the instance when the definition changes. `anchorMap` names the
 * free joints synthesized from the definition's joint-ground constraints (keyed by
 * that ground constraint's def-local id); `groupId` is the rigid chassis group.
 */
export interface ComponentInstance {
  id: number;
  defId: number;
  bodyMap: InstanceBodyEntry[];
  jointMap: InstanceMapEntry[];
  constraintMap: InstanceMapEntry[];
  anchorMap: InstanceMapEntry[];
  /** Groups recreated from the definition's own groups (src = def group id). */
  groupMap: InstanceMapEntry[];
  groupId: number | null;
}

/** The material one occurrence of a component expanded into, in the ids of the context
 *  being queried — a direct instance, or one nested inside other instances (see
 *  `Scene.componentOccurrences`). */
export interface ComponentOccurrence {
  bodyIds: number[];
  /** Mechanism joints + synthesized anchors. */
  jointIds: number[];
}

/** A rigid placement mapping def-frame coordinates into the owning context. */
export interface InstanceTransform {
  pos: Vec2;
  angle: number;
}

/** Serializable snapshot of an entire scene (for save / load / autosave). */
export interface SceneData {
  version: number;
  bodies: Body[];
  joints: Joint[];
  constraints: Constraint[];
  /** Working unit (1 world unit = 1 of these). Absent pre-v12 → "mm". */
  unit?: Unit;
  /** Draw-mode and sim-mode measurements together (each carries its `mode`). Absent pre-v7. */
  measurements?: Measurement[];
  /** Draw-mode sketch constraints. Absent pre-v8. */
  sketch?: SketchConstraint[];
  /** Permanent body groups. Absent pre-v9. */
  groups?: BodyGroup[];
  /** Construction guidelines. Absent pre-v11. */
  guides?: Guide[];
  /** Component definitions (v14) — document-level, present only in the root snapshot. */
  components?: ComponentDef[];
  /** Component instances expanded into *this* context (v14). */
  instances?: ComponentInstance[];
  /** Live patterns of this context (v19). */
  patterns?: Pattern[];
}

/**
 * A self-contained snapshot of a selection for copy/paste: one or more bodies (with the
 * joints attached to them), any free joints included in the selection, and every
 * constraint whose joints all travel with the clip — grounds, internal sliders,
 * and pins (including pins *between* copied bodies, which is how a linked pair or a
 * permanent group copies as a working unit). Permanent groups among the copied bodies
 * are captured too, so pasting a group yields a new group. Everything is stored in
 * world coordinates; pasting translates the whole fragment so `center` lands at the
 * drop point. `tmp` ids are the original body / joint / slider ids, remapped to fresh
 * elements on paste. Anything referencing an element outside the selection (a pin to
 * an uncopied body, a cross-selection sketch constraint / driving dimension) is dropped.
 */
export interface SelectionClip {
  /** Paste reference: the mass-weighted centre of the copied bodies (a single body's
   *  own centroid), or the copied joints' average for a body-less clip. */
  center: Vec2;
  bodies: {
    tmp: number;
    controlWorld: Vec2[];
    /** Hole shapes in world coords (control polygon + rounding each). */
    holes: { control: Vec2[]; radius: number; radii?: (number | null)[]; round?: RoundMode }[];
    radius: number;
    radii?: (number | null)[];
    round: RoundMode;
    color: string;
    grounded: boolean;
  }[];
  /** Copied joints: attached ones carry their body's tmp id, free ones null. */
  joints: { tmp: number; bodyTmp: number | null; world: Vec2 }[];
  grounds: { joint: number; anchor: Vec2 }[];
  sliders: { tmp: number; railA: number; railB: number; riders: number[]; locked: number[] }[];
  pins: { a: number; b: number; rigid?: boolean }[];
  /** Powered constraints fully internal to the clip (slider/rider — body/joints — copied). */
  actuators: { slider: number; rider: number; speed: number; profile: "triangle" | "sine" }[];
  motors: { body: number; pivot: number; crank: number; speed: number }[];
  /** Permanent groups among the copied members (body / free-joint tmp ids). */
  groups: { bodies: number[]; joints: number[] }[];
  /** Fully-internal sketch constraints; refs carry the original ids, remapped on paste. */
  sketch: { kind: SketchConstraintKind; refA: MeasureRef; refB: MeasureRef | null }[];
  /** Fully-internal draw-mode dimensions; refs carry the original ids. Driving ones carry
   *  a `target`; driven (reference) ones travel only when the clip asks for them. */
  dims: { refA: MeasureRef; refB: MeasureRef; labelOffset: Vec2; axis: MeasureAxis; target?: number }[];
  /**
   * Patterns of the copied bodies (v19): the seed (hole index / joint tmp id), the layout
   * with its vectors in **world** orientation (a pasted body is re-baked at angle 0),
   * and the members (hole indices / joint tmp ids).
   */
  patterns: {
    tmp: number;
    body: number;
    seed: { kind: "hole"; hole: number } | { kind: "joint"; joint: number };
    layout: PatternLayout;
    members: number[];
  }[];
}

const FORMAT_VERSION = 19;

/** Below this angle two measured lines count as parallel: show their distance, not the angle. */
const MEASURE_PARALLEL_TOL = (0.5 * Math.PI) / 180;

/** A driving dimension further than this from its target renders as violated (a bit
 *  above the sketch/pose solve tolerance of 1e-3 so a satisfied dim never flickers). */
export const DIM_VIOLATION_TOL = 2e-3;

/** Default speeds for newly-created actuators. */
const DEFAULT_LINEAR_ACTUATOR_SPEED = 0.5; // cycles per second (one back-and-forth every 2s)
const DEFAULT_MOTOR_SPEED = 0.25;          // revolutions per second (4s per turn)

const PALETTE = [
  "#4f9dff",
  "#ff7b54",
  "#5bd6a6",
  "#c98bff",
  "#ffd166",
  "#ff6b9d",
  "#46c2cb",
];

export class Scene {
  bodies: Body[] = [];
  joints: Joint[] = [];
  constraints: Constraint[] = [];
  measurements: Measurement[] = [];
  sketch: SketchConstraint[] = [];
  groups: BodyGroup[] = [];
  guides: Guide[] = [];
  /** Component definitions — document-level (kept across context switches). */
  components: ComponentDef[] = [];
  /** Component instances expanded into this context. */
  instances: ComponentInstance[] = [];
  /** Live patterns of this context (v19). */
  patterns: Pattern[] = [];
  /** Re-entrancy guard: a pattern sync's own rebuilds / removals must not re-sync. */
  private syncingPatterns = false;
  /** Working unit: 1 world unit = 1 of these (display + import conversion only). */
  unit: Unit = DEFAULT_UNIT;
  private nextId = 1;

  private id(): number {
    return this.nextId++;
  }

  /**
   * Create a body from a control polygon (world coords). `radius` rounds it: `fillet`
   * rounds the corners in place (keeps concavity); `offset` grows the convex hull outward.
   * `holesWorld` (optional) adds editable hole outlines — each either a plain loop
   * (becomes a radius-0 hole control polygon) or a control polygon + rounding
   * (see `HoleSpec` / `Body.holes`).
   * `radii` (optional) seeds per-corner overrides, one per vertex (see `Body.radii`).
   */
  addBody(
    worldVerts: Vec2[],
    radius = 0,
    round: RoundMode = "fillet",
    holesWorld?: HoleSpec[],
    radii?: (number | null)[]
  ): Body {
    const body: Body = {
      id: this.id(),
      controlLocal: worldVerts.map((p) => vec(p.x, p.y)), // world for now; rebuild re-centers it
      radius,
      round,
      local: [],
      pos: vec(0, 0),
      angle: 0,
      invMass: 1,
      invInertia: 1,
      color: PALETTE[this.bodies.length % PALETTE.length],
      grounded: false,
    };
    if (radii && radii.length === worldVerts.length && radii.some((r) => r !== null)) {
      body.radii = radii.map((r) => (typeof r === "number" ? Math.max(0, r) : null));
    }
    if (holesWorld?.length) {
      // World coords for now; rebuild re-centers them into the local frame.
      body.holes = holesWorld.map((h) => {
        if (Array.isArray(h)) return { controlLocal: h.map((p) => vec(p.x, p.y)), radius: 0 };
        const hole: BodyHole = {
          controlLocal: h.control.map((p) => vec(p.x, p.y)),
          radius: Math.max(0, h.radius ?? 0),
        };
        if (h.round) hole.round = h.round;
        if (h.radii && h.radii.length === h.control.length && h.radii.some((r) => r !== null)) {
          hole.radii = h.radii.map((r) => (typeof r === "number" ? Math.max(0, r) : null));
        }
        return hole;
      });
    }
    this.bodies.push(body);
    this.rebuildBody(body);
    return body;
  }

  /**
   * Recompute a body's render/physics polygon, centroid and mass from its control
   * polygon + radius, keeping attached joints anchored in world space (the centroid,
   * and thus the local frame, shifts when the shape changes).
   */
  private rebuildBody(body: Body): void {
    const ctrlWorld = body.controlLocal.map((p) => add(body.pos, rotate(p, body.angle)));
    const radii = body.radii ? this.bodyCornerRadii(body) : body.radius;
    const finalWorld =
      body.round === "offset"
        ? roundedConvexBody(ctrlWorld, radii)
        : filletPolygon(ctrlWorld, radii);
    // Holes derive exactly like the outer outline: each hole's sampled loop comes from
    // its own control polygon + rounding, then subtracts from the mass properties below.
    const holeCtrlWorld = (body.holes ?? []).map((h) =>
      h.controlLocal.map((p) => add(body.pos, rotate(p, body.angle)))
    );
    const holesWorld = (body.holes ?? []).map((h, hi) => deriveHoleOutline(holeCtrlWorld[hi], h));
    // Composite mass properties: outer minus holes (each hole via its own centroid +
    // parallel-axis shift). If bad data makes the holes outweigh the outer, fall back
    // to the outer-only properties rather than a zero/negative mass.
    const outerArea = Math.abs(polygonArea(finalWorld));
    const outerC = polygonCentroid(finalWorld);
    let area = outerArea;
    let cx = outerArea * outerC.x;
    let cy = outerArea * outerC.y;
    const holeProps = holesWorld.map((loop) => {
      const a = Math.abs(polygonArea(loop));
      const c = polygonCentroid(loop);
      return { a, c, i: polygonInertiaAboutCentroid(loop, c) };
    });
    for (const h of holeProps) {
      area -= h.a;
      cx -= h.a * h.c.x;
      cy -= h.a * h.c.y;
    }
    let centroid: Vec2;
    let inertia: number;
    if (area > 1e-9) {
      centroid = vec(cx / area, cy / area);
      inertia =
        polygonInertiaAboutCentroid(finalWorld, outerC) + outerArea * lenSq(sub(outerC, centroid));
      for (const h of holeProps) inertia -= h.i + h.a * lenSq(sub(h.c, centroid));
    } else {
      area = outerArea;
      centroid = outerC;
      inertia = polygonInertiaAboutCentroid(finalWorld, outerC);
    }
    const attached = this.joints.filter((j) => j.bodyId === body.id);
    const jointWorlds = attached.map((j) => this.jointWorld(j));
    body.pos = centroid;
    body.local = finalWorld.map((p) => rotate(sub(p, centroid), -body.angle));
    body.controlLocal = ctrlWorld.map((p) => rotate(sub(p, centroid), -body.angle));
    if (body.holes?.length) {
      body.holes.forEach((h, hi) => {
        h.controlLocal = holeCtrlWorld[hi].map((p) => rotate(sub(p, centroid), -body.angle));
      });
      body.holesLocal = holesWorld.map((loop) =>
        loop.map((p) => rotate(sub(p, centroid), -body.angle))
      );
    } else {
      delete body.holes;
      delete body.holesLocal;
    }
    body.invMass = 1 / Math.max(area, 1);
    body.invInertia = 1 / Math.max(inertia, 1);
    attached.forEach((j, i) => {
      j.local = rotate(sub(jointWorlds[i], centroid), -body.angle);
    });
    // Any pattern on this body re-derives its members from the (possibly changed) seed.
    this.syncPatternsOfBody(body.id);
  }

  /**
   * Move a control vertex of a body (or of its hole `hole`) by a world-space delta,
   * then rebuild its shape. A joint of this body sitting exactly on the vertex is
   * *stuck* to it and carried along (a body built from joints keeps its joints and
   * control nodes together); all other attached joints stay anchored in world space.
   */
  moveBodyVertex(bodyId: number, index: number, delta: Vec2, hole?: number | null): void {
    const body = this.getBody(bodyId);
    const ctrl = body ? this.controlListOf(body, hole) : null;
    if (!body || !ctrl || index < 0 || index >= ctrl.length) return;
    const vw = add(body.pos, rotate(ctrl[index], body.angle));
    const linked = this.joints.filter(
      (j) => j.bodyId === bodyId && dist(this.jointWorld(j), vw) < VERTEX_LINK_EPS
    );
    ctrl[index] = add(ctrl[index], rotate(delta, -body.angle));
    this.rebuildBody(body); // keeps every joint anchored...
    for (const j of linked) this.shiftJoint(j, delta); // ...then the stuck ones follow
  }

  /** The control polygon `hole` names: the hole's, or the outer one when null/absent. */
  private controlListOf(body: Body, hole?: number | null): Vec2[] | null {
    if (hole === null || hole === undefined) return body.controlLocal;
    return body.holes?.[hole]?.controlLocal ?? null;
  }

  /**
   * Insert a new control vertex at `index` (world position → local) into the body's
   * outer outline or its hole `hole`, then rebuild. Used to add a node on a polygon
   * edge: pass the index it should occupy (i.e. the later endpoint of the clicked edge).
   */
  insertBodyVertex(bodyId: number, index: number, worldPos: Vec2, hole?: number | null): void {
    const body = this.getBody(bodyId);
    const ctrl = body ? this.controlListOf(body, hole) : null;
    if (!body || !ctrl) return;
    const clamped = Math.min(Math.max(index, 0), ctrl.length);
    ctrl.splice(clamped, 0, rotate(sub(worldPos, body.pos), -body.angle));
    const radii = hole === null || hole === undefined ? body.radii : body.holes![hole].radii;
    if (radii) radii.splice(clamped, 0, null); // the new corner uses the outline default
    this.shiftMeasureIndices(bodyId, clamped, 1, hole ?? null);
    this.rebuildBody(body);
  }

  /**
   * Remove a control vertex from the outer outline or hole `hole`, then rebuild.
   * No-op if it would leave fewer than 3 vertices (1 for an offset-mode hole, which
   * is a disk) or the index is out of range.
   */
  removeBodyVertex(bodyId: number, index: number, hole?: number | null): void {
    const body = this.getBody(bodyId);
    const ctrl = body ? this.controlListOf(body, hole) : null;
    if (!body || !ctrl) return;
    const holeShape = hole === null || hole === undefined ? null : body.holes![hole];
    const min = holeShape?.round === "offset" ? 1 : 3;
    if (ctrl.length <= min || index < 0 || index >= ctrl.length) return;
    ctrl.splice(index, 1);
    const shape: { radii?: (number | null)[] } = holeShape ?? body;
    if (shape.radii) {
      shape.radii.splice(index, 1);
      if (!shape.radii.some((r) => r !== null)) delete shape.radii;
    }
    this.shiftMeasureIndices(bodyId, index, -1, hole ?? null);
    this.rebuildBody(body);
  }

  /**
   * Add an editable hole outline to an existing body (world coords — same `HoleSpec`
   * shapes as `addBody`'s `holesWorld`), then rebuild. Appended after any existing
   * holes, so refs to them keep their indices. Returns the new hole's index, or null
   * when rejected (missing body, or too few control points: an offset-mode hole needs
   * ≥ 1 — a disk — and a fillet-mode one ≥ 3).
   */
  addBodyHole(bodyId: number, spec: HoleSpec): number | null {
    const body = this.getBody(bodyId);
    if (!body) return null;
    const control = Array.isArray(spec) ? spec : spec.control;
    const round = Array.isArray(spec) ? undefined : spec.round;
    if (control.length < (round === "offset" ? 1 : 3)) return null;
    const hole: BodyHole = {
      controlLocal: control.map((p) => rotate(sub(p, body.pos), -body.angle)),
      radius: Array.isArray(spec) ? 0 : Math.max(0, spec.radius ?? 0),
    };
    if (round) hole.round = round;
    if (!Array.isArray(spec) && spec.radii && spec.radii.length === control.length &&
        spec.radii.some((r) => r !== null)) {
      hole.radii = spec.radii.map((r) => (typeof r === "number" ? Math.max(0, r) : null));
    }
    body.holes = [...(body.holes ?? []), hole];
    this.rebuildBody(body);
    return body.holes.length - 1;
  }

  /**
   * Remove a whole hole from a body, then rebuild. Measurements / sketch constraints
   * on the hole (and index remaps for later holes' refs) cascade.
   */
  removeBodyHole(bodyId: number, hole: number): void {
    const body = this.getBody(bodyId);
    if (!body || !body.holes || hole < 0 || hole >= body.holes.length) return;
    // A pattern member is derived geometry: deleting one deletes the whole array (the
    // seed stays). Removing a seed dissolves its pattern inside dropHoles (members stay).
    const ph = this.patternOfHole(bodyId, hole);
    if (ph?.role === "member") {
      this.removePattern(ph.pattern.id);
      return;
    }
    this.dropHoles(body, [hole]);
  }

  // --- patterns (live linear / circular arrays of a hole or an attached joint) --------

  getPattern(id: number): Pattern | undefined {
    return this.patterns.find((p) => p.id === id);
  }

  /** The pattern a hole of `bodyId` belongs to, and whether it is the seed or a member. */
  patternOfHole(bodyId: number, hole: number): { pattern: Pattern; role: "seed" | "member" } | undefined {
    for (const p of this.patterns) {
      if (p.bodyId !== bodyId || p.seed.kind !== "hole") continue;
      if (p.seed.hole === hole) return { pattern: p, role: "seed" };
      if (p.members.includes(hole)) return { pattern: p, role: "member" };
    }
    return undefined;
  }

  /** The pattern a joint belongs to, and whether it is the seed or a member. */
  patternOfJoint(jointId: number): { pattern: Pattern; role: "seed" | "member" } | undefined {
    for (const p of this.patterns) {
      if (p.seed.kind !== "joint") continue;
      if (p.seed.jointId === jointId) return { pattern: p, role: "seed" };
      if (p.members.includes(jointId)) return { pattern: p, role: "member" };
    }
    return undefined;
  }

  /** Whether a measurement reference names a pattern **member** (derived geometry). */
  refPatternMember(ref: MeasureRef): boolean {
    if (ref.kind === "joint") return this.patternOfJoint(ref.jointId)?.role === "member";
    if ((ref.kind === "vertex" || ref.kind === "edge") && ref.hole !== undefined) {
      return this.patternOfHole(ref.bodyId, ref.hole)?.role === "member";
    }
    return false;
  }

  /** The pattern a reference's element belongs to (as seed or member), or undefined. */
  patternOfRef(ref: MeasureRef): Pattern | undefined {
    if (ref.kind === "joint") return this.patternOfJoint(ref.jointId)?.pattern;
    if ((ref.kind === "vertex" || ref.kind === "edge") && ref.hole !== undefined) {
      return this.patternOfHole(ref.bodyId, ref.hole)?.pattern;
    }
    return undefined;
  }

  /** A hole's index redirected to its pattern's seed when it is a member (else itself). */
  patternSeedHole(bodyId: number, hole: number): number {
    const ph = this.patternOfHole(bodyId, hole);
    return ph?.role === "member" && ph.pattern.seed.kind === "hole" ? ph.pattern.seed.hole : hole;
  }

  /** The seed's anchor in its body's local frame: a hole's control centroid, or the joint. */
  private patternAnchorLocal(p: Pattern): Vec2 | null {
    const body = this.getBody(p.bodyId);
    if (!body) return null;
    if (p.seed.kind === "joint") {
      const j = this.getJoint(p.seed.jointId);
      return j && j.bodyId === body.id ? j.local : null;
    }
    const hole = body.holes?.[p.seed.hole];
    if (!hole) return null;
    return hole.controlLocal.length < 3 ? centroidOfPoints(hole.controlLocal) : polygonCentroid(hole.controlLocal);
  }

  /** The world anchor of a pattern seed picked by the UI, or null (missing, free joint). */
  patternSeedAnchor(seed: PatternSeed): Vec2 | null {
    const body = this.patternSeedBody(seed);
    if (!body) return null;
    const p = this.seedRecord(seed);
    const a = this.patternAnchorLocal({ id: 0, bodyId: body.id, seed: p, layout: { kind: "linear", axes: [] }, members: [] });
    return a ? add(body.pos, rotate(a, body.angle)) : null;
  }

  /** The body a UI seed lives on (an attached joint's body, the hole's body), or undefined. */
  patternSeedBody(seed: PatternSeed): Body | undefined {
    if (seed.kind === "joint") {
      const j = this.getJoint(seed.jointId);
      return j && j.bodyId !== null ? this.getBody(j.bodyId) : undefined;
    }
    const body = this.getBody(seed.bodyId);
    return body?.holes?.[seed.hole] ? body : undefined;
  }

  private seedRecord(seed: PatternSeed): Pattern["seed"] {
    return seed.kind === "joint" ? { kind: "joint", jointId: seed.jointId } : { kind: "hole", hole: seed.hole };
  }

  /** A body-local vector from a world one. */
  private toLocalVec(body: Body, worldVec: Vec2): Vec2 {
    return rotate(worldVec, -body.angle);
  }

  /**
   * Create a linear pattern: the seed is repeated `count` times towards `target` (the
   * world point the 2nd instance lands on). Null when the seed is invalid, already in a
   * pattern, or the target coincides with the seed.
   */
  createLinearPattern(seed: PatternSeed, target: Vec2, count = 3): Pattern | null {
    const body = this.patternSeedBody(seed);
    const anchor = body ? this.patternSeedAnchor(seed) : null;
    if (!body || !anchor || this.seedTaken(seed)) return null;
    const step = this.toLocalVec(body, sub(target, anchor));
    if (len(step) < 1e-6) return null;
    const p: Pattern = {
      id: this.id(),
      bodyId: body.id,
      seed: this.seedRecord(seed),
      layout: { kind: "linear", axes: [{ count: this.clampCount(count), step }] },
      members: [],
    };
    this.patterns.push(p);
    this.syncPattern(p);
    return p;
  }

  /**
   * Add the second axis of a linear pattern (turning the row into a grid): `target` is
   * where the first instance along the new axis lands. Refused for circular / already
   * two-axis patterns and for a direction parallel to the first axis.
   */
  addPatternAxis(id: number, target: Vec2, count = 3): boolean {
    const p = this.getPattern(id);
    const body = p ? this.getBody(p.bodyId) : undefined;
    const anchorL = p ? this.patternAnchorLocal(p) : null;
    if (!p || !body || !anchorL || p.layout.kind !== "linear" || p.layout.axes.length !== 1) return false;
    const anchor = add(body.pos, rotate(anchorL, body.angle));
    const step = this.toLocalVec(body, sub(target, anchor));
    const a0 = p.layout.axes[0].step;
    if (len(step) < 1e-6 || Math.abs(cross(normalize(a0), normalize(step))) < 1e-3) return false;
    p.layout.axes.push({ count: this.clampCount(count, p.layout.axes[0].count), step });
    this.syncPattern(p);
    return true;
  }

  /**
   * Create a circular pattern of `count` instances evenly around `centre` (world), each
   * rotated with the arc. Null when the seed is invalid / taken or the centre sits on it.
   */
  createCircularPattern(seed: PatternSeed, centre: Vec2, count = 6): Pattern | null {
    const body = this.patternSeedBody(seed);
    const anchor = body ? this.patternSeedAnchor(seed) : null;
    if (!body || !anchor || this.seedTaken(seed)) return null;
    const off = this.toLocalVec(body, sub(centre, anchor));
    if (len(off) < 1e-6) return null;
    const p: Pattern = {
      id: this.id(),
      bodyId: body.id,
      seed: this.seedRecord(seed),
      layout: { kind: "circular", centre: off, count: this.clampCount(count), rotate: true },
      members: [],
    };
    this.patterns.push(p);
    this.syncPattern(p);
    return p;
  }

  /** Whether a UI seed already belongs to a pattern (as seed or member). */
  private seedTaken(seed: PatternSeed): boolean {
    return seed.kind === "joint"
      ? this.patternOfJoint(seed.jointId) !== undefined
      : this.patternOfHole(seed.bodyId, seed.hole) !== undefined;
  }

  /** Clamp an instance count (≥ 2, ≤ the axis cap, and the grid cap given the other axis). */
  private clampCount(count: number, otherAxisCount = 1): number {
    const n = Math.floor(Number.isFinite(count) ? count : 2);
    const gridCap = Math.max(2, Math.floor(PATTERN_MAX_MEMBERS / Math.max(1, otherAxisCount)));
    return Math.min(PATTERN_MAX_COUNT, gridCap, Math.max(2, n));
  }

  /** Set the instance count along a linear axis (≥ 2). */
  setPatternAxisCount(id: number, axis: number, count: number): boolean {
    const p = this.getPattern(id);
    if (!p || p.layout.kind !== "linear" || !p.layout.axes[axis]) return false;
    const other = p.layout.axes[1 - axis]?.count ?? 1;
    p.layout.axes[axis].count = this.clampCount(count, other);
    this.syncPattern(p);
    return true;
  }

  /** Set the spacing along a linear axis (world units, > 0); the direction is kept. */
  setPatternAxisStep(id: number, axis: number, length: number): boolean {
    const p = this.getPattern(id);
    if (!p || p.layout.kind !== "linear" || !p.layout.axes[axis] || !(length > 0)) return false;
    const a = p.layout.axes[axis];
    a.step = scale(normalize(a.step), length);
    this.syncPattern(p);
    return true;
  }

  /**
   * Re-aim a linear axis so its **last** instance lands on `end` (world) — direction and
   * spacing both follow. Refused when `end` is on the anchor or parallel to the other axis.
   */
  setPatternAxisEnd(id: number, axis: number, end: Vec2): boolean {
    const p = this.getPattern(id);
    const body = p ? this.getBody(p.bodyId) : undefined;
    const anchorL = p ? this.patternAnchorLocal(p) : null;
    if (!p || !body || !anchorL || p.layout.kind !== "linear" || !p.layout.axes[axis]) return false;
    const a = p.layout.axes[axis];
    const anchor = add(body.pos, rotate(anchorL, body.angle));
    const step = scale(this.toLocalVec(body, sub(end, anchor)), 1 / (a.count - 1));
    if (len(step) < 1e-6) return false;
    const other = p.layout.axes[1 - axis];
    if (other && Math.abs(cross(normalize(other.step), normalize(step))) < 1e-3) return false;
    a.step = step;
    this.syncPattern(p);
    return true;
  }

  /**
   * Set a linear axis's step from a **world** vector (the seed anchor → the next
   * instance) — the sketch solver's writeback for an axis line it re-aimed.
   */
  setPatternAxisVector(id: number, axis: number, stepWorld: Vec2): boolean {
    const p = this.getPattern(id);
    const body = p ? this.getBody(p.bodyId) : undefined;
    if (!p || !body || p.layout.kind !== "linear" || !p.layout.axes[axis] || len(stepWorld) < 1e-6) return false;
    p.layout.axes[axis].step = this.toLocalVec(body, stepWorld);
    this.syncPattern(p);
    return true;
  }

  /** Set a circular pattern's instance count (≥ 2). */
  setPatternCount(id: number, count: number): boolean {
    const p = this.getPattern(id);
    if (!p || p.layout.kind !== "circular") return false;
    p.layout.count = this.clampCount(count);
    this.syncPattern(p);
    return true;
  }

  /**
   * Set a circular pattern's angular step in **screen** degrees (counter-clockwise
   * positive; world y points down, so the stored radians are negated), or `null` to
   * spread the instances evenly over the full circle. A zero angle is refused.
   */
  setPatternAngle(id: number, degrees: number | null): boolean {
    const p = this.getPattern(id);
    if (!p || p.layout.kind !== "circular") return false;
    if (degrees === null) delete p.layout.angleStep;
    else {
      if (!Number.isFinite(degrees) || Math.abs(degrees) < 1e-6) return false;
      p.layout.angleStep = (-degrees * Math.PI) / 180;
    }
    this.syncPattern(p);
    return true;
  }

  /** Whether a circular pattern's instances turn with the arc (else they keep the seed's orientation). */
  setPatternRotate(id: number, rotateWithArc: boolean): boolean {
    const p = this.getPattern(id);
    if (!p || p.layout.kind !== "circular") return false;
    p.layout.rotate = rotateWithArc;
    this.syncPattern(p);
    return true;
  }

  /** Move a circular pattern's centre to a world point (refused on the seed's anchor). */
  setPatternCentre(id: number, centre: Vec2): boolean {
    const p = this.getPattern(id);
    const body = p ? this.getBody(p.bodyId) : undefined;
    const anchorL = p ? this.patternAnchorLocal(p) : null;
    if (!p || !body || !anchorL || p.layout.kind !== "circular") return false;
    const anchor = add(body.pos, rotate(anchorL, body.angle));
    const off = this.toLocalVec(body, sub(centre, anchor));
    if (len(off) < 1e-6) return false;
    p.layout.centre = off;
    this.syncPattern(p);
    return true;
  }

  /** Delete a pattern **and its members**; the seed stays as a plain hole / joint. */
  removePattern(id: number): void {
    const p = this.getPattern(id);
    if (!p) return;
    this.patterns = this.patterns.filter((x) => x.id !== id); // record first: no hooks re-enter
    const body = this.getBody(p.bodyId);
    if (p.seed.kind === "hole") {
      if (body) this.dropHoles(body, p.members);
    } else {
      for (const jid of p.members) this.removeJoint(jid);
    }
    this.prunePatternRefs();
  }

  /** Make a pattern's members independent: the record goes, every hole / joint stays. */
  dissolvePattern(id: number): void {
    this.patterns = this.patterns.filter((x) => x.id !== id);
    this.prunePatternRefs();
  }

  /** Dissolve every pattern on a body (the body's holes / joints become plain again). */
  private dissolvePatternsOfBody(bodyId: number): void {
    this.patterns = this.patterns.filter((p) => p.bodyId !== bodyId);
    this.prunePatternRefs();
  }

  /** Drop sketch constraints / measurements whose pattern-axis reference is gone. */
  private prunePatternRefs(): void {
    const dead = (r: MeasureRef | null): boolean =>
      !!r && r.kind === "patternAxis" && !this.resolveMeasureRef(r);
    this.sketch = this.sketch.filter((c) => !dead(c.refA) && !dead(c.refB));
    this.measurements = this.measurements.filter((mm) => !dead(mm.refA) && !dead(mm.refB));
  }

  /**
   * Re-derive a pattern's members from its seed + layout: the member list grows or
   * shrinks to the layout's count, and every member takes the seed's shape (holes:
   * control polygon, radius, per-corner radii, rounding) carried by its motion.
   * Called from the body / joint edit paths whenever the seed may have changed.
   */
  private syncPattern(p: Pattern): void {
    if (this.syncingPatterns) return;
    const body = this.getBody(p.bodyId);
    const anchor = body ? this.patternAnchorLocal(p) : null;
    if (!body || !anchor) return;
    const motions = patternLocalMotions(p.layout, anchor);
    this.syncingPatterns = true;
    try {
      if (p.seed.kind === "hole") {
        const seed = body.holes![p.seed.hole];
        if (p.members.length > motions.length) {
          const extra = p.members.slice(motions.length);
          p.members = p.members.slice(0, motions.length);
          this.dropHoles(body, extra); // remaps this pattern's surviving member indices too
        }
        while (p.members.length < motions.length) {
          body.holes!.push({ controlLocal: [], radius: seed.radius });
          p.members.push(body.holes!.length - 1);
        }
        p.members.forEach((hi, k) => {
          const h = body.holes![hi];
          h.controlLocal = seed.controlLocal.map(motions[k]);
          h.radius = seed.radius;
          if (seed.radii) h.radii = [...seed.radii];
          else delete h.radii;
          if (seed.round) h.round = seed.round;
          else delete h.round;
        });
        this.rebuildBody(body);
      } else {
        const seedJ = this.getJoint(p.seed.jointId)!;
        if (p.members.length > motions.length) {
          const extra = p.members.slice(motions.length);
          p.members = p.members.slice(0, motions.length);
          for (const jid of extra) this.removeJoint(jid);
        }
        while (p.members.length < motions.length) {
          const j: Joint = { id: this.id(), bodyId: body.id, local: vec(anchor.x, anchor.y) };
          this.joints.push(j);
          p.members.push(j.id);
        }
        p.members.forEach((jid, k) => {
          const j = this.getJoint(jid);
          if (!j) return;
          j.local = motions[k](seedJ.local);
          const w = this.jointWorld(j);
          for (const c of this.constraints) {
            if (c.kind === "ground" && c.joint === j.id) c.anchor = vec(w.x, w.y);
          }
        });
      }
    } finally {
      this.syncingPatterns = false;
    }
  }

  /** Re-derive every pattern of a body (a shape / seed change) — the rebuild hook. */
  private syncPatternsOfBody(bodyId: number): void {
    if (this.syncingPatterns) return;
    for (const p of this.patterns) if (p.bodyId === bodyId) this.syncPattern(p);
  }

  /**
   * Remove several holes of a body at once. Measurement / sketch refs on a removed hole
   * are dropped and refs on later holes shift down; patterns on the body follow the same
   * renumbering — one whose **seed** is removed dissolves (its members stay as plain holes).
   */
  private dropHoles(body: Body, gone: number[]): void {
    const set = new Set(gone.filter((h) => body.holes && h >= 0 && h < body.holes.length));
    if (!body.holes || set.size === 0) return;
    const map = new Map<number, number>();
    let k = 0;
    for (let i = 0; i < body.holes.length; i++) if (!set.has(i)) map.set(i, k++);
    body.holes = body.holes.filter((_, i) => !set.has(i));
    body.holesLocal = body.holesLocal?.filter((_, i) => !set.has(i));
    if (body.holes.length === 0) {
      delete body.holes;
      delete body.holesLocal;
    }
    const onHole = (ref: MeasureRef | null): ref is MeasureRef & { hole: number } =>
      !!ref && (ref.kind === "vertex" || ref.kind === "edge") && ref.bodyId === body.id && ref.hole !== undefined;
    const refGone = (ref: MeasureRef | null): boolean => onHole(ref) && set.has(ref.hole);
    const remap = (ref: MeasureRef | null): void => {
      if (onHole(ref)) ref.hole = map.get(ref.hole)!;
    };
    this.measurements = this.measurements.filter((m) => !refGone(m.refA) && !refGone(m.refB));
    this.sketch = this.sketch.filter((c) => !refGone(c.refA) && !refGone(c.refB));
    for (const m of this.measurements) { remap(m.refA); remap(m.refB); }
    for (const c of this.sketch) { remap(c.refA); remap(c.refB); }
    this.patterns = this.patterns.filter((p) => {
      if (p.bodyId !== body.id || p.seed.kind !== "hole") return true;
      if (set.has(p.seed.hole)) return false; // seed gone → the pattern dissolves
      p.seed.hole = map.get(p.seed.hole)!;
      p.members = p.members.filter((h) => !set.has(h)).map((h) => map.get(h)!);
      return true;
    });
    this.prunePatternRefs(); // a dissolved pattern's axis constraints go with it
    this.rebuildBody(body);
  }

  /** Whether a hole loop of `body` fits: inside the outer outline and clear of every other hole. */
  private holeLoopFits(outer: Vec2[], loop: Vec2[], others: Vec2[][]): boolean {
    if (!loop.every((p) => pointInPolygon(p, outer))) return false;
    const bb = boundsOf(loop);
    return !others.some((o) => {
      const ob = boundsOf(o);
      if (bb.min.x > ob.max.x || bb.max.x < ob.min.x || bb.min.y > ob.max.y || bb.max.y < ob.min.y) return false;
      return loop.some((p) => pointInPolygon(p, o)) || o.some((p) => pointInPolygon(p, loop));
    });
  }

  /** A pattern resolved to world geometry (labels, handles, member fit), or null if broken. */
  patternInfo(id: number): PatternInfo | null {
    const p = this.getPattern(id);
    const body = p ? this.getBody(p.bodyId) : undefined;
    const anchorL = p ? this.patternAnchorLocal(p) : null;
    if (!p || !body || !anchorL) return null;
    const toWorld = (q: Vec2): Vec2 => add(body.pos, rotate(q, body.angle));
    const anchor = toWorld(anchorL);
    const outer = this.bodyWorldVerts(body);
    let members: PatternInfo["members"];
    if (p.seed.kind === "hole") {
      const loops = this.bodyHolesWorld(body);
      members = p.members.map((hi) => {
        const loop = loops[hi];
        const others = loops.filter((_, i) => i !== hi);
        return { point: toWorld(centroidOfPoints(body.holes![hi].controlLocal)), ok: !!loop && this.holeLoopFits(outer, loop, others) };
      });
    } else {
      const onBody = this.joints.filter((j) => j.bodyId === body.id);
      members = p.members.map((jid) => {
        const j = this.getJoint(jid);
        const w = j ? this.jointWorld(j) : anchor;
        const ok =
          !!j && pointInPolygon(w, outer) &&
          !onBody.some((o) => o.id !== jid && dist(this.jointWorld(o), w) <= PATTERN_JOINT_EPS);
        return { point: w, ok };
      });
    }
    if (p.layout.kind === "linear") {
      return {
        id: p.id, bodyId: body.id, kind: "linear", anchor, circular: null, members,
        axes: p.layout.axes.map((a) => ({
          end: add(anchor, rotate(scale(a.step, a.count - 1), body.angle)),
          count: a.count,
          step: len(a.step),
        })),
      };
    }
    const l = p.layout;
    return {
      id: p.id, bodyId: body.id, kind: "circular", anchor, axes: [], members,
      circular: {
        centre: add(anchor, rotate(l.centre, body.angle)),
        radius: len(l.centre),
        count: l.count,
        angleDeg: l.angleStep === undefined ? null : (-l.angleStep * 180) / Math.PI,
        rotate: l.rotate,
      },
    };
  }

  /**
   * Preview a layout for a UI seed (world coordinates in, world geometry out) without
   * creating anything: the seed's anchor / outline and every would-be instance with its
   * fit. `axis` previews a **second** axis added to an existing linear pattern.
   */
  patternPreview(
    seed: PatternSeed,
    spec:
      | { kind: "linear"; target: Vec2; count: number; axis?: number }
      | { kind: "circular"; centre: Vec2; count: number }
  ): PatternPreview | null {
    const body = this.patternSeedBody(seed);
    const anchor = body ? this.patternSeedAnchor(seed) : null;
    if (!body || !anchor) return null;
    const anchorL = rotate(sub(anchor, body.pos), -body.angle);
    let layout: PatternLayout;
    if (spec.kind === "linear") {
      const step = this.toLocalVec(body, sub(spec.target, anchor));
      if (len(step) < 1e-6) return null;
      const axes: PatternAxis[] = [];
      if (spec.axis !== undefined) {
        const p = this.getPattern(spec.axis);
        if (p?.layout.kind === "linear") axes.push(...p.layout.axes.map((a) => ({ count: a.count, step: a.step })));
        // A second direction parallel to the first is refused by addPatternAxis: no preview.
        if (axes[0] && Math.abs(cross(normalize(axes[0].step), normalize(step))) < 1e-3) return null;
      }
      axes.push({ count: this.clampCount(spec.count, axes[0]?.count ?? 1), step });
      layout = { kind: "linear", axes };
    } else {
      const off = this.toLocalVec(body, sub(spec.centre, anchor));
      if (len(off) < 1e-6) return null;
      layout = { kind: "circular", centre: off, count: this.clampCount(spec.count), rotate: true };
    }
    const motions = patternLocalMotions(layout, anchorL);
    const toWorld = (q: Vec2): Vec2 => add(body.pos, rotate(q, body.angle));
    const outer = this.bodyWorldVerts(body);
    if (seed.kind === "hole") {
      const existing = this.patternOfHole(seed.bodyId, seed.hole)?.pattern;
      const loops = this.bodyHolesWorld(body);
      // An existing pattern's members are being re-laid — they don't count as obstacles.
      const memberSet = new Set(existing?.members ?? []);
      const obstacles = loops.filter((_, i) => i !== seed.hole && !memberSet.has(i));
      const seedLocal = body.holesLocal![seed.hole];
      const instances = motions.map((m) => {
        const loop = seedLocal.map((q) => toWorld(m(q)));
        const ok = this.holeLoopFits(outer, loop, obstacles);
        if (ok) obstacles.push(loop);
        return { point: toWorld(m(anchorL)), loop, ok };
      });
      return { anchor, seedLoop: loops[seed.hole], instances };
    }
    const existing = this.patternOfJoint(seed.jointId)?.pattern;
    const memberSet = new Set(existing?.members ?? []);
    const taken = this.joints
      .filter((j) => j.bodyId === body.id && j.id !== seed.jointId && !memberSet.has(j.id))
      .map((j) => this.jointWorld(j));
    const instances = motions.map((m) => {
      const point = toWorld(m(anchorL));
      const ok = pointInPolygon(point, outer) && !taken.some((q) => dist(q, point) <= PATTERN_JOINT_EPS);
      if (ok) taken.push(point);
      return { point, loop: null, ok };
    });
    return { anchor, seedLoop: null, instances };
  }

  // --- split / combine ------------------------------------------------------

  /**
   * Scale-relative tolerance for split / combine geometry (locating cut points on the
   * outline, matching corners across a union), derived from the shapes involved.
   */
  private static shapeTol(loops: Vec2[][]): number {
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const l of loops) for (const p of l) {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
    }
    const diag = Number.isFinite(minX) ? Math.hypot(maxX - minX, maxY - minY) : 0;
    return Math.max(VERTEX_LINK_EPS, diag * 1e-7);
  }

  /**
   * The editable outline the split / combine tools operate on: the control polygon of a
   * fillet-mode body with its effective per-corner radii — or, for an offset-mode body
   * (whose rounded shape is *larger* than its control polygon), the sampled outline
   * **baked** as sharp corners (radius 0 everywhere). Holes likewise (an offset hole —
   * a disk — bakes to its sampled loop). World coordinates.
   */
  private editableOutline(body: Body): {
    outer: { p: Vec2; rad: number }[];
    radius: number;
    holes: { control: { p: Vec2; rad: number }[]; radius: number; baked: boolean }[];
  } {
    const baked = body.round === "offset";
    const outerRadii = this.bodyCornerRadii(body);
    const outer = baked
      ? this.bodyWorldVerts(body).map((p) => ({ p, rad: 0 }))
      : this.bodyControlWorld(body).map((p, i) => ({ p, rad: outerRadii[i] }));
    const sampledHoles = this.bodyHolesWorld(body);
    const holes = (body.holes ?? []).map((h, hi) => {
      if (h.round === "offset") {
        return { control: sampledHoles[hi].map((p) => ({ p, rad: 0 })), radius: 0, baked: true };
      }
      const radii = this.bodyCornerRadii(body, hi);
      return {
        control: this.bodyHoleControlWorld(body, hi).map((p, i) => ({ p, rad: radii[i] })),
        radius: h.radius,
        baked: false,
      };
    });
    return { outer, radius: baked ? 0 : body.radius, holes };
  }

  /** Per-corner override list for `addBody` / `Body.radii`: null where the corner uses the default. */
  private static overrides(rads: number[], def: number): (number | null)[] {
    return rads.map((r) => (Math.abs(r - def) < 1e-12 ? null : r));
  }

  /**
   * Split a body in two along `cut` — a world-space polyline whose first and last points
   * lie on the body's outer outline (on an edge, or exactly on a corner; the caller
   * projects the clicks there) and whose interior points lie inside the body. The
   * outline is cut at both ends and each side becomes a body: the original keeps its
   * id (side A), a new body takes the other side (side B — inserted right after A in the
   * z-order, same colour / grounded flag / group). Corners keep their radii; the two cut
   * points and the path vertices start sharp. An offset-mode body is baked first (see
   * `editableOutline`). Holes go to the side containing them (a cut through a hole is
   * rejected); joints go to the side containing them (a joint on the cut line stays with
   * A); a rail or motor whose two joints end up on different sides is dropped;
   * measurement / sketch refs to surviving corners and holes remap, refs on the two cut
   * edges are pruned. Component-instance bodies are refused.
   */
  splitBody(bodyId: number, cut: Vec2[]): SplitResult {
    const body = this.getBody(bodyId);
    if (!body) return { ok: false, reason: "The body no longer exists." };
    if (this.instanceOfBody(bodyId)) {
      return { ok: false, reason: "This body belongs to a component instance — edit the definition to split it." };
    }
    if (cut.length < 2) return { ok: false, reason: "A cut needs a start and an end on the outline." };
    const shape = this.editableOutline(body);
    const ctrl = shape.outer;
    const n = ctrl.length;
    const poly = ctrl.map((c) => c.p);
    const tol = Math.max(Scene.shapeTol([poly]) * 100, VERTEX_LINK_EPS);

    // Locate an end point on the outline: a corner when within tolerance, else its
    // projection onto the nearest edge (must be within tolerance of the outline).
    type Hit = { vertex: number } | { edge: number; t: number; p: Vec2 };
    const locate = (q: Vec2): Hit | null => {
      let bestV = -1, bestVD = tol;
      for (let i = 0; i < n; i++) {
        const d = dist(q, poly[i]);
        if (d < bestVD) { bestVD = d; bestV = i; }
      }
      if (bestV >= 0) return { vertex: bestV };
      let best: Hit | null = null, bestD = tol;
      for (let i = 0; i < n; i++) {
        const a = poly[i], b = poly[(i + 1) % n];
        const ab = sub(b, a);
        const l2 = lenSq(ab);
        if (l2 < 1e-18) continue;
        const t = Math.max(0, Math.min(1, dot(sub(q, a), ab) / l2));
        const p = add(a, scale(ab, t));
        const d = dist(q, p);
        if (d < bestD) { bestD = d; best = { edge: i, t, p }; }
      }
      return best;
    };
    const hS = locate(cut[0]);
    const hE = locate(cut[cut.length - 1]);
    if (!hS || !hE) return { ok: false, reason: "The cut must start and end on the body's outline." };
    const pS = "vertex" in hS ? poly[hS.vertex] : hS.p;
    const pE = "vertex" in hE ? poly[hE.vertex] : hE.p;
    if (dist(pS, pE) <= tol) return { ok: false, reason: "The cut starts and ends at the same point." };
    const interior = cut.slice(1, -1);

    // --- validate the cut path against the outline and the holes ---
    const path = [pS, ...interior, pE];
    for (const q of interior) {
      if (!pointInPolygon(q, poly)) return { ok: false, reason: "The cut leaves the body." };
    }
    for (let k = 0; k + 1 < path.length; k++) {
      const a = path[k], b = path[k + 1];
      if (dist(a, b) <= tol) return { ok: false, reason: "The cut has a zero-length segment." };
      for (let i = 0; i < n; i++) {
        if (segmentsCross(a, b, poly[i], poly[(i + 1) % n])) {
          return { ok: false, reason: "The cut crosses the body's outline." };
        }
      }
      // A segment lying along the outline (or outside a concave part) has its midpoint
      // outside/on the boundary; the end segments touch the outline at one point only.
      const mid = scale(add(a, b), 0.5);
      if (!pointInPolygon(mid, poly)) return { ok: false, reason: "The cut runs outside the body." };
      for (let m = k + 2; m + 1 < path.length; m++) {
        if (segmentsCross(a, b, path[m], path[m + 1])) return { ok: false, reason: "The cut crosses itself." };
      }
    }
    const holeLoops = this.bodyHolesWorld(body);
    for (const loop of holeLoops) {
      for (let k = 0; k + 1 < path.length; k++) {
        const a = path[k], b = path[k + 1];
        for (let i = 0; i < loop.length; i++) {
          const ha = loop[i], hb = loop[(i + 1) % loop.length];
          if (segmentsCross(a, b, ha, hb) || pointOnSegment(a, ha, hb, tol) || pointOnSegment(b, ha, hb, tol)) {
            return { ok: false, reason: "The cut runs through a hole — route it around (a cut must start and end on the outer outline)." };
          }
        }
      }
      for (const q of path) {
        if (pointInPolygon(q, loop)) return { ok: false, reason: "The cut runs through a hole." };
      }
    }

    // --- build the two loops (entries remember their original corner for ref remapping) ---
    type Entry = { p: Vec2; orig: number | null; rad: number };
    const P: Entry[] = [];
    let iS = -1, iE = -1;
    for (let i = 0; i < n; i++) {
      if ("vertex" in hS && hS.vertex === i) iS = P.length;
      if ("vertex" in hE && hE.vertex === i) iE = P.length;
      P.push({ p: poly[i], orig: i, rad: ctrl[i].rad });
      const inserts: { t: number; which: "S" | "E"; p: Vec2 }[] = [];
      if ("edge" in hS && hS.edge === i) inserts.push({ t: hS.t, which: "S", p: hS.p });
      if ("edge" in hE && hE.edge === i) inserts.push({ t: hE.t, which: "E", p: hE.p });
      inserts.sort((x, y) => x.t - y.t);
      for (const ins of inserts) {
        if (ins.which === "S") iS = P.length; else iE = P.length;
        P.push({ p: ins.p, orig: null, rad: 0 });
      }
    }
    const m = P.length;
    // The cut points are sharp on both sides (a corner cut in two keeps no fillet).
    const cutEntry = (e: Entry): Entry => ({ p: e.p, orig: e.orig, rad: 0 });
    const walk = (from: number, to: number): Entry[] => {
      const out: Entry[] = [];
      let i = from;
      for (let guard = 0; guard <= m; guard++) {
        out.push(i === from || i === to ? cutEntry(P[i]) : P[i]);
        if (i === to) break;
        i = (i + 1) % m;
      }
      return out;
    };
    const inner = interior.map((p): Entry => ({ p, orig: null, rad: 0 }));
    const loopA: Entry[] = [...walk(iS, iE), ...inner.slice().reverse()];
    const loopB: Entry[] = [...walk(iE, iS), ...inner];
    const areaTotal = Math.abs(polygonArea(poly));
    for (const l of [loopA, loopB]) {
      if (l.length < 3 || Math.abs(polygonArea(l.map((e) => e.p))) < 1e-6 * areaTotal) {
        return { ok: false, reason: "One side of the cut has no area — the cut runs along the outline." };
      }
    }
    const ptsA = loopA.map((e) => e.p);
    const ptsB = loopB.map((e) => e.p);

    // --- holes: each to the side that contains it ---
    const holesA: number[] = [], holesB: number[] = [];
    holeLoops.forEach((loop, hi) => (pointInPolygon(loop[0], ptsA) ? holesA : holesB).push(hi));
    // Holes carry over unchanged — their own editable spec (a disk stays a 1-point disk).
    const holeSpec = (hi: number): Exclude<HoleSpec, Vec2[]> => {
      const h = body.holes![hi];
      return {
        control: this.bodyHoleControlWorld(body, hi),
        radius: h.radius,
        radii: h.radii ? [...h.radii] : undefined,
        round: h.round,
      };
    };

    // bodyPoint refs on this body: remember world positions to re-anchor after the reshape.
    const bodyPointWorlds: { ref: { bodyId: number; local: Vec2 }; world: Vec2 }[] = [];
    for (const mm of this.measurements) {
      for (const ref of [mm.refA, mm.refB]) {
        if (ref.kind === "bodyPoint" && ref.bodyId === bodyId) {
          bodyPointWorlds.push({ ref, world: add(body.pos, rotate(ref.local, body.angle)) });
        }
      }
    }

    // Patterns are index-bound to this body's holes, which the split renumbers: their
    // members become plain holes / joints on whichever side they land.
    this.dissolvePatternsOfBody(bodyId);

    // --- side B: a new body right after A in the z-order ---
    const def = shape.radius;
    const bBody = this.addBody(
      ptsB, def, "fillet", holesB.map(holeSpec), Scene.overrides(loopB.map((e) => e.rad), def)
    );
    bBody.color = body.color;
    bBody.grounded = body.grounded;
    this.bodies = this.bodies.filter((b) => b !== bBody);
    this.bodies.splice(this.bodies.indexOf(body) + 1, 0, bBody);

    // --- joints: by containment (on the cut line → A) ---
    const attached = this.joints.filter((j) => j.bodyId === bodyId);
    const ownJoints = new Set(attached.map((j) => j.id));
    for (const j of attached) {
      const w = this.jointWorld(j);
      const onCut = path.some((_, k) => k + 1 < path.length && pointOnSegment(w, path[k], path[k + 1], tol));
      if (onCut || pointInPolygon(w, ptsA)) continue;
      j.bodyId = bBody.id;
      j.local = rotate(sub(w, bBody.pos), -bBody.angle);
    }

    // --- side A: reshape the original in place (joints already sorted out) ---
    const toLocal = (p: Vec2): Vec2 => rotate(sub(p, body.pos), -body.angle);
    body.round = "fillet";
    body.radius = def;
    body.controlLocal = ptsA.map(toLocal);
    const ovA = Scene.overrides(loopA.map((e) => e.rad), def);
    if (ovA.some((r) => r !== null)) body.radii = ovA; else delete body.radii;
    const newHolesA: BodyHole[] = holesA.map((hi) => {
      const spec = holeSpec(hi);
      const hole: BodyHole = { controlLocal: spec.control.map(toLocal), radius: spec.radius ?? 0 };
      if (spec.round) hole.round = spec.round;
      if (spec.radii && spec.radii.some((r) => r !== null)) hole.radii = spec.radii;
      return hole;
    });
    if (newHolesA.length) body.holes = newHolesA; else { delete body.holes; delete body.holesLocal; }
    this.rebuildBody(body);

    // --- constraints that needed both joints on one body ---
    for (const c of [...this.constraints]) {
      if (c.kind === "slider" && ownJoints.has(c.railA) && ownJoints.has(c.railB)) {
        if (this.getJoint(c.railA)!.bodyId !== this.getJoint(c.railB)!.bodyId) this.removeConstraint(c.id);
      } else if (c.kind === "motor" && c.bodyId === bodyId) {
        const pb = this.getJoint(c.pivotJointId)?.bodyId;
        const cb = this.getJoint(c.crankJointId)?.bodyId;
        if (pb === undefined || pb === null || pb !== cb) this.removeConstraint(c.id);
        else c.bodyId = pb;
      }
    }

    // --- measurement / sketch refs ---
    const vertexMap = new Map<number, { bodyId: number; index: number }>();
    const edgeMap = new Map<number, { bodyId: number; index: number }>();
    for (const [loop, owner] of [[loopA, body], [loopB, bBody]] as [Entry[], Body][]) {
      loop.forEach((e, i) => {
        if (e.orig !== null && !vertexMap.has(e.orig)) vertexMap.set(e.orig, { bodyId: owner.id, index: i });
        const nx = loop[(i + 1) % loop.length];
        if (e.orig !== null && nx.orig === (e.orig + 1) % n && !edgeMap.has(e.orig)) {
          edgeMap.set(e.orig, { bodyId: owner.id, index: i });
        }
      });
    }
    const holeMap = new Map<number, { bodyId: number; hole: number }>();
    holesA.forEach((hi, k) => holeMap.set(hi, { bodyId: body.id, hole: k }));
    holesB.forEach((hi, k) => holeMap.set(hi, { bodyId: bBody.id, hole: k }));
    const remap = (ref: MeasureRef | null): boolean => {
      if (!ref || (ref.kind !== "vertex" && ref.kind !== "edge") || ref.bodyId !== bodyId) return true;
      if (ref.hole !== undefined) {
        const h = holeMap.get(ref.hole);
        if (!h) return false;
        ref.bodyId = h.bodyId;
        ref.hole = h.hole;
        return true;
      }
      const hit = (ref.kind === "vertex" ? vertexMap : edgeMap).get(ref.index);
      if (!hit) return false;
      ref.bodyId = hit.bodyId;
      ref.index = hit.index;
      return true;
    };
    this.measurements = this.measurements.filter((mm) => remap(mm.refA) && remap(mm.refB));
    this.sketch = this.sketch.filter((c) => remap(c.refA) && remap(c.refB));
    for (const { ref, world } of bodyPointWorlds) {
      const owner = pointInPolygon(world, ptsA) ? body : bBody;
      ref.bodyId = owner.id;
      ref.local = rotate(sub(world, owner.pos), -owner.angle);
    }
    this.pruneMeasurements();
    this.pruneSketch();

    const g = this.groupOf(bodyId);
    if (g) g.bodyIds.push(bBody.id);
    return { ok: true, a: body, b: bBody };
  }

  /**
   * Combine two or more bodies into one: the first id survives (keeps its id, colour and
   * z-position) and takes the polygon **union** of every body's editable outline (see
   * `editableOutline`; offset-mode bodies are baked). Corners that survive unchanged keep
   * their radii, new intersection corners are sharp; holes not covered by other material
   * stay (partially covered ones shrink; a region the union encloses becomes a new
   * hole), and an untouched hole keeps its exact editable shape (a disk stays a disk).
   * Joints of the absorbed bodies re-attach to the survivor; pins / welds *between* the
   * combined bodies are removed (they'd be intra-body); motors move over; rails stay.
   * Groups touched are merged (the result grounded if any input was). Refs to surviving
   * corners remap, others are pruned.
   * Rejected when the bodies don't all connect (overlap or share an edge), touch only at
   * a point, or include component-instance material.
   */
  combineBodies(ids: number[]): CombineResult {
    const bodies: Body[] = [];
    for (const id of ids) {
      const b = this.getBody(id);
      if (b && !bodies.includes(b)) bodies.push(b);
    }
    if (bodies.length < 2) return { ok: false, reason: "Select at least two bodies to combine." };
    if (bodies.some((b) => this.instanceOfBody(b.id))) {
      return { ok: false, reason: "Component instances can't be combined — edit the definition, or fork the instance first." };
    }
    const shapes = bodies.map((b) => this.editableOutline(b));
    const regions: PolyRegion[] = shapes.map((s) => ({
      outer: s.outer.map((c) => c.p),
      holes: s.holes.map((h) => h.control.map((c) => c.p)),
    }));
    const tol = Scene.shapeTol(regions.flatMap((r) => [r.outer, ...r.holes])) * 10;
    const union = unionRegions(regions);
    if (!union) return { ok: false, reason: "The bodies have no area to combine." };
    if (union.pinched) {
      return { ok: false, reason: "The bodies touch only at a point — the combined outline would pinch there. Overlap them, or share an edge." };
    }
    if (union.regions.length > 1) {
      // Which bodies sit apart from the main piece? Each body is located by a point
      // just inside its first edge (its own vertices lie on the union boundary).
      const pieceOf = (r: PolyRegion): number => {
        const a = r.outer[0], b = r.outer[1];
        const d = normalize(sub(b, a));
        const mid = scale(add(a, b), 0.5);
        const off = Math.max(tol, dist(a, b) * 1e-4);
        const probe = [add(mid, scale(perp(d), off)), sub(mid, scale(perp(d), off))]
          .find((q) => pointInPolygon(q, r.outer)) ?? mid;
        return union.regions.findIndex((u) => pointInPolygon(probe, u.outer));
      };
      const counts = new Map<number, number>();
      const pieces = regions.map(pieceOf);
      for (const p of pieces) counts.set(p, (counts.get(p) ?? 0) + 1);
      const main = [...counts.entries()].sort((x, y) => y[1] - x[1])[0][0];
      const apart = pieces.filter((p) => p !== main).length;
      return {
        ok: false,
        reason: `${apart === 1 ? "One body doesn't" : `${apart} bodies don't`} touch the rest — bodies must overlap or share an edge to combine (${union.regions.length} separate pieces).`,
      };
    }
    const result = union.regions[0];
    const survivor = bodies[0];
    for (const b of bodies) this.dissolvePatternsOfBody(b.id); // hole indices are rebuilt below
    const absorbed = bodies.slice(1);

    // --- radii: a result corner that is an unchanged input corner keeps that radius ---
    const corners: { p: Vec2; prev: Vec2; next: Vec2; rad: number }[] = [];
    for (const s of shapes) {
      for (const loop of [s.outer, ...s.holes.map((h) => h.control)]) {
        const k = loop.length;
        if (k < 3) continue;
        loop.forEach((c, i) => corners.push({
          p: c.p, prev: loop[(i + k - 1) % k].p, next: loop[(i + 1) % k].p, rad: c.rad,
        }));
      }
    }
    const sameDir = (a: Vec2, b: Vec2): boolean => {
      const u = normalize(a), v = normalize(b);
      return dot(u, v) > 0 && Math.abs(cross(u, v)) < 1e-6;
    };
    const radiiOf = (loop: Vec2[]): number[] =>
      loop.map((p, i) => {
        const prev = loop[(i + loop.length - 1) % loop.length];
        const next = loop[(i + 1) % loop.length];
        for (const c of corners) {
          if (dist(c.p, p) > tol) continue;
          const fwd = sameDir(sub(next, p), sub(c.next, c.p)) && sameDir(sub(prev, p), sub(c.prev, c.p));
          const rev = sameDir(sub(next, p), sub(c.prev, c.p)) && sameDir(sub(prev, p), sub(c.next, c.p));
          if (fwd || rev) return c.rad;
        }
        return 0;
      });

    // --- holes: an untouched input hole keeps its exact editable spec ---
    const sameLoop = (a: Vec2[], b: Vec2[]): boolean =>
      a.length === b.length && a.every((p) => b.some((q) => dist(p, q) <= tol));
    type HoleOrigin = { bodyId: number; hole: number };
    type Spec = Exclude<HoleSpec, Vec2[]>;
    const holeSpecs: { spec: Spec; origin: HoleOrigin | null }[] = result.holes.map((loop) => {
      for (let bi = 0; bi < bodies.length; bi++) {
        const b = bodies[bi];
        for (let hi = 0; hi < shapes[bi].holes.length; hi++) {
          const h = shapes[bi].holes[hi];
          if (!sameLoop(loop, h.control.map((c) => c.p))) continue;
          const src = b.holes![hi];
          const control = h.baked ? this.bodyHoleControlWorld(b, hi) : h.control.map((c) => c.p);
          return {
            spec: { control, radius: src.radius, radii: src.radii ? [...src.radii] : undefined, round: src.round },
            origin: { bodyId: b.id, hole: hi },
          };
        }
      }
      return { spec: { control: loop, radius: 0, radii: Scene.overrides(radiiOf(loop), 0) }, origin: null };
    });

    // --- joints of the absorbed bodies move to the survivor (world positions kept) ---
    const moved = new Set<number>();
    for (const b of absorbed) {
      for (const j of this.joints) {
        if (j.bodyId !== b.id) continue;
        const w = this.jointWorld(j);
        j.bodyId = survivor.id;
        j.local = rotate(sub(w, survivor.pos), -survivor.angle);
        moved.add(j.id);
      }
    }
    // bodyPoint refs on any input: remember their world positions to re-anchor after.
    const bodyPoints: { ref: { bodyId: number; local: Vec2 }; world: Vec2 }[] = [];
    const inputIds = new Set(bodies.map((b) => b.id));
    for (const mm of this.measurements) {
      for (const ref of [mm.refA, mm.refB]) {
        if (ref.kind === "bodyPoint" && inputIds.has(ref.bodyId)) {
          const b = this.getBody(ref.bodyId)!;
          bodyPoints.push({ ref, world: add(b.pos, rotate(ref.local, b.angle)) });
        }
      }
    }
    // Vertex/edge refs: remember world geometry of the referenced corners to match after.
    type RefRec = { ref: MeasureRef & { kind: "vertex" | "edge" }; a: Vec2; b: Vec2 | null; origin: HoleOrigin | null };
    const refRecs: RefRec[] = [];
    const record = (ref: MeasureRef | null): void => {
      if (!ref || (ref.kind !== "vertex" && ref.kind !== "edge") || !inputIds.has(ref.bodyId)) return;
      const b = this.getBody(ref.bodyId)!;
      const ctrl = this.controlListOf(b, ref.hole ?? null);
      if (!ctrl || ref.index >= ctrl.length) return;
      const w = (i: number) => add(b.pos, rotate(ctrl[i % ctrl.length], b.angle));
      refRecs.push({
        ref,
        a: w(ref.index),
        b: ref.kind === "edge" ? w(ref.index + 1) : null,
        origin: ref.hole !== undefined ? { bodyId: ref.bodyId, hole: ref.hole } : null,
      });
    };
    for (const mm of this.measurements) { record(mm.refA); record(mm.refB); }
    for (const c of this.sketch) { record(c.refA); record(c.refB); }

    // --- survivor takes the union ---
    const def = shapes[0].radius;
    const toLocal = (p: Vec2): Vec2 => rotate(sub(p, survivor.pos), -survivor.angle);
    survivor.round = "fillet";
    survivor.radius = def;
    survivor.controlLocal = result.outer.map(toLocal);
    const ov = Scene.overrides(radiiOf(result.outer), def);
    if (ov.some((r) => r !== null)) survivor.radii = ov; else delete survivor.radii;
    const holes: BodyHole[] = holeSpecs.map(({ spec }) => {
      const hole: BodyHole = { controlLocal: spec.control.map(toLocal), radius: Math.max(0, spec.radius ?? 0) };
      if (spec.round) hole.round = spec.round;
      if (spec.radii && spec.radii.length === spec.control.length && spec.radii.some((r) => r !== null)) {
        hole.radii = [...spec.radii];
      }
      return hole;
    });
    if (holes.length) survivor.holes = holes; else { delete survivor.holes; delete survivor.holesLocal; }
    survivor.grounded = bodies.some((b) => b.grounded);
    this.rebuildBody(survivor);

    // --- constraints: motors follow their body; pins now internal to the survivor go ---
    for (const c of [...this.constraints]) {
      if (c.kind === "motor" && absorbed.some((b) => b.id === c.bodyId)) c.bodyId = survivor.id;
      if (c.kind === "pin") {
        const a = this.getJoint(c.jointA), b = this.getJoint(c.jointB);
        if (a && b && a.bodyId === survivor.id && b.bodyId === survivor.id && (moved.has(a.id) || moved.has(b.id))) {
          this.removeConstraint(c.id);
        }
      }
    }

    // --- groups: merge everything touched around the survivor ---
    const absorbedIds = new Set(absorbed.map((b) => b.id));
    const touched = bodies.map((b) => this.groupOf(b.id)).filter((g): g is BodyGroup => !!g);
    if (touched.length) {
      const gb = new Set<number>([survivor.id]);
      const gj = new Set<number>();
      for (const g of touched) {
        g.bodyIds.forEach((id) => { if (!absorbedIds.has(id)) gb.add(id); });
        g.jointIds.forEach((id) => gj.add(id));
      }
      this.groups = this.groups.filter((g) => !touched.includes(g));
      if (gb.size + gj.size >= 2) this.addGroup([...gb], [...gj]);
    }

    // --- drop the absorbed bodies (their joints already moved) ---
    this.bodies = this.bodies.filter((b) => !absorbedIds.has(b.id));

    // --- refs: match recorded world geometry against the new outline ---
    const outerW = this.bodyControlWorld(survivor);
    const holeW = (survivor.holes ?? []).map((_, hi) => this.bodyHoleControlWorld(survivor, hi));
    const findVertex = (p: Vec2): { hole: number | null; index: number } | null => {
      const i0 = outerW.findIndex((q) => dist(p, q) <= tol);
      if (i0 >= 0) return { hole: null, index: i0 };
      for (let hi = 0; hi < holeW.length; hi++) {
        const i = holeW[hi].findIndex((q) => dist(p, q) <= tol);
        if (i >= 0) return { hole: hi, index: i };
      }
      return null;
    };
    const ok = new Set<MeasureRef>();
    for (const rec of refRecs) {
      const { ref } = rec;
      // A ref on a hole carried over unchanged keeps its index (a disk's centre isn't on the loop).
      const kept = rec.origin
        ? holeSpecs.findIndex((h) => h.origin && h.origin.bodyId === rec.origin!.bodyId && h.origin.hole === rec.origin!.hole)
        : -1;
      if (kept >= 0) {
        ref.bodyId = survivor.id;
        ref.hole = kept;
        ok.add(ref);
        continue;
      }
      const va = findVertex(rec.a);
      if (!va) continue;
      if (ref.kind === "edge") {
        const loop = va.hole === null ? outerW : holeW[va.hole];
        const vb = rec.b ? findVertex(rec.b) : null;
        if (!vb || vb.hole !== va.hole) continue;
        const L = loop.length;
        let index: number;
        if ((va.index + 1) % L === vb.index) index = va.index;
        else if ((vb.index + 1) % L === va.index) index = vb.index;
        else continue;
        ref.bodyId = survivor.id;
        ref.index = index;
      } else {
        ref.bodyId = survivor.id;
        ref.index = va.index;
      }
      if (va.hole === null) delete ref.hole; else ref.hole = va.hole;
      ok.add(ref);
    }
    const stale = new Set<MeasureRef>(refRecs.map((r) => r.ref).filter((r) => !ok.has(r)));
    this.measurements = this.measurements.filter((mm) => !stale.has(mm.refA) && !stale.has(mm.refB));
    this.sketch = this.sketch.filter((c) => !stale.has(c.refA) && !(c.refB && stale.has(c.refB)));
    for (const { ref, world } of bodyPoints) {
      ref.bodyId = survivor.id;
      ref.local = rotate(sub(world, survivor.pos), -survivor.angle);
    }
    this.pruneMeasurements();
    this.pruneSketch();
    this.pruneGroups();
    this.pruneInstances();
    return { ok: true, body: survivor };
  }

  /**
   * Set a body's (or its hole `hole`'s) corner radius / margin (clamped ≥ 0), then
   * rebuild its shape. This is the outline-wide default; corners with a per-corner
   * override (`radii`) keep it.
   */
  setBodyRadius(bodyId: number, radius: number, hole?: number | null): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    const shape = hole === null || hole === undefined ? body : body.holes?.[hole];
    if (!shape) return;
    shape.radius = Math.max(0, radius);
    this.rebuildBody(body);
  }

  /**
   * Set a **disk** outline's radius (a one-point offset-mode outer outline or hole — see
   * `diskOfRef`). A disk has a single corner, so a per-corner override on it is
   * redundant and would shadow the default: this drops the override and sets the
   * default, so the effective radius is exactly `radius` however the disk was sized
   * before (rim-handle drag, diameter dimension, `[` / `]` keys). No-op for non-disks.
   */
  setDiskRadius(bodyId: number, radius: number, hole?: number | null): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    const h = hole ?? null;
    const shape = h === null ? body : body.holes?.[h];
    const ctrl = this.controlListOf(body, h);
    if (!shape || !ctrl || ctrl.length !== 1 || shape.round !== "offset") return;
    delete shape.radii;
    shape.radius = Math.max(0, radius);
    this.rebuildBody(body);
  }

  /** Effective radius of every corner (its override, or the outline default), parallel
   *  to the outline's control polygon. `hole` selects a hole; null/absent = the outer. */
  bodyCornerRadii(body: Body, hole?: number | null): number[] {
    const shape = hole === null || hole === undefined ? body : body.holes?.[hole];
    const ctrl = this.controlListOf(body, hole);
    if (!shape || !ctrl) return [];
    return ctrl.map((_, i) => {
      const r = shape.radii?.[i];
      return typeof r === "number" ? r : shape.radius;
    });
  }

  /**
   * Override one corner's radius / margin (clamped ≥ 0) on the outer outline or hole
   * `hole`, or clear the override back to the outline default with `null`, then
   * rebuild. The override array parallels the control polygon and is dropped entirely
   * once no corner is overridden any more.
   */
  setBodyCornerRadius(bodyId: number, index: number, radius: number | null, hole?: number | null): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    const shape = hole === null || hole === undefined ? body : body.holes?.[hole];
    const ctrl = this.controlListOf(body, hole);
    if (!shape || !ctrl || index < 0 || index >= ctrl.length) return;
    const src = shape.radii ?? [];
    const radii = ctrl.map((_, i) => (typeof src[i] === "number" ? src[i] : null));
    radii[index] = radius === null ? null : Math.max(0, radius);
    if (radii.some((r) => r !== null)) shape.radii = radii;
    else delete shape.radii;
    this.rebuildBody(body);
  }

  /** World positions of a hole's control vertices (like `bodyControlWorld`, for holes). */
  bodyHoleControlWorld(body: Body, hole: number): Vec2[] {
    const h = body.holes?.[hole];
    if (!h) return [];
    return h.controlLocal.map((p) => add(body.pos, rotate(p, body.angle)));
  }

  /**
   * The disk outline a vertex ref names, if that outline is a one-point offset-mode
   * outline (a circular hole, or a disk body): its centre is the ref's vertex and its
   * radius the outline's effective corner radius. Null for any other ref.
   */
  diskOfRef(ref: MeasureRef): DiskRef | null {
    if (ref.kind !== "vertex" || ref.index !== 0) return null;
    const body = this.getBody(ref.bodyId);
    if (!body) return null;
    const hole = ref.hole ?? null;
    const shape = hole === null ? body : body.holes?.[hole];
    const ctrl = this.controlListOf(body, hole);
    if (!shape || !ctrl || ctrl.length !== 1 || shape.round !== "offset") return null;
    return {
      bodyId: body.id,
      hole,
      c: add(body.pos, rotate(ctrl[0], body.angle)),
      r: this.bodyCornerRadii(body, hole)[0] ?? shape.radius,
    };
  }

  /**
   * The largest radius a disk centred at world point `c` can have while staying inside
   * `body`'s material: the distance from `c` to the nearest point of the body's sampled
   * outer outline and of every hole loop (`excludeHole` leaves one hole out — e.g. the
   * disk being resized). 0 when `c` is outside the body or inside a hole.
   */
  bodyInscribedRadius(body: Body, c: Vec2, excludeHole: number | null = null): number {
    const outer = this.bodyWorldVerts(body);
    if (!pointInPolygon(c, outer)) return 0;
    const holes = this.bodyHolesWorld(body);
    for (let hi = 0; hi < holes.length; hi++) {
      if (hi !== excludeHole && pointInPolygon(c, holes[hi])) return 0;
    }
    let best = Infinity;
    const scan = (loop: Vec2[]): void => {
      for (let i = 0; i < loop.length; i++) {
        best = Math.min(best, distToSegment(c, loop[i], loop[(i + 1) % loop.length]));
      }
    };
    scan(outer);
    holes.forEach((loop, hi) => {
      if (hi !== excludeHole) scan(loop);
    });
    return best === Infinity ? 0 : best;
  }

  addJoint(bodyId: number, worldPos: Vec2): Joint {
    const body = this.getBody(bodyId)!;
    const offset = sub(worldPos, body.pos);
    const local = rotate(offset, -body.angle);
    const joint: Joint = { id: this.id(), bodyId, local };
    this.joints.push(joint);
    return joint;
  }

  /** Create a free joint: a body-less point at `worldPos` that the solver can move. */
  addFreeJoint(worldPos: Vec2): Joint {
    const joint: Joint = { id: this.id(), bodyId: null, local: vec(worldPos.x, worldPos.y) };
    this.joints.push(joint);
    return joint;
  }

  /**
   * Build a body from existing joints, with its polygon the rounded convex hull of
   * those joints expanded outward by `margin`. Free joints are absorbed into the new
   * body; joints on other bodies stay put and get a new joint here pinned to them.
   * Returns the new body, or null if the joints can't form an area.
   */
  buildBodyFromJoints(jointIds: number[], margin: number): Body | null {
    const joints = jointIds
      .map((id) => this.getJoint(id))
      .filter((j): j is Joint => j !== undefined);
    if (joints.length < 2) return null;
    const worlds = joints.map((j) => this.jointWorld(j));
    // Store one control point per joint; the rounded outline (hull + outward offset) is
    // recomputed from these on every rebuild, rather than baking the hull in.
    const body = this.addBody(worlds, margin, "offset");
    if (body.local.length < 3) return null;
    joints.forEach((j, i) => {
      const w = worlds[i];
      // Does this joint belong to a slider — as a rail node or as a rider?
      const slider = this.constraints.find(
        (c) =>
          c.kind === "slider" &&
          (c.railA === j.id || c.railB === j.id || c.riders.includes(j.id))
      ) as SliderConstraint | undefined;
      const isRailNode = !!slider && (slider.railA === j.id || slider.railB === j.id);
      const grounded =
        j.bodyId === null &&
        this.constraints.some((c) => c.kind === "ground" && c.joint === j.id);
      if (slider && isRailNode) {
        // A slider rail node can't be folded into the new body: add a coincident joint and
        // confine it to the slider as a rider, so the body connects to the slider track
        // itself rather than being pinned to one of the rail's endpoints.
        const nj = this.addJoint(body.id, w);
        this.attachSliderRider(slider.id, nj.id);
      } else if (j.bodyId === null && !grounded && !this.groupOfJoint(j.id)) {
        // A loose free joint, or a free slider rider: absorb it. It now belongs to the new
        // body (angle 0 at creation); if it was a rider it stays one (rider ids are kept —
        // and a pending orientation lock activates now that the rider has a body).
        // A group-locked free joint is chassis material and stays independent (pinned below).
        j.bodyId = body.id;
        j.local = sub(w, body.pos);
      } else {
        // A joint on another body (including a rider on another body), or a grounded free
        // joint (an anchor we must keep independent): add a coincident joint here and pin
        // them together, so the new body is pinned to it (free to rotate) rather than
        // absorbing/grounding it. Pinning to another body's rider joins the two bodies at
        // that point and lets them ride the slider together through the shared pin.
        const nj = this.addJoint(body.id, w);
        this.addPin(nj.id, j.id);
      }
    });
    return body;
  }

  addPin(jointA: number, jointB: number, rigid = false): PinConstraint {
    const c: PinConstraint = { kind: "pin", id: this.id(), jointA, jointB };
    if (rigid) c.rigid = true;
    this.constraints.push(c);
    return c;
  }

  /** Toggle a pin between revolute (default) and weld (`rigid` — no relative rotation). */
  setPinRigid(pinId: number, rigid: boolean): void {
    const c = this.constraints.find((x) => x.id === pinId);
    if (!c || c.kind !== "pin") return;
    if (rigid) c.rigid = true;
    else delete c.rigid;
  }

  /** Every pin constraint a joint participates in (either end). */
  pinsOfJoint(jointId: number): PinConstraint[] {
    return this.constraints.filter(
      (c): c is PinConstraint => c.kind === "pin" && (c.jointA === jointId || c.jointB === jointId)
    );
  }

  addGround(joint: number, anchor: Vec2): GroundConstraint {
    const c: GroundConstraint = { kind: "ground", id: this.id(), joint, anchor };
    this.constraints.push(c);
    return c;
  }

  /**
   * Create a slider rail from two joints (`railA`/`railB`). Normally these are two joints
   * on the same body (a rail that moves with it). They may instead be two free joints, which
   * define a track fixed in world space — in that case each free rail joint must be anchored,
   * so any that isn't already grounded gets grounded here at its current position.
   */
  addSlider(railA: number, railB: number): SliderConstraint {
    for (const id of [railA, railB]) {
      const j = this.getJoint(id);
      if (
        j &&
        j.bodyId === null &&
        // A group-locked free joint is already anchored to its group — the rail rides
        // the group instead of being world-fixed, so it must not be auto-grounded.
        !this.groupOfJoint(id) &&
        !this.constraints.some((c) => c.kind === "ground" && c.joint === id)
      ) {
        this.addGround(id, this.jointWorld(j));
      }
    }
    const c: SliderConstraint = { kind: "slider", id: this.id(), railA, railB, riders: [], locked: [] };
    this.constraints.push(c);
    return c;
  }

  /**
   * Attach a joint as a rider of an existing rail (confined to its segment). `locked`
   * makes it a slider (prismatic carriage): the rider's body keeps its orientation
   * relative to the rail instead of rotating freely (see `SliderConstraint.locked`).
   */
  attachSliderRider(sliderId: number, jointId: number, locked = false): void {
    const c = this.constraints.find((x) => x.id === sliderId && x.kind === "slider") as
      | SliderConstraint
      | undefined;
    if (!c) return;
    if (!c.riders.includes(jointId)) c.riders.push(jointId);
    if (locked && !c.locked.includes(jointId)) c.locked.push(jointId);
  }

  /** The rail a joint rides (as a rider — not as a rail-defining joint), if any. */
  sliderOfRider(jointId: number): SliderConstraint | undefined {
    return this.constraints.find(
      (c): c is SliderConstraint => c.kind === "slider" && c.riders.includes(jointId)
    );
  }

  /**
   * Lock / unlock a rider's orientation to its rail (prismatic slider vs pin-in-slot).
   * A no-op when the joint isn't a rider of that rail.
   */
  setSliderRiderLocked(sliderId: number, jointId: number, locked: boolean): void {
    const c = this.constraints.find((x) => x.id === sliderId && x.kind === "slider") as
      | SliderConstraint
      | undefined;
    if (!c || !c.riders.includes(jointId)) return;
    if (locked && !c.locked.includes(jointId)) c.locked.push(jointId);
    else if (!locked) c.locked = c.locked.filter((r) => r !== jointId);
  }

  /**
   * Create a linear actuator on `sliderId`: places a new free joint on the rail (at the
   * point on the rail nearest `worldPos`, or the rail midpoint if `worldPos` is omitted),
   * attaches it as a rider of that slider, and creates the actuator constraint that drives
   * the rider during animation. Returns the new constraint, or null if the slider is
   * missing / degenerate. The rider stays at its placed position until animation runs.
   */
  addLinearActuator(sliderId: number, worldPos?: Vec2): LinearActuatorConstraint | null {
    const slider = this.constraints.find(
      (c) => c.id === sliderId && c.kind === "slider"
    ) as SliderConstraint | undefined;
    if (!slider) return null;
    const ja = this.getJoint(slider.railA);
    const jb = this.getJoint(slider.railB);
    if (!ja || !jb) return null;
    const a = this.jointWorld(ja);
    const b = this.jointWorld(jb);
    const d = sub(b, a);
    const dl = Math.hypot(d.x, d.y);
    if (dl < 1e-9) return null;
    const dir = { x: d.x / dl, y: d.y / dl };
    // Place the actuator's rider at the closest point on the rail segment to worldPos
    // (midpoint when worldPos isn't given), so a single click anywhere on the rail lands
    // the actuator under the cursor.
    const t = worldPos
      ? Math.max(0, Math.min(dl, dir.x * (worldPos.x - a.x) + dir.y * (worldPos.y - a.y)))
      : dl / 2;
    const place = vec(a.x + dir.x * t, a.y + dir.y * t);
    const rider = this.addFreeJoint(place);
    this.attachSliderRider(sliderId, rider.id);
    const c: LinearActuatorConstraint = {
      kind: "linearActuator",
      id: this.id(),
      sliderId,
      riderId: rider.id,
      speed: DEFAULT_LINEAR_ACTUATOR_SPEED,
      profile: "triangle",
    };
    this.constraints.push(c);
    return c;
  }

  /**
   * Create a motor on `bodyId` using `pivotJointId` as the rotation centre and `crankJointId`
   * as the orbiting crank pin. Both joints must already belong to that body; returns null on
   * a mismatch or missing element. Off-animation the body behaves normally; while animation is
   * running the motor pins the pivot in place and spins the crank around it at `speed` revs/s.
   */
  addMotor(bodyId: number, pivotJointId: number, crankJointId: number): MotorConstraint | null {
    if (pivotJointId === crankJointId) return null;
    if (!this.getBody(bodyId)) return null;
    const jp = this.getJoint(pivotJointId);
    const jc = this.getJoint(crankJointId);
    if (!jp || !jc) return null;
    if (jp.bodyId !== bodyId || jc.bodyId !== bodyId) return null;
    const c: MotorConstraint = {
      kind: "motor",
      id: this.id(),
      bodyId,
      pivotJointId,
      crankJointId,
      speed: DEFAULT_MOTOR_SPEED,
    };
    this.constraints.push(c);
    return c;
  }

  // --- measurements -------------------------------------------------------

  /**
   * Create a measurement between two references, with its value displayed at `labelPos`.
   * For a point–point pair the label placement picks the axis (see
   * `measureAxisForPlacement`); other pairs always measure "direct". Returns null if a
   * reference doesn't resolve.
   */
  addMeasurement(
    mode: MeasureMode,
    refA: MeasureRef,
    refB: MeasureRef,
    labelPos: Vec2
  ): Measurement | null {
    const a = this.resolveMeasureRef(refA);
    const b = this.resolveMeasureRef(refB);
    if (!a || !b) return null;
    const anchor = scale(add(refCenter(a), refCenter(b)), 0.5);
    const m: Measurement = {
      id: this.id(),
      mode,
      refA: cloneMeasureRef(refA),
      refB: cloneMeasureRef(refB),
      labelOffset: sub(labelPos, anchor),
      axis: this.measureAxisFor(refA, refB, a, b, labelPos),
    };
    this.measurements.push(m);
    return m;
  }

  /**
   * The axis a new dimension between two refs gets: the same disk vertex twice → a
   * diameter dimension; a point pair → picked from the label placement; else direct.
   */
  private measureAxisFor(
    refA: MeasureRef,
    refB: MeasureRef,
    a: ResolvedMeasureRef,
    b: ResolvedMeasureRef,
    labelPos: Vec2
  ): MeasureAxis {
    if (sameMeasureRef(refA, refB) && this.diskOfRef(refA)) return "diameter";
    return a.kind === "point" && b.kind === "point" ? measureAxisForPlacement(a.p, b.p, labelPos) : "direct";
  }

  getMeasurement(id: number): Measurement | undefined {
    return this.measurements.find((m) => m.id === id);
  }

  removeMeasurement(id: number): void {
    this.measurements = this.measurements.filter((m) => m.id !== id);
  }

  /** Resolve a reference to its current world geometry, or null if its element is gone. */
  resolveMeasureRef(ref: MeasureRef): ResolvedMeasureRef | null {
    switch (ref.kind) {
      case "joint": {
        const j = this.getJoint(ref.jointId);
        return j ? { kind: "point", p: this.jointWorld(j) } : null;
      }
      case "vertex": {
        const b = this.getBody(ref.bodyId);
        const ctrl = b ? this.controlListOf(b, ref.hole ?? null) : null;
        if (!b || !ctrl || ref.index < 0 || ref.index >= ctrl.length) return null;
        return { kind: "point", p: add(b.pos, rotate(ctrl[ref.index], b.angle)) };
      }
      case "bodyPoint": {
        const b = this.getBody(ref.bodyId);
        return b ? { kind: "point", p: add(b.pos, rotate(ref.local, b.angle)) } : null;
      }
      case "rail": {
        const c = this.constraints.find(
          (x) => x.id === ref.sliderId && x.kind === "slider"
        ) as SliderConstraint | undefined;
        if (!c) return null;
        const ja = this.getJoint(c.railA);
        const jb = this.getJoint(c.railB);
        if (!ja || !jb) return null;
        return { kind: "line", a: this.jointWorld(ja), b: this.jointWorld(jb) };
      }
      case "edge": {
        const b = this.getBody(ref.bodyId);
        const ctrl = b ? this.controlListOf(b, ref.hole ?? null) : null;
        // A 1-point outline (a disk hole) has no edges to reference.
        if (!b || !ctrl || ctrl.length < 2 || ref.index < 0 || ref.index >= ctrl.length) return null;
        const w = (i: number) => add(b.pos, rotate(ctrl[i], b.angle));
        return { kind: "line", a: w(ref.index), b: w((ref.index + 1) % ctrl.length) };
      }
      case "patternAxis": {
        // The dotted axis line: from the seed's anchor to the last instance along it.
        const info = this.patternInfo(ref.patternId);
        const ax = info?.axes[ref.axis];
        return info && ax ? { kind: "line", a: clone(info.anchor), b: clone(ax.end) } : null;
      }
      case "guidePoint": {
        const g = this.getGuide(ref.guideId);
        return g ? { kind: "point", p: clone(g[ref.which]) } : null;
      }
      case "guideLine": {
        // Resolved as the defining segment; consumers that need the infinite line
        // (point+line measurements, the renderer) already extend line refs themselves.
        const g = this.getGuide(ref.guideId);
        return g ? { kind: "line", a: clone(g.a), b: clone(g.b), infinite: true } : null;
      }
    }
  }

  /** Current world position of a measurement's value label, or null if a ref is gone. */
  measurementLabelPos(m: Measurement): Vec2 | null {
    const a = this.resolveMeasureRef(m.refA);
    const b = this.resolveMeasureRef(m.refB);
    if (!a || !b) return null;
    return add(scale(add(refCenter(a), refCenter(b)), 0.5), m.labelOffset);
  }

  /**
   * Move a measurement's label to a new world position. For a point–point measurement
   * the new placement also re-derives the axis (h / v / direct), like at creation.
   */
  setMeasurementLabel(id: number, labelPos: Vec2): void {
    const m = this.getMeasurement(id);
    if (!m) return;
    const a = this.resolveMeasureRef(m.refA);
    const b = this.resolveMeasureRef(m.refB);
    if (!a || !b) return;
    const anchor = scale(add(refCenter(a), refCenter(b)), 0.5);
    m.labelOffset = sub(labelPos, anchor);
    // A diameter dimension keeps its axis wherever the label goes (the label's direction
    // only picks where the diameter line is drawn through the disk).
    if (m.axis !== "diameter" && a.kind === "point" && b.kind === "point") {
      const before = m.axis;
      m.axis = measureAxisForPlacement(a.p, b.p, labelPos);
      // A driving dimension that changed axis measures a different quantity — its held
      // side re-captures from the current geometry (h/v gain one, direct drops it).
      if (m.driving && m.axis !== before) this.captureMeasurementSide(m);
    }
  }

  /**
   * Compute a measurement's current value + drawing geometry. Returns null when a
   * reference is gone or degenerate (e.g. a zero-length rail) — the measurement is
   * simply not displayed that frame.
   */
  measureInfo(m: Measurement): MeasureInfo | null {
    const a = this.resolveMeasureRef(m.refA);
    const b = this.resolveMeasureRef(m.refB);
    if (!a || !b) return null;
    const labelPos = add(scale(add(refCenter(a), refCenter(b)), 0.5), m.labelOffset);
    let info: MeasureInfo | null;
    if (m.axis === "diameter") {
      // The disk may have stopped being one (a node added to the hole): then no display.
      const disk = this.diskOfRef(m.refA);
      info = disk ? diameterInfo(m.id, disk.c, disk.r, labelPos) : null;
    } else if (a.kind === "point" && b.kind === "point") {
      info = pointPointInfo(m.id, a.p, b.p, m.axis, labelPos);
    } else if (a.kind === "line" && b.kind === "line") {
      info = lineLineInfo(m.id, a, b, labelPos);
    } else {
      const p = a.kind === "point" ? a.p : (b as { kind: "point"; p: Vec2 }).p;
      const line = a.kind === "line" ? a : (b as { kind: "line"; a: Vec2; b: Vec2 });
      info = pointLineInfo(m.id, p, line, labelPos);
    }
    if (info && m.driving) {
      info.driving = true;
      if (m.mode === "draw" && m.target !== undefined && info.kind === "distance") {
        const off = Math.abs(info.value - m.target) > DIM_VIOLATION_TOL;
        // A pose at the right absolute distance but on the flipped side of the held
        // direction is violated too (the geometry crossed through — e.g. a def edit).
        const cur = m.side !== undefined ? this.measurementSide(m) : null;
        if (off || (m.side !== undefined && cur !== null && cur !== m.side)) {
          info.violated = true;
        }
      }
    }
    return info;
  }

  /** Like `measureInfo`, but for a not-yet-created measurement (live placement preview). */
  measurePreview(refA: MeasureRef, refB: MeasureRef, labelPos: Vec2): MeasureInfo | null {
    const a = this.resolveMeasureRef(refA);
    const b = this.resolveMeasureRef(refB);
    if (!a || !b) return null;
    const anchor = scale(add(refCenter(a), refCenter(b)), 0.5);
    return this.measureInfo({
      id: -1,
      mode: "draw",
      refA,
      refB,
      labelOffset: sub(labelPos, anchor),
      axis: this.measureAxisFor(refA, refB, a, b, labelPos),
    });
  }

  /** Drop measurements whose references no longer resolve (their element was removed). */
  private pruneMeasurements(): void {
    this.measurements = this.measurements.filter(
      (m) => this.resolveMeasureRef(m.refA) && this.resolveMeasureRef(m.refB)
    );
  }

  /**
   * Keep vertex/edge measurement refs pointing at the same geometry across a control
   * vertex insert (`delta = 1` at `at`) or removal (`delta = -1`): later indices shift;
   * a ref *on* a removed vertex/edge loses its subject, so its measurement is dropped.
   * `hole` scopes the shift to one outline (a hole's, or the outer when null).
   */
  private shiftMeasureIndices(bodyId: number, at: number, delta: 1 | -1, hole: number | null = null): void {
    const affected = (ref: MeasureRef | null): ref is MeasureRef & { index: number } =>
      !!ref &&
      (ref.kind === "vertex" || ref.kind === "edge") &&
      ref.bodyId === bodyId &&
      (ref.hole ?? null) === hole;
    const gone = new Set<number>();
    for (const m of this.measurements) {
      for (const ref of [m.refA, m.refB]) {
        if (!affected(ref)) continue;
        if (delta === -1) {
          if (ref.index === at) gone.add(m.id);
          else if (ref.index > at) ref.index--;
        } else if (ref.index >= at) {
          ref.index++;
        }
      }
    }
    if (gone.size) this.measurements = this.measurements.filter((m) => !gone.has(m.id));
    const cGone = new Set<number>();
    for (const c of this.sketch) {
      for (const ref of [c.refA, c.refB]) {
        if (!affected(ref)) continue;
        if (delta === -1) {
          if (ref.index === at) cGone.add(c.id);
          else if (ref.index > at) ref.index--;
        } else if (ref.index >= at) {
          ref.index++;
        }
      }
    }
    if (cGone.size) this.sketch = this.sketch.filter((c) => !cGone.has(c.id));
  }

  // --- sketch constraints ---------------------------------------------------

  /**
   * Create a sketch constraint. Reference kinds are validated per constraint kind:
   * `coincident` takes two point refs, or a point ref + a line ref (the point is held
   * on the **infinite** line; normalized so the point is stored as `refA`);
   * `horizontal`/`vertical` take one line ref (refB
   * omitted) or two point refs; `parallel`/`perpendicular`/`equal` take two line refs.
   * Point refs are joints, body control vertices, or guideline defining points
   * (`bodyPoint` refs are measurement-only — the sketch solver can't move them
   * independently); line refs are slider rails, body control edges, or guidelines
   * (except `equal`, which rejects guidelines — an infinite line has no length).
   * Returns null on a kind mismatch, an unresolvable ref, or two refs naming the
   * same element (for point-on-line: a point that *is* an endpoint of the line, which
   * would be trivially satisfied forever).
   */
  addSketchConstraint(
    kind: SketchConstraintKind,
    refA: MeasureRef,
    refB?: MeasureRef
  ): SketchConstraint | null {
    const isPoint = (r: MeasureRef) => r.kind === "joint" || r.kind === "vertex" || r.kind === "guidePoint";
    const isLine = (r: MeasureRef) => r.kind === "rail" || r.kind === "edge" || r.kind === "guideLine" || r.kind === "patternAxis";
    let b = refB ?? null;
    if (kind === "coincident") {
      if (!b) return null;
      // Normalize point-on-line order: the point is stored as refA (the badge anchors there).
      if (isLine(refA) && isPoint(b)) [refA, b] = [b, refA];
      if (!isPoint(refA) || !(isPoint(b) || isLine(b))) return null;
      if (isLine(b) && this.pointIsLineEndpoint(refA, b)) return null;
    } else if (kind === "horizontal" || kind === "vertical") {
      if (b ? !(isPoint(refA) && isPoint(b)) : !isLine(refA)) return null;
    } else {
      if (!b || !isLine(refA) || !isLine(b)) return null;
      // Equal length is meaningless on an infinite guideline (its defining segment's
      // length is arbitrary construction, not geometry) — reject it.
      if (kind === "equal" && (refA.kind === "guideLine" || b.kind === "guideLine")) return null;
    }
    if (!this.resolveMeasureRef(refA) || (b && !this.resolveMeasureRef(b))) return null;
    if (b && sameMeasureRef(refA, b)) return null;
    // Instance geometry is design-locked: its shape belongs to the component definition.
    // A constraint with ONE end free is shape material for the free side (instance
    // variables are immovable in the sketch solver — see varRank in sketch.ts). One
    // whose every end is instance-owned is a *pose constraint* (pose.ts): it moves
    // rigid parts, so it's rejected only when no pose can satisfy it — two ends rigid
    // to one another (same body / chassis group), or "equal" (both lengths locked).
    if (this.refInstanceOwned(refA) && (!b || this.refInstanceOwned(b))) {
      if (kind === "equal") return null;
      if (b) {
        const ka = this.refRigidUnitKey(refA);
        const kb = this.refRigidUnitKey(b);
        if (ka === null || kb === null || ka === kb) return null;
      }
    }
    const c: SketchConstraint = {
      kind,
      id: this.id(),
      refA: cloneMeasureRef(refA),
      refB: b ? cloneMeasureRef(b) : null,
    };
    this.sketch.push(c);
    return c;
  }

  /**
   * Whether a point ref structurally names one of a line ref's two defining points
   * (a vertex that ends the edge, a rail's own joint, a guideline's defining point) —
   * such a point lies on the line by construction, so a coincident between them is a
   * permanent no-op and gets rejected.
   */
  private pointIsLineEndpoint(pt: MeasureRef, ln: MeasureRef): boolean {
    if (ln.kind === "edge" && pt.kind === "vertex") {
      if (pt.bodyId !== ln.bodyId || (pt.hole ?? null) !== (ln.hole ?? null)) return false;
      const body = this.getBody(ln.bodyId);
      const ctrl =
        body && (ln.hole ?? null) === null
          ? body.controlLocal
          : body?.holes?.[ln.hole!]?.controlLocal ?? null;
      if (!ctrl) return false;
      return pt.index === ln.index || pt.index === (ln.index + 1) % ctrl.length;
    }
    if (ln.kind === "rail" && pt.kind === "joint") {
      const s = this.constraints.find((c) => c.kind === "slider" && c.id === ln.sliderId);
      return !!s && s.kind === "slider" && (s.railA === pt.jointId || s.railB === pt.jointId);
    }
    if (ln.kind === "guideLine" && pt.kind === "guidePoint") return ln.guideId === pt.guideId;
    return false;
  }

  getSketchConstraint(id: number): SketchConstraint | undefined {
    return this.sketch.find((c) => c.id === id);
  }

  removeSketchConstraint(id: number): void {
    this.sketch = this.sketch.filter((c) => c.id !== id);
  }

  /** Drop sketch constraints whose references no longer resolve (their element was removed). */
  private pruneSketch(): void {
    this.sketch = this.sketch.filter(
      (c) => this.resolveMeasureRef(c.refA) && (!c.refB || this.resolveMeasureRef(c.refB))
    );
  }

  /**
   * Make a draw-mode distance dimension driving at `target` (world units, > 0). The
   * caller is expected to have run the sketch solve first (see `applyDrivingDimension`
   * in sketch.ts, which validates + solves + commits via this). Returns false for a
   * missing / sim-mode measurement or a non-positive target.
   */
  /**
   * The relative direction a dimension currently measures (see `Measurement.side`):
   * the sign of (B − A) along an h/v axis, or of the signed perpendicular distance for
   * point+line / line+line. Null when the dimension has no side (a direct point-point
   * distance is free to rotate) or its refs don't resolve. The sign conventions match
   * the solvers' correction math exactly (sketch.ts `buildDimensionItem`, pose.ts
   * `poseCorrection`).
   */
  measurementSide(m: Measurement): 1 | -1 | null {
    const a = this.resolveMeasureRef(m.refA);
    const b = this.resolveMeasureRef(m.refB);
    if (!a || !b) return null;
    const sgn = (v: number): 1 | -1 => (v < 0 ? -1 : 1);
    if (a.kind === "point" && b.kind === "point") {
      if (m.axis === "h") return sgn(b.p.x - a.p.x);
      if (m.axis === "v") return sgn(b.p.y - a.p.y);
      return null; // direct distance: no side
    }
    if (a.kind === "line" && b.kind === "line") {
      const d = sub(a.b, a.a);
      const l = len(d);
      if (l < 1e-9) return null;
      const n = perp(scale(d, 1 / l));
      const midA = scale(add(a.a, a.b), 0.5);
      const midB = scale(add(b.a, b.b), 0.5);
      return sgn(dot(sub(midB, midA), n));
    }
    const pt = a.kind === "point" ? a : (b as { kind: "point"; p: Vec2 });
    const ln = a.kind === "line" ? a : (b as { kind: "line"; a: Vec2; b: Vec2 });
    const d = sub(ln.b, ln.a);
    const l = len(d);
    if (l < 1e-9) return null;
    const n = perp(scale(d, 1 / l));
    return sgn(dot(sub(pt.p, ln.a), n));
  }

  /** Capture (or clear) a driving dimension's held side from the current geometry. */
  private captureMeasurementSide(m: Measurement): void {
    const side = this.measurementSide(m);
    if (side !== null) m.side = side;
    else delete m.side;
  }

  setMeasurementDriving(id: number, target: number): boolean {
    const m = this.getMeasurement(id);
    if (!m || m.mode !== "draw" || !(target > 0)) return false;
    // Instance shape belongs to the definition, so a dimension between two ends that
    // are rigid to one another inside instances (same body / same chassis group) can
    // never drive. Any other pairing may: with both ends instance-owned the pose
    // machinery moves rigid parts (pose.ts); with one end free the sketch moves the
    // free side (instance variables are immovable there — see varRank in sketch.ts).
    if (this.refInstanceOwned(m.refA) && this.refInstanceOwned(m.refB)) {
      const ka = this.refRigidUnitKey(m.refA);
      const kb = this.refRigidUnitKey(m.refB);
      if (ka === null || kb === null || ka === kb) return false;
    }
    m.driving = true;
    m.target = target;
    // The side is (re-)captured from the current geometry: the drawn relative
    // direction is what the dimension holds from here on (same philosophy as
    // slider-lock / weld baselines — what you draw is what gets locked).
    this.captureMeasurementSide(m);
    return true;
  }

  /** Turn a driving dimension back into a driven (read-only) one. */
  clearMeasurementDriving(id: number): void {
    const m = this.getMeasurement(id);
    if (!m) return;
    delete m.driving;
    delete m.target;
    delete m.side;
  }

  /**
   * Uniformly scale a body by `factor` about its centroid — control polygon, corner
   * radius, attached joints, their ground anchors, and `bodyPoint` measurement refs all
   * scale together, so the body keeps its form factor (used when the first driving
   * dimension on an otherwise unconstrained body is set). The centroid stays put.
   */
  scaleBody(bodyId: number, factor: number): void {
    const body = this.getBody(bodyId);
    if (!body || !(factor > 0) || factor === 1) return;
    body.controlLocal = body.controlLocal.map((p) => scale(p, factor));
    body.radius *= factor;
    if (body.radii) body.radii = body.radii.map((r) => (typeof r === "number" ? r * factor : r));
    body.holes?.forEach((h) => {
      h.controlLocal = h.controlLocal.map((p) => scale(p, factor));
      h.radius *= factor;
      if (h.radii) h.radii = h.radii.map((r) => (typeof r === "number" ? r * factor : r));
    });
    const attached = this.joints.filter((j) => j.bodyId === bodyId);
    for (const j of attached) j.local = scale(j.local, factor);
    for (const p of this.patterns) {
      if (p.bodyId !== bodyId) continue;
      if (p.layout.kind === "linear") for (const a of p.layout.axes) a.step = scale(a.step, factor);
      else p.layout.centre = scale(p.layout.centre, factor);
    }
    this.rebuildBody(body); // re-anchors joints at their (already scaled) world positions
    const owned = new Set(attached.map((j) => j.id));
    for (const c of this.constraints) {
      if (c.kind === "ground" && owned.has(c.joint)) {
        const j = this.getJoint(c.joint);
        if (j) c.anchor = this.jointWorld(j);
      }
    }
    for (const m of this.measurements) {
      for (const ref of [m.refA, m.refB]) {
        if (ref.kind === "bodyPoint" && ref.bodyId === bodyId) ref.local = scale(ref.local, factor);
      }
    }
  }

  getBody(id: number): Body | undefined {
    return this.bodies.find((b) => b.id === id);
  }

  getJoint(id: number): Joint | undefined {
    return this.joints.find((j) => j.id === id);
  }

  /** World position of a joint: its body's pose + local offset, or its own point if free. */
  jointWorld(joint: Joint): Vec2 {
    if (joint.bodyId === null) return vec(joint.local.x, joint.local.y);
    const body = this.getBody(joint.bodyId)!;
    return add(body.pos, rotate(joint.local, body.angle));
  }

  /** World-space polygon of a body. */
  bodyWorldVerts(body: Body): Vec2[] {
    return body.local.map((p) => add(body.pos, rotate(p, body.angle)));
  }

  /** World-space control-polygon vertices (the editable corners) of a body. */
  bodyControlWorld(body: Body): Vec2[] {
    return body.controlLocal.map((p) => add(body.pos, rotate(p, body.angle)));
  }

  /** World-space hole loops of a body (empty when it has none). */
  bodyHolesWorld(body: Body): Vec2[][] {
    return (body.holesLocal ?? []).map((loop) =>
      loop.map((p) => add(body.pos, rotate(p, body.angle)))
    );
  }

  /** Whether a world point lies inside a body's (rounded) polygon. */
  pointInBody(body: Body, p: Vec2): boolean {
    return pointInPolygon(p, this.bodyWorldVerts(body));
  }

  /** `p` if it lies inside the body; otherwise the nearest point on the body's outline. */
  clampIntoBody(body: Body, p: Vec2): Vec2 {
    return this.pointInBody(body, p) ? p : closestPointOnPolygon(p, this.bodyWorldVerts(body));
  }

  /**
   * Attached joints stranded outside their body's rounded outline (beyond
   * `CONTAINMENT_WARN_EPS` — a joint clamped exactly onto the edge is fine).
   * Direct edits can't violate containment (placement and drags clamp), but shape
   * changes keep joints at their world positions — a component-definition edit
   * cascading into instances, or a removed control vertex, can leave a joint outside
   * the new outline. Nothing is moved: the UI flags these for the user to resolve.
   */
  jointsOutsideBody(): number[] {
    const out: number[] = [];
    const verts = new Map<number, Vec2[]>(); // body id → outline, derived once per body
    for (const j of this.joints) {
      if (j.bodyId === null) continue;
      const body = this.getBody(j.bodyId);
      if (!body) continue;
      let poly = verts.get(body.id);
      if (!poly) verts.set(body.id, (poly = this.bodyWorldVerts(body)));
      const w = this.jointWorld(j);
      if (pointInPolygon(w, poly)) continue;
      if (dist(w, closestPointOnPolygon(w, poly)) > CONTAINMENT_WARN_EPS) out.push(j.id);
    }
    return out;
  }

  /** Topmost body whose polygon contains the point, or undefined. */
  bodyAt(p: Vec2): Body | undefined {
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      if (pointInPolygon(p, this.bodyWorldVerts(this.bodies[i]))) return this.bodies[i];
    }
    return undefined;
  }

  /**
   * The hole under a world point: the topmost body one of whose cut-outs contains it,
   * with that hole's index. (Picking elsewhere treats cut-outs as body material — this
   * is for tools that address a hole itself, like Pattern.)
   */
  holeAt(p: Vec2): { body: Body; hole: number } | undefined {
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      const body = this.bodies[i];
      if (!body.holesLocal || !pointInPolygon(p, this.bodyWorldVerts(body))) continue;
      const hole = this.bodyHolesWorld(body).findIndex((loop) => pointInPolygon(p, loop));
      if (hole >= 0) return { body, hole };
    }
    return undefined;
  }

  /** Every body whose polygon contains the point (topmost first). */
  bodiesAt(p: Vec2): Body[] {
    const hits: Body[] = [];
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      if (pointInPolygon(p, this.bodyWorldVerts(this.bodies[i]))) hits.push(this.bodies[i]);
    }
    return hits;
  }

  /** Nearest joint within `radius` world units of the point, or undefined. */
  jointAt(p: Vec2, radius: number): Joint | undefined {
    let best: Joint | undefined;
    let bestD = radius;
    for (const j of this.joints) {
      const d = dist(this.jointWorld(j), p);
      if (d <= bestD) {
        bestD = d;
        best = j;
      }
    }
    return best;
  }

  /** True if a joint participates in any grounding or slider constraint. */
  isAnchored(jointId: number): boolean {
    return this.constraints.some(
      (c) =>
        (c.kind === "ground" && c.joint === jointId) ||
        (c.kind === "slider" &&
          (c.railA === jointId || c.railB === jointId || c.riders.includes(jointId)))
    );
  }

  /** Slider whose rail *segment* passes within `radius` (world units) of the point. */
  sliderAt(p: Vec2, radius: number): SliderConstraint | undefined {
    for (let i = this.constraints.length - 1; i >= 0; i--) {
      const c = this.constraints[i];
      if (c.kind !== "slider") continue;
      const ja = this.getJoint(c.railA);
      const jb = this.getJoint(c.railB);
      if (!ja || !jb) continue;
      const a = this.jointWorld(ja);
      const b = this.jointWorld(jb);
      if (distToSegment(p, a, b) <= radius) return c;
    }
    return undefined;
  }

  /**
   * Translate the body's pose by `delta`, carrying along the world-space ground
   * anchors of any grounded joints on it (so the ground moves with the part).
   */
  moveBody(bodyId: number, delta: Vec2): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    body.pos = add(body.pos, delta);
    const owned = new Set(this.joints.filter((j) => j.bodyId === bodyId).map((j) => j.id));
    for (const c of this.constraints) {
      if (c.kind === "ground" && owned.has(c.joint)) c.anchor = add(c.anchor, delta);
    }
  }

  /**
   * Reposition a joint by a world-space `delta`. A free joint's world point moves; a
   * body joint's local offset shifts (the body stays put), clamped so the joint can
   * never leave its body's outline. A body joint sitting exactly on one of its body's
   * control vertices is *stuck* to it: the move is delegated to `moveBodyVertex`, which
   * reshapes the body and carries the joint along. Any ground anchor on the joint
   * follows, so it stays grounded where it now sits.
   */
  moveJoint(id: number, delta: Vec2): void {
    const j = this.getJoint(id);
    if (!j) return;
    if (j.bodyId !== null) {
      const body = this.getBody(j.bodyId)!;
      const v = this.coincidentVertexIndex(body, this.jointWorld(j));
      if (v) {
        this.moveBodyVertex(body.id, v.index, delta, v.hole);
        return;
      }
    }
    this.shiftJoint(j, delta);
  }

  /** The control vertex (outer, or of any hole) exactly coincident with `p`, or null. */
  private coincidentVertexIndex(body: Body, p: Vec2): { index: number; hole: number | null } | null {
    const ctrl = this.bodyControlWorld(body);
    for (let i = 0; i < ctrl.length; i++) {
      if (dist(ctrl[i], p) < VERTEX_LINK_EPS) return { index: i, hole: null };
    }
    for (let hi = 0; hi < (body.holes?.length ?? 0); hi++) {
      const hc = this.bodyHoleControlWorld(body, hi);
      for (let i = 0; i < hc.length; i++) {
        if (dist(hc[i], p) < VERTEX_LINK_EPS) return { index: i, hole: hi };
      }
    }
    return null;
  }

  /**
   * Raw joint shift by a world delta (no vertex linking): body joints are clamped
   * inside their body's outline; ground anchors follow the joint.
   */
  private shiftJoint(j: Joint, delta: Vec2): void {
    if (j.bodyId === null) {
      j.local = add(j.local, delta);
    } else {
      const body = this.getBody(j.bodyId)!;
      const target = this.clampIntoBody(body, add(this.jointWorld(j), delta));
      j.local = rotate(sub(target, body.pos), -body.angle);
    }
    const w = this.jointWorld(j);
    for (const c of this.constraints) {
      if (c.kind === "ground" && c.joint === j.id) c.anchor = vec(w.x, w.y);
    }
    const pj = this.patternOfJoint(j.id);
    if (pj?.role === "seed") this.syncPattern(pj.pattern); // the array follows its seed
  }

  /**
   * Rigidly rotate a body by `delta` radians about a fixed world `pivot`: its pose
   * (centroid + angle) turns about the pivot and attached joints follow automatically
   * (they live in the local frame). Ground anchors on the body's joints rotate too, so
   * a grounded part stays grounded where it now sits. No shape rebuild is needed.
   */
  rotateBody(bodyId: number, pivot: Vec2, delta: number): void {
    const body = this.getBody(bodyId);
    if (!body || delta === 0) return;
    body.pos = add(pivot, rotate(sub(body.pos, pivot), delta));
    body.angle += delta;
    const owned = new Set(this.joints.filter((j) => j.bodyId === bodyId).map((j) => j.id));
    for (const c of this.constraints) {
      if (c.kind === "ground" && owned.has(c.joint)) {
        c.anchor = add(pivot, rotate(sub(c.anchor, pivot), delta));
      }
    }
  }

  /**
   * Mirror a body across a line through its centroid: `"h"` flips it left↔right (reflect
   * x), `"v"` flips it top↔bottom (reflect y). The control polygon and every attached
   * joint (and its ground anchor) are reflected; the polygon winding is reversed so the
   * fillet/offset rounding stays correct. The centroid is fixed by the reflection, so
   * the body doesn't move — it just turns into its mirror image in place.
   */
  mirrorBody(bodyId: number, axis: "h" | "v"): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    const c = body.pos;
    const reflect = (w: Vec2): Vec2 =>
      axis === "h" ? vec(2 * c.x - w.x, w.y) : vec(w.x, 2 * c.y - w.y);
    // bodyPoint refs name a spot fixed in the body's frame — capture their current world
    // positions now, so they can be re-baked onto the reflected material afterwards.
    const bodyPoints: { ref: { local: Vec2 }; world: Vec2 }[] = [];
    for (const m of this.measurements) {
      for (const ref of [m.refA, m.refB]) {
        if (ref.kind === "bodyPoint" && ref.bodyId === bodyId) {
          bodyPoints.push({ ref, world: add(body.pos, rotate(ref.local, body.angle)) });
        }
      }
    }
    // Reflect the control polygon in world space; reverse it to preserve the winding.
    const ctrlWorld = this.bodyControlWorld(body).map(reflect).reverse();
    // Hole control polygons reflect + reverse the same way (their radii reverse below).
    const holeCtrlWorld = (body.holes ?? []).map((_, hi) =>
      this.bodyHoleControlWorld(body, hi).map(reflect).reverse()
    );
    const attached = this.joints.filter((j) => j.bodyId === bodyId);
    const jointWorlds = new Map(attached.map((j) => [j.id, reflect(this.jointWorld(j))]));
    const owned = new Set(attached.map((j) => j.id));
    for (const con of this.constraints) {
      if (con.kind === "ground" && owned.has(con.joint)) con.anchor = reflect(con.anchor);
    }
    // Bake the reflected world geometry back in at angle 0 (a reflection isn't a rotation,
    // so the prior angle no longer applies), then let rebuildBody re-derive shape/mass and
    // re-anchor joints to their now-reflected world positions.
    // Pattern layouts are body-local vectors: take them through the same reflection
    // (world → reflect → re-baked at angle 0); a fixed circular step flips its sense.
    const reflectVec = (v: Vec2): Vec2 => {
      const w = rotate(v, body.angle);
      return axis === "h" ? vec(-w.x, w.y) : vec(w.x, -w.y);
    };
    for (const p of this.patterns) {
      if (p.bodyId !== bodyId) continue;
      if (p.layout.kind === "linear") for (const a of p.layout.axes) a.step = reflectVec(a.step);
      else {
        p.layout.centre = reflectVec(p.layout.centre);
        if (p.layout.angleStep !== undefined) p.layout.angleStep = -p.layout.angleStep;
      }
    }
    body.angle = 0;
    body.controlLocal = ctrlWorld.map((p) => sub(p, c));
    // The reversal renumbers the corners (vertex i → n−1−i), so per-corner radius
    // overrides reverse with them to stay on the same physical corner.
    if (body.radii) body.radii = [...body.radii].reverse();
    body.holes?.forEach((h, hi) => {
      h.controlLocal = holeCtrlWorld[hi].map((p) => sub(p, c));
      if (h.radii) h.radii = [...h.radii].reverse();
    });
    for (const j of attached) j.local = sub(jointWorlds.get(j.id)!, c);
    this.rebuildBody(body);
    // The reversal renumbered the control polygon, so vertex/edge refs (sketch constraints
    // and measurements) must be remapped to keep naming the same — now reflected — corner
    // or edge: vertex i → n−1−i; edge i (vᵢ→vᵢ₊₁) → n−2−i, wrapping for i = n−1. Without
    // this, an H/V constraint jumps to a different edge and the next sketch solve drags
    // the geometry to satisfy the wrong element. (Reflection itself preserves every
    // constraint kind: H stays H, V stays V, parallel/perpendicular/equal/coincident and
    // distances are all reflection-invariant.)
    const remap = (ref: MeasureRef | null): void => {
      if (!ref || (ref.kind !== "vertex" && ref.kind !== "edge") || ref.bodyId !== bodyId) return;
      // Each outline reversed independently, so a ref remaps within its own outline.
      const ctrl = this.controlListOf(body, ref.hole ?? null);
      if (!ctrl) return;
      const n = ctrl.length;
      ref.index = ref.kind === "vertex" ? n - 1 - ref.index : (2 * n - 2 - ref.index) % n;
    };
    for (const m of this.measurements) {
      remap(m.refA);
      remap(m.refB);
    }
    for (const sc of this.sketch) {
      remap(sc.refA);
      remap(sc.refB);
    }
    // bodyPoint refs land on the reflection of the spot they marked (angle is 0 now).
    for (const bp of bodyPoints) bp.ref.local = sub(reflect(bp.world), body.pos);
  }

  /**
   * Mirror a whole selection (bodies + free joints) across an axis through the centre of
   * its combined bounding box: every body is mirrored in place, then its centroid is
   * reflected across the shared axis (in-place reflection + centroid reflection = a true
   * reflection of the whole arrangement), and free joints reflect their world position.
   * Pins between selected bodies stay coincident, since both endpoints land on the same
   * reflected point.
   */
  mirrorBodies(bodyIds: number[], freeJointIds: number[], axis: "h" | "v"): void {
    const bodies = [...new Set(bodyIds)]
      .map((id) => this.getBody(id))
      .filter((b): b is Body => b !== undefined);
    const joints = [...new Set(freeJointIds)]
      .map((id) => this.getJoint(id))
      .filter((j): j is Joint => j !== undefined && j.bodyId === null);
    let min = Infinity;
    let max = -Infinity;
    const include = (p: Vec2): void => {
      const v = axis === "h" ? p.x : p.y;
      if (v < min) min = v;
      if (v > max) max = v;
    };
    for (const b of bodies) this.bodyWorldVerts(b).forEach(include);
    for (const j of joints) include(this.jointWorld(j));
    if (!Number.isFinite(min)) return;
    const c = (min + max) / 2;
    const reflectDelta = (p: Vec2): Vec2 =>
      axis === "h" ? vec(2 * (c - p.x), 0) : vec(0, 2 * (c - p.y));
    for (const b of bodies) {
      this.mirrorBody(b.id, axis); // mirror in place about its own centroid...
      this.moveBody(b.id, reflectDelta(b.pos)); // ...then reflect the centroid itself
    }
    for (const j of joints) {
      this.moveJoint(j.id, reflectDelta(this.jointWorld(j)));
    }
  }

  // --- permanent body groups ------------------------------------------------

  /** The group a body belongs to, or undefined (a body is in at most one group). */
  groupOf(bodyId: number): BodyGroup | undefined {
    return this.groups.find((g) => g.bodyIds.includes(bodyId));
  }

  /** The group a free joint is locked to, or undefined. */
  groupOfJoint(jointId: number): BodyGroup | undefined {
    return this.groups.find((g) => g.jointIds.includes(jointId));
  }

  /**
   * Create a permanent group over `bodyIds` plus (optionally) free joints locked to it.
   * Any existing group touching one of the members is absorbed (grouping is a union — a
   * member belongs to at most one group), so grouping a selection that includes grouped
   * elements merges everything into a single group. Returns the new group, or null when
   * fewer than 2 distinct existing members remain.
   */
  addGroup(bodyIds: number[], jointIds: number[] = []): BodyGroup | null {
    const bodies = new Set<number>();
    const joints = new Set<number>();
    const absorb = (g: BodyGroup): void => {
      g.bodyIds.forEach((b) => bodies.add(b));
      g.jointIds.forEach((j) => joints.add(j));
    };
    for (const id of bodyIds) {
      if (!this.getBody(id)) continue;
      const existing = this.groupOf(id);
      if (existing) absorb(existing);
      else bodies.add(id);
    }
    for (const id of jointIds) {
      const j = this.getJoint(id);
      if (!j || j.bodyId !== null) continue; // only free joints can be group members
      const existing = this.groupOfJoint(id);
      if (existing) absorb(existing);
      else joints.add(id);
    }
    if (bodies.size + joints.size < 2) return null;
    this.groups = this.groups.filter(
      (g) => !g.bodyIds.some((b) => bodies.has(b)) && !g.jointIds.some((j) => joints.has(j))
    );
    const group: BodyGroup = { id: this.id(), bodyIds: [...bodies], jointIds: [...joints] };
    this.groups.push(group);
    return group;
  }

  /** Dissolve every group containing any of the given members. Returns whether anything changed. */
  ungroup(bodyIds: number[], jointIds: number[] = []): boolean {
    const hitB = new Set(bodyIds);
    const hitJ = new Set(jointIds);
    const before = this.groups.length;
    this.groups = this.groups.filter(
      (g) => !g.bodyIds.some((b) => hitB.has(b)) && !g.jointIds.some((j) => hitJ.has(j))
    );
    return this.groups.length !== before;
  }

  /**
   * Toggle grounding on a body — and, when it belongs to a permanent group, on the whole
   * group (one grounded member would fix the group anyway, since it's rigid; keeping the
   * flags in step keeps the ground symbols honest). If any affected body is grounded,
   * all are ungrounded; otherwise all are grounded. Returns whether anything changed.
   */
  toggleBodyGround(bodyId: number): boolean {
    if (!this.getBody(bodyId)) return false;
    const ids = this.groupOf(bodyId)?.bodyIds ?? [bodyId];
    const bodies = ids
      .map((id) => this.getBody(id))
      .filter((b): b is Body => b !== undefined);
    const on = !bodies.some((b) => b.grounded);
    for (const b of bodies) b.grounded = on;
    return true;
  }

  /**
   * Move bodies to the back (start) or front (end) of the z-order. The `bodies` array *is*
   * the z-order: it renders first-to-last (first = bottom) and hit-tests last-to-first
   * (last = topmost wins the click), so one reorder fixes both drawing and picking. A
   * grouped body moves with its whole group; moved bodies keep their relative order.
   * Returns whether the order actually changed.
   */
  reorderBodies(bodyIds: number[], where: "back" | "front"): boolean {
    const moved = new Set<number>();
    for (const id of bodyIds) {
      if (!this.getBody(id)) continue;
      const g = this.groupOf(id);
      if (g) g.bodyIds.forEach((b) => moved.add(b));
      else moved.add(id);
    }
    if (moved.size === 0 || moved.size === this.bodies.length) return false;
    const picked = this.bodies.filter((b) => moved.has(b.id));
    const rest = this.bodies.filter((b) => !moved.has(b.id));
    const next = where === "back" ? [...picked, ...rest] : [...rest, ...picked];
    if (next.every((b, i) => b === this.bodies[i])) return false;
    this.bodies = next;
    return true;
  }

  /** Drop removed bodies / non-free joints from groups; a group left with fewer than 2
   *  members (bodies + joints combined) dissolves. */
  private pruneGroups(): void {
    for (const g of this.groups) {
      g.bodyIds = g.bodyIds.filter((b) => this.getBody(b));
      g.jointIds = g.jointIds.filter((j) => this.getJoint(j)?.bodyId === null);
    }
    this.groups = this.groups.filter((g) => g.bodyIds.length + g.jointIds.length >= 2);
  }

  // --- construction guidelines ----------------------------------------------

  /** Below this separation two guide points can't define a direction (rejected). */
  static readonly GUIDE_MIN_SPAN = 1e-6;

  getGuide(id: number): Guide | undefined {
    return this.guides.find((g) => g.id === id);
  }

  /** Create a guideline through two points. Returns null when they (nearly) coincide. */
  addGuide(a: Vec2, b: Vec2): Guide | null {
    if (dist(a, b) < Scene.GUIDE_MIN_SPAN) return null;
    const guide: Guide = { id: this.id(), a: clone(a), b: clone(b) };
    this.guides.push(guide);
    return guide;
  }

  removeGuide(id: number): void {
    this.guides = this.guides.filter((g) => g.id !== id);
    // Constraints / measurements referencing the removed guide cascade away.
    this.pruneMeasurements();
    this.pruneSketch();
  }

  /** Translate a whole guideline by `delta` (both points move; the angle is preserved). */
  moveGuide(id: number, delta: Vec2): void {
    const g = this.getGuide(id);
    if (!g) return;
    g.a = add(g.a, delta);
    g.b = add(g.b, delta);
  }

  /**
   * Move one of a guideline's defining points to `worldPos` (re-aiming the line).
   * Ignored when it would land on the other point (the line needs a direction).
   */
  moveGuidePoint(id: number, which: "a" | "b", worldPos: Vec2): void {
    const g = this.getGuide(id);
    if (!g) return;
    const other = which === "a" ? g.b : g.a;
    if (dist(worldPos, other) < Scene.GUIDE_MIN_SPAN) return;
    g[which] = clone(worldPos);
  }

  /** Nearest guideline whose **infinite** line passes within `radius` of `p` (topmost first). */
  guideAt(p: Vec2, radius: number): Guide | undefined {
    let best: Guide | undefined;
    let bestD = radius;
    for (let i = this.guides.length - 1; i >= 0; i--) {
      const g = this.guides[i];
      const d = distToLine(p, g.a, normalize(sub(g.b, g.a)));
      if (d <= bestD) {
        bestD = d;
        best = g;
      }
    }
    return best;
  }

  /** Guideline defining point within `radius` of `p` (topmost first), or null.
   *  `excludeGuide` leaves one guideline's points out (a dragged point can't pick itself). */
  guidePointAt(p: Vec2, radius: number, excludeGuide?: number): { guide: Guide; which: "a" | "b" } | null {
    let best: { guide: Guide; which: "a" | "b" } | null = null;
    let bestD = radius;
    for (let i = this.guides.length - 1; i >= 0; i--) {
      const g = this.guides[i];
      if (g.id === excludeGuide) continue;
      for (const which of ["a", "b"] as const) {
        const d = dist(g[which], p);
        if (d <= bestD) {
          bestD = d;
          best = { guide: g, which };
        }
      }
    }
    return best;
  }

  /**
   * Snapshot a selection of bodies (with their joints) and free joints into a copy/paste
   * clip (see `SelectionClip`). Constraints travel when every joint they reference is in
   * the copied set — so pins *between* copied bodies are kept, while anything reaching
   * outside the selection is dropped. Permanent groups among the copied bodies travel
   * too. Returns null when nothing copyable is selected.
   */
  extractSelection(
    bodyIds: number[],
    freeJointIds: number[] = [],
    opts?: { drivenDims?: boolean }
  ): SelectionClip | null {
    const bodies = [...new Set(bodyIds)]
      .map((id) => this.getBody(id))
      .filter((b): b is Body => b !== undefined);
    const bodyIdSet = new Set(bodies.map((b) => b.id));
    const freeJoints = [...new Set(freeJointIds)]
      .map((id) => this.getJoint(id))
      .filter((j): j is Joint => j !== undefined && j.bodyId === null);
    const attached = this.joints.filter((j) => j.bodyId !== null && bodyIdSet.has(j.bodyId));
    const copiedJoints = [...attached, ...freeJoints];
    if (bodies.length === 0 && copiedJoints.length === 0) return null;
    const owned = new Set(copiedJoints.map((j) => j.id));
    // Paste reference: mass-weighted centre of the bodies (a single body's own centroid),
    // or the joints' average for a body-less clip.
    let center: Vec2;
    if (bodies.length > 0) {
      let m = 0;
      let cx = 0;
      let cy = 0;
      for (const b of bodies) {
        const bm = 1 / b.invMass;
        m += bm;
        cx += bm * b.pos.x;
        cy += bm * b.pos.y;
      }
      center = vec(cx / m, cy / m);
    } else {
      let cx = 0;
      let cy = 0;
      for (const j of copiedJoints) {
        const w = this.jointWorld(j);
        cx += w.x;
        cy += w.y;
      }
      center = vec(cx / copiedJoints.length, cy / copiedJoints.length);
    }
    const grounds: SelectionClip["grounds"] = [];
    const sliders: SelectionClip["sliders"] = [];
    const pins: SelectionClip["pins"] = [];
    for (const c of this.constraints) {
      if (c.kind === "ground" && owned.has(c.joint)) {
        grounds.push({ joint: c.joint, anchor: clone(c.anchor) });
      } else if (c.kind === "slider" && owned.has(c.railA) && owned.has(c.railB)) {
        sliders.push({
          tmp: c.id,
          railA: c.railA,
          railB: c.railB,
          riders: c.riders.filter((r) => owned.has(r)),
          locked: c.locked.filter((r) => owned.has(r)),
        });
      } else if (c.kind === "pin" && owned.has(c.jointA) && owned.has(c.jointB)) {
        pins.push({ a: c.jointA, b: c.jointB, rigid: c.rigid === true });
      }
    }
    // Powered constraints travel when everything they reference does.
    const clippedSliderIds = new Set(sliders.map((s) => s.tmp));
    const actuators: SelectionClip["actuators"] = [];
    const motors: SelectionClip["motors"] = [];
    for (const c of this.constraints) {
      if (c.kind === "linearActuator" && clippedSliderIds.has(c.sliderId) && owned.has(c.riderId)) {
        actuators.push({ slider: c.sliderId, rider: c.riderId, speed: c.speed, profile: c.profile });
      } else if (
        c.kind === "motor" &&
        bodyIdSet.has(c.bodyId) &&
        owned.has(c.pivotJointId) &&
        owned.has(c.crankJointId)
      ) {
        motors.push({ body: c.bodyId, pivot: c.pivotJointId, crank: c.crankJointId, speed: c.speed });
      }
    }
    // Permanent groups whose members were copied (at least 2 of them) travel with the clip.
    const freeJointIdSet = new Set(freeJoints.map((j) => j.id));
    const groups: SelectionClip["groups"] = [];
    for (const g of this.groups) {
      const bodies2 = g.bodyIds.filter((id) => bodyIdSet.has(id));
      const joints2 = g.jointIds.filter((id) => freeJointIdSet.has(id));
      if (bodies2.length + joints2.length >= 2) groups.push({ bodies: bodies2, joints: joints2 });
    }
    // A ref is internal when its element travels with the clip: a copied joint, a copied
    // body's vertices/edges/frame, or the rail of a copied slider.
    const clippedSliders = new Set(sliders.map((s) => s.tmp));
    const internal = (r: MeasureRef | null): boolean => {
      if (!r) return true;
      switch (r.kind) {
        case "joint":
          return owned.has(r.jointId);
        case "vertex":
        case "edge":
        case "bodyPoint":
          return bodyIdSet.has(r.bodyId);
        case "rail":
          return clippedSliders.has(r.sliderId);
        case "patternAxis": {
          const p = this.getPattern(r.patternId);
          return !!p && bodyIdSet.has(p.bodyId);
        }
        case "guidePoint":
        case "guideLine":
          return false; // guides don't travel with a selection clip
      }
    };
    const sketch: SelectionClip["sketch"] = [];
    for (const c of this.sketch) {
      if (internal(c.refA) && internal(c.refB)) {
        sketch.push({
          kind: c.kind,
          refA: cloneMeasureRef(c.refA),
          refB: c.refB ? cloneMeasureRef(c.refB) : null,
        });
      }
    }
    const dims: SelectionClip["dims"] = [];
    for (const m of this.measurements) {
      const driving = m.driving === true && m.target !== undefined;
      // Driving dimensions always travel (they're constraints); driven reference ones are
      // annotations and travel only on request (component creation keeps them).
      if (!driving && !opts?.drivenDims) continue;
      if (m.mode === "draw" && internal(m.refA) && internal(m.refB)) {
        dims.push({
          refA: cloneMeasureRef(m.refA),
          refB: cloneMeasureRef(m.refB),
          labelOffset: clone(m.labelOffset),
          axis: m.axis,
          target: driving ? m.target : undefined,
        });
      }
    }
    return {
      center,
      bodies: bodies.map((b) => ({
        tmp: b.id,
        controlWorld: this.bodyControlWorld(b).map(clone),
        holes: (b.holes ?? []).map((h, hi) => ({
          control: this.bodyHoleControlWorld(b, hi),
          radius: h.radius,
          radii: h.radii ? [...h.radii] : undefined,
          round: h.round,
        })),
        radius: b.radius,
        radii: b.radii ? [...b.radii] : undefined,
        round: b.round,
        color: b.color,
        grounded: b.grounded,
      })),
      joints: copiedJoints.map((j) => ({ tmp: j.id, bodyTmp: j.bodyId, world: this.jointWorld(j) })),
      grounds,
      sliders,
      pins,
      actuators,
      motors,
      groups,
      sketch,
      dims,
      patterns: this.patterns
        .filter((p) => bodyIdSet.has(p.bodyId))
        .map((p) => ({
          tmp: p.id,
          body: p.bodyId,
          seed: p.seed.kind === "hole" ? { kind: "hole" as const, hole: p.seed.hole } : { kind: "joint" as const, joint: p.seed.jointId },
          layout: rotateLayout(cloneLayout(p.layout), this.getBody(p.bodyId)!.angle),
          members: [...p.members],
        })),
    };
  }

  /** Single-body convenience wrapper around `extractSelection`. */
  extractBody(bodyId: number): SelectionClip | null {
    return this.getBody(bodyId) ? this.extractSelection([bodyId], []) : null;
  }

  /**
   * Paste a `SelectionClip` so its `center` lands at `at` (the whole fragment is
   * translated by `at − clip.center`). Recreates the bodies, joints, clipped constraints,
   * and permanent groups with fresh ids. Returns the new body / free-joint ids (for
   * re-selecting the pasted fragment), or null on failure.
   */
  insertSelection(
    clip: SelectionClip,
    at: Vec2
  ): { bodyIds: number[]; freeJointIds: number[] } | null {
    const offset = sub(at, clip.center);
    const bodyIdMap = new Map<number, number>(); // body tmp id → new body id
    for (const b of clip.bodies) {
      const body = this.addBody(
        b.controlWorld.map((p) => add(p, offset)),
        b.radius,
        b.round,
        (b.holes ?? []).map((h) => ({ ...h, control: h.control.map((p) => add(p, offset)) })),
        b.radii // paste keeps per-corner radius overrides
      );
      if (body.local.length < 3) return null;
      body.color = b.color; // paste keeps the source body's colour
      body.grounded = b.grounded ?? false; // ...and whether it's grounded
      bodyIdMap.set(b.tmp, body.id);
    }
    const idMap = new Map<number, number>(); // joint tmp id → new joint id
    const freeJointIds: number[] = [];
    for (const j of clip.joints) {
      if (j.bodyTmp === null) {
        const nj = this.addFreeJoint(add(j.world, offset));
        idMap.set(j.tmp, nj.id);
        freeJointIds.push(nj.id);
      } else {
        const bid = bodyIdMap.get(j.bodyTmp);
        if (bid === undefined) continue;
        idMap.set(j.tmp, this.addJoint(bid, add(j.world, offset)).id);
      }
    }
    // Grounds before sliders, so addSlider's auto-grounding of free rail joints sees them
    // already grounded (a copied world-fixed track keeps exactly one ground per rail joint).
    for (const g of clip.grounds) {
      const id = idMap.get(g.joint);
      if (id !== undefined) this.addGround(id, add(g.anchor, offset));
    }
    const sliderIdMap = new Map<number, number>(); // tmp slider id → new slider id
    for (const s of clip.sliders) {
      const a = idMap.get(s.railA);
      const b = idMap.get(s.railB);
      if (a === undefined || b === undefined) continue;
      const sl = this.addSlider(a, b);
      sliderIdMap.set(s.tmp, sl.id);
      for (const r of s.riders) {
        const nr = idMap.get(r);
        if (nr !== undefined) this.attachSliderRider(sl.id, nr, (s.locked ?? []).includes(r));
      }
    }
    for (const p of clip.pins) {
      const a = idMap.get(p.a);
      const b = idMap.get(p.b);
      if (a !== undefined && b !== undefined) this.addPin(a, b, p.rigid === true);
    }
    for (const a of clip.actuators ?? []) {
      const slider = sliderIdMap.get(a.slider);
      const rider = idMap.get(a.rider);
      if (slider === undefined || rider === undefined) continue;
      this.constraints.push({
        kind: "linearActuator",
        id: this.id(),
        sliderId: slider,
        riderId: rider,
        speed: a.speed,
        profile: a.profile,
      });
    }
    for (const m of clip.motors ?? []) {
      const body = bodyIdMap.get(m.body);
      const pivot = idMap.get(m.pivot);
      const crank = idMap.get(m.crank);
      if (body === undefined || pivot === undefined || crank === undefined) continue;
      this.constraints.push({
        kind: "motor",
        id: this.id(),
        bodyId: body,
        pivotJointId: pivot,
        crankJointId: crank,
        speed: m.speed,
      });
    }
    for (const g of clip.groups) {
      const bids = g.bodies.map((t) => bodyIdMap.get(t)).filter((x): x is number => x !== undefined);
      const jids = g.joints.map((t) => idMap.get(t)).filter((x): x is number => x !== undefined);
      if (bids.length + jids.length >= 2) this.addGroup(bids, jids);
    }
    // Patterns: the pasted bodies keep their hole order and sit at angle 0, so the
    // world-oriented layout vectors apply as-is; joint members map through idMap.
    const patternIdMap = new Map<number, number>(); // tmp pattern id → new pattern id
    for (const pc of clip.patterns ?? []) {
      const bid = bodyIdMap.get(pc.body);
      if (bid === undefined) continue;
      let seed: Pattern["seed"];
      if (pc.seed.kind === "hole") seed = { kind: "hole", hole: pc.seed.hole };
      else {
        const jid = idMap.get(pc.seed.joint);
        if (jid === undefined) continue;
        seed = { kind: "joint", jointId: jid };
      }
      const members =
        pc.seed.kind === "hole"
          ? [...pc.members]
          : pc.members.map((t) => idMap.get(t)).filter((x): x is number => x !== undefined);
      const p: Pattern = { id: this.id(), bodyId: bid, seed, layout: cloneLayout(pc.layout), members };
      this.patterns.push(p);
      patternIdMap.set(pc.tmp, p.id);
      this.syncPattern(p);
    }
    // Recreate the clipped sketch constraints / driving dimensions on the new elements.
    // Everything they express is translation-invariant, so the pasted geometry already
    // satisfies them — no solve needed. Vertex/edge indices carry over unchanged
    // (addBody keeps the control polygon's order).
    const remapRef = (r: MeasureRef): MeasureRef | null => {
      switch (r.kind) {
        case "joint": {
          const id = idMap.get(r.jointId);
          return id === undefined ? null : { kind: "joint", jointId: id };
        }
        case "vertex":
        case "edge":
        case "bodyPoint": {
          const bid = bodyIdMap.get(r.bodyId);
          if (bid === undefined) return null;
          if (r.kind === "bodyPoint") return { kind: "bodyPoint", bodyId: bid, local: clone(r.local) };
          return r.hole === undefined
            ? { kind: r.kind, bodyId: bid, index: r.index }
            : { kind: r.kind, bodyId: bid, index: r.index, hole: r.hole }; // hole refs keep their hole
        }
        case "rail": {
          const id = sliderIdMap.get(r.sliderId);
          return id === undefined ? null : { kind: "rail", sliderId: id };
        }
        case "patternAxis": {
          const id = patternIdMap.get(r.patternId);
          return id === undefined ? null : { kind: "patternAxis", patternId: id, axis: r.axis };
        }
        case "guidePoint":
        case "guideLine":
          return null; // guides never travel with a clip (extractSelection drops these refs)
      }
    };
    for (const c of clip.sketch) {
      const ra = remapRef(c.refA);
      const rb = c.refB ? remapRef(c.refB) : null;
      if (!ra || (c.refB && !rb)) continue;
      this.addSketchConstraint(c.kind, ra, rb ?? undefined);
    }
    for (const d of clip.dims) {
      const ra = remapRef(d.refA);
      const rb = remapRef(d.refB);
      if (!ra || !rb || !this.resolveMeasureRef(ra) || !this.resolveMeasureRef(rb)) continue;
      const m: Measurement = {
        id: this.id(),
        mode: "draw",
        refA: ra,
        refB: rb,
        labelOffset: clone(d.labelOffset),
        axis: d.axis,
      };
      if (d.target !== undefined) {
        m.driving = true;
        m.target = d.target;
      }
      this.measurements.push(m);
      // The pasted fragment is a pure translation of the source, so the current
      // geometry shows the side the dimension was holding — capture it.
      if (m.driving) this.captureMeasurementSide(m);
    }
    return { bodyIds: [...bodyIdMap.values()], freeJointIds };
  }

  /** Single-body convenience wrapper around `insertSelection`: returns the new body's id. */
  insertBody(clip: SelectionClip, at: Vec2): number | null {
    return this.insertSelection(clip, at)?.bodyIds[0] ?? null;
  }

  // --- components (definitions + materialized instances) ---------------------

  getComponent(id: number): ComponentDef | undefined {
    return this.components.find((c) => c.id === id);
  }

  /** The instance owning a body / joint / constraint of this context, or undefined. */
  instanceOfBody(bodyId: number): ComponentInstance | undefined {
    return this.instances.find((i) => i.bodyMap.some((e) => e.id === bodyId));
  }

  instanceOfJoint(jointId: number): ComponentInstance | undefined {
    return this.instances.find(
      (i) => i.jointMap.some((e) => e.id === jointId) || i.anchorMap.some((e) => e.id === jointId)
    );
  }

  instanceOfConstraint(constraintId: number): ComponentInstance | undefined {
    return this.instances.find((i) => i.constraintMap.some((e) => e.id === constraintId));
  }

  /**
   * Every occurrence of `defId` in this context: its direct instances, plus instances
   * nested inside other components' instances at any depth. A nested instance has no
   * record in this context (only its enclosing instance does), so its material is traced
   * def-local id → enclosing instance map → … → this context's ids.
   */
  componentOccurrences(defId: number): ComponentOccurrence[] {
    const out: ComponentOccurrence[] = [];
    type Lift = (id: number) => number | undefined;
    const lifted = (entries: InstanceMapEntry[], lift: Lift): number[] =>
      entries.map((e) => lift(e.id)).filter((id): id is number => id !== undefined);
    const visit = (instances: ComponentInstance[], liftBody: Lift, liftJoint: Lift, depth: number): void => {
      if (depth > 64) return; // defs form a DAG — a guard against corrupt data only
      for (const inst of instances) {
        const jointEntries = [...inst.jointMap, ...inst.anchorMap];
        if (inst.defId === defId) {
          out.push({ bodyIds: lifted(inst.bodyMap, liftBody), jointIds: lifted(jointEntries, liftJoint) });
        }
        const def = this.getComponent(inst.defId);
        if (!def || !(def.data.instances ?? []).length) continue;
        // Nested instances speak the def's local ids: map src → this instance's id, then lift on.
        const bodyBySrc = new Map(inst.bodyMap.map((e) => [e.src, e.id]));
        const jointBySrc = new Map(jointEntries.map((e) => [e.src, e.id]));
        const through = (m: Map<number, number>, lift: Lift): Lift => (id) => {
          const local = m.get(id);
          return local === undefined ? undefined : lift(local);
        };
        visit(def.data.instances ?? [], through(bodyBySrc, liftBody), through(jointBySrc, liftJoint), depth + 1);
      }
    };
    const identity: Lift = (id) => id;
    visit(this.instances, identity, identity, 0);
    return out;
  }

  /**
   * The definitions a body / joint of this context belongs to, outermost first: the
   * enclosing instance's def, then each nested def down to the one owning the element.
   * Empty for plain (non-instance) material.
   */
  componentChainOf(kind: "body" | "joint", id: number): number[] {
    const chain: number[] = [];
    let instances = this.instances;
    let cur = id;
    for (let depth = 0; depth < 64; depth++) {
      let hit: { inst: ComponentInstance; src: number } | null = null;
      for (const inst of instances) {
        const entries = kind === "body" ? inst.bodyMap : [...inst.jointMap, ...inst.anchorMap];
        const e = entries.find((x) => x.id === cur);
        if (e) {
          hit = { inst, src: e.src };
          break;
        }
      }
      if (!hit) break;
      chain.push(hit.inst.defId);
      const def = this.getComponent(hit.inst.defId);
      if (!def) break;
      instances = def.data.instances ?? [];
      cur = hit.src;
    }
    return chain;
  }

  /** The component instance that owns a reference's element, or undefined for plain /
   *  guide geometry. */
  instanceOfRef(ref: MeasureRef): ComponentInstance | undefined {
    switch (ref.kind) {
      case "joint":
        return this.instanceOfJoint(ref.jointId);
      case "vertex":
      case "edge":
      case "bodyPoint":
        return this.instanceOfBody(ref.bodyId);
      case "rail":
        return this.instanceOfConstraint(ref.sliderId);
      default:
        return undefined;
    }
  }

  /** Whether a measurement/sketch reference names instance-owned geometry (whose shape is
   *  locked: the sketch solver never moves it — a constraint or dimension with one free
   *  end moves the free side; one with every end instance-owned drives *poses*, see
   *  pose.ts). */
  refInstanceOwned(ref: MeasureRef): boolean {
    return this.instanceOfRef(ref) !== undefined;
  }

  /**
   * Key of the draw-mode rigid unit a reference's element belongs to: a group (`g:`,
   * e.g. a component chassis), a lone body (`b:`), or a lone free joint (`j:`). Two
   * refs with the same key are rigid to one another — a pose dimension between them
   * can never change, so driving it is rejected.
   */
  refRigidUnitKey(ref: MeasureRef): string | null {
    const bodyKey = (bodyId: number): string | null => {
      if (!this.getBody(bodyId)) return null;
      const g = this.groupOf(bodyId);
      return g ? `g:${g.id}` : `b:${bodyId}`;
    };
    switch (ref.kind) {
      case "vertex":
      case "edge":
      case "bodyPoint":
        return bodyKey(ref.bodyId);
      case "joint": {
        const j = this.getJoint(ref.jointId);
        if (!j) return null;
        if (j.bodyId !== null) return bodyKey(j.bodyId);
        const g = this.groupOfJoint(j.id);
        return g ? `g:${g.id}` : `j:${j.id}`;
      }
      case "rail": {
        const c = this.constraints.find((x) => x.kind === "slider" && x.id === ref.sliderId);
        if (!c || c.kind !== "slider") return null;
        return this.refRigidUnitKey({ kind: "joint", jointId: c.railA });
      }
      case "patternAxis": {
        const p = this.getPattern(ref.patternId);
        return p ? bodyKey(p.bodyId) : null;
      }
      default:
        return null; // guides aren't rigid material
    }
  }

  /**
   * Translate a whole component instance rigidly: every expanded body and free joint
   * (mechanism joints and synthesized anchors alike) moves by `delta` — placement
   * motion, the same thing dragging the instance does. Shapes are untouched.
   */
  moveInstance(instanceId: number, delta: Vec2): void {
    const inst = this.instances.find((i) => i.id === instanceId);
    if (!inst) return;
    for (const e of inst.bodyMap) this.moveBody(e.id, delta);
    for (const e of [...inst.jointMap, ...inst.anchorMap]) {
      const j = this.getJoint(e.id);
      if (j && j.bodyId === null) this.moveJoint(j.id, delta);
    }
  }

  /**
   * Rigidly rotate a whole component instance by `delta` radians about a world `pivot`:
   * every expanded body turns with `rotateBody` (attached joints ride along), every free
   * joint (mechanism joints and synthesized anchors alike) orbits the pivot — the same
   * motion the rotate drag makes. Shapes are untouched and the placement derived by
   * `instancePlacement` stays coherent (chassis poses and free points turn together).
   */
  rotateInstance(instanceId: number, pivot: Vec2, delta: number): void {
    const inst = this.instances.find((i) => i.id === instanceId);
    if (!inst || delta === 0) return;
    for (const e of inst.bodyMap) this.rotateBody(e.id, pivot, delta);
    for (const e of [...inst.jointMap, ...inst.anchorMap]) {
      const j = this.getJoint(e.id);
      if (!j || j.bodyId !== null) continue;
      const w = this.jointWorld(j);
      const nw = add(pivot, rotate(sub(w, pivot), delta));
      this.moveJoint(j.id, sub(nw, w));
    }
  }

  /** Every def id a definition's expansion (transitively) uses — for cycle checks:
   *  a def may not be instantiated into a context whose def it uses. */
  componentUses(defId: number): Set<number> {
    const used = new Set<number>();
    const visit = (id: number): void => {
      const def = this.getComponent(id);
      if (!def) return;
      for (const i of def.data.instances ?? []) {
        if (!used.has(i.defId)) {
          used.add(i.defId);
          visit(i.defId);
        }
      }
    };
    visit(defId);
    return used;
  }

  /** Centre of a definition's bounding box in def-frame coordinates (for placement). */
  componentCenter(defId: number): Vec2 {
    const def = this.getComponent(defId);
    if (!def) return vec(0, 0);
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    const include = (p: Vec2): void => {
      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
    };
    for (const b of def.data.bodies) {
      for (const l of b.local) include(add(b.pos, rotate(l, b.angle)));
    }
    for (const j of def.data.joints) {
      if (j.bodyId === null) include(j.local);
    }
    return Number.isFinite(minX) ? vec((minX + maxX) / 2, (minY + maxY) / 2) : vec(0, 0);
  }

  /** Create a new, completely empty component definition (no instance is placed) —
   *  meant to be opened for editing and filled with bodies and/or instances of other
   *  components. Instances of it can only be placed once it has content
   *  (`instantiateComponent` refuses an empty definition). */
  createEmptyComponent(name: string): ComponentDef {
    const tmp = new Scene();
    tmp.unit = this.unit;
    const defId = this.components.reduce((m, c) => Math.max(m, c.id), 0) + 1;
    const def: ComponentDef = { id: defId, name, data: tmp.serializeContext() };
    this.components.push(def);
    return def;
  }

  /**
   * Turn a selection into a new component definition and replace it with one instance
   * placed exactly where the originals were. The definition's frame is the selection's
   * current world coordinates. Everything internal to the selection travels into the
   * definition — bodies (colour, grounded flag), joints, grounds, pins, sliders,
   * actuators/motors, internal sketch constraints and dimensions (driving *and* driven).
   * Anything reaching outside the selection (a pin to an uncopied body, a guide ref) is
   * dropped, like copy/paste. Returns null when the selection is empty, has no body, or
   * contains material already owned by another instance.
   */
  createComponentFromSelection(
    name: string,
    bodyIds: number[],
    freeJointIds: number[] = []
  ): { def: ComponentDef; instance: ComponentInstance } | null {
    const bodies = [...new Set(bodyIds)].filter((id) => this.getBody(id));
    const joints = [...new Set(freeJointIds)].filter((id) => this.getJoint(id)?.bodyId === null);
    if (bodies.length === 0) return null;
    if (bodies.some((id) => this.instanceOfBody(id)) || joints.some((id) => this.instanceOfJoint(id))) {
      return null; // instance material belongs to its own definition
    }
    const clip = this.extractSelection(bodies, joints, { drivenDims: true });
    if (!clip) return null;
    const tmp = new Scene();
    tmp.unit = this.unit;
    if (!tmp.insertSelection(clip, clip.center)) return null; // zero offset: def frame = world
    const defId = this.components.reduce((m, c) => Math.max(m, c.id), 0) + 1;
    const def: ComponentDef = { id: defId, name, data: tmp.serializeContext() };
    this.components.push(def);
    for (const id of bodies) this.removeBody(id);
    for (const id of joints) if (this.getJoint(id)) this.removeJoint(id);
    const instance = this.instantiateComponent(def.id, { pos: vec(0, 0), angle: 0 })!;
    return { def, instance };
  }

  /** Expand one new instance of a definition into this context, placed by `t` (def frame
   *  → context). Returns null for an unknown or still-empty definition (an instance with
   *  no material would dissolve immediately). */
  instantiateComponent(defId: number, t: InstanceTransform): ComponentInstance | null {
    const def = this.getComponent(defId);
    if (!def) return null;
    if (def.data.bodies.length === 0 && def.data.joints.length === 0) return null;
    const inst = this.expandInstance(def, null, t);
    this.instances.push(inst);
    return inst;
  }

  /**
   * Fork an instance's definition: deep-copy the definition into a new one and re-point
   * the instance at the copy, so the two evolve independently from here on. The
   * instance's expanded material is untouched — the copy is identical, so every
   * provenance src id stays valid and no re-expansion is needed. Definitions the copy
   * itself instantiates stay shared (the def DAG gains a node, edges unchanged).
   * Returns the new definition, or null for an unknown instance / definition.
   */
  makeInstanceUnique(instanceId: number): ComponentDef | null {
    const inst = this.instances.find((i) => i.id === instanceId);
    const def = inst ? this.getComponent(inst.defId) : undefined;
    if (!inst || !def) return null;
    const defId = this.components.reduce((m, c) => Math.max(m, c.id), 0) + 1;
    let name = `${def.name} copy`;
    for (let n = 2; this.components.some((c) => c.name === name); n++) {
      name = `${def.name} copy ${n}`;
    }
    const copy: ComponentDef = {
      id: defId,
      name,
      data: JSON.parse(JSON.stringify(def.data)) as SceneData,
    };
    this.components.push(copy);
    inst.defId = defId;
    return copy;
  }

  /**
   * Re-expand every instance whose definition is in `changed`, reconciling the expanded
   * elements against the (new) definition: surviving elements keep their scene identity
   * (ids), every part's pose snaps to the definition's layout at the instance's current
   * placement (the def is the reference — cascades through nested defs), new elements
   * appear, removed ones cascade away. Returns whether anything was re-expanded.
   */
  reexpandInstances(changed: ReadonlySet<number>): boolean {
    let did = false;
    for (const inst of [...this.instances]) {
      if (!changed.has(inst.defId)) continue;
      const def = this.getComponent(inst.defId);
      if (!def) {
        this.removeInstance(inst.id);
        did = true;
        continue;
      }
      this.expandInstance(def, inst, this.instanceTransform(inst, def));
      did = true;
    }
    if (did) this.pruneInstances();
    return did;
  }

  /** Remove an instance and everything it expanded to. */
  removeInstance(instanceId: number): void {
    const inst = this.instances.find((i) => i.id === instanceId);
    if (!inst) return;
    // Detach the record first: removeBody/removeJoint prune instance maps as they go,
    // so snapshot the member lists and work from those.
    this.instances = this.instances.filter((i) => i.id !== instanceId);
    const bodyIds = inst.bodyMap.map((e) => e.id);
    const jointIds = [...inst.jointMap, ...inst.anchorMap].map((e) => e.id);
    const conIds = inst.constraintMap.map((e) => e.id);
    const gids = new Set(inst.groupMap.map((e) => e.id));
    if (inst.groupId !== null) gids.add(inst.groupId);
    for (const id of bodyIds) if (this.getBody(id)) this.removeBody(id);
    for (const id of jointIds) if (this.getJoint(id)) this.removeJoint(id);
    for (const id of conIds) this.removeConstraint(id);
    this.groups = this.groups.filter((g) => !gids.has(g.id));
  }

  /** Dissolve an instance into plain elements: the record is dropped, the expanded
   *  bodies/joints/constraints (and the chassis group, now an ordinary rigid group)
   *  stay behind and stop following the definition. */
  dissolveInstance(instanceId: number): void {
    this.instances = this.instances.filter((i) => i.id !== instanceId);
  }

  /** Delete a definition. Refused (returns false) while any instance of it exists in this
   *  context or inside any other definition's data (see `dissolveComponent` for the
   *  converting variant). */
  removeComponent(defId: number): boolean {
    if (this.instances.some((i) => i.defId === defId)) return false;
    for (const c of this.components) {
      if (c.id !== defId && (c.data.instances ?? []).some((i) => i.defId === defId)) return false;
    }
    const before = this.components.length;
    this.components = this.components.filter((c) => c.id !== defId);
    return this.components.length !== before;
  }

  /**
   * Delete a definition, converting every instance of it into plain elements: each
   * instance's expanded bodies / joints / constraints stay exactly where they are and
   * stop following the definition (its chassis group survives as an ordinary rigid
   * group, so the bodies from one instance still move as a block — but instances no
   * longer share anything with each other). Applies to instances in this context AND
   * inside every other definition's stored data. Nested material needs no re-expansion:
   * a definition that instantiated the deleted one already carries the expanded
   * elements in its own data, so dropping the record there changes nothing anyone sees.
   * Returns the number of instances dissolved, or -1 for an unknown definition.
   */
  dissolveComponent(defId: number): number {
    if (!this.getComponent(defId)) return -1;
    let n = 0;
    for (const inst of [...this.instances]) {
      if (inst.defId !== defId) continue;
      this.dissolveInstance(inst.id);
      n++;
    }
    for (const c of this.components) {
      if (c.id === defId || !c.data.instances) continue;
      const kept = c.data.instances.filter((i) => i.defId !== defId);
      n += c.data.instances.length - kept.length;
      c.data.instances = kept;
    }
    this.components = this.components.filter((c) => c.id !== defId);
    return n;
  }

  /** Public view of an instance's current placement (def frame → context). */
  instancePlacement(instanceId: number): InstanceTransform | null {
    const inst = this.instances.find((i) => i.id === instanceId);
    const def = inst ? this.getComponent(inst.defId) : undefined;
    return inst && def ? this.instanceTransform(inst, def) : null;
  }

  /** The instance's current placement (def frame → context), derived from a surviving
   *  reference body's scene pose vs its cached def pose (chassis bodies preferred — they
   *  move rigidly with the instance). Falls back to a surviving free joint (translation
   *  only), then to identity. */
  private instanceTransform(inst: ComponentInstance, def: ComponentDef): InstanceTransform {
    const pick =
      inst.bodyMap.find((e) => e.chassis && this.getBody(e.id)) ??
      inst.bodyMap.find((e) => this.getBody(e.id));
    if (pick) {
      const b = this.getBody(pick.id)!;
      const angle = b.angle - pick.defAngle;
      return { pos: sub(b.pos, rotate(pick.defPos, angle)), angle };
    }
    const dJoint = new Map(def.data.joints.map((j) => [j.id, j]));
    const dBody = new Map(def.data.bodies.map((b) => [b.id, b]));
    for (const e of inst.jointMap) {
      const j = this.getJoint(e.id);
      const dj = dJoint.get(e.src);
      if (!j || j.bodyId !== null || !dj) continue;
      const dw = dj.bodyId === null
        ? dj.local
        : add(dBody.get(dj.bodyId)!.pos, rotate(dj.local, dBody.get(dj.bodyId)!.angle));
      return { pos: sub(this.jointWorld(j), dw), angle: 0 };
    }
    return { pos: vec(0, 0), angle: 0 };
  }

  /**
   * The expansion engine: materialize (or reconcile) one instance of `def` into this
   * context at placement `t`. With `inst` given, elements are matched by their def-local
   * src ids — surviving ones are updated in place (design fields AND poses from the def:
   * every part snaps to the def layout at the instance's placement), missing ones
   * created, orphaned ones removed. Ground constraints inside the definition are **converted**, never expanded:
   * a grounded body joins the rigid chassis group; a grounded free joint becomes a
   * group-locked chassis point; a joint-ground on a non-grounded body becomes a pin to a
   * synthesized chassis point (revolute to the component's frame). The def's own groups
   * (user groups, nested chassis) are recreated as plain groups.
   */
  private expandInstance(
    def: ComponentDef,
    inst: ComponentInstance | null,
    t: InstanceTransform
  ): ComponentInstance {
    const d = def.data;
    const xf = (p: Vec2): Vec2 => add(t.pos, rotate(p, t.angle));
    const dBody = new Map(d.bodies.map((b) => [b.id, b]));
    const dJoint = new Map(d.joints.map((j) => [j.id, j]));
    const defJointWorld = (j: Joint): Vec2 =>
      j.bodyId === null
        ? j.local
        : add(dBody.get(j.bodyId)!.pos, rotate(j.local, dBody.get(j.bodyId)!.angle));

    // Classify the definition's grounds: chassis bodies, chassis free joints, and
    // joint-grounds on non-grounded bodies (each of those becomes a pinned anchor point).
    const chassisBodySrc = new Set(d.bodies.filter((b) => b.grounded).map((b) => b.id));
    const chassisJointSrc = new Set<number>();
    const anchorSrcs: GroundConstraint[] = [];
    for (const c of d.constraints) {
      if (c.kind !== "ground") continue;
      const j = dJoint.get(c.joint);
      if (!j) continue;
      if (j.bodyId === null) chassisJointSrc.add(j.id);
      else if (!chassisBodySrc.has(j.bodyId)) anchorSrcs.push(c);
    }

    // --- bodies ---------------------------------------------------------------
    const oldBodyEntries = new Map((inst?.bodyMap ?? []).map((e) => [e.src, e]));
    const bodyIdMap = new Map<number, number>();
    const bodyMap: InstanceBodyEntry[] = [];
    for (const db of d.bodies) {
      const chassis = chassisBodySrc.has(db.id);
      const old = oldBodyEntries.get(db.id);
      let sb = old ? this.getBody(old.id) : undefined;
      if (sb) {
        // Design AND pose come from the def — the definition is the reference, so a
        // re-expansion snaps every part back to the def layout at the instance's
        // placement. Only the grounded flag is instance state (grounding an instance
        // happens outside).
        sb.controlLocal = db.controlLocal.map((p) => vec(p.x, p.y));
        sb.local = db.local.map((p) => vec(p.x, p.y));
        sb.radius = db.radius;
        if (db.radii) sb.radii = [...db.radii];
        else delete sb.radii;
        sb.round = db.round;
        if (db.holes?.length) {
          sb.holes = cloneBodyHoles(db.holes);
          sb.holesLocal = (db.holesLocal ?? []).map((l) => l.map((p) => vec(p.x, p.y)));
        } else {
          delete sb.holes;
          delete sb.holesLocal;
        }
        sb.invMass = db.invMass;
        sb.invInertia = db.invInertia;
        sb.color = db.color;
        sb.pos = xf(db.pos);
        sb.angle = db.angle + t.angle;
      } else {
        sb = {
          id: this.id(),
          controlLocal: db.controlLocal.map((p) => vec(p.x, p.y)),
          radius: db.radius,
          round: db.round,
          ...(db.radii ? { radii: [...db.radii] } : {}),
          local: db.local.map((p) => vec(p.x, p.y)),
          pos: xf(db.pos),
          angle: db.angle + t.angle,
          invMass: db.invMass,
          invInertia: db.invInertia,
          color: db.color,
          grounded: false,
        };
        if (db.holes?.length) {
          sb.holes = cloneBodyHoles(db.holes);
          sb.holesLocal = (db.holesLocal ?? []).map((l) => l.map((p) => vec(p.x, p.y)));
        }
        this.bodies.push(sb);
      }
      bodyIdMap.set(db.id, sb.id);
      bodyMap.push({ src: db.id, id: sb.id, defPos: vec(db.pos.x, db.pos.y), defAngle: db.angle, chassis });
    }
    // Orphaned bodies (their def source is gone) cascade away with their joints/constraints.
    // The rebuilt map is installed first: removals prune instance maps as they go, and the
    // record must never look empty mid-reconcile (pruneInstances would drop it).
    const oldBodyList = inst?.bodyMap ?? [];
    if (inst) inst.bodyMap = bodyMap;
    for (const e of oldBodyList) {
      if (!dBody.has(e.src) && this.getBody(e.id)) this.removeBody(e.id);
    }

    // --- joints ---------------------------------------------------------------
    const oldJointEntries = new Map((inst?.jointMap ?? []).map((e) => [e.src, e]));
    const jointIdMap = new Map<number, number>();
    const jointMap: InstanceMapEntry[] = [];
    for (const dj of d.joints) {
      const old = oldJointEntries.get(dj.id);
      let sj = old ? this.getJoint(old.id) : undefined;
      if (sj) {
        if (dj.bodyId === null) {
          // Free joints follow the def layout too (a joint the def detached from a
          // body is reset the same way).
          sj.bodyId = null;
          sj.local = xf(defJointWorld(dj));
        } else {
          const bid = bodyIdMap.get(dj.bodyId);
          if (bid === undefined) {
            this.removeJoint(sj.id);
            sj = undefined;
          } else {
            sj.bodyId = bid;
            sj.local = vec(dj.local.x, dj.local.y);
          }
        }
      }
      if (!sj) {
        if (dj.bodyId === null) {
          sj = { id: this.id(), bodyId: null, local: xf(defJointWorld(dj)) };
        } else {
          const bid = bodyIdMap.get(dj.bodyId);
          if (bid === undefined) continue;
          sj = { id: this.id(), bodyId: bid, local: vec(dj.local.x, dj.local.y) };
        }
        this.joints.push(sj);
      }
      jointIdMap.set(dj.id, sj.id);
      jointMap.push({ src: dj.id, id: sj.id });
    }
    const oldJointList = inst?.jointMap ?? [];
    if (inst) inst.jointMap = jointMap;
    for (const e of oldJointList) {
      if (!dJoint.has(e.src) && this.getJoint(e.id)) this.removeJoint(e.id);
    }

    // --- synthesized chassis anchors (one per joint-ground in the def) ---------
    const oldAnchors = new Map((inst?.anchorMap ?? []).map((e) => [e.src, e]));
    const anchorMap: InstanceMapEntry[] = [];
    const anchorIds = new Map<number, number>(); // def ground id → scene anchor joint id
    for (const g of anchorSrcs) {
      const at = xf(g.anchor);
      const old = oldAnchors.get(g.id);
      let aj = old ? this.getJoint(old.id) : undefined;
      if (aj && aj.bodyId === null) aj.local = vec(at.x, at.y);
      else aj = this.addFreeJoint(at);
      anchorMap.push({ src: g.id, id: aj.id });
      anchorIds.set(g.id, aj.id);
    }
    const oldAnchorList = inst?.anchorMap ?? [];
    if (inst) inst.anchorMap = anchorMap;
    for (const e of oldAnchorList) {
      if (!anchorIds.has(e.src) && this.getJoint(e.id)) this.removeJoint(e.id);
    }

    // --- constraints ------------------------------------------------------------
    // Two passes: structural first (pins / sliders — grounds convert to anchor pins),
    // then the powered constraints that reference sliders by id.
    const oldCons = new Map((inst?.constraintMap ?? []).map((e) => [e.src, e]));
    const constraintMap: InstanceMapEntry[] = [];
    const conIdMap = new Map<number, number>(); // def constraint id → scene constraint id
    const oldSceneCon = (src: number): Constraint | undefined => {
      const e = oldCons.get(src);
      return e ? this.constraints.find((x) => x.id === e.id) : undefined;
    };
    const keep = (src: number, id: number): void => {
      constraintMap.push({ src, id });
      conIdMap.set(src, id);
    };
    for (const dc of d.constraints) {
      if (dc.kind === "ground") {
        const aj = anchorIds.get(dc.id);
        if (aj === undefined) continue; // chassis membership — no expanded constraint
        const mj = jointIdMap.get(dc.joint);
        if (mj === undefined) continue;
        const sc = oldSceneCon(dc.id);
        if (sc && sc.kind === "pin") {
          sc.jointA = mj;
          sc.jointB = aj;
          keep(dc.id, sc.id);
        } else {
          keep(dc.id, this.addPin(mj, aj).id);
        }
      } else if (dc.kind === "pin") {
        const a = jointIdMap.get(dc.jointA);
        const b = jointIdMap.get(dc.jointB);
        if (a === undefined || b === undefined) continue;
        const sc = oldSceneCon(dc.id);
        if (sc && sc.kind === "pin") {
          sc.jointA = a;
          sc.jointB = b;
          // Rigidity is a design field — the definition is the reference.
          if (dc.rigid === true) sc.rigid = true;
          else delete sc.rigid;
          keep(dc.id, sc.id);
        } else {
          keep(dc.id, this.addPin(a, b, dc.rigid === true).id);
        }
      } else if (dc.kind === "slider") {
        const a = jointIdMap.get(dc.railA);
        const b = jointIdMap.get(dc.railB);
        if (a === undefined || b === undefined) continue;
        const riders = dc.riders
          .map((r) => jointIdMap.get(r))
          .filter((x): x is number => x !== undefined);
        // Pre-v17 def data has no `locked`; sanitized loads always do.
        const locked = (dc.locked ?? [])
          .map((r) => jointIdMap.get(r))
          .filter((x): x is number => x !== undefined && riders.includes(x));
        const sc = oldSceneCon(dc.id);
        if (sc && sc.kind === "slider") {
          sc.railA = a;
          sc.railB = b;
          sc.riders = riders;
          sc.locked = locked;
          keep(dc.id, sc.id);
        } else {
          // Created directly (not via addSlider): a def's world-fixed track means
          // "fixed to the chassis", so its free rail joints must NOT be auto-grounded
          // here — they're group-locked chassis points instead.
          const ns: SliderConstraint = { kind: "slider", id: this.id(), railA: a, railB: b, riders, locked };
          this.constraints.push(ns);
          keep(dc.id, ns.id);
        }
      }
    }
    for (const dc of d.constraints) {
      if (dc.kind === "linearActuator") {
        const rider = jointIdMap.get(dc.riderId);
        const slider = conIdMap.get(dc.sliderId);
        if (rider === undefined || slider === undefined) continue;
        const sc = oldSceneCon(dc.id);
        if (sc && sc.kind === "linearActuator") {
          sc.sliderId = slider;
          sc.riderId = rider;
          sc.speed = dc.speed;
          sc.profile = dc.profile;
          keep(dc.id, sc.id);
        } else {
          const na: LinearActuatorConstraint = {
            kind: "linearActuator",
            id: this.id(),
            sliderId: slider,
            riderId: rider,
            speed: dc.speed,
            profile: dc.profile,
          };
          this.constraints.push(na);
          keep(dc.id, na.id);
        }
      } else if (dc.kind === "motor") {
        const body = bodyIdMap.get(dc.bodyId);
        const pivot = jointIdMap.get(dc.pivotJointId);
        const crank = jointIdMap.get(dc.crankJointId);
        if (body === undefined || pivot === undefined || crank === undefined) continue;
        const sc = oldSceneCon(dc.id);
        if (sc && sc.kind === "motor") {
          sc.bodyId = body;
          sc.pivotJointId = pivot;
          sc.crankJointId = crank;
          sc.speed = dc.speed;
          keep(dc.id, sc.id);
        } else {
          const nm: MotorConstraint = {
            kind: "motor",
            id: this.id(),
            bodyId: body,
            pivotJointId: pivot,
            crankJointId: crank,
            speed: dc.speed,
          };
          this.constraints.push(nm);
          keep(dc.id, nm.id);
        }
      }
    }
    // Orphaned expanded constraints: anything previously mapped that wasn't kept.
    const keptConIds = new Set(constraintMap.map((e) => e.id));
    const oldConList = inst?.constraintMap ?? [];
    if (inst) inst.constraintMap = constraintMap;
    for (const e of oldConList) {
      if (!keptConIds.has(e.id) && this.constraints.some((x) => x.id === e.id)) {
        this.removeConstraint(e.id);
      }
    }

    // --- groups -----------------------------------------------------------------
    // The def's own groups (user groups; nested instances' chassis) are recreated as
    // plain groups; then the chassis group locks the grounded material together.
    const oldGroups = new Map((inst?.groupMap ?? []).map((e) => [e.src, e]));
    const groupMap: InstanceMapEntry[] = [];
    for (const dg of d.groups ?? []) {
      const old = oldGroups.get(dg.id);
      if (old) this.groups = this.groups.filter((g) => g.id !== old.id);
      const bids = dg.bodyIds.map((x) => bodyIdMap.get(x)).filter((x): x is number => x !== undefined);
      const jids = (dg.jointIds ?? []).map((x) => jointIdMap.get(x)).filter((x): x is number => x !== undefined);
      if (bids.length + jids.length >= 2) {
        const ng = this.addGroup(bids, jids);
        if (ng) groupMap.push({ src: dg.id, id: ng.id });
      }
    }
    const keptGroupIds = new Set(groupMap.map((e) => e.id));
    for (const e of inst?.groupMap ?? []) {
      if (!keptGroupIds.has(e.id)) this.groups = this.groups.filter((g) => g.id !== e.id);
    }
    if (inst?.groupId !== null && inst?.groupId !== undefined) {
      this.groups = this.groups.filter((g) => g.id !== inst.groupId);
    }
    const chassisB = [...chassisBodySrc]
      .map((x) => bodyIdMap.get(x))
      .filter((x): x is number => x !== undefined);
    const chassisJ = [
      ...[...chassisJointSrc].map((x) => jointIdMap.get(x)).filter((x): x is number => x !== undefined),
      ...anchorMap.map((e) => e.id),
    ];
    let groupId: number | null = null;
    if (chassisB.length + chassisJ.length >= 2) {
      groupId = this.addGroup(chassisB, chassisJ)?.id ?? null;
    }

    if (inst) {
      inst.bodyMap = bodyMap;
      inst.jointMap = jointMap;
      inst.constraintMap = constraintMap;
      inst.anchorMap = anchorMap;
      inst.groupMap = groupMap;
      inst.groupId = groupId;
      return inst;
    }
    return {
      id: this.id(),
      defId: def.id,
      bodyMap,
      jointMap,
      constraintMap,
      anchorMap,
      groupMap,
      groupId,
    };
  }

  /** Drop map entries whose elements are gone; an instance with nothing left dissolves. */
  private pruneInstances(): void {
    for (const inst of this.instances) {
      inst.bodyMap = inst.bodyMap.filter((e) => this.getBody(e.id));
      inst.jointMap = inst.jointMap.filter((e) => this.getJoint(e.id));
      inst.anchorMap = inst.anchorMap.filter((e) => this.getJoint(e.id));
      inst.constraintMap = inst.constraintMap.filter((e) =>
        this.constraints.some((c) => c.id === e.id)
      );
      inst.groupMap = inst.groupMap.filter((e) => this.groups.some((g) => g.id === e.id));
      if (inst.groupId !== null && !this.groups.some((g) => g.id === inst.groupId)) {
        inst.groupId = null;
      }
    }
    this.instances = this.instances.filter(
      (i) => i.bodyMap.length + i.jointMap.length + i.anchorMap.length > 0
    );
  }

  /** Remove a body along with its joints, pruning the constraints that used them. */
  removeBody(id: number): void {
    const removed = new Set(this.joints.filter((j) => j.bodyId === id).map((j) => j.id));
    this.patterns = this.patterns.filter((p) => p.bodyId !== id);
    this.bodies = this.bodies.filter((b) => b.id !== id);
    this.joints = this.joints.filter((j) => j.bodyId !== id);
    this.pruneConstraints(removed);
    this.pruneGroups();
    this.pruneInstances();
  }

  /** Remove a single joint, pruning the constraints (and group memberships) that used it. */
  removeJoint(id: number): void {
    // A pattern member is derived geometry: deleting one deletes the whole array (the
    // seed stays); deleting the seed dissolves the pattern (its members stay).
    const pj = this.patternOfJoint(id);
    if (pj?.role === "member") {
      this.removePattern(pj.pattern.id);
      return;
    }
    if (pj?.role === "seed") this.dissolvePattern(pj.pattern.id);
    this.joints = this.joints.filter((j) => j.id !== id);
    this.pruneConstraints(new Set([id]));
    this.pruneGroups();
    this.pruneInstances();
  }

  /**
   * Drop or trim constraints after some joints are removed: a pin/ground that uses
   * a gone joint is dropped; a slider is dropped if a *rail* joint is gone, but only
   * loses the affected *riders* otherwise (the rail itself survives). A second pass
   * drops any actuator whose slider was just removed (the actuator's rider survives
   * as a free joint, but the actuator itself is meaningless without its slider).
   */
  private pruneConstraints(removed: Set<number>): void {
    const trimmed = this.constraints
      .map((c) => pruneConstraint(c, removed))
      .filter((c): c is Constraint => c !== null);
    const sliderIds = new Set(trimmed.filter((c) => c.kind === "slider").map((c) => c.id));
    this.constraints = trimmed.filter(
      (c) => c.kind !== "linearActuator" || sliderIds.has(c.sliderId)
    );
    this.pruneMeasurements();
    this.pruneSketch();
  }

  /**
   * Remove a single constraint (e.g. a slider) by id, leaving its joints intact. Removing
   * a slider cascades to any actuator bound to it (the actuator's rider stays around as a
   * free joint); other constraint kinds have no cascade.
   */
  removeConstraint(id: number): void {
    const c = this.constraints.find((x) => x.id === id);
    this.constraints = this.constraints.filter((x) => x.id !== id);
    if (c && c.kind === "slider") {
      this.constraints = this.constraints.filter(
        (x) => !(x.kind === "linearActuator" && x.sliderId === id)
      );
    }
    this.pruneMeasurements();
    this.pruneSketch();
  }

  /** Clear the current context's contents (component definitions are kept unless asked). */
  clear(dropComponents = true): void {
    this.bodies = [];
    this.joints = [];
    this.constraints = [];
    this.measurements = [];
    this.sketch = [];
    this.groups = [];
    this.guides = [];
    this.instances = [];
    this.patterns = [];
    if (dropComponents) this.components = [];
    this.nextId = 1;
  }

  /** Plain-data snapshot of the scene (current context + the document's component
   *  definitions); safe to JSON.stringify. */
  serialize(): SceneData {
    return {
      ...this.serializeContext(),
      components: this.components,
    };
  }

  /** Snapshot of the current editing context only — no component definitions. Used when
   *  the app switches between the root assembly and a component definition's sub-scene. */
  serializeContext(): SceneData {
    return {
      version: FORMAT_VERSION,
      unit: this.unit,
      bodies: this.bodies,
      joints: this.joints,
      constraints: this.constraints,
      measurements: this.measurements,
      sketch: this.sketch,
      groups: this.groups,
      guides: this.guides,
      instances: this.instances,
      patterns: this.patterns,
    };
  }

  /** Replace the scene's contents from a snapshot, including the document's component
   *  definitions. Throws on malformed data. */
  load(data: SceneData): void {
    this.loadContext(data);
    // Component definitions arrived in v14; older files simply have none. Deep-clone so
    // the loaded document is independent of the parsed JSON object.
    this.components = Array.isArray(data.components)
      ? (JSON.parse(JSON.stringify(data.components)) as ComponentDef[]).filter(
          (c) => c && typeof c.id === "number" && c.data && Array.isArray(c.data.bodies)
        )
      : [];
  }

  /** Replace the current context's contents from a snapshot, keeping the document's
   *  component definitions untouched. Throws on malformed data. */
  loadContext(data: SceneData): void {
    if (
      !data ||
      !Array.isArray(data.bodies) ||
      !Array.isArray(data.joints) ||
      !Array.isArray(data.constraints)
    ) {
      throw new Error("Not a valid Disjointed file.");
    }
    // The working unit arrived in v12; older files simply work in the default.
    this.unit = data.unit && data.unit in UNIT_TO_MM ? data.unit : DEFAULT_UNIT;
    // Deep-clone so loaded data is independent of the parsed JSON object.
    this.bodies = data.bodies.map((b) => {
      const local = b.local.map((p) => vec(p.x, p.y));
      // Older files (< v5) have no control polygon: treat the saved polygon as a sharp control.
      const src = b as Body & { controlLocal?: Vec2[]; radius?: number; radii?: (number | null)[]; round?: RoundMode; grounded?: boolean };
      const controlLocal = src.controlLocal ? src.controlLocal.map((p) => vec(p.x, p.y)) : local.map((p) => vec(p.x, p.y));
      // Per-corner radius overrides arrived in v15; older files simply have none.
      // Sanitize: entries must be non-negative numbers or null, one per control vertex.
      const sanitizeRadii = (raw: unknown, n: number): (number | null)[] => {
        let rr = Array.isArray(raw)
          ? raw.slice(0, n).map((r) => (typeof r === "number" && r >= 0 ? r : null))
          : [];
        while (rr.length > 0 && rr.length < n) rr.push(null);
        if (!rr.some((r) => r !== null)) rr = [];
        return rr;
      };
      const radii = sanitizeRadii(src.radii, controlLocal.length);
      // Editable hole shapes arrived in v16. Older files (v13–v15) carry only baked
      // `holesLocal` loops — each becomes a radius-0 hole control polygon, so legacy
      // holes load as editable ones with exactly the same outline.
      let holes: BodyHole[] = [];
      if (Array.isArray(src.holes)) {
        holes = src.holes
          .filter((h) => h && Array.isArray(h.controlLocal) && h.controlLocal.length >= 1)
          .map((h) => {
            const out: BodyHole = {
              controlLocal: h.controlLocal.map((p) => vec(p.x, p.y)),
              radius: typeof h.radius === "number" && h.radius >= 0 ? h.radius : 0,
            };
            if (h.round === "offset") out.round = "offset";
            const hr = sanitizeRadii(h.radii, out.controlLocal.length);
            if (hr.length) out.radii = hr;
            return out;
          });
      } else if (Array.isArray(src.holesLocal)) {
        holes = src.holesLocal
          .filter((loop) => Array.isArray(loop) && loop.length >= 3)
          .map((loop) => ({ controlLocal: loop.map((p) => vec(p.x, p.y)), radius: 0 }));
      }
      // Grounded bodies arrived in v10; older files simply have none.
      const out: Body = { ...b, pos: vec(b.pos.x, b.pos.y), local, controlLocal, radius: src.radius ?? 0, round: src.round ?? "fillet", grounded: src.grounded ?? false };
      if (radii.length) out.radii = radii;
      else delete out.radii;
      if (holes.length) {
        out.holes = holes;
        // Re-derive the sampled loops from the (sanitized) controls rather than trusting
        // the file's copy — they can never disagree with the shapes this way.
        out.holesLocal = holes.map((h) => deriveHoleOutline(h.controlLocal, h));
      } else {
        delete out.holes;
        delete out.holesLocal;
      }
      return out;
    });
    this.joints = data.joints.map((j) => ({ ...j, local: vec(j.local.x, j.local.y) }));
    // Patterns arrived in v19; older files simply have none. Records are validated now
    // (measurement / sketch refs below may name a pattern axis); members are re-derived
    // at the end, once the id counter is set (a sync may mint new member joints).
    this.patterns = Array.isArray(data.patterns) ? data.patterns.flatMap((p) => this.sanitizePattern(p)) : [];
    // Sliders: drop the legacy origin+dir form (no railA); migrate the earlier
    // single-`slider` rider field to the `riders` array; normalize riders to an array.
    // `locked` (orientation-locked riders) arrived in v17 — older files have none, and
    // a hand-edited lock on a non-rider is dropped (the invariant is locked ⊆ riders).
    this.constraints = data.constraints
      .filter((c) => c.kind !== "slider" || (c as { railA?: number }).railA !== undefined)
      .map((c) => {
        if (c.kind === "pin") {
          // Welds (`rigid`) arrived in v18 — older files simply have revolute pins.
          // Sanitize: the flag is present (true) or absent, never any other value.
          const p: PinConstraint = { kind: "pin", id: c.id, jointA: c.jointA, jointB: c.jointB };
          if ((c as PinConstraint).rigid === true) p.rigid = true;
          return p;
        }
        if (c.kind !== "slider") return { ...c };
        const s = c as SliderConstraint & { slider?: number };
        const riders = Array.isArray(s.riders)
          ? s.riders.slice()
          : typeof s.slider === "number"
          ? [s.slider]
          : [];
        const locked = Array.isArray(s.locked) ? s.locked.filter((r) => riders.includes(r)) : [];
        return { kind: "slider", id: s.id, railA: s.railA, railB: s.railB, riders, locked };
      });
    // Measurements arrived in v7; older files simply have none.
    this.measurements = Array.isArray(data.measurements)
      ? data.measurements.map((m) => ({
          ...m,
          refA: cloneMeasureRef(m.refA),
          refB: cloneMeasureRef(m.refB),
          labelOffset: vec(m.labelOffset.x, m.labelOffset.y),
        }))
      : [];
    // Sketch constraints arrived in v8; older files simply have none.
    this.sketch = Array.isArray(data.sketch)
      ? data.sketch.map((c) => ({
          ...c,
          refA: cloneMeasureRef(c.refA),
          refB: c.refB ? cloneMeasureRef(c.refB) : null,
        }))
      : [];
    // Permanent groups arrived in v9 (free-joint members in v14); older files have none.
    this.groups = Array.isArray(data.groups)
      ? data.groups.map((g) => ({
          id: g.id,
          bodyIds: Array.isArray(g.bodyIds) ? g.bodyIds.slice() : [],
          jointIds: Array.isArray(g.jointIds) ? g.jointIds.slice() : [],
        }))
      : [];
    this.pruneGroups(); // drop stale member ids / degenerate groups from hand-edited files
    // Component instances arrived in v14; older files simply have none.
    this.instances = Array.isArray(data.instances)
      ? (JSON.parse(JSON.stringify(data.instances)) as ComponentInstance[]).filter(
          (i) => i && typeof i.id === "number" && typeof i.defId === "number"
        )
      : [];
    for (const inst of this.instances) {
      inst.bodyMap = Array.isArray(inst.bodyMap) ? inst.bodyMap : [];
      inst.jointMap = Array.isArray(inst.jointMap) ? inst.jointMap : [];
      inst.constraintMap = Array.isArray(inst.constraintMap) ? inst.constraintMap : [];
      inst.anchorMap = Array.isArray(inst.anchorMap) ? inst.anchorMap : [];
      inst.groupMap = Array.isArray(inst.groupMap) ? inst.groupMap : [];
      inst.groupId = typeof inst.groupId === "number" ? inst.groupId : null;
    }
    this.pruneInstances();
    // Construction guidelines arrived in v11; older files simply have none.
    this.guides = Array.isArray(data.guides)
      ? data.guides
          .map((g) => ({ id: g.id, a: vec(g.a.x, g.a.y), b: vec(g.b.x, g.b.y) }))
          .filter((g) => dist(g.a, g.b) >= Scene.GUIDE_MIN_SPAN)
      : [];
    // Driving-dimension sides: sanitize hand-edited values, back-fill files saved
    // before the side existed (captured from the loaded — satisfied — geometry), and
    // drop the field from driven dimensions. Runs last so every ref kind resolves.
    for (const m of this.measurements) {
      if (m.side !== 1 && m.side !== -1) delete m.side;
      if (m.driving === true && m.target !== undefined) {
        if (m.side === undefined) this.captureMeasurementSide(m);
      } else {
        delete m.side;
      }
    }
    const ids = [
      ...this.bodies.map((b) => b.id),
      ...this.joints.map((j) => j.id),
      ...this.constraints.map((c) => c.id),
      ...this.measurements.map((m) => m.id),
      ...this.sketch.map((c) => c.id),
      ...this.groups.map((g) => g.id),
      ...this.guides.map((g) => g.id),
      ...this.instances.map((i) => i.id),
      ...(Array.isArray(data.patterns) ? data.patterns.map((p) => (p && typeof p.id === "number" ? p.id : 0)) : []),
    ];
    this.nextId = (ids.length ? Math.max(...ids) : 0) + 1;
    // Re-derive every pattern's members from its seed (repairs a hand-edited count or a
    // stale member list); pattern records were validated above.
    for (const p of this.patterns) this.syncPattern(p);
  }

  /** A loaded pattern record validated + deep-cloned, or nothing when it can't be trusted. */
  private sanitizePattern(raw: unknown): Pattern[] {
    const p = raw as Partial<Pattern> | null;
    if (!p || typeof p.id !== "number" || typeof p.bodyId !== "number" || !p.seed || !p.layout) return [];
    const body = this.getBody(p.bodyId);
    if (!body) return [];
    const isVec = (v: unknown): v is Vec2 =>
      !!v && typeof (v as Vec2).x === "number" && typeof (v as Vec2).y === "number" &&
      Number.isFinite((v as Vec2).x) && Number.isFinite((v as Vec2).y);
    let seed: Pattern["seed"];
    if (p.seed.kind === "hole") {
      if (typeof p.seed.hole !== "number" || !body.holes?.[p.seed.hole]) return [];
      seed = { kind: "hole", hole: p.seed.hole };
    } else if (p.seed.kind === "joint") {
      if (typeof p.seed.jointId !== "number" || this.getJoint(p.seed.jointId)?.bodyId !== body.id) return [];
      seed = { kind: "joint", jointId: p.seed.jointId };
    } else return [];
    let layout: PatternLayout;
    const l = p.layout as Partial<Extract<PatternLayout, { kind: "linear" }>> & Partial<Extract<PatternLayout, { kind: "circular" }>>;
    if (l.kind === "linear") {
      if (!Array.isArray(l.axes)) return [];
      const axes = l.axes
        .filter((a) => a && typeof a.count === "number" && isVec(a.step) && len(a.step) >= 1e-6)
        .slice(0, 2)
        .map((a) => ({ count: this.clampCount(a.count), step: vec(a.step.x, a.step.y) }));
      if (axes.length === 0) return [];
      layout = { kind: "linear", axes };
    } else if (l.kind === "circular") {
      if (!isVec(l.centre) || typeof l.count !== "number" || len(l.centre) < 1e-6) return [];
      layout = { kind: "circular", centre: vec(l.centre.x, l.centre.y), count: this.clampCount(l.count), rotate: l.rotate !== false };
      if (typeof l.angleStep === "number" && Number.isFinite(l.angleStep) && Math.abs(l.angleStep) >= 1e-9) layout.angleStep = l.angleStep;
    } else return [];
    const members = (Array.isArray(p.members) ? p.members : []).filter(
      (m): m is number =>
        typeof m === "number" &&
        (seed.kind === "hole"
          ? m !== seed.hole && !!body.holes?.[m]
          : m !== seed.jointId && this.getJoint(m)?.bodyId === body.id)
    );
    return [{ id: p.id, bodyId: body.id, seed, layout, members: [...new Set(members)] }];
  }

  /** Snapshot of every body's pose, for save/restore around a simulation run. */
  snapshotPoses(): Map<number, { pos: Vec2; angle: number }> {
    const m = new Map<number, { pos: Vec2; angle: number }>();
    for (const b of this.bodies) m.set(b.id, { pos: vec(b.pos.x, b.pos.y), angle: b.angle });
    return m;
  }

  restorePoses(snapshot: Map<number, { pos: Vec2; angle: number }>): void {
    for (const b of this.bodies) {
      const s = snapshot.get(b.id);
      if (s) {
        b.pos = vec(s.pos.x, s.pos.y);
        b.angle = s.angle;
      }
    }
  }
}

/**
 * Given a set of removed joint ids, return the constraint to keep — possibly a
 * trimmed copy — or `null` to drop it. Pins/grounds drop if any referenced joint is
 * gone; sliders drop only if a rail joint is gone, otherwise they shed dead riders.
 * A linear actuator drops if its rider is gone (the slider may have been trimmed but
 * the actuator's rider is the joint we just removed); a motor drops if either of its
 * joints is gone.
 */
function pruneConstraint(c: Constraint, removed: Set<number>): Constraint | null {
  if (c.kind === "pin") return removed.has(c.jointA) || removed.has(c.jointB) ? null : c;
  if (c.kind === "ground") return removed.has(c.joint) ? null : c;
  if (c.kind === "linearActuator") return removed.has(c.riderId) ? null : c;
  if (c.kind === "motor") {
    return removed.has(c.pivotJointId) || removed.has(c.crankJointId) ? null : c;
  }
  if (removed.has(c.railA) || removed.has(c.railB)) return null;
  const riders = c.riders.filter((r) => !removed.has(r));
  const locked = c.locked.filter((r) => !removed.has(r));
  return riders.length === c.riders.length && locked.length === c.locked.length
    ? c
    : { ...c, riders, locked };
}

// --- component cascade helpers ----------------------------------------------

/**
 * Re-expand a stored context snapshot against the (updated) component definitions:
 * every instance of a changed definition is reconciled in a scratch scene and the
 * refreshed snapshot returned. Used for contexts that aren't currently loaded (the
 * root assembly while a definition is being edited, ancestor definitions on the
 * editing stack).
 */
export function reexpandData(
  data: SceneData,
  components: ComponentDef[],
  changed: ReadonlySet<number>
): SceneData {
  if (!(data.instances ?? []).some((i) => changed.has(i.defId))) return data;
  const tmp = new Scene();
  tmp.loadContext(data);
  tmp.components = components;
  tmp.reexpandInstances(changed);
  return tmp.serializeContext();
}

/**
 * Propagate a definition change through the definition DAG: any definition whose data
 * instantiates a changed definition is re-expanded (its stored data updated in place)
 * and marked changed itself, until a fixpoint. Returns the full set of changed def ids —
 * the caller then re-expands the live/root contexts against that set.
 */
export function cascadeComponentChange(
  components: ComponentDef[],
  changedIds: Iterable<number>
): Set<number> {
  const changed = new Set(changedIds);
  let progress = true;
  while (progress) {
    progress = false;
    for (const def of components) {
      if (changed.has(def.id)) continue;
      if (!(def.data.instances ?? []).some((i) => changed.has(i.defId))) continue;
      def.data = reexpandData(def.data, components, changed);
      changed.add(def.id);
      progress = true;
    }
  }
  return changed;
}

// --- measurement geometry --------------------------------------------------

function cloneMeasureRef(r: MeasureRef): MeasureRef {
  return r.kind === "bodyPoint" ? { ...r, local: clone(r.local) } : { ...r };
}

/** Whether two references name the same element (bodyPoint refs never match). */
export function sameMeasureRef(a: MeasureRef, b: MeasureRef): boolean {
  if (a.kind !== b.kind) return false;
  switch (a.kind) {
    case "joint":
      return a.jointId === (b as { jointId: number }).jointId;
    case "vertex":
    case "edge":
      return (
        a.bodyId === (b as { bodyId: number }).bodyId &&
        a.index === (b as { index: number }).index &&
        (a.hole ?? null) === ((b as { hole?: number }).hole ?? null)
      );
    case "rail":
      return a.sliderId === (b as { sliderId: number }).sliderId;
    case "guidePoint":
      return (
        a.guideId === (b as { guideId: number }).guideId &&
        a.which === (b as { which: "a" | "b" }).which
      );
    case "guideLine":
      return a.guideId === (b as { guideId: number }).guideId;
    case "patternAxis":
      return a.patternId === (b as { patternId: number }).patternId && a.axis === (b as { axis: number }).axis;
    default:
      return false;
  }
}

/** Representative centre of a resolved reference (the label anchors to the midpoint of the two). */
function refCenter(r: ResolvedMeasureRef): Vec2 {
  return r.kind === "point" ? r.p : scale(add(r.a, r.b), 0.5);
}

/**
 * CAD-style axis pick for a point–point dimension from where the label was placed:
 * within the pair's x-range but outside its y-range (above/below) → horizontal distance;
 * within the y-range but outside the x-range (beside) → vertical; anywhere else
 * (diagonal zones, or between the points) → direct.
 */
export function measureAxisForPlacement(p: Vec2, q: Vec2, label: Vec2): MeasureAxis {
  const inX = label.x >= Math.min(p.x, q.x) && label.x <= Math.max(p.x, q.x);
  const inY = label.y >= Math.min(p.y, q.y) && label.y <= Math.max(p.y, q.y);
  if (inX && !inY) return "h";
  if (inY && !inX) return "v";
  return "direct";
}

/** Collect non-degenerate extension segments. */
function pushExt(ext: { a: Vec2; b: Vec2 }[], a: Vec2, b: Vec2): void {
  if (dist(a, b) > 1e-6) ext.push({ a, b });
}

function pointPointInfo(
  id: number,
  p: Vec2,
  q: Vec2,
  axis: MeasureAxis,
  labelPos: Vec2
): MeasureInfo {
  const ext: { a: Vec2; b: Vec2 }[] = [];
  if (axis === "h") {
    // Horizontal distance: the dimension line runs at the label's height.
    const d1 = vec(p.x, labelPos.y);
    const d2 = vec(q.x, labelPos.y);
    pushExt(ext, p, d1);
    pushExt(ext, q, d2);
    return { id, kind: "distance", value: Math.abs(q.x - p.x), labelPos, dim: { a: d1, b: d2 }, ext };
  }
  if (axis === "v") {
    const d1 = vec(labelPos.x, p.y);
    const d2 = vec(labelPos.x, q.y);
    pushExt(ext, p, d1);
    pushExt(ext, q, d2);
    return { id, kind: "distance", value: Math.abs(q.y - p.y), labelPos, dim: { a: d1, b: d2 }, ext };
  }
  // Direct: the dimension line is parallel to p–q, offset sideways to pass by the label.
  const d = sub(q, p);
  const l = len(d);
  let off = vec(0, 0);
  if (l > 1e-9) {
    const u = scale(d, 1 / l);
    const w = sub(labelPos, p);
    off = sub(w, scale(u, dot(w, u))); // component of the label offset perpendicular to p–q
  }
  const d1 = add(p, off);
  const d2 = add(q, off);
  pushExt(ext, p, d1);
  pushExt(ext, q, d2);
  return { id, kind: "distance", value: l, labelPos, dim: { a: d1, b: d2 }, ext };
}

/**
 * Diameter of a disk: the dimension line runs through the centre towards the label
 * (rim to rim, arrowed at both ends), with a dashed leader from the rim out to a label
 * placed outside the circle. Value = 2·r.
 */
function diameterInfo(id: number, c: Vec2, r: number, labelPos: Vec2): MeasureInfo {
  const w = sub(labelPos, c);
  const l = len(w);
  const u = l > 1e-9 ? scale(w, 1 / l) : vec(1, 0);
  const near = add(c, scale(u, r));
  const far = sub(c, scale(u, r));
  const ext: { a: Vec2; b: Vec2 }[] = [];
  if (l > r) pushExt(ext, near, labelPos);
  return { id, kind: "distance", value: 2 * r, labelPos, circle: { c, r }, dim: { a: far, b: near }, ext };
}

function pointLineInfo(
  id: number,
  p: Vec2,
  line: { a: Vec2; b: Vec2 },
  labelPos: Vec2
): MeasureInfo | null {
  const d = sub(line.b, line.a);
  const l = len(d);
  if (l < 1e-9) return null; // degenerate line (e.g. a collapsed rail) — nothing to measure
  const u = scale(d, 1 / l);
  const tp = dot(sub(p, line.a), u);
  const footP = add(line.a, scale(u, tp)); // perpendicular foot of the point on the *infinite* line
  const across = sub(p, footP); // the measured offset, perpendicular to the line
  // The dimension line is dropped perpendicular to the reference line *where the label
  // sits* (like the parallel-lines case), so it follows the label as it slides along.
  const t = dot(sub(labelPos, line.a), u);
  const d1 = add(line.a, scale(u, t));
  const d2 = add(d1, across);
  const ext: { a: Vec2; b: Vec2 }[] = [];
  pushExt(ext, p, d2); // from the point along to the dimension line's far end
  if (t < 0) pushExt(ext, line.a, d1);
  else if (t > l) pushExt(ext, line.b, d1);
  return { id, kind: "distance", value: len(across), labelPos, dim: { a: d2, b: d1 }, ext };
}

function lineLineInfo(
  id: number,
  l1: { a: Vec2; b: Vec2 },
  l2: { a: Vec2; b: Vec2 },
  labelPos: Vec2
): MeasureInfo | null {
  const d1 = sub(l1.b, l1.a);
  const d2 = sub(l2.b, l2.a);
  const len1 = len(d1);
  const len2 = len(d2);
  if (len1 < 1e-9 || len2 < 1e-9) return null;
  const u1 = scale(d1, 1 / len1);
  const u2 = scale(d2, 1 / len2);
  const lineAngle = Math.acos(Math.min(1, Math.abs(dot(u1, u2)))); // between *lines*: 0..π/2
  if (lineAngle < MEASURE_PARALLEL_TOL) {
    // (Near-)parallel: perpendicular distance, measured where the label sits so the
    // dimension line stays local to it (and continuous as the pair moves in sim).
    const t1 = dot(sub(labelPos, l1.a), u1);
    const f1 = add(l1.a, scale(u1, t1));
    const t2 = dot(sub(f1, l2.a), u2);
    const f2 = add(l2.a, scale(u2, t2));
    const ext: { a: Vec2; b: Vec2 }[] = [];
    if (t1 < 0) pushExt(ext, l1.a, f1);
    else if (t1 > len1) pushExt(ext, l1.b, f1);
    if (t2 < 0) pushExt(ext, l2.a, f2);
    else if (t2 > len2) pushExt(ext, l2.b, f2);
    return { id, kind: "distance", value: dist(f1, f2), labelPos, dim: { a: f1, b: f2 }, ext };
  }
  // Not parallel: the angle of whichever sector the label sits in (of the four the two
  // infinite lines cut the plane into), so placing/dragging the label picks θ vs 180−θ.
  const denom = cross(u1, u2);
  const t = cross(sub(l2.a, l1.a), u2) / denom;
  const v = add(l1.a, scale(u1, t)); // intersection of the infinite lines
  let w = sub(labelPos, v);
  if (len(w) < 1e-9) w = add(u1, u2); // label exactly on the vertex — fall back to a bisector
  const tau = 2 * Math.PI;
  const norm = (a: number) => ((a % tau) + tau) % tau;
  const wa = norm(Math.atan2(w.y, w.x));
  const rays = [
    norm(Math.atan2(u1.y, u1.x)),
    norm(Math.atan2(u1.y, u1.x) + Math.PI),
    norm(Math.atan2(u2.y, u2.x)),
    norm(Math.atan2(u2.y, u2.x) + Math.PI),
  ].sort((a, b) => a - b);
  // Find the pair of adjacent rays (cyclically) that bound the label's direction.
  let a0 = rays[3] - tau;
  let a1 = rays[0];
  for (let i = 0; i < 3; i++) {
    if (wa >= rays[i] && wa < rays[i + 1]) {
      a0 = rays[i];
      a1 = rays[i + 1];
    }
  }
  if (wa >= rays[3]) {
    a0 = rays[3];
    a1 = rays[0] + tau;
  }
  const sweep = a1 - a0;
  const r = len(w);
  const ext: { a: Vec2; b: Vec2 }[] = [];
  pushExt(ext, v, add(v, vec(r * Math.cos(a0), r * Math.sin(a0))));
  pushExt(ext, v, add(v, vec(r * Math.cos(a1), r * Math.sin(a1))));
  return {
    id,
    kind: "angle",
    value: (sweep * 180) / Math.PI,
    labelPos,
    arc: { c: v, r, a0, sweep },
    ext,
  };
}
