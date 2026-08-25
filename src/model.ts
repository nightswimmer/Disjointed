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
  distToSegment,
  filletPolygon,
  roundedConvexBody,
  polygonCentroid,
  polygonArea,
  polygonInertiaAboutCentroid,
  pointInPolygon,
  closestPointOnPolygon,
} from "./geometry";

/**
 * A joint exactly coincident with a body control vertex is "stuck" to it — they move
 * together (how joint-built bodies keep their joints and nodes linked). Coincidence is
 * exact up to float error from frame transforms, so the tolerance can be tiny.
 */
export const VERTEX_LINK_EPS = 1e-6;

/** How a body's `radius` shapes it: round the corners in place, or offset the hull outward. */
export type RoundMode = "fillet" | "offset";

export interface Body {
  id: number;
  /** Editable control polygon (the corners), relative to the centroid, in the local frame. */
  controlLocal: Vec2[];
  /** Corner radius (fillet) or outward margin (offset) applied to the control polygon. 0 = sharp. */
  radius: number;
  round: RoundMode;
  /** Derived render/physics polygon, relative to the centroid — rebuilt from control + radius. */
  local: Vec2[];
  /** World position of the centroid (the body's local origin). */
  pos: Vec2;
  angle: number;
  invMass: number;
  invInertia: number;
  color: string;
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

/** Two joints (on different bodies) share a world position; free relative rotation. */
export interface PinConstraint {
  kind: "pin";
  id: number;
  jointA: number;
  jointB: number;
}

/** A joint is locked to a fixed world point; the body may rotate about it. */
export interface GroundConstraint {
  kind: "ground";
  id: number;
  joint: number;
  anchor: Vec2;
}

/**
 * A slider / prismatic rail: the segment between `railA` and `railB` — two joints
 * on one body. Any joint in `riders` (on other bodies) is confined to that segment
 * and slides along it. The rail moves with its body, so it couples the rail's body
 * to each rider's body. (For a world-fixed track, put the rail joints on a grounded
 * body.) A rail with no riders is just a (selectable, deletable) guide.
 */
export interface SliderConstraint {
  kind: "slider";
  id: number;
  railA: number;
  railB: number;
  riders: number[];
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

// --- measurements ---------------------------------------------------------

/** Which mode a measurement belongs to — draw and sim each keep their own set. */
export type MeasureMode = "draw" | "sim";

/**
 * For a point–point measurement, which distance the label placement selected:
 * `"h"` horizontal (|Δx|), `"v"` vertical (|Δy|), `"direct"` straight-line.
 */
export type MeasureAxis = "direct" | "h" | "v";

/**
 * A measurement reference — a point or a line, anchored to scene *elements* (never to
 * bare coordinates), so its world geometry is re-resolved every frame and the value
 * tracks the mechanism as it moves in simulation.
 */
export type MeasureRef =
  | { kind: "joint"; jointId: number } // point: a joint
  | { kind: "vertex"; bodyId: number; index: number } // point: a body control vertex
  | { kind: "bodyPoint"; bodyId: number; local: Vec2 } // point: fixed in a body's frame
  | { kind: "rail"; sliderId: number } // line: a slider rail
  | { kind: "edge"; bodyId: number; index: number }; // line: control edge index → index+1

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
}

// --- sketch constraints -----------------------------------------------------

/**
 * Kinds of CAD-style sketch constraints (draw mode only):
 * `coincident` — two points share a position; `horizontal`/`vertical` — a line (or a
 * point pair) is axis-aligned; `parallel`/`perpendicular` — two lines' directions;
 * `equal` — two lines have equal length.
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
  | { kind: "line"; a: Vec2; b: Vec2 };

/** Everything needed to display a measurement this frame (value + drawing geometry). */
export interface MeasureInfo {
  id: number;
  kind: "distance" | "angle";
  /** World units for a distance; degrees for an angle. */
  value: number;
  /** True when the source dimension is driving (drawn without the CAD parentheses). */
  driving?: boolean;
  labelPos: Vec2;
  /** Arrowed dimension segment (distance only). */
  dim?: { a: Vec2; b: Vec2 };
  /** Dashed extension / leader segments. */
  ext: { a: Vec2; b: Vec2 }[];
  /** Angle arc (angle only): centre, radius, start angle, positive CCW sweep. */
  arc?: { c: Vec2; r: number; a0: number; sweep: number };
}

/** Serializable snapshot of an entire scene (for save / load / autosave). */
export interface SceneData {
  version: number;
  bodies: Body[];
  joints: Joint[];
  constraints: Constraint[];
  /** Draw-mode and sim-mode measurements together (each carries its `mode`). Absent pre-v7. */
  measurements?: Measurement[];
  /** Draw-mode sketch constraints. Absent pre-v8. */
  sketch?: SketchConstraint[];
}

/**
 * A self-contained snapshot of a body for copy/paste: its control polygon, the joints
 * attached to it, and the constraints that reference *only* those joints (grounds,
 * fully-internal sliders, intra-body pins). Everything is stored in world coordinates
 * relative to the original centroid; pasting translates the whole fragment so the
 * centroid lands at the drop point. `tmp` ids are the original joint ids, remapped to
 * fresh joints on paste. Cross-body pins can't be reproduced and are dropped.
 */
export interface BodyClip {
  controlWorld: Vec2[];
  radius: number;
  round: RoundMode;
  color: string;
  centroid: Vec2;
  joints: { tmp: number; world: Vec2 }[];
  grounds: { joint: number; anchor: Vec2 }[];
  sliders: { railA: number; railB: number; riders: number[] }[];
  pins: { a: number; b: number }[];
}

const FORMAT_VERSION = 8;

/** Below this angle two measured lines count as parallel: show their distance, not the angle. */
const MEASURE_PARALLEL_TOL = (0.5 * Math.PI) / 180;

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
  private nextId = 1;

  private id(): number {
    return this.nextId++;
  }

  /**
   * Create a body from a control polygon (world coords). `radius` rounds it: `fillet`
   * rounds the corners in place (keeps concavity); `offset` grows the convex hull outward.
   */
  addBody(worldVerts: Vec2[], radius = 0, round: RoundMode = "fillet"): Body {
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
    };
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
    const finalWorld =
      body.round === "offset"
        ? roundedConvexBody(ctrlWorld, body.radius)
        : filletPolygon(ctrlWorld, body.radius);
    const centroid = polygonCentroid(finalWorld);
    const attached = this.joints.filter((j) => j.bodyId === body.id);
    const jointWorlds = attached.map((j) => this.jointWorld(j));
    body.pos = centroid;
    body.local = finalWorld.map((p) => rotate(sub(p, centroid), -body.angle));
    body.controlLocal = ctrlWorld.map((p) => rotate(sub(p, centroid), -body.angle));
    body.invMass = 1 / Math.max(Math.abs(polygonArea(finalWorld)), 1);
    body.invInertia = 1 / Math.max(polygonInertiaAboutCentroid(finalWorld, centroid), 1);
    attached.forEach((j, i) => {
      j.local = rotate(sub(jointWorlds[i], centroid), -body.angle);
    });
  }

  /**
   * Move a control vertex of a body by a world-space delta, then rebuild its shape.
   * A joint of this body sitting exactly on the vertex is *stuck* to it and carried
   * along (a body built from joints keeps its joints and control nodes together);
   * all other attached joints stay anchored in world space as usual.
   */
  moveBodyVertex(bodyId: number, index: number, delta: Vec2): void {
    const body = this.getBody(bodyId);
    if (!body || index < 0 || index >= body.controlLocal.length) return;
    const vw = add(body.pos, rotate(body.controlLocal[index], body.angle));
    const linked = this.joints.filter(
      (j) => j.bodyId === bodyId && dist(this.jointWorld(j), vw) < VERTEX_LINK_EPS
    );
    body.controlLocal[index] = add(body.controlLocal[index], rotate(delta, -body.angle));
    this.rebuildBody(body); // keeps every joint anchored...
    for (const j of linked) this.shiftJoint(j, delta); // ...then the stuck ones follow
  }

  /**
   * Insert a new control vertex at `index` (world position → local), then rebuild.
   * Used to add a node on a polygon edge: pass the index it should occupy (i.e. the
   * later endpoint of the clicked edge).
   */
  insertBodyVertex(bodyId: number, index: number, worldPos: Vec2): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    const clamped = Math.min(Math.max(index, 0), body.controlLocal.length);
    body.controlLocal.splice(clamped, 0, rotate(sub(worldPos, body.pos), -body.angle));
    this.shiftMeasureIndices(bodyId, clamped, 1);
    this.rebuildBody(body);
  }

  /**
   * Remove a control vertex, then rebuild. No-op if it would leave fewer than 3
   * vertices (the minimum to define a polygon) or the index is out of range.
   */
  removeBodyVertex(bodyId: number, index: number): void {
    const body = this.getBody(bodyId);
    if (!body || body.controlLocal.length <= 3) return;
    if (index < 0 || index >= body.controlLocal.length) return;
    body.controlLocal.splice(index, 1);
    this.shiftMeasureIndices(bodyId, index, -1);
    this.rebuildBody(body);
  }

  /** Set a body's corner radius / margin (clamped ≥ 0), then rebuild its shape. */
  setBodyRadius(bodyId: number, radius: number): void {
    const body = this.getBody(bodyId);
    if (!body) return;
    body.radius = Math.max(0, radius);
    this.rebuildBody(body);
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
      } else if (j.bodyId === null && !grounded) {
        // A loose free joint, or a free slider rider: absorb it. It now belongs to the new
        // body (angle 0 at creation); if it was a rider it stays one (rider ids are kept).
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

  addPin(jointA: number, jointB: number): PinConstraint {
    const c: PinConstraint = { kind: "pin", id: this.id(), jointA, jointB };
    this.constraints.push(c);
    return c;
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
        !this.constraints.some((c) => c.kind === "ground" && c.joint === id)
      ) {
        this.addGround(id, this.jointWorld(j));
      }
    }
    const c: SliderConstraint = { kind: "slider", id: this.id(), railA, railB, riders: [] };
    this.constraints.push(c);
    return c;
  }

  /** Attach a joint as a rider of an existing slider (confined to its rail segment). */
  attachSliderRider(sliderId: number, jointId: number): void {
    const c = this.constraints.find((x) => x.id === sliderId && x.kind === "slider") as
      | SliderConstraint
      | undefined;
    if (c && !c.riders.includes(jointId)) c.riders.push(jointId);
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
    const axis =
      a.kind === "point" && b.kind === "point"
        ? measureAxisForPlacement(a.p, b.p, labelPos)
        : "direct";
    const m: Measurement = {
      id: this.id(),
      mode,
      refA: cloneMeasureRef(refA),
      refB: cloneMeasureRef(refB),
      labelOffset: sub(labelPos, anchor),
      axis,
    };
    this.measurements.push(m);
    return m;
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
        if (!b || ref.index < 0 || ref.index >= b.controlLocal.length) return null;
        return { kind: "point", p: this.bodyControlWorld(b)[ref.index] };
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
        if (!b || ref.index < 0 || ref.index >= b.controlLocal.length) return null;
        const verts = this.bodyControlWorld(b);
        return { kind: "line", a: verts[ref.index], b: verts[(ref.index + 1) % verts.length] };
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
    if (a.kind === "point" && b.kind === "point") {
      m.axis = measureAxisForPlacement(a.p, b.p, labelPos);
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
    if (a.kind === "point" && b.kind === "point") {
      info = pointPointInfo(m.id, a.p, b.p, m.axis, labelPos);
    } else if (a.kind === "line" && b.kind === "line") {
      info = lineLineInfo(m.id, a, b, labelPos);
    } else {
      const p = a.kind === "point" ? a.p : (b as { kind: "point"; p: Vec2 }).p;
      const line = a.kind === "line" ? a : (b as { kind: "line"; a: Vec2; b: Vec2 });
      info = pointLineInfo(m.id, p, line, labelPos);
    }
    if (info && m.driving) info.driving = true;
    return info;
  }

  /** Like `measureInfo`, but for a not-yet-created measurement (live placement preview). */
  measurePreview(refA: MeasureRef, refB: MeasureRef, labelPos: Vec2): MeasureInfo | null {
    const a = this.resolveMeasureRef(refA);
    const b = this.resolveMeasureRef(refB);
    if (!a || !b) return null;
    const anchor = scale(add(refCenter(a), refCenter(b)), 0.5);
    const axis =
      a.kind === "point" && b.kind === "point"
        ? measureAxisForPlacement(a.p, b.p, labelPos)
        : "direct";
    return this.measureInfo({
      id: -1,
      mode: "draw",
      refA,
      refB,
      labelOffset: sub(labelPos, anchor),
      axis,
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
   */
  private shiftMeasureIndices(bodyId: number, at: number, delta: 1 | -1): void {
    const gone = new Set<number>();
    for (const m of this.measurements) {
      for (const ref of [m.refA, m.refB]) {
        if ((ref.kind !== "vertex" && ref.kind !== "edge") || ref.bodyId !== bodyId) continue;
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
        if (!ref || (ref.kind !== "vertex" && ref.kind !== "edge") || ref.bodyId !== bodyId) continue;
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
   * `coincident` takes two point refs; `horizontal`/`vertical` take one line ref (refB
   * omitted) or two point refs; `parallel`/`perpendicular`/`equal` take two line refs.
   * Point refs must be joints or body control vertices (`bodyPoint` refs are
   * measurement-only — the sketch solver can't move them independently). Returns null
   * on a kind mismatch, an unresolvable ref, or two refs naming the same element.
   */
  addSketchConstraint(
    kind: SketchConstraintKind,
    refA: MeasureRef,
    refB?: MeasureRef
  ): SketchConstraint | null {
    const isPoint = (r: MeasureRef) => r.kind === "joint" || r.kind === "vertex";
    const isLine = (r: MeasureRef) => r.kind === "rail" || r.kind === "edge";
    const b = refB ?? null;
    if (kind === "coincident") {
      if (!b || !isPoint(refA) || !isPoint(b)) return null;
    } else if (kind === "horizontal" || kind === "vertical") {
      if (b ? !(isPoint(refA) && isPoint(b)) : !isLine(refA)) return null;
    } else {
      if (!b || !isLine(refA) || !isLine(b)) return null;
    }
    if (!this.resolveMeasureRef(refA) || (b && !this.resolveMeasureRef(b))) return null;
    if (b && sameMeasureRef(refA, b)) return null;
    const c: SketchConstraint = {
      kind,
      id: this.id(),
      refA: cloneMeasureRef(refA),
      refB: b ? cloneMeasureRef(b) : null,
    };
    this.sketch.push(c);
    return c;
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
  setMeasurementDriving(id: number, target: number): boolean {
    const m = this.getMeasurement(id);
    if (!m || m.mode !== "draw" || !(target > 0)) return false;
    m.driving = true;
    m.target = target;
    return true;
  }

  /** Turn a driving dimension back into a driven (read-only) one. */
  clearMeasurementDriving(id: number): void {
    const m = this.getMeasurement(id);
    if (!m) return;
    delete m.driving;
    delete m.target;
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
    const attached = this.joints.filter((j) => j.bodyId === bodyId);
    for (const j of attached) j.local = scale(j.local, factor);
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

  /** Whether a world point lies inside a body's (rounded) polygon. */
  pointInBody(body: Body, p: Vec2): boolean {
    return pointInPolygon(p, this.bodyWorldVerts(body));
  }

  /** `p` if it lies inside the body; otherwise the nearest point on the body's outline. */
  clampIntoBody(body: Body, p: Vec2): Vec2 {
    return this.pointInBody(body, p) ? p : closestPointOnPolygon(p, this.bodyWorldVerts(body));
  }

  /** Topmost body whose polygon contains the point, or undefined. */
  bodyAt(p: Vec2): Body | undefined {
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      if (pointInPolygon(p, this.bodyWorldVerts(this.bodies[i]))) return this.bodies[i];
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
      const vi = this.coincidentVertexIndex(body, this.jointWorld(j));
      if (vi >= 0) {
        this.moveBodyVertex(body.id, vi, delta);
        return;
      }
    }
    this.shiftJoint(j, delta);
  }

  /** Index of the control vertex exactly coincident with world point `p`, or -1. */
  private coincidentVertexIndex(body: Body, p: Vec2): number {
    const ctrl = this.bodyControlWorld(body);
    for (let i = 0; i < ctrl.length; i++) {
      if (dist(ctrl[i], p) < VERTEX_LINK_EPS) return i;
    }
    return -1;
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
    // Reflect the control polygon in world space; reverse it to preserve the winding.
    const ctrlWorld = this.bodyControlWorld(body).map(reflect).reverse();
    const attached = this.joints.filter((j) => j.bodyId === bodyId);
    const jointWorlds = new Map(attached.map((j) => [j.id, reflect(this.jointWorld(j))]));
    const owned = new Set(attached.map((j) => j.id));
    for (const con of this.constraints) {
      if (con.kind === "ground" && owned.has(con.joint)) con.anchor = reflect(con.anchor);
    }
    // Bake the reflected world geometry back in at angle 0 (a reflection isn't a rotation,
    // so the prior angle no longer applies), then let rebuildBody re-derive shape/mass and
    // re-anchor joints to their now-reflected world positions.
    body.angle = 0;
    body.controlLocal = ctrlWorld.map((p) => sub(p, c));
    for (const j of attached) j.local = sub(jointWorlds.get(j.id)!, c);
    this.rebuildBody(body);
  }

  /**
   * Snapshot a body and its dependent features into a copy/paste clip (see `BodyClip`).
   * Returns null if the body doesn't exist.
   */
  extractBody(bodyId: number): BodyClip | null {
    const body = this.getBody(bodyId);
    if (!body) return null;
    const attached = this.joints.filter((j) => j.bodyId === bodyId);
    const owned = new Set(attached.map((j) => j.id));
    const grounds: BodyClip["grounds"] = [];
    const sliders: BodyClip["sliders"] = [];
    const pins: BodyClip["pins"] = [];
    for (const c of this.constraints) {
      if (c.kind === "ground" && owned.has(c.joint)) {
        grounds.push({ joint: c.joint, anchor: clone(c.anchor) });
      } else if (c.kind === "slider" && owned.has(c.railA) && owned.has(c.railB)) {
        sliders.push({
          railA: c.railA,
          railB: c.railB,
          riders: c.riders.filter((r) => owned.has(r)),
        });
      } else if (c.kind === "pin" && owned.has(c.jointA) && owned.has(c.jointB)) {
        pins.push({ a: c.jointA, b: c.jointB });
      }
    }
    return {
      controlWorld: this.bodyControlWorld(body).map(clone),
      radius: body.radius,
      round: body.round,
      color: body.color,
      centroid: clone(body.pos),
      joints: attached.map((j) => ({ tmp: j.id, world: this.jointWorld(j) })),
      grounds,
      sliders,
      pins,
    };
  }

  /**
   * Paste a `BodyClip` so its original centroid lands at `at` (the whole fragment is
   * translated by `at − clip.centroid`). Recreates the body, its joints, and the clipped
   * constraints with fresh ids. Returns the new body's id, or null on failure.
   */
  insertBody(clip: BodyClip, at: Vec2): number | null {
    const offset = sub(at, clip.centroid);
    const body = this.addBody(
      clip.controlWorld.map((p) => add(p, offset)),
      clip.radius,
      clip.round
    );
    if (body.local.length < 3) return null;
    body.color = clip.color; // paste keeps the source body's colour
    const idMap = new Map<number, number>(); // tmp id → new joint id
    for (const j of clip.joints) {
      idMap.set(j.tmp, this.addJoint(body.id, add(j.world, offset)).id);
    }
    for (const g of clip.grounds) {
      const id = idMap.get(g.joint);
      if (id !== undefined) this.addGround(id, add(g.anchor, offset));
    }
    for (const s of clip.sliders) {
      const a = idMap.get(s.railA);
      const b = idMap.get(s.railB);
      if (a === undefined || b === undefined) continue;
      const sl = this.addSlider(a, b);
      for (const r of s.riders) {
        const nr = idMap.get(r);
        if (nr !== undefined) this.attachSliderRider(sl.id, nr);
      }
    }
    for (const p of clip.pins) {
      const a = idMap.get(p.a);
      const b = idMap.get(p.b);
      if (a !== undefined && b !== undefined) this.addPin(a, b);
    }
    return body.id;
  }

  /** Remove a body along with its joints, pruning the constraints that used them. */
  removeBody(id: number): void {
    const removed = new Set(this.joints.filter((j) => j.bodyId === id).map((j) => j.id));
    this.bodies = this.bodies.filter((b) => b.id !== id);
    this.joints = this.joints.filter((j) => j.bodyId !== id);
    this.pruneConstraints(removed);
  }

  /** Remove a single joint, pruning the constraints that used it. */
  removeJoint(id: number): void {
    this.joints = this.joints.filter((j) => j.id !== id);
    this.pruneConstraints(new Set([id]));
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

  clear(): void {
    this.bodies = [];
    this.joints = [];
    this.constraints = [];
    this.measurements = [];
    this.sketch = [];
    this.nextId = 1;
  }

  /** Plain-data snapshot of the scene; safe to JSON.stringify. */
  serialize(): SceneData {
    return {
      version: FORMAT_VERSION,
      bodies: this.bodies,
      joints: this.joints,
      constraints: this.constraints,
      measurements: this.measurements,
      sketch: this.sketch,
    };
  }

  /** Replace the scene's contents from a snapshot. Throws on malformed data. */
  load(data: SceneData): void {
    if (
      !data ||
      !Array.isArray(data.bodies) ||
      !Array.isArray(data.joints) ||
      !Array.isArray(data.constraints)
    ) {
      throw new Error("Not a valid Disjointed file.");
    }
    // Deep-clone so loaded data is independent of the parsed JSON object.
    this.bodies = data.bodies.map((b) => {
      const local = b.local.map((p) => vec(p.x, p.y));
      // Older files (< v5) have no control polygon: treat the saved polygon as a sharp control.
      const src = b as Body & { controlLocal?: Vec2[]; radius?: number; round?: RoundMode };
      const controlLocal = src.controlLocal ? src.controlLocal.map((p) => vec(p.x, p.y)) : local.map((p) => vec(p.x, p.y));
      return { ...b, pos: vec(b.pos.x, b.pos.y), local, controlLocal, radius: src.radius ?? 0, round: src.round ?? "fillet" };
    });
    this.joints = data.joints.map((j) => ({ ...j, local: vec(j.local.x, j.local.y) }));
    // Sliders: drop the legacy origin+dir form (no railA); migrate the earlier
    // single-`slider` rider field to the `riders` array; normalize riders to an array.
    this.constraints = data.constraints
      .filter((c) => c.kind !== "slider" || (c as { railA?: number }).railA !== undefined)
      .map((c) => {
        if (c.kind !== "slider") return { ...c };
        const s = c as SliderConstraint & { slider?: number };
        const riders = Array.isArray(s.riders)
          ? s.riders.slice()
          : typeof s.slider === "number"
          ? [s.slider]
          : [];
        return { kind: "slider", id: s.id, railA: s.railA, railB: s.railB, riders };
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
    const ids = [
      ...this.bodies.map((b) => b.id),
      ...this.joints.map((j) => j.id),
      ...this.constraints.map((c) => c.id),
      ...this.measurements.map((m) => m.id),
      ...this.sketch.map((c) => c.id),
    ];
    this.nextId = (ids.length ? Math.max(...ids) : 0) + 1;
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
  return riders.length === c.riders.length ? c : { ...c, riders };
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
        a.index === (b as { index: number }).index
      );
    case "rail":
      return a.sliderId === (b as { sliderId: number }).sliderId;
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
  const t = dot(sub(p, line.a), u);
  const foot = add(line.a, scale(u, t)); // perpendicular foot on the *infinite* line
  const ext: { a: Vec2; b: Vec2 }[] = [];
  if (t < 0) pushExt(ext, line.a, foot);
  else if (t > l) pushExt(ext, line.b, foot);
  return { id, kind: "distance", value: dist(p, foot), labelPos, dim: { a: p, b: foot }, ext };
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
