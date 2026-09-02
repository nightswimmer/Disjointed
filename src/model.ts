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
  /**
   * Inner cut-outs: hole loops relative to the centroid, in the local frame (v13,
   * absent = none). *Baked* geometry riding the body: holes render as cut-outs and are
   * subtracted from mass/centroid/inertia, and they mirror/rotate/scale/copy with the
   * body — but they have no editable handles, no vertex/edge refs (measurements and
   * sketch constraints can't name them), the corner `radius` doesn't touch them, and
   * picking + joint containment deliberately use the outer outline only (so a joint
   * can sit at the centre of a shaft hole).
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

/**
 * A permanent group of bodies (created from a multi-selection in draw mode). Grouped
 * bodies are selected and moved as one in draw mode, and behave as a **single rigid
 * body** in simulation: the solver applies every impulse to the whole group about its
 * combined centroid, so the members never move relative to each other. A body belongs
 * to at most one group; groups need at least 2 members (smaller ones dissolve).
 */
export interface BodyGroup {
  id: number;
  bodyIds: number[];
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
  | { kind: "edge"; bodyId: number; index: number } // line: control edge index → index+1
  | { kind: "guidePoint"; guideId: number; which: "a" | "b" } // point: a guideline defining point
  | { kind: "guideLine"; guideId: number }; // line: a construction guideline (infinite)

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

/**
 * The document's working unit: what one world unit means physically. Purely a
 * declaration — no coordinate ever changes when the unit changes — but measurements
 * display it and the DXF importer converts incoming files into it.
 */
export type Unit = "mm" | "cm" | "m" | "in";

/** Millimetres per working unit (also used to convert DXF `$INSUNITS` on import). */
export const UNIT_TO_MM: Record<Unit, number> = { mm: 1, cm: 10, m: 1000, in: 25.4 };

const DEFAULT_UNIT: Unit = "mm";

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
  bodies: { tmp: number; controlWorld: Vec2[]; holesWorld: Vec2[][]; radius: number; round: RoundMode; color: string; grounded: boolean }[];
  /** Copied joints: attached ones carry their body's tmp id, free ones null. */
  joints: { tmp: number; bodyTmp: number | null; world: Vec2 }[];
  grounds: { joint: number; anchor: Vec2 }[];
  sliders: { tmp: number; railA: number; railB: number; riders: number[] }[];
  pins: { a: number; b: number }[];
  /** Permanent groups among the copied bodies (member body tmp ids). */
  groups: number[][];
  /** Fully-internal sketch constraints; refs carry the original ids, remapped on paste. */
  sketch: { kind: SketchConstraintKind; refA: MeasureRef; refB: MeasureRef | null }[];
  /** Fully-internal draw-mode driving dimensions; refs carry the original ids. */
  dims: { refA: MeasureRef; refB: MeasureRef; labelOffset: Vec2; axis: MeasureAxis; target: number }[];
}

const FORMAT_VERSION = 13;

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
  groups: BodyGroup[] = [];
  guides: Guide[] = [];
  /** Working unit: 1 world unit = 1 of these (display + import conversion only). */
  unit: Unit = DEFAULT_UNIT;
  private nextId = 1;

  private id(): number {
    return this.nextId++;
  }

  /**
   * Create a body from a control polygon (world coords). `radius` rounds it: `fillet`
   * rounds the corners in place (keeps concavity); `offset` grows the convex hull outward.
   * `holesWorld` (optional) bakes inner cut-out loops into the body (see `Body.holesLocal`).
   */
  addBody(worldVerts: Vec2[], radius = 0, round: RoundMode = "fillet", holesWorld?: Vec2[][]): Body {
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
    if (holesWorld?.length) body.holesLocal = holesWorld.map((loop) => loop.map((p) => vec(p.x, p.y)));
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
    // Holes are baked outlines: the corner radius never touches them, they just ride
    // the body's frame and subtract from the mass properties below.
    const holesWorld = (body.holesLocal ?? []).map((loop) =>
      loop.map((p) => add(body.pos, rotate(p, body.angle)))
    );
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
    if (body.holesLocal)
      body.holesLocal = holesWorld.map((loop) =>
        loop.map((p) => rotate(sub(p, centroid), -body.angle))
      );
    body.invMass = 1 / Math.max(area, 1);
    body.invInertia = 1 / Math.max(inertia, 1);
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
      case "guidePoint": {
        const g = this.getGuide(ref.guideId);
        return g ? { kind: "point", p: clone(g[ref.which]) } : null;
      }
      case "guideLine": {
        // Resolved as the defining segment; consumers that need the infinite line
        // (point+line measurements, the renderer) already extend line refs themselves.
        const g = this.getGuide(ref.guideId);
        return g ? { kind: "line", a: clone(g.a), b: clone(g.b) } : null;
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
   * Point refs are joints, body control vertices, or guideline defining points
   * (`bodyPoint` refs are measurement-only — the sketch solver can't move them
   * independently); line refs are slider rails, body control edges, or guidelines
   * (except `equal`, which rejects guidelines — an infinite line has no length).
   * Returns null on a kind mismatch, an unresolvable ref, or two refs naming the
   * same element.
   */
  addSketchConstraint(
    kind: SketchConstraintKind,
    refA: MeasureRef,
    refB?: MeasureRef
  ): SketchConstraint | null {
    const isPoint = (r: MeasureRef) => r.kind === "joint" || r.kind === "vertex" || r.kind === "guidePoint";
    const isLine = (r: MeasureRef) => r.kind === "rail" || r.kind === "edge" || r.kind === "guideLine";
    const b = refB ?? null;
    if (kind === "coincident") {
      if (!b || !isPoint(refA) || !isPoint(b)) return null;
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
    if (body.holesLocal)
      body.holesLocal = body.holesLocal.map((loop) => loop.map((p) => scale(p, factor)));
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
    const holesWorld = this.bodyHolesWorld(body).map((loop) => loop.map(reflect).reverse());
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
    if (body.holesLocal) body.holesLocal = holesWorld.map((loop) => loop.map((p) => sub(p, c)));
    for (const j of attached) j.local = sub(jointWorlds.get(j.id)!, c);
    this.rebuildBody(body);
    // The reversal renumbered the control polygon, so vertex/edge refs (sketch constraints
    // and measurements) must be remapped to keep naming the same — now reflected — corner
    // or edge: vertex i → n−1−i; edge i (vᵢ→vᵢ₊₁) → n−2−i, wrapping for i = n−1. Without
    // this, an H/V constraint jumps to a different edge and the next sketch solve drags
    // the geometry to satisfy the wrong element. (Reflection itself preserves every
    // constraint kind: H stays H, V stays V, parallel/perpendicular/equal/coincident and
    // distances are all reflection-invariant.)
    const n = body.controlLocal.length;
    const remap = (ref: MeasureRef | null): void => {
      if (!ref || (ref.kind !== "vertex" && ref.kind !== "edge") || ref.bodyId !== bodyId) return;
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

  /**
   * Create a permanent group over `bodyIds`. Any existing group touching one of them is
   * absorbed (grouping is a union — a body belongs to at most one group), so grouping a
   * selection that includes grouped bodies merges everything into a single group.
   * Returns the new group, or null when fewer than 2 distinct existing bodies remain.
   */
  addGroup(bodyIds: number[]): BodyGroup | null {
    const members = new Set<number>();
    for (const id of bodyIds) {
      if (!this.getBody(id)) continue;
      const existing = this.groupOf(id);
      if (existing) existing.bodyIds.forEach((b) => members.add(b));
      else members.add(id);
    }
    if (members.size < 2) return null;
    this.groups = this.groups.filter((g) => !g.bodyIds.some((b) => members.has(b)));
    const group: BodyGroup = { id: this.id(), bodyIds: [...members] };
    this.groups.push(group);
    return group;
  }

  /** Dissolve every group containing any of `bodyIds`. Returns whether anything changed. */
  ungroup(bodyIds: number[]): boolean {
    const hit = new Set(bodyIds);
    const before = this.groups.length;
    this.groups = this.groups.filter((g) => !g.bodyIds.some((b) => hit.has(b)));
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

  /** Drop removed bodies from groups; a group left with fewer than 2 members dissolves. */
  private pruneGroups(): void {
    for (const g of this.groups) g.bodyIds = g.bodyIds.filter((b) => this.getBody(b));
    this.groups = this.groups.filter((g) => g.bodyIds.length >= 2);
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
  extractSelection(bodyIds: number[], freeJointIds: number[] = []): SelectionClip | null {
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
        });
      } else if (c.kind === "pin" && owned.has(c.jointA) && owned.has(c.jointB)) {
        pins.push({ a: c.jointA, b: c.jointB });
      }
    }
    // Permanent groups whose members were copied (at least 2 of them) travel with the clip.
    const groups: number[][] = [];
    for (const g of this.groups) {
      const members = g.bodyIds.filter((id) => bodyIdSet.has(id));
      if (members.length >= 2) groups.push(members);
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
      if (
        m.mode === "draw" &&
        m.driving &&
        m.target !== undefined &&
        internal(m.refA) &&
        internal(m.refB)
      ) {
        dims.push({
          refA: cloneMeasureRef(m.refA),
          refB: cloneMeasureRef(m.refB),
          labelOffset: clone(m.labelOffset),
          axis: m.axis,
          target: m.target,
        });
      }
    }
    return {
      center,
      bodies: bodies.map((b) => ({
        tmp: b.id,
        controlWorld: this.bodyControlWorld(b).map(clone),
        holesWorld: this.bodyHolesWorld(b),
        radius: b.radius,
        round: b.round,
        color: b.color,
        grounded: b.grounded,
      })),
      joints: copiedJoints.map((j) => ({ tmp: j.id, bodyTmp: j.bodyId, world: this.jointWorld(j) })),
      grounds,
      sliders,
      pins,
      groups,
      sketch,
      dims,
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
        (b.holesWorld ?? []).map((loop) => loop.map((p) => add(p, offset)))
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
        if (nr !== undefined) this.attachSliderRider(sl.id, nr);
      }
    }
    for (const p of clip.pins) {
      const a = idMap.get(p.a);
      const b = idMap.get(p.b);
      if (a !== undefined && b !== undefined) this.addPin(a, b);
    }
    for (const g of clip.groups) {
      const ids = g.map((t) => bodyIdMap.get(t)).filter((x): x is number => x !== undefined);
      if (ids.length >= 2) this.addGroup(ids);
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
          return r.kind === "bodyPoint"
            ? { kind: "bodyPoint", bodyId: bid, local: clone(r.local) }
            : { kind: r.kind, bodyId: bid, index: r.index };
        }
        case "rail": {
          const id = sliderIdMap.get(r.sliderId);
          return id === undefined ? null : { kind: "rail", sliderId: id };
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
      this.measurements.push({
        id: this.id(),
        mode: "draw",
        refA: ra,
        refB: rb,
        labelOffset: clone(d.labelOffset),
        axis: d.axis,
        driving: true,
        target: d.target,
      });
    }
    return { bodyIds: [...bodyIdMap.values()], freeJointIds };
  }

  /** Single-body convenience wrapper around `insertSelection`: returns the new body's id. */
  insertBody(clip: SelectionClip, at: Vec2): number | null {
    return this.insertSelection(clip, at)?.bodyIds[0] ?? null;
  }

  /** Remove a body along with its joints, pruning the constraints that used them. */
  removeBody(id: number): void {
    const removed = new Set(this.joints.filter((j) => j.bodyId === id).map((j) => j.id));
    this.bodies = this.bodies.filter((b) => b.id !== id);
    this.joints = this.joints.filter((j) => j.bodyId !== id);
    this.pruneConstraints(removed);
    this.pruneGroups();
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
    this.groups = [];
    this.guides = [];
    this.nextId = 1;
  }

  /** Plain-data snapshot of the scene; safe to JSON.stringify. */
  serialize(): SceneData {
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
    // The working unit arrived in v12; older files simply work in the default.
    this.unit = data.unit && data.unit in UNIT_TO_MM ? data.unit : DEFAULT_UNIT;
    // Deep-clone so loaded data is independent of the parsed JSON object.
    this.bodies = data.bodies.map((b) => {
      const local = b.local.map((p) => vec(p.x, p.y));
      // Older files (< v5) have no control polygon: treat the saved polygon as a sharp control.
      const src = b as Body & { controlLocal?: Vec2[]; radius?: number; round?: RoundMode; grounded?: boolean };
      const controlLocal = src.controlLocal ? src.controlLocal.map((p) => vec(p.x, p.y)) : local.map((p) => vec(p.x, p.y));
      // Hole loops arrived in v13; older files simply have none. Keep only valid loops.
      const holesLocal = Array.isArray(src.holesLocal)
        ? src.holesLocal
            .filter((loop) => Array.isArray(loop) && loop.length >= 3)
            .map((loop) => loop.map((p) => vec(p.x, p.y)))
        : [];
      // Grounded bodies arrived in v10; older files simply have none.
      const out: Body = { ...b, pos: vec(b.pos.x, b.pos.y), local, controlLocal, radius: src.radius ?? 0, round: src.round ?? "fillet", grounded: src.grounded ?? false };
      if (holesLocal.length) out.holesLocal = holesLocal;
      else delete out.holesLocal;
      return out;
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
    // Permanent groups arrived in v9; older files simply have none.
    this.groups = Array.isArray(data.groups)
      ? data.groups.map((g) => ({ id: g.id, bodyIds: Array.isArray(g.bodyIds) ? g.bodyIds.slice() : [] }))
      : [];
    this.pruneGroups(); // drop stale body ids / degenerate groups from hand-edited files
    // Construction guidelines arrived in v11; older files simply have none.
    this.guides = Array.isArray(data.guides)
      ? data.guides
          .map((g) => ({ id: g.id, a: vec(g.a.x, g.a.y), b: vec(g.b.x, g.b.y) }))
          .filter((g) => dist(g.a, g.b) >= Scene.GUIDE_MIN_SPAN)
      : [];
    const ids = [
      ...this.bodies.map((b) => b.id),
      ...this.joints.map((j) => j.id),
      ...this.constraints.map((c) => c.id),
      ...this.measurements.map((m) => m.id),
      ...this.sketch.map((c) => c.id),
      ...this.groups.map((g) => g.id),
      ...this.guides.map((g) => g.id),
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
    case "guidePoint":
      return (
        a.guideId === (b as { guideId: number }).guideId &&
        a.which === (b as { which: "a" | "b" }).which
      );
    case "guideLine":
      return a.guideId === (b as { guideId: number }).guideId;
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
