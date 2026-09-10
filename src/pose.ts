/**
 * Pose-level sketch: draw-mode driving dimensions AND sketch constraints whose every
 * end lives on component-instance geometry. Instance shape is design-locked (the
 * definition owns it), so the sketch (shape) solver never touches it — but the *pose*
 * of instances, and of the mobile parts inside them, is assembly state, and that is
 * what these items drive. Components stay a transparent grouping: dimensioning and
 * constraining work as if everything were flat.
 *
 * Every pose item reduces to a closed-form rigid correction (`PoseMove`):
 * - distances, coincident, point-on-line and point-pair H/V → a **translation**;
 * - line H/V, parallel, perpendicular → a **rotation** about the constrained line's
 *   midpoint (which keeps the line in place while aligning it — translations and
 *   rotations then alternate in the same Gauss-Seidel rounds; each is exact, so a
 *   mixed set settles in a couple of rounds);
 * - "equal" between two locked shapes has no pose solution and is rejected at
 *   creation (model.ts).
 *
 * - Ends on two DIFFERENT instances → one instance moves rigidly (the same motion
 *   dragging / rotating it makes — draw-mode placement is free; pins render dotted
 *   until sim closes them). A grounded instance never moves. All pose items are
 *   enforced together, so an edit that conflicts with another one rejects.
 * - Ends on two mobile parts of ONE instance → the rigid-drag machinery re-poses the
 *   internal mechanism: the sim solver drives one part's ref point while everything
 *   outside the instance, plus the other end's rigid unit, is frozen — pins hold.
 *   Two ends actually rigid to each other (same body / chassis group / welded)
 *   can't reach the target and the edit rejects: "already dimensioned at a deeper
 *   level" falls out of unreachability instead of needing special detection.
 *
 * Failure semantics mirror sketch.ts: **reject** — the scene is left untouched and
 * the offending items are returned as `SketchBreak`s (the UI flashes them red).
 *
 * `enforcePose` also runs live during draw-mode drags (main.ts): dragging a
 * component pulls its dimensioned / constrained partner instances along, CAD-style.
 * An item that can't hold (grounded partner, internal pose reset by a definition
 * edit, an instance rotated against its H constraint) renders **violated** (see
 * `MeasureInfo.violated` / `SketchGlyphView.violated`) until re-applied — the
 * un-anchored settle on drag end re-asserts what it can.
 */
import {
  Scene,
  Measurement,
  MeasureRef,
  ComponentInstance,
  SketchConstraint,
  SketchConstraintKind,
  DIM_VIOLATION_TOL,
  sameMeasureRef,
} from "./model";
import { solve, Driver, SolveFreeze, resetPoseBaselines } from "./solver";
import {
  SketchBreak,
  sketchConfig,
  solveSketch,
  applyDrivingDimension,
  tryAddConstraint,
} from "./sketch";
import { Vec2, vec, add, sub, scale, len, dot, perp, rotate } from "./geometry";

/** Whether a dimension is pose-level: draw-mode with both ends on instance geometry. */
export function isPoseDim(scene: Scene, m: Measurement): boolean {
  return (
    m.mode === "draw" &&
    m.axis !== "diameter" && // a disk's diameter is shape material, never a pose
    scene.instanceOfRef(m.refA) !== undefined &&
    scene.instanceOfRef(m.refB) !== undefined
  );
}

/** Whether a sketch constraint is pose-level: every end on instance geometry (a
 *  single-line H/V included). One free end makes it ordinary shape material instead —
 *  the sketch solver moves the free side, instance variables being immovable there. */
export function isPoseConstraint(scene: Scene, c: SketchConstraint): boolean {
  return scene.refInstanceOwned(c.refA) && (c.refB === null || scene.refInstanceOwned(c.refB));
}

/**
 * Route a dimension-value edit to the right machinery: pose dimensions (both ends on
 * instance geometry) re-pose rigid parts here; everything else goes through the
 * sketch's `applyDrivingDimension` (shape solve). Same reject semantics either way.
 */
export function applyDimensionValue(
  scene: Scene,
  measurementId: number,
  target: number
): SketchBreak[] {
  const m = scene.getMeasurement(measurementId);
  if (m && isPoseDim(scene, m)) return applyPoseDimension(scene, measurementId, target);
  return applyDrivingDimension(scene, measurementId, target);
}

/**
 * Route a constraint placement the same way: every end on instance geometry → pose
 * constraint (rigid parts move, here); otherwise the sketch's `tryAddConstraint`
 * (shape solve). Both return the created constraint, or null plus the conflicts.
 */
export function placeConstraint(
  scene: Scene,
  kind: SketchConstraintKind,
  refA: MeasureRef,
  refB?: MeasureRef
): { constraint: SketchConstraint | null; breaks: SketchBreak[] } {
  const pose = scene.refInstanceOwned(refA) && (!refB || scene.refInstanceOwned(refB));
  if (!pose) return tryAddConstraint(scene, kind, refA, refB);
  return applyPoseConstraint(scene, kind, refA, refB);
}

/** Whether a pose constraint currently fails to hold (rendered in the error style). */
export function poseConstraintViolated(scene: Scene, c: SketchConstraint): boolean {
  if (!isPoseConstraint(scene, c)) return false;
  const err = constraintItem(scene, c).error();
  return err === null || err > DIM_VIOLATION_TOL;
}

// --- pose items ------------------------------------------------------------------

/**
 * A closed-form rigid correction. As given it applies to the **refB side**; the refA
 * side takes the inverse (negated translation, or the negated angle about its own
 * pivot). A single-ref item (line H/V) only has a refA side.
 */
type PoseMove =
  | { kind: "translate"; delta: Vec2 }
  | { kind: "rotate"; angle: number; pivotA: Vec2; pivotB: Vec2 };

/** One enforceable pose item: a driving pose dimension or a pose constraint. Geometry
 *  is re-resolved on every call, so items stay valid across the solve rounds. */
interface PoseItem {
  id: number;
  kind: "constraint" | "dimension";
  refA: MeasureRef;
  refB: MeasureRef | null;
  /** Side-aware residual in world units; null when unresolvable / not enforceable. */
  error(): number | null;
  /** The move that would zero the residual, or null when none exists. */
  correction(): PoseMove | null;
}

type Line = { kind: "line"; a: Vec2; b: Vec2 };
type Point = { kind: "point"; p: Vec2 };

/** Wrap an angle difference into (-π/2, π/2] — direction mismatch modulo a half-turn. */
function wrapHalfPi(a: number): number {
  let d = ((a % Math.PI) + Math.PI) % Math.PI;
  if (d > Math.PI / 2) d -= Math.PI;
  return d;
}

const lineAngle = (l: Line): number => Math.atan2(l.b.y - l.a.y, l.b.x - l.a.x);
const lineMid = (l: Line): Vec2 => scale(add(l.a, l.b), 0.5);
const lineLen = (l: Line): number => len(sub(l.b, l.a));

/** Displacement-scale residual of an angular mismatch on lines of the given length
 *  (same metric as the sketch solver's parallel projection). */
const angularError = (dd: number, l: number): number => Math.abs(Math.sin(dd)) * (l / 2);

/**
 * World translation of the **refB side** that would satisfy a dimension at `target`
 * (moving the refA side instead uses the negation). Null when no translation can
 * satisfy it (degenerate geometry; angle-mode pairs are filtered by the callers via
 * `measureInfo`). The math mirrors `measureInfo`'s value semantics per kind/axis, and
 * the callers iterate against `measureInfo` itself, so any near-parallel approximation
 * converges to the displayed value.
 */
function poseCorrection(scene: Scene, m: Measurement, target: number): Vec2 | null {
  const a = scene.resolveMeasureRef(m.refA);
  const b = scene.resolveMeasureRef(m.refB);
  if (!a || !b) return null;
  // The held side (captured when the dimension started driving) wins over the current
  // geometry's sign, so a fast drag that overshoots past the partner can never flip
  // the two sides through each other — the correction pushes back to the drawn side.
  if (a.kind === "point" && b.kind === "point") {
    if (m.axis === "h" || m.axis === "v") {
      const d = m.axis === "h" ? b.p.x - a.p.x : b.p.y - a.p.y;
      const s = m.side ?? (d === 0 ? 1 : Math.sign(d));
      return m.axis === "h" ? vec(s * target - d, 0) : vec(0, s * target - d);
    }
    const d = sub(b.p, a.p);
    const l = len(d);
    const u = l > 1e-9 ? scale(d, 1 / l) : vec(1, 0);
    return scale(u, target - l);
  }
  if (a.kind === "line" && b.kind === "line") {
    // (Near-)parallel distance: shift line B along line A's normal.
    const d1 = sub(a.b, a.a);
    const l1 = len(d1);
    if (l1 < 1e-9) return null;
    const n = perp(scale(d1, 1 / l1));
    const s = dot(sub(lineMid(b), lineMid(a)), n);
    const sg = m.side ?? (s === 0 ? 1 : Math.sign(s));
    return scale(n, sg * target - s);
  }
  // Point + line (either order): perpendicular distance to the infinite line.
  const pt = a.kind === "point" ? a : (b as Point);
  const ln = a.kind === "line" ? a : (b as Line);
  const d = sub(ln.b, ln.a);
  const l = len(d);
  if (l < 1e-9) return null;
  const n = perp(scale(d, 1 / l));
  const s = dot(sub(pt.p, ln.a), n);
  const sg = m.side ?? (s === 0 ? 1 : Math.sign(s));
  const deltaPoint = scale(n, sg * target - s);
  // deltaPoint moves the point side; when the point is refA, refB (the line) moves the
  // opposite way instead.
  return a.kind === "point" ? scale(deltaPoint, -1) : deltaPoint;
}

/** A driving pose dimension as an item (at `target`, which may differ from `m.target`
 *  while an edit is being tried). */
function dimItem(scene: Scene, m: Measurement, target: number): PoseItem {
  return {
    id: m.id,
    kind: "dimension",
    refA: m.refA,
    refB: m.refB,
    error() {
      // Side-aware: a pose at the right absolute distance but on the flipped side
      // reads as (value + target), never as satisfied.
      const info = scene.measureInfo(m);
      if (!info || info.kind !== "distance") return null;
      const delta = poseCorrection(scene, m, target);
      return delta ? len(delta) : Math.abs(info.value - target);
    },
    correction() {
      const delta = poseCorrection(scene, m, target);
      return delta ? { kind: "translate", delta } : null;
    },
  };
}

/** Build an item from a shift function: the translation that moves the refB side
 *  onto satisfaction (null = unresolvable); its length is the residual. */
function translateItem(c: SketchConstraint, shift: () => Vec2 | null): PoseItem {
  return {
    id: c.id,
    kind: "constraint",
    refA: c.refA,
    refB: c.refB,
    error() {
      const d = shift();
      return d ? len(d) : null;
    },
    correction() {
      const d = shift();
      return d ? { kind: "translate", delta: d } : null;
    },
  };
}

/** A pose constraint as an item. Point pairs and point-on-line translate; lines rotate
 *  about their own midpoint. Geometry is resolved afresh on every call. */
function constraintItem(scene: Scene, c: SketchConstraint): PoseItem {
  const resolveA = () => scene.resolveMeasureRef(c.refA);
  const resolveB = () => (c.refB ? scene.resolveMeasureRef(c.refB) : null);
  switch (c.kind) {
    case "coincident":
      // Point–point: bring B onto A. Point–line (either order): zero the point's signed
      // distance off the infinite line; the point side moves along the normal.
      return translateItem(c, () => {
        const a = resolveA();
        const b = resolveB();
        if (!a || !b) return null;
        if (a.kind === "point" && b.kind === "point") return sub(a.p, b.p);
        if (a.kind === "line" && b.kind === "line") return null;
        const pt = a.kind === "point" ? a : (b as Point);
        const ln = a.kind === "line" ? a : (b as Line);
        const d = sub(ln.b, ln.a);
        const l = len(d);
        if (l < 1e-9) return null;
        const n = perp(scale(d, 1 / l));
        const s = dot(sub(pt.p, ln.a), n);
        const deltaPoint = scale(n, -s);
        return a.kind === "point" ? scale(deltaPoint, -1) : deltaPoint;
      });
    case "horizontal":
    case "vertical": {
      const axis: "x" | "y" = c.kind === "horizontal" ? "y" : "x";
      if (c.refB) {
        // Point pair: level B with A along the constrained axis.
        return translateItem(c, () => {
          const a = resolveA();
          const b = resolveB();
          if (!a || !b || a.kind !== "point" || b.kind !== "point") return null;
          const d = a.p[axis] - b.p[axis];
          return axis === "y" ? vec(0, d) : vec(d, 0);
        });
      }
      // A single line: rotate its instance about the line's midpoint onto the axis.
      const targetAng = c.kind === "horizontal" ? 0 : Math.PI / 2;
      const mismatch = (): { dd: number; ln: Line } | null => {
        const a = resolveA();
        if (!a || a.kind !== "line" || lineLen(a) < 1e-9) return null;
        return { dd: wrapHalfPi(targetAng - lineAngle(a)), ln: a };
      };
      return {
        id: c.id,
        kind: "constraint",
        refA: c.refA,
        refB: null,
        error() {
          const mm = mismatch();
          return mm ? angularError(mm.dd, lineLen(mm.ln)) : null;
        },
        correction() {
          const mm = mismatch();
          if (!mm) return null;
          // Only a refA side exists: it takes the negated angle, i.e. +dd.
          const mid = lineMid(mm.ln);
          return { kind: "rotate", angle: -mm.dd, pivotA: mid, pivotB: mid };
        },
      };
    }
    case "parallel":
    case "perpendicular": {
      const offset = c.kind === "perpendicular" ? Math.PI / 2 : 0;
      const mismatch = (): { dd: number; a: Line; b: Line } | null => {
        const a = resolveA();
        const b = resolveB();
        if (!a || !b || a.kind !== "line" || b.kind !== "line") return null;
        if (lineLen(a) < 1e-9 || lineLen(b) < 1e-9) return null;
        return { dd: wrapHalfPi(lineAngle(b) - lineAngle(a) - offset), a, b };
      };
      return {
        id: c.id,
        kind: "constraint",
        refA: c.refA,
        refB: c.refB,
        error() {
          const mm = mismatch();
          return mm ? angularError(mm.dd, Math.max(lineLen(mm.a), lineLen(mm.b))) : null;
        },
        correction() {
          const mm = mismatch();
          if (!mm) return null;
          // B turns by -dd onto A's direction; A instead turns by +dd onto B's.
          return { kind: "rotate", angle: -mm.dd, pivotA: lineMid(mm.a), pivotB: lineMid(mm.b) };
        },
      };
    }
    case "equal":
      // Both lengths are locked to their definitions — never satisfiable as a pose.
      return { id: c.id, kind: "constraint", refA: c.refA, refB: c.refB, error: () => Infinity, correction: () => null };
  }
}

/** Every enforceable pose item in the scene: driving pose dims + pose constraints. */
function poseItems(scene: Scene): PoseItem[] {
  const out: PoseItem[] = [];
  for (const m of scene.measurements) {
    if (m.mode === "draw" && m.driving && m.target !== undefined && isPoseDim(scene, m)) {
      out.push(dimItem(scene, m, m.target));
    }
  }
  for (const c of scene.sketch) if (isPoseConstraint(scene, c)) out.push(constraintItem(scene, c));
  return out;
}

/** Apply a move to one side's whole instance. */
function applyMove(scene: Scene, inst: ComponentInstance, move: PoseMove, side: "A" | "B"): void {
  if (move.kind === "translate") {
    scene.moveInstance(inst.id, side === "B" ? move.delta : scale(move.delta, -1));
  } else {
    scene.rotateInstance(
      inst.id,
      side === "B" ? move.pivotB : move.pivotA,
      side === "B" ? move.angle : -move.angle
    );
  }
}

/** Whether any of an instance's bodies is world-grounded (the instance can't move). */
function instanceGrounded(scene: Scene, inst: ComponentInstance): boolean {
  return inst.bodyMap.some((e) => scene.getBody(e.id)?.grounded);
}

/** Gauss-Seidel budget for the rigid enforcement rounds. */
const POSE_MAX_ROUNDS = 32;

/**
 * Enforce every pose item together: each round moves, per out-of-tolerance item, the
 * movable side's whole instance by the closed-form correction (a translation or a
 * rotation). A side is movable when its instance isn't grounded and isn't pinned by
 * the active drag (`anchoredInstances` — pass the dragged instances so partners follow
 * the drag, never the other way around). Same-instance items can't be fixed by a rigid
 * move and are only *verified* here (`applyPoseDimension` / `applyPoseConstraint`
 * re-pose them via the sim solver). Returns the items still out of tolerance — those
 * render violated.
 */
export function enforcePose(
  scene: Scene,
  anchoredInstances?: ReadonlySet<number>
): SketchBreak[] {
  const items = poseItems(scene);
  if (!items.length) return [];
  const held = (inst: ComponentInstance): boolean =>
    (anchoredInstances?.has(inst.id) ?? false) || instanceGrounded(scene, inst);
  for (let round = 0; round < POSE_MAX_ROUNDS; round++) {
    let worst = 0;
    for (const it of items) {
      const err = it.error();
      if (err === null) continue; // unresolvable / angle-mode — reported below
      if (err <= sketchConfig.tol) continue;
      worst = Math.max(worst, err);
      const instA = scene.instanceOfRef(it.refA);
      const instB = it.refB ? scene.instanceOfRef(it.refB) : undefined;
      if (!instA) continue;
      if (instB && instA.id === instB.id) continue; // internal pose — see above
      const move = it.correction();
      if (!move) continue;
      if (instB && !held(instB)) applyMove(scene, instB, move, "B");
      else if (!held(instA)) applyMove(scene, instA, move, "A");
      // Every side held: leave the residual — the item renders violated.
    }
    if (worst <= sketchConfig.tol) return [];
  }
  const out: SketchBreak[] = [];
  for (const it of items) {
    const err = it.error() ?? Infinity;
    if (err > sketchConfig.tol) out.push({ id: it.id, kind: it.kind, error: err });
  }
  return out;
}

/**
 * Set a pose dimension to drive `target`, moving rigid parts to satisfy it (see the
 * module doc). On success the dimension is marked driving and [] is returned; on an
 * unsatisfiable edit the scene and the dimension are left untouched and the
 * conflicting items are returned (reject semantics, like `applyDrivingDimension`).
 */
export function applyPoseDimension(
  scene: Scene,
  measurementId: number,
  target: number
): SketchBreak[] {
  const m = scene.getMeasurement(measurementId);
  const reject = [{ id: measurementId, kind: "dimension" as const, error: Infinity }];
  if (!m || m.mode !== "draw" || !(target > 0)) return reject;
  const info = scene.measureInfo(m);
  if (!info || info.kind !== "distance") return reject; // angle dimensions can't drive
  const instA = scene.instanceOfRef(m.refA);
  const instB = scene.instanceOfRef(m.refB);
  if (!instA || !instB) return reject; // not a pose dim — the router sends those to sketch
  const snap = JSON.stringify(scene.serialize());
  const fail = (breaks: SketchBreak[]): SketchBreak[] => {
    scene.load(JSON.parse(snap));
    resetPoseBaselines();
    return breaks;
  };
  // Backstop: two ends rigid to one another (same body / chassis group) can never drive.
  if (!scene.setMeasurementDriving(m.id, target)) return fail(reject);
  // Same instance: re-pose the internal mechanism first — a rigid move can't change an
  // internal distance.
  if (instA.id === instB.id && !poseSolveIntra(scene, dimItem(scene, m, target), "B")) return fail(reject);
  const leftover = settlePose(scene, instA.id);
  return leftover.length ? fail(leftover) : [];
}

/**
 * Add a pose constraint (every end on instance geometry), moving rigid parts to satisfy
 * it. Returns the constraint, or null plus the conflicts with the scene left untouched
 * (reject semantics, like `tryAddConstraint`). Null with no breaks means the reference
 * combination is invalid or the ends are rigid to one another at a deeper level.
 * The **second-picked** side moves by preference (`refA` is the first pick — the model
 * may store a point-on-line pair the other way round); when it can't (grounded), the
 * first-picked side moves instead.
 */
export function applyPoseConstraint(
  scene: Scene,
  kind: SketchConstraintKind,
  refA: MeasureRef,
  refB?: MeasureRef
): { constraint: SketchConstraint | null; breaks: SketchBreak[] } {
  const snap = JSON.stringify(scene.serialize());
  const c = scene.addSketchConstraint(kind, refA, refB);
  if (!c) return { constraint: null, breaks: [] };
  const fail = (breaks: SketchBreak[]): { constraint: null; breaks: SketchBreak[] } => {
    scene.load(JSON.parse(snap)); // predates the constraint: it's gone with the restore
    resetPoseBaselines();
    return { constraint: null, breaks };
  };
  const reject = [{ id: c.id, kind: "constraint" as const, error: Infinity }];
  const item = constraintItem(scene, c);
  const instA = scene.instanceOfRef(c.refA);
  const instB = c.refB ? scene.instanceOfRef(c.refB) : undefined;
  if (!instA || (c.refB && !instB)) return fail(reject);
  // The side holding the user's first pick stays put by preference.
  const firstIsA = sameMeasureRef(c.refA, refA);
  const firstInst = firstIsA ? instA : instB!;
  if (instB && instA.id === instB.id && !poseSolveIntra(scene, item, firstIsA ? "B" : "A")) {
    return fail(reject);
  }
  const leftover = settlePose(scene, firstInst.id);
  return leftover.length ? fail(leftover) : { constraint: c, breaks: [] };
}

/** Shared tail of a pose edit: enforce every pose item together (the candidate
 *  included — a conflicting edit fails here with the actual conflicts flagged), let
 *  free geometry follow the moved instances (mixed dims, sketch constraints on material
 *  pinned to them), then re-capture slider-lock / weld baselines from the new layout.
 *  `preferHeld` names the instance that should stay put if the rest can satisfy the
 *  edit (the user's first pick); when it can't, everything movable is fair game.
 *  Returns the conflicts (empty on success). */
function settlePose(scene: Scene, preferHeld?: number): SketchBreak[] {
  let leftover = preferHeld !== undefined ? enforcePose(scene, new Set([preferHeld])) : [];
  if (leftover.length || preferHeld === undefined) leftover = enforcePose(scene);
  if (leftover.length) return leftover;
  const sk = solveSketch(scene);
  if (sk.length) return sk;
  resetPoseBaselines();
  return [];
}

// --- internal (same-instance) pose solve ---------------------------------------

const INTRA_ROUNDS = 60;
const INTRA_ITERS = 40;

/**
 * Members (bodies + locked free joints) of the rigid unit a ref belongs to: its
 * body's whole group (e.g. the chassis), the lone body, or the lone free joint.
 */
function refUnitMembers(
  scene: Scene,
  ref: MeasureRef
): { bodies: Set<number>; joints: Set<number> } | null {
  const out = { bodies: new Set<number>(), joints: new Set<number>() };
  const addBody = (bodyId: number): boolean => {
    if (!scene.getBody(bodyId)) return false;
    const g = scene.groupOf(bodyId);
    if (g) {
      g.bodyIds.forEach((id) => out.bodies.add(id));
      g.jointIds.forEach((id) => out.joints.add(id));
    } else {
      out.bodies.add(bodyId);
    }
    return true;
  };
  switch (ref.kind) {
    case "vertex":
    case "edge":
    case "bodyPoint":
      return addBody(ref.bodyId) ? out : null;
    case "joint": {
      const j = scene.getJoint(ref.jointId);
      if (!j) return null;
      if (j.bodyId !== null) return addBody(j.bodyId) ? out : null;
      const g = scene.groupOfJoint(j.id);
      if (g) {
        g.bodyIds.forEach((id) => out.bodies.add(id));
        g.jointIds.forEach((id) => out.joints.add(id));
      } else {
        out.joints.add(j.id);
      }
      return out;
    }
    case "rail": {
      const c = scene.constraints.find((x) => x.kind === "slider" && x.id === ref.sliderId);
      if (!c || c.kind !== "slider") return null;
      return refUnitMembers(scene, { kind: "joint", jointId: c.railA });
    }
    default:
      return null;
  }
}

/** A sim driver grabbing a world point `p` that belongs to the ref's element: a joint
 *  ref drives the joint, body refs drive the body at `p`, a rail drives one of its
 *  joints (railB when `p` is its far end, railA otherwise). */
function driverAt(scene: Scene, ref: MeasureRef, p: Vec2): Driver | null {
  switch (ref.kind) {
    case "joint":
      return scene.getJoint(ref.jointId) ? { jointId: ref.jointId, target: vec(0, 0) } : null;
    case "vertex":
    case "bodyPoint":
    case "edge": {
      const b = scene.getBody(ref.bodyId);
      if (!b) return null;
      return { bodyId: b.id, local: rotate(sub(p, b.pos), -b.angle), target: p };
    }
    case "rail": {
      const c = scene.constraints.find((x) => x.kind === "slider" && x.id === ref.sliderId);
      if (!c || c.kind !== "slider") return null;
      const jb = scene.getJoint(c.railB);
      const far = jb !== undefined && len(sub(scene.jointWorld(jb), p)) < 1e-6;
      const id = far ? c.railB : c.railA;
      return scene.getJoint(id) ? { jointId: id, target: vec(0, 0) } : null;
    }
    default:
      return null;
  }
}

/** The world point a ref's driver should grab: a point ref's point; a line ref's far
 *  end (`b`) when the correction is a rotation (turning it about `a`), else its midpoint. */
function grabPoint(scene: Scene, ref: MeasureRef, rotating: boolean): Vec2 | null {
  const r = scene.resolveMeasureRef(ref);
  if (!r) return null;
  if (r.kind === "point") return r.p;
  return rotating ? r.b : lineMid(r);
}

/** Current world point a driver grabs. */
function driverWorld(scene: Scene, drv: Driver): Vec2 | null {
  if (drv.jointId !== undefined) {
    const j = scene.getJoint(drv.jointId);
    return j ? scene.jointWorld(j) : null;
  }
  const b = drv.bodyId !== undefined ? scene.getBody(drv.bodyId) : undefined;
  return b && drv.local ? add(b.pos, rotate(drv.local, b.angle)) : null;
}

/**
 * Satisfy a same-instance pose item by re-posing the instance's internal mechanism:
 * the sim solver drives one end's grab point toward where the closed-form correction
 * would put it (recomputed each round, so pin-constrained parts pivot into reach — a
 * rotation drives the line's far end around its near end) while everything outside
 * the instance, plus the other end's rigid unit, is frozen. Tries moving the
 * `moveFirst` side first, then the other one. Returns whether the item converged; on
 * failure the scene is restored to entry state.
 */
function poseSolveIntra(scene: Scene, item: PoseItem, moveFirst: "A" | "B"): boolean {
  const inst = scene.instanceOfRef(item.refA);
  if (!inst || !item.refB) return false;
  const refB = item.refB;
  const instBodies = new Set(inst.bodyMap.map((e) => e.id));
  const instJoints = new Set([...inst.jointMap, ...inst.anchorMap].map((e) => e.id));
  const snap = JSON.stringify(scene.serialize());
  const attempt = (moveRef: MeasureRef, holdRef: MeasureRef, side: "A" | "B"): boolean => {
    const heldUnit = refUnitMembers(scene, holdRef);
    const first = item.correction();
    if (!heldUnit || !first) return false;
    const rotating = first.kind === "rotate";
    const grab = grabPoint(scene, moveRef, rotating);
    const drv = grab ? driverAt(scene, moveRef, grab) : null;
    if (!drv) return false;
    // Frozen: the world outside this instance, plus the held end's rigid unit — the
    // rest of the instance's mechanism re-poses around it, pins intact.
    const freeze: SolveFreeze = {
      bodies: new Set(
        scene.bodies
          .filter((b) => !instBodies.has(b.id) || heldUnit.bodies.has(b.id))
          .map((b) => b.id)
      ),
      joints: new Set(
        scene.joints
          .filter((j) => j.bodyId === null && (!instJoints.has(j.id) || heldUnit.joints.has(j.id)))
          .map((j) => j.id)
      ),
    };
    for (let r = 0; r < INTRA_ROUNDS; r++) {
      const e = item.error();
      if (e === null) return false;
      if (e <= sketchConfig.tol) return true;
      const move = item.correction();
      const at = driverWorld(scene, drv);
      if (!move || !at) return false;
      if (move.kind === "translate") {
        drv.target = add(at, side === "B" ? move.delta : scale(move.delta, -1));
      } else {
        // Turn the grabbed far end about the line's near end by this side's angle.
        const ln = scene.resolveMeasureRef(moveRef);
        if (!ln || ln.kind !== "line") return false;
        const ang = side === "B" ? move.angle : -move.angle;
        drv.target = add(ln.a, rotate(sub(at, ln.a), ang));
      }
      solve(scene, drv, INTRA_ITERS, 1, undefined, undefined, freeze);
    }
    const e = item.error();
    return e !== null && e <= sketchConfig.tol;
  };
  const tryB = () => attempt(refB, item.refA, "B");
  const tryA = () => attempt(item.refA, refB, "A");
  if (moveFirst === "B" ? tryB() : tryA()) return true;
  scene.load(JSON.parse(snap));
  if (moveFirst === "B" ? tryA() : tryB()) return true;
  scene.load(JSON.parse(snap));
  return false;
}
