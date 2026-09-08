/**
 * Pose-level driving dimensions: draw-mode dimensions whose BOTH ends live on
 * component-instance geometry. Instance shape is design-locked (the definition owns
 * it), so the sketch (shape) solver never touches it — but the *pose* of instances,
 * and of the mobile parts inside them, is assembly state, and that is what these
 * dimensions drive. Components stay a transparent grouping: dimensioning works as if
 * everything were flat.
 *
 * - Ends on two DIFFERENT instances → one instance translates rigidly to the target
 *   (the same motion dragging it makes — draw-mode placement is free; pins render
 *   dotted until sim closes them). A grounded instance never moves. All pose
 *   dimensions are enforced together (Gauss-Seidel rounds over closed-form
 *   translations), so an edit that conflicts with another driving dimension rejects.
 * - Ends on two mobile parts of ONE instance → the rigid-drag machinery re-poses the
 *   internal mechanism: the sim solver drives one part's ref point while everything
 *   outside the instance, plus the other end's rigid unit, is frozen — pins hold.
 *   Two ends actually rigid to each other (same body / chassis group / welded)
 *   can't reach the target and the edit rejects: "already dimensioned at a deeper
 *   level" falls out of unreachability instead of needing special detection.
 *
 * Failure semantics mirror sketch.ts: **reject** — the scene is left untouched and
 * the offending dimensions are returned as `SketchBreak`s (the UI flashes them red).
 *
 * `enforcePoseDims` also runs live during draw-mode drags (main.ts): dragging a
 * component pulls its dimensioned partner instances along, CAD-style. A dimension
 * that can't hold (grounded partner, internal pose reset by a definition edit)
 * renders **violated** (see `MeasureInfo.violated`) until re-applied.
 */
import { Scene, Measurement, MeasureRef, ComponentInstance } from "./model";
import { solve, Driver, SolveFreeze, resetPoseBaselines } from "./solver";
import { SketchBreak, sketchConfig, solveSketch, applyDrivingDimension } from "./sketch";
import { Vec2, vec, add, sub, scale, len, dot, perp, rotate } from "./geometry";

/** Whether a dimension is pose-level: draw-mode with both ends on instance geometry. */
export function isPoseDim(scene: Scene, m: Measurement): boolean {
  return (
    m.mode === "draw" &&
    scene.instanceOfRef(m.refA) !== undefined &&
    scene.instanceOfRef(m.refB) !== undefined
  );
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
 * World translation of the **refB side** that would satisfy the dimension at `target`
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
    const midA = scale(add(a.a, a.b), 0.5);
    const midB = scale(add(b.a, b.b), 0.5);
    const s = dot(sub(midB, midA), n);
    const sg = m.side ?? (s === 0 ? 1 : Math.sign(s));
    return scale(n, sg * target - s);
  }
  // Point + line (either order): perpendicular distance to the infinite line.
  const pt = a.kind === "point" ? a : (b as { kind: "point"; p: Vec2 });
  const ln = a.kind === "line" ? a : (b as { kind: "line"; a: Vec2; b: Vec2 });
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

/** Whether any of an instance's bodies is world-grounded (the instance can't move). */
function instanceGrounded(scene: Scene, inst: ComponentInstance): boolean {
  return inst.bodyMap.some((e) => scene.getBody(e.id)?.grounded);
}

/** Gauss-Seidel budget for the translation enforcement rounds. */
const POSE_MAX_ROUNDS = 32;

/**
 * Enforce every pose dimension together: each round translates, per out-of-tolerance
 * dimension, the movable side's whole instance by the closed-form correction. A side
 * is movable when its instance isn't grounded and isn't pinned by the active drag
 * (`anchoredInstances` — pass the dragged instances so partners follow the drag, never
 * the other way around). Same-instance dimensions can't be fixed by translation and
 * are only *verified* here (`applyPoseDimension` re-poses them via the sim solver).
 * Returns the dimensions still out of tolerance — those render violated.
 */
export function enforcePoseDims(
  scene: Scene,
  anchoredInstances?: ReadonlySet<number>
): SketchBreak[] {
  const dims = scene.measurements.filter(
    (m) => m.mode === "draw" && m.driving && m.target !== undefined && isPoseDim(scene, m)
  );
  if (!dims.length) return [];
  const held = (inst: ComponentInstance): boolean =>
    (anchoredInstances?.has(inst.id) ?? false) || instanceGrounded(scene, inst);
  for (let round = 0; round < POSE_MAX_ROUNDS; round++) {
    let worst = 0;
    for (const m of dims) {
      const err = poseDimError(scene, m);
      if (err === null) continue; // unresolvable / angle-mode — reported below
      if (err <= sketchConfig.tol) continue;
      worst = Math.max(worst, err);
      const instA = scene.instanceOfRef(m.refA);
      const instB = scene.instanceOfRef(m.refB);
      if (!instA || !instB || instA.id === instB.id) continue; // internal pose — see above
      const delta = poseCorrection(scene, m, m.target!);
      if (!delta) continue;
      if (!held(instB)) scene.moveInstance(instB.id, delta);
      else if (!held(instA)) scene.moveInstance(instA.id, scale(delta, -1));
      // Both sides held: leave the residual — the dimension renders violated.
    }
    if (worst <= sketchConfig.tol) return [];
  }
  const out: SketchBreak[] = [];
  for (const m of dims) {
    const err = poseDimError(scene, m) ?? Infinity;
    if (err > sketchConfig.tol) out.push({ id: m.id, kind: "dimension", error: err });
  }
  return out;
}

/**
 * Side-aware residual of a driving pose dimension: the length of the translation that
 * would satisfy it — for a dimension with a held side, a pose at the right absolute
 * distance but on the flipped side reads as (value + target), never as satisfied.
 * Null when the dimension is unresolvable or in angle mode.
 */
function poseDimError(scene: Scene, m: Measurement): number | null {
  const info = scene.measureInfo(m);
  if (!info || info.kind !== "distance") return null;
  const delta = poseCorrection(scene, m, m.target!);
  return delta ? len(delta) : Math.abs(info.value - m.target!);
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
  // Same instance: re-pose the internal mechanism first — translation can't change an
  // internal distance.
  if (instA.id === instB.id && !poseSolveIntra(scene, m, target)) return fail(reject);
  // Enforce every pose dimension together (the candidate included): a conflicting edit
  // fails here with the actual conflicts flagged.
  const leftover = enforcePoseDims(scene);
  if (leftover.length) return fail(leftover);
  // Free geometry follows the moved instances (mixed dimensions, sketch constraints on
  // material pinned to them). A sketch that can't re-satisfy rejects the whole edit.
  const sk = solveSketch(scene);
  if (sk.length) return fail(sk);
  // The drawn layout changed rigidly: slider-lock / weld baselines re-capture from it.
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

/** A sim driver grabbing the ref's world point: a joint ref drives the joint, body
 *  refs drive the body at the resolved point, a rail drives its railA joint. */
function driverForRef(scene: Scene, ref: MeasureRef): Driver | null {
  switch (ref.kind) {
    case "joint":
      return scene.getJoint(ref.jointId) ? { jointId: ref.jointId, target: vec(0, 0) } : null;
    case "vertex":
    case "bodyPoint":
    case "edge": {
      const b = scene.getBody(ref.bodyId);
      const r = scene.resolveMeasureRef(ref);
      if (!b || !r) return null;
      const p = r.kind === "point" ? r.p : scale(add(r.a, r.b), 0.5);
      return { bodyId: b.id, local: rotate(sub(p, b.pos), -b.angle), target: p };
    }
    case "rail": {
      const c = scene.constraints.find((x) => x.kind === "slider" && x.id === ref.sliderId);
      if (!c || c.kind !== "slider") return null;
      return scene.getJoint(c.railA) ? { jointId: c.railA, target: vec(0, 0) } : null;
    }
    default:
      return null;
  }
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
 * Satisfy a same-instance pose dimension by re-posing the instance's internal
 * mechanism: the sim solver drives one end's ref point toward the target distance
 * (recomputed along the current direction each round, so pin-constrained parts pivot
 * into reach) while everything outside the instance, plus the other end's rigid unit,
 * is frozen. Tries driving the refB side first, then the refA side. Returns whether
 * the dimension converged; on failure the scene is restored to entry state.
 */
function poseSolveIntra(scene: Scene, m: Measurement, target: number): boolean {
  const inst = scene.instanceOfRef(m.refA);
  if (!inst) return false;
  const instBodies = new Set(inst.bodyMap.map((e) => e.id));
  const instJoints = new Set([...inst.jointMap, ...inst.anchorMap].map((e) => e.id));
  const snap = JSON.stringify(scene.serialize());
  const attempt = (moveRef: MeasureRef, holdRef: MeasureRef): boolean => {
    const heldUnit = refUnitMembers(scene, holdRef);
    const drv = driverForRef(scene, moveRef);
    if (!heldUnit || !drv) return false;
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
    const err = (): number | null => {
      const info = scene.measureInfo(m);
      if (!info || info.kind !== "distance") return null;
      const delta = poseCorrection(scene, m, target);
      return delta ? len(delta) : Math.abs(info.value - target); // side-aware residual
    };
    for (let r = 0; r < INTRA_ROUNDS; r++) {
      const e = err();
      if (e === null) return false;
      if (e <= sketchConfig.tol) return true;
      const delta = poseCorrection(scene, m, target);
      const at = driverWorld(scene, drv);
      if (!delta || !at) return false;
      drv.target = add(at, moveRef === m.refB ? delta : scale(delta, -1));
      solve(scene, drv, INTRA_ITERS, 1, undefined, undefined, freeze);
    }
    const e = err();
    return e !== null && e <= sketchConfig.tol;
  };
  if (attempt(m.refB, m.refA)) return true;
  scene.load(JSON.parse(snap));
  if (attempt(m.refA, m.refB)) return true;
  scene.load(JSON.parse(snap));
  return false;
}
