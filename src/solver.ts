/**
 * Iterative position-based constraint solver (Gauss-Seidel projection).
 *
 * Each constraint is satisfied by projecting the involved bodies' poses so the
 * positional error at the anchor points is removed. A positional impulse `λ`
 * applied at world offset `r` from a body's centroid updates the pose by:
 *     pos   += invMass     * λ
 *     angle += invInertia  * cross(r, λ)
 * Looping over all constraints several times converges the whole mechanism.
 */
import { Body, Joint, Scene } from "./model";
import { Vec2, add, rotate, cross, perp, scale, sub, len, dot } from "./geometry";

export interface Driver {
  jointId: number;
  target: Vec2;
}

/** Inverse mass of a free (body-less) joint — a light translational point particle. */
const FREE_INV_MASS = 1;

/**
 * A constraint participant the solver can move: a rigid body, a free joint, or an
 * immovable world point. `point` is where the impulse acts; `pos` is the rotation
 * reference (body centroid, or the point itself); `apply` adds an impulse there.
 */
interface Host {
  point: Vec2;
  pos: Vec2;
  invMass: number;
  invInertia: number;
  apply(imp: Vec2): void;
}

/**
 * The rail side of a slider. Like a Host, but the impulse acts at an arbitrary world
 * point (the rider's), not the rail's own anchor — so it carries `applyAt(point, imp)`.
 * A rail is either a rigid body (two rail joints on it) or an immovable world line (two
 * grounded free joints), in which case `invMass`/`invInertia` are 0 and `applyAt` is a no-op.
 */
interface RailHost {
  pos: Vec2;
  invMass: number;
  invInertia: number;
  applyAt(point: Vec2, imp: Vec2): void;
}

/** Apply impulse `imp` at world `point` to a rigid body (updates pos + angle). */
function bodyImpulse(body: Body, point: Vec2, imp: Vec2): void {
  const r = sub(point, body.pos);
  body.pos = add(body.pos, scale(imp, body.invMass));
  body.angle += body.invInertia * cross(r, imp);
}

/** Resolve a joint to its solver host: a body-attached point, or a free point. */
function hostFor(scene: Scene, joint: Joint, grounded: Set<number>): Host {
  if (joint.bodyId === null) {
    // A grounded free joint is an immovable anchor: fixed for *every* constraint, so a
    // heavy body pinned to it gets pulled onto the anchor (not the light point shoved away).
    if (grounded.has(joint.id)) return fixedHost(joint.local);
    // Otherwise a free joint is a translational point particle (no rotation); its world
    // position lives in joint.local, which `apply` mutates directly.
    return {
      point: joint.local,
      pos: joint.local,
      invMass: FREE_INV_MASS,
      invInertia: 0,
      apply(imp) {
        joint.local = add(joint.local, scale(imp, FREE_INV_MASS));
      },
    };
  }
  const body = scene.getBody(joint.bodyId)!;
  const point = add(body.pos, rotate(joint.local, body.angle));
  return {
    point,
    pos: body.pos,
    invMass: body.invMass,
    invInertia: body.invInertia,
    apply(imp) {
      bodyImpulse(body, point, imp);
    },
  };
}

/** An immovable world point (a ground anchor or the mouse target). */
function fixedHost(p: Vec2): Host {
  return { point: p, pos: p, invMass: 0, invInertia: 0, apply() {} };
}

/** Solve K·x = -c for a symmetric 2×2 K = [[a,b],[b,d]]; returns [0,0] if singular. */
function solve2x2(a: number, b: number, d: number, c: Vec2): Vec2 {
  const det = a * d - b * b;
  if (Math.abs(det) < 1e-12) return { x: 0, y: 0 };
  const inv = 1 / det;
  // x = -K^{-1} c
  return {
    x: -inv * (d * c.x - b * c.y),
    y: -inv * (-b * c.x + a * c.y),
  };
}

/**
 * Drive a two-point coincidence: make host `a`'s point meet host `b`'s point.
 * `relax` (0..1) under-relaxes so coupled closed loops stay stable. `maxStep` caps
 * the error corrected in one call (used by the mouse driver so a far / possibly
 * unreachable target is approached in small, stable steps). Pass a fixedHost to pin
 * one side to a fixed world point.
 */
function solveCoincident(a: Host, b: Host, relax: number, maxStep = Infinity): void {
  let c = sub(a.point, b.point); // error to remove
  const e = len(c);
  if (e > maxStep) c = scale(c, maxStep / e);

  // Effective-mass matrix K = sum over hosts of invMass*I + invI*[r]x[r]x^T.
  const rA = sub(a.point, a.pos);
  const rB = sub(b.point, b.pos);
  const ka = a.invMass + b.invMass + a.invInertia * rA.y * rA.y + b.invInertia * rB.y * rB.y;
  const kb = -a.invInertia * rA.x * rA.y - b.invInertia * rB.x * rB.y;
  const kd = a.invMass + b.invMass + a.invInertia * rA.x * rA.x + b.invInertia * rB.x * rB.x;

  const lambda = scale(solve2x2(ka, kb, kd, c), relax);
  a.apply(lambda);
  b.apply(scale(lambda, -1));
}

/**
 * Apply a scalar positional impulse driving `C = c → 0` along a unit direction `u`
 * fixed in the rail's frame. Couples the `rider` host (feels +u at its point) and the
 * `rail` (feels −u, applied at the rider's point). The rail angular Jacobian reduces to
 * cross(u, point − rail.pos); a grounded/immovable rail makes it single-sided.
 */
function solveAxis(rider: Host, rail: RailHost, u: Vec2, c: number, relax: number): void {
  const pQ = rider.point;
  const jQ = cross(sub(pQ, rider.pos), u);
  const jR = cross(u, sub(pQ, rail.pos));
  const w =
    rider.invMass + rider.invInertia * jQ * jQ +
    rail.invMass + rail.invInertia * jR * jR;
  if (w < 1e-12) return;
  const lambda = (-c / w) * relax;
  rider.apply(scale(u, lambda));
  rail.applyAt(pQ, scale(u, -lambda));
}

/**
 * Slider/prismatic constraint with end-stops: keep the `rider` on the line through
 * rail points `pA`,`pB` AND between them. The perpendicular part holds it on the rail;
 * the tangential part is one-sided — it only activates once the rider passes an
 * endpoint, clamping it back into the span. The `rail` host carries the reaction (a
 * rigid body, or an immovable world line for a rail of two grounded free joints).
 */
function solveSliderRail(rider: Host, pA: Vec2, pB: Vec2, rail: RailHost, relax: number): void {
  const d = sub(pB, pA);
  const dl = len(d);
  if (dl < 1e-9) return; // degenerate rail (rail joints coincide)
  const dir = scale(d, 1 / dl);
  const n = perp(dir); // unit normal to the rail

  // Hold the rider on the rail line (n ⊥ dir, so this doesn't change position along dir).
  solveAxis(rider, rail, n, dot(sub(rider.point, pA), n), relax);

  // Clamp the rider between the endpoints: only correct when it has overshot one.
  const s = dot(sub(rider.point, pA), dir);
  if (s < 0) solveAxis(rider, rail, dir, s, relax);
  else if (s > dl) solveAxis(rider, rail, dir, s - dl, relax);
}

/**
 * Classify a slider's rail: two joints on one body ("body" — a moving rail), two
 * grounded free joints ("fixed" — a track fixed in world space), or an unsolvable
 * configuration (null, e.g. a free rail joint that isn't grounded).
 */
function railKind(ja: Joint, jb: Joint, grounded: Set<number>): "body" | "fixed" | null {
  if (ja.bodyId !== null && ja.bodyId === jb.bodyId) return "body";
  if (ja.bodyId === null && jb.bodyId === null && grounded.has(ja.id) && grounded.has(jb.id))
    return "fixed";
  return null;
}

/** Build the rail's reaction host for one solve: a moving body, or an immovable world line. */
function railHostFor(railBody: Body | null, fixedPos: Vec2): RailHost {
  if (railBody) {
    return {
      pos: railBody.pos,
      invMass: railBody.invMass,
      invInertia: railBody.invInertia,
      applyAt: (pt, imp) => bodyImpulse(railBody, pt, imp),
    };
  }
  return { pos: fixedPos, invMass: 0, invInertia: 0, applyAt() {} };
}

/** One Gauss-Seidel sweep over the structural constraints (pin / ground / slider). */
function sweepStructural(scene: Scene, relax: number, grounded: Set<number>): void {
  for (const con of scene.constraints) {
    if (con.kind === "pin") {
      const ja = scene.getJoint(con.jointA);
      const jb = scene.getJoint(con.jointB);
      if (!ja || !jb) continue;
      solveCoincident(hostFor(scene, ja, grounded), hostFor(scene, jb, grounded), relax);
    } else if (con.kind === "ground") {
      const j = scene.getJoint(con.joint);
      if (!j) continue;
      solveCoincident(hostFor(scene, j, grounded), fixedHost(con.anchor), relax);
    } else if (con.kind === "slider") {
      const ja = scene.getJoint(con.railA);
      const jb = scene.getJoint(con.railB);
      if (!ja || !jb) continue;
      // Rail = two joints on one body (moving), or two grounded free joints (fixed track).
      const kind = railKind(ja, jb, grounded);
      if (!kind) continue;
      const railBody = kind === "body" ? scene.getBody(ja.bodyId!)! : null;
      for (const riderId of con.riders) {
        const jq = scene.getJoint(riderId);
        if (!jq) continue;
        if (railBody && jq.bodyId === railBody.id) continue; // rider rigid to the rail: nothing to do
        // Recompute the rail each rider, since a rider's reaction can move the rail body.
        solveSliderRail(
          hostFor(scene, jq, grounded),
          scene.jointWorld(ja),
          scene.jointWorld(jb),
          railHostFor(railBody, scene.jointWorld(ja)),
          relax
        );
      }
    }
  }
}

/** Joint ids fixed by a ground constraint (used to make grounded free joints immovable). */
function groundedJoints(scene: Scene): Set<number> {
  const set = new Set<number>();
  for (const c of scene.constraints) if (c.kind === "ground") set.add(c.joint);
  return set;
}

/**
 * Max distance (world units) the driven joint is pulled toward the cursor per
 * sweep. Small steps keep the linearized correction valid, so when the target
 * is unreachable the joint walks stably along its feasible path (e.g. the circle
 * around a ground point) instead of overshooting and spinning the body.
 */
const DRIVER_MAX_STEP = 8;
/** Convergence target: keep sweeping until the worst constraint error is below this (world units). */
const STRUCTURAL_TOL = 1e-4;
/** Safety cap on convergence sweeps so an over-constrained scene can't loop forever. */
const MAX_CLEANUP_SWEEPS = 1000;

/** Largest positional error across all structural constraints (world units). */
function structuralResidual(scene: Scene, grounded: Set<number>): number {
  let max = 0;
  for (const con of scene.constraints) {
    if (con.kind === "pin") {
      const ja = scene.getJoint(con.jointA);
      const jb = scene.getJoint(con.jointB);
      if (!ja || !jb) continue;
      max = Math.max(max, len(sub(scene.jointWorld(ja), scene.jointWorld(jb))));
    } else if (con.kind === "ground") {
      const j = scene.getJoint(con.joint);
      if (!j) continue;
      max = Math.max(max, len(sub(scene.jointWorld(j), con.anchor)));
    } else if (con.kind === "slider") {
      const ja = scene.getJoint(con.railA);
      const jb = scene.getJoint(con.railB);
      if (!ja || !jb) continue;
      if (!railKind(ja, jb, grounded)) continue; // matches sweep's guard
      const a = scene.jointWorld(ja);
      const d = sub(scene.jointWorld(jb), a);
      const dl = len(d);
      if (dl < 1e-9) continue;
      const dir = scale(d, 1 / dl);
      for (const riderId of con.riders) {
        const jq = scene.getJoint(riderId);
        if (!jq) continue;
        const q = sub(scene.jointWorld(jq), a);
        const perpDist = Math.abs(dot(q, perp(dir))); // off the rail line
        const s = dot(q, dir); // position along the rail
        const overshoot = s < 0 ? -s : s > dl ? s - dl : 0; // past an endpoint
        max = Math.max(max, perpDist, overshoot);
      }
    }
  }
  return max;
}

/**
 * Run the solver. `iterations` Gauss-Seidel sweeps over the structural
 * constraints (plus the optional soft mouse driver). `relax` under-relaxes the
 * structural corrections. Afterwards, structural-only sweeps keep running until
 * the worst constraint error falls below STRUCTURAL_TOL (or MAX_CLEANUP_SWEEPS
 * is hit). This adapts to mechanism complexity — simple scenes converge in a
 * couple of sweeps, closed loops get as many as they need — and makes the driver
 * yield to the structural constraints, so it can never break a pin/ground/slider.
 */
export function solve(scene: Scene, driver: Driver | null, iterations = 100, relax = 1): void {
  const grounded = groundedJoints(scene);
  for (let iter = 0; iter < iterations; iter++) {
    sweepStructural(scene, relax, grounded);
    if (driver) {
      const j = scene.getJoint(driver.jointId);
      if (j) solveCoincident(hostFor(scene, j, grounded), fixedHost(driver.target), 1, DRIVER_MAX_STEP);
    }
  }

  // Converge structural constraints to tolerance; they take strict priority over
  // the driver. Stops early once the mechanism is tight enough.
  for (let i = 0; i < MAX_CLEANUP_SWEEPS; i++) {
    if (structuralResidual(scene, grounded) < STRUCTURAL_TOL) break;
    sweepStructural(scene, relax, grounded);
  }
}
