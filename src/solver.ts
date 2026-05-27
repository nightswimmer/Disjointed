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
import { Body, Scene } from "./model";
import { Vec2, add, cross, perp, scale, sub, len, dot } from "./geometry";

export interface Driver {
  jointId: number;
  target: Vec2;
}

/** Apply a positional impulse `imp` to `body` at world offset `r` from its centroid. */
function applyImpulse(body: Body, r: Vec2, imp: Vec2): void {
  body.pos = add(body.pos, scale(imp, body.invMass));
  body.angle += body.invInertia * cross(r, imp);
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
 * Drive a two-point coincidence: make world point `pA` on `bodyA` meet world
 * point `pB` on `bodyB`. `bodyB` may be null to pin against a fixed world point.
 * `relax` (0..1) under-relaxes the correction so coupled closed loops stay stable.
 * `maxStep` caps the error corrected in one call; used by the mouse driver so a
 * far-away (possibly unreachable) target is approached in small, stable steps.
 */
function solveCoincident(
  pA: Vec2,
  bodyA: Body,
  pB: Vec2,
  bodyB: Body | null,
  relax: number,
  maxStep = Infinity
): void {
  const rA = sub(pA, bodyA.pos);
  let c = sub(pA, pB); // error to remove
  const e = len(c);
  if (e > maxStep) c = scale(c, maxStep / e);

  // Effective-mass matrix K = sum over bodies of invMass*I + invI*[r]x[r]x^T.
  let a = bodyA.invMass;
  let b = 0;
  let d = bodyA.invMass;
  a += bodyA.invInertia * rA.y * rA.y;
  b += -bodyA.invInertia * rA.x * rA.y;
  d += bodyA.invInertia * rA.x * rA.x;

  let rB: Vec2 | null = null;
  if (bodyB) {
    rB = sub(pB, bodyB.pos);
    a += bodyB.invMass + bodyB.invInertia * rB.y * rB.y;
    b += -bodyB.invInertia * rB.x * rB.y;
    d += bodyB.invMass + bodyB.invInertia * rB.x * rB.x;
  }

  const lambda = scale(solve2x2(a, b, d, c), relax);
  applyImpulse(bodyA, rA, lambda);
  if (bodyB && rB) applyImpulse(bodyB, rB, scale(lambda, -1));
}

/**
 * Apply a scalar positional impulse driving the constraint `C = c → 0` along a
 * unit direction `u` that is fixed in `bodyR`'s frame (so it rotates with it).
 * Couples `bodyQ` (feels +u at pQ) and `bodyR` (feels −u). The rail-body angular
 * Jacobian reduces to cross(u, pQ − posR); with `bodyR` grounded it becomes a
 * single-body line constraint.
 */
function solveAxis(pQ: Vec2, bodyQ: Body, u: Vec2, bodyR: Body, c: number, relax: number): void {
  const jQ = cross(sub(pQ, bodyQ.pos), u);
  const jR = cross(u, sub(pQ, bodyR.pos));
  const w =
    bodyQ.invMass + bodyQ.invInertia * jQ * jQ +
    bodyR.invMass + bodyR.invInertia * jR * jR;
  if (w < 1e-12) return;
  const lambda = (-c / w) * relax;
  bodyQ.pos = add(bodyQ.pos, scale(u, bodyQ.invMass * lambda));
  bodyQ.angle += bodyQ.invInertia * jQ * lambda;
  bodyR.pos = add(bodyR.pos, scale(u, -bodyR.invMass * lambda));
  bodyR.angle += bodyR.invInertia * jR * lambda;
}

/**
 * Slider/prismatic constraint with end-stops: keep `pQ` (a joint on `bodyQ`) on
 * the line through rail points `pA`,`pB` (two joints on `bodyR`) AND between them.
 * The perpendicular part holds it on the rail; the tangential part is one-sided —
 * it only activates once `pQ` passes an endpoint, clamping it back into the span.
 */
function solveSliderRail(
  pQ: Vec2,
  bodyQ: Body,
  pA: Vec2,
  pB: Vec2,
  bodyR: Body,
  relax: number
): void {
  const d = sub(pB, pA);
  const dl = len(d);
  if (dl < 1e-9) return; // degenerate rail (rail joints coincide)
  const dir = scale(d, 1 / dl);
  const n = perp(dir); // unit normal to the rail

  // Hold the rider on the rail line (n ⊥ dir, so this doesn't change the position along dir).
  solveAxis(pQ, bodyQ, n, bodyR, dot(sub(pQ, pA), n), relax);

  // Clamp the rider between the endpoints: only correct when it has overshot one.
  const s = dot(sub(pQ, pA), dir); // signed position along the rail from pA
  if (s < 0) solveAxis(pQ, bodyQ, dir, bodyR, s, relax);
  else if (s > dl) solveAxis(pQ, bodyQ, dir, bodyR, s - dl, relax);
}

/** One Gauss-Seidel sweep over the structural constraints (pin / ground / slider). */
function sweepStructural(scene: Scene, relax: number): void {
  for (const con of scene.constraints) {
    if (con.kind === "pin") {
      const ja = scene.getJoint(con.jointA);
      const jb = scene.getJoint(con.jointB);
      if (!ja || !jb) continue;
      const bodyA = scene.getBody(ja.bodyId)!;
      const bodyB = scene.getBody(jb.bodyId)!;
      solveCoincident(scene.jointWorld(ja), bodyA, scene.jointWorld(jb), bodyB, relax);
    } else if (con.kind === "ground") {
      const j = scene.getJoint(con.joint);
      if (!j) continue;
      const body = scene.getBody(j.bodyId)!;
      solveCoincident(scene.jointWorld(j), body, con.anchor, null, relax);
    } else if (con.kind === "slider") {
      const ja = scene.getJoint(con.railA);
      const jb = scene.getJoint(con.railB);
      if (!ja || !jb) continue;
      if (ja.bodyId !== jb.bodyId) continue; // rail joints must share a body
      const bodyR = scene.getBody(ja.bodyId)!;
      for (const riderId of con.riders) {
        const jq = scene.getJoint(riderId);
        if (!jq) continue;
        const bodyQ = scene.getBody(jq.bodyId)!;
        if (bodyQ.id === bodyR.id) continue; // rider on the rail body: nothing to do
        // Recompute the rail each rider, since a rider's reaction can move the rail body.
        solveSliderRail(
          scene.jointWorld(jq),
          bodyQ,
          scene.jointWorld(ja),
          scene.jointWorld(jb),
          bodyR,
          relax
        );
      }
    }
  }
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
function structuralResidual(scene: Scene): number {
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
  for (let iter = 0; iter < iterations; iter++) {
    sweepStructural(scene, relax);
    if (driver) {
      const j = scene.getJoint(driver.jointId);
      if (j) {
        const body = scene.getBody(j.bodyId)!;
        solveCoincident(scene.jointWorld(j), body, driver.target, null, 1, DRIVER_MAX_STEP);
      }
    }
  }

  // Converge structural constraints to tolerance; they take strict priority over
  // the driver. Stops early once the mechanism is tight enough.
  for (let i = 0; i < MAX_CLEANUP_SWEEPS; i++) {
    if (structuralResidual(scene) < STRUCTURAL_TOL) break;
    sweepStructural(scene, relax);
  }
}
