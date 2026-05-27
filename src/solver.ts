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
import { Vec2, add, cross, perp, scale, sub, normalize, len } from "./geometry";

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

/** Keep world point `pA` on `bodyA` on the infinite line (origin + unit dir). */
function solveOnLine(pA: Vec2, bodyA: Body, origin: Vec2, dir: Vec2, relax: number): void {
  const n = normalize(perp(dir)); // line normal
  const rA = sub(pA, bodyA.pos);
  const c = cross(sub(pA, origin), dir); // signed perpendicular distance
  const rn = cross(rA, n);
  const w = bodyA.invMass + bodyA.invInertia * rn * rn;
  if (w < 1e-12) return;
  // δc = -λ_s·w, so λ_s = c/w drives the perpendicular distance c to zero.
  const lambda = scale(n, (c / w) * relax);
  applyImpulse(bodyA, rA, lambda);
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
      const j = scene.getJoint(con.joint);
      if (!j) continue;
      const body = scene.getBody(j.bodyId)!;
      solveOnLine(scene.jointWorld(j), body, con.origin, con.dir, relax);
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
/** Structural-only sweeps run after driving, guaranteeing hard constraints hold exactly. */
const CLEANUP_SWEEPS = 30;

/**
 * Run the solver. `iterations` Gauss-Seidel sweeps over the structural
 * constraints (plus the optional soft mouse driver). `relax` under-relaxes the
 * structural corrections. After driving, extra structural-only sweeps ensure
 * ground/pin/slider constraints are satisfied exactly — the driver yields to
 * them, so it can never drag a ground point (or break a pin/slider) away.
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

  // Structural constraints take strict priority over the driver.
  if (driver) {
    for (let i = 0; i < CLEANUP_SWEEPS; i++) sweepStructural(scene, relax);
  }
}
