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
import { Vec2, add, rotate, cross, perp, scale, sub, len, lenSq, dot } from "./geometry";

/**
 * The mouse driver: pull a point toward `target`. The point is either an existing joint
 * (`jointId`) or an arbitrary location fixed in a body's frame (`bodyId` + `local`, the
 * centroid-offset in body coordinates) — the latter lets the user grab any part of a body,
 * not just a joint.
 */
export interface Driver {
  target: Vec2;
  jointId?: number;
  bodyId?: number;
  local?: Vec2;
}

/**
 * A constraint that couldn't be satisfied (the assembly is impossible): the two world
 * points that should coincide but can't, and the leftover gap between them. The UI draws
 * a red dotted line between `a` and `b` and flags the assembly as unsolvable. `joints` lists
 * the joint ids involved (pin endpoints, the grounded joint, an unreachable slider rider,
 * or the anchor's joint) so the renderer can mark them red.
 */
export interface ConstraintBreak {
  a: Vec2;
  b: Vec2;
  error: number;
  joints: number[];
}

/** Optional out-parameter for `solve` — populated with per-call counters useful for tuning. */
export interface SolveStats {
  /** Number of Phase-A sweeps actually run (≤ `iterations`; less when early-out kicked in). */
  phaseASweeps: number;
  /** Number of post-Phase-A cleanup sweeps run before residual fell below `structuralTol`. */
  cleanupSweeps: number;
  /** Worst constraint error in the final pose (tested against `structuralTol` and `breakTol`). */
  finalResidual: number;
}

/**
 * Live-tunable solver knobs. Held in a mutable object (rather than module-level constants) so the
 * UI's debug panel can adjust them at runtime without a rebuild — useful for feeling out the
 * convergence/error trade-off on real mechanisms.
 */
export const solverConfig = {
  /** Convergence target: keep sweeping until the worst constraint error is below this. */
  structuralTol: 1e-4,
  /** Final per-constraint error above which a constraint counts as a genuine "break". */
  breakTol: 1e-1,
  /** Safety cap on convergence sweeps so an over-constrained scene can't loop forever. */
  maxCleanupSweeps: 1000,
};

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

// --- permanent body groups (rigid composites) ------------------------------
/**
 * Per-solve view of the scene's permanent groups: every grouped body maps to its group's
 * (shared) member list and group id. Grouped bodies act as a **single rigid body**: any
 * impulse on a member is applied to the whole group about the combined centroid, so the
 * members never move relative to each other. Rebuilt at the top of each `solve` call
 * (module-level because it threads through every host helper; solve is not reentrant).
 */
interface GroupCtx {
  members: Map<number, Body[]>; // body id → the group's member bodies (shared array)
  groupIdOf: Map<number, number>; // body id → group id
}
let groupCtx: GroupCtx = { members: new Map(), groupIdOf: new Map() };

function buildGroupCtx(scene: Scene): GroupCtx {
  const ctx: GroupCtx = { members: new Map(), groupIdOf: new Map() };
  for (const g of scene.groups) {
    const bodies = g.bodyIds
      .map((id) => scene.getBody(id))
      .filter((b): b is Body => b !== undefined);
    if (bodies.length < 2) continue;
    for (const b of bodies) {
      ctx.members.set(b.id, bodies);
      ctx.groupIdOf.set(b.id, g.id);
    }
  }
  return ctx;
}

/** Whether two bodies are rigid to each other: the same body, or members of one group. */
function sameRigid(aBodyId: number | null, bBodyId: number | null): boolean {
  if (aBodyId === null || bBodyId === null) return false;
  if (aBodyId === bBodyId) return true;
  const ga = groupCtx.groupIdOf.get(aBodyId);
  return ga !== undefined && ga === groupCtx.groupIdOf.get(bBodyId);
}

/** Combined mass properties of a group, from the members' current poses. */
function groupProps(members: Body[]): { c: Vec2; invMass: number; invInertia: number } {
  let mass = 0;
  let cx = 0;
  let cy = 0;
  for (const b of members) {
    const m = 1 / b.invMass;
    mass += m;
    cx += m * b.pos.x;
    cy += m * b.pos.y;
  }
  const c = { x: cx / mass, y: cy / mass };
  let inertia = 0;
  for (const b of members) {
    // Parallel-axis: each member's inertia about its own centroid plus m·d² to the combined one.
    inertia += 1 / b.invInertia + (1 / b.invMass) * lenSq(sub(b.pos, c));
  }
  return { c, invMass: 1 / mass, invInertia: 1 / inertia };
}

/**
 * Apply impulse `imp` at world `point` to a whole group: translate all members together and
 * rotate them rigidly about the combined centroid (the group-rigid analogue of `bodyImpulse`).
 */
function groupImpulse(members: Body[], point: Vec2, imp: Vec2): void {
  const { c, invMass, invInertia } = groupProps(members);
  const dpos = scale(imp, invMass);
  const dang = invInertia * cross(sub(point, c), imp);
  for (const b of members) {
    b.pos = add(add(c, dpos), rotate(sub(b.pos, c), dang));
    b.angle += dang;
  }
}

/** Host acting at `point` on a body — or, when the body is grouped, on its whole group. */
function bodyHostAt(body: Body, point: Vec2): Host {
  const members = groupCtx.members.get(body.id);
  if (members) {
    const { c, invMass, invInertia } = groupProps(members);
    return {
      point,
      pos: c,
      invMass,
      invInertia,
      apply(imp) {
        groupImpulse(members, point, imp);
      },
    };
  }
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

/**
 * Resolve a joint to its solver host **for its own ground constraint**: a body-attached
 * point, or a free point. A grounded *free* joint is an immovable anchor; a grounded *body*
 * joint stays a body host here, so its ground constraint can lock the body's pose (two such
 * grounds on one body fix its rotation, which translation-only projection can't). Pins and
 * sliders go through `pinHostFor` instead, which treats any grounded joint as fixed.
 */
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
  return bodyHostAt(body, point); // a grouped body moves as its whole group
}

/** An immovable world point (a ground anchor or the mouse target). */
function fixedHost(p: Vec2): Host {
  return { point: p, pos: p, invMass: 0, invInertia: 0, apply() {} };
}

/**
 * Host for a joint as seen by a *pin, slider, or the mouse driver* — anything other than the
 * joint's own ground. A grounded joint of any kind (free, or sitting on a body) is an
 * immovable world point here: pinning to it pulls the *other* side onto the anchor and never
 * drags the body the joint belongs to. That body still pivots about the joint via its own
 * ground constraint, driven by its other joints. Ungrounded joints fall back to `hostFor`.
 */
function pinHostFor(scene: Scene, joint: Joint, grounded: Set<number>): Host {
  if (grounded.has(joint.id)) return fixedHost(scene.jointWorld(joint));
  return hostFor(scene, joint, grounded);
}

/**
 * The host the mouse driver pulls toward its target: an existing joint (via `pinHostFor`,
 * so a grounded joint stays put), or an arbitrary point fixed in a body's frame — a body
 * translate+rotate host at the grabbed local offset, so dragging anywhere on a body pivots
 * it like a joint would (grounds still win, being projected every sweep).
 */
function driverHost(scene: Scene, driver: Driver, grounded: Set<number>): Host | null {
  if (driver.jointId !== undefined) {
    const j = scene.getJoint(driver.jointId);
    return j ? pinHostFor(scene, j, grounded) : null;
  }
  if (driver.bodyId !== undefined && driver.local) {
    const body = scene.getBody(driver.bodyId);
    if (!body) return null;
    const point = add(body.pos, rotate(driver.local, body.angle));
    return bodyHostAt(body, point); // grabbing a grouped body drags the whole group
  }
  return null;
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
    const members = groupCtx.members.get(railBody.id);
    if (members) {
      // A grouped rail body reacts as its whole group (rigid composite).
      const { c, invMass, invInertia } = groupProps(members);
      return {
        pos: c,
        invMass,
        invInertia,
        applyAt: (pt, imp) => groupImpulse(members, pt, imp),
      };
    }
    return {
      pos: railBody.pos,
      invMass: railBody.invMass,
      invInertia: railBody.invInertia,
      applyAt: (pt, imp) => bodyImpulse(railBody, pt, imp),
    };
  }
  return { pos: fixedPos, invMass: 0, invInertia: 0, applyAt() {} };
}

/** Shared empty exclusion set (no constraints disabled). */
const NONE: ReadonlySet<number> = new Set<number>();
/** Shared empty temp-anchor map (no animation actuators active). */
const NO_ANCHORS: ReadonlyMap<number, Vec2> = new Map<number, Vec2>();

/**
 * One Gauss-Seidel sweep over the structural constraints (pin / ground / slider). `skip`
 * names "broken" units to leave disabled — a pin by its constraint id, a slider rider by
 * its joint id (ids are globally unique). Grounds are never skipped. `anchors` adds extra
 * world targets joint ids must meet (animation actuators / motors) — treated like grounds
 * during this sweep so the solver pulls everything else onto them.
 */
function sweepStructural(
  scene: Scene,
  relax: number,
  grounded: Set<number>,
  skip: ReadonlySet<number> = NONE,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS
): void {
  for (const con of scene.constraints) {
    if (con.kind === "pin") {
      if (skip.has(con.id)) continue;
      const ja = scene.getJoint(con.jointA);
      const jb = scene.getJoint(con.jointB);
      if (!ja || !jb) continue;
      // A pin between two bodies of one group is inert: the group is rigid, so the pin
      // can't change their relative pose (it would only fight the rigidity).
      if (sameRigid(ja.bodyId, jb.bodyId)) continue;
      solveCoincident(pinHostFor(scene, ja, grounded), pinHostFor(scene, jb, grounded), relax);
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
        if (skip.has(riderId)) continue;
        const jq = scene.getJoint(riderId);
        if (!jq) continue;
        if (railBody && sameRigid(jq.bodyId, railBody.id)) continue; // rider rigid to the rail: nothing to do
        // Recompute the rail each rider, since a rider's reaction can move the rail body.
        solveSliderRail(
          pinHostFor(scene, jq, grounded),
          scene.jointWorld(ja),
          scene.jointWorld(jb),
          railHostFor(railBody, scene.jointWorld(ja)),
          relax
        );
      }
    }
  }
  // Animation anchors get a coincident pull each sweep — same treatment as grounds (and
  // they're in `grounded`, so the projection below also snaps them exactly into place).
  if (anchors.size > 0) {
    for (const [jointId, target] of anchors) {
      const j = scene.getJoint(jointId);
      if (!j) continue;
      solveCoincident(hostFor(scene, j, grounded), fixedHost(target), relax);
    }
  }
  // Grounds get the last word every sweep, so no other constraint can drag a grounded
  // joint off its anchor — an impossible assembly leaves its error on the pins/sliders.
  projectGrounds(scene, anchors);
}

/**
 * Hard-project every ground (and every animation `anchor`): snap each grounded joint exactly
 * onto its anchor so it can never be moved. A grounded *free* joint is already an immovable
 * fixed host (we just keep it exact); a grounded *body* joint is restored by translating its
 * body so the joint lands on the anchor — pure translation preserves the angle, so combined
 * with the sweep's rotation impulse the body effectively pivots about the anchor (correct
 * revolute-to-ground motion) while the anchor stays put. A body carrying *several* grounds
 * is over-determined; we translate it by the average of the per-ground corrections, so
 * conflicting grounds settle deterministically at the midpoint instead of teleporting between
 * them each sweep. Animation anchors join the same averaging, so a motor's pivot+crank pair
 * (two anchors on one body) settles to the pose that satisfies both.
 */
function projectGrounds(scene: Scene, anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS): void {
  // Corrections are pooled per rigid unit: a lone body under its own key, a grouped body
  // under its group's key (the whole group translates together, keeping it rigid).
  const perUnit = new Map<string, { sum: Vec2; n: number; bodies: Body[] }>();
  const visit = (joint: Joint, anchor: Vec2): void => {
    if (joint.bodyId === null) {
      joint.local = { x: anchor.x, y: anchor.y };
      return;
    }
    const body = scene.getBody(joint.bodyId);
    if (!body) return;
    const world = add(body.pos, rotate(joint.local, body.angle));
    const corr = sub(anchor, world);
    const groupId = groupCtx.groupIdOf.get(body.id);
    const key = groupId !== undefined ? `g${groupId}` : `b${body.id}`;
    const e =
      perUnit.get(key) ??
      { sum: { x: 0, y: 0 }, n: 0, bodies: groupCtx.members.get(body.id) ?? [body] };
    perUnit.set(key, { sum: add(e.sum, corr), n: e.n + 1, bodies: e.bodies });
  };
  for (const con of scene.constraints) {
    if (con.kind !== "ground") continue;
    const j = scene.getJoint(con.joint);
    if (j) visit(j, con.anchor);
  }
  for (const [jointId, target] of anchors) {
    const j = scene.getJoint(jointId);
    if (j) visit(j, target);
  }
  for (const e of perUnit.values()) {
    const corr = scale(e.sum, 1 / e.n);
    for (const body of e.bodies) body.pos = add(body.pos, corr);
  }
}

/**
 * Joint ids that are fixed to a world point this solve — ground constraints plus any
 * animation `anchors` (actuator targets, motor pivot/crank). Used to treat them as
 * immovable hosts for pins/sliders/the driver, so nothing can drag them off-target.
 */
function groundedJoints(scene: Scene, anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS): Set<number> {
  const set = new Set<number>();
  for (const c of scene.constraints) if (c.kind === "ground") set.add(c.joint);
  for (const id of anchors.keys()) set.add(id);
  return set;
}

/**
 * Max distance (world units) the driven joint is pulled toward the cursor per
 * sweep. Small steps keep the linearized correction valid, so when the target
 * is unreachable the joint walks stably along its feasible path (e.g. the circle
 * around a ground point) instead of overshooting and spinning the body.
 */
const DRIVER_MAX_STEP = 8;
/**
 * Under-relaxation used to settle an over-constrained scene. Full-relaxation Gauss-Seidel
 * oscillates when the assembly can't be satisfied, so it never settles and leaves inflated,
 * arbitrary errors on *every* constraint — flagging joints that can actually be reached.
 * A gentle relaxation damps the oscillation so the scene converges to its true least-error
 * pose, where reachable constraints rest at ~0 and only the genuinely stuck ones remain.
 */
const STABILIZE_RELAX = 0.1;
/** Gentle relaxation for pulling disabled ("broken") units closed — small so they yield to
 *  the active constraints, using only the assembly's leftover freedom. */
const CLOSE_RELAX = 0.5;
/** Cap on gap-closing steps (each fully re-tightens the active set); stops early when stable. */
const CLOSE_SWEEPS = 200;
/** Stop closing once a step shrinks the worst gap by less than this (world units). */
const CLOSE_TOL = 0.05;

/**
 * A solvable "unit" of structural error: a pin, a ground, or one slider rider. `id` is the
 * pin/ground constraint id or the rider's joint id (globally unique). `a`/`b` are the two
 * world points that should coincide (for a rider, `b` is the nearest point on the rail
 * segment); `error` is their gap. `ground` marks the units that are sacred (never disabled).
 * `joints` lists the joint ids the unit involves, used to mark "stuck" joints in the UI.
 */
interface StructuralUnit {
  id: number;
  a: Vec2;
  b: Vec2;
  error: number;
  ground: boolean;
  joints: number[];
}

/**
 * Visit every structural unit (pin, ground, slider rider, animation anchor) with its current
 * error. Animation anchors count as `ground: true` units — they're sacred for the same reasons
 * a ground is (the user is asking the actuator/motor to put the joint exactly there).
 */
function eachUnit(
  scene: Scene,
  grounded: Set<number>,
  visit: (u: StructuralUnit) => void,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS
): void {
  for (const con of scene.constraints) {
    if (con.kind === "pin") {
      const ja = scene.getJoint(con.jointA);
      const jb = scene.getJoint(con.jointB);
      if (!ja || !jb) continue;
      if (sameRigid(ja.bodyId, jb.bodyId)) continue; // intra-group pin: inert, never an error
      const a = scene.jointWorld(ja);
      const b = scene.jointWorld(jb);
      visit({ id: con.id, a, b, error: len(sub(a, b)), ground: false, joints: [con.jointA, con.jointB] });
    } else if (con.kind === "ground") {
      const j = scene.getJoint(con.joint);
      if (!j) continue;
      const a = scene.jointWorld(j);
      visit({ id: con.id, a, b: con.anchor, error: len(sub(a, con.anchor)), ground: true, joints: [con.joint] });
    } else if (con.kind === "slider") {
      const ja = scene.getJoint(con.railA);
      const jb = scene.getJoint(con.railB);
      if (!ja || !jb || !railKind(ja, jb, grounded)) continue;
      const a0 = scene.jointWorld(ja);
      const d = sub(scene.jointWorld(jb), a0);
      const dl = len(d);
      if (dl < 1e-9) continue;
      const dir = scale(d, 1 / dl);
      for (const riderId of con.riders) {
        const jq = scene.getJoint(riderId);
        if (!jq) continue;
        if (sameRigid(jq.bodyId, ja.bodyId)) continue; // rider rigid to its rail: inert
        const q = scene.jointWorld(jq);
        const s = Math.max(0, Math.min(dl, dot(sub(q, a0), dir))); // nearest point on the rail segment
        const closest = add(a0, scale(dir, s));
        visit({ id: riderId, a: q, b: closest, error: len(sub(q, closest)), ground: false, joints: [riderId] });
      }
    }
  }
  // Animation anchors are sacred too: a joint that should sit exactly at a moving target.
  // Their ids may collide with a constraint id; that's fine — visit() callers key by id.
  for (const [jointId, target] of anchors) {
    const j = scene.getJoint(jointId);
    if (!j) continue;
    const a = scene.jointWorld(j);
    visit({ id: jointId, a, b: target, error: len(sub(a, target)), ground: true, joints: [jointId] });
  }
}

/** Largest positional error across the active (non-`skip`) structural units (world units). */
function structuralResidual(
  scene: Scene,
  grounded: Set<number>,
  skip: ReadonlySet<number> = NONE,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS
): number {
  let max = 0;
  eachUnit(scene, grounded, (u) => {
    if (!skip.has(u.id)) max = Math.max(max, u.error);
  }, anchors);
  return max;
}

/** The worst-violated active, non-ground unit (the next candidate to disable), or null. */
function worstActiveUnit(
  scene: Scene,
  grounded: Set<number>,
  skip: ReadonlySet<number>,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS
): { id: number; error: number } | null {
  let best: { id: number; error: number } | null = null;
  eachUnit(scene, grounded, (u) => {
    if (u.ground || skip.has(u.id)) return;
    if (!best || u.error > best.error) best = { id: u.id, error: u.error };
  }, anchors);
  return best;
}

/**
 * Settle the active (non-`skip`) constraints into a stable pose with under-relaxed sweeps,
 * stopping once the worst active error stops shrinking. Under-relaxation damps the
 * oscillation that full relaxation shows on an unconverged scene, so the active set reaches
 * its true least-error pose and the "worst active unit" reading is meaningful.
 */
function settle(
  scene: Scene,
  grounded: Set<number>,
  skip: ReadonlySet<number>,
  relax: number,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS
): void {
  let prev = Infinity;
  for (let i = 0; i < solverConfig.maxCleanupSweeps; i++) {
    const res = structuralResidual(scene, grounded, skip, anchors);
    if (res < solverConfig.structuralTol || prev - res < solverConfig.structuralTol) break;
    prev = res;
    sweepStructural(scene, relax, grounded, skip, anchors);
  }
}

/** Apply just the disabled ("broken") units, gently — used to pull broken gaps closed. */
function applyBroken(scene: Scene, grounded: Set<number>, broken: ReadonlySet<number>, relax: number): void {
  for (const con of scene.constraints) {
    if (con.kind === "pin") {
      if (!broken.has(con.id)) continue;
      const ja = scene.getJoint(con.jointA);
      const jb = scene.getJoint(con.jointB);
      if (ja && jb) solveCoincident(pinHostFor(scene, ja, grounded), pinHostFor(scene, jb, grounded), relax);
    } else if (con.kind === "slider") {
      const ja = scene.getJoint(con.railA);
      const jb = scene.getJoint(con.railB);
      if (!ja || !jb) continue;
      const kind = railKind(ja, jb, grounded);
      if (!kind) continue;
      const railBody = kind === "body" ? scene.getBody(ja.bodyId!)! : null;
      for (const riderId of con.riders) {
        if (!broken.has(riderId)) continue;
        const jq = scene.getJoint(riderId);
        if (!jq || (railBody && sameRigid(jq.bodyId, railBody.id))) continue;
        solveSliderRail(
          pinHostFor(scene, jq, grounded),
          scene.jointWorld(ja),
          scene.jointWorld(jb),
          railHostFor(railBody, scene.jointWorld(ja)),
          relax
        );
      }
    }
  }
}

/** Largest gap across the disabled ("broken") units (world units). */
function brokenResidual(
  scene: Scene,
  grounded: Set<number>,
  broken: ReadonlySet<number>,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS
): number {
  let max = 0;
  eachUnit(scene, grounded, (u) => {
    if (!u.ground && broken.has(u.id)) max = Math.max(max, u.error);
  }, anchors);
  return max;
}

/**
 * Pull the disabled ("broken") units as close together as the still-rigid assembly allows:
 * each step nudges them gently, then *fully* re-settles the active set + grounds so the
 * solved parts snap back to satisfied. The broken pull therefore only consumes the
 * assembly's leftover freedom (e.g. a body's free rotation about its ground), minimizing the
 * red-line gap without disturbing anything that's actually satisfied. Stops once the gap
 * stabilizes (the freedom is used up).
 */
function closeBroken(
  scene: Scene,
  grounded: Set<number>,
  broken: ReadonlySet<number>,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS
): void {
  if (broken.size === 0) return;
  let prev = Infinity;
  for (let i = 0; i < CLOSE_SWEEPS; i++) {
    applyBroken(scene, grounded, broken, CLOSE_RELAX);
    settle(scene, grounded, broken, 1, anchors); // re-tighten the active set + grounds + anchors to convergence
    const gap = brokenResidual(scene, grounded, broken, anchors);
    if (prev - gap < CLOSE_TOL) break; // freedom exhausted — gap won't shrink meaningfully further
    prev = gap;
  }
}

/**
 * Report the genuinely unsatisfiable constraints as red-line breaks: the disabled units
 * whose gap is still visible, plus any ground that can't be met (e.g. two grounds fighting
 * over one body — grounds are never disabled, so they surface here instead).
 */
function breaksForBroken(
  scene: Scene,
  grounded: Set<number>,
  broken: ReadonlySet<number>,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS
): ConstraintBreak[] {
  const breaks: ConstraintBreak[] = [];
  eachUnit(scene, grounded, (u) => {
    const isBreak = u.ground ? u.error > solverConfig.breakTol : broken.has(u.id) && u.error > solverConfig.breakTol;
    if (isBreak) breaks.push({ a: u.a, b: u.b, error: u.error, joints: u.joints });
  }, anchors);
  return breaks;
}

/**
 * Run the solver. `iterations` Gauss-Seidel sweeps over the structural
 * constraints (plus the optional soft mouse driver). `relax` under-relaxes the
 * structural corrections. Afterwards, structural-only sweeps keep running until
 * the worst constraint error falls below STRUCTURAL_TOL (or MAX_CLEANUP_SWEEPS
 * is hit). This adapts to mechanism complexity — simple scenes converge in a
 * couple of sweeps, closed loops get as many as they need — and makes the driver
 * yield to the structural constraints, so it can never break a pin/ground/slider.
 *
 * Returns the constraints that stay unsatisfied (an impossible assembly) — empty when
 * everything resolves. Grounds are sacred (never disabled); when the assembly is
 * over-constrained the solver disables only the genuinely unreachable pins/sliders and
 * reports them, leaving the rest correctly solved.
 */
export function solve(
  scene: Scene,
  driver: Driver | null,
  iterations = 100,
  relax = 1,
  anchors: ReadonlyMap<number, Vec2> = NO_ANCHORS,
  stats?: SolveStats
): ConstraintBreak[] {
  groupCtx = buildGroupCtx(scene); // permanent groups act as rigid composites this solve
  const grounded = groundedJoints(scene, anchors);
  // Initialise stats: callers that pass `stats` see these zeroed even on early return.
  if (stats) {
    stats.phaseASweeps = 0;
    stats.cleanupSweeps = 0;
    stats.finalResidual = 0;
  }

  // Phase A — driver + structural blended sweeps, bounded by `iterations` but early-exiting
  // once the structural residual is already within tolerance (so the slider acts as a maximum
  // on easy frames rather than a hard count). Driverless only: the driver is step-limited
  // (DRIVER_MAX_STEP per sweep), so a weakly-constrained grab (a lone free joint, an
  // unconstrained body) keeps residual ≈ 0 and an unconditional early-exit would cut the
  // driver to one pull per solve — the dragged point would crawl instead of tracking the
  // cursor. With a driver present the loop always runs the full budget, as it did before.
  let phaseASweeps = 0;
  for (let iter = 0; iter < iterations; iter++) {
    sweepStructural(scene, relax, grounded, NONE, anchors);
    if (driver) {
      const host = driverHost(scene, driver, grounded);
      if (host) solveCoincident(host, fixedHost(driver.target), 1, DRIVER_MAX_STEP);
    }
    phaseASweeps++;
    if (!driver && structuralResidual(scene, grounded, NONE, anchors) < solverConfig.structuralTol) break;
  }
  let cleanupSweepsDone = 0;
  for (let i = 0; i < solverConfig.maxCleanupSweeps; i++) {
    if (structuralResidual(scene, grounded, NONE, anchors) < solverConfig.structuralTol) break;
    sweepStructural(scene, relax, grounded, NONE, anchors);
    cleanupSweepsDone++;
  }
  const postCleanupResidual = structuralResidual(scene, grounded, NONE, anchors);
  if (stats) {
    stats.phaseASweeps = phaseASweeps;
    stats.cleanupSweeps = cleanupSweepsDone;
    stats.finalResidual = postCleanupResidual;
  }
  if (postCleanupResidual < solverConfig.structuralTol) return []; // everything resolved

  // Phase B — over-constrained. Greedily disable the worst-violated non-ground unit and
  // re-settle, until the remaining (active) constraints can all be satisfied. Grounds and
  // animation anchors are never disabled, so the disabled units are exactly the pins/sliders
  // that can't be met (anchors join grounds as "sacred").
  const broken = new Set<number>();
  for (let guard = 0; guard <= scene.constraints.length; guard++) {
    settle(scene, grounded, broken, STABILIZE_RELAX, anchors);
    const worst = worstActiveUnit(scene, grounded, broken, anchors);
    if (!worst || worst.error < solverConfig.breakTol) break;
    broken.add(worst.id);
  }

  // Phase C — pull the disabled units as close as the now-rigid assembly allows, then report
  // whatever gap remains as a red-line break.
  closeBroken(scene, grounded, broken, anchors);
  if (stats) stats.finalResidual = structuralResidual(scene, grounded, NONE, anchors);
  return breaksForBroken(scene, grounded, broken, anchors);
}
