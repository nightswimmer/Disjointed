/**
 * Sketch solver: CAD-style draw-mode constraints (coincident / horizontal / vertical /
 * parallel / perpendicular / equal) plus driving dimensions. Iterative projection
 * (Gauss-Seidel, the same philosophy as solver.ts) — but where the mechanism solver
 * moves rigid poses, this one moves *shape*: the variables are the world positions of
 * body control vertices and joints. After a converged solve the new positions are
 * applied through the scene's edit paths (moveBodyVertex / moveJoint), so bodies
 * rebuild, attached joints stay anchored, and the node ↔ joint link keeps working.
 *
 * Failure semantics are **reject**: an unsatisfiable solve leaves the scene untouched
 * and returns the offending items as `SketchBreak`s (the UI flashes them red).
 */
import {
  Scene,
  SketchConstraint,
  Measurement,
  MeasureRef,
  VERTEX_LINK_EPS,
} from "./model";
import { Vec2, vec, add, sub, scale, dist, len, dot, perp, rotate } from "./geometry";

/** An unsatisfied sketch item after a failed solve: a constraint or a driving dimension. */
export interface SketchBreak {
  id: number;
  kind: "constraint" | "dimension";
  /** Residual in world units (Infinity for an item the solver can't act on at all). */
  error: number;
}

/** Runtime-tunable solver parameters (mirrors solverConfig in solver.ts). */
export const sketchConfig = {
  /** A solve converges when every item's residual is below this (world units). */
  tol: 1e-3,
  /** Gauss-Seidel sweep budget per solve. */
  maxSweeps: 400,
};

// --- variables --------------------------------------------------------------

/**
 * A solver variable is one movable world point: a body control vertex (`v:body:index`),
 * a joint (`j:id`), or a guideline defining point (`g:id:a` / `g:id:b`). A joint
 * coincident with one of its body's control vertices is *linked* to it (see
 * VERTEX_LINK_EPS in model.ts), so its refs map onto the vertex variable — the
 * constraint then drives the shape, exactly like dragging the joint does.
 */
interface System {
  keys: string[];
  pos: Vec2[];
  index: Map<string, number>;
  /**
   * Per-variable mobility rank: 0 = construction (guideline defining points),
   * 1 = geometry (body control vertices, joints), 2 = pinned by the active drag.
   * Corrections always flow to the **lowest** rank in a pair — so guide constraints
   * are satisfied by moving free guide points, never by moving joints or body nodes,
   * and a drag is never tugged back by its constraints. Equal ranks split evenly.
   */
  rank: number[];
  anchorSet?: ReadonlySet<string>;
}

/** Mobility rank of a variable (see System.rank). */
function varRank(key: string, anchored: boolean): number {
  if (anchored) return 2;
  return key.startsWith("g:") ? 0 : 1;
}

interface SolveItem {
  id: number;
  kind: "constraint" | "dimension";
  /** Residual before correction; applies the correction when `apply` is true. */
  run(pos: Vec2[], apply: boolean): number;
}

function vertexKey(bodyId: number, index: number): string {
  return `v:${bodyId}:${index}`;
}

/** Variable key for a point ref, or null when the ref can't be a solver variable. */
function pointVarKey(scene: Scene, ref: MeasureRef): string | null {
  if (ref.kind === "vertex") {
    const b = scene.getBody(ref.bodyId);
    return b && ref.index >= 0 && ref.index < b.controlLocal.length
      ? vertexKey(ref.bodyId, ref.index)
      : null;
  }
  if (ref.kind === "joint") {
    const j = scene.getJoint(ref.jointId);
    if (!j) return null;
    if (j.bodyId !== null) {
      const body = scene.getBody(j.bodyId);
      if (!body) return null;
      const w = scene.jointWorld(j);
      const ctrl = scene.bodyControlWorld(body);
      for (let i = 0; i < ctrl.length; i++) {
        if (dist(ctrl[i], w) < VERTEX_LINK_EPS) return vertexKey(body.id, i);
      }
    }
    return `j:${ref.jointId}`;
  }
  if (ref.kind === "guidePoint") {
    return scene.getGuide(ref.guideId) ? `g:${ref.guideId}:${ref.which}` : null;
  }
  return null; // bodyPoint refs are measurement-only; line refs aren't points
}

/** Variable keys for a line ref's two endpoints, or null. */
function lineVarKeys(scene: Scene, ref: MeasureRef): [string, string] | null {
  if (ref.kind === "edge") {
    const b = scene.getBody(ref.bodyId);
    if (!b || ref.index < 0 || ref.index >= b.controlLocal.length) return null;
    return [
      vertexKey(ref.bodyId, ref.index),
      vertexKey(ref.bodyId, (ref.index + 1) % b.controlLocal.length),
    ];
  }
  if (ref.kind === "rail") {
    const c = scene.constraints.find((x) => x.id === ref.sliderId && x.kind === "slider");
    if (!c || c.kind !== "slider") return null;
    const a = pointVarKey(scene, { kind: "joint", jointId: c.railA });
    const b = pointVarKey(scene, { kind: "joint", jointId: c.railB });
    return a && b ? [a, b] : null;
  }
  if (ref.kind === "guideLine") {
    return scene.getGuide(ref.guideId)
      ? [`g:${ref.guideId}:a`, `g:${ref.guideId}:b`]
      : null;
  }
  return null;
}

/** Current world position of a variable. */
function varWorld(scene: Scene, key: string): Vec2 | null {
  const parts = key.split(":");
  if (parts[0] === "v") {
    const body = scene.getBody(Number(parts[1]));
    const index = Number(parts[2]);
    if (!body || index < 0 || index >= body.controlLocal.length) return null;
    return scene.bodyControlWorld(body)[index];
  }
  if (parts[0] === "g") {
    const g = scene.getGuide(Number(parts[1]));
    return g ? vec(g[parts[2] as "a" | "b"].x, g[parts[2] as "a" | "b"].y) : null;
  }
  const j = scene.getJoint(Number(parts[1]));
  return j ? scene.jointWorld(j) : null;
}

function acquire(scene: Scene, sys: System, key: string): number | null {
  const existing = sys.index.get(key);
  if (existing !== undefined) return existing;
  const w = varWorld(scene, key);
  if (!w) return null;
  sys.index.set(key, sys.keys.length);
  sys.keys.push(key);
  sys.pos.push(vec(w.x, w.y));
  sys.rank.push(varRank(key, sys.anchorSet?.has(key) ?? false));
  return sys.keys.length - 1;
}

/**
 * Fraction of a pairwise correction the *first* participant absorbs: equal ranks
 * split evenly; otherwise the lower-ranked (more mobile) side takes the whole
 * correction — construction yields to geometry, everything yields to the drag.
 */
function shareOf(rankA: number, rankB: number): number {
  if (rankA === rankB) return 0.5;
  return rankA < rankB ? 1 : 0;
}

// --- projections -------------------------------------------------------------

const EPS = 1e-9;

/** Wrap an angle difference into (-π/2, π/2] — direction mismatch modulo a half-turn. */
function wrapHalfPi(a: number): number {
  let d = ((a % Math.PI) + Math.PI) % Math.PI;
  if (d > Math.PI / 2) d -= Math.PI;
  return d;
}

function rotateAboutMid(pos: Vec2[], i: number, j: number, ang: number): void {
  const mid = scale(add(pos[i], pos[j]), 0.5);
  pos[i] = add(mid, rotate(sub(pos[i], mid), ang));
  pos[j] = add(mid, rotate(sub(pos[j], mid), ang));
}

/** Rotate both lines toward a common direction; returns the displacement-scale residual.
 *  `w1` is the fraction of the misalignment line 1 absorbs (0.5 = even split; 0 = only
 *  line 2 rotates — line 1 is anchored by the active drag). */
function projectParallel(
  pos: Vec2[],
  l1: [number, number],
  l2: [number, number],
  offset: number,
  apply: boolean,
  w1 = 0.5
): number {
  const d1 = sub(pos[l1[1]], pos[l1[0]]);
  const d2 = sub(pos[l2[1]], pos[l2[0]]);
  const n1 = len(d1);
  const n2 = len(d2);
  if (n1 < EPS || n2 < EPS) return 0; // degenerate line: nothing to align
  const dd = wrapHalfPi(Math.atan2(d2.y, d2.x) - Math.atan2(d1.y, d1.x) - offset);
  const err = Math.abs(Math.sin(dd)) * (Math.max(n1, n2) / 2);
  if (apply && Math.abs(dd) > EPS) {
    if (w1 > EPS) rotateAboutMid(pos, l1[0], l1[1], dd * w1);
    if (1 - w1 > EPS) rotateAboutMid(pos, l2[0], l2[1], -dd * (1 - w1));
  }
  return err;
}

// --- system building ----------------------------------------------------------

/** A driving dimension to solve for: an existing measurement at a (possibly new) target. */
interface DimSpec {
  m: Measurement;
  target: number;
}

interface BuildResult {
  sys: System;
  items: SolveItem[];
  /** Items the solver cannot act on (unsupported refs, degenerate variables). */
  invalid: SketchBreak[];
}

function buildConstraintItem(
  scene: Scene,
  sys: System,
  c: SketchConstraint
): SolveItem | null | "invalid" {
  const kind = c.kind;
  if (kind === "coincident" || ((kind === "horizontal" || kind === "vertical") && c.refB)) {
    const ka = pointVarKey(scene, c.refA);
    const kb = c.refB ? pointVarKey(scene, c.refB) : null;
    if (!ka || !kb) return "invalid";
    const i = acquire(scene, sys, ka);
    const j = acquire(scene, sys, kb);
    if (i === null || j === null) return "invalid";
    if (i === j) return null; // same variable: trivially satisfied
    const wp = shareOf(sys.rank[i], sys.rank[j]); // fraction i absorbs
    if (kind === "coincident") {
      return {
        id: c.id,
        kind: "constraint",
        run(pos, apply) {
          const err = dist(pos[i], pos[j]) / 2;
          if (apply) {
            // Weighted meeting point: an anchored side stays put, the free side comes to it.
            const m = add(scale(pos[i], 1 - wp), scale(pos[j], wp));
            pos[i] = vec(m.x, m.y);
            pos[j] = vec(m.x, m.y);
          }
          return err;
        },
      };
    }
    const axis: "x" | "y" = kind === "horizontal" ? "y" : "x";
    return {
      id: c.id,
      kind: "constraint",
      run(pos, apply) {
        const err = Math.abs(pos[i][axis] - pos[j][axis]) / 2;
        if (apply) {
          const m = pos[i][axis] * (1 - wp) + pos[j][axis] * wp;
          pos[i][axis] = m;
          pos[j][axis] = m;
        }
        return err;
      },
    };
  }
  if (kind === "horizontal" || kind === "vertical") {
    const keys = lineVarKeys(scene, c.refA);
    if (!keys) return "invalid";
    const i = acquire(scene, sys, keys[0]);
    const j = acquire(scene, sys, keys[1]);
    if (i === null || j === null) return "invalid";
    if (i === j) return null;
    const wl = shareOf(sys.rank[i], sys.rank[j]);
    const axis: "x" | "y" = kind === "horizontal" ? "y" : "x";
    return {
      id: c.id,
      kind: "constraint",
      run(pos, apply) {
        const err = Math.abs(pos[i][axis] - pos[j][axis]) / 2;
        if (apply) {
          const m = pos[i][axis] * (1 - wl) + pos[j][axis] * wl;
          pos[i][axis] = m;
          pos[j][axis] = m;
        }
        return err;
      },
    };
  }
  // parallel / perpendicular / equal: two lines
  const ka = lineVarKeys(scene, c.refA);
  const kb = c.refB ? lineVarKeys(scene, c.refB) : null;
  if (!ka || !kb) return "invalid";
  const a0 = acquire(scene, sys, ka[0]);
  const a1 = acquire(scene, sys, ka[1]);
  const b0 = acquire(scene, sys, kb[0]);
  const b1 = acquire(scene, sys, kb[1]);
  if (a0 === null || a1 === null || b0 === null || b1 === null) return "invalid";
  // A line is as mobile as its most mobile endpoint (it can rotate/shift through it).
  const wLine = shareOf(
    Math.min(sys.rank[a0], sys.rank[a1]),
    Math.min(sys.rank[b0], sys.rank[b1])
  );
  if (kind === "equal") {
    return {
      id: c.id,
      kind: "constraint",
      run(pos, apply) {
        const n1 = dist(pos[a0], pos[a1]);
        const n2 = dist(pos[b0], pos[b1]);
        const err = Math.abs(n1 - n2) / 2;
        if (apply && n1 > EPS && n2 > EPS) {
          const target = n1 * (1 - wLine) + n2 * wLine; // anchored line keeps its length
          const scaleLine = (i: number, j: number, from: number) => {
            const mid = scale(add(pos[i], pos[j]), 0.5);
            const f = target / from;
            pos[i] = add(mid, scale(sub(pos[i], mid), f));
            pos[j] = add(mid, scale(sub(pos[j], mid), f));
          };
          scaleLine(a0, a1, n1);
          scaleLine(b0, b1, n2);
        }
        return err;
      },
    };
  }
  const offset = kind === "perpendicular" ? Math.PI / 2 : 0;
  return {
    id: c.id,
    kind: "constraint",
    run: (pos, apply) => projectParallel(pos, [a0, a1], [b0, b1], offset, apply, wLine),
  };
}

function buildDimensionItem(
  scene: Scene,
  sys: System,
  spec: DimSpec
): SolveItem | "invalid" {
  const { m, target } = spec;
  const isLine = (r: MeasureRef) => r.kind === "rail" || r.kind === "edge" || r.kind === "guideLine";
  const aLine = isLine(m.refA);
  const bLine = isLine(m.refB);
  if (!aLine && !bLine) {
    const ka = pointVarKey(scene, m.refA);
    const kb = pointVarKey(scene, m.refB);
    if (!ka || !kb) return "invalid";
    const i = acquire(scene, sys, ka);
    const j = acquire(scene, sys, kb);
    if (i === null || j === null || i === j) return "invalid";
    const wi = shareOf(sys.rank[i], sys.rank[j]); // fraction i absorbs
    if (m.axis === "h" || m.axis === "v") {
      const axis: "x" | "y" = m.axis === "h" ? "x" : "y";
      return {
        id: m.id,
        kind: "dimension",
        run(pos, apply) {
          const d = pos[j][axis] - pos[i][axis];
          const s = d === 0 ? 1 : Math.sign(d);
          const err = target - Math.abs(d);
          if (apply) {
            pos[j][axis] += s * err * (1 - wi);
            pos[i][axis] -= s * err * wi;
          }
          return Math.abs(err);
        },
      };
    }
    return {
      id: m.id,
      kind: "dimension",
      run(pos, apply) {
        const d = sub(pos[j], pos[i]);
        const l = len(d);
        const u = l > EPS ? scale(d, 1 / l) : vec(1, 0);
        const err = target - l;
        if (apply) {
          pos[j] = add(pos[j], scale(u, err * (1 - wi)));
          pos[i] = sub(pos[i], scale(u, err * wi));
        }
        return Math.abs(err);
      },
    };
  }
  if (aLine && bLine) {
    const ka = lineVarKeys(scene, m.refA);
    const kb = lineVarKeys(scene, m.refB);
    if (!ka || !kb) return "invalid";
    const a0 = acquire(scene, sys, ka[0]);
    const a1 = acquire(scene, sys, ka[1]);
    const b0 = acquire(scene, sys, kb[0]);
    const b1 = acquire(scene, sys, kb[1]);
    if (a0 === null || a1 === null || b0 === null || b1 === null) return "invalid";
    const wA = shareOf(
      Math.min(sys.rank[a0], sys.rank[a1]),
      Math.min(sys.rank[b0], sys.rank[b1])
    ); // fraction line A absorbs
    // A driving line–line distance implies the pair is parallel (CAD convention):
    // align the directions, then set the gap along the common normal.
    return {
      id: m.id,
      kind: "dimension",
      run(pos, apply) {
        const alignErr = projectParallel(pos, [a0, a1], [b0, b1], 0, apply, wA);
        const d1 = sub(pos[a1], pos[a0]);
        const n1 = len(d1);
        if (n1 < EPS) return alignErr;
        const n = perp(scale(d1, 1 / n1));
        const m1 = scale(add(pos[a0], pos[a1]), 0.5);
        const m2 = scale(add(pos[b0], pos[b1]), 0.5);
        const s = dot(sub(m2, m1), n);
        const sg = s === 0 ? 1 : Math.sign(s);
        const err = sg * target - s;
        if (apply) {
          const shiftB = scale(n, err * (1 - wA));
          const shiftA = scale(n, err * wA);
          pos[b0] = add(pos[b0], shiftB);
          pos[b1] = add(pos[b1], shiftB);
          pos[a0] = sub(pos[a0], shiftA);
          pos[a1] = sub(pos[a1], shiftA);
        }
        return Math.max(alignErr, Math.abs(err));
      },
    };
  }
  // point + line: perpendicular distance to the infinite line, keeping the point's side.
  const pRef = aLine ? m.refB : m.refA;
  const lRef = aLine ? m.refA : m.refB;
  const kp = pointVarKey(scene, pRef);
  const kl = lineVarKeys(scene, lRef);
  if (!kp || !kl) return "invalid";
  const p = acquire(scene, sys, kp);
  const l0 = acquire(scene, sys, kl[0]);
  const l1 = acquire(scene, sys, kl[1]);
  if (p === null || l0 === null || l1 === null || p === l0 || p === l1) return "invalid";
  const wp = shareOf(sys.rank[p], Math.min(sys.rank[l0], sys.rank[l1])); // fraction the point absorbs
  return {
    id: m.id,
    kind: "dimension",
    run(pos, apply) {
      const d = sub(pos[l1], pos[l0]);
      const l = len(d);
      if (l < EPS) return target; // degenerate line: can't measure, full residual
      const n = perp(scale(d, 1 / l));
      const s = dot(sub(pos[p], pos[l0]), n);
      const sg = s === 0 ? 1 : Math.sign(s);
      const err = sg * target - s;
      if (apply) {
        pos[p] = add(pos[p], scale(n, err * wp));
        const shift = scale(n, err * (1 - wp));
        pos[l0] = sub(pos[l0], shift);
        pos[l1] = sub(pos[l1], shift);
      }
      return Math.abs(err);
    },
  };
}

/**
 * Gather every draw-mode sketch constraint and driving dimension into a solvable
 * system. `override` replaces (or adds) one dimension's target — the candidate edit.
 * `anchors` marks the variables pinned by an active drag (see System.anchored).
 */
function buildSystem(scene: Scene, override?: DimSpec, anchors?: ReadonlySet<string>): BuildResult {
  const sys: System = { keys: [], pos: [], index: new Map(), rank: [], anchorSet: anchors };
  const items: SolveItem[] = [];
  const invalid: SketchBreak[] = [];
  for (const c of scene.sketch) {
    const item = buildConstraintItem(scene, sys, c);
    if (item === "invalid") invalid.push({ id: c.id, kind: "constraint", error: Infinity });
    else if (item) items.push(item);
  }
  const dims: DimSpec[] = scene.measurements
    .filter((m) => m.mode === "draw" && m.driving && m.target !== undefined)
    .filter((m) => !override || m.id !== override.m.id)
    .map((m) => ({ m, target: m.target! }));
  if (override) dims.push(override);
  for (const spec of dims) {
    const item = buildDimensionItem(scene, sys, spec);
    if (item === "invalid") invalid.push({ id: spec.m.id, kind: "dimension", error: Infinity });
    else items.push(item);
  }
  return { sys, items, invalid };
}

/** Run Gauss-Seidel sweeps until every residual is under tolerance (or the budget runs out). */
function iterate(sys: System, items: SolveItem[]): boolean {
  for (let sweep = 0; sweep < sketchConfig.maxSweeps; sweep++) {
    let maxErr = 0;
    for (const item of items) maxErr = Math.max(maxErr, item.run(sys.pos, true));
    if (maxErr < sketchConfig.tol) return true;
  }
  return items.every((item) => item.run(sys.pos, false) < sketchConfig.tol);
}

/** Residuals of every item against the solved (or live) positions, as breaks. */
function residualBreaks(sys: System, items: SolveItem[]): SketchBreak[] {
  const out: SketchBreak[] = [];
  for (const item of items) {
    const err = item.run(sys.pos, false);
    if (err >= sketchConfig.tol) out.push({ id: item.id, kind: item.kind, error: err });
  }
  return out;
}

/**
 * Write the solved positions back through the scene's edit paths: control vertices
 * first (bodies reshape; linked joints are carried), then joints (attached joints are
 * clamped into their — already reshaped — bodies; ground anchors follow).
 */
function applySystem(scene: Scene, sys: System): void {
  const order = sys.keys
    .map((key, i) => ({ key, i }))
    .sort((a, b) => Number(b.key.startsWith("v")) - Number(a.key.startsWith("v")));
  for (const { key, i } of order) {
    const cur = varWorld(scene, key);
    if (!cur) continue;
    const delta = sub(sys.pos[i], cur);
    if (len(delta) < EPS) continue;
    const parts = key.split(":");
    if (parts[0] === "v") scene.moveBodyVertex(Number(parts[1]), Number(parts[2]), delta);
    else if (parts[0] === "g") {
      scene.moveGuidePoint(Number(parts[1]), parts[2] as "a" | "b", sys.pos[i]);
    } else scene.moveJoint(Number(parts[1]), delta);
  }
}

/** Deep plain-data snapshot for reject-and-revert. */
function snapshot(scene: Scene): string {
  return JSON.stringify(scene.serialize());
}

function restore(scene: Scene, snap: string): void {
  scene.load(JSON.parse(snap));
}

/**
 * Solve, apply, and verify the sketch system (optionally with one dimension's target
 * overridden — the candidate edit). On success returns []; on failure the scene is
 * left exactly as it was (unconverged solves never touch it; a verification failure
 * after applying — e.g. joint containment clamped a solved position away — reverts).
 */
function solveAndApply(scene: Scene, override?: DimSpec, anchors?: ReadonlySet<string>): SketchBreak[] {
  const build = buildSystem(scene, override, anchors);
  if (build.invalid.length) return build.invalid;
  if (!build.items.length) return [];
  if (!iterate(build.sys, build.items)) {
    return residualBreaks(build.sys, build.items);
  }
  const snap = snapshot(scene);
  applySystem(scene, build.sys);
  // Re-measure from the actual scene: the edit paths may have adjusted positions
  // (containment clamps), so verify the applied state truly satisfies everything.
  const after = buildSystem(scene, override, anchors);
  const bad = residualBreaks(after.sys, after.items).concat(after.invalid);
  if (bad.length) {
    restore(scene, snap);
    return bad;
  }
  return [];
}

/**
 * Re-solve every sketch constraint + driving dimension from the current geometry and
 * apply the result. Returns [] on success; on failure the scene is untouched and the
 * unsatisfiable items are returned. Call after edits that may have violated the sketch.
 *
 * `anchors` (optional) names the drag-pinned variables — pass the keys from the
 * `anchorVars*` helpers while live-solving during a drag, so the dragged geometry is
 * never tugged back by its constraints (free elements follow it instead). Static
 * solves omit it and stay fully symmetric.
 */
export function solveSketch(scene: Scene, anchors?: ReadonlySet<string>): SketchBreak[] {
  return solveAndApply(scene, undefined, anchors);
}

// --- drag anchoring ------------------------------------------------------------

/** Anchor keys pinning a whole body's shape: every control vertex + every joint on it. */
export function anchorVarsForBody(scene: Scene, bodyId: number): string[] {
  const body = scene.getBody(bodyId);
  if (!body) return [];
  const keys = body.controlLocal.map((_, i) => vertexKey(bodyId, i));
  for (const j of scene.joints) {
    if (j.bodyId !== bodyId) continue;
    const k = pointVarKey(scene, { kind: "joint", jointId: j.id });
    if (k) keys.push(k);
  }
  return keys;
}

/** Anchor key(s) pinning one joint (resolves to its linked vertex variable if stuck). */
export function anchorVarsForJoint(scene: Scene, jointId: number): string[] {
  const k = pointVarKey(scene, { kind: "joint", jointId });
  return k ? [k] : [];
}

/** Anchor keys pinning a whole guideline (both defining points). */
export function anchorVarsForGuide(guideId: number): string[] {
  return [`g:${guideId}:a`, `g:${guideId}:b`];
}

/** Anchor key pinning one guideline defining point. */
export function anchorVarForGuidePoint(guideId: number, which: "a" | "b"): string {
  return `g:${guideId}:${which}`;
}

/** Anchor key pinning one body control vertex. */
export function anchorVarForVertex(bodyId: number, index: number): string {
  return vertexKey(bodyId, index);
}

/**
 * Add a sketch constraint and immediately solve for it (geometry moves to satisfy it).
 * If the solve is unsatisfiable the constraint is removed again and the scene left
 * untouched (reject semantics): `constraint` is null and `breaks` names the conflicts.
 * `constraint` is also null for an invalid reference combination (with no breaks).
 */
export function tryAddConstraint(
  scene: Scene,
  kind: SketchConstraint["kind"],
  refA: MeasureRef,
  refB?: MeasureRef
): { constraint: SketchConstraint | null; breaks: SketchBreak[] } {
  const c = scene.addSketchConstraint(kind, refA, refB);
  if (!c) return { constraint: null, breaks: [] };
  const breaks = solveSketch(scene);
  if (breaks.length) {
    scene.removeSketchConstraint(c.id);
    return { constraint: null, breaks };
  }
  return { constraint: c, breaks: [] };
}

/** Auto-constraint threshold: an edge within this angle of horizontal/vertical gets H/V. */
export const AUTO_HV_TOL = (5 * Math.PI) / 180;

/**
 * Auto-constraints for a freshly drawn freehand body: every control edge within
 * `tol` of horizontal or vertical gets the matching H/V constraint (solved in as it's
 * added, so the edge snaps exactly straight). A constraint the sketch can't satisfy is
 * skipped. Returns the constraints that stuck.
 */
export function autoConstrainBody(
  scene: Scene,
  bodyId: number,
  tol = AUTO_HV_TOL
): SketchConstraint[] {
  const body = scene.getBody(bodyId);
  if (!body) return [];
  const out: SketchConstraint[] = [];
  for (let i = 0; i < body.controlLocal.length; i++) {
    const verts = scene.bodyControlWorld(body); // re-read: earlier edges may have snapped
    const a = verts[i];
    const b = verts[(i + 1) % verts.length];
    const d = sub(b, a);
    if (len(d) < EPS) continue;
    const ang = Math.abs(Math.atan2(d.y, d.x)); // 0..π
    const kind =
      ang < tol || Math.PI - ang < tol
        ? ("horizontal" as const)
        : Math.abs(ang - Math.PI / 2) < tol
        ? ("vertical" as const)
        : null;
    if (!kind) continue;
    const { constraint } = tryAddConstraint(scene, kind, { kind: "edge", bodyId, index: i });
    if (constraint) out.push(constraint);
  }
  return out;
}

/**
 * Set a draw-mode distance dimension to drive `target` (world units, > 0), moving
 * geometry to satisfy it:
 *
 * - If the dimension lives entirely on one body that has **no other driving dimensions
 *   and no sketch constraints to anything outside it** (internal constraints are
 *   scale-invariant, so they're fine), the whole body **scales uniformly about its
 *   centroid** — same form factor, first-dimension CAD behaviour.
 * - Otherwise the sketch solver moves only the involved nodes, holding every other
 *   constraint and driving dimension satisfied.
 *
 * On success the dimension is marked driving at `target` and [] is returned. On an
 * unsatisfiable edit the scene **and** the dimension are left untouched and the
 * conflicting items are returned (reject semantics).
 */
export function applyDrivingDimension(
  scene: Scene,
  measurementId: number,
  target: number
): SketchBreak[] {
  const m = scene.getMeasurement(measurementId);
  const reject = [{ id: measurementId, kind: "dimension" as const, error: Infinity }];
  if (!m || m.mode !== "draw" || !(target > 0)) return reject;
  const info = scene.measureInfo(m);
  if (!info || info.kind !== "distance") return reject; // angle dimensions can't drive (v1)
  const body = scaleEligibleBody(scene, m);
  if (body !== null && info.value > EPS) {
    const snap = snapshot(scene);
    scene.scaleBody(body, target / info.value);
    scene.setMeasurementDriving(m.id, target);
    const check = scene.measureInfo(m);
    if (!check || Math.abs(check.value - target) > sketchConfig.tol) {
      restore(scene, snap);
      return reject;
    }
    return [];
  }
  const breaks = solveAndApply(scene, { m, target });
  if (breaks.length) return breaks;
  scene.setMeasurementDriving(m.id, target);
  return breaks;
}

// --- scale-on-first-dimension eligibility -------------------------------------

/** The body that owns a ref outright, or null (a free joint / cross-body rail owns none). */
function refOwnerBody(scene: Scene, ref: MeasureRef): number | null {
  switch (ref.kind) {
    case "vertex":
    case "edge":
    case "bodyPoint":
      return scene.getBody(ref.bodyId) ? ref.bodyId : null;
    case "joint": {
      const j = scene.getJoint(ref.jointId);
      return j ? j.bodyId : null;
    }
    case "rail": {
      const c = scene.constraints.find((x) => x.id === ref.sliderId && x.kind === "slider");
      if (!c || c.kind !== "slider") return null;
      const a = scene.getJoint(c.railA);
      const b = scene.getJoint(c.railB);
      return a && b && a.bodyId !== null && a.bodyId === b.bodyId ? a.bodyId : null;
    }
    case "guidePoint":
    case "guideLine":
      return null; // guides are world construction — no body owns them
  }
}

/** Whether a ref touches the body at all (owner match; a rail touches via either joint). */
function refTouchesBody(scene: Scene, ref: MeasureRef, bodyId: number): boolean {
  if (ref.kind === "rail") {
    const c = scene.constraints.find((x) => x.id === ref.sliderId && x.kind === "slider");
    if (!c || c.kind !== "slider") return false;
    return [c.railA, c.railB].some((id) => scene.getJoint(id)?.bodyId === bodyId);
  }
  return refOwnerBody(scene, ref) === bodyId;
}

/**
 * The body to uniformly scale for this dimension edit, or null for the node-solve path.
 * Eligible when both refs live on one body, no *other* driving dimension touches that
 * body, and every sketch constraint touching it stays fully inside it.
 */
function scaleEligibleBody(scene: Scene, m: Measurement): number | null {
  const a = refOwnerBody(scene, m.refA);
  const b = refOwnerBody(scene, m.refB);
  if (a === null || a !== b) return null;
  for (const other of scene.measurements) {
    if (other.id === m.id || other.mode !== "draw" || !other.driving) continue;
    if (refTouchesBody(scene, other.refA, a) || refTouchesBody(scene, other.refB, a)) {
      return null;
    }
  }
  for (const c of scene.sketch) {
    const refs = c.refB ? [c.refA, c.refB] : [c.refA];
    const touching = refs.filter((r) => refTouchesBody(scene, r, a)).length;
    if (touching > 0 && touching < refs.length) return null; // external constraint
  }
  return a;
}
