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
  sameMeasureRef,
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
   * 1 = geometry (body control vertices, joints), 2 = pinned by the active drag,
   * 3 = component-instance geometry (immovable — the shape belongs to the definition,
   * so it outranks even the drag: dragging against it yields instead).
   * Corrections always flow to the **lowest** rank in a pair — so guide constraints
   * are satisfied by moving free guide points, never by moving joints or body nodes,
   * and a drag is never tugged back by its constraints. Equal ranks split evenly.
   */
  rank: number[];
  anchorSet?: ReadonlySet<string>;
  /**
   * Guide variables **tied to geometry**: they appear in a coincident constraint with a
   * body node / joint / edge / rail (directly, or through a chain of guide–guide
   * coincidences). Such a guide is a *reference fixed to the geometry*, not free
   * construction: the tie itself still moves the guide onto the geometry (rank 0), but
   * every other item — a driving dimension to the guide, a parallel to it — sees it at
   * `TIED_GUIDE_RANK`, so the correction flows into the geometry instead. Without this
   * a tie and a dimension on the same guide fight over it and never converge (the
   * body, the only thing that could move, never does).
   */
  tied: Pick<ReadonlySet<string>, "has">;
  /**
   * Guides carrying **two or more ties**. A single point-on-line tie is met by
   * translating the guide onto the point (construction yields); a second one can't be —
   * the translation that reaches one point leaves the other, and the two ties oscillate
   * forever. Such a guide is the *reference* for its point-on-line ties: the points come
   * to the line (like every other demand on a tied guide), so several corners / joints
   * can be aligned on one guideline. Point–point ties are unaffected — they keep the
   * guide's defining point glued to its joint / corner.
   */
  multiTied: ReadonlySet<number>;
}

/**
 * Effective rank of a tied guide for items other than its ties: above geometry (1),
 * so geometry moves instead of the guide; below a drag anchor (2) and instance
 * geometry (3), so those still win and the guide yields (its tie then reports the break).
 */
const TIED_GUIDE_RANK = 1.5;

/** The rank an item should weigh variable `i` at (`tie`: the item is a guide–geometry tie). */
function itemRank(sys: System, i: number, tie: boolean): number {
  const r = sys.rank[i];
  return !tie && r === 0 && sys.tied.has(sys.keys[i]) ? TIED_GUIDE_RANK : r;
}

const isGuideRef = (r: MeasureRef) => r.kind === "guidePoint" || r.kind === "guideLine";

/** Guide variable keys a ref names (a guideline names both its points). */
function guideVarKeys(ref: MeasureRef): string[] {
  if (ref.kind === "guidePoint") return [`g:${ref.guideId}:${ref.which}`];
  if (ref.kind === "guideLine") return [`g:${ref.guideId}:a`, `g:${ref.guideId}:b`];
  return [];
}

/** Guide variables tied to geometry through coincident constraints (see System.tied). */
function tiedGuideVars(scene: Scene): Set<string> {
  const tied = new Set<string>();
  const links: [string[], string[]][] = []; // guide–guide coincidences, for the chain
  for (const c of scene.sketch) {
    if (c.kind !== "coincident" || !c.refB) continue;
    const ga = isGuideRef(c.refA);
    const gb = isGuideRef(c.refB);
    if (ga && gb) links.push([guideVarKeys(c.refA), guideVarKeys(c.refB)]);
    else if (ga) for (const k of guideVarKeys(c.refA)) tied.add(k);
    else if (gb) for (const k of guideVarKeys(c.refB)) tied.add(k);
  }
  // A guide tied to a tied guide is tied too (a whole guide is tied when either point is).
  const wholeGuide = (k: string) => { const g = k.split(":")[1]; return [`g:${g}:a`, `g:${g}:b`]; };
  let grew = true;
  while (grew) {
    grew = false;
    for (const k of [...tied]) for (const w of wholeGuide(k)) if (!tied.has(w)) { tied.add(w); grew = true; }
    for (const [ka, kb] of links) {
      const a = ka.some((k) => tied.has(k));
      const b = kb.some((k) => tied.has(k));
      if (a !== b) { for (const k of a ? kb : ka) tied.add(k); grew = true; }
    }
  }
  return tied;
}

/** Ids of guides tied to geometry by two or more coincidences (see System.multiTied). */
function multiTiedGuides(scene: Scene): Set<number> {
  const count = new Map<number, number>();
  for (const c of scene.sketch) {
    if (c.kind !== "coincident" || !c.refB) continue;
    const ga = isGuideRef(c.refA);
    const gb = isGuideRef(c.refB);
    if (ga === gb) continue; // geometry–geometry, or guide–guide: not a tie
    const r = ga ? c.refA : c.refB;
    if (r.kind !== "guidePoint" && r.kind !== "guideLine") continue;
    count.set(r.guideId, (count.get(r.guideId) ?? 0) + 1);
  }
  return new Set([...count].filter(([, n]) => n >= 2).map(([id]) => id));
}

/** Mobility rank of a variable (see System.rank). */
function varRank(scene: Scene, key: string, anchored: boolean): number {
  // A pattern axis: the seed anchor never moves for the axis's sake (the direction pivots
  // about the seed), the first-instance point is ordinary geometry.
  if (key.startsWith("pa:")) return 3;
  if (key.startsWith("pb:")) return anchored ? 2 : 1;
  if (!key.startsWith("g:")) {
    // v:bodyId:… or j:jointId — instance-owned geometry never moves in a sketch solve.
    const id = Number(key.split(":")[1]);
    const owned = key.startsWith("v:")
      ? scene.instanceOfBody(id) !== undefined
      : scene.instanceOfJoint(id) !== undefined;
    if (owned) return 3;
    // Pattern members are derived from their seed: the solve never moves them either
    // (a constraint on one moves the other side; the seed itself stays free geometry).
    const parts = key.split(":");
    const member = key.startsWith("v:")
      ? parts.length > 3 && scene.patternOfHole(id, Number(parts[3]))?.role === "member"
      : key.startsWith("j:") && scene.patternOfJoint(id)?.role === "member";
    if (member) return 3;
  }
  if (anchored) return 2;
  return key.startsWith("g:") ? 0 : 1;
}

interface SolveItem {
  id: number;
  kind: "constraint" | "dimension";
  /** Residual before correction; applies the correction when `apply` is true. */
  run(pos: Vec2[], apply: boolean): number;
}

function vertexKey(bodyId: number, index: number, hole: number | null = null): string {
  return hole === null ? `v:${bodyId}:${index}` : `v:${bodyId}:${index}:${hole}`;
}

/** The control polygon a vertex/edge ref names: a hole's, or the body's outer one. */
function refControl(scene: Scene, bodyId: number, hole: number | null): Vec2[] | null {
  const b = scene.getBody(bodyId);
  if (!b) return null;
  return hole === null ? b.controlLocal : b.holes?.[hole]?.controlLocal ?? null;
}

/** Variable key for a point ref, or null when the ref can't be a solver variable. */
function pointVarKey(scene: Scene, ref: MeasureRef): string | null {
  if (ref.kind === "vertex") {
    const hole = ref.hole ?? null;
    const ctrl = refControl(scene, ref.bodyId, hole);
    return ctrl && ref.index >= 0 && ref.index < ctrl.length
      ? vertexKey(ref.bodyId, ref.index, hole)
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
      for (let hi = 0; hi < (body.holes?.length ?? 0); hi++) {
        const hc = scene.bodyHoleControlWorld(body, hi);
        for (let i = 0; i < hc.length; i++) {
          if (dist(hc[i], w) < VERTEX_LINK_EPS) return vertexKey(body.id, i, hi);
        }
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
    const hole = ref.hole ?? null;
    const ctrl = refControl(scene, ref.bodyId, hole);
    if (!ctrl || ctrl.length < 2 || ref.index < 0 || ref.index >= ctrl.length) return null;
    return [
      vertexKey(ref.bodyId, ref.index, hole),
      vertexKey(ref.bodyId, (ref.index + 1) % ctrl.length, hole),
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
  if (ref.kind === "patternAxis") {
    // A pattern axis: its seed anchor (`pa`, immovable — the axis pivots about the seed)
    // and the first instance along it (`pb`, movable — writes back the axis step).
    const p = scene.getPattern(ref.patternId);
    return p && p.layout.kind === "linear" && p.layout.axes[ref.axis]
      ? [`pa:${ref.patternId}`, `pb:${ref.patternId}:${ref.axis}`]
      : null;
  }
  return null;
}

/** Current world position of a variable. */
function varWorld(scene: Scene, key: string): Vec2 | null {
  const parts = key.split(":");
  if (parts[0] === "v") {
    const bodyId = Number(parts[1]);
    const index = Number(parts[2]);
    const hole = parts.length > 3 ? Number(parts[3]) : null;
    const body = scene.getBody(bodyId);
    const ctrl = refControl(scene, bodyId, hole);
    if (!body || !ctrl || index < 0 || index >= ctrl.length) return null;
    return hole === null
      ? scene.bodyControlWorld(body)[index]
      : scene.bodyHoleControlWorld(body, hole)[index];
  }
  if (parts[0] === "g") {
    const g = scene.getGuide(Number(parts[1]));
    return g ? vec(g[parts[2] as "a" | "b"].x, g[parts[2] as "a" | "b"].y) : null;
  }
  if (parts[0] === "pa" || parts[0] === "pb") {
    const info = scene.patternInfo(Number(parts[1]));
    if (!info) return null;
    if (parts[0] === "pa") return vec(info.anchor.x, info.anchor.y);
    const ax = info.axes[Number(parts[2])];
    if (!ax) return null;
    // The first instance along the axis: anchor + (end − anchor) / (count − 1).
    return add(info.anchor, scale(sub(ax.end, info.anchor), 1 / Math.max(1, ax.count - 1)));
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
  sys.rank.push(varRank(scene, key, sys.anchorSet?.has(key) ?? false));
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

/**
 * The world-axis normal of a line ref that carries a horizontal / vertical sketch
 * constraint ((0,1) for a horizontal line, (1,0) for a vertical one), or null. A
 * distance measured off such a line shifts along this exact axis rather than the
 * line's momentary normal: mid-sweep the line can be transiently tilted (a side
 * dimension has pulled one end before the other), and a shift along that tilted
 * normal leaks motion into the free direction that nothing pulls back — bodies drift
 * sideways while being dragged against a vertical dimension.
 */
function axisNormalOf(scene: Scene, ref: MeasureRef): Vec2 | null {
  for (const c of scene.sketch) {
    if (c.refB !== null && c.refB !== undefined) continue;
    if (c.kind !== "horizontal" && c.kind !== "vertical") continue;
    if (!sameMeasureRef(c.refA, ref)) continue;
    return c.kind === "horizontal" ? vec(0, 1) : vec(1, 0);
  }
  return null;
}

/** `axis` signed to agree with the momentary normal `n` (keeps the held side stable). */
function alignedAxis(axis: Vec2, n: Vec2): Vec2 {
  return dot(axis, n) < 0 ? scale(axis, -1) : axis;
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
  const isLineRef = (r: MeasureRef) => r.kind === "rail" || r.kind === "edge" || r.kind === "guideLine" || r.kind === "patternAxis";
  // A guide–geometry coincidence is a *tie*: it always brings the guide to the geometry.
  const tie = kind === "coincident" && !!c.refB && isGuideRef(c.refA) !== isGuideRef(c.refB);
  let rank = (i: number) => itemRank(sys, i, tie);
  if (kind === "coincident" && c.refB && (isLineRef(c.refA) || isLineRef(c.refB))) {
    // A point-on-line tie onto a guide that carries several ties can't be met by
    // translating the guide (see System.multiTied): the guide is the reference and
    // the point comes to it.
    const gl = c.refA.kind === "guideLine" ? c.refA : c.refB.kind === "guideLine" ? c.refB : null;
    if (gl && sys.multiTied.has(gl.guideId)) rank = (i: number) => itemRank(sys, i, false);
    // Point on an infinite line: zero the signed perpendicular distance. The model
    // normalizes the point into refA, but handle either order (robust to hand-edited
    // saves). Same projection as a point+line driving dimension with target 0.
    const aLine = isLineRef(c.refA);
    const kp = pointVarKey(scene, aLine ? c.refB : c.refA);
    const kl = lineVarKeys(scene, aLine ? c.refA : c.refB);
    if (!kp || !kl) return "invalid";
    const p = acquire(scene, sys, kp);
    const l0 = acquire(scene, sys, kl[0]);
    const l1 = acquire(scene, sys, kl[1]);
    if (p === null || l0 === null || l1 === null) return "invalid";
    if (p === l0 || p === l1) return null; // the point ends the line: on it by construction
    const wp = shareOf(rank(p), Math.min(rank(l0), rank(l1))); // fraction the point absorbs
    return {
      id: c.id,
      kind: "constraint",
      run(pos, apply) {
        const d = sub(pos[l1], pos[l0]);
        const l = len(d);
        if (l < EPS) return 0; // degenerate line: nothing to project onto
        const n = perp(scale(d, 1 / l));
        const s = dot(sub(pos[p], pos[l0]), n); // signed distance off the line
        if (apply) {
          pos[p] = sub(pos[p], scale(n, s * wp));
          const shift = scale(n, s * (1 - wp)); // line comes to the point (rank-weighted)
          pos[l0] = add(pos[l0], shift);
          pos[l1] = add(pos[l1], shift);
        }
        return Math.abs(s);
      },
    };
  }
  if (kind === "coincident" || ((kind === "horizontal" || kind === "vertical") && c.refB)) {
    const ka = pointVarKey(scene, c.refA);
    const kb = c.refB ? pointVarKey(scene, c.refB) : null;
    if (!ka || !kb) return "invalid";
    const i = acquire(scene, sys, ka);
    const j = acquire(scene, sys, kb);
    if (i === null || j === null) return "invalid";
    if (i === j) return null; // same variable: trivially satisfied
    const wp = shareOf(rank(i), rank(j)); // fraction i absorbs
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
    const wl = shareOf(rank(i), rank(j));
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
    Math.min(rank(a0), rank(a1)),
    Math.min(rank(b0), rank(b1))
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
  const rank = (i: number) => itemRank(sys, i, false); // a dimension to a tied guide moves geometry
  const isLine = (r: MeasureRef) => r.kind === "rail" || r.kind === "edge" || r.kind === "guideLine" || r.kind === "patternAxis";
  const aLine = isLine(m.refA);
  const bLine = isLine(m.refB);
  if (!aLine && !bLine) {
    const ka = pointVarKey(scene, m.refA);
    const kb = pointVarKey(scene, m.refB);
    if (!ka || !kb) return "invalid";
    const i = acquire(scene, sys, ka);
    const j = acquire(scene, sys, kb);
    if (i === null || j === null || i === j) return "invalid";
    const wi = shareOf(rank(i), rank(j)); // fraction i absorbs
    if (m.axis === "h" || m.axis === "v") {
      const axis: "x" | "y" = m.axis === "h" ? "x" : "y";
      // The held side (Measurement.side, captured when the dim started driving) makes
      // the error signed: an overshoot past the partner reads as a large error back
      // toward the drawn side, so a fast drag can never flip the two sides.
      const side = m.side ?? null;
      return {
        id: m.id,
        kind: "dimension",
        run(pos, apply) {
          const d = pos[j][axis] - pos[i][axis];
          const s = side ?? (d === 0 ? 1 : Math.sign(d));
          const err = s * target - d;
          if (apply) {
            pos[j][axis] += err * (1 - wi);
            pos[i][axis] -= err * wi;
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
      Math.min(rank(a0), rank(a1)),
      Math.min(rank(b0), rank(b1))
    ); // fraction line A absorbs
    // A driving line–line distance implies the pair is parallel (CAD convention):
    // align the directions, then set the gap along the common normal. The held side
    // (see the h/v case) keeps the pair from flipping through each other.
    const side = m.side ?? null;
    const axisN = axisNormalOf(scene, m.refA) ?? axisNormalOf(scene, m.refB);
    return {
      id: m.id,
      kind: "dimension",
      run(pos, apply) {
        const alignErr = projectParallel(pos, [a0, a1], [b0, b1], 0, apply, wA);
        const d1 = sub(pos[a1], pos[a0]);
        const n1 = len(d1);
        if (n1 < EPS) return alignErr;
        const nRaw = perp(scale(d1, 1 / n1));
        const n = axisN ? alignedAxis(axisN, nRaw) : nRaw;
        const m1 = scale(add(pos[a0], pos[a1]), 0.5);
        const m2 = scale(add(pos[b0], pos[b1]), 0.5);
        const s = dot(sub(m2, m1), n);
        const sg = side ?? (s === 0 ? 1 : Math.sign(s));
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
  const wp = shareOf(rank(p), Math.min(rank(l0), rank(l1))); // fraction the point absorbs
  const side = m.side ?? null; // held side — the point can't flip across the line
  const axisN = axisNormalOf(scene, lRef);
  return {
    id: m.id,
    kind: "dimension",
    run(pos, apply) {
      const d = sub(pos[l1], pos[l0]);
      const l = len(d);
      if (l < EPS) return target; // degenerate line: can't measure, full residual
      const nRaw = perp(scale(d, 1 / l));
      const n = axisN ? alignedAxis(axisN, nRaw) : nRaw;
      const s = dot(sub(pos[p], pos[l0]), n);
      const sg = side ?? (s === 0 ? 1 : Math.sign(s));
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
function buildSystem(
  scene: Scene,
  override?: DimSpec,
  anchors?: ReadonlySet<string>,
  guidesAsReference = false
): BuildResult {
  // `guidesAsReference`: the fallback pass — *every* guide counts as tied (see
  // System.tied), so geometry absorbs whatever the guides alone couldn't satisfy.
  const sys: System = {
    keys: [], pos: [], index: new Map(), rank: [], anchorSet: anchors,
    tied: guidesAsReference ? { has: () => true } : tiedGuideVars(scene),
    multiTied: multiTiedGuides(scene),
  };
  const items: SolveItem[] = [];
  const invalid: SketchBreak[] = [];
  for (const c of scene.sketch) {
    // Pose constraints (every end on instance geometry) are not shape material: they
    // move rigid parts and are enforced by pose.ts, never by this solver.
    if (scene.refInstanceOwned(c.refA) && (!c.refB || scene.refInstanceOwned(c.refB))) continue;
    const item = buildConstraintItem(scene, sys, c);
    if (item === "invalid") invalid.push({ id: c.id, kind: "constraint", error: Infinity });
    else if (item) items.push(item);
  }
  const dims: DimSpec[] = scene.measurements
    .filter((m) => m.mode === "draw" && m.driving && m.target !== undefined)
    // Pose dimensions (both ends on instance geometry) are not shape material: they
    // move rigid parts and are enforced by pose.ts, never by this solver.
    .filter((m) => !(scene.refInstanceOwned(m.refA) && scene.refInstanceOwned(m.refB)))
    // Diameter dimensions set a disk's radius directly (no vertex moves) — see
    // `enforceDiameterDims`; they have no place in the vertex/joint system.
    .filter((m) => m.axis !== "diameter")
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
 * Write the solved positions back through the scene's edit paths: whole-body rigid
 * motion first (see `applyRigidParts`), then control vertices (bodies reshape; linked
 * joints are carried), then joints (attached joints are clamped into their — already
 * reshaped — bodies; ground anchors follow).
 */
function applySystem(scene: Scene, sys: System): void {
  applyRigidParts(scene, sys);
  const order = sys.keys
    .map((key, i) => ({ key, i }))
    .sort((a, b) => Number(b.key.startsWith("v")) - Number(a.key.startsWith("v")));
  for (const { key, i } of order) {
    const cur = varWorld(scene, key);
    if (!cur) continue;
    const delta = sub(sys.pos[i], cur);
    if (len(delta) < EPS) continue;
    const parts = key.split(":");
    if (parts[0] === "pa") continue; // derived from the seed: never written back
    if (parts[0] === "pb") {
      // The axis step is the solved first instance relative to the solved anchor (both
      // in the solver's frame, so a body that moved in the same solve doesn't skew it).
      const ia = sys.index.get(`pa:${parts[1]}`);
      const anchor = ia === undefined ? varWorld(scene, `pa:${parts[1]}`) : sys.pos[ia];
      if (anchor) scene.setPatternAxisVector(Number(parts[1]), Number(parts[2]), sub(sys.pos[i], anchor));
      continue;
    }
    if (parts[0] === "v") {
      scene.moveBodyVertex(
        Number(parts[1]),
        Number(parts[2]),
        delta,
        parts.length > 3 ? Number(parts[3]) : null
      );
    } else if (parts[0] === "g") {
      scene.moveGuidePoint(Number(parts[1]), parts[2] as "a" | "b", sys.pos[i]);
    } else scene.moveJoint(Number(parts[1]), delta);
  }
}

/**
 * Move each body by the **rigid part** of its solved outline motion before the
 * per-vertex writeback. `moveBodyVertex` keeps holes and non-stuck joints fixed in
 * *world* space (right for a corner tweak), so a body whose whole outline is shifted
 * by a dimension / constraint would otherwise leave its holes and joints behind. The
 * best-fit translation + rotation mapping the current outer polygon onto the solved
 * one (vertices outside the system count as staying put) is applied first, and the
 * vertex pass then applies only the residual reshape (zero when the outline moved
 * rigidly):
 * - every outer vertex a solver variable → the whole body moves (`moveBody` /
 *   `rotateBody`): holes, joints and ground anchors ride along;
 * - otherwise the body is reshaping, not moving: only its **holes** follow the rigid
 *   part (holes are material, as rigid as the outline — a squashed plate keeps its
 *   hole centred), while non-stuck joints keep their world-fixed reshape behaviour.
 *
 * Hole vertices / joints that are solver **variables** (e.g. two holes with a
 * vertical constraint between them) get their solved position in the vertex pass,
 * which would undo the carry: the solver saw them satisfied where they were and
 * never moved them. So a body-owned variable the solve left **unchanged** rides
 * along — its solved position is mapped through the same rigid motion — while one
 * the solve did move (a hole dimensioned to the outline, a joint pulled by a
 * coincident) keeps its absolute solution.
 */
function applyRigidParts(scene: Scene, sys: System): void {
  const init = sys.keys.map((k) => varWorld(scene, k)); // pre-writeback positions
  const perBody = new Map<number, Map<number, Vec2>>();
  sys.keys.forEach((key, i) => {
    const parts = key.split(":");
    if (parts[0] !== "v" || parts.length > 3) return; // outer control vertices only
    const bodyId = Number(parts[1]);
    let m = perBody.get(bodyId);
    if (!m) perBody.set(bodyId, (m = new Map()));
    m.set(Number(parts[2]), sys.pos[i]);
  });
  for (const [bodyId, solved] of perBody) {
    const body = scene.getBody(bodyId);
    if (!body) continue;
    const whole = solved.size >= body.controlLocal.length; // solver holds every vertex
    if (!whole && !body.holes?.length) continue; // a plain reshape: nothing to carry
    const cur = scene.bodyControlWorld(body);
    const tgt = cur.map((p, i) => solved.get(i) ?? p);
    if (tgt.every((q, i) => len(sub(q, cur[i])) < EPS)) continue;
    const n = cur.length;
    const c0 = scale(cur.reduce((a, p) => add(a, p), vec(0, 0)), 1 / n);
    const c1 = scale(tgt.reduce((a, p) => add(a, p), vec(0, 0)), 1 / n);
    let sc = 0; // Σ cross(p', q')
    let sd = 0; // Σ dot(p', q')
    for (let i = 0; i < n; i++) {
      const p = sub(cur[i], c0);
      const q = sub(tgt[i], c1);
      sc += p.x * q.y - p.y * q.x;
      sd += dot(p, q);
    }
    const ang = Math.hypot(sc, sd) > EPS ? Math.atan2(sc, sd) : 0;
    const delta = sub(c1, c0);
    const carry = (p: Vec2) => add(c1, rotate(sub(p, c0), ang));
    // Body-owned variables the solve left where they were ride with the body (holes
    // always; joints only when the whole body moves — on a reshape they stay put).
    sys.keys.forEach((key, i) => {
      const parts = key.split(":");
      const holeVar = parts[0] === "v" && parts.length > 3 && Number(parts[1]) === bodyId;
      const jointVar = whole && parts[0] === "j" && scene.getJoint(Number(parts[1]))?.bodyId === bodyId;
      const p0 = init[i];
      if ((!holeVar && !jointVar) || !p0 || dist(sys.pos[i], p0) >= sketchConfig.tol) return;
      sys.pos[i] = carry(sys.pos[i]);
    });
    if (whole) {
      if (Math.abs(ang) > 1e-12) scene.rotateBody(bodyId, c0, ang);
      if (len(delta) >= EPS) scene.moveBody(bodyId, delta);
      continue;
    }
    // Partial reshape: carry each hole vertex by the outline's rigid part. Hole
    // vertices that are variables themselves get their solved position in the vertex
    // pass (deltas are re-read live), so moving them here is harmless.
    for (let hi = 0; hi < body.holes!.length; hi++) {
      const hw = scene.bodyHoleControlWorld(body, hi);
      hw.forEach((p, k) => {
        const d = sub(carry(p), p);
        if (len(d) >= EPS) scene.moveBodyVertex(bodyId, k, d, hi);
      });
    }
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
  let build = buildSystem(scene, override, anchors);
  if (build.invalid.length) return build.invalid;
  if (!build.items.length) return [];
  let guidesAsReference = false;
  if (!iterate(build.sys, build.items)) {
    // Construction-first didn't settle. With guides in play the usual cause is several
    // demands on one guide (two driving dimensions to it, a tie plus a dimension…):
    // each pushes the guide its own way and the geometry — the only thing that could
    // give — never moves. Retry with every guide as a fixed reference, so geometry
    // absorbs the corrections. A guide with a single demand still yields (first pass).
    if (scene.guides.length === 0) return residualBreaks(build.sys, build.items);
    guidesAsReference = true;
    build = buildSystem(scene, override, anchors, true);
    if (!iterate(build.sys, build.items)) return residualBreaks(build.sys, build.items);
  }
  const snap = snapshot(scene);
  applySystem(scene, build.sys);
  // Re-measure from the actual scene: the edit paths may have adjusted positions
  // (containment clamps), so verify the applied state truly satisfies everything.
  const after = buildSystem(scene, override, anchors, guidesAsReference);
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
  const breaks = solveAndApply(scene, undefined, anchors);
  enforceDiameterDims(scene);
  return breaks;
}

/**
 * Re-apply every driving diameter dimension: set its disk's radius to target / 2. A
 * disk's radius never moves a vertex or joint, so this is independent of the vertex
 * system — but a uniform body scale (first-dimension behaviour) scales hole radii too,
 * and this puts a dimensioned disk back. A dimension whose disk is gone (a node was
 * added to the hole) is left alone; it renders violated / not at all.
 */
export function enforceDiameterDims(scene: Scene): void {
  for (const m of scene.measurements) {
    if (m.mode !== "draw" || !m.driving || m.target === undefined || m.axis !== "diameter") continue;
    const disk = scene.diskOfRef(m.refA);
    if (!disk || Math.abs(disk.r * 2 - m.target) <= sketchConfig.tol) continue;
    scene.setDiskRadius(disk.bodyId, m.target / 2, disk.hole);
  }
}

// --- drag anchoring ------------------------------------------------------------

/** Anchor keys pinning a whole body's shape: every control vertex (outer + hole) +
 *  every joint on it. */
export function anchorVarsForBody(scene: Scene, bodyId: number): string[] {
  const body = scene.getBody(bodyId);
  if (!body) return [];
  const keys = body.controlLocal.map((_, i) => vertexKey(bodyId, i));
  body.holes?.forEach((h, hi) => {
    for (let i = 0; i < h.controlLocal.length; i++) keys.push(vertexKey(bodyId, i, hi));
  });
  for (const j of scene.joints) {
    if (j.bodyId !== bodyId) continue;
    const k = pointVarKey(scene, { kind: "joint", jointId: j.id });
    if (k) keys.push(k);
  }
  // The body's pattern axes ride with it: a drag never re-aims them.
  for (const p of scene.patterns) {
    if (p.bodyId === bodyId && p.layout.kind === "linear") p.layout.axes.forEach((_, i) => keys.push(`pb:${p.id}:${i}`));
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

/** Anchor key pinning one body control vertex (of the outer outline, or hole `hole`). */
export function anchorVarForVertex(bodyId: number, index: number, hole: number | null = null): string {
  return vertexKey(bodyId, index, hole);
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
 * Component-instance geometry is design-locked (its shape belongs to the definition):
 * a dimension with a single instance-owned end drives by moving only the free side
 * (instance variables are rank-immovable in the solve). One with **both** ends on
 * instance geometry is a *pose* dimension — not shape material at all — and belongs
 * to `applyPoseDimension` (pose.ts); reaching this function with one is rejected
 * (callers route through `applyDimensionValue`).
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
  if (scene.refInstanceOwned(m.refA) && scene.refInstanceOwned(m.refB)) return reject;
  // Both ends inside one pattern (seed ↔ member, member ↔ member): the spacing is the
  // pattern's own parameter — edit it on the pattern, not through a dimension.
  const pa = scene.patternOfRef(m.refA);
  if (pa && pa === scene.patternOfRef(m.refB)) return reject;
  const info = scene.measureInfo(m);
  if (!info || info.kind !== "distance") return reject; // angle dimensions can't drive (v1)
  if (m.axis === "diameter") {
    // A disk's diameter is its own parameter: set the radius directly. No vertex or
    // joint moves, so nothing else in the sketch can be disturbed.
    const disk = scene.diskOfRef(m.refA);
    if (!disk) return reject;
    scene.setDiskRadius(disk.bodyId, target / 2, disk.hole);
    scene.setMeasurementDriving(m.id, target);
    return [];
  }
  const body = scaleEligibleBody(scene, m);
  if (body !== null && info.value > EPS) {
    const snap = snapshot(scene);
    scene.scaleBody(body, target / info.value);
    enforceDiameterDims(scene); // dimensioned disks on the body keep their diameter
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
    case "patternAxis":
      return scene.getPattern(ref.patternId)?.bodyId ?? null;
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
    // A diameter dimension is re-applied after a scale, so it never blocks one.
    if (other.axis === "diameter") continue;
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
