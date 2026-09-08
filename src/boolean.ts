/**
 * Polygon union for the Combine tool (and segment helpers shared with Split).
 *
 * The union works on a planar straight-line graph: every input edge is split where it
 * crosses, touches or overlaps another one, coincident vertices are merged, and each
 * remaining edge is classified by sampling a point just left and just right of its
 * midpoint against the input regions — an edge with material on exactly one side is a
 * boundary edge of the union (directed so the material is on its left). Boundary edges
 * are then chained into loops; positive (counter-clockwise) loops are outer outlines,
 * negative ones are holes, assigned to the outer that contains them.
 *
 * Sampling instead of face-tracing keeps the degenerate cases the tool must handle
 * (bodies sharing an edge, a vertex landing on another body's edge, exactly coincident
 * vertices) on the same code path as the general one: a shared edge simply has
 * material on both sides and drops out.
 */
import {
  Vec2, vec, add, sub, scale, dot, cross, len, dist, normalize, perp,
  polygonArea, pointInPolygon,
} from "./geometry";

/** A filled region: an outer loop minus its holes (orientation of either is irrelevant). */
export interface PolyRegion {
  outer: Vec2[];
  holes: Vec2[][];
}

export interface UnionResult {
  /** Connected pieces of the union (one when every input touches the others). */
  regions: PolyRegion[];
  /**
   * Some boundary vertex is used by more than one boundary loop pass — the union
   * touches itself at a point (two pieces meeting at a corner, or a hole touching the
   * outer). The loops are still valid polygons, but the shape is degenerate.
   */
  pinched: boolean;
}

/** Strict crossing test: the open segments a–b and c–d intersect at a single interior point. */
export function segmentsCross(a: Vec2, b: Vec2, c: Vec2, d: Vec2): boolean {
  const o = (p: Vec2, q: Vec2, r: Vec2): number => Math.sign(cross(sub(q, p), sub(r, p)));
  const o1 = o(a, b, c), o2 = o(a, b, d), o3 = o(c, d, a), o4 = o(c, d, b);
  return o1 !== o2 && o3 !== o4 && o1 !== 0 && o2 !== 0 && o3 !== 0 && o4 !== 0;
}

/** Whether `p` lies on the closed segment a–b (within `eps`). */
export function pointOnSegment(p: Vec2, a: Vec2, b: Vec2, eps: number): boolean {
  const ab = sub(b, a);
  const l2 = dot(ab, ab);
  if (l2 < 1e-24) return dist(p, a) <= eps;
  const t = dot(sub(p, a), ab) / l2;
  if (t < -eps || t > 1 + eps) return false;
  const q = add(a, scale(ab, Math.max(0, Math.min(1, t))));
  return dist(p, q) <= eps;
}

/** Is `p` inside the region (inside the outer, outside every hole)? */
function inRegion(p: Vec2, r: PolyRegion): boolean {
  return pointInPolygon(p, r.outer) && !r.holes.some((h) => pointInPolygon(p, h));
}

/** Merge points closer than `eps` into shared vertex ids (hash grid, cell = eps). */
class VertexPool {
  readonly points: Vec2[] = [];
  private cells = new Map<string, number[]>();
  constructor(private eps: number) {}

  private cellKey(cx: number, cy: number): string {
    return `${cx},${cy}`;
  }

  id(p: Vec2): number {
    const cx = Math.floor(p.x / this.eps);
    const cy = Math.floor(p.y / this.eps);
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        const ids = this.cells.get(this.cellKey(cx + dx, cy + dy));
        if (!ids) continue;
        for (const i of ids) if (dist(this.points[i], p) <= this.eps) return i;
      }
    }
    const i = this.points.length;
    this.points.push(vec(p.x, p.y));
    const k = this.cellKey(cx, cy);
    const list = this.cells.get(k);
    if (list) list.push(i);
    else this.cells.set(k, [i]);
    return i;
  }
}

/** Drop vertices that sit on the straight line between their neighbours (kept ≥ 3). */
export function simplifyCollinear(loop: Vec2[], eps: number): Vec2[] {
  let pts = loop.slice();
  let changed = true;
  while (changed && pts.length > 3) {
    changed = false;
    for (let i = 0; i < pts.length && pts.length > 3; i++) {
      const prev = pts[(i + pts.length - 1) % pts.length];
      const cur = pts[i];
      const next = pts[(i + 1) % pts.length];
      const d1 = sub(cur, prev);
      const d2 = sub(next, cur);
      const l1 = len(d1), l2 = len(d2);
      if (l1 < eps || l2 < eps) {
        pts.splice(i, 1);
        changed = true;
        i--;
        continue;
      }
      // Perpendicular deviation of `cur` from the prev→next chord, and same heading.
      const dev = Math.abs(cross(d1, d2)) / Math.max(l1, l2);
      if (dev <= eps && dot(d1, d2) > 0) {
        pts.splice(i, 1);
        changed = true;
        i--;
      }
    }
  }
  return pts;
}

/**
 * Union of filled regions. Returns the connected pieces (each an outer loop + holes,
 * outer loops counter-clockwise by `polygonArea`'s sign, holes clockwise), with
 * collinear boundary vertices removed. `null` when the inputs give no area at all.
 */
export function unionRegions(inputs: PolyRegion[]): UnionResult | null {
  // --- scale-relative tolerances ---
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const r of inputs) {
    for (const loop of [r.outer, ...r.holes]) {
      for (const p of loop) {
        minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
      }
    }
  }
  if (!Number.isFinite(minX)) return null;
  const diag = Math.max(Math.hypot(maxX - minX, maxY - minY), 1e-9);
  const eps = diag * 1e-7; // vertex-merge / on-line tolerance
  const delta = diag * 1e-5; // side-sampling offset (well above eps, well below feature size)

  // --- collect segments ---
  const segs: { a: Vec2; b: Vec2; ts: number[] }[] = [];
  for (const r of inputs) {
    for (const loop of [r.outer, ...r.holes]) {
      const n = loop.length;
      if (n < 2) continue;
      for (let i = 0; i < n; i++) {
        const a = loop[i], b = loop[(i + 1) % n];
        if (dist(a, b) > eps) segs.push({ a, b, ts: [] });
      }
    }
  }
  if (segs.length === 0) return null;

  // --- split every segment where it meets another (crossing, T-junction, overlap) ---
  const addT = (s: { a: Vec2; b: Vec2; ts: number[] }, t: number): void => {
    const l = dist(s.a, s.b);
    const et = eps / l;
    if (t > et && t < 1 - et) s.ts.push(t);
  };
  for (let i = 0; i < segs.length; i++) {
    const s1 = segs[i];
    const d1 = sub(s1.b, s1.a);
    const l1 = len(d1);
    for (let j = i + 1; j < segs.length; j++) {
      const s2 = segs[j];
      const d2 = sub(s2.b, s2.a);
      const l2 = len(d2);
      const denom = cross(d1, d2);
      if (Math.abs(denom) > 1e-9 * l1 * l2) {
        const r = sub(s2.a, s1.a);
        const t = cross(r, d2) / denom;
        const u = cross(r, d1) / denom;
        const et1 = eps / l1, et2 = eps / l2;
        if (t >= -et1 && t <= 1 + et1 && u >= -et2 && u <= 1 + et2) {
          addT(s1, t);
          addT(s2, u);
        }
      } else {
        // Parallel: collinear overlaps split each segment at the other's endpoints.
        const offLine = Math.abs(cross(sub(s2.a, s1.a), d1)) / l1;
        if (offLine > eps) continue;
        const proj1 = (p: Vec2) => dot(sub(p, s1.a), d1) / (l1 * l1);
        const proj2 = (p: Vec2) => dot(sub(p, s2.a), d2) / (l2 * l2);
        addT(s1, proj1(s2.a));
        addT(s1, proj1(s2.b));
        addT(s2, proj2(s1.a));
        addT(s2, proj2(s1.b));
      }
    }
  }

  // --- build the merged planar graph ---
  const pool = new VertexPool(eps);
  const edgeKeys = new Set<string>();
  const edges: [number, number][] = [];
  for (const s of segs) {
    const ts = [0, ...s.ts.sort((x, y) => x - y), 1];
    const d = sub(s.b, s.a);
    let prev = pool.id(s.a);
    for (let k = 1; k < ts.length; k++) {
      const p = k === ts.length - 1 ? s.b : add(s.a, scale(d, ts[k]));
      const cur = pool.id(p);
      if (cur !== prev) {
        const key = prev < cur ? `${prev}-${cur}` : `${cur}-${prev}`;
        if (!edgeKeys.has(key)) {
          edgeKeys.add(key);
          edges.push([prev, cur]);
        }
      }
      prev = cur;
    }
  }
  const P = pool.points;
  const filled = (p: Vec2): boolean => inputs.some((r) => inRegion(p, r));

  // --- classify: keep edges with material on exactly one side, directed material-left ---
  const directed: { from: number; to: number; used: boolean }[] = [];
  for (const [u, v] of edges) {
    const d = normalize(sub(P[v], P[u]));
    const n = perp(d);
    const m = scale(add(P[u], P[v]), 0.5);
    const left = filled(add(m, scale(n, delta)));
    const right = filled(sub(m, scale(n, delta)));
    if (left && !right) directed.push({ from: u, to: v, used: false });
    else if (right && !left) directed.push({ from: v, to: u, used: false });
  }
  if (directed.length === 0) return null;

  const outgoing = new Map<number, typeof directed>();
  for (const e of directed) {
    const list = outgoing.get(e.from);
    if (list) list.push(e);
    else outgoing.set(e.from, [e]);
  }
  let pinched = false;
  for (const list of outgoing.values()) if (list.length > 1) pinched = true;

  // --- chain boundary edges into loops ---
  const angle = (v: Vec2): number => Math.atan2(v.y, v.x);
  const loops: Vec2[][] = [];
  for (const start of directed) {
    if (start.used) continue;
    const loop: Vec2[] = [];
    let cur = start;
    let guard = directed.length + 1;
    while (guard-- > 0) {
      cur.used = true;
      loop.push(P[cur.from]);
      if (cur.to === start.from) break;
      const cands = (outgoing.get(cur.to) ?? []).filter((e) => !e.used);
      if (cands.length === 0) break; // broken chain (shouldn't happen) — keep what we have
      let next = cands[0];
      if (cands.length > 1) {
        // Material is on our left; the next edge is the first one met turning
        // clockwise from the reversed incoming direction (it bounds the same wedge).
        const back = angle(sub(P[cur.from], P[cur.to]));
        let best = Infinity;
        for (const c of cands) {
          let cw = back - angle(sub(P[c.to], P[c.from]));
          while (cw <= 1e-12) cw += Math.PI * 2;
          while (cw > Math.PI * 2 + 1e-12) cw -= Math.PI * 2;
          if (cw < best) { best = cw; next = c; }
        }
      }
      cur = next;
    }
    if (loop.length >= 3) loops.push(simplifyCollinear(loop, eps));
  }

  // --- outers vs holes, holes nested by containment ---
  const outers: PolyRegion[] = [];
  const holes: Vec2[][] = [];
  for (const l of loops) {
    if (l.length < 3) continue;
    const a = polygonArea(l);
    if (Math.abs(a) < eps * eps) continue;
    if (a > 0) outers.push({ outer: l, holes: [] });
    else holes.push(l);
  }
  if (outers.length === 0) return null;
  for (const h of holes) {
    // The smallest outer containing the hole's first vertex owns it.
    let owner: PolyRegion | null = null;
    let ownerArea = Infinity;
    for (const o of outers) {
      if (!pointInPolygon(h[0], o.outer)) continue;
      const oa = polygonArea(o.outer);
      if (oa < ownerArea) { ownerArea = oa; owner = o; }
    }
    if (owner) owner.holes.push(h);
  }
  return { regions: outers, pinched };
}
