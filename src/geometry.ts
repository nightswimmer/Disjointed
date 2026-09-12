/** 2D vector + polygon math shared across the editor, solver and renderer. */

export interface Vec2 {
  x: number;
  y: number;
}

export const vec = (x: number, y: number): Vec2 => ({ x, y });
export const clone = (a: Vec2): Vec2 => ({ x: a.x, y: a.y });

export const add = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x + b.x, y: a.y + b.y });
export const sub = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x - b.x, y: a.y - b.y });
export const scale = (a: Vec2, s: number): Vec2 => ({ x: a.x * s, y: a.y * s });
export const dot = (a: Vec2, b: Vec2): number => a.x * b.x + a.y * b.y;

/** 2D scalar cross product (z-component of the 3D cross). */
export const cross = (a: Vec2, b: Vec2): number => a.x * b.y - a.y * b.x;

export const len = (a: Vec2): number => Math.hypot(a.x, a.y);
export const lenSq = (a: Vec2): number => a.x * a.x + a.y * a.y;
export const dist = (a: Vec2, b: Vec2): number => Math.hypot(a.x - b.x, a.y - b.y);

export function normalize(a: Vec2): Vec2 {
  const l = len(a);
  return l > 1e-9 ? { x: a.x / l, y: a.y / l } : { x: 0, y: 0 };
}

/** Left-hand perpendicular (rotate +90°). */
export const perp = (a: Vec2): Vec2 => ({ x: -a.y, y: a.x });

/** Rotate `a` by `angle` radians. */
export function rotate(a: Vec2, angle: number): Vec2 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return { x: a.x * c - a.y * s, y: a.x * s + a.y * c };
}

/** Signed area of a polygon (positive when wound counter-clockwise in screen-y-down). */
export function polygonArea(pts: Vec2[]): number {
  let a = 0;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    a += cross(pts[j], pts[i]);
  }
  return a / 2;
}

/** Area-weighted centroid of a polygon. Falls back to vertex average for degenerate input. */
export function polygonCentroid(pts: Vec2[]): Vec2 {
  let area = 0;
  let cx = 0;
  let cy = 0;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const f = cross(pts[j], pts[i]);
    area += f;
    cx += (pts[j].x + pts[i].x) * f;
    cy += (pts[j].y + pts[i].y) * f;
  }
  if (Math.abs(area) < 1e-9) {
    const avg = pts.reduce((acc, p) => add(acc, p), vec(0, 0));
    return scale(avg, 1 / Math.max(1, pts.length));
  }
  return { x: cx / (3 * area), y: cy / (3 * area) };
}

/**
 * Second moment of area of a polygon about its centroid (per unit density).
 * Used as the rotational inertia of a body.
 */
export function polygonInertiaAboutCentroid(pts: Vec2[], centroid: Vec2): number {
  let denom = 0;
  let numer = 0;
  const c = pts.map((p) => sub(p, centroid));
  for (let i = 0, j = c.length - 1; i < c.length; j = i++) {
    const a = c[j];
    const b = c[i];
    const f = Math.abs(cross(a, b));
    denom += f;
    numer += f * (dot(a, a) + dot(a, b) + dot(b, b));
  }
  return denom < 1e-9 ? 0 : numer / (6 * denom) * Math.abs(polygonArea(pts));
}

/** Point-in-polygon test (ray casting). */
export function pointInPolygon(p: Vec2, pts: Vec2[]): boolean {
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const a = pts[i];
    const b = pts[j];
    const intersects =
      a.y > p.y !== b.y > p.y &&
      p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x;
    if (intersects) inside = !inside;
  }
  return inside;
}

/** Distance from point `p` to the infinite line through `o` with unit direction `d`. */
export function distToLine(p: Vec2, o: Vec2, d: Vec2): number {
  return Math.abs(cross(sub(p, o), d));
}

/** Distance from point `p` to the segment `a`–`b` (clamped to the endpoints). */
export function distToSegment(p: Vec2, a: Vec2, b: Vec2): number {
  const ab = sub(b, a);
  const l2 = lenSq(ab);
  const t = l2 > 1e-12 ? Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2)) : 0;
  return dist(p, add(a, scale(ab, t)));
}

/** Closest point to `p` on the boundary of a closed polygon. */
export function closestPointOnPolygon(p: Vec2, pts: Vec2[]): Vec2 {
  let best = clone(pts[0]);
  let bestD = Infinity;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const a = pts[j];
    const b = pts[i];
    const ab = sub(b, a);
    const l2 = lenSq(ab);
    const t = l2 > 1e-12 ? Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2)) : 0;
    const q = add(a, scale(ab, t));
    const d = dist(p, q);
    if (d < bestD) {
      bestD = d;
      best = q;
    }
  }
  return best;
}

/** Convex hull of a point set (Andrew's monotone chain). Returns hull vertices in order. */
export function convexHull(points: Vec2[]): Vec2[] {
  const pts = [...points].sort((a, b) => a.x - b.x || a.y - b.y);
  if (pts.length < 3) return pts;
  const half = (src: Vec2[]): Vec2[] => {
    const h: Vec2[] = [];
    for (const p of src) {
      while (h.length >= 2 && cross(sub(h[h.length - 1], h[h.length - 2]), sub(p, h[h.length - 2])) <= 0) {
        h.pop();
      }
      h.push(p);
    }
    h.pop(); // drop the last point (it's the first of the other half)
    return h;
  };
  const lower = half(pts);
  const upper = half([...pts].reverse());
  return lower.concat(upper);
}

/** One corner's solved fillet: the tangent arc actually emitted (after all clamping). */
export interface FilletArc {
  center: Vec2;
  /** Actual radius after clamping (may be smaller than the requested one on tight shapes). */
  r: number;
  /** Arc start angle and signed sweep (the "short way" around the corner). */
  a1: number;
  da: number;
}

/** Requested radius of corner `i` — `radius` is uniform, or per-corner when an array. */
const cornerRadius = (radius: number | number[], i: number): number =>
  typeof radius === "number" ? radius : radius[i] ?? 0;

const maxCornerRadius = (radius: number | number[]): number =>
  typeof radius === "number" ? radius : radius.reduce((m, r) => Math.max(m, r), 0);

/**
 * Shared fillet solver behind `filletPolygon` / `filletCornerArcs`: per-corner half
 * interior angle + tangent length, clamped so fillets never overlap or poke through.
 */
function solveFillets(
  verts: Vec2[],
  radius: number | number[]
): { half: number[]; want: number[]; t: number[] } {
  const n = verts.length;

  // Pass 1 — per-corner geometry. `half` is half the interior angle; `want` is the
  // tangent length needed for the requested radius (0 for degenerate / near-straight
  // corners, which take no fillet and yield their whole edge to their neighbours).
  const half = new Array<number>(n).fill(0);
  const want = new Array<number>(n).fill(0);
  for (let i = 0; i < n; i++) {
    const r = cornerRadius(radius, i);
    if (r <= 0) continue; // this corner stays sharp
    const prev = verts[(i - 1 + n) % n];
    const v = verts[i];
    const next = verts[(i + 1) % n];
    const u1 = normalize(sub(prev, v)); // edge toward prev
    const u2 = normalize(sub(next, v)); // edge toward next
    const angle = Math.acos(Math.max(-1, Math.min(1, dot(u1, u2)))); // 0..π between edges
    if (angle < 1e-3 || angle > Math.PI - 1e-3) continue; // degenerate / nearly straight
    half[i] = angle / 2;
    want[i] = r / Math.tan(half[i]);
  }

  // Pass 2 — shared-edge budget. On every edge the two corners' tangent lengths must
  // fit within the edge, split in proportion to demand. Each corner's tangent is then
  // the smallest share its two edges allow, so neighbouring fillets can never overlap
  // (this is what prevents narrow shapes from pinching or folding over themselves).
  const tMax = new Array<number>(n).fill(Infinity);
  for (let i = 0; i < n; i++) {
    const a = i;
    const b = (i + 1) % n;
    const L = dist(verts[a], verts[b]);
    const sum = want[a] + want[b];
    const fit = sum > L && sum > 1e-9 ? L / sum : 1; // shrink both ends to fit the edge
    tMax[a] = Math.min(tMax[a], want[a] * fit);
    tMax[b] = Math.min(tMax[b], want[b] * fit);
  }
  const t = want.map((w, i) => Math.min(w, tMax[i]));

  // Pass 2b — keep a fillet from poking through a non-adjacent edge (the opposite side
  // of a thin feature, e.g. a narrow neck). The inscribed circle's centre must stay ≥ r
  // from every other edge; where it doesn't, shrink that corner's radius. Shrinking pulls
  // the centre back toward the vertex and re-opens clearance, so a few relaxation passes
  // settle each corner to the largest radius that still fits. This is what stops thin
  // shapes from folding over themselves at large radii.
  for (let pass = 0; pass < 6; pass++) {
    let changed = false;
    for (let i = 0; i < n; i++) {
      if (t[i] < 1e-6) continue;
      const v = verts[i];
      const u1 = normalize(sub(verts[(i - 1 + n) % n], v));
      const u2 = normalize(sub(verts[(i + 1) % n], v));
      const bis = normalize(add(u1, u2));
      const r = t[i] * Math.tan(half[i]);
      const center = add(v, scale(bis, r / Math.sin(half[i])));
      let minD = Infinity;
      for (let e = 0; e < n; e++) {
        if (e === i || e === (i - 1 + n) % n) continue; // skip the two edges meeting at vertex i
        minD = Math.min(minD, distToSegment(center, verts[e], verts[(e + 1) % n]));
      }
      if (minD < r - 1e-6) {
        t[i] = Math.max(0, minD) / Math.tan(half[i]);
        changed = true;
      }
    }
    if (!changed) break;
  }

  return { half, want, t };
}

/** The solved arc of corner `i` (from `solveFillets` output), or null when it takes no fillet. */
function filletArcAt(
  verts: Vec2[],
  i: number,
  half: number[],
  want: number[],
  t: number[]
): FilletArc | null {
  if (want[i] === 0 || t[i] < 1e-6) return null;
  const n = verts.length;
  const v = verts[i];
  const u1 = normalize(sub(verts[(i - 1 + n) % n], v));
  const u2 = normalize(sub(verts[(i + 1) % n], v));
  const r = t[i] * Math.tan(half[i]); // actual radius after clamping
  const t1 = add(v, scale(u1, t[i]));
  const t2 = add(v, scale(u2, t[i]));
  // The fillet centre lies along the bisector of the two edge directions. That bisector
  // points to the correct tangent-circle side for both convex corners (into the body)
  // and reflex corners (into the notch), so no per-corner flip is needed — the "short
  // way" arc sweep below then rounds each corner in the right direction.
  const bis = normalize(add(u1, u2));
  const center = add(v, scale(bis, r / Math.sin(half[i])));
  const a1 = Math.atan2(t1.y - center.y, t1.x - center.x);
  const a2 = Math.atan2(t2.y - center.y, t2.x - center.x);
  let da = a2 - a1; // sweep the short way
  while (da > Math.PI) da -= 2 * Math.PI;
  while (da < -Math.PI) da += 2 * Math.PI;
  return { center, r, a1, da };
}

/**
 * Round the corners of a simple polygon in place: each corner becomes a circular arc
 * tangent to its two edges, with radius clamped so adjacent fillets don't overlap.
 * Convex and reflex (concave) corners are both handled. `radius` is one uniform value,
 * or an array with one radius per corner (≤ 0 keeps that corner sharp). No positive
 * radius returns a copy. `segPerCorner` counts segments per 180° of sweep (the default
 * 24 samples arcs at 7.5° per segment, matching the DXF importer).
 */
export function filletPolygon(verts: Vec2[], radius: number | number[], segPerCorner = 24): Vec2[] {
  const n = verts.length;
  if (n < 3 || maxCornerRadius(radius) <= 0) return verts.map((v) => ({ x: v.x, y: v.y }));
  const { half, want, t } = solveFillets(verts, radius);

  // Emit the rounded outline (arc per filleted corner).
  const out: Vec2[] = [];
  for (let i = 0; i < n; i++) {
    const arc = filletArcAt(verts, i, half, want, t);
    if (!arc) {
      out.push({ x: verts[i].x, y: verts[i].y }); // no fillet here
      continue;
    }
    const steps = Math.max(1, Math.round((segPerCorner * Math.abs(arc.da)) / Math.PI));
    for (let s = 0; s <= steps; s++) {
      const a = arc.a1 + (arc.da * s) / steps;
      out.push({ x: arc.center.x + arc.r * Math.cos(a), y: arc.center.y + arc.r * Math.sin(a) });
    }
  }
  return out;
}

/**
 * The solved fillet arc of every corner of `filletPolygon(verts, radius)` — same
 * clamping, no sampling. `null` where a corner takes no fillet. Used by the editor to
 * place per-corner radius handles exactly on the drawn arcs.
 */
export function filletCornerArcs(verts: Vec2[], radius: number | number[]): (FilletArc | null)[] {
  const n = verts.length;
  if (n < 3 || maxCornerRadius(radius) <= 0) return verts.map(() => null);
  const { half, want, t } = solveFillets(verts, radius);
  return verts.map((_, i) => filletArcAt(verts, i, half, want, t));
}

/**
 * The convex hull of `points`, expanded outward by `margin` with rounded corners —
 * i.e. the Minkowski sum of their hull with a disk, built as the hull of circles
 * (sampled into `segments` points) placed at each point. The corners are true circular
 * arcs; `segments` only sets how finely they're sampled. Handles 1 point (a disk),
 * 2 (a stadium), or many. The default sampling is 48 facets per full circle (7.5° per
 * segment, matching the fillet sampling and the DXF importer). `margin` is one uniform
 * value, or an array with one margin per point (the hull of different-size circles;
 * ≤ 0 keeps that point bare).
 */
export function roundedConvexBody(points: Vec2[], margin: number | number[], segments?: number): Vec2[] {
  const mAt = (i: number): number => cornerRadius(margin, i);
  const n = segments ?? 48;
  const cloud: Vec2[] = [];
  points.forEach((p, i) => {
    const m = mAt(i);
    if (m <= 0) {
      cloud.push({ x: p.x, y: p.y });
      return;
    }
    for (let j = 0; j < n; j++) {
      const a = (j / n) * Math.PI * 2;
      cloud.push({ x: p.x + m * Math.cos(a), y: p.y + m * Math.sin(a) });
    }
  });
  return convexHull(cloud);
}

/** A circular arc: centre, radius, start angle and signed sweep (radians, screen-y-down). */
export interface Arc {
  c: Vec2;
  r: number;
  a0: number;
  sweep: number;
}

/**
 * The arc from `a` through `m` to `b` (the unique circle through three points, swept
 * the way that passes `m`), or null when the points are (near-)collinear.
 */
export function arcThrough(a: Vec2, m: Vec2, b: Vec2): Arc | null {
  const d = 2 * (a.x * (m.y - b.y) + m.x * (b.y - a.y) + b.x * (a.y - m.y));
  const span = Math.max(dist(a, b), dist(a, m), dist(m, b));
  if (Math.abs(d) < 1e-9 * span * span || span < 1e-12) return null;
  const a2 = lenSq(a), m2 = lenSq(m), b2 = lenSq(b);
  const c = vec(
    (a2 * (m.y - b.y) + m2 * (b.y - a.y) + b2 * (a.y - m.y)) / d,
    (a2 * (b.x - m.x) + m2 * (a.x - b.x) + b2 * (m.x - a.x)) / d
  );
  const r = dist(c, a);
  const ang = (p: Vec2): number => Math.atan2(p.y - c.y, p.x - c.x);
  const a0 = ang(a);
  const am = ang(m);
  const ab = ang(b);
  const tau = Math.PI * 2;
  // Counter-clockwise (increasing angle) sweep from a to b, and where m sits along it.
  const ccwB = ((ab - a0) % tau + tau) % tau;
  const ccwM = ((am - a0) % tau + tau) % tau;
  const sweep = ccwM <= ccwB ? ccwB : ccwB - tau; // m on the ccw way → ccw, else the other way round
  return { c, r, a0, sweep };
}

/** `n` points along an arc (its endpoints included, `n` ≥ 2). */
export function sampleArc(arc: Arc, n: number): Vec2[] {
  const out: Vec2[] = [];
  const k = Math.max(2, n);
  for (let i = 0; i < k; i++) {
    const t = arc.a0 + (arc.sweep * i) / (k - 1);
    out.push(vec(arc.c.x + arc.r * Math.cos(t), arc.c.y + arc.r * Math.sin(t)));
  }
  return out;
}

/** Distance from `p` to the nearest point of an arc (its endpoints included). */
export function distToArc(p: Vec2, arc: Arc): number {
  const ang = Math.atan2(p.y - arc.c.y, p.x - arc.c.x);
  const tau = Math.PI * 2;
  const rel = ((ang - arc.a0) % tau + tau) % tau; // ccw offset from the start
  const on = arc.sweep >= 0 ? rel <= arc.sweep : rel >= tau + arc.sweep;
  if (on) return Math.abs(dist(p, arc.c) - arc.r);
  const end = vec(arc.c.x + arc.r * Math.cos(arc.a0 + arc.sweep), arc.c.y + arc.r * Math.sin(arc.a0 + arc.sweep));
  const start = vec(arc.c.x + arc.r * Math.cos(arc.a0), arc.c.y + arc.r * Math.sin(arc.a0));
  return Math.min(dist(p, start), dist(p, end));
}

/** The `n` vertices of a regular polygon centred on `c` with one vertex at `first`. */
export function regularPolygon(c: Vec2, first: Vec2, n: number): Vec2[] {
  const k = Math.max(3, Math.round(n));
  const r = dist(c, first);
  const a0 = Math.atan2(first.y - c.y, first.x - c.x);
  const out: Vec2[] = [];
  for (let i = 0; i < k; i++) {
    const t = a0 + (i / k) * Math.PI * 2;
    out.push(vec(c.x + r * Math.cos(t), c.y + r * Math.sin(t)));
  }
  return out;
}
