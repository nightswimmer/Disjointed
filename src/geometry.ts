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

/**
 * Round the corners of a simple polygon in place: each corner becomes a circular arc
 * tangent to its two edges, with radius clamped so adjacent fillets don't overlap.
 * Convex and reflex (concave) corners are both handled. `radius <= 0` returns a copy.
 */
export function filletPolygon(verts: Vec2[], radius: number, segPerCorner = 8): Vec2[] {
  const n = verts.length;
  if (n < 3 || radius <= 0) return verts.map((v) => ({ x: v.x, y: v.y }));

  // Pass 1 — per-corner geometry. `half` is half the interior angle; `want` is the
  // tangent length needed for the requested radius (0 for degenerate / near-straight
  // corners, which take no fillet and yield their whole edge to their neighbours).
  const half = new Array<number>(n).fill(0);
  const want = new Array<number>(n).fill(0);
  for (let i = 0; i < n; i++) {
    const prev = verts[(i - 1 + n) % n];
    const v = verts[i];
    const next = verts[(i + 1) % n];
    const u1 = normalize(sub(prev, v)); // edge toward prev
    const u2 = normalize(sub(next, v)); // edge toward next
    const angle = Math.acos(Math.max(-1, Math.min(1, dot(u1, u2)))); // 0..π between edges
    if (angle < 1e-3 || angle > Math.PI - 1e-3) continue; // degenerate / nearly straight
    half[i] = angle / 2;
    want[i] = radius / Math.tan(half[i]);
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

  // Pass 3 — emit the rounded outline (arc per filleted corner).
  const out: Vec2[] = [];
  for (let i = 0; i < n; i++) {
    const prev = verts[(i - 1 + n) % n];
    const v = verts[i];
    const next = verts[(i + 1) % n];
    if (want[i] === 0 || t[i] < 1e-6) {
      out.push({ x: v.x, y: v.y }); // no fillet here
      continue;
    }
    const u1 = normalize(sub(prev, v));
    const u2 = normalize(sub(next, v));
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
    const steps = Math.max(1, Math.round((segPerCorner * Math.abs(da)) / Math.PI));
    for (let s = 0; s <= steps; s++) {
      const a = a1 + (da * s) / steps;
      out.push({ x: center.x + r * Math.cos(a), y: center.y + r * Math.sin(a) });
    }
  }
  return out;
}

/**
 * The convex hull of `points`, expanded outward by `margin` with rounded corners —
 * i.e. the Minkowski sum of their hull with a disk, built as the hull of circles
 * (sampled into `segments` points) placed at each point. The corners are true circular
 * arcs; `segments` only sets how finely they're sampled. Handles 1 point (a disk),
 * 2 (a stadium), or many. When `segments` is omitted it scales with `margin` so the
 * arc facets stay small (smooth) at any size.
 */
export function roundedConvexBody(points: Vec2[], margin: number, segments?: number): Vec2[] {
  // ~1 facet per 4 world units of circumference, clamped to a sensible range.
  const n = segments ?? Math.max(24, Math.min(96, Math.round((Math.PI * margin) / 2)));
  const cloud: Vec2[] = [];
  for (const p of points) {
    for (let j = 0; j < n; j++) {
      const a = (j / n) * Math.PI * 2;
      cloud.push({ x: p.x + margin * Math.cos(a), y: p.y + margin * Math.sin(a) });
    }
  }
  return convexHull(cloud);
}
