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
