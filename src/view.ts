/**
 * Camera/view transform mapping world coordinates to screen (CSS) pixels:
 *     screen = R(angle) * world * scale + (tx, ty)
 * where R(angle) turns the picture counter-clockwise on screen by `angle` radians (screen
 * y points down, so the matrix is [cos sin; -sin cos]). Used by the renderer (to draw)
 * and by main (to interpret pointer input). The rotation is purely visual: the document,
 * its H/V axes, grid and snapping all stay in world axes and turn with the picture.
 */
import { Vec2 } from "./geometry";

export interface View {
  scale: number;
  tx: number;
  ty: number;
  /** View rotation (radians, counter-clockwise on screen; 0 = world x to the right). */
  angle: number;
}

export const MIN_SCALE = 0.05;
export const MAX_SCALE = 200;

/** Turn a world-frame vector into the screen frame (rotation only, no scale / offset). */
export function rotateToScreen(v: Vec2, angle: number): Vec2 {
  const c = Math.cos(angle), s = Math.sin(angle);
  return { x: c * v.x + s * v.y, y: -s * v.x + c * v.y };
}

/** Inverse of `rotateToScreen`. */
export function rotateToWorld(v: Vec2, angle: number): Vec2 {
  const c = Math.cos(angle), s = Math.sin(angle);
  return { x: c * v.x - s * v.y, y: s * v.x + c * v.y };
}

export function screenToWorld(view: View, s: Vec2): Vec2 {
  const r = rotateToWorld({ x: s.x - view.tx, y: s.y - view.ty }, view.angle);
  return { x: r.x / view.scale, y: r.y / view.scale };
}

export function worldToScreen(view: View, w: Vec2): Vec2 {
  const r = rotateToScreen(w, view.angle);
  return { x: r.x * view.scale + view.tx, y: r.y * view.scale + view.ty };
}

/** Zoom by `factor` while keeping the world point under `anchor` (screen px) fixed. */
export function zoomAt(view: View, anchor: Vec2, factor: number): void {
  const before = screenToWorld(view, anchor);
  view.scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, view.scale * factor));
  const r = rotateToScreen(before, view.angle);
  view.tx = anchor.x - r.x * view.scale;
  view.ty = anchor.y - r.y * view.scale;
}

/** Rotate the view to `angle` while keeping the world point under `pivot` (screen px) fixed. */
export function rotateViewTo(view: View, pivot: Vec2, angle: number): void {
  const before = screenToWorld(view, pivot);
  view.angle = angle;
  const r = rotateToScreen(before, view.angle);
  view.tx = pivot.x - r.x * view.scale;
  view.ty = pivot.y - r.y * view.scale;
}

/** Wrap an angle into (-π, π]. */
export function wrapAngle(a: number): number {
  a = a % (2 * Math.PI);
  if (a <= -Math.PI) a += 2 * Math.PI;
  else if (a > Math.PI) a -= 2 * Math.PI;
  return a;
}

/** The 2D canvas transform (a, b, c, d, e, f) for drawing world coordinates, times `dpr`. */
export function viewMatrix(view: View, dpr: number): [number, number, number, number, number, number] {
  const k = view.scale * dpr;
  const c = Math.cos(view.angle), s = Math.sin(view.angle);
  // x' = c·x + s·y + tx ; y' = -s·x + c·y + ty  ⇒  a = c, b = -s, c = s, d = c.
  return [k * c, -k * s, k * s, k * c, dpr * view.tx, dpr * view.ty];
}

/** Axis-aligned world bounding box of the screen rectangle (w × h px) — what the view shows. */
export function visibleWorldRect(view: View, w: number, h: number): { left: number; top: number; right: number; bottom: number } {
  const corners = [
    screenToWorld(view, { x: 0, y: 0 }),
    screenToWorld(view, { x: w, y: 0 }),
    screenToWorld(view, { x: 0, y: h }),
    screenToWorld(view, { x: w, y: h }),
  ];
  return {
    left: Math.min(...corners.map((p) => p.x)),
    top: Math.min(...corners.map((p) => p.y)),
    right: Math.max(...corners.map((p) => p.x)),
    bottom: Math.max(...corners.map((p) => p.y)),
  };
}
