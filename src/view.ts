/**
 * Camera/view transform mapping world coordinates to screen (CSS) pixels:
 *     screen = world * scale + (tx, ty)
 * Used by the renderer (to draw) and by main (to interpret pointer input).
 */
import { Vec2 } from "./geometry";

export interface View {
  scale: number;
  tx: number;
  ty: number;
}

export const MIN_SCALE = 0.2;
export const MAX_SCALE = 5;

export function screenToWorld(view: View, s: Vec2): Vec2 {
  return { x: (s.x - view.tx) / view.scale, y: (s.y - view.ty) / view.scale };
}

export function worldToScreen(view: View, w: Vec2): Vec2 {
  return { x: w.x * view.scale + view.tx, y: w.y * view.scale + view.ty };
}

/** Zoom by `factor` while keeping the world point under `anchor` (screen px) fixed. */
export function zoomAt(view: View, anchor: Vec2, factor: number): void {
  const before = screenToWorld(view, anchor);
  view.scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, view.scale * factor));
  view.tx = anchor.x - before.x * view.scale;
  view.ty = anchor.y - before.y * view.scale;
}
