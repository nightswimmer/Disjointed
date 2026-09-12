/** View transform with rotation: screen ↔ world round trips, the canvas matrix, zoom and
 *  rotation pivots, the visible world rectangle, angle wrapping. */
import {
  View, screenToWorld, worldToScreen, zoomAt, rotateViewTo, rotateToScreen, rotateToWorld,
  viewMatrix, visibleWorldRect, wrapAngle,
} from "../src/view";
import { Vec2, dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: Vec2, b: Vec2, tol = 1e-9) => dist(a, b) < tol;
const fmt = (p: Vec2) => `(${p.x.toFixed(4)}, ${p.y.toFixed(4)})`;
const deg = (d: number) => (d * Math.PI) / 180;

// Unrotated view behaves exactly as before.
const v0: View = { scale: 2, tx: 100, ty: 50, angle: 0 };
check("worldToScreen at 0°", near(worldToScreen(v0, { x: 10, y: -5 }), { x: 120, y: 40 }));
check("screenToWorld at 0°", near(screenToWorld(v0, { x: 120, y: 40 }), { x: 10, y: -5 }));

// Rotation direction: +90° turns the picture counter-clockwise on screen, so world +x
// (right at 0°) points up (screen -y).
const v90: View = { scale: 1, tx: 0, ty: 0, angle: deg(90) };
check("+90° sends world +x up the screen", near(worldToScreen(v90, { x: 1, y: 0 }), { x: 0, y: -1 }), fmt(worldToScreen(v90, { x: 1, y: 0 })));
check("+90° sends world +y (down at 0°) to the right", near(worldToScreen(v90, { x: 0, y: 1 }), { x: 1, y: 0 }));
check("rotateToWorld inverts rotateToScreen", near(rotateToWorld(rotateToScreen({ x: 3, y: -7 }, 0.7), 0.7), { x: 3, y: -7 }));

// Round trip at an arbitrary angle, scale and offset.
const v: View = { scale: 1.7, tx: 233, ty: -41, angle: deg(37) };
for (const p of [{ x: 0, y: 0 }, { x: 12.5, y: -80 }, { x: -1000, y: 3 }]) {
  check(`round trip ${fmt(p)}`, near(screenToWorld(v, worldToScreen(v, p)), p, 1e-9));
}
// The mapping is a similarity: distances scale uniformly, whatever the angle.
const a = worldToScreen(v, { x: 0, y: 0 });
const b = worldToScreen(v, { x: 3, y: 4 });
check("rotation keeps lengths (× scale)", Math.abs(dist(a, b) - 5 * 1.7) < 1e-9);

// The canvas matrix agrees with worldToScreen (dpr folded in).
const m = viewMatrix(v, 2);
const via = (p: Vec2) => ({ x: (m[0] * p.x + m[2] * p.y + m[4]) / 2, y: (m[1] * p.x + m[3] * p.y + m[5]) / 2 });
check("viewMatrix matches worldToScreen", near(via({ x: 12.5, y: -80 }), worldToScreen(v, { x: 12.5, y: -80 }), 1e-9));

// zoomAt keeps the anchored world point under the anchor at any angle.
const vz: View = { ...v };
const anchor = { x: 400, y: 300 };
const under = screenToWorld(vz, anchor);
zoomAt(vz, anchor, 1.8);
check("zoomAt scales", Math.abs(vz.scale - 1.7 * 1.8) < 1e-12);
check("zoomAt keeps the anchor's world point (rotated view)", near(worldToScreen(vz, under), anchor, 1e-9), fmt(worldToScreen(vz, under)));
check("zoomAt keeps the angle", vz.angle === v.angle);

// rotateViewTo keeps the pivot's world point fixed and sets the angle.
const vr: View = { scale: 1.7, tx: 233, ty: -41, angle: 0 };
const pivot = { x: 512, y: 384 };
const pivotWorld = screenToWorld(vr, pivot);
rotateViewTo(vr, pivot, deg(35));
check("rotateViewTo sets the angle", Math.abs(vr.angle - deg(35)) < 1e-12);
check("rotateViewTo keeps the pivot's world point", near(worldToScreen(vr, pivotWorld), pivot, 1e-9), fmt(worldToScreen(vr, pivotWorld)));
check("rotateViewTo keeps the scale", vr.scale === 1.7);
// A world point off the pivot moves on a circle about the pivot by the turn.
const off = worldToScreen(vr, { x: pivotWorld.x + 10, y: pivotWorld.y });
check("off-pivot point stays at the same screen distance", Math.abs(dist(off, pivot) - 17) < 1e-9);
check("...and turned counter-clockwise by 35°", Math.abs(-Math.atan2(off.y - pivot.y, off.x - pivot.x) - deg(35)) < 1e-9);
rotateViewTo(vr, pivot, 0);
check("rotating back restores the original view", Math.abs(vr.tx - 233) < 1e-9 && Math.abs(vr.ty + 41) < 1e-9);

// Visible world rect: at 0° the screen rectangle itself; rotated, the bounding box of the
// turned rectangle (a superset that still contains the screen's world corners).
const r0 = visibleWorldRect({ scale: 2, tx: 100, ty: 50, angle: 0 }, 800, 600);
check("visible rect at 0°", Math.abs(r0.left + 50) < 1e-9 && Math.abs(r0.top + 25) < 1e-9 && Math.abs(r0.right - 350) < 1e-9 && Math.abs(r0.bottom - 275) < 1e-9);
const r45 = visibleWorldRect({ scale: 1, tx: 400, ty: 300, angle: deg(45) }, 800, 600);
const diag = Math.hypot(800, 600) / 2;
check("visible rect at 45° spans the rotated corners", Math.abs(r45.right - r45.left - (800 + 600) / Math.SQRT2) < 1e-6, `width=${(r45.right - r45.left).toFixed(3)}`);
check("visible rect at 45° is centred on the screen centre's world point", Math.abs((r45.left + r45.right) / 2) < 1e-9 && Math.abs((r45.top + r45.bottom) / 2) < 1e-9);
check("visible rect contains every screen corner", [
  { x: 0, y: 0 }, { x: 800, y: 0 }, { x: 0, y: 600 }, { x: 800, y: 600 },
].every((s) => {
  const w = screenToWorld({ scale: 1, tx: 400, ty: 300, angle: deg(45) }, s);
  return w.x >= r45.left - 1e-9 && w.x <= r45.right + 1e-9 && w.y >= r45.top - 1e-9 && w.y <= r45.bottom + 1e-9 && Math.hypot(w.x, w.y) <= diag + 1e-9;
}));

// wrapAngle lands in (-π, π].
check("wrapAngle(3π) = π", Math.abs(wrapAngle(3 * Math.PI) - Math.PI) < 1e-12);
check("wrapAngle(-π) = π", Math.abs(wrapAngle(-Math.PI) - Math.PI) < 1e-12);
check("wrapAngle(370°) = 10°", Math.abs(wrapAngle(deg(370)) - deg(10)) < 1e-12);
check("wrapAngle(-190°) = 170°", Math.abs(wrapAngle(deg(-190)) - deg(170)) < 1e-12);

// The 5° snap the dial applies while dragging (mirrors main's rounding).
const SNAP = deg(5);
const snap = (x: number) => Math.round(x / SNAP) * SNAP;
check("snap 37.4° → 35°", Math.abs(snap(deg(37.4)) - deg(35)) < 1e-12);
check("snap 37.6° → 40°", Math.abs(snap(deg(37.6)) - deg(40)) < 1e-12);
check("snap -2.4° → 0°", Math.abs(snap(deg(-2.4))) < 1e-12);

console.log(failures ? `\n${failures} FAILED` : "\nALL PASSED");
if (failures) process.exit(1);
