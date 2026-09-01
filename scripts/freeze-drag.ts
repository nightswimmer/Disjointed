/**
 * Headless coverage for scoped solves (`SolveFreeze`) — the engine behind draw mode's
 * rigid (Shift) drag. Verifies that:
 *  - Frozen bodies and frozen free joints never move, no matter what is dragged.
 *  - The movable selection still obeys its constraints: it pivots about a pin to a
 *    frozen body, keeps its ground anchors exactly, and a rider stays on a frozen rail.
 *  - A still-open pin from the selection to the frozen world snaps closed when the
 *    drag solves (assemble-on-drag).
 *  - Constraints entirely outside the movable set are out of scope: an open pin
 *    between two frozen bodies is neither closed nor reported as a break.
 *  - The same open pin WITHOUT a freeze is closed by a normal solve (the skip is
 *    freeze-only).
 */
import { Scene } from "../src/model";
import { solve, SolveFreeze } from "../src/solver";
import { dist, Vec2 } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

/** A 100x100 square body centred at (cx, cy). */
function square(scene: Scene, cx: number, cy: number) {
  return scene.addBody([
    { x: cx - 50, y: cy - 50 },
    { x: cx + 50, y: cy - 50 },
    { x: cx + 50, y: cy + 50 },
    { x: cx - 50, y: cy + 50 },
  ]);
}

function pose(scene: Scene, id: number): { pos: Vec2; angle: number } {
  const b = scene.getBody(id)!;
  return { pos: { x: b.pos.x, y: b.pos.y }, angle: b.angle };
}

function moved(a: { pos: Vec2; angle: number }, b: { pos: Vec2; angle: number }): number {
  return dist(a.pos, b.pos) + Math.abs(a.angle - b.angle) * 100;
}

/** Freeze everything in the scene except the given bodies / free joints. */
function freezeAllBut(scene: Scene, bodies: number[], joints: number[] = []): SolveFreeze {
  const keepB = new Set(bodies);
  const keepJ = new Set(joints);
  return {
    bodies: new Set(scene.bodies.filter((b) => !keepB.has(b.id)).map((b) => b.id)),
    joints: new Set(
      scene.joints.filter((j) => j.bodyId === null && !keepJ.has(j.id)).map((j) => j.id)
    ),
  };
}

// --- dragging pivots about a pin to a frozen body; the frozen body never moves ---
{
  const scene = new Scene();
  const a = square(scene, 200, 200); // will be frozen
  const b = scene.addBody([
    { x: 230, y: 180 }, { x: 370, y: 180 }, { x: 370, y: 220 }, { x: 230, y: 220 },
  ]);
  const ja = scene.addJoint(a.id, { x: 240, y: 200 });
  const jb = scene.addJoint(b.id, { x: 240, y: 200 });
  const handle = scene.addJoint(b.id, { x: 360, y: 200 }); // 120 from the pin
  scene.addPin(ja.id, jb.id);
  const beforeA = pose(scene, a.id);
  const freeze = freezeAllBut(scene, [b.id]);
  const breaks = solve(scene, { jointId: handle.id, target: { x: 240, y: 320 } }, 200, 1, undefined, undefined, freeze);
  check("frozen body never moves", moved(beforeA, pose(scene, a.id)) < 1e-9,
    `delta ${moved(beforeA, pose(scene, a.id)).toExponential(2)}`);
  check("dragged body pivots about the pin", dist(scene.jointWorld(handle), { x: 240, y: 320 }) < 1,
    `off by ${dist(scene.jointWorld(handle), { x: 240, y: 320 }).toFixed(3)} px`);
  check("pin to the frozen body holds", dist(scene.jointWorld(ja), scene.jointWorld(jb)) < 0.5,
    `gap ${dist(scene.jointWorld(ja), scene.jointWorld(jb)).toFixed(4)} px`);
  check("scoped drag reports no breaks", breaks.length === 0, `${breaks.length} break(s)`);
}

// --- ground anchors hold during a scoped drag (the point of rigid dragging) ---
{
  const scene = new Scene();
  const a = square(scene, 200, 200);
  const g = scene.addJoint(a.id, { x: 200, y: 200 });
  const handle = scene.addJoint(a.id, { x: 240, y: 200 });
  scene.addGround(g.id, { x: 200, y: 200 });
  const other = square(scene, 600, 600); // unrelated frozen body
  const beforeOther = pose(scene, other.id);
  const freeze = freezeAllBut(scene, [a.id]);
  solve(scene, { jointId: handle.id, target: { x: 200, y: 400 } }, 200, 1, undefined, undefined, freeze);
  check("ground anchor never moves in a scoped drag", dist(scene.jointWorld(g), { x: 200, y: 200 }) < 1e-6,
    `drift ${dist(scene.jointWorld(g), { x: 200, y: 200 }).toExponential(2)}`);
  check("body pivoted about its ground", dist(scene.jointWorld(handle), { x: 200, y: 240 }) < 0.5,
    `handle at (${scene.jointWorld(handle).x.toFixed(2)}, ${scene.jointWorld(handle).y.toFixed(2)})`);
  check("unrelated frozen body untouched", moved(beforeOther, pose(scene, other.id)) < 1e-9,
    `delta ${moved(beforeOther, pose(scene, other.id)).toExponential(2)}`);
}

// --- a frozen free joint acts as an anchor; an open pin to it snaps closed ---
{
  const scene = new Scene();
  const b = square(scene, 400, 200);
  const jb = scene.addJoint(b.id, { x: 360, y: 200 });
  const anchor = scene.addFreeJoint({ x: 300, y: 200 }); // drawn 60 apart from jb (open pin)
  scene.addPin(jb.id, anchor.id);
  const freeze = freezeAllBut(scene, [b.id]);
  solve(scene, { jointId: scene.addJoint(b.id, { x: 440, y: 200 }).id, target: { x: 440, y: 200 } }, 200, 1, undefined, undefined, freeze);
  check("frozen free joint never moves", dist(scene.jointWorld(anchor), { x: 300, y: 200 }) < 1e-9,
    `at (${scene.jointWorld(anchor).x.toFixed(2)}, ${scene.jointWorld(anchor).y.toFixed(2)})`);
  check("open pin to the frozen world snaps closed", dist(scene.jointWorld(jb), { x: 300, y: 200 }) < 0.5,
    `gap ${dist(scene.jointWorld(jb), { x: 300, y: 200 }).toFixed(4)} px`);
}

// --- a rider dragged along a frozen body's rail stays on the segment ---
{
  const scene = new Scene();
  const railBody = square(scene, 200, 200);
  const r1 = scene.addJoint(railBody.id, { x: 160, y: 200 });
  const r2 = scene.addJoint(railBody.id, { x: 240, y: 200 });
  const rider = scene.addFreeJoint({ x: 200, y: 200 });
  const s = scene.addSlider(r1.id, r2.id);
  scene.attachSliderRider(s.id, rider.id);
  const before = pose(scene, railBody.id);
  const freeze = freezeAllBut(scene, [], [rider.id]);
  solve(scene, { jointId: rider.id, target: { x: 400, y: 300 } }, 200, 1, undefined, undefined, freeze);
  check("frozen rail body stays put under a rider drag", moved(before, pose(scene, railBody.id)) < 1e-9,
    `delta ${moved(before, pose(scene, railBody.id)).toExponential(2)}`);
  check("rider clamps to the frozen rail's end-stop", dist(scene.jointWorld(rider), { x: 240, y: 200 }) < 0.5,
    `rider at (${scene.jointWorld(rider).x.toFixed(2)}, ${scene.jointWorld(rider).y.toFixed(2)})`);
}

// --- constraints entirely inside the frozen world are out of scope ---
{
  const scene = new Scene();
  const a = square(scene, 200, 200);
  const c = square(scene, 600, 200);
  const mover = square(scene, 200, 600);
  const ja = scene.addJoint(a.id, { x: 200, y: 200 });
  const jc = scene.addJoint(c.id, { x: 600, y: 200 });
  scene.addPin(ja.id, jc.id); // open pin between two soon-frozen bodies
  const handle = scene.addJoint(mover.id, { x: 200, y: 600 });
  const beforeA = pose(scene, a.id);
  const beforeC = pose(scene, c.id);
  const freeze = freezeAllBut(scene, [mover.id]);
  const breaks = solve(scene, { jointId: handle.id, target: { x: 300, y: 700 } }, 200, 1, undefined, undefined, freeze);
  check("open pin between frozen bodies is not closed",
    moved(beforeA, pose(scene, a.id)) < 1e-9 && moved(beforeC, pose(scene, c.id)) < 1e-9,
    "both frozen bodies unmoved");
  check("...and not reported as a break", breaks.length === 0, `${breaks.length} break(s)`);
  check("the movable body still followed the drag", dist(scene.jointWorld(handle), { x: 300, y: 700 }) < 0.5,
    `handle at (${scene.jointWorld(handle).x.toFixed(2)}, ${scene.jointWorld(handle).y.toFixed(2)})`);

  // Sanity: without the freeze, a normal solve closes that same pin (the skip is freeze-only).
  const plain = solve(scene, null, 200);
  check("without a freeze the same pin closes", dist(scene.jointWorld(ja), scene.jointWorld(jc)) < 0.5,
    `gap ${dist(scene.jointWorld(ja), scene.jointWorld(jc)).toFixed(4)} px`);
  check("...with no breaks", plain.length === 0, `${plain.length} break(s)`);
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
