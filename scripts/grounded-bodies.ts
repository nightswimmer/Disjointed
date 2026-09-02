/**
 * Headless coverage for grounded bodies (and grounded groups). Verifies that:
 *  - toggleBodyGround toggles a lone body, and grounds/ungrounds a whole group at once.
 *  - In simulation a grounded body is immovable: dragging its joints does nothing, a
 *    body pinned to it pivots about the pin while the grounded body stays put, and a
 *    slider rail on a grounded body is a fixed track its riders still slide along.
 *  - An unreachable pin onto a grounded body is reported as a break, not "solved" by
 *    dragging the grounded body.
 *  - serialize/load round-trips the flag (v10); legacy files without it load as false.
 *  - Copy/paste (extractSelection/insertSelection) carries the flag.
 */
import { Scene, SceneData } from "../src/model";
import { solve } from "../src/solver";
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

// --- toggle semantics -------------------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 200, 200);
  check("new body starts ungrounded", !a.grounded, `grounded=${a.grounded}`);
  scene.toggleBodyGround(a.id);
  check("toggle grounds a lone body", a.grounded, `grounded=${a.grounded}`);
  scene.toggleBodyGround(a.id);
  check("toggle ungrounds it again", !a.grounded, `grounded=${a.grounded}`);
  check("toggle on unknown body is a no-op", !scene.toggleBodyGround(9999), "returned false");

  const b = square(scene, 400, 200);
  scene.addGroup([a.id, b.id]);
  scene.toggleBodyGround(a.id);
  check("grounding a group member grounds the whole group", a.grounded && b.grounded,
    `a=${a.grounded} b=${b.grounded}`);
  scene.toggleBodyGround(b.id);
  check("ungrounding via any member ungrounds the whole group", !a.grounded && !b.grounded,
    `a=${a.grounded} b=${b.grounded}`);
}

// --- a grounded body is immovable under drag --------------------------------
{
  const scene = new Scene();
  const a = square(scene, 200, 200);
  const handle = scene.addJoint(a.id, { x: 240, y: 200 });
  scene.toggleBodyGround(a.id);
  const before = pose(scene, a.id);
  solve(scene, { jointId: handle.id, target: { x: 500, y: 500 } }, 120);
  check("dragging a grounded body's joint moves nothing", moved(before, pose(scene, a.id)) < 1e-9,
    `delta ${moved(before, pose(scene, a.id)).toExponential(2)}`);
}

// --- a body pinned to a grounded body pivots about the pin ------------------
{
  const scene = new Scene();
  const a = square(scene, 200, 200);
  const b = scene.addBody([
    { x: 230, y: 180 }, { x: 370, y: 180 }, { x: 370, y: 220 }, { x: 230, y: 220 },
  ]);
  const ja = scene.addJoint(a.id, { x: 240, y: 200 });
  const jb = scene.addJoint(b.id, { x: 240, y: 200 });
  const handle = scene.addJoint(b.id, { x: 360, y: 200 }); // 120 from the pin
  scene.addPin(ja.id, jb.id);
  scene.toggleBodyGround(a.id);
  const beforeA = pose(scene, a.id);
  const breaks = solve(scene, { jointId: handle.id, target: { x: 240, y: 320 } }, 200);
  const pinDrift = dist(scene.jointWorld(ja), { x: 240, y: 200 });
  check("pinned drag leaves the grounded body untouched", moved(beforeA, pose(scene, a.id)) < 1e-9,
    `delta ${moved(beforeA, pose(scene, a.id)).toExponential(2)}`);
  check("the pin stays on the grounded body", pinDrift < 0.5, `drift ${pinDrift.toFixed(4)} px`);
  check("pin coincidence holds", dist(scene.jointWorld(ja), scene.jointWorld(jb)) < 0.5,
    `gap ${dist(scene.jointWorld(ja), scene.jointWorld(jb)).toFixed(4)} px`);
  check("the free body pivoted to the target", dist(scene.jointWorld(handle), { x: 240, y: 320 }) < 1,
    `off by ${dist(scene.jointWorld(handle), { x: 240, y: 320 }).toFixed(3)} px`);
  check("a reachable pinned drag reports no breaks", breaks.length === 0, `${breaks.length} break(s)`);
}

// --- a slider rail on a grounded body is a fixed track ----------------------
{
  const scene = new Scene();
  const a = square(scene, 200, 200);
  const r1 = scene.addJoint(a.id, { x: 160, y: 200 });
  const r2 = scene.addJoint(a.id, { x: 240, y: 200 });
  const rider = scene.addFreeJoint({ x: 200, y: 200 });
  const s = scene.addSlider(r1.id, r2.id);
  scene.attachSliderRider(s.id, rider.id);
  scene.toggleBodyGround(a.id);
  const before = pose(scene, a.id);
  solve(scene, { jointId: rider.id, target: { x: 300, y: 260 } }, 120);
  const q = scene.jointWorld(rider);
  check("rail body stays fixed while the rider is dragged", moved(before, pose(scene, a.id)) < 1e-9,
    `delta ${moved(before, pose(scene, a.id)).toExponential(2)}`);
  check("rider clamps to the fixed rail's end-stop", dist(q, { x: 240, y: 200 }) < 0.5,
    `rider at (${q.x.toFixed(2)}, ${q.y.toFixed(2)})`);
}

// --- an unreachable pin onto grounded bodies is a break ---------------------
{
  const scene = new Scene();
  const a = square(scene, 200, 200);
  const c = square(scene, 600, 200);
  const ja = scene.addJoint(a.id, { x: 200, y: 200 });
  const jc = scene.addJoint(c.id, { x: 600, y: 200 });
  scene.addPin(ja.id, jc.id);
  scene.toggleBodyGround(a.id);
  scene.toggleBodyGround(c.id);
  const beforeA = pose(scene, a.id);
  const beforeC = pose(scene, c.id);
  const breaks = solve(scene, null, 120);
  check("pin between two grounded bodies apart is reported broken", breaks.length === 1,
    `${breaks.length} break(s)`);
  check("neither grounded body moved to fake it",
    moved(beforeA, pose(scene, a.id)) < 1e-9 && moved(beforeC, pose(scene, c.id)) < 1e-9,
    "both poses unchanged");
}

// --- persistence + copy/paste ------------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 200, 200);
  const b = square(scene, 400, 200);
  scene.toggleBodyGround(a.id);
  const data = scene.serialize();
  check("serialize is format v13", data.version === 13, `version ${data.version}`);

  const loaded = new Scene();
  loaded.load(JSON.parse(JSON.stringify(data)) as SceneData);
  check("load round-trips the grounded flag",
    loaded.getBody(a.id)!.grounded === true && loaded.getBody(b.id)!.grounded === false,
    `a=${loaded.getBody(a.id)!.grounded} b=${loaded.getBody(b.id)!.grounded}`);

  // Legacy file: no grounded field at all — every body loads ungrounded.
  const legacy = JSON.parse(JSON.stringify(data)) as SceneData;
  legacy.version = 9;
  for (const lb of legacy.bodies) delete (lb as { grounded?: boolean }).grounded;
  const oldScene = new Scene();
  oldScene.load(legacy);
  check("legacy (pre-v10) bodies load ungrounded",
    oldScene.bodies.every((body) => body.grounded === false), "all false");

  // Copy/paste carries the flag.
  const clip = scene.extractSelection([a.id, b.id])!;
  const pasted = scene.insertSelection(clip, { x: 300, y: 500 })!;
  const pastedBodies = pasted.bodyIds.map((id) => scene.getBody(id)!);
  check("paste keeps grounded flags per body",
    pastedBodies.filter((body) => body.grounded).length === 1 && pastedBodies.length === 2,
    `${pastedBodies.filter((body) => body.grounded).length} of ${pastedBodies.length} grounded`);
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
