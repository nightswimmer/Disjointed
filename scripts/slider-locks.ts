/**
 * Headless checks of orientation-locked riders (prismatic "sliders" on a rail, v17).
 * A locked rider's body must translate along the rail while keeping its drawn angle
 * relative to the rail; an unlocked rider stays a pin-in-slot (rotates freely).
 * Also covers: baseline re-capture, a moving rail carrying the lock, persistence,
 * cascade removal, copy/paste, and break reporting when the lock is unreachable.
 */
import { Scene } from "../src/model";
import { solve, resetPoseBaselines } from "../src/solver";
import { dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

/** Smallest signed difference between two angles (radians). */
function angDiff(a: number, b: number): number {
  let d = a - b;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

// --- 1. Locked rider on a world-fixed track: slides, never rotates ------------
{
  resetPoseBaselines();
  const scene = new Scene();
  const railA = scene.addFreeJoint({ x: 0, y: 0 });
  const railB = scene.addFreeJoint({ x: 200, y: 0 });
  const rail = scene.addSlider(railA.id, railB.id); // auto-grounds the track

  // A wide body with a rider joint at its centre and a handle joint off to the side.
  const body = scene.addBody([
    { x: 60, y: -10 },
    { x: 140, y: -10 },
    { x: 140, y: 10 },
    { x: 60, y: 10 },
  ])!;
  const rider = scene.addJoint(body.id, { x: 100, y: 0 });
  const handle = scene.addJoint(body.id, { x: 140, y: 0 });
  scene.attachSliderRider(rail.id, rider.id, true);
  const angle0 = body.angle;

  // Drag the handle around; the body must slide along the rail without rotating.
  let worstAngle = 0;
  let worstOff = 0;
  for (let i = 0; i < 24; i++) {
    const a = (i / 24) * Math.PI * 2;
    solve(scene, { jointId: handle.id, target: { x: 100 + 90 * Math.cos(a), y: 70 * Math.sin(a) } }, 60);
    worstAngle = Math.max(worstAngle, Math.abs(angDiff(body.angle, angle0)));
    worstOff = Math.max(worstOff, Math.abs(scene.jointWorld(rider).y));
  }
  check("locked rider: body never rotates on a fixed track", worstAngle < 0.01,
    `max |Δangle| ${worstAngle.toFixed(5)} rad`);
  check("locked rider: stays on the rail line", worstOff < 0.5, `max offset ${worstOff.toFixed(4)}`);

  // Control: unlock it — the same drag must now rotate the body (pin-in-slot).
  scene.setSliderRiderLocked(rail.id, rider.id, false);
  resetPoseBaselines();
  let maxAngle = 0;
  for (let i = 0; i < 24; i++) {
    const a = (i / 24) * Math.PI * 2;
    solve(scene, { jointId: handle.id, target: { x: 100 + 90 * Math.cos(a), y: 70 * Math.sin(a) } }, 60);
    maxAngle = Math.max(maxAngle, Math.abs(angDiff(body.angle, angle0)));
  }
  check("unlocked rider: same drag rotates the body (pin-in-slot)", maxAngle > 0.3,
    `max |Δangle| ${maxAngle.toFixed(3)} rad`);
}

// --- 2. Baseline captures the DRAWN relative angle (including a tilted one) ---
{
  resetPoseBaselines();
  const scene = new Scene();
  const railA = scene.addFreeJoint({ x: 0, y: 0 });
  const railB = scene.addFreeJoint({ x: 200, y: 0 });
  const rail = scene.addSlider(railA.id, railB.id);
  const body = scene.addBody([
    { x: 80, y: -10 },
    { x: 120, y: -10 },
    { x: 120, y: 10 },
    { x: 80, y: 10 },
  ])!;
  const rider = scene.addJoint(body.id, { x: 100, y: 0 });
  const handle = scene.addJoint(body.id, { x: 120, y: 0 });
  scene.attachSliderRider(rail.id, rider.id, true);
  body.angle = 0.5; // drawn tilted relative to the rail
  const drawn = body.angle;
  for (let i = 0; i < 10; i++) {
    solve(scene, { jointId: handle.id, target: { x: 40 + 10 * i, y: 30 } }, 60);
  }
  check("baseline: a tilted drawn angle is what gets locked",
    Math.abs(angDiff(body.angle, drawn)) < 0.01,
    `angle ${body.angle.toFixed(4)} vs drawn ${drawn.toFixed(4)}`);
}

// --- 3. Moving rail: rider body keeps its angle RELATIVE to the rail ----------
{
  resetPoseBaselines();
  const scene = new Scene();
  // Rail body, grounded at one end so it can pivot about it.
  const railBody = scene.addBody([
    { x: 0, y: -8 },
    { x: 200, y: -8 },
    { x: 200, y: 8 },
    { x: 0, y: 8 },
  ])!;
  const pivotJ = scene.addJoint(railBody.id, { x: 0, y: 0 });
  const railA = scene.addJoint(railBody.id, { x: 10, y: 0 });
  const railB = scene.addJoint(railBody.id, { x: 190, y: 0 });
  scene.addGround(pivotJ.id, { x: 0, y: 0 });
  const rail = scene.addSlider(railA.id, railB.id);
  // Rider body on the rail, locked.
  const slab = scene.addBody([
    { x: 80, y: 10 },
    { x: 120, y: 10 },
    { x: 120, y: 30 },
    { x: 80, y: 30 },
  ])!;
  const rider = scene.addJoint(slab.id, { x: 100, y: 20 });
  scene.attachSliderRider(rail.id, rider.id, true);
  const rel0 = slab.angle - railBody.angle;
  // Swing the rail by dragging its far end; the slab must swing with it.
  let worstRel = 0;
  for (let i = 1; i <= 16; i++) {
    const a = (i / 16) * 0.8;
    solve(scene, { jointId: railB.id, target: { x: 190 * Math.cos(a), y: 190 * Math.sin(a) } }, 80);
    worstRel = Math.max(worstRel, Math.abs(angDiff(slab.angle - railBody.angle, rel0)));
  }
  check("moving rail: locked body keeps its angle relative to the rail", worstRel < 0.02,
    `max |Δrel| ${worstRel.toFixed(5)} rad, rail tilted ${railBody.angle.toFixed(2)} rad`);
  check("moving rail: the rail actually moved", Math.abs(railBody.angle) > 0.5,
    `rail angle ${railBody.angle.toFixed(3)} rad`);
}

// --- 4. Persistence: locked survives save/load; legacy files load clean -------
{
  const scene = new Scene();
  const railA = scene.addFreeJoint({ x: 0, y: 0 });
  const railB = scene.addFreeJoint({ x: 100, y: 0 });
  const rail = scene.addSlider(railA.id, railB.id);
  const rider = scene.addFreeJoint({ x: 50, y: 0 });
  scene.attachSliderRider(rail.id, rider.id, true);

  const data = JSON.parse(JSON.stringify(scene.serialize()));
  const loaded = new Scene();
  loaded.load(data);
  const lc = loaded.constraints.find((c) => c.kind === "slider");
  check("save/load keeps the lock", !!lc && lc.kind === "slider" && lc.locked.includes(rider.id),
    `locked ${lc && lc.kind === "slider" ? JSON.stringify(lc.locked) : "?"}`);

  // Legacy (pre-v17) slider without `locked` — and a hand-edited lock on a non-rider.
  const legacy = JSON.parse(JSON.stringify(data));
  for (const c of legacy.constraints) {
    if (c.kind === "slider") delete c.locked;
  }
  const loaded2 = new Scene();
  loaded2.load(legacy);
  const lc2 = loaded2.constraints.find((c) => c.kind === "slider");
  check("legacy slider loads with no locks", !!lc2 && lc2.kind === "slider" && lc2.locked.length === 0,
    `locked ${lc2 && lc2.kind === "slider" ? JSON.stringify(lc2.locked) : "?"}`);
}

// --- 5. Cascades: deleting the rider joint drops its lock ----------------------
{
  const scene = new Scene();
  const railA = scene.addFreeJoint({ x: 0, y: 0 });
  const railB = scene.addFreeJoint({ x: 100, y: 0 });
  const rail = scene.addSlider(railA.id, railB.id);
  const rider = scene.addFreeJoint({ x: 50, y: 0 });
  scene.attachSliderRider(rail.id, rider.id, true);
  scene.removeJoint(rider.id);
  const c = scene.constraints.find((x) => x.kind === "slider");
  check("removing the rider joint sheds rider + lock",
    !!c && c.kind === "slider" && c.riders.length === 0 && c.locked.length === 0,
    `riders ${c && c.kind === "slider" ? c.riders.length : "?"}, locked ${c && c.kind === "slider" ? c.locked.length : "?"}`);
}

// --- 6. Copy/paste: the lock travels with an internal slider -------------------
{
  const scene = new Scene();
  const body = scene.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 30 },
    { x: 0, y: 30 },
  ])!;
  const railA = scene.addJoint(body.id, { x: 10, y: 15 });
  const railB = scene.addJoint(body.id, { x: 90, y: 15 });
  const rail = scene.addSlider(railA.id, railB.id);
  const slab = scene.addBody([
    { x: 40, y: 0 },
    { x: 60, y: 0 },
    { x: 60, y: 30 },
    { x: 40, y: 30 },
  ])!;
  const rider = scene.addJoint(slab.id, { x: 50, y: 15 });
  scene.attachSliderRider(rail.id, rider.id, true);

  const clip = scene.extractSelection([body.id, slab.id])!;
  const before = scene.constraints.filter((c) => c.kind === "slider").length;
  scene.insertSelection(clip, { x: 300, y: 300 });
  const sliders = scene.constraints.filter((c) => c.kind === "slider");
  const pasted = sliders.find((c) => c.id !== rail.id);
  check("paste recreates the slider with its lock",
    sliders.length === before + 1 &&
      !!pasted && pasted.kind === "slider" && pasted.riders.length === 1 &&
      pasted.locked.length === 1 && pasted.locked[0] === pasted.riders[0],
    `pasted riders ${pasted && pasted.kind === "slider" ? pasted.riders.length : "?"}, locked ${pasted && pasted.kind === "slider" ? pasted.locked.length : "?"}`);
}

// --- 7. Unreachable lock/rider reports a break, grounds hold -------------------
{
  resetPoseBaselines();
  const scene = new Scene();
  // Fixed track along y = 40.
  const railA = scene.addFreeJoint({ x: 0, y: 40 });
  const railB = scene.addFreeJoint({ x: 200, y: 40 });
  const rail = scene.addSlider(railA.id, railB.id);
  // Body grounded at one joint; its rider joint can only reach the rail by rotating,
  // which the lock forbids — the assembly is impossible.
  const body = scene.addBody([
    { x: 0, y: -10 },
    { x: 100, y: -10 },
    { x: 100, y: 10 },
    { x: 0, y: 10 },
  ])!;
  const anchor = scene.addJoint(body.id, { x: 0, y: 0 });
  const rider = scene.addJoint(body.id, { x: 100, y: 0 });
  scene.addGround(anchor.id, { x: 0, y: 0 });
  scene.attachSliderRider(rail.id, rider.id, true);
  const breaks = solve(scene, null, 100);
  const anchorHeld = dist(scene.jointWorld(anchor), { x: 0, y: 0 }) < 0.5;
  check("impossible lock+ground reports a break", breaks.length > 0, `${breaks.length} break(s)`);
  check("ground stays sacred while the lock breaks", anchorHeld,
    `anchor drift ${dist(scene.jointWorld(anchor), { x: 0, y: 0 }).toFixed(4)}`);
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
