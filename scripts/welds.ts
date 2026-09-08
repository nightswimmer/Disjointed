/**
 * Headless checks of welds (rigid pins, v18). A weld holds two bodies completely
 * together at a joint: coincident position (like a pin) AND the drawn relative angle
 * (no relative rotation). Covers: rigidity under dragging vs a plain pin, baseline
 * re-capture from the drawn pose, a free-joint weld staying inert, persistence +
 * legacy sanitization, copy/paste, break reporting when the weld is unreachable,
 * component expansion carrying the flag, and the analyzer's DOF accounting.
 */
import { Scene, PinConstraint } from "../src/model";
import { solve, resetPoseBaselines } from "../src/solver";
import { analyzeScene } from "../src/analyzer";

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

/** Two overlapping bars welded at (90, 0): A spans x 0..100, B spans x 80..180. */
function weldedPair(scene: Scene) {
  const bodyA = scene.addBody([
    { x: 0, y: -10 },
    { x: 100, y: -10 },
    { x: 100, y: 10 },
    { x: 0, y: 10 },
  ])!;
  const bodyB = scene.addBody([
    { x: 80, y: -10 },
    { x: 180, y: -10 },
    { x: 180, y: 10 },
    { x: 80, y: 10 },
  ])!;
  const weldA = scene.addJoint(bodyA.id, { x: 90, y: 0 });
  const weldB = scene.addJoint(bodyB.id, { x: 90, y: 0 });
  const pin = scene.addPin(weldA.id, weldB.id, true);
  return { bodyA, bodyB, weldA, weldB, pin };
}

// --- 1. A weld locks the relative angle; a plain pin doesn't ------------------
{
  resetPoseBaselines();
  const scene = new Scene();
  const { bodyA, bodyB, pin } = weldedPair(scene);
  const pivot = scene.addJoint(bodyA.id, { x: 10, y: 0 });
  scene.addGround(pivot.id, { x: 10, y: 0 });
  const handle = scene.addJoint(bodyB.id, { x: 170, y: 0 });

  // Swing the handle around the pivot; the welded pair must rotate as one piece.
  let worstRel = 0;
  let maxA = 0;
  for (let i = 0; i <= 16; i++) {
    const a = -Math.PI / 3 + (i / 16) * (2 * Math.PI / 3);
    solve(scene, { jointId: handle.id, target: { x: 10 + 160 * Math.cos(a), y: 160 * Math.sin(a) } }, 60);
    worstRel = Math.max(worstRel, Math.abs(angDiff(bodyB.angle, bodyA.angle)));
    maxA = Math.max(maxA, Math.abs(bodyA.angle));
  }
  check("weld: bodies keep their drawn relative angle under dragging", worstRel < 0.01,
    `max |relΔ| ${worstRel.toFixed(5)} rad`);
  check("weld: the welded pair still moves (rotates about the ground)", maxA > 0.5,
    `max |bodyA.angle| ${maxA.toFixed(3)} rad`);

  // Control: toggle the same pin back to revolute — the drag must now fold the pair.
  scene.setPinRigid(pin.id, false);
  resetPoseBaselines();
  let maxRel = 0;
  for (let i = 0; i <= 16; i++) {
    const a = -Math.PI / 3 + (i / 16) * (2 * Math.PI / 3);
    solve(scene, { jointId: handle.id, target: { x: 10 + 100 * Math.cos(a), y: 100 * Math.sin(a) } }, 60);
    maxRel = Math.max(maxRel, Math.abs(angDiff(bodyB.angle, bodyA.angle)));
  }
  check("revolute control: the same drag folds the pair at the pin", maxRel > 0.3,
    `max |relΔ| ${maxRel.toFixed(3)} rad`);
}

// --- 2. The baseline is the DRAWN pose (re-captured after edits) ---------------
{
  resetPoseBaselines();
  const scene = new Scene();
  const { bodyA, bodyB } = weldedPair(scene);
  const pivot = scene.addJoint(bodyA.id, { x: 10, y: 0 });
  scene.addGround(pivot.id, { x: 10, y: 0 });
  const handle = scene.addJoint(bodyB.id, { x: 170, y: 0 });

  // Re-draw: tilt B by 30° about the weld point, as a draw-mode edit would (which
  // also resets the baselines). The weld must now hold the NEW relative angle.
  const tilt = Math.PI / 6;
  scene.rotateBody(bodyB.id, { x: 90, y: 0 }, tilt);
  resetPoseBaselines();
  let worst = 0;
  for (let i = 0; i <= 12; i++) {
    const a = -Math.PI / 4 + (i / 12) * (Math.PI / 2);
    solve(scene, { jointId: handle.id, target: { x: 10 + 150 * Math.cos(a), y: 150 * Math.sin(a) } }, 60);
    worst = Math.max(worst, Math.abs(angDiff(bodyB.angle - tilt, bodyA.angle)));
  }
  check("weld: baseline re-captures the newly drawn relative angle", worst < 0.01,
    `max |relΔ − 30°| ${worst.toFixed(5)} rad`);
}

// --- 3. A weld to a free joint is inert (plain pin) until it gains a body -----
{
  resetPoseBaselines();
  const scene = new Scene();
  const body = scene.addBody([
    { x: 0, y: -10 },
    { x: 100, y: -10 },
    { x: 100, y: 10 },
    { x: 0, y: 10 },
  ])!;
  const bj = scene.addJoint(body.id, { x: 10, y: 0 });
  const free = scene.addFreeJoint({ x: 10, y: 0 });
  scene.addGround(free.id, { x: 10, y: 0 });
  scene.addPin(bj.id, free.id, true); // "weld" to an anchor point — no orientation to lock
  const handle = scene.addJoint(body.id, { x: 90, y: 0 });
  let maxA = 0;
  for (let i = 0; i <= 12; i++) {
    const a = (i / 12) * Math.PI - Math.PI / 2;
    const breaks = solve(scene, { jointId: handle.id, target: { x: 10 + 80 * Math.cos(a), y: 80 * Math.sin(a) } }, 60);
    maxA = Math.max(maxA, Math.abs(body.angle));
    if (breaks.length > 0 && i === 0) check("free-joint weld: no spurious breaks", false, `${breaks.length} breaks`);
  }
  check("free-joint weld: behaves as a plain pin (body still rotates)", maxA > 0.5,
    `max |angle| ${maxA.toFixed(3)} rad`);
}

// --- 4. Persistence: the rigid flag survives save/load; legacy pins load plain -
{
  const scene = new Scene();
  const { pin } = weldedPair(scene);
  const data = JSON.parse(JSON.stringify(scene.serialize()));
  const loaded = new Scene();
  loaded.load(data);
  const lp = loaded.constraints.find((c) => c.id === pin.id);
  check("persistence: rigid survives save/load", lp?.kind === "pin" && lp.rigid === true,
    `rigid=${lp?.kind === "pin" ? lp.rigid : "?"}`);

  // A pre-v18 file simply has no flag; a hand-edited non-boolean value is dropped.
  const legacy = JSON.parse(JSON.stringify(data)) as { constraints: { kind: string; rigid?: unknown }[] };
  for (const c of legacy.constraints) if (c.kind === "pin") c.rigid = "yes";
  const loaded2 = new Scene();
  loaded2.load(legacy as never);
  const lp2 = loaded2.constraints.find((c) => c.id === pin.id);
  check("persistence: a non-boolean rigid value is sanitized away",
    lp2?.kind === "pin" && lp2.rigid === undefined, `rigid=${lp2?.kind === "pin" ? String(lp2.rigid) : "?"}`);
}

// --- 5. Copy/paste carries the weld ------------------------------------------
{
  const scene = new Scene();
  const { bodyA, bodyB } = weldedPair(scene);
  const clip = scene.extractSelection([bodyA.id, bodyB.id])!;
  const before = scene.constraints.length;
  scene.insertSelection(clip, { x: 500, y: 500 });
  const newPins = scene.constraints.filter(
    (c): c is PinConstraint => c.kind === "pin" && scene.constraints.indexOf(c) >= before
  );
  check("copy/paste: the pasted pin is still a weld", newPins.length === 1 && newPins[0].rigid === true,
    `${newPins.length} new pins, rigid=${newPins[0]?.rigid}`);
}

// --- 6. An unreachable weld is reported as a break (the rest still solves) ----
{
  resetPoseBaselines();
  const scene = new Scene();
  const { bodyA, bodyB, weldA, weldB } = weldedPair(scene);
  scene.toggleBodyGround(bodyA.id);
  scene.toggleBodyGround(bodyB.id);
  // Capture the drawn baseline, then rotate B about the weld point WITHOUT re-capturing
  // (as if the assembly demanded an angle the fixed bodies can't reach).
  solve(scene, null, 30);
  scene.rotateBody(bodyB.id, { x: 90, y: 0 }, Math.PI / 4);
  const breaks = solve(scene, null, 60);
  const weldBreak = breaks.find((b) => b.joints.includes(weldA.id) && b.joints.includes(weldB.id));
  check("impossible weld: reported as a break naming both joints", weldBreak !== undefined,
    `${breaks.length} breaks`);
  check("impossible weld: fixed bodies never move", bodyA.angle === 0 && Math.abs(bodyB.angle - Math.PI / 4) < 1e-9,
    `angles ${bodyA.angle.toFixed(3)} / ${bodyB.angle.toFixed(3)}`);
}

// --- 7. Component expansion carries the weld; the definition is the reference -
{
  const scene = new Scene();
  const { bodyA, bodyB } = weldedPair(scene);
  const res = scene.createComponentFromSelection("welded pair", [bodyA.id, bodyB.id]);
  check("component: creation from a welded selection succeeds", res !== null, `res=${res !== null}`);
  if (res) {
    const instPin = scene.constraints.find((c): c is PinConstraint => c.kind === "pin");
    check("component: the expanded instance's pin is a weld", instPin?.rigid === true,
      `rigid=${instPin?.rigid}`);
    // Toggle the flag in the DEFINITION and re-expand: the instance must follow.
    const defPin = res.def.data.constraints.find((c) => c.kind === "pin");
    if (defPin && defPin.kind === "pin") delete defPin.rigid;
    scene.reexpandInstances(new Set([res.def.id]));
    const after = scene.constraints.find((c): c is PinConstraint => c.kind === "pin");
    check("component: un-welding the definition cascades to the instance", after !== undefined && after.rigid === undefined,
      `rigid=${String(after?.rigid)}`);
  }
}

// --- 8. Dragging a welded assembly can never pull it off its rail --------------
// Regression: a rod locked on a fixed track, welded to a second body. Dragging the
// second body (long lever, mostly-unreachable targets) used to exhaust the cleanup
// budget — the phantom-pair weld converged too slowly — so Phase B misreported the
// reachable rider/lock as breaks and the rod popped off the rail. The driver must
// always yield: worst case here is the assembly sliding along its one free axis.
{
  resetPoseBaselines();
  const scene = new Scene();
  const railA = scene.addFreeJoint({ x: 0, y: 0 });
  const railB = scene.addFreeJoint({ x: 200, y: 0 });
  const rail = scene.addSlider(railA.id, railB.id); // fixed track
  const rod = scene.addBody([
    { x: 60, y: -8 },
    { x: 140, y: -8 },
    { x: 140, y: 8 },
    { x: 60, y: 8 },
  ])!;
  const rider = scene.addJoint(rod.id, { x: 100, y: 0 });
  scene.attachSliderRider(rail.id, rider.id, true); // orientation-locked slider
  const other = scene.addBody([
    { x: 120, y: -8 },
    { x: 320, y: -8 },
    { x: 320, y: 8 },
    { x: 120, y: 8 },
  ])!;
  const jr = scene.addJoint(rod.id, { x: 130, y: 0 });
  const jo = scene.addJoint(other.id, { x: 130, y: 0 });
  scene.addPin(jr.id, jo.id, true);

  let breakFrames = 0;
  let worstOff = 0;
  let worstAngle = 0;
  const targets = [
    { x: 300, y: 300 }, // far perpendicular (unreachable)
    { x: 800, y: 0 },   // far past the end-stop
    { x: 800, y: 400 }, // both at once
    { x: -400, y: -300 }, // and back the other way
  ];
  for (const target of targets) {
    for (let f = 0; f < 10; f++) {
      const breaks = solve(scene, { bodyId: other.id, local: { x: 100, y: 0 }, target }, 100);
      breakFrames += breaks.length > 0 ? 1 : 0;
      worstOff = Math.max(worstOff, Math.abs(scene.jointWorld(rider).y));
      worstAngle = Math.max(worstAngle, Math.abs(angDiff(rod.angle, 0)));
    }
  }
  check("drag yield: unreachable drags on a welded rail assembly never break", breakFrames === 0,
    `${breakFrames} frames reported breaks`);
  check("drag yield: the rod never leaves the rail line", worstOff < 0.01, `max offset ${worstOff.toFixed(4)}`);
  check("drag yield: the rod never rotates off the rail lock", worstAngle < 0.01,
    `max |angle| ${worstAngle.toFixed(4)} rad`);
}

// --- 9. Analyzer: a weld removes 3 DOF (a pin only 2) --------------------------
{
  const scene = new Scene();
  const { bodyA, pin } = weldedPair(scene);
  const pivot = scene.addJoint(bodyA.id, { x: 10, y: 0 });
  scene.addGround(pivot.id, { x: 10, y: 0 });
  const welded = analyzeScene(scene).components[0].dofEstimate;
  scene.setPinRigid(pin.id, false);
  const pinned = analyzeScene(scene).components[0].dofEstimate;
  check("analyzer: weld removes one more DOF than a pin", welded === 1 && pinned === 2,
    `weld ${welded} DOF, pin ${pinned} DOF`);
}

if (failures > 0) {
  console.error(`\n${failures} check(s) FAILED`);
  process.exit(1);
}
console.log("\nAll weld checks passed.");
