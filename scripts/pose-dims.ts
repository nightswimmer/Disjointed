/**
 * Headless coverage for pose-level driving dimensions (pose.ts): dimensions whose both
 * ends live on component-instance geometry drive the *pose* of rigid parts (instance
 * shape stays locked to the definition). Verifies that:
 *  - a dimension between two different instances drives by rigidly translating one of
 *    them (the second-picked side by preference); shapes and the untouched instance stay
 *    exactly put, and the driving flag + target are set;
 *  - a grounded instance never moves (the other side moves instead); both grounded
 *    rejects with the scene untouched;
 *  - h/v-axis dimensions translate along their axis only;
 *  - conflicting pose dimensions reject (reject semantics: scene + dimension untouched);
 *  - enforcePoseDims follows live: with the dragged instance anchored, the partner
 *    translates to hold the dimension;
 *  - a dimension between two MOBILE parts of one instance re-poses the internal
 *    mechanism via the sim solver (pin intact, shapes + definition untouched);
 *  - a dimension between two ends rigid to each other (same instance body) rejects;
 *  - pose dimensions never enter the sketch (shape) solver;
 *  - a mixed dimension (instance ↔ free geometry) still routes to the sketch and drives
 *    by moving the free side.
 */
import { Scene } from "../src/model";
import { applyDimensionValue, enforcePose, isPoseDim } from "../src/pose";
import { solveSketch, sketchConfig, anchorVarsForJoint } from "../src/sketch";
import { dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

function square(scene: Scene, cx: number, cy: number, half = 20) {
  return scene.addBody([
    { x: cx - half, y: cy - half },
    { x: cx + half, y: cy - half },
    { x: cx + half, y: cy + half },
    { x: cx - half, y: cy + half },
  ]);
}

const TOL = sketchConfig.tol * 2;

// --- inter-instance dimensions: rigid translation --------------------------------
{
  const scene = new Scene();
  const b1 = square(scene, 0, 0);
  scene.toggleBodyGround(b1.id); // becomes the def's chassis
  const res = scene.createComponentFromSelection("Block", [b1.id])!;
  const inst1 = res.instance;
  const inst2 = scene.instantiateComponent(res.def.id, { pos: { x: 200, y: 0 }, angle: 0 })!;
  const body1 = scene.getBody(inst1.bodyMap[0].id)!;
  const body2 = scene.getBody(inst2.bodyMap[0].id)!;

  const m = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body1.id, index: 0 },
    { kind: "vertex", bodyId: body2.id, index: 0 },
    { x: 100, y: -60 }
  )!;
  m.axis = "direct";
  check("dimension is pose-level", isPoseDim(scene, m), "both ends instance-owned");

  const shape1 = JSON.stringify(body1.controlLocal);
  const ok = applyDimensionValue(scene, m.id, 150);
  const val = scene.measureInfo(m)!.value;
  check("inter-instance dim drives", ok.length === 0 && m.driving === true, `${ok.length} breaks`);
  check("dim hits its target", Math.abs(val - 150) < TOL, `value ${val.toFixed(4)}`);
  check("refA's instance stayed put", dist(body1.pos, { x: 0, y: 0 }) < 1e-9, `at (${body1.pos.x}, ${body1.pos.y})`);
  check("refB's instance translated", Math.abs(body2.pos.x - 150) < TOL && Math.abs(body2.pos.y) < TOL, `at (${body2.pos.x.toFixed(3)}, ${body2.pos.y.toFixed(3)})`);
  check("no reshape", JSON.stringify(body1.controlLocal) === shape1 && JSON.stringify(body2.controlLocal) === shape1, "shapes identical");

  // Grounding the refB side makes the other side move instead.
  scene.toggleBodyGround(body2.id);
  const p2 = { ...body2.pos };
  const ok2 = applyDimensionValue(scene, m.id, 100);
  check("grounded partner never moves", ok2.length === 0 && dist(body2.pos, p2) < 1e-9, `${ok2.length} breaks, at (${body2.pos.x.toFixed(1)}, ${body2.pos.y.toFixed(1)})`);
  const val2 = scene.measureInfo(m)!.value;
  check("dim re-hit via the free side", Math.abs(val2 - 100) < TOL, `value ${val2.toFixed(4)}`);

  // Both sides grounded: the edit rejects and nothing moves.
  scene.toggleBodyGround(body1.id);
  const p1 = { ...body1.pos };
  const rej = applyDimensionValue(scene, m.id, 60);
  check("both grounded rejects", rej.length > 0, `${rej.length} breaks`);
  check("rejected edit leaves the scene untouched", dist(scene.getBody(body1.id)!.pos, p1) < 1e-9 && dist(scene.getBody(body2.id)!.pos, p2) < 1e-9, "held");
  check("rejected edit keeps the old target", scene.getMeasurement(m.id)!.target === 100, `target ${scene.getMeasurement(m.id)!.target}`);
}

// --- h-axis pose dimension + live follow + conflicts ------------------------------
{
  const scene = new Scene();
  const b1 = square(scene, 0, 0);
  scene.toggleBodyGround(b1.id);
  const res = scene.createComponentFromSelection("Block", [b1.id])!;
  const inst1 = res.instance;
  const inst2 = scene.instantiateComponent(res.def.id, { pos: { x: 200, y: 80 }, angle: 0 })!;
  const body1 = scene.getBody(inst1.bodyMap[0].id)!;
  const body2 = scene.getBody(inst2.bodyMap[0].id)!;

  const mh = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body1.id, index: 0 },
    { kind: "vertex", bodyId: body2.id, index: 0 },
    { x: 100, y: 0 }
  )!;
  mh.axis = "h";
  const ok = applyDimensionValue(scene, mh.id, 120);
  check("h-axis pose dim drives", ok.length === 0, `${ok.length} breaks`);
  check("h dim translates along x only", Math.abs(body2.pos.x - 120) < TOL && Math.abs(body2.pos.y - 80) < 1e-9, `at (${body2.pos.x.toFixed(3)}, ${body2.pos.y})`);

  // Live follow: drag instance 1 (anchored) — instance 2 translates to hold the dim.
  scene.moveInstance(inst1.id, { x: 30, y: -10 });
  const left = enforcePose(scene, new Set([inst1.id]));
  const val = scene.measureInfo(mh)!.value;
  check("partner follows a drag", left.length === 0 && Math.abs(val - 120) < TOL, `value ${val.toFixed(4)}`);
  check("dragged instance stays where the user put it", dist(body1.pos, { x: 30, y: -10 }) < 1e-9, `at (${body1.pos.x}, ${body1.pos.y})`);

  // A second dim between the same rigid pair at an incompatible value conflicts.
  const m2 = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body1.id, index: 1 },
    { kind: "vertex", bodyId: body2.id, index: 1 },
    { x: 100, y: 40 }
  )!;
  m2.axis = "h";
  const p2 = { ...body2.pos };
  const rej = applyDimensionValue(scene, m2.id, 40); // dim1 pins |dx| at 120 — 40 is impossible
  check("conflicting pose dim rejects", rej.length > 0, `${rej.length} breaks`);
  check("conflict leaves the scene untouched", dist(scene.getBody(body2.id)!.pos, p2) < 1e-9 && !scene.getMeasurement(m2.id)!.driving, "held, not driving");

  // Pose dims never enter the shape solver: a plain sketch solve moves nothing.
  const snap = JSON.stringify(scene.serialize());
  const sk = solveSketch(scene);
  check("pose dims are excluded from the sketch", sk.length === 0 && JSON.stringify(scene.serialize()) === snap, "sketch no-op");
}

// --- intra-instance dimension: re-pose the internal mechanism ---------------------
{
  const scene = new Scene();
  const base = square(scene, 0, 0, 30);
  scene.toggleBodyGround(base.id); // chassis
  const arm = square(scene, 60, 0, 30); // shares the edge x = 30 with base
  const jb = scene.addJoint(base.id, { x: 30, y: 0 });
  const ja = scene.addJoint(arm.id, { x: 30, y: 0 });
  scene.addPin(jb.id, ja.id);
  const res = scene.createComponentFromSelection("Linkage", [base.id, arm.id])!;
  const inst = res.instance;
  const nBase = scene.getBody(inst.bodyMap.find((e) => e.chassis)!.id)!;
  const nArm = scene.getBody(inst.bodyMap.find((e) => !e.chassis)!.id)!;
  const defBefore = JSON.stringify(res.def.data);
  const armShape = JSON.stringify(nArm.controlLocal);

  // Base far corner (-30,-30) to arm far corner (90,-30): 120 as drawn; the arm can
  // pivot about the pin at (30, 0), so 100 is reachable.
  const m = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: nBase.id, index: 0 },
    { kind: "vertex", bodyId: nArm.id, index: 1 },
    { x: 30, y: -80 }
  )!;
  m.axis = "direct";
  check("intra dim is pose-level", isPoseDim(scene, m), "both ends instance-owned");
  const ok = applyDimensionValue(scene, m.id, 100);
  const val = scene.measureInfo(m)!.value;
  check("intra-instance dim drives", ok.length === 0 && m.driving === true, `${ok.length} breaks`);
  check("intra dim hits its target", Math.abs(val - 100) < TOL, `value ${val.toFixed(4)}`);
  check("held side never moves", dist(nBase.pos, { x: 0, y: 0 }) < 1e-9, `base at (${nBase.pos.x}, ${nBase.pos.y})`);
  check("the arm re-posed", dist(nArm.pos, { x: 60, y: 0 }) > 1, `arm at (${nArm.pos.x.toFixed(2)}, ${nArm.pos.y.toFixed(2)})`);
  const pin = scene.constraints.find((c) => c.kind === "pin")!;
  const gap =
    pin.kind === "pin"
      ? dist(scene.jointWorld(scene.getJoint(pin.jointA)!), scene.jointWorld(scene.getJoint(pin.jointB)!))
      : Infinity;
  check("the pin stayed closed", gap < TOL, `gap ${gap.toExponential(2)}`);
  check("no reshape from the re-pose", JSON.stringify(nArm.controlLocal) === armShape, "shape identical");
  check("the definition is untouched", JSON.stringify(res.def.data) === defBefore, "def identical");

  // Two ends rigid to each other (same instance body) can never drive.
  const mRigid = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: nArm.id, index: 0 },
    { kind: "vertex", bodyId: nArm.id, index: 1 },
    { x: 60, y: 60 }
  )!;
  mRigid.axis = "direct";
  const rej = applyDimensionValue(scene, mRigid.id, 80);
  check("dim rigid at a deeper level rejects", rej.length > 0 && !scene.getMeasurement(mRigid.id)!.driving, `${rej.length} breaks`);
}

// --- the held side: a fast drag can never flip the pair through each other --------
{
  const scene = new Scene();
  const b1 = square(scene, 0, 0);
  scene.toggleBodyGround(b1.id);
  const res = scene.createComponentFromSelection("Block", [b1.id])!;
  const inst1 = res.instance;
  const inst2 = scene.instantiateComponent(res.def.id, { pos: { x: 0, y: 80 }, angle: 0 })!;
  const body1 = scene.getBody(inst1.bodyMap[0].id)!;
  const body2 = scene.getBody(inst2.bodyMap[0].id)!;

  const m = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body1.id, index: 0 },
    { kind: "vertex", bodyId: body2.id, index: 0 },
    { x: -60, y: 40 }
  )!;
  m.axis = "v";
  const ok = applyDimensionValue(scene, m.id, 30);
  check("v dim drives and captures its side", ok.length === 0 && scene.getMeasurement(m.id)!.side === 1, `side ${scene.getMeasurement(m.id)!.side}`);
  check("partner sits 30 above", Math.abs(body2.pos.y - 30) < TOL, `y ${body2.pos.y.toFixed(3)}`);

  // A violent one-frame drag jumps instance 1 far PAST instance 2. Re-deriving the
  // sign from the overshot geometry would settle the partner on the flipped side
  // (y = 170); the held side must push it back above the dragged instance instead.
  scene.moveInstance(inst1.id, { x: 0, y: 200 });
  const left = enforcePose(scene, new Set([inst1.id]));
  check("overshoot drag keeps the relative direction", left.length === 0 && Math.abs(body2.pos.y - 230) < TOL, `y ${body2.pos.y.toFixed(3)}`);

  // Same protection in the sketch (shape) solver: a driving v dim between two free
  // joints, the first dragged far past the second in one frame.
  const ja = scene.addFreeJoint({ x: 300, y: 0 });
  const jb = scene.addFreeJoint({ x: 300, y: 40 });
  const ms = scene.addMeasurement(
    "draw",
    { kind: "joint", jointId: ja.id },
    { kind: "joint", jointId: jb.id },
    { x: 260, y: 20 }
  )!;
  ms.axis = "v";
  const ok2 = applyDimensionValue(scene, ms.id, 25);
  check("sketch v dim drives with a held side", ok2.length === 0 && scene.getMeasurement(ms.id)!.side === 1, `side ${scene.getMeasurement(ms.id)!.side}`);
  scene.moveJoint(ja.id, { x: 0, y: 300 }); // one-frame overshoot far past jb
  const sk = solveSketch(scene, new Set(anchorVarsForJoint(scene, ja.id)));
  const ya = scene.getJoint(ja.id)!.local.y;
  const yb = scene.getJoint(jb.id)!.local.y;
  check("sketch dim keeps the relative direction too", sk.length === 0 && Math.abs(yb - (ya + 25)) < TOL, `jb.y ${yb.toFixed(3)} vs ja.y ${ya.toFixed(3)} (held above)`);

  // Clearing the value back to driven drops the held side.
  scene.clearMeasurementDriving(ms.id);
  check("cleared dim drops its side", scene.getMeasurement(ms.id)!.side === undefined, "side gone");
}

// --- mixed dimension still routes to the sketch -----------------------------------
{
  const scene = new Scene();
  const b1 = square(scene, 0, 0);
  scene.toggleBodyGround(b1.id);
  const res = scene.createComponentFromSelection("Block", [b1.id])!;
  const instBody = scene.getBody(res.instance.bodyMap[0].id)!;
  const fj = scene.addFreeJoint({ x: 100, y: 0 });
  const m = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: instBody.id, index: 1 },
    { kind: "joint", jointId: fj.id },
    { x: 60, y: -60 }
  )!;
  check("mixed dim is not pose-level", !isPoseDim(scene, m), "one free end");
  const ok = applyDimensionValue(scene, m.id, 150);
  const val = scene.measureInfo(m)!.value;
  check("mixed dim still drives through the sketch", ok.length === 0 && Math.abs(val - 150) < TOL, `value ${val.toFixed(4)}`);
  check("mixed dim moved the free side only", dist(instBody.pos, { x: 0, y: 0 }) < 1e-9, "instance held");
}

if (failures > 0) {
  console.error(`${failures} FAILURE(S)`);
  process.exit(1);
}
console.log("All pose-dimension checks passed");
