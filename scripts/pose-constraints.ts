/**
 * Headless coverage for sketch constraints on component instances (pose.ts). Instance
 * shape stays locked to the definition; constraints touching instances either move the
 * free side (one free end — the sketch solver) or the *pose* of rigid parts (every end
 * on instance geometry — pose constraints). Verifies that:
 *  - MIXED constraints (instance ↔ free geometry) route to the sketch and are satisfied
 *    by the free side only: point-on-line, parallel, equal; the free side keeps
 *    following when the instance moves;
 *  - TRANSLATIONAL pose constraints between two instances (coincident point–point,
 *    point-on-line, H/V point pairs) translate one instance rigidly (shapes untouched,
 *    the other instance put); a grounded partner never moves; both grounded rejects
 *    with the scene untouched; conflicting constraints reject; live follow with the
 *    dragged instance anchored; pose constraints never enter the shape solver;
 *  - ROTATIONAL pose constraints (single-line H/V, parallel, perpendicular) rotate an
 *    instance about the constrained line's midpoint; rotation and translation items
 *    settle together; rotateInstance keeps the placement coherent (free chassis points
 *    orbit with the bodies); a constraint the drag can't hold reads as violated until
 *    the un-anchored settle re-asserts it;
 *  - INTRA-instance constraints between two mobile parts of one instance re-pose the
 *    internal mechanism (pin intact, shapes + definition untouched), both a
 *    translational (H point pair) and a rotational (perpendicular) one; a pair rigid to
 *    each other rejects; "equal" between two instance edges rejects.
 */
import { Scene } from "../src/model";
import { placeConstraint, enforcePose, isPoseConstraint, poseConstraintViolated } from "../src/pose";
import { solveSketch, sketchConfig } from "../src/sketch";
import { Vec2, dist, sub } from "../src/geometry";

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
const fmt = (p: Vec2) => `(${p.x.toFixed(3)}, ${p.y.toFixed(3)})`;
const edgeAngle = (scene: Scene, bodyId: number, index: number): number => {
  const r = scene.resolveMeasureRef({ kind: "edge", bodyId, index })!;
  if (r.kind !== "line") return NaN;
  return Math.atan2(r.b.y - r.a.y, r.b.x - r.a.x);
};
const wrapHalfPi = (a: number): number => {
  let d = ((a % Math.PI) + Math.PI) % Math.PI;
  if (d > Math.PI / 2) d -= Math.PI;
  return d;
};

/** Two instances of a 40×40 block component (chassis = the block). */
function twoBlocks(pos2: Vec2, angle2 = 0) {
  const scene = new Scene();
  const b1 = square(scene, 0, 0);
  scene.toggleBodyGround(b1.id);
  const res = scene.createComponentFromSelection("Block", [b1.id])!;
  const inst1 = res.instance;
  const inst2 = scene.instantiateComponent(res.def.id, { pos: pos2, angle: angle2 })!;
  const body1 = scene.getBody(inst1.bodyMap[0].id)!;
  const body2 = scene.getBody(inst2.bodyMap[0].id)!;
  return { scene, def: res.def, inst1, inst2, body1, body2 };
}

// --- tier 1: mixed constraints route to the sketch, the free side moves -------------
{
  const { scene, body1 } = twoBlocks({ x: 500, y: 500 });
  const shape = JSON.stringify(body1.controlLocal);
  const fj = scene.addFreeJoint({ x: 60, y: -35 });
  // Point on line: the free joint drops onto the instance's top edge (y = -20).
  const r1 = placeConstraint(scene, "coincident", { kind: "joint", jointId: fj.id }, { kind: "edge", bodyId: body1.id, index: 0 });
  check("mixed point-on-line accepted", r1.constraint !== null && r1.breaks.length === 0, `${r1.breaks.length} breaks`);
  check("mixed constraint is not pose-level", r1.constraint !== null && !isPoseConstraint(scene, r1.constraint), "one free end");
  const jw = scene.jointWorld(scene.getJoint(fj.id)!);
  check("free joint moved onto the edge", Math.abs(jw.y + 20) < TOL && Math.abs(jw.x - 60) < TOL, fmt(jw));
  check("instance untouched by the mixed constraint", dist(body1.pos, { x: 0, y: 0 }) < 1e-9 && body1.angle === 0 && JSON.stringify(body1.controlLocal) === shape, "held");

  // The free side keeps following the instance.
  scene.moveInstance(scene.instances[0].id, { x: 0, y: 30 });
  const sk = solveSketch(scene);
  const jw2 = scene.jointWorld(scene.getJoint(fj.id)!);
  check("free side follows the moved instance", sk.length === 0 && Math.abs(jw2.y - 10) < TOL, fmt(jw2));

  // Parallel: a free tilted body aligns to the instance edge; equal: it rescales.
  const free = scene.addBody([
    { x: 100, y: 100 },
    { x: 160, y: 120 },
    { x: 150, y: 170 },
  ]);
  const r2 = placeConstraint(scene, "parallel", { kind: "edge", bodyId: body1.id, index: 0 }, { kind: "edge", bodyId: free.id, index: 0 });
  const ang = wrapHalfPi(edgeAngle(scene, free.id, 0) - edgeAngle(scene, body1.id, 0));
  check("mixed parallel accepted, free edge aligned", r2.constraint !== null && Math.abs(ang) < 1e-4, `mismatch ${ang.toExponential(2)}`);
  const r3 = placeConstraint(scene, "equal", { kind: "edge", bodyId: body1.id, index: 1 }, { kind: "edge", bodyId: free.id, index: 0 });
  const e = scene.resolveMeasureRef({ kind: "edge", bodyId: free.id, index: 0 })!;
  const l = e.kind === "line" ? dist(e.a, e.b) : NaN;
  check("mixed equal accepted, free edge rescaled to 40", r3.constraint !== null && Math.abs(l - 40) < TOL, `len ${l.toFixed(4)}`);
  check("instance shape still identical", JSON.stringify(body1.controlLocal) === shape && dist(body1.pos, { x: 0, y: 30 }) < 1e-9, "held");
}

// --- tier 2: translational pose constraints between two instances -------------------
{
  const { scene, inst1, inst2, body1, body2 } = twoBlocks({ x: 200, y: 70 });
  const shape = JSON.stringify(body1.controlLocal);
  // Coincident vertex 1 of block 1 (20,-20) with vertex 0 of block 2 (180, 50).
  const r = placeConstraint(scene, "coincident", { kind: "vertex", bodyId: body1.id, index: 1 }, { kind: "vertex", bodyId: body2.id, index: 0 });
  check("inter-instance coincident accepted", r.constraint !== null && r.breaks.length === 0, `${r.breaks.length} breaks`);
  check("constraint is pose-level", r.constraint !== null && isPoseConstraint(scene, r.constraint), "both ends instance-owned");
  check("refA's instance stayed put", dist(body1.pos, { x: 0, y: 0 }) < 1e-9, fmt(body1.pos));
  check("refB's instance translated onto it", dist(body2.pos, { x: 40, y: 0 }) < TOL, fmt(body2.pos));
  check("no reshape", JSON.stringify(body1.controlLocal) === shape && JSON.stringify(body2.controlLocal) === shape, "shapes identical");
  check("not violated once placed", r.constraint !== null && !poseConstraintViolated(scene, r.constraint), "holds");

  // Pose constraints never enter the shape solver.
  const snap = JSON.stringify(scene.serialize());
  const sk = solveSketch(scene);
  check("pose constraint excluded from the sketch solver", sk.length === 0 && JSON.stringify(scene.serialize()) === snap, "sketch no-op");

  // Live follow: drag instance 1 (anchored) — instance 2 comes along.
  scene.moveInstance(inst1.id, { x: -25, y: 15 });
  const left = enforcePose(scene, new Set([inst1.id]));
  check("partner follows a drag", left.length === 0 && dist(body2.pos, { x: 15, y: 15 }) < TOL, fmt(body2.pos));
  check("dragged instance stays where the user put it", dist(body1.pos, { x: -25, y: 15 }) < 1e-9, fmt(body1.pos));

  // A second, incompatible constraint between the same rigid pair rejects untouched.
  const before = JSON.stringify(scene.serialize());
  const n = scene.sketch.length;
  const rej = placeConstraint(scene, "coincident", { kind: "vertex", bodyId: body1.id, index: 0 }, { kind: "vertex", bodyId: body2.id, index: 2 });
  check("conflicting pose constraint rejects", rej.constraint === null && rej.breaks.length > 0, `${rej.breaks.length} breaks`);
  check("rejected edit leaves the scene untouched", JSON.stringify(scene.serialize()) === before && scene.sketch.length === n, "identical");

  // Grounding the refB side makes the other side move instead; both grounded rejects.
  scene.removeSketchConstraint(r.constraint!.id);
  scene.toggleBodyGround(body2.id);
  const p2 = { ...body2.pos };
  const rh = placeConstraint(scene, "horizontal", { kind: "vertex", bodyId: body1.id, index: 0 }, { kind: "vertex", bodyId: body2.id, index: 3 });
  check("H point pair drives via the free side", rh.constraint !== null && dist(body2.pos, p2) < 1e-9, `${rh.breaks.length} breaks, b2 ${fmt(body2.pos)}`);
  const a0 = scene.resolveMeasureRef({ kind: "vertex", bodyId: body1.id, index: 0 })!;
  const b3 = scene.resolveMeasureRef({ kind: "vertex", bodyId: body2.id, index: 3 })!;
  check("H levels the pair along y only", a0.kind === "point" && b3.kind === "point" && Math.abs(a0.p.y - b3.p.y) < TOL && Math.abs(body1.pos.x + 25) < 1e-9, `y ${a0.kind === "point" ? a0.p.y.toFixed(3) : "?"} vs ${b3.kind === "point" ? b3.p.y.toFixed(3) : "?"}`);
  scene.toggleBodyGround(body1.id);
  const beforeG = JSON.stringify(scene.serialize());
  const rg = placeConstraint(scene, "coincident", { kind: "vertex", bodyId: body1.id, index: 2 }, { kind: "edge", bodyId: body2.id, index: 0 });
  check("both grounded rejects", rg.constraint === null && rg.breaks.length > 0 && JSON.stringify(scene.serialize()) === beforeG, `${rg.breaks.length} breaks`);
  check("a violated placement is reported by enforcePose", enforcePose(scene).length === 0, "existing H still holds");
  void inst2;
}

// --- tier 2b: point-on-line translates along the normal only; second pick moves --------
{
  // Line picked first, point second: the point's instance (block 2) drops onto the line.
  const { scene, body1, body2 } = twoBlocks({ x: 100, y: 55 });
  const r = placeConstraint(scene, "coincident", { kind: "edge", bodyId: body1.id, index: 0 }, { kind: "vertex", bodyId: body2.id, index: 0 });
  check("point-on-line pose constraint accepted", r.constraint !== null && r.breaks.length === 0, `${r.breaks.length} breaks`);
  check("stored with the point as refA (model normalization)", r.constraint !== null && r.constraint.refA.kind === "vertex", r.constraint?.refA.kind ?? "null");
  // Block 1's top edge is y = -20; block 2's vertex 0 was at (80, 35): it drops 55.
  check("second-picked (point) side translated along the edge normal", Math.abs(body2.pos.x - 100) < 1e-9 && Math.abs(body2.pos.y - 0) < TOL, fmt(body2.pos));
  check("first-picked (line) instance never moved", dist(body1.pos, { x: 0, y: 0 }) < 1e-9, fmt(body1.pos));
}
{
  // Point picked first, line second: now the line's instance (block 1) comes to the point.
  const { scene, body1, body2 } = twoBlocks({ x: 100, y: 55 });
  const r = placeConstraint(scene, "coincident", { kind: "vertex", bodyId: body2.id, index: 0 }, { kind: "edge", bodyId: body1.id, index: 0 });
  check("reverse pick order accepted", r.constraint !== null && r.breaks.length === 0, `${r.breaks.length} breaks`);
  check("second-picked (line) side moved instead", dist(body2.pos, { x: 100, y: 55 }) < 1e-9 && Math.abs(body1.pos.y - 55) < TOL && Math.abs(body1.pos.x) < 1e-9, `b1 ${fmt(body1.pos)}`);
}

// --- tier 3: rotational pose constraints ----------------------------------------------
{
  // Block 2 placed tilted by 0.4 rad: a single-line H on its top edge rotates it back.
  const { scene, inst2, body1, body2 } = twoBlocks({ x: 200, y: 0 }, 0.4);
  const shape = JSON.stringify(body2.controlLocal);
  const e0 = scene.resolveMeasureRef({ kind: "edge", bodyId: body2.id, index: 0 })!;
  const mid0 = e0.kind === "line" ? { x: (e0.a.x + e0.b.x) / 2, y: (e0.a.y + e0.b.y) / 2 } : { x: 0, y: 0 };
  const rh = placeConstraint(scene, "horizontal", { kind: "edge", bodyId: body2.id, index: 0 });
  check("line H on an instance edge accepted", rh.constraint !== null && rh.breaks.length === 0, `${rh.breaks.length} breaks`);
  check("instance rotated onto horizontal", Math.abs(wrapHalfPi(edgeAngle(scene, body2.id, 0))) < 1e-6 && Math.abs(wrapHalfPi(body2.angle)) < 1e-6, `angle ${body2.angle.toFixed(5)}`);
  const e1 = scene.resolveMeasureRef({ kind: "edge", bodyId: body2.id, index: 0 })!;
  const mid1 = e1.kind === "line" ? { x: (e1.a.x + e1.b.x) / 2, y: (e1.a.y + e1.b.y) / 2 } : { x: 9, y: 9 };
  check("rotation pivots on the edge midpoint", dist(mid0, mid1) < TOL, `${fmt(mid0)} -> ${fmt(mid1)}`);
  check("no reshape from the rotation", JSON.stringify(body2.controlLocal) === shape, "shape identical");
  const placement = scene.instancePlacement(inst2.id)!;
  check("placement stays coherent after rotateInstance", Math.abs(wrapHalfPi(placement.angle)) < 1e-6, `placement angle ${placement.angle.toFixed(5)}`);

  // Rotating the instance against its H reads as violated while it's held (dragged),
  // and the un-anchored settle re-asserts it.
  scene.rotateInstance(inst2.id, body2.pos, 0.3);
  const heldBreaks = enforcePose(scene, new Set([inst2.id]));
  check("held rotation leaves the constraint violated", heldBreaks.length === 1 && poseConstraintViolated(scene, rh.constraint!), `${heldBreaks.length} breaks`);
  const settled = enforcePose(scene);
  check("un-anchored settle re-asserts the H", settled.length === 0 && !poseConstraintViolated(scene, rh.constraint!) && Math.abs(wrapHalfPi(body2.angle)) < 1e-6, `angle ${body2.angle.toFixed(5)}`);

  // Perpendicular between block 1's edge 0 and block 2's edge 0 (both horizontal now):
  // block 2 turns a quarter; then a coincident on top translates it — both hold.
  scene.removeSketchConstraint(rh.constraint!.id);
  const rp = placeConstraint(scene, "perpendicular", { kind: "edge", bodyId: body1.id, index: 0 }, { kind: "edge", bodyId: body2.id, index: 0 });
  const dd = wrapHalfPi(edgeAngle(scene, body2.id, 0) - edgeAngle(scene, body1.id, 0) - Math.PI / 2);
  check("perpendicular between instances accepted", rp.constraint !== null && Math.abs(dd) < 1e-6, `mismatch ${dd.toExponential(2)}`);
  check("refA's instance did not rotate", body1.angle === 0 && dist(body1.pos, { x: 0, y: 0 }) < 1e-9, fmt(body1.pos));
  const rc = placeConstraint(scene, "coincident", { kind: "vertex", bodyId: body1.id, index: 1 }, { kind: "vertex", bodyId: body2.id, index: 0 });
  const v1 = scene.resolveMeasureRef({ kind: "vertex", bodyId: body1.id, index: 1 })!;
  const v0 = scene.resolveMeasureRef({ kind: "vertex", bodyId: body2.id, index: 0 })!;
  const gap = v1.kind === "point" && v0.kind === "point" ? dist(v1.p, v0.p) : Infinity;
  const dd2 = wrapHalfPi(edgeAngle(scene, body2.id, 0) - edgeAngle(scene, body1.id, 0) - Math.PI / 2);
  check("rotation + translation settle together", rc.constraint !== null && gap < TOL && Math.abs(dd2) < 1e-6, `gap ${gap.toExponential(2)}, mismatch ${dd2.toExponential(2)}`);

  // Parallel: block 1 grounded → block 2 is the one that turns; both grounded rejects.
  scene.removeSketchConstraint(rp.constraint!.id);
  scene.removeSketchConstraint(rc.constraint!.id);
  scene.toggleBodyGround(body1.id);
  const rpar = placeConstraint(scene, "parallel", { kind: "edge", bodyId: body2.id, index: 0 }, { kind: "edge", bodyId: body1.id, index: 0 });
  const dd3 = wrapHalfPi(edgeAngle(scene, body2.id, 0) - edgeAngle(scene, body1.id, 0));
  check("parallel turns the ungrounded side", rpar.constraint !== null && Math.abs(dd3) < 1e-6 && body1.angle === 0, `mismatch ${dd3.toExponential(2)}`);
  scene.removeSketchConstraint(rpar.constraint!.id);
  scene.rotateInstance(inst2.id, body2.pos, 0.5);
  scene.toggleBodyGround(body2.id);
  const before = JSON.stringify(scene.serialize());
  const rrej = placeConstraint(scene, "parallel", { kind: "edge", bodyId: body2.id, index: 0 }, { kind: "edge", bodyId: body1.id, index: 0 });
  check("parallel between two grounded instances rejects", rrej.constraint === null && rrej.breaks.length > 0 && JSON.stringify(scene.serialize()) === before, `${rrej.breaks.length} breaks`);
}

// --- rotateInstance carries free chassis points ---------------------------------------
{
  const scene = new Scene();
  const base = square(scene, 0, 0);
  scene.toggleBodyGround(base.id);
  const free = scene.addFreeJoint({ x: 0, y: -50 });
  const res = scene.createComponentFromSelection("Snap", [base.id], [free.id])!;
  const inst = res.instance;
  const body = scene.getBody(inst.bodyMap[0].id)!;
  const fid = inst.jointMap.find((e) => scene.getJoint(e.id)?.bodyId === null)!.id;
  scene.rotateInstance(inst.id, { x: 0, y: 0 }, Math.PI / 2);
  const fw = scene.jointWorld(scene.getJoint(fid)!);
  check("free chassis point orbits with the bodies", dist(fw, { x: 50, y: 0 }) < 1e-9 && Math.abs(body.angle - Math.PI / 2) < 1e-12, fmt(fw));
  const t = scene.instancePlacement(inst.id)!;
  check("placement reports the rotation", Math.abs(t.angle - Math.PI / 2) < 1e-9 && dist(t.pos, { x: 0, y: 0 }) < 1e-9, `angle ${t.angle.toFixed(5)} pos ${fmt(t.pos)}`);
}

// --- intra-instance constraints: re-pose the internal mechanism ------------------------
{
  const build = () => {
    const scene = new Scene();
    const base = square(scene, 0, 0, 30);
    scene.toggleBodyGround(base.id); // chassis
    const arm = square(scene, 60, 0, 30); // pinned to the base at (30, 0)
    const jb = scene.addJoint(base.id, { x: 30, y: 0 });
    const ja = scene.addJoint(arm.id, { x: 30, y: 0 });
    scene.addPin(jb.id, ja.id);
    const res = scene.createComponentFromSelection("Linkage", [base.id, arm.id])!;
    const inst = res.instance;
    const nBase = scene.getBody(inst.bodyMap.find((e) => e.chassis)!.id)!;
    const nArm = scene.getBody(inst.bodyMap.find((e) => !e.chassis)!.id)!;
    const pinGap = () => {
      const pin = scene.constraints.find((c) => c.kind === "pin")!;
      return pin.kind === "pin"
        ? dist(scene.jointWorld(scene.getJoint(pin.jointA)!), scene.jointWorld(scene.getJoint(pin.jointB)!))
        : Infinity;
    };
    return { scene, def: res.def, nBase, nArm, pinGap };
  };

  // Translational: level the arm's far-bottom vertex (90, 30) with the base's top-left
  // (-30, -30) — the arm pivots about the pin to bring that vertex to y = -30.
  {
    const { scene, def, nBase, nArm, pinGap } = build();
    const defBefore = JSON.stringify(def.data);
    const armShape = JSON.stringify(nArm.controlLocal);
    const r = placeConstraint(scene, "horizontal", { kind: "vertex", bodyId: nBase.id, index: 0 }, { kind: "vertex", bodyId: nArm.id, index: 2 });
    check("intra H point pair accepted", r.constraint !== null && r.breaks.length === 0, `${r.breaks.length} breaks`);
    const v = scene.resolveMeasureRef({ kind: "vertex", bodyId: nArm.id, index: 2 })!;
    check("arm re-posed to level the pair", v.kind === "point" && Math.abs(v.p.y + 30) < TOL, v.kind === "point" ? fmt(v.p) : "?");
    check("held side never moved", dist(nBase.pos, { x: 0, y: 0 }) < 1e-9 && nBase.angle === 0, fmt(nBase.pos));
    check("the pin stayed closed", pinGap() < TOL, `gap ${pinGap().toExponential(2)}`);
    check("no reshape, definition untouched", JSON.stringify(nArm.controlLocal) === armShape && JSON.stringify(def.data) === defBefore, "identical");
  }

  // Rotational: base edge 0 ⊥ arm edge 0 — the arm swings a quarter turn about the pin.
  {
    const { scene, nBase, nArm, pinGap } = build();
    const r = placeConstraint(scene, "perpendicular", { kind: "edge", bodyId: nBase.id, index: 0 }, { kind: "edge", bodyId: nArm.id, index: 0 });
    const dd = wrapHalfPi(edgeAngle(scene, nArm.id, 0) - edgeAngle(scene, nBase.id, 0) - Math.PI / 2);
    check("intra perpendicular accepted", r.constraint !== null && r.breaks.length === 0, `${r.breaks.length} breaks`);
    check("arm edge turned perpendicular", Math.abs(dd) < 1e-4, `mismatch ${dd.toExponential(2)}`);
    check("base held, pin closed", dist(nBase.pos, { x: 0, y: 0 }) < 1e-9 && pinGap() < TOL, `gap ${pinGap().toExponential(2)}`);
  }

  // Rigid at a deeper level: two edges of one instance body can't be made perpendicular
  // (they're 90° already — pick two parallel ones), and equal between instance edges
  // never enters. Both reject with nothing created.
  {
    const { scene, nBase, nArm } = build();
    const n = scene.sketch.length;
    const r1 = placeConstraint(scene, "perpendicular", { kind: "edge", bodyId: nArm.id, index: 0 }, { kind: "edge", bodyId: nArm.id, index: 2 });
    check("constraint rigid at a deeper level rejects", r1.constraint === null && scene.sketch.length === n, "nothing created");
    const r2 = placeConstraint(scene, "equal", { kind: "edge", bodyId: nBase.id, index: 0 }, { kind: "edge", bodyId: nArm.id, index: 0 });
    check("equal between instance edges rejects", r2.constraint === null && scene.sketch.length === n, "nothing created");
  }
}

if (failures > 0) {
  console.error(`${failures} FAILURE(S)`);
  process.exit(1);
}
console.log("All pose-constraint checks passed");
