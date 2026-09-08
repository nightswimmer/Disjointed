/**
 * Headless coverage for hierarchical components (v14). Verifies that:
 *  - createComponentFromSelection packs a selection into a definition (bodies, joints,
 *    pins, grounds, sliders, sketch constraints, dimensions) and replaces it with one
 *    instance placed exactly where the originals were; the definition's constraints do
 *    NOT exist in the assembly (they live only in the def).
 *  - Grounding inside a definition converts on expansion: grounded bodies + grounded
 *    free joints become one rigid chassis group (never grounded to the world); a
 *    joint-ground on a non-grounded body becomes a pin to a synthesized chassis point
 *    (revolute to the component frame).
 *  - Instances simulate correctly: the chassis is rigid, internal mechanisms still move,
 *    grounding the instance fixes the chassis and internal parts pivot about their
 *    converted anchors.
 *  - Editing the definition cascades: re-expansion updates every instance's shape while
 *    preserving each instance's placement (translation + rotation) and the scene ids of
 *    surviving elements; every part's pose snaps back to the definition layout (the def
 *    is the reference); added/removed def elements appear/disappear; nested definitions
 *    (a def using another def) cascade through cascadeComponentChange.
 *  - Empty components: createEmptyComponent starts a blank definition (instantiation is
 *    refused until it has content) that can be filled with bodies and/or instances of
 *    other components.
 *  - removeInstance / dissolveInstance / removeComponent behave; serialize/load
 *    round-trips components + instances (v14) and pre-v14 files load with none.
 */
import {
  Scene,
  SceneData,
  cascadeComponentChange,
  reexpandData,
} from "../src/model";
import { solve, Driver } from "../src/solver";
import { applyDrivingDimension, solveSketch } from "../src/sketch";
import { Vec2, dist, sub, rotate } from "../src/geometry";

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

/** Move a whole instance like the UI does: every mapped body + free joint by `delta`. */
function moveInstance(scene: Scene, instId: number, delta: Vec2): void {
  const inst = scene.instances.find((i) => i.id === instId)!;
  for (const e of inst.bodyMap) scene.moveBody(e.id, delta);
  for (const e of [...inst.jointMap, ...inst.anchorMap]) {
    const j = scene.getJoint(e.id);
    if (j && j.bodyId === null) scene.moveJoint(e.id, delta);
  }
}

function rotateInstance(scene: Scene, instId: number, pivot: Vec2, ang: number): void {
  const inst = scene.instances.find((i) => i.id === instId)!;
  for (const e of inst.bodyMap) scene.rotateBody(e.id, pivot, ang);
  for (const e of [...inst.jointMap, ...inst.anchorMap]) {
    const j = scene.getJoint(e.id);
    if (j && j.bodyId === null) {
      const w = scene.jointWorld(j);
      scene.moveJoint(e.id, sub(rotate(sub(w, pivot), ang), sub(w, pivot)));
    }
  }
}

/** Edit a definition the way the app does: load its data, mutate, store back. */
function editDef(scene: Scene, defId: number, mutate: (s: Scene) => void): Set<number> {
  const def = scene.getComponent(defId)!;
  const tmp = new Scene();
  tmp.loadContext(def.data);
  tmp.components = scene.components;
  mutate(tmp);
  def.data = tmp.serializeContext();
  const changed = cascadeComponentChange(scene.components, [defId]);
  scene.reexpandInstances(changed);
  return changed;
}

// --- creation: pack a selection into a definition -----------------------------
{
  const scene = new Scene();
  const base = square(scene, 0, 0); // will be grounded → chassis
  const arm = square(scene, 100, 0);
  const bj = scene.addJoint(base.id, { x: 15, y: 0 });
  const aj = scene.addJoint(arm.id, { x: 85, y: 0 });
  scene.addPin(bj.id, aj.id);
  scene.toggleBodyGround(base.id);
  scene.addSketchConstraint("horizontal", { kind: "edge", bodyId: base.id, index: 0 });
  scene.addMeasurement("draw", { kind: "joint", jointId: bj.id }, { kind: "joint", jointId: aj.id }, { x: 50, y: -60 });
  const armPosBefore = { ...scene.getBody(arm.id)!.pos };

  const res = scene.createComponentFromSelection("Linkage", [base.id, arm.id]);
  check("creation returns def + instance", res !== null, res ? `def ${res.def.id}` : "null");
  const { def, instance } = res!;
  check("definition stored", scene.components.length === 1 && scene.getComponent(def.id) === def, def.name);
  check("def data carries the sketch constraint", (def.data.sketch ?? []).length === 1, `${def.data.sketch?.length}`);
  check("def data carries the measurement", (def.data.measurements ?? []).length === 1, `${def.data.measurements?.length}`);
  check("def data keeps the grounded flag", def.data.bodies.filter((b) => b.grounded).length === 1, "1 grounded body");
  check("assembly has no sketch constraints", scene.sketch.length === 0, `${scene.sketch.length}`);
  check("assembly has no measurements", scene.measurements.length === 0, `${scene.measurements.length}`);
  check("instance replaced the originals", scene.bodies.length === 2 && scene.instances.length === 1, `${scene.bodies.length} bodies`);
  check("instance bodies are not world-grounded", scene.bodies.every((b) => !b.grounded), "flags stripped");
  const newArm = scene.getBody(instance.bodyMap.find((e) => !e.chassis)!.id)!;
  check("instance placed exactly in place", dist(newArm.pos, armPosBefore) < 1e-9, `arm at (${newArm.pos.x}, ${newArm.pos.y})`);
  check("pin expanded into the assembly", scene.constraints.filter((c) => c.kind === "pin").length === 1, "1 pin");
  check("no world grounds in the assembly", scene.constraints.every((c) => c.kind !== "ground"), "0 grounds");
  check("chassis flag marks the base", instance.bodyMap.filter((e) => e.chassis).length === 1, "1 chassis body");
  check(
    "sketch constraint on instance geometry rejected",
    scene.addSketchConstraint("horizontal", { kind: "edge", bodyId: newArm.id, index: 0 }) === null,
    "rejected"
  );
}

// --- driving dimensions vs instance geometry ------------------------------------
{
  const scene = new Scene();
  const base = square(scene, 0, 0);
  scene.toggleBodyGround(base.id);
  const res = scene.createComponentFromSelection("Block", [base.id])!;
  const inst = res.instance;
  const instBody = scene.getBody(inst.bodyMap[0].id)!;

  // A dimension fully internal to the instance: driving it must be rejected — the
  // shape belongs to the definition — and must not reshape the instance (this used to
  // scale the body while the label stayed "driven").
  const before = scene.bodyControlWorld(instBody).map((p) => ({ ...p }));
  const mIn = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: instBody.id, index: 0 },
    { kind: "vertex", bodyId: instBody.id, index: 1 },
    { x: 0, y: -60 }
  )!;
  const rej = applyDrivingDimension(scene, mIn.id, 80);
  const after = scene.bodyControlWorld(scene.getBody(instBody.id)!);
  const drift = Math.max(...before.map((p, i) => dist(p, after[i])));
  check("dim internal to an instance can't drive", rej.length > 0, `${rej.length} breaks`);
  check("rejected edit leaves the instance shape untouched", drift < 1e-9, `drift ${drift.toExponential(2)}`);
  check("rejected dim stays driven", !scene.getMeasurement(mIn.id)!.driving, "no driving flag");

  // A dimension between the instance and free geometry CAN drive — by moving only the
  // free side (instance variables are rank-immovable in the sketch solve).
  const fj = scene.addFreeJoint({ x: 100, y: 0 });
  const mMix = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: instBody.id, index: 1 },
    { kind: "joint", jointId: fj.id },
    { x: 60, y: -60 }
  )!;
  const ok = applyDrivingDimension(scene, mMix.id, 150);
  const after2 = scene.bodyControlWorld(scene.getBody(instBody.id)!);
  const drift2 = Math.max(...before.map((p, i) => dist(p, after2[i])));
  const val = scene.measureInfo(scene.getMeasurement(mMix.id)!)!.value;
  check("mixed instance↔free dim drives", ok.length === 0 && scene.getMeasurement(mMix.id)!.driving === true, `${ok.length} breaks`);
  check("mixed dim hits its target", Math.abs(val - 150) < 1e-2, `value ${val.toFixed(3)}`);
  check("mixed dim moved only the free side", drift2 < 1e-9, `instance drift ${drift2.toExponential(2)}`);

  // Later sketch solves keep the instance immovable: move the instance, re-solve — the
  // free joint follows to hold the dimension, the instance stays where it was put.
  moveInstance(scene, inst.id, { x: 0, y: 30 });
  const posMoved = { ...scene.getBody(instBody.id)!.pos };
  const solved = solveSketch(scene);
  const val2 = scene.measureInfo(scene.getMeasurement(mMix.id)!)!.value;
  check("re-solve holds the dim by moving the free side", solved.length === 0 && Math.abs(val2 - 150) < 1e-2, `value ${val2.toFixed(3)}`);
  check("re-solve never moves the instance", dist(scene.getBody(instBody.id)!.pos, posMoved) < 1e-9, "held");
}

// --- joint-ground conversion: revolute to the component frame ------------------
{
  const scene = new Scene();
  const base = square(scene, 0, 0);
  scene.toggleBodyGround(base.id);
  const arm = square(scene, 100, 0);
  const pivot = scene.addJoint(arm.id, { x: 85, y: 0 });
  scene.addGround(pivot.id, { x: 85, y: 0 }); // revolute anchor in the def-to-be
  const res = scene.createComponentFromSelection("Pivot arm", [base.id, arm.id])!;
  const inst = res.instance;

  check("anchor joint synthesized", inst.anchorMap.length === 1, `${inst.anchorMap.length}`);
  const anchorJ = scene.getJoint(inst.anchorMap[0].id)!;
  check("anchor sits at the def anchor", dist(scene.jointWorld(anchorJ), { x: 85, y: 0 }) < 1e-9, "at (85, 0)");
  check("chassis group locks base + anchor", inst.groupId !== null, `group ${inst.groupId}`);
  const g = scene.groups.find((x) => x.id === inst.groupId)!;
  check("group members", g.bodyIds.length === 1 && g.jointIds.length === 1, `${g.bodyIds.length}b + ${g.jointIds.length}j`);
  check("ground became a pin", scene.constraints.filter((c) => c.kind === "pin").length === 1, "1 pin");

  // Ground the instance (fix the chassis), then drag the arm: it must pivot about the anchor.
  scene.toggleBodyGround(g.bodyIds[0]);
  const armBody = scene.getBody(inst.bodyMap.find((e) => !e.chassis)!.id)!;
  const drv: Driver = { bodyId: armBody.id, local: { x: 0, y: 0 }, target: { x: 85, y: 120 } };
  let breaks: ReturnType<typeof solve> = [];
  for (let i = 0; i < 120; i++) breaks = solve(scene, drv, 60);
  const r = dist(armBody.pos, { x: 85, y: 0 });
  check("arm pivots about the converted anchor", Math.abs(r - 15) < 1e-2 && armBody.pos.y > 10, `radius ${r.toFixed(3)}, y ${armBody.pos.y.toFixed(1)}`);
  check("anchor never moved", dist(scene.jointWorld(scene.getJoint(inst.anchorMap[0].id)!), { x: 85, y: 0 }) < 1e-6, "held");
  check("no breaks while pivoting", breaks.length === 0, `${breaks.length}`);
}

// --- multiple instances + rigid chassis + internal mechanism -------------------
{
  const scene = new Scene();
  const baseA = square(scene, 0, 0);
  const baseB = square(scene, 60, 0);
  scene.toggleBodyGround(baseA.id);
  scene.toggleBodyGround(baseB.id);
  const rod = square(scene, 0, 60, 45); // overlaps the base so the pin joints coincide
  const j1 = scene.addJoint(baseA.id, { x: 0, y: 15 });
  const j2 = scene.addJoint(rod.id, { x: 0, y: 15 });
  scene.addPin(j1.id, j2.id);
  const res = scene.createComponentFromSelection("Cart", [baseA.id, baseB.id, rod.id])!;
  const first = res.instance;

  const second = scene.instantiateComponent(res.def.id, { pos: { x: 300, y: 0 }, angle: Math.PI / 2 })!;
  check("second instance expanded", scene.instances.length === 2 && second.bodyMap.length === 3, `${scene.instances.length} instances`);
  const sb = scene.getBody(second.bodyMap.find((e) => e.chassis)!.id)!;
  check("second instance rotated into place", Math.abs(sb.angle - Math.PI / 2) < 1e-9, `angle ${sb.angle.toFixed(3)}`);

  // Chassis rigidity: tow the first instance's rod; its two chassis bodies keep their
  // relative pose (in the first chassis body's frame), and the second instance holds still.
  const chassis = first.bodyMap.filter((e) => e.chassis).map((e) => scene.getBody(e.id)!);
  const relPose = () => ({
    off: rotate(sub(chassis[1].pos, chassis[0].pos), -chassis[0].angle),
    dAng: chassis[1].angle - chassis[0].angle,
  });
  const relBefore = relPose();
  const rodBody = scene.getBody(first.bodyMap.find((e) => !e.chassis)!.id)!;
  const secondPos = { ...sb.pos };
  const drv: Driver = { bodyId: rodBody.id, local: { x: 0, y: 0 }, target: { x: -100, y: 200 } };
  for (let i = 0; i < 80; i++) solve(scene, drv, 60);
  const relAfter = relPose();
  const drift = Math.max(dist(relBefore.off, relAfter.off), Math.abs(relBefore.dAng - relAfter.dAng) * 100);
  check("chassis stays rigid under tow", drift < 1e-9, `drift ${drift.toExponential(2)}`);
  check("rod moved relative to chassis", dist(rodBody.pos, { x: 0, y: 60 }) > 50, `moved ${dist(rodBody.pos, { x: 0, y: 60 }).toFixed(1)}`);
  check("other instance untouched", dist(sb.pos, secondPos) < 1e-9, `moved ${dist(sb.pos, secondPos).toExponential(2)}`);
}

// --- cascade: definition edits update every instance ----------------------------
{
  const scene = new Scene();
  const base = square(scene, 0, 0);
  scene.toggleBodyGround(base.id);
  const arm = square(scene, 100, 0);
  const bj = scene.addJoint(base.id, { x: 15, y: 0 });
  const aj = scene.addJoint(arm.id, { x: 85, y: 0 });
  scene.addPin(bj.id, aj.id);
  const res = scene.createComponentFromSelection("Widget", [base.id, arm.id])!;
  const defId = res.def.id;
  const inst1 = res.instance;
  const inst2 = scene.instantiateComponent(defId, { pos: { x: 400, y: 100 }, angle: 0 })!;

  // Move + rotate instance 2, remember an id and its placement.
  moveInstance(scene, inst2.id, { x: 50, y: -30 });
  rotateInstance(scene, inst2.id, { x: 500, y: 0 }, Math.PI / 4);
  const inst2Chassis = scene.getBody(inst2.bodyMap.find((e) => e.chassis)!.id)!;
  const chassisPoseBefore = { pos: { ...inst2Chassis.pos }, angle: inst2Chassis.angle };
  const armIdBefore = inst2.bodyMap.find((e) => !e.chassis)!.id;

  // Edit the def: recolor the arm, grow the base, add a third body, and check cascade.
  editDef(scene, defId, (s) => {
    const armDef = s.bodies.find((b) => !b.grounded)!;
    armDef.color = "#abcdef";
    square(s, -80, 0, 10); // new body in the def
  });

  check("instance count unchanged", scene.instances.length === 2, `${scene.instances.length}`);
  check("both instances gained the new body", scene.instances.every((i) => i.bodyMap.length === 3), `${inst1.bodyMap.length} / ${inst2.bodyMap.length}`);
  const inst2ArmAfter = scene.getBody(inst2.bodyMap.find((e) => !e.chassis && e.id === armIdBefore)?.id ?? -1);
  check("surviving body kept its scene id", inst2ArmAfter !== undefined && inst2ArmAfter !== null, `id ${armIdBefore}`);
  check("recolor cascaded", inst2ArmAfter!.color === "#abcdef", inst2ArmAfter!.color);
  const chassisAfter = scene.getBody(inst2.bodyMap.find((e) => e.chassis)!.id)!;
  check(
    "instance placement preserved (pos)",
    dist(chassisAfter.pos, chassisPoseBefore.pos) < 1e-6,
    `moved ${dist(chassisAfter.pos, chassisPoseBefore.pos).toExponential(2)}`
  );
  check(
    "instance placement preserved (angle)",
    Math.abs(chassisAfter.angle - chassisPoseBefore.angle) < 1e-9,
    `dAngle ${(chassisAfter.angle - chassisPoseBefore.angle).toExponential(2)}`
  );
  // The added def body lands at the instance's placement (rotated 45°).
  const added = inst2.bodyMap.find((e) => e.chassis === false && e.id !== armIdBefore)!;
  const addedBody = scene.getBody(added.id)!;
  check("new def body arrives rotated with the instance", Math.abs(addedBody.angle - Math.PI / 4) < 1e-9, `angle ${addedBody.angle.toFixed(3)}`);

  // Remove the arm from the def: instances lose it (and the pin dies with its joint).
  editDef(scene, defId, (s) => {
    const armDef = s.bodies.find((b) => b.color === "#abcdef")!;
    s.removeBody(armDef.id);
  });
  check("removed def body cascades away", scene.instances.every((i) => i.bodyMap.length === 2), `${inst1.bodyMap.length} bodies`);
  check("its pin cascaded away too", scene.constraints.filter((c) => c.kind === "pin").length === 0, "0 pins");
}

// --- containment warning: a def edit stranding an assembly joint is flagged, not moved ----
{
  const scene = new Scene();
  const base = square(scene, 0, 0);
  scene.toggleBodyGround(base.id);
  const arm = square(scene, 100, 0);
  const res = scene.createComponentFromSelection("Widget", [base.id, arm.id])!;
  const defId = res.def.id;
  const instArmId = res.instance.bodyMap.find((e) => !e.chassis)!.id;

  // An assembly-level joint on the instance's arm, well inside the ±20 outline.
  const aj = scene.addJoint(instArmId, { x: 115, y: 0 });
  check("containment: clean scene reports none", scene.jointsOutsideBody().length === 0, "0 outside");

  // A joint clamped exactly onto the outline (the drag behaviour) must not false-positive.
  const edge = scene.addJoint(instArmId, { x: 110, y: 0 });
  scene.moveJoint(edge.id, { x: 500, y: 0 }); // clamps to the outline at x = 120
  check(
    "containment: edge-clamped joint is not flagged",
    scene.jointsOutsideBody().length === 0,
    `joint at x = ${scene.jointWorld(scene.getJoint(edge.id)!).x}`
  );
  scene.removeJoint(edge.id);

  // Shrink the arm in the def: the instance body reshapes under the assembly joint.
  editDef(scene, defId, (s) => {
    const armDef = s.bodies.find((b) => !b.grounded)!;
    s.scaleBody(armDef.id, 0.5); // arm now spans ±10 — world (115, 0) is outside
  });
  const outside = scene.jointsOutsideBody();
  check("containment: stranded joint flagged after cascade", outside.length === 1 && outside[0] === aj.id, `[${outside}]`);
  const w = scene.jointWorld(scene.getJoint(aj.id)!);
  check("containment: the joint was NOT auto-moved", dist(w, { x: 115, y: 0 }) < 1e-9, `at (${w.x}, ${w.y})`);

  // Dragging the joint clamps it back onto the body — the flag clears itself.
  scene.moveJoint(aj.id, { x: -1, y: 0 });
  check("containment: dragging back inside clears the flag", scene.jointsOutsideBody().length === 0, "0 outside");
}

// --- re-expansion snaps poses back to the definition layout ------------------------
{
  const scene = new Scene();
  const base = square(scene, 0, 0);
  scene.toggleBodyGround(base.id);
  const arm = square(scene, 100, 0);
  const bj = scene.addJoint(base.id, { x: 15, y: 0 });
  const aj = scene.addJoint(arm.id, { x: 85, y: 0 });
  scene.addPin(bj.id, aj.id);
  const free = scene.addFreeJoint({ x: 0, y: -50 });
  const res = scene.createComponentFromSelection("Snap", [base.id, arm.id], [free.id])!;
  const inst = res.instance;
  moveInstance(scene, inst.id, { x: 200, y: 100 }); // place the instance somewhere
  const armEntry = inst.bodyMap.find((e) => !e.chassis)!;
  const armHome = { ...scene.getBody(armEntry.id)!.pos }; // = T · def pose
  const freeId = inst.jointMap.find((e) => scene.getJoint(e.id)?.bodyId === null)!.id;
  const freeHome = { ...scene.jointWorld(scene.getJoint(freeId)!) };
  const chassisBody = scene.getBody(inst.bodyMap.find((e) => e.chassis)!.id)!;
  const chassisPose = { pos: { ...chassisBody.pos }, angle: chassisBody.angle };

  // Pose the mechanism away from the def layout (drag the arm and the free joint).
  scene.moveBody(armEntry.id, { x: 40, y: -25 });
  scene.rotateBody(armEntry.id, scene.getBody(armEntry.id)!.pos, 0.7);
  scene.moveJoint(freeId, { x: -30, y: 10 });

  // Any def edit re-expands: the definition is the reference, poses snap back.
  editDef(scene, res.def.id, (s) => {
    s.bodies[0].color = "#222222";
  });
  const armAfter = scene.getBody(armEntry.id)!;
  check(
    "re-expansion snaps a posed body back to the def layout",
    dist(armAfter.pos, armHome) < 1e-9 && Math.abs(armAfter.angle) < 1e-9,
    `pos off ${dist(armAfter.pos, armHome).toExponential(2)}, angle ${armAfter.angle.toFixed(3)}`
  );
  const freeAfter = scene.jointWorld(scene.getJoint(freeId)!);
  check(
    "re-expansion snaps a posed free joint back",
    dist(freeAfter, freeHome) < 1e-9,
    `off ${dist(freeAfter, freeHome).toExponential(2)}`
  );
  const chassisAfter = scene.getBody(inst.bodyMap.find((e) => e.chassis)!.id)!;
  check(
    "instance placement itself is kept",
    dist(chassisAfter.pos, chassisPose.pos) < 1e-9 && Math.abs(chassisAfter.angle - chassisPose.angle) < 1e-9,
    `moved ${dist(chassisAfter.pos, chassisPose.pos).toExponential(2)}`
  );
}

// --- empty components ---------------------------------------------------------------
{
  const scene = new Scene();
  const blank = scene.createEmptyComponent("Blank");
  check(
    "empty component stored with no content",
    scene.components.length === 1 && blank.data.bodies.length === 0 && blank.data.joints.length === 0,
    blank.name
  );
  check(
    "empty definition refuses instantiation",
    scene.instantiateComponent(blank.id, { pos: { x: 0, y: 0 }, angle: 0 }) === null,
    "null"
  );
  // Fill it the way the editor does (load its context, add material, store back).
  editDef(scene, blank.id, (s) => {
    const b = square(s, 0, 0);
    s.toggleBodyGround(b.id);
  });
  const inst = scene.instantiateComponent(blank.id, { pos: { x: 50, y: 50 }, angle: 0 });
  check("filled empty component instantiates", inst !== null && inst.bodyMap.length === 1, `${inst?.bodyMap.length ?? 0} bodies`);

  // A component made entirely of other components: an empty def filled with only an
  // instance of another def (the workflow that used to need a throwaway body).
  const wrapper = scene.createEmptyComponent("Wrapper");
  editDef(scene, wrapper.id, (s) => {
    s.instantiateComponent(blank.id, { pos: { x: 0, y: 0 }, angle: 0 });
  });
  const wInst = scene.instantiateComponent(wrapper.id, { pos: { x: 300, y: 0 }, angle: 0 });
  check(
    "empty def filled with only another component instantiates",
    wInst !== null && wInst.bodyMap.length === 1,
    `${wInst?.bodyMap.length ?? 0} bodies`
  );
  check("wrapper appears in the def DAG", scene.componentUses(wrapper.id).has(blank.id), "Wrapper uses Blank");
}

// --- nested definitions cascade -------------------------------------------------
{
  const scene = new Scene();
  const core = square(scene, 0, 0);
  scene.toggleBodyGround(core.id);
  const resA = scene.createComponentFromSelection("Inner", [core.id])!;
  const defA = resA.def.id;

  // Build "Outer" around an instance of Inner plus its own body, in the root context.
  const shell = square(scene, 80, 0, 10);
  scene.toggleBodyGround(shell.id);
  // Select the Inner instance's material via dissolve → it becomes plain… instead, build
  // Outer directly as a def whose data instantiates Inner (the app does this by editing
  // a def context; here we edit a fresh def the same way).
  scene.removeInstance(resA.instance.id);
  scene.removeBody(shell.id);
  const outerId = scene.components.reduce((m, c) => Math.max(m, c.id), 0) + 1;
  {
    const tmp = new Scene();
    tmp.components = scene.components;
    const s2 = square(tmp, 80, 0, 10);
    tmp.toggleBodyGround(s2.id);
    tmp.instantiateComponent(defA, { pos: { x: 0, y: 0 }, angle: 0 });
    scene.components.push({ id: outerId, name: "Outer", data: tmp.serializeContext() });
  }
  const outerInst = scene.instantiateComponent(outerId, { pos: { x: 500, y: 500 }, angle: 0 })!;
  check("outer instance expands nested material", outerInst.bodyMap.length === 2, `${outerInst.bodyMap.length} bodies`);

  // Edit Inner: scale its body; the change must ripple Outer → root.
  const before = scene.getBody(outerInst.bodyMap[1]?.id ?? outerInst.bodyMap[0].id)!;
  const beforeVerts = before.local.length;
  editDef(scene, defA, (s) => {
    s.scaleBody(s.bodies[0].id, 2);
  });
  const innerInOuter = scene.instances.find((i) => i.defId === outerId)!;
  const bodies = innerInOuter.bodyMap.map((e) => scene.getBody(e.id)!);
  const grew = bodies.some((b) => Math.max(...b.local.map((p) => Math.hypot(p.x, p.y))) > 35);
  check("nested edit cascades to the root instance", grew, `verts ${beforeVerts}, max r ${Math.max(...bodies.flatMap((b) => b.local.map((p) => Math.hypot(p.x, p.y)))).toFixed(1)}`);
  check("cycle guard: Inner can't use Outer", scene.componentUses(outerId).has(defA) && !scene.componentUses(defA).has(outerId), "DAG direction");
}

// --- powered constraints inside a component ---------------------------------------
{
  const scene = new Scene();
  // A world-fixed track (two grounded free joints) with a linear actuator, plus a base.
  const base = square(scene, -80, 0);
  scene.toggleBodyGround(base.id);
  const r1 = scene.addFreeJoint({ x: 0, y: 0 });
  const r2 = scene.addFreeJoint({ x: 100, y: 0 });
  const slider = scene.addSlider(r1.id, r2.id); // auto-grounds both rail joints
  const act = scene.addLinearActuator(slider.id, { x: 20, y: 0 })!;
  const res = scene.createComponentFromSelection("Pusher", [base.id], [r1.id, r2.id, act.riderId])!;
  const inst = res.instance;

  const kinds = scene.constraints.map((c) => c.kind).sort().join(",");
  check("slider + actuator expanded, no grounds", kinds === "linearActuator,slider", kinds);
  const g = scene.groups.find((x) => x.id === inst.groupId);
  check("rail joints locked to the chassis", g !== undefined && g.jointIds.length === 2, `${g?.jointIds.length ?? 0} locked joints`);

  // Drive the rider along the chassis rail via the solver's anchors (like animation does).
  const actExp = scene.constraints.find((c) => c.kind === "linearActuator")!;
  const rider = scene.getJoint((actExp as { riderId: number }).riderId)!;
  const anchors = new Map<number, Vec2>([[rider.id, { x: 80, y: 0 }]]);
  let breaks: ReturnType<typeof solve> = [];
  for (let i = 0; i < 40; i++) breaks = solve(scene, null, 60, 1, anchors);
  const q = scene.jointWorld(rider);
  check("anchor drives the rider along the chassis track", dist(q, { x: 80, y: 0 }) < 1e-3, `at (${q.x.toFixed(3)}, ${q.y.toFixed(3)})`);
  check("no breaks driving the expanded actuator", breaks.length === 0, `${breaks.length}`);
}

// --- removal / dissolve / component deletion -------------------------------------
{
  const scene = new Scene();
  const b1 = square(scene, 0, 0);
  scene.toggleBodyGround(b1.id);
  const b2 = square(scene, 60, 0);
  const res = scene.createComponentFromSelection("Thing", [b1.id, b2.id])!;
  const defId = res.def.id;
  const inst2 = scene.instantiateComponent(defId, { pos: { x: 200, y: 0 }, angle: 0 })!;

  // An assembly-level pin from outside onto an instance joint dies with the instance.
  const outside = square(scene, -100, 0);
  const oj = scene.addJoint(outside.id, { x: -100, y: 0 });
  const instBody = scene.getBody(res.instance.bodyMap[1].id)!;
  const ij = scene.addJoint(instBody.id, instBody.pos);
  scene.addPin(oj.id, ij.id);
  check("assembly joint on an instance body is not instance-owned", scene.instanceOfJoint(ij.id) === undefined, "external joint");

  check("removeComponent refused while instances exist", scene.removeComponent(defId) === false, "refused");
  scene.removeInstance(res.instance.id);
  check("removeInstance drops its elements", scene.instances.length === 1 && scene.bodies.length === 3, `${scene.bodies.length} bodies left`);
  check("external pin cascaded away", scene.constraints.filter((c) => c.kind === "pin").length === 0, "0 pins");

  scene.dissolveInstance(inst2.id);
  check("dissolve keeps the elements", scene.instances.length === 0 && scene.bodies.length === 3, `${scene.bodies.length} bodies`);
  check("removeComponent works once unused", scene.removeComponent(defId) === true && scene.components.length === 0, "deleted");
}

// --- fork: make an instance unique --------------------------------------------------
{
  const scene = new Scene();
  const b1 = square(scene, 0, 0);
  scene.toggleBodyGround(b1.id);
  const b2 = square(scene, 60, 0);
  const res = scene.createComponentFromSelection("Widget", [b1.id, b2.id])!;
  const inst2 = scene.instantiateComponent(res.def.id, { pos: { x: 200, y: 0 }, angle: 0 })!;

  const posBefore = scene.bodies.map((b) => ({ ...b.pos }));
  const copy = scene.makeInstanceUnique(inst2.id)!;
  check("fork returns a new definition", copy !== null && copy.id !== res.def.id, `def ${copy?.id}`);
  check("fork names the copy after the source", copy.name === "Widget copy", copy.name);
  check("both definitions stored", scene.components.length === 2, `${scene.components.length} defs`);
  check("instance re-pointed at the copy", inst2.defId === copy.id, `defId ${inst2.defId}`);
  check("sibling instance keeps the original def", res.instance.defId === res.def.id, `defId ${res.instance.defId}`);
  let drift = Math.max(...scene.bodies.map((b, i) => dist(b.pos, posBefore[i])));
  check("fork moves nothing", drift === 0 && scene.bodies.length === 4, `drift ${drift}`);

  // Provenance maps stay valid against the identical copy: reconcile is a no-op.
  scene.reexpandInstances(new Set([copy.id]));
  drift = Math.max(...scene.bodies.map((b, i) => dist(b.pos, posBefore[i])));
  check("re-expansion after fork is a no-op", drift < 1e-9, `drift ${drift.toExponential(2)}`);

  // From here the two defs evolve independently, in both directions.
  editDef(scene, copy.id, (s) => { s.bodies[1].color = "#123456"; });
  const inst2Ids = new Set(inst2.bodyMap.map((e) => e.id));
  const recolored = scene.bodies.filter((b) => b.color === "#123456");
  check("editing the copy cascades to the forked instance only",
    recolored.length === 1 && inst2Ids.has(recolored[0].id), `${recolored.length} recolored`);
  editDef(scene, res.def.id, (s) => { s.bodies[1].color = "#654321"; });
  const recolored2 = scene.bodies.filter((b) => b.color === "#654321");
  check("editing the original no longer touches the fork",
    recolored2.length === 1 && !inst2Ids.has(recolored2[0].id), `${recolored2.length} recolored`);

  // A second fork from the same source name dedupes.
  const copy2 = scene.makeInstanceUnique(res.instance.id)!;
  check("fork dedupes the copy name", copy2.name === "Widget copy 2", copy2.name);

  // Forking an instance whose def nests another def shares the nested def (DAG node, no new edges).
  const s2 = new Scene();
  const ib = square(s2, 0, 0);
  s2.toggleBodyGround(ib.id);
  const inner = s2.createComponentFromSelection("Inner", [ib.id])!;
  const ob = square(s2, 100, 0);
  s2.toggleBodyGround(ob.id);
  const outer = s2.createComponentFromSelection("Outer", [ob.id])!;
  // put an Inner instance inside Outer's definition
  editDef(s2, outer.def.id, (s) => {
    s.instantiateComponent(inner.def.id, { pos: { x: 0, y: 80 }, angle: 0 });
  });
  const outerCopy = s2.makeInstanceUnique(outer.instance.id)!;
  check("nested fork copies the outer def only", s2.components.length === 3, `${s2.components.length} defs`);
  check("nested def stays shared", (outerCopy.data.instances ?? []).some((i) => i.defId === inner.def.id), "Inner referenced");
  check("fork appears in the def DAG", s2.componentUses(outerCopy.id).has(inner.def.id), "Outer copy uses Inner");
}

// --- persistence -------------------------------------------------------------------
{
  const scene = new Scene();
  const b1 = square(scene, 0, 0);
  scene.toggleBodyGround(b1.id);
  const b2 = square(scene, 60, 0);
  const res = scene.createComponentFromSelection("Saved", [b1.id, b2.id])!;
  scene.instantiateComponent(res.def.id, { pos: { x: 300, y: 50 }, angle: 1 });
  const data = JSON.parse(JSON.stringify(scene.serialize())) as SceneData;
  check("serialize writes components + instances", (data.components ?? []).length === 1 && (data.instances ?? []).length === 2, `${data.components?.length} defs, ${data.instances?.length} instances`);

  const loaded = new Scene();
  loaded.load(data);
  check("load restores components", loaded.components.length === 1 && loaded.components[0].name === "Saved", loaded.components[0]?.name ?? "-");
  check("load restores instances", loaded.instances.length === 2 && loaded.instances[0].bodyMap.length === 2, `${loaded.instances.length}`);
  // Re-expansion after a load is a no-op (idempotent reconcile).
  const posBefore = loaded.bodies.map((b) => ({ ...b.pos }));
  loaded.reexpandInstances(new Set([res.def.id]));
  const drift = Math.max(...loaded.bodies.map((b, i) => dist(b.pos, posBefore[i])));
  check("re-expansion after load is a no-op", drift < 1e-9, `drift ${drift.toExponential(2)}`);

  // reexpandData refreshes a stored context snapshot.
  const snap = JSON.parse(JSON.stringify(loaded.serializeContext())) as SceneData;
  loaded.components[0].data.bodies[0].color = "#0f0f0f";
  const refreshed = reexpandData(snap, loaded.components, new Set([res.def.id]));
  const recolored = refreshed.bodies.filter((b) => b.color === "#0f0f0f").length;
  check("reexpandData refreshes a stored snapshot", recolored === 2, `${recolored} recolored`);

  // A pre-v14 file (no components/instances keys) loads clean.
  const legacy = JSON.parse(JSON.stringify(data)) as SceneData;
  delete legacy.components;
  delete legacy.instances;
  const l2 = new Scene();
  l2.load(legacy);
  check("pre-v14 files load with no components", l2.components.length === 0 && l2.instances.length === 0, "clean");
}

if (failures > 0) {
  console.error(`${failures} FAILURE(S)`);
  process.exit(1);
}
console.log("All component checks passed");
