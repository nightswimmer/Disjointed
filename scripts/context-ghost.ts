/**
 * Headless coverage for the context ghost (context.ts): the enclosing assembly mapped
 * into a component definition's own frame while that definition is edited. Verifies:
 *  - buildContextGhost maps the root's material by the inverse of the reference
 *    instance's placement (translation + rotation): an external body's corner and a
 *    free joint land where the definition frame sees them;
 *  - the reference instance's own bodies and joints are gone from the ghost, but an
 *    assembly-level joint placed ON an instance body survives as a free point at its
 *    def-frame position (exactly the definition's local spot it sits over);
 *  - a mirrored instance mirrors the world (y → −y in the def frame);
 *  - a sibling instance of the same definition stays in the ghost (as a body);
 *  - the ghost carries no guides, measurements or sketch constraints;
 *  - depth: 0 shows nothing, 1 only the immediate parent; a level with no known
 *    instance (via null) breaks the chain — it and everything outside it stay null;
 *  - nesting composes: with Outer placed in the root and Inner placed inside Outer, the
 *    root's material reaches Inner's frame through both inverse placements, and Outer's
 *    own other material through one;
 *  - applyInversePlacement is the exact inverse of the placement for plain material;
 *  - measureInfoFor measures a point pair between two scenes.
 */
import { Scene, SceneData, measureInfoFor } from "../src/model";
import { buildContextGhost, applyInversePlacement } from "../src/context";
import { Vec2, add, dist, rotate, sub, vec } from "../src/geometry";

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

/** def frame → context, as InstanceTransform defines it. */
function place(t: { pos: Vec2; angle: number; mirrored?: boolean }, p: Vec2): Vec2 {
  const r = t.mirrored ? vec(p.x, -p.y) : p;
  return add(t.pos, rotate(r, t.angle));
}
/** context → def frame. */
function unplace(t: { pos: Vec2; angle: number; mirrored?: boolean }, q: Vec2): Vec2 {
  const r = rotate(sub(q, t.pos), -t.angle);
  return t.mirrored ? vec(r.x, -r.y) : r;
}

const near = (a: Vec2, b: Vec2, tol = 1e-6) => dist(a, b) <= tol;
const fmt = (p: Vec2) => `(${p.x.toFixed(3)}, ${p.y.toFixed(3)})`;

// --- fixture: a frame body + a free joint in the root; a component placed at an angle ---
{
  const root = new Scene();
  const frame = square(root, 300, 100, 30); // external body
  const freeJ = root.addFreeJoint(vec(400, 250)); // external free joint
  const cBody = square(root, 0, 0, 20); // becomes the component (def frame = its centroid at origin)
  const made = root.createComponentFromSelection("Link", [cBody.id])!;
  const inst = made.instance;
  // Place the instance: rotate 30° about its centroid, then move to (120, 80).
  const ang = Math.PI / 6;
  rotateInstance(root, inst.id, vec(0, 0), ang);
  moveInstance(root, inst.id, vec(120, 80));
  const T = root.instancePlacement(inst.id)!;
  check("placement derived", near(T.pos, vec(120, 80)) && Math.abs(T.angle - ang) < 1e-9, `pos ${fmt(T.pos)} angle ${T.angle.toFixed(4)}`);
  // An assembly-level joint on the instance body, at def-local (10, 5) → world via T.
  const instBodyId = inst.bodyMap[0].id;
  const onInst = root.addJoint(instBodyId, place(T, vec(10, 5)));
  check("assembly joint sits on the instance body", onInst !== null && onInst.bodyId === instBodyId, `bodyId ${onInst?.bodyId}`);
  // A sibling instance of the same def, elsewhere.
  const sib = root.instantiateComponent(made.def.id, { pos: vec(-200, -50), angle: 0 })!;
  // Drawing aids in the root that must not reach the ghost.
  root.addGuide(vec(0, 0), vec(1, 1));
  root.addMeasurement("draw", { kind: "joint", jointId: freeJ.id }, { kind: "vertex", bodyId: frame.id, index: 0 }, vec(0, 0));

  const rootData: SceneData = JSON.parse(JSON.stringify(root.serializeContext()));
  const levels = buildContextGhost([{ data: rootData, via: inst.id }], root.components, Infinity);
  check("one ghost level for a one-deep path", levels.length === 1 && levels[0] !== null, `levels ${levels.map((l) => (l ? "scene" : "null")).join(",")}`);
  const g = levels[0]!;

  // External body corner → def frame.
  const frameCornerWorld = root.bodyControlWorld(frame)[0];
  const gFrame = g.getBody(frame.id)!;
  check("external body survives with its id", !!gFrame, `body ${frame.id}`);
  check(
    "external corner mapped by the inverse placement",
    near(g.bodyControlWorld(gFrame)[0], unplace(T, frameCornerWorld), 1e-6),
    `${fmt(g.bodyControlWorld(gFrame)[0])} vs ${fmt(unplace(T, frameCornerWorld))}`
  );
  // External free joint.
  const gFree = g.getJoint(freeJ.id)!;
  check("external free joint mapped", !!gFree && near(g.jointWorld(gFree), unplace(T, vec(400, 250))), gFree ? fmt(g.jointWorld(gFree)) : "gone");
  // The reference instance's material is gone…
  check("reference instance bodies stripped", !g.getBody(instBodyId), `body ${instBodyId} ${g.getBody(instBodyId) ? "present" : "gone"}`);
  check("reference instance record stripped", !g.instances.some((i) => i.id === inst.id), `${g.instances.length} instances left`);
  // …but the assembly-level joint on it stays, as a free point at its def-frame spot.
  const gOn = g.getJoint(onInst!.id);
  check("assembly joint on the instance kept as a free point", !!gOn && gOn.bodyId === null, gOn ? `bodyId ${gOn.bodyId}` : "gone");
  check("…at the definition-local position it covers", !!gOn && near(g.jointWorld(gOn), vec(10, 5)), gOn ? fmt(g.jointWorld(gOn)) : "gone");
  // Sibling instance stays (as a body), mapped.
  const sibBody = sib.bodyMap[0].id;
  const gSib = g.getBody(sibBody);
  check("sibling instance body kept", !!gSib && near(gSib.pos, unplace(T, root.getBody(sibBody)!.pos)), gSib ? fmt(gSib.pos) : "gone");
  // No drawing aids.
  check("ghost carries no guides / dims / sketch", g.guides.length === 0 && g.measurements.length === 0 && g.sketch.length === 0, `${g.guides.length} guides, ${g.measurements.length} dims, ${g.sketch.length} sketch`);
  // The source snapshot is untouched.
  check("source snapshot untouched", rootData.bodies.some((b) => b.id === instBodyId) && rootData.guides!.length === 1, `${rootData.bodies.length} bodies`);

  // Depth 0 → nothing; via null → nothing.
  check("depth 0 shows nothing", buildContextGhost([{ data: rootData, via: inst.id }], root.components, 0)[0] === null, "level null");
  check("unknown instance breaks the chain", buildContextGhost([{ data: rootData, via: null }], root.components, Infinity)[0] === null, "level null");

  // Cross-scene measurement: def-frame origin (live) to the ghost's kept joint.
  const info = measureInfoFor(-1, { kind: "point", p: vec(0, 0) }, g.resolveMeasureRef({ kind: "joint", jointId: onInst!.id })!, "direct", vec(5, 20));
  check("measureInfoFor across scenes", !!info && Math.abs(info.value - Math.hypot(10, 5)) < 1e-9, `value ${info?.value.toFixed(4)}`);

  // Mirrored reference instance: mirror the instance in the root and rebuild.
  root.mirrorBodies([instBodyId], [], "h");
  const Tm = root.instancePlacement(inst.id)!;
  check("mirrored placement", Tm.mirrored === true, `mirrored ${Tm.mirrored}`);
  const rootDataM: SceneData = JSON.parse(JSON.stringify(root.serializeContext()));
  const gm = buildContextGhost([{ data: rootDataM, via: inst.id }], root.components, Infinity)[0]!;
  const gmFree = gm.getJoint(freeJ.id)!;
  check("mirrored instance: world reflected into the def frame", near(gm.jointWorld(gmFree), unplace(Tm, vec(400, 250)), 1e-6), `${fmt(gm.jointWorld(gmFree))} vs ${fmt(unplace(Tm, vec(400, 250)))}`);
  const gmFrame = gm.getBody(frame.id)!;
  const cornersOk = root.bodyControlWorld(frame).every((w) => gm.bodyControlWorld(gmFrame).some((q) => near(q, unplace(Tm, w), 1e-6)));
  check("mirrored instance: external corners all present, reflected", cornersOk, "every corner matched");
}

// --- nesting: root → Outer → Inner --------------------------------------------------------
{
  const root = new Scene();
  const marker = root.addFreeJoint(vec(500, -100)); // root-level landmark
  // Inner def from a body at the origin (its throwaway root instance is dropped).
  const innerBody = square(root, 0, 0, 10);
  const inner = root.createComponentFromSelection("Inner", [innerBody.id])!;
  root.removeInstance(inner.instance.id);
  // Outer def built directly (the app does this by editing a def context): a plate plus
  // an instance of Inner placed at (0, 40) / 90° in Outer's frame.
  const outerId = root.components.reduce((m, c) => Math.max(m, c.id), 0) + 1;
  {
    const tmp = new Scene();
    tmp.components = root.components;
    square(tmp, 60, 0, 15);
    tmp.instantiateComponent(inner.def.id, { pos: vec(0, 40), angle: Math.PI / 2 });
    root.components.push({ id: outerId, name: "Outer", data: tmp.serializeContext() });
  }
  // Place Outer in the root at (200, 100) / 45°.
  const outerInst = root.instantiateComponent(outerId, { pos: vec(200, 100), angle: Math.PI / 4 })!;
  const Touter = root.instancePlacement(outerInst.id)!;
  check("Outer placed in the root", near(Touter.pos, vec(200, 100), 1e-6) && Math.abs(Touter.angle - Math.PI / 4) < 1e-9, `pos ${fmt(Touter.pos)} angle ${Touter.angle.toFixed(4)}`);
  const rootData: SceneData = JSON.parse(JSON.stringify(root.serializeContext()));
  // Enter Outer: its data is the context; find Inner's instance there.
  const outerDef = root.getComponent(outerId)!;
  const outerScene = new Scene();
  outerScene.loadContext(outerDef.data);
  outerScene.components = root.components;
  const innerInOuter = outerScene.instances.find((i) => i.defId === inner.def.id)!;
  const Tinner = outerScene.instancePlacement(innerInOuter.id)!;
  check("Inner placed inside Outer at (0,40) / 90°", near(Tinner.pos, vec(0, 40), 1e-6) && Math.abs(Tinner.angle - Math.PI / 2) < 1e-9, `pos ${fmt(Tinner.pos)} angle ${Tinner.angle.toFixed(4)}`);

  const sources = [
    { data: rootData, via: outerInst.id },
    { data: JSON.parse(JSON.stringify(outerDef.data)) as SceneData, via: innerInOuter.id },
  ];
  const levels = buildContextGhost(sources, root.components, Infinity);
  check("two ghost levels", levels.length === 2 && levels.every((l) => l !== null), levels.map((l) => (l ? "scene" : "null")).join(","));
  // Root landmark reaches Inner's frame through both inverse placements.
  const expectMarker = unplace(Tinner, unplace(Touter, vec(500, -100)));
  const gRootMarker = levels[0]!.getJoint(marker.id)!;
  check("root material composed through two placements", !!gRootMarker && near(levels[0]!.jointWorld(gRootMarker), expectMarker, 1e-6), gRootMarker ? `${fmt(levels[0]!.jointWorld(gRootMarker))} vs ${fmt(expectMarker)}` : "gone");
  // Outer's plate (def-frame centroid (60, 0)) reaches Inner's frame through Inner's inverse only.
  const plateInOuter = outerScene.bodies.find((b) => !outerScene.instanceOfBody(b.id))!;
  const gPlate = levels[1]!.getBody(plateInOuter.id)!;
  check("parent-level material through one placement", !!gPlate && near(gPlate.pos, unplace(Tinner, plateInOuter.pos), 1e-6), gPlate ? `${fmt(gPlate.pos)} vs ${fmt(unplace(Tinner, plateInOuter.pos))}` : "gone");
  check("parent level drops Inner's own body", !levels[1]!.getBody(innerInOuter.bodyMap[0].id), "inner body gone");
  // Depth 1: only the immediate parent.
  const d1 = buildContextGhost(sources, root.components, 1);
  check("depth 1 shows only the immediate parent", d1[0] === null && d1[1] !== null, d1.map((l) => (l ? "scene" : "null")).join(","));
  // A broken chain at the parent hides the root too.
  const broken = buildContextGhost([sources[0], { ...sources[1], via: null }], root.components, Infinity);
  check("broken placement chain hides everything outside it", broken.every((l) => l === null), broken.map((l) => (l ? "scene" : "null")).join(","));
}

// --- applyInversePlacement is the inverse of the placement on plain material ------------
{
  const s = new Scene();
  const b = square(s, 0, 0, 10);
  s.rotateBody(b.id, vec(0, 0), 0.3);
  const j = s.addFreeJoint(vec(50, 20));
  const t = { pos: vec(30, -10), angle: 0.7, mirrored: true };
  // Forward-place the material by hand (mirror about y = 0 as the def frame reflection).
  const cornersBefore = s.bodyControlWorld(b).map((p) => place(t, p));
  const jointBefore = place(t, s.jointWorld(j));
  // Put the scene at the placed pose: mirror y → -y about y=0 via mirrorBodies + shift, rotate, translate.
  s.mirrorBodies([b.id], [j.id], "v");
  const c = (s.getBody(b.id)!.pos.y + 0) / 2; // probe: body was at y=0 before → c = y_after/2
  s.moveBody(b.id, vec(0, -2 * c));
  s.moveJoint(j.id, vec(0, -2 * c));
  s.rotateBody(b.id, vec(0, 0), t.angle);
  const jw = s.jointWorld(s.getJoint(j.id)!);
  s.moveJoint(j.id, sub(rotate(jw, t.angle), jw));
  s.moveBody(b.id, t.pos);
  s.moveJoint(j.id, t.pos);
  const placedOk = s.bodyControlWorld(s.getBody(b.id)!).every((p) => cornersBefore.some((q) => near(p, q, 1e-6))) && near(s.jointWorld(s.getJoint(j.id)!), jointBefore, 1e-6);
  check("fixture placed forward", placedOk, "corners + joint match the analytic placement");
  applyInversePlacement(s, t);
  const back = s.bodyControlWorld(s.getBody(b.id)!);
  const orig = cornersBefore.map((q) => unplace(t, q));
  check("inverse placement restores the corners", back.every((p) => orig.some((q) => near(p, q, 1e-6))), `${back.map(fmt).join(" ")}`);
  check("inverse placement restores the joint", near(s.jointWorld(s.getJoint(j.id)!), vec(50, 20), 1e-6), fmt(s.jointWorld(s.getJoint(j.id)!)));
}

console.log(failures === 0 ? "\nALL CONTEXT-GHOST CHECKS PASSED" : `\n${failures} CONTEXT-GHOST CHECK(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
