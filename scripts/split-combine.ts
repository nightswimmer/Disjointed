/** Split (cut a body along a path) and Combine (polygon union of bodies): model-level checks. */
import { Scene } from "../src/model";
import { unionRegions } from "../src/boolean";
import { polygonArea, dist, vec, Vec2 } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const sq = (x: number, y: number, w: number, h: number): Vec2[] => [vec(x, y), vec(x + w, y), vec(x + w, y + h), vec(x, y + h)];
const area = (s: Scene, id: number): number => {
  const b = s.getBody(id)!;
  let a = Math.abs(polygonArea(s.bodyWorldVerts(b)));
  for (const h of s.bodyHolesWorld(b)) a -= Math.abs(polygonArea(h));
  return a;
};
const near = (a: number, b: number, tol = 1e-6) => Math.abs(a - b) <= tol;

// ---------------------------------------------------------------- union primitive
{
  const u = unionRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(50, 50, 100, 100), holes: [] }])!;
  check("union: overlapping squares → one 8-gon", u.regions.length === 1 && u.regions[0].outer.length === 8 && near(polygonArea(u.regions[0].outer), 17500), `${u.regions[0].outer.length} verts`);
  const shared = unionRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(100, 0, 100, 100), holes: [] }])!;
  check("union: shared edge → 4-vertex rectangle", shared.regions.length === 1 && shared.regions[0].outer.length === 4 && near(Math.abs(polygonArea(shared.regions[0].outer)), 20000));
  const touch = unionRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(100, 100, 100, 100), holes: [] }])!;
  check("union: corner touch is flagged pinched", touch.pinched && touch.regions.length === 2);
  const apart = unionRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(200, 0, 100, 100), holes: [] }])!;
  check("union: disjoint → two regions, not pinched", apart.regions.length === 2 && !apart.pinched);
  const c1 = [vec(0, 0), vec(100, 0), vec(100, 20), vec(20, 20), vec(20, 80), vec(100, 80), vec(100, 100), vec(0, 100)];
  const c2 = c1.map((p) => vec(100 - p.x, p.y));
  const ring = unionRegions([{ outer: c1, holes: [] }, { outer: c2, holes: [] }])!;
  check("union: two C shapes enclose a new hole", ring.regions.length === 1 && ring.regions[0].holes.length === 1 && near(Math.abs(polygonArea(ring.regions[0].holes[0])), 3600));
  const partial = unionRegions([{ outer: sq(0, 0, 100, 100), holes: [sq(40, 40, 20, 20)] }, { outer: sq(50, 30, 100, 40), holes: [] }])!;
  check("union: partially covered hole shrinks", partial.regions[0].holes.length === 1 && near(Math.abs(polygonArea(partial.regions[0].holes[0])), 200));
  const cw = unionRegions([{ outer: sq(0, 0, 100, 100).reverse(), holes: [] }, { outer: sq(50, 50, 100, 100), holes: [] }])!;
  check("union: input orientation is irrelevant", near(polygonArea(cw.regions[0].outer), 17500));
}

// ---------------------------------------------------------------- split: straight cut
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100));
  const other = s.addBody(sq(200, 0, 50, 50));
  body.color = "#123456";
  const jl = s.addJoint(body.id, vec(25, 50));
  const jr = s.addJoint(body.id, vec(75, 50));
  const jo = s.addJoint(other.id, vec(225, 25));
  const pin = s.addPin(jr.id, jo.id);
  // Refs: corner (100,0) → index 1; the bottom edge (0) will be cut; the right edge (1) survives.
  const mCorner = s.addMeasurement("draw", { kind: "vertex", bodyId: body.id, index: 1 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!;
  const mCutEdge = s.addMeasurement("draw", { kind: "edge", bodyId: body.id, index: 0 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!;
  const mRightEdge = s.addMeasurement("draw", { kind: "edge", bodyId: body.id, index: 1 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!;
  const mLeftEdge = s.addMeasurement("draw", { kind: "edge", bodyId: body.id, index: 3 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!;
  const mPoint = s.addMeasurement("draw", { kind: "bodyPoint", bodyId: body.id, local: vec(30, 0) }, { kind: "joint", jointId: jo.id }, vec(0, 0))!;
  const pointWorldBefore = s.resolveMeasureRef(mPoint.refA)!;
  const r = s.splitBody(body.id, [vec(50, 0), vec(50, 100)]);
  check("split: straight cut succeeds", r.ok, r.ok ? "" : r.reason);
  if (r.ok) {
    check("split: A keeps the original id", r.a.id === body.id);
    check("split: equal halves", near(area(s, r.a.id), 5000) && near(area(s, r.b.id), 5000), `${area(s, r.a.id)} / ${area(s, r.b.id)}`);
    check("split: B sits right after A in z-order", s.bodies.indexOf(r.b) === s.bodies.indexOf(r.a) + 1);
    check("split: B inherits the colour", r.b.color === "#123456");
    const [L, R] = r.a.pos.x < r.b.pos.x ? [r.a, r.b] : [r.b, r.a];
    check("split: left joint on the left half, right joint on the right half", s.getJoint(jl.id)!.bodyId === L.id && s.getJoint(jr.id)!.bodyId === R.id);
    check("split: moved joint keeps its world position", dist(s.jointWorld(s.getJoint(jr.id)!), vec(75, 50)) < 1e-9);
    check("split: pin to the other body survives", s.constraints.some((c) => c.id === pin.id));
    const corner = s.getMeasurement(mCorner.id);
    check("split: corner ref remaps to the right half", !!corner && corner.refA.kind === "vertex" && corner.refA.bodyId === R.id && dist(s.resolveMeasureRef(corner.refA)!.kind === "point" ? (s.resolveMeasureRef(corner.refA) as any).p : vec(0, 0), vec(100, 0)) < 1e-9);
    check("split: ref on the cut edge is pruned", !s.getMeasurement(mCutEdge.id));
    const right = s.getMeasurement(mRightEdge.id);
    const rr = right && s.resolveMeasureRef(right.refA);
    check("split: right edge ref remaps to the right half", !!right && right.refA.kind === "edge" && right.refA.bodyId === R.id && !!rr && rr.kind === "line" && ((dist(rr.a, vec(100, 0)) < 1e-9 && dist(rr.b, vec(100, 100)) < 1e-9)));
    const left = s.getMeasurement(mLeftEdge.id);
    check("split: left edge ref lands on the left half", !!left && left.refA.kind === "edge" && left.refA.bodyId === L.id);
    const pt = s.getMeasurement(mPoint.id);
    const pw = pt && s.resolveMeasureRef(pt.refA);
    check("split: bodyPoint ref keeps its world spot", !!pw && pw.kind === "point" && pointWorldBefore.kind === "point" && dist(pw.p, pointWorldBefore.p) < 1e-9 && pt!.refA.kind === "bodyPoint" && pt!.refA.bodyId === (pointWorldBefore.p.x > 50 ? R : L).id);
  }
}

// ---------------------------------------------------------------- split: rail / motor spanning the cut, group, grounded
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100));
  const buddy = s.addBody(sq(300, 0, 20, 20));
  body.grounded = true;
  s.addGroup([body.id, buddy.id]);
  const a = s.addJoint(body.id, vec(20, 50));
  const b = s.addJoint(body.id, vec(80, 50));
  const c = s.addJoint(body.id, vec(20, 80));
  const rail = s.addSlider(a.id, b.id);
  const railKept = s.addSlider(a.id, c.id);
  const motor = s.addMotor(body.id, a.id, b.id)!;
  const motorKept = s.addMotor(body.id, a.id, c.id)!;
  const r = s.splitBody(body.id, [vec(50, 0), vec(50, 100)]);
  check("split: rail across the cut is dropped", r.ok && !s.constraints.some((k) => k.id === rail.id));
  check("split: rail on one side survives", r.ok && s.constraints.some((k) => k.id === railKept.id));
  check("split: motor across the cut is dropped", r.ok && !s.constraints.some((k) => k.id === motor.id));
  check("split: motor on one side survives", r.ok && s.constraints.some((k) => k.id === motorKept.id));
  check("split: B joins the group", r.ok && s.groupOf(r.b.id) === s.groupOf(body.id) && !!s.groupOf(body.id));
  check("split: B inherits grounded", r.ok && r.b.grounded);
}

// ---------------------------------------------------------------- split: radii, corner start, polyline, holes, offset bake
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100), 10);
  const r = s.splitBody(body.id, [vec(50, 0), vec(50, 100)]);
  check("split: cut corners are sharp overrides, others keep the default", r.ok && r.a.radius === 10 && !!r.a.radii && r.a.radii.filter((x) => x === 0).length === 2 && r.a.radii.filter((x) => x === null).length === 2, r.ok ? JSON.stringify(r.a.radii) : "");
  check("split: B has the same default radius and 2 sharp corners", r.ok && r.b.radius === 10 && !!r.b.radii && r.b.radii.filter((x) => x === 0).length === 2);
}
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100));
  const r = s.splitBody(body.id, [vec(0, 0), vec(100, 100)]);
  check("split: corner-to-corner diagonal → two triangles", r.ok && r.a.controlLocal.length === 3 && r.b.controlLocal.length === 3 && near(area(s, r.a.id), 5000));
}
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100));
  const r = s.splitBody(body.id, [vec(30, 0), vec(50, 50), vec(70, 100)]);
  check("split: polyline cut gives 5 + 5 vertices", r.ok && r.a.controlLocal.length === 5 && r.b.controlLocal.length === 5 && near(area(s, r.a.id) + area(s, r.b.id), 10000));
}
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100), 0, "fillet", [sq(10, 10, 20, 20), { control: [vec(80, 50)], radius: 5, round: "offset" }]);
  const mHole = s.addMeasurement("draw", { kind: "vertex", bodyId: body.id, index: 0, hole: 1 }, { kind: "vertex", bodyId: body.id, index: 0, hole: 0 }, vec(0, 0))!;
  const bad = s.splitBody(body.id, [vec(20, 0), vec(20, 100)]);
  check("split: cut through a hole is rejected", !bad.ok, bad.ok ? "" : bad.reason);
  check("split: rejected cut leaves the scene untouched", s.bodies.length === 1 && s.getBody(body.id)!.holes!.length === 2);
  const r = s.splitBody(body.id, [vec(50, 0), vec(50, 100)]);
  check("split: holes go to their sides", r.ok && r.a.holes?.length === 1 && r.b.holes?.length === 1);
  const R = r.ok ? (r.a.pos.x < r.b.pos.x ? r.b : r.a) : null;
  const L = r.ok ? (r.a.pos.x < r.b.pos.x ? r.a : r.b) : null;
  check("split: disk hole stays an offset disk on the right half", !!R && R.holes![0].round === "offset" && R.holes![0].controlLocal.length === 1 && R.holes![0].radius === 5);
  const m = s.getMeasurement(mHole.id);
  check("split: hole refs remap across bodies", !!m && !!R && !!L && m.refA.kind === "vertex" && m.refA.bodyId === R.id && m.refA.hole === 0 && m.refB.kind === "vertex" && m.refB.bodyId === L.id && m.refB.hole === 0);
}
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100));
  const rejects = [
    s.splitBody(body.id, [vec(20, 0), vec(80, 0)]),
    s.splitBody(body.id, [vec(50, 0), vec(150, 50), vec(50, 100)]),
    s.splitBody(body.id, [vec(50, 0), vec(50, 0)]),
    s.splitBody(body.id, [vec(50, 50), vec(60, 60)]),
    s.splitBody(body.id, [vec(30, 0), vec(70, 60), vec(30, 60), vec(70, 0)]),
  ];
  check("split: along-edge / outside / same-point / interior-start / self-crossing cuts are rejected", rejects.every((x) => !x.ok), rejects.map((x) => (x.ok ? "OK?!" : x.reason.slice(0, 24))).join(" | "));
  check("split: rejections leave one body", s.bodies.length === 1);
}
{
  const s = new Scene();
  const j1 = s.addFreeJoint(vec(0, 0));
  const j2 = s.addFreeJoint(vec(100, 0));
  const bar = s.buildBodyFromJoints([j1.id, j2.id], 10)!;
  const before = area(s, bar.id);
  const r = s.splitBody(bar.id, [vec(50, -10), vec(50, 10)]);
  check("split: offset bar is baked to a sharp fillet outline and split", r.ok && r.a.round === "fillet" && r.b.round === "fillet" && r.a.radius === 0 && near(area(s, r.a.id) + area(s, r.b.id), before, 1e-6));
  check("split: baked halves keep the joints at their ends", r.ok && s.getJoint(j1.id)!.bodyId !== s.getJoint(j2.id)!.bodyId && [r.a.id, r.b.id].includes(s.getJoint(j1.id)!.bodyId!));
}
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100));
  const on = s.addJoint(body.id, vec(50, 50));
  const r = s.splitBody(body.id, [vec(50, 0), vec(50, 100)]);
  check("split: a joint on the cut line stays with A", r.ok && s.getJoint(on.id)!.bodyId === r.a.id);
}

// ---------------------------------------------------------------- combine: overlap, joints, pins, refs
{
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100, 100));
  const b = s.addBody(sq(50, 50, 100, 100));
  const c = s.addBody(sq(300, 0, 20, 20));
  a.color = "#abcdef";
  const ja = s.addJoint(a.id, vec(20, 20));
  const jb = s.addJoint(b.id, vec(120, 120));
  const jb2 = s.addJoint(b.id, vec(75, 75));
  const ja2 = s.addJoint(a.id, vec(75, 75));
  const jc = s.addJoint(c.id, vec(310, 10));
  const internal = s.addPin(ja2.id, jb2.id);
  const external = s.addPin(jb.id, jc.id);
  const motor = s.addMotor(b.id, jb.id, jb2.id)!;
  const mFar = s.addMeasurement("draw", { kind: "vertex", bodyId: b.id, index: 2 }, { kind: "joint", jointId: jc.id }, vec(0, 0))!; // (150,150)
  const mGone = s.addMeasurement("draw", { kind: "vertex", bodyId: b.id, index: 0 }, { kind: "joint", jointId: jc.id }, vec(0, 0))!; // (50,50) — swallowed
  const mEdge = s.addMeasurement("draw", { kind: "edge", bodyId: b.id, index: 1 }, { kind: "joint", jointId: jc.id }, vec(0, 0))!; // (150,50)-(150,150) survives whole
  const mCutEdge = s.addMeasurement("draw", { kind: "edge", bodyId: b.id, index: 0 }, { kind: "joint", jointId: jc.id }, vec(0, 0))!; // (50,50)-(150,50) — shortened
  const mPoint = s.addMeasurement("draw", { kind: "bodyPoint", bodyId: b.id, local: vec(10, 10) }, { kind: "joint", jointId: jc.id }, vec(0, 0))!;
  const pw = (s.resolveMeasureRef(mPoint.refA) as { p: Vec2 }).p;
  const r = s.combineBodies([a.id, b.id]);
  check("combine: overlapping squares succeed", r.ok, r.ok ? "" : r.reason);
  if (r.ok) {
    check("combine: survivor is the first id with the union area", r.body.id === a.id && near(area(s, a.id), 17500) && r.body.controlLocal.length === 8, `${area(s, a.id)}`);
    check("combine: absorbed body is gone, others stay", !s.getBody(b.id) && !!s.getBody(c.id) && s.bodies.length === 2);
    check("combine: survivor keeps its colour", r.body.color === "#abcdef");
    check("combine: absorbed joints re-attach at the same world spots", s.getJoint(jb.id)!.bodyId === a.id && dist(s.jointWorld(s.getJoint(jb.id)!), vec(120, 120)) < 1e-9 && s.getJoint(ja.id)!.bodyId === a.id);
    check("combine: pin between the combined bodies is removed", !s.constraints.some((k) => k.id === internal.id));
    check("combine: pin to a third body survives", s.constraints.some((k) => k.id === external.id));
    const m = s.constraints.find((k) => k.id === motor.id);
    check("combine: motor follows onto the survivor", !!m && m.kind === "motor" && m.bodyId === a.id);
    const far = s.getMeasurement(mFar.id);
    const fp = far && s.resolveMeasureRef(far.refA);
    check("combine: surviving corner ref remaps", !!far && far.refA.kind === "vertex" && far.refA.bodyId === a.id && !!fp && fp.kind === "point" && dist(fp.p, vec(150, 150)) < 1e-9);
    check("combine: swallowed corner ref is pruned", !s.getMeasurement(mGone.id));
    const e = s.getMeasurement(mEdge.id);
    const el = e && s.resolveMeasureRef(e.refA);
    check("combine: whole surviving edge ref remaps", !!e && e.refA.kind === "edge" && e.refA.bodyId === a.id && !!el && el.kind === "line" && Math.min(dist(el.a, vec(150, 50)), dist(el.a, vec(150, 150))) < 1e-9);
    check("combine: shortened edge ref is pruned", !s.getMeasurement(mCutEdge.id));
    const p = s.getMeasurement(mPoint.id);
    const pp = p && s.resolveMeasureRef(p.refA);
    check("combine: bodyPoint ref moves over, world spot kept", !!p && p.refA.kind === "bodyPoint" && p.refA.bodyId === a.id && !!pp && pp.kind === "point" && dist(pp.p, pw) < 1e-9);
  }
}

// ---------------------------------------------------------------- combine: rejections
{
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100, 100));
  const b = s.addBody(sq(200, 0, 100, 100));
  const c = s.addBody(sq(100, 100, 50, 50));
  const apart = s.combineBodies([a.id, b.id]);
  check("combine: disjoint bodies are refused", !apart.ok && /touch the rest/.test(apart.ok ? "" : apart.reason), apart.ok ? "" : apart.reason);
  const pinch = s.combineBodies([a.id, c.id]);
  check("combine: corner-touching bodies are refused (pinch)", !pinch.ok && /point/.test(pinch.ok ? "" : pinch.reason), pinch.ok ? "" : pinch.reason);
  check("combine: rejections leave the scene untouched", s.bodies.length === 3 && s.getBody(a.id)!.controlLocal.length === 4);
  const one = s.combineBodies([a.id]);
  check("combine: a single body is refused", !one.ok);
}

// ---------------------------------------------------------------- combine: shared edge, radii, holes, groups, grounded, three bodies
{
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100, 100), 10);
  const b = s.addBody(sq(100, 0, 100, 100));
  const r = s.combineBodies([a.id, b.id]);
  check("combine: shared edge → a 4-corner rectangle", r.ok && r.body.controlLocal.length === 4 && near(area(s, a.id), 20000 - 2 * (100 - Math.PI * 25), 2), r.ok ? `${r.body.controlLocal.length} verts` : r.reason);
  check("combine: rounded corners keep the default, sharp ones get a 0 override", r.ok && r.body.radius === 10 && !!r.body.radii && r.body.radii.filter((x) => x === 0).length === 2 && r.body.radii.filter((x) => x === null).length === 2, r.ok ? JSON.stringify(r.body.radii) : "");
}
{
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100, 100), 0, "fillet", [{ control: [vec(30, 30)], radius: 8, round: "offset" }, sq(60, 10, 20, 20)]);
  const b = s.addBody(sq(90, 0, 100, 100), 0, "fillet", [sq(92, 40, 6, 20)]);
  const mDisk = s.addMeasurement("draw", { kind: "vertex", bodyId: a.id, index: 0, hole: 0 }, { kind: "vertex", bodyId: a.id, index: 0 }, vec(0, 0))!;
  const r = s.combineBodies([a.id, b.id]);
  check("combine: untouched disk hole keeps its editable spec", r.ok && !!r.body.holes && r.body.holes.some((h) => h.round === "offset" && h.controlLocal.length === 1 && h.radius === 8), r.ok ? `${r.body.holes?.length} holes` : r.reason);
  check("combine: hole overlapped by other material is swallowed, the rest stays", r.ok && r.body.holes!.length === 2, r.ok ? JSON.stringify(r.body.holes!.map((h) => h.controlLocal.length)) : "");
  check("combine: total area = union minus surviving holes", r.ok && near(area(s, a.id), 190 * 100 - Math.abs(polygonArea(s.bodyHolesWorld(r.body)[0])) - Math.abs(polygonArea(s.bodyHolesWorld(r.body)[1])), 1e-6));
  const m = s.getMeasurement(mDisk.id);
  check("combine: ref on the kept disk hole remaps", !!m && m.refA.kind === "vertex" && m.refA.bodyId === a.id && m.refA.hole !== undefined && r.ok && r.body.holes![m.refA.hole].round === "offset");
}
{
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100, 100));
  const b = s.addBody(sq(50, 0, 100, 100));
  const c = s.addBody(sq(300, 0, 20, 20));
  const d = s.addBody(sq(400, 0, 20, 20));
  b.grounded = true;
  s.addGroup([a.id, c.id]);
  s.addGroup([b.id, d.id]);
  const r = s.combineBodies([a.id, b.id]);
  const g = s.groupOf(a.id);
  check("combine: touched groups merge around the survivor", r.ok && !!g && g.bodyIds.length === 3 && g.bodyIds.includes(c.id) && g.bodyIds.includes(d.id) && s.groups.length === 1);
  check("combine: grounded input grounds the result", r.ok && r.body.grounded);
}
{
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100, 100));
  const b = s.addBody(sq(80, 0, 100, 100));
  const c = s.addBody(sq(160, 0, 100, 100));
  const r = s.combineBodies([a.id, b.id, c.id]);
  check("combine: three chained bodies become one", r.ok && s.bodies.length === 1 && near(area(s, a.id), 26000));
  const s2 = new Scene();
  const a2 = s2.addBody(sq(0, 0, 100, 100));
  const b2 = s2.addBody(sq(80, 0, 100, 100));
  const c2 = s2.addBody(sq(400, 0, 100, 100));
  const r2 = s2.combineBodies([a2.id, b2.id, c2.id]);
  check("combine: one stray body is reported", !r2.ok && /One body/.test(r2.ok ? "" : r2.reason), r2.ok ? "" : r2.reason);
}
{
  const s = new Scene();
  const j1 = s.addFreeJoint(vec(0, 0));
  const j2 = s.addFreeJoint(vec(100, 0));
  const bar = s.buildBodyFromJoints([j1.id, j2.id], 10)!;
  const plate = s.addBody(sq(90, -50, 100, 100));
  const barArea = area(s, bar.id);
  const r = s.combineBodies([bar.id, plate.id]);
  check("combine: offset bar bakes and merges with a plate", r.ok && r.body.round === "fillet" && area(s, bar.id) > barArea && area(s, bar.id) < barArea + 10000 && s.getJoint(j2.id)!.bodyId === bar.id);
}
{
  // Split then combine round-trips to the same area.
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100), 5, "fillet", [sq(10, 10, 20, 20)]);
  const before = area(s, body.id);
  const r = s.splitBody(body.id, [vec(60, 0), vec(60, 100)]);
  const back = r.ok ? s.combineBodies([r.a.id, r.b.id]) : r;
  check("split then combine restores the shape", back.ok && s.bodies.length === 1 && near(area(s, body.id), before, 1e-6) && s.getBody(body.id)!.controlLocal.length === 4 && s.getBody(body.id)!.holes!.length === 1, back.ok ? `${area(s, body.id)} vs ${before}` : back.reason);
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
if (failures > 0) process.exit(1);
