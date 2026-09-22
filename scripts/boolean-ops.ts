/** Subtract and Intersect — the selection booleans over bodies (`Scene.booleanBodies`): model-level checks. */
import { CombineResult, Scene } from "../src/model";
import { differenceRegions, intersectRegions } from "../src/boolean";
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
const why = (r: CombineResult): string => (r.ok ? "" : r.reason);
/** Area a rounded corner of radius r takes off a right-angled corner. */
const cornerLoss = (r: number) => r * r - (Math.PI * r * r) / 4;

// ---------------------------------------------------------------- intersection primitive
{
  const both = intersectRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(50, 50, 100, 100), holes: [] }]);
  check("intersect: overlapping squares → the 50×50 overlap", !!both && both.regions.length === 1 && both.regions[0].outer.length === 4 && near(Math.abs(polygonArea(both.regions[0].outer)), 2500), both ? `${both.regions[0].outer.length} verts, ${Math.abs(polygonArea(both.regions[0].outer))}` : "null");
  check("intersect: disjoint squares → null", intersectRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(200, 0, 100, 100), holes: [] }]) === null);
  check("intersect: a shared edge is no overlap", intersectRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(100, 0, 100, 100), holes: [] }]) === null);
  check("intersect: a corner touch is no overlap", intersectRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(100, 100, 100, 100), holes: [] }]) === null);
  const three = intersectRegions([{ outer: sq(0, 0, 100, 100), holes: [] }, { outer: sq(50, 0, 100, 100), holes: [] }, { outer: sq(0, 50, 100, 100), holes: [] }]);
  check("intersect: three squares → their common corner", !!three && three.regions.length === 1 && near(Math.abs(polygonArea(three.regions[0].outer)), 2500));
  const holed = intersectRegions([{ outer: sq(0, 0, 100, 100), holes: [sq(60, 60, 20, 20)] }, { outer: sq(50, 50, 100, 100), holes: [] }]);
  check("intersect: a hole inside the overlap survives", !!holed && holed.regions.length === 1 && holed.regions[0].holes.length === 1 && near(Math.abs(polygonArea(holed.regions[0].holes[0])), 400));
  // A U shape and a bar across its arms: two separate pieces.
  const u = [vec(0, 0), vec(100, 0), vec(100, 100), vec(70, 100), vec(70, 30), vec(30, 30), vec(30, 100), vec(0, 100)];
  const two = intersectRegions([{ outer: u, holes: [] }, { outer: sq(-10, 50, 120, 20), holes: [] }]);
  check("intersect: a bar across a U → two pieces", !!two && two.regions.length === 2 && !two.pinched, two ? `${two.regions.length} pieces` : "null");
  const cw = intersectRegions([{ outer: sq(0, 0, 100, 100).reverse(), holes: [] }, { outer: sq(50, 50, 100, 100), holes: [] }]);
  check("intersect: input orientation is irrelevant", !!cw && near(Math.abs(polygonArea(cw.regions[0].outer)), 2500));
  check("intersect: no inputs → null", intersectRegions([]) === null);
}

// ---------------------------------------------------------------- subtract: corner notch, joints, constraints, refs
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 100, 100), 10);
  const tool = s.addBody(sq(50, 50, 100, 100));
  const other = s.addBody(sq(300, 0, 20, 20));
  plate.color = "#abcdef";
  plate.grounded = true;
  const jp = s.addJoint(plate.id, vec(20, 20));
  const jpIn = s.addJoint(plate.id, vec(75, 75)); // inside the removed corner
  const jt = s.addJoint(tool.id, vec(120, 120));
  const jt2 = s.addJoint(tool.id, vec(75, 75));
  const jo = s.addJoint(other.id, vec(310, 10));
  const pinTool = s.addPin(jt.id, jo.id);
  const pinBoth = s.addPin(jpIn.id, jt2.id);
  const pinOther = s.addPin(jp.id, jo.id);
  const motorTool = s.addMotor(tool.id, jt.id, jt2.id)!;
  const mKept = s.addMeasurement("draw", { kind: "vertex", bodyId: plate.id, index: 1 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!; // (100,0) survives
  const mGone = s.addMeasurement("draw", { kind: "vertex", bodyId: plate.id, index: 2 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!; // (100,100) removed
  const mToolCorner = s.addMeasurement("draw", { kind: "vertex", bodyId: tool.id, index: 0 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!; // (50,50) becomes a plate corner
  const mToolFar = s.addMeasurement("draw", { kind: "vertex", bodyId: tool.id, index: 2 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!; // (150,150) goes with the tool
  const mEdge = s.addMeasurement("draw", { kind: "edge", bodyId: plate.id, index: 0 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!; // bottom edge, whole
  const mCutEdge = s.addMeasurement("draw", { kind: "edge", bodyId: plate.id, index: 1 }, { kind: "joint", jointId: jo.id }, vec(0, 0))!; // right edge, shortened
  const r = s.booleanBodies("subtract", [plate.id, tool.id]);
  check("subtract: corner overlap succeeds", r.ok, why(r));
  if (r.ok) {
    check("subtract: the first body survives as an L with 6 corners", r.body.id === plate.id && r.body.controlLocal.length === 6, `${r.body.controlLocal.length} verts`);
    check("subtract: area = plate minus the overlap, minus the three kept roundings", near(area(s, plate.id), 7500 - 3 * cornerLoss(10), 2), `${area(s, plate.id)}`);
    check("subtract: the tool is gone, the third body stays", !s.getBody(tool.id) && !!s.getBody(other.id) && s.bodies.length === 2);
    check("subtract: survivor keeps colour and grounding", r.body.color === "#abcdef" && r.body.grounded);
    check("subtract: untouched corners keep the default radius, new corners are sharp", r.body.radius === 10 && !!r.body.radii && r.body.radii.filter((x) => x === 0).length === 3 && r.body.radii.filter((x) => x === null).length === 3, JSON.stringify(r.body.radii));
    check("subtract: the subject's joints stay, even one inside the removed corner", s.getJoint(jp.id)!.bodyId === plate.id && s.getJoint(jpIn.id)!.bodyId === plate.id);
    check("subtract: the tool's joints go", !s.getJoint(jt.id) && !s.getJoint(jt2.id));
    check("subtract: constraints on the tool's joints go, the rest stay", !s.constraints.some((k) => k.id === pinTool.id || k.id === pinBoth.id || k.id === motorTool.id) && s.constraints.some((k) => k.id === pinOther.id));
    const kept = s.getMeasurement(mKept.id);
    const kp = kept && s.resolveMeasureRef(kept.refA);
    check("subtract: surviving corner ref stays put", !!kept && kept.refA.kind === "vertex" && kept.refA.bodyId === plate.id && !!kp && kp.kind === "point" && dist(kp.p, vec(100, 0)) < 1e-9);
    check("subtract: removed corner ref is pruned", !s.getMeasurement(mGone.id));
    const tc = s.getMeasurement(mToolCorner.id);
    const tp = tc && s.resolveMeasureRef(tc.refA);
    check("subtract: a tool corner that became a subject corner remaps onto the survivor", !!tc && tc.refA.kind === "vertex" && tc.refA.bodyId === plate.id && !!tp && tp.kind === "point" && dist(tp.p, vec(50, 50)) < 1e-9);
    check("subtract: a tool corner outside the subject is pruned", !s.getMeasurement(mToolFar.id));
    const e = s.getMeasurement(mEdge.id);
    check("subtract: a whole surviving edge ref stays", !!e && e.refA.kind === "edge" && e.refA.bodyId === plate.id);
    check("subtract: a shortened edge ref is pruned", !s.getMeasurement(mCutEdge.id));
  }
}

// ---------------------------------------------------------------- subtract: tools inside become holes with their own specs
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 100, 100));
  const disk = s.addBody([vec(30, 30)], 8, "offset");
  const rounded = s.addBody(sq(60, 60, 20, 20), 4);
  const hex = s.addBody([0, 1, 2, 3, 4, 5].map((i) => vec(30 + 10 * Math.cos((i * Math.PI) / 3), 75 + 10 * Math.sin((i * Math.PI) / 3))), 0, "fillet", undefined, undefined, 6);
  const before = area(s, plate.id);
  const toolArea = area(s, disk.id) + area(s, rounded.id) + area(s, hex.id);
  const r = s.booleanBodies("subtract", [plate.id, disk.id, rounded.id, hex.id]);
  check("subtract: three tools inside the plate succeed", r.ok, why(r));
  if (r.ok) {
    check("subtract: every tool is consumed", s.bodies.length === 1);
    const holes = r.body.holes ?? [];
    check("subtract: three holes, outline unchanged", holes.length === 3 && r.body.controlLocal.length === 4 && r.body.radius === 0, `${holes.length} holes, ${r.body.controlLocal.length} verts`);
    const dh = holes.findIndex((h) => h.round === "offset");
    check("subtract: the disk body became a true disk hole at its centre", dh >= 0 && holes[dh].controlLocal.length === 1 && holes[dh].radius === 8 && dist(s.bodyHoleControlWorld(r.body, dh)[0], vec(30, 30)) < 1e-9);
    const rh = holes.find((h) => h.round !== "offset" && h.controlLocal.length === 4);
    check("subtract: the rounded body became a rounded 4-corner hole", !!rh && rh.radius === 4 && !rh.radii && !rh.regular);
    const hh = holes.find((h) => h.controlLocal.length === 6);
    check("subtract: the regular hexagon became a regular hexagonal hole", !!hh && hh.regular === 6 && hh.radius === 0);
    check("subtract: area drops by the tools' areas", near(area(s, plate.id), before - toolArea, 1e-6), `${area(s, plate.id)} vs ${before - toolArea}`);
  }
}

// ---------------------------------------------------------------- subtract: a tool's hole is not subtracted (and leaves an island)
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 100, 100));
  const ring = s.addBody(sq(50, 50, 100, 100), 0, "fillet", [sq(60, 60, 20, 20)]);
  const r = s.booleanBodies("subtract", [plate.id, ring.id]);
  check("subtract: a ring tool would leave an island → refused as two pieces", !r.ok && /2 pieces/.test(why(r)), why(r));
  check("subtract: the refusal leaves both bodies", s.bodies.length === 2 && !s.getBody(plate.id)!.holes);
}

// ---------------------------------------------------------------- subtract: rejections leave the scene untouched
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 100, 100));
  const beside = s.addBody(sq(100, 0, 100, 100)); // shares an edge
  const apart = s.addBody(sq(300, 0, 10, 10));
  const cover = s.addBody(sq(-10, -10, 120, 120));
  const bar = s.addBody(sq(40, -10, 20, 120));
  const inside = s.addBody(sq(20, 20, 10, 10));
  const diamond = s.addBody([vec(50, 0), vec(70, 30), vec(50, 60), vec(30, 30)]); // touches the outline at one point from inside
  const edge = s.booleanBodies("subtract", [plate.id, beside.id]);
  check("subtract: a tool that only shares an edge is refused", !edge.ok && /doesn't overlap/.test(why(edge)) && /nothing to subtract/.test(why(edge)), why(edge));
  const far = s.booleanBodies("subtract", [plate.id, inside.id, apart.id]);
  check("subtract: one idle tool among several is named", !far.ok && /One of the other bodies/.test(why(far)), why(far));
  const far2 = s.booleanBodies("subtract", [plate.id, inside.id, apart.id, beside.id]);
  check("subtract: several idle tools are counted", !far2.ok && /2 of the other bodies/.test(why(far2)), why(far2));
  const all = s.booleanBodies("subtract", [plate.id, cover.id]);
  check("subtract: a tool covering the subject is refused", !all.ok && /whole body/.test(why(all)), why(all));
  const sever = s.booleanBodies("subtract", [plate.id, bar.id]);
  check("subtract: a tool severing the subject is refused, pointing at Split", !sever.ok && /2 pieces/.test(why(sever)) && /Split/.test(why(sever)), why(sever));
  const pinch = s.booleanBodies("subtract", [plate.id, diamond.id]);
  check("subtract: a tool touching the outline at a point from inside is refused", !pinch.ok && /single point/.test(why(pinch)), why(pinch));
  const one = s.booleanBodies("subtract", [plate.id]);
  check("subtract: a single body is refused", !one.ok);
  check("subtract: rejections leave the scene untouched", s.bodies.length === 7 && s.getBody(plate.id)!.controlLocal.length === 4 && !s.getBody(plate.id)!.holes);
}

// ---------------------------------------------------------------- subtract: groups, patterns, an offset bar
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 100, 100));
  const tool = s.addBody(sq(80, 80, 40, 40));
  const buddy = s.addBody(sq(300, 0, 20, 20));
  s.addGroup([plate.id, tool.id, buddy.id]);
  const r = s.booleanBodies("subtract", [plate.id, tool.id]);
  const g = s.groupOf(plate.id);
  check("subtract: the group loses the tool and keeps the rest", r.ok && !!g && g.bodyIds.length === 2 && g.bodyIds.includes(buddy.id) && !g.bodyIds.includes(tool.id), why(r));
  check("subtract: a corner notch → 6-corner outline, area 10000 − 400", r.ok && r.body.controlLocal.length === 6 && near(area(s, plate.id), 9600));
}
{
  const s = new Scene();
  const j1 = s.addFreeJoint(vec(0, 0));
  const j2 = s.addFreeJoint(vec(100, 0));
  const bar = s.buildBodyFromJoints([j1.id, j2.id], 10)!;
  const bite = s.addBody(sq(40, 5, 20, 20));
  const before = area(s, bar.id);
  const r = s.booleanBodies("subtract", [bar.id, bite.id]);
  check("subtract: an offset bar is baked, then notched, joints kept", r.ok && r.body.round === "fillet" && near(area(s, bar.id), before - 20 * 5, 1e-6) && s.getJoint(j1.id)!.bodyId === bar.id && s.getJoint(j2.id)!.bodyId === bar.id, r.ok ? `${area(s, bar.id)} vs ${before - 100}` : why(r));
}

// ---------------------------------------------------------------- intersect: overlap, radii, joints, constraints, refs
{
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100, 100), 10);
  const b = s.addBody(sq(50, 50, 100, 100));
  const c = s.addBody(sq(300, 0, 20, 20));
  a.color = "#123456";
  a.grounded = true;
  const ja = s.addJoint(a.id, vec(75, 75));
  const jaOut = s.addJoint(a.id, vec(20, 20)); // outside the overlap
  const jb = s.addJoint(b.id, vec(75, 75));
  const jc = s.addJoint(c.id, vec(310, 10));
  const pinAB = s.addPin(ja.id, jb.id);
  const pinAC = s.addPin(ja.id, jc.id);
  const pinBC = s.addPin(jb.id, jc.id);
  const mKept = s.addMeasurement("draw", { kind: "vertex", bodyId: a.id, index: 2 }, { kind: "joint", jointId: jc.id }, vec(0, 0))!; // (100,100) survives
  const mGone = s.addMeasurement("draw", { kind: "vertex", bodyId: a.id, index: 0 }, { kind: "joint", jointId: jc.id }, vec(0, 0))!; // (0,0) gone
  const mB = s.addMeasurement("draw", { kind: "vertex", bodyId: b.id, index: 0 }, { kind: "joint", jointId: jc.id }, vec(0, 0))!; // (50,50) becomes a's corner
  const r = s.booleanBodies("intersect", [a.id, b.id]);
  check("intersect: overlapping squares succeed", r.ok, why(r));
  if (r.ok) {
    check("intersect: survivor is the first id, a 50×50 square", r.body.id === a.id && r.body.controlLocal.length === 4 && near(area(s, a.id), 2500 - cornerLoss(10), 2), `${area(s, a.id)}`);
    check("intersect: the other body is gone, the third stays", !s.getBody(b.id) && !!s.getBody(c.id) && s.bodies.length === 2);
    check("intersect: colour and grounding kept", r.body.color === "#123456" && r.body.grounded);
    check("intersect: the surviving corner keeps its rounding, new corners are sharp", r.body.radius === 10 && !!r.body.radii && r.body.radii.filter((x) => x === 0).length === 3 && r.body.radii.filter((x) => x === null).length === 1, JSON.stringify(r.body.radii));
    check("intersect: the subject's joints stay", s.getJoint(ja.id)!.bodyId === a.id && s.getJoint(jaOut.id)!.bodyId === a.id);
    check("intersect: the other body's joints and their constraints go", !s.getJoint(jb.id) && !s.constraints.some((k) => k.id === pinAB.id || k.id === pinBC.id) && s.constraints.some((k) => k.id === pinAC.id));
    const kept = s.getMeasurement(mKept.id);
    const kp = kept && s.resolveMeasureRef(kept.refA);
    check("intersect: surviving corner ref stays", !!kept && !!kp && kp.kind === "point" && dist(kp.p, vec(100, 100)) < 1e-9);
    check("intersect: lost corner ref is pruned", !s.getMeasurement(mGone.id));
    const mb = s.getMeasurement(mB.id);
    const bp = mb && s.resolveMeasureRef(mb.refA);
    check("intersect: the other body's corner ref remaps onto the survivor", !!mb && mb.refA.kind === "vertex" && mb.refA.bodyId === a.id && !!bp && bp.kind === "point" && dist(bp.p, vec(50, 50)) < 1e-9);
  }
}

// ---------------------------------------------------------------- intersect: holes, three bodies, rejections
{
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100, 100), 0, "fillet", [{ control: [vec(70, 70)], radius: 5, round: "offset" }, sq(10, 10, 20, 20)]);
  const ring = s.addBody(sq(50, 50, 100, 100), 0, "fillet", [sq(55, 80, 10, 10)]);
  const r = s.booleanBodies("intersect", [a.id, ring.id]);
  check("intersect: holes inside the overlap survive, others go", r.ok && r.body.holes?.length === 2, r.ok ? `${r.body.holes?.length} holes` : why(r));
  check("intersect: the subject's disk hole stays a disk, the tool's hole keeps its shape", r.ok && !!r.body.holes && r.body.holes.some((h) => h.round === "offset" && h.radius === 5 && h.controlLocal.length === 1) && r.body.holes.some((h) => h.round !== "offset" && h.controlLocal.length === 4));
  check("intersect: the outline is the 50×50 overlap", r.ok && near(Math.abs(polygonArea(s.bodyWorldVerts(r.body))), 2500));
}
{
  const s = new Scene();
  const p = s.addBody(sq(0, 0, 100, 100));
  const q = s.addBody(sq(50, 0, 100, 100));
  const t = s.addBody(sq(0, 50, 100, 100));
  const r = s.booleanBodies("intersect", [p.id, q.id, t.id]);
  check("intersect: three bodies → their common corner, one body left", r.ok && s.bodies.length === 1 && near(area(s, p.id), 2500) && r.body.id === p.id, why(r));
}
{
  const s = new Scene();
  const x = s.addBody(sq(0, 0, 100, 100));
  const y = s.addBody(sq(200, 0, 100, 100));
  const z = s.addBody(sq(100, 0, 100, 100));
  const v = s.addBody(sq(-50, 0, 100, 100));
  const w = s.addBody(sq(50, 0, 100, 100));
  const u = s.addBody([vec(0, 200), vec(100, 200), vec(100, 300), vec(70, 300), vec(70, 230), vec(30, 230), vec(30, 300), vec(0, 300)]);
  const bar = s.addBody(sq(-10, 250, 120, 20));
  const apart = s.booleanBodies("intersect", [x.id, y.id]);
  check("intersect: disjoint bodies are refused", !apart.ok && /doesn't overlap/.test(why(apart)), why(apart));
  const edge = s.booleanBodies("intersect", [x.id, z.id]);
  check("intersect: a shared edge is refused", !edge.ok, why(edge));
  const none = s.booleanBodies("intersect", [x.id, v.id, w.id]);
  check("intersect: three bodies with no common region are refused", !none.ok && /in common/.test(why(none)), why(none));
  const two = s.booleanBodies("intersect", [u.id, bar.id]);
  check("intersect: a disconnected overlap is refused", !two.ok && /2 separate pieces/.test(why(two)), why(two));
  const one = s.booleanBodies("intersect", [x.id]);
  check("intersect: a single body is refused", !one.ok);
  check("intersect: rejections leave the scene untouched", s.bodies.length === 7 && s.getBody(x.id)!.controlLocal.length === 4);
}

// ---------------------------------------------------------------- near-degenerate: a disk vertex a hair outside a plate edge (from a user scene)
{
  // A slightly skewed plate and a 48-gon disk centred on its top-left corner: disk vertex 12
  // sits 1.55e-4 outside the almost vertical left edge, so a 1.55e-4 sliver edge appears where
  // the disk crosses the plate edge. Probed at the fixed sample offset (far larger than the
  // sliver) that edge read as a boundary spur and faked a pinch.
  const plate = [vec(1623.2796481, 438.7350485), vec(1743.2793832, 438.7350485), vec(1743.2791201, 548.8601948), vec(1623.279876, 548.8601948)];
  const c = vec(1623.279677053778, 548.8601948023737), r = 21.144526546779296;
  const disk = Array.from({ length: 48 }, (_, k) => vec(c.x + r * Math.cos(Math.PI + (k * Math.PI) / 24), c.y + r * Math.sin(Math.PI + (k * Math.PI) / 24)));
  const d = differenceRegions({ outer: plate, holes: [] }, [{ outer: disk, holes: [] }]);
  const plateArea = Math.abs(polygonArea(plate)), diskArea = Math.abs(polygonArea(disk));
  check("difference: a sliver where a disk crosses a near-vertical edge doesn't fake a pinch", !!d && !d.pinched && d.regions.length === 1 && near(Math.abs(polygonArea(d.regions[0].outer)), plateArea - diskArea / 4, 0.05), d ? `pinched=${d.pinched}, ${d.regions.length} region(s), area ${Math.abs(polygonArea(d.regions[0].outer)).toFixed(3)} vs ${(plateArea - diskArea / 4).toFixed(3)}` : "null");
  const s = new Scene();
  const p = s.addBody(plate);
  const t = s.addBody([c], r, "offset");
  const before = area(s, p.id);
  const rr = s.booleanBodies("subtract", [p.id, t.id]);
  check("subtract: a disk body snapped onto a plate corner takes a quarter-disk bite", rr.ok && rr.body.controlLocal.length > 4 && near(area(s, p.id), before - diskArea / 4, 1), rr.ok ? `${before.toFixed(1)} -> ${area(s, p.id).toFixed(1)} (expected ${(before - diskArea / 4).toFixed(1)})` : why(rr));
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
if (failures > 0) process.exit(1);
