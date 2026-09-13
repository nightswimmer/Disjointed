/** Reference lines (two-point reference polylines): CRUD, moves, hit-tests, serialize/load
 *  round-trip, and sketch constraints between guides and geometry. */
import { Scene, MeasureRef } from "../src/model";
import { dist, sub, add, normalize, cross } from "../src/geometry";
import { tryAddConstraint, solveSketch, anchorVarsForBody, anchorVarForGuidePoint } from "../src/sketch";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

const s = new Scene();

// Creation: a valid pair defines a guide; a (near-)coincident pair is rejected.
const g1 = s.addGuidePoly([{ x: 0, y: 0 }, { x: 100, y: 50 }], false);
check("addGuidePoly creates a segment guide", g1 !== null && s.guides.length === 1);
check("degenerate guide rejected", s.addGuidePoly([{ x: 5, y: 5 }, { x: 5, y: 5 }], false) === null);
check("degenerate guide not stored", s.guides.length === 1);

// Whole-line move: both points translate, direction (angle) is preserved.
const dirBefore = normalize(sub(g1!.pts[1], g1!.pts[0]));
s.moveGuide(g1!.id, { x: 30, y: -20 });
const dirAfter = normalize(sub(g1!.pts[1], g1!.pts[0]));
check("moveGuide translates a", dist(g1!.pts[0], { x: 30, y: -20 }) < 1e-9, `a=(${g1!.pts[0].x},${g1!.pts[0].y})`);
check("moveGuide translates b", dist(g1!.pts[1], { x: 130, y: 30 }) < 1e-9, `b=(${g1!.pts[1].x},${g1!.pts[1].y})`);
check("moveGuide preserves angle", Math.abs(cross(dirBefore, dirAfter)) < 1e-12);

// Point move reshapes the segment; collapsing onto the neighbouring point is ignored.
s.moveGuidePoint(g1!.id, "1", { x: 30, y: 100 });
check("moveGuidePoint moves b", dist(g1!.pts[1], { x: 30, y: 100 }) < 1e-9);
s.moveGuidePoint(g1!.id, "0", { x: 30, y: 100 });
check("collapse onto other point ignored", dist(g1!.pts[0], { x: 30, y: -20 }) < 1e-9);

// Hit tests: g1 is now the vertical segment x = 30, y ∈ [-20, 100]. The segment is hit
// along its span only (a reference line is finite); the point pick finds the nearest point.
check("guideAt hits the segment", s.guideAt({ x: 32, y: 50 }, 5)?.id === g1!.id);
check("guideAt misses beyond the ends", s.guideAt({ x: 32, y: 5000 }, 5) === undefined);
check("guideAt misses off-line", s.guideAt({ x: 60, y: 0 }, 5) === undefined);
const gp = s.guidePointAt({ x: 28, y: 102 }, 5);
check("guidePointAt finds point 1", gp?.guide.id === g1!.id && gp?.which === "1");
check("guidePointAt misses far away", s.guidePointAt({ x: 30, y: 40 }, 5) === null);

// Serialize → JSON → load round-trip.
s.addGuidePoly([{ x: -50, y: 10 }, { x: 20, y: 10 }], false);
const text = JSON.stringify(s.serialize());
const t = new Scene();
t.load(JSON.parse(text));
check("guides survive round-trip", t.guides.length === 2, `${t.guides.length}`);
const l1 = t.getGuide(g1!.id);
check(
  "guide points preserved",
  l1?.kind === "poly" && dist(l1.pts[0], g1!.pts[0]) < 1e-9 && dist(l1.pts[1], g1!.pts[1]) < 1e-9
);
t.guides[0].pts[0].x += 999;
check("load is a deep copy", s.guides[0].pts[0].x !== t.guides[0].pts[0].x);

// Pre-v11 files simply have no guides.
const legacy = new Scene();
const noGuides = JSON.parse(text);
delete noGuides.guides;
legacy.load(noGuides);
check("pre-v11 file loads with no guides", legacy.guides.length === 0);

// New ids continue past loaded guide ids.
const maxId = Math.max(...t.guides.map((g) => g.id));
const fresh = t.addGuidePoly([{ x: 0, y: 0 }, { x: 1, y: 1 }], false)!;
check("nextId continues past guide ids", fresh.id > maxId, `new id ${fresh.id} > ${maxId}`);

// Removal + clear.
t.removeGuide(fresh.id);
check("removeGuide removes", t.getGuide(fresh.id) === undefined && t.guides.length === 2);
t.clear();
check("clear drops guides", t.guides.length === 0);

// --- constraints between guides and geometry --------------------------------

const c = new Scene();
const cg = c.addGuidePoly([{ x: 0, y: 0 }, { x: 100, y: 30 }], false)!;
const glRef: MeasureRef = { kind: "guideLine", guideId: cg.id, edge: 0 };
const gaRef: MeasureRef = { kind: "guidePoint", guideId: cg.id, which: "0" };

// Refs resolve to the guide's current geometry.
const rp = c.resolveMeasureRef(gaRef);
check("guidePoint ref resolves", rp?.kind === "point" && dist(rp.p, cg.pts[0]) < 1e-9);
const rl = c.resolveMeasureRef(glRef);
check("guideLine ref resolves", rl?.kind === "line" && dist(rl.a, cg.pts[0]) < 1e-9 && dist(rl.b, cg.pts[1]) < 1e-9);

// Horizontal on the guide: the sketch solver levels the line.
const h = tryAddConstraint(c, "horizontal", glRef);
check("H constraint accepted on a guide", h.constraint !== null);
check("guide solved horizontal", Math.abs(cg.pts[0].y - cg.pts[1].y) < 1e-2, `dy=${cg.pts[0].y - cg.pts[1].y}`);

// Coincident between a guide point and a free joint pulls them together.
const fj = c.addFreeJoint({ x: 40, y: 80 });
const co = tryAddConstraint(c, "coincident", gaRef, { kind: "joint", jointId: fj.id });
check("coincident guidePoint–joint accepted", co.constraint !== null);
check(
  "guide point and joint meet",
  dist(c.getGuide(cg.id)!.pts[0], c.jointWorld(c.getJoint(fj.id)!)) < 1e-2
);
check("guide stayed horizontal through it", Math.abs(cg.pts[0].y - cg.pts[1].y) < 1e-2);

// Dragging the guide re-solves: after a raw move, solveSketch restores coincidence.
c.moveGuide(cg.id, { x: 15, y: -25 });
const dragBreaks = solveSketch(c);
check("re-solve after a guide move succeeds", dragBreaks.length === 0);
check(
  "coincidence holds after the move",
  dist(c.getGuide(cg.id)!.pts[0], c.jointWorld(c.getJoint(fj.id)!)) < 1e-2
);

// Equal length works on a reference segment (it has a real length, unlike the retired
// infinite guideline): the segment and the edge end up the same length.
const body = c.addBody([{ x: 200, y: 200 }, { x: 260, y: 200 }, { x: 260, y: 260 }, { x: 200, y: 260 }]);
const eq = tryAddConstraint(c, "equal", glRef, { kind: "edge", bodyId: body.id, index: 0 });
check("equal with a reference segment accepted", eq.constraint !== null);
const edgeLen = () => { const v = c.bodyControlWorld(body); return dist(v[0], v[1]); };
check("segment and edge share a length", Math.abs(dist(cg.pts[0], cg.pts[1]) - edgeLen()) < 1e-2, `seg ${dist(cg.pts[0], cg.pts[1])} edge ${edgeLen()}`);

// Parallel between a guide and a body edge is accepted and solved.
const par = tryAddConstraint(c, "parallel", glRef, { kind: "edge", bodyId: body.id, index: 0 });
check("parallel guide–edge accepted", par.constraint !== null);

// Constraints referencing a removed guide cascade away.
const sketchBefore = c.sketch.length;
c.removeGuide(cg.id);
check(
  "removeGuide prunes its constraints",
  c.sketch.length < sketchBefore && c.sketch.every(
    (sc) => ![sc.refA, sc.refB].some((r) => r && (r.kind === "guideLine" || r.kind === "guidePoint"))
  ),
  `${sketchBefore} -> ${c.sketch.length}`
);

// Measurements can reference guides too — and prune with them.
const mScene = new Scene();
const mg = mScene.addGuidePoly([{ x: 0, y: 0 }, { x: 0, y: 100 }], false)!; // vertical line x=0
const mj = mScene.addFreeJoint({ x: 70, y: 700 }); // far beyond the segment's span
const m = mScene.addMeasurement(
  "draw",
  { kind: "joint", jointId: mj.id },
  { kind: "guideLine", guideId: mg.id, edge: 0 },
  { x: 35, y: 700 }
);
check("point–guide measurement created", m !== null);
const info = m ? mScene.measureInfo(m) : null;
check(
  "measures perpendicular distance to the segment's carrier line",
  info !== null && Math.abs(info.value - 70) < 1e-9,
  info ? `value ${info.value}` : "no info"
);
mScene.removeGuide(mg.id);
check("measurement pruned with its guide", mScene.measurements.length === 0);

// --- drag anchoring: a rigid move is never tugged back by guide constraints --------
// Regression: two bodies moved together (a group drag), with a guide coincident to two
// of body A's joints. The anchored live solve must leave the bodies' motion exactly
// rigid and make the guide follow in full.
const dS = new Scene();
const bodyA = dS.addBody([{ x: 0, y: 0 }, { x: 60, y: 0 }, { x: 60, y: 40 }, { x: 0, y: 40 }]);
const bodyB = dS.addBody([{ x: 200, y: 0 }, { x: 260, y: 0 }, { x: 260, y: 40 }, { x: 200, y: 40 }]);
const ja = dS.addJoint(bodyA.id, { x: 10, y: 20 });
const jb = dS.addJoint(bodyA.id, { x: 50, y: 20 });
const dg = dS.addGuidePoly([dS.jointWorld(ja), dS.jointWorld(jb)], false)!;
tryAddConstraint(dS, "coincident", { kind: "guidePoint", guideId: dg.id, which: "0" }, { kind: "joint", jointId: ja.id });
tryAddConstraint(dS, "coincident", { kind: "guidePoint", guideId: dg.id, which: "1" }, { kind: "joint", jointId: jb.id });
check("both drag-test coincidents in place", dS.sketch.length === 2);
const delta = { x: 37, y: -12 };
const posA0 = { x: bodyA.pos.x, y: bodyA.pos.y };
const posB0 = { x: bodyB.pos.x, y: bodyB.pos.y };
const jaw0 = dS.jointWorld(ja);
// One multi-drag step: rigid translation of both bodies, then the anchored live solve.
dS.moveBody(bodyA.id, delta);
dS.moveBody(bodyB.id, delta);
const anchors = new Set([
  ...anchorVarsForBody(dS, bodyA.id),
  ...anchorVarsForBody(dS, bodyB.id),
]);
const liveBreaks = solveSketch(dS, anchors);
check("anchored live solve converges", liveBreaks.length === 0);
check("body A moved exactly (not tugged)", dist(bodyA.pos, add(posA0, delta)) < 1e-9, `off ${dist(bodyA.pos, add(posA0, delta))}`);
check("body B moved exactly", dist(bodyB.pos, add(posB0, delta)) < 1e-9);
check("A's joint moved exactly with it", dist(dS.jointWorld(ja), add(jaw0, delta)) < 1e-9);
check(
  "guide followed the joints in full",
  dist(dS.getGuide(dg.id)!.pts[0], dS.jointWorld(ja)) < 1e-6 &&
    dist(dS.getGuide(dg.id)!.pts[1], dS.jointWorld(jb)) < 1e-6
);

// --- guide constraints move only free guide points, never geometry -----------------
// Scenario from the field: one body with two joints; guide 1 through both joints,
// guide 2 through one of them + a free point. Perpendicular between the guides must
// be satisfied by moving ONLY guide 2's free point — no joint, no body node, and no
// re-aim of the fully-bound guide 1.
const pS = new Scene();
const pBody = pS.addBody([{ x: 0, y: 0 }, { x: 80, y: 0 }, { x: 80, y: 50 }, { x: 0, y: 50 }]);
const pj1 = pS.addJoint(pBody.id, { x: 15, y: 20 });
const pj2 = pS.addJoint(pBody.id, { x: 65, y: 35 }); // different y: guide 1 is NOT horizontal
const pg1 = pS.addGuidePoly([pS.jointWorld(pj1), pS.jointWorld(pj2)], false)!;
tryAddConstraint(pS, "coincident", { kind: "guidePoint", guideId: pg1.id, which: "0" }, { kind: "joint", jointId: pj1.id });
tryAddConstraint(pS, "coincident", { kind: "guidePoint", guideId: pg1.id, which: "1" }, { kind: "joint", jointId: pj2.id });
const pg2 = pS.addGuidePoly([pS.jointWorld(pj1), { x: 60, y: 90 }], false)!; // one joint + a free point
tryAddConstraint(pS, "coincident", { kind: "guidePoint", guideId: pg2.id, which: "0" }, { kind: "joint", jointId: pj1.id });
const pPos0 = { x: pBody.pos.x, y: pBody.pos.y };
const pj1w0 = pS.jointWorld(pj1);
const pj2w0 = pS.jointWorld(pj2);
const perp = tryAddConstraint(
  pS, "perpendicular",
  { kind: "guideLine", guideId: pg1.id, edge: 0 },
  { kind: "guideLine", guideId: pg2.id, edge: 0 }
);
check("perpendicular guide–guide accepted", perp.constraint !== null);
check("body untouched by guide constraint", dist(pBody.pos, pPos0) < 1e-9);
check("joint 1 untouched", dist(pS.jointWorld(pj1), pj1w0) < 1e-9);
check("joint 2 untouched", dist(pS.jointWorld(pj2), pj2w0) < 1e-9);
const d1 = normalize(sub(pS.getGuide(pg1.id)!.pts[1], pS.getGuide(pg1.id)!.pts[0]));
const d2 = normalize(sub(pS.getGuide(pg2.id)!.pts[1], pS.getGuide(pg2.id)!.pts[0]));
check("guides ended perpendicular", Math.abs(d1.x * d2.x + d1.y * d2.y) < 1e-3, `dot ${d1.x * d2.x + d1.y * d2.y}`);
check("bound guide 1 stayed on its joints",
  dist(pS.getGuide(pg1.id)!.pts[0], pj1w0) < 2e-3 && dist(pS.getGuide(pg1.id)!.pts[1], pj2w0) < 2e-3);
check("guide 2 stayed on its joint", dist(pS.getGuide(pg2.id)!.pts[0], pj1w0) < 2e-3);

// H on guide 1 (both points bound to joints) is unsatisfiable under the rule that
// geometry never moves for a guide constraint → rejected, scene untouched.
const hFail = tryAddConstraint(pS, "horizontal", { kind: "guideLine", guideId: pg1.id, edge: 0 });
check("H on a fully joint-bound guide is rejected", hFail.constraint === null && hFail.breaks.length > 0);
check("rejection left the joints untouched", dist(pS.jointWorld(pj1), pj1w0) < 1e-9 && dist(pS.jointWorld(pj2), pj2w0) < 1e-9);

// H on guide 2 (one bound + one free point) works — only the free point moves.
// (Drop the perpendicular first: H plus ⊥-to-a-non-vertical-line would conflict.)
pS.removeSketchConstraint(perp.constraint!.id);
const h2 = tryAddConstraint(pS, "horizontal", { kind: "guideLine", guideId: pg2.id, edge: 0 });
check("H on a half-bound guide accepted", h2.constraint !== null);
check("H solved by the free point only", dist(pS.jointWorld(pj1), pj1w0) < 1e-9);
check(
  "guide 2 is horizontal (through its joint)",
  Math.abs(pS.getGuide(pg2.id)!.pts[0].y - pS.getGuide(pg2.id)!.pts[1].y) < 2e-3
);

// --- constraints hold during a drag (anchored-infeasible → symmetric fallback) -----
// V guide bound to a joint at `a`: dragging `b` sideways may not break the constraint.
// The drag-anchored solve is infeasible (neither the dragged point nor the joint may
// move), which is the app's cue to re-solve symmetrically — mirrored here.
const vS = new Scene();
const vj = vS.addFreeJoint({ x: 50, y: 10 });
const vg = vS.addGuidePoly([{ x: 50, y: 10 }, { x: 50, y: 110 }], false)!;
tryAddConstraint(vS, "coincident", { kind: "guidePoint", guideId: vg.id, which: "0" }, { kind: "joint", jointId: vj.id });
tryAddConstraint(vS, "vertical", { kind: "guideLine", guideId: vg.id, edge: 0 });
vS.moveGuidePoint(vg.id, "1", { x: 90, y: 130 }); // one drag step, off-axis
const anchoredBreaks = solveSketch(vS, new Set([anchorVarForGuidePoint(vg.id, "1")]));
check("anchored solve reports the infeasible drag", anchoredBreaks.length > 0);
const fbBreaks = solveSketch(vS); // the fallback the app runs on that failure
check("fallback solve converges", fbBreaks.length === 0);
check("verticality restored mid-drag", Math.abs(vg.pts[0].x - vg.pts[1].x) < 2e-3, `dx=${vg.pts[0].x - vg.pts[1].x}`);
check("drag's feasible component kept", Math.abs(vg.pts[1].y - 130) < 1e-9, `y=${vg.pts[1].y}`);
check("bound joint untouched by the fallback", dist(vS.jointWorld(vS.getJoint(vj.id)!), { x: 50, y: 10 }) < 1e-9);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
