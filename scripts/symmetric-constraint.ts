/**
 * The Symmetrical sketch constraint: two points, or two lines, mirror images of each
 * other across a third reference, the mirror line. Covers validation, the point form
 * (true mirror images: equal perpendicular distances *and* one perpendicular), the line
 * form (angle and offset mirrored, endpoints free), who moves — a free reference mirror
 * is re-placed onto the bisector by its first symmetry, holds once it carries two, is
 * tied, or is locked; an anchored point stays and its partner follows — conflict
 * rejects, cascade removal, serialize/load, copy/paste, and the pose route on component
 * instances (a side that carries the mirror moves by the reflected shift).
 */
import { Scene, MeasureRef, SceneData, Vec2, sketchRefs } from "../src/model";
import { solveSketch, tryAddConstraint, anchorVarsForJoint, sketchConfig } from "../src/sketch";
import { placeConstraint, enforcePose, isPoseConstraint, poseConstraintViolated } from "../src/pose";
import { dist, sub, dot, perp, len, scale } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: number, b: number, eps = 5e-3) => Math.abs(a - b) < eps;
const TOL = sketchConfig.tol * 2;
const fmt = (p: Vec2) => `(${p.x.toFixed(3)}, ${p.y.toFixed(3)})`;
const square = (x0 = 0, y0 = 0, s = 100): Vec2[] => [
  { x: x0, y: y0 },
  { x: x0 + s, y: y0 },
  { x: x0 + s, y: y0 + s },
  { x: x0, y: y0 + s },
];

/** Resolved line of a ref, as a point + unit direction + unit normal. */
function lineOf(s: Scene, ref: MeasureRef) {
  const r = s.resolveMeasureRef(ref);
  if (!r || r.kind !== "line") throw new Error("not a line");
  const d = sub(r.b, r.a);
  const u = scale(d, 1 / len(d));
  return { a: r.a, b: r.b, u, n: perp(u) };
}
function pointOf(s: Scene, ref: MeasureRef): Vec2 {
  const r = s.resolveMeasureRef(ref);
  if (!r || r.kind !== "point") throw new Error("not a point");
  return r.p;
}
const reflect = (L: { a: Vec2; n: Vec2 }, p: Vec2): Vec2 => sub(p, scale(L.n, 2 * dot(sub(p, L.a), L.n)));
/** How far two points are from being mirror images across a line. */
const mirrorGap = (s: Scene, a: MeasureRef, b: MeasureRef, m: MeasureRef): number =>
  dist(reflect(lineOf(s, m), pointOf(s, a)), pointOf(s, b));
/** How far two lines are from being mirror images across a line (as infinite lines). */
function lineMirrorGap(s: Scene, a: MeasureRef, b: MeasureRef, m: MeasureRef): number {
  const M = lineOf(s, m);
  const A = lineOf(s, a);
  const B = lineOf(s, b);
  const ia = reflect(M, A.a);
  const ib = reflect(M, A.b);
  const d = sub(ib, ia);
  const n = perp(scale(d, 1 / len(d)));
  return Math.max(Math.abs(dot(sub(B.a, ia), n)), Math.abs(dot(sub(B.b, ia), n)));
}
const jref = (id: number): MeasureRef => ({ kind: "joint", jointId: id });
const vref = (bodyId: number, index: number): MeasureRef => ({ kind: "vertex", bodyId, index });
const eref = (bodyId: number, index: number): MeasureRef => ({ kind: "edge", bodyId, index });
const gref = (guideId: number): MeasureRef => ({ kind: "guideLine", guideId, edge: 0 });

// --- validation ---------------------------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  const j1 = s.addFreeJoint({ x: 200, y: 10 });
  const j2 = s.addFreeJoint({ x: 260, y: 40 });
  const g = s.addGuidePoly([{ x: 150, y: -50 }, { x: 150, y: 150 }], false)!;
  const m = gref(g.id);
  check("points about a line ok", s.addSketchConstraint("symmetric", jref(j1.id), jref(j2.id), m) !== null);
  check("lines about a line ok", s.addSketchConstraint("symmetric", eref(b.id, 0), eref(b.id, 2), m) !== null);
  check("a point and a line rejected", s.addSketchConstraint("symmetric", jref(j1.id), eref(b.id, 0), m) === null);
  check("no mirror rejected", s.addSketchConstraint("symmetric", jref(j1.id), jref(j2.id)) === null);
  check("a point as the mirror rejected", s.addSketchConstraint("symmetric", jref(j1.id), jref(j2.id), vref(b.id, 0)) === null);
  check("the same element twice rejected", s.addSketchConstraint("symmetric", jref(j1.id), jref(j1.id), m) === null);
  check("a line mirrored about itself rejected", s.addSketchConstraint("symmetric", eref(b.id, 1), eref(b.id, 3), eref(b.id, 1)) === null);
  check("a mirror's own endpoint as an object rejected", s.addSketchConstraint("symmetric", vref(b.id, 0), vref(b.id, 2), eref(b.id, 0)) === null);
  check("a bodyPoint rejected", s.addSketchConstraint("symmetric", { kind: "bodyPoint", bodyId: b.id, local: { x: 0, y: 0 } }, jref(j1.id), m) === null);
  check("a mirror on another kind rejected", s.addSketchConstraint("parallel", eref(b.id, 0), eref(b.id, 2), m) === null);
  const c = s.sketch.find((k) => k.kind === "symmetric" && k.refA.kind === "joint")!;
  check("the constraint carries its mirror", !!c.mirror && c.mirror.kind === "guideLine" && sketchRefs(c).length === 3);
}

// --- points about a locked reference line: the pair moves, the mirror holds ---------
{
  const s = new Scene();
  const g = s.addGuidePoly([{ x: 100, y: -50 }, { x: 100, y: 150 }], false)!; // x = 100
  s.addSketchConstraint("fixed", gref(g.id));
  const a = s.addFreeJoint({ x: 20, y: 30 });
  const b = s.addFreeJoint({ x: 150, y: 70 });
  const { constraint } = tryAddConstraint(s, "symmetric", jref(a.id), jref(b.id), gref(g.id));
  check("symmetric points about a locked line accepted", constraint !== null);
  const pa = pointOf(s, jref(a.id));
  const pb = pointOf(s, jref(b.id));
  check("…they are mirror images", mirrorGap(s, jref(a.id), jref(b.id), gref(g.id)) < TOL, `${fmt(pa)} / ${fmt(pb)}`);
  check("…equidistant from the mirror, on opposite sides", near(100 - pa.x, pb.x - 100, TOL) && pa.x < 100 && pb.x > 100);
  check("…and on one perpendicular", near(pa.y, pb.y, TOL));
  const M = lineOf(s, gref(g.id));
  check("…the locked mirror never moved", near(M.a.x, 100, TOL) && near(M.b.x, 100, TOL));
  check("…each moved half the way (equal ranks)", near(pa.y, 50, TOL) && near(pb.y, 50, TOL), `y ${pa.y.toFixed(3)} / ${pb.y.toFixed(3)}`);
}

// --- a free reference mirror is re-placed by its first symmetry ---------------------
{
  const s = new Scene();
  const g = s.addGuidePoly([{ x: 100, y: -50 }, { x: 100, y: 150 }], false)!;
  const a = s.addFreeJoint({ x: 20, y: 30 });
  const b = s.addFreeJoint({ x: 150, y: 70 });
  const { constraint } = tryAddConstraint(s, "symmetric", jref(a.id), jref(b.id), gref(g.id));
  check("symmetric points about a free reference line accepted", constraint !== null);
  const pa = pointOf(s, jref(a.id));
  const pb = pointOf(s, jref(b.id));
  check("…the points did not move", near(pa.x, 20) && near(pa.y, 30) && near(pb.x, 150) && near(pb.y, 70), `${fmt(pa)} / ${fmt(pb)}`);
  check("…the reference line became their perpendicular bisector", mirrorGap(s, jref(a.id), jref(b.id), gref(g.id)) < TOL);
  const M = lineOf(s, gref(g.id));
  const mid = { x: 85, y: 50 };
  check("…through the pair's midpoint", Math.abs(dot(sub(mid, M.a), M.n)) < TOL);
  check("…perpendicular to the pair", Math.abs(dot(M.u, { x: 130, y: 40 })) / len({ x: 130, y: 40 }) < 1e-6);
  // A second pair about the same line: the line now carries two demands and holds;
  // the new pair comes to it.
  const c = s.addFreeJoint({ x: 10, y: 120 });
  const d = s.addFreeJoint({ x: 200, y: 100 });
  const M0 = lineOf(s, gref(g.id));
  const r2 = tryAddConstraint(s, "symmetric", jref(c.id), jref(d.id), gref(g.id));
  check("a second symmetry about the same line accepted", r2.constraint !== null);
  const M1 = lineOf(s, gref(g.id));
  check("…the twice-demanded mirror held still", dist(M0.a, M1.a) < TOL && dist(M0.b, M1.b) < TOL, `${fmt(M1.a)} / ${fmt(M1.b)}`);
  check("…both pairs are mirror images", mirrorGap(s, jref(a.id), jref(b.id), gref(g.id)) < TOL && mirrorGap(s, jref(c.id), jref(d.id), gref(g.id)) < TOL);
  const pc = pointOf(s, jref(c.id));
  check("…the second pair moved", !(near(pc.x, 10) && near(pc.y, 120)), fmt(pc));
}

// --- a tied reference mirror is the reference ----------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square()); // corners 0..3: (0,0) (100,0) (100,100) (0,100)
  const g = s.addGuidePoly([{ x: 50, y: -20 }, { x: 50, y: 120 }], false)!;
  // Tie the mirror to the body: its first point coincident with the bottom edge's midpoint.
  s.addSketchConstraint("coincident", { kind: "guidePoint", guideId: g.id, which: "0" }, { kind: "midpoint", of: { kind: "edge", bodyId: b.id, index: 0 } });
  s.addSketchConstraint("vertical", gref(g.id));
  check("setup solves", solveSketch(s).length === 0);
  s.moveBodyVertex(b.id, 2, { x: 30, y: 10 }, null); // (130, 110): break the symmetry of corners 2 and 3
  const { constraint } = tryAddConstraint(s, "symmetric", vref(b.id, 2), vref(b.id, 3), gref(g.id));
  check("symmetric corners about a tied reference line accepted", constraint !== null);
  const M = lineOf(s, gref(g.id));
  check("…the tied mirror stayed at x = 50", near(M.a.x, 50, TOL) && near(M.b.x, 50, TOL), `x ${M.a.x.toFixed(3)} / ${M.b.x.toFixed(3)}`);
  check("…the corners became mirror images", mirrorGap(s, vref(b.id, 2), vref(b.id, 3), gref(g.id)) < TOL);
  const c = s.bodyControlWorld(b);
  check("…the bottom edge still spans the mirror's foot", near((c[0].x + c[1].x) / 2, 50, TOL));
}

// --- two lines about a locked mirror: angle and offset mirror, ends stay free ---------
{
  const s = new Scene();
  const g = s.addGuidePoly([{ x: 0, y: -100 }, { x: 0, y: 300 }], false)!; // the y axis
  s.addSketchConstraint("fixed", gref(g.id));
  const left = s.addGuidePoly([{ x: -120, y: 0 }, { x: -40, y: 90 }], false)!; // a slanted segment
  const right = s.addGuidePoly([{ x: 30, y: 10 }, { x: 130, y: 20 }], false)!; // a shorter, flatter one
  const { constraint } = tryAddConstraint(s, "symmetric", gref(left.id), gref(right.id), gref(g.id));
  check("symmetric lines about a locked mirror accepted", constraint !== null);
  check("…the two lines are mirror images as infinite lines", lineMirrorGap(s, gref(left.id), gref(right.id), gref(g.id)) < TOL);
  const L = lineOf(s, gref(left.id));
  const R = lineOf(s, gref(right.id));
  check("…their angles mirror", near(Math.atan2(L.u.y, L.u.x), Math.PI - Math.atan2(R.u.y, R.u.x), 1e-4) || near(Math.atan2(L.u.y, L.u.x), -Math.atan2(R.u.y, R.u.x), 1e-4));
  check("…their lengths may differ", !near(dist(L.a, L.b), dist(R.a, R.b), 1), `${dist(L.a, L.b).toFixed(1)} vs ${dist(R.a, R.b).toFixed(1)}`);
  check("…and each kept its own length (a rigid turn and shift, not a projection)", near(dist(L.a, L.b), Math.hypot(80, 90), 1e-3) && near(dist(R.a, R.b), Math.hypot(100, 10), 1e-3));
  const M = lineOf(s, gref(g.id));
  check("…the locked mirror never moved", near(M.a.x, 0, TOL) && near(M.b.x, 0, TOL));
  // Sliding an endpoint along its own line keeps the symmetry: nothing else moves.
  const before = lineOf(s, gref(right.id));
  s.moveGuidePoint(left.id, "1", { x: L.b.x + L.u.x * 30, y: L.b.y + L.u.y * 30 });
  check("stretching one line along itself solves", solveSketch(s).length === 0);
  const after = lineOf(s, gref(right.id));
  check("…and leaves the partner alone", dist(before.a, after.a) < TOL && dist(before.b, after.b) < TOL);
}

// --- two body edges about a body edge: geometry against geometry shares the work -----
{
  const s = new Scene();
  const plate = s.addBody(square(0, 0, 200)); // its bottom edge (y = 0) is the mirror
  const wingA = s.addBody([{ x: 20, y: 20 }, { x: 80, y: 20 }, { x: 80, y: 60 }, { x: 20, y: 60 }]);
  const wingB = s.addBody([{ x: 20, y: -70 }, { x: 90, y: -70 }, { x: 90, y: -20 }, { x: 15, y: -25 }]);
  const { constraint } = tryAddConstraint(s, "symmetric", eref(wingA.id, 0), eref(wingB.id, 2), eref(plate.id, 0));
  check("symmetric edges about a body edge accepted", constraint !== null);
  check("…the edges mirror across the plate's edge", lineMirrorGap(s, eref(wingA.id, 0), eref(wingB.id, 2), eref(plate.id, 0)) < TOL);
}

// --- a dragged point stays; its partner follows ------------------------------------
{
  const s = new Scene();
  const g = s.addGuidePoly([{ x: 100, y: -50 }, { x: 100, y: 150 }], false)!;
  s.addSketchConstraint("fixed", gref(g.id));
  const a = s.addFreeJoint({ x: 40, y: 30 });
  const b = s.addFreeJoint({ x: 160, y: 30 });
  check("setup: already symmetric", tryAddConstraint(s, "symmetric", jref(a.id), jref(b.id), gref(g.id)).constraint !== null);
  s.moveJoint(a.id, { x: -15, y: 25 }); // as a drag would leave it: (25, 55)
  check("the anchored solve converges", solveSketch(s, new Set(anchorVarsForJoint(s, a.id))).length === 0);
  const pa = pointOf(s, jref(a.id));
  const pb = pointOf(s, jref(b.id));
  check("…the dragged point stayed where the drag put it", near(pa.x, 25, TOL) && near(pa.y, 55, TOL), fmt(pa));
  check("…its partner moved to the mirror image", near(pb.x, 175, TOL) && near(pb.y, 55, TOL), fmt(pb));
}

// --- conflicts are rejected with the scene untouched ---------------------------------
{
  const s = new Scene();
  const g = s.addGuidePoly([{ x: 100, y: -50 }, { x: 100, y: 150 }], false)!;
  s.addSketchConstraint("fixed", gref(g.id));
  const a = s.addFreeJoint({ x: 20, y: 30 });
  const b = s.addFreeJoint({ x: 150, y: 70 });
  s.addSketchConstraint("fixed", jref(a.id));
  s.addSketchConstraint("fixed", jref(b.id));
  const { constraint, breaks } = tryAddConstraint(s, "symmetric", jref(a.id), jref(b.id), gref(g.id));
  check("a symmetry between two locked points about a locked line is refused", constraint === null && breaks.length > 0, `${breaks.length} break(s)`);
  check("…geometry untouched", near(pointOf(s, jref(a.id)).x, 20) && near(pointOf(s, jref(b.id)).x, 150));
  check("…and the constraint is gone", !s.sketch.some((c) => c.kind === "symmetric"));
}

// --- cascade removal: any of the three elements going away drops the constraint -------
{
  const s = new Scene();
  const g = s.addGuidePoly([{ x: 100, y: -50 }, { x: 100, y: 150 }], false)!;
  const a = s.addFreeJoint({ x: 20, y: 30 });
  const b = s.addFreeJoint({ x: 180, y: 30 });
  s.addSketchConstraint("symmetric", jref(a.id), jref(b.id), gref(g.id));
  s.removeGuide(g.id);
  check("deleting the mirror drops the symmetry", !s.sketch.some((c) => c.kind === "symmetric"));
  const g2 = s.addGuidePoly([{ x: 100, y: -50 }, { x: 100, y: 150 }], false)!;
  s.addSketchConstraint("symmetric", jref(a.id), jref(b.id), gref(g2.id));
  s.removeJoint(b.id);
  check("deleting one of the pair drops it too", !s.sketch.some((c) => c.kind === "symmetric"));
}

// --- the mirror follows its element through an index remap --------------------------
{
  const s = new Scene();
  const b = s.addBody(square()); // edge 2 is the top (100,100)→(0,100)
  const a = s.addFreeJoint({ x: 20, y: 130 });
  const c = s.addFreeJoint({ x: 30, y: 70 });
  const k = s.addSketchConstraint("symmetric", jref(a.id), jref(c.id), eref(b.id, 2))!;
  s.insertBodyVertex(b.id, 1, { x: 50, y: 0 }); // a new node on the bottom edge shifts every later index
  check("the mirror's edge index follows a node insert", k.mirror?.kind === "edge" && k.mirror.index === 3, `index ${k.mirror?.kind === "edge" ? k.mirror.index : "?"}`);
  const M = lineOf(s, k.mirror!);
  check("…and still names the top edge", near(M.a.y, 100) && near(M.b.y, 100));
}

// --- serialize / load and copy / paste ---------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  const a = s.addFreeJoint({ x: 20, y: 130 });
  const c = s.addFreeJoint({ x: 80, y: 130 });
  s.addSketchConstraint("symmetric", jref(a.id), jref(c.id), eref(b.id, 1));
  const data = JSON.parse(JSON.stringify(s.serialize())) as SceneData;
  const t = new Scene();
  t.load(data);
  const k = t.sketch.find((x) => x.kind === "symmetric");
  check("a symmetry round-trips with its mirror", !!k?.mirror && k.mirror.kind === "edge" && k.mirror.index === 1);
  check("the loaded mirror is a copy", k!.mirror !== data.sketch!.find((x) => x.kind === "symmetric")!.mirror);
  check("a loaded scene's symmetry solves clean", solveSketch(t).length === 0);
  // Copy the body with both joints: the symmetry is fully internal and travels along.
  const clip = s.extractSelection([b.id], [a.id, c.id])!;
  const before = s.sketch.length;
  s.insertSelection(clip, { x: 300, y: 0 });
  check("copy / paste carries a fully-internal symmetry", s.sketch.length === before + 1);
  const pasted = s.sketch[s.sketch.length - 1];
  check("…re-pointed at the pasted body", pasted.kind === "symmetric" && pasted.mirror?.kind === "edge" && pasted.mirror.bodyId !== b.id);
}

// --- instance points about a reference line: the sketch route, the line does the moving --
/** Two instances of a 40×40 block (block 1 grounded at the origin, block 2 at `pos2`). */
function twoBlocks(pos2: Vec2) {
  const s = new Scene();
  const sq = (cx: number, cy: number, half = 20) =>
    s.addBody([{ x: cx - half, y: cy - half }, { x: cx + half, y: cy - half }, { x: cx + half, y: cy + half }, { x: cx - half, y: cy + half }]);
  const b1 = sq(0, 0);
  s.toggleBodyGround(b1.id);
  const res = s.createComponentFromSelection("Block", [b1.id])!;
  const inst2 = s.instantiateComponent(res.def.id, { pos: pos2, angle: 0 })!;
  const body1 = s.getBody(res.instance.bodyMap[0].id)!;
  const body2 = s.getBody(inst2.bodyMap[0].id)!;
  return { s, inst2, body1, body2 };
}
{
  const { s, body1, body2 } = twoBlocks({ x: 260, y: 45 });
  const g = s.addGuidePoly([{ x: 100, y: -100 }, { x: 100, y: 200 }], false)!;
  const shape1 = JSON.stringify(body1.controlLocal);
  const shape2 = JSON.stringify(body2.controlLocal);
  const pos2 = { x: body2.pos.x, y: body2.pos.y };
  // Instance geometry is immovable in the sketch; the reference line is the only thing
  // that can move, so it becomes the pair's bisector.
  const r = placeConstraint(s, "symmetric", vref(body1.id, 1), vref(body2.id, 0), gref(g.id));
  check("instance points about a free reference line is a sketch constraint", r.constraint !== null && !isPoseConstraint(s, r.constraint), `${r.breaks.length} breaks`);
  check("…the reference line became the bisector", mirrorGap(s, vref(body1.id, 1), vref(body2.id, 0), gref(g.id)) < TOL);
  check("…neither instance moved or deformed", dist(body1.pos, { x: 0, y: 0 }) < 1e-9 && dist(body2.pos, pos2) < 1e-9 && JSON.stringify(body1.controlLocal) === shape1 && JSON.stringify(body2.controlLocal) === shape2);
}
{
  // The same about a *locked* reference line: nothing may move — refused, instances intact.
  const { s, body1, body2 } = twoBlocks({ x: 260, y: 45 });
  const g = s.addGuidePoly([{ x: 100, y: -100 }, { x: 100, y: 200 }], false)!;
  s.addSketchConstraint("fixed", gref(g.id));
  const shape2 = JSON.stringify(body2.controlLocal);
  const pos2 = { x: body2.pos.x, y: body2.pos.y };
  const r = placeConstraint(s, "symmetric", vref(body1.id, 1), vref(body2.id, 0), gref(g.id));
  check("instance points about a locked reference line is refused", r.constraint === null && r.breaks.length > 0, `${r.breaks.length} break(s)`);
  check("…instances untouched", dist(body1.pos, { x: 0, y: 0 }) < 1e-9 && dist(body2.pos, pos2) < 1e-9 && JSON.stringify(body2.controlLocal) === shape2);
}

// --- pose route: every reference on instance geometry moves a part rigidly ------------
{
  const { s, inst2, body1, body2 } = twoBlocks({ x: 260, y: 45 });
  const shape = JSON.stringify(body2.controlLocal);
  // Block 1's corner 0 is (-20, -20); its image across block 1's right edge (x = 20) is
  // (60, -20). Block 2's corner 0 is (240, 25): block 2 translates so it lands there.
  const mirror = eref(body1.id, 1);
  const r = placeConstraint(s, "symmetric", vref(body1.id, 0), vref(body2.id, 0), mirror);
  check("symmetric points on two instances about an instance edge is a pose constraint", r.constraint !== null && isPoseConstraint(s, r.constraint), `${r.breaks.length} breaks`);
  const c0 = pointOf(s, vref(body2.id, 0));
  check("…block 2 translated onto the mirror image", near(c0.x, 60, TOL) && near(c0.y, -20, TOL), fmt(c0));
  check("…the grounded block held", dist(body1.pos, { x: 0, y: 0 }) < 1e-9);
  check("…the instance's shape is untouched", JSON.stringify(body2.controlLocal) === shape);
  check("…nothing is violated", !poseConstraintViolated(s, r.constraint!));
  s.moveInstance(inst2.id, { x: 30, y: -12 });
  check("moving the instance reads as a violation", poseConstraintViolated(s, r.constraint!));
  const left = enforcePose(s);
  const c1 = pointOf(s, vref(body2.id, 0));
  check("enforcePose puts it back", left.length === 0 && near(c1.x, 60, TOL) && near(c1.y, -20, TOL), fmt(c1));
}
{
  // Lines: block 2's top edge mirrored to block 1's bottom edge across block 1's right
  // edge (x = 20) — a turn is undone and the edge shifted onto the image line y = -20.
  const { s, inst2, body1, body2 } = twoBlocks({ x: 200, y: 60 });
  const mirror = eref(body1.id, 1);
  s.rotateInstance(inst2.id, body2.pos, 0.3);
  const r = placeConstraint(s, "symmetric", eref(body1.id, 0), eref(body2.id, 2), mirror);
  check("symmetric lines on two instances is a pose constraint", r.constraint !== null && r.breaks.length === 0 && isPoseConstraint(s, r.constraint), `${r.breaks.length} breaks`);
  const E = lineOf(s, eref(body2.id, 2));
  check("…block 2's edge turned back level", Math.abs(E.u.y) < 1e-4, `u ${fmt(E.u)}`);
  check("…and sits on the image line", near(E.a.y, -20, TOL) && near(E.b.y, -20, TOL), `y ${E.a.y.toFixed(3)} / ${E.b.y.toFixed(3)}`);
  check("…the grounded block held", dist(body1.pos, { x: 0, y: 0 }) < 1e-9);
  check("…the symmetry reads satisfied", !poseConstraintViolated(s, r.constraint!));
}
{
  // The mirror riding with the moving side: block 2's own edge is the mirror, block 1
  // (free this time) is grounded — so block 2 must move, carrying the mirror along, and
  // the closed-form move is the *reflected* shift. Block 1's corner 1 (20, -20) and
  // block 2's corner 0 must end up mirror images across block 2's left edge.
  const s = new Scene();
  const sq = (cx: number, cy: number, half = 20) =>
    s.addBody([{ x: cx - half, y: cy - half }, { x: cx + half, y: cy - half }, { x: cx + half, y: cy + half }, { x: cx - half, y: cy + half }]);
  const b1 = sq(0, 0);
  s.toggleBodyGround(b1.id);
  const res = s.createComponentFromSelection("Block", [b1.id])!;
  const inst2 = s.instantiateComponent(res.def.id, { pos: { x: 200, y: 30 }, angle: 0 })!;
  const body1 = s.getBody(res.instance.bodyMap[0].id)!;
  const body2 = s.getBody(inst2.bodyMap[0].id)!;
  // Block 2's corner 0 is its bottom-left (180, 10); edge 3 is its left side x = 180.
  // A corner *on* the mirror can't be mirrored to anything but itself — use a
  // second-instance corner off the mirror: corner 1 (220, 10) about edge 3 (x = 180)
  // has its image at (140, 10); block 1's corner 1 is (20, -20), so block 2 must slide
  // until 2·x_edge − 220' = 20 and y matches: a pure translation of (−60, −30)… but the
  // mirror moves with it, so the solver has to find t with R·t = plain shift.
  const r = placeConstraint(s, "symmetric", vref(body1.id, 1), vref(body2.id, 1), eref(body2.id, 3));
  check("a symmetry whose mirror rides with the moving side is accepted", r.constraint !== null && r.breaks.length === 0, `${r.breaks.length} breaks`);
  const gap = mirrorGap(s, vref(body1.id, 1), vref(body2.id, 1), eref(body2.id, 3));
  check("…and satisfied", gap < TOL, `gap ${gap.toExponential(2)}`);
  check("…the grounded block held", dist(body1.pos, { x: 0, y: 0 }) < 1e-9);
  check("…the symmetry reads satisfied", !poseConstraintViolated(s, r.constraint!));
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll symmetric-constraint checks passed.");
