/**
 * The Tangential sketch constraint: a line touching a circle — the circle's centre held
 * exactly one radius off the infinite line, on the side it is on. Covers validation
 * (a circle reference is a disk body, a circular hole, a reference circle or arc; a
 * line is a body edge, rail or reference segment), who moves by rank (a free reference
 * circle yields, a locked line holds, a locked disk centre pushes the edge), the arc
 * moving as a rigid piece, drag follow, a diameter edit re-solving the tangent, the
 * conflict reject, remaps (node added to a disk hole, hole removal, copy / paste,
 * serialize / load) and the pose route on component instances.
 */
import { Scene, MeasureRef, SceneData, Vec2, isCircleRef } from "../src/model";
import { solveSketch, tryAddConstraint, applyDrivingDimension, anchorVarsForBody, sketchConfig } from "../src/sketch";
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
const square = (x0 = 0, y0 = 0, w = 100, h = w): Vec2[] => [
  { x: x0, y: y0 },
  { x: x0 + w, y: y0 },
  { x: x0 + w, y: y0 + h },
  { x: x0, y: y0 + h },
];

const dref = (bodyId: number, hole?: number): MeasureRef => (hole === undefined ? { kind: "disk", bodyId } : { kind: "disk", bodyId, hole });
const gcref = (guideId: number): MeasureRef => ({ kind: "guideCircle", guideId });
const eref = (bodyId: number, index: number, hole?: number): MeasureRef => (hole === undefined ? { kind: "edge", bodyId, index } : { kind: "edge", bodyId, index, hole });
const vref = (bodyId: number, index: number, hole?: number): MeasureRef => (hole === undefined ? { kind: "vertex", bodyId, index } : { kind: "vertex", bodyId, index, hole });
const gref = (guideId: number, edge = 0): MeasureRef => ({ kind: "guideLine", guideId, edge });
const jref = (id: number): MeasureRef => ({ kind: "joint", jointId: id });

/** Resolved line of a ref, as a point + unit direction + unit normal. */
function lineOf(s: Scene, ref: MeasureRef) {
  const r = s.resolveMeasureRef(ref);
  if (!r || r.kind !== "line") throw new Error("not a line");
  const d = sub(r.b, r.a);
  const u = scale(d, 1 / len(d));
  return { a: r.a, b: r.b, u, n: perp(u) };
}
function circleOf(s: Scene, ref: MeasureRef): { c: Vec2; r: number } {
  const c = s.circleOfRef(ref);
  if (!c) throw new Error("not a circle");
  return c;
}
/** Signed distance of the circle's centre off the line (in the line's normal). */
const centreOff = (s: Scene, circle: MeasureRef, line: MeasureRef): number => {
  const L = lineOf(s, line);
  return dot(sub(circleOf(s, circle).c, L.a), L.n);
};
/** How far a line is from touching a circle: | |centre off line| − r |. */
const tangentGap = (s: Scene, circle: MeasureRef, line: MeasureRef): number =>
  Math.abs(Math.abs(centreOff(s, circle, line)) - circleOf(s, circle).r);

// --- validation ---------------------------------------------------------------
{
  const s = new Scene();
  const plate = s.addBody(square(), 0, "fillet", [{ control: [{ x: 50, y: 50 }], radius: 10, round: "offset" }]);
  const disk = s.addBody([{ x: 200, y: 200 }], 20, "offset");
  const poly = s.addBody(square(300, 0));
  const circ = s.addGuideCircle({ x: 400, y: 0 }, 30)!;
  const arc = s.addGuideArc({ x: 500, y: 0 }, { x: 530, y: 30 }, { x: 560, y: 0 })!;
  const seg = s.addGuidePoly([{ x: 0, y: -50 }, { x: 100, y: -60 }], false)!;
  const rail = s.addSlider(s.addFreeJoint({ x: 0, y: 300 }).id, s.addFreeJoint({ x: 100, y: 300 }).id);
  const j = s.addFreeJoint({ x: -100, y: -100 });
  const both = (a: MeasureRef, b: MeasureRef): boolean => {
    const problem = s.sketchConstraintProblem("tangent", a, b);
    const c = s.addSketchConstraint("tangent", a, b);
    if (c) s.removeSketchConstraint(c.id);
    return (problem === null) === (c !== null) && c !== null;
  };
  const refused = (a: MeasureRef, b?: MeasureRef): boolean => {
    const problem = s.sketchConstraintProblem("tangent", a, b);
    return problem !== null && s.addSketchConstraint("tangent", a, b) === null;
  };
  check("a disk body and a body edge ok", both(dref(disk.id), eref(plate.id, 0)));
  check("a circular hole and a body edge ok", both(dref(plate.id, 0), eref(poly.id, 0)));
  check("a reference circle and a reference segment ok", both(gcref(circ.id), gref(seg.id)));
  check("a reference arc and a rail ok", both(gcref(arc.id), { kind: "rail", sliderId: rail.id }));
  check("either pick order is accepted", both(eref(plate.id, 1), dref(disk.id)));
  {
    const c = s.addSketchConstraint("tangent", eref(plate.id, 1), dref(disk.id))!;
    check("…and the circle is stored as refA, the line as refB", isCircleRef(c.refA) && c.refB?.kind === "edge");
    s.removeSketchConstraint(c.id);
  }
  check("two circles refused", refused(dref(disk.id), gcref(circ.id)));
  check("two lines refused", refused(eref(plate.id, 0), gref(seg.id)));
  check("a point and a circle refused", refused(jref(j.id), dref(disk.id)));
  check("a point and a line refused", refused(jref(j.id), eref(plate.id, 0)));
  check("a single reference refused", refused(dref(disk.id)));
  check("a disk ref on a polygon body names no circle", refused(dref(poly.id), gref(seg.id)));
  check("a disk ref on a missing hole names no circle", refused(dref(plate.id, 3), gref(seg.id)));
  check("a guideCircle ref on a reference segment names no circle", refused(gcref(seg.id), eref(plate.id, 0)));
  check("the disk ref resolves to the disk's centre", (() => { const r = s.resolveMeasureRef(dref(disk.id)); return r?.kind === "point" && near(r.p.x, 200) && near(r.p.y, 200); })());
  check("the arc ref resolves to the arc's centre", (() => { const r = s.resolveMeasureRef(gcref(arc.id)); return r?.kind === "point" && near(r.p.x, 530) && near(r.p.y, 0); })());
  check("a disk ref is a circle, not a point or line", isCircleRef(dref(disk.id)) && !s.addSketchConstraint("coincident", dref(disk.id), jref(j.id)));
}

// --- a free disk comes to a locked edge; the plate never moves -------------------
{
  const s = new Scene();
  const plate = s.addBody(square()); // bottom edge y = 0 … top edge 2 at y = 100
  s.addSketchConstraint("fixed", eref(plate.id, 2)); // the top edge, y = 100
  const disk = s.addBody([{ x: 50, y: 160 }], 20, "offset"); // 40 above the edge: centre must come to y = 120
  const shape = JSON.stringify(plate.controlLocal);
  const side0 = Math.sign(centreOff(s, dref(disk.id), eref(plate.id, 2)));
  const { constraint } = tryAddConstraint(s, "tangent", dref(disk.id), eref(plate.id, 2));
  check("tangent between a disk body and a locked edge accepted", constraint !== null);
  const c = circleOf(s, dref(disk.id));
  check("…the disk touches the edge", tangentGap(s, dref(disk.id), eref(plate.id, 2)) < TOL, fmt(c.c));
  check("…moving straight down the normal to y = 120", near(c.c.x, 50, TOL) && near(c.c.y, 120, TOL), fmt(c.c));
  check("…keeping its radius", near(c.r, 20));
  check("…on the side it started on", Math.sign(centreOff(s, dref(disk.id), eref(plate.id, 2))) === side0);
  check("…the plate untouched", JSON.stringify(plate.controlLocal) === shape && dist(plate.pos, { x: 50, y: 50 }) < 1e-9);
  // Dragging the disk sideways slides it along the edge; away from the edge it is pulled back.
  s.moveBody(disk.id, { x: 30, y: 25 });
  check("the anchored solve after a drag converges", solveSketch(s, new Set(anchorVarsForBody(s, disk.id))).length === 0 || solveSketch(s).length === 0);
  const c2 = circleOf(s, dref(disk.id));
  check("…the disk stays tangent", tangentGap(s, dref(disk.id), eref(plate.id, 2)) < TOL, fmt(c2.c));
  check("…sliding along the edge to x = 80", near(c2.c.x, 80, TOL), fmt(c2.c));
}

// --- a circular hole tangent to its own plate's (locked) edge: the hole moves inside ------
{
  const s = new Scene();
  const plate = s.addBody(square(), 0, "fillet", [{ control: [{ x: 50, y: 50 }], radius: 10, round: "offset" }]);
  s.addSketchConstraint("fixed", eref(plate.id, 0)); // the bottom edge, y = 0
  const outer = s.bodyControlWorld(plate).map((p) => ({ ...p })); // world: moving the hole re-centres the body's frame
  const { constraint } = tryAddConstraint(s, "tangent", dref(plate.id, 0), eref(plate.id, 0));
  check("tangent between a hole and its plate's edge accepted", constraint !== null);
  const c = circleOf(s, dref(plate.id, 0));
  check("…the hole touches the edge from inside", tangentGap(s, dref(plate.id, 0), eref(plate.id, 0)) < TOL && near(c.c.y, 10, TOL), fmt(c.c));
  check("…keeping its radius", near(c.r, 10));
  check("…the outline untouched", s.bodyControlWorld(plate).every((p, i) => dist(p, outer[i]) < 1e-9));
}

// --- geometry against geometry shares the work ------------------------------------
{
  const s = new Scene();
  const plate = s.addBody(square()); // right edge 1 at x = 100
  const disk = s.addBody([{ x: 140, y: 50 }], 20, "offset"); // gap of 20 to the edge
  const { constraint } = tryAddConstraint(s, "tangent", dref(disk.id), eref(plate.id, 1));
  check("tangent between two free bodies accepted", constraint !== null);
  check("…they touch", tangentGap(s, dref(disk.id), eref(plate.id, 1)) < TOL);
  const c = circleOf(s, dref(disk.id));
  const E = lineOf(s, eref(plate.id, 1));
  check("…each moved half the gap (equal ranks)", near(c.c.x, 130, TOL) && near(E.a.x, 110, TOL) && near(E.b.x, 110, TOL), `disk x ${c.c.x.toFixed(3)}, edge x ${E.a.x.toFixed(3)}`);
}

// --- a free reference circle yields; the body edge stays --------------------------
{
  const s = new Scene();
  const plate = s.addBody(square());
  const circ = s.addGuideCircle({ x: 50, y: 180 }, 30)!; // 50 above the top edge
  const shape = JSON.stringify(plate.controlLocal);
  const { constraint } = tryAddConstraint(s, "tangent", eref(plate.id, 2), gcref(circ.id));
  check("tangent between a body edge and a reference circle accepted", constraint !== null);
  const c = circleOf(s, gcref(circ.id));
  check("…the reference circle came to the edge", tangentGap(s, gcref(circ.id), eref(plate.id, 2)) < TOL && near(c.c.y, 130, TOL), fmt(c.c));
  check("…with its radius intact", near(c.r, 30));
  check("…the body untouched", JSON.stringify(plate.controlLocal) === shape && dist(plate.pos, { x: 50, y: 50 }) < 1e-9);
}

// --- a reference arc moves as a rigid piece onto a locked reference line -------------
{
  const s = new Scene();
  const line = s.addGuidePoly([{ x: -100, y: 0 }, { x: 300, y: 0 }], false)!; // y = 0
  s.addSketchConstraint("fixed", gref(line.id));
  const arc = s.addGuideArc({ x: 0, y: 60 }, { x: 50, y: 110 }, { x: 100, y: 60 })!; // centre (50, 60), r 50 — 10 short of touching
  const before = circleOf(s, gcref(arc.id));
  const chord = dist(arc.a, arc.b);
  const { constraint } = tryAddConstraint(s, "tangent", gcref(arc.id), gref(line.id));
  check("tangent between a reference arc and a locked reference line accepted", constraint !== null);
  const after = circleOf(s, gcref(arc.id));
  check("…the arc touches the line", tangentGap(s, gcref(arc.id), gref(line.id)) < TOL, fmt(after.c));
  check("…moved straight down to y = 50", near(after.c.x, 50, TOL) && near(after.c.y, 50, TOL), fmt(after.c));
  check("…as a rigid piece: radius and chord unchanged", near(after.r, before.r, 1e-6) && near(dist(arc.a, arc.b), chord, 1e-6));
  const L = lineOf(s, gref(line.id));
  check("…the locked line never moved", near(L.a.y, 0, TOL) && near(L.b.y, 0, TOL));
}

// --- a line blending into an arc: tangent + coincident at the shared end ----------------
/** Where the centre's foot lands on a line (its tangent point) and the arc's geometry. */
function touch(s: Scene, arcRef: MeasureRef, lineRef: MeasureRef) {
  const A = circleOf(s, arcRef);
  const L = lineOf(s, lineRef);
  const t = dot(sub(A.c, L.a), L.u);
  return { c: A.c, r: A.r, foot: { x: L.a.x + L.u.x * t, y: L.a.y + L.u.y * t }, end: L.b };
}
{
  // Field repro (a user scene, not kept in the repo): two long, nearly parallel reference lines and a
  // ~154° arc — the rounded end of a slot — on 6000-unit geometry. The tangents alone
  // were fine; the coincident that should blend a line into the arc used to be refused:
  // the tangent (translating the whole arc) and the coincident (pulling one end back)
  // chased each other, and the "centre one radius off the line" residual is second-order
  // in the offset of the touching point, so even without the chase the error only fell
  // like 1/n². Pinned to the shared end, the tangent is an angle condition and settles in
  // a handful of sweeps.
  const s = new Scene();
  const l1 = s.addGuidePoly([{ x: -691.48518089921, y: -89.1887633043779 }, { x: 5746.426989565288, y: -213.91480147361816 }], false)!;
  const l2 = s.addGuidePoly([{ x: -790.5360364397807, y: 3133.0646556153533 }, { x: 5933.505369839645, y: 2926.171073883678 }], false)!;
  const arc = s.addGuideArc({ x: 6438.994248578267, y: 2.369651603099316 }, { x: 7196.869462296778, y: 1628.6207470525942 }, { x: 5519.553638972131, y: 2929.0946893659575 })!;
  const gp = (guideId: number, which: string): MeasureRef => ({ kind: "guidePoint", guideId, which });
  check("slot end: tangent to the first line", tryAddConstraint(s, "tangent", gcref(arc.id), gref(l1.id)).constraint !== null);
  check("slot end: tangent to the second line", tryAddConstraint(s, "tangent", gcref(arc.id), gref(l2.id)).constraint !== null);
  const sweeps: number[] = [];
  sketchConfig.trace = (k) => { sweeps[0] = k + 1; };
  const c1 = tryAddConstraint(s, "coincident", gp(arc.id, "a"), gp(l1.id, "1"));
  check("blending the arc's start into the first line's end is accepted", c1.constraint !== null, `${c1.breaks.length} break(s)`);
  check("…and settles fast (under 40 sweeps, not the whole budget)", (sweeps[0] ?? Infinity) < 40, `${sweeps[0]} sweeps`);
  const c2 = tryAddConstraint(s, "coincident", gp(arc.id, "b"), gp(l2.id, "1"));
  check("blending the arc's end into the second line's end is accepted too", c2.constraint !== null, `${c2.breaks.length} break(s)`);
  sketchConfig.trace = undefined;
  const g = s.getGuide(arc.id)!;
  if (g.kind === "arc") {
    const t1 = touch(s, gcref(arc.id), gref(l1.id));
    const t2 = touch(s, gcref(arc.id), gref(l2.id));
    check("…the arc's ends sit on the lines' ends", dist(g.a, t1.end) < TOL && dist(g.b, t2.end) < TOL, `${fmt(g.a)} vs ${fmt(t1.end)}`);
    check("…and each line touches the arc exactly at that shared end", dist(t1.foot, g.a) < 0.01 && dist(t2.foot, g.b) < 0.01, `foot ${fmt(t1.foot)} / ${fmt(t2.foot)}`);
    check("…both tangencies hold", tangentGap(s, gcref(arc.id), gref(l1.id)) < TOL && tangentGap(s, gcref(arc.id), gref(l2.id)) < TOL);
    // Dragging the shared end: the anchored solve keeps the blend, the line swings.
    const farBefore = { ...(s.getGuide(l1.id) as { kind: "poly"; pts: Vec2[] }).pts[0] };
    const target = { x: g.a.x + 150, y: g.a.y - 40 }; // (g is the live guide: capture before moving it)
    s.moveGuidePoint(arc.id, "a", target);
    const breaks = solveSketch(s, new Set([`g:${arc.id}:a`]));
    check("dragging the shared end keeps the blend solvable", breaks.length === 0, `${breaks.length} break(s)`);
    const t1b = touch(s, gcref(arc.id), gref(l1.id));
    const gA = (s.getGuide(arc.id) as { kind: "arc"; a: Vec2 }).a;
    check("…the dragged end stayed where the drag put it", dist(gA, target) < TOL, fmt(gA));
    check("…the line's end followed it and the line still touches there", dist(t1b.end, gA) < TOL && dist(t1b.foot, gA) < 0.01);
    check("…the line swung about the shared end (its far end moved)", dist((s.getGuide(l1.id) as { kind: "poly"; pts: Vec2[] }).pts[0], farBefore) > 1);
  }
}
{
  // The same blend against a *horizontal* line: the axis holds, so the arc does all the
  // turning and the line stays level.
  const s = new Scene();
  const line = s.addGuidePoly([{ x: -100, y: 0 }, { x: 100, y: 0 }], false)!;
  s.addSketchConstraint("horizontal", gref(line.id));
  const arc = s.addGuideArc({ x: 130, y: 20 }, { x: 180, y: 60 }, { x: 130, y: 100 })!;
  const gp = (guideId: number, which: string): MeasureRef => ({ kind: "guidePoint", guideId, which });
  check("H line + arc: tangent accepted", tryAddConstraint(s, "tangent", gcref(arc.id), gref(line.id)).constraint !== null);
  const r = tryAddConstraint(s, "coincident", gp(arc.id, "a"), gp(line.id, "1"));
  check("blending an arc into a horizontal line's end is accepted", r.constraint !== null, `${r.breaks.length} break(s)`);
  const L = lineOf(s, gref(line.id));
  check("…the line stayed horizontal (to tolerance)", Math.abs(L.b.y - L.a.y) < TOL, `Δy ${(L.b.y - L.a.y).toExponential(2)}`);
  const g = s.getGuide(arc.id) as { kind: "arc"; a: Vec2 };
  const t = touch(s, gcref(arc.id), gref(line.id));
  check("…the arc starts at the line's end and is tangent there", dist(g.a, t.end) < TOL && dist(t.foot, g.a) < 0.01, `a ${fmt(g.a)} end ${fmt(t.end)} foot ${fmt(t.foot)}`);
  // Point-on-line pinning: the arc's other end held onto a second line by a point-on-line.
  const line2 = s.addGuidePoly([{ x: 300, y: 150 }, { x: 100, y: 160 }], false)!;
  check("second line: tangent accepted", tryAddConstraint(s, "tangent", gcref(arc.id), gref(line2.id)).constraint !== null);
  const r2 = tryAddConstraint(s, "coincident", gp(arc.id, "b"), gref(line2.id));
  check("an arc end held onto a line by point-on-line blends too", r2.constraint !== null, `${r2.breaks.length} break(s)`);
  const gb = (s.getGuide(arc.id) as { kind: "arc"; b: Vec2 }).b;
  const t2 = touch(s, gcref(arc.id), gref(line2.id));
  check("…that line touches the arc at its end", dist(t2.foot, gb) < 0.01 && tangentGap(s, gcref(arc.id), gref(line2.id)) < TOL, `foot ${fmt(t2.foot)} b ${fmt(gb)}`);
}

// --- a locked disk centre pushes the edge instead ----------------------------------
{
  const s = new Scene();
  const plate = s.addBody(square());
  const disk = s.addBody([{ x: 50, y: 150 }], 20, "offset"); // 30 above the top edge (y = 100)
  s.addSketchConstraint("fixed", vref(disk.id, 0)); // the disk's centre is its one control vertex
  const { constraint } = tryAddConstraint(s, "tangent", dref(disk.id), eref(plate.id, 2));
  check("tangent onto a locked disk accepted", constraint !== null);
  const c = circleOf(s, dref(disk.id));
  check("…the disk never moved", near(c.c.x, 50) && near(c.c.y, 150), fmt(c.c));
  const E = lineOf(s, eref(plate.id, 2));
  check("…the edge rose to y = 130", near(E.a.y, 130, TOL) && near(E.b.y, 130, TOL), `y ${E.a.y.toFixed(3)} / ${E.b.y.toFixed(3)}`);
}

// --- a driving diameter re-solves the tangent -----------------------------------------
{
  const s = new Scene();
  const plate = s.addBody(square());
  s.addSketchConstraint("fixed", eref(plate.id, 2)); // y = 100
  const disk = s.addBody([{ x: 50, y: 120 }], 20, "offset"); // already tangent
  check("setup: tangent placed", tryAddConstraint(s, "tangent", dref(disk.id), eref(plate.id, 2)).constraint !== null);
  const m = s.addMeasurement("draw", vref(disk.id, 0), vref(disk.id, 0), { x: 50, y: 120 })!;
  check("a diameter dimension on the disk", m.axis === "diameter");
  const breaks = applyDrivingDimension(s, m.id, 60);
  check("driving the diameter to 60 succeeds", breaks.length === 0, `${breaks.length} break(s)`);
  const c = circleOf(s, dref(disk.id));
  check("…the radius is 30", near(c.r, 30));
  check("…and the disk rose to stay tangent (centre at y = 130)", near(c.c.y, 130, TOL) && tangentGap(s, dref(disk.id), eref(plate.id, 2)) < TOL, fmt(c.c));
}

// --- conflicts are rejected with the scene untouched ---------------------------------
{
  const s = new Scene();
  const line = s.addGuidePoly([{ x: -100, y: 0 }, { x: 300, y: 0 }], false)!;
  s.addSketchConstraint("fixed", gref(line.id));
  const disk = s.addBody([{ x: 50, y: 80 }], 20, "offset");
  s.addSketchConstraint("fixed", vref(disk.id, 0));
  const { constraint, breaks } = tryAddConstraint(s, "tangent", dref(disk.id), gref(line.id));
  check("a tangent between a locked disk and a locked line at the wrong distance is refused", constraint === null && breaks.length > 0, `${breaks.length} break(s)`);
  check("…geometry untouched", near(circleOf(s, dref(disk.id)).c.y, 80) && near(lineOf(s, gref(line.id)).a.y, 0));
  check("…and the constraint is gone", !s.sketch.some((c) => c.kind === "tangent"));
}

// --- remaps: a node added to the hole, hole removal, cascade, copy / paste, load ---------
{
  const s = new Scene();
  const plate = s.addBody(square(), 0, "fillet", [
    { control: [{ x: 25, y: 50 }], radius: 8, round: "offset" },
    { control: [{ x: 70, y: 50 }], radius: 10, round: "offset" },
  ]);
  const seg = s.addGuidePoly([{ x: 0, y: 200 }, { x: 100, y: 200 }], false)!;
  s.addSketchConstraint("fixed", gref(seg.id));
  const k = s.addSketchConstraint("tangent", dref(plate.id, 1), gref(seg.id))!;
  s.removeBodyHole(plate.id, 0);
  check("the disk ref follows its hole through a hole removal", k.refA.kind === "disk" && k.refA.hole === 0 && s.sketch.includes(k));
  check("…and still resolves", s.circleOfRef(k.refA) !== null);
  s.insertBodyVertex(plate.id, 1, { x: 80, y: 50 }, 0); // the round hole becomes a two-point outline
  check("a node added to the hole drops the tangent (no circle left)", !s.sketch.some((c) => c.kind === "tangent"));
  const disk = s.addBody([{ x: 50, y: 300 }], 20, "offset");
  const k2 = s.addSketchConstraint("tangent", dref(disk.id), gref(seg.id))!;
  s.removeGuide(seg.id);
  check("deleting the line drops the tangent", !s.sketch.some((c) => c.id === k2.id));
  const circ = s.addGuideCircle({ x: 300, y: 300 }, 25)!;
  const k3 = s.addSketchConstraint("tangent", gcref(circ.id), eref(plate.id, 0))!;
  s.removeGuide(circ.id);
  check("deleting the circle drops the tangent", !s.sketch.some((c) => c.id === k3.id));
  s.removeBody(disk.id);
  check("deleting a disk body drops its tangents", !s.sketch.some((c) => c.refA.kind === "disk" && c.refA.bodyId === disk.id));
}
{
  const s = new Scene();
  const plate = s.addBody(square());
  const disk = s.addBody([{ x: 50, y: 120 }], 20, "offset");
  s.addSketchConstraint("tangent", dref(disk.id), eref(plate.id, 2));
  const data = JSON.parse(JSON.stringify(s.serialize())) as SceneData;
  const t = new Scene();
  t.load(data);
  const k = t.sketch.find((x) => x.kind === "tangent");
  check("a tangent round-trips through save / load", !!k && k.refA.kind === "disk" && k.refB?.kind === "edge");
  check("…and solves clean", solveSketch(t).length === 0);
  const clip = s.extractSelection([plate.id, disk.id], [])!;
  const before = s.sketch.length;
  s.insertSelection(clip, { x: 300, y: 0 });
  check("copy / paste carries a tangent between the copied bodies", s.sketch.length === before + 1);
  const pasted = s.sketch[s.sketch.length - 1];
  check("…re-pointed at the pasted disk", pasted.kind === "tangent" && pasted.refA.kind === "disk" && pasted.refA.bodyId !== disk.id && s.circleOfRef(pasted.refA) !== null);
  check("…and satisfied where it landed", pasted.refB !== null && tangentGap(s, pasted.refA, pasted.refB) < TOL);
}

// --- component instances: one free end is a sketch constraint, two are a pose -------------
/** A grounded 40×40 block instance at the origin and a disk instance at `pos2`. */
function blockAndDisk(pos2: Vec2) {
  const s = new Scene();
  const b1 = s.addBody(square(-20, -20, 40));
  s.toggleBodyGround(b1.id);
  const block = s.createComponentFromSelection("Block", [b1.id])!;
  const d = s.addBody([{ x: 0, y: 0 }], 15, "offset"); // defined at the origin, so an instance's centre is its `pos`
  const diskDef = s.createComponentFromSelection("Disk", [d.id])!;
  const inst2 = s.instantiateComponent(diskDef.def.id, { pos: pos2, angle: 0 })!;
  s.removeInstance(diskDef.instance.id); // keep only the second disk instance
  const body1 = s.getBody(block.instance.bodyMap[0].id)!;
  const body2 = s.getBody(inst2.bodyMap[0].id)!;
  return { s, inst2, body1, body2 };
}
{
  const s = new Scene();
  const d = s.addBody([{ x: 50, y: 150 }], 20, "offset");
  const def = s.createComponentFromSelection("Disk", [d.id])!;
  const diskBody = s.getBody(def.instance.bodyMap[0].id)!;
  const plate = s.addBody(square()); // top edge y = 100, 30 below the instance disk's rim
  const r = placeConstraint(s, "tangent", dref(diskBody.id), eref(plate.id, 2));
  check("an instance disk and a free edge is a sketch constraint", r.constraint !== null && !isPoseConstraint(s, r.constraint), `${r.breaks.length} breaks`);
  const E = lineOf(s, eref(plate.id, 2));
  check("…the free edge rose to the instance's rim (y = 130)", near(E.a.y, 130, TOL) && near(E.b.y, 130, TOL), `y ${E.a.y.toFixed(3)}`);
  check("…the instance never moved", near(circleOf(s, dref(diskBody.id)).c.y, 150));
}
{
  const { s, inst2, body1, body2 } = blockAndDisk({ x: 60, y: 45 });
  check("setup: the instance body is a disk", s.circleOfRef(dref(body2.id)) !== null);
  const r = placeConstraint(s, "tangent", eref(body1.id, 1), dref(body2.id)); // block's right edge x = 20
  check("a tangent between two instances is a pose constraint", r.constraint !== null && r.breaks.length === 0 && isPoseConstraint(s, r.constraint), `${r.breaks.length} breaks`);
  const c = circleOf(s, dref(body2.id));
  check("…the disk instance translated onto the edge (centre at x = 35)", near(c.c.x, 35, TOL) && near(c.c.y, 45, TOL), fmt(c.c));
  check("…the grounded block held", dist(body1.pos, { x: 0, y: 0 }) < 1e-9);
  check("…nothing is violated", !poseConstraintViolated(s, r.constraint!));
  s.moveInstance(inst2.id, { x: 25, y: -10 });
  check("moving the instance reads as a violation", poseConstraintViolated(s, r.constraint!));
  const left = enforcePose(s);
  check("enforcePose puts it back", left.length === 0 && tangentGap(s, dref(body2.id), eref(body1.id, 1)) < TOL);
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll tangent-constraint checks passed.");
