/**
 * Line midpoints as sketch references (`MeasureRef.midpoint`, nesting the line's own
 * ref): resolution, constraint validation, the solver's derived-point handle (who moves
 * when a midpoint is pulled), a locked midpoint, and — the part that matters most —
 * that a midpoint follows its line through every index remap (node insert / remove,
 * mirror, split, copy/paste) and a serialize/load round trip.
 */
import { Scene, MeasureRef, MidpointHost, Vec2, sameMeasureRef, refHost } from "../src/model";
import { solveSketch, anchorVarsForBody, anchorVarForVertex, sketchConfig } from "../src/sketch";
import { vec, regularPolygon } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: number, b: number, eps = 5e-3) => Math.abs(a - b) < eps;
const nearPt = (a: Vec2, b: Vec2, eps = 5e-3) => near(a.x, b.x, eps) && near(a.y, b.y, eps);
const TOL = sketchConfig.tol;
const square = (x = 0, y = 0, w = 100, h = 100): Vec2[] => [
  { x, y },
  { x: x + w, y },
  { x: x + w, y: y + h },
  { x, y: y + h },
];
const mid = (of: MidpointHost): MeasureRef => ({ kind: "midpoint", of });
const edge = (bodyId: number, index: number, hole?: number): MidpointHost =>
  hole === undefined ? { kind: "edge", bodyId, index } : { kind: "edge", bodyId, index, hole };
const pointOf = (s: Scene, r: MeasureRef): Vec2 | null => {
  const res = s.resolveMeasureRef(r);
  return res?.kind === "point" ? res.p : null;
};

// --- resolution + equality ---------------------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  const rail = s.addSlider(s.addFreeJoint({ x: 0, y: 200 }).id, s.addFreeJoint({ x: 100, y: 240 }).id);
  const g = s.addGuidePoly([{ x: 10, y: 300 }, { x: 30, y: 300 }, { x: 30, y: 340 }], false)!;

  check("midpoint of a body edge resolves to its middle", nearPt(pointOf(s, mid(edge(b.id, 0)))!, vec(50, 0)));
  check("midpoint of the closing edge wraps", nearPt(pointOf(s, mid(edge(b.id, 3)))!, vec(0, 50)));
  check("midpoint of a rail resolves", nearPt(pointOf(s, mid({ kind: "rail", sliderId: rail.id }))!, vec(50, 220)));
  check("midpoint of a reference segment resolves", nearPt(pointOf(s, mid({ kind: "guideLine", guideId: g.id, edge: 1 }))!, vec(30, 320)));
  check("midpoint of a missing edge is null", s.resolveMeasureRef(mid(edge(b.id, 7))) === null);
  check("midpoint of a missing rail is null", s.resolveMeasureRef(mid({ kind: "rail", sliderId: 9999 })) === null);
  check("same midpoint compares equal", sameMeasureRef(mid(edge(b.id, 1)), mid(edge(b.id, 1))));
  check("midpoints of different edges differ", !sameMeasureRef(mid(edge(b.id, 1)), mid(edge(b.id, 2))));
  check("a midpoint never equals its own line", !sameMeasureRef(mid(edge(b.id, 1)), edge(b.id, 1)));
  check("refHost unwraps a midpoint", refHost(mid(edge(b.id, 1))).kind === "edge" && refHost({ kind: "joint", jointId: 1 }).kind === "joint");
}

// --- constraint validation -----------------------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 200, y: 0 });
  const jointRef: MeasureRef = { kind: "joint", jointId: j.id };
  const m0 = mid(edge(b.id, 0));

  check("coincident joint + midpoint ok", s.addSketchConstraint("coincident", jointRef, m0) !== null);
  check("coincident midpoint + its own line rejected", s.addSketchConstraint("coincident", m0, edge(b.id, 0)) === null);
  check("coincident own line + midpoint rejected either order", s.addSketchConstraint("coincident", edge(b.id, 0), m0) === null);
  check("coincident midpoint + another edge ok", s.addSketchConstraint("coincident", m0, edge(b.id, 2)) !== null);
  check("horizontal joint + midpoint ok", s.addSketchConstraint("horizontal", jointRef, mid(edge(b.id, 1))) !== null);
  check("vertical midpoint + midpoint ok", s.addSketchConstraint("vertical", mid(edge(b.id, 1)), mid(edge(b.id, 3))) !== null);
  check("same midpoint twice rejected", s.addSketchConstraint("coincident", m0, mid(edge(b.id, 0))) === null);
  check("horizontal on a bare midpoint rejected (not a line)", s.addSketchConstraint("horizontal", m0) === null);
  check("parallel with a midpoint rejected", s.addSketchConstraint("parallel", m0, edge(b.id, 1)) === null);
  check("fixed on a midpoint ok", s.addSketchConstraint("fixed", mid(edge(b.id, 2))) !== null);
  check("midpoint of a missing edge rejected", s.addSketchConstraint("coincident", jointRef, mid(edge(b.id, 9))) === null);
}

// --- solving: coincident / H / V onto a midpoint ---------------------------------------
{
  // Symmetric solve: the joint and the edge share the correction, but they meet exactly.
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 70, y: 30 });
  s.addSketchConstraint("coincident", { kind: "joint", jointId: j.id }, mid(edge(b.id, 0)));
  check("coincident onto a midpoint solves", solveSketch(s).length === 0);
  const m = pointOf(s, mid(edge(b.id, 0)))!;
  check("joint sits on the edge's midpoint", nearPt(s.jointWorld(j), m, TOL * 10), `joint (${s.jointWorld(j).x.toFixed(3)}, ${s.jointWorld(j).y.toFixed(3)}) mid (${m.x.toFixed(3)}, ${m.y.toFixed(3)})`);
}
{
  // Anchored body (a drag): the joint comes to the midpoint, the body doesn't move.
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 70, y: 30 });
  s.addSketchConstraint("coincident", { kind: "joint", jointId: j.id }, mid(edge(b.id, 0)));
  check("anchored solve holds", solveSketch(s, new Set(anchorVarsForBody(s, b.id))).length === 0);
  check("joint lands at (50, 0)", nearPt(s.jointWorld(j), vec(50, 0), TOL * 10));
  check("anchored body untouched", nearPt(s.bodyControlWorld(b)[0], vec(0, 0)) && nearPt(s.bodyControlWorld(b)[1], vec(100, 0)));
}
{
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 300, y: 80 });
  s.addSketchConstraint("horizontal", { kind: "joint", jointId: j.id }, mid(edge(b.id, 1))); // right edge, mid y = 50
  check("horizontal to a midpoint solves", solveSketch(s, new Set(anchorVarsForBody(s, b.id))).length === 0);
  check("joint level with the midpoint, x kept", near(s.jointWorld(j).y, 50, TOL * 10) && near(s.jointWorld(j).x, 300));
  const k = s.addFreeJoint({ x: 10, y: -80 });
  s.addSketchConstraint("vertical", { kind: "joint", jointId: k.id }, mid(edge(b.id, 2))); // top edge, mid x = 50
  check("vertical to a midpoint solves", solveSketch(s, new Set(anchorVarsForBody(s, b.id))).length === 0);
  check("joint above the midpoint, y kept", near(s.jointWorld(k).x, 50, TOL * 10) && near(s.jointWorld(k).y, -80));
}
{
  // A rail's midpoint onto a corner: the rail (two free joints) slides so its middle meets it.
  const s = new Scene();
  const b = s.addBody(square());
  const ja = s.addFreeJoint({ x: 200, y: 300 });
  const jb = s.addFreeJoint({ x: 300, y: 340 });
  const rail = s.addSlider(ja.id, jb.id);
  s.addSketchConstraint("coincident", mid({ kind: "rail", sliderId: rail.id }), { kind: "vertex", bodyId: b.id, index: 2 });
  check("rail midpoint onto a corner solves", solveSketch(s, new Set(anchorVarsForBody(s, b.id))).length === 0);
  const rm = pointOf(s, mid({ kind: "rail", sliderId: rail.id }))!;
  check("rail's middle sits on the corner", nearPt(rm, vec(100, 100), TOL * 10));
  check("rail kept its length and direction", nearPt(vec(s.jointWorld(jb).x - s.jointWorld(ja).x, s.jointWorld(jb).y - s.jointWorld(ja).y), vec(100, 40), 1e-6));
}
{
  // A reference segment's midpoint tied to a joint: construction yields, the joint stays.
  const s = new Scene();
  const j = s.addFreeJoint({ x: 50, y: 50 });
  const g = s.addGuidePoly([{ x: 200, y: 0 }, { x: 300, y: 0 }], false)!;
  s.addSketchConstraint("coincident", mid({ kind: "guideLine", guideId: g.id, edge: 0 }), { kind: "joint", jointId: j.id });
  check("reference midpoint tie solves", solveSketch(s).length === 0);
  check("the joint stayed", nearPt(s.jointWorld(j), vec(50, 50)));
  check("the reference segment moved onto it", nearPt(pointOf(s, mid({ kind: "guideLine", guideId: g.id, edge: 0 }))!, vec(50, 50), TOL * 10));
}
{
  // A body's edge midpoint held on a rail's (infinite) line: point-on-line with a derived point.
  const s = new Scene();
  const b = s.addBody(square());
  const rail = s.addSlider(s.addFreeJoint({ x: -50, y: 130 }).id, s.addFreeJoint({ x: 200, y: 130 }).id);
  s.addSketchConstraint("coincident", mid(edge(b.id, 2)), { kind: "rail", sliderId: rail.id });
  check("midpoint on a line solves", solveSketch(s).length === 0);
  const m = pointOf(s, mid(edge(b.id, 2)))!;
  const r = s.resolveMeasureRef({ kind: "rail", sliderId: rail.id })!;
  const onLine = r.kind === "line" && near(m.y, r.a.y, TOL * 10) && near(r.a.y, r.b.y, TOL * 10);
  check("edge's middle lies on the rail line", onLine, `mid y ${m.y.toFixed(3)} rail y ${r.kind === "line" ? r.a.y.toFixed(3) : "?"}`);
}

// --- who moves: the push spreads over the ends by rank ------------------------------------
{
  // Joint locked (immovable), corner 0 held by the drag: only corner 1 may move, and it
  // must swing twice as far as the midpoint does.
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 60, y: 20 });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: j.id });
  s.addSketchConstraint("coincident", { kind: "joint", jointId: j.id }, mid(edge(b.id, 0)));
  const breaks = solveSketch(s, new Set([anchorVarForVertex(b.id, 0)]));
  check("locked joint + anchored corner: solves", breaks.length === 0);
  const v = s.bodyControlWorld(b);
  check("locked joint stayed", nearPt(s.jointWorld(j), vec(60, 20)));
  check("anchored corner stayed", nearPt(v[0], vec(0, 0), TOL * 10));
  check("free corner swung to 2·mid − v0", nearPt(v[1], vec(120, 40), TOL * 10), `v1 (${v[1].x.toFixed(3)}, ${v[1].y.toFixed(3)})`);
}
{
  // A locked midpoint: drag one end, the other mirrors it about the lock.
  const s = new Scene();
  const b = s.addBody(square());
  s.addSketchConstraint("fixed", mid(edge(b.id, 0))); // holds (50, 0)
  s.moveBodyVertex(b.id, 1, { x: 20, y: 30 }); // corner 1 → (120, 30)
  const breaks = solveSketch(s, new Set([anchorVarForVertex(b.id, 1)]));
  check("dragging an end of a locked-midpoint edge solves", breaks.length === 0);
  const v = s.bodyControlWorld(b);
  check("midpoint held", nearPt(pointOf(s, mid(edge(b.id, 0)))!, vec(50, 0), TOL * 10));
  check("dragged end kept", nearPt(v[1], vec(120, 30), TOL * 10));
  check("other end mirrored about the lock", nearPt(v[0], vec(-20, -30), TOL * 10), `v0 (${v[0].x.toFixed(3)}, ${v[0].y.toFixed(3)})`);
}

// --- regular polygons: a midpoint tie reads like a corner tie ------------------------------
{
  const s = new Scene();
  const hex = s.addBody(regularPolygon(vec(100, 100), vec(140, 100), 6), 0, "fillet", undefined, undefined, 6);
  const j1 = s.addFreeJoint(pointOf(s, mid(edge(hex.id, 0)))!);
  s.addSketchConstraint("coincident", mid(edge(hex.id, 0)), { kind: "joint", jointId: j1.id });
  check("one midpoint tie leaves the rotation free", !s.regularRotationLocked(hex.id, null));
  const j2 = s.addFreeJoint(s.bodyControlWorld(hex)[3]);
  s.addSketchConstraint("coincident", { kind: "vertex", bodyId: hex.id, index: 3 }, { kind: "joint", jointId: j2.id });
  check("a midpoint tie plus a corner tie lock it", s.regularRotationLocked(hex.id, null));
  check("the tied hexagon still solves", solveSketch(s).length === 0);
}

// --- remaps: a midpoint follows its edge ------------------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 50, y: 100 });
  const c = s.addSketchConstraint("coincident", { kind: "joint", jointId: j.id }, mid(edge(b.id, 2)))!; // top edge
  const ofIndex = () => (c.refB as { of: { index: number } }).of.index;

  s.insertBodyVertex(b.id, 1, { x: 50, y: -10 }); // a node on the bottom edge: later edges shift
  check("node insert shifts the midpoint's edge", ofIndex() === 3);
  check("it still names the top edge's middle", nearPt(pointOf(s, c.refB!)!, vec(50, 100)));
  s.removeBodyVertex(b.id, 1);
  check("node removal shifts it back", ofIndex() === 2);

  s.removeBodyVertex(b.id, 2); // the corner that starts the top edge: the edge is gone
  check("removing the edge's own corner drops the constraint", s.getSketchConstraint(c.id) === undefined);
}
{
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 50, y: 0 });
  const c = s.addSketchConstraint("coincident", { kind: "joint", jointId: j.id }, mid(edge(b.id, 0)))!; // bottom edge
  s.mirrorBody(b.id, "h");
  check("mirror keeps the constraint", s.getSketchConstraint(c.id) !== undefined);
  check("mirror remaps the midpoint to the same physical edge", nearPt(pointOf(s, c.refB!)!, vec(50, 0)), `now (${pointOf(s, c.refB!)?.x}, ${pointOf(s, c.refB!)?.y})`);
  check("mirrored scene still solves", solveSketch(s).length === 0);
}
{
  const s = new Scene();
  const b = s.addBody(square());
  const jl = s.addFreeJoint({ x: 0, y: 50 });
  const jt = s.addFreeJoint({ x: 50, y: 100 });
  const cLeft = s.addSketchConstraint("coincident", { kind: "joint", jointId: jl.id }, mid(edge(b.id, 3)))!; // left edge survives the cut
  const cTop = s.addSketchConstraint("coincident", { kind: "joint", jointId: jt.id }, mid(edge(b.id, 2)))!; // top edge is cut in two
  const r = s.splitBody(b.id, [vec(50, 0), vec(50, 100)]);
  check("split ok", r.ok);
  check("a midpoint on a surviving edge follows it", s.getSketchConstraint(cLeft.id) !== undefined && nearPt(pointOf(s, cLeft.refB!)!, vec(0, 50)));
  check("a midpoint on a severed edge is dropped", s.getSketchConstraint(cTop.id) === undefined);
}
{
  const s = new Scene();
  const b = s.addBody(square());
  const c = s.addSketchConstraint("coincident", mid(edge(b.id, 0)), { kind: "vertex", bodyId: b.id, index: 2 });
  check("internal midpoint constraint accepted", c !== null);
  const clip = s.extractSelection([b.id])!;
  const pasted = s.insertSelection(clip, { x: 500, y: 500 })!;
  const nb = s.getBody(pasted.bodyIds[0])!;
  const copy = s.sketch.find((k) => k.refA.kind === "midpoint" && refHost(k.refA).kind === "edge" && (refHost(k.refA) as { bodyId: number }).bodyId === nb.id);
  check("paste carries the midpoint constraint onto the copy", copy !== undefined);
  check("pasted midpoint resolves on the copy", copy !== undefined && nearPt(pointOf(s, copy.refA)!, vec(s.bodyControlWorld(nb)[0].x + 50, s.bodyControlWorld(nb)[0].y)));
}

// --- serialize / load ----------------------------------------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 50, y: 0 });
  s.addSketchConstraint("coincident", { kind: "joint", jointId: j.id }, mid(edge(b.id, 0)));
  s.addSketchConstraint("fixed", mid(edge(b.id, 2)));
  const t = new Scene();
  t.load(JSON.parse(JSON.stringify(s.serialize())));
  check("round trip keeps both midpoint constraints", t.sketch.length === 2);
  const c0 = t.sketch[0];
  check("nested ref survives", c0.refB?.kind === "midpoint" && c0.refB.of.kind === "edge" && nearPt(pointOf(t, c0.refB)!, vec(50, 0)));
  check("loaded midpoint ref is a fresh object", c0.refB !== s.sketch[0].refB && (c0.refB as { of: unknown }).of !== (s.sketch[0].refB as { of: unknown }).of);
  check("loaded scene solves", solveSketch(t).length === 0);
}

console.log(failures === 0 ? "\nmidpoint: all checks passed" : `\nmidpoint: ${failures} check(s) FAILED`);
if (failures > 0) process.exit(1);
