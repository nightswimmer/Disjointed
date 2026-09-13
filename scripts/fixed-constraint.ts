/**
 * The Fixed sketch constraint: a single reference nailed down where it is — a point
 * keeps its world position, a line is pinned whole (angle and position) with its ends
 * free only to slide along it and stretch it. Covers validation, the captured anchor,
 * who yields to a lock, the conflict reject, mirror re-capture and serialize/load.
 */
import { Scene, MeasureRef, SceneData, Vec2 } from "../src/model";
import { solveSketch, tryAddConstraint, applyDrivingDimension, anchorVarsForBody, sketchConfig } from "../src/sketch";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: number, b: number, eps = 5e-3) => Math.abs(a - b) < eps;
const TOL = sketchConfig.tol;
const square = (): Vec2[] => [
  { x: 0, y: 0 },
  { x: 100, y: 0 },
  { x: 100, y: 100 },
  { x: 0, y: 100 },
];

// --- validation ---------------------------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 200, y: 0 });
  const jointRef: MeasureRef = { kind: "joint", jointId: j.id };
  const vertexRef: MeasureRef = { kind: "vertex", bodyId: b.id, index: 0 };
  const edgeRef: MeasureRef = { kind: "edge", bodyId: b.id, index: 0 };
  const bodyPointRef: MeasureRef = { kind: "bodyPoint", bodyId: b.id, local: { x: 0, y: 0 } };

  check("fixed on a point ok", s.addSketchConstraint("fixed", jointRef) !== null);
  check("fixed on a line ok", s.addSketchConstraint("fixed", edgeRef) !== null);
  check("fixed on a vertex ok", s.addSketchConstraint("fixed", vertexRef) !== null);
  check("a second lock on the same element is rejected", s.addSketchConstraint("fixed", { kind: "joint", jointId: j.id }) === null);
  check("fixed with a second reference rejected", s.addSketchConstraint("fixed", { kind: "edge", bodyId: b.id, index: 1 }, vertexRef) === null);
  check("fixed on a bodyPoint rejected", s.addSketchConstraint("fixed", bodyPointRef) === null);
  check("fixed on an unresolvable ref rejected", s.addSketchConstraint("fixed", { kind: "joint", jointId: 9999 }) === null);

  const pt = s.sketch.find((c) => c.kind === "fixed" && c.refA.kind === "joint")!;
  const ln = s.sketch.find((c) => c.kind === "fixed" && c.refA.kind === "edge")!;
  check("a point lock captures a position, not an angle", !!pt.at && near(pt.at.x, 200) && near(pt.at.y, 0) && pt.angle === undefined);
  check(
    "a line lock captures the whole line: a point on it and an angle",
    ln.angle !== undefined && near(ln.angle, 0) && !!ln.at && near(ln.at.x, 50) && near(ln.at.y, 0),
    ln.at ? `(${ln.at.x}, ${ln.at.y}) @ ${ln.angle}` : "no anchor"
  );
}

// --- a locked point never moves; everything else yields to it -------------------
{
  const s = new Scene();
  const a = s.addFreeJoint({ x: 0, y: 0 });
  const b = s.addFreeJoint({ x: 100, y: 0 });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: a.id });
  const { constraint } = tryAddConstraint(s, "coincident", { kind: "joint", jointId: a.id }, { kind: "joint", jointId: b.id });
  check("a coincident onto a locked point is accepted", constraint !== null);
  const wa = s.jointWorld(s.getJoint(a.id)!);
  const wb = s.jointWorld(s.getJoint(b.id)!);
  check("…the locked point stayed put", near(wa.x, 0, TOL) && near(wa.y, 0, TOL), `(${wa.x.toFixed(3)}, ${wa.y.toFixed(3)})`);
  check("…the free point came to it", near(wb.x, 0, TOL) && near(wb.y, 0, TOL), `(${wb.x.toFixed(3)}, ${wb.y.toFixed(3)})`);
}

// --- a locked point pulls back against a move that isn't its own ---------------
{
  const s = new Scene();
  const j = s.addFreeJoint({ x: 40, y: 40 });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: j.id });
  s.moveJoint(j.id, { x: 25, y: -10 }); // as a drag would leave it
  check("a locked joint's solve converges", solveSketch(s).length === 0);
  const w = s.jointWorld(s.getJoint(j.id)!);
  check("…and puts it back on its anchor", near(w.x, 40, TOL) && near(w.y, 40, TOL), `(${w.x.toFixed(3)}, ${w.y.toFixed(3)})`);
}
{
  const s = new Scene();
  const b = s.addBody(square());
  s.addSketchConstraint("fixed", { kind: "vertex", bodyId: b.id, index: 0 });
  const before = s.bodyControlWorld(b)[0];
  s.moveBody(b.id, { x: 60, y: 15 });
  // The drag anchors the whole body — the lock outranks even that.
  check("a body drag past a locked corner converges", solveSketch(s, new Set(anchorVarsForBody(s, b.id))).length === 0);
  const after = s.bodyControlWorld(b)[0];
  check(
    "…the locked corner is back where it was",
    near(after.x, before.x, TOL) && near(after.y, before.y, TOL),
    `(${after.x.toFixed(3)}, ${after.y.toFixed(3)}) vs (${before.x.toFixed(3)}, ${before.y.toFixed(3)})`
  );
  const v2 = s.bodyControlWorld(b)[2];
  check("…while the rest of the body did move", !near(v2.x, 100, 1) || !near(v2.y, 100, 1), `(${v2.x.toFixed(2)}, ${v2.y.toFixed(2)})`);
}

// --- a locked line holds the line; its ends only slide along it ------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  s.addSketchConstraint("fixed", { kind: "edge", bodyId: b.id, index: 0 }); // v0 → v1, the y = 0 line
  // Sliding an end along the locked line is free: nothing should move.
  s.moveBodyVertex(b.id, 1, { x: 50, y: 0 }, null);
  check("stretching a locked line along itself converges", solveSketch(s).length === 0);
  let c = s.bodyControlWorld(b);
  check("…and is left alone", near(c[1].x, 150, TOL) && near(c[1].y, 0, TOL), `(${c[1].x.toFixed(3)}, ${c[1].y.toFixed(3)})`);
  // Taking an end off the line is not: only that end is put back, along the normal —
  // its slide along the line survives, and the other end is not swung to compensate.
  s.moveBodyVertex(b.id, 1, { x: -30, y: 60 }, null);
  check("taking an end off a locked line converges", solveSketch(s).length === 0);
  c = s.bodyControlWorld(b);
  check("…the moved end drops back onto the line, keeping its slide", near(c[1].x, 120, TOL) && near(c[1].y, 0, TOL), `(${c[1].x.toFixed(3)}, ${c[1].y.toFixed(3)})`);
  check("…and the other end never moved", near(c[0].x, 0, TOL) && near(c[0].y, 0, TOL), `(${c[0].x.toFixed(3)}, ${c[0].y.toFixed(3)})`);
  // A move of the whole body is undone on the locked edge — the line cannot shift.
  s.moveBody(b.id, { x: 12, y: 25 });
  check("shifting a locked line converges", solveSketch(s).length === 0);
  c = s.bodyControlWorld(b);
  check("…both ends are back on the line", near(c[0].y, 0, TOL) && near(c[1].y, 0, TOL), `y ${c[0].y.toFixed(3)} / ${c[1].y.toFixed(3)}`);
  check("…having slid along it, not back to where they were", near(c[0].x, 12, TOL) && near(c[1].x, 132, TOL), `x ${c[0].x.toFixed(3)} / ${c[1].x.toFixed(3)}`);
}

// --- two locks that disagree are rejected --------------------------------------
{
  const s = new Scene();
  const a = s.addFreeJoint({ x: 0, y: 0 });
  const b = s.addFreeJoint({ x: 100, y: 0 });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: a.id });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: b.id });
  const { constraint, breaks } = tryAddConstraint(s, "coincident", { kind: "joint", jointId: a.id }, { kind: "joint", jointId: b.id });
  check("a coincident between two locked points is refused", constraint === null && breaks.length > 0, `${breaks.length} break(s)`);
  const wa = s.jointWorld(s.getJoint(a.id)!);
  const wb = s.jointWorld(s.getJoint(b.id)!);
  check("…geometry untouched", near(wa.x, 0) && near(wb.x, 100), `${wa.x.toFixed(3)} / ${wb.x.toFixed(3)}`);
}
{
  // A driving dimension that would have to move a locked point can't be applied.
  const s = new Scene();
  const a = s.addFreeJoint({ x: 0, y: 0 });
  const b = s.addFreeJoint({ x: 100, y: 0 });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: a.id });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: b.id });
  const m = s.addMeasurement("draw", { kind: "joint", jointId: a.id }, { kind: "joint", jointId: b.id }, { x: 50, y: -20 })!;
  const breaks = applyDrivingDimension(s, m.id, 160);
  check("a driving dimension across two locked points is refused", breaks.length > 0, `${breaks.length} break(s)`);
  check("…the dimension stayed driven", s.getMeasurement(m.id)!.driving !== true);
  check("…geometry untouched", near(s.jointWorld(s.getJoint(b.id)!).x, 100));
}
{
  // A point locked off the line its own locked edge lies on can never be satisfied.
  const s = new Scene();
  const b = s.addBody(square());
  s.addSketchConstraint("fixed", { kind: "edge", bodyId: b.id, index: 0 }); // the y = 0 line
  s.moveBodyVertex(b.id, 1, { x: 0, y: 40 }, null); // take v1 off it, then lock it there
  s.addSketchConstraint("fixed", { kind: "vertex", bodyId: b.id, index: 1 });
  const breaks = solveSketch(s);
  check("an end locked off its own locked line is unsatisfiable", breaks.length > 0, `${breaks.length} break(s)`);
  check("…and the scene is left untouched", near(s.bodyControlWorld(b)[1].y, 40, TOL));
}

// --- mirror re-anchors the lock -------------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  s.addSketchConstraint("fixed", { kind: "vertex", bodyId: b.id, index: 0 });
  s.addSketchConstraint("fixed", { kind: "edge", bodyId: b.id, index: 2 });
  s.mirrorBody(b.id, "h");
  check("a mirrored body's locks still solve", solveSketch(s).length === 0);
  const pt = s.sketch.find((c) => c.kind === "fixed" && c.refA.kind === "vertex")!;
  const w = s.resolveMeasureRef(pt.refA)!;
  check(
    "…the point lock re-anchored onto the reflected corner",
    w.kind === "point" && near(w.p.x, pt.at!.x, TOL) && near(w.p.y, pt.at!.y, TOL),
    w.kind === "point" ? `(${w.p.x.toFixed(3)}, ${w.p.y.toFixed(3)})` : "not a point"
  );
}

// --- serialize / load -----------------------------------------------------------
{
  const s = new Scene();
  const b = s.addBody(square());
  const j = s.addFreeJoint({ x: 33, y: -12 });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: j.id });
  s.addSketchConstraint("fixed", { kind: "edge", bodyId: b.id, index: 1 }); // the x = 100 line
  const data = JSON.parse(JSON.stringify(s.serialize())) as SceneData;
  const t = new Scene();
  t.load(data);
  const pt = t.sketch.find((c) => c.kind === "fixed" && c.refA.kind === "joint")!;
  const ln = t.sketch.find((c) => c.kind === "fixed" && c.refA.kind === "edge")!;
  check("a point lock round-trips its anchor", !!pt?.at && near(pt.at.x, 33) && near(pt.at.y, -12));
  check(
    "a line lock round-trips its line",
    ln?.angle !== undefined && near(Math.abs(ln.angle), Math.PI / 2) && !!ln.at && near(ln.at.x, 100) && near(ln.at.y, 50),
    ln?.at ? `(${ln.at.x}, ${ln.at.y})` : "no anchor"
  );
  check("a loaded scene's locks solve clean", solveSketch(t).length === 0);
  // The clone must not share the anchor object with the data it came from.
  pt.at!.x = 999;
  check("the loaded anchor is a copy", data.sketch!.find((c) => c.kind === "fixed" && c.refA.kind === "joint")!.at!.x === 33);
}

// --- cascade removal -------------------------------------------------------------
{
  const s = new Scene();
  const j = s.addFreeJoint({ x: 10, y: 10 });
  s.addSketchConstraint("fixed", { kind: "joint", jointId: j.id });
  s.removeJoint(j.id);
  check("deleting the locked element drops its lock", s.sketch.length === 0);
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll fixed-constraint checks passed.");
