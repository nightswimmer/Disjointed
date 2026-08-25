/**
 * Sketch constraints + driving dimensions: constraint validation, the Gauss-Seidel
 * sketch solve (coincident / horizontal / vertical / parallel / perpendicular / equal),
 * driving-dimension edits (scale-on-first-dimension vs move-involved-nodes), reject
 * semantics on conflicts, cascade removal / index remapping, and serialize/load (v8).
 */
import { Scene, MeasureRef, SceneData } from "../src/model";
import {
  solveSketch,
  applyDrivingDimension,
  tryAddConstraint,
  autoConstrainBody,
  sketchConfig,
} from "../src/sketch";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: number, b: number, eps = 5e-3) => Math.abs(a - b) < eps;
const TOL = sketchConfig.tol;

// --- constraint validation ----------------------------------------------------
{
  const s = new Scene();
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ]);
  const j = s.addFreeJoint({ x: 200, y: 0 });
  const jointRef: MeasureRef = { kind: "joint", jointId: j.id };
  const vertexRef: MeasureRef = { kind: "vertex", bodyId: body.id, index: 0 };
  const edgeRef: MeasureRef = { kind: "edge", bodyId: body.id, index: 0 };
  const bodyPointRef: MeasureRef = { kind: "bodyPoint", bodyId: body.id, local: { x: 0, y: 0 } };

  check("coincident point+point ok", s.addSketchConstraint("coincident", jointRef, vertexRef) !== null);
  check("coincident with a line ref rejected", s.addSketchConstraint("coincident", jointRef, edgeRef) === null);
  check("coincident with a bodyPoint rejected", s.addSketchConstraint("coincident", jointRef, bodyPointRef) === null);
  check("coincident same element rejected", s.addSketchConstraint("coincident", jointRef, { kind: "joint", jointId: j.id }) === null);
  check("horizontal on a line ok", s.addSketchConstraint("horizontal", edgeRef) !== null);
  check("horizontal on a point pair ok", s.addSketchConstraint("horizontal", jointRef, vertexRef) !== null);
  check("horizontal on a bare point rejected", s.addSketchConstraint("horizontal", jointRef) === null);
  check("parallel needs two lines", s.addSketchConstraint("parallel", edgeRef, jointRef) === null);
  check("equal line+line ok", s.addSketchConstraint("equal", edgeRef, { kind: "edge", bodyId: body.id, index: 1 }) !== null);
  check("unresolvable ref rejected", s.addSketchConstraint("coincident", jointRef, { kind: "joint", jointId: 9999 }) === null);
}

// --- horizontal / vertical / coincident (points) -------------------------------
{
  const s = new Scene();
  const a = s.addFreeJoint({ x: 0, y: 0 });
  const b = s.addFreeJoint({ x: 100, y: 30 });
  s.addSketchConstraint("horizontal", { kind: "joint", jointId: a.id }, { kind: "joint", jointId: b.id });
  check("H solve leaves scene satisfied", solveSketch(s).length === 0);
  check(
    "H levels the pair at the mean y",
    near(s.jointWorld(a).y, 15) && near(s.jointWorld(b).y, 15),
    `${s.jointWorld(a).y}, ${s.jointWorld(b).y}`
  );

  // Violate it, re-solve: the constraint re-levels from wherever the points are.
  s.moveJoint(b.id, { x: 0, y: 30 }); // b.y → 45
  check("re-solve after violation", solveSketch(s).length === 0);
  check("H holds again", near(s.jointWorld(a).y, s.jointWorld(b).y, TOL * 2));

  const c = s.addFreeJoint({ x: 50, y: 0 });
  const d = s.addFreeJoint({ x: 58, y: 90 });
  s.addSketchConstraint("vertical", { kind: "joint", jointId: c.id }, { kind: "joint", jointId: d.id });
  s.addSketchConstraint("coincident", { kind: "joint", jointId: a.id }, { kind: "joint", jointId: c.id });
  check("V + coincident solve", solveSketch(s).length === 0);
  check("V aligns x", near(s.jointWorld(c).x, s.jointWorld(d).x, TOL * 2));
  const wa = s.jointWorld(a);
  const wc = s.jointWorld(c);
  check("coincident points meet", near(wa.x, wc.x, TOL * 2) && near(wa.y, wc.y, TOL * 2));
}

// --- horizontal on an edge reshapes the body, joints stay anchored -------------
{
  const s = new Scene();
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 20 },
    { x: 110, y: 100 },
    { x: -10, y: 90 },
  ]);
  const j = s.addJoint(body.id, { x: 50, y: 60 });
  const before = s.jointWorld(j);
  s.addSketchConstraint("horizontal", { kind: "edge", bodyId: body.id, index: 0 });
  check("edge H solve", solveSketch(s).length === 0);
  const v = s.bodyControlWorld(body);
  check("edge is horizontal", near(v[0].y, v[1].y, TOL * 2), `${v[0].y} vs ${v[1].y}`);
  check("edge levels at the mean", near(v[0].y, 10, TOL * 2) && near(v[0].x, 0) && near(v[1].x, 100));
  check("other vertices untouched", near(v[2].y, 100) && near(v[3].y, 90));
  const after = s.jointWorld(j);
  check("attached joint stays anchored", near(after.x, before.x) && near(after.y, before.y));
}

// --- a joint linked to a control vertex maps onto the vertex variable ----------
{
  const s = new Scene();
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 50, y: 80 },
  ]);
  const v0 = s.bodyControlWorld(body)[0];
  const linked = s.addJoint(body.id, { x: v0.x, y: v0.y }); // exactly on vertex 0 → stuck
  const free = s.addFreeJoint({ x: 0, y: 40 });
  s.addSketchConstraint("horizontal", { kind: "joint", jointId: linked.id }, { kind: "joint", jointId: free.id });
  check("linked-joint solve", solveSketch(s).length === 0);
  const w = s.jointWorld(linked);
  check("constraint on the joint drives the vertex", near(w.y, 20, TOL * 2), `${w.y}`);
  const nv0 = s.bodyControlWorld(body)[0];
  check("joint still stuck to the vertex", near(nv0.x, w.x, 1e-6) && near(nv0.y, w.y, 1e-6));
}

// --- parallel / perpendicular / equal ------------------------------------------
{
  const s = new Scene();
  const r1 = s.addSlider(s.addFreeJoint({ x: 0, y: 0 }).id, s.addFreeJoint({ x: 100, y: 0 }).id);
  const j2a = s.addFreeJoint({ x: 0, y: 50 });
  const j2b = s.addFreeJoint({ x: 80, y: 110 });
  const r2 = s.addSlider(j2a.id, j2b.id);
  s.addSketchConstraint("parallel", { kind: "rail", sliderId: r1.id }, { kind: "rail", sliderId: r2.id });
  check("parallel solve", solveSketch(s).length === 0);
  const a2 = s.jointWorld(j2a);
  const b2 = s.jointWorld(j2b);
  const ang2 = Math.atan2(b2.y - a2.y, b2.x - a2.x);
  const cr1 = s.constraints.find((c) => c.kind === "slider" && c.id === r1.id) as {
    railA: number;
    railB: number;
  };
  const r1a = s.jointWorld(s.getJoint(cr1.railA)!);
  const r1b = s.jointWorld(s.getJoint(cr1.railB)!);
  const ang1 = Math.atan2(r1b.y - r1a.y, r1b.x - r1a.x);
  const diff = Math.abs(Math.sin(ang2 - ang1));
  check("rails are parallel", diff < 1e-4, `sin=${diff}`);
  check("rail 2 keeps its length", near(Math.hypot(b2.x - a2.x, b2.y - a2.y), 100, 1e-2));
  check("rail 2 keeps its midpoint", near((a2.x + b2.x) / 2, 40) && near((a2.y + b2.y) / 2, 80));

  // Perpendicular between two edges of one body (they share a vertex).
  const t = new Scene();
  const body = t.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 10 },
    { x: 90, y: 100 },
    { x: -10, y: 90 },
  ]);
  t.addSketchConstraint(
    "perpendicular",
    { kind: "edge", bodyId: body.id, index: 0 },
    { kind: "edge", bodyId: body.id, index: 1 }
  );
  check("perpendicular solve", solveSketch(t).length === 0);
  const v = t.bodyControlWorld(body);
  const e0 = { x: v[1].x - v[0].x, y: v[1].y - v[0].y };
  const e1 = { x: v[2].x - v[1].x, y: v[2].y - v[1].y };
  const dp = (e0.x * e1.x + e0.y * e1.y) / (Math.hypot(e0.x, e0.y) * Math.hypot(e1.x, e1.y));
  check("edges are perpendicular", Math.abs(dp) < 1e-4, `cos=${dp}`);

  // Equal rail lengths.
  const u = new Scene();
  const s1 = u.addSlider(u.addFreeJoint({ x: 0, y: 0 }).id, u.addFreeJoint({ x: 100, y: 0 }).id);
  const s2 = u.addSlider(u.addFreeJoint({ x: 0, y: 50 }).id, u.addFreeJoint({ x: 60, y: 50 }).id);
  u.addSketchConstraint("equal", { kind: "rail", sliderId: s1.id }, { kind: "rail", sliderId: s2.id });
  check("equal solve", solveSketch(u).length === 0);
  const lens = u.constraints
    .filter((c) => c.kind === "slider")
    .map((c) => {
      const sc = c as { railA: number; railB: number };
      const a = u.jointWorld(u.getJoint(sc.railA)!);
      const b = u.jointWorld(u.getJoint(sc.railB)!);
      return Math.hypot(b.x - a.x, b.y - a.y);
    });
  check("lengths equalize at the mean", near(lens[0], 80, 1e-2) && near(lens[1], 80, 1e-2), `${lens}`);
}

// --- driving dimensions: node path (free points) --------------------------------
{
  const s = new Scene();
  const a = s.addFreeJoint({ x: 0, y: 0 });
  const b = s.addFreeJoint({ x: 30, y: 40 });
  const c = s.addFreeJoint({ x: 999, y: 999 });
  const m = s.addMeasurement(
    "draw",
    { kind: "joint", jointId: a.id },
    { kind: "joint", jointId: b.id },
    { x: -50, y: 50 } // diagonal zone → direct
  )!;
  check("direct drive succeeds", applyDrivingDimension(s, m.id, 100).length === 0);
  check("dimension is now driving", m.driving === true && m.target === 100);
  check("distance hits the target", near(s.measureInfo(m)!.value, 100, TOL * 2));
  const wa = s.jointWorld(a);
  const wb = s.jointWorld(b);
  check("pair midpoint preserved", near((wa.x + wb.x) / 2, 15) && near((wa.y + wb.y) / 2, 20));
  const wc = s.jointWorld(c);
  check("uninvolved node untouched", wc.x === 999 && wc.y === 999);

  // Horizontal-axis dimension: only the x's move.
  const t = new Scene();
  const p = t.addFreeJoint({ x: 0, y: 0 });
  const q = t.addFreeJoint({ x: 60, y: 80 });
  const mh = t.addMeasurement(
    "draw",
    { kind: "joint", jointId: p.id },
    { kind: "joint", jointId: q.id },
    { x: 30, y: -50 } // above the pair → h
  )!;
  check("h-axis drive succeeds", applyDrivingDimension(t, mh.id, 100).length === 0);
  const wp = t.jointWorld(p);
  const wq = t.jointWorld(q);
  check("|Δx| hits the target", near(Math.abs(wq.x - wp.x), 100, TOL * 2));
  check("y's unchanged by an h dimension", wp.y === 0 && wq.y === 80);
}

// --- scale-on-first-dimension -----------------------------------------------------
{
  const s = new Scene();
  const body = s.addBody(
    [
      { x: 0, y: 0 },
      { x: 100, y: 0 },
      { x: 100, y: 100 },
      { x: 0, y: 100 },
    ],
    10 // fillet radius, to check it scales too
  );
  const j = s.addJoint(body.id, { x: 75, y: 50 });
  s.addGround(j.id, s.jointWorld(j));
  const mRef = s.addMeasurement(
    "draw",
    { kind: "bodyPoint", bodyId: body.id, local: { x: 25, y: 0 } }, // world (75, 50)
    { kind: "vertex", bodyId: body.id, index: 0 },
    { x: 40, y: 30 }
  )!;
  const refBefore = s.measureInfo(mRef)!.value;
  const m = s.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body.id, index: 0 },
    { kind: "vertex", bodyId: body.id, index: 2 },
    { x: 250, y: 250 } // diagonal zone → direct
  )!;
  const diag = s.measureInfo(m)!.value; // √2·100
  check("scale drive succeeds", applyDrivingDimension(s, m.id, diag * 2).length === 0);
  check("dimension hits 2× the diagonal", near(s.measureInfo(m)!.value, diag * 2, 1e-6));
  check("centroid stays put", near(body.pos.x, 50, 1e-6) && near(body.pos.y, 50, 1e-6));
  const v = s.bodyControlWorld(body);
  check("vertices scaled about the centroid", near(v[0].x, -50, 1e-6) && near(v[0].y, -50, 1e-6) && near(v[2].x, 150, 1e-6));
  check("corner radius scaled", near(body.radius, 20, 1e-9), `${body.radius}`);
  const wj = s.jointWorld(j);
  check("attached joint scaled with the body", near(wj.x, 100, 1e-6) && near(wj.y, 50, 1e-6));
  const g = s.constraints.find((c) => c.kind === "ground")!;
  check("ground anchor followed", near((g as { anchor: { x: number } }).anchor.x, 100, 1e-6));
  check("bodyPoint measurement scaled too", near(s.measureInfo(mRef)!.value, refBefore * 2, 1e-6));

  // A second dimension on the same body takes the node path: the first still holds.
  const m2 = s.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body.id, index: 0 },
    { kind: "vertex", bodyId: body.id, index: 1 },
    { x: 50, y: -200 } // above → h... points share y, so this is h of a horizontal edge
  )!;
  const v01 = s.measureInfo(m2)!.value; // 200 after the scale
  check("second dim drive succeeds", applyDrivingDimension(s, m2.id, v01 * 0.9).length === 0);
  check("second dim hits its target", near(s.measureInfo(m2)!.value, v01 * 0.9, TOL * 2));
  check("first (driving) dim still holds", near(s.measureInfo(m)!.value, diag * 2, TOL * 2));
  const v2 = s.bodyControlWorld(body);
  check("node path, not a rescale: vertex 3 untouched", near(v2[3].x, -50, TOL * 2) && near(v2[3].y, 150, TOL * 2));
}

// --- an external constraint disables the scale path --------------------------------
{
  const s = new Scene();
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ]);
  const ext = s.addFreeJoint({ x: 0, y: 0 }); // sits on vertex 0
  s.addSketchConstraint(
    "coincident",
    { kind: "joint", jointId: ext.id },
    { kind: "vertex", bodyId: body.id, index: 0 }
  );
  const m = s.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body.id, index: 0 },
    { kind: "vertex", bodyId: body.id, index: 2 },
    { x: 250, y: 250 }
  )!;
  check("drive with external constraint succeeds", applyDrivingDimension(s, m.id, 200).length === 0);
  check("dimension hits the target", near(s.measureInfo(m)!.value, 200, TOL * 2));
  const v = s.bodyControlWorld(body);
  check("no uniform scale: vertex 1 untouched", near(v[1].x, 100, TOL * 2) && near(v[1].y, 0, TOL * 2));
  const we = s.jointWorld(ext);
  check("external coincident held through the drive", near(we.x, v[0].x, TOL * 2) && near(we.y, v[0].y, TOL * 2));
}

// --- constraints + dimensions together: a parametric rectangle ----------------------
{
  const s = new Scene();
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 60 },
    { x: 0, y: 60 },
  ]);
  const e = (i: number): MeasureRef => ({ kind: "edge", bodyId: body.id, index: i });
  s.addSketchConstraint("horizontal", e(0));
  s.addSketchConstraint("horizontal", e(2));
  s.addSketchConstraint("vertical", e(1));
  s.addSketchConstraint("vertical", e(3));
  const width = s.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body.id, index: 0 },
    { kind: "vertex", bodyId: body.id, index: 1 },
    { x: 300, y: 300 } // diagonal zone → direct
  )!;
  // First dimension on the body (constraints are all internal) → uniform scale.
  check("width drive (first dim) succeeds", applyDrivingDimension(s, width.id, 150).length === 0);
  let v = s.bodyControlWorld(body);
  check("first dim scaled uniformly: height went to 90", near(Math.abs(v[2].y - v[1].y), 90, 1e-6));
  const height = s.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body.id, index: 1 },
    { kind: "vertex", bodyId: body.id, index: 2 },
    { x: 300, y: 100 } // diagonal-ish → direct for a vertical pair placed beside
  )!;
  check("height drive (second dim) succeeds", applyDrivingDimension(s, height.id, 60).length === 0);
  v = s.bodyControlWorld(body);
  check("width still 150", near(s.measureInfo(width)!.value, 150, TOL * 2));
  check("height now 60", near(s.measureInfo(height)!.value, 60, TOL * 2));
  check(
    "still a rectangle (H/V held)",
    near(v[0].y, v[1].y, TOL * 2) && near(v[2].y, v[3].y, TOL * 2) &&
      near(v[1].x, v[2].x, TOL * 2) && near(v[0].x, v[3].x, TOL * 2)
  );
}

// --- point-line and line-line driving dimensions -------------------------------------
{
  const s = new Scene();
  const rail = s.addSlider(s.addFreeJoint({ x: 0, y: 0 }).id, s.addFreeJoint({ x: 100, y: 0 }).id);
  const p = s.addFreeJoint({ x: 50, y: 40 });
  const m = s.addMeasurement(
    "draw",
    { kind: "rail", sliderId: rail.id },
    { kind: "joint", jointId: p.id },
    { x: 55, y: 20 }
  )!;
  check("point-line drive succeeds", applyDrivingDimension(s, m.id, 80).length === 0);
  check("perpendicular distance hits the target", near(s.measureInfo(m)!.value, 80, TOL * 2));
  const anchors = s.constraints.filter((c) => c.kind === "ground").map((c) => (c as { anchor: { y: number } }).anchor.y);
  check("rail moved and its ground anchors followed", anchors.every((y) => near(y, -20, TOL * 2)), `${anchors}`);

  const t = new Scene();
  const r1 = t.addSlider(t.addFreeJoint({ x: 0, y: 0 }).id, t.addFreeJoint({ x: 100, y: 0 }).id);
  const r2 = t.addSlider(t.addFreeJoint({ x: 0, y: 50 }).id, t.addFreeJoint({ x: 100, y: 50 }).id);
  const mm = t.addMeasurement(
    "draw",
    { kind: "rail", sliderId: r1.id },
    { kind: "rail", sliderId: r2.id },
    { x: 50, y: 25 }
  )!;
  check("line-line drive succeeds", applyDrivingDimension(t, mm.id, 120).length === 0);
  const after = t.measureInfo(mm)!;
  check("gap hits the target and stays a distance", after.kind === "distance" && near(after.value, 120, TOL * 2), `${after.value}`);
}

// --- rejects: conflicts, angles, bad input --------------------------------------------
{
  const s = new Scene();
  const a = s.addFreeJoint({ x: 0, y: 0 });
  const b = s.addFreeJoint({ x: 100, y: 0 });
  const direct = s.addMeasurement(
    "draw",
    { kind: "joint", jointId: a.id },
    { kind: "joint", jointId: b.id },
    { x: 150, y: 150 }
  )!;
  check("first driving dim ok", applyDrivingDimension(s, direct.id, 100).length === 0);
  const h = s.addMeasurement(
    "draw",
    { kind: "joint", jointId: a.id },
    { kind: "joint", jointId: b.id },
    { x: 50, y: -60 } // above → h
  )!;
  const before = JSON.stringify(s.serialize());
  const breaks = applyDrivingDimension(s, h.id, 200); // |Δx|=200 with |ab|=100: impossible
  check("conflicting dim rejected", breaks.length > 0);
  check("reject leaves the scene untouched", JSON.stringify(s.serialize()) === before);
  check("rejected dim is not driving", !h.driving);

  // An angle dimension can't drive (v1: distances only).
  const t = new Scene();
  const body = t.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ]);
  const ma = t.addMeasurement(
    "draw",
    { kind: "edge", bodyId: body.id, index: 0 },
    { kind: "edge", bodyId: body.id, index: 1 },
    { x: 50, y: 50 }
  )!;
  check("angle kind detected", t.measureInfo(ma)!.kind === "angle");
  check("angle dimension can't drive", applyDrivingDimension(t, ma.id, 45).length > 0);
  check("sim-mode dim can't drive", (() => {
    const msim = t.addMeasurement("sim", { kind: "vertex", bodyId: body.id, index: 0 }, { kind: "vertex", bodyId: body.id, index: 1 }, { x: 0, y: -50 })!;
    return applyDrivingDimension(t, msim.id, 50).length > 0 && !msim.driving;
  })());
  check("non-positive target rejected", applyDrivingDimension(s, direct.id, 0).length > 0);
  check("driving flag survives a failed re-drive", direct.driving === true && direct.target === 100);
}

// --- cascade removal + index remapping --------------------------------------------------
{
  const s = new Scene();
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ]);
  const j = s.addFreeJoint({ x: 200, y: 0 });
  const rail = s.addSlider(s.addFreeJoint({ x: 0, y: 200 }).id, s.addFreeJoint({ x: 100, y: 200 }).id);
  const cJoint = s.addSketchConstraint("coincident", { kind: "joint", jointId: j.id }, { kind: "vertex", bodyId: body.id, index: 0 })!;
  const cEdge = s.addSketchConstraint("horizontal", { kind: "edge", bodyId: body.id, index: 2 })!;
  const cRail = s.addSketchConstraint("parallel", { kind: "rail", sliderId: rail.id }, { kind: "edge", bodyId: body.id, index: 0 })!;

  s.insertBodyVertex(body.id, 1, { x: 50, y: -10 });
  check("insert shifts sketch edge refs", (cEdge.refA as { index: number }).index === 3);
  s.removeBodyVertex(body.id, 1);
  check("remove shifts them back", (cEdge.refA as { index: number }).index === 2);

  s.removeJoint(j.id);
  check("removing a joint prunes its constraints", s.getSketchConstraint(cJoint.id) === undefined);
  s.removeConstraint(rail.id);
  check("removing a slider prunes rail constraints", s.getSketchConstraint(cRail.id) === undefined);
  check("unrelated constraint survives", s.getSketchConstraint(cEdge.id) !== undefined);
  s.removeBody(body.id);
  check("removing the body prunes the rest", s.sketch.length === 0);
}

// --- serialize / load (v8) ----------------------------------------------------------------
{
  const s = new Scene();
  const a = s.addFreeJoint({ x: 0, y: 0 });
  const b = s.addFreeJoint({ x: 100, y: 0 });
  const body = s.addBody([
    { x: 0, y: 50 },
    { x: 100, y: 50 },
    { x: 50, y: 120 },
  ]);
  const cc = s.addSketchConstraint("coincident", { kind: "joint", jointId: a.id }, { kind: "vertex", bodyId: body.id, index: 0 })!;
  const ch = s.addSketchConstraint("horizontal", { kind: "edge", bodyId: body.id, index: 0 })!;
  const m = s.addMeasurement("draw", { kind: "joint", jointId: a.id }, { kind: "joint", jointId: b.id }, { x: 50, y: -40 })!;
  applyDrivingDimension(s, m.id, 100);

  const text = JSON.stringify(s.serialize());
  const t = new Scene();
  t.load(JSON.parse(text));
  check("sketch constraints round-trip", t.sketch.length === 2, `${t.sketch.length}`);
  check("kinds + single-ref form preserved", t.getSketchConstraint(cc.id)?.kind === "coincident" && t.getSketchConstraint(ch.id)?.refB === null);
  const tm = t.getMeasurement(m.id)!;
  check("driving flag + target round-trip", tm.driving === true && tm.target === 100);
  check("driving flag reaches MeasureInfo", t.measureInfo(tm)!.driving === true);

  const maxId = Math.max(...t.sketch.map((c) => c.id), ...t.joints.map((j) => j.id), ...t.measurements.map((x) => x.id));
  const fresh = t.addSketchConstraint("horizontal", { kind: "edge", bodyId: body.id, index: 1 })!;
  check("nextId continues past sketch ids", fresh.id > maxId, `${fresh.id} > ${maxId}`);

  (t.getSketchConstraint(cc.id)!.refA as { jointId: number }).jointId = 424242;
  check("load deep-copies sketch refs", (s.getSketchConstraint(cc.id)!.refA as { jointId: number }).jointId === a.id);

  // Pre-v8 file: no sketch field, no driving flags.
  const legacy = JSON.parse(text) as SceneData;
  delete legacy.sketch;
  for (const lm of legacy.measurements!) {
    delete lm.driving;
    delete lm.target;
  }
  const u = new Scene();
  u.load(legacy);
  check("pre-v8 file loads with no sketch constraints", u.sketch.length === 0);
  check("pre-v8 dimensions load as driven", u.measurements.every((x) => !x.driving));
}

// --- tryAddConstraint (add + solve, reject-and-remove on conflict) ----------------------
{
  const s = new Scene();
  const a = s.addFreeJoint({ x: 0, y: 0 });
  const b = s.addFreeJoint({ x: 100, y: 4 });
  const ra: MeasureRef = { kind: "joint", jointId: a.id };
  const rb: MeasureRef = { kind: "joint", jointId: b.id };
  const ok = tryAddConstraint(s, "horizontal", ra, rb);
  check("tryAddConstraint solves in the new constraint", ok.constraint !== null && ok.breaks.length === 0);
  check("geometry satisfied it", near(s.jointWorld(a).y, s.jointWorld(b).y, TOL * 2));

  // A vertical on the same (now-horizontal, well-separated) pair conflicts with a
  // driving h dimension holding |Δx| — the add is rejected and rolled back.
  const m = s.addMeasurement("draw", ra, rb, { x: 50, y: -60 })!;
  applyDrivingDimension(s, m.id, 100);
  const before = JSON.stringify(s.serialize());
  const bad = tryAddConstraint(s, "vertical", ra, rb);
  check("conflicting constraint rejected", bad.constraint === null && bad.breaks.length > 0);
  check("rejected add leaves the scene untouched", JSON.stringify(s.serialize()) === before);
  check("invalid refs: null constraint, no breaks", tryAddConstraint(s, "parallel", ra, rb).constraint === null);
}

// --- autoConstrainBody (H/V inference on a freehand polygon) ----------------------------
{
  const s = new Scene();
  // Top edge ~2° off horizontal, right edge ~2° off vertical, the rest well diagonal.
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 3.5 }, // edge 0: near-horizontal
    { x: 103, y: 103 }, // edge 1: near-vertical
    { x: 30, y: 140 },  // edges 2 and 3: diagonal
  ]);
  const made = autoConstrainBody(s, body.id);
  check("auto-constraints: one H + one V inferred", made.length === 2 &&
    made.some((c) => c.kind === "horizontal") && made.some((c) => c.kind === "vertical"),
    made.map((c) => c.kind).join(","));
  const v = s.bodyControlWorld(body);
  check("near-horizontal edge snapped level", near(v[0].y, v[1].y, TOL * 2));
  check("near-vertical edge snapped plumb", near(v[1].x, v[2].x, TOL * 2));
  const d2 = Math.abs(v[3].y - v[2].y) > 5 && Math.abs(v[3].x - v[2].x) > 5;
  check("diagonal edges left alone", d2);
  check("constraints registered on the scene", s.sketch.length === 2);
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll sketch checks passed.");
