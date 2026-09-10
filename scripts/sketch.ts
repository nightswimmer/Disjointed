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
  anchorVarsForBody,
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
  check("coincident point+line ok", s.addSketchConstraint("coincident", jointRef, edgeRef) !== null);
  check("coincident line+line rejected", s.addSketchConstraint("coincident", edgeRef, { kind: "edge", bodyId: body.id, index: 1 }) === null);
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

// --- constraints on hole geometry (v16) -----------------------------------------
{
  const s = new Scene();
  // Plate with a slightly tilted quadrilateral hole.
  const body = s.addBody(
    [{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }],
    0,
    "fillet",
    [[{ x: 30, y: 30 }, { x: 60, y: 34 }, { x: 60, y: 60 }, { x: 30, y: 62 }]]
  );
  const holeEdge: MeasureRef = { kind: "edge", bodyId: body.id, index: 0, hole: 0 };
  const { constraint } = tryAddConstraint(s, "horizontal", holeEdge);
  check("horizontal constraint accepted on a hole edge", constraint !== null);
  const hv = body.holes![0].controlLocal;
  const yA = hv[0].y;
  const yB = hv[1].y;
  check("hole edge solved horizontal", near(yA, yB, TOL * 2), `Δy ${(yB - yA).toExponential(2)}`);
  check("outer outline untouched by the hole solve",
    s.bodyControlWorld(body).every((p) => [0, 100].includes(Math.round(p.x)) && [0, 100].includes(Math.round(p.y))),
    "outer square intact");

  // Coincident between a joint and a hole corner drags the hole corner onto the joint
  // (the joint is on a body, so both ends are geometry-ranked; they meet in between).
  const j = s.addFreeJoint({ x: 20, y: 20 });
  const holeCorner: MeasureRef = { kind: "vertex", bodyId: body.id, index: 0, hole: 0 };
  const res = tryAddConstraint(s, "coincident", { kind: "joint", jointId: j.id }, holeCorner);
  check("coincident joint ↔ hole corner accepted", res.constraint !== null);
  const cw = s.resolveMeasureRef(holeCorner);
  const jw = s.jointWorld(s.getJoint(j.id)!);
  check("hole corner and joint coincide after solve",
    cw?.kind === "point" && Math.hypot(cw.p.x - jw.x, cw.p.y - jw.y) < TOL * 2,
    cw?.kind === "point" ? `Δ ${Math.hypot(cw.p.x - jw.x, cw.p.y - jw.y).toExponential(2)}` : "null");
}

// --- coincident point-on-line (infinite line) ------------------------------------
{
  const s = new Scene();
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ]);
  const edge0: MeasureRef = { kind: "edge", bodyId: body.id, index: 0 }; // bottom, y = 0

  // Normalization: line first still stores the point as refA (badges anchor there).
  const j = s.addFreeJoint({ x: 200, y: 30 });
  const c = s.addSketchConstraint("coincident", edge0, { kind: "joint", jointId: j.id });
  check("point+line accepted with the line picked first", c !== null);
  check("normalized: point stored as refA", c!.refA.kind === "joint" && c!.refB?.kind === "edge");

  // Structural endpoints of the line are on it forever — rejected as no-ops.
  check("edge start vertex on its own edge rejected",
    s.addSketchConstraint("coincident", { kind: "vertex", bodyId: body.id, index: 0 }, edge0) === null);
  check("edge end vertex (wrapping) rejected",
    s.addSketchConstraint("coincident", { kind: "vertex", bodyId: body.id, index: 0 }, { kind: "edge", bodyId: body.id, index: 3 }) === null);
  const nonEnd = s.addSketchConstraint("coincident", { kind: "vertex", bodyId: body.id, index: 2 }, edge0);
  check("non-endpoint vertex on an edge accepted", nonEnd !== null);
  s.removeSketchConstraint(nonEnd!.id); // don't let it fold the square in the solves below

  // Solve: the joint sits beyond the edge's segment (x = 200 > 100) — it must land on
  // the *infinite* line, not get clamped to the segment. The correction is purely
  // perpendicular, so the joint's x never changes; joint and edge (equal rank) split
  // the perpendicular gap.
  check("point-on-line solve", solveSketch(s).length === 0);
  const v = s.bodyControlWorld(body);
  const jw = s.jointWorld(s.getJoint(j.id)!);
  const lineDist = (p: { x: number; y: number }, a: { x: number; y: number }, b: { x: number; y: number }) => {
    const dx = b.x - a.x, dy = b.y - a.y;
    return Math.abs(dx * (p.y - a.y) - dy * (p.x - a.x)) / Math.hypot(dx, dy);
  };
  check("joint lies on the infinite edge line", lineDist(jw, v[0], v[1]) < TOL * 2,
    `dist ${lineDist(jw, v[0], v[1]).toExponential(2)}`);
  check("correction is perpendicular (x kept)", near(jw.x, 200, TOL * 2), `${jw.x}`);
  check("gap split between joint and edge", near(jw.y, 15, 1) && near(v[0].y, 15, 1),
    `joint y ${jw.y}, edge y ${v[0].y}`);

  // Survives serialize/load and keeps solving.
  const t = new Scene();
  t.load(JSON.parse(JSON.stringify(s.serialize())) as SceneData);
  check("point-on-line survives save/load", t.sketch.some((k) => k.kind === "coincident" && k.refB?.kind === "edge"));
  t.moveJoint(j.id, { x: 0, y: 40 });
  check("re-solves after load + violation", solveSketch(t).length === 0);
}

// --- point-on-line: rails and guidelines ------------------------------------------
{
  const s = new Scene();
  const rail = s.addSlider(s.addFreeJoint({ x: 0, y: 0 }).id, s.addFreeJoint({ x: 100, y: 0 }).id)!;
  const railRef: MeasureRef = { kind: "rail", sliderId: rail.id };
  check("rail's own joint on its rail rejected",
    s.addSketchConstraint("coincident", { kind: "joint", jointId: rail.railA }, railRef) === null);
  const j = s.addFreeJoint({ x: 50, y: 30 });
  const res = tryAddConstraint(s, "coincident", { kind: "joint", jointId: j.id }, railRef);
  check("free joint onto a rail line accepted + solved", res.constraint !== null);
  const jw = s.jointWorld(s.getJoint(j.id)!);
  const a = s.jointWorld(s.getJoint(rail.railA)!);
  const b = s.jointWorld(s.getJoint(rail.railB)!);
  const cross = Math.abs((b.x - a.x) * (jw.y - a.y) - (b.y - a.y) * (jw.x - a.x)) / Math.hypot(b.x - a.x, b.y - a.y);
  check("joint on the rail line", cross < TOL * 2, `dist ${cross.toExponential(2)}`);

  // Guideline: construction rank yields — the guide comes to the joint, the joint stays.
  const g = s.addGuide({ x: 0, y: 100 }, { x: 100, y: 100 })!;
  check("guide's own defining point rejected",
    s.addSketchConstraint("coincident", { kind: "guidePoint", guideId: g.id, which: "a" }, { kind: "guideLine", guideId: g.id }) === null);
  const fixed = s.addFreeJoint({ x: 40, y: 60 });
  const res2 = tryAddConstraint(s, "coincident", { kind: "joint", jointId: fixed.id }, { kind: "guideLine", guideId: g.id });
  check("joint onto a guideline accepted + solved", res2.constraint !== null);
  const fw = s.jointWorld(s.getJoint(fixed.id)!);
  check("geometry outranks construction: joint unmoved", near(fw.x, 40, TOL * 2) && near(fw.y, 60, TOL * 2),
    `${fw.x}, ${fw.y}`);
  const gg = s.getGuide(g.id)!;
  const gDist = Math.abs((gg.b.x - gg.a.x) * (fw.y - gg.a.y) - (gg.b.y - gg.a.y) * (fw.x - gg.a.x)) / Math.hypot(gg.b.x - gg.a.x, gg.b.y - gg.a.y);
  check("guide moved onto the joint", gDist < TOL * 2, `dist ${gDist.toExponential(2)}`);
}

// --- several points onto one guideline: the guide becomes the reference ------------
{
  // One tie translates the guide onto the point; a second can't be met that way (the
  // translation reaching one point leaves the other). With 2+ ties the guide is the
  // reference: later points move perpendicular onto it, the guide + first point stay.
  const s = new Scene();
  const g = s.addGuide({ x: 0, y: 100 }, { x: 100, y: 100 })!;
  s.addSketchConstraint("horizontal", { kind: "guideLine", guideId: g.id });
  const gl: MeasureRef = { kind: "guideLine", guideId: g.id };
  const js = [s.addFreeJoint({ x: 40, y: 60 }), s.addFreeJoint({ x: 140, y: 20 }), s.addFreeJoint({ x: 240, y: 90 })];
  const jy = (i: number) => s.jointWorld(s.getJoint(js[i].id)!).y;
  const jx = (i: number) => s.jointWorld(s.getJoint(js[i].id)!).x;
  check("1st joint onto guide", tryAddConstraint(s, "coincident", { kind: "joint", jointId: js[0].id }, gl).constraint !== null);
  check("guide came to the 1st joint", near(s.getGuide(g.id)!.a.y, 60, TOL * 2), `${s.getGuide(g.id)!.a.y}`);
  check("2nd joint onto the same guide accepted", tryAddConstraint(s, "coincident", { kind: "joint", jointId: js[1].id }, gl).constraint !== null);
  check("3rd joint onto the same guide accepted", tryAddConstraint(s, "coincident", { kind: "joint", jointId: js[2].id }, gl).constraint !== null);
  check("guide + 1st joint held, later joints moved onto the line",
    near(s.getGuide(g.id)!.a.y, 60, TOL * 2) && [0, 1, 2].every((i) => near(jy(i), 60, TOL * 2)), `${jy(0)} ${jy(1)} ${jy(2)}`);
  check("aligned joints only moved perpendicular to the guide", near(jx(0), 40) && near(jx(1), 140) && near(jx(2), 240));
  // Dragging one aligned joint carries the guide and the other aligned joints.
  s.moveJoint(js[0].id, { x: 0, y: 30 });
  check("drag an aligned joint: solves", solveSketch(s, new Set([`j:${js[0].id}`])).length === 0);
  check("guide and the other joints followed the drag",
    near(s.getGuide(g.id)!.a.y, 90, TOL * 2) && near(jy(1), 90, TOL * 2) && near(jy(2), 90, TOL * 2), `${jy(1)} ${jy(2)}`);
  // Dragging the guide carries every aligned joint.
  const gg = s.getGuide(g.id)!;
  gg.a = { x: gg.a.x, y: gg.a.y + 20 };
  gg.b = { x: gg.b.x, y: gg.b.y + 20 };
  check("drag the guide: solves", solveSketch(s, new Set([`g:${g.id}:a`, `g:${g.id}:b`])).length === 0);
  check("aligned joints followed the guide", [0, 1, 2].every((i) => near(jy(i), 110, TOL * 2)), `${jy(0)} ${jy(1)} ${jy(2)}`);

  // Mixed: a guide glued point–point to joint A plus joint B on its line — B comes to
  // the line, A and the guide stay (used to be rejected: the two ties fought).
  const t = new Scene();
  const g2 = t.addGuide({ x: 0, y: 100 }, { x: 100, y: 100 })!;
  const A = t.addFreeJoint({ x: 0, y: 100 });
  t.addSketchConstraint("coincident", { kind: "guidePoint", guideId: g2.id, which: "a" }, { kind: "joint", jointId: A.id });
  const B = t.addFreeJoint({ x: 150, y: 130 });
  check("glued guide + point-on-line accepted",
    tryAddConstraint(t, "coincident", { kind: "joint", jointId: B.id }, { kind: "guideLine", guideId: g2.id }).constraint !== null);
  const aw = t.jointWorld(t.getJoint(A.id)!);
  const bw = t.jointWorld(t.getJoint(B.id)!);
  check("glued joint + guide held, the other joint came to the line",
    near(aw.y, 100, TOL * 2) && near(t.getGuide(g2.id)!.a.y, 100, TOL * 2) && near(bw.y, 100, TOL * 2) && near(bw.x, 150, TOL * 2),
    `A ${aw.y}, B ${bw.x},${bw.y}`);
}

// --- guides tied to geometry: dimensions to them move the geometry ------------------
{
  const rect = (s: Scene) => s.addBody([{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 50 }, { x: 0, y: 50 }], 0);
  const gl = (id: number): MeasureRef => ({ kind: "guideLine", guideId: id });
  const vx = (b: number, i: number): MeasureRef => ({ kind: "vertex", bodyId: b, index: i });

  // A free guide still yields: the dimension moves the guide, the body stays.
  {
    const s = new Scene();
    const b = rect(s);
    const g = s.addGuide({ x: 200, y: -100 }, { x: 200, y: 200 })!;
    const m = s.addMeasurement("draw", vx(b.id, 1), gl(g.id), { x: 150, y: 25 })!;
    check("free guide: dimension drives", applyDrivingDimension(s, m.id, 120).length === 0);
    check("free guide: the guide moved", near(s.getGuide(g.id)!.a.x, 220, TOL * 2), `${s.getGuide(g.id)!.a.x}`);
    check("free guide: body unmoved", near(s.bodyControlWorld(b)[1].x, 100, TOL * 2));
  }
  // A guide coincident with one corner is a fixed reference: dimensioning another
  // corner to it moves that corner (used to fail — tie and dimension fought over the guide).
  {
    const s = new Scene();
    const b = rect(s);
    const g = s.addGuide({ x: 0, y: -100 }, { x: 0, y: 200 })!;
    check("tie placed", tryAddConstraint(s, "coincident", vx(b.id, 0), gl(g.id)).constraint !== null);
    const m = s.addMeasurement("draw", vx(b.id, 1), gl(g.id), { x: 50, y: -25 })!;
    check("tied guide: dimension drives", applyDrivingDimension(s, m.id, 120).length === 0);
    check("tied guide: guide stayed put", near(s.getGuide(g.id)!.a.x, 0, TOL * 2) && near(s.getGuide(g.id)!.b.x, 0, TOL * 2));
    const v = s.bodyControlWorld(b);
    check("tied guide: the dimensioned corner moved", near(v[1].x, 120, TOL * 2), `${v[1].x}`);
    check("tied guide: the tied corner stayed", near(v[0].x, 0, TOL * 2) && near(v[0].y, 0, TOL * 2));
    check("tied guide: re-solve holds everything", solveSketch(s).length === 0);
    check("tied guide: value holds", near(s.measureInfo(m)!.value, 120, TOL * 2));
  }
  // Placing the tie itself still brings the guide to the geometry (rank 0 for ties).
  {
    const s = new Scene();
    const b = rect(s);
    const g = s.addGuide({ x: 30, y: -100 }, { x: 30, y: 200 })!;
    check("placing a tie", tryAddConstraint(s, "coincident", vx(b.id, 0), gl(g.id)).constraint !== null);
    check("placing a tie moves the guide, not the body",
      near(s.getGuide(g.id)!.a.x, 0, TOL * 2) && near(s.bodyControlWorld(b)[0].x, 0, TOL * 2));
  }
  // A guide through two grounded joints: the body corner comes to the dimension.
  {
    const s = new Scene();
    const b = rect(s);
    const j1 = s.addFreeJoint({ x: 200, y: 0 });
    const j2 = s.addFreeJoint({ x: 200, y: 100 });
    s.addGround(j1.id);
    s.addGround(j2.id);
    const g = s.addGuide({ x: 200, y: 0 }, { x: 200, y: 100 })!;
    tryAddConstraint(s, "coincident", { kind: "joint", jointId: j1.id }, { kind: "guidePoint", guideId: g.id, which: "a" });
    tryAddConstraint(s, "coincident", { kind: "joint", jointId: j2.id }, { kind: "guidePoint", guideId: g.id, which: "b" });
    const m = s.addMeasurement("draw", vx(b.id, 1), gl(g.id), { x: 150, y: 25 })!;
    check("guide on grounded joints: dimension drives", applyDrivingDimension(s, m.id, 80).length === 0);
    check("guide on grounded joints: guide unmoved", near(s.getGuide(g.id)!.a.x, 200, TOL * 2));
    check("guide on grounded joints: corner moved", near(s.bodyControlWorld(b)[1].x, 120, TOL * 2), `${s.bodyControlWorld(b)[1].x}`);
  }
  // Chain: a guide tied to a tied guide counts as tied too.
  {
    const s = new Scene();
    const b = rect(s);
    const g1 = s.addGuide({ x: 0, y: -100 }, { x: 0, y: 200 })!;
    const g2 = s.addGuide({ x: 0, y: 300 }, { x: 100, y: 300 })!;
    tryAddConstraint(s, "coincident", vx(b.id, 0), gl(g1.id));
    tryAddConstraint(s, "coincident", { kind: "guidePoint", guideId: g2.id, which: "a" }, gl(g1.id));
    const m = s.addMeasurement("draw", vx(b.id, 3), gl(g2.id), { x: -20, y: 150 })!;
    check("chained tie: dimension drives", applyDrivingDimension(s, m.id, 200).length === 0);
    check("chained tie: guides unmoved", near(s.getGuide(g2.id)!.a.y, 300, TOL * 2) && near(s.getGuide(g1.id)!.a.x, 0, TOL * 2));
    check("chained tie: corner moved", near(s.bodyControlWorld(b)[3].y, 100, TOL * 2), `${s.bodyControlWorld(b)[3].y}`);
  }
}

// --- several driving dimensions on one free guide ------------------------------------
{
  // Two squares, one guide below both. The first edge–guide dimension moves the guide
  // (single demand: construction yields). The second used to be rejected — both
  // dimensions pushed the guide and neither body moved. Now the fallback pass treats
  // the guide as a reference and the second body comes to it.
  const s = new Scene();
  const A = s.addBody([{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }], 0);
  const B = s.addBody([{ x: 200, y: 0 }, { x: 300, y: 0 }, { x: 300, y: 100 }, { x: 200, y: 100 }], 0);
  const g = s.addGuide({ x: -50, y: 150 }, { x: 350, y: 150 })!;
  const gl: MeasureRef = { kind: "guideLine", guideId: g.id };
  const m1 = s.addMeasurement("draw", { kind: "edge", bodyId: A.id, index: 2 }, gl, { x: 50, y: 125 })!;
  check("first edge–guide dimension drives", applyDrivingDimension(s, m1.id, 30).length === 0);
  check("single demand: the guide moved", near(s.getGuide(g.id)!.a.y, 130, TOL * 2), `${s.getGuide(g.id)!.a.y}`);
  const m2 = s.addMeasurement("draw", { kind: "edge", bodyId: B.id, index: 2 }, gl, { x: 250, y: 125 })!;
  check("second dimension to the same guide drives", applyDrivingDimension(s, m2.id, 50).length === 0);
  check("guide stayed, second body moved",
    near(s.getGuide(g.id)!.a.y, 130, TOL * 2) && near(s.bodyControlWorld(B)[2].y, 80, TOL * 2),
    `guide ${s.getGuide(g.id)!.a.y}, B top ${s.bodyControlWorld(B)[2].y}`);
  check("first dimension still holds", near(s.measureInfo(m1)!.value, 30, TOL * 2));
  // Dragging A pulls the guide (its dimension), which pulls B (the other one).
  s.moveBody(A.id, { x: 0, y: 10 });
  check("drag re-solve holds", solveSketch(s, new Set(anchorVarsForBody(s, A.id))).length === 0);
  check("drag chains through the guide",
    near(s.getGuide(g.id)!.a.y, 140, TOL * 2) && near(s.bodyControlWorld(B)[2].y, 90, TOL * 2),
    `guide ${s.getGuide(g.id)!.a.y}, B top ${s.bodyControlWorld(B)[2].y}`);
}

// --- holes + joints ride along when a dimension moves a body ------------------
{
  // Two squares with a disk hole each; B also carries a joint not stuck to a corner.
  // B is shape-locked (H/V edges + two driving side lengths) so a dimension to A can
  // only translate it. Dragging A used to move B's outline vertex by vertex, leaving
  // its hole and joint at their old world positions (`moveBodyVertex` keeps them
  // world-fixed); the rigid part of the motion is now applied to the whole body.
  const sq = (x: number, y: number, s: number) => [
    { x, y }, { x: x + s, y }, { x: x + s, y: y + s }, { x, y: y + s },
  ];
  const s = new Scene();
  const A = s.addBody(sq(0, 0, 100), 0, "fillet", [{ control: [{ x: 50, y: 50 }], radius: 20, round: "offset" }]);
  const B = s.addBody(sq(200, 0, 100), 0, "fillet", [{ control: [{ x: 250, y: 50 }], radius: 20, round: "offset" }]);
  autoConstrainBody(s, A.id);
  autoConstrainBody(s, B.id);
  const j = s.addJoint(B.id, { x: 270, y: 30 });
  const vB = (i: number): MeasureRef => ({ kind: "vertex", bodyId: B.id, index: i });
  const w = s.addMeasurement("draw", vB(0), vB(1), { x: 250, y: -20 })!;
  const h = s.addMeasurement("draw", vB(1), vB(2), { x: 320, y: 50 })!;
  check("B side dims drive", applyDrivingDimension(s, w.id, 100).length === 0 && applyDrivingDimension(s, h.id, 100).length === 0);
  const gap = s.addMeasurement("draw", { kind: "edge", bodyId: A.id, index: 1 }, { kind: "edge", bodyId: B.id, index: 3 }, { x: 150, y: 50 })!;
  check("A–B gap dimension drives", applyDrivingDimension(s, gap.id, 100).length === 0);
  s.moveBody(A.id, { x: 30, y: 10 });
  check("drag re-solve holds", solveSketch(s, new Set(anchorVarsForBody(s, A.id))).length === 0);
  const c0 = s.bodyControlWorld(B)[0];
  check("B translated whole", near(c0.x, 230, TOL * 2) && near(s.bodyControlWorld(B)[2].x, 330, TOL * 2), `${c0.x}, ${s.bodyControlWorld(B)[2].x}`);
  const hole = s.bodyHoleControlWorld(B, 0)[0];
  check("B's hole rode along", near(hole.x - c0.x, 50, TOL * 2) && near(hole.y - c0.y, 50, TOL * 2), `${hole.x - c0.x}, ${hole.y - c0.y}`);
  const jw = s.jointWorld(j);
  check("B's joint rode along", near(jw.x - c0.x, 70, TOL * 2) && near(jw.y - c0.y, 30, TOL * 2), `${jw.x - c0.x}, ${jw.y - c0.y}`);
  check("A's hole rode along too", near(s.bodyHoleControlWorld(A, 0)[0].x, 80, TOL * 2) && near(s.bodyHoleControlWorld(A, 0)[0].y, 60, TOL * 2));
  // A genuine reshape (one corner pulled, the others outside the system) carries the
  // hole only by the outline's rigid part — never the full corner move.
  const t = new Scene();
  const P = t.addBody(sq(0, 0, 100), 0, "fillet", [{ control: [{ x: 50, y: 50 }], radius: 20, round: "offset" }]);
  const Q = t.addBody(sq(200, 0, 100), 0, "fillet", [{ control: [{ x: 250, y: 50 }], radius: 20, round: "offset" }]);
  const d = t.addMeasurement("draw", { kind: "vertex", bodyId: P.id, index: 1 }, { kind: "vertex", bodyId: Q.id, index: 0 }, { x: 150, y: -20 })!;
  check("corner–corner dimension drives", applyDrivingDimension(t, d.id, 100).length === 0);
  t.moveBody(P.id, { x: 40, y: 0 });
  check("reshape re-solve holds", solveSketch(t, new Set(anchorVarsForBody(t, P.id))).length === 0);
  const qh = t.bodyHoleControlWorld(Q, 0)[0];
  const moved = Math.hypot(qh.x - 250, qh.y - 50);
  check("reshape carries the hole only by the outline's rigid part", moved > 0 && moved < 20, `${moved}`);

  // Dragging a body dimensioned to a locked (tied) horizontal guide, like main.ts
  // does per frame: moveBody, anchored solve fails (the guide can't yield), symmetric
  // fallback. Without internal constraints only the dimensioned edge is a variable, so
  // the body squashes; its hole used to keep the full cursor rise — now it stays centred.
  const u = new Scene();
  const R = u.addBody(sq(0, 0, 100), 0, "fillet", [{ control: [{ x: 50, y: 50 }], radius: 20, round: "offset" }]);
  const g = u.addGuide({ x: -100, y: 150 }, { x: 300, y: 150 })!;
  const fj = u.addFreeJoint({ x: -100, y: 150 });
  u.addSketchConstraint("coincident", { kind: "guidePoint", guideId: g.id, which: "a" }, { kind: "joint", jointId: fj.id });
  const gd = u.addMeasurement("draw", { kind: "edge", bodyId: R.id, index: 2 }, { kind: "guideLine", guideId: g.id }, { x: 50, y: 125 })!;
  check("edge–locked-guide dimension drives", applyDrivingDimension(u, gd.id, 50).length === 0);
  for (let f = 0; f < 5; f++) {
    u.moveBody(R.id, { x: 10, y: 7 });
    if (solveSketch(u, new Set(anchorVarsForBody(u, R.id))).length > 0) solveSketch(u);
  }
  solveSketch(u);
  const rv = u.bodyControlWorld(R);
  const rh = u.bodyHoleControlWorld(R, 0)[0];
  check("guide held, top edge held", near(u.getGuide(g.id)!.a.y, 150, TOL * 2) && near(rv[2].y, 100, TOL * 2), `${rv[2].y}`);
  check("hole followed horizontally", near(rh.x, 100, TOL * 2), `${rh.x}`);
  check("hole stays centred in the squashed body", near(rh.y, (rv[0].y + rv[3].y) / 2, TOL * 2), `hole ${rh.y}, outline ${rv[0].y}..${rv[3].y}`);

  // Rigid body (H/V + side dims) dimensioned to the locked guide, with TWO disk holes
  // aligned by a vertical constraint between their centres and a joint aligned
  // horizontally with one of them. Those hole centres / the joint are solver
  // variables the solve never moves (already satisfied), so the vertex pass used to
  // re-apply their stale positions after the whole-body carry — the holes and the
  // joint kept the cursor's full rise while the outline shed it.
  const r = new Scene();
  const S = r.addBody(sq(0, 0, 100), 0, "fillet", [
    { control: [{ x: 50, y: 30 }], radius: 10, round: "offset" },
    { control: [{ x: 50, y: 70 }], radius: 10, round: "offset" },
  ]);
  autoConstrainBody(r, S.id);
  const vS = (i: number): MeasureRef => ({ kind: "vertex", bodyId: S.id, index: i });
  applyDrivingDimension(r, r.addMeasurement("draw", vS(0), vS(1), { x: 50, y: -20 })!.id, 100);
  applyDrivingDimension(r, r.addMeasurement("draw", vS(1), vS(2), { x: 120, y: 50 })!.id, 100);
  const g2 = r.addGuide({ x: -100, y: 150 }, { x: 300, y: 150 })!;
  const fj2 = r.addFreeJoint({ x: -100, y: 150 });
  r.addSketchConstraint("coincident", { kind: "guidePoint", guideId: g2.id, which: "a" }, { kind: "joint", jointId: fj2.id });
  check("rigid body–guide dimension drives",
    applyDrivingDimension(r, r.addMeasurement("draw", { kind: "edge", bodyId: S.id, index: 2 }, { kind: "guideLine", guideId: g2.id }, { x: 50, y: 125 })!.id, 50).length === 0);
  const sj = r.addJoint(S.id, { x: 80, y: 30 });
  const hc0: MeasureRef = { kind: "vertex", bodyId: S.id, index: 0, hole: 0 };
  const hc1: MeasureRef = { kind: "vertex", bodyId: S.id, index: 0, hole: 1 };
  check("vertical between hole centres accepted", r.addSketchConstraint("vertical", hc0, hc1) !== null);
  check("horizontal joint–hole centre accepted", r.addSketchConstraint("horizontal", { kind: "joint", jointId: sj.id }, hc0) !== null);
  check("hole-aligned body solves", solveSketch(r).length === 0);
  for (let f = 0; f < 5; f++) {
    r.moveBody(S.id, { x: 10, y: 7 });
    if (solveSketch(r, new Set(anchorVarsForBody(r, S.id))).length > 0) solveSketch(r);
  }
  solveSketch(r);
  const sv = r.bodyControlWorld(S);
  check("rigid body slid horizontally only", near(sv[0].x, 50, TOL * 2) && near(sv[0].y, 0, TOL * 2) && near(sv[2].y, 100, TOL * 2), `${sv[0].x}, ${sv[0].y}`);
  const k0 = r.bodyHoleControlWorld(S, 0)[0];
  const k1 = r.bodyHoleControlWorld(S, 1)[0];
  check("first aligned hole rode along", near(k0.x - sv[0].x, 50, TOL * 2) && near(k0.y - sv[0].y, 30, TOL * 2), `${k0.x - sv[0].x}, ${k0.y - sv[0].y}`);
  check("second aligned hole rode along", near(k1.x - sv[0].x, 50, TOL * 2) && near(k1.y - sv[0].y, 70, TOL * 2), `${k1.x - sv[0].x}, ${k1.y - sv[0].y}`);
  const sjw = r.jointWorld(sj);
  check("constrained joint rode along", near(sjw.x - sv[0].x, 80, TOL * 2) && near(sjw.y - sv[0].y, 30, TOL * 2), `${sjw.x - sv[0].x}, ${sjw.y - sv[0].y}`);
}

// --- no sideways drift when dragging against vertical line–line dimensions --------
{
  // Locked horizontal guide → A (edge–line), A → B (edge–edge), both bodies rigid
  // (H/V edges + side dims). Dragging B vertically is refused by the dimensions, but
  // used to walk A (and B) sideways: mid-sweep a side dimension tilts A's top edge for
  // a moment and the edge–edge shift along that tilted normal leaked into x, which
  // nothing pulls back. H/V-constrained lines now shift along the exact world axis.
  const sq = (x: number, y: number, s: number) => [
    { x, y }, { x: x + s, y }, { x: x + s, y: y + s }, { x, y: y + s },
  ];
  const s = new Scene();
  const rigid = (x: number, y: number) => {
    const b = s.addBody(sq(x, y, 100), 0, "fillet", [{ control: [{ x: x + 50, y: y + 50 }], radius: 15, round: "offset" }]);
    autoConstrainBody(s, b.id);
    const v = (i: number): MeasureRef => ({ kind: "vertex", bodyId: b.id, index: i });
    applyDrivingDimension(s, s.addMeasurement("draw", v(0), v(1), { x: x + 50, y: y - 20 })!.id, 100);
    applyDrivingDimension(s, s.addMeasurement("draw", v(1), v(2), { x: x + 120, y: y + 50 })!.id, 100);
    return b;
  };
  const g = s.addGuide({ x: -100, y: -50 }, { x: 300, y: -50 })!;
  const fj = s.addFreeJoint({ x: -100, y: -50 });
  s.addSketchConstraint("coincident", { kind: "guidePoint", guideId: g.id, which: "a" }, { kind: "joint", jointId: fj.id });
  const A = rigid(0, 0);
  const B = rigid(0, 200);
  check("guide–A edge dimension drives",
    applyDrivingDimension(s, s.addMeasurement("draw", { kind: "edge", bodyId: A.id, index: 0 }, { kind: "guideLine", guideId: g.id }, { x: 50, y: -25 })!.id, 50).length === 0);
  check("A–B edge–edge dimension drives",
    applyDrivingDimension(s, s.addMeasurement("draw", { kind: "edge", bodyId: A.id, index: 2 }, { kind: "edge", bodyId: B.id, index: 0 }, { x: 150, y: 150 })!.id, 100).length === 0);
  for (let f = 0; f < 10; f++) {
    s.moveBody(B.id, { x: 0, y: 7 });
    if (solveSketch(s, new Set(anchorVarsForBody(s, B.id))).length > 0) solveSketch(s);
  }
  solveSketch(s);
  const a = s.bodyControlWorld(A)[0];
  const b = s.bodyControlWorld(B)[0];
  check("vertical drag of B refused (both stay at their heights)", near(a.y, 0, TOL * 2) && near(b.y, 200, TOL * 2), `${a.y}, ${b.y}`);
  check("A did not drift sideways", near(a.x, 0, 1e-6), `${a.x}`);
  check("B did not drift sideways", near(b.x, 0, 1e-6), `${b.x}`);
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll sketch checks passed.");
