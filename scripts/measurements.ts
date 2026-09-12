/**
 * Measurements: point/line references, axis selection from label placement, live value
 * updates as geometry moves, the parallel↔angle flip for line pairs, vertex-index
 * remapping across control-polygon edits, cascade removal, and serialize/load.
 */
import { Scene, MeasureRef, SceneData, measureAxisForPlacement } from "../src/model";
import { applyDrivingDimension, solveSketch } from "../src/sketch";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: number, b: number, eps = 1e-6) => Math.abs(a - b) < eps;

// --- axis selection from label placement ----------------------------------
{
  const p = { x: 0, y: 0 };
  const q = { x: 100, y: 100 };
  check("axis: above the pair → h", measureAxisForPlacement(p, q, { x: 50, y: -40 }) === "h");
  check("axis: beside the pair → v", measureAxisForPlacement(p, q, { x: 160, y: 50 }) === "v");
  check("axis: diagonal zone → direct", measureAxisForPlacement(p, q, { x: 160, y: -40 }) === "direct");
  check("axis: between the points → direct", measureAxisForPlacement(p, q, { x: 50, y: 50 }) === "direct");
}

// --- point + point ----------------------------------------------------------
{
  const s = new Scene();
  const jA = s.addFreeJoint({ x: 0, y: 0 });
  const jB = s.addFreeJoint({ x: 60, y: 80 });
  const refA: MeasureRef = { kind: "joint", jointId: jA.id };
  const refB: MeasureRef = { kind: "joint", jointId: jB.id };

  const mDirect = s.addMeasurement("draw", refA, refB, { x: 120, y: 120 })!;
  const mH = s.addMeasurement("draw", refA, refB, { x: 30, y: -50 })!;
  const mV = s.addMeasurement("draw", refA, refB, { x: 120, y: 40 })!;
  check("point-point direct axis", mDirect.axis === "direct", mDirect.axis);
  check("point-point h axis", mH.axis === "h", mH.axis);
  check("point-point v axis", mV.axis === "v", mV.axis);

  const iD = s.measureInfo(mDirect)!;
  const iH = s.measureInfo(mH)!;
  const iV = s.measureInfo(mV)!;
  check("direct value = straight distance", near(iD.value, 100), `${iD.value}`);
  check("h value = |Δx|", near(iH.value, 60), `${iH.value}`);
  check("v value = |Δy|", near(iV.value, 80), `${iV.value}`);
  check("h dimension line at label height", near(iH.dim!.a.y, -50) && near(iH.dim!.b.y, -50));

  // Preview (no measurement created) matches what placement would create.
  const before = s.measurements.length;
  const prev = s.measurePreview(refA, refB, { x: 120, y: 120 })!;
  check("preview value matches placement", prev.kind === "distance" && near(prev.value, 100));
  check("preview creates nothing", s.measurements.length === before);

  // Re-placing the label re-derives the axis.
  s.setMeasurementLabel(mH.id, { x: 120, y: 40 });
  check("label move re-derives axis", mH.axis === "v" && near(s.measureInfo(mH)!.value, 80));

  // Values and the label track the geometry as it moves.
  s.moveJoint(jB.id, { x: 10, y: 0 }); // B → (70, 80)
  check("value updates as a joint moves", near(s.measureInfo(mDirect)!.value, Math.hypot(70, 80)));
  const lp = s.measurementLabelPos(mDirect)!;
  check("label follows the geometry", near(lp.x, 125) && near(lp.y, 120), `(${lp.x}, ${lp.y})`);

  // Removing a referenced joint removes every measurement that used it.
  s.removeJoint(jB.id);
  check("cascade: joint removal prunes its measurements", s.measurements.length === 0);
}

// --- point + line (perpendicular distance to the infinite line) -------------
{
  const s = new Scene();
  const ra = s.addFreeJoint({ x: 0, y: 0 });
  const rb = s.addFreeJoint({ x: 100, y: 0 });
  const slider = s.addSlider(ra.id, rb.id); // world-fixed rail (auto-grounded)
  const jp = s.addFreeJoint({ x: 150, y: 40 });
  const m = s.addMeasurement(
    "draw",
    { kind: "rail", sliderId: slider.id },
    { kind: "joint", jointId: jp.id },
    { x: 150, y: 20 }
  )!;
  const info = s.measureInfo(m)!;
  check("point-line uses the infinite line", info.kind === "distance" && near(info.value, 40), `${info.value}`);
  check("foot beyond the rail end gets an extension line", info.ext.length === 1);
  check("dimension line runs point → foot", near(info.dim!.b.x, 150) && near(info.dim!.b.y, 0));
  // Sliding the label along the rail carries the dimension line with it: dropped
  // perpendicular at the label's position, with a dashed extension from the point.
  s.setMeasurementLabel(m.id, { x: 50, y: 20 });
  const moved = s.measureInfo(m)!;
  check("point-line value is unchanged by label position", near(moved.value, 40));
  check(
    "dimension line follows the label along the line",
    near(moved.dim!.a.x, 50) && near(moved.dim!.a.y, 40) && near(moved.dim!.b.x, 50) && near(moved.dim!.b.y, 0)
  );
  check(
    "extension runs from the point to the dimension line",
    moved.ext.length === 1 && near(moved.ext[0].a.x, 150) && near(moved.ext[0].b.x, 50) && near(moved.ext[0].b.y, 40)
  );

  s.removeConstraint(slider.id);
  check("cascade: slider removal prunes rail measurements", s.measurements.length === 0);
}

// --- line + line: parallel distance, dynamic flip to angle, label sector ----
{
  const s = new Scene();
  const r1 = s.addSlider(s.addFreeJoint({ x: 0, y: 0 }).id, s.addFreeJoint({ x: 100, y: 0 }).id);
  const j2b = s.addFreeJoint({ x: 100, y: 50 });
  const r2 = s.addSlider(s.addFreeJoint({ x: 0, y: 50 }).id, j2b.id);
  const m = s.addMeasurement(
    "draw",
    { kind: "rail", sliderId: r1.id },
    { kind: "rail", sliderId: r2.id },
    { x: 50, y: 25 }
  )!;
  const parallel = s.measureInfo(m)!;
  check("parallel lines measure distance", parallel.kind === "distance" && near(parallel.value, 50), `${parallel.value}`);

  // Tilt the second rail: the same measurement now reports the angle.
  s.moveJoint(j2b.id, { x: 0, y: 100 }); // rail 2 → (0,50)-(100,150), 45° to rail 1
  const tilted = s.measureInfo(m)!;
  check("tilted lines flip to angle", tilted.kind === "angle" && near(tilted.value, 45, 1e-9), `${tilted.value}`);

  // The label's sector picks θ vs 180°−θ.
  const m2 = s.addMeasurement(
    "draw",
    { kind: "rail", sliderId: r1.id },
    { kind: "rail", sliderId: r2.id },
    { x: -100, y: 30 }
  )!;
  const obtuse = s.measureInfo(m2)!;
  check("label sector picks the obtuse angle", near(obtuse.value, 135, 1e-9), `${obtuse.value}`);
}

// --- vertex / edge refs + index remapping across control-polygon edits ------
{
  const s = new Scene();
  const body = s.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ]);
  const mv = s.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body.id, index: 0 },
    { kind: "vertex", bodyId: body.id, index: 2 },
    { x: 200, y: 200 }
  )!;
  const me = s.addMeasurement(
    "draw",
    { kind: "edge", bodyId: body.id, index: 0 },
    { kind: "edge", bodyId: body.id, index: 1 },
    { x: 50, y: 50 }
  )!;
  check("vertex-vertex diagonal", near(s.measureInfo(mv)!.value, Math.hypot(100, 100)));
  check("perpendicular edges measure 90°", s.measureInfo(me)!.kind === "angle" && near(s.measureInfo(me)!.value, 90, 1e-9));

  s.insertBodyVertex(body.id, 1, { x: 50, y: -10 });
  check(
    "insert shifts later vertex/edge refs",
    (mv.refB as { index: number }).index === 3 && (me.refB as { index: number }).index === 2
  );
  check("vertex refs still point at the same corners", near(s.measureInfo(mv)!.value, Math.hypot(100, 100)));

  s.removeBodyVertex(body.id, 1);
  check(
    "remove shifts refs back",
    (mv.refB as { index: number }).index === 2 && (me.refB as { index: number }).index === 1
  );
  check("edge angle intact after insert+remove", near(s.measureInfo(me)!.value, 90, 1e-9));

  // A bodyPoint reference rides the body's frame.
  const jr = s.addFreeJoint({ x: 200, y: 50 });
  const mb = s.addMeasurement(
    "draw",
    { kind: "bodyPoint", bodyId: body.id, local: { x: 100 - body.pos.x, y: 50 - body.pos.y } },
    { kind: "joint", jointId: jr.id },
    { x: 150, y: 80 }
  )!;
  check("bodyPoint distance", near(s.measureInfo(mb)!.value, 100), `${s.measureInfo(mb)!.value}`);
  s.moveBody(body.id, { x: 10, y: 5 });
  check("bodyPoint tracks the body", near(s.measureInfo(mb)!.value, 90), `${s.measureInfo(mb)!.value}`);

  s.removeBody(body.id);
  check("cascade: body removal prunes vertex/edge/bodyPoint measurements", s.measurements.length === 0);
}

// --- serialize / load --------------------------------------------------------
{
  const s = new Scene();
  const jA = s.addFreeJoint({ x: 0, y: 0 });
  const jB = s.addFreeJoint({ x: 30, y: 40 });
  const rail = s.addSlider(s.addFreeJoint({ x: 0, y: 100 }).id, s.addFreeJoint({ x: 100, y: 100 }).id);
  const mDraw = s.addMeasurement("draw", { kind: "joint", jointId: jA.id }, { kind: "joint", jointId: jB.id }, { x: 60, y: 60 })!;
  const mSim = s.addMeasurement("sim", { kind: "rail", sliderId: rail.id }, { kind: "joint", jointId: jA.id }, { x: 40, y: 50 })!;

  const text = JSON.stringify(s.serialize());
  const t = new Scene();
  t.load(JSON.parse(text));
  check("measurements round-trip", t.measurements.length === 2, `${t.measurements.length}`);
  check("modes preserved", t.getMeasurement(mDraw.id)?.mode === "draw" && t.getMeasurement(mSim.id)?.mode === "sim");
  check("draw value survives", near(t.measureInfo(t.getMeasurement(mDraw.id)!)!.value, 50));
  check("sim value survives", near(t.measureInfo(t.getMeasurement(mSim.id)!)!.value, 100));

  const maxId = Math.max(...t.measurements.map((m) => m.id), ...t.joints.map((j) => j.id), ...t.constraints.map((c) => c.id));
  const fresh = t.addMeasurement("draw", { kind: "joint", jointId: jA.id }, { kind: "joint", jointId: jB.id }, { x: 0, y: 0 })!;
  check("nextId continues past measurement ids", fresh.id > maxId, `${fresh.id} > ${maxId}`);

  t.getMeasurement(mDraw.id)!.labelOffset.x += 999;
  check("load deep-copies measurements", s.getMeasurement(mDraw.id)!.labelOffset.x < 999);

  // Pre-v7 file: no measurements field at all.
  const legacy = JSON.parse(text) as SceneData;
  delete legacy.measurements;
  const u = new Scene();
  u.load(legacy);
  check("pre-v7 file loads with no measurements", u.measurements.length === 0);
}

// --- diameter dimensions on disks (round holes / disk bodies) -----------------
{
  const s = new Scene();
  const sq = [
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ];
  const body = s.addBody(sq, 0, "fillet", [{ control: [{ x: 50, y: 50 }], radius: 10, round: "offset" }]);
  const refC: MeasureRef = { kind: "vertex", bodyId: body.id, index: 0, hole: 0 };
  const refV: MeasureRef = { kind: "vertex", bodyId: body.id, index: 0 };

  const disk = s.diskOfRef(refC);
  check("diskOfRef finds the disk hole", !!disk && near(disk.c.x, 50) && near(disk.c.y, 50) && near(disk.r, 10));
  check("diskOfRef rejects a polygon vertex", s.diskOfRef(refV) === null);
  check("diskOfRef rejects a non-zero index", s.diskOfRef({ ...refC, index: 1 }) === null);

  // The same disk vertex twice → a diameter dimension.
  const m = s.addMeasurement("draw", refC, refC, { x: 80, y: 50 })!;
  check("same-disk pair gets the diameter axis", m.axis === "diameter", m.axis);
  let info = s.measureInfo(m)!;
  check("diameter value is 2·r", near(info.value, 20), `${info.value}`);
  check("diameter info carries the circle", !!info.circle && near(info.circle.r, 10));
  check(
    "diameter line runs rim to rim towards the label",
    !!info.dim && near(info.dim.a.x, 40) && near(info.dim.a.y, 50) && near(info.dim.b.x, 60) && near(info.dim.b.y, 50)
  );
  check("leader from the rim to a label outside the disk", info.ext.length === 1 && near(info.ext[0].b.x, 80));
  const pv = s.measurePreview(refC, refC, { x: 50, y: 90 })!;
  check("preview of a same-disk pair is a diameter too", !!pv.circle && near(pv.value, 20));

  // The label can move anywhere: the axis stays "diameter", the line follows the label.
  s.setMeasurementLabel(m.id, { x: 50, y: 10 });
  info = s.measureInfo(m)!;
  check("label move keeps the diameter axis", m.axis === "diameter" && !!info.dim && near(info.dim.b.y, 40));

  // Centre + another point is an ordinary point–point dimension.
  const mc = s.addMeasurement("draw", refC, refV, { x: 25, y: 25 })!;
  check("centre + corner is a plain distance", mc.axis !== "diameter" && near(s.measureInfo(mc)!.value, Math.hypot(50, 50)));

  // Driving: sets the hole's radius directly.
  check("driving the diameter succeeds", applyDrivingDimension(s, m.id, 30).length === 0);
  check("hole radius became target / 2", near(body.holes![0].radius, 15), `${body.holes![0].radius}`);
  check("dimension marked driving", m.driving === true && m.target === 30);
  check("driven value tracks", near(s.measureInfo(m)!.value, 30) && !s.measureInfo(m)!.violated);

  // A first driving dimension on the body scales it uniformly — the dimensioned hole
  // keeps its diameter (re-applied after the scale), and nothing is violated.
  const mEdge = s.addMeasurement("draw", refV, { kind: "vertex", bodyId: body.id, index: 1 }, { x: 50, y: -30 })!;
  check("outer driving dimension scales the body", applyDrivingDimension(s, mEdge.id, 200).length === 0);
  const vw = s.bodyControlWorld(body);
  check("body doubled", near(vw[1].x - vw[0].x, 200, 1e-6));
  check("dimensioned hole kept its radius through the scale", near(body.holes![0].radius, 15, 1e-9), `${body.holes![0].radius}`);
  check("diameter dimension not violated after the scale", !s.measureInfo(m)!.violated);

  // A plain sketch solve re-enforces too (e.g. after a radius-handle drag).
  s.setBodyRadius(body.id, 4, 0);
  solveSketch(s);
  check("solveSketch re-applies a driving diameter", near(body.holes![0].radius, 15, 1e-9), `${body.holes![0].radius}`);

  // Serialize / load round-trips the axis.
  const t = new Scene();
  t.load(JSON.parse(JSON.stringify(s.serialize())) as SceneData);
  const tm = t.getMeasurement(m.id)!;
  check("diameter axis survives load", tm.axis === "diameter" && near(t.measureInfo(tm)!.value, 30));

  // Removing the hole cascades the dimension away.
  s.removeBodyHole(body.id, 0);
  check("hole removal drops its diameter dimension", s.getMeasurement(m.id) === undefined && s.getMeasurement(mc.id) === undefined);

  // A one-point offset *body* is a disk too.
  const diskBody = s.addBody([{ x: 300, y: 300 }], 25, "offset");
  const refD: MeasureRef = { kind: "vertex", bodyId: diskBody.id, index: 0 };
  const dd = s.diskOfRef(refD);
  check("a disk body is a disk ref", !!dd && near(dd.r, 25) && near(dd.c.x, 300) && near(dd.c.y, 300));
  const md = s.addMeasurement("draw", refD, refD, { x: 300, y: 250 })!;
  check("disk body diameter dimension", md.axis === "diameter" && near(s.measureInfo(md)!.value, 50));
  check("driving a disk body's diameter", applyDrivingDimension(s, md.id, 80).length === 0 && near(diskBody.radius, 40));

  // A rim-handle drag stores a per-corner override; driving the diameter afterwards
  // must still change the *effective* radius (the override used to shadow the default).
  s.setBodyCornerRadius(diskBody.id, 0, 30);
  check("rim override shows in the driven value", near(s.measureInfo(md)!.value, 60) && s.measureInfo(md)!.violated === true);
  check("driving after a rim drag succeeds", applyDrivingDimension(s, md.id, 100).length === 0);
  check(
    "effective radius follows the target despite the override",
    near(s.diskOfRef(refD)!.r, 50) && !s.measureInfo(md)!.violated && diskBody.radii === undefined,
    `${s.diskOfRef(refD)!.r}`
  );
  // setDiskRadius is a no-op on a non-disk outline.
  const tri = s.addBody([{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 10, y: 10 }], 2);
  s.setDiskRadius(tri.id, 7);
  check("setDiskRadius ignores a polygon outline", near(tri.radius, 2));
}

// --- bodyInscribedRadius (largest disk that fits around a centre) --------------
{
  const s = new Scene();
  const sq = [
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ];
  const body = s.addBody(sq, 0, "fillet", [{ control: [{ x: 20, y: 50 }], radius: 10, round: "offset" }]);
  check("inscribed radius: nearest outer edge", near(s.bodyInscribedRadius(body, { x: 50, y: 20 }), 20, 1e-6));
  check("inscribed radius: a hole is closer than the outline", near(s.bodyInscribedRadius(body, { x: 50, y: 50 }), 20, 0.2));
  check("inscribed radius: centre outside → 0", s.bodyInscribedRadius(body, { x: 150, y: 50 }) === 0);
  check("inscribed radius: centre inside a hole → 0", s.bodyInscribedRadius(body, { x: 20, y: 50 }) === 0);
  check("inscribed radius: the excluded hole is ignored", near(s.bodyInscribedRadius(body, { x: 20, y: 50 }, 0), 20, 1e-6));
}

// --- radius dimensions on rounded corners --------------------------------------
{
  const s = new Scene();
  const sq = [
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 100, y: 100 },
    { x: 0, y: 100 },
  ];
  const body = s.addBody(sq, 10, "fillet");
  const ref0: MeasureRef = { kind: "vertex", bodyId: body.id, index: 0 };
  const ref2: MeasureRef = { kind: "vertex", bodyId: body.id, index: 2 };

  const c0 = s.cornerOfRef(ref0);
  check("cornerOfRef finds a rounded corner", !!c0 && near(c0.r, 10) && !!c0.arc && near(c0.arc.r, 10));
  check("corner arc centre sits inside the corner", !!c0?.arc && near(c0.arc.c.x, 10) && near(c0.arc.c.y, 10));
  const disk = s.addBody([{ x: 300, y: 300 }], 25, "offset");
  check("cornerOfRef rejects a disk centre", s.cornerOfRef({ kind: "vertex", bodyId: disk.id, index: 0 }) === null);
  check("cornerOfRef rejects a non-vertex ref", s.cornerOfRef({ kind: "bodyPoint", bodyId: body.id, local: { x: 0, y: 0 } }) === null);

  // The same corner vertex twice → a radius dimension.
  const m = s.addMeasurement("draw", ref0, ref0, { x: -30, y: -30 })!;
  check("same-corner pair gets the radius axis", m.axis === "radius", m.axis);
  let info = s.measureInfo(m)!;
  check("radius value is r", near(info.value, 10), `${info.value}`);
  check("radius info carries the fillet + single arrow", !!info.fillet && near(info.fillet.r, 10) && info.singleArrow === true);
  check(
    "radius line runs from the arc centre out to the arc towards the label",
    !!info.dim && near(info.dim.a.x, 10) && near(info.dim.a.y, 10) && near(Math.hypot(info.dim.b.x - 10, info.dim.b.y - 10), 10)
  );
  check("leader from the arc to a label beyond it", info.ext.length === 1);
  s.setMeasurementLabel(m.id, { x: 50, y: 50 });
  check("label move keeps the radius axis", s.getMeasurement(m.id)!.axis === "radius");

  // Driving on a uniform outline (no per-corner override) sets every corner via the default.
  check("driving the radius succeeds", applyDrivingDimension(s, m.id, 20).length === 0);
  check(
    "uniform outline: the default follows, every corner with it",
    near(body.radius, 20) && body.radii === undefined && s.bodyCornerRadii(body).every((r) => near(r, 20))
  );
  info = s.measureInfo(m)!;
  check("dimension marked driving, value tracks", m.driving === true && near(info.value, 20) && !info.violated);

  // Once a corner carries its own override the outline isn't uniform any more: the
  // dimension drives just its corner (as an override).
  s.setBodyCornerRadius(body.id, 2, 5);
  info = s.measureInfo(m)!;
  check("dimensioned corner unaffected by another corner's override", near(info.value, 20) && !info.violated);
  check("driving again on a non-uniform outline succeeds", applyDrivingDimension(s, m.id, 8).length === 0);
  const radii = s.bodyCornerRadii(body);
  check("only the dimensioned corner changed", near(radii[0], 8) && near(radii[1], 20) && near(radii[2], 5) && near(radii[3], 20), radii.join(","));

  // The Ctrl-drag path: every corner at once, overrides dropped → uniform again.
  s.setOutlineRadiusUniform(body.id, 12);
  check(
    "uniform set drops overrides and sets the default",
    body.radii === undefined && near(body.radius, 12) && s.outlineRadiiUniform(body.id, null)
  );
  check("radius dimension reads violated after the direct resize", s.measureInfo(m)!.violated === true);
  solveSketch(s);
  check(
    "solveSketch re-applies a driving radius — to every corner of the uniform outline",
    s.bodyCornerRadii(body).every((r) => near(r, 8, 1e-9)) && !s.measureInfo(m)!.violated,
    s.bodyCornerRadii(body).join(",")
  );

  // A first driving distance dimension scales the body uniformly; the dimensioned
  // radius is put back afterwards (and never blocks the scale).
  const mEdge = s.addMeasurement("draw", ref0, { kind: "vertex", bodyId: body.id, index: 1 }, { x: 50, y: -30 })!;
  check("outer driving dimension scales the body", applyDrivingDimension(s, mEdge.id, 200).length === 0);
  const vw = s.bodyControlWorld(body);
  check("body doubled", near(vw[1].x - vw[0].x, 200, 1e-6));
  check(
    "dimensioned radius kept through the scale",
    s.bodyCornerRadii(body).every((r) => near(r, 8, 1e-6)) && !s.measureInfo(m)!.violated,
    s.bodyCornerRadii(body).join(",")
  );

  // Preview + serialize / load round-trip.
  const pv = s.measurePreview(ref2, ref2, { x: 120, y: 120 })!;
  check("preview of a same-corner pair is a radius too", !!pv.fillet && near(pv.value, 8, 1e-6));
  const t = new Scene();
  t.load(JSON.parse(JSON.stringify(s.serialize())) as SceneData);
  const tm = t.getMeasurement(m.id)!;
  check("radius axis survives load", tm.axis === "radius" && near(t.measureInfo(tm)!.value, 8, 1e-6));

  // Hole corners work the same way, scoped to the hole's own outline.
  const hb = s.addBody(
    sq.map((p) => ({ x: p.x + 300, y: p.y })),
    0,
    "fillet",
    [{ control: [{ x: 320, y: 20 }, { x: 380, y: 20 }, { x: 380, y: 80 }, { x: 320, y: 80 }], radius: 4 }]
  );
  const hr: MeasureRef = { kind: "vertex", bodyId: hb.id, index: 1, hole: 0 };
  const hm = s.addMeasurement("draw", hr, hr, { x: 350, y: 50 })!;
  check("hole corner radius dimension", hm.axis === "radius" && near(s.measureInfo(hm)!.value, 4));
  check(
    "driving a hole corner radius (uniform hole → all its corners, outer untouched)",
    applyDrivingDimension(s, hm.id, 6).length === 0 && near(hb.holes![0].radius, 6) && hb.holes![0].radii === undefined && near(hb.radius, 0)
  );

  // Removing the corner's vertex drops the dimension like any vertex ref.
  s.removeBodyVertex(hb.id, 1, 0);
  check("vertex removal drops its radius dimension", s.getMeasurement(hm.id) === undefined);
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll measurement checks passed.");
