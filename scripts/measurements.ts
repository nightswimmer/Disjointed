/**
 * Measurements: point/line references, axis selection from label placement, live value
 * updates as geometry moves, the parallel↔angle flip for line pairs, vertex-index
 * remapping across control-polygon edits, cascade removal, and serialize/load.
 */
import { Scene, MeasureRef, SceneData, measureAxisForPlacement } from "../src/model";

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

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll measurement checks passed.");
