/** Corner filleting + body editing: radius changes and vertex moves keep joints anchored. */
import { Scene } from "../src/model";
import { filletPolygon, roundedConvexBody, polygonArea, dist, add, vec } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

const finite = (pts: { x: number; y: number }[]) => pts.every((p) => Number.isFinite(p.x) && Number.isFinite(p.y));

type Pt = { x: number; y: number };
/** Count proper crossings between non-adjacent edges of a closed polygon. */
function selfIntersections(poly: Pt[]): number {
  const o = (p: Pt, q: Pt, r: Pt) => Math.sign((q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x));
  const cross = (a: Pt, b: Pt, c: Pt, d: Pt) => {
    const o1 = o(a, b, c), o2 = o(a, b, d), o3 = o(c, d, a), o4 = o(c, d, b);
    return o1 !== o2 && o3 !== o4 && o1 !== 0 && o2 !== 0 && o3 !== 0 && o4 !== 0;
  };
  // Drop near-coincident points so two fillets meeting tangentially at a maxed-out edge
  // (a valid touch) collapse to one shared vertex instead of registering as a crossing.
  const p: Pt[] = [];
  for (const v of poly) {
    const last = p[p.length - 1];
    if (!last || Math.hypot(v.x - last.x, v.y - last.y) > 0.5) p.push(v);
  }
  let count = 0;
  const n = p.length;
  for (let i = 0; i < n; i++)
    for (let j = i + 1; j < n; j++) {
      if ((i + 1) % n === j || (j + 1) % n === i) continue; // skip adjacent edges
      if (cross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n])) count++;
    }
  return count;
}

// --- filletPolygon ---
const sq = [{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }];
const rounded = filletPolygon(sq, 20);
check("fillet adds arc vertices", rounded.length > 4, `${rounded.length}`);
check("fillet output is finite", finite(rounded));
const areaR = Math.abs(polygonArea(rounded));
check("fillet trims a little area off the square", areaR > 9000 && areaR < 10000, `${areaR.toFixed(0)}`);
check("radius 0 returns the original polygon", filletPolygon(sq, 0).length === 4);

const ell = [
  { x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 40 },
  { x: 40, y: 40 }, { x: 40, y: 100 }, { x: 0, y: 100 },
]; // concave L (has a reflex corner)
const roundedL = filletPolygon(ell, 10);
check("concave fillet stays valid", finite(roundedL) && Math.abs(polygonArea(roundedL)) > 0, `area ${Math.abs(polygonArea(roundedL)).toFixed(0)}`);
// The reflex corner at (40,40) must round into the *notch* (x>40, y>40), not the material
// (x<40, y<40). The old code flipped the arc to the wrong side.
const nearReflex = (pred: (p: { x: number; y: number }) => boolean) =>
  roundedL.filter((p) => Math.hypot(p.x - 40, p.y - 40) < 14 && pred(p)).length;
check("reflex corner rounds into the notch", nearReflex((p) => p.x > 40 && p.y > 40) > 0, `${nearReflex((p) => p.x > 40 && p.y > 40)} pts`);
check("reflex fillet stays off the material side", nearReflex((p) => p.x < 40 - 0.1 && p.y < 40 - 0.1) === 0);

// --- editing keeps attached joints anchored ---
const s = new Scene();
const body = s.addBody(sq);
const j = s.addJoint(body.id, { x: 20, y: 20 });

const beforeRadius = s.jointWorld(j);
s.setBodyRadius(body.id, 15);
check("joint stays put when rounding the body", dist(beforeRadius, s.jointWorld(j)) < 1e-9, `moved ${dist(beforeRadius, s.jointWorld(j)).toExponential(1)}`);
check("rounded body still has area", Math.abs(polygonArea(s.bodyWorldVerts(body))) > 0);

const areaBefore = Math.abs(polygonArea(s.bodyWorldVerts(body)));
const beforeMove = s.jointWorld(j);
s.moveBodyVertex(body.id, 0, { x: -40, y: -40 }); // drag a corner out
check("joint stays put when moving a vertex", dist(beforeMove, s.jointWorld(j)) < 1e-9, `moved ${dist(beforeMove, s.jointWorld(j)).toExponential(1)}`);
check("moving a vertex changes the body area", Math.abs(Math.abs(polygonArea(s.bodyWorldVerts(body))) - areaBefore) > 1);

// --- narrow-neck fillet doesn't overlap / fold (shared-edge budget) ---
const neck = [
  { x: 0, y: 0 }, { x: 300, y: 0 }, { x: 300, y: 40 }, { x: 175, y: 50 },
  { x: 165, y: 200 }, { x: 135, y: 200 }, { x: 125, y: 50 }, { x: 0, y: 40 },
]; // wide base + thin 30px-wide neck rising to the top
for (const r of [16, 30, 60, 200]) {
  const f = filletPolygon(neck, r);
  check(`narrow neck fillet stays simple at r=${r}`, finite(f) && selfIntersections(f) === 0, `${selfIntersections(f)} crossings`);
}

// --- moveJoint keeps an attached joint inside its body ---
const m = new Scene();
const mb = m.addBody([{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }]);
const mj = m.addJoint(mb.id, { x: 50, y: 50 });

m.moveJoint(mj.id, { x: 30, y: 0 }); // stays well inside
check("in-body joint move lands where asked", dist(m.jointWorld(mj), { x: 80, y: 50 }) < 1e-9);

m.moveJoint(mj.id, { x: 500, y: 0 }); // way past the right edge → clamp to the outline
const clamped = m.jointWorld(mj);
check("joint move past the edge clamps to the outline", dist(clamped, { x: 100, y: 50 }) < 1e-6, `at (${clamped.x.toFixed(2)}, ${clamped.y.toFixed(2)})`);

m.moveJoint(mj.id, { x: 200, y: -200 }); // diagonal escape → nearest boundary point
const corner = m.jointWorld(mj);
check("diagonal escape clamps to the nearest boundary point", m.pointInBody(mb, corner) || dist(corner, { x: 100, y: 0 }) < 1e-6, `at (${corner.x.toFixed(2)}, ${corner.y.toFixed(2)})`);

const mg = m.addGround(mj.id, m.jointWorld(mj));
m.moveJoint(mj.id, { x: 500, y: 500 });
check("ground anchor follows the clamped position", dist(mg.anchor, m.jointWorld(mj)) < 1e-9);

const freeJ = m.addFreeJoint({ x: 300, y: 300 });
m.moveJoint(freeJ.id, { x: 500, y: 500 });
check("free joint still moves without clamping", dist(m.jointWorld(freeJ), { x: 800, y: 800 }) < 1e-9);

// --- node ↔ joint link: a joint coincident with a control vertex is stuck to it ---
const lk = new Scene();
const l1 = lk.addFreeJoint({ x: 0, y: 0 });
const l2 = lk.addFreeJoint({ x: 100, y: 0 });
const l3 = lk.addFreeJoint({ x: 50, y: 80 });
const lb = lk.buildBodyFromJoints([l1.id, l2.id, l3.id], 20)!;
check("joint-built body created", !!lb);

// buildBodyFromJoints stores one control point per joint, in order → vertex 0 ↔ l1.
lk.moveBodyVertex(lb.id, 0, { x: -10, y: -5 });
check("moving a body node carries its joint", dist(lk.jointWorld(l1), { x: -10, y: -5 }) < 1e-6, `at (${lk.jointWorld(l1).x.toFixed(2)}, ${lk.jointWorld(l1).y.toFixed(2)})`);
check("other joints stay anchored", dist(lk.jointWorld(l2), { x: 100, y: 0 }) < 1e-6 && dist(lk.jointWorld(l3), { x: 50, y: 80 }) < 1e-6);

// The reverse: moving the joint drags its node, reshaping the body.
lk.moveJoint(l2.id, { x: 15, y: 10 });
check("joint landed where asked", dist(lk.jointWorld(l2), { x: 115, y: 10 }) < 1e-6);
check("moving the joint carries its body node", lk.bodyControlWorld(lb).some((v) => dist(v, { x: 115, y: 10 }) < 1e-6));

// A grounded linked joint keeps its anchor in step.
const lg = lk.addGround(l3.id, lk.jointWorld(l3));
lk.moveBodyVertex(lb.id, 2, { x: 0, y: 12 });
check("ground anchor follows a linked node move", dist(lg.anchor, lk.jointWorld(l3)) < 1e-9 && dist(lg.anchor, { x: 50, y: 92 }) < 1e-6);

// A joint *not* on a node still moves freely without reshaping the body.
const inner = lk.addJoint(lb.id, { x: 50, y: 30 });
const ctrlBefore = lk.bodyControlWorld(lb).map((v) => ({ x: v.x, y: v.y }));
lk.moveJoint(inner.id, { x: 5, y: 5 });
const ctrlAfter = lk.bodyControlWorld(lb);
check("non-node joint move leaves the shape alone", ctrlBefore.every((v, i) => dist(v, ctrlAfter[i]) < 1e-9));
check("non-node joint moved normally", dist(lk.jointWorld(inner), { x: 55, y: 35 }) < 1e-6);

// --- add / remove control vertices ---
const e = new Scene();
const eb = e.addBody([{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }]);
const ej = e.addJoint(eb.id, { x: 50, y: 50 });

const beforeInsert = e.jointWorld(ej);
e.insertBodyVertex(eb.id, 1, { x: 50, y: 0 }); // node on the bottom edge (between v0 and v1)
check("insert adds a control vertex", eb.controlLocal.length === 5, `${eb.controlLocal.length}`);
check("joint stays put when inserting a vertex", dist(beforeInsert, e.jointWorld(ej)) < 1e-9, `moved ${dist(beforeInsert, e.jointWorld(ej)).toExponential(1)}`);

e.removeBodyVertex(eb.id, 1);
check("remove drops a control vertex", eb.controlLocal.length === 4, `${eb.controlLocal.length}`);
e.removeBodyVertex(eb.id, 0);
check("remove down to a triangle works", eb.controlLocal.length === 3, `${eb.controlLocal.length}`);
e.removeBodyVertex(eb.id, 0);
check("remove is a no-op at the 3-vertex minimum", eb.controlLocal.length === 3, `${eb.controlLocal.length}`);

// --- per-corner radii (v15) ---
// geometry: an array radius rounds only the corners it names.
const oneCorner = filletPolygon(sq, [20, 0, 0, 0]);
check("per-corner fillet rounds only the named corner", oneCorner.length > 4 && oneCorner.length < rounded.length, `${oneCorner.length} pts vs ${rounded.length} all-round`);
check("unrounded corners stay exactly sharp", oneCorner.some((p) => p.x === 100 && p.y === 0));
const areaOne = Math.abs(polygonArea(oneCorner));
check("one fillet trims a quarter of the all-round loss", areaOne > areaR && areaOne < 10000, `${areaOne.toFixed(0)}`);

// offset mode: per-point margins = hull of different-size circles.
const stadium = roundedConvexBody([{ x: 0, y: 0 }, { x: 100, y: 0 }], [10, 30]);
const xs = stadium.map((p) => p.x);
const maxX = Math.max(...xs);
const minX = Math.min(...xs);
check("per-point offset grows each end by its own margin", maxX > 125 && maxX <= 130 && minX < -8 && minX >= -10, `x ∈ [${minX.toFixed(1)}, ${maxX.toFixed(1)}]`);

// scene: overriding one corner keeps joints anchored and the others on the default.
const pc = new Scene();
const pb = pc.addBody([{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }], 10);
const pj = pc.addJoint(pb.id, { x: 50, y: 50 });
const pjBefore = pc.jointWorld(pj);
pc.setBodyCornerRadius(pb.id, 2, 30);
check("corner override recorded (others null)", pb.radii?.[2] === 30 && pb.radii?.[0] === null, JSON.stringify(pb.radii));
check("effective radii mix override + default", JSON.stringify(pc.bodyCornerRadii(pb)) === "[10,10,30,10]", JSON.stringify(pc.bodyCornerRadii(pb)));
check("joint stays put on a per-corner change", dist(pjBefore, pc.jointWorld(pj)) < 1e-9);

// insert / remove control vertices keep overrides on the same physical corner.
pc.insertBodyVertex(pb.id, 1, { x: 50, y: 0 }); // node on the bottom edge, before the override
check("insert shifts the override with its corner", pb.radii?.length === 5 && pb.radii?.[3] === 30, JSON.stringify(pb.radii));
pc.removeBodyVertex(pb.id, 3);
check("removing the overridden corner drops the whole array", pb.radii === undefined, JSON.stringify(pb.radii));

// clearing an override back to the default.
pc.setBodyCornerRadius(pb.id, 0, 25);
pc.setBodyCornerRadius(pb.id, 0, null);
check("clearing the last override drops the array", pb.radii === undefined);

// mirror reverses the override array along with the renumbered corners.
const mi = new Scene();
const mib = mi.addBody([{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }], 10);
mi.setBodyCornerRadius(mib.id, 0, 40);
const cornerBefore = mi.bodyControlWorld(mib)[0];
const centroidX = mib.pos.x; // the fillet shifts the centroid off (50,50) — reflect about the real axis
mi.mirrorBody(mib.id, "h");
const n = mib.controlLocal.length;
check("mirror moves the override to the renumbered corner", mib.radii?.[n - 1] === 40, JSON.stringify(mib.radii));
check("the overridden corner is the reflected one", dist(mi.bodyControlWorld(mib)[n - 1], { x: 2 * centroidX - cornerBefore.x, y: cornerBefore.y }) < 1e-6);

// scale scales overrides with the body.
mi.scaleBody(mib.id, 2);
check("scale doubles the override", mib.radii?.[n - 1] === 80, JSON.stringify(mib.radii));

// save / load and copy / paste round-trip the overrides.
const rt = new Scene();
rt.load(JSON.parse(JSON.stringify(mi.serialize())));
const rtb = rt.bodies[0];
check("radii survive save/load", rtb.radii?.[n - 1] === 80, JSON.stringify(rtb.radii));
const clip = rt.extractBody(rtb.id)!;
const pasted = rt.insertSelection(clip, { x: 500, y: 500 })!;
const pastedBody = rt.getBody(pasted.bodyIds[0])!;
check("radii survive copy/paste", pastedBody.radii?.[n - 1] === 80, JSON.stringify(pastedBody.radii));
check("pasted outline uses the override", Math.abs(Math.abs(polygonArea(rt.bodyWorldVerts(pastedBody))) - Math.abs(polygonArea(rt.bodyWorldVerts(rtb)))) < 1e-6);

// --- editable holes (v16) ---
const rect4 = (x0: number, y0: number, x1: number, y1: number) => [
  { x: x0, y: y0 }, { x: x1, y: y0 }, { x: x1, y: y1 }, { x: x0, y: y1 },
];
const hs = new Scene();
const hb = hs.addBody(rect4(0, 0, 100, 100), 0, "fillet", [
  rect4(30, 30, 50, 50), // square cut-out
  { control: [{ x: 70, y: 70 }], radius: 8, round: "offset" }, // circular (disk) hole
]);
check("holes become editable shapes", hb.holes?.length === 2 && hb.holesLocal?.length === 2,
  `${hb.holes?.length} shapes, ${hb.holesLocal?.length} derived`);
const disk = hs.bodyHolesWorld(hb)[1];
const diskErr = Math.max(...disk.map((p) => Math.abs(dist(p, { x: 70, y: 70 }) - 8)));
check("disk hole derives as a true circle", disk.length >= 24 && diskErr < 1e-9,
  `${disk.length} pts, max radius error ${diskErr.toExponential(2)}`);
const expectArea = 10000 - 400 - Math.PI * 64;
check("both holes subtract from mass", Math.abs(1 / hb.invMass - expectArea) < 10,
  `area ${(1 / hb.invMass).toFixed(1)} vs ~${expectArea.toFixed(1)}`);

// Measurements + sketch refs on hole geometry resolve and track edits.
const mRes = hs.resolveMeasureRef({ kind: "vertex", bodyId: hb.id, index: 0, hole: 0 });
check("hole vertex resolves as a measure ref", mRes?.kind === "point" && dist(mRes.p, { x: 30, y: 30 }) < 1e-9,
  mRes?.kind === "point" ? `(${mRes.p.x}, ${mRes.p.y})` : "null");
const eRes = hs.resolveMeasureRef({ kind: "edge", bodyId: hb.id, index: 0, hole: 0 });
check("hole edge resolves as a measure ref", eRes?.kind === "line", `${eRes?.kind}`);
check("disk hole has no edge refs", hs.resolveMeasureRef({ kind: "edge", bodyId: hb.id, index: 0, hole: 1 }) === null, "1-point outline");

// Editing a hole vertex reshapes the hole only; the outer polygon stays put.
const outerBefore = hs.bodyControlWorld(hb).map((p) => ({ x: p.x, y: p.y }));
const areaBeforeMove = 1 / hb.invMass;
hs.moveBodyVertex(hb.id, 0, { x: -10, y: -10 }, 0);
check("hole vertex move grows the cut-out", 1 / hb.invMass < areaBeforeMove - 50,
  `area ${(1 / hb.invMass).toFixed(1)}`);
check("outer outline untouched by a hole edit",
  hs.bodyControlWorld(hb).every((p, i) => dist(p, outerBefore[i]) < 1e-9), "outer verts");
const movedRes = hs.resolveMeasureRef({ kind: "vertex", bodyId: hb.id, index: 0, hole: 0 });
check("hole ref tracks the moved vertex", movedRes?.kind === "point" && dist(movedRes.p, { x: 20, y: 20 }) < 1e-9,
  movedRes?.kind === "point" ? `(${movedRes.p.x.toFixed(1)}, ${movedRes.p.y.toFixed(1)})` : "null");

// Insert / remove hole vertices remap hole refs (and only same-hole refs).
const hm = hs.addMeasurement("draw",
  { kind: "vertex", bodyId: hb.id, index: 2, hole: 0 },
  { kind: "vertex", bodyId: hb.id, index: 2 }, // outer vertex 2 must NOT shift
  { x: 0, y: -20 })!;
hs.insertBodyVertex(hb.id, 1, { x: 50, y: 25 }, 0);
const hmRefA = hm.refA as { index: number; hole?: number };
const hmRefB = hm.refB as { index: number; hole?: number };
check("hole vertex insert shifts same-hole refs", hmRefA.index === 3 && hmRefA.hole === 0, `index ${hmRefA.index}`);
check("outer refs unaffected by hole inserts", hmRefB.index === 2 && hmRefB.hole === undefined, `index ${hmRefB.index}`);
hs.removeBodyVertex(hb.id, 1, 0);
check("hole vertex remove shifts refs back", (hm.refA as { index: number }).index === 2, `index ${(hm.refA as { index: number }).index}`);

// Per-corner radius on a hole corner.
hs.setBodyCornerRadius(hb.id, 0, 6, 0);
check("hole corner radius override recorded", hb.holes?.[0].radii?.[0] === 6, JSON.stringify(hb.holes?.[0].radii));
check("hole outline rounds that corner", (hb.holesLocal?.[0].length ?? 0) > 4, `${hb.holesLocal?.[0].length} pts`);

// Mirror remaps hole refs within their own outline.
const mScene = new Scene();
const mBody = mScene.addBody(rect4(0, 0, 100, 100), 0, "fillet", [rect4(20, 20, 40, 40)]);
const mm = mScene.addMeasurement("draw",
  { kind: "vertex", bodyId: mBody.id, index: 0, hole: 0 },
  { kind: "vertex", bodyId: mBody.id, index: 0 },
  { x: 0, y: -20 })!;
const mAxisX = mBody.pos.x; // the hole shifts the centroid off x=50 — reflect about the real axis
mScene.mirrorBody(mBody.id, "h");
const mmA = mm.refA as { index: number; hole?: number };
check("mirror remaps hole vertex refs in the hole's own outline", mmA.index === 3 && mmA.hole === 0, `index ${mmA.index}`);
const mmRes = mScene.resolveMeasureRef(mm.refA);
check("remapped hole ref names the reflected corner",
  mmRes?.kind === "point" && dist(mmRes.p, { x: 2 * mAxisX - 20, y: 20 }) < 1e-6,
  mmRes?.kind === "point" ? `(${mmRes.p.x.toFixed(1)}, ${mmRes.p.y.toFixed(1)})` : "null");

// Copy/paste carries hole shapes (control + rounding), and save/load round-trips them.
const hClip = hs.extractBody(hb.id)!;
const hPaste = hs.insertSelection(hClip, { x: 500, y: 500 })!;
const hPasted = hs.getBody(hPaste.bodyIds[0])!;
check("paste carries editable holes", hPasted.holes?.length === 2 && hPasted.holes[0].radii?.[0] === 6 && hPasted.holes[1].round === "offset",
  `${hPasted.holes?.length} holes`);
const hLoad = new Scene();
hLoad.load(JSON.parse(JSON.stringify(hs.serialize())));
const hLoaded = hLoad.getBody(hb.id)!;
check("holes survive save/load with their rounding", hLoaded.holes?.length === 2 && hLoaded.holes[0].radii?.[0] === 6 && hLoaded.holes[1].radius === 8,
  `${hLoaded.holes?.length} holes`);

// Legacy (≤ v15) files carry only baked holesLocal loops — they load as editable holes.
const legacyData = JSON.parse(JSON.stringify(hs.serialize()));
for (const b of legacyData.bodies) delete b.holes; // simulate a pre-v16 file
const legacy = new Scene();
legacy.load(legacyData);
const lHoles = legacy.getBody(hb.id)!.holes;
check("legacy baked holes load as radius-0 editable holes",
  lHoles?.length === 2 && lHoles.every((h) => h.radius === 0) && (lHoles[0].controlLocal.length > 4),
  `${lHoles?.length} holes, ${lHoles?.[0].controlLocal.length} ctrl pts`);

// Removing a whole hole cascades its refs and shifts later holes' refs down.
const dScene = new Scene();
const dBody = dScene.addBody(rect4(0, 0, 100, 100), 0, "fillet", [rect4(10, 10, 20, 20), rect4(60, 60, 80, 80)]);
const dGone = dScene.addMeasurement("draw",
  { kind: "vertex", bodyId: dBody.id, index: 0, hole: 0 },
  { kind: "vertex", bodyId: dBody.id, index: 0 }, { x: 0, y: -10 })!;
const dKept = dScene.addMeasurement("draw",
  { kind: "vertex", bodyId: dBody.id, index: 1, hole: 1 },
  { kind: "vertex", bodyId: dBody.id, index: 1 }, { x: 0, y: -10 })!;
dScene.removeBodyHole(dBody.id, 0);
check("removing a hole drops its refs' measurements", dScene.getMeasurement(dGone.id) === undefined, "cascaded");
const dRef = dScene.getMeasurement(dKept.id)?.refA as { hole?: number } | undefined;
check("later holes' refs shift down", dRef?.hole === 0 && dBody.holes?.length === 1, `hole ${dRef?.hole}`);

// addBodyHole: cut a hole into an existing body (the UI hole tool's model op).
const aScene = new Scene();
const aBody = aScene.addBody(rect4(0, 0, 100, 100));
const aJoint = aScene.addJoint(aBody.id, { x: 10, y: 10 });
const aJw = aScene.jointWorld(aJoint);
const aMassBefore = 1 / aBody.invMass;
const aIdx = aScene.addBodyHole(aBody.id, rect4(30, 30, 70, 70));
check("addBodyHole returns the new hole's index", aIdx === 0, `${aIdx}`);
check("added hole derives its sampled loop", aBody.holesLocal?.length === 1 && (aBody.holesLocal[0].length ?? 0) >= 4);
check("added hole subtracts from the mass", 1 / aBody.invMass < aMassBefore - 1000, `${(1 / aBody.invMass).toFixed(0)} < ${aMassBefore.toFixed(0)}`);
check("joint stays anchored through the cut's centroid shift", dist(aJw, aScene.jointWorld(aJoint)) < 1e-9);
check("too few points is rejected", aScene.addBodyHole(aBody.id, [{ x: 40, y: 40 }, { x: 60, y: 60 }]) === null && aBody.holes?.length === 1);
const aDisk = aScene.addBodyHole(aBody.id, { control: [{ x: 80, y: 15 }], radius: 8, round: "offset" });
check("a 1-point offset spec cuts a disk hole, appended after existing holes", aDisk === 1 && aBody.holes?.[1].round === "offset");
// World → local conversion holds on a moved + rotated body.
aScene.rotateBody(aBody.id, aBody.pos, Math.PI / 3);
aScene.moveBody(aBody.id, { x: 25, y: -13 });
const aTarget = aScene.jointWorld(aJoint); // a known world point inside the body
const aIdx2 = aScene.addBodyHole(aBody.id, {
  control: [aTarget, add(aTarget, vec(12, 0)), add(aTarget, vec(12, 12))].map((p) => vec(p.x, p.y)),
});
const aBack = aScene.bodyHoleControlWorld(aBody, aIdx2!)[0];
check("hole control converts world → local on a rotated body", dist(aBack, aTarget) < 1e-9, `off ${dist(aBack, aTarget).toExponential(1)}`);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
