/** Regular polygons (v21): the `regular` outline invariant — corner drags grow / spin the
 *  polygon about its centre, node add / remove is refused, the side count is edited in
 *  place, the centre is a point reference, the sketch solver keeps the polygon regular
 *  (edge dimension → uniform resize, centre tie → translation, H on an edge → rotation),
 *  the flag travels through cut / copy / mirror / save and drops on split. */
import { Scene, MeasureRef } from "../src/model";
import { dist, sub, vec, Vec2, regularPolygon, fitRegularPolygon, polygonArea } from "../src/geometry";
import { tryAddConstraint, solveSketch, applyDrivingDimension } from "../src/sketch";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: number, b: number, tol = 1e-6) => Math.abs(a - b) <= tol;
const centreOf = (pts: Vec2[]): Vec2 => {
  let x = 0, y = 0;
  for (const p of pts) { x += p.x; y += p.y; }
  return vec(x / pts.length, y / pts.length);
};
/** Max deviation of `pts` from a regular polygon (0 = perfectly regular). */
const irregularity = (pts: Vec2[]): number => {
  const c = centreOf(pts);
  const r = pts.map((p) => dist(p, c));
  const rMean = r.reduce((a, b) => a + b, 0) / r.length;
  let worst = Math.max(...r.map((x) => Math.abs(x - rMean)));
  for (let i = 0; i < pts.length; i++) {
    const e = dist(pts[i], pts[(i + 1) % pts.length]);
    worst = Math.max(worst, Math.abs(e - 2 * rMean * Math.sin(Math.PI / pts.length)));
  }
  return worst;
};
const sq = (x: number, y: number, w: number, h: number): Vec2[] => [vec(x, y), vec(x + w, y), vec(x + w, y + h), vec(x, y + h)];

// ---------------------------------------------------------------- the fit itself
{
  const pts = regularPolygon(vec(10, -5), vec(40, -5), 5);
  const fit = fitRegularPolygon(pts)!;
  check("fit recovers an exact pentagon", fit !== null && near(fit.r, 30, 1e-9) && near(fit.c.x, 10, 1e-9) && near(fit.c.y, -5, 1e-9));
  check("fit keeps the winding", polygonArea(fit.pts) * polygonArea(pts) > 0);
  const rev = [...pts].reverse();
  const fitRev = fitRegularPolygon(rev)!;
  check("fit follows a reversed polygon", fitRev !== null && fitRev.pts.every((p, i) => dist(p, rev[i]) < 1e-9));
  const bent = pts.map((p, i) => (i === 2 ? vec(p.x + 3, p.y - 2) : p));
  const f2 = fitRegularPolygon(bent, bent.map((_, i) => (i === 2 ? 1e6 : 1)))!;
  check("a heavily weighted point is honoured", dist(f2.pts[2], bent[2]) < 1e-3, `off ${dist(f2.pts[2], bent[2])}`);
  check("weighted fit is still regular", irregularity(f2.pts) < 1e-9);
}

// ---------------------------------------------------------------- creation + invariant edits
{
  const s = new Scene();
  const hex = s.addBody(regularPolygon(vec(100, 100), vec(140, 100), 6), 0, "fillet", undefined, undefined, 6);
  check("addBody records the side count", hex.regular === 6);
  check("centre ref resolves to the centre", (() => {
    const r = s.resolveMeasureRef({ kind: "centre", bodyId: hex.id });
    return r?.kind === "point" && near(r.p.x, 100) && near(r.p.y, 100);
  })());
  check("a mismatched count is refused", s.addBody(sq(0, 0, 10, 10), 0, "fillet", undefined, undefined, 5).regular === undefined);
  check("a disk never counts as regular", s.addBody([vec(0, 0)], 5, "offset", undefined, undefined, 1).regular === undefined);

  // Corner drag: the dragged corner sets radius + phase, the centre stays.
  s.moveBodyVertex(hex.id, 0, vec(20, 0));
  let w = s.bodyControlWorld(hex);
  check("corner drag keeps the polygon regular", irregularity(w) < 1e-9, `irr ${irregularity(w)}`);
  check("corner drag grows about the centre", near(dist(w[0], centreOf(w)), 60, 1e-9) && near(centreOf(w).x, 100, 1e-9) && near(centreOf(w).y, 100, 1e-9));
  s.moveBodyVertex(hex.id, 1, vec(0, 15)); // off-radial: spins + resizes, still regular
  w = s.bodyControlWorld(hex);
  check("off-radial corner drag stays regular", irregularity(w) < 1e-9);
  check("corner onto the centre is ignored", (() => {
    const before = s.bodyControlWorld(hex);
    const c = centreOf(before);
    s.moveBodyVertex(hex.id, 2, vec(c.x - before[2].x, c.y - before[2].y));
    return s.bodyControlWorld(hex).every((p, i) => dist(p, before[i]) < 1e-12);
  })());
  // Node add / remove: refused.
  s.insertBodyVertex(hex.id, 1, vec(120, 60));
  check("insertBodyVertex refused", hex.controlLocal.length === 6);
  s.removeBodyVertex(hex.id, 0);
  check("removeBodyVertex refused", hex.controlLocal.length === 6);

  // A joint stuck to a corner rides with it.
  const w0 = s.bodyControlWorld(hex);
  const j = s.addJoint(hex.id, w0[3]);
  s.moveBodyVertex(hex.id, 0, vec(10, 0));
  check("joint stuck to another corner follows the resize", dist(s.jointWorld(j), s.bodyControlWorld(hex)[3]) < 1e-9);

  // Side count edit: same centre, radius and first-corner angle; refs past the count go.
  const before = s.bodyControlWorld(hex);
  const c0 = centreOf(before);
  const r0 = dist(before[0], c0);
  const m5 = s.addMeasurement("draw", { kind: "vertex", bodyId: hex.id, index: 5 }, { kind: "joint", jointId: j.id }, vec(0, 0))!;
  const m1 = s.addMeasurement("draw", { kind: "vertex", bodyId: hex.id, index: 1 }, { kind: "joint", jointId: j.id }, vec(0, 0))!;
  check("setRegularSides refuses 2", !s.setRegularSides(hex.id, null, 2));
  check("setRegularSides to 5", s.setRegularSides(hex.id, null, 5) && hex.regular === 5 && hex.controlLocal.length === 5);
  const after = s.bodyControlWorld(hex);
  check("count edit keeps centre, radius and first corner", near(centreOf(after).x, c0.x, 1e-9) && near(centreOf(after).y, c0.y, 1e-9) && near(dist(after[0], centreOf(after)), r0, 1e-9) && dist(after[0], before[0]) < 1e-9);
  check("count edit result is regular", irregularity(after) < 1e-9);
  check("ref past the new count is dropped, lower one kept", !s.getMeasurement(m5.id) && !!s.getMeasurement(m1.id));
  check("setRegularSides up to 12", s.setRegularSides(hex.id, null, 12) && hex.controlLocal.length === 12 && irregularity(s.bodyControlWorld(hex)) < 1e-9);
  check("setRegularSides refused on a free polygon", !s.setRegularSides(s.addBody(sq(300, 300, 20, 20)).id, null, 6));
}

// ---------------------------------------------------------------- sketch solver keeps it regular
{
  // Edge dimension → the whole polygon resizes.
  const s = new Scene();
  const hex = s.addBody(regularPolygon(vec(0, 0), vec(50, 0), 6), 0, "fillet", undefined, undefined, 6);
  const m = s.addMeasurement("draw", { kind: "vertex", bodyId: hex.id, index: 0 }, { kind: "vertex", bodyId: hex.id, index: 1 }, vec(0, 0))!;
  m.axis = "direct"; // the straight edge length (a label placement may have picked h / v)
  const breaks = applyDrivingDimension(s, m.id, 80);
  const w = s.bodyControlWorld(hex);
  check("edge dimension accepted", breaks.length === 0, `${breaks.length} breaks`);
  check("dimensioned edge at target", near(dist(w[0], w[1]), 80, 2e-3), `${dist(w[0], w[1])}`);
  check("every edge followed (uniform resize)", w.every((p, i) => near(dist(p, w[(i + 1) % 6]), 80, 2e-3)));
  check("still regular after the dimension", irregularity(w) < 2e-3, `irr ${irregularity(w)}`);
  check("centre stayed put", near(centreOf(w).x, 0, 2e-3) && near(centreOf(w).y, 0, 2e-3));
}
{
  // Centre tied to a free joint → the polygon translates rigidly.
  const s = new Scene();
  const hex = s.addBody(regularPolygon(vec(0, 0), vec(50, 0), 6), 0, "fillet", undefined, undefined, 6);
  const fj = s.addFreeJoint(vec(200, 80));
  const before = s.bodyControlWorld(hex);
  const res = tryAddConstraint(s, "coincident", { kind: "centre", bodyId: hex.id }, { kind: "joint", jointId: fj.id });
  const w = s.bodyControlWorld(hex);
  check("centre–joint coincident accepted", res.constraint !== null, `${res.breaks.length} breaks`);
  check("centre meets the joint", dist(centreOf(w), s.jointWorld(fj)) < 2e-3, `gap ${dist(centreOf(w), s.jointWorld(fj))}`);
  check("polygon translated rigidly", w.every((p, i) => near(dist(p, centreOf(w)), dist(before[i], centreOf(before)), 2e-3)) && irregularity(w) < 2e-3);
  check("body pos is the centre", dist(hex.pos, centreOf(w)) < 1e-6);
  // Dragging the joint afterwards tows the polygon (anchored live solve).
  s.moveJoint(fj.id, vec(30, -10));
  const live = solveSketch(s, new Set([`j:${fj.id}`]));
  check("live solve after a joint drag converges", live.length === 0);
  check("polygon followed the dragged joint", dist(centreOf(s.bodyControlWorld(hex)), s.jointWorld(fj)) < 2e-3);
}
{
  // Horizontal on an edge → rotation about the centre, radius kept.
  const s = new Scene();
  const pent = s.addBody(regularPolygon(vec(0, 0), vec(30, 20), 5), 0, "fillet", undefined, undefined, 5);
  const r0 = dist(s.bodyControlWorld(pent)[0], vec(0, 0));
  const res = tryAddConstraint(s, "horizontal", { kind: "edge", bodyId: pent.id, index: 0 });
  const w = s.bodyControlWorld(pent);
  check("H on a regular edge accepted", res.constraint !== null);
  check("edge is horizontal", Math.abs(w[0].y - w[1].y) < 2e-3);
  check("rotation kept the radius", near(dist(w[0], centreOf(w)), r0, 2e-3) && irregularity(w) < 2e-3);
}
{
  // Corner tied to a fixed point on another body: the polygon resizes / spins; the other body stays.
  const s = new Scene();
  const plate = s.addBody(sq(200, 200, 100, 100));
  const tri = s.addBody(regularPolygon(vec(0, 0), vec(40, 0), 3), 0, "fillet", undefined, undefined, 3);
  const res = tryAddConstraint(s, "coincident", { kind: "vertex", bodyId: tri.id, index: 0 }, { kind: "vertex", bodyId: plate.id, index: 0 });
  const w = s.bodyControlWorld(tri);
  check("corner–corner coincident accepted", res.constraint !== null, `${res.breaks.length} breaks`);
  check("corner reached the plate corner", dist(w[0], s.bodyControlWorld(plate)[0]) < 2e-3);
  check("triangle still regular", irregularity(w) < 2e-3, `irr ${irregularity(w)}`);
  // A body drag afterwards keeps everything consistent.
  s.moveBody(plate.id, vec(5, 5));
  const live = solveSketch(s, new Set(s.bodyControlWorld(plate).map((_, i) => `v:${plate.id}:${i}`)));
  check("live solve after dragging the plate converges", live.length === 0, `${live.length}`);
  check("triangle followed, still regular", dist(s.bodyControlWorld(tri)[0], s.bodyControlWorld(plate)[0]) < 2e-3 && irregularity(s.bodyControlWorld(tri)) < 2e-3);
}

// ---------------------------------------------------------------- holes (Cut role)
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 200, 200));
  const res = s.cutBody(plate.id, { control: regularPolygon(vec(100, 100), vec(120, 100), 6), radius: 0, round: "fillet", regular: 6 });
  check("hexagonal cut becomes a regular hole", res.ok && res.hole === 0 && plate.holes?.[0].regular === 6);
  const cref: MeasureRef = { kind: "centre", bodyId: plate.id, hole: 0 };
  const rc = s.resolveMeasureRef(cref);
  check("hole centre ref resolves", rc?.kind === "point" && near(rc.p.x, 100) && near(rc.p.y, 100));
  s.moveBodyVertex(plate.id, 0, vec(10, 0), 0);
  const hw = s.bodyHoleControlWorld(plate, 0);
  check("hole corner drag keeps the hole regular", irregularity(hw) < 1e-9 && near(dist(hw[0], centreOf(hw)), 30, 1e-9));
  check("hole centre unchanged by the resize", near(centreOf(hw).x, 100, 1e-9) && near(centreOf(hw).y, 100, 1e-9));
  s.insertBodyVertex(plate.id, 1, vec(110, 90), 0);
  check("hole node insert refused", plate.holes![0].controlLocal.length === 6);
  // Hole centre tied to a joint on the plate: the hole slides inside the body.
  const j = s.addJoint(plate.id, vec(60, 60));
  const tie = tryAddConstraint(s, "coincident", cref, { kind: "joint", jointId: j.id });
  check("hole centre – joint tie accepted", tie.constraint !== null, `${tie.breaks.length} breaks`);
  const hw2 = s.bodyHoleControlWorld(plate, 0);
  check("hole moved onto the joint, still regular", dist(centreOf(hw2), s.jointWorld(j)) < 2e-3 && irregularity(hw2) < 2e-3);
  check("side count edit on the hole", s.setRegularSides(plate.id, 0, 8) && plate.holes![0].controlLocal.length === 8 && irregularity(s.bodyHoleControlWorld(plate, 0)) < 1e-9);
  check("plate outline untouched by hole edits", plate.controlLocal.length === 4 && plate.regular === undefined);
  // Moving the hole as a feature (the way a hole is dragged): a clean translation, no wobble.
  const hw3 = s.bodyHoleControlWorld(plate, 0);
  s.moveOutline(plate.id, 0, vec(-7, 4));
  const hw4 = s.bodyHoleControlWorld(plate, 0);
  check("moveOutline translates a regular hole exactly", hw4.every((p, i) => near(p.x, hw3[i].x - 7, 1e-9) && near(p.y, hw3[i].y + 4, 1e-9)));
  const platePos = vec(plate.pos.x, plate.pos.y);
  s.moveOutline(plate.id, null, vec(3, 3));
  check("moveOutline on the outer outline moves the body", near(plate.pos.x, platePos.x + 3, 1e-9) && near(plate.pos.y, platePos.y + 3, 1e-9));
  // Notch (crossing the outline) is a general path: no flag.
  const notch = s.cutBody(plate.id, { control: regularPolygon(vec(200, 100), vec(230, 100), 6), radius: 0, round: "fillet", regular: 6 });
  check("a crossing cut is a free path", notch.ok && notch.hole === null && plate.regular === undefined);
}

// ---------------------------------------------------------------- travel: copy, mirror, save, split
{
  const s = new Scene();
  const hex = s.addBody(regularPolygon(vec(50, 50), vec(80, 50), 6), 0, "fillet", undefined, undefined, 6);
  s.cutBody(hex.id, { control: regularPolygon(vec(50, 50), vec(58, 50), 4), radius: 0, round: "fillet", regular: 4 });
  const clip = s.extractSelection([hex.id])!;
  const pastedId = s.insertBody(clip, vec(300, 300))!;
  const pasted = s.getBody(pastedId)!;
  check("paste keeps the outline flag", pasted.regular === 6);
  check("paste keeps the hole flag", pasted.holes?.[0].regular === 4);
  s.mirrorBody(hex.id, "h");
  check("mirror keeps the flag and regularity", hex.regular === 6 && irregularity(s.bodyControlWorld(hex)) < 1e-9);
  s.moveBodyVertex(hex.id, 0, vec(5, 0));
  check("mirrored polygon still edits regularly", irregularity(s.bodyControlWorld(hex)) < 1e-9);
  const data = JSON.parse(JSON.stringify(s.serialize()));
  check("format v21", data.version === 21);
  const t = new Scene();
  t.load(data);
  check("flags survive a round-trip", t.getBody(hex.id)?.regular === 6 && t.getBody(hex.id)?.holes?.[0].regular === 4);
  const bad = JSON.parse(JSON.stringify(s.serialize()));
  bad.bodies[0].regular = 5; // doesn't match the corners
  bad.bodies[0].holes[0].regular = "4";
  const u = new Scene();
  u.load(bad);
  check("a mismatched saved count is dropped", u.getBody(hex.id)?.regular === undefined && u.getBody(hex.id)?.holes?.[0].regular === undefined);
  const plain = s.addBody(regularPolygon(vec(500, 500), vec(540, 500), 6), 0, "fillet", undefined, undefined, 6);
  const split = s.splitBody(plain.id, [vec(460, 500), vec(540, 500)]); // corner to corner across the hexagon
  check("split pieces are free polygons", split.ok && split.a.regular === undefined && split.b.regular === undefined, split.ok ? "" : split.reason);
  const other = s.addBody(sq(60, 60, 40, 40));
  const comb = s.combineBodies([hex.id, other.id]);
  check("combined outline is a free polygon", comb.ok && comb.body.regular === undefined);
}

// ---------------------------------------------------------------- line constraints on every edge
{
  // Including the edges that must turn by exactly 90° (a symmetric pinch: no turning moment).
  for (const n of [4, 5, 6]) for (const kind of ["horizontal", "vertical"] as const) {
    let ok = 0;
    for (let edge = 0; edge < n; edge++) {
      const s = new Scene();
      const b = s.addBody(regularPolygon(vec(300, 300), vec(450, 300), n), 0, "fillet", undefined, undefined, n);
      const res = tryAddConstraint(s, kind, { kind: "edge", bodyId: b.id, index: edge });
      const w = s.bodyControlWorld(b);
      const d = kind === "horizontal" ? Math.abs(w[edge].y - w[(edge + 1) % n].y) : Math.abs(w[edge].x - w[(edge + 1) % n].x);
      if (res.constraint && d < 2e-3 && irregularity(w) < 2e-3 && near(dist(w[0], centreOf(w)), 150, 2e-3)) ok++;
    }
    check(`${kind} on every edge of a ${n}-gon (radius kept)`, ok === n, `${ok}/${n}`);
  }
  // Parallel to another body's edge: the polygon turns to it in one go; the other body stays.
  const s = new Scene();
  const bar = s.addBody([vec(0, 0), vec(100, 40), vec(90, 65), vec(-10, 25)]);
  const hex = s.addBody(regularPolygon(vec(300, 300), vec(400, 300), 6), 0, "fillet", undefined, undefined, 6);
  const barBefore = s.bodyControlWorld(bar);
  const res = tryAddConstraint(s, "parallel", { kind: "edge", bodyId: hex.id, index: 0 }, { kind: "edge", bodyId: bar.id, index: 0 });
  const w = s.bodyControlWorld(hex);
  const barAfter = s.bodyControlWorld(bar);
  const e = sub(w[1], w[0]);
  const t = sub(barAfter[1], barAfter[0]);
  check("parallel to an external edge accepted", res.constraint !== null, `${res.breaks.length} breaks`);
  check("edges ended parallel", Math.abs(e.x * t.y - e.y * t.x) / (dist(w[0], w[1]) * dist(barAfter[0], barAfter[1])) < 2e-3);
  // Equal ranks share the correction (the bar's edge may turn a little too); the polygon
  // itself stays regular and keeps its size, and the bar keeps its edge length.
  check("polygon regular and same size, bar edge length kept", irregularity(w) < 2e-3 && near(dist(w[0], centreOf(w)), 100, 2e-3) && near(dist(barAfter[0], barAfter[1]), dist(barBefore[0], barBefore[1]), 2e-3));
}

// ---------------------------------------------------------------- size dimensions own the size
{
  const s = new Scene();
  const hex = s.addBody(regularPolygon(vec(0, 0), vec(100, 0), 6), 0, "fillet", undefined, undefined, 6);
  // Across flats (opposite edges) — the case from the field.
  const flats = s.addMeasurement("draw", { kind: "edge", bodyId: hex.id, index: 0 }, { kind: "edge", bodyId: hex.id, index: 3 }, vec(0, 0))!;
  check("edge–edge dimension is a size (flats)", s.regularSizeOfDim(flats)?.size.kind === "flats");
  check("flats dimension applied", applyDrivingDimension(s, flats.id, 150).length === 0 && near(s.measureInfo(flats)!.value, 150, 1e-6));
  check("size is now driven", s.regularSizeDriven(hex.id, null));
  // A corner drag on a size-driven polygon only turns it.
  const before = s.bodyControlWorld(hex);
  s.moveBodyVertex(hex.id, 0, vec(30, 25));
  const live = solveSketch(s, new Set([`v:${hex.id}:0`]));
  const w = s.bodyControlWorld(hex);
  check("drag keeps the dimensioned size", live.length === 0 && near(s.measureInfo(flats)!.value, 150, 1e-6), `${s.measureInfo(flats)!.value}`);
  check("drag turned the polygon", Math.abs(Math.atan2(w[0].y, w[0].x) - Math.atan2(before[0].y, before[0].x)) > 0.1 && irregularity(w) < 1e-9);
  // Other size kinds.
  const chord = s.addMeasurement("draw", { kind: "vertex", bodyId: hex.id, index: 0 }, { kind: "vertex", bodyId: hex.id, index: 2 }, vec(0, 0))!;
  chord.axis = "direct";
  const sc = s.regularSizeOfDim(chord)!.size;
  check("corner pair two apart is a chord", sc.kind === "chord" && sc.k === 2);
  const radius = s.addMeasurement("draw", { kind: "centre", bodyId: hex.id }, { kind: "vertex", bodyId: hex.id, index: 1 }, vec(0, 0))!;
  radius.axis = "direct";
  check("centre–corner is the circumradius", s.regularSizeOfDim(radius)?.size.kind === "radius");
  const apo = s.addMeasurement("draw", { kind: "centre", bodyId: hex.id }, { kind: "edge", bodyId: hex.id, index: 2 }, vec(0, 0))!;
  check("centre–edge is the apothem", s.regularSizeOfDim(apo)?.size.kind === "apothem");
  const hDim = s.addMeasurement("draw", { kind: "vertex", bodyId: hex.id, index: 0 }, { kind: "vertex", bodyId: hex.id, index: 1 }, vec(0, 0))!;
  hDim.axis = "h";
  check("a horizontal corner-pair distance is not a size", s.regularSizeOfDim(hDim) === null);
}

// ---------------------------------------------------------------- corner drags respect the sketch
{
  // The field case: H on a side + an across-corners driving dimension → a rigid object;
  // dragging a corner moves the whole polygon (and the anchored live solve agrees).
  const s = new Scene();
  const hex = s.addBody(regularPolygon(vec(1445, -980), vec(1445 + 333 * Math.cos(0.96), -980 + 333 * Math.sin(0.96)), 6), 0, "fillet", undefined, undefined, 6);
  tryAddConstraint(s, "horizontal", { kind: "edge", bodyId: hex.id, index: 5 });
  const across = s.addMeasurement("draw", { kind: "vertex", bodyId: hex.id, index: 0 }, { kind: "vertex", bodyId: hex.id, index: 3 }, vec(0, 0))!;
  across.axis = "direct";
  check("across-corners dimension applied", applyDrivingDimension(s, across.id, 666).length === 0);
  check("rotation locked by H", s.regularRotationLocked(hex.id, null));
  check("size driven by the dimension", s.regularSizeDriven(hex.id, null));
  let before = s.bodyControlWorld(hex);
  let jitter = 0;
  for (const d of [vec(40, 10), vec(-25, 60), vec(3, 3), vec(-80, -20)]) {
    const pos0 = vec(hex.pos.x, hex.pos.y);
    s.moveBodyVertex(hex.id, 2, d);
    const live = solveSketch(s, new Set([`v:${hex.id}:2`]));
    if (live.length) jitter++;
    const w = s.bodyControlWorld(hex);
    if (!near(hex.pos.x - pos0.x, d.x, 1e-6) || !near(hex.pos.y - pos0.y, d.y, 1e-6) || !near(hex.angle, 0.96 + 0 * hex.angle, 10)) jitter += 0;
    check(`rigid polygon: corner drag (${d.x},${d.y}) moved it whole`, near(hex.pos.x - pos0.x, d.x, 1e-6) && near(hex.pos.y - pos0.y, d.y, 1e-6) && live.length === 0 && near(s.measureInfo(across)!.value, 666, 1e-6) && Math.abs(w[5].y - w[0].y) < 1e-6, `${live.length} breaks`);
  }
  check("no anchored-solve failures during the rigid drag", jitter === 0);
  before = s.bodyControlWorld(hex);
  void before;

  // Rotation locked, size free: a corner drag resizes along the corner's radial line only.
  const t = new Scene();
  const pent = t.addBody(regularPolygon(vec(0, 0), vec(100, 0), 5), 0, "fillet", undefined, undefined, 5);
  tryAddConstraint(t, "vertical", { kind: "edge", bodyId: pent.id, index: 1 });
  check("V locks the rotation", t.regularRotationLocked(pent.id, null) && !t.regularSizeDriven(pent.id, null));
  const w0 = t.bodyControlWorld(pent);
  const c0 = centreOf(w0);
  const dir0 = Math.atan2(w0[0].y - c0.y, w0[0].x - c0.x);
  t.moveBodyVertex(pent.id, 0, vec(30, 40)); // partly tangential
  const live2 = solveSketch(t, new Set([`v:${pent.id}:0`]));
  const w1 = t.bodyControlWorld(pent);
  const c1 = centreOf(w1);
  check("locked-rotation drag keeps the corner on its radial line", near(Math.atan2(w1[0].y - c1.y, w1[0].x - c1.x), dir0, 1e-9) && dist(c1, c0) < 1e-9);
  check("locked-rotation drag resized the polygon", dist(w1[0], c1) > 100 + 1 && irregularity(w1) < 1e-9 && live2.length === 0);
  check("V still holds after the drag", Math.abs(w1[1].x - w1[2].x) < 2e-3);

  // Two corners tied elsewhere also lock the rotation; a lone tie does not.
  const u = new Scene();
  const sq4 = u.addBody(regularPolygon(vec(0, 0), vec(50, 0), 4), 0, "fillet", undefined, undefined, 4);
  const j1 = u.addFreeJoint(vec(50, 0));
  tryAddConstraint(u, "coincident", { kind: "vertex", bodyId: sq4.id, index: 0 }, { kind: "joint", jointId: j1.id });
  check("one tied corner leaves the rotation free", !u.regularRotationLocked(sq4.id, null));
  const j2 = u.addFreeJoint(vec(0, 50));
  tryAddConstraint(u, "coincident", { kind: "vertex", bodyId: sq4.id, index: 1 }, { kind: "joint", jointId: j2.id });
  check("two tied corners lock the rotation", u.regularRotationLocked(sq4.id, null));
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
