/**
 * Feature clips: extractFeatures / insertFeatures (holes + joints copied between bodies,
 * with grounds, rails, sketch constraints, driving dims and patterns) and
 * removeBodyFeatures (multi-feature delete with the outline / hole minimums).
 */
import { Scene } from "../src/model";
import { Vec2, dist, sub } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: Vec2, b: Vec2, tol = 1e-6) => dist(a, b) < tol;
const sq = (x0: number, y0: number, s: number): Vec2[] => [
  { x: x0, y: y0 },
  { x: x0 + s, y: y0 },
  { x: x0 + s, y: y0 + s },
  { x: x0, y: y0 + s },
];

/** Source body: a disk hole (pattern seed, 3 along +y), a square hole, two joints on a
 *  horizontal line with a ground, a rail, a horizontal constraint and a driving dim. */
function source() {
  const s = new Scene();
  const a = s.addBody(sq(0, 0, 100));
  const disk = s.addBodyHole(a.id, { control: [{ x: 20, y: 20 }], radius: 5, round: "offset" })!;
  const square = s.addBodyHole(a.id, { control: sq(60, 60, 20), radius: 2 })!;
  const p = s.createLinearPattern({ kind: "hole", bodyId: a.id, hole: disk }, { x: 20, y: 60 }, 3)!;
  const j1 = s.addJoint(a.id, { x: 40, y: 85 });
  const j2 = s.addJoint(a.id, { x: 90, y: 85 });
  s.addGround(j1.id, s.jointWorld(j1));
  const rail = s.addSlider(j1.id, j2.id);
  const sk = s.addSketchConstraint("horizontal", { kind: "joint", jointId: j1.id }, { kind: "joint", jointId: j2.id })!;
  const m = s.addMeasurement("draw", { kind: "joint", jointId: j1.id }, { kind: "joint", jointId: j2.id }, { x: 65, y: 95 })!;
  s.setMeasurementDriving(m.id, 50);
  return { s, a, disk, square, p, j1, j2, rail, sk, m };
}

// --- 1) extractFeatures: a copied seed brings its members; internal constraints travel. ---
{
  const { s, a, disk, square, p, j1, j2 } = source();
  check("setup: pattern has 2 members", p.members.length === 2, `${p.members.length}`);
  const clip = s.extractFeatures(a.id, [disk, square], [j1.id, j2.id])!;
  check("clip exists", !!clip);
  check("clip holes = seed + members + square", clip.holes.length === 4, `${clip.holes.length}`);
  check("clip joints = 2", clip.joints.length === 2);
  check("ground travels", clip.grounds.length === 1);
  check("rail travels", clip.sliders.length === 1);
  check("horizontal constraint travels", clip.sketch.length === 1 && clip.sketch[0].kind === "horizontal");
  check("driving dim travels", clip.dims.length === 1 && clip.dims[0].target === 50);
  check("pattern travels with its members", clip.patterns.length === 1 && clip.patterns[0].members.length === 2);
  // A joints-only copy leaves the hole-anchored things behind.
  const jc = s.extractFeatures(a.id, [], [j1.id])!;
  check("joint-only clip: no holes / rail (other end missing) / constraint", jc.holes.length === 0 && jc.sliders.length === 0 && jc.sketch.length === 0 && jc.dims.length === 0);
  check("joint-only clip keeps the ground", jc.grounds.length === 1);
  check("nothing named → null", s.extractFeatures(a.id, [], []) === null);
}

// --- 2) insertFeatures into another body: geometry translates, everything is recreated. ---
{
  const { s, a, disk, square, p, j1, j2 } = source();
  const srcInfo = s.patternInfo(p.id)!;
  const srcStep = sub(srcInfo.members[0].point, srcInfo.anchor);
  const clip = s.extractFeatures(a.id, [disk, square], [j1.id, j2.id])!;
  const b = s.addBody(sq(200, 200, 200));
  const before = {
    joints: s.joints.length,
    constraints: s.constraints.length,
    sketch: s.sketch.length,
    dims: s.measurements.length,
    patterns: s.patterns.length,
  };
  const at = { x: 300, y: 300 };
  const res = s.insertFeatures(b.id, clip, at)!;
  check("insert: 4 holes, 2 joints, none skipped", res.holes.length === 4 && res.joints.length === 2 && res.skipped === 0, JSON.stringify(res));
  check("target body has 4 holes", (b.holes?.length ?? 0) === 4);
  const offset = sub(at, clip.center);
  const nj = res.joints.map((id) => s.getJoint(id)!);
  check("joints translated by the drop offset", near(s.jointWorld(nj[0]), { x: 40 + offset.x, y: 85 + offset.y }) && near(s.jointWorld(nj[1]), { x: 90 + offset.x, y: 85 + offset.y }));
  check("joints attached to the target body", nj.every((j) => j.bodyId === b.id));
  const g = s.constraints.find((c) => c.kind === "ground" && c.joint === nj[0].id);
  check("ground recreated on the pasted joint at its position", g?.kind === "ground" && near(g.anchor, s.jointWorld(nj[0])));
  const rail = s.constraints.find((c) => c.kind === "slider" && c.railA === nj[0].id && c.railB === nj[1].id);
  check("rail recreated between the pasted joints", !!rail);
  check("sketch constraint recreated", s.sketch.length === before.sketch + 1);
  const dim = s.measurements.find((m) => m.refA.kind === "joint" && m.refA.jointId === nj[0].id);
  check("driving dim recreated with its target", dim?.driving === true && dim.target === 50);
  check("pattern recreated on the target", s.patterns.length === before.patterns + 1);
  const np = s.patterns.find((p) => p.bodyId === b.id)!;
  const info = s.patternInfo(np.id)!;
  check("pattern members keep the source spacing", near(sub(info.members[0].point, info.anchor), srcStep), `${info.members[0].point.x - info.anchor.x},${info.members[0].point.y - info.anchor.y} vs ${srcStep.x},${srcStep.y}`);
  check("source body untouched", (a.holes?.length ?? 0) === 4 && s.joints.filter((j) => j.bodyId === a.id).length === 2);
  check("constraint count grew by ground + rail", s.constraints.length === before.constraints + 2);
  check("joint count grew by 2", s.joints.length === before.joints + 2);
}

// --- 3) insertFeatures into a rotated body keeps world spacing; misfits are skipped. ---
{
  const { s, a, disk, p, j1, j2 } = source();
  const srcInfo = s.patternInfo(p.id)!;
  const srcStep2 = sub(srcInfo.members[1].point, srcInfo.anchor);
  const clip = s.extractFeatures(a.id, [disk], [j1.id, j2.id])!;
  const b = s.addBody(sq(200, 200, 200));
  s.rotateBody(b.id, b.pos, Math.PI / 6);
  const res = s.insertFeatures(b.id, clip, { x: 300, y: 300 })!;
  check("rotated target: 3 holes + 2 joints", res.holes.length === 3 && res.joints.length === 2, JSON.stringify(res));
  const np = s.patterns.find((p) => p.bodyId === b.id)!;
  const info = s.patternInfo(np.id)!;
  check("rotated target: pattern step stays world-oriented", near(sub(info.members[1].point, info.anchor), srcStep2, 1e-6), `${info.members[1].point.x - info.anchor.x},${info.members[1].point.y - info.anchor.y} vs ${srcStep2.x},${srcStep2.y}`);
  // Drop near the corner: the far members / joints fall outside and are skipped.
  const c = s.addBody(sq(500, 500, 60));
  const res2 = s.insertFeatures(c.id, clip, { x: 555, y: 555 })!;
  check("misfits skipped", res2.skipped > 0 && res2.holes.length + res2.joints.length < 5, JSON.stringify(res2));
  const cp = s.patterns.filter((p) => p.bodyId === c.id);
  check("a pattern missing a member pastes as plain holes", cp.length === 0);
}

// --- 4) removeBodyFeatures: joints, vertices, whole holes, and the 3-corner floor. ---
{
  const s = new Scene();
  const body = s.addBody([{ x: 0, y: 0 }, { x: 50, y: -10 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }]);
  const hole = s.addBodyHole(body.id, { control: sq(40, 40, 20), radius: 0 })!;
  const disk = s.addBodyHole(body.id, { control: [{ x: 20, y: 80 }], radius: 4, round: "offset" })!;
  const j = s.addJoint(body.id, { x: 80, y: 20 });
  const keep = s.addJoint(body.id, { x: 80, y: 80 });
  let r = s.removeBodyFeatures(body.id, [{ hole: null, index: 1 }, { hole, index: 0 }], [j.id]);
  check("joint removed, other kept", !s.getJoint(j.id) && !!s.getJoint(keep.id));
  check("outer vertex removed (5 → 4)", body.controlLocal.length === 4);
  check("one hole vertex removed (4 → 3), hole kept", body.holes?.[hole]?.controlLocal.length === 3);
  check("not refused", !r.outerRefused);
  r = s.removeBodyFeatures(body.id, [{ hole: null, index: 0 }, { hole: null, index: 1 }], []);
  check("outline below 3 corners refused", r.outerRefused && body.controlLocal.length === 4);
  r = s.removeBodyFeatures(body.id, [{ hole, index: 0 }, { hole: disk, index: 0 }], []);
  check("a hole left too small goes whole (both holes gone)", (body.holes?.length ?? 0) === 0, `${body.holes?.length}`);
  check("body survives", !!s.getBody(body.id));
}

// --- 5) Deleting hole vertices across two holes handles the index shift (higher first). ---
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100));
  const h0 = s.addBodyHole(body.id, { control: sq(10, 10, 20), radius: 0 })!;
  const h1 = s.addBodyHole(body.id, { control: [{ x: 70, y: 70 }], radius: 5, round: "offset" })!;
  const h2 = s.addBodyHole(body.id, { control: sq(10, 60, 20), radius: 0 })!;
  // Remove h1 whole (its only vertex) and one vertex of h0 and h2: h2's edit must land on h2.
  s.removeBodyFeatures(body.id, [{ hole: h0, index: 0 }, { hole: h1, index: 0 }, { hole: h2, index: 3 }], []);
  check("two holes remain", body.holes?.length === 2, `${body.holes?.length}`);
  check("each remaining hole lost exactly one vertex", body.holes?.every((h) => h.controlLocal.length === 3) === true, body.holes?.map((h) => h.controlLocal.length).join(","));
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll feature-clip checks passed.");
