/**
 * The Equal sketch constraint between **radii**: two disks, round holes, rounded corners,
 * reference circles / arcs made the same size. Covers validation (which references
 * name a radius, two lines still mean equal lengths, a line and a circle are refused, an
 * arc can be copied but not set), who follows whom (the second pick takes the first
 * one's radius; a driving diameter / radius dimension outranks every member; a
 * reference arc and component-instance geometry can't change; a direct resize is the
 * reference), classes (A = B and B = C is one class), uniform vs mixed corner outlines,
 * a follower disk's tangent line re-solving, conflicts rejected with the scene
 * untouched, the violated flag, pattern seeds / members, and the usual remaps (hole
 * removal, a node added to a disk, element deletion, save / load, copy / paste).
 */
import { Scene, MeasureRef, SceneData, Vec2, isEqualRadiusConstraint } from "../src/model";
import {
  solveSketch,
  tryAddConstraint,
  applyDrivingDimension,
  enforceEqualRadii,
  equalRadiusPartners,
  equalRadiusViolated,
  sketchConfig,
} from "../src/sketch";
import { placeConstraint, isPoseConstraint } from "../src/pose";

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
const eref = (bodyId: number, index: number): MeasureRef => ({ kind: "edge", bodyId, index });
const vref = (bodyId: number, index: number, hole?: number): MeasureRef => (hole === undefined ? { kind: "vertex", bodyId, index } : { kind: "vertex", bodyId, index, hole });
const gref = (guideId: number, edge = 0): MeasureRef => ({ kind: "guideLine", guideId, edge });
const jref = (id: number): MeasureRef => ({ kind: "joint", jointId: id });

/** The radius a reference names (throws when it names none — a test setup error). */
function rad(s: Scene, ref: MeasureRef): number {
  const r = s.radiusOfRef(ref);
  if (r === null) throw new Error(`no radius: ${JSON.stringify(ref)}`);
  return r;
}
function centre(s: Scene, ref: MeasureRef): Vec2 {
  const c = s.circleOfRef(ref);
  if (!c) throw new Error("not a circle");
  return c.c;
}
const eqCount = (s: Scene): number => s.sketch.filter(isEqualRadiusConstraint).length;

// --- validation ---------------------------------------------------------------
{
  const s = new Scene();
  const plate = s.addBody(square(), 0, "fillet", [{ control: [{ x: 50, y: 50 }], radius: 10, round: "offset" }]);
  const disk = s.addBody([{ x: 200, y: 200 }], 20, "offset");
  const round = s.addBody(square(300, 0), 5, "fillet"); // uniform rounded corners
  const circ = s.addGuideCircle({ x: 400, y: 0 }, 30)!;
  const arc = s.addGuideArc({ x: 500, y: 0 }, { x: 530, y: 30 }, { x: 560, y: 0 })!; // r = 30
  const arc2 = s.addGuideArc({ x: 600, y: 0 }, { x: 620, y: 20 }, { x: 640, y: 0 })!;
  const seg = s.addGuidePoly([{ x: 0, y: -50 }, { x: 100, y: -60 }], false)!;
  const j = s.addFreeJoint({ x: -100, y: -100 });
  const accepts = (a: MeasureRef, b: MeasureRef): boolean => {
    const problem = s.sketchConstraintProblem("equal", a, b);
    const c = s.addSketchConstraint("equal", a, b);
    if (c) s.removeSketchConstraint(c.id);
    return problem === null && c !== null;
  };
  const refuses = (a: MeasureRef, b?: MeasureRef, re?: RegExp): boolean => {
    const problem = s.sketchConstraintProblem("equal", a, b);
    return problem !== null && s.addSketchConstraint("equal", a, b) === null && (!re || re.test(problem));
  };
  check("disk body + round hole", accepts(dref(disk.id), dref(plate.id, 0)));
  check("disk + rounded corner", accepts(dref(disk.id), vref(round.id, 0)));
  check("rounded corner + round hole", accepts(vref(round.id, 2), dref(plate.id, 0)));
  check("disk + reference circle", accepts(dref(disk.id), gcref(circ.id)));
  check("disk + reference arc (the arc can be copied)", accepts(dref(disk.id), gcref(arc.id)));
  check("two reference arcs are refused: neither can be set", refuses(gcref(arc.id), gcref(arc2.id), /three points/));
  check("a sharp corner names a radius too (0)", accepts(vref(plate.id, 1), dref(disk.id)));
  check("two lines still make equal lengths", accepts(eref(plate.id, 0), gref(seg.id)));
  check("a line and a circle are refused, with the reason", refuses(eref(plate.id, 0), dref(disk.id), /lengths.*radii/i));
  check("a joint is refused", refuses(jref(j.id), dref(disk.id)));
  check("a circle with itself is refused", refuses(dref(disk.id), dref(disk.id), /different/));
  check("one reference is refused", refuses(dref(disk.id)));
  const eq = s.addSketchConstraint("equal", dref(disk.id), gcref(circ.id))!;
  const eqL = s.addSketchConstraint("equal", eref(plate.id, 0), gref(seg.id))!;
  check("isEqualRadiusConstraint tells the two forms apart", isEqualRadiusConstraint(eq) && !isEqualRadiusConstraint(eqL));
  check(
    "radiusOfRef reads every kind, and nothing else",
    near(rad(s, dref(disk.id)), 20) && near(rad(s, dref(plate.id, 0)), 10) && near(rad(s, vref(round.id, 0)), 5) &&
      near(rad(s, gcref(circ.id)), 30) && near(rad(s, gcref(arc.id)), 30) && near(rad(s, vref(plate.id, 1)), 0) &&
      s.radiusOfRef(eref(plate.id, 0)) === null && s.radiusOfRef(jref(j.id)) === null
  );
  check("radiusSettable: everything but an arc", s.radiusSettable(dref(disk.id)) && s.radiusSettable(vref(round.id, 0)) && s.radiusSettable(gcref(circ.id)) && !s.radiusSettable(gcref(arc.id)));
  check("a disk read through its centre vertex is the same radius", near(rad(s, vref(disk.id, 0)), 20));
}

// --- the second pick takes the first one's radius; every kind follows ------------------
{
  const s = new Scene();
  const a = s.addBody([{ x: 0, y: 0 }], 20, "offset");
  const b = s.addBody([{ x: 100, y: 0 }], 15, "offset");
  const r = tryAddConstraint(s, "equal", dref(a.id), dref(b.id));
  check("Equal(A, B) is placed", r.constraint !== null, `${r.breaks.length} breaks`);
  check("…B took A's radius (20), A kept it", near(rad(s, dref(b.id)), 20) && near(rad(s, dref(a.id)), 20), `A ${rad(s, dref(a.id))} B ${rad(s, dref(b.id))}`);
  const plate = s.addBody(square(200, 0), 0, "fillet", [{ control: [{ x: 250, y: 50 }], radius: 8, round: "offset" }]);
  check("Equal(disk, hole): the hole follows", tryAddConstraint(s, "equal", dref(a.id), dref(plate.id, 0)).constraint !== null && near(rad(s, dref(plate.id, 0)), 20));
  const round = s.addBody(square(400, 0), 5, "fillet");
  check(
    "Equal(hole, corner) on a uniform outline: every corner follows",
    tryAddConstraint(s, "equal", dref(plate.id, 0), vref(round.id, 0)).constraint !== null && s.bodyCornerRadii(round).every((x) => near(x, 20)),
    s.bodyCornerRadii(round).join(" ")
  );
  const mixed = s.addBody(square(600, 0), 0, "fillet", undefined, [8, null, null, null]);
  check(
    "Equal(disk, corner) on a mixed outline: only that corner follows",
    tryAddConstraint(s, "equal", dref(a.id), vref(mixed.id, 0)).constraint !== null && near(s.bodyCornerRadii(mixed)[0], 20) && near(s.bodyCornerRadii(mixed)[1], 0),
    s.bodyCornerRadii(mixed).join(" ")
  );
  const circ = s.addGuideCircle({ x: 0, y: 300 }, 30)!;
  check("Equal(disk, reference circle): the circle follows", tryAddConstraint(s, "equal", dref(a.id), gcref(circ.id)).constraint !== null && near(rad(s, gcref(circ.id)), 20));
  check("A = B, A = hole, hole = corner, A = corner', A = circle: one class of six", equalRadiusPartners(s, dref(a.id)).length === 5);
  check("…found through the disk's centre vertex too", equalRadiusPartners(s, vref(b.id, 0)).length === 5);
  s.setDiskRadius(a.id, 12); // a rim drag / `[` — main.ts then propagates from the resized element
  const breaks = enforceEqualRadii(s, dref(a.id));
  check(
    "a direct resize of A is the reference: everything follows",
    breaks.length === 0 && near(rad(s, dref(b.id)), 12) && near(rad(s, dref(plate.id, 0)), 12) && near(s.bodyCornerRadii(round)[3], 12) &&
      near(s.bodyCornerRadii(mixed)[0], 12) && near(s.bodyCornerRadii(mixed)[2], 0) && near(rad(s, gcref(circ.id)), 12)
  );
  check("nothing reads as violated", s.sketch.every((c) => !equalRadiusViolated(s, c)));
  check("a plain solve changes nothing more", solveSketch(s).length === 0 && near(rad(s, dref(a.id)), 12));
}

// --- a reference arc can only be copied ------------------------------------------------
{
  const s = new Scene();
  const disk = s.addBody([{ x: 0, y: 0 }], 20, "offset");
  const arc = s.addGuideArc({ x: 500, y: 0 }, { x: 530, y: 30 }, { x: 560, y: 0 })!; // r = 30
  const r = tryAddConstraint(s, "equal", dref(disk.id), gcref(arc.id));
  check("Equal(disk, arc): the disk follows the arc although it was the first pick", r.constraint !== null && near(rad(s, dref(disk.id)), 30), `r ${rad(s, dref(disk.id))}`);
  s.moveGuidePoint(arc.id, "m", { x: 530, y: 20 }); // half-chord 30, sagitta 20 → r = 32.5
  check("reshaping the arc reads as a violation", equalRadiusViolated(s, r.constraint!));
  check("…which the next solve clears by resizing the disk", solveSketch(s).length === 0 && near(rad(s, dref(disk.id)), 32.5) && !equalRadiusViolated(s, r.constraint!), `r ${rad(s, dref(disk.id))}`);
  const d2 = s.addBody([{ x: 0, y: 200 }], 20, "offset");
  const m = s.addMeasurement("draw", vref(d2.id, 0), vref(d2.id, 0), { x: 0, y: 200 })!;
  check("setup: a driving Ø40", m.axis === "diameter" && applyDrivingDimension(s, m.id, 40).length === 0);
  const r2 = tryAddConstraint(s, "equal", dref(d2.id), gcref(arc.id));
  check("Equal(dimensioned disk, arc) is rejected: the dimension asks the arc to change", r2.constraint === null && r2.breaks.some((b) => b.id === m.id), `${r2.breaks.length} breaks`);
  check("…scene untouched", near(rad(s, dref(d2.id)), 20) && near(rad(s, gcref(arc.id)), 32.5) && eqCount(s) === 1);
}

// --- a driving dimension outranks every member -------------------------------------------
{
  const s = new Scene();
  const a = s.addBody([{ x: 0, y: 0 }], 20, "offset");
  const b = s.addBody([{ x: 100, y: 0 }], 35, "offset");
  const ma = s.addMeasurement("draw", vref(a.id, 0), vref(a.id, 0), { x: 0, y: 0 })!;
  check("setup: A holds Ø40", applyDrivingDimension(s, ma.id, 40).length === 0 && s.getMeasurement(ma.id)!.driving === true);
  const r = tryAddConstraint(s, "equal", dref(b.id), dref(a.id));
  check("Equal(B, A) with A dimensioned: B follows A, not the other way", r.constraint !== null && near(rad(s, dref(b.id)), 20) && near(rad(s, dref(a.id)), 20), `A ${rad(s, dref(a.id))} B ${rad(s, dref(b.id))}`);
  const mb = s.addMeasurement("draw", vref(b.id, 0), vref(b.id, 0), { x: 100, y: 0 })!;
  const conflict = applyDrivingDimension(s, mb.id, 50);
  check("driving B's diameter to 50 conflicts with A's Ø40", conflict.length > 0 && conflict.some((k) => k.id === ma.id), `${conflict.length} breaks`);
  check("…scene untouched: both radii 20, B's dimension still driven", near(rad(s, dref(a.id)), 20) && near(rad(s, dref(b.id)), 20) && !s.getMeasurement(mb.id)!.driving);
  check("driving B's diameter to 40 (agreeing) is fine", applyDrivingDimension(s, mb.id, 40).length === 0 && s.getMeasurement(mb.id)!.driving === true);
  s.clearMeasurementDriving(mb.id);
  check("driving A to Ø60 carries B", applyDrivingDimension(s, ma.id, 60).length === 0 && near(rad(s, dref(b.id)), 30), `B ${rad(s, dref(b.id))}`);
  // A direct resize of B against A's dimension: at the model level the dimension wins
  // (main.ts demotes the partner's dimension first, so there the handle wins).
  s.setDiskRadius(b.id, 45);
  check("a direct resize of B against A's dimension snaps back", enforceEqualRadii(s, dref(b.id)).length === 0 && near(rad(s, dref(b.id)), 30));
  s.clearMeasurementDriving(ma.id);
  s.setDiskRadius(b.id, 45);
  check("…with no dimension the resized one is the reference", enforceEqualRadii(s, dref(b.id)).length === 0 && near(rad(s, dref(a.id)), 45));
  const c = s.addBody([{ x: 200, y: 0 }], 10, "offset");
  const mc = s.addMeasurement("draw", vref(c.id, 0), vref(c.id, 0), { x: 200, y: 0 })!;
  check("setup: C holds Ø50, A holds Ø90", applyDrivingDimension(s, mc.id, 50).length === 0 && applyDrivingDimension(s, ma.id, 90).length === 0 && near(rad(s, dref(b.id)), 45));
  const bad = tryAddConstraint(s, "equal", dref(a.id), dref(c.id));
  check("Equal between a Ø90 and a Ø50 disk is rejected, both dimensions flagged", bad.constraint === null && bad.breaks.some((k) => k.id === ma.id) && bad.breaks.some((k) => k.id === mc.id), `${bad.breaks.length} breaks`);
  check("…scene untouched", near(rad(s, dref(a.id)), 45) && near(rad(s, dref(c.id)), 25) && eqCount(s) === 1);
}

// --- a follower disk's tangent line follows the new rim ----------------------------------
{
  const s = new Scene();
  const plate = s.addBody(square());
  s.addSketchConstraint("fixed", eref(plate.id, 2)); // top edge, y = 100
  const a = s.addBody([{ x: 50, y: 120 }], 20, "offset"); // already tangent to it
  check("setup: tangent placed", tryAddConstraint(s, "tangent", dref(a.id), eref(plate.id, 2)).constraint !== null);
  const b = s.addBody([{ x: 300, y: 300 }], 35, "offset");
  const r = tryAddConstraint(s, "equal", dref(b.id), dref(a.id));
  check("Equal(B, A): A takes B's radius (35)", r.constraint !== null && near(rad(s, dref(a.id)), 35), `${r.breaks.length} breaks`);
  check("…and rose to stay tangent (centre y = 135)", near(centre(s, dref(a.id)).y, 135, TOL), fmt(centre(s, dref(a.id))));
  const mb = s.addMeasurement("draw", vref(b.id, 0), vref(b.id, 0), { x: 300, y: 300 })!;
  check(
    "driving B's diameter to 60 resizes A and moves it (centre y = 130)",
    applyDrivingDimension(s, mb.id, 60).length === 0 && near(rad(s, dref(a.id)), 30) && near(centre(s, dref(a.id)).y, 130, TOL),
    fmt(centre(s, dref(a.id)))
  );
  const round = s.addBody(square(500, 0), 5, "fillet");
  check(
    "Equal(corner, A): the corner joins the dimensioned class (30)",
    tryAddConstraint(s, "equal", vref(round.id, 0), dref(a.id)).constraint !== null && near(s.bodyCornerRadii(round)[0], 30),
    s.bodyCornerRadii(round).join(" ")
  );
  s.clearMeasurementDriving(mb.id);
  const mr = s.addMeasurement("draw", vref(round.id, 0), vref(round.id, 0), { x: 500, y: 0 })!;
  check("a radius dimension on the corner", mr.axis === "radius");
  check(
    "driving the corner to 10 carries both disks and re-solves the tangent (centre y = 110)",
    applyDrivingDimension(s, mr.id, 10).length === 0 && near(rad(s, dref(a.id)), 10) && near(rad(s, dref(b.id)), 10) && near(centre(s, dref(a.id)).y, 110, TOL),
    `A ${rad(s, dref(a.id))} B ${rad(s, dref(b.id))} ${fmt(centre(s, dref(a.id)))}`
  );
}

// --- component instances can't change: the free side follows -------------------------------
{
  const s = new Scene();
  const d = s.addBody([{ x: 50, y: 150 }], 20, "offset");
  const def = s.createComponentFromSelection("Disk", [d.id])!;
  const inst = s.getBody(def.instance.bodyMap[0].id)!;
  const free = s.addBody([{ x: 300, y: 0 }], 35, "offset");
  const r = placeConstraint(s, "equal", dref(free.id), dref(inst.id));
  check(
    "Equal(free disk, instance disk): the free one follows although it was the first pick",
    r.constraint !== null && !isPoseConstraint(s, r.constraint) && near(rad(s, dref(free.id)), 20) && near(rad(s, dref(inst.id)), 20),
    `${r.breaks.length} breaks, free ${rad(s, dref(free.id))}`
  );
  const inst2 = s.instantiateComponent(def.def.id, { pos: { x: 500, y: 0 }, angle: 0 })!;
  const b2 = s.getBody(inst2.bodyMap[0].id)!;
  const problem = s.sketchConstraintProblem("equal", dref(inst.id), dref(b2.id));
  check("two instance radii are refused, the reason speaks of radii", problem !== null && /radii/.test(problem), problem ?? "none");
}

// --- patterns: a member is its seed ---------------------------------------------------------
{
  const s = new Scene();
  const plate = s.addBody(square(0, 0, 300, 100), 0, "fillet", [{ control: [{ x: 40, y: 50 }], radius: 8, round: "offset" }]);
  const p = s.createLinearPattern({ kind: "hole", bodyId: plate.id, hole: 0 }, { x: 140, y: 50 }, 3)!;
  const member = p.slots[0].members[0];
  check("setup: a 3-hole pattern", s.getBody(plate.id)!.holes!.length === 3 && member !== 0);
  check("seed ↔ member is refused: the copies already share the seed's radius", /pattern/.test(s.sketchConstraintProblem("equal", dref(plate.id, 0), dref(plate.id, member)) ?? ""));
  const disk = s.addBody([{ x: 0, y: 300 }], 20, "offset");
  const r = tryAddConstraint(s, "equal", dref(disk.id), dref(plate.id, member));
  check(
    "Equal(disk, member hole) writes the seed, so every hole follows",
    r.constraint !== null && s.getBody(plate.id)!.holes!.every((_, hi) => near(rad(s, dref(plate.id, hi)), 20)),
    `${r.breaks.length} breaks`
  );
  s.setDiskRadius(plate.id, 6, 0); // the seed resized directly (a rim drag)
  check("resizing the seed carries the disk: the member keys as its seed", enforceEqualRadii(s, dref(plate.id, 0)).length === 0 && near(rad(s, dref(disk.id)), 6));
}

// --- a class that disagrees with no edit behind it settles on the first-named radius ---------
{
  const s = new Scene();
  const a = s.addBody([{ x: 0, y: 0 }], 20, "offset");
  const b = s.addBody([{ x: 100, y: 0 }], 15, "offset");
  const c = s.addSketchConstraint("equal", dref(b.id), dref(a.id))!; // placed without a solve, as a file might carry it
  check("unsolved, it reads as violated", equalRadiusViolated(s, c));
  check("a plain solve settles it on the first-named radius (B's 15)", solveSketch(s).length === 0 && near(rad(s, dref(a.id)), 15) && !equalRadiusViolated(s, c));
}

// --- remaps: hole removal, a node added to the disk, deletions, save / load, copy / paste ------
{
  const s = new Scene();
  const plate = s.addBody(square(), 0, "fillet", [
    { control: [{ x: 25, y: 50 }], radius: 8, round: "offset" },
    { control: [{ x: 70, y: 50 }], radius: 10, round: "offset" },
  ]);
  const disk = s.addBody([{ x: 50, y: 300 }], 20, "offset");
  const k = tryAddConstraint(s, "equal", dref(disk.id), dref(plate.id, 1)).constraint!;
  check("setup", !!k && near(rad(s, dref(plate.id, 1)), 20));
  s.removeBodyHole(plate.id, 0);
  check("the disk ref follows its hole through a hole removal", k.refB!.kind === "disk" && k.refB!.hole === 0 && s.sketch.includes(k));
  s.insertBodyVertex(plate.id, 1, { x: 80, y: 50 }, 0); // the round hole becomes a two-point outline
  check("a node added to the hole drops the Equal (no circle left)", !s.sketch.some((c) => c.id === k.id));
  const circ = s.addGuideCircle({ x: 300, y: 300 }, 25)!;
  const k2 = tryAddConstraint(s, "equal", dref(disk.id), gcref(circ.id)).constraint!;
  s.removeGuide(circ.id);
  check("deleting the reference circle drops the Equal", !s.sketch.some((c) => c.id === k2.id));
  const round = s.addBody(square(300, 0), 5, "fillet");
  const k3 = tryAddConstraint(s, "equal", dref(disk.id), vref(round.id, 2)).constraint!;
  s.removeBody(disk.id);
  check("deleting the disk drops its Equal", !s.sketch.some((c) => c.id === k3.id));
}
{
  const s = new Scene();
  const a = s.addBody([{ x: 0, y: 0 }], 20, "offset");
  const round = s.addBody(square(100, 0), 5, "fillet");
  tryAddConstraint(s, "equal", dref(a.id), vref(round.id, 1));
  const data = JSON.parse(JSON.stringify(s.serialize())) as SceneData;
  const t = new Scene();
  t.load(data);
  const k = t.sketch.find((c) => c.kind === "equal");
  check("an Equal between radii round-trips through save / load", !!k && isEqualRadiusConstraint(k) && k.refA.kind === "disk" && k.refB?.kind === "vertex");
  check("…and solves clean", solveSketch(t).length === 0);
  const clip = s.extractSelection([a.id, round.id], [])!;
  const before = s.sketch.length;
  s.insertSelection(clip, { x: 300, y: 0 });
  check("copy / paste carries the Equal between the copied bodies", s.sketch.length === before + 1);
  const pasted = s.sketch[s.sketch.length - 1];
  check("…re-pointed at the pasted disk", isEqualRadiusConstraint(pasted) && pasted.refA.kind === "disk" && pasted.refA.bodyId !== a.id && s.radiusOfRef(pasted.refA) !== null);
  s.setDiskRadius(a.id, 9);
  check("the copies form their own class", enforceEqualRadii(s, dref(a.id)).length === 0 && near(s.bodyCornerRadii(round)[1], 9) && near(rad(s, pasted.refA), 20));
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll equal-radius checks passed.");
