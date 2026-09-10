/**
 * Live patterns (v19): linear / circular / grid arrays of a hole or an attached joint
 * on one body. Covers member derivation and re-sync (seed drags, body reshapes, count
 * and spacing edits), the fit flags, dissolve vs remove semantics, index renumbering
 * when holes go, mirror / scale / copy-paste carrying the layout, load sanitizing, and
 * the sketch solver treating members as immovable.
 */
import { Scene, SceneData, MeasureRef, patternLocalMotions } from "../src/model";
import { solveSketch, applyDrivingDimension, tryAddConstraint } from "../src/sketch";
import { dist, vec, Vec2 } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: number, b: number, eps = 1e-6) => Math.abs(a - b) < eps;
const nearPt = (a: Vec2, b: Vec2, eps = 1e-6) => dist(a, b) < eps;

/** A 400 x 200 plate centred on the origin with a diameter-20 disk hole at (-150, 0). */
function plate(): { s: Scene; bodyId: number } {
  const s = new Scene();
  const b = s.addBody(
    [vec(-200, -100), vec(200, -100), vec(200, 100), vec(-200, 100)],
    0,
    "fillet",
    [{ control: [vec(-150, 0)], radius: 10, round: "offset" }]
  );
  return { s, bodyId: b.id };
}
const holeCentre = (s: Scene, bodyId: number, hole: number): Vec2 =>
  s.patternSeedAnchor({ kind: "hole", bodyId, hole })!;
const holeCentres = (s: Scene, bodyId: number): Vec2[] =>
  (s.getBody(bodyId)!.holes ?? []).map((_, i) => holeCentre(s, bodyId, i));

// --- local motions ------------------------------------------------------------
{
  const lin = patternLocalMotions({ kind: "linear", axes: [{ count: 3, step: vec(10, 0) }] }, vec(0, 0));
  check("linear: count-1 motions", lin.length === 2);
  check("linear: i-th instance at i*step", nearPt(lin[1](vec(1, 1)), vec(21, 1)));
  const grid = patternLocalMotions({ kind: "linear", axes: [{ count: 3, step: vec(10, 0) }, { count: 2, step: vec(0, 5) }] }, vec(0, 0));
  check("grid: 3x2 → 5 members", grid.length === 5);
  check("grid: row-major, axis 0 fastest", nearPt(grid[1](vec(0, 0)), vec(20, 0)) && nearPt(grid[2](vec(0, 0)), vec(0, 5)) && nearPt(grid[4](vec(0, 0)), vec(20, 5)));
  const rot = patternLocalMotions({ kind: "circular", centre: vec(-10, 0), count: 4, rotate: true }, vec(10, 0));
  check("circular even: anchor orbits the centre", nearPt(rot[0](vec(10, 0)), vec(0, 10)) && nearPt(rot[1](vec(10, 0)), vec(-10, 0)));
  check("circular rotating: shape turns with the arc", nearPt(rot[0](vec(12, 0)), vec(0, 12)));
  const orb = patternLocalMotions({ kind: "circular", centre: vec(-10, 0), count: 4, rotate: false }, vec(10, 0));
  check("circular orbiting: shape keeps its orientation", nearPt(orb[0](vec(12, 0)), vec(2, 10)));
  const fixed = patternLocalMotions({ kind: "circular", centre: vec(-10, 0), count: 3, angleStep: Math.PI / 2, rotate: true }, vec(10, 0));
  check("circular fixed angle: 90 deg steps", nearPt(fixed[0](vec(10, 0)), vec(0, 10)) && nearPt(fixed[1](vec(10, 0)), vec(-10, 0)));
}

// --- linear hole pattern: creation, members, live edits ---------------------------
{
  const { s, bodyId } = plate();
  const seed = { kind: "hole" as const, bodyId, hole: 0 };
  const p = s.createLinearPattern(seed, vec(-50, 0), 4)!;
  check("linear: pattern created", !!p && p.layout.kind === "linear");
  const body = s.getBody(bodyId)!;
  check("linear: 3 members cut", p.members.length === 3 && body.holes!.length === 4);
  check("linear: member centres at +100 steps", nearPt(holeCentre(s, bodyId, p.members[2]), vec(150, 0)));
  check("linear: members keep the disk shape", body.holes!.every((h) => h.controlLocal.length === 1 && h.round === "offset" && near(h.radius, 10)));
  const info = s.patternInfo(p.id)!;
  check("info: anchor at the seed centre", nearPt(info.anchor, vec(-150, 0)));
  check("info: axis end on the last member", info.axes.length === 1 && nearPt(info.axes[0].end, vec(150, 0)) && info.axes[0].count === 4 && near(info.axes[0].step, 100));
  check("info: every member fits", info.members.every((m) => m.ok));
  check("roles: seed / member / other", s.patternOfHole(bodyId, 0)?.role === "seed" && s.patternOfHole(bodyId, 2)?.role === "member" && s.patternSeedHole(bodyId, 2) === 0);

  // Count up and down.
  s.setPatternAxisCount(p.id, 0, 6);
  check("count 6: 5 members", p.members.length === 5 && body.holes!.length === 6);
  check("count 6: last two stick out → flagged", s.patternInfo(p.id)!.members.map((m) => m.ok).join() === "true,true,true,false,false");
  s.setPatternAxisCount(p.id, 0, 3);
  check("count 3: members trimmed", p.members.length === 2 && body.holes!.length === 3);
  check("count 3: members are holes 1 and 2", p.members.join() === "1,2");

  // Spacing edit keeps the direction.
  s.setPatternAxisStep(p.id, 0, 50);
  check("step 50: 2nd member at -100", nearPt(holeCentre(s, bodyId, p.members[0]), vec(-100, 0)));
  // Re-aim: last instance lands where asked.
  s.setPatternAxisEnd(p.id, 0, vec(-150, 80));
  check("re-aim: last member on the new end", nearPt(holeCentre(s, bodyId, p.members[1]), vec(-150, 80)));
  check("re-aim: first member half-way", nearPt(holeCentre(s, bodyId, p.members[0]), vec(-150, 40)));

  // Seed drag: members follow (vertex move = the disk's centre node).
  s.moveBodyVertex(bodyId, 0, vec(20, 0), 0);
  check("seed moved: members ride along", nearPt(holeCentre(s, bodyId, p.members[0]), vec(-130, 40)) && nearPt(holeCentre(s, bodyId, p.members[1]), vec(-130, 80)));
  // Seed resize: members copy the radius.
  s.setDiskRadius(bodyId, 15, 0);
  check("seed resized: members copy the radius", body.holes!.every((h) => near(h.radius, 15)));

  // Body pose changes carry the pattern (layout is body-local).
  s.rotateBody(bodyId, vec(0, 0), Math.PI / 2);
  const c0 = holeCentre(s, bodyId, 0);
  const c1 = holeCentre(s, bodyId, p.members[0]);
  check("rotated body: member offset rotates with it", nearPt(vec(c1.x - c0.x, c1.y - c0.y), vec(-40, 0)));
}

// --- grid (two axes) -----------------------------------------------------------
{
  const { s, bodyId } = plate();
  const seed = { kind: "hole" as const, bodyId, hole: 0 };
  const p = s.createLinearPattern(seed, vec(-50, 0), 4)!;
  check("grid: adding a parallel axis is refused", !s.addPatternAxis(p.id, vec(-250, 0), 2));
  check("grid: second axis added", s.addPatternAxis(p.id, vec(-150, 50), 3));
  check("grid: 4x3 → 11 members", p.members.length === 11 && s.getBody(bodyId)!.holes!.length === 12);
  const centres = holeCentres(s, bodyId);
  check("grid: corner member at (150, 100)", centres.some((c) => nearPt(c, vec(150, 100))));
  check("grid: a third axis is refused", !s.addPatternAxis(p.id, vec(-150, -50), 2));
  s.setPatternAxisCount(p.id, 1, 2);
  check("grid: axis 1 count 2 → 7 members", p.members.length === 7);
  const info = s.patternInfo(p.id)!;
  check("grid info: two axes", info.axes.length === 2 && nearPt(info.axes[1].end, vec(-150, 50)));
}

// --- fit flags: outside / overlapping -------------------------------------------
{
  const { s, bodyId } = plate();
  const seed = { kind: "hole" as const, bodyId, hole: 0 };
  const p = s.createLinearPattern(seed, vec(-30, 0), 4)!; // step 120: last at 210 → outside
  check("fit: instance outside the body flagged", s.patternInfo(p.id)!.members.map((m) => m.ok).join() === "true,true,false");
  s.setPatternAxisStep(p.id, 0, 15); // radius-10 disks 15 apart overlap
  check("fit: overlapping instances flagged", s.patternInfo(p.id)!.members.every((m) => !m.ok));
  s.setPatternAxisStep(p.id, 0, 25);
  check("fit: clear instances fit", s.patternInfo(p.id)!.members.every((m) => m.ok));
  // Preview without creating.
  const before = s.getBody(bodyId)!.holes!.length;
  const pv = s.patternPreview(seed, { kind: "linear", target: vec(-100, 0), count: 3 })!;
  check("preview: instances listed, nothing created", pv.instances.length === 2 && s.getBody(bodyId)!.holes!.length === before);
  check("preview: existing members are not obstacles", pv.instances.every((i) => i.ok));
}

// --- circular hole pattern: rotating vs orbiting, angle modes -----------------------
{
  const square = [vec(-200, -200), vec(200, -200), vec(200, 200), vec(-200, 200)];
  const slot = [vec(90, -10), vec(130, -10), vec(130, 10), vec(90, 10)]; // 40 x 20, centred at (110, 0)
  const s = new Scene();
  const b = s.addBody(square, 0, "fillet", [slot]);
  const seed = { kind: "hole" as const, bodyId: b.id, hole: 0 };
  const p = s.createCircularPattern(seed, vec(0, 0), 4)!;
  check("circular: 3 slots", p.members.length === 3);
  const c1 = holeCentre(s, b.id, p.members[0]);
  check("circular: member on the orbit", near(dist(c1, vec(0, 0)), 110) && near(Math.abs(c1.x), 0, 1e-6));
  const slot1 = s.bodyHolesWorld(s.getBody(b.id)!)[p.members[0]];
  const ext = (loop: Vec2[]) => ({ w: Math.max(...loop.map((q) => q.x)) - Math.min(...loop.map((q) => q.x)), h: Math.max(...loop.map((q) => q.y)) - Math.min(...loop.map((q) => q.y)) });
  check("circular rotating: slot turned 90 deg", near(ext(slot1).w, 20) && near(ext(slot1).h, 40));
  s.setPatternRotate(p.id, false);
  const slotB = s.bodyHolesWorld(s.getBody(b.id)!)[p.members[0]];
  check("circular orbiting: slot keeps orientation", near(ext(slotB).w, 40) && near(ext(slotB).h, 20));
  const info = s.patternInfo(p.id)!;
  check("circular info: centre / radius / even", info.circular !== null && nearPt(info.circular!.centre, vec(0, 0)) && near(info.circular!.radius, 110) && info.circular!.angleDeg === null && info.circular!.rotate === false);
  // Fixed angle: 90 deg counter-clockwise on screen (y down) → member goes to (0, -110).
  s.setPatternAngle(p.id, 90);
  check("circular fixed 90: first member above the seed (screen CCW)", nearPt(holeCentre(s, b.id, p.members[0]), vec(0, -110)));
  check("circular fixed: info reports 90", near(s.patternInfo(p.id)!.circular!.angleDeg!, 90));
  check("circular: zero angle refused", !s.setPatternAngle(p.id, 0));
  s.setPatternAngle(p.id, null);
  check("circular back to even", s.patternInfo(p.id)!.circular!.angleDeg === null);
  s.setPatternCount(p.id, 6);
  check("circular count 6: 5 members", p.members.length === 5);
  s.setPatternCentre(p.id, vec(110, 60));
  check("circular centre moved: radius 60", near(s.patternInfo(p.id)!.circular!.radius, 60));
  check("circular centre on the seed refused", !s.setPatternCentre(p.id, vec(110, 0)));
}

// --- remove vs dissolve, hole renumbering ---------------------------------------
{
  const { s, bodyId } = plate();
  s.addBodyHole(bodyId, { control: [vec(150, 60)], radius: 8, round: "offset" }); // an unrelated hole (index 1)
  const p = s.createLinearPattern({ kind: "hole", bodyId, hole: 0 }, vec(-50, 0), 3)!;
  const body = s.getBody(bodyId)!;
  check("setup: 4 holes, members are 2 and 3", body.holes!.length === 4 && p.members.join() === "2,3");
  // Removing the unrelated hole renumbers the members.
  s.removeBodyHole(bodyId, 1);
  check("unrelated hole removed: members renumbered", p.members.join() === "1,2" && body.holes!.length === 3);
  // Removing a member removes every member, keeps the seed, drops the pattern.
  s.removeBodyHole(bodyId, 2);
  check("member removed: pattern + members gone, seed stays", s.patterns.length === 0 && body.holes!.length === 1 && nearPt(holeCentre(s, bodyId, 0), vec(-150, 0)));

  const p2 = s.createLinearPattern({ kind: "hole", bodyId, hole: 0 }, vec(-50, 0), 3)!;
  s.dissolvePattern(p2.id);
  check("dissolve: record gone, holes stay", s.patterns.length === 0 && body.holes!.length === 3);
  check("dissolved holes are plain again", s.patternOfHole(bodyId, 1) === undefined);
  // Removing the seed hole of a live pattern dissolves it (members stay).
  const p3 = s.createLinearPattern({ kind: "hole", bodyId, hole: 0 }, vec(-150, 50), 2)!;
  check("setup p3", p3.members.length === 1 && body.holes!.length === 4);
  s.removeBodyHole(bodyId, 0);
  check("seed removed: pattern dissolves, member stays", s.patterns.length === 0 && body.holes!.length === 3);
  // removePattern keeps the seed.
  const p4 = s.createLinearPattern({ kind: "hole", bodyId, hole: 0 }, vec(0, 50), 2)!;
  s.removePattern(p4.id);
  check("removePattern: members gone, seed kept", s.patterns.length === 0 && body.holes!.length === 3);
  // A hole already in a pattern can't seed another.
  const p5 = s.createLinearPattern({ kind: "hole", bodyId, hole: 0 }, vec(0, 50), 2)!;
  check("member can't seed a pattern", s.createLinearPattern({ kind: "hole", bodyId, hole: p5.members[0] }, vec(0, 0), 2) === null);
}

// --- joint patterns --------------------------------------------------------------
{
  const { s, bodyId } = plate();
  const j = s.addJoint(bodyId, vec(-150, 50));
  const free = s.addFreeJoint(vec(0, 300));
  check("free joint can't seed", s.createLinearPattern({ kind: "joint", jointId: free.id }, vec(10, 300), 2) === null);
  const p = s.createLinearPattern({ kind: "joint", jointId: j.id }, vec(-50, 50), 5)!;
  check("joint pattern: 4 members", p.members.length === 4 && s.joints.filter((x) => x.bodyId === bodyId).length === 5);
  check("joint pattern: last outside → flagged", s.patternInfo(p.id)!.members.map((m) => m.ok).join() === "true,true,true,false");
  check("joint roles", s.patternOfJoint(j.id)?.role === "seed" && s.patternOfJoint(p.members[0])?.role === "member");
  // Seed moves → members follow; ground on a member follows too.
  s.addGround(p.members[0], s.jointWorld(s.getJoint(p.members[0])!));
  s.moveJoint(j.id, vec(0, -20));
  check("seed joint moved: members follow", nearPt(s.jointWorld(s.getJoint(p.members[1])!), vec(50, 30)));
  const g = s.constraints.find((c) => c.kind === "ground")!;
  check("member's ground anchor follows", g.kind === "ground" && nearPt(g.anchor, vec(-50, 30)));
  // The seed's own constraints are not replicated.
  s.addGround(j.id, s.jointWorld(j));
  const n = s.constraints.length;
  s.setPatternAxisCount(p.id, 0, 6);
  check("new members carry no constraints", s.constraints.length === n && p.members.length === 5);
  // Deleting a member deletes all members (seed stays); deleting the seed dissolves.
  s.removeJoint(p.members[2]);
  check("member joint removed: pattern + members gone, seed stays", s.patterns.length === 0 && s.joints.filter((x) => x.bodyId === bodyId).length === 1 && s.getJoint(j.id) !== undefined);
  const p2 = s.createCircularPattern({ kind: "joint", jointId: j.id }, vec(-150, 0), 4)!;
  check("circular joints: 3 members on the orbit", p2.members.length === 3 && p2.members.every((id) => near(dist(s.jointWorld(s.getJoint(id)!), vec(-150, 0)), 30)));
  s.removeJoint(j.id);
  check("seed joint removed: members stay as plain joints", s.patterns.length === 0 && s.joints.filter((x) => x.bodyId === bodyId).length === 3);
}

// --- sketch solver: members are immovable, the seed drives ----------------------------
{
  const { s, bodyId } = plate();
  const p = s.createLinearPattern({ kind: "hole", bodyId, hole: 0 }, vec(-50, 0), 3)!;
  // Driving dimension from the outer left edge (vertex 0..3 = -200,-100 / 200,-100 / 200,100 / -200,100)
  // to the seed centre: |x| distance 50 → the seed moves, members follow.
  // Pin the plate's width first: the *first* driving dimension on an undimensioned body
  // scales it uniformly (patterns scale along, tested below) — this exercises the solver.
  const mw = s.addMeasurement("draw", { kind: "vertex", bodyId, index: 0 }, { kind: "vertex", bodyId, index: 1 }, vec(0, -150))!;
  check("width dim drives (no-op scale)", applyDrivingDimension(s, mw.id, 400).length === 0);
  const m = s.addMeasurement("draw", { kind: "vertex", bodyId, index: 0 }, { kind: "vertex", bodyId, index: 0, hole: 0 }, vec(-175, -150))!;
  check("dim axis h", m.axis === "h");
  const breaks = applyDrivingDimension(s, m.id, 30);
  check("driving the seed solves", breaks.length === 0, JSON.stringify(breaks));
  const seedX = holeCentre(s, bodyId, 0).x;
  check("dimension holds 30 from the corner", near(Math.abs(seedX - s.bodyControlWorld(s.getBody(bodyId)!)[0].x), 30, 1e-3));
  check("seed moved (the corner shares the correction)", seedX < -150 - 1e-3);
  check("members followed the seed", near(holeCentre(s, bodyId, p.members[0]).x - seedX, 100, 1e-3) && near(holeCentre(s, bodyId, p.members[1]).x - seedX, 200, 1e-3));
  // A dimension between the seed and one of its members is the pattern's own spacing:
  // driving it is rejected (the member never moves on its own).
  const m2 = s.addMeasurement("draw", { kind: "vertex", bodyId, index: 0, hole: 0 }, { kind: "vertex", bodyId, index: 0, hole: p.members[0] }, vec(-120, -150))!;
  const rej = applyDrivingDimension(s, m2.id, 120);
  check("seed↔member dimension rejected", rej.length === 1 && rej[0].error === Infinity && !m2.driving);
  check("member spacing stays derived (100)", near(dist(holeCentre(s, bodyId, p.members[0]), holeCentre(s, bodyId, 0)), 100, 1e-3));
  solveSketch(s);
  check("re-solve keeps members derived", near(dist(holeCentre(s, bodyId, p.members[0]), holeCentre(s, bodyId, 0)), 100, 1e-3));
}

// --- mirror / scale / copy-paste / save-load ------------------------------------------
{
  const { s, bodyId } = plate();
  const p = s.createLinearPattern({ kind: "hole", bodyId, hole: 0 }, vec(-50, 0), 3)!;
  s.addPatternAxis(p.id, vec(-150, 40), 2);
  s.mirrorBodies([bodyId], [], "h");
  const cs = holeCentres(s, bodyId);
  check("mirror h: seed at (150, 0), members mirrored", nearPt(cs[0], vec(150, 0)) && cs.some((c) => nearPt(c, vec(-50, 0))) && cs.some((c) => nearPt(c, vec(-50, 40))));
  check("mirror: pattern survives", s.patterns.length === 1 && p.members.length === 5);
  s.scaleBody(bodyId, 0.5);
  const cs2 = holeCentres(s, bodyId);
  const rel2 = cs2.slice(1).map((c) => vec(c.x - cs2[0].x, c.y - cs2[0].y));
  check("scale 0.5: spacing halves", rel2.some((d) => nearPt(d, vec(-50, 0))) && rel2.some((d) => nearPt(d, vec(0, 20))));

  // Copy-paste carries the pattern (rotated body → world-oriented layout).
  const s2 = new Scene();
  const b2 = s2.addBody([vec(-200, -100), vec(200, -100), vec(200, 100), vec(-200, 100)], 0, "fillet", [{ control: [vec(-150, 0)], radius: 10, round: "offset" }]);
  s2.createLinearPattern({ kind: "hole", bodyId: b2.id, hole: 0 }, vec(-50, 0), 3);
  s2.rotateBody(b2.id, vec(0, 0), Math.PI / 2);
  const clip = s2.extractBody(b2.id)!;
  const pasted = s2.insertSelection(clip, vec(1000, 0))!;
  const nb = pasted.bodyIds[0];
  check("paste: pattern recreated", s2.patterns.length === 2 && s2.patterns.some((x) => x.bodyId === nb));
  const pc = holeCentres(s2, nb);
  const prel = pc.slice(1).map((c) => vec(c.x - pc[0].x, c.y - pc[0].y));
  check("paste: members at the rotated spacing", pc.length === 3 && prel.some((d) => nearPt(d, vec(0, 100))) && prel.some((d) => nearPt(d, vec(0, 200))));

  // Save / load round trip; a corrupt count is repaired by the sync.
  const data = JSON.parse(JSON.stringify(s2.serialize())) as SceneData;
  check("format v19 with patterns", data.version === 19 && (data.patterns?.length ?? 0) === 2);
  data.patterns![0].members = []; // stale member list
  const s3 = new Scene();
  s3.load(data);
  check("load: patterns restored", s3.patterns.length === 2);
  // (The old member holes are unknown to the record, so they stay as plain holes.)
  check("load: members re-derived", s3.patterns[0].members.length === 2 && s3.getBody(s3.patterns[0].bodyId)!.holes!.length === 5);
  data.patterns![1].seed = { kind: "hole", hole: 99 };
  const s4 = new Scene();
  s4.load(data);
  check("load: pattern with a missing seed dropped", s4.patterns.length === 1);
  // Old files have no patterns.
  delete data.patterns;
  const s5 = new Scene();
  s5.load(data);
  check("load: pre-v19 file → no patterns", s5.patterns.length === 0);
  // Split dissolves.
  const { s: s6, bodyId: b6 } = plate();
  const p6 = s6.createLinearPattern({ kind: "hole", bodyId: b6, hole: 0 }, vec(-50, 0), 3)!;
  const r = s6.splitBody(b6, [vec(0, -100), vec(0, 100)]);
  check("split: ok, pattern dissolved, holes kept", r.ok && s6.patterns.length === 0 && s6.bodies.reduce((n, b) => n + (b.holes?.length ?? 0), 0) === 3, r.ok ? "" : r.reason + p6.id);
}

// --- pattern axis as a constraint reference ---------------------------------------
{
  const { s, bodyId } = plate();
  const p = s.createLinearPattern({ kind: "hole", bodyId, hole: 0 }, vec(-50, 8), 3)!; // slightly off horizontal
  const ref: MeasureRef = { kind: "patternAxis", patternId: p.id, axis: 0 };
  const res = s.resolveMeasureRef(ref);
  check("axis ref resolves to the seed → last line", res?.kind === "line" && nearPt(res.a, vec(-150, 0)) && nearPt(res.b, vec(50, 16)));
  const { constraint, breaks } = tryAddConstraint(s, "horizontal", ref);
  check("H on the axis accepted", !!constraint && breaks.length === 0, JSON.stringify(breaks));
  const c1 = holeCentre(s, bodyId, p.members[1]);
  check("H: axis snapped level (seed stays, members re-laid)", near(c1.y, 0, 1e-3) && near(c1.x, 50, 1e-3), `${c1.x},${c1.y}`);
  check("H: seed did not move", nearPt(holeCentre(s, bodyId, 0), vec(-150, 0), 1e-6));
  // The seed moves (a node drag): the constraint holds through the re-solve.
  s.moveBodyVertex(bodyId, 0, vec(10, 30), 0);
  solveSketch(s);
  const c2 = holeCentre(s, bodyId, p.members[1]);
  check("H survives a seed drag", near(c2.y, 30, 1e-3) && near(c2.x, 60, 1e-3), `${c2.x},${c2.y}`);
  // Parallel to the plate's top edge (edge 0: horizontal): a tilted second axis levels out.
  s.addPatternAxis(p.id, vec(-140, 60), 2); // steep: (0, 30) from the seed at (-140, 30)
  const ref1: MeasureRef = { kind: "patternAxis", patternId: p.id, axis: 1 };
  s.setPatternAxisEnd(p.id, 1, vec(-100, 80)); // now (40, 50): a slant
  const par = tryAddConstraint(s, "parallel", ref1, { kind: "vertex", bodyId, index: 0 });
  check("parallel needs two lines (point ref refused)", par.constraint === null);
  const perp = tryAddConstraint(s, "perpendicular", ref1, { kind: "edge", bodyId, index: 0 });
  check("perpendicular axis ↔ edge accepted", !!perp.constraint && perp.breaks.length === 0, JSON.stringify(perp.breaks));
  const infoP = s.patternInfo(p.id)!;
  const d1 = vec(infoP.axes[1].end.x - infoP.anchor.x, infoP.axes[1].end.y - infoP.anchor.y);
  const e0 = s.bodyControlWorld(s.getBody(bodyId)!);
  const de = vec(e0[1].x - e0[0].x, e0[1].y - e0[0].y);
  check("perpendicular holds: axis ⟂ edge", near(Math.abs(d1.x * de.x + d1.y * de.y) / (Math.hypot(d1.x, d1.y) * Math.hypot(de.x, de.y)), 0, 1e-3));
  // The pattern is a rigid unit for pose purposes.
  check("axis rigid unit = its body", s.refRigidUnitKey(ref) === s.refRigidUnitKey({ kind: "vertex", bodyId, index: 0 }));
  // Serialize / load keeps the constraints.
  const data = JSON.parse(JSON.stringify(s.serialize())) as SceneData;
  const s2 = new Scene();
  s2.load(data);
  check("load keeps axis constraints", s2.sketch.length === 2 && s2.sketch.every((c) => s2.resolveMeasureRef(c.refA) !== null));
  // Copy / paste carries them with remapped pattern ids.
  const clip = s.extractBody(bodyId)!;
  const pasted = s.insertSelection(clip, vec(2000, 0))!;
  const newP = s.patterns.find((x) => x.bodyId === pasted.bodyIds[0])!;
  check("paste: axis constraints remapped to the new pattern", s.sketch.filter((c) => c.refA.kind === "patternAxis" && c.refA.patternId === newP.id).length === 2);
  // Dissolving the pattern drops the constraints on its axes.
  s.dissolvePattern(p.id);
  check("dissolve prunes the axis constraints", s.sketch.every((c) => c.refA.kind !== "patternAxis" || c.refA.patternId !== p.id));
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
if (failures > 0) process.exit(1);
