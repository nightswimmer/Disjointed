/**
 * Headless coverage for permanent body groups. Verifies that:
 *  - addGroup needs 2+ distinct existing bodies, groupOf finds membership, grouping a
 *    selection that touches an existing group merges everything into one group.
 *  - ungroup dissolves the touched groups; removing a body prunes it from its group and
 *    dissolves groups left with fewer than 2 members.
 *  - serialize/load round-trips groups (v9); legacy files without groups load fine.
 *  - In simulation a group behaves as ONE rigid body: driving any member carries the
 *    others with the relative pose exactly preserved; a ground on one member anchors the
 *    whole group; an intra-group pin is inert (never reported as a break); pins to
 *    outside bodies and slider riders on grouped bodies still solve.
 */
import { Scene, SceneData } from "../src/model";
import { solve, Driver } from "../src/solver";
import { tryAddConstraint, solveSketch } from "../src/sketch";
import { Vec2, dist, sub, rotate } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

/** A 40x40 square body centred at (cx, cy). */
function square(scene: Scene, cx: number, cy: number) {
  return scene.addBody([
    { x: cx - 20, y: cy - 20 },
    { x: cx + 20, y: cy - 20 },
    { x: cx + 20, y: cy + 20 },
    { x: cx - 20, y: cy + 20 },
  ]);
}

/** Pose of body `b` relative to body `a` (offset in a's frame + relative angle). */
function relPose(scene: Scene, aId: number, bId: number): { off: Vec2; dAng: number } {
  const a = scene.getBody(aId)!;
  const b = scene.getBody(bId)!;
  return { off: rotate(sub(b.pos, a.pos), -a.angle), dAng: b.angle - a.angle };
}

function relError(before: { off: Vec2; dAng: number }, after: { off: Vec2; dAng: number }): number {
  return Math.max(dist(before.off, after.off), Math.abs(before.dAng - after.dAng) * 100);
}

// --- group management -------------------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 0);
  const c = square(scene, 200, 0);

  check("addGroup rejects a single body", scene.addGroup([a.id]) === null, "1 member");
  check("addGroup rejects missing bodies", scene.addGroup([a.id, 9999]) === null, "1 valid member");

  const g1 = scene.addGroup([a.id, b.id])!;
  check("addGroup creates a 2-body group", !!g1 && g1.bodyIds.length === 2, `ids ${g1?.bodyIds.join(",")}`);
  check("groupOf finds membership", scene.groupOf(a.id)?.id === g1.id && scene.groupOf(b.id)?.id === g1.id, `group ${g1.id}`);
  check("groupOf misses non-members", scene.groupOf(c.id) === undefined, "body c ungrouped");

  // Grouping b + c absorbs a's group: one merged group of all three.
  const g2 = scene.addGroup([b.id, c.id])!;
  check("overlapping groups merge", scene.groups.length === 1 && g2.bodyIds.length === 3, `groups ${scene.groups.length}, members ${g2.bodyIds.length}`);

  check("ungroup dissolves the touched group", scene.ungroup([a.id]) && scene.groups.length === 0, "after ungroup([a])");
  check("ungroup on ungrouped bodies is a no-op", !scene.ungroup([a.id]), "nothing to dissolve");

  // Removing a body prunes its group; a group left with 1 member dissolves.
  scene.addGroup([a.id, b.id, c.id]);
  scene.removeBody(c.id);
  check("removeBody prunes the group", scene.groups[0]?.bodyIds.length === 2, `members ${scene.groups[0]?.bodyIds.join(",")}`);
  scene.removeBody(b.id);
  check("a 1-member group dissolves", scene.groups.length === 0, "after removing b");
}

// --- persistence -------------------------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 0);
  const g = scene.addGroup([a.id, b.id])!;

  const json = JSON.stringify(scene.serialize());
  const loaded = new Scene();
  loaded.load(JSON.parse(json) as SceneData);
  const lg = loaded.groups[0];
  check(
    "groups round-trip through serialize/load",
    loaded.groups.length === 1 && lg.id === g.id && [...lg.bodyIds].sort().join(",") === [a.id, b.id].sort().join(","),
    `group ${lg?.id} members ${lg?.bodyIds.join(",")}`
  );
  const fresh = loaded.addGroup([a.id, b.id]); // merges the loaded group into a new id
  check("nextId continues past group ids", fresh !== null && fresh.id > g.id, `new id ${fresh?.id} > ${g.id}`);

  // Legacy data (pre-v9) has no groups field at all.
  const legacy = JSON.parse(json) as SceneData;
  delete legacy.groups;
  const old = new Scene();
  old.load(legacy);
  check("legacy files load with no groups", old.groups.length === 0, "groups []");
}

// --- rigid behaviour: driving one member carries the whole group -------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 40);
  const ja = scene.addJoint(a.id, { x: 0, y: 0 });
  scene.addGroup([a.id, b.id]);

  const before = relPose(scene, a.id, b.id);
  const driver: Driver = { jointId: ja.id, target: { x: -80, y: 60 } };
  const breaks = solve(scene, driver, 200, 1);
  const at = scene.jointWorld(scene.getJoint(ja.id)!);
  check("driven grouped joint reaches the target", dist(at, driver.target) < 1e-3, `at (${at.x.toFixed(3)}, ${at.y.toFixed(3)})`);
  const after = relPose(scene, a.id, b.id);
  check("group stays rigid under the drag", relError(before, after) < 1e-5, `rel error ${relError(before, after).toExponential(2)}`);
  const bBody = scene.getBody(b.id)!;
  check("the other member actually moved", dist(bBody.pos, { x: 100, y: 40 }) > 50, `b at (${bBody.pos.x.toFixed(1)}, ${bBody.pos.y.toFixed(1)})`);
  check("no breaks reported", breaks.length === 0, `${breaks.length} breaks`);
}

// --- rigid behaviour: a ground on one member anchors the whole group ----------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 0);
  const ja = scene.addJoint(a.id, { x: 0, y: 0 });
  const jb = scene.addJoint(b.id, { x: 100, y: 0 });
  scene.addGround(ja.id, { x: 0, y: 0 });
  scene.addGroup([a.id, b.id]);

  const before = relPose(scene, a.id, b.id);
  // Pull b's joint straight up: the group can only pivot about the ground at the origin.
  const breaks = solve(scene, { jointId: jb.id, target: { x: 0, y: 100 } }, 300, 1);
  const anchor = scene.jointWorld(scene.getJoint(ja.id)!);
  check("ground stays exact on a grouped body", dist(anchor, { x: 0, y: 0 }) < 1e-6, `anchor at (${anchor.x.toExponential(2)}, ${anchor.y.toExponential(2)})`);
  const jbAt = scene.jointWorld(scene.getJoint(jb.id)!);
  check("grouped body pivots about the ground", dist(jbAt, { x: 0, y: 100 }) < 1, `jb at (${jbAt.x.toFixed(2)}, ${jbAt.y.toFixed(2)})`);
  check("radius to the ground preserved", Math.abs(dist(jbAt, anchor) - 100) < 1e-3, `r ${dist(jbAt, anchor).toFixed(4)}`);
  const after = relPose(scene, a.id, b.id);
  check("group stays rigid while pivoting", relError(before, after) < 1e-5, `rel error ${relError(before, after).toExponential(2)}`);
  check("no breaks reported", breaks.length === 0, `${breaks.length} breaks`);
}

// --- intra-group pin is inert --------------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 0);
  const ja = scene.addJoint(a.id, { x: 10, y: 0 });
  const jb = scene.addJoint(b.id, { x: 90, y: 0 });
  scene.addPin(ja.id, jb.id); // endpoints 80 apart — unsatisfiable if it acted
  scene.addGroup([a.id, b.id]);

  const posA = { ...scene.getBody(a.id)!.pos };
  const breaks = solve(scene, null, 60, 1);
  check("intra-group pin reports no break", breaks.length === 0, `${breaks.length} breaks`);
  check("intra-group pin moves nothing", dist(scene.getBody(a.id)!.pos, posA) < 1e-9, "a unmoved");
}

// --- pin from a grouped body to an outside body still solves --------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 0);
  const c = square(scene, 200, 0);
  const jb = scene.addJoint(b.id, { x: 115, y: 0 });
  const jc = scene.addJoint(c.id, { x: 185, y: 0 });
  scene.addPin(jb.id, jc.id);
  scene.addGroup([a.id, b.id]);
  const ja = scene.addJoint(a.id, { x: 0, y: 0 });

  let breaks = solve(scene, null, 60, 1);
  check("external pin closes", dist(scene.jointWorld(scene.getJoint(jb.id)!), scene.jointWorld(scene.getJoint(jc.id)!)) < 1e-3, "pin gap ~0");
  check("no breaks on the external pin", breaks.length === 0, `${breaks.length} breaks`);

  const before = relPose(scene, a.id, b.id);
  breaks = solve(scene, { jointId: ja.id, target: { x: -60, y: 30 } }, 200, 1);
  const gap = dist(scene.jointWorld(scene.getJoint(jb.id)!), scene.jointWorld(scene.getJoint(jc.id)!));
  check("external pin holds while the group is dragged", gap < 1e-3 && breaks.length === 0, `gap ${gap.toExponential(2)}`);
  const after = relPose(scene, a.id, b.id);
  check("group rigid while towing a pinned body", relError(before, after) < 1e-5, `rel error ${relError(before, after).toExponential(2)}`);
}

// --- slider rider on a grouped body ---------------------------------------------
{
  const scene = new Scene();
  // World-fixed track along y = 0.
  const railA = scene.addFreeJoint({ x: 0, y: 0 });
  const railB = scene.addFreeJoint({ x: 300, y: 0 });
  const slider = scene.addSlider(railA.id, railB.id);
  // Body a carries the rider; body b hangs above, grouped to a.
  const a = square(scene, 50, 20);
  const b = square(scene, 50, 100);
  const rider = scene.addJoint(a.id, { x: 50, y: 0 });
  scene.attachSliderRider(slider.id, rider.id);
  scene.addGroup([a.id, b.id]);
  const jb = scene.addJoint(b.id, { x: 50, y: 100 });

  const before = relPose(scene, a.id, b.id);
  const breaks = solve(scene, { jointId: jb.id, target: { x: 200, y: 100 } }, 300, 1);
  const q = scene.jointWorld(scene.getJoint(rider.id)!);
  check("grouped rider stays on the fixed track", Math.abs(q.y) < 1e-3, `y ${q.y.toExponential(2)}`);
  check("group slid along the track", q.x > 150, `x ${q.x.toFixed(1)}`);
  const after = relPose(scene, a.id, b.id);
  check("group rigid while sliding", relError(before, after) < 1e-5, `rel error ${relError(before, after).toExponential(2)}`);
  check("no breaks reported", breaks.length === 0, `${breaks.length} breaks`);
}

// --- copy/paste a multi-selection (group + cross-body pin + free joint) ---------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 0);
  a.color = "#123456";
  b.color = "#654321";
  const ja = scene.addJoint(a.id, { x: 20, y: 0 });
  const jb = scene.addJoint(b.id, { x: 20, y: 0 }); // coincident with ja
  scene.addPin(ja.id, jb.id);
  scene.addGround(ja.id, { x: 20, y: 0 });
  scene.addGroup([a.id, b.id]);
  const free = scene.addFreeJoint({ x: 50, y: 60 });

  const clip = scene.extractSelection([a.id, b.id], [free.id])!;
  check("multi clip captures both bodies", clip.bodies.length === 2, `${clip.bodies.length}`);
  check("multi clip keeps the cross-body pin", clip.pins.length === 1, `${clip.pins.length}`);
  check("multi clip keeps the group", clip.groups.length === 1 && clip.groups[0].length === 2, `${clip.groups.length}`);
  check("multi clip carries the free joint", clip.joints.some((j) => j.bodyTmp === null), "free joint present");

  const res = scene.insertSelection(clip, { x: 400, y: 200 })!;
  check("paste creates both bodies", res.bodyIds.length === 2, `${res.bodyIds.length}`);
  check("paste creates the free joint", res.freeJointIds.length === 1, `${res.freeJointIds.length}`);
  check("scene now has two groups", scene.groups.length === 2, `${scene.groups.length}`);
  const pastedGroup = scene.groups[1];
  check(
    "pasted group holds the pasted bodies",
    [...pastedGroup.bodyIds].sort().join(",") === [...res.bodyIds].sort().join(","),
    `members ${pastedGroup.bodyIds.join(",")}`
  );
  const pins = scene.constraints.filter((c) => c.kind === "pin");
  check("pasted pin exists and is coincident", pins.length === 2, `${pins.length} pins`);
  const colors = res.bodyIds.map((id) => scene.getBody(id)!.color).sort();
  check("pasted bodies keep their colours", colors.join(",") === "#123456,#654321", colors.join(","));
  // The clip centre (mass-weighted, equal squares → midpoint (50, 0)) lands at the drop point.
  const nA = scene.getBody(res.bodyIds[0])!;
  const nB = scene.getBody(res.bodyIds[1])!;
  const mid = { x: (nA.pos.x + nB.pos.x) / 2, y: (nA.pos.y + nB.pos.y) / 2 };
  check("paste lands the selection centre at the drop point", dist(mid, { x: 400, y: 200 }) < 1e-6, `centre (${mid.x}, ${mid.y})`);
  // Pasted arrangement preserved: bodies still 100 apart, free joint at the same offset.
  check("pasted bodies keep their spacing", Math.abs(dist(nA.pos, nB.pos) - 100) < 1e-6, `d ${dist(nA.pos, nB.pos)}`);
  const nFree = scene.getJoint(res.freeJointIds[0])!;
  check("pasted free joint keeps its offset", dist(scene.jointWorld(nFree), { x: 400, y: 260 }) < 1e-6, JSON.stringify(scene.jointWorld(nFree)));
}

// --- mirror a multi-selection about its combined bounding-box centre -------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 40);
  const ja = scene.addJoint(a.id, { x: 15, y: 5 });
  const jb = scene.addJoint(b.id, { x: 15, y: 5 }); // coincident cross-body pin
  scene.addPin(ja.id, jb.id);
  const free = scene.addFreeJoint({ x: 30, y: -50 });

  // Combined bbox: x in [-20, 120] → mirror axis at x = 50 (free joint inside that range).
  scene.mirrorBodies([a.id, b.id], [free.id], "h");
  const pa = scene.getBody(a.id)!.pos;
  const pb = scene.getBody(b.id)!.pos;
  check("mirror h reflects body centroids", dist(pa, { x: 100, y: 0 }) < 1e-6 && dist(pb, { x: 0, y: 40 }) < 1e-6, `a (${pa.x.toFixed(2)}, ${pa.y.toFixed(2)}) b (${pb.x.toFixed(2)}, ${pb.y.toFixed(2)})`);
  const fw = scene.jointWorld(scene.getJoint(free.id)!);
  check("mirror h reflects the free joint", dist(fw, { x: 70, y: -50 }) < 1e-6, `at (${fw.x}, ${fw.y})`);
  const wa = scene.jointWorld(scene.getJoint(ja.id)!);
  const wb = scene.jointWorld(scene.getJoint(jb.id)!);
  check("cross-body pin stays coincident after mirror", dist(wa, wb) < 1e-6, `gap ${dist(wa, wb).toExponential(2)}`);
  check("pinned joints land reflected", dist(wa, { x: 85, y: 5 }) < 1e-6, `at (${wa.x}, ${wa.y})`);
}

// --- rotating every selected body about a shared pivot stays rigid ----------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 0);
  const ja = scene.addJoint(a.id, { x: 20, y: 0 });
  const jb = scene.addJoint(b.id, { x: 20, y: 0 });
  scene.addPin(ja.id, jb.id);

  const before = relPose(scene, a.id, b.id);
  const pivot = { x: 50, y: 0 }; // shared pivot (what the rotate tool uses for a selection)
  scene.rotateBody(a.id, pivot, Math.PI / 3);
  scene.rotateBody(b.id, pivot, Math.PI / 3);
  const after = relPose(scene, a.id, b.id);
  check("shared-pivot rotation keeps the pair rigid", relError(before, after) < 1e-9, `rel error ${relError(before, after).toExponential(2)}`);
  const wa = scene.jointWorld(scene.getJoint(ja.id)!);
  const wb = scene.jointWorld(scene.getJoint(jb.id)!);
  check("cross-body pin stays coincident after rotate", dist(wa, wb) < 1e-9, `gap ${dist(wa, wb).toExponential(2)}`);
}

// --- mirror remaps sketch/measurement refs (the reversal renumbers vertices) ------
{
  const scene = new Scene();
  // Asymmetric shape so the reflection actually moves every corner.
  const body = scene.addBody([
    { x: 0, y: 0 },
    { x: 100, y: 0 },
    { x: 0, y: 60 },
  ]);
  const cx = body.pos.x; // centroid x — the mirror axis
  const { constraint } = tryAddConstraint(scene, "horizontal", { kind: "edge", bodyId: body.id, index: 0 });
  check("H constraint placed on the bottom edge", !!constraint, `id ${constraint?.id}`);
  const m = scene.addMeasurement(
    "draw",
    { kind: "vertex", bodyId: body.id, index: 0 },
    { kind: "vertex", bodyId: body.id, index: 1 },
    { x: 50, y: -30 }
  )!;

  scene.mirrorBody(body.id, "h");
  const refl = (p: Vec2) => ({ x: 2 * cx - p.x, y: p.y });
  // The measurement's vertex refs must still name the same (now reflected) corners.
  const ra = scene.resolveMeasureRef(m.refA)!;
  const rb = scene.resolveMeasureRef(m.refB)!;
  check(
    "vertex refs follow their corners through a mirror",
    ra.kind === "point" && rb.kind === "point" &&
      dist(ra.p, refl({ x: 0, y: 0 })) < 1e-6 && dist(rb.p, refl({ x: 100, y: 0 })) < 1e-6,
    `refA ${JSON.stringify(ra)} refB ${JSON.stringify(rb)}`
  );
  // The H constraint must still sit on the (reflected) bottom edge.
  const re = scene.resolveMeasureRef(scene.sketch[0].refA)!;
  const ends = re.kind === "line" ? [re.a, re.b] : [];
  const want = [refl({ x: 0, y: 0 }), refl({ x: 100, y: 0 })];
  const matches =
    ends.length === 2 &&
    ((dist(ends[0], want[0]) < 1e-6 && dist(ends[1], want[1]) < 1e-6) ||
      (dist(ends[0], want[1]) < 1e-6 && dist(ends[1], want[0]) < 1e-6));
  check("edge ref follows its edge through a mirror", matches, JSON.stringify(ends));
  // Everything is still satisfied: a solve must not move anything (the reported bug —
  // wrong refs made the next sketch solve snap the geometry to weird places).
  const before = scene.bodyControlWorld(body).map((p) => ({ ...p }));
  const breaks = solveSketch(scene);
  const after = scene.bodyControlWorld(body);
  const moved = Math.max(...before.map((p, i) => dist(p, after[i])));
  check("sketch solve after mirror is a no-op", breaks.length === 0 && moved < 1e-3, `breaks ${breaks.length}, moved ${moved.toExponential(2)}`);
}

// --- mirrored group + cross-body constraint survives a group drag ------------------
{
  const scene = new Scene();
  const a = scene.addBody([
    { x: 0, y: 0 }, { x: 60, y: 0 }, { x: 60, y: 60 }, { x: 0, y: 60 },
  ]);
  const b = scene.addBody([
    { x: 60, y: 0 }, { x: 140, y: 0 }, { x: 140, y: 50 }, { x: 60, y: 50 },
  ]);
  const { constraint } = tryAddConstraint(
    scene,
    "coincident",
    { kind: "vertex", bodyId: a.id, index: 1 }, // (60, 0)
    { kind: "vertex", bodyId: b.id, index: 0 }  // (60, 0)
  );
  check("cross-body coincident placed", !!constraint, `id ${constraint?.id}`);
  scene.addGroup([a.id, b.id]);

  scene.mirrorBodies([a.id, b.id], [], "h"); // bbox 0..140 → axis x = 70
  const ra = scene.resolveMeasureRef(scene.sketch[0].refA)!;
  const rb = scene.resolveMeasureRef(scene.sketch[0].refB!)!;
  const gap = ra.kind === "point" && rb.kind === "point" ? dist(ra.p, rb.p) : Infinity;
  check("coincident refs still meet after group mirror", gap < 1e-6, `gap ${gap.toExponential(2)}, at ${ra.kind === "point" ? JSON.stringify(ra.p) : "?"}`);
  check("shared corner landed reflected", ra.kind === "point" && dist(ra.p, { x: 80, y: 0 }) < 1e-6, ra.kind === "point" ? JSON.stringify(ra.p) : "?");

  // The reported symptom: dragging the mirrored group made the sketch solve snap parts
  // to wrong places. A translation + solve must now be exact.
  scene.moveBody(a.id, { x: 13, y: 7 });
  scene.moveBody(b.id, { x: 13, y: 7 });
  const breaks = solveSketch(scene);
  const pa = scene.getBody(a.id)!.pos;
  const pb = scene.getBody(b.id)!.pos;
  check(
    "group drag after mirror stays put",
    breaks.length === 0 && dist(pa, { x: 123, y: 37 }) < 1e-3 && dist(pb, { x: 53, y: 32 }) < 1e-3,
    `a (${pa.x.toFixed(3)}, ${pa.y.toFixed(3)}) b (${pb.x.toFixed(3)}, ${pb.y.toFixed(3)}), breaks ${breaks.length}`
  );
}

if (failures > 0) {
  console.error(`\n${failures} check(s) failed`);
  process.exit(1);
}
console.log("\nAll group checks passed");
