/**
 * Headless coverage for free joints as group members (v14). Verifies that:
 *  - addGroup accepts free joints (bodies + joints ≥ 2), rejects attached joints,
 *    merges groups touched through a joint, groupOfJoint finds membership, and
 *    pruning drops removed / absorbed joints (dissolving < 2-member groups).
 *  - In simulation a locked free joint is rigid group material: towing a member body
 *    carries the joint exactly; driving a pin on the joint tows the whole group;
 *    grounding the joint lets the group pivot about it (revolute to the world) while
 *    the anchor holds; a grounded member body fixes the joint too.
 *  - A rail of two group-locked free joints is a track that moves with the group
 *    (riders slide along it and follow the group's motion).
 *  - Serialize/load round-trips jointIds; legacy groups without jointIds load fine.
 */
import { Scene, SceneData } from "../src/model";
import { solve, Driver } from "../src/solver";
import { Vec2, dist, sub, rotate, add } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

function square(scene: Scene, cx: number, cy: number) {
  return scene.addBody([
    { x: cx - 20, y: cy - 20 },
    { x: cx + 20, y: cy - 20 },
    { x: cx + 20, y: cy + 20 },
    { x: cx - 20, y: cy + 20 },
  ]);
}

/** Position of a free joint relative to a body's frame. */
function relToBody(scene: Scene, bodyId: number, jointId: number): Vec2 {
  const b = scene.getBody(bodyId)!;
  const j = scene.getJoint(jointId)!;
  return rotate(sub(scene.jointWorld(j), b.pos), -b.angle);
}

// --- membership management ---------------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const j1 = scene.addFreeJoint({ x: 60, y: 0 });
  const j2 = scene.addFreeJoint({ x: 0, y: 60 });
  const attached = scene.addJoint(a.id, { x: 0, y: 0 });

  check("addGroup accepts body + free joint", scene.addGroup([a.id], [j1.id]) !== null, "1 body + 1 joint");
  check("groupOfJoint finds membership", scene.groupOfJoint(j1.id) !== undefined, `joint ${j1.id}`);
  check(
    "attached joints can't be members",
    !scene.addGroup([a.id], [attached.id])!.jointIds.includes(attached.id),
    "attached joint skipped"
  );
  // The re-group above absorbed the previous group (a is shared), keeping j1.
  check("merge through a shared body keeps the joint", scene.groupOfJoint(j1.id) !== undefined, "j1 still grouped");

  const b = square(scene, 100, 0);
  const g2 = scene.addGroup([b.id], [j1.id])!;
  check("merge through a shared joint absorbs the group", scene.groups.length === 1 && g2.bodyIds.length === 2, `groups ${scene.groups.length}, bodies ${g2.bodyIds.length}`);

  check("joints-only group allowed", scene.addGroup([], [j1.id, j2.id]) !== null, "2 joints");
  check("joint group merge kept one group", scene.groups.length === 1, `${scene.groups.length}`);

  scene.removeJoint(j2.id);
  check("removeJoint prunes group joints", scene.groups[0].jointIds.length === 1, `jointIds ${scene.groups[0].jointIds.join(",")}`);

  // A group reduced to a single member dissolves.
  const lone = new Scene();
  const j3 = lone.addFreeJoint({ x: 0, y: 0 });
  const j4 = lone.addFreeJoint({ x: 10, y: 0 });
  lone.addGroup([], [j3.id, j4.id]);
  lone.removeJoint(j4.id);
  check("a 1-member group dissolves", lone.groups.length === 0, `${lone.groups.length}`);
}

// --- rigid behaviour in simulation --------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const jl = scene.addFreeJoint({ x: 80, y: 20 }); // locked point off the body
  scene.addGroup([a.id], [jl.id]);
  const before = relToBody(scene, a.id, jl.id);

  // Tow the body far away: the locked joint must ride along exactly.
  const grab = scene.addJoint(a.id, { x: 0, y: 0 });
  const drv: Driver = { jointId: grab.id, target: { x: 300, y: -150 } };
  for (let i = 0; i < 60; i++) solve(scene, drv, 60);
  check(
    "locked joint rides a towed group",
    dist(relToBody(scene, a.id, jl.id), before) < 1e-9,
    `rel drift ${dist(relToBody(scene, a.id, jl.id), before).toExponential(2)}`
  );

  // Pin an outside body to the locked joint and drive the outside body: the group is towed.
  const outside = square(scene, 200, 200);
  const oj = scene.addJoint(outside.id, { x: 200, y: 200 });
  scene.addPin(oj.id, jl.id);
  const drv2: Driver = { bodyId: outside.id, local: { x: 0, y: 0 }, target: { x: -200, y: 100 } };
  const posBefore = { ...scene.getBody(a.id)!.pos };
  for (let i = 0; i < 80; i++) solve(scene, drv2, 60);
  const gap = dist(scene.jointWorld(scene.getJoint(oj.id)!), scene.jointWorld(scene.getJoint(jl.id)!));
  check("pin to a locked joint closes", gap < 1e-3, `gap ${gap.toExponential(2)}`);
  check(
    "the pin tows the whole group",
    dist(scene.getBody(a.id)!.pos, posBefore) > 50,
    `moved ${dist(scene.getBody(a.id)!.pos, posBefore).toFixed(1)}`
  );
  check(
    "rigidity kept while towed",
    dist(relToBody(scene, a.id, jl.id), before) < 1e-6,
    `rel drift ${dist(relToBody(scene, a.id, jl.id), before).toExponential(2)}`
  );
}

// --- grounding a locked joint: revolute to the world ---------------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const jl = scene.addFreeJoint({ x: 100, y: 0 });
  scene.addGroup([a.id], [jl.id]);
  scene.addGround(jl.id, { x: 100, y: 0 });
  const before = relToBody(scene, a.id, jl.id);

  const grab = scene.addJoint(a.id, { x: 0, y: 0 });
  const drv: Driver = { jointId: grab.id, target: { x: 100, y: 100 } };
  let breaks: ReturnType<typeof solve> = [];
  for (let i = 0; i < 120; i++) breaks = solve(scene, drv, 60);
  const anchorPos = scene.jointWorld(scene.getJoint(jl.id)!);
  check("grounded locked joint holds its anchor", dist(anchorPos, { x: 100, y: 0 }) < 1e-6, `at (${anchorPos.x.toFixed(3)}, ${anchorPos.y.toFixed(3)})`);
  const bodyPos = scene.getBody(a.id)!.pos;
  check(
    "group pivots about the anchor",
    Math.abs(dist(bodyPos, { x: 100, y: 0 }) - 100) < 1e-3 && dist(bodyPos, { x: 0, y: 0 }) > 30,
    `radius ${dist(bodyPos, { x: 100, y: 0 }).toFixed(3)}, moved ${dist(bodyPos, { x: 0, y: 0 }).toFixed(1)}`
  );
  check("rigidity kept while pivoting", dist(relToBody(scene, a.id, jl.id), before) < 1e-6, "rel pose exact");
  check("no breaks reported", breaks.length === 0, `${breaks.length}`);

  // Grounding a member body fixes the locked joint too.
  scene.toggleBodyGround(a.id);
  const jlBefore = scene.jointWorld(scene.getJoint(jl.id)!);
  const drv3: Driver = { jointId: jl.id, target: { x: -500, y: -500 } };
  for (let i = 0; i < 40; i++) solve(scene, drv3, 60);
  check(
    "fixed group makes the locked joint immovable",
    dist(scene.jointWorld(scene.getJoint(jl.id)!), jlBefore) < 1e-9,
    "joint unmoved under drag"
  );
}

// --- a rail of two locked free joints moves with its group ---------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const r1 = scene.addFreeJoint({ x: 60, y: -40 });
  const r2 = scene.addFreeJoint({ x: 60, y: 40 });
  scene.addGroup([a.id], [r1.id, r2.id]);
  // Build the slider directly on the locked joints (addSlider would auto-ground loose
  // free rails; locked ones are legitimate rail ends, so grounding must not happen —
  // it only auto-grounds joints, so the flags stay clear and the group carries the rail).
  const slider = scene.addSlider(r1.id, r2.id);
  const grounds = scene.constraints.filter((c) => c.kind === "ground").length;
  check("addSlider does not ground locked rail joints", grounds === 0, `${grounds} grounds`);

  const riderBody = square(scene, 120, 0);
  const rider = scene.addJoint(riderBody.id, { x: 120, y: 0 });
  scene.attachSliderRider(slider.id, rider.id);

  // Drive the rider along the rail: it slides between the endpoints.
  const drv: Driver = { jointId: rider.id, target: { x: 60, y: 30 } };
  let breaks: ReturnType<typeof solve> = [];
  for (let i = 0; i < 80; i++) breaks = solve(scene, drv, 60);
  const q = scene.jointWorld(scene.getJoint(rider.id)!);
  check("rider lands on the locked rail", Math.abs(q.x - 60) < 1e-3, `x ${q.x.toFixed(4)}`);
  check("rider reaches along the rail", Math.abs(q.y - 30) < 1e-2, `y ${q.y.toFixed(3)}`);
  check("no breaks on the group rail", breaks.length === 0, `${breaks.length}`);

  // Tow the group: the rail moves with it and carries the rider.
  const grab = scene.addJoint(a.id, { x: 0, y: 0 });
  const drv2: Driver = { jointId: grab.id, target: { x: -200, y: 0 } };
  for (let i = 0; i < 100; i++) breaks = solve(scene, drv2, 60);
  const ra = scene.jointWorld(scene.getJoint(r1.id)!);
  const q2 = scene.jointWorld(scene.getJoint(rider.id)!);
  check("rail followed the towed group", ra.x < -100, `rail x ${ra.x.toFixed(1)}`);
  check("rider stayed on the moving rail", Math.abs(q2.x - ra.x) < 1e-2, `dx ${(q2.x - ra.x).toExponential(2)}`);
}

// --- persistence ---------------------------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const b = square(scene, 100, 0);
  const j = scene.addFreeJoint({ x: 50, y: 50 });
  scene.addGroup([a.id, b.id], [j.id]);
  const data = JSON.parse(JSON.stringify(scene.serialize())) as SceneData;

  const loaded = new Scene();
  loaded.load(data);
  check("jointIds survive serialize/load", loaded.groups[0]?.jointIds.length === 1, `jointIds ${loaded.groups[0]?.jointIds.join(",")}`);

  // Legacy group without jointIds loads as an empty list.
  const legacy = JSON.parse(JSON.stringify(data)) as SceneData;
  delete (legacy.groups![0] as unknown as { jointIds?: number[] }).jointIds;
  const l2 = new Scene();
  l2.load(legacy);
  check("legacy groups load without jointIds", Array.isArray(l2.groups[0].jointIds) && l2.groups[0].jointIds.length === 0, "defaulted to []");
}

// --- copy/paste carries group joints -------------------------------------------
{
  const scene = new Scene();
  const a = square(scene, 0, 0);
  const j = scene.addFreeJoint({ x: 60, y: 0 });
  scene.addGroup([a.id], [j.id]);
  const clip = scene.extractSelection([a.id], [j.id])!;
  check("clip group carries the joint", clip.groups.length === 1 && clip.groups[0].joints.length === 1, `${clip.groups[0]?.joints.length ?? 0} joints`);
  const res = scene.insertSelection(clip, { x: 300, y: 0 })!;
  const g = scene.groupOf(res.bodyIds[0]);
  check("pasted group locks the pasted joint", g !== undefined && g.jointIds.length === 1, `jointIds ${g?.jointIds.join(",")}`);
}

if (failures > 0) {
  console.error(`${failures} FAILURE(S)`);
  process.exit(1);
}
console.log("All group-joint checks passed");
