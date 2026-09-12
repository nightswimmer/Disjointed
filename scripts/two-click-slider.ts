/**
 * Headless checks of the two-click slider (v19): `Scene.createSlider` builds a complete
 * prismatic joint from an owner body + two points, the whole-slider deletion cascade
 * (`deleteSlider` / `deleteJoint` / `deleteBody`) takes every role-free part with it while
 * the raw removers stay surgical, `addLinearActuatorOn` drives the owner's own rider, and
 * `jointAt` tells a coincident start pair apart by preference (rail joint by default, rider on request).
 */
import { Scene, SliderConstraint } from "../src/model";
import { solve, resetPoseBaselines } from "../src/solver";
import { dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

/** Smallest signed difference between two angles (radians). */
function angDiff(a: number, b: number): number {
  let d = a - b;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

/** A 80×20 block centred on (cx, cy). */
function block(scene: Scene, cx: number, cy: number, w = 80, h = 20) {
  return scene.addBody([
    { x: cx - w / 2, y: cy - h / 2 },
    { x: cx + w / 2, y: cy - h / 2 },
    { x: cx + w / 2, y: cy + h / 2 },
    { x: cx - w / 2, y: cy + h / 2 },
  ])!;
}

const sliderOf = (scene: Scene, id: number) =>
  scene.constraints.find((c): c is SliderConstraint => c.kind === "slider" && c.id === id);

// --- 1. World-fixed track: the owner translates along the arrow within its range -------
{
  resetPoseBaselines();
  const scene = new Scene();
  const owner = block(scene, 100, 0);
  const made = scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 })!;
  check("createSlider returns the construct", !!made, `slider ${made?.slider.id}`);
  check("rider on the owner, rail joints free", made.rider.bodyId === owner.id && made.railA.bodyId === null && made.railB.bodyId === null,
    `rider body ${made.rider.bodyId}, rails ${made.railA.bodyId} / ${made.railB.bodyId}`);
  check("rider is orientation-locked", made.slider.locked.includes(made.rider.id) && made.slider.riders.includes(made.rider.id),
    `locked ${made.slider.locked.join(",")}`);
  const grounds = scene.constraints.filter((c) => c.kind === "ground").map((c) => c.joint);
  check("free rail joints auto-grounded", grounds.includes(made.railA.id) && grounds.includes(made.railB.id), `grounds ${grounds.join(",")}`);
  check("rail joint A sits on the rider", dist(scene.jointWorld(made.railA), scene.jointWorld(made.rider)) < 1e-9, "coincident start");
  check("rider recorded as the start rider", (made.slider.startRiders ?? []).join(",") === String(made.rider.id), `startRiders ${made.slider.startRiders}`);

  const handle = scene.addJoint(owner.id, { x: 140, y: 0 });
  const angle0 = owner.angle;
  let worstAngle = 0;
  let worstOff = 0;
  let minX = Infinity;
  let maxX = -Infinity;
  for (let i = 0; i < 24; i++) {
    const a = (i / 24) * Math.PI * 2;
    solve(scene, { jointId: handle.id, target: { x: 200 + 220 * Math.cos(a), y: 80 * Math.sin(a) } }, 60);
    worstAngle = Math.max(worstAngle, Math.abs(angDiff(owner.angle, angle0)));
    const q = scene.jointWorld(made.rider);
    worstOff = Math.max(worstOff, Math.abs(q.y));
    minX = Math.min(minX, q.x);
    maxX = Math.max(maxX, q.x);
  }
  check("owner never rotates", worstAngle < 0.01, `max |Δangle| ${worstAngle.toFixed(5)} rad`);
  check("owner's point stays on the line", worstOff < 0.5, `max offset ${worstOff.toFixed(4)}`);
  check("travel clamped to the arrow's range", minX > 100 - 0.5 && maxX < 300 + 0.5, `x ∈ [${minX.toFixed(2)}, ${maxX.toFixed(2)}]`);
  check("travel actually reaches both ends", minX < 101 && maxX > 299, `x ∈ [${minX.toFixed(2)}, ${maxX.toFixed(2)}]`);
}

// --- 2. Moving track: the rail joints attach to the other body and carry the owner ------
{
  resetPoseBaselines();
  const scene = new Scene();
  const track = block(scene, 200, 0, 400, 60); // a long bar
  const owner = block(scene, 100, 0);
  const made = scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 }, track.id)!;
  check("moving track: rail joints on the track body", !!made && made.railA.bodyId === track.id && made.railB.bodyId === track.id,
    `rails ${made?.railA.bodyId} / ${made?.railB.bodyId}`);
  check("moving track: nothing auto-grounded", !scene.constraints.some((c) => c.kind === "ground"), `${scene.constraints.length} constraints`);
  // Drive the track's far end down: the owner rides along, keeping its angle relative to the track.
  const trackHandle = scene.addJoint(track.id, { x: 380, y: 0 });
  const rel0 = owner.angle - track.angle;
  solve(scene, { jointId: trackHandle.id, target: { x: 380, y: 40 } }, 80);
  const ra = scene.jointWorld(made.railA);
  const rb = scene.jointWorld(made.railB);
  const q = scene.jointWorld(made.rider);
  const dx = rb.x - ra.x;
  const dy = rb.y - ra.y;
  const off = Math.abs((q.x - ra.x) * dy - (q.y - ra.y) * dx) / Math.hypot(dx, dy);
  check("moving track: track tilted", Math.abs(track.angle) > 0.05, `track angle ${track.angle.toFixed(3)}`);
  check("moving track: owner's point still on the (tilted) rail", off < 0.5, `off-line ${off.toFixed(4)}`);
  check("moving track: relative angle kept", Math.abs(angDiff(owner.angle - track.angle, rel0)) < 0.01,
    `Δrel ${angDiff(owner.angle - track.angle, rel0).toFixed(5)}`);

  // Rejections: degenerate travel, track = owner, track not reaching the start.
  check("rejects a zero-length travel", scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 100, y: 0 }) === null, "same point");
  check("rejects the owner as its own track", scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 120, y: 0 }, owner.id) === null, "self");
  const far = block(scene, 900, 0);
  check("rejects a track that doesn't contain the start", scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 900, y: 0 }, far.id) === null, "start outside");
  // The start is clamped into the owner (a click just outside lands on its edge).
  const s2 = new Scene();
  const o2 = block(s2, 100, 0);
  const clamped = s2.createSlider(o2.id, { x: 100, y: 30 }, { x: 100, y: 200 })!;
  check("start clamped into the owner", Math.abs(s2.jointWorld(clamped.rider).y - 10) < 1e-6, `y ${s2.jointWorld(clamped.rider).y}`);

  // An existing joint of the owner can be the rider (first click landed on it): no new
  // joint is stacked on it and the start is that joint's position, whatever p1 says.
  const s3 = new Scene();
  const o3 = block(s3, 100, 0);
  const existing = s3.addJoint(o3.id, { x: 120, y: 5 });
  const before = s3.joints.length;
  const reused = s3.createSlider(o3.id, { x: 100, y: 0 }, { x: 300, y: 5 }, null, existing.id)!;
  check("existing joint reused as the rider", !!reused && reused.rider.id === existing.id && s3.joints.length === before + 2,
    `rider ${reused?.rider.id}, joints ${before} → ${s3.joints.length}`);
  check("start is the existing joint's position", dist(s3.jointWorld(reused.railA), { x: 120, y: 5 }) < 1e-9, `railA at ${JSON.stringify(s3.jointWorld(reused.railA))}`);
  check("rider must belong to the owner", s3.createSlider(o3.id, { x: 100, y: 0 }, { x: 300, y: 0 }, null, reused.railB.id) === null, "free joint refused");
  check("a joint already riding a rail is refused", s3.createSlider(o3.id, { x: 100, y: 0 }, { x: 300, y: 0 }, null, existing.id) === null, "rider refused");
}

// --- 3. Deletion: any part of a slider takes the whole thing (role-free parts) ----------
{
  const fresh = () => {
    const scene = new Scene();
    const owner = block(scene, 100, 0);
    const made = scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 })!;
    return { scene, owner, made };
  };
  const counts = (scene: Scene) => `${scene.joints.length} joints, ${scene.constraints.length} constraints`;
  {
    const { scene, made } = fresh();
    scene.deleteSlider(made.slider.id);
    check("deleteSlider: arrow, endpoints and rider all gone", scene.joints.length === 0 && scene.constraints.length === 0, counts(scene));
  }
  {
    const { scene, made } = fresh();
    scene.deleteJoint(made.railB.id);
    check("deleteJoint(end): whole slider gone", scene.joints.length === 0 && scene.constraints.length === 0, counts(scene));
  }
  {
    const { scene, made } = fresh();
    scene.deleteJoint(made.rider.id);
    check("deleteJoint(rider): track goes with its last rider", scene.joints.length === 0 && scene.constraints.length === 0, counts(scene));
  }
  {
    const { scene, owner } = fresh();
    scene.deleteBody(owner.id);
    check("deleteBody(owner): track goes too", scene.joints.length === 0 && scene.constraints.length === 0 && scene.bodies.length === 0, counts(scene));
  }
  {
    // A rider with another role survives: pin it to a second body's joint.
    const { scene, made } = fresh();
    const other = block(scene, 100, 60, 80, 200);
    const oj = scene.addJoint(other.id, { x: 100, y: 0 });
    scene.addPin(made.rider.id, oj.id);
    scene.deleteSlider(made.slider.id);
    check("deleteSlider keeps a pinned rider", !!scene.getJoint(made.rider.id) && !scene.getJoint(made.railA.id) && !scene.getJoint(made.railB.id),
      counts(scene));
    check("…and its pin", scene.constraints.some((c) => c.kind === "pin"), counts(scene));
  }
  {
    // Two carriages on one track: deleting one owner keeps the track for the other.
    const { scene, owner, made } = fresh();
    const second = block(scene, 200, 0);
    const r2 = scene.addJoint(second.id, { x: 200, y: 0 });
    scene.attachSliderRider(made.slider.id, r2.id, true);
    scene.deleteBody(owner.id);
    check("deleteBody with another rider left: track stays", !!sliderOf(scene, made.slider.id) && !!scene.getJoint(made.railA.id) && !!scene.getJoint(made.railB.id),
      counts(scene));
  }
  {
    // The raw removers stay surgical (internal callers: aborted drafts, split / combine).
    const { scene, made } = fresh();
    scene.removeJoint(made.rider.id);
    check("removeJoint(rider) leaves the track", !!sliderOf(scene, made.slider.id) && !!scene.getJoint(made.railA.id), counts(scene));
    scene.removeConstraint(made.slider.id);
    check("removeConstraint leaves the joints", scene.joints.length === 2, counts(scene));
  }
}

// --- 4. Actuator on the owner's own rider drives the body -----------------------------
{
  resetPoseBaselines();
  const scene = new Scene();
  const owner = block(scene, 100, 0);
  const made = scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 })!;
  const act = scene.addLinearActuatorOn(made.slider.id, made.rider.id);
  check("addLinearActuatorOn drives the existing rider", !!act && act.riderId === made.rider.id && act.sliderId === made.slider.id,
    `rider ${act?.riderId}`);
  check("no extra joint minted", scene.joints.length === 3, `${scene.joints.length} joints`);
  check("a second actuator on the same rider is refused", scene.addLinearActuatorOn(made.slider.id, made.rider.id) === null, "null");
  check("a non-rider is refused", scene.addLinearActuatorOn(made.slider.id, made.railA.id) === null, "null");
  let worstMiss = 0;
  let worstAngle = 0;
  for (let i = 0; i <= 10; i++) {
    const target = { x: 100 + 20 * i, y: 0 };
    solve(scene, null, 60, 1, new Map([[made.rider.id, target]]));
    worstMiss = Math.max(worstMiss, dist(scene.jointWorld(made.rider), target));
    worstAngle = Math.max(worstAngle, Math.abs(angDiff(owner.angle, 0)));
  }
  check("anchor on the body rider moves the body along the arrow", worstMiss < 0.5, `max miss ${worstMiss.toFixed(4)}`);
  check("…without rotating it", worstAngle < 0.01, `max |Δangle| ${worstAngle.toFixed(5)}`);
  check("body centre followed", Math.abs(owner.pos.x - 300) < 0.5, `centre x ${owner.pos.x.toFixed(2)}`);

  // Deleting the slider takes the actuator (and its rider) along.
  scene.deleteSlider(made.slider.id);
  check("deleteSlider drops the actuator too", !scene.constraints.some((c) => c.kind === "linearActuator") && scene.joints.length === 0,
    `${scene.constraints.length} constraints`);
}

// --- 5. Persistence + hit-test preference ---------------------------------------------
{
  const a = new Scene();
  const owner = block(a, 100, 0);
  const made = a.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 })!;
  a.addLinearActuatorOn(made.slider.id, made.rider.id);
  const b = new Scene();
  b.load(JSON.parse(JSON.stringify(a.serialize())));
  const sb = sliderOf(b, made.slider.id);
  check("round-trips: slider with locked rider", !!sb && sb.locked.includes(made.rider.id) && sb.railA === made.railA.id && sb.railB === made.railB.id,
    `locked ${sb?.locked.join(",")}`);
  check("round-trips: actuator on the body rider", b.constraints.some((c) => c.kind === "linearActuator" && c.riderId === made.rider.id), "actuator kept");
  check("round-trips: grounds on the track", b.constraints.filter((c) => c.kind === "ground").length === 2, "2 grounds");

  // jointAt on the coincident start: the rail joint by default (draw mode edits endpoints),
  // the rider on request (sim drags).
  const hit = a.jointAt({ x: 100, y: 0 }, 5);
  check("jointAt picks the rail joint of a coincident pair by default", hit?.id === made.railA.id, `hit ${hit?.id} (rider ${made.rider.id}, railA ${made.railA.id})`);
  const hitR = a.jointAt({ x: 100, y: 0 }, 5, "rider");
  check("jointAt('rider') picks the rider", hitR?.id === made.rider.id, `hit ${hitR?.id}`);
  const hitB = a.jointAt({ x: 300, y: 0 }, 5);
  check("jointAt still finds a lone free joint", hitB?.id === made.railB.id, `hit ${hitB?.id}`);
}

// --- 5b. Moving the start joint brings the carriage home ---------------------------------
{
  const scene = new Scene();
  const owner = block(scene, 100, 0); // x ∈ [60, 140], y ∈ [-10, 10]
  const made = scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 })!;
  scene.moveJoint(made.railA.id, { x: 20, y: 5 }); // still inside
  check("start moved inside the body: rider follows exactly", dist(scene.jointWorld(made.rider), { x: 120, y: 5 }) < 1e-9,
    `rider ${JSON.stringify(scene.jointWorld(made.rider))}`);
  scene.moveJoint(made.railA.id, { x: 0, y: 100 }); // (120, 105): outside
  check("start taken outside: rider stays behind", dist(scene.jointWorld(made.rider), { x: 120, y: 5 }) < 1e-9,
    `rider ${JSON.stringify(scene.jointWorld(made.rider))}`);
  scene.moveJoint(made.railA.id, { x: -50, y: -100 }); // (70, 5): back inside, elsewhere
  check("start brought back inside: rider snaps home", dist(scene.jointWorld(made.rider), { x: 70, y: 5 }) < 1e-9,
    `rider ${JSON.stringify(scene.jointWorld(made.rider))}`);
  scene.moveJoint(made.railB.id, { x: -220, y: 0 }); // B to (80, 0): inside the body too
  check("moving the far end never moves the rider", dist(scene.jointWorld(made.rider), { x: 70, y: 5 }) < 1e-9,
    `rider ${JSON.stringify(scene.jointWorld(made.rider))}`);
  // The body drag carries the track (owned) and the rider rides the body — still home.
  scene.moveBody(owner.id, { x: 0, y: 30 });
  check("body drag keeps the carriage home", dist(scene.jointWorld(made.rider), scene.jointWorld(made.railA)) < 1e-9, "coincident");
  // Persistence + copy keep the home; removing the rider drops it from the list.
  const b = new Scene();
  b.load(JSON.parse(JSON.stringify(scene.serialize())));
  check("startRiders survives save/load", (sliderOf(b, made.slider.id)?.startRiders ?? []).join(",") === String(made.rider.id), "kept");
  const clip = scene.extractBody(owner.id)!;
  const newId = scene.insertBody(clip, { x: 100, y: 300 })!;
  const copied = scene.constraints.find((c): c is SliderConstraint => c.kind === "slider" && c.riders.some((r) => scene.getJoint(r)?.bodyId === newId));
  check("paste keeps the start rider", !!copied && (copied.startRiders ?? []).length === 1 && copied.riders.includes(copied.startRiders![0]), `startRiders ${copied?.startRiders}`);
  scene.removeJoint(made.rider.id);
  check("removing the rider drops it from startRiders", sliderOf(scene, made.slider.id)?.startRiders === undefined, "undefined");
  // A plain rail (K tool) has no start riders: moving its first joint leaves riders alone.
  const s2 = new Scene();
  const ra = s2.addFreeJoint({ x: 0, y: 0 });
  const rb = s2.addFreeJoint({ x: 200, y: 0 });
  const rail = s2.addSlider(ra.id, rb.id);
  const bod = block(s2, 100, 0);
  const rj = s2.addJoint(bod.id, { x: 100, y: 0 });
  s2.attachSliderRider(rail.id, rj.id, true);
  s2.moveJoint(ra.id, { x: 90, y: 0 }); // A to (90, 0): inside the body
  check("a plain rail's rider is not re-placed", dist(s2.jointWorld(rj), { x: 100, y: 0 }) < 1e-9, `rider ${JSON.stringify(s2.jointWorld(rj))}`);
}

// --- 6. The owner's rigid edits carry its world-fixed track -----------------------------
{
  const groundAnchor = (scene: Scene, jointId: number) =>
    scene.constraints.find((c) => c.kind === "ground" && c.joint === jointId)!.anchor as { x: number; y: number };
  const trackOK = (scene: Scene, made: NonNullable<ReturnType<Scene["createSlider"]>>, a: { x: number; y: number }, b: { x: number; y: number }) =>
    dist(scene.jointWorld(made.railA), a) < 1e-6 &&
    dist(scene.jointWorld(made.railB), b) < 1e-6 &&
    dist(groundAnchor(scene, made.railA.id), a) < 1e-6 &&
    dist(groundAnchor(scene, made.railB.id), b) < 1e-6;
  {
    const scene = new Scene();
    const owner = block(scene, 100, 0);
    const made = scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 })!;
    check("ownedTrackJoints: both rail joints", scene.ownedTrackJoints(owner.id).sort().join(",") === [made.railA.id, made.railB.id].sort().join(","),
      scene.ownedTrackJoints(owner.id).join(","));
    scene.moveBody(owner.id, { x: 10, y: 20 });
    check("moveBody carries the track + its ground anchors", trackOK(scene, made, { x: 110, y: 20 }, { x: 310, y: 20 }),
      `A ${JSON.stringify(scene.jointWorld(made.railA))} B ${JSON.stringify(scene.jointWorld(made.railB))}`);
    check("…and the rider still sits on the start", dist(scene.jointWorld(made.rider), scene.jointWorld(made.railA)) < 1e-9, "coincident");
    scene.rotateBody(owner.id, { x: 110, y: 20 }, Math.PI / 2); // about the start point: B swings to (110, 220)
    check("rotateBody turns the track about the pivot", trackOK(scene, made, { x: 110, y: 20 }, { x: 110, y: 220 }),
      `A ${JSON.stringify(scene.jointWorld(made.railA))} B ${JSON.stringify(scene.jointWorld(made.railB))}`);
    scene.rotateBody(owner.id, { x: 110, y: 20 }, -Math.PI / 2);
    scene.mirrorBody(owner.id, "h"); // across the vertical line through the centroid (x = 110)
    check("mirrorBody reflects the track", trackOK(scene, made, { x: 110, y: 20 }, { x: -90, y: 20 }),
      `A ${JSON.stringify(scene.jointWorld(made.railA))} B ${JSON.stringify(scene.jointWorld(made.railB))}`);
  }
  {
    // A shared track (riders on two bodies) is nobody's — neither body carries it.
    const scene = new Scene();
    const owner = block(scene, 100, 0);
    const made = scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 })!;
    const second = block(scene, 200, 0);
    scene.attachSliderRider(made.slider.id, scene.addJoint(second.id, { x: 200, y: 0 }).id, true);
    check("shared track: not owned", scene.ownedTrackJoints(owner.id).length === 0 && scene.ownedTrackJoints(second.id).length === 0, "no owner");
    scene.moveBody(owner.id, { x: 0, y: 50 });
    check("shared track stays put", trackOK(scene, made, { x: 100, y: 0 }, { x: 300, y: 0 }), `A ${JSON.stringify(scene.jointWorld(made.railA))}`);
  }
  {
    // A moving track (rail joints on another body) isn't owned by the rider's body.
    const scene = new Scene();
    const track = block(scene, 200, 0, 400, 60);
    const owner = block(scene, 100, 0);
    scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 }, track.id)!;
    check("moving track: not owned by the rider's body", scene.ownedTrackJoints(owner.id).length === 0, "none");
  }
  {
    // Copy/paste: the owned track travels with the body copy.
    const scene = new Scene();
    const owner = block(scene, 100, 0);
    scene.createSlider(owner.id, { x: 100, y: 0 }, { x: 300, y: 0 })!;
    const clip = scene.extractBody(owner.id)!;
    const newId = scene.insertBody(clip, { x: 100, y: 200 })!;
    const copied = scene.constraints.filter((c): c is SliderConstraint => c.kind === "slider");
    const nb = scene.getBody(newId)!;
    const mine = copied.find((c) => c.riders.some((r) => scene.getJoint(r)?.bodyId === newId));
    check("paste recreates the slider on the copy", copied.length === 2 && !!mine && mine.locked.length === 1, `${copied.length} sliders`);
    const a = mine ? scene.jointWorld(scene.getJoint(mine.railA)!) : null;
    const b = mine ? scene.jointWorld(scene.getJoint(mine.railB)!) : null;
    check("pasted track offset with the body", !!a && !!b && dist(a, { x: 100, y: 200 }) < 1e-6 && dist(b, { x: 300, y: 200 }) < 1e-6,
      `A ${JSON.stringify(a)} B ${JSON.stringify(b)} body ${JSON.stringify(nb.pos)}`);
    check("pasted track grounded", !!mine && [mine.railA, mine.railB].every((id) => scene.constraints.some((c) => c.kind === "ground" && c.joint === id)), "2 grounds");
    check("pasted track owned by the copy", !!mine && scene.ownedTrackJoints(newId).sort().join(",") === [mine.railA, mine.railB].sort().join(","), "owned");
  }
}

console.log(failures === 0 ? "\nAll two-click slider checks passed." : `\n${failures} check(s) FAILED.`);
if (failures > 0) process.exit(1);
