/**
 * Verifies that dragging a joint on a grounded body cannot move the ground
 * point: the body may only rotate about it, and the dragged joint snaps to the
 * nearest point it can reach (its circle around the ground).
 */
import { Scene } from "../src/model";
import { solve } from "../src/solver";
import { dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

const G = { x: 200, y: 200 }; // ground point
const scene = new Scene();
const body = scene.addBody([
  { x: 180, y: 180 }, { x: 320, y: 180 }, { x: 320, y: 220 }, { x: 180, y: 220 },
]);
const groundJoint = scene.addJoint(body.id, G);
const handle = scene.addJoint(body.id, { x: 300, y: 200 }); // 100px right of ground
scene.addGround(groundJoint.id, G);

const radius = dist(scene.jointWorld(handle), G); // fixed distance handle->ground

// Drag the handle to assorted unreachable targets (too far, off-axis, behind).
const targets = [
  { x: 600, y: 200 },   // straight out, way past reach
  { x: 200, y: -300 },  // straight up, way past reach
  { x: 500, y: 500 },   // diagonal, past reach
  { x: 50, y: 250 },    // behind / other side
];

let worstGroundDrift = 0;
let worstRadiusErr = 0;
let worstAngleErr = 0;
for (const target of targets) {
  solve(scene, { jointId: handle.id, target }, 120);
  const g = scene.jointWorld(groundJoint);
  const h = scene.jointWorld(handle);
  worstGroundDrift = Math.max(worstGroundDrift, dist(g, G));
  worstRadiusErr = Math.max(worstRadiusErr, Math.abs(dist(h, G) - radius));
  // Nearest reachable point lies along the ground->target direction at `radius`.
  const wantAngle = Math.atan2(target.y - G.y, target.x - G.x);
  const gotAngle = Math.atan2(h.y - G.y, h.x - G.x);
  let da = Math.abs(wantAngle - gotAngle) % (2 * Math.PI);
  if (da > Math.PI) da = 2 * Math.PI - da;
  worstAngleErr = Math.max(worstAngleErr, da);
}

check("ground point never moves", worstGroundDrift < 0.5, `max drift ${worstGroundDrift.toFixed(4)} px`);
check("handle stays on its circle", worstRadiusErr < 0.5, `max radius error ${worstRadiusErr.toFixed(4)} px`);
check(
  "handle snaps to nearest reachable angle",
  worstAngleErr < 0.02,
  `max angle error ${((worstAngleErr * 180) / Math.PI).toFixed(3)} deg`
);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
