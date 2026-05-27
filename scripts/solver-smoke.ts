/**
 * Headless sanity check of the constraint solver — no DOM required.
 * Builds a grounded crank, drives a joint, and asserts constraints hold.
 */
import { Scene } from "../src/model";
import { solve } from "../src/solver";
import { dist, distToLine } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

const scene = new Scene();

// Crank: a small square near the origin. Coupler: a bar to its right.
const crank = scene.addBody([
  { x: 80, y: 180 },
  { x: 120, y: 180 },
  { x: 120, y: 220 },
  { x: 80, y: 220 },
]);
const coupler = scene.addBody([
  { x: 200, y: 190 },
  { x: 360, y: 190 },
  { x: 360, y: 210 },
  { x: 200, y: 210 },
]);

// Crank pivot (grounded) at its center, and a crank pin on its right edge.
const crankCenter = scene.addJoint(crank.id, { x: 100, y: 200 });
const crankPin = scene.addJoint(crank.id, { x: 120, y: 200 });
// Coupler ends.
const couplerLeft = scene.addJoint(coupler.id, { x: 200, y: 200 });
const couplerRight = scene.addJoint(coupler.id, { x: 360, y: 200 });

scene.addGround(crankCenter.id, { x: 100, y: 200 });
scene.addPin(crankPin.id, couplerLeft.id);
// Constrain the coupler's far end to a horizontal slider rail.
scene.addSlider(couplerRight.id, { x: 360, y: 200 }, { x: 1, y: 0 });

// Drive the crank pin around the pivot and verify constraints stay satisfied.
let worstGround = 0;
let worstPin = 0;
let worstSlide = 0;
for (let i = 0; i < 16; i++) {
  const a = (i / 16) * Math.PI * 2;
  const target = { x: 100 + 20 * Math.cos(a), y: 200 + 20 * Math.sin(a) };
  solve(scene, { jointId: crankPin.id, target }, 60);

  worstGround = Math.max(worstGround, dist(scene.jointWorld(crankCenter), { x: 100, y: 200 }));
  worstPin = Math.max(worstPin, dist(scene.jointWorld(crankPin), scene.jointWorld(couplerLeft)));
  worstSlide = Math.max(
    worstSlide,
    distToLine(scene.jointWorld(couplerRight), { x: 360, y: 200 }, { x: 1, y: 0 })
  );
}

check("ground stays fixed", worstGround < 0.5, `max drift ${worstGround.toFixed(4)} px`);
check("pin stays coincident", worstPin < 0.5, `max gap ${worstPin.toFixed(4)} px`);
check("slider stays on rail", worstSlide < 0.5, `max offset ${worstSlide.toFixed(4)} px`);

// Crank pivot must remain on the ground after a full revolution.
console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
