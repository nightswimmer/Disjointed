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

// A grounded rail body gives a fixed horizontal track at y = 200; the coupler's far
// end rides it. Two grounds fully lock the rail body, so the rail is world-fixed.
// The rail spans the coupler end's full travel (~x 239–280) so the end-stops never trip.
const railBody = scene.addBody([
  { x: 210, y: 196 },
  { x: 330, y: 196 },
  { x: 330, y: 204 },
  { x: 210, y: 204 },
]);
const railA = scene.addJoint(railBody.id, { x: 220, y: 200 });
const railB = scene.addJoint(railBody.id, { x: 320, y: 200 });
scene.addGround(railA.id, { x: 220, y: 200 });
scene.addGround(railB.id, { x: 320, y: 200 });
const railSlider = scene.addSlider(railA.id, railB.id);
scene.attachSliderRider(railSlider.id, couplerRight.id);

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
    distToLine(scene.jointWorld(couplerRight), { x: 220, y: 200 }, { x: 1, y: 0 })
  );
}

check("ground stays fixed", worstGround < 0.5, `max drift ${worstGround.toFixed(4)} px`);
check("pin stays coincident", worstPin < 0.5, `max gap ${worstPin.toFixed(4)} px`);
check("slider stays on rail", worstSlide < 0.5, `max offset ${worstSlide.toFixed(4)} px`);

// --- Slider end-stops: the rider is clamped between the two rail joints. ---
const limit = new Scene();
const limRail = limit.addBody([{ x: -10, y: -6 }, { x: 110, y: -6 }, { x: 110, y: 6 }, { x: -10, y: 6 }]);
const lr1 = limit.addJoint(limRail.id, { x: 0, y: 0 });
const lr2 = limit.addJoint(limRail.id, { x: 100, y: 0 });
limit.addGround(lr1.id, { x: 0, y: 0 });
limit.addGround(lr2.id, { x: 100, y: 0 });
const block = limit.addBody([{ x: 40, y: -6 }, { x: 60, y: -6 }, { x: 60, y: 6 }, { x: 40, y: 6 }]);
const slide = limit.addJoint(block.id, { x: 50, y: 0 });
const limSlider = limit.addSlider(lr1.id, lr2.id);
limit.attachSliderRider(limSlider.id, slide.id);

// Drive the rider far past the right end (x = 100) and the left end (x = 0).
for (let i = 0; i < 60; i++) solve(limit, { jointId: slide.id, target: { x: 500, y: 0 } }, 40);
const farEnd = limit.jointWorld(slide);
for (let i = 0; i < 60; i++) solve(limit, { jointId: slide.id, target: { x: -500, y: 0 } }, 40);
const nearEnd = limit.jointWorld(slide);

check("rider stops at far end", farEnd.x <= 100.5 && Math.abs(farEnd.y) < 0.5, `x ${farEnd.x.toFixed(3)}, y ${farEnd.y.toFixed(3)}`);
check("rider stops at near end", nearEnd.x >= -0.5 && Math.abs(nearEnd.y) < 0.5, `x ${nearEnd.x.toFixed(3)}, y ${nearEnd.y.toFixed(3)}`);

// --- Free joints: a grounded free joint is a body-less pivot; ungrounded it moves. ---
const fj = new Scene();
const anchor = fj.addFreeJoint({ x: 0, y: 0 }); // body-less point
fj.addGround(anchor.id, { x: 0, y: 0 }); // ...pinned in place → an anchor
const bar = fj.addBody([{ x: 0, y: -8 }, { x: 120, y: -8 }, { x: 120, y: 8 }, { x: 0, y: 8 }]);
const barLeft = fj.addJoint(bar.id, { x: 0, y: 0 });
const barRight = fj.addJoint(bar.id, { x: 120, y: 0 });
fj.addPin(anchor.id, barLeft.id); // the bar hangs off the free-joint anchor

let worstAnchor = 0;
let worstPivot = 0;
for (let i = 0; i < 24; i++) {
  const a = (i / 24) * Math.PI * 2;
  solve(fj, { jointId: barRight.id, target: { x: 120 * Math.cos(a), y: 120 * Math.sin(a) } }, 40);
  worstAnchor = Math.max(worstAnchor, dist(fj.jointWorld(anchor), { x: 0, y: 0 }));
  worstPivot = Math.max(worstPivot, dist(fj.jointWorld(anchor), fj.jointWorld(barLeft)));
}
check("grounded free joint stays put", worstAnchor < 0.5, `max drift ${worstAnchor.toFixed(4)} px`);
check("body pivots on free-joint anchor", worstPivot < 0.5, `max gap ${worstPivot.toFixed(4)} px`);

// An ungrounded free joint is movable: pinned to a joint grounded elsewhere, it follows.
const fj2 = new Scene();
const gb = fj2.addBody([{ x: 0, y: -8 }, { x: 40, y: -8 }, { x: 40, y: 8 }, { x: 0, y: 8 }]);
const gj = fj2.addJoint(gb.id, { x: 20, y: 0 });
fj2.addGround(gj.id, { x: 200, y: 50 });
const free = fj2.addFreeJoint({ x: 0, y: 0 });
fj2.addPin(free.id, gj.id);
solve(fj2, null, 80);
const moved = fj2.jointWorld(free);
check("ungrounded free joint follows its pin", dist(moved, { x: 200, y: 50 }) < 0.5, `at (${moved.x.toFixed(2)}, ${moved.y.toFixed(2)})`);

// Crank pivot must remain on the ground after a full revolution.
console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
