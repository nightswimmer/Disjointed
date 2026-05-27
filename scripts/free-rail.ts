/**
 * Headless check of a world-fixed slider rail built from two FREE joints.
 * `addSlider` must auto-ground the free rail joints, and the solver must treat the
 * rail as an immovable track: a rider stays on the line, clamps at the end-stops,
 * and the rail joints never move.
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
// Two body-less points define the rail; neither is grounded yet.
const railA = scene.addFreeJoint({ x: 0, y: 0 });
const railB = scene.addFreeJoint({ x: 100, y: 0 });
const groundsBefore = scene.constraints.filter((c) => c.kind === "ground").length;

// Creating the slider on free joints should ground both of them automatically.
const slider = scene.addSlider(railA.id, railB.id);
const groundsAfter = scene.constraints.filter((c) => c.kind === "ground").length;
check("addSlider auto-grounded both free rail joints", groundsAfter - groundsBefore === 2,
  `grounds ${groundsBefore} -> ${groundsAfter}`);

// A free rider confined to the fixed track.
const rider = scene.addFreeJoint({ x: 50, y: 0 });
scene.attachSliderRider(slider.id, rider.id);

// Drive the rider off the line and along it; it must stay on the rail and the rail must hold.
let worstOffset = 0;
let worstRail = 0;
for (let i = 0; i < 24; i++) {
  const a = (i / 24) * Math.PI * 2;
  solve(scene, { jointId: rider.id, target: { x: 50 + 80 * Math.cos(a), y: 60 * Math.sin(a) } }, 40);
  worstOffset = Math.max(worstOffset, distToLine(scene.jointWorld(rider), { x: 0, y: 0 }, { x: 1, y: 0 }));
  worstRail = Math.max(
    worstRail,
    dist(scene.jointWorld(railA), { x: 0, y: 0 }),
    dist(scene.jointWorld(railB), { x: 100, y: 0 })
  );
}
check("rider stays on the fixed rail line", worstOffset < 0.5, `max offset ${worstOffset.toFixed(4)} px`);
check("rail joints stay world-fixed", worstRail < 0.5, `max drift ${worstRail.toFixed(4)} px`);

// End-stops: driving far past each grounded endpoint clamps the rider to it.
for (let i = 0; i < 60; i++) solve(scene, { jointId: rider.id, target: { x: 500, y: 0 } }, 40);
const farEnd = scene.jointWorld(rider);
for (let i = 0; i < 60; i++) solve(scene, { jointId: rider.id, target: { x: -500, y: 0 } }, 40);
const nearEnd = scene.jointWorld(rider);
check("rider stops at far endpoint", farEnd.x <= 100.5 && Math.abs(farEnd.y) < 0.5,
  `x ${farEnd.x.toFixed(3)}, y ${farEnd.y.toFixed(3)}`);
check("rider stops at near endpoint", nearEnd.x >= -0.5 && Math.abs(nearEnd.y) < 0.5,
  `x ${nearEnd.x.toFixed(3)}, y ${nearEnd.y.toFixed(3)}`);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
