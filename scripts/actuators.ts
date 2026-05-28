/**
 * Headless coverage for the new linear-actuator and motor constraints. Verifies that:
 *  - addLinearActuator places its rider on the rail and rides it via the existing slider.
 *  - addMotor rejects bad inputs and accepts a valid pivot/crank pair on one body.
 *  - The solver's `anchors` parameter drives an actuator rider to an arbitrary point on
 *    the rail (the rider stays *on* the rail and lands at the requested position).
 *  - The same anchors mechanism turns a body into a motor: pivot stays fixed and crank
 *    orbits at the prescribed radius without disturbing pinned downstream bodies.
 *  - Serialize / load round-trips the new constraint kinds intact.
 *  - Removing the slider that an actuator rides drops the actuator (its rider stays as a
 *    free joint), and removing a motor's body drops the motor.
 */
import { Scene, LinearActuatorConstraint, MotorConstraint } from "../src/model";
import { solve } from "../src/solver";
import { dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

// --- LinearActuator: addLinearActuator + solver `anchors` drive the rider ---
{
  const scene = new Scene();
  // World-fixed track via two free joints (addSlider auto-grounds them).
  const railA = scene.addFreeJoint({ x: 0, y: 0 });
  const railB = scene.addFreeJoint({ x: 200, y: 0 });
  const slider = scene.addSlider(railA.id, railB.id);

  const actuator = scene.addLinearActuator(slider.id, { x: 50, y: 0 })!;
  check("addLinearActuator returns a constraint", !!actuator, `id ${actuator?.id ?? "?"}`);
  const rider = scene.getJoint(actuator.riderId)!;
  check("rider placed near the clicked point", dist(scene.jointWorld(rider), { x: 50, y: 0 }) < 1e-6, `at ${JSON.stringify(scene.jointWorld(rider))}`);
  // Rider is a slider rider on the same slider.
  const sl = scene.constraints.find((c) => c.id === slider.id);
  check("rider attached to the actuator's slider", sl?.kind === "slider" && sl.riders.includes(rider.id), `riders ${(sl?.kind === "slider" ? sl.riders : []).join(",")}`);

  // Drive the rider via the solver's anchors: a moving target on the rail axis.
  let worstOffRail = 0;
  let worstMiss = 0;
  for (let i = 0; i < 10; i++) {
    const target = { x: 20 + 16 * i, y: 0 }; // sweep along the rail
    const anchors = new Map([[rider.id, target]]);
    solve(scene, null, 40, 1, anchors);
    const q = scene.jointWorld(rider);
    worstOffRail = Math.max(worstOffRail, Math.abs(q.y));
    worstMiss = Math.max(worstMiss, dist(q, target));
  }
  check("actuator stays on rail", worstOffRail < 0.5, `max y-offset ${worstOffRail.toFixed(4)} px`);
  check("actuator reaches anchor", worstMiss < 0.5, `max miss ${worstMiss.toFixed(4)} px`);

  // Sweep to each rail endpoint (matching how the wave generator drives during animation):
  // the rider lands exactly on each endpoint without drifting off the rail.
  for (const target of [{ x: 0, y: 0 }, { x: 200, y: 0 }]) {
    solve(scene, null, 40, 1, new Map([[rider.id, target]]));
    const at = scene.jointWorld(rider);
    check(`actuator reaches endpoint (${target.x},${target.y})`, dist(at, target) < 0.5, `at (${at.x.toFixed(3)}, ${at.y.toFixed(3)})`);
  }
}

// --- Motor: addMotor validation ---
{
  const scene = new Scene();
  const bar = scene.addBody([{ x: 0, y: -6 }, { x: 80, y: -6 }, { x: 80, y: 6 }, { x: 0, y: 6 }]);
  const jp = scene.addJoint(bar.id, { x: 0, y: 0 });
  const jc = scene.addJoint(bar.id, { x: 80, y: 0 });
  const otherBody = scene.addBody([{ x: 200, y: -6 }, { x: 280, y: -6 }, { x: 280, y: 6 }, { x: 200, y: 6 }]);
  const jo = scene.addJoint(otherBody.id, { x: 240, y: 0 });
  const freeJ = scene.addFreeJoint({ x: 0, y: 0 });
  check("addMotor rejects same joint as pivot+crank", scene.addMotor(bar.id, jp.id, jp.id) === null, "");
  check("addMotor rejects free-joint pivot", scene.addMotor(bar.id, freeJ.id, jc.id) === null, "");
  check("addMotor rejects cross-body crank", scene.addMotor(bar.id, jp.id, jo.id) === null, "");
  const motor = scene.addMotor(bar.id, jp.id, jc.id);
  check("addMotor accepts valid pivot+crank", motor !== null, `id ${motor?.id ?? "?"}`);
}

// --- Motor in action via the solver's anchors: pivot fixed, crank orbits ---
{
  const scene = new Scene();
  const crank = scene.addBody([{ x: -10, y: -10 }, { x: 10, y: -10 }, { x: 10, y: 10 }, { x: -10, y: 10 }]);
  const pivot = scene.addJoint(crank.id, { x: 0, y: 0 });
  const crankPin = scene.addJoint(crank.id, { x: 0, y: 20 });
  // Downstream: a coupler bar pinned to the crank pin.
  const coupler = scene.addBody([{ x: 0, y: 15 }, { x: 100, y: 15 }, { x: 100, y: 25 }, { x: 0, y: 25 }]);
  const cLeft = scene.addJoint(coupler.id, { x: 0, y: 20 });
  scene.addPin(crankPin.id, cLeft.id);
  scene.addMotor(crank.id, pivot.id, crankPin.id);

  const pivotWorld = { x: 0, y: 0 };
  const r = dist(scene.jointWorld(pivot), scene.jointWorld(crankPin)); // crank radius
  let worstPivotDrift = 0;
  let worstRadiusError = 0;
  let worstPinGap = 0;
  for (let i = 0; i < 16; i++) {
    const theta = (i / 16) * Math.PI * 2;
    const target = { x: pivotWorld.x + r * Math.cos(theta), y: pivotWorld.y + r * Math.sin(theta) };
    const anchors = new Map([
      [pivot.id, pivotWorld],
      [crankPin.id, target],
    ]);
    solve(scene, null, 40, 1, anchors);
    worstPivotDrift = Math.max(worstPivotDrift, dist(scene.jointWorld(pivot), pivotWorld));
    worstRadiusError = Math.max(worstRadiusError, Math.abs(dist(scene.jointWorld(crankPin), pivotWorld) - r));
    worstPinGap = Math.max(worstPinGap, dist(scene.jointWorld(crankPin), scene.jointWorld(cLeft)));
  }
  check("motor pivot stays put", worstPivotDrift < 0.5, `max drift ${worstPivotDrift.toFixed(4)} px`);
  check("crank orbits at constant radius", worstRadiusError < 0.5, `max ΔR ${worstRadiusError.toFixed(4)} px`);
  check("downstream pin stays satisfied during rotation", worstPinGap < 0.5, `max gap ${worstPinGap.toFixed(4)} px`);
}

// --- Persistence round-trip preserves the new constraint kinds ---
{
  const scene = new Scene();
  const ra = scene.addFreeJoint({ x: 0, y: 0 });
  const rb = scene.addFreeJoint({ x: 50, y: 0 });
  const sl = scene.addSlider(ra.id, rb.id);
  const a = scene.addLinearActuator(sl.id);
  a!.speed = 1.5;
  a!.profile = "sine";
  const body = scene.addBody([{ x: 100, y: -8 }, { x: 140, y: -8 }, { x: 140, y: 8 }, { x: 100, y: 8 }]);
  const jp = scene.addJoint(body.id, { x: 120, y: 0 });
  const jc = scene.addJoint(body.id, { x: 140, y: 0 });
  const m = scene.addMotor(body.id, jp.id, jc.id);
  m!.speed = 2.0;

  const data = JSON.parse(JSON.stringify(scene.serialize()));
  const loaded = new Scene();
  loaded.load(data);
  const la = loaded.constraints.find((c) => c.kind === "linearActuator") as LinearActuatorConstraint | undefined;
  const lm = loaded.constraints.find((c) => c.kind === "motor") as MotorConstraint | undefined;
  check("linearActuator survives round-trip", la?.speed === 1.5 && la?.profile === "sine", `speed ${la?.speed} profile ${la?.profile}`);
  check("motor survives round-trip", lm?.speed === 2.0, `speed ${lm?.speed}`);
}

// --- Cascade removal: deleting a slider drops its actuator (rider survives as a free joint) ---
{
  const scene = new Scene();
  const ra = scene.addFreeJoint({ x: 0, y: 0 });
  const rb = scene.addFreeJoint({ x: 100, y: 0 });
  const sl = scene.addSlider(ra.id, rb.id);
  const a = scene.addLinearActuator(sl.id)!;
  scene.removeConstraint(sl.id);
  const stillThere = scene.constraints.some((c) => c.id === a.id);
  const riderStill = !!scene.getJoint(a.riderId);
  check("removing slider drops its actuator", !stillThere, "");
  check("...but the rider joint survives", riderStill, "");
}

// --- Cascade removal: deleting a motor's body drops the motor (via joint removal) ---
{
  const scene = new Scene();
  const bar = scene.addBody([{ x: 0, y: -6 }, { x: 40, y: -6 }, { x: 40, y: 6 }, { x: 0, y: 6 }]);
  const jp = scene.addJoint(bar.id, { x: 0, y: 0 });
  const jc = scene.addJoint(bar.id, { x: 40, y: 0 });
  const m = scene.addMotor(bar.id, jp.id, jc.id)!;
  scene.removeBody(bar.id);
  check("removing the motor's body drops the motor", !scene.constraints.some((c) => c.id === m.id), "");
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
