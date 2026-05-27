/** Round-trips a scene through serialize → JSON → load and checks it survives. */
import { Scene } from "../src/model";
import { dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

const a = new Scene();
const b1 = a.addBody([{ x: 80, y: 180 }, { x: 120, y: 180 }, { x: 120, y: 220 }, { x: 80, y: 220 }]);
const b2 = a.addBody([{ x: 200, y: 190 }, { x: 360, y: 190 }, { x: 360, y: 210 }, { x: 200, y: 210 }]);
const j1 = a.addJoint(b1.id, { x: 100, y: 200 });
const j2 = a.addJoint(b1.id, { x: 120, y: 200 });
const j3 = a.addJoint(b2.id, { x: 200, y: 200 });
const j4 = a.addJoint(b2.id, { x: 360, y: 200 });
a.addGround(j1.id, { x: 100, y: 200 });
a.addPin(j2.id, j3.id);
// Slider: rail = two joints on b1 (j1, j2); j4 (on b2) is attached as a rider.
const slider = a.addSlider(j1.id, j2.id);
a.attachSliderRider(slider.id, j4.id);

// Serialize → JSON text → parse → load into a fresh scene (simulates save/load).
const text = JSON.stringify(a.serialize());
const b = new Scene();
b.load(JSON.parse(text));

check("body count", b.bodies.length === a.bodies.length, `${b.bodies.length}`);
check("joint count", b.joints.length === a.joints.length, `${b.joints.length}`);
check("constraint count", b.constraints.length === a.constraints.length, `${b.constraints.length}`);

let worstJoint = 0;
for (const j of a.joints) {
  const jb = b.getJoint(j.id)!;
  worstJoint = Math.max(worstJoint, dist(a.jointWorld(j), b.jointWorld(jb)));
}
check("joint world positions preserved", worstJoint < 1e-9, `max diff ${worstJoint}`);

const kinds = b.constraints.map((c) => c.kind).sort().join(",");
check("constraint kinds preserved", kinds === "ground,pin,slider", kinds);

const ls = b.constraints.find((c) => c.kind === "slider");
check(
  "slider riders preserved",
  ls?.kind === "slider" && ls.riders.length === 1 && ls.riders[0] === j4.id,
  ls?.kind === "slider" ? `riders=[${ls.riders}]` : "no slider"
);

// New ids must not collide with loaded ones.
const maxId = Math.max(...b.bodies.map((x) => x.id), ...b.joints.map((x) => x.id), ...b.constraints.map((x) => x.id));
const fresh = b.addJoint(b.bodies[0].id, { x: 0, y: 0 });
check("nextId continues past loaded ids", fresh.id > maxId, `new id ${fresh.id} > ${maxId}`);

// Loaded scene must be independent of the source (no shared references).
b.bodies[0].pos.x += 999;
check("load is a deep copy", a.bodies[0].pos.x !== b.bodies[0].pos.x);

// Malformed input must throw.
let threw = false;
try {
  new Scene().load({ bodies: "nope" } as never);
} catch {
  threw = true;
}
check("rejects malformed data", threw);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
