/** Building a body from existing joints: absorbs free joints, pins to other bodies. */
import { Scene } from "../src/model";
import { polygonArea, pointInPolygon } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

// --- 1) Body from two free joints: both get absorbed and end up inside the body. ---
const s = new Scene();
const a = s.addFreeJoint({ x: 0, y: 0 });
const b = s.addFreeJoint({ x: 100, y: 0 });
const body = s.buildBodyFromJoints([a.id, b.id], 10);

check("body created from joints", body !== null && s.bodies.length === 1, `${s.bodies.length}`);
check("free joints absorbed into body", a.bodyId === body!.id && b.bodyId === body!.id, `${a.bodyId}, ${b.bodyId}`);
const verts = s.bodyWorldVerts(body!);
check("expanded body has area", Math.abs(polygonArea(verts)) > 100, `${Math.abs(polygonArea(verts)).toFixed(0)}`);
check("joint A lies inside the body", pointInPolygon(s.jointWorld(a), verts));
check("joint B lies inside the body", pointInPolygon(s.jointWorld(b), verts));

// --- 2) Body using a joint from another body: that joint stays, a pinned twin is made. ---
const s2 = new Scene();
const other = s2.addBody([{ x: 200, y: -10 }, { x: 240, y: -10 }, { x: 240, y: 10 }, { x: 200, y: 10 }]);
const oj = s2.addJoint(other.id, { x: 220, y: 0 });
const f1 = s2.addFreeJoint({ x: 0, y: 0 });
const f2 = s2.addFreeJoint({ x: 0, y: 80 });
const before = s2.joints.length; // oj, f1, f2
const body2 = s2.buildBodyFromJoints([f1.id, f2.id, oj.id], 12);

check("other body's joint stays on it", oj.bodyId === other.id);
check("a coincident joint was added", s2.joints.length === before + 1, `${s2.joints.length} vs ${before + 1}`);
const pins = s2.constraints.filter((c) => c.kind === "pin");
check("exactly one pin created", pins.length === 1, `${pins.length}`);
const pin = pins[0];
const links = pin.kind === "pin" && (pin.jointA === oj.id || pin.jointB === oj.id);
check("pin connects the new body to the other joint", links);
check("free joints f1/f2 absorbed", f1.bodyId === body2!.id && f2.bodyId === body2!.id);

// --- 3) Too few joints can't form a body. ---
const s3 = new Scene();
const only = s3.addFreeJoint({ x: 0, y: 0 });
check("one joint is rejected", s3.buildBodyFromJoints([only.id], 10) === null && s3.bodies.length === 0);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
