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

// --- 2b) A grounded free joint is kept as an independent anchor (pinned twin), not absorbed. ---
const s2b = new Scene();
const g = s2b.addFreeJoint({ x: 0, y: 0 });
s2b.addGround(g.id, { x: 0, y: 0 });
const f = s2b.addFreeJoint({ x: 100, y: 0 });
const beforeG = s2b.joints.length; // g, f
const body2b = s2b.buildBodyFromJoints([g.id, f.id], 10);

check("grounded free joint stays free (not absorbed)", g.bodyId === null, `${g.bodyId}`);
check("loose free joint still absorbed", f.bodyId === body2b!.id, `${f.bodyId}`);
check("a coincident twin was added for the anchor", s2b.joints.length === beforeG + 1, `${s2b.joints.length} vs ${beforeG + 1}`);
const gpins = s2b.constraints.filter((c) => c.kind === "pin");
check("anchor gets exactly one pin", gpins.length === 1, `${gpins.length}`);
const gpin = gpins[0];
const gLinks = gpin.kind === "pin" && (gpin.jointA === g.id || gpin.jointB === g.id);
check("pin connects the body's twin to the grounded anchor", gLinks);
check("ground constraint still references the original free joint", s2b.constraints.some((c) => c.kind === "ground" && c.joint === g.id));

// --- 2c) Building from a slider's rail node attaches the body to the slider as a rider. ---
const s2c = new Scene();
const r1 = s2c.addFreeJoint({ x: 0, y: 0 });
const r2 = s2c.addFreeJoint({ x: 100, y: 0 }); // these become a world-fixed rail (auto-grounded)
const slider = s2c.addSlider(r1.id, r2.id);
const fc = s2c.addFreeJoint({ x: 50, y: 80 });
const beforeC = s2c.joints.length; // r1, r2, fc
const body2c = s2c.buildBodyFromJoints([r1.id, fc.id], 10);

check("rail node stays a free rail node (not absorbed)", r1.bodyId === null, `${r1.bodyId}`);
check("a coincident joint was added for the rail node", s2c.joints.length === beforeC + 1, `${s2c.joints.length} vs ${beforeC + 1}`);
check("no pin was created to the rail node", s2c.constraints.filter((c) => c.kind === "pin").length === 0);
const twin = s2c.joints.find((j) => j.bodyId === body2c!.id && j.id !== fc.id)!;
check("the body's twin joint rides the slider", slider.riders.includes(twin.id), `riders=${slider.riders.join(",")}`);
check("rail node itself is not made a rider", !slider.riders.includes(r1.id));
check("loose free joint alongside is still absorbed", fc.bodyId === body2c!.id, `${fc.bodyId}`);

// --- 2d) A free slider rider used as a reference is absorbed and stays a rider. ---
const s2d = new Scene();
const d1 = s2d.addFreeJoint({ x: 0, y: 0 });
const d2 = s2d.addFreeJoint({ x: 100, y: 0 });
const slider2 = s2d.addSlider(d1.id, d2.id);
const freeRider = s2d.addFreeJoint({ x: 50, y: 0 });
s2d.attachSliderRider(slider2.id, freeRider.id);
const partner = s2d.addFreeJoint({ x: 50, y: 60 });
const body2d = s2d.buildBodyFromJoints([freeRider.id, partner.id], 10);

check("free rider absorbed into the body", freeRider.bodyId === body2d!.id, `${freeRider.bodyId}`);
check("absorbed rider stays a rider of the slider", slider2.riders.includes(freeRider.id));
check("no extra pin created for the free rider", s2d.constraints.filter((c) => c.kind === "pin").length === 0);

// --- 2e) A rider that belongs to ANOTHER body gets a pinned twin (bodies joined), ---
// ---      not a second independent rider that could slide apart from it.            ---
const s2e = new Scene();
const e1 = s2e.addFreeJoint({ x: 0, y: 0 });
const e2 = s2e.addFreeJoint({ x: 100, y: 0 });
const slider3 = s2e.addSlider(e1.id, e2.id);
const bodyOne = s2e.addBody([{ x: 40, y: 20 }, { x: 60, y: 20 }, { x: 60, y: 60 }, { x: 40, y: 60 }]);
const riderOnBody = s2e.addJoint(bodyOne.id, { x: 50, y: 0 }); // body-one's rider on the slider
s2e.attachSliderRider(slider3.id, riderOnBody.id);
const ep = s2e.addFreeJoint({ x: 50, y: 80 });
const ridersBefore = slider3.riders.length; // [riderOnBody]
const body2e = s2e.buildBodyFromJoints([riderOnBody.id, ep.id], 10);

check("other body's rider stays on its body", riderOnBody.bodyId === bodyOne.id, `${riderOnBody.bodyId}`);
const e2eTwin = s2e.joints.find((j) => j.bodyId === body2e!.id && j.id !== ep.id)!;
const e2ePin = s2e.constraints.filter((c) => c.kind === "pin");
check("exactly one pin created joining the two bodies", e2ePin.length === 1, `${e2ePin.length}`);
const e2eLinks = e2ePin[0].kind === "pin" && (e2ePin[0].jointA === riderOnBody.id || e2ePin[0].jointB === riderOnBody.id);
check("pin connects the new body's twin to the existing rider", e2eLinks);
check("no second independent rider was added", slider3.riders.length === ridersBefore, `riders=${slider3.riders.join(",")}`);
check("the new twin is not itself a rider", !slider3.riders.includes(e2eTwin.id));

// --- 3) Too few joints can't form a body. ---
const s3 = new Scene();
const only = s3.addFreeJoint({ x: 0, y: 0 });
check("one joint is rejected", s3.buildBodyFromJoints([only.id], 10) === null && s3.bodies.length === 0);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
