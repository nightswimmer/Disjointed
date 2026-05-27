/** Edit utilities: rotateBody (rigid turn about a pivot), mirrorBody, copy/paste. */
import { Scene } from "../src/model";
import { Vec2, dist, polygonArea } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const near = (a: Vec2, b: Vec2, tol = 1e-6) => dist(a, b) < tol;

// --- 1) rotateBody: 90° about the centroid carries the joint and its ground anchor. ---
{
  const s = new Scene();
  const body = s.addBody([{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: 40 }, { x: 0, y: 40 }]);
  const j = s.addJoint(body.id, { x: 40, y: 20 }); // right-edge midpoint
  s.addGround(j.id, s.jointWorld(j));
  const pivot = { x: 20, y: 20 }; // centroid
  s.rotateBody(body.id, pivot, Math.PI / 2);

  check("centroid fixed when pivot = centroid", near(body.pos, { x: 20, y: 20 }), `${body.pos.x},${body.pos.y}`);
  check("body angle advanced by 90°", Math.abs(body.angle - Math.PI / 2) < 1e-9);
  check("joint rotated about pivot", near(s.jointWorld(j), { x: 20, y: 40 }), `${s.jointWorld(j).x.toFixed(2)},${s.jointWorld(j).y.toFixed(2)}`);
  const g = s.constraints.find((c) => c.kind === "ground");
  check("ground anchor followed the joint", g?.kind === "ground" && near(g.anchor, { x: 20, y: 40 }));
}

// --- 2) rotateBody about a control node leaves that node fixed in world space. ---
{
  const s = new Scene();
  const body = s.addBody([{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: 40 }, { x: 0, y: 40 }]);
  const node = s.bodyControlWorld(body)[0]; // (0,0)
  s.rotateBody(body.id, node, Math.PI / 3);
  check("pivot node stays put under rotation", near(s.bodyControlWorld(body)[0], node, 1e-6), `${s.bodyControlWorld(body)[0].x.toFixed(2)},${s.bodyControlWorld(body)[0].y.toFixed(2)}`);
}

// --- 3) mirrorBody: reflects joints in place, keeping the centroid and area. ---
{
  const s = new Scene();
  const body = s.addBody([{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: 20 }, { x: 0, y: 20 }]);
  const j = s.addJoint(body.id, { x: 40, y: 10 }); // right-middle
  const areaBefore = Math.abs(polygonArea(s.bodyWorldVerts(body)));
  s.mirrorBody(body.id, "h");
  check("centroid unchanged by mirror", near(body.pos, { x: 20, y: 10 }, 1e-6), `${body.pos.x.toFixed(2)},${body.pos.y.toFixed(2)}`);
  check("joint reflected across the vertical axis", near(s.jointWorld(j), { x: 0, y: 10 }, 1e-6), `${s.jointWorld(j).x.toFixed(2)},${s.jointWorld(j).y.toFixed(2)}`);
  const areaAfter = Math.abs(polygonArea(s.bodyWorldVerts(body)));
  check("area preserved by mirror", Math.abs(areaAfter - areaBefore) < 1e-6, `${areaBefore.toFixed(1)} -> ${areaAfter.toFixed(1)}`);
}

// --- 4) copy/paste: extract → insert makes an independent, offset duplicate. ---
{
  const s = new Scene();
  const body = s.addBody([{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: 40 }, { x: 0, y: 40 }]);
  const j1 = s.addJoint(body.id, { x: 20, y: 20 });
  const j2 = s.addJoint(body.id, { x: 0, y: 0 });
  s.addGround(j1.id, s.jointWorld(j1));
  s.addSlider(j1.id, j2.id); // rail fully on this body → should be copied

  const clip = s.extractBody(body.id)!;
  const at = { x: 120, y: 20 }; // centroid (20,20) → translate by (100,0)
  const newId = s.insertBody(clip, at)!;

  check("a second body was created", s.bodies.length === 2 && newId !== body.id);
  check("pasted centroid lands at the drop point", near(s.getBody(newId)!.pos, at, 1e-6), `${s.getBody(newId)!.pos.x.toFixed(2)},${s.getBody(newId)!.pos.y.toFixed(2)}`);
  const newJoints = s.joints.filter((j) => j.bodyId === newId);
  check("both joints duplicated", newJoints.length === 2, `${newJoints.length}`);
  check("grounds duplicated", s.constraints.filter((c) => c.kind === "ground").length === 2);
  check("sliders duplicated", s.constraints.filter((c) => c.kind === "slider").length === 2);
  // The duplicate references only its own fresh joints (no shared ids with the original).
  const origIds = new Set([j1.id, j2.id]);
  check("duplicate joints are fresh ids", newJoints.every((j) => !origIds.has(j.id)));

  // Independence: moving the original body leaves the copy untouched.
  const copyPosBefore = { ...s.getBody(newId)!.pos };
  s.moveBody(body.id, { x: 999, y: 999 });
  check("copy is independent of the original", near(s.getBody(newId)!.pos, copyPosBefore, 1e-9));
}

// --- 5) copy/paste drops a cross-body pin (it can't be reproduced in isolation). ---
{
  const s = new Scene();
  const a = s.addBody([{ x: 0, y: 0 }, { x: 20, y: 0 }, { x: 20, y: 20 }, { x: 0, y: 20 }]);
  const b = s.addBody([{ x: 50, y: 0 }, { x: 70, y: 0 }, { x: 70, y: 20 }, { x: 50, y: 20 }]);
  const ja = s.addJoint(a.id, { x: 10, y: 10 });
  const jb = s.addJoint(b.id, { x: 60, y: 10 });
  s.addPin(ja.id, jb.id);
  const clip = s.extractBody(a.id)!;
  check("cross-body pin not captured in the clip", clip.pins.length === 0, `${clip.pins.length}`);
  const pinsBefore = s.constraints.filter((c) => c.kind === "pin").length;
  s.insertBody(clip, { x: 0, y: 100 });
  const pinsAfter = s.constraints.filter((c) => c.kind === "pin").length;
  check("paste adds no new pins", pinsAfter === pinsBefore, `${pinsBefore} -> ${pinsAfter}`);
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
