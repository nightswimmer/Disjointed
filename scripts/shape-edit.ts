/** Corner filleting + body editing: radius changes and vertex moves keep joints anchored. */
import { Scene } from "../src/model";
import { filletPolygon, polygonArea, dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

const finite = (pts: { x: number; y: number }[]) => pts.every((p) => Number.isFinite(p.x) && Number.isFinite(p.y));

// --- filletPolygon ---
const sq = [{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }];
const rounded = filletPolygon(sq, 20);
check("fillet adds arc vertices", rounded.length > 4, `${rounded.length}`);
check("fillet output is finite", finite(rounded));
const areaR = Math.abs(polygonArea(rounded));
check("fillet trims a little area off the square", areaR > 9000 && areaR < 10000, `${areaR.toFixed(0)}`);
check("radius 0 returns the original polygon", filletPolygon(sq, 0).length === 4);

const ell = [
  { x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 40 },
  { x: 40, y: 40 }, { x: 40, y: 100 }, { x: 0, y: 100 },
]; // concave L (has a reflex corner)
const roundedL = filletPolygon(ell, 10);
check("concave fillet stays valid", finite(roundedL) && Math.abs(polygonArea(roundedL)) > 0, `area ${Math.abs(polygonArea(roundedL)).toFixed(0)}`);

// --- editing keeps attached joints anchored ---
const s = new Scene();
const body = s.addBody(sq);
const j = s.addJoint(body.id, { x: 20, y: 20 });

const beforeRadius = s.jointWorld(j);
s.setBodyRadius(body.id, 15);
check("joint stays put when rounding the body", dist(beforeRadius, s.jointWorld(j)) < 1e-9, `moved ${dist(beforeRadius, s.jointWorld(j)).toExponential(1)}`);
check("rounded body still has area", Math.abs(polygonArea(s.bodyWorldVerts(body))) > 0);

const areaBefore = Math.abs(polygonArea(s.bodyWorldVerts(body)));
const beforeMove = s.jointWorld(j);
s.moveBodyVertex(body.id, 0, { x: -40, y: -40 }); // drag a corner out
check("joint stays put when moving a vertex", dist(beforeMove, s.jointWorld(j)) < 1e-9, `moved ${dist(beforeMove, s.jointWorld(j)).toExponential(1)}`);
check("moving a vertex changes the body area", Math.abs(Math.abs(polygonArea(s.bodyWorldVerts(body))) - areaBefore) > 1);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
