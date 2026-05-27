/** Corner filleting + body editing: radius changes and vertex moves keep joints anchored. */
import { Scene } from "../src/model";
import { filletPolygon, polygonArea, dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

const finite = (pts: { x: number; y: number }[]) => pts.every((p) => Number.isFinite(p.x) && Number.isFinite(p.y));

type Pt = { x: number; y: number };
/** Count proper crossings between non-adjacent edges of a closed polygon. */
function selfIntersections(poly: Pt[]): number {
  const o = (p: Pt, q: Pt, r: Pt) => Math.sign((q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x));
  const cross = (a: Pt, b: Pt, c: Pt, d: Pt) => {
    const o1 = o(a, b, c), o2 = o(a, b, d), o3 = o(c, d, a), o4 = o(c, d, b);
    return o1 !== o2 && o3 !== o4 && o1 !== 0 && o2 !== 0 && o3 !== 0 && o4 !== 0;
  };
  // Drop near-coincident points so two fillets meeting tangentially at a maxed-out edge
  // (a valid touch) collapse to one shared vertex instead of registering as a crossing.
  const p: Pt[] = [];
  for (const v of poly) {
    const last = p[p.length - 1];
    if (!last || Math.hypot(v.x - last.x, v.y - last.y) > 0.5) p.push(v);
  }
  let count = 0;
  const n = p.length;
  for (let i = 0; i < n; i++)
    for (let j = i + 1; j < n; j++) {
      if ((i + 1) % n === j || (j + 1) % n === i) continue; // skip adjacent edges
      if (cross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n])) count++;
    }
  return count;
}

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
// The reflex corner at (40,40) must round into the *notch* (x>40, y>40), not the material
// (x<40, y<40). The old code flipped the arc to the wrong side.
const nearReflex = (pred: (p: { x: number; y: number }) => boolean) =>
  roundedL.filter((p) => Math.hypot(p.x - 40, p.y - 40) < 14 && pred(p)).length;
check("reflex corner rounds into the notch", nearReflex((p) => p.x > 40 && p.y > 40) > 0, `${nearReflex((p) => p.x > 40 && p.y > 40)} pts`);
check("reflex fillet stays off the material side", nearReflex((p) => p.x < 40 - 0.1 && p.y < 40 - 0.1) === 0);

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

// --- narrow-neck fillet doesn't overlap / fold (shared-edge budget) ---
const neck = [
  { x: 0, y: 0 }, { x: 300, y: 0 }, { x: 300, y: 40 }, { x: 175, y: 50 },
  { x: 165, y: 200 }, { x: 135, y: 200 }, { x: 125, y: 50 }, { x: 0, y: 40 },
]; // wide base + thin 30px-wide neck rising to the top
for (const r of [16, 30, 60, 200]) {
  const f = filletPolygon(neck, r);
  check(`narrow neck fillet stays simple at r=${r}`, finite(f) && selfIntersections(f) === 0, `${selfIntersections(f)} crossings`);
}

// --- add / remove control vertices ---
const e = new Scene();
const eb = e.addBody([{ x: 0, y: 0 }, { x: 100, y: 0 }, { x: 100, y: 100 }, { x: 0, y: 100 }]);
const ej = e.addJoint(eb.id, { x: 50, y: 50 });

const beforeInsert = e.jointWorld(ej);
e.insertBodyVertex(eb.id, 1, { x: 50, y: 0 }); // node on the bottom edge (between v0 and v1)
check("insert adds a control vertex", eb.controlLocal.length === 5, `${eb.controlLocal.length}`);
check("joint stays put when inserting a vertex", dist(beforeInsert, e.jointWorld(ej)) < 1e-9, `moved ${dist(beforeInsert, e.jointWorld(ej)).toExponential(1)}`);

e.removeBodyVertex(eb.id, 1);
check("remove drops a control vertex", eb.controlLocal.length === 4, `${eb.controlLocal.length}`);
e.removeBodyVertex(eb.id, 0);
check("remove down to a triangle works", eb.controlLocal.length === 3, `${eb.controlLocal.length}`);
e.removeBodyVertex(eb.id, 0);
check("remove is a no-op at the 3-vertex minimum", eb.controlLocal.length === 3, `${eb.controlLocal.length}`);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
