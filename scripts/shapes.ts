/** Shape tools' model layer: polygon difference, `Scene.cutBody` (hole fast path, notches,
 *  refusals, ref remaps), the generalized reference geometry (polylines, circles, arcs,
 *  text labels) and the geometry helpers the tools build on. */
import { Scene, MeasureRef } from "../src/model";
import { differenceRegions } from "../src/boolean";
import { polygonArea, dist, vec, Vec2, arcThrough, regularPolygon, distToArc } from "../src/geometry";
import { tryAddConstraint, solveSketch } from "../src/sketch";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}
const sq = (x: number, y: number, w: number, h: number): Vec2[] => [vec(x, y), vec(x + w, y), vec(x + w, y + h), vec(x, y + h)];
const area = (s: Scene, id: number): number => {
  const b = s.getBody(id)!;
  let a = Math.abs(polygonArea(s.bodyWorldVerts(b)));
  for (const h of s.bodyHolesWorld(b)) a -= Math.abs(polygonArea(h));
  return a;
};
const near = (a: number, b: number, tol = 1e-6) => Math.abs(a - b) <= tol;

// ---------------------------------------------------------------- difference primitive
{
  const hole = differenceRegions({ outer: sq(0, 0, 100, 100), holes: [] }, [{ outer: sq(40, 40, 20, 20), holes: [] }])!;
  check("difference: inner square → one region with one hole", hole.regions.length === 1 && hole.regions[0].holes.length === 1 && near(Math.abs(polygonArea(hole.regions[0].holes[0])), 400));
  const notch = differenceRegions({ outer: sq(0, 0, 100, 100), holes: [] }, [{ outer: sq(80, 40, 40, 20), holes: [] }])!;
  check("difference: rectangle across the edge → a notch (8 corners, area 9600)", notch.regions.length === 1 && notch.regions[0].outer.length === 8 && near(polygonArea(notch.regions[0].outer), 9600), `${notch.regions[0].outer.length} verts, area ${polygonArea(notch.regions[0].outer)}`);
  const split = differenceRegions({ outer: sq(0, 0, 100, 100), holes: [] }, [{ outer: sq(45, -10, 10, 120), holes: [] }])!;
  check("difference: a bar right across → two pieces", split.regions.length === 2);
  check("difference: cutter covering everything → null", differenceRegions({ outer: sq(0, 0, 100, 100), holes: [] }, [{ outer: sq(-10, -10, 120, 120), holes: [] }]) === null);
  const grow = differenceRegions({ outer: sq(0, 0, 100, 100), holes: [sq(40, 40, 20, 20)] }, [{ outer: sq(50, 30, 20, 40), holes: [] }])!;
  check("difference: cutter overlapping a hole → one merged hole", grow.regions.length === 1 && grow.regions[0].holes.length === 1 && near(Math.abs(polygonArea(grow.regions[0].holes[0])), 400 + 800 - 200));
}

// ---------------------------------------------------------------- cutBody: fast path (a plain hole)
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 100, 100));
  const res = s.cutBody(plate.id, { control: [vec(50, 50)], radius: 10, round: "offset" });
  check("cut inside → a hole (index 0)", res.ok && res.hole === 0, JSON.stringify(res));
  const h = plate.holes?.[0];
  check("disk cutter stays a parametric disk", !!h && h.round === "offset" && h.controlLocal.length === 1 && near(h.radius, 10));
  check("plate area lost the disk", near(area(s, plate.id), 10000 - Math.abs(polygonArea(s.bodyHolesWorld(plate)[0])), 1e-6));
  check("outline untouched", plate.controlLocal.length === 4);
  // A polygon cutter inside → a radius-0 hole with its exact corners.
  const res2 = s.cutBody(plate.id, sq(10, 10, 15, 15));
  check("second inside cut → hole index 1", res2.ok && res2.hole === 1 && plate.holes?.length === 2);
  check("polygon hole keeps its 4 corners", plate.holes?.[1].controlLocal.length === 4 && plate.holes?.[1].radius === 0);
  // Entirely inside an existing hole: refused.
  const inside = s.cutBody(plate.id, { control: [vec(50, 50)], radius: 3, round: "offset" });
  check("cut inside an existing hole is refused", !inside.ok && /inside an existing hole/.test(inside.ok ? "" : inside.reason));
  check("refusal leaves the plate alone", plate.holes?.length === 2);
}

// ---------------------------------------------------------------- cutBody: notch (general path)
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 100, 100), 5); // rounded corners: they must survive
  const j = s.addJoint(plate.id, vec(20, 20));
  const m = s.addMeasurement("draw", { kind: "vertex", bodyId: plate.id, index: 0 }, { kind: "vertex", bodyId: plate.id, index: 3 }, vec(-10, 50));
  const res = s.cutBody(plate.id, sq(80, 40, 40, 20));
  check("notch across the edge succeeds (no hole index)", res.ok && res.hole === null, JSON.stringify(res));
  check("outline gained the notch's four corners", plate.controlLocal.length === 8, `${plate.controlLocal.length}`);
  // The four original corners stay rounded (r = 5), so the plate never had the full 10000.
  const rounded = 10000 - (4 - Math.PI) * 25;
  check("area lost 20×20", near(area(s, plate.id), rounded - 400, 1), `${area(s, plate.id)} vs ${rounded - 400}`);
  const radii = s.bodyCornerRadii(plate);
  const sharp = radii.filter((r) => r === 0).length;
  const round = radii.filter((r) => near(r, 5)).length;
  check("original corners keep r=5, notch corners start sharp", round === 4 && sharp === 4, `round ${round} sharp ${sharp}`);
  check("joint stays attached to the plate", s.getJoint(j.id)?.bodyId === plate.id);
  check("measurement on untouched corners survives", !!m && s.getMeasurement(m.id) !== undefined);
  check("body stays fillet mode", plate.round === "fillet");
}

// ---------------------------------------------------------------- cutBody: refusals + kept holes
{
  const s = new Scene();
  const plate = s.addBody(sq(0, 0, 100, 100));
  s.cutBody(plate.id, { control: [vec(20, 80)], radius: 5, round: "offset" }); // an untouched disk hole
  const split = s.cutBody(plate.id, sq(45, -10, 10, 120));
  check("a cut that severs the body is refused", !split.ok && /Split/.test(split.ok ? "" : split.reason), split.ok ? "" : split.reason);
  const all = s.cutBody(plate.id, sq(-10, -10, 120, 120));
  check("a cut removing everything is refused", !all.ok);
  check("refusals leave the plate intact", plate.controlLocal.length === 4 && plate.holes?.length === 1);
  // A notch elsewhere keeps the disk hole's exact spec.
  const res = s.cutBody(plate.id, sq(90, 10, 20, 20));
  check("notch with an existing hole succeeds", res.ok);
  const h = plate.holes?.[0];
  check("untouched disk hole keeps its spec", !!h && h.round === "offset" && h.controlLocal.length === 1 && near(h.radius, 5));
  // A cutter overlapping the disk merges into one (now polygonal) hole.
  const over = s.cutBody(plate.id, sq(20, 60, 5, 20));
  check("cut overlapping a hole merges into it", over.ok && plate.holes?.length === 1 && plate.holes[0].controlLocal.length > 4);
  // Instances / missing bodies.
  check("unknown body refused", !s.cutBody(9999, sq(0, 0, 1, 1)).ok);
}

// ---------------------------------------------------------------- one- and two-point offset bodies (circle / slot in the Body role)
{
  const s = new Scene();
  const disk = s.addBody([vec(0, 0)], 20, "offset");
  check("one-point offset body is a disk", near(Math.abs(polygonArea(s.bodyWorldVerts(disk))), Math.PI * 400, 8), `${Math.abs(polygonArea(s.bodyWorldVerts(disk)))}`);
  const slot = s.addBody([vec(0, 0), vec(100, 0)], 10, "offset");
  check("two-point offset body is a slot", near(Math.abs(polygonArea(s.bodyWorldVerts(slot))), 2000 + Math.PI * 100, 5), `${Math.abs(polygonArea(s.bodyWorldVerts(slot)))}`);
}

// ---------------------------------------------------------------- geometry helpers
{
  const arc = arcThrough(vec(0, 0), vec(10, 10), vec(20, 0))!;
  check("arcThrough: centre and radius", !!arc && near(arc.c.x, 10) && near(arc.c.y, 0) && near(arc.r, 10));
  check("arcThrough: passes through the middle point (half turn)", near(Math.abs(arc.sweep), Math.PI, 1e-9));
  check("arcThrough: collinear → null", arcThrough(vec(0, 0), vec(5, 0), vec(10, 0)) === null);
  check("distToArc: on the arc / off its ends", near(distToArc(vec(10, 10), arc), 0) && near(distToArc(vec(10, -10), arc), 10 * Math.SQRT2, 1e-9));
  const hex = regularPolygon(vec(0, 0), vec(10, 0), 6);
  check("regularPolygon: 6 vertices at radius 10", hex.length === 6 && hex.every((p) => near(dist(p, vec(0, 0)), 10)));
}

// ---------------------------------------------------------------- reference geometry
{
  const s = new Scene();
  const body = s.addBody(sq(0, 0, 100, 100));
  const poly = s.addGuidePoly([vec(0, 200), vec(100, 200), vec(100, 250), vec(0, 250), vec(0, 250)], true)!;
  check("closed reference polygon drops the duplicate point", poly.kind === "poly" && poly.pts.length === 4 && poly.closed);
  check("2-point closed polygon rejected", s.addGuidePoly([vec(0, 0), vec(1, 1)], true) === null);
  const seg = s.addGuidePoly([vec(300, 0), vec(400, 0)], false)!;
  check("open 2-point polyline is a segment", seg.kind === "poly" && !seg.closed && seg.pts.length === 2);
  const circle = s.addGuideCircle(vec(500, 0), 30)!;
  check("circle guide created; zero radius refused", circle.kind === "circle" && s.addGuideCircle(vec(0, 0), 0) === null);
  const arc = s.addGuideArc(vec(600, 0), vec(650, 50), vec(700, 0))!;
  check("arc guide created; collinear refused", arc.kind === "arc" && s.addGuideArc(vec(0, 0), vec(1, 0), vec(2, 0)) === null);
  const label = s.addGuideText(vec(20, 30), "  plate ", 8, body.id)!;
  check("text label anchored to a body (trimmed)", label.kind === "text" && label.text === "plate" && label.bodyId === body.id);
  check("empty text refused", s.addGuideText(vec(0, 0), "   ", 8) === null);
  const w0 = s.guideTextWorld(label)!.p;
  s.moveBody(body.id, vec(10, 5));
  const w1 = s.guideTextWorld(label)!.p;
  check("anchored label rides with its body", near(w1.x - w0.x, 10) && near(w1.y - w0.y, 5));
  // Points, lines, hit tests.
  check("poly point keys are indices", s.guidePointKeys(poly).join(",") === "0,1,2,3");
  check("circle: centre is a ref point, rim is a handle only", s.guidePointIsRef(circle, "c") && !s.guidePointIsRef(circle, "r") && s.guideHandleKeys(circle).includes("r"));
  check("guideLines: closed polygon has 4 edges, open segment 1", s.guideLines(poly).length === 4 && s.guideLines(seg).length === 1);
  const gp = s.guidePointAt(vec(101, 251), 5);
  check("guidePointAt finds a polygon vertex", gp?.guide.id === poly.id && gp?.which === "2");
  const gl = s.guideLineAt(vec(50, 199), 5);
  check("guideLineAt finds a polygon edge", gl?.guide.id === poly.id && gl?.edge === 0);
  check("guideAt hits a circle rim and misses its centre", s.guideAt(vec(530, 1), 3)?.id === circle.id && s.guideAt(vec(500, 0), 3) === undefined);
  check("guideAt hits the arc", s.guideAt(vec(650, 50), 3)?.id === arc.id);
  check("guideAt hits the label box", s.guideAt(s.guideTextWorld(label)!.p, 1)?.id === label.id);
  // Resolving refs.
  const edgeRef: MeasureRef = { kind: "guideLine", guideId: poly.id, edge: 1 };
  const r = s.resolveMeasureRef(edgeRef);
  check("polyline edge resolves as a finite line", r?.kind === "line" && !r.infinite && near(r.a.x, 100) && near(r.b.y, 250));
  check("rim handle is not a reference", s.resolveMeasureRef({ kind: "guidePoint", guideId: circle.id, which: "r" }) === null);
  check("centre is a reference", s.resolveMeasureRef({ kind: "guidePoint", guideId: circle.id, which: "c" })?.kind === "point");
  // Moves.
  s.moveGuidePoint(circle.id, "r", vec(540, 0));
  check("dragging the rim handle resizes the circle", circle.kind === "circle" && near(circle.r, 40));
  s.moveGuide(poly.id, vec(1, 1));
  check("moveGuide translates every polygon point", poly.kind === "poly" && near(poly.pts[0].x, 1) && near(poly.pts[3].y, 251));
  // Constraints: a tie pulls the (construction) guide onto geometry; equal works on a finite edge, not on an infinite line.
  const j = s.addFreeJoint(vec(150, 150));
  const tie = tryAddConstraint(s, "coincident", { kind: "guidePoint", guideId: poly.id, which: "0" }, { kind: "joint", jointId: j.id });
  check("polygon vertex ties to a joint (guide moves)", tie.constraint !== null && poly.kind === "poly" && near(poly.pts[0].x, 150) && near(poly.pts[0].y, 150));
  check("joint did not move for the tie", near(s.jointWorld(j).x, 150) && near(s.jointWorld(j).y, 150));
  const eq = s.addSketchConstraint("equal", { kind: "guideLine", guideId: seg.id, edge: 0 }, { kind: "edge", bodyId: body.id, index: 0 });
  check("equal accepted on a reference segment", eq !== null);
  const line = s.addGuide(vec(0, -50), vec(100, -50))!;
  check("equal still rejected on an infinite guideline", s.addSketchConstraint("equal", { kind: "guideLine", guideId: line.id }, { kind: "edge", bodyId: body.id, index: 0 }) === null);
  const breaks = solveSketch(s);
  check("sketch with mixed guides solves", breaks.length === 0, `${breaks.length} breaks`);
  // Deleting the body drops its label, nothing else.
  const count = s.guides.length;
  s.deleteBody(body.id);
  check("deleting the body removes its anchored label only", s.guides.length === count - 1 && s.getGuide(label.id) === undefined);
}

// ---------------------------------------------------------------- serialization
{
  const s = new Scene();
  const b = s.addBody(sq(0, 0, 50, 50));
  s.addGuide(vec(0, 0), vec(10, 0));
  s.addGuidePoly([vec(0, 0), vec(10, 0), vec(10, 10)], true);
  s.addGuidePoly([vec(0, 100), vec(10, 100)], false);
  s.addGuideCircle(vec(50, 50), 7);
  s.addGuideArc(vec(0, 0), vec(5, 5), vec(10, 0));
  s.addGuideText(vec(5, 5), "hi", 4, b.id);
  const data = JSON.parse(JSON.stringify(s.serialize()));
  check("format v20", data.version === 20);
  const t = new Scene();
  t.load(data);
  check("all six guide kinds round-trip", t.guides.length === 6 && t.guides.map((g) => g.kind).join(",") === "line,poly,poly,circle,arc,text");
  const txt = t.guides.find((g) => g.kind === "text");
  check("text keeps its body anchor", !!txt && txt.kind === "text" && txt.bodyId === b.id && txt.size === 4);
  // Legacy (≤ v19) guides carry no kind: they load as infinite lines.
  const legacy = new Scene();
  legacy.load({ version: 19, bodies: [], joints: [], constraints: [], guides: [{ id: 1, a: { x: 0, y: 0 }, b: { x: 5, y: 5 } } as never] });
  check("legacy guide loads as an infinite line", legacy.guides.length === 1 && legacy.guides[0].kind === "line");
  // Corrupt records are dropped, a label whose body is gone too.
  const junk = new Scene();
  junk.load({ version: 20, bodies: [], joints: [], constraints: [], guides: [
    { id: 1, kind: "circle", c: { x: 0, y: 0 }, r: 0 } as never,
    { id: 2, kind: "text", p: { x: 0, y: 0 }, text: "orphan", size: 3, bodyId: 77 } as never,
    { id: 3, kind: "poly", pts: [{ x: 0, y: 0 }], closed: false } as never,
    { id: 4, kind: "arc", a: { x: 0, y: 0 }, m: { x: 1, y: 1 }, b: { x: 2, y: 0 } } as never,
  ] });
  check("degenerate / orphaned guides dropped on load", junk.guides.length === 1 && junk.guides[0].kind === "arc", `${junk.guides.length}`);
}

if (failures) {
  console.log(`\n${failures} FAILURE(S)`);
  process.exit(1);
}
console.log("All shape checks passed.");
