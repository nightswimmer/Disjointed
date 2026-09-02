/**
 * Headless checks of the DXF importer (src/dxf.ts), the working-unit setting, and
 * bodies with holes: LWPOLYLINE / POLYLINE loops, bulge + arc sampling, LINE/ARC
 * chaining into loops, CIRCLE sampling, $INSUNITS reading, skipped open paths /
 * unsupported entities, loop nesting (holes / islands), composite mass properties,
 * hole-agnostic containment, hole transforms (mirror / scale / rotate / copy), and
 * unit + holes surviving serialize → load (v13) with pre-v12/v13 defaults.
 */
import { parseDxf, nestLoops, loopSignedArea } from "../src/dxf";
import { Scene, SceneData } from "../src/model";
import { Vec2, dist, polygonCentroid } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}

/** Wrap entity pair lines in SECTION scaffolding (plus an optional HEADER body). */
function dxf(entities: string, header = ""): string {
  const h = header ? `0\nSECTION\n2\nHEADER\n${header}0\nENDSEC\n` : "";
  return `${h}0\nSECTION\n2\nENTITIES\n${entities}0\nENDSEC\n0\nEOF\n`;
}
const insunits = (code: number) => `9\n$INSUNITS\n70\n${code}\n`;

// --- closed LWPOLYLINE ------------------------------------------------------
{
  const square =
    "0\nLWPOLYLINE\n90\n4\n70\n1\n10\n0\n20\n0\n10\n10\n20\n0\n10\n10\n20\n10\n10\n0\n20\n10\n";
  const res = parseDxf(dxf(square));
  check("closed LWPOLYLINE becomes one loop", res.loops.length === 1, `${res.loops.length} loops`);
  const loop = res.loops[0] ?? [];
  check("square keeps its 4 corners", loop.length === 4, `${loop.length} points`);
  const xs = loop.map((p) => p.x);
  const ys = loop.map((p) => p.y);
  check(
    "square extents preserved",
    Math.min(...xs) === 0 && Math.max(...xs) === 10 && Math.min(...ys) === 0 && Math.max(...ys) === 10,
    `x ${Math.min(...xs)}..${Math.max(...xs)}, y ${Math.min(...ys)}..${Math.max(...ys)}`
  );
  check("no skips reported", res.skippedPaths === 0 && res.skippedEntities === 0,
    `paths ${res.skippedPaths}, entities ${res.skippedEntities}`);
}

// --- bulge sampling ---------------------------------------------------------
{
  // A closed "D": bottom edge (0,0)→(10,0) is straight, top edge returns via a
  // semicircular bulge (bulge 1 on the (10,0) vertex).
  const dee = "0\nLWPOLYLINE\n90\n2\n70\n1\n10\n0\n20\n0\n10\n10\n20\n0\n42\n1\n";
  const res = parseDxf(dxf(dee));
  check("bulge polyline becomes one loop", res.loops.length === 1, `${res.loops.length} loops`);
  const loop = res.loops[0] ?? [];
  check("semicircle bulge is sampled", loop.length > 12, `${loop.length} points`);
  let worstR = 0;
  for (const p of loop) worstR = Math.max(worstR, Math.abs(dist(p, { x: 5, y: 0 }) - 5));
  check("all points sit on the arc's circle", worstR < 1e-9, `max radius error ${worstR.toExponential(2)}`);
  // Positive bulge runs CCW (y-up): the return arc from (10,0) to (0,0) passes above.
  const maxY = Math.max(...loop.map((p) => p.y));
  check("positive bulge arcs counter-clockwise", Math.abs(maxY - 5) < 1e-9, `apex y ${maxY.toFixed(4)}`);
}

// --- LINE chaining ----------------------------------------------------------
{
  // Triangle out of three LINEs, deliberately unordered and one reversed.
  const tri =
    "0\nLINE\n10\n0\n20\n0\n11\n10\n21\n0\n" + // (0,0)→(10,0)
    "0\nLINE\n10\n5\n20\n8\n11\n10\n21\n0\n" + // (5,8)→(10,0)  (reversed continuation)
    "0\nLINE\n10\n5\n20\n8\n11\n0\n21\n0\n"; // (5,8)→(0,0)
  const res = parseDxf(dxf(tri));
  check("three LINEs chain into one loop", res.loops.length === 1 && res.skippedPaths === 0,
    `${res.loops.length} loops, ${res.skippedPaths} skipped`);
  check("chained triangle has 3 points", (res.loops[0] ?? []).length === 3, `${res.loops[0]?.length} points`);
}

// --- ARC + LINE chaining ----------------------------------------------------
{
  // Upper semicircle (centre (5,0), r 5, 0°→180°: (10,0)→(0,0)) closed by a LINE.
  const half =
    "0\nARC\n10\n5\n20\n0\n40\n5\n50\n0\n51\n180\n" +
    "0\nLINE\n10\n0\n20\n0\n11\n10\n21\n0\n";
  const res = parseDxf(dxf(half));
  check("ARC + LINE chain into one loop", res.loops.length === 1, `${res.loops.length} loops`);
  const loop = res.loops[0] ?? [];
  const above = loop.filter((p) => p.y > 1).length;
  check("arc points sampled above the chord", above > 8, `${above} points with y > 1`);
  let worstR = 0;
  for (const p of loop) if (p.y > 1e-9) worstR = Math.max(worstR, Math.abs(dist(p, { x: 5, y: 0 }) - 5));
  check("arc samples sit on the circle", worstR < 1e-9, `max radius error ${worstR.toExponential(2)}`);
}

// --- CIRCLE -----------------------------------------------------------------
{
  const res = parseDxf(dxf("0\nCIRCLE\n10\n20\n20\n30\n40\n5\n"));
  const loop = res.loops[0] ?? [];
  check("CIRCLE becomes one sampled loop", res.loops.length === 1 && loop.length >= 24, `${loop.length} points`);
  let worstR = 0;
  for (const p of loop) worstR = Math.max(worstR, Math.abs(dist(p, { x: 20, y: 30 }) - 5));
  check("circle points at the radius", worstR < 1e-9, `max radius error ${worstR.toExponential(2)}`);
  check("circle loop is CCW (positive area)", loopSignedArea(loop) > 0, `area ${loopSignedArea(loop).toFixed(2)}`);
}

// --- old-style POLYLINE / VERTEX / SEQEND ------------------------------------
{
  const poly =
    "0\nPOLYLINE\n70\n1\n" +
    "0\nVERTEX\n10\n0\n20\n0\n0\nVERTEX\n10\n10\n20\n0\n0\nVERTEX\n10\n10\n20\n10\n0\nVERTEX\n10\n0\n20\n10\n" +
    "0\nSEQEND\n";
  const res = parseDxf(dxf(poly));
  check("closed POLYLINE/VERTEX becomes one loop", res.loops.length === 1 && res.loops[0].length === 4,
    `${res.loops.length} loops, ${res.loops[0]?.length} points`);
}

// --- open geometry + unsupported entities ------------------------------------
{
  const res = parseDxf(dxf(
    "0\nLINE\n10\n0\n20\n0\n11\n10\n21\n0\n" + // dangling line — cannot close
    "0\nTEXT\n10\n0\n20\n0\n1\nhello\n"
  ));
  check("dangling LINE is skipped, not looped", res.loops.length === 0 && res.skippedPaths === 1,
    `${res.loops.length} loops, ${res.skippedPaths} skipped paths`);
  check("unsupported entity counted", res.skippedEntities === 1, `${res.skippedEntities} entities`);
}

// --- $INSUNITS ---------------------------------------------------------------
{
  const square =
    "0\nLWPOLYLINE\n90\n4\n70\n1\n10\n0\n20\n0\n10\n10\n20\n0\n10\n10\n20\n10\n10\n0\n20\n10\n";
  check("$INSUNITS 4 (mm) read", parseDxf(dxf(square, insunits(4))).unitToMm === 1,
    `${parseDxf(dxf(square, insunits(4))).unitToMm}`);
  check("$INSUNITS 1 (inches) read", parseDxf(dxf(square, insunits(1))).unitToMm === 25.4,
    `${parseDxf(dxf(square, insunits(1))).unitToMm}`);
  check("$INSUNITS 0 (unitless) → null", parseDxf(dxf(square, insunits(0))).unitToMm === null,
    `${parseDxf(dxf(square, insunits(0))).unitToMm}`);
  check("no header → null", parseDxf(dxf(square)).unitToMm === null, `${parseDxf(dxf(square)).unitToMm}`);
}

// --- not a DXF ----------------------------------------------------------------
{
  let threw = false;
  try {
    parseDxf('{"version":12,"bodies":[]}');
  } catch {
    threw = true;
  }
  check("non-DXF input throws", threw, "JSON text rejected");
}

// --- working unit: serialize / load -------------------------------------------
{
  const scene = new Scene();
  scene.addBody([{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: 30 }]);
  scene.unit = "in";
  const data = JSON.parse(JSON.stringify(scene.serialize())) as SceneData;
  check("serialize writes v13 + unit", data.version === 13 && data.unit === "in",
    `version ${data.version}, unit ${data.unit}`);
  const loaded = new Scene();
  loaded.load(data);
  check("unit round-trips through load", loaded.unit === "in", `${loaded.unit}`);
  delete data.unit; // a pre-v12 file
  const legacy = new Scene();
  legacy.load(data);
  check("pre-v12 file defaults to mm", legacy.unit === "mm", `${legacy.unit}`);
}

// --- loop nesting (solids with holes) ------------------------------------------
const rect = (x0: number, y0: number, x1: number, y1: number): Vec2[] => [
  { x: x0, y: y0 }, { x: x1, y: y0 }, { x: x1, y: y1 }, { x: x0, y: y1 },
];
{
  // A plate with 4 cut-outs → one solid carrying 4 holes.
  const plate = rect(0, 0, 100, 100);
  const holes = [rect(10, 10, 25, 25), rect(75, 10, 90, 25), rect(10, 75, 25, 90), rect(75, 75, 90, 90)];
  const solids = nestLoops([holes[0], plate, holes[1], holes[2], holes[3]]); // order shouldn't matter
  check("plate + 4 cut-outs nest into one solid", solids.length === 1 && solids[0].holes.length === 4,
    `${solids.length} solids, ${solids[0]?.holes.length} holes`);
  check("the solid's outer is the plate", solids[0].outer === plate, "outer identity");
}
{
  // An island inside a hole starts a new solid (even-odd nesting).
  const solids = nestLoops([rect(0, 0, 100, 100), rect(20, 20, 80, 80), rect(40, 40, 60, 60)]);
  const withHole = solids.find((s) => s.holes.length === 1);
  const island = solids.find((s) => s.holes.length === 0);
  check("island inside a hole becomes its own solid", solids.length === 2 && !!withHole && !!island,
    `${solids.length} solids, holes ${solids.map((s) => s.holes.length).join("/")}`);
  check("island is the innermost loop", island?.outer[0].x === 40, `outer starts at x ${island?.outer[0].x}`);
}
{
  // Disjoint loops stay separate solids.
  const solids = nestLoops([rect(0, 0, 10, 10), rect(20, 0, 30, 10)]);
  check("disjoint loops stay separate", solids.length === 2 && solids.every((s) => s.holes.length === 0),
    `${solids.length} solids`);
}

// --- bodies with holes: mass properties, containment, transforms, persistence ---
{
  // 100×100 plate with a 20×20 hole centred at (70, 50):
  // area = 10000 − 400 = 9600; centroid.x = (10000·50 − 400·70)/9600 = 49.1666…
  const scene = new Scene();
  const body = scene.addBody(rect(0, 0, 100, 100), 0, "fillet", [rect(60, 40, 80, 60)]);
  check("hole subtracts from mass", Math.abs(1 / body.invMass - 9600) < 1e-6, `area ${(1 / body.invMass).toFixed(2)}`);
  check("composite centroid shifts away from the hole",
    Math.abs(body.pos.x - 472000 / 9600) < 1e-9 && Math.abs(body.pos.y - 50) < 1e-9,
    `pos (${body.pos.x.toFixed(4)}, ${body.pos.y.toFixed(4)})`);
  const solid = scene.addBody(rect(200, 0, 300, 100));
  check("hole reduces inertia vs the solid plate", body.invInertia > solid.invInertia,
    `inv ${body.invInertia.toExponential(3)} vs ${solid.invInertia.toExponential(3)}`);

  // Joints allowed anywhere inside the outer outline — including the hole's centre.
  check("point inside a hole still counts as inside the body", scene.pointInBody(body, { x: 70, y: 50 }),
    "pointInBody(70,50)");
  const j = scene.addJoint(body.id, { x: 70, y: 50 });
  check("joint placed at the hole centre stays put", dist(scene.jointWorld(j), { x: 70, y: 50 }) < 1e-9,
    `at (${scene.jointWorld(j).x}, ${scene.jointWorld(j).y})`);

  // Serialize / load round-trip keeps the holes (v13).
  const data = JSON.parse(JSON.stringify(scene.serialize())) as SceneData;
  const loaded = new Scene();
  loaded.load(data);
  const lb = loaded.getBody(body.id)!;
  check("holes round-trip through save/load", lb.holesLocal?.length === 1 && lb.holesLocal[0].length === 4,
    `${lb.holesLocal?.length} holes`);
  check("loaded hole geometry is exact",
    dist(polygonCentroid(loaded.bodyHolesWorld(lb)[0]), { x: 70, y: 50 }) < 1e-9,
    "hole centre (70,50)");
  check("solid body loads without a holes field", loaded.getBody(solid.id)!.holesLocal === undefined,
    `${loaded.getBody(solid.id)!.holesLocal}`);

  // Mirror: the hole reflects across the body centroid with the material.
  scene.mirrorBody(body.id, "h");
  const mc = polygonCentroid(scene.bodyHolesWorld(body)[0]);
  const expectX = 2 * (472000 / 9600) - 70;
  check("mirror carries the hole to the reflected side",
    Math.abs(mc.x - expectX) < 1e-6 && Math.abs(mc.y - 50) < 1e-6,
    `hole centre (${mc.x.toFixed(4)}, ${mc.y.toFixed(4)})`);
  check("mirror keeps the composite centroid", Math.abs(body.pos.x - 472000 / 9600) < 1e-6,
    `pos.x ${body.pos.x.toFixed(4)}`);

  // Copy/paste carries the holes (translation-invariant).
  const clip = scene.extractBody(body.id)!;
  const nid = scene.insertBody(clip, { x: 500, y: 500 })!;
  const nb = scene.getBody(nid)!;
  const nc = polygonCentroid(scene.bodyHolesWorld(nb)[0]);
  const rel = { x: nc.x - nb.pos.x, y: nc.y - nb.pos.y };
  const relSrc = { x: mc.x - body.pos.x, y: mc.y - body.pos.y };
  check("paste carries the hole at the same offset", dist(rel, relSrc) < 1e-6,
    `Δ (${rel.x.toFixed(3)}, ${rel.y.toFixed(3)}) vs (${relSrc.x.toFixed(3)}, ${relSrc.y.toFixed(3)})`);
  check("paste keeps the net mass", Math.abs(1 / nb.invMass - 9600) < 1e-6, `area ${(1 / nb.invMass).toFixed(2)}`);

  // Scale: the hole scales with the body about the centroid.
  scene.scaleBody(nid, 2);
  check("scale doubles the hole with the body", Math.abs(1 / nb.invMass - 4 * 9600) < 1e-6,
    `area ${(1 / nb.invMass).toFixed(2)}`);

  // Rotate: holes ride the body's frame.
  const before = polygonCentroid(scene.bodyHolesWorld(nb)[0]);
  scene.rotateBody(nid, nb.pos, Math.PI / 2);
  const after = polygonCentroid(scene.bodyHolesWorld(nb)[0]);
  const d0 = dist(before, nb.pos);
  check("rotate keeps the hole's centroid distance", Math.abs(dist(after, nb.pos) - d0) < 1e-6,
    `r ${d0.toFixed(3)} → ${dist(after, nb.pos).toFixed(3)}`);
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
