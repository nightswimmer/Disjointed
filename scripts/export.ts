/**
 * Headless checks of the cut-file exporter (src/export.ts): exact fillet arcs (convex +
 * reflex corners, per-corner radii), offset disks → circles, stadiums and uniform
 * offset hulls → arcs + tangents, holes, joint drill holes (attached joints of exported
 * bodies only, deduplicated), the y-flip / origin translation, `$INSUNITS`, layers, and
 * SVG millimetre scaling — plus a DXF round trip through the importer (src/dxf.ts),
 * which must reconstruct the fillets it reads back.
 */
import { collectCutSheet, sheetBounds, toDxf, toSvg, arcOf, CutLoop } from "../src/export";
import { parseDxf, nestLoops } from "../src/dxf";
import { Scene } from "../src/model";
import { Vec2, vec, dist, filletPolygon, closestPointOnPolygon } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail: string) {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}  (${detail})`);
  if (!ok) failures++;
}
const near = (a: number, b: number, eps = 1e-6) => Math.abs(a - b) <= eps;
const count = (text: string, entity: string) => text.split(`\n0\n${entity}\n`).length - 1;
const ringOf = (l: CutLoop) => (l.kind === "ring" ? l.verts : []);
/** Max distance of `pts` from the closed polyline `ref` (how far a sampled loop strays from a reference outline). */
function maxDeviation(pts: Vec2[], ref: Vec2[]): number {
  return Math.max(...pts.map((p) => dist(p, closestPointOnPolygon(p, ref))));
}

// --- rounded rectangle: exact fillets + DXF round trip ------------------------
{
  const scene = new Scene();
  const rect = [vec(10, 10), vec(50, 10), vec(50, 30), vec(10, 30)];
  const body = scene.addBody(rect, 5);
  const sheet = collectCutSheet(scene, [body]);
  const ring = ringOf(sheet.parts[0].outer);
  check("rounded rect → 8-vertex ring", ring.length === 8, `${ring.length} verts`);
  const bulged = ring.filter((v) => v.bulge !== 0);
  check("4 arcs, 4 straights", bulged.length === 4, `${bulged.length} bulged`);
  check("bulge = tan(90°/4)", bulged.every((v) => near(Math.abs(v.bulge), Math.tan(Math.PI / 8))), `${bulged.map((v) => v.bulge.toFixed(4))}`);
  // Each arc's centre must sit 5 in from two edges — inside the rectangle (sign check).
  const centres = ring.map((v, i) => (v.bulge ? arcOf(v.p, ring[(i + 1) % ring.length].p, v.bulge) : null)).filter((a) => a !== null);
  check(
    "arc centres inside, radius 5",
    centres.every((a) => a!.c.x > 10 && a!.c.x < 50 && a!.c.y > 10 && a!.c.y < 30 && near(a!.r, 5, 1e-6)),
    centres.map((a) => `(${a!.c.x.toFixed(2)},${a!.c.y.toFixed(2)}) r${a!.r.toFixed(3)}`).join(" ")
  );
  const b = sheetBounds(sheet);
  check("bounds = the rectangle", near(b.min.x, 10) && near(b.min.y, 10) && near(b.max.x, 50) && near(b.max.y, 30), `${b.min.x},${b.min.y}..${b.max.x},${b.max.y}`);

  const dxf = toDxf(sheet);
  check("DXF: one closed POLYLINE, 8 vertices", count(dxf, "POLYLINE") === 1 && count(dxf, "VERTEX") === 8, `${count(dxf, "POLYLINE")} polylines, ${count(dxf, "VERTEX")} vertices`);
  check("DXF: mm units header", dxf.includes("$INSUNITS\n70\n4\n"), "INSUNITS 4");
  check("DXF: CUT layer only (no joint holes)", dxf.includes("\n2\nCUT\n") && !dxf.includes("JOINTS"), "layers");
  const res = parseDxf(dxf);
  check("round trip: importer finds one loop", res.loops.length === 1, `${res.loops.length}`);
  const loop = res.loops[0];
  check("round trip: fillets reconstructed", !!loop?.fillet && loop.fillet.control.length === 4, `${loop?.fillet?.control.length} corners`);
  check("round trip: radii = 5", !!loop?.fillet && loop.fillet.radii.every((r) => near(r, 5, 1e-4)), `${loop?.fillet?.radii.map((r) => r.toFixed(4))}`);
  const xs = (loop?.fillet?.control ?? []).map((p) => p.x);
  const ys = (loop?.fillet?.control ?? []).map((p) => p.y);
  check(
    "round trip: translated to origin, 40×20",
    near(Math.min(...xs), 0, 1e-6) && near(Math.max(...xs), 40, 1e-6) && near(Math.min(...ys), 0, 1e-6) && near(Math.max(...ys), 20, 1e-6),
    `x ${Math.min(...xs)}..${Math.max(...xs)}, y ${Math.min(...ys)}..${Math.max(...ys)}`
  );
  // The sampled loop must hug the reference outline (flipped into DXF space).
  const ref = filletPolygon(rect, 5).map((p) => vec(p.x - 10, 30 - p.y));
  check("round trip: sampled loop hugs the fillet outline", maxDeviation(loop.pts, ref) < 0.02, `max dev ${maxDeviation(loop.pts, ref).toFixed(4)}`);

  const svg = toSvg(sheet);
  check("SVG: mm size + viewBox", svg.includes('width="40mm" height="20mm" viewBox="0 0 40 20"'), "40×20 mm");
  const arcs = (svg.match(/A[\d.]+ [\d.]+ 0 [01] [01] /g) ?? []).length;
  check("SVG: 4 arc commands, one path", arcs === 4 && (svg.match(/<path /g) ?? []).length === 1, `${arcs} arcs`);
  const d0 = svg.match(/ d="([^"]{0,40})/)?.[1] ?? "";
  check("SVG: y not flipped (path starts on the top edge, y=0)", /^M[\d.]+ 0[ LA]/.test(d0) || /^M0 [\d.]+/.test(d0), d0);
}

// --- sharp square: straight edges only ---------------------------------------
{
  const scene = new Scene();
  const body = scene.addBody([vec(0, 0), vec(10, 0), vec(10, 10), vec(0, 10)], 0);
  const sheet = collectCutSheet(scene, [body]);
  const ring = ringOf(sheet.parts[0].outer);
  check("sharp square → 4 straight verts", ring.length === 4 && ring.every((v) => v.bulge === 0), `${ring.length}`);
  const dxf = toDxf(sheet);
  check("DXF: no bulge codes", !dxf.includes("\n42\n"), "no 42 groups");
  const res = parseDxf(dxf);
  check("round trip: 4-point loop, no fillet", res.loops[0]?.pts.length === 4 && res.loops[0]?.fillet === null, `${res.loops[0]?.pts.length}`);
  const svg = toSvg(sheet);
  const d1 = svg.match(/ d="([^"]*)"/)?.[1] ?? "";
  check("SVG: M + 3 L + Z, no arcs", !d1.includes("A") && (d1.match(/L/g) ?? []).length === 3, d1);
}

// --- reflex corner fillet (L-shape) + per-corner radii -----------------------
{
  const scene = new Scene();
  const L = [vec(0, 0), vec(30, 0), vec(30, 10), vec(10, 10), vec(10, 30), vec(0, 30)];
  const body = scene.addBody(L, 4);
  const sheet = collectCutSheet(scene, [body]);
  const ring = ringOf(sheet.parts[0].outer);
  check("L-shape → 6 arcs (5 convex + 1 reflex)", ring.filter((v) => v.bulge !== 0).length === 6, `${ring.filter((v) => v.bulge !== 0).length}`);
  const res = parseDxf(toDxf(sheet));
  const ref = filletPolygon(L, 4).map((p) => vec(p.x, 30 - p.y));
  check("L round trip hugs the fillet outline (reflex arc turns the right way)", maxDeviation(res.loops[0].pts, ref) < 0.02, `max dev ${maxDeviation(res.loops[0].pts, ref).toFixed(4)}`);
  check("L round trip: 6 fillets reconstructed", res.loops[0].fillet?.control.length === 6, `${res.loops[0].fillet?.control.length}`);

  // Only corner 0 rounded (per-corner override), default radius 0.
  const body2 = scene.addBody([vec(50, 0), vec(70, 0), vec(70, 20), vec(50, 20)], 0, "fillet", undefined, [6, null, null, null]);
  const ring2 = ringOf(collectCutSheet(scene, [body2]).parts[0].outer);
  check("one overridden corner → 5 verts, 1 arc", ring2.length === 5 && ring2.filter((v) => v.bulge !== 0).length === 1, `${ring2.length} verts`);
  const res2 = parseDxf(toDxf(collectCutSheet(scene, [body2])));
  const radii = res2.loops[0].fillet?.radii ?? [];
  check("per-corner round trip: radii {6,0,0,0}", radii.length === 4 && radii.filter((r) => near(r, 6, 1e-4)).length === 1 && radii.filter((r) => r === 0).length === 3, `${radii.map((r) => r.toFixed(3))}`);
}

// --- offset bodies: disk, stadium, hull -------------------------------------
{
  const scene = new Scene();
  const disk = scene.addBody([vec(100, 100)], 10, "offset");
  const sheet = collectCutSheet(scene, [disk]);
  check("disk → circle loop r=10", sheet.parts[0].outer.kind === "circle" && near((sheet.parts[0].outer as { r: number }).r, 10), `${sheet.parts[0].outer.kind}`);
  const dxf = toDxf(sheet);
  check("DXF: CIRCLE entity", count(dxf, "CIRCLE") === 1 && count(dxf, "POLYLINE") === 0, `${count(dxf, "CIRCLE")} circles`);
  const res = parseDxf(dxf);
  check("round trip: circle r=10 centred at (10,10)", !!res.loops[0]?.circle && near(res.loops[0].circle.r, 10) && near(res.loops[0].circle.c.x, 10) && near(res.loops[0].circle.c.y, 10), `${JSON.stringify(res.loops[0]?.circle)}`);
  check("SVG: <circle> 20×20 mm", toSvg(sheet).includes('<circle cx="10" cy="10" r="10"') && toSvg(sheet).includes('width="20mm"'), "circle");

  const stadium = scene.addBody([vec(0, 0), vec(30, 0)], 5, "offset");
  const s2 = collectCutSheet(scene, [stadium]);
  const ring = ringOf(s2.parts[0].outer);
  check("stadium → 4 verts, two half-circle bulges (|b|=1)", ring.length === 4 && ring.filter((v) => near(Math.abs(v.bulge), 1)).length === 2, `${ring.map((v) => v.bulge.toFixed(3))}`);
  const b2 = sheetBounds(s2);
  check("stadium bounds 40×10", near(b2.max.x - b2.min.x, 40) && near(b2.max.y - b2.min.y, 10), `${(b2.max.x - b2.min.x).toFixed(3)}×${(b2.max.y - b2.min.y).toFixed(3)}`);
  const r2 = parseDxf(toDxf(s2));
  const seg = [vec(5, 5), vec(35, 5)]; // the stadium's spine in DXF space (translated to origin)
  const devs = r2.loops[0].pts.map((p) => Math.abs(dist(p, closestPointOnPolygon(p, [seg[0], seg[1], seg[1]])) - 5));
  check("stadium round trip: every point 5 from the spine", Math.max(...devs) < 0.02, `max ${Math.max(...devs).toFixed(4)}`);

  const tri = [vec(0, 0), vec(40, 0), vec(20, 30)];
  const hull = scene.addBody(tri, 6, "offset");
  const s3 = collectCutSheet(scene, [hull]);
  const ring3 = ringOf(s3.parts[0].outer);
  check("offset triangle → 6 verts, 3 arcs", ring3.length === 6 && ring3.filter((v) => v.bulge !== 0).length === 3, `${ring3.length} verts`);
  const r3 = parseDxf(toDxf(s3));
  const b3 = sheetBounds(s3);
  const triDxf = tri.map((p) => vec(p.x - b3.min.x, b3.max.y - p.y));
  const devs3 = r3.loops[0].pts.map((p) => Math.abs(dist(p, closestPointOnPolygon(p, triDxf)) - 6));
  check("offset hull round trip: every point 6 from the hull", Math.max(...devs3) < 0.02, `max ${Math.max(...devs3).toFixed(4)}`);
  check("offset hull round trip: arcs reconstructed as fillets", r3.loops[0].fillet?.control.length === 3, `${r3.loops[0].fillet?.control.length} corners`);

  // Mixed margins: falls back to the sampled outline (still a closed ring).
  const mixed = scene.addBody(tri, 6, "offset", undefined, [6, 2, null]);
  const ring4 = ringOf(collectCutSheet(scene, [mixed]).parts[0].outer);
  check("mixed-margin hull falls back to a sampled ring", ring4.length > 20 && ring4.every((v) => v.bulge === 0), `${ring4.length} verts`);
}

// --- holes, joint holes, selection scope -----------------------------------
{
  const scene = new Scene();
  const plate = scene.addBody(
    [vec(0, 0), vec(60, 0), vec(60, 40), vec(0, 40)],
    3,
    "fillet",
    [{ control: [vec(30, 20)], radius: 5, round: "offset" }, [vec(5, 5), vec(15, 5), vec(15, 15), vec(5, 15)]]
  );
  const other = scene.addBody([vec(100, 0), vec(120, 0), vec(120, 20), vec(100, 20)], 0);
  scene.addJoint(plate.id, vec(50, 10));
  scene.addJoint(plate.id, vec(50, 30));
  scene.addJoint(plate.id, vec(50, 30)); // duplicate spot → one hole
  scene.addJoint(other.id, vec(110, 10)); // not exported
  scene.addFreeJoint(vec(80, 80)); // free joint → never a hole

  const plain = collectCutSheet(scene, [plate]);
  check("plate: 2 holes (disk + square)", plain.parts[0].holes.length === 2 && plain.parts[0].holes[0].kind === "circle" && ringOf(plain.parts[0].holes[1]).length === 4, `${plain.parts[0].holes.map((h) => h.kind)}`);
  check("no diameter → no joint holes", plain.joints.length === 0, `${plain.joints.length}`);

  const sheet = collectCutSheet(scene, [plate], { jointHoleDiameter: 4 });
  check("joint holes: 2 (dedup, other body + free joint excluded)", sheet.joints.length === 2 && sheet.joints.every((j) => j.kind === "circle" && near(j.r, 2)), `${sheet.joints.length}`);
  const dxf = toDxf(sheet);
  check("DXF: 2 POLYLINE + 3 CIRCLE", count(dxf, "POLYLINE") === 2 && count(dxf, "CIRCLE") === 3, `${count(dxf, "POLYLINE")} / ${count(dxf, "CIRCLE")}`);
  check("DXF: JOINTS layer declared + used", (dxf.match(/\n8\nJOINTS\n/g) ?? []).length === 2 && dxf.includes("\n2\nJOINTS\n"), "layers");
  const res = parseDxf(dxf);
  const solids = nestLoops(res.loops.map((l) => l.pts));
  check("round trip: importer nests into 1 solid with 4 holes", solids.length === 1 && solids[0].holes.length === 4, `${solids.length} solids, ${solids[0]?.holes.length} holes`);
  const svg = toSvg(sheet);
  check("SVG: body group with path + circle, joints group with 2 circles", svg.includes('<g id="joints">') && (svg.match(/<circle /g) ?? []).length === 3 && svg.includes('fill-rule="evenodd"'), `${(svg.match(/<circle /g) ?? []).length} circles`);
  const evenOdd = svg.match(/<path fill-rule="evenodd" d="([^"]*)"/)?.[1] ?? "";
  check("SVG: outer + square hole as two subpaths", (evenOdd.match(/M/g) ?? []).length === 2 && (evenOdd.match(/Z/g) ?? []).length === 2, `${(evenOdd.match(/M/g) ?? []).length} subpaths`);

  const both = collectCutSheet(scene, [plate, other], { jointHoleDiameter: 4 });
  check("two bodies → 2 parts, 3 joint holes", both.parts.length === 2 && both.joints.length === 3, `${both.parts.length} parts, ${both.joints.length} joints`);
}

// --- units -----------------------------------------------------------------
{
  const scene = new Scene();
  scene.unit = "in";
  const body = scene.addBody([vec(0, 0), vec(2, 0), vec(2, 1), vec(0, 1)], 0);
  const sheet = collectCutSheet(scene, [body]);
  const dxf = toDxf(sheet);
  check("DXF inches: INSUNITS 1, coordinates unscaled", dxf.includes("$INSUNITS\n70\n1\n") && dxf.includes("\n10\n2\n20\n1\n"), "2×1 in");
  const res = parseDxf(dxf);
  check("importer reads back 25.4 mm/unit", res.unitToMm === 25.4, `${res.unitToMm}`);
  const svg = toSvg(sheet);
  check("SVG inches → 50.8 × 25.4 mm", svg.includes('width="50.8mm" height="25.4mm" viewBox="0 0 50.8 25.4"'), svg.match(/<svg[^>]*>/)?.[0] ?? "");
  scene.unit = "cm";
  check("DXF cm: INSUNITS 5", toDxf(collectCutSheet(scene, [body])).includes("$INSUNITS\n70\n5\n"), "cm");
}

// --- posed body: export follows the current pose ---------------------------
{
  const scene = new Scene();
  const body = scene.addBody([vec(0, 0), vec(20, 0), vec(20, 10), vec(0, 10)], 0);
  body.angle = Math.PI / 2;
  const b = sheetBounds(collectCutSheet(scene, [body]));
  check("rotated body exports rotated (10×20 box)", near(b.max.x - b.min.x, 10, 1e-9) && near(b.max.y - b.min.y, 20, 1e-9), `${(b.max.x - b.min.x).toFixed(3)}×${(b.max.y - b.min.y).toFixed(3)}`);
}

console.log(failures === 0 ? "\nAll export checks passed." : `\n${failures} export check(s) FAILED.`);
if (failures > 0) process.exit(1);
